#!/usr/bin/env python3
"""手机上的导航通信基准 (在 Termux 里运行): python3 ~/bench_nav.py [实例=i01] [圈数=1]
同时做三件事并汇总成一段文字:
  1. 四工位循环 (每圈 4 个 Nav2 任务): 到达数、用时、停车误差
  2. DDS 往返延迟探针 (tools/dds_probe.py，在独立 proot 里，沿用实例的 ROS 环境变量)
  3. CPU 占用 (单核百分比): Nav2 各节点、proot 追踪进程、仿真、执行进程；以及日志里的应答超时 / 行为树超频次数"""
import glob, json, math, os, re, subprocess, sys, time, urllib.request

INST = sys.argv[1] if len(sys.argv) > 1 else "i01"
ROUNDS = int(sys.argv[2]) if len(sys.argv) > 2 else 1
PREFIX = os.environ.get("PREFIX", "/data/data/com.termux/files/usr")
ROOTFS = f"{PREFIX}/var/lib/proot-distro/containers/{os.environ.get('AGV_DISTRO', 'ubuntu')}/rootfs"
LOG = f"{ROOTFS}/root/.agv-agent/logs/agv-nav-{INST}.log"
HZ = os.sysconf("SC_CLK_TCK")


def call(m, u, b=None, t=15):
    r = urllib.request.Request(u, data=None if b is None else json.dumps(b).encode(), method=m, headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=t).read() or b"{}")


def procs():
    out = {}
    for d in glob.glob("/proc/[0-9]*"):
        try:
            cmd = open(d + "/cmdline", "rb").read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
            st = open(d + "/stat").read().rsplit(")", 1)[1].split()
            out[int(d[6:])] = (cmd, int(st[11]) + int(st[12]), int(st[1]))      # utime+stime, ppid
        except Exception:
            pass
    return out


def group(cmd, pid, all_p):
    m = re.match(r"/opt/ros/humble/lib/[^/]+/(\S+)", cmd)
    if m:
        return m.group(1)
    if "agv_ros_bridge/lib" in cmd: return "agv_ros_bridge"
    if "sim_server.api" in cmd: return "sim_server"
    if "nav_runtime.main" in cmd: return "nav_runtime"
    if "web_gateway.py" in cmd and "spawn.py" not in cmd: return "web_gateway"
    if "fast-discovery-server" in cmd: return "discovery_server"
    if "ros2 launch" in cmd: return "ros2_launch"
    if cmd.startswith(PREFIX + "/bin/proot") or "/proot " in cmd[:80]: return "proot(追踪)"
    return None


def find_env():
    for pid, (cmd, _, _) in procs().items():
        if "nav_runtime.main" in cmd and "python" in cmd.split(" ")[0]:
            try:
                env = dict(x.split("=", 1) for x in open(f"/proc/{pid}/environ", "rb").read().decode("utf-8", "replace").split("\0") if "=" in x)
            except Exception:
                continue
            if env.get("INSTANCE_ID", INST) == INST or "INSTANCE_ID" not in env:
                return env
    return {}


env = find_env()
keep = {k: v for k, v in env.items() if k.startswith(("ROS_", "FASTRTPS_", "RMW_", "CYCLONEDDS")) }
nav_port = env.get("NAV_API_PORT", "8102")
sim_api = env.get("SIM_API", "http://127.0.0.1:8100")
NAV = f"http://127.0.0.1:{nav_port}"
print(f"实例 {INST}: 执行进程 :{nav_port}  ROS 环境 {keep}")
print("cpuset", open("/proc/self/cpuset").read().strip(), " 大核频率 MHz",
      [int(open(f"/sys/devices/system/cpu/cpu{c}/cpufreq/scaling_cur_freq").read()) // 1000 for c in (0, 3, 7)])

goals = [(5, 0, 0), (0, 5, 1.5708), (-5, 0, 3.1416), (0, -5, -1.5708)] * ROUNDS
dur = 36 * len(goals)
exports = " ".join(f"export {k}='{v}';" for k, v in keep.items())
probe = subprocess.Popen(["proot-distro", "login", os.environ.get("AGV_DISTRO", "ubuntu"), "--", "bash", "-c",
                          f". /opt/ros/humble/setup.bash; {exports} python3 /opt/agv/tools/dds_probe.py {dur} 10"],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=os.path.expanduser("~"))
log0 = os.path.getsize(LOG) if os.path.exists(LOG) else 0
call("PUT", NAV + "/api/v1/nav/planner", {"type": "nav2"})
time.sleep(12)                                    # 探针启动与发现
p0, t0 = procs(), time.time()
res = []
for gx, gy, gyaw in goals:
    call("POST", NAV + "/api/v1/missions", {"x": gx, "y": gy, "yaw": gyaw})
    ts = time.time(); st = "?"
    while time.time() - ts < 150:
        time.sleep(1.0)
        st = call("GET", NAV + "/api/v1/nav")["status"]
        if st in ("ARRIVED", "FAILED", "ABORTED", "CANCELED", "IDLE") and time.time() - ts > 3: break
    tr = call("GET", sim_api + "/api/v1/snapshot")["state"]["truth"]
    res.append((st, time.time() - ts, math.hypot(tr["x"] - gx, tr["y"] - gy) * 1000))
    time.sleep(1.5)
p1, t1 = procs(), time.time()
cpu = {}
for pid, (cmd, ticks, _) in p1.items():
    g = group(cmd, pid, p1)
    if g and pid in p0:
        cpu[g] = cpu.get(g, 0.0) + (ticks - p0[pid][1]) / HZ / (t1 - t0) * 100
n_proc = {}
for pid, (cmd, _, _) in p1.items():
    g = group(cmd, pid, p1)
    if g: n_proc[g] = n_proc.get(g, 0) + 1
print(f"任务: {sum(r[0] == 'ARRIVED' for r in res)}/{len(res)} 到达, 用时 {[round(r[1]) for r in res]} s, 停车误差 {[round(r[2]) for r in res]} mm")
print("CPU (单核 %): " + ", ".join(f"{k}×{n_proc.get(k, 0)} {v:.0f}" for k, v in sorted(cpu.items(), key=lambda kv: -kv[1])) + f"  | 合计 {sum(cpu.values()):.0f}")
new = open(LOG, "rb").read()[log0:].decode("utf-8", "replace") if os.path.exists(LOG) else ""
print(f"日志: 应答超时 {new.count('Timed out while waiting for action server')} 次, 行为树超频 {new.count('tick rate')} 次, "
      f"TF 外推/过旧 {new.count('extrapolation') + new.count('too old')} 次")
try:
    out, _ = probe.communicate(timeout=90)
except subprocess.TimeoutExpired:
    probe.kill(); out = ""
m = re.search(r"PROBE (\{.*\})", out or "")
if m:
    d = json.loads(m.group(1))
    print(f"探针: 发现全部服务用了 {d['discovery_s']} s")
    for s, v in d["servers"].items():
        print(f"  {s:18s} " + (f"n={v['n']} 中位 {v['p50']} p90 {v['p90']} p99 {v['p99']} 最大 {v['max']} ms | >20ms {v['gt20']} >200ms {v['gt200']} >1s {v['gt1000']} 无应答 {v['lost']}" if v.get("n") else f"无数据 {v}"))
else:
    print("探针无输出:", (out or "")[-400:])
s = call("GET", sim_api + "/api/v1/sim"); print("仿真 rtf", s.get("rtf"), "碰撞", s.get("collisions"))
