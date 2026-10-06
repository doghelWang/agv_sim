#!/usr/bin/env python3
"""手机上的导航性能时序采样 (在 Termux 里运行): python3 ~/nav_profile.py [标签=run] [圈数=2] [实例=i01]
一边跑四工位任务，一边每秒采样一次，写 ~/prof_<标签>.csv 并打印汇总:
  - 各进程组的 CPU (单核 %) 和 **等待 CPU 的时间** (ms/s，/proc/<pid>/schedstat 的 run_delay —— 排队等核的时间，是"核不够用"的直接证据)
  - 车辆阶段 (直行 / 原地转向 / 进站 / 静止)、定位 TF 龄、各簇当前频率、可用的核
  - 结束后: 各组 均值/95 分位/最大值，按阶段分组的总需求，需求最高的 5 个时刻在干什么，日志里的告警计数"""
import glob, json, math, os, re, sys, time, urllib.request

TAG = sys.argv[1] if len(sys.argv) > 1 else "run"
ROUNDS = int(sys.argv[2]) if len(sys.argv) > 2 else 2
INST = sys.argv[3] if len(sys.argv) > 3 else "i01"
PREFIX = os.environ.get("PREFIX", "/data/data/com.termux/files/usr")
ROOTFS = f"{PREFIX}/var/lib/proot-distro/containers/{os.environ.get('AGV_DISTRO', 'ubuntu')}/rootfs"
LOG = f"{ROOTFS}/root/.agv-agent/logs/agv-nav-{INST}.log"
HZ = os.sysconf("SC_CLK_TCK")
NAV2 = ("controller_server", "bt_navigator", "planner_server", "behavior_server", "velocity_smoother", "lifecycle_manager")


def call(m, u, b=None, t=10):
    r = urllib.request.Request(u, data=None if b is None else json.dumps(b).encode(), method=m, headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=t).read() or b"{}")


def group(cmd):
    m = re.match(r"/opt/ros/humble/lib/[^/]+/(\S+)", cmd)
    if m:
        n = m.group(1)
        return "slam" if "slam" in n else n
    if "agv_ros_bridge/lib" in cmd: return "bridge"
    if "sim_server.api" in cmd: return "sim"
    if "nav_runtime.main" in cmd: return "nav_runtime"
    if "web_gateway.py" in cmd and "spawn" not in cmd: return "gateway"
    if "fast-discovery-server" in cmd: return "dds_server"
    if cmd.startswith(PREFIX + "/bin/proot"): return "proot"
    if "agent.server" in cmd or "hub.server" in cmd: return "platform"
    return None


def sample():
    out = {}
    for d in glob.glob("/proc/[0-9]*"):
        try:
            cmd = open(d + "/cmdline", "rb").read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
            g = group(cmd)
            if not g:
                continue
            st = open(d + "/stat").read().rsplit(")", 1)[1].split()
            ss = open(d + "/schedstat").read().split()
            # 线程级 run_delay: 进程的 schedstat 只是主线程，逐线程累加
            delay = 0
            for t in glob.glob(d + "/task/*/schedstat"):
                try:
                    delay += int(open(t).read().split()[1])
                except Exception:
                    pass
            a = out.setdefault(g, [0, 0])
            a[0] += int(st[11]) + int(st[12]); a[1] += delay
        except Exception:
            pass
    return out


def freq(c):
    try:
        return int(open(f"/sys/devices/system/cpu/cpu{c}/cpufreq/scaling_cur_freq").read()) // 1000
    except Exception:
        return 0


env = {}
for d in glob.glob("/proc/[0-9]*"):
    try:
        if b"nav_runtime.main" in open(d + "/cmdline", "rb").read():
            env = dict(x.split("=", 1) for x in open(d + "/environ", "rb").read().decode("utf-8", "replace").split("\0") if "=" in x)
            break
    except Exception:
        pass
NAV = f"http://127.0.0.1:{env.get('NAV_API_PORT', '8102')}"
SIM = env.get("SIM_API", "http://127.0.0.1:8100")
cpuset0 = open("/proc/self/cpuset").read().strip()
allowed = [l.split()[1] for l in open("/proc/self/status") if l.startswith("Cpus_allowed_list")][0]
ncore = len(os.sched_getaffinity(0))
print(f"[{TAG}] cpuset {cpuset0} 可用核 {allowed} ({ncore} 个)")
call("PUT", NAV + "/api/v1/nav/planner", {"type": "nav2"})
goals = [(5, 0, 0), (0, 5, 1.5708), (-5, 0, 3.1416), (0, -5, -1.5708)] * ROUNDS
log0 = os.path.getsize(LOG) if os.path.exists(LOG) else 0
rows, results = [], []
prev, tprev = sample(), time.time()
names = set(prev)
for gi, (gx, gy, gyaw) in enumerate(goals):
    call("POST", NAV + "/api/v1/missions", {"x": gx, "y": gy, "yaw": gyaw})
    ts = time.time(); st = "?"
    while time.time() - ts < 150:
        time.sleep(max(0.0, 1.0 - (time.time() - tprev)))
        cur, tnow = sample(), time.time()
        dt = tnow - tprev
        try:
            n = call("GET", NAV + "/api/v1/nav"); st = n["status"]
            tr = call("GET", SIM + "/api/v1/snapshot")["state"]["truth"]
            sl = call("GET", NAV + "/api/v1/slam")
        except Exception:
            n, tr, sl = {}, {"x": 0, "y": 0, "yaw": 0, "vx": 0, "vy": 0, "wz": 0}, {}
        v, w = math.hypot(tr.get("vx", 0), tr.get("vy", 0)), abs(tr.get("wz", 0))
        dist = math.hypot(tr["x"] - gx, tr["y"] - gy)
        phase = "进站" if (v > 0.02 and dist < 0.6) else "直行" if v > 0.05 else "转向" if w > 0.1 else "静止"
        row = {"t": round(tnow - ts, 1), "task": gi + 1, "phase": phase, "status": st, "v": round(v, 2), "w": round(w, 2),
               "tf_age": (sl.get("ext") or {}).get("tf_age_s") or 0, "cpuset": open("/proc/self/cpuset").read().strip(),
               "f_small": freq(0), "f_big": freq(3), "f_prime": freq(7)}
        for g in set(cur) | names:
            a, b = cur.get(g, [0, 0]), prev.get(g, [0, 0])
            row["cpu_" + g] = round(max(0, a[0] - b[0]) / HZ / dt * 100, 1)
            row["wait_" + g] = round(max(0, a[1] - b[1]) / 1e6 / dt, 1)        # ms / s
        names |= set(cur)
        rows.append(row); prev, tprev = cur, tnow
        if st in ("ARRIVED", "FAILED", "ABORTED", "CANCELED", "IDLE") and time.time() - ts > 3:
            break
    tr = call("GET", SIM + "/api/v1/snapshot")["state"]["truth"]
    results.append((st, time.time() - ts, math.hypot(tr["x"] - gx, tr["y"] - gy) * 1000))
names = sorted(names)
cols = ["task", "t", "phase", "status", "v", "w", "tf_age", "cpuset", "f_small", "f_big", "f_prime"] + ["cpu_" + g for g in names] + ["wait_" + g for g in names]
out = os.path.expanduser(f"~/prof_{TAG}.csv")
with open(out, "w") as f:
    f.write(",".join(cols) + "\n")
    for r in rows:
        f.write(",".join(str(r.get(c, 0)) for c in cols) + "\n")


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(len(v) * p))] if v else 0


tot = [sum(r.get("cpu_" + g, 0) for g in names) for r in rows]
nav2 = [sum(r.get("cpu_" + g, 0) for g in NAV2) for r in rows]
wait_nav2 = [sum(r.get("wait_" + g, 0) for g in NAV2) for r in rows]
print(f"任务: {sum(r[0] == 'ARRIVED' for r in results)}/{len(results)} 到达, 用时 {[round(r[1]) for r in results]} s, 停车误差 {[round(r[2]) for r in results]} mm")
print(f"采样 {len(rows)} 次; 小核频率 均值 {sum(r['f_small'] for r in rows) // max(1, len(rows))} MHz; cpuset 变化: {sorted(set(r['cpuset'] for r in rows))}")
print(f"总需求 (单核 %): 均值 {sum(tot) / len(tot):.0f}  p95 {q(tot, .95):.0f}  最大 {max(tot):.0f}   | 可用 {ncore * 100}  → 占用率 均值 {sum(tot) / len(tot) / ncore:.0f}%  峰值 {max(tot) / ncore:.0f}%")
print(f"{'进程组':18s} {'CPU均值':>7s} {'p95':>6s} {'最大':>6s} | {'等核 ms/s 均值':>12s} {'p95':>6s} {'最大':>6s}")
for g in sorted(names, key=lambda g: -sum(r.get('cpu_' + g, 0) for r in rows)):
    c = [r.get("cpu_" + g, 0) for r in rows]; w = [r.get("wait_" + g, 0) for r in rows]
    print(f"{g:18s} {sum(c) / len(c):7.1f} {q(c, .95):6.1f} {max(c):6.1f} | {sum(w) / len(w):12.1f} {q(w, .95):6.1f} {max(w):6.1f}")
print(f"Nav2 六个节点合计: CPU 均值 {sum(nav2) / len(nav2):.1f} p95 {q(nav2, .95):.1f} 最大 {max(nav2):.1f}; 等核 均值 {sum(wait_nav2) / len(wait_nav2):.0f} p95 {q(wait_nav2, .95):.0f} 最大 {max(wait_nav2):.0f} ms/s")
print("按阶段: 阶段  采样数  总需求均值/最大   Nav2 均值/最大   Nav2 等核均值/最大 (ms/s)   TF 龄最大")
for ph in ("直行", "转向", "进站", "静止"):
    idx = [i for i, r in enumerate(rows) if r["phase"] == ph]
    if idx:
        print(f"   {ph}  {len(idx):4d}   {sum(tot[i] for i in idx) / len(idx):5.0f} / {max(tot[i] for i in idx):5.0f}      {sum(nav2[i] for i in idx) / len(idx):5.1f} / {max(nav2[i] for i in idx):5.1f}      "
              f"{sum(wait_nav2[i] for i in idx) / len(idx):6.0f} / {max(wait_nav2[i] for i in idx):6.0f}        {max(rows[i]['tf_age'] for i in idx):.2f}")
print("需求最高的 5 个时刻:")
for i in sorted(range(len(rows)), key=lambda i: -tot[i])[:5]:
    r = rows[i]; top = sorted(names, key=lambda g: -r.get("cpu_" + g, 0))[:4]
    print(f"   任务 {r['task']} 第 {r['t']:.0f} s {r['phase']}: 总 {tot[i]:.0f}  " + ", ".join(f"{g} {r.get('cpu_' + g, 0):.0f}" for g in top) + f"  | Nav2 等核 {wait_nav2[i]:.0f} ms/s  TF 龄 {r['tf_age']:.2f}")
print("Nav2 等核最久的 5 个时刻:")
for i in sorted(range(len(rows)), key=lambda i: -wait_nav2[i])[:5]:
    r = rows[i]
    print(f"   任务 {r['task']} 第 {r['t']:.0f} s {r['phase']}: Nav2 等核 {wait_nav2[i]:.0f} ms/s (CPU {nav2[i]:.0f})  总需求 {tot[i]:.0f}  slam {r.get('cpu_slam', 0):.0f} sim {r.get('cpu_sim', 0):.0f}  TF 龄 {r['tf_age']:.2f}")
new = open(LOG, "rb").read()[log0:].decode("utf-8", "replace") if os.path.exists(LOG) else ""
print(f"日志: 控制环掉拍 {new.count('Control loop missed')} 次, 应答超时 {new.count('Timed out while waiting for action server')} 次, 行为树超频 {new.count('tick rate')} 次, "
      f"TF 外推 {new.count('extrapolation')} 次, 中断重试 {new.count('Controller patience exceeded')} 次")
s = call("GET", SIM + "/api/v1/sim"); print("仿真 rtf", s.get("rtf"), "overruns", s.get("overruns"), "碰撞", s.get("collisions"), "| 数据:", out)
