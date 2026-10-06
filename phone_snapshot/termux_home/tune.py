#!/usr/bin/env python3
"""对实例进程做调度调整 (Termux 里运行，不需要 root): python3 tune.py show | nice | pin <核> | reset
  nice : 非关键进程 (网页网关、节点代理/资源平台、robot_state_publisher、发现服务器) 连同各自的 proot 追踪进程调到 nice 10
  pin N: Nav2 的 controller/bt_navigator/planner/behavior/velocity_smoother (含追踪进程) 绑到核 N，其余进程绑到其它可用核
  reset: 取消绑核 (恢复为当前 cpuset 的全部核)；nice 降回去需要特权，做不到 (重启实例即恢复)"""
import glob, os, re, sys
PREFIX = os.environ.get("PREFIX", "/data/data/com.termux/files/usr")
NAV2 = ("controller_server", "bt_navigator", "planner_server", "behavior_server", "velocity_smoother")
LOW = ("gateway", "platform", "robot_state_publisher", "dds_server")


def group(cmd):
    m = re.match(r"/opt/ros/humble/lib/[^/]+/(\S+)", cmd)
    if m: return "slam" if "slam" in m.group(1) else m.group(1)
    if "agv_ros_bridge/lib" in cmd: return "bridge"
    if "sim_server.api" in cmd: return "sim"
    if "nav_runtime.main" in cmd: return "nav_runtime"
    if "web_gateway.py" in cmd: return "gateway"
    if "fast-discovery-server" in cmd: return "dds_server"
    if "agent.server" in cmd or "hub.server" in cmd: return "platform"
    return None


procs = {}
for d in glob.glob("/proc/[0-9]*"):
    try:
        pid = int(d[6:]); cmd = open(d + "/cmdline", "rb").read().replace(b"\0", b" ").decode("utf-8", "replace")
        g = group(cmd)
        if not g: continue
        tr = int([l.split()[1] for l in open(d + "/status") if l.startswith("TracerPid")][0])
        procs.setdefault(g, set()).add(pid)
        if tr: procs[g].add(tr)
    except Exception:
        pass


def tids(pid):
    return [int(os.path.basename(t)) for t in glob.glob(f"/proc/{pid}/task/*")]


act = sys.argv[1] if len(sys.argv) > 1 else "show"
allowed = sorted(os.sched_getaffinity(0))
if act == "boost":
    n = 0
    for g, v in (("controller_server", -20), ("bt_navigator", -20), ("planner_server", -20), ("behavior_server", -20), ("velocity_smoother", -20), ("lifecycle_manager", -20), ("bridge", -18), ("nav_runtime", -18)):
        for pid in procs.get(g, ()):
            for t in tids(pid):
                try: os.setpriority(os.PRIO_PROCESS, t, v); n += 1
                except OSError as e: print("失败", g, e); break
    print(f"已提高 Nav2/桥接/执行进程 {n} 个线程的优先级 (nice -20 / -18)")
elif act == "nice":
    n = 0
    for g in LOW:
        for pid in procs.get(g, ()):
            for t in tids(pid):
                try: os.setpriority(os.PRIO_PROCESS, t, 10); n += 1
                except OSError: pass
    print(f"已把 {LOW} 的 {n} 个线程调到 nice 10")
elif act in ("pin", "reset"):
    core = int(sys.argv[2]) if act == "pin" else None
    rest = set(c for c in allowed if c != core) or set(allowed)
    n = bad = 0
    for g, pids in procs.items():
        mask = set(allowed) if act == "reset" else ({core} if g in NAV2 else rest)
        for pid in pids:
            for t in tids(pid):
                try: os.sched_setaffinity(t, mask); n += 1
                except OSError: bad += 1
    print(f"{act}: 设置 {n} 个线程" + (f"，失败 {bad}" if bad else "") + (f" (Nav2 → 核 {core}，其余 → {sorted(rest)})" if act == "pin" else ""))
for g in sorted(procs):
    info = []
    for pid in sorted(procs[g]):
        try:
            st = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
            aff = sorted(os.sched_getaffinity(pid))
            info.append(f"{pid}(nice {st[16]}, 核 {aff[0]}-{aff[-1]}" + ("" if len(aff) == aff[-1] - aff[0] + 1 else f"×{len(aff)}") + ")")
        except Exception:
            pass
    print(f"  {g:22s} " + " ".join(info))
