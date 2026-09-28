#!/usr/bin/env python3
"""
运行性能采样 (只用标准库；树莓派宿主机 / 手机 Termux 原生 Python / 容器内都能跑)

  python3 tools/perf_sample.py --secs 180 --out perf_pi.json \
      --sim http://127.0.0.1:8100 --nav http://127.0.0.1:8100

每 --interval 秒采一次:
  * 本机进程 CPU% (按命令行关键字归类: sim_server / web_gateway / nav_runtime / slam_toolbox / ekf / Nav2 各服务器 / proot …)
    与常驻内存 (RSS；能读 smaps_rollup 时另记 PSS: 共享库按使用进程数分摊，ROS 节点之间共享库多，RSS 相加会重复计算)；本机总 CPU 占用 (能读 /proc/stat 时)
  * CPU 频率 (各核 scaling_cur_freq) 与温度 (thermal_zone / 电池)
  * 仿真: /api/v1/health 的 rtf；执行: /api/v1/nav 的 link.state_hz、nav2 状态，/api/v1/slam 的 tf_age_s
输出 JSON: 每项的平均 / 最小 / 最大，以及原始采样序列。
"""
import argparse
import json
import os
import time
import urllib.request

GROUPS = [
    ("ros2_launch(py)", "/opt/ros/humble/bin/ros2 "),      # ros2 launch / ros2 run 的 Python 外壳 (须排在具体节点之前)
    ("agv_ros_bridge", "agv_ros_bridge --in"),
    ("sim_server", "sim_server.api"), ("web_gateway", "web_gateway"), ("nav_runtime", "nav_runtime.main"),
    ("slam_toolbox", "slam_toolbox"), ("ekf", "ekf_node"), ("controller_server", "controller_server"),
    ("planner_server", "planner_server"), ("bt_navigator", "bt_navigator"), ("behavior_server", "behavior_server"),
    ("smoother_server", "smoother_server"), ("velocity_smoother", "velocity_smoother"),
    ("waypoint_follower", "waypoint_follower"), ("lifecycle_manager", "lifecycle_manager"),
    ("robot_state_publisher", "robot_state_publisher"), ("proot_tracers", "/usr/bin/proot"),
    ("platform(hub+agent)", "hub.server"), ("platform(hub+agent)", "agent.server"),
    ("vendor(/home/*)", "/home/"),                          # RK3588 车载控制器原有业务进程 (carServer 等)
]
HZ = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100
NCPU = os.cpu_count() or 1


def rd(p):
    try:
        with open(p) as f:
            return f.read()
    except Exception:
        return None


def get_json(url, timeout=3):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read() or b"{}")
    except Exception:
        return None


def procs():
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        cmd = rd(f"/proc/{pid}/cmdline")
        st = rd(f"/proc/{pid}/stat")
        if not cmd or not st:
            continue
        cmd = cmd.replace("\0", " ")
        g = next((name for name, key in GROUPS if key in cmd), None)
        if g is None:
            continue
        try:
            f = st.rsplit(")", 1)[1].split()
            ticks = int(f[11]) + int(f[12])
            rss = int(f[21]) * 4096
        except Exception:
            continue
        out[int(pid)] = (g, ticks, rss, pss(pid))
    return out


def pss(pid):
    t = rd(f"/proc/{pid}/smaps_rollup")
    for ln in (t or "").splitlines():
        if ln.startswith("Pss:"):
            return int(ln.split()[1]) * 1024
    return None


def total_cpu():
    t = rd("/proc/stat")
    if not t:
        return None
    v = [int(x) for x in t.splitlines()[0].split()[1:]]
    return (sum(v) - v[3] - (v[4] if len(v) > 4 else 0), sum(v)) if sum(v) > 0 else None


def mem_used():
    m = {}
    for ln in (rd("/proc/meminfo") or "").splitlines():
        k, _, v = ln.partition(":")
        m[k] = int(v.split()[0]) if v.split() else 0
    return round((m["MemTotal"] - m["MemAvailable"]) / 1024, 1) if "MemAvailable" in m else None


def freqs():
    r = {}
    for i in range(NCPU):
        v = rd(f"/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_cur_freq")
        if v and v.strip().isdigit():
            r[i] = int(v) // 1000
    return r


def temp():
    best = None
    for i in range(80):
        v = rd(f"/sys/class/thermal/thermal_zone{i}/temp")
        typ = (rd(f"/sys/class/thermal/thermal_zone{i}/type") or "").strip().lower()
        if v is None:
            continue
        try:
            t = int(v) / 1000.0
        except Exception:
            continue
        if (i == 0 or any(k in typ for k in ("cpu", "soc", "tsens"))) and 0 < t < 130:
            best = max(best or 0, t)
    if best is None:
        b = rd("/sys/class/power_supply/battery/temp")
        if b and b.strip().lstrip("-").isdigit():
            best = int(b) / 10.0
    return best


def stats(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return None
    return {"avg": round(sum(xs) / len(xs), 2), "min": round(min(xs), 2), "max": round(max(xs), 2), "n": len(xs)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=120)
    ap.add_argument("--interval", type=float, default=2.0)
    ap.add_argument("--sim", default="")
    ap.add_argument("--nav", default="")
    ap.add_argument("--label", default="")
    ap.add_argument("--out", default="perf_sample.json")
    ap.add_argument("--stop-file", default="", help="该文件出现时提前结束采样")
    a = ap.parse_args()
    series = {"t": [], "cpu": {}, "rss_mb": {}, "pss_mb": {}, "mem_used_mb": [], "host_cpu": [], "freq_mhz": [], "temp_c": [], "rtf": [], "state_hz": [],
              "tf_age_s": [], "nav2_active": [], "step_ms": [], "lidar_ms": []}
    prev, prev_tot, t_prev = procs(), total_cpu(), time.time()
    t0 = time.time()
    while time.time() - t0 < a.secs and not (a.stop_file and os.path.exists(a.stop_file)):
        time.sleep(a.interval)
        cur, tot, now = procs(), total_cpu(), time.time()
        dt = now - t_prev
        cpu, rss, ps = {}, {}, {}
        for pid, (g, ticks, r, pp) in cur.items():
            if pid in prev:
                cpu[g] = cpu.get(g, 0.0) + 100.0 * (ticks - prev[pid][1]) / HZ / dt
            rss[g] = rss.get(g, 0) + r
            if pp is not None:
                ps[g] = ps.get(g, 0) + pp
        for g in set(cpu) | set(series["cpu"]):
            series["cpu"].setdefault(g, []).append(round(cpu.get(g, 0.0), 1))
        for g, r in rss.items():
            series["rss_mb"].setdefault(g, []).append(round(r / 1e6, 1))
        for g, r in ps.items():
            series["pss_mb"].setdefault(g, []).append(round(r / 1e6, 1))
        series["mem_used_mb"].append(mem_used())
        if tot and prev_tot and tot[1] > prev_tot[1]:
            series["host_cpu"].append(round(100.0 * (tot[0] - prev_tot[0]) / (tot[1] - prev_tot[1]), 1))
        f = freqs()
        series["freq_mhz"].append(f)
        series["temp_c"].append(temp())
        series["t"].append(round(now - t0, 1))
        if a.sim:
            h = get_json(a.sim + "/api/v1/sim")        # rtf + 物理步/激光耗时 (EMA，含 GIL 等待)
            series["rtf"].append(h.get("rtf") if h else None)
            series["step_ms"].append(h.get("step_ms") if h else None)
            series["lidar_ms"].append(h.get("lidar_ms") if h else None)
        if a.nav:
            n = get_json(a.nav + "/api/v1/nav")
            series["state_hz"].append(((n or {}).get("link") or {}).get("state_hz"))
            series["nav2_active"].append(((n or {}).get("nav2") or {}).get("active"))
            s = get_json(a.nav + "/api/v1/slam")
            series["tf_age_s"].append(((s or {}).get("ext") or {}).get("tf_age_s"))
        prev, prev_tot, t_prev = cur, tot, now
    fr = {}
    for d in series["freq_mhz"]:
        for c, v in d.items():
            fr.setdefault(str(c), []).append(v)
    summary = {
        "label": a.label, "host": (rd("/proc/device-tree/model") or os.environ.get("AGV_DEVICE_MODEL") or os.uname().nodename).strip("\0\n "),
        "ncpu": NCPU, "secs": a.secs,
        "cpu_percent_by_group": {g: stats(v) for g, v in sorted(series["cpu"].items())},
        "cpu_percent_total_procs": stats([sum(series["cpu"][g][i] for g in series["cpu"] if i < len(series["cpu"][g]))
                                          for i in range(len(series["t"]))]),
        "rss_mb_by_group": {g: stats(v) for g, v in sorted(series["rss_mb"].items())},
        "pss_mb_by_group": {g: stats(v) for g, v in sorted(series["pss_mb"].items())},
        "mem_used_mb": stats(series["mem_used_mb"]),
        "host_cpu_percent": stats(series["host_cpu"]),
        "freq_mhz_by_core": {c: stats(v) for c, v in fr.items()},
        "temp_c": stats(series["temp_c"]),
        "rtf": stats(series["rtf"]), "step_ms": stats(series["step_ms"]), "lidar_ms": stats(series["lidar_ms"]),
        "state_hz": stats(series["state_hz"]), "tf_age_s": stats(series["tf_age_s"]),
        "nav2_active_ratio": (sum(1 for x in series["nav2_active"] if x) / len(series["nav2_active"])) if series["nav2_active"] else None,
    }
    with open(a.out, "w") as f:
        json.dump({"summary": summary, "series": series}, f, ensure_ascii=False)
    print(json.dumps(summary, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
