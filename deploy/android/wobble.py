#!/usr/bin/env python3
"""跟线"歪扭"量化 (Termux 里运行): python3 ~/wobble.py [标签] [圈数=1]
四工位任务，10 Hz 采样仿真真值与定位结果，只统计直线段 (车速 > 0.2 m/s，沿 x 或 y 轴线行驶):
  横向偏差 (离线路中心线，mm)、车头偏角 (°)、角速度、角速度换向次数/米 —— 车身实际扭不扭
  定位横向误差 (定位结果 - 真值，mm)、定位朝向误差 (°)、它们每 0.1 s 的跳变量 —— 控制器看到的"自己在哪"稳不稳
两者一起看: 定位误差在跳而真值跟着扭 → 是定位在带着车扭；定位很稳而真值仍扭 → 是控制/执行的问题"""
import json, math, os, sys, time, urllib.request

TAG = sys.argv[1] if len(sys.argv) > 1 else "run"
ROUNDS = int(sys.argv[2]) if len(sys.argv) > 2 else 1
NAV, SIM = "http://127.0.0.1:8102", "http://127.0.0.1:8100"


def call(m, u, b=None, t=10):
    r = urllib.request.Request(u, data=None if b is None else json.dumps(b).encode(), method=m, headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=t).read() or b"{}")


def wrap(a): return math.atan2(math.sin(a), math.cos(a))


rows = []
goals = [(5, 0, 0), (0, 5, 1.5708), (-5, 0, 3.1416), (0, -5, -1.5708)] * ROUNDS
call("PUT", NAV + "/api/v1/nav/planner", {"type": "nav2"})
for gi, (gx, gy, gyaw) in enumerate(goals):
    call("POST", NAV + "/api/v1/missions", {"x": gx, "y": gy, "yaw": gyaw})
    ts = time.time(); k = 0; st = "?"
    while time.time() - ts < 150:
        t0 = time.time()
        try:
            tr = call("GET", SIM + "/api/v1/snapshot")["state"]["truth"]
            p = call("GET", NAV + "/api/v1/slam")["pose"]
        except Exception:
            time.sleep(0.1); continue
        v = math.hypot(tr.get("vx", 0), tr.get("vy", 0))
        on_x, on_y = abs(tr["y"]) < 0.6 and abs(tr["x"]) > 0.8, abs(tr["x"]) < 0.6 and abs(tr["y"]) > 0.8
        if v > 0.2 and (on_x or on_y):
            if on_x:      # 沿 x 轴线: 横向 = y，线路方向 0 或 π
                lat, hd = tr["y"], min(abs(wrap(tr["yaw"])), abs(wrap(tr["yaw"] - math.pi)))
                sgn = 1 if abs(wrap(tr["yaw"])) < 1.57 else -1
                hd_s = wrap(tr["yaw"]) if sgn > 0 else wrap(tr["yaw"] - math.pi)
                loc_lat = p["y"] - tr["y"]
            else:
                lat = tr["x"]
                sgn = 1 if abs(wrap(tr["yaw"] - 1.5708)) < 1.57 else -1
                hd_s = wrap(tr["yaw"] - 1.5708) if sgn > 0 else wrap(tr["yaw"] + 1.5708)
                loc_lat = p["x"] - tr["x"]
            rows.append((gi, time.time(), v, lat * sgn * (1 if on_x else -1), hd_s, tr.get("wz", 0), loc_lat, wrap(p["yaw"] - tr["yaw"])))
        k += 1
        if k % 10 == 0:
            st = call("GET", NAV + "/api/v1/nav")["status"]
            if st in ("ARRIVED", "FAILED", "ABORTED", "CANCELED", "IDLE") and time.time() - ts > 3:
                break
        time.sleep(max(0.0, 0.1 - (time.time() - t0)))
    print(f"任务 {gi + 1} {st} {time.time() - ts:.0f}s", flush=True)
if len(rows) < 20:
    print("直线段采样太少"); sys.exit(0)
rms = lambda v: math.sqrt(sum(x * x for x in v) / len(v))
lat = [r[3] * 1000 for r in rows]; hd = [math.degrees(r[4]) for r in rows]; wz = [math.degrees(r[5]) for r in rows]
ll = [r[6] * 1000 for r in rows]; ly = [math.degrees(r[7]) for r in rows]
dist = sum(r[2] for r in rows) * 0.1
flips = sum(1 for i in range(1, len(rows)) if rows[i][0] == rows[i - 1][0] and wz[i] * wz[i - 1] < 0 and abs(wz[i] - wz[i - 1]) > 2.0)
dj = [abs(ll[i] - ll[i - 1]) for i in range(1, len(rows)) if rows[i][0] == rows[i - 1][0] and rows[i][1] - rows[i - 1][1] < 0.3]
dy = [abs(ly[i] - ly[i - 1]) for i in range(1, len(rows)) if rows[i][0] == rows[i - 1][0] and rows[i][1] - rows[i - 1][1] < 0.3]
cs = open("/proc/self/cpuset").read().strip()
print(f"[{TAG}] cpuset {cs}  直线段采样 {len(rows)} 点 ≈ {dist:.0f} m")
print(f"车身 (真值): 横向偏差 RMS {rms(lat):.0f} mm 最大 {max(abs(x) for x in lat):.0f} mm | 车头偏角 RMS {rms(hd):.2f}° 最大 {max(abs(x) for x in hd):.1f}° | "
      f"角速度 RMS {rms(wz):.1f}°/s 最大 {max(abs(x) for x in wz):.0f}°/s | 角速度换向 {flips} 次 = {flips / max(dist, 1):.2f} 次/米")
print(f"定位 (结果-真值): 横向误差 RMS {rms(ll):.0f} mm 最大 {max(abs(x) for x in ll):.0f} mm | 朝向误差 RMS {rms(ly):.2f}° 最大 {max(abs(x) for x in ly):.1f}° | "
      f"0.1 s 内跳变: 横向 均值 {sum(dj) / len(dj):.1f} 最大 {max(dj):.0f} mm, 朝向 均值 {sum(dy) / len(dy):.2f} 最大 {max(dy):.1f}°; 跳变 > 20 mm 的次数 {sum(x > 20 for x in dj)}")
n = len(rows); mx, my = sum(ll) / n, sum(hd) / n
cov = sum((ll[i] - mx) * (hd[i] - my) for i in range(n)); sx = math.sqrt(sum((x - mx) ** 2 for x in ll)); sy = math.sqrt(sum((y - my) ** 2 for y in hd))
print(f"定位横向误差 与 车头偏角 的相关系数: {cov / max(sx * sy, 1e-9):+.2f}")
with open(os.path.expanduser(f"~/wobble_{TAG}.csv"), "w") as f:
    f.write("task,t,v,lat_mm,head_deg,wz_dps,loc_lat_mm,loc_yaw_deg\n")
    for i, r in enumerate(rows):
        f.write(f"{r[0]},{r[1]:.2f},{r[2]:.2f},{lat[i]:.1f},{hd[i]:.2f},{wz[i]:.1f},{ll[i]:.1f},{ly[i]:.2f}\n")
