import json, math, sys, time, urllib.request
NAV, SIM = "http://127.0.0.1:8102", "http://127.0.0.1:8100"
def call(m, u, b=None, t=15):
    r = urllib.request.Request(u, data=None if b is None else json.dumps(b).encode(), method=m, headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=t).read() or b"{}")
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
planner, rounds = sys.argv[1], int(sys.argv[2])
call("PUT", NAV + "/api/v1/nav/planner", {"type": planner}); time.sleep(1)
goals = [(5, 0, 0), (0, 5, 1.5708), (-5, 0, 3.1416), (0, -5, -1.5708)] * rounds
ev0 = max([e["id"] for e in call("GET", NAV + "/api/v1/events?since=0").get("events", [])] or [0])
allerr = []
print(f"== 规划器 {planner}")
for gx, gy, gyaw in goals:
    call("POST", NAV + "/api/v1/missions", {"x": gx, "y": gy, "yaw": gyaw})
    t0 = time.time(); errs = []; yerrs = []; turn = 0.0; lastyaw = None; st = "?"; k = 0; mo = None; tfage = []
    while time.time() - t0 < 150:
        time.sleep(0.2); k += 1
        tr = call("GET", SIM + "/api/v1/snapshot")["state"]["truth"]
        sl = call("GET", NAV + "/api/v1/slam"); p = sl["pose"]; mo = sl["map_to_odom"]; od = sl["odom"]
        tfage.append((sl.get("ext") or {}).get("tf_age_s") or 0)
        errs.append(math.hypot(p["x"] - tr["x"], p["y"] - tr["y"])); yerrs.append(abs(wrap(p["yaw"] - tr["yaw"])))
        if lastyaw is not None: turn += abs(wrap(tr["yaw"] - lastyaw))
        lastyaw = tr["yaw"]
        if k % 5 == 0:
            st = call("GET", NAV + "/api/v1/nav")["status"]
            if st in ("ARRIVED", "FAILED", "ABORTED", "CANCELED", "IDLE") and time.time() - t0 > 3: break
    tr = call("GET", SIM + "/api/v1/snapshot")["state"]["truth"]
    errs.sort(); allerr += errs
    print(f"({gx:>2},{gy:>2}) {st:8s} {time.time() - t0:4.0f} s  终点 {math.hypot(tr['x'] - gx, tr['y'] - gy) * 1000:6.1f} mm/{math.degrees(wrap(tr['yaw'] - gyaw)):+5.2f}°  "
          f"定位误差 中位 {errs[len(errs) // 2] * 1000:5.0f} 最大 {errs[-1] * 1000:5.0f} mm, 朝向最大 {math.degrees(max(yerrs)):4.1f}°  累计转角 {math.degrees(turn):4.0f}°  "
          f"map→odom 朝向 {math.degrees(mo['yaw']):+6.1f}°  TF 龄最大 {max(tfage):.2f} s", flush=True)
    time.sleep(1.5)
allerr.sort()
print(f"   定位误差全程: 中位 {allerr[len(allerr) // 2] * 1000:.0f} mm, 95% {allerr[int(len(allerr) * 0.95)] * 1000:.0f} mm, 最大 {allerr[-1] * 1000:.0f} mm")
bad = [e for e in call("GET", NAV + f"/api/v1/events?since={ev0}").get("events", []) if e.get("level") in ("warning", "danger")]
for e in bad[:12]: print("   事件", e.get("type"), str(e.get("title"))[:30], "|", str(e.get("message"))[:110])
s = call("GET", SIM + "/api/v1/sim"); print("   rtf", s["rtf"], "碰撞", s["collisions"], "overruns", s["overruns"])
