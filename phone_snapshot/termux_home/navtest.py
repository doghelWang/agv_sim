import json, math, sys, time, urllib.request
NAV, SIM = "http://127.0.0.1:8102", "http://127.0.0.1:8100"
def call(m, u, b=None, t=15):
    r = urllib.request.Request(u, data=None if b is None else json.dumps(b).encode(), method=m, headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=t).read() or b"{}")
def truth():
    return call("GET", SIM + "/api/v1/snapshot")["state"]["truth"]
goals = json.loads(sys.argv[1])
ev0 = max([e["id"] for e in call("GET", NAV + "/api/v1/events?since=0").get("events", [])] or [0])
print("planner", call("GET", NAV + "/api/v1/nav")["planner"])
for gx, gy, gyaw in goals:
    m = call("POST", NAV + "/api/v1/missions", {"x": gx, "y": gy, "yaw": gyaw})
    t0 = time.time(); trace = []; st = "?"; nav = {}
    while time.time() - t0 < 170:
        time.sleep(0.2)
        p = truth(); trace.append((time.time() - t0, p["x"], p["y"], p["yaw"], math.hypot(p["vx"], p["vy"]), p["wz"]))
        if len(trace) % 5 == 0:
            nav = call("GET", NAV + "/api/v1/nav"); st = nav["status"]
            if st in ("ARRIVED", "FAILED", "ABORTED", "CANCELED", "IDLE") and time.time() - t0 > 3: break
    p = truth(); dyaw = math.degrees(math.atan2(math.sin(p["yaw"] - gyaw), math.cos(p["yaw"] - gyaw)))
    path = nav.get("path") or []; curve = nav.get("curve") or []
    print(f"目标 ({gx},{gy},{math.degrees(gyaw):.0f}°): {st}  用时 {time.time() - t0:.0f} s  终点误差 {math.hypot(p['x'] - gx, p['y'] - gy) * 1000:.1f} mm / {dyaw:+.2f}°  路线节点 {len(path)} 曲线点 {len(curve)}  恢复 {(nav.get('nav2_feedback') or {}).get('recoveries')}")
    for q in path[1:-1]:      # 每个中间节点: 车离它最近多少、经过时的最低速度与最大角速度 (圆弧过弯: 不经过节点、不停车)
        near = [r for r in trace if math.hypot(r[1] - q["x"], r[2] - q["y"]) < 1.6]
        if near:
            dmin = min(math.hypot(r[1] - q["x"], r[2] - q["y"]) for r in near)
            turned = abs(near[-1][3] - near[0][3])
            print(f"   节点 ({q['x']:.1f},{q['y']:.1f}): 最近 {dmin:.2f} m  转过 {math.degrees(turned):.0f}°  最低速度 {min(r[4] for r in near):.2f} m/s  最大角速度 {max(abs(r[5]) for r in near):.2f} rad/s")
    time.sleep(2)
for e in call("GET", NAV + f"/api/v1/events?since={ev0}").get("events", []):
    if e.get("type") not in ("LOC_REFINE", "TASKFLOW_STEP"): print("  事件", e.get("type"), e.get("level"), str(e.get("title"))[:40], "|", str(e.get("message"))[:130])
s = call("GET", SIM + "/api/v1/sim"); print("rtf", s["rtf"], "碰撞", s["collisions"], "overruns", s["overruns"])
