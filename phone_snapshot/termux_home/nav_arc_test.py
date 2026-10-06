import json, math, sys, time, urllib.request
N, S = "http://127.0.0.1:8102", "http://127.0.0.1:8100"
def call(m, u, b=None):
    r = urllib.request.Request(u, data=json.dumps(b).encode() if b is not None else None, method=m, headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=15).read() or b"{}")
goals = [(0, 5, 1.5708), (5, 0, 0.0), (0, -5, -1.5708), (-5, 0, 3.1416)]
ev0 = 0
for gx, gy, gyaw in goals:
    m = call("POST", N + "/api/v1/missions", {"x": gx, "y": gy, "yaw": gyaw}); t0 = time.time(); maxw = 0.0; vmin_turn = 9.0; arc_v = []
    st = "?"
    while time.time() - t0 < 240:
        time.sleep(0.5)
        d = call("GET", N + "/api/v1/nav"); st = d["status"]
        tr = call("GET", S + "/api/v1/snapshot")["state"]["truth"]
        if abs(tr["wz"]) > 0.15: arc_v.append(round(math.hypot(tr["vx"], tr["vy"]), 2))
        if st in ("ARRIVED", "FAILED", "ABORTED", "CANCELED") and time.time() - t0 > 3: break
    tr = call("GET", S + "/api/v1/snapshot")["state"]["truth"]
    err = math.hypot(tr["x"] - gx, tr["y"] - gy) * 1000; dy = math.degrees(math.atan2(math.sin(tr["yaw"] - gyaw), math.cos(tr["yaw"] - gyaw)))
    cur = d.get("curve") or []; rec = (d.get("nav2_feedback") or {}).get("recoveries")
    print(f"goal ({gx},{gy}) {st} {time.time() - t0:.0f}s err {err:.1f} mm yaw {dy:+.2f} deg curve_pts {len(cur)} recov {rec}; turning samples {len(arc_v)}, of which moving(>0.15 m/s) {sum(v > 0.15 for v in arc_v)}", flush=True)
    evs = call("GET", N + f"/api/v1/events?since={ev0}"); evs = evs.get("events", evs)
    for e in evs:
        ev0 = max(ev0, e.get("id", 0))
        if e.get("type") in ("MISSION_DISPATCH", "NAV2_RETRY", "NAV2_ALIGN", "NAV2_GIVEUP", "ROTATE_BLOCKED", "ADJUST_FAIL", "ARRIVE_CHECK", "CORNER_TIGHT", "MISSION_FAILED", "ROUTE_PLAN"): print("   ", e.get("type"), e.get("title"), "|", str(e.get("message"))[:160], flush=True)
print("col", call("GET", S + "/api/v1/sim").get("collisions"))
