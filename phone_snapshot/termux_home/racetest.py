import json, math, sys, time, urllib.request
NAV, SIM = "http://127.0.0.1:8102", "http://127.0.0.1:8100"
def call(m, u, b=None, t=15):
    r = urllib.request.Request(u, data=None if b is None else json.dumps(b).encode(), method=m, headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=t).read() or b"{}")
def truth(): return call("GET", SIM + "/api/v1/snapshot")["state"]["truth"]
call("PUT", NAV + "/api/v1/nav/planner", {"type": "nav2"}); time.sleep(1)
ev0 = max([e["id"] for e in call("GET", NAV + "/api/v1/events?since=0").get("events", [])] or [0])
goals = [(5, 0, 0), (0, 5, 1.5708), (-5, 0, 3.1416), (0, -5, -1.5708), (5, 0, 0), (0, 0, 0)]
for i, (gx, gy, gyaw) in enumerate(goals):
    call("POST", NAV + "/api/v1/missions", {"x": gx, "y": gy, "yaw": gyaw})
    time.sleep(5)
    if i < len(goals) - 1:
        a = truth(); print(f"第 {i+1} 个目标 ({gx},{gy}) 发出 5 s 后: 状态 {call('GET', NAV + '/api/v1/nav')['status']} 位置 {a['x']:.2f},{a['y']:.2f} → 立刻换下一个目标", flush=True)
t0 = time.time(); p0 = truth()
while time.time() - t0 < 150:
    time.sleep(1); st = call("GET", NAV + "/api/v1/nav")["status"]
    if st in ("ARRIVED", "FAILED", "ABORTED", "CANCELED", "IDLE"): break
a = truth(); print(f"最后一个目标 (0,0): {st} {time.time()-t0:.0f} s, 终点偏差 {math.hypot(a['x'], a['y'])*1000:.0f} mm")
for e in call("GET", NAV + f"/api/v1/events?since={ev0}").get("events", []):
    if e.get("level") in ("warning", "danger"): print("   事件", e.get("type"), "|", str(e.get("message"))[:110])
