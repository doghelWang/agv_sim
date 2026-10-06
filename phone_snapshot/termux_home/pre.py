import json, math, sys, time, urllib.request
NAV, SIM = "http://127.0.0.1:8102", "http://127.0.0.1:8100"
def call(m, u, b=None, t=15):
    r = urllib.request.Request(u, data=None if b is None else json.dumps(b).encode(), method=m, headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=t).read() or b"{}")
def truth(): return call("GET", SIM + "/api/v1/snapshot")["state"]["truth"]
def show(tag):
    n = call("GET", NAV + "/api/v1/nav"); a = truth()
    print(f"{time.time()-T0:5.1f}s {tag} 状态 {n['status']} 任务 {n['mission_id']} 位置 {a['x']:.2f},{a['y']:.2f},{math.degrees(a['yaw']):.0f}° 路线 {[(round(p['x'],1),round(p['y'],1)) for p in n.get('path',[])]} 曲线点 {len(n.get('curve',[]))} 曲线终点 {(n.get('curve') or [{}])[-1]}", flush=True)
T0 = time.time()
call("POST", NAV + "/api/v1/missions", {"x": 5, "y": 0, "yaw": 0}); time.sleep(4); show("目标(5,0) 4 s 后")
t = time.time(); call("POST", NAV + "/api/v1/missions", {"x": 0, "y": 5, "yaw": 1.5708}); print(f"   换目标 (0,5) 调用耗时 {time.time()-t:.2f} s")
for i in range(40):
    time.sleep(2); show("  ")
    if call("GET", NAV + "/api/v1/nav")["status"] not in ("NAVIGATING", "PLANNING"): break
