import json,urllib.request,time,math
N="http://127.0.0.1:8102"; S="http://127.0.0.1:8100"
def call(m,u,b=None):
    r=urllib.request.Request(u,data=None if b is None else json.dumps(b).encode(),method=m,headers={"Content-Type":"application/json"}); return json.loads(urllib.request.urlopen(r,timeout=15).read() or b"{}")
call("POST",N+"/api/v1/missions",{"x":5,"y":0,"yaw":0}); time.sleep(5)
call("DELETE",N+"/api/v1/missions/current"); call("POST",N+"/api/v1/missions",{"x":0,"y":5,"yaw":1.5708})
t=time.time()
while time.time()-t<90:
    time.sleep(1); st=call("GET",N+"/api/v1/nav")["status"]
    if st not in ("NAVIGATING","PLANNING"): break
a=call("GET",S+"/api/v1/snapshot")["state"]["truth"]
print("行驶中取消后立刻下发 (0,5): %s %.0f s, 终点偏差 %.0f mm" % (st, time.time()-t, math.hypot(a["x"],a["y"]-5)*1000))
