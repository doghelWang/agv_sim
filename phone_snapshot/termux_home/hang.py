import json,urllib.request,time,math,subprocess,sys
N="http://127.0.0.1:8102"; S="http://127.0.0.1:8100"
gy=float(sys.argv[1])
def call(m,u,b=None):
    r=urllib.request.Request(u,data=None if b is None else json.dumps(b).encode(),method=m,headers={"Content-Type":"application/json"}); return json.loads(urllib.request.urlopen(r,timeout=15).read() or b"{}")
ev0=call("GET",N+"/api/v1/events?since=0")["latest_id"]
call("POST",N+"/api/v1/missions",{"x":0,"y":gy,"yaw":math.copysign(1.5708,gy)}); time.sleep(6)
out=subprocess.run(["ps","-eo","ppid,args"],capture_output=True,text=True).stdout
pids=[l.split()[0] for l in out.splitlines() if l.split()[1:2]==["/opt/ros/humble/lib/nav2_controller/controller_server"]]
subprocess.run(["kill","-STOP"]+pids); print("冻结 controller_server 的 proot", pids, flush=True)
t=time.time(); st="?"
while time.time()-t<280:
    time.sleep(2); st=call("GET",N+"/api/v1/nav")["status"]
    if st not in ("NAVIGATING","PLANNING"): break
a=call("GET",S+"/api/v1/snapshot")["state"]["truth"]
print("结果: %s %.0f s, 终点偏差 %.0f mm" % (st, time.time()-t, math.hypot(a["x"],a["y"]-gy)*1000))
for e in call("GET",N+"/api/v1/events?since=%d"%ev0)["events"]:
    print("  事件", e.get("level"), e.get("type"), "|", str(e.get("title"))[:40], "|", str(e.get("message"))[:90])
