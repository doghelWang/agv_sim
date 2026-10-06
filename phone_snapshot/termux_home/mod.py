import json,math,time,urllib.request,sys
N="http://127.0.0.1:8102"; S="http://127.0.0.1:8100"
def call(m,u,b=None):
    r=urllib.request.Request(u,data=None if b is None else json.dumps(b).encode(),method=m,headers={"Content-Type":"application/json"}); return json.loads(urllib.request.urlopen(r,timeout=15).read() or b"{}")
print("cpuset",open("/proc/self/cpuset").read().strip(),flush=True)
for g in [(5,0,0),(0,5,1.5708)]:
    call("POST",N+"/api/v1/missions",{"x":g[0],"y":g[1],"yaw":g[2]}); t0=time.time(); last=None; still=0
    while time.time()-t0<110:
        time.sleep(2)
        n=call("GET",N+"/api/v1/nav"); a=call("GET",S+"/api/v1/snapshot")["state"]["truth"]; sl=call("GET",N+"/api/v1/slam")
        p=(round(a["x"],2),round(a["y"],2),round(math.degrees(a["yaw"])))
        still = still+2 if p==last else 0; last=p
        if still in (0,10,30,60) or n["status"]!="NAVIGATING": print("%s %5.0fs %s 位置 %s 不动 %ds tf龄 %.2f rec=%s"%(time.strftime("%T"),time.time()-t0,n["status"],p,still,(sl.get("ext") or {}).get("tf_age_s") or 0,(n.get("nav2_feedback") or {}).get("recoveries")),flush=True)
        if n["status"] not in ("NAVIGATING","PLANNING") and time.time()-t0>3: break
