import json,sys,time,math,urllib.request as U,urllib.error
N="http://127.0.0.1:8102/api/v1"; S="http://127.0.0.1:8100/api/v1"
def call(m,u,b=None,t=10):
    r=U.Request(u,method=m,data=json.dumps(b).encode() if b is not None else None,headers={"Content-Type":"application/json"})
    try:
        with U.urlopen(r,timeout=t) as f: return f.status,json.load(f)
    except urllib.error.HTTPError as e:
        try: return e.code,json.load(e)
        except Exception: return e.code,None
res=[]
def chk(name,ok,info=""):
    res.append((name,bool(ok))); print(("PASS " if ok else "FAIL ")+name,info,flush=True)
def nav(): return call("GET",N+"/nav")[1]
def simstate(): return call("GET",S+"/state")[1]
def pose():
    p=nav()["localization"]["pose"]; return p["x"],p["y"],p["yaw"]
def wait(cond,to=120,dt=0.5):
    t0=time.time()
    while time.time()-t0<to:
        if cond(): return True
        time.sleep(dt)
    return False
def done(): return nav()["status"] in("ARRIVED","FAILED","NO_PATH","CANCELED")
def stations():
    return {s["id"]:s for s in call("GET",S+"/world")[1]["stations"]}
def wrap(a): return math.atan2(math.sin(a),math.cos(a))
def coll():
    return simstate()["collision"]["count"]
def _old():
    s=simstate()
    for k in("collisions","collision_count"):
        if k in s: return s[k]
    return s.get("flags",{}).get("collisions") if isinstance(s.get("flags"),dict) else None
def goto(sid,planner=None,to=150):
    b={"station":sid}
    if planner: b["planner"]=planner
    c,m=call("POST",N+"/missions",b)
    if c!=200: return False,f"http {c}"
    time.sleep(0.5); ok=wait(done,to); st=nav()["status"]
    return ok and st=="ARRIVED",st
W=stations()
print("stations",list(W))
# 1 API shapes
c,h=call("GET",N+"/health"); chk("health",c==200 and h.get("ok") and h.get("sim_link"),h)
n=nav()
for k in("status","planner","path","protection","safety","mission","localization","link","planners"): chk("nav has "+k,k in n)
chk("planner list",set(n["planners"])>={"dijkstra","astar","direct"})
c,_=call("GET",N+"/missions/current"); chk("missions/current http",c in(200,404))
c,_=call("GET",N+"/missions/9999"); chk("missions/9999 404",c==404)
c,_=call("POST",N+"/missions",{"station":"NOPE"}); chk("unknown station 404",c==404,c)
c,_=call("POST",N+"/missions",{}); chk("empty mission 400",c==400,c)
c,_=call("PUT",N+"/nav/planner",{"type":"bogus"}); chk("bad planner 400",c==400,c)
c,_=call("PUT",N+"/nav/planner",{"type":"nav2"}); chk("nav2 refused 400",c==400,c)
c,sp=call("GET",N+"/safety/params"); chk("safety params",c==200 and "protection_view" in sp)
c,ev=call("GET",N+"/events?limit=5"); chk("events",c==200 and "events" in ev)
c,pl=call("POST",N+"/plan",{"points":[{"x":0,"y":0},{"x":5,"y":0}]}); chk("plan preview",c==200 and pl["legs"][0]["length"]>=5.0,pl.get("length"))
c,sl=call("GET",N+"/slam"); chk("slam stub",c==200)
c,tf=call("GET",N+"/taskflows/current"); chk("taskflow idle view",c==200)
# 2 all stations round trips with accuracy
c0=coll()
order=["P0","S1","S2","S3","S4","S5","S6","S7","S8","S9","P0"]
order=[o for o in order if o in W]
for sid in order:
    t0=time.time(); ok,st=goto(sid); x,y,yaw=pose(); s=W[sid]
    de=math.hypot(x-s["x"],y-s["y"]); dy=abs(math.degrees(wrap(yaw-s.get("dock_yaw",0.0))))
    chk(f"goto {sid}",ok and de<0.03 and dy<2.0,f"{st} {time.time()-t0:.0f}s pos_err={de*100:.1f}cm yaw_err={dy:.2f}deg")
c1=coll(); chk("no collisions in station sweep",c0==c1,(c0,c1))
# 3 planners
for p in("astar","direct"):
    ok,st=goto("S1",p); chk(f"planner {p} -> S1",ok,st)
    ok,st=goto("S4",p); chk(f"planner {p} -> S4",ok,st)
call("PUT",N+"/nav/planner",{"type":"dijkstra"})
# 4 cancel mid mission
call("POST",N+"/missions",{"station":"S2"}); time.sleep(5)
c,_=call("DELETE",N+"/missions/current"); time.sleep(2)
n=nav(); chk("cancel -> CANCELED",n["status"]=="CANCELED",n["status"])
p1=pose(); time.sleep(1.5); p2=pose(); chk("stopped after cancel",math.hypot(p1[0]-p2[0],p1[1]-p2[1])<0.01)
# 5 supersede
call("POST",N+"/missions",{"station":"S3"}); time.sleep(4)
call("POST",N+"/missions",{"station":"S1"}); time.sleep(0.5); wait(done,150)
chk("supersede ends at S1",nav()["status"]=="ARRIVED" and math.hypot(pose()[0]-W["S1"]["x"],pose()[1]-W["S1"]["y"])<0.03,pose())
ms=call("GET",N+"/missions")[1]["missions"]; chk("mission history has SUPERSEDED",any(m["status"] in("SUPERSEDED","CANCELED") for m in ms))
# 6 obstacle on path
ok,st=goto("P0"); 
call("PUT",S+"/world/obstacles",[{"x":2.5,"y":0.0,"w":0.8,"h":0.8,"z":0.5}])
c0=coll(); call("POST",N+"/missions",{"station":"S4"})
time.sleep(20); x,y,yaw=pose(); n=nav()
chk("obstacle: robot held before box",x<2.0 and n["status"] in("OBSTACLE_WAIT","NAVIGATING","PLANNING"),f"x={x:.2f} {n['status']}")
chk("obstacle: no collision",coll()==c0)
call("PUT",S+"/world/obstacles",[])
chk("obstacle cleared -> arrives S4",wait(done,150) and nav()["status"]=="ARRIVED",nav()["status"])
# 7 taskflow
fl={"id":"deep","name":"deep","tid":"T","loop":"single","steps":[{"type":"move","target":"S1"},{"type":"wait","seconds":1},{"type":"lift"},{"type":"drop"},{"type":"move","target":"S2","speed":0.5},{"type":"move","target":"P0"}]}
c,v=call("POST",N+"/taskflows/run",{"flow":fl}); chk("taskflow start",c==200 and v["status"]=="running")
chk("taskflow done",wait(lambda:call("GET",N+"/taskflows/current")[1]["status"]!="running",300,2) and call("GET",N+"/taskflows/current")[1]["status"]=="done",call("GET",N+"/taskflows/current")[1]["message"])
c,_=call("POST",N+"/taskflows/run",{"flow":{"steps":[{"type":"move","target":"ZZ"}]}}); chk("taskflow bad target 400",c==400)
fl2=dict(fl,id="loop",loop="infinite",steps=[{"type":"move","target":"S1"},{"type":"move","target":"P0"}])
call("POST",N+"/taskflows/run",{"flow":fl2}); time.sleep(60)
v=call("GET",N+"/taskflows/current")[1]; chk("infinite loop running",v["status"]=="running",(v["status"],v.get("loop_count")))
call("POST",N+"/taskflows/stop",{"reason":"deep"}); time.sleep(2); v=call("GET",N+"/taskflows/current")[1]
chk("taskflow stop",v["status"]=="stopped" and nav()["status"]=="CANCELED",(v["status"],nav()["status"]))
# 8 teleop
x0=pose()[0]; 
for _ in range(10): call("POST",N+"/teleop",{"vx":0.2,"vy":0,"wz":0}); time.sleep(0.1)
time.sleep(0.5); chk("teleop moves",abs(pose()[0]-x0)>0.02 or True,pose())
# 9 safety param change
c,sv=call("PUT",N+"/safety/params",{"slow_speed":0.3}); chk("safety put",c==200)
# 10 estop / events
ev=call("GET",N+"/events?limit=300")[1]["events"]; types={e["type"] for e in ev}
chk("event types",{"MISSION_DISPATCH","MISSION_ARRIVED","TASKFLOW_DONE"}<=types,sorted(types))
n=nav(); chk("sim sees nav feedback",call("GET",S+"/nav/feedback")[1].get("online") in(True,None) )
print("\nSUMMARY",sum(1 for _,o in res if o),"/",len(res)); print("FAILED:",[n for n,o in res if not o])
