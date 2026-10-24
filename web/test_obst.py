import json,sys,time,urllib.request as u
S=(sys.argv[1] if len(sys.argv)>1 else "http://127.0.0.1:8100")+"/api/v1"
N=(sys.argv[2] if len(sys.argv)>2 else "http://127.0.0.1:8102")+"/api/v1"
planners=sys.argv[3].split(",") if len(sys.argv)>3 else ["dijkstra","nav2"]
def g(url): return json.load(u.urlopen(url,timeout=10))
def r(m,url,b=None):
    q=u.Request(url,data=None if b is None else json.dumps(b).encode(),method=m,headers={"Content-Type":"application/json"})
    return json.load(u.urlopen(q,timeout=30))
fails=[]
def chk(n,ok,i=""):
    print(("PASS " if ok else "FAIL ")+n,i,flush=True)
    if not ok: fails.append(n)
def pose(): t=g(S+"/state")["truth"]; return t["x"],t["y"]
def col(): return g(S+"/state")["collision"]["count"]
def wait_done(tmo):
    t0=time.time(); st=None
    while time.time()-t0<tmo:
        time.sleep(2); st=g(N+"/missions/current").get("status")
        if st in("ARRIVED","FAILED","ABORTED","NO_PATH","REJECTED","CANCELED","BUMPER_STOP"): break
    return st,time.time()-t0
def goto(st_id,tmo=150):
    x,y=pose(); s=[a for a in g(S+"/world")["stations"] if a["id"]==st_id][0]
    if abs(x-s["x"])<0.3 and abs(y-s["y"])<0.3: return "ARRIVED",0
    r("POST",N+"/missions",{"station":st_id}); return wait_done(tmo)
for p in planners:
    print("== planner",p,flush=True)
    r("PUT",S+"/world/obstacles",[])
    try: r("PUT",N+"/nav/planner",{"type":p})
    except Exception as e: chk(f"[{p}] planner available",False,str(e)); continue
    st,_=goto("P0"); chk(f"[{p}] go P0",st=="ARRIVED",st)
    c0=col()
    st,t=goto("S1"); chk(f"[{p}] free run to S1",st=="ARRIVED",f"{st} {t:.0f}s")
    st,_=goto("P0"); chk(f"[{p}] back to P0",st=="ARRIVED",st)
    # obstacle on the lane to S3 (south): vehicle must stop short without contact, then continue once cleared
    r("PUT",S+"/world/obstacles",[{"x":0,"y":-3.0,"w":0.8,"h":0.8}])
    r("POST",N+"/missions",{"station":"S3"})
    time.sleep(45); x,y=pose(); m=g(N+"/missions/current")
    chk(f"[{p}] stops short of obstacle",y>-2.7 and col()==c0,f"y={y:.2f} status={m.get('status')} collisions+={col()-c0}")
    r("PUT",S+"/world/obstacles",[])
    st,t=wait_done(120)
    if st!="ARRIVED":   # some planners need the mission re-issued after a hard failure
        st,t=goto("S3",120)
    chk(f"[{p}] resumes/arrives after obstacle cleared",st=="ARRIVED",f"{st} {t:.0f}s")
    chk(f"[{p}] no collisions in whole scenario",col()==c0,f"+{col()-c0}")
r("PUT",S+"/world/obstacles",[])
print("RESULT:","ALL PASS" if not fails else f"FAILED {fails}")
