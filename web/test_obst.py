import json,sys,time,urllib.request as u
S=(sys.argv[1] if len(sys.argv)>1 else "http://127.0.0.1:8100")+"/api/v1"
N=(sys.argv[2] if len(sys.argv)>2 else "http://127.0.0.1:8102")+"/api/v1"
def g(url): return json.load(u.urlopen(url,timeout=10))
def r(m,url,b=None):
    q=u.Request(url,data=None if b is None else json.dumps(b).encode(),method=m,headers={"Content-Type":"application/json"})
    return json.load(u.urlopen(q,timeout=20))
fails=[]
def chk(n,ok,i=""):
    print(("PASS " if ok else "FAIL ")+n,i); 
    if not ok: fails.append(n)
w=g(S+"/world"); print("stations:",[s["id"] for s in w.get("stations",[])][:12])
pl=g(N+"/nav").get("planners") or g(N+"/nav").get("available_planners")
for planner in ["Dijkstra","AStar","None"]:
    pass
def run(station,planner,obst,label):
    r("PUT",S+"/world/obstacles",obst)
    try: r("PUT",N+"/nav/planner",{"type":planner})
    except Exception as e: chk(f"{label} [{planner}] planner available",False,str(e)); return None
    c0=g(S+"/state")["collision"]["count"]
    r("POST",N+"/missions",{"station":station})
    t0=time.time(); st=None
    while time.time()-t0<150:
        time.sleep(2); m=g(N+"/missions/current"); st=m.get("status")
        if st in("ARRIVED","SUCCEEDED","DONE","SUCCESS","FAILED","ABORTED","NO_PATH","REJECTED","CANCELED"): break
    s=g(S+"/state"); tr=s["truth"]; col=s["collision"]["count"]-c0
    chk(f"{label} [{planner}] reached ({st})",st=="ARRIVED" and col==0,f"t={time.time()-t0:.0f}s pose=({tr['x']:.2f},{tr['y']:.2f}) new_collisions={col}")
    return s
for p in ["dijkstra","nav2"]:
    run("S1",p,[],"free S1")
    run("S3",p,[{"x":0,"y":-3,"w":0.8,"h":0.8}],"blocked corridor S3")
    run("P0",p,[{"x":0,"y":-0.5,"w":0.6,"h":0.6}],"obstacle near goal path")
r("PUT",S+"/world/obstacles",[])
print("RESULT:","ALL PASS" if not fails else f"FAILED {fails}")
