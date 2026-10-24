import json,sys,time,os,re,math,statistics as st,urllib.request as U,urllib.error
label=sys.argv[1]
N="http://127.0.0.1:8102/api/v1"; S="http://127.0.0.1:8100/api/v1"; WEB="http://127.0.0.1:8101"
HZ=os.sysconf("SC_CLK_TCK"); PG=os.sysconf("SC_PAGE_SIZE"); NCPU=os.cpu_count()
def call(m,u,b=None,t=10):
    r=U.Request(u,method=m,data=json.dumps(b).encode() if b is not None else None,headers={"Content-Type":"application/json"})
    try:
        with U.urlopen(r,timeout=t) as f: return f.status,json.load(f)
    except urllib.error.HTTPError as e:
        try: return e.code,json.load(e)
        except Exception: return e.code,None
    except Exception as e: return 0,None
def procs():
    out={}
    for p in os.listdir("/proc"):
        if not p.isdigit(): continue
        try:
            cmd=open(f"/proc/{p}/cmdline","rb").read().replace(b"\0",b" ").decode(errors="ignore").strip()
            if not cmd: continue
            s=open(f"/proc/{p}/stat").read(); rp=s.rfind(")"); f=s[rp+2:].split()
            ut,stm=int(f[11]),int(f[12]); nth=int(f[17])
            rss=int(open(f"/proc/{p}/statm").read().split()[1])*PG
            out[int(p)]=(cmd,ut+stm,nth,rss)
        except Exception: pass
    return out
GROUPS=[("sim",r"bin/sim-server|sim_server"),("web",r"bin/web-gateway|web_gateway"),
 ("nav",r"bin/nav-runtime|nav_runtime|nav2|slam_toolbox|ekf_node|robot_state_publisher|component_container|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager|agv_ros|fast-discovery|map_server|amcl|nav2_")]
def grp(cmd):
    if re.search(r"hub|agent|imgai|piagent|autostart|deep\.py|perf\.py|clip",cmd) and not re.search(r"bin/(sim-server|web-gateway|nav-runtime)",cmd): 
        if not re.search(r"nav_runtime|sim_server|web_gateway",cmd): return None
    for g,rx in GROUPS:
        if re.search(rx,cmd): return g
    return None
def sample(sec,work=None):
    a=procs(); t0=time.time(); simt0=call("GET",S+"/state")[1]
    if work: work()
    rem=sec-(time.time()-t0)
    if rem>0: time.sleep(rem)
    b=procs(); dt=time.time()-t0; simt1=call("GET",S+"/state")[1]
    res={g:{"cpu_pct":0.0,"rss_mb":0.0,"threads":0,"procs":0} for g,_ in GROUPS}; top=[]
    for pid,(cmd,tk,nth,rss) in b.items():
        if pid not in a: continue
        c=(tk-a[pid][1])/HZ/dt*100
        g=grp(cmd)
        if g:
            r=res[g]; r["cpu_pct"]+=c; r["rss_mb"]+=rss/1e6; r["threads"]+=nth; r["procs"]+=1
            top.append((round(c,1),cmd[:70]))
    for g in res:
        for k in("cpu_pct","rss_mb"): res[g][k]=round(res[g][k],1)
    top.sort(reverse=True)
    rtf=None
    try: rtf=(simt1["t"]-simt0["t"])/(simt1["wall_time"]-simt0["wall_time"])
    except Exception: pass
    seqrate=None
    try: seqrate=(simt1["seq"]-simt0["seq"])/(simt1["wall_time"]-simt0["wall_time"])
    except Exception: pass
    return {"window_s":round(dt,1),"groups":res,"top":top[:8],"sim_rtf":round(rtf,3) if rtf else None,"sim_steps_per_s":round(seqrate,1) if seqrate else None}
def lat(url,n=150):
    ts=[]
    for _ in range(n):
        t=time.perf_counter(); c,_=call("GET",url); ts.append((time.perf_counter()-t)*1000)
    ts.sort(); return {"p50_ms":round(ts[len(ts)//2],2),"p95_ms":round(ts[int(len(ts)*.95)],2),"max_ms":round(ts[-1],2)}
def wrap(a): return math.atan2(math.sin(a),math.cos(a))
W={s["id"]:s for s in call("GET",S+"/world")[1]["stations"]}
def nav(): return call("GET",N+"/nav")[1]
def go(sid,to=160):
    t0=time.time(); call("POST",N+"/missions",{"station":sid}); time.sleep(0.6)
    while time.time()-t0<to:
        n=nav()
        if n and n.get("status") in("ARRIVED","FAILED","NO_PATH","CANCELED"): break
        time.sleep(0.4)
    n=nav(); p=n["localization"]["pose"]; s=W[sid]
    return {"to":sid,"status":n["status"],"t_s":round(time.time()-t0,1),"pos_err_cm":round(math.hypot(p["x"]-s["x"],p["y"]-s["y"])*100,2),"yaw_err_deg":round(abs(math.degrees(wrap(p["yaw"]-s.get("dock_yaw",0)))),3)}
R={"label":label,"ncpu":NCPU}
call("PUT",N+"/nav/planner",{"type":"dijkstra"})
print("planner",nav().get("planner"),flush=True)
R["idle"]=sample(30); print("idle",R["idle"],flush=True)
R["lat_sim_state"]=lat(S+"/state"); R["lat_nav"]=lat(N+"/nav"); R["lat_sim_world"]=lat(S+"/world",50)
print(R["lat_sim_state"],R["lat_nav"],flush=True)
legs=[]
def work():
    for sid in["S1","S2","S3","S4"]: legs.append(go(sid))
R["drive"]=sample(1,work)   # sample window = total mission loop
R["legs"]=legs; print("drive",R["drive"],legs,flush=True)
R["lat_under_idle2"]=lat(N+"/nav",50)
json.dump(R,open(f"/root/perf_{label}.json","w"),indent=1)
print("DONE",flush=True)
