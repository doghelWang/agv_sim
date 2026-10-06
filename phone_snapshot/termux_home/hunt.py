import json,math,time,urllib.request,sys,os,glob,subprocess
N="http://127.0.0.1:8102"; S="http://127.0.0.1:8100"
def call(m,u,b=None):
    r=urllib.request.Request(u,data=None if b is None else json.dumps(b).encode(),method=m,headers={"Content-Type":"application/json"}); return json.loads(urllib.request.urlopen(r,timeout=15).read() or b"{}")
def pid_of(name):
    for d in glob.glob("/proc/[0-9]*"):
        try:
            c=open(d+"/cmdline","rb").read().split(b"\0")[0].decode()
            if c.endswith("/"+name): return int(d[6:])
        except Exception: pass
def threads(pid):
    o={}
    for t in glob.glob(f"/proc/{pid}/task/*"):
        try:
            st=open(t+"/stat").read().rsplit(")",1)[1].split()
            try: sc=open(t+"/syscall").read().split()[:3]
            except Exception as e: sc=[type(e).__name__]
            o[int(t.rsplit("/",1)[1])]=(open(t+"/comm").read().strip(),st[0],int(st[11])+int(st[12]),sc)
        except Exception: pass
    return o
def dump(name):
    pid=pid_of(name)
    if not pid: print("  ",name,"不在"); return
    a=threads(pid); time.sleep(2); b=threads(pid)
    print(f"  == {name} pid {pid}: {len(b)} 线程")
    for tid,(comm,state,ticks,sc) in sorted(b.items()):
        d=ticks-a.get(tid,(0,0,ticks))[2]
        print(f"     tid {tid} {comm:16s} 状态 {state} 2s内CPU {d:3d} tick  syscall {' '.join(sc)}")
print("cpuset",open("/proc/self/cpuset").read().strip(),flush=True)
goals=[(5,0,0),(0,5,1.5708),(-5,0,3.1416),(0,-5,-1.5708)]*4
for i,g in enumerate(goals):
    call("POST",N+"/api/v1/missions",{"x":g[0],"y":g[1],"yaw":g[2]}); t0=time.time(); last=None; still=0; dumped=False
    while time.time()-t0<70:
        time.sleep(1)
        n=call("GET",N+"/api/v1/nav"); a=call("GET",S+"/api/v1/snapshot")["state"]["truth"]
        p=(round(a["x"],2),round(a["y"],2),round(math.degrees(a["yaw"])))
        still = still+1 if p==last else 0; last=p
        if still>=12 and n["status"]=="NAVIGATING" and not dumped:
            dumped=True; print(time.strftime("%T"),f"任务 {i+1} 停住 {still}s 位置 {p}，线程状态:",flush=True)
            for nm in ("controller_server","bt_navigator","async_slam_toolbox_node","agv_ros_bridge"): dump(nm)
            sys.stdout.flush()
        if n["status"] not in ("NAVIGATING","PLANNING") and time.time()-t0>3: break
    print(time.strftime("%T"),f"任务 {i+1} {n['status']} {time.time()-t0:.0f}s",flush=True)
    if dumped: break
