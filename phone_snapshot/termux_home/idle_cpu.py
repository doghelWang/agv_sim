import glob,time,sys
def snap():
    d={}
    for p in glob.glob("/proc/[0-9]*"):
        try:
            c=open(p+"/cmdline","rb").read().replace(b"\0",b" ").decode("utf-8","replace")
            f=open(p+"/stat").read().rsplit(")",1)[1].split()
            d[p]=(int(f[11])+int(f[12]),c)
        except Exception: pass
    return d
N=int(sys.argv[1]) if len(sys.argv)>1 else 30
a=snap(); time.sleep(N); b=snap()
g={}
for p,(t,c) in b.items():
    if p in a:
        k=("proot追踪" if "/proot " in c[:60] or c.startswith("proot ") else "web_gateway" if "web_gateway" in c else "sim" if "sim_server" in c else "nav_runtime" if "nav_runtime" in c else "bridge" if "agv_ros_bridge" in c else next((n for n in ("bt_navigator","controller_server","planner_server","behavior_server","velocity_smoother","robot_state_publisher","lifecycle_manager","map_server","spawn.py","proot_spawner","hub.server","agent.server","fastdds","sshd") if n in c), "其它"))
        g[k]=g.get(k,0)+(t-a[p][0])
print("静止 %d s, 单核%%:"%N, "  ".join(f"{k} {v/N:.1f}" for k,v in sorted(g.items(),key=lambda x:-x[1]) if v/N>=0.5), " 合计 %.0f"%(sum(g.values())/N))
