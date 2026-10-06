import glob,time,os,json,urllib.request
HZ=os.sysconf("SC_CLK_TCK")
def snap():
    o={}
    for d in glob.glob("/proc/[0-9]*"):
        try:
            cmd=open(d+"/cmdline","rb").read().replace(b"\0",b" ").decode("utf-8","replace")
            st=open(d+"/stat").read().rsplit(")",1)[1].split()
            o[int(d[6:])]=(cmd,int(st[11]),int(st[12]),int(st[1]))
        except Exception: pass
    return o
urllib.request.urlopen(urllib.request.Request("http://127.0.0.1:8102/api/v1/missions",data=json.dumps({"x":5,"y":0,"yaw":0}).encode(),method="POST",headers={"Content-Type":"application/json"}))
time.sleep(3); a=snap(); time.sleep(20); b=snap()
kids={}
for pid,(cmd,u,s,pp) in b.items(): kids.setdefault(pp,[]).append(pid)
rows=[]
for pid,(cmd,u,s,pp) in b.items():
    if "/bin/proot" in cmd.split(" ")[0] and pid in a:
        names=[]; kcpu=0
        for k in kids.get(pid,[]):
            c=b[k][0].split(" ")
            names.append(os.path.basename(c[0])+(" "+" ".join(x for x in c[1:4] if not x.startswith("-")) if "python" in c[0] else ""))
            if k in a: kcpu+=(b[k][1]+b[k][2]-a[k][1]-a[k][2])/HZ/20*100
        rows.append(((u+s-a[pid][1]-a[pid][2])/HZ/20*100,(s-a[pid][2])/HZ/20*100,kcpu,", ".join(names)[:70]))
for r in sorted(rows,reverse=True)[:12]: print("追踪 %5.1f%% (内核态 %4.1f%%) | 被追踪进程自身 %5.1f%% | %s"%r)
