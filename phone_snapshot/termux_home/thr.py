import glob,time,os
pid=[p.split("/")[2] for p in glob.glob("/proc/[0-9]*/cmdline") if b"web_gateway" in open(p,"rb").read() and b"proot" not in open(p,"rb").read()[:40]][0]
def snap():
    d={}
    for t in glob.glob(f"/proc/{pid}/task/*/stat"):
        try:
            f=open(t).read().rsplit(")",1)[1].split(); d[t.split("/")[4]]=int(f[11])+int(f[12])
        except Exception: pass
    f=open(f"/proc/{pid}/stat").read().rsplit(")",1)[1].split()
    return d,int(f[11])+int(f[12])
a,pa=snap(); time.sleep(20); b,pb=snap()
live=sorted(((b[t]-a.get(t,0))/20,t) for t in b)
print("进程合计 %.1f%%  线程数 %d  新增线程 %d  消失线程 %d"%((pb-pa)/20,len(b),len(set(b)-set(a)),len(set(a)-set(b))))
print("最忙线程:", ["%.1f"%x for x,_ in live[-6:]], " 现存线程合计 %.1f"%sum(x for x,_ in live))
