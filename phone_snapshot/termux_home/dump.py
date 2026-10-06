import glob,time,sys
def pid_of(name):
    r=[]
    for d in glob.glob("/proc/[0-9]*"):
        try:
            c=open(d+"/cmdline","rb").read().split(b"\0")[0].decode()
            if c.endswith("/"+name): r.append(int(d[6:]))
        except Exception: pass
    return r
def threads(pid):
    o={}
    for t in glob.glob(f"/proc/{pid}/task/*"):
        try:
            st=open(t+"/stat").read().rsplit(")",1)[1].split()
            try: sc=open(t+"/syscall").read().split()[:4]
            except Exception as e: sc=[type(e).__name__]
            o[int(t.rsplit("/",1)[1])]=(open(t+"/comm").read().strip(),st[0],int(st[11])+int(st[12]),sc)
        except Exception: pass
    return o
for name in sys.argv[1:]:
    for pid in pid_of(name):
        a=threads(pid); time.sleep(2); b=threads(pid)
        print(f"== {name} pid {pid}: {len(b)} 线程")
        for tid,(comm,state,ticks,sc) in sorted(b.items()):
            print(f"   tid {tid} {comm:16s} {state} cpu {ticks-a.get(tid,(0,0,ticks))[2]:3d}  syscall {' '.join(sc)}")
