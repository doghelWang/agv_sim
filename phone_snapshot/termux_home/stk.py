import sys,struct,os,re,glob
pid=int(sys.argv[1]); tids=[int(x) for x in sys.argv[2:]] or [int(os.path.basename(t)) for t in glob.glob(f"/proc/{pid}/task/*")]
maps=[]
for l in open(f"/proc/{pid}/maps"):
    m=re.match(r"([0-9a-f]+)-([0-9a-f]+) (\S+) ([0-9a-f]+) \S+ \S+\s*(.*)",l)
    if m and "x" in m.group(3) and m.group(5).startswith("/"):
        maps.append((int(m.group(1),16),int(m.group(2),16),int(m.group(4),16),m.group(5)))
def loc(a):
    for s,e,off,path in maps:
        if s<=a<e: return os.path.basename(path), a-s+off
mem=open(f"/proc/{pid}/mem","rb",0)
for tid in tids:
    f=open(f"/proc/{pid}/task/{tid}/syscall").read().split()
    if len(f)<9: print(tid,f); continue
    sp=int(f[7],16); pc=int(f[8],16)
    out=[]
    try:
        mem.seek(sp); data=mem.read(8*3000)
    except Exception as e:
        print(tid,"读栈失败",e); continue
    for i in range(0,len(data)-7,8):
        v=struct.unpack_from("<Q",data,i)[0]
        l=loc(v)
        if l and (not out or out[-1]!=l): out.append(l)
    print(f"TID {tid} syscall {f[0]} arg0 {f[1]} pc {loc(pc)}")
    for lib,off in out[:int(os.environ.get('N','40'))]: print(f"    {lib}+0x{off:x}")
