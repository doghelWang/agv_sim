import subprocess,sys,bisect
def syms(lib):
    out=subprocess.run(["nm","-D","-C","--defined-only",lib],capture_output=True,text=True).stdout
    t=[]
    for l in out.splitlines():
        p=l.split(" ",2)
        if len(p)==3 and p[1] in "TtWw":
            t.append((int(p[0],16),p[2]))
    t.sort(); return t
req={"/opt/ros/humble/lib/libtf2_ros.so":[0x45f48,0x46660,0x46d08,0x47d50,0x47b90,0x4db20],
     "/opt/ros/humble/lib/libtf2.so":[0xaddc,0x8ef4,0x8f04,0xc5e0,0xffc0,0x10424],
     "/opt/ros/humble/lib/libtoolbox_common.so":[0x107f40]}
for lib,addrs in req.items():
    t=syms(lib); keys=[a for a,_ in t]
    for a in addrs:
        i=bisect.bisect_right(keys,a)-1
        print(lib.split("/")[-1], hex(a), "→", (t[i][1][:150]+" +0x%x"%(a-t[i][0])) if i>=0 else "?")
