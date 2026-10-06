import os,time,socket,select,threading,sys
N=4000
def b(name,f,n=N):
    t=time.perf_counter()
    for _ in range(n): f()
    return name,(time.perf_counter()-t)/n*1e6
r,w=os.pipe(); a=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); a.bind(("127.0.0.1",0)); c=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); c.connect(a.getsockname())
u1,u2=socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM)
ep=select.epoll(); ep.register(r,select.EPOLLIN)
p=b"x"*512; lk=threading.Lock()
def pw(): os.write(w,p); os.read(r,512)
def us(): c.send(p); a.recv(600)
def ust(): c.sendto(p,a.getsockname()) ; a.recvfrom(600)
def un(): u1.send(p); u2.recv(600)
def sm(): c.sendmsg([p]); a.recvmsg(600)
res=[b("getppid",os.getppid),b("pipe write+read",pw),b("udp send+recv (connected)",us),b("udp sendto+recvfrom",ust),b("udp sendmsg+recvmsg",sm),b("unix dgram send+recv",un),
     b("epoll_wait(0)",lambda: ep.poll(0)),b("clock_gettime",time.monotonic),b("stat",lambda: os.stat("/proc/self/stat"),1000),b("open+read+close /proc",lambda: open("/proc/self/stat").read(),1000),
     b("getuid",os.getuid),b("sched_yield",os.sched_yield),b("nanosleep(0)",lambda: time.sleep(0))]
print(" | ".join("%s %.0f"%x for x in res))
