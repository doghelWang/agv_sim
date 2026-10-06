import os,time,socket,select,threading
def b(name,f,n=3000):
    t=time.perf_counter()
    for _ in range(n): f()
    return "%s %.0f"%(name,(time.perf_counter()-t)/n*1e6)
r,w=os.pipe(); a=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); a.bind(("127.0.0.1",0)); c=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); c.connect(a.getsockname())
u1,u2=socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM)
ep=select.epoll(); ep.register(r,select.EPOLLIN); p=b"x"*512
e1,e2=threading.Event(),threading.Event(); stop=[False]
def peer():
    while not stop[0]:
        e1.wait(); e1.clear(); e2.set()
th=threading.Thread(target=peer,daemon=True); th.start()
def pingpong(): e1.set(); e2.wait(); e2.clear()
res=[b("udp收发",lambda:(c.send(p),a.recv(600))),b("unix报文收发",lambda:(u1.send(p),u2.recv(600))),b("epoll_wait",lambda: ep.poll(0)),
     b("线程唤醒往返(futex)",pingpong,1500),b("fstat",lambda: os.fstat(r)),b("getuid",os.getuid),b("stat",lambda: os.stat("/proc/self/stat"),800),
     b("打开读关闭",lambda: open("/proc/self/stat").read(),500),b("pipe读写",lambda:(os.write(w,p),os.read(r,512)))]
stop[0]=True; e1.set()
print(" | ".join(res))
