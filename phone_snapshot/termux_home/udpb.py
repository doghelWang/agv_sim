import socket,time,os,sys
a=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); a.bind(("127.0.0.1",0)); b=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
addr=a.getsockname(); p=b"x"*1200; N=20000
t=time.perf_counter()
for i in range(N): b.sendto(p,addr); a.recvfrom(2048)
dt=time.perf_counter()-t
print("UDP 回环 收发一对: %.1f µs" % (dt/N*1e6))
t=time.perf_counter()
for i in range(N): os.stat("/proc/self/stat")
print("stat (路径类调用): %.1f µs" % ((time.perf_counter()-t)/N*1e6))
t=time.perf_counter()
for i in range(N): b.sendto(p,("127.0.0.1",7399+i%120))
print("发往未监听端口 sendto: %.1f µs" % ((time.perf_counter()-t)/N*1e6))
import threading
ev=threading.Event(); n=[0]
def w():
    for i in range(2000): ev.wait(); ev.clear(); 
t=time.perf_counter()
l=threading.Lock(); c=threading.Condition(l)
