import time,sys,os
sys.path.insert(0,"/opt/agv")
from agent.server import SysInfo, free_disk
s=SysInfo()
def T(n,f):
    t=time.perf_counter(); c=time.process_time()
    try: r=f()
    except Exception as e: r="ERR %r"%e
    print("%-16s wall %7.1f ms cpu %7.1f ms  %s"%(n,(time.perf_counter()-t)*1e3,(time.process_time()-c)*1e3,str(r)[:60]))
s.cpu_percent()
T("cpu_percent",s.cpu_percent); T("mem",s.mem); T("temp",s.temp); T("model",s.model); T("os_name",s.os_name); T("ips",s.ips)
T("ports 8090-8200",lambda: s.listening_ports(8090,8200)); T("disk",lambda: free_disk("/root")); T("loadavg",os.getloadavg)
