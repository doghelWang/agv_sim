import time,os,sys,math
c=int(sys.argv[1]); os.sched_setaffinity(0,{c})
def work():
    s=0.0
    for i in range(400000): s+=math.sqrt(i)*1.0001
    return s
work(); best=1e9
for _ in range(5):
    t=time.perf_counter(); work(); best=min(best,time.perf_counter()-t)
f=lambda p: int(open(p).read())//1000
print("cpu%d  %.1f ms  (当时频率 %d / 上限 %d MHz)"%(c,best*1000,f(f"/sys/devices/system/cpu/cpu{c}/cpufreq/scaling_cur_freq"),f(f"/sys/devices/system/cpu/cpu{c}/cpufreq/cpuinfo_max_freq")))
