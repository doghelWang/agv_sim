cd ~
E=$(tr "\0" "\n" < /proc/$(pgrep -f "nav_runtime.mai[n]" | head -1)/environ | grep -E "^(ROS_DOMAIN_ID|ROS_DISCOVERY_SERVER|FASTRTPS_DEFAULT_PROFILES_FILE|ROS_LOCALHOST_ONLY)=" | sed "s/^/export /" | tr "\n" ";")
timeout 100 proot-distro login ubuntu -- bash -c ". /opt/ros/humble/setup.bash; $E timeout 60 ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap 2>&1 | tail -1 | cut -c1-80"
python3 - <<PY
import json,urllib.request,time,math
N="http://127.0.0.1:8102"; S="http://127.0.0.1:8100"
def call(m,u,b=None):
    r=urllib.request.Request(u,data=None if b is None else json.dumps(b).encode(),method=m,headers={"Content-Type":"application/json"}); return json.loads(urllib.request.urlopen(r,timeout=15).read() or b"{}")
a0=call("GET",S+"/api/v1/snapshot")["state"]["truth"]
gx=0 if abs(a0["x"])>2 else 5
call("POST",N+"/api/v1/missions",{"x":gx,"y":0,"yaw":0}); t=time.time(); st="?"
while time.time()-t<60:
    time.sleep(2); st=call("GET",N+"/api/v1/nav")["status"]
    if st not in ("NAVIGATING","PLANNING"): break
a=call("GET",S+"/api/v1/snapshot")["state"]["truth"]
print("清空局部代价地图后的任务: %s %.0f s, 移动 %.2f m, cpuset %s"%(st,time.time()-t,math.hypot(a["x"]-a0["x"],a["y"]-a0["y"]),open("/proc/self/cpuset").read().strip()))
PY
