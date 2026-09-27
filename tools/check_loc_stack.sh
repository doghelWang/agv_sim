#!/bin/bash
# ============================================================================
# 检查执行容器里的开源定位 / 导航栈 (slam_toolbox + robot_localization + Nav2)
#
#   bash tools/check_loc_stack.sh                 # 单机部署 (容器 agv-nav，执行端口 8091)
#   bash tools/check_loc_stack.sh agv-nav-i01 8101  # 平台部署的实例 (容器名 / 执行端口见实例详情)
# 把完整输出发给开发者即可定位问题。
# ============================================================================
C="${1:-agv-nav}"; PORT="${2:-8091}"
echo "== 容器 $C"
docker ps --format '{{.Names}}  {{.Status}}' | grep -E "^$C\b" || { echo "容器 $C 未运行"; docker ps --format '{{.Names}}'; exit 1; }
docker exec "$C" bash -c '
source /opt/ros/humble/setup.bash 2>/dev/null
echo "== 软件包"
for p in slam_toolbox robot_localization nav2_rotation_shim_controller nav2_regulated_pure_pursuit_controller nav2_controller; do
  ros2 pkg prefix $p >/dev/null 2>&1 && echo "  ✔ $p" || echo "  ✘ $p (未安装：请重新构建 agv-nav 镜像)"
done
echo "== 节点"; timeout 8 ros2 node list 2>/dev/null | sed "s/^/  /"
echo "== 话题频率 (各 4 s)"
for t in /odom /imu /scan /odometry/filtered /map; do
  printf "  %-20s " $t; timeout 4 ros2 topic hz $t 2>/dev/null | grep -m1 "average rate" || echo "无数据"
done
echo "== TF map→base_footprint"; timeout 4 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | grep -m2 -E "Translation|RPY \(degree\)" | sed "s/^/  /"
echo "== TF odom→base_footprint"; timeout 4 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | grep -m1 "Translation" | sed "s/^/  /"
echo "== Nav2 action"; timeout 6 ros2 action list 2>/dev/null | grep -E "follow_path|navigate" | sed "s/^/  /"
echo "== 已保存的 SLAM 地图"; ls -la /data/slam_maps 2>/dev/null | sed "s/^/  /"
'
echo "== 执行进程定位状态"
curl -s "http://127.0.0.1:$PORT/api/v1/slam" | python3 -c "import json,sys; d=json.load(sys.stdin); d.pop('stats',None); print(json.dumps(d, ensure_ascii=False, indent=1))" 2>/dev/null || echo "  执行进程 :$PORT 无响应"
echo "== 规划器 / Nav2"
curl -s "http://127.0.0.1:$PORT/api/v1/nav" | python3 -c "import json,sys; d=json.load(sys.stdin); print('  planner', d.get('planner'), ' nav2', d.get('nav2'))" 2>/dev/null
