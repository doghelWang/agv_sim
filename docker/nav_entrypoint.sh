#!/bin/bash
# agv-nav 入口: ROS 2 环境 → 等待仿真进程 REST 就绪 → 执行进程 (:NAV_API_PORT)
# 容器内 AGV_HOME=/opt/agv；节点代理的 process 运行时会把 AGV_HOME 指向源码目录
set -e
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
cd "${AGV_HOME:-/opt/agv}"
# 本仓库 C++ 包 (agv_ros_bridge、agv_nav2_plugins): 加入 ament 索引，Nav2 才能按名字加载插件
[ -f ros2/install/local_setup.bash ] && source ros2/install/local_setup.bash
PY="${PYTHON:-python3}"
# 数学库线程: 仿真/执行进程自己已有多线程，OpenBLAS/OpenMP 默认按核数开线程会互相抢核 (OPTIMIZATION R2)
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}" OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}" MKL_NUM_THREADS="${MKL_NUM_THREADS:-1}"
SIM_API="${SIM_API:-http://127.0.0.1:8090}"
export SIM_API
if [ -f /system/build.prop ] && [ -z "$FASTRTPS_DEFAULT_PROFILES_FILE$CYCLONEDDS_URI" ]; then
    # Android (proot): 回环网卡不支持组播，ROS_LOCALHOST_ONLY=1 时 10+ 个 ROS 进程互相发现不全；
    # 改用 DDS 配置文件: 只走 127.0.0.1，单播发现覆盖 120 个参与者 (deploy/android/*_localhost.xml)
    # 默认 Fast DDS；AGV_ANDROID_RMW=cyclonedds 试用 Cyclone DDS (需 ros-humble-rmw-cyclonedds-cpp；
    # 2026-09 在 Pixel 4 上试过一次，执行进程启动时卡住，尚未查明)
    export ROS_LOCALHOST_ONLY=0
    D="${AGV_HOME:-/opt/agv}/deploy/android"
    if [ "${AGV_ANDROID_RMW:-fastrtps}" = cyclonedds ] && [ -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]; then
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$D/cyclonedds_localhost.xml"
    elif [ "${AGV_DDS_DISCOVERY:-server}" = server ] && [ -x /opt/ros/humble/bin/fast-discovery-server ]; then
        # 发现服务器 (默认): 每个实例一个，端口 11811 + ROS_DOMAIN_ID；执行进程 (nav_runtime.main) 负责启动/停止它。
        # 代替"向 120 个端口盲发探测包" (见 fastdds_ds.xml 开头的说明)；AGV_DDS_DISCOVERY=peers 退回旧方式
        export AGV_DDS_SERVER_PORT="${AGV_DDS_SERVER_PORT:-$((11811 + ${ROS_DOMAIN_ID:-0}))}"
        export ROS_DISCOVERY_SERVER="127.0.0.1:$AGV_DDS_SERVER_PORT"
        export FASTRTPS_DEFAULT_PROFILES_FILE="$D/fastdds_ds.xml"
    else
        export FASTRTPS_DEFAULT_PROFILES_FILE="$D/fastdds_localhost.xml"
    fi
else
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"   # 避免局域网内其它设备的 ROS 节点串话
fi
echo "=========================================================="
echo " agv-nav ${INSTANCE_ID:+实例 $INSTANCE_ID }执行进程 :${NAV_API_PORT:-8091}   仿真进程 SIM_API=$SIM_API"
echo " ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY  RMW=${RMW_IMPLEMENTATION:-默认}${FASTRTPS_DEFAULT_PROFILES_FILE:+  DDS 配置 $FASTRTPS_DEFAULT_PROFILES_FILE}${CYCLONEDDS_URI:+  DDS 配置 $CYCLONEDDS_URI}${ROS_DISCOVERY_SERVER:+  发现服务器 $ROS_DISCOVERY_SERVER}  NAV_USE_ROS=${NAV_USE_ROS:-1}  NAV2_AUTOSTART=${NAV2_AUTOSTART:-1}"
echo "=========================================================="
until $PY -c "import urllib.request,sys; urllib.request.urlopen('$SIM_API/api/v1/health', timeout=2)" >/dev/null 2>&1; do
    echo "[agv-nav] 等待仿真进程 $SIM_API ..."; sleep 2
done
exec $PY -m nav_runtime.main
