#!/bin/bash
# Sourced by the Rust nav-runtime before launching agv_ros_bridge / ros2 launch (Nav2 backend only).
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
H="${AGV_HOME:-/opt/agv}"
[ -f "$H/ros2/install/local_setup.bash" ] && source "$H/ros2/install/local_setup.bash"
if [ -f /system/build.prop ]; then
    export ROS_LOCALHOST_ONLY=0
    [ -z "$FASTRTPS_DEFAULT_PROFILES_FILE$CYCLONEDDS_URI" ] && export FASTRTPS_DEFAULT_PROFILES_FILE="$H/deploy/android/fastdds_localhost.xml"
else
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
fi
