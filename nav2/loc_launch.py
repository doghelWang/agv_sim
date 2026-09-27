#!/usr/bin/env python3
"""
定位栈启动文件 (开源方案)：robot_localization EKF + slam_toolbox

  ros2 launch nav2/loc_launch.py mode:=mapping
  ros2 launch nav2/loc_launch.py mode:=localization map_file:=/data/slam_maps/fms_workshop start_x:=0 start_y:=0 start_yaw:=0

  ekf_filter_node (robot_localization)  /odom (轮式里程计速度) + /imu (角速度) → TF odom→base_footprint、/odometry/filtered
  slam_toolbox                          /scan (base_link 360° 合并扫描) + TF → TF map→odom、/map
      mode=mapping       async_slam_toolbox_node        边建图边定位 (Karto 扫描匹配 + 位姿图回环)
      mode=localization  localization_slam_toolbox_node 在已保存的位姿图上定位 (map_file 不含扩展名)

由执行进程 nav_runtime/ros_slam.py 监管 (场景切换、保存地图、模式切换时重启)。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as _RosNode

HERE = os.path.dirname(os.path.abspath(__file__))

def Node(**kw):
    """Android (proot) 下每个 ROS 节点放进独立 proot 会话 (AGV_SPAWNER，见 common/spawn.py)；其它环境即 launch_ros Node"""
    if os.environ.get("AGV_SPAWNER") and os.environ.get("AGV_SPAWN_PER_NODE", "1") == "1" and "prefix" not in kw:
        spawn_py = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common", "spawn.py")
        kw["prefix"] = f"python3 {spawn_py} exec {kw.get('name') or kw.get('executable')} --"
    return _RosNode(**kw)



def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    ust = LaunchConfiguration("use_sim_time").perform(context).lower() in ("1", "true")
    common = {"use_sim_time": ust}
    log = ["--ros-args", "--log-level", LaunchConfiguration("log_level").perform(context)]
    nodes = []
    if LaunchConfiguration("ekf").perform(context).lower() in ("1", "true"):   # false: 执行进程发布 odom→base_footprint
        nodes.append(Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
                          parameters=[os.path.join(HERE, "ekf.yaml"), common], arguments=log))
    st = [os.path.join(HERE, "slam_toolbox.yaml"), common]
    # map→odom 的时间戳 = 最近处理的那帧扫描时刻 + transform_timeout。slam_toolbox 处理扫描偶尔卡顿 (树莓派实测
    # 最多落后 0.37 s)，预留不够时 Nav2 控制器查 map 系位姿会"外推到未来"失败，连续失败 → FollowPath ABORTED
    # ("Nav2 线路跟随中断")。Android (proot) 扫描匹配滞后更大，预留 0.8 s；其它平台 0.5 s。
    android = os.path.exists("/system/build.prop")
    st.append({"transform_timeout": float(os.environ.get("SLAM_TRANSFORM_TIMEOUT", "0.8" if android else "0.5"))})
    if android:
        # map→odom 50 Hz → 20 Hz：/tf 要扇出给十来个节点，proot 下每条都是被追踪的系统调用
        st.append({"transform_publish_period": float(os.environ.get("SLAM_TF_PERIOD", "0.05"))})
    if mode == "localization":
        sx, sy, syaw = (float(LaunchConfiguration(k).perform(context)) for k in ("start_x", "start_y", "start_yaw"))
        st.append({"mode": "localization", "map_file_name": LaunchConfiguration("map_file").perform(context),
                   "map_start_pose": [sx, sy, syaw]})
        nodes.append(Node(package="slam_toolbox", executable="localization_slam_toolbox_node", name="slam_toolbox",
                          output="screen", parameters=st, arguments=log))
    else:
        st.append({"mode": "mapping"})
        nodes.append(Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
                          output="screen", parameters=st, arguments=log))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="mapping"),
        DeclareLaunchArgument("ekf", default_value="true"),
        DeclareLaunchArgument("map_file", default_value=""),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        OpaqueFunction(function=_setup),
    ])
