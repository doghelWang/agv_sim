#!/usr/bin/env python3
"""
Nav2 (Humble) 自主导航启动文件 —— CModel AGV 仿真专用

  ros2 launch nav2/nav2_launch.py chassis:=single_steer map:=maps/grid_9_square.yaml \
       localization:=ground_truth use_sim_time:=false

chassis      : diff_drive | single_steer | dual_steer  → 选择 nav2/nav2_params_<chassis>.yaml (由 cmodel 生成)
map          : 场景地图 yaml (tools/scenario_to_map.py 生成；运行中可通过 /map_server/load_map 热切换)
localization : slam / ground_truth → map→odom 由执行进程提供 (slam_toolbox + robot_localization，
               或内置 nav_runtime/slam.py)；amcl → 启动 AMCL 用 /scan 定位
map_source   : map_server → 加载场景地图 (map 参数)；topic → 不启动 map_server，代价地图用 slam_toolbox 发布的 /map
initial_x/y/yaw : AMCL 初始位姿

cmd_vel 链路: controller_server → /cmd_vel_nav → velocity_smoother(按 cmodel 加减速) → /cmd_vel → 仿真
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as _RosNode

HERE = os.path.dirname(os.path.abspath(__file__))
ANDROID = os.path.exists("/system/build.prop")
# Android (proot): 关闭 bond 心跳 (每个节点 10 Hz 的心跳在 proot 下占可观 CPU)；其它环境 4 s
BOND_TIMEOUT = float(os.environ.get("NAV2_BOND_TIMEOUT", "0.0" if ANDROID else "4.0"))
ROOT = os.path.dirname(HERE)

def Node(**kw):
    """Android (proot) 下每个 ROS 节点放进独立 proot 会话 (AGV_SPAWNER，见 common/spawn.py)；其它环境即 launch_ros Node"""
    if os.environ.get("AGV_SPAWNER") and os.environ.get("AGV_SPAWN_PER_NODE", "1") == "1" and "prefix" not in kw:
        spawn_py = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common", "spawn.py")
        kw["prefix"] = f"python3 {spawn_py} exec {kw.get('name') or kw.get('executable')} --"
    return _RosNode(**kw)



def _setup(context, *args, **kwargs):
    chassis = LaunchConfiguration("chassis").perform(context)
    map_yaml = LaunchConfiguration("map").perform(context)
    loc = LaunchConfiguration("localization").perform(context)
    ust = LaunchConfiguration("use_sim_time").perform(context).lower() in ("1", "true")
    params = LaunchConfiguration("params_file").perform(context) or os.path.join(HERE, f"nav2_params_{chassis}.yaml")
    if not os.path.isabs(map_yaml):
        map_yaml = os.path.join(ROOT, map_yaml)
    ix = float(LaunchConfiguration("initial_x").perform(context))
    iy = float(LaunchConfiguration("initial_y").perform(context))
    iyaw = float(LaunchConfiguration("initial_yaw").perform(context))

    common = {"use_sim_time": ust}
    log = LaunchConfiguration("log_level").perform(context)
    args = ["--ros-args", "--log-level", log]

    map_source = LaunchConfiguration("map_source").perform(context)
    nodes, loc_nodes = [], []
    if map_source != "topic":
        nodes.append(Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
                          parameters=[params, common, {"yaml_filename": map_yaml}], arguments=args))
        loc_nodes.append("map_server")
    if loc == "amcl":
        nodes.append(Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
                          parameters=[params, common, {"initial_pose": {"x": ix, "y": iy, "z": 0.0, "yaw": iyaw}}],
                          arguments=args))
        loc_nodes.append("amcl")

    # smoother_server / waypoint_follower: 本项目的行为树和接口都不用 (路线由 AgvRoute 生成，任务由执行进程逐个下发)。
    # Android 上默认不启动 —— 每个节点是一个独立 proot 会话 + 一个 DDS 参与者；NAV2_EXTRA_SERVERS=1/0 强制开/关
    extra = os.environ.get("NAV2_EXTRA_SERVERS", "0" if os.path.exists("/system/build.prop") else "1") == "1"
    nav_nodes = ["controller_server"] + (["smoother_server"] if extra else []) + ["planner_server", "behavior_server", "bt_navigator"] \
        + (["waypoint_follower"] if extra else []) + ["velocity_smoother"]
    nodes += [
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[params, common], remappings=[("cmd_vel", "cmd_vel_nav")], arguments=args),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
             parameters=[params, common], arguments=args),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen",
             parameters=[params, common], arguments=args),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen",
             parameters=[params, common], arguments=args),
    ]
    if extra:
        nodes += [
            Node(package="nav2_smoother", executable="smoother_server", name="smoother_server", output="screen",
                 parameters=[params, common], arguments=args),
            Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower", output="screen",
                 parameters=[params, common], arguments=args),
        ]
    nodes += [
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother", output="screen",
             parameters=[params, common], remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
             arguments=args),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=[common, {"autostart": True, "node_names": nav_nodes, "bond_timeout": BOND_TIMEOUT}]),
    ]
    if loc_nodes:
        nodes.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
                          output="screen", parameters=[common, {"autostart": True, "node_names": loc_nodes, "bond_timeout": BOND_TIMEOUT}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("chassis", default_value="single_steer"),
        DeclareLaunchArgument("map", default_value=os.path.join(ROOT, "maps", "grid_9_square.yaml")),
        DeclareLaunchArgument("localization", default_value="slam"),
        DeclareLaunchArgument("map_source", default_value="map_server"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=""),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        OpaqueFunction(function=_setup),
    ])
