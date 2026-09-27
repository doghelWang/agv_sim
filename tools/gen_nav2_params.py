#!/usr/bin/env python3
"""
robot_config.json (cmodel 解析结果) → Nav2 (ROS 2 Humble) 参数文件，按车型各生成一份:

  nav2/nav2_params_diff_drive.yaml    RPP 控制器 (原地转向对准)
  nav2/nav2_params_single_steer.yaml  RPP 控制器 (绕后桥原地转向；角速度按舵轮能力折算)
  nav2/nav2_params_dual_steer.yaml    DWB 全向控制器 (vx/vy/wz 采样)
  其它车型                            RotationShim + Regulated Pure Pursuit (线路跟随，precise_goal_checker ±20 mm)

所有尺寸/限速都取自 cmodel:
  footprint       ← motionCenterAttr (head/tail/left/right offset)，运动中心为原点
  速度/加速度      ← chassisAttr (空载/满载) ∩ 轮端电机能力
  观测源          ← 每个 cmodel 激光单独接入 (含安装高度 → 障碍高度过滤)
"""

import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)

CHASSIS_TYPES = ("diff_drive", "single_steer", "dual_steer")


def _fmt_fp(fp, pad=0.0):
    return "[" + ", ".join(f"[{x + math.copysign(pad, x):.3f}, {y + math.copysign(pad, y):.3f}]" for x, y in fp) + "]"


def effective_limits(spec: dict, chassis_type: str) -> dict:
    """底盘设定 ∩ 轮组能力 → Nav2 可用的速度上限
    执行进程 (独立镜像，不含 sim_core) 使用仿真进程 /api/v1/model 下发的 spec["nav_limits"]"""
    pre = (spec.get("nav_limits") or {}).get(chassis_type)
    if pre:
        return dict(pre)
    from sim_core.kinematics import build_kinematics
    kin, wheels = build_kinematics(spec, chassis_type)
    ch = spec["chassis"]
    v = float(ch.get("max_speed_mps", 1.0))
    w = float(ch.get("max_ang_speed_radps", 1.0))
    caps = [wh.max_speed for wh in kin.wheels if wh.driven]
    if caps:
        v = min(v, min(caps))
    # 原地转向时最远驱动轮的线速度不能超过轮端能力
    rmax = max([math.hypot(wh.x - kin.axle_x if not kin.holonomic else wh.x, wh.y) for wh in kin.wheels if wh.driven] or [0.5])
    if caps and rmax > 1e-3:
        w = min(w, min(caps) / rmax)
    return {
        "v": round(v * 0.95, 3), "w": round(min(w * 0.9, 1.5), 3),
        "a": float(ch.get("max_accel_mps2", 0.5)), "d": float(ch.get("max_decel_mps2", 0.5)),
        "aw": float(ch.get("max_ang_accel_radps2", 1.0)), "dw": float(ch.get("max_ang_decel_radps2", 1.0)),
        "holonomic": kin.holonomic,
    }


def render(spec: dict, chassis_type: str, use_sim_time: bool = False) -> str:
    ch = spec["chassis"]
    fp = ch["footprint"]
    pad = 0.03
    try:   # 保护空间: 静态代价地图外形取 车体 ∪ 带载外形，footprint_padding = 车体净空
        from planning import protection as _pr
        _P = _pr.effective(spec)
        _h, _t, _l, _r = _pr.nav2_outline(ch, _P)
        fp = _pr.rect(_h, _t, _l, _r)
        pad = round(_P["body_margin"], 3)
    except Exception:
        pass
    lim = effective_limits(spec, chassis_type)
    v, w, a, d, aw, dw = lim["v"], lim["w"], lim["a"], lim["d"], lim["aw"], lim["dw"]
    holo = lim["holonomic"]
    vy = v if holo else 0.0
    inscribed = min(ch["left_offset_m"], ch["right_offset_m"], ch["head_offset_m"], ch["tail_offset_m"])
    circum = max(math.hypot(x, y) for x, y in fp)
    infl = round(max(0.55, inscribed + 0.25), 2)
    local_size = max(6.0, round(circum * 2 + 3.0))
    lidars = spec.get("lidars", [])
    sources = " ".join(l["name"] for l in lidars)
    max_z = max([l.get("z", 0.3) for l in lidars] + [2.0]) + 0.5
    ust = "true" if use_sim_time else "false"
    # 全局代价地图的障碍层: Android (proot) 上默认关闭 —— 它在 map 坐标系下用 tf2 MessageFilter 等 slam_toolbox 的
    # map→odom，proot 下这条等待路径会让 planner_server 的 TF 监听卡死。线路跟随 (FollowPath) 不用全局规划，
    # 动态障碍由局部代价地图 (odom 坐标系) 负责。NAV2_GLOBAL_OBSTACLES=1/0 强制开/关。
    _go = os.environ.get("NAV2_GLOBAL_OBSTACLES", "0" if os.path.exists("/system/build.prop") else "1") == "1"
    ctrl_hz = 20.0 if os.path.exists("/system/build.prop") else 30.0
    # RPP 查 map→odom 时最多等待 (s)；0.1 在树莓派上不够 (slam_toolbox 的 map→odom 偶尔落后 0.1~0.4 s → 线路跟随中断)
    rpp_tf_tol = float(os.environ.get("NAV2_TF_TOLERANCE", "0.3"))
    global_plugins = '["static_layer", "obstacle_layer", "inflation_layer"]' if _go else '["static_layer", "inflation_layer"]'

    def src_block(indent):
        s = []
        for l in lidars:
            rng = float(l.get("max_range", 20.0))
            if l.get("type") == "3d":
                # 3D 激光 (Livox Mid-360S): 直接用点云，按车身高度带过滤地面与屋顶
                zmax = float(ch.get("height_m", 2.0)) + 0.15
                s.append(f"{indent}{l['name']}:\n"
                         f"{indent}  topic: {l.get('topic') or '/points/' + l['name']}\n"
                         f"{indent}  data_type: \"PointCloud2\"\n"
                         f"{indent}  marking: true\n"
                         f"{indent}  clearing: true\n"
                         f"{indent}  min_obstacle_height: 0.05\n"
                         f"{indent}  max_obstacle_height: {zmax:.2f}\n"
                         f"{indent}  raytrace_max_range: 12.0\n"
                         f"{indent}  raytrace_min_range: 0.0\n"
                         f"{indent}  obstacle_max_range: 10.0\n"
                         f"{indent}  obstacle_min_range: {float(l.get('min_range', 0.1)):.2f}")
                continue
            s.append(f"{indent}{l['name']}:\n"
                     f"{indent}  topic: /scan/{l['name']}\n"
                     f"{indent}  data_type: \"LaserScan\"\n"
                     f"{indent}  marking: true\n"
                     f"{indent}  clearing: true\n"
                     f"{indent}  inf_is_valid: true\n"
                     f"{indent}  max_obstacle_height: {max_z:.2f}\n"
                     f"{indent}  min_obstacle_height: 0.0\n"
                     f"{indent}  raytrace_max_range: {min(rng, 12.0):.1f}\n"
                     f"{indent}  raytrace_min_range: 0.0\n"
                     f"{indent}  obstacle_max_range: {min(rng, 10.0):.1f}\n"
                     f"{indent}  obstacle_min_range: 0.0")
        return "\n".join(s)

    if chassis_type == "dual_steer":
        follow = f"""    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: false
      min_vel_x: {-v * 0.5:.3f}
      min_vel_y: {-vy:.3f}
      max_vel_x: {v:.3f}
      max_vel_y: {vy:.3f}
      max_vel_theta: {w:.3f}
      min_speed_xy: 0.0
      max_speed_xy: {v:.3f}
      min_speed_theta: 0.0
      acc_lim_x: {a:.3f}
      acc_lim_y: {a:.3f}
      acc_lim_theta: {aw:.3f}
      decel_lim_x: {-d:.3f}
      decel_lim_y: {-d:.3f}
      decel_lim_theta: {-dw:.3f}
      vx_samples: 16
      vy_samples: 10
      vtheta_samples: 16
      sim_time: 1.8
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.10
      trans_stopped_velocity: 0.1
      short_circuit_trajectory_evaluation: true
      stateful: true
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0"""
    else:
        # 精确线路跟随: RotationShim (偏离路径方向时先原地转向) + Regulated Pure Pursuit
        # 执行进程把拓扑路线按拐点切成直线段逐段 FollowPath，终点位姿朝向 = 下一段方向，实现「拐点停车转向、车头与线路一致」
        vr = min(v, 0.6)
        follow = f"""    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      angular_dist_threshold: 0.012
      forward_sampling_distance: 0.3
      rotate_to_heading_angular_vel: {min(w, 0.6):.3f}
      max_angular_accel: {aw:.3f}
      simulate_ahead_time: 1.0
      desired_linear_vel: {vr:.3f}
      lookahead_dist: 0.5
      min_lookahead_dist: 0.35
      max_lookahead_dist: 0.8
      lookahead_time: 1.0
      transform_tolerance: {rpp_tf_tol}
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.03
      approach_velocity_scaling_dist: {vr * vr / max(d * 0.8, 0.05) + 0.1:.2f}
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: false
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: {max(0.9, inscribed * 2):.2f}
      regulated_linear_scaling_min_speed: 0.2
      use_rotate_to_heading: true
      allow_reversing: false
      rotate_to_heading_min_angle: 0.3
      max_robot_pose_search_dist: 10.0"""

    motion_model = "nav2_amcl::OmniMotionModel" if holo else "nav2_amcl::DifferentialMotionModel"
    o = {"x": 0.0, "y": 0.0}
    return f"""# ============================================================================
# Nav2 (Humble) params — auto-generated by tools/gen_nav2_params.py
# cmodel: {spec.get('model_file', '')}   chassis: {chassis_type}   holonomic: {holo}
# footprint (motion center origin): {fp}
# limits: v={v} m/s  w={w} rad/s  a={a}  d={d}  (chassis ∩ wheel-motor capability)
# ============================================================================
amcl:
  ros__parameters:
    use_sim_time: {ust}
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    global_frame_id: "map"
    scan_topic: /scan
    robot_model_type: "{motion_model}"
    alpha1: 0.1
    alpha2: 0.1
    alpha3: 0.1
    alpha4: 0.1
    alpha5: 0.1
    laser_model_type: "likelihood_field"
    laser_max_range: 25.0
    laser_min_range: 0.1
    max_beams: 90
    max_particles: 2000
    min_particles: 300
    update_min_d: 0.15
    update_min_a: 0.15
    resample_interval: 1
    transform_tolerance: 0.5
    tf_broadcast: true
    set_initial_pose: true
    initial_pose: {{x: {o['x']}, y: {o['y']}, z: 0.0, yaw: 0.0}}

bt_navigator:
  ros__parameters:
    use_sim_time: {ust}
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    transform_tolerance: 0.3

controller_server:
  ros__parameters:
    use_sim_time: {ust}
    controller_frequency: {ctrl_hz}
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: {0.001 if holo else 0.5}
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.5
    odom_topic: /odom
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker", "precise_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.3
      movement_time_allowance: 15.0
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      stateful: true
      xy_goal_tolerance: 0.10
      yaw_goal_tolerance: 0.08
    precise_goal_checker:                 # 线路跟随 (FollowPath) 逐段到位: ±20 mm / ±1°
      plugin: "nav2_controller::SimpleGoalChecker"
      stateful: true
      xy_goal_tolerance: 0.02
      yaw_goal_tolerance: 0.017
{follow}

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: {ust}
      update_frequency: 8.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_footprint
      rolling_window: true
      width: {int(local_size)}
      height: {int(local_size)}
      resolution: 0.05
      transform_tolerance: 0.3
      footprint: "{_fmt_fp(fp)}"
      footprint_padding: {pad}
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        max_obstacle_height: {max_z:.2f}
        observation_sources: {sources}
{src_block('        ')}
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.0
        inflation_radius: {infl}
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: {ust}
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      resolution: 0.05
      transform_tolerance: 0.3
      track_unknown_space: true
      footprint: "{_fmt_fp(fp)}"
      footprint_padding: {pad}
      plugins: {global_plugins}
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        max_obstacle_height: {max_z:.2f}
        observation_sources: {sources}
{src_block('        ')}
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: {round(infl + 0.3, 2)}
      always_send_full_costmap: true

map_server:
  ros__parameters:
    use_sim_time: {ust}
    yaml_filename: ""

planner_server:
  ros__parameters:
    use_sim_time: {ust}
    expected_planner_frequency: 10.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.3
      use_astar: true
      allow_unknown: false

smoother_server:
  ros__parameters:
    use_sim_time: {ust}
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    use_sim_time: {ust}
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"
    global_frame: odom
    robot_base_frame: base_footprint
    transform_tolerance: 0.2
    simulate_ahead_time: 2.0
    max_rotational_vel: {min(w, 1.0):.3f}
    min_rotational_vel: 0.2
    rotational_acc_lim: {aw:.3f}

waypoint_follower:
  ros__parameters:
    use_sim_time: {ust}
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200

velocity_smoother:
  ros__parameters:
    use_sim_time: {ust}
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [{v:.3f}, {vy:.3f}, {w:.3f}]
    min_velocity: [{-v * 0.5:.3f}, {-vy:.3f}, {-w:.3f}]
    max_accel: [{a:.3f}, {a if holo else 0.0:.3f}, {aw:.3f}]
    max_decel: [{-d:.3f}, {-d if holo else 0.0:.3f}, {-dw:.3f}]
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
"""


def write_all(spec: dict, out_dir: str, use_sim_time: bool = False):
    os.makedirs(out_dir, exist_ok=True)
    out = []
    for ct in CHASSIS_TYPES:
        p = os.path.join(out_dir, f"nav2_params_{ct}.yaml")
        with open(p, "w", encoding="utf-8") as f:
            f.write(render(spec, ct, use_sim_time))
        out.append(p)
    return out


if __name__ == "__main__":
    cfg = sys.argv[1] if len(sys.argv) > 1 else os.path.join(_ROOT, "robot_config.json")
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(_ROOT, "nav2")
    ust = "--use-sim-time" in sys.argv
    with open(cfg, "r", encoding="utf-8") as f:
        spec = json.load(f)
    for p in write_all(spec, out, ust):
        print("nav2 params:", p)
