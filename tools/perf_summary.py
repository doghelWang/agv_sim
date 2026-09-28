#!/usr/bin/env python3
"""perf_sample.py 结果一行汇总: python3 tools/perf_summary.py a.json [b.json ...]"""
import json, sys, collections
def load(p):
    return json.load(open(p))["summary"]
for p in sys.argv[1:]:
    d = load(p)
    c = d["cpu_percent_by_group"]
    g = lambda k: (c.get(k) or {}).get("avg")
    nav2 = sum((c.get(k) or {}).get("avg", 0) for k in ("controller_server","planner_server","bt_navigator","behavior_server","smoother_server","velocity_smoother","waypoint_follower","lifecycle_manager"))
    av = lambda k: (d.get(k) or {}).get("avg")
    tot = (d.get("cpu_percent_total_procs") or {}).get("avg")
    print(f"{d['label']:<22} host={av('host_cpu_percent')}% procs={tot}% proot={g('proot_tracers')}  sim={g('sim_server')}  nav_rt={g('nav_runtime')}  slam={g('slam_toolbox')}  ekf={g('ekf')}  nav2={nav2:.1f}  web={g('web_gateway')}  rsp={g('robot_state_publisher')}  cppbridge={g('agv_ros_bridge')}  state_hz={av('state_hz')}  step_ms={av('step_ms')}  lidar_ms={av('lidar_ms')}  rtf={av('rtf')}  temp={av('temp_c')}")
