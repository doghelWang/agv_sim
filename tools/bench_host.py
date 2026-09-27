#!/usr/bin/env python3
"""
主机性能基准 (对比树莓派 / 手机 / x86)：仿真各环节单次耗时，决定哪个节点跑什么

  python3 tools/bench_host.py            # 默认场景 grid_9_square，当前 robot_config.json
  taskset -c 4-7 python3 tools/bench_host.py   # 手机上只用大核对比

输出每项耗时 (ms) 与按实时 (RTF=1) 推算的 CPU 占用:
  python_loop   单核纯 Python 算力        numpy_gemm   多核 BLAS
  mj_step       MuJoCo 物理步 (10 ms 步长，100 Hz)
  lidar2d       全部 2D 激光一次扫描      lidar3d      3D 激光一帧 (如有)
  camera        相机一帧 (射线渲染)       slam_match   内置 SLAM 一次扫描匹配 (10 Hz)
"""
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import numpy as np  # noqa: E402


def t_ms(fn, n=20, warm=2):
    for _ in range(warm):
        fn()
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) * 1000.0 / n


def main():
    from common.hostinfo import core_groups, model
    res = {"host": model(), "cpus": os.cpu_count(), "cores": core_groups(),
           "affinity": sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None}

    def pyloop():
        s = 0
        for i in range(200000):
            s += i * i
    res["python_loop_ms"] = round(t_ms(pyloop, 5), 2)
    a = np.random.rand(400, 400)
    res["numpy_gemm_ms"] = round(t_ms(lambda: a @ a, 10), 2)

    from planning.dijkstra_planner import SCENARIO_DEFINITIONS
    from sim_core.engine import SimCore
    spec = json.load(open(os.path.join(ROOT, "robot_config.json"), encoding="utf-8"))
    for backend in ("mujoco", "kinematic"):
        try:
            c = SimCore(spec, SCENARIO_DEFINITIONS, "grid_9_square", backend=backend, noise=True)
            break
        except Exception as e:  # noqa
            res[f"{backend}_error"] = str(e)[:200]
    res["backend"] = c.backend_name
    c.set_cmd(0.5, 0, 0.1)
    res["mj_step_ms"] = round(t_ms(c.step, 200), 3)
    rs = []
    res["lidar2d_ms"] = round(t_ms(lambda: rs.append([l.scan(c.world, c.x, c.y, c.th) for l in c.lidars]) or rs.pop(), 10), 2)
    res["lidar2d_beams"] = sum(l.n for l in c.lidars)
    if c.lidars3d:
        res["lidar3d_ms"] = round(t_ms(lambda: [l.scan(c.world, c.x, c.y, c.th) for l in c.lidars3d], 5), 2)
        res["lidar3d_points"] = sum(l.n for l in c.lidars3d)
    if c.cameras:
        try:
            res["camera_ms"] = round(t_ms(lambda: c.capture_camera(c.cameras[0]), 3, 1), 1)
            res["camera"] = c.cameras[0].name
        except Exception as e:  # noqa
            res["camera_error"] = str(e)[:200]
    # 内置 SLAM 匹配
    try:
        from nav_runtime.slam import GridMap, build_fields, match
        gm = GridMap(0.025)
        rsl, _ = c.scan_all()
        P = np.concatenate([l.points_base(r) for l, r in zip(c.lidars, rsl)])
        for _ in range(3):
            gm.insert((c.x, c.y, c.th), P[:, 0], P[:, 1], np.ones(len(P), bool))
        t0 = time.perf_counter()
        F = build_fields(gm)
        res["slam_field_ms"] = round((time.perf_counter() - t0) * 1000, 1)
        idx = np.linspace(0, len(P) - 1, min(1200, len(P))).astype(int)
        res["slam_match_ms"] = round(t_ms(lambda: match(F, P[idx, 0], P[idx, 1], (c.x + 0.02, c.y - 0.02, c.th + 0.01)), 10), 2)
    except Exception as e:  # noqa
        res["slam_error"] = str(e)[:200]
    # 按实时推算单核占用 (%)：物理 100 Hz、2D 激光 25 Hz、3D 10 Hz、相机 10 Hz、SLAM 10 Hz
    load = res["mj_step_ms"] * 100 / 10 + res["lidar2d_ms"] * 25 / 10
    load += res.get("lidar3d_ms", 0) * 10 / 10 + res.get("camera_ms", 0) * 10 / 10 + res.get("slam_match_ms", 0) * 10 / 10
    res["est_core_percent_at_rtf1"] = round(load, 1)
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
