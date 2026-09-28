#!/usr/bin/env python3
"""3D 激光 C 实现 (sim_core/native/simcore_rt.c scan3d_one) 与 Python (Lidar3DSensor.scan + slice_to_scan) 对比

  需要 MuJoCo 与 libsimcore (带 MuJoCo 头文件编译)。python3 tests/test_rt_lidar3d.py
无噪声: 同一位姿、同一帧序号下点数、点坐标、线号、帧内时间、高度带切片、融合扫描逐项对比；另给出两种实现的耗时。
"""
import json
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from planning.dijkstra_planner import SCENARIO_DEFINITIONS  # noqa: E402
from sim_core import sensors  # noqa: E402
from sim_core.engine import SimCore  # noqa: E402


def main():
    spec = json.load(open(os.path.join(ROOT, "robot_config.json")))
    if not any(l.get("type") == "3d" for l in spec.get("lidars", [])):
        print("SKIP robot_config.json 没有 3D 激光")
        return True
    sim = SimCore(spec, SCENARIO_DEFINITIONS, "grid_9_square", noise=False)
    if sim.mj is None or not sim.enable_rt():
        print("SKIP 需要 MuJoCo + libsimcore 实时循环")
        return True
    rt = sim.rt
    with rt.hold():
        pass
    if not rt.lidars_on or not rt._l3d_bufs:
        print("FAIL 3D 激光未进入 C 实时循环")
        return False
    ok = True
    poses = [(0.0, 0.0, 0.0), (1.3, -0.4, 0.7), (-2.0, 1.5, -2.4), (3.1, 2.2, 3.0), (0.4, -3.0, 1.57)]
    tc = tp = 0.0
    for k, (x, y, th) in enumerate(poses):
        sim.reset_pose(x, y, th)
        frame0 = [l.frame for l in sim.lidars3d]
        # 让 3D 到期: 步进直到出新帧
        seq0 = rt.read_lidar3d(0, 0)
        seq0 = seq0[0] if seq0 else 0
        t0 = time.perf_counter()
        r = None
        for _ in range(200):
            with rt.hold(sync=False):
                rt.lib.sc_rt_step_n(rt.h, 1)
                r = rt.read_lidar3d(0, seq0)
            if r is not None:
                break
        tc += time.perf_counter() - t0
        if r is None:
            print("FAIL C 侧没有产生 3D 帧")
            return False
        cseq, t, (px, py, pth), cl, sl = r
        l3 = sim.lidars3d[0]
        l3.frame = sim.lidars3d[0].frame - 1           # read_lidar3d 已把 frame 同步为扫描后的值
        t1 = time.perf_counter()
        pc = l3.scan(sim.world, px, py, pth, noise=False)
        ps = sensors.slice_to_scan(l3.points_base(pc), sim.slice_zmin, sim.slice_zmax, sim.merged_bins, sim.merged_range)
        tp += time.perf_counter() - t1
        n_ok = len(pc["points"]) == len(cl["points"])
        dp = float(np.abs(pc["points"] - cl["points"]).max()) if n_ok and len(pc["points"]) else (0.0 if n_ok else float("inf"))
        same_line = n_ok and np.array_equal(pc["line"], cl["line"])
        dt_ = float(np.abs(pc["offset_time"] - cl["offset_time"]).max()) if n_ok and len(pc["points"]) else 0.0
        fin = np.isfinite(ps) | np.isfinite(sl)
        with np.errstate(invalid="ignore"):
            ds = float(np.nanmax(np.abs(np.where(fin, ps - sl, 0.0)))) if fin.any() else 0.0
        same_inf = np.array_equal(np.isfinite(ps), np.isfinite(sl))
        good = n_ok and dp < 1e-5 and same_line and dt_ < 1e-12 and same_inf and ds < 1e-6
        ok &= good
        print(f"{'PASS' if good else 'FAIL'} 位姿 {k} ({px:.2f},{py:.2f},{math.degrees(pth):.0f}°) 帧 {frame0[0]}: 点数 C {len(cl['points'])} / Py {len(pc['points'])}"
              f"  |Δp|max={dp:.2e}  线号一致={same_line}  |Δt|={dt_:.1e}  切片 inf 一致={same_inf} |Δ|={ds:.1e}")
    print(f"     C 实时循环 (含 2D 与步进) {tc / len(poses) * 1e3:.1f} ms/帧，Python 3D 扫描+切片 {tp / len(poses) * 1e3:.1f} ms/帧 "
          f"(每帧 {sim.lidars3d[0].n} 射线)")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
