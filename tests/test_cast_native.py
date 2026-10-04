#!/usr/bin/env python3
"""相机/3D 射线求交 C 实现 (sim_core/native/simcore.c sc_cast_prims) 与 MuJoCo mj_multiRay 对比

  需要 MuJoCo 与 libsimcore。python3 tests/test_cast_native.py
两个场景 + 障碍物 (旋转的箱子、行人圆柱)，多个位姿逐射线对比距离 / 几何体编号 / 法向，并给出耗时。
"""
import json
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from planning.dijkstra_planner import SCENARIO_DEFINITIONS  # noqa: E402
from sim_core import mujoco_backend as mb  # noqa: E402
from sim_core.cameras import build_camera  # noqa: E402
from sim_core.engine import SimCore  # noqa: E402


def main():
    spec = json.load(open(os.path.join(ROOT, "robot_config.json")))
    ok = True
    tn = tm = 0.0
    k = 0
    for scen in ("grid_9_square", "fms_workshop"):
        sim = SimCore(spec, SCENARIO_DEFINITIONS, scen, noise=False)
        mj = sim.mj
        if mj is None or mb._native.lib is None or not hasattr(mb._native.lib, "sc_cast_prims"):
            print("SKIP 需要 MuJoCo 与带 sc_cast_prims 的 libsimcore")
            return True
        sim.set_obstacles([{"x": 2.0, "y": 1.0, "w": 0.8, "h": 0.6, "yaw": 0.5, "z": 1.0},
                           {"x": -1.5, "y": 2.5, "type": "person", "w": 0.5, "h": 0.5, "z": 1.7}])
        cam = build_camera({"type": "camera", "name": "c", "x": 1.0, "z": 0.5})
        rng = np.random.default_rng(7)
        for _ in range(6):
            o, dw = cam._pose(rng.uniform(-6, 6), rng.uniform(-6, 6), rng.uniform(-3.14, 3.14))
            mb.NATIVE_CAST = True
            t0 = time.perf_counter()
            d1, g1, n1 = mj.cast(o, dw, cam.range_max, True)
            t1 = time.perf_counter()
            mb.NATIVE_CAST = False
            d0, g0, n0 = mj.cast(o, dw, cam.range_max, True)
            tm += time.perf_counter() - t1
            tn += t1 - t0
            mb.NATIVE_CAST = True
            k += 1
            f = np.isfinite(d0) & np.isfinite(d1)
            miss = int((np.isfinite(d0) != np.isfinite(d1)).sum())
            err = np.abs(d0[f] - d1[f])
            par = np.abs(np.abs((n0[f] * n1[f]).sum(1)) - 1.0)
            # 棱边上的射线两种实现可能选到相邻的面: 允许万分之五
            lim = 5e-4 * len(d0)
            good = miss <= lim and int((err > 1e-3).sum()) <= lim and int((g0[f] != g1[f]).sum()) <= lim and int((par > 1e-3).sum()) <= lim
            ok &= good
            print(f"{'PASS' if good else 'FAIL'} {scen} 射线 {len(d0)}: 有无回波不同 {miss}，距离差 > 1 mm {int((err > 1e-3).sum())}，"
                  f"几何体不同 {int((g0[f] != g1[f]).sum())}，法向不同 {int((par > 1e-3).sum())}，最大距离差 {float(err.max()):.2e} m")
    print(f"耗时 (每次 {len(d0)} 条射线): C {tn / k * 1000:.2f} ms，mj_multiRay {tm / k * 1000:.2f} ms")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
