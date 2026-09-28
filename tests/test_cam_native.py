#!/usr/bin/env python3
"""相机着色 C 实现 (sim_core/native/simcore.c sc_cam_shade) 与 numpy (MuJoCoBackend.shade + CameraSensor._rgb) 对比

  需要 MuJoCo 与 libsimcore。python3 tests/test_cam_native.py
默认车型的 RGB / 双目相机，多个位姿无噪声成像逐像素对比，并给出耗时。
"""
import json
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from planning.dijkstra_planner import SCENARIO_DEFINITIONS  # noqa: E402
from sim_core import cameras  # noqa: E402
from sim_core.engine import SimCore  # noqa: E402


def main():
    spec = json.load(open(os.path.join(ROOT, "robot_config.json")))
    sim = SimCore(spec, SCENARIO_DEFINITIONS, "fms_workshop", noise=False)
    cams = [c for c in sim.cameras if c.kind in ("camera", "stereo") and not getattr(c, "use_gl", False)]
    if sim.mj is None or not cams:
        print("SKIP 需要 MuJoCo 与 RGB 相机")
        return True
    ok = True
    tn = tp = 0.0
    poses = [(0.0, 0.0, 0.0), (1.5, -0.5, 1.2), (-2.0, 1.0, 3.0), (3.0, 2.0, -1.6)]
    for c in cams:
        for x, y, th in poses:
            o, dw, dist, gid, nrm = c._cast(sim.world, sim.mj, x, y, th)
            cameras.NATIVE_CAM = True
            t0 = time.perf_counter()
            a = c._rgb(sim.mj, o, dw, dist, gid, nrm, False)
            t1 = time.perf_counter()
            cameras.NATIVE_CAM = False
            b = c._rgb(sim.mj, o, dw, dist, gid, nrm, False)
            tp += time.perf_counter() - t1
            tn += t1 - t0
            d = int(np.abs(a.astype(int) - b.astype(int)).max())
            good = a.shape == b.shape and d == 0
            ok &= good
            print(f"{'PASS' if good else 'FAIL'} {c.name} ({x:.1f},{y:.1f},{th:.1f}) {a.shape}: 最大像素差 {d}"
                  + ("" if good else f"，不同像素 {int((a != b).any(-1).sum())}"))
            cameras.NATIVE_CAM = True
            a2 = c._rgb(sim.mj, o, dw, dist, gid, nrm, True)          # 有噪声: 只检查分布合理
            ok &= abs(float(a2.astype(float).mean()) - float(b.astype(float).mean())) < 1.0
    k = len(cams) * len(poses)
    print(f"     着色+量化: C {tn / k * 1e3:.2f} ms/帧, numpy {tp / k * 1e3:.2f} ms/帧")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
