#!/usr/bin/env python3
"""GPU 射线求交服务 (sim_core/native/gpucast: gpucastd + 客户端) 与 C 内核 sc_cast_prims 对比

  需要 MuJoCo、libsimcore，以及已经在运行的 gpucastd (默认 127.0.0.1:8068；没有运行则 SKIP)。
  python3 tests/test_gpucast.py [宽 高]       默认 320×240，可传更大分辨率看耗时
两个场景 + 障碍物，多个位姿逐射线对比距离 / 几何体编号 / 法向 (GPU 用 float，允许 1 mm)，并给出耗时。
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
    W, H = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) >= 3 else (320, 240)
    spec = json.load(open(os.path.join(ROOT, "robot_config.json")))
    ok = True
    tg = tc = trg = trc = 0.0
    k = 0
    for scen in ("grid_9_square", "fms_workshop"):
        sim = SimCore(spec, SCENARIO_DEFINITIONS, scen, noise=False)
        mj = sim.mj
        if mj is None or mb._native.lib is None or not hasattr(mb._native.lib, "sc_cast_prims"):
            print("SKIP 需要 MuJoCo 与带 sc_cast_prims 的 libsimcore")
            return True
        if not mj.gpu.available():
            print("SKIP gpucastd 没有运行 (sim_core/native/gpucast/build.sh 编译后启动)")
            return True
        sim.set_obstacles([{"x": 2.0, "y": 1.0, "w": 0.8, "h": 0.6, "yaw": 0.5, "z": 1.0},
                           {"x": -1.5, "y": 2.5, "type": "person", "w": 0.5, "h": 0.5, "z": 1.7}])
        cam = build_camera({"type": "camera", "name": "c", "x": 1.0, "z": 0.5, "width": W, "height": H})
        rng = np.random.default_rng(7)
        gpu = mj.gpu
        for _ in range(6):
            o, dw = cam._pose(rng.uniform(-6, 6), rng.uniform(-6, 6), rng.uniform(-3.14, 3.14))
            mb._gpucast.MIN_RAYS = 0
            mj.gpu = gpu
            c0 = gpu.calls
            mj.cast(o, dw, cam.range_max, True)              # 预热 (首次发场景)
            t0 = time.perf_counter()
            d1, g1, n1 = mj.cast(o, dw, cam.range_max, True)
            t1 = time.perf_counter()
            if gpu.calls != c0 + 2:
                print("FAIL 没有走 GPU 路径")
                return False
            mb._gpucast.MIN_RAYS = 1 << 30
            d0, g0, n0 = mj.cast(o, dw, cam.range_max, True)
            tc += time.perf_counter() - t1
            tg += t1 - t0
            k += 1
            f = np.isfinite(d0) & np.isfinite(d1)
            miss = int((np.isfinite(d0) != np.isfinite(d1)).sum())
            err = np.abs(d0[f] - d1[f])
            par = np.abs(np.abs((n0[f] * n1[f]).sum(1)) - 1.0)
            lim = 5e-4 * len(d0)                              # 棱边 / 量程边界上的射线允许万分之五
            bad_d, bad_g, bad_n = int((err > 1e-3).sum()), int((g0[f] != g1[f]).sum()), int((par > 1e-3).sum())
            good = miss <= lim and bad_d <= lim and bad_g <= lim and bad_n <= lim
            ok &= good
            print(f"{'PASS' if good else 'FAIL'} {scen} 射线 {len(d0)}: 有无回波不同 {miss}，距离差 > 1 mm {bad_d}，"
                  f"几何体不同 {bad_g}，法向不同 {bad_n}，距离差中位数 {float(np.median(err)):.1e} m")
            # 整帧相机: GPU 成像 (无噪声) 与 CPU (C 求交 + sc_cam_shade) 逐像素对比；贴图 8 位上传，允许差 2 个灰度级，棱边像素允许 0.5%
            x, y, th = rng.uniform(-6, 6), rng.uniform(-6, 6), rng.uniform(-3.14, 3.14)
            mb._gpucast.MIN_RAYS = 0
            cam._rgb_gpu(mj, x, y, th, False)                # 预热 (首次上传地面贴图)
            t0 = time.perf_counter()
            a = cam._rgb_gpu(mj, x, y, th, False)
            tr_g = time.perf_counter() - t0
            if a is None:
                print("FAIL GPU 整帧成像没有返回图像")
                return False
            an = cam._rgb_gpu(mj, x, y, th, True)
            mb._gpucast.MIN_RAYS = 1 << 30
            t0 = time.perf_counter()
            o, dw, dist, gid, nrm = cam._cast(sim.world, mj, x, y, th)
            b = cam._rgb(mj, o, dw, dist, gid, nrm, False)
            tr_c = time.perf_counter() - t0
            diff = np.abs(a.astype(int) - b.astype(int)).max(-1)
            badpx = int((diff > 2).sum())
            sd = float((an.astype(float) - a.astype(float))[(a > 8).all(-1) & (a < 247).all(-1)].std())
            good = badpx <= 5e-3 * diff.size and 1.5 < sd < 2.5
            ok &= good
            trg += tr_g; trc += tr_c
            print(f"{'PASS' if good else 'FAIL'} {scen} 整帧 {cam.W}x{cam.H}: 差 > 2 级的像素 {badpx} ({badpx / diff.size * 100:.3f}%)，"
                  f"噪声标准差 {sd:.2f} (设定 2.0)，GPU {tr_g * 1000:.1f} ms / CPU {tr_c * 1000:.1f} ms")
    inf = gpu.info()
    print(f"整帧相机平均: GPU {trg / k * 1000:.2f} ms (其中服务内 {inf['render_avg_gpu_ms']} ms)，CPU {trc / k * 1000:.2f} ms")
    print(f"设备 {inf['device']}  耗时 (每次 {len(d0)} 条射线): GPU 路径 {tg / k * 1000:.2f} ms (其中服务内 {inf['avg_gpu_ms']} ms)，"
          f"C {tc / k * 1000:.2f} ms")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
