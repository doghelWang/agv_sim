#!/usr/bin/env python3
"""内置 SLAM 计算内核: C (planning/native/slamcore.c) 与 numpy (nav_runtime/slam.py) 逐项对比

  bash planning/native/build.sh && python3 tests/test_slamcore.py [帧数]
合成房间 (墙 + 箱体) 光线投射 → 同一串带噪声扫描分别用两种实现插入 → 对比 log-odds / 命中密度 / 匹配场 / 扫描匹配结果，
并给出耗时对比。
"""
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from nav_runtime import slam, slam_native  # noqa: E402

LIB = slam_native.lib
SEGS = np.array([(0, 0, 12, 0), (12, 0, 12, 7), (12, 7, 0, 7), (0, 7, 0, 0),            # 房间
                 (4, 2, 5, 2), (5, 2, 5, 3), (5, 3, 4, 3), (4, 3, 4, 2),                  # 箱体
                 (8, 4.5, 9.5, 4.5), (9.5, 4.5, 9.5, 5.2), (9.5, 5.2, 8, 5.2), (8, 5.2, 8, 4.5)], float)


def cast(pose, n=720, rmax=15.0, rng=None):
    x, y, th = pose
    a = th + np.linspace(-math.pi, math.pi, n, endpoint=False)
    dx, dy = np.cos(a)[:, None], np.sin(a)[:, None]
    x1, y1, x2, y2 = (SEGS[:, i][None, :] for i in range(4))
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((x1 - x) * ey - (y1 - y) * ex) / den
        u = ((x1 - x) * dy - (y1 - y) * dx) / den
    t = np.where((np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1), t, np.inf)
    r = t.min(1)
    hit = r < rmax
    if rng is not None:
        r = r + rng.normal(0, 0.01, n)
        hit &= rng.random(n) > 0.03             # 丢点
    r = np.where(hit, r, rmax)
    ab = a - th
    return r * np.cos(ab), r * np.sin(ab), hit


def with_native(on, fn, *a, **k):
    slam_native.lib = LIB if on else None
    try:
        return fn(*a, **k)
    finally:
        slam_native.lib = LIB


def main(frames):
    rng = np.random.default_rng(3)
    poses = [(1.5 + 9 * i / frames, 1.2 + 4.5 * (0.5 + 0.5 * math.sin(i * 0.3)), 0.2 * i) for i in range(frames)]
    scans = [cast(p, rng=rng) for p in poses]
    mounts = [(None, None) if i % 2 else (np.full(720, 0.3), np.full(720, -0.1)) for i in range(frames)]
    ok = True
    gp, gc = slam.GridMap(), slam.GridMap()
    tp = tc = 0.0
    for p, (px, py, hit), (sx, sy) in zip(poses, scans, mounts):
        t0 = time.perf_counter()
        with_native(False, gp.insert, p, px, py, hit, sx, sy)
        t1 = time.perf_counter()
        with_native(True, gc.insert, p, px, py, hit, sx, sy)
        tc += time.perf_counter() - t1
        tp += t1 - t0
    same_shape = gp.L.shape == gc.L.shape and (gp.ox, gp.oy) == (gc.ox, gc.oy)
    dL = float(np.abs(gp.L - gc.L).max()) if same_shape else float("inf")
    dH = float(np.abs(gp.Hd - gc.Hd).max()) if same_shape else float("inf")
    good = same_shape and dL == 0.0 and dH <= 1e-3 * max(1.0, float(gp.Hd.max()))
    ok &= good
    print(f"{'PASS' if good else 'FAIL'} 栅格插入 {frames} 帧: 形状 {gp.L.shape} 一致={same_shape}  |ΔL|max={dL:g}  |ΔHd|max={dH:g}"
          f"  (numpy {tp / frames * 1e3:.2f} ms/帧, C {tc / frames * 1e3:.2f} ms/帧)")

    t0 = time.perf_counter()
    fp = with_native(False, slam.build_fields, gp)
    t1 = time.perf_counter()
    fc = with_native(True, slam.build_fields, gp)
    t2 = time.perf_counter()
    for name in ("fine", "coarse"):
        a, b = fp[name], fc[name]
        sm = a.F.shape == b.F.shape and abs(a.ox - b.ox) < 1e-12 and abs(a.oy - b.oy) < 1e-12 and a.res == b.res
        d = float(np.abs(a.F - b.F).max()) if sm else float("inf")
        good = sm and d < 1e-4
        ok &= good
        print(f"{'PASS' if good else 'FAIL'} 匹配场 {name}: 形状 {a.F.shape} / {b.F.shape}  |ΔF|max={d:g}")
    print(f"     build_fields: numpy {(t1 - t0) * 1e3:.1f} ms, C {(t2 - t1) * 1e3:.1f} ms")

    bad = 0
    tp = tc = 0.0
    for i in range(40):
        truth = (rng.uniform(1.5, 10.5), rng.uniform(1, 6), rng.uniform(-3, 3))
        px, py, hit = cast(truth, rng=rng)
        px, py = px[hit], py[hit]
        init = (truth[0] + rng.normal(0, 0.08), truth[1] + rng.normal(0, 0.08), truth[2] + rng.normal(0, 0.04))
        P0 = np.diag([0.1 ** 2, 0.1 ** 2, 0.05 ** 2]) if i % 2 else None
        t0 = time.perf_counter()
        a = with_native(False, slam.match, fp, px, py, init, P0)
        t1 = time.perf_counter()
        b = with_native(True, slam.match, fp, px, py, init, P0)
        tc += time.perf_counter() - t1
        tp += t1 - t0
        dp = max(abs(a[0][0] - b[0][0]), abs(a[0][1] - b[0][1]), abs(slam._wrap(a[0][2] - b[0][2])))
        dc = float(np.abs(a[1] - b[1]).max() / max(1e-12, np.abs(a[1]).max()))
        di = max(abs(a[2][k] - b[2][k]) / max(1.0, abs(a[2][k])) for k in ("inliers", "score", "eig_min", "eig_max"))
        if not (dp < 1e-7 and dc < 1e-6 and di < 1e-6):
            bad += 1
            if bad <= 3:
                print("  不一致", i, a, "\n        ", b)
    ok &= bad == 0
    print(f"{'PASS' if bad == 0 else 'FAIL'} 扫描匹配: {40 - bad}/40 一致 (位姿 <1e-7, 协方差相对 <1e-6)"
          f"  (numpy {tp / 40 * 1e3:.2f} ms, C {tc / 40 * 1e3:.2f} ms)")
    return ok


if __name__ == "__main__":
    if LIB is None:
        print("SKIP libagvnav/slamcore 未加载:", slam_native.status)
        sys.exit(0)
    sys.exit(0 if main(int(sys.argv[1]) if len(sys.argv) > 1 else 60) else 1)
