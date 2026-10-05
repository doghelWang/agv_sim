#!/usr/bin/env python3
"""SLAM 定位 + 里程计融合 离线单测 (运动学后端，无需 MuJoCo/ROS)

  python3 tests/test_slam.py
"""
import json
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402
from planning.dijkstra_planner import SCENARIO_DEFINITIONS  # noqa: E402
from sim_core.engine import SimCore  # noqa: E402
from nav_runtime.slam import SlamLocalizer, _wrap, compose, inverse  # noqa: E402


def _spec():
    with open(os.path.join(ROOT, "robot_config.json"), encoding="utf-8") as f:
        return json.load(f)


def _drive(c, S, wps, tmax=120.0):
    """真值闭环驱动 (只为让车动起来)，定位全部来自 SLAM；返回每步定位误差 (m, rad)"""
    errs, wi, k, phase = [], 0, 0, "rot"
    while wi < len(wps) and c.t < tmax:
        gx, gy = wps[wi]
        dx, dy = gx - c.x, gy - c.y
        d = math.hypot(dx, dy)
        if d < 0.03:
            wi, phase = wi + 1, "rot"
            c.set_cmd(0, 0, 0)
        else:
            e = _wrap(math.atan2(dy, dx) - c.th)
            if phase == "rot":
                if abs(e) < 0.01:
                    phase = "go"
                c.set_cmd(0, 0, 0 if phase == "go" else max(-0.4, min(0.4, 1.5 * e)))
            else:
                c.set_cmd(min(1.0, 0.6 * d + 0.05), 0, 2.0 * e)
        c.step()
        k += 1
        if k % 2 == 0:
            p = S.on_odom(c.t, (c.odom.x, c.odom.y, c.odom.th), (c.x, c.y, c.th), (c.odom.vx, c.odom.vy, c.odom.wz))
            errs.append((math.hypot(p[0] - c.x, p[1] - c.y), _wrap(p[2] - c.th)))
        if k % 7 == 0:
            rsl, _ = c.scan_all()
            P = np.concatenate([l.points_base(r) for l, r in zip(c.lidars, rsl)])
            SX = np.concatenate([np.full(int(np.isfinite(r).sum()), l.mx) for l, r in zip(c.lidars, rsl)])
            SY = np.concatenate([np.full(int(np.isfinite(r).sum()), l.my) for l, r in zip(c.lidars, rsl)])
            S.on_points(c.t, P[:, 0], P[:, 1], None, SX, SY)
    return np.array(errs)


def test_se2():
    a, b = (1.0, 2.0, 0.7), (-0.3, 0.5, -1.2)
    c = compose(inverse(a), compose(a, b))
    assert max(abs(c[0] - b[0]), abs(c[1] - b[1]), abs(_wrap(c[2] - b[2]))) < 1e-9


def test_slam_mapping_and_localization():
    tmp = tempfile.mkdtemp()
    spec = _spec()
    ch = spec["chassis"]
    c = SimCore(spec, SCENARIO_DEFINITIONS, "grid_9_square", backend="kinematic", noise=True)
    S = SlamLocalizer(map_dir=tmp, log=lambda *a: None)
    S.on_scenario("grid_9_square")
    S.set_body(ch.get("head_offset_m", 0.6), ch.get("tail_offset_m", 0.6), max(ch.get("left_offset_m", 0.4), ch.get("right_offset_m", 0.4)))
    assert S.mode == "slam"
    e = _drive(c, S, [[0, 5], [0, 0], [5, 0], [0, 0]], tmax=90)
    ex = e[:, 0] * 1000
    print(f"  slam: 定位误差 mean {ex.mean():.1f} / p95 {np.percentile(ex, 95):.1f} / max {ex.max():.1f} mm, 航向 max {np.degrees(np.abs(e[:, 1])).max():.3f}°")
    assert ex.max() < 20.0 and np.percentile(ex, 95) < 12.0
    st = S.status()
    assert st["stats"]["matches"] > 50 and st["map"]["updates"] > 5
    # 保存 → 新定位器加载 → localization 模式
    out = S.save()
    assert os.path.exists(out["npz"]) and open(out["pgm"], "rb").read(2) == b"P5"
    S2 = SlamLocalizer(map_dir=tmp, log=lambda *a: None)
    S2.on_scenario("grid_9_square")
    assert S2.mode == "localization" and S2.fields is not None
    S2.set_body(*S.body)
    e2 = _drive(c, S2, [[0, -5], [0, 0]], tmax=c.t + 40)
    ex2 = e2[:, 0] * 1000
    print(f"  localization: 定位误差 mean {ex2.mean():.1f} / max {ex2.max():.1f} mm")
    assert ex2.max() < 20.0
    assert S2.gm.updates == 0            # 定位模式不改地图


def test_odom_only_drifts():
    """纯里程计会漂移，SLAM 的意义所在 (确认仿真的里程计误差确实存在)"""
    spec = _spec()
    c = SimCore(spec, SCENARIO_DEFINITIONS, "grid_9_square", backend="kinematic", noise=True)
    S = SlamLocalizer(map_dir=tempfile.mkdtemp(), mode="odom", log=lambda *a: None)
    S.on_scenario("grid_9_square")
    e = _drive(c, S, [[0, 5], [0, 0], [5, 0], [0, 0]], tmax=90)
    print(f"  odom: 终点误差 {e[-1, 0] * 1000:.1f} mm, 最大 {e[:, 0].max() * 1000:.1f} mm")
    assert e[:, 0].max() > 0.005


def test_auto_freeze_unfreeze():
    """地图收敛自动冻结 (Android 默认开): 冻结条件与解冻条件"""
    s = SlamLocalizer(map_dir=tempfile.mkdtemp(), log=lambda *a: None)
    s._fz_on, s._fz_travel, s.scenario = True, 5.0, "t"
    s.gm.ensure(-2, -2, 2, 2)
    s.gm.L[:] = 1.0
    s._fz_ins = 4; s._auto_freeze()                 # 第一次统计: 记下基准，不冻结
    assert s.mode == "slam" and s._fz_known > 0
    s._fz_dist = 6.0; s._fz_ins = 4; s._auto_freeze()   # 行驶 6 m 没有新增 → 保存并冻结
    assert s.mode == "localization" and s._fz_auto and s.has_saved("t")
    s._auto_unfreeze(0.0, {"inliers": 0.4}); s._auto_unfreeze(2.0, {"inliers": 0.5})
    s._auto_unfreeze(2.5, {"inliers": 0.9}); s._auto_unfreeze(3.0, {"inliers": 0.4}); s._auto_unfreeze(5.9, {"inliers": 0.4})
    assert s.mode == "localization"                 # 中间恢复过，重新计时
    s._auto_unfreeze(6.1, {"inliers": 0.4})
    assert s.mode == "slam" and not s._fz_auto


if __name__ == "__main__":
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as ex:
                fails += 1
                print(f"FAIL {name} {ex}")
    sys.exit(1 if fails else 0)
