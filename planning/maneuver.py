#!/usr/bin/env python3
"""
车体机动可行性 (规划器与执行进程共用)

车体为矩形: 机体系 x ∈ [-tail, +head]，y ∈ [-hw, +hw]，原点为控制点 (后轴/转向中心)。
长车头车型原地转向扫掠半径 = hypot(head, hw)，在设备岛/货架旁的拓扑拐点容易扫到障碍，
因此拐点过弯要在「圆弧过弯 (多种半径)」与「原地转向 (两个方向)」之间选车体净空足够的方式；
规划器用同一套检查给「拐点不可通过」的转移加惩罚，从而换走能转得过去的路线。
"""

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

CLEAR_MIN = 0.05        # 过弯车体净空要求 (m)
SWEEP_STEP = 0.01       # 扫掠取位姿: 车体上任一点相邻两位姿之间移动 ≤ 1 cm → 采样间漏检的净空 ≤ 5 mm


def sweep_samples(max_travel: float, min_n: int) -> int:
    """扫掠 (原地转向/圆弧) 的位姿段数 (与 planning/native/agvnav.c an_sweep_samples 相同)"""
    return max(min_n, int(math.ceil(max_travel / SWEEP_STEP)))


def box_segments(obstacles: Sequence[dict]) -> List[tuple]:
    segs = []
    for o in obstacles or []:
        ox, oy = float(o.get("x", 0)), float(o.get("y", 0))
        hw, hh = float(o.get("w", .8)) / 2, float(o.get("h", .8)) / 2
        segs += [(ox - hw, oy - hh, ox + hw, oy - hh), (ox + hw, oy - hh, ox + hw, oy + hh),
                 (ox + hw, oy + hh, ox - hw, oy + hh), (ox - hw, oy + hh, ox - hw, oy - hh)]
    return segs


def clearance(segs: Sequence[tuple], poses: Sequence[Tuple[float, float, float]], head: float, tail: float, hw: float) -> float:
    """车体在一组位姿上到线段集合的最小距离 (m)；线段端点落入车体内返回 -1 (有 libagvnav 时用 C 实现)"""
    from planning import native
    if native.lib is not None:
        return native.clearance(segs, poses, head, tail, hw) if segs and poses else 9.0
    return _clearance_py(segs, poses, head, tail, hw)


def _clearance_py(segs, poses, head, tail, hw) -> float:
    """与 planning/native/agvnav.c an_clearance 同一算法 (无 C 库时的兜底)：车体系下线段 vs 矩形的精确距离。
    线段端点落入车体 → -1；线段穿过车体 → 0；位姿包围盒 ±R 内没有线段 → 9.0"""
    if not segs or not poses:
        return 9.0
    S = np.asarray([w[:4] for w in segs], float)
    xs = np.array([p[0] for p in poses]); ys = np.array([p[1] for p in poses])
    R = math.hypot(max(head, tail), hw) + 0.3
    near = ((np.minimum(S[:, 0], S[:, 2]) < xs.max() + R) & (np.maximum(S[:, 0], S[:, 2]) > xs.min() - R) &
            (np.minimum(S[:, 1], S[:, 3]) < ys.max() + R) & (np.maximum(S[:, 1], S[:, 3]) > ys.min() - R))
    if not near.any():
        return 9.0
    S = S[near]
    best, crossing = np.inf, False

    def pt_seg2(px, py, ax, ay, bx, by):
        dx, dy = bx - ax, by - ay
        L2 = np.maximum(dx * dx + dy * dy, 1e-12)
        u = np.clip(((px - ax) * dx + (py - ay) * dy) / L2, 0.0, 1.0)
        return (ax + u * dx - px) ** 2 + (ay + u * dy - py) ** 2

    def pt_box2(x, y):
        ddx = np.where(x < -tail, -tail - x, np.where(x > head, x - head, 0.0))
        ddy = np.maximum(np.abs(y) - hw, 0.0)
        return ddx * ddx + ddy * ddy

    for x, y, yaw in poses:
        c, s = math.cos(yaw), math.sin(yaw)
        ax = (S[:, 0] - x) * c + (S[:, 1] - y) * s; ay = -(S[:, 0] - x) * s + (S[:, 1] - y) * c
        bx = (S[:, 2] - x) * c + (S[:, 3] - y) * s; by = -(S[:, 2] - x) * s + (S[:, 3] - y) * c
        if np.any(((ax > -tail) & (ax < head) & (np.abs(ay) < hw)) | ((bx > -tail) & (bx < head) & (np.abs(by) < hw))):
            return -1.0
        # Liang–Barsky: 线段与闭矩形相交
        dx, dy = bx - ax, by - ay
        t0, t1, ok = np.zeros(len(S)), np.ones(len(S)), np.ones(len(S), bool)
        for p, q in ((-dx, ax + tail), (dx, head - ax), (-dy, ay + hw), (dy, hw - ay)):
            par = np.abs(p) < 1e-15
            ok &= ~(par & (q < 0))
            with np.errstate(divide="ignore", invalid="ignore"):
                r = np.where(par, 0.0, q / np.where(par, 1.0, p))
            neg, pos = ~par & (p < 0), ~par & (p > 0)
            t0 = np.where(neg, np.maximum(t0, r), t0)
            t1 = np.where(pos, np.minimum(t1, r), t1)
        if np.any(ok & (t0 <= t1)):
            crossing = True
            continue
        d2 = np.minimum(pt_box2(ax, ay), pt_box2(bx, by))
        for cx, cy in ((-tail, -hw), (head, -hw), (head, hw), (-tail, hw)):
            d2 = np.minimum(d2, pt_seg2(cx, cy, ax, ay, bx, by))
        best = min(best, float(d2.min()))
    return 0.0 if crossing else math.sqrt(best)


def plan_corner(segs, node, h1: float, h2: float, len_in: float, len_out: float,
                head: float, tail: float, hw: float, r_pref: float, clear_min: float = CLEAR_MIN,
                allow_arcs: bool = False, mode: Optional[str] = None) -> Tuple[dict, float]:
    """拐点过弯方式 (默认只允许拐点原地转向 —— 车辆严格沿拓扑线路行驶；allow_arcs=True 时先尝试圆弧): 圆弧 (1.25/1/0.7/0.45 倍 r_pref，受路段长度限制) 中净空 ≥ CLEAR_MIN 且最大的；
    圆弧都不行再试拐点原地转向 (最短方向、反方向)；都不满足返回净空最大的。
    返回 ({"heading","turn","R","d","v"} 圆弧 | {"rotate": ±1} 原地转向, 净空)。有 libagvnav 时用 C 实现"""
    mode = mode or ("arc" if allow_arcs else "rotate")
    from planning import native
    if native.lib is not None:
        return native.plan_corner(segs, node, h1, h2, len_in, len_out, head, tail, hw, r_pref, clear_min, mode)
    return _plan_corner_py(segs, node, h1, h2, len_in, len_out, head, tail, hw, r_pref, clear_min, mode)


def _plan_corner_py(segs, node, h1, h2, len_in, len_out, head, tail, hw, r_pref, clear_min, mode):
    turn = math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))
    sgn = 1.0 if turn > 0 else -1.0
    cands = []
    rots = []
    rmax = math.hypot(max(head, tail), hw)
    for tt in (turn, turn - sgn * 2 * math.pi):
        n = sweep_samples(abs(tt) * rmax, 16)
        rot = [(node[0], node[1], h1 + tt * i / n) for i in range(n + 1)]
        rots.append(({"rotate": 1.0 if tt > 0 else -1.0}, clearance(segs, rot, head, tail, hw)))
    if mode == "auto":                      # 优先拐点原地转向 (严格沿线路)，转不开再用圆弧
        for c in rots:
            if c[1] >= clear_min:
                return c
    if mode in ("arc", "auto") and abs(turn) < 2.6:
        t_half = math.tan(abs(turn) / 2.0)
        r_lim = 0.45 * min(len_in, len_out) / max(t_half, 1e-3)
        r_max = min(r_pref, r_lim)
        arcs = []
        for R in sorted({min(r_pref * 1.25, r_lim), r_max, r_max * 0.7, r_max * 0.45}, reverse=True):
            if R < 0.25:
                continue
            d = R * t_half
            sx, sy = node[0] - d * math.cos(h1), node[1] - d * math.sin(h1)
            cx, cy = sx - sgn * R * math.sin(h1), sy + sgn * R * math.cos(h1)
            n = sweep_samples(abs(turn) * (R + rmax), 12)
            poses = [(cx + sgn * R * math.sin(h1 + turn * i / n), cy - sgn * R * math.cos(h1 + turn * i / n), h1 + turn * i / n)
                     for i in range(n + 1)]
            clr = clearance(segs, poses, head, tail, hw)
            arcs.append(({"heading": h2, "turn": turn, "R": R, "d": d, "v": 0.35, "cx": cx, "cy": cy}, clr))
        # 圆弧中取车体净空最大的 (净空相同取大半径)
        good = [a for a in arcs if a[1] >= clear_min]
        if good:
            return max(good, key=lambda a: (round(a[1], 2), a[0]["R"]))
        cands += arcs
    for c in rots:
        cands.append(c)
        if c[1] >= clear_min:
            return c
    return max(cands, key=lambda z: z[1])
