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


def box_segments(obstacles: Sequence[dict]) -> List[tuple]:
    segs = []
    for o in obstacles or []:
        ox, oy = float(o.get("x", 0)), float(o.get("y", 0))
        hw, hh = float(o.get("w", .8)) / 2, float(o.get("h", .8)) / 2
        segs += [(ox - hw, oy - hh, ox + hw, oy - hh), (ox + hw, oy - hh, ox + hw, oy + hh),
                 (ox + hw, oy + hh, ox - hw, oy + hh), (ox - hw, oy + hh, ox - hw, oy - hh)]
    return segs


def _perimeter(head: float, tail: float, hw: float, step: float = 0.08) -> np.ndarray:
    per = []
    for a, b in (((-tail, -hw), (head, -hw)), ((head, -hw), (head, hw)), ((head, hw), (-tail, hw)), ((-tail, hw), (-tail, -hw))):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step) + 1)
        per += [(a[0] + (b[0] - a[0]) * k / (n - 1), a[1] + (b[1] - a[1]) * k / (n - 1)) for k in range(n)]
    return np.array(per)


def clearance(segs: Sequence[tuple], poses: Sequence[Tuple[float, float, float]], head: float, tail: float, hw: float) -> float:
    """车体在一组位姿上到线段集合的最小距离 (m)；线段端点落入车体内返回 -1"""
    if not segs or not poses:
        return 9.0
    S = np.asarray([w[:4] for w in segs], float)
    A, B = S[:, :2], S[:, 2:]
    D = B - A
    L2 = np.maximum((D * D).sum(1), 1e-12)
    # 只考虑车体扫掠范围附近的线段
    xs = np.array([p[0] for p in poses]); ys = np.array([p[1] for p in poses])
    R = math.hypot(max(head, tail), hw) + 0.3
    near = ((np.minimum(A[:, 0], B[:, 0]) < xs.max() + R) & (np.maximum(A[:, 0], B[:, 0]) > xs.min() - R) &
            (np.minimum(A[:, 1], B[:, 1]) < ys.max() + R) & (np.maximum(A[:, 1], B[:, 1]) > ys.min() - R))
    if not near.any():
        return 9.0
    A, B, D, L2 = A[near], B[near], D[near], L2[near]
    P0 = _perimeter(head, tail, hw)
    pts = []
    for x, y, yaw in poses:
        c, s = math.cos(yaw), math.sin(yaw)
        for E in (A, B):
            lx = (E[:, 0] - x) * c + (E[:, 1] - y) * s
            ly = -(E[:, 0] - x) * s + (E[:, 1] - y) * c
            if np.any((lx > -tail) & (lx < head) & (np.abs(ly) < hw)):
                return -1.0
        pts.append(np.stack([x + c * P0[:, 0] - s * P0[:, 1], y + s * P0[:, 0] + c * P0[:, 1]], 1))
    Q = np.concatenate(pts)
    u = np.clip(((Q[:, None, :] - A[None]) * D[None]).sum(2) / L2[None], 0, 1)
    C = A[None] + u[..., None] * D[None]
    return float(np.sqrt(((Q[:, None, :] - C) ** 2).sum(2)).min())


def plan_corner(segs, node, h1: float, h2: float, len_in: float, len_out: float,
                head: float, tail: float, hw: float, r_pref: float, clear_min: float = CLEAR_MIN,
                allow_arcs: bool = False, mode: Optional[str] = None) -> Tuple[dict, float]:
    """拐点过弯方式 (默认只允许拐点原地转向 —— 车辆严格沿拓扑线路行驶；allow_arcs=True 时先尝试圆弧): 圆弧 (1.25/1/0.7/0.45 倍 r_pref，受路段长度限制) 中净空 ≥ CLEAR_MIN 且最大的；
    圆弧都不行再试拐点原地转向 (最短方向、反方向)；都不满足返回净空最大的。
    返回 ({"heading","turn","R","d","v"} 圆弧 | {"rotate": ±1} 原地转向, 净空)"""
    mode = mode or ("arc" if allow_arcs else "rotate")
    turn = math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))
    sgn = 1.0 if turn > 0 else -1.0
    cands = []
    rots = []
    for tt in (turn, turn - sgn * 2 * math.pi):
        rot = [(node[0], node[1], h1 + tt * i / 16.0) for i in range(17)]
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
            poses = [(cx + sgn * R * math.sin(h1 + turn * i / 12.0), cy - sgn * R * math.cos(h1 + turn * i / 12.0), h1 + turn * i / 12.0)
                     for i in range(13)]
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
