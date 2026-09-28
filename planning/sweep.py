#!/usr/bin/env python3
"""
扫掠判定的精确解 (numpy 向量化)，与 ros2/agv_nav2_plugins/include/agv_nav2_plugins/sweep.hpp 同一算法：
点绕中心转过一段圆弧，是否进入 (或起点就在) 开矩形 (x0, x1) × (y0, y1)。

原先按几个固定角度采样 (0.33/0.66/1.0 × 前视角)，车角半径 1.4 m 时每步移动约 12 cm，
比 2 cm 的外扩余量大得多，贴边的点会在采样之间"穿过"外扩带而漏判。
"""
import math

import numpy as np

TWO_PI = 2.0 * math.pi


def _in(x, y, box):
    x0, x1, y0, y1 = box
    return (x > x0) & (x < x1) & (y > y0) & (y < y1)


def arc_hits_box(px, py, phi: float, box, cx: float = 0.0, cy: float = 0.0) -> np.ndarray:
    """px, py: 点坐标数组；phi: 绕 (cx, cy) 转过的带符号角 (逆时针为正)；返回每个点是否进入矩形"""
    px = np.asarray(px, dtype=float)
    py = np.asarray(py, dtype=float)
    x0, x1, y0, y1 = box
    hit = _in(px, py, box)
    ux, uy = px - cx, py - cy
    r2 = ux * ux + uy * uy
    fx = max(abs(x0 - cx), abs(x1 - cx))
    fy = max(abs(y0 - cy), abs(y1 - cy))
    cand = ~hit & (r2 < fx * fx + fy * fy) & (r2 > 1e-24)       # 剪枝: 圆比矩形最远角还远的点不可能进入
    if abs(phi) < 1e-12 or not cand.any():
        return hit
    idx = np.nonzero(cand)[0]
    ux, uy, r2 = ux[idx], uy[idx], r2[idx]
    rho = np.sqrt(r2)
    a0 = np.arctan2(uy, ux)
    span = abs(phi)
    full = span >= TWO_PI
    sub = _in(cx + rho * np.cos(a0 + phi), cy + rho * np.sin(a0 + phi), box)

    def on_arc(a):
        if full:
            return np.ones_like(a, dtype=bool)
        t = (a - a0) if phi >= 0 else (a0 - a)
        return np.mod(t, TWO_PI) <= span

    for X in (x0, x1):                                   # 竖边 x = X
        dx = X - cx
        ok = np.abs(dx) <= rho
        dy = np.sqrt(np.maximum(r2 - dx * dx, 0.0))
        for sy in (dy, -dy):
            y = cy + sy
            sub |= ok & (y > y0) & (y < y1) & on_arc(np.arctan2(sy, dx))
    for Y in (y0, y1):                                   # 横边 y = Y
        dy = Y - cy
        ok = np.abs(dy) <= rho
        dx = np.sqrt(np.maximum(r2 - dy * dy, 0.0))
        for sx in (dx, -dx):
            x = cx + sx
            sub |= ok & (x > x0) & (x < x1) & on_arc(np.arctan2(dy, sx))
    hit[idx] |= sub
    return hit
