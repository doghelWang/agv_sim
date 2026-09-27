#!/usr/bin/env python3
"""
仿真世界几何: 墙/货架/动态障碍 (带高度的竖直线段集合) + 车身轮廓碰撞检测 (numpy 向量化)

每条线段: (x0, y0, x1, y1, z_top)  —— 竖直面，从地面到 z_top。
激光只看得见 z_top 高于自身安装高度的线段 (低矮障碍物对顶部激光不可见，这是真实盲区)。
"""

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from . import native

WALL_HEIGHT = 6.0        # 场景外墙 (到顶)
CEILING_HEIGHT = 6.0     # 库房屋顶 (3D 激光向上的点打在屋顶上)
SHELF_HEIGHT = 2.5       # 货架/设备岛
OBSTACLE_HEIGHT = 1.0    # 动态障碍物默认高度 (托盘/纸箱/人腿)


class World:
    def __init__(self):
        self.static_segments: np.ndarray = np.zeros((0, 5))
        self.dynamic_segments: np.ndarray = np.zeros((0, 5))
        self.obstacles: List[dict] = []
        self.segments: np.ndarray = np.zeros((0, 5))
        self.bounds = (-10.0, -10.0, 10.0, 10.0)
        self.engine = None          # 几何引擎 (MuJoCoBackend)；存在时所有射线求交委托给它

    # ------------------------------------------------------------------
    def load_scenario(self, scenario: dict):
        walls = scenario.get("walls", [])
        segs = []
        for i, w in enumerate(walls):
            h = WALL_HEIGHT if i < 4 else SHELF_HEIGHT
            segs.append([w[0], w[1], w[2], w[3], h])
        self.static_segments = np.asarray(segs, dtype=float).reshape(-1, 5)
        if len(walls) >= 4:
            xs = [c for w in walls[:4] for c in (w[0], w[2])]
            ys = [c for w in walls[:4] for c in (w[1], w[3])]
            self.bounds = (min(xs), min(ys), max(xs), max(ys))
        self.set_obstacles([])

    def set_obstacles(self, obstacles: Iterable[dict]):
        self.obstacles = [dict(o) for o in obstacles or []]
        self._rebuild_dynamic()

    def _rebuild_dynamic(self):
        segs = []
        for o in self.obstacles:
            ox, oy = float(o.get("x", 0.0)), float(o.get("y", 0.0))
            hw, hh = float(o.get("w", 0.8)) / 2.0, float(o.get("h", 0.8)) / 2.0
            z = float(o.get("z", o.get("height", OBSTACLE_HEIGHT)))
            yaw = float(o.get("yaw", 0.0))
            c, s = math.cos(yaw), math.sin(yaw)
            pts = [(ox + c * dx - s * dy, oy + s * dx + c * dy) for dx, dy in ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh))]
            for i in range(4):
                a, b = pts[i], pts[(i + 1) % 4]
                segs.append([a[0], a[1], b[0], b[1], z])
        self.dynamic_segments = np.asarray(segs, dtype=float).reshape(-1, 5)
        self.segments = np.ascontiguousarray(np.vstack([self.static_segments, self.dynamic_segments]))

    # ------------------------------------------------------------------
    @staticmethod
    def footprint_world(fp: Sequence[Sequence[float]], x: float, y: float, th: float, pad: float = 0.0) -> np.ndarray:
        P = np.asarray(fp, dtype=float)
        if pad:
            cx, cy = P.mean(axis=0)
            d = P - [cx, cy]
            n = np.linalg.norm(d, axis=1, keepdims=True)
            P = P + d / np.maximum(n, 1e-9) * pad * math.sqrt(2)
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s], [s, c]])
        return P @ R.T + [x, y]

    def collides(self, fp: Sequence[Sequence[float]], x: float, y: float, th: float) -> Tuple[bool, Optional[np.ndarray]]:
        """车身多边形 vs 全部线段: 边相交 或 线段端点落在多边形内。返回 (是否碰撞, 接触点世界坐标)"""
        segs = self.segments
        if len(segs) == 0:
            return False, None
        if native.lib is not None:
            if not (isinstance(fp, np.ndarray) and fp.dtype == np.float64 and fp.flags.c_contiguous):
                fp = native.f64(fp)
            return native.collides(fp, float(x), float(y), float(th), segs)
        return self._collides_py(fp, x, y, th)

    def _collides_py(self, fp, x, y, th):
        segs = self.segments
        if len(segs) == 0:
            return False, None
        poly = self.footprint_world(fp, x, y, th)
        # 粗筛: 包围盒
        mnx, mny = poly.min(axis=0) - 0.01
        mxx, mxy = poly.max(axis=0) + 0.01
        sx0 = np.minimum(segs[:, 0], segs[:, 2]); sx1 = np.maximum(segs[:, 0], segs[:, 2])
        sy0 = np.minimum(segs[:, 1], segs[:, 3]); sy1 = np.maximum(segs[:, 1], segs[:, 3])
        m = (sx1 >= mnx) & (sx0 <= mxx) & (sy1 >= mny) & (sy0 <= mxy)
        if not m.any():
            return False, None
        S = segs[m]
        # 1) 多边形边 × 线段 相交
        E0 = poly
        E1 = np.roll(poly, -1, axis=0)
        p = E0[:, None, :]; r = (E1 - E0)[:, None, :]
        q = S[None, :, 0:2]; s = (S[None, :, 2:4] - S[None, :, 0:2])
        rxs = r[..., 0] * s[..., 1] - r[..., 1] * s[..., 0]
        qp = q - p
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (qp[..., 0] * s[..., 1] - qp[..., 1] * s[..., 0]) / rxs
            u = (qp[..., 0] * r[..., 1] - qp[..., 1] * r[..., 0]) / rxs
        hit = (np.abs(rxs) > 1e-12) & (t >= 0) & (t <= 1) & (u >= 0) & (u <= 1)
        if hit.any():
            i, j = np.argwhere(hit)[0]
            pt = E0[i] + t[i, j] * (E1[i] - E0[i])
            return True, pt
        # 2) 线段端点在多边形内 (小障碍物完全落入车身)
        pts = np.vstack([S[:, 0:2], S[:, 2:4]])
        inside = self._points_in_poly(pts, poly)
        if inside.any():
            return True, pts[np.argmax(inside)]
        return False, None

    @staticmethod
    def _points_in_poly(pts: np.ndarray, poly: np.ndarray) -> np.ndarray:
        x, y = pts[:, 0][:, None], pts[:, 1][:, None]
        x0, y0 = poly[:, 0][None, :], poly[:, 1][None, :]
        x1, y1 = np.roll(poly[:, 0], -1)[None, :], np.roll(poly[:, 1], -1)[None, :]
        cond = ((y0 > y) != (y1 > y))
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = (x1 - x0) * (y - y0) / (y1 - y0) + x0
        cross = cond & (x < xint)
        return (cross.sum(axis=1) % 2) == 1

    # ------------------------------------------------------------------
    def raycast(self, ox: float, oy: float, angles: np.ndarray, max_range: float, min_seg_height: float = 0.0,
                few: bool = False) -> np.ndarray:
        """从 (ox, oy) 沿 angles 方向投射射线，返回命中距离 (未命中 = inf)。仅考虑 z_top > min_seg_height 的线段
        有几何引擎 (MuJoCo) 时一律由引擎求交；兜底后端有 C 内核时用 C 线段求交 (与下面 numpy 实现一致)"""
        if self.engine is None and native.lib is not None:
            return native.raycast2d(self.segments, None, min_seg_height, ox, oy, angles, max_range)
        if self.engine is not None:
            return self.engine.raycast2d(ox, oy, max(0.005, min_seg_height), np.asarray(angles, float), max_range)
        return self._raycast_py(ox, oy, angles, max_range, min_seg_height)

    def _raycast_py(self, ox, oy, angles, max_range, min_seg_height=0.0):
        segs = self.segments
        if len(segs):
            segs = segs[segs[:, 4] > min_seg_height]
        n = len(angles)
        if len(segs) == 0 or n == 0:
            return np.full(n, np.inf)
        # 粗筛: 线段与射线圆的距离
        a = segs[:, 0:2]; b = segs[:, 2:4]
        ab = b - a
        l2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
        t = np.clip(((ox - a[:, 0]) * ab[:, 0] + (oy - a[:, 1]) * ab[:, 1]) / l2, 0, 1)
        near = a + ab * t[:, None]
        dmin = np.hypot(near[:, 0] - ox, near[:, 1] - oy)
        segs = segs[dmin <= max_range]
        if len(segs) == 0:
            return np.full(n, np.inf)
        dx = np.cos(angles)[:, None]; dy = np.sin(angles)[:, None]
        qx = segs[None, :, 0] - ox; qy = segs[None, :, 1] - oy
        sx = (segs[:, 2] - segs[:, 0])[None, :]; sy = (segs[:, 3] - segs[:, 1])[None, :]
        den = dx * sy - dy * sx
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (qx * sy - qy * sx) / den
            u = (qx * dy - qy * dx) / den
        valid = (np.abs(den) > 1e-12) & (t > 1e-6) & (u >= 0.0) & (u <= 1.0)
        t = np.where(valid, t, np.inf)
        r = t.min(axis=1)
        r[r > max_range] = np.inf
        return r


    # ------------------------------------------------------------------
    def raycast3d(self, origin, dirs: np.ndarray, max_range: float, min_range: float = 0.0) -> np.ndarray:
        """3D 射线 (世界系单位方向 N×3) 与 竖直线段墙面 (高 z_top) 及地面 z=0 求交，返回距离 (无命中 = inf)。
        场景是 2.5D 的: 墙/货架/障碍物为从地面到 z_top 的竖直面，上方开放 (无天花板)"""
        if self.engine is not None:
            return self.engine.raycast3d(origin, np.asarray(dirs, float), max_range, min_range)
        ox, oy, oz = float(origin[0]), float(origin[1]), float(origin[2])
        n = len(dirs)
        best = np.full(n, np.inf)
        vz = dirs[:, 2]
        # 地面
        down = vz < -1e-6
        best[down] = -oz / vz[down]
        up = vz > 1e-6
        if CEILING_HEIGHT > oz:
            best[up] = (CEILING_HEIGHT - oz) / vz[up]
        hx, hy = dirs[:, 0], dirs[:, 1]
        hl = np.hypot(hx, hy)
        segs = self.segments
        if len(segs):
            a = segs[:, 0:2]; b = segs[:, 2:4]; ab = b - a
            l2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
            t = np.clip(((ox - a[:, 0]) * ab[:, 0] + (oy - a[:, 1]) * ab[:, 1]) / l2, 0, 1)
            near = a + ab * t[:, None]
            segs = segs[np.hypot(near[:, 0] - ox, near[:, 1] - oy) <= max_range]
        if len(segs):
            ok = hl > 1e-6
            idx = np.nonzero(ok)[0]
            ux = (hx[idx] / hl[idx])[:, None]; uy = (hy[idx] / hl[idx])[:, None]
            qx = segs[None, :, 0] - ox; qy = segs[None, :, 1] - oy
            sx = (segs[:, 2] - segs[:, 0])[None, :]; sy = (segs[:, 3] - segs[:, 1])[None, :]
            den = ux * sy - uy * sx
            with np.errstate(divide="ignore", invalid="ignore"):
                th = (qx * sy - qy * sx) / den          # 水平距离
                u = (qx * uy - qy * ux) / den
            s3 = th / hl[idx][:, None]                  # 3D 射线长度
            z = oz + s3 * vz[idx][:, None]
            valid = (np.abs(den) > 1e-12) & (th > 1e-6) & (u >= 0) & (u <= 1) & (z >= 0.0) & (z <= segs[None, :, 4])
            s3 = np.where(valid, s3, np.inf).min(axis=1)
            best[idx] = np.minimum(best[idx], s3)
        best[(best > max_range) | (best < min_range)] = np.inf
        return best
