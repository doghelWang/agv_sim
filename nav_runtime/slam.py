"""
激光 SLAM 定位 + 里程计融合 (执行进程内运行，只依赖 numpy)

数据来源 (均来自仿真进程 REST，不使用真值):
    合并激光   /api/v1/sensors/scan   base_link 系 360° 虚拟扫描 (10 Hz，带仿真时间戳 t)
    轮式里程计 /api/v1/state.odom     odom 系，含轮径误差 / 舵角偏置 / 编码器量化，并且真值带打滑 → 会漂移

组成:
    1. 建图      占据栅格 (log-odds)：激光束穿过的格子降低占据概率，终点格子提高 (2.5 cm)；
                 同时累积终点的亚栅格命中密度 (双线性分摊)，用于匹配 (保留墙面的厘米级位置)。
    2. 匹配场    命中密度 × 占据掩码 → 高斯平滑 (细: 2.5 cm 栅格 σ 5 cm，粗: 10 cm 栅格 σ 15 cm) → 按局部峰值归一化 → 饱和变换 F∈[0,1)。
    3. 扫描匹配  Gauss-Newton (Hector 风格)，粗→细两级，最小化 Σ(1-F(T·p))²；
                 以里程计预测位姿为先验 (迭代 EKF 形式)，协方差 = (JᵀJ/σ² + P⁻¹)⁻¹，
                 长走廊等退化方向由里程计补足。
    4. 融合      定位结果表示为 map→odom 修正量 M：  map 位姿 = M ∘ odom 位姿。
                 50 Hz 用里程计推算 (控制用)，10 Hz 用激光匹配修正 M (按激光时间戳插值里程计)。

模式:
    slam          边建图边定位 (无保存地图时的默认)
    localization  加载已保存的地图，只定位不更新地图
    odom          只用里程计 (演示漂移)
    ground_truth  直接用仿真真值 (对照 / 调试)

初始位姿: 车辆在已知位置启动 (等同于实车在工位上设置初始位姿)。仿真里取启动/复位瞬间的车辆位姿，
         也可通过 POST /api/v1/slam/initialpose 手动设置。此后定位不再读取真值 (真值只用于误差统计)。
"""
from __future__ import annotations

import base64
import bisect
import math
import os
import threading
import time
from collections import deque
from typing import Optional

import numpy as np

from nav_runtime import slam_native

MODES = ("slam", "localization", "odom", "ground_truth")


# ---------------------------------------------------------------------- SE2
def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


# ---------------------------------------------------------------------- 栅格地图
class GridMap:
    L_FREE, L_OCC, L_MIN, L_MAX = -0.4, 0.85, -4.0, 4.0
    HIT_CAP = 5000.0

    def __init__(self, res=0.025):
        self.res = res
        self.ox = self.oy = 0.0
        self.L = np.zeros((0, 0), np.float32)       # log-odds
        self.Hd = np.zeros((0, 0), np.float32)      # 终点命中密度 (双线性分摊)
        self.updates = 0
        self.rev = 0

    @property
    def shape(self):
        return self.L.shape

    def empty(self):
        return self.L.size == 0

    def ensure(self, xmin, ymin, xmax, ymax, pad=5.0):
        """保证世界范围落在地图内 (按 pad 米为单位扩展)"""
        r = self.res
        if self.empty():
            self.ox, self.oy = math.floor((xmin - pad) / r) * r, math.floor((ymin - pad) / r) * r
            w = int(math.ceil((xmax + pad - self.ox) / r))
            h = int(math.ceil((ymax + pad - self.oy) / r))
            self.L = np.zeros((h, w), np.float32)
            self.Hd = np.zeros((h, w), np.float32)
            return
        h, w = self.L.shape
        x1, y1 = self.ox + w * r, self.oy + h * r
        if xmin >= self.ox and ymin >= self.oy and xmax < x1 and ymax < y1:
            return
        px0 = int(math.ceil(max(0.0, self.ox - (xmin - pad)) / r)) if xmin < self.ox else 0
        py0 = int(math.ceil(max(0.0, self.oy - (ymin - pad)) / r)) if ymin < self.oy else 0
        px1 = int(math.ceil(max(0.0, (xmax + pad) - x1) / r)) if xmax >= x1 else 0
        py1 = int(math.ceil(max(0.0, (ymax + pad) - y1) / r)) if ymax >= y1 else 0
        self.L = np.pad(self.L, ((py0, py1), (px0, px1)))
        self.Hd = np.pad(self.Hd, ((py0, py1), (px0, px1)))
        self.ox -= px0 * r
        self.oy -= py0 * r

    def insert(self, pose, px, py, hit, sx=None, sy=None, free_cap=8.0):
        """插入一帧 (机体系): 终点 (px,py)，光束起点 (sx,sy) = 各激光安装位置 (缺省为车体中心)。
        hit=False 为无回波方向 (终点取在最大量程处)，只用于清空。"""
        if slam_native.lib is not None:
            slam_native.insert(self, pose, px, py, hit, sx, sy, free_cap)
            self.updates += 1
            self.rev += 1
            return
        x, y, th = pose
        r = self.res
        c, s = math.cos(th), math.sin(th)
        sx = np.zeros_like(px) if sx is None else np.broadcast_to(sx, px.shape)
        sy = np.zeros_like(py) if sy is None else np.broadcast_to(sy, py.shape)
        wsx, wsy = x + c * sx - s * sy, y + s * sx + c * sy
        ex, ey = x + c * px - s * py, y + s * px + c * py
        dx, dy = ex - wsx, ey - wsy
        rng = np.hypot(dx, dy)
        ca, sa = dx / np.maximum(rng, 1e-9), dy / np.maximum(rng, 1e-9)
        reach = np.where(hit, rng, np.minimum(rng, free_cap))
        fx, fy = wsx + reach * ca, wsy + reach * sa
        self.ensure(min(x, fx.min()), min(y, fy.min()), max(x, fx.max()), max(y, fy.max()))
        h, w = self.L.shape
        # 空闲: 光束穿过的格子，终点前留 10 cm (测距噪声 + 掠射光束不擦除墙面)
        step = max(r * 0.8, 0.03)
        rf = np.where(hit, rng - max(4 * r, 0.10), reach)
        k = np.maximum(0, (rf / step).astype(np.int64))
        if k.sum() > 300000:                     # 光束太多时抽稀做空闲更新 (终点仍全部使用)
            keep = np.zeros(len(k), bool)
            keep[:: int(math.ceil(k.sum() / 300000))] = True
            k = np.where(keep, k, 0)
        tot = int(k.sum())
        if tot:
            ray = np.repeat(np.arange(len(k)), k)
            start = np.repeat(np.cumsum(k) - k, k)
            d = (np.arange(tot) - start + 0.5) * step
            ix = ((wsx[ray] + d * ca[ray] - self.ox) / r).astype(np.int64)
            iy = ((wsy[ray] + d * sa[ray] - self.oy) / r).astype(np.int64)
            free = np.unique(iy * w + ix)
        else:
            free = np.zeros(0, np.int64)
        ex, ey = ex[hit], ey[hit]
        ix = ((ex - self.ox) / r).astype(np.int64)
        iy = ((ey - self.oy) / r).astype(np.int64)
        occ = np.unique(iy * w + ix)
        free = np.setdiff1d(free, occ, assume_unique=True)
        Lf = self.L.reshape(-1)
        Lf[free] = np.maximum(self.L_MIN, Lf[free] + self.L_FREE)
        Lf[occ] = np.minimum(self.L_MAX, Lf[occ] + self.L_OCC)
        # 命中密度: 双线性分摊到 4 个格心，保留亚栅格位置
        gx = (ex - self.ox) / r - 0.5
        gy = (ey - self.oy) / r - 0.5
        i0, j0 = np.floor(gx).astype(np.int64), np.floor(gy).astype(np.int64)
        fx_, fy_ = gx - i0, gy - j0
        Hf = self.Hd.reshape(-1)
        for di, dj, wt in ((0, 0, (1 - fx_) * (1 - fy_)), (1, 0, fx_ * (1 - fy_)), (0, 1, (1 - fx_) * fy_), (1, 1, fx_ * fy_)):
            ii, jj = i0 + di, j0 + dj
            ok = (ii >= 0) & (ii < w) & (jj >= 0) & (jj < h)
            np.add.at(Hf, jj[ok] * w + ii[ok], wt[ok].astype(np.float32))
        # 长时间停在原地时近处墙体命中数会远大于远处，封顶保持平衡
        Hf[occ] = np.minimum(Hf[occ], self.HIT_CAP)
        self.updates += 1
        self.rev += 1

    # ---- 导出
    def classes(self):
        """0 未知 / 1 空闲 / 2 占据"""
        c = np.zeros(self.L.shape, np.uint8)
        c[self.L < -0.5] = 1
        c[self.L > 0.5] = 2
        return c

    def to_pgm_yaml(self, image_name):
        c = self.classes()
        img = np.full(c.shape, 205, np.uint8)
        img[c == 1] = 254
        img[c == 2] = 0
        img = np.flipud(img)
        h, w = img.shape
        pgm = b"P5\n%d %d\n255\n" % (w, h) + img.tobytes()
        yaml = (f"image: {image_name}\nresolution: {self.res}\norigin: [{self.ox:.3f}, {self.oy:.3f}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")
        return pgm, yaml


# ---------------------------------------------------------------------- 匹配场
def _blur(a, sigma_cells):
    """可分离高斯模糊 (numpy 切片累加，无 scipy)"""
    rad = max(1, int(math.ceil(3 * sigma_cells)))
    k = np.exp(-0.5 * (np.arange(-rad, rad + 1) / sigma_cells) ** 2)
    k = (k / k.sum()).astype(np.float32)
    out = a
    for ax in (0, 1):
        p = np.pad(out, [(rad, rad) if i == ax else (0, 0) for i in range(2)])
        acc = np.zeros_like(out)
        n = out.shape[ax]
        for i, kv in enumerate(k):
            sl = [slice(None), slice(None)]
            sl[ax] = slice(i, i + n)
            acc += kv * p[tuple(sl)]
        out = acc
    return out


def _maxfilt(a, rad):
    """可分离最大值滤波 (窗口 2·rad+1)"""
    out = a
    for ax in (0, 1):
        p = np.pad(out, [(rad, rad) if i == ax else (0, 0) for i in range(2)])
        acc = out.copy()
        n = out.shape[ax]
        for i in range(2 * rad + 1):
            sl = [slice(None), slice(None)]
            sl[ax] = slice(i, i + n)
            np.maximum(acc, p[tuple(sl)], out=acc)
        out = acc
    return out


class Field:
    def __init__(self, F, ox, oy, res):
        self.F, self.ox, self.oy, self.res = F, ox, oy, res
        self.gy, self.gx = F.shape[0] - 2, F.shape[1] - 2

    def sample(self, wx, wy):
        g_x = (wx - self.ox) / self.res - 0.5
        g_y = (wy - self.oy) / self.res - 0.5
        i0 = np.floor(g_x).astype(np.int64)
        j0 = np.floor(g_y).astype(np.int64)
        ok = (i0 >= 0) & (i0 <= self.gx) & (j0 >= 0) & (j0 <= self.gy)
        i0c, j0c = np.clip(i0, 0, self.gx), np.clip(j0, 0, self.gy)
        fx, fy = g_x - i0, g_y - j0
        F = self.F
        v00, v10 = F[j0c, i0c], F[j0c, i0c + 1]
        v01, v11 = F[j0c + 1, i0c], F[j0c + 1, i0c + 1]
        val = (1 - fx) * (1 - fy) * v00 + fx * (1 - fy) * v10 + (1 - fx) * fy * v01 + fx * fy * v11
        dx = ((1 - fy) * (v10 - v00) + fy * (v11 - v01)) / self.res
        dy = ((1 - fx) * (v01 - v00) + fx * (v11 - v10)) / self.res
        return np.where(ok, val, 0.0), np.where(ok, dx, 0.0), np.where(ok, dy, 0.0)


# (名称, 下采样倍数, 高斯 σ [m], 归一化最大值窗口半径 [m])
FIELD_LEVELS = (("fine", 1, 0.05, 0.10), ("coarse", 4, 0.15, 0.30))


def build_fields(gm: GridMap):
    """命中密度 (仅占据格) → 细/粗两级匹配场。只在有占据的包围盒内计算。
    每条墙的密度按局部最大值归一化 (峰值=1)，因此远近墙、扫描次数不同的区域权重一致；
    峰值位置 = 命中点的平均位置 (亚栅格精度)。"""
    if slam_native.lib is not None:
        return slam_native.build_fields(gm, FIELD_LEVELS, Field)
    occ = gm.L > 0.0
    if not occ.any():
        return None
    occ = _maxfilt(occ.astype(np.uint8), 2) > 0      # 占据掩码外扩 2 格，墙面命中分布不被截断
    ys, xs = np.nonzero(occ)
    m = int(math.ceil(0.8 / gm.res))
    y0, y1 = max(0, ys.min() - m), min(occ.shape[0], ys.max() + m + 1)
    x0, x1 = max(0, xs.min() - m), min(occ.shape[1], xs.max() + m + 1)
    H0 = np.where(occ[y0:y1, x0:x1], gm.Hd[y0:y1, x0:x1], 0.0).astype(np.float32)
    ox, oy = gm.ox + x0 * gm.res, gm.oy + y0 * gm.res
    out = {"rev": gm.rev}
    for name, k, sig, rad in FIELD_LEVELS:
        H, r = H0, gm.res
        if k > 1:
            h, w = H.shape
            H = np.pad(H, ((0, (-h) % k), (0, (-w) % k)))
            H = H.reshape(H.shape[0] // k, k, H.shape[1] // k, k).sum(axis=(1, 3))
            r = gm.res * k
        B = _blur(H, sig / r)
        mx = _maxfilt(B, max(1, int(round(rad / r))))
        F = B / np.maximum(mx, 1e-3 * float(B.max()) + 1e-6)
        out[name] = Field(np.clip(F, 0.0, 1.0).astype(np.float32), ox, oy, r)
    return out


# ---------------------------------------------------------------------- 扫描匹配
def match(fields, px, py, init, P0=None, sigma=0.25, iters=(8, 10)):
    """Gauss-Newton 扫描匹配 (带里程计先验)。返回 (pose, cov3x3, info)"""
    if slam_native.lib is not None:
        return slam_native.match(fields, px, py, init, P0, sigma, iters)
    x = np.array(init, float)
    x0 = x.copy()
    Pinv = np.linalg.inv(P0) if P0 is not None else np.zeros((3, 3))
    # 远处点测距噪声大 (σ ≈ 1.5 cm + 0.2%·r)，按方差反比加权
    rr = np.hypot(px, py)
    wgt = (0.015 / (0.015 + 0.002 * rr)) ** 2
    wgt = wgt / wgt.mean()
    JtJ = np.zeros((3, 3))
    F = np.zeros(0)
    for lvl, n_it in zip(("coarse", "fine"), iters):
        fld = fields[lvl]
        s2 = sigma * sigma * (4.0 if lvl == "coarse" else 1.0)
        for _ in range(n_it):
            c, s = math.cos(x[2]), math.sin(x[2])
            wx = x[0] + c * px - s * py
            wy = x[1] + s * px + c * py
            F, gx, gy = fld.sample(wx, wy)
            dth = gx * (-s * px - c * py) + gy * (c * px - s * py)
            J = np.stack([gx, gy, dth], 1)
            r = 1.0 - F
            Jw = J * wgt[:, None]
            JtJ = Jw.T @ J
            e = x - x0
            e[2] = _wrap(e[2])
            H = JtJ / s2 + Pinv + np.diag([1e-6, 1e-6, 1e-6])
            b = Jw.T @ r / s2 - Pinv @ e
            try:
                dx = np.linalg.solve(H, b)
            except np.linalg.LinAlgError:
                break
            dx[:2] = np.clip(dx[:2], -0.2, 0.2)
            dx[2] = max(-0.1, min(0.1, dx[2]))
            x += dx
            x[2] = _wrap(x[2])
            if abs(dx[0]) < 1e-5 and abs(dx[1]) < 1e-5 and abs(dx[2]) < 1e-5:
                break
    Hm = JtJ / (sigma * sigma)
    try:
        cov = np.linalg.inv(Hm + Pinv + np.eye(3) * 1e-9)
    except np.linalg.LinAlgError:
        cov = P0 if P0 is not None else np.eye(3)
    ev = np.linalg.eigvalsh(Hm[:2, :2]) if Hm.any() else np.zeros(2)
    info = {"inliers": float(np.mean(F > 0.3)) if F.size else 0.0, "score": float(np.mean(F)) if F.size else 0.0,
            "n": int(len(px)), "eig_min": float(ev.min()), "eig_max": float(ev.max())}
    return (float(x[0]), float(x[1]), float(x[2])), cov, info


def _default_map_dir():
    """SLAM_MAP_DIR > $AGV_DATA/slam_maps > /data/slam_maps (容器挂载) > <项目>/data/slam_maps"""
    if os.environ.get("SLAM_MAP_DIR"):
        return os.environ["SLAM_MAP_DIR"]
    if os.environ.get("AGV_DATA"):
        return os.path.join(os.environ["AGV_DATA"], "slam_maps")
    if os.path.isdir("/data") and os.access("/data", os.W_OK):
        return "/data/slam_maps"
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", "slam_maps")


# ---------------------------------------------------------------------- 定位器
class SlamLocalizer:
    """map 位姿 = M ∘ odom 位姿；M 由激光匹配修正"""

    def __init__(self, map_dir: Optional[str] = None, mode: str = "slam", log=print, res=0.025):
        self.log = log
        self.lock = threading.RLock()
        self.map_dir = map_dir or _default_map_dir()
        self.want_mode = mode if mode in MODES else "slam"
        self.mode = self.want_mode
        self.res = res
        self.scenario = None
        self.gm = GridMap(res)
        self.fields = None
        self._field_busy = False
        self.M = (0.0, 0.0, 0.0)
        self.P = np.diag([1e-4, 1e-4, 1e-4])
        self.inited = False
        self.hist = deque(maxlen=600)      # (t, odom pose)
        self.hist_t = deque(maxlen=600)
        self.odom = (0.0, 0.0, 0.0)
        self.odom_t = 0.0
        self.odom_vel = (0.0, 0.0, 0.0)
        self.pose = (0.0, 0.0, 0.0)
        self.truth = None
        self._last_truth = None
        self._last_odom = None
        self._last_ins = None
        self._last_ins_t = -1e9
        self._last_scan_od = None
        self.max_points = 1200             # 每帧参与匹配的最多点数
        self.body = (0.6, 0.6, 0.4)        # head, tail, half width (滤除车体/负载自身点)
        self.stats = {"matches": 0, "rejects": 0, "match_ms": 0.0, "insert_ms": 0.0, "field_ms": 0.0,
                      "hz": 0.0, "last_info": {}, "reinit": 0}
        self._hz_n, self._hz_t = 0, time.time()
        self.err_hist = deque(maxlen=3000)
        self.events = []
        self.on_event = None
        self._proc = threading.Lock()          # 激光帧串行处理
        _hz = float(os.environ.get("SLAM_BUILTIN_HZ", "0"))
        self._min_dt = (1.0 / _hz - 1e-3) if _hz > 0 else 0.0
        self._last_proc_t = -1e9
        # 地图收敛后自动冻结 (内置引擎): 建图模式下每插入一帧都要重建距离场 (手机上一次 0.3~0.7 s，执行进程因此
        # 常驻 70% CPU)；地图不再长大以后这些都是白算。连续行驶 SLAM_FREEZE_TRAVEL 米而已知栅格数增长不到 0.5%
        # → 保存地图、转定位模式 (只匹配，Flip 5 小核上执行进程 72% → 18%)。之后若匹配内点率持续偏低
        # (开进了没建过图的区域) → 自动回到建图模式。SLAM_AUTO_FREEZE=0 关闭；默认只在 Android 上开。
        self._fz_on = os.environ.get("SLAM_AUTO_FREEZE", "1" if os.path.exists("/system/build.prop") else "0") == "1"
        self._fz_travel = float(os.environ.get("SLAM_FREEZE_TRAVEL", "25"))
        self._fz_dist = 0.0                    # 上次地图明显长大以来行驶的距离
        self._fz_known = 0                     # 当时的已知栅格数
        self._fz_ins = 0
        self._fz_low_t = None                  # 定位模式下内点率开始偏低的时刻
        self._fz_auto = False                  # 当前的定位模式是自动冻结来的 (只有这种才自动解冻)
        # 外部定位引擎 (ROS 2: robot_localization EKF + slam_toolbox，见 ros_slam.py)；None = 内置
        self.ext = None
        self.whist = deque(maxlen=600)         # (墙钟时间, odom 位姿)：对齐 ROS TF 时间戳
        self._ext_pending = None               # 待启动的外部定位栈模式 (等到拿到初始位姿)
        self._ext_last = 0.0

    # ---- 引擎
    @property
    def engine(self):
        return "slam_toolbox" if (self.ext is not None and self.mode in ("slam", "localization")) else "builtin"

    def ros_active(self):
        """外部定位栈接管 TF (odom→base_footprint 由 EKF、map→odom 由 slam_toolbox 发布)"""
        return self.engine == "slam_toolbox" and self.ext.running()

    def attach_external(self, ext):
        """有 ROS 2 且安装了 slam_toolbox / robot_localization 时由 RosBridge 调用"""
        with self.lock:
            self.ext = ext
            if self.mode in ("slam", "localization"):
                self.mode = "localization" if ext.has_saved(self.scenario) else "slam"
                self._ext_pending = self.mode
        self._emit("info", "定位引擎: slam_toolbox + robot_localization (开源)")

    def _ext_start(self, mode, pose):
        ext = self.ext
        if ext is None:
            return
        m = "localization" if mode == "localization" else "mapping"
        try:
            ext.start(m, self.scenario or "default", pose)
            self._emit("info", f"slam_toolbox {'定位' if m == 'localization' else '建图'}模式启动",
                       f"初始位姿 ({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):.1f}°)")
        except Exception as e:  # noqa
            self._emit("warn", f"slam_toolbox 启动失败: {e}")

    def set_external(self, t_sim, pose):
        """外部定位结果 (TF map→base_footprint，时间已换算为仿真时间)：换算成相对轮式里程计的修正 M，之后按里程计零延迟推算"""
        with self.lock:
            if not self.hist_t:
                return
            od = self._odom_at(t_sim)
            self.M = compose(pose, inverse(od))
            self.pose = compose(self.M, self.odom)
            self.stats["matches"] += 1
            self.stats["last_info"] = {"accepted": True, "source": "slam_toolbox"}
            self._ext_last = time.time()
            self._hz_n += 1
            if time.time() - self._hz_t > 2.0:
                self.stats["hz"] = round(self._hz_n / (time.time() - self._hz_t), 1)
                self._hz_n, self._hz_t = 0, time.time()

    def _odom_at_wall(self, tw):
        h = self.whist
        if tw >= h[-1][0]:
            dt = min(tw - h[-1][0], 0.1)
            vx, vy, wz = self.odom_vel
            return compose(h[-1][1], (vx * dt, vy * dt, wz * dt))
        if tw <= h[0][0]:
            return h[0][1]
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid][0] <= tw:
                lo = mid
            else:
                hi = mid
        (t0, a), (t1, b) = h[lo], h[hi]
        u = 0.0 if t1 <= t0 else (tw - t0) / (t1 - t0)
        return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), _wrap(a[2] + u * _wrap(b[2] - a[2])))

    # ---- 配置
    def set_body(self, head, tail, hw):
        self.body = (float(head), float(tail), float(hw))

    def _emit(self, level, msg, detail=""):
        self.events.append((level, msg, detail))
        del self.events[:-20]
        self.log(f"[slam] {msg} {detail}")
        if self.on_event:
            try:
                self.on_event(level, msg, detail)
            except Exception:  # noqa
                pass

    def pop_events(self):
        ev, self.events = self.events, []
        return ev

    def set_mode(self, mode):
        if mode not in MODES:
            raise ValueError(f"mode 须为 {MODES}")
        if self.ext is not None:
            with self.lock:
                if mode == "localization" and not self.ext.has_saved(self.scenario):
                    return "当前场景没有已保存的 slam_toolbox 地图，请先在 slam 模式下建图并保存"
                self.want_mode = self.mode = mode
                pose = self.pose
            if mode in ("slam", "localization"):
                self._ext_start(mode, pose)
            else:
                self.ext.stop()
            return None
        with self.lock:
            self.want_mode = mode
            if mode == "localization" and self.gm.empty():
                if not self._load(self.scenario):
                    self.want_mode = "slam"
                    self.mode = "slam"
                    return "当前场景没有已保存的地图，保持 slam (边建图边定位)"
            self.mode = mode
            self._fz_auto, self._fz_dist, self._fz_known = False, 0.0, 0     # 手动选的模式不自动切换
            if mode in ("odom", "ground_truth"):
                return None
            self._rebuild_fields_async(force=True)
        return None

    def on_scenario(self, sid):
        """场景切换: 有保存的地图 → localization，否则清空重新建图"""
        with self.lock:
            if sid == self.scenario:
                return
            self.scenario = sid
            self.gm = GridMap(self.res)
            self.fields = None
            self.inited = False
            self.err_hist.clear()
            if self.ext is not None and self.want_mode in ("slam", "localization"):
                self.mode = "localization" if self.ext.has_saved(sid) else "slam"
                self._ext_pending = self.mode          # 下一个状态帧拿到初始位姿后启动
                self._emit("info", f"场景 [{sid}]: " + ("加载已保存的 slam_toolbox 地图定位" if self.mode == "localization" else "slam_toolbox 边建图边定位"))
            elif self.want_mode in ("slam", "localization"):
                if self._load(sid):
                    self.mode = "localization"
                    self._fz_auto, self._fz_low_t = self._fz_on and self.want_mode == "slam", None
                    self._emit("info", f"加载已保存的 SLAM 地图 [{sid}]，进入定位模式")
                else:
                    self.mode = "slam"
                    self._emit("info", f"场景 [{sid}] 没有已保存的地图，边建图边定位")
            else:
                self.mode = self.want_mode

    def reset_map(self):
        if self.ext is not None:
            with self.lock:
                self.mode = self.want_mode = "slam"
                pose = self.pose
            self._ext_start("slam", pose)
            self._emit("info", "slam_toolbox 重新建图")
            return
        with self.lock:
            self.gm = GridMap(self.res)
            self.fields = None
            self._last_ins = None
            self._fz_auto, self._fz_dist, self._fz_known = False, 0.0, 0
            if self.mode == "localization":
                self.mode = self.want_mode = "slam"
        self._emit("info", "SLAM 地图已清空，重新建图")

    def set_initial_pose(self, x, y, yaw, std_xy=0.05, std_yaw=0.03):
        with self.lock:
            self.M = compose((x, y, yaw), inverse(self.odom))
            self.P = np.diag([std_xy ** 2, std_xy ** 2, std_yaw ** 2])
            self.pose = (x, y, _wrap(yaw))
            self.inited = True
            self._last_ins = None
        self._emit("info", f"设置初始位姿 ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}°)")
        if self.ext is not None and self.mode in ("slam", "localization"):
            self._ext_start(self.mode, (x, y, yaw))

    # ---- 50 Hz 里程计
    def on_odom(self, t, odom, truth=None, vel=None):
        """odom/truth: (x, y, yaw)。返回融合后的 map 位姿。truth 只用于初始位姿 (启动/复位/被搬动) 和误差统计。"""
        with self.lock:
            teleport = False
            if truth is not None and self._last_truth is not None and self._last_odom is not None:
                dt_ = math.hypot(truth[0] - self._last_truth[0], truth[1] - self._last_truth[1])
                do_ = math.hypot(odom[0] - self._last_odom[0], odom[1] - self._last_odom[1])
                dth = abs(_wrap(truth[2] - self._last_truth[2])) - abs(_wrap(odom[2] - self._last_odom[2]))
                teleport = abs(dt_ - do_) > 0.25 or abs(dth) > 0.3
            if teleport or (self._last_odom is not None and math.hypot(odom[0] - self._last_odom[0], odom[1] - self._last_odom[1]) > 0.5):
                self.hist.clear()
                self.hist_t.clear()
            if t < self.odom_t - 1e-6:              # 仿真复位 (时间回退)
                self.hist.clear()
                self.hist_t.clear()
                teleport = True
            self._last_truth, self._last_odom = truth, odom
            self.odom, self.odom_t, self.truth = odom, t, truth
            if vel is not None:
                self.odom_vel = vel
            self.whist.append((time.time(), odom))
            if teleport:
                self.whist.clear()
                self.whist.append((time.time(), odom))
            if (not self.inited or teleport) and truth is not None and self.ext is not None and self.mode in ("slam", "localization"):
                # 外部定位栈按新的初始位姿重启 (建图模式下被搬动：有保存的地图则转为定位，否则重新建图)
                if self.inited and self.mode == "slam" and self.ext.has_saved(self.scenario):
                    self.mode = "localization"
                self._ext_pending = self.mode
            if (not self.inited or teleport) and truth is not None:
                # 已知初始位姿 (车辆在启动位置/被人工搬到新位置): 等同于实车设置初始位姿
                self.M = compose(truth, inverse(odom))
                self.P = np.diag([1e-4, 1e-4, 3e-5])
                self._last_ins = None
                if self.inited:
                    self.stats["reinit"] += 1
                    self._emit("warn", "车辆被移动/仿真复位，按当前位置重新初始化定位")
                self.inited = True
            self.hist.append(odom)
            self.hist_t.append(t)
            pend = self._ext_pending if (self.ext is not None and truth is not None and self.inited) else None
            if pend:
                self._ext_pending = None
            if self.mode == "ground_truth" and truth is not None:
                self.pose = tuple(truth)
                self.M = compose(truth, inverse(odom))
            else:
                self.pose = compose(self.M, odom)
            if truth is not None:
                self.err_hist.append((t, math.hypot(self.pose[0] - truth[0], self.pose[1] - truth[1]),
                                      _wrap(self.pose[2] - truth[2])))
            pose = self.pose
        if pend:
            threading.Thread(target=self._ext_start, args=(pend, tuple(truth)), daemon=True, name="slam-ext-start").start()
        return pose

    def _odom_at(self, t):
        ht = self.hist_t
        if not ht:
            return self.odom
        i = bisect.bisect_left(ht, t)
        if i <= 0:
            return self.hist[0]
        if i >= len(ht):
            dt = min(max(0.0, t - ht[-1]), 0.1)          # 激光比状态新: 按里程计速度外推
            vx, vy, wz = self.odom_vel
            return compose(self.hist[-1], (vx * dt, vy * dt, wz * dt))
        t0, t1 = ht[i - 1], ht[i]
        a, b = self.hist[i - 1], self.hist[i]
        u = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), _wrap(a[2] + u * _wrap(b[2] - a[2])))

    # ---- 10 Hz 激光
    def on_scan(self, t, ranges, a0, inc, rmax):
        """合并虚拟扫描 (base_link 360°)。有原始激光帧时优先用 on_points (角度精确)。"""
        if self.mode in ("odom", "ground_truth") or not self.inited or self.ext is not None:
            return
        r = np.array([rmax if v is None else float(v) for v in ranges], float)
        n = len(r)
        if n == 0:
            return
        ang = a0 + inc * np.arange(n)
        hit = np.isfinite(r) & (r > 0.05) & (r < rmax - 1e-3)
        r = np.where(np.isfinite(r), r, rmax)
        self.on_points(t, r * np.cos(ang), r * np.sin(ang), hit)

    def on_points(self, t, px, py, hit=None, sx=None, sy=None):
        if self.ext is not None:               # 外部定位栈 (slam_toolbox) 处理激光
            return
        # 限频 (SLAM_BUILTIN_HZ，0 = 不限): 激光 40~50 Hz 时逐帧匹配占掉大半个核，手机 (Android) 默认 15 Hz；
        # 两帧之间的位姿由里程计外推，15 Hz 时 1.2 m/s 下两次修正间隔 8 cm
        if self._min_dt > 0.0 and 0.0 <= t - self._last_proc_t < self._min_dt:
            return
        self._last_proc_t = t
        with self._proc:
            self._on_points(t, px, py, hit, sx, sy)

    def _on_points(self, t, px, py, hit=None, sx=None, sy=None):
        # 等状态 (里程计) 覆盖到激光时间戳，最多 80 ms，避免用旧里程计造成时间错位
        t_end = time.time() + 0.08
        while self.hist_t and self.hist_t[-1] < t - 1e-4 and time.time() < t_end:
            time.sleep(0.005)
        """一帧激光点 (base_link 系，时间戳 t 为仿真时间)。hit=False 的点只用于清空 (无回波方向)；
        sx/sy 为每个点的激光安装位置 (建图时光束起点)。"""
        if self.mode in ("odom", "ground_truth") or not self.inited or len(px) == 0:
            return
        px, py = np.asarray(px, float), np.asarray(py, float)
        hit = np.ones(len(px), bool) if hit is None else np.asarray(hit, bool)
        r = np.hypot(px, py)
        h, tl, hw = self.body
        self_pts = (px < h + 0.05) & (px > -tl - 0.05) & (np.abs(py) < hw + 0.05)
        hit = hit & ~self_pts & (r > 0.05)
        with self.lock:
            od = self._odom_at(t)
            pred = compose(self.M, od)
            # 里程计预测不确定度随运动增长
            if self._last_scan_od is not None:
                d = compose(inverse(self._last_scan_od), od)
                dist, dth = math.hypot(d[0], d[1]), abs(d[2])
                q_xy = (0.02 * dist) ** 2 + (0.002) ** 2 * (dist > 1e-4)
                q_th = (0.02 * dth + 0.003 * dist) ** 2
                self._fz_dist += dist
                self.P = self.P + np.diag([q_xy, q_xy, q_th]) + np.diag([1e-7, 1e-7, 1e-8])
            self._last_scan_od = od
            fields = self.fields
            mode = self.mode
            gm_empty = self.gm.empty()
        info = {}
        pose = pred
        ok = False
        if fields is not None:
            idx = np.nonzero(hit)[0]
            if len(idx) > self.max_points:
                idx = idx[np.linspace(0, len(idx) - 1, self.max_points).astype(int)]
            t0 = time.perf_counter()
            pose, cov, info = match(fields, px[idx], py[idx], pred, P0=self.P)
            self.stats["match_ms"] = round((time.perf_counter() - t0) * 1000, 2)
            ok = info["inliers"] > 0.35 and info["n"] > 30
            with self.lock:
                if ok:
                    od_now = self._odom_at(t)
                    self.M = compose(pose, inverse(od_now))
                    self.P = cov
                    self.pose = compose(self.M, self.odom)
                    self.stats["matches"] += 1
                else:
                    self.stats["rejects"] += 1
                self.stats["last_info"] = {k: round(v, 4) if isinstance(v, float) else v for k, v in info.items()}
                self.stats["last_info"]["accepted"] = ok
            if mode == "localization" and self._fz_auto:
                self._auto_unfreeze(t, info)
            self._hz_n += 1
            if time.time() - self._hz_t > 2.0:
                self.stats["hz"] = round(self._hz_n / (time.time() - self._hz_t), 1)
                self._hz_n, self._hz_t = 0, time.time()
        # 建图 (slam 模式): 匹配成功 (或地图为空) 且移动超过阈值时插入
        if mode == "slam" and (ok or gm_empty or fields is None):
            li = self._last_ins
            moved = li is None or math.hypot(pose[0] - li[0], pose[1] - li[1]) > 0.08 or abs(_wrap(pose[2] - li[2])) > 0.05 \
                or (t - self._last_ins_t) > 3.0
            if moved:
                t0 = time.perf_counter()
                with self.lock:
                    self.gm.insert(pose, px, py, hit, sx, sy)
                    self._last_ins, self._last_ins_t = pose, t
                self.stats["insert_ms"] = round((time.perf_counter() - t0) * 1000, 1)
                self._rebuild_fields_async(sync=fields is None)
                if self._fz_on and self.ext is None:
                    self._auto_freeze()

    def _auto_freeze(self):
        """建图模式: 地图收敛 (行驶 _fz_travel 米，已知栅格数增长 < 0.5%) → 保存并转定位模式"""
        self._fz_ins += 1
        if self._fz_ins % 5:
            return
        with self.lock:
            known = int(np.count_nonzero(np.abs(self.gm.L) > 0.5))
        self.stats["known"], self.stats["still_m"] = known, round(self._fz_dist, 1)
        if known > self._fz_known * 1.005 + 50:
            self._fz_known, self._fz_dist = known, 0.0
            return
        if self._fz_dist < self._fz_travel:
            return
        try:
            self.save()
        except Exception as e:  # noqa
            self._emit("warn", f"SLAM 地图自动保存失败，继续建图: {e}")
            self._fz_dist = 0.0
            return
        with self.lock:
            self.mode = "localization"
            self._fz_auto, self._fz_low_t = True, None
        self._emit("info", f"SLAM 地图已收敛 (行驶 {self._fz_dist:.0f} m 没有新增区域)，自动保存并转为定位模式",
                   "开进未建图区域时会自动回到建图模式；SLAM_AUTO_FREEZE=0 关闭")

    def _auto_unfreeze(self, t, info):
        """自动冻结后的定位模式: 内点率连续 3 s 低于 0.6 (正常 > 0.95) → 开进了没建过图的区域，回到建图模式"""
        if not info or info.get("inliers", 1.0) >= 0.6:
            self._fz_low_t = None
            return
        if self._fz_low_t is None:
            self._fz_low_t = t
        elif t - self._fz_low_t > 3.0:
            with self.lock:
                self.mode = "slam"
                self._fz_auto, self._fz_low_t = False, None
                self._fz_dist, self._fz_known = 0.0, 0
            self._emit("info", "定位匹配内点率持续偏低 (进入未建图区域)，自动回到建图模式")

    def _rebuild_fields_async(self, sync=False, force=False):
        if self._field_busy and not sync:
            return
        if self.fields is not None and not force and self.fields.get("rev") == self.gm.rev:
            return

        def job():
            try:
                t0 = time.perf_counter()
                with self.lock:
                    gm = self.gm
                    L, Hd = gm.L.copy(), gm.Hd.copy()
                    snap = GridMap(gm.res)
                    snap.ox, snap.oy, snap.L, snap.Hd, snap.rev = gm.ox, gm.oy, L, Hd, gm.rev
                f = build_fields(snap)
                with self.lock:
                    if gm is self.gm:
                        self.fields = f
                self.stats["field_ms"] = round((time.perf_counter() - t0) * 1000, 1)
            finally:
                self._field_busy = False

        self._field_busy = True
        if sync:
            job()
        else:
            threading.Thread(target=job, daemon=True, name="slam-field").start()

    # ---- 保存 / 加载
    def _paths(self, sid):
        return (os.path.join(self.map_dir, f"{sid}.npz"), os.path.join(self.map_dir, f"{sid}.pgm"),
                os.path.join(self.map_dir, f"{sid}.yaml"))

    def has_saved(self, sid=None):
        sid = sid or self.scenario
        if self.ext is not None:
            return self.ext.has_saved(sid)
        return bool(sid) and os.path.exists(self._paths(sid)[0])

    def save(self, sid=None):
        sid = sid or self.scenario
        if not sid:
            raise ValueError("未知场景")
        if self.ext is not None:
            if not self.ext.running():
                raise ValueError("slam_toolbox 未运行 (定位模式为 odom/ground_truth 时不能保存)")
            try:
                out = self.ext.save(sid)
            except Exception as e:  # noqa
                raise ValueError(f"slam_toolbox 保存失败: {e}")
            self._emit("info", f"slam_toolbox 地图已保存 [{sid}]", out.get("posegraph", ""))
            return out
        with self.lock:
            if self.gm.empty():
                raise ValueError("地图为空")
            gm = self.gm
            os.makedirs(self.map_dir, exist_ok=True)
            npz, pgm_p, yaml_p = self._paths(sid)
            np.savez_compressed(npz, L=gm.L, Hd=gm.Hd, origin=np.array([gm.ox, gm.oy]), res=np.array([gm.res]))
            pgm, yaml = gm.to_pgm_yaml(os.path.basename(pgm_p))
        with open(pgm_p, "wb") as f:
            f.write(pgm)
        with open(yaml_p, "w") as f:
            f.write(yaml)
        self._emit("info", f"SLAM 地图已保存 [{sid}]", npz)
        return {"npz": npz, "pgm": pgm_p, "yaml": yaml_p}

    def delete_saved(self, sid=None):
        sid = sid or self.scenario
        if self.ext is not None:
            return self.ext.delete_saved(sid)
        n = 0
        for p in self._paths(sid):
            if os.path.exists(p):
                os.remove(p)
                n += 1
        return n

    def _load(self, sid):
        if not sid or not self.has_saved(sid):
            return False
        try:
            z = np.load(self._paths(sid)[0])
            gm = GridMap(float(z["res"][0]))
            gm.L, gm.Hd = z["L"].astype(np.float32), z["Hd"].astype(np.float32)
            gm.ox, gm.oy = float(z["origin"][0]), float(z["origin"][1])
            gm.rev = 1
            self.gm = gm
            self.fields = build_fields(gm)
            return True
        except Exception as e:  # noqa
            self._emit("warn", f"加载 SLAM 地图失败: {e}")
            return False

    # ---- 状态 / 地图输出
    def status(self):
        with self.lock:
            cov = self.P
            e = list(self.err_hist)
            st = {
                "mode": self.mode, "want_mode": self.want_mode, "scenario": self.scenario, "inited": self.inited,
                "engine": self.engine, "auto_frozen": self._fz_auto and self.mode == "localization", "ext": self.ext.status() if self.ext is not None else None,
                "pose": {"x": round(self.pose[0], 4), "y": round(self.pose[1], 4), "yaw": round(self.pose[2], 5)},
                "map_to_odom": {"x": round(self.M[0], 4), "y": round(self.M[1], 4), "yaw": round(self.M[2], 5)},
                "odom": {"x": round(self.odom[0], 4), "y": round(self.odom[1], 4), "yaw": round(self.odom[2], 5)},
                "std_xy_mm": round(1000 * math.sqrt(max(cov[0, 0], cov[1, 1])), 1),
                "std_yaw_deg": round(math.degrees(math.sqrt(max(cov[2, 2], 0))), 3),
                "map": {"res": self.gm.res, "w": self.gm.shape[1] if not self.gm.empty() else 0,
                        "h": self.gm.shape[0] if not self.gm.empty() else 0, "updates": self.gm.updates,
                        "saved": self.has_saved()},
                "stats": dict(self.stats),
            }
        if self.engine == "slam_toolbox":
            g = self.ext.grid()
            st["map"] = {"res": g[1] if g else None, "w": g[0].shape[1] if g else 0, "h": g[0].shape[0] if g else 0,
                         "updates": self.ext.map_rev, "saved": self.has_saved()}
            st["std_xy_mm"] = st["std_yaw_deg"] = None
        if self.truth is not None:
            st["err_mm"] = round(1000 * math.hypot(self.pose[0] - self.truth[0], self.pose[1] - self.truth[1]), 1)
            st["err_yaw_deg"] = round(math.degrees(_wrap(self.pose[2] - self.truth[2])), 3)
            if e:
                tl = e[-1][0]
                rec = [v for v in e if v[0] >= tl - 30.0]
                st["err_max_30s_mm"] = round(1000 * max(v[1] for v in rec), 1)
                st["err_rms_30s_mm"] = round(1000 * math.sqrt(sum(v[1] ** 2 for v in rec) / len(rec)), 1)
        return st

    def brief(self):
        """遥测用的简要定位状态 (50 Hz 路径上调用，保持轻量)"""
        cov = self.P
        d = {"mode": self.mode, "engine": self.engine, "auto_frozen": self._fz_auto and self.mode == "localization", "std_xy_mm": round(1000 * math.sqrt(max(cov[0, 0], cov[1, 1])), 1),
             "std_yaw_deg": round(math.degrees(math.sqrt(max(cov[2, 2], 0.0))), 3),
             "score": self.stats["last_info"].get("score"), "accepted": self.stats["last_info"].get("accepted"),
             "map_rev": self.ext.map_rev if self.engine == "slam_toolbox" else self.gm.rev}
        if self.engine == "slam_toolbox":
            d["std_xy_mm"] = d["std_yaw_deg"] = None       # slam_toolbox 不输出协方差
        if self.truth is not None:
            d["err_mm"] = round(1000 * math.hypot(self.pose[0] - self.truth[0], self.pose[1] - self.truth[1]), 1)
            d["err_yaw_deg"] = round(math.degrees(_wrap(self.pose[2] - self.truth[2])), 3)
        return d

    def map_payload(self, step=1):
        """占据栅格 (0 未知/1 空闲/2 占据) base64，供界面叠加"""
        if self.engine == "slam_toolbox":
            g = self.ext.grid()
            if g is None:
                return {"empty": True, "mode": self.mode, "engine": self.engine}
            c, res, ox, oy, rev = g
        else:
            with self.lock:
                if self.gm.empty():
                    return {"empty": True, "mode": self.mode}
                c = self.gm.classes()
                ox, oy, res, rev = self.gm.ox, self.gm.oy, self.gm.res, self.gm.rev
        if step > 1:
            h, w = c.shape
            c = c[: h - h % step, : w - w % step].reshape(h // step, step, w // step, step).max(axis=(1, 3))
            res *= step
        # 去掉四周全未知的边
        nz = np.nonzero(c)
        if len(nz[0]):
            y0, y1, x0, x1 = nz[0].min(), nz[0].max() + 1, nz[1].min(), nz[1].max() + 1
            c = c[y0:y1, x0:x1]
            ox, oy = ox + x0 * res, oy + y0 * res
        return {"empty": False, "mode": self.mode, "rev": rev, "res": res, "origin": [round(ox, 4), round(oy, 4)],
                "w": int(c.shape[1]), "h": int(c.shape[0]), "data": base64.b64encode(c.tobytes()).decode()}

    def pgm_yaml(self):
        if self.engine == "slam_toolbox":
            g = self.ext.grid()
            if g is None:
                return None, None
            c, res, ox, oy, _ = g
            gm = GridMap(res)
            gm.ox, gm.oy = ox, oy
            gm.L = np.where(c == 2, 1.0, np.where(c == 1, -1.0, 0.0)).astype(np.float32)
            return gm.to_pgm_yaml(f"{self.scenario or 'slam'}.pgm")
        with self.lock:
            if self.gm.empty():
                return None, None
            return self.gm.to_pgm_yaml(f"{self.scenario or 'slam'}.pgm")
