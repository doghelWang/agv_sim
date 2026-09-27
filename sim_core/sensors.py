#!/usr/bin/env python3
"""
传感器模型: 多激光 (按 cmodel 安装位姿/倒装/视场/量程/分辨率) + 融合扫描 + 编码器里程计 + IMU + 读码相机
"""

import math
import random
from typing import Dict, List, Optional, Tuple

import numpy as np

import ctypes
import os

from . import native
from .kinematics import ChassisKinematics, se2_integrate, wrap
from .world import World

# 2D 激光: native = 噪声/丢点在 C 里做 (有 MuJoCo 时求交仍由 mj_multiRay 完成)；engine = 原 Python/numpy 路径
LIDAR_RAY = os.environ.get("SIM_LIDAR_RAY", "native").strip().lower()


def _native_lidar() -> bool:
    return native.lib is not None and LIDAR_RAY != "engine"


# ======================================================================
# 激光
# ======================================================================
class LidarSensor:
    def __init__(self, cfg: dict):
        self.cfg = dict(cfg)
        self.name = cfg["name"]
        self.frame_id = f"{self.name}_link"
        self.mx, self.my, self.mz = float(cfg.get("x", 0)), float(cfg.get("y", 0)), float(cfg.get("z", 0.3))
        self.yaw = float(cfg.get("yaw", 0.0))
        roll, pitch = float(cfg.get("roll", 0.0)), float(cfg.get("pitch", 0.0))
        # 倒装 (roll≈180°) → 扫描方向在机体系中反向
        self.sign = 1.0 if math.cos(roll) * math.cos(pitch) >= 0 else -1.0
        self.native_res = math.radians(float(cfg.get("resolution_deg", 0.25)))
        self.fov = math.radians(float(cfg.get("fov_deg", 270.0)))
        self.range_min = float(cfg.get("min_range", 0.05))
        self.native_range_max = float(cfg.get("max_range", 25.0))
        self.freq_hz = float(cfg.get("freq_hz", 15.0))
        self.noise_std = float(cfg.get("range_noise_std", 0.015))
        self.noise_prop = 0.002          # 距离相关噪声 (0.2%)
        self.dropout = 0.002             # 随机丢点
        self.range_max = self.native_range_max
        self.set_resolution(self.native_res)
        self.last_ms = 0.0

    def set_resolution(self, res_rad: float):
        res_rad = max(math.radians(0.05), res_rad)
        full = self.fov >= 2 * math.pi - 1e-3
        if full:
            n = int(round(2 * math.pi / res_rad))
            self.angle_min = -math.pi
            self.angle_inc = 2 * math.pi / n
            self.angle_max = self.angle_min + (n - 1) * self.angle_inc
        else:
            n = int(round(self.fov / res_rad)) + 1
            self.angle_min = -self.fov / 2
            self.angle_max = self.fov / 2
            self.angle_inc = self.fov / (n - 1)
        self.n = n
        self.local_angles = self.angle_min + np.arange(n) * self.angle_inc

    def scan(self, world: World, x: float, y: float, th: float, noise: bool = True) -> np.ndarray:
        c, s = math.cos(th), math.sin(th)
        ox = x + c * self.mx - s * self.my
        oy = y + s * self.mx + c * self.my
        if _native_lidar():
            if not hasattr(self, "_rng"):
                self._rng = native.Rng()
            if world.engine is not None:     # MuJoCo 求交 (经 C 调 mj_multiRay)，噪声/丢点在 C 里做
                r = world.engine.raycast2d(ox, oy, max(0.005, self.mz), th + self.yaw + self.sign * self.local_angles, self.range_max)
                native.lib.sc_lidar_post(r.ctypes.data, self.n, self.range_min, 1 if noise else 0, self.noise_std, self.noise_prop,
                                         self.dropout, self._rng.addr)
                return r
            r = np.empty(self.n)
            segs = world.segments
            native.lib.sc_lidar_scan(native.ptr(segs), len(segs), self.mz, ox, oy, th + self.yaw, self.sign, self.angle_min,
                                     self.angle_inc, self.n, self.range_max, self.range_min, 1 if noise else 0, self.noise_std,
                                     self.noise_prop, self.dropout, self._rng.addr, r.ctypes.data)
            return r
        world_angles = th + self.yaw + self.sign * self.local_angles
        r = world.raycast(ox, oy, world_angles, self.range_max, min_seg_height=self.mz)
        if noise:
            fin = np.isfinite(r)
            k = int(fin.sum())
            if k:
                r[fin] = r[fin] + np.random.normal(0.0, 1.0, k) * (self.noise_std + self.noise_prop * r[fin])
            if self.dropout > 0:
                r[np.random.random(self.n) < self.dropout] = np.inf
        r[(r < self.range_min)] = self.range_min
        return r

    def points_base(self, ranges: np.ndarray) -> np.ndarray:
        """激光测距 → 机体系 (base_link) 平面点 (仅有限值)"""
        fin = np.isfinite(ranges)
        a = self.yaw + self.sign * self.local_angles[fin]
        rr = ranges[fin]
        return np.stack([self.mx + rr * np.cos(a), self.my + rr * np.sin(a)], axis=1)


def _rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class Lidar3DSensor:
    """
    Livox Mid-360 / Mid-360S 类非重复扫描 3D 激光
      视场 360° × (vfov_min ~ vfov_max)，默认 -7° ~ +52°；盲区 0.1 m；量程 40 m @10%
      非重复扫描: 以低差异序列 (黄金分割) 生成方向，逐帧错位 → 积分时间越长覆盖越致密 (与 Livox 特性一致)
      噪声: 距离 1σ ≤2 cm，角度 1σ 0.15°；4 条激光线 (line 0~3)
      输出: 传感器系点云 (x, y, z, intensity, tag, line, timestamp)，格式对齐 livox_ros_driver2 PointCloud2
    """
    PHI1 = 0.6180339887498949
    PHI2 = 0.7548776662466927   # plastic number 共轭，与 PHI1 组成 2D 低差异序列

    def __init__(self, cfg: dict, points_per_frame: int = 10000):
        self.cfg = dict(cfg)
        self.name = cfg["name"]
        self.frame_id = f"{self.name}_link"
        self.mx, self.my, self.mz = float(cfg.get("x", 0)), float(cfg.get("y", 0)), float(cfg.get("z", 0.3))
        self.R = _rot(float(cfg.get("roll", 0)), float(cfg.get("pitch", 0)), float(cfg.get("yaw", 0)))
        self.vmin = math.radians(float(cfg.get("vfov_min_deg", -7.0)))
        self.vmax = math.radians(float(cfg.get("vfov_max_deg", 52.0)))
        self.range_min = float(cfg.get("min_range", 0.1))
        self.range_max = self.native_range_max = float(cfg.get("max_range", 40.0))
        self.freq_hz = float(cfg.get("freq_hz", 10.0))
        self.noise_std = float(cfg.get("range_noise_std", 0.02))
        self.ang_noise = math.radians(float(cfg.get("angular_noise_deg", 0.15)))
        self.lines = int(cfg.get("lines", 4))
        self.real_points = int(float(cfg.get("point_rate", 200000)) / max(1.0, self.freq_hz))
        self.n = int(points_per_frame)
        self.frame = 0
        self.last_ms = 0.0
        # 2D 兼容参数 (merge/兼容旧接口)
        self.fov = 2 * math.pi
        self.angle_min, self.angle_inc = -math.pi, math.radians(0.5)

    def scan(self, world: World, x: float, y: float, th: float, noise: bool = True):
        n = self.n
        i = np.arange(n, dtype=np.float64) + self.frame * n
        self.frame += 1
        az = 2 * math.pi * np.mod(i * self.PHI1, 1.0)
        # 仰角: 球面带内均匀 (按 sin 均匀采样)
        s0, s1 = math.sin(self.vmin), math.sin(self.vmax)
        el = np.arcsin(s0 + (s1 - s0) * np.mod(i * self.PHI2, 1.0))
        if noise and self.ang_noise > 0:
            az = az + np.random.normal(0, self.ang_noise, n)
            el = el + np.random.normal(0, self.ang_noise, n)
        ce = np.cos(el)
        d_s = np.stack([ce * np.cos(az), ce * np.sin(az), np.sin(el)], axis=1)       # 传感器系
        c, s = math.cos(th), math.sin(th)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        Rw = Rz @ self.R
        d_w = d_s @ Rw.T
        o = (x + c * self.mx - s * self.my, y + s * self.mx + c * self.my, self.mz)
        r = world.raycast3d(o, d_w, self.range_max, self.range_min)
        hit = np.isfinite(r)
        r = r[hit]
        if noise and len(r):
            r = r + np.random.normal(0, self.noise_std, len(r))
        pts = d_s[hit] * r[:, None]
        line = (np.nonzero(hit)[0] % self.lines).astype(np.uint8)
        # 反射强度: 近似与距离/入射角相关 (0~255)
        inten = np.clip(180.0 - 3.0 * r + np.random.normal(0, 8, len(r)), 5, 255).astype(np.float32)
        tstamp = np.nonzero(hit)[0].astype(np.float64) * (1.0 / max(1.0, self.freq_hz) / n)   # 帧内相对时间 s
        return {"points": pts.astype(np.float32), "intensity": inten, "line": line, "offset_time": tstamp}

    def points_base(self, cloud) -> np.ndarray:
        """传感器系点 → 机体系 (N×3)"""
        P = cloud["points"].astype(np.float64) @ self.R.T
        P[:, 0] += self.mx; P[:, 1] += self.my; P[:, 2] += self.mz
        return P


def slice_to_scan(pts_base: np.ndarray, zmin: float, zmax: float, n_bins: int, range_max: float) -> np.ndarray:
    """3D 点云 → 2D 虚拟扫描 (机体系，类似 pointcloud_to_laserscan)，高度带 [zmin, zmax]"""
    out = np.full(n_bins, np.inf)
    if len(pts_base) == 0:
        return out
    m = (pts_base[:, 2] >= zmin) & (pts_base[:, 2] <= zmax)
    P = pts_base[m]
    if len(P) == 0:
        return out
    d = np.hypot(P[:, 0], P[:, 1])
    idx = np.clip(((np.arctan2(P[:, 1], P[:, 0]) + math.pi) / (2 * math.pi / n_bins)).astype(int), 0, n_bins - 1)
    np.minimum.at(out, idx, d)
    out[out > range_max] = np.inf
    return out


def merge_scans(lidars: List[LidarSensor], ranges: List[np.ndarray], n_bins: int, range_max: float) -> np.ndarray:
    """所有激光点 → 以 base_link 为中心的 360° 虚拟扫描 (每个角度取最近点)"""
    out = np.full(n_bins, np.inf)
    if native.lib is not None:
        for l, r in zip(lidars, ranges):
            r = np.ascontiguousarray(r, dtype=np.float64)
            native.lib.sc_merge_add(out.ctypes.data, n_bins, r.ctypes.data, len(r), l.mx, l.my, l.yaw, l.sign, l.angle_min, l.angle_inc)
        native.lib.sc_merge_finish(out.ctypes.data, n_bins, range_max)
        return out
    return merge_scans_py(lidars, ranges, n_bins, range_max)


def merge_scans_py(lidars, ranges, n_bins: int, range_max: float) -> np.ndarray:
    out = np.full(n_bins, np.inf)
    inc = 2 * math.pi / n_bins
    for l, r in zip(lidars, ranges):
        pts = l.points_base(r)
        if len(pts) == 0:
            continue
        d = np.hypot(pts[:, 0], pts[:, 1])
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        idx = np.clip(((ang + math.pi) / inc).astype(int), 0, n_bins - 1)
        np.minimum.at(out, idx, d)
    out[out > range_max] = np.inf
    return out


# ======================================================================
# 编码器里程计 (带轮径误差、量化、打滑) —— /odom 会随时间漂移，与真值分离
# ======================================================================
class WheelOdometry:
    def __init__(self, kin: ChassisKinematics, enabled: bool = True, seed: Optional[int] = None):
        self.kin = kin
        self.enabled = enabled
        rnd = random.Random(seed)
        # 每个驱动轮的有效半径误差 (轮胎磨损/载重压缩) σ=0.1% (标定后残差)
        self.radius_err = {w.name: (1.0 + rnd.gauss(0, 0.001)) if enabled else 1.0 for w in kin.wheels}
        self.steer_bias = {w.name: math.radians(rnd.gauss(0, 0.08)) if enabled else 0.0 for w in kin.wheels}
        self.cpr = 4096 * 4
        self.steer_sigma = math.radians(0.05)      # 舵角读数噪声
        self.x = self.y = self.th = 0.0
        self.vx = self.vy = self.wz = 0.0
        self._nat = None
        if getattr(kin, "native", False):
            n = len(kin.wheels)
            self._nat = ((ctypes.c_double * n)(*[self.radius_err[w.name] for w in kin.wheels]),
                         (ctypes.c_double * n)(*[self.steer_bias[w.name] for w in kin.wheels]),
                         (ctypes.c_double * 6)(), native.Rng(rnd.getrandbits(64)))

    def reset(self, x=0.0, y=0.0, th=0.0):
        self.x, self.y, self.th = x, y, th
        self.vx = self.vy = self.wz = 0.0

    def update(self, dt: float):
        kin = self.kin
        if self.enabled and self._nat is not None and kin.native:
            re, sb, pose, rng = self._nat
            for c, w in zip(kin._cw, kin.wheels):
                c.steer, c.speed = w.steer, w.speed
            pose[0], pose[1], pose[2] = self.x, self.y, self.th
            native.lib.sc_odom_update(ctypes.addressof(kin._cw), len(kin.wheels), 1 if kin.holonomic else 0, kin.axle_x,
                                      ctypes.addressof(re), ctypes.addressof(sb), self.steer_sigma, float(self.cpr), dt,
                                      rng.addr, ctypes.addressof(pose))
            self.x, self.y, self.th, self.vx, self.vy, self.wz = pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]
            return
        if not self.enabled:
            vx, vy, wz = kin.vx, kin.vy, kin.wz
        else:
            spd, st = {}, {}
            for w in kin.wheels:
                if not w.driven:
                    continue
                # 编码器量化: 电机端增量 → 轮速
                ticks_per_m = self.cpr * w.gear_ratio / (2 * math.pi * w.radius)
                q = round(w.speed * dt * ticks_per_m) / max(ticks_per_m * dt, 1e-9)
                spd[w.name] = q * self.radius_err[w.name]
                if w.kind == "steer":
                    st[w.name] = w.steer + self.steer_bias[w.name] + random.gauss(0, self.steer_sigma)
            vx, vy, wz, _ = kin.forward(steer_override=st, speed_override=spd)
        self.vx, self.vy, self.wz = vx, vy, wz
        self.x, self.y, self.th = se2_integrate(self.x, self.y, self.th, vx, vy, wz, dt)


class GroundSlip:
    """真值运动中的轮地打滑: 加减速/急转时实际位移略小于轮子转动 (纵向打滑率 ~ 加速度)"""

    def __init__(self, enabled=True):
        self.enabled = enabled
        self.prev = (0.0, 0.0, 0.0)

    def apply(self, vx, vy, wz, dt):
        if not self.enabled:
            return vx, vy, wz
        pvx, pvy, pwz = self.prev
        acc = math.hypot(vx - pvx, vy - pvy) / max(dt, 1e-3)
        self.prev = (vx, vy, wz)
        k = 1.0 - min(0.03, 0.004 + 0.01 * acc) - random.gauss(0, 0.002)
        kw = 1.0 - min(0.05, 0.01 * abs(wz)) - random.gauss(0, 0.002)
        return vx * k, vy * k, wz * kw


# ======================================================================
# IMU
# ======================================================================
class ImuModel:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.gyro_bias = random.gauss(0, math.radians(0.05)) if enabled else 0.0
        self.gyro_noise = math.radians(0.02) if enabled else 0.0
        self.acc_noise = 0.02 if enabled else 0.0
        self.prev = (0.0, 0.0)
        self.yaw = 0.0

    def sample(self, vx, vy, wz, th, dt) -> dict:

        ax = (vx - self.prev[0]) / max(dt, 1e-3) - wz * vy
        ay = (vy - self.prev[1]) / max(dt, 1e-3) + wz * vx
        self.prev = (vx, vy)
        self.gyro_bias += random.gauss(0, math.radians(0.001)) if self.enabled else 0.0
        gz = wz + self.gyro_bias + random.gauss(0, self.gyro_noise)
        self.yaw = wrap(self.yaw + gz * dt)
        return {"wz": gz, "ax": ax + random.gauss(0, self.acc_noise), "ay": ay + random.gauss(0, self.acc_noise),
                "az": 9.81 + random.gauss(0, self.acc_noise), "yaw": self.yaw}


# ======================================================================
# 读码相机: 朝下 → 地面二维码 (拓扑节点)，朝前/默认 → 工位标签
# ======================================================================
class CodeReader:
    def __init__(self, cams: List[dict]):
        self.cams = cams or []

    def detect(self, x, y, th, ground_tags: List[dict], station_tags: List[dict]) -> List[dict]:
        out = []
        c, s = math.cos(th), math.sin(th)
        for cam in self.cams:
            cx = x + c * cam["x"] - s * cam["y"]
            cy = y + s * cam["x"] + c * cam["y"]
            orient = cam.get("orientation", "")
            if orient.endswith("DOWN"):
                win = float(cam.get("read_window_m", 0.05))
                for t in ground_tags:
                    dx, dy = t["x"] - cx, t["y"] - cy
                    if abs(dx) <= win and abs(dy) <= win:
                        rx = c * dx + s * dy
                        ry = -s * dx + c * dy
                        out.append({"camera": cam["name"], "type": "ground_qr", "id": t["id"], "name": t.get("name", t["id"]),
                                    "rel_x": round(rx + random.gauss(0, 0.0015), 4), "rel_y": round(ry + random.gauss(0, 0.0015), 4),
                                    "rel_yaw_deg": round(math.degrees(wrap(t.get("yaw", 0.0) - th)) + random.gauss(0, 0.1), 2),
                                    "distance_m": round(math.hypot(dx, dy), 4), "bearing_deg": 0.0})
        return out


class StationTagCamera:
    """前视工位标签识别 (兼容旧 VisionSimulator 输出格式)"""

    def __init__(self, fov_deg=85.0, max_dist=4.0, mount_x=0.0):
        self.fov = math.radians(fov_deg)
        self.max_dist = max_dist
        self.mount_x = mount_x

    def detect(self, x, y, th, tags: List[dict]) -> List[dict]:
        out = []
        ox, oy = x + math.cos(th) * self.mount_x, y + math.sin(th) * self.mount_x
        for m in tags:
            dx, dy = m["x"] - ox, m["y"] - oy
            d = math.hypot(dx, dy)
            if d > self.max_dist:
                continue
            b = wrap(math.atan2(dy, dx) - th)
            if abs(b) <= self.fov / 2:
                out.append({"id": m["id"], "name": m["name"], "type": "station_tag", "distance_m": round(d, 2),
                            "bearing_deg": round(math.degrees(b), 1),
                            "rel_x": round(math.cos(-th) * dx - math.sin(-th) * dy, 3),
                            "rel_y": round(math.sin(-th) * dx + math.cos(-th) * dy, 3),
                            "rel_yaw_deg": round(math.degrees(wrap(m.get("yaw", 0.0) - th)), 1)})
        return out
