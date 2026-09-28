#!/usr/bin/env python3
"""
相机类传感器仿真: 单目 RGB / 双目 (左右 RGB + 深度) / ToF (深度 + 幅度 + 点云)

成像方式: 逐像素光线投射 (MuJoCo mj_multiRay，返回距离/几何体/法向) + Lambert 着色，不依赖 OpenGL，
          树莓派 Docker 内可用；无 MuJoCo 时退回 numpy 几何求交 (只有深度灰度)。
坐标约定:
  安装位姿 (x,y,z,roll,pitch,yaw) 描述 <name>_link 在 base_link 下的位姿，link 系 x 轴 = 光轴 (前)
  图像/点云使用 ROS 光学坐标系 <name>_optical_frame: z 前、x 右、y 下
噪声模型:
  RGB   : 高斯像素噪声 σ=2 (8bit)
  双目  : 深度 σz = z²·σd / (f·b)，σd = 0.08 px (亚像素匹配)，超出基线可测范围置 NaN
  ToF   : σ = 0.5 cm + 0.4 %·z，幅度 ∝ cosθ / z²，低幅度 (<阈值) 像素无效；多径忽略
"""

import math
import os
import random
from typing import Dict, List, Optional

import numpy as np

# 着色 + 噪声 + 量化在 C 里做 (sim_core/native sc_cam_shade)；SIM_NATIVE_CAM=0 用 numpy
NATIVE_CAM = os.environ.get("SIM_NATIVE_CAM", "1") != "0"

CAMERA_TYPES = ("camera", "stereo", "tof")

# 型号参数库 (人工添加传感器时的默认值，可在模型补全界面修改)
CAMERA_LIBRARY = {
    "camera": {"model": "通用 USB RGB 相机", "width": 320, "height": 240, "hfov_deg": 70.0, "fps": 10.0,
               "range_max": 30.0, "pixel_noise": 2.0},
    "stereo": {"model": "Intel RealSense D435 (仿)", "width": 424, "height": 240, "hfov_deg": 87.0, "fps": 10.0,
               "baseline_m": 0.05, "range_min": 0.3, "range_max": 10.0, "subpixel_px": 0.08, "pixel_noise": 2.0},   # D435 原生 1280×720，仿真默认 424×240
    "tof": {"model": "PMD/Sunny 224×172 ToF (仿)", "width": 224, "height": 172, "hfov_deg": 62.0, "fps": 10.0,
            "range_min": 0.1, "range_max": 4.0, "noise_abs_m": 0.005, "noise_rel": 0.004},
}


def _rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class CameraSensor:
    kind = "camera"

    def __init__(self, cfg: dict):
        lib = CAMERA_LIBRARY.get(cfg.get("type", self.kind), {})
        c = dict(lib)
        c.update({k: v for k, v in cfg.items() if v is not None})
        self.cfg = c
        self.name = c["name"]
        self.frame_id = f"{self.name}_optical_frame"
        self.mx, self.my, self.mz = float(c.get("x", 0)), float(c.get("y", 0)), float(c.get("z", 0.5))
        self.R = _rot(float(c.get("roll", 0)), float(c.get("pitch", 0)), float(c.get("yaw", 0)))
        self.W, self.H = int(c.get("width", 640)), int(c.get("height", 480))
        self.hfov = math.radians(float(c.get("hfov_deg", 70.0)))
        self.fps = float(c.get("fps", 10.0))
        self.range_min = float(c.get("range_min", 0.05))
        self.range_max = float(c.get("range_max", 30.0))
        self.fx = (self.W / 2.0) / math.tan(self.hfov / 2.0)
        self.fy = self.fx
        self.cx, self.cy = (self.W - 1) / 2.0, (self.H - 1) / 2.0
        self.vfov = 2 * math.atan((self.H / 2.0) / self.fy)
        u, v = np.meshgrid(np.arange(self.W), np.arange(self.H))
        xo = (u - self.cx) / self.fx
        yo = (v - self.cy) / self.fy
        d_link = np.stack([np.ones_like(xo), -xo, -yo], -1).reshape(-1, 3)    # link 系: x 前 y 左 z 上
        self.axis_cos = 1.0 / np.linalg.norm(d_link, axis=1)                  # 光轴方向分量 (深度 = 距离 × cos)
        self.d_link = d_link * self.axis_cos[:, None]
        self.frame = 0
        self.last_ms = 0.0

    # ------------------------------------------------------------------
    def K(self) -> List[float]:
        return [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]

    def info(self) -> dict:
        return {"name": self.name, "type": self.kind, "model": self.cfg.get("model"), "frame_id": self.frame_id,
                "width": self.W, "height": self.H, "hfov_deg": round(math.degrees(self.hfov), 2),
                "vfov_deg": round(math.degrees(self.vfov), 2), "fps": self.fps, "K": [round(k, 4) for k in self.K()],
                "range": [self.range_min, self.range_max],
                "mount": {k: float(self.cfg.get(k, 0.0)) for k in ("x", "y", "z", "roll", "pitch", "yaw")},
                "streams": self.streams()}

    def streams(self) -> List[str]:
        return ["rgb"]

    def _pose(self, x, y, th, offset_y=0.0):
        c, s = math.cos(th), math.sin(th)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        Rw = Rz @ self.R
        lo = np.array([self.mx, self.my, self.mz]) + self.R @ np.array([0.0, offset_y, 0.0])
        o = np.array([x + c * lo[0] - s * lo[1], y + s * lo[0] + c * lo[1], lo[2]])
        return o, self.d_link @ Rw.T

    def _cast(self, world, engine, x, y, th, normals=True, offset_y=0.0):
        self._pose_now = (x, y, th)
        o, dw = self._pose(x, y, th, offset_y)
        if engine is not None:
            dist, gid, nrm = engine.cast(o, dw, self.range_max, want_normal=normals)
        else:
            dist = world.raycast3d(tuple(o), dw, self.range_max, 0.0)
            gid, nrm = None, None
        return o, dw, dist, gid, nrm

    def gl_defs(self) -> List[dict]:
        return [{"name": self.name, "pos": [self.mx, self.my, self.mz], "R": self.R, "fovy": math.degrees(self.vfov)}]

    def _rgb_gl(self, engine, noise, name=None):
        if engine is None or not getattr(self, "use_gl", False):
            return None
        img = engine.render_gl(name or self.name, self.W, self.H, pose=getattr(self, "_pose_now", None))
        if img is None:
            return None
        if noise:
            img = np.clip(img + np.random.normal(0, float(self.cfg.get("pixel_noise", 2.0)), img.shape), 0, 255).astype(np.uint8)
        return img

    def _rgb_native(self, engine, o, dw, dist, gid, nrm, noise: bool) -> Optional[np.ndarray]:
        from . import native
        lib = native.lib
        tex = getattr(engine, "_floor_tex", None)
        if not NATIVE_CAM or lib is None or not hasattr(lib, "sc_cam_shade") or tex is None:
            return None
        if getattr(self, "_rng", None) is None:
            self._rng = np.zeros(6, np.uint64)
            lib.sc_rng_seed(self._rng.ctypes.data, random.getrandbits(64))
        n = len(dist)
        o = np.ascontiguousarray(o, np.float64)
        dw = np.ascontiguousarray(dw, np.float64)
        dist = np.ascontiguousarray(dist, np.float64)
        gid = np.ascontiguousarray(gid, np.int32)
        nrm = None if nrm is None else np.ascontiguousarray(nrm, np.float64)
        rgb = np.ascontiguousarray(engine.geom_rgb, np.float32)
        tex = np.ascontiguousarray(tex, np.float32)
        x0, y0, res = engine._floor_org
        out = np.empty((self.H, self.W, 3), np.uint8)
        sigma = float(self.cfg.get("pixel_noise", 2.0)) if noise else 0.0
        lib.sc_cam_shade(o.ctypes.data, dw.ctypes.data, dist.ctypes.data, gid.ctypes.data, None if nrm is None else nrm.ctypes.data, n,
                         rgb.ctypes.data, len(rgb), int(engine.floor_geom), tex.ctypes.data, tex.shape[0], tex.shape[1],
                         float(x0), float(y0), float(res), sigma, self._rng.ctypes.data, out.ctypes.data)
        return out

    def _rgb(self, engine, o, dw, dist, gid, nrm, noise: bool) -> np.ndarray:
        if engine is not None and gid is not None:
            img = self._rgb_native(engine, o, dw, dist, gid, nrm, noise)
            if img is not None:
                return img
        if engine is not None:
            col = engine.shade(o, dw, dist, gid, nrm)
        else:
            z = np.where(np.isfinite(dist), dist, self.range_max)
            g = np.clip(1.0 - z / self.range_max, 0.1, 1.0)
            col = np.stack([g, g, g], 1)
        img = (col.reshape(self.H, self.W, 3) * 255.0)
        if noise:
            img = img + np.random.normal(0, float(self.cfg.get("pixel_noise", 2.0)), img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)

    def capture(self, world, engine, x, y, th, noise=True) -> Dict[str, np.ndarray]:
        self.frame += 1
        self._pose_now = (x, y, th)
        img = self._rgb_gl(engine, noise)
        if img is not None:
            return {"rgb": img}
        o, dw, dist, gid, nrm = self._cast(world, engine, x, y, th)
        return {"rgb": self._rgb(engine, o, dw, dist, gid, nrm, noise)}


class StereoCamera(CameraSensor):
    """左目为参考 (<name>_optical_frame)，右目沿 -y 偏移基线"""
    kind = "stereo"

    def __init__(self, cfg: dict):
        super().__init__(cfg)
        self.baseline = float(self.cfg.get("baseline_m", 0.05))
        self.range_min = float(self.cfg.get("range_min", 0.3))

    def streams(self):
        return ["left", "right", "depth"]

    def info(self):
        d = super().info()
        d["baseline_m"] = self.baseline
        d["depth_sigma_at_2m"] = round(4.0 * float(self.cfg.get("subpixel_px", 0.08)) / (self.fx * self.baseline), 4)
        return d

    def gl_defs(self):
        R = self.R
        off = R @ np.array([0.0, -self.baseline, 0.0])
        return [{"name": self.name, "pos": [self.mx, self.my, self.mz], "R": R, "fovy": math.degrees(self.vfov)},
                {"name": self.name + "_right", "pos": [self.mx + off[0], self.my + off[1], self.mz + off[2]], "R": R,
                 "fovy": math.degrees(self.vfov)}]

    def capture(self, world, engine, x, y, th, noise=True):
        self._pose_now = (x, y, th)
        gl = self._rgb_gl(engine, noise)
        need_normals = gl is None
        o, dw, dist, gid, nrm = self._cast(world, engine, x, y, th, normals=need_normals)
        o2, dw2, dist2, gid2, nrm2 = self._cast(world, engine, x, y, th, normals=need_normals, offset_y=-self.baseline)
        if gl is not None:
            left, right = gl, self._rgb_gl(engine, noise, self.name + "_right")
        else:
            left = self._rgb(engine, o, dw, dist, gid, nrm, noise)
            right = self._rgb(engine, o2, dw2, dist2, gid2, nrm2, noise)
        z = (dist * self.axis_cos).reshape(self.H, self.W).astype(np.float32)
        if noise:
            sig = z * z * float(self.cfg.get("subpixel_px", 0.08)) / (self.fx * self.baseline)
            z = z + np.random.normal(0, 1, z.shape).astype(np.float32) * np.nan_to_num(sig, posinf=0.0).astype(np.float32)
        z[(~np.isfinite(z)) | (z < self.range_min) | (z > self.range_max)] = np.nan
        # 双目遮挡: 右目看不到的像素 (左右距离差异大) 无有效视差
        with np.errstate(invalid="ignore"):
            occl = np.abs(np.nan_to_num(dist2 - dist, nan=0.0, posinf=1e9, neginf=1e9)).reshape(self.H, self.W) > 0.25 * np.nan_to_num(z, nan=1e9)
        z[occl] = np.nan
        self.frame += 1
        return {"left": left, "right": right, "depth": z}


class TofCamera(CameraSensor):
    kind = "tof"

    def __init__(self, cfg: dict):
        super().__init__(cfg)
        self.range_min = float(self.cfg.get("range_min", 0.1))

    def streams(self):
        return ["depth", "amplitude", "points"]

    def capture(self, world, engine, x, y, th, noise=True):
        o, dw, dist, gid, nrm = self._cast(world, engine, x, y, th, normals=True)
        z = (dist * self.axis_cos).astype(np.float32)
        cos_inc = np.abs((nrm * dw).sum(1)) if nrm is not None else np.ones_like(z)
        amp = np.where(np.isfinite(z), 2000.0 * cos_inc / np.maximum(z, 0.05) ** 2, 0.0)
        if noise:
            sig = float(self.cfg.get("noise_abs_m", 0.005)) + float(self.cfg.get("noise_rel", 0.004)) * np.nan_to_num(z, posinf=0)
            z = z + np.random.normal(0, 1, z.shape).astype(np.float32) * sig.astype(np.float32)
            amp = amp * (1 + np.random.normal(0, 0.03, amp.shape))
        valid = np.isfinite(z) & (z >= self.range_min) & (z <= self.range_max) & (amp > 20.0)
        z[~valid] = np.nan
        Z = z.reshape(self.H, self.W)
        u, v = np.meshgrid(np.arange(self.W), np.arange(self.H))
        X = (u - self.cx) / self.fx * Z
        Y = (v - self.cy) / self.fy * Z
        pts = np.stack([X, Y, Z], -1).reshape(-1, 3)
        pts = pts[np.isfinite(pts[:, 2])].astype(np.float32)
        self.frame += 1
        return {"depth": Z, "amplitude": np.clip(amp, 0, 65535).astype(np.uint16).reshape(self.H, self.W), "points": pts}


def build_camera(cfg: dict) -> Optional[CameraSensor]:
    t = cfg.get("type")
    if t == "depthCamera":
        t = "tof"
    cls = {"camera": CameraSensor, "stereo": StereoCamera, "tof": TofCamera}.get(t)
    if cls is None:
        return None
    c = dict(cfg)
    c["type"] = t
    return cls(c)
