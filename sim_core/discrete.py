#!/usr/bin/env python3
"""
开关量传感器仿真: 光电 (漫反射/对射) + 防撞触边 (碰撞条)

光电传感器 PhotoSensor
  * 安装位姿来自 cmodel (sensor/photoelectric|PT|proximitySensor)；缺省时按车身轮廓生成 4 个角部避障光电
  * 每个光电 = 窄锥 3 条射线 (发散角 ±beam_half_deg)，检测距离 ≤ trigger_m 时输出 ON (带 2 cm 回差防抖)
  * 只看得见高于安装高度的障碍 (低矮物体盲区与激光一致)
  * 输出: detected / distance_m / 对应 DI 名 (di_pe_<name>)

防撞触边 BumperStrip
  * 沿车身轮廓前/后(可选左/右)边外扩一条 thickness 厚的检测带，与世界线段做多边形相交检测
  * 触边被压下 → DI (di_bumper_front / di_bumper_rear ...)；同时引擎按方向禁止继续朝该侧运动
  * 释放后保持 hold_s 再复位 (模拟机械回弹 + 安全继电器延时)
"""

import math
from typing import Dict, List, Optional

import numpy as np

from .world import World


class PhotoSensor:
    def __init__(self, cfg: dict):
        self.cfg = dict(cfg)
        self.name = cfg["name"]
        self.di = cfg.get("di") or f"di_pe_{self.name}"
        self.x, self.y, self.z = float(cfg.get("x", 0.0)), float(cfg.get("y", 0.0)), float(cfg.get("z", 0.15))
        self.yaw = float(cfg.get("yaw", 0.0))
        self.range_m = float(cfg.get("range_m", 1.0))            # 最大可测
        self.trigger_m = float(cfg.get("trigger_m", 0.4))        # 触发阈值 (可调节距离)
        self.hyst = float(cfg.get("hysteresis_m", 0.02))
        self.half = math.radians(float(cfg.get("beam_half_deg", 2.0)))
        self.kind = cfg.get("kind", "diffuse")
        self.detected = False
        self.distance = math.inf

    def update(self, world: World, x: float, y: float, th: float) -> bool:
        c, s = math.cos(th), math.sin(th)
        ox, oy = x + c * self.x - s * self.y, y + s * self.x + c * self.y
        a = th + self.yaw
        d = world.raycast(ox, oy, np.array([a - self.half, a, a + self.half]), self.range_m, min_seg_height=self.z, few=True)
        self.distance = float(np.min(d))
        thr = self.trigger_m + (self.hyst if self.detected else 0.0)
        self.detected = self.distance <= thr
        return self.detected

    def view(self) -> dict:
        return {"name": self.name, "di": self.di, "kind": self.kind, "detected": self.detected,
                "distance_m": None if not math.isfinite(self.distance) else round(self.distance, 3),
                "trigger_m": self.trigger_m, "range_m": self.range_m,
                "mount": {"x": self.x, "y": self.y, "z": self.z, "yaw": round(self.yaw, 4)}, "source": self.cfg.get("source", "")}


class BumperStrip:
    def __init__(self, name: str, side: str, poly_body: List[List[float]], di: str, hold_s: float = 0.5):
        self.name, self.side, self.poly, self.di, self.hold_s = name, side, poly_body, di, hold_s
        self._poly_np = np.ascontiguousarray(poly_body, dtype=np.float64)
        self.pressed = False
        self.contact_t = -1e9
        self.press_count = 0

    def update(self, world: World, x: float, y: float, th: float, t: float, forced: bool = False) -> bool:
        hit, _ = world.collides(self._poly_np, x, y, th)
        if hit or forced:
            if not self.pressed:
                self.press_count += 1
            self.pressed = True
            self.contact_t = t
        elif self.pressed and t - self.contact_t > self.hold_s:
            self.pressed = False
        return self.pressed

    def view(self) -> dict:
        return {"name": self.name, "side": self.side, "di": self.di, "pressed": self.pressed,
                "press_count": self.press_count, "polygon": self.poly}


def _bbox(fp):
    P = np.asarray(fp, dtype=float)
    return P[:, 0].max(), P[:, 0].min(), P[:, 1].max(), P[:, 1].min()


def build_photos(spec: dict) -> List[PhotoSensor]:
    ch = spec["chassis"]
    xmax, xmin, ymax, ymin = _bbox(ch["footprint"])
    cfgs = []
    for d in spec.get("photoelectric", []) or []:
        cfgs.append(dict(d, source=d.get("source", "cmodel")))
    if not cfgs:
        for io in (spec.get("io", {}) or {}).get("inputs", []):
            if io.get("type") in ("photoelectric", "PT", "proximitySensor") and "x" in io:
                cfgs.append({"name": io["name"], "x": io["x"], "y": io["y"], "z": io.get("z", 0.15), "yaw": io.get("yaw", 0.0),
                             "trigger_m": 0.3 if io.get("type") == "proximitySensor" else 0.5, "source": "cmodel:" + io["type"]})
    if not cfgs:   # 缺省: 四角 45° 斜向避障光电 (补激光近场盲区)
        z = 0.12
        for nm, px, py, yaw in (("front_left", xmax, ymax, math.radians(30)), ("front_right", xmax, ymin, -math.radians(30)),
                                ("rear_left", xmin, ymax, math.radians(150)), ("rear_right", xmin, ymin, -math.radians(150))):
            cfgs.append({"name": nm, "x": px, "y": py, "z": z, "yaw": yaw, "trigger_m": 0.35, "range_m": 1.0,
                         "source": "default: 角部避障光电"})
    return [PhotoSensor(c) for c in cfgs]


def build_bumpers(spec: dict, thickness: float = 0.03) -> List[BumperStrip]:
    ch = spec["chassis"]
    xmax, xmin, ymax, ymin = _bbox(ch["footprint"])
    inset = 0.02   # 触边两端略缩进 (圆角)
    strips = [
        BumperStrip("front", "front", [[xmax, ymax - inset], [xmax + thickness, ymax - inset], [xmax + thickness, ymin + inset], [xmax, ymin + inset]],
                    "di_bumper_front"),
        BumperStrip("rear", "rear", [[xmin - thickness, ymax - inset], [xmin, ymax - inset], [xmin, ymin + inset], [xmin - thickness, ymin + inset]],
                    "di_bumper_rear"),
    ]
    if (spec.get("bumpers") or {}).get("sides"):
        strips += [
            BumperStrip("left", "left", [[xmin + inset, ymax], [xmax - inset, ymax], [xmax - inset, ymax + thickness], [xmin + inset, ymax + thickness]],
                        "di_bumper_left"),
            BumperStrip("right", "right", [[xmin + inset, ymin - thickness], [xmax - inset, ymin - thickness], [xmax - inset, ymin], [xmin + inset, ymin]],
                        "di_bumper_right"),
        ]
    return strips


def motion_block(strips: List[BumperStrip], vx: float, vy: float, wz: float):
    """触边压下时禁止继续朝该侧运动 (允许后退脱困)"""
    for b in strips:
        if not b.pressed:
            continue
        if b.side == "front" and vx > 0:
            vx = 0.0
        elif b.side == "rear" and vx < 0:
            vx = 0.0
        elif b.side == "left" and vy > 0:
            vy = 0.0
        elif b.side == "right" and vy < 0:
            vy = 0.0
    if any(b.pressed for b in strips):
        wz = 0.0   # 接触状态下禁止原地旋转 (车角扫刮)
    return vx, vy, wz
