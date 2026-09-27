#!/usr/bin/env python3
"""
通用轮式底盘运动学 + 执行器动力学 (差速 / 单舵轮 / 双舵轮 / 多舵轮 统一模型)

建模要点
---------
* base_link = cmodel 运动中心；所有轮子坐标 (x, y) 相对运动中心。
* 轮子类型:
    drive  —— 固定朝向的驱动轮 (差速轮)            : 纵向驱动 + 横向无滑
    steer  —— 舵轮 (转向 + 行走)                    : 纵向驱动 + 横向无滑，朝向可变
    fixed  —— 固定朝向从动轮 (单舵轮后桥承重轮)     : 仅横向无滑约束
    caster —— 万向脚轮                              : 无约束
* 逆运动学: 每个轮子接触点速度 v_i = (vx - wz*y_i, vy + wz*x_i)
    - 舵轮在 {θ, θ±π(反转)} 中选择"限位内且离当前角最近"的解
    - 轮端超速 → 整体等比例缩放底盘指令 (保持曲率不变)
    - 转向未到位 → 行走速度按角度误差衰减 (真实舵轮控制器的"先转后走")
* 执行器: 转向 = 限位 + 限速 (转向电机额定转速/减速比)；行走 = 一阶惯性 + 加速度限制
* 正运动学: 所有轮子的纵向速度方程 + 横向无滑约束 → 加权最小二乘求 (vx, vy, wz)。
  舵轮未对齐时的"轮间打架"体现为最小二乘残差 (slip_residual)，而不是凭空的速度。
* 位姿积分: SE(2) 指数映射 (精确圆弧)，与步长无关。
"""

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def se2_integrate(x: float, y: float, th: float, vx: float, vy: float, wz: float, dt: float) -> Tuple[float, float, float]:
    """机体系速度 (vx, vy, wz) 在 dt 内的精确积分"""
    dth = wz * dt
    if abs(dth) < 1e-9:
        dxb, dyb = vx * dt, vy * dt
    else:
        s, c = math.sin(dth), math.cos(dth)
        a = s / wz
        b = (1.0 - c) / wz
        dxb = a * vx - b * vy
        dyb = b * vx + a * vy
    ct, st = math.cos(th), math.sin(th)
    return x + ct * dxb - st * dyb, y + st * dxb + ct * dyb, wrap(th + dth)


@dataclass
class Wheel:
    name: str
    kind: str                     # drive | steer | fixed | caster
    x: float
    y: float
    radius: float
    steer_min: float = -math.pi / 2
    steer_max: float = math.pi / 2
    steer_rate: float = math.radians(90.0)
    max_speed: float = 2.0        # 轮缘线速度上限 m/s
    drive_tau: float = 0.06       # 行走电机速度环时间常数 s
    gear_ratio: float = 20.0
    # --- 状态 ---
    steer: float = 0.0
    steer_target: float = 0.0
    speed: float = 0.0            # 轮缘线速度 (沿轮子朝向) m/s
    speed_target: float = 0.0
    cmd_speed: float = 0.0        # 行走电机速度环输出 (驱动器指令)
    angle: float = 0.0            # 车轮转角累积 rad
    # 电气量
    motor_rpm: float = 0.0
    current_a: float = 0.0
    torque_nm: float = 0.0
    steer_current_a: float = 0.0

    @property
    def heading(self) -> float:
        return self.steer if self.kind == "steer" else 0.0

    @property
    def driven(self) -> bool:
        return self.kind in ("drive", "steer")

    @property
    def constrained(self) -> bool:
        return self.kind in ("drive", "steer", "fixed")

    @property
    def joint_names(self) -> List[str]:
        if self.kind == "steer":
            return [f"{self.name}_steer_joint", f"{self.name}_drive_joint"]
        if self.kind == "caster":
            return []
        return [f"{self.name}_joint"]


@dataclass
class ChassisLimits:
    max_speed: float = 1.2
    max_accel: float = 0.5
    max_decel: float = 0.5
    max_ang_speed: float = 1.0
    max_ang_accel: float = 1.0
    max_ang_decel: float = 1.0
    mass: float = 200.0


class ChassisKinematics:
    """一个统一的轮式底盘模型"""

    STEER_GATE_START = math.radians(10.0)   # 转向误差超过 10° 开始限制行走
    STEER_GATE_STOP = math.radians(45.0)    # 超过 45° 行走速度降为 0

    def __init__(self, chassis_type: str, wheels: List[Wheel], limits: ChassisLimits):
        self.type = chassis_type
        self.wheels = wheels
        self.lim = limits
        fixed_dir = [w for w in wheels if w.kind in ("drive", "fixed")]
        self.holonomic = len(fixed_dir) == 0 and len([w for w in wheels if w.kind == "steer"]) >= 2
        self.axle_x = (sum(w.x for w in fixed_dir) / len(fixed_dir)) if fixed_dir else 0.0
        # 单舵轮等"一个舵轮 + 无固定轮"的特殊情况按舵轮位置约束 (退化为自行车模型)
        self.shaped = [0.0, 0.0, 0.0]   # 整形后的底盘指令 (vx, vy, wz)
        self.vx = self.vy = self.wz = 0.0
        self.slip_residual = 0.0
        self.saturation = 1.0
        # 电气参数 (通用 48V 伺服系统近似)
        self.kt = 0.12                   # 电机转矩常数 Nm/A
        self.rolling_coeff = 0.012       # 滚动阻力系数
        self.efficiency = 0.85

    # ------------------------------------------------------------------
    # 指令整形: 限速 + 加减速限制 (模拟车载运动控制器)
    # ------------------------------------------------------------------
    def _project(self, vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
        if not self.holonomic:
            vy = -wz * self.axle_x   # 非完整约束: 固定轮轴线横向速度为 0
        return vx, vy, wz

    def shape_command(self, cvx: float, cvy: float, cwz: float, dt: float) -> Tuple[float, float, float]:
        L = self.lim
        cvx, cvy, cwz = self._project(cvx, cvy, cwz)
        v = math.hypot(cvx, cvy)
        if v > L.max_speed:
            k = L.max_speed / v
            cvx, cvy = cvx * k, cvy * k
        cwz = max(-L.max_ang_speed, min(L.max_ang_speed, cwz))

        def ramp(cur, tgt, acc, dec):
            if abs(tgt) > abs(cur) and (tgt * cur >= 0):
                lim = acc * dt
            else:
                lim = dec * dt
            d = tgt - cur
            return tgt if abs(d) <= lim else cur + math.copysign(lim, d)

        sx, sy, sw = self.shaped
        sx = ramp(sx, cvx, L.max_accel, L.max_decel)
        sy = ramp(sy, cvy, L.max_accel, L.max_decel)
        sw = ramp(sw, cwz, L.max_ang_accel, L.max_ang_decel)
        sx, sy, sw = self._project(sx, sy, sw)
        self.shaped = [sx, sy, sw]
        return sx, sy, sw

    # ------------------------------------------------------------------
    # 逆运动学
    # ------------------------------------------------------------------
    def _best_steer(self, w: Wheel, ang: float, spd: float) -> Tuple[float, float]:
        cands = []
        for a, s in ((ang, spd), (wrap(ang + math.pi), -spd), (wrap(ang - math.pi), -spd)):
            if w.steer_min - 1e-6 <= a <= w.steer_max + 1e-6:
                cands.append((abs(a - w.steer), a, s))
        if not cands:
            # 超出限位: 夹到最近限位 (此时会产生横向滑移残差，真实车辆同样无法实现)
            a = max(w.steer_min, min(w.steer_max, ang))
            return a, spd * math.cos(ang - a)
        cands.sort()
        return cands[0][1], cands[0][2]

    def inverse(self, vx: float, vy: float, wz: float) -> Dict[str, Tuple[float, float]]:
        out = {}
        scale = 1.0
        for w in self.wheels:
            if not w.driven:
                continue
            wx, wy = vx - wz * w.y, vy + wz * w.x
            if w.kind == "drive":
                spd, ang = wx, 0.0
            else:
                spd = math.hypot(wx, wy)
                if spd < 1e-4:
                    ang = w.steer_target  # 静止时保持舵角，不回零
                    spd = 0.0
                else:
                    ang, spd = self._best_steer(w, math.atan2(wy, wx), spd)
            if abs(spd) > w.max_speed:
                scale = min(scale, w.max_speed / abs(spd))
            out[w.name] = (ang, spd)
        if scale < 1.0:
            out = {k: (a, s * scale) for k, (a, s) in out.items()}
        self.saturation = scale
        return out

    # ------------------------------------------------------------------
    # 正运动学 (加权最小二乘)
    # ------------------------------------------------------------------
    def forward(self, steer_override: Optional[Dict[str, float]] = None,
                speed_override: Optional[Dict[str, float]] = None) -> Tuple[float, float, float, float]:
        rows, rhs, wts = [], [], []
        for w in self.wheels:
            if not w.constrained:
                continue
            a = (steer_override or {}).get(w.name, w.heading)
            c, s = math.cos(a), math.sin(a)
            if w.driven:
                spd = (speed_override or {}).get(w.name, w.speed)
                rows.append([c, s, -w.y * c + w.x * s]); rhs.append(spd); wts.append(1.0)
            rows.append([-s, c, w.y * s + w.x * c]); rhs.append(0.0); wts.append(1.0)
        if not rows:
            return 0.0, 0.0, 0.0, 0.0
        A = np.asarray(rows) * np.asarray(wts)[:, None]
        b = np.asarray(rhs) * np.asarray(wts)
        sol, res, rank, _ = np.linalg.lstsq(A, b, rcond=None)
        resid = float(np.sqrt(np.mean((A @ sol - b) ** 2)))
        vx, vy, wz = (float(v) for v in sol)
        if not self.holonomic:
            vy = -wz * self.axle_x if rank >= 3 else vy
        return vx, vy, wz, resid

    # ------------------------------------------------------------------
    # 单步: 指令 → 执行器 → 车体速度
    # ------------------------------------------------------------------
    def step(self, cvx: float, cvy: float, cwz: float, dt: float, brake: bool = False) -> Tuple[float, float, float]:
        if brake:
            cvx = cvy = cwz = 0.0
        vx_c, vy_c, wz_c = self.shape_command(cvx, cvy, cwz, dt)
        targets = self.inverse(vx_c, vy_c, wz_c)
        pure_rot = math.hypot(vx_c, vy_c) < 0.02 and abs(wz_c) > 1e-3

        for w in self.wheels:
            if w.name in targets:
                ang_t, spd_t = targets[w.name]
                if w.kind == "steer":
                    w.steer_target = ang_t
                    err = ang_t - w.steer
                    max_d = w.steer_rate * dt
                    if abs(err) > max_d:
                        w.steer += math.copysign(max_d, err)
                        w.steer_current_a = 4.5
                    else:
                        w.steer = ang_t
                        w.steer_current_a = 0.8
                    w.steer = max(w.steer_min, min(w.steer_max, w.steer))
                    # 先转后走
                    e = abs(w.steer_target - w.steer)
                    # 原地旋转 (线速度≈0) 时舵角必须基本到位才行走，否则转向过程会把车体"推"出去
                    g0, g1 = (math.radians(2.0), math.radians(8.0)) if pure_rot else (self.STEER_GATE_START, self.STEER_GATE_STOP)
                    if e > g0:
                        g = max(0.0, 1.0 - (e - g0) / (g1 - g0))
                        spd_t *= g
                w.speed_target = 0.0 if brake else spd_t
                # 一阶惯性 + 轮端加速度上限
                alpha = dt / (w.drive_tau + dt)
                dv = (w.speed_target - w.cmd_speed) * alpha
                acc_lim = max(self.lim.max_accel, self.lim.max_decel) * 2.5 * dt
                dv = max(-acc_lim, min(acc_lim, dv))
                w.cmd_speed += dv
                if brake and abs(w.cmd_speed) < 0.01:
                    w.cmd_speed = 0.0
                w.speed = w.cmd_speed   # 运动学层: 实际 = 指令 (一阶惯性)；MuJoCo 后端再叠加接触/碰撞

        vx, vy, wz, resid = self.forward()
        self.slip_residual = resid
        # 被动轮 & 电气量
        acc = (vx - self.vx) / dt if dt > 0 else 0.0
        self.vx, self.vy, self.wz = vx, vy, wz
        n_drv = max(1, len([w for w in self.wheels if w.driven]))
        for w in self.wheels:
            if w.kind in ("fixed", "caster"):
                w.speed = (vx - wz * w.y) * math.cos(w.heading) + (vy + wz * w.x) * math.sin(w.heading)
            w.angle += (w.speed / max(w.radius, 1e-3)) * dt
            if w.driven:
                omega = w.speed / max(w.radius, 1e-3)
                w.motor_rpm = omega * w.gear_ratio * 60.0 / (2 * math.pi)
                f = self.lim.mass / n_drv * (abs(acc) + 9.81 * self.rolling_coeff * (1 if abs(w.speed) > 1e-3 else 0))
                w.torque_nm = f * w.radius / max(w.gear_ratio, 1.0) / self.efficiency
                w.current_a = (0.3 + w.torque_nm / self.kt) if abs(w.speed_target) > 1e-3 or abs(w.speed) > 1e-3 else 0.05
        return vx, vy, wz

    def reset(self):
        self.shaped = [0.0, 0.0, 0.0]
        self.vx = self.vy = self.wz = 0.0
        for w in self.wheels:
            w.speed = w.speed_target = w.cmd_speed = 0.0

    def stop_now(self):
        """碰撞/急停: 执行器速度立即清零 (机械抱闸)"""
        self.reset()

    # ------------------------------------------------------------------
    def joint_state(self) -> Tuple[List[str], List[float], List[float], List[float]]:
        names, pos, vel, eff = [], [], [], []
        for w in self.wheels:
            if w.kind == "steer":
                names += [f"{w.name}_steer_joint", f"{w.name}_drive_joint"]
                pos += [w.steer, w.angle]
                vel += [0.0, w.speed / max(w.radius, 1e-3)]
                eff += [0.0, w.torque_nm]
            elif w.kind in ("drive", "fixed"):
                names.append(f"{w.name}_joint")
                pos.append(w.angle)
                vel.append(w.speed / max(w.radius, 1e-3))
                eff.append(w.torque_nm)
        return names, pos, vel, eff

    def telemetry(self) -> dict:
        out = {"type": self.type, "holonomic": self.holonomic, "saturation": round(self.saturation, 3),
               "slip_residual": round(self.slip_residual, 4), "wheels": {}}
        for w in self.wheels:
            if w.kind == "caster":
                continue
            d = {"kind": w.kind, "speed_mps": round(w.speed, 3), "rpm": round(w.motor_rpm, 1),
                 "current_a": round(w.current_a, 2), "torque_nm": round(w.torque_nm, 2), "rad": round(w.angle, 2)}
            if w.kind == "steer":
                d.update({"steer_angle_deg": round(math.degrees(w.steer), 1),
                          "target_steer_deg": round(math.degrees(w.steer_target), 1),
                          "steer_current_a": round(w.steer_current_a, 2)})
            out["wheels"][w.name] = d
        return out


# ----------------------------------------------------------------------
# 由 robot_config 构建 / 预设车型
# ----------------------------------------------------------------------
def _limits_from_spec(spec: dict) -> ChassisLimits:
    ch = spec.get("chassis", {})
    return ChassisLimits(
        max_speed=float(ch.get("max_speed_mps", 1.2)),
        max_accel=float(ch.get("max_accel_mps2", 0.5)),
        max_decel=float(ch.get("max_decel_mps2", ch.get("max_accel_mps2", 0.5))),
        max_ang_speed=float(ch.get("max_ang_speed_radps", 1.0)),
        max_ang_accel=float(ch.get("max_ang_accel_radps2", 1.0)),
        max_ang_decel=float(ch.get("max_ang_decel_radps2", ch.get("max_ang_accel_radps2", 1.0))),
        mass=float(ch.get("mass_kg", 200.0)),
    )


def _wheel_from_dict(d: dict) -> Wheel:
    gear = 20.0
    dm = d.get("drive_motor") or {}
    if dm.get("gear_ratio"):
        gear = float(dm["gear_ratio"])
    return Wheel(
        name=d["name"], kind=d["kind"], x=float(d["x"]), y=float(d["y"]), radius=float(d.get("radius_m", 0.1)),
        steer_min=float(d.get("steer_min_rad", -math.pi / 2)), steer_max=float(d.get("steer_max_rad", math.pi / 2)),
        steer_rate=float(d.get("steer_rate_radps", math.radians(90))),
        max_speed=float(d.get("max_speed_mps", 3.0)), gear_ratio=gear,
    )


def preset_wheels(spec: dict, chassis_type: str) -> List[dict]:
    """以 cmodel 车体尺寸为基础，构造其它车型的典型轮组布局 (用于车型对比仿真)"""
    ch = spec["chassis"]
    head, tail = ch["head_offset_m"], ch["tail_offset_m"]
    left, right = ch["left_offset_m"], ch["right_offset_m"]
    r = float(spec.get("drive_wheels", {}).get("radius_m", 0.1))
    v_cap = float(ch.get("max_speed_mps", 1.5)) * 1.05
    rate = math.radians(90.0)
    xc = (head - tail) / 2.0
    L = head + tail
    m = 0.12
    if chassis_type == "diff_drive":
        cr = r * 0.6
        ws = [{"name": "left_wheel", "kind": "drive", "x": 0.0, "y": left - m - 0.05, "radius_m": r, "max_speed_mps": v_cap},
              {"name": "right_wheel", "kind": "drive", "x": 0.0, "y": -(right - m - 0.05), "radius_m": r, "max_speed_mps": v_cap}]
        for tag, x in (("front", head - m - cr), ("rear", -(tail - m - cr))):
            for side, y in (("left", left - m - cr), ("right", -(right - m - cr))):
                if abs(x) > 0.2:
                    ws.append({"name": f"caster_{tag}_{side}", "kind": "caster", "x": x, "y": y, "radius_m": cr})
        return ws
    if chassis_type == "single_steer":
        sx = max(0.4, head - 0.2)
        return [{"name": "steer_wheel", "kind": "steer", "x": sx, "y": 0.0, "radius_m": r,
                 "steer_min_rad": -math.radians(110), "steer_max_rad": math.radians(110), "steer_rate_radps": rate, "max_speed_mps": v_cap},
                {"name": "rear_left_load_wheel", "kind": "fixed", "x": 0.0, "y": left - m, "radius_m": r * 0.75},
                {"name": "rear_right_load_wheel", "kind": "fixed", "x": 0.0, "y": -(right - m), "radius_m": r * 0.75}]
    if chassis_type == "dual_steer":
        d = L * 0.36
        ws = [{"name": "front_steer_wheel", "kind": "steer", "x": xc + d, "y": 0.0, "radius_m": r,
               "steer_min_rad": -math.radians(175), "steer_max_rad": math.radians(175), "steer_rate_radps": rate, "max_speed_mps": v_cap},
              {"name": "rear_steer_wheel", "kind": "steer", "x": xc - d, "y": 0.0, "radius_m": r,
               "steer_min_rad": -math.radians(175), "steer_max_rad": math.radians(175), "steer_rate_radps": rate, "max_speed_mps": v_cap}]
        cr = r * 0.7
        for i, (x, y) in enumerate([(xc, left - m - cr), (xc, -(right - m - cr))]):
            ws.append({"name": f"caster_{i}", "kind": "caster", "x": x, "y": y, "radius_m": cr})
        return ws
    raise ValueError(chassis_type)


def build_kinematics(spec: dict, chassis_type: Optional[str] = None) -> Tuple[ChassisKinematics, List[dict]]:
    """chassis_type=None 或等于 cmodel 车型 → 使用 cmodel 真实轮组；否则使用同车体的预设轮组"""
    native = spec.get("chassis", {}).get("type", "diff_drive")
    wheels_d = spec.get("wheels")
    if chassis_type is None or chassis_type == native or (chassis_type == "dual_steer" and native == "multi_steer"):
        chassis_type = native
        if not wheels_d:
            wheels_d = preset_wheels(spec, native if native in ("diff_drive", "single_steer", "dual_steer") else "dual_steer")
    else:
        wheels_d = preset_wheels(spec, chassis_type)
    wheels = [_wheel_from_dict(w) for w in wheels_d]
    return ChassisKinematics(chassis_type, wheels, _limits_from_spec(spec)), wheels_d
