#!/usr/bin/env python3
"""
车辆保护空间 (统一配置)

保存在车辆模型的人工补全里 (model_overrides.json → "protection")，随模型版本管理、部署时随模型下发；
执行进程 (激光防护区/转向防护/过弯净空/进站)、规划器 (拐点可行性)、Nav2 参数生成、界面可视化都从这里取值。

    body_margin        车体净空 (过弯/原地转向/规划的统一基准)，m
    payload            带载外形 (顶升到位后生效)：相对运动中心的 head/tail/left/right，m；enabled=false 时按车体
    fields[]           行驶防护区，按速度分档 (v_max 升序，取第一个 v_max ≥ |v| 的档)
                         front/rear: 车头/车尾外的停车距离；side: 两侧外扩
    slow_ratio         减速区 = 停车区 × slow_ratio；进入减速区限速 slow_speed
    rotate_margin      原地转向防护：车体外扩该值后，前方 rotate_lookahead_rad 弧度的扫掠区内有点即停止转向
    docking            末段进站: front=车头剩余行程外允许的最小余量；arrive_tolerance=前方受限时视为到位的距离
    reaction_s         制动校核用的系统反应时间 (传感器+控制+制动建立)
"""

import copy
import math
from typing import Dict, List, Optional, Tuple

DEFAULTS = {
    "body_margin": 0.05,
    "payload": {"enabled": False, "head": None, "tail": None, "left": None, "right": None, "note": ""},
    "fields": [
        {"name": "低速", "v_max": 0.3, "front": 0.30, "rear": 0.20, "side": 0.08},
        {"name": "中速", "v_max": 0.8, "front": 0.80, "rear": 0.30, "side": 0.10},
        {"name": "高速", "v_max": 9.0, "front": 1.40, "rear": 0.40, "side": 0.15},
    ],
    "slow_ratio": 2.0,
    "slow_speed": 0.35,
    "rotate_margin": 0.02,
    "rotate_lookahead_rad": 0.25,
    "docking": {"front": 0.02, "arrive_tolerance": 0.25},
    "reaction_s": 0.3,
    # 光电保护包络: 光电触发时按检测点位置决定是否响应 (检测点在包络内才停车/禁止朝该侧运动)
    #   mode = field  包络 = 当前速度档的停车区 (随速度切换，等同安全激光的区域组)
    #          custom 包络 = 当前外形外扩 front/rear/side
    #          always 任何触发都响应 (旧行为)
    #   mute_near_stop: 距前方停车点 (拐点/工位) 小于该值时屏蔽前向光电 (muting，车辆本就要在此停下)
    "photo": {"mode": "field", "front": 0.30, "rear": 0.30, "side": 0.10, "mute_near_stop": 0.10},
    "corner_mode": "auto",         # auto: 能原地转向就在拐点停车转向 (严格沿线路)，转不开的拐点用圆弧过渡；rotate: 只允许原地转向；arc: 优先圆弧
    "enabled": True,
}

FIELD_KEYS = ("front", "rear", "side")


def _merge(base: dict, patch: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (patch or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def effective(spec_or_prot: Optional[dict], ch: Optional[dict] = None) -> dict:
    """spec (含 protection) 或 protection 片段 → 补齐默认值、排序、校正后的完整配置"""
    raw = spec_or_prot or {}
    if "chassis" in raw:
        ch = ch or raw["chassis"]
        raw = raw.get("protection") or {}
    P = _merge(DEFAULTS, raw)
    if raw.get("fields"):
        P["fields"] = copy.deepcopy(raw["fields"])
    elif ch:
        P["fields"] = default_fields(ch, P["reaction_s"])
        P["fields_auto"] = True
    fs = []
    for i, f in enumerate(P["fields"] or DEFAULTS["fields"]):
        f = dict(f)
        f.setdefault("name", f"档{i + 1}")
        f["v_max"] = float(f.get("v_max", 9.0))
        for k in FIELD_KEYS:
            f[k] = max(0.0, float(f.get(k, 0.0) or 0.0))
        fs.append(f)
    fs.sort(key=lambda f: f["v_max"])
    if fs:
        fs[-1]["v_max"] = max(fs[-1]["v_max"], 9.0)        # 最后一档兜底覆盖所有速度
    P["fields"] = fs
    for k in ("body_margin", "slow_ratio", "slow_speed", "rotate_margin", "rotate_lookahead_rad", "reaction_s"):
        P[k] = max(0.0, float(P.get(k) or 0.0))
    P["slow_ratio"] = max(1.0, P["slow_ratio"])
    if ch:
        pl = P["payload"]
        for k, ck in (("head", "head_offset_m"), ("tail", "tail_offset_m"), ("left", "left_offset_m"), ("right", "right_offset_m")):
            if pl.get(k) is None:
                pl[k] = float(ch.get(ck, 0.5))
    return P


NAV_MAX_SPEED = 1.2          # 执行进程导引的速度上限 (与 navigator max_v 一致)


def default_fields(ch: dict, reaction_s: float = 0.3) -> List[dict]:
    """未配置防护区时按车型动力学生成: 停车距离 = 反应距离 + 制动距离 + 0.1 m (向上取整到 5 cm)"""
    dec = float(ch.get("max_decel_mps2", ch.get("max_accel_mps2", 0.5)))
    vtop = min(NAV_MAX_SPEED, float(ch.get("max_speed_mps", 1.5)))
    up = lambda d: math.ceil((d + 0.1) * 20.0) / 20.0
    out = []
    for name, v, side in (("低速", min(0.3, vtop), 0.08), ("中速", min(0.8, vtop), 0.10), ("高速", vtop, 0.15)):
        if out and v <= out[-1]["v_max"] + 1e-6:
            continue
        out.append({"name": name, "v_max": round(v, 2), "front": up(braking_distance(v, dec, reaction_s)),
                    "rear": up(braking_distance(min(v, 0.5), dec, reaction_s)), "side": side})
    return out


def photo_envelope(ch: dict, P: dict, loaded: bool, field_idx: int = 0) -> Tuple[float, float, float, float]:
    """光电保护包络 (机体系矩形 x∈[-tail', head']，y∈[-right', left'])；mode=always 返回 None"""
    ph = P.get("photo") or {}
    mode = ph.get("mode", "field")
    if mode == "always":
        return None
    h, t, l, r = outline(ch, P, loaded)
    if mode == "custom":
        fr, rr, sd = float(ph.get("front", 0.3)), float(ph.get("rear", 0.3)), float(ph.get("side", 0.1))
    else:
        f = P["fields"][max(0, min(field_idx, len(P["fields"]) - 1))]
        fr, rr, sd = f["front"], f["rear"], f["side"]
    return (h + fr, t + rr, l + sd, r + sd)


def body_box(ch: dict) -> Tuple[float, float, float, float]:
    return (float(ch.get("head_offset_m", 0.6)), float(ch.get("tail_offset_m", 0.6)),
            float(ch.get("left_offset_m", 0.4)), float(ch.get("right_offset_m", 0.4)))


def outline(ch: dict, P: dict, loaded: bool) -> Tuple[float, float, float, float]:
    """当前外形 (head, tail, left, right)：空载 = 车体；带载且启用 payload = 车体 ∪ 负载"""
    b = body_box(ch)
    pl = P.get("payload") or {}
    if not (loaded and pl.get("enabled")):
        return b
    p = tuple(float(pl.get(k) if pl.get(k) is not None else b[i]) for i, k in enumerate(("head", "tail", "left", "right")))
    return tuple(max(b[i], p[i]) for i in range(4))


def nav2_outline(ch: dict, P: dict) -> Tuple[float, float, float, float]:
    """Nav2 代价地图外形是静态的：启用负载时取车体 ∪ 负载 (保守)"""
    return outline(ch, P, True)


def field_for_speed(P: dict, v: float) -> Tuple[int, dict]:
    v = abs(v)
    for i, f in enumerate(P["fields"]):
        if v <= f["v_max"] + 1e-6:
            return i, f
    return len(P["fields"]) - 1, P["fields"][-1]


def braking_distance(v: float, decel: float, reaction_s: float) -> float:
    return v * reaction_s + v * v / (2.0 * max(decel, 0.05))


def rect(h, t, l, r) -> List[List[float]]:
    return [[h, l], [h, -r], [-t, -r], [-t, l]]


def polygons(ch: dict, P: dict, loaded: bool, field_idx: Optional[int] = None, reverse: bool = False) -> Dict[str, list]:
    """机体系多边形 (供界面绘制)：body / payload / stop / slow / rotate"""
    h, t, l, r = outline(ch, P, loaded)
    i = field_idx if field_idx is not None else 0
    f = P["fields"][max(0, min(i, len(P["fields"]) - 1))]
    fr, rr, sd = f["front"], f["rear"], f["side"]
    k = P["slow_ratio"]
    out = {"body": rect(*body_box(ch)), "outline": rect(h, t, l, r),
           "stop": rect(h + fr, t + rr, l + sd, r + sd),
           "slow": rect(h + fr * k, t + rr * k, l + sd, r + sd)}
    pl = P.get("payload") or {}
    if pl.get("enabled"):
        out["payload"] = rect(*(float(pl[k2]) for k2 in ("head", "tail", "left", "right")))
    env = photo_envelope(ch, P, loaded, i)
    if env:
        out["photo"] = rect(*env)
    m = P["rotate_margin"]
    out["rotate_radius"] = round(max(math.hypot(h, max(l, r)), math.hypot(t, max(l, r))) + m, 3)
    return out


def validate(spec: dict, P: dict) -> List[dict]:
    """配置校核 → [{key, label, value, severity(ok/info/warn/missing), note}]"""
    ch = spec.get("chassis", {})
    out = []
    vmax = min(NAV_MAX_SPEED, float(ch.get("max_speed_mps", 1.5)))
    dec = float(ch.get("max_decel_mps2", ch.get("max_accel_mps2", 0.5)))
    # 1) 制动校核: 每一档在其速度上限 (或整车最高速) 下，停车距离需 ≥ 制动距离
    prev_v = 0.0
    for i, f in enumerate(P["fields"]):
        v = min(f["v_max"], vmax)
        if v <= prev_v and i > 0:
            continue
        need = braking_distance(v, dec, P["reaction_s"])
        ok = f["front"] >= need - 1e-6
        out.append({"key": f"fields.{i}.front", "label": f"{f['name']} (≤ {v:.2f} m/s) 前向停车距离", "value": f["front"],
                    "severity": "ok" if ok else "warn",
                    "note": f"制动距离 {need:.2f} m (反应 {P['reaction_s']} s + 减速度 {dec} m/s²)" + ("" if ok else "，防护区不足以停住")})
        rv = min(v, 0.5)
        need_r = braking_distance(rv, dec, P["reaction_s"])
        if f["rear"] < need_r - 1e-6:
            out.append({"key": f"fields.{i}.rear", "label": f"{f['name']} 后向停车距离", "value": f["rear"], "severity": "info",
                        "note": f"倒车限速 0.5 m/s 时制动距离 {need_r:.2f} m"})
        prev_v = v
    # 2) 负载外形
    pl = P["payload"]
    if pl.get("enabled"):
        b = body_box(ch)
        big = [k for i, k in enumerate(("head", "tail", "left", "right")) if float(pl[k]) > b[i] + 1e-6]
        out.append({"key": "payload", "label": "带载外形", "value": [pl["head"], pl["tail"], pl["left"], pl["right"]], "severity": "ok",
                    "note": "顶升到位后生效" + (f"，超出车体: {'/'.join(big)}" if big else "，未超出车体")})
    else:
        lift = spec.get("lift") or {}
        if lift.get("enabled") or any("lift" in (m.get("name") or "").lower() for m in spec.get("other_actuators", [])):
            out.append({"key": "payload", "label": "带载外形", "value": "未配置", "severity": "warn",
                        "note": "车辆有顶升机构，货架/托盘通常比车体大，建议配置带载外形"})
    # 3) 激光覆盖: 停车区前沿/后沿是否在某个 2D/3D 激光视场内且超过最小量程
    lidars = spec.get("lidars", [])
    if lidars:
        h, t, l, r = outline(ch, P, pl.get("enabled", False))
        for side, sign in (("front", 1), ("rear", -1)):
            f = P["fields"][0]
            d = (h + f["front"]) if sign > 0 else -(t + f["rear"])
            pts = [(d, y) for y in [-(r + f["side"]) + (l + r + 2 * f["side"]) * k / 8 for k in range(9)]]
            seen = sum(1 for p in pts if _lidar_sees(lidars, p))
            sev = "ok" if seen == len(pts) else ("warn" if seen < len(pts) * 0.6 else "info")
            out.append({"key": f"coverage.{side}", "label": f"{'前' if sign > 0 else '后'}向防护区激光覆盖", "value": f"{seen}/{len(pts)}",
                        "severity": sev, "note": "低速档停车区边沿采样点被激光视场覆盖的比例" + ("" if sev == "ok" else "，存在盲区")})
    return out


def _lidar_sees(lidars, p) -> bool:
    for L in lidars:
        x, y = float(L.get("x", 0.0)), float(L.get("y", 0.0))
        dx, dy = p[0] - x, p[1] - y
        rng = math.hypot(dx, dy)
        if rng < float(L.get("min_range", 0.05)) or rng > float(L.get("max_range", 20.0)):
            continue
        if L.get("type") == "3d":
            return True
        fov = math.radians(float(L.get("fov_deg", 270.0)))
        inv = -1.0 if (math.cos(float(L.get("roll", 0.0))) < -0.5) else 1.0
        a = math.atan2(dy, dx)
        rel = math.atan2(math.sin(a - float(L.get("yaw", 0.0))), math.cos(a - float(L.get("yaw", 0.0)))) * inv
        if abs(rel) <= fov / 2 + 1e-6:
            return True
    return False
