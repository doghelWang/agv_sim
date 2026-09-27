#!/usr/bin/env python3
"""
AMR Studio V4 · CModel → 仿真机器人描述 (robot_config.json v2 + robot.urdf)

解析链路:
  .cmodel (ZIP+Protobuf) ──cmodel_proto.decoder──▶ CompDesc.json
        ──extract_robot_spec()──▶ robot_config.json (schema v2, 仿真引擎/Nav2 共用)
        ──generate_urdf()──────▶ robot.urdf (robot_state_publisher / RViz；仿真由 robot spec 生成 MJCF)

相比旧版解析器的还原度提升:
  * 递归遍历嵌套模块组 (旧版只遍历一层 → 轮组/电机全部漏读)
  * 同时兼容 privateAttr(新格式) 与 structParam(旧格式) 两种属性位置
  * Protobuf3 省略零值字段 → 缺省按 0 处理 (旧版把 locCoordX=0 的部件当"无坐标")
  * 底盘类型由 底盘子类型 + 轮组子类型 共同推断 (单舵轮/双舵轮/差速/差速舵轮/多舵轮)
  * 轮组: 位置、半径、舵角限位、转向速率、关联行走/转向电机 (额定转速/减速比 → 轮端极限速度)
  * 空载/满载两套运动参数 (--load idle|full)
  * 激光: 安装 6DoF 位姿、倒装(roll=180°)、2D/3D、扫描方向；型号→参数库(视场/量程/分辨率/频率)
  * 读码相机(朝上/朝下)、陀螺仪/IMU、急停按钮、IO 接近开关/DO、电池、顶升/门架电机
  * 由 motionCenterAttr 生成真实车身轮廓 footprint (运动中心 ≠ 几何中心)
  * 所有推断值记录在 "assumptions" 中，便于核对
"""

import json
import math
import os
import sys
import tempfile
from typing import Any, Dict, List, Optional, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

SCHEMA_VERSION = 2

# ---------------------------------------------------------------------------
# 传感器型号参数库 (cmodel 不含视场/量程时使用；可用 --overrides 覆盖)
# ---------------------------------------------------------------------------
# Livox Mid-360 / Mid-360S: 非重复扫描 3D 激光 (官方规格 livoxtech.com/mid-360s/specs)
MID360S_SPEC = {"type": "3d", "vendor_model": "Livox Mid-360S", "fov_deg": 360.0, "vfov_min_deg": -7.0, "vfov_max_deg": 52.0,
                "min_range": 0.1, "max_range": 40.0, "range_cutoff_m": 100.0, "point_rate": 200000, "freq_hz": 10.0,
                "range_noise_std": 0.02, "angular_noise_deg": 0.15, "lines": 4, "scan_pattern": "non_repetitive",
                "imu": "ICM40609", "resolution_deg": 0.5, "housing_box": [0.065, 0.065, 0.060],
                "topic": "/livox/lidar", "imu_topic": "/livox/imu"}
MID360_SPEC = dict(MID360S_SPEC, vendor_model="Livox Mid-360", imu="ICM40609")

LIDAR_LIBRARY = [
    ("mid-360s", MID360S_SPEC), ("mid360s", MID360S_SPEC), ("mid-360", MID360_SPEC), ("mid360", MID360_SPEC),
    # (匹配关键字(小写), 参数)
    ("pepperl", {"fov_deg": 360.0, "min_range": 0.1, "max_range": 30.0, "resolution_deg": 0.25, "freq_hz": 20.0, "range_noise_std": 0.012, "vendor_model": "Pepperl+Fuchs R2000"}),
    ("r2000", {"fov_deg": 360.0, "min_range": 0.1, "max_range": 30.0, "resolution_deg": 0.25, "freq_hz": 20.0, "range_noise_std": 0.012, "vendor_model": "Pepperl+Fuchs R2000"}),
    ("mr-ls-07h", {"fov_deg": 270.0, "min_range": 0.05, "max_range": 20.0, "resolution_deg": 0.33, "freq_hz": 25.0, "range_noise_std": 0.02, "vendor_model": "MR-LS-07H 安全激光"}),
    ("ls-05h", {"fov_deg": 270.0, "min_range": 0.05, "max_range": 15.0, "resolution_deg": 0.33, "freq_hz": 25.0, "range_noise_std": 0.02, "vendor_model": "MR-LS-05H 激光"}),
    ("sick", {"fov_deg": 270.0, "min_range": 0.05, "max_range": 25.0, "resolution_deg": 0.25, "freq_hz": 25.0, "range_noise_std": 0.015, "vendor_model": "SICK"}),
    ("nanoscan", {"fov_deg": 275.0, "min_range": 0.05, "max_range": 30.0, "resolution_deg": 0.17, "freq_hz": 25.0, "range_noise_std": 0.015, "vendor_model": "SICK nanoScan3"}),
]
LIDAR_DEFAULT = {"fov_deg": 270.0, "min_range": 0.05, "max_range": 25.0, "resolution_deg": 0.25, "freq_hz": 15.0, "range_noise_std": 0.015, "vendor_model": "generic 2D"}
LIDAR3D_DEFAULT = {"fov_deg": 360.0, "min_range": 0.3, "max_range": 40.0, "resolution_deg": 0.2, "freq_hz": 10.0, "range_noise_std": 0.02, "vertical_fov_deg": 30.0, "vendor_model": "generic 3D"}

STEER_WHEEL_SUBTYPES = {"verticalSteerWheel", "horizontalSteerWheel", "steerWheel", "diffSteerWheel"}
DIFF_WHEEL_SUBTYPES = {"diffWheel", "driveWheel", "fixedWheel"}


# ---------------------------------------------------------------------------
# CompDesc 读取工具
# ---------------------------------------------------------------------------
def _value(e: Dict[str, Any], default=None):
    """Protobuf JSON 字段取值（proto3 省略零值 → 返回 default）"""
    if not isinstance(e, dict):
        return default
    for k in ("doubleValue", "floatValue", "int32Value", "int64Value", "intValue",
              "uint32Value", "boolValue", "stringValue", "stringFix"):
        if k in e:
            v = e[k]
            if k == "int64Value":
                try:
                    v = int(v)
                except Exception:
                    pass
            return v
    if "comboType" in e:
        return (e.get("comboType") or {}).get("typeKey", default)
    return default


def _num(v, default=0.0) -> float:
    try:
        if v is None or v == "" or v == "FIXED_RELATED_NONE":
            return default
        return float(v)
    except (TypeError, ValueError):
        return default


def walk_components(groups: List[dict], parent_group: str = ""):
    """递归遍历 moreModuleInfo (模块组可无限嵌套)"""
    for g in groups or []:
        gname = g.get("moduleGroupName")
        if isinstance(gname, dict):
            gname = gname.get("stringValue", "")
        for c in g.get("moduleComponets") or g.get("moduleComponents") or []:
            yield (gname or parent_group), c
        yield from walk_components(g.get("moreModuleInfo") or [], gname or parent_group)


class Component:
    def __init__(self, raw: dict, group: str):
        self.raw = raw
        self.group = group
        gen = raw.get("generalAttr", {}) or {}
        self.name = _value(gen.get("moduleName"), "") or group or "unnamed"
        self.uuid = _value(gen.get("moduleUuid"), "")
        self.main_type = _value(gen.get("mainModuleType"), "") or ""
        self.sub_type = _value(gen.get("subModuleType"), "") or ""
        self.model_file_3d = _value(gen.get("module3dIcon"), "")
        self.dsc_type = _value(gen.get("moduleDscType"), "") or ""
        self.vendor = _value(gen.get("venderName"), "") or ""
        self.shape = gen.get("moduleShape") or {}
        ext = {e.get("key"): _value(e) for e in gen.get("extendParams", []) or []}
        self.src_name = ext.get("module_srcname") or ""
        self.alias = ext.get("module_alias") or ""

        # structParam: 位姿 + 旧格式属性
        self.struct = {}
        for p in (raw.get("structParam") or {}).get("extendParams", []) or []:
            self.struct[p.get("key")] = _value(p)
        # privateAttr: 分组属性（新格式），展平为 {key: value}，并保留组合框子项
        self.attrs: Dict[str, Any] = {}
        self.attr_groups: Dict[str, Dict[str, Any]] = {}
        self.combo_children: Dict[str, Dict[str, Any]] = {}
        pa = raw.get("privateAttr") or raw.get("privateAttrs") or {}
        for grp in pa.get("privateAttrs", []) or []:
            gk = grp.get("key", "")
            d = {}
            for ele in grp.get("arrayBaseEle", []) or []:
                k = ele.get("key")
                v = _value(ele)
                d[k] = v
                self.attrs.setdefault(k, v)
                # 组合框的已选子组参数 (如 angleSensor → gearRatio / relatedEncode)
                ct = ele.get("comboType") or {}
                sel = ct.get("typeKey")
                for tg in ct.get("typeGroups", []) or []:
                    if tg.get("key") == sel and tg.get("arrayCmobEle"):
                        self.combo_children[k] = {x.get("key"): _value(x) for x in tg["arrayCmobEle"]}
            self.attr_groups[gk] = d

    # 属性查找: privateAttr 优先，其次 structParam
    def get(self, key: str, default=None):
        if key in self.attrs and self.attrs[key] is not None:
            return self.attrs[key]
        if key in self.struct and self.struct[key] is not None:
            return self.struct[key]
        return default

    def num(self, key: str, default=0.0) -> float:
        return _num(self.get(key), default)

    def pose(self) -> Dict[str, float]:
        """mm/deg → m/rad；proto3 缺省字段 = 0"""
        s = self.struct
        return {
            "x": _num(s.get("locCoordX")) / 1000.0,
            "y": _num(s.get("locCoordY")) / 1000.0,
            "z": _num(s.get("locCoordZ")) / 1000.0,
            "roll": math.radians(_num(s.get("locCoordROLL"))),
            "pitch": math.radians(_num(s.get("locCoordPITCH"))),
            "yaw": math.radians(_num(s.get("locCoordYAW"))),
        }

    def parent_uuid(self) -> str:
        v = self.struct.get("parentNodeUuid")
        return v if isinstance(v, str) and v not in ("None", "") else ""

    def box_size(self) -> Optional[Tuple[float, float, float]]:
        b = self.shape.get("box")
        if isinstance(b, dict) and b:
            return (_num(b.get("sizeLen")) / 1000.0, _num(b.get("sizeWidth")) / 1000.0, _num(b.get("sizeHeight")) / 1000.0)
        return None

    def cylinder_size(self) -> Optional[Tuple[float, float]]:
        c = self.shape.get("cylinder")
        if isinstance(c, dict) and c:
            return (_num(c.get("diameter")) / 1000.0, _num(c.get("height")) / 1000.0)
        return None


# ---------------------------------------------------------------------------
# 主解析
# ---------------------------------------------------------------------------
def _load_pair(c: Component, idle_key: str, full_key: str, load: str) -> float:
    v_idle = c.num(idle_key)
    v_full = c.num(full_key)
    if load == "full":
        return v_full if v_full > 0 else v_idle
    return v_idle if v_idle > 0 else v_full


def _motor_info(c: Optional[Component]) -> Optional[dict]:
    if c is None:
        return None
    rpm = c.num("RPM") or c.num("ratedSpeed")
    gear = c.num("gearRatio")
    return {
        "name": c.name,
        "model": c.src_name or c.dsc_type,
        "rated_rpm": rpm,
        "gear_ratio": gear,
        "reverse": bool(c.get("bReverse") is True),
        "holding_brake": c.get("bHbrake") in (True, "WITH_HBRAKE"),
        "encoder": c.get("ENCType") or "",
    }


def _lidar_params(c: Component, is3d: bool) -> dict:
    key = f"{c.src_name} {c.dsc_type} {c.name}".lower()
    for kw, params in LIDAR_LIBRARY:
        if kw in key:
            return dict(params, spec_source=f"library:{kw}")
    base = LIDAR3D_DEFAULT if is3d else LIDAR_DEFAULT
    return dict(base, spec_source="library:default")


def extract_robot_spec(comp_desc: dict, model_filename: str = "", load: str = "full",
                       overrides: Optional[dict] = None) -> dict:
    comps = [Component(c, g) for g, c in walk_components(comp_desc.get("moreModuleInfo", []))]
    by_name = {c.name: c for c in comps}
    by_uuid = {c.uuid: c for c in comps if c.uuid}
    assumptions: List[str] = []
    warnings: List[str] = []

    # ---------------- 底盘 ----------------
    chassis_c = next((c for c in comps if c.main_type == "chassis"), None)
    if chassis_c is None:
        warnings.append("未找到 chassis 模块，使用默认底盘参数")
    ch = chassis_c or Component({}, "chassis")

    head = _load_pair(ch, "headOffset(Idle)", "headOffset (Full Load)", load) / 1000.0
    tail = _load_pair(ch, "tailOffset(Idle)", "tailOffset (Full Load)", load) / 1000.0
    left = _load_pair(ch, "leftOffset(Idle)", "leftOffset (Full Load)", load) / 1000.0
    right = _load_pair(ch, "rightOffset(Idle)", "rightOffset (Full Load)", load) / 1000.0
    box = ch.box_size()
    if head <= 0 and tail <= 0:
        L = box[0] if box and box[0] > 0 else 1.2
        head = tail = L / 2.0
        assumptions.append("底盘 head/tail offset 缺失，按外形长度对半")
    if left <= 0 and right <= 0:
        W = box[1] if box and box[1] > 0 else 0.8
        left = right = W / 2.0
        assumptions.append("底盘 left/right offset 缺失，按外形宽度对半")
    height = box[2] if box and box[2] > 0 else 0.35
    if not (box and box[2] > 0):
        assumptions.append("底盘高度缺失，默认 0.35 m")

    def motion(load_state: str) -> dict:
        v = _load_pair(ch, "maxSpeed(Idle)", "maxSpeed (Full Load)", load_state) / 1000.0
        a = _load_pair(ch, "maxAcceleration(Idle)", "maxAcceleration (Full Load)", load_state) / 1000.0
        d = _load_pair(ch, "maxDeceleration(Idle)", "maxDeceleration (Full Load)", load_state) / 1000.0
        w = math.radians(_load_pair(ch, "rotateMaxAngSpeed (Idle)", "rotateMaxAngSpeed (Full Load)", load_state))
        wa = math.radians(_load_pair(ch, "rotateMaxAngAcceleration (Idle)", "rotateMaxAngAcceleration (Full Load)", load_state))
        wd = math.radians(_load_pair(ch, "rotateMaxAngDeceleration (Idle)", "rotateMaxAngDeceleration (Full Load)", load_state))
        ad = _load_pair(ch, "avoidMaxDec (Idle)", "avoidMaxDec (Full Load)", load_state) / 1000.0
        return {
            "max_speed_mps": round(v or 1.0, 4),
            "max_accel_mps2": round(a or 0.5, 4),
            "max_decel_mps2": round(d or a or 0.5, 4),
            "max_ang_speed_radps": round(w or 1.0, 4),
            "max_ang_accel_radps2": round(wa or 1.0, 4),
            "max_ang_decel_radps2": round(wd or wa or 1.0, 4),
            "avoid_max_decel_mps2": round(ad or d or a or 0.5, 4),
        }

    motion_idle = motion("idle")
    motion_full = motion("full")
    active_motion = motion_full if load == "full" else motion_idle

    # ---------------- 电机 ----------------
    motors = {c.name: c for c in comps if c.main_type == "driver" and c.sub_type in ("PMSMMotor", "BDCMotor", "stepMotor", "motor")}

    # ---------------- 轮组 ----------------
    wheels: List[dict] = []
    for c in comps:
        if c.main_type != "driveWheel" and not (c.main_type == "driver" and c.num("wheelRadius") > 0):
            continue
        p = c.pose()
        r = c.num("wheelRadius") / 1000.0 or 0.1
        sub = c.sub_type
        w = {
            "name": c.name.replace(" ", "_").replace("-", "_"),
            "cmodel_name": c.name,
            "cmodel_subtype": sub,
            "x": round(p["x"], 4), "y": round(p["y"], 4), "z": round(r, 4),
            "radius_m": round(r, 4),
            "width_m": round(max(0.04, r * 0.6), 4),
        }
        if sub in STEER_WHEEL_SUBTYPES:
            w["kind"] = "steer"
            lim_p = c.num("angleLmtPos", 0.0)
            lim_n = c.num("angleLmtNeg", 0.0)
            if lim_p <= 0 and lim_n >= 0:
                lim_p, lim_n = 90.0, -90.0
                assumptions.append(f"{c.name}: 舵角限位缺失，默认 ±90°")
            w["steer_min_rad"] = round(math.radians(lim_n), 5)
            w["steer_max_rad"] = round(math.radians(lim_p), 5)
            walk_m = motors.get(c.get("relateWalkMotor") or "")
            rot_m = motors.get(c.get("relateRotMotor") or "")
            lm = motors.get(c.get("relateLeftMotor") or "")
            rm = motors.get(c.get("relateRightMotor") or "")
            if sub == "diffSteerWheel":
                # 差速舵轮: 两个行走电机差速实现转向
                ws = c.num("wheelSpace") / 1000.0 or 0.25
                w["diff_steer_wheel_space_m"] = round(ws, 4)
                w["drive_motor"] = _motor_info(lm or rm)
                w["drive_motor_right"] = _motor_info(rm)
            else:
                w["drive_motor"] = _motor_info(walk_m)
                w["steer_motor"] = _motor_info(rot_m)
            # 行走极限
            dm = w.get("drive_motor")
            if dm and dm["rated_rpm"] > 0 and dm["gear_ratio"] > 0:
                w["max_speed_mps"] = round(dm["rated_rpm"] / dm["gear_ratio"] * 2 * math.pi / 60.0 * r, 4)
            # 转向速率
            rate = c.num("rotOmgLmt")
            if rate > 0:
                w["steer_rate_radps"] = round(math.radians(rate), 4)
            elif w.get("steer_motor") and w["steer_motor"]["rated_rpm"] > 0 and w["steer_motor"]["gear_ratio"] > 0:
                sm = w["steer_motor"]
                w["steer_rate_radps"] = round(sm["rated_rpm"] / sm["gear_ratio"] * 2 * math.pi / 60.0, 4)
            elif sub == "diffSteerWheel" and w.get("max_speed_mps"):
                w["steer_rate_radps"] = round(2 * w["max_speed_mps"] * 0.3 / w["diff_steer_wheel_space_m"], 4)
                assumptions.append(f"{c.name}: 差速舵轮转向速率按 30% 差速能力估算")
            else:
                w["steer_rate_radps"] = round(math.radians(90.0), 4)
                assumptions.append(f"{c.name}: 转向速率缺失，默认 90°/s")
            asens = c.get("angleSensorType")
            if asens:
                w["angle_sensor"] = asens
                if c.combo_children.get("angleSensorType"):
                    w["angle_sensor_params"] = c.combo_children["angleSensorType"]
        else:
            w["kind"] = "drive"  # 固定方向驱动轮 (差速轮)
            m = motors.get(c.get("relateMotor") or "")
            w["drive_motor"] = _motor_info(m)
            if m:
                mi = w["drive_motor"]
                if mi["rated_rpm"] > 0 and mi["gear_ratio"] > 0:
                    w["max_speed_mps"] = round(mi["rated_rpm"] / mi["gear_ratio"] * 2 * math.pi / 60.0 * r, 4)
        wheels.append(w)

    # 未关联电机的轮子: 若 cmodel 中恰好有同侧电机名，尝试按名字匹配 (left/right)
    for w in wheels:
        if w.get("drive_motor") is None and w["kind"] == "drive":
            side = "left" if w["y"] > 0 else "right"
            cand = [m for n, m in motors.items() if side in n.lower() or (side == "left" and "lft" in n.lower())]
            if cand:
                w["drive_motor"] = _motor_info(cand[0])
                mi = w["drive_motor"]
                if mi["rated_rpm"] > 0 and mi["gear_ratio"] > 0:
                    w["max_speed_mps"] = round(mi["rated_rpm"] / mi["gear_ratio"] * 2 * math.pi / 60.0 * w["radius_m"], 4)
                assumptions.append(f"{w['cmodel_name']}: 按名称匹配行走电机 {mi['name']}")

    # ---------------- 底盘类型推断 ----------------
    steer = [w for w in wheels if w["kind"] == "steer"]
    drive = [w for w in wheels if w["kind"] == "drive"]
    ch_sub = ch.sub_type
    if len(steer) == 1 and not drive:
        ctype = "single_steer"
    elif len(steer) >= 2:
        ctype = "dual_steer" if len(steer) == 2 else "multi_steer"
    elif len(drive) >= 2:
        ctype = "diff_drive"
    elif ch_sub == "diffChassis":
        ctype = "diff_drive"
    elif ch_sub == "steerChassis":
        ctype = "single_steer"
    else:
        ctype = "diff_drive"

    # 卧式舵轮成对左右布置且 x 相同 → 实际表现为差速（轮间距=左右距离），如 proj_1234
    if ctype == "dual_steer" and all(w["cmodel_subtype"] == "horizontalSteerWheel" for w in steer) \
            and abs(steer[0]["x"] - steer[1]["x"]) < 1e-3 and abs(steer[0]["y"] + steer[1]["y"]) < 1e-3 \
            and "steer_max_rad" in steer[0] and steer[0].get("steer_motor") is None and not steer[0].get("angle_sensor"):
        for w in steer:
            w["kind"] = "drive"
            for k in ("steer_min_rad", "steer_max_rad", "steer_rate_radps"):
                w.pop(k, None)
        ctype = "diff_drive"
        assumptions.append("两个卧式舵轮左右对称且无转向电机/角度传感器 → 按差速驱动处理")

    # 若 cmodel 无轮组: 按底盘类型补齐
    if not wheels:
        wheel_space = ch.num("wheelSpace") / 1000.0 or (left + right) * 0.8
        r = 0.1
        if ctype == "diff_drive":
            for side, y in (("left", wheel_space / 2), ("right", -wheel_space / 2)):
                wheels.append({"name": f"{side}_wheel", "kind": "drive", "x": 0.0, "y": round(y, 4), "z": r,
                               "radius_m": r, "width_m": 0.05, "source": "inferred"})
        else:
            wheels.append({"name": "steer_wheel", "kind": "steer", "x": round(head * 0.8, 4), "y": 0.0, "z": r,
                           "radius_m": r, "width_m": 0.06, "steer_min_rad": -math.pi / 2, "steer_max_rad": math.pi / 2,
                           "steer_rate_radps": math.radians(90), "source": "inferred"})
        assumptions.append("cmodel 无轮组模块，按底盘类型推断轮组布局")

    # 差速轮距校验: 底盘 wheelSpace 与轮子 y 坐标
    ws_cfg = ch.num("wheelSpace") / 1000.0
    drive = [w for w in wheels if w["kind"] == "drive"]
    if ctype == "diff_drive" and len(drive) >= 2:
        track = max(w["y"] for w in drive) - min(w["y"] for w in drive)
        if ws_cfg > 0 and abs(ws_cfg - track) > 0.01:
            warnings.append(f"底盘 wheelSpace={ws_cfg:.3f}m 与轮子坐标轮距 {track:.3f}m 不一致，以轮子坐标为准")

    # ---------------- 被动轮 (cmodel 不建模 → 推断，保证物理支撑) ----------------
    passive = []
    margin = 0.12
    if ctype == "single_steer":
        # 单舵轮: 后桥两个定向承重轮位于运动中心横线上 (运动中心 = 后桥中心，舵轮前置)
        sw = steer[0] if steer else wheels[0]
        rr = sw["radius_m"] * 0.75
        axle_x = 0.0
        for side, y in (("left", left - margin), ("right", -(right - margin))):
            passive.append({"name": f"rear_{side}_load_wheel", "kind": "fixed", "x": axle_x, "y": round(y, 4),
                            "z": round(rr, 4), "radius_m": round(rr, 4), "width_m": 0.08, "source": "inferred"})
        assumptions.append("单舵轮: 在运动中心(后桥)两侧推断 2 个定向承重轮")
    elif ctype == "diff_drive":
        cr = min(w["radius_m"] for w in wheels) * 0.6
        for tag, x in (("front", head - margin - cr), ("rear", -(tail - margin - cr))):
            for side, y in (("left", left - margin - cr), ("right", -(right - margin - cr))):
                passive.append({"name": f"caster_{tag}_{side}", "kind": "caster", "x": round(x, 4), "y": round(y, 4),
                                "z": round(cr, 4), "radius_m": round(cr, 4), "source": "inferred"})
        assumptions.append("差速: 在车体四角推断 4 个万向脚轮")
    else:
        # 双/多舵轮: 在未被舵轮占据的对角推断万向轮
        cr = min(w["radius_m"] for w in wheels) * 0.7
        corners = [(head - margin - cr, left - margin - cr), (head - margin - cr, -(right - margin - cr)),
                   (-(tail - margin - cr), left - margin - cr), (-(tail - margin - cr), -(right - margin - cr))]
        for i, (x, y) in enumerate(corners):
            if all(math.hypot(x - w["x"], y - w["y"]) > 0.35 for w in wheels):
                passive.append({"name": f"caster_{i}", "kind": "caster", "x": round(x, 4), "y": round(y, 4),
                                "z": round(cr, 4), "radius_m": round(cr, 4), "source": "inferred"})
        if passive:
            assumptions.append(f"舵轮底盘: 推断 {len(passive)} 个万向承重脚轮")
    wheels_all = wheels + passive

    # 轮端能力 vs 底盘设定
    wheel_caps = [w["max_speed_mps"] for w in wheels if w.get("max_speed_mps")]
    if wheel_caps and min(wheel_caps) + 1e-3 < active_motion["max_speed_mps"]:
        warnings.append(
            f"电机额定转速/减速比限制轮端最高 {min(wheel_caps):.3f} m/s，低于底盘设定 {active_motion['max_speed_mps']:.3f} m/s；仿真按轮端能力限速")

    # ---------------- 传感器 ----------------
    lidars, cameras, io_inputs, io_outputs, imus = [], [], [], [], []
    for c in comps:
        if c.main_type != "sensor":
            continue
        p = c.pose()
        nm = c.name.replace(" ", "_").replace("-", "_")
        if c.sub_type in ("laser", "3DLaser", "lidar", "safetyLaser"):
            is3d = c.sub_type == "3DLaser"
            prm = _lidar_params(c, is3d)
            fov = prm["fov_deg"]
            li = {
                "name": nm, "cmodel_name": c.name, "type": "3d" if is3d else "2d",
                "model": c.src_name or c.dsc_type or "",
                "x": round(p["x"], 4), "y": round(p["y"], 4), "z": round(p["z"], 4),
                "roll": round(p["roll"], 6), "pitch": round(p["pitch"], 6), "yaw": round(p["yaw"], 6),
                "inverted": abs(math.cos(p["roll"])) > 0.5 and math.cos(p["roll"]) < 0,
                "scan_direction": c.get("scanDirect") or "SCAN_COUNTERCLOCKWISE",
                "usage": c.get("usageMode") or "",
                "wavelength_nm": c.num("waveLength"),
                "anti_light_klux": c.num("antiLight"),
                "angle_min": round(-math.radians(fov) / 2.0, 6),
                "angle_max": round(math.radians(fov) / 2.0, 6),
            }
            li.update({k: v for k, v in prm.items()})
            cyl = c.cylinder_size()
            bx = c.box_size()
            li["housing"] = {"cylinder_d_h": cyl} if cyl else ({"box": bx} if bx else {})
            if is3d:
                assumptions.append(f"{c.name}: 3D 激光在 2D 仿真中按其安装高度水平切片输出")
            lidars.append(li)
        elif c.sub_type in ("codeReader", "camera", "depthCamera"):
            cameras.append({
                "name": nm, "cmodel_name": c.name, "type": c.sub_type,
                "model": c.src_name or c.dsc_type, "orientation": c.get("lensOrientation") or "LENS_DIR_FRONT",
                "x": round(p["x"], 4), "y": round(p["y"], 4), "z": round(p["z"], 4),
                "roll": round(p["roll"], 6), "pitch": round(p["pitch"], 6), "yaw": round(p["yaw"], 6),
                "read_window_m": 0.05 if (c.get("lensOrientation") or "").endswith("DOWN") else 0.08,
                "scan_distance_m": c.num("scanDistence") / 1000.0 or None,
                "accuracy_mm": c.num("accuracy") or None,
            })
        elif c.sub_type == "gyro":
            imus.append({"name": nm, "x": round(p["x"], 4), "y": round(p["y"], 4), "z": round(p["z"], 4),
                         "angular_resolution": c.num("angularResolution"), "source": "sensor/gyro"})
        elif c.sub_type in ("proximitySensor", "PT", "photoelectric"):
            io_inputs.append({"name": nm, "type": c.sub_type, "cmodel_name": c.name,
                              "x": round(p["x"], 4), "y": round(p["y"], 4), "z": round(p["z"], 4), "yaw": round(p["yaw"], 6)})
        elif c.sub_type in ("comDo",):
            io_outputs.append({"name": nm, "type": c.sub_type})
    # 主控内置陀螺
    for c in comps:
        if c.main_type == "mainCPU" and (c.get("isWithGyro") in ("yes", True)) and not imus:
            p = c.pose()
            imus.append({"name": "imu", "x": round(p["x"], 4), "y": round(p["y"], 4), "z": round(p["z"], 4) or 0.1,
                         "source": f"mainCPU:{c.name} (内置陀螺)"})
    if not imus:
        imus.append({"name": "imu", "x": 0.0, "y": 0.0, "z": 0.1, "source": "default"})
    if not lidars:
        prm = dict(LIDAR_DEFAULT, spec_source="library:default")
        lidars.append({"name": "laser_front", "type": "2d", "model": "", "x": round(head * 0.9, 4), "y": 0.0, "z": 0.25,
                       "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "inverted": False,
                       "angle_min": -math.radians(prm["fov_deg"]) / 2, "angle_max": math.radians(prm["fov_deg"]) / 2, **prm})
        assumptions.append("cmodel 无激光模块，默认车头 1 个 270° 激光")

    buttons = []
    for c in comps:
        if c.main_type == "button":
            buttons.append({"name": c.name, "type": c.get("buttonType") or "", "self_lock": bool(c.get("selfLock"))})
    battery = None
    for c in comps:
        if c.main_type == "battery":
            battery = {"name": c.name, "voltage_v": c.num("voltage"), "capacity_ah": c.num("capacity"),
                       "max_discharge_a": c.num("maxDischargeCurrent"), "std_discharge_a": c.num("stdDischargeCurrent"),
                       "max_power_w": c.num("maxPower")}
            break
    actuators = []
    used_motor_names = set()
    for w in wheels:
        for k in ("drive_motor", "drive_motor_right", "steer_motor"):
            if w.get(k):
                used_motor_names.add(w[k]["name"])
    for n, m in motors.items():
        if n not in used_motor_names:
            actuators.append(_motor_info(m))

    self_weight = ch.num("selfWeight")
    load_weight = ch.num("totalLoadWeight")
    mass = self_weight if self_weight > 0 else round(max(80.0, (head + tail) * (left + right) * 150.0), 1)
    if self_weight <= 0:
        assumptions.append(f"自重缺失，按投影面积估算 {mass} kg")

    footprint = [[round(head, 4), round(left, 4)], [round(head, 4), round(-right, 4)],
                 [round(-tail, 4), round(-right, 4)], [round(-tail, 4), round(left, 4)]]

    # 兼容旧字段 (web/旧代码读取)
    drive_ref = next((w for w in wheels if w["kind"] in ("drive", "steer")), wheels[0])
    dws = [w for w in wheels if w["kind"] == "drive"]
    legacy_track = (max(w["y"] for w in dws) - min(w["y"] for w in dws)) if len(dws) >= 2 else (left + right) * 0.8

    spec = {
        "schema_version": SCHEMA_VERSION,
        "model_file": model_filename,
        "robot_name": "cmodel_agv",
        "load_state": load,
        "chassis": {
            "type": ctype,
            "cmodel_type": ch_sub,
            "cmodel_name": ch.name,
            "length_m": round(head + tail, 4),
            "width_m": round(left + right, 4),
            "height_m": round(height, 4),
            "head_offset_m": round(head, 4),
            "tail_offset_m": round(tail, 4),
            "left_offset_m": round(left, 4),
            "right_offset_m": round(right, 4),
            "footprint": footprint,
            "rotate_diameter_m": round(ch.num("rotateDiameter") / 1000.0, 4),
            "max_climb_deg": ch.num("maxClimbingAngle"),
            "self_weight_kg": self_weight,
            "mass_kg": mass,
            "max_load_kg": load_weight,
            **active_motion,
            "motion_idle": motion_idle,
            "motion_full_load": motion_full,
        },
        "wheels": wheels_all,
        "drive_wheels": {
            "radius_m": drive_ref["radius_m"],
            "track_width_m": round(legacy_track, 4),
            "wheel_width_m": drive_ref.get("width_m", 0.05),
            "x_offset_m": drive_ref["x"],
        },
        "lidars": lidars,
        "cameras": cameras,
        "imu": imus[0],
        "io": {"inputs": io_inputs, "outputs": io_outputs, "buttons": buttons},
        "battery": battery,
        "other_actuators": actuators,
        "assumptions": assumptions,
        "warnings": warnings,
    }

    if overrides:
        _apply_overrides(spec, overrides)
    return spec


def _apply_overrides(spec: dict, ov: dict):
    """overrides.json: {"chassis": {...}, "lidars": {"laser": {...}}, "wheels": {"Steerwheel": {...}}}
    激光可写 {"library": "mid-360s"} 直接套用型号参数库 (再叠加其余字段)"""
    for name, patch in (ov.get("lidars") or {}).items():
        lib = str(patch.get("library", "")).lower()
        if lib:
            base = next((p for kw, p in LIDAR_LIBRARY if kw == lib), None)
            if base:
                full = dict(base, spec_source=f"override:{lib}")
                full.update({k: v for k, v in patch.items() if k != "library"})
                ov["lidars"][name] = full
    for k, v in (ov.get("chassis") or {}).items():
        spec["chassis"][k] = v
    for sec in ("lidars", "wheels", "cameras"):
        for name, patch in (ov.get(sec) or {}).items():
            for item in spec.get(sec, []):
                if item.get("name") == name or item.get("cmodel_name") == name:
                    item.update(patch)
                    if sec == "lidars":
                        fov = float(item.get("fov_deg", 270.0))
                        item["angle_min"] = round(-math.radians(fov) / 2.0, 6)
                        item["angle_max"] = round(math.radians(fov) / 2.0, 6)
                        if item.get("housing_box"):
                            item["housing"] = {"box": item["housing_box"]}
    spec["assumptions"].append("已应用 overrides 覆盖参数")


# ---------------------------------------------------------------------------
# URDF 生成
# ---------------------------------------------------------------------------
_SMALL_INERTIAL = ('<inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.05"/>'
                   '<inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/></inertial>')


def _box_inertia(m, x, y, z):
    return (m * (y * y + z * z) / 12.0, m * (x * x + z * z) / 12.0, m * (x * x + y * y) / 12.0)


def _cyl_inertia(m, r, h):
    ixx = m * (3 * r * r + h * h) / 12.0
    return (ixx, m * r * r / 2.0, ixx)  # 轴沿 y


def _inertial(xml, m, ixx, iyy, izz, origin="0 0 0"):
    xml.append(f'    <inertial><origin xyz="{origin}" rpy="0 0 0"/><mass value="{m:.4f}"/>'
               f'<inertia ixx="{ixx:.5f}" ixy="0" ixz="0" iyy="{iyy:.5f}" iyz="0" izz="{izz:.5f}"/></inertial>')


def generate_urdf(spec: dict) -> str:
    ch = spec["chassis"]
    head, tail = ch["head_offset_m"], ch["tail_offset_m"]
    left, right = ch["left_offset_m"], ch["right_offset_m"]
    L, W, H = head + tail, left + right, ch["height_m"]
    x_c = (head - tail) / 2.0
    y_c = (left - right) / 2.0
    wheels = spec.get("wheels", [])
    r_min = min([w["radius_m"] for w in wheels] or [0.1])
    clearance = round(max(0.02, r_min * 0.5), 4)
    # 车体分两段: 底盘板 (碰撞/物理) + 上装 (视觉，高于 0.6m 的部分，如门架/立柱)
    base_h = min(H, 0.35) if H > 0.6 else H
    upper_h = H - base_h - clearance if H > 0.6 else 0.0
    mass = float(ch.get("mass_kg") or 150.0)
    wheel_mass = 5.0
    body_mass = max(10.0, mass - sum(float(w.get("mass_kg", wheel_mass)) for w in wheels))

    x = ['<?xml version="1.0"?>',
         f'<!-- Auto-generated by cmodel_parser.py from {spec.get("model_file", "")} (schema v{spec.get("schema_version")}) -->',
         f'<!-- chassis={ch["type"]} ({ch.get("cmodel_type", "")}), base_link = 运动中心(motion center), 地面高度 -->',
         f'<robot name="{spec.get("robot_name", "cmodel_agv")}">',
         '  <material name="body"><color rgba="0.95 0.55 0.1 1"/></material>',
         '  <material name="upper"><color rgba="0.85 0.85 0.88 0.6"/></material>',
         '  <material name="rubber"><color rgba="0.15 0.15 0.15 1"/></material>',
         '  <material name="steel"><color rgba="0.55 0.58 0.62 1"/></material>',
         '  <material name="sensor"><color rgba="0.0 0.8 0.95 1"/></material>',
         '  <material name="camera"><color rgba="0.6 0.2 0.9 1"/></material>',
         '  <link name="base_footprint"/>',
         '  <joint name="base_footprint_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/>'
         '<origin xyz="0 0 0" rpy="0 0 0"/></joint>',
         '  <link name="base_link">']
    zc = clearance + base_h / 2.0
    x.append(f'    <visual><origin xyz="{x_c:.4f} {y_c:.4f} {zc:.4f}"/><geometry><box size="{L:.4f} {W:.4f} {base_h:.4f}"/></geometry><material name="body"/></visual>')
    x.append(f'    <collision><origin xyz="{x_c:.4f} {y_c:.4f} {zc:.4f}"/><geometry><box size="{L:.4f} {W:.4f} {base_h:.4f}"/></geometry></collision>')
    ixx, iyy, izz = _box_inertia(body_mass, L, W, base_h)
    com = ch.get("com")
    com_s = f"{com[0]:.4f} {com[1]:.4f} {com[2]:.4f}" if isinstance(com, (list, tuple)) and len(com) == 3 else f"{x_c:.4f} {y_c:.4f} {zc:.4f}"
    _inertial(x, body_mass, ixx, iyy, izz, com_s)
    x.append('  </link>')
    if upper_h > 0.05:
        zu = clearance + base_h + upper_h / 2.0
        x.append('  <link name="upper_body_link">')
        x.append(f'    <visual><origin xyz="0 0 0"/><geometry><box size="{L:.4f} {W:.4f} {upper_h:.4f}"/></geometry><material name="upper"/></visual>')
        x.append('    ' + _SMALL_INERTIAL)
        x.append('  </link>')
        x.append(f'  <joint name="upper_body_joint" type="fixed"><parent link="base_link"/><child link="upper_body_link"/>'
                 f'<origin xyz="{x_c:.4f} {y_c:.4f} {zu:.4f}" rpy="0 0 0"/></joint>')

    for w in wheels:
        n, r = w["name"], w["radius_m"]
        wid = w.get("width_m", 0.05)
        kind = w["kind"]
        if kind == "caster":
            x.append(f'  <link name="{n}_link">')
            x.append(f'    <visual><geometry><sphere radius="{r:.4f}"/></geometry><material name="steel"/></visual>')
            x.append(f'    <collision><geometry><sphere radius="{r:.4f}"/></geometry></collision>')
            i = 0.4 * 1.0 * r * r
            _inertial(x, 1.0, i, i, i)
            x.append('  </link>')
            x.append(f'  <joint name="{n}_joint" type="fixed"><parent link="base_link"/><child link="{n}_link"/>'
                     f'<origin xyz="{w["x"]:.4f} {w["y"]:.4f} {r:.4f}" rpy="0 0 0"/></joint>')
            continue
        parent = "base_link"
        origin = f'{w["x"]:.4f} {w["y"]:.4f} {r:.4f}'
        if kind == "steer":
            x.append(f'  <link name="{n}_steer_link">')
            x.append(f'    <visual><origin xyz="0 0 {r * 0.9:.4f}"/><geometry><cylinder radius="{r * 0.8:.4f}" length="{r * 0.4:.4f}"/></geometry><material name="steel"/></visual>')
            _inertial(x, 2.0, 0.01, 0.01, 0.01)
            x.append('  </link>')
            eff = 500.0
            vel = w.get("steer_rate_radps", 1.5)
            x.append(f'  <joint name="{n}_steer_joint" type="revolute"><parent link="base_link"/><child link="{n}_steer_link"/>'
                     f'<origin xyz="{origin}" rpy="0 0 0"/><axis xyz="0 0 1"/>'
                     f'<limit lower="{w["steer_min_rad"]:.5f}" upper="{w["steer_max_rad"]:.5f}" effort="{eff}" velocity="{vel:.4f}"/></joint>')
            parent = f"{n}_steer_link"
            origin = "0 0 0"
        x.append(f'  <link name="{n}_link">')
        x.append(f'    <visual><origin xyz="0 0 0" rpy="1.5708 0 0"/><geometry><cylinder radius="{r:.4f}" length="{wid:.4f}"/></geometry><material name="rubber"/></visual>')
        x.append(f'    <collision><origin xyz="0 0 0" rpy="1.5708 0 0"/><geometry><cylinder radius="{r:.4f}" length="{wid:.4f}"/></geometry></collision>')
        wm = float(w.get("mass_kg", wheel_mass))
        ixx, iyy, izz = _cyl_inertia(wm, r, wid)
        _inertial(x, wm, ixx, iyy, izz)
        x.append('  </link>')
        jn = f"{n}_drive_joint" if kind == "steer" else f"{n}_joint"
        max_v = w.get("max_speed_mps", 2.0) / max(r, 1e-3)
        x.append(f'  <joint name="{jn}" type="continuous"><parent link="{parent}"/><child link="{n}_link"/>'
                 f'<origin xyz="{origin}" rpy="0 0 0"/><axis xyz="0 1 0"/><limit effort="200" velocity="{max_v:.3f}"/></joint>')

    imu = spec.get("imu") or {}
    x.append(f'  <link name="imu_link">{_SMALL_INERTIAL}</link>')
    x.append(f'  <joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/>'
             f'<origin xyz="{imu.get("x", 0):.4f} {imu.get("y", 0):.4f} {imu.get("z", 0.1):.4f}" rpy="0 0 0"/></joint>')

    for li in spec.get("lidars", []):
        n = li["name"]
        hs = li.get("housing") or {}
        if hs.get("cylinder_d_h"):
            d, h = hs["cylinder_d_h"]
            geo = f'<cylinder radius="{d / 2:.4f}" length="{h:.4f}"/>'
        elif hs.get("box"):
            bx = hs["box"]
            geo = f'<box size="{bx[0]:.4f} {bx[1]:.4f} {bx[2]:.4f}"/>'
        else:
            geo = '<cylinder radius="0.04" length="0.06"/>'
        x.append(f'  <link name="{n}_link"><visual><geometry>{geo}</geometry><material name="sensor"/></visual>{_SMALL_INERTIAL}</link>')
        x.append(f'  <joint name="{n}_joint" type="fixed"><parent link="base_link"/><child link="{n}_link"/>'
                 f'<origin xyz="{li["x"]:.4f} {li["y"]:.4f} {li["z"]:.4f}" rpy="{li["roll"]:.5f} {li["pitch"]:.5f} {li["yaw"]:.5f}"/></joint>')

    OPT = f"{-math.pi / 2:.5f} 0 {-math.pi / 2:.5f}"      # link(x 前) → optical(z 前, x 右, y 下)
    for cam in spec.get("cameras", []):
        n = cam["name"]
        ctype = {"depthCamera": "tof"}.get(cam.get("type"), cam.get("type"))
        pitch = float(cam.get("pitch", 0.0))
        if ctype in ("codeReader", None):
            # 读码相机: 朝下 pitch=+90°, 朝上 pitch=-90° (叠加 cmodel 姿态)
            o = cam.get("orientation", "")
            pitch += (math.pi / 2 if o.endswith("DOWN") else (-math.pi / 2 if o.endswith("UP") else 0.0))
        size = {"stereo": f"0.03 {float(cam.get('baseline_m', 0.05)) + 0.04:.3f} 0.03", "tof": "0.03 0.06 0.03"}.get(ctype, "0.03 0.05 0.03")
        x.append(f'  <link name="{n}_link"><visual><geometry><box size="{size}"/></geometry><material name="camera"/></visual>{_SMALL_INERTIAL}</link>')
        x.append(f'  <joint name="{n}_joint" type="fixed"><parent link="base_link"/><child link="{n}_link"/>'
                 f'<origin xyz="{cam["x"]:.4f} {cam["y"]:.4f} {cam["z"]:.4f}" rpy="{float(cam.get("roll", 0)):.5f} {pitch:.5f} {float(cam.get("yaw", 0)):.5f}"/></joint>')
        x.append(f'  <link name="{n}_optical_frame"/>')
        x.append(f'  <joint name="{n}_optical_joint" type="fixed"><parent link="{n}_link"/><child link="{n}_optical_frame"/>'
                 f'<origin xyz="0 0 0" rpy="{OPT}"/></joint>')
        if ctype == "stereo":
            b = float(cam.get("baseline_m", 0.05))
            x.append(f'  <link name="{n}_right_optical_frame"/>')
            x.append(f'  <joint name="{n}_right_optical_joint" type="fixed"><parent link="{n}_link"/><child link="{n}_right_optical_frame"/>'
                     f'<origin xyz="0 {-b:.4f} 0" rpy="{OPT}"/></joint>')

    for pe in spec.get("photoelectric", []):
        n = pe["name"]
        x.append(f'  <link name="{n}_pe_link"><visual><geometry><box size="0.02 0.02 0.02"/></geometry><material name="sensor"/></visual>{_SMALL_INERTIAL}</link>')
        x.append(f'  <joint name="{n}_pe_joint" type="fixed"><parent link="base_link"/><child link="{n}_pe_link"/>'
                 f'<origin xyz="{pe["x"]:.4f} {pe["y"]:.4f} {pe.get("z", 0.12):.4f}" rpy="0 0 {float(pe.get("yaw", 0)):.5f}"/></joint>')

    lift = spec.get("lift") or {}
    if lift.get("enabled"):
        sz = lift.get("size") or [1.0, 0.7, 0.04]
        stroke = float(lift.get("stroke_m", 0.08))
        x.append(f'  <link name="lift_link"><visual><geometry><box size="{sz[0]:.3f} {sz[1]:.3f} {sz[2]:.3f}"/></geometry><material name="steel"/></visual>'
                 f'<collision><geometry><box size="{sz[0]:.3f} {sz[1]:.3f} {sz[2]:.3f}"/></geometry></collision>')
        _inertial(x, float(lift.get("mass_kg", 20.0)), *_box_inertia(float(lift.get("mass_kg", 20.0)), sz[0], sz[1], sz[2]))
        x.append('  </link>')
        x.append(f'  <joint name="lift_joint" type="prismatic"><parent link="base_link"/><child link="lift_link"/>'
                 f'<origin xyz="{float(lift.get("x", 0)):.4f} {float(lift.get("y", 0)):.4f} {float(lift.get("z", 0.3)):.4f}" rpy="0 0 0"/>'
                 f'<axis xyz="0 0 1"/><limit lower="0" upper="{stroke:.4f}" effort="5000" velocity="{float(lift.get("speed_mps", 0.02)):.4f}"/></joint>')

    x.append('</robot>')
    return "\n".join(x) + "\n"


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def load_comp_desc(path: str) -> Tuple[dict, str]:
    """支持: .cmodel / 解码后的 CompDesc.json / 包含 CompDesc.json 的目录"""
    path = os.path.abspath(path)
    if os.path.isdir(path):
        path = os.path.join(path, "CompDesc.json")
    if path.lower().endswith(".json"):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f), os.path.basename(os.path.dirname(path)) or os.path.basename(path)
    from cmodel_proto.decoder import decode_cmodel_to_dicts
    return decode_cmodel_to_dicts(path)["CompDesc"], os.path.basename(path)


def parse_cmodel_file(cmodel_path: str, load: str = "full", overrides: Optional[dict] = None) -> dict:
    comp, name = load_comp_desc(cmodel_path)
    return extract_robot_spec(comp, name, load=load, overrides=overrides)


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="CModel → robot_config.json + robot.urdf")
    ap.add_argument("cmodel", help=".cmodel 文件 / CompDesc.json / 解码目录")
    ap.add_argument("out_dir", nargs="?", default=_HERE)
    ap.add_argument("--load", choices=["idle", "full"], default="full", help="使用空载或满载运动参数 (默认满载, 更保守)")
    ap.add_argument("--overrides", help="参数覆盖 JSON (传感器视场/量程等 cmodel 未包含的参数)")
    ap.add_argument("--no-nav2", action="store_true", help="不重新生成 Nav2 参数")
    a = ap.parse_args(argv)

    # 1) 纯 cmodel 解析 → robot_config.base.json (补全界面的基线)；2) 叠加人工补全 model_overrides.json
    from model_overrides import apply_overrides, load_overrides
    base = parse_cmodel_file(a.cmodel, load=a.load)
    base["model_path"] = os.path.abspath(a.cmodel)
    os.makedirs(a.out_dir, exist_ok=True)
    with open(os.path.join(a.out_dir, "robot_config.base.json"), "w", encoding="utf-8") as f:
        json.dump(base, f, indent=2, ensure_ascii=False)
    ov = load_overrides(a.overrides or os.path.join(a.out_dir, "model_overrides.json"))
    spec = apply_overrides(base, ov)
    if spec.get("provenance"):
        print(f"  (应用人工补全 {len(spec['provenance'])} 项: model_overrides.json)")
    os.makedirs(a.out_dir, exist_ok=True)
    spec_path = os.path.join(a.out_dir, "robot_config.json")
    urdf_path = os.path.join(a.out_dir, "robot.urdf")
    with open(spec_path, "w", encoding="utf-8") as f:
        json.dump(spec, f, indent=2, ensure_ascii=False)
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write(generate_urdf(spec))
    print(f"[cmodel] {spec['model_file']}  chassis={spec['chassis']['type']}  wheels={len(spec['wheels'])}  lidars={len(spec['lidars'])}")
    print(f"  - Config: {spec_path}\n  - URDF:   {urdf_path}")
    for s in spec["warnings"]:
        print("  ! " + s)
    for s in spec["assumptions"]:
        print("  ~ " + s)
    if not a.no_nav2:
        try:
            from tools.gen_nav2_params import write_all
            for p in write_all(spec, os.path.join(a.out_dir, "nav2")):
                print(f"  - Nav2:   {p}")
        except Exception as e:  # nav2 生成为可选
            print(f"  (Nav2 参数未生成: {e})")


if __name__ == "__main__":
    main()
