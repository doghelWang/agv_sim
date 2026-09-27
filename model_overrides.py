#!/usr/bin/env python3
"""
模型补全 (人工覆盖) —— cmodel 解析结果 + model_overrides.json → 最终机器人模型 (robot_config.json / robot.urdf)

model_overrides.json (v1):
{
  "version": 1,
  "chassis":  {"mass_kg": 320, "com": [0.4, 0, 0.35], "height_m": 2.03, ...},
  "wheels":   {"Steerwheel": {"radius_m": 0.115, "mass_kg": 8, "friction": 0.9}, "rear_left_load_wheel": {...}},
  "motors":   {"walk-motor": {"rated_torque_nm": 5.1, "rated_current_a": 60}},
  "lift":     {"enabled": true, "stroke_m": 0.08, "speed_mps": 0.02, "x": 0.4, "y": 0, "z": 0.3, "size": [1.2, 0.8, 0.04]},
  "imu":      {"x": 0, "y": 0, "z": 0.1},
  "sensors":  {
      "laser":     {"library": "mid-360s"},                               # 修改 cmodel 已有传感器
      "front_cam": {"type": "camera", "x": 1.3, "z": 0.9, "pitch": 0.15, "_added": true},   # 人工安装的新传感器
      "laser0":    {"_removed": true}                                       # 删除
  }
}
所有人工修改的字段在 spec["provenance"] 中记为 "manual"；audit() 给出每项参数的来源与缺失情况。
"""

import copy
import json
import math
import os
from typing import Dict, List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
OVERRIDES_FILE = os.path.join(HERE, "model_overrides.json")
LEGACY_FILE = os.path.join(HERE, "sensor_overrides.json")

SENSOR_TYPES = {
    "lidar2d": "2D 激光雷达", "lidar3d": "3D 激光雷达", "camera": "单目 RGB 相机", "stereo": "双目深度相机",
    "tof": "ToF 深度相机", "photoelectric": "光电传感器", "codeReader": "读码相机",
}
POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


# ============================================================================ 读写
def load_overrides(path: str = OVERRIDES_FILE) -> dict:
    """优先 model_overrides.json；不存在时从旧版 sensor_overrides.json 迁移"""
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            ov = json.load(f)
    elif os.path.exists(os.path.join(os.path.dirname(os.path.abspath(path)), "sensor_overrides.json")) or os.path.exists(LEGACY_FILE):
        legacy = os.path.join(os.path.dirname(os.path.abspath(path)), "sensor_overrides.json")
        with open(legacy if os.path.exists(legacy) else LEGACY_FILE, "r", encoding="utf-8") as f:
            old = json.load(f)
        ov = {"version": 1, "sensors": {k: dict(v) for k, v in (old.get("lidars") or {}).items()},
              "_migrated_from": "sensor_overrides.json"}
        for sec in ("chassis", "wheels"):
            if old.get(sec):
                ov[sec] = old[sec]
    else:
        ov = {"version": 1}
    ov.setdefault("version", 1)
    for k in ("chassis", "wheels", "motors", "sensors", "lift", "imu"):
        ov.setdefault(k, {})
    return ov


def save_overrides(ov: dict, path: str = OVERRIDES_FILE):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(ov, f, indent=2, ensure_ascii=False)
    os.replace(tmp, path)


# ============================================================================ 传感器默认参数 (新增时)
def sensor_template(stype: str) -> dict:
    from cmodel_parser import LIDAR_DEFAULT, LIDAR_LIBRARY
    from sim_core.cameras import CAMERA_LIBRARY
    pose = {"x": 0.5, "y": 0.0, "z": 0.5, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    if stype == "lidar2d":
        d = dict(LIDAR_DEFAULT)
        d.update({"type": "2d", "model": "通用 2D 安全激光", "z": 0.25})
    elif stype == "lidar3d":
        base = next((p for kw, p in LIDAR_LIBRARY if kw == "mid-360s"), {})
        d = dict(base)
        d.update({"type": "3d", "z": 1.8})
    elif stype in CAMERA_LIBRARY:
        d = dict(CAMERA_LIBRARY[stype])
        d["type"] = stype
    elif stype == "photoelectric":
        d = {"type": "photoelectric", "model": "漫反射光电", "trigger_m": 0.35, "range_m": 1.0, "z": 0.12, "beam_half_deg": 2.0}
    elif stype == "codeReader":
        d = {"type": "codeReader", "model": "下视读码相机", "orientation": "LENS_DIR_DOWN", "read_window_m": 0.05, "z": 0.1}
    else:
        raise ValueError(f"未知传感器类型 {stype}")
    out = dict(pose)
    out.update(d)
    return out


def _stype_of(item: dict, sec: str) -> str:
    if sec == "lidars":
        return "lidar3d" if item.get("type") == "3d" else "lidar2d"
    if sec == "photoelectric":
        return "photoelectric"
    t = item.get("type", "camera")
    return {"depthCamera": "tof"}.get(t, t)


# ============================================================================ 应用覆盖
def apply_overrides(spec: dict, ov: dict) -> dict:
    """返回新 spec (不修改入参)。cmodel 解析结果 → 叠加人工补全"""
    from cmodel_parser import LIDAR_LIBRARY
    sp = copy.deepcopy(spec)
    prov: Dict[str, str] = {}
    ch = sp["chassis"]
    for k, v in (ov.get("chassis") or {}).items():
        ch[k] = v
        prov[f"chassis.{k}"] = "manual"
    if any(k in (ov.get("chassis") or {}) for k in ("head_offset_m", "tail_offset_m", "left_offset_m", "right_offset_m")):
        h, t, l, r = ch["head_offset_m"], ch["tail_offset_m"], ch["left_offset_m"], ch["right_offset_m"]
        ch["footprint"] = [[h, l], [h, -r], [-t, -r], [-t, l]]
        ch["length_m"], ch["width_m"] = round(h + t, 4), round(l + r, 4)
    for name, patch in (ov.get("wheels") or {}).items():
        for w in sp.get("wheels", []):
            if w.get("name") == name or w.get("cmodel_name") == name:
                w.update(patch)
                for k in patch:
                    prov[f"wheels.{w['name']}.{k}"] = "manual"
    for name, patch in (ov.get("motors") or {}).items():
        for w in sp.get("wheels", []):
            for mk in ("drive_motor", "steer_motor"):
                m = w.get(mk)
                if m and m.get("name") == name:
                    m.update(patch)
                    for k in patch:
                        prov[f"motors.{name}.{k}"] = "manual"
        for m in sp.get("other_actuators", []):
            if m.get("name") == name:
                m.update(patch)
                for k in patch:
                    prov[f"motors.{name}.{k}"] = "manual"
    if ov.get("lift"):
        sp["lift"] = dict(ov["lift"])
        prov["lift"] = "manual"
    if ov.get("protection"):
        sp["protection"] = copy.deepcopy(ov["protection"])     # 保护空间 (planning/protection.py 补齐默认值)
        prov["protection"] = "manual"
    if ov.get("imu"):
        sp.setdefault("imu", {}).update(ov["imu"])
        for k in ov["imu"]:
            prov[f"imu.{k}"] = "manual"

    # ---- 传感器
    sp.setdefault("lidars", []); sp.setdefault("cameras", []); sp.setdefault("photoelectric", [])
    secs = ("lidars", "cameras", "photoelectric")
    for name, patch in (ov.get("sensors") or {}).items():
        patch = dict(patch)
        if patch.get("_removed"):
            for sec in secs:
                sp[sec] = [s for s in sp[sec] if s.get("name") != name and s.get("cmodel_name") != name]
            prov[f"sensors.{name}"] = "removed"
            continue
        lib = str(patch.pop("library", "") or "").lower()
        if lib:
            base = next((p for kw, p in LIDAR_LIBRARY if kw == lib), None)
            if base:
                full = dict(base, spec_source=f"override:{lib}")
                full.update(patch)
                patch = full
        found = None
        for sec in secs:
            for s in sp[sec]:
                if s.get("name") == name or s.get("cmodel_name") == name:
                    found = (sec, s)
        added = patch.pop("_added", False)
        if found:
            sec, s = found
            s.update({k: v for k, v in patch.items() if not k.startswith("_")})
        else:
            stype = patch.get("stype") or {"2d": "lidar2d", "3d": "lidar3d"}.get(patch.get("type"), patch.get("type", "camera"))
            s = sensor_template(stype)
            s.update({k: v for k, v in patch.items() if not k.startswith("_")})
            s["name"] = name
            s["source"] = "manual"
            sec = "lidars" if stype in ("lidar2d", "lidar3d") else ("photoelectric" if stype == "photoelectric" else "cameras")
            if sec == "lidars":
                s["type"] = "3d" if stype == "lidar3d" else "2d"
            sp[sec].append(s)
            added = True
        s.pop("stype", None)
        if sec == "lidars":
            fov = float(s.get("fov_deg", 270.0))
            s["angle_min"] = round(-math.radians(fov) / 2.0, 6)
            s["angle_max"] = round(math.radians(fov) / 2.0, 6)
            s["inverted"] = abs(math.cos(float(s.get("roll", 0.0)))) > 0.5 and math.cos(float(s.get("roll", 0.0))) < 0
            if s.get("housing_box"):
                s["housing"] = {"box": s["housing_box"]}
            s.setdefault("resolution_deg", 0.25)
        prov[f"sensors.{name}"] = "added" if added else "manual"
    sp["provenance"] = prov
    try:
        from planning import protection as _pr
        sp["protection_effective"] = _pr.effective(sp)          # 补齐默认值后的保护空间 (编辑器/界面显示用)
    except Exception:
        pass
    if prov:
        sp.setdefault("assumptions", [])
        msg = f"已应用人工补全 {len(prov)} 项 (model_overrides.json)"
        sp["assumptions"] = [a for a in sp["assumptions"] if "overrides" not in a] + [msg]
    return sp


# ============================================================================ 统一传感器清单 (编辑器)
def sensor_list(spec: dict, photos_effective: Optional[List[dict]] = None) -> List[dict]:
    prov = spec.get("provenance", {})
    out = []
    for sec in ("lidars", "cameras", "photoelectric"):
        for s in spec.get(sec, []):
            st = _stype_of(s, sec)
            src = prov.get(f"sensors.{s['name']}")
            if src is None:
                src = "cmodel" if s.get("cmodel_name") else s.get("source", "default")
            params = {k: v for k, v in s.items() if k not in POSE_KEYS and k not in ("name", "cmodel_name", "angle_min", "angle_max")}
            out.append({"name": s["name"], "stype": st, "type_label": SENSOR_TYPES.get(st, st), "section": sec,
                        "model": s.get("vendor_model") or s.get("model") or "", "source": src,
                        "spec_source": s.get("spec_source"), "pose": {k: float(s.get(k, 0.0) or 0.0) for k in POSE_KEYS},
                        "params": params})
    if photos_effective and not spec.get("photoelectric"):
        for p in photos_effective:     # 缺省光电 (仿真引擎按车身四角生成)
            m = p["mount"]
            out.append({"name": p["name"], "stype": "photoelectric", "type_label": SENSOR_TYPES["photoelectric"], "section": "photoelectric",
                        "model": "漫反射光电", "source": "default", "pose": {"x": m["x"], "y": m["y"], "z": m["z"], "roll": 0.0, "pitch": 0.0, "yaw": m["yaw"]},
                        "params": {"trigger_m": p["trigger_m"], "range_m": p["range_m"]}})
    return out


# ============================================================================ 完整度审计
def audit(spec: dict, ov: Optional[dict] = None) -> dict:
    """逐项列出 URDF / 仿真所需参数的来源 (cmodel / library / inferred / default / manual) 与缺失情况"""
    ov = ov or {}
    prov = spec.get("provenance", {})
    ch = spec["chassis"]
    items: List[dict] = []

    def add(group, key, label, value, source, severity, note="", unit="", editable=True, path=None):
        if (path or key) in prov or any(p.startswith((path or key) + ".") for p in prov):
            source, severity = "manual", "ok"
        items.append({"group": group, "key": key, "path": path or key, "label": label, "value": value, "unit": unit,
                      "source": source, "severity": severity, "note": note, "editable": editable})

    assum = " ".join(spec.get("assumptions", []))
    # ---- 车体
    add("车体", "chassis.footprint", "外形轮廓 (头/尾/左/右偏移)",
        [ch["head_offset_m"], ch["tail_offset_m"], ch["left_offset_m"], ch["right_offset_m"]], "cmodel", "ok", "", "m", False)
    add("车体", "chassis.height_m", "车体高度", ch.get("height_m"), "cmodel", "ok", "", "m")
    mass_default = float(ch.get("self_weight_kg") or 0) <= 0
    add("车体", "chassis.mass_kg", "整车质量", ch.get("mass_kg"), "default" if mass_default else "cmodel",
        "warn" if mass_default else "ok", "cmodel 自重缺失，按投影面积估算" if mass_default else "", "kg")
    add("车体", "chassis.com", "质心位置", ch.get("com") or "几何中心", "default", "info", "cmodel 不含质心，URDF 惯量按均质长方体", "m")
    add("车体", "chassis.max_load_kg", "额定载荷", ch.get("max_load_kg"), "cmodel" if ch.get("max_load_kg") else "default",
        "ok" if ch.get("max_load_kg") else "info", "" if ch.get("max_load_kg") else "cmodel 为 0", "kg")
    add("车体", "chassis.visual", "车体外观网格", "包围盒 (底盘 + 上装)", "default", "info",
        "cmodel 仅引用模块库 3D 资源路径，未内嵌网格文件；URDF 用包围盒", "", False)
    # ---- 轮组
    for w in spec.get("wheels", []):
        inferred = w.get("source") == "inferred"
        nm = w["name"]
        add("轮组", f"wheels.{nm}.pose", f"{nm} 安装位置", [w.get("x"), w.get("y")], "inferred" if inferred else "cmodel",
            "warn" if inferred else "ok", "cmodel 未建模被动轮，按底盘类型推断" if inferred else "", "m",
            path=f"wheels.{nm}")
        add("轮组", f"wheels.{nm}.radius_m", f"{nm} 轮半径", w.get("radius_m"), "inferred" if inferred else "cmodel",
            "warn" if inferred else "ok", "", "m", path=f"wheels.{nm}.radius_m")
        add("轮组", f"wheels.{nm}.width_m", f"{nm} 轮宽", w.get("width_m"), "default", "info", "cmodel 不含轮宽", "m",
            path=f"wheels.{nm}.width_m")
        add("轮组", f"wheels.{nm}.mass_kg", f"{nm} 轮质量", w.get("mass_kg", 5.0), "default", "info", "URDF 惯量用", "kg",
            path=f"wheels.{nm}.mass_kg")
        add("轮组", f"wheels.{nm}.friction", f"{nm} 轮地摩擦系数", w.get("friction", 1.2), "default", "info", "聚氨酯轮 0.8~1.2", "",
            path=f"wheels.{nm}.friction")
        for mk in ("drive_motor", "steer_motor"):
            m = w.get(mk)
            if m:
                ok = bool(m.get("rated_torque_nm"))
                add("轮组", f"motors.{m['name']}.rated_torque_nm", f"{m['name']} 额定转矩", m.get("rated_torque_nm"),
                    "cmodel" if ok else "default", "ok" if ok else "warn", "" if ok else "cmodel torque 为空，电流/力矩仿真用经验值", "N·m",
                    path=f"motors.{m['name']}.rated_torque_nm")
    # ---- 顶升
    lift_motor = next((m for m in spec.get("other_actuators", []) if "lift" in m.get("name", "").lower()), None)
    lift = spec.get("lift")
    if lift_motor or lift:
        add("顶升机构", "lift", "顶升/货叉机构几何与行程", lift or None, "manual" if lift else "missing",
            "ok" if lift else "missing", f"cmodel 有电机 {lift_motor['name']} 但无机构几何/行程" if lift_motor and not lift else "", "",
            path="lift")
    # ---- 传感器
    for l in spec.get("lidars", []):
        src = l.get("spec_source", "")
        sev = "ok" if src.startswith(("override", "library:")) and "default" not in src else "warn"
        add("传感器", f"sensors.{l['name']}", f"激光 {l['name']} 型号参数 (视场/量程/分辨率)",
            l.get("vendor_model") or l.get("model") or "未知型号", "library" if "library" in src else ("manual" if "override" in src else "default"),
            sev, "" if sev == "ok" else "cmodel 只有安装位姿，未识别型号 → 通用参数", "", path=f"sensors.{l['name']}")
    cams = [c for c in spec.get("cameras", [])]
    if not cams:
        add("传感器", "sensors.camera", "相机 (单目/双目/ToF)", None, "missing", "info", "cmodel 未定义相机，可在传感器安装页添加", "", path="sensors.__camera")
    for c in cams:
        needs = c.get("type") in ("camera", "stereo", "tof", "depthCamera") and not c.get("width")
        add("传感器", f"sensors.{c['name']}", f"相机 {c['name']} 内参 (分辨率/视场/帧率)", c.get("model") or c.get("type"),
            "default" if needs else ("manual" if c.get("source") == "manual" else "cmodel"), "warn" if needs else "ok",
            "cmodel 不含相机内参 → 型号库默认值" if needs else "", "", path=f"sensors.{c['name']}")
    imu = spec.get("imu") or {}
    zero = abs(imu.get("x", 0)) + abs(imu.get("y", 0)) + abs(imu.get("z", 0)) < 1e-6
    add("传感器", "imu", "IMU 安装位置", [imu.get("x"), imu.get("y"), imu.get("z")], "cmodel", "info" if zero else "ok",
        "cmodel 位姿为 0 (多为主控内置陀螺)" if zero else "", "m", path="imu")
    if not spec.get("photoelectric"):
        add("传感器", "sensors.photoelectric", "光电传感器", "四角缺省 4 个", "default", "info", "cmodel 无光电模块，仿真按车身四角生成", "",
            path="sensors.__photo")
    add("传感器", "sensors.bumpers", "防撞触边", "前/后缺省", "default", "info", "cmodel 无触边模块，按车身前后边缘生成", "", False)
    # ---- 保护空间 (planning/protection.py)
    try:
        from planning import protection as _pr
        P = _pr.effective(spec)
        src = "manual" if "protection" in prov else "default"
        items.append({"group": "保护空间", "key": "protection.mode", "path": "protection", "label": "过弯方式 / 车体净空",
                      "value": f"{P['corner_mode']} · {P['body_margin']} m", "unit": "", "source": src, "severity": "ok",
                      "note": "auto=能原地转向就停车转向，转不开用圆弧；防护区" + ("按车型动力学自动生成" if P.get("fields_auto") else "为人工配置"),
                      "editable": True})
        for it in _pr.validate(spec, P):
            items.append({"group": "保护空间", "key": "protection." + it["key"], "path": "protection", "label": it["label"],
                          "value": it["value"], "unit": "m" if isinstance(it["value"], (int, float)) else "", "source": src,
                          "severity": it["severity"], "note": it["note"], "editable": True})
    except Exception as e:  # pragma: no cover
        items.append({"group": "保护空间", "key": "protection", "path": "protection", "label": "保护空间", "value": None, "unit": "",
                      "source": "default", "severity": "warn", "note": f"校核失败: {e}", "editable": True})

    # ---- URDF 结构
    add("URDF", "urdf.optical", "相机光学坐标系 (_optical_frame)", "自动生成", "default", "ok", "", "", False)
    add("URDF", "urdf.ros2_control", "ros2_control / transmission", None, "missing", "info",
        "仿真与执行解耦后由 REST 驱动，无需 ros2_control；上真车需补", "", False)
    add("URDF", "urdf.sensor_inertial", "传感器质量/惯量", "0.05 kg 占位", "default", "info", "", "", False)

    cnt = {}
    for it in items:
        cnt[it["severity"]] = cnt.get(it["severity"], 0) + 1
    manual = len([i for i in items if i["source"] == "manual"])
    return {"items": items, "summary": {"total": len(items), "by_severity": cnt, "manual": manual},
            "sensor_types": SENSOR_TYPES}
