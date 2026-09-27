#!/usr/bin/env python3
"""
车辆模型仓库: .cmodel 上传即解析 (cmodel_parser) → 基线 base.json；人工补全 overrides.json；
实例启动时经 bundle 接口下载 (基线 + 补全)，与仿真进程内的补全机制完全一致。

模型 doc:   {id, name, project, vtype, material_no, note, latest, versions:[ver...], summary, created, updated}
版本目录:   models/<mid>/<ver>/{model.cmodel, base.json, overrides.json, meta.json}
"""

import json
import math
import os
import re
import shutil
import sys
import tempfile
import time
from typing import Optional

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from common.rest import ApiError  # noqa: E402
from hub.store import Store, slug  # noqa: E402

CHASSIS_LABEL = {"single_steer": "单舵轮", "dual_steer": "双舵轮", "diff_drive": "差速双驱", "quad_steer": "四舵轮",
                 "omni": "全向", "mecanum": "麦克纳姆轮"}


class ModelRepo:
    def __init__(self, store: Store):
        self.s = store

    # ------------------------------------------------------------------ 读取
    def vdir(self, mid: str, ver: str) -> str:
        return self.s.path("models", mid, ver)

    def get(self, mid: str) -> dict:
        m = self.s.get("models", mid)
        if not m:
            raise ApiError(404, f"模型 {mid} 不存在")
        return m

    def _ver(self, m: dict, ver: Optional[str]) -> str:
        v = ver or m.get("latest")
        if v in (None, "", "latest"):
            v = m.get("latest")
        if v not in m.get("versions", []):
            raise ApiError(404, f"模型 {m['id']} 没有版本 {v}")
        return v

    def base(self, mid: str, ver: Optional[str] = None) -> dict:
        m = self.get(mid)
        v = self._ver(m, ver)
        b = Store.read_json(os.path.join(self.vdir(mid, v), "base.json"))
        if b is None:
            raise ApiError(500, "模型基线缺失")
        return b

    def overrides(self, mid: str, ver: Optional[str] = None) -> dict:
        from model_overrides import load_overrides
        m = self.get(mid)
        v = self._ver(m, ver)
        return load_overrides(os.path.join(self.vdir(mid, v), "overrides.json"))

    def spec(self, mid: str, ver: Optional[str] = None) -> dict:
        from model_overrides import apply_overrides
        return apply_overrides(self.base(mid, ver), self.overrides(mid, ver))

    # ------------------------------------------------------------------ 概要 (卡片/向导用)
    @staticmethod
    def summarize(spec: dict) -> dict:
        ch = spec.get("chassis", {})
        wheels = spec.get("wheels", [])
        xs = [w.get("x", 0.0) for w in wheels]
        ys = [w.get("y", 0.0) for w in wheels]
        drive = [w for w in wheels if w.get("kind") in ("steer", "drive")]
        wb = (max(xs) - min(xs)) if len(xs) >= 2 else 0.0
        track = (max(ys) - min(ys)) if len(ys) >= 2 else 0.0
        lift = spec.get("lift") or {}
        cams = spec.get("cameras", [])
        return {"chassis": ch.get("type"), "chassis_label": CHASSIS_LABEL.get(ch.get("type"), ch.get("type")),
                "length_m": ch.get("length_m"), "width_m": ch.get("width_m"), "height_m": ch.get("height_m"),
                "mass_kg": ch.get("mass_kg"), "max_load_kg": ch.get("max_load_kg"), "max_speed_mps": ch.get("max_speed_mps"),
                "max_ang_speed_radps": ch.get("max_ang_speed_radps"), "wheelbase_m": round(wb, 3), "track_m": round(track, 3),
                "footprint": ch.get("footprint"), "rotate_diameter_m": ch.get("rotate_diameter_m"),
                "wheels": [{"name": w.get("name"), "kind": w.get("kind"), "x": w.get("x"), "y": w.get("y"),
                            "r": w.get("radius_m"), "w": w.get("width_m")} for w in wheels],
                "drive_wheels": len(drive), "lidars": [{"name": l.get("name"), "type": l.get("type"), "model": l.get("model"),
                                                          "x": l.get("x"), "y": l.get("y"), "z": l.get("z"), "yaw": l.get("yaw"),
                                                          "fov_deg": l.get("fov_deg"), "max_range": l.get("max_range")}
                                                         for l in spec.get("lidars", [])],
                "cameras": [{"name": c.get("name"), "type": c.get("type"), "x": c.get("x"), "y": c.get("y"), "z": c.get("z"),
                             "yaw": c.get("yaw"), "hfov_deg": c.get("hfov_deg")} for c in cams],
                "photoelectric": len(spec.get("photoelectric", [])),
                "lift": {"enabled": bool(lift.get("enabled")), "stroke_m": lift.get("stroke_m")} if lift else None,
                "model_file": spec.get("model_file"), "robot_name": spec.get("robot_name")}

    def view(self, m: dict) -> dict:
        d = dict(m)
        try:
            from model_overrides import audit
            v = m.get("latest")
            base = Store.read_json(os.path.join(self.vdir(m["id"], v), "base.json")) or {}
            ov = Store.read_json(os.path.join(self.vdir(m["id"], v), "overrides.json")) or {}
            from model_overrides import apply_overrides
            sp = apply_overrides(base, ov) if base else {}
            d["summary"] = self.summarize(sp) if sp else m.get("summary", {})
            a = audit(sp, ov) if sp else {}
            d["audit"] = a.get("summary", {}) if a else {}
        except Exception as e:
            d["audit"] = {"error": str(e)}
        return d

    def list(self) -> list:
        return [self.view(m) for m in sorted(self.s.list("models"), key=lambda m: m.get("created", 0))]

    # ------------------------------------------------------------------ 上传
    def add_cmodel(self, filename: str, data: bytes, meta: dict) -> dict:
        from cmodel_parser import parse_cmodel_file
        if not filename.lower().endswith(".cmodel"):
            raise ApiError(400, "仅支持 .cmodel 模型文件")
        tmpd = tempfile.mkdtemp(dir=self.s.path("tmp"))
        try:
            tmp = os.path.join(tmpd, filename)
            with open(tmp, "wb") as f:
                f.write(data)
            try:
                base = parse_cmodel_file(tmp, load=meta.get("load", "full"))
            except Exception as e:
                raise ApiError(400, f"cmodel 解析失败: {e}")
            base["model_file"] = filename
            name = (meta.get("name") or "").strip() or re.sub(r"\.cmodel$", "", filename, flags=re.I)
            mid = meta.get("model_id") or self._find_by_name(name) or self._new_mid(name)
            m = self.s.get("models", mid) or {"id": mid, "name": name, "versions": [], "created": time.time()}
            ver = (meta.get("version") or "").strip() or _next_ver(m.get("versions", []))
            if ver in m["versions"]:
                raise ApiError(409, f"版本 {ver} 已存在")
            vd = self.vdir(mid, ver)
            os.makedirs(vd, exist_ok=True)
            shutil.copy(tmp, os.path.join(vd, "model.cmodel"))
            self.s.write_json(os.path.join(vd, "base.json"), base)
            # 继承上一版本的人工补全 (参数名相同的传感器/轮组会继续生效)
            prev = m.get("latest")
            ov = Store.read_json(os.path.join(self.vdir(mid, prev), "overrides.json")) if prev else None
            self.s.write_json(os.path.join(vd, "overrides.json"), ov or {"version": 1})
            self.s.write_json(os.path.join(vd, "meta.json"), {"file": filename, "uploaded": time.time(), "size": len(data),
                                                              "operator": meta.get("operator", "")})
            m["versions"] = m.get("versions", []) + [ver]
            m["latest"] = ver
            for k in ("project", "vtype", "material_no", "note"):
                if meta.get(k) is not None:
                    m[k] = meta[k]
            m.setdefault("vtype", _guess_vtype(base))
            m["file"] = filename
            m["summary"] = self.summarize(base)
            self.s.put("models", m)
            return self.view(m)
        finally:
            shutil.rmtree(tmpd, ignore_errors=True)

    def _find_by_name(self, name: str) -> Optional[str]:
        for m in self.s.list("models"):
            if m.get("name") == name:
                return m["id"]
        return None

    def _new_mid(self, name: str) -> str:
        base = "m-" + slug(name, "model")
        mid, i = base, 2
        while self.s.get("models", mid):
            mid, i = f"{base}-{i}", i + 1
        return mid

    def update_meta(self, mid: str, fields: dict) -> dict:
        m = self.get(mid)
        for k in ("name", "project", "vtype", "material_no", "note"):
            if k in fields:
                m[k] = fields[k]
        self.s.put("models", m)
        return self.view(m)

    def delete(self, mid: str):
        self.get(mid)
        shutil.rmtree(self.s.path("models", mid), ignore_errors=True)
        self.s.delete("models", mid)

    def cmodel_file(self, mid: str, ver: Optional[str]) -> str:
        m = self.get(mid)
        return os.path.join(self.vdir(mid, self._ver(m, ver)), "model.cmodel")

    # ------------------------------------------------------------------ 补全编辑 (与仿真进程 /api/v1/model/* 同构)
    def editor(self, mid: str, ver: Optional[str]) -> dict:
        from model_overrides import SENSOR_TYPES, audit, sensor_list
        sp, ov = self.spec(mid, ver), self.overrides(mid, ver)
        return {"spec": sp, "base_file": "base.json", "overrides": ov, "audit": audit(sp, ov), "sensors": sensor_list(sp),
                "sensor_types": SENSOR_TYPES, "repo": {"model_id": mid, "version": self._ver(self.get(mid), ver)}}

    def preview(self, mid: str, ver: Optional[str], ov: dict) -> dict:
        from cmodel_parser import generate_urdf
        from model_overrides import apply_overrides, audit, sensor_list
        sp = apply_overrides(self.base(mid, ver), ov)
        return {"spec": sp, "audit": audit(sp, ov), "sensors": sensor_list(sp), "urdf": generate_urdf(sp)}

    def save_overrides(self, mid: str, ver: Optional[str], ov: dict) -> dict:
        from model_overrides import save_overrides
        m = self.get(mid)
        v = self._ver(m, ver)
        ov = dict(ov)
        ov.setdefault("version", 1)
        save_overrides(ov, os.path.join(self.vdir(mid, v), "overrides.json"))
        m["summary"] = self.summarize(self.spec(mid, v))
        self.s.put("models", m)
        return self.editor(mid, v)

    def urdf(self, mid: str, ver: Optional[str]) -> str:
        from cmodel_parser import generate_urdf
        return generate_urdf(self.spec(mid, ver))

    def bundle(self, mid: str, ver: Optional[str]) -> dict:
        """实例启动时下载: 基线 + 补全 (仿真进程据此生成 robot_config.json / robot.urdf)"""
        m = self.get(mid)
        v = self._ver(m, ver)
        return {"model_id": mid, "version": v, "name": m.get("name"), "file": m.get("file"), "material_no": m.get("material_no"),
                "vtype": m.get("vtype"), "project": m.get("project"),
                "base": self.base(mid, v), "overrides": self.overrides(mid, v)}


def _next_ver(vers) -> str:
    n = 1
    for v in vers:
        mt = re.match(r"v(\d+)$", v)
        if mt:
            n = max(n, int(mt.group(1)) + 1)
    return f"v{n}"


def _guess_vtype(spec: dict) -> str:
    ch = spec.get("chassis", {})
    t = CHASSIS_LABEL.get(ch.get("type"), ch.get("type") or "AMR")
    if (spec.get("lift") or {}).get("enabled"):
        return f"{t}顶升 AMR"
    return f"{t} AMR"
