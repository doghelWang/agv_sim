//! Manual model completion (port of model_overrides.py)
use crate::protection;
use crate::{fnum, pstr, rnd};
use serde_json::{json, Map, Value};

pub const POSE_KEYS: [&str; 6] = ["x", "y", "z", "roll", "pitch", "yaw"];

pub fn truthy(v: &Value) -> bool {
    match v { Value::Bool(b) => *b, Value::Null => false, Value::Number(n) => n.as_f64().map(|x| x != 0.0).unwrap_or(false), Value::String(s) => !s.is_empty(), Value::Array(a) => !a.is_empty(), Value::Object(o) => !o.is_empty() }
}

pub fn sensor_types() -> Value {
    json!({"lidar2d": "2D 激光雷达", "lidar3d": "3D 激光雷达", "camera": "单目 RGB 相机", "stereo": "双目深度相机", "tof": "ToF 深度相机", "photoelectric": "光电传感器", "codeReader": "读码相机"})
}

// ---- parameter libraries
pub fn mid360s() -> Value {
    json!({"type": "3d", "vendor_model": "Livox Mid-360S", "fov_deg": 360.0, "vfov_min_deg": -7.0, "vfov_max_deg": 52.0, "min_range": 0.1, "max_range": 40.0, "range_cutoff_m": 100.0,
        "point_rate": 200000, "freq_hz": 10.0, "range_noise_std": 0.02, "angular_noise_deg": 0.15, "lines": 4, "scan_pattern": "non_repetitive", "imu": "ICM40609",
        "resolution_deg": 0.5, "housing_box": [0.065, 0.065, 0.060], "topic": "/livox/lidar", "imu_topic": "/livox/imu"})
}
pub fn lidar_library(kw: &str) -> Option<Value> {
    let p2 = |fov: f64, mn: f64, mx: f64, res: f64, hz: f64, noise: f64, name: &str| json!({"fov_deg": fov, "min_range": mn, "max_range": mx, "resolution_deg": res, "freq_hz": hz, "range_noise_std": noise, "vendor_model": name});
    match kw {
        "mid-360s" | "mid360s" => Some(mid360s()),
        "mid-360" | "mid360" => { let mut m = mid360s(); m["vendor_model"] = json!("Livox Mid-360"); Some(m) }
        "pepperl" | "r2000" => Some(p2(360.0, 0.1, 30.0, 0.25, 20.0, 0.012, "Pepperl+Fuchs R2000")),
        "mr-ls-07h" => Some(p2(270.0, 0.05, 20.0, 0.33, 25.0, 0.02, "MR-LS-07H 安全激光")),
        "ls-05h" => Some(p2(270.0, 0.05, 15.0, 0.33, 25.0, 0.02, "MR-LS-05H 激光")),
        "sick" => Some(p2(270.0, 0.05, 25.0, 0.25, 25.0, 0.015, "SICK")),
        "nanoscan" => Some(p2(275.0, 0.05, 30.0, 0.17, 25.0, 0.015, "SICK nanoScan3")),
        _ => None,
    }
}
pub fn lidar_default() -> Value { json!({"fov_deg": 270.0, "min_range": 0.05, "max_range": 25.0, "resolution_deg": 0.25, "freq_hz": 15.0, "range_noise_std": 0.015, "vendor_model": "generic 2D"}) }
pub fn camera_library(t: &str) -> Option<Value> {
    match t {
        "camera" => Some(json!({"model": "通用 USB RGB 相机", "width": 320, "height": 240, "hfov_deg": 70.0, "fps": 10.0, "range_max": 30.0, "pixel_noise": 2.0})),
        "stereo" => Some(json!({"model": "Intel RealSense D435 (仿)", "width": 424, "height": 240, "hfov_deg": 87.0, "fps": 10.0, "baseline_m": 0.05, "range_min": 0.3, "range_max": 10.0, "subpixel_px": 0.08, "pixel_noise": 2.0})),
        "tof" => Some(json!({"model": "PMD/Sunny 224×172 ToF (仿)", "width": 224, "height": 172, "hfov_deg": 62.0, "fps": 10.0, "range_min": 0.1, "range_max": 4.0, "noise_abs_m": 0.005, "noise_rel": 0.004})),
        _ => None,
    }
}

fn upd(dst: &mut Value, src: &Value) { if let (Some(d), Some(s)) = (dst.as_object_mut(), src.as_object()) { for (k, v) in s { d.insert(k.clone(), v.clone()); } } }

// ---- read / write
pub fn load_overrides(path: &str) -> Result<Value, String> {
    let dir = std::path::Path::new(path).parent().map(|p| p.to_path_buf()).unwrap_or_default();
    let legacy = dir.join("sensor_overrides.json");
    let mut ov = if std::path::Path::new(path).exists() {
        serde_json::from_str::<Value>(&std::fs::read_to_string(path).map_err(|e| e.to_string())?).map_err(|e| format!("{path}: {e}"))?
    } else if legacy.exists() {
        let old: Value = serde_json::from_str(&std::fs::read_to_string(&legacy).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
        let mut o = json!({"version": 1, "sensors": old.get("lidars").cloned().unwrap_or(json!({})), "_migrated_from": "sensor_overrides.json"});
        for sec in ["chassis", "wheels"] { if let Some(v) = old.get(sec).filter(|v| truthy(v)) { o[sec] = v.clone(); } }
        o
    } else { json!({"version": 1}) };
    if ov.get("version").is_none() { ov["version"] = json!(1); }
    for k in ["chassis", "wheels", "motors", "sensors", "lift", "imu"] { if ov.get(k).is_none() { ov[k] = json!({}); } }
    Ok(ov)
}
pub fn save_overrides(ov: &Value, path: &str) -> Result<(), String> {
    let tmp = format!("{path}.tmp");
    std::fs::write(&tmp, serde_json::to_string_pretty(ov).unwrap()).map_err(|e| e.to_string())?;
    std::fs::rename(&tmp, path).map_err(|e| e.to_string())
}

pub fn sensor_template(stype: &str) -> Result<Value, String> {
    let mut d = match stype {
        "lidar2d" => { let mut d = lidar_default(); upd(&mut d, &json!({"type": "2d", "model": "通用 2D 安全激光", "z": 0.25})); d }
        "lidar3d" => { let mut d = mid360s(); upd(&mut d, &json!({"type": "3d", "z": 1.8})); d }
        "camera" | "stereo" | "tof" => { let mut d = camera_library(stype).unwrap(); d["type"] = json!(stype); d }
        "photoelectric" => json!({"type": "photoelectric", "model": "漫反射光电", "trigger_m": 0.35, "range_m": 1.0, "z": 0.12, "beam_half_deg": 2.0}),
        "codeReader" => json!({"type": "codeReader", "model": "下视读码相机", "orientation": "LENS_DIR_DOWN", "read_window_m": 0.05, "z": 0.1}),
        _ => return Err(format!("未知传感器类型 {stype}")),
    };
    let mut out = json!({"x": 0.5, "y": 0.0, "z": 0.5, "roll": 0.0, "pitch": 0.0, "yaw": 0.0});
    upd(&mut out, &d);
    d = out;
    Ok(d)
}

fn stype_of(item: &Value, sec: &str) -> String {
    match sec {
        "lidars" => if item.get("type").and_then(|t| t.as_str()) == Some("3d") { "lidar3d".into() } else { "lidar2d".into() },
        "photoelectric" => "photoelectric".into(),
        _ => { let t = item.get("type").and_then(|t| t.as_str()).unwrap_or("camera"); if t == "depthCamera" { "tof".into() } else { t.into() } }
    }
}

fn name_match(s: &Value, name: &str) -> bool { s.get("name").and_then(|n| n.as_str()) == Some(name) || s.get("cmodel_name").and_then(|n| n.as_str()) == Some(name) }

pub fn apply_overrides(spec: &Value, ov: &Value) -> Value {
    let mut sp = spec.clone();
    let mut prov = Map::new();
    let secs = ["lidars", "cameras", "photoelectric"];
    let ovc = ov.get("chassis").and_then(|c| c.as_object()).cloned().unwrap_or_default();
    for (k, v) in &ovc { sp["chassis"][k] = v.clone(); prov.insert(format!("chassis.{k}"), json!("manual")); }
    if ["head_offset_m", "tail_offset_m", "left_offset_m", "right_offset_m"].iter().any(|k| ovc.contains_key(*k)) {
        let ch = &mut sp["chassis"];
        let (h, t, l, r) = (fnum(ch.get("head_offset_m"), 0.0), fnum(ch.get("tail_offset_m"), 0.0), fnum(ch.get("left_offset_m"), 0.0), fnum(ch.get("right_offset_m"), 0.0));
        ch["footprint"] = json!([[h, l], [h, -r], [-t, -r], [-t, l]]);
        ch["length_m"] = json!(rnd(h + t, 4));
        ch["width_m"] = json!(rnd(l + r, 4));
    }
    if let Some(ws) = ov.get("wheels").and_then(|w| w.as_object()) {
        for (name, patch) in ws {
            if let Some(arr) = sp.get_mut("wheels").and_then(|w| w.as_array_mut()) {
                for w in arr.iter_mut() {
                    if name_match(w, name) {
                        let wn = w["name"].as_str().unwrap_or("").to_string();
                        upd(w, patch);
                        if let Some(p) = patch.as_object() { for k in p.keys() { prov.insert(format!("wheels.{wn}.{k}"), json!("manual")); } }
                    }
                }
            }
        }
    }
    if let Some(ms) = ov.get("motors").and_then(|m| m.as_object()) {
        for (name, patch) in ms {
            if let Some(arr) = sp.get_mut("wheels").and_then(|w| w.as_array_mut()) {
                for w in arr.iter_mut() {
                    for mk in ["drive_motor", "steer_motor"] {
                        if let Some(m) = w.get_mut(mk) {
                            if m.get("name").and_then(|n| n.as_str()) == Some(name) {
                                upd(m, patch);
                                if let Some(p) = patch.as_object() { for k in p.keys() { prov.insert(format!("motors.{name}.{k}"), json!("manual")); } }
                            }
                        }
                    }
                }
            }
            if let Some(arr) = sp.get_mut("other_actuators").and_then(|w| w.as_array_mut()) {
                for m in arr.iter_mut() {
                    if m.get("name").and_then(|n| n.as_str()) == Some(name) {
                        upd(m, patch);
                        if let Some(p) = patch.as_object() { for k in p.keys() { prov.insert(format!("motors.{name}.{k}"), json!("manual")); } }
                    }
                }
            }
        }
    }
    if ov.get("lift").map(truthy).unwrap_or(false) { sp["lift"] = ov["lift"].clone(); prov.insert("lift".into(), json!("manual")); }
    if ov.get("protection").map(truthy).unwrap_or(false) { sp["protection"] = ov["protection"].clone(); prov.insert("protection".into(), json!("manual")); }
    if ov.get("imu").map(truthy).unwrap_or(false) {
        if sp.get("imu").map(|i| !i.is_object()).unwrap_or(true) { sp["imu"] = json!({}); }
        let patch = ov["imu"].clone();
        upd(&mut sp["imu"], &patch);
        if let Some(p) = patch.as_object() { for k in p.keys() { prov.insert(format!("imu.{k}"), json!("manual")); } }
    }
    for s in secs { if sp.get(s).is_none() { sp[s] = json!([]); } }
    if let Some(sensors) = ov.get("sensors").and_then(|s| s.as_object()) {
        for (name, patch0) in sensors {
            let mut patch = patch0.as_object().cloned().unwrap_or_default();
            if patch.get("_removed").map(truthy).unwrap_or(false) {
                for sec in secs { if let Some(a) = sp[sec].as_array_mut() { a.retain(|s| !name_match(s, name)); } }
                prov.insert(format!("sensors.{name}"), json!("removed"));
                continue;
            }
            let lib = patch.remove("library").map(|v| v.as_str().unwrap_or("").to_lowercase()).unwrap_or_default();
            if !lib.is_empty() {
                if let Some(mut base) = lidar_library(&lib) {
                    base["spec_source"] = json!(format!("override:{lib}"));
                    upd(&mut base, &Value::Object(patch.clone()));
                    patch = base.as_object().cloned().unwrap();
                }
            }
            let mut found: Option<(&str, usize)> = None;
            for sec in secs { if let Some(a) = sp[sec].as_array() { for (i, s) in a.iter().enumerate() { if name_match(s, name) { found = Some((sec, i)); } } } }
            let mut added = patch.remove("_added").map(|v| truthy(&v)).unwrap_or(false);
            let clean: Map<String, Value> = patch.iter().filter(|(k, _)| !k.starts_with('_')).map(|(k, v)| (k.clone(), v.clone())).collect();
            let (sec, idx);
            if let Some((fs, fi)) = found {
                sec = fs; idx = fi;
                upd(&mut sp[sec][idx], &Value::Object(clean));
            } else {
                let stype = match patch.get("stype").filter(|v| truthy(v)) {
                    Some(v) => v.as_str().unwrap_or("camera").to_string(),
                    None => match patch.get("type").and_then(|t| t.as_str()) { Some("2d") => "lidar2d".into(), Some("3d") => "lidar3d".into(), Some(t) => t.to_string(), None => "camera".into() },
                };
                let mut s = sensor_template(&stype).unwrap_or_else(|_| json!({}));
                upd(&mut s, &Value::Object(clean));
                s["name"] = json!(name);
                s["source"] = json!("manual");
                let sc = if stype == "lidar2d" || stype == "lidar3d" { "lidars" } else if stype == "photoelectric" { "photoelectric" } else { "cameras" };
                if sc == "lidars" { s["type"] = json!(if stype == "lidar3d" { "3d" } else { "2d" }); }
                sp[sc].as_array_mut().unwrap().push(s);
                sec = sc; idx = sp[sc].as_array().unwrap().len() - 1;
                added = true;
            }
            let s = &mut sp[sec][idx];
            s.as_object_mut().unwrap().remove("stype");
            if sec == "lidars" {
                let fov = fnum(s.get("fov_deg"), 270.0);
                s["angle_min"] = json!(rnd(-fov.to_radians() / 2.0, 6));
                s["angle_max"] = json!(rnd(fov.to_radians() / 2.0, 6));
                let c = fnum(s.get("roll"), 0.0).cos();
                s["inverted"] = json!(c.abs() > 0.5 && c < 0.0);
                if s.get("housing_box").map(truthy).unwrap_or(false) { s["housing"] = json!({"box": s["housing_box"].clone()}); }
                if s.get("resolution_deg").is_none() { s["resolution_deg"] = json!(0.25); }
            }
            prov.insert(format!("sensors.{name}"), json!(if added { "added" } else { "manual" }));
        }
    }
    let nprov = prov.len();
    sp["provenance"] = Value::Object(prov);
    sp["protection_effective"] = protection::effective(Some(&sp), None);
    if nprov > 0 {
        let mut a: Vec<Value> = sp.get("assumptions").and_then(|a| a.as_array()).cloned().unwrap_or_default();
        a.retain(|x| !x.as_str().unwrap_or("").contains("overrides"));
        a.push(json!(format!("已应用人工补全 {nprov} 项 (model_overrides.json)")));
        sp["assumptions"] = Value::Array(a);
    }
    sp
}

pub fn sensor_list(spec: &Value, photos_effective: Option<&[Value]>) -> Vec<Value> {
    let prov = spec.get("provenance").cloned().unwrap_or(json!({}));
    let st_labels = sensor_types();
    let mut out = vec![];
    for sec in ["lidars", "cameras", "photoelectric"] {
        for s in spec.get(sec).and_then(|a| a.as_array()).cloned().unwrap_or_default() {
            let st = stype_of(&s, sec);
            let name = s["name"].as_str().unwrap_or("").to_string();
            let src = match prov.get(format!("sensors.{name}")) { Some(v) => v.clone(), None => if s.get("cmodel_name").map(truthy).unwrap_or(false) { json!("cmodel") } else { s.get("source").cloned().unwrap_or(json!("default")) } };
            let params: Map<String, Value> = s.as_object().unwrap().iter().filter(|(k, _)| !POSE_KEYS.contains(&k.as_str()) && !["name", "cmodel_name", "angle_min", "angle_max"].contains(&k.as_str())).map(|(k, v)| (k.clone(), v.clone())).collect();
            let model = s.get("vendor_model").filter(|v| truthy(v)).or_else(|| s.get("model").filter(|v| truthy(v))).cloned().unwrap_or(json!(""));
            let pose: Map<String, Value> = POSE_KEYS.iter().map(|k| (k.to_string(), json!(fnum(s.get(*k), 0.0)))).collect();
            out.push(json!({"name": name, "stype": st, "type_label": st_labels.get(&st).cloned().unwrap_or(json!(st)), "section": sec, "model": model, "source": src,
                "spec_source": s.get("spec_source").cloned().unwrap_or(Value::Null), "pose": Value::Object(pose), "params": Value::Object(params)}));
        }
    }
    if let Some(pe) = photos_effective {
        if spec.get("photoelectric").map(|p| !truthy(p)).unwrap_or(true) {
            for p in pe {
                let m = &p["mount"];
                out.push(json!({"name": p["name"], "stype": "photoelectric", "type_label": st_labels["photoelectric"], "section": "photoelectric", "model": "漫反射光电", "source": "default",
                    "pose": {"x": m["x"], "y": m["y"], "z": m["z"], "roll": 0.0, "pitch": 0.0, "yaw": m["yaw"]}, "params": {"trigger_m": p["trigger_m"], "range_m": p["range_m"]}}));
            }
        }
    }
    out
}

// ---- audit
struct Audit<'a> { prov: &'a Map<String, Value>, items: Vec<Value> }
impl<'a> Audit<'a> {
    #[allow(clippy::too_many_arguments)]
    fn add(&mut self, group: &str, key: &str, label: &str, value: Value, source: &str, severity: &str, note: &str, unit: &str, editable: bool, path: Option<&str>) {
        let p = path.unwrap_or(key);
        let pd = format!("{p}.");
        let manual = self.prov.contains_key(p) || self.prov.keys().any(|k| k.starts_with(&pd));
        let (source, severity) = if manual { ("manual", "ok") } else { (source, severity) };
        self.items.push(json!({"group": group, "key": key, "path": p, "label": label, "value": value, "unit": unit, "source": source, "severity": severity, "note": note, "editable": editable}));
    }
}

pub fn audit(spec: &Value, ov: Option<&Value>) -> Value {
    let _ = ov;
    let empty = Map::new();
    let prov = spec.get("provenance").and_then(|p| p.as_object()).unwrap_or(&empty);
    let ch = spec.get("chassis").cloned().unwrap_or(json!({}));
    let mut a = Audit { prov, items: vec![] };
    let g = |k: &str| ch.get(k).cloned().unwrap_or(Value::Null);
    a.add("车体", "chassis.footprint", "外形轮廓 (头/尾/左/右偏移)", json!([g("head_offset_m"), g("tail_offset_m"), g("left_offset_m"), g("right_offset_m")]), "cmodel", "ok", "", "m", false, None);
    a.add("车体", "chassis.height_m", "车体高度", g("height_m"), "cmodel", "ok", "", "m", true, None);
    let mass_default = fnum(ch.get("self_weight_kg"), 0.0) <= 0.0;
    a.add("车体", "chassis.mass_kg", "整车质量", g("mass_kg"), if mass_default { "default" } else { "cmodel" }, if mass_default { "warn" } else { "ok" }, if mass_default { "cmodel 自重缺失，按投影面积估算" } else { "" }, "kg", true, None);
    a.add("车体", "chassis.com", "质心位置", if ch.get("com").map(truthy).unwrap_or(false) { g("com") } else { json!("几何中心") }, "default", "info", "cmodel 不含质心，URDF 惯量按均质长方体", "m", true, None);
    let ml = ch.get("max_load_kg").map(truthy).unwrap_or(false);
    a.add("车体", "chassis.max_load_kg", "额定载荷", g("max_load_kg"), if ml { "cmodel" } else { "default" }, if ml { "ok" } else { "info" }, if ml { "" } else { "cmodel 为 0" }, "kg", true, None);
    a.add("车体", "chassis.visual", "车体外观网格", json!("包围盒 (底盘 + 上装)"), "default", "info", "cmodel 仅引用模块库 3D 资源路径，未内嵌网格文件；URDF 用包围盒", "", false, None);
    let wheels: Vec<Value> = spec.get("wheels").and_then(|w| w.as_array()).cloned().unwrap_or_default();
    for w in &wheels {
        let inferred = w.get("source").and_then(|s| s.as_str()) == Some("inferred");
        let nm = w["name"].as_str().unwrap_or("");
        let wg = |k: &str| w.get(k).cloned().unwrap_or(Value::Null);
        let (src, sev) = if inferred { ("inferred", "warn") } else { ("cmodel", "ok") };
        a.add("轮组", &format!("wheels.{nm}.pose"), &format!("{nm} 安装位置"), json!([wg("x"), wg("y")]), src, sev, if inferred { "cmodel 未建模被动轮，按底盘类型推断" } else { "" }, "m", true, Some(&format!("wheels.{nm}")));
        a.add("轮组", &format!("wheels.{nm}.radius_m"), &format!("{nm} 轮半径"), wg("radius_m"), src, sev, "", "m", true, Some(&format!("wheels.{nm}.radius_m")));
        a.add("轮组", &format!("wheels.{nm}.width_m"), &format!("{nm} 轮宽"), wg("width_m"), "default", "info", "cmodel 不含轮宽", "m", true, Some(&format!("wheels.{nm}.width_m")));
        a.add("轮组", &format!("wheels.{nm}.mass_kg"), &format!("{nm} 轮质量"), w.get("mass_kg").cloned().unwrap_or(json!(5.0)), "default", "info", "URDF 惯量用", "kg", true, Some(&format!("wheels.{nm}.mass_kg")));
        a.add("轮组", &format!("wheels.{nm}.friction"), &format!("{nm} 轮地摩擦系数"), w.get("friction").cloned().unwrap_or(json!(1.2)), "default", "info", "聚氨酯轮 0.8~1.2", "", true, Some(&format!("wheels.{nm}.friction")));
        for mk in ["drive_motor", "steer_motor"] {
            if let Some(m) = w.get(mk).filter(|m| truthy(m)) {
                let mn = m["name"].as_str().unwrap_or("");
                let ok = m.get("rated_torque_nm").map(truthy).unwrap_or(false);
                a.add("轮组", &format!("motors.{mn}.rated_torque_nm"), &format!("{mn} 额定转矩"), m.get("rated_torque_nm").cloned().unwrap_or(Value::Null), if ok { "cmodel" } else { "default" }, if ok { "ok" } else { "warn" },
                    if ok { "" } else { "cmodel torque 为空，电流/力矩仿真用经验值" }, "N·m", true, Some(&format!("motors.{mn}.rated_torque_nm")));
            }
        }
    }
    let lift_motor = spec.get("other_actuators").and_then(|o| o.as_array()).and_then(|o| o.iter().find(|m| m.get("name").and_then(|n| n.as_str()).unwrap_or("").to_lowercase().contains("lift"))).cloned();
    let lift = spec.get("lift").filter(|l| truthy(l)).cloned();
    if lift_motor.is_some() || lift.is_some() {
        let note = match (&lift_motor, &lift) { (Some(m), None) => format!("cmodel 有电机 {} 但无机构几何/行程", m["name"].as_str().unwrap_or("")), _ => String::new() };
        a.add("顶升机构", "lift", "顶升/货叉机构几何与行程", lift.clone().unwrap_or(Value::Null), if lift.is_some() { "manual" } else { "missing" }, if lift.is_some() { "ok" } else { "missing" }, &note, "", true, Some("lift"));
    }
    for l in spec.get("lidars").and_then(|l| l.as_array()).cloned().unwrap_or_default() {
        let src = l.get("spec_source").and_then(|s| s.as_str()).unwrap_or("").to_string();
        let sev = if (src.starts_with("override") || src.starts_with("library:")) && !src.contains("default") { "ok" } else { "warn" };
        let nm = l["name"].as_str().unwrap_or("");
        let model = l.get("vendor_model").filter(|v| truthy(v)).or_else(|| l.get("model").filter(|v| truthy(v))).cloned().unwrap_or(json!("未知型号"));
        a.add("传感器", &format!("sensors.{nm}"), &format!("激光 {nm} 型号参数 (视场/量程/分辨率)"), model, if src.contains("library") { "library" } else if src.contains("override") { "manual" } else { "default" }, sev,
            if sev == "ok" { "" } else { "cmodel 只有安装位姿，未识别型号 → 通用参数" }, "", true, Some(&format!("sensors.{nm}")));
    }
    let cams: Vec<Value> = spec.get("cameras").and_then(|c| c.as_array()).cloned().unwrap_or_default();
    if cams.is_empty() { a.add("传感器", "sensors.camera", "相机 (单目/双目/ToF)", Value::Null, "missing", "info", "cmodel 未定义相机，可在传感器安装页添加", "", true, Some("sensors.__camera")); }
    for c in &cams {
        let t = c.get("type").and_then(|t| t.as_str());
        let needs = matches!(t, Some("camera" | "stereo" | "tof" | "depthCamera")) && !c.get("width").map(truthy).unwrap_or(false);
        let nm = c["name"].as_str().unwrap_or("");
        let src = if needs { "default" } else if c.get("source").and_then(|s| s.as_str()) == Some("manual") { "manual" } else { "cmodel" };
        let val = c.get("model").filter(|v| truthy(v)).or_else(|| c.get("type")).cloned().unwrap_or(Value::Null);
        a.add("传感器", &format!("sensors.{nm}"), &format!("相机 {nm} 内参 (分辨率/视场/帧率)"), val, src, if needs { "warn" } else { "ok" }, if needs { "cmodel 不含相机内参 → 型号库默认值" } else { "" }, "", true, Some(&format!("sensors.{nm}")));
    }
    let imu = spec.get("imu").cloned().unwrap_or(json!({}));
    let zero = fnum(imu.get("x"), 0.0).abs() + fnum(imu.get("y"), 0.0).abs() + fnum(imu.get("z"), 0.0).abs() < 1e-6;
    a.add("传感器", "imu", "IMU 安装位置", json!([imu.get("x").cloned().unwrap_or(Value::Null), imu.get("y").cloned().unwrap_or(Value::Null), imu.get("z").cloned().unwrap_or(Value::Null)]), "cmodel", if zero { "info" } else { "ok" }, if zero { "cmodel 位姿为 0 (多为主控内置陀螺)" } else { "" }, "m", true, Some("imu"));
    if spec.get("photoelectric").map(|p| !truthy(p)).unwrap_or(true) { a.add("传感器", "sensors.photoelectric", "光电传感器", json!("四角缺省 4 个"), "default", "info", "cmodel 无光电模块，仿真按车身四角生成", "", true, Some("sensors.__photo")); }
    a.add("传感器", "sensors.bumpers", "防撞触边", json!("前/后缺省"), "default", "info", "cmodel 无触边模块，按车身前后边缘生成", "", false, None);
    {
        let p = protection::effective(Some(spec), None);
        let src = if prov.contains_key("protection") { "manual" } else { "default" };
        let auto = p.get("fields_auto").map(truthy).unwrap_or(false);
        a.items.push(json!({"group": "保护空间", "key": "protection.mode", "path": "protection", "label": "过弯方式 / 车体净空", "value": format!("{} · {} m", pstr(&p["corner_mode"]), pstr(&p["body_margin"])), "unit": "", "source": src, "severity": "ok",
            "note": format!("auto=能原地转向就停车转向，转不开用圆弧；防护区{}", if auto { "按车型动力学自动生成" } else { "为人工配置" }), "editable": true}));
        for it in protection::validate(spec, &p) {
            let num = it["value"].is_number();
            a.items.push(json!({"group": "保护空间", "key": format!("protection.{}", it["key"].as_str().unwrap_or("")), "path": "protection", "label": it["label"], "value": it["value"], "unit": if num { "m" } else { "" }, "source": src, "severity": it["severity"], "note": it["note"], "editable": true}));
        }
    }
    a.add("URDF", "urdf.optical", "相机光学坐标系 (_optical_frame)", json!("自动生成"), "default", "ok", "", "", false, None);
    a.add("URDF", "urdf.ros2_control", "ros2_control / transmission", Value::Null, "missing", "info", "仿真与执行解耦后由 REST 驱动，无需 ros2_control；上真车需补", "", false, None);
    a.add("URDF", "urdf.sensor_inertial", "传感器质量/惯量", json!("0.05 kg 占位"), "default", "info", "", "", false, None);
    let mut cnt = Map::new();
    for it in &a.items { let s = it["severity"].as_str().unwrap_or("").to_string(); let c = cnt.get(&s).and_then(|c| c.as_i64()).unwrap_or(0); cnt.insert(s, json!(c + 1)); }
    let manual = a.items.iter().filter(|i| i["source"] == json!("manual")).count();
    json!({"items": a.items.clone(), "summary": {"total": a.items.len(), "by_severity": Value::Object(cnt), "manual": manual}, "sensor_types": sensor_types()})
}
