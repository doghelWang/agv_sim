//! Robot assembly: one articulated geometry model derived from the cmodel spec.
//!
//! The cmodel (`CompDesc`) only describes modules electrically and logically.  It has the chassis footprint, wheels, motors,
//! sensors, IO and limits, but no shapes, strokes or mechanisms.  This module turns the parsed spec into a **RobotAssembly**:
//! a tree of links connected by joints, each with primitive visuals.  It also classifies the top actuator (jacking lifter,
//! reach-truck fork, stacker fork, pallet-truck fork, roller conveyor, tow hitch).  Every value that had to be guessed is
//! listed in `missing`.
//!
//! The same assembly drives:
//!  * the URDF export (`model::urdf`),
//!  * the robot bodies in the MuJoCo world (`mj::build_mjcf`),
//!  * the web 3D view (`web/js/assembly3d.js`), which animates joints from `joint_states`,
//!  * the actuator runtime (`actuator.rs` / `fork.rs`), which reads `actuator` parameters.
//!
//! Frame convention: `base_link` = motion center on the floor (z = 0), x forward, y left, z up (REP-103).
use serde_json::{json, Map, Value};
use std::f64::consts::PI;

pub const ARCHETYPES: [&str; 7] = ["jacking", "reach_fork", "stacker_fork", "pallet_truck", "roller", "tugger", "none"];

pub fn archetype_label(a: &str) -> &'static str {
    match a {
        "jacking" => "顶升潜伏式 AMR (顶升 + 托盘旋转)",
        "reach_fork" => "前移式叉车 AMR (门架前移 + 液压起升)",
        "stacker_fork" => "堆垛式叉车 AMR (门架起升)",
        "pallet_truck" => "搬运式叉车 AMR (低位托盘搬运)",
        "roller" => "辊筒对接 AMR (辊筒输送)",
        "tugger" => "牵引式 AMR (挂钩牵引料车)",
        _ => "底盘 (无上装执行机构)",
    }
}

fn n(v: &Value, k: &str, d: f64) -> f64 { v.get(k).and_then(|x| x.as_f64()).unwrap_or(d) }
fn npos(v: &Value, k: &str, d: f64) -> f64 { v.get(k).and_then(|x| x.as_f64()).filter(|x| *x > 0.0).unwrap_or(d) }
fn s<'a>(v: &'a Value, k: &str) -> &'a str { v.get(k).and_then(|x| x.as_str()).unwrap_or("") }
fn r4(x: f64) -> f64 { (x * 1e4).round() / 1e4 }
fn v3(a: [f64; 3]) -> Value { json!([r4(a[0]), r4(a[1]), r4(a[2])]) }

/// box visual: `size` = full extents
pub fn vbox(size: [f64; 3], xyz: [f64; 3], color: &str, role: &str) -> Value {
    json!({"shape": "box", "size": v3(size), "xyz": v3(xyz), "rpy": [0.0, 0.0, 0.0], "color": color, "role": role})
}
/// cylinder visual: axis = local z, rotated by rpy
pub fn vcyl(radius: f64, length: f64, xyz: [f64; 3], rpy: [f64; 3], color: &str, role: &str) -> Value {
    json!({"shape": "cylinder", "radius": r4(radius), "length": r4(length), "xyz": v3(xyz), "rpy": v3(rpy), "color": color, "role": role})
}
pub fn vsphere(radius: f64, xyz: [f64; 3], color: &str, role: &str) -> Value {
    json!({"shape": "sphere", "radius": r4(radius), "xyz": v3(xyz), "rpy": [0.0, 0.0, 0.0], "color": color, "role": role})
}

const C_BODY: &str = "#f59e0b";
const C_BODY_DARK: &str = "#b45309";
const C_COVER: &str = "#e5e7eb";
const C_STEEL: &str = "#475569";
const C_DARK: &str = "#1f2937";
const C_RUBBER: &str = "#111827";
const C_BUMPER: &str = "#0f172a";
const C_FORK: &str = "#f97316";
const C_TRAY: &str = "#334155";
const C_PAD: &str = "#0b0f19";
const C_RED: &str = "#dc2626";
const C_LIGHT: &str = "#22d3ee";
const C_SENSOR: &str = "#06b6d4";
const C_CAMERA: &str = "#7c3aed";
const C_ROLLER: &str = "#cbd5e1";
const C_YELLOW: &str = "#facc15";

struct B { links: Vec<Value> }
impl B {
    fn link(&mut self, name: &str, parent: &str, joint: Value, visuals: Vec<Value>) {
        let mut l = json!({"name": name, "parent": parent, "visuals": visuals});
        if !parent.is_empty() { l["joint"] = joint; }
        self.links.push(l);
    }
    fn fixed(&mut self, name: &str, parent: &str, xyz: [f64; 3], rpy: [f64; 3], visuals: Vec<Value>) {
        self.link(name, parent, json!({"name": format!("{}_joint", name.trim_end_matches("_link")), "type": "fixed", "xyz": v3(xyz), "rpy": v3(rpy)}), visuals);
    }
    #[allow(clippy::too_many_arguments)]
    fn joint(&mut self, name: &str, parent: &str, jname: &str, jtype: &str, axis: [f64; 3], xyz: [f64; 3], limits: Option<(f64, f64)>, extra: Value, visuals: Vec<Value>) {
        let mut j = json!({"name": jname, "type": jtype, "axis": v3(axis), "xyz": v3(xyz), "rpy": [0.0, 0.0, 0.0]});
        if let Some((lo, hi)) = limits { j["lower"] = json!(r4(lo)); j["upper"] = json!(r4(hi)); }
        if let Some(o) = extra.as_object() { for (k, v) in o { j[k] = v.clone(); } }
        self.link(name, parent, j, visuals);
    }
}

/// all text of the spec that hints at the mechanism (module names, motor models, IO names)
fn hint_text(spec: &Value) -> String {
    let mut t = vec![s(spec, "robot_name").to_string(), s(spec, "model_file").to_string()];
    for a in spec.get("other_actuators").and_then(|a| a.as_array()).into_iter().flatten() { t.push(s(a, "name").into()); t.push(s(a, "model").into()); }
    for k in ["inputs", "outputs"] {
        for d in spec.pointer(&format!("/io/{k}")).and_then(|a| a.as_array()).into_iter().flatten() { t.push(s(d, "name").into()); }
    }
    if let Some(f) = spec.get("fork").filter(|f| f.is_object()) { t.push(s(f, "name").into()); t.push("fork".into()); }
    t.join(" ").to_lowercase()
}

/// Decide which top actuator the vehicle carries.  Returns (archetype, source/reason).
pub fn classify(spec: &Value) -> (String, String) {
    for p in ["/actuator/type", "/actuator_type"] {
        if let Some(a) = spec.pointer(p).and_then(|v| v.as_str()) {
            if ARCHETYPES.contains(&a) { return (a.into(), "override: 人工指定 actuator.type".into()); }
        }
    }
    let t = hint_text(spec);
    let has = |ks: &[&str]| ks.iter().find(|k| t.contains(*k)).map(|k| k.to_string());
    let ch = &spec["chassis"];
    let (h, ctype) = (n(ch, "height_m", 0.0), s(ch, "type"));
    if let Some(k) = has(&["roller", "conveyor", "滚筒", "辊"]) { return ("roller".into(), format!("inferred: 模块名含 '{k}'")); }
    if let Some(k) = has(&["tow", "hitch", "hook", "tug", "牵引", "挂钩"]) { return ("tugger".into(), format!("inferred: 模块名含 '{k}'")); }
    if let Some(k) = has(&["doorframe", "door_frame", "reach", "oil_pump", "门架", "前移"]) { return ("reach_fork".into(), format!("inferred: 模块名含 '{k}' (门架/液压)")); }
    if let Some(f) = spec.get("fork").filter(|f| f.is_object()) {
        let mm = n(f, "mast_max_m", 0.0);
        if mm > 0.0 && mm <= 0.35 { return ("pallet_truck".into(), format!("inferred: lifter 起升行程 {mm:.2} m ≤ 0.35 m")); }
        if ctype.contains("steer") && h >= 1.8 && t.contains("reach") { return ("reach_fork".into(), "inferred: 舵轮 + 门架".into()); }
        return ("stacker_fork".into(), format!("inferred: lifter forkMast 起升行程 {mm:.2} m"));
    }
    if let Some(k) = has(&["lift", "jack", "顶升", "举升"]) {
        // a tall vehicle with a lift motor is a mast (pallet stacker), not a low jacking robot; a hydraulic pump motor or a
        // single steer wheel confirms it
        if h > 0.8 {
            let why = if t.contains("zapi") || t.contains("pump") || t.contains("油泵") { " + 液压泵电机" } else if ctype.contains("single_steer") { " + 单舵轮" } else { "" };
            return ("stacker_fork".into(), format!("inferred: 提升电机 '{k}' + 车高 {h:.3} m > 0.8 m (门架){why}; cmodel 无 fork 模块"));
        }
        return ("jacking".into(), format!("inferred: 顶升电机 '{k}' + 车高 {h:.3} m{}", if t.contains("up_sensor") || t.contains("down_sensor") { " + 上/下限位" } else { "" }));
    }
    ("none".into(), "inferred: cmodel 无上装执行机构模块".into())
}

fn miss(out: &mut Vec<Value>, cat: &str, item: &str, status: &str, value: Value, note: &str) {
    out.push(json!({"category": cat, "item": item, "status": status, "value": value, "note": note}));
}

/// height of the chassis body top (deck / tray / roller surface). A cmodel `height_m` above 0.8 m on a low vehicle is
/// the envelope incl. mast / lidar tower / guard rails, not the deck, so the deck is capped at 0.45 m.
pub fn body_top(arch: &str, hh: f64) -> f64 {
    match arch { "pallet_truck" => (0.30f64).min(hh), "reach_fork" => (0.38f64).min(hh), "stacker_fork" => (0.45f64).min(hh), _ => if hh > 0.8 { (0.45f64).min(hh) } else { hh } }
}

/// actuator parameters (with defaults for what the cmodel does not carry) + missing entries
fn actuator_params(spec: &Value, arch: &str, miss_out: &mut Vec<Value>) -> Value {
    let ch = &spec["chassis"];
    let (head, tail, left, right, hh) = (n(ch, "head_offset_m", 0.5), n(ch, "tail_offset_m", 0.5), n(ch, "left_offset_m", 0.4), n(ch, "right_offset_m", 0.4), npos(ch, "height_m", 0.3));
    let (l, w) = (head + tail, left + right);
    let ov = spec.get("actuator").cloned().unwrap_or(json!({}));
    // value from override → spec → default (recording provenance)
    let mut pick = |key: &str, from_spec: Option<f64>, def: f64, note: &str| -> f64 {
        if let Some(v) = ov.get(key).and_then(|v| v.as_f64()) { return v; }
        if let Some(v) = from_spec { return v; }
        miss(miss_out, "actuator", key, "defaulted", json!(r4(def)), note);
        def
    };
    let fork = spec.get("fork").filter(|f| f.is_object());
    let fv = |k: &str| fork.and_then(|f| f.get(k)).and_then(|v| v.as_f64()).filter(|v| *v > 0.0);
    let motor = spec.get("other_actuators").and_then(|a| a.as_array()).and_then(|a| a.first()).cloned().unwrap_or(Value::Null);
    match arch {
        "jacking" => {
            let stroke = pick("stroke_m", None, 0.06, "cmodel 只有顶升电机 (转速/减速比) 与上下限位, 无凸轮/丝杠行程");
            let speed = pick("lift_speed_mps", None, 0.03, "无丝杠导程/凸轮曲线, 无法由电机参数换算起升速度");
            let tray_d = pick("tray_diameter_m", None, r4((0.72 * l.min(w)).min(0.9)), "顶升托盘尺寸 cmodel 未描述");
            let rot = pick("rotate_speed_dps", None, 60.0, "托盘旋转机构 (同步旋转/独立电机) cmodel 未描述");
            let deck = body_top(arch, hh);
            let clr = pick("rack_clearance_m", None, r4(deck + 0.02), "货架底部净空 = 车体顶面 + 20 mm (推断)");
            if hh > 0.8 { miss(miss_out, "actuator", "deck_z_m", "inferred", json!(r4(deck)), "cmodel 车高 > 0.8 m 视为外形包络 (含上装/护栏), 托盘面按 0.45 m 车体计"); }
            json!({"type": "jacking", "stroke_m": stroke, "lift_speed_mps": speed, "lift_accel_mps2": 0.1, "tray_diameter_m": tray_d, "tray_thickness_m": 0.03,
                "rotate_speed_dps": rot, "rack_clearance_m": clr, "deck_z_m": r4(deck), "motor": motor,
                "limit_inputs": {"top": io_name(spec, &["up_sensor", "up_limit", "top"]), "bottom": io_name(spec, &["down_sensor", "down_limit", "bottom"])}})
        }
        "reach_fork" | "stacker_fork" | "pallet_truck" => {
            let pt = arch == "pallet_truck";
            let lift = pick("mast_max_m", fv("mast_max_m"), if pt { 0.2 } else if arch == "reach_fork" { 1.6 } else { 2.0 }, "起升高度: cmodel 只有门架/液压电机与限位 (拉线编码器量程未给)");
            let speed = pick("lift_speed_mps", fv("lift_speed_mps"), if pt { 0.06 } else { 0.2 }, "起升速度: 液压泵流量/缸径未给");
            let tl = pick("tine_length_m", fv("tine_length_m"), if pt { 1.15 } else { 1.07 }, "货叉长度未描述");
            let tw = pick("tine_width_m", fv("tine_width_m"), if pt { 0.16 } else { 0.10 }, "货叉宽度未描述");
            let tt = pick("tine_thickness_m", fv("tine_thickness_m"), if pt { 0.06 } else { 0.04 }, "货叉厚度未描述");
            let sp = pick("tine_spacing_m", fv("tine_spacing_m"), if pt { 0.36 } else { 0.56 }, "货叉外/内间距未描述");
            // fork direction: a cmodel without a fork module whose steer wheel sits ahead of the load axle (x = 0) carries
            // its forks at the rear (pallet stacker: drive unit + mast at the front, load wheels in the fork arms)
            let steer_x: Vec<f64> = spec.get("wheels").and_then(|w| w.as_array()).into_iter().flatten().filter(|w| s(w, "kind") == "steer").map(|w| n(w, "x", 0.0)).collect();
            let rear = fork.is_none() && !steer_x.is_empty() && steer_x.iter().all(|x| *x > 0.2) && head > tail + 0.3;
            let dir = match ov.get("fork_dir").and_then(|v| v.as_f64()).or(fork.and_then(|f| f.get("dir")).and_then(|v| v.as_f64())) {
                Some(d) => if d < 0.0 { -1.0 } else { 1.0 },
                None => if rear { -1.0 } else { 1.0 },
            };
            let dir_inferred = rear && dir < 0.0 && ov.get("fork_dir").is_none();
            let root = match fv("tine_root_x_m").or(fork.and_then(|f| f.get("x")).and_then(|v| v.as_f64())) { Some(x) => x, None => pick("tine_root_x_m", None, r4(if dir < 0.0 { -tail + tl } else if arch == "reach_fork" { -tail + (0.48 * l).max(0.5) + 0.08 } else { head }), "叉根位置未描述 (前移式: 缩回时门架位于车体前缘; 后向叉: 叉尖在车尾)") };
            if dir_inferred { pick("fork_dir", None, -1.0, "舵轮在载荷轴前方且 cmodel 无 fork 模块: 判定为后向货叉 (堆高车), 叉尖 = 车尾"); }
            let mut p = json!({"type": arch, "mast_max_m": lift, "lift_speed_mps": speed, "lift_accel_mps2": fv("lift_accel_mps2").unwrap_or(0.3),
                "tine_length_m": tl, "tine_width_m": tw, "tine_thickness_m": tt, "tine_spacing_m": sp, "tine_root_x_m": root, "fork_dir": dir,
                "max_load_kg": fv("max_load_kg").unwrap_or(n(ch, "max_load_kg", 0.0).max(1000.0)), "load_speed_factor": fv("load_speed_factor").unwrap_or(0.6), "motor": motor});
            if arch == "reach_fork" {
                let stroke = pick("reach_stroke_m", None, r4((0.42 * l).min(0.65)), "门架前移行程: cmodel 只有门架电机 + 原点/限位开关");
                let rs = pick("reach_speed_mps", None, 0.12, "前移速度: 电机减速比已知, 丝杠/链轮参数未给");
                p["reach_stroke_m"] = json!(stroke); p["reach_speed_mps"] = json!(rs);
                p["mast_height_m"] = json!(hh);
            } else if arch == "stacker_fork" { p["mast_height_m"] = json!(hh.max(1.6)); }
            p["limit_inputs"] = json!({"top": io_name(spec, &["up_limit", "up_sensor", "top"]), "bottom": io_name(spec, &["down_limit", "down_sensor", "bottom"]),
                "reach_origin": io_name(spec, &["doorframe_origin"]), "reach_limit": io_name(spec, &["doorframe_limit"])});
            p
        }
        "roller" => {
            let rd = pick("roller_diameter_m", None, 0.06, "辊筒直径/节距 cmodel 未描述");
            let pitch = pick("roller_pitch_m", None, 0.09, "辊筒节距未描述");
            let spd = pick("transfer_speed_mps", None, 0.3, "辊筒线速度: 无减速比/链轮参数");
            let deck = pick("deck_z_m", None, r4(body_top(arch, hh)), "辊面高度 = 车体顶面 (推断)");
            json!({"type": "roller", "roller_diameter_m": rd, "roller_pitch_m": pitch, "transfer_speed_mps": spd, "deck_z_m": deck, "transfer_axis": "y",
                "deck_length_m": r4(l - 0.08), "deck_width_m": r4(w), "motor": motor})
        }
        "tugger" => {
            let hz = pick("hitch_z_m", None, r4((hh * 0.6).clamp(0.2, 0.45)), "挂钩高度未描述");
            let hx = pick("hitch_x_m", None, r4(-tail - 0.06), "挂钩安装位置未描述");
            let ps = pick("pin_stroke_m", None, 0.08, "挂钩销行程未描述");
            json!({"type": "tugger", "hitch_x_m": hx, "hitch_z_m": hz, "pin_stroke_m": ps, "pin_speed_mps": 0.08, "capture_radius_m": 0.08, "max_articulation_deg": 75.0, "motor": motor})
        }
        _ => json!({"type": "none"}),
    }
}

fn io_name(spec: &Value, keys: &[&str]) -> Value {
    for d in spec.pointer("/io/inputs").and_then(|a| a.as_array()).into_iter().flatten() {
        let nm = s(d, "name");
        if keys.iter().any(|k| nm.to_lowercase().contains(k)) { return json!(format!("di_{nm}")); }
    }
    Value::Null
}

/// Build the assembly from a parsed spec (cmodel_spec output + overrides).
pub fn build(spec: &Value) -> Value {
    let ch = &spec["chassis"];
    let (head, tail, left, right) = (n(ch, "head_offset_m", 0.5), n(ch, "tail_offset_m", 0.5), n(ch, "left_offset_m", 0.4), n(ch, "right_offset_m", 0.4));
    let hh = npos(ch, "height_m", 0.3);
    let (l, w) = (head + tail, left + right);
    let (xc, yc) = ((head - tail) / 2.0, (left - right) / 2.0);
    let wheels: Vec<Value> = spec.get("active_wheels").or_else(|| spec.get("wheels")).and_then(|w| w.as_array()).cloned().unwrap_or_default();
    let rmin = wheels.iter().map(|w| n(w, "radius_m", 0.1)).fold(f64::INFINITY, f64::min);
    let rmin = if rmin.is_finite() { rmin } else { 0.1 };
    let clearance = (rmin * 0.45).max(0.02);
    let mut missing: Vec<Value> = vec![];
    let (arch, why) = classify(spec);
    if why.starts_with("inferred") { miss(&mut missing, "mechanism", "actuator.type", "inferred", json!(arch), &format!("cmodel 不声明机械结构类型, {why}")); }
    let act = actuator_params(spec, &arch, &mut missing);
    // --------------------------------------------------- generic cmodel gaps
    if n(ch, "self_weight_kg", 0.0) <= 0.0 { miss(&mut missing, "chassis", "mass_kg", "estimated", json!(n(ch, "mass_kg", 0.0)), "cmodel 自重缺失, 按投影面积估算"); }
    if n(ch, "max_load_kg", 0.0) <= 0.0 { miss(&mut missing, "chassis", "max_load_kg", "missing", Value::Null, "额定载荷缺失"); }
    miss(&mut missing, "chassis", "inertia/CoM", "defaulted", json!("均质长方体"), "cmodel 无质心/惯量; URDF 用均质盒体惯量");
    miss(&mut missing, "appearance", "mesh/material", "defaulted", json!("基本体拼装"), "cmodel 无外观网格 (STL/DAE), 车体/上装用参数化基本体生成");
    miss(&mut missing, "chassis", "body_shape", "defaulted", json!("footprint 拉伸 + 圆角盖板"), "仅有外廓矩形 (车头/车尾/左/右偏移), 无倒角、裙边、盖板分块");
    miss(&mut missing, "wheel", "tyre_friction", "defaulted", json!(0.8), "轮胎材料/摩擦系数未描述");
    for wd in &wheels {
        if s(wd, "source") == "inferred" { miss(&mut missing, "wheel", &format!("{}.position", s(wd, "name")), "inferred", json!([n(wd, "x", 0.0), n(wd, "y", 0.0)]), "cmodel 无万向轮模块, 在车体四角推断"); }
        if s(wd, "kind") == "caster" { miss(&mut missing, "wheel", &format!("{}.caster_offset", s(wd, "name")), "defaulted", json!(r4(0.35 * n(wd, "radius_m", 0.04))), "万向轮偏心距未描述 (影响转向时脚轮摆动)"); }
    }
    for li in spec.get("lidars").and_then(|a| a.as_array()).into_iter().flatten() {
        if s(li, "spec_source").starts_with("library:default") { miss(&mut missing, "sensor", &format!("{}.optics", s(li, "name")), "defaulted", json!(s(li, "vendor_model")), "激光型号未在库中, 视场/量程/分辨率用通用默认"); }
    }
    for c in spec.get("cameras").and_then(|a| a.as_array()).into_iter().flatten() {
        if c.get("width").is_none() { miss(&mut missing, "sensor", &format!("{}.intrinsics", s(c, "name")), "defaulted", json!(s(c, "type")), "相机分辨率/视场/畸变未描述, 用型号库默认"); }
    }
    for a in spec.get("assumptions").and_then(|a| a.as_array()).into_iter().flatten() { if let Some(t) = a.as_str() { miss(&mut missing, "spec", "assumption", "inferred", Value::Null, t); } }

    // --------------------------------------------------- links
    let mut b = B { links: vec![] };
    let top = hh;
    let body_h = body_top(&arch, top);
    let mut base_vis: Vec<Value> = vec![];
    let body_z0 = clearance;
    let cover_t = 0.012;
    // body geometry depends on the archetype (straddle legs for fork trucks)
    match arch.as_str() {
        "reach_fork" | "pallet_truck" | "stacker_fork" => {
            // drive unit / battery housing at the rear, straddle legs or fork arms towards the front
            let root = n(&act, "tine_root_x_m", head);
            let fdir = n(&act, "fork_dir", 1.0);
            let rear_len = if arch == "reach_fork" { (0.48 * l).max(0.5) } else if fdir < 0.0 { (head - root).max(0.4) } else { (root + tail).max(0.4) };
            let rx0 = if fdir < 0.0 { head - rear_len } else { -tail };
            let tower_h = if arch == "reach_fork" { (top * 0.62).max(1.0) } else if arch == "pallet_truck" { top } else { (top * 0.5).max(0.9) };
            base_vis.push(vbox([rear_len, w, body_h - body_z0], [rx0 + rear_len / 2.0, yc, body_z0 + (body_h - body_z0) / 2.0], C_BODY, "chassis"));
            // upper housing (battery / control cabinet), slightly narrower, with dark top cover
            let up_w = w * 0.86;
            let up_len = rear_len * if arch == "pallet_truck" { 0.42 } else { 0.62 };
            if tower_h > body_h + 0.05 {
                let ux = rx0 + up_len / 2.0 + 0.03;
                base_vis.push(vbox([up_len, up_w, tower_h - body_h], [ux, yc, body_h + (tower_h - body_h) / 2.0], C_BODY, "cabinet"));
                base_vis.push(vbox([up_len + 0.01, up_w + 0.01, cover_t], [ux, yc, tower_h + cover_t / 2.0], C_DARK, "cover"));
                // HMI screen + tower light on the cabinet
                base_vis.push(vbox([0.02, 0.24, 0.16], [ux + up_len / 2.0 + 0.01, yc, tower_h - 0.16], C_DARK, "hmi"));
                base_vis.push(vcyl(0.03, 0.18, [ux - up_len / 4.0, yc + up_w / 2.0 - 0.06, tower_h + 0.09], [0.0, 0.0, 0.0], C_LIGHT, "tower_light"));
                // overhead guard (reach truck) - two posts + roof grid
                if arch == "reach_fork" && top > tower_h + 0.2 {
                    for sy in [-1.0, 1.0] {
                        base_vis.push(vbox([0.05, 0.05, top - tower_h], [ux - up_len / 2.0 + 0.04, yc + sy * (up_w / 2.0 - 0.03), tower_h + (top - tower_h) / 2.0], C_DARK, "guard"));
                    }
                    base_vis.push(vbox([up_len * 0.9, up_w, 0.03], [ux, yc, top - 0.015], C_DARK, "guard"));
                }
            }
            if arch == "reach_fork" {
                // straddle legs with load wheels under the tips
                let leg_len = head - (rx0 + rear_len);
                let leg_w = 0.16f64.min(w * 0.12);
                for sy in [-1.0, 1.0] {
                    let ly = yc + sy * (w / 2.0 - leg_w / 2.0);
                    base_vis.push(vbox([leg_len, leg_w, 0.11], [rx0 + rear_len + leg_len / 2.0, ly, 0.075], C_BODY_DARK, "straddle_leg"));
                    base_vis.push(vbox([leg_len, leg_w + 0.004, 0.012], [rx0 + rear_len + leg_len / 2.0, ly, 0.136], C_YELLOW, "leg_stripe"));
                }
            }
            // bumpers / scanner guard on the rear
            base_vis.push(vbox([0.04, w * 0.95, 0.09], [rx0 - 0.02, yc, body_z0 + 0.08], C_BUMPER, "bumper_rear"));
        }
        _ => {
            // slab chassis: lower skirt + body + top cover (jacking top is the tray deck)
            let skirt = 0.03f64.min(body_h * 0.2);
            base_vis.push(vbox([l - 0.02, w - 0.02, skirt], [xc, yc, body_z0 + skirt / 2.0], C_DARK, "skirt"));
            base_vis.push(vbox([l, w, body_h - body_z0 - skirt - cover_t], [xc, yc, body_z0 + skirt + (body_h - body_z0 - skirt - cover_t) / 2.0], C_BODY, "chassis"));
            base_vis.push(vbox([l - 0.01, w - 0.01, cover_t], [xc, yc, body_h - cover_t / 2.0], if arch == "jacking" { C_DARK } else { C_COVER }, "cover"));
            // front / rear bumpers (rubber strips)
            for sx in [1.0, -1.0] {
                let x = if sx > 0.0 { head } else { -tail };
                base_vis.push(vbox([0.03, w * 0.9, (body_h - body_z0) * 0.38], [x + sx * 0.015, yc, body_z0 + (body_h - body_z0) * 0.42], C_BUMPER, if sx > 0.0 { "bumper_front" } else { "bumper_rear" }));
            }
            // light strips on the four sides
            for sy in [1.0, -1.0] { base_vis.push(vbox([l * 0.6, 0.006, 0.014], [xc, yc + sy * (w / 2.0 + 0.003), body_h * 0.72], C_LIGHT, "light_strip")); }
            for sx in [1.0, -1.0] { base_vis.push(vbox([0.006, w * 0.5, 0.014], [xc + sx * (l / 2.0 + 0.003), yc, body_h * 0.72], C_LIGHT, "light_strip")); }
            if top > body_h + 0.05 {
                // superstructure (unknown upper body): transparent cabinet outline
                base_vis.push(json!({"shape": "box", "size": v3([l * 0.94, w * 0.94, top - body_h]), "xyz": v3([xc, yc, body_h + (top - body_h) / 2.0]), "rpy": [0.0, 0.0, 0.0], "color": "#94a3b8", "opacity": 0.25, "role": "upper"}));
            }
        }
    }
    // e-stop buttons
    let n_estop = spec.pointer("/io/buttons").and_then(|a| a.as_array()).map(|a| a.iter().filter(|b| s(b, "type").contains("EMCY") && !s(b, "type").contains("NOT")).count()).unwrap_or(0).max(1);
    let estop_z = if ["reach_fork", "stacker_fork", "pallet_truck"].contains(&arch.as_str()) { body_h + 0.02 } else { body_h + 0.012 };
    // rear-facing forks: the body (and its back end: e-stops, charging contacts) is at the +x end
    let rear_fork = act.get("fork_dir").and_then(|v| v.as_f64()).unwrap_or(1.0) < 0.0;
    let back = if rear_fork { head } else { -tail };
    let inward = if rear_fork { -1.0 } else { 1.0 };
    for i in 0..n_estop.min(4) {
        let (sx, sy) = [(1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0)][i];
        let x = if ["reach_fork", "stacker_fork", "pallet_truck"].contains(&arch.as_str()) { back + inward * 0.08 } else { xc + sx * (l / 2.0 - 0.08) };
        base_vis.push(vcyl(0.025, 0.02, [x, yc + sy * (w / 2.0 - 0.08), estop_z], [0.0, 0.0, 0.0], C_RED, "estop"));
        base_vis.push(vcyl(0.035, 0.008, [x, yc + sy * (w / 2.0 - 0.08), estop_z - 0.012], [0.0, 0.0, 0.0], C_YELLOW, "estop_ring"));
    }
    // charging contacts (rear, low)
    base_vis.push(vbox([0.01, 0.16, 0.05], [back - inward * 0.005, yc, body_z0 + 0.05], "#b8860b", "charge_contacts"));
    b.link("base_link", "", Value::Null, base_vis);

    // --------------------------------------------------- wheels
    for wd in &wheels {
        let (name, kind) = (s(wd, "name"), s(wd, "kind"));
        let (x, y, r) = (n(wd, "x", 0.0), n(wd, "y", 0.0), npos(wd, "radius_m", 0.08));
        let wid = npos(wd, "width_m", (r * 0.55).max(0.035));
        let tire = |rr: f64| vec![vcyl(rr, wid, [0.0, 0.0, 0.0], [PI / 2.0, 0.0, 0.0], C_RUBBER, "tyre"), vcyl(rr * 0.62, wid * 1.04, [0.0, 0.0, 0.0], [PI / 2.0, 0.0, 0.0], C_STEEL, "hub"),
            vbox([rr * 1.5, wid * 1.08, rr * 0.16], [0.0, 0.0, 0.0], C_YELLOW, "spoke")];
        match kind {
            "steer" => {
                let st = format!("{name}_steer_link");
                b.joint(&st, "base_link", &format!("{name}_steer_joint"), "revolute", [0.0, 0.0, 1.0], [x, y, r],
                    Some((n(wd, "steer_min_rad", -PI / 2.0), n(wd, "steer_max_rad", PI / 2.0))), json!({"source": "wheel"}),
                    vec![vcyl(r * 0.85, 0.06, [0.0, 0.0, r + 0.05], [0.0, 0.0, 0.0], C_STEEL, "steer_ring"), vbox([r * 0.9, wid + 0.05, r * 0.9], [0.0, 0.0, r * 0.6], C_DARK, "steer_motor"),
                        vbox([r * 0.35, 0.01, 0.01], [r * 0.6, 0.0, r + 0.09], C_RED, "steer_mark")]);
                b.joint(&format!("{name}_link"), &st, &format!("{name}_drive_joint"), "continuous", [0.0, 1.0, 0.0], [0.0, 0.0, 0.0], None, json!({"source": "wheel"}), tire(r));
            }
            "caster" => {
                let off = 0.35 * r;
                let sz = 2.0 * r + 0.015;
                let sw = format!("{name}_swivel_link");
                b.joint(&sw, "base_link", &format!("{name}_swivel_joint"), "continuous", [0.0, 0.0, 1.0], [x, y, sz], None, json!({"source": "caster"}),
                    vec![vcyl(r * 0.7, 0.012, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], C_STEEL, "caster_plate"), vbox([off + r * 0.4, wid + 0.016, 0.01], [-off / 2.0, 0.0, -0.01], C_STEEL, "caster_fork"),
                        vbox([0.008, 0.006, sz - r], [-off, wid / 2.0 + 0.008, -(sz - r) / 2.0], C_STEEL, "caster_fork"), vbox([0.008, 0.006, sz - r], [-off, -wid / 2.0 - 0.008, -(sz - r) / 2.0], C_STEEL, "caster_fork")]);
                b.joint(&format!("{name}_link"), &sw, &format!("{name}_wheel_joint"), "continuous", [0.0, 1.0, 0.0], [-off, 0.0, r - sz], None, json!({"source": "caster"}),
                    vec![vcyl(r, wid, [0.0, 0.0, 0.0], [PI / 2.0, 0.0, 0.0], "#9ca3af", "tyre"), vcyl(r * 0.4, wid * 1.05, [0.0, 0.0, 0.0], [PI / 2.0, 0.0, 0.0], C_STEEL, "hub")]);
            }
            _ => {
                b.joint(&format!("{name}_link"), "base_link", &format!("{name}_joint"), "continuous", [0.0, 1.0, 0.0], [x, y, r], None, json!({"source": "wheel"}), tire(r));
            }
        }
    }

    // --------------------------------------------------- sensors (fixed)
    for li in spec.get("lidars").and_then(|a| a.as_array()).into_iter().flatten() {
        let name = s(li, "name");
        let is3d = s(li, "type") == "3d";
        let (dr, dh) = li.pointer("/housing/cylinder_d_h").and_then(|a| a.as_array()).map(|a| (a[0].as_f64().unwrap_or(0.08) / 2.0, a[1].as_f64().unwrap_or(0.08))).unwrap_or(if is3d { (0.0325, 0.065) } else { (0.04, 0.08) });
        let mut v = vec![vcyl(dr, dh * 0.72, [0.0, 0.0, -dh * 0.14], [0.0, 0.0, 0.0], C_DARK, "lidar_body"), vcyl(dr * 0.96, dh * 0.28, [0.0, 0.0, dh * 0.22], [0.0, 0.0, 0.0], C_SENSOR, "lidar_window")];
        if n(li, "z", 0.2) - dh / 2.0 - body_h > 0.01 { v.push(vcyl(0.012, (n(li, "z", 0.2) - dh / 2.0 - body_h).max(0.0), [0.0, 0.0, -(n(li, "z", 0.2) - body_h) / 2.0 - dh / 4.0], [0.0, 0.0, 0.0], C_STEEL, "lidar_post")); }
        b.fixed(&format!("{name}_link"), "base_link", [n(li, "x", 0.0), n(li, "y", 0.0), n(li, "z", 0.2)], [n(li, "roll", 0.0), n(li, "pitch", 0.0), n(li, "yaw", 0.0)], v);
    }
    for c in spec.get("cameras").and_then(|a| a.as_array()).into_iter().flatten() {
        let name = s(c, "name");
        let down = s(c, "type") == "codeReader";
        let mut z = n(c, "z", 0.2);
        if down && z <= 0.0 { z = (body_z0 + 0.06).max(0.08); }
        let v = if down { vec![vbox([0.06, 0.06, 0.04], [0.0, 0.0, 0.0], C_CAMERA, "code_reader"), vcyl(0.012, 0.012, [0.0, 0.0, -0.026], [0.0, 0.0, 0.0], C_SENSOR, "lens")] }
            else { vec![vbox([0.04, 0.10, 0.035], [0.0, 0.0, 0.0], C_CAMERA, "camera"), vcyl(0.012, 0.012, [0.024, 0.0, 0.0], [0.0, PI / 2.0, 0.0], C_SENSOR, "lens")] };
        let up = s(c, "orientation").ends_with("UP");
        b.fixed(&format!("{name}_link"), "base_link", [n(c, "x", 0.0), n(c, "y", 0.0), z], [0.0, if down { if up { -PI / 2.0 } else { PI / 2.0 } } else { n(c, "pitch", 0.0) }, n(c, "yaw", 0.0)], v);
    }

    // --------------------------------------------------- actuator links
    let mut ajoints: Vec<Value> = vec![];
    match arch.as_str() {
        "jacking" => {
            let (stroke, td) = (n(&act, "stroke_m", 0.06), n(&act, "tray_diameter_m", 0.7));
            let tz = body_h;
            b.joint("lift_link", "base_link", "lift_joint", "prismatic", [0.0, 0.0, 1.0], [xc, yc, tz - 0.036], Some((0.0, stroke)), json!({"source": "actuator", "effort": 3000.0, "velocity": n(&act, "lift_speed_mps", 0.03)}),
                vec![vcyl(td * 0.30, stroke + 0.01, [0.0, 0.0, -(stroke + 0.01) / 2.0], [0.0, 0.0, 0.0], C_STEEL, "lift_column")]);
            let mut tv = vec![vcyl(td / 2.0, 0.03, [0.0, 0.0, 0.015], [0.0, 0.0, 0.0], C_TRAY, "tray"), vcyl(td / 2.0 - 0.02, 0.006, [0.0, 0.0, 0.033], [0.0, 0.0, 0.0], C_PAD, "tray_pad")];
            for k in 0..4 { let a = k as f64 * PI / 2.0 + PI / 4.0; tv.push(vbox([0.06, 0.06, 0.008], [(td * 0.36) * a.cos(), (td * 0.36) * a.sin(), 0.04], C_YELLOW, "tray_locator")); }
            tv.push(vbox([td * 0.25, 0.02, 0.004], [td * 0.2, 0.0, 0.037], C_RED, "tray_mark"));
            b.joint("tray_link", "lift_link", "tray_rotate_joint", "continuous", [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], None, json!({"source": "actuator"}), tv);
            ajoints = vec![json!({"name": "lift_joint", "quantity": "height_m"}), json!({"name": "tray_rotate_joint", "quantity": "tray_yaw_rad"})];
        }
        "reach_fork" | "stacker_fork" | "pallet_truck" => {
            let (lift, tl, tw, tt, sp, root) = (n(&act, "mast_max_m", 1.6), n(&act, "tine_length_m", 1.07), n(&act, "tine_width_m", 0.1), n(&act, "tine_thickness_m", 0.04), n(&act, "tine_spacing_m", 0.56), n(&act, "tine_root_x_m", head));
            let mut mast_parent;
            let fdir = n(&act, "fork_dir", 1.0);
            // rear-facing forks: the whole mast / carriage is built in a mount frame turned by 180° at the fork root
            let rear = fdir < 0.0 && arch != "reach_fork";
            let root = if rear { b.fixed("fork_mount_link", "base_link", [root, yc, 0.0], [0.0, 0.0, PI], vec![]); 0.0 } else { root };
            if arch == "reach_fork" {
                let stroke = n(&act, "reach_stroke_m", 0.5);
                b.joint("reach_link", "base_link", "reach_joint", "prismatic", [1.0, 0.0, 0.0], [root, yc, 0.0], Some((0.0, stroke)), json!({"source": "actuator", "velocity": n(&act, "reach_speed_mps", 0.12)}),
                    vec![vbox([0.24, w * 0.62, 0.08], [-0.14, 0.0, 0.10], C_STEEL, "reach_carriage"), vbox([0.04, w * 0.55, 0.3], [-0.24, 0.0, 0.3], C_DARK, "hydraulic_block")]);
                mast_parent = "reach_link".to_string();
            } else { mast_parent = if rear { "fork_mount_link".into() } else { "base_link".into() }; }
            let mx = if arch == "reach_fork" { 0.0 } else { root };
            if arch != "pallet_truck" {
                let mh = n(&act, "mast_height_m", hh.max(1.6));
                let mw = (sp + tw + 0.18).min(w * 0.9);
                let mut mv = vec![];
                for sy in [-1.0, 1.0] { mv.push(vbox([0.07, 0.06, mh], [mx - 0.07, sy * mw / 2.0, mh / 2.0 + 0.02], C_DARK, "mast_outer")); }
                mv.push(vbox([0.06, mw, 0.06], [mx - 0.07, 0.0, mh - 0.01], C_DARK, "mast_cross"));
                mv.push(vcyl(0.03, mh * 0.85, [mx - 0.11, 0.0, mh * 0.45], [0.0, 0.0, 0.0], "#94a3b8", "lift_cylinder"));
                b.fixed("mast_link", &mast_parent, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], mv);
                let mut iv = vec![];
                for sy in [-1.0, 1.0] { iv.push(vbox([0.05, 0.05, mh * 0.96], [mx - 0.05, sy * (mw / 2.0 - 0.06), mh * 0.48 + 0.04], C_STEEL, "mast_inner")); }
                iv.push(vbox([0.05, mw - 0.1, 0.05], [mx - 0.05, 0.0, mh * 0.96 + 0.02], C_STEEL, "mast_inner_cross"));
                b.joint("mast_inner_link", "mast_link", "mast_stage_joint", "prismatic", [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], Some((0.0, lift / 2.0)), json!({"source": "actuator", "mimic": {"joint": "fork_lift_joint", "multiplier": 0.5}}), iv);
            }
            let carriage_h = if arch == "pallet_truck" { 0.0 } else { 0.42 };
            let mut cv = vec![];
            if arch == "pallet_truck" {
                // fork arms carry the load wheels; the "carriage" is the fork frame welded to the lift plate
                cv.push(vbox([0.06, sp + tw + 0.04, 0.22], [mx - 0.03, 0.0, 0.14], C_DARK, "backrest"));
            } else {
                cv.push(vbox([0.04, (sp + tw + 0.16).min(w * 0.86), carriage_h], [mx - 0.02, 0.0, carriage_h / 2.0 + 0.03], C_DARK, "carriage"));
                for zz in [0.6 * carriage_h, carriage_h + 0.25] { cv.push(vbox([0.03, (sp + tw + 0.2).min(w * 0.9), 0.03], [mx - 0.02, 0.0, zz + 0.03], C_STEEL, "load_backrest")); }
                for k in 0..5 { let y = -((sp + tw + 0.2).min(w * 0.9)) / 2.0 + k as f64 * ((sp + tw + 0.2).min(w * 0.9)) / 4.0; cv.push(vbox([0.02, 0.02, 0.25], [mx - 0.02, y, carriage_h + 0.155], C_STEEL, "load_backrest")); }
            }
            for sy in [-1.0, 1.0] {
                let ty = sy * sp / 2.0;
                cv.push(vbox([tl, tw, tt], [mx + tl / 2.0, ty, tt / 2.0], C_FORK, "tine"));
                if arch != "pallet_truck" { cv.push(vbox([tt, tw, 0.42], [mx + tt / 2.0, ty, 0.21], C_FORK, "tine_shank")); }
                cv.push(vbox([0.06, tw * 0.9, tt * 0.6], [mx + tl - 0.03, ty, tt * 0.3], "#ea580c", "tine_tip"));
            }
            if arch == "pallet_truck" && !rear { mast_parent = "base_link".into(); }
            b.joint("carriage_link", if arch == "pallet_truck" { mast_parent.as_str() } else { "mast_link" }, "fork_lift_joint", "prismatic", [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], Some((0.0, lift)),
                json!({"source": "actuator", "velocity": n(&act, "lift_speed_mps", 0.2), "effort": 20000.0}), cv);
            if arch == "pallet_truck" {
                // load rollers under the fork tips (stay on the floor; the fork lifts relative to them)
                for sy in [-1.0, 1.0] {
                    b.fixed(&format!("load_roller_{}", if sy > 0.0 { "l" } else { "r" }), &mast_parent, [root + tl - 0.18, if rear { 0.0 } else { yc } + sy * sp / 2.0, 0.035], [0.0, 0.0, 0.0],
                        vec![vcyl(0.035, tw * 0.8, [0.0, 0.0, 0.0], [PI / 2.0, 0.0, 0.0], "#9ca3af", "load_roller")]);
                }
            }
            ajoints = vec![json!({"name": "fork_lift_joint", "quantity": "height_m"})];
            if arch != "pallet_truck" { ajoints.push(json!({"name": "mast_stage_joint", "quantity": "height_m", "multiplier": 0.5})); }
            if arch == "reach_fork" { ajoints.push(json!({"name": "reach_joint", "quantity": "reach_m"})); }
        }
        "roller" => {
            let (rd, pitch, deck) = (n(&act, "roller_diameter_m", 0.06), n(&act, "roller_pitch_m", 0.09), n(&act, "deck_z_m", hh));
            let (dl, dw) = (n(&act, "deck_length_m", l - 0.08), n(&act, "deck_width_m", w));
            let nr = ((dl / pitch).floor() as usize).clamp(3, 40);
            let mut fv = vec![];
            for sx in [-1.0, 1.0] { fv.push(vbox([0.03, dw, 0.06], [xc + sx * (dl / 2.0 + 0.015), yc, deck - 0.01], C_STEEL, "deck_frame")); fv.push(vbox([0.02, dw, 0.05], [xc + sx * (dl / 2.0 + 0.04), yc, deck + 0.03], C_YELLOW, "end_guide")); }
            b.fixed("deck_link", "base_link", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], fv);
            for i in 0..nr {
                let x = xc - dl / 2.0 + pitch / 2.0 + i as f64 * (dl - pitch) / (nr as f64 - 1.0).max(1.0);
                b.joint(&format!("roller_{i}_link"), "deck_link", &format!("roller_{i}_joint"), "continuous", [1.0, 0.0, 0.0], [x, yc, deck - rd / 2.0], None,
                    json!({"source": "actuator", "mimic": {"joint": "roller_0_joint", "multiplier": 1.0}}),
                    vec![vcyl(rd / 2.0, dw - 0.04, [0.0, 0.0, 0.0], [PI / 2.0, 0.0, 0.0], C_ROLLER, "roller"), vbox([0.004, dw - 0.05, rd * 0.9], [0.0, 0.0, 0.0], "#64748b", "roller_mark")]);
            }
            // retractable stop pins at the two transfer edges
            for (k, sy) in [("l", 1.0), ("r", -1.0)] {
                b.joint(&format!("stopper_{k}_link"), "deck_link", &format!("stopper_{k}_joint"), "prismatic", [0.0, 0.0, 1.0], [xc, yc + sy * (dw / 2.0 - 0.01), deck - 0.04], Some((0.0, 0.06)), json!({"source": "actuator"}),
                    vec![vbox([dl * 0.3, 0.015, 0.05], [0.0, 0.0, 0.025], C_RED, "stopper")]);
            }
            ajoints = vec![json!({"name": "roller_*_joint", "quantity": "roller_angle_rad"}), json!({"name": "stopper_l_joint", "quantity": "stop_m"}), json!({"name": "stopper_r_joint", "quantity": "stop_m"})];
        }
        "tugger" => {
            let (hx, hz, ps) = (n(&act, "hitch_x_m", -tail - 0.06), n(&act, "hitch_z_m", 0.3), n(&act, "pin_stroke_m", 0.08));
            b.fixed("hitch_link", "base_link", [hx, yc, hz], [0.0, 0.0, 0.0],
                vec![vbox([0.14, 0.16, 0.02], [0.04, 0.0, 0.03], C_DARK, "hitch_jaw"), vbox([0.14, 0.16, 0.02], [0.04, 0.0, -0.03], C_DARK, "hitch_jaw"), vbox([0.04, 0.16, 0.1], [0.11, 0.0, 0.0], C_STEEL, "hitch_mount"), vbox([0.06, 0.18, 0.012], [0.06, 0.0, 0.05], C_YELLOW, "hitch_mark")]);
            b.joint("hitch_pin_link", "hitch_link", "hitch_pin_joint", "prismatic", [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], Some((0.0, ps)), json!({"source": "actuator"}),
                vec![vcyl(0.015, 0.14, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], C_STEEL, "hitch_pin"), vcyl(0.025, 0.02, [0.0, 0.0, 0.08], [0.0, 0.0, 0.0], C_RED, "pin_head")]);
            ajoints = vec![json!({"name": "hitch_pin_joint", "quantity": "pin_up_m"})];
        }
        _ => {}
    }
    let mut a = act;
    a["joints"] = Value::Array(ajoints);
    let sensors: Vec<Value> = spec.get("lidars").and_then(|a| a.as_array()).into_iter().flatten().map(|l| json!({"name": s(l, "name"), "kind": if s(l, "type") == "3d" { "lidar3d" } else { "lidar2d" }, "link": format!("{}_link", s(l, "name"))}))
        .chain(spec.get("cameras").and_then(|a| a.as_array()).into_iter().flatten().map(|c| json!({"name": s(c, "name"), "kind": s(c, "type"), "link": format!("{}_link", s(c, "name"))}))).collect();
    let mut m = Map::new();
    m.insert("version".into(), json!(1));
    m.insert("name".into(), json!(s(spec, "robot_name")));
    m.insert("model_file".into(), spec.get("model_file").cloned().unwrap_or(Value::Null));
    m.insert("archetype".into(), json!(arch));
    m.insert("archetype_label".into(), json!(archetype_label(&arch)));
    m.insert("archetype_source".into(), json!(why));
    m.insert("chassis".into(), json!({"type": s(ch, "type"), "length_m": r4(l), "width_m": r4(w), "height_m": r4(hh), "body_top_m": r4(body_h), "clearance_m": r4(clearance), "center": [r4(xc), r4(yc)], "mass_kg": n(ch, "mass_kg", 200.0)}));
    m.insert("actuator".into(), a);
    m.insert("links".into(), Value::Array(b.links));
    m.insert("sensors".into(), Value::Array(sensors));
    m.insert("missing".into(), Value::Array(missing));
    Value::Object(m)
}

/// joint name list of the assembly (non-fixed joints, in link order)
pub fn joint_names(asm: &Value) -> Vec<String> {
    asm["links"].as_array().into_iter().flatten().filter_map(|l| l.get("joint")).filter(|j| s(j, "type") != "fixed").map(|j| s(j, "name").to_string()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    fn spec(ch_type: &str, h: f64, acts: &[&str], fork: Option<f64>) -> Value {
        let mut s = json!({"robot_name": "t", "chassis": {"type": ch_type, "head_offset_m": 0.7, "tail_offset_m": 0.7, "left_offset_m": 0.5, "right_offset_m": 0.5, "height_m": h, "mass_kg": 200.0},
            "wheels": [{"name": "wl", "kind": "drive", "x": 0.0, "y": 0.45, "radius_m": 0.065}, {"name": "wr", "kind": "drive", "x": 0.0, "y": -0.45, "radius_m": 0.065},
                {"name": "c1", "kind": "caster", "x": 0.5, "y": 0.3, "radius_m": 0.04, "source": "inferred"}],
            "other_actuators": acts.iter().map(|a| json!({"name": a})).collect::<Vec<_>>(), "io": {"inputs": [{"name": "up_sensor"}, {"name": "down_sensor"}]}});
        if let Some(m) = fork { s["fork"] = json!({"name": "fork_mast", "x": 0.7, "mast_max_m": m}); }
        s
    }
    #[test]
    fn classify_types() {
        assert_eq!(classify(&spec("diff_drive", 0.18, &["motor-lift"], None)).0, "jacking");
        assert_eq!(classify(&spec("dual_steer", 1.9, &["motor_DoorFrame"], None)).0, "reach_fork");
        assert_eq!(classify(&spec("diff_drive", 1.1, &["lift_motor"], Some(0.2))).0, "pallet_truck");
        assert_eq!(classify(&spec("multi_steer", 2.1, &["lift_motor"], Some(3.0))).0, "stacker_fork");
        assert_eq!(classify(&spec("diff_drive", 0.6, &["roller_motor"], None)).0, "roller");
        assert_eq!(classify(&spec("diff_drive", 0.6, &["tow_hook"], None)).0, "tugger");
        assert_eq!(classify(&spec("diff_drive", 0.4, &[], None)).0, "none");
        let mut o = spec("diff_drive", 0.4, &[], None);
        o["actuator"] = json!({"type": "roller"});
        assert_eq!(classify(&o).0, "roller");
    }
    #[test]
    fn jacking_assembly_has_joints_and_missing() {
        let a = build(&spec("diff_drive", 0.18, &["motor-lift"], None));
        let j = joint_names(&a);
        for k in ["wl_joint", "wr_joint", "c1_swivel_joint", "c1_wheel_joint", "lift_joint", "tray_rotate_joint"] { assert!(j.contains(&k.to_string()), "{k} missing in {j:?}"); }
        let miss = a["missing"].as_array().unwrap();
        assert!(miss.iter().any(|m| m["item"] == "stroke_m"));
        assert!(miss.iter().any(|m| m["item"] == "c1.position"));
        assert_eq!(a["actuator"]["limit_inputs"]["top"], "di_up_sensor");
    }
    #[test]
    fn all_archetypes_build() {
        for t in ARCHETYPES {
            let mut s = spec("diff_drive", 1.2, &[], None);
            s["actuator"] = json!({"type": t});
            let a = build(&s);
            assert_eq!(a["archetype"], t);
            assert!(a["links"].as_array().unwrap().len() > 3);
        }
    }
}
