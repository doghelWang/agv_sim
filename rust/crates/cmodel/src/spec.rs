//! Port of cmodel_parser.extract_robot_spec: CompDesc JSON -> robot spec (robot_config schema v2).
use serde_json::{json, Map, Value};

pub const SCHEMA_VERSION: i64 = 2;

// ---------------------------------------------------------------- helpers
fn truthy(v: &Value) -> bool {
    match v { Value::Null => false, Value::Bool(b) => *b, Value::Number(n) => n.as_f64().map(|x| x != 0.0).unwrap_or(true), Value::String(s) => !s.is_empty(), Value::Array(a) => !a.is_empty(), Value::Object(o) => !o.is_empty() }
}
/// Python round(x, n) (exact-decimal, half-even) via Rust's exact formatting
fn rd(x: f64, n: usize) -> f64 { if !x.is_finite() { return x; } format!("{:.*}", n, x).parse().unwrap_or(x) }
fn fv(x: f64) -> Value { json!(x) }
fn r4(x: f64) -> Value { json!(rd(x, 4)) }
fn or_f(a: f64, b: f64) -> f64 { if a != 0.0 { a } else { b } }

fn value_of(e: &Value) -> Value {
    let Some(o) = e.as_object() else { return Value::Null };
    for k in ["doubleValue", "floatValue", "int32Value", "int64Value", "intValue", "uint32Value", "boolValue", "stringValue", "stringFix"] {
        if let Some(v) = o.get(k) {
            if k == "int64Value" { if let Some(s) = v.as_str() { if let Ok(i) = s.trim().parse::<i64>() { return json!(i); } } }
            return v.clone();
        }
    }
    if let Some(c) = o.get("comboType") { if truthy(c) { return c.get("typeKey").cloned().unwrap_or(Value::Null); } else { return Value::Null; } }
    Value::Null
}

fn numv(v: &Value, d: f64) -> f64 {
    match v {
        Value::Null => d,
        Value::Bool(b) => if *b { 1.0 } else { 0.0 },
        Value::Number(n) => n.as_f64().unwrap_or(d),
        Value::String(s) => { if s.is_empty() || s == "FIXED_RELATED_NONE" { d } else { s.trim().parse::<f64>().unwrap_or(d) } }
        _ => d,
    }
}
fn txt(v: &Value) -> String { match v { Value::Null => String::new(), Value::String(s) => s.clone(), Value::Bool(b) => if *b { "True".into() } else { "False".into() }, other => other.to_string() } }

// ---------------------------------------------------------------- components
struct Comp {
    group: String, name: String, main_type: String, sub_type: String, dsc_type: String, src_name: String,
    st: Vec<(String, Value)>, attrs: Vec<(String, Value)>, combo: Vec<(String, Vec<(String, Value)>)>, shape: Value,
}

fn vget<'a>(v: &'a [(String, Value)], k: &str) -> Option<&'a Value> { v.iter().find(|x| x.0 == k).map(|x| &x.1) }

impl Comp {
    fn new(raw: &Value, group: &str) -> Comp {
        let gen = raw.get("generalAttr").filter(|g| g.is_object()).cloned().unwrap_or(json!({}));
        let gv = |k: &str| gen.get(k).map(value_of).unwrap_or(Value::Null);
        let name0 = txt(&gv("moduleName"));
        let name = if !name0.is_empty() { name0 } else if !group.is_empty() { group.to_string() } else { "unnamed".to_string() };
        let mut ext: Vec<(String, Value)> = vec![];
        for e in gen.get("extendParams").and_then(|a| a.as_array()).cloned().unwrap_or_default() {
            let k = txt(e.get("key").unwrap_or(&Value::Null)); let v = value_of(&e);
            if let Some(p) = ext.iter_mut().find(|x| x.0 == k) { p.1 = v } else { ext.push((k, v)) }
        }
        let extget = |k: &str| vget(&ext, k).map(txt).unwrap_or_default();
        let mut st: Vec<(String, Value)> = vec![];
        if let Some(a) = raw.get("structParam").and_then(|s| s.get("extendParams")).and_then(|a| a.as_array()) {
            for p in a {
                let k = txt(p.get("key").unwrap_or(&Value::Null)); let v = value_of(p);
                if let Some(x) = st.iter_mut().find(|x| x.0 == k) { x.1 = v } else { st.push((k, v)) }
            }
        }
        let mut attrs: Vec<(String, Value)> = vec![];
        let mut combo: Vec<(String, Vec<(String, Value)>)> = vec![];
        let pa = ["privateAttr", "privateAttrs"].iter().filter_map(|k| raw.get(*k)).find(|v| truthy(v)).cloned().unwrap_or(json!({}));
        for grp in pa.get("privateAttrs").and_then(|a| a.as_array()).cloned().unwrap_or_default() {
            for ele in grp.get("arrayBaseEle").and_then(|a| a.as_array()).cloned().unwrap_or_default() {
                let k = txt(ele.get("key").unwrap_or(&Value::Null));
                let v = value_of(&ele);
                if !attrs.iter().any(|x| x.0 == k) { attrs.push((k.clone(), v)); }
                if let Some(ct) = ele.get("comboType").filter(|c| c.is_object()) {
                    let sel = ct.get("typeKey").cloned().unwrap_or(Value::Null);
                    for tg in ct.get("typeGroups").and_then(|a| a.as_array()).cloned().unwrap_or_default() {
                        if tg.get("key") == Some(&sel) || (tg.get("key").is_none() && sel.is_null()) {
                            if let Some(arr) = tg.get("arrayCmobEle").filter(|a| truthy(a)).and_then(|a| a.as_array()) {
                                let mut m: Vec<(String, Value)> = vec![];
                                for x in arr { let kk = txt(x.get("key").unwrap_or(&Value::Null)); let vv = value_of(x); if let Some(p) = m.iter_mut().find(|p| p.0 == kk) { p.1 = vv } else { m.push((kk, vv)) } }
                                if let Some(p) = combo.iter_mut().find(|p| p.0 == k) { p.1 = m } else { combo.push((k.clone(), m)) }
                            }
                        }
                    }
                }
            }
        }
        Comp { group: group.to_string(), name, main_type: txt(&gv("mainModuleType")), sub_type: txt(&gv("subModuleType")), dsc_type: txt(&gv("moduleDscType")),
            src_name: extget("module_srcname"), st, attrs, combo, shape: gen.get("moduleShape").filter(|s| s.is_object()).cloned().unwrap_or(json!({})) }
    }
    fn get(&self, k: &str) -> Value {
        if let Some(v) = vget(&self.attrs, k) { if !v.is_null() { return v.clone(); } }
        if let Some(v) = vget(&self.st, k) { if !v.is_null() { return v.clone(); } }
        Value::Null
    }
    fn num(&self, k: &str) -> f64 { numv(&self.get(k), 0.0) }
    fn pose(&self) -> [f64; 6] {
        let g = |k: &str| numv(vget(&self.st, k).unwrap_or(&Value::Null), 0.0);
        [g("locCoordX") / 1000.0, g("locCoordY") / 1000.0, g("locCoordZ") / 1000.0, g("locCoordROLL").to_radians(), g("locCoordPITCH").to_radians(), g("locCoordYAW").to_radians()]
    }
    fn box_size(&self) -> Option<[f64; 3]> {
        let b = self.shape.get("box")?; if !b.is_object() || !truthy(b) { return None; }
        let g = |k: &str| numv(b.get(k).unwrap_or(&Value::Null), 0.0) / 1000.0;
        Some([g("sizeLen"), g("sizeWidth"), g("sizeHeight")])
    }
    fn cyl_size(&self) -> Option<[f64; 2]> {
        let c = self.shape.get("cylinder")?; if !c.is_object() || !truthy(c) { return None; }
        let g = |k: &str| numv(c.get(k).unwrap_or(&Value::Null), 0.0) / 1000.0;
        Some([g("diameter"), g("height")])
    }
}

fn walk(groups: &Value, parent: &str, out: &mut Vec<Comp>) {
    for g in groups.as_array().cloned().unwrap_or_default() {
        let gn = match g.get("moduleGroupName") { Some(Value::Object(o)) => txt(o.get("stringValue").unwrap_or(&json!(""))), Some(v) => txt(v), None => String::new() };
        let eff = if !gn.is_empty() { gn.clone() } else { parent.to_string() };
        let comps = ["moduleComponets", "moduleComponents"].iter().filter_map(|k| g.get(*k)).find(|v| truthy(v)).cloned().unwrap_or(json!([]));
        for c in comps.as_array().cloned().unwrap_or_default() { out.push(Comp::new(&c, &eff)); }
        walk(g.get("moreModuleInfo").unwrap_or(&Value::Null), &eff, out);
    }
}

fn load_pair(c: &Comp, idle: &str, full: &str, load: &str) -> f64 {
    let (i, f) = (c.num(idle), c.num(full));
    if load == "full" { if f > 0.0 { f } else { i } } else if i > 0.0 { i } else { f }
}

fn motor_info(c: Option<&Comp>) -> Value {
    let Some(c) = c else { return Value::Null };
    let rpm = or_f(c.num("RPM"), c.num("ratedSpeed"));
    let hb = c.get("bHbrake");
    json!({"name": c.name, "model": if !c.src_name.is_empty() { c.src_name.clone() } else { c.dsc_type.clone() }, "rated_rpm": rpm, "gear_ratio": c.num("gearRatio"),
        "reverse": c.get("bReverse") == json!(true), "holding_brake": hb == json!(true) || hb == json!("WITH_HBRAKE"), "encoder": txt(&c.get("ENCType"))})
}

// ---------------------------------------------------------------- lidar library
fn mid360s() -> Value {
    json!({"type": "3d", "vendor_model": "Livox Mid-360S", "fov_deg": 360.0, "vfov_min_deg": -7.0, "vfov_max_deg": 52.0, "min_range": 0.1, "max_range": 40.0, "range_cutoff_m": 100.0,
        "point_rate": 200000, "freq_hz": 10.0, "range_noise_std": 0.02, "angular_noise_deg": 0.15, "lines": 4, "scan_pattern": "non_repetitive", "imu": "ICM40609",
        "resolution_deg": 0.5, "housing_box": [0.065, 0.065, 0.060], "topic": "/livox/lidar", "imu_topic": "/livox/imu"})
}
fn lidar_library() -> Vec<(&'static str, Value)> {
    let mut m360 = mid360s(); m360["vendor_model"] = json!("Livox Mid-360");
    let pf = json!({"fov_deg": 360.0, "min_range": 0.1, "max_range": 30.0, "resolution_deg": 0.25, "freq_hz": 20.0, "range_noise_std": 0.012, "vendor_model": "Pepperl+Fuchs R2000"});
    vec![("mid-360s", mid360s()), ("mid360s", mid360s()), ("mid-360", m360.clone()), ("mid360", m360), ("pepperl", pf.clone()), ("r2000", pf),
        ("mr-ls-07h", json!({"fov_deg": 270.0, "min_range": 0.05, "max_range": 20.0, "resolution_deg": 0.33, "freq_hz": 25.0, "range_noise_std": 0.02, "vendor_model": "MR-LS-07H 安全激光"})),
        ("ls-05h", json!({"fov_deg": 270.0, "min_range": 0.05, "max_range": 15.0, "resolution_deg": 0.33, "freq_hz": 25.0, "range_noise_std": 0.02, "vendor_model": "MR-LS-05H 激光"})),
        ("sick", json!({"fov_deg": 270.0, "min_range": 0.05, "max_range": 25.0, "resolution_deg": 0.25, "freq_hz": 25.0, "range_noise_std": 0.015, "vendor_model": "SICK"})),
        ("nanoscan", json!({"fov_deg": 275.0, "min_range": 0.05, "max_range": 30.0, "resolution_deg": 0.17, "freq_hz": 25.0, "range_noise_std": 0.015, "vendor_model": "SICK nanoScan3"}))]
}
fn lidar_default() -> Value { json!({"fov_deg": 270.0, "min_range": 0.05, "max_range": 25.0, "resolution_deg": 0.25, "freq_hz": 15.0, "range_noise_std": 0.015, "vendor_model": "generic 2D"}) }
fn lidar3d_default() -> Value { json!({"fov_deg": 360.0, "min_range": 0.3, "max_range": 40.0, "resolution_deg": 0.2, "freq_hz": 10.0, "range_noise_std": 0.02, "vertical_fov_deg": 30.0, "vendor_model": "generic 3D"}) }

fn with(mut base: Value, k: &str, v: Value) -> Value { base[k] = v; base }

fn lidar_params(c: &Comp, is3d: bool) -> Value {
    let key = format!("{} {} {}", c.src_name, c.dsc_type, c.name).to_lowercase();
    for (kw, p) in lidar_library() { if key.contains(kw) { return with(p, "spec_source", json!(format!("library:{kw}"))); } }
    with(if is3d { lidar3d_default() } else { lidar_default() }, "spec_source", json!("library:default"))
}

const STEER: [&str; 4] = ["verticalSteerWheel", "horizontalSteerWheel", "steerWheel", "diffSteerWheel"];

fn s2(x: &str) -> String { x.replace([' ', '-'], "_") }

pub fn extract_robot_spec(comp_desc: &Value, model_filename: &str, load: &str, overrides: Option<&Value>) -> Value {
    let mut comps: Vec<Comp> = vec![];
    walk(comp_desc.get("moreModuleInfo").unwrap_or(&Value::Null), "", &mut comps);
    let mut assumptions: Vec<String> = vec![];
    let mut warnings: Vec<String> = vec![];

    let chassis_c = comps.iter().find(|c| c.main_type == "chassis");
    if chassis_c.is_none() { warnings.push("未找到 chassis 模块，使用默认底盘参数".into()); }
    let dummy = Comp::new(&json!({}), "chassis");
    let ch = chassis_c.unwrap_or(&dummy);

    let mut head = load_pair(ch, "headOffset(Idle)", "headOffset (Full Load)", load) / 1000.0;
    let mut tail = load_pair(ch, "tailOffset(Idle)", "tailOffset (Full Load)", load) / 1000.0;
    let mut left = load_pair(ch, "leftOffset(Idle)", "leftOffset (Full Load)", load) / 1000.0;
    let mut right = load_pair(ch, "rightOffset(Idle)", "rightOffset (Full Load)", load) / 1000.0;
    let bx = ch.box_size();
    if head <= 0.0 && tail <= 0.0 {
        let l = match bx { Some(b) if b[0] > 0.0 => b[0], _ => 1.2 };
        head = l / 2.0; tail = l / 2.0;
        assumptions.push("底盘 head/tail offset 缺失，按外形长度对半".into());
    }
    if left <= 0.0 && right <= 0.0 {
        let w = match bx { Some(b) if b[1] > 0.0 => b[1], _ => 0.8 };
        left = w / 2.0; right = w / 2.0;
        assumptions.push("底盘 left/right offset 缺失，按外形宽度对半".into());
    }
    let height = match bx { Some(b) if b[2] > 0.0 => b[2], _ => 0.35 };
    if !matches!(bx, Some(b) if b[2] > 0.0) { assumptions.push("底盘高度缺失，默认 0.35 m".into()); }

    let motion = |ls: &str| -> Value {
        let v = load_pair(ch, "maxSpeed(Idle)", "maxSpeed (Full Load)", ls) / 1000.0;
        let a = load_pair(ch, "maxAcceleration(Idle)", "maxAcceleration (Full Load)", ls) / 1000.0;
        let d = load_pair(ch, "maxDeceleration(Idle)", "maxDeceleration (Full Load)", ls) / 1000.0;
        let w = load_pair(ch, "rotateMaxAngSpeed (Idle)", "rotateMaxAngSpeed (Full Load)", ls).to_radians();
        let wa = load_pair(ch, "rotateMaxAngAcceleration (Idle)", "rotateMaxAngAcceleration (Full Load)", ls).to_radians();
        let wd = load_pair(ch, "rotateMaxAngDeceleration (Idle)", "rotateMaxAngDeceleration (Full Load)", ls).to_radians();
        let ad = load_pair(ch, "avoidMaxDec (Idle)", "avoidMaxDec (Full Load)", ls) / 1000.0;
        json!({"max_speed_mps": rd(or_f(v, 1.0), 4), "max_accel_mps2": rd(or_f(a, 0.5), 4), "max_decel_mps2": rd(or_f(d, or_f(a, 0.5)), 4),
            "max_ang_speed_radps": rd(or_f(w, 1.0), 4), "max_ang_accel_radps2": rd(or_f(wa, 1.0), 4), "max_ang_decel_radps2": rd(or_f(wd, or_f(wa, 1.0)), 4),
            "avoid_max_decel_mps2": rd(or_f(ad, or_f(d, or_f(a, 0.5))), 4)})
    };
    let motion_idle = motion("idle");
    let motion_full = motion("full");
    let active = if load == "full" { motion_full.clone() } else { motion_idle.clone() };

    // motors (ordered, later duplicates overwrite)
    let mut motors: Vec<(String, usize)> = vec![];
    for (i, c) in comps.iter().enumerate() {
        if c.main_type == "driver" && ["PMSMMotor", "BDCMotor", "stepMotor", "motor"].contains(&c.sub_type.as_str()) {
            if let Some(p) = motors.iter_mut().find(|m| m.0 == c.name) { p.1 = i } else { motors.push((c.name.clone(), i)) }
        }
    }
    let motor = |key: &Value| -> Option<&Comp> { let k = txt(key); motors.iter().find(|m| m.0 == k).map(|m| &comps[m.1]) };

    let mut wheels: Vec<Map<String, Value>> = vec![];
    for c in &comps {
        if c.main_type != "driveWheel" && !(c.main_type == "driver" && c.num("wheelRadius") > 0.0) { continue; }
        let p = c.pose();
        let r = or_f(c.num("wheelRadius") / 1000.0, 0.1);
        let sub = c.sub_type.as_str();
        let mut w = Map::new();
        w.insert("name".into(), json!(s2(&c.name))); w.insert("cmodel_name".into(), json!(c.name)); w.insert("cmodel_subtype".into(), json!(sub));
        w.insert("x".into(), r4(p[0])); w.insert("y".into(), r4(p[1])); w.insert("z".into(), r4(r)); w.insert("radius_m".into(), r4(r));
        w.insert("width_m".into(), r4(f64::max(0.04, r * 0.6)));
        if STEER.contains(&sub) {
            w.insert("kind".into(), json!("steer"));
            let mut lp = c.num("angleLmtPos"); let mut ln = c.num("angleLmtNeg");
            if lp <= 0.0 && ln >= 0.0 { lp = 90.0; ln = -90.0; assumptions.push(format!("{}: 舵角限位缺失，默认 ±90°", c.name)); }
            w.insert("steer_min_rad".into(), json!(rd(ln.to_radians(), 5))); w.insert("steer_max_rad".into(), json!(rd(lp.to_radians(), 5)));
            let walk_m = motor(&c.get("relateWalkMotor")); let rot_m = motor(&c.get("relateRotMotor"));
            let lm = motor(&c.get("relateLeftMotor")); let rm = motor(&c.get("relateRightMotor"));
            if sub == "diffSteerWheel" {
                let ws = or_f(c.num("wheelSpace") / 1000.0, 0.25);
                w.insert("diff_steer_wheel_space_m".into(), r4(ws));
                w.insert("drive_motor".into(), motor_info(lm.or(rm)));
                w.insert("drive_motor_right".into(), motor_info(rm));
            } else {
                w.insert("drive_motor".into(), motor_info(walk_m));
                w.insert("steer_motor".into(), motor_info(rot_m));
            }
            let dm = w["drive_motor"].clone();
            if dm.is_object() && dm["rated_rpm"].as_f64().unwrap_or(0.0) > 0.0 && dm["gear_ratio"].as_f64().unwrap_or(0.0) > 0.0 {
                w.insert("max_speed_mps".into(), json!(rd(dm["rated_rpm"].as_f64().unwrap() / dm["gear_ratio"].as_f64().unwrap() * 2.0 * std::f64::consts::PI / 60.0 * r, 4)));
            }
            let rate = c.num("rotOmgLmt");
            let sm = w.get("steer_motor").cloned().unwrap_or(Value::Null);
            if rate > 0.0 {
                w.insert("steer_rate_radps".into(), json!(rd(rate.to_radians(), 4)));
            } else if sm.is_object() && sm["rated_rpm"].as_f64().unwrap_or(0.0) > 0.0 && sm["gear_ratio"].as_f64().unwrap_or(0.0) > 0.0 {
                w.insert("steer_rate_radps".into(), json!(rd(sm["rated_rpm"].as_f64().unwrap() / sm["gear_ratio"].as_f64().unwrap() * 2.0 * std::f64::consts::PI / 60.0, 4)));
            } else if sub == "diffSteerWheel" && w.get("max_speed_mps").and_then(|v| v.as_f64()).map(|v| v != 0.0).unwrap_or(false) {
                let ms = w["max_speed_mps"].as_f64().unwrap(); let sp = w["diff_steer_wheel_space_m"].as_f64().unwrap();
                w.insert("steer_rate_radps".into(), json!(rd(2.0 * ms * 0.3 / sp, 4)));
                assumptions.push(format!("{}: 差速舵轮转向速率按 30% 差速能力估算", c.name));
            } else {
                w.insert("steer_rate_radps".into(), json!(rd(90f64.to_radians(), 4)));
                assumptions.push(format!("{}: 转向速率缺失，默认 90°/s", c.name));
            }
            let asens = c.get("angleSensorType");
            if truthy(&asens) {
                w.insert("angle_sensor".into(), asens);
                if let Some(cc) = c.combo.iter().find(|x| x.0 == "angleSensorType").filter(|x| !x.1.is_empty()) {
                    let mut m = Map::new(); for (k, v) in &cc.1 { m.insert(k.clone(), v.clone()); }
                    w.insert("angle_sensor_params".into(), Value::Object(m));
                }
            }
        } else {
            w.insert("kind".into(), json!("drive"));
            let m = motor(&c.get("relateMotor"));
            w.insert("drive_motor".into(), motor_info(m));
            if m.is_some() {
                let mi = w["drive_motor"].clone();
                if mi["rated_rpm"].as_f64().unwrap_or(0.0) > 0.0 && mi["gear_ratio"].as_f64().unwrap_or(0.0) > 0.0 {
                    w.insert("max_speed_mps".into(), json!(rd(mi["rated_rpm"].as_f64().unwrap() / mi["gear_ratio"].as_f64().unwrap() * 2.0 * std::f64::consts::PI / 60.0 * r, 4)));
                }
            }
        }
        wheels.push(w);
    }

    for w in wheels.iter_mut() {
        if w["drive_motor"].is_null() && w["kind"] == "drive" {
            let side = if w["y"].as_f64().unwrap_or(0.0) > 0.0 { "left" } else { "right" };
            let cand = motors.iter().find(|(n, _)| { let l = n.to_lowercase(); l.contains(side) || (side == "left" && l.contains("lft")) });
            if let Some((_, i)) = cand {
                let mi = motor_info(Some(&comps[*i]));
                w.insert("drive_motor".into(), mi.clone());
                if mi["rated_rpm"].as_f64().unwrap_or(0.0) > 0.0 && mi["gear_ratio"].as_f64().unwrap_or(0.0) > 0.0 {
                    w.insert("max_speed_mps".into(), json!(rd(mi["rated_rpm"].as_f64().unwrap() / mi["gear_ratio"].as_f64().unwrap() * 2.0 * std::f64::consts::PI / 60.0 * w["radius_m"].as_f64().unwrap(), 4)));
                }
                assumptions.push(format!("{}: 按名称匹配行走电机 {}", txt(&w["cmodel_name"]), txt(&mi["name"])));
            }
        }
    }

    let steer_n = wheels.iter().filter(|w| w["kind"] == "steer").count();
    let drive_n = wheels.iter().filter(|w| w["kind"] == "drive").count();
    let ch_sub = ch.sub_type.as_str();
    let mut ctype = if steer_n == 1 && drive_n == 0 { "single_steer" } else if steer_n >= 2 { if steer_n == 2 { "dual_steer" } else { "multi_steer" } }
        else if drive_n >= 2 { "diff_drive" } else if ch_sub == "diffChassis" { "diff_drive" } else if ch_sub == "steerChassis" { "single_steer" } else { "diff_drive" };

    if ctype == "dual_steer" {
        let st: Vec<usize> = (0..wheels.len()).filter(|i| wheels[*i]["kind"] == "steer").collect();
        let f = |i: usize, k: &str| wheels[i][k].as_f64().unwrap_or(0.0);
        if st.iter().all(|i| wheels[*i]["cmodel_subtype"] == "horizontalSteerWheel") && (f(st[0], "x") - f(st[1], "x")).abs() < 1e-3 && (f(st[0], "y") + f(st[1], "y")).abs() < 1e-3
            && wheels[st[0]].contains_key("steer_max_rad") && wheels[st[0]].get("steer_motor").map(|v| v.is_null()).unwrap_or(true) && !wheels[st[0]].get("angle_sensor").map(truthy).unwrap_or(false) {
            for i in st { let w = &mut wheels[i]; w.insert("kind".into(), json!("drive")); for k in ["steer_min_rad", "steer_max_rad", "steer_rate_radps"] { w.shift_remove(k); } }
            ctype = "diff_drive";
            assumptions.push("两个卧式舵轮左右对称且无转向电机/角度传感器 → 按差速驱动处理".into());
        }
    }

    if wheels.is_empty() {
        let wheel_space = or_f(ch.num("wheelSpace") / 1000.0, (left + right) * 0.8);
        let r = 0.1;
        if ctype == "diff_drive" {
            for (side, y) in [("left", wheel_space / 2.0), ("right", -wheel_space / 2.0)] {
                let mut w = Map::new();
                for (k, v) in [("name", json!(format!("{side}_wheel"))), ("kind", json!("drive")), ("x", fv(0.0)), ("y", r4(y)), ("z", fv(r)), ("radius_m", fv(r)), ("width_m", fv(0.05)), ("source", json!("inferred"))] { w.insert(k.into(), v); }
                wheels.push(w);
            }
        } else {
            let mut w = Map::new();
            for (k, v) in [("name", json!("steer_wheel")), ("kind", json!("steer")), ("x", r4(head * 0.8)), ("y", fv(0.0)), ("z", fv(r)), ("radius_m", fv(r)), ("width_m", fv(0.06)),
                ("steer_min_rad", fv(-std::f64::consts::FRAC_PI_2)), ("steer_max_rad", fv(std::f64::consts::FRAC_PI_2)), ("steer_rate_radps", fv(90f64.to_radians())), ("source", json!("inferred"))] { w.insert(k.into(), v); }
            wheels.push(w);
        }
        assumptions.push("cmodel 无轮组模块，按底盘类型推断轮组布局".into());
    }

    let ws_cfg = ch.num("wheelSpace") / 1000.0;
    let dys: Vec<f64> = wheels.iter().filter(|w| w["kind"] == "drive").map(|w| w["y"].as_f64().unwrap_or(0.0)).collect();
    if ctype == "diff_drive" && dys.len() >= 2 {
        let track = dys.iter().cloned().fold(f64::MIN, f64::max) - dys.iter().cloned().fold(f64::MAX, f64::min);
        if ws_cfg > 0.0 && (ws_cfg - track).abs() > 0.01 { warnings.push(format!("底盘 wheelSpace={ws_cfg:.3}m 与轮子坐标轮距 {track:.3}m 不一致，以轮子坐标为准")); }
    }

    // passive wheels
    let mut passive: Vec<Map<String, Value>> = vec![];
    let margin = 0.12;
    let rmin = wheels.iter().map(|w| w["radius_m"].as_f64().unwrap_or(0.1)).fold(f64::MAX, f64::min);
    let mk = |pairs: Vec<(&str, Value)>| -> Map<String, Value> { let mut m = Map::new(); for (k, v) in pairs { m.insert(k.into(), v); } m };
    if ctype == "single_steer" {
        let sw = wheels.iter().find(|w| w["kind"] == "steer").unwrap_or(&wheels[0]);
        let rr = sw["radius_m"].as_f64().unwrap_or(0.1) * 0.75;
        for (side, y) in [("left", left - margin), ("right", -(right - margin))] {
            passive.push(mk(vec![("name", json!(format!("rear_{side}_load_wheel"))), ("kind", json!("fixed")), ("x", fv(0.0)), ("y", r4(y)), ("z", r4(rr)), ("radius_m", r4(rr)), ("width_m", fv(0.08)), ("source", json!("inferred"))]));
        }
        assumptions.push("单舵轮: 在运动中心(后桥)两侧推断 2 个定向承重轮".into());
    } else if ctype == "diff_drive" {
        let cr = rmin * 0.6;
        for (tag, x) in [("front", head - margin - cr), ("rear", -(tail - margin - cr))] {
            for (side, y) in [("left", left - margin - cr), ("right", -(right - margin - cr))] {
                passive.push(mk(vec![("name", json!(format!("caster_{tag}_{side}"))), ("kind", json!("caster")), ("x", r4(x)), ("y", r4(y)), ("z", r4(cr)), ("radius_m", r4(cr)), ("source", json!("inferred"))]));
            }
        }
        assumptions.push("差速: 在车体四角推断 4 个万向脚轮".into());
    } else {
        let cr = rmin * 0.7;
        let corners = [(head - margin - cr, left - margin - cr), (head - margin - cr, -(right - margin - cr)), (-(tail - margin - cr), left - margin - cr), (-(tail - margin - cr), -(right - margin - cr))];
        for (i, (x, y)) in corners.iter().enumerate() {
            if wheels.iter().all(|w| (x - w["x"].as_f64().unwrap_or(0.0)).hypot(y - w["y"].as_f64().unwrap_or(0.0)) > 0.35) {
                passive.push(mk(vec![("name", json!(format!("caster_{i}"))), ("kind", json!("caster")), ("x", r4(*x)), ("y", r4(*y)), ("z", r4(cr)), ("radius_m", r4(cr)), ("source", json!("inferred"))]));
            }
        }
        if !passive.is_empty() { assumptions.push(format!("舵轮底盘: 推断 {} 个万向承重脚轮", passive.len())); }
    }
    let wheels_all: Vec<Value> = wheels.iter().chain(passive.iter()).map(|m| Value::Object(m.clone())).collect();

    let caps: Vec<f64> = wheels.iter().filter_map(|w| w.get("max_speed_mps").and_then(|v| v.as_f64()).filter(|v| *v != 0.0)).collect();
    if !caps.is_empty() {
        let mn = caps.iter().cloned().fold(f64::MAX, f64::min);
        let ms = active["max_speed_mps"].as_f64().unwrap_or(0.0);
        if mn + 1e-3 < ms { warnings.push(format!("电机额定转速/减速比限制轮端最高 {mn:.3} m/s，低于底盘设定 {ms:.3} m/s；仿真按轮端能力限速")); }
    }

    // sensors
    let mut lidars: Vec<Value> = vec![]; let mut cameras: Vec<Value> = vec![]; let mut io_in: Vec<Value> = vec![]; let mut io_out: Vec<Value> = vec![]; let mut imus: Vec<Value> = vec![];
    for c in &comps {
        if c.main_type != "sensor" { continue; }
        let p = c.pose(); let nm = s2(&c.name);
        let sub = c.sub_type.as_str();
        let model = if !c.src_name.is_empty() { c.src_name.clone() } else { c.dsc_type.clone() };
        if ["laser", "3DLaser", "lidar", "safetyLaser"].contains(&sub) {
            let is3d = sub == "3DLaser";
            let prm = lidar_params(c, is3d);
            let fov = prm["fov_deg"].as_f64().unwrap_or(270.0);
            let sd = c.get("scanDirect"); let um = c.get("usageMode");
            let mut li = json!({"name": nm, "cmodel_name": c.name, "type": if is3d { "3d" } else { "2d" }, "model": model,
                "x": r4(p[0]), "y": r4(p[1]), "z": r4(p[2]), "roll": rd(p[3], 6), "pitch": rd(p[4], 6), "yaw": rd(p[5], 6),
                "inverted": p[3].cos().abs() > 0.5 && p[3].cos() < 0.0,
                "scan_direction": if truthy(&sd) { sd } else { json!("SCAN_COUNTERCLOCKWISE") }, "usage": if truthy(&um) { um } else { json!("") },
                "wavelength_nm": c.num("waveLength"), "anti_light_klux": c.num("antiLight"),
                "angle_min": rd(-fov.to_radians() / 2.0, 6), "angle_max": rd(fov.to_radians() / 2.0, 6)});
            for (k, v) in prm.as_object().unwrap() { li[k.as_str()] = v.clone(); }
            li["housing"] = match (c.cyl_size(), c.box_size()) { (Some(cy), _) => json!({"cylinder_d_h": cy}), (None, Some(b)) => json!({"box": b}), _ => json!({}) };
            if is3d { assumptions.push(format!("{}: 3D 激光在 2D 仿真中按其安装高度水平切片输出", c.name)); }
            lidars.push(li);
        } else if ["codeReader", "camera", "depthCamera"].contains(&sub) {
            let lo = c.get("lensOrientation"); let lo_s = if truthy(&lo) { txt(&lo) } else { "LENS_DIR_FRONT".into() };
            let lo_raw = if truthy(&lo) { txt(&lo) } else { String::new() };
            let sd = c.num("scanDistence") / 1000.0; let acc = c.num("accuracy");
            cameras.push(json!({"name": nm, "cmodel_name": c.name, "type": sub, "model": model, "orientation": lo_s,
                "x": r4(p[0]), "y": r4(p[1]), "z": r4(p[2]), "roll": rd(p[3], 6), "pitch": rd(p[4], 6), "yaw": rd(p[5], 6),
                "read_window_m": if lo_raw.ends_with("DOWN") { 0.05 } else { 0.08 },
                "scan_distance_m": if sd != 0.0 { json!(sd) } else { Value::Null }, "accuracy_mm": if acc != 0.0 { json!(acc) } else { Value::Null }}));
        } else if sub == "gyro" {
            imus.push(json!({"name": nm, "x": r4(p[0]), "y": r4(p[1]), "z": r4(p[2]), "angular_resolution": c.num("angularResolution"), "source": "sensor/gyro"}));
        } else if ["proximitySensor", "PT", "photoelectric"].contains(&sub) {
            io_in.push(json!({"name": nm, "type": sub, "cmodel_name": c.name, "x": r4(p[0]), "y": r4(p[1]), "z": r4(p[2]), "yaw": rd(p[5], 6)}));
        } else if sub == "comDo" {
            io_out.push(json!({"name": nm, "type": sub}));
        }
    }
    for c in &comps {
        let g = c.get("isWithGyro");
        if c.main_type == "mainCPU" && (g == json!("yes") || g == json!(true)) && imus.is_empty() {
            let p = c.pose();
            imus.push(json!({"name": "imu", "x": r4(p[0]), "y": r4(p[1]), "z": rd(or_f(rd(p[2], 4), 0.1), 4), "source": format!("mainCPU:{} (内置陀螺)", c.name)}));
        }
    }
    if imus.is_empty() { imus.push(json!({"name": "imu", "x": 0.0, "y": 0.0, "z": 0.1, "source": "default"})); }
    if lidars.is_empty() {
        let prm = with(lidar_default(), "spec_source", json!("library:default"));
        let fov = prm["fov_deg"].as_f64().unwrap();
        let mut li = json!({"name": "laser_front", "type": "2d", "model": "", "x": r4(head * 0.9), "y": 0.0, "z": 0.25, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "inverted": false,
            "angle_min": -fov.to_radians() / 2.0, "angle_max": fov.to_radians() / 2.0});
        for (k, v) in prm.as_object().unwrap() { li[k.as_str()] = v.clone(); }
        lidars.push(li);
        assumptions.push("cmodel 无激光模块，默认车头 1 个 270° 激光".into());
    }

    let buttons: Vec<Value> = comps.iter().filter(|c| c.main_type == "button").map(|c| { let t = c.get("buttonType"); json!({"name": c.name, "type": if truthy(&t) { t } else { json!("") }, "self_lock": truthy(&c.get("selfLock"))}) }).collect();
    let battery = comps.iter().find(|c| c.main_type == "battery").map(|c| json!({"name": c.name, "voltage_v": c.num("voltage"), "capacity_ah": c.num("capacity"),
        "max_discharge_a": c.num("maxDischargeCurrent"), "std_discharge_a": c.num("stdDischargeCurrent"), "max_power_w": c.num("maxPower")})).unwrap_or(Value::Null);
    let mut used: Vec<String> = vec![];
    for w in &wheels { for k in ["drive_motor", "drive_motor_right", "steer_motor"] { if let Some(m) = w.get(k).filter(|m| m.is_object()) { used.push(txt(&m["name"])); } } }
    let actuators: Vec<Value> = motors.iter().filter(|(n, _)| !used.contains(n)).map(|(_, i)| motor_info(Some(&comps[*i]))).collect();

    let self_weight = ch.num("selfWeight"); let load_weight = ch.num("totalLoadWeight");
    let mass = if self_weight > 0.0 { self_weight } else { rd(f64::max(80.0, (head + tail) * (left + right) * 150.0), 1) };
    if self_weight <= 0.0 { assumptions.push(format!("自重缺失，按投影面积估算 {} kg", py_float(mass))); }

    let footprint = json!([[rd(head, 4), rd(left, 4)], [rd(head, 4), rd(-right, 4)], [rd(-tail, 4), rd(-right, 4)], [rd(-tail, 4), rd(left, 4)]]);
    let drive_ref = wheels.iter().find(|w| w["kind"] == "drive" || w["kind"] == "steer").unwrap_or(&wheels[0]);
    let dws: Vec<f64> = wheels.iter().filter(|w| w["kind"] == "drive").map(|w| w["y"].as_f64().unwrap_or(0.0)).collect();
    let legacy_track = if dws.len() >= 2 { dws.iter().cloned().fold(f64::MIN, f64::max) - dws.iter().cloned().fold(f64::MAX, f64::min) } else { (left + right) * 0.8 };

    let mut chassis = json!({"type": ctype, "cmodel_type": ch_sub, "cmodel_name": ch.name, "length_m": rd(head + tail, 4), "width_m": rd(left + right, 4), "height_m": rd(height, 4),
        "head_offset_m": rd(head, 4), "tail_offset_m": rd(tail, 4), "left_offset_m": rd(left, 4), "right_offset_m": rd(right, 4), "footprint": footprint,
        "rotate_diameter_m": rd(ch.num("rotateDiameter") / 1000.0, 4), "max_climb_deg": ch.num("maxClimbingAngle"), "self_weight_kg": self_weight, "mass_kg": mass, "max_load_kg": load_weight});
    for (k, v) in active.as_object().unwrap() { chassis[k.as_str()] = v.clone(); }
    chassis["motion_idle"] = motion_idle; chassis["motion_full_load"] = motion_full;

    let mut spec = json!({"schema_version": SCHEMA_VERSION, "model_file": model_filename, "robot_name": "cmodel_agv", "load_state": load, "chassis": chassis, "wheels": wheels_all,
        "drive_wheels": {"radius_m": drive_ref["radius_m"], "track_width_m": rd(legacy_track, 4), "wheel_width_m": drive_ref.get("width_m").cloned().unwrap_or(json!(0.05)), "x_offset_m": drive_ref["x"]},
        "lidars": lidars, "cameras": cameras, "imu": imus[0].clone(), "io": {"inputs": io_in, "outputs": io_out, "buttons": buttons}, "battery": battery,
        "other_actuators": actuators, "assumptions": assumptions, "warnings": warnings});
    if let Some(ov) = overrides { apply_overrides(&mut spec, ov); }
    let _ = &comps.iter().map(|c| &c.group).count();
    spec
}

fn py_float(x: f64) -> String { if x.fract() == 0.0 && x.abs() < 1e16 { format!("{x:.1}") } else { format!("{x}") } }

fn apply_overrides(spec: &mut Value, ov: &Value) {
    let mut ov = ov.clone();
    if let Some(l) = ov.get("lidars").and_then(|l| l.as_object()).cloned() {
        for (name, patch) in l {
            let lib = txt(patch.get("library").unwrap_or(&Value::Null)).to_lowercase();
            if !lib.is_empty() {
                if let Some((_, base)) = lidar_library().into_iter().find(|(kw, _)| *kw == lib) {
                    let mut full = with(base, "spec_source", json!(format!("override:{lib}")));
                    for (k, v) in patch.as_object().unwrap() { if k != "library" { full[k.as_str()] = v.clone(); } }
                    ov["lidars"][name.as_str()] = full;
                }
            }
        }
    }
    if let Some(c) = ov.get("chassis").and_then(|c| c.as_object()) { for (k, v) in c { spec["chassis"][k.as_str()] = v.clone(); } }
    for sec in ["lidars", "wheels", "cameras"] {
        if let Some(m) = ov.get(sec).and_then(|m| m.as_object()) {
            for (name, patch) in m {
                if let Some(items) = spec.get_mut(sec).and_then(|a| a.as_array_mut()) {
                    for item in items.iter_mut() {
                        if item.get("name").and_then(|n| n.as_str()) == Some(name) || item.get("cmodel_name").and_then(|n| n.as_str()) == Some(name) {
                            if let Some(p) = patch.as_object() { for (k, v) in p { item[k.as_str()] = v.clone(); } }
                            if sec == "lidars" {
                                let fov = numv(item.get("fov_deg").unwrap_or(&Value::Null), 270.0);
                                item["angle_min"] = json!(rd(-fov.to_radians() / 2.0, 6)); item["angle_max"] = json!(rd(fov.to_radians() / 2.0, 6));
                                if let Some(hb) = item.get("housing_box").filter(|h| truthy(h)).cloned() { item["housing"] = json!({"box": hb}); }
                            }
                        }
                    }
                }
            }
        }
    }
    if let Some(a) = spec["assumptions"].as_array_mut() { a.push(json!("已应用 overrides 覆盖参数")); }
}
