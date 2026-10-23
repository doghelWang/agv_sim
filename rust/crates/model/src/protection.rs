//! Vehicle protection space (port of planning/protection.py)
use crate::{fnum, pf, rnd};
use serde_json::{json, Map, Value};

pub const NAV_MAX_SPEED: f64 = 1.2;
pub const FIELD_KEYS: [&str; 3] = ["front", "rear", "side"];

pub fn defaults() -> Value {
    json!({
        "body_margin": 0.05,
        "payload": {"enabled": false, "head": null, "tail": null, "left": null, "right": null, "note": ""},
        "fields": [
            {"name": "低速", "v_max": 0.3, "front": 0.30, "rear": 0.20, "side": 0.08},
            {"name": "中速", "v_max": 0.8, "front": 0.80, "rear": 0.30, "side": 0.10},
            {"name": "高速", "v_max": 9.0, "front": 1.40, "rear": 0.40, "side": 0.15}],
        "slow_ratio": 2.0, "slow_speed": 0.35, "rotate_margin": 0.02, "rotate_lookahead_rad": 0.25,
        "docking": {"front": 0.02, "arrive_tolerance": 0.25}, "reaction_s": 0.3,
        "photo": {"mode": "field", "front": 0.30, "rear": 0.30, "side": 0.10, "mute_near_stop": 0.10},
        "corner_mode": "auto", "enabled": true
    })
}

fn merge(base: &Value, patch: &Value) -> Value {
    let mut out = base.clone();
    if let Some(p) = patch.as_object() {
        for (k, v) in p {
            if v.is_object() && out.get(k).map(|o| o.is_object()).unwrap_or(false) { out[k] = merge(&out[k], v); } else { out[k] = v.clone(); }
        }
    }
    out
}

pub fn braking_distance(v: f64, decel: f64, reaction_s: f64) -> f64 { v * reaction_s + v * v / (2.0 * decel.max(0.05)) }

pub fn default_fields(ch: &Value, reaction_s: f64) -> Vec<Value> {
    let dec = fnum(ch.get("max_decel_mps2").or_else(|| ch.get("max_accel_mps2")), 0.5);
    let vtop = NAV_MAX_SPEED.min(fnum(ch.get("max_speed_mps"), 1.5));
    let up = |d: f64| ((d + 0.1) * 20.0).ceil() / 20.0;
    let mut out: Vec<Value> = vec![];
    for (name, v, side) in [("低速", 0.3f64.min(vtop), 0.08), ("中速", 0.8f64.min(vtop), 0.10), ("高速", vtop, 0.15)] {
        if let Some(l) = out.last() { if v <= l["v_max"].as_f64().unwrap() + 1e-6 { continue; } }
        out.push(json!({"name": name, "v_max": rnd(v, 2), "front": up(braking_distance(v, dec, reaction_s)), "rear": up(braking_distance(v.min(0.5), dec, reaction_s)), "side": side}));
    }
    out
}

/// spec (with "chassis") or a protection fragment → complete configuration
pub fn effective(spec_or_prot: Option<&Value>, ch: Option<&Value>) -> Value {
    let empty = json!({});
    let mut raw = spec_or_prot.unwrap_or(&empty);
    let mut ch = ch;
    let prot;
    if raw.get("chassis").is_some() {
        if ch.is_none() { ch = raw.get("chassis"); }
        prot = raw.get("protection").filter(|p| !p.is_null()).cloned().unwrap_or(json!({}));
        raw = &prot;
    }
    let mut p = merge(&defaults(), raw);
    let raw_fields = raw.get("fields").and_then(|f| f.as_array()).filter(|f| !f.is_empty());
    if let Some(f) = raw_fields { p["fields"] = Value::Array(f.clone()); }
    else if let Some(c) = ch { p["fields"] = Value::Array(default_fields(c, fnum(p.get("reaction_s"), 0.3))); p["fields_auto"] = json!(true); }
    let src: Vec<Value> = match p["fields"].as_array() { Some(a) if !a.is_empty() => a.clone(), _ => defaults()["fields"].as_array().unwrap().clone() };
    let mut fs: Vec<Map<String, Value>> = vec![];
    for (i, f) in src.iter().enumerate() {
        let mut f = f.as_object().cloned().unwrap_or_default();
        if !f.contains_key("name") { f.insert("name".into(), json!(format!("档{}", i + 1))); }
        let vm = fnum(f.get("v_max"), 9.0);
        f.insert("v_max".into(), json!(vm));
        for k in FIELD_KEYS { let v = fnum_or0(f.get(k)).max(0.0); f.insert(k.into(), json!(v)); }
        fs.push(f);
    }
    fs.sort_by(|a, b| a["v_max"].as_f64().unwrap().partial_cmp(&b["v_max"].as_f64().unwrap()).unwrap());
    if let Some(l) = fs.last_mut() { let v = l["v_max"].as_f64().unwrap().max(9.0); l.insert("v_max".into(), json!(v)); }
    p["fields"] = Value::Array(fs.into_iter().map(Value::Object).collect());
    for k in ["body_margin", "slow_ratio", "slow_speed", "rotate_margin", "rotate_lookahead_rad", "reaction_s"] { let v = fnum_or0(p.get(k)).max(0.0); p[k] = json!(v); }
    p["slow_ratio"] = json!(p["slow_ratio"].as_f64().unwrap().max(1.0));
    if let Some(c) = ch {
        for (k, ck) in [("head", "head_offset_m"), ("tail", "tail_offset_m"), ("left", "left_offset_m"), ("right", "right_offset_m")] {
            if p["payload"].get(k).map(|v| v.is_null()).unwrap_or(true) { p["payload"][k] = json!(fnum(c.get(ck), 0.5)); }
        }
    }
    p
}
fn fnum_or0(v: Option<&Value>) -> f64 { crate::fnum(v, 0.0) }

pub fn body_box(ch: &Value) -> [f64; 4] {
    [fnum(ch.get("head_offset_m"), 0.6), fnum(ch.get("tail_offset_m"), 0.6), fnum(ch.get("left_offset_m"), 0.4), fnum(ch.get("right_offset_m"), 0.4)]
}
pub fn outline(ch: &Value, p: &Value, loaded: bool) -> [f64; 4] {
    let b = body_box(ch);
    let pl = p.get("payload").cloned().unwrap_or(json!({}));
    if !(loaded && pl.get("enabled").map(crate::overrides::truthy).unwrap_or(false)) { return b; }
    let mut o = b;
    for (i, k) in ["head", "tail", "left", "right"].iter().enumerate() {
        let v = match pl.get(*k) { Some(v) if !v.is_null() => fnum(Some(v), b[i]), _ => b[i] };
        o[i] = b[i].max(v);
    }
    o
}
pub fn nav2_outline(ch: &Value, p: &Value) -> [f64; 4] { outline(ch, p, true) }
pub fn rect(h: f64, t: f64, l: f64, r: f64) -> Value { json!([[h, l], [h, -r], [-t, -r], [-t, l]]) }

pub fn field_for_speed(p: &Value, v: f64) -> (usize, Value) {
    let v = v.abs();
    let fs = p["fields"].as_array().unwrap();
    for (i, f) in fs.iter().enumerate() { if v <= f["v_max"].as_f64().unwrap() + 1e-6 { return (i, f.clone()); } }
    (fs.len() - 1, fs.last().unwrap().clone())
}

pub fn rotate_lookahead(p: &Value, w: f64, ang_decel: f64, rot_left: f64) -> f64 {
    let w = w.abs();
    let mut look = p["rotate_lookahead_rad"].as_f64().unwrap() + w * p["reaction_s"].as_f64().unwrap() + w * w / (2.0 * ang_decel.max(0.1));
    if rot_left >= 0.0 { look = look.min(rot_left + p["rotate_lookahead_rad"].as_f64().unwrap()); }
    look.min(std::f64::consts::PI)
}

pub fn photo_envelope(ch: &Value, p: &Value, loaded: bool, field_idx: usize) -> Option<[f64; 4]> {
    let ph = p.get("photo").cloned().unwrap_or(json!({}));
    let mode = ph.get("mode").and_then(|m| m.as_str()).unwrap_or("field");
    if mode == "always" { return None; }
    let [h, t, l, r] = outline(ch, p, loaded);
    let (fr, rr, sd) = if mode == "custom" { (fnum(ph.get("front"), 0.3), fnum(ph.get("rear"), 0.3), fnum(ph.get("side"), 0.1)) } else {
        let fs = p["fields"].as_array().unwrap();
        let f = &fs[field_idx.min(fs.len() - 1)];
        (f["front"].as_f64().unwrap(), f["rear"].as_f64().unwrap(), f["side"].as_f64().unwrap())
    };
    Some([h + fr, t + rr, l + sd, r + sd])
}

pub fn polygons(ch: &Value, p: &Value, loaded: bool, field_idx: Option<usize>) -> Value {
    let [h, t, l, r] = outline(ch, p, loaded);
    let fs = p["fields"].as_array().unwrap();
    let i = field_idx.unwrap_or(0);
    let f = &fs[i.min(fs.len() - 1)];
    let (fr, rr, sd) = (f["front"].as_f64().unwrap(), f["rear"].as_f64().unwrap(), f["side"].as_f64().unwrap());
    let k = p["slow_ratio"].as_f64().unwrap();
    let b = body_box(ch);
    let mut out = json!({"body": rect(b[0], b[1], b[2], b[3]), "outline": rect(h, t, l, r), "stop": rect(h + fr, t + rr, l + sd, r + sd), "slow": rect(h + fr * k, t + rr * k, l + sd, r + sd)});
    let pl = p.get("payload").cloned().unwrap_or(json!({}));
    if pl.get("enabled").map(crate::overrides::truthy).unwrap_or(false) {
        out["payload"] = rect(fnum(pl.get("head"), 0.0), fnum(pl.get("tail"), 0.0), fnum(pl.get("left"), 0.0), fnum(pl.get("right"), 0.0));
    }
    if let Some(e) = photo_envelope(ch, p, loaded, i) { out["photo"] = rect(e[0], e[1], e[2], e[3]); }
    let m = p["rotate_margin"].as_f64().unwrap();
    out["rotate_radius"] = json!(rnd(h.hypot(l.max(r)).max(t.hypot(l.max(r))) + m, 3));
    out
}

fn lidar_sees(lidars: &[Value], p: (f64, f64)) -> bool {
    for l in lidars {
        let (x, y) = (fnum(l.get("x"), 0.0), fnum(l.get("y"), 0.0));
        let (dx, dy) = (p.0 - x, p.1 - y);
        let rng = dx.hypot(dy);
        if rng < fnum(l.get("min_range"), 0.05) || rng > fnum(l.get("max_range"), 20.0) { continue; }
        if l.get("type").and_then(|t| t.as_str()) == Some("3d") { return true; }
        let fov = fnum(l.get("fov_deg"), 270.0).to_radians();
        let inv = if fnum(l.get("roll"), 0.0).cos() < -0.5 { -1.0 } else { 1.0 };
        let a = dy.atan2(dx);
        let yaw = fnum(l.get("yaw"), 0.0);
        let rel = (a - yaw).sin().atan2((a - yaw).cos()) * inv;
        if rel.abs() <= fov / 2.0 + 1e-6 { return true; }
    }
    false
}

pub fn validate(spec: &Value, p: &Value) -> Vec<Value> {
    let ch = spec.get("chassis").cloned().unwrap_or(json!({}));
    let mut out: Vec<Value> = vec![];
    let vmax = NAV_MAX_SPEED.min(fnum(ch.get("max_speed_mps"), 1.5));
    let dec = fnum(ch.get("max_decel_mps2").or_else(|| ch.get("max_accel_mps2")), 0.5);
    let react = p["reaction_s"].as_f64().unwrap();
    let mut prev_v = 0.0;
    for (i, f) in p["fields"].as_array().unwrap().iter().enumerate() {
        let v = f["v_max"].as_f64().unwrap().min(vmax);
        if v <= prev_v && i > 0 { continue; }
        let need = braking_distance(v, dec, react);
        let front = f["front"].as_f64().unwrap();
        let ok = front >= need - 1e-6;
        let name = f["name"].as_str().unwrap_or("");
        out.push(json!({"key": format!("fields.{i}.front"), "label": format!("{name} (≤ {v:.2} m/s) 前向停车距离"), "value": front, "severity": if ok { "ok" } else { "warn" },
            "note": format!("制动距离 {need:.2} m (反应 {} s + 减速度 {} m/s²){}", pf(react), pf(dec), if ok { "" } else { "，防护区不足以停住" })}));
        let rv = v.min(0.5);
        let need_r = braking_distance(rv, dec, react);
        let rear = f["rear"].as_f64().unwrap();
        if rear < need_r - 1e-6 {
            out.push(json!({"key": format!("fields.{i}.rear"), "label": format!("{name} 后向停车距离"), "value": rear, "severity": "info", "note": format!("倒车限速 0.5 m/s 时制动距离 {need_r:.2} m")}));
        }
        prev_v = v;
    }
    let pl = &p["payload"];
    let enabled = pl.get("enabled").map(crate::overrides::truthy).unwrap_or(false);
    if enabled {
        let b = body_box(&ch);
        let big: Vec<&str> = ["head", "tail", "left", "right"].iter().enumerate().filter(|(i, k)| fnum(pl.get(**k), 0.0) > b[*i] + 1e-6).map(|(_, k)| *k).collect();
        out.push(json!({"key": "payload", "label": "带载外形", "value": [pl["head"], pl["tail"], pl["left"], pl["right"]], "severity": "ok",
            "note": format!("顶升到位后生效{}", if big.is_empty() { "，未超出车体".to_string() } else { format!("，超出车体: {}", big.join("/")) })}));
    } else {
        let lift = spec.get("lift").cloned().unwrap_or(json!({}));
        let has = lift.get("enabled").map(crate::overrides::truthy).unwrap_or(false)
            || spec.get("other_actuators").and_then(|a| a.as_array()).map(|a| a.iter().any(|m| m.get("name").and_then(|n| n.as_str()).unwrap_or("").to_lowercase().contains("lift"))).unwrap_or(false);
        if has {
            out.push(json!({"key": "payload", "label": "带载外形", "value": "未配置", "severity": "warn", "note": "车辆有顶升机构，货架/托盘通常比车体大，建议配置带载外形"}));
        }
    }
    let lidars: Vec<Value> = spec.get("lidars").and_then(|l| l.as_array()).cloned().unwrap_or_default();
    if !lidars.is_empty() {
        let [h, t, l, r] = outline(&ch, p, enabled);
        for (side, sign) in [("front", 1.0f64), ("rear", -1.0)] {
            let f = &p["fields"][0];
            let d = if sign > 0.0 { h + f["front"].as_f64().unwrap() } else { -(t + f["rear"].as_f64().unwrap()) };
            let sd = f["side"].as_f64().unwrap();
            let pts: Vec<(f64, f64)> = (0..9).map(|k| (d, -(r + sd) + (l + r + 2.0 * sd) * k as f64 / 8.0)).collect();
            let seen = pts.iter().filter(|q| lidar_sees(&lidars, **q)).count();
            let sev = if seen == pts.len() { "ok" } else if (seen as f64) < pts.len() as f64 * 0.6 { "warn" } else { "info" };
            out.push(json!({"key": format!("coverage.{side}"), "label": format!("{}向防护区激光覆盖", if sign > 0.0 { "前" } else { "后" }), "value": format!("{seen}/{}", pts.len()),
                "severity": sev, "note": format!("低速档停车区边沿采样点被激光视场覆盖的比例{}", if sev == "ok" { "" } else { "，存在盲区" })}));
        }
    }
    out
}
