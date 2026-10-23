//! Vehicle protection space config (port of planning/protection.py).
use serde_json::{json, Map, Value};

pub const NAV_MAX_SPEED: f64 = 1.2;

#[derive(Clone, Debug)]
pub struct Field { pub name: String, pub v_max: f64, pub front: f64, pub rear: f64, pub side: f64 }

#[derive(Clone, Debug)]
pub struct Prot {
    pub json: Value,
    pub enabled: bool,
    pub body_margin: f64,
    pub payload_enabled: bool,
    pub payload: [f64; 4], // head tail left right
    pub fields: Vec<Field>,
    pub slow_ratio: f64,
    pub slow_speed: f64,
    pub rotate_margin: f64,
    pub rotate_lookahead_rad: f64,
    pub docking_front: f64,
    pub arrive_tolerance: f64,
    pub reaction_s: f64,
    pub photo_mode: String,
    pub photo_front: f64,
    pub photo_rear: f64,
    pub photo_side: f64,
    pub photo_mute_near_stop: f64,
    pub corner_mode: String,
}

pub fn defaults() -> Value {
    json!({
        "body_margin": 0.05,
        "payload": {"enabled": false, "head": null, "tail": null, "left": null, "right": null, "note": ""},
        "fields": [
            {"name": "低速", "v_max": 0.3, "front": 0.30, "rear": 0.20, "side": 0.08},
            {"name": "中速", "v_max": 0.8, "front": 0.80, "rear": 0.30, "side": 0.10},
            {"name": "高速", "v_max": 9.0, "front": 1.40, "rear": 0.40, "side": 0.15},
        ],
        "slow_ratio": 2.0, "slow_speed": 0.35, "rotate_margin": 0.02, "rotate_lookahead_rad": 0.25,
        "docking": {"front": 0.02, "arrive_tolerance": 0.25},
        "reaction_s": 0.3,
        "photo": {"mode": "field", "front": 0.30, "rear": 0.30, "side": 0.10, "mute_near_stop": 0.10},
        "corner_mode": "auto",
        "enabled": true,
    })
}

pub fn merge(base: &Value, patch: &Value) -> Value {
    let mut out = base.clone();
    if let (Some(o), Some(p)) = (out.as_object_mut(), patch.as_object()) {
        for (k, v) in p {
            let nv = match (o.get(k), v) {
                (Some(b @ Value::Object(_)), Value::Object(_)) => merge(b, v),
                _ => v.clone(),
            };
            o.insert(k.clone(), nv);
        }
    }
    out
}

fn num(v: Option<&Value>, d: f64) -> f64 {
    match v { Some(Value::Number(n)) => n.as_f64().unwrap_or(d), Some(Value::Bool(b)) => *b as i32 as f64,
              Some(Value::String(s)) => s.trim().parse().unwrap_or(d), _ => d }
}
fn numz(v: Option<&Value>) -> f64 { num(v, 0.0) }

/// Python `float(x or 0.0)` + max(0,..)
fn fld(f: &Value, k: &str) -> f64 { numz(f.get(k)).max(0.0) }

pub fn braking_distance(v: f64, decel: f64, reaction_s: f64) -> f64 { v * reaction_s + v * v / (2.0 * decel.max(0.05)) }

pub fn default_fields(ch: &Value, reaction_s: f64) -> Vec<Value> {
    let dec = num(ch.get("max_decel_mps2").or(ch.get("max_accel_mps2")), 0.5);
    let vtop = NAV_MAX_SPEED.min(num(ch.get("max_speed_mps"), 1.5));
    let up = |d: f64| ((d + 0.1) * 20.0).ceil() / 20.0;
    let mut out: Vec<Value> = vec![];
    let mut last_v = f64::NEG_INFINITY;
    for (name, v, side) in [("低速", 0.3f64.min(vtop), 0.08), ("中速", 0.8f64.min(vtop), 0.10), ("高速", vtop, 0.15)] {
        if !out.is_empty() && v <= last_v + 1e-6 { continue; }
        let vr = (v * 100.0).round_ties_even() / 100.0;
        out.push(json!({"name": name, "v_max": vr, "front": up(braking_distance(v, dec, reaction_s)),
                        "rear": up(braking_distance(v.min(0.5), dec, reaction_s)), "side": side}));
        last_v = vr;
    }
    out
}

pub fn body_box(ch: &Value) -> [f64; 4] {
    [num(ch.get("head_offset_m"), 0.6), num(ch.get("tail_offset_m"), 0.6), num(ch.get("left_offset_m"), 0.4), num(ch.get("right_offset_m"), 0.4)]
}

/// spec (with "protection") or a bare protection fragment → complete config
pub fn effective(spec_or_prot: &Value, ch: Option<&Value>) -> Prot {
    let empty = Value::Object(Map::new());
    let mut raw = if spec_or_prot.is_null() { &empty } else { spec_or_prot };
    let mut ch_v: Option<Value> = ch.cloned();
    if let Some(c) = raw.get("chassis") {
        if ch_v.is_none() { ch_v = Some(c.clone()); }
        raw = raw.get("protection").filter(|x| !x.is_null()).unwrap_or(&empty);
    }
    let chr = ch_v.as_ref().filter(|c| c.as_object().map(|o| !o.is_empty()).unwrap_or(false));
    let mut p = merge(&defaults(), raw);
    let raw_fields = raw.get("fields").and_then(|f| f.as_array()).filter(|a| !a.is_empty());
    let reaction = num(p.get("reaction_s"), 0.3);
    {
        let o = p.as_object_mut().unwrap();
        if let Some(rf) = raw_fields {
            o.insert("fields".into(), Value::Array(rf.clone()));
        } else if let Some(c) = chr {
            o.insert("fields".into(), Value::Array(default_fields(c, reaction)));
            o.insert("fields_auto".into(), Value::Bool(true));
        }
    }
    let src: Vec<Value> = {
        let f = p["fields"].as_array().cloned().unwrap_or_default();
        if f.is_empty() { defaults()["fields"].as_array().unwrap().clone() } else { f }
    };
    let mut fs: Vec<Map<String, Value>> = vec![];
    for (i, f) in src.iter().enumerate() {
        let mut m = f.as_object().cloned().unwrap_or_default();
        if !m.contains_key("name") { m.insert("name".into(), json!(format!("档{}", i + 1))); }
        m.insert("v_max".into(), json!(num(f.get("v_max"), 9.0)));
        for k in ["front", "rear", "side"] { m.insert(k.into(), json!(fld(f, k))); }
        fs.push(m);
    }
    fs.sort_by(|a, b| a["v_max"].as_f64().unwrap().total_cmp(&b["v_max"].as_f64().unwrap()));
    if let Some(l) = fs.last_mut() {
        let v = l["v_max"].as_f64().unwrap().max(9.0);
        l.insert("v_max".into(), json!(v));
    }
    {
        let o = p.as_object_mut().unwrap();
        o.insert("fields".into(), Value::Array(fs.into_iter().map(Value::Object).collect()));
        for k in ["body_margin", "slow_ratio", "slow_speed", "rotate_margin", "rotate_lookahead_rad", "reaction_s"] {
            let v = numz(o.get(k)).max(0.0);
            o.insert(k.into(), json!(v));
        }
        let sr = numz(o.get("slow_ratio")).max(1.0);
        o.insert("slow_ratio".into(), json!(sr));
        if let Some(c) = chr {
            let pl = o.get_mut("payload").and_then(|x| x.as_object_mut()).unwrap();
            for (k, ck) in [("head", "head_offset_m"), ("tail", "tail_offset_m"), ("left", "left_offset_m"), ("right", "right_offset_m")] {
                if pl.get(k).map(|v| v.is_null()).unwrap_or(true) { pl.insert(k.into(), json!(num(c.get(ck), 0.5))); }
            }
        }
    }
    Prot::from_value(p)
}

impl Prot {
    pub fn from_value(p: Value) -> Prot {
        let fields = p["fields"].as_array().map(|a| a.iter().map(|f| Field {
            name: f["name"].as_str().unwrap_or("").to_string(), v_max: numz(f.get("v_max")),
            front: numz(f.get("front")), rear: numz(f.get("rear")), side: numz(f.get("side")) }).collect()).unwrap_or_default();
        let pl = &p["payload"];
        let ph = &p["photo"];
        Prot {
            enabled: p.get("enabled").map(|v| v.as_bool().unwrap_or(true)).unwrap_or(true),
            body_margin: numz(p.get("body_margin")),
            payload_enabled: pl.get("enabled").and_then(|v| v.as_bool()).unwrap_or(false),
            payload: [num(pl.get("head"), f64::NAN), num(pl.get("tail"), f64::NAN), num(pl.get("left"), f64::NAN), num(pl.get("right"), f64::NAN)],
            fields,
            slow_ratio: num(p.get("slow_ratio"), 2.0), slow_speed: numz(p.get("slow_speed")),
            rotate_margin: numz(p.get("rotate_margin")), rotate_lookahead_rad: numz(p.get("rotate_lookahead_rad")),
            docking_front: numz(p["docking"].get("front")), arrive_tolerance: num(p["docking"].get("arrive_tolerance"), 0.25),
            reaction_s: numz(p.get("reaction_s")),
            photo_mode: ph.get("mode").and_then(|v| v.as_str()).unwrap_or("field").to_string(),
            photo_front: num(ph.get("front"), 0.3), photo_rear: num(ph.get("rear"), 0.3), photo_side: num(ph.get("side"), 0.1),
            photo_mute_near_stop: num(ph.get("mute_near_stop"), 0.1),
            corner_mode: p.get("corner_mode").and_then(|v| v.as_str()).unwrap_or("auto").to_string(),
            json: p,
        }
    }

    /// current outline (head, tail, left, right)
    pub fn outline(&self, body: &[f64; 4], loaded: bool) -> [f64; 4] {
        if !(loaded && self.payload_enabled) { return *body; }
        let mut o = [0.0; 4];
        for i in 0..4 {
            let p = if self.payload[i].is_nan() { body[i] } else { self.payload[i] };
            o[i] = body[i].max(p);
        }
        o
    }

    pub fn field_for_speed(&self, v: f64) -> (usize, &Field) {
        let v = v.abs();
        for (i, f) in self.fields.iter().enumerate() { if v <= f.v_max + 1e-6 { return (i, f); } }
        (self.fields.len() - 1, self.fields.last().unwrap())
    }

    pub fn photo_envelope(&self, body: &[f64; 4], loaded: bool, field_idx: usize) -> Option<[f64; 4]> {
        if self.photo_mode == "always" { return None; }
        let [h, t, l, r] = self.outline(body, loaded);
        let (fr, rr, sd) = if self.photo_mode == "custom" { (self.photo_front, self.photo_rear, self.photo_side) } else {
            let f = &self.fields[field_idx.min(self.fields.len() - 1)];
            (f.front, f.rear, f.side)
        };
        Some([h + fr, t + rr, l + sd, r + sd])
    }

    /// base look-ahead + braking angle of the current angular rate; capped by remaining rotation (+base)
    pub fn rotate_lookahead(&self, w: f64, ang_decel: f64, rot_left: f64) -> f64 {
        let w = w.abs();
        let mut look = self.rotate_lookahead_rad + w * self.reaction_s + w * w / (2.0 * ang_decel.max(0.1));
        if rot_left >= 0.0 { look = look.min(rot_left + self.rotate_lookahead_rad); }
        look.min(std::f64::consts::PI)
    }
}

pub fn rect(h: f64, t: f64, l: f64, r: f64) -> Value { json!([[h, l], [h, -r], [-t, -r], [-t, l]]) }

/// body-frame polygons for the UI
pub fn polygons(body: &[f64; 4], p: &Prot, loaded: bool, field_idx: usize) -> Value {
    let [h, t, l, r] = p.outline(body, loaded);
    let f = &p.fields[field_idx.min(p.fields.len() - 1)];
    let k = p.slow_ratio;
    let mut out = Map::new();
    out.insert("body".into(), rect(body[0], body[1], body[2], body[3]));
    out.insert("outline".into(), rect(h, t, l, r));
    out.insert("stop".into(), rect(h + f.front, t + f.rear, l + f.side, r + f.side));
    out.insert("slow".into(), rect(h + f.front * k, t + f.rear * k, l + f.side, r + f.side));
    if p.payload_enabled { out.insert("payload".into(), rect(p.payload[0], p.payload[1], p.payload[2], p.payload[3])); }
    if let Some(e) = p.photo_envelope(body, loaded, field_idx) { out.insert("photo".into(), rect(e[0], e[1], e[2], e[3])); }
    let w = l.max(r);
    let rad = (h.hypot(w).max(t.hypot(w)) + p.rotate_margin) * 1000.0;
    out.insert("rotate_radius".into(), json!(rad.round_ties_even() / 1000.0));
    Value::Object(out)
}
