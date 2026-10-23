//! Vehicle model repository (base.json + overrides.json per version).
use crate::store::*;
use model::{fnum, rnd};
use serde_json::{json, Map, Value};
use std::path::PathBuf;
use std::sync::Arc;

pub struct ModelRepo { pub s: Arc<Store> }

fn chassis_label(t: &str) -> String {
    match t { "single_steer" => "单舵轮", "dual_steer" => "双舵轮", "diff_drive" => "差速双驱", "quad_steer" => "四舵轮", "omni" => "全向", "mecanum" => "麦克纳姆轮", o => o }.to_string()
}
fn nv(v: Option<&Value>) -> Value { v.cloned().unwrap_or(Value::Null) }
fn arr(v: &Value, k: &str) -> Vec<Value> { v.get(k).and_then(|a| a.as_array()).cloned().unwrap_or_default() }

impl ModelRepo {
    fn vdir(&self, mid: &str, ver: &str) -> PathBuf { self.s.path(&["models", mid, ver]) }
    pub fn get(&self, mid: &str) -> R<Value> { self.s.get("models", mid).ok_or_else(|| err(404, format!("模型 {mid} 不存在"))) }
    fn ver(&self, m: &Value, ver: Option<&str>) -> R<String> {
        let latest = sget(m, "latest").to_string();
        let v = match ver { Some(v) if !v.is_empty() && v != "latest" => v.to_string(), _ => latest };
        if !arr(m, "versions").iter().any(|x| x.as_str() == Some(&v)) { return Err(err(404, format!("模型 {} 没有版本 {}", sget(m, "id"), v))); }
        Ok(v)
    }
    pub fn base(&self, mid: &str, ver: Option<&str>) -> R<Value> {
        let m = self.get(mid)?;
        let v = self.ver(&m, ver)?;
        Store::read_json(&self.vdir(mid, &v).join("base.json")).ok_or_else(|| err(500, "模型基线缺失"))
    }
    pub fn overrides(&self, mid: &str, ver: Option<&str>) -> R<Value> {
        let m = self.get(mid)?;
        let v = self.ver(&m, ver)?;
        model::overrides::load_overrides(&self.vdir(mid, &v).join("overrides.json").to_string_lossy()).map_err(|e| err(500, e))
    }
    pub fn spec(&self, mid: &str, ver: Option<&str>) -> R<Value> { Ok(model::overrides::apply_overrides(&self.base(mid, ver)?, &self.overrides(mid, ver)?)) }

    pub fn summarize(spec: &Value) -> Value {
        let ch = spec.get("chassis").cloned().unwrap_or(json!({}));
        let wheels = arr(spec, "wheels");
        let xs: Vec<f64> = wheels.iter().map(|w| fnum(w.get("x"), 0.0)).collect();
        let ys: Vec<f64> = wheels.iter().map(|w| fnum(w.get("y"), 0.0)).collect();
        let mx = |v: &Vec<f64>| v.iter().cloned().fold(f64::MIN, f64::max) - v.iter().cloned().fold(f64::MAX, f64::min);
        let drive = wheels.iter().filter(|w| matches!(sget(w, "kind"), "steer" | "drive")).count();
        let wb = if xs.len() >= 2 { mx(&xs) } else { 0.0 };
        let track = if ys.len() >= 2 { mx(&ys) } else { 0.0 };
        let lift = spec.get("lift").filter(|l| truthy(Some(l))).cloned();
        let ctype = ch.get("type").and_then(|t| t.as_str()).unwrap_or("");
        let pick = |o: &Value, ks: &[&str]| -> Value { let mut m = Map::new(); for k in ks { m.insert((*k).into(), nv(o.get(*k))); } Value::Object(m) };
        json!({"chassis": nv(ch.get("type")), "chassis_label": if ch.get("type").map(|t| t.is_string()).unwrap_or(false) { json!(chassis_label(ctype)) } else { Value::Null },
            "length_m": nv(ch.get("length_m")), "width_m": nv(ch.get("width_m")), "height_m": nv(ch.get("height_m")), "mass_kg": nv(ch.get("mass_kg")),
            "max_load_kg": nv(ch.get("max_load_kg")), "max_speed_mps": nv(ch.get("max_speed_mps")), "max_ang_speed_radps": nv(ch.get("max_ang_speed_radps")),
            "wheelbase_m": rnd(wb, 3), "track_m": rnd(track, 3), "footprint": nv(ch.get("footprint")), "rotate_diameter_m": nv(ch.get("rotate_diameter_m")),
            "wheels": wheels.iter().map(|w| json!({"name": nv(w.get("name")), "kind": nv(w.get("kind")), "x": nv(w.get("x")), "y": nv(w.get("y")), "r": nv(w.get("radius_m")), "w": nv(w.get("width_m"))})).collect::<Vec<_>>(),
            "drive_wheels": drive,
            "lidars": arr(spec, "lidars").iter().map(|l| pick(l, &["name", "type", "model", "x", "y", "z", "yaw", "fov_deg", "max_range"])).collect::<Vec<_>>(),
            "cameras": arr(spec, "cameras").iter().map(|c| pick(c, &["name", "type", "x", "y", "z", "yaw", "hfov_deg"])).collect::<Vec<_>>(),
            "photoelectric": arr(spec, "photoelectric").len(),
            "lift": match &lift { Some(l) => json!({"enabled": truthy(l.get("enabled")), "stroke_m": nv(l.get("stroke_m"))}), None => Value::Null },
            "model_file": nv(spec.get("model_file")), "robot_name": nv(spec.get("robot_name"))})
    }

    pub fn view(&self, m: &Value) -> Value {
        let mut d = m.clone();
        let id = sget(m, "id").to_string();
        let v = sget(m, "latest").to_string();
        let dir = self.vdir(&id, &v);
        let base = Store::read_json(&dir.join("base.json")).unwrap_or(json!({}));
        let ov = Store::read_json(&dir.join("overrides.json")).unwrap_or(json!({}));
        let sp = if truthy(Some(&base)) { model::overrides::apply_overrides(&base, &ov) } else { json!({}) };
        d["summary"] = if truthy(Some(&sp)) { Self::summarize(&sp) } else { m.get("summary").cloned().unwrap_or(json!({})) };
        d["audit"] = if truthy(Some(&sp)) { model::overrides::audit(&sp, Some(&ov)).get("summary").cloned().unwrap_or(json!({})) } else { json!({}) };
        d
    }
    pub fn list(&self) -> Vec<Value> {
        let mut ms = self.s.list("models");
        ms.sort_by(|a, b| fnum(a.get("created"), 0.0).partial_cmp(&fnum(b.get("created"), 0.0)).unwrap_or(std::cmp::Ordering::Equal));
        ms.iter().map(|m| self.view(m)).collect()
    }
    pub fn update_meta(&self, mid: &str, f: &Value) -> R<Value> {
        let mut m = self.get(mid)?;
        for k in ["name", "project", "vtype", "material_no", "note"] { if let Some(v) = f.get(k) { m[k] = v.clone(); } }
        let m = self.s.put("models", m);
        Ok(self.view(&m))
    }
    pub fn delete(&self, mid: &str) -> R<()> {
        self.get(mid)?;
        let _ = std::fs::remove_dir_all(self.s.path(&["models", mid]));
        self.s.delete("models", mid);
        Ok(())
    }
    pub fn cmodel_file(&self, mid: &str, ver: Option<&str>) -> R<PathBuf> { let m = self.get(mid)?; Ok(self.vdir(mid, &self.ver(&m, ver)?).join("model.cmodel")) }

    pub fn editor(&self, mid: &str, ver: Option<&str>) -> R<Value> {
        let (sp, ov) = (self.spec(mid, ver)?, self.overrides(mid, ver)?);
        let v = self.ver(&self.get(mid)?, ver)?;
        Ok(json!({"spec": sp, "base_file": "base.json", "overrides": ov, "audit": model::overrides::audit(&sp, Some(&ov)), "sensors": model::overrides::sensor_list(&sp, None),
            "sensor_types": model::overrides::sensor_types(), "repo": {"model_id": mid, "version": v}}))
    }
    pub fn preview(&self, mid: &str, ver: Option<&str>, ov: &Value) -> R<Value> {
        let sp = model::overrides::apply_overrides(&self.base(mid, ver)?, ov);
        Ok(json!({"spec": sp, "audit": model::overrides::audit(&sp, Some(ov)), "sensors": model::overrides::sensor_list(&sp, None), "urdf": model::urdf::generate_urdf(&sp)}))
    }
    pub fn save_overrides(&self, mid: &str, ver: Option<&str>, ov: &Value) -> R<Value> {
        let mut m = self.get(mid)?;
        let v = self.ver(&m, ver)?;
        let mut ov = if ov.is_object() { ov.clone() } else { json!({}) };
        if ov.get("version").is_none() { ov["version"] = json!(1); }
        model::overrides::save_overrides(&ov, &self.vdir(mid, &v).join("overrides.json").to_string_lossy()).map_err(|e| err(500, e))?;
        m["summary"] = Self::summarize(&self.spec(mid, Some(&v))?);
        self.s.put("models", m);
        self.editor(mid, Some(&v))
    }
    pub fn urdf(&self, mid: &str, ver: Option<&str>) -> R<String> { Ok(model::urdf::generate_urdf(&self.spec(mid, ver)?)) }
    pub fn bundle(&self, mid: &str, ver: Option<&str>) -> R<Value> {
        let m = self.get(mid)?;
        let v = self.ver(&m, ver)?;
        Ok(json!({"model_id": mid, "version": v, "name": nv(m.get("name")), "file": nv(m.get("file")), "material_no": nv(m.get("material_no")), "vtype": nv(m.get("vtype")),
            "project": nv(m.get("project")), "base": self.base(mid, Some(&v))?, "overrides": self.overrides(mid, Some(&v))?}))
    }
}
