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
    /// built-in library: every <AGV_HOME>/models/*.cmodel not yet in the library (matched by file name) is imported once
    pub fn seed_builtins(&self, dir: &std::path::Path) {
        let Ok(rd) = std::fs::read_dir(dir) else { return };
        for e in rd.flatten() {
            let p = e.path();
            let fname = p.file_name().and_then(|f| f.to_str()).unwrap_or("").to_string();
            if !fname.to_lowercase().ends_with(".cmodel") { continue; }
            if self.s.list("models").iter().any(|m| m.get("file").and_then(|f| f.as_str()) == Some(fname.as_str())) { continue; }
            let Ok(data) = std::fs::read(&p) else { continue };
            let (name, vtype) = match fname.as_str() {
                "forklift_4steer.cmodel" => ("四舵轮叉车 AMR", "叉车 AMR"),
                "pallet_truck_amr.cmodel" => ("托盘搬运 AMR (地牛式)", "托盘搬运 AMR"),
                _ => ("", ""),
            };
            let mut meta = json!({"project": "示例项目", "note": "平台内置示例模型", "operator": "builtin"});
            if !name.is_empty() { meta["name"] = json!(name); meta["vtype"] = json!(vtype); }
            match self.add_cmodel(&fname, &data, &meta) { Ok(_) => eprintln!("[hub] 内置模型已入库: {fname}"), Err(e) => eprintln!("[hub] 内置模型 {fname} 入库失败: {}", e.message) }
        }
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
    /// .cmodel upload: decode + parse into base.json, keep previous overrides (Python hub/models_repo.add_cmodel)
    pub fn add_cmodel(&self, filename: &str, data: &[u8], meta: &Value) -> R<Value> {
        if !filename.to_lowercase().ends_with(".cmodel") { return Err(err(400, "仅支持 .cmodel 模型文件")); }
        let load = meta.get("load").and_then(|l| l.as_str()).unwrap_or("full").to_string();
        let comp = cmodel::decode_cmodel_bytes(data).map_err(|e| err(400, format!("cmodel 解析失败: {e}")))?;
        let mut base = cmodel::spec::extract_robot_spec(&comp["CompDesc"], filename, &load, None);
        base["model_file"] = json!(filename);
        let stem = if filename.len() > 7 { &filename[..filename.len() - 7] } else { filename };
        let name = { let n = meta.get("name").and_then(|v| v.as_str()).unwrap_or("").trim().to_string(); if n.is_empty() { stem.to_string() } else { n } };
        let find_by_name = || self.s.list("models").into_iter().find(|m| sget(m, "name") == name).map(|m| sget(&m, "id").to_string());
        let new_mid = || { let b = format!("m-{}", slug(&name, "model")); let (mut mid, mut i) = (b.clone(), 2); while self.s.get("models", &mid).is_some() { mid = format!("{b}-{i}"); i += 1; } mid };
        let mid = meta.get("model_id").and_then(|v| v.as_str()).filter(|s| !s.is_empty()).map(|s| s.to_string()).or_else(find_by_name).unwrap_or_else(new_mid);
        let mut m = self.s.get("models", &mid).unwrap_or_else(|| json!({"id": mid, "name": name, "versions": [], "created": now()}));
        let versions = arr(&m, "versions");
        let ver = match meta.get("version").and_then(|v| v.as_str()).map(|s| s.trim().to_string()).filter(|s| !s.is_empty()) {
            Some(v) => v,
            None => { let mut n = 1u64; for v in &versions { if let Some(k) = v.as_str().and_then(|s| s.strip_prefix('v')).and_then(|d| d.parse::<u64>().ok()) { n = n.max(k + 1); } } format!("v{n}") }
        };
        if versions.iter().any(|v| v.as_str() == Some(&ver)) { return Err(err(409, format!("版本 {ver} 已存在"))); }
        let vd = self.vdir(&mid, &ver);
        std::fs::create_dir_all(&vd).map_err(|e| err(500, e.to_string()))?;
        std::fs::write(vd.join("model.cmodel"), data).map_err(|e| err(500, e.to_string()))?;
        self.s.write_json(&vd.join("base.json"), &base);
        let prev = m.get("latest").and_then(|v| v.as_str()).map(|s| s.to_string()).filter(|s| !s.is_empty());
        let ov = prev.and_then(|p| Store::read_json(&self.vdir(&mid, &p).join("overrides.json"))).filter(|o| truthy(Some(o))).unwrap_or(json!({"version": 1}));
        self.s.write_json(&vd.join("overrides.json"), &ov);
        self.s.write_json(&vd.join("meta.json"), &json!({"file": filename, "uploaded": now(), "size": data.len(), "operator": meta.get("operator").cloned().unwrap_or(json!(""))}));
        let mut vs = versions; vs.push(json!(ver));
        m["versions"] = Value::Array(vs); m["latest"] = json!(ver);
        for k in ["project", "vtype", "material_no", "note"] { if let Some(v) = meta.get(k).filter(|v| !v.is_null()) { m[k] = v.clone(); } }
        if m.get("vtype").is_none() {
            let t = base["chassis"]["type"].as_str().unwrap_or("").to_string();
            let lab = if t.is_empty() { "AMR".to_string() } else { chassis_label(&t) };
            m["vtype"] = json!(if base["lift"]["enabled"].as_bool().unwrap_or(false) { format!("{lab}顶升 AMR") } else { format!("{lab} AMR") });
        }
        m["file"] = json!(filename);
        m["summary"] = Self::summarize(&base);
        self.s.put("models", m.clone());
        Ok(self.view(&m))
    }
    pub fn urdf(&self, mid: &str, ver: Option<&str>) -> R<String> { Ok(model::urdf::generate_urdf(&self.spec(mid, ver)?)) }
    pub fn bundle(&self, mid: &str, ver: Option<&str>) -> R<Value> {
        let m = self.get(mid)?;
        let v = self.ver(&m, ver)?;
        Ok(json!({"model_id": mid, "version": v, "name": nv(m.get("name")), "file": nv(m.get("file")), "material_no": nv(m.get("material_no")), "vtype": nv(m.get("vtype")),
            "project": nv(m.get("project")), "base": self.base(mid, Some(&v))?, "overrides": self.overrides(mid, Some(&v))?}))
    }
}
