//! Admin auth (PBKDF2 password, persisted bearer tokens). Update/APK/adb/panel operations are not provided by the Rust hub.
use crate::store::*;
use base64::Engine;
use serde_json::{json, Map, Value};
use sha2::{Digest, Sha256};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

pub struct Admin { pub s: Arc<Store>, pub dir: PathBuf, pub job: Mutex<Option<Value>>, tokens: Mutex<Map<String, Value>>, pub started: f64, pub root: String }
fn b64(b: &[u8]) -> String { base64::engine::general_purpose::STANDARD.encode(b) }
fn sha(t: &str) -> String { hex(&Sha256::digest(t.as_bytes())) }

impl Admin {
    pub fn new(s: Arc<Store>, root: &str) -> Admin {
        let dir = s.path(&["admin"]);
        for d in ["updates", "backups", "jobs", "apps"] { let _ = std::fs::create_dir_all(dir.join(d)); }
        let t = Store::read_json(&dir.join("tokens.json")).and_then(|v| v.as_object().cloned()).unwrap_or_default();
        let t: Map<String, Value> = t.into_iter().filter(|(_, v)| v.as_f64().unwrap_or(0.0) > now()).collect();
        Admin { s, dir, job: Mutex::new(None), tokens: Mutex::new(t), started: now(), root: root.into() }
    }
    fn save(&self, t: &Map<String, Value>) {
        let p = self.dir.join("tokens.json");
        let live: Map<String, Value> = t.iter().filter(|(_, v)| v.as_f64().unwrap_or(0.0) > now()).map(|(k, v)| (k.clone(), v.clone())).collect();
        let _ = std::fs::write(&p, serde_json::to_string(&live).unwrap());
        #[cfg(unix)] { use std::os::unix::fs::PermissionsExt; let _ = std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o600)); }
    }
    fn hash(pw: &str, salt: &[u8]) -> String { let mut out = [0u8; 32]; pbkdf2::pbkdf2_hmac::<Sha256>(pw.as_bytes(), salt, 120000, &mut out); b64(&out) }
    pub fn check(&self, token: &str, raise: bool) -> R<bool> {
        let ok = !token.is_empty() && self.tokens.lock().unwrap().get(&sha(token)).and_then(|v| v.as_f64()).unwrap_or(0.0) > now();
        if !ok && raise { return Err(rest::ApiError::coded(401, "请先登录运维管理", "admin_auth")); }
        Ok(ok)
    }
    pub fn state(&self, token: &str) -> Value { json!({"configured": truthy(self.s.setting("admin_pw").as_ref()), "authed": self.check(token, false).unwrap_or(false)}) }
    pub fn setup(&self, pw: &str) -> R<Value> { if truthy(self.s.setting("admin_pw").as_ref()) { return Err(err(409, "管理员密码已设置，请直接登录")); } self.set_pw(pw) }
    fn set_pw(&self, pw: &str) -> R<Value> {
        if pw.chars().count() < 6 { return Err(err(400, "密码至少 6 位")); }
        let salt = rand_bytes(16);
        self.s.set_setting("admin_pw", json!(json!({"salt": b64(&salt), "hash": Self::hash(pw, &salt)}).to_string()));
        { let mut t = self.tokens.lock().unwrap(); t.clear(); self.save(&t); }
        Ok(self.issue())
    }
    pub fn login(&self, pw: &str) -> R<Value> {
        let rec = self.s.setting("admin_pw").and_then(|v| v.as_str().map(|s| s.to_string())).filter(|s| !s.is_empty()).ok_or_else(|| err(409, "尚未设置管理员密码"))?;
        let rec: Value = serde_json::from_str(&rec).map_err(|_| err(500, "管理员密码记录损坏"))?;
        let salt = base64::engine::general_purpose::STANDARD.decode(sget(&rec, "salt")).unwrap_or_default();
        if !ct_eq(&Self::hash(pw, &salt), sget(&rec, "hash")) { std::thread::sleep(std::time::Duration::from_secs(1)); return Err(rest::ApiError::coded(401, "密码错误", "bad_password")); }
        Ok(self.issue())
    }
    pub fn change_pw(&self, old: &str, new: &str) -> R<Value> { self.login(old)?; self.set_pw(new) }
    fn issue(&self) -> Value {
        let t = token_urlsafe(24);
        let exp = now() + 12.0 * 3600.0;
        let mut m = self.tokens.lock().unwrap();
        m.insert(sha(&t), json!(exp));
        self.save(&m);
        json!({"token": t, "expires": exp})
    }
    pub fn logout(&self, token: &str) -> Value { let mut m = self.tokens.lock().unwrap(); m.remove(&sha(token)); self.save(&m); json!({"ok": true}) }
    pub fn version(&self) -> Value {
        let line = std::fs::read_to_string(std::path::Path::new(&self.root).join(".agv_version")).ok().and_then(|s| s.lines().next().map(|l| l.trim().to_string())).unwrap_or_default();
        let (v, t) = line.split_once(' ').unwrap_or((&line, ""));
        json!({"version": if v.is_empty() { "unknown" } else { v }, "title": t})
    }
}
