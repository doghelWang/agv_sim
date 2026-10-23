//! Persistence: SQLite document tables (id → JSON) + file directories. Compatible with the Python hub's hub.db.
use rest::ApiError;
use rusqlite::{Connection, OptionalExtension};
use serde_json::{json, Value};
use std::path::PathBuf;
use std::sync::Mutex;

pub type R<T> = Result<T, ApiError>;
pub const KINDS: [&str; 8] = ["nodes", "tokens", "models", "scenes", "packages", "instances", "records", "settings"];

pub fn now() -> f64 { std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0) }

pub fn rand_bytes(n: usize) -> Vec<u8> {
    use std::io::Read;
    let mut b = vec![0u8; n];
    if let Ok(mut f) = std::fs::File::open("/dev/urandom") { let _ = f.read_exact(&mut b); }
    b
}
pub fn hex(b: &[u8]) -> String { b.iter().map(|x| format!("{x:02x}")).collect() }
pub fn token_urlsafe(n: usize) -> String { use base64::Engine; base64::engine::general_purpose::URL_SAFE_NO_PAD.encode(rand_bytes(n)) }
pub fn new_id(prefix: &str, exists: &dyn Fn(&str) -> bool) -> String {
    loop { let i = format!("{prefix}{}", &hex(&rand_bytes(4))[..6]); if !exists(&i) { return i; } }
}
pub fn slug(s: &str, fallback: &str) -> String {
    let mut o = String::new();
    let mut last_dash = false;
    for c in s.chars() {
        if c.is_ascii_alphanumeric() || c == '_' || c == '.' || c == '-' { o.push(c); last_dash = false; }
        else if !last_dash { o.push('-'); last_dash = true; }
    }
    let t: String = o.trim_matches(|c| c == '-' || c == '.').to_lowercase().chars().take(40).collect();
    if t.is_empty() { fallback.into() } else { t }
}
pub fn ct_eq(a: &str, b: &str) -> bool {
    if a.len() != b.len() { return false; }
    a.bytes().zip(b.bytes()).fold(0u8, |x, (p, q)| x | (p ^ q)) == 0
}

pub struct Store { pub root: PathBuf, db: Mutex<Connection> }

fn get_in(c: &Connection, kind: &str, id: &str) -> Option<Value> {
    c.query_row(&format!("SELECT data FROM {kind} WHERE id=?1"), [id], |r| r.get::<_, String>(0)).optional().ok().flatten().and_then(|s| serde_json::from_str(&s).ok())
}
fn put_in(c: &Connection, kind: &str, mut doc: Value) -> Value {
    let t = now();
    if let Some(o) = doc.as_object_mut() {
        if !o.contains_key("created") { o.insert("created".into(), json!(t)); }
        o.insert("updated".into(), json!(t));
    }
    let id = doc.get("id").and_then(|v| v.as_str()).unwrap_or("").to_string();
    if let Err(e) = c.execute(&format!("INSERT OR REPLACE INTO {kind} (id, data, updated) VALUES (?1,?2,?3)"), rusqlite::params![id, serde_json::to_string(&doc).unwrap(), t]) {
        eprintln!("[hub] 写入 {kind}/{id} 失败: {e}");
    }
    doc
}

impl Store {
    pub fn new(root: &str) -> Result<Store, String> {
        let root = if let Some(r) = root.strip_prefix("~/") { format!("{}/{}", std::env::var("HOME").unwrap_or_else(|_| "/root".into()), r) } else { root.to_string() };
        let root = PathBuf::from(root);
        for d in ["models", "scenes", "packages", "records", "tmp"] { std::fs::create_dir_all(root.join(d)).map_err(|e| e.to_string())?; }
        let c = Connection::open(root.join("hub.db")).map_err(|e| e.to_string())?;
        let _ = c.query_row("PRAGMA journal_mode=WAL", [], |r| r.get::<_, String>(0));
        let _ = c.busy_timeout(std::time::Duration::from_secs(10));
        for k in KINDS { c.execute(&format!("CREATE TABLE IF NOT EXISTS {k} (id TEXT PRIMARY KEY, data TEXT NOT NULL, updated REAL)"), []).map_err(|e| e.to_string())?; }
        Ok(Store { root, db: Mutex::new(c) })
    }
    pub fn path(&self, parts: &[&str]) -> PathBuf { let mut p = self.root.clone(); for x in parts { p.push(x); } p }
    pub fn get(&self, kind: &str, id: &str) -> Option<Value> { get_in(&self.db.lock().unwrap_or_else(|e| e.into_inner()), kind, id) }
    pub fn put(&self, kind: &str, doc: Value) -> Value { put_in(&self.db.lock().unwrap_or_else(|e| e.into_inner()), kind, doc) }
    pub fn update(&self, kind: &str, id: &str, f: impl FnOnce(&mut Value)) -> Option<Value> {
        let c = self.db.lock().unwrap_or_else(|e| e.into_inner());
        let mut d = get_in(&c, kind, id)?;
        f(&mut d);
        Some(put_in(&c, kind, d))
    }
    pub fn delete(&self, kind: &str, id: &str) { let _ = self.db.lock().unwrap_or_else(|e| e.into_inner()).execute(&format!("DELETE FROM {kind} WHERE id=?1"), [id]); }
    pub fn list(&self, kind: &str) -> Vec<Value> {
        let c = self.db.lock().unwrap_or_else(|e| e.into_inner());
        let Ok(mut st) = c.prepare(&format!("SELECT data FROM {kind} ORDER BY updated DESC")) else { return vec![] };
        let rows = st.query_map([], |r| r.get::<_, String>(0)).map(|it| it.filter_map(|x| x.ok()).collect::<Vec<_>>()).unwrap_or_default();
        rows.iter().filter_map(|s| serde_json::from_str(s).ok()).collect()
    }
    pub fn list_eq(&self, kind: &str, key: &str, val: &str) -> Vec<Value> { self.list(kind).into_iter().filter(|d| d.get(key).and_then(|v| v.as_str()) == Some(val)).collect() }
    pub fn setting(&self, key: &str) -> Option<Value> { self.get("settings", key).and_then(|d| d.get("value").cloned()) }
    pub fn set_setting(&self, key: &str, v: Value) { self.put("settings", json!({"id": key, "value": v})); }
    pub fn write_json(&self, path: &std::path::Path, v: &Value) {
        if let Some(d) = path.parent() { let _ = std::fs::create_dir_all(d); }
        let tmp = format!("{}.{}.tmp", path.display(), std::process::id());
        if std::fs::write(&tmp, serde_json::to_string_pretty(v).unwrap()).is_ok() { let _ = std::fs::rename(&tmp, path); }
        let _ = std::fs::remove_file(&tmp);
    }
    pub fn read_json(path: &std::path::Path) -> Option<Value> { std::fs::read_to_string(path).ok().and_then(|s| serde_json::from_str(&s).ok()) }
}

pub fn err(status: u16, m: impl Into<String>) -> ApiError { ApiError::new(status, m) }
pub fn sget<'a>(v: &'a Value, k: &str) -> &'a str { v.get(k).and_then(|x| x.as_str()).unwrap_or("") }
pub fn truthy(v: Option<&Value>) -> bool {
    match v { None | Some(Value::Null) => false, Some(Value::Bool(b)) => *b, Some(Value::Number(n)) => n.as_f64().unwrap_or(0.0) != 0.0,
        Some(Value::String(s)) => !s.is_empty(), Some(Value::Array(a)) => !a.is_empty(), Some(Value::Object(o)) => !o.is_empty() }
}
