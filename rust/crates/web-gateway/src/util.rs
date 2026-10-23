use serde_json::Value;
use std::sync::atomic::{AtomicU64, Ordering};

pub fn now() -> f64 { std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0) }
pub fn rnd(v: f64, n: i32) -> f64 { let p = 10f64.powi(n); (v * p).round() / p }
pub fn jr(v: f64, n: i32) -> Value { serde_json::json!(rnd(v, n)) }

pub fn fmt_local(ts: f64, fmt: &str) -> String {
    unsafe {
        let t = ts as libc::time_t;
        let mut tm: libc::tm = std::mem::zeroed();
        libc::localtime_r(&t, &mut tm);
        let mut buf = [0u8; 128];
        let f = std::ffi::CString::new(fmt).unwrap();
        let n = libc::strftime(buf.as_mut_ptr() as *mut libc::c_char, buf.len(), f.as_ptr(), &tm);
        String::from_utf8_lossy(&buf[..n]).to_string()
    }
}

static SEED: AtomicU64 = AtomicU64::new(0);
pub fn rand_u64() -> u64 {
    let mut s = SEED.load(Ordering::Relaxed);
    if s == 0 { s = (now() * 1e6) as u64 ^ 0x9E3779B97F4A7C15; }
    s ^= s << 13; s ^= s >> 7; s ^= s << 17;
    SEED.store(s, Ordering::Relaxed);
    s
}
pub fn rand_f64() -> f64 { (rand_u64() >> 11) as f64 / (1u64 << 53) as f64 }
pub fn uniform(a: f64, b: f64) -> f64 { a + (b - a) * rand_f64() }
pub fn randint(a: u64, b: u64) -> u64 { a + rand_u64() % (b - a + 1) }
pub fn shuffle<T>(v: &mut [T]) { for i in (1..v.len()).rev() { let j = (rand_u64() % (i as u64 + 1)) as usize; v.swap(i, j); } }

pub fn f(v: &Value, k: &str, d: f64) -> f64 { v.get(k).and_then(|x| x.as_f64()).unwrap_or(d) }
pub fn s<'a>(v: &'a Value, k: &str) -> &'a str { v.get(k).and_then(|x| x.as_str()).unwrap_or("") }
pub fn truthy(v: &Value) -> bool {
    match v { Value::Bool(b) => *b, Value::Null => false, Value::Number(n) => n.as_f64().map(|x| x != 0.0).unwrap_or(false), Value::String(s) => !s.is_empty(), Value::Array(a) => !a.is_empty(), Value::Object(o) => !o.is_empty() }
}
pub fn env_f(k: &str, d: f64) -> f64 { std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d) }
pub fn is_android() -> bool { std::path::Path::new("/system/build.prop").exists() }
