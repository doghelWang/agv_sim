//! Event bus (port of common/events.py)
use serde_json::{json, Value};
use std::sync::Mutex;

pub struct EventHub {
    inner: Mutex<Inner>,
    max_history: usize,
}
struct Inner { events: Vec<Value>, counter: u64, cats: Vec<(String, u64)> }

pub fn now() -> f64 { std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0) }

fn time_str(t: f64) -> String {
    unsafe {
        let secs = t.floor() as libc::time_t;
        let mut tm: libc::tm = std::mem::zeroed();
        libc::localtime_r(&secs, &mut tm);
        format!("{:02}:{:02}:{:02}.{:03}", tm.tm_hour, tm.tm_min, tm.tm_sec, ((t % 1.0) * 1000.0) as i64)
    }
}

impl EventHub {
    pub fn new(max_history: usize) -> EventHub {
        EventHub { inner: Mutex::new(Inner { events: vec![], counter: 0,
            cats: ["chassis", "navigation", "safety", "sensors", "system"].iter().map(|c| (c.to_string(), 0)).collect() }), max_history }
    }
    pub fn emit(&self, cat: &str, typ: &str, level: &str, title: &str, msg: &str, payload: Value) -> Value {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.counter += 1;
        let t = now();
        if let Some(c) = g.cats.iter_mut().find(|c| c.0 == cat) { c.1 += 1; }
        let ev = json!({"id": g.counter, "timestamp": t, "time_str": time_str(t), "category": cat, "type": typ, "level": level,
                        "title": title, "message": msg, "payload": if payload.is_null() { json!({}) } else { payload }});
        g.events.push(ev.clone());
        if g.events.len() > self.max_history { g.events.remove(0); }
        ev
    }
    pub fn get(&self, since: u64, limit: usize) -> Value {
        let g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let m: Vec<&Value> = g.events.iter().filter(|e| e["id"].as_u64().unwrap_or(0) > since).collect();
        let res: Vec<Value> = if limit > 0 && m.len() > limit { m[m.len() - limit..].iter().map(|v| (*v).clone()).collect() } else { m.into_iter().cloned().collect() };
        let cats: serde_json::Map<String, Value> = g.cats.iter().map(|(k, v)| (k.clone(), json!(v))).collect();
        json!({"events": res, "latest_id": g.counter, "total_retained": g.events.len(), "category_counts": cats})
    }
}
