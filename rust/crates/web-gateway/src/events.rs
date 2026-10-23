use crate::util::*;
use serde_json::{json, Map, Value};
use std::collections::VecDeque;
use std::sync::Mutex;

struct Inner { events: VecDeque<Value>, counter: u64, counts: Map<String, Value> }
pub struct EventHub { inner: Mutex<Inner>, max: usize }

impl EventHub {
    pub fn new(max: usize) -> EventHub {
        let mut counts = Map::new();
        for c in ["chassis", "navigation", "safety", "sensors", "system"] { counts.insert(c.into(), json!(0)); }
        EventHub { inner: Mutex::new(Inner { events: VecDeque::new(), counter: 0, counts }), max }
    }
    fn lock(&self) -> std::sync::MutexGuard<'_, Inner> { self.inner.lock().unwrap_or_else(|e| e.into_inner()) }
    pub fn counter(&self) -> u64 { self.lock().counter }
    pub fn emit(&self, category: &str, etype: &str, level: &str, title: &str, message: &str, payload: Value) -> Value {
        let mut g = self.lock();
        g.counter += 1;
        let t = now();
        let time_str = format!("{}.{:03}", fmt_local(t, "%H:%M:%S"), ((t % 1.0) * 1000.0) as i64);
        if let Some(c) = g.counts.get_mut(category) { *c = json!(c.as_i64().unwrap_or(0) + 1); }
        let payload = if truthy(&payload) { payload } else { json!({}) };
        let ev = json!({"id": g.counter, "timestamp": t, "time_str": time_str, "category": category, "type": etype, "level": level, "title": title, "message": message, "payload": payload});
        g.events.push_back(ev.clone());
        if g.events.len() > self.max { g.events.pop_front(); }
        ev
    }
    pub fn get_events(&self, since_id: u64, categories: Option<Vec<String>>, limit: usize) -> Value {
        let g = self.lock();
        let cats: Option<Vec<String>> = categories.map(|c| c.into_iter().map(|x| x.trim().to_string()).filter(|x| !x.is_empty()).collect());
        let matched: Vec<&Value> = g.events.iter().filter(|e| e["id"].as_u64().unwrap_or(0) > since_id && cats.as_ref().map(|c| c.iter().any(|x| Some(x.as_str()) == e["category"].as_str())).unwrap_or(true)).collect();
        let res: Vec<Value> = if limit > 0 && matched.len() > limit { matched[matched.len() - limit..].iter().map(|v| (*v).clone()).collect() } else { matched.into_iter().cloned().collect() };
        json!({"events": res, "latest_id": g.counter, "total_retained": g.events.len(), "category_counts": g.counts})
    }
    pub fn clear(&self) -> Value { let mut g = self.lock(); g.events.clear(); json!({"status": "cleared", "latest_id": g.counter}) }
}
