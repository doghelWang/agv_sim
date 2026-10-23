//! Scenario definitions (nodes/connections/walls/shelves/stations) — same JSON as the sim's scenarios.
use crate::{Pt, Seg};
use serde_json::Value;

#[derive(Clone, Debug)]
pub struct Scenario {
    pub id: String,
    pub name: String,
    pub description: String,
    pub origin: Value,
    pub shelves: Vec<Value>,
    pub walls: Vec<Seg>,
    pub stations: Vec<Value>,
    pub nodes: Vec<(String, Pt)>,
    pub connections: Vec<(String, String)>,
}

fn f(v: &Value) -> f64 { v.as_f64().unwrap_or(0.0) }

impl Scenario {
    pub fn from_value(v: &Value) -> Option<Scenario> {
        let id = v.get("id")?.as_str()?.to_string();
        let nodes = v.get("nodes").and_then(|n| n.as_object()).map(|o| {
            o.iter().map(|(k, p)| (k.clone(), (f(&p[0]), f(&p[1])))).collect()
        }).unwrap_or_default();
        let connections = v.get("connections").and_then(|c| c.as_array()).map(|a| {
            a.iter().filter_map(|c| Some((c.get(0)?.as_str()?.to_string(), c.get(1)?.as_str()?.to_string()))).collect()
        }).unwrap_or_default();
        let walls = v.get("walls").and_then(|c| c.as_array()).map(|a| {
            a.iter().map(|w| [f(&w[0]), f(&w[1]), f(&w[2]), f(&w[3])]).collect()
        }).unwrap_or_default();
        Some(Scenario {
            name: v.get("name").and_then(|x| x.as_str()).unwrap_or("").to_string(),
            description: v.get("description").and_then(|x| x.as_str()).unwrap_or("").to_string(),
            origin: v.get("origin").cloned().unwrap_or(Value::Null),
            shelves: v.get("shelves").and_then(|x| x.as_array()).cloned().unwrap_or_default(),
            stations: v.get("stations").and_then(|x| x.as_array()).cloned().unwrap_or_default(),
            id, walls, nodes, connections,
        })
    }

    /// walls + shelf outlines (static obstacle segments)
    pub fn static_segments(&self) -> Vec<Seg> {
        let mut segs = self.walls.clone();
        for sh in &self.shelves {
            let (mut x1, mut x2) = (f(&sh["x1"]), f(&sh["x2"]));
            let (mut y1, mut y2) = (f(&sh["y1"]), f(&sh["y2"]));
            if x1 > x2 { std::mem::swap(&mut x1, &mut x2); }
            if y1 > y2 { std::mem::swap(&mut y1, &mut y2); }
            segs.push([x1, y1, x2, y1]);
            segs.push([x2, y1, x2, y2]);
            segs.push([x2, y2, x1, y2]);
            segs.push([x1, y2, x1, y1]);
        }
        segs
    }
}

#[derive(Clone, Default)]
pub struct Library { pub list: Vec<Scenario> }

impl Library {
    pub fn from_json_map(m: &serde_json::Map<String, Value>) -> Library {
        Library { list: m.values().filter_map(Scenario::from_value).collect() }
    }
    pub fn get(&self, id: &str) -> Option<&Scenario> { self.list.iter().find(|s| s.id == id) }
    pub fn register(&mut self, v: &Value) -> Option<String> {
        let s = Scenario::from_value(v)?;
        let id = s.id.clone();
        match self.list.iter().position(|x| x.id == id) { Some(i) => self.list[i] = s, None => self.list.push(s) }
        Some(id)
    }
}
