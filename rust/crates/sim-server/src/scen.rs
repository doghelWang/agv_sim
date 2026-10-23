//! Scenario helpers: planner metadata/topology (port of DijkstraPlanner getters) and occupancy-grid rasterisation (tools/scenario_to_map.py).
use serde_json::{json, Map, Value};

pub fn r2(x: f64) -> f64 { (x * 100.0).round_ties_even() / 100.0 }

fn f(v: &Value) -> f64 { v.as_f64().unwrap_or(0.0) }

pub fn metadata(scenarios: &Map<String, Value>, id: &str) -> Value {
    let sc = &scenarios[id];
    let walls: Vec<Vec<f64>> = sc.get("walls").and_then(|w| w.as_array()).map(|a| a.iter().map(|w| w.as_array().map(|x| x.iter().map(f).collect()).unwrap_or_default()).collect()).unwrap_or_default();
    let w4 = &walls[..walls.len().min(4)];
    let (minx, maxx, miny, maxy) = if walls.len() >= 4 {
        let xs: Vec<f64> = w4.iter().flat_map(|w| [w[0], w[2]]).collect();
        let ys: Vec<f64> = w4.iter().flat_map(|w| [w[1], w[3]]).collect();
        (xs.iter().cloned().fold(f64::INFINITY, f64::min), xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
         ys.iter().cloned().fold(f64::INFINITY, f64::min), ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max))
    } else { (-7.5, 7.5, -7.5, 7.5) };
    let all: Vec<Value> = scenarios.iter().map(|(k, v)| json!({"id": k, "name": v["name"], "description": v["description"]})).collect();
    json!({"id": sc["id"], "name": sc["name"], "description": sc["description"], "walls": sc.get("walls").cloned().unwrap_or(json!([])),
        "shelves": sc.get("shelves").cloned().unwrap_or(json!([])), "stations": sc.get("stations").cloned().unwrap_or(json!([])), "origin": sc["origin"],
        "bounds": {"min_x": minx, "max_x": maxx, "min_y": miny, "max_y": maxy}, "all_scenarios": all})
}

pub fn topology(scenarios: &Map<String, Value>, id: &str) -> Value {
    let sc = &scenarios[id];
    let nodes = sc.get("nodes").and_then(|n| n.as_object()).cloned().unwrap_or_default();
    let mut edges: Vec<(String, Vec<(String, f64)>)> = nodes.keys().map(|k| (k.clone(), vec![])).collect();
    let idx = |k: &str, e: &Vec<(String, Vec<(String, f64)>)>| e.iter().position(|(n, _)| n == k);
    if let Some(cs) = sc.get("connections").and_then(|c| c.as_array()) {
        for c in cs {
            let (Some(u), Some(v)) = (c[0].as_str(), c[1].as_str()) else { continue };
            let (Some(pu), Some(pv)) = (nodes.get(u), nodes.get(v)) else { continue };
            let d = (f(&pv[0]) - f(&pu[0])).hypot(f(&pv[1]) - f(&pu[1]));
            let (iu, iv) = (idx(u, &edges).unwrap(), idx(v, &edges).unwrap());
            edges[iu].1.push((v.to_string(), d));
            edges[iv].1.push((u.to_string(), d));
        }
    }
    let mut seen: Vec<(String, String)> = vec![];
    let mut list = vec![];
    for (u, nb) in &edges {
        for (v, d) in nb {
            let mut k = (u.clone(), v.clone());
            if k.0 > k.1 { k = (k.1, k.0); }
            if seen.contains(&k) { continue; }
            seen.push(k);
            let (pu, pv) = (&nodes[u], &nodes[v]);
            list.push(json!({"from": u, "to": v, "p1": {"x": pu[0], "y": pu[1]}, "p2": {"x": pv[0], "y": pv[1]}, "distance": r2(*d)}));
        }
    }
    let nd: Map<String, Value> = nodes.iter().map(|(k, v)| (k.clone(), json!({"x": v[0], "y": v[1], "name": k}))).collect();
    json!({"scenario_id": id, "nodes": Value::Object(nd), "edges": list})
}

/// Returns (image row-major top-down, W, H, (x0, y0))
pub fn rasterize(sc: &Value, res: f64) -> (Vec<u8>, usize, usize, (f64, f64)) {
    let (margin, wall_thick) = (1.0, 0.10);
    let walls: Vec<[f64; 4]> = sc["walls"].as_array().unwrap().iter().map(|w| [f(&w[0]), f(&w[1]), f(&w[2]), f(&w[3])]).collect();
    let xs: Vec<f64> = walls.iter().flat_map(|w| [w[0], w[2]]).collect();
    let ys: Vec<f64> = walls.iter().flat_map(|w| [w[1], w[3]]).collect();
    let mn = |v: &[f64]| v.iter().cloned().fold(f64::INFINITY, f64::min);
    let mx = |v: &[f64]| v.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let (x0, y0) = (mn(&xs) - margin, mn(&ys) - margin);
    let (x1, y1) = (mx(&xs) + margin, mx(&ys) + margin);
    let w = ((x1 - x0) / res).ceil() as i64;
    let h = ((y1 - y0) / res).ceil() as i64;
    let mut img = vec![205u8; (w * h).max(0) as usize];
    let mut setp = |ix: i64, iy: i64, v: u8| { if ix >= 0 && ix < w && iy >= 0 && iy < h { img[((h - 1 - iy) * w + ix) as usize] = v; } };
    let w4 = &walls[..walls.len().min(4)];
    let bx: Vec<f64> = w4.iter().flat_map(|w| [w[0], w[2]]).collect();
    let by: Vec<f64> = w4.iter().flat_map(|w| [w[1], w[3]]).collect();
    let (bx0, bx1, by0, by1) = (mn(&bx), mx(&bx), mn(&by), mx(&by));
    for iy in (((by0 - y0) / res) as i64)..=(((by1 - y0) / res) as i64) {
        for ix in (((bx0 - x0) / res) as i64)..=(((bx1 - x0) / res) as i64) { setp(ix, iy, 254); }
    }
    if let Some(shs) = sc.get("shelves").and_then(|s| s.as_array()) {
        for sh in shs {
            let (mut a, mut b) = ([f(&sh["x1"]), f(&sh["x2"])], [f(&sh["y1"]), f(&sh["y2"])]);
            if a[0] > a[1] { a.swap(0, 1); }
            if b[0] > b[1] { b.swap(0, 1); }
            for iy in (((b[0] - y0) / res) as i64)..=(((b[1] - y0) / res).ceil() as i64) {
                for ix in (((a[0] - x0) / res) as i64)..=(((a[1] - x0) / res).ceil() as i64) { setp(ix, iy, 0); }
            }
        }
    }
    let half = wall_thick / 2.0;
    for wl in &walls {
        let (ax, ay, bx, by) = (wl[0], wl[1], wl[2], wl[3]);
        let l = (bx - ax).hypot(by - ay);
        let n = ((l / (res * 0.5)) as i64).max(1);
        for k in 0..=n {
            let (px, py) = (ax + (bx - ax) * k as f64 / n as f64, ay + (by - ay) * k as f64 / n as f64);
            let r = (half / res).ceil() as i64;
            let (cx, cy) = (((px - x0) / res) as i64, ((py - y0) / res) as i64);
            for dy in -r..=r { for dx in -r..=r { setp(cx + dx, cy + dy, 0); } }
        }
    }
    (img, w as usize, h as usize, (x0, y0))
}
