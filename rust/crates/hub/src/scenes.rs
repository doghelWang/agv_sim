//! Simulation scene repository: scene.json + optional PGM/YAML grid, mesh, taskflows.
use crate::store::*;
use model::{fnum, pf, rnd};
use serde_json::{json, Map, Value};
use std::io::{Cursor, Read, Write};
use std::path::PathBuf;
use std::sync::Arc;

const SCENARIOS: &str = include_str!("../../../data/scenarios.json");
fn builtin_tag(id: &str) -> &'static str {
    match id { "grid_9_square" => "立体仓储", "standard_cross" => "十字交叉", "narrow_aisle" => "窄巷道", "rect_loop" => "环线", "fms_workshop" => "柔性车间", "fms_workshop_xl" => "柔性车间·大车", _ => "内置" }
}

// ------------------------------------------------------------------ grid helpers
pub fn parse_yaml_map(text: &str) -> Map<String, Value> {
    let mut d = Map::new();
    for line in text.lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        let Some((k, v)) = line.split_once(':') else { continue };
        let (k, v) = (k.trim().to_string(), v.trim());
        if v.starts_with('[') {
            let xs: Vec<Value> = v.trim_matches(|c| c == '[' || c == ']').split(',').filter(|x| !x.trim().is_empty()).filter_map(|x| x.trim().parse::<f64>().ok()).map(|x| json!(x)).collect();
            d.insert(k, Value::Array(xs));
        } else if !v.is_empty() && v.chars().all(|c| c.is_ascii_digit() || "-.eE+".contains(c)) {
            match v.parse::<f64>() { Ok(f) => { d.insert(k, json!(f)); } Err(_) => { d.insert(k, json!(v)); } }
        } else { d.insert(k, json!(v.trim_matches(|c| c == '\'' || c == '"'))); }
    }
    d
}

pub fn parse_pgm(data: &[u8]) -> Result<(usize, usize, u32, Vec<u8>), String> {
    let magic = data.get(..2).ok_or("PGM 数据不完整")?;
    let mut i = 2usize;
    let mut toks: Vec<u32> = vec![];
    while toks.len() < 3 {
        while i < data.len() && data[i].is_ascii_whitespace() { i += 1; }
        if i < data.len() && data[i] == b'#' { while i < data.len() && data[i] != b'\n' && data[i] != b'\r' { i += 1; } continue; }
        let j0 = i;
        while i < data.len() && !data[i].is_ascii_whitespace() { i += 1; }
        if j0 == i { return Err("PGM 头无效".into()); }
        toks.push(std::str::from_utf8(&data[j0..i]).map_err(|e| e.to_string())?.parse().map_err(|_| "PGM 头无效".to_string())?);
    }
    let (w, h, mx) = (toks[0] as usize, toks[1] as usize, toks[2]);
    i += 1;
    let px: Vec<u8> = if magic == b"P5" { data.get(i..).unwrap_or(&[]).iter().take(w * h).cloned().collect() }
        else if magic == b"P2" { String::from_utf8_lossy(data.get(i..).unwrap_or(&[])).split_whitespace().take(w * h).filter_map(|v| v.parse::<u32>().ok()).map(|v| (v * 255 / mx.max(1)).min(255) as u8).collect() }
        else { return Err("仅支持 PGM (P5/P2)".into()) };
    if px.len() < w * h { return Err("PGM 数据不完整".into()); }
    Ok((w, h, mx, px))
}

fn edges(g: &[Vec<bool>], cell: f64, origin: (f64, f64), height_m: f64) -> Vec<Value> {
    let (hk, wk) = (g.len() as isize, g.first().map(|r| r.len()).unwrap_or(0) as isize);
    let gv = |r: isize, c: isize| -> bool { r >= 0 && c >= 0 && r < hk && c < wk && g[r as usize][c as usize] };
    let (ox, oy) = origin;
    let mut segs = vec![];
    for r in 0..=hk {
        let y = oy + height_m - r as f64 * cell;
        let mut c = 0;
        while c < wk {
            if gv(r - 1, c) != gv(r, c) {
                let c0 = c;
                while c < wk && gv(r - 1, c) != gv(r, c) { c += 1; }
                segs.push(json!([rnd(ox + c0 as f64 * cell, 3), rnd(y, 3), rnd(ox + c as f64 * cell, 3), rnd(y, 3)]));
            } else { c += 1; }
        }
    }
    for c in 0..=wk {
        let x = ox + c as f64 * cell;
        let mut r = 0;
        while r < hk {
            if gv(r, c - 1) != gv(r, c) {
                let r0 = r;
                while r < hk && gv(r, c - 1) != gv(r, c) { r += 1; }
                segs.push(json!([rnd(x, 3), rnd(oy + height_m - r0 as f64 * cell, 3), rnd(x, 3), rnd(oy + height_m - r as f64 * cell, 3)]));
            } else { r += 1; }
        }
    }
    segs
}

pub fn grid_to_walls(w: usize, h: usize, px: &[u8], res: f64, origin: (f64, f64), negate: bool, occ_thresh: f64, max_segments: usize) -> Vec<Value> {
    let occ = |x: usize, y: usize| -> bool { let a = px[y * w + x] as f32 / 255.0; let p = if negate { a } else { (255.0 - px[y * w + x] as f32) / 255.0 }; let _ = a; (p as f64) > occ_thresh };
    let mut k = ((0.10 / res).round_ties_even() as usize).max(1);
    loop {
        let (hk, wk) = (h / k, w / k);
        let mut g = vec![vec![false; wk]; hk];
        for r in 0..hk { for c in 0..wk {
            'blk: for dy in 0..k { for dx in 0..k { if occ(c * k + dx, r * k + dy) { g[r][c] = true; break 'blk; } } }
        } }
        let segs = edges(&g, res * k as f64, origin, h as f64 * res);
        if segs.len() <= max_segments || k > 64 { return segs; }
        k *= 2;
    }
}

pub fn rasterize(sc: &Value, res: f64) -> (Vec<u8>, usize, usize, (f64, f64)) {
    let (margin, wall_thick) = (1.0, 0.10);
    let walls: Vec<[f64; 4]> = sc.get("walls").and_then(|w| w.as_array()).cloned().unwrap_or_default().iter().map(|w| [fnum(w.get(0), 0.0), fnum(w.get(1), 0.0), fnum(w.get(2), 0.0), fnum(w.get(3), 0.0)]).collect();
    let xs: Vec<f64> = walls.iter().flat_map(|w| [w[0], w[2]]).collect();
    let ys: Vec<f64> = walls.iter().flat_map(|w| [w[1], w[3]]).collect();
    let fmin = |v: &[f64]| v.iter().cloned().fold(f64::MAX, f64::min);
    let fmax = |v: &[f64]| v.iter().cloned().fold(f64::MIN, f64::max);
    let (x0, y0, x1, y1) = (fmin(&xs) - margin, fmin(&ys) - margin, fmax(&xs) + margin, fmax(&ys) + margin);
    let w = ((x1 - x0) / res).ceil() as usize;
    let h = ((y1 - y0) / res).ceil() as usize;
    let mut img = vec![205u8; w * h];
    let mut setp = |ix: i64, iy: i64, v: u8| { if ix >= 0 && iy >= 0 && (ix as usize) < w && (iy as usize) < h { img[(h - 1 - iy as usize) * w + ix as usize] = v; } };
    let b4: Vec<&[f64; 4]> = walls.iter().take(4).collect();
    let bx: Vec<f64> = b4.iter().flat_map(|w| [w[0], w[2]]).collect();
    let by: Vec<f64> = b4.iter().flat_map(|w| [w[1], w[3]]).collect();
    let (bx0, bx1, by0, by1) = (fmin(&bx), fmax(&bx), fmin(&by), fmax(&by));
    for iy in ((by0 - y0) / res) as i64..=(((by1 - y0) / res) as i64) { for ix in ((bx0 - x0) / res) as i64..=(((bx1 - x0) / res) as i64) { setp(ix, iy, 254); } }
    for sh in sc.get("shelves").and_then(|s| s.as_array()).cloned().unwrap_or_default() {
        let (mut a, mut b) = ([fnum(sh.get("x1"), 0.0), fnum(sh.get("x2"), 0.0)], [fnum(sh.get("y1"), 0.0), fnum(sh.get("y2"), 0.0)]);
        a.sort_by(|p, q| p.partial_cmp(q).unwrap()); b.sort_by(|p, q| p.partial_cmp(q).unwrap());
        for iy in ((b[0] - y0) / res) as i64..=(((b[1] - y0) / res).ceil() as i64) { for ix in ((a[0] - x0) / res) as i64..=(((a[1] - x0) / res).ceil() as i64) { setp(ix, iy, 0); } }
    }
    let half = wall_thick / 2.0;
    for [ax, ay, bx_, by_] in walls {
        let l = ((bx_ - ax).powi(2) + (by_ - ay).powi(2)).sqrt();
        let n = ((l / (res * 0.5)) as i64).max(1);
        for k in 0..=n {
            let px = ax + (bx_ - ax) * k as f64 / n as f64;
            let py = ay + (by_ - ay) * k as f64 / n as f64;
            let r = (half / res).ceil() as i64;
            let (cx, cy) = (((px - x0) / res) as i64, ((py - y0) / res) as i64);
            for dy in -r..=r { for dx in -r..=r { setp(cx + dx, cy + dy, 0); } }
        }
    }
    (img, w, h, (x0, y0))
}

fn yaml_text(d: &Map<String, Value>) -> String {
    let mut out = vec![];
    for k in ["image", "mode", "resolution", "origin", "negate", "occupied_thresh", "free_thresh"] {
        if let Some(v) = d.get(k) {
            out.push(match v {
                Value::Array(a) => format!("{k}: [{}]", a.iter().map(|x| x.as_f64().map(pf).unwrap_or_else(|| x.to_string())).collect::<Vec<_>>().join(", ")),
                Value::Number(n) => format!("{k}: {}", if n.is_f64() { pf(n.as_f64().unwrap()) } else { n.to_string() }),
                Value::String(s) => format!("{k}: {s}"), o => format!("{k}: {o}"),
            });
        }
    }
    out.join("\n") + "\n"
}

fn pystr_id(v: Option<&Value>, d: String) -> String {
    match v { Some(Value::String(s)) if !s.is_empty() => s.clone(), Some(Value::Number(n)) if n.as_f64() != Some(0.0) => n.to_string(), _ => d }
}

pub fn normalize(sc: &Value, sid: &str) -> Value {
    let mut sc = sc.as_object().cloned().unwrap_or_default();
    sc.insert("id".into(), json!(sid));
    if !sc.contains_key("name") { sc.insert("name".into(), json!(sid)); }
    if !sc.contains_key("description") { sc.insert("description".into(), json!("")); }
    let mut walls: Vec<[f64; 4]> = sc.get("walls").and_then(|w| w.as_array()).cloned().unwrap_or_default().iter().map(|w| [fnum(w.get(0), 0.0), fnum(w.get(1), 0.0), fnum(w.get(2), 0.0), fnum(w.get(3), 0.0)]).collect();
    if !walls.is_empty() {
        let xs: Vec<f64> = walls.iter().flat_map(|w| [w[0], w[2]]).collect();
        let ys: Vec<f64> = walls.iter().flat_map(|w| [w[1], w[3]]).collect();
        let b0: Vec<[f64; 4]> = walls.iter().take(4).cloned().collect();
        let bx: Vec<f64> = b0.iter().flat_map(|w| [w[0], w[2]]).collect();
        let by: Vec<f64> = b0.iter().flat_map(|w| [w[1], w[3]]).collect();
        let (mn, mxf) = (|v: &[f64]| v.iter().cloned().fold(f64::MAX, f64::min), |v: &[f64]| v.iter().cloned().fold(f64::MIN, f64::max));
        if walls.len() < 4 || mn(&bx) > mn(&xs) + 1e-6 || mxf(&bx) < mxf(&xs) - 1e-6 || mn(&by) > mn(&ys) + 1e-6 || mxf(&by) < mxf(&ys) - 1e-6 {
            let (x0, y0, x1, y1) = (mn(&xs) - 1.0, mn(&ys) - 1.0, mxf(&xs) + 1.0, mxf(&ys) + 1.0);
            let mut nw = vec![[x0, y0, x1, y0], [x1, y0, x1, y1], [x1, y1, x0, y1], [x0, y1, x0, y0]];
            nw.extend(walls.iter().cloned());
            walls = nw;
        }
    }
    sc.insert("walls".into(), json!(walls));
    if !sc.contains_key("shelves") { sc.insert("shelves".into(), json!([])); }
    let stations: Vec<Value> = sc.get("stations").and_then(|s| s.as_array()).cloned().unwrap_or_default().iter().enumerate().map(|(i, s)| {
        let mut m = s.as_object().cloned().unwrap_or_default();
        let id = pystr_id(s.get("id"), format!("P{i}"));
        let name = match s.get("name") { Some(Value::String(n)) if !n.is_empty() => n.clone(), _ => id.clone() };
        let dy = if s.get("dock_yaw").is_some() { fnum(s.get("dock_yaw"), 0.0) } else { fnum(s.get("yaw"), 0.0) };
        m.insert("id".into(), json!(id)); m.insert("name".into(), json!(name)); m.insert("dock_yaw".into(), json!(dy));
        Value::Object(m)
    }).collect();
    sc.insert("stations".into(), json!(stations));
    let nodes_v = sc.get("nodes").cloned().unwrap_or(json!({}));
    let mut nodes = Map::new();
    match &nodes_v {
        Value::Array(a) => for n in a { nodes.insert(pystr_id(n.get("id"), String::new()), json!([fnum(n.get("x"), 0.0), fnum(n.get("y"), 0.0)])); },
        Value::Object(o) => for (k, v) in o { nodes.insert(k.clone(), json!([fnum(v.get(0), 0.0), fnum(v.get(1), 0.0)])); },
        _ => {}
    }
    sc.insert("nodes".into(), Value::Object(nodes));
    let conns = sc.get("connections").or_else(|| sc.get("edges")).and_then(|c| c.as_array()).cloned().unwrap_or_default();
    let conns: Vec<Value> = conns.iter().map(|c| json!([pystr_id(c.get(0), String::new()), pystr_id(c.get(1), String::new())])).collect();
    sc.insert("connections".into(), json!(conns));
    sc.remove("edges");
    if !truthy(sc.get("origin")) {
        let sts = sc["stations"].as_array().cloned().unwrap_or_default();
        let s0 = sts.iter().find(|s| sget(s, "id").to_uppercase() == "P0").or_else(|| sts.first()).cloned();
        let o = if let Some(s0) = s0 { json!({"x": s0["x"], "y": s0["y"], "yaw": s0.get("dock_yaw").cloned().unwrap_or(json!(0.0))}) }
            else if !walls.is_empty() { json!({"x": (walls[0][0] + walls[1][0]) / 2.0, "y": (walls[0][1] + walls[1][3]) / 2.0, "yaw": 0.0}) }
            else { json!({"x": 0.0, "y": 0.0, "yaw": 0.0}) };
        sc.insert("origin".into(), o);
    }
    if !sc.contains_key("meta") { sc.insert("meta".into(), json!({})); }
    Value::Object(sc)
}

pub fn default_taskflows(sc: &Value) -> Value {
    let st: Vec<Value> = sc.get("stations").and_then(|s| s.as_array()).cloned().unwrap_or_default().into_iter().take(4).collect();
    if st.len() < 2 { return json!([]); }
    let mut steps = vec![];
    for (i, s) in st[1..].iter().enumerate() {
        steps.push(json!({"type": "move", "target": s["id"], "speed": 1.0}));
        steps.push(json!({"type": if i % 2 == 0 { "lift" } else { "drop" }, "target": s["id"]}));
    }
    steps.push(json!({"type": "move", "target": st[0]["id"], "speed": 1.0}));
    let ids: Vec<String> = st.iter().map(|s| sget(s, "id").to_string()).collect();
    json!([{"id": "TF-01", "name": format!("{} 示例巡回", sget(sc, "name")), "tid": "26001", "loop": "single", "description": format!("{} → {}", ids.join(" → "), ids[0]), "steps": steps}])
}

// ------------------------------------------------------------------ repository
pub struct SceneRepo { pub s: Arc<Store> }

impl SceneRepo {
    fn sdir(&self, sid: &str) -> PathBuf { self.s.path(&["scenes", sid]) }
    pub fn get(&self, sid: &str) -> R<Value> { self.s.get("scenes", sid).ok_or_else(|| err(404, format!("场景 {sid} 不存在"))) }
    pub fn scene_def(&self, sid: &str) -> R<Value> { self.get(sid)?; Store::read_json(&self.sdir(sid).join("scene.json")).ok_or_else(|| err(500, "场景定义缺失")) }

    pub fn seed_builtins(&self) {
        let all: Map<String, Value> = serde_json::from_str::<Value>(SCENARIOS).ok().and_then(|v| v.as_object().cloned()).unwrap_or_default();
        for (sid, sc) in all {
            if self.s.get("scenes", &sid).is_some() { continue; }
            let mut d = sc.clone();
            if !d.get("meta").map(|m| m.is_object()).unwrap_or(false) { d["meta"] = json!({}); }
            d["meta"]["resolution"] = json!(0.05);
            self.save(&sid, &d, json!({"builtin": true, "tags": [builtin_tag(&sid)], "source": "内置场景库"}));
        }
    }

    pub fn summarize(sc: &Value, sdir: Option<&PathBuf>) -> Value {
        let walls = sc.get("walls").and_then(|w| w.as_array()).cloned().unwrap_or_default();
        let mut pts: Vec<(f64, f64)> = walls.iter().map(|w| (fnum(w.get(0), 0.0), fnum(w.get(1), 0.0))).collect();
        pts.extend(walls.iter().map(|w| (fnum(w.get(2), 0.0), fnum(w.get(3), 0.0))));
        let b = if pts.is_empty() { [0.0; 4] } else {
            [pts.iter().map(|p| p.0).fold(f64::MAX, f64::min), pts.iter().map(|p| p.1).fold(f64::MAX, f64::min), pts.iter().map(|p| p.0).fold(f64::MIN, f64::max), pts.iter().map(|p| p.1).fold(f64::MIN, f64::max)] };
        let (w, h) = (b[2] - b[0], b[3] - b[1]);
        let meta = sc.get("meta").cloned().unwrap_or(json!({}));
        let res = if meta.get("resolution").is_some() { fnum(meta.get("resolution"), 0.05) } else { 0.05 };
        let has_pgm = sdir.map(|d| d.join("map.pgm").exists()).unwrap_or(false);
        let mesh = sdir.and_then(|d| std::fs::read_dir(d).ok()).and_then(|rd| rd.filter_map(|e| e.ok()).map(|e| e.file_name().to_string_lossy().to_string()).find(|f| f.to_lowercase().starts_with("mesh.")));
        let cnt = |k: &str| sc.get(k).map(|v| match v { Value::Array(a) => a.len(), Value::Object(o) => o.len(), _ => 0 }).unwrap_or(0);
        json!({"bounds": b, "size": [rnd(w, 2), rnd(h, 2)], "area_m2": rnd(w * h, 1), "resolution": res, "grid": [(w / res).ceil() as i64, (h / res).ceil() as i64],
            "walls": walls.len(), "shelves": cnt("shelves"), "stations": cnt("stations"), "nodes": cnt("nodes"), "edges": cnt("connections"), "reflectors": cnt("reflectors"),
            "map": if has_pgm { "pgm" } else { "generated" }, "mesh": mesh, "origin": sc.get("origin").cloned().unwrap_or(Value::Null),
            "ceiling_m": meta.get("ceiling_m").cloned().unwrap_or(json!(6.0)), "shelf_height_m": meta.get("shelf_height_m").cloned().unwrap_or(json!(2.5))})
    }
    pub fn view(&self, d: &Value) -> Value { let mut v = d.clone(); v["summary"] = if truthy(d.get("summary")) { d["summary"].clone() } else { json!({}) }; v }
    pub fn list(&self) -> Vec<Value> {
        let mut ds = self.s.list("scenes");
        ds.sort_by(|a, b| (!truthy(a.get("builtin")), fnum(a.get("created"), 0.0)).partial_cmp(&(!truthy(b.get("builtin")), fnum(b.get("created"), 0.0))).unwrap_or(std::cmp::Ordering::Equal));
        ds.iter().map(|d| self.view(d)).collect()
    }

    fn save(&self, sid: &str, sc: &Value, extra: Value) -> Value {
        let sc = normalize(sc, sid);
        let sd = self.sdir(sid);
        let _ = std::fs::create_dir_all(&sd);
        self.s.write_json(&sd.join("scene.json"), &sc);
        let mut d = self.s.get("scenes", sid).unwrap_or(json!({"id": sid, "created": now()}));
        d["name"] = sc["name"].clone();
        d["description"] = sc.get("description").cloned().unwrap_or(json!(""));
        if let Some(e) = extra.as_object() { for (k, v) in e { d[k] = v.clone(); } }
        d["summary"] = Self::summarize(&sc, Some(&sd));
        self.s.put("scenes", d)
    }

    pub fn import_upload(&self, filename: &str, data: &[u8], meta: &Map<String, Value>) -> R<Value> {
        let fn_l = filename.to_lowercase();
        let mut files: Vec<(String, Vec<u8>)> = vec![];
        if fn_l.ends_with(".zip") {
            let mut z = zip::ZipArchive::new(Cursor::new(data)).map_err(|_| err(400, "不是有效的 zip 场景包"))?;
            for i in 0..z.len() {
                let mut f = z.by_index(i).map_err(|_| err(400, "不是有效的 zip 场景包"))?;
                let n = f.name().to_string();
                if n.ends_with('/') { continue; }
                let mut b = vec![];
                f.read_to_end(&mut b).map_err(|e| err(400, e.to_string()))?;
                let base = n.rsplit('/').next().unwrap_or("").to_lowercase();
                files.retain(|(k, _)| *k != base);
                files.push((base, b));
            }
        } else if fn_l.ends_with(".json") { files.push(("scene.json".into(), data.to_vec())); }
        else { return Err(err(400, "场景包需为 .zip (scene.json / map.pgm+map.yaml / topology.json / mesh.obj|glb) 或 scene.json")); }
        let find = |p: &dyn Fn(&str) -> bool| files.iter().find(|(k, _)| p(k)).map(|(_, v)| v.clone());
        let mut sc: Value = json!({});
        let sj = find(&|k| k == "scene.json").or_else(|| find(&|k| k.ends_with(".json") && !k.contains("topo")));
        if let Some(sj) = sj { sc = serde_json::from_slice(&sj).map_err(|e| err(400, format!("scene.json 解析失败: {e}")))?; }
        if !sc.is_object() { sc = json!({}); }
        let topo = find(&|k| k == "topology.json").or_else(|| find(&|k| k.contains("topo") && k.ends_with(".json")));
        if let Some(t) = topo {
            let t: Value = serde_json::from_slice(&t).map_err(|e| err(400, e.to_string()))?;
            for k in ["nodes", "connections", "stations"] { if truthy(t.get(k)) { sc[k] = t[k].clone(); } }
        }
        let pgm = find(&|k| k.ends_with(".pgm"));
        let yml = find(&|k| k.ends_with(".yaml") || k.ends_with(".yml"));
        let mut ymap = yml.map(|y| parse_yaml_map(&String::from_utf8_lossy(&y))).unwrap_or_default();
        if let Some(pg) = &pgm {
            let (w, h, _mx, px) = parse_pgm(pg).map_err(|e| err(400, format!("PGM 解析失败: {e}")))?;
            let res = fnum(ymap.get("resolution"), 0.05);
            let org: Vec<f64> = ymap.get("origin").and_then(|o| o.as_array()).map(|a| a.iter().map(|x| fnum(Some(x), 0.0)).collect()).unwrap_or(vec![0.0, 0.0, 0.0]);
            if !sc.get("meta").map(|m| m.is_object()).unwrap_or(false) { sc["meta"] = json!({}); }
            sc["meta"]["resolution"] = json!(res); sc["meta"]["map_origin"] = json!([org[0], org[1]]); sc["meta"]["map_size_px"] = json!([w, h]);
            if !truthy(sc.get("walls")) {
                let inner = grid_to_walls(w, h, &px, res, (org[0], org[1]), fnum(ymap.get("negate"), 0.0) != 0.0, if ymap.contains_key("occupied_thresh") { fnum(ymap.get("occupied_thresh"), 0.65) } else { 0.65 }, 4000);
                let (x0, y0) = (org[0], org[1]);
                let (x1, y1) = (x0 + w as f64 * res, y0 + h as f64 * res);
                let mut walls = vec![json!([x0, y0, x1, y0]), json!([x1, y0, x1, y1]), json!([x1, y1, x0, y1]), json!([x0, y1, x0, y0])];
                walls.extend(inner);
                sc["walls"] = Value::Array(walls);
                sc["meta"]["walls_from_grid"] = json!(true);
            }
        }
        if !truthy(sc.get("walls")) { return Err(err(400, "场景包缺少几何: 需要 scene.json 的 walls，或 map.pgm + map.yaml")); }
        let mget = |k: &str| meta.get(k).and_then(|v| v.as_str()).unwrap_or("");
        let stem = filename.rsplit_once('.').map(|x| x.0).unwrap_or(filename);
        let name = [mget("name"), sget(&sc, "name"), stem].iter().find(|s| !s.is_empty()).unwrap().trim().to_string();
        sc["name"] = json!(name);
        if !mget("description").is_empty() { sc["description"] = json!(mget("description")); }
        let given = mget("scene_id");
        let mut sid = if !given.is_empty() { given.to_string() } else { format!("s-{}", slug(&name, "scene")) };
        let base = sid.clone();
        let mut i = 2;
        while self.s.get("scenes", &sid).is_some() && given.is_empty() { sid = format!("{base}-{i}"); i += 1; }
        let sd = self.sdir(&sid);
        if sd.is_dir() { let _ = std::fs::remove_dir_all(&sd); }
        let _ = std::fs::create_dir_all(&sd);
        if let Some(pg) = &pgm {
            let _ = std::fs::write(sd.join("map.pgm"), pg);
            ymap.insert("image".into(), json!("map.pgm"));
            let _ = std::fs::write(sd.join("map.yaml"), yaml_text(&ymap));
        }
        for (k, v) in &files {
            if [".obj", ".glb", ".gltf", ".stl"].iter().any(|e| k.ends_with(e)) {
                let ext = k.rfind('.').map(|p| &k[p..]).unwrap_or("");
                let _ = std::fs::write(sd.join(format!("mesh{ext}")), v);
                break;
            }
        }
        let tags: Vec<Value> = match meta.get("tags") { Some(Value::String(t)) => t.split(',').filter(|x| !x.trim().is_empty()).map(|x| json!(x)).collect(), Some(Value::Array(a)) => a.clone(), _ => vec![] };
        Ok(self.save(&sid, &sc, json!({"builtin": false, "tags": if tags.is_empty() { vec![json!("自定义")] } else { tags }, "source": filename, "operator": mget("operator")})))
    }

    pub fn update_meta(&self, sid: &str, f: &Value) -> R<Value> {
        let mut d = self.get(sid)?;
        let mut sc = self.scene_def(sid)?;
        for k in ["name", "description"] { if let Some(v) = f.get(k) { sc[k] = v.clone(); d[k] = v.clone(); } }
        if let Some(t) = f.get("tags") { d["tags"] = t.clone(); }
        self.s.write_json(&self.sdir(sid).join("scene.json"), &sc);
        let d = self.s.put("scenes", d);
        Ok(self.view(&d))
    }
    pub fn delete(&self, sid: &str) -> R<()> {
        let d = self.get(sid)?;
        if truthy(d.get("builtin")) { return Err(err(400, "内置场景不能删除")); }
        let _ = std::fs::remove_dir_all(self.sdir(sid));
        self.s.delete("scenes", sid);
        Ok(())
    }

    pub fn map_files(&self, sid: &str) -> R<(Vec<u8>, String)> {
        let sd = self.sdir(sid);
        let (pgm_p, yml_p) = (sd.join("map.pgm"), sd.join("map.yaml"));
        if pgm_p.exists() && yml_p.exists() { return Ok((std::fs::read(&pgm_p).map_err(|e| err(500, e.to_string()))?, std::fs::read_to_string(&yml_p).map_err(|e| err(500, e.to_string()))?)); }
        let (gen_p, gyml_p) = (sd.join("map.gen.pgm"), sd.join("map.gen.yaml"));
        if !gen_p.exists() {
            let sc = self.scene_def(sid)?;
            let res = fnum(sc.get("meta").and_then(|m| m.get("resolution")), 0.05);
            let (img, w, h, (ox, oy)) = rasterize(&sc, res);
            let mut out = format!("P5\n# scene {sid}\n{w} {h}\n255\n").into_bytes();
            out.extend_from_slice(&img);
            let _ = std::fs::write(&gen_p, out);
            let _ = std::fs::write(&gyml_p, format!("image: map.pgm\nmode: trinary\nresolution: {}\norigin: [{ox:.3}, {oy:.3}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n", pf(res)));
        }
        Ok((std::fs::read(&gen_p).map_err(|e| err(500, e.to_string()))?, std::fs::read_to_string(&gyml_p).map_err(|e| err(500, e.to_string()))?))
    }

    pub fn package_zip(&self, sid: &str) -> R<Vec<u8>> {
        let sc = self.scene_def(sid)?;
        let sd = self.sdir(sid);
        let (pgm, yml) = self.map_files(sid)?;
        let mut buf = Cursor::new(vec![]);
        {
            let mut z = zip::ZipWriter::new(&mut buf);
            let o = zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);
            let io = |e: zip::result::ZipError| err(500, e.to_string());
            let w = |z: &mut zip::ZipWriter<&mut Cursor<Vec<u8>>>, n: &str, b: &[u8]| -> R<()> { z.start_file(n, o).map_err(io)?; z.write_all(b).map_err(|e| err(500, e.to_string())) };
            w(&mut z, "scene.json", serde_json::to_string_pretty(&sc).unwrap().as_bytes())?;
            w(&mut z, "topology.json", serde_json::to_string_pretty(&json!({"nodes": sc.get("nodes").cloned().unwrap_or(json!({})), "connections": sc.get("connections").cloned().unwrap_or(json!([])), "stations": sc.get("stations").cloned().unwrap_or(json!([]))})).unwrap().as_bytes())?;
            w(&mut z, "map.pgm", &pgm)?;
            w(&mut z, "map.yaml", yml.replace("map.gen.pgm", "map.pgm").as_bytes())?;
            if let Ok(rd) = std::fs::read_dir(&sd) { for e in rd.filter_map(|e| e.ok()) { let n = e.file_name().to_string_lossy().to_string(); if n.starts_with("mesh.") { if let Ok(b) = std::fs::read(e.path()) { w(&mut z, &n, &b)?; } } } }
            if let Ok(b) = std::fs::read(sd.join("taskflows.json")) { w(&mut z, "taskflows.json", &b)?; }
            z.finish().map_err(io)?;
        }
        Ok(buf.into_inner())
    }

    pub fn taskflows(&self, sid: &str) -> R<Value> {
        self.get(sid)?;
        match Store::read_json(&self.sdir(sid).join("taskflows.json")) { Some(t) => Ok(t), None => Ok(default_taskflows(&self.scene_def(sid)?)) }
    }
    pub fn save_taskflows(&self, sid: &str, flows: &Value) -> R<Value> {
        self.get(sid)?;
        if !flows.is_array() { return Err(err(400, "任务流需为数组")); }
        self.s.write_json(&self.sdir(sid).join("taskflows.json"), flows);
        Ok(flows.clone())
    }
}
