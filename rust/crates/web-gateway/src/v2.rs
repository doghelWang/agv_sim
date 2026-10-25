//! Gateway v2 (port of gateway_v2.py): control lock, injection, task recorder, SLAM/safety pass-through
use crate::gw::{safe, safe_t, Gateway};
use crate::util::*;
use rest::{ok, problem, ApiError, HResult, Request, Response, Server};
use serde_json::{json, Value};
use std::io::Write;
use std::sync::{Arc, Mutex, MutexGuard};

const LOCK_TTL: f64 = 30.0;

fn catalog() -> Value {
    json!([{"type": "pallet", "name": "标准木质栈板", "icon": "🪵", "w": 1.2, "h": 1.0, "z": 0.15, "desc": "1.2 × 1.0 × 0.15m"},
        {"type": "shelf", "name": "双层轻型货架", "icon": "🏗️", "w": 2.0, "h": 1.0, "z": 2.2, "desc": "2.0 × 1.0 × 2.2m"},
        {"type": "box", "name": "工业周转纸箱", "icon": "📦", "w": 0.8, "h": 0.8, "z": 0.8, "desc": "0.8 × 0.8 × 0.8m"},
        {"type": "person", "name": "车间作业人员", "icon": "👷", "w": 0.5, "h": 0.5, "z": 1.7, "desc": "反光工装 · 可行走"}])
}
fn placements() -> Value {
    json!([{"id": "ahead_2.5", "name": "车身正前方 2.5 米 (规划路径线上)"}, {"id": "ahead_1.2", "name": "车身正前方 1.2 米 (近距减速区)"}, {"id": "next_station", "name": "下一工位站点附近"},
        {"id": "random", "name": "随机拓扑节点附近"}, {"id": "custom", "name": "自定义场景坐标 (画布点选/手动输入)"}])
}
const TAG_BY_TYPE: [(&str, &str); 24] = [("TASKFLOW", "TSK"), ("MISSION", "NAV"), ("PLAN", "NAV"), ("NAV2", "NAV"), ("PLANNER", "NAV"), ("REPLAN", "NAV"), ("ARRIVED", "NAV"), ("OBS_", "OBS"), ("OBSTACLE_WAIT", "OBS"),
    ("INJECT", "INJ"), ("OBSTACLE_CHANGE", "INJ"), ("SIM_RESET", "RST"), ("RESET", "RST"), ("CHASSIS", "DRV"), ("WHEEL", "DRV"), ("SLIP", "DRV"), ("ESTOP", "SAF"), ("BUMPER", "SAF"), ("COLLISION", "SAF"),
    ("SAFETY", "SAF"), ("PHOTO", "SAF"), ("DI_", "SAF"), ("", "SYS"), ("", "SYS")];
pub fn tag_of(ev: &Value) -> &'static str {
    let t = s(ev, "type").to_uppercase();
    for (k, v) in TAG_BY_TYPE.iter() { if !k.is_empty() && t.starts_with(k) { return v; } }
    match s(ev, "category") { "task" => "TSK", "navigation" => "NAV", "safety" => "SAF", "sensors" => "OBS", "chassis" => "DRV", _ => "SYS" }
}

pub struct Holder { user: String, token: String, since: f64, renewed: f64 }
pub struct Locks { h: Mutex<Option<Holder>> }
fn eqt(a: &str, b: &str) -> bool { a.len() == b.len() && a.bytes().zip(b.bytes()).fold(0, |x, (p, q)| x | (p ^ q)) == 0 }
fn token() -> String { (0..22).map(|_| { let c = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_"; c[(rand_u64() % 64) as usize] as char }).collect() }
impl Locks {
    fn g(&self) -> MutexGuard<'_, Option<Holder>> { self.h.lock().unwrap_or_else(|e| e.into_inner()) }
    fn expire(g: &mut Option<Holder>) { if g.as_ref().map(|h| now() - h.renewed > LOCK_TTL).unwrap_or(false) { *g = None; } }
    pub fn status(&self, tok: &str) -> Value {
        let mut g = self.g(); Self::expire(&mut g);
        match &*g { None => json!({"held": false}), Some(h) => json!({"held": true, "user": h.user, "since": h.since, "mine": !tok.is_empty() && eqt(tok, &h.token), "ttl": rnd(LOCK_TTL - (now() - h.renewed), 1)}) }
    }
    pub fn acquire(&self, user: &str, force: bool) -> Result<Value, ApiError> {
        let mut g = self.g(); Self::expire(&mut g);
        if let Some(h) = &*g { if !force && h.user != user { return Err(ApiError::coded(409, format!("控制权已被 {} 占用", h.user), "locked")); } }
        let user = if user.is_empty() { "未命名用户".to_string() } else { user.to_string() };
        let h = Holder { user: user.clone(), token: token(), since: now(), renewed: now() };
        let r = json!({"held": true, "user": user, "token": h.token, "mine": true, "ttl": LOCK_TTL});
        *g = Some(h);
        Ok(r)
    }
    pub fn renew(&self, tok: &str) -> Result<Value, ApiError> {
        let mut g = self.g(); Self::expire(&mut g);
        match g.as_mut() { Some(h) if eqt(tok, &h.token) => { h.renewed = now(); Ok(json!({"held": true, "user": h.user, "mine": true, "ttl": LOCK_TTL})) } _ => Err(ApiError::coded(409, "控制权已失效，请重新获取", "lock_lost")) }
    }
    pub fn release(&self, tok: &str) -> Value { let mut g = self.g(); if g.as_ref().map(|h| eqt(tok, &h.token)).unwrap_or(false) { *g = None; } json!({"held": false}) }
    pub fn allowed(&self, tok: &str) -> Option<String> {
        let mut g = self.g(); Self::expire(&mut g);
        match &*g { None => None, Some(h) => if !tok.is_empty() && eqt(tok, &h.token) { None } else { Some(h.user.clone()) } }
    }
}

struct Active { id: String, t0: f64, flow: Value, ev_from: u64, traj: Vec<Value>, inj: Vec<Value>, info: Value, scene: Value, topo: Value, spec: Value }
struct Rec { on: bool, active: Option<Active>, records: Vec<Value> }

fn light(mut b: Value) -> Value { if let Some(o) = b.as_object_mut() { o.remove("trajectory"); o.remove("events"); } b }

pub struct V2 { pub gw: Arc<Gateway>, pub locks: Locks, rec: Mutex<Rec>, dir: String, info: Mutex<(f64, Value)>, api: Mutex<Option<Arc<Server>>> }

impl V2 {
    pub fn new(gw: Arc<Gateway>) -> Arc<V2> {
        let dir = format!("{}/records", std::env::var("AGV_DATA").or_else(|_| std::env::var("AGV_HOME")).unwrap_or(".".into()));
        let _ = std::fs::create_dir_all(&dir);
        let mut records: Vec<Value> = vec![];
        if let Ok(rd) = std::fs::read_dir(&dir) {
            let mut fs: Vec<String> = rd.flatten().map(|e| e.file_name().to_string_lossy().to_string()).filter(|f| f.ends_with(".json")).collect();
            fs.sort(); fs.reverse(); fs.truncate(50);
            for f in fs {
                // 超大记录(>8MB)不整体解析进内存，留在磁盘；其余只保留轻量字段，完整 bundle 按需从磁盘读取
                if std::fs::metadata(format!("{dir}/{f}")).map(|m| m.len() > 8_000_000).unwrap_or(true) { continue; }
                if let Some(b) = std::fs::read_to_string(format!("{dir}/{f}")).ok().and_then(|t| serde_json::from_str::<Value>(&t).ok()) { records.push(light(b)); }
            }
            records.sort_by(|a, b| s(&b["metadata"], "recordedAt").cmp(s(&a["metadata"], "recordedAt")));
        }
        let v = Arc::new(V2 { gw, locks: Locks { h: Mutex::new(None) }, rec: Mutex::new(Rec { on: true, active: None, records }), dir, info: Mutex::new((0.0, json!({}))), api: Mutex::new(None) });
        let _ = GLOBAL.set(v.clone());
        let v2 = v.clone();
        std::thread::Builder::new().name("task-recorder".into()).spawn(move || loop { std::thread::sleep(std::time::Duration::from_millis(100)); v2.sample(); }).ok();
        let api = Arc::new(v.build());
        *v.api.lock().unwrap() = Some(api);
        v
    }
    fn rec(&self) -> MutexGuard<'_, Rec> { self.rec.lock().unwrap_or_else(|e| e.into_inner()) }

    pub fn info(&self) -> Value {
        let mut c = self.info.lock().unwrap_or_else(|e| e.into_inner());
        if now() - c.0 > 5.0 || !truthy(&c.1) {
            let inst = safe(&self.gw.sim, "GET", "/api/v1/instance", None).unwrap_or(json!({}));
            let envv = |k: &str| std::env::var(k).ok().map(|v| json!(v)).unwrap_or(Value::Null);
            let or_env = |k: &str, e: &str| inst.get(k).filter(|v| truthy(v)).cloned().unwrap_or_else(|| envv(e));
            let engine = self.gw.lock().telemetry.get("sim_status").and_then(|s| s.get("backend")).cloned().unwrap_or(Value::Null);
            c.1 = json!({"instance_id": or_env("instance_id", "INSTANCE_ID"), "instance_name": or_env("instance_name", "INSTANCE_NAME"), "sim_api": self.gw.sim_api, "nav_api": self.gw.nav_api,
                "hub": or_env("hub", "HUB_API"), "model": inst.get("model").cloned().unwrap_or(json!({})), "scene": inst.get("scene").cloned().unwrap_or(json!({})), "engine": engine});
            c.0 = now();
        }
        let mut d = c.1.clone();
        d["sim_online"] = json!(self.gw.sim_online()); d["nav_online"] = json!(self.gw.nav_online());
        d
    }
    fn reset_info(&self) { self.info.lock().unwrap_or_else(|e| e.into_inner()).0 = 0.0; }

    fn brief(&self) -> Value {
        let (t, paused) = { let g = self.gw.lock(); (g.telemetry.clone(), g.is_paused) };
        let _ = paused;
        let tv = Value::Object(t);
        let tf = tv.get("taskflow").cloned().filter(|v| v.is_object()).unwrap_or(json!({}));
        let steps = tf.get("steps").and_then(|s| s.as_array()).cloned().unwrap_or_default();
        let i = tf.get("step_index").and_then(|i| i.as_u64());
        json!({"sim_online": self.gw.sim_online(), "nav_online": self.gw.nav_online(), "nav_status": tv.get("nav_status").cloned().unwrap_or(Value::Null), "paused": tv.get("is_paused").cloned().unwrap_or(Value::Null),
            "pose": {"x": rnd(f(&tv, "x", 0.0), 2), "y": rnd(f(&tv, "y", 0.0), 2), "yaw": rnd(f(&tv, "yaw", 0.0), 3)}, "planner": tv.get("planner_type").cloned().unwrap_or(Value::Null),
            "nav2_ready": tv.get("nav2").and_then(|n| n.get("server_ready")).cloned().unwrap_or(Value::Null),
            "taskflow": {"status": tf.get("status").cloned().unwrap_or(Value::Null), "name": tf.pointer("/flow/name").cloned().unwrap_or(Value::Null), "tid": tf.pointer("/flow/tid").cloned().unwrap_or(Value::Null),
                "step_index": tf.get("step_index").cloned().unwrap_or(Value::Null), "step": i.and_then(|i| steps.get(i as usize).cloned()).unwrap_or(Value::Null), "total": steps.len()},
            "lock": self.locks.status(""), "records": self.rec().records.len()})
    }

    pub fn archive(&self, bundle: &Value) -> bool {
        let hub = self.info().get("hub").and_then(|h| h.as_str()).unwrap_or("").to_string();
        if hub.is_empty() { return false; }
        let c = rest::Client::new(hub.trim_end_matches('/'), 15.0);
        match c.json("POST", "/api/hub/records", Some(bundle), Some(15.0)) {
            Ok(_) => { let id = s(&bundle["metadata"], "recordId").to_string(); let mut r = self.rec(); for b in r.records.iter_mut() { if s(&b["metadata"], "recordId") == id { b["metadata"]["archived"] = json!(true); } } true }
            Err(e) => { eprintln!("[gateway-v2] 记录归档到平台失败: {e}"); false }
        }
    }

    // ---- task recorder
    fn rec_start(&self, flow: &Value) {
        let mut r = self.rec();
        if r.active.is_some() { self.finish(&mut r, "SUPERSEDED"); }
        if !r.on { return; }
        let info = { drop(r); let i = self.info(); r = self.rec(); i };
        let (scene, topo, spec) = { let g = self.gw.lock(); (g.telemetry.get("scenario_metadata").cloned().unwrap_or(json!({})), g.telemetry.get("topo_graph").cloned().filter(truthy).unwrap_or(json!({})), g.telemetry.get("robot_spec").cloned().unwrap_or(json!({}))) };
        let t = now();
        let id = format!("{}{}", fmt_local(t, "%Y%m%d%H%M%S"), randint(10, 99));
        r.active = Some(Active { id, t0: t, flow: flow.clone(), ev_from: self.gw.events.counter(), traj: vec![], inj: vec![], info, scene, topo, spec });
    }
    fn rec_injected(&self, el: &Value) {
        let mut r = self.rec();
        if let Some(a) = r.active.as_mut() {
            a.inj.push(json!({"type": el["type"], "name": el.get("name"), "x": el["x"], "y": el["y"], "spawnTime": rnd(now() - a.t0, 2), "size": [el.get("w"), el.get("h"), el.get("z")], "motion": el.get("motion"), "id": el.get("id")}));
        }
    }
    fn rec_removed(&self, oid: i64) {
        let mut r = self.rec();
        if let Some(a) = r.active.as_mut() { let t0 = a.t0; for e in a.inj.iter_mut() { if e.get("id").and_then(|i| i.as_i64()) == Some(oid) && e.get("removedAt").is_none() { e["removedAt"] = json!(rnd(now() - t0, 2)); } } }
    }
    fn rec_end(&self, result: &str) -> Option<Value> { let mut r = self.rec(); self.finish(&mut r, result) }

    fn sample(&self) {
        if self.rec().active.is_none() { return; }   // don't clone the whole telemetry map (scan etc.) 10x/s when nothing is recording
        let (fr, tf) = {
            let g = self.gw.lock();
            let t = Value::Object(g.telemetry.clone());
            let safety = t.get("safety").cloned().filter(|v| v.is_object()).unwrap_or(json!({}));
            let obs = safety.pointer("/obstacle/front").filter(|v| !v.is_null()).cloned();
            let tf = t.get("taskflow").cloned().filter(|v| v.is_object()).unwrap_or(json!({}));
            (json!({"time": 0.0, "x": rnd(f(&t, "x", 0.0), 3), "y": rnd(f(&t, "y", 0.0), 3), "yaw": rnd(f(&t, "yaw", 0.0), 4), "vx": rnd(f(&t, "vx", 0.0), 3), "vy": rnd(f(&t, "vy", 0.0), 3), "w": rnd(f(&t, "wz", 0.0), 3),
                "obsDist": match obs { Some(o) => jr(o.as_f64().unwrap_or(0.0), 2), None => t.get("scan_min_dist").cloned().unwrap_or(Value::Null) },
                "status": t.get("nav_status").cloned().unwrap_or(json!("IDLE")), "safety": safety.pointer("/obstacle/zone").cloned().unwrap_or(Value::Null), "locErrMm": t.pointer("/localization/err_mm").cloned().unwrap_or(Value::Null)}), tf)
        };
        let mut r = self.rec();
        let Some(a) = r.active.as_mut() else { return };
        let mut fr = fr;
        fr["step"] = tf.get("step_index").cloned().unwrap_or(Value::Null);
        fr["time"] = jr(now() - a.t0, 2);
        a.traj.push(fr);
        let st = tf.get("status").and_then(|s| s.as_str()).unwrap_or("");
        if matches!(st, "done" | "failed" | "stopped") && now() - a.t0 > 1.5 && f(&tf, "started", 0.0) >= a.t0 - 1.0 {
            let res = match st { "done" => "SUCCESS", "failed" => "FAILED", _ => "STOPPED" };
            self.finish(&mut r, res);
        }
    }

    fn finish(&self, r: &mut Rec, result: &str) -> Option<Value> {
        let a = r.active.take()?;
        let dur = rnd(now() - a.t0, 2);
        let evs = self.gw.events.get_events(a.ev_from, None, 2000);
        let events: Vec<Value> = evs["events"].as_array().cloned().unwrap_or_default().iter().map(|e| json!({"time": rnd(f(e, "timestamp", a.t0) - a.t0, 2), "tag": tag_of(e), "title": e.get("title"), "detail": e.get("message"), "source": e.get("category"), "level": e.get("level")})).collect();
        let avoid = events.iter().filter(|e| e["tag"] == "OBS").count();
        let collided = events.iter().any(|e| e["tag"] == "SAF" && { let t = s(e, "title"); t.contains("碰撞") || t.contains("触边") });
        let mut result = result.to_string();
        if result == "SUCCESS" && !a.inj.is_empty() && avoid > 0 { result = "AVOIDED".into(); }
        if (result == "SUCCESS" || result == "AVOIDED") && collided { result = "COLLIDED".into(); }
        let (flow, info, sc, spec) = (&a.flow, &a.info, &a.scene, &a.spec);
        let b = sc.get("bounds").cloned().unwrap_or(json!({}));
        let ch = spec.get("chassis").cloned().unwrap_or(json!({}));
        let wheels: Vec<Value> = spec.get("wheels").and_then(|w| w.as_array()).cloned().unwrap_or_default();
        let xs: Vec<f64> = wheels.iter().map(|w| f(w, "x", 0.0)).collect();
        let model = info.get("model").cloned().unwrap_or(json!({}));
        let topo = &a.topo;
        let rid = a.id.clone();
        let tid = flow.get("tid").filter(|v| truthy(v)).cloned().unwrap_or(json!(rid));
        let tid_s = tid.as_str().map(|s| s.to_string()).unwrap_or(tid.to_string());
        let bundle = json!({"schema": "sim_bundle/1",
            "metadata": {"taskId": format!("TID-{tid_s}"), "taskName": flow.get("name"), "recordId": rid, "recordedAt": fmt_local(a.t0, "%Y-%m-%d %H:%M:%S"), "durationSeconds": dur, "result": result,
                "instanceId": info.get("instance_id"), "instanceName": info.get("instance_name"), "operator": self.locks.status("").get("user"), "simApi": info.get("sim_api"), "navApi": info.get("nav_api"), "avoidCount": avoid, "collided": collided},
            "taskFlow": {"id": flow.get("id"), "name": flow.get("name"), "tid": flow.get("tid"), "loop": flow.get("loop").cloned().unwrap_or(json!("single")), "steps": flow.get("steps").cloned().unwrap_or(json!([]))},
            "environment": {"sceneName": sc.get("name"), "sceneId": sc.get("id"), "size": [rnd(f(&b, "max_x", 0.0) - f(&b, "min_x", 0.0), 2), rnd(f(&b, "max_y", 0.0) - f(&b, "min_y", 0.0), 2)], "bounds": b, "slamResolution": 0.05,
                "topologyNodes": topo.get("nodes").and_then(|n| n.as_object()).map(|o| o.iter().map(|(k, v)| json!({"id": k, "x": v.get("x"), "y": v.get("y")})).collect::<Vec<_>>()).unwrap_or_default(),
                "topologyEdges": topo.get("edges").and_then(|e| e.as_array()).map(|a| a.iter().map(|e| json!([e.get("from"), e.get("to")])).collect::<Vec<_>>()).unwrap_or_default(),
                "stations": sc.get("stations").and_then(|s| s.as_array()).map(|a| a.iter().map(|s| json!({"id": s.get("id"), "name": s.get("name"), "x": s.get("x"), "y": s.get("y")})).collect::<Vec<_>>()).unwrap_or_default(),
                "walls": sc.get("walls").cloned().unwrap_or(json!([])), "shelves": sc.get("shelves").cloned().unwrap_or(json!([]))},
            "vehicleModel": {"modelName": model.get("name").filter(|v| truthy(v)).cloned().or_else(|| spec.get("model_file").cloned()), "materialNo": model.get("material_no"), "type": ch.get("type"),
                "wheelbase": if xs.len() > 1 { jr(xs.iter().cloned().fold(f64::MIN, f64::max) - xs.iter().cloned().fold(f64::MAX, f64::min), 3) } else { Value::Null }, "modelId": model.get("model_id"), "version": model.get("version"),
                "chassis": spec.get("active_chassis"), "footprint": ch.get("footprint").filter(|v| truthy(v)).cloned().or_else(|| spec.get("footprint").cloned()), "length": ch.get("length_m"), "width": ch.get("width_m")},
            "injectedElements": a.inj, "events": events, "trajectory": a.traj});
        r.records.insert(0, light(bundle.clone()));
        r.records.truncate(50);
        let _ = std::fs::write(format!("{}/{}.json", self.dir, rid), serde_json::to_vec(&bundle).unwrap_or_default());
        if let Ok(rd) = std::fs::read_dir(&self.dir) {
            let mut fs: Vec<String> = rd.flatten().map(|e| e.file_name().to_string_lossy().to_string()).collect(); fs.sort();
            if fs.len() > 50 { for old in &fs[..fs.len() - 50] { let _ = std::fs::remove_file(format!("{}/{old}", self.dir)); } }
        }
        self.gw.events.emit("task", "RECORD_SAVED", "info", &format!("仿真记录已保存 {}", bundle["metadata"]["taskId"].as_str().unwrap_or("")), &format!("时长 {dur}s，结果 {result}，轨迹 {} 帧", bundle["trajectory"].as_array().map(|a| a.len()).unwrap_or(0)), json!({"record": rid}));
        let me = self.me();
        let b2 = bundle.clone();
        std::thread::spawn(move || { me.archive(&b2); });
        Some(bundle)
    }
    /// 按需从磁盘读取完整 bundle（内存里只保留轻量记录）
    fn full(&self, b: &Value) -> Value {
        let id = s(&b["metadata"], "recordId");
        std::fs::read_to_string(format!("{}/{}.json", self.dir, id)).ok().and_then(|t| serde_json::from_str::<Value>(&t).ok()).unwrap_or_else(|| b.clone())
    }
    fn me(&self) -> Arc<V2> { GLOBAL.get().cloned().expect("v2") }

    fn summary(b: &Value) -> Value {
        let m = &b["metadata"];
        json!({"id": m.get("recordId"), "taskId": m.get("taskId"), "taskName": m.get("taskName"), "recordedAt": m.get("recordedAt"), "durationSeconds": m.get("durationSeconds"), "result": m.get("result"),
            "scene": b["environment"].get("sceneName"), "model": b["vehicleModel"].get("modelName"), "injections": b.get("injectedElements").and_then(|a| a.as_array()).map(|a| a.len()).unwrap_or(0),
            "avoidCount": m.get("avoidCount").cloned().unwrap_or(json!(0)), "archived": m.get("archived").cloned().unwrap_or(json!(false))})
    }
    fn stats(&self) -> Value {
        let r = self.rec();
        let n = r.records.len();
        let ok: Vec<&Value> = r.records.iter().filter(|x| matches!(s(&x["metadata"], "result"), "SUCCESS" | "AVOIDED")).collect();
        json!({"count": n, "success_rate": if n > 0 { jr(100.0 * ok.len() as f64 / n as f64, 1) } else { Value::Null }, "avoid_count": r.records.iter().map(|x| f(&x["metadata"], "avoidCount", 0.0) as i64).sum::<i64>(),
            "avg_duration": if !ok.is_empty() { jr(ok.iter().map(|x| f(&x["metadata"], "durationSeconds", 0.0)).sum::<f64>() / ok.len() as f64, 1) } else { Value::Null },
            "recording": r.on, "active": r.active.as_ref().map(|a| json!({"id": a.id, "elapsed": rnd(now() - a.t0, 1), "name": a.flow.get("name")})).unwrap_or(Value::Null)})
    }

    // ---- injection
    fn path_point(&self, dist: f64) -> (f64, f64, f64) {
        let (x, y, yaw, path, head, pidx) = {
            let g = self.gw.lock();
            let t = Value::Object(g.telemetry.clone());
            let path: Vec<(f64, f64)> = t.get("plan_path").and_then(|p| p.as_array()).map(|a| a.iter().filter(|p| p.is_object()).map(|p| (f(p, "x", 0.0), f(p, "y", 0.0))).collect()).unwrap_or_default();
            let head = t.pointer("/robot_spec/chassis/head_offset_m").and_then(|h| h.as_f64()).filter(|h| *h != 0.0).unwrap_or(0.5);
            (f(&t, "x", 0.0), f(&t, "y", 0.0), f(&t, "yaw", 0.0), path, head, t.get("path_index").and_then(|p| p.as_i64()).unwrap_or(0))
        };
        let dist = dist + head;
        if path.len() >= 2 {
            let k = if pidx >= 1 && (pidx as usize) < path.len() { pidx as usize - 1 } else {
                (0..path.len()).min_by(|a, b| (path[*a].0 - x).hypot(path[*a].1 - y).partial_cmp(&(path[*b].0 - x).hypot(path[*b].1 - y)).unwrap()).unwrap_or(0)
            };
            let mut pts = vec![(x, y)]; pts.extend_from_slice(&path[k + 1..]);
            let mut left = dist;
            for w in pts.windows(2) {
                let ((ax, ay), (bx, by)) = (w[0], w[1]);
                let l = (bx - ax).hypot(by - ay);
                if l >= left && l > 1e-6 { return (ax + (bx - ax) * left / l, ay + (by - ay) * left / l, (by - ay).atan2(bx - ax)); }
                left -= l;
            }
            if pts.len() >= 2 { let ((ax, ay), (bx, by)) = (pts[pts.len() - 2], pts[pts.len() - 1]); return (bx, by, (by - ay).atan2(bx - ax)); }
        }
        (x + dist * yaw.cos(), y + dist * yaw.sin(), yaw)
    }

    fn inject(&self, b: &Value) -> Result<Value, ApiError> {
        let cat = catalog().as_array().unwrap().iter().find(|c| Some(s(c, "type")) == b.get("type").and_then(|t| t.as_str())).cloned()
            .ok_or_else(|| ApiError::new(400, format!("未知元素类型 {}", b.get("type").map(|t| t.as_str().map(|s| s.to_string()).unwrap_or(t.to_string())).unwrap_or("None".into()))))?;
        let fork_cargo = { let g = self.gw.lock(); g.telemetry.get("fork").map(|v| v.is_object()).unwrap_or(false) } && matches!(s(&cat, "type"), "pallet" | "box");
        if !fork_cargo && !self.gw.is_paused() && !b.get("force").map(truthy).unwrap_or(false) { return Err(ApiError::coded(409, "请先暂停仿真，再从注入库放置扰动元素", "not_paused")); }
        let pl = b.get("placement").and_then(|p| p.as_str()).unwrap_or("ahead_2.5").to_string();
        let mut yaw = 0.0;
        let (x, y);
        let (rx, ry, tel_goal, tf, scene_stations, nodes) = {
            let g = self.gw.lock();
            let t = Value::Object(g.telemetry.clone());
            (f(&t, "x", 0.0), f(&t, "y", 0.0), t.get("target_goal").cloned().unwrap_or(Value::Null), t.get("taskflow").cloned().filter(|v| v.is_object()).unwrap_or(json!({})),
                t.pointer("/scenario_metadata/stations").and_then(|s| s.as_array()).cloned().unwrap_or_default(),
                t.pointer("/topo_graph/nodes").and_then(|n| n.as_object()).map(|o| o.values().cloned().collect::<Vec<_>>()).unwrap_or_default())
        };
        if pl == "ahead_2.5" || pl == "ahead_1.2" { let p = self.path_point(if pl == "ahead_2.5" { 2.5 } else { 1.2 }); x = p.0; y = p.1; yaw = p.2; }
        else if pl == "next_station" {
            let mut st: Option<Value> = None;
            let steps = tf.get("steps").and_then(|s| s.as_array()).cloned().unwrap_or_default();
            if let Some(i) = tf.get("step_index").and_then(|i| i.as_u64()) {
                for stp in steps.iter().skip(i as usize) {
                    if s(stp, "type") == "move" { st = scene_stations.iter().find(|q| q.get("id") == stp.get("target")).cloned(); if st.is_some() { break; } }
                }
            }
            let (tx, ty) = match (&st, tel_goal.is_object()) { (Some(q), _) => (f(q, "x", 0.0), f(q, "y", 0.0)), (None, true) => (f(&tel_goal, "x", 0.0), f(&tel_goal, "y", 0.0)), _ => { let p = self.path_point(3.0); (p.0, p.1) } };
            let d = { let d = (tx - rx).hypot(ty - ry); if d == 0.0 { 1.0 } else { d } };
            let off = (1.4f64).min(d * 0.5);
            x = tx - (tx - rx) / d * off; y = ty - (ty - ry) / d * off; yaw = (ty - ry).atan2(tx - rx);
        } else if pl == "random" {
            let far: Vec<&Value> = nodes.iter().filter(|n| (f(n, "x", 0.0) - rx).hypot(f(n, "y", 0.0) - ry) > 2.0).collect();
            let pool: Vec<&Value> = if far.is_empty() { nodes.iter().collect() } else { far };
            if pool.is_empty() { return Err(ApiError::new(400, "场景没有拓扑节点")); }
            let n = pool[(rand_u64() % pool.len() as u64) as usize];
            x = f(n, "x", 0.0) + uniform(-0.2, 0.2); y = f(n, "y", 0.0) + uniform(-0.2, 0.2);
        } else {
            let (bx, by) = (b.get("x").and_then(|v| v.as_f64()), b.get("y").and_then(|v| v.as_f64()));
            let (Some(bx), Some(by)) = (bx, by) else { return Err(ApiError::new(400, "自定义位置需要 x, y")) };
            x = bx; y = by; yaw = f(b, "yaw", 0.0);
        }
        // 叉车: 托盘/货箱是「可叉取货物」(sim cargo), 不是实体障碍
        let has_fork = { let g = self.gw.lock(); g.telemetry.get("fork").map(|v| v.is_object() && v.get("tine").is_some()).unwrap_or(false) };
        if has_fork && matches!(s(&cat, "type"), "pallet" | "box") {
            let is_pal = s(&cat, "type") == "pallet";
            let id = format!("{}{}", if is_pal { "P" } else { "B" }, rand_u64() % 10000);
            let mut body = json!({"kind": s(&cat, "type"), "id": id, "x": rnd(x, 3), "y": rnd(y, 3), "yaw": rnd(yaw, 3)});
            if !is_pal { body["size"] = json!([0.7, 0.5, 0.6]); }
            self.gw.cargo_add(&body).map_err(|e| ApiError::new(400, e))?;
            self.gw.events.emit("sensors", "CARGO_ADD", "info", &format!("放置{} {}", if is_pal { "托盘" } else { "货箱" }, id), &format!("({:.2}, {:.2}) 可叉取货物", x, y), json!({"id": id}));
            return Ok(json!({"type": s(&cat, "type"), "name": cat["name"], "x": rnd(x, 2), "y": rnd(y, 2), "yaw": rnd(yaw, 3), "id": 0, "cargo": id, "placement": pl}));
        }
        let mut el = json!({"type": cat["type"], "name": cat["name"], "x": rnd(x, 2), "y": rnd(y, 2), "yaw": rnd(yaw, 3), "w": cat["w"], "h": cat["h"], "z": cat["z"], "placement": pl});
        if s(&cat, "type") == "person" && b.get("walk").map(truthy).unwrap_or(false) {
            let span = f(b, "span", 1.5);
            let (px, py) = (-yaw.sin(), yaw.cos());
            let m = json!({"type": "patrol", "ax": rnd(x - px * span, 2), "ay": rnd(y - py * span, 2), "bx": rnd(x + px * span, 2), "by": rnd(y + py * span, 2), "speed": f(b, "speed", 0.6)});
            el["x"] = m["ax"].clone(); el["y"] = m["ay"].clone(); el["motion"] = m;
        }
        {
            let mut g = self.gw.lock();
            let mx = g.dyn_obs.iter().filter_map(|o| o.get("id").and_then(|i| i.as_i64())).max().unwrap_or(0).max(0);
            el["id"] = json!(mx + 1);
            el["active"] = json!(!g.is_paused);
            g.dyn_obs.push(el.clone());
            let o = Value::Array(g.dyn_obs.clone());
            g.telemetry.insert("dynamic_obstacles".into(), o.clone()); g.telemetry.insert("obstacles".into(), o);
        }
        self.gw.broadcast_obstacles();
        self.rec_injected(&el);
        let pname = if pl != "custom" { placements().as_array().unwrap().iter().find(|p| s(p, "id") == pl).map(|p| s(p, "name").to_string()).unwrap_or_default() } else { "自定义".into() };
        self.gw.events.emit("sensors", "INJECT_ADD", "warning", &format!("注入扰动元素 #{} {}", el["id"], s(&cat, "name")), &format!("{pname} ({:.2}, {:.2}){}", f(&el, "x", 0.0), f(&el, "y", 0.0), if el.get("motion").is_some() { " · 往返行走" } else { "" }), el.clone());
        Ok(el)
    }
    fn remove(&self, oid: i64) -> Result<Value, ApiError> {
        let n = {
            let mut g = self.gw.lock();
            let before = g.dyn_obs.len();
            g.dyn_obs.retain(|o| o.get("id").and_then(|i| i.as_i64()) != Some(oid));
            let o = Value::Array(g.dyn_obs.clone());
            g.telemetry.insert("dynamic_obstacles".into(), o.clone()); g.telemetry.insert("obstacles".into(), o);
            before - g.dyn_obs.len()
        };
        if n == 0 { return Err(ApiError::new(404, format!("没有注入元素 #{oid}"))); }
        self.gw.broadcast_obstacles();
        self.rec_removed(oid);
        self.gw.events.emit("sensors", "INJECT_REMOVE", "info", &format!("移除扰动元素 #{oid}"), "", json!({"id": oid}));
        Ok(json!({"removed": oid}))
    }
    fn reset(&self) -> Value {
        safe(&self.gw.nav, "POST", "/api/v1/taskflows/stop", Some(&json!({"reason": "环境重置"})));
        let n = self.gw.lock().dyn_obs.len();
        self.gw.reset_simulation();
        self.rec_end("RESET");
        self.gw.events.emit("system", "RESET_ENV", "warning", "仿真环境已重置", &format!("车辆回到起点 P0，清除 {n} 个注入元素，任务流已终止"), json!({}));
        json!({"reset": true})
    }
    fn run_flow(&self, flow: &Value) -> Result<Value, ApiError> {
        if !flow.get("steps").map(truthy).unwrap_or(false) { return Err(ApiError::new(400, "任务流没有工步")); }
        if self.gw.is_paused() { self.gw.set_simulation_pause(false); }
        let r = self.gw.nav.json("POST", "/api/v1/taskflows/run", Some(&json!({"flow": flow})), Some(8.0)).map_err(|e| {
            if e.code == "remote" { ApiError::new(400, e.message.split(": ").last().unwrap_or("").to_string()) } else { ApiError::new(502, format!("执行进程不可用: {}", e.message)) }
        })?;
        self.rec_start(flow);
        Ok(r)
    }
    fn stop_flow(&self) -> Value {
        let r = safe(&self.gw.nav, "POST", "/api/v1/taskflows/stop", Some(&json!({"reason": "操作员终止"}))).unwrap_or(json!({}));
        self.rec_end("STOPPED");
        r
    }

    fn zip(files: Vec<(String, Vec<u8>)>) -> Vec<u8> {
        let mut buf = std::io::Cursor::new(Vec::new());
        {
            let mut z = zip::ZipWriter::new(&mut buf);
            let o = zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);
            for (n, d) in files { if z.start_file(n, o).is_ok() { let _ = z.write_all(&d); } }
            let _ = z.finish();
        }
        buf.into_inner()
    }
    fn logs_zip(&self) -> Vec<u8> {
        let evs = self.gw.events.get_events(0, None, 5000)["events"].as_array().cloned().unwrap_or_default();
        let tagged: Vec<Value> = evs.into_iter().map(|mut e| { e["tag"] = json!(tag_of(&e)); e }).collect();
        let mut files = vec![("events.evt.json".to_string(), serde_json::to_vec_pretty(&tagged).unwrap_or_default())];
        for b in self.rec().records.iter().take(20) { files.push((format!("records/sim_bundle_{}.rcd.json", s(&b["metadata"], "recordId")), serde_json::to_vec(&self.full(b)).unwrap_or_default())); }
        files.push(("instance.json".into(), serde_json::to_vec_pretty(&self.info()).unwrap_or_default()));
        Self::zip(files)
    }

    fn build(&self) -> Server {
        let mut api = Server::with_base("gateway-v2", 0, "/api/v2", "设备端 Web 网关 v2", "仿真实例的工作台接口: 控制权锁、任务流、注入、记录、回放 (经平台 /inst/<实例>/api/v2 代理访问)");
        let me = self.me();
        macro_rules! R { ($m:expr, $p:expr, $d:expr, $f:expr) => {{ let v = me.clone(); api.route($m, $p, $d, move |q: &Request| -> HResult { ($f)(&v, q) }); }}; }
        let tok = |q: &Request| q.header("x-lock-token").unwrap_or("").to_string();
        R!("GET", "/api/v2/info", "实例信息", |v: &Arc<V2>, _q: &Request| ok(v.info()));
        R!("GET", "/api/v2/brief", "轻量状态", |v: &Arc<V2>, _q: &Request| ok(v.brief()));
        R!("GET", "/api/v2/lock", "控制权状态", move |v: &Arc<V2>, q: &Request| ok(v.locks.status(&tok(q))));
        R!("POST", "/api/v2/lock", "获取控制权 {user, force}", |v: &Arc<V2>, q: &Request| {
            let b = q.json()?;
            let r = v.locks.acquire(s(&b, "user"), b.get("force").map(truthy).unwrap_or(false))?;
            v.gw.events.emit("system", "LOCK", "info", &format!("{} 获取控制权", s(&r, "user")), "", json!({}));
            ok(r)
        });
        R!("PUT", "/api/v2/lock", "续约", move |v: &Arc<V2>, q: &Request| ok(v.locks.renew(&tok(q))?));
        R!("DELETE", "/api/v2/lock", "释放控制权", move |v: &Arc<V2>, q: &Request| {
            let t = tok(q);
            let st = v.locks.status(&t);
            let r = v.locks.release(&t);
            if st["mine"] == json!(true) { v.gw.events.emit("system", "LOCK", "info", &format!("{} 释放控制权", s(&st, "user")), "", json!({})); }
            ok(r)
        });
        R!("GET", "/api/v2/events", "带标签事件流", |v: &Arc<V2>, q: &Request| {
            let d = v.gw.events.get_events(q.q_i64("after", 0)? as u64, None, q.q_i64("limit", 200)? as usize);
            let mut evs: Vec<Value> = d["events"].as_array().cloned().unwrap_or_default().into_iter().map(|mut e| { e["tag"] = json!(tag_of(&e)); e }).collect();
            let tag = q.q_str("tag", "");
            if !tag.is_empty() { let tags: Vec<&str> = tag.split(',').collect(); evs.retain(|e| tags.contains(&s(e, "tag"))); }
            ok(json!({"events": evs, "latest_id": d["latest_id"]}))
        });
        R!("POST", "/api/v2/pause", "暂停/恢复", |v: &Arc<V2>, q: &Request| {
            let b = q.json()?;
            v.gw.set_simulation_pause(b.get("paused").map(truthy).unwrap_or(true));
            ok(json!({"paused": v.gw.is_paused()}))
        });
        R!("POST", "/api/v2/reset", "重置环境", |v: &Arc<V2>, _q: &Request| ok(v.reset()));
        R!("POST", "/api/v2/taskflow/run", "执行任务流", |v: &Arc<V2>, q: &Request| { let b = q.json()?; let fl = b.get("flow").filter(|f| truthy(f)).cloned().unwrap_or(b); ok(v.run_flow(&fl)?) });
        R!("POST", "/api/v2/taskflow/stop", "终止任务流", |v: &Arc<V2>, _q: &Request| ok(v.stop_flow()));
        R!("GET", "/api/v2/inject/catalog", "注入库", |_v: &Arc<V2>, _q: &Request| ok(json!({"catalog": catalog(), "placements": placements()})));
        R!("POST", "/api/v2/inject", "注入元素 {type, placement, x, y, w, h, z, motion, force} (类型见 GET /api/v2/inject/catalog)", |v: &Arc<V2>, q: &Request| ok(v.inject(&q.json()?)?));
        R!("DELETE", "/api/v2/inject/{oid}", "移除注入元素", |v: &Arc<V2>, q: &Request| ok(v.remove(q.params["oid"].parse::<i64>().map_err(|_| ApiError::new(400, "无效 id"))?)?));
        R!("GET", "/api/v2/records", "仿真记录", |v: &Arc<V2>, _q: &Request| { let recs: Vec<Value> = v.rec().records.iter().map(V2::summary).collect(); ok(json!({"records": recs, "stats": v.stats()})) });
        R!("GET", "/api/v2/records/{rid}/bundle", "下载 sim_bundle", |v: &Arc<V2>, q: &Request| {
            let rid = &q.params["rid"];
            let b = v.rec().records.iter().find(|b| s(&b["metadata"], "recordId") == rid).cloned().ok_or_else(|| ApiError::new(404, "记录不存在"))?;
            let b = v.full(&b);
            Ok(Response::raw(serde_json::to_vec(&b).unwrap_or_default(), "application/json; charset=utf-8").header("Content-Disposition", &format!("attachment; filename=sim_bundle_{}_{}.json", s(&b["metadata"], "taskId"), rid)))
        });
        R!("GET", "/api/v2/records/bundle_all", "打包下载全部记录", |v: &Arc<V2>, _q: &Request| {
            let recs: Vec<Value> = v.rec().records.clone();
            let files = recs.iter().map(|b| (format!("sim_bundle_{}_{}.json", s(&b["metadata"], "taskId"), s(&b["metadata"], "recordId")), serde_json::to_vec(&v.full(b)).unwrap_or_default())).collect();
            Ok(Response::raw(V2::zip(files), "application/zip").header("Content-Disposition", "attachment; filename=sim_bundles_all.zip"))
        });
        R!("POST", "/api/v2/records/archive", "归档全部记录到平台", |v: &Arc<V2>, _q: &Request| {
            if v.rec().active.is_some() { v.rec_end("STOPPED"); }
            let todo: Vec<Value> = v.rec().records.iter().filter(|b| !b["metadata"].get("archived").map(truthy).unwrap_or(false)).cloned().collect();
            ok(json!({"archived": todo.iter().filter(|b| v.archive(&v.full(b))).count()}))
        });
        R!("POST", "/api/v2/recording", "开关录制", |v: &Arc<V2>, q: &Request| {
            let on = q.json()?.get("on").map(truthy).unwrap_or(true);
            v.rec().on = on;
            if !on { v.rec_end("STOPPED"); }
            ok(v.stats())
        });
        R!("GET", "/api/v2/logs", "下载仿真日志 RCD/EVT", |v: &Arc<V2>, _q: &Request| Ok(Response::raw(v.logs_zip(), "application/zip").header("Content-Disposition", &format!("attachment; filename=sim_logs_{}.zip", fmt_local(now(), "%Y%m%d_%H%M%S")))));
        R!("GET", "/api/v2/safety", "避障参数", |v: &Arc<V2>, _q: &Request| ok(safe(&v.gw.nav, "GET", "/api/v1/safety/params", None).unwrap_or(json!({}))));
        R!("POST", "/api/v2/plan", "路线预览 (执行进程规划器) {points: [{x, y}], obstacles}", |v: &Arc<V2>, q: &Request| ok(safe(&v.gw.nav, "POST", "/api/v1/plan", Some(&q.json()?)).unwrap_or(json!({"legs": []}))));
        R!("PUT", "/api/v2/safety", "修改避障参数 {protection: {...}}", |v: &Arc<V2>, q: &Request| ok(v.gw.nav.put("/api/v1/safety/params", &q.json()?)?));
        R!("GET", "/api/v2/slam", "定位状态", |v: &Arc<V2>, _q: &Request| ok(safe(&v.gw.nav, "GET", "/api/v1/slam", None).unwrap_or(json!({"online": false}))));
        R!("GET", "/api/v2/slam/map", "SLAM 地图 ?part=grid|pgm|yaml", |v: &Arc<V2>, q: &Request| {
            let part = q.q_str("part", "grid");
            if part == "pgm" || part == "yaml" {
                let r = v.gw.nav.request("GET", &format!("/api/v1/slam/map?part={part}"), None, Some(10.0), "application/octet-stream").map_err(|_| ApiError::new(502, "地图为空或执行进程不可用"))?;
                if r.status != 200 { return Err(ApiError::new(r.status, "地图为空或执行进程不可用")); }
                let cd = r.headers.get("content-disposition").cloned().unwrap_or(format!("attachment; filename=\"slam.{part}\""));
                return Ok(Response::raw(r.body, if part == "pgm" { "image/x-portable-graymap" } else { "text/plain; charset=utf-8" }).header("Content-Disposition", &cd));
            }
            ok(safe_t(&v.gw.nav, "GET", &format!("/api/v1/slam/map?step={}", q.q_i64("step", 1)?), None, 5.0).unwrap_or(json!({"empty": true})))
        });
        for (act, doc) in [("mode", "切换定位模式 {mode: slam|localization|odom|ground_truth}"), ("save", "保存当前场景的 SLAM 地图"), ("reset", "重置定位/建图"), ("initialpose", "设置初始位姿 {x, y, yaw}")] {
            let v = me.clone();
            api.route("POST", &format!("/api/v2/slam/{act}"), doc, move |q: &Request| ok(v.gw.nav.json("POST", &format!("/api/v1/slam/{act}"), Some(&q.json()?), Some(10.0))?));
        }
        R!("PUT", "/api/v2/planner", "切换规划器", |v: &Arc<V2>, q: &Request| { v.gw.set_planner_type(s(&q.json()?, "type")); ok(json!({"planner": v.gw.lock().active_planner})) });
        R!("PUT", "/api/v2/chassis", "切换车型 (热切换)", |v: &Arc<V2>, q: &Request| { v.gw.set_chassis_type(s(&q.json()?, "type")); ok(json!({"ok": true})) });
        R!("POST", "/api/v2/scene", "热切换场景 {hub_scene_id}", |v: &Arc<V2>, q: &Request| {
            let b = q.json()?;
            safe(&v.gw.nav, "POST", "/api/v1/taskflows/stop", Some(&json!({"reason": "场景切换"})));
            v.gw.cancel_nav();
            v.gw.lock().dyn_obs.clear();
            safe(&v.gw.sim, "PUT", "/api/v1/world/obstacles", Some(&json!([])));
            let r = v.gw.sim.json("POST", "/api/v1/scene/load", Some(&b), Some(60.0))?;
            v.gw.lock().world = json!({});
            v.reset_info();
            v.gw.events.emit("system", "SCENARIO_SWITCH", "info", "仿真场景热切换", &format!("→ {}", r.get("scenario").map(|s| s.as_str().map(|x| x.to_string()).unwrap_or(s.to_string())).unwrap_or("None".into())), r.clone());
            ok(r)
        });
        R!("POST", "/api/v2/model", "热切换车辆模型 {model_id, version}", |v: &Arc<V2>, q: &Request| {
            let b = q.json()?;
            let r = v.gw.sim.json("POST", "/api/v1/model/load", Some(&b), Some(60.0))?;
            v.gw.lock().model = json!({});
            v.reset_info();
            v.gw.events.emit("chassis", "CHASSIS_SWITCH", "info", "车辆模型热切换", &format!("→ {}", b.get("model_id").map(|s| s.as_str().map(|x| x.to_string()).unwrap_or(s.to_string())).unwrap_or("None".into())), json!({}));
            ok(json!({"ok": true, "model_rev": r.get("model_rev").cloned().unwrap_or(Value::Null)}))
        });
        api
    }

    /// full request handling incl. lock gate; Err(response) never — always a Response
    pub fn handle(&self, req: &Request, method: &str) -> Response {
        let path = req.path.as_str();
        if method != "GET" && !path.starts_with("/api/v2/lock") && path != "/api/v2/plan" {
            if let Some(who) = self.locks.allowed(req.header("x-lock-token").unwrap_or("")) { return problem(423, "locked", &format!("控制权由 {who} 持有，当前为只读"), path); }
        }
        let api = self.api.lock().unwrap().clone().unwrap();
        let mut r = req.clone();
        match api.dispatch(&mut r, method) {
            Ok(resp) => resp,
            Err(e) => problem(e.status, &e.code, &e.message, path),
        }
    }
    pub fn guard_legacy(&self, req: &Request) -> Option<Response> {
        let who = self.locks.allowed(req.header("x-lock-token").unwrap_or(""))?;
        let b = json!({"error": {"code": "locked", "message": format!("控制权由 {who} 持有，当前为只读")}, "status": "error", "message": format!("控制权由 {who} 持有")});
        Some(Response::json(&b).with_status(423))
    }
}

pub static GLOBAL: std::sync::OnceLock<Arc<V2>> = std::sync::OnceLock::new();
