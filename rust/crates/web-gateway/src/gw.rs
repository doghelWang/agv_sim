//! Gateway core: aggregates sim + nav REST into the legacy front-end telemetry and relays commands
use crate::events::EventHub;
use crate::perf::PerfMonitor;
use crate::recorder::FlightRecorder;
use crate::util::*;
use rest::Client;
use serde_json::{json, Map, Value};
use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::Duration;

pub const SCENARIOS: &str = include_str!("../../../data/scenarios.json");
const BUSY: [&str; 4] = ["NAVIGATING", "PLANNING", "OBSTACLE_WAIT", "DOCKING"];

pub struct St {
    pub telemetry: Map<String, Value>,
    pub model: Value,
    pub world: Value,
    pub nav_status: Value,
    pub is_paused: bool,
    pub dyn_obs: Vec<Value>,
    pub active_scenario: String,
    pub active_chassis: String,
    pub active_planner: String,
    pub lidar_config: Value,
    sim_ev: u64,
    nav_ev: u64,
    last_mission: Option<Value>,
    last_status: Option<String>,
    pause_override: Option<(bool, f64)>,
    model_dirty: bool,
    known: HashSet<String>,
}

pub struct Gateway {
    pub sim: Client,
    pub nav: Client,
    pub sim_api: String,
    pub nav_api: String,
    pub st: Mutex<St>,
    pub events: Arc<EventHub>,
    pub rec: Arc<FlightRecorder>,
    pub perf: OnceLock<Arc<PerfMonitor>>,
    pub sim_online: AtomicBool,
    pub nav_online: AtomicBool,
    last_client_ms: AtomicU64,
    want_scans_until: Mutex<f64>,
}

pub fn safe(c: &Client, m: &str, p: &str, b: Option<&Value>) -> Option<Value> { c.json(m, p, b, None).ok() }
pub fn safe_t(c: &Client, m: &str, p: &str, b: Option<&Value>, t: f64) -> Option<Value> { c.json(m, p, b, Some(t)).ok() }

fn level(l: &str) -> &str { match l { "danger" | "warning" | "success" | "info" => l, _ => "info" } }

impl Gateway {
    pub fn new(sim_api: &str, nav_api: &str, events: Arc<EventHub>, rec: Arc<FlightRecorder>) -> Arc<Gateway> {
        let known: HashSet<String> = serde_json::from_str::<Value>(SCENARIOS).ok().and_then(|v| v.as_object().map(|o| o.keys().cloned().collect())).unwrap_or_default();
        let lc = json!({"beams": 720, "angle_resolution_deg": 0.5, "freq_hz": 10.0, "range_max": 30.0});
        let t = now();
        let mut tel = Map::new();
        for (k, v) in [("x", json!(0.0)), ("y", json!(0.0)), ("yaw", json!(0.0)), ("vx", json!(0.0)), ("vy", json!(0.0)), ("wz", json!(0.0)), ("nav_status", json!("IDLE")), ("plan_path", json!([])), ("path_labels", json!([])),
            ("path_index", json!(0)), ("next_segment", Value::Null), ("target_goal", Value::Null), ("nav_dist_rem", json!(0.0)), ("is_paused", json!(false)), ("dynamic_obstacles", json!([])), ("obstacles", json!([])),
            ("scan_ranges", json!([])), ("io_states", json!({})), ("chassis_type", json!("single_steer")), ("planner_type", json!("dijkstra")), ("active_scenario", json!("grid_9_square")), ("map_scenario", json!("grid_9_square")),
            ("lidar_config", lc.clone()), ("timestamp", json!(t)), ("last_update", json!(t))] { tel.insert(k.into(), v); }
        Arc::new(Gateway {
            sim: Client::new(sim_api, 2.0), nav: Client::new(nav_api, 2.0), sim_api: sim_api.into(), nav_api: nav_api.into(),
            st: Mutex::new(St { telemetry: tel, model: json!({}), world: json!({}), nav_status: json!({}), is_paused: false, dyn_obs: vec![], active_scenario: "grid_9_square".into(), active_chassis: "single_steer".into(),
                active_planner: "dijkstra".into(), lidar_config: lc, sim_ev: 0, nav_ev: 0, last_mission: None, last_status: None, pause_override: None, model_dirty: false, known }),
            events, rec, perf: OnceLock::new(), sim_online: AtomicBool::new(false), nav_online: AtomicBool::new(false), last_client_ms: AtomicU64::new((t * 1000.0) as u64), want_scans_until: Mutex::new(0.0),
        })
    }

    pub fn lock(&self) -> MutexGuard<'_, St> { self.st.lock().unwrap_or_else(|e| e.into_inner()) }
    pub fn touch(&self) { self.last_client_ms.store((now() * 1000.0) as u64, Ordering::Relaxed); }
    pub fn since_client(&self) -> f64 { now() - self.last_client_ms.load(Ordering::Relaxed) as f64 / 1000.0 }
    pub fn want_scans(&self) { *self.want_scans_until.lock().unwrap_or_else(|e| e.into_inner()) = now() + 2.0; }
    pub fn sim_online(&self) -> bool { self.sim_online.load(Ordering::Relaxed) }
    pub fn nav_online(&self) -> bool { self.nav_online.load(Ordering::Relaxed) }
    pub fn is_paused(&self) -> bool { self.lock().is_paused }

    fn is_busy(st: &St) -> bool { st.telemetry.get("nav_status").and_then(|s| s.as_str()).map(|s| BUSY.contains(&s)).unwrap_or(false) }
    pub fn is_idle(&self) -> bool {
        let after = env_f("GW_IDLE_AFTER_S", 15.0);
        if after <= 0.0 || self.since_client() < after { return false; }
        !Self::is_busy(&self.lock())
    }

    pub fn start(self: &Arc<Self>) {
        for (name, f) in [("gw-sim", Self::sim_loop as fn(&Arc<Self>)), ("gw-nav", Self::nav_loop), ("gw-slow", Self::slow_loop), ("gw-rec", Self::recorder_loop)] {
            let me = self.clone();
            std::thread::Builder::new().name(name.into()).spawn(move || f(&me)).ok();
        }
    }

    fn sim_loop(self: &Arc<Self>) {
        loop {
            let t0 = now();
            let scans = now() < *self.want_scans_until.lock().unwrap_or_else(|e| e.into_inner());
            let r = safe(&self.sim, "GET", &format!("/api/v1/snapshot{}", if scans { "?scans=1" } else { "" }), None).and_then(|s| self.apply_snapshot(&s));
            if r.is_some() { self.sim_online.store(true, Ordering::Relaxed); } else { self.sim_online.store(false, Ordering::Relaxed); std::thread::sleep(Duration::from_millis(500)); }
            let busy = Self::is_busy(&self.lock());
            let idle = env_f("GW_IDLE_POLL_S", 1.0);
            let fast = env_f("GW_BUSY_POLL_S", if is_android() { 0.1 } else { 0.05 });
            let period = if busy || self.since_client() < 3.0 { fast } else { idle };
            std::thread::sleep(Duration::from_secs_f64((period - (now() - t0)).max(0.0)));
        }
    }

    fn nav_loop(self: &Arc<Self>) {
        loop {
            let r: Option<()> = (|| {
                let st = safe(&self.nav, "GET", "/api/v1/nav", None)?;
                self.nav_online.store(true, Ordering::Relaxed);
                {
                    let mut g = self.lock();
                    if let Some(p) = st.get("planner").and_then(|p| p.as_str()) { g.active_planner = p.into(); }
                    let p = g.active_planner.clone();
                    g.telemetry.insert("planner_type".into(), json!(p));
                    if let Some(ros) = st.get("ros").filter(|r| truthy(r)) {
                        let topics = ros.get("topics").and_then(|t| t.as_array()).cloned().unwrap_or_default();
                        g.telemetry.insert("ros_graph".into(), json!({"nodes": ros.get("nodes").cloned().unwrap_or(json!([])), "topic_count": topics.len(), "topics": topics.iter().take(80).collect::<Vec<_>>()}));
                    }
                    g.nav_status = st;
                }
                let since = self.lock().nav_ev;
                let evs = safe(&self.nav, "GET", &format!("/api/v1/events?since={since}"), None)?;
                for e in evs.get("events").and_then(|e| e.as_array()).cloned().unwrap_or_default() {
                    { let mut g = self.lock(); g.nav_ev = g.nav_ev.max(e["id"].as_u64().unwrap_or(0)); }
                    let g = |k: &str, d: &str| e.get(k).and_then(|v| v.as_str()).unwrap_or(d).to_string();
                    self.events.emit(&g("category", "navigation"), &g("type", "NAV"), &g("level", "info"), &format!("[执行进程] {}", g("title", "")), &g("message", ""), e.get("payload").cloned().unwrap_or(json!({})));
                }
                Some(())
            })();
            if r.is_none() { self.nav_online.store(false, Ordering::Relaxed); std::thread::sleep(Duration::from_secs(1)); }
            std::thread::sleep(Duration::from_secs_f64(if self.is_idle() { 1.0 } else { 0.2 }));
        }
    }

    fn slow_loop(self: &Arc<Self>) {
        let mut cnt = 0u64;
        loop {
            let _: Option<()> = (|| {
                let (need, dirty) = { let g = self.lock(); (!truthy(&g.model), g.model_dirty) };
                if need || cnt % 10 == 0 || dirty {
                    self.lock().model_dirty = false;
                    let m = safe(&self.sim, "GET", "/api/v1/model", None)?;
                    let changed = { let g = self.lock(); ["active_chassis", "model_file", "model_rev"].iter().any(|k| m.get(*k) != g.model.get(*k)) };
                    if changed { self.apply_model(m); }
                }
                let w = safe(&self.sim, "GET", "/api/v1/world", None)?;
                let newworld = { let g = self.lock(); w.get("id") != g.world.get("id") || !truthy(&g.world) };
                if newworld { self.apply_world(&w); }
                {
                    let mut g = self.lock();
                    if !g.is_paused {
                        g.dyn_obs = w.get("obstacles").and_then(|o| o.as_array()).map(|a| a.iter().map(|o| { let mut o = o.clone(); o["active"] = json!(true); o }).collect()).unwrap_or_default();
                    }
                }
                let since = self.lock().sim_ev;
                let evs = safe(&self.sim, "GET", &format!("/api/v1/events?since={since}"), None)?;
                for e in evs.get("events").and_then(|e| e.as_array()).cloned().unwrap_or_default() {
                    { let mut g = self.lock(); g.sim_ev = g.sim_ev.max(e["id"].as_u64().unwrap_or(0)); }
                    let ty = s(&e, "type");
                    let cat = match ty { "COLLISION" | "ESTOP" => "safety", _ => "system" };
                    let msg = s(&e, "message");
                    self.events.emit(cat, ty, level(s(&e, "level")), &format!("[仿真进程] {msg}"), msg, e.get("data").cloned().unwrap_or(json!({})));
                }
                Some(())
            })();
            cnt += 1;
            std::thread::sleep(Duration::from_secs_f64(if self.is_idle() { 2.0 } else { 0.5 }));
        }
    }

    fn recorder_loop(self: &Arc<Self>) {
        loop {
            std::thread::sleep(Duration::from_millis(100));
            let t = {
                let g = self.lock();
                let T = &g.telemetry;
                let gv = |k: &str, d: Value| T.get(k).cloned().unwrap_or(d);
                json!({"x": gv("x", json!(0.0)), "y": gv("y", json!(0.0)), "yaw": gv("yaw", json!(0.0)), "vx": gv("vx", json!(0.0)), "vy": gv("vy", json!(0.0)), "wz": gv("wz", json!(0.0)), "nav_status": gv("nav_status", json!("IDLE")),
                    "nav_dist_rem": gv("nav_dist_rem", json!(0.0)), "io_states": gv("io_states", json!({})), "scan_ranges": gv("scan_ranges", json!([])), "scan_min_dist": gv("scan_min_dist", json!(12.0)),
                    "vision_markers": gv("vision_markers", json!([])), "dynamic_obstacles": gv("dynamic_obstacles", json!([])), "obstacles": gv("dynamic_obstacles", json!([]))})
            };
            let perf = self.perf.get().map(|p| {
                let l = p.latest();
                let (h, r) = (&l["host"], &l["ros_simulation_total"]);
                json!({"host_cpu": f(h, "cpu_total_percent", 0.0), "ros_cpu": f(r, "combined_cpu_percent", 0.0), "ram_mb": f(h, "memory_used_mb", 0.0), "temp_c": h.get("cpu_temp_c").cloned().unwrap_or(json!(0.0))})
            });
            self.rec.record_frame(&t, perf);
        }
    }

    pub fn apply_model(&self, spec: Value) {
        let mut g = self.lock();
        let ch = spec.get("chassis").cloned().unwrap_or(json!({}));
        let pick = |v: &Value, keys: &[&str]| -> Value { let mut m = Map::new(); for k in keys { m.insert((*k).into(), v.get(*k).cloned().unwrap_or(Value::Null)); } Value::Object(m) };
        let lidars: Vec<Value> = spec.get("lidars").and_then(|l| l.as_array()).cloned().unwrap_or_default();
        let sg = |k: &str, d: Value| spec.get(k).cloned().unwrap_or(d);
        let wheels = spec.get("active_wheels").cloned().or_else(|| spec.get("wheels").cloned()).unwrap_or(json!([]));
        let rs = json!({"model_file": sg("model_file", Value::Null), "native_chassis": ch.get("type").cloned().unwrap_or(Value::Null), "active_chassis": sg("active_chassis", Value::Null), "model_rev": sg("model_rev", Value::Null),
            "photoelectric": sg("photoelectric", json!([])), "lift": sg("lift", Value::Null), "footprint": ch.get("footprint").cloned().unwrap_or(Value::Null), "wheels": wheels,
            "lidars": lidars.iter().map(|l| pick(l, &["name", "model", "x", "y", "z", "yaw", "inverted", "fov_deg", "max_range"])).collect::<Vec<_>>(),
            "cameras": sg("cameras", json!([])), "warnings": sg("warnings", json!([])), "assumptions": sg("assumptions", json!([])),
            "chassis": pick(&ch, &["type", "cmodel_type", "cmodel_name", "length_m", "width_m", "height_m", "head_offset_m", "tail_offset_m", "left_offset_m", "right_offset_m", "max_speed_mps", "max_accel_mps2", "max_decel_mps2", "max_ang_speed_radps", "mass_kg"]),
            "imu": sg("imu", Value::Null), "battery": sg("battery", Value::Null), "io": sg("io", Value::Null), "other_actuators": sg("other_actuators", json!([])),
            "lidars_full": lidars.iter().map(|l| pick(l, &["name", "model", "vendor_model", "type", "x", "y", "z", "roll", "pitch", "yaw", "inverted", "fov_deg", "min_range", "max_range", "resolution_deg", "freq_hz", "spec_source", "vfov_min_deg", "vfov_max_deg", "point_rate", "topic", "lines", "scan_pattern"])).collect::<Vec<_>>()});
        g.telemetry.insert("config".into(), spec.clone());
        g.telemetry.insert("robot_spec".into(), rs);
        g.model = spec;
    }

    pub fn apply_world(&self, w: &Value) {
        let mut g = self.lock();
        if let Some(sd) = w.get("scenario_def").filter(|v| truthy(v)) { if let Some(id) = sd.get("id").and_then(|i| i.as_str()) { g.known.insert(id.into()); } }
        g.world = w.clone();
        if let Some(id) = w.get("id").and_then(|i| i.as_str()) { g.active_scenario = id.into(); }
        let meta: Map<String, Value> = w.as_object().map(|o| o.iter().filter(|(k, _)| !["topology", "obstacles", "heights"].contains(&k.as_str())).map(|(k, v)| (k.clone(), v.clone())).collect()).unwrap_or_default();
        let a = g.active_scenario.clone();
        g.telemetry.insert("scenario_metadata".into(), Value::Object(meta));
        g.telemetry.insert("topo_graph".into(), w.get("topology").cloned().unwrap_or(Value::Null));
        g.telemetry.insert("active_scenario".into(), json!(a));
        g.telemetry.insert("map_scenario".into(), json!(a));
    }

    fn apply_snapshot(&self, snap: &Value) -> Option<()> {
        let st = snap.get("state")?;
        let (sim, nav) = (snap.get("sim").cloned().unwrap_or(json!({})), snap.get("nav").cloned().unwrap_or(json!({})));
        let (tr, od) = (st.get("truth")?, st.get("odom")?);
        let joints = st.get("joints")?;
        let mission;
        {
            let mut g = self.lock();
            if truthy(&g.model) && st.get("model_rev") != g.model.get("model_rev") { g.model_dirty = true; }
            let dyn_obs = Value::Array(g.dyn_obs.clone());
            let mut tel = std::mem::take(&mut g.telemetry);
            let rnd_arr = |v: Option<&Value>, n: i32| Value::Array(v.and_then(|a| a.as_array()).map(|a| a.iter().map(|p| jr(p.as_f64().unwrap_or(0.0), n)).collect()).unwrap_or_default());
            let upd: Vec<(&str, Value)> = vec![("x", tr["x"].clone()), ("y", tr["y"].clone()), ("yaw", tr["yaw"].clone()), ("vx", tr["vx"].clone()), ("vy", tr["vy"].clone()), ("wz", tr["wz"].clone()),
                ("odom_pose", json!({"x": rnd(f(od, "x", 0.0), 3), "y": rnd(f(od, "y", 0.0), 3)})), ("chassis_type", st.get("chassis").cloned().unwrap_or(Value::Null)),
                ("is_paused", json!(st.get("paused").and_then(|p| p.as_bool()).unwrap_or(false))), ("io_states", snap.get("io").cloned().unwrap_or(json!({}))), ("vision_markers", snap.get("codes").cloned().unwrap_or(json!([]))),
                ("joint_states", json!({"names": joints.get("names").cloned().unwrap_or(json!([])), "positions": rnd_arr(joints.get("position"), 4), "velocities": rnd_arr(joints.get("velocity"), 4)})),
                ("sim_status", sim.clone()), ("actuators", sim.get("kinematics").cloned().unwrap_or(Value::Null)), ("timestamp", json!(now())), ("last_update", json!(now())),
                ("dynamic_obstacles", dyn_obs.clone()), ("obstacles", dyn_obs), ("photoelectric", snap.get("photoelectric").cloned().unwrap_or(json!([]))), ("cameras", snap.get("cameras").cloned().unwrap_or(json!([]))),
                ("bumpers", snap.get("bumpers").cloned().unwrap_or(json!({}))), ("cargo", snap.get("cargo").cloned().unwrap_or(json!([]))), ("fork", snap.get("fork").cloned().unwrap_or(Value::Null))];
            for (k, v) in upd { tel.insert(k.into(), v); }
            let mut sim_paused = st.get("paused").and_then(|p| p.as_bool()).unwrap_or(false);
            match g.pause_override {
                Some((v, until)) if now() < until && sim_paused != v => sim_paused = v,
                _ => g.pause_override = None,
            }
            g.is_paused = sim_paused;
            tel.insert("is_paused".into(), json!(sim_paused));
            if let Some(c) = st.get("chassis").and_then(|c| c.as_str()) { g.active_chassis = c.into(); }
            if let Some(ms) = snap.get("merged_scan").filter(|m| truthy(m)) {
                let rmax = f(ms, "range_max", 0.0);
                let rs: Vec<f64> = ms.get("ranges").and_then(|r| r.as_array()).map(|a| a.iter().map(|r| r.as_f64().unwrap_or(rmax)).collect()).unwrap_or_default();
                let step = (rs.len() / 720).max(1);
                let inc = f(ms, "angle_increment", 0.0);
                tel.insert("scan_ranges".into(), Value::Array(rs.iter().step_by(step).map(|r| jr(*r, 2)).collect()));
                tel.insert("scan_angle_min".into(), ms["angle_min"].clone());
                tel.insert("scan_angle_inc".into(), json!(inc * step as f64));
                tel.insert("scan_angle_max".into(), json!(f(ms, "angle_min", 0.0) + inc * (rs.len().saturating_sub(1)) as f64));
                let fin = rs.iter().cloned().filter(|r| *r < rmax).fold(f64::INFINITY, f64::min);
                tel.insert("scan_min_dist".into(), jr(if fin.is_finite() { fin } else { rmax }, 2));
                tel.insert("scan_range_max".into(), json!(rmax));
                tel.insert("scan_pose".into(), ms.get("pose").cloned().unwrap_or(Value::Null));
                let hz = ms.get("scan_hz").and_then(|h| h.as_f64()).unwrap_or(10.0);
                let lc = json!({"beams": rs.len(), "angle_resolution_deg": rnd(inc.to_degrees(), 3), "freq_hz": hz, "total_rays_per_sec": (rs.len() as f64 * hz) as i64, "range_max": rmax});
                g.lidar_config = lc.clone();
                tel.insert("lidar_config".into(), lc);
            }
            if let Some(ls) = snap.get("lidar_scans") { tel.insert("lidar_scans".into(), ls.clone()); }
            tel.insert("safety".into(), nav.get("safety").cloned().unwrap_or(Value::Null));
            tel.insert("taskflow".into(), nav.get("taskflow").cloned().unwrap_or(Value::Null));
            tel.insert("localization".into(), nav.get("localization").cloned().unwrap_or(Value::Null));
            if nav.get("online").map(truthy).unwrap_or(false) {
                let ng = |k: &str, d: Value| nav.get(k).filter(|v| truthy(v)).cloned().unwrap_or(d);
                tel.insert("nav_status".into(), nav.get("status").cloned().unwrap_or(json!("IDLE")));
                tel.insert("plan_path".into(), ng("path", json!([])));
                tel.insert("path_labels".into(), ng("path_labels", json!([])));
                tel.insert("plan_curve".into(), ng("curve", json!([])));
                tel.insert("protection".into(), nav.get("protection").cloned().unwrap_or(Value::Null));
                tel.insert("path_index".into(), nav.get("path_index").cloned().unwrap_or(json!(0)));
                tel.insert("next_segment".into(), nav.get("next_segment").cloned().unwrap_or(Value::Null));
                tel.insert("target_goal".into(), nav.get("goal").cloned().unwrap_or(Value::Null));
                tel.insert("nav_dist_rem".into(), nav.get("dist_remaining").cloned().unwrap_or(json!(0.0)));
                tel.insert("nav2".into(), nav.get("nav2").cloned().unwrap_or(Value::Null));
                tel.insert("nav2_feedback".into(), nav.get("nav2_feedback").cloned().unwrap_or(Value::Null));
                let pl = nav.get("planner").cloned().or_else(|| tel.get("planner_type").cloned()).unwrap_or(Value::Null);
                tel.insert("planner_type".into(), pl);
            } else {
                tel.insert("nav2".into(), json!({"msgs": false, "server_ready": false, "process": false}));
            }
            let n_obs = g.dyn_obs.len();
            tel.insert("bullet_metrics".into(), Self::metrics(&sim, n_obs));
            tel.insert("arch".into(), json!({"sim_api": self.sim_api, "nav_api": self.nav_api, "sim_online": self.sim_online(), "nav_online": self.nav_online(), "nav_feedback_age_s": nav.get("age_s").cloned().unwrap_or(Value::Null),
                "control": snap.get("control").cloned().unwrap_or(Value::Null), "link": g.nav_status.get("link").cloned().unwrap_or(Value::Null)}));
            g.telemetry = tel;
            mission = nav.get("mission").filter(|m| truthy(m)).cloned().unwrap_or(json!({}));
        }
        self.track_mission(&mission);
        Some(())
    }

    fn metrics(sm: &Value, n_obs: usize) -> Value {
        if !truthy(sm) { return json!({}); }
        let lid: Vec<Value> = sm.get("lidars").and_then(|l| l.as_array()).cloned().unwrap_or_default();
        let rays: f64 = lid.iter().map(|l| f(l, "beams", 0.0) * f(l, "freq_hz", 10.0)).sum();
        let dt = { let d = f(sm, "dt", 0.01); if d == 0.0 { 0.01 } else { d } };
        let step = sm.get("step_ms").and_then(|v| v.as_f64()).unwrap_or(0.0);
        let lms = sm.get("lidar_ms").and_then(|v| v.as_f64()).unwrap_or(0.0);
        json!({"engine": format!("SimCore/{} (独立仿真进程)", sm.get("backend").map(|b| b.as_str().map(|s| s.to_string()).unwrap_or(b.to_string())).unwrap_or("None".into())),
            "mode": format!("固定步长 {:.0} ms · RTF {}", dt * 1000.0, sm.get("rtf").map(|r| r.to_string()).unwrap_or("None".into())),
            "parameters": {"fixed_timestep_s": dt, "physics_rate_hz": rnd(1.0 / dt, 1), "num_solver_iterations": 0, "gravity": [0, 0, -9.81], "wheel_friction": 1.2, "ground_friction": 1.0},
            "data_volume": {"total_rigid_bodies": 0, "obstacle_bodies": n_obs, "simulated_joints": 0, "active_contact_points": sm.get("collisions").cloned().unwrap_or(json!(0)),
                "lidar_beams_per_scan": lid.iter().map(|l| f(l, "beams", 0.0)).sum::<f64>() as i64, "lidar_scan_freq_hz": lid.iter().map(|l| f(l, "freq_hz", 0.0)).fold(0.0, f64::max),
                "lidar_rays_per_second": rays as i64, "physics_steps_total": (f(sm, "sim_time", 0.0) / dt) as i64},
            "performance": {"physics_step_ms": sm.get("step_ms").cloned().unwrap_or(Value::Null), "raycast_step_ms": sm.get("lidar_ms").cloned().unwrap_or(Value::Null), "physics_capacity_hz": (1000.0 / step.max(0.01)).round() as i64,
                "raycast_throughput_rays_per_ms": 0, "step_cpu_time_pct": rnd(step / (dt * 1000.0) * 100.0, 2), "raycast_cpu_time_pct": rnd(lms * 10.0 / 1000.0 * 100.0, 2),
                "combined_core_load_pct": 0, "rtf": sm.get("rtf").cloned().unwrap_or(Value::Null), "overruns": sm.get("overruns").cloned().unwrap_or(Value::Null)},
            "sim_status": sm})
    }

    fn track_mission(&self, m: &Value) {
        let mid = m.get("id").filter(|v| !v.is_null()).cloned();
        let status = m.get("status").and_then(|s| s.as_str()).map(|s| s.to_string());
        let (start, supersede, end);
        {
            let mut g = self.lock();
            start = match &mid { Some(id) if g.last_mission.as_ref() != Some(id) => true, _ => false };
            supersede = start && g.last_mission.is_some();
            if start { g.last_mission = mid.clone(); }
            end = mid.is_some() && status != g.last_status && matches!(status.as_deref(), Some("ARRIVED" | "FAILED" | "CANCELED" | "NO_PATH" | "REJECTED"));
            g.last_status = status.clone();
        }
        if supersede { self.rec.end_session("SUPERSEDED"); }
        if start {
            let g = self.lock();
            let (sc, ch) = (g.active_scenario.clone(), g.active_chassis.clone());
            drop(g);
            let title = m.get("title").filter(|t| truthy(t)).cloned().unwrap_or(json!(format!("任务 #{}", mid.as_ref().map(|v| v.as_str().map(|s| s.to_string()).unwrap_or(v.to_string())).unwrap_or_default())));
            let wps: Vec<Value> = m.get("path").and_then(|p| p.as_array()).map(|a| a.iter().map(|p| json!([p["x"], p["y"]])).collect()).unwrap_or_default();
            self.rec.start_session("mission", json!({"mission_id": mid, "title": title, "scenario": sc, "chassis": ch, "planner": m.get("planner").cloned().unwrap_or(Value::Null),
                "destination": m.get("goal").cloned().unwrap_or(Value::Null), "planned_route": {"planner": m.get("planner").cloned().unwrap_or(Value::Null), "waypoints": wps}}));
        }
        if end { self.rec.end_session(status.as_deref().unwrap_or("")); }
    }

    // ------------------------------------------------------------------ telemetry JSON
    pub fn telemetry_json(&self, full: bool, scans: bool) -> Vec<u8> {
        if scans { self.want_scans(); }
        let mut g = self.lock();
        if let Some(p) = self.perf.get() {
            let l = p.latest();
            let (h, r) = (&l["host"], &l["ros_simulation_total"]);
            g.telemetry.insert("perf_summary".into(), json!({"cpu_tot": f(h, "cpu_total_percent", 0.0), "mem_pct": f(h, "memory_percent", 0.0), "temp_c": h.get("cpu_temp_c").cloned().unwrap_or(json!(0.0)),
                "ros_cpu": f(r, "combined_cpu_percent", 0.0), "ros_mb": f(r, "combined_rss_mb", 0.0)}));
        }
        let m: Map<String, Value> = g.telemetry.iter().filter(|(k, _)| (scans || *k != "lidar_scans") && (full || !["scenario_metadata", "topo_graph", "config"].contains(&k.as_str()))).map(|(k, v)| (k.clone(), v.clone())).collect();
        drop(g);
        serde_json::to_vec(&Value::Object(m)).unwrap_or_default()
    }

    // ------------------------------------------------------------------ commands
    pub fn publish_cmd_vel(&self, vx: f64, vy: f64, wz: f64) {
        self.rec.set_cmd_vel(vx, vy, wz);
        if safe(&self.nav, "POST", "/api/v1/teleop", Some(&json!({"vx": vx, "vy": vy, "wz": wz}))).is_none() {
            safe(&self.sim, "PUT", "/api/v1/control/cmd_vel", Some(&json!({"vx": vx, "vy": vy, "wz": wz, "source": "web-teleop"})));
        }
    }
    pub fn send_nav_goal(&self, x: f64, y: f64, yaw: f64) {
        match self.nav.json("POST", "/api/v1/missions", Some(&json!({"x": x, "y": y, "yaw": yaw})), None) {
            Ok(m) => { let st = m.get("status").cloned().unwrap_or(json!("PLANNING")); self.lock().telemetry.insert("nav_status".into(), st); }
            Err(e) => { self.events.emit("navigation", "MISSION_FAILED", "danger", "任务下发失败", &format!("执行进程不可用: {e}"), json!({})); }
        }
    }
    pub fn cancel_nav(&self) { safe(&self.nav, "DELETE", "/api/v1/missions/current", None); }
    pub fn set_planner_type(&self, t: &str) {
        if let Some(r) = safe(&self.nav, "PUT", "/api/v1/nav/planner", Some(&json!({"type": t}))) {
            if let Some(p) = r.get("planner").and_then(|p| p.as_str()) { let mut g = self.lock(); g.active_planner = p.into(); g.telemetry.insert("planner_type".into(), json!(p)); }
        }
    }
    pub fn set_chassis_type(&self, t: &str) {
        if safe(&self.sim, "PUT", "/api/v1/model/chassis", Some(&json!({"type": t}))).is_some() {
            self.lock().model = json!({});
            self.events.emit("chassis", "CHASSIS_SWITCH", "info", "底盘运动学模型切换", &format!("仿真进程车型 → {t}"), json!({"chassis_type": t}));
        }
    }
    pub fn set_map_scenario(&self, sid: &str) {
        if !self.lock().known.contains(sid) { return; }
        self.cancel_nav();
        self.lock().dyn_obs.clear();
        safe(&self.sim, "PUT", "/api/v1/world/obstacles", Some(&json!([])));
        safe(&self.sim, "PUT", "/api/v1/world/scenario", Some(&json!({"id": sid})));
        self.lock().world = json!({});
        self.events.emit("system", "SCENARIO_SWITCH", "info", "仓储场景地图切换", &format!("仿真进程场景 → {sid}"), json!({"scenario_id": sid}));
    }
    pub fn set_lidar_config(&self, body: Map<String, Value>) -> Value {
        let r = safe(&self.sim, "PUT", "/api/v1/sensors/lidar_config", Some(&Value::Object(body))).filter(truthy);
        let mut g = self.lock();
        let r = r.unwrap_or_else(|| g.lidar_config.clone());
        if let (Some(d), Some(s)) = (g.lidar_config.as_object_mut(), r.as_object()) { for (k, v) in s { d.insert(k.clone(), v.clone()); } }
        let lc = g.lidar_config.clone();
        g.telemetry.insert("lidar_config".into(), lc.clone());
        lc
    }
    pub fn fork_cmd(&self, b: &Value) -> Value { safe(&self.sim, "PUT", "/api/v1/fork", Some(b)).unwrap_or(json!({"error": "sim 不可用或车型无货叉"})) }
    pub fn cargo_add(&self, b: &Value) -> Result<Value, String> { self.sim.json("POST", "/api/v1/world/cargo", Some(b), None).map_err(|e| e.to_string()) }
    pub fn cargo_clear(&self) -> usize {
        let list = safe(&self.sim, "GET", "/api/v1/world/cargo", None);
        let items: Vec<Value> = list.as_ref().and_then(|v| v.as_array().cloned().or_else(|| v.get("cargo").and_then(|c| c.as_array().cloned()))).unwrap_or_default();
        let mut n = 0;
        for it in &items { if let Some(id) = it.get("id").and_then(|i| i.as_str()) { if safe(&self.sim, "DELETE", &format!("/api/v1/world/cargo/{id}"), None).is_some() { n += 1; } } }
        n
    }
    fn veh_pose(&self) -> Option<(f64, f64, f64)> {
        let st = safe(&self.sim, "GET", "/api/v1/state", None)?;
        let t = st.get("truth")?;
        Some((t.get("x")?.as_f64()?, t.get("y")?.as_f64()?, t.get("yaw")?.as_f64()?))
    }
    /// one-click fork pick: nearest floor pallet (or b.cargo) → nav taskflow {pick}
    pub fn fork_pick(&self, b: &Value) -> Result<Value, String> {
        let (px, py, _) = self.veh_pose().ok_or("仿真进程不可用")?;
        let list = safe(&self.sim, "GET", "/api/v1/world/cargo", None).unwrap_or(json!([]));
        let items: Vec<Value> = list.as_array().cloned().or_else(|| list.get("cargo").and_then(|c| c.as_array().cloned())).unwrap_or_default();
        let want = b.get("cargo").and_then(|c| c.as_str()).map(|s| s.to_string());
        let pick = items.iter().filter(|c| c.get("kind").and_then(|k| k.as_str()) == Some("pallet") && !c.get("carried").and_then(|v| v.as_bool()).unwrap_or(false)
                && want.as_deref().map(|w| c.get("id").and_then(|i| i.as_str()) == Some(w)).unwrap_or(true))
            .min_by(|a, b2| {
                let d = |c: &Value| { let (x, y) = (c["x"].as_f64().unwrap_or(0.0), c["y"].as_f64().unwrap_or(0.0)); (x - px).hypot(y - py) };
                d(a).partial_cmp(&d(b2)).unwrap_or(std::cmp::Ordering::Equal)
            }).ok_or("场景里没有可叉取的托盘 (先点「+ 车前托盘」)")?;
        let id = pick["id"].as_str().unwrap_or("").to_string();
        let flow = json!({"id": "ui_pick", "name": "叉取托盘", "steps": [{"type": "pick", "cargo": id, "height": b.get("height").and_then(|h| h.as_f64()).unwrap_or(0.25)}]});
        self.nav.json("POST", "/api/v1/taskflows/run", Some(&flow), None).map_err(|e| e.to_string())
    }
    /// one-click demo: clear cargo, lower fork, put a pallet 3.3 m ahead, then run the pick flow and lift high
    pub fn fork_demo(&self, b: &Value) -> Result<Value, String> {
        self.fork_stop_flow();
        self.cargo_clear();
        self.fork_cmd(&json!({"height": 0.0}));
        let (px, py, yaw) = self.veh_pose().ok_or("仿真进程不可用")?;
        let d = b.get("distance").and_then(|v| v.as_f64()).unwrap_or(3.3);
        let (x, y) = (px + yaw.cos() * d, py + yaw.sin() * d);
        self.cargo_add(&json!({"kind": "pallet", "id": "DEMO_P", "x": x, "y": y, "yaw": yaw}))?;
        let mast_max = safe(&self.sim, "GET", "/api/v1/fork", None).and_then(|f| f.get("mast_max_m").and_then(|v| v.as_f64())).unwrap_or(3.0);
        let h = b.get("height").and_then(|v| v.as_f64()).unwrap_or(1.2).min(mast_max);
        self.fork_pick(&json!({"cargo": "DEMO_P", "height": h}))
    }
    /// one-click place: put the carried pallet on the floor ~3.5 m ahead of the vehicle, same heading
    pub fn fork_place(&self, b: &Value) -> Result<Value, String> {
        let (px, py, yaw) = self.veh_pose().ok_or("仿真进程不可用")?;
        let d = b.get("distance").and_then(|v| v.as_f64()).unwrap_or(3.5);
        let (x, y) = (b.get("x").and_then(|v| v.as_f64()).unwrap_or(px + yaw.cos() * d), b.get("y").and_then(|v| v.as_f64()).unwrap_or(py + yaw.sin() * d));
        let flow = json!({"id": "ui_place", "name": "放置托盘", "steps": [{"type": "place", "x": x, "y": y, "yaw": b.get("yaw").and_then(|v| v.as_f64()).unwrap_or(yaw), "z": b.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0)}]});
        self.nav.json("POST", "/api/v1/taskflows/run", Some(&flow), None).map_err(|e| e.to_string())
    }
    pub fn fork_stop_flow(&self) { safe(&self.nav, "POST", "/api/v1/taskflows/stop", Some(&json!({}))); }
    pub fn set_io(&self, key: &str, v: bool) { safe(&self.sim, "PUT", &format!("/api/v1/io/di/{key}"), Some(&json!({"value": v}))); }

    pub fn broadcast_obstacles(&self) {
        let obs = Value::Array(self.lock().dyn_obs.clone());
        safe(&self.sim, "PUT", "/api/v1/world/obstacles", Some(&obs));
    }
    fn sync_obs(g: &mut St) { let o = Value::Array(g.dyn_obs.clone()); g.telemetry.insert("dynamic_obstacles".into(), o.clone()); g.telemetry.insert("obstacles".into(), o); }
    pub fn clear_obstacles(&self) {
        { let mut g = self.lock(); g.dyn_obs.clear(); Self::sync_obs(&mut g); }
        self.broadcast_obstacles();
        self.events.emit("sensors", "OBSTACLE_CHANGE", "info", "动态干扰路障已清空", "已清空移除主干道上所有动态干扰路障与物理实体，导轨路网恢复畅通", json!({"count": 0}));
    }
    pub fn add_obstacle(&self, x: f64, y: f64, w: f64, h: f64, ty: &str) -> Result<(), String> {
        let obs;
        {
            let mut g = self.lock();
            if !g.is_paused { return Err("仅在物理仿真暂停状态下允许添加扰动模块，请先暂停仿真".into()); }
            let id = g.dyn_obs.len() + 1;
            obs = json!({"id": id, "x": rnd(x, 2), "y": rnd(y, 2), "w": rnd(w, 2), "h": rnd(h, 2), "type": ty, "active": false});
            g.dyn_obs.push(obs.clone());
            Self::sync_obs(&mut g);
        }
        self.broadcast_obstacles();
        let tn = match ty { "pallet" => "标准木质栈板", "shelf" => "双层轻型货架", "box" => "工业周转纸箱", "person" => "车间作业人员", _ => "工业实体障碍物" };
        self.events.emit("sensors", "OBSTACLE_CHANGE", "warning", &format!("布置扰动物理实体 #{} ({tn})", obs["id"]), &format!("在坐标 ({x:.2}, {y:.2}) 放置规格为 {w:.2}x{h:.2}m 的{tn}，将在恢复仿真时正式生效"), obs);
        Ok(())
    }
    pub fn generate_random_obstacles(&self, count: usize) {
        let (cx, cy, stations, mut nodes, paused);
        {
            let g = self.lock();
            cx = f(&Value::Object(g.telemetry.clone()), "x", 0.0); cy = f(&Value::Object(g.telemetry.clone()), "y", 0.0);
            let xy = |v: &Value| (f(v, "x", 0.0), f(v, "y", 0.0));
            stations = g.world.get("stations").and_then(|s| s.as_array()).map(|a| a.iter().map(xy).collect::<Vec<_>>()).unwrap_or_default();
            nodes = g.world.pointer("/topology/nodes").and_then(|n| n.as_object()).map(|o| o.values().map(xy).collect::<Vec<_>>()).unwrap_or_default();
            paused = g.is_paused;
        }
        shuffle(&mut nodes);
        let defs = [("pallet", 1.2, 1.0), ("shelf", 2.0, 1.0), ("box", 0.8, 0.8), ("person", 0.5, 0.5)];
        let mut new_obs: Vec<Value> = vec![];
        for (nx, ny) in nodes {
            if new_obs.len() >= count { break; }
            let (ox, oy) = (rnd(nx + uniform(-0.15, 0.15), 2), rnd(ny + uniform(-0.15, 0.15), 2));
            if (ox - cx).hypot(oy - cy) < 1.4 { continue; }
            if stations.iter().any(|(sx, sy)| (ox - sx).hypot(oy - sy) < 1.1) { continue; }
            let (t, dw, dh) = defs[(rand_u64() % 4) as usize];
            new_obs.push(json!({"id": new_obs.len() + 1, "x": ox, "y": oy, "w": rnd(dw, 2), "h": rnd(dh, 2), "type": t, "active": !paused}));
        }
        let n = new_obs.len();
        { let mut g = self.lock(); g.dyn_obs = new_obs.clone(); Self::sync_obs(&mut g); }
        self.broadcast_obstacles();
        self.events.emit("sensors", "OBSTACLE_CHANGE", "warning", "随机布置动态干扰物理实体", &format!("在仓储拓扑主通道内随机部署了 {n} 处工业干扰实体(托盘/货架/纸箱/人员)，检验动态避障能力"), json!({"count": n, "obstacles": new_obs}));
    }
    pub fn set_simulation_pause(&self, paused: bool) {
        {
            let mut g = self.lock();
            g.is_paused = paused;
            g.pause_override = Some((paused, now() + 2.0));
            g.telemetry.insert("is_paused".into(), json!(paused));
            if !paused { for o in g.dyn_obs.iter_mut() { o["active"] = json!(true); } Self::sync_obs(&mut g); }
        }
        safe(&self.sim, "PUT", "/api/v1/sim", Some(&json!({"paused": paused})));
        if !paused { self.broadcast_obstacles(); }
        let a = if paused { "暂停" } else { "恢复" };
        self.events.emit("system", "SIM_STATE_CHANGE", "info", &format!("仿真环境已{a}"),
            &format!("操作员已{a}仿真时钟与物理步进{}", if !paused { "，所有暂存扰动模块已正式生效进入物理碰撞引擎" } else { "，当前可布置扰动模块" }), json!({"paused": paused}));
    }
    pub fn reset_simulation(&self) {
        self.cancel_nav();
        let orig;
        {
            let mut g = self.lock();
            orig = g.world.pointer("/scenario_def/origin").filter(|o| truthy(o)).cloned().unwrap_or(json!({"x": 0.0, "y": 0.0, "yaw": 0.0}));
            let (ox, oy, oyaw) = (f(&orig, "x", 0.0), f(&orig, "y", 0.0), f(&orig, "yaw", 0.0));
            for (k, v) in [("x", json!(ox)), ("y", json!(oy)), ("yaw", json!(oyaw)), ("vx", json!(0.0)), ("vy", json!(0.0)), ("wz", json!(0.0)), ("plan_path", json!([])), ("target_goal", Value::Null),
                ("nav_status", json!("IDLE")), ("nav_dist_rem", json!(0.0)), ("scan_pose", json!({"x": ox, "y": oy, "yaw": oyaw}))] { g.telemetry.insert(k.into(), v); }
        }
        self.clear_obstacles();
        safe(&self.sim, "POST", "/api/v1/sim/reset", Some(&json!({})));
        self.set_simulation_pause(false);
        self.events.emit("system", "SIM_RESET", "warning", "仿真环境已重置", "已将车辆位姿重置至初始原点并重置仿真状态", json!({"origin": orig}));
    }
}
