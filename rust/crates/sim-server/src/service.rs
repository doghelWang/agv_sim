//! SimService: owns the SimCore, runs physics/sensor/camera threads, buffers sensor frames, keeps events and nav feedback.
use crate::scen;
use serde_json::{json, Map, Value};
use sim_core::cameras::CamFrames;
use sim_core::core::{Frame3D, Settings, SimCore};
use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub const API_VERSION: &str = "1.0";
pub const F_BUMPER_FRONT: u8 = 1;
pub const F_BUMPER_REAR: u8 = 2;
pub const F_PAUSED: u8 = 4;
pub const F_CONTACT: u8 = 8;

pub fn wall() -> f64 { SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0) }
pub fn rnd(x: f64, n: i32) -> f64 { let m = 10f64.powi(n); (x * m).round_ties_even() / m }

pub fn ranges_json(v: &[f64], nd: i32) -> Value {
    Value::Array(v.iter().map(|&x| if x.is_finite() { json!(rnd(x, nd)) } else { Value::Null }).collect())
}

#[derive(Clone)]
pub struct Scan2D { pub seq: u64, pub t: f64, pub frame_id: String, pub pose: [f64; 3], pub angle_min: f64, pub angle_inc: f64, pub range_min: f64, pub range_max: f64, pub scan_hz: f64, pub ranges: Arc<Vec<f64>>, pub merged: bool }
#[derive(Clone)]
pub struct Scan3D { pub seq: u64, pub t: f64, pub frame_id: String, pub pose: [f64; 3], pub scan_hz: f64, pub f: Frame3D }
#[derive(Clone)]
pub enum Scan { D2(Scan2D), D3(Scan3D) }
impl Scan {
    pub fn seq(&self) -> u64 { match self { Scan::D2(s) => s.seq, Scan::D3(s) => s.seq } }
}
#[derive(Default)]
pub struct ScanBuf { pub seq: u64, pub data: Option<Scan> }

pub struct CamFrame { pub seq: u64, pub t: f64, pub pose: [f64; 3], pub info: Value, pub frames: Arc<CamFrames> }
#[derive(Default)]
pub struct CamBuf { pub seq: u64, pub data: Option<Arc<CamFrame>> }

pub struct CmdMeta { pub source: String, pub t_wall: f64, pub vx: f64, pub vy: f64, pub wz: f64 }

pub struct Inner {
    pub core: SimCore,
    pub scenarios: Map<String, Value>,
    pub spec: Value,
    pub base_spec: Value,
    pub overrides: Value,
    pub events: VecDeque<Value>,
    pub event_id: u64,
    pub nav_feedback: Value,
    pub cmd_meta: CmdMeta,
    pub lidar_bufs: Vec<(String, ScanBuf)>,
    pub lid_seen: Vec<u32>,
    pub lid3_seen: Vec<u32>,
    pub merged: ScanBuf,
    pub merged_seen: u32,
    pub merged_json: Option<(u64, Value)>,
    pub state_seq: u64,
    pub model_rev: u32,
    pub rtf_target: f64,
    pub cam_bufs: Vec<(String, CamBuf)>,
    pub next_cam: HashMap<String, f64>,
    pub cam_req: HashMap<String, f64>,
    pub last_discrete: Vec<bool>,
    pub last_coll: u32,
    pub cmd_udp: Option<(&'static str, u16)>,
}

pub struct Service {
    pub inner: Mutex<Inner>,
    pub cond: Condvar,
    pub stop: AtomicBool,
    pub config_path: String,
    pub base_path: String,
    pub overrides_path: String,
    pub camera_max_hz: f64,
    pub camera_idle_s: f64,
    pub lib: Option<Arc<mujoco_sys::MuJoCo>>,
}

pub fn upgrade_spec(mut spec: Value) -> Value {
    if spec.get("schema_version").and_then(|v| v.as_i64()).unwrap_or(1) >= 2 { return spec; }
    let g = |c: &Value, k: &str, d: f64| c.get(k).and_then(|v| v.as_f64()).unwrap_or(d);
    let ch = spec.as_object_mut().unwrap().entry("chassis").or_insert(json!({}));
    let (h, t, l, r) = (g(ch, "head_offset_m", 0.6), g(ch, "tail_offset_m", 0.6), g(ch, "left_offset_m", 0.4), g(ch, "right_offset_m", 0.4));
    if ch.get("footprint").is_none() { ch["footprint"] = json!([[h, l], [h, -r], [-t, -r], [-t, l]]); }
    let ty = ch.get("type").and_then(|t| t.as_str()).unwrap_or("diff_drive").to_string();
    ch["type"] = json!(if ty == "diff" { "diff_drive".to_string() } else { ty });
    if ch.get("mass_kg").is_none() { ch["mass_kg"] = json!(200.0); }
    let o = spec.as_object_mut().unwrap();
    o.entry("wheels").or_insert(json!([]));
    spec
}

pub fn register_scenario(scen: &mut Map<String, Value>, sc: &Value) -> Result<String, String> {
    let id = sc.get("id").and_then(|i| i.as_str()).ok_or("场景缺少 id")?.to_string();
    scen.insert(id.clone(), sc.clone());
    Ok(id)
}

impl Service {
    pub fn new(config_path: &str, scenarios: Map<String, Value>, scenario: &str, lib: Option<Arc<mujoco_sys::MuJoCo>>) -> Result<Arc<Service>, String> {
        let dir = std::path::Path::new(config_path).parent().map(|p| p.to_path_buf()).unwrap_or_default();
        let base_path = dir.join("robot_config.base.json").to_string_lossy().to_string();
        let overrides_path = dir.join("model_overrides.json").to_string_lossy().to_string();
        let rd = |p: &str| -> Result<Value, String> { serde_json::from_str(&std::fs::read_to_string(p).map_err(|e| format!("读取 {p} 失败: {e}"))?).map_err(|e| format!("{p}: {e}")) };
        let base = if std::path::Path::new(&base_path).exists() { upgrade_spec(rd(&base_path)?) } else {
            let b = upgrade_spec(rd(config_path)?);
            let _ = std::fs::write(&base_path, serde_json::to_string_pretty(&b).unwrap());
            b
        };
        let overrides = model::overrides::load_overrides(&overrides_path)?;
        let mut spec = model::overrides::apply_overrides(&base, &overrides);
        if let Some(repo) = rd(config_path).ok().and_then(|c| c.get("repo").cloned()).filter(model::overrides::truthy) { spec["repo"] = repo; }
        let env = |k: &str, d: f64| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
        let core = SimCore::new(spec.clone(), &scenarios, scenario, None, lib.clone(), Settings::default(), 1)?;
        let svc = Arc::new(Service {
            inner: Mutex::new(Inner {
                spec, base_spec: base, overrides, scenarios, events: VecDeque::new(), event_id: 0, nav_feedback: json!({"status": "OFFLINE", "t_wall": 0.0}),
                cmd_meta: CmdMeta { source: "none".into(), t_wall: 0.0, vx: 0.0, vy: 0.0, wz: 0.0 },
                lidar_bufs: vec![], lid_seen: vec![], lid3_seen: vec![], merged: ScanBuf::default(), merged_seen: 0, merged_json: None,
                state_seq: 0, model_rev: 1, rtf_target: 1.0, cam_bufs: vec![], next_cam: HashMap::new(), cam_req: HashMap::new(),
                last_discrete: vec![], last_coll: 0, cmd_udp: None, core,
            }),
            cond: Condvar::new(), stop: AtomicBool::new(false), config_path: config_path.into(), base_path, overrides_path,
            camera_max_hz: env("SIM_CAMERA_MAX_HZ", 10.0), camera_idle_s: env("SIM_CAMERA_IDLE_S", 3.0), lib,
        });
        { let mut g = svc.lock(); Self::reindex(&mut g); }
        Ok(svc)
    }

    pub fn lock(&self) -> MutexGuard<'_, Inner> { self.inner.lock().unwrap_or_else(|e| e.into_inner()) }

    /// (re)build per-sensor buffers after the core is (re)created
    pub fn reindex(g: &mut Inner) {
        let names: Vec<String> = g.core.lidars.iter().map(|l| l.name.clone()).chain(g.core.lidars3d.iter().map(|l| l.name.clone())).collect();
        let old: HashMap<String, ScanBuf> = std::mem::take(&mut g.lidar_bufs).into_iter().collect();
        g.lidar_bufs = names.into_iter().map(|n| { let b = ScanBuf { seq: old.get(&n).map(|b| b.seq).unwrap_or(0), data: None }; (n, b) }).collect();
        g.lid_seen = vec![0; g.core.lidars.len()];
        g.lid3_seen = vec![0; g.core.lidars3d.len()];
        g.merged_seen = 0;
        let oldc: HashMap<String, CamBuf> = std::mem::take(&mut g.cam_bufs).into_iter().collect();
        let cn: Vec<String> = g.core.cameras.iter().map(|c| c.lock().unwrap().name.clone()).collect();
        g.cam_bufs = cn.iter().map(|n| { let b = CamBuf { seq: oldc.get(n).map(|b| b.seq).unwrap_or(0), data: None }; (n.clone(), b) }).collect();
        g.next_cam = cn.into_iter().map(|n| (n, 0.0)).collect();
        g.last_discrete = Self::discrete_sig(&g.core);
    }

    pub fn emit(g: &mut Inner, kind: &str, level: &str, message: &str, data: Value) {
        g.event_id += 1;
        let e = json!({"id": g.event_id, "t": rnd(g.core.t, 3), "wall": wall(), "type": kind, "level": level, "message": message, "data": data});
        g.events.push_back(e);
        while g.events.len() > 500 { g.events.pop_front(); }
    }
    pub fn emit_s(&self, kind: &str, level: &str, message: &str, data: Value) { let mut g = self.lock(); Self::emit(&mut g, kind, level, message, data); }

    pub fn discrete_sig(c: &SimCore) -> Vec<bool> { c.bumpers.iter().map(|b| b.pressed).chain(c.photos.iter().map(|p| p.detected)).collect() }

    fn discrete_events(g: &mut Inner, old: &[bool], new: &[bool]) {
        let items: Vec<(&str, String, bool)> = g.core.bumpers.iter().map(|b| ("bumper", b.name.clone(), b.pressed)).chain(g.core.photos.iter().map(|p| ("photo", p.name.clone(), p.detected))).collect();
        if old.len() != items.len() { return; }
        let _ = new;
        for ((kind, name, v), o) in items.iter().zip(old) {
            if o == v { continue; }
            if *kind == "bumper" {
                Self::emit(g, "BUMPER", if *v { "danger" } else { "success" }, &format!("防撞触边[{name}] {}", if *v { "压下 — 禁止向该侧运动" } else { "释放" }), json!({"strip": name, "pressed": v}));
            } else {
                Self::emit(g, "PHOTO", if *v { "warning" } else { "info" }, &format!("光电[{name}] {}", if *v { "触发" } else { "解除" }), json!({"sensor": name, "detected": v}));
            }
        }
    }

    // ------------------------------------------------------------------ threads
    pub fn start(self: &Arc<Self>) {
        let me = self.clone();
        std::thread::Builder::new().name("physics".into()).spawn(move || me.physics_loop()).unwrap();
        let me = self.clone();
        std::thread::Builder::new().name("sensors".into()).spawn(move || me.sensor_loop()).unwrap();
        let me = self.clone();
        std::thread::Builder::new().name("cameras".into()).spawn(move || me.camera_loop()).unwrap();
        let (ct, sc, lids) = { let g = self.lock(); (g.core.chassis_type.clone(), g.core.scenario_id.clone(), g.lidar_bufs.iter().map(|(n, _)| n.clone()).collect::<Vec<_>>()) };
        self.emit_s("SIM_START", "info", &format!("仿真进程启动: 车型 {ct}, 场景 {sc}, 激光 [{}]", lids.iter().map(|n| format!("'{n}'")).collect::<Vec<_>>().join(", ")), json!({}));
    }

    fn physics_loop(&self) {
        while !self.stop.load(Ordering::Relaxed) {
            let dt = { self.lock().core.settings.dt };
            std::thread::sleep(Duration::from_secs_f64(dt * 0.5));
            let mut g = self.lock();
            let n = g.core.advance();
            if n > 0 { g.state_seq += 1; }
            let ds = Self::discrete_sig(&g.core);
            if ds != g.last_discrete {
                let old = std::mem::replace(&mut g.last_discrete, ds.clone());
                Self::discrete_events(&mut g, &old, &ds);
            }
            if g.core.collisions != g.last_coll {
                g.last_coll = g.core.collisions;
                let (n, c) = (g.last_coll, g.core.last_contact);
                Self::emit(&mut g, "COLLISION", "danger", &format!("车体碰撞 (累计 {n} 次)，防撞触边触发"), json!({"contact": c.map(|c| json!([c.0, c.1])), "count": n}));
            }
        }
    }

    fn sensor_loop(&self) {
        while !self.stop.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_millis(10));
            let mut g = self.lock();
            let g = &mut *g;
            let mut got = false;
            let lmax = g.core.settings.lidar_max_hz;
            for i in 0..g.core.lidars.len() {
                let f = &g.core.lid_out[i];
                if f.seq == 0 || f.seq == g.lid_seen[i] { continue; }
                g.lid_seen[i] = f.seq;
                let l = &g.core.lidars[i];
                let hz = if lmax > 0.0 { l.freq_hz.min(lmax) } else { l.freq_hz };
                let b = &mut g.lidar_bufs[i].1;
                b.seq += 1;
                b.data = Some(Scan::D2(Scan2D { seq: b.seq, t: rnd(f.t, 4), frame_id: l.frame_id.clone(), pose: [rnd(f.pose[0], 4), rnd(f.pose[1], 4), rnd(f.pose[2], 5)],
                    angle_min: l.angle_min, angle_inc: l.angle_inc, range_min: l.range_min, range_max: l.range_max, scan_hz: hz, ranges: f.ranges.clone(), merged: false }));
                got = true;
            }
            let n2 = g.core.lidars.len();
            for i in 0..g.core.lidars3d.len() {
                let f = &g.core.lid3_out[i];
                if f.seq == 0 || f.seq == g.lid3_seen[i] { continue; }
                g.lid3_seen[i] = f.seq;
                let l = &g.core.lidars3d[i];
                let b = &mut g.lidar_bufs[n2 + i].1;
                b.seq += 1;
                b.data = Some(Scan::D3(Scan3D { seq: b.seq, t: rnd(f.t, 4), frame_id: l.frame_id.clone(), pose: [rnd(f.pose[0], 4), rnd(f.pose[1], 4), rnd(f.pose[2], 5)], scan_hz: l.freq_hz, f: f.clone() }));
                got = true;
            }
            let f = &g.core.merged;
            if f.seq != 0 && f.seq != g.merged_seen {
                g.merged_seen = f.seq;
                let n = f.ranges.len() as f64;
                g.merged.seq += 1;
                g.merged.data = Some(Scan::D2(Scan2D { seq: g.merged.seq, t: rnd(f.t, 4), frame_id: "base_link".into(), pose: [rnd(f.pose[0], 4), rnd(f.pose[1], 4), rnd(f.pose[2], 5)],
                    angle_min: -std::f64::consts::PI + std::f64::consts::PI / n, angle_inc: 2.0 * std::f64::consts::PI / n, range_min: 0.05, range_max: g.core.merged_range,
                    scan_hz: g.core.settings.merged_hz, ranges: f.ranges.clone(), merged: true }));
                got = true;
            }
            if got { self.cond.notify_all(); }
        }
    }

    fn cam_wanted(&self, g: &Inner, name: &str, now: f64) -> bool { self.camera_idle_s <= 0.0 || now - g.cam_req.get(name).cloned().unwrap_or(0.0) <= self.camera_idle_s }

    fn camera_loop(&self) {
        while !self.stop.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_millis(5));
            let mut jobs = vec![];
            {
                let mut g = self.lock();
                let g = &mut *g;
                if g.core.cameras.is_empty() { drop(g); std::thread::sleep(Duration::from_millis(100)); continue; }
                let (t, now) = (g.core.t, wall());
                for i in 0..g.core.cameras.len() {
                    let (name, fps) = { let c = g.core.cameras[i].lock().unwrap(); (c.name.clone(), c.fps) };
                    let due = t >= g.next_cam.get(&name).cloned().unwrap_or(0.0) && (self.camera_idle_s <= 0.0 || now - g.cam_req.get(&name).cloned().unwrap_or(0.0) <= self.camera_idle_s);
                    if !due { continue; }
                    let hz = if self.camera_max_hz > 0.0 { fps.min(self.camera_max_hz) } else { fps };
                    g.next_cam.insert(name.clone(), t + 1.0 / hz.max(0.2));
                    if let Some(j) = g.core.cam_job(i) { jobs.push((name, j, t)); }
                }
            }
            for (name, (cam, scene, pose, noise), t) in jobs {
                let t0 = Instant::now();
                let (frames, info) = {
                    let mut c = cam.lock().unwrap();
                    let f = c.capture(&scene, pose[0], pose[1], pose[2], noise, 4);
                    c.last_ms = 0.8 * c.last_ms + 0.2 * t0.elapsed().as_secs_f64() * 1000.0;
                    (f, c.info())
                };
                let mut g = self.lock();
                if let Some(b) = g.cam_bufs.iter_mut().find(|(n, _)| *n == name).map(|(_, b)| b) {
                    b.seq += 1;
                    b.data = Some(Arc::new(CamFrame { seq: b.seq, t: rnd(t, 4), pose, info, frames: Arc::new(frames) }));
                    self.cond.notify_all();
                }
            }
        }
    }

    // ------------------------------------------------------------------ long-poll waits
    pub fn wait_lidar(&self, name: &str, after_seq: i64, wait: f64) -> Result<Option<Scan>, ()> {
        let deadline = Instant::now() + Duration::from_secs_f64(wait.clamp(0.0, 5.0));
        let mut g = self.lock();
        if name != "merged" && !g.lidar_bufs.iter().any(|(n, _)| n == name) { return Err(()); }
        loop {
            let b = if name == "merged" { &g.merged } else { &g.lidar_bufs.iter().find(|(n, _)| n == name).unwrap().1 };
            if (b.seq as i64 > after_seq && b.data.is_some()) || Instant::now() >= deadline { return Ok(b.data.clone()); }
            let left = deadline.saturating_duration_since(Instant::now()).max(Duration::from_millis(1));
            g = self.cond.wait_timeout(g, left).unwrap_or_else(|e| e.into_inner()).0;
        }
    }

    pub fn wait_camera(&self, name: &str, after_seq: i64, wait: f64) -> Result<Option<Arc<CamFrame>>, ()> {
        let mut after = after_seq;
        let mut wait = wait;
        let mut g = self.lock();
        let Some(i) = g.cam_bufs.iter().position(|(n, _)| n == name) else { return Err(()) };
        let now = wall();
        if !self.cam_wanted(&g, name, now) { after = after.max(g.cam_bufs[i].1.seq as i64); wait = wait.max(1.0); }
        g.cam_req.insert(name.to_string(), now);
        let deadline = Instant::now() + Duration::from_secs_f64(wait.clamp(0.0, 5.0));
        loop {
            let b = &g.cam_bufs[i].1;
            if (b.seq as i64 > after && b.data.is_some()) || Instant::now() >= deadline { return Ok(b.data.clone()); }
            let left = deadline.saturating_duration_since(Instant::now()).max(Duration::from_millis(1));
            g = self.cond.wait_timeout(g, left).unwrap_or_else(|e| e.into_inner()).0;
        }
    }

    pub fn cameras(&self) -> Value {
        let g = self.lock();
        let now = wall();
        let mut out = vec![];
        for (i, cam) in g.core.cameras.iter().enumerate() {
            let c = cam.lock().unwrap();
            let mut d = c.info();
            let name = c.name.clone();
            d["seq"] = json!(g.cam_bufs[i].1.seq);
            d["capture_ms"] = json!(rnd(c.last_ms, 1));
            d["active"] = json!(self.cam_wanted(&g, &name, now));
            d["render"] = json!("ray");
            out.push(d);
        }
        Value::Array(out)
    }

    // ------------------------------------------------------------------ state
    pub fn state(&self) -> Value {
        let g = self.lock();
        let c = &g.core;
        let (names, pos, vel, eff) = c.joint_state();
        let (mx, my, mth) = c.map_to_odom();
        let im = &c.imu_sample;
        let imu = json!({"wz": rnd(im.wz, 6), "ax": rnd(im.ax, 6), "ay": rnd(im.ay, 6), "az": rnd(im.az, 6), "yaw": rnd(im.yaw, 6)});
        json!({"seq": g.state_seq, "t": rnd(c.t, 4), "wall_time": wall(),
            "truth": {"frame": "map", "x": c.x, "y": c.y, "yaw": c.th, "vx": c.vx, "vy": c.vy, "wz": c.wz},
            "odom": {"frame": "odom", "x": c.odom.x, "y": c.odom.y, "yaw": c.odom.th, "vx": c.odom.vx, "vy": c.odom.vy, "wz": c.odom.wz},
            "map_to_odom": {"x": mx, "y": my, "yaw": mth}, "imu": imu,
            "joints": {"names": names, "position": pos, "velocity": vel, "effort": eff},
            "collision": {"count": c.collisions, "bumper_front": c.io.di("di_bumper_front"), "bumper_rear": c.io.di("di_bumper_rear"), "last_contact": c.last_contact.map(|p| json!([p.0, p.1]))},
            "paused": c.paused, "chassis": c.chassis_type, "scenario": c.scenario_id, "model_rev": g.model_rev})
    }

    /// stream state frame: returns (frame bytes, meta frame, key)
    pub fn state_frames(&self, last_meta: &Option<String>) -> (Vec<u8>, Option<Vec<u8>>, Option<String>) {
        let g = self.lock();
        let c = &g.core;
        let (names, pos, vel, eff) = c.joint_state();
        let (mx, my, mth) = c.map_to_odom();
        let im = &c.imu_sample;
        let fl = (if c.io.di("di_bumper_front") { F_BUMPER_FRONT } else { 0 }) | (if c.io.di("di_bumper_rear") { F_BUMPER_REAR } else { 0 }) | (if c.paused { F_PAUSED } else { 0 }) | (if c.last_contact.is_some() { F_CONTACT } else { 0 });
        let lc = c.last_contact.unwrap_or((0.0, 0.0));
        let od = &c.odom;
        let mut b: Vec<u8> = Vec::with_capacity(207 + 24 * names.len());
        b.extend_from_slice(&(g.state_seq as u32).to_le_bytes());
        for v in [c.t, wall(), c.x, c.y, c.th, c.vx, c.vy, c.wz, od.x, od.y, od.th, od.vx, od.vy, od.wz, mx, my, mth, im.wz, im.ax, im.ay, im.az, im.yaw] { b.extend_from_slice(&v.to_le_bytes()); }
        b.extend_from_slice(&c.collisions.to_le_bytes());
        b.push(fl);
        b.extend_from_slice(&lc.0.to_le_bytes());
        b.extend_from_slice(&lc.1.to_le_bytes());
        b.extend_from_slice(&g.model_rev.to_le_bytes());
        b.extend_from_slice(&(names.len() as u16).to_le_bytes());
        for v in pos.iter().chain(vel.iter()).chain(eff.iter()) { b.extend_from_slice(&v.to_le_bytes()); }
        let key = format!("{}|{}|{}", c.chassis_type, c.scenario_id, names.join(","));
        let meta = if last_meta.as_deref() != Some(key.as_str()) {
            let m = serde_json::to_vec(&json!({"chassis": c.chassis_type, "scenario": c.scenario_id, "joint_names": names})).unwrap();
            Some(frame(2, &m))
        } else { None };
        (frame(1, &b), meta, Some(key))
    }

    pub fn io_frame(&self) -> Vec<u8> {
        let b = serde_json::to_vec(&json!({"io": self.io(), "photos": self.photoelectric()["sensors"]})).unwrap();
        frame(3, &b)
    }

    pub fn scan_frames(&self, sent: &mut HashMap<String, u64>) -> Vec<u8> {
        let g = self.lock();
        let mut out = vec![];
        let items: Vec<(String, &ScanBuf)> = g.lidar_bufs.iter().filter(|(_, b)| matches!(b.data, Some(Scan::D2(_)))).map(|(n, b)| (n.clone(), b)).chain(std::iter::once(("merged".to_string(), &g.merged))).collect();
        for (name, b) in items {
            let Some(Scan::D2(d)) = &b.data else { continue };
            if b.seq <= sent.get(&name).cloned().unwrap_or(0) { continue; }
            sent.insert(name.clone(), b.seq);
            let mut m = scan2d_meta(d);
            m["name"] = json!(name);
            let mj = serde_json::to_vec(&m).unwrap();
            let mut body = Vec::with_capacity(2 + mj.len() + 4 * d.ranges.len());
            body.extend_from_slice(&(mj.len() as u16).to_le_bytes());
            body.extend_from_slice(&mj);
            for r in d.ranges.iter() { body.extend_from_slice(&(*r as f32).to_le_bytes()); }
            out.extend_from_slice(&frame(4, &body));
        }
        out
    }

    // ------------------------------------------------------------------ misc status
    pub fn imu(&self) -> Value {
        let g = self.lock();
        let im = &g.core.imu_sample;
        json!({"wz": im.wz, "ax": im.ax, "ay": im.ay, "az": im.az, "yaw": im.yaw, "t": rnd(g.core.t, 4), "frame_id": "imu_link"})
    }
    pub fn sim_status(&self) -> Value {
        let g = self.lock();
        let mut s = g.core.status();
        s["paused"] = json!(g.core.paused);
        s["rtf_target"] = json!(g.rtf_target);
        s["api_version"] = json!(API_VERSION);
        s["cmd_udp_port"] = json!(g.cmd_udp.map(|c| c.1));
        s["cmd_udp"] = json!(g.cmd_udp.map(|c| c.0));
        s
    }
    pub fn photoelectric(&self) -> Value { let g = self.lock(); json!({"t": rnd(g.core.t, 4), "sensors": g.core.photos.iter().map(|p| p.view()).collect::<Vec<_>>()}) }
    pub fn bumpers(&self) -> Value {
        let g = self.lock();
        let c = &g.core;
        json!({"t": rnd(c.t, 4), "count": c.collisions, "last_contact": c.last_contact.map(|p| json!([p.0, p.1])), "any_pressed": c.bumpers.iter().any(|b| b.pressed),
            "strips": c.bumpers.iter().map(|b| b.view()).collect::<Vec<_>>(), "bumper_front": c.io.di("di_bumper_front"), "bumper_rear": c.io.di("di_bumper_rear")})
    }
    pub fn io(&self) -> Value { self.lock().core.io.state() }
    pub fn set_io(&self, di: Option<&Value>, do_: Option<&Value>) -> Value {
        let mut g = self.lock();
        if let Some(d) = di.and_then(|d| d.as_object()) { for (k, v) in d { g.core.io.set_di(k, truthy(v)); } }
        if let Some(d) = do_.and_then(|d| d.as_object()) { for (k, v) in d { g.core.io.set_do(k, truthy(v)); } }
        let st = g.core.io.state();
        if let Some(es) = di.and_then(|d| d.get("di_estop")) {
            let on = truthy(es);
            Self::emit(&mut g, "ESTOP", if on { "danger" } else { "success" }, if on { "急停按下 (抱闸)" } else { "急停复位" }, json!({}));
        }
        st
    }
    pub fn codes(&self) -> Value { Value::Array(self.lock().core.detect_codes()) }

    pub fn sensors(&self) -> Value {
        let g = self.lock();
        let c = &g.core;
        let mut lid = vec![];
        for (i, l) in c.lidars.iter().enumerate() {
            let mount: Map<String, Value> = ["x", "y", "z", "roll", "pitch", "yaw"].iter().map(|k| (k.to_string(), l.cfg.get(*k).cloned().unwrap_or(json!(0.0)))).collect();
            lid.push(json!({"name": l.name, "type": "2d", "frame_id": l.frame_id, "model": l.cfg.get("vendor_model").filter(|v| !v.is_null()).or_else(|| l.cfg.get("model")).cloned().unwrap_or(Value::Null),
                "mount": Value::Object(mount), "fov_deg": l.fov.to_degrees(), "beams": l.n, "range_max": l.range_max, "freq_hz": l.freq_hz, "seq": g.lidar_bufs[i].1.seq}));
        }
        let n2 = c.lidars.len();
        for (i, l) in c.lidars3d.iter().enumerate() {
            let mount: Map<String, Value> = ["x", "y", "z", "roll", "pitch", "yaw"].iter().map(|k| (k.to_string(), l.cfg.get(*k).cloned().unwrap_or(json!(0.0)))).collect();
            lid.push(json!({"name": l.name, "type": "3d", "frame_id": l.frame_id, "model": l.cfg.get("vendor_model").cloned().unwrap_or(Value::Null), "mount": Value::Object(mount),
                "vfov_deg": [l.vmin.to_degrees(), l.vmax.to_degrees()], "points_per_frame": l.n, "real_points_per_frame": l.real_points, "range_max": l.range_max, "freq_hz": l.freq_hz,
                "topic_hint": l.cfg.get("topic").cloned().unwrap_or(Value::Null), "imu_topic_hint": l.cfg.get("imu_topic").cloned().unwrap_or(Value::Null), "seq": g.lidar_bufs[n2 + i].1.seq}));
        }
        let cs: Vec<Value> = c.cameras.iter().enumerate().map(|(i, cam)| { let mut d = cam.lock().unwrap().info(); d["seq"] = json!(g.cam_bufs[i].1.seq); d }).collect();
        let bump: Vec<Value> = c.bumpers.iter().map(|b| { let mut v = b.view(); v.as_object_mut().unwrap().remove("pressed"); v }).collect();
        json!({"lidars": lid, "imu": g.spec.get("imu").cloned().unwrap_or(Value::Null), "cameras": g.spec.get("cameras").cloned().unwrap_or(json!([])),
            "io": g.spec.get("io").cloned().unwrap_or(json!({})), "merged_scan": {"seq": g.merged.seq, "hz": c.settings.merged_hz},
            "photoelectric": c.photos.iter().map(|p| p.view()).collect::<Vec<_>>(), "camera_streams": cs, "bumpers": bump})
    }

    pub fn set_lidar_config(&self, cfg: &Value) -> Value {
        let mut beams = cfg.get("beams").and_then(|b| b.as_i64()).filter(|b| *b != 0);
        if beams.is_none() { if let Some(a) = cfg.get("angle_resolution_deg").and_then(|a| a.as_f64()).filter(|a| *a != 0.0) { beams = Some((360.0 / a).round_ties_even() as i64); } }
        let mut g = self.lock();
        g.core.set_lidar_config(beams, cfg.get("sensor_resolution_deg").and_then(|v| v.as_f64()), cfg.get("range_max").and_then(|v| v.as_f64()), None);
        if let Some(f) = cfg.get("freq_hz").and_then(|f| f.as_f64()).filter(|f| *f != 0.0) { g.core.settings.merged_hz = f.clamp(1.0, 50.0); }
        let bins = g.core.merged_bins;
        json!({"beams": bins, "angle_resolution_deg": rnd(360.0 / bins as f64, 3), "freq_hz": g.core.settings.merged_hz, "range_max": g.core.merged_range})
    }

    // ------------------------------------------------------------------ control & feedback
    pub fn set_cmd(&self, vx: f64, vy: f64, wz: f64, source: &str) -> Value {
        let mut g = self.lock();
        g.core.set_cmd(vx, vy, wz);
        g.cmd_meta = CmdMeta { source: source.into(), t_wall: wall(), vx, vy, wz };
        json!({"accepted": true, "estop": g.core.estop(), "paused": g.core.paused, "t": rnd(g.core.t, 4)})
    }
    pub fn control(&self) -> Value {
        let g = self.lock();
        let m = &g.cmd_meta;
        let age = wall() - m.t_wall;
        json!({"source": m.source, "t_wall": m.t_wall, "vx": m.vx, "vy": m.vy, "wz": m.wz, "age_s": rnd(age, 3),
            "watchdog_timeout_s": g.core.settings.cmd_timeout, "watchdog_active": age > g.core.settings.cmd_timeout, "estop": g.core.estop()})
    }
    pub fn set_nav_feedback(&self, fb: &Value) {
        let mut g = self.lock();
        let prev = g.nav_feedback.get("status").cloned();
        let mut f = fb.clone();
        if let Some(o) = f.as_object_mut() { o.insert("t_wall".into(), json!(wall())); }
        g.nav_feedback = f;
        let st = fb.get("status").and_then(|s| s.as_str()).unwrap_or("");
        if Some(json!(st)) != prev && ["ARRIVED", "FAILED", "NO_PATH", "CANCELED"].contains(&st) {
            let mid = fb.get("mission_id").cloned().unwrap_or(Value::Null);
            let ms = match &mid { Value::Null => "None".to_string(), Value::String(s) => s.clone(), v => v.to_string() };
            Self::emit(&mut g, "NAV_RESULT", if st == "ARRIVED" { "success" } else { "warning" }, &format!("执行进程回馈: 任务 #{ms} {st}"), json!({"mission_id": mid}));
        }
    }
    pub fn get_nav_feedback(&self) -> Value {
        let g = self.lock();
        let mut fb = g.nav_feedback.clone();
        let age = rnd(wall() - fb.get("t_wall").and_then(|t| t.as_f64()).unwrap_or(0.0), 3);
        fb["age_s"] = json!(age);
        if age > 3.0 { fb["online"] = json!(false); } else if fb.get("online").is_none() { fb["online"] = json!(true); }
        fb
    }

    pub fn set_sim(&self, paused: Option<bool>, rtf: Option<f64>) {
        let mut g = self.lock();
        if let Some(p) = paused { g.core.paused = p; }
        if let Some(r) = rtf { g.rtf_target = r.clamp(0.1, 5.0); }
        g.core.rtf_target = g.rtf_target;
        if let Some(p) = paused { Self::emit(&mut g, "SIM_PAUSE", "info", if p { "仿真暂停" } else { "仿真恢复" }, json!({})); }
    }
    pub fn reset(&self, x: Option<f64>, y: Option<f64>, yaw: Option<f64>) {
        let mut g = self.lock();
        let o = g.scenarios.get(&g.core.scenario_id).and_then(|s| s.get("origin")).cloned().unwrap_or(json!({}));
        let gv = |k: &str, v: Option<f64>| v.unwrap_or_else(|| o.get(k).and_then(|x| x.as_f64()).unwrap_or(0.0));
        g.core.reset_pose(gv("x", x), gv("y", y), gv("yaw", yaw));
        Self::emit(&mut g, "SIM_RESET", "warning", "车辆位姿复位", json!({}));
    }
    pub fn events_since(&self, since: i64) -> Value {
        let g = self.lock();
        Value::Array(g.events.iter().filter(|e| e["id"].as_i64().unwrap_or(0) > since).cloned().collect())
    }

    // ------------------------------------------------------------------ world
    pub fn world(&self) -> Result<Value, String> {
        let g = self.lock();
        let id = g.core.scenario_id.clone();
        let mut meta = scen::metadata(&g.scenarios, &id);
        meta["topology"] = scen::topology(&g.scenarios, &id);
        meta["obstacles"] = Value::Array(g.core.world.obstacles.iter().map(|o| Value::Object(o.clone())).collect());
        meta["heights"] = json!({"wall": 6.0, "shelf": 2.5, "ceiling": 6.0});
        meta["scenario_def"] = g.scenarios[&id].clone();
        Ok(meta)
    }
    pub fn set_scenario(&self, sid: &str) -> Result<(), String> {
        let mut g = self.lock();
        if !g.scenarios.contains_key(sid) { return Err(sid.to_string()); }
        let sc = g.scenarios.clone();
        g.core.set_scenario(&sc, sid)?;
        g.core.cmd = [0.0; 3];
        Self::emit(&mut g, "SCENARIO", "info", &format!("场景切换 → {sid}"), json!({"scenario": sid}));
        Ok(())
    }
    pub fn load_scene(&self, sc: &Value) -> Result<String, String> {
        let id = { let mut g = self.lock(); register_scenario(&mut g.scenarios, sc)? };
        self.set_scenario(&id)?;
        Ok(id)
    }
    pub fn set_obstacles(&self, obs: Vec<Map<String, Value>>) -> Result<(), String> {
        let mut g = self.lock();
        let n = obs.len();
        g.core.set_obstacles(obs)?;
        g.core.revision += 0;
        Self::emit(&mut g, "OBSTACLES", "info", &format!("动态障碍物更新: {n} 个"), json!({"count": n}));
        Ok(())
    }
    pub fn set_chassis(&self, ct: Option<&str>) -> Result<String, String> {
        let mut g = self.lock();
        let c = g.core.set_chassis(ct)?;
        Self::emit(&mut g, "CHASSIS", "info", &format!("车型切换 → {c}"), json!({"chassis": c}));
        Ok(c)
    }
    pub fn occupancy_map(&self, res: f64) -> Value {
        let g = self.lock();
        let sid = g.core.scenario_id.clone();
        let sc = &g.scenarios[&sid];
        if let Some(mf) = sc.get("map_files") {
            let (pp, yp) = (mf.get("pgm").and_then(|v| v.as_str()), mf.get("yaml").and_then(|v| v.as_str()));
            if let (Some(pp), Some(yp)) = (pp, yp) {
                if let (Ok(pgm), Ok(yml)) = (std::fs::read(pp), std::fs::read_to_string(yp)) {
                    let re_i = regex::Regex::new(r"(?m)^image:.*$").unwrap();
                    let yml = re_i.replace_all(&yml, format!("image: {sid}.pgm")).to_string();
                    let mres = regex::Regex::new(r"(?m)^resolution:\s*([\d.eE+-]+)").unwrap().captures(&yml).and_then(|m| m[1].parse::<f64>().ok());
                    let org: Vec<f64> = regex::Regex::new(r"(?m)^origin:\s*\[([^\]]*)\]").unwrap().captures(&yml)
                        .map(|m| m[1].split(',').take(2).filter_map(|x| x.trim().parse().ok()).collect()).unwrap_or(vec![0.0, 0.0]);
                    let hdr: Vec<&[u8]> = pgm[..pgm.len().min(200)].split(|b| b.is_ascii_whitespace()).filter(|t| !t.is_empty() && t[0] != b'#').collect();
                    let (w, h) = (hdr.get(1).and_then(|t| std::str::from_utf8(t).ok()?.parse::<usize>().ok()).unwrap_or(0), hdr.get(2).and_then(|t| std::str::from_utf8(t).ok()?.parse::<usize>().ok()).unwrap_or(0));
                    return json!({"id": sid, "pgm": pgm, "yaml": yml, "width": w, "height": h, "resolution": mres.unwrap_or(res), "origin": org});
                }
            }
        }
        let (img, w, h, (ox, oy)) = scen::rasterize(sc, res);
        let mut pgm = format!("P5\n# scenario {sid}\n{w} {h}\n255\n").into_bytes();
        pgm.extend_from_slice(&img);
        let yml = format!("image: {sid}.pgm\nmode: trinary\nresolution: {res}\norigin: [{ox:.3}, {oy:.3}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n");
        json!({"id": sid, "pgm": pgm, "yaml": yml, "width": w, "height": h, "resolution": res, "origin": [ox, oy]})
    }

    pub fn instance(&self) -> Value {
        let g = self.lock();
        let sc = g.scenarios.get(&g.core.scenario_id).cloned().unwrap_or(json!({}));
        let env = |k: &str| std::env::var(k).ok().map(|v| json!(v)).unwrap_or(Value::Null);
        let mut model = g.spec.get("repo").cloned().filter(|r| r.is_object()).unwrap_or(json!({}));
        model["file"] = g.spec.get("model_file").cloned().unwrap_or(Value::Null);
        model["chassis"] = json!(g.core.chassis_type);
        json!({"instance_id": env("INSTANCE_ID"), "instance_name": env("INSTANCE_NAME"), "hub": env("HUB_API"), "model": model,
            "scene": {"id": sc.get("id"), "name": sc.get("name")}, "model_rev": g.model_rev})
    }

    pub fn model(&self) -> Value {
        let g = self.lock();
        let mut sp = g.spec.clone();
        sp["active_chassis"] = json!(g.core.chassis_type);
        sp["active_wheels"] = Value::Array(g.core.wheels_d.clone());
        sp["model_rev"] = json!(g.model_rev);
        let mut nl = Map::new();
        for ct in model::limits::CHASSIS_TYPES { match model::limits::effective_limits(&g.spec, ct) { Ok(l) => { nl.insert(ct.to_string(), l); } Err(_) => { nl.clear(); break; } } }
        if !nl.is_empty() { sp["nav_limits"] = Value::Object(nl); }
        sp
    }

    pub fn urdf(&self) -> String {
        let g = self.lock();
        let mut sp = g.spec.clone();
        sp["wheels"] = Value::Array(g.core.wheels_d.clone());
        model::urdf::generate_urdf(&sp)
    }

    pub fn model_editor(&self) -> Value {
        let g = self.lock();
        let photos: Vec<Value> = g.core.photos.iter().map(|p| p.view()).collect();
        json!({"spec": g.spec, "base_file": std::path::Path::new(&self.base_path).file_name().and_then(|n| n.to_str()).unwrap_or(""), "overrides": g.overrides,
            "audit": model::overrides::audit(&g.spec, Some(&g.overrides)), "sensors": model::overrides::sensor_list(&g.spec, Some(&photos)), "sensor_types": model::overrides::sensor_types()})
    }

    pub fn preview_overrides(&self, ov: &Value) -> Value {
        let g = self.lock();
        let sp = model::overrides::apply_overrides(&g.base_spec, ov);
        json!({"spec": sp, "audit": model::overrides::audit(&sp, Some(ov)), "sensors": model::overrides::sensor_list(&sp, None), "urdf": model::urdf::generate_urdf(&sp)})
    }

    fn write_model_files(&self, sp: &Value) {
        let dir = std::path::Path::new(&self.config_path).parent().map(|p| p.to_path_buf()).unwrap_or_default();
        let _ = std::fs::write(&self.config_path, serde_json::to_string_pretty(sp).unwrap());
        let _ = std::fs::write(dir.join("robot.urdf"), model::urdf::generate_urdf(sp));
    }

    /// rebuild the core from a spec; keep=true preserves pose and dynamic obstacles
    fn rebuild(g: &mut Inner, spec: Value, chassis: Option<String>, keep: bool, lib: Option<Arc<mujoco_sys::MuJoCo>>) -> Result<(), String> {
        let (pose, obs, sid) = ((g.core.x, g.core.y, g.core.th), g.core.world.obstacles.clone(), g.core.scenario_id.clone());
        let rev = g.model_rev + 1;
        let core = SimCore::new(spec.clone(), &g.scenarios, &sid, chassis.as_deref(), lib, Settings::default(), rev)?;
        g.core = core;
        g.spec = spec;
        g.model_rev = rev;
        if keep { let _ = g.core.set_obstacles(obs); g.core.reset_pose(pose.0, pose.1, pose.2); }
        Self::reindex(g);
        Ok(())
    }

    pub fn apply_overrides(&self, ov: &Value, save: bool) -> Result<Value, String> {
        let mut ov = ov.clone();
        if ov.get("version").is_none() { ov["version"] = json!(1); }
        let (mut sp, n_ov) = { let g = self.lock(); let sp = model::overrides::apply_overrides(&g.base_spec, &ov); (sp, g.spec.get("repo").cloned()) };
        if let Some(r) = n_ov.filter(model::overrides::truthy) { sp["repo"] = r; }
        if save { model::overrides::save_overrides(&ov, &self.overrides_path)?; self.write_model_files(&sp); }
        let msg = format!("模型补全已应用: {} 项人工参数，传感器 {} 激光 / {} 相机", sp["provenance"].as_object().map(|o| o.len()).unwrap_or(0),
            sp["lidars"].as_array().map(|a| a.len()).unwrap_or(0), sp["cameras"].as_array().map(|a| a.len()).unwrap_or(0));
        {
            let mut g = self.lock();
            let ct = g.core.chassis_type.clone();
            let ch = if Some(ct.as_str()) != sp["chassis"]["type"].as_str() { Some(ct) } else { None };
            g.overrides = ov;
            Self::rebuild(&mut g, sp, ch, true, self.lib.clone())?;
            Self::emit(&mut g, "MODEL_APPLY", "success", &msg, json!({}));
        }
        Ok(self.model_editor())
    }

    /// re-read base + overrides and rebuild (no cmodel parsing)
    pub fn reload_model(&self, save: bool) -> Result<Value, String> {
        let (sp, repo) = { let g = self.lock(); (model::overrides::apply_overrides(&g.base_spec, &g.overrides), g.spec.get("repo").cloned()) };
        let mut sp = sp;
        if let Some(r) = repo.filter(model::overrides::truthy) { sp["repo"] = r; }
        if save { self.write_model_files(&sp); }
        let msg = format!("模型重建完成: {} ({})", sp.get("model_file").map(|v| v.as_str().map(|s| s.to_string()).unwrap_or(v.to_string())).unwrap_or("None".into()), sp["chassis"]["type"].as_str().unwrap_or(""));
        { let mut g = self.lock(); Self::rebuild(&mut g, sp, None, false, self.lib.clone())?; Self::emit(&mut g, "MODEL_RELOAD", "success", &msg, json!({})); }
        Ok(self.model())
    }

    pub fn load_model_from_hub(&self, hub: &str, mid: &str, ver: &str) -> Result<Value, String> {
        let data = std::path::Path::new(&self.config_path).parent().map(|p| p.to_path_buf()).unwrap_or_default();
        crate::boot::pull_model(hub, &data.to_string_lossy(), mid, ver)?;
        let base = upgrade_spec(serde_json::from_str(&std::fs::read_to_string(&self.base_path).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?);
        let ov = model::overrides::load_overrides(&self.overrides_path)?;
        let mut sp = model::overrides::apply_overrides(&base, &ov);
        if let Ok(t) = std::fs::read_to_string(&self.config_path) { if let Some(r) = serde_json::from_str::<Value>(&t).ok().and_then(|c| c.get("repo").cloned()) { sp["repo"] = r; } }
        {
            let mut g = self.lock();
            g.base_spec = base; g.overrides = ov;
            Self::rebuild(&mut g, sp, None, true, self.lib.clone())?;
            Self::emit(&mut g, "MODEL_RELOAD", "success", &format!("已加载平台模型 {mid} {ver}"), json!({}));
        }
        Ok(self.model())
    }

    pub fn snapshot(&self, scans: bool) -> Value {
        let mut snap = json!({"state": self.state(), "io": self.io(), "codes": self.codes(), "sim": self.sim_status(), "photoelectric": self.photoelectric()["sensors"],
            "bumpers": self.bumpers(), "cameras": self.cameras(), "control": self.control(), "nav": self.get_nav_feedback()});
        let mut g = self.lock();
        if let Some(Scan::D2(m)) = g.merged.data.clone() {
            let cached = match &g.merged_json { Some((s, v)) if *s == m.seq => v.clone(), _ => { let v = ranges_json(&m.ranges, 3); g.merged_json = Some((m.seq, v.clone())); v } };
            let mut meta = scan2d_meta(&m);
            meta["ranges"] = cached;
            snap["merged_scan"] = meta;
        }
        if scans {
            let mut out = Map::new();
            let n2 = g.core.lidars.len();
            for (i, (name, b)) in g.lidar_bufs.iter().enumerate() {
                match &b.data {
                    Some(Scan::D2(d)) => {
                        let step = (d.ranges.len() / 360).max(1);
                        let r: Vec<Value> = d.ranges.iter().step_by(step).map(|&v| json!(rnd(if v.is_finite() { v } else { -1.0 }, 2))).collect();
                        out.insert(name.clone(), json!({"type": "2d", "seq": d.seq, "pose": pose_json(&d.pose), "angle_min": d.angle_min, "angle_inc": d.angle_inc * step as f64, "ranges": r}));
                    }
                    Some(Scan::D3(d)) => {
                        let l3 = &g.core.lidars3d[i - n2];
                        let n = d.f.xyz.len() / 3;
                        let step = (n / 3000).max(1);
                        let mut pts = vec![];
                        for k in (0..n).step_by(step) {
                            let p = l3.points_base([d.f.xyz[3 * k] as f64, d.f.xyz[3 * k + 1] as f64, d.f.xyz[3 * k + 2] as f64]);
                            pts.extend(p.iter().map(|v| json!(rnd(*v, 2))));
                        }
                        out.insert(name.clone(), json!({"type": "3d", "seq": d.seq, "pose": pose_json(&d.pose), "n_raw": n, "points3d": pts}));
                    }
                    None => {}
                }
            }
            snap["lidar_scans"] = Value::Object(out);
        }
        snap
    }

    /// UDP "AGVC" velocity channel: 48-byte packets (u32 magic, u32 seq, f64 vx, vy, wz, 16-byte source)
    pub fn start_cmd_udp(self: &Arc<Self>, port: u16) {
        if std::env::var("SIM_CMD_UDP").map(|v| v == "0").unwrap_or(false) { return; }
        let host = std::env::var("AGV_BIND").unwrap_or_default();
        let host = if host.is_empty() || host == "0.0.0.0" { "0.0.0.0".to_string() } else { host };
        let sk = match std::net::UdpSocket::bind((host.as_str(), port)) { Ok(s) => s, Err(e) => { eprintln!("[sim] UDP 指令通道未启用: {e}"); return; } };
        self.lock().cmd_udp = Some(("rust", port));
        let me = self.clone();
        std::thread::Builder::new().name("cmd-udp".into()).spawn(move || {
            let mut buf = [0u8; 128];
            while !me.stop.load(Ordering::Relaxed) {
                let Ok(n) = sk.recv(&mut buf) else { continue };
                if n >= 48 && &buf[..4] == b"AGVC" {
                    let f = |o: usize| f64::from_le_bytes(buf[o..o + 8].try_into().unwrap());
                    let (vx, vy, wz) = (f(8), f(16), f(24));
                    let src = &buf[32..48];
                    let end = src.iter().position(|b| *b == 0).unwrap_or(16);
                    let src = String::from_utf8_lossy(&src[..end]).to_string();
                    if vx.is_finite() && vy.is_finite() && wz.is_finite() { me.set_cmd(vx, vy, wz, if src.is_empty() { "udp" } else { &src }); }
                }
            }
        }).unwrap();
    }
}

pub fn truthy(v: &Value) -> bool {
    match v { Value::Bool(b) => *b, Value::Null => false, Value::Number(n) => n.as_f64().map(|x| x != 0.0).unwrap_or(false), Value::String(s) => !s.is_empty(), Value::Array(a) => !a.is_empty(), Value::Object(o) => !o.is_empty() }
}
pub fn pose_json(p: &[f64; 3]) -> Value { json!({"x": p[0], "y": p[1], "yaw": p[2]}) }
pub fn frame(ty: u8, payload: &[u8]) -> Vec<u8> {
    let mut o = Vec::with_capacity(5 + payload.len());
    o.extend_from_slice(&(payload.len() as u32).to_le_bytes());
    o.push(ty);
    o.extend_from_slice(payload);
    o
}
/// meta fields of a 2D scan, same order as the Python dict (without ranges)
pub fn scan2d_meta(d: &Scan2D) -> Value {
    let mut m = Map::new();
    if !d.merged { m.insert("type".into(), json!("2d")); }
    m.insert("seq".into(), json!(d.seq)); m.insert("t".into(), json!(d.t)); m.insert("frame_id".into(), json!(d.frame_id)); m.insert("pose".into(), pose_json(&d.pose));
    m.insert("angle_min".into(), json!(d.angle_min)); m.insert("angle_increment".into(), json!(d.angle_inc));
    m.insert("range_min".into(), json!(d.range_min)); m.insert("range_max".into(), json!(d.range_max)); m.insert("scan_hz".into(), json!(d.scan_hz));
    Value::Object(m)
}
