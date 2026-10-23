//! SimCore: the whole simulation state + fixed-step real-time loop (port of engine.py + rt.py + simcore_rt.c).
use crate::geom::{collides, raycast2d, se2_integrate, wrap};
use crate::io::IoSim;
use crate::kin::{build_kinematics, Chassis};
use crate::mj::*;
use crate::rng::Rng;
use crate::sensors::*;
use crate::world::*;
use mujoco_sys as ms;
use serde_json::{json, Map, Value};
use std::f64::consts::PI;
use std::sync::Arc;
use std::time::Instant;

pub const MAXL3: usize = 4;
pub const MAXB: usize = 8;
pub const MAXP: usize = 16;

#[derive(Clone, Debug, Default)]
pub struct Frame2D { pub seq: u32, pub t: f64, pub pose: [f64; 3], pub ranges: Arc<Vec<f64>> }

#[derive(Clone, Debug, Default)]
pub struct Frame3D {
    pub seq: u32, pub t: f64, pub pose: [f64; 3],
    pub xyz: Arc<Vec<f32>>, pub intensity: Arc<Vec<f32>>, pub line: Arc<Vec<u8>>, pub offset_time: Arc<Vec<f64>>, pub slice: Arc<Vec<f64>>,
}

pub struct Settings {
    pub dt: f64,
    pub noise: bool,
    pub max_substeps: u32,
    pub cmd_timeout: f64,
    pub lidar_max_hz: f64,
    pub merged_hz: f64,
    pub n3d_points: usize,
}
impl Default for Settings {
    fn default() -> Self {
        let ev = |k: &str, d: f64| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
        Settings { dt: ev("SIM_DT", 0.01), noise: std::env::var("SIM_NOISE").map(|v| v == "1").unwrap_or(true), max_substeps: 10, cmd_timeout: 0.5,
            lidar_max_hz: ev("SIM_LIDAR_MAX_HZ", 10.0), merged_hz: 10.0, n3d_points: ev("SIM_LIDAR3D_POINTS", 6000.0) as usize }
    }
}

pub struct SimCore {
    pub spec: Value,
    pub settings: Settings,
    pub lib: Option<Arc<ms::MuJoCo>>,
    pub mj: Option<MjCtx>,
    pub mj_version: String,
    pub robot: RobotBody,
    pub footprint: Vec<[f64; 2]>,
    pub world: World,
    pub chassis: Chassis,
    pub wheels_d: Vec<Value>,
    pub chassis_type: String,
    pub odom: Odometry,
    pub imu: ImuState,
    pub imu_sample: ImuSample,
    pub slip_prev: [f64; 3],
    pub rng: Rng,
    pub io: IoSim,
    // scenario
    pub scenario_id: String,
    pub scenario: Value,
    pub ground_tags: Vec<Value>,
    pub station_tags: Vec<Value>,
    pub compile_ms: f64,
    // sensors
    pub lidars: Vec<Lidar2D>,
    pub lidars3d: Vec<Lidar3D>,
    pub photos: Vec<Photo>,
    pub bumpers: Vec<Bumper>,
    pub merged_bins: usize,
    pub merged_range: f64,
    pub slice_zmin: f64,
    pub slice_zmax: f64,
    pub lidars_on: bool,
    lid_next: Vec<f64>,
    pub lid_out: Vec<Frame2D>,
    lid3_next: Vec<f64>,
    pub lid3_out: Vec<Frame3D>,
    pub merged: Frame2D,
    merged_next: f64,
    // state
    pub t: f64, pub x: f64, pub y: f64, pub th: f64, pub vx: f64, pub vy: f64, pub wz: f64,
    pub cmd: [f64; 3],
    pub cmd_time: f64,
    pub paused: bool,
    pub rtf_target: f64,
    pub rtf: f64,
    pub collisions: u32,
    pub overruns: u32,
    pub steps: u32,
    discrete_n: u32,
    pub last_contact: Option<(f64, f64)>,
    pub step_ms: f64, pub lidar_ms: f64, pub mj_step_ms: f64,
    // scheduling
    acc: f64,
    last_wall: Instant,
    rtf_t0: Instant,
    rtf_sim: f64,
    mover_acc: f64,
    hk_t: f64,
    pub model_rev: u32,
    pub revision: u64,         // bumps on world/model changes (camera prims cache)
    scratch_v: Vec<f64>, scratch_g: Vec<i32>, scratch_a: Vec<f64>,
}

impl SimCore {
    pub fn new(spec: Value, scenarios: &Map<String, Value>, scenario_id: &str, chassis: Option<&str>, lib: Option<Arc<ms::MuJoCo>>, settings: Settings, model_rev: u32) -> Result<SimCore, String> {
        let mut rng = Rng::new(std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos() as u64).unwrap_or(1));
        let (chassis_k, wheels_d) = build_kinematics(&spec, chassis)?;
        let odom = Odometry::new(&chassis_k, settings.noise, &mut rng);
        let imu = ImuState::new(settings.noise, &mut rng);
        let lidars: Vec<Lidar2D> = spec.get("lidars").and_then(|l| l.as_array()).map(|a| a.iter().filter(|l| l.get("type").and_then(|t| t.as_str()) != Some("3d")).map(Lidar2D::new).collect()).unwrap_or_default();
        let lidars3d: Vec<Lidar3D> = spec.get("lidars").and_then(|l| l.as_array()).map(|a| a.iter().filter(|l| l.get("type").and_then(|t| t.as_str()) == Some("3d")).map(|l| Lidar3D::new(l, settings.n3d_points)).collect()).unwrap_or_default();
        let merged_range = lidars.iter().map(|l| l.range_max).chain(lidars3d.iter().map(|l| l.range_max)).fold(f64::NEG_INFINITY, f64::max);
        let merged_range = if merged_range.is_finite() { merged_range } else { 12.0 };
        let photos = build_photos(&spec);
        let bumpers = build_bumpers(&spec);
        let mut io = IoSim::new();
        if let Some(inp) = spec.get("io").and_then(|io| io.get("inputs")).and_then(|i| i.as_array()) {
            for d in inp { if let Some(n) = d.get("name").and_then(|n| n.as_str()) { io.setdefault_di(&format!("di_{n}"), false); } }
        }
        if let Some(out) = spec.get("io").and_then(|io| io.get("outputs")).and_then(|i| i.as_array()) {
            for d in out { if let Some(n) = d.get("name").and_then(|n| n.as_str()) { io.setdefault_do(&format!("do_{n}"), false); } }
        }
        for p in &photos { io.setdefault_di(&p.di, false); }
        for b in &bumpers { io.setdefault_di(&b.di, false); }
        let mj_version = lib.as_ref().map(|l| unsafe { std::ffi::CStr::from_ptr(l.mj_versionString() as *const _).to_string_lossy().to_string() }).unwrap_or_default();
        let now = Instant::now();
        let nl = lidars.len();
        let n3 = lidars3d.len();
        let mut c = SimCore {
            robot: RobotBody::from_spec(&spec), footprint: footprint_of(&spec), world: World::new(), chassis_type: chassis_k.kind.clone(), chassis: chassis_k, wheels_d, odom, imu,
            imu_sample: ImuSample { wz: 0.0, ax: 0.0, ay: 0.0, az: 9.81, yaw: 0.0 }, slip_prev: [0.0; 3], rng, io,
            scenario_id: String::new(), scenario: Value::Null, ground_tags: vec![], station_tags: vec![], compile_ms: 0.0,
            lidars_on: !(lidars.is_empty() && lidars3d.is_empty()),
            slice_zmin: 0.05, slice_zmax: spec["chassis"].get("height_m").and_then(|h| h.as_f64()).unwrap_or(2.0) + 0.15,
            lid_next: vec![0.0; nl], lid_out: vec![Frame2D::default(); nl], lid3_next: vec![0.0; n3], lid3_out: vec![Frame3D::default(); n3],
            merged: Frame2D::default(), merged_next: 0.0, merged_bins: 720, merged_range,
            lidars, lidars3d, photos, bumpers,
            t: 0.0, x: 0.0, y: 0.0, th: 0.0, vx: 0.0, vy: 0.0, wz: 0.0, cmd: [0.0; 3], cmd_time: -1e9, paused: false, rtf_target: 1.0, rtf: 1.0,
            collisions: 0, overruns: 0, steps: 0, discrete_n: 0, last_contact: None, step_ms: 0.0, lidar_ms: 0.0, mj_step_ms: 0.0,
            acc: 0.0, last_wall: now, rtf_t0: now, rtf_sim: 0.0, mover_acc: 0.0, hk_t: 0.0, model_rev, revision: 1,
            scratch_v: vec![], scratch_g: vec![], scratch_a: vec![],
            mj: None, lib, mj_version, spec, settings,
        };
        c.merged.ranges = Arc::new(vec![f64::INFINITY; c.merged_bins]);
        c.set_scenario(scenarios, scenario_id)?;
        Ok(c)
    }

    pub fn backend_name(&self) -> String { if self.mj.is_some() { format!("mujoco {}", self.mj_version) } else { "kinematic".into() } }
    pub fn estop(&self) -> bool { self.io.estop() }

    // ------------------------------------------------------------------ scenario / world
    pub fn set_scenario(&mut self, scenarios: &Map<String, Value>, id: &str) -> Result<(), String> {
        let sc = scenarios.get(id).ok_or_else(|| format!("unknown scenario {id}"))?.clone();
        self.scenario_id = id.to_string();
        self.world.load_scenario(&sc);
        let nodes = sc.get("nodes").and_then(|n| n.as_object()).cloned().unwrap_or_default();
        self.ground_tags = nodes.iter().map(|(k, v)| json!({"id": format!("QR_{k}"), "name": k, "x": v[0], "y": v[1], "yaw": 0.0})).collect();
        self.station_tags = sc.get("stations").and_then(|s| s.as_array()).map(|a| a.iter().enumerate().map(|(i, s)| {
            json!({"id": 100 + i, "name": s.get("name").or_else(|| s.get("id")).cloned().unwrap_or(Value::Null), "x": s["x"], "y": s["y"], "yaw": s.get("dock_yaw").cloned().unwrap_or(json!(0.0))})
        }).collect()).unwrap_or_default();
        self.scenario = sc;
        self.rebuild_mj(false)?;
        let o = self.scenario.get("origin").cloned().unwrap_or(json!({"x": 0.0, "y": 0.0, "yaw": 0.0}));
        self.reset_pose(o["x"].as_f64().unwrap_or(0.0), o["y"].as_f64().unwrap_or(0.0), o.get("yaw").and_then(|y| y.as_f64()).unwrap_or(0.0));
        Ok(())
    }

    fn decor(&self) -> (Vec<[f64; 2]>, Vec<[f64; 2]>, Vec<[f64; 4]>) {
        let st: Vec<[f64; 2]> = self.station_tags.iter().map(|s| [s["x"].as_f64().unwrap_or(0.0), s["y"].as_f64().unwrap_or(0.0)]).collect();
        let tg: Vec<[f64; 2]> = self.ground_tags.iter().map(|s| [s["x"].as_f64().unwrap_or(0.0), s["y"].as_f64().unwrap_or(0.0)]).collect();
        let nodes = self.scenario.get("nodes").and_then(|n| n.as_object());
        let lanes: Vec<[f64; 4]> = self.scenario.get("connections").and_then(|c| c.as_array()).map(|cs| cs.iter().filter_map(|c| {
            let (a, b) = (c[0].as_str()?, c[1].as_str()?);
            let (na, nb) = (nodes?.get(a)?, nodes?.get(b)?);
            Some([na[0].as_f64()?, na[1].as_f64()?, nb[0].as_f64()?, nb[1].as_f64()?])
        }).collect()).unwrap_or_default();
        (st, tg, lanes)
    }

    pub fn rebuild_mj(&mut self, keep_state: bool) -> Result<(), String> {
        let Some(lib) = self.lib.clone() else { return Ok(()); };
        let (st, tg, ln) = self.decor();
        let xml = build_mjcf(&self.world, &self.robot, self.settings.dt, &Decor { stations: &st, tags: &tg, lanes: &ln });
        let t0 = Instant::now();
        let mut ctx = MjCtx::load(lib, &xml, self.world.obstacles.len())?;
        if keep_state {
            if let Some(old) = &self.mj {
                if old.nq() == ctx.nq() {
                    for i in 0..ctx.nq() { ctx.set_qpos(i, old.qpos(i)); ctx.set_qvel(i, old.qvel(i)); }
                }
            }
            ctx.forward();
        }
        self.compile_ms = t0.elapsed().as_secs_f64() * 1000.0;
        self.mj = Some(ctx);
        self.revision += 1;
        Ok(())
    }

    pub fn set_obstacles(&mut self, obs: Vec<Map<String, Value>>) -> Result<(), String> {
        let t = self.t;
        let mut obs = obs;
        for o in obs.iter_mut() {
            let (x, y) = (o.get("x").cloned().unwrap_or(json!(0.0)), o.get("y").cloned().unwrap_or(json!(0.0)));
            if let Some(m) = o.get_mut("motion").and_then(|m| m.as_object_mut()) {
                m.entry("t0".to_string()).or_insert(json!(t));
                m.entry("ax".to_string()).or_insert(x);
                m.entry("ay".to_string()).or_insert(y);
            }
        }
        self.world.set_obstacles(obs);
        self.rebuild_mj(true)
    }

    /// patrol obstacles: A<->B back and forth
    pub fn update_movers(&mut self) {
        let mut poses = vec![];
        let t = self.t;
        for (i, o) in self.world.obstacles.iter_mut().enumerate() {
            let Some(m) = o.get("motion").and_then(|m| m.as_object()).cloned() else { continue };
            if m.get("type").and_then(|t| t.as_str()).unwrap_or("patrol") != "patrol" { continue; }
            let g = |k: &str, d: f64| m.get(k).and_then(|v| v.as_f64()).unwrap_or(d);
            let (ax, ay) = (g("ax", 0.0), g("ay", 0.0));
            let (bx, by) = (g("bx", ax), g("by", ay));
            let l = (bx - ax).hypot(by - ay);
            if l < 1e-3 { continue; }
            let s = crate::geom::pymod((t - g("t0", 0.0)) * g("speed", 0.8), 2.0 * l);
            let k = if s <= l { s / l } else { 2.0 - s / l };
            let (x, y) = (ax + (bx - ax) * k, ay + (by - ay) * k);
            let yaw = (by - ay).atan2(bx - ax) + if s <= l { 0.0 } else { PI };
            let r3 = |v: f64| (v * 1000.0).round() / 1000.0;
            o.insert("x".into(), json!(r3(x))); o.insert("y".into(), json!(r3(y))); o.insert("yaw".into(), json!(r3(yaw)));
            poses.push((i, x, y, yaw));
        }
        if !poses.is_empty() {
            self.world.rebuild_dynamic();
            self.revision += 1;
            if let Some(mj) = self.mj.as_mut() { for (i, x, y, yaw) in poses { mj.move_obstacle(i, x, y, yaw); } }
        }
    }

    pub fn reset_pose(&mut self, x: f64, y: f64, th: f64) {
        self.x = x; self.y = y; self.th = th;
        for b in self.bumpers.iter_mut() { b.pressed = false; }
        let dis: Vec<String> = self.bumpers.iter().map(|b| b.di.clone()).collect();
        for d in dis { self.io.set_di(&d, false); }
        self.vx = 0.0; self.vy = 0.0; self.wz = 0.0;
        self.chassis.reset();
        self.odom.reset(x, y, th);
        self.cmd = [0.0; 3];
        if let Some(mj) = self.mj.as_mut() { mj.reset_pose(x, y, th); }
    }

    pub fn set_chassis(&mut self, ct: Option<&str>) -> Result<String, String> {
        let keep = (self.x, self.y, self.th);
        let (c, w) = build_kinematics(&self.spec, ct)?;
        self.chassis = c;
        self.wheels_d = w;
        self.chassis_type = self.chassis.kind.clone();
        self.odom = Odometry::new(&self.chassis, self.settings.noise, &mut self.rng);
        self.odom.reset(keep.0, keep.1, keep.2);
        Ok(self.chassis_type.clone())
    }

    pub fn set_cmd(&mut self, vx: f64, vy: f64, wz: f64) { self.cmd = [vx, vy, wz]; self.cmd_time = self.t; }

    pub fn set_lidar_config(&mut self, beams: Option<i64>, res_deg: Option<f64>, range_max: Option<f64>, freq_hz: Option<f64>) {
        if let Some(b) = beams { if b != 0 { self.merged_bins = (b.max(90).min(3600)) as usize; self.merged.ranges = Arc::new(vec![f64::INFINITY; self.merged_bins]); self.lid3_out.iter_mut().for_each(|f| f.slice = Arc::new(vec![]));} }
        if let Some(r) = res_deg { if r != 0.0 { for l in self.lidars.iter_mut() { l.set_resolution(r.to_radians()); } } }
        if let Some(r) = range_max { if r != 0.0 {
            for l in self.lidars.iter_mut() { l.range_max = if r < l.native_range_max { l.native_range_max.min(r) } else { r }; }
            self.merged_range = self.lidars.iter().map(|l| l.range_max).fold(f64::NEG_INFINITY, f64::max);
        } }
        if let Some(f) = freq_hz { if f != 0.0 { for l in self.lidars.iter_mut() { l.freq_hz = f; } } }
        // resolution/beam changes invalidate buffers
        self.lid_out = self.lidars.iter().map(|_| Frame2D::default()).collect();
        self.lid_next = vec![0.0; self.lidars.len()];
        self.merged_next = 0.0;
    }

    // ------------------------------------------------------------------ stepping
    fn bumper_apply(&mut self, i: usize, hit: bool, forced: bool) {
        let t = self.t;
        let b = &mut self.bumpers[i];
        if hit || forced {
            if !b.pressed { b.press_count += 1; }
            b.pressed = true;
            b.contact_t = t;
        } else if b.pressed && t - b.contact_t > b.hold_s {
            b.pressed = false;
        }
    }
    fn any_pressed(&self) -> bool { self.bumpers.iter().any(|b| b.pressed) }

    fn on_collision(&mut self, px: f64, py: f64) {
        if !self.any_pressed() { self.collisions += 1; }
        let (s, c) = self.th.sin_cos();
        let bx = c * (px - self.x) + s * (py - self.y);
        let ch = &self.spec["chassis"];
        let (head, tail) = (ch.get("head_offset_m").and_then(|v| v.as_f64()).unwrap_or(0.5), ch.get("tail_offset_m").and_then(|v| v.as_f64()).unwrap_or(0.5));
        let front = bx >= (head - tail) / 2.0;
        self.last_contact = Some((px, py));
        let side = if front { 0 } else { 1 };
        for i in 0..self.bumpers.len().min(MAXB) { if self.bumpers[i].side == side { self.bumper_apply(i, true, true); } }
    }

    fn photos_update(&mut self) {
        let (c, sn) = (self.th.cos(), self.th.sin());
        let n = self.photos.len().min(MAXP);
        for i in 0..n {
            let p = &self.photos[i];
            let (ox, oy) = (self.x + c * p.x - sn * p.y, self.y + sn * p.x + c * p.y);
            let a = self.th + p.yaw;
            let ang = [a - p.half, a, a + p.half];
            let mut d = [0.0f64; 3];
            let (z, range) = (p.z, p.range_m);
            if let Some(mj) = self.mj.as_mut() {
                mj.rays2d(ox, oy, z.max(0.005), &ang, range, &mut d, &mut self.scratch_v, &mut self.scratch_g);
            } else {
                raycast2d(&self.world.segments, &[], z, ox, oy, &ang, range, &mut d);
            }
            let dist = d[0].min(d[1]).min(d[2]);
            let p = &mut self.photos[i];
            p.distance = dist;
            let thr = p.trigger_m + if p.detected { p.hyst } else { 0.0 };
            p.detected = dist <= thr;
        }
    }

    fn discrete_update(&mut self) {
        for i in 0..self.bumpers.len().min(MAXB) {
            let was = self.bumpers[i].pressed;
            let hit = !self.world.segments.is_empty() && collides(&self.bumpers[i].poly, self.x, self.y, self.th, &self.world.segments).is_some();
            self.bumper_apply(i, hit, false);
            if self.bumpers[i].pressed && !was {
                self.collisions += 1;
                self.last_contact = Some((self.x, self.y));
            }
        }
        self.discrete_n += 1;
        if self.discrete_n % 2 == 0 { self.photos_update(); }
    }

    pub fn step(&mut self) {
        let dt = self.settings.dt;
        let t0 = Instant::now();
        let watchdog = (self.t - self.cmd_time) > self.settings.cmd_timeout;
        let brake = self.estop();
        let (mut cvx, mut cvy, mut cwz) = if watchdog || brake { (0.0, 0.0, 0.0) } else { (self.cmd[0], self.cmd[1], self.cmd[2]) };
        for b in self.bumpers.iter().take(MAXB) {
            if !b.pressed { continue; }
            match b.side { 0 if cvx > 0.0 => cvx = 0.0, 1 if cvx < 0.0 => cvx = 0.0, 2 if cvy > 0.0 => cvy = 0.0, 3 if cvy < 0.0 => cvy = 0.0, _ => {} }
        }
        if self.any_pressed() { cwz = 0.0; }
        self.chassis.step(cvx, cvy, cwz, dt, brake);
        let mut v = [self.chassis.vx, self.chassis.vy, self.chassis.wz];
        if self.settings.noise { slip(&mut self.slip_prev, &mut v, dt, &mut self.rng); }
        if self.mj.is_some() {
            let tm = Instant::now();
            let mj = self.mj.as_mut().unwrap();
            let th = mj.qpos(2);
            let (c, sn) = (th.cos(), th.sin());
            mj.set_ctrl(0, c * v[0] - sn * v[1]);
            mj.set_ctrl(1, sn * v[0] + c * v[1]);
            mj.set_ctrl(2, v[2]);
            mj.step();
            self.mj_step_ms = 0.9 * self.mj_step_ms + 0.1 * tm.elapsed().as_secs_f64() * 1000.0;
            let (x, y, th) = (mj.qpos(0), mj.qpos(1), mj.qpos(2));
            let (vxw, vyw, w) = (mj.qvel(0), mj.qvel(1), mj.qvel(2));
            let contact = mj.robot_contact();
            let (c, sn) = (th.cos(), th.sin());
            if let Some((hx, hy)) = contact { self.on_collision(hx, hy); }
            self.x = x; self.y = y; self.th = th;
            self.vx = c * vxw + sn * vyw; self.vy = -sn * vxw + c * vyw; self.wz = w;
        } else {
            let (nx, ny, nth) = se2_integrate(self.x, self.y, self.th, v[0], v[1], v[2], dt);
            let hit = if !self.world.segments.is_empty() && !self.footprint.is_empty() { collides(&self.footprint, nx, ny, nth, &self.world.segments) } else { None };
            if let Some(pt) = hit {
                self.on_collision(pt[0], pt[1]);
                self.chassis.stop_now();
                v = [0.0; 3];
            } else { self.x = nx; self.y = ny; self.th = nth; }
            self.vx = v[0]; self.vy = v[1]; self.wz = v[2];
        }
        self.discrete_update();
        if self.settings.noise { let ch = &self.chassis; self.odom.update_noisy(ch, dt, &mut self.rng); } else { let ch = &self.chassis; self.odom.update_ideal(ch, dt); }
        self.imu_sample = self.imu.sample(self.vx, self.vy, self.wz, dt, &mut self.rng);
        self.t += dt;
        self.steps += 1;
        self.step_ms = 0.9 * self.step_ms + 0.1 * t0.elapsed().as_secs_f64() * 1000.0;
    }

    // ------------------------------------------------------------------ lidars
    fn scan_one(&mut self, i: usize) -> Vec<f64> {
        let l = &self.lidars[i];
        let (cth, sth) = (self.th.cos(), self.th.sin());
        let (ox, oy) = (self.x + cth * l.mx - sth * l.my, self.y + sth * l.mx + cth * l.my);
        let base = self.th + l.yaw;
        let mut ang = std::mem::take(&mut self.scratch_a);
        ang.clear();
        ang.extend((0..l.n).map(|k| base + l.sign * (l.angle_min + k as f64 * l.angle_inc)));
        let mut out = vec![0.0f64; l.n];
        let (mz, rmax, rmin, std, prop, dropout) = (l.mz, l.range_max, l.range_min, l.noise_std, l.noise_prop, l.dropout);
        if let Some(mj) = self.mj.as_mut() { mj.rays2d(ox, oy, mz.max(0.005), &ang, rmax, &mut out, &mut self.scratch_v, &mut self.scratch_g); }
        else { raycast2d(&self.world.segments, &[], mz, ox, oy, &ang, rmax, &mut out); }
        self.scratch_a = ang;
        lidar_post(&mut out, rmin, self.settings.noise, std, prop, dropout, &mut self.rng);
        out
    }

    fn scan3d_one(&mut self, i: usize) {
        let noise = self.settings.noise;
        let l = self.lidars3d[i].clone();
        let n = l.n;
        const PHI1: f64 = 0.6180339887498949;
        const PHI2: f64 = 0.7548776662466927;
        let (cth, sth) = (self.th.cos(), self.th.sin());
        let mut rw = [0.0f64; 9];
        for k in 0..3 {
            rw[k] = cth * l.r[k] - sth * l.r[3 + k];
            rw[3 + k] = sth * l.r[k] + cth * l.r[3 + k];
            rw[6 + k] = l.r[6 + k];
        }
        let (s0, s1) = (l.vmin.sin(), l.vmax.sin());
        let mut dw = vec![0.0f64; 3 * n];
        let mut ds = vec![0.0f64; 3 * n];
        for k in 0..n {
            let ii = k as f64 + self.lidars3d[i].frame * n as f64;
            let mut az = 2.0 * PI * ((ii * PHI1) % 1.0);
            let mut el = (s0 + (s1 - s0) * ((ii * PHI2) % 1.0)).asin();
            if noise && l.ang_noise > 0.0 { az += self.rng.gauss() * l.ang_noise; el += self.rng.gauss() * l.ang_noise; }
            let (ce, x, y, z) = (el.cos(), el.cos() * az.cos(), el.cos() * az.sin(), el.sin());
            let _ = ce;
            ds[3 * k] = x; ds[3 * k + 1] = y; ds[3 * k + 2] = z;
            dw[3 * k] = rw[0] * x + rw[1] * y + rw[2] * z;
            dw[3 * k + 1] = rw[3] * x + rw[4] * y + rw[5] * z;
            dw[3 * k + 2] = rw[6] * x + rw[7] * y + rw[8] * z;
        }
        self.lidars3d[i].frame += 1.0;
        let o = [self.x + cth * l.mx - sth * l.my, self.y + sth * l.mx + cth * l.my, l.mz];
        let mut dist = vec![f64::INFINITY; n];
        if let Some(mj) = self.mj.as_mut() { mj.rays3d(o, &dw, l.range_max, &mut dist, &mut self.scratch_g); }
        let nb = self.merged_bins;
        let binw = 2.0 * PI / nb.max(1) as f64;
        let period = 1.0 / l.freq_hz.max(1.0);
        let dt_pt = 1.0 / (1.0 / period).max(1.0) / n as f64;
        let mut sl = vec![f64::INFINITY; nb];
        let (mut xyz, mut inten, mut line, mut ot) = (Vec::with_capacity(3 * n), Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n));
        for k in 0..n {
            let mut r = dist[k];
            let dz = dw[3 * k + 2];
            let tf = if dz < -1e-9 { -o[2] / dz } else { f64::INFINITY };
            let tc = if dz > 1e-9 { (CEILING_HEIGHT - o[2]) / dz } else { f64::INFINITY };
            if tf < r { r = tf; }
            if tc < r { r = tc; }
            if r > l.range_max || r < l.range_min || !r.is_finite() { continue; }
            if noise { r += self.rng.gauss() * l.noise_std; }
            let (px, py, pz) = (ds[3 * k] * r, ds[3 * k + 1] * r, ds[3 * k + 2] * r);
            let (fx, fy, fz) = (px as f32, py as f32, pz as f32);
            xyz.extend_from_slice(&[fx, fy, fz]);
            line.push((k as i32 % l.lines.max(1)) as u8);
            let it = 180.0 - 3.0 * r + if noise { self.rng.gauss() * 8.0 } else { 0.0 };
            inten.push(it.max(5.0).min(255.0) as f32);
            ot.push(k as f64 * dt_pt);
            let (fx, fy, fz) = (fx as f64, fy as f64, fz as f64);
            let bx = l.r[0] * fx + l.r[1] * fy + l.r[2] * fz + l.mx;
            let by = l.r[3] * fx + l.r[4] * fy + l.r[5] * fz + l.my;
            let bz = l.r[6] * fx + l.r[7] * fy + l.r[8] * fz + l.mz;
            if nb == 0 || bz < self.slice_zmin || bz > self.slice_zmax { continue; }
            let mut kk = ((by.atan2(bx) + PI) / binw) as i64;
            kk = kk.max(0).min(nb as i64 - 1);
            let d = bx.hypot(by);
            if d < sl[kk as usize] { sl[kk as usize] = d; }
        }
        for v in sl.iter_mut() { if *v > self.merged_range { *v = f64::INFINITY; } }
        let f = &mut self.lid3_out[i];
        f.seq += 1; f.t = self.t; f.pose = [self.x, self.y, self.th];
        f.xyz = Arc::new(xyz); f.intensity = Arc::new(inten); f.line = Arc::new(line); f.offset_time = Arc::new(ot); f.slice = Arc::new(sl);
    }

    fn lidars_update(&mut self) {
        if !self.lidars_on || (self.lidars.is_empty() && self.lidars3d.is_empty()) { return; }
        let t = self.t;
        let any = (0..self.lidars.len()).any(|i| t >= self.lid_next[i]) || (0..self.lidars3d.len()).any(|i| t >= self.lid3_next[i]);
        let mdue = t >= self.merged_next;
        if !any && !mdue { return; }
        let t0 = Instant::now();
        let nbins = self.merged_bins;
        let mut mb = vec![f64::INFINITY; nbins];
        for i in 0..self.lidars.len() {
            let b = self.scan_one(i);
            let l = &self.lidars[i];
            merge_add(&mut mb, &b, l.mx, l.my, l.yaw, l.sign, l.angle_min, l.angle_inc);
            if t >= self.lid_next[i] {
                let hz = if self.settings.lidar_max_hz > 0.0 { l.freq_hz.min(self.settings.lidar_max_hz) } else { l.freq_hz };
                self.lid_next[i] = t + 1.0 / hz.max(1.0);
                let f = &mut self.lid_out[i];
                f.seq += 1; f.t = t; f.pose = [self.x, self.y, self.th]; f.ranges = Arc::new(b);
            }
        }
        for i in 0..self.lidars3d.len() {
            if t >= self.lid3_next[i] {
                self.scan3d_one(i);
                self.lid3_next[i] = t + 1.0 / self.lidars3d[i].freq_hz.max(1.0);
            }
            if self.lid3_out[i].seq > 0 {
                let sl = &self.lid3_out[i].slice;
                for k in 0..nbins.min(sl.len()) { if sl[k] < mb[k] { mb[k] = sl[k]; } }
            }
        }
        if mdue {
            merge_finish(&mut mb, self.merged_range);
            self.merged_next = t + 1.0 / self.settings.merged_hz.max(0.1);
            self.merged.seq += 1; self.merged.t = t; self.merged.pose = [self.x, self.y, self.th]; self.merged.ranges = Arc::new(mb);
        }
        self.lidar_ms = 0.8 * self.lidar_ms + 0.2 * t0.elapsed().as_secs_f64() * 1000.0;
    }

    /// housekeeping after steps: DI sync from sensors, lift physics, movers
    fn housekeeping(&mut self) {
        let sync: Vec<(String, bool)> = self.bumpers.iter().map(|b| (b.di.clone(), b.pressed)).chain(self.photos.iter().map(|p| (p.di.clone(), p.detected))).collect();
        for (di, v) in sync { if self.io.di(&di) != v { self.io.set_di(&di, v); } }
        if self.t > self.hk_t {
            let d = self.t - self.hk_t;
            self.io.update_lift(d);
            self.mover_acc += d;
            if self.mover_acc >= 0.05 {
                self.mover_acc = 0.0;
                if self.world.obstacles.iter().any(|o| o.get("motion").map(|m| !m.is_null()).unwrap_or(false)) { self.update_movers(); }
            }
        }
        self.hk_t = self.t;
    }

    /// advance by the elapsed wall time (called by the real-time thread). Returns steps executed.
    pub fn advance(&mut self) -> u32 {
        let dt = self.settings.dt;
        let now = Instant::now();
        let real = now.duration_since(self.last_wall).as_secs_f64().min(0.25).max(0.0);
        self.last_wall = now;
        let mut n = 0;
        self.acc += real * self.rtf_target;
        while self.acc >= dt && n < self.settings.max_substeps {
            if !self.paused { self.step(); }
            self.acc -= dt;
            n += 1;
        }
        if self.acc >= dt { self.overruns += 1; self.acc = 0.0; }
        if n > 0 && !self.paused { self.lidars_update(); }
        self.housekeeping();
        self.rtf_sim += if self.paused { 0.0 } else { n as f64 * dt };
        let el = now.duration_since(self.rtf_t0).as_secs_f64();
        if el >= 1.0 { self.rtf = self.rtf_sim / el; self.rtf_t0 = now; self.rtf_sim = 0.0; }
        n
    }

    /// test/offline: advance n fixed steps (lidars scanned when due) without wall-clock scheduling
    pub fn step_n(&mut self, n: u32) {
        for _ in 0..n { if !self.paused { self.step(); self.lidars_update(); } self.housekeeping(); }
    }
    /// force a scan of all 2D/3D lidars now (publishes merged)
    pub fn scan_now(&mut self) {
        self.lid_next.iter_mut().for_each(|v| *v = -1.0);
        self.lid3_next.iter_mut().for_each(|v| *v = -1.0);
        self.merged_next = -1.0;
        self.lidars_update();
    }

    pub fn map_to_odom(&self) -> (f64, f64, f64) {
        let (ox, oy, oth) = (self.odom.x, self.odom.y, self.odom.th);
        let dth = wrap(self.th - oth);
        let (s, c) = dth.sin_cos();
        (self.x - (c * ox - s * oy), self.y - (s * ox + c * oy), dth)
    }

    pub fn detect_codes(&mut self) -> Vec<Value> {
        let cams: Vec<Value> = self.spec.get("cameras").and_then(|c| c.as_array()).cloned().unwrap_or_default();
        let mut out = detect_ground_codes(&cams, self.x, self.y, self.th, &self.ground_tags, &mut self.rng);
        let mount = self.spec["chassis"].get("head_offset_m").and_then(|v| v.as_f64()).unwrap_or(0.5) * 0.8;
        out.extend(detect_station_tags(self.x, self.y, self.th, mount, &self.station_tags));
        out
    }

    pub fn joint_state(&self) -> (Vec<String>, Vec<f64>, Vec<f64>, Vec<f64>) { self.chassis.joint_state() }
}
