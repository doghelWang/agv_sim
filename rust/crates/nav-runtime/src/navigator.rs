//! Navigator: state, safety layer, planning dispatch, mission records (port of nav_runtime/navigator.py, pure-guidance mode).
use crate::events::{now, EventHub};
use crate::simlink::{Scan, SimLink, State as SimState};
use planning::astar::AStar;
use planning::maneuver::{self, Corner, CornerMode};
use planning::protection::{self, Prot};
use planning::route::{Obs, Planner};
use planning::scenario::Library;
use planning::sweep;
use planning::{r2, wrap, Pt, Seg};
use serde_json::{json, Value};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::Duration;

pub const PLANNERS: [&str; 5] = ["dijkstra", "astar", "direct", "straight", "nav2"];

#[derive(Clone, Debug, Default)]
pub struct Tele {
    pub x: f64, pub y: f64, pub yaw: f64, pub vx: f64, pub vy: f64, pub wz: f64,
    pub truth: [f64; 3],
    pub nav_status: String,
    pub target_goal: Option<[f64; 3]>,
    pub plan_path: Vec<Pt>,
    pub path_labels: Vec<Option<String>>,
    pub path_index: usize,
    pub path_reverse: Vec<bool>,
    pub plan_curve: Vec<Pt>,
    pub nav_dist_rem: f64,
}

#[derive(Clone, Debug)]
pub struct Mission { pub id: u64, pub planner: String, pub goal: [f64; 3], pub status: String, pub created: f64, pub ended: Option<f64>, pub title: Option<String> }

#[derive(Clone, Debug, Default)]
pub struct ObsState {
    pub front: Option<f64>, pub rear: Option<f64>, pub zone: String, pub layer: Option<String>,
    pub bands: Vec<(Option<f64>, Option<f64>)>, pub band: usize, pub photo_ignored: Vec<String>,
    pub t_warn: f64, pub t_clear: f64,
}

pub struct St {
    pub tele: Tele,
    pub prot: Prot,
    pub body_v: Value,
    pub loaded: bool,
    pub head_offset: f64, pub tail_offset: f64, pub left_off: f64, pub right_off: f64, pub half_width: f64,
    pub max_decel: f64,
    pub chassis_cfg: Value,
    pub active_planner: String,
    pub active_scenario: String,
    pub chassis_type: String,
    pub paused: bool,
    pub dyn_obs: Vec<Value>,
    pub mission: Option<Mission>,
    pub missions: VecDeque<Mission>,
    pub cur_mid: u64,
    pub obs: ObsState,
    pub speed_cap: Option<f64>,
    pub approach_left: Option<f64>,
    pub in_arc: bool,
    pub steer: Vec<f64>,
    pub rot_left: f64,
    pub pts: Option<(Vec<f64>, Vec<f64>)>,
    pub scan_body: Option<(Vec<f64>, f64, f64, f64)>,
    pub track_l: f64,
    pub mission_corners: (u64, Vec<Option<(Corner, f64)>>),
    pub estop_resume: Option<[f64; 3]>,
    pub runtime_prot: bool,
    pub zone_info: (Option<String>, Option<f64>, Option<f64>),
    pub astar: AStar,
    pub walls: Vec<Seg>,
    pub lib: Library,
    pub last_state_t: f64,
}

pub struct Navigator {
    pub link: Arc<SimLink>,
    pub events: EventHub,
    pub st: Mutex<St>,
    pub planner: Mutex<Planner>,
    pub running: AtomicBool,
    pub taskflow: OnceLock<Arc<crate::taskflow::TaskFlow>>,
    pub nav2: OnceLock<Arc<crate::nav2::Nav2>>,
}

pub fn sleep(ms: u64) { std::thread::sleep(Duration::from_millis(ms)); }
pub fn pt_json(p: &Pt) -> Value { json!({"x": p.0, "y": p.1}) }
fn r3(x: f64) -> f64 { (x * 1000.0).round_ties_even() / 1000.0 }
fn g(v: &Value, k: &str, d: f64) -> f64 { v.get(k).and_then(|x| x.as_f64()).unwrap_or(d) }

impl Navigator {
    pub fn new(link: Arc<SimLink>, scenarios: &serde_json::Map<String, Value>) -> Arc<Navigator> {
        let lib = Library::from_json_map(scenarios);
        let planner = Planner::new(&lib, "grid_9_square");
        let prot = protection::effective(&json!({}), None);
        let walls = planner.walls();
        let st = St {
            tele: Tele { nav_status: "IDLE".into(), ..Default::default() }, prot, body_v: json!({"head_offset_m": 0.6, "tail_offset_m": 0.6, "left_offset_m": 0.4, "right_offset_m": 0.4}),
            loaded: false, head_offset: 0.6, tail_offset: 0.6, left_off: 0.4, right_off: 0.4, half_width: 0.4, max_decel: 0.5, chassis_cfg: json!({}),
            active_planner: "dijkstra".into(), active_scenario: "grid_9_square".into(), chassis_type: "single_steer".into(), paused: false, dyn_obs: vec![],
            mission: None, missions: VecDeque::new(), cur_mid: 0,
            obs: ObsState { zone: "clear".into(), ..Default::default() }, speed_cap: None, approach_left: None, in_arc: false, steer: vec![], rot_left: -1.0,
            pts: None, scan_body: None, track_l: 0.6, mission_corners: (0, vec![]), estop_resume: None, runtime_prot: false, zone_info: (None, None, None),
            astar: AStar::new(0.1, 0.5), walls, lib, last_state_t: 0.0,
        };
        let nav = Arc::new(Navigator { link: link.clone(), events: EventHub::new(1000), st: Mutex::new(st), planner: Mutex::new(planner), running: AtomicBool::new(true), taskflow: OnceLock::new(), nav2: OnceLock::new() });
        let _ = nav.nav2.set(crate::nav2::Nav2::new(Arc::downgrade(&nav)));
        link.set_sink(nav.clone());
        let n2 = nav.clone();
        std::thread::Builder::new().name("nav-safety".into()).spawn(move || n2.safety_loop()).unwrap();
        nav
    }

    pub fn s(&self) -> MutexGuard<'_, St> { self.st.lock().unwrap_or_else(|e| e.into_inner()) }
    pub fn pl(&self) -> MutexGuard<'_, Planner> { self.planner.lock().unwrap_or_else(|e| e.into_inner()) }
    pub fn ev(&self, cat: &str, typ: &str, level: &str, title: &str, msg: &str, payload: Value) { self.events.emit(cat, typ, level, title, msg, payload); }
    pub fn log(&self, m: &str) { println!("{m}"); }
    pub fn is_running(&self) -> bool { self.running.load(Ordering::Relaxed) }

    pub fn pose(&self) -> (f64, f64, f64) { let s = self.s(); (s.tele.x, s.tele.y, s.tele.yaw) }
    pub fn mid_ok(&self, mid: u64) -> bool { self.s().cur_mid == mid }
    pub fn status(&self) -> String { self.s().tele.nav_status.clone() }
    pub fn set_status(&self, st: &str) { self.s().tele.nav_status = st.to_string(); }
    pub fn outline_of(s: &St) -> [f64; 4] { s.prot.outline(&protection::body_box(&s.body_v), s.loaded) }
    pub fn outline(&self) -> [f64; 4] { Self::outline_of(&self.s()) }

    // ------------------------------------------------------------------ data input
    fn apply_outline(&self) {
        let (h, t, l, r, bm, mode, cr, side0) = {
            let mut s = self.s();
            let [h, t, l, r] = Self::outline_of(&s);
            s.head_offset = h; s.tail_offset = t; s.left_off = l; s.right_off = r; s.half_width = l.max(r);
            let cr = Self::corner_radius_of(&s);
            let side0 = s.prot.fields.first().map(|f| f.side).unwrap_or(0.1);
            s.astar = AStar::new(0.1, s.half_width + side0 + s.prot.body_margin);
            (h, t, l, r, s.prot.body_margin, CornerMode::parse(&s.prot.corner_mode), cr, side0)
        };
        let _ = side0;
        let hw = l.max(r);
        let mut p = self.pl();
        p.robot_half_width = hw;
        p.set_footprint(h, t, hw, cr, bm, mode);
        p.robot_circum_radius = h.hypot(hw).max(t.hypot(hw));
    }

    pub fn corner_radius_of(s: &St) -> f64 {
        if let Some(c) = s.prot.json.get("corner_radius").and_then(|v| v.as_f64()) { if c != 0.0 { return c; } }
        (0.6 * s.head_offset + 0.3).min(1.4).max(0.6)
    }
    pub fn corner_radius(&self) -> f64 { Self::corner_radius_of(&self.s()) }

    pub fn sweep_radius(&self) -> f64 {
        let s = self.s();
        s.head_offset.max(s.tail_offset).hypot(s.half_width) + s.prot.rotate_margin
    }

    // ------------------------------------------------------------------ protection
    pub fn set_protection(&self, patch: &Value) -> Value {
        {
            let mut s = self.s();
            let mut base = s.prot.json.clone();
            let has_fields = patch.get("fields").and_then(|f| f.as_array()).map(|a| !a.is_empty()).unwrap_or(false);
            if has_fields { if let Some(o) = base.as_object_mut() { o.remove("fields_auto"); } }
            let mut merged = protection::merge(&base, patch);
            if has_fields { merged["fields"] = patch["fields"].clone(); }
            let body = s.body_v.clone();
            s.prot = protection::effective(&merged, Some(&body));
            s.runtime_prot = true;
        }
        self.apply_outline();
        self.protection_view()
    }

    pub fn protection_view(&self) -> Value {
        let s = self.s();
        Self::protection_view_of(&s)
    }
    fn protection_view_of(s: &St) -> Value {
        let i = s.obs.band;
        let band_name = s.prot.fields.get(i).map(|f| Value::String(f.name.clone())).unwrap_or(Value::Null);
        let o = Self::outline_of(s);
        let body = protection::body_box(&s.body_v);
        json!({"config": s.prot.json, "loaded": s.loaded, "outline": o, "band": i, "band_name": band_name,
               "polygons": protection::polygons(&body, &s.prot, s.loaded, i), "zone": s.obs.zone, "layer": s.obs.layer,
               "front": s.obs.front, "rear": s.obs.rear, "runtime_modified": s.runtime_prot, "reverse": s.tele.vx < -0.02})
    }

    pub fn safety_params(&self) -> Value { Self::safety_params_of(&self.s()) }
    fn safety_params_of(s: &St) -> Value {
        let p = &s.prot;
        let i = if p.fields.is_empty() { 0 } else { s.obs.band.min(p.fields.len() - 1) };
        let f = &p.fields[i];
        json!({"enabled": p.enabled, "stop_dist": f.front, "slow_dist": r2(f.front * p.slow_ratio), "slow_speed": p.slow_speed, "corridor_margin": f.side,
               "corner_radius": p.json.get("corner_radius").cloned().unwrap_or(Value::Null), "protection": p.json})
    }

    /// (allowed speed, d_hit, need_hit)
    pub fn allowed_speed(&self, sign: i32, left: Option<f64>) -> (f64, Option<f64>, Option<f64>) {
        let s = self.s();
        Self::allowed_speed_of(&s, sign, left)
    }
    fn allowed_speed_of(s: &St, sign: i32, left: Option<f64>) -> (f64, Option<f64>, Option<f64>) {
        let fs = &s.prot.fields;
        let mut allowed: Option<f64> = None;
        let (mut d_hit, mut need_hit) = (None, None);
        for (i, f) in fs.iter().enumerate() {
            let d = s.obs.bands.get(i).and_then(|b| if sign > 0 { b.0 } else { b.1 });
            let mut need = if sign > 0 { f.front } else { f.rear };
            if let Some(l) = left { need = need.min(l + s.prot.docking_front); }
            if let Some(dd) = d { if dd < need { d_hit = Some(dd); need_hit = Some(need); break; } }
            allowed = Some(if i < fs.len() - 1 { f.v_max } else { 99.0 });
        }
        (allowed.unwrap_or(0.0), d_hit, need_hit)
    }

    fn rotation_blocked_of(s: &St, direction: f64) -> bool {
        let Some((px, py)) = &s.pts else { return false };
        if px.is_empty() { return false; }
        let [h, t, l, r] = Self::outline_of(s);
        let m = s.prot.rotate_margin;
        let bx = [-t - m, h + m, -r - m, l + m];
        let mut rest: Vec<(f64, f64)> = vec![];
        for i in 0..px.len() {
            let inside = px[i] < h + m && px[i] > -t - m && py[i] < l + m && py[i] > -r - m;
            if !inside { rest.push((px[i], py[i])); }
        }
        if rest.is_empty() { return false; } // all points inside
        let ad = g(&s.chassis_cfg, "max_ang_decel_radps2", 1.0);
        let ad = if ad == 0.0 { 1.0 } else { ad };
        let look = s.prot.rotate_lookahead(s.tele.wz, ad, s.rot_left);
        sweep::arc_hits_any(&rest, -direction * look, &bx)
    }

    // ------------------------------------------------------------------ output
    pub fn publish_cmd_vel(&self, vx: f64, vy: f64, wz: f64) {
        let (vx, vy, wz) = self.safety_filter(vx, vy, wz);
        let planner = self.s().active_planner.clone();
        self.link.send_cmd(vx, vy, wz, &format!("nav:{planner}"));
    }

    // ------------------------------------------------------------------ safety layer
    fn photo_sides_of(&self, s: &mut St, skip_diag: bool, apply_envelope: bool) -> [Vec<String>; 4] {
        // order: front, rear, left, right
        let io = self.link.io();
        let di = &io["inputs"];
        let photos = self.link.photos.lock().unwrap().clone();
        let mut out: [Vec<String>; 4] = Default::default();
        let body = protection::body_box(&s.body_v);
        let env = if apply_envelope { s.prot.photo_envelope(&body, s.loaded, s.obs.band) } else { None };
        let mute = s.approach_left.map(|l| l < s.prot.photo_mute_near_stop).unwrap_or(false);
        let mut ignored: Vec<String> = vec![];
        let sensors = self.link.sensors();
        for p in sensors["photoelectric"].as_array().cloned().unwrap_or_default() {
            let dik = p["di"].as_str().unwrap_or("");
            if !di[dik].as_bool().unwrap_or(false) { continue; }
            let name = p["name"].as_str().unwrap_or("").to_string();
            let yaw = g(&p["mount"], "yaw", 0.0);
            if skip_diag { let a = wrap(yaw).abs(); if a > 0.3 && a < 2.8 { continue; } }
            let (c, sn) = (yaw.cos(), yaw.sin());
            let side = if c > 0.3 { 0 } else if c < -0.3 { 1 } else if sn > 0.0 { 2 } else { 3 };
            if side == 0 && mute { ignored.push(name); continue; }
            if let Some(e) = env {
                let d = photos.get(&name).and_then(|v| v["distance_m"].as_f64());
                let Some(d) = d else { out[side].push(name); continue };
                let (hx, hy) = (g(&p["mount"], "x", 0.0) + d * c, g(&p["mount"], "y", 0.0) + d * sn);
                if !(-e[1] <= hx && hx <= e[0] && -e[3] <= hy && hy <= e[2]) { ignored.push(name); continue; }
            }
            out[side].push(name);
        }
        s.obs.photo_ignored = ignored;
        out
    }
    pub fn photo_sides(&self, skip_diag: bool, apply_envelope: bool) -> [Vec<String>; 4] {
        let mut s = self.s();
        self.photo_sides_of(&mut s, skip_diag, apply_envelope)
    }

    pub fn safety_filter(&self, mut vx: f64, mut vy: f64, mut wz: f64) -> (f64, f64, f64) {
        let io = self.link.io();
        if io["inputs"]["di_estop"].as_bool().unwrap_or(false) { return (0.0, 0.0, 0.0); }
        let mut events: Vec<(&str, &str, &str, String, String, Value)> = vec![];
        let res = {
            let mut s = self.s();
            if let Some(cap) = s.speed_cap { if cap > 0.0 {
                let v = vx.hypot(vy);
                if v > cap { vx = vx * cap / v; vy = vy * cap / v; }
            } }
            let enabled = s.prot.enabled;
            if enabled && vx.abs() > 1e-3 && !(s.in_arc && vx > 0.0) {
                let sign = if vx > 0.0 { 1 } else { -1 };
                let left = if sign > 0 { s.approach_left } else { None };
                let (mut allowed, d_hit, need) = Self::allowed_speed_of(&s, sign, left);
                if let (Some(l), true) = (left, allowed > 0.0) {
                    let (full, _, _) = Self::allowed_speed_of(&s, sign, None);
                    if full < allowed {
                        let a = 0.5 * { let d = g(&s.chassis_cfg, "max_decel_mps2", 0.5); if d == 0.0 { 0.5 } else { d } };
                        let v_stop = if l > 0.002 { 0.01f64.max((2.0 * a * l).sqrt()) } else { 0.0 };
                        allowed = allowed.min(full.max(v_stop));
                    }
                }
                let (mut zone, mut layer) = ("clear", None::<String>);
                let fl = format!("field_{}", if sign > 0 { "front" } else { "rear" });
                if allowed <= 1e-6 { vx = 0.0; zone = "stop"; layer = Some(fl); }
                else if vx.abs() > allowed { vx = sign as f64 * allowed; zone = "slow"; layer = Some(fl); }
                else {
                    let (i, f) = s.prot.field_for_speed(vx);
                    let d = s.obs.bands.get(i).and_then(|b| if sign > 0 { b.0 } else { b.1 });
                    if let Some(d) = d { if d < (if sign > 0 { f.front } else { f.rear }) * s.prot.slow_ratio { zone = "warn"; } }
                }
                self.zone_locked(&mut s, zone, layer, d_hit, need, &mut events);
            } else if enabled && vx.abs() < 0.05 && wz.abs() > 0.05 {
                if Self::rotation_blocked_of(&s, if wz > 0.0 { 1.0 } else { -1.0 }) {
                    wz = 0.0;
                    self.zone_locked(&mut s, "stop", Some("rotate".into()), None, None, &mut events);
                } else if s.obs.layer.as_deref() == Some("rotate") {
                    self.zone_locked(&mut s, "clear", None, None, None, &mut events);
                }
            }
            let skip = s.in_arc;
            let ph = self.photo_sides_of(&mut s, skip, true);
            if !ph[0].is_empty() && vx > 0.0 { vx = 0.0; }
            if !ph[1].is_empty() && vx < 0.0 { vx = 0.0; }
            if !ph[2].is_empty() && vy > 0.0 { vy = 0.0; }
            if !ph[3].is_empty() && vy < 0.0 { vy = 0.0; }
            (vx, vy, wz)
        };
        for (c, t, l, ti, m, p) in events { self.ev(c, t, l, &ti, &m, p); }
        res
    }

    #[allow(clippy::too_many_arguments)]
    fn zone_locked(&self, s: &mut St, zone: &str, layer: Option<String>, d: Option<f64>, need: Option<f64>,
                   events: &mut Vec<(&'static str, &'static str, &'static str, String, String, Value)>) {
        let prev = s.obs.zone.clone();
        s.obs.layer = if zone == "slow" || zone == "stop" { layer.clone() } else { None };
        let band_name = |s: &St| s.prot.fields.get(s.obs.band).map(|f| f.name.clone()).unwrap_or_default();
        if zone == "warn" {
            if prev == "clear" && now() - s.obs.t_warn > 3.0 {
                s.obs.t_warn = now();
                events.push(("sensors", "OBS_WARN", "info", "近距避障: 预警区有障碍".into(),
                    format!("当前档 ({}) 预警区内探测到障碍，保持速度并准备降档", band_name(s)), json!({"band": s.obs.band})));
            }
            s.obs.zone = "warn".into();
            return;
        }
        if zone == prev || (prev == "warn" && zone == "clear") { s.obs.zone = zone.to_string(); return; }
        s.obs.zone = zone.to_string();
        s.zone_info = (layer.clone(), d, need);
        let t = now();
        if zone == "clear" && t - s.obs.t_clear < 0.5 { return; }
        s.obs.t_clear = t;
        let (layer, dd, need) = s.zone_info.clone();
        let band = band_name(s);
        let where_ = match layer.as_deref() { Some("field_front") => "前向防护区", Some("field_rear") => "后向防护区", Some("rotate") => "转向防护区", _ => "防护区" };
        let info = json!({"layer": layer, "dist": dd, "need": need, "band": band, "loaded": s.loaded});
        let fmt = |v: Option<f64>| v.map(|x| format!("{}", x)).unwrap_or("None".into());
        if zone == "slow" {
            events.push(("sensors", "OBS_SLOW", "warning", "近距避障: 减速避让".into(),
                format!("{where_}: 障碍 {} m < {} m，按防护区分档降速{}", fmt(dd), fmt(need), if s.loaded { " (带载外形)" } else { "" }), info));
        } else if zone == "stop" {
            if layer.as_deref() == Some("rotate") {
                events.push(("sensors", "OBS_STOP", "danger", "近距避障: 停车等待".into(),
                    format!("{where_}: 原地转向扫掠区 (外扩 {} m) 内有障碍，停止转向", s.prot.rotate_margin), info));
            } else {
                events.push(("sensors", "OBS_STOP", "danger", "近距避障: 停车等待".into(),
                    format!("{where_}: 障碍 {} m < 最低档停车距离 {} m，停车等待{}", fmt(dd), fmt(need), if s.loaded { " (带载外形)" } else { "" }), info));
            }
        } else if prev == "slow" || prev == "stop" {
            events.push(("sensors", "OBS_CLEAR", "success", "障碍解除，恢复巡航".into(), String::new(), json!({})));
        }
    }

    pub fn safety_state(&self) -> Value {
        let io = self.link.io();
        let di = io["inputs"].as_object().cloned().unwrap_or_default();
        let mut bumpers: Vec<String> = di.iter().filter(|(k, v)| k.starts_with("di_bumper_") && v.as_bool().unwrap_or(false)).map(|(k, _)| k["di_bumper_".len()..].to_string()).collect();
        bumpers.sort();
        let mut s = self.s();
        let raw = self.photo_sides_of(&mut s, false, false);
        let ph = self.photo_sides_of(&mut s, false, true);
        let sides = |p: &[Vec<String>; 4]| json!({"front": p[0], "rear": p[1], "left": p[2], "right": p[3]});
        json!({"estop": di.get("di_estop").and_then(|v| v.as_bool()).unwrap_or(false), "bumpers": bumpers,
               "photo_raw": sides(&raw), "photo": sides(&ph), "photo_ignored": s.obs.photo_ignored,
               "obstacle": {"front": s.obs.front, "rear": s.obs.rear, "zone": s.obs.zone, "layer": s.obs.layer, "band": s.obs.band},
               "params": Self::safety_params_of(&s), "speed_cap": s.speed_cap, "protection": Self::protection_view_of(&s)})
    }

    fn safety_loop(self: Arc<Self>) {
        let (mut prev_estop, mut prev_bump) = (false, false);
        while self.is_running() {
            sleep(50);
            let st = self.safety_state();
            let estop = st["estop"].as_bool().unwrap_or(false);
            let bump = st["bumpers"].as_array().map(|a| !a.is_empty()).unwrap_or(false);
            let status = self.status();
            let busy = ["NAVIGATING", "PLANNING", "OBSTACLE_WAIT", "SAFETY_STOP"].contains(&status.as_str());
            if estop && !prev_estop {
                self.ev("safety", "ESTOP", "danger", "急停触发", "执行进程暂停当前任务，等待急停复位", st.clone());
                if busy {
                    let mut s = self.s();
                    s.estop_resume = s.tele.target_goal;
                    s.cur_mid += 1;
                    s.tele.nav_status = "SAFETY_STOP".into();
                }
            }
            if !estop && prev_estop {
                let gl = self.s().estop_resume.take();
                self.ev("safety", "ESTOP_RELEASE", "success", "急停复位", if gl.is_some() { "恢复任务" } else { "" }, json!({}));
                if let Some(gl) = gl {
                    let me = self.clone();
                    std::thread::spawn(move || me.send_nav_goal(gl[0], gl[1], gl[2], 2));
                }
            }
            if bump && !prev_bump && busy && self.status() != "SAFETY_STOP" {
                {
                    let mut s = self.s();
                    s.cur_mid += 1;
                    s.tele.nav_status = "BUMPER_STOP".into();
                    s.tele.plan_path.clear();
                }
                self.mission_end("BUMPER_STOP");
                for _ in 0..3 { self.link.send_cmd(0.0, 0.0, 0.0, "nav:safety"); }
                let b: Vec<String> = st["bumpers"].as_array().unwrap().iter().map(|v| v.as_str().unwrap_or("").to_string()).collect();
                self.ev("safety", "BUMPER_STOP", "danger", "防撞触边触发，任务终止", &format!("触边 {} 压下；请移除障碍后重新下发任务", b.join(",")), st.clone());
            }
            prev_estop = estop;
            prev_bump = bump;
        }
    }

    pub fn set_planner_type(&self, t: &str) -> bool {
        if !PLANNERS.contains(&t) { return false; }
        if t == "nav2" {
            match self.nav2.get().map(|n| n.start()) {
                Some(Ok(())) => {}
                other => {
                    let why = match other { Some(Err(e)) => e, _ => "Nav2 后端未初始化".into() };
                    self.ev("navigation", "PLANNER_SWITCH", "danger", "Nav2 不可用", &why, json!({}));
                    return false;
                }
            }
        }
        if t != "nav2" {
            if let Some(n) = self.nav2.get() { if n.started() { n.cancel(); n.stop(); } }
        }
        self.s().active_planner = t.to_string();
        self.ev("navigation", "PLANNER_SWITCH", "info", "导航规划引擎切换", &format!("执行进程规划器 → {t}"), json!({"planner_type": t}));
        true
    }

    // ------------------------------------------------------------------ mission records
    pub fn new_mission(&self, mid: u64, x: f64, y: f64, yaw: f64, planner: &str) {
        let m = Mission { id: mid, planner: planner.into(), goal: [x, y, yaw], status: "PLANNING".into(), created: now(), ended: None, title: None };
        let mut s = self.s();
        if let Some(p) = s.mission.as_mut() { if p.ended.is_none() { p.ended = Some(now()); p.status = "SUPERSEDED".into(); } }
        // keep the superseded copy in history in sync
        if let Some(p) = s.mission.clone() { if let Some(h) = s.missions.iter_mut().find(|h| h.id == p.id) { *h = p; } }
        s.mission = Some(m.clone());
        s.missions.push_back(m);
        if s.missions.len() > 50 { s.missions.pop_front(); }
    }
    pub fn nav2_result(&self, mid: u64, res: &str) {
        if self.s().cur_mid != mid { return; }
        if let Some(n) = self.nav2.get() { n.set_active(false); }
        match res {
            "SUCCEEDED" => {
                let (x, y, yaw) = { let mut s = self.s(); s.tele.nav_status = "ARRIVED".into(); s.tele.plan_path.clear(); s.tele.nav_dist_rem = 0.0; (s.tele.x, s.tele.y, s.tele.yaw) };
                self.mission_end("ARRIVED");
                self.ev("navigation", "MISSION_ARRIVED", "success", &format!("任务 #{mid} 停靠到位完成"), &format!("Nav2 导航完成，位置: ({x:.2}, {y:.2})"),
                    json!({"mission_id": mid, "x": x, "y": y, "yaw": yaw}));
            }
            "CANCELED" => {}
            _ => {
                self.s().tele.nav_status = "FAILED".into();
                self.mission_end("FAILED");
                self.ev("navigation", "MISSION_FAILED", "danger", &format!("任务 #{mid} 导航失败"), &format!("Nav2 返回 {res}"), json!({"mission_id": mid}));
            }
        }
    }
    pub fn mission_set_title(&self, title: &str) {
        let mut s = self.s();
        if let Some(m) = s.mission.as_mut() { m.title = Some(title.to_string()); }
        if let Some(m) = s.mission.clone() { if let Some(h) = s.missions.iter_mut().find(|h| h.id == m.id) { *h = m; } }
    }
    pub fn mission_end(&self, status: &str) {
        let mut s = self.s();
        if let Some(m) = s.mission.as_mut() { if m.ended.is_none() { m.ended = Some(now()); m.status = status.to_string(); } }
        if let Some(m) = s.mission.clone() { if let Some(h) = s.missions.iter_mut().find(|h| h.id == m.id) { *h = m; } }
    }
    pub fn mission_view(&self, id: Option<u64>) -> Option<Value> {
        let s = self.s();
        let m = match id { Some(i) => s.missions.iter().find(|m| m.id == i).cloned(), None => s.mission.clone() }?;
        let live = s.mission.as_ref().map(|c| c.id == m.id).unwrap_or(false);
        let st = if live && m.ended.is_none() { s.tele.nav_status.clone() } else { m.status.clone() };
        let dur = m.ended.unwrap_or_else(now) - m.created;
        Some(json!({"id": m.id, "planner": m.planner, "goal": {"x": m.goal[0], "y": m.goal[1], "yaw": m.goal[2]}, "status": st,
                    "created": m.created, "ended": m.ended, "duration_s": r2(dur), "title": m.title,
                    "path": if live { Value::Array(s.tele.plan_path.iter().map(pt_json).collect()) } else { Value::Null },
                    "dist_remaining": if live { s.tele.nav_dist_rem } else { 0.0 }}))
    }
    pub fn missions_view(&self) -> Vec<Value> {
        let ids: Vec<u64> = self.s().missions.iter().map(|m| m.id).collect();
        ids.into_iter().rev().filter_map(|i| self.mission_view(Some(i))).collect()
    }

    pub fn submit(self: &Arc<Self>, x: f64, y: f64, yaw: f64, planner: Option<&str>) -> Result<Option<Value>, String> {
        if let Some(p) = planner { if !self.set_planner_type(p) { return Err(format!("规划器不可用: {p}")); } }
        self.send_nav_goal(x, y, yaw, 2);
        Ok(self.mission_view(None))
    }

    fn next_segment(s: &St) -> Value {
        let t = &s.tele;
        let i = t.path_index;
        if t.plan_path.len() < 2 || i == 0 || i >= t.plan_path.len() { return Value::Null; }
        let lab = |k: usize| t.path_labels.get(k).cloned().flatten();
        let tgt = t.plan_path[i];
        let mut seg = json!({"index": i, "to": pt_json(&tgt), "to_label": lab(i), "from_label": lab(i - 1),
                             "dist": r2((tgt.0 - t.x).hypot(tgt.1 - t.y)), "remaining_waypoints": t.plan_path.len() - i});
        if i + 1 < t.plan_path.len() {
            let n = t.plan_path[i + 1];
            seg["next"] = json!({"to": pt_json(&n), "to_label": lab(i + 1), "dist": r2((n.0 - tgt.0).hypot(n.1 - tgt.1))});
        }
        seg
    }

    pub fn localization(&self) -> Value {
        let s = self.s();
        json!({"mode": "ground_truth", "engine": "truth", "auto_frozen": false, "std_xy_mm": 0.0, "std_yaw_deg": 0.0, "score": null, "accepted": null, "map_rev": 0,
               "err_mm": 0.0, "err_yaw_deg": 0.0,
               "pose": {"x": r4(s.tele.x), "y": r4(s.tele.y), "yaw": r5(s.tele.yaw)}})
    }

    pub fn feedback_json(&self) -> Value {
        let (mut fb, planner) = {
            let s = self.s();
            let t = &s.tele;
            let curve = if t.plan_path.is_empty() { json!([]) } else { Value::Array(t.plan_curve.iter().map(pt_json).collect()) };
            (json!({"online": true, "mission_id": s.cur_mid, "status": t.nav_status, "planner": s.active_planner,
                    "goal": t.target_goal.map(|g| json!({"x": g[0], "y": g[1], "yaw": g[2]})).unwrap_or(Value::Null),
                    "path": t.plan_path.iter().map(pt_json).collect::<Vec<_>>(), "path_labels": t.path_labels,
                    "curve": curve, "protection": Self::protection_view_of(&s), "path_index": t.path_index,
                    "next_segment": Self::next_segment(&s), "dist_remaining": t.nav_dist_rem, "nav2_feedback": null, "planners": PLANNERS}),
             s.active_planner.clone())
        };
        let _ = planner;
        let mut loc = self.localization();
        let _ = &mut loc;
        fb["localization"] = loc;
        { let (on, rdy) = self.nav2.get().map(|x| (x.started(), x.ready())).unwrap_or((false, false));
          fb["nav2"] = json!({"msgs": on, "server_ready": rdy, "process": on, "autostart": false, "chassis": null, "localization": if on { json!("ground_truth") } else { Value::Null }}); }
        fb["mission"] = self.mission_view(None).unwrap_or(Value::Null);
        fb["safety"] = self.safety_state();
        if let Some(tf) = self.taskflow.get() { fb["taskflow"] = tf.view(); }
        fb
    }

    // ------------------------------------------------------------------ obstacle segments
    pub fn obstacle_segments_all(&self) -> Vec<Seg> {
        let mut segs = self.pl().static_segments().to_vec();
        let obs: Vec<(f64, f64, f64, f64)> = self.s().dyn_obs.iter().map(|o| (g(o, "x", 0.0), g(o, "y", 0.0), g(o, "w", 0.8), g(o, "h", 0.8))).collect();
        segs.extend(maneuver::box_segments(&obs));
        segs
    }
    pub fn dyn_obstacles(&self) -> Vec<Obs> { self.s().dyn_obs.iter().map(Obs::from_value).collect() }

    pub fn plan_corner(&self, node: Pt, h1: f64, h2: f64, len_in: f64, len_out: f64) -> (Corner, f64) {
        let segs = self.obstacle_segments_all();
        let (head, tail, hw, r, bm, mode) = {
            let s = self.s();
            (s.head_offset, s.tail_offset, s.half_width, Self::corner_radius_of(&s), s.prot.body_margin, CornerMode::parse(&s.prot.corner_mode))
        };
        maneuver::plan_corner(&segs, node, h1, h2, len_in, len_out, head, tail, hw, r, bm, mode)
    }

    pub fn plan_route_corners(&self, path: &[Pt]) -> (Vec<Option<(Corner, f64)>>, Vec<Pt>) {
        let n = path.len();
        let mut corners: Vec<Option<(Corner, f64)>> = vec![None; n];
        let mut curve: Vec<Pt> = if n > 0 { vec![(r3(path[0].0), r3(path[0].1))] } else { vec![] };
        let (arc_mode, chassis) = { let s = self.s(); (s.prot.corner_mode == "arc", s.chassis_type.clone()) };
        for i in 1..n {
            let (a, b) = (path[i - 1], path[i]);
            let mut arc: Option<(f64, f64, f64, f64, f64, f64)> = None; // R,d,turn,cx,cy
            if i < n - 1 {
                let c = path[i + 1];
                let (h1, h2) = ((b.1 - a.1).atan2(b.0 - a.0), (c.1 - b.1).atan2(c.0 - b.0));
                let turn = wrap(h2 - h1);
                if turn.abs() > (if arc_mode { 0.35 } else { 0.02 }) && chassis != "dual_steer" {
                    let cr = self.plan_corner(b, h1, h2, (b.0 - a.0).hypot(b.1 - a.1), (c.0 - b.0).hypot(c.1 - b.1));
                    if let Corner::Arc { r, d, turn, cx, cy, .. } = cr.0 { arc = Some((r, d, turn, cx, cy, 0.0)); }
                    corners[i] = Some(cr);
                }
            }
            if let Some((r, _d, turn, cx, cy, _)) = arc {
                let h1 = (b.1 - a.1).atan2(b.0 - a.0);
                let sgn = if turn > 0.0 { 1.0 } else { -1.0 };
                for k in 0..=12 {
                    let th = h1 + turn * k as f64 / 12.0;
                    curve.push((r3(cx + sgn * r * th.sin()), r3(cy - sgn * r * th.cos())));
                }
            } else { curve.push((r3(b.0), r3(b.1))); }
        }
        (corners, curve)
    }
}

fn r4(x: f64) -> f64 { (x * 1e4).round_ties_even() / 1e4 }
fn r5(x: f64) -> f64 { (x * 1e5).round_ties_even() / 1e5 }

impl crate::simlink::Sink for Navigator {
    fn on_state(&self, st: &SimState) {
        let (loaded, changed, outline_msg) = {
            let io = self.link.io();
            let di = &io["inputs"];
            let loaded = di["di_lift_top"].as_bool().unwrap_or(false) && !di["di_lift_bottom"].as_bool().unwrap_or(false);
            let mut s = self.s();
            let tr = &st.truth;
            s.tele.x = tr[0]; s.tele.y = tr[1]; s.tele.yaw = tr[2];
            s.tele.truth = [tr[0], tr[1], tr[2]];
            s.tele.vx = st.odom[3]; s.tele.vy = st.odom[4]; s.tele.wz = st.odom[5];
            s.paused = st.paused;
            s.steer = st.steer.clone();
            if !st.chassis.is_empty() { s.chassis_type = st.chassis.clone(); }
            s.last_state_t = st.t;
            if let Some(n2) = self.nav2.get() { n2.feed([tr[0], tr[1], tr[2]], [st.odom[0], st.odom[1], st.odom[2]]); }
            let active = s.active_planner != "nav2";
            if active {
                if let Some(gl) = s.tele.target_goal {
                    if ["NAVIGATING", "PLANNING", "OBSTACLE_WAIT"].contains(&s.tele.nav_status.as_str()) {
                        s.tele.nav_dist_rem = r2((gl[0] - s.tele.x).hypot(gl[1] - s.tele.y));
                    }
                }
            }
            let changed = loaded != s.loaded;
            let mut msg = None;
            if changed {
                s.loaded = loaded;
                msg = Some(s.prot.payload_enabled);
            }
            (loaded, changed, msg)
        };
        if changed {
            self.apply_outline();
            let [h, t, l, r] = self.outline();
            self.ev("navigation", "OUTLINE", "info", &format!("保护外形切换: {}", if loaded { "带载" } else { "空载" }),
                &format!("外形 头 {h:.2} / 尾 {t:.2} / 左 {l:.2} / 右 {r:.2} m{}", if outline_msg == Some(true) || !loaded { "" } else { " (未配置带载外形，沿用车体)" }), json!({"loaded": loaded}));
        }
    }

    fn on_merged(&self, sc: &Scan) {
        let rmax = sc.range_max;
        let mut px = vec![]; let mut py = vec![];
        for (i, &r) in sc.ranges.iter().enumerate() {
            if r > 0.02 && r < rmax - 1e-3 {
                let a = sc.angle_min + sc.angle_inc * i as f64;
                px.push(r * a.cos()); py.push(r * a.sin());
            }
        }
        let mut s = self.s();
        s.scan_body = Some((sc.ranges.clone(), sc.angle_min, sc.angle_inc, rmax));
        let [h, t, lo, ro] = Self::outline_of(&s);
        let mut bands = vec![];
        for f in &s.prot.fields {
            let (mut fmin, mut rmin) = (f64::INFINITY, f64::INFINITY);
            for i in 0..px.len() {
                if py[i] <= lo + f.side && py[i] >= -(ro + f.side) {
                    if px[i] > 0.0 { let v = px[i] - h; if v > -0.05 { fmin = fmin.min(v); } }
                    else { let v = -px[i] - t; if v > -0.05 { rmin = rmin.min(v); } }
                }
            }
            let c = |v: f64| if v.is_finite() { Some(r2(v.max(0.0))) } else { None };
            bands.push((c(fmin), c(rmin)));
        }
        let v = s.tele.vx;
        let (i, _) = s.prot.field_for_speed(v);
        s.obs.band = i;
        s.obs.front = bands.get(i).and_then(|b| b.0);
        s.obs.rear = bands.get(i).and_then(|b| b.1);
        s.obs.bands = bands;
        s.pts = Some((px, py));
    }

    fn on_world(&self, w: &Value) {
        let sid = w["id"].as_str().map(|x| x.to_string());
        let changed = {
            let mut s = self.s();
            let sid = sid.clone().unwrap_or(s.active_scenario.clone());
            if let Some(d) = w.get("scenario_def").filter(|d| !d.is_null()) { s.lib.register(d); }
            let changed = sid != s.active_scenario;
            s.active_scenario = sid.clone();
            let lib = s.lib.clone();
            let mut p = self.pl();
            p.set_scenario(&lib, &sid);
            s.walls = p.walls();
            s.dyn_obs = w["obstacles"].as_array().cloned().unwrap_or_default();
            changed
        };
        // the planner was reset: re-apply the footprint
        self.apply_outline_force();
        if changed { self.cancel_nav(); }
    }

    fn on_model(&self, m: &Value) {
        {
            let mut s = self.s();
            let ch = m["chassis"].clone();
            s.max_decel = { let d = g(&ch, "max_decel_mps2", g(&ch, "max_accel_mps2", 0.5)); d };
            s.body_v = ch.clone();
            s.chassis_cfg = ch;
            s.prot = protection::effective(m, None);
            s.runtime_prot = false;
        }
        self.apply_outline_force();
    }

    fn feedback(&self) -> Value { self.feedback_json() }
}

impl Navigator {
    /// set_scenario clears the planner footprint caches only via set_footprint; re-apply unconditionally
    fn apply_outline_force(&self) { self.apply_outline(); }
}
