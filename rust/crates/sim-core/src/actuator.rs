//! Top-module actuators other than the fork: jacking lifter with rotating tray, roller conveyor deck, tow hitch.
//! (Fork / reach mast live in `fork.rs`.)  Every actuator:
//!  * owns its kinematic state (trapezoid velocity profiles, limits),
//!  * attaches / carries / releases cargo that lives in `World::obstacles` (so lidar, cameras, MuJoCo see it move),
//!  * drives DI/DO (limit switches, presence sensors, jog outputs),
//!  * reports joint positions for `joint_states` (names match the RobotAssembly),
//!  * emits events (`JACK_*`, `ROLLER_*`, `TOW_*`).
use crate::fork::{height_of, is_cargo, obst_rect, onum, ostr, point_in, rect, round3, sat, Fork, ForkEvent, Poly};
use crate::geom::{collides, wrap, Seg};
use crate::io::IoSim;
use crate::world::World;
use serde_json::{json, Map, Value};
use std::f64::consts::PI;

/// side effects of an actuator step, applied by the core (MuJoCo mocap moves, collision toggles, world rebuild)
#[derive(Default, Debug)]
pub struct ActOut {
    pub moved: Vec<(usize, f64, f64, f64, f64)>,
    pub toggles: Vec<(usize, bool)>,
    pub dirty: bool,
    pub events: Vec<ForkEvent>,
}

fn ev(o: &mut ActOut, kind: &'static str, level: &'static str, msg: String, data: Value) { o.events.push(ForkEvent { kind, level, msg, data }); }
fn p(a: &Value, k: &str, d: f64) -> f64 { a.get(k).and_then(|v| v.as_f64()).unwrap_or(d) }

/// trapezoid profile towards `target` (or jog direction); returns new (pos, vel)
fn profile(pos: f64, vel: f64, target: Option<f64>, jog: i8, vmax: f64, acc: f64, lo: f64, hi: f64, dt: f64, brake: bool) -> (f64, f64, bool) {
    let mut vdes = 0.0;
    let mut done = false;
    if jog != 0 { vdes = jog as f64 * vmax; }
    else if let Some(t) = target {
        let d = t - pos;
        if d.abs() < 5e-4 { done = true; } else { vdes = d.signum() * vmax.min((2.0 * acc * d.abs()).sqrt()).max(0.005); }
    }
    if brake { vdes = 0.0; }
    let dv = acc * dt;
    let mut v = vel + (vdes - vel).clamp(-dv, dv);
    if let Some(t) = target { if jog == 0 && ((pos < t && pos + v * dt > t) || (pos > t && pos + v * dt < t)) { return (t, 0.0, true); } }
    let mut np = pos + v * dt;
    if np <= lo { np = lo; if v < 0.0 { v = 0.0; } }
    if np >= hi { np = hi; if v > 0.0 { v = 0.0; } }
    (np, v, done)
}

fn idx_of(world: &World, id: &str) -> Option<usize> { world.obstacles.iter().position(|o| is_cargo(o) && ostr(o, "id") == id) }

fn set_pose(world: &mut World, out: &mut ActOut, i: usize, x: f64, y: f64, yaw: f64, z0: f64) {
    let o = &mut world.obstacles[i];
    o.insert("x".into(), json!(round3(x))); o.insert("y".into(), json!(round3(y))); o.insert("yaw".into(), json!(round3(wrap(yaw)))); o.insert("z0".into(), json!(round3(z0)));
    out.moved.push((i, x, y, yaw, z0));
}

/// is the 2D footprint of obstacle `i` (at a hypothetical pose) hitting walls/shelves or other floor obstacles overlapping its z band?
fn footprint_hits(world: &World, skip: &[usize], poly: &Poly, z0: f64, zt: f64, segs: &[Seg]) -> Option<String> {
    for (j, o) in world.obstacles.iter().enumerate() {
        if skip.contains(&j) || o.get("carried").and_then(|v| v.as_bool()).unwrap_or(false) { continue; }
        let (oz0, oh) = (onum(o, "z0", 0.0), height_of(o));
        if zt <= oz0 + 1e-6 || z0 >= oz0 + oh - 1e-6 { continue; }
        if sat(poly, &obst_rect(o)) { return Some(o.get("id").and_then(|v| v.as_str()).map(|s| s.to_string()).unwrap_or_else(|| format!("{}#{j}", ostr(o, "type")))); }
    }
    let fp: Vec<[f64; 2]> = poly.to_vec();
    if collides(&fp, 0.0, 0.0, 0.0, segs).is_some() { return Some("墙/货架".into()); }
    None
}

// =====================================================================================================================
// Jacking lifter (latent / submarine AMR): lift plate + rotating tray; picks up shelf racks from underneath.
// =====================================================================================================================
#[derive(Clone, Debug)]
pub struct Jack {
    pub stroke: f64, pub speed: f64, pub accel: f64, pub deck: f64, pub tray_d: f64, pub rot_speed: f64,
    pub center: [f64; 2],
    pub di_top: String, pub di_bottom: String,
    pub h: f64, pub v: f64, pub target: Option<f64>, pub jog: i8,
    /// tray yaw relative to the chassis
    pub tray: f64, pub tray_w: f64, pub tray_target: Option<f64>,
    /// counter-rotate the tray while the chassis turns so the rack keeps its world heading (synchronous rotation)
    pub follow: bool,
    /// (rack id, rack yaw relative to tray, rack center offset in tray frame)
    pub load: Option<(String, f64, [f64; 2])>,
    pub offset_m: f64,
    pub blocked: String,
    last_th: f64,
}

impl Jack {
    pub fn new(a: &Value, center: [f64; 2]) -> Jack {
        let s = |k: &str| a.pointer(&format!("/limit_inputs/{k}")).and_then(|v| v.as_str()).unwrap_or("").to_string();
        Jack { stroke: p(a, "stroke_m", 0.06), speed: p(a, "lift_speed_mps", 0.03), accel: p(a, "lift_accel_mps2", 0.1), deck: p(a, "deck_z_m", 0.2), tray_d: p(a, "tray_diameter_m", 0.7),
            rot_speed: p(a, "rotate_speed_dps", 60.0).to_radians(), center, di_top: s("top"), di_bottom: s("bottom"),
            h: 0.0, v: 0.0, target: None, jog: 0, tray: 0.0, tray_w: 0.0, tray_target: None, follow: true, load: None, offset_m: 0.0, blocked: String::new(), last_th: f64::NAN }
    }
    pub fn top_z(&self) -> f64 { self.deck + self.h }
    fn tray_world(&self, pose: [f64; 3]) -> [f64; 2] {
        let (s, c) = pose[2].sin_cos();
        [pose[0] + c * self.center[0] - s * self.center[1], pose[1] + s * self.center[0] + c * self.center[1]]
    }
    /// rack the tray is under (center within the rack footprint)
    fn rack_above(&self, world: &World, t: [f64; 2]) -> Option<usize> {
        world.obstacles.iter().position(|o| is_cargo(o) && ostr(o, "kind") == "rack" && o.get("carried").is_none() && point_in(o, t[0], t[1], -0.05)
            && (onum(o, "z0", 0.0) + onum(o, "leg_h", 0.2)) > self.deck - 0.01)
    }
    fn rack_world(&self, pose: [f64; 3], rel_yaw: f64, off: [f64; 2]) -> (f64, f64, f64) {
        let t = self.tray_world(pose);
        let a = pose[2] + self.tray;
        let (s, c) = a.sin_cos();
        (t[0] + c * off[0] - s * off[1], t[1] + s * off[0] + c * off[1], a + rel_yaw)
    }
    /// carried rack leg footprint at a pose (for motion blocking)
    pub fn blocks_motion(&self, world: &World, pose: [f64; 3], segs: &[Seg]) -> Option<String> {
        let (id, ry, off) = self.load.as_ref()?;
        let i = idx_of(world, id)?;
        let o = &world.obstacles[i];
        let (x, y, yaw) = self.rack_world(pose, *ry, *off);
        let poly = rect(x, y, onum(o, "w", 1.0) / 2.0, onum(o, "h", 1.0) / 2.0, yaw);
        let z0 = onum(o, "z0", 0.0);
        footprint_hits(world, &[i], &poly, z0, z0 + height_of(o), segs)
    }
    pub fn step(&mut self, dt: f64, pose: [f64; 3], world: &mut World, io: &mut IoSim, brake: bool, out: &mut ActOut) {
        let (up, dn) = (io.do_("do_lift_motor_up", false), io.do_("do_lift_motor_down", false));
        let jog = if up && !dn { 1 } else if dn && !up { -1 } else { self.jog };
        let vmax = self.speed;
        let (nh, nv, done) = profile(self.h, self.v, self.target, jog, vmax, self.accel, 0.0, self.stroke, dt, brake);
        if done { self.target = None; }
        let rising = nh > self.h + 1e-9;
        let t = self.tray_world(pose);
        // ---------------------------------------------------------------- pick up
        if self.load.is_none() && rising {
            if let Some(i) = self.rack_above(world, t) {
                let o = &world.obstacles[i];
                let under = onum(o, "z0", 0.0) + onum(o, "leg_h", 0.2);
                if self.deck + nh >= under - 1e-4 {
                    let (rx, ry, ryaw) = (onum(o, "x", 0.0), onum(o, "y", 0.0), onum(o, "yaw", 0.0));
                    let a = pose[2] + self.tray;
                    let (s, c) = a.sin_cos();
                    let (dx, dy) = (rx - t[0], ry - t[1]);
                    let off = [c * dx + s * dy, -s * dx + c * dy];
                    self.offset_m = off[0].hypot(off[1]);
                    let id = ostr(o, "id").to_string();
                    if self.offset_m > 0.2 {
                        self.target = None; self.jog = 0; self.v = 0.0;
                        if self.blocked.is_empty() { ev(out, "JACK_MISALIGNED", "warning", format!("顶升偏心 {:.0} mm > 200 mm, 拒绝顶升货架 {id}", self.offset_m * 1000.0), json!({"id": id, "offset_m": round3(self.offset_m)})); }
                        self.blocked = "misaligned".into();
                        self.io_update(io);
                        return;
                    }
                    if self.offset_m > 0.03 { ev(out, "JACK_OFFSET", "warning", format!("顶升偏心 {:.0} mm (二维码对位不足)", self.offset_m * 1000.0), json!({"id": id, "offset_m": round3(self.offset_m)})); }
                    world.obstacles[i].insert("carried".into(), json!(true));
                    out.toggles.push((i, false));
                    out.dirty = true;
                    self.load = Some((id.clone(), wrap(ryaw - a), off));
                    ev(out, "JACK_PICK", "success", format!("顶升货架 {id}"), json!({"id": id, "height": round3(nh), "offset_m": round3(self.offset_m)}));
                }
            }
        }
        self.blocked.clear();
        self.h = nh; self.v = nv;
        // ---------------------------------------------------------------- tray rotation
        let dth = if self.last_th.is_nan() { 0.0 } else { wrap(pose[2] - self.last_th) };
        self.last_th = pose[2];
        let mut w = 0.0;
        if let Some(tt) = self.tray_target {
            let d = wrap(tt - self.tray);
            if d.abs() < 1e-3 { self.tray = tt; self.tray_target = None; } else { w = d.signum() * self.rot_speed.min(d.abs() / dt); }
        } else if self.follow && self.load.is_some() { w = -dth / dt.max(1e-9); }
        if brake { w = 0.0; }
        self.tray = wrap(self.tray + w * dt);
        self.tray_w = w;
        // ---------------------------------------------------------------- carry / set down
        if let Some((id, ry, off)) = self.load.clone() {
            if let Some(i) = idx_of(world, &id) {
                let lh = onum(&world.obstacles[i], "leg_h", 0.2);
                let z0 = self.deck + self.h - lh;
                if z0 <= 1e-4 && self.v <= 0.0 {
                    let (x, y, yaw) = self.rack_world(pose, ry, off);
                    set_pose(world, out, i, x, y, yaw, 0.0);
                    world.obstacles[i].remove("carried");
                    out.toggles.push((i, true));
                    out.dirty = true;
                    self.load = None;
                    ev(out, "JACK_PLACE", "success", format!("放下货架 {id}"), json!({"id": id, "x": round3(x), "y": round3(y), "yaw_deg": round3(yaw.to_degrees())}));
                } else {
                    let (x, y, yaw) = self.rack_world(pose, ry, off);
                    set_pose(world, out, i, x, y, yaw, z0.max(0.0));
                }
            } else { self.load = None; }
        }
        self.io_update(io);
    }
    fn io_update(&self, io: &mut IoSim) {
        io.lift_height_m = self.h;
        let (top, bot) = (self.h >= self.stroke - 0.001, self.h <= 0.001);
        io.set_di_raw("di_lift_top", top);
        io.set_di_raw("di_lift_bottom", bot);
        if !self.di_top.is_empty() { io.set_di_raw(&self.di_top, top); }
        if !self.di_bottom.is_empty() { io.set_di_raw(&self.di_bottom, bot); }
        io.set_di_raw("di_cargo_present", self.load.is_some());
    }
    pub fn joints(&self) -> Vec<(String, f64, f64)> { vec![("lift_joint".into(), self.h, self.v), ("tray_rotate_joint".into(), self.tray, self.tray_w)] }
    pub fn view(&self) -> Value {
        json!({"type": "jacking", "height_m": round3(self.h), "velocity_mps": round3(self.v), "target_m": self.target, "stroke_m": self.stroke, "tray_deg": round3(self.tray.to_degrees()),
            "tray_follow": self.follow, "top_z_m": round3(self.top_z()), "loaded": self.load.is_some(), "load_id": self.load.as_ref().map(|l| l.0.clone()), "offset_m": round3(self.offset_m),
            "state": if !self.blocked.is_empty() { "BLOCKED" } else if self.v > 1e-4 { "LIFTING" } else if self.v < -1e-4 { "LOWERING" } else { "IDLE" },
            "limits": {"top": self.h >= self.stroke - 0.001, "bottom": self.h <= 0.001}})
    }
}

// =====================================================================================================================
// Roller deck: powered rollers transfer totes/boxes sideways to/from fixed conveyor stations.
// =====================================================================================================================
#[derive(Clone, Debug, PartialEq)]
pub enum RollerMode { Idle, Load(f64), Unload(f64), Jog(f64) }

#[derive(Clone, Debug)]
pub struct Roller {
    pub deck: f64, pub speed: f64, pub roller_r: f64, pub deck_l: f64, pub deck_w: f64, pub center: [f64; 2],
    pub mode: RollerMode,
    pub surface_v: f64,
    pub angle: f64,
    pub stop: [f64; 2],
    /// (id, rel x, rel y, rel yaw) in robot frame
    pub load: Option<(String, f64, f64, f64)>,
    pub last: String,
}

impl Roller {
    pub fn new(a: &Value, center: [f64; 2]) -> Roller {
        Roller { deck: p(a, "deck_z_m", 0.5), speed: p(a, "transfer_speed_mps", 0.3), roller_r: p(a, "roller_diameter_m", 0.06) / 2.0, deck_l: p(a, "deck_length_m", 1.0), deck_w: p(a, "deck_width_m", 0.8),
            center, mode: RollerMode::Idle, surface_v: 0.0, angle: 0.0, stop: [0.04, 0.04], load: None, last: String::new() }
    }
    fn to_world(pose: [f64; 3], rx: f64, ry: f64) -> (f64, f64) { let (s, c) = pose[2].sin_cos(); (pose[0] + c * rx - s * ry, pose[1] + s * rx + c * ry) }
    fn to_robot(pose: [f64; 3], x: f64, y: f64) -> (f64, f64) { let (s, c) = pose[2].sin_cos(); let (dx, dy) = (x - pose[0], y - pose[1]); (c * dx + s * dy, -s * dx + c * dy) }
    /// start a transfer: side +1 = left (+y), -1 = right
    pub fn command(&mut self, action: &str, side: f64) -> Result<(), String> {
        match action {
            "load" => { if self.load.is_some() { return Err("辊筒上已有货物".into()); } self.mode = RollerMode::Load(side); }
            "unload" => { if self.load.is_none() { return Err("辊筒上没有货物".into()); } self.mode = RollerMode::Unload(side); }
            "forward" => self.mode = RollerMode::Jog(1.0),
            "reverse" => self.mode = RollerMode::Jog(-1.0),
            "stop" => self.mode = RollerMode::Idle,
            a => return Err(format!("未知辊筒动作 {a}")),
        }
        Ok(())
    }
    /// conveyor station (support) under world point with top near the deck height
    fn station_at(world: &World, x: f64, y: f64, deck: f64) -> Option<(usize, f64)> {
        world.obstacles.iter().enumerate().find(|(_, o)| ostr(o, "type") == "conveyor" && point_in(o, x, y, 0.0) && (onum(o, "z0", 0.0) + height_of(o) - deck).abs() < 0.06)
            .map(|(i, o)| (i, onum(o, "z0", 0.0) + height_of(o)))
    }
    pub fn blocks_motion(&self, world: &World, pose: [f64; 3], segs: &[Seg]) -> Option<String> {
        let (id, rx, ry, ryaw) = self.load.as_ref()?;
        let i = idx_of(world, id)?;
        let o = &world.obstacles[i];
        let (x, y) = Self::to_world(pose, *rx, *ry);
        let poly = rect(x, y, onum(o, "w", 0.6) / 2.0, onum(o, "h", 0.4) / 2.0, pose[2] + ryaw);
        let z0 = onum(o, "z0", 0.0);
        // only parts sticking out of the deck can hit things
        if ry.abs() + onum(o, "h", 0.4) / 2.0 <= self.deck_w / 2.0 + 0.01 { return None; }
        footprint_hits(world, &[i], &poly, z0, z0 + height_of(o), segs)
    }
    pub fn step(&mut self, dt: f64, pose: [f64; 3], world: &mut World, io: &mut IoSim, brake: bool, out: &mut ActOut) {
        let jog_do = if io.do_("do_roller_forward", false) { 1.0 } else if io.do_("do_roller_reverse", false) { -1.0 } else { 0.0 };
        let mut v = match self.mode { RollerMode::Load(s) => -s * self.speed, RollerMode::Unload(s) => s * self.speed, RollerMode::Jog(s) => s * self.speed, RollerMode::Idle => jog_do * self.speed };
        if brake { v = 0.0; }
        self.surface_v = v;
        self.angle += v / self.roller_r.max(1e-3) * dt;
        // stopper pins: retract on the transfer side
        let side = match self.mode { RollerMode::Load(s) | RollerMode::Unload(s) => s, _ => 0.0 };
        for (k, sgn) in [(0usize, 1.0), (1usize, -1.0)] {
            let want = if side == sgn { 0.0 } else { 0.04 };
            let d = want - self.stop[k];
            self.stop[k] += d.clamp(-0.2 * dt, 0.2 * dt);
        }
        match self.mode.clone() {
            RollerMode::Load(s) if self.load.is_none() => {
                // find a box on an adjacent station, next to the deck edge on side s
                let cand = world.obstacles.iter().enumerate().find(|(_, o)| {
                    if !is_cargo(o) || o.get("carried").is_some() || ostr(o, "kind") == "rack" || ostr(o, "kind") == "pallet" || ostr(o, "kind") == "cart" { return false; }
                    let (rx, ry) = Self::to_robot(pose, onum(o, "x", 0.0), onum(o, "y", 0.0));
                    (rx - self.center[0]).abs() < 0.25 && s * (ry - self.center[1]) > self.deck_w / 2.0 - 0.15 && s * (ry - self.center[1]) < self.deck_w / 2.0 + 1.2 && (onum(o, "z0", 0.0) - self.deck).abs() < 0.06
                }).map(|(i, o)| (i, ostr(o, "id").to_string()));
                match cand {
                    Some((i, id)) => {
                        let o = &world.obstacles[i];
                        let (rx, ry) = Self::to_robot(pose, onum(o, "x", 0.0), onum(o, "y", 0.0));
                        let ryaw = wrap(onum(o, "yaw", 0.0) - pose[2]);
                        world.obstacles[i].insert("carried".into(), json!(true));
                        out.toggles.push((i, false));
                        out.dirty = true;
                        self.load = Some((id.clone(), rx, ry, ryaw));
                        ev(out, "ROLLER_LOAD_START", "info", format!("辊筒接货 {id}"), json!({"id": id, "side": if s > 0.0 { "left" } else { "right" }}));
                    }
                    None => { self.mode = RollerMode::Idle; ev(out, "ROLLER_NO_CARGO", "warning", "辊筒接货失败: 对接侧没有货物 / 输送线高度不匹配".into(), json!({"side": if s > 0.0 { "left" } else { "right" }})); }
                }
            }
            _ => {}
        }
        if let Some((id, rx, mut ry, ryaw)) = self.load.clone() {
            let Some(i) = idx_of(world, &id) else { self.load = None; return; };
            let half = onum(&world.obstacles[i], "h", 0.4) / 2.0;
            let rel = ry - self.center[1];
            // rollers move the box only while it rests (partly) on the deck; pins stop it at the edge when idle
            let on_deck = rel.abs() - half < self.deck_w / 2.0;
            if on_deck || matches!(self.mode, RollerMode::Load(_) | RollerMode::Unload(_)) { ry += v * dt; }
            let lim = self.deck_w / 2.0 - half;
            match self.mode {
                RollerMode::Load(_) if (ry - self.center[1]).abs() <= 0.004 || (rel.signum() != (ry - self.center[1]).signum() && rel != 0.0) => {
                    ry = self.center[1];
                    self.mode = RollerMode::Idle;
                    ev(out, "ROLLER_LOADED", "success", format!("货物 {id} 已到位 (辊筒居中)"), json!({"id": id}));
                }
                RollerMode::Jog(_) | RollerMode::Idle => { ry = self.center[1] + (ry - self.center[1]).clamp(-lim.max(0.0), lim.max(0.0)); }
                _ => {}
            }
            let (x, y) = Self::to_world(pose, rx, ry);
            let rel = ry - self.center[1];
            if let RollerMode::Unload(s) = self.mode {
                if s * rel > self.deck_w / 2.0 + half {
                    // fully off the deck: a station (same height) keeps conveying it to its centre line; otherwise it falls to the floor
                    let st = Self::station_at(world, x, y, self.deck);
                    if let Some((si, _)) = st {
                        let so = &world.obstacles[si];
                        let (_, sy) = Self::to_robot(pose, onum(so, "x", 0.0), onum(so, "y", 0.0));
                        if s * (ry - sy) < -0.005 {
                            if let Some(l) = self.load.as_mut() { l.2 = ry; }
                            set_pose(world, out, i, x, y, pose[2] + ryaw, self.deck);
                            self.io_update(io);
                            return;
                        }
                    }
                    let (z0, lvl, msg) = match st {
                        Some((_, top)) => (top, "success", format!("货物 {id} 已送入输送线")),
                        None => (Fork::support_under(world, &[id.clone()], x, y, self.deck).0, "danger", format!("货物 {id} 跌落: 对接侧无输送线")),
                    };
                    set_pose(world, out, i, x, y, pose[2] + ryaw, z0);
                    world.obstacles[i].remove("carried");
                    out.toggles.push((i, true));
                    out.dirty = true;
                    self.load = None;
                    self.mode = RollerMode::Idle;
                    ev(out, if lvl == "success" { "ROLLER_UNLOADED" } else { "ROLLER_DROP" }, lvl, msg, json!({"id": id, "x": round3(x), "y": round3(y)}));
                    self.io_update(io);
                    return;
                }
            }
            if let Some(l) = self.load.as_mut() { l.2 = ry; }
            set_pose(world, out, i, x, y, pose[2] + ryaw, self.deck);
        }
        self.io_update(io);
    }
    fn io_update(&self, io: &mut IoSim) {
        let centered = self.load.as_ref().map(|l| (l.2 - self.center[1]).abs() < 0.01).unwrap_or(false);
        io.set_di_raw("di_cargo_present", self.load.is_some());
        io.set_di_raw("di_roller_in_place", centered);
        io.set_di_raw("di_roller_running", self.surface_v.abs() > 1e-3);
    }
    pub fn joints(&self, n_rollers: usize) -> Vec<(String, f64, f64)> {
        let w = self.surface_v / self.roller_r.max(1e-3);
        let mut v: Vec<(String, f64, f64)> = (0..n_rollers).map(|i| (format!("roller_{i}_joint"), self.angle, w)).collect();
        v.push(("stopper_l_joint".into(), self.stop[0], 0.0));
        v.push(("stopper_r_joint".into(), self.stop[1], 0.0));
        v
    }
    pub fn view(&self) -> Value {
        let mode = match self.mode { RollerMode::Idle => "IDLE".to_string(), RollerMode::Load(s) => format!("LOADING_{}", if s > 0.0 { "LEFT" } else { "RIGHT" }), RollerMode::Unload(s) => format!("UNLOADING_{}", if s > 0.0 { "LEFT" } else { "RIGHT" }), RollerMode::Jog(s) => if s > 0.0 { "JOG_FWD".into() } else { "JOG_REV".into() } };
        json!({"type": "roller", "state": mode, "surface_speed_mps": round3(self.surface_v), "deck_z_m": self.deck, "loaded": self.load.is_some(), "load_id": self.load.as_ref().map(|l| l.0.clone()),
            "load_offset_m": self.load.as_ref().map(|l| round3(l.2 - self.center[1])), "stoppers_m": [round3(self.stop[0]), round3(self.stop[1])]})
    }
}

// =====================================================================================================================
// Tow hitch: pin hitch at the rear; carts follow as a trailer (drawbar ring on the hitch point).
// =====================================================================================================================
#[derive(Clone, Debug)]
pub struct Tow {
    pub hitch: [f64; 2], pub stroke: f64, pub pin_speed: f64, pub capture: f64, pub max_art: f64,
    pub pin: f64, pub pin_v: f64, pub pin_target: f64,
    /// (cart id, drawbar length ring→cart center)
    pub cart: Option<(String, f64)>,
    pub articulation: f64,
    pub jackknife: bool,
}

impl Tow {
    pub fn new(a: &Value, yc: f64) -> Tow {
        let st = p(a, "pin_stroke_m", 0.08);
        Tow { hitch: [p(a, "hitch_x_m", -0.6), yc], stroke: st, pin_speed: p(a, "pin_speed_mps", 0.08), capture: p(a, "capture_radius_m", 0.08), max_art: p(a, "max_articulation_deg", 75.0).to_radians(),
            pin: st, pin_v: 0.0, pin_target: st, cart: None, articulation: 0.0, jackknife: false }
    }
    pub fn hitch_world(&self, pose: [f64; 3]) -> [f64; 2] { let (s, c) = pose[2].sin_cos(); [pose[0] + c * self.hitch[0] - s * self.hitch[1], pose[1] + s * self.hitch[0] + c * self.hitch[1]] }
    pub fn lock(&mut self) { self.pin_target = 0.0; }
    pub fn release(&mut self) { self.pin_target = self.stroke; }
    fn ring_of(o: &Map<String, Value>) -> [f64; 2] {
        let (x, y, yaw) = (onum(o, "x", 0.0), onum(o, "y", 0.0), onum(o, "yaw", 0.0));
        let d = onum(o, "w", 1.2) / 2.0 + onum(o, "drawbar_m", 0.35);
        [x + yaw.cos() * d, y + yaw.sin() * d]
    }
    /// would the towed cart hit something if the robot were at `pose`?
    pub fn blocks_motion(&self, world: &World, pose: [f64; 3], segs: &[Seg]) -> Option<String> {
        let (id, l) = self.cart.as_ref()?;
        let i = idx_of(world, id)?;
        let o = &world.obstacles[i];
        let hw = self.hitch_world(pose);
        let (cx, cy) = (onum(o, "x", 0.0), onum(o, "y", 0.0));
        let th = (hw[1] - cy).atan2(hw[0] - cx);
        let (nx, ny) = (hw[0] - l * th.cos(), hw[1] - l * th.sin());
        let poly = rect(nx, ny, onum(o, "w", 1.2) / 2.0, onum(o, "h", 0.8) / 2.0, th);
        footprint_hits(world, &[i], &poly, 0.0, height_of(o), segs)
    }
    pub fn step(&mut self, dt: f64, pose: [f64; 3], world: &mut World, io: &mut IoSim, brake: bool, out: &mut ActOut) {
        if io.do_("do_hitch_lock", false) { self.pin_target = 0.0; }
        let (np, nv, _) = profile(self.pin, self.pin_v, Some(self.pin_target), 0, self.pin_speed, 0.5, 0.0, self.stroke, dt, brake);
        let closing = np < self.pin - 1e-9;
        self.pin = np; self.pin_v = nv;
        let hw = self.hitch_world(pose);
        if self.cart.is_none() && closing && self.pin <= 0.001 {
            let c = world.obstacles.iter().enumerate().find(|(_, o)| is_cargo(o) && ostr(o, "kind") == "cart" && { let r = Self::ring_of(o); (r[0] - hw[0]).hypot(r[1] - hw[1]) <= self.capture })
                .map(|(i, o)| (i, ostr(o, "id").to_string(), onum(o, "w", 1.2) / 2.0 + onum(o, "drawbar_m", 0.35)));
            match c {
                Some((_, id, l)) => { ev(out, "TOW_HITCHED", "success", format!("挂接料车 {id}"), json!({"id": id})); self.cart = Some((id, l)); }
                None => ev(out, "TOW_EMPTY", "warning", "挂钩落下, 但挂钩位置没有料车牵引环".into(), json!({"hitch": [round3(hw[0]), round3(hw[1])]})),
            }
        }
        if self.cart.is_some() && self.pin > 0.01 {
            let id = self.cart.take().unwrap().0;
            ev(out, "TOW_RELEASED", "info", format!("脱钩 {id}"), json!({"id": id}));
        }
        if let Some((id, l)) = self.cart.clone() {
            if let Some(i) = idx_of(world, &id) {
                let o = &world.obstacles[i];
                let (cx, cy) = (onum(o, "x", 0.0), onum(o, "y", 0.0));
                // trailer kinematics: the cart center trails the hitch point along the drawbar (tractrix)
                let th = (hw[1] - cy).atan2(hw[0] - cx);
                let (nx, ny) = (hw[0] - l * th.cos(), hw[1] - l * th.sin());
                if (nx - cx).abs() > 1e-5 || (ny - cy).abs() > 1e-5 {
                    set_pose(world, out, i, nx, ny, th, 0.0);
                    out.dirty = true;
                }
                // articulation: angle between the robot's rearward axis and the drawbar
                self.articulation = wrap(pose[2] - th);
                let jk = self.articulation.abs() > self.max_art;
                if jk && !self.jackknife { ev(out, "TOW_JACKKNIFE", "danger", format!("折叠角 {:.0}° 超限 ({:.0}°)", self.articulation.to_degrees(), self.max_art.to_degrees()), json!({"id": id, "deg": round3(self.articulation.to_degrees())})); }
                self.jackknife = jk;
            } else { self.cart = None; }
        } else { self.articulation = 0.0; self.jackknife = false; }
        io.set_di_raw("di_hitch_locked", self.pin <= 0.001);
        io.set_di_raw("di_cart_present", self.cart.is_some());
        io.set_di_raw("di_cargo_present", self.cart.is_some());
    }
    pub fn joints(&self) -> Vec<(String, f64, f64)> { vec![("hitch_pin_joint".into(), self.pin, self.pin_v)] }
    pub fn view(&self) -> Value {
        json!({"type": "tugger", "pin_up_m": round3(self.pin), "locked": self.pin <= 0.001, "loaded": self.cart.is_some(), "load_id": self.cart.as_ref().map(|c| c.0.clone()),
            "articulation_deg": round3(self.articulation.to_degrees()), "jackknife": self.jackknife, "max_articulation_deg": round3(self.max_art.to_degrees()),
            "state": if self.jackknife { "JACKKNIFE" } else if self.pin_v.abs() > 1e-4 { if self.pin_v > 0.0 { "RELEASING" } else { "LOCKING" } } else if self.pin <= 0.001 { "LOCKED" } else { "OPEN" }})
    }
}

// =====================================================================================================================
pub enum Act { Jack(Jack), Roller(Roller, usize), Tow(Tow) }

impl Act {
    pub fn from_assembly(asm: &Value) -> Option<Act> {
        let a = &asm["actuator"];
        let c = asm["chassis"]["center"].as_array().map(|v| [v[0].as_f64().unwrap_or(0.0), v[1].as_f64().unwrap_or(0.0)]).unwrap_or([0.0, 0.0]);
        let nrol = asm["links"].as_array().map(|l| l.iter().filter(|l| l["name"].as_str().map(|n| n.starts_with("roller_")).unwrap_or(false)).count()).unwrap_or(0);
        match asm["archetype"].as_str()? {
            "jacking" => Some(Act::Jack(Jack::new(a, c))),
            "roller" => Some(Act::Roller(Roller::new(a, c), nrol)),
            "tugger" => Some(Act::Tow(Tow::new(a, c[1]))),
            _ => None,
        }
    }
    pub fn step(&mut self, dt: f64, pose: [f64; 3], world: &mut World, io: &mut IoSim, brake: bool) -> ActOut {
        let mut o = ActOut::default();
        match self { Act::Jack(j) => j.step(dt, pose, world, io, brake, &mut o), Act::Roller(r, _) => r.step(dt, pose, world, io, brake, &mut o), Act::Tow(t) => t.step(dt, pose, world, io, brake, &mut o) }
        o
    }
    pub fn blocks_motion(&self, world: &World, pose: [f64; 3], segs: &[Seg]) -> Option<String> {
        match self { Act::Jack(j) => j.blocks_motion(world, pose, segs), Act::Roller(r, _) => r.blocks_motion(world, pose, segs), Act::Tow(t) => t.blocks_motion(world, pose, segs) }
    }
    pub fn joints(&self) -> Vec<(String, f64, f64)> { match self { Act::Jack(j) => j.joints(), Act::Roller(r, n) => r.joints(*n), Act::Tow(t) => t.joints() } }
    pub fn view(&self) -> Value { match self { Act::Jack(j) => j.view(), Act::Roller(r, _) => r.view(), Act::Tow(t) => t.view() } }
    pub fn loaded(&self) -> bool { match self { Act::Jack(j) => j.load.is_some(), Act::Roller(r, _) => r.load.is_some(), Act::Tow(t) => t.cart.is_some() } }
    pub fn carries(&self, id: &str) -> bool {
        match self { Act::Jack(j) => j.load.as_ref().map(|l| l.0 == id).unwrap_or(false), Act::Roller(r, _) => r.load.as_ref().map(|l| l.0 == id).unwrap_or(false), Act::Tow(t) => t.cart.as_ref().map(|c| c.0 == id).unwrap_or(false) }
    }
    /// generic command: {action, height?, tray_deg?, follow?, side?}
    pub fn command(&mut self, b: &Value) -> Result<(), String> {
        let action = b.get("action").and_then(|a| a.as_str()).unwrap_or("");
        match self {
            Act::Jack(j) => {
                match action {
                    "up" | "lift" => { j.target = Some(j.stroke); j.jog = 0; }
                    "down" | "lower" => { j.target = Some(0.0); j.jog = 0; }
                    "stop" => { j.target = None; j.jog = 0; j.tray_target = None; }
                    "rotate" => {}
                    "" => {}
                    a => return Err(format!("未知顶升动作 {a} (up|down|stop|rotate)")),
                }
                if let Some(h) = b.get("height").and_then(|v| v.as_f64()) { j.target = Some(h.clamp(0.0, j.stroke)); j.jog = 0; }
                if let Some(d) = b.get("tray_deg").and_then(|v| v.as_f64()) { j.tray_target = Some(wrap(d.to_radians())); }
                if let Some(d) = b.get("rotate_deg").and_then(|v| v.as_f64()) { j.tray_target = Some(wrap(j.tray + d.to_radians())); }
                if let Some(f) = b.get("follow").and_then(|v| v.as_bool()) { j.follow = f; }
                Ok(())
            }
            Act::Roller(r, _) => {
                let side = match b.get("side").and_then(|s| s.as_str()).unwrap_or("left") { "right" => -1.0, _ => 1.0 };
                r.command(action, side)
            }
            Act::Tow(t) => match action { "lock" | "hitch" => { t.lock(); Ok(()) } "release" | "unhitch" => { t.release(); Ok(()) } a => Err(format!("未知牵引动作 {a} (lock|release)")) },
        }
    }
}

/// Build cargo for the new actuator types (rack, cart, tote); pallets/boxes come from `fork::make_cargo`.
pub fn make_cargo(kind: &str, id: &str, x: f64, y: f64, yaw: f64, z0: f64, size: Option<[f64; 3]>, extra: &Value) -> Option<Map<String, Value>> {
    let (w, h, z, typ) = match kind {
        "rack" => { let s = size.unwrap_or([1.2, 1.0, 1.6]); (s[0], s[1], s[2], "rack") }
        "cart" => { let s = size.unwrap_or([1.2, 0.8, 1.0]); (s[0], s[1], s[2], "cart") }
        "tote" => { let s = size.unwrap_or([0.6, 0.4, 0.32]); (s[0], s[1], s[2], "box") }
        _ => return None,
    };
    let mut m = Map::new();
    m.insert("cargo".into(), json!(true)); m.insert("id".into(), json!(id)); m.insert("kind".into(), json!(kind)); m.insert("type".into(), json!(typ));
    m.insert("x".into(), json!(x)); m.insert("y".into(), json!(y)); m.insert("yaw".into(), json!(yaw));
    m.insert("w".into(), json!(w)); m.insert("h".into(), json!(h)); m.insert("z".into(), json!(z)); m.insert("z0".into(), json!(z0));
    if kind == "rack" { m.insert("leg_h".into(), json!(extra.get("leg_h").and_then(|v| v.as_f64()).unwrap_or(0.2))); m.insert("leg_w".into(), json!(0.05)); }
    if kind == "cart" { m.insert("drawbar_m".into(), json!(0.35)); m.insert("hitch_z".into(), json!(extra.get("hitch_z").and_then(|v| v.as_f64()).unwrap_or(0.3))); m.insert("deck_z".into(), json!(0.28)); }
    let _ = PI;
    Some(m)
}
