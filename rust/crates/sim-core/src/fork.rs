//! Fork / mast lifter + cargo (pallet / box) simulation.
//!
//! Cargo lives inside `World::obstacles` (entries with `"cargo": true`, `"id"`, `"z0"`), so MuJoCo geoms, lidar rays, camera
//! scene and bumpers see it like any obstacle.  The fork itself is a kinematic mechanism (mast height with trapezoid velocity
//! profile) whose tines are tested against the world each step; pallets have real runners/pockets so mis-aligned insertion is
//! blocked, and lifting under the deck attaches the pallet (with everything stacked on it) until it is lowered back onto a
//! support (floor, platform or other cargo).
use crate::geom::{collides, Seg};
use crate::io::IoSim;
use crate::world::World;
use serde_json::{json, Map, Value};

pub const RUNNER_W: f64 = 0.10;

#[derive(Clone, Debug)]
pub struct ForkSpec {
    pub name: String,
    pub x: f64,
    pub mast_max: f64,
    pub speed: f64,
    pub accel: f64,
    pub tine_len: f64,
    pub tine_w: f64,
    pub tine_t: f64,
    pub spacing: f64,
    pub max_load: f64,
    pub loaded_factor: f64,
}

impl ForkSpec {
    pub fn from_spec(spec: &Value) -> Option<ForkSpec> {
        let f = spec.get("fork").filter(|f| f.is_object())?;
        let g = |k: &str, d: f64| f.get(k).and_then(|v| v.as_f64()).filter(|v| *v > 0.0).unwrap_or(d);
        Some(ForkSpec {
            name: f.get("name").and_then(|v| v.as_str()).unwrap_or("fork").to_string(),
            x: f.get("x").and_then(|v| v.as_f64()).unwrap_or(0.9),
            mast_max: g("mast_max_m", 3.0), speed: g("lift_speed_mps", 0.3), accel: g("lift_accel_mps2", 0.4),
            tine_len: g("tine_length_m", 1.1), tine_w: g("tine_width_m", 0.12), tine_t: g("tine_thickness_m", 0.06),
            spacing: g("tine_spacing_m", 0.35), max_load: g("max_load_kg", 1000.0), loaded_factor: g("load_speed_factor", 0.6),
        })
    }
}

#[derive(Clone, Debug)]
pub struct Loaded { pub id: String, pub rel: [f64; 3], pub dz: f64 }

#[derive(Clone, Debug)]
pub struct ForkEvent { pub kind: &'static str, pub level: &'static str, pub msg: String, pub data: Value }

#[derive(Clone, Debug)]
pub struct Fork {
    pub sp: ForkSpec,
    pub h: f64,
    pub v: f64,
    pub target: Option<f64>,
    pub jog: i8,
    pub load: Vec<Loaded>,
    pub root: Option<String>,
    pub blocked_n: u32,
    pub blocked_reason: String,
    pub events: Vec<ForkEvent>,
    pub moved: Vec<(usize, f64, f64, f64, f64)>,
    pub collide_toggle: Vec<(usize, bool)>,
    pub dirty_world: bool,
    last_pose: [f64; 3],
    last_good: Option<[f64; 3]>,
}

type Poly = [[f64; 2]; 4];

fn onum(o: &Map<String, Value>, k: &str, d: f64) -> f64 { o.get(k).and_then(|x| x.as_f64()).unwrap_or(d) }
fn ostr<'a>(o: &'a Map<String, Value>, k: &str) -> &'a str { o.get(k).and_then(|x| x.as_str()).unwrap_or("") }
pub fn is_cargo(o: &Map<String, Value>) -> bool { o.get("cargo").and_then(|c| c.as_bool()).unwrap_or(false) }
fn height_of(o: &Map<String, Value>) -> f64 { o.get("z").and_then(|v| v.as_f64()).or_else(|| o.get("height").and_then(|v| v.as_f64())).unwrap_or(1.0) }

fn rect(cx: f64, cy: f64, hx: f64, hy: f64, yaw: f64) -> Poly {
    let (s, c) = yaw.sin_cos();
    let p = |dx: f64, dy: f64| [cx + c * dx - s * dy, cy + s * dx + c * dy];
    [p(-hx, -hy), p(hx, -hy), p(hx, hy), p(-hx, hy)]
}

fn sat(a: &Poly, b: &Poly) -> bool {
    for poly in [a, b] {
        for i in 0..4 {
            let (p, q) = (poly[i], poly[(i + 1) % 4]);
            let (nx, ny) = (q[1] - p[1], p[0] - q[0]);
            let proj = |pl: &Poly| pl.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(mn, mx), v| { let d = v[0] * nx + v[1] * ny; (mn.min(d), mx.max(d)) });
            let (a0, a1) = proj(a);
            let (b0, b1) = proj(b);
            if a1 <= b0 + 1e-9 || b1 <= a0 + 1e-9 { return false; }
        }
    }
    true
}

fn obst_rect(o: &Map<String, Value>) -> Poly { rect(onum(o, "x", 0.0), onum(o, "y", 0.0), onum(o, "w", 0.8) / 2.0, onum(o, "h", 0.8) / 2.0, onum(o, "yaw", 0.0)) }

fn point_in(o: &Map<String, Value>, px: f64, py: f64, margin: f64) -> bool {
    let (x, y, yaw) = (onum(o, "x", 0.0), onum(o, "y", 0.0), onum(o, "yaw", 0.0));
    let (s, c) = yaw.sin_cos();
    let (dx, dy) = (px - x, py - y);
    let (lx, ly) = (c * dx + s * dy, -s * dx + c * dy);
    lx.abs() <= onum(o, "w", 0.8) / 2.0 + margin && ly.abs() <= onum(o, "h", 0.8) / 2.0 + margin
}

pub fn round3(v: f64) -> f64 { (v * 1000.0).round() / 1000.0 }

impl Fork {
    pub fn new(sp: ForkSpec) -> Fork {
        Fork { sp, h: 0.0, v: 0.0, target: None, jog: 0, load: vec![], root: None, blocked_n: 0, blocked_reason: String::new(), events: vec![], moved: vec![], collide_toggle: vec![], dirty_world: false, last_pose: [f64::NAN; 3], last_good: None }
    }

    fn ev(&mut self, kind: &'static str, level: &'static str, msg: String, data: Value) { self.events.push(ForkEvent { kind, level, msg, data }); }

    pub fn tine_polys(&self, x: f64, y: f64, th: f64) -> [Poly; 2] {
        let (s, c) = th.sin_cos();
        let mk = |ty: f64| {
            let (lx, ly) = (self.sp.x + self.sp.tine_len / 2.0, ty);
            rect(x + c * lx - s * ly, y + s * lx + c * ly, self.sp.tine_len / 2.0, self.sp.tine_w / 2.0, th)
        };
        [mk(self.sp.spacing / 2.0), mk(-self.sp.spacing / 2.0)]
    }

    fn is_carried(&self, id: &str) -> bool { self.load.iter().any(|l| l.id == id) }

    /// ids of `root` plus all cargo stacked (directly or indirectly) on it
    fn collect_stack(world: &World, root: &str) -> Vec<String> {
        let mut out = vec![root.to_string()];
        let mut i = 0;
        while i < out.len() {
            let cur = out[i].clone();
            for o in &world.obstacles {
                if is_cargo(o) && ostr(o, "parent") == cur && !out.iter().any(|x| x == ostr(o, "id")) { out.push(ostr(o, "id").to_string()); }
            }
            i += 1;
        }
        out
    }

    fn idx_of(world: &World, id: &str) -> Option<usize> { world.obstacles.iter().position(|o| is_cargo(o) && ostr(o, "id") == id) }

    /// highest top surface (floor 0) below `bottom + tol` under (px,py), ignoring ids; returns (top, id of cargo)
    pub fn support_under(world: &World, ignore: &[String], px: f64, py: f64, bottom: f64) -> (f64, Option<String>) {
        let mut best = (0.0, None);
        for o in &world.obstacles {
            if ignore.iter().any(|i| i == ostr(o, "id")) && is_cargo(o) { continue; }
            let t = ostr(o, "type");
            let supportive = is_cargo(o) || t == "platform" || t == "table" || t == "rack_beam" || o.get("support").and_then(|v| v.as_bool()).unwrap_or(false);
            if !supportive || o.get("carried").and_then(|v| v.as_bool()).unwrap_or(false) { continue; }
            let top = onum(o, "z0", 0.0) + height_of(o);
            if top <= bottom + 0.02 && top > best.0 && point_in(o, px, py, 0.0) { best = (top, if is_cargo(o) { Some(ostr(o, "id").to_string()) } else { None }); }
        }
        best
    }

    /// recompute `parent` for every cargo item (what it is resting on)
    pub fn settle_parents(world: &mut World) {
        let snap = world.obstacles.clone();
        let w2 = World { obstacles: snap, ..world.clone() };
        for o in world.obstacles.iter_mut() {
            if !is_cargo(o) { continue; }
            let id = ostr(o, "id").to_string();
            let (px, py, z0) = (onum(o, "x", 0.0), onum(o, "y", 0.0), onum(o, "z0", 0.0));
            let (_, p) = Self::support_under(&w2, &[id], px, py, z0);
            match p { Some(p) => { o.insert("parent".into(), json!(p)); } None => { o.remove("parent"); } }
        }
    }

    /// Does the tine volume at height `h` (tine bottom) intersect something solid at pose?  `allow_attach` returns the pallet index when
    /// the contact is a pallet deck seen from below with proper insertion (pick-up).
    fn tines_hit(&self, world: &World, x: f64, y: f64, th: f64, h: f64, segs: &[Seg]) -> Option<(usize, bool)> {
        let tp = self.tine_polys(x, y, th);
        let (zb, zt) = (h, h + self.sp.tine_t);
        for (i, o) in world.obstacles.iter().enumerate() {
            if is_cargo(o) && self.is_carried(ostr(o, "id")) { continue; }
            let (z0, hh) = (onum(o, "z0", 0.0), height_of(o));
            if zt <= z0 + 1e-6 || zb >= z0 + hh - 1e-6 { continue; }
            let pr = obst_rect(o);
            if !tp.iter().any(|t| sat(t, &pr)) { continue; }
            let ph = onum(o, "pocket_h", 0.0);
            if ph > 0.0 {
                let (x0, y0, yaw) = (onum(o, "x", 0.0), onum(o, "y", 0.0), onum(o, "yaw", 0.0));
                let (hx, hy) = (onum(o, "w", 1.2) / 2.0, onum(o, "h", 0.8) / 2.0);
                // runners (full depth) occupy z in [z0, z0+ph]
                if zb < z0 + ph {
                    let (s, c) = yaw.sin_cos();
                    for ry in [-(hy - RUNNER_W / 2.0), 0.0, hy - RUNNER_W / 2.0] {
                        let rc = rect(x0 - s * ry, y0 + c * ry, hx, RUNNER_W / 2.0, yaw);
                        if tp.iter().any(|t| sat(t, &rc)) { return Some((i, false)); }
                    }
                }
                if zt > z0 + ph + 1e-6 {
                    return Some((i, self.insertion_ok(o, x, y, th)));
                }
                continue;
            }
            return Some((i, false));
        }
        for t in &tp {
            let fp: Vec<[f64; 2]> = t.to_vec();
            if collides(&fp, 0.0, 0.0, 0.0, segs).is_some() { return Some((usize::MAX, false)); }
        }
        None
    }

    fn insertion_ok(&self, o: &Map<String, Value>, x: f64, y: f64, th: f64) -> bool {
        if onum(o, "pocket_h", 0.0) <= 0.0 { return false; }
        let (s, c) = th.sin_cos();
        let mut cnt = [0, 0];
        for (k, ty) in [self.sp.spacing / 2.0, -self.sp.spacing / 2.0].iter().enumerate() {
            for j in 0..8 {
                let lx = self.sp.x + self.sp.tine_len * (j as f64 + 0.5) / 8.0;
                let (px, py) = (x + c * lx - s * ty, y + s * lx + c * ty);
                if point_in(o, px, py, 0.0) { cnt[k] += 1; }
            }
        }
        cnt[0] >= 5 && cnt[1] >= 5
    }

    fn carried_polys(&self, world: &World, x: f64, y: f64, th: f64) -> Vec<(Poly, f64, f64)> {
        let (s, c) = th.sin_cos();
        let mut v = vec![];
        for l in &self.load {
            let Some(i) = Self::idx_of(world, &l.id) else { continue };
            let o = &world.obstacles[i];
            let (wx, wy) = (x + c * l.rel[0] - s * l.rel[1], y + s * l.rel[0] + c * l.rel[1]);
            v.push((rect(wx, wy, onum(o, "w", 0.8) / 2.0, onum(o, "h", 0.8) / 2.0, th + l.rel[2]), onum(o, "z0", 0.0), height_of(o)));
        }
        v
    }

    /// would the vehicle's fork part (tines + carried load) hit something if the base moved to this pose?
    pub fn blocks_motion(&self, world: &World, nx: f64, ny: f64, nth: f64, segs: &[Seg]) -> bool {
        // already overlapping at the current pose -> allow escaping
        let (cx, cy, cth) = (self.last_pose[0], self.last_pose[1], self.last_pose[2]);
        let now_hit = !cx.is_nan() && self.hit_at(world, cx, cy, cth, segs);
        if now_hit { return false; }
        self.hit_at(world, nx, ny, nth, segs)
    }

    fn hit_at(&self, world: &World, x: f64, y: f64, th: f64, segs: &[Seg]) -> bool {
        if let Some((_, att)) = self.tines_hit(world, x, y, th, self.h, segs) { return !att; }
        for (poly, z0, hh) in self.carried_polys(world, x, y, th) {
            for (i, o) in world.obstacles.iter().enumerate() {
                let _ = i;
                if is_cargo(o) && self.is_carried(ostr(o, "id")) { continue; }
                let (oz0, oh) = (onum(o, "z0", 0.0), height_of(o));
                if z0 >= oz0 + oh - 1e-6 || z0 + hh <= oz0 + 1e-6 { continue; }
                if sat(&poly, &obst_rect(o)) { return true; }
            }
            let fp: Vec<[f64; 2]> = poly.to_vec();
            if collides(&fp, 0.0, 0.0, 0.0, segs).is_some() { return true; }
        }
        false
    }

    /// after the physics step: if the tines / load now overlap something, return the last collision-free pose to snap back to
    pub fn post_check(&mut self, world: &World, x: f64, y: f64, th: f64) -> Option<[f64; 3]> {
        let hit = self.hit_at(world, x, y, th, &world.static_segments);
        if !hit { self.last_good = Some([x, y, th]); return None; }
        self.last_good
    }

    pub fn set_target(&mut self, h: f64) { self.target = Some(h.clamp(0.0, self.sp.mast_max)); self.jog = 0; }
    pub fn stop(&mut self) { self.target = None; self.jog = 0; }

    fn attach(&mut self, world: &mut World, idx: usize, x: f64, y: f64, th: f64) {
        let id = ostr(&world.obstacles[idx], "id").to_string();
        let stack = Self::collect_stack(world, &id);
        let base_z0 = onum(&world.obstacles[idx], "z0", 0.0);
        let (s, c) = th.sin_cos();
        self.load.clear();
        for sid in &stack {
            let Some(i) = Self::idx_of(world, sid) else { continue };
            let o = &mut world.obstacles[i];
            let (ox, oy, oyaw, oz0) = (onum(o, "x", 0.0), onum(o, "y", 0.0), onum(o, "yaw", 0.0), onum(o, "z0", 0.0));
            let (dx, dy) = (ox - x, oy - y);
            self.load.push(Loaded { id: sid.clone(), rel: [c * dx + s * dy, -s * dx + c * dy, oyaw - th], dz: oz0 - base_z0 });
            o.insert("carried".into(), json!(true));
            self.collide_toggle.push((i, false));
        }
        self.root = Some(id.clone());
        self.dirty_world = true;
        let n = self.load.len();
        self.ev("FORK_PICK", "success", format!("叉取托盘 {id} (含堆叠 {} 件) 高度 {:.2} m", n - 1, self.h), json!({"id": id, "stack": n, "height": round3(self.h)}));
    }

    fn release(&mut self, world: &mut World) {
        let id = self.root.take().unwrap_or_default();
        for l in std::mem::take(&mut self.load) {
            if let Some(i) = Self::idx_of(world, &l.id) {
                world.obstacles[i].remove("carried");
                self.collide_toggle.push((i, true));
            }
        }
        Self::settle_parents(world);
        self.dirty_world = true;
        self.ev("FORK_PLACE", "success", format!("放下托盘 {id}"), json!({"id": id}));
    }

    fn sync_load(&mut self, world: &mut World, x: f64, y: f64, th: f64) {
        if self.load.is_empty() { return; }
        let root_i = self.root.as_ref().and_then(|r| Self::idx_of(world, r));
        let ph = root_i.map(|i| onum(&world.obstacles[i], "pocket_h", 0.1)).unwrap_or(0.1);
        let base = self.h + self.sp.tine_t - ph;
        let (s, c) = th.sin_cos();
        for l in &self.load {
            let Some(i) = Self::idx_of(world, &l.id) else { continue };
            let o = &mut world.obstacles[i];
            let (wx, wy) = (x + c * l.rel[0] - s * l.rel[1], y + s * l.rel[0] + c * l.rel[1]);
            let (wyaw, z0) = (th + l.rel[2], base + l.dz);
            o.insert("x".into(), json!(round3(wx))); o.insert("y".into(), json!(round3(wy))); o.insert("yaw".into(), json!(round3(wyaw))); o.insert("z0".into(), json!(round3(z0)));
            self.moved.push((i, wx, wy, wyaw, z0));
        }
    }

    pub fn step(&mut self, dt: f64, pose: [f64; 3], world: &mut World, io: &mut IoSim, brake: bool) {
        let [x, y, th] = pose;
        let sp = self.sp.clone();
        let segs: Vec<Seg> = world.static_segments.clone();
        // jog by DO
        let (up, dn) = (io.do_("do_lift_motor_up", false), io.do_("do_lift_motor_down", false));
        let jog = if up && !dn { 1 } else if dn && !up { -1 } else { self.jog };
        let mut dir = 0.0;
        let mut vdes = 0.0;
        let vmax = sp.speed * if self.load.is_empty() { 1.0 } else { sp.loaded_factor };
        if jog != 0 { dir = jog as f64; vdes = dir * vmax; self.target = None; }
        else if let Some(t) = self.target {
            let d = t - self.h;
            if d.abs() < 0.001 { self.target = None; } else { dir = d.signum(); vdes = dir * vmax.min((2.0 * sp.accel * d.abs()).sqrt()).max(0.02); }
        }
        if brake { vdes = 0.0; }
        let dv = sp.accel * dt;
        self.v += (vdes - self.v).clamp(-dv, dv);
        if self.v.abs() > 1e-6 {
            let mut nh = (self.h + self.v * dt).clamp(0.0, sp.mast_max);
            if (nh - self.h).abs() < 1e-9 { self.v = 0.0; }
            else if self.load.is_empty() {
                match self.tines_hit(world, x, y, th, nh, &segs) {
                    Some((i, true)) if self.v > 0.0 && i != usize::MAX => { self.attach(world, i, x, y, th); }
                    Some((i, _)) => {
                        self.v = 0.0; nh = self.h;
                        self.blocked_n += 1;
                        let what = if i == usize::MAX { "墙/货架".to_string() } else { world.obstacles[i].get("id").and_then(|v| v.as_str()).map(|s| s.to_string()).unwrap_or_else(|| format!("障碍物#{i}")) };
                        if self.blocked_reason != what { self.ev("FORK_BLOCKED", "warning", format!("货叉升降受阻: {what}"), json!({"with": what, "height": round3(self.h)})); }
                        self.blocked_reason = what;
                        self.target = None;
                    }
                    None => { self.blocked_reason.clear(); }
                }
            }
            if !self.load.is_empty() {
                let ph = self.root.as_ref().and_then(|r| Self::idx_of(world, r)).map(|i| onum(&world.obstacles[i], "pocket_h", 0.1)).unwrap_or(0.1);
                let stack: Vec<String> = self.load.iter().map(|l| l.id.clone()).collect();
                let ri = self.root.as_ref().and_then(|r| Self::idx_of(world, r));
                let z0n = nh + sp.tine_t - ph;
                let sup = ri.map(|i| { let o = &world.obstacles[i]; Self::support_under(world, &stack, onum(o, "x", 0.0), onum(o, "y", 0.0), self.h + sp.tine_t - ph).0 }).unwrap_or(0.0);
                if self.v < 0.0 && z0n <= sup + 1e-4 {
                    // set it down, tines keep going (they were inside the pockets)
                    self.h = (sup - sp.tine_t + ph).max(0.0) + 0.0;
                    self.sync_load(world, x, y, th);
                    self.release(world);
                    nh = self.h;
                } else {
                    self.h = nh;
                }
            }
            self.h = nh;
        }
        if !self.load.is_empty() && ((x - self.last_pose[0]).abs() > 1e-6 || (y - self.last_pose[1]).abs() > 1e-6 || (th - self.last_pose[2]).abs() > 1e-6 || self.v.abs() > 1e-6) {
            self.sync_load(world, x, y, th);
        }
        self.last_pose = [x, y, th];
        // IO
        io.lift_height_m = self.h;
        io.set_di_raw("di_lift_top", self.h >= sp.mast_max - 0.005);
        io.set_di_raw("di_lift_bottom", self.h <= 0.005);
        io.set_di_raw("di_cargo_present", !self.load.is_empty());
        let tp = self.tine_polys(x, y, th);
        let tip = world.obstacles.iter().any(|o| is_cargo(o) && onum(o, "pocket_h", 0.0) > 0.0 && !self.is_carried(ostr(o, "id")) && {
            let r = obst_rect(o);
            let (s, c) = th.sin_cos();
            let (lx, ly) = (sp.x + sp.tine_len + 0.15, 0.0);
            let probe = rect(x + c * lx - s * ly, y + s * lx + c * ly, 0.1, sp.spacing / 2.0, th);
            sat(&probe, &r) || tp.iter().any(|t| sat(t, &r))
        });
        io.set_di_raw("di_pallet_detect", tip);
    }

    pub fn take_events(&mut self) -> Vec<ForkEvent> { std::mem::take(&mut self.events) }

    pub fn view(&self, world: &World) -> Value {
        let load: Vec<Value> = self.load.iter().filter_map(|l| Self::idx_of(world, &l.id).map(|i| Value::Object(world.obstacles[i].clone()))).collect();
        json!({"name": self.sp.name, "height_m": round3(self.h), "velocity_mps": round3(self.v), "target_m": self.target, "mast_max_m": self.sp.mast_max,
            "state": if self.blocked_reason.is_empty() { if self.v.abs() > 1e-3 { if self.v > 0.0 { "LIFTING" } else { "LOWERING" } } else { "IDLE" } } else { "BLOCKED" },
            "blocked": self.blocked_reason, "blocked_count": self.blocked_n, "loaded": !self.load.is_empty(), "load_id": self.root,
            "load": load, "tine": {"length": self.sp.tine_len, "width": self.sp.tine_w, "thickness": self.sp.tine_t, "spacing": self.sp.spacing, "root_x": self.sp.x},
            "limits": {"top": self.h >= self.sp.mast_max - 0.005, "bottom": self.h <= 0.005}})
    }
}

/// Build a cargo obstacle map.
pub fn make_cargo(kind: &str, id: &str, x: f64, y: f64, yaw: f64, z0: f64, size: Option<[f64; 3]>) -> Map<String, Value> {
    let (w, h, z, pocket, typ) = match kind {
        "pallet" | "euro_pallet" => { let s = size.unwrap_or([1.2, 0.8, 0.144]); (s[0], s[1], s[2], 0.10, "pallet") }
        _ => { let s = size.unwrap_or([0.6, 0.4, 0.4]); (s[0], s[1], s[2], 0.0, "box") }
    };
    let mut m = Map::new();
    m.insert("cargo".into(), json!(true)); m.insert("id".into(), json!(id)); m.insert("kind".into(), json!(kind)); m.insert("type".into(), json!(typ));
    m.insert("x".into(), json!(x)); m.insert("y".into(), json!(y)); m.insert("yaw".into(), json!(yaw));
    m.insert("w".into(), json!(w)); m.insert("h".into(), json!(h)); m.insert("z".into(), json!(z)); m.insert("z0".into(), json!(z0));
    if pocket > 0.0 { m.insert("pocket_h".into(), json!(pocket)); }
    m
}
