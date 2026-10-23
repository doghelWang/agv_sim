//! Sensor models: 2D/3D lidar configs + post-processing, photoelectric, bumpers, odometry, IMU, slip, code readers.
use crate::geom::{se2_integrate, wrap, Seg};
use crate::kin::{Chassis, Kind};
use crate::rng::Rng;
use serde_json::{json, Value};
use std::f64::consts::PI;

fn f(v: &Value, k: &str, d: f64) -> f64 { v.get(k).and_then(|x| x.as_f64()).unwrap_or(d) }
fn s<'a>(v: &'a Value, k: &str) -> Option<&'a str> { v.get(k).and_then(|x| x.as_str()) }

// ---------------------------------------------------------------- 2D lidar
#[derive(Clone, Debug)]
pub struct Lidar2D {
    pub cfg: Value,
    pub name: String,
    pub frame_id: String,
    pub mx: f64, pub my: f64, pub mz: f64, pub yaw: f64,
    pub sign: f64,
    pub native_res: f64, pub fov: f64,
    pub range_min: f64, pub native_range_max: f64, pub range_max: f64,
    pub freq_hz: f64,
    pub noise_std: f64, pub noise_prop: f64, pub dropout: f64,
    pub angle_min: f64, pub angle_max: f64, pub angle_inc: f64, pub n: usize,
}

impl Lidar2D {
    pub fn new(cfg: &Value) -> Lidar2D {
        let (roll, pitch) = (f(cfg, "roll", 0.0), f(cfg, "pitch", 0.0));
        let name = s(cfg, "name").unwrap_or("lidar").to_string();
        let mut l = Lidar2D {
            cfg: cfg.clone(), frame_id: format!("{name}_link"), name,
            mx: f(cfg, "x", 0.0), my: f(cfg, "y", 0.0), mz: f(cfg, "z", 0.3), yaw: f(cfg, "yaw", 0.0),
            sign: if roll.cos() * pitch.cos() >= 0.0 { 1.0 } else { -1.0 },
            native_res: f(cfg, "resolution_deg", 0.25).to_radians(), fov: f(cfg, "fov_deg", 270.0).to_radians(),
            range_min: f(cfg, "min_range", 0.05), native_range_max: f(cfg, "max_range", 25.0), range_max: f(cfg, "max_range", 25.0),
            freq_hz: f(cfg, "freq_hz", 15.0), noise_std: f(cfg, "range_noise_std", 0.015), noise_prop: 0.002, dropout: 0.002,
            angle_min: 0.0, angle_max: 0.0, angle_inc: 0.0, n: 0,
        };
        l.set_resolution(l.native_res);
        l
    }
    pub fn set_resolution(&mut self, res_rad: f64) {
        let res_rad = res_rad.max(0.05f64.to_radians());
        let full = self.fov >= 2.0 * PI - 1e-3;
        if full {
            let n = (2.0 * PI / res_rad).round() as usize;
            self.angle_min = -PI;
            self.angle_inc = 2.0 * PI / n as f64;
            self.angle_max = self.angle_min + (n as f64 - 1.0) * self.angle_inc;
            self.n = n;
        } else {
            let n = (self.fov / res_rad).round() as usize + 1;
            self.angle_min = -self.fov / 2.0;
            self.angle_max = self.fov / 2.0;
            self.angle_inc = self.fov / (n as f64 - 1.0);
            self.n = n;
        }
    }
}

/// range noise / dropout / min clamp (simcore.c sc_lidar_post)
pub fn lidar_post(out: &mut [f64], rmin: f64, noise: bool, std: f64, prop: f64, dropout: f64, rng: &mut Rng) {
    if noise {
        for v in out.iter_mut() { if v.is_finite() { *v += rng.gauss() * (std + prop * *v); } }
        if dropout > 0.0 { for v in out.iter_mut() { if rng.uniform() < dropout { *v = f64::INFINITY; } } }
    }
    for v in out.iter_mut() { if *v < rmin { *v = rmin; } }
}

pub fn merge_add(bins: &mut [f64], r: &[f64], mx: f64, my: f64, yaw: f64, sign: f64, a0: f64, inc: f64) {
    let nb = bins.len();
    let binw = 2.0 * PI / nb as f64;
    for (i, &rv) in r.iter().enumerate() {
        if !rv.is_finite() { continue; }
        let a = yaw + sign * (a0 + i as f64 * inc);
        let (px, py) = (mx + rv * a.cos(), my + rv * a.sin());
        let d = px.hypot(py);
        let mut k = ((py.atan2(px) + PI) / binw) as i64;
        if k < 0 { k = 0; }
        if k > nb as i64 - 1 { k = nb as i64 - 1; }
        if d < bins[k as usize] { bins[k as usize] = d; }
    }
}
pub fn merge_finish(bins: &mut [f64], rmax: f64) { for b in bins.iter_mut() { if *b > rmax { *b = f64::INFINITY; } } }

// ---------------------------------------------------------------- 3D lidar
#[derive(Clone, Debug)]
pub struct Lidar3D {
    pub cfg: Value,
    pub name: String,
    pub frame_id: String,
    pub mx: f64, pub my: f64, pub mz: f64,
    pub r: [f64; 9],
    pub vmin: f64, pub vmax: f64, pub range_min: f64, pub range_max: f64,
    pub freq_hz: f64, pub noise_std: f64, pub ang_noise: f64,
    pub lines: i32, pub real_points: i64, pub n: usize, pub frame: f64,
}

pub fn rot_rpy(roll: f64, pitch: f64, yaw: f64) -> [f64; 9] {
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
     sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
     -sp, cp * sr, cp * cr]
}

impl Lidar3D {
    pub fn new(cfg: &Value, points_per_frame: usize) -> Lidar3D {
        let name = s(cfg, "name").unwrap_or("lidar3d").to_string();
        let freq = f(cfg, "freq_hz", 10.0);
        Lidar3D {
            cfg: cfg.clone(), frame_id: format!("{name}_link"), name,
            mx: f(cfg, "x", 0.0), my: f(cfg, "y", 0.0), mz: f(cfg, "z", 0.3),
            r: rot_rpy(f(cfg, "roll", 0.0), f(cfg, "pitch", 0.0), f(cfg, "yaw", 0.0)),
            vmin: f(cfg, "vfov_min_deg", -7.0).to_radians(), vmax: f(cfg, "vfov_max_deg", 52.0).to_radians(),
            range_min: f(cfg, "min_range", 0.1), range_max: f(cfg, "max_range", 40.0),
            freq_hz: freq, noise_std: f(cfg, "range_noise_std", 0.02), ang_noise: f(cfg, "angular_noise_deg", 0.15).to_radians(),
            lines: f(cfg, "lines", 4.0) as i32, real_points: (f(cfg, "point_rate", 200000.0) / freq.max(1.0)) as i64,
            n: points_per_frame, frame: 0.0,
        }
    }
    /// sensor-frame points -> base frame
    pub fn points_base(&self, p: [f64; 3]) -> [f64; 3] {
        let r = &self.r;
        [r[0] * p[0] + r[1] * p[1] + r[2] * p[2] + self.mx, r[3] * p[0] + r[4] * p[1] + r[5] * p[2] + self.my, r[6] * p[0] + r[7] * p[1] + r[8] * p[2] + self.mz]
    }
}

// ---------------------------------------------------------------- photoelectric / bumper
#[derive(Clone, Debug)]
pub struct Photo {
    pub cfg: Value, pub name: String, pub di: String, pub x: f64, pub y: f64, pub z: f64, pub yaw: f64,
    pub range_m: f64, pub trigger_m: f64, pub hyst: f64, pub half: f64, pub kind: String,
    pub detected: bool, pub distance: f64,
}
impl Photo {
    pub fn new(cfg: &Value) -> Photo {
        let name = s(cfg, "name").unwrap_or("pe").to_string();
        Photo { cfg: cfg.clone(), di: s(cfg, "di").map(|d| d.to_string()).unwrap_or_else(|| format!("di_pe_{name}")), name,
            x: f(cfg, "x", 0.0), y: f(cfg, "y", 0.0), z: f(cfg, "z", 0.15), yaw: f(cfg, "yaw", 0.0),
            range_m: f(cfg, "range_m", 1.0), trigger_m: f(cfg, "trigger_m", 0.4), hyst: f(cfg, "hysteresis_m", 0.02),
            half: f(cfg, "beam_half_deg", 2.0).to_radians(), kind: s(cfg, "kind").unwrap_or("diffuse").to_string(), detected: false, distance: f64::INFINITY }
    }
    pub fn view(&self) -> Value {
        json!({"name": self.name, "di": self.di, "kind": self.kind, "detected": self.detected,
            "distance_m": if self.distance.is_finite() { json!((self.distance * 1000.0).round() / 1000.0) } else { Value::Null },
            "trigger_m": self.trigger_m, "range_m": self.range_m,
            "mount": {"x": self.x, "y": self.y, "z": self.z, "yaw": (self.yaw * 10000.0).round() / 10000.0},
            "source": s(&self.cfg, "source").unwrap_or("")})
    }
}

#[derive(Clone, Debug)]
pub struct Bumper {
    pub name: String, pub side: u8, pub side_name: String, pub poly: Vec<[f64; 2]>, pub di: String, pub hold_s: f64,
    pub pressed: bool, pub contact_t: f64, pub press_count: i32,
}
impl Bumper {
    pub fn view(&self) -> Value {
        json!({"name": self.name, "side": self.side_name, "di": self.di, "pressed": self.pressed, "press_count": self.press_count,
            "polygon": self.poly.iter().map(|p| json!([p[0], p[1]])).collect::<Vec<_>>()})
    }
}

fn bbox(fp: &[[f64; 2]]) -> (f64, f64, f64, f64) {
    let xmax = fp.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
    let xmin = fp.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
    let ymax = fp.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max);
    let ymin = fp.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
    (xmax, xmin, ymax, ymin)
}

pub fn footprint_of(spec: &Value) -> Vec<[f64; 2]> {
    spec["chassis"]["footprint"].as_array().map(|a| a.iter().map(|p| [p[0].as_f64().unwrap_or(0.0), p[1].as_f64().unwrap_or(0.0)]).collect()).unwrap_or_default()
}

pub fn build_photos(spec: &Value) -> Vec<Photo> {
    let (xmax, xmin, ymax, ymin) = bbox(&footprint_of(spec));
    let mut cfgs: Vec<Value> = vec![];
    if let Some(a) = spec.get("photoelectric").and_then(|a| a.as_array()) {
        for d in a {
            let mut d = d.clone();
            if d.get("source").is_none() { d["source"] = json!("cmodel"); }
            cfgs.push(d);
        }
    }
    if cfgs.is_empty() {
        if let Some(inp) = spec.get("io").and_then(|io| io.get("inputs")).and_then(|i| i.as_array()) {
            for io in inp {
                let t = s(io, "type").unwrap_or("");
                if ["photoelectric", "PT", "proximitySensor"].contains(&t) && io.get("x").is_some() {
                    cfgs.push(json!({"name": io["name"], "x": io["x"], "y": io["y"], "z": f(io, "z", 0.15), "yaw": f(io, "yaw", 0.0),
                        "trigger_m": if t == "proximitySensor" { 0.3 } else { 0.5 }, "source": format!("cmodel:{t}")}));
                }
            }
        }
    }
    if cfgs.is_empty() {
        let z = 0.12;
        for (nm, px, py, yaw) in [("front_left", xmax, ymax, 30f64.to_radians()), ("front_right", xmax, ymin, -30f64.to_radians()),
                                  ("rear_left", xmin, ymax, 150f64.to_radians()), ("rear_right", xmin, ymin, -150f64.to_radians())] {
            cfgs.push(json!({"name": nm, "x": px, "y": py, "z": z, "yaw": yaw, "trigger_m": 0.35, "range_m": 1.0, "source": "default: 角部避障光电"}));
        }
    }
    cfgs.iter().map(Photo::new).collect()
}

pub fn build_bumpers(spec: &Value) -> Vec<Bumper> {
    let (xmax, xmin, ymax, ymin) = bbox(&footprint_of(spec));
    let (inset, t) = (0.02, 0.03);
    let mk = |name: &str, side: u8, poly: Vec<[f64; 2]>| Bumper { name: name.into(), side, side_name: name.into(), poly, di: format!("di_bumper_{name}"), hold_s: 0.5, pressed: false, contact_t: -1e9, press_count: 0 };
    let mut v = vec![
        mk("front", 0, vec![[xmax, ymax - inset], [xmax + t, ymax - inset], [xmax + t, ymin + inset], [xmax, ymin + inset]]),
        mk("rear", 1, vec![[xmin - t, ymax - inset], [xmin, ymax - inset], [xmin, ymin + inset], [xmin - t, ymin + inset]]),
    ];
    if spec.get("bumpers").and_then(|b| b.get("sides")).map(|x| x.as_bool().unwrap_or(!x.is_null() && x != &json!(0))).unwrap_or(false) {
        v.push(mk("left", 2, vec![[xmin + inset, ymax], [xmax - inset, ymax], [xmax - inset, ymax + t], [xmin + inset, ymax + t]]));
        v.push(mk("right", 3, vec![[xmin + inset, ymin - t], [xmax - inset, ymin - t], [xmax - inset, ymin], [xmin + inset, ymin]]));
    }
    v
}

// ---------------------------------------------------------------- odometry / IMU / slip
#[derive(Clone, Debug)]
pub struct Odometry {
    pub enabled: bool,
    pub radius_err: Vec<f64>, pub steer_bias: Vec<f64>,
    pub cpr: f64, pub steer_sigma: f64,
    pub x: f64, pub y: f64, pub th: f64, pub vx: f64, pub vy: f64, pub wz: f64,
}
impl Odometry {
    pub fn new(ch: &Chassis, enabled: bool, rng: &mut Rng) -> Odometry {
        let radius_err = ch.wheels.iter().map(|_| if enabled { 1.0 + rng.gauss() * 0.001 } else { 1.0 }).collect();
        let steer_bias = ch.wheels.iter().map(|_| if enabled { (rng.gauss() * 0.08).to_radians() } else { 0.0 }).collect();
        Odometry { enabled, radius_err, steer_bias, cpr: 4096.0 * 4.0, steer_sigma: 0.05f64.to_radians(), x: 0.0, y: 0.0, th: 0.0, vx: 0.0, vy: 0.0, wz: 0.0 }
    }
    pub fn reset(&mut self, x: f64, y: f64, th: f64) { self.x = x; self.y = y; self.th = th; self.vx = 0.0; self.vy = 0.0; self.wz = 0.0; }

    /// noisy encoder odometry (simcore.c sc_odom_update)
    pub fn update_noisy(&mut self, ch: &Chassis, dt: f64, rng: &mut Rng) {
        let n = ch.wheels.len().min(64);
        let mut hov = vec![f64::NAN; ch.wheels.len()];
        let mut sov = vec![f64::NAN; ch.wheels.len()];
        for i in 0..n {
            let w = &ch.wheels[i];
            if !w.kind.driven() { continue; }
            let tpm = self.cpr * w.gear_ratio / (2.0 * PI * w.radius);
            let q = round_half_even(w.speed * dt * tpm) / (tpm * dt).max(1e-9);
            sov[i] = q * self.radius_err[i];
            if w.kind == Kind::Steer {
                hov[i] = w.steer + self.steer_bias[i] + if self.steer_sigma > 0.0 { rng.gauss() * self.steer_sigma } else { 0.0 };
            }
        }
        let (vx, vy, wz, _, _) = ch.forward(Some(&hov), Some(&sov));
        let (x, y, th) = se2_integrate(self.x, self.y, self.th, vx, vy, wz, dt);
        self.x = x; self.y = y; self.th = th; self.vx = vx; self.vy = vy; self.wz = wz;
    }
    pub fn update_ideal(&mut self, ch: &Chassis, dt: f64) {
        let (x, y, th) = se2_integrate(self.x, self.y, self.th, ch.vx, ch.vy, ch.wz, dt);
        self.x = x; self.y = y; self.th = th; self.vx = ch.vx; self.vy = ch.vy; self.wz = ch.wz;
    }
}

pub fn round_half_even(x: f64) -> f64 {
    let r = x.round();
    if (x - x.trunc()).abs() == 0.5 { 2.0 * (x / 2.0).round() } else { r }
}

#[derive(Clone, Debug, Default)]
pub struct ImuState { pub prev: [f64; 2], pub gyro_bias: f64, pub yaw: f64, pub bias_walk: f64, pub gyro_noise: f64, pub acc_noise: f64 }
#[derive(Clone, Debug)]
pub struct ImuSample { pub wz: f64, pub ax: f64, pub ay: f64, pub az: f64, pub yaw: f64 }

impl ImuState {
    pub fn new(enabled: bool, rng: &mut Rng) -> ImuState {
        ImuState { prev: [0.0; 2], gyro_bias: if enabled { rng.gauss() * 0.05f64.to_radians() } else { 0.0 }, yaw: 0.0,
            bias_walk: if enabled { 0.001f64.to_radians() } else { 0.0 }, gyro_noise: if enabled { 0.02f64.to_radians() } else { 0.0 }, acc_noise: if enabled { 0.02 } else { 0.0 } }
    }
    pub fn sample(&mut self, vx: f64, vy: f64, wz: f64, dt: f64, rng: &mut Rng) -> ImuSample {
        let k = dt.max(1e-3);
        let ax = (vx - self.prev[0]) / k - wz * vy;
        let ay = (vy - self.prev[1]) / k + wz * vx;
        self.prev = [vx, vy];
        if self.bias_walk > 0.0 { self.gyro_bias += rng.gauss() * self.bias_walk; }
        let gz = wz + self.gyro_bias + if self.gyro_noise > 0.0 { rng.gauss() * self.gyro_noise } else { 0.0 };
        self.yaw = wrap(self.yaw + gz * dt);
        let an = self.acc_noise;
        let ax = ax + if an > 0.0 { rng.gauss() * an } else { 0.0 };
        let ay = ay + if an > 0.0 { rng.gauss() * an } else { 0.0 };
        let az = 9.81 + if an > 0.0 { rng.gauss() * an } else { 0.0 };
        ImuSample { wz: gz, ax, ay, az, yaw: self.yaw }
    }
}

pub fn slip(prev: &mut [f64; 3], v: &mut [f64; 3], dt: f64, rng: &mut Rng) {
    let acc = (v[0] - prev[0]).hypot(v[1] - prev[1]) / dt.max(1e-3);
    *prev = *v;
    let k = 1.0 - (0.004 + 0.01 * acc).min(0.03) - rng.gauss() * 0.002;
    let kw = 1.0 - (0.01 * v[2].abs()).min(0.05) - rng.gauss() * 0.002;
    v[0] *= k; v[1] *= k; v[2] *= kw;
}

// ---------------------------------------------------------------- code readers (ground QR / station tags)
fn r(v: f64, nd: i32) -> f64 { let m = 10f64.powi(nd); (v * m).round() / m }

pub fn detect_ground_codes(cams: &[Value], x: f64, y: f64, th: f64, ground_tags: &[Value], rng: &mut Rng) -> Vec<Value> {
    let (sn, cs) = th.sin_cos();
    let mut out = vec![];
    for cam in cams {
        let (cx, cy) = (x + cs * f(cam, "x", 0.0) - sn * f(cam, "y", 0.0), y + sn * f(cam, "x", 0.0) + cs * f(cam, "y", 0.0));
        if !s(cam, "orientation").unwrap_or("").ends_with("DOWN") { continue; }
        let win = f(cam, "read_window_m", 0.05);
        for t in ground_tags {
            let (dx, dy) = (f(t, "x", 0.0) - cx, f(t, "y", 0.0) - cy);
            if dx.abs() <= win && dy.abs() <= win {
                let (rx, ry) = (cs * dx + sn * dy, -sn * dx + cs * dy);
                out.push(json!({"camera": cam["name"], "type": "ground_qr", "id": t["id"], "name": t.get("name").unwrap_or(&t["id"]),
                    "rel_x": r(rx + rng.gauss() * 0.0015, 4), "rel_y": r(ry + rng.gauss() * 0.0015, 4),
                    "rel_yaw_deg": r(wrap(f(t, "yaw", 0.0) - th).to_degrees() + rng.gauss() * 0.1, 2),
                    "distance_m": r(dx.hypot(dy), 4), "bearing_deg": 0.0}));
            }
        }
    }
    out
}

pub fn detect_station_tags(x: f64, y: f64, th: f64, mount_x: f64, tags: &[Value]) -> Vec<Value> {
    let (fov, max_dist) = (85f64.to_radians(), 4.0);
    let (ox, oy) = (x + th.cos() * mount_x, y + th.sin() * mount_x);
    let mut out = vec![];
    for m in tags {
        let (dx, dy) = (f(m, "x", 0.0) - ox, f(m, "y", 0.0) - oy);
        let d = dx.hypot(dy);
        if d > max_dist { continue; }
        let b = wrap(dy.atan2(dx) - th);
        if b.abs() <= fov / 2.0 {
            out.push(json!({"id": m["id"], "name": m["name"], "type": "station_tag", "distance_m": r(d, 2), "bearing_deg": r(b.to_degrees(), 1),
                "rel_x": r((-th).cos() * dx - (-th).sin() * dy, 3), "rel_y": r((-th).sin() * dx + (-th).cos() * dy, 3),
                "rel_yaw_deg": r(wrap(f(m, "yaw", 0.0) - th).to_degrees(), 1)}));
        }
    }
    out
}

pub type SegVec = Vec<Seg>;
