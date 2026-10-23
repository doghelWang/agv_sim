//! Wheeled-chassis kinematics + actuator dynamics (port of simcore.c sc_kin_step / sc_forward / spec builders).
use crate::geom::wrap;
use serde_json::Value;
use std::f64::consts::PI;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Kind { Drive, Steer, Fixed, Caster }
impl Kind {
    pub fn parse(s: &str) -> Kind { match s { "drive" => Kind::Drive, "steer" => Kind::Steer, "fixed" => Kind::Fixed, _ => Kind::Caster } }
    pub fn name(self) -> &'static str { match self { Kind::Drive => "drive", Kind::Steer => "steer", Kind::Fixed => "fixed", Kind::Caster => "caster" } }
    pub fn driven(self) -> bool { matches!(self, Kind::Drive | Kind::Steer) }
    pub fn constrained(self) -> bool { matches!(self, Kind::Drive | Kind::Steer | Kind::Fixed) }
}

#[derive(Clone, Debug)]
pub struct Wheel {
    pub name: String,
    pub kind: Kind,
    pub x: f64, pub y: f64, pub radius: f64,
    pub steer_min: f64, pub steer_max: f64, pub steer_rate: f64,
    pub max_speed: f64, pub drive_tau: f64, pub gear_ratio: f64,
    pub steer: f64, pub steer_target: f64, pub speed: f64, pub speed_target: f64, pub cmd_speed: f64, pub angle: f64,
    pub motor_rpm: f64, pub current_a: f64, pub torque_nm: f64, pub steer_current_a: f64,
}

impl Wheel {
    pub fn new(name: &str, kind: Kind, x: f64, y: f64, radius: f64) -> Wheel {
        Wheel { name: name.into(), kind, x, y, radius, steer_min: -PI / 2.0, steer_max: PI / 2.0, steer_rate: 90f64.to_radians(),
            max_speed: 2.0, drive_tau: 0.06, gear_ratio: 20.0, steer: 0.0, steer_target: 0.0, speed: 0.0, speed_target: 0.0,
            cmd_speed: 0.0, angle: 0.0, motor_rpm: 0.0, current_a: 0.0, torque_nm: 0.0, steer_current_a: 0.0 }
    }
    pub fn heading(&self) -> f64 { if self.kind == Kind::Steer { self.steer } else { 0.0 } }
}

#[derive(Clone, Debug)]
pub struct Limits {
    pub max_speed: f64, pub max_accel: f64, pub max_decel: f64,
    pub max_ang_speed: f64, pub max_ang_accel: f64, pub max_ang_decel: f64, pub mass: f64,
}
impl Default for Limits {
    fn default() -> Self { Limits { max_speed: 1.2, max_accel: 0.5, max_decel: 0.5, max_ang_speed: 1.0, max_ang_accel: 1.0, max_ang_decel: 1.0, mass: 200.0 } }
}

#[derive(Clone, Debug)]
pub struct Chassis {
    pub kind: String,
    pub wheels: Vec<Wheel>,
    pub lim: Limits,
    pub holonomic: bool,
    pub axle_x: f64,
    pub kt: f64, pub rolling_coeff: f64, pub efficiency: f64,
    pub shaped: [f64; 3],
    pub vx: f64, pub vy: f64, pub wz: f64,
    pub slip_residual: f64, pub saturation: f64,
}

/// Weighted least squares (m x 3) via one-sided Jacobi SVD; same rank rule as numpy lstsq (rcond=None).
fn lstsq3(a_in: &[[f64; 3]], b: &[f64]) -> ([f64; 3], f64, i32) {
    let m = a_in.len();
    let mut u: Vec<[f64; 3]> = a_in.to_vec();
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _sweep in 0..60 {
        let mut off = 0.0f64;
        for p in 0..2 {
            for q in (p + 1)..3 {
                let (mut alpha, mut beta, mut gamma) = (0.0, 0.0, 0.0);
                for r in u.iter() { alpha += r[p] * r[p]; beta += r[q] * r[q]; gamma += r[p] * r[q]; }
                if gamma == 0.0 { continue; }
                let conv = gamma.abs() / (alpha * beta).sqrt();
                if !(conv > 1e-15) { continue; }
                if conv > off { off = conv; }
                let zeta = (beta - alpha) / (2.0 * gamma);
                let t = 1f64.copysign(zeta) / (zeta.abs() + (1.0 + zeta * zeta).sqrt());
                let cs = 1.0 / (1.0 + t * t).sqrt();
                let sn = cs * t;
                for r in u.iter_mut() { let (up, uq) = (r[p], r[q]); r[p] = cs * up - sn * uq; r[q] = sn * up + cs * uq; }
                for r in v.iter_mut() { let (vp, vq) = (r[p], r[q]); r[p] = cs * vp - sn * vq; r[q] = sn * vp + cs * vq; }
            }
        }
        if off <= 1e-15 { break; }
    }
    let mut sig = [0.0f64; 3];
    let mut smax = 0.0f64;
    for k in 0..3 {
        let s2: f64 = u.iter().map(|r| r[k] * r[k]).sum();
        sig[k] = s2.sqrt();
        if sig[k] > smax { smax = sig[k]; }
    }
    let tol = 2.220446049250313e-16 * (m.max(3) as f64) * smax;
    let mut rank = 0;
    let mut x = [0.0f64; 3];
    for k in 0..3 {
        if !(sig[k] > tol) { continue; }
        rank += 1;
        let utb: f64 = (0..m).map(|i| u[i][k] * b[i]).sum();
        let coef = utb / (sig[k] * sig[k]);
        for r in 0..3 { x[r] += v[r][k] * coef; }
    }
    let mut ss = 0.0;
    for i in 0..m {
        let e = a_in[i][0] * x[0] + a_in[i][1] * x[1] + a_in[i][2] * x[2] - b[i];
        ss += e * e;
    }
    (x, (ss / m as f64).sqrt(), rank)
}

fn ramp(cur: f64, tgt: f64, acc: f64, dec: f64, dt: f64) -> f64 {
    let lim = if tgt.abs() > cur.abs() && tgt * cur >= 0.0 { acc * dt } else { dec * dt };
    let d = tgt - cur;
    if d.abs() <= lim { tgt } else { cur + lim.copysign(d) }
}

const GATE_START: f64 = 10.0 * PI / 180.0;
const GATE_STOP: f64 = 45.0 * PI / 180.0;

fn best_steer(w: &Wheel, ang: f64, spd: f64) -> (f64, f64) {
    let ca = [ang, wrap(ang + PI), wrap(ang - PI)];
    let cs = [spd, -spd, -spd];
    let mut best: Option<(f64, f64, f64)> = None;
    for k in 0..3 {
        let a = ca[k];
        if !(w.steer_min - 1e-6 <= a && a <= w.steer_max + 1e-6) { continue; }
        let e = (a - w.steer).abs();
        let better = match best { None => true, Some((be, ba, bs)) => e < be || (e == be && (a < ba || (a == ba && cs[k] < bs))) };
        if better { best = Some((e, a, cs[k])); }
    }
    match best {
        None => { let a = ang.clamp(w.steer_min, w.steer_max); (a, spd * (ang - a).cos()) }
        Some((_, a, s)) => (a, s),
    }
}

impl Chassis {
    pub fn new(kind: &str, wheels: Vec<Wheel>, lim: Limits) -> Chassis {
        let fixed: Vec<&Wheel> = wheels.iter().filter(|w| matches!(w.kind, Kind::Drive | Kind::Fixed)).collect();
        let n_steer = wheels.iter().filter(|w| w.kind == Kind::Steer).count();
        let holonomic = fixed.is_empty() && n_steer >= 2;
        let axle_x = if fixed.is_empty() { 0.0 } else { fixed.iter().map(|w| w.x).sum::<f64>() / fixed.len() as f64 };
        Chassis { kind: kind.into(), wheels, lim, holonomic, axle_x, kt: 0.12, rolling_coeff: 0.012, efficiency: 0.85,
            shaped: [0.0; 3], vx: 0.0, vy: 0.0, wz: 0.0, slip_residual: 0.0, saturation: 1.0 }
    }

    /// Forward kinematics. `heading_ovr` / `speed_ovr`: NaN = no override. Returns (vx,vy,wz,resid,rank) (rank -1 = no constraints)
    pub fn forward(&self, heading_ovr: Option<&[f64]>, speed_ovr: Option<&[f64]>) -> (f64, f64, f64, f64, i32) {
        let mut a: Vec<[f64; 3]> = Vec::with_capacity(self.wheels.len() * 2);
        let mut b: Vec<f64> = Vec::with_capacity(self.wheels.len() * 2);
        for (i, w) in self.wheels.iter().enumerate() {
            if a.len() + 2 > 64 { break; }
            if !w.kind.constrained() { continue; }
            let mut ang = if w.kind == Kind::Steer { w.steer } else { 0.0 };
            if let Some(h) = heading_ovr { if !h[i].is_nan() { ang = h[i]; } }
            let (s, c) = ang.sin_cos();
            if w.kind.driven() {
                let mut spd = w.speed;
                if let Some(sp) = speed_ovr { if !sp[i].is_nan() { spd = sp[i]; } }
                a.push([c, s, -w.y * c + w.x * s]); b.push(spd);
            }
            a.push([-s, c, w.y * s + w.x * c]); b.push(0.0);
        }
        if a.is_empty() { return (0.0, 0.0, 0.0, 0.0, -1); }
        let (sol, resid, rank) = lstsq3(&a, &b);
        let mut vy = sol[1];
        if !self.holonomic && rank >= 3 { vy = -sol[2] * self.axle_x; }
        (sol[0], vy, sol[2], resid, rank)
    }

    pub fn step(&mut self, mut cvx: f64, mut cvy: f64, mut cwz: f64, dt: f64, brake: bool) -> (f64, f64, f64) {
        if brake { cvx = 0.0; cvy = 0.0; cwz = 0.0; }
        if !self.holonomic { cvy = -cwz * self.axle_x; }
        let v = cvx.hypot(cvy);
        if v > self.lim.max_speed { let k = self.lim.max_speed / v; cvx *= k; cvy *= k; }
        cwz = cwz.clamp(-self.lim.max_ang_speed, self.lim.max_ang_speed);
        let l = &self.lim;
        let sx = ramp(self.shaped[0], cvx, l.max_accel, l.max_decel, dt);
        let mut sy = ramp(self.shaped[1], cvy, l.max_accel, l.max_decel, dt);
        let sw = ramp(self.shaped[2], cwz, l.max_ang_accel, l.max_ang_decel, dt);
        if !self.holonomic { sy = -sw * self.axle_x; }
        self.shaped = [sx, sy, sw];
        let (vx_c, vy_c, wz_c) = (sx, sy, sw);

        let n = self.wheels.len().min(64);
        let mut tang = [0.0f64; 64];
        let mut tspd = [0.0f64; 64];
        let mut has = [false; 64];
        let mut scale = 1.0f64;
        for i in 0..n {
            let w = &self.wheels[i];
            if !w.kind.driven() { continue; }
            let (wx, wy) = (vx_c - wz_c * w.y, vy_c + wz_c * w.x);
            let (mut spd, ang);
            if w.kind == Kind::Drive { spd = wx; ang = 0.0; } else {
                spd = wx.hypot(wy);
                if spd < 1e-4 { ang = w.steer_target; spd = 0.0; } else { let r = best_steer(w, wy.atan2(wx), spd); ang = r.0; spd = r.1; }
            }
            if spd.abs() > w.max_speed { let r = w.max_speed / spd.abs(); if r < scale { scale = r; } }
            tang[i] = ang; tspd[i] = spd; has[i] = true;
        }
        if scale < 1.0 { for i in 0..n { if has[i] { tspd[i] *= scale; } } }
        self.saturation = scale;
        let pure_rot = vx_c.hypot(vy_c) < 0.02 && wz_c.abs() > 1e-3;

        let accmax = l.max_accel.max(l.max_decel);
        for i in 0..n {
            if !has[i] { continue; }
            let w = &mut self.wheels[i];
            let (ang_t, mut spd_t) = (tang[i], tspd[i]);
            if w.kind == Kind::Steer {
                w.steer_target = ang_t;
                let err = ang_t - w.steer;
                let max_d = w.steer_rate * dt;
                if err.abs() > max_d { w.steer += max_d.copysign(err); w.steer_current_a = 4.5; } else { w.steer = ang_t; w.steer_current_a = 0.8; }
                w.steer = w.steer.max(w.steer_min).min(w.steer_max);
                let e = (w.steer_target - w.steer).abs();
                let g0 = if pure_rot { 2.0 * PI / 180.0 } else { GATE_START };
                let g1 = if pure_rot { 8.0 * PI / 180.0 } else { GATE_STOP };
                if e > g0 { let g = (1.0 - (e - g0) / (g1 - g0)).max(0.0); spd_t *= g; }
            }
            w.speed_target = if brake { 0.0 } else { spd_t };
            let alpha = dt / (w.drive_tau + dt);
            let mut dv = (w.speed_target - w.cmd_speed) * alpha;
            let acc_lim = accmax * 2.5 * dt;
            dv = dv.max(-acc_lim).min(acc_lim);
            w.cmd_speed += dv;
            if brake && w.cmd_speed.abs() < 0.01 { w.cmd_speed = 0.0; }
            w.speed = w.cmd_speed;
        }

        let (vx, vy, wz, resid, _) = self.forward(None, None);
        self.slip_residual = resid;
        let acc = if dt > 0.0 { (vx - self.vx) / dt } else { 0.0 };
        self.vx = vx; self.vy = vy; self.wz = wz;
        let n_drv = self.wheels.iter().filter(|w| w.kind.driven()).count().max(1) as f64;
        let (mass, kt, rc, eff) = (self.lim.mass, self.kt, self.rolling_coeff, self.efficiency);
        for w in self.wheels.iter_mut() {
            let heading = if w.kind == Kind::Steer { w.steer } else { 0.0 };
            if matches!(w.kind, Kind::Fixed | Kind::Caster) { w.speed = (vx - wz * w.y) * heading.cos() + (vy + wz * w.x) * heading.sin(); }
            let rad = w.radius.max(1e-3);
            w.angle += (w.speed / rad) * dt;
            if w.kind.driven() {
                let omega = w.speed / rad;
                w.motor_rpm = omega * w.gear_ratio * 60.0 / (2.0 * PI);
                let f = mass / n_drv * (acc.abs() + 9.81 * rc * if w.speed.abs() > 1e-3 { 1.0 } else { 0.0 });
                w.torque_nm = f * w.radius / w.gear_ratio.max(1.0) / eff;
                w.current_a = if w.speed_target.abs() > 1e-3 || w.speed.abs() > 1e-3 { 0.3 + w.torque_nm / kt } else { 0.05 };
            }
        }
        (vx, vy, wz)
    }

    pub fn reset(&mut self) {
        self.shaped = [0.0; 3]; self.vx = 0.0; self.vy = 0.0; self.wz = 0.0;
        for w in self.wheels.iter_mut() { w.speed = 0.0; w.speed_target = 0.0; w.cmd_speed = 0.0; }
    }
    pub fn stop_now(&mut self) { self.reset(); }

    pub fn telemetry(&self) -> serde_json::Value {
        use serde_json::{json, Map, Value};
        let r = |x: f64, n: i32| { let m = 10f64.powi(n); (x * m).round_ties_even() / m };
        let mut wheels = Map::new();
        for w in &self.wheels {
            if w.kind == Kind::Caster { continue; }
            let mut d = json!({"kind": w.kind.name(), "speed_mps": r(w.speed, 3), "rpm": r(w.motor_rpm, 1), "current_a": r(w.current_a, 2), "torque_nm": r(w.torque_nm, 2), "rad": r(w.angle, 2)});
            if w.kind == Kind::Steer {
                d["steer_angle_deg"] = json!(r(w.steer.to_degrees(), 1));
                d["target_steer_deg"] = json!(r(w.steer_target.to_degrees(), 1));
                d["steer_current_a"] = json!(r(w.steer_current_a, 2));
            }
            wheels.insert(w.name.clone(), d);
        }
        json!({"type": self.kind, "holonomic": self.holonomic, "saturation": r(self.saturation, 3), "slip_residual": r(self.slip_residual, 4), "wheels": Value::Object(wheels)})
    }

    pub fn joint_state(&self) -> (Vec<String>, Vec<f64>, Vec<f64>, Vec<f64>) {
        let (mut names, mut pos, mut vel, mut eff) = (vec![], vec![], vec![], vec![]);
        for w in &self.wheels {
            let r = w.radius.max(1e-3);
            match w.kind {
                Kind::Steer => {
                    names.push(format!("{}_steer_joint", w.name)); names.push(format!("{}_drive_joint", w.name));
                    pos.push(w.steer); pos.push(w.angle); vel.push(0.0); vel.push(w.speed / r); eff.push(0.0); eff.push(w.torque_nm);
                }
                Kind::Drive | Kind::Fixed => { names.push(format!("{}_joint", w.name)); pos.push(w.angle); vel.push(w.speed / r); eff.push(w.torque_nm); }
                Kind::Caster => {}
            }
        }
        (names, pos, vel, eff)
    }
}

// ------------------------------------------------------------ spec builders
fn f(v: &Value, k: &str, d: f64) -> f64 { v.get(k).and_then(|x| x.as_f64()).unwrap_or(d) }

pub fn limits_from_spec(spec: &Value) -> Limits {
    let ch = spec.get("chassis").cloned().unwrap_or(Value::Null);
    let acc = f(&ch, "max_accel_mps2", 0.5);
    let aacc = f(&ch, "max_ang_accel_radps2", 1.0);
    Limits { max_speed: f(&ch, "max_speed_mps", 1.2), max_accel: acc, max_decel: f(&ch, "max_decel_mps2", acc),
        max_ang_speed: f(&ch, "max_ang_speed_radps", 1.0), max_ang_accel: aacc, max_ang_decel: f(&ch, "max_ang_decel_radps2", aacc),
        mass: f(&ch, "mass_kg", 200.0) }
}

pub fn wheel_from_value(d: &Value) -> Wheel {
    let gear = d.get("drive_motor").and_then(|m| m.get("gear_ratio")).and_then(|g| g.as_f64()).filter(|g| *g != 0.0).unwrap_or(20.0);
    let mut w = Wheel::new(d["name"].as_str().unwrap_or(""), Kind::parse(d["kind"].as_str().unwrap_or("caster")), f(d, "x", 0.0), f(d, "y", 0.0), f(d, "radius_m", 0.1));
    w.steer_min = f(d, "steer_min_rad", -PI / 2.0);
    w.steer_max = f(d, "steer_max_rad", PI / 2.0);
    w.steer_rate = f(d, "steer_rate_radps", 90f64.to_radians());
    w.max_speed = f(d, "max_speed_mps", 3.0);
    w.gear_ratio = gear;
    w
}

pub fn preset_wheels(spec: &Value, chassis_type: &str) -> Result<Vec<Value>, String> {
    use serde_json::json;
    let ch = &spec["chassis"];
    let (head, tail, left, right) = (f(ch, "head_offset_m", 0.5), f(ch, "tail_offset_m", 0.5), f(ch, "left_offset_m", 0.4), f(ch, "right_offset_m", 0.4));
    let r = spec.get("drive_wheels").map(|d| f(d, "radius_m", 0.1)).unwrap_or(0.1);
    let v_cap = f(ch, "max_speed_mps", 1.5) * 1.05;
    let rate = 90f64.to_radians();
    let xc = (head - tail) / 2.0;
    let l = head + tail;
    let m = 0.12;
    match chassis_type {
        "diff_drive" => {
            let cr = r * 0.6;
            let mut ws = vec![json!({"name":"left_wheel","kind":"drive","x":0.0,"y":left - m - 0.05,"radius_m":r,"max_speed_mps":v_cap}),
                json!({"name":"right_wheel","kind":"drive","x":0.0,"y":-(right - m - 0.05),"radius_m":r,"max_speed_mps":v_cap})];
            for (tag, x) in [("front", head - m - cr), ("rear", -(tail - m - cr))] {
                for (side, y) in [("left", left - m - cr), ("right", -(right - m - cr))] {
                    if x.abs() > 0.2 { ws.push(json!({"name":format!("caster_{tag}_{side}"),"kind":"caster","x":x,"y":y,"radius_m":cr})); }
                }
            }
            Ok(ws)
        }
        "single_steer" => {
            let sx = (head - 0.2).max(0.4);
            Ok(vec![json!({"name":"steer_wheel","kind":"steer","x":sx,"y":0.0,"radius_m":r,"steer_min_rad":-110f64.to_radians(),"steer_max_rad":110f64.to_radians(),"steer_rate_radps":rate,"max_speed_mps":v_cap}),
                json!({"name":"rear_left_load_wheel","kind":"fixed","x":0.0,"y":left - m,"radius_m":r * 0.75}),
                json!({"name":"rear_right_load_wheel","kind":"fixed","x":0.0,"y":-(right - m),"radius_m":r * 0.75})])
        }
        "dual_steer" => {
            let d = l * 0.36;
            let mut ws = vec![json!({"name":"front_steer_wheel","kind":"steer","x":xc + d,"y":0.0,"radius_m":r,"steer_min_rad":-175f64.to_radians(),"steer_max_rad":175f64.to_radians(),"steer_rate_radps":rate,"max_speed_mps":v_cap}),
                json!({"name":"rear_steer_wheel","kind":"steer","x":xc - d,"y":0.0,"radius_m":r,"steer_min_rad":-175f64.to_radians(),"steer_max_rad":175f64.to_radians(),"steer_rate_radps":rate,"max_speed_mps":v_cap})];
            let cr = r * 0.7;
            for (i, (x, y)) in [(xc, left - m - cr), (xc, -(right - m - cr))].iter().enumerate() {
                ws.push(json!({"name":format!("caster_{i}"),"kind":"caster","x":x,"y":y,"radius_m":cr}));
            }
            Ok(ws)
        }
        o => Err(format!("unknown chassis type {o}")),
    }
}

pub fn build_kinematics(spec: &Value, chassis_type: Option<&str>) -> Result<(Chassis, Vec<Value>), String> {
    let native = spec["chassis"].get("type").and_then(|t| t.as_str()).unwrap_or("diff_drive").to_string();
    let given: Option<Vec<Value>> = spec.get("wheels").and_then(|w| w.as_array()).filter(|w| !w.is_empty()).cloned();
    let (ct, wheels_d) = match chassis_type {
        None => (native.clone(), given),
        Some(c) if c == native || (c == "dual_steer" && native == "multi_steer") => (native.clone(), given),
        Some(c) => (c.to_string(), None),
    };
    let wheels_d = match wheels_d {
        Some(w) if ct == native => w,
        _ => {
            let key = if ct == native { if ["diff_drive", "single_steer", "dual_steer"].contains(&native.as_str()) { native.as_str() } else { "dual_steer" } } else { ct.as_str() };
            preset_wheels(spec, key)?
        }
    };
    let wheels: Vec<Wheel> = wheels_d.iter().map(wheel_from_value).collect();
    Ok((Chassis::new(&ct, wheels, limits_from_spec(spec)), wheels_d))
}
