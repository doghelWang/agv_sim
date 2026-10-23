//! Nav limits per chassis type (port of tools/gen_nav2_params.effective_limits)
use crate::{fnum, rnd};
use serde_json::{json, Value};
use sim_core::kin::build_kinematics;

pub const CHASSIS_TYPES: [&str; 3] = ["diff_drive", "single_steer", "dual_steer"];

pub fn effective_limits(spec: &Value, chassis_type: &str) -> Result<Value, String> {
    if let Some(pre) = spec.get("nav_limits").and_then(|n| n.get(chassis_type)).filter(|p| crate::overrides::truthy(p)) { return Ok(pre.clone()); }
    let (kin, _) = build_kinematics(spec, Some(chassis_type))?;
    let ch = &spec["chassis"];
    let mut v = fnum(ch.get("max_speed_mps"), 1.0);
    let mut w = fnum(ch.get("max_ang_speed_radps"), 1.0);
    let caps: Vec<f64> = kin.wheels.iter().filter(|wh| wh.kind.driven()).map(|wh| wh.max_speed).collect();
    let cmin = caps.iter().cloned().fold(f64::INFINITY, f64::min);
    if !caps.is_empty() { v = v.min(cmin); }
    let mut rmax = kin.wheels.iter().filter(|wh| wh.kind.driven()).map(|wh| (if !kin.holonomic { wh.x - kin.axle_x } else { wh.x }).hypot(wh.y)).fold(f64::NEG_INFINITY, f64::max);
    if !rmax.is_finite() { rmax = 0.5; }
    if !caps.is_empty() && rmax > 1e-3 { w = w.min(cmin / rmax); }
    Ok(json!({"v": rnd(v * 0.95, 3), "w": rnd((w * 0.9).min(1.5), 3), "a": fnum(ch.get("max_accel_mps2"), 0.5), "d": fnum(ch.get("max_decel_mps2"), 0.5),
        "aw": fnum(ch.get("max_ang_accel_radps2"), 1.0), "dw": fnum(ch.get("max_ang_decel_radps2"), 1.0), "holonomic": kin.holonomic}))
}
