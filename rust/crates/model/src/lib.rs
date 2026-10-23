//! Vehicle model logic shared by sim / nav / hub: protection space, manual overrides, audit, URDF, nav limits.
pub mod protection;
pub mod overrides;
pub mod urdf;
pub mod limits;

use serde_json::Value;

pub fn fnum(v: Option<&Value>, d: f64) -> f64 {
    match v {
        Some(Value::Number(n)) => n.as_f64().unwrap_or(d),
        Some(Value::String(s)) => s.trim().parse().unwrap_or(d),
        Some(Value::Bool(b)) => *b as i64 as f64,
        _ => d,
    }
}
/// `float(x.get(k) or d)` – falsy (null/0/"") falls back to d
pub fn fnum_or(v: Option<&Value>, d: f64) -> f64 {
    let x = fnum(v, 0.0);
    if v.is_none() || matches!(v, Some(Value::Null)) || x == 0.0 { d } else { x }
}
pub fn rnd(x: f64, n: i32) -> f64 { let m = 10f64.powi(n); (x * m).round_ties_even() / m }
/// Python repr/str of a float
pub fn pf(x: f64) -> String {
    if x.is_finite() && x.fract() == 0.0 && x.abs() < 1e16 { format!("{:.1}", x) } else { format!("{}", x) }
}
/// Python str() of a JSON value
pub fn pstr(v: &Value) -> String {
    match v {
        Value::Null => "None".into(),
        Value::String(s) => s.clone(),
        Value::Bool(b) => if *b { "True".into() } else { "False".into() },
        Value::Number(n) => if n.is_f64() { pf(n.as_f64().unwrap()) } else { n.to_string() },
        other => other.to_string(),
    }
}
