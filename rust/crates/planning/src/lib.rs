//! Path planning + vehicle protection geometry (port of planning/*.py).
pub mod astar;
pub mod maneuver;
pub mod protection;
pub mod route;
pub mod scenario;
pub mod sweep;

pub type Pt = (f64, f64);
pub type Seg = [f64; 4];

#[inline]
pub fn wrap(a: f64) -> f64 { a.sin().atan2(a.cos()) }
/// Python round(x, 2) for key purposes
#[inline]
pub fn r2(x: f64) -> f64 { (x * 100.0).round_ties_even() / 100.0 }
