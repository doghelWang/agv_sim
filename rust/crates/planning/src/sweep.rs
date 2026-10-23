//! Exact arc-vs-box sweep test (port of planning/sweep.py).
use std::f64::consts::PI;
const TWO_PI: f64 = 2.0 * PI;

#[inline]
fn inb(x: f64, y: f64, b: &[f64; 4]) -> bool { x > b[0] && x < b[1] && y > b[2] && y < b[3] }

/// Does a point rotated about (cx,cy) by phi enter (or start inside) the open box (x0,x1,y0,y1)?
pub fn arc_hits_one(px: f64, py: f64, phi: f64, b: &[f64; 4], cx: f64, cy: f64) -> bool {
    if inb(px, py, b) { return true; }
    let (ux, uy) = (px - cx, py - cy);
    let r2 = ux * ux + uy * uy;
    let fx = (b[0] - cx).abs().max((b[1] - cx).abs());
    let fy = (b[2] - cy).abs().max((b[3] - cy).abs());
    if !(r2 < fx * fx + fy * fy && r2 > 1e-24) || phi.abs() < 1e-12 { return false; }
    let rho = r2.sqrt();
    let a0 = uy.atan2(ux);
    let span = phi.abs();
    let full = span >= TWO_PI;
    if inb(cx + rho * (a0 + phi).cos(), cy + rho * (a0 + phi).sin(), b) { return true; }
    let on_arc = |a: f64| -> bool {
        if full { return true; }
        let t = if phi >= 0.0 { a - a0 } else { a0 - a };
        t.rem_euclid(TWO_PI) <= span
    };
    for &xx in &[b[0], b[1]] {
        let dx = xx - cx;
        if dx.abs() <= rho {
            let dy = (r2 - dx * dx).max(0.0).sqrt();
            for &sy in &[dy, -dy] {
                let y = cy + sy;
                if y > b[2] && y < b[3] && on_arc(sy.atan2(dx)) { return true; }
            }
        }
    }
    for &yy in &[b[2], b[3]] {
        let dy = yy - cy;
        if dy.abs() <= rho {
            let dx = (r2 - dy * dy).max(0.0).sqrt();
            for &sx in &[dx, -dx] {
                let x = cx + sx;
                if x > b[0] && x < b[1] && on_arc(dy.atan2(sx)) { return true; }
            }
        }
    }
    false
}

pub fn arc_hits_any(pts: &[(f64, f64)], phi: f64, b: &[f64; 4]) -> bool {
    pts.iter().any(|&(x, y)| arc_hits_one(x, y, phi, b, 0.0, 0.0))
}
