//! SE(2) helpers, polygon-vs-segment collision, 2D raycast against vertical segments/circles.
use std::f64::consts::PI;

#[inline]
pub fn pymod(a: f64, b: f64) -> f64 {
    let m = a % b;
    if m != 0.0 { if (b < 0.0) != (m < 0.0) { m + b } else { m } } else { 0.0f64.copysign(b) }
}
#[inline]
pub fn wrap(a: f64) -> f64 { pymod(a + PI, 2.0 * PI) - PI }

pub fn se2_integrate(x: f64, y: f64, th: f64, vx: f64, vy: f64, wz: f64, dt: f64) -> (f64, f64, f64) {
    let dth = wz * dt;
    let (dxb, dyb) = if dth.abs() < 1e-9 {
        (vx * dt, vy * dt)
    } else {
        let (s, c) = dth.sin_cos();
        let a = s / wz;
        let b = (1.0 - c) / wz;
        (a * vx - b * vy, b * vx + a * vy)
    };
    let (st, ct) = th.sin_cos();
    (x + ct * dxb - st * dyb, y + st * dxb + ct * dyb, wrap(th + dth))
}

/// Segment: x0 y0 x1 y1 z_top
pub type Seg = [f64; 5];

fn point_in_poly(px: f64, py: f64, p: &[[f64; 2]]) -> bool {
    let n = p.len();
    let mut cross = 0;
    for k in 0..n {
        let (x0, y0) = (p[k][0], p[k][1]);
        let k1 = (k + 1) % n;
        let (x1, y1) = (p[k1][0], p[k1][1]);
        if (y0 > py) != (y1 > py) {
            let xint = (x1 - x0) * (py - y0) / (y1 - y0) + x0;
            if px < xint { cross += 1; }
        }
    }
    cross % 2 == 1
}

/// Polygon `fp` (body frame) at pose (x,y,th) vs vertical segments. Returns contact point.
pub fn collides(fp: &[[f64; 2]], x: f64, y: f64, th: f64, segs: &[Seg]) -> Option<[f64; 2]> {
    let np = fp.len();
    if segs.is_empty() || np == 0 || np > 64 { return None; }
    let (s, c) = th.sin_cos();
    let mut p = [[0.0f64; 2]; 64];
    let (mut mnx, mut mny, mut mxx, mut mxy) = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for k in 0..np {
        let (fx, fy) = (fp[k][0], fp[k][1]);
        let px = fx * c + fy * (-s) + x;
        let py = fx * s + fy * c + y;
        p[k] = [px, py];
        mnx = mnx.min(px); mxx = mxx.max(px); mny = mny.min(py); mxy = mxy.max(py);
    }
    mnx -= 0.01; mny -= 0.01; mxx += 0.01; mxy += 0.01;
    let p = &p[..np];
    let near = |sg: &Seg| {
        let (sx0, sx1) = (sg[0].min(sg[2]), sg[0].max(sg[2]));
        let (sy0, sy1) = (sg[1].min(sg[3]), sg[1].max(sg[3]));
        sx1 >= mnx && sx0 <= mxx && sy1 >= mny && sy0 <= mxy
    };
    let mut any = false;
    for i in 0..np {
        let i1 = (i + 1) % np;
        let (px, py) = (p[i][0], p[i][1]);
        let (rx, ry) = (p[i1][0] - px, p[i1][1] - py);
        for sg in segs {
            if !near(sg) { continue; }
            any = true;
            let (qx, qy) = (sg[0], sg[1]);
            let (sx, sy) = (sg[2] - sg[0], sg[3] - sg[1]);
            let rxs = rx * sy - ry * sx;
            if !(rxs.abs() > 1e-12) { continue; }
            let (qpx, qpy) = (qx - px, qy - py);
            let t = (qpx * sy - qpy * sx) / rxs;
            let u = (qpx * ry - qpy * rx) / rxs;
            if t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0 {
                return Some([px + t * rx, py + t * ry]);
            }
        }
    }
    if !any { return None; }
    for e in 0..2 {
        for sg in segs {
            if !near(sg) { continue; }
            let (qx, qy) = (sg[2 * e], sg[2 * e + 1]);
            if point_in_poly(qx, qy, p) { return Some([qx, qy]); }
        }
    }
    None
}

/// Circle: cx cy r z_top
pub type Circle = [f64; 4];

/// Horizontal rays vs segments (+circles) taller than `zmin`. out[i] = hit distance or INFINITY.
pub fn raycast2d(segs: &[Seg], circles: &[Circle], zmin: f64, ox: f64, oy: f64, angles: &[f64], max_range: f64, out: &mut [f64]) {
    let idx: Vec<usize> = segs.iter().enumerate().filter_map(|(j, s)| {
        if !(s[4] > zmin) { return None; }
        let (abx, aby) = (s[2] - s[0], s[3] - s[1]);
        let l2 = (abx * abx + aby * aby).max(1e-12);
        let tt = (((ox - s[0]) * abx + (oy - s[1]) * aby) / l2).clamp(0.0, 1.0);
        if (s[0] + abx * tt - ox).hypot(s[1] + aby * tt - oy) <= max_range { Some(j) } else { None }
    }).collect();
    for (i, &a) in angles.iter().enumerate() {
        let (dy, dx) = a.sin_cos();
        let mut best = f64::INFINITY;
        for &j in &idx {
            let s = &segs[j];
            let (abx, aby) = (s[2] - s[0], s[3] - s[1]);
            let (qx, qy) = (s[0] - ox, s[1] - oy);
            let den = dx * aby - dy * abx;
            if !(den.abs() > 1e-12) { continue; }
            let t = (qx * aby - qy * abx) / den;
            let u = (qx * dy - qy * dx) / den;
            if t > 1e-6 && u >= 0.0 && u <= 1.0 && t < best { best = t; }
        }
        for cc in circles {
            if !(cc[3] > zmin) { continue; }
            let (fx, fy) = (ox - cc[0], oy - cc[1]);
            let b = fx * dx + fy * dy;
            let c2 = fx * fx + fy * fy - cc[2] * cc[2];
            let disc = b * b - c2;
            if disc < 0.0 { continue; }
            let sq = disc.sqrt();
            let mut t = -b - sq;
            if t <= 1e-6 { t = -b + sq; }
            if t > 1e-6 && t < best { best = t; }
        }
        out[i] = if best > max_range { f64::INFINITY } else { best };
    }
}
