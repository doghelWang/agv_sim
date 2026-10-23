//! Body maneuver feasibility (port of planning/maneuver.py `_py` implementations).
use crate::{Pt, Seg};
use std::f64::consts::PI;

pub const CLEAR_MIN: f64 = 0.05;
pub const SWEEP_STEP: f64 = 0.01;

pub fn sweep_samples(max_travel: f64, min_n: usize) -> usize {
    min_n.max((max_travel / SWEEP_STEP).ceil() as usize)
}

/// obstacle boxes (x,y,w,h) → wall segments
pub fn box_segments(obs: &[(f64, f64, f64, f64)]) -> Vec<Seg> {
    let mut s = vec![];
    for &(ox, oy, w, h) in obs {
        let (hw, hh) = (w / 2.0, h / 2.0);
        s.push([ox - hw, oy - hh, ox + hw, oy - hh]);
        s.push([ox + hw, oy - hh, ox + hw, oy + hh]);
        s.push([ox + hw, oy + hh, ox - hw, oy + hh]);
        s.push([ox - hw, oy + hh, ox - hw, oy - hh]);
    }
    s
}

fn pt_seg2(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let (dx, dy) = (bx - ax, by - ay);
    let l2 = (dx * dx + dy * dy).max(1e-12);
    let u = (((px - ax) * dx + (py - ay) * dy) / l2).clamp(0.0, 1.0);
    let (qx, qy) = (ax + u * dx - px, ay + u * dy - py);
    qx * qx + qy * qy
}

fn pt_box2(x: f64, y: f64, head: f64, tail: f64, hw: f64) -> f64 {
    let ddx = if x < -tail { -tail - x } else if x > head { x - head } else { 0.0 };
    let ddy = (y.abs() - hw).max(0.0);
    ddx * ddx + ddy * ddy
}

/// Min distance (m) of body to segment set over poses; -1 if a segment endpoint lies inside the body;
/// 0 if a segment crosses; 9.0 if nothing within the pose bounding box ± R.
pub fn clearance(segs: &[Seg], poses: &[(f64, f64, f64)], head: f64, tail: f64, hw: f64) -> f64 {
    if segs.is_empty() || poses.is_empty() { return 9.0; }
    let (mut xmin, mut xmax, mut ymin, mut ymax) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for p in poses { xmin = xmin.min(p.0); xmax = xmax.max(p.0); ymin = ymin.min(p.1); ymax = ymax.max(p.1); }
    let rr = head.max(tail).hypot(hw) + 0.3;
    let s: Vec<&Seg> = segs.iter().filter(|w| {
        w[0].min(w[2]) < xmax + rr && w[0].max(w[2]) > xmin - rr && w[1].min(w[3]) < ymax + rr && w[1].max(w[3]) > ymin - rr
    }).collect();
    if s.is_empty() { return 9.0; }
    let mut best = f64::INFINITY;
    let mut crossing = false;
    let mut tr: Vec<[f64; 4]> = Vec::with_capacity(s.len());
    for &(x, y, yaw) in poses {
        let (sn, c) = yaw.sin_cos();
        tr.clear();
        let mut inside = false;
        for w in &s {
            let ax = (w[0] - x) * c + (w[1] - y) * sn;
            let ay = -(w[0] - x) * sn + (w[1] - y) * c;
            let bx = (w[2] - x) * c + (w[3] - y) * sn;
            let by = -(w[2] - x) * sn + (w[3] - y) * c;
            if (ax > -tail && ax < head && ay.abs() < hw) || (bx > -tail && bx < head && by.abs() < hw) { inside = true; }
            tr.push([ax, ay, bx, by]);
        }
        if inside { return -1.0; }
        let mut cross_here = false;
        for t in &tr {
            let (ax, ay, bx, by) = (t[0], t[1], t[2], t[3]);
            let (dx, dy) = (bx - ax, by - ay);
            let (mut t0, mut t1, mut ok) = (0.0f64, 1.0f64, true);
            for (p, q) in [(-dx, ax + tail), (dx, head - ax), (-dy, ay + hw), (dy, hw - ay)] {
                let par = p.abs() < 1e-15;
                if par && q < 0.0 { ok = false; }
                let r = if par { 0.0 } else { q / p };
                if !par && p < 0.0 { t0 = t0.max(r); }
                if !par && p > 0.0 { t1 = t1.min(r); }
            }
            if ok && t0 <= t1 { cross_here = true; break; }
        }
        if cross_here { crossing = true; continue; }
        let mut d2 = f64::INFINITY;
        for t in &tr {
            let (ax, ay, bx, by) = (t[0], t[1], t[2], t[3]);
            let mut d = pt_box2(ax, ay, head, tail, hw).min(pt_box2(bx, by, head, tail, hw));
            for (cx, cy) in [(-tail, -hw), (head, -hw), (head, hw), (-tail, hw)] { d = d.min(pt_seg2(cx, cy, ax, ay, bx, by)); }
            d2 = d2.min(d);
        }
        best = best.min(d2);
    }
    if crossing { 0.0 } else { best.sqrt() }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Corner {
    Rotate(f64),
    Arc { heading: f64, turn: f64, r: f64, d: f64, v: f64, cx: f64, cy: f64 },
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum CornerMode { Rotate, Arc, Auto }
impl CornerMode {
    pub fn parse(s: &str) -> CornerMode { match s { "arc" => CornerMode::Arc, "auto" => CornerMode::Auto, _ => CornerMode::Rotate } }
    pub fn name(&self) -> &'static str { match self { CornerMode::Rotate => "rotate", CornerMode::Arc => "arc", CornerMode::Auto => "auto" } }
}

#[allow(clippy::too_many_arguments)]
pub fn plan_corner(segs: &[Seg], node: Pt, h1: f64, h2: f64, len_in: f64, len_out: f64, head: f64, tail: f64, hw: f64,
                   r_pref: f64, clear_min: f64, mode: CornerMode) -> (Corner, f64) {
    let turn = crate::wrap(h2 - h1);
    let sgn = if turn > 0.0 { 1.0 } else { -1.0 };
    let mut cands: Vec<(Corner, f64)> = vec![];
    let mut rots: Vec<(Corner, f64)> = vec![];
    let rmax = head.max(tail).hypot(hw);
    for tt in [turn, turn - sgn * 2.0 * PI] {
        let n = sweep_samples(tt.abs() * rmax, 16);
        let rot: Vec<(f64, f64, f64)> = (0..=n).map(|i| (node.0, node.1, h1 + tt * i as f64 / n as f64)).collect();
        rots.push((Corner::Rotate(if tt > 0.0 { 1.0 } else { -1.0 }), clearance(segs, &rot, head, tail, hw)));
    }
    if mode == CornerMode::Auto {
        for c in &rots { if c.1 >= clear_min { return c.clone(); } }
    }
    if matches!(mode, CornerMode::Arc | CornerMode::Auto) && turn.abs() < 2.6 {
        let t_half = (turn.abs() / 2.0).tan();
        let r_lim = 0.45 * len_in.min(len_out) / t_half.max(1e-3);
        let r_max = r_pref.min(r_lim);
        let mut rs = vec![(r_pref * 1.25).min(r_lim), r_max, r_max * 0.7, r_max * 0.45];
        rs.sort_by(|a, b| b.total_cmp(a));
        rs.dedup();
        let mut arcs: Vec<(Corner, f64)> = vec![];
        for r in rs {
            if r < 0.25 { continue; }
            let d = r * t_half;
            let (sx, sy) = (node.0 - d * h1.cos(), node.1 - d * h1.sin());
            let (cx, cy) = (sx - sgn * r * h1.sin(), sy + sgn * r * h1.cos());
            let n = sweep_samples(turn.abs() * (r + rmax), 12);
            let poses: Vec<(f64, f64, f64)> = (0..=n).map(|i| {
                let a = h1 + turn * i as f64 / n as f64;
                (cx + sgn * r * a.sin(), cy - sgn * r * a.cos(), a)
            }).collect();
            let clr = clearance(segs, &poses, head, tail, hw);
            arcs.push((Corner::Arc { heading: h2, turn, r, d, v: 0.35, cx, cy }, clr));
        }
        let mut best: Option<&(Corner, f64)> = None;
        let key = |a: &(Corner, f64)| -> (f64, f64) { (crate::r2(a.1), if let Corner::Arc { r, .. } = a.0 { r } else { 0.0 }) };
        for a in arcs.iter().filter(|a| a.1 >= clear_min) {
            match best { None => best = Some(a), Some(b) => { let (ka, kb) = (key(a), key(b)); if ka.0 > kb.0 || (ka.0 == kb.0 && ka.1 > kb.1) { best = Some(a); } } }
        }
        if let Some(b) = best { return b.clone(); }
        cands.extend(arcs);
    }
    for c in rots {
        cands.push(c.clone());
        if c.1 >= clear_min { return c; }
    }
    let mut best = cands[0].clone();
    for c in &cands[1..] { if c.1 > best.1 { best = c.clone(); } }
    best
}
