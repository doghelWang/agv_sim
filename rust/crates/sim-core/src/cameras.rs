//! Camera-class sensors (mono RGB / stereo / ToF): per-pixel ray casting against analytic primitives + Lambert shading.
use crate::rng::Rng;
use crate::sensors::rot_rpy;
use crate::world::*;
use serde_json::{json, Value};

pub const NORMAL_LIGHT: [f64; 3] = [0.35, 0.25, 0.9];

/// Static scene description for ray casting / shading (rebuilt when the world revision changes)
pub struct CamScene {
    pub revision: u64,
    pub prims: Vec<Prim>,
    pub rgb: Vec<[f32; 3]>,
    pub floor_gid: i32,
    pub ceil_gid: i32,
    pub tex: std::sync::Arc<Vec<f32>>,
    pub tex_h: usize,
    pub tex_w: usize,
    pub tex_org: (f64, f64, f64),
}


pub fn build_scene(world: &World, stations: &[[f64; 2]], tags: &[[f64; 2]], lanes: &[[f64; 4]], revision: u64, reuse_tex: Option<std::sync::Arc<Vec<f32>>>) -> CamScene {
    let mut prims: Vec<Prim> = vec![];
    let mut rgb: Vec<[f32; 3]> = vec![CAT_COLORS[0], CAT_COLORS[4]];
    for s in &world.static_segments {
        let (x0, y0, x1, y1, h) = (s[0], s[1], s[2], s[3], s[4]);
        let l = (x1 - x0).hypot(y1 - y0);
        if l < 0.01 { continue; }
        let cat = if h >= CEILING_HEIGHT - 1e-3 { "wall" } else { "shelf" };
        let half = [l / 2.0, 0.025, h / 2.0];
        let rad = (half[0] * half[0] + half[1] * half[1] + half[2] * half[2]).sqrt();
        let r = rot_z((y1 - y0).atan2(x1 - x0));
        let gid = rgb.len() as f64;
        rgb.push(CAT_COLORS[cat_id(cat)]);
        let mut p = [0.0f64; 18];
        p[0] = (x0 + x1) / 2.0; p[1] = (y0 + y1) / 2.0; p[2] = h / 2.0;
        p[3..12].copy_from_slice(&r);
        p[12..15].copy_from_slice(&half);
        p[15] = 0.0; p[16] = gid; p[17] = rad * 1.0001 + 1e-6;
        prims.push(p);
    }
    for o in &world.obstacles {
        let g = |k: &str, d: f64| o.get(k).and_then(|v| v.as_f64()).unwrap_or(d);
        let z = o.get("z").and_then(|v| v.as_f64()).or_else(|| o.get("height").and_then(|v| v.as_f64())).unwrap_or(1.0);
        let typ = o.get("type").and_then(|t| t.as_str());
        let (w, h) = (g("w", 0.8), g("h", 0.8));
        let gid = rgb.len() as f64;
        rgb.push(obstacle_color(typ));
        let mut p = [0.0f64; 18];
        p[0] = g("x", 0.0); p[1] = g("y", 0.0); p[2] = g("z0", 0.0) + z / 2.0;
        p[3..12].copy_from_slice(&rot_z(g("yaw", 0.0)));
        if typ == Some("person") {
            let r = w.max(h) / 2.0;
            p[12] = r; p[13] = r; p[14] = z / 2.0; p[15] = 1.0; p[17] = r.hypot(z / 2.0) * 1.0001 + 1e-6;
        } else {
            p[12] = w / 2.0; p[13] = h / 2.0; p[14] = z / 2.0; p[15] = 0.0;
            p[17] = ((w / 2.0).powi(2) + (h / 2.0).powi(2) + (z / 2.0).powi(2)).sqrt() * 1.0001 + 1e-6;
        }
        p[16] = gid;
        prims.push(p);
    }
    match reuse_tex {
        Some(t) => {
            let b = world.bounds;
            let (x0, y0) = (b[0] - 1.0, b[1] - 1.0);
            let w = ((b[2] - b[0] + 2.0) / 0.02).ceil() as usize;
            let h = ((b[3] - b[1] + 2.0) / 0.02).ceil() as usize;
            CamScene { revision, prims, rgb, floor_gid: 0, ceil_gid: 1, tex: t, tex_h: h, tex_w: w, tex_org: (x0, y0, 0.02) }
        }
        None => {
            let (tex, th, tw, org) = make_floor_texture(world, stations, tags, lanes);
            CamScene { revision, prims, rgb, floor_gid: 0, ceil_gid: 1, tex: std::sync::Arc::new(tex), tex_h: th, tex_w: tw, tex_org: org }
        }
    }
}

fn make_floor_texture(world: &World, stations: &[[f64; 2]], tags: &[[f64; 2]], lanes: &[[f64; 4]]) -> (Vec<f32>, usize, usize, (f64, f64, f64)) {
    let res = 0.02;
    let b = world.bounds;
    let (x0, y0) = (b[0] - 1.0, b[1] - 1.0);
    let w = ((b[2] - b[0] + 2.0) / res).ceil() as usize;
    let h = ((b[3] - b[1] + 2.0) / res).ceil() as usize;
    let mut t = vec![0f32; h * w * 3];
    let ix: Vec<i32> = (0..w).map(|i| ((x0 + (i as f64 + 0.5) * res) / 0.5).floor() as i32).collect();
    let iy: Vec<i32> = (0..h).map(|j| ((y0 + (j as f64 + 0.5) * res) / 0.5).floor() as i32).collect();
    let base = CAT_COLORS[0];
    for j in 0..h {
        for i in 0..w {
            let chk = ((iy[j] + ix[i]).rem_euclid(2)) as f32;
            let k = 0.92f32 + 0.08f32 * chk;
            for c in 0..3 { t[(j * w + i) * 3 + c] = base[c] * k; }
        }
    }
    let win = |cx0: f64, cy0: f64, cx1: f64, cy1: f64| {
        let i0 = (((cx0 - x0) / res) as i64).max(0) as usize;
        let i1 = (((cx1 - x0) / res) as i64 + 1).min(w as i64).max(0) as usize;
        let j0 = (((cy0 - y0) / res) as i64).max(0) as usize;
        let j1 = (((cy1 - y0) / res) as i64 + 1).min(h as i64).max(0) as usize;
        (i0, i1, j0, j1)
    };
    for l in lanes {
        let (ax, ay, bx, by) = (l[0], l[1], l[2], l[3]);
        let (i0, i1, j0, j1) = win(ax.min(bx) - 0.05, ay.min(by) - 0.05, ax.max(bx) + 0.05, ay.max(by) + 0.05);
        let (abx, aby) = (bx - ax, by - ay);
        let l2 = (abx * abx + aby * aby).max(1e-9);
        for j in j0..j1 { for i in i0..i1 {
            let (x, y) = (x0 + (i as f64 + 0.5) * res, y0 + (j as f64 + 0.5) * res);
            let tt = (((x - ax) * abx + (y - ay) * aby) / l2).clamp(0.0, 1.0);
            if (x - (ax + tt * abx)).hypot(y - (ay + tt * aby)) < 0.025 { for c in 0..3 { t[(j * w + i) * 3 + c] = CAT_COLORS[8][c]; } }
        } }
    }
    for s in stations {
        let (i0, i1, j0, j1) = win(s[0] - 0.4, s[1] - 0.4, s[0] + 0.4, s[1] + 0.4);
        for j in j0..j1 { for i in i0..i1 {
            let (x, y) = (x0 + (i as f64 + 0.5) * res, y0 + (j as f64 + 0.5) * res);
            if (x - s[0]).hypot(y - s[1]) < 0.35 { for c in 0..3 { t[(j * w + i) * 3 + c] = CAT_COLORS[5][c]; } }
        } }
    }
    for g in tags {
        let (i0, i1, j0, j1) = win(g[0] - 0.05, g[1] - 0.05, g[0] + 0.05, g[1] + 0.05);
        for j in j0..j1 { for i in i0..i1 {
            let (x, y) = (x0 + (i as f64 + 0.5) * res, y0 + (j as f64 + 0.5) * res);
            if (x - g[0]).abs().max((y - g[1]).abs()) < 0.04 { for c in 0..3 { t[(j * w + i) * 3 + c] = CAT_COLORS[6][c]; } }
        } }
    }
    (t, h, w, (x0, y0, res))
}

// ------------------------------------------------------------------ ray / primitive intersection (port of C kernel)
fn hit_box(p: &Prim, ol3: &[f64; 3], d: &[f64; 3], n_out: &mut [f64; 3]) -> Option<f64> {
    let r = &p[3..12];
    let h = &p[12..15];
    let (mut tn, mut tf) = (-1e300f64, 1e300f64);
    let mut an: i32 = -1;
    let mut sn = 0.0f64;
    for a in 0..3 {
        let (ax, ay, az) = (r[a], r[3 + a], r[6 + a]);
        let ol = ol3[a];
        let dl = d[0] * ax + d[1] * ay + d[2] * az;
        if dl.abs() < 1e-12 {
            if ol.abs() > h[a] { return None; }
            continue;
        }
        let (mut t1, mut t2, mut s) = ((-h[a] - ol) / dl, (h[a] - ol) / dl, -1.0);
        if t1 > t2 { std::mem::swap(&mut t1, &mut t2); s = 1.0; }
        if t1 > tn { tn = t1; an = a as i32; sn = s; }
        if t2 < tf { tf = t2; }
        if tn > tf { return None; }
    }
    if tf < 0.0 { return None; }
    let mut t = tn;
    if tn < 0.0 {
        t = tf;
        an = -1;
        let mut best = 1e300;
        for a in 0..3 {
            let (ax, ay, az) = (r[a], r[3 + a], r[6 + a]);
            let ol = ol3[a];
            let dl = d[0] * ax + d[1] * ay + d[2] * az;
            let pl = ol + dl * t;
            let e = (pl.abs() - h[a]).abs();
            if e < best { best = e; an = a as i32; sn = if pl > 0.0 { 1.0 } else { -1.0 }; }
        }
    }
    if an >= 0 { let a = an as usize; n_out[0] = sn * r[a]; n_out[1] = sn * r[3 + a]; n_out[2] = sn * r[6 + a]; }
    Some(t)
}

fn hit_cyl(p: &Prim, ol: &[f64; 3], d: &[f64; 3], n_out: &mut [f64; 3]) -> Option<f64> {
    let r = &p[3..12];
    let (rad, hz) = (p[12], p[14]);
    let mut dl = [0.0f64; 3];
    for a in 0..3 { dl[a] = d[0] * r[a] + d[1] * r[3 + a] + d[2] * r[6 + a]; }
    let mut best = 1e300f64;
    let mut nl = [0.0f64; 3];
    let aa = dl[0] * dl[0] + dl[1] * dl[1];
    let bb = ol[0] * dl[0] + ol[1] * dl[1];
    let cc = ol[0] * ol[0] + ol[1] * ol[1] - rad * rad;
    if aa > 1e-18 {
        let disc = bb * bb - aa * cc;
        if disc >= 0.0 {
            let sq = disc.sqrt();
            for k in 0..2 {
                let t = (if k == 0 { -bb - sq } else { -bb + sq }) / aa;
                if t < 0.0 || t >= best { continue; }
                let z = ol[2] + dl[2] * t;
                if z.abs() > hz { continue; }
                best = t; nl = [(ol[0] + dl[0] * t) / rad, (ol[1] + dl[1] * t) / rad, 0.0];
            }
        }
    }
    if dl[2].abs() > 1e-12 {
        for k in 0..2 {
            let zc = if k == 0 { -hz } else { hz };
            let t = (zc - ol[2]) / dl[2];
            if t < 0.0 || t >= best { continue; }
            let (x, y) = (ol[0] + dl[0] * t, ol[1] + dl[1] * t);
            if x * x + y * y > rad * rad { continue; }
            best = t; nl = [0.0, 0.0, if k == 0 { -1.0 } else { 1.0 }];
        }
    }
    if best >= 1e299 { return None; }
    for a in 0..3 { n_out[a] = r[3 * a] * nl[0] + r[3 * a + 1] * nl[1] + r[3 * a + 2] * nl[2]; }
    Some(best)
}

pub struct CastResult { pub dist: Vec<f64>, pub gid: Vec<i32>, pub nrm: Option<Vec<f64>> }

/// Single-origin multi-ray cast against primitives + floor/ceiling planes (port of sc_cast_prims)
pub fn cast_prims(scene: &CamScene, origin: [f64; 3], dirs: &[f64], max_range: f64, want_normal: bool, threads: usize) -> CastResult {
    let n = dirs.len() / 3;
    let mut sel: Vec<(Prim, [f64; 4])> = vec![];
    for p in &scene.prims {
        let r = &p[3..12];
        let oc = [origin[0] - p[0], origin[1] - p[1], origin[2] - p[2]];
        let c2 = oc[0] * oc[0] + oc[1] * oc[1] + oc[2] * oc[2];
        if c2.sqrt() - p[17] > max_range { continue; }
        let mut ol = [0.0; 4];
        for a in 0..3 { ol[a] = oc[0] * r[a] + oc[1] * r[3 + a] + oc[2] * r[6 + a]; }
        ol[3] = c2;
        sel.push((*p, ol));
    }
    let mut dist = vec![f64::INFINITY; n];
    let mut gid = vec![-1i32; n];
    let mut nrm = if want_normal { vec![0.0f64; 3 * n] } else { vec![] };
    let nt = if n < 1024 { 1 } else { threads.max(1).min(16) };
    let work = |n0: usize, n1: usize, dist: &mut [f64], gid: &mut [i32], nrm: &mut [f64]| {
        for i in n0..n1 {
            let d = [dirs[3 * i], dirs[3 * i + 1], dirs[3 * i + 2]];
            let mut best = max_range;
            let mut nn = [0.0f64; 3];
            let mut nb = [0.0f64; 3];
            let mut g: i32 = -1;
            for (p, ol) in &sel {
                let b = (p[0] - origin[0]) * d[0] + (p[1] - origin[1]) * d[1] + (p[2] - origin[2]) * d[2];
                let r = p[17];
                if b + r < 0.0 || b - r > best { continue; }
                if ol[3] - b * b > r * r { continue; }
                let o3 = [ol[0], ol[1], ol[2]];
                let hit = if p[15] < 0.5 { hit_box(p, &o3, &d, &mut nb) } else { hit_cyl(p, &o3, &d, &mut nb) };
                if let Some(t) = hit { if t < best { best = t; g = p[16] as i32; nn = nb; } }
            }
            let mut dd = if g >= 0 { best } else { f64::INFINITY };
            let (dz, oz) = (d[2], origin[2]);
            if dz < -1e-9 { let tf = -oz / dz; if tf < dd { dd = tf; g = scene.floor_gid; nn = [0.0, 0.0, 1.0]; } }
            if dz > 1e-9 { let tc = (CEILING_HEIGHT - oz) / dz; if tc < dd { dd = tc; g = scene.ceil_gid; nn = [0.0, 0.0, -1.0]; } }
            if dd > max_range { dd = f64::INFINITY; }
            dist[i - n0] = dd; gid[i - n0] = g;
            if !nrm.is_empty() { nrm[3 * (i - n0)] = nn[0]; nrm[3 * (i - n0) + 1] = nn[1]; nrm[3 * (i - n0) + 2] = nn[2]; }
        }
    };
    if nt == 1 {
        work(0, n, &mut dist, &mut gid, &mut nrm);
    } else {
        let mut dchunks: Vec<&mut [f64]> = vec![];
        let mut gchunks: Vec<&mut [i32]> = vec![];
        let mut nchunks: Vec<&mut [f64]> = vec![];
        let (mut dr, mut gr, mut nr) = (&mut dist[..], &mut gid[..], &mut nrm[..]);
        let bounds: Vec<(usize, usize)> = (0..nt).map(|t| (n * t / nt, n * (t + 1) / nt)).collect();
        for &(a, b) in &bounds {
            let (d1, d2) = dr.split_at_mut(b - a); dchunks.push(d1); dr = d2;
            let (g1, g2) = gr.split_at_mut(b - a); gchunks.push(g1); gr = g2;
            if want_normal { let (n1, n2) = nr.split_at_mut(3 * (b - a)); nchunks.push(n1); nr = n2; } else { nchunks.push(&mut []); }
        }
        std::thread::scope(|sc| {
            for (((dc, gc), nc), &(a, b)) in dchunks.into_iter().zip(gchunks).zip(nchunks).zip(&bounds) {
                let work = &work;
                sc.spawn(move || work(a, b, dc, gc, nc));
            }
        });
    }
    CastResult { dist, gid, nrm: if want_normal { Some(nrm) } else { None } }
}

/// port of sc_cam_shade
pub fn shade(scene: &CamScene, o: [f64; 3], dirs: &[f64], r: &CastResult, sigma: f64, rng: Option<&mut Rng>) -> Vec<u8> {
    let n = r.dist.len();
    let mut out = vec![0u8; 3 * n];
    let ln = (NORMAL_LIGHT[0] * NORMAL_LIGHT[0] + NORMAL_LIGHT[1] * NORMAL_LIGHT[1] + NORMAL_LIGHT[2] * NORMAL_LIGHT[2]).sqrt();
    let l = [NORMAL_LIGHT[0] / ln, NORMAL_LIGHT[1] / ln, NORMAL_LIGHT[2] / ln];
    let mut rng = rng;
    let (x0, y0, res) = scene.tex_org;
    for i in 0..n {
        let mut col = [0.55f32; 3];
        let d = r.dist[i];
        if d.is_finite() {
            let g = r.gid[i];
            let mut base = [0.0f32; 3];
            if g >= 0 && (g as usize) < scene.rgb.len() { base = scene.rgb[g as usize]; }
            if g == scene.floor_gid && scene.tex_h > 0 && scene.tex_w > 0 {
                let (px, py) = (o[0] + dirs[3 * i] * d, o[1] + dirs[3 * i + 1] * d);
                let ix = (((px - x0) / res) as i64).clamp(0, scene.tex_w as i64 - 1) as usize;
                let iy = (((py - y0) / res) as i64).clamp(0, scene.tex_h as i64 - 1) as usize;
                let t = &scene.tex[3 * (iy * scene.tex_w + ix)..3 * (iy * scene.tex_w + ix) + 3];
                base = [t[0], t[1], t[2]];
            }
            let (nx, ny, nz) = match &r.nrm { Some(v) => (v[3 * i], v[3 * i + 1], v[3 * i + 2]), None => (0.0, 0.0, 1.0) };
            let lam = (nx * l[0] + ny * l[1] + nz * l[2]).abs().min(1.0);
            let view = (nx * dirs[3 * i] + ny * dirs[3 * i + 1] + nz * dirs[3 * i + 2]).abs().min(1.0);
            let k = 0.35 + 0.45 * lam + 0.2 * view;
            let fog = (-d / 45.0).exp();
            for c in 0..3 { col[c] = (base[c] as f64 * (k * fog) + 0.55 * (1.0 - fog)) as f32; }
        }
        for c in 0..3 {
            let v = col[c].clamp(0.0, 1.0);
            let f = v * 255.0f32;
            let mut x = f as f64;
            if let Some(rg) = rng.as_deref_mut() { if sigma > 0.0 { x += rg.gauss() * sigma; } }
            let x = x.clamp(0.0, 255.0);
            out[3 * i + c] = x as u8;
        }
    }
    out
}

// ------------------------------------------------------------------ sensors
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CamKind { Camera, Stereo, Tof }

fn library(kind: &str) -> Value {
    match kind {
        "camera" => json!({"model": "通用 USB RGB 相机", "width": 320, "height": 240, "hfov_deg": 70.0, "fps": 10.0, "range_max": 30.0, "pixel_noise": 2.0}),
        "stereo" => json!({"model": "Intel RealSense D435 (仿)", "width": 424, "height": 240, "hfov_deg": 87.0, "fps": 10.0, "baseline_m": 0.05, "range_min": 0.3, "range_max": 10.0, "subpixel_px": 0.08, "pixel_noise": 2.0}),
        "tof" => json!({"model": "PMD/Sunny 224×172 ToF (仿)", "width": 224, "height": 172, "hfov_deg": 62.0, "fps": 10.0, "range_min": 0.1, "range_max": 4.0, "noise_abs_m": 0.005, "noise_rel": 0.004}),
        _ => json!({}),
    }
}

#[derive(Clone, Debug)]
pub enum Plane { Rgb(Vec<u8>), Depth(Vec<f32>), Amp(Vec<u16>), Points(Vec<f32>) }

#[derive(Clone, Debug)]
pub struct CamFrames { pub streams: Vec<(String, Plane)> }

pub struct Camera {
    pub kind: CamKind,
    pub cfg: Value,
    pub name: String,
    pub frame_id: String,
    pub m: [f64; 3],
    pub r: [f64; 9],
    pub w: usize, pub h: usize,
    pub hfov: f64, pub vfov: f64, pub fps: f64,
    pub range_min: f64, pub range_max: f64,
    pub fx: f64, pub fy: f64, pub cx: f64, pub cy: f64,
    pub axis_cos: Vec<f64>,
    pub d_link: Vec<f64>,
    pub baseline: f64,
    pub frame: u64,
    pub last_ms: f64,
    pub rng: Rng,
}

fn cf(c: &Value, k: &str, d: f64) -> f64 { c.get(k).and_then(|v| v.as_f64()).unwrap_or(d) }

pub fn build_camera(cfg: &Value) -> Option<Camera> {
    let t = cfg.get("type").and_then(|t| t.as_str())?;
    let t = if t == "depthCamera" { "tof" } else { t };
    let kind = match t { "camera" => CamKind::Camera, "stereo" => CamKind::Stereo, "tof" => CamKind::Tof, _ => return None };
    let mut c = library(t);
    if let Some(o) = cfg.as_object() { for (k, v) in o { if !v.is_null() { c[k.as_str()] = v.clone(); } } }
    c["type"] = json!(t);
    let name = c.get("name").and_then(|n| n.as_str()).unwrap_or("camera").to_string();
    let (w, h) = (cf(&c, "width", 640.0) as usize, cf(&c, "height", 480.0) as usize);
    let hfov = cf(&c, "hfov_deg", 70.0).to_radians();
    let fx = (w as f64 / 2.0) / (hfov / 2.0).tan();
    let fy = fx;
    let (cx, cy) = ((w as f64 - 1.0) / 2.0, (h as f64 - 1.0) / 2.0);
    let vfov = 2.0 * ((h as f64 / 2.0) / fy).atan();
    let mut d_link = Vec::with_capacity(3 * w * h);
    let mut axis_cos = Vec::with_capacity(w * h);
    for v in 0..h { for u in 0..w {
        let (xo, yo) = ((u as f64 - cx) / fx, (v as f64 - cy) / fy);
        let ac = 1.0 / (1.0 + xo * xo + yo * yo).sqrt();
        axis_cos.push(ac);
        d_link.extend_from_slice(&[ac, -xo * ac, -yo * ac]);
    } }
    let range_min = match kind { CamKind::Stereo => cf(&c, "range_min", 0.3), CamKind::Tof => cf(&c, "range_min", 0.1), _ => cf(&c, "range_min", 0.05) };
    Some(Camera {
        kind, frame_id: format!("{name}_optical_frame"), name: name.clone(),
        m: [cf(&c, "x", 0.0), cf(&c, "y", 0.0), cf(&c, "z", 0.5)],
        r: rot_rpy(cf(&c, "roll", 0.0), cf(&c, "pitch", 0.0), cf(&c, "yaw", 0.0)),
        w, h, hfov, vfov, fps: cf(&c, "fps", 10.0), range_min, range_max: cf(&c, "range_max", 30.0), fx, fy, cx, cy, axis_cos, d_link,
        baseline: cf(&c, "baseline_m", 0.05), frame: 0, last_ms: 0.0, rng: Rng::new(0x9e3779b97f4a7c15 ^ name.bytes().fold(7u64, |a, b| a.wrapping_mul(131).wrapping_add(b as u64))), cfg: c,
    })
}

impl Camera {
    pub fn kind_name(&self) -> &'static str { match self.kind { CamKind::Camera => "camera", CamKind::Stereo => "stereo", CamKind::Tof => "tof" } }
    pub fn streams(&self) -> Vec<&'static str> {
        match self.kind { CamKind::Camera => vec!["rgb"], CamKind::Stereo => vec!["left", "right", "depth"], CamKind::Tof => vec!["depth", "amplitude", "points"] }
    }
    pub fn k(&self) -> [f64; 9] { [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0] }
    pub fn info(&self) -> Value {
        let rnd = |v: f64, n: i32| { let m = 10f64.powi(n); (v * m).round() / m };
        let mut d = json!({"name": self.name, "type": self.kind_name(), "model": self.cfg.get("model"), "frame_id": self.frame_id, "width": self.w, "height": self.h,
            "hfov_deg": rnd(self.hfov.to_degrees(), 2), "vfov_deg": rnd(self.vfov.to_degrees(), 2), "fps": self.fps,
            "K": self.k().iter().map(|k| json!(rnd(*k, 4))).collect::<Vec<_>>(), "range": [self.range_min, self.range_max],
            "mount": {"x": cf(&self.cfg, "x", 0.0), "y": cf(&self.cfg, "y", 0.0), "z": cf(&self.cfg, "z", 0.0), "roll": cf(&self.cfg, "roll", 0.0), "pitch": cf(&self.cfg, "pitch", 0.0), "yaw": cf(&self.cfg, "yaw", 0.0)},
            "streams": self.streams()});
        if self.kind == CamKind::Stereo {
            d["baseline_m"] = json!(self.baseline);
            d["depth_sigma_at_2m"] = json!(rnd(4.0 * cf(&self.cfg, "subpixel_px", 0.08) / (self.fx * self.baseline), 4));
        }
        d
    }

    fn pose(&self, x: f64, y: f64, th: f64, offset_y: f64) -> ([f64; 3], Vec<f64>) {
        let (s, c) = th.sin_cos();
        let rz = [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0];
        let mut rw = [0.0; 9];
        for i in 0..3 { for j in 0..3 { rw[3 * i + j] = (0..3).map(|k| rz[3 * i + k] * self.r[3 * k + j]).sum(); } }
        let off = [self.r[1] * offset_y, self.r[4] * offset_y, self.r[7] * offset_y];
        let lo = [self.m[0] + off[0], self.m[1] + off[1], self.m[2] + off[2]];
        let o = [x + c * lo[0] - s * lo[1], y + s * lo[0] + c * lo[1], lo[2]];
        let mut dw = Vec::with_capacity(self.d_link.len());
        for p in self.d_link.chunks_exact(3) {
            dw.push(rw[0] * p[0] + rw[1] * p[1] + rw[2] * p[2]);
            dw.push(rw[3] * p[0] + rw[4] * p[1] + rw[5] * p[2]);
            dw.push(rw[6] * p[0] + rw[7] * p[1] + rw[8] * p[2]);
        }
        (o, dw)
    }

    fn cast(&self, scene: &CamScene, x: f64, y: f64, th: f64, normals: bool, offset_y: f64, threads: usize) -> ([f64; 3], Vec<f64>, CastResult) {
        let (o, dw) = self.pose(x, y, th, offset_y);
        let r = cast_prims(scene, o, &dw, self.range_max, normals, threads);
        (o, dw, r)
    }

    pub fn capture(&mut self, scene: &CamScene, x: f64, y: f64, th: f64, noise: bool, threads: usize) -> CamFrames {
        self.frame += 1;
        let (w, h) = (self.w, self.h);
        let n = w * h;
        let pix_noise = if noise { cf(&self.cfg, "pixel_noise", 2.0) } else { 0.0 };
        match self.kind {
            CamKind::Camera => {
                let (o, dw, r) = self.cast(scene, x, y, th, true, 0.0, threads);
                let img = shade(scene, o, &dw, &r, pix_noise, Some(&mut self.rng));
                CamFrames { streams: vec![("rgb".into(), Plane::Rgb(img))] }
            }
            CamKind::Stereo => {
                let (o, dw, r) = self.cast(scene, x, y, th, true, 0.0, threads);
                let (o2, dw2, r2) = self.cast(scene, x, y, th, true, -self.baseline, threads);
                let left = shade(scene, o, &dw, &r, pix_noise, Some(&mut self.rng));
                let right = shade(scene, o2, &dw2, &r2, pix_noise, Some(&mut self.rng));
                let mut z: Vec<f32> = (0..n).map(|i| (r.dist[i] * self.axis_cos[i]) as f32).collect();
                if noise {
                    let sp = cf(&self.cfg, "subpixel_px", 0.08);
                    for i in 0..n {
                        let zi = z[i] as f64;
                        let sig = zi * zi * sp / (self.fx * self.baseline);
                        let sig = if sig.is_finite() { sig } else { 0.0 };
                        z[i] = (zi + (self.rng.gauss() as f32) as f64 * sig) as f32;
                    }
                }
                for v in z.iter_mut() { if !v.is_finite() || (*v as f64) < self.range_min || (*v as f64) > self.range_max { *v = f32::NAN; } }
                for i in 0..n {
                    let dd = r2.dist[i] - r.dist[i];
                    let dd = if dd.is_nan() { 0.0 } else if dd == f64::INFINITY { 1e9 } else if dd == f64::NEG_INFINITY { -1e9 } else { dd };
                    let zz = if z[i].is_nan() { 1e9 } else { z[i] as f64 };
                    if dd.abs() > 0.25 * zz { z[i] = f32::NAN; }
                }
                CamFrames { streams: vec![("left".into(), Plane::Rgb(left)), ("right".into(), Plane::Rgb(right)), ("depth".into(), Plane::Depth(z))] }
            }
            CamKind::Tof => {
                let (_o, dw, r) = self.cast(scene, x, y, th, true, 0.0, threads);
                let mut z: Vec<f32> = (0..n).map(|i| (r.dist[i] * self.axis_cos[i]) as f32).collect();
                let nrm = r.nrm.as_ref().unwrap();
                let mut amp: Vec<f64> = (0..n).map(|i| {
                    let cos_inc = (nrm[3 * i] * dw[3 * i] + nrm[3 * i + 1] * dw[3 * i + 1] + nrm[3 * i + 2] * dw[3 * i + 2]).abs();
                    if z[i].is_finite() { 2000.0 * cos_inc / (z[i] as f64).max(0.05).powi(2) } else { 0.0 }
                }).collect();
                if noise {
                    let (na, nr) = (cf(&self.cfg, "noise_abs_m", 0.005), cf(&self.cfg, "noise_rel", 0.004));
                    for i in 0..n {
                        let zi = if z[i].is_finite() { z[i] as f64 } else { 0.0 };
                        z[i] = (z[i] as f64 + self.rng.gauss() * (na + nr * zi)) as f32;
                        amp[i] *= 1.0 + self.rng.gauss() * 0.03;
                    }
                }
                let mut pts: Vec<f32> = vec![];
                for i in 0..n {
                    let zi = z[i] as f64;
                    let valid = zi.is_finite() && zi >= self.range_min && zi <= self.range_max && amp[i] > 20.0;
                    if !valid { z[i] = f32::NAN; continue; }
                    let (u, v) = (i % w, i / w);
                    pts.extend_from_slice(&[((u as f64 - self.cx) / self.fx * zi) as f32, ((v as f64 - self.cy) / self.fy * zi) as f32, zi as f32]);
                }
                let amp16: Vec<u16> = amp.iter().map(|a| a.clamp(0.0, 65535.0) as u16).collect();
                CamFrames { streams: vec![("depth".into(), Plane::Depth(z)), ("amplitude".into(), Plane::Amp(amp16)), ("points".into(), Plane::Points(pts))] }
            }
        }
    }
}

