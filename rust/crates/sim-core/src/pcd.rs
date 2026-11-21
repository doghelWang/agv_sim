//! SLAM-style point-cloud map (PCD v0.7) sampled from the scene geometry: walls/shelves, props, cargo, optional floor/ceiling.
//! Points lie on the visible surfaces (box faces / cylinder mantles) on a regular grid of `res`, with intensity from the
//! surface reflectance — the same primitives the lidar and the cameras see.
use crate::cameras::obstacle_prims;
use crate::world::*;

pub struct PcdOpts { pub res: f64, pub floor: bool, pub ceiling: bool, pub z_max: f64, pub max_points: usize, pub binary: bool }
impl Default for PcdOpts { fn default() -> Self { PcdOpts { res: 0.05, floor: false, ceiling: false, z_max: CEILING_HEIGHT, max_points: 4_000_000, binary: true } } }

fn lum(c: [f32; 3]) -> f32 { (0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2]) * 255.0 }

/// sample one primitive; returns false when the point budget is exhausted
fn sample_prim(p: &Prim, inten: f32, res: f64, z_max: f64, out: &mut Vec<[f32; 4]>, max: usize) -> bool {
    let r = &p[3..12];
    let h = [p[12], p[13], p[14]];
    let mut push = |lx: f64, ly: f64, lz: f64, out: &mut Vec<[f32; 4]>| {
        let x = p[0] + r[0] * lx + r[1] * ly + r[2] * lz;
        let y = p[1] + r[3] * lx + r[4] * ly + r[5] * lz;
        let z = p[2] + r[6] * lx + r[7] * ly + r[8] * lz;
        if z < 0.005 || z > z_max { return; }
        out.push([x as f32, y as f32, z as f32, inten]);
    };
    let steps = |half: f64| ((2.0 * half / res).ceil() as usize).max(1);
    if p[15] < 0.5 {
        // 6 faces of the box
        for axis in 0..3 {
            let (a, b) = ((axis + 1) % 3, (axis + 2) % 3);
            let (na, nb) = (steps(h[a]), steps(h[b]));
            for sgn in [-1.0, 1.0] {
                for i in 0..=na { for j in 0..=nb {
                    let mut l = [0.0; 3];
                    l[axis] = sgn * h[axis];
                    l[a] = -h[a] + 2.0 * h[a] * i as f64 / na as f64;
                    l[b] = -h[b] + 2.0 * h[b] * j as f64 / nb as f64;
                    push(l[0], l[1], l[2], out);
                    if out.len() >= max { return false; }
                } }
            }
        }
    } else {
        let (rad, hz) = (h[0], h[2]);
        let na = ((2.0 * std::f64::consts::PI * rad / res).ceil() as usize).max(6);
        let nz = steps(hz);
        for i in 0..na {
            let a = 2.0 * std::f64::consts::PI * i as f64 / na as f64;
            for j in 0..=nz { push(rad * a.cos(), rad * a.sin(), -hz + 2.0 * hz * j as f64 / nz as f64, out); }
            if out.len() >= max { return false; }
        }
        let nr = steps(rad / 2.0);
        for k in 1..=nr {
            let rr = rad * k as f64 / nr as f64;
            let n = ((2.0 * std::f64::consts::PI * rr / res).ceil() as usize).max(6);
            for i in 0..n { let a = 2.0 * std::f64::consts::PI * i as f64 / n as f64; push(rr * a.cos(), rr * a.sin(), hz, out); push(rr * a.cos(), rr * a.sin(), -hz, out); }
        }
    }
    out.len() < max
}

pub fn sample_world(world: &World, o: &PcdOpts) -> Vec<[f32; 4]> {
    let mut pts: Vec<[f32; 4]> = vec![];
    let res = o.res.clamp(0.01, 1.0);
    for s in &world.static_segments {
        let (x0, y0, x1, y1, h) = (s[0], s[1], s[2], s[3], s[4]);
        let l = (x1 - x0).hypot(y1 - y0);
        if l < 0.01 { continue; }
        let mut p = [0.0f64; 18];
        p[0] = (x0 + x1) / 2.0; p[1] = (y0 + y1) / 2.0; p[2] = h / 2.0;
        p[3..12].copy_from_slice(&rot_z((y1 - y0).atan2(x1 - x0)));
        p[12] = l / 2.0; p[13] = 0.025; p[14] = h / 2.0;
        let cat = if h >= CEILING_HEIGHT - 1e-3 { "wall" } else { "shelf" };
        if !sample_prim(&p, lum(CAT_COLORS[cat_id(cat)]), res, o.z_max, &mut pts, o.max_points) { return pts; }
    }
    let (mut rgb, mut mat) = (vec![], vec![]);
    for ob in &world.obstacles {
        if ob.get("motion").map(|m| !m.is_null()).unwrap_or(false) || ob.get("type").and_then(|t| t.as_str()) == Some("person") { continue; }   // movers are not part of a map
        rgb.clear(); mat.clear();
        for p in obstacle_prims(ob, &mut rgb, &mut mat) {
            let c = rgb.get(p[16] as usize).copied().unwrap_or([0.5; 3]);
            if !sample_prim(&p, lum(c), res, o.z_max, &mut pts, o.max_points) { return pts; }
        }
    }
    let b = world.bounds;
    let fl = res * 2.0;
    for (on, z, inten) in [(o.floor, 0.0, 90.0f32), (o.ceiling && CEILING_HEIGHT <= o.z_max, CEILING_HEIGHT, 200.0)] {
        if !on { continue; }
        let (nx, ny) = (((b[2] - b[0]) / fl) as usize, ((b[3] - b[1]) / fl) as usize);
        for i in 0..=nx { for j in 0..=ny {
            pts.push([(b[0] + i as f64 * fl) as f32, (b[1] + j as f64 * fl) as f32, z as f32, inten]);
            if pts.len() >= o.max_points { return pts; }
        } }
    }
    pts
}

pub fn encode(pts: &[[f32; 4]], binary: bool) -> Vec<u8> {
    let hdr = format!("# .PCD v0.7 - agv_sim scene map (frame: map)\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA {}\n", if binary { "binary" } else { "ascii" }, n = pts.len());
    let mut out = hdr.into_bytes();
    if binary {
        out.reserve(pts.len() * 16);
        for p in pts { for v in p { out.extend_from_slice(&v.to_le_bytes()); } }
    } else {
        for p in pts { out.extend_from_slice(format!("{:.3} {:.3} {:.3} {:.0}\n", p[0], p[1], p[2], p[3]).as_bytes()); }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pcd_has_walls_and_header() {
        let mut w = World::new();
        w.load_scenario(&serde_json::json!({"walls": [[-2.0, -2.0, 2.0, -2.0], [2.0, -2.0, 2.0, 2.0], [2.0, 2.0, -2.0, 2.0], [-2.0, 2.0, -2.0, -2.0]]}));
        let pts = sample_world(&w, &PcdOpts { res: 0.1, z_max: 3.0, ..Default::default() });
        assert!(pts.len() > 1000);
        assert!(pts.iter().all(|p| p[2] <= 3.0 + 1e-6));
        let b = encode(&pts[..10], false);
        let s = String::from_utf8(b).unwrap();
        assert!(s.contains("POINTS 10") && s.contains("DATA ascii"));
    }
}
