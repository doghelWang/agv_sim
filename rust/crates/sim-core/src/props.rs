//! Warehouse dressing: turns a bare scenario (walls + shelves + topology) into a plausible facility.
//!
//! * static props: pillars on a structural grid, charging stations, pallet racking (uprights + beams), roller conveyor
//!   stations next to docking points, safety barriers;
//! * cargo matching the vehicle family: jacking racks, EUR pallets (with box stacks, on the floor and on rack beams),
//!   totes on conveyors, tow carts.
//!
//! Everything is placed **off the lanes** (distance to every topology edge > lane clearance) unless the `demo` preset is
//! used, which additionally puts one pick-up object at each station so a task flow can be tried immediately.
use serde_json::{json, Map, Value};

#[derive(Clone, Debug, Default)]
pub struct Dressing { pub props: Vec<Map<String, Value>>, pub cargo: Vec<(String, String, f64, f64, f64, Option<f64>, Option<[f64; 3]>)> }

fn seg_dist(px: f64, py: f64, a: [f64; 4]) -> f64 {
    let (ax, ay, bx, by) = (a[0], a[1], a[2], a[3]);
    let (dx, dy) = (bx - ax, by - ay);
    let l2 = dx * dx + dy * dy;
    let t = if l2 < 1e-12 { 0.0 } else { (((px - ax) * dx + (py - ay) * dy) / l2).clamp(0.0, 1.0) };
    (px - ax - t * dx).hypot(py - ay - t * dy)
}

/// rectangle (center, half sizes, yaw) clearance against a segment list: min distance from the rectangle outline samples
fn rect_clear(cx: f64, cy: f64, hx: f64, hy: f64, yaw: f64, segs: &[[f64; 4]]) -> f64 {
    let (s, c) = yaw.sin_cos();
    let mut best = f64::INFINITY;
    for i in 0..=4 { for j in 0..=4 {
        let (lx, ly) = (-hx + 2.0 * hx * i as f64 / 4.0, -hy + 2.0 * hy * j as f64 / 4.0);
        let (x, y) = (cx + c * lx - s * ly, cy + s * lx + c * ly);
        for g in segs { best = best.min(seg_dist(x, y, *g)); }
    } }
    best
}

fn prop(typ: &str, id: &str, x: f64, y: f64, yaw: f64, w: f64, h: f64, z: f64) -> Map<String, Value> {
    let mut m = Map::new();
    for (k, v) in [("prop", json!(true)), ("type", json!(typ)), ("id", json!(id)), ("x", json!(x)), ("y", json!(y)), ("yaw", json!(yaw)), ("w", json!(w)), ("h", json!(h)), ("z", json!(z))] { m.insert(k.into(), v); }
    m
}

struct Placer { lanes: Vec<[f64; 4]>, walls: Vec<[f64; 4]>, taken: Vec<(f64, f64, f64, f64, f64)>, lane_clear: f64 }
impl Placer {
    fn ok(&self, x: f64, y: f64, hx: f64, hy: f64, yaw: f64, wall_clear: f64) -> bool {
        if rect_clear(x, y, hx, hy, yaw, &self.lanes) < self.lane_clear { return false; }
        if rect_clear(x, y, hx, hy, yaw, &self.walls) < wall_clear { return false; }
        !self.taken.iter().any(|(tx, ty, thx, thy, _)| (x - tx).abs() < hx + thx + 0.25 && (y - ty).abs() < hy + thy + 0.25)
    }
    fn take(&mut self, x: f64, y: f64, hx: f64, hy: f64, yaw: f64) { self.taken.push((x, y, hx, hy, yaw)); }
}

#[allow(clippy::too_many_arguments)]
fn place_item(d: &mut Dressing, item: &str, cx: f64, cy: f64, yaw: f64, along: f64, depth: f64, slot: usize, next: &mut impl FnMut(&str) -> String) {
    let (s, c) = yaw.sin_cos();
    match item {
        "pallet_stack" => {
            d.cargo.push(("pallet".into(), next("PAL_"), cx, cy, yaw, Some(0.0), None));
            if slot % 2 == 0 {
                for (k, (dx, dy)) in [(-0.3, -0.2), (0.3, -0.2), (-0.3, 0.2), (0.3, 0.2)].iter().enumerate() {
                    d.cargo.push(("box".into(), next("BOX_"), cx + c * dx - s * dy, cy + s * dx + c * dy, yaw, None, Some([0.58, 0.38, 0.4 + 0.05 * (k % 2) as f64])));
                }
            }
        }
        "rack" => d.cargo.push(("rack".into(), next("RACK_"), cx, cy, yaw, Some(0.0), None)),
        "cart" => d.cargo.push(("cart".into(), next("CART_"), cx, cy, yaw, Some(0.0), None)),
        "tote_stack" => {
            for (dx, dy) in [(-0.32, 0.0), (0.32, 0.0)] { d.cargo.push(("tote".into(), next("TOTE_"), cx + c * dx - s * dy, cy + s * dx + c * dy, yaw, None, None)); }
        }
        "barrier" => d.props.push(prop("barrier", &next("BAR_"), cx, cy, yaw, along, 0.08, 1.0)),
        "pallet_rack" => {
            // two bays: uprights + 2 beam levels, a pallet on each first-level bay
            for k in 0..3 {
                let u = -along / 2.0 + 0.05 + k as f64 * (along - 0.1) / 2.0;
                for v in [-depth / 2.0 + 0.04, depth / 2.0 - 0.04] { d.props.push(prop("rack_upright", &next("UP_"), cx + c * u - s * v, cy + s * u + c * v, yaw, 0.08, 0.08, 3.0)); }
            }
            for lvl in [1.25, 2.5] {
                for v in [-depth / 2.0 + 0.04, depth / 2.0 - 0.04] {
                    let mut m = prop("rack_beam", &next("BM_"), cx - s * v, cy + c * v, yaw, along - 0.1, 0.06, 0.12);
                    m.insert("z0".into(), json!(lvl));
                    d.props.push(m);
                }
            }
            for bay in [-1.0, 1.0] {
                let u = bay * (along - 0.1) / 4.0;
                d.cargo.push(("pallet".into(), next("PAL_"), cx + c * u, cy + s * u, yaw, Some(1.37), Some([1.2, 0.8, 0.144])));
            }
        }
        _ => {}
    }
}

/// `archetype`: assembly archetype of the robot; `deck_z`: roller deck height; `mode`: "auto" | "demo" | "none"
pub fn dress(sc: &Value, archetype: &str, robot_w: f64, deck_z: f64, mode: &str) -> Dressing {
    let mut d = Dressing::default();
    if mode == "none" { return d; }
    let nodes = sc.get("nodes").and_then(|n| n.as_object()).cloned().unwrap_or_default();
    let pt = |k: &str| nodes.get(k).and_then(|p| Some((p.get(0)?.as_f64()?, p.get(1)?.as_f64()?)));
    let lanes: Vec<[f64; 4]> = sc.get("connections").and_then(|c| c.as_array()).map(|cs| cs.iter().filter_map(|c| {
        let (a, b) = (pt(c.get(0)?.as_str()?)?, pt(c.get(1)?.as_str()?)?);
        Some([a.0, a.1, b.0, b.1])
    }).collect()).unwrap_or_default();
    let mut lanes = lanes;
    for (_, p) in &nodes { if let (Some(x), Some(y)) = (p.get(0).and_then(|v| v.as_f64()), p.get(1).and_then(|v| v.as_f64())) { lanes.push([x, y, x, y]); } }
    let walls: Vec<[f64; 4]> = sc.get("walls").and_then(|w| w.as_array()).map(|a| a.iter().filter_map(|w| { let v = w.as_array()?; Some([v.first()?.as_f64()?, v.get(1)?.as_f64()?, v.get(2)?.as_f64()?, v.get(3)?.as_f64()?]) }).collect()).unwrap_or_default();
    if walls.len() < 4 { return d; }
    let xs: Vec<f64> = walls[..4].iter().flat_map(|w| [w[0], w[2]]).collect();
    let ys: Vec<f64> = walls[..4].iter().flat_map(|w| [w[1], w[3]]).collect();
    let (x0, x1) = (xs.iter().cloned().fold(f64::INFINITY, f64::min), xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
    let (y0, y1) = (ys.iter().cloned().fold(f64::INFINITY, f64::min), ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
    let mut pl = Placer { lanes, walls, taken: vec![], lane_clear: (robot_w / 2.0 + 0.2).max(0.55) };
    let mut nid = 0usize;
    let mut next = move |p: &str| { nid += 1; format!("{p}{nid}") };

    // ---- structural pillars (6 m grid, only where they do not obstruct lanes)
    let step = 6.0;
    let mut gx = (x0 / step).ceil() * step;
    while gx < x1 - 0.5 {
        let mut gy = (y0 / step).ceil() * step;
        while gy < y1 - 0.5 {
            if pl.ok(gx, gy, 0.3, 0.3, 0.0, 0.8) { d.props.push(prop("pillar", &next("PIL_"), gx, gy, 0.0, 0.5, 0.5, 6.0)); pl.take(gx, gy, 0.25, 0.25, 0.0); }
            gy += step;
        }
        gx += step;
    }
    // ---- charging station at the charging node (or the origin), against the nearest wall
    let charge = nodes.iter().find(|(k, _)| k.to_uppercase().contains("CHARG")).and_then(|(k, _)| pt(k))
        .or_else(|| sc.get("origin").and_then(|o| Some((o.get("x")?.as_f64()?, o.get("y")?.as_f64()?))));
    if let Some((cx, cy)) = charge {
        let cand = [(cx - x0, std::f64::consts::PI, (x0 + 0.2, cy)), (x1 - cx, 0.0, (x1 - 0.2, cy)), (cy - y0, -std::f64::consts::FRAC_PI_2, (cx, y0 + 0.2)), (y1 - cy, std::f64::consts::FRAC_PI_2, (cx, y1 - 0.2))];
        let (dist, dir, pos) = cand.iter().cloned().fold((f64::INFINITY, 0.0, (0.0, 0.0)), |b, c| if c.0 < b.0 { c } else { b });
        if dist < 4.0 {
            // charger faces the node: its local -x (back plate) against the wall
            let yaw = dir + std::f64::consts::PI;
            let (s, c) = dir.sin_cos();
            let (px, py) = (pos.0 - c * 0.15, pos.1 - s * 0.15);
            d.props.push(prop("charger", &next("CHG_"), px, py, yaw, 0.3, 0.6, 0.9));
            pl.take(px, py, 0.3, 0.3, yaw);
        }
    }
    // ---- docking furniture at stations
    let stations: Vec<(f64, f64, f64, String)> = sc.get("stations").and_then(|s| s.as_array()).map(|a| a.iter().filter_map(|s| Some((s.get("x")?.as_f64()?, s.get("y")?.as_f64()?, s.get("dock_yaw").and_then(|v| v.as_f64()).unwrap_or(0.0),
        s.get("name").or_else(|| s.get("id")).and_then(|v| v.as_str()).unwrap_or("ST").to_string()))).collect()).unwrap_or_default();
    for (k, (sx, sy, yaw, _name)) in stations.iter().enumerate() {
        match archetype {
            "roller" => {
                // conveyor on the left of the docking pose, top = deck height; the first one holds a tote
                let (s, c) = yaw.sin_cos();
                let off = robot_w / 2.0 + 0.45 + 0.04;
                let (cx, cy) = (sx - s * off, sy + c * off);
                d.props.push({ let mut m = prop("conveyor", &next("CV_"), cx, cy, *yaw, 1.0, 0.9, deck_z); m.insert("roll_axis".into(), json!("x")); m });
                pl.take(cx, cy, 0.5, 0.45, *yaw);
                if k % 2 == 0 { d.cargo.push(("tote".into(), next("TOTE_"), cx, cy, *yaw, None, None)); }
            }
            "jacking" if mode == "demo" => d.cargo.push(("rack".into(), next("RACK_"), *sx, *sy, *yaw, Some(0.0), None)),
            "reach_fork" | "stacker_fork" | "pallet_truck" if mode == "demo" => {
                let (s, c) = yaw.sin_cos();
                let off = 1.0 + 0.6;
                d.cargo.push(("pallet".into(), next("PAL_"), sx + c * off, sy + s * off, *yaw, None, None));
            }
            "tugger" if mode == "demo" => {
                let (s, c) = yaw.sin_cos();
                let off = 2.0;
                d.cargo.push(("cart".into(), next("CART_"), sx - c * off, sy - s * off, *yaw, Some(0.0), None));
            }
            _ => {}
        }
    }
    // ---- storage zones along the outer walls (off-lane)
    let fam_item = |i: usize| -> &'static str {
        match archetype {
            "jacking" => if i % 3 == 2 { "pallet_stack" } else { "rack" },
            "reach_fork" | "stacker_fork" => if i % 4 == 0 { "pallet_rack" } else { "pallet_stack" },
            "pallet_truck" => "pallet_stack",
            "tugger" => if i % 2 == 0 { "cart" } else { "pallet_stack" },
            "roller" => if i % 2 == 0 { "tote_stack" } else { "pallet_stack" },
            _ => ["pallet_stack", "tote_stack", "barrier"][i % 3],
        }
    };
    // candidate grid over the whole floor; each family cycles through its item list, every item tried at 0° and 90°
    let mut slot = 0usize;
    let area = (x1 - x0) * (y1 - y0);
    let cap = ((area / 18.0) as usize).clamp(6, 40);
    let size_of = |item: &str| -> (f64, f64) { match item { "pallet_rack" => (2.9, 1.1), "rack" => (robot_w + 0.4, robot_w + 0.4), "cart" => (1.6, 0.9), "barrier" => (1.5, 0.3), "tote_stack" => (1.3, 0.5), _ => (1.25, 0.85) } };
    let mut placed = 0usize;
    let g = 0.4;
    let mut cy = y0 + 0.5;
    'outer: while cy < y1 - 0.5 {
        let mut cx = x0 + 0.5;
        while cx < x1 - 0.5 {
            if placed >= cap { break 'outer; }
            for attempt in 0..3 {
                let item = fam_item(slot + attempt);
                let (along, depth) = size_of(item);
                let mut done = false;
                for yaw in [0.0, std::f64::consts::FRAC_PI_2] {
                    if !pl.ok(cx, cy, along / 2.0, depth / 2.0, yaw, 0.15) { continue; }
                    let (hx, hy) = if yaw == 0.0 { (along / 2.0, depth / 2.0) } else { (depth / 2.0, along / 2.0) };
                    pl.take(cx, cy, hx, hy, yaw);
                    slot += attempt + 1;
                    placed += 1;
                    place_item(&mut d, item, cx, cy, yaw, along, depth, slot, &mut next);
                    done = true;
                    break;
                }
                if done { break; }
            }
            cx += g;
        }
        cy += g;
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn dressing_stays_off_lanes() {
        let sc: Value = serde_json::from_str(&std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/scenarios.json")).unwrap()).unwrap();
        let mut total = 0;
        for (id, s) in sc.as_object().unwrap() {
            for arch in ["jacking", "roller", "reach_fork", "tugger", "none"] {
                let d = dress(s, arch, 1.0, 0.6, "auto");
                total += d.cargo.len() + d.props.len();
                println!("{id}/{arch}: {} props, {} cargo", d.props.len(), d.cargo.len());
                let nodes = s["nodes"].as_object().unwrap();
                for p in d.props.iter().filter(|p| p["type"] != "conveyor") {
                    let (x, y) = (p["x"].as_f64().unwrap(), p["y"].as_f64().unwrap());
                    for (_, n) in nodes { assert!((n[0].as_f64().unwrap() - x).hypot(n[1].as_f64().unwrap() - y) > 0.7, "{id}/{arch}: prop {p:?} on a node"); }
                }
            }
        }
        assert!(total > 100, "dressing produced {total} items");
    }
}
