//! Model × scenario fit check: sweeps each cmodel's footprint along every topology lane (heading along the lane, both
//! directions) and rotates it in place at every node, against walls/shelves alone and with the warehouse dressing.
//! cargo run --release -p sim-core --example fit_check -- a.cmodel b.cmodel ...
use serde_json::{json, Map, Value};
use sim_core::core::*;
use sim_core::geom::collides;

fn inflate(fp: &[[f64; 2]], m: f64) -> Vec<[f64; 2]> {
    let (cx, cy) = (fp.iter().map(|p| p[0]).sum::<f64>() / fp.len() as f64, fp.iter().map(|p| p[1]).sum::<f64>() / fp.len() as f64);
    fp.iter().map(|p| { let (dx, dy) = (p[0] - cx, p[1] - cy); [p[0] + m * dx.signum(), p[1] + m * dy.signum()] }).collect()
}

fn main() {
    let scen: Map<String, Value> = serde_json::from_str::<Value>(&std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/scenarios.json")).unwrap()).unwrap().as_object().unwrap().clone();
    let mut report = vec![];
    for path in std::env::args().skip(1) {
        let d = cmodel::decode_cmodel_bytes(&std::fs::read(&path).unwrap()).unwrap();
        let pp = std::path::Path::new(&path); let mut name = pp.file_name().unwrap().to_string_lossy().to_string(); if name == "source.cmodel" { name = pp.parent().unwrap().file_name().unwrap().to_string_lossy().to_string(); }
        let spec = cmodel::spec::extract_robot_spec(&d["CompDesc"], &name, "full", None);
        let ch = &spec["chassis"];
        for (sid, _) in &scen {
            let mut st = Settings::default(); st.noise = false;
            let mut c = SimCore::new(spec.clone(), &scen, sid, None, None, st, 1).unwrap();
            let arch = c.asm["archetype"].as_str().unwrap_or("").to_string();
            let fp = c.footprint.clone();
            // forks / carried rack stick out of the body footprint: use the outline with the actuator envelope
            let mut fpe = fp.clone();
            if let Some(f) = &c.fork { let hy = (f.sp.spacing / 2.0 + f.sp.tine_w / 2.0).max(fp.iter().map(|p| p[1].abs()).fold(0.0, f64::max));
                let x1 = fp.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max).max(f.sp.at(1.0)); let x0 = fp.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min).min(f.sp.at(1.0));
                fpe = vec![[x1, hy], [x1, -hy], [x0, -hy], [x0, hy]]; }
            let sc = &scen[sid];
            let nodes = sc["nodes"].as_object().unwrap().clone();
            let conns: Vec<(String, String)> = sc["connections"].as_array().unwrap().iter().map(|c| (c[0].as_str().unwrap().to_string(), c[1].as_str().unwrap().to_string())).collect();
            let mut res = Map::new();
            for (mode, dress) in [("static", "none"), ("dressed", "auto")] {
                c.set_dressing(dress).unwrap();
                let segs = c.world.segments.clone();
                let (mut lane_hits, mut lane_tight, mut rot_hits) = (vec![], 0, vec![]);
                for (a, b) in &conns {
                    let (pa, pb) = (&nodes[a], &nodes[b]);
                    let (ax, ay, bx, by) = (pa[0].as_f64().unwrap(), pa[1].as_f64().unwrap(), pb[0].as_f64().unwrap(), pb[1].as_f64().unwrap());
                    let l = (bx - ax).hypot(by - ay);
                    if l < 1e-3 { continue; }
                    for dir in [0.0, std::f64::consts::PI] {
                        let th = (by - ay).atan2(bx - ax) + dir;
                        let n = (l / 0.1).ceil() as usize;
                        let mut hit = false; let mut tight = false;
                        for k in 0..=n { let t = k as f64 / n as f64; let (x, y) = (ax + (bx - ax) * t, ay + (by - ay) * t);
                            if collides(&fpe, x, y, th, &segs).is_some() { hit = true; break; }
                            if collides(&inflate(&fpe, 0.1), x, y, th, &segs).is_some() { tight = true; } }
                        if hit { lane_hits.push(format!("{a}->{b}{}", if dir > 0.0 { "(rev)" } else { "" })); } else if tight { lane_tight += 1; }
                    }
                }
                for (nm, p) in &nodes {
                    let (x, y) = (p[0].as_f64().unwrap(), p[1].as_f64().unwrap());
                    let deg = conns.iter().filter(|(a, b)| a == nm || b == nm).count();
                    if deg < 2 { continue; }
                    if (0..36).any(|k| collides(&fpe, x, y, k as f64 * 10f64.to_radians(), &segs).is_some()) { rot_hits.push(nm.clone()); }
                }
                res.insert(mode.into(), json!({"lane_collisions": lane_hits, "lanes_tight_lt_10cm": lane_tight, "rotate_blocked_nodes": rot_hits,
                    "props": c.props_list().len(), "cargo": c.cargo_list().len()}));
            }
            let a = &c.asm["actuator"];
            report.push(json!({"model": name, "archetype": arch, "reason": c.asm["archetype_source"], "swept_r": c.swept_radius(), "deck_z": a.get("deck_z_m"), "rack_clear": a.get("rack_clearance_m"),
                "missing": c.asm["missing"].as_array().map(|m| m.len()), "size": [ch["length_m"], ch["width_m"], ch["height_m"]], "rotate_d": ch["rotate_diameter_m"], "scenario": sid, "result": res}));
        }
    }
    println!("{}", serde_json::to_string_pretty(&report).unwrap());
}
