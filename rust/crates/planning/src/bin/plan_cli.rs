//! Parity-test CLI: JSON cases on stdin (one per line) → JSON results on stdout.
use planning::maneuver::{self, Corner, CornerMode};
use planning::protection;
use planning::route::{Obs, Planner};
use planning::scenario::Library;
use planning::sweep;
use serde_json::{json, Value};
use std::io::BufRead;

fn pt(v: &Value) -> (f64, f64) { (v[0].as_f64().unwrap(), v[1].as_f64().unwrap()) }
fn segs(v: &Value) -> Vec<[f64; 4]> { v.as_array().unwrap().iter().map(|w| [w[0].as_f64().unwrap(), w[1].as_f64().unwrap(), w[2].as_f64().unwrap(), w[3].as_f64().unwrap()]).collect() }

fn main() {
    let scen: serde_json::Map<String, Value> = serde_json::from_str::<Value>(include_str!("../../../../data/scenarios.json")).unwrap().as_object().cloned().unwrap();
    let lib = Library::from_json_map(&scen);
    let mut planners: std::collections::HashMap<String, Planner> = Default::default();
    for line in std::io::stdin().lock().lines() {
        let line = line.unwrap();
        if line.trim().is_empty() { continue; }
        let c: Value = serde_json::from_str(&line).unwrap();
        let out = match c["op"].as_str().unwrap() {
            "route" => {
                let sid = c["scenario"].as_str().unwrap().to_string();
                let key = format!("{}|{}", sid, c["fp"]);
                let p = planners.entry(key).or_insert_with(|| Planner::new(&lib, &sid));
                p.robot_half_width = c["rhw"].as_f64().unwrap_or(0.35);
                p.robot_circum_radius = c["rcr"].as_f64().unwrap_or(0.6);
                if let Some(f) = c["fp"].as_array() {
                    p.set_footprint(f[0].as_f64().unwrap(), f[1].as_f64().unwrap(), f[2].as_f64().unwrap(), f[3].as_f64().unwrap(), f[4].as_f64().unwrap(), CornerMode::parse(f[5].as_str().unwrap()));
                }
                p.rot_blocked = c["rot_blocked"].as_array().map(|a| a.iter().map(pt).collect()).unwrap_or_default();
                let obs: Vec<Obs> = c["obstacles"].as_array().map(|a| a.iter().map(Obs::from_value).collect()).unwrap_or_default();
                let r = if let Some(sy) = c["start_yaw"].as_f64() {
                    p.plan_route_dir(pt(&c["start"]), pt(&c["goal"]), &obs, sy, c["goal_yaw"].as_f64())
                } else { p.plan_route(pt(&c["start"]), pt(&c["goal"]), &obs) };
                json!({"points": r.points.iter().map(|p| json!([p.0, p.1])).collect::<Vec<_>>(), "labels": r.labels, "length": r.length,
                       "reverse": r.reverse, "blocked": r.blocked.iter().map(|b| json!([b.0, b.1, b.2])).collect::<Vec<_>>()})
            }
            "corner" => {
                let f = &c["a"];
                let (cr, clr) = maneuver::plan_corner(&segs(&f["segs"]), pt(&f["node"]), f["h1"].as_f64().unwrap(), f["h2"].as_f64().unwrap(),
                    f["len_in"].as_f64().unwrap(), f["len_out"].as_f64().unwrap(), f["head"].as_f64().unwrap(), f["tail"].as_f64().unwrap(),
                    f["hw"].as_f64().unwrap(), f["r_pref"].as_f64().unwrap(), f["clear_min"].as_f64().unwrap(), CornerMode::parse(f["mode"].as_str().unwrap()));
                let d = match cr { Corner::Rotate(r) => json!({"rotate": r}),
                    Corner::Arc { heading, turn, r, d, v, cx, cy } => json!({"heading": heading, "turn": turn, "R": r, "d": d, "v": v, "cx": cx, "cy": cy}) };
                json!([d, clr])
            }
            "astar" => {
                let a = planning::astar::AStar::new(c["res"].as_f64().unwrap(), c["infl"].as_f64().unwrap());
                let r = a.plan(pt(&c["start"]), pt(&c["goal"]), &segs(&c["walls"]));
                json!(r.iter().map(|p| json!([p.0, p.1])).collect::<Vec<_>>())
            }
            "arc" => {
                let pts: Vec<(f64, f64)> = c["pts"].as_array().unwrap().iter().map(pt).collect();
                let b = c["box"].as_array().unwrap(); let b = [b[0].as_f64().unwrap(), b[1].as_f64().unwrap(), b[2].as_f64().unwrap(), b[3].as_f64().unwrap()];
                let phi = c["phi"].as_f64().unwrap();
                json!(pts.iter().map(|&(x, y)| sweep::arc_hits_one(x, y, phi, &b, 0.0, 0.0)).collect::<Vec<_>>())
            }
            "eff" => {
                let p = protection::effective(&c["spec"], c.get("ch").filter(|x| !x.is_null()));
                json!({"json": p.json})
            }
            _ => json!(null),
        };
        println!("{}", out);
    }
}
