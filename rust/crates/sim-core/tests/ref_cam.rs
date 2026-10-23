use serde_json::Value;
use sim_core::cameras::*;
use sim_core::world::*;

fn load() -> Option<(Value, Vec<f32>)> {
    let s = std::fs::read_to_string("/tmp/ref_cam.json").ok()?;
    let t = std::fs::read("/tmp/ref_tex.bin").ok()?;
    let tex: Vec<f32> = t.chunks_exact(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])).collect();
    Some((serde_json::from_str(&s).ok()?, tex))
}

#[test]
fn cameras_match_python() {
    let Some((j, tex)) = load() else { eprintln!("no ref"); return; };
    let prims: Vec<Prim> = j["prims"].as_array().unwrap().iter().map(|r| { let mut p = [0.0; 18]; for (i, v) in r.as_array().unwrap().iter().enumerate() { p[i] = v.as_f64().unwrap(); } p }).collect();
    let rgb: Vec<[f32; 3]> = j["rgb"].as_array().unwrap().iter().map(|r| { let a = r.as_array().unwrap(); [a[0].as_f64().unwrap() as f32, a[1].as_f64().unwrap() as f32, a[2].as_f64().unwrap() as f32] }).collect();
    let ts = j["tex_shape"].as_array().unwrap();
    let org = j["org"].as_array().unwrap();
    let scene = CamScene { revision: 0, prims, rgb, floor_gid: j["floor"].as_i64().unwrap() as i32, ceil_gid: j["ceil"].as_i64().unwrap() as i32, tex: std::sync::Arc::new(tex),
        tex_h: ts[0].as_u64().unwrap() as usize, tex_w: ts[1].as_u64().unwrap() as usize,
        tex_org: (org[0].as_f64().unwrap(), org[1].as_f64().unwrap(), org[2].as_f64().unwrap()) };
    let pose = j["pose"].as_array().unwrap();
    let (x, y, th) = (pose[0].as_f64().unwrap(), pose[1].as_f64().unwrap(), pose[2].as_f64().unwrap());
    for cj in j["cams"].as_array().unwrap() {
        let mut cam = build_camera(&cj["cfg"]).expect("cam");
        let fr = cam.capture(&scene, x, y, th, false, 4);
        for (name, plane) in &fr.streams {
            let r = &cj["streams"][name.as_str()];
            let data = r[1].as_array().unwrap();
            match plane {
                Plane::Rgb(v) => {
                    assert_eq!(v.len(), data.len());
                    let bad = v.iter().zip(data).filter(|(a, b)| (**a as i64 - b.as_i64().unwrap()).abs() > 1).count();
                    let exact = v.iter().zip(data).filter(|(a, b)| **a as i64 == b.as_i64().unwrap()).count();
                    eprintln!("{} {} rgb bad>1: {} / {} (exact {})", cj["name"], name, bad, v.len(), exact);
                    assert!(bad * 1000 <= v.len(), "{} {}", cj["name"], name);
                }
                Plane::Depth(v) => {
                    assert_eq!(v.len(), data.len());
                    let mut bad = 0; let mut mx = 0f64;
                    for (a, b) in v.iter().zip(data) {
                        match (a.is_nan(), b.as_f64()) { (true, None) => {}, (false, Some(bv)) => { let d = (*a as f64 - bv).abs(); mx = mx.max(d); if d > 1e-4 { bad += 1; } }, _ => bad += 1 }
                    }
                    eprintln!("{} {} depth bad {} maxdiff {:e}", cj["name"], name, bad, mx);
                    assert!(bad * 1000 <= v.len(), "{} {}", cj["name"], name);
                }
                Plane::Amp(v) => {
                    assert_eq!(v.len(), data.len());
                    let bad = v.iter().zip(data).filter(|(a, b)| (**a as i64 - b.as_i64().unwrap()).abs() > 1).count();
                    eprintln!("{} amp bad {}", cj["name"], bad);
                    assert!(bad * 500 <= v.len());
                }
                Plane::Points(v) => { eprintln!("{} points {} vs {}", cj["name"], v.len(), data.len()); assert!((v.len() as i64 - data.len() as i64).abs() <= 3 * (data.len() as i64 / 500 + 1)); }
            }
        }
    }
}

#[test]
fn simcore_scene_matches_python() {
    let Some((j, _)) = load() else { return; };
    let lib_path = std::env::var("MUJOCO_LIB").unwrap_or("/tmp/claude-0/work/mjx86/mujoco/libmujoco.so.3.16.0".into());
    let lib = std::sync::Arc::new(mujoco_sys::load(Some(&lib_path)).expect("lib"));
    let scen: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/scenarios.json")).unwrap()).unwrap();
    let spec: Value = serde_json::from_str(&std::fs::read_to_string("/home/claude/agv_sim/robot_config.json").unwrap()).unwrap();
    std::env::set_var("SIM_NOISE", "0");
    let mut c = sim_core::core::SimCore::new(spec, scen.as_object().unwrap(), "grid_9_square", None, Some(lib), sim_core::core::Settings::default(), 1).unwrap();
    for _ in 0..300 { c.set_cmd(0.8, 0.0, 0.3); c.step(); }
    let pose = j["pose"].as_array().unwrap();
    eprintln!("pose rust {:.6} {:.6} {:.6} py {:.6} {:.6} {:.6}", c.x, c.y, c.th, pose[0], pose[1], pose[2]);
    for (i, cj) in j["cams"].as_array().unwrap().iter().enumerate() {
        let fr = c.capture_camera(i, false).unwrap();
        for (name, plane) in &fr.streams {
            let data = cj["streams"][name.as_str()][1].as_array().unwrap();
            if let Plane::Rgb(v) = plane {
                let bad = v.iter().zip(data).filter(|(a, b)| (**a as i64 - b.as_i64().unwrap()).abs() > 2).count();
                eprintln!("{} {} rgb bad {} / {}", cj["name"], name, bad, v.len());
                assert!(bad * 200 <= v.len());
            }
            if let Plane::Depth(v) = plane {
                let bad = v.iter().zip(data).filter(|(a, b)| match (a.is_nan(), b.as_f64()) { (true, None) => false, (false, Some(bv)) => (**a as f64 - bv).abs() > 1e-3, _ => true }).count();
                eprintln!("{} {} depth bad {} / {}", cj["name"], name, bad, v.len());
                assert!(bad * 200 <= v.len());
            }
        }
    }
}
