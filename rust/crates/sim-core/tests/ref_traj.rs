//! Compare against the Python/C implementation (noise off) — reference produced by /tmp/ref_run.py
use serde_json::Value;
use sim_core::core::*;

#[test]
fn trajectory_matches_python() {
    let Ok(txt) = std::fs::read_to_string("/tmp/ref_out.json") else { eprintln!("no reference, skipping"); return; };
    let rf: Value = serde_json::from_str(&txt).unwrap();
    let lib = std::env::var("MUJOCO_LIB").unwrap_or("/tmp/claude-0/work/mjx86/mujoco/libmujoco.so.3.16.0".into());
    let lib = std::sync::Arc::new(mujoco_sys::load(Some(&lib)).expect("libmujoco"));
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../..");
    let spec: Value = serde_json::from_str(&std::fs::read_to_string(format!("{root}/robot_config.json")).unwrap()).unwrap();
    let scen: Value = serde_json::from_str(&std::fs::read_to_string(format!("{root}/rust/data/scenarios.json")).unwrap()).unwrap();
    let mut st = Settings::default();
    st.noise = false;
    let mut c = SimCore::new(spec, scen.as_object().unwrap(), "grid_9_square", None, Some(lib), st, 1).unwrap();
    assert_eq!(c.chassis_type, "single_steer");
    let script: Vec<(f64, f64, f64)> = [(0.8, 0.0, 0.0, 200), (0.5, 0.0, 0.6, 300), (0.0, 0.0, 0.0, 100), (1.0, 0.0, 0.0, 400)].iter().flat_map(|&(a, b, cc, n)| std::iter::repeat((a, b, cc)).take(n)).collect();
    let traj = rf["traj"].as_array().unwrap();
    let mut maxd = 0.0f64;
    for (i, cmd) in script.iter().enumerate() {
        c.set_cmd(cmd.0, cmd.1, cmd.2);
        c.step();
        if i % 50 == 49 {
            let r = traj[i / 50].as_array().unwrap();
            let mine = [c.t, c.x, c.y, c.th, c.vx, c.vy, c.wz, c.odom.x, c.odom.y, c.odom.th];
            for k in 0..10 { maxd = maxd.max((r[k].as_f64().unwrap() - mine[k]).abs()); }
        }
    }
    println!("trajectory max abs diff {maxd:e}; pose {:.4} {:.4} {:.4}", c.x, c.y, c.th);
    assert!(maxd < 1e-6, "trajectory differs: {maxd}");
    c.scan_now();
    let m = rf["merged"].as_array().unwrap();
    assert_eq!(m.len(), c.merged.ranges.len());
    let mut worst = 0.0f64;
    let mut miss = 0;
    for (a, b) in m.iter().zip(c.merged.ranges.iter()) {
        match (a.as_f64(), b.is_finite()) {
            (Some(x), true) => worst = worst.max((x - b).abs()),
            (None, false) => {}
            _ => miss += 1,
        }
    }
    println!("merged scan: worst diff {worst:e}, finite mismatch {miss}/{}", m.len());
    assert!(worst < 1e-6 && miss == 0);
    assert_eq!(rf["collisions"].as_u64().unwrap() as u32, c.collisions);
}
