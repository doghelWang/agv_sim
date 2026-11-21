//! Actuator behaviour on cmodel-derived robots: jacking (rack), roller (tote ↔ conveyor), tugger (cart), reach fork, casters.
//! Runs in kinematic mode; with MUJOCO_LIB set the same scenarios also run on MuJoCo (articulated MJCF, part geoms).
use serde_json::{json, Map, Value};
use sim_core::core::*;
use std::sync::Arc;

fn root() -> String { concat!(env!("CARGO_MANIFEST_DIR"), "/../../..").to_string() }
fn spec_of(file: &str) -> Value {
    let p = format!("{}/{file}", root());
    let d = cmodel::decode_cmodel_bytes(&std::fs::read(&p).unwrap()).unwrap();
    cmodel::spec::extract_robot_spec(&d["CompDesc"], file.rsplit('/').next().unwrap(), "full", None)
}
fn scen() -> Map<String, Value> { serde_json::from_str::<Value>(&std::fs::read_to_string(format!("{}/rust/data/scenarios.json", root())).unwrap()).unwrap().as_object().unwrap().clone() }
fn lib() -> Option<Arc<mujoco_sys::MuJoCo>> { std::env::var("MUJOCO_LIB").ok().and_then(|p| mujoco_sys::load(Some(&p)).ok()).map(Arc::new) }
fn core(spec: Value, mj: bool) -> SimCore {
    let mut st = Settings::default();
    st.noise = false;
    let mut c = SimCore::new(spec, &scen(), "standard_cross", None, if mj { lib() } else { None }, st, 1).unwrap();
    c.set_dressing("none").unwrap();
    c.reset_pose(0.0, 0.0, 0.0);
    c
}
fn run(c: &mut SimCore, vx: f64, wz: f64, secs: f64) { let n = (secs / c.settings.dt) as u32; for _ in 0..n { c.set_cmd(vx, 0.0, wz); c.step_n(1); } }
fn idle(c: &mut SimCore, secs: f64) { run(c, 0.0, 0.0, secs) }
fn cargo(c: &SimCore, id: &str) -> Map<String, Value> { c.world.obstacles.iter().find(|o| o.get("id").and_then(|v| v.as_str()) == Some(id)).unwrap().clone() }
fn f(o: &Map<String, Value>, k: &str) -> f64 { o[k].as_f64().unwrap() }
fn events(c: &mut SimCore) -> Vec<&'static str> { c.fork_events.drain(..).map(|e| e.kind).collect() }
fn joint(c: &SimCore, n: &str) -> f64 { let (names, pos, _, _) = c.joint_state(); pos[names.iter().position(|x| x == n).unwrap_or_else(|| panic!("no joint {n} in {names:?}"))] }

fn jacking(mj: bool) {
    let mut c = core(spec_of("models/latent_jacking_amr.cmodel"), mj);
    assert_eq!(c.asm["archetype"], "jacking");
    c.add_cargo("rack", Some("R1"), 0.02, 0.0, 0.3, None, None).unwrap();
    assert!(f(&cargo(&c, "R1"), "w") > 1.6 && f(&cargo(&c, "R1"), "h") > 1.25);
    assert!((f(&cargo(&c, "R1"), "leg_h") - 0.198).abs() < 1e-6);
    // robot is under the rack: legs only → no collision / bumper
    idle(&mut c, 0.2);
    assert_eq!(c.collisions, 0, "robot under rack must not collide with the rack outline (contact {:?}, rack {:?})", c.last_contact, cargo(&c, "R1"));
    c.actuator_command(&json!({"action": "up"})).unwrap();
    idle(&mut c, 3.5);
    let ev = events(&mut c);
    assert!(ev.contains(&"JACK_PICK"), "{ev:?}");
    let r = cargo(&c, "R1");
    assert!(f(&r, "z0") > 0.03, "rack lifted: {r:?}");
    assert!((joint(&c, "lift_joint") - 0.06).abs() < 1e-3);
    // rotate in place: synchronous tray keeps the rack heading
    run(&mut c, 0.0, 0.6, 2.0);
    idle(&mut c, 1.0);
    assert!(c.th.abs() > 0.5, "chassis rotated {}", c.th);
    assert!((f(&cargo(&c, "R1"), "yaw") - 0.3).abs() < 0.05, "rack heading kept {:?}", cargo(&c, "R1")["yaw"]);
    assert!(joint(&c, "tray_rotate_joint").abs() > 0.4);
    // drive and set down elsewhere
    let x0 = c.x;
    run(&mut c, 0.4, 0.0, 2.5);
    idle(&mut c, 1.0);
    assert!((c.x - x0).abs() + (c.y).abs() > 0.5);
    let r = cargo(&c, "R1");
    assert!((f(&r, "x") - c.x).abs() < 0.1 && (f(&r, "y") - c.y).abs() < 0.1, "rack follows the robot");
    c.actuator_command(&json!({"action": "down"})).unwrap();
    idle(&mut c, 3.5);
    assert!(events(&mut c).contains(&"JACK_PLACE"));
    assert!(f(&cargo(&c, "R1"), "z0").abs() < 1e-6);
    assert!(c.world.obstacles.iter().all(|o| o.get("carried").is_none()));
}
#[test] fn jacking_kinematic() { jacking(false) }
#[test] fn jacking_mujoco() { if lib().is_some() { jacking(true) } }

fn roller(mj: bool) {
    let mut c = core(spec_of("models/roller_amr.cmodel"), mj);
    assert_eq!(c.asm["archetype"], "roller");
    let deck = c.asm["actuator"]["deck_z_m"].as_f64().unwrap();
    // conveyor stations left and right of the robot, top at deck height
    let mut conv = |id: &str, y: f64| { let mut m = Map::new(); for (k, v) in [("id", json!(id)), ("type", json!("conveyor")), ("x", json!(0.0)), ("y", json!(y)), ("w", json!(1.0)), ("h", json!(0.9)), ("z", json!(deck)), ("roll_axis", json!("x"))] { m.insert(k.into(), v); } m };
    let (a, b) = (conv("CV_L", 0.86), conv("CV_R", -0.86));
    c.set_obstacles(vec![a, b]).unwrap();
    c.add_cargo("tote", Some("T1"), 0.0, 0.95, 0.0, None, None).unwrap();
    assert!((f(&cargo(&c, "T1"), "z0") - deck).abs() < 1e-6, "tote rests on the conveyor: {:?} deck {deck} obstacles {:?}", cargo(&c, "T1"), c.world.obstacles.iter().filter(|o| o["type"] == "conveyor").collect::<Vec<_>>());
    c.actuator_command(&json!({"action": "load", "side": "left"})).unwrap();
    idle(&mut c, 5.0);
    let ev = events(&mut c);
    assert!(ev.contains(&"ROLLER_LOADED"), "{ev:?}");
    let t = cargo(&c, "T1");
    assert!(f(&t, "y").abs() < 0.02, "tote centred on deck {t:?}");
    assert!(joint(&c, "roller_0_joint").abs() > 1.0, "rollers turned");
    c.actuator_command(&json!({"action": "unload", "side": "right"})).unwrap();
    idle(&mut c, 5.0);
    let ev = events(&mut c);
    assert!(ev.contains(&"ROLLER_UNLOADED"), "{ev:?}");
    let t = cargo(&c, "T1");
    assert!((f(&t, "y") + 0.86).abs() < 0.03 && (f(&t, "z0") - deck).abs() < 1e-6, "tote carried to the centre of the right station {t:?}");
    // load it back and unload where no station is → drop
    c.actuator_command(&json!({"action": "load", "side": "right"})).unwrap();
    idle(&mut c, 5.0);
    c.set_obstacles(vec![]).unwrap();
    c.actuator_command(&json!({"action": "unload", "side": "left"})).unwrap();
    idle(&mut c, 5.0);
    assert!(events(&mut c).contains(&"ROLLER_DROP"));
    assert!(f(&cargo(&c, "T1"), "z0") < 0.05);
}
#[test] fn roller_kinematic() { roller(false) }
#[test] fn roller_mujoco() { if lib().is_some() { roller(true) } }

fn tugger(mj: bool) {
    let mut c = core(spec_of("models/tugger_amr.cmodel"), mj);
    assert_eq!(c.asm["archetype"], "tugger");
    let hx = c.asm["actuator"]["hitch_x_m"].as_f64().unwrap();
    // cart behind the robot, drawbar ring 3 cm from the hitch pin
    let l = 0.6 + 0.35;
    c.add_cargo("cart", Some("C1"), hx - l + 0.03, 0.01, 0.0, None, None).unwrap();
    c.actuator_command(&json!({"action": "lock"})).unwrap();
    idle(&mut c, 1.5);
    let ev = events(&mut c);
    assert!(ev.contains(&"TOW_HITCHED"), "{ev:?}");
    run(&mut c, 0.5, 0.25, 4.0);
    idle(&mut c, 0.5);
    let o = cargo(&c, "C1");
    let (s, co) = c.th.sin_cos();
    let hw = (c.x + co * hx, c.y + s * hx);
    let ring = (f(&o, "x") + f(&o, "yaw").cos() * l, f(&o, "y") + f(&o, "yaw").sin() * l);
    assert!((ring.0 - hw.0).hypot(ring.1 - hw.1) < 0.02, "ring stays on the hitch");
    assert!(f(&o, "x") > -0.5, "cart was pulled {o:?}");
    let art = c.actuator_view()["articulation_deg"].as_f64().unwrap();
    assert!(art.abs() > 1.0 && art.abs() < 75.0, "articulation {art}");
    c.actuator_command(&json!({"action": "release"})).unwrap();
    idle(&mut c, 1.5);
    assert!(events(&mut c).contains(&"TOW_RELEASED"));
    let before = cargo(&c, "C1");
    run(&mut c, 0.4, 0.0, 1.0);
    assert_eq!(f(&before, "x"), f(&cargo(&c, "C1"), "x"), "released cart stays");
}
#[test] fn tugger_kinematic() { tugger(false) }
#[test] fn tugger_mujoco() { if lib().is_some() { tugger(true) } }

#[test]
fn reach_fork_moves_mast() {
    let mut s = spec_of("models/forklift_4steer.cmodel");
    s["actuator"] = json!({"type": "reach_fork", "reach_stroke_m": 0.5});
    let mut c = core(s, false);
    assert_eq!(c.asm["archetype"], "reach_fork");
    let x0 = c.fork.as_ref().unwrap().sp.x;
    c.actuator_command(&json!({"action": "reach_out"})).unwrap();
    idle(&mut c, 6.0);
    assert!((joint(&c, "reach_joint") - 0.5).abs() < 2e-3, "reach {}", joint(&c, "reach_joint"));
    assert!((c.fork.as_ref().unwrap().sp.x - x0 - 0.5).abs() < 2e-3);
    c.actuator_command(&json!({"height": 0.5})).unwrap();
    idle(&mut c, 5.0);
    assert!((joint(&c, "fork_lift_joint") - 0.5).abs() < 2e-3);
    assert!((joint(&c, "mast_stage_joint") - 0.25).abs() < 2e-3);
}

#[test]
fn casters_swivel_when_rotating() {
    let mut c = core(spec_of("models/latent_jacking_amr.cmodel"), false);
    run(&mut c, 0.3, 0.0, 1.0);
    let a0 = joint(&c, "caster_front_left_swivel_joint");
    assert!(a0.abs() < 0.05, "straight → swivel aligned {a0}");
    run(&mut c, 0.0, 0.8, 2.0);
    let a1 = joint(&c, "caster_front_left_swivel_joint");
    assert!(a1.abs() > 1.0, "spin in place → caster turns tangential, got {a1}");
    assert!(joint(&c, "caster_front_left_wheel_joint").abs() > 0.5);
}

#[test]
fn mjcf_compiles_for_every_family() {
    let Some(_) = lib() else { return };
    for (f, arch) in [("models/latent_jacking_amr.cmodel", "jacking"), ("models/roller_amr.cmodel", "roller"), ("models/tugger_amr.cmodel", "tugger"),
        ("models/pallet_truck_amr.cmodel", "pallet_truck"), ("models/forklift_4steer.cmodel", "stacker_fork"), ("tests/data/测试车模型.cmodel", "")] {
        let mut c = core(spec_of(f), true);
        if !arch.is_empty() { assert_eq!(c.asm["archetype"], arch, "{f}"); }
        assert!(!c.mj.as_ref().unwrap().joint_adr.is_empty(), "{f}: articulated joints in MJCF");
        run(&mut c, 0.2, 0.1, 0.5);
        c.scan_now();
    }
}

#[test]
fn lidar_sees_rack_legs_not_outline() {
    let Some(_) = lib() else { return };
    let mut c = core(spec_of("models/latent_jacking_amr.cmodel"), true);
    c.add_cargo("rack", Some("R1"), 3.0, 0.0, 0.0, None, None).unwrap();
    let r1 = cargo(&c, "R1");
    let (lx, ly) = (3.0 - f(&r1, "w") / 2.0 + 0.025, f(&r1, "h") / 2.0 - 0.025);
    c.scan_now();
    let l = &c.lidars[0];
    let r = &c.lid_out[0].ranges;
    // straight ahead (between the legs) the beam passes under the rack
    let k0 = ((0.0 - l.angle_min) / l.angle_inc).round() as usize;
    assert!(r[k0] > 3.0 + f(&r1, "w") / 2.0, "beam passes between the legs: {}", r[k0]);
    // towards the near-left leg (lidar at x=0.7)
    let a = ly.atan2(lx - 0.7);
    let k = ((a - l.angle_min) / l.angle_inc).round() as usize;
    assert!((r[k] - ly.hypot(lx - 0.7)).abs() < 0.1, "leg hit {} vs {}", r[k], ly.hypot(lx - 0.7));
}

#[test]
fn code_reader_images_qr() {
    let mut c = core(spec_of("models/latent_jacking_amr.cmodel"), false);
    let idx = c.cameras.iter().position(|k| k.lock().unwrap().kind_name() == "codeReader").expect("code reader camera");
    // origin node N_ORIGIN has a QR decal at (0,0); smart camera is at the motion center
    let fr = c.capture_camera(idx, false).unwrap();
    let sim_core::cameras::Plane::Rgb(img) = &fr.streams[0].1 else { panic!() };
    let (dark, light) = img.chunks(3).fold((0, 0), |(d, l), p| if p[0] < 60 { (d + 1, l) } else if p[0] > 170 { (d, l + 1) } else { (d, l) });
    assert!(dark > 2000 && light > 2000, "QR modules visible: dark {dark} light {light}");
    let codes = c.detect_codes();
    assert!(codes.iter().any(|k| k["payload"] == "QR_N_ORIGIN" && k["symbology"] == "QR"), "{codes:?}");
}

#[test]
fn dressing_same_in_both_backends() {
    let mut a = core(spec_of("models/roller_amr.cmodel"), false);
    a.set_dressing("auto").unwrap();
    assert!(a.props_list().len() > 5 && a.cargo_list().len() > 3, "auto dressing: {} props {} cargo", a.props_list().len(), a.cargo_list().len());
    // robot at the origin is clear of every prop
    idle(&mut a, 0.3);
    assert_eq!(a.collisions, 0);
    let n = |c: &SimCore| c.world.obstacles.iter().map(|o| format!("{}@{:.2},{:.2},{:.2}", o["id"], o["x"].as_f64().unwrap_or(0.0), o["y"].as_f64().unwrap_or(0.0), o.get("z0").and_then(|v| v.as_f64()).unwrap_or(0.0))).collect::<Vec<_>>();
    if lib().is_none() { return; }
    let mut b = core(spec_of("models/roller_amr.cmodel"), true);
    b.set_dressing("auto").unwrap();
    assert_eq!(n(&a), n(&b));
    idle(&mut b, 0.3);
    assert_eq!(b.collisions, 0);
}
