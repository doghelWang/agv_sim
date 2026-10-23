//! Parity against the original C kernel (sim_core/native/simcore.c compiled for the host).
use libloading::{Library, Symbol};
use sim_core::{geom, kin::*, rng::Rng};
use std::os::raw::c_int;
use std::path::PathBuf;
use std::process::Command;

#[repr(C)] #[derive(Clone, Copy, Default)]
struct CWheel { kind: c_int, pad: c_int, x: f64, y: f64, radius: f64, steer_min: f64, steer_max: f64, steer_rate: f64, max_speed: f64, drive_tau: f64, gear_ratio: f64,
    steer: f64, steer_target: f64, speed: f64, speed_target: f64, cmd_speed: f64, angle: f64, motor_rpm: f64, current_a: f64, torque_nm: f64, steer_current_a: f64 }
#[repr(C)] #[derive(Clone, Copy, Default)]
struct CChassis { max_speed: f64, max_accel: f64, max_decel: f64, max_ang_speed: f64, max_ang_accel: f64, max_ang_decel: f64, mass: f64,
    axle_x: f64, kt: f64, rolling_coeff: f64, efficiency: f64, holonomic: c_int, pad: c_int, shaped: [f64; 3], vx: f64, vy: f64, wz: f64, slip_residual: f64, saturation: f64 }

fn lib() -> Library {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../..");
    let src = root.join("sim_core/native/simcore.c");
    let out = std::env::temp_dir().join("libsimcore_host.so");
    let st = Command::new("gcc").args(["-O2", "-shared", "-fPIC", "-o"]).arg(&out).arg(&src).args(["-lm", "-lpthread"]).status().expect("gcc");
    assert!(st.success());
    unsafe { Library::new(out).unwrap() }
}

fn to_c(w: &Wheel) -> CWheel {
    CWheel { kind: match w.kind { Kind::Drive => 0, Kind::Steer => 1, Kind::Fixed => 2, Kind::Caster => 3 }, pad: 0, x: w.x, y: w.y, radius: w.radius,
        steer_min: w.steer_min, steer_max: w.steer_max, steer_rate: w.steer_rate, max_speed: w.max_speed, drive_tau: w.drive_tau, gear_ratio: w.gear_ratio,
        steer: w.steer, steer_target: w.steer_target, speed: w.speed, speed_target: w.speed_target, cmd_speed: w.cmd_speed, angle: w.angle,
        motor_rpm: 0.0, current_a: 0.0, torque_nm: 0.0, steer_current_a: 0.0 }
}

fn chassis(kind: &str) -> Chassis {
    let spec = serde_json::json!({"chassis":{"type":kind,"head_offset_m":0.9,"tail_offset_m":0.6,"left_offset_m":0.45,"right_offset_m":0.45,"max_speed_mps":1.5,"mass_kg":300.0},
        "drive_wheels":{"radius_m":0.1}});
    build_kinematics(&spec, None).unwrap().0
}

#[test]
fn kin_step_matches_c() {
    let l = lib();
    let step: Symbol<unsafe extern "C" fn(*mut CChassis, *mut CWheel, c_int, f64, f64, f64, f64, c_int, *mut f64)> = unsafe { l.get(b"sc_kin_step").unwrap() };
    let mut rng = Rng::new(7);
    for kind in ["diff_drive", "single_steer", "dual_steer"] {
        let mut ch = chassis(kind);
        let mut cc = CChassis { max_speed: ch.lim.max_speed, max_accel: ch.lim.max_accel, max_decel: ch.lim.max_decel, max_ang_speed: ch.lim.max_ang_speed,
            max_ang_accel: ch.lim.max_ang_accel, max_ang_decel: ch.lim.max_ang_decel, mass: ch.lim.mass, axle_x: ch.axle_x, kt: ch.kt, rolling_coeff: ch.rolling_coeff,
            efficiency: ch.efficiency, holonomic: ch.holonomic as c_int, ..Default::default() };
        let mut cw: Vec<CWheel> = ch.wheels.iter().map(to_c).collect();
        let mut maxd = 0.0f64;
        for i in 0..3000 {
            let (cvx, cvy, cwz) = if i % 400 < 300 { ((rng.uniform() - 0.3) * 2.0, (rng.uniform() - 0.5) * 1.0, (rng.uniform() - 0.5) * 2.0) } else { (0.0, 0.0, 0.0) };
            let brake = i % 997 == 996;
            let mut out = [0.0f64; 3];
            unsafe { step(&mut cc, cw.as_mut_ptr(), cw.len() as c_int, cvx, cvy, cwz, 0.01, brake as c_int, out.as_mut_ptr()) };
            let r = ch.step(cvx, cvy, cwz, 0.01, brake);
            for (a, b) in [(out[0], r.0), (out[1], r.1), (out[2], r.2), (cc.slip_residual, ch.slip_residual), (cc.saturation, ch.saturation)] {
                maxd = maxd.max((a - b).abs());
            }
            for (c, w) in cw.iter().zip(&ch.wheels) {
                for (a, b) in [(c.steer, w.steer), (c.speed, w.speed), (c.angle, w.angle), (c.current_a, w.current_a), (c.torque_nm, w.torque_nm), (c.motor_rpm, w.motor_rpm)] {
                    maxd = maxd.max((a - b).abs());
                }
            }
        }
        println!("{kind}: max abs diff {maxd:e}");
        assert!(maxd < 1e-9, "{kind} diverged: {maxd}");
    }
}

#[test]
fn rng_and_geom_match_c() {
    let l = lib();
    let seed: Symbol<unsafe extern "C" fn(*mut u64, u64)> = unsafe { l.get(b"sc_rng_seed").unwrap() };
    let gauss: Symbol<unsafe extern "C" fn(*mut u64) -> f64> = unsafe { l.get(b"sc_rng_gauss").unwrap() };
    let mut s = [0u64; 6];
    unsafe { seed(s.as_mut_ptr(), 12345) };
    let mut r = Rng::new(12345);
    for _ in 0..1000 { let a = unsafe { gauss(s.as_mut_ptr()) }; let b = r.gauss(); assert_eq!(a.to_bits(), b.to_bits()); }

    let ray: Symbol<unsafe extern "C" fn(*const f64, c_int, *const f64, c_int, f64, f64, f64, *const f64, c_int, f64, *mut f64)> = unsafe { l.get(b"sc_raycast2d").unwrap() };
    let coll: Symbol<unsafe extern "C" fn(*const f64, c_int, f64, f64, f64, *const f64, c_int, *mut f64) -> c_int> = unsafe { l.get(b"sc_collides").unwrap() };
    let mut rr = Rng::new(99);
    let segs: Vec<geom::Seg> = (0..40).map(|_| { let x = rr.uniform() * 20.0 - 10.0; let y = rr.uniform() * 20.0 - 10.0; [x, y, x + rr.uniform() * 4.0 - 2.0, y + rr.uniform() * 4.0 - 2.0, 0.5 + rr.uniform() * 5.0] }).collect();
    let circ: Vec<geom::Circle> = vec![[1.0, 1.0, 0.3, 1.8], [-3.0, 2.0, 0.4, 1.8]];
    let flat: Vec<f64> = segs.iter().flatten().copied().collect();
    let cflat: Vec<f64> = circ.iter().flatten().copied().collect();
    let angs: Vec<f64> = (0..720).map(|i| -3.14159 + i as f64 * 0.00873).collect();
    for k in 0..30 {
        let (ox, oy, z) = (rr.uniform() * 6.0 - 3.0, rr.uniform() * 6.0 - 3.0, rr.uniform());
        let mut oc = vec![0.0; 720];
        let mut orr = vec![0.0; 720];
        unsafe { ray(flat.as_ptr(), 40, cflat.as_ptr(), 2, z, ox, oy, angs.as_ptr(), 720, 12.0, oc.as_mut_ptr()) };
        geom::raycast2d(&segs, &circ, z, ox, oy, &angs, 12.0, &mut orr);
        for i in 0..720 { assert!(oc[i] == orr[i] || (oc[i] - orr[i]).abs() < 1e-12, "ray {k}/{i}: {} vs {}", oc[i], orr[i]); }
    }
    let fp = [[0.8, 0.5], [0.8, -0.5], [-0.8, -0.5], [-0.8, 0.5]];
    let fpf: Vec<f64> = fp.iter().flatten().copied().collect();
    let (mut nh, mut tot) = (0, 0);
    for _ in 0..3000 {
        let (x, y, th) = (rr.uniform() * 14.0 - 7.0, rr.uniform() * 14.0 - 7.0, rr.uniform() * 6.28 - 3.14);
        let mut pt = [0.0; 2];
        let c = unsafe { coll(fpf.as_ptr(), 4, x, y, th, flat.as_ptr(), 40, pt.as_mut_ptr()) } != 0;
        let r = geom::collides(&fp, x, y, th, &segs);
        assert_eq!(c, r.is_some());
        if let Some(p) = r { assert!((p[0] - pt[0]).abs() < 1e-12 && (p[1] - pt[1]).abs() < 1e-12); nh += 1; }
        tot += 1;
    }
    println!("collisions hit {nh}/{tot}");
    assert!(nh > 50);
}
