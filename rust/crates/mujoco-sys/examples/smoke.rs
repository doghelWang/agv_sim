use mujoco_sys::*;
use std::ffi::CString;
fn main() {
    let lib = load(None).expect("load mujoco");
    let xml = r#"<mujoco><option timestep="0.01" gravity="0 0 0" integrator="implicitfast"/>
<worldbody><geom name="w" type="box" pos="3 0 1" size="0.025 2 1" group="0"/>
<body name="agv"><joint name="jx" type="slide" axis="1 0 0"/><geom type="box" size="0.3 0.3 0.1" mass="100" group="1"/></body></worldbody>
<actuator><velocity name="vx" joint="jx" kv="4000"/></actuator></mujoco>"#;
    let path = std::env::temp_dir().join("smoke.xml");
    std::fs::write(&path, xml).unwrap();
    unsafe {
        let mut err = vec![0u8; 1000];
        let c = CString::new(path.to_str().unwrap()).unwrap();
        let m = lib.mj_loadXML(c.as_ptr() as *const _, std::ptr::null(), err.as_mut_ptr() as *mut _, 1000);
        assert!(!m.is_null(), "load failed");
        let d = lib.mj_makeData(m);
        let ctrl = std::slice::from_raw_parts_mut((*d).ctrl, 1);
        ctrl[0] = 1.0;
        for _ in 0..100 { lib.mj_step(m, d); }
        let q = std::slice::from_raw_parts((*d).qpos, 1);
        println!("mujoco {} qpos after 1s = {:.4}", std::ffi::CStr::from_ptr(lib.mj_versionString()).to_str().unwrap(), q[0]);
        // ray
        let pnt = [0.0f64, 0.0, 1.0];
        let vec = [1.0f64, 0.0, 0.0];
        let mut gid = -1i32;
        let gg = [1u8, 0, 0, 0, 0, 0];
        let dist = lib.mj_ray(m, d, pnt.as_ptr(), vec.as_ptr(), gg.as_ptr(), true as _, -1, &mut gid, std::ptr::null_mut());
        println!("ray dist {:.4} geom {}", dist, gid);
    }
}
