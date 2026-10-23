//! MuJoCo model construction (MJCF) and a safe-ish context wrapper.
use crate::world::*;
use mujoco_sys as ms;
use serde_json::Value;
use std::ffi::CString;
use std::sync::Arc;

fn q(v: f64) -> String { format!("{:.5}", v) }

#[derive(Clone, Debug)]
pub struct RobotBody { pub footprint: Vec<[f64; 2]>, pub height: f64, pub mass: f64, pub clearance: f64 }

impl RobotBody {
    pub fn from_spec(spec: &Value) -> RobotBody {
        let ch = &spec["chassis"];
        let footprint: Vec<[f64; 2]> = ch["footprint"].as_array().map(|a| a.iter().map(|p| [p[0].as_f64().unwrap_or(0.0), p[1].as_f64().unwrap_or(0.0)]).collect()).unwrap_or_default();
        let r_min = spec.get("wheels").and_then(|w| w.as_array()).map(|ws| ws.iter().map(|w| w.get("radius_m").and_then(|r| r.as_f64()).unwrap_or(0.1)).fold(f64::INFINITY, f64::min)).filter(|v| v.is_finite()).unwrap_or(0.1);
        let mass = ch.get("mass_kg").and_then(|m| m.as_f64()).filter(|m| *m != 0.0).unwrap_or(200.0);
        RobotBody { footprint, height: ch.get("height_m").and_then(|h| h.as_f64()).unwrap_or(1.0), mass, clearance: (r_min * 0.5).max(0.02) }
    }
}

pub struct Decor<'a> {
    pub stations: &'a [[f64; 2]],
    pub tags: &'a [[f64; 2]],
    pub lanes: &'a [[f64; 4]],
}

/// Build the MJCF. Returns (xml, geom names for static decorations -> category not needed here).
pub fn build_mjcf(world: &World, robot: &RobotBody, dt: f64, decor: &Decor) -> String {
    let b = world.bounds;
    let (cx, cy) = ((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0);
    let sx = 5.0f64.max((b[2] - b[0]) / 2.0 + 2.0);
    let sy = 5.0f64.max((b[3] - b[1]) / 2.0 + 2.0);
    let mut g: Vec<String> = vec![];
    for (i, s) in world.static_segments.iter().enumerate() {
        let (x0, y0, x1, y1, h) = (s[0], s[1], s[2], s[3], s[4]);
        let l = (x1 - x0).hypot(y1 - y0);
        if l < 0.01 { continue; }
        let cat = if h >= CEILING_HEIGHT - 1e-3 { "wall" } else { "shelf" };
        let rgb = CAT_COLORS[cat_id(cat)];
        g.push(format!("<geom name=\"s{i}\" type=\"box\" pos=\"{} {} {}\" size=\"{} 0.025 {}\" euler=\"0 0 {}\" rgba=\"{} {} {} 1\" group=\"0\"/>",
            q((x0 + x1) / 2.0), q((y0 + y1) / 2.0), q(h / 2.0), q(l / 2.0), q(h / 2.0), q((y1 - y0).atan2(x1 - x0)), rgb[0], rgb[1], rgb[2]));
    }
    for (i, o) in world.obstacles.iter().enumerate() {
        let gf = |k: &str, d: f64| o.get(k).and_then(|v| v.as_f64()).unwrap_or(d);
        let z = o.get("z").and_then(|v| v.as_f64()).or_else(|| o.get("height").and_then(|v| v.as_f64())).unwrap_or(1.0);
        let typ = o.get("type").and_then(|t| t.as_str());
        let rgb = obstacle_color(typ);
        let (w, h) = (gf("w", 0.8), gf("h", 0.8));
        let shape = if typ == Some("person") { format!("type=\"cylinder\" size=\"{} {}\"", q(w.max(h) / 2.0), q(z / 2.0)) }
            else { format!("type=\"box\" size=\"{} {} {}\"", q(w / 2.0), q(h / 2.0), q(z / 2.0)) };
        g.push(format!("<body name=\"ob{i}\" mocap=\"true\" pos=\"{} {} 0\" euler=\"0 0 {}\"><geom name=\"o{i}\" {shape} pos=\"0 0 {}\" rgba=\"{} {} {} 1\" group=\"0\"/></body>",
            q(gf("x", 0.0)), q(gf("y", 0.0)), q(gf("yaw", 0.0)), q(z / 2.0), rgb[0], rgb[1], rgb[2]));
    }
    for (i, s) in decor.stations.iter().enumerate() {
        g.push(format!("<geom name=\"st{i}\" type=\"cylinder\" pos=\"{} {} 0.0005\" size=\"0.35 0.0005\" rgba=\"0.10 0.70 0.35 1\" contype=\"0\" conaffinity=\"0\" group=\"2\"/>", q(s[0]), q(s[1])));
    }
    for (i, t) in decor.tags.iter().enumerate() {
        g.push(format!("<geom name=\"tag{i}\" type=\"box\" pos=\"{} {} 0.0008\" size=\"0.04 0.04 0.0008\" rgba=\"0.05 0.05 0.05 1\" contype=\"0\" conaffinity=\"0\" group=\"2\"/>", q(t[0]), q(t[1])));
    }
    for (i, l) in decor.lanes.iter().enumerate() {
        let len = (l[2] - l[0]).hypot(l[3] - l[1]);
        if len < 0.05 { continue; }
        g.push(format!("<geom name=\"ln{i}\" type=\"box\" pos=\"{} {} 0.0003\" size=\"{} 0.025 0.0003\" euler=\"0 0 {}\" rgba=\"0.95 0.80 0.10 1\" contype=\"0\" conaffinity=\"0\" group=\"2\"/>",
            q((l[0] + l[2]) / 2.0), q((l[1] + l[3]) / 2.0), q(len / 2.0), q((l[3] - l[1]).atan2(l[2] - l[0]))));
    }
    // robot
    let p = &robot.footprint;
    let xmax = p.iter().map(|v| v[0]).fold(f64::NEG_INFINITY, f64::max);
    let xmin = p.iter().map(|v| v[0]).fold(f64::INFINITY, f64::min);
    let ymax = p.iter().map(|v| v[1]).fold(f64::NEG_INFINITY, f64::max);
    let ymin = p.iter().map(|v| v[1]).fold(f64::INFINITY, f64::min);
    let (bx, by) = ((xmax + xmin) / 2.0, (ymax + ymin) / 2.0);
    let (hx, hy) = ((xmax - xmin) / 2.0, (ymax - ymin) / 2.0);
    let (hh, cl, mass) = (robot.height, robot.clearance, robot.mass);
    let base_h = if hh > 0.6 { hh.min(0.35) } else { hh };
    let izz = mass * (4.0 * hx * hx + 4.0 * hy * hy) / 12.0;
    let (kv_lin, kv_ang, f_lin) = (mass * 40.0, izz * 40.0, mass * 4.0);
    let mut robot_xml = format!("<body name=\"agv\" pos=\"0 0 0\"><joint name=\"jx\" type=\"slide\" axis=\"1 0 0\" damping=\"0\"/><joint name=\"jy\" type=\"slide\" axis=\"0 1 0\" damping=\"0\"/><joint name=\"jyaw\" type=\"hinge\" axis=\"0 0 1\" damping=\"0\"/><geom name=\"agv_base\" type=\"box\" pos=\"{} {} {}\" size=\"{} {} {}\" mass=\"{}\" rgba=\"0.95 0.55 0.1 1\" group=\"1\" condim=\"3\" friction=\"0.4\"/>",
        q(bx), q(by), q(cl + base_h / 2.0), q(hx), q(hy), q(base_h / 2.0), q(mass));
    if hh - base_h - cl > 0.05 {
        let up = hh - base_h - cl;
        robot_xml += &format!("<geom name=\"agv_upper\" type=\"box\" pos=\"{} {} {}\" size=\"{} {} {}\" mass=\"0.001\" rgba=\"0.85 0.85 0.88 0.6\" group=\"1\"/>",
            q(bx), q(by), q(cl + base_h + up / 2.0), q(hx), q(hy), q(up / 2.0));
    }
    robot_xml += "</body>";
    format!("<mujoco model=\"agv_world\">
  <compiler angle=\"radian\"/>
  <option timestep=\"{dt}\" integrator=\"implicitfast\" gravity=\"0 0 0\" cone=\"elliptic\"/>
  <visual><global offwidth=\"1280\" offheight=\"960\"/><quality shadowsize=\"0\"/>
    <headlight ambient=\"0.45 0.45 0.45\" diffuse=\"0.35 0.35 0.35\" specular=\"0 0 0\"/></visual>
  <asset>
    <texture name=\"floor_tex\" type=\"2d\" builtin=\"checker\" rgb1=\"0.78 0.78 0.76\" rgb2=\"0.72 0.72 0.70\" width=\"256\" height=\"256\"/>
    <material name=\"floor_mat\" texture=\"floor_tex\" texrepeat=\"{} {}\" rgba=\"1 1 1 1\"/>
  </asset>
  <worldbody>
    <light pos=\"{} {} 5.5\" dir=\"0.3 0.2 -1\" directional=\"true\" diffuse=\"0.6 0.6 0.6\" castshadow=\"false\"/>
    <geom name=\"floor\" type=\"plane\" pos=\"{} {} 0\" size=\"{} {} 0.1\" material=\"floor_mat\" contype=\"0\" conaffinity=\"0\" group=\"2\"/>
    <geom name=\"ceiling\" type=\"box\" pos=\"{} {} {}\" size=\"{} {} 0.05\" rgba=\"0.92 0.92 0.94 1\" contype=\"0\" conaffinity=\"0\" group=\"2\"/>
    {}
    {}
  </worldbody>
  <actuator>
    <velocity name=\"vx\" joint=\"jx\" kv=\"{}\" forcelimited=\"true\" forcerange=\"{} {}\"/>
    <velocity name=\"vy\" joint=\"jy\" kv=\"{}\" forcelimited=\"true\" forcerange=\"{} {}\"/>
    <velocity name=\"wz\" joint=\"jyaw\" kv=\"{}\"/>
  </actuator>
</mujoco>",
        q(sx * 2.0), q(sy * 2.0), q(cx), q(cy), q(cx), q(cy), q(sx), q(sy), q(cx), q(cy), CEILING_HEIGHT + 0.05, q(sx), q(sy),
        g.iter().map(|s| format!("    {s}")).collect::<Vec<_>>().join("\n"), robot_xml,
        q(kv_lin), q(-f_lin), q(f_lin), q(kv_lin), q(-f_lin), q(f_lin), q(kv_ang))
}

pub struct MjCtx {
    pub lib: Arc<ms::MuJoCo>,
    pub m: *mut ms::mjModel,
    pub d: *mut ms::mjData,
    pub robot_body: i32,
    pub robot_geom: Vec<bool>,
    pub mocap: Vec<i32>, // per obstacle index -> mocap id (or -1)
}
unsafe impl Send for MjCtx {}

impl Drop for MjCtx {
    fn drop(&mut self) {
        unsafe { self.lib.mj_deleteData(self.d); self.lib.mj_deleteModel(self.m); }
    }
}

impl MjCtx {
    pub fn load(lib: Arc<ms::MuJoCo>, xml: &str, n_obstacles: usize) -> Result<MjCtx, String> {
        static CNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
        let path = std::env::temp_dir().join(format!("agv_sim_{}_{}.xml", std::process::id(), CNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)));
        std::fs::write(&path, xml).map_err(|e| e.to_string())?;
        let c = CString::new(path.to_str().unwrap()).unwrap();
        let mut err = vec![0u8; 1024];
        let m = unsafe { lib.mj_loadXML(c.as_ptr() as *const _, std::ptr::null(), err.as_mut_ptr() as *mut _, 1024) };
        let _ = std::fs::remove_file(&path);
        if m.is_null() {
            let n = err.iter().position(|&b| b == 0).unwrap_or(0);
            return Err(format!("MuJoCo compile failed: {}", String::from_utf8_lossy(&err[..n])));
        }
        let d = unsafe { lib.mj_makeData(m) };
        unsafe { lib.mj_forward(m, d); }
        let name = |obj: i32, s: &str| -> i32 { let c = CString::new(s).unwrap(); unsafe { lib.mj_name2id(m, obj, c.as_ptr() as *const _) } };
        let robot_body = name(ms::mjtObj_mjOBJ_BODY as i32, "agv");
        let ngeom = unsafe { (*m).ngeom } as usize;
        let robot_geom: Vec<bool> = (0..ngeom).map(|g| unsafe { *(*m).geom_bodyid.add(g) } == robot_body).collect();
        let mocap: Vec<i32> = (0..n_obstacles).map(|i| {
            let b = name(ms::mjtObj_mjOBJ_BODY as i32, &format!("ob{i}"));
            if b < 0 { -1 } else { unsafe { *(*m).body_mocapid.add(b as usize) } }
        }).collect();
        Ok(MjCtx { lib, m, d, robot_body, robot_geom, mocap })
    }

    pub fn qpos(&self, i: usize) -> f64 { unsafe { *(*self.d).qpos.add(i) } }
    pub fn qvel(&self, i: usize) -> f64 { unsafe { *(*self.d).qvel.add(i) } }
    pub fn nq(&self) -> usize { unsafe { (*self.m).nq as usize } }
    pub fn set_qpos(&mut self, i: usize, v: f64) { unsafe { *(*self.d).qpos.add(i) = v } }
    pub fn set_qvel(&mut self, i: usize, v: f64) { unsafe { *(*self.d).qvel.add(i) = v } }
    pub fn set_ctrl(&mut self, i: usize, v: f64) { unsafe { *(*self.d).ctrl.add(i) = v } }
    pub fn forward(&mut self) { unsafe { self.lib.mj_forward(self.m, self.d) } }
    pub fn step(&mut self) { unsafe { self.lib.mj_step(self.m, self.d) } }
    pub fn ngeom(&self) -> usize { self.robot_geom.len() }

    pub fn reset_pose(&mut self, x: f64, y: f64, th: f64) {
        self.set_qpos(0, x); self.set_qpos(1, y); self.set_qpos(2, th);
        for i in 0..3 { self.set_qvel(i, 0.0); self.set_ctrl(i, 0.0); }
        self.forward();
    }

    pub fn move_obstacle(&mut self, idx: usize, x: f64, y: f64, yaw: f64) {
        let Some(&mid) = self.mocap.get(idx) else { return };
        if mid < 0 { return; }
        unsafe {
            let mid = mid as usize;
            let p = (*self.d).mocap_pos.add(3 * mid);
            *p = x; *p.add(1) = y; *p.add(2) = 0.0;
            let qd = (*self.d).mocap_quat.add(4 * mid);
            *qd = (yaw / 2.0).cos(); *qd.add(1) = 0.0; *qd.add(2) = 0.0; *qd.add(3) = (yaw / 2.0).sin();
        }
    }

    /// Contact between a robot geom and a non-robot geom with dist < 2 mm -> contact point.
    pub fn robot_contact(&self) -> Option<(f64, f64)> {
        unsafe {
            let d = &*self.d;
            for i in 0..d.ncon as usize {
                let con = &*d.contact.add(i);
                let (g1, g2) = (con.geom[0], con.geom[1]);
                let r = |g: i32| g >= 0 && (g as usize) < self.robot_geom.len() && self.robot_geom[g as usize];
                if r(g1) != r(g2) && con.dist < 0.002 { return Some((con.pos[0], con.pos[1])); }
            }
        }
        None
    }

    /// Horizontal rays from (ox,oy,z) with world angles; group-0 geoms only; unhit/out of range = inf
    pub fn rays2d(&mut self, ox: f64, oy: f64, z: f64, angles: &[f64], maxr: f64, out: &mut [f64], scratch_vec: &mut Vec<f64>, scratch_gid: &mut Vec<i32>) {
        let n = angles.len();
        scratch_vec.clear();
        for a in angles { let (s, c) = a.sin_cos(); scratch_vec.extend_from_slice(&[c, s, 0.0]); }
        scratch_gid.resize(n, -1);
        let pnt = [ox, oy, z];
        let group: [u8; 6] = [1, 0, 0, 0, 0, 0];
        unsafe {
            self.lib.mj_multiRay(self.m, self.d, pnt.as_ptr(), scratch_vec.as_ptr(), group.as_ptr(), true as _, self.robot_body,
                scratch_gid.as_mut_ptr(), out.as_mut_ptr(), std::ptr::null_mut(), n as i32, maxr);
        }
        for i in 0..n { if scratch_gid[i] < 0 || out[i] < 0.0 || out[i] > maxr { out[i] = f64::INFINITY; } }
    }

    /// 3D rays (world unit dirs, flat xyz) from origin; returns dist with inf for no hit
    pub fn rays3d(&mut self, o: [f64; 3], dirs: &[f64], maxr: f64, dist: &mut [f64], gid: &mut Vec<i32>) {
        let n = dist.len();
        gid.resize(n, -1);
        let group: [u8; 6] = [1, 0, 0, 0, 0, 0];
        unsafe {
            self.lib.mj_multiRay(self.m, self.d, o.as_ptr(), dirs.as_ptr(), group.as_ptr(), true as _, self.robot_body,
                gid.as_mut_ptr(), dist.as_mut_ptr(), std::ptr::null_mut(), n as i32, maxr);
        }
        for i in 0..n { if gid[i] < 0 || dist[i] < 0.0 { dist[i] = f64::INFINITY; } }
    }
}
