//! Optional Nav2 backend: drives the existing C++ `agv_ros_bridge` (core mode) + `ros2 launch nav2_launch.py`.
//!
//! Default is OFF (pure guidance). It is started lazily when the planner is switched to "nav2"
//! (PUT /api/v1/nav/planner {"type":"nav2"}) or when NAV_NAV2=1. The bridge connects to the simulator itself
//! (stream + UDP cmd), runs the C++ safety layer and forwards Nav2 /cmd_vel; Rust only sends CONFIG / MODE /
//! ROUTE / CANCEL over an abstract Unix datagram socket and receives NAV feedback.
use crate::navigator::Navigator;
use serde_json::{json, Value};
use std::os::linux::net::SocketAddrExt;
use std::os::unix::net::{SocketAddr, UnixDatagram};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicI64, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

const MAGIC: &[u8] = b"AGV1";
/// process-group ids of children (signal-handler safe), killed on SIGTERM so Nav2 never outlives nav-runtime
pub static PGIDS: [std::sync::atomic::AtomicI32; 4] = [std::sync::atomic::AtomicI32::new(0), std::sync::atomic::AtomicI32::new(0), std::sync::atomic::AtomicI32::new(0), std::sync::atomic::AtomicI32::new(0)];
pub fn reg_pgid(slot: usize, pid: i32) { PGIDS[slot].store(pid, Ordering::SeqCst); }
pub fn kill_all_children() { for p in PGIDS.iter() { let g = p.load(Ordering::SeqCst); if g > 0 { unsafe { libc::kill(-g, libc::SIGKILL); } } } }
const T_ROUTE: u8 = 3;
const T_CANCEL: u8 = 4;
const T_CONFIG: u8 = 5;
const T_MODE: u8 = 6;
const T_NAV: u8 = 12;
const T_SAFETY: u8 = 30;

pub struct Nav2 {
    nav: Weak<Navigator>,
    started: AtomicBool,
    nav2_active: AtomicBool,
    bridge_up: AtomicBool,
    active: AtomicBool,
    token: AtomicI64,
    seq: AtomicI64,
    tx: UnixDatagram,
    rx: Mutex<Option<UnixDatagram>>,
    to: SocketAddr,
    kids: Mutex<Vec<Child>>,
    pose: Mutex<([f64; 3], [f64; 3])>, // (truth, odom)
    tag: String,
    pub safety: Mutex<Value>,
    last: Mutex<Option<(u64, Vec<(f64, f64)>, [f64; 3], Option<Vec<bool>>, std::time::Instant, u8)>>,
    ready_since: Mutex<Option<std::time::Instant>>,
    n2_child: Mutex<Option<Child>>,
    n2_launched: Mutex<Option<std::time::Instant>>,
    n2_tries: std::sync::atomic::AtomicU32,
}

fn pack(t: u8, body: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(5 + body.len() + 1);
    v.extend_from_slice(MAGIC);
    v.push(t);
    v.extend_from_slice(body);
    if body.is_empty() { v.push(0); } // older bridges drop exactly-5-byte frames
    v
}

fn find_bin() -> Option<String> {
    let home = std::env::var("AGV_HOME").unwrap_or("/opt/agv".into());
    for c in [std::env::var("NAV_ROS_BRIDGE_BIN").unwrap_or_default(),
              format!("{home}/ros2/install/agv_ros_bridge/lib/agv_ros_bridge/agv_ros_bridge"),
              "/opt/agv_ros/lib/agv_ros_bridge/agv_ros_bridge".into()] {
        if !c.is_empty() && std::path::Path::new(&c).is_file() { return Some(c); }
    }
    None
}

impl Nav2 {
    pub fn new(nav: Weak<Navigator>) -> Arc<Nav2> {
        let tag = format!("agv_bridge_rs_{}_{}", std::process::id(), std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_millis() % 100_000_000).unwrap_or(0));
        let rx = UnixDatagram::bind_addr(&SocketAddr::from_abstract_name(format!("{tag}_py").as_bytes()).unwrap()).expect("bridge rx socket");
        rx.set_read_timeout(Some(Duration::from_millis(500))).ok();
        let tx = UnixDatagram::unbound().expect("bridge tx socket");
        tx.set_nonblocking(true).ok();
        Arc::new(Nav2 {
            nav, started: AtomicBool::new(false), nav2_active: AtomicBool::new(false), bridge_up: AtomicBool::new(false), active: AtomicBool::new(false),
            token: AtomicI64::new(-1), seq: AtomicI64::new(0), tx, rx: Mutex::new(Some(rx)),
            to: SocketAddr::from_abstract_name(format!("{tag}_cpp").as_bytes()).unwrap(),
            kids: Mutex::new(vec![]), pose: Mutex::new(([0.0; 3], [0.0; 3])), tag, safety: Mutex::new(Value::Null), last: Mutex::new(None), ready_since: Mutex::new(None),
            n2_child: Mutex::new(None), n2_launched: Mutex::new(None), n2_tries: std::sync::atomic::AtomicU32::new(0),
        })
    }

    pub fn started(&self) -> bool { self.started.load(Ordering::Relaxed) }
    /// lifecycle active AND stable for 6 s (right after bring-up goals get ABORTED)
    pub fn ready(&self) -> bool {
        let up = self.started() && self.nav2_active.load(Ordering::Relaxed);
        let mut rs = self.ready_since.lock().unwrap();
        if !up { *rs = None; return false; }
        rs.get_or_insert_with(std::time::Instant::now).elapsed().as_secs_f64() > 6.0
    }
    pub fn active(&self) -> bool { self.active.load(Ordering::Relaxed) }
    pub fn set_active(&self, v: bool) { self.active.store(v, Ordering::Relaxed); }
    pub fn bridge_up(&self) -> bool { self.bridge_up.load(Ordering::Relaxed) }
    pub fn feed(&self, truth: [f64; 3], odom: [f64; 3]) { *self.pose.lock().unwrap() = (truth, odom); }

    fn send(&self, t: u8, body: &[u8]) { let _ = self.tx.send_to_addr(&pack(t, body), &self.to); }

    fn spawn(&self, name: &str, argv: &[String], env: &[(&str, String)]) -> Result<Child, String> {
        let home = std::env::var("AGV_HOME").unwrap_or("/opt/agv".into());
        let mut c = Command::new("bash");
        c.arg("-c").arg(format!(". {home}/docker/nav2_env.sh; exec \"$@\"")).arg("_");
        c.args(argv).stdin(Stdio::null());
        let logd = std::env::var("AGV_LOG_DIR").unwrap_or("/tmp".into());
        if let Ok(f) = std::fs::OpenOptions::new().create(true).append(true).open(format!("{logd}/nav2_{name}.log")) {
            if let Ok(g) = f.try_clone() { c.stdout(g); }
            c.stderr(f);
        }
        for (k, v) in env { c.env(k, v); }
        unsafe { use std::os::unix::process::CommandExt; c.pre_exec(|| { libc::setpgid(0, 0); Ok(()) }); }
        let ch = c.spawn().map_err(|e| format!("启动 {name} 失败: {e}"))?;
        reg_pgid(if name == "bridge" { 0 } else { 1 }, ch.id() as i32);
        Ok(ch)
    }

    /// Start bridge + Nav2 (idempotent).
    pub fn start(self: &Arc<Self>) -> Result<(), String> {
        if self.started.swap(true, Ordering::SeqCst) { return Ok(()); }
        let nav = self.nav.upgrade().ok_or("navigator gone")?;
        let bin = match find_bin() { Some(b) => b, None => { self.started.store(false, Ordering::SeqCst); return Err("未找到 agv_ros_bridge (ros2/install 未构建)".into()); } };
        let home = std::env::var("AGV_HOME").unwrap_or("/opt/agv".into());
        let sim = std::env::var("SIM_API").unwrap_or("http://127.0.0.1:8090".into());
        let r = self.spawn("bridge", &[bin, "--core".into(), "--in".into(), format!("@{}_cpp", self.tag), "--out".into(), format!("@{}_py", self.tag)],
            &[("SIM_API", sim)]).map(|c| self.kids.lock().unwrap().push(c));
        let _ = &home; let _ = &nav;
        if let Err(e) = r { self.stop(); self.started.store(false, Ordering::SeqCst); return Err(e); }
        self.n2_tries.store(0, Ordering::SeqCst);
        *self.n2_launched.lock().unwrap() = None;
        let rx = self.rx.lock().unwrap().take();
        if let Some(rx) = rx { let me = self.clone(); std::thread::Builder::new().name("nav2-rx".into()).spawn(move || me.rx_loop(rx)).ok(); }
        let me = self.clone();
        std::thread::Builder::new().name("nav2-sync".into()).spawn(move || me.sync_loop()).ok();
        Ok(())
    }

    fn launch_nav2(&self) -> Result<(), String> {
        let nav = self.nav.upgrade().ok_or("navigator gone")?;
        let home = std::env::var("AGV_HOME").unwrap_or("/opt/agv".into());
        let (chassis, scenario) = { let s = nav.s(); (s.chassis_type.clone(), s.active_scenario.clone()) };
        let chassis = if ["diff_drive", "single_steer", "dual_steer"].contains(&chassis.as_str()) { chassis } else { "dual_steer".into() };
        let (x, y, yaw) = { let p = self.pose.lock().unwrap(); (p.0[0], p.0[1], p.0[2]) };
        let c = self.spawn("nav2", &["ros2".into(), "launch".into(), format!("{home}/nav2/nav2_launch.py"), format!("chassis:={chassis}"),
            format!("map:={home}/maps/{scenario}.yaml"), "localization:=ground_truth".into(), "use_sim_time:=false".into(), "map_source:=map_server".into(),
            format!("initial_x:={x}"), format!("initial_y:={y}"), format!("initial_yaw:={yaw}")], &[])?;
        *self.n2_child.lock().unwrap() = Some(c);
        *self.n2_launched.lock().unwrap() = Some(std::time::Instant::now());
        self.n2_tries.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }

    fn kill_nav2(&self) {
        if let Some(mut c) = self.n2_child.lock().unwrap().take() {
            unsafe { libc::kill(-(c.id() as i32), libc::SIGINT); }
            std::thread::sleep(Duration::from_millis(1500));
            unsafe { libc::kill(-(c.id() as i32), libc::SIGKILL); }
            let _ = c.kill();
            let _ = c.wait();
        }
        *self.n2_launched.lock().unwrap() = None;
        reg_pgid(1, 0);
    }

    /// bridge up → launch Nav2; Nav2 not active 75 s after launch → relaunch (max 3 tries)
    fn supervise(&self) {
        let launched = *self.n2_launched.lock().unwrap();
        match launched {
            None => { if self.bridge_up() && self.n2_tries.load(Ordering::SeqCst) < 3 { std::thread::sleep(Duration::from_secs(2)); let _ = self.launch_nav2(); } }
            Some(t) => {
                if !self.nav2_active.load(Ordering::Relaxed) && t.elapsed().as_secs() > 75 && self.n2_tries.load(Ordering::SeqCst) < 3 {
                    eprintln!("[nav2] 75 s 内未就绪，重启 Nav2 (第 {} 次)", self.n2_tries.load(Ordering::SeqCst) + 1);
                    self.kill_nav2();
                }
            }
        }
    }

    pub fn stop(&self) {
        self.kill_nav2();
        for mut c in self.kids.lock().unwrap().drain(..) {
            unsafe { libc::kill(-(c.id() as i32), libc::SIGINT); }
            std::thread::sleep(Duration::from_millis(300));
            let _ = c.kill();
            let _ = c.wait();
        }
        self.nav2_active.store(false, Ordering::Relaxed);
        self.started.store(false, Ordering::SeqCst);
    }

    fn config(&self, nav: &Navigator) -> Value {
        let s = nav.s();
        let o = Navigator::outline_of(&s);
        json!({"prot": s.prot.json, "outline": o, "photos": [], "refine_lidar": {}, "media": {"lidars3d": [], "cameras": []},
               "max_decel": s.max_decel, "max_ang_decel": s.chassis_cfg.get("max_ang_decel_radps2").and_then(|v| v.as_f64()).unwrap_or(1.0),
               "loc_stale_s": 1.0})
    }

    fn mode(&self, nav: &Navigator) -> Value {
        let (truth, odom) = *self.pose.lock().unwrap();
        // map→odom = truth ∘ odom⁻¹ (ground-truth localisation)
        let yaw = truth[2] - odom[2];
        let (c, s) = (yaw.cos(), yaw.sin());
        let m = [truth[0] - (c * odom[0] - s * odom[1]), truth[1] - (s * odom[0] + c * odom[1]), yaw];
        let (fwd, cap) = {
            let st = nav.s();
            (st.active_planner == "nav2" && ["NAVIGATING", "PLANNING", "OBSTACLE_WAIT"].contains(&st.tele.nav_status.as_str()), st.speed_cap.unwrap_or(0.0))
        };
        json!({"own_odom": true, "map_odom": true, "m2o": m, "loc_ext": false, "want_raw_lidar": false, "nav2_forward": fwd, "speed_cap": cap})
    }

    fn sync_loop(self: Arc<Self>) {
        let mut n = 0u64;
        loop {
            std::thread::sleep(Duration::from_millis(200));
            if !self.started() { return; }
            if n % 5 == 0 { self.supervise(); }
            let Some(nav) = self.nav.upgrade() else { return };
            if n % 25 == 0 || !self.bridge_up() { let c = serde_json::to_vec(&self.config(&nav)).unwrap_or_default(); self.send(T_CONFIG, &c); }
            let m = serde_json::to_vec(&self.mode(&nav)).unwrap_or_default();
            self.send(T_MODE, &m);
            n += 1;
        }
    }

    fn rx_loop(self: Arc<Self>, rx: UnixDatagram) {
        let mut buf = vec![0u8; 1 << 20];
        loop {
            let n = match rx.recv(&mut buf) { Ok(n) => n, Err(e) if matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => continue, Err(_) => { std::thread::sleep(Duration::from_millis(200)); continue } };
            if n < 5 || &buf[..4] != MAGIC { continue; }
            self.bridge_up.store(true, Ordering::Relaxed);
            let ty = buf[4];
            if ty != T_NAV && ty != T_SAFETY { continue; }
            let Ok(m) = serde_json::from_slice::<Value>(&buf[5..n]) else { continue };
            if ty == T_SAFETY { *self.safety.lock().unwrap() = m; continue; }
            let Some(nav) = self.nav.upgrade() else { return };
            match m["k"].as_str().unwrap_or("") {
                "ros" => self.nav2_active.store(m["nav2_active"].as_bool().unwrap_or(false), Ordering::Relaxed),
                "fb" if m["mid"].as_i64() == Some(self.token.load(Ordering::Relaxed)) => {
                    nav.s().tele.nav_dist_rem = (m["dist"].as_f64().unwrap_or(0.0) * 100.0).round() / 100.0;
                }
                "result" if m["mid"].as_i64() == Some(self.token.load(Ordering::Relaxed)) => {
                    let res = m["result"].as_str().unwrap_or("ABORTED");
                    let retry = if res == "ABORTED" {
                        let mut l = self.last.lock().unwrap();
                        match l.as_mut() { Some(x) if x.4.elapsed().as_secs_f64() < 6.0 && x.5 < 3 => { x.5 += 1; Some((x.0, x.1.clone(), x.2, x.3.clone(), x.5)) } _ => None }
                    } else { None };
                    if let Some((mid, r, g, rv, k)) = retry {
                        let me = self.clone();
                        std::thread::spawn(move || { std::thread::sleep(Duration::from_secs(3)); if me.token.load(Ordering::Relaxed) < 0 { return; } me.route_inner(mid, &r, g, rv.as_deref(), k); });
                        continue;
                    }
                    nav.nav2_result((self.token.load(Ordering::Relaxed) / 1000) as u64, m["result"].as_str().unwrap_or("ABORTED"));
                }
                "plan" => {
                    let c: Vec<(f64, f64)> = m["curve"].as_array().map(|a| a.iter().filter_map(|p| Some((p[0].as_f64()?, p[1].as_f64()?))).collect()).unwrap_or_default();
                    if !c.is_empty() { nav.s().tele.plan_curve = c; }
                }
                "sevent" => {}
                _ => {}
            }
        }
    }

    pub fn route(&self, mid: u64, route: &[(f64, f64)], goal: [f64; 3], rev: Option<&[bool]>) { self.route_inner(mid, route, goal, rev, 0) }
    fn route_inner(&self, mid: u64, route: &[(f64, f64)], goal: [f64; 3], rev: Option<&[bool]>, tries: u8) {
        *self.last.lock().unwrap() = Some((mid, route.to_vec(), goal, rev.map(|r| r.to_vec()), std::time::Instant::now(), tries));
        let home = std::env::var("AGV_HOME").unwrap_or("/opt/agv".into());
        let seq = self.seq.fetch_add(1, Ordering::SeqCst) + 1;
        let tok = mid as i64 * 1000 + seq % 1000;
        self.token.store(tok, Ordering::SeqCst);
        self.active.store(true, Ordering::SeqCst);
        let body = json!({"mid": tok, "route": route.iter().map(|p| json!([(p.0 * 1e4).round() / 1e4, (p.1 * 1e4).round() / 1e4])).collect::<Vec<_>>(),
            "rev": rev.map(|r| std::iter::once(0).chain(r.iter().map(|x| *x as i32)).collect::<Vec<_>>()).unwrap_or_default(),
            "goal": goal, "bt": format!("{home}/nav2/agv_route_bt.xml"), "segs": []});
        self.send(T_ROUTE, serde_json::to_vec(&body).unwrap_or_default().as_slice());
    }

    pub fn cancel(&self) {
        self.token.store(-1, Ordering::SeqCst);
        self.active.store(false, Ordering::SeqCst);
        self.send(T_CANCEL, &[0]);
    }
}
