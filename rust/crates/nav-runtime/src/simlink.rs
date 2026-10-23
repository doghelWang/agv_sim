//! SimLink: nav-process side of the sim connection (pure REST + push stream + UDP cmd).
use rest::Client;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

#[derive(Clone, Debug, Default)]
pub struct State {
    pub seq: u32,
    pub t: f64,
    pub truth: [f64; 6],
    pub odom: [f64; 6],
    pub steer: Vec<f64>,
    pub paused: bool,
    pub chassis: String,
    pub scenario: String,
    pub model_rev: u32,
}

pub struct Scan { pub ranges: Vec<f64>, pub angle_min: f64, pub angle_inc: f64, pub range_max: f64 }

pub trait Sink: Send + Sync {
    fn on_state(&self, st: &State);
    fn on_merged(&self, s: &Scan);
    fn on_world(&self, w: &Value);
    fn on_model(&self, m: &Value);
    fn feedback(&self) -> Value;
}

pub struct SimLink {
    pub url: String,
    pub c_misc: Client,
    c_cmd: Client,
    pub state: Mutex<State>,
    pub io: Mutex<Value>,
    pub photos: Mutex<HashMap<String, Value>>,
    pub model: Mutex<Value>,
    pub world: Mutex<Value>,
    pub sensors: Mutex<Value>,
    pub online: AtomicBool,
    pub stop: AtomicBool,
    stream_io: AtomicBool,
    sink: OnceLock<Arc<dyn Sink>>,
    udp: Mutex<Option<(UdpSocket, std::net::SocketAddr)>>,
    cmd_seq: AtomicU64,
    pub stats: Mutex<Value>,
    pub meta: Mutex<Value>,
}

fn f64le(b: &[u8], o: usize) -> f64 { f64::from_le_bytes(b[o..o + 8].try_into().unwrap()) }

pub fn decode_state(b: &[u8], meta: &Value) -> Option<State> {
    if b.len() < 207 { return None; }
    let seq = u32::from_le_bytes(b[0..4].try_into().unwrap());
    let t = f64le(b, 4);
    let mut truth = [0.0; 6];
    let mut odom = [0.0; 6];
    for i in 0..6 { truth[i] = f64le(b, 20 + 8 * i); odom[i] = f64le(b, 68 + 8 * i); }
    let rev = u32::from_le_bytes(b[201..205].try_into().unwrap());
    let flags = b[184];
    let nj = u16::from_le_bytes(b[205..207].try_into().unwrap()) as usize;
    let off = 207;
    let names: Vec<String> = meta["joint_names"].as_array().map(|a| a.iter().map(|x| x.as_str().unwrap_or("").to_string()).collect()).unwrap_or_default();
    let mut steer = vec![];
    if b.len() >= off + 24 * nj {
        for j in 0..nj {
            let n = names.get(j).cloned().unwrap_or_default();
            if n.to_lowercase().contains("steer") { steer.push(f64le(b, off + 8 * j)); }
        }
    }
    Some(State { seq, t, truth, odom, steer, paused: flags & 4 != 0,
        chassis: meta["chassis"].as_str().unwrap_or("").to_string(), scenario: meta["scenario"].as_str().unwrap_or("").to_string(), model_rev: rev })
}

impl SimLink {
    pub fn new(url: &str) -> Arc<SimLink> {
        Arc::new(SimLink {
            url: url.trim_end_matches('/').to_string(), c_misc: Client::new(url, 3.0), c_cmd: Client::new(url, 1.0),
            state: Mutex::new(State::default()), io: Mutex::new(json!({})), photos: Mutex::new(HashMap::new()),
            model: Mutex::new(json!({})), world: Mutex::new(json!({})), sensors: Mutex::new(json!({})),
            online: AtomicBool::new(false), stop: AtomicBool::new(false), stream_io: AtomicBool::new(false),
            sink: OnceLock::new(), udp: Mutex::new(None), cmd_seq: AtomicU64::new(0),
            stats: Mutex::new(json!({"state_hz": 0.0, "cmd_sent": 0, "errors": 0, "transport": "poll"})), meta: Mutex::new(json!({})),
        })
    }
    pub fn set_sink(&self, s: Arc<dyn Sink>) { let _ = self.sink.set(s); }
    pub fn is_online(&self) -> bool { self.online.load(Ordering::Relaxed) }
    pub fn world(&self) -> Value { self.world.lock().unwrap().clone() }
    pub fn io(&self) -> Value { self.io.lock().unwrap().clone() }
    pub fn sensors(&self) -> Value { self.sensors.lock().unwrap().clone() }
    pub fn stat_add(&self, k: &str, n: i64) {
        let mut s = self.stats.lock().unwrap();
        let v = s[k].as_i64().unwrap_or(0) + n;
        s[k] = json!(v);
    }
    fn stat_set(&self, k: &str, v: Value) { self.stats.lock().unwrap()[k] = v; }

    pub fn start(self: &Arc<Self>) {
        for (name, f) in [("state", Self::state_loop as fn(&Arc<Self>)), ("slow", Self::slow_loop), ("feedback", Self::feedback_loop)] {
            let me = self.clone();
            std::thread::Builder::new().name(format!("simlink-{name}")).spawn(move || f(&me)).unwrap();
        }
    }

    fn handle_state(&self, st: State) {
        self.online.store(true, Ordering::Relaxed);
        let prev = std::mem::replace(&mut *self.state.lock().unwrap(), st.clone());
        if let Some(s) = self.sink.get() { s.on_state(&st); }
        if prev.seq != 0 || prev.t != 0.0 {
            if prev.chassis != st.chassis || prev.model_rev != st.model_rev { self.refresh_model(); }
            if prev.scenario != st.scenario { self.refresh_world(); }
        }
    }

    fn connect_stream(&self) -> Option<TcpStream> {
        let u = self.url.trim_start_matches("http://");
        let (hostport, path) = match u.find('/') { Some(i) => (&u[..i], &u[i..]), None => (u, "") };
        let addr = hostport.to_socket_addrs().ok()?.next()?;
        let mut s = TcpStream::connect_timeout(&addr, Duration::from_secs(3)).ok()?;
        s.set_read_timeout(Some(Duration::from_secs(3))).ok()?;
        let req = format!("GET {}/api/v1/stream?hz=50&io_hz=20&scans=1 HTTP/1.1\r\nHost: {}\r\nAccept: */*\r\nConnection: close\r\n\r\n", path.trim_end_matches('/'), hostport);
        s.write_all(req.as_bytes()).ok()?;
        // read response headers
        let mut hdr = vec![];
        let mut one = [0u8; 1];
        while !hdr.ends_with(b"\r\n\r\n") {
            if s.read(&mut one).ok()? == 0 { return None; }
            hdr.push(one[0]);
            if hdr.len() > 8192 { return None; }
        }
        if !hdr.starts_with(b"HTTP/1.1 200") && !hdr.starts_with(b"HTTP/1.0 200") { return None; }
        Some(s)
    }

    /// returns false if the stream is unsupported
    fn stream_loop(&self) -> bool {
        let Some(mut s) = self.connect_stream() else { return false };
        self.stat_set("transport", json!("stream"));
        let mut meta = json!({});
        let (mut n, mut t0) = (0u32, Instant::now());
        loop {
            if self.stop.load(Ordering::Relaxed) { break; }
            let mut h = [0u8; 5];
            if s.read_exact(&mut h).is_err() { break; }
            let ln = u32::from_le_bytes(h[0..4].try_into().unwrap()) as usize;
            let typ = h[4];
            let mut body = vec![0u8; ln];
            if s.read_exact(&mut body).is_err() { break; }
            match typ {
                1 => {
                    if let Some(st) = decode_state(&body, &meta) { self.handle_state(st); }
                    n += 1;
                    if t0.elapsed().as_secs_f64() >= 1.0 {
                        self.stat_set("state_hz", json!((n as f64 / t0.elapsed().as_secs_f64() * 10.0).round() / 10.0));
                        n = 0; t0 = Instant::now();
                    }
                }
                2 => { if let Ok(m) = serde_json::from_slice::<Value>(&body) { *self.meta.lock().unwrap() = m.clone(); meta = m; } }
                3 => {
                    if let Ok(d) = serde_json::from_slice::<Value>(&body) {
                        self.stream_io.store(true, Ordering::Relaxed);
                        *self.io.lock().unwrap() = d["io"].clone();
                        let m: HashMap<String, Value> = d["photos"].as_array().map(|a| a.iter().filter_map(|p| Some((p["name"].as_str()?.to_string(), p.clone()))).collect()).unwrap_or_default();
                        *self.photos.lock().unwrap() = m;
                    }
                }
                4 => {
                    if body.len() < 2 { continue; }
                    let ml = u16::from_le_bytes([body[0], body[1]]) as usize;
                    if body.len() < 2 + ml { continue; }
                    let Ok(m) = serde_json::from_slice::<Value>(&body[2..2 + ml]) else { continue };
                    if m["name"].as_str() == Some("merged") {
                        let rmax = m["range_max"].as_f64().unwrap_or(12.0);
                        let ranges: Vec<f64> = body[2 + ml..].chunks_exact(4).map(|c| { let v = f32::from_le_bytes(c.try_into().unwrap()) as f64; if v.is_finite() { v } else { rmax } }).collect();
                        if let Some(sk) = self.sink.get() {
                            sk.on_merged(&Scan { ranges, angle_min: m["angle_min"].as_f64().unwrap_or(-std::f64::consts::PI), angle_inc: m["angle_increment"].as_f64().unwrap_or(0.0), range_max: rmax });
                        }
                    }
                }
                _ => {}
            }
        }
        self.stream_io.store(false, Ordering::Relaxed);
        self.stat_set("transport", json!("poll"));
        true
    }

    fn state_loop(me: &Arc<Self>) {
        while !me.stop.load(Ordering::Relaxed) {
            if !me.stream_loop() {
                // sim does not support the stream: poll /state
                break;
            }
            me.online.store(false, Ordering::Relaxed);
            std::thread::sleep(Duration::from_millis(500));
        }
        while !me.stop.load(Ordering::Relaxed) {
            let ts = Instant::now();
            match me.c_cmd.get("/api/v1/state") {
                Ok(v) => {
                    let g = |p: &str, k: &str| v[p][k].as_f64().unwrap_or(0.0);
                    let tr = |p: &str| [g(p, "x"), g(p, "y"), g(p, "yaw"), g(p, "vx"), g(p, "vy"), g(p, "wz")];
                    let js = &v["joints"];
                    let steer: Vec<f64> = js["names"].as_array().map(|ns| ns.iter().zip(js["position"].as_array().cloned().unwrap_or_default())
                        .filter(|(n, _)| n.as_str().unwrap_or("").to_lowercase().contains("steer")).map(|(_, p)| p.as_f64().unwrap_or(0.0)).collect()).unwrap_or_default();
                    me.handle_state(State { seq: v["seq"].as_u64().unwrap_or(0) as u32, t: v["t"].as_f64().unwrap_or(0.0), truth: tr("truth"), odom: tr("odom"), steer,
                        paused: v["paused"].as_bool().unwrap_or(false), chassis: v["chassis"].as_str().unwrap_or("").into(),
                        scenario: v["scenario"].as_str().unwrap_or("").into(), model_rev: v["model_rev"].as_u64().unwrap_or(0) as u32 });
                }
                Err(_) => { me.online.store(false, Ordering::Relaxed); me.stat_add("errors", 1); std::thread::sleep(Duration::from_millis(500)); }
            }
            let el = ts.elapsed().as_secs_f64();
            if el < 0.02 { std::thread::sleep(Duration::from_secs_f64(0.02 - el)); }
        }
    }

    fn slow_loop(me: &Arc<Self>) {
        let mut first = true;
        while !me.stop.load(Ordering::Relaxed) {
            if first || me.model.lock().unwrap().as_object().map(|o| o.is_empty()).unwrap_or(true) {
                me.refresh_model();
                me.refresh_world();
                first = false;
            }
            if !me.stream_io.load(Ordering::Relaxed) {
                if let Ok(io) = me.c_misc.get("/api/v1/io") { *me.io.lock().unwrap() = io; }
                if let Ok(pe) = me.c_misc.get("/api/v1/sensors/photoelectric") {
                    let m: HashMap<String, Value> = pe["sensors"].as_array().map(|a| a.iter().filter_map(|p| Some((p["name"].as_str()?.to_string(), p.clone()))).collect()).unwrap_or_default();
                    *me.photos.lock().unwrap() = m;
                }
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    fn refresh_cmd_channel(&self) {
        let port = self.c_misc.get("/api/v1/sim").ok().and_then(|v| v["cmd_udp_port"].as_u64());
        let mut g = self.udp.lock().unwrap();
        *g = None;
        if let (Some(p), true) = (port, std::env::var("NAV_CMD_UDP").map(|v| v != "0").unwrap_or(true)) {
            let host = self.url.trim_start_matches("http://").split(['/', ':']).next().unwrap_or("127.0.0.1").to_string();
            if let Some(addr) = (host.as_str(), p as u16).to_socket_addrs().ok().and_then(|mut a| a.next()) {
                if let Ok(s) = UdpSocket::bind("0.0.0.0:0") { *g = Some((s, addr)); }
            }
        }
    }

    pub fn refresh_model(&self) {
        self.refresh_cmd_channel();
        match (self.c_misc.get("/api/v1/model"), self.c_misc.get("/api/v1/sensors")) {
            (Ok(m), Ok(s)) => {
                *self.model.lock().unwrap() = m.clone();
                *self.sensors.lock().unwrap() = s;
                if let Some(sk) = self.sink.get() { sk.on_model(&m); }
            }
            (Err(e), _) | (_, Err(e)) => eprintln!("[simlink] 刷新模型失败: {e}"),
        }
    }

    pub fn refresh_world(&self) {
        match self.c_misc.get("/api/v1/world") {
            Ok(w) => { *self.world.lock().unwrap() = w.clone(); if let Some(sk) = self.sink.get() { sk.on_world(&w); } }
            Err(e) => eprintln!("[simlink] 刷新场景失败: {e}"),
        }
    }

    pub fn send_cmd(&self, vx: f64, vy: f64, wz: f64, source: &str) -> bool {
        {
            let mut g = self.udp.lock().unwrap();
            if let Some((sock, addr)) = g.as_ref() {
                let seq = (self.cmd_seq.fetch_add(1, Ordering::Relaxed) + 1) as u32;
                let mut p = Vec::with_capacity(48);
                p.extend_from_slice(b"AGVC");
                p.extend_from_slice(&seq.to_le_bytes());
                for v in [vx, vy, wz] { p.extend_from_slice(&v.to_le_bytes()); }
                let mut src = [0u8; 16];
                let sb = source.as_bytes();
                src[..sb.len().min(16)].copy_from_slice(&sb[..sb.len().min(16)]);
                p.extend_from_slice(&src);
                if sock.send_to(&p, addr).is_ok() { drop(g); self.stat_add("cmd_sent", 1); return true; }
                *g = None;
                drop(g);
                self.stat_add("errors", 1);
            }
        }
        match self.c_cmd.put("/api/v1/control/cmd_vel", &json!({"vx": vx, "vy": vy, "wz": wz, "source": source})) {
            Ok(_) => { self.stat_add("cmd_sent", 1); true }
            Err(_) => { self.stat_add("errors", 1); false }
        }
    }

    fn feedback_loop(me: &Arc<Self>) {
        let c = Client::new(&me.url, 1.0);
        while !me.stop.load(Ordering::Relaxed) {
            if let Some(sk) = me.sink.get() { let _ = c.put("/api/v1/nav/feedback", &sk.feedback()); }
            std::thread::sleep(Duration::from_millis(200));
        }
    }
}
