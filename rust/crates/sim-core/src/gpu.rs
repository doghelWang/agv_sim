//! Client for gpucastd (GPU ray-cast service, OpenCL, see sim_core/native/gpucast/gpucastd.c).
//! SIM_GPU_CAST=auto(default)|0|host:port ; SIM_GPU_CAST_MIN (default 250000 rays: measured on the Adreno, GPU loses below that — 320x240 frame 24 ms GPU vs 6 ms CPU). Any error → None (caller falls back to CPU), retry after 5 s.
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::sync::Mutex;
use std::time::{Duration, Instant};

const MAGIC: u32 = 0x3143_5047;

struct Conn { s: TcpStream, scene: Option<(u64, usize, usize)> }
struct State { conn: Option<Conn>, retry_at: Option<Instant>, device: String, calls: u64, errors: u64, gpu_us: u64, wall_us: u64 }
static ST: Mutex<State> = Mutex::new(State { conn: None, retry_at: None, device: String::new(), calls: 0, errors: 0, gpu_us: 0, wall_us: 0 });

fn addr() -> Option<SocketAddr> {
    let v = std::env::var("SIM_GPU_CAST").unwrap_or_else(|_| "auto".into()).trim().to_lowercase();
    match v.as_str() {
        "0" | "off" | "false" | "no" | "" => None,
        "auto" | "1" | "on" | "true" | "yes" => "127.0.0.1:8068".parse().ok(),
        o => o.parse().ok().or_else(|| format!("127.0.0.1:{o}").parse().ok()),
    }
}
pub fn min_rays() -> usize { std::env::var("SIM_GPU_CAST_MIN").ok().and_then(|v| v.parse().ok()).unwrap_or(250000) }

fn hdr(op: u32, n: u32, flags: u32) -> Vec<u8> {
    let mut h = Vec::with_capacity(32);
    for v in [MAGIC, op, n, flags] { h.extend_from_slice(&v.to_le_bytes()); }
    h.extend_from_slice(&[0u8; 16]);
    h
}
fn rd_hdr(s: &mut TcpStream) -> std::io::Result<(u32, u32)> {
    let mut b = [0u8; 16];
    s.read_exact(&mut b)?;
    let magic = u32::from_le_bytes(b[0..4].try_into().unwrap());
    let status = i32::from_le_bytes(b[4..8].try_into().unwrap());
    if magic != MAGIC || status != 0 { return Err(std::io::Error::new(std::io::ErrorKind::Other, "gpucastd bad reply")); }
    Ok((u32::from_le_bytes(b[8..12].try_into().unwrap()), u32::from_le_bytes(b[12..16].try_into().unwrap())))
}
fn connect(a: SocketAddr) -> std::io::Result<(Conn, String)> {
    let mut s = TcpStream::connect_timeout(&a, Duration::from_millis(500))?;
    s.set_nodelay(true).ok();
    s.set_read_timeout(Some(Duration::from_secs(5))).ok();
    s.set_write_timeout(Some(Duration::from_secs(5))).ok();
    s.write_all(&hdr(3, 0, 0))?;
    rd_hdr(&mut s)?;
    let mut d = [0u8; 64];
    s.read_exact(&mut d)?;
    let name = String::from_utf8_lossy(&d[..d.iter().position(|&c| c == 0).unwrap_or(64)]).to_string();
    Ok((Conn { s, scene: None }, name))
}

/// (dist[inf = miss], gid, normals) or None when the GPU service is unavailable.
pub fn cast(key: (u64, usize, usize), prims: &[[f64; 18]], ceil_h: f64, floor_gid: i32, ceil_gid: i32,
            origin: [f64; 3], dirs: &[f64], max_range: f64, want_normal: bool) -> Option<(Vec<f64>, Vec<i32>, Option<Vec<f64>>)> {
    let a = addr()?;
    let mut st = ST.lock().ok()?;
    if st.conn.is_none() {
        if st.retry_at.map(|t| Instant::now() < t).unwrap_or(false) { return None; }
        match connect(a) {
            Ok((c, name)) => { st.conn = Some(c); st.device = name; }
            Err(_) => { st.retry_at = Some(Instant::now() + Duration::from_secs(5)); return None; }
        }
    }
    let t0 = Instant::now();
    let n = dirs.len() / 3;
    let res: std::io::Result<(Vec<f64>, Vec<i32>, Option<Vec<f64>>, u32)> = (|| {
        let c = st.conn.as_mut().unwrap();
        if c.scene != Some(key) {
            let mut m = hdr(1, prims.len() as u32, 0);
            m.extend_from_slice(&floor_gid.to_le_bytes());
            m.extend_from_slice(&ceil_gid.to_le_bytes());
            m.extend_from_slice(&ceil_h.to_le_bytes());
            for p in prims { for v in p { m.extend_from_slice(&v.to_le_bytes()); } }
            c.s.write_all(&m)?;
            rd_hdr(&mut c.s)?;
            c.scene = Some(key);
        }
        let mut m = hdr(2, n as u32, if want_normal { 1 } else { 0 });
        for v in [origin[0], origin[1], origin[2], max_range] { m.extend_from_slice(&v.to_le_bytes()); }
        m.reserve(n * 12);
        for v in dirs { m.extend_from_slice(&(*v as f32).to_le_bytes()); }
        c.s.write_all(&m)?;
        let (_, us) = rd_hdr(&mut c.s)?;
        let mut b = vec![0u8; n * 4];
        c.s.read_exact(&mut b)?;
        let dist: Vec<f64> = b.chunks_exact(4).map(|x| f32::from_le_bytes(x.try_into().unwrap()) as f64).collect();
        c.s.read_exact(&mut b)?;
        let gid: Vec<i32> = b.chunks_exact(4).map(|x| i32::from_le_bytes(x.try_into().unwrap())).collect();
        let nrm = if want_normal {
            let mut b3 = vec![0u8; n * 12];
            c.s.read_exact(&mut b3)?;
            Some(b3.chunks_exact(4).map(|x| f32::from_le_bytes(x.try_into().unwrap()) as f64).collect())
        } else { None };
        Ok((dist, gid, nrm, us))
    })();
    match res {
        Ok((d, g, nr, us)) => { st.calls += 1; st.gpu_us += us as u64; st.wall_us += t0.elapsed().as_micros() as u64; Some((d, g, nr)) }
        Err(_) => { st.errors += 1; st.conn = None; st.retry_at = Some(Instant::now() + Duration::from_secs(5)); None }
    }
}

pub fn info() -> serde_json::Value {
    let st = ST.lock().unwrap();
    let a = addr();
    serde_json::json!({"addr": a.map(|x| x.to_string()), "connected": st.conn.is_some(), "device": st.device, "calls": st.calls, "errors": st.errors,
        "min_rays": min_rays(), "avg_ms": if st.calls > 0 { Some(st.wall_us as f64 / st.calls as f64 / 1000.0) } else { None },
        "avg_gpu_ms": if st.calls > 0 { Some(st.gpu_us as f64 / st.calls as f64 / 1000.0) } else { None }})
}
