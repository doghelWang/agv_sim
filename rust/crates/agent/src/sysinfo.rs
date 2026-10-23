//! Host information (Linux / Android chroot): CPU, memory, temperature, model, IPs, listening ports, disk.
use serde_json::{json, Value};
use std::sync::Mutex;
use std::time::Instant;

fn read(p: &str) -> Option<String> { std::fs::read_to_string(p).ok() }
pub fn is_android() -> bool { std::env::var("ANDROID_ROOT").is_ok() || std::env::var("TERMUX_VERSION").is_ok() || std::path::Path::new("/system/build.prop").exists() || std::env::var("AGV_PLATFORM").map(|v| v == "android").unwrap_or(false) }
fn prop(name: &str) -> Option<String> {
    for p in ["/system/build.prop", "/vendor/build.prop", "/system/vendor/build.prop", "/odm/etc/build.prop"] {
        if let Some(t) = read(p) { for l in t.lines() { if let Some(v) = l.strip_prefix(&format!("{name}=")) { return Some(v.trim().to_string()); } } }
    }
    None
}

pub fn model() -> String {
    if let Ok(m) = std::env::var("AGV_DEVICE_MODEL") { if !m.is_empty() { return m; } }
    if let Some(t) = read("/proc/device-tree/model") { return t.trim_matches(|c| c == '\0' || c == '\n' || c == ' ').to_string(); }
    if is_android() {
        let m = prop("ro.product.model").or_else(|| prop("ro.product.system.model")).or_else(|| prop("ro.product.vendor.model"));
        let soc = prop("ro.soc.model").or_else(|| prop("ro.board.platform")).unwrap_or_default();
        let rel = prop("ro.build.version.release").unwrap_or_default();
        let parts = [m.unwrap_or("Android".into()), if soc.is_empty() { String::new() } else { format!("({soc})") }, if rel.is_empty() { String::new() } else { format!("Android {rel}") }];
        return parts.iter().filter(|x| !x.is_empty()).cloned().collect::<Vec<_>>().join(" ");
    }
    for l in read("/proc/cpuinfo").unwrap_or_default().lines() { if l.starts_with("model name") || l.starts_with("Hardware") { return l.split_once(':').map(|x| x.1.trim().to_string()).unwrap_or_default(); } }
    uname_machine()
}
pub fn uname_machine() -> String { unsafe { let mut u: libc::utsname = std::mem::zeroed(); libc::uname(&mut u); std::ffi::CStr::from_ptr(u.machine.as_ptr()).to_string_lossy().to_string() } }
pub fn norm_arch(m: &str) -> String { match m { "aarch64" => "arm64".into(), "x86_64" => "amd64".into(), "armv7l" => "arm".into(), o => o.into() } }
pub fn hostname() -> String { let mut b = [0u8; 256]; unsafe { libc::gethostname(b.as_mut_ptr() as *mut libc::c_char, 255); } let n = b.iter().position(|&c| c == 0).unwrap_or(0); String::from_utf8_lossy(&b[..n]).to_string() }
pub fn os_name() -> String {
    for l in read("/etc/os-release").unwrap_or_default().lines() { if let Some(v) = l.strip_prefix("PRETTY_NAME=") { return v.trim().trim_matches('"').to_string(); } }
    "Linux".into()
}

static CPU_PREV: Mutex<Option<(u64, u64)>> = Mutex::new(None);
pub fn cpu_percent() -> f64 {
    let Some(t) = read("/proc/stat") else { return 0.0 };
    let v: Vec<u64> = t.lines().next().unwrap_or("").split_whitespace().skip(1).filter_map(|x| x.parse().ok()).collect();
    if v.len() < 5 { return 0.0; }
    let (idle, total) = (v[3] + v[4], v.iter().sum::<u64>());
    let mut p = CPU_PREV.lock().unwrap();
    let r = match *p { Some((pi, pt)) if total > pt => { let (di, dt) = (idle - pi.min(idle), total - pt); ((100.0 * (1.0 - di as f64 / dt as f64)) * 10.0).round() / 10.0 } _ => 0.0 };
    *p = Some((idle, total));
    r.max(0.0)
}
pub fn mem() -> Value {
    let mut d = std::collections::HashMap::new();
    for l in read("/proc/meminfo").unwrap_or_default().lines() { if let Some((k, v)) = l.split_once(':') { if let Some(n) = v.split_whitespace().next().and_then(|x| x.parse::<u64>().ok()) { d.insert(k.to_string(), n * 1024); } } }
    let t = d.get("MemTotal").cloned().unwrap_or(0);
    if t == 0 { return json!({}); }
    let a = d.get("MemAvailable").or_else(|| d.get("MemFree")).cloned().unwrap_or(0);
    json!({"total": t, "used": t - a.min(t), "percent": ((100.0 * (t - a.min(t)) as f64 / t as f64) * 10.0).round() / 10.0})
}

static TEMP: Mutex<(Option<Vec<usize>>, Option<Instant>, Option<f64>)> = Mutex::new((None, None, None));
pub fn temp_c() -> Option<f64> {
    let ttl: f64 = std::env::var("AGV_TEMP_TTL_S").ok().and_then(|v| v.parse().ok()).unwrap_or(if is_android() { 10.0 } else { 2.0 });
    let mut st = TEMP.lock().unwrap();
    if let Some(t) = st.1 { if t.elapsed().as_secs_f64() < ttl { return st.2; } }
    st.1 = Some(Instant::now());
    if st.0.is_none() {
        let mut zones = vec![];
        for i in 0..80 {
            match read(&format!("/sys/class/thermal/thermal_zone{i}/type")) {
                None => { if i > 10 && zones.is_empty() { break; } }
                Some(t) => { let t = t.trim().to_lowercase(); if (i == 0 && !is_android()) || ["cpu", "soc", "tsens", "skin", "xo-therm", "cluster"].iter().any(|k| t.contains(k)) { zones.push(i); if i == 0 && !is_android() { break; } } }
            }
        }
        zones.truncate(12);
        st.0 = Some(zones);
    }
    let mut best: Option<f64> = None;
    let mut ret: Option<Option<f64>> = None;
    for &i in st.0.as_ref().unwrap() {
        let Some(v) = read(&format!("/sys/class/thermal/thermal_zone{i}/temp")) else { continue };
        let Ok(n) = v.trim().parse::<i64>() else { continue };
        let t = if n.abs() > 1000 { n as f64 / 1000.0 } else { n as f64 };
        if i == 0 && !is_android() { ret = Some(Some((t * 10.0).round() / 10.0)); break; }
        if t > 0.0 && t < 130.0 { best = Some(best.unwrap_or(0.0).max(t)); }
    }
    let v = match ret { Some(r) => r, None => match best { Some(b) => Some((b * 10.0).round() / 10.0), None => read("/sys/class/power_supply/battery/temp").and_then(|b| b.trim().parse::<i64>().ok()).map(|b| (b as f64 / 10.0 * 10.0).round() / 10.0) } };
    st.2 = v;
    v
}

pub fn ips() -> Vec<String> {
    let mut out = vec![];
    unsafe {
        let mut ifa: *mut libc::ifaddrs = std::ptr::null_mut();
        if libc::getifaddrs(&mut ifa) == 0 {
            let mut p = ifa;
            while !p.is_null() {
                let a = &*p;
                if !a.ifa_addr.is_null() && (*a.ifa_addr).sa_family as i32 == libc::AF_INET {
                    let name = std::ffi::CStr::from_ptr(a.ifa_name).to_string_lossy().to_string();
                    let sin = &*(a.ifa_addr as *const libc::sockaddr_in);
                    let ip = std::net::Ipv4Addr::from(u32::from_be(sin.sin_addr.s_addr));
                    if name != "lo" && !["docker", "veth", "br-", "virbr", "cni", "flannel"].iter().any(|x| name.starts_with(x)) && !ip.is_loopback() { out.push(ip.to_string()); }
                }
                p = a.ifa_next;
            }
            libc::freeifaddrs(ifa);
        }
    }
    out
}

pub fn listening_ports(lo: u16, hi: u16) -> Vec<u16> {
    let mut s = std::collections::BTreeSet::new();
    for f in ["/proc/net/tcp", "/proc/net/tcp6"] {
        for l in read(f).unwrap_or_default().lines().skip(1) {
            let c: Vec<&str> = l.split_whitespace().collect();
            if c.len() > 3 && c[3] == "0A" { if let Some(p) = c[1].rsplit(':').next().and_then(|p| u16::from_str_radix(p, 16).ok()) { if p >= lo && p <= hi { s.insert(p); } } }
        }
    }
    s.into_iter().collect()
}
pub fn port_free(port: u16) -> bool { std::net::TcpListener::bind(("0.0.0.0", port)).is_ok() }

pub fn disk(path: &str) -> Value {
    let c = std::ffi::CString::new(path).unwrap();
    unsafe {
        let mut s: libc::statvfs = std::mem::zeroed();
        if libc::statvfs(c.as_ptr(), &mut s) != 0 { return json!({}); }
        json!({"total": s.f_blocks as u64 * s.f_frsize as u64, "free": s.f_bavail as u64 * s.f_frsize as u64})
    }
}
pub fn loadavg() -> Value { let mut l = [0f64; 3]; unsafe { if libc::getloadavg(l.as_mut_ptr(), 3) < 0 { return json!([]); } } json!(l) }
