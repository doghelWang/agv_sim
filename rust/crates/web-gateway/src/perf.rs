//! Host / process resource monitor (port of SystemPerformanceMonitor; reads /proc directly, no psutil)
use crate::util::*;
use serde_json::{json, Value};
use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};

fn rd(p: &str) -> Option<String> { std::fs::read_to_string(p).ok() }
fn hz() -> f64 { let v = unsafe { libc::sysconf(libc::_SC_CLK_TCK) }; if v > 0 { v as f64 } else { 100.0 } }
fn ncpu() -> usize { std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1) }

fn prop(name: &str) -> Option<String> {
    for p in ["/system/build.prop", "/vendor/build.prop", "/system/vendor/build.prop", "/odm/etc/build.prop"] {
        if let Some(t) = rd(p) { for l in t.lines() { if let Some(v) = l.strip_prefix(&format!("{name}=")) { return Some(v.trim().to_string()); } } }
    }
    None
}
fn model() -> String {
    if let Ok(m) = std::env::var("AGV_DEVICE_MODEL") { if !m.is_empty() { return m; } }
    if let Some(t) = rd("/proc/device-tree/model") { return t.trim_matches(|c| c == '\0' || c == '\n' || c == ' ').to_string(); }
    if is_android() {
        let m = prop("ro.product.model").or_else(|| prop("ro.product.system.model")).or_else(|| prop("ro.product.vendor.model")).unwrap_or("Android".into());
        let soc = prop("ro.soc.model").or_else(|| prop("ro.board.platform")).unwrap_or_default();
        let rel = prop("ro.build.version.release").unwrap_or_default();
        return [m, if soc.is_empty() { String::new() } else { format!("({soc})") }, if rel.is_empty() { String::new() } else { format!("Android {rel}") }].into_iter().filter(|x| !x.is_empty()).collect::<Vec<_>>().join(" ");
    }
    for l in rd("/proc/cpuinfo").unwrap_or_default().lines() { if l.starts_with("model name") || l.starts_with("Hardware") { if let Some((_, v)) = l.split_once(':') { return v.trim().to_string(); } } }
    std::env::consts::ARCH.to_string()
}
fn core_groups() -> Value {
    let mut g: std::collections::BTreeMap<i64, Vec<usize>> = Default::default();
    for i in 0..ncpu() { if let Some(v) = rd(&format!("/sys/devices/system/cpu/cpu{i}/cpufreq/cpuinfo_max_freq")).and_then(|f| f.trim().parse::<i64>().ok()) { g.entry(v).or_default().push(i); } }
    if g.is_empty() { return json!({}); }
    let ks: Vec<i64> = g.keys().rev().cloned().collect();
    let (big, little): (Vec<usize>, Vec<usize>) = if ks.len() > 1 { (ks[..ks.len() - 1].iter().flat_map(|k| g[k].clone()).collect(), g[ks.last().unwrap()].clone()) } else { (g[&ks[0]].clone(), vec![]) };
    let mut mm = serde_json::Map::new();
    for k in &ks { for c in &g[k] { mm.insert(c.to_string(), json!(k / 1000)); } }
    json!({"big": big, "little": little, "max_mhz": mm})
}
fn temp_c(zones: &mut Option<Vec<usize>>) -> Option<f64> {
    if zones.is_none() {
        let mut z = vec![];
        for i in 0..80 {
            let Some(typ) = rd(&format!("/sys/class/thermal/thermal_zone{i}/type")) else { if i > 10 && z.is_empty() { break; } continue };
            let typ = typ.trim().to_lowercase();
            if (i == 0 && !is_android()) || ["cpu", "soc", "tsens", "skin", "xo-therm", "cluster"].iter().any(|k| typ.contains(k)) { z.push(i); if i == 0 && !is_android() { break; } }
        }
        z.truncate(12);
        *zones = Some(z);
    }
    let mut best: Option<f64> = None;
    for i in zones.as_ref().unwrap() {
        let Some(v) = rd(&format!("/sys/class/thermal/thermal_zone{i}/temp")).and_then(|t| t.trim().parse::<i64>().ok()) else { continue };
        let t = if v.abs() > 1000 { v as f64 / 1000.0 } else { v as f64 };
        if *i == 0 && !is_android() { return Some(rnd(t, 1)); }
        if t > 0.0 && t < 130.0 { best = Some(best.unwrap_or(0.0).max(t)); }
    }
    if let Some(b) = best { return Some(rnd(b, 1)); }
    rd("/sys/class/power_supply/battery/temp").and_then(|b| b.trim().parse::<i64>().ok()).map(|v| rnd(v as f64 / 10.0, 1))
}

/// (utime+stime ticks, threads, rss_bytes, state)
fn proc_info(pid: i32) -> Option<(u64, u64, u64, char)> {
    let t = rd(&format!("/proc/{pid}/stat"))?;
    let f: Vec<&str> = t.rsplit_once(')')?.1.split_whitespace().collect();
    let ticks = f.get(11)?.parse::<u64>().ok()? + f.get(12)?.parse::<u64>().ok()?;
    let thr = f.get(17)?.parse().ok()?;
    let rss = rd(&format!("/proc/{pid}/statm")).and_then(|s| s.split_whitespace().nth(1).and_then(|x| x.parse::<u64>().ok())).unwrap_or(0) * 4096;
    Some((ticks, thr, rss, f.first()?.chars().next()?))
}
fn all_ticks() -> u64 {
    let mut s = 0;
    if let Ok(d) = std::fs::read_dir("/proc") { for e in d.flatten() { if let Ok(pid) = e.file_name().to_string_lossy().parse::<i32>() { if let Some(t) = rd(&format!("/proc/{pid}/stat")) { if let Some(r) = t.rsplit_once(')') { let f: Vec<&str> = r.1.split_whitespace().collect(); if f.len() > 12 { s += f[11].parse::<u64>().unwrap_or(0) + f[12].parse::<u64>().unwrap_or(0); } } } } } }
    s
}
fn cpu_lines() -> Vec<(u64, u64)> {
    rd("/proc/stat").unwrap_or_default().lines().filter(|l| l.starts_with("cpu")).map(|l| {
        let v: Vec<u64> = l.split_whitespace().skip(1).filter_map(|x| x.parse().ok()).collect();
        let tot: u64 = v.iter().sum(); let idle = v.get(3).copied().unwrap_or(0) + v.get(4).copied().unwrap_or(0);
        (tot, idle)
    }).collect()
}
fn state_name(c: char) -> &'static str { match c { 'R' => "running", 'S' => "sleeping", 'D' => "disk-sleep", 'Z' => "zombie", 'T' | 't' => "stopped", 'I' => "idle", _ => "sleeping" } }

const TARGETS: [(&str, &str, &[&str], &str); 5] = [
    ("sim_server", "仿真进程 sim_server (REST :8100)", &["sim_server.api", "sim-server"], "物理步进 (MuJoCo)、激光/相机/IO/碰撞数据产生"),
    ("nav_runtime", "执行进程 nav_runtime (ROS2+Nav2, REST :8102)", &["nav_runtime.main"], "Nav2/拓扑导引执行，cmd_vel 与任务状态回馈仿真进程"),
    ("web_gateway", "Web 网关 web_gateway (:8101)", &["web_gateway.py", "web-gateway"], "聚合两进程 REST，前端/事件/回放"),
    ("map_server", "Nav2 栅格地图服务器 (map_server)", &["nav2_map_server/map_server"], "ROS2 占据栅格地图发布与代价地图广播"),
    ("lifecycle_manager", "Nav2 生命周期管理 (lifecycle_manager)", &["nav2_lifecycle_manager/lifecycle_manager"], "ROS2 节点健康监控与生命周期状态切换"),
];

pub struct PerfMonitor { pub latest: Mutex<Value>, history: Mutex<VecDeque<Value>> }

impl PerfMonitor {
    pub fn start(is_idle: Arc<dyn Fn() -> bool + Send + Sync>) -> Arc<PerfMonitor> {
        let n = ncpu();
        let latest = json!({"timestamp": now(), "host": {"model": "", "cpu_count": n, "cpu_total_percent": 0.0, "cpu_per_core": vec![0.0; n], "cpu_temp_c": 50.0, "cpu_freq_mhz": 1800.0, "memory_total_mb": 4049.0,
            "memory_used_mb": 800.0, "memory_free_mb": 3249.0, "memory_percent": 19.8, "load_avg": [0.0, 0.0, 0.0]}, "ros_simulation_total": {"combined_cpu_percent": 0.0, "combined_rss_mb": 0.0, "combined_ram_percent": 0.0}, "processes": []});
        let pm = Arc::new(PerfMonitor { latest: Mutex::new(latest), history: Mutex::new(VecDeque::new()) });
        let p2 = pm.clone();
        std::thread::Builder::new().name("perf".into()).spawn(move || p2.run(is_idle)).ok();
        pm
    }
    pub fn latest(&self) -> Value { self.latest.lock().unwrap_or_else(|e| e.into_inner()).clone() }
    pub fn get_stats(&self) -> Value {
        let mut d = self.latest();
        d["history"] = Value::Array(self.history.lock().unwrap_or_else(|e| e.into_inner()).iter().cloned().collect());
        d
    }
    pub fn set_bullet(&self, m: Value) { self.latest.lock().unwrap_or_else(|e| e.into_inner())["bullet_simulation"] = m; }

    fn scan(my: i32) -> HashMap<&'static str, i32> {
        let mut found = HashMap::new();
        if let Ok(d) = std::fs::read_dir("/proc") {
            for e in d.flatten() {
                let Ok(pid) = e.file_name().to_string_lossy().parse::<i32>() else { continue };
                if pid == my { continue; }
                let Some(c) = std::fs::read(format!("/proc/{pid}/cmdline")).ok() else { continue };
                let args: Vec<String> = c.split(|b| *b == 0).filter(|a| !a.is_empty()).map(|a| String::from_utf8_lossy(a).to_string()).collect();
                if args.iter().any(|a| a == "-c") { continue; }
                let cmd = args.join(" ");
                for (k, _, pats, _) in TARGETS.iter() { if !found.contains_key(k) && pats.iter().any(|p| cmd.contains(p)) { found.insert(*k, pid); break; } }
            }
        }
        found
    }

    fn run(&self, is_idle: Arc<dyn Fn() -> bool + Send + Sync>) {
        let my = std::process::id() as i32;
        let (mdl, groups) = (model(), core_groups());
        let hzv = hz();
        let mut zones: Option<Vec<usize>> = None;
        let mut tracked = Self::scan(my);
        let mut prev_proc: HashMap<i32, (f64, u64)> = HashMap::new();
        let mut prev_stat = cpu_lines();
        let mut prev_est = (now(), all_ticks());
        let mut scan_t = 0.0;
        let stat_ok = prev_stat.first().map(|c| c.0 > 0).unwrap_or(false);
        std::thread::sleep(std::time::Duration::from_millis(500));
        let period = env_f("GW_MONITOR_PERIOD_S", if is_android() { 2.0 } else { 1.0 });
        loop {
            let t_loop = now();
            let cur = cpu_lines();
            let pct = |a: &(u64, u64), b: &(u64, u64)| { let dt = b.0.saturating_sub(a.0) as f64; if dt <= 0.0 { 0.0 } else { rnd(100.0 * (dt - b.1.saturating_sub(a.1) as f64) / dt, 1) } };
            let mut cpu_tot = if stat_ok && !cur.is_empty() && !prev_stat.is_empty() { pct(&prev_stat[0], &cur[0]) } else { 0.0 };
            let per_core: Vec<f64> = if stat_ok { (1..cur.len().min(prev_stat.len())).map(|i| pct(&prev_stat[i], &cur[i])).collect() } else { vec![0.0; ncpu()] };
            prev_stat = cur;
            let (tn, tk) = (now(), all_ticks());
            let est = if tn - prev_est.0 > 0.0 { rnd((100.0 * (tk.saturating_sub(prev_est.1)) as f64 / hzv / (tn - prev_est.0) / ncpu() as f64).min(100.0), 1) } else { 0.0 };
            prev_est = (tn, tk);
            let cpu_estimated = cpu_tot <= 0.0 && est > 0.0;
            if cpu_estimated { cpu_tot = est; }
            let mi = rd("/proc/meminfo").unwrap_or_default();
            let kb = |k: &str| mi.lines().find(|l| l.starts_with(k)).and_then(|l| l.split_whitespace().nth(1)).and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0) / 1024.0;
            let (mtot, mavail) = (kb("MemTotal:"), kb("MemAvailable:"));
            let mused = (mtot - mavail).max(0.0);
            let mpct = if mtot > 0.0 { rnd(mused / mtot * 100.0, 1) } else { 0.0 };
            let temp = temp_c(&mut zones);
            let freq = rd("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq").and_then(|v| v.trim().parse::<f64>().ok()).map(|v| rnd(v / 1000.0, 1));
            let la: Vec<f64> = rd("/proc/loadavg").map(|s| s.split_whitespace().take(3).filter_map(|x| x.parse::<f64>().ok()).map(|x| rnd(x, 2)).collect()).unwrap_or(vec![0.0; 3]);
            if tracked.len() < 3 && now() - scan_t > 10.0 { scan_t = now(); tracked = Self::scan(my); }
            let (mut sim_cpu, mut tele_cpu, mut map_cpu, mut rss_tot) = (0.0, 0.0, 0.0, 0.0);
            let mut procs = vec![];
            for (k, name, _, role) in TARGETS.iter() {
                let Some(&pid) = tracked.get(k) else { continue };
                let Some((ticks, thr, rss, st)) = proc_info(pid) else { tracked.remove(k); continue };
                if st == 'Z' { tracked.remove(k); continue; }
                let t = now();
                let cpu_p = match prev_proc.get(&pid) { Some((pt, pk)) if t > *pt => rnd(100.0 * ticks.saturating_sub(*pk) as f64 / hzv / (t - pt), 1), _ => 0.0 };
                prev_proc.insert(pid, (t, ticks));
                let rss_mb = rnd(rss as f64 / 1048576.0, 1);
                rss_tot += rss_mb;
                match *k { "sim_server" => sim_cpu = cpu_p, "web_gateway" => tele_cpu = cpu_p, "map_server" | "lifecycle_manager" => map_cpu += cpu_p, _ => {} }
                procs.push(json!({"key": k, "name": name, "pid": pid, "cpu_percent": cpu_p, "memory_mb": rss_mb, "memory_percent": rnd(rss_mb / mtot.max(1.0) * 100.0, 1), "threads": thr, "status": state_name(st), "role": role}));
            }
            let t = now();
            let point = json!({"time": fmt_local(t, "%H:%M:%S"), "timestamp": rnd(t, 2), "cpu_total": cpu_tot, "mem_used_mb": rnd(mused, 1), "mem_percent": mpct, "sim_cpu": sim_cpu, "teleop_cpu": tele_cpu, "map_cpu": map_cpu, "temp_c": temp.unwrap_or(50.0)});
            let mut snap = json!({"timestamp": t, "host": {"model": mdl, "cpu_count": ncpu(), "cpu_estimated": cpu_estimated, "core_groups": groups, "cpu_total_percent": cpu_tot, "cpu_per_core": per_core,
                "cpu_temp_c": temp, "cpu_freq_mhz": freq, "memory_total_mb": rnd(mtot, 1), "memory_used_mb": rnd(mused, 1), "memory_free_mb": rnd(mavail, 1), "memory_percent": mpct, "load_avg": la},
                "ros_simulation_total": {"combined_cpu_percent": rnd(sim_cpu + tele_cpu + map_cpu, 1), "combined_rss_mb": rnd(rss_tot, 1), "combined_ram_percent": rnd(rss_tot / mtot.max(1.0) * 100.0, 1)}, "processes": procs});
            {
                let mut l = self.latest.lock().unwrap_or_else(|e| e.into_inner());
                if let Some(b) = l.get("bullet_simulation") { snap["bullet_simulation"] = b.clone(); }
                *l = snap;
            }
            { let mut h = self.history.lock().unwrap_or_else(|e| e.into_inner()); h.push_back(point); if h.len() > 300 { h.pop_front(); } }
            let wait = if is_idle() { env_f("GW_MONITOR_IDLE_S", 10.0) } else { period };
            std::thread::sleep(std::time::Duration::from_secs_f64((wait - (now() - t_loop)).max(0.1)));
        }
    }
}
