//! Process runtime: runs the sim / nav entry scripts of an instance as detached process groups.
use serde_json::{json, Map, Value};
use std::collections::HashMap;
use std::io::{Read, Seek, SeekFrom};
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

pub struct Proc { pub pid: i32, pub role: String, pub image: String, pub instance: String, pub started: String, pub exit: Arc<Mutex<Option<i32>>>, pub owned: bool }

pub struct ProcessRuntime { pub code_dir: PathBuf, pub log_dir: PathBuf, procs: Mutex<HashMap<String, Proc>>, table: PathBuf }

fn alive(pid: i32) -> bool {
    if pid <= 0 { return false; }
    match std::fs::read_to_string(format!("/proc/{pid}/stat")) {
        Ok(s) => s.rsplit(')').next().and_then(|r| r.split_whitespace().next()).map(|st| st != "Z" && st != "X").unwrap_or(false),
        Err(_) => false,
    }
}
fn local_ts() -> String {
    unsafe { let t = libc::time(std::ptr::null_mut()); let mut tm: libc::tm = std::mem::zeroed(); libc::localtime_r(&t, &mut tm);
        format!("{:04}-{:02}-{:02}T{:02}:{:02}:{:02}", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec) }
}

impl ProcessRuntime {
    pub fn new(code_dir: &str, data_dir: &str) -> ProcessRuntime {
        let log_dir = Path::new(data_dir).join("logs");
        let _ = std::fs::create_dir_all(&log_dir);
        let rt = ProcessRuntime { code_dir: code_dir.into(), log_dir, procs: Mutex::new(HashMap::new()), table: Path::new(data_dir).join("procs.json") };
        rt.adopt();
        rt
    }
    /// Re-attach to instance processes started by a previous agent run (pid table on disk).
    fn adopt(&self) {
        let Some(t) = std::fs::read_to_string(&self.table).ok().and_then(|s| serde_json::from_str::<Value>(&s).ok()) else { return };
        let mut p = self.procs.lock().unwrap();
        for (name, e) in t.as_object().cloned().unwrap_or_default() {
            let pid = e.get("pid").and_then(|v| v.as_i64()).unwrap_or(0) as i32;
            if alive(pid) {
                p.insert(name, Proc { pid, role: e["role"].as_str().unwrap_or("").into(), image: e["image"].as_str().unwrap_or("").into(), instance: e["instance"].as_str().unwrap_or("").into(), started: e["started"].as_str().unwrap_or("").into(), exit: Arc::new(Mutex::new(None)), owned: false });
            }
        }
    }
    fn save(&self, p: &HashMap<String, Proc>) {
        let m: Map<String, Value> = p.iter().map(|(n, e)| (n.clone(), json!({"pid": e.pid, "role": e.role, "image": e.image, "instance": e.instance, "started": e.started}))).collect();
        let tmp = format!("{}.tmp", self.table.display());
        if std::fs::write(&tmp, serde_json::to_string_pretty(&Value::Object(m)).unwrap()).is_ok() { let _ = std::fs::rename(&tmp, &self.table); }
    }
    pub fn name(&self) -> &'static str { "process" }
    pub fn describe(&self) -> Value { json!({"runtime": "process", "code_dir": self.code_dir.to_string_lossy()}) }
    pub fn images(&self) -> Vec<Value> {
        let mut v = std::fs::metadata(self.code_dir.join("web_gateway.py")).ok().and_then(|m| m.modified().ok()).map(|t| {
            let s = t.duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0) as libc::time_t;
            unsafe { let mut tm: libc::tm = std::mem::zeroed(); libc::localtime_r(&s, &mut tm); format!("src-{:04}{:02}{:02}", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday) }
        }).unwrap_or_else(|| "src-unknown".into());
        if let Some(h) = std::fs::read_to_string(self.code_dir.join(".agv_version")).ok().and_then(|s| s.split_whitespace().next().map(|x| x.to_string())) { v = format!("src-{h}"); }
        ["sim", "nav"].iter().map(|k| json!({"ref": format!("process:{k}"), "id": format!("process:{k}"), "kind": k, "size": 0, "created": 0, "version": v, "api": "v1", "labels": {}})).collect()
    }
    pub fn image(&self, r: &str) -> Option<Value> { self.images().into_iter().find(|i| i["ref"] == r) }
    fn entry_script(&self, role: &str) -> PathBuf {
        let d = self.code_dir.join("docker");
        if role == "sim" {
            for n in ["sim_entrypoint_rs2.sh", "sim_entrypoint_rs.sh"] { if d.join(n).exists() && std::env::var("AGENT_SIM_PYTHON").is_err() { return d.join(n); } }
            d.join("sim_entrypoint.sh")
        } else { d.join("nav_entrypoint.sh") }
    }
    pub fn run(&self, spec: &Value) -> Result<Value, String> {
        let name = spec["name"].as_str().ok_or("缺少 name")?.to_string();
        self.remove(&name);
        let image = spec["image"].as_str().unwrap_or("");
        let role = spec.get("role").and_then(|r| r.as_str()).filter(|r| !r.is_empty()).map(|r| r.to_string()).unwrap_or(if image.ends_with("sim") { "sim".into() } else { "nav".into() });
        let script = self.entry_script(&role);
        let logp = self.log_dir.join(format!("{name}.log"));
        let log = std::fs::OpenOptions::new().create(true).append(true).open(&logp).map_err(|e| e.to_string())?;
        let log2 = log.try_clone().map_err(|e| e.to_string())?;
        let mut cmd = std::process::Command::new("bash");
        cmd.arg(&script).current_dir(&self.code_dir).stdin(std::process::Stdio::null()).stdout(log).stderr(log2);
        if let Some(e) = spec.get("env").and_then(|e| e.as_object()) { for (k, v) in e { cmd.env(k, match v { Value::String(s) => s.clone(), o => o.to_string() }); } }
        cmd.env("AGV_HOME", &self.code_dir).env("PYTHONUNBUFFERED", "1");
        if let Some(d) = spec.get("data_dir").and_then(|d| d.as_str()) { let _ = std::fs::create_dir_all(d); cmd.env("AGV_DATA", d); }
        if role == "nav" && std::env::var("NAV_USE_ROS").is_err() && spec.pointer("/env/NAV_USE_ROS").is_none() { cmd.env("NAV_USE_ROS", if Path::new("/opt/ros").exists() { "1" } else { "0" }); }
        unsafe { cmd.pre_exec(|| { libc::setsid(); Ok(()) }); }
        let mut child = cmd.spawn().map_err(|e| format!("启动失败: {e}"))?;
        let pid = child.id() as i32;
        let exit = Arc::new(Mutex::new(None));
        let ex2 = exit.clone();
        std::thread::spawn(move || { let c = child.wait().ok().and_then(|s| s.code().or(Some(-1))); *ex2.lock().unwrap() = Some(c.unwrap_or(-1)); });
        let mut p = self.procs.lock().unwrap();
        p.insert(name.clone(), Proc { pid, role, image: image.into(), instance: spec.get("instance").and_then(|i| i.as_str()).unwrap_or("").into(), started: local_ts(), exit, owned: true });
        self.save(&p);
        Ok(json!({"name": name, "id": pid.to_string()}))
    }
    fn is_running(e: &Proc) -> bool { if e.owned { e.exit.lock().unwrap().is_none() } else { alive(e.pid) } }
    pub fn stop(&self, name: &str) {
        let (pid, owned, exit) = { let p = self.procs.lock().unwrap(); match p.get(name) { Some(e) if Self::is_running(e) => (e.pid, e.owned, e.exit.clone()), _ => return } };
        let _ = owned;
        let gone = |exit: &Arc<Mutex<Option<i32>>>| exit.lock().unwrap().is_some() || !alive(pid);
        unsafe { libc::kill(-pid, libc::SIGTERM); }
        for _ in 0..80 { if gone(&exit) { return; } std::thread::sleep(std::time::Duration::from_millis(100)); }
        unsafe { libc::kill(-pid, libc::SIGKILL); }
        for _ in 0..30 { if gone(&exit) { return; } std::thread::sleep(std::time::Duration::from_millis(100)); }
    }
    pub fn remove(&self, name: &str) {
        self.stop(name);
        let mut p = self.procs.lock().unwrap();
        if p.remove(name).is_some() { self.save(&p); }
    }
    pub fn status(&self, name: &str) -> Option<Value> {
        let p = self.procs.lock().unwrap();
        let e = p.get(name)?;
        let run = Self::is_running(e);
        let code = if run { Value::Null } else { e.exit.lock().unwrap().map(|c| json!(c)).unwrap_or(Value::Null) };
        Some(json!({"name": name, "state": if run { "running" } else { "exited" }, "running": run, "exit_code": code, "started_at": e.started, "restarts": 0, "image": e.image, "health": null}))
    }
    pub fn list(&self) -> Vec<Value> {
        self.procs.lock().unwrap().iter().map(|(n, e)| json!({"name": n, "state": if Self::is_running(e) { "running" } else { "exited" }, "status": "", "image": e.image, "role": e.role, "instance": e.instance})).collect()
    }
    pub fn logs(&self, name: &str, tail: usize) -> String {
        let Ok(mut f) = std::fs::File::open(self.log_dir.join(format!("{name}.log"))) else { return String::new() };
        let size = f.seek(SeekFrom::End(0)).unwrap_or(0);
        let _ = f.seek(SeekFrom::Start(size.saturating_sub(256 * 1024)));
        let mut b = vec![];
        let _ = f.read_to_end(&mut b);
        let s = String::from_utf8_lossy(&b).to_string();
        let lines: Vec<&str> = s.lines().collect();
        lines[lines.len().saturating_sub(tail)..].join("\n")
    }
}
