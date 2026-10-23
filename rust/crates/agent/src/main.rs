mod runtime;
mod sysinfo;

use rest::{ok, ApiError, HResult, Request, Response, Server};
use runtime::ProcessRuntime;
use serde_json::{json, Map, Value};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Instant;

const AGENT_VERSION: &str = "1.0";
fn log(m: &str) { println!("[agent] {m}"); use std::io::Write; let _ = std::io::stdout().flush(); }
fn now() -> f64 { std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0) }
fn truthy(v: Option<&Value>) -> bool { match v { None | Some(Value::Null) => false, Some(Value::Bool(b)) => *b, Some(Value::String(s)) => !s.is_empty(), Some(Value::Number(n)) => n.as_f64() != Some(0.0), Some(Value::Array(a)) => !a.is_empty(), Some(Value::Object(o)) => !o.is_empty() } }
fn ct_eq(a: &str, b: &str) -> bool { a.len() == b.len() && a.bytes().zip(b.bytes()).fold(0u8, |x, (p, q)| x | (p ^ q)) == 0 }
fn rand_hex(n: usize) -> String { use std::io::Read; let mut b = vec![0u8; n]; if let Ok(mut f) = std::fs::File::open("/dev/urandom") { let _ = f.read_exact(&mut b); } b.iter().map(|x| format!("{x:02x}")).collect() }

struct Agent {
    hub: String, port: u16, name: String, data: String, port_lo: u16, port_hi: u16, kind: String, advertise: String,
    rt: ProcessRuntime, state_path: String, state: Mutex<Value>, jobs: Mutex<HashMap<String, Value>>,
    hub_online: Mutex<bool>, last_error: Mutex<String>, role: Mutex<String>, ports_cache: Mutex<Option<(Instant, Vec<u16>)>>, alloc: Mutex<()>,
}

impl Agent {
    fn new(root: &str) -> Agent {
        let env = |k: &str, d: &str| std::env::var(k).ok().filter(|v| !v.is_empty()).unwrap_or(d.to_string());
        let data = { let d = env("AGENT_DATA", "~/.agv-agent"); if let Some(r) = d.strip_prefix("~/") { format!("{}/{}", std::env::var("HOME").unwrap_or("/root".into()), r) } else { d } };
        let _ = std::fs::create_dir_all(&data);
        let (lo, hi) = env("AGENT_PORT_RANGE", "8100-8199").split_once('-').map(|(a, b)| (a.parse().unwrap_or(8100), b.parse().unwrap_or(8199))).unwrap_or((8100, 8199));
        let state_path = format!("{data}/agent.json");
        let state = std::fs::read_to_string(&state_path).ok().and_then(|s| serde_json::from_str::<Value>(&s).ok()).unwrap_or(json!({"reservations": {}}));
        let role = state.get("role").and_then(|r| r.as_str()).filter(|r| !r.is_empty()).map(|r| r.to_string()).unwrap_or(env("AGENT_ROLE", "full"));
        Agent { hub: env("HUB_API", "").trim_end_matches('/').to_string(), port: env("AGENT_PORT", "8070").parse().unwrap_or(8070), name: std::env::var("AGENT_NAME").ok().filter(|v| !v.is_empty()).unwrap_or_else(sysinfo::hostname),
            rt: ProcessRuntime::new(root, &data), data, port_lo: lo, port_hi: hi, kind: env("AGENT_KIND", "hybrid"), advertise: env("AGENT_HOST", ""), state_path, state: Mutex::new(state), jobs: Mutex::new(HashMap::new()),
            hub_online: Mutex::new(false), last_error: Mutex::new(String::new()), role: Mutex::new(role), ports_cache: Mutex::new(None), alloc: Mutex::new(()) }
    }
    fn save_state(&self) {
        let s = self.state.lock().unwrap();
        let tmp = format!("{}.tmp", self.state_path);
        if std::fs::write(&tmp, serde_json::to_string_pretty(&*s).unwrap()).is_ok() { let _ = std::fs::rename(&tmp, &self.state_path); }
    }
    fn node_key(&self) -> String { self.state.lock().unwrap().get("node_key").and_then(|k| k.as_str()).unwrap_or("").to_string() }
    fn check(&self, q: &Request) -> Result<(), ApiError> {
        let k = self.node_key();
        if k.is_empty() || !ct_eq(q.header("x-node-key").unwrap_or(""), &k) { return Err(ApiError::coded(401, "节点密钥无效", "unauthorized")); }
        Ok(())
    }
    fn full(&self, q: &Request) -> Result<(), ApiError> {
        self.check(q)?;
        if *self.role.lock().unwrap() == "monitor" { return Err(ApiError::coded(409, "该节点只安装了基础监测，部署时由平台启用运行环境", "monitor_only")); }
        Ok(())
    }
    fn info(&self, full: bool) -> Value {
        { let mut e = self.last_error.lock().unwrap(); if e.starts_with("镜像列表") || e.starts_with("容器列表") { e.clear(); } }
        let mut d = json!({"name": self.name, "hostname": sysinfo::hostname(), "arch": sysinfo::norm_arch(&sysinfo::uname_machine()), "cpu_count": std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1),
            "cpu_percent": sysinfo::cpu_percent(), "mem": sysinfo::mem(), "temp_c": sysinfo::temp_c(), "load": sysinfo::loadavg(), "disk": sysinfo::disk(&self.data), "model": sysinfo::model(), "os": sysinfo::os_name(),
            "ips": sysinfo::ips(), "kind": self.kind, "runtime": self.rt.describe(), "agent_version": AGENT_VERSION, "role": *self.role.lock().unwrap(), "api_port": self.port, "advertise_host": self.advertise,
            "images": self.rt.images(), "containers": self.rt.list(), "port_range": [self.port_lo, self.port_hi], "reservations": self.state.lock().unwrap().get("reservations").cloned().unwrap_or(json!({})),
            "last_error": *self.last_error.lock().unwrap(), "time": now()});
        if full {
            let ttl: f64 = std::env::var("AGENT_PORTS_TTL_S").ok().and_then(|v| v.parse().ok()).unwrap_or(60.0);
            let mut c = self.ports_cache.lock().unwrap();
            if c.as_ref().map(|(t, _)| t.elapsed().as_secs_f64() > ttl).unwrap_or(true) { *c = Some((Instant::now(), sysinfo::listening_ports(self.port_lo, self.port_hi))); }
            d["ports_in_use"] = json!(c.as_ref().unwrap().1);
        }
        d
    }

    fn hub_call(&self, path: &str, body: &Value) -> Result<Value, (u16, String)> {
        let mut c = rest::Client::new(&self.hub, 8.0);
        let k = self.node_key();
        if !k.is_empty() { c = c.with_header("X-Node-Key", &k); }
        let r = c.request("POST", path, Some(body), Some(8.0), "application/json").map_err(|e| (0, e))?;
        let obj: Value = if r.body.is_empty() { json!({}) } else { serde_json::from_slice(&r.body).unwrap_or(json!({})) };
        if r.status >= 400 { return Err((r.status, obj.pointer("/error/message").and_then(|m| m.as_str()).map(|s| s.to_string()).unwrap_or(r.status.to_string()))); }
        Ok(obj)
    }
    fn register(&self) -> bool {
        let st = self.state.lock().unwrap().clone();
        let body = json!({"name": self.name, "api_port": self.port, "advertise_host": self.advertise, "kind": self.kind, "node_id": st.get("node_id").cloned().unwrap_or(Value::Null), "node_key": st.get("node_key").cloned().unwrap_or(Value::Null),
            "token": std::env::var("JOIN_TOKEN").unwrap_or_default(), "hub_url": self.hub, "info": self.info(false)});
        match self.hub_call("/api/hub/nodes/register", &body) {
            Err((_, e)) => { let m = format!("注册失败: {e}"); log(&m); *self.last_error.lock().unwrap() = m; false }
            Ok(r) => {
                { let mut s = self.state.lock().unwrap(); s["node_id"] = r["node_id"].clone(); s["node_key"] = r["node_key"].clone(); s["hub"] = json!(self.hub); }
                self.save_state();
                log(&format!("已注册到平台 {}: 节点 {} ({})", self.hub, r["node_id"].as_str().unwrap_or(""), r["host"].as_str().unwrap_or("")));
                self.last_error.lock().unwrap().clear();
                true
            }
        }
    }
    fn heartbeat_loop(self: Arc<Self>) {
        let mut registered = self.state.lock().unwrap().get("node_id").is_some() && self.register();
        loop {
            if !registered {
                registered = self.register();
                if !registered { *self.hub_online.lock().unwrap() = false; std::thread::sleep(std::time::Duration::from_secs(5)); continue; }
            }
            let nid = self.state.lock().unwrap().get("node_id").and_then(|n| n.as_str()).unwrap_or("").to_string();
            match self.hub_call(&format!("/api/hub/nodes/{nid}/heartbeat"), &self.info(true)) {
                Ok(_) => { *self.hub_online.lock().unwrap() = true; let mut e = self.last_error.lock().unwrap(); if e.starts_with("心跳失败") { e.clear(); } }
                Err((s, m)) if s != 0 => { *self.hub_online.lock().unwrap() = s < 500; if s == 401 || s == 404 { log(&format!("平台拒绝心跳 ({m})，重新注册")); registered = false; } }
                Err((_, m)) => { *self.hub_online.lock().unwrap() = false; *self.last_error.lock().unwrap() = format!("心跳失败: {m}"); }
            }
            std::thread::sleep(std::time::Duration::from_secs(5));
        }
    }

    fn allocate(&self, instance: &str, names: &[String]) -> Result<Value, ApiError> {
        let _g = self.alloc.lock().unwrap();
        let (cur, taken): (Map<String, Value>, Vec<i64>) = {
            let s = self.state.lock().unwrap();
            let res = s.get("reservations").and_then(|r| r.as_object()).cloned().unwrap_or_default();
            (res.get(instance).and_then(|c| c.as_object()).cloned().unwrap_or_default(),
             res.iter().filter(|(i, _)| i.as_str() != instance).flat_map(|(_, ps)| ps.as_object().cloned().unwrap_or_default().values().filter_map(|p| p.as_i64()).collect::<Vec<_>>()).collect())
        };
        if !cur.is_empty() && names.iter().all(|n| cur.contains_key(n)) { return Ok(Value::Object(cur)); }
        let mut out = Map::new();
        for n in names {
            let mut p = self.port_lo as i64;
            while p <= self.port_hi as i64 && (taken.contains(&p) || out.values().any(|v| v.as_i64() == Some(p)) || !sysinfo::port_free(p as u16)) { p += 1; }
            if p > self.port_hi as i64 { return Err(ApiError::coded(409, format!("端口范围 {}-{} 已用尽", self.port_lo, self.port_hi), "error")); }
            out.insert(n.clone(), json!(p));
        }
        { let mut s = self.state.lock().unwrap(); if !s.get("reservations").map(|r| r.is_object()).unwrap_or(false) { s["reservations"] = json!({}); } s["reservations"][instance] = Value::Object(out.clone()); }
        self.save_state();
        Ok(Value::Object(out))
    }
    fn release(&self, instance: &str) {
        { let mut s = self.state.lock().unwrap(); if let Some(r) = s.get_mut("reservations").and_then(|r| r.as_object_mut()) { r.remove(instance); } }
        self.save_state();
    }
    fn job(&self, kind: &str, r: &str, status: &str, msg: &str) -> Value {
        let id = rand_hex(5);
        let j = json!({"id": id, "kind": kind, "ref": r, "status": status, "bytes": 0, "total": 0, "message": msg, "started": now(), "ended": if status == "running" { Value::Null } else { json!(now()) }});
        let mut jobs = self.jobs.lock().unwrap();
        jobs.insert(id, j.clone());
        if jobs.len() > 50 { let mut ks: Vec<(String, f64)> = jobs.iter().map(|(k, v)| (k.clone(), v["started"].as_f64().unwrap_or(0.0))).collect(); ks.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap()); for (k, _) in ks.into_iter().take(10) { jobs.remove(&k); } }
        j
    }
}

fn build(ag: Arc<Agent>) -> Server {
    let mut api = Server::new("agent", ag.port, "节点代理 agv-agent", "在设备上分配端口、导入镜像、启停实例进程/容器、读取日志。除 health / openapi 外都要请求头 X-Node-Key");
    macro_rules! R { ($m:expr, $p:expr, $d:expr, $a:ident, $q:ident => $body:expr) => {{ let $a = ag.clone(); api.route($m, $p, $d, move |$q: &Request| -> HResult { let _ = &$a; let _ = $q; $body }); }}; }
    R!("GET", "/api/v1/health", "健康检查", a, q => ok(json!({"service": "agent", "ok": true, "name": a.name, "node_id": a.state.lock().unwrap().get("node_id").cloned().unwrap_or(Value::Null), "hub": a.hub, "hub_online": *a.hub_online.lock().unwrap(), "runtime": a.rt.name()})));
    R!("GET", "/api/v1/info", "节点信息", a, q => { a.check(q)?; ok(a.info(true)) });
    R!("POST", "/api/v1/ports/allocate", "分配端口 {instance, names[]}", a, q => {
        a.full(q)?;
        let b = q.json()?;
        let inst = b.get("instance").and_then(|i| i.as_str()).ok_or_else(|| ApiError::coded(400, "缺少 instance", "bad_request"))?.to_string();
        let names: Vec<String> = b.get("names").and_then(|n| n.as_array()).filter(|n| !n.is_empty()).map(|n| n.iter().filter_map(|x| x.as_str().map(|s| s.to_string())).collect()).unwrap_or(vec!["port".into()]);
        ok(json!({"ports": a.allocate(&inst, &names)?}))
    });
    R!("POST", "/api/v1/ports/release", "释放端口", a, q => { a.check(q)?; a.release(q.json()?.get("instance").and_then(|i| i.as_str()).unwrap_or("")); ok(json!({"ok": true})) });
    R!("POST", "/api/v1/images/ensure", "确保镜像存在 (缺失时从平台下载并导入)", a, q => {
        a.full(q)?;
        let b = q.json()?;
        let r = b.get("ref").and_then(|r| r.as_str()).unwrap_or("");
        if a.rt.image(r).is_some() { return ok(a.job("ensure", r, "done", "镜像已存在")); }
        Err(ApiError::coded(404, format!("本机没有镜像 {r}，process 运行时不支持导入镜像"), "error"))
    });
    R!("POST", "/api/v1/images/export", "导出本机镜像并上传到平台", a, q => { a.check(q)?; Err(ApiError::coded(501, "process 运行时不支持导出镜像", "not_implemented")) });
    R!("GET", "/api/v1/jobs/{jid}", "任务进度", a, q => { a.check(q)?; a.jobs.lock().unwrap().get(&q.params["jid"]).cloned().map(|j| Response::json(&j)).ok_or_else(|| ApiError::new(404, "无此任务")) });
    R!("GET", "/api/v1/containers", "托管容器", a, q => { a.check(q)?; ok(json!({"containers": a.rt.list()})) });
    R!("POST", "/api/v1/containers/run", "启动容器 {name,image,role,instance,env}", a, q => {
        a.full(q)?;
        let mut spec = q.json()?;
        let image = spec.get("image").and_then(|i| i.as_str()).unwrap_or("").to_string();
        if a.rt.image(&image).is_none() { return Err(ApiError::new(404, format!("本机没有镜像 {image}"))); }
        let iid = spec.get("instance").and_then(|i| i.as_str()).unwrap_or("x").to_string();
        if spec.get("data") != Some(&json!(false)) { let role = spec.get("role").and_then(|r| r.as_str()).unwrap_or("sim").to_string(); spec["data_dir"] = json!(format!("{}/instances/{iid}/{role}", a.data)); }
        ok(a.rt.run(&spec).map_err(|e| ApiError::new(500, e))?)
    });
    R!("GET", "/api/v1/containers/{name}", "容器状态", a, q => { a.check(q)?; a.rt.status(&q.params["name"]).map(|s| Response::json(&s)).ok_or_else(|| ApiError::new(404, "容器不存在")) });
    R!("POST", "/api/v1/containers/{name}/stop", "停止", a, q => { a.check(q)?; a.rt.stop(&q.params["name"]); ok(json!({"ok": true})) });
    R!("DELETE", "/api/v1/containers/{name}", "删除", a, q => { a.check(q)?; a.rt.remove(&q.params["name"]); ok(json!({"ok": true})) });
    R!("GET", "/api/v1/containers/{name}/logs", "日志", a, q => { a.check(q)?; ok(json!({"name": q.params["name"], "logs": a.rt.logs(&q.params["name"], q.q_i64("tail", 300)?.max(0) as usize)})) });
    R!("POST", "/api/v1/probe", "探测 URL 连通性 {url}", a, q => {
        a.check(q)?;
        let b = q.json()?;
        let url = b.get("url").and_then(|u| u.as_str()).ok_or_else(|| ApiError::coded(400, "缺少 url", "bad_request"))?.trim_start_matches("http://").to_string();
        let t0 = Instant::now();
        let (hp, path) = match url.find('/') { Some(i) => (url[..i].to_string(), url[i..].to_string()), None => (url.clone(), "/".to_string()) };
        let tmo = b.get("timeout").and_then(|t| t.as_f64()).unwrap_or(3.0);
        let c = rest::Client::new(&format!("http://{hp}"), tmo);
        let ms = |t: Instant| ((t.elapsed().as_secs_f64() * 1000.0) * 10.0).round() / 10.0;
        ok(match c.request("GET", &path, None, Some(tmo), "*/*") { Ok(r) => json!({"ok": r.status < 500, "status": r.status, "ms": ms(t0)}), Err(e) => json!({"ok": false, "error": e, "ms": ms(t0)}) })
    });
    R!("POST", "/api/v1/admin/restart", "重启节点代理 (平台运维)", a, q => { a.check(q)?; Err(ApiError::coded(501, "Rust 节点代理请通过设备上的启动脚本重启", "not_implemented")) });
    R!("POST", "/api/v1/admin/role", "切换节点角色 {role: monitor|full} (平台部署前启用运行环境)", a, q => {
        a.check(q)?;
        let r = q.json()?.get("role").and_then(|r| r.as_str()).unwrap_or("").to_string();
        if r != "monitor" && r != "full" { return Err(ApiError::coded(400, "role 只能是 monitor 或 full", "error")); }
        *a.role.lock().unwrap() = r.clone();
        a.state.lock().unwrap()["role"] = json!(r);
        a.save_state();
        log(&format!("角色切换为 {r}"));
        ok(json!({"ok": true, "role": r, "info": a.info(false)}))
    });
    R!("DELETE", "/api/v1/instances/{iid}", "释放实例资源", a, q => { a.check(q)?; a.release(&q.params["iid"]); ok(json!({"ok": true})) });
    api
}

fn main() {
    let root = std::env::var("AGV_HOME").unwrap_or_else(|_| ".".into());
    let ag = Arc::new(Agent::new(&root));
    if ag.hub.is_empty() { log("未设置 HUB_API，节点代理仅本地运行 (不会注册到平台)"); }
    else { let a2 = ag.clone(); std::thread::Builder::new().name("heartbeat".into()).spawn(move || a2.heartbeat_loop()).ok(); }
    let mut api = build(ag.clone());
    api.host = std::env::var("AGENT_BIND").ok().filter(|h| !h.is_empty()).unwrap_or_else(|| "0.0.0.0".into());
    log(&format!("节点代理 {} :{}  运行时 {}  数据 {}  平台 {}", ag.name, ag.port, ag.rt.name(), ag.data, if ag.hub.is_empty() { "-" } else { &ag.hub }));
    if let Err(e) = Arc::new(api).serve_forever() { eprintln!("[agent] 监听失败: {e}"); std::process::exit(1); }
}
