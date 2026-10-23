//! Compute node registry, agent client, and (lite) package repository.
use crate::store::*;
use serde_json::{json, Map, Value};
use std::path::PathBuf;
use std::sync::Arc;

pub fn kind_label(k: &str) -> &'static str {
    match k { "controller" => "实体运行设备 (控制器)", "sim" => "虚拟机设备 (仿真引擎)", _ => "虚拟机设备 (运行 + 仿真引擎)" }
}
pub fn kind_labels() -> Value { json!({"controller": kind_label("controller"), "hybrid": kind_label("hybrid"), "sim": kind_label("sim")}) }
pub fn caps(n: &Value) -> Value {
    match n.get("kind").and_then(|k| k.as_str()).unwrap_or("hybrid") { "controller" => json!({"sim": false, "nav": true}), "sim" => json!({"sim": true, "nav": false}), _ => json!({"sim": true, "nav": true}) }
}
const OFFLINE_AFTER: f64 = 20.0;

pub fn http_json(url: &str, timeout: f64, method: &str, body: Option<&Value>) -> Result<Value, String> {
    let u = url.trim_start_matches("http://");
    let (hp, path) = match u.find('/') { Some(i) => (&u[..i], &u[i..]), None => (u, "/") };
    let c = rest::Client::new(&format!("http://{hp}"), timeout);
    let r = c.request(method, path, body, Some(timeout), "application/json")?;
    if r.status >= 400 { return Err(format!("HTTP {} {}", r.status, String::from_utf8_lossy(&r.body[..r.body.len().min(200)]))); }
    if r.body.is_empty() { return Ok(json!({})); }
    serde_json::from_slice(&r.body).map_err(|e| e.to_string())
}

pub struct AgentClient { node: Value, c: rest::Client }
impl AgentClient {
    pub fn new(node: &Value) -> AgentClient {
        let base = format!("http://{}:{}", sget(node, "host"), node.get("api_port").and_then(|p| p.as_i64()).unwrap_or(8070));
        AgentClient { node: node.clone(), c: rest::Client::new(&base, 10.0).with_header("X-Node-Key", sget(node, "key")) }
    }
    pub fn call(&self, method: &str, path: &str, body: Option<&Value>, timeout: Option<f64>) -> Result<Value, String> {
        let r = self.c.request(method, path, body, Some(timeout.unwrap_or(10.0)), "application/json")
            .map_err(|e| format!("节点 {} ({}:{}) 不可达: {e}", sget(&self.node, "name"), self.c.host, self.c.port))?;
        let obj: Value = if r.body.is_empty() { json!({}) } else { serde_json::from_slice(&r.body).unwrap_or(json!({})) };
        if r.status >= 400 {
            let m = obj.pointer("/error/message").and_then(|m| m.as_str()).map(|s| s.to_string()).unwrap_or(r.status.to_string());
            return Err(format!("节点 {}: {m}", sget(&self.node, "name")));
        }
        Ok(obj)
    }
}

pub struct NodeRegistry { pub s: Arc<Store>, pub packages: Arc<PackageRepo> }

impl NodeRegistry {
    pub fn new(s: Arc<Store>, packages: Arc<PackageRepo>) -> NodeRegistry {
        if !truthy(s.setting("cluster_token").as_ref()) { s.set_setting("cluster_token", json!(format!("agv-{}", token_urlsafe(12)))); }
        NodeRegistry { s, packages }
    }
    fn live(&self, n: &Value) -> Value {
        let mut n = n.clone();
        if sget(&n, "status") == "pending" { return n; }
        if now() - n.get("last_seen").and_then(|v| v.as_f64()).unwrap_or(0.0) > OFFLINE_AFTER { n["status"] = json!("offline"); }
        n
    }
    pub fn get(&self, nid: &str) -> R<Value> { let n = self.s.get("nodes", nid).ok_or_else(|| err(404, format!("节点 {nid} 不存在")))?; Ok(self.live(&n)) }
    pub fn list(&self) -> Vec<Value> {
        let mut ns = self.s.list("nodes");
        ns.sort_by(|a, b| a.get("created").and_then(|v| v.as_f64()).unwrap_or(0.0).partial_cmp(&b.get("created").and_then(|v| v.as_f64()).unwrap_or(0.0)).unwrap_or(std::cmp::Ordering::Equal));
        ns.iter().map(|n| self.live(n)).collect()
    }
    pub fn lan_host(n: &Value) -> String {
        let h = n.get("host").and_then(|h| h.as_str()).filter(|h| !h.is_empty()).unwrap_or("127.0.0.1");
        if h.starts_with("127.") || h == "localhost" {
            if let Some(ip) = n.pointer("/info/ips").and_then(|i| i.as_array()).and_then(|a| a.first()).and_then(|x| x.as_str()) { return ip.to_string(); }
        }
        h.to_string()
    }
    pub fn addr_between(&self, src: &Value, dst: &Value) -> String { if sget(src, "id") == sget(dst, "id") { "127.0.0.1".into() } else { Self::lan_host(dst) } }

    pub fn view(&self, n: &Value, instances: &[Value]) -> Value {
        let info = n.get("info").cloned().unwrap_or(json!({}));
        let id = sget(n, "id");
        let mine: Vec<&Value> = instances.iter().filter(|i| (sget(i, "sim_node") == id || sget(i, "nav_node") == id) && matches!(sget(i, "status"), "deploying" | "running" | "degraded" | "starting")).collect();
        let st = sget(n, "status");
        let busy = if st == "offline" || st == "pending" { st.to_string() } else if !mine.is_empty() { "running".into() } else { "idle".into() };
        let mem = info.get("mem").cloned().unwrap_or(json!({}));
        let kind = n.get("kind").and_then(|k| k.as_str()).unwrap_or("hybrid");
        let ssh: Map<String, Value> = n.get("ssh").and_then(|s| s.as_object()).map(|o| o.iter().filter(|(k, _)| *k != "password").map(|(k, v)| (k.clone(), v.clone())).collect()).unwrap_or_default();
        let g = |k: &str| info.get(k).cloned().unwrap_or(Value::Null);
        let hist: Vec<Value> = n.get("history").and_then(|h| h.as_array()).cloned().unwrap_or_default();
        let hist: Vec<Value> = hist[hist.len().saturating_sub(60)..].to_vec();
        json!({"id": n["id"], "name": n.get("name").cloned().unwrap_or(Value::Null), "host": n.get("host").cloned().unwrap_or(Value::Null), "lan_host": Self::lan_host(n),
            "api_port": n.get("api_port").cloned().unwrap_or(Value::Null), "kind": kind, "kind_label": kind_label(kind), "caps": caps(n), "arch": n.get("arch").cloned().unwrap_or(Value::Null),
            "status": busy, "online": st != "offline" && st != "pending", "pending": st == "pending", "ssh_check": n.get("ssh_check").cloned().unwrap_or(Value::Null),
            "last_seen": n.get("last_seen").cloned().unwrap_or(Value::Null), "cpu_count": g("cpu_count"), "cpu_percent": g("cpu_percent"), "mem_total": mem.get("total").cloned().unwrap_or(Value::Null),
            "mem_percent": mem.get("percent").cloned().unwrap_or(Value::Null), "temp_c": g("temp_c"), "model": g("model"), "os": g("os"), "runtime": g("runtime"),
            "images": info.get("images").cloned().unwrap_or(json!([])), "containers": info.get("containers").cloned().unwrap_or(json!([])), "ports_in_use": info.get("ports_in_use").cloned().unwrap_or(json!([])),
            "disk": g("disk"), "ips": info.get("ips").cloned().unwrap_or(json!([])), "load": g("load"), "ssh": if ssh.is_empty() { Value::Null } else { Value::Object(ssh) },
            "max_instances": n.get("max_instances").cloned().unwrap_or(json!(1)), "note": n.get("note").cloned().unwrap_or(json!("")),
            "instances": mine.iter().map(|i| json!({"id": i["id"], "name": i.get("name").cloned().unwrap_or(Value::Null), "operator": i.get("operator").cloned().unwrap_or(Value::Null),
                "role": if i.get("sim_node") == i.get("nav_node") { "sim+nav" } else if sget(i, "sim_node") == id { "sim" } else { "nav" }})).collect::<Vec<_>>(),
            "agent_error": g("last_error"), "role": if truthy(info.get("role")) { info["role"].clone() } else { json!("full") }, "history": hist, "created": n.get("created").cloned().unwrap_or(Value::Null)})
    }

    pub fn new_token(&self, operator: &str, note: &str, meta: Value, ttl: f64) -> Value {
        let tok = format!("jt-{}", token_urlsafe(10));
        self.s.put("tokens", json!({"id": tok, "operator": operator, "note": note, "used": false, "expires": now() + ttl, "meta": meta}))
    }

    pub fn register(&self, body: &Value, client_ip: &str) -> R<Value> {
        let nid = sget(body, "node_id");
        let key = sget(body, "node_key");
        let tok = sget(body, "token");
        let host = { let a = sget(body, "advertise_host"); if a.is_empty() { client_ip.to_string() } else { a.to_string() } };
        let mut n = if nid.is_empty() { None } else { self.s.get("nodes", nid) };
        let mut used_tok: Option<String> = None;
        let known = n.as_ref().map(|n| !key.is_empty() && ct_eq(sget(n, "key"), key)).unwrap_or(false);
        if !known {
            let mut ok = !tok.is_empty() && self.s.setting("cluster_token").and_then(|v| v.as_str().map(|s| s.to_string())).map(|c| c == tok).unwrap_or(false);
            let t = if !tok.is_empty() && !ok { self.s.get("tokens", tok) } else { None };
            let mut meta = json!({});
            if let Some(t) = &t { if !truthy(t.get("used")) && t.get("expires").and_then(|e| e.as_f64()).unwrap_or(0.0) > now() { ok = true; meta = t.get("meta").cloned().unwrap_or(json!({})); used_tok = Some(tok.to_string()); } }
            if !ok { return Err(ApiError2::coded(401, "接入令牌无效或已过期，请在平台「添加计算节点」重新生成", "bad_token")); }
            n = meta.get("node_id").and_then(|i| i.as_str()).and_then(|i| self.s.get("nodes", i));
            if n.is_none() { n = self.s.list("nodes").into_iter().find(|x| sget(x, "name") == sget(body, "name") && sget(x, "host") == host); }
            let mut nn = n.unwrap_or_else(|| { let ex: Vec<String> = self.s.list("nodes").iter().map(|x| sget(x, "id").to_string()).collect();
                json!({"id": new_id("n-", &|i| ex.iter().any(|e| e == i)), "created": now(), "kind": if sget(body, "kind").is_empty() { "hybrid" } else { sget(body, "kind") }, "max_instances": 1}) });
            nn["key"] = json!(token_urlsafe(24));
            for k in ["kind", "note", "ssh"] { if truthy(meta.get(k)) { nn[k] = meta[k].clone(); } }
            n = Some(nn);
        }
        let mut n = n.unwrap();
        let info = body.get("info").cloned().unwrap_or(json!({}));
        let name = if truthy(n.get("ssh")) && truthy(n.get("name")) { n["name"].clone() } else if truthy(body.get("name")) { body["name"].clone() } else if truthy(n.get("name")) { n["name"].clone() } else { json!(host) };
        n["name"] = name; n["host"] = json!(host);
        n["api_port"] = json!(body.get("api_port").and_then(|p| p.as_i64()).filter(|p| *p != 0).unwrap_or(8070));
        n["hub_url"] = body.get("hub_url").cloned().unwrap_or(Value::Null);
        n["arch"] = if truthy(info.get("arch")) { info["arch"].clone() } else { n.get("arch").cloned().unwrap_or(Value::Null) };
        n["status"] = json!("online"); n["last_seen"] = json!(now());
        if truthy(Some(&info)) { n["info"] = info; }
        let n = self.s.put("nodes", n);
        if let Some(t) = used_tok { let nid = sget(&n, "id").to_string(); let nm = body.get("name").cloned().unwrap_or(Value::Null);
            self.s.update("tokens", &t, |d| { d["used"] = json!(true); d["used_by"] = nm; d["used_at"] = json!(now()); d["node_id"] = json!(nid); }); }
        Ok(json!({"node_id": n["id"], "node_key": n["key"], "host": host}))
    }

    pub fn heartbeat(&self, nid: &str, key: &str, info: &Value) -> R<Value> {
        let n = self.s.get("nodes", nid).ok_or_else(|| err(404, "节点未注册"))?;
        if !ct_eq(sget(&n, "key"), key) { return Err(err(401, "节点密钥无效")); }
        let mut n = n;
        let mut hist: Vec<Value> = n.get("history").and_then(|h| h.as_array()).cloned().unwrap_or_default();
        hist.push(json!([now().round() as i64, info.get("cpu_percent").cloned().unwrap_or(Value::Null), info.pointer("/mem/percent").cloned().unwrap_or(Value::Null), info.get("temp_c").cloned().unwrap_or(Value::Null)]));
        let keep = hist.len().saturating_sub(120);
        n["info"] = info.clone(); n["last_seen"] = json!(now()); n["status"] = json!("online");
        n["arch"] = if truthy(info.get("arch")) { info["arch"].clone() } else { n.get("arch").cloned().unwrap_or(Value::Null) };
        n["history"] = Value::Array(hist[keep..].to_vec());
        let n = self.s.put("nodes", n);
        self.packages.sync_node_images(&n);
        Ok(json!({"ok": true}))
    }
    pub fn update(&self, nid: &str, f: &Value) -> R<Value> {
        let mut n = self.s.get("nodes", nid).ok_or_else(|| err(404, "节点不存在"))?;
        for k in ["name", "kind", "note", "max_instances"] { if let Some(v) = f.get(k) { n[k] = if k == "max_instances" { json!(v.as_i64().or_else(|| v.as_str().and_then(|s| s.parse().ok())).unwrap_or(1)) } else { v.clone() }; } }
        Ok(self.s.put("nodes", n))
    }
    pub fn delete(&self, nid: &str) { self.s.delete("nodes", nid); }
    pub fn client(&self, nid: &str) -> R<AgentClient> { Ok(AgentClient::new(&self.get(nid)?)) }
    pub fn client_of(&self, n: &Value) -> AgentClient { AgentClient::new(n) }
}

struct ApiError2;
impl ApiError2 { fn coded(s: u16, m: &str, c: &str) -> rest::ApiError { rest::ApiError::coded(s, m, c) } }

// ---------------------------------------------------------------- packages (process/node sources; docker upload not supported)
pub struct PackageRepo { pub s: Arc<Store> }
pub fn node_has_image(node: &Value, p: &Value) -> bool {
    node.pointer("/info/images").and_then(|i| i.as_array()).map(|a| a.iter().any(|im| sget(im, "ref") == sget(p, "image_ref") || (truthy(p.get("image_id")) && im.get("id") == p.get("image_id")))).unwrap_or(false)
}
fn norm_arch(a: &str) -> String { match a { "aarch64" => "arm64".into(), "x86_64" => "amd64".into(), o => o.into() } }
pub const PKG_TAGS: [(&str, &str); 5] = [("baseline", "基准推荐"), ("test", "测试版"), ("current", "当前运行"), ("lite", "轻量版"), ("node", "节点镜像")];

impl PackageRepo {
    pub fn get(&self, pid: &str) -> R<Value> { self.s.get("packages", pid).ok_or_else(|| err(404, format!("程序包 {pid} 不存在"))) }
    pub fn list(&self, kind: Option<&str>) -> Vec<Value> {
        let mut ps = self.s.list("packages");
        if let Some(k) = kind { ps.retain(|p| sget(p, "kind") == k); }
        ps.sort_by(|a, b| (sget(a, "kind").to_string(), -(a.get("created").and_then(|v| v.as_f64()).unwrap_or(0.0))).partial_cmp(&(sget(b, "kind").to_string(), -(b.get("created").and_then(|v| v.as_f64()).unwrap_or(0.0)))).unwrap_or(std::cmp::Ordering::Equal));
        ps
    }
    pub fn file(&self, p: &Value) -> Option<PathBuf> { let f = sget(p, "file"); if f.is_empty() { return None; } let f = self.s.path(&["packages", f]); if f.exists() { Some(f) } else { None } }
    pub fn sync_node_images(&self, node: &Value) {
        let rt = node.pointer("/info/runtime/runtime").and_then(|r| r.as_str()).unwrap_or("");
        let nid = sget(node, "id").to_string();
        for im in node.pointer("/info/images").and_then(|i| i.as_array()).cloned().unwrap_or_default() {
            let key = format!("{}:{}", sget(&im, "kind"), sget(&im, "id"));
            let all = self.s.list("packages");
            if let Some(ex) = all.iter().find(|p| sget(p, "image_key") == key) {
                let mut nodes: Vec<String> = ex.get("nodes").and_then(|n| n.as_array()).map(|a| a.iter().filter_map(|x| x.as_str().map(|s| s.to_string())).collect()).unwrap_or_default();
                if !nodes.contains(&nid) { nodes.push(nid.clone()); nodes.sort(); self.s.update("packages", sget(ex, "id"), |d| d["nodes"] = json!(nodes)); }
                if rt == "process" && truthy(im.get("version")) && ex.get("version") != im.get("version") { let v = im["version"].clone(); self.s.update("packages", sget(ex, "id"), |d| d["version"] = v); }
                continue;
            }
            let ids: Vec<String> = all.iter().map(|p| sget(p, "id").to_string()).collect();
            let pid = new_id("pkg-", &|i| ids.iter().any(|e| e == i));
            let rf = sget(&im, "ref");
            self.s.put("packages", json!({"id": pid, "kind": im["kind"], "source": if rt == "process" { "process" } else { "node" }, "image_ref": rf, "image_key": key, "image_id": im.get("id").cloned().unwrap_or(Value::Null),
                "size": im.get("size").cloned().unwrap_or(json!(0)), "arch": node.get("arch").cloned().unwrap_or(Value::Null),
                "version": if truthy(im.get("version")) { im["version"].clone() } else { json!(rf.rsplit(':').next().unwrap_or("")) }, "api": im.get("api").cloned().unwrap_or(json!("v1")), "nodes": [nid], "tag": "node",
                "note": if rt == "process" { "开发源码运行时".to_string() } else { format!("节点 {} 本机镜像", sget(node, "name")) }, "image_created": im.get("created").cloned().unwrap_or(Value::Null)}));
        }
    }
    pub fn update(&self, pid: &str, f: &Value) -> R<Value> {
        let mut p = self.get(pid)?;
        for k in ["version", "note", "tag"] { if let Some(v) = f.get(k) { p[k] = v.clone(); } }
        if sget(f, "tag") == "baseline" {
            for o in self.s.list("packages") { if sget(&o, "kind") == sget(&p, "kind") && sget(&o, "id") != pid && sget(&o, "tag") == "baseline" { self.s.update("packages", sget(&o, "id"), |d| d["tag"] = json!("")); } }
        }
        Ok(self.s.put("packages", p))
    }
    pub fn delete(&self, pid: &str) -> R<()> { let p = self.get(pid)?; if let Some(f) = self.file(&p) { let _ = std::fs::remove_file(f); } self.s.delete("packages", pid); Ok(()) }
    pub fn deployable_on(&self, p: &Value, node: &Value) -> Option<String> {
        if sget(p, "source") == "process" {
            let rt = node.pointer("/info/runtime/runtime").and_then(|r| r.as_str()).unwrap_or("");
            return if rt == "process" { None } else { Some("开发源码包只能部署到 process 运行时节点".into()) };
        }
        let in_nodes = p.get("nodes").and_then(|n| n.as_array()).map(|a| a.iter().any(|x| x.as_str() == Some(sget(node, "id")))).unwrap_or(false);
        if in_nodes && node_has_image(node, p) { return None; }
        if truthy(p.get("arch")) && truthy(node.get("arch")) && norm_arch(sget(p, "arch")) != norm_arch(sget(node, "arch")) { return Some(format!("架构不符: 包 {} / 节点 {}", sget(p, "arch"), sget(node, "arch"))); }
        if sget(p, "source") == "node" && self.file(p).is_none() {
            let ns: Vec<String> = p.get("nodes").and_then(|n| n.as_array()).map(|a| a.iter().filter_map(|x| x.as_str().map(|s| s.to_string())).collect()).unwrap_or_default();
            return Some(format!("镜像只在节点 {} 上，需先「入库」才能分发到其它节点", ns.join(",")));
        }
        None
    }
}
