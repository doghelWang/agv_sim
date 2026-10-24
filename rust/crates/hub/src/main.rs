mod admin;
mod deployer;
mod models;
mod ops;
mod nodes;
mod scenes;
mod store;

use deployer::Deployer;
use models::ModelRepo;
use nodes::*;
use rest::{ok, ApiError, HResult, Mount, Request, Response, Server};
use scenes::SceneRepo;
use serde_json::{json, Map, Value};
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};
use store::*;

const P: &str = "/api/hub";
const HUB_VERSION: &str = "1.0";

struct Hub { port: u16, root: PathBuf, s: Arc<Store>, packages: Arc<PackageRepo>, nodes: Arc<NodeRegistry>, models: Arc<ModelRepo>, scenes: Arc<SceneRepo>, dep: Arc<Deployer>, admin: admin::Admin, srv: OnceLock<Arc<Server>> }

fn ctype(p: &Path) -> &'static str {
    match p.extension().and_then(|e| e.to_str()).unwrap_or("").to_lowercase().as_str() {
        "html" | "htm" => "text/html; charset=utf-8", "js" | "mjs" => "application/javascript; charset=utf-8", "css" => "text/css; charset=utf-8", "json" => "application/json; charset=utf-8",
        "png" => "image/png", "jpg" | "jpeg" => "image/jpeg", "gif" => "image/gif", "svg" => "image/svg+xml", "ico" => "image/x-icon", "woff2" => "font/woff2", "woff" => "font/woff",
        "txt" | "md" | "yaml" | "yml" => "text/plain; charset=utf-8", "map" => "application/json", "wasm" => "application/wasm", _ => "application/octet-stream",
    }
}
fn user(q: &Request) -> String { rest::url_decode(&q.header("x-user").unwrap_or("").replace('+', "%2B")) }
fn j(v: Value) -> HResult { ok(v) }
fn qd(q: &Request, k: &str) -> Option<String> { q.qs(k).map(|v| v.to_string()) }
fn file_resp(path: &Path, ctype_: &str, filename: &str) -> HResult {
    let data = std::fs::read(path).map_err(|_| err(404, "文件不存在"))?;
    Ok(Response::binary(data, vec![("Content-Disposition".into(), format!("attachment; filename=\"{filename}\""))], ctype_))
}
fn token(q: &Request) -> String { q.header("x-admin-token").map(|s| s.to_string()).or_else(|| qd(q, "token")).unwrap_or_default() }
fn reexec_later() {
    std::thread::spawn(|| {
        std::thread::sleep(std::time::Duration::from_secs(2));
        use std::os::unix::process::CommandExt;
        let exe = std::env::current_exe().unwrap_or_else(|_| "/proc/self/exe".into());
        let e = std::process::Command::new(exe).args(std::env::args().skip(1)).exec();
        eprintln!("[restart] exec 失败: {e}");
    });
}

impl Hub {
    fn insts(&self) -> Vec<Value> { self.dep.list(false) }
    fn inst_view(&self, i: &Value) -> Value {
        let mut v = Map::new();
        for k in ["id", "name", "operator", "status", "created", "started", "ended", "error", "steps", "warnings", "model_id", "model_ver", "model_name", "scene_id", "scene_name", "sim_node", "nav_node", "sim_pkg", "nav_pkg", "ports", "external", "health", "brief", "ros_domain", "containers"] {
            v.insert(k.into(), i.get(k).cloned().unwrap_or(Value::Null));
        }
        for role in ["sim_node", "nav_node"] {
            let n = self.s.get("nodes", sget(i, role));
            v.insert(format!("{role}_view"), match n { Some(n) => json!({"id": n["id"], "name": n.get("name").cloned().unwrap_or(Value::Null), "host": NodeRegistry::lan_host(&n), "kind": n.get("kind").cloned().unwrap_or(Value::Null),
                "kind_label": n.get("kind").and_then(|k| k.as_str()).map(|k| json!(kind_label(k))).unwrap_or(Value::Null)}), None => Value::Null });
        }
        for role in ["sim_pkg", "nav_pkg"] {
            let p = self.s.get("packages", sget(i, role));
            v.insert(format!("{role}_view"), match p { Some(p) => json!({"id": p["id"], "version": p.get("version").cloned().unwrap_or(Value::Null), "image_ref": p.get("image_ref").cloned().unwrap_or(Value::Null)}), None => Value::Null });
        }
        v.insert("web_url".into(), i.pointer("/urls/web").cloned().unwrap_or(Value::Null));
        Value::Object(v)
    }
    fn overview(&self) -> Value {
        let insts = self.insts();
        let nodes: Vec<Value> = self.nodes.list().iter().map(|n| self.nodes.view(n, &insts)).collect();
        let (models, scenes, pkgs) = (self.s.list("models"), self.s.list("scenes"), self.s.list("packages"));
        let mut by_kind = Map::new();
        for n in &nodes { let k = sget(n, "kind").to_string(); let c = by_kind.get(&k).and_then(|c| c.as_i64()).unwrap_or(0); by_kind.insert(k, json!(c + 1)); }
        let mut vt = Map::new();
        for m in &models { let k = m.pointer("/summary/chassis_label").and_then(|k| k.as_str()).filter(|k| !k.is_empty()).unwrap_or("AMR").to_string(); let c = vt.get(&k).and_then(|c| c.as_i64()).unwrap_or(0); vt.insert(k, json!(c + 1)); }
        let active = insts.iter().filter(|i| deployer::ACTIVE.contains(&sget(i, "status"))).count();
        let cnt = |ns: &Vec<Value>, f: &dyn Fn(&Value) -> bool| ns.iter().filter(|n| f(n)).count();
        json!({"nodes": {"total": nodes.len(), "idle": cnt(&nodes, &|n| sget(n, "status") == "idle"), "online": cnt(&nodes, &|n| n["online"] == json!(true)), "by_kind": by_kind},
            "models": {"total": models.len(), "by_chassis": vt}, "scenes": {"total": scenes.len(), "names": scenes.iter().take(4).map(|s| s.get("name").cloned().unwrap_or(Value::Null)).collect::<Vec<_>>()},
            "packages": {"total": pkgs.len(), "nav": cnt(&pkgs, &|p| sget(p, "kind") == "nav"), "sim": cnt(&pkgs, &|p| sget(p, "kind") == "sim")}, "instances": {"active": active, "total": insts.len()}})
    }
}

// ---------------------------------------------------------------- reverse proxy (workbench → instance gateway)
const HOP: [&str; 9] = ["connection", "keep-alive", "proxy-authenticate", "proxy-authorization", "te", "trailers", "transfer-encoding", "upgrade", "host"];
fn proxy(q: &Request, target_base: &str, sub: &str) -> Result<Response, ApiError> {
    let c = rest::Client::new(target_base, 15.0);
    let wait = q.qs("wait").and_then(|w| w.parse::<f64>().ok()).unwrap_or(0.0);
    let mut extra: Vec<(String, String)> = q.headers.iter().filter(|(k, _)| !HOP.contains(&k.as_str()) && *k != "content-length" && *k != "content-type" && *k != "accept").map(|(k, v)| (k.clone(), v.clone())).collect();
    let prefix = &q.target[..q.target.len() - sub.len()];
    extra.push(("X-Forwarded-Prefix".into(), prefix.to_string()));
    let body = if q.body.is_empty() { None } else { Some(q.body.as_slice()) };
    let ct = q.header("content-type").unwrap_or("application/octet-stream");
    let accept = q.header("accept").unwrap_or("*/*");
    let method = if q.method == "HEAD" { "GET" } else { q.method.as_str() };
    let r = c.request_raw(method, sub, body, ct, &extra, Some(15.0 + wait), accept).map_err(|e| ApiError::coded(502, format!("实例网关不可达 ({target_base}): {e}"), "bad_gateway"))?;
    let mut resp = Response::raw(r.body, r.headers.get("content-type").map(|s| s.as_str()).unwrap_or("application/octet-stream")).with_status(r.status);
    for (k, v) in &r.headers { if !HOP.contains(&k.as_str()) && k != "content-length" && k != "content-type" { resp = resp.header(k, v); } }
    Ok(resp)
}

fn static_file(hub: &Hub, q: &Request) -> HResult {
    let web = hub.root.join("web");
    let p = q.path.as_str();
    let f: PathBuf = if p == "/" || p == "/index.html" { web.join("index.html") }
        else if let Some(r) = p.strip_prefix("/vendor/") { hub.root.join("vendor").join(r) }
        else if p == "/model-editor" || p == "/model-editor.html" { hub.root.join("model_editor.html") }
        else { web.join(p.trim_start_matches('/')) };
    let canon = f.canonicalize().map_err(|_| err(404, format!("未找到 {p}")))?;
    let allowed = [web.canonicalize().ok(), hub.root.join("vendor").canonicalize().ok(), hub.root.join("model_editor.html").canonicalize().ok()];
    if !canon.is_file() || !allowed.iter().flatten().any(|a| canon.starts_with(a) || &canon == a) { return Err(err(404, format!("未找到 {p}"))); }
    let data = std::fs::read(&canon).map_err(|_| err(404, format!("未找到 {p}")))?;
    Ok(Response::raw(data, ctype(&canon)).header("Cache-Control", "no-cache"))
}

fn build(hub: Arc<Hub>) -> Server {
    let mut api = Server::with_base("hub", hub.port, "/api/v1", "资源平台 agv-hub", "计算节点、车辆模型、仿真场景、软件程序包、仿真实例部署与记录、运维管理。接口在 /api/hub 下 (v1 契约)；/inst/<实例>/ 反向代理到实例的 Web 网关");
    macro_rules! R { ($m:expr, $p:expr, $d:expr, $h:ident, $q:ident => $body:expr) => {{ let $h = hub.clone(); api.route($m, &$p, $d, move |$q: &Request| -> HResult { let _ = &$h; let _ = $q; $body }); }}; }
    let pp = |s: &str| format!("{P}{s}");
    R!("GET", pp("/health"), "健康检查", h, q => j(json!({"service": "hub", "ok": true, "version": HUB_VERSION})));
    R!("GET", pp("/overview"), "资源总览", h, q => j(h.overview()));

    // ---- nodes
    R!("GET", pp("/nodes"), "计算节点", h, q => { let ins = h.insts(); j(json!({"nodes": h.nodes.list().iter().map(|n| h.nodes.view(n, &ins)).collect::<Vec<_>>(), "kinds": kind_labels()})) });
    R!("GET", pp("/nodes/{nid}"), "节点详情", h, q => j(h.nodes.view(&h.nodes.get(&q.params["nid"])?, &h.insts())));
    R!("PATCH", pp("/nodes/{nid}"), "修改节点 {name, kind: controller|hybrid|sim, note, max_instances}", h, q => { let n = h.nodes.update(&q.params["nid"], &q.json()?)?; j(h.nodes.view(&n, &h.insts())) });
    R!("DELETE", pp("/nodes/{nid}"), "移除节点", h, q => { h.nodes.delete(&q.params["nid"]); j(json!({"ok": true})) });
    R!("POST", pp("/nodes/enroll"), "生成节点接入令牌与命令", h, q => {
        let b = q.json()?;
        let t = h.nodes.new_token(&user(q), sget(&b, "note"), json!({}), 24.0 * 3600.0);
        let host = q.header("host").map(|s| s.split(',').next().unwrap_or("").to_string()).unwrap_or(format!("127.0.0.1:{}", h.port));
        let hub_url = format!("http://{host}");
        let kind = b.get("kind").and_then(|k| k.as_str()).unwrap_or("hybrid");
        j(json!({"token": t["id"], "expires": t["expires"], "hub_url": hub_url, "command": format!("bash deploy.sh agent --hub {hub_url} --token {} --kind {kind}", sget(&t, "id")),
            "docker": format!("docker run -d --name agv-agent --restart unless-stopped --network host -v /var/run/docker.sock:/var/run/docker.sock -v $HOME/.agv-agent:$HOME/.agv-agent -e AGENT_DATA=$HOME/.agv-agent -e HUB_API={hub_url} -e JOIN_TOKEN={} -e AGENT_KIND={kind} agv-platform:latest agent", sget(&t, "id"))}))
    });
    R!("POST", pp("/nodes/add"), "添加计算节点 {kind,note,name,ip,ssh_port,username,password}: 验证 SSH 后保存 (首次部署时安装运行环境)", h, q => j(ops::nodes_add(&h, &q.json()?)?));
    R!("POST", pp("/nodes/register"), "节点代理注册 {name, token, node_id, node_key, api_port, advertise_host, kind, hub_url, info} (节点代理调用)", h, q => {
        let ip = q.client.rsplit_once(':').map(|x| x.0).unwrap_or("127.0.0.1").to_string();
        j(h.nodes.register(&q.json()?, &ip)?)
    });
    R!("POST", pp("/nodes/{nid}/heartbeat"), "节点心跳 (请求体为节点信息 info；请求头 X-Node-Key)", h, q => j(h.nodes.heartbeat(&q.params["nid"], q.header("x-node-key").unwrap_or(""), &q.json()?)?));
    R!("GET", pp("/nodes/{nid}/jobs/{jid}"), "节点任务进度", h, q => j(h.nodes.client(&q.params["nid"])?.call("GET", &format!("/api/v1/jobs/{}", q.params["jid"]), None, None).map_err(|e| err(502, e))?));
    R!("POST", pp("/nodes/probe"), "探测两节点之间的连通性 {from,to}", h, q => {
        let b = q.json()?;
        let (a, bn) = (h.nodes.get(sget(&b, "from"))?, h.nodes.get(sget(&b, "to"))?);
        let url = format!("http://{}:{}/api/v1/health", h.nodes.addr_between(&a, &bn), bn.get("api_port").and_then(|p| p.as_i64()).unwrap_or(8070));
        j(h.nodes.client_of(&a).call("POST", "/api/v1/probe", Some(&json!({"url": url})), None).map_err(|e| err(502, e))?)
    });

    // ---- api docs
    const DOC_SVCS: [(&str, &str); 5] = [("hub", "资源平台 agv-hub"), ("agent", "节点代理 agv-agent"), ("sim", "仿真进程 sim_server"), ("nav", "执行进程 nav_runtime"), ("gateway", "设备端 Web 网关 v2")];
    R!("GET", pp("/apidocs"), "接口文档目录", h, q => j(json!({"services": DOC_SVCS.iter().map(|(k, t)| json!({"id": k, "title": t})).collect::<Vec<_>>(), "standards": ["OpenAPI 3.1", "RFC 9457", "RFC 8631", "RFC 9727", "Google AIP-185"]})));
    R!("GET", pp("/apidocs/{svc}"), "某个服务的 OpenAPI 3.1 文档 (svc: hub|agent|sim|nav|gateway)；服务在运行时取实时文档，否则取仓库里的离线快照", h, q => {
        let svc = q.params["svc"].as_str();
        if svc == "hub" { let mut spec = h.srv.get().map(|s| s.openapi()).unwrap_or(json!({})); spec["x-source"] = json!({"kind": "live", "from": "hub"}); return j(spec); }
        if !DOC_SVCS.iter().any(|(k, _)| *k == svc) { return Err(err(404, format!("没有服务 {svc} 的接口文档"))); }
        let live: Option<(String, String)> = if svc == "agent" {
            h.nodes.list().iter().find(|n| !matches!(sget(n, "status"), "offline" | "pending") && truthy(n.get("key"))).map(|n| (format!("http://{}:{}/api/v1/openapi.json", sget(n, "host"), n.get("api_port").and_then(|p| p.as_i64()).unwrap_or(8070)), sget(n, "name").to_string()))
        } else {
            let key = match svc { "sim" => "sim_api", "nav" => "nav_api", _ => "web" };
            h.insts().iter().find_map(|i| { let u = i.pointer(&format!("/urls/{key}")).and_then(|u| u.as_str()).filter(|u| !u.is_empty())?; if matches!(sget(i, "status"), "running" | "degraded") { Some((format!("{}{}", u.trim_end_matches('/'), if svc == "gateway" { "/api/v2/openapi.json" } else { "/api/v1/openapi.json" }), sget(i, "id").to_string())) } else { None } })
        };
        if let Some((url, who)) = live { if let Ok(mut spec) = http_json(&url, 4.0, "GET", None) { spec["x-source"] = json!({"kind": "live", "from": who}); return j(spec); } }
        let rel = format!("docs/openapi/{svc}.json");
        let mut spec = Store::read_json(&h.root.join(&rel)).ok_or_else(|| err(404, format!("{svc} 没有在运行，也没有离线文档 ({rel})")))?;
        spec["x-source"] = json!({"kind": "snapshot", "from": rel});
        j(spec)
    });
    R!("GET", "/.well-known/api-catalog".to_string(), "API 目录 (RFC 9727)", h, q => {
        let origin = format!("http://{}", q.header("host").map(|s| s.split(',').next().unwrap_or("").to_string()).unwrap_or(format!("127.0.0.1:{}", h.port)));
        let mut items = vec![json!({"anchor": format!("{origin}/api/hub"), "service-desc": [{"href": format!("{origin}/api/v1/openapi.json"), "type": "application/openapi+json"}],
            "service-doc": [{"href": format!("{origin}/#/platform/apidocs"), "type": "text/html"}], "status": [{"href": format!("{origin}/api/hub/health"), "type": "application/json"}]})];
        for i in h.insts() { if matches!(sget(&i, "status"), "running" | "degraded") { let id = sget(&i, "id");
            items.push(json!({"anchor": format!("{origin}/inst/{id}/api/v2"), "service-desc": [{"href": format!("{origin}/inst/{id}/api/v2/openapi.json"), "type": "application/openapi+json"}], "title": [{"value": format!("仿真实例 {id} 的 Web 网关")}]})); } }
        for n in h.nodes.list() { if !matches!(sget(&n, "status"), "offline" | "pending") && truthy(n.get("key")) {
            let base = format!("http://{}:{}/api/v1", NodeRegistry::lan_host(&n), n.get("api_port").and_then(|p| p.as_i64()).unwrap_or(8070));
            items.push(json!({"anchor": base, "service-desc": [{"href": format!("{base}/openapi.json"), "type": "application/openapi+json"}], "title": [{"value": format!("节点代理 {}", sget(&n, "name"))}]})); } }
        Ok(Response::raw(serde_json::to_vec(&json!({"linkset": items})).unwrap(), "application/linkset+json; profile=\"https://www.rfc-editor.org/info/rfc9727\"").header("Link", &format!("<{origin}/.well-known/api-catalog>; rel=\"api-catalog\"")))
    });

    // ---- admin
    let a = format!("{P}/admin");
    R!("GET", format!("{a}/state"), "运维登录状态", h, q => j(h.admin.state(&token(q))));
    R!("POST", format!("{a}/setup"), "首次设置管理员密码", h, q => j(h.admin.setup(sget(&q.json()?, "password"))?));
    R!("POST", format!("{a}/login"), "管理员登录", h, q => j(h.admin.login(sget(&q.json()?, "password"))?));
    R!("POST", format!("{a}/logout"), "退出登录", h, q => j(h.admin.logout(&token(q))));
    R!("POST", format!("{a}/password"), "修改管理员密码", h, q => { h.admin.check(&token(q), true)?; let b = q.json()?; j(h.admin.change_pw(sget(&b, "old"), sget(&b, "new"))?) });
    R!("GET", format!("{a}/services"), "服务状态", h, q => {
        h.admin.check(&token(q), true)?;
        let v = h.admin.version();
        let nodes: Vec<Value> = h.nodes.list().iter().map(|n| { let rt = n.pointer("/info/runtime").cloned().unwrap_or(json!({})); json!({"id": n["id"], "name": n.get("name").cloned().unwrap_or(Value::Null), "host": n.get("host").cloned().unwrap_or(Value::Null),
            "online": sget(n, "status") != "offline", "last_seen": n.get("last_seen").cloned().unwrap_or(Value::Null), "runtime": rt.get("runtime").cloned().unwrap_or(Value::Null),
            "local_code": sget(&rt, "runtime") == "process" && std::fs::canonicalize(sget(&rt, "code_dir")).ok() == std::fs::canonicalize(&h.root).ok()}) }).collect();
        let insts: Vec<Value> = h.insts().iter().filter(|i| matches!(sget(i, "status"), "running" | "degraded" | "deploying" | "stopping")).map(|i| json!({"id": i["id"], "name": i.get("name").cloned().unwrap_or(Value::Null), "status": i["status"], "started": i.get("started").cloned().unwrap_or(Value::Null)})).collect();
        j(json!({"hub": {"version": v["version"], "title": v["title"], "pid": std::process::id(), "started": h.admin.started, "root": h.root.to_string_lossy(), "python": "rust", "host": std::fs::read_to_string("/etc/hostname").unwrap_or_default().trim()},
            "nodes": nodes, "instances": insts, "helper": null, "termux": false, "job": null}))
    });
    R!("POST", format!("{a}/restart"), "重启 {target: hub|agent}", h, q => { h.admin.check(&token(q), true)?; 
        match q.json().ok().and_then(|b| b.get("target").and_then(|t| t.as_str().map(|s| s.to_string()))).as_deref() {
            Some("hub") => { reexec_later(); j(json!({"ok": true, "msg": "平台将在 2 秒后重启，约 10 秒后恢复"})) }
            Some("agent") => j(ops::restart_agent_job(&h, &user(q))?),
            _ => Err::<Response, _>(err(400, "未知的重启目标")),
        } });
    R!("GET", format!("{a}/logs"), "日志文件列表", h, q => { h.admin.check(&token(q), true)?; j(ops::logs(&h)) });
    R!("GET", format!("{a}/logs/{{name}}"), "下载日志", h, q => { h.admin.check(&token(q), true)?; let n = q.params["name"].clone(); Ok(Response::binary(ops::log_file(&h, &n)?, vec![("Content-Disposition".into(), format!("attachment; filename=\"{n}\""))], "text/plain; charset=utf-8")) });
    R!("GET", format!("{a}/updates"), "更新包列表", h, q => { h.admin.check(&token(q), true)?; j(json!({"updates": ops::updates(&h), "current": h.admin.version()})) });
    R!("POST", format!("{a}/updates/upload"), "上传更新包 (请求体为 .tgz)", h, q => { h.admin.check(&token(q), true)?; j(ops::upload(&h, &q.body, &user(q))?) });
    R!("GET", format!("{a}/updates/{{uid}}"), "更新包详情与预览", h, q => { h.admin.check(&token(q), true)?; j(ops::view(&h, &q.params["uid"])?) });
    R!("POST", format!("{a}/updates/{{uid}}/apply"), "应用更新 {restart_instances}", h, q => { h.admin.check(&token(q), true)?; j(ops::apply(&h, &q.params["uid"], &q.json().unwrap_or(json!({})), &user(q))?) });
    R!("POST", format!("{a}/updates/{{uid}}/rollback"), "回滚最近一次更新 {restart_instances}", h, q => { h.admin.check(&token(q), true)?; j(ops::rollback(&h, &q.params["uid"], &q.json().unwrap_or(json!({})), &user(q))?) });
    R!("DELETE", format!("{a}/updates/{{uid}}"), "删除更新包", h, q => { h.admin.check(&token(q), true)?; j(ops::delete_update(&h, &q.params["uid"])?) });
    R!("GET", format!("{a}/jobs"), "运维任务", h, q => { h.admin.check(&token(q), true)?; j(ops::jobs(&h)) });
    R!("GET", format!("{a}/jobs/{{jid}}"), "运维任务详情", h, q => { h.admin.check(&token(q), true)?; j(ops::job_view(&h, &q.params["jid"])?) });
    R!("GET", format!("{a}/apps"), "App 列表与安装状态", h, q => { h.admin.check(&token(q), true)?; j(ops::apps(&h)) });
    R!("POST", format!("{a}/apps/upload"), "上传 APK (请求体为文件)", h, q => { h.admin.check(&token(q), true)?; j(ops::app_upload(&h, &q.body, &rest::url_decode(&q.q_str("filename", "")), &user(q))?) });
    R!("GET", format!("{a}/apps/{{aid}}/download"), "下载 APK", h, q => { h.admin.check(&token(q), true)?; let id = q.params["aid"].clone(); Ok(Response::binary(ops::app_download(&h, &id)?, vec![("Content-Disposition".into(), format!("attachment; filename=\"{id}.apk\""))], "application/vnd.android.package-archive")) });
    R!("DELETE", format!("{a}/apps/{{aid}}"), "删除 APK", h, q => { h.admin.check(&token(q), true)?; j(ops::app_delete(&h, &q.params["aid"])?) });
    R!("POST", format!("{a}/apps/{{aid}}/install"), "安装 APK {mode: adb, serial}", h, q => { h.admin.check(&token(q), true)?; let b = q.json().unwrap_or(json!({})); j(ops::app_install(&h, &q.params["aid"], if truthy(b.get("mode")) { sget(&b, "mode") } else { "adb" }, sget(&b, "serial"))?) });
    R!("POST", format!("{a}/adb/{{action}}"), "无线调试 adb (pair|connect|disconnect|mdns) {host, port, code}", h, q => { h.admin.check(&token(q), true)?; j(ops::adb_action(&q.params["action"], &q.json().unwrap_or(json!({})))?) });
    R!("POST", format!("{a}/panel"), "外屏面板 open|close (需要安卓助手，Linux 平台没有外屏)", h, q => { h.admin.check(&token(q), true)?; Err::<Response, _>(err(400, "外屏面板只在带安卓助手的手机上可用")) });

    // ---- models
    R!("GET", pp("/models"), "车辆模型", h, q => j(json!({"models": h.models.list()})));
    R!("GET", pp("/models/{mid}"), "模型详情 (含生效参数)", h, q => { let m = h.models.get(&q.params["mid"])?; let mut v = h.models.view(&m); v["spec"] = h.models.spec(&q.params["mid"], q.qs("ver"))?; j(v) });
    R!("POST", pp("/models/upload"), "上传 .cmodel (请求体为文件内容，?filename=&name=&project=&vtype=&material_no=)", h, q => {
        let un = |k: &str| qd(q, k).map(|v| rest::url_decode(&v.replace('+', "%2B")));
        let fnm = un("filename").unwrap_or("model.cmodel".into());
        let mut meta = serde_json::Map::new();
        for k in ["name", "project", "vtype", "material_no", "note", "version", "model_id"] { if let Some(v) = un(k) { meta.insert(k.into(), json!(v)); } }
        meta.insert("operator".into(), json!(user(q)));
        j(h.models.add_cmodel(&fnm, &q.body, &Value::Object(meta))?)
    });
    R!("PATCH", pp("/models/{mid}"), "修改模型信息 {name, project, vtype, material_no, note}", h, q => j(h.models.update_meta(&q.params["mid"], &q.json()?)?));
    R!("DELETE", pp("/models/{mid}"), "删除模型", h, q => { h.models.delete(&q.params["mid"])?; j(json!({"ok": true})) });
    R!("GET", pp("/models/{mid}/cmodel"), "下载 cmodel", h, q => { let m = h.models.get(&q.params["mid"])?; let f = h.models.cmodel_file(&q.params["mid"], q.qs("ver"))?; file_resp(&f, "application/octet-stream", if truthy(m.get("file")) { sget(&m, "file") } else { "model.cmodel" }) });
    let v = format!("{P}/models/{{mid}}/versions/{{ver}}");
    R!("GET", format!("{v}/bundle"), "实例启动用模型包 (基线+补全)", h, q => j(h.models.bundle(&q.params["mid"], Some(&q.params["ver"]))?));
    R!("GET", format!("{v}/api/v1/model/editor"), "补全编辑器数据", h, q => j(h.models.editor(&q.params["mid"], Some(&q.params["ver"]))?));
    R!("POST", format!("{v}/api/v1/model/preview"), "试算补全", h, q => j(h.models.preview(&q.params["mid"], Some(&q.params["ver"]), &q.json()?)?));
    R!("PUT", format!("{v}/api/v1/model/overrides"), "保存补全", h, q => j(h.models.save_overrides(&q.params["mid"], Some(&q.params["ver"]), &q.json()?)?));
    R!("GET", format!("{v}/api/v1/model/sensor_template"), "传感器模板", h, q => j(model::overrides::sensor_template(&q.q_str("type", "lidar2d")).map_err(|e| err(400, e))?));
    R!("GET", format!("{v}/api/v1/model/urdf"), "URDF", h, q => Ok(Response::raw(h.models.urdf(&q.params["mid"], Some(&q.params["ver"]))?.into_bytes(), "application/xml")));
    R!("GET", format!("{v}/api/v1/model"), "生效参数", h, q => j(h.models.spec(&q.params["mid"], Some(&q.params["ver"]))?));
    R!("GET", format!("{v}/api/v1/sensors/cameras"), "离线编辑无相机画面", h, q => j(json!({"cameras": [], "offline": true})));

    // ---- scenes
    R!("GET", pp("/scenes"), "仿真场景", h, q => j(json!({"scenes": h.scenes.list()})));
    R!("GET", pp("/scenes/{sid}"), "场景详情 (含定义)", h, q => { let d = h.scenes.get(&q.params["sid"])?; let mut v = h.scenes.view(&d); v["scene"] = h.scenes.scene_def(&q.params["sid"])?; j(v) });
    R!("POST", pp("/scenes/upload"), "上传场景包 (.zip / scene.json)", h, q => {
        let fnm = rest::url_decode(&q.q_str("filename", "scene.zip"));
        let mut meta = Map::new();
        for k in ["name", "description", "tags", "scene_id"] { if let Some(v) = q.qs(k) { meta.insert(k.into(), json!(v)); } }
        meta.insert("operator".into(), json!(user(q)));
        j(h.scenes.import_upload(&fnm, &q.body, &meta)?)
    });
    R!("PATCH", pp("/scenes/{sid}"), "修改场景信息 {name, description, tags}", h, q => j(h.scenes.update_meta(&q.params["sid"], &q.json()?)?));
    R!("DELETE", pp("/scenes/{sid}"), "删除场景", h, q => { h.scenes.delete(&q.params["sid"])?; j(json!({"ok": true})) });
    R!("GET", pp("/scenes/{sid}/definition"), "场景定义 JSON", h, q => j(h.scenes.scene_def(&q.params["sid"])?));
    R!("GET", pp("/scenes/{sid}/map"), "栅格地图 ?part=pgm|yaml", h, q => { let (pgm, yml) = h.scenes.map_files(&q.params["sid"])?; Ok(if q.q_str("part", "pgm") == "pgm" { Response::raw(pgm, "image/x-portable-graymap") } else { Response::raw(yml.into_bytes(), "text/yaml") }) });
    R!("GET", pp("/scenes/{sid}/package"), "下载场景包", h, q => { let z = h.scenes.package_zip(&q.params["sid"])?; Ok(Response::binary(z, vec![("Content-Disposition".into(), format!("attachment; filename*=UTF-8''{}.zip", rest::url_encode(&q.params["sid"])))], "application/zip")) });
    R!("GET", pp("/scenes/{sid}/taskflows"), "场景任务流", h, q => j(json!({"taskflows": h.scenes.taskflows(&q.params["sid"])?})));
    R!("PUT", pp("/scenes/{sid}/taskflows"), "保存任务流", h, q => j(json!({"taskflows": h.scenes.save_taskflows(&q.params["sid"], q.json()?.get("taskflows").unwrap_or(&Value::Null))?})));

    // ---- packages
    R!("GET", pp("/packages"), "软件程序包", h, q => j(json!({"packages": h.packages.list(q.qs("kind")), "tags": PKG_TAGS.iter().map(|(k, v)| (k.to_string(), json!(v))).collect::<Map<_, _>>()})));
    R!("PATCH", pp("/packages/{pid}"), "修改程序包 {version, note, tag}", h, q => j(h.packages.update(&q.params["pid"], &q.json()?)?));
    R!("DELETE", pp("/packages/{pid}"), "删除程序包", h, q => { h.packages.delete(&q.params["pid"])?; j(json!({"ok": true})) });
    R!("GET", pp("/packages/{pid}/image"), "下载镜像包 (节点代理导入用)", h, q => { let p = h.packages.get(&q.params["pid"])?; let f = h.packages.file(&p).ok_or_else(|| err(404, "该程序包没有镜像文件 (节点镜像需先入库)"))?; file_resp(&f, "application/x-tar", &format!("{}.tar", sget(&p, "id"))) });
    R!("POST", pp("/packages/from_node"), "节点镜像入库 (导出上传到平台) {node_id, ref, kind, version, note}", h, q => j(ops::package_from_node(&h, q)?));
    R!("POST", pp("/packages/upload"), "上传程序包 (请求体为 tar/tar.gz: docker save 镜像或含 bin/ 的程序包)", h, q => j(ops::package_upload(&h, q)?));

    // ---- instances
    R!("POST", pp("/deployments/check"), "部署前校验 {sim_node, nav_node, model_id, model_ver, scene_id, sim_pkg, nav_pkg}", h, q => j(h.dep.check(&q.json()?)));
    R!("GET", pp("/instances"), "仿真实例", h, q => j(json!({"instances": h.dep.list(q.q_bool("active", false)).iter().map(|i| h.inst_view(i)).collect::<Vec<_>>()})));
    R!("POST", pp("/instances"), "部署新实例 {name, sim_node, nav_node, model_id, model_ver, scene_id, sim_pkg, nav_pkg, options} (先用 /deployments/check 校验)", h, q => { let i = h.dep.deploy(&q.json()?, &user(q))?; j(h.inst_view(&i)) });
    R!("POST", pp("/instances/attach"), "接入外部实例 {name, web_url}", h, q => { let i = h.dep.attach(&q.json()?, &user(q))?; j(h.inst_view(&i)) });
    R!("GET", pp("/instances/{iid}"), "实例详情", h, q => j(h.inst_view(&h.dep.get(&q.params["iid"])?)));
    R!("POST", pp("/instances/{iid}/stop"), "终止仿真", h, q => { let i = h.dep.stop(&q.params["iid"])?; j(h.inst_view(&i)) });
    R!("POST", pp("/instances/{iid}/restart"), "重启实例", h, q => { let i = h.dep.restart(&q.params["iid"])?; j(h.inst_view(&i)) });
    R!("DELETE", pp("/instances/{iid}"), "删除实例记录", h, q => { h.dep.delete(&q.params["iid"])?; j(json!({"ok": true})) });
    R!("GET", pp("/instances/{iid}/logs"), "容器日志", h, q => j(h.dep.logs(&q.params["iid"], &q.q_str("svc", "sim"), q.q_i64("tail", 300)?)?));
    R!("GET", pp("/instances/{iid}/logs/file"), "下载实例日志文件 ?svc=sim|nav&tail=20000", h, q => {
        let (iid, svc) = (q.params["iid"].clone(), q.q_str("svc", "sim"));
        let r = h.dep.logs(&iid, &svc, q.q_i64("tail", 20000)?)?;
        let fnm = format!("{iid}_{}_{}.log", if svc == "sim" { "sim-engine" } else { "nav" }, unix_stamp());
        Ok(Response::binary(sget(&r, "logs").as_bytes().to_vec(), vec![("Content-Disposition".into(), format!("attachment; filename=\"{fnm}\""))], "text/plain; charset=utf-8"))
    });

    // ---- records
    R!("POST", pp("/records"), "归档仿真记录 (sim_bundle)", h, q => {
        let b = q.json()?;
        let meta = b.get("metadata").cloned().unwrap_or(json!({}));
        let mv = |k: &str| meta.get(k).filter(|v| truthy(Some(v))).map(|v| match v { Value::String(s) => s.clone(), o => o.to_string() });
        let ex: Vec<String> = h.s.list("records").iter().map(|r| sget(r, "id").to_string()).collect();
        let rid = format!("{}-{}", mv("instanceId").unwrap_or("x".into()), mv("recordId").or_else(|| mv("taskId")).unwrap_or_else(|| new_id("r", &|i| ex.iter().any(|e| e == i))));
        h.s.write_json(&h.s.path(&["records", &format!("{rid}.json")]), &b);
        let g = |k: &str| meta.get(k).cloned().unwrap_or(Value::Null);
        j(h.s.put("records", json!({"id": rid, "instance_id": g("instanceId"), "task_id": g("taskId"), "task_name": g("taskName"), "recorded_at": g("recordedAt"), "duration": g("durationSeconds"), "result": g("result"),
            "scene": b.pointer("/environment/sceneName").cloned().unwrap_or(Value::Null), "model": b.pointer("/vehicleModel/modelName").cloned().unwrap_or(Value::Null),
            "injections": b.get("injectedElements").and_then(|e| e.as_array()).map(|a| a.len()).unwrap_or(0), "operator": g("operator")})))
    });
    R!("GET", pp("/records"), "仿真记录", h, q => { let inst = q.qs("instance").map(|s| s.to_string()); j(json!({"records": h.s.list("records").into_iter().filter(|r| inst.as_deref().map(|i| sget(r, "instance_id") == i).unwrap_or(true)).collect::<Vec<_>>()})) });
    R!("GET", pp("/records/{rid}"), "下载记录 sim_bundle", h, q => { let f = h.s.path(&["records", &format!("{}.json", q.params["rid"])]); if !f.exists() { return Err(err(404, "记录不存在")); } file_resp(&f, "application/json", &format!("sim_bundle_{}.json", q.params["rid"])) });
    R!("DELETE", pp("/records/{rid}"), "删除记录", h, q => { h.s.delete("records", &q.params["rid"]); j(json!({"ok": true})) });

    // ---- workbench proxy
    let h2 = hub.clone();
    api.mount("/inst/", move |q: &Request, _s| {
        let rest_ = &q.target["/inst/".len()..];
        let (iid, sub) = match rest_.find('/') { Some(i) => (&rest_[..i], &rest_[i..]), None => (rest_, "/") };
        let iid = iid.split('?').next().unwrap_or("");
        let web = h2.s.get("instances", iid).and_then(|i| i.pointer("/urls/web").and_then(|w| w.as_str()).map(|s| s.to_string()));
        let Some(web) = web else { return Err(err(404, format!("实例 {iid} 不存在或尚未分配地址"))) };
        let sub = if sub.starts_with('?') { format!("/{sub}") } else { sub.to_string() };
        Ok(Mount::Reply(proxy(q, &web, &sub)?))
    });

    // ---- static front end
    let h3 = hub.clone();
    api.fallback = Some(Arc::new(move |q: &Request| static_file(&h3, q)));
    api
}

fn unix_stamp() -> String {
    let t = now() as i64;
    let (days, rem) = (t.div_euclid(86400), t.rem_euclid(86400));
    // civil-from-days (UTC)
    let z = days + 719468; let era = z.div_euclid(146097); let doe = z.rem_euclid(146097);
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365; let y = yoe + era * 400; let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153; let d = doy - (153 * mp + 2) / 5 + 1; let m = if mp < 10 { mp + 3 } else { mp - 9 }; let y = if m <= 2 { y + 1 } else { y };
    format!("{y:04}{m:02}{d:02}-{:02}{:02}{:02}", rem / 3600, rem % 3600 / 60, rem % 60)
}

fn main() {
    let env = |k: &str, d: &str| std::env::var(k).ok().filter(|v| !v.is_empty()).unwrap_or(d.to_string());
    let port: u16 = env("HUB_PORT", "8080").parse().expect("HUB_PORT");
    let data = env("HUB_DATA", "~/.agv-hub");
    let root = PathBuf::from(env("AGV_HOME", "."));
    let s = Arc::new(Store::new(&data).unwrap_or_else(|e| { eprintln!("[hub] 数据目录打开失败: {e}"); std::process::exit(2) }));
    let packages = Arc::new(PackageRepo { s: s.clone() });
    let nodes = Arc::new(NodeRegistry::new(s.clone(), packages.clone()));
    let models = Arc::new(ModelRepo { s: s.clone() });
    let scenes = Arc::new(SceneRepo { s: s.clone() });
    scenes.seed_builtins();
    let dep = Deployer::new(s.clone(), nodes.clone(), packages.clone(), models.clone(), scenes.clone(), port);
    let admin = admin::Admin::new(s.clone(), &root.to_string_lossy());
    let hub = Arc::new(Hub { port, root, s: s.clone(), packages, nodes, models, scenes, dep, admin, srv: OnceLock::new() });
    let mut api = build(hub.clone());
    api.host = env("HUB_BIND", "0.0.0.0");
    let api = Arc::new(api);
    let _ = hub.srv.set(api.clone());
    if let Some(t) = s.setting("cluster_token").and_then(|t| t.as_str().map(|s| s.to_string())) { let _ = std::fs::write(s.path(&["cluster_token"]), t); }
    println!("[hub] AMR 仿真资源管理平台 :{port}  数据 {}  集群令牌 {}", s.root.display(), s.setting("cluster_token").and_then(|t| t.as_str().map(|s| s.to_string())).unwrap_or_default());
    use std::io::Write;
    let _ = std::io::stdout().flush();
    if let Err(e) = api.serve_forever() { eprintln!("[hub] 监听失败: {e}"); std::process::exit(1); }
}
