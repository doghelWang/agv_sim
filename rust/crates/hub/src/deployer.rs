//! Simulation instance orchestration: deploy (sim + nav on agents), monitor, restart, stop.
use crate::models::ModelRepo;
use crate::nodes::*;
use crate::scenes::SceneRepo;
use crate::store::*;
use serde_json::{json, Map, Value};
use std::sync::{Arc, Mutex};
use std::time::Duration;

const STEPS: [(&str, &str); 8] = [("check", "资源校验"), ("ports", "分配端口"), ("images", "准备镜像"), ("sim", "启动仿真程序"), ("sim_ready", "仿真引擎就绪"), ("nav", "启动运行程序"), ("nav_ready", "执行进程就绪"), ("nav2", "Nav2 就绪")];
pub const ACTIVE: [&str; 4] = ["deploying", "running", "degraded", "stopping"];
fn fresh_steps() -> Value { json!(STEPS.iter().map(|(k, l)| json!({"key": k, "label": l, "status": "pending", "msg": ""})).collect::<Vec<_>>()) }
fn fnum(v: Option<&Value>) -> f64 { v.and_then(|x| x.as_f64()).unwrap_or(0.0) }

pub struct Deployer { pub s: Arc<Store>, pub nodes: Arc<NodeRegistry>, pub pk: Arc<PackageRepo>, pub models: Arc<ModelRepo>, pub scenes: Arc<SceneRepo>, pub hub_port: u16, lock: Mutex<()> }

impl Deployer {
    pub fn new(s: Arc<Store>, nodes: Arc<NodeRegistry>, pk: Arc<PackageRepo>, models: Arc<ModelRepo>, scenes: Arc<SceneRepo>, hub_port: u16) -> Arc<Deployer> {
        let d = Arc::new(Deployer { s, nodes, pk, models, scenes, hub_port, lock: Mutex::new(()) });
        let d2 = d.clone();
        std::thread::Builder::new().name("inst-monitor".into()).spawn(move || d2.monitor()).ok();
        d
    }
    pub fn get(&self, iid: &str) -> R<Value> { self.s.get("instances", iid).ok_or_else(|| err(404, format!("实例 {iid} 不存在"))) }
    pub fn list(&self, active_only: bool) -> Vec<Value> {
        let mut xs = self.s.list("instances");
        xs.sort_by(|a, b| fnum(b.get("created")).partial_cmp(&fnum(a.get("created"))).unwrap_or(std::cmp::Ordering::Equal));
        if active_only { xs.retain(|i| ACTIVE.contains(&sget(i, "status"))); }
        xs
    }
    fn new_iid(&self) -> String {
        let n = self.s.setting("instance_seq").and_then(|v| v.as_i64()).unwrap_or(0) + 1;
        self.s.set_setting("instance_seq", json!(n));
        format!("i{n:02}")
    }
    fn has_image(node: &Value, p: &Value) -> bool { node_has_image(node, p) || sget(p, "source") == "process" }

    pub fn check(&self, req: &Value) -> Value {
        let (mut issues, mut warns): (Vec<String>, Vec<String>) = (vec![], vec![]);
        let sim_node = self.nodes.get(sget(req, "sim_node")).ok();
        if sim_node.is_none() { issues.push("未选择仿真节点".into()); }
        let nav_id = if !sget(req, "nav_node").is_empty() { sget(req, "nav_node") } else { sget(req, "sim_node") };
        let nav_node = self.nodes.get(nav_id).ok();
        if nav_node.is_none() { issues.push("未选择执行节点".into()); }
        let active = self.list(true);
        for (role, n) in [("sim", &sim_node), ("nav", &nav_node)] {
            let Some(n) = n else { continue };
            let nm = sget(n, "name");
            if sget(n, "status") == "offline" { issues.push(format!("节点 {nm} 离线")); }
            if sget(n, "status") == "pending" && !warns.join("").contains(&format!("节点 {nm}")) { warns.push(format!("节点 {nm} 首次部署: 先通过 SSH 安装运行环境 (约 1 分钟)，目标设备需有 python3 与 Docker")); }
            if caps(n).get(role).and_then(|c| c.as_bool()) != Some(true) { issues.push(format!("节点 {nm} 类型为「{}」，不能运行{}", sget(n, "kind"), if role == "sim" { "仿真引擎" } else { "运行程序" })); }
            let used: Vec<&Value> = active.iter().filter(|i| sget(i, "sim_node") == sget(n, "id") || sget(i, "nav_node") == sget(n, "id")).collect();
            let maxi = n.get("max_instances").and_then(|m| m.as_i64()).unwrap_or(1) as usize;
            let same = matches!((&sim_node, &nav_node), (Some(a), Some(b)) if sget(a, "id") == sget(b, "id"));
            if used.len() >= maxi && !(role == "nav" && same) {
                let owners: Vec<String> = used.iter().map(|i| format!("{}({})", sget(i, "id"), if sget(i, "operator").is_empty() { "-" } else { sget(i, "operator") })).collect();
                issues.push(format!("节点 {nm} 已被实例 {} 占用 (上限 {maxi} 个)", owners.join(", ")));
            }
            let cpu = n.pointer("/info/cpu_percent").and_then(|c| c.as_f64()).unwrap_or(0.0);
            if cpu > 85.0 { warns.push(format!("节点 {nm} CPU 占用 {cpu}%")); }
        }
        for (key, kind, node) in [("sim_pkg", "sim", &sim_node), ("nav_pkg", "nav", &nav_node)] {
            let pid = sget(req, key);
            if pid.is_empty() { issues.push(format!("未选择{}", if kind == "sim" { "仿真程序包" } else { "运行程序包" })); continue; }
            let Ok(p) = self.pk.get(pid) else { issues.push(format!("程序包 {pid} 不存在")); continue };
            if sget(&p, "kind") != kind { issues.push(format!("程序包 {} 类型不符", sget(&p, "version"))); }
            if let Some(node) = node {
                if let Some(why) = self.pk.deployable_on(&p, node) { issues.push(why); }
                else if !Self::has_image(node, &p) && truthy(p.get("file")) { warns.push(format!("节点 {} 首次使用 {}，需要先分发镜像 ({:.0} MB)", sget(node, "name"), sget(&p, "version"), fnum(p.get("size")) / 1e6)); }
            }
        }
        if let (Ok(a), Ok(b)) = (self.pk.get(sget(req, "sim_pkg")), self.pk.get(sget(req, "nav_pkg"))) {
            let (x, y) = (a.get("api").and_then(|v| v.as_str()).unwrap_or("v1"), b.get("api").and_then(|v| v.as_str()).unwrap_or("v1"));
            if x != y { issues.push(format!("两个程序包协议版本不兼容: 仿真 {x} / 运行 {y}")); }
        }
        let model = self.models.get(sget(req, "model_id")).ok();
        match &model {
            Some(m) => { let miss = self.models.view(m).pointer("/audit/by_severity/missing").and_then(|v| v.as_i64()).unwrap_or(0); if miss > 0 { warns.push(format!("车辆模型有 {miss} 项参数缺失，可先到模型详情「补全与传感器安装」补齐")); } }
            None => issues.push("未选择车辆模型".into()),
        }
        let scene = self.scenes.get(sget(req, "scene_id")).ok();
        if scene.is_none() { issues.push("未选择仿真场景".into()); }
        if let (Some(m), Some(sc)) = (&model, &scene) {
            let w = m.pointer("/summary/width_m").and_then(|v| v.as_f64()).unwrap_or(0.0);
            if let Some(c) = sc.pointer("/summary/min_corridor_m").and_then(|v| v.as_f64()).filter(|c| *c != 0.0) {
                if w != 0.0 && w + 0.2 > c { warns.push(format!("车宽 {w:.2} m 接近场景最窄通道 {c:.2} m")); }
            }
        }
        let split = matches!((&sim_node, &nav_node), (Some(a), Some(b)) if sget(a, "id") != sget(b, "id"));
        json!({"ok": issues.is_empty(), "issues": issues, "warnings": warns, "split": split})
    }

    pub fn deploy(self: &Arc<Self>, req: &Value, operator: &str) -> R<Value> {
        let chk = self.check(req);
        if chk["ok"] != json!(true) { return Err(rest::ApiError::coded(409, chk["issues"].as_array().unwrap().iter().filter_map(|x| x.as_str()).collect::<Vec<_>>().join("；"), "check_failed")); }
        let iid = self.new_iid();
        let m = self.models.get(sget(req, "model_id"))?;
        let sc = self.scenes.get(sget(req, "scene_id"))?;
        let name = if truthy(req.get("name")) { req["name"].clone() } else { json!(format!("{} @ {}", sget(&m, "name"), sget(&sc, "name"))) };
        let nav_node = if truthy(req.get("nav_node")) { req["nav_node"].clone() } else { req["sim_node"].clone() };
        let inst = json!({"id": iid, "name": name, "operator": if !operator.is_empty() { json!(operator) } else { req.get("operator").cloned().unwrap_or(json!("")) },
            "sim_node": req["sim_node"], "nav_node": nav_node, "model_id": m["id"], "model_ver": if truthy(req.get("model_ver")) { req["model_ver"].clone() } else { m.get("latest").cloned().unwrap_or(Value::Null) },
            "model_name": m.get("name").cloned().unwrap_or(Value::Null), "scene_id": sc["id"], "scene_name": sc.get("name").cloned().unwrap_or(Value::Null), "sim_pkg": req["sim_pkg"], "nav_pkg": req["nav_pkg"],
            "options": if truthy(req.get("options")) { req["options"].clone() } else { json!({}) }, "status": "deploying", "created": now(), "started": null, "steps": fresh_steps(), "warnings": chk["warnings"]});
        let inst = self.s.put("instances", inst);
        let d = self.clone();
        let i2 = iid.clone();
        std::thread::Builder::new().name(format!("deploy-{iid}")).spawn(move || d.run_deploy(&i2)).ok();
        Ok(inst)
    }

    fn step(&self, iid: &str, key: &str, status: &str, msg: &str, progress: Option<i64>) {
        let _g = self.lock.lock().unwrap_or_else(|e| e.into_inner());
        self.s.update("instances", iid, |i| {
            if let Some(steps) = i.get_mut("steps").and_then(|s| s.as_array_mut()) {
                for st in steps { if sget(st, "key") == key {
                    if status == "running" && sget(st, "status") != "running" { st["t0"] = json!(now()); }
                    if matches!(status, "done" | "error" | "skipped") { st["t1"] = json!(now()); }
                    st["status"] = json!(status); st["msg"] = json!(msg);
                    if let Some(p) = progress { st["progress"] = json!(p); }
                } }
            }
        });
    }
    fn set(&self, iid: &str, f: impl FnOnce(&mut Value)) { let _g = self.lock.lock().unwrap_or_else(|e| e.into_inner()); self.s.update("instances", iid, f); }

    fn run_deploy(self: &Arc<Self>, iid: &str) {
        let mut started: Vec<(AgentClient, String)> = vec![];
        if let Err(msg) = self.do_deploy(iid, &mut started) {
            eprintln!("[hub] 部署 {iid} 失败: {msg}");
            { let _g = self.lock.lock().unwrap_or_else(|e| e.into_inner());
              self.s.update("instances", iid, |i| {
                if let Some(steps) = i.get_mut("steps").and_then(|s| s.as_array_mut()) { for st in steps { if sget(st, "status") == "running" { st["status"] = json!("error"); st["msg"] = json!(msg); st["t1"] = json!(now()); } } }
                i["status"] = json!("error"); i["error"] = json!(msg); i["ended"] = json!(now());
              }); }
            for (ag, name) in started { let _ = ag.call("DELETE", &format!("/api/v1/containers/{name}"), None, Some(40.0)); }
        }
    }

    fn do_deploy(self: &Arc<Self>, iid: &str, started: &mut Vec<(AgentClient, String)>) -> Result<(), String> {
        let i = self.get(iid).map_err(|e| e.message)?;
        let (sid_n, nid_n) = (sget(&i, "sim_node").to_string(), sget(&i, "nav_node").to_string());
        self.step(iid, "check", "running", "", None);
        for nid in if sid_n == nid_n { vec![sid_n.clone()] } else { vec![sid_n.clone(), nid_n.clone()] } {
            if sget(&self.nodes.get(&nid).map_err(|e| e.message)?, "status") == "pending" { return Err("节点尚未安装运行环境: Rust 平台暂不支持 SSH 首次部署，请先在设备上启动节点代理".into()); }
        }
        let sim_n = self.nodes.get(&sid_n).map_err(|e| e.message)?;
        let nav_n = self.nodes.get(&nid_n).map_err(|e| e.message)?;
        let (ag_s, ag_n) = (self.nodes.client_of(&sim_n), self.nodes.client_of(&nav_n));
        let sim_p = self.pk.get(sget(&i, "sim_pkg")).map_err(|e| e.message)?;
        let nav_p = self.pk.get(sget(&i, "nav_pkg")).map_err(|e| e.message)?;
        ag_s.call("GET", "/api/v1/health", None, Some(5.0))?;
        ag_n.call("GET", "/api/v1/health", None, Some(5.0))?;
        let mut enabled: Vec<String> = vec![];
        let mut seen: Vec<String> = vec![];
        for (node, ag) in [(&sim_n, &ag_s), (&nav_n, &ag_n)] {
            if seen.contains(&sget(node, "id").to_string()) { continue; }
            seen.push(sget(node, "id").to_string());
            if node.pointer("/info/role").and_then(|r| r.as_str()) == Some("monitor") {
                self.step(iid, "check", "running", &format!("节点 {} 首次部署，启用运行环境…", sget(node, "name")), None);
                let r = ag.call("POST", "/api/v1/admin/role", Some(&json!({"role": "full"})), Some(15.0))?;
                let mut info = node.get("info").cloned().unwrap_or(json!({}));
                if let Some(ri) = r.get("info").and_then(|x| x.as_object()) { for (k, v) in ri { info[k] = v.clone(); } }
                info["role"] = json!("full");
                self.s.update("nodes", sget(node, "id"), |n| n["info"] = info);
                enabled.push(sget(node, "name").to_string());
            }
        }
        self.step(iid, "check", "done", &format!("仿真 {} · 执行 {}{}", sget(&sim_n, "name"), sget(&nav_n, "name"), if enabled.is_empty() { String::new() } else { format!(" · 已启用运行环境: {}", enabled.join("、")) }), None);

        self.step(iid, "ports", "running", "", None);
        let same = sid_n == nid_n;
        let ports: Map<String, Value> = if same {
            ag_s.call("POST", "/api/v1/ports/allocate", Some(&json!({"instance": iid, "names": ["sim_api", "web", "nav_api"]})), None)?["ports"].as_object().cloned().unwrap_or_default()
        } else {
            let mut p = ag_s.call("POST", "/api/v1/ports/allocate", Some(&json!({"instance": iid, "names": ["sim_api", "web"]})), None)?["ports"].as_object().cloned().unwrap_or_default();
            for (k, v) in ag_n.call("POST", "/api/v1/ports/allocate", Some(&json!({"instance": iid, "names": ["nav_api"]})), None)?["ports"].as_object().cloned().unwrap_or_default() { p.insert(k, v); }
            p
        };
        let pn = |k: &str| ports.get(k).and_then(|p| p.as_i64()).unwrap_or(0);
        let dom = self.ros_domain(&nid_n, iid);
        let (hs, hn) = (sget(&sim_n, "host").to_string(), sget(&nav_n, "host").to_string());
        let urls = json!({"sim_api": format!("http://{hs}:{}", pn("sim_api")), "web": format!("http://{hs}:{}", pn("web")), "nav_api": format!("http://{hn}:{}", pn("nav_api"))});
        { let (p2, u2) = (Value::Object(ports.clone()), urls.clone()); self.set(iid, move |i| { i["ports"] = p2; i["urls"] = u2; i["ros_domain"] = json!(dom); }); }
        self.step(iid, "ports", "done", &format!("仿真 {} · Web {} · 执行 {}", pn("sim_api"), pn("web"), pn("nav_api")), None);

        self.step(iid, "images", "running", "", None);
        for (node, ag, p) in [(&sim_n, &ag_s, &sim_p), (&nav_n, &ag_n, &nav_p)] { self.ensure_image(iid, node, ag, p)?; }
        self.step(iid, "images", "done", &format!("{} / {}", sget(&sim_p, "version"), sget(&nav_p, "version")), None);

        self.step(iid, "sim", "running", "", None);
        let nav_host = self.nodes.addr_between(&sim_n, &nav_n);
        let sim_host_for_nav = self.nodes.addr_between(&nav_n, &sim_n);
        let hub_url = if truthy(sim_n.get("hub_url")) { sget(&sim_n, "hub_url").to_string() } else { format!("http://127.0.0.1:{}", self.hub_port) };
        let opts = i.get("options").cloned().unwrap_or(json!({}));
        let mut env: Map<String, Value> = Map::new();
        for (k, v) in [("SIM_API_PORT", json!(pn("sim_api"))), ("WEB_PORT", json!(pn("web"))), ("NAV_API", json!(format!("http://{nav_host}:{}", pn("nav_api")))), ("HUB_API", json!(hub_url)), ("INSTANCE_ID", json!(iid)),
            ("MODEL_ID", i["model_id"].clone()), ("MODEL_VER", i["model_ver"].clone()), ("SCENE_ID", i["scene_id"].clone()), ("INSTANCE_NAME", i["name"].clone()),
            ("SIM_CAMERA_RENDER", json!(opts.get("camera_render").and_then(|c| c.as_str()).unwrap_or("ray")))] { env.insert(k.into(), v); }
        let solo = same && hs.starts_with("127.") && sim_n.pointer("/info/runtime/runtime").and_then(|r| r.as_str()) == Some("process");
        if solo { env.insert("WEB_BIND".into(), json!("127.0.0.1")); env.insert("AGV_BIND".into(), json!("127.0.0.1")); }
        if let Some(o) = opts.get("env_sim").and_then(|e| e.as_object()) { for (k, v) in o { env.insert(k.clone(), v.clone()); } }
        let (cname_s, cname_n) = (format!("agv-sim-{iid}"), format!("agv-nav-{iid}"));
        let strenv = |e: &Map<String, Value>| -> Value { Value::Object(e.iter().map(|(k, v)| (k.clone(), json!(match v { Value::String(s) => s.clone(), o => o.to_string() }))).collect()) };
        ag_s.call("POST", "/api/v1/containers/run", Some(&json!({"name": cname_s, "image": sim_p["image_ref"], "role": "sim", "instance": iid, "env": strenv(&env)})), Some(60.0))?;
        started.push((self.nodes.client_of(&sim_n), cname_s.clone()));
        { let c = json!({"sim": cname_s, "nav": cname_n}); self.set(iid, move |i| i["containers"] = c); }
        self.step(iid, "sim", "done", &cname_s, None);

        self.step(iid, "sim_ready", "running", "", None);
        let u = |k: &str| urls[k].as_str().unwrap_or("").to_string();
        self.wait(iid, "sim_ready", &format!("{}/api/v1/health", u("sim_api")), 120.0, &ag_s, &cname_s)?;
        self.wait(iid, "sim_ready", &format!("{}/api/v2/info", u("web")), 60.0, &ag_s, &cname_s)?;
        self.step(iid, "sim_ready", "done", "模型/场景已加载", None);

        self.step(iid, "nav", "running", "", None);
        let mut env_n: Map<String, Value> = Map::new();
        for (k, v) in [("NAV_API_PORT", json!(pn("nav_api"))), ("SIM_API", json!(format!("http://{sim_host_for_nav}:{}", pn("sim_api")))), ("ROS_DOMAIN_ID", json!(dom)), ("INSTANCE_ID", json!(iid)),
            ("NAV_USE_ROS", json!(if truthy(opts.get("no_ros")) { "0" } else { "1" })), ("NAV_LOCALIZATION", json!(opts.get("localization").and_then(|c| c.as_str()).unwrap_or("slam")))] { env_n.insert(k.into(), v); }
        if solo { env_n.insert("AGV_BIND".into(), json!("127.0.0.1")); }
        if let Some(o) = opts.get("env_nav").and_then(|e| e.as_object()) { for (k, v) in o { env_n.insert(k.clone(), v.clone()); } }
        ag_n.call("POST", "/api/v1/containers/run", Some(&json!({"name": cname_n, "image": nav_p["image_ref"], "role": "nav", "instance": iid, "env": strenv(&env_n)})), Some(60.0))?;
        started.push((self.nodes.client_of(&nav_n), cname_n.clone()));
        self.step(iid, "nav", "done", &cname_n, None);
        self.step(iid, "nav_ready", "running", "", None);
        let h = self.wait(iid, "nav_ready", &format!("{}/api/v1/health", u("nav_api")), 180.0, &ag_n, &cname_n)?;
        self.step(iid, "nav_ready", "done", if truthy(h.get("sim_link")) { "已连接仿真进程" } else { "" }, None);

        self.step(iid, "nav2", "running", "", None);
        if !truthy(h.get("ros")) { self.step(iid, "nav2", "skipped", "未启用 ROS 2 (内置导引)", None); }
        else {
            let t0 = now();
            let mut ok = false;
            while now() - t0 < 150.0 {
                if http_json(&format!("{}/api/v1/health", u("nav_api")), 3.0, "GET", None).map(|v| truthy(v.get("nav2_ready"))).unwrap_or(false) { ok = true; break; }
                self.step(iid, "nav2", "running", &format!("等待 Nav2 生命周期节点 {} s", (now() - t0) as i64), None);
                std::thread::sleep(Duration::from_secs(2));
            }
            self.step(iid, "nav2", if ok { "done" } else { "skipped" }, if ok { "" } else { "Nav2 未在 150 s 内就绪，可先用内置规划器" }, None);
        }
        self.set(iid, |i| { i["status"] = json!("running"); i["started"] = json!(now()); i["error"] = Value::Null; });
        Ok(())
    }

    fn ros_domain(&self, node_id: &str, iid: &str) -> i64 {
        let used: Vec<i64> = self.list(true).iter().filter(|i| sget(i, "nav_node") == node_id && sget(i, "id") != iid).filter_map(|i| i.get("ros_domain").and_then(|d| d.as_i64())).collect();
        let mut d = 10;
        while used.contains(&d) { d += 1; }
        d
    }

    fn ensure_image(&self, iid: &str, node: &Value, ag: &AgentClient, p: &Value) -> Result<(), String> {
        if Self::has_image(node, p) { return Ok(()); }
        let url = if truthy(p.get("file")) { let hub = if truthy(node.get("hub_url")) { sget(node, "hub_url").to_string() } else { format!("http://127.0.0.1:{}", self.hub_port) }; json!(format!("{hub}/api/hub/packages/{}/image", sget(p, "id"))) } else { Value::Null };
        let mut job = ag.call("POST", "/api/v1/images/ensure", Some(&json!({"ref": p["image_ref"], "package_url": url})), None)?;
        while sget(&job, "status") == "running" {
            std::thread::sleep(Duration::from_secs(1));
            job = ag.call("GET", &format!("/api/v1/jobs/{}", sget(&job, "id")), None, None)?;
            let tot = job.get("total").and_then(|t| t.as_f64()).filter(|t| *t != 0.0).unwrap_or(fnum(p.get("size")));
            let by = fnum(job.get("bytes"));
            let pct = if tot != 0.0 { Some((100.0 * by / tot).round() as i64) } else { None };
            self.step(iid, "images", "running", &format!("{}: 导入 {} {:.0}/{:.0} MB", sget(node, "name"), sget(p, "version"), by / 1e6, tot / 1e6), pct);
        }
        if sget(&job, "status") != "done" { return Err(format!("节点 {} 导入镜像失败: {}", sget(node, "name"), sget(&job, "message"))); }
        Ok(())
    }

    fn wait(&self, iid: &str, key: &str, url: &str, timeout: f64, ag: &AgentClient, cname: &str) -> Result<Value, String> {
        let t0 = now();
        let mut last = String::new();
        while now() - t0 < timeout {
            match http_json(url, 3.0, "GET", None) { Ok(v) => return Ok(v), Err(e) => last = e }
            if let Ok(st) = ag.call("GET", &format!("/api/v1/containers/{cname}"), None, Some(5.0)) {
                let exit = st.get("exit_code").filter(|e| !e.is_null()).and_then(|e| e.as_i64());
                if st.get("running") != Some(&json!(true)) && (matches!(exit, Some(c) if c != 0) || matches!(sget(&st, "state"), "exited" | "dead")) {
                    let logs = ag.call("GET", &format!("/api/v1/containers/{cname}/logs?tail=15"), None, Some(8.0)).ok().and_then(|l| l.get("logs").and_then(|x| x.as_str()).map(|s| s.to_string())).unwrap_or_default();
                    let tail: String = logs.chars().rev().take(800).collect::<Vec<_>>().into_iter().rev().collect();
                    return Err(format!("{cname} 已退出 (exit {})\n{tail}", exit.map(|c| c.to_string()).unwrap_or("None".into())));
                }
            }
            self.step(iid, key, "running", &format!("等待 {} s", (now() - t0) as i64), None);
            std::thread::sleep(Duration::from_millis(1500));
        }
        Err(format!("{url} 在 {timeout} s 内未就绪: {last}"))
    }

    pub fn stop(&self, iid: &str) -> R<Value> {
        let i = self.get(iid)?;
        if truthy(i.get("external")) { self.set(iid, |i| { i["status"] = json!("stopped"); i["ended"] = json!(now()); }); return self.get(iid); }
        self.set(iid, |i| i["status"] = json!("stopping"));
        if let Some(web) = i.pointer("/urls/web").and_then(|w| w.as_str()) { let _ = http_json(&format!("{web}/api/v2/records/archive"), 10.0, "POST", Some(&json!({}))); }
        let mut errs: Vec<String> = vec![];
        for (role, nid) in [("nav", sget(&i, "nav_node")), ("sim", sget(&i, "sim_node"))] {
            let name = i.pointer(&format!("/containers/{role}")).and_then(|n| n.as_str()).unwrap_or("");
            if name.is_empty() || nid.is_empty() { continue; }
            match self.nodes.client(nid) { Ok(ag) => if let Err(e) = ag.call("DELETE", &format!("/api/v1/containers/{name}"), None, Some(40.0)) { errs.push(e); }, Err(e) => errs.push(e.message) }
        }
        let mut seen: Vec<&str> = vec![];
        for nid in [sget(&i, "sim_node"), sget(&i, "nav_node")] { if seen.contains(&nid) { continue; } seen.push(nid); if let Ok(ag) = self.nodes.client(nid) { let _ = ag.call("DELETE", &format!("/api/v1/instances/{iid}"), None, Some(10.0)); } }
        let e = errs.join("；");
        self.set(iid, move |i| { i["status"] = json!("stopped"); i["ended"] = json!(now()); i["error"] = if e.is_empty() { Value::Null } else { json!(e) }; });
        self.get(iid)
    }

    pub fn restart(self: &Arc<Self>, iid: &str) -> R<Value> {
        let i = self.get(iid)?;
        if truthy(i.get("external")) { return Err(err(400, "外部接入的实例不能由平台重启")); }
        for (role, nid) in [("nav", sget(&i, "nav_node")), ("sim", sget(&i, "sim_node"))] {
            let name = i.pointer(&format!("/containers/{role}")).and_then(|n| n.as_str()).unwrap_or("");
            if !name.is_empty() { if let Ok(ag) = self.nodes.client(nid) { let _ = ag.call("DELETE", &format!("/api/v1/containers/{name}"), None, Some(40.0)); } }
        }
        let i = self.s.update("instances", iid, |i| { i["status"] = json!("deploying"); i["error"] = Value::Null; i["steps"] = fresh_steps(); }).ok_or_else(|| err(404, "实例不存在"))?;
        let d = self.clone();
        let i2 = iid.to_string();
        std::thread::spawn(move || d.run_deploy(&i2));
        Ok(i)
    }

    pub fn delete(&self, iid: &str) -> R<()> {
        let i = self.get(iid)?;
        if ACTIVE.contains(&sget(&i, "status")) && !truthy(i.get("external")) { return Err(err(409, "请先终止仿真")); }
        self.s.delete("instances", iid);
        Ok(())
    }

    pub fn attach(&self, body: &Value, operator: &str) -> R<Value> {
        let web = sget(body, "web_url").trim_end_matches('/').to_string();
        if !web.starts_with("http") { return Err(err(400, "需要 Web 网关地址，例如 http://192.168.1.10:8088")); }
        let info = match http_json(&format!("{web}/api/v2/info"), 4.0, "GET", None) {
            Ok(i) => i,
            Err(_) => { http_json(&format!("{web}/api/telemetry"), 4.0, "GET", None).map_err(|e| err(400, format!("无法访问 {web}: {e}")))?; json!({}) }
        };
        let iid = self.new_iid();
        let inst = json!({"id": iid, "name": if truthy(body.get("name")) { body["name"].clone() } else { json!(format!("外部实例 {web}")) }, "external": true, "operator": operator,
            "urls": {"web": web, "sim_api": info.get("sim_api").cloned().unwrap_or(Value::Null), "nav_api": info.get("nav_api").cloned().unwrap_or(Value::Null)}, "status": "running", "created": now(), "started": now(),
            "model_name": info.pointer("/model/name").cloned().unwrap_or(Value::Null), "scene_name": info.pointer("/scene/name").cloned().unwrap_or(Value::Null), "steps": []});
        Ok(self.s.put("instances", inst))
    }

    pub fn logs(&self, iid: &str, svc: &str, tail: i64) -> R<Value> {
        let i = self.get(iid)?;
        let name = i.pointer(&format!("/containers/{svc}")).and_then(|n| n.as_str()).unwrap_or("");
        let nid = if svc == "sim" { sget(&i, "sim_node") } else { sget(&i, "nav_node") };
        if name.is_empty() || nid.is_empty() { return Err(err(404, "该实例没有此服务容器")); }
        self.nodes.client(nid)?.call("GET", &format!("/api/v1/containers/{name}/logs?tail={tail}"), None, Some(15.0)).map_err(|e| err(502, e))
    }

    fn monitor(self: Arc<Self>) {
        loop {
            std::thread::sleep(Duration::from_secs(4));
            for i in self.list(true) {
                if !matches!(sget(&i, "status"), "running" | "degraded") { continue; }
                self.probe(&i);
            }
        }
    }
    fn probe(&self, i: &Value) {
        let Some(web) = i.pointer("/urls/web").and_then(|w| w.as_str()) else { return };
        let (mut web_ok, mut sim_ok, mut nav_ok, mut brief) = (false, false, false, None);
        match http_json(&format!("{web}/api/v2/brief"), 2.5, "GET", None) {
            Ok(b) => { web_ok = true; sim_ok = b.get("sim_online").map(|v| truthy(Some(v))).unwrap_or(true); nav_ok = b.get("nav_online").map(|v| truthy(Some(v))).unwrap_or(true); brief = Some(b); }
            Err(_) => if http_json(&format!("{web}/api/telemetry"), 2.5, "GET", None).is_ok() { web_ok = true; sim_ok = true; nav_ok = true; },
        }
        let st = if web_ok && sim_ok && nav_ok { "running" } else { "degraded" };
        let health = json!({"web": web_ok, "sim": sim_ok, "nav": nav_ok, "t": now()});
        let _g = self.lock.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(cur) = self.s.get("instances", sget(i, "id")) {
            if matches!(sget(&cur, "status"), "running" | "degraded") {
                let id = sget(i, "id").to_string();
                self.s.update("instances", &id, |c| { c["status"] = json!(st); c["health"] = health; if let Some(b) = brief { c["brief"] = b; } });
            }
        }
    }
}
