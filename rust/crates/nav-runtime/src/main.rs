mod events;
mod guidance;
mod navigator;
mod simlink;
mod taskflow;

use navigator::Navigator;
use rest::{ok, ApiError, Request, Server};
use serde_json::{json, Map, Value};
use simlink::SimLink;
use std::sync::Arc;

const SCENARIOS: &str = include_str!("../../../data/scenarios.json");

extern "C" fn on_term(_: libc::c_int) { unsafe { libc::_exit(0) } }

fn f(b: &Value, k: &str, d: f64) -> Result<f64, ApiError> {
    match b.get(k) {
        None | Some(Value::Null) => Ok(d),
        Some(v) => v.as_f64().or_else(|| v.as_str().and_then(|s| s.parse().ok())).ok_or_else(|| ApiError::new(400, "需要 x, y (可选 yaw) 或 station")),
    }
}

fn build_api(nav: Arc<Navigator>, link: Arc<SimLink>, port: u16) -> Server {
    let mut api = Server::new("nav", port, "执行进程 nav_runtime", "定位、拓扑路径规划、导航任务、任务流、安全防护、遥控 (Rust)");
    {
        let (l, n) = (link.clone(), nav.clone());
        api.route("GET", "/api/v1/health", "健康检查", move |_| {
            let _ = &n;
            ok(json!({"service": "nav", "ok": true, "sim_link": l.is_online(), "ros": false, "nav2_ready": false}))
        });
    }
    {
        let (l, n) = (link.clone(), nav.clone());
        api.route("GET", "/api/v1/nav", "执行状态", move |_| {
            let mut fb = n.feedback_json();
            let mut lk = json!({"sim_url": l.url, "online": l.is_online()});
            if let Some(o) = l.stats.lock().unwrap().as_object() { for (k, v) in o { lk[k] = v.clone(); } }
            fb["link"] = lk;
            fb["ros"] = Value::Null;
            fb["ros_bridge"] = Value::Null;
            ok(fb)
        });
    }
    {
        let n = nav.clone();
        api.route("PUT", "/api/v1/nav/planner", "切换规划器", move |q| {
            let b = q.json()?;
            let t = b["type"].as_str().unwrap_or("None").to_string();
            if !n.set_planner_type(&t) { return Err(ApiError::new(400, format!("规划器不可用: {t}"))); }
            ok(json!({"planner": n.s().active_planner}))
        });
    }
    { let n = nav.clone(); api.route("GET", "/api/v1/missions", "最近任务", move |_| ok(json!({"missions": n.missions_view()}))); }
    {
        let (n, l) = (nav.clone(), link.clone());
        api.route("POST", "/api/v1/missions", "下发任务", move |q| {
            let b = q.json()?;
            let (x, y, yaw);
            if let Some(sid) = b.get("station").and_then(|s| s.as_str()) {
                let w = l.world();
                let st = w["stations"].as_array().and_then(|a| a.iter().find(|s| s["id"].as_str() == Some(sid)).cloned())
                    .ok_or_else(|| ApiError::new(404, format!("未知工位 {sid}")))?;
                x = st["x"].as_f64().unwrap_or(0.0); y = st["y"].as_f64().unwrap_or(0.0); yaw = st.get("dock_yaw").and_then(|v| v.as_f64()).unwrap_or(0.0);
            } else {
                if b.get("x").is_none() || b.get("y").is_none() { return Err(ApiError::new(400, "需要 x, y (可选 yaw) 或 station")); }
                x = f(&b, "x", 0.0)?; y = f(&b, "y", 0.0)?; yaw = f(&b, "yaw", 0.0)?;
            }
            match n.submit(x, y, yaw, b.get("planner").and_then(|p| p.as_str())) {
                Ok(v) => ok(v.unwrap_or(Value::Null)),
                Err(e) => Err(ApiError::new(400, e)),
            }
        });
    }
    {
        let n = nav.clone();
        api.route("GET", "/api/v1/missions/current", "当前任务", move |_| n.mission_view(None).map(|v| rest::Response::json(&v)).ok_or_else(|| ApiError::new(404, "无任务")));
    }
    {
        let n = nav.clone();
        api.route("GET", "/api/v1/missions/{id}", "任务详情", move |q| {
            let id = q.params.get("id").cloned().unwrap_or_default();
            id.parse::<u64>().ok().and_then(|i| n.mission_view(Some(i))).map(|v| rest::Response::json(&v)).ok_or_else(|| ApiError::new(404, format!("无任务 {id}")))
        });
    }
    for p in ["/api/v1/missions/current", "/api/v1/missions/{id}"] {
        let n = nav.clone();
        api.route("DELETE", p, "取消任务", move |_| { n.cancel_nav(); ok(json!({"canceled": true})) });
    }
    {
        let l = link.clone();
        api.route("POST", "/api/v1/teleop", "手动遥控", move |q| {
            let b = q.json()?;
            ok(json!({"forwarded": l.send_cmd(f(&b, "vx", 0.0)?, f(&b, "vy", 0.0)?, f(&b, "wz", 0.0)?, "teleop")}))
        });
    }
    let tf = taskflow::TaskFlow::new(&nav);
    let _ = nav.taskflow.set(tf.clone());
    {
        let tf = tf.clone();
        api.route("POST", "/api/v1/taskflows/run", "执行任务流", move |q| {
            let b = q.json()?;
            let flow = b.get("flow").filter(|v| v.is_object()).cloned().unwrap_or(b);
            tf.run(&flow).map(|v| rest::Response::json(&v)).map_err(|e| ApiError::new(400, e))
        });
    }
    {
        let tf = tf.clone();
        api.route("POST", "/api/v1/taskflows/stop", "终止任务流", move |q| {
            let b = q.json().unwrap_or(json!({}));
            ok(tf.stop(false, b["reason"].as_str().unwrap_or("操作员终止")))
        });
    }
    { let tf = tf.clone(); api.route("GET", "/api/v1/taskflows/current", "任务流进度", move |_| ok(tf.view())); }
    {
        let n = nav.clone();
        api.route("POST", "/api/v1/plan", "路线预览 (拓扑规划，不下发)", move |q| {
            let b = q.json()?;
            let pts: Vec<(f64, f64)> = b["points"].as_array().cloned().unwrap_or_default().iter()
                .map(|p| (p["x"].as_f64().unwrap_or(0.0), p["y"].as_f64().unwrap_or(0.0))).collect();
            let obs = if b.get("obstacles").map(|v| !v.is_null() && *v != json!(false) && *v != json!(0)).unwrap_or(false) { n.dyn_obstacles() } else { vec![] };
            let mut legs = vec![];
            let mut total = 0.0;
            for w in pts.windows(2) {
                let r = n.pl().plan_route(w[0], w[1], &obs);
                let r3 = |x: f64| (x * 1000.0).round_ties_even() / 1000.0;
                total += r.length;
                legs.push(json!({"points": r.points.iter().map(|p| json!({"x": r3(p.0), "y": r3(p.1)})).collect::<Vec<_>>(), "labels": r.labels, "length": r.length}));
            }
            ok(json!({"legs": legs, "length": (total * 100.0).round_ties_even() / 100.0}))
        });
    }
    {
        let n = nav.clone();
        api.route("GET", "/api/v1/safety/params", "保护空间 / 避障参数", move |_| {
            let mut v = n.safety_params();
            v["protection_view"] = n.protection_view();
            ok(v)
        });
    }
    {
        let n = nav.clone();
        api.route("PUT", "/api/v1/safety/params", "运行时调整保护空间", move |q| {
            let b = q.json().unwrap_or(json!({}));
            let mut patch = b.get("protection").and_then(|p| p.as_object().cloned()).unwrap_or_default();
            if let Some(e) = b.get("enabled") { patch.insert("enabled".into(), json!(e.as_bool().unwrap_or(false))); }
            for k in ["slow_speed", "corner_radius"] {
                if let Some(v) = b.get(k).and_then(|v| v.as_f64()) { patch.insert(k.into(), json!(v.max(0.0))); }
            }
            if ["stop_dist", "corridor_margin", "slow_dist"].iter().any(|k| b.get(*k).is_some()) {
                let mut fs: Vec<Value> = n.s().prot.json.get("fields").and_then(|f| f.as_array().cloned()).unwrap_or_default();
                let sd = b.get("stop_dist").and_then(|v| v.as_f64());
                for (i, fl) in fs.iter_mut().enumerate() {
                    if let Some(sd) = sd {
                        let cur = fl["front"].as_f64().unwrap_or(0.0);
                        fl["front"] = json!(if i == 0 { sd } else { cur.max(sd) });
                    }
                    if let Some(cm) = b.get("corridor_margin").and_then(|v| v.as_f64()) { fl["side"] = json!(cm); }
                }
                patch.insert("fields".into(), Value::Array(fs));
                if let (Some(sl), Some(sd)) = (b.get("slow_dist").and_then(|v| v.as_f64()), sd) { if sd > 0.0 { patch.insert("slow_ratio".into(), json!((sl / sd).max(1.0))); } }
            }
            let view = n.set_protection(&Value::Object(patch));
            let (txt, bm) = {
                let s = n.s();
                let fs = s.prot.json.get("fields").and_then(|f| f.as_array().cloned()).unwrap_or_default();
                let g = |x: &Value, k: &str| x.get(k).map(|v| v.to_string().trim_matches('"').to_string()).unwrap_or_default();
                (fs.iter().map(|x| format!("{}≤{}: 前{} 后{} 侧{}", g(x, "name"), g(x, "v_max"), g(x, "front"), g(x, "rear"), g(x, "side"))).collect::<Vec<_>>().join(" / "), s.prot.body_margin)
            };
            n.ev("safety", "SAFETY_PARAMS", "info", "保护空间参数更新 (本次运行)", &format!("防护区 {txt}；车体净空 {bm} m"), json!({}));
            let mut v = n.safety_params();
            v["protection_view"] = view;
            ok(v)
        });
    }
    {
        let n = nav.clone();
        api.route("GET", "/api/v1/slam", "定位状态 (纯导引模式: 真值)", move |_| { let mut v = n.localization(); v["note"] = json!("Rust nav-runtime: 真值定位，无 SLAM"); ok(v) });
    }
    {
        let n = nav.clone();
        api.route("GET", "/api/v1/events", "执行事件", move |q| ok(n.events.get(q.q_i64("since", 0)?.max(0) as u64, q.q_i64("limit", 200)?.max(0) as usize)));
    }
    api.route("GET", "/api/v1/ros", "ROS 图", |_| ok(json!({"nodes": [], "topics": [], "note": "ROS 未启用"})));
    api
}

fn main() {
    let sim_url = std::env::var("SIM_API").ok().filter(|v| !v.is_empty()).unwrap_or("http://127.0.0.1:8090".into());
    let port: u16 = std::env::var("NAV_API_PORT").ok().and_then(|v| v.parse().ok()).unwrap_or(8091);
    unsafe { libc::signal(libc::SIGTERM, on_term as usize); }
    let scen: Map<String, Value> = serde_json::from_str::<Value>(SCENARIOS).unwrap().as_object().cloned().unwrap();
    let link = SimLink::new(&sim_url);
    println!("[nav_runtime] 连接仿真进程 {sim_url} ...");
    let c = rest::Client::new(&sim_url, 2.0);
    let t0 = std::time::Instant::now();
    while c.get("/api/v1/health").is_err() {
        if t0.elapsed().as_secs() > 120 { eprintln!("[nav_runtime] 仿真进程不可达"); std::process::exit(2); }
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    let nav = Navigator::new(link.clone(), &scen);
    link.start();
    std::thread::sleep(std::time::Duration::from_secs(1));
    let api = Arc::new(build_api(nav, link, port));
    println!("[nav_runtime] REST API http://0.0.0.0:{port}/api/v1 (Rust, 纯导引模式)");
    use std::io::Write;
    let _ = std::io::stdout().flush();
    if let Err(e) = api.serve_forever() { eprintln!("[nav_runtime] 监听失败: {e}"); std::process::exit(1); }
}
