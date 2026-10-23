mod api;
mod scen;
mod service;

use serde_json::{Map, Value};
use std::sync::Arc;

const SCENARIOS: &str = include_str!("../../../data/scenarios.json");

fn arg(args: &[String], k: &str) -> Option<String> { args.iter().position(|a| a == k).and_then(|i| args.get(i + 1)).cloned() }

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let env = |k: &str, d: &str| std::env::var(k).ok().filter(|v| !v.is_empty()).unwrap_or(d.to_string());
    let config = arg(&args, "--config").unwrap_or_else(|| env("ROBOT_CONFIG", "robot_config.json"));
    let port: u16 = arg(&args, "--port").unwrap_or_else(|| env("SIM_API_PORT", "8090")).parse().expect("port");
    let mut scenario = arg(&args, "--scenario").unwrap_or_else(|| env("SIM_SCENARIO", "grid_9_square"));
    let mut scenarios: Map<String, Value> = serde_json::from_str::<Value>(SCENARIOS).unwrap().as_object().cloned().unwrap();
    if let Ok(sf) = std::env::var("SIM_SCENE_FILE") {
        if let Ok(t) = std::fs::read_to_string(&sf) {
            match serde_json::from_str::<Value>(&t).map_err(|e| e.to_string()).and_then(|v| service::register_scenario(&mut scenarios, &v)) {
                Ok(id) => scenario = id,
                Err(e) => eprintln!("[sim_server] 场景文件 {sf} 无效: {e}"),
            }
        }
    }
    let lib = match mujoco_sys::load(None) { Ok(l) => Some(Arc::new(l)), Err(e) => { eprintln!("[sim_server] MuJoCo 不可用，使用运动学兜底: {e}"); None } };
    let svc = service::Service::new(&config, scenarios, &scenario, lib).unwrap_or_else(|e| { eprintln!("[sim_server] 启动失败: {e}"); std::process::exit(2) });
    svc.start();
    svc.start_cmd_udp(port);
    let (ct, sc, lids) = { let g = svc.lock(); (g.core.chassis_type.clone(), g.core.scenario_id.clone(), g.lidar_bufs.iter().map(|(n, _)| n.clone()).collect::<Vec<_>>()) };
    let server = Arc::new(api::build_api(svc.clone(), port));
    println!("[sim_server] REST API http://0.0.0.0:{port}/api/v1  车型={ct} 场景={sc} 激光={lids:?}");
    use std::io::Write;
    let _ = std::io::stdout().flush();
    if let Err(e) = server.serve_forever() { eprintln!("[sim_server] 监听失败: {e}"); std::process::exit(1); }
}
