//! Industrial DI/DO simulator (port of sensors/io_simulator.py)
use serde_json::{json, Map, Value};

#[derive(Clone, Debug)]
pub struct IoSim {
    pub di: Vec<(String, bool)>,
    pub do_: Vec<(String, bool)>,
    pub lift_height_m: f64,
}

fn get(v: &[(String, bool)], k: &str) -> Option<bool> { v.iter().find(|(n, _)| n == k).map(|(_, b)| *b) }
fn set(v: &mut Vec<(String, bool)>, k: &str, b: bool) -> bool {
    if let Some(e) = v.iter_mut().find(|(n, _)| n == k) { e.1 = b; true } else { false }
}

impl IoSim {
    pub fn new() -> IoSim {
        let di = ["di_estop", "di_bumper_front", "di_bumper_rear", "di_cargo_present", "di_lift_top", "di_lift_bottom"].iter().map(|k| (k.to_string(), *k == "di_lift_bottom")).collect();
        let do_ = [("do_brake_release", true), ("do_tower_green", true), ("do_tower_yellow", false), ("do_tower_red", false), ("do_buzzer", false), ("do_lift_motor_up", false), ("do_lift_motor_down", false)]
            .iter().map(|(k, v)| (k.to_string(), *v)).collect();
        IoSim { di, do_, lift_height_m: 0.0 }
    }
    pub fn setdefault_di(&mut self, k: &str, v: bool) { if get(&self.di, k).is_none() { self.di.push((k.to_string(), v)); } }
    pub fn setdefault_do(&mut self, k: &str, v: bool) { if get(&self.do_, k).is_none() { self.do_.push((k.to_string(), v)); } }
    pub fn di(&self, k: &str) -> bool { get(&self.di, k).unwrap_or(false) }
    pub fn do_(&self, k: &str, d: bool) -> bool { get(&self.do_, k).unwrap_or(d) }
    pub fn set_di(&mut self, k: &str, v: bool) {
        if set(&mut self.di, k, v) { self.interlock(); }
    }
    pub fn set_do(&mut self, k: &str, v: bool) { set(&mut self.do_, k, v); }
    fn interlock(&mut self) {
        let bump = self.di.iter().any(|(k, v)| k.starts_with("di_bumper") && *v);
        let estop = self.di("di_estop");
        set(&mut self.do_, "do_brake_release", !estop);
        let alarm = estop || bump;
        set(&mut self.do_, "do_tower_red", alarm);
        set(&mut self.do_, "do_buzzer", alarm);
        if alarm { set(&mut self.do_, "do_tower_green", false); }
    }
    pub fn update_lift(&mut self, dt: f64) {
        let speed = 0.02;
        if self.do_("do_lift_motor_up", false) { self.lift_height_m = (self.lift_height_m + speed * dt).min(0.08); }
        else if self.do_("do_lift_motor_down", false) { self.lift_height_m = (self.lift_height_m - speed * dt).max(0.0); }
        let top = self.lift_height_m >= 0.078;
        let bot = self.lift_height_m <= 0.002;
        set(&mut self.di, "di_lift_top", top);
        set(&mut self.di, "di_lift_bottom", bot);
    }
    pub fn estop(&self) -> bool { !self.do_("do_brake_release", true) }
    pub fn state(&self) -> Value {
        let m = |v: &Vec<(String, bool)>| Value::Object(v.iter().map(|(k, b)| (k.clone(), json!(b))).collect::<Map<String, Value>>());
        json!({"inputs": m(&self.di), "outputs": m(&self.do_), "lift_height_mm": (self.lift_height_m * 1000.0 * 10.0).round() / 10.0,
            "is_emergency_stop": !self.do_("do_brake_release", true), "bumper_active": self.di.iter().any(|(k, v)| k.starts_with("di_bumper") && *v)})
    }
}
