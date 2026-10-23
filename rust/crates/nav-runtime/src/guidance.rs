//! Mission execution: planning dispatch + closed-loop guidance (port of Navigator.send_nav_goal / _autonomous_guidance_loop & helpers).
use crate::navigator::{pt_json, sleep, Navigator};
use planning::maneuver::{self, Corner};
use planning::route::Obs;
use planning::sweep;
use planning::{r2, wrap, Pt};
use serde_json::{json, Value};
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug)]
pub struct ArcC { pub r: f64, pub d: f64, pub turn: f64, pub heading: f64, pub cx: f64, pub cy: f64, pub v: f64 }

fn r3(x: f64) -> f64 { (x * 1000.0).round_ties_even() / 1000.0 }
fn deg(x: f64) -> f64 { x.to_degrees() }
fn rd2(x: f64) -> f64 { (x * 100.0).round_ties_even() / 100.0 }
fn el(t: &Instant) -> f64 { t.elapsed().as_secs_f64() }
fn hyp(a: Pt, b: Pt) -> f64 { (b.0 - a.0).hypot(b.1 - a.1) }
fn cfgv(n: &Navigator, k: &str, d: f64) -> f64 { n.s().chassis_cfg.get(k).and_then(|v| v.as_f64()).filter(|v| *v != 0.0).unwrap_or(d) }

impl Navigator {
    /// (x, y, yaw, vx, vy, wz, paused)
    pub fn kin(&self) -> (f64, f64, f64, f64, f64, f64, bool) {
        let s = self.s();
        (s.tele.x, s.tele.y, s.tele.yaw, s.tele.vx, s.tele.vy, s.tele.wz, s.paused)
    }

    pub fn wait_until_stopped(&self, mid: u64, timeout: f64) {
        let t0 = Instant::now();
        while el(&t0) < timeout {
            self.publish_cmd_vel(0.0, 0.0, 0.0);
            let (ok, v) = { let s = self.s(); (s.cur_mid == mid, s.tele.vx.hypot(s.tele.vy)) };
            if !ok { return; }
            if v < 0.03 { return; }
            sleep(30);
        }
    }

    pub fn cancel_nav(&self) {
        {
            let mut s = self.s();
            s.approach_left = None;
            s.tele.nav_status = "CANCELED".into();
            s.cur_mid += 1;
            s.tele.target_goal = None;
            s.tele.plan_path.clear();
            s.tele.nav_dist_rem = 0.0;
        }
        let canceled = self.s().cur_mid - 1;
        self.mission_end("CANCELED");
        for _ in 0..4 { self.publish_cmd_vel(0.0, 0.0, 0.0); sleep(20); }
        self.ev("navigation", "MISSION_CANCELED", "warning", "调度任务手动中止", &format!("操作员已中止当前导航任务 #{canceled}，底盘已安全急停抱闸"), json!({"mission_id": canceled}));
    }

    fn align_steer(&self, mid: u64, mode: &str, sign: f64) -> bool {
        let (diff, has_steer) = { let s = self.s(); (s.chassis_type == "diff_drive", !s.steer.is_empty()) };
        if diff || !has_steer { return true; }
        let cmd = if mode == "drive" { (0.002 * sign, 0.0, 0.0) } else { (0.0, 0.0, 0.004 * sign) };
        let t0 = Instant::now();
        let (mut last, mut still, mut moved): (Option<Vec<f64>>, i32, bool) = (None, 0, false);
        while self.is_running() && el(&t0) < 2.5 {
            if !self.mid_ok(mid) { return false; }
            self.publish_cmd_vel(cmd.0, cmd.1, cmd.2);
            let cur = self.s().steer.clone();
            let lim = 0.15f64.to_radians();
            if let Some(l) = &last {
                if !cur.is_empty() && cur.len() == l.len() && cur.iter().zip(l).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max) < lim { still += 1; }
                else { moved = true; still = 0; }
            } else { still = 0; }
            last = Some(cur);
            if (moved && still >= 5) || (!moved && el(&t0) > 0.7) { break; }
            sleep(30);
        }
        true
    }

    fn rotate_to(&self, mid: u64, target: f64, rot_dir: f64, max_w: f64, block_limit: f64) -> &'static str {
        let alpha = 0.6 * cfgv(self, "max_ang_decel_radps2", 1.0);
        let mut block_since: Option<Instant> = None;
        let mut settle = 0;
        while self.is_running() {
            let (ok, paused, cur_yaw, wnow) = { let s = self.s(); (s.cur_mid == mid, s.paused, s.tele.yaw, s.tele.wz) };
            if !ok { return "abort"; }
            if paused { self.publish_cmd_vel(0.0, 0.0, 0.0); sleep(40); continue; }
            let mut e = wrap(target - cur_yaw);
            if rot_dir != 0.0 && e.abs() > 0.5 && e * rot_dir < 0.0 { e += rot_dir * 2.0 * std::f64::consts::PI; }
            let e_pred = e - wnow * 0.12;
            if e.abs() < 0.005 && wnow.abs() < 0.02 {
                settle += 1;
                if settle >= 3 { self.publish_cmd_vel(0.0, 0.0, 0.0); return "done"; }
            } else { settle = 0; }
            let mut w = max_w.min((2.0 * alpha * e_pred.abs().max(0.0)).sqrt()).min(1.5 * e_pred.abs());
            let floor = if e.abs() >= 0.005 { 0.01 } else { 0.0 };
            w = w.max(floor).copysign(if e_pred.abs() > 1e-4 { e_pred } else { e });
            self.s().rot_left = e.abs();
            self.publish_cmd_vel(0.0, 0.0, w);
            self.s().rot_left = -1.0;
            if self.s().obs.layer.as_deref() == Some("rotate") {
                if block_since.is_none() { block_since = Some(Instant::now()); }
                self.set_status("OBSTACLE_WAIT");
                if el(block_since.as_ref().unwrap()) > block_limit { return "blocked"; }
            } else { block_since = None; }
            sleep(30);
        }
        "abort"
    }

    fn sweep_block_side(&self, turn: f64) -> i32 {
        let s = self.s();
        let Some((px, py)) = &s.pts else { return 0 };
        if px.is_empty() { return 0; }
        let [h, t, l, r] = Navigator::outline_of(&s);
        let m = s.prot.rotate_margin;
        let body = [-t, h, -r, l];
        let grown = [-t - m, h + m, -r - m, l + m];
        let mut sum = 0.0;
        let mut any = false;
        for i in 0..px.len() {
            let (x, y) = (px[i], py[i]);
            let inb = x > body[0] && x < body[1] && y > body[2] && y < body[3];
            let ing = x > grown[0] && x < grown[1] && y > grown[2] && y < grown[3];
            let hit = sweep::arc_hits_one(x, y, -turn, &body, 0.0, 0.0) || (!ing && sweep::arc_hits_one(x, y, -turn, &grown, 0.0, 0.0));
            if hit && !inb { any = true; sum += x; }
        }
        if !any { 0 } else if sum >= 0.0 { 1 } else { -1 }
    }

    fn retreat_for_turn(&self, mid: u64, turn: f64, max_d: f64) -> bool {
        let side = self.sweep_block_side(turn);
        if side == 0 { return true; }
        let d = -(side as f64);
        if !self.align_steer(mid, "drive", d) { return false; }
        let segs = self.obstacle_segments_all();
        let [h, t, l, r] = self.outline();
        let (x0, y0, yaw0) = self.pose();
        let (co, si) = (yaw0.cos(), yaw0.sin());
        let (mut clear_at, mut done): (Option<f64>, f64) = (None, 0.0);
        let t_end = Instant::now() + Duration::from_secs_f64(4.0 + max_d / 0.05);
        let bm = self.s().prot.body_margin;
        while self.is_running() && Instant::now() < t_end {
            let (ok, cx, cy, cyaw, paused) = { let s = self.s(); (s.cur_mid == mid, s.tele.x, s.tele.y, s.tele.yaw, s.paused) };
            if !ok { return false; }
            if paused { self.publish_cmd_vel(0.0, 0.0, 0.0); sleep(40); continue; }
            done = d * ((cx - x0) * co + (cy - y0) * si);
            if clear_at.is_none() && self.sweep_block_side(turn) == 0 { clear_at = Some(done); }
            if clear_at.map(|c| done >= c + 0.02).unwrap_or(false) || done >= max_d { break; }
            if maneuver::clearance(&segs, &[(cx + d * 0.03 * co, cy + d * 0.03 * si, cyaw)], h, t, l.max(r)) < bm { break; }
            self.publish_cmd_vel(d * 0.06, 0.0, 2.0 * wrap(yaw0 - cyaw));
            sleep(30);
        }
        self.wait_until_stopped(mid, 6.0);
        let ok = self.mid_ok(mid) && self.sweep_block_side(turn) == 0;
        let rm = self.s().prot.rotate_margin;
        self.ev("navigation", "TURN_RETREAT", if ok { "info" } else { "warning" },
            &format!("转向受阻: {} {:.2} m", if d < 0.0 { "后退" } else { "前移" }, done.abs()),
            &if ok { format!("剩余转角的扫掠区 (外扩 {rm:.2} m) 已无障碍，重新转向") } else { "移动后扫掠区仍有障碍 (地图余量不足或障碍物较近)".to_string() },
            json!({"dist": r3(done.abs())}));
        ok
    }

    pub fn rotation_free(&self, x: f64, y: f64) -> bool {
        let r = self.sweep_radius();
        let seg_d = |a: [f64; 4]| {
            let (dx, dy) = (a[2] - a[0], a[3] - a[1]);
            let l2 = dx * dx + dy * dy;
            let u = if l2 < 1e-12 { 0.0 } else { (((x - a[0]) * dx + (y - a[1]) * dy) / l2).clamp(0.0, 1.0) };
            (x - a[0] - u * dx).hypot(y - a[1] - u * dy)
        };
        if self.pl().static_segments().iter().any(|w| seg_d(*w) < r) { return false; }
        let obs = self.s().dyn_obs.clone();
        for o in obs {
            let g = |k: &str, d: f64| o.get(k).and_then(|v| v.as_f64()).unwrap_or(d);
            if (g("x", 0.0) - x).hypot(g("y", 0.0) - y) < r + 0.5 * g("w", 0.8).hypot(g("h", 0.8)) { return false; }
        }
        true
    }

    /// feasible in-place rotation direction (+1/-1), 0 if none
    pub fn rotation_dir(&self, x: f64, y: f64, yaw_from: f64, yaw_to: f64) -> f64 {
        let turn = wrap(yaw_to - yaw_from);
        if turn.abs() < 1e-3 { return 1.0; }
        let sgn = if turn > 0.0 { 1.0 } else { -1.0 };
        let segs = self.obstacle_segments_all();
        let [h, t, l, r] = self.outline();
        let need = self.s().prot.rotate_margin + 0.01;
        let rmax = h.max(t).hypot(l.max(r));
        for tt in [turn, turn - sgn * 2.0 * std::f64::consts::PI] {
            let n = maneuver::sweep_samples(tt.abs() * rmax, 4);
            let poses: Vec<(f64, f64, f64)> = (0..=n).map(|i| (x, y, yaw_from + tt * i as f64 / n as f64)).collect();
            if maneuver::clearance(&segs, &poses, h, t, l.max(r)) >= need { return if tt > 0.0 { 1.0 } else { -1.0 }; }
        }
        0.0
    }

    fn back_out(&self, mid: u64, x: f64, y: f64, yaw: f64, yaw_to: Option<f64>) -> bool {
        let segs = self.obstacle_segments_all();
        let [h, t, l, r] = self.outline();
        let hw = l.max(r);
        let bm = self.s().prot.body_margin;
        let mut target: Option<f64> = None;
        for k in 1..=30 {
            let d = 0.1 * k as f64;
            let (bx, by) = (x - d * yaw.cos(), y - d * yaw.sin());
            if maneuver::clearance(&segs, &[(bx, by, yaw)], h, t, hw) < bm { break; }
            let free = match yaw_to { Some(yt) => self.rotation_dir(bx, by, yaw, yt) != 0.0, None => self.rotation_free(bx, by) };
            if free { target = Some(d); break; }
        }
        let Some(target) = target else {
            self.ev("navigation", "BACKOUT_FAIL", "warning", "离站倒车: 未找到可转向位置", &format!("车尾方向 3 m 内没有原地转向空间 (扫掠半径 {:.2} m)，尝试直接转向", self.sweep_radius()), json!({}));
            return true;
        };
        self.ev("navigation", "BACKOUT", "info", &format!("离站倒车 {target:.1} m"), &format!("原地转向扫掠半径 {:.2} m 内有设备/墙体，先倒车到可转向位置", self.sweep_radius()), json!({"dist": rd2(target)}));
        let t_end = Instant::now() + Duration::from_secs_f64(6.0 + target / 0.1);
        while self.is_running() && Instant::now() < t_end {
            let (ok, cx, cy, cyaw, paused) = { let s = self.s(); (s.cur_mid == mid, s.tele.x, s.tele.y, s.tele.yaw, s.paused) };
            if !ok { return false; }
            if paused { self.publish_cmd_vel(0.0, 0.0, 0.0); sleep(40); continue; }
            let done = -((cx - x) * yaw.cos() + (cy - y) * yaw.sin());
            if done >= target - 0.02 { break; }
            let v = -(0.06f64.max(0.3f64.min((target - done) * 1.2)));
            self.publish_cmd_vel(v, 0.0, 2.0 * wrap(yaw - cyaw));
            sleep(30);
        }
        self.wait_until_stopped(mid, 6.0);
        self.mid_ok(mid)
    }

    fn arc_blocked(&self, r: f64, sgn: f64, remain: f64) -> bool {
        let s = self.s();
        let Some((rs, a0, inc, rmax)) = &s.scan_body else { return false };
        if remain <= 0.02 { return false; }
        let (mut px, mut py) = (vec![], vec![]);
        for (i, &v) in rs.iter().enumerate() {
            if v > 0.02 && v < rmax - 1e-3 && v < r + s.head_offset + 1.5 {
                let a = a0 + inc * i as f64;
                px.push(v * a.cos()); py.push(v * a.sin());
            }
        }
        if px.is_empty() { return false; }
        let m = 0.03;
        let bx = [-s.tail_offset - m, s.head_offset + m, -s.half_width - m, s.half_width + m];
        (0..px.len()).any(|i| sweep::arc_hits_one(px[i], py[i], -sgn * remain, &bx, 0.0, sgn * r))
    }

    fn corner_arc(&self, mid: u64, c: &ArcC, max_w: f64) -> bool {
        let sgn = if c.turn > 0.0 { 1.0 } else { -1.0 };
        let t_end = Instant::now() + Duration::from_secs_f64(4.0 * c.turn.abs() * c.r / c.v.max(0.05) + 2.0);
        self.s().in_arc = true;
        let res = (|| {
            let t0 = Instant::now();
            let (mut last, mut still): (Option<Vec<f64>>, i32) = (None, 0);
            while self.is_running() && el(&t0) < 2.5 {
                if !self.mid_ok(mid) { return false; }
                self.publish_cmd_vel(0.002, 0.0, sgn * 0.002 / c.r);
                let cur = self.s().steer.clone();
                if let Some(l) = &last {
                    if !cur.is_empty() && cur.len() == l.len() && cur.iter().zip(l).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max) < 0.15f64.to_radians() { still += 1; } else { still = 0; }
                } else { still = 0; }
                last = Some(cur);
                if still >= 5 && el(&t0) > 0.3 { break; }
                sleep(30);
            }
            self.corner_arc_loop(mid, c.v.min(0.2), c.r, c.heading, sgn, t_end + Duration::from_secs(3), max_w, Some((c.cx, c.cy)), Some(c.turn.abs()))
        })();
        self.s().in_arc = false;
        res
    }

    #[allow(clippy::too_many_arguments)]
    fn corner_arc_loop(&self, mid: u64, v: f64, r: f64, tgt: f64, sgn: f64, mut t_end: Instant, max_w: f64, cxy: Option<(f64, f64)>, sweep: Option<f64>) -> bool {
        let mut stalled: Option<Instant> = None;
        let mut phi0: Option<f64> = None;
        let dec = self.s().max_decel * 0.55;
        let track_l = self.s().track_l;
        while self.is_running() && Instant::now() < t_end {
            let (ok, paused, cur_yaw, vnow, px0, py0) = { let s = self.s(); (s.cur_mid == mid, s.paused, s.tele.yaw, s.tele.vx.hypot(s.tele.vy), s.tele.x, s.tele.y) };
            if !ok { return false; }
            if paused { self.publish_cmd_vel(0.0, 0.0, 0.0); sleep(40); continue; }
            let err = wrap(tgt - cur_yaw);
            let mut vv;
            if let (Some(c), Some(sw)) = (cxy, sweep) {
                let phi = (py0 - c.1).atan2(px0 - c.0);
                if phi0.is_none() { phi0 = Some(phi); }
                let prog = sgn * wrap(phi - phi0.unwrap());
                let rem = r * (sw - prog);
                if rem <= 0.003 { self.publish_cmd_vel(0.0, 0.0, 0.0); self.wait_until_stopped(mid, 6.0); return true; }
                let rem_eff = rem - vnow * 0.12;
                vv = 0.01f64.max(v.min((2.0 * dec * rem_eff.max(0.0)).sqrt()).min(2.0 * rem_eff.max(0.0)));
            } else {
                if err.abs() < 0.06 || err * sgn < 0.0 { return true; }
                vv = 0.08f64.max(v.min(vnow + 0.1));
            }
            let vref = vnow.max(0.03);
            let mut wz = sgn * vref / r;
            if let Some(c) = cxy {
                let (rx, ry) = (px0 - c.0, py0 - c.1);
                let rho = rx.hypot(ry);
                let tangent = ry.atan2(rx) + sgn * std::f64::consts::FRAC_PI_2;
                let e_y = sgn * (r - rho);
                let e_th = wrap(cur_yaw - tangent);
                wz -= vref * (1.8 / track_l * e_th + e_y / (track_l * track_l));
            }
            wz = wz.clamp(-max_w, max_w);
            let (vx_s, _, _) = self.safety_filter(vv, 0.0, wz);
            let mut vx_s = vx_s;
            if vx_s > 1e-3 && self.arc_blocked(r, sgn, err.abs().min(1.2)) {
                vx_s = 0.0; vv = 0.0;
                self.set_status("OBSTACLE_WAIT");
            } else if vx_s > 1e-3 {
                let mut s = self.s();
                if s.tele.nav_status == "OBSTACLE_WAIT" { s.tele.nav_status = "NAVIGATING".into(); }
            }
            if vx_s <= 1e-3 {
                wz = 0.0;
                if stalled.is_none() { stalled = Some(Instant::now()); }
                t_end += Duration::from_millis(30);
                if el(stalled.as_ref().unwrap()) > 20.0 { return true; }
            } else { stalled = None; }
            self.publish_cmd_vel(vv, 0.0, wz);
            sleep(30);
        }
        true
    }

    fn reroute_after_rot_block(self: &Arc<Self>, mid: u64, x: f64, y: f64, yaw: f64, replan_left: i32) -> bool {
        if replan_left <= 0 || std::env::var("NAV_PLAN_REVERSE").map(|v| v == "0").unwrap_or(false) || self.pl().footprint.is_none() { return false; }
        let (ok, cx, cy) = { let s = self.s(); (s.cur_mid == mid, s.tele.x, s.tele.y) };
        if !ok { return false; }
        self.publish_cmd_vel(0.0, 0.0, 0.0);
        self.pl().mark_rot_blocked(cx, cy);
        self.ev("navigation", "REROUTE", "warning", &format!("任务 #{mid}: 这里转不开，换路线"),
            &format!("在 ({cx:.2}, {cy:.2}) 原地转向受阻，按当前车头朝向重新规划 (不在此处转向，可倒车)；剩余 {} 次", replan_left - 1),
            json!({"mission_id": mid, "x": rd2(cx), "y": rd2(cy)}));
        let me = self.clone();
        std::thread::spawn(move || me.send_nav_goal(x, y, yaw, replan_left - 1));
        true
    }

    // ------------------------------------------------------------------ dispatch
    pub fn send_nav_goal(self: &Arc<Self>, x: f64, y: f64, yaw: f64, replan_left: i32) {
        let world_obs = self.link.world().get("obstacles").and_then(|o| o.as_array().cloned());
        let stations: Vec<Value> = self.pl().sc.stations.clone();
        let (mid, cur_x, cur_y, planner, yaw0, target_dock_yaw, matched) = {
            let mut s = self.s();
            s.cur_mid += 1;
            let mid = s.cur_mid;
            if let Some(o) = world_obs { s.dyn_obs = o; }
            let m = stations.iter().find(|st| (st["x"].as_f64().unwrap_or(1e9) - x).hypot(st["y"].as_f64().unwrap_or(1e9) - y) < 0.35).cloned();
            let tdy = m.as_ref().and_then(|m| m.get("dock_yaw").and_then(|v| v.as_f64())).unwrap_or(yaw);
            s.tele.target_goal = Some([x, y, tdy]);
            s.tele.nav_status = "PLANNING".into();
            (mid, s.tele.x, s.tele.y, s.active_planner.clone(), s.tele.yaw, tdy, m)
        };
        if replan_left >= 2 { self.pl().rot_blocked.clear(); }
        self.new_mission(mid, x, y, target_dock_yaw, &planner);
        let obs: Vec<Obs> = self.dyn_obstacles();

        let mut raw: Vec<Pt>;
        if planner == "nav2" {
            self.set_status("FAILED");
            self.mission_end("FAILED");
            return;
        } else if planner == "direct" || planner == "straight" {
            raw = vec![(cur_x, cur_y), (x, y)];
        } else if planner == "astar" {
            let (mut walls, boxes) = { let s = self.s(); (s.walls.clone(), s.dyn_obs.iter().map(|o| (o["x"].as_f64().unwrap_or(0.0), o["y"].as_f64().unwrap_or(0.0), o["w"].as_f64().unwrap_or(0.8), o["h"].as_f64().unwrap_or(0.8))).collect::<Vec<_>>()) };
            walls.extend(maneuver::box_segments(&boxes));
            raw = { let s = self.s(); s.astar.plan((cur_x, cur_y), (x, y), &walls) };
            if raw.len() <= 2 { raw = self.pl().plan((cur_x, cur_y), (x, y), &obs, None, None); }
        } else {
            let rev = std::env::var("NAV_PLAN_REVERSE").map(|v| v != "0").unwrap_or(true);
            raw = if rev { self.pl().plan((cur_x, cur_y), (x, y), &obs, Some(yaw0), Some(target_dock_yaw)) } else { self.pl().plan((cur_x, cur_y), (x, y), &obs, None, None) };
        }
        if raw.len() < 2 {
            { let mut s = self.s(); s.tele.nav_status = "NO_PATH".into(); s.tele.plan_path.clear(); }
            self.mission_end("NO_PATH");
            self.ev("navigation", "PLAN_FAILED", "danger", &format!("任务 #{mid} 规划失败"),
                &format!("[{}] 从 ({cur_x:.2},{cur_y:.2}) 到 ({x:.2},{y:.2}) 无可行路径 (障碍物阻断全部路线)", planner.to_uppercase()), json!({}));
            return;
        }
        let lr0 = self.pl().last_route.clone();
        let dir_plan = planner == "dijkstra" && lr0.dir && lr0.points.len() == raw.len();
        if planner == "dijkstra" && raw.len() >= 3 && self.pl().footprint.is_some() {
            let mut bad: Vec<Pt> = vec![];
            if dir_plan {
                bad = lr0.blocked.iter().filter(|b| b.2 == "拐点转向").map(|b| (b.0, b.1)).collect();
            } else {
                let mut p = self.pl();
                for i in 1..raw.len() - 1 {
                    let (a, b, c) = (raw[i - 1], raw[i], raw[i + 1]);
                    if planning::route::Planner::turn(a, b, c) > 0.02 && p.corner_penalty(a, b, c) > 0.0 { bad.push(b); }
                }
            }
            if !bad.is_empty() {
                { let mut s = self.s(); s.tele.nav_status = "NO_PATH".into(); s.tele.plan_path.clear(); }
                self.mission_end("NO_PATH");
                let [h, t, l, r] = self.outline();
                let bm = self.s().prot.body_margin;
                self.ev("navigation", "PLAN_FAILED", "danger", &format!("任务 #{mid} 规划失败: 拐点转不过去"),
                    &format!("拐点 {} 处车体 (头 {h:.2}/尾 {t:.2}/半宽 {:.2} m) 原地转向净空不足 {:.0} cm，且无其它可行路线。可在保护空间中改用 auto/arc 过弯模式，或调整拓扑/工位",
                        bad.iter().map(|p| format!("({:.1},{:.1})", p.0, p.1)).collect::<Vec<_>>().join(", "), l.max(r), bm * 100.0),
                    json!({"corners": bad.iter().map(|p| json!([p.0, p.1])).collect::<Vec<_>>()}));
                return;
            }
        }
        let mut labels: Vec<Option<String>> = vec![None; raw.len()];
        let lr = self.pl().last_route.clone();
        if lr.points.len() == raw.len() && lr.points.iter().zip(&raw).all(|(a, b)| hyp(*a, *b) < 1e-6) { labels = lr.labels.clone(); }
        let seg_rev: Option<Vec<bool>> = if dir_plan { Some(lr0.reverse.clone()) } else { None };
        {
            let mut s = self.s();
            s.tele.plan_path = raw.iter().map(|p| (r3(p.0), r3(p.1))).collect();
            s.tele.path_labels = labels.clone();
            s.tele.path_reverse = seg_rev.clone().unwrap_or_default();
            s.tele.path_index = 1;
        }
        if let Some(sr) = &seg_rev {
            if sr.iter().any(|r| *r) {
                let len: f64 = raw.windows(2).zip(sr).filter(|(_, r)| **r).map(|(w, _)| hyp(w[0], w[1])).sum();
                let desc: Vec<String> = raw[1..].iter().zip(&labels[1..]).zip(sr).map(|((b, lab), r)|
                    format!("{}到 {}", if *r { "倒车" } else { "前进" }, lab.clone().unwrap_or(format!("({:.1},{:.1})", b.0, b.1)))).collect();
                self.ev("navigation", "PLAN_REVERSE", "info", &format!("任务 #{mid}: 路线含倒车段 {len:.1} m"),
                    &format!("按车头朝向规划: {}，终点车头朝向 {:.0}°", desc.join(" → "), deg(target_dock_yaw)), json!({"mission_id": mid, "reverse": sr}));
            }
        }
        if dir_plan && !lr0.blocked.is_empty() {
            self.ev("navigation", "PLAN_WARN", "warning", &format!("任务 #{mid}: 路线上有转不开的位置"),
                &format!("{}：按地图计算车体原地转向净空不足，到现场可能受阻", lr0.blocked.iter().map(|b| format!("({:.1},{:.1}) {}", b.0, b.1, b.2)).collect::<Vec<_>>().join("；")),
                json!({"blocked": lr0.blocked.iter().map(|b| json!([b.0, b.1, b.2])).collect::<Vec<_>>()}));
        }
        let (corners, curve) = self.plan_route_corners(&raw);
        { let mut s = self.s(); s.mission_corners = (mid, corners); s.tele.plan_curve = curve; }
        self.log(&format!("Mission #{mid} dispatched via [{}]: ({x}, {y}, yaw={target_dock_yaw:.2}) with {} waypoints", planner.to_uppercase(), raw.len()));
        let origin = stations.iter().find(|st| (st["x"].as_f64().unwrap_or(1e9) - cur_x).hypot(st["y"].as_f64().unwrap_or(1e9) - cur_y) < 0.8);
        let origin_name = origin.and_then(|o| o["name"].as_str().map(|s| s.to_string())).unwrap_or(format!("起点 ({cur_x:.1}, {cur_y:.1})"));
        let dest_name = matched.as_ref().and_then(|o| o["name"].as_str().map(|s| s.to_string())).unwrap_or(format!("工位 ({x:.1}, {y:.1})"));
        self.mission_set_title(&format!("{origin_name} ➔ {dest_name}"));
        self.ev("navigation", "MISSION_DISPATCH", "info", &format!("下发调度导航任务 #{mid}"),
            &format!("目标工位: ({x:.2}, {y:.2}) | 目标航向: {:.1}° | 规划算法: {} | 规划航点: {} 个", deg(target_dock_yaw), planner.to_uppercase(), raw.len()),
            json!({"mission_id": mid, "target_x": x, "target_y": y, "target_yaw": target_dock_yaw, "waypoints_count": raw.len()}));
        let me = self.clone();
        std::thread::Builder::new().name("guidance".into()).spawn(move || me.guidance_loop(mid, raw, target_dock_yaw, replan_left, seg_rev)).unwrap();
    }

    // ------------------------------------------------------------------ guidance
    fn guidance_loop(self: &Arc<Self>, mid: u64, wps: Vec<Pt>, target_yaw: f64, replan_left: i32, seg_rev: Option<Vec<bool>>) {
        self.guidance_inner(mid, &wps, target_yaw, replan_left, seg_rev.as_deref());
        {
            let mut s = self.s();
            if s.cur_mid == mid { s.approach_left = None; }
        }
        for _ in 0..3 { self.publish_cmd_vel(0.0, 0.0, 0.0); sleep(20); }
    }

    fn guidance_inner(self: &Arc<Self>, mid: u64, wps: &[Pt], target_yaw: f64, replan_left: i32, seg_rev: Option<&[bool]>) {
        let (chassis_type, planner_kind) = {
            let mut s = self.s();
            if s.cur_mid != mid { return; }
            s.tele.nav_status = "NAVIGATING".into();
            (s.chassis_type.clone(), s.active_planner.clone())
        };
        let dual = chassis_type == "dual_steer";
        let max_v = 1.2f64.min(cfgv(self, "max_speed_mps", 1.5));
        let max_w = 1.6f64.min(cfgv(self, "max_ang_speed_radps", 2.0));
        let n = wps.len();
        let mut wait_since: Option<Instant> = None;
        let mut seg_rest = vec![0.0; n];
        for i in (0..n.saturating_sub(1)).rev() { seg_rest[i] = seg_rest[i + 1] + hyp(wps[i], wps[i + 1]); }
        let _ = seg_rest;
        let mut stop_rest = vec![0.0; n];
        for i in (1..n.saturating_sub(1)).rev() {
            let (a, b, c) = (wps[i - 1], wps[i], wps[i + 1]);
            let (h1, h2) = ((b.1 - a.1).atan2(b.0 - a.0), (c.1 - b.1).atan2(c.0 - b.0));
            let mut straight = wrap(h2 - h1).abs() <= 0.02;
            if let Some(sr) = seg_rev { if i < sr.len() && sr[i - 1] != sr[i] { straight = false; } }
            stop_rest[i] = if straight { hyp(b, c) + stop_rest[i + 1] } else { 0.0 };
        }
        let mut next_rot_dir = 0.0;
        for wp_idx in 1..n {
            let rot_dir_init = std::mem::replace(&mut next_rot_dir, 0.0);
            let mut rot_dir = rot_dir_init;
            let (prev_wp, target_wp) = (wps[wp_idx - 1], wps[wp_idx]);
            let seg_labels = {
                let mut s = self.s();
                if s.cur_mid == mid {
                    s.tele.path_index = wp_idx;
                    let labs = &s.tele.path_labels;
                    let a = labs.get(wp_idx - 1).cloned().flatten();
                    let b = labs.get(wp_idx).cloned().flatten();
                    if wp_idx < labs.len() && (b.is_some() || a.is_some()) { Some((a, b)) } else { None }
                } else { None }
            };
            if let Some((a, b)) = &seg_labels {
                let fr = a.clone().unwrap_or(format!("({:.1},{:.1})", prev_wp.0, prev_wp.1));
                let to = b.clone().unwrap_or(format!("({:.1},{:.1})", target_wp.0, target_wp.1));
                self.ev("navigation", "SEGMENT", "info", &format!("进入路段 {}/{}", wp_idx, n - 1),
                    &format!("{fr} → {to}  {:.2} m", hyp(prev_wp, target_wp)), json!({"mission_id": mid, "index": wp_idx, "from": a, "to": b}));
            }
            let (target_x, target_y) = target_wp;
            let is_final = wp_idx == n - 1;
            let (sdx, sdy) = (target_x - prev_wp.0, target_y - prev_wp.1);
            let seg_dist = sdx.hypot(sdy);
            let seg_heading = if seg_dist > 0.02 { sdy.atan2(sdx) } else { target_yaw };
            let (cur_x, cur_y, cur_yaw, ..) = self.kin();
            let planned_rev = seg_rev.map(|sr| wp_idx - 1 < sr.len() && sr[wp_idx - 1]).unwrap_or(false) && seg_dist > 0.02;
            let align_heading = if planned_rev { seg_heading + std::f64::consts::PI } else { seg_heading };
            let init_err = wrap(align_heading - cur_yaw);
            let dist_to_target = (target_x - cur_x).hypot(target_y - cur_y);
            let need_rot = !dual && init_err.abs() > 0.02 && dist_to_target > 0.1;
            let mut rd = if need_rot { self.rotation_dir(cur_x, cur_y, cur_yaw, align_heading) } else { 1.0 };
            let mut reverse = seg_rev.is_none() && !dual && init_err.abs() > 2.4 && dist_to_target > 0.3 && rd == 0.0;
            if reverse {
                self.ev("navigation", "REVERSE", "info", &format!("路段 {wp_idx} 倒车行驶"),
                    &format!("原地转向扫掠半径 {:.2} m 内有障碍，改为倒车 {dist_to_target:.1} m", self.sweep_radius()), json!({"index": wp_idx}));
            }
            // PHASE 1
            if !reverse && init_err.abs() > 0.02 && dist_to_target > 0.1 {
                self.wait_until_stopped(mid, 6.0);
                if !dual && rd == 0.0 {
                    if !self.back_out(mid, cur_x, cur_y, cur_yaw, Some(align_heading)) { return; }
                    let (bx, by, byaw) = self.pose();
                    rd = self.rotation_dir(bx, by, byaw, align_heading);
                }
                if rd != 0.0 { rot_dir = rd; }
                if !self.align_steer(mid, "rotate", if (if rot_dir != 0.0 { rot_dir } else { init_err }) > 0.0 { 1.0 } else { -1.0 }) { return; }
                let qyaw0 = self.pose().2;
                let mut turn0 = wrap(align_heading - qyaw0);
                if rot_dir != 0.0 && turn0.abs() > 0.5 && turn0 * rot_dir < 0.0 { turn0 += rot_dir * 2.0 * std::f64::consts::PI; }
                let mut res = if self.sweep_block_side(turn0) != 0 { "blocked" } else { self.rotate_to(mid, align_heading, rot_dir, max_w, 2.0) };
                for _ in 0..3 {
                    if res != "blocked" { break; }
                    self.wait_until_stopped(mid, 6.0);
                    let qyaw = self.pose().2;
                    let mut left = wrap(align_heading - qyaw);
                    if rot_dir != 0.0 && left.abs() > 0.5 && left * rot_dir < 0.0 { left += rot_dir * 2.0 * std::f64::consts::PI; }
                    let ok = self.retreat_for_turn(mid, left, 0.3);
                    if !self.mid_ok(mid) { return; }
                    if !self.align_steer(mid, "rotate", if left > 0.0 { 1.0 } else { -1.0 }) { return; }
                    res = self.rotate_to(mid, align_heading, rot_dir, max_w, if ok { 2.0 } else { 8.0 });
                    if !ok { break; }
                }
                if res == "abort" { return; }
                if res == "blocked" && self.reroute_after_rot_block(mid, wps[n - 1].0, wps[n - 1].1, target_yaw, replan_left) { return; }
                if res == "blocked" {
                    self.ev("navigation", "ROTATE_BLOCKED", "danger", &format!("任务 #{mid} 原地转向受阻"), "转向防护区内持续有障碍 8 s，任务终止；请清除障碍或调整工位/保护空间后重新下发", json!({}));
                    self.set_status("FAILED");
                    self.mission_end("FAILED");
                    return;
                }
                let mut s = self.s();
                if s.tele.nav_status == "OBSTACLE_WAIT" { s.tele.nav_status = "NAVIGATING".into(); }
            }
            reverse = reverse || planned_rev;
            if !dual && !self.align_steer(mid, "drive", if reverse { -1.0 } else { 1.0 }) { return; }

            // corner handling
            let mut corner: Option<ArcC> = None;
            let mut corner_rot = false;
            let nxt_rev = seg_rev.map(|sr| wp_idx < sr.len() && sr[wp_idx]).unwrap_or(false);
            if seg_rev.is_some() && !is_final && (reverse || nxt_rev) {
                let nxt = wps[wp_idx + 1];
                let nh = (nxt.1 - target_y).atan2(nxt.0 - target_x) + if nxt_rev { std::f64::consts::PI } else { 0.0 };
                if wrap(nh - align_heading).abs() > 0.02 || nxt_rev != reverse { corner_rot = true; }
            }
            if !reverse && !nxt_rev && !is_final && !dual && seg_dist > 0.05 {
                let nxt = wps[wp_idx + 1];
                let n_len = hyp((target_x, target_y), nxt);
                if n_len > 0.1 {
                    let next_heading = (nxt.1 - target_y).atan2(nxt.0 - target_x);
                    let turn = wrap(next_heading - seg_heading);
                    let arc_mode = self.s().prot.corner_mode == "arc";
                    if turn.abs() > if arc_mode { 0.35 } else { 0.02 } {
                        let pre = { let s = self.s(); if s.mission_corners.0 == mid && wp_idx < s.mission_corners.1.len() { s.mission_corners.1[wp_idx].clone() } else { None } };
                        let (cr, clr) = match pre { Some(p) => p, None => self.plan_corner((target_x, target_y), seg_heading, next_heading, seg_dist, n_len) };
                        match cr {
                            Corner::Rotate(r) => { corner_rot = true; next_rot_dir = r; }
                            Corner::Arc { heading, turn, r, d, v, cx, cy } => { corner = Some(ArcC { r, d, turn, heading, cx, cy, v: v.min(max_v) }); }
                        }
                        if clr < 0.05 {
                            let (hd, hw) = { let s = self.s(); (s.head_offset, s.half_width) };
                            self.ev("navigation", "CORNER_TIGHT", "warning", &format!("拐点 {wp_idx} 净空不足"),
                                &format!("{} 车体最小净空 {:.0} cm (车头 {hd:.2} m / 半宽 {hw:.2} m)",
                                    corner.map(|c| format!("圆弧 R={:.2} m", c.r)).unwrap_or("拐点原地转向".into()), clr * 100.0),
                                json!({"index": wp_idx, "clearance": r3(clr)}));
                        }
                    }
                }
            }
            let mut sharp_next = corner_rot;
            if !is_final && corner.is_none() && !corner_rot {
                let nxt = wps[wp_idx + 1];
                if hyp((target_x, target_y), nxt) > 0.1 {
                    let nh = (nxt.1 - target_y).atan2(nxt.0 - target_x);
                    sharp_next = wrap(nh - seg_heading).abs() > 0.6;
                }
            }
            let _ = sharp_next;

            // PHASE 2
            while self.is_running() {
                let (ok, paused, cur_x, cur_y, cur_yaw, v_meas) = { let s = self.s(); (s.cur_mid == mid, s.paused, s.tele.x, s.tele.y, s.tele.yaw, s.tele.vx.abs()) };
                if !ok { return; }
                if paused { self.publish_cmd_vel(0.0, 0.0, 0.0); sleep(40); continue; }
                let (ch, sh) = (seg_heading.cos(), seg_heading.sin());
                let along = (cur_x - target_x) * ch + (cur_y - target_y) * sh;
                let lateral = -(cur_x - target_x) * sh + (cur_y - target_y) * ch;
                let stop_here = is_final || corner_rot;
                if let Some(c) = &corner {
                    if along >= -c.d - 0.003 && lateral.abs() < 0.35 { break; }
                } else if stop_here {
                    if along > -0.003 && lateral.abs() < 0.35 { break; }
                } else if along >= 0.0 && lateral.abs() < 0.6 { break; }
                let cross_track = -(cur_x - prev_wp.0) * sh + (cur_y - prev_wp.1) * ch;
                let (dec, track_l, prot_enabled, arrive_tol) = { let s = self.s(); (s.max_decel * 0.55, s.track_l, s.prot.enabled, s.prot.arrive_tolerance) };
                let rem_stop = (-along - corner.map(|c| c.d).unwrap_or(0.0)).max(0.0) + if corner.is_some() { 0.0 } else { stop_rest[wp_idx] };
                let rem_eff = rem_stop - v_meas * 0.12;
                self.s().approach_left = if reverse { None } else { Some(rem_stop) };
                let v_stop = (2.0 * dec * (rem_eff - 0.002).max(0.0)).sqrt().min(2.0 * rem_eff.max(0.0));
                let mut vx_nom = max_v.min(v_stop.max(if rem_stop > 0.002 { 0.01 } else { 0.0 }));
                let sgn = if reverse { -1 } else { 1 };
                let al = if reverse { None } else { self.s().approach_left };
                let (mut allowed, mut d_hit, _need) = self.allowed_speed(sgn, al);
                if !prot_enabled { allowed = 99.0; }
                if !self.photo_sides(false, true)[if reverse { 1 } else { 0 }].is_empty() { allowed = 0.0; d_hit = Some(0.0); }
                if allowed <= 1e-6 {
                    self.publish_cmd_vel(0.0, 0.0, 0.0);
                    match wait_since {
                        None => wait_since = Some(Instant::now()),
                        Some(w) => {
                            if is_final && -along < arrive_tol && el(&w) > 2.0 {
                                self.ev("navigation", "DOCK_LIMITED", "warning", "工位前方受限，提前停靠",
                                    &format!("距目标 {:.0} cm 处前方障碍 {} m，按到位处理", (-along).max(0.0) * 100.0, d_hit.map(|d| format!("{d}")).unwrap_or("None".into())), json!({}));
                                break;
                            } else if el(&w) > 5.0 && planner_kind != "direct" && planner_kind != "straight" && replan_left > 0 {
                                self.ev("navigation", "REPLAN", "warning", &format!("任务 #{mid} 阻塞超时，重新规划"), "前方障碍物持续 5 s 未解除，结合动态障碍物重新计算绕行路线", json!({}));
                                let me = self.clone();
                                let (gx, gy) = wps[n - 1];
                                std::thread::spawn(move || me.send_nav_goal(gx, gy, target_yaw, replan_left - 1));
                                return;
                            }
                        }
                    }
                    self.set_status("OBSTACLE_WAIT");
                    sleep(40);
                    continue;
                } else { wait_since = None; }
                if allowed < vx_nom { vx_nom = 0.08f64.max(allowed); }
                { let mut s = self.s(); if s.tele.nav_status == "OBSTACLE_WAIT" { s.tele.nav_status = "NAVIGATING".into(); } }
                let move_heading = cur_yaw + if reverse { std::f64::consts::PI } else { 0.0 };
                let e_th = wrap(move_heading - seg_heading);
                let zeta = 0.9;
                let mut vx = vx_nom;
                if e_th.abs() > 0.35 { vx = vx.min(0.1); }
                let mut wz = -(vx.max(0.02)) * (2.0 * zeta / track_l * e_th + cross_track / (track_l * track_l));
                wz = wz.clamp(-max_w, max_w);
                if dual {
                    let h_err = wrap(seg_heading - cur_yaw);
                    let wz2 = (h_err * 1.5).clamp(-max_w.min(0.6), max_w.min(0.6));
                    let vy = (-cross_track * 1.2).clamp(-0.3, 0.3);
                    let vx2 = 0.15f64.min(vx_nom).max(vx_nom * 0.3f64.max(h_err.abs().min(std::f64::consts::FRAC_PI_2).cos()));
                    self.publish_cmd_vel(vx2, vy, wz2);
                } else if reverse { self.publish_cmd_vel(-(vx.min(0.5)), 0.0, wz); }
                else { self.publish_cmd_vel(vx, 0.0, wz); }
                sleep(30);
            }
            if let Some(c) = &corner { if !self.corner_arc(mid, c, max_w) { return; } }
        }
        // PHASE 3
        self.s().approach_left = None;
        let target_dock_yaw = (target_yaw / std::f64::consts::FRAC_PI_2).round_ties_even() * std::f64::consts::FRAC_PI_2;
        self.wait_until_stopped(mid, 6.0);
        self.ev("navigation", "NAV_DOCKING", "info", "工位末端调姿对齐", &format!("进入工位末端高精度调姿阶段，目标 Cardinal 航向: {:.1}°", (deg(target_dock_yaw) * 10.0).round_ties_even() / 10.0), json!({"target_dock_yaw": r3(target_dock_yaw)}));
        let (fx, fy, fyaw) = self.pose();
        let mut do_align = true;
        if !dual && wrap(target_dock_yaw - fyaw).abs() > 0.3 && self.rotation_dir(fx, fy, fyaw, target_dock_yaw) == 0.0 {
            self.ev("navigation", "DOCK_SKIP_ROTATE", "warning", "工位调姿跳过", &format!("原地转向扫掠半径 {:.2} m 内有障碍，保持当前航向 {:.0}°", self.sweep_radius(), deg(fyaw)), json!({}));
            do_align = false;
        }
        if do_align {
            self.align_steer(mid, "rotate", 1.0);
            if self.rotate_to(mid, target_dock_yaw, 0.0, 1.0f64.min(max_w), 8.0) == "abort" { return; }
        }
        self.publish_cmd_vel(0.0, 0.0, 0.0);
        let (fin_x, fin_y, fin_yaw) = {
            let mut s = self.s();
            if s.cur_mid == mid { s.tele.nav_status = "ARRIVED".into(); s.tele.plan_path.clear(); s.tele.nav_dist_rem = 0.0; }
            (s.tele.x, s.tele.y, s.tele.yaw)
        };
        self.mission_end("ARRIVED");
        let dev = (deg(wrap(target_dock_yaw - fin_yaw)).abs() * 100.0).round_ties_even() / 100.0;
        self.log(&format!("Mission #{mid} ARRIVED & DOCKED (final yaw={fin_yaw:.2} rad, target={target_dock_yaw:.2})."));
        self.ev("navigation", "MISSION_ARRIVED", "success", &format!("任务 #{mid} 停靠到位完成"),
            &format!("已就位停靠，位置: ({fin_x:.2}, {fin_y:.2}) | 航向对齐偏差: {dev}° (严格满足正交平行)"),
            json!({"mission_id": mid, "x": r3(fin_x), "y": r3(fin_y), "yaw": r3(fin_yaw), "dev_deg": dev}));
        let _ = pt_json;
    }
}
