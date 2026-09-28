// ============================================================================
// 执行进程自研导引 (C++)：nav_runtime/navigator.py _autonomous_guidance_loop 及其辅助动作的移植
//   逐路段: 原地转向对准 (转不开先离站倒车；近乎反向且转不开则倒车行驶) → 舵轮就位 → 直线跟踪
//   (按到下一停车点的剩余行程减速，二阶临界阻尼横向跟踪) → 拐点圆弧过弯 / 停车原地转向 → 工位末端调姿
//   防护区: 允许速度 = 停车距离放得下的最高一档 (末段进站按剩余行程)，光电触发视同障碍；持续阻塞 5 s 请求重新规划
//   末段精定位 (与 agv_nav2_plugins 同一套 localize.hpp): 最后一段剩余 < refine_dist 时停车，主激光原始帧对场景
//   墙体做 ICP，修正定位偏置后继续进站；调姿后再核对一次朝向
//   依赖通过 Io 回调注入 (位姿/速度/舵角/暂停急停/防护区/激光点/指令/事件/状态)，本文件不含 ROS
// ============================================================================
#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "agv_nav2_plugins/localize.hpp"
#include "safety.hpp"

extern "C" {
double an_clearance(const double *segs, int ns, const double *poses, int np, double head, double tail, double hw);
double an_plan_corner(const double *segs, int ns, double nx, double ny, double h1, double h2, double len_in, double len_out,
                      double head, double tail, double hw, double r_pref, double clear_min, int mode, double *out);
}

namespace guide {

inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

struct Corner {                 // 拐点过弯方式 (路线下发时由执行进程按 planning.maneuver 确定)
  int kind = 0;                 // 0 无 1 圆弧 2 原地转向
  double rot = 0, R = 0, d = 0, turn = 0, heading = 0, v = 0.35, cx = 0, cy = 0, clr = 9.0;
};

struct Mission {
  long mid = 0;
  std::vector<std::pair<double, double>> wps;
  std::vector<std::string> labels;
  std::vector<Corner> corners;   // 与 wps 同长 (下标 = 航点)
  double target_yaw = 0;
  int replan_left = 2;
  std::string planner = "dijkstra", chassis = "single_steer", corner_mode = "auto";
  double max_v = 1.2, max_w = 1.6, max_decel = 0.5, max_ang_decel = 1.0, track_L = 0.6;
  double head = 0.6, tail = 0.6, hw = 0.4, corner_radius = 0.9, body_margin = 0.05, rotate_margin = 0.02;
  double arrive_tol = 0.25;
  std::vector<double> segs;      // 静态 + 动态障碍线段 (x0,y0,x1,y1)*n：原地转向/倒车净空
  std::vector<agv::Seg> refine_segs;   // 静态墙/货架 (精定位)
  bool refine = true;
  double refine_dist = 0.8;
};

struct Pose { double x = 0, y = 0, th = 0; };

// 与执行进程核心的接口
struct Io {
  std::function<Pose()> pose;                               // map 系位姿 (定位融合)
  std::function<double()> v_meas;                           // 里程计前进速度 (机体系 vx)
  std::function<double()> w_meas;
  std::function<std::vector<double>()> steer;               // 舵角
  std::function<bool()> hold;                               // 暂停/急停
  // 防护区: sign=+1 前进 / -1 倒车；has_left/left 末段剩余行程 → 允许速度 (0 = 停车)，d_hit 触发距离
  std::function<double(int, bool, double, double *)> allowed;
  std::function<bool(bool, bool, double)> photo_block;      // 光电 (前向?, 有剩余行程?, 剩余行程) 触发 (距停车点很近时屏蔽前向)
  std::function<bool(double)> rotation_blocked;             // 原地转向扫掠 (方向 ±1) 是否受阻 (与安全层同一判定)
  std::function<std::vector<agv::Pt>()> scan_pts;           // 融合扫描点 (机体系)
  std::function<bool(std::vector<agv::Pt> &, double)> refine_scan;   // 停车后 (参数: 停车时刻) 取主激光原始帧点
  std::function<void(double, double, double, bool, bool, double)> cmd;   // vx, vy, wz, in_arc, has_left, left → 经安全层下发
  std::function<void(const Pose &)> set_correction;         // 精定位结果 (map 位姿) → 定位修正
  std::function<void(const std::string &, const std::string &, const std::string &, const std::string &)> event;
  std::function<void(const std::string &, int, double)> status;   // 状态 (NAVIGATING/OBSTACLE_WAIT/...), 航点, 剩余
  std::function<void(bool, const std::string &)> done;      // 结束: ok, 结果 (ARRIVED/FAILED/REPLAN/ABORT)
};

class Guidance {
 public:
  explicit Guidance(Io io) : io_(std::move(io)) {}
  ~Guidance() { cancel(); }

  void start(Mission m) {
    cancel();
    stop_ = false;
    m_ = std::move(m);
    th_ = std::thread([this] { run(); });
  }
  void cancel() {
    stop_ = true;
    if (th_.joinable()) th_.join();
  }
  bool active() const { return running_; }

 private:
  // ---------------------------------------------------------------- 基本动作
  static void sleep_s(double s) { std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long>(s * 1e6))); }
  static double now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
  void cmd(double vx, double vy, double wz, bool in_arc = false) { io_.cmd(vx, vy, wz, in_arc, has_left_, left_); }
  bool dual() const { return m_.chassis == "dual_steer"; }
  double sweep_radius() const { return std::hypot(std::max(m_.head, m_.tail), m_.hw) + m_.rotate_margin; }
  static std::string f2(double v) { char b[32]; std::snprintf(b, sizeof(b), "%.2f", v); return b; }

  double clearance(const std::vector<double> &poses) const {
    return an_clearance(m_.segs.data(), static_cast<int>(m_.segs.size() / 4), poses.data(), static_cast<int>(poses.size() / 3),
                        m_.head, m_.tail, m_.hw);
  }

  // 原地从 yaw_from 转到 yaw_to 的可行方向 (navigator._rotation_dir)
  double rotation_dir(double x, double y, double yaw_from, double yaw_to) const {
    const double turn = wrap(yaw_to - yaw_from);
    if (std::fabs(turn) < 1e-3) return 1.0;
    const double sgn = turn > 0 ? 1.0 : -1.0;
    const double need = m_.rotate_margin + 0.01;
    for (double tt : {turn, turn - sgn * 2 * M_PI}) {
      const int n = std::max(4, static_cast<int>(std::fabs(tt) / 0.12) + 1);
      std::vector<double> p;
      for (int i = 0; i <= n; ++i) { p.push_back(x); p.push_back(y); p.push_back(yaw_from + tt * i / n); }
      if (clearance(p) >= need) return tt > 0 ? 1.0 : -1.0;
    }
    return 0.0;
  }
  bool rotation_free(double x, double y) const {       // navigator._rotation_free (扫掠圆内无线段)
    const double r = sweep_radius();
    for (size_t i = 0; i + 3 < m_.segs.size(); i += 4) {
      const double ax = m_.segs[i], ay = m_.segs[i + 1], dx = m_.segs[i + 2] - ax, dy = m_.segs[i + 3] - ay;
      const double L2 = dx * dx + dy * dy;
      const double u = L2 < 1e-12 ? 0.0 : std::max(0.0, std::min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2));
      if (std::hypot(x - ax - u * dx, y - ay - u * dy) < r) return false;
    }
    return true;
  }

  void wait_until_stopped(double timeout = 6.0) {
    const double t0 = now();
    while (now() - t0 < timeout && !stop_) {
      cmd(0, 0, 0);
      if (std::fabs(io_.v_meas()) < 0.03) return;
      sleep_s(0.03);
    }
  }

  // 先转后走: 微小指令让舵轮就位 (navigator._align_steer)
  bool align_steer(bool rotate, double sign, double timeout = 2.5) {
    if (m_.chassis == "diff_drive") return !stop_;
    const double t0 = now();
    std::vector<double> last;
    int still = 0;
    bool moved = false, have = false;
    while (!stop_ && now() - t0 < timeout) {
      if (rotate) cmd(0, 0, 0.004 * sign); else cmd(0.002 * sign, 0, 0);
      const auto cur = io_.steer();
      if (have && !cur.empty() && cur.size() == last.size()) {
        double mx = 0;
        for (size_t i = 0; i < cur.size(); ++i) mx = std::max(mx, std::fabs(cur[i] - last[i]));
        if (mx < 0.15 * M_PI / 180) ++still; else { moved = true; still = 0; }
      } else if (have) { moved = true; still = 0; }
      last = cur;
      have = true;
      if ((moved && still >= 5) || (!moved && now() - t0 > 0.7)) break;
      sleep_s(0.03);
    }
    return !stop_;
  }

  // 原地转向到 target (±0.3°)，ω = √(2·α·|e|) (navigator._rotate_to)。返回 0 done / 1 abort / 2 blocked
  int rotate_to(double target, double rot_dir, double max_w, double tol = 0.005) {
    const double alpha = 0.6 * m_.max_ang_decel;
    double block_since = -1;
    int settle = 0;
    while (!stop_) {
      if (io_.hold()) { cmd(0, 0, 0); sleep_s(0.04); continue; }
      const Pose p = io_.pose();
      const double wnow = io_.w_meas();
      double e = wrap(target - p.th);
      if (rot_dir != 0 && std::fabs(e) > 0.5 && e * rot_dir < 0) e += rot_dir * 2 * M_PI;
      const double e_pred = e - wnow * 0.12;
      if (std::fabs(e) < tol && std::fabs(wnow) < 0.02) {
        if (++settle >= 3) { cmd(0, 0, 0); return 0; }
      } else settle = 0;
      double w = std::min({max_w, std::sqrt(2.0 * alpha * std::max(0.0, std::fabs(e_pred))), 1.5 * std::fabs(e_pred)});
      w = std::copysign(std::max(std::fabs(e) >= tol ? 0.01 : 0.0, w), std::fabs(e_pred) > 1e-4 ? e_pred : e);
      cmd(0, 0, w);
      if (std::fabs(w) > 0.05 && io_.rotation_blocked(w > 0 ? 1.0 : -1.0)) {
        if (block_since < 0) block_since = now();
        io_.status("OBSTACLE_WAIT", idx_, rem_);
        if (now() - block_since > 8.0) return 2;
      } else block_since = -1;
      sleep_s(0.03);
    }
    return 1;
  }

  // 离站倒车: 沿车身反方向找最近的可转向位置 (≤ 3 m) 倒过去 (navigator._back_out)
  bool back_out(double x, double y, double yaw, double yaw_to) {
    double target = -1;
    for (int k = 1; k <= 30; ++k) {
      const double d = 0.1 * k, bx = x - d * std::cos(yaw), by = y - d * std::sin(yaw);
      std::vector<double> p = {bx, by, yaw};
      if (clearance(p) < m_.body_margin) break;
      if (rotation_dir(bx, by, yaw, yaw_to) != 0.0) { target = d; break; }
    }
    if (target < 0) {
      io_.event("BACKOUT_FAIL", "warning", "离站倒车: 未找到可转向位置",
                "车尾方向 3 m 内没有原地转向空间 (扫掠半径 " + f2(sweep_radius()) + " m)，尝试直接转向");
      return !stop_;
    }
    io_.event("BACKOUT", "info", "离站倒车 " + f2(target) + " m",
              "原地转向扫掠半径 " + f2(sweep_radius()) + " m 内有设备/墙体，先倒车到可转向位置");
    const double t_end = now() + 6.0 + target / 0.1;
    while (!stop_ && now() < t_end) {
      if (io_.hold()) { cmd(0, 0, 0); sleep_s(0.04); continue; }
      const Pose p = io_.pose();
      const double done = -((p.x - x) * std::cos(yaw) + (p.y - y) * std::sin(yaw));
      if (done >= target - 0.02) break;
      const double v = -std::max(0.06, std::min(0.3, (target - done) * 1.2));
      cmd(v, 0, 2.0 * wrap(yaw - p.th));
      sleep_s(0.03);
    }
    wait_until_stopped();
    return !stop_;
  }

  bool arc_blocked(double R, double sgn, double remain) const {     // navigator._arc_blocked
    if (remain <= 0.02) return false;
    const auto pts = io_.scan_pts();
    const double m = 0.03;
    const int n = std::max(2, static_cast<int>(remain / 0.15) + 1);
    for (int k = 0; k < n; ++k) {
      const double phi = remain * k / (n - 1);
      const double x = R * std::sin(phi), y = sgn * R * (1 - std::cos(phi)), th = sgn * phi, c = std::cos(th), s = std::sin(th);
      for (const auto &q : pts) {
        if (std::hypot(q.x, q.y) >= R + m_.head + 1.5) continue;
        const double lx = (q.x - x) * c + (q.y - y) * s, ly = -(q.x - x) * s + (q.y - y) * c;
        if (lx > -m_.tail - m && lx < m_.head + m && std::fabs(ly) < m_.hw + m) return true;
      }
    }
    return false;
  }

  // 圆弧过弯 (navigator._corner_arc + _corner_arc_loop)
  bool corner_arc(const Corner &c, double max_w) {
    const double sgn = c.turn > 0 ? 1.0 : -1.0;
    double t_end = now() + 4.0 * std::fabs(c.turn) * c.R / std::max(c.v, 0.05) + 2.0;
    {   // 切点处舵轮先转到圆弧曲率对应的舵角
      const double t0 = now();
      std::vector<double> last;
      int still = 0;
      bool have = false;
      while (!stop_ && now() - t0 < 2.5) {
        cmd(0.002, 0, sgn * 0.002 / c.R, true);
        const auto cur = io_.steer();
        bool st = false;
        if (have && !cur.empty() && cur.size() == last.size()) {
          double mx = 0;
          for (size_t i = 0; i < cur.size(); ++i) mx = std::max(mx, std::fabs(cur[i] - last[i]));
          st = mx < 0.15 * M_PI / 180;
        }
        still = st ? still + 1 : 0;
        last = cur;
        have = true;
        if (still >= 5 && now() - t0 > 0.3) break;
        sleep_s(0.03);
      }
    }
    const double v = std::min(c.v, 0.2), R = c.R, tgt = c.heading, sweep = std::fabs(c.turn);
    const double dec = m_.max_decel * 0.55;
    t_end += 3.0;
    double stalled = -1;
    bool have_phi0 = false;
    double phi0 = 0;
    while (!stop_ && now() < t_end) {
      if (io_.hold()) { cmd(0, 0, 0, true); sleep_s(0.04); continue; }
      const Pose p = io_.pose();
      const double vnow = std::fabs(io_.v_meas());
      const double phi = std::atan2(p.y - c.cy, p.x - c.cx);
      if (!have_phi0) { phi0 = phi; have_phi0 = true; }
      const double prog = sgn * wrap(phi - phi0);
      const double rem = R * (sweep - prog);
      if (rem <= 0.003) { cmd(0, 0, 0, true); wait_until_stopped(); return !stop_; }
      const double rem_eff = rem - vnow * 0.12;
      double vv = std::max(0.01, std::min({v, std::sqrt(2.0 * dec * std::max(0.0, rem_eff)), 2.0 * std::max(0.0, rem_eff)}));
      const double err = wrap(tgt - p.th);
      const double vref = std::max(vnow, 0.03);
      double wz = sgn * vref / R;
      const double rx = p.x - c.cx, ry = p.y - c.cy, rho = std::hypot(rx, ry);
      const double tangent = std::atan2(ry, rx) + sgn * M_PI / 2;
      const double e_y = sgn * (R - rho), e_th = wrap(p.th - tangent), Lc = m_.track_L;
      wz -= vref * (1.8 / Lc * e_th + e_y / (Lc * Lc));
      wz = std::max(-max_w, std::min(max_w, wz));
      // 防护区 (过弯按弧线扫掠区检查): 安全层允许速度 + 弧线扫掠激光点
      double d_hit;
      const double allowed = io_.allowed(1, false, 0.0, &d_hit);
      bool blocked = allowed <= 1e-6 || io_.photo_block(true, false, 0.0);
      if (!blocked && arc_blocked(R, sgn, std::min(std::fabs(err), 1.2))) blocked = true;
      if (blocked) {
        vv = 0.0;
        wz = 0.0;
        io_.status("OBSTACLE_WAIT", idx_, rem_);
        if (stalled < 0) stalled = now();
        t_end += 0.03;
        if (now() - stalled > 20.0) return !stop_;
      } else {
        stalled = -1;
        io_.status("NAVIGATING", idx_, rem_);
      }
      cmd(vv, 0, wz, true);
      sleep_s(0.03);
    }
    return !stop_;
  }

  // 沿 heading 方向后退 dist (保持航向；经安全层，后向防护区生效)
  bool back_along(double heading, double dist) {
    wait_until_stopped();
    if (!align_steer(false, -1.0)) return false;
    const Pose s0 = io_.pose();
    const double t_end = now() + 6.0 + dist / 0.1;
    while (!stop_ && now() < t_end) {
      if (io_.hold()) { cmd(0, 0, 0); sleep_s(0.04); continue; }
      const Pose p = io_.pose();
      const double done = -((p.x - s0.x) * std::cos(heading) + (p.y - s0.y) * std::sin(heading));
      if (done >= dist - 0.01) break;
      const double v = -std::max(0.05, std::min(0.2, (dist - done) * 1.0));
      cmd(v, 0, 2.0 * wrap(heading - p.th));
      sleep_s(0.03);
    }
    wait_until_stopped();
    return !stop_ && align_steer(false, 1.0);
  }

  // 末段精定位: 停车 → 主激光原始帧 ICP → 定位修正
  void refine_here(const char *why) {
    if (!m_.refine || m_.refine_segs.empty()) return;
    wait_until_stopped();
    const double t_stop = now();
    sleep_s(0.3);
    std::vector<agv::Pt> pts;
    if (!io_.refine_scan(pts, t_stop)) {
      io_.event("LOC_REFINE", "warning", "精定位未采用", std::string(why) + ": 等不到停车后的主激光帧");
      return;
    }
    const Pose est = io_.pose();
    const auto r = agv::icp(pts, m_.refine_segs, {est.x, est.y, est.th}, 0.025, 40, 0.025);
    const double corr = std::hypot(r.pose.x - est.x, r.pose.y - est.y);
    char b[220];
    if (r.ok && corr < 0.15) {
      io_.set_correction({r.pose.x, r.pose.y, r.pose.th});
      std::snprintf(b, sizeof(b), "%s: 修正定位 %.1f mm / %.2f°，内点 %d，残差 %.1f mm", why, corr * 1000,
                    wrap(r.pose.th - est.th) * 180 / M_PI, r.inliers, r.rms * 1000);
      io_.event("LOC_REFINE", "info", "精定位", b);
    } else {
      std::snprintf(b, sizeof(b), "%s: 未采用 (%s，修正量 %.1f mm)", why, r.ok ? "修正量过大" : r.why, corr * 1000);
      io_.event("LOC_REFINE", "warning", "精定位未采用", b);
    }
  }

  // ---------------------------------------------------------------- 主流程
  void run() {
    running_ = true;
    const auto result = run_inner();
    for (int i = 0; i < 3; ++i) { cmd(0, 0, 0); sleep_s(0.02); }
    has_left_ = false;
    running_ = false;
    if (!stop_ || result == "REPLAN") io_.done(result == "ARRIVED", result);
  }

  std::string run_inner() {
    const auto &W = m_.wps;
    const size_t n = W.size();
    if (n < 2) return "FAILED";
    const double max_v = m_.max_v, max_w = m_.max_w;
    std::vector<double> stop_rest(n, 0.0);
    for (size_t i = n - 2; i >= 1; --i) {
      const auto &a = W[i - 1], &b = W[i], &c = W[i + 1];
      const double h1 = std::atan2(b.second - a.second, b.first - a.first), h2 = std::atan2(c.second - b.second, c.first - b.first);
      const bool straight = std::fabs(wrap(h2 - h1)) <= 0.02;
      stop_rest[i] = straight ? std::hypot(c.first - b.first, c.second - b.second) + stop_rest[i + 1] : 0.0;
      if (i == 1) break;
    }
    io_.status("NAVIGATING", 1, 0);
    double wait_since = -1, next_rot_dir = 0.0;
    bool refined = false;
    int redo = 0;
    for (size_t wi = 1; wi < n && !stop_; ++wi) {
      idx_ = static_cast<int>(wi);
      double rot_dir = next_rot_dir;
      next_rot_dir = 0.0;
      const auto prev = W[wi - 1], tgt = W[wi];
      if (wi < m_.labels.size() && (!m_.labels[wi].empty() || !m_.labels[wi - 1].empty())) {
        auto nm = [&](size_t k) {
          if (!m_.labels[k].empty()) return m_.labels[k];
          char b[48];
          std::snprintf(b, sizeof(b), "(%.1f,%.1f)", W[k].first, W[k].second);
          return std::string(b);
        };
        io_.event("SEGMENT", "info", "进入路段 " + std::to_string(wi) + "/" + std::to_string(n - 1),
                  nm(wi - 1) + " → " + nm(wi) + "  " + f2(std::hypot(tgt.first - prev.first, tgt.second - prev.second)) + " m");
      }
      io_.status("NAVIGATING", idx_, 0);
      const double tx = tgt.first, ty = tgt.second;
      const bool is_final = wi == n - 1;
      const double sdx = tx - prev.first, sdy = ty - prev.second, seg_dist = std::hypot(sdx, sdy);
      const double seg_h = seg_dist > 0.02 ? std::atan2(sdy, sdx) : m_.target_yaw;
      Pose p = io_.pose();
      const double init_err = wrap(seg_h - p.th);
      const double dist_to = std::hypot(tx - p.x, ty - p.y);
      const bool need_rot = !dual() && std::fabs(init_err) > 0.02 && dist_to > 0.1;
      double rd = need_rot ? rotation_dir(p.x, p.y, p.th, seg_h) : 1.0;
      const bool reverse = !dual() && std::fabs(init_err) > 2.4 && dist_to > 0.3 && rd == 0.0;
      if (reverse)
        io_.event("REVERSE", "info", "路段 " + std::to_string(wi) + " 倒车行驶",
                  "原地转向扫掠半径 " + f2(sweep_radius()) + " m 内有障碍，改为倒车 " + f2(dist_to) + " m");
      // PHASE 1: 原地转向对准路段方向
      if (!reverse && std::fabs(init_err) > 0.02 && dist_to > 0.1) {
        wait_until_stopped();
        if (!dual() && rd == 0.0) {
          if (!back_out(p.x, p.y, p.th, seg_h)) return "ABORT";
          const Pose b = io_.pose();
          rd = rotation_dir(b.x, b.y, b.th, seg_h);
        }
        if (rd != 0.0) rot_dir = rd;
        if (!align_steer(true, (rot_dir != 0 ? rot_dir : init_err) > 0 ? 1.0 : -1.0)) return "ABORT";
        const int res = rotate_to(seg_h, rot_dir, max_w);
        if (res == 1) return "ABORT";
        if (res == 2) {
          io_.event("ROTATE_BLOCKED", "danger", "原地转向受阻",
                    "转向防护区内持续有障碍 8 s，任务终止；请清除障碍或调整工位/保护空间后重新下发");
          return "FAILED";
        }
        io_.status("NAVIGATING", idx_, 0);
      }
      if (!dual() && !align_steer(false, reverse ? -1.0 : 1.0)) return "ABORT";
      // 拐点: 圆弧过弯 / 停车原地转向
      Corner corner;
      bool corner_rot = false;
      if (!reverse && !is_final && !dual() && seg_dist > 0.05) {
        const auto nxt = W[wi + 1];
        const double n_len = std::hypot(nxt.first - tx, nxt.second - ty);
        if (n_len > 0.1) {
          const double nh = std::atan2(nxt.second - ty, nxt.first - tx), turn = wrap(nh - seg_h);
          if (std::fabs(turn) > (m_.corner_mode == "arc" ? 0.35 : 0.02)) {
            Corner c = wi < m_.corners.size() ? m_.corners[wi] : Corner();
            if (c.kind == 0) {       // 路线下发时未确定 (不应出现): 现场计算
              double out[8];
              const int mode = m_.corner_mode == "arc" ? 1 : (m_.corner_mode == "auto" ? 2 : 0);
              c.clr = an_plan_corner(m_.segs.data(), static_cast<int>(m_.segs.size() / 4), tx, ty, seg_h, nh, seg_dist, n_len,
                                     m_.head, m_.tail, m_.hw, m_.corner_radius, m_.body_margin, mode, out);
              c.kind = static_cast<int>(out[0]); c.rot = out[1]; c.R = out[2]; c.d = out[3]; c.heading = out[4];
              c.turn = out[5]; c.cx = out[6]; c.cy = out[7]; c.v = 0.35;
            }
            if (c.kind == 2) { corner_rot = true; next_rot_dir = c.rot; }
            else { corner = c; corner.v = std::min(corner.v, max_v); }
            if (c.clr < 0.05)
              io_.event("CORNER_TIGHT", "warning", "拐点 " + std::to_string(wi) + " 净空不足",
                        std::string(c.kind == 1 ? "圆弧 R=" + f2(c.R) + " m" : "拐点原地转向") + " 车体最小净空 " +
                            std::to_string(static_cast<int>(c.clr * 100)) + " cm");
          }
        }
      }
      // PHASE 2: 直线跟踪
      while (!stop_) {
        if (io_.hold()) { cmd(0, 0, 0); sleep_s(0.04); continue; }
        p = io_.pose();
        const double ch = std::cos(seg_h), sh = std::sin(seg_h);
        const double along = (p.x - tx) * ch + (p.y - ty) * sh;
        const double lateral = -(p.x - tx) * sh + (p.y - ty) * ch;
        const bool stop_here = is_final || corner_rot;
        if (corner.kind == 1) { if (along >= -corner.d - 0.003 && std::fabs(lateral) < 0.35) break; }
        else if (stop_here) {
          if (along > -0.003 && std::fabs(lateral) < 0.35) {
            if (is_final) {
              char b[160];
              std::snprintf(b, sizeof(b), "进站结束: 纵向 %+.1f mm，横向 %+.1f mm，航向差 %+.2f°", along * 1000, lateral * 1000,
                            wrap(p.th - seg_h) * 180 / M_PI);
              io_.event("APPROACH_END", "info", "末段进站结束", b);
              // 横向没收敛 (精定位修正量大时，剩余行程不够): 沿路段后退 0.4 m 重新进站，最多 2 次
              if (refined && !reverse && !dual() && std::fabs(lateral) > 0.008 && redo < 2) {
                ++redo;
                io_.event("APPROACH_RETRY", "info", "末段重新进站", "横向偏差 " + f2(lateral * 1000) + " mm > 8 mm，后退 0.4 m 再进站");
                if (!back_along(seg_h, 0.4)) return "ABORT";
                continue;
              }
            }
            break;
          }
        }
        else if (along >= 0.0 && std::fabs(lateral) < 0.6) break;
        const double cross = -(p.x - prev.first) * sh + (p.y - prev.second) * ch;
        const double dec = m_.max_decel * 0.55, v_meas = std::fabs(io_.v_meas());
        const double rem_stop = std::max(0.0, -along - (corner.kind == 1 ? corner.d : 0.0)) + (corner.kind == 1 ? 0.0 : stop_rest[wi]);
        rem_ = rem_stop;
        // 末段精定位: 最后一段剩余 < refine_dist 时停车配准一次
        if (is_final && !refined && !reverse && m_.refine && rem_stop < m_.refine_dist && rem_stop > 0.05) {
          refined = true;
          refine_here("进站前");
          continue;
        }
        const double rem_eff = rem_stop - v_meas * 0.12;
        has_left_ = !reverse;
        left_ = rem_stop;
        const double v_stop = std::min(std::sqrt(2.0 * dec * std::max(0.0, rem_eff - 0.002)), 2.0 * std::max(0.0, rem_eff));
        double vx_nom = std::min(max_v, std::max(rem_stop > 0.002 ? 0.01 : 0.0, v_stop));
        const int sgn = reverse ? -1 : 1;
        double d_hit = -1;
        double allowed = io_.allowed(sgn, !reverse, rem_stop, &d_hit);
        if (io_.photo_block(!reverse, !reverse, rem_stop)) { allowed = 0.0; d_hit = 0.0; }
        if (allowed <= 1e-6) {
          cmd(0, 0, 0);
          if (wait_since < 0) wait_since = now();
          else if (is_final && -along < m_.arrive_tol && now() - wait_since > 2.0) {
            io_.event("DOCK_LIMITED", "warning", "工位前方受限，提前停靠",
                      "距目标 " + std::to_string(static_cast<int>(std::max(0.0, -along) * 100)) + " cm 处前方障碍 " + f2(d_hit) + " m，按到位处理");
            break;
          } else if (now() - wait_since > 5.0 && m_.planner != "direct" && m_.planner != "straight" && m_.replan_left > 0) {
            io_.event("REPLAN", "warning", "阻塞超时，重新规划", "前方障碍物持续 5 s 未解除，结合动态障碍物重新计算绕行路线");
            return "REPLAN";
          }
          io_.status("OBSTACLE_WAIT", idx_, rem_stop);
          sleep_s(0.04);
          continue;
        }
        wait_since = -1;
        if (allowed < vx_nom) vx_nom = std::max(0.08, allowed);
        io_.status("NAVIGATING", idx_, rem_stop);
        const double move_h = p.th + (reverse ? M_PI : 0.0);
        if (is_final && refined && !reverse && !dual()) {
          // 精定位后的末段进站: 与 RouteController FINAL 同一控制律 —— 预瞄点在路段直线上、投影点前方 0.25 m，
          // 纯跟踪收敛横向偏差 (二阶跟踪律在低速/单舵轮舵角滞后时收敛不完，实测停车时横向残差可达 30 mm)
          const double look = 0.25;
          const double px = tx + along * ch, py = ty + along * sh;          // 车辆在路段直线上的投影
          const double cx = px + look * ch, cy = py + look * sh;
          const double bx = std::cos(p.th) * (cx - p.x) + std::sin(p.th) * (cy - p.y);
          const double by = -std::sin(p.th) * (cx - p.x) + std::cos(p.th) * (cy - p.y);
          const double vx = std::min(vx_nom, 0.2);
          const double wz = std::max(-0.4, std::min(0.4, vx * 2.0 * by / std::max(bx * bx + by * by, 1e-4)));
          cmd(vx, 0, wz);
          sleep_s(0.03);
          continue;
        }
        const double L = m_.track_L;
        const double e_th = wrap(move_h - seg_h), zeta = 0.9;
        double vx = vx_nom;
        if (std::fabs(e_th) > 0.35) vx = std::min(vx, 0.1);
        double wz = -std::max(vx, 0.02) * (2.0 * zeta / L * e_th + cross / (L * L));
        wz = std::max(-max_w, std::min(max_w, wz));
        if (dual()) {
          const double h_err = wrap(seg_h - p.th);
          wz = std::max(-std::min(max_w, 0.6), std::min(std::min(max_w, 0.6), h_err * 1.5));
          const double vy = std::max(-0.3, std::min(0.3, -cross * 1.2));
          vx = std::max(std::min(0.15, vx_nom), vx_nom * std::max(0.3, std::cos(std::min(M_PI / 2, std::fabs(h_err)))));
          cmd(vx, vy, wz);
        } else if (reverse) cmd(-std::min(vx, 0.5), 0, wz);
        else cmd(vx, 0, wz);
        sleep_s(0.03);
      }
      if (stop_) return "ABORT";
      if (corner.kind == 1 && !corner_arc(corner, max_w)) return "ABORT";
    }
    if (stop_) return "ABORT";
    has_left_ = false;
    // PHASE 3: 工位末端调姿 (按 90° 取整的正交航向)
    const double dock = std::round(m_.target_yaw / (M_PI / 2.0)) * (M_PI / 2.0);
    wait_until_stopped();
    {
      char b[120];
      std::snprintf(b, sizeof(b), "进入工位末端高精度调姿阶段，目标 Cardinal 航向: %.1f°", dock * 180 / M_PI);
      io_.event("NAV_DOCKING", "info", "工位末端调姿对齐", b);
    }
    const Pose f = io_.pose();
    bool do_rot = true;
    if (!dual() && std::fabs(wrap(dock - f.th)) > 0.3 && rotation_dir(f.x, f.y, f.th, dock) == 0.0) {
      io_.event("DOCK_SKIP_ROTATE", "warning", "工位调姿跳过",
                "原地转向扫掠半径 " + f2(sweep_radius()) + " m 内有障碍，保持当前航向");
      do_rot = false;
    }
    if (do_rot) {
      align_steer(true, 1.0);
      if (rotate_to(dock, 0.0, std::min(1.0, max_w)) == 1) return "ABORT";
      // 精定位复核朝向: 偏差 > 0.15° 再对位一次 (最多 2 次)
      for (int k = 0; k < 2 && m_.refine && !m_.refine_segs.empty(); ++k) {
        wait_until_stopped();
        const Pose before = io_.pose();
        refine_here("到位复核");
        const Pose after = io_.pose();
        if (std::fabs(wrap(dock - after.th)) <= 0.0026 || std::hypot(after.x - before.x, after.y - before.y) > 0.15) break;
        if (rotate_to(dock, 0.0, std::min(0.5, max_w), 0.003) == 1) return "ABORT";
      }
    }
    cmd(0, 0, 0);
    return "ARRIVED";
  }

  Io io_;
  Mission m_;
  std::thread th_;
  std::atomic<bool> stop_{false}, running_{false};
  int idx_ = 1;
  double rem_ = 0;
  bool has_left_ = false;
  double left_ = 0;

};

}  // namespace guide
