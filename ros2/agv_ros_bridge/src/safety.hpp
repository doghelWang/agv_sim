// ============================================================================
// 执行进程安全层 (C++，不依赖 ROS)：Nav2 /cmd_vel → 本层 → 仿真指令
//   与 nav_runtime/navigator.py 的 safety_filter / _allowed_speed / _photo_sides / _on_merged (防护区走廊) 逐项对应：
//     急停 → 零速；定位停更 (Nav2 执行中，slam 定位 > loc_stale_s 未更新) → 零速；工步限速；
//     行驶防护区按速度分档 (末段进站时按剩余行程缩短，同时速度封顶 √(2·a·剩余))；原地转向扫掠防护；
//     光电: 检测点在保护包络内才响应，距停车点很近时屏蔽前向光电
//   原地转向扫掠检查采用 agv_nav2_plugins 的约定: 进入车体本身即受阻，外扩区只看新进入的点
//   (Python 版起始已在外扩区内的点整个忽略，贴墙时向墙转也不算受阻)
// ============================================================================
#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace agvsafe {

struct Field { std::string name; double v_max = 9.0, front = 0.3, rear = 0.2, side = 0.08; };

struct Photo { std::string name, di; double x = 0, y = 0, yaw = 0; };

struct Config {
  bool enabled = true;
  std::vector<Field> fields;
  double slow_ratio = 2.0, rotate_margin = 0.02, rotate_lookahead = 0.25, docking_front = 0.02;
  std::string photo_mode = "field";
  double photo_front = 0.3, photo_rear = 0.3, photo_side = 0.1, mute_near_stop = 0.1;
  double h = 0.6, t = 0.6, l = 0.4, r = 0.4;       // 当前外形 (带载时 = 车体 ∪ 负载)
  double max_decel = 0.5;
  double loc_stale_s = 1.0;
  std::vector<Photo> photos;
};

struct Pt { double x, y; };

// 输入 (每次过滤时的环境)
struct Env {
  bool estop = false;
  bool loc_check = false;          // Nav2 执行中且外部定位栈 (slam_toolbox) 在运行
  double loc_age = 0.0;            // 定位最近一次更新距今 (s)
  double speed_cap = 0.0;          // >0 生效
  bool has_left = false;           // 末段进站剩余行程
  double approach_left = 0.0;
  bool in_arc = false;
  double v_meas = 0.0;             // 当前速度 (选档)
  std::vector<std::pair<double, double>> bands;   // 各档走廊内最近障碍 (前, 后)，<0 = 无
  const std::vector<Pt> *pts = nullptr;           // 融合扫描点 (机体系)
  // 触发的光电: 名称 → 检测距离 (<0 = 未知，保守响应)
  std::vector<std::pair<const Photo *, double>> photo_hits;
};

struct Result {
  double vx = 0.0, vy = 0.0, wz = 0.0;
  std::string zone = "clear", layer;    // clear/warn/slow/stop；field_front/field_rear/rotate
  double d_hit = -1, need = -1;
  bool loc_stale = false;
  std::vector<std::string> photo_front, photo_rear, photo_left, photo_right, photo_ignored;
};

inline int field_for_speed(const Config &c, double v) {
  v = std::fabs(v);
  for (size_t i = 0; i < c.fields.size(); ++i)
    if (v <= c.fields[i].v_max + 1e-6) return static_cast<int>(i);
  return static_cast<int>(c.fields.size()) - 1;
}

// 各档防护区走廊内最近障碍 (机体系；走廊 = 外形两侧外扩 side；距离从车头/车尾算起)
inline std::vector<std::pair<double, double>> compute_bands(const Config &c, const std::vector<Pt> &pts) {
  std::vector<std::pair<double, double>> out;
  for (const auto &f : c.fields) {
    double fr = 1e9, rr = 1e9;
    for (const auto &p : pts) {
      if (p.y > c.l + f.side || p.y < -(c.r + f.side)) continue;
      if (p.x > 0) { double d = p.x - c.h; if (d > -0.05) fr = std::min(fr, d); }
      else { double d = -p.x - c.t; if (d > -0.05) rr = std::min(rr, d); }
    }
    auto rnd = [](double d) { return d >= 1e8 ? -1.0 : std::round(std::max(0.0, d) * 100.0) / 100.0; };
    out.emplace_back(rnd(fr), rnd(rr));
  }
  return out;
}

// 按防护区分档求允许速度: 取停车距离放得下的最高一档的 v_max；最低档都放不下 → 0
inline double allowed_speed(const Config &c, const Env &e, int sign, bool use_left, double left, double *d_hit, double *need_hit) {
  double allowed = -1.0;
  *d_hit = *need_hit = -1.0;
  for (size_t i = 0; i < c.fields.size(); ++i) {
    const auto &f = c.fields[i];
    double d = -1.0;
    if (i < e.bands.size()) d = sign > 0 ? e.bands[i].first : e.bands[i].second;
    double need = sign > 0 ? f.front : f.rear;
    if (use_left) need = std::min(need, left + c.docking_front);
    if (d >= 0.0 && d < need) { *d_hit = d; *need_hit = need; break; }
    allowed = i + 1 < c.fields.size() ? f.v_max : 99.0;
  }
  return allowed < 0 ? 0.0 : allowed;
}

// 原地转向扫掠 (外形外扩 rotate_margin，向 dir 转 rotate_lookahead 的 1/3、2/3、全程)
inline bool rotation_blocked(const Config &c, const std::vector<Pt> &pts, double dir) {
  const double m = c.rotate_margin;
  auto in = [&](double x, double y, double mm) { return x < c.h + mm && x > -c.t - mm && y < c.l + mm && y > -c.r - mm; };
  for (const auto &p : pts) {
    if (in(p.x, p.y, 0.0)) continue;                      // 起始已在车体内: 噪声
    const bool in_m0 = in(p.x, p.y, m);
    for (double k : {0.33, 0.66, 1.0}) {
      const double phi = dir * c.rotate_lookahead * k, co = std::cos(phi), s = std::sin(phi);
      const double lx = p.x * co + p.y * s, ly = -p.x * s + p.y * co;
      if (in(lx, ly, 0.0) || (!in_m0 && in(lx, ly, m))) return true;
    }
  }
  return false;
}

// 光电分组: 检测点落在保护包络内的才响应
inline void photo_sides(const Config &c, const Env &e, int band, bool skip_diag, Result &r) {
  bool env = c.photo_mode != "always";
  double H = 0, T = 0, L = 0, R = 0;
  if (env) {
    double fr, rr, sd;
    if (c.photo_mode == "custom") { fr = c.photo_front; rr = c.photo_rear; sd = c.photo_side; }
    else {
      const auto &f = c.fields[std::max(0, std::min(band, static_cast<int>(c.fields.size()) - 1))];
      fr = f.front; rr = f.rear; sd = f.side;
    }
    H = c.h + fr; T = c.t + rr; L = c.l + sd; R = c.r + sd;
  }
  const bool mute = e.has_left && e.approach_left < c.mute_near_stop;
  for (const auto &hit : e.photo_hits) {
    const Photo &p = *hit.first;
    const double yaw = p.yaw, a = std::fabs(std::atan2(std::sin(yaw), std::cos(yaw)));
    if (skip_diag && a > 0.3 && a < 2.8) continue;
    const double co = std::cos(yaw), s = std::sin(yaw);
    std::vector<std::string> *side = co > 0.3 ? &r.photo_front : (co < -0.3 ? &r.photo_rear : (s > 0 ? &r.photo_left : &r.photo_right));
    if (side == &r.photo_front && mute) { r.photo_ignored.push_back(p.name); continue; }
    if (env && hit.second >= 0.0) {
      const double hx = p.x + hit.second * co, hy = p.y + hit.second * s;
      if (!(-T <= hx && hx <= H && -R <= hy && hy <= L)) { r.photo_ignored.push_back(p.name); continue; }
    }
    side->push_back(p.name);
  }
}

inline Result filter(const Config &c, const Env &e, double vx, double vy, double wz) {
  Result r;
  r.vx = vx;
  r.vy = vy;
  r.wz = wz;
  if (e.estop) { r.vx = r.vy = r.wz = 0.0; return r; }
  if (e.loc_check && e.loc_age > c.loc_stale_s) { r.vx = r.vy = r.wz = 0.0; r.loc_stale = true; return r; }
  if (e.speed_cap > 0.0) {
    const double v = std::hypot(r.vx, r.vy);
    if (v > e.speed_cap) { r.vx *= e.speed_cap / v; r.vy *= e.speed_cap / v; }
  }
  const int band_now = c.fields.empty() ? 0 : field_for_speed(c, e.v_meas);
  if (c.enabled && !c.fields.empty() && std::fabs(r.vx) > 1e-3 && !(e.in_arc && r.vx > 0)) {
    const int sign = r.vx > 0 ? 1 : -1;
    const bool use_left = sign > 0 && e.has_left;
    double d_hit, need;
    double allowed = allowed_speed(c, e, sign, use_left, e.approach_left, &d_hit, &need);
    if (use_left && allowed > 0) {
      double dd, nn;
      const double full = allowed_speed(c, e, sign, false, 0.0, &dd, &nn);
      if (full < allowed) {
        const double a = 0.5 * c.max_decel;
        const double v_stop = e.approach_left > 0.002 ? std::max(0.01, std::sqrt(2.0 * a * e.approach_left)) : 0.0;
        allowed = std::min(allowed, std::max(full, v_stop));
      }
    }
    if (allowed <= 1e-6) { r.vx = 0.0; r.zone = "stop"; r.layer = sign > 0 ? "field_front" : "field_rear"; }
    else if (std::fabs(r.vx) > allowed) { r.vx = sign * allowed; r.zone = "slow"; r.layer = sign > 0 ? "field_front" : "field_rear"; }
    else {
      const int i = field_for_speed(c, r.vx);
      const auto &f = c.fields[i];
      double d = -1.0;
      if (i < static_cast<int>(e.bands.size())) d = sign > 0 ? e.bands[i].first : e.bands[i].second;
      if (d >= 0.0 && d < (sign > 0 ? f.front : f.rear) * c.slow_ratio) r.zone = "warn";
    }
    r.d_hit = d_hit;
    r.need = need;
  } else if (c.enabled && std::fabs(r.vx) < 0.05 && std::fabs(r.wz) > 0.05) {
    if (e.pts && rotation_blocked(c, *e.pts, r.wz > 0 ? 1.0 : -1.0)) { r.wz = 0.0; r.zone = "stop"; r.layer = "rotate"; }
  } else {
    r.zone = "";                                   // 不涉及防护区的指令: 保持原状态
  }
  photo_sides(c, e, band_now, e.in_arc, r);
  if (!r.photo_front.empty() && r.vx > 0) r.vx = 0.0;
  if (!r.photo_rear.empty() && r.vx < 0) r.vx = 0.0;
  if (!r.photo_left.empty() && r.vy > 0) r.vy = 0.0;
  if (!r.photo_right.empty() && r.vy < 0) r.vy = 0.0;
  return r;
}

}  // namespace agvsafe
