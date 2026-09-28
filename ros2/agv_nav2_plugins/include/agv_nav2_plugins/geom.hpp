// ============================================================================
// agv_nav2_plugins 几何内核 (不依赖 ROS，可单独测试: test/test_geom.cpp)
//
//   车体矩形 (机体系，原点 = 控制点): x ∈ [-tail, head]，y ∈ [-right, left]
//   · 激光点扫掠检查: 原地转向 / 直线平移时是否扫到激光点。一个点受阻的条件:
//       进入车体本身 (起始时已在车体内的点视为噪声忽略)，或
//       进入车体外扩 margin 的区域且起始时不在该区域 (贴墙时车角离墙 < margin 仍允许远离/平行的动作)
//   · 路线稠密化: 拓扑折线 → 直线 + 过渡圆弧 (车体净空满足时) / 拐点停车原地转向 (尖点: 同一位置两个朝向)
//     圆弧候选半径与 planning/maneuver.py plan_corner 一致 (1.25/1/0.7/0.45 倍首选半径，受相邻路段长度限制)
// ============================================================================
#pragma once
#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace agv
{

inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

struct Pt { double x, y; };
struct Pose2 { double x, y, th; };

struct Rect
{
  double head{0.5}, tail{0.5}, left{0.4}, right{0.4};
  bool contains(double lx, double ly, double m) const {return contains(lx, ly, m, m);}
  // mx: 前后方向外扩，my: 左右方向外扩
  bool contains(double lx, double ly, double mx, double my) const
  {
    return lx < head + mx && lx > -tail - mx && ly < left + my && ly > -right - my;
  }
};

// 点 p (机体系，当前位姿) 在车体处于相对位姿 (dx, dy, dth) 时的车体系坐标
inline Pt toPose(const Pt & p, double dx, double dy, double dth)
{
  const double c = std::cos(dth), s = std::sin(dth);
  const double qx = p.x - dx, qy = p.y - dy;
  return {qx * c + qy * s, -qx * s + qy * c};
}

// 车体从相对位姿 (dx, dy, th0) 原地转 delta (带符号) 的扫掠区是否有新的点
inline bool rotationBlocked(
  const std::vector<Pt> & pts, const Rect & r, double m, double dx, double dy, double th0, double delta,
  double step = 0.05)
{
  if (std::fabs(delta) < 1e-4) {return false;}
  const int n = std::max(2, static_cast<int>(std::ceil(std::fabs(delta) / step)));
  for (const auto & p : pts) {
    const Pt q0 = toPose(p, dx, dy, th0);
    if (r.contains(q0.x, q0.y, 0.0)) {continue;}        // 起始已在车体内: 噪声
    const bool in_m0 = r.contains(q0.x, q0.y, m);
    for (int k = 1; k <= n; ++k) {
      const Pt q = toPose(p, dx, dy, th0 + delta * k / n);
      if (r.contains(q.x, q.y, 0.0) || (!in_m0 && r.contains(q.x, q.y, m))) {return true;}
    }
  }
  return false;
}

// 诊断用: 原地转 delta 过程中车体到激光点的最小距离 (m；点进入车体为负，起始已在车体内的点忽略)
inline double rotationClearance(
  const std::vector<Pt> & pts, const Rect & r, double dx, double dy, double th0, double delta, double step = 0.05)
{
  const int n = std::max(2, static_cast<int>(std::ceil(std::fabs(delta) / step)));
  double best = 1e9;
  for (const auto & p : pts) {
    const Pt q0 = toPose(p, dx, dy, th0);
    if (r.contains(q0.x, q0.y, 0.0)) {continue;}
    for (int k = 0; k <= n; ++k) {
      const Pt q = toPose(p, dx, dy, th0 + delta * k / n);
      const double ox = std::max({-r.tail - q.x, 0.0, q.x - r.head}), oy = std::max({-r.right - q.y, 0.0, q.y - r.left});
      const double d = (ox == 0.0 && oy == 0.0) ?
        -std::min({r.head - q.x, q.x + r.tail, r.left - q.y, q.y + r.right}) : std::hypot(ox, oy);
      best = std::min(best, d);
    }
  }
  return best;
}

// 车体从相对位姿 (0, 0, th) 沿自身朝向平移 dist (负 = 后退) 的扫掠区是否有新的点
// (外扩只加在行驶方向的前后端；左右用车体本身 —— 贴着墙平行后退/前进不算受阻)
inline bool translationBlocked(
  const std::vector<Pt> & pts, const Rect & r, double m, double th, double dist, double step = 0.03)
{
  if (std::fabs(dist) < 1e-4) {return false;}
  const int n = std::max(2, static_cast<int>(std::ceil(std::fabs(dist) / step)));
  const double c = std::cos(th), s = std::sin(th);
  for (const auto & p : pts) {
    const Pt q0 = toPose(p, 0.0, 0.0, th);
    if (r.contains(q0.x, q0.y, 0.0)) {continue;}
    const bool in_m0 = r.contains(q0.x, q0.y, m, 0.0);
    for (int k = 1; k <= n; ++k) {
      const double d = dist * k / n;
      const Pt q = toPose(p, d * c, d * s, th);
      if (r.contains(q.x, q.y, 0.0) || (!in_m0 && r.contains(q.x, q.y, m, 0.0))) {return true;}
    }
  }
  return false;
}

// ---------------------------------------------------------------- 路线稠密化
struct RouteOptions
{
  double step{0.05};           // 直线/圆弧采样间距 (m)
  double r_pref{0.9};          // 首选过弯半径 (m)
  double arc_min_turn{0.35};   // 小于该转角: 直接折线 (控制器平滑切过，不停车)
  double arc_max_turn{2.6};    // 大于该转角 (掉头): 只能原地转向
  bool use_arcs{true};
  double r_min{0.25};
};

// 过弯决策 (供测试与调用方查看)
struct Corner { int kind{0}; double R{0.0}; };   // 0 折线 1 圆弧 2 原地转向

// free(x, y, th): 车体在该位姿无碰撞
inline std::vector<Pose2> densify(
  const std::vector<Pt> & pts, double final_yaw, const RouteOptions & o,
  const std::function<bool(double, double, double)> & free, std::vector<Corner> * corners = nullptr)
{
  std::vector<Pose2> out;
  const size_t n = pts.size();
  if (n == 0) {return out;}
  if (n == 1) {
    out.push_back({pts[0].x, pts[0].y, final_yaw});
    return out;
  }
  auto line = [&](Pt a, Pt b, double h) {
      const double L = std::hypot(b.x - a.x, b.y - a.y);
      const int k = std::max(1, static_cast<int>(std::ceil(L / o.step)));
      for (int i = 1; i <= k; ++i) {
        out.push_back({a.x + (b.x - a.x) * i / k, a.y + (b.y - a.y) * i / k, h});
      }
    };
  auto head = [&](size_t i) {return std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);};
  out.push_back({pts[0].x, pts[0].y, head(0)});
  Pt cur = pts[0];
  if (corners) {corners->assign(n, Corner{});}
  for (size_t i = 1; i < n; ++i) {
    const Pt b = pts[i];
    const double h_in = std::atan2(b.y - pts[i - 1].y, b.x - pts[i - 1].x);
    if (i == n - 1) {
      line(cur, b, h_in);
      break;
    }
    const double h_out = head(i);
    const double turn = wrap(h_out - h_in);
    const double at = std::fabs(turn);
    if (at < 0.02) {                     // 共线
      line(cur, b, h_in);
      cur = b;
      continue;
    }
    if (at < o.arc_min_turn) {           // 小折角: 折线，控制器切过
      line(cur, b, h_in);
      cur = b;
      continue;
    }
    bool done = false;
    if (o.use_arcs && at <= o.arc_max_turn) {
      const double len_in = std::hypot(b.x - cur.x, b.y - cur.y);
      const double len_out = std::hypot(pts[i + 1].x - b.x, pts[i + 1].y - b.y);
      const double t_half = std::tan(at / 2.0);
      const double r_lim = 0.45 * std::min(len_in, len_out) / std::max(t_half, 1e-3);
      const double r_max = std::min(o.r_pref, r_lim);
      std::vector<double> cands = {std::min(o.r_pref * 1.25, r_lim), r_max, r_max * 0.7, r_max * 0.45};
      std::sort(cands.begin(), cands.end(), std::greater<double>());
      const double sgn = turn > 0 ? 1.0 : -1.0;
      for (double R : cands) {
        if (R < o.r_min) {continue;}
        const double d = R * t_half;
        const double sx = b.x - d * std::cos(h_in), sy = b.y - d * std::sin(h_in);
        const double cx = sx - sgn * R * std::sin(h_in), cy = sy + sgn * R * std::cos(h_in);
        const int N = std::max(6, static_cast<int>(std::ceil(at * R / o.step)));
        std::vector<Pose2> arc;
        bool ok = true;
        for (int k = 1; k <= N && ok; ++k) {
          const double th = h_in + turn * k / N;
          Pose2 p{cx + sgn * R * std::sin(th), cy - sgn * R * std::cos(th), th};
          ok = free(p.x, p.y, p.th);
          arc.push_back(p);
        }
        if (!ok || !free(sx, sy, h_in)) {continue;}
        line(cur, {sx, sy}, h_in);
        out.insert(out.end(), arc.begin(), arc.end());
        cur = {arc.back().x, arc.back().y};
        if (corners) {(*corners)[i] = {1, R};}
        done = true;
        break;
      }
    }
    if (!done) {                         // 拐点停车原地转向: 尖点 (同一位置，出向朝向)
      line(cur, b, h_in);
      out.push_back({b.x, b.y, h_out});
      cur = b;
      if (corners) {(*corners)[i] = {2, 0.0};}
    }
  }
  const Pose2 last = out.back();
  if (std::fabs(wrap(final_yaw - last.th)) > 0.01) {
    out.push_back({last.x, last.y, final_yaw});   // 终点对位转向 (尖点)
  }
  return out;
}

// 按尖点 (相邻两点位置重合、朝向不同) 切成若干段；每段至少一个点
inline std::vector<std::vector<Pose2>> splitCusps(const std::vector<Pose2> & path, double pos_eps = 1e-3)
{
  std::vector<std::vector<Pose2>> pieces(1);
  for (size_t i = 0; i < path.size(); ++i) {
    if (i > 0 && std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y) < pos_eps &&
      std::fabs(wrap(path[i].th - path[i - 1].th)) > 0.02 && !pieces.back().empty())
    {
      pieces.emplace_back();
    }
    pieces.back().push_back(path[i]);
  }
  return pieces;
}

// 段的行驶方向: 起点到第一个离起点 ≥ look 的点 (段内只有一个点时用其朝向)
inline double pieceHeading(const std::vector<Pose2> & piece, double look = 0.15)
{
  for (const auto & p : piece) {
    if (std::hypot(p.x - piece.front().x, p.y - piece.front().y) >= look) {
      return std::atan2(p.y - piece.front().y, p.x - piece.front().x);
    }
  }
  if (piece.size() >= 2) {
    const auto & a = piece[piece.size() - 2];
    const auto & b = piece.back();
    if (std::hypot(b.x - a.x, b.y - a.y) > 1e-4) {return std::atan2(b.y - a.y, b.x - a.x);}
  }
  return piece.front().th;
}

// 从位置 (x, y) 沿段剩余的行程 (最近点 + 投影)；hint: 最近点搜索起点 (单调前进)
inline double remaining(const std::vector<Pose2> & piece, double x, double y, size_t * hint = nullptr)
{
  if (piece.size() < 2) {return std::hypot(piece.front().x - x, piece.front().y - y);}
  size_t i0 = hint ? *hint : 0, best = i0;
  double bd = 1e18;
  for (size_t i = i0; i < piece.size(); ++i) {
    const double d = std::hypot(piece[i].x - x, piece[i].y - y);
    if (d < bd) {bd = d; best = i;}
  }
  if (hint) {*hint = best;}
  double tail = 0.0;
  for (size_t i = best; i + 1 < piece.size(); ++i) {
    tail += std::hypot(piece[i + 1].x - piece[i].x, piece[i + 1].y - piece[i].y);
  }
  const size_t a = best < piece.size() - 1 ? best : best - 1;
  const double ex = piece[a + 1].x - piece[a].x, ey = piece[a + 1].y - piece[a].y;
  const double L = std::max(std::hypot(ex, ey), 1e-9);
  const double proj = ((x - piece[best].x) * ex + (y - piece[best].y) * ey) / L;
  return std::max(0.0, tail - proj);
}

}  // namespace agv
