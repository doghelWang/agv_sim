// ============================================================================
// 扫掠判定的精确解 (不依赖 ROS)：点绕某个中心转一段圆弧 / 沿直线平移一段，是否进入矩形
//
//   车体原地转向 (绕控制点)、沿圆弧过弯 (绕瞬心)、直线平移时，激光点在车体系下的轨迹分别是
//   圆弧 / 圆弧 / 线段。原先按固定步长采样 (0.05 rad、3 个角度、0.15 rad、3 cm) 判断，车角半径 1.4 m 时
//   每步移动 7~13 cm，比 2~3 cm 的外扩余量还大，贴边的点会在两个采样之间"穿过"外扩带而漏判；
//   且每个点每一步都要重新算 sin/cos。这里用圆 (线段) 与矩形四条边求交，每个点常数次运算且无漏判。
//
//   约定：矩形为开区间 (x0, x1) × (y0, y1)，与 Rect::contains 一致 (恰好压在边界上不算进入)。
// ============================================================================
#pragma once
#include <algorithm>
#include <cmath>

namespace agv
{
namespace sweep
{

struct Box
{
  double x0, x1, y0, y1;
  bool in(double x, double y) const {return x > x0 && x < x1 && y > y0 && y < y1;}
};

// 点 (px, py) 绕中心 (cx, cy) 转过带符号角 phi (逆时针为正) 的圆弧，是否进入 (或起点就在) 矩形内
inline bool arcHitsBox(double px, double py, double cx, double cy, double phi, const Box & b)
{
  if (b.in(px, py)) {return true;}
  const double ux = px - cx, uy = py - cy;
  const double r2 = ux * ux + uy * uy;
  // 剪枝: 圆比矩形最远角还远 → 不可能进入 (绝大多数激光点在这里返回，不做三角运算)
  const double fx = std::max(std::fabs(b.x0 - cx), std::fabs(b.x1 - cx)), fy = std::max(std::fabs(b.y0 - cy), std::fabs(b.y1 - cy));
  if (r2 >= fx * fx + fy * fy) {return false;}
  const double rho = std::sqrt(r2);
  if (rho < 1e-12 || std::fabs(phi) < 1e-12) {return false;}
  const double a0 = std::atan2(uy, ux);
  if (b.in(cx + rho * std::cos(a0 + phi), cy + rho * std::sin(a0 + phi))) {return true;}
  const double span = std::fabs(phi);
  const bool full = span >= 2.0 * M_PI;
  // 角 a 是否落在从 a0 出发、沿 phi 方向转过 span 的弧上
  auto onArc = [&](double a) {
      if (full) {return true;}
      double t = phi >= 0.0 ? a - a0 : a0 - a;
      t = std::fmod(t, 2.0 * M_PI);
      if (t < 0.0) {t += 2.0 * M_PI;}
      return t <= span;
    };
  for (double X : {b.x0, b.x1}) {                  // 竖边 x = X
    const double dx = X - cx;
    if (std::fabs(dx) > rho) {continue;}
    const double dy = std::sqrt(r2 - dx * dx);
    for (double sy : {dy, -dy}) {
      const double y = cy + sy;
      if (y > b.y0 && y < b.y1 && onArc(std::atan2(sy, dx))) {return true;}
    }
  }
  for (double Y : {b.y0, b.y1}) {                  // 横边 y = Y
    const double dy = Y - cy;
    if (std::fabs(dy) > rho) {continue;}
    const double dx = std::sqrt(r2 - dy * dy);
    for (double sx : {dx, -dx}) {
      const double x = cx + sx;
      if (x > b.x0 && x < b.x1 && onArc(std::atan2(dy, sx))) {return true;}
    }
  }
  return false;
}

// 点从 (px, py) 平移 (dx, dy) 的线段是否进入 (或起点就在) 矩形内 (Liang–Barsky)
inline bool segHitsBox(double px, double py, double dx, double dy, const Box & b)
{
  double t0 = 0.0, t1 = 1.0;
  const double p[4] = {-dx, dx, -dy, dy}, q[4] = {px - b.x0, b.x1 - px, py - b.y0, b.y1 - py};
  for (int i = 0; i < 4; ++i) {
    if (std::fabs(p[i]) < 1e-15) {
      if (q[i] <= 0.0) {return false;}              // 平行且在该边外 (或恰在边上: 开区间不算)
      continue;
    }
    const double r = q[i] / p[i];
    if (p[i] < 0.0) {if (r > t1) {return false;} if (r > t0) {t0 = r;}} else {if (r < t0) {return false;} if (r < t1) {t1 = r;}}
  }
  return t0 < t1;                                     // 只擦过一个角点 (t0 == t1) 不算进入
}

}  // namespace sweep
}  // namespace agv
