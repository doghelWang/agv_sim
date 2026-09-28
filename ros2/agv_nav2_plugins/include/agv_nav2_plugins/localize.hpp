// ============================================================================
// 末段精定位 (不依赖 ROS，可单独测试): 激光点 ↔ 场景静态几何 (墙/货架 = 沿线段的 5 cm 厚盒体) 的点到面 ICP
//
//   slam_toolbox 建图模式下 map 系相对场景会缓慢漂移，贴墙工位附近实测定位偏差 20~30 mm (偏置，不是抖动)，
//   停车误差基本等于该偏差。进站前停车取一帧激光，与场景几何做 Gauss-Newton 配准，得到场景坐标系下的车体位姿，
//   用它 (而不是 slam 的 map→odom) 把终点换算到 odom 系 —— slam_toolbox 本身不改，只在停车点做一次精定位。
//   残差 r = |p - 最近线段中心线| - 半厚度；内点门限分级收紧 (每级迭代到收敛再收紧，末级 4 cm ≈ 4σ 激光噪声，
//   过早收紧会把还没拉回来的点当外点丢掉而停在局部解) + Huber 权重剔除动态障碍/行人；
//   平移方向的信息矩阵最小特征值过小 (只看到一面墙) 时判为退化，不采用。
// ============================================================================
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

#include "agv_nav2_plugins/geom.hpp"

namespace agv
{

struct Seg { double x0, y0, x1, y1; };

struct IcpResult
{
  bool ok{false};
  Pose2 pose{0, 0, 0};
  int inliers{0};
  double rms{0.0};
  double min_eig{0.0};     // 平移信息矩阵最小特征值 / 内点数 (0~1，≥ 约 0.1 说明两个方向都有约束)
  const char * why{""};
};

inline double segDist(const Seg & s, double px, double py, double * nx, double * ny)
{
  const double ex = s.x1 - s.x0, ey = s.y1 - s.y0;
  const double L2 = ex * ex + ey * ey;
  const double u = L2 < 1e-12 ? 0.0 : std::clamp(((px - s.x0) * ex + (py - s.y0) * ey) / L2, 0.0, 1.0);
  const double cx = s.x0 + u * ex, cy = s.y0 + u * ey;
  const double dx = px - cx, dy = py - cy;
  const double d = std::hypot(dx, dy);
  if (d > 1e-9) {
    *nx = dx / d;
    *ny = dy / d;
  } else {               // 恰在中心线上: 取线段法向
    const double L = std::sqrt(std::max(L2, 1e-12));
    *nx = -ey / L;
    *ny = ex / L;
  }
  return d;
}

// pts: 机体系激光点；init: 场景系初值；half: 盒体半厚度
inline IcpResult icp(
  const std::vector<Pt> & pts, const std::vector<Seg> & segs, Pose2 init, double half = 0.025,
  int min_inliers = 40, double max_rms = 0.015)
{
  IcpResult res;
  res.pose = init;
  if (pts.size() < static_cast<size_t>(min_inliers) || segs.empty()) {res.why = "激光点或场景几何不足"; return res;}
  const double gates[] = {0.20, 0.10, 0.05, 0.04};
  Pose2 p = init;
  double H[3][3], g[3];
  int n_in = 0;
  double sse = 0.0;
  int stage = 0, in_stage = 0;
  for (int it = 0; it < 80; ++it) {
    const double gate = gates[stage];
    for (auto & r : H) {r[0] = r[1] = r[2] = 0.0;}
    g[0] = g[1] = g[2] = 0.0;
    n_in = 0;
    sse = 0.0;
    const double c = std::cos(p.th), s = std::sin(p.th);
    for (const auto & b : pts) {
      const double wx = p.x + c * b.x - s * b.y, wy = p.y + s * b.x + c * b.y;
      double best = 1e18, bnx = 0, bny = 0;
      for (const auto & sg : segs) {
        double nx, ny;
        const double d = segDist(sg, wx, wy, &nx, &ny);
        if (d < best) {best = d; bnx = nx; bny = ny;}
      }
      const double r = best - half;
      if (std::fabs(r) > gate) {continue;}
      const double w = std::fabs(r) < 0.01 ? 1.0 : 0.01 / std::fabs(r);     // Huber
      // dr/dθ = n · d(R·b)/dθ
      const double jt = bnx * (-s * b.x - c * b.y) + bny * (c * b.x - s * b.y);
      const double J[3] = {bnx, bny, jt};
      for (int i = 0; i < 3; ++i) {
        g[i] += w * J[i] * r;
        for (int j = 0; j < 3; ++j) {H[i][j] += w * J[i] * J[j];}
      }
      ++n_in;
      sse += r * r;
    }
    if (n_in < min_inliers) {res.why = "内点不足"; res.inliers = n_in; return res;}
    // 解 H δ = -g (3x3，Cramer)
    const double det = H[0][0] * (H[1][1] * H[2][2] - H[1][2] * H[2][1]) - H[0][1] * (H[1][0] * H[2][2] - H[1][2] * H[2][0]) +
      H[0][2] * (H[1][0] * H[2][1] - H[1][1] * H[2][0]);
    if (std::fabs(det) < 1e-12) {res.why = "配准方程奇异"; return res;}
    auto solve = [&](int k) {
        double M[3][3];
        for (int i = 0; i < 3; ++i) {for (int j = 0; j < 3; ++j) {M[i][j] = j == k ? -g[i] : H[i][j];}}
        return (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
               M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det;
      };
    const double dx = solve(0), dy = solve(1), dth = solve(2);
    p.x += dx;
    p.y += dy;
    p.th = wrap(p.th + dth);
    ++in_stage;
    if ((std::hypot(dx, dy) < 2e-5 && std::fabs(dth) < 2e-5) || in_stage >= 20) {
      if (stage == 3) {break;}
      ++stage;
      in_stage = 0;
    }
  }
  // 平移信息矩阵 (2x2) 最小特征值，按内点数归一
  const double a = H[0][0], b = H[0][1], d = H[1][1];
  const double tr = a + d, disc = std::sqrt(std::max(0.0, (a - d) * (a - d) / 4.0 + b * b));
  res.min_eig = (tr / 2.0 - disc) / std::max(1, n_in);
  res.inliers = n_in;
  res.rms = std::sqrt(sse / std::max(1, n_in));
  res.pose = p;
  if (res.rms > max_rms) {res.why = "配准残差过大"; return res;}
  if (res.min_eig < 0.05) {res.why = "几何退化 (只有一个方向的墙面)"; return res;}
  res.ok = true;
  return res;
}

}  // namespace agv
