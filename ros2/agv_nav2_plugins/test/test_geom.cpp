// agv_nav2_plugins 几何内核单测 (不依赖 ROS): g++ -std=c++17 -Iinclude test/test_geom.cpp && ./a.out
#include <cmath>
#include <cstdio>
#include <vector>

#include "agv_nav2_plugins/geom.hpp"
#include "agv_nav2_plugins/localize.hpp"
#include <random>

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { ++fails; std::printf("FAIL %s:%d ", __FILE__, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

int main()
{
  using namespace agv;
  Rect body{0.6, 0.4, 0.35, 0.35};

  // ---- 原地转向扫掠: 墙在车头前方 5 cm (起始已在 margin 外) 时，转向必然扫到 (车角外摆)
  {
    std::vector<Pt> wall;
    for (double y = -1.0; y <= 1.0; y += 0.02) {wall.push_back({0.65, y});}
    CHECK(rotationBlocked(wall, body, 0.02, 0, 0, 0, 0.5), "前墙 5 cm 转向应受阻");
    CHECK(!translationBlocked(wall, body, 0.02, 0.0, -0.3), "后退远离前墙不应受阻");
    CHECK(translationBlocked(wall, body, 0.02, 0.0, 0.1), "前进 10 cm 应撞前墙");
    // 后退 0.4 m 后 (车角扫掠半径 hypot(0.6,0.35)=0.695 < 1.05) 可以转
    CHECK(!rotationBlocked(wall, body, 0.02, -0.4, 0, 0, 1.57), "后退 0.4 m 后应可转 90°");
  }
  // ---- 起始已在外扩区内的点不算 (贴墙 1 cm 平行行驶的墙)
  {
    std::vector<Pt> side;
    for (double x = -2.0; x <= 2.0; x += 0.02) {side.push_back({x, 0.36});}
    CHECK(!translationBlocked(side, body, 0.02, 0.0, 0.5), "平行贴墙前进不应受阻");
    CHECK(rotationBlocked(side, body, 0.02, 0, 0, 0, 0.3), "贴墙转向应受阻");
  }
  // ---- 稠密化: 直角拐点，空旷 → 圆弧；全阻 → 尖点
  {
    std::vector<Pt> route{{0, 0}, {3, 0}, {3, 3}};
    RouteOptions o;
    o.r_pref = 0.9;
    std::vector<Corner> cs;
    auto p = densify(route, M_PI / 2, o, [](double, double, double) {return true;}, &cs);
    CHECK(cs[1].kind == 1 && std::fabs(cs[1].R - 1.125) < 1e-9, "空旷拐点应走 1.25 倍半径圆弧 (kind=%d R=%.3f)", cs[1].kind, cs[1].R);
    auto pieces = splitCusps(p);
    CHECK(pieces.size() == 1, "圆弧路径不应有尖点 (%zu 段)", pieces.size());
    // 路径连续: 相邻点间距 ≤ step + ε
    double mx = 0;
    for (size_t i = 1; i < p.size(); ++i) {mx = std::max(mx, std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y));}
    CHECK(mx <= o.step + 1e-6, "最大点距 %.4f", mx);
    CHECK(std::fabs(p.back().x - 3) < 1e-9 && std::fabs(p.back().y - 3) < 1e-9, "终点");
    // 圆弧中点离拐点 (3,0) 的距离 = R(1/cos(45°) - 1)
    std::vector<Corner> cs2;
    auto q = densify(route, M_PI / 2, o, [](double, double, double) {return false;}, &cs2);
    CHECK(cs2[1].kind == 2, "全阻应原地转向");
    auto pq = splitCusps(q);
    CHECK(pq.size() == 2, "尖点应切成 2 段 (%zu)", pq.size());
    CHECK(std::fabs(pieceHeading(pq[1]) - M_PI / 2) < 1e-6, "第二段方向 90°");
    CHECK(std::fabs(pq[0].back().x - 3) < 1e-9 && std::fabs(pq[1].front().y) < 1e-9, "尖点位置");
    // 终点朝向与到达方向不同 → 终点尖点 (单点段)
    auto r = densify(route, M_PI, o, [](double, double, double) {return true;});
    auto pr = splitCusps(r);
    CHECK(pr.size() == 2 && pr.back().size() == 1 && std::fabs(pr.back()[0].th - M_PI) < 1e-9, "终点对位尖点");
    // 掉头 (转角 > arc_max_turn) → 原地转向
    std::vector<Corner> cs3;
    densify({{0, 0}, {2, 0}, {0, 0.05}}, 0, o, [](double, double, double) {return true;}, &cs3);
    CHECK(cs3[1].kind == 2, "掉头应原地转向");
    // 小折角 → 折线
    std::vector<Corner> cs4;
    densify({{0, 0}, {2, 0}, {4, 0.3}}, 0, o, [](double, double, double) {return true;}, &cs4);
    CHECK(cs4[1].kind == 0, "小折角折线");
    // 短路段限制半径: 0.45·min(len)/tan(45°)
    std::vector<Corner> cs5;
    densify({{0, 0}, {1, 0}, {1, 1}}, 0, o, [](double, double, double) {return true;}, &cs5);
    CHECK(cs5[1].kind == 1 && std::fabs(cs5[1].R - 0.45) < 1e-9, "短路段半径 0.45 (R=%.3f)", cs5[1].R);
  }
  // ---- 剩余行程
  {
    std::vector<Pose2> seg;
    for (int i = 0; i <= 20; ++i) {seg.push_back({0.1 * i, 0, 0});}
    size_t h = 0;
    CHECK(std::fabs(remaining(seg, 0.55, 0.1, &h) - 1.45) < 1e-9, "剩余 1.45 (%.4f)", remaining(seg, 0.55, 0.1));
    CHECK(std::fabs(remaining(seg, 2.05, 0.0) - 0.0) < 1e-9, "越过终点剩余 0");
    CHECK(std::fabs(remaining(seg, 1.97, 0.0) - 0.03) < 1e-9, "终点前 3 cm");
  }
  // ---- 末段精定位 ICP: 20 m 方形房间 + 一个货架，射线求交生成激光 (墙为 5 cm 厚盒体)，加噪声与一个行人
  {
    std::vector<Seg> segs{{-10, -10, 10, -10}, {10, -10, 10, 10}, {10, 10, -10, 10}, {-10, 10, -10, -10},
      {2, 5, 6, 5}, {6, 5, 6, 6}, {6, 6, 2, 6}, {2, 6, 2, 5}};
    // 盒体表面 = 中心线两侧各偏 0.025 的线段 (端面忽略)
    std::vector<Seg> faces;
    for (const auto & g : segs) {
      const double L = std::hypot(g.x1 - g.x0, g.y1 - g.y0), nx = -(g.y1 - g.y0) / L * 0.025, ny = (g.x1 - g.x0) / L * 0.025;
      faces.push_back({g.x0 + nx, g.y0 + ny, g.x1 + nx, g.y1 + ny});
      faces.push_back({g.x0 - nx, g.y0 - ny, g.x1 - nx, g.y1 - ny});
    }
    auto cast = [&](Pose2 t, std::mt19937 & rng, double sigma) {
        std::normal_distribution<double> nd(0.0, sigma);
        std::vector<Pt> pts;
        for (int k = 0; k < 720; ++k) {
          const double a = -M_PI + 2 * M_PI * k / 720.0, wa = t.th + a;
          const double dx = std::cos(wa), dy = std::sin(wa);
          double best = 1e9;
          for (const auto & f : faces) {       // 射线 vs 线段
            const double ex = f.x1 - f.x0, ey = f.y1 - f.y0, den = dx * ey - dy * ex;
            if (std::fabs(den) < 1e-12) {continue;}
            const double qx = f.x0 - t.x, qy = f.y0 - t.y;
            const double tt = (qx * ey - qy * ex) / den, uu = (qx * dy - qy * dx) / den;
            if (tt > 0.05 && uu >= 0 && uu <= 1) {best = std::min(best, tt);}
          }
          // 行人 (圆柱 r=0.25) 在车前 1.5 m
          const double px = t.x + 1.5 * std::cos(t.th), py = t.y + 1.5 * std::sin(t.th);
          const double b = dx * (t.x - px) + dy * (t.y - py), cc = std::pow(t.x - px, 2) + std::pow(t.y - py, 2) - 0.0625;
          if (b * b - cc > 0 && -b - std::sqrt(b * b - cc) > 0) {best = std::min(best, -b - std::sqrt(b * b - cc));}
          if (best > 25) {continue;}
          const double r = best + nd(rng);
          pts.push_back({r * std::cos(a), r * std::sin(a)});
        }
        return pts;
      };
    std::mt19937 rng(7);
    const Pose2 truth{4.3, 7.2, 1.5708};
    auto pts = cast(truth, rng, 0.005);
    auto res = icp(pts, segs, {truth.x + 0.03, truth.y - 0.02, truth.th + 0.026});
    CHECK(res.ok, "ICP 应成功 (%s)", res.why);
    CHECK(std::hypot(res.pose.x - truth.x, res.pose.y - truth.y) < 0.002, "ICP 位置误差 %.2f mm",
      1000 * std::hypot(res.pose.x - truth.x, res.pose.y - truth.y));
    CHECK(std::fabs(wrap(res.pose.th - truth.th)) < 0.002, "ICP 角度误差 %.3f°", wrap(res.pose.th - truth.th) * 180 / M_PI);
    std::printf("  ICP: 误差 %.2f mm / %.3f°，内点 %d，rms %.1f mm，min_eig %.2f\n",
      1000 * std::hypot(res.pose.x - truth.x, res.pose.y - truth.y), wrap(res.pose.th - truth.th) * 180 / M_PI,
      res.inliers, res.rms * 1000, res.min_eig);
    // 退化: 只有一面长墙 → 拒绝
    std::vector<Seg> one{{-50, 10, 50, 10}};
    std::vector<Pt> line;
    for (double x = -5; x <= 5; x += 0.02) {line.push_back({x, 10 - 7.2 - 0.025});}
    auto r2 = icp(line, one, {0, 7.2, 0});
    CHECK(!r2.ok, "单面墙应判退化 (min_eig %.3f)", r2.min_eig);
  }
  // ---- 倒车段: 窄巷道工位 (车头朝外停靠)。路线 (-3.9,4.5)→(-3.9,6)→(0,6)→(0,0)，最后一段倒车，终点朝北
  {
    std::vector<Pt> route{{-3.9, 4.5}, {-3.9, 6.0}, {0.0, 6.0}, {0.0, 0.0}};
    std::vector<char> rev{0, 0, 0, 1};
    RouteOptions o;
    std::vector<Corner> cs;
    auto p = densify(route, M_PI / 2, o, [](double, double, double) {return true;}, &cs, &rev);
    auto pc = splitCusps(p);
    CHECK(cs[1].kind == 1, "第一个拐点 (前进→前进) 仍走圆弧 (kind=%d)", cs[1].kind);
    CHECK(cs[2].kind == 2, "巷道口: 前进朝东 → 倒车朝北，原地转 (kind=%d)", cs[2].kind);
    CHECK(pc.size() == 2, "分成 2 段 (实际 %zu)", pc.size());
    if (pc.size() == 2) {
      CHECK(!pieceReverse(pc[0]) && pieceReverse(pc[1]), "第 1 段前进，第 2 段倒车");
      CHECK(std::fabs(wrap(pieceHeading(pc[1]) + M_PI / 2)) < 1e-6, "第 2 段行驶方向朝南");
      CHECK(std::fabs(wrap(pc[1].back().th - M_PI / 2)) < 1e-6, "倒车段车身朝北 (%.2f)", pc[1].back().th);
      CHECK(std::fabs(wrap(pc[1].front().th - M_PI / 2)) < 1e-6, "倒车段起点 (尖点) 车身朝北");
    }
    CHECK(std::fabs(wrap(p.back().th - M_PI / 2)) < 1e-6, "终点朝向已满足，不再追加对位尖点");
    // 退出死胡同: 车头朝南停在 (0,0)，倒车向北退到 (0,6)，再前进向西 —— 第一段倒车后车身仍朝南，巷道口原地转到朝西
    std::vector<Pt> out{{0, 0}, {0, 6}, {-3.9, 6}};
    std::vector<char> rev2{0, 1, 0};
    auto q = densify(out, M_PI, o, [](double, double, double) {return true;}, &cs, &rev2);
    auto pq = splitCusps(q);
    CHECK(pq.size() == 2 && pieceReverse(pq[0]) && !pieceReverse(pq[1]), "倒车退出 + 前进 (段数 %zu)", pq.size());
    CHECK(std::fabs(wrap(q.front().th + M_PI / 2)) < 1e-6, "倒车段车身朝南");
    // 只换向不转车身: 前进开进死胡同再原路倒车退出
    std::vector<Pt> io{{0, 6}, {0, 0}, {0, 6}};
    std::vector<char> rev3{0, 0, 1};
    auto w = densify(io, -M_PI / 2, o, [](double, double, double) {return true;}, &cs, &rev3);
    auto pw = splitCusps(w);
    CHECK(cs[1].kind == 3, "停车换向 (kind=%d)", cs[1].kind);
    CHECK(pw.size() == 2 && !pieceReverse(pw[0]) && pieceReverse(pw[1]), "换向处分段 (段数 %zu)", pw.size());
    CHECK(std::fabs(wrap(w.back().th + M_PI / 2)) < 1e-6, "全程车身朝南");
  }
  // ---- 多步挪车
  {
    auto wallX = [](std::vector<Pt> & v, double x, double y0, double y1) {for (double y = y0; y <= y1; y += 0.02) {v.push_back({x, y});}};
    auto wallY = [](std::vector<Pt> & v, double y, double x0, double x1) {for (double x = x0; x <= x1; x += 0.02) {v.push_back({x, y});}};
    auto show = [](const char * name, bool ok, const std::vector<MStep> & st, int n) {
        std::printf("  挪车 [%s]: %s，展开 %d 个状态:", name, ok ? "可行" : "不可行", n);
        for (const auto & s : st) {
          if (s.kind == 0) {std::printf(" 转 %+.0f°", s.val * 180 / M_PI);} else {std::printf(" %s %.2f m", s.val > 0 ? "前进" : "后退", std::fabs(s.val));}
        }
        std::printf("\n");
      };
    ManeuverOpts o;
    std::vector<MStep> st;
    int n = 0;
    // (1) 空旷: 直接转
    bool ok = planManeuver({}, body, 0.05, 0.05, 1.57, o, &st, &n);
    show("空旷转 90°", ok, st, n);
    CHECK(ok && st.size() == 1 && st[0].kind == 0, "空旷处应一步转到位");
    // (2) 车头前 5 cm 有墙 (工位车头朝里): 后退再转
    std::vector<Pt> a;
    wallX(a, 0.65, -1.5, 1.5);
    ok = planManeuver(a, body, 0.05, 0.05, M_PI, o, &st, &n);
    show("车头贴墙掉头", ok, st, n);
    CHECK(ok, "后退后应能掉头");
    CHECK(!st.empty() && st[0].kind == 1 && st[0].val < 0, "第一步应是后退");
    // (3) 口袋: 前方、左右三面是墙，后方 1 m 处才开阔 —— 旧策略 (摆头 + 后退 ≤ 0.5 m + 一次转到位) 做不到
    std::vector<Pt> b;
    wallX(b, 0.65, -0.45, 0.45);
    wallY(b, 0.45, -0.9, 0.65);
    wallY(b, -0.45, -0.9, 0.65);
    wallX(b, -2.6, -2.0, 2.0);          // 后方远处的墙
    wallY(b, 2.0, -2.6, -0.9);
    wallY(b, -2.0, -2.6, -0.9);
    wallX(b, -0.9, 0.45, 2.0);
    wallX(b, -0.9, -2.0, -0.45);
    o.max_dist = 2.0;
    ok = planManeuver(b, body, 0.05, 0.05, M_PI, o, &st, &n);
    show("口袋里掉头", ok, st, n);
    CHECK(ok, "退出口袋后应能掉头");
    // (4) 巷道宽 1.0 m，车体 1.0 × 0.7 (对角线 1.22 m): 无论怎么挪都转不过来，应明确返回不可行
    std::vector<Pt> c;
    wallY(c, 0.5, -4.0, 4.0);
    wallY(c, -0.5, -4.0, 4.0);
    ok = planManeuver(c, body, 0.02, 0.02, M_PI, o, &st, &n);
    show("窄巷道掉头", ok, {}, n);
    CHECK(!ok, "窄巷道里应判不可行");
    // (5) 斜向空间: 只能转一点、挪一点、再转 (多点掉头)。宽 1.3 m 的通道 (对角线 1.22 < 1.3 但扫掠半径 0.69×2=1.39 > 1.3)
    std::vector<Pt> d;
    wallY(d, 0.65, -4.0, 4.0);
    wallY(d, -0.65, -4.0, 4.0);
    o.max_dist = 1.0;
    ok = planManeuver(d, body, 0.02, 0.02, M_PI, o, &st, &n);
    show("1.3 m 通道多点掉头", ok, st, n);
    CHECK(ok && st.size() >= 3, "应通过转 + 挪的组合掉头 (步数 %zu)", st.size());
    // (6) 手机现场: 巷道口 (0,6)，车头朝东，要转成车头朝北后沿 x=0 向南倒车；拐点两侧各有一个障碍，原地两个方向都转不开。
    //     只要求朝向 → 后退 0.4 m 转 90°，但车不在 x=0 线上 (重新规划会开回拐点再转，无限循环)；
    //     要求落在下一段线上 → 多转一点、斜着倒到线上、再回正
    std::vector<Pt> e;
    Rect big{1.31, 0.48, 0.5, 0.5};
    auto box = [&](double cx, double cy) {
        for (double t = -0.2; t <= 0.2; t += 0.02) {
          e.push_back({cx + t, cy - 0.2}); e.push_back({cx + t, cy + 0.2}); e.push_back({cx - 0.2, cy + t}); e.push_back({cx + 0.2, cy + t});
        }
      };
    box(0.95, 1.05);            // 车体系 = 场景系平移 (0,6)，车头朝东
    box(-0.95, -1.05);
    for (double y = -2.5; y >= -6.0; y -= 0.02) {e.push_back({-1.0, y}); e.push_back({1.0, y});}   // 巷道两侧货架
    for (double x = -2.4; x <= -1.0; x += 0.02) {e.push_back({x, -2.5});}
    for (double x = 1.0; x <= 2.4; x += 0.02) {e.push_back({x, -2.5});}
    o.max_dist = 2.5;
    ok = planManeuver(e, big, 0.05, 0.05, M_PI / 2, o, &st, &n);
    show("巷道口转向 (只要朝向)", ok, st, n);
    CHECK(ok, "只要求朝向应可行");
    ManeuverGoal g;
    g.T = M_PI / 2;
    g.mode = 1;
    g.lx = 0.0; g.ly = 0.0; g.ldir = -M_PI / 2;     // 下一段: 过拐点、向南行驶 (倒车)
    g.along_max = 2.5;
    ok = planManeuver(e, big, 0.05, 0.05, g, o, &st, &n);
    show("巷道口转向 (落在下一段线上)", ok, st, n);
    CHECK(ok, "落在线上的挪车应可行");
    if (ok) {                   // 复算终点位姿
      double x = 0, y = 0, th = 0;
      for (const auto & s : st) {if (s.kind == 0) {th += s.val;} else {x += s.val * std::cos(th); y += s.val * std::sin(th);}}
      CHECK(std::fabs(x) < 0.031 && y <= 0.05 && std::fabs(wrap(th - M_PI / 2)) < 2e-3, "终点应在 x=0 线上、朝北 (%.3f, %.3f, %.1f°)", x, y, th * 180 / M_PI);
    }
    // (7) 终点对位 (位置不能变): 窄巷道工位里掉头 → 不可行
    ManeuverGoal g2;
    g2.T = M_PI;
    g2.mode = 2;
    std::vector<Pt> f;
    wallY(f, 1.0, -6.0, 6.0);
    wallY(f, -1.0, -6.0, 6.0);
    ok = planManeuver(f, big, 0.02, 0.02, g2, o, &st, &n);
    show("窄巷道工位原位掉头", ok, {}, n);
    CHECK(!ok, "2 m 巷道里 1.79 × 1.0 m 的车原位掉头应判不可行");
  }
  std::printf(fails ? "%d FAILED\n" : "all passed\n", fails);
  return fails ? 1 : 0;
}
