// ============================================================================
// RoutePlanner (nav2_core::GlobalPlanner) —— 拓扑路线 → Nav2 可跟随的稠密路径
//
//   执行进程 (Dijkstra 拓扑规划) 把路线节点发到 /agv/route；行为树的 ComputePathToPose 调本插件:
//     · 从机器人当前位置接到路线上 (投影到最近路段，只向前不回退) —— 恢复行为后重新规划不会退回起点重走
//     · 拐点: 转角小 → 折线 (控制器切过)；可行时走过渡圆弧 (全局代价地图上车体外形沿圆弧无碰撞)，不停车；
//       圆弧放不下 / 掉头 → 拐点停车原地转向 (路径里用"尖点"表示: 同一位置两个朝向)
//     · 终点朝向与到达方向不同 → 终点尖点 (控制器到位后原地对位)
//   参数 (<name>.*): corner_radius (0 = 按车头长度自动), arc_min_turn, arc_max_turn, use_arcs, step, route_wait_s
// ============================================================================
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "agv_nav2_plugins/common.hpp"
#include "agv_nav2_plugins/geom.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace agv_nav2_plugins
{

class RoutePlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    name_ = name;
    costmap_ros_ = costmap_ros;
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    opt_.r_pref = param<double>(node, name + ".corner_radius", 0.0);
    opt_.arc_min_turn = param<double>(node, name + ".arc_min_turn", 0.35);
    opt_.arc_max_turn = param<double>(node, name + ".arc_max_turn", 2.6);
    opt_.use_arcs = param<bool>(node, name + ".use_arcs", true);
    opt_.step = param<double>(node, name + ".step", 0.05);
    wait_s_ = param<double>(node, name + ".route_wait_s", 1.5);
    const agv::Rect body = rectParam(node, name);
    if (opt_.r_pref <= 0.0) {   // 与 nav_runtime Navigator.corner_radius 一致: 车头越长半径越大
      opt_.r_pref = std::max(0.6, std::min(1.4, 0.6 * body.head + 0.3));
    }
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    sub_ = node->create_subscription<nav_msgs::msg::Path>(
      "/agv/route", qos, [this](nav_msgs::msg::Path::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mu_);
        route_ = *m;
        progress_ = 0;
      });
    events_.init(node);
  }

  void cleanup() override {sub_.reset();}
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) override
  {
    const double gx = goal.pose.position.x, gy = goal.pose.position.y, gyaw = tf2::getYaw(goal.pose.orientation);
    // 等执行进程发来与本目标一致的路线 (目标先于路线到达时最多等 route_wait_s)
    std::vector<agv::Pt> route;
    std::vector<char> rflag;      // rflag[i]: 到达 route[i] 的这一段倒车 (/agv/route 里用 position.z = 1 标记)
    const auto t_end = clock_->now() + rclcpp::Duration::from_seconds(wait_s_);
    while (true) {
      {
        std::lock_guard<std::mutex> lk(mu_);
        if (!route_.poses.empty()) {
          const auto & b = route_.poses.back().pose.position;
          if (std::hypot(b.x - gx, b.y - gy) < 0.05) {
            for (const auto & p : route_.poses) {
              route.push_back({p.pose.position.x, p.pose.position.y});
              rflag.push_back(p.pose.position.z > 0.5 ? 1 : 0);
            }
          }
        }
      }
      if (!route.empty() || clock_->now() > t_end) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const agv::Pt s{start.pose.position.x, start.pose.position.y};
    std::vector<agv::Pt> pts{s};
    std::vector<char> rev{0};
    if (route.size() >= 2) {
      // 接入: 投影到 [progress_, …] 中最近的路段 (只向前)，接到该路段终点及之后的节点
      size_t best = progress_;
      double bd = 1e18;
      for (size_t i = progress_; i + 1 < route.size(); ++i) {
        const agv::Pt a = route[i], b = route[i + 1];
        const double ex = b.x - a.x, ey = b.y - a.y, L2 = std::max(ex * ex + ey * ey, 1e-12);
        const double u = std::clamp(((s.x - a.x) * ex + (s.y - a.y) * ey) / L2, 0.0, 1.0);
        const double d = std::hypot(s.x - a.x - u * ex, s.y - a.y - u * ey);
        if (d < bd - 1e-6) {bd = d; best = i;}
      }
      progress_ = best;
      for (size_t i = best + 1; i < route.size(); ++i) {
        if (std::hypot(route[i].x - pts.back().x, route[i].y - pts.back().y) > 0.03) {
          pts.push_back(route[i]);
          rev.push_back(rflag[i]);
        }
      }
    } else {
      RCLCPP_WARN(logger_, "[%s] 没有与目标一致的拓扑路线，按直线规划", name_.c_str());
    }
    if (pts.size() == 1 || std::hypot(pts.back().x - gx, pts.back().y - gy) > 0.03) {
      if (std::hypot(gx - pts.back().x, gy - pts.back().y) > 0.03) {
        pts.push_back({gx, gy});
        rev.push_back(rev.size() > 1 ? rev.back() : 0);
      }
    }

    auto * cm = costmap_ros_->getCostmap();
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(cm);
    const auto fp = costmap_ros_->getRobotFootprint();   // 含 footprint_padding (= 车体净空 body_margin)
    auto free = [&](double x, double y, double th) {
        const double c = checker.footprintCostAtPose(x, y, th, fp);
        return c < nav2_costmap_2d::LETHAL_OBSTACLE;
      };
    std::vector<agv::Corner> corners;
    std::vector<agv::Pose2> dense;
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lk(*cm->getMutex());
      dense = agv::densify(pts, gyaw, opt_, free, &corners, &rev);
    }
    int n_arc = 0, n_rot = 0, n_sw = 0, n_rev = 0;
    for (const auto & c : corners) {n_arc += c.kind == 1; n_rot += c.kind == 2; n_sw += c.kind == 3;}
    for (size_t i = 1; i < rev.size(); ++i) {n_rev += rev[i] != 0;}
    RCLCPP_INFO(logger_, "[%s] 路线 %zu 个节点 → %zu 个路径点，圆弧过弯 %d 处，停车转向 %d 处，倒车 %d 段 (停车换向 %d 处)",
      name_.c_str(), pts.size(), dense.size(), n_arc, n_rot, n_rev, n_sw);

    nav_msgs::msg::Path path;
    path.header.frame_id = goal.header.frame_id.empty() ? "map" : goal.header.frame_id;
    path.header.stamp = clock_->now();
    for (const auto & p : dense) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = p.rev ? 1.0 : 0.0;      // 倒车段标记 (RouteController 据此分段、倒车跟线)
      tf2::Quaternion q;
      q.setRPY(0, 0, p.th);
      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();
      ps.pose.orientation.w = q.w();
      path.poses.push_back(ps);
    }
    return path;
  }

private:
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("agv_route_planner")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
  std::mutex mu_;
  nav_msgs::msg::Path route_;
  size_t progress_{0};
  agv::RouteOptions opt_;
  double wait_s_{1.5};
  EventPub events_;
};

}  // namespace agv_nav2_plugins

PLUGINLIB_EXPORT_CLASS(agv_nav2_plugins::RoutePlanner, nav2_core::GlobalPlanner)
