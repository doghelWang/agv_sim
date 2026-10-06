// ============================================================================
// AdjustPose (Nav2 behavior_server 恢复行为，动作名 adjust_pose，复用 nav2_msgs/action/BackUp 接口)
//
//   行为树里用 <BackUp server_name="adjust_pose" backup_dist="最大后退距离" backup_speed="后退速度"/> 调用
//   · 有新的原地转向受阻请求 (/agv/turn_request，RouteController 发布: 位置 + 目标朝向，odom 系):
//       多步挪车 (像汽车掉头/揉库: 转一点 → 前后挪一点 → 再转)。在 (位置, 朝向) 上搜索由"原地转 ±10°、前进/后退 0.1 m"
//       组成的最短动作序列 (geom.hpp planManeuver)，每一步都用实测激光点对车体外扩 (rotate_margin + plan_extra_margin)
//       做扫掠检查 (规划多留余量，执行时按 rotate_margin 持续检查，激光噪声/挪车误差不至于让执行途中判为受阻)；
//       范围 = max(backup_dist, maneuver_radius)。目标不只是朝向: 拐点处要求转完**落在下一段路线上** (请求的 position.z = 1/2)，
//       终点对位要求位置不变 (3) —— 只转到朝向的话车不在线上，重新规划会先开回拐点、在同一个转不开的地方再转一次，每次重试都失败。
//       完成后返回成功，行为树重新规划 (RoutePlanner 从当前位置接回路线)。
//       搜不到 = 这块空间里车体确实转不过来 (如巷道宽度小于车体对角线) → 失败，由执行进程换路线 (倒车离开)。
//       (以前只试"摆头一次 + 后退 ≤ backup_dist + 一次转到位"，后退距离内转不开就每次重试都失败)
//   · 没有转向请求 (一般的线路跟随中断，如前方障碍/定位抖动): 后方扫掠区无障碍时后退 min(0.2, backup_dist)
//   执行中持续检查扫掠区，出现新障碍立即停车并返回失败 (行为树按重试次数继续或放弃)
// ============================================================================
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "agv_nav2_plugins/common.hpp"
#include "agv_nav2_plugins/geom.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace agv_nav2_plugins
{

using BackUpAction = nav2_msgs::action::BackUp;
using nav2_behaviors::Status;
using agv::wrap;

class AdjustPose : public nav2_behaviors::TimedBehavior<BackUpAction>
{
  struct Step { int kind; double target; };   // 0 原地转 target (rad，带符号，相对本步起点) 1 平移距离 (负 = 后退) 2 舵轮就位

public:
  void onConfigure() override
  {
    auto node = node_.lock();
    const std::string p = behavior_name_ + ".";
    body_ = rectParam(node, behavior_name_);
    margin_ = param<double>(node, p + "rotate_margin", 0.02);
    back_margin_ = param<double>(node, p + "backup_margin", 0.03);
    plan_extra_ = param<double>(node, p + "plan_extra_margin", 0.03);
    rot_w_ = param<double>(node, p + "rotate_max_w", 0.5);
    rot_acc_ = param<double>(node, p + "rotate_accel", 0.5);
    settle_s_ = param<double>(node, p + "steer_settle_s", 0.0);
    req_max_age_ = param<double>(node, p + "request_max_age_s", 60.0);
    radius_ = param<double>(node, p + "maneuver_radius", 1.2);
    scan_.init(node, tf_, robot_base_frame_);
    events_.init(node);
    sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/agv/turn_request", rclcpp::QoS(1).transient_local().reliable(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mu_);
        req_ = *m;
        have_req_ = true;
      });
  }

  Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override
  {
    max_back_ = std::clamp(std::fabs(static_cast<double>(command->target.x)), 0.05, 1.0);
    speed_ = std::clamp(std::fabs(static_cast<double>(command->speed)), 0.03, 0.3);
    t_end_ = clock_->now().seconds() + std::max(5.0, rclcpp::Duration(command->time_allowance).seconds());
    geometry_msgs::msg::PoseStamped pose;
    if (!nav2_util::getCurrentPose(pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
      RCLCPP_ERROR(logger_, "adjust_pose: 取不到当前位姿");
      return Status::FAILED;
    }
    const double x = pose.pose.position.x, y = pose.pose.position.y, yaw = tf2::getYaw(pose.pose.orientation);
    double age = 0;
    const auto pts = scan_.points(&age);
    steps_.clear();
    last_rot_ = -1;
    k_ = 0;
    step_t_ = -1;

    geometry_msgs::msg::PoseStamped req;
    bool fresh = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (have_req_ && req_.header.frame_id == global_frame_) {
        const double a = (clock_->now() - rclcpp::Time(req_.header.stamp)).seconds();
        fresh = a < req_max_age_ && std::hypot(req_.pose.position.x - x, req_.pose.position.y - y) < 1.5;
        req = req_;
      }
      have_req_ = false;                      // 每个请求只处理一次
    }
    if (fresh && age < 1.0) {
      const double T = tf2::getYaw(req.pose.orientation);
      // 要求 (RouteController 填在 position.z): 1/2 转完落在下一段的线上 (前进/倒车)，3 位置不变 (终点对位)，0 只要朝向
      agv::ManeuverGoal G;
      G.T = wrap(T - yaw);
      const double zc = req.pose.position.z;
      const int kind = static_cast<int>(std::floor(zc + 1e-6));
      const double c = std::cos(yaw), sn = std::sin(yaw), dx = req.pose.position.x - x, dy = req.pose.position.y - y;
      G.lx = c * dx + sn * dy;                  // 请求点在当前车体系下
      G.ly = -sn * dx + c * dy;
      if (kind == 1 || kind == 2) {
        G.mode = 1;
        G.ldir = wrap(G.T + (kind == 2 ? M_PI : 0.0));       // 行驶方向: 前进 = 车身朝向，倒车 = 反向
        G.along_max = std::max(0.3, (zc - kind) * 10.0);
      } else if (kind == 3) {
        G.mode = 2;
      }
      std::string why;
      if (!plan(pts, yaw, T, G, &why)) {
        events_.emit("ADJUST_FAIL", "danger", "位姿调整: 空间不足，挪车也转不过去", why);
        return Status::FAILED;
      }
      return Status::SUCCEEDED;
    }
    // 没有转向请求: 普通中断 → 后方无障碍时后退一小段
    const double d = std::min(0.2, max_back_);
    if (age < 1.0 && agv::translationBlocked(pts, body_, back_margin_, 0.0, -d)) {
      events_.emit("ADJUST_FAIL", "warning", "位姿调整: 后方有障碍，不后退", "后退 " + fmt(d) + " m 的扫掠区有激光点");
      return Status::FAILED;
    }
    if (settle_s_ > 0) {steps_.push_back({2, -1.0});}
    steps_.push_back({1, -d});
    events_.emit("NAV2_RETRY", "warning", "线路跟随中断: 后退 " + fmt(d) + " m 后重新规划", "");
    return Status::SUCCEEDED;
  }

  Status onCycleUpdate() override
  {
    const double now = clock_->now().seconds();
    if (now > t_end_) {
      stopRobot();
      return Status::FAILED;
    }
    if (k_ >= steps_.size()) {
      stopRobot();
      return Status::SUCCEEDED;
    }
    geometry_msgs::msg::PoseStamped pose;
    if (!nav2_util::getCurrentPose(pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
      stopRobot();
      return Status::FAILED;
    }
    const double x = pose.pose.position.x, y = pose.pose.position.y, yaw = tf2::getYaw(pose.pose.orientation);
    if (step_t_ < 0) {step_t_ = now; sx_ = x; sy_ = y; syaw_ = yaw; turned_ = 0.0; prev_yaw_ = yaw;}
    if (step_t_ == now && steps_[k_].kind == 0 && static_cast<int>(k_) == last_rot_) {
      // 最后一次转向按绝对目标朝向收尾 (前面各步的到位误差不累积)
      steps_[k_].target += wrap(final_yaw_ - yaw - steps_[k_].target);
    }
    turned_ += wrap(yaw - prev_yaw_);          // 本步累计转角 (可超过 ±π: 规划可能要求绕远的那一边)
    prev_yaw_ = yaw;
    auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
    const Step & s = steps_[k_];
    double age = 0;
    const auto pts = scan_.points(&age);
    if (s.kind == 2) {                        // 舵轮就位 (微小指令，下一步是转向则给角速度，否则给后退)
      const bool next_rot = k_ + 1 < steps_.size() && steps_[k_ + 1].kind == 0;
      if (next_rot) {
        cmd->angular.z = 0.004 * (steps_[k_ + 1].target > 0 ? 1 : -1);
      } else {
        cmd->linear.x = (k_ + 1 < steps_.size() && steps_[k_ + 1].target > 0) ? 0.002 : -0.002;
      }
      if (now - step_t_ > settle_s_) {next();}
    } else if (s.kind == 0) {
      const double e = s.target - turned_;
      if (std::fabs(e) < 0.01) {
        next();
      } else if (age < 1.0 && agv::rotationBlocked(pts, body_, margin_, 0, 0, 0, (e > 0 ? 1 : -1) * std::min(std::fabs(e), 0.5))) {
        stopRobot();
        events_.emit("ADJUST_FAIL", "warning", "位姿调整: 转向途中出现障碍，停止",
          "距目标朝向还差 " + fmt(std::fabs(e) * 180.0 / M_PI) + "°，扫掠区出现新的激光点");
        return Status::FAILED;
      } else {
        double w = std::min({rot_w_, std::sqrt(2.0 * rot_acc_ * std::fabs(e)), 1.5 * std::fabs(e)});
        cmd->angular.z = std::copysign(std::max(w, 0.02), e);
      }
    } else {
      const double sg = s.target >= 0 ? 1.0 : -1.0;
      const double done = sg * ((x - sx_) * std::cos(syaw_) + (y - sy_) * std::sin(syaw_));
      const double left = std::fabs(s.target) - done;
      if (left <= 0.005) {
        next();
      } else if (age < 1.0 && agv::translationBlocked(pts, body_, back_margin_, 0.0, sg * std::min(left, 0.15))) {
        stopRobot();
        events_.emit("ADJUST_FAIL", "warning", std::string("位姿调整: ") + (sg > 0 ? "前进" : "后退") + "途中出现障碍，停止",
          "还差 " + fmt(left) + " m");
        return Status::FAILED;
      } else {
        cmd->linear.x = sg * std::max(0.02, std::min(speed_, std::sqrt(2.0 * 0.2 * left)));
        cmd->angular.z = 2.0 * wrap(syaw_ - yaw);   // 保持行驶方向
      }
    }
    vel_pub_->publish(std::move(cmd));
    return Status::RUNNING;
  }

private:
  void next() {++k_; step_t_ = -1;}
  static std::string fmt(double v)
  {
    char b[32];
    snprintf(b, sizeof(b), "%.2f", v);
    return b;
  }

  // 在当前位姿 (yaw) 下规划到目标朝向 T 的多步挪车 (geom.hpp planManeuver): 转一点 → 前后挪一点 → 再转 …
  // 规划时扫掠检查多留 plan_extra_margin；why: 失败原因 (给事件用)
  bool plan(const std::vector<agv::Pt> & pts, double yaw, double T, const agv::ManeuverGoal & G, std::string * why)
  {
    (void)yaw;
    agv::ManeuverOpts o;
    o.max_dist = std::max(max_back_, radius_) + (G.mode == 1 ? 0.8 : 0.0);
    std::vector<agv::MStep> st;
    int n = 0;
    if (!agv::planManeuver(pts, body_, margin_ + plan_extra_, back_margin_ + plan_extra_, G, o, &st, &n)) {
      *why = "在 " + fmt(o.max_dist) + " m 范围内前后挪动、分步转向的各种组合 (搜索了 " + std::to_string(n) + " 个位姿) 都做不到「" +
        (G.mode == 1 ? "转到目标朝向并落在下一段路线上" : (G.mode == 2 ? "原位转到目标朝向" : "转到目标朝向")) +
        "」: 这里的空间不够，需要换一条路线 (如倒车离开)";
      return false;
    }
    std::string desc;
    for (const auto & s : st) {
      if (settle_s_ > 0) {steps_.push_back({2, -1.0});}
      if (s.kind == 0) {
        steps_.push_back({0, s.val});
        desc += (desc.empty() ? "" : " → ") + std::string("转 ") + fmt(s.val * 180.0 / M_PI) + "°";
      } else {
        steps_.push_back({1, s.val});
        desc += (desc.empty() ? "" : " → ") + std::string(s.val > 0 ? "前进 " : "后退 ") + fmt(std::fabs(s.val)) + " m";
      }
    }
    final_yaw_ = T;
    last_rot_ = -1;
    for (size_t i = 0; i < steps_.size(); ++i) {
      if (steps_[i].kind == 0) {last_rot_ = static_cast<int>(i);}
    }
    if (G.mode == 1) {desc += "，落在下一段路线上";}
    events_.emit("NAV2_ALIGN", "info", "空间不足，分步挪车后转向", desc);
    RCLCPP_INFO(logger_, "adjust_pose: %s (搜索 %d 个位姿)", desc.c_str(), n);
    return true;
  }

  ScanPoints scan_;
  EventPub events_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  std::mutex mu_;
  geometry_msgs::msg::PoseStamped req_;
  bool have_req_{false};
  agv::Rect body_;
  double margin_{0.02}, back_margin_{0.03}, plan_extra_{0.03}, rot_w_{0.5}, rot_acc_{0.5}, settle_s_{0.0}, req_max_age_{60.0};
  double radius_{1.2}, turned_{0}, prev_yaw_{0}, final_yaw_{0};
  int last_rot_{-1};
  double max_back_{0.3}, speed_{0.1}, t_end_{0};
  std::vector<Step> steps_;
  size_t k_{0};
  double step_t_{-1}, sx_{0}, sy_{0}, syaw_{0};
};

}  // namespace agv_nav2_plugins

PLUGINLIB_EXPORT_CLASS(agv_nav2_plugins::AdjustPose, nav2_core::Behavior)
