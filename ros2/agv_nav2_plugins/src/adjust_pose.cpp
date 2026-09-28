// ============================================================================
// AdjustPose (Nav2 behavior_server 恢复行为，动作名 adjust_pose，复用 nav2_msgs/action/BackUp 接口)
//
//   行为树里用 <BackUp server_name="adjust_pose" backup_dist="最大后退距离" backup_speed="后退速度"/> 调用
//   · 有新的原地转向受阻请求 (/agv/turn_request，RouteController 发布: 位置 + 目标朝向，odom 系):
//       在"摆头 Δ (±10/20/30°) → 后退 d (0.05 m 步长，≤ backup_dist) → 原地转到目标朝向"的组合里，
//       用实测激光点对车体外扩 (rotate_margin + plan_extra_margin) 做扫掠检查 (规划多留余量，执行时按 rotate_margin
//       持续检查，激光噪声/后退误差不至于让执行途中判为受阻)，取代价 (d + 0.3·|Δ|) 最小的可行组合执行；
//       转到目标朝向后返回成功，行为树重新规划 (RoutePlanner 从当前位置接回路线)
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
  struct Step { int kind; double target; };   // 0 转向到绝对朝向 (odom) 1 后退距离 2 舵轮就位

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
    k_ = 0;
    step_t_ = -1;

    geometry_msgs::msg::PoseStamped req;
    bool fresh = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (have_req_ && req_.header.frame_id == global_frame_) {
        const double a = (clock_->now() - rclcpp::Time(req_.header.stamp)).seconds();
        fresh = a < req_max_age_ && std::hypot(req_.pose.position.x - x, req_.pose.position.y - y) < 1.0;
        req = req_;
      }
      have_req_ = false;                      // 每个请求只处理一次
    }
    if (fresh && age < 1.0) {
      const double T = tf2::getYaw(req.pose.orientation);
      if (!plan(pts, yaw, T)) {
        events_.emit("ADJUST_FAIL", "danger", "位姿调整: 找不到可行的摆头/后退组合",
          "摆头 ±30°、后退 ≤ " + fmt(max_back_) + " m 内都无法原地转到目标朝向");
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
    steps_.push_back({1, d});
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
    if (step_t_ < 0) {step_t_ = now; sx_ = x; sy_ = y; syaw_ = yaw;}
    auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
    const Step & s = steps_[k_];
    double age = 0;
    const auto pts = scan_.points(&age);
    if (s.kind == 2) {                        // 舵轮就位 (微小指令，下一步是转向则给角速度，否则给后退)
      const bool next_rot = k_ + 1 < steps_.size() && steps_[k_ + 1].kind == 0;
      if (next_rot) {cmd->angular.z = 0.004 * (wrap(steps_[k_ + 1].target - yaw) > 0 ? 1 : -1);} else {cmd->linear.x = -0.002;}
      if (now - step_t_ > settle_s_) {next();}
    } else if (s.kind == 0) {
      const double e = wrap(s.target - yaw);
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
      const double done = -((x - sx_) * std::cos(syaw_) + (y - sy_) * std::sin(syaw_));
      const double left = s.target - done;
      if (left <= 0.005) {
        next();
      } else if (age < 1.0 && agv::translationBlocked(pts, body_, back_margin_, 0.0, -std::min(left, 0.15))) {
        stopRobot();
        events_.emit("ADJUST_FAIL", "warning", "位姿调整: 后退途中出现障碍，停止", "还差 " + fmt(left) + " m");
        return Status::FAILED;
      } else {
        cmd->linear.x = -std::max(0.02, std::min(speed_, std::sqrt(2.0 * 0.2 * left)));
        cmd->angular.z = 2.0 * wrap(syaw_ - yaw);   // 保持后退方向
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

  // 在当前位姿 (yaw) 下规划到目标朝向 T 的"摆头 + 后退 + 转向"
  bool plan(const std::vector<agv::Pt> & pts, double yaw, double T)
  {
    struct Cand { double cost, swing, back, turn; };
    std::vector<Cand> ok;
    const double swings[] = {0.0, 0.175, -0.175, 0.35, -0.35, 0.52, -0.52};
    const double pm = margin_ + plan_extra_;
    for (double sw : swings) {
      if (sw != 0.0 && agv::rotationBlocked(pts, body_, pm, 0, 0, 0, sw)) {continue;}
      for (double d = 0.0; d <= max_back_ + 1e-6; d += 0.05) {
        if (d > 0 && agv::translationBlocked(pts, body_, back_margin_ + plan_extra_, sw, -d)) {break;}
        const double bx = -d * std::cos(sw), by = -d * std::sin(sw);   // 后退后的位置 (当前机体系)
        const double need = wrap(T - (yaw + sw));
        for (double turn : {need, need - (need > 0 ? 1.0 : -1.0) * 2.0 * M_PI}) {
          if (!agv::rotationBlocked(pts, body_, pm, bx, by, sw, turn)) {
            ok.push_back({d + 0.3 * std::fabs(sw) + 0.02 * std::fabs(turn), sw, d, turn});
            break;
          }
        }
        if (!ok.empty() && ok.back().swing == sw && ok.back().back == d) {break;}   // 同一摆头取最短后退
      }
    }
    if (ok.empty()) {return false;}
    const auto best = *std::min_element(ok.begin(), ok.end(), [](const Cand & a, const Cand & b) {return a.cost < b.cost;});
    const double yaw_sw = wrap(yaw + best.swing);
    if (best.swing != 0.0) {
      if (settle_s_ > 0) {steps_.push_back({2, -1.0});}
      steps_.push_back({0, yaw_sw});
    }
    if (best.back > 0.0) {
      if (settle_s_ > 0) {steps_.push_back({2, -1.0});}
      steps_.push_back({1, best.back});
    }
    if (settle_s_ > 0) {steps_.push_back({2, -1.0});}
    steps_.push_back({0, wrap(yaw_sw + best.turn)});
    events_.emit("NAV2_ALIGN", "info", "位姿调整后转向",
      "摆头 " + fmt(best.swing * 180.0 / M_PI) + "°，后退 " + fmt(best.back) + " m，再原地转 " +
      fmt(best.turn * 180.0 / M_PI) + "° 到目标朝向");
    RCLCPP_INFO(logger_, "adjust_pose: 摆头 %.1f°，后退 %.2f m，转 %.1f°", best.swing * 180 / M_PI, best.back,
      best.turn * 180 / M_PI);
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
  double max_back_{0.3}, speed_{0.1}, t_end_{0};
  std::vector<Step> steps_;
  size_t k_{0};
  double step_t_{-1}, sx_{0}, sy_{0}, syaw_{0};
};

}  // namespace agv_nav2_plugins

PLUGINLIB_EXPORT_CLASS(agv_nav2_plugins::AdjustPose, nav2_core::Behavior)
