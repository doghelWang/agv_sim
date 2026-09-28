// ============================================================================
// AgvGoalChecker (nav2_core::GoalChecker) —— 到位由 RouteController 判定
//   controller_server 每周期把"路径终点 (按当前 map→odom 换算)"和机器人位姿交给到位判定；
//   末段 RouteController 已把终点冻结在 odom 系并按 ±final_stop_tol / ±yaw_tol 控到位，
//   这里只读它的"已到位"标志 (同一进程内共享)，避免用抖动的 map→odom 重新判定
// ============================================================================
#include <memory>
#include <string>

#include "agv_nav2_plugins/common.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace agv_nav2_plugins
{

class AgvGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/) override
  {
    auto node = parent.lock();
    xy_ = param<double>(node, name + ".xy_goal_tolerance", 0.01);
    yaw_ = param<double>(node, name + ".yaw_goal_tolerance", 0.01);
  }
  void reset() override {}
  bool isGoalReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Twist &) override
  {
    return SharedState::get().done.load();
  }
  bool getTolerances(geometry_msgs::msg::Pose & pose_tol, geometry_msgs::msg::Twist & vel_tol) override
  {
    pose_tol.position.x = pose_tol.position.y = xy_;
    pose_tol.orientation.z = yaw_;
    vel_tol.linear.x = vel_tol.linear.y = 0.01;
    vel_tol.angular.z = 0.02;
    return true;
  }

private:
  double xy_{0.01}, yaw_{0.01};
};

}  // namespace agv_nav2_plugins

PLUGINLIB_EXPORT_CLASS(agv_nav2_plugins::AgvGoalChecker, nav2_core::GoalChecker)
