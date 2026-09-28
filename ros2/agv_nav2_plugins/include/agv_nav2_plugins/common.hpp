// ============================================================================
// agv_nav2_plugins 公共部分 (ROS)
//   ScanPoints   订阅融合激光 /scan，按 TF 换算到机体系 (base_footprint) 的点集，供扫掠检查
//   SharedState  同一进程 (controller_server) 内控制器 → 到位判定插件的"已到位"标志
//   EventPub     /agv/nav_event (std_msgs/String, JSON) → 执行进程转成导航事件 (工作台事件栏)
//   话题约定:
//     /agv/route          nav_msgs/Path (transient local)  执行进程下发的拓扑路线 (稀疏节点，含起点与终点)
//     /agv/stop_distance  std_msgs/Float32                  到下一个停车点 (尖点/终点) 的剩余行程，执行进程据此缩短防护区
//     /agv/turn_request   geometry_msgs/PoseStamped (transient local)  原地转向受阻: 位置 + 目标朝向 (odom 系)，
//                         恢复行为 adjust_pose 据此规划"摆头 + 后退 + 转向"
// ============================================================================
#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "agv_nav2_plugins/geom.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

namespace agv_nav2_plugins
{

class ScanPoints
{
public:
  void init(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string & base_frame, const std::string & topic = "/scan")
  {
    tf_ = tf;
    base_ = base_frame;
    sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {onScan(*m);});
  }

  // 机体系点集；age: 距最近一帧的秒数 (没有数据时 1e9)
  std::vector<agv::Pt> points(double * age = nullptr) const
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (age) {
      *age = have_ ? std::chrono::duration<double>(std::chrono::steady_clock::now() - t_).count() : 1e9;
    }
    return pts_;
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan & m)
  {
    tf2::Transform T;
    T.setIdentity();
    if (m.header.frame_id != base_) {
      try {   // 激光安装位姿是静态的，取最新变换即可 (完整旋转: 安全激光倒装 roll = π 时扫描是镜像的)
        auto t = tf_->lookupTransform(base_, m.header.frame_id, tf2::TimePointZero);
        T.setOrigin(tf2::Vector3(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
        T.setRotation(tf2::Quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
          t.transform.rotation.w));
      } catch (const std::exception &) {
        return;
      }
    }
    std::vector<agv::Pt> v;
    v.reserve(m.ranges.size());
    for (size_t i = 0; i < m.ranges.size(); ++i) {
      const float r = m.ranges[i];
      if (!std::isfinite(r) || r < m.range_min || r >= m.range_max - 1e-3) {continue;}
      const double a = m.angle_min + m.angle_increment * i;
      const tf2::Vector3 q = T * tf2::Vector3(r * std::cos(a), r * std::sin(a), 0.0);
      v.push_back({q.x(), q.y()});
    }
    std::lock_guard<std::mutex> lk(mu_);
    pts_.swap(v);
    t_ = std::chrono::steady_clock::now();
    have_ = true;
  }

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string base_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  mutable std::mutex mu_;
  std::vector<agv::Pt> pts_;
  std::chrono::steady_clock::time_point t_;
  bool have_{false};
};

struct SharedState
{
  std::atomic<bool> done{false};
  static SharedState & get()
  {
    static SharedState s;
    return s;
  }
};

class EventPub
{
public:
  void init(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
  {
    // 普通 (非生命周期) 发布器: 插件自己的诊断话题不随节点激活状态开关
    pub_ = rclcpp::create_publisher<std_msgs::msg::String>(*node, "/agv/nav_event", rclcpp::QoS(20).reliable());
  }
  // type: 事件类型 (如 NAV2_ALIGN)；level: info/warning/danger
  void emit(const std::string & type, const std::string & level, const std::string & title, const std::string & msg)
  {
    if (!pub_) {return;}
    std_msgs::msg::String s;
    s.data = "{\"type\":\"" + esc(type) + "\",\"level\":\"" + esc(level) + "\",\"title\":\"" + esc(title) +
      "\",\"message\":\"" + esc(msg) + "\"}";
    pub_->publish(s);
  }

private:
  static std::string esc(const std::string & in)
  {
    std::string o;
    for (char c : in) {
      if (c == '"' || c == '\\') {o += '\\';}
      o += c;
    }
    return o;
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

inline agv::Rect rectParam(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & ns)
{
  auto d = [&](const std::string & k, double v) {
      if (!node->has_parameter(ns + "." + k)) {node->declare_parameter(ns + "." + k, v);}
      return node->get_parameter(ns + "." + k).as_double();
    };
  agv::Rect r;
  r.head = d("body_head", 0.5);
  r.tail = d("body_tail", 0.5);
  r.left = d("body_left", 0.4);
  r.right = d("body_right", 0.4);
  return r;
}

template<typename T>
T param(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name, T def)
{
  if (!node->has_parameter(name)) {node->declare_parameter(name, def);}
  T v = def;
  node->get_parameter(name, v);
  return v;
}

}  // namespace agv_nav2_plugins
