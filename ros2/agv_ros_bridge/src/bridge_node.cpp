// ============================================================================
// agv_ros_bridge —— 执行进程 (nav_runtime) 的 ROS 2 发布端，C++ 实现 (NAV_ROS_BRIDGE=cpp)
//
//   Python (nav_runtime/cpp_bridge.py)  ──Unix 数据报──▶  本节点
//     STATE  真值/里程计/IMU/关节/map→odom      →  /odom  /ground_truth/odom  /imu  /joint_states
//                                                   TF odom→base_footprint (own_odom)、map→odom (内置定位)
//     SCAN   各 2D 激光 / 融合扫描               →  /scan/<name> (代价地图时间戳钳位)  /scan
//     ROUTE  拓扑路线导航 (JSON: 任务号/路线点/终点/行为树/场景线段) → /agv/route、/agv/world_segments 并
//            下发 NavigateToPose (agv_nav2_plugins 行为树)；CANCEL 取消
//   本节点  ──Unix 数据报──▶  Python
//     TF     50 Hz 查询 map→base_footprint、odom→base_footprint (slam_toolbox / EKF 输出) + 墙钟-仿真时间偏移
//     NAV    路线导航回馈 (JSON): 结果、反馈 (限 5 Hz)、/agv/stop_distance (限 20 Hz)、/agv/nav_event、/plan 曲线
//            —— bt_navigator 每个行为树周期 (10 ms) 发一次反馈，由 Python rclpy 接收时执行进程 CPU 翻倍
//
// 与 nav_runtime/ros_bridge.py 的 Python 发布逻辑逐项一致: 时间戳 = 仿真时间 + 最小延迟偏移、协方差、
// 关节每 2 帧发布一次、AGV_COSTMAP_SCAN_CLAMP / AGV_COSTMAP_SCAN_MARGIN 钳位规则。
//
// 核心模式 (--core，NAV_CPP_CORE=1 默认)：本节点直接连仿真推送流 (SIM_API /api/v1/stream)，不再经 Python 转发 ——
//   状态/激光直接发布 ROS；状态帧/元信息/IO/融合扫描原样转给 Python (定位融合、任务、界面)；
//   融合扫描 → 各档防护区走廊最近障碍；Nav2 /cmd_vel → 安全层 (safety.hpp) → UDP 指令 (无 UDP 时 REST)；
//   防护区状态变化/定位停更 → 事件。Python 下发 CONFIG (保护空间/外形/光电) 与 MODE (是否转发 Nav2 指令、TF 发布标志等)
// 用法: agv_ros_bridge --in <本节点接收的 socket 路径> --out <Python 接收的 socket 路径> [--core] [--ros-args ...]
// ============================================================================
#include <netdb.h>
#ifdef __linux__
#include <sys/prctl.h>
#endif
#include <csignal>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <deque>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <set>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <slam_toolbox/srv/serialize_pose_graph.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <fcntl.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "guidance.hpp"
#include "json_lite.hpp"
#include "safety.hpp"
#include "sim_stream.hpp"

namespace {

constexpr char MAGIC[4] = {'A', 'G', 'V', '1'};
enum : uint8_t { T_STATE = 1, T_SCAN = 2, T_ROUTE = 3, T_CANCEL = 4, T_CONFIG = 5, T_MODE = 6, T_GUIDE = 7, T_GUIDE_CANCEL = 8,
                 T_ROSCALL = 9,   // 核心模式: 执行进程的 ROS 操作请求 (JSON {"op": load_map|set_pose|save_map, ...})
                 T_TF = 10, T_STATS = 11, T_NAV = 12,
                 // 核心模式 C++ → Python: 仿真推送流帧原样转发 (类型号 = 20 + 推送流帧类型) 与安全层快照
                 T_RELAY = 20, T_SAFETY = 30 };
enum : uint8_t { F_OWN_ODOM = 1, F_MAP_ODOM = 2, F_IMU = 4, F_HAS_T = 8 };

struct Reader {
  const uint8_t *p, *end;
  bool ok = true;
  template <typename T>
  T get() {
    T v{};
    if (p + sizeof(T) > end) { ok = false; return v; }
    std::memcpy(&v, p, sizeof(T));
    p += sizeof(T);
    return v;
  }
  std::string str() {
    uint8_t n = get<uint8_t>();
    if (!ok || p + n > end) { ok = false; return {}; }
    std::string s(reinterpret_cast<const char *>(p), n);
    p += n;
    return s;
  }
};

template <typename T>
void put(std::vector<uint8_t> &b, T v) {
  const uint8_t *q = reinterpret_cast<const uint8_t *>(&v);
  b.insert(b.end(), q, q + sizeof(T));
}

double yaw_of(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void set_quat(geometry_msgs::msg::Quaternion &q, double yaw) {
  q.x = 0.0; q.y = 0.0; q.z = std::sin(yaw / 2.0); q.w = std::cos(yaw / 2.0);
}

builtin_interfaces::msg::Time to_msg(double ts) {
  builtin_interfaces::msg::Time m;
  m.sec = static_cast<int32_t>(std::floor(ts));
  m.nanosec = static_cast<uint32_t>((ts - std::floor(ts)) * 1e9);
  return m;
}

double to_sec(const builtin_interfaces::msg::Time &t) { return t.sec + t.nanosec * 1e-9; }

// "@name" = Linux 抽象命名空间套接字 (无文件路径: proot 不做路径翻译，也不受 108 字节路径上限限制)
socklen_t make_addr(const std::string &path, sockaddr_un &a) {
  std::memset(&a, 0, sizeof(a));
  a.sun_family = AF_UNIX;
  if (!path.empty() && path[0] == '@') {
    size_t n = std::min(path.size() - 1, sizeof(a.sun_path) - 1);
    std::memcpy(a.sun_path + 1, path.data() + 1, n);
    return static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + n);
  }
  std::strncpy(a.sun_path, path.c_str(), sizeof(a.sun_path) - 1);
  return static_cast<socklen_t>(sizeof(a));
}

// 路线消息的极简 JSON 读取 (格式由 nav_runtime/cpp_bridge.py 生成: 只有数字、数字数组、字符串)
std::string json_str(const std::string &j, const std::string &key) {
  auto k = j.find("\"" + key + "\"");
  if (k == std::string::npos) return {};
  auto a = j.find('"', j.find(':', k) + 1);
  std::string out;
  for (size_t i = a + 1; i < j.size() && j[i] != '"'; ++i) {
    if (j[i] == '\\' && i + 1 < j.size()) ++i;
    out += j[i];
  }
  return out;
}
std::vector<double> json_nums(const std::string &j, const std::string &key) {
  std::vector<double> v;
  auto k = j.find("\"" + key + "\"");
  if (k == std::string::npos) return v;
  size_t i = j.find(':', k) + 1;
  int depth = 0;
  for (; i < j.size(); ++i) {
    char c = j[i];
    if (c == '[') { ++depth; continue; }
    if (c == ']') { if (--depth <= 0) break; continue; }
    if (depth == 0 && (c == ',' || c == '}')) break;
    if (c == '-' || c == '.' || (c >= '0' && c <= '9')) {
      char *e = nullptr;
      v.push_back(std::strtod(j.c_str() + i, &e));
      i = static_cast<size_t>(e - j.c_str()) - 1;
    }
  }
  return v;
}
std::string jesc(const std::string &in) {
  std::string o;
  for (char c : in) { if (c == '"' || c == '\\') o += '\\'; o += c; }
  return o;
}

}  // namespace

class AgvRosBridge : public rclcpp::Node {
 public:
  AgvRosBridge(const std::string &in_path, const std::string &out_path, bool core)
      : Node("nav_runtime_bridge_cpp"), in_path_(in_path), out_path_(out_path), core_(core) {
    const char *cl = std::getenv("AGV_COSTMAP_SCAN_CLAMP");
    clamp_ = !(cl && std::string(cl) != "1");
    const char *mg = std::getenv("AGV_COSTMAP_SCAN_MARGIN");
    margin_env_ = (mg && *mg) ? std::atof(mg) : NAN;
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    sensor_qos_ = sensor_qos;
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    gt_pub_ = create_publisher<nav_msgs::msg::Odometry>("/ground_truth/odom", 10);
    js_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 20);
    tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    buf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tfl_ = std::make_unique<tf2_ros::TransformListener>(*buf_);   // 独立线程接收 /tf，不占主执行器

    in_fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    out_fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (in_path_[0] != '@') unlink(in_path_.c_str());
    sockaddr_un a{};
    socklen_t alen = make_addr(in_path_, a);
    if (bind(in_fd_, reinterpret_cast<sockaddr *>(&a), alen) != 0) {
      RCLCPP_FATAL(get_logger(), "bind %s 失败: %s", in_path_.c_str(), std::strerror(errno));
      throw std::runtime_error("bind");
    }
    int rcv = 1 << 20;
    setsockopt(in_fd_, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof(rcv));
    timeval tv{0, 200000};
    setsockopt(in_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    out_len_ = make_addr(out_path_, out_addr_);

    rx_ = std::thread([this] { rx_loop(); });
    tf_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { poll_tf(); });
    // 路线导航 (agv_nav2_plugins)
    auto latched = rclcpp::QoS(1).transient_local().reliable();
    route_pub_ = create_publisher<nav_msgs::msg::Path>("/agv/route", latched);
    segs_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/agv/world_segments", latched);
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    stop_sub_ = create_subscription<std_msgs::msg::Float32>("/agv/stop_distance", 10, [this](std_msgs::msg::Float32::ConstSharedPtr m) {
      {
        std::lock_guard<std::mutex> lk(smu_);
        stop_dist_ = m->data;
        stop_wall_ = wall_now();
      }
      double t = now_s();
      if (t - last_stop_tx_ < 0.05) return;                   // 限 20 Hz
      last_stop_tx_ = t;
      send_nav("{\"k\":\"stop\",\"d\":" + std::to_string(m->data) + "}");
    });
    event_sub_ = create_subscription<std_msgs::msg::String>("/agv/nav_event", 20, [this](std_msgs::msg::String::ConstSharedPtr m) {
      send_nav("{\"k\":\"event\",\"e\":" + m->data + "}");
    });
    plan_sub_ = create_subscription<nav_msgs::msg::Path>("/plan", 2, [this](nav_msgs::msg::Path::ConstSharedPtr m) {
      if (m->poses.empty()) return;
      std::string c = "{\"k\":\"plan\",\"curve\":[";
      double lx = 1e9, ly = 1e9;
      char b[64];
      for (size_t i = 0; i < m->poses.size(); ++i) {
        const auto &p = m->poses[i].pose.position;
        if (i + 1 < m->poses.size() && std::hypot(p.x - lx, p.y - ly) < 0.2) continue;   // 约 0.2 m 一个点
        std::snprintf(b, sizeof(b), "%s[%.3f,%.3f]", (lx < 1e8 ? "," : ""), p.x, p.y);
        c += b;
        lx = p.x; ly = p.y;
      }
      send_nav(c + "]}");
    });
    stats_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { send_stats(); });
    if (core_) start_core();
    RCLCPP_INFO(get_logger(), "agv_ros_bridge (C++) 就绪: in=%s out=%s clamp=%d core=%d", in_path_.c_str(), out_path_.c_str(), clamp_, core_);
  }

  ~AgvRosBridge() override {
    stop_ = true;
    if (guide_) guide_->cancel();
    if (stream_) stream_->stop();
    for (auto &t : media_threads_) if (t.joinable()) t.join();
    if (udp_fd_ >= 0) close(udp_fd_);
    if (rx_.joinable()) rx_.join();
    close(in_fd_);
    close(out_fd_);
    if (in_path_[0] != '@') unlink(in_path_.c_str());
  }

 private:
  // ---------------------------------------------------------------- 时间
  double now_s() { return get_clock()->now().nanoseconds() * 1e-9; }

  void track_offset(double t_sim) {
    double o = now_s() - t_sim;
    std::lock_guard<std::mutex> lk(tmu_);
    if (!has_off_ || o < off_ || std::fabs(o - off_) > 1.0) {
      off_ = o;
      has_off_ = true;
    } else {
      off_ = off_ + 0.005 * (o - off_);
    }
  }

  builtin_interfaces::msg::Time sim_stamp(double t_sim) {
    double now = now_s();
    std::lock_guard<std::mutex> lk(tmu_);
    if (!has_off_ || std::isnan(t_sim)) return to_msg(now);
    return to_msg(std::min(t_sim + off_, now));
  }

  // ---------------------------------------------------------------- 接收
  void rx_loop() {
    std::vector<uint8_t> buf(1 << 20);
    while (!stop_ && rclcpp::ok()) {
      ssize_t n = recv(in_fd_, buf.data(), buf.size(), 0);
      if (n <= 5) continue;
      if (std::memcmp(buf.data(), MAGIC, 4) != 0) continue;
      Reader r{buf.data() + 5, buf.data() + n};
      try {
        if (buf[4] == T_STATE) on_state(r);
        else if (buf[4] == T_SCAN) on_scan(r);
        else if (buf[4] == T_ROUTE) on_route(std::string(reinterpret_cast<const char *>(buf.data() + 5), n - 5));
        else if (buf[4] == T_CANCEL) cancel_route();
        else if (buf[4] == T_CONFIG) on_config(std::string(reinterpret_cast<const char *>(buf.data() + 5), n - 5));
        else if (buf[4] == T_MODE) on_mode(std::string(reinterpret_cast<const char *>(buf.data() + 5), n - 5));
        else if (buf[4] == T_GUIDE) on_guide(std::string(reinterpret_cast<const char *>(buf.data() + 5), n - 5));
        else if (buf[4] == T_GUIDE_CANCEL) { if (guide_) guide_->cancel(); }
        else if (buf[4] == T_ROSCALL) on_roscall(std::string(reinterpret_cast<const char *>(buf.data() + 5), n - 5));
      } catch (const std::exception &e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "消息处理失败: %s", e.what());
      }
    }
  }

  void on_state(Reader &r) {
    double t = r.get<double>();
    double o[6], tr[3], m2o[3], im[4];
    for (double &v : o) v = r.get<double>();
    for (double &v : tr) v = r.get<double>();
    for (double &v : m2o) v = r.get<double>();
    for (double &v : im) v = r.get<double>();
    uint8_t flags = r.get<uint8_t>();
    uint16_t nj = r.get<uint16_t>();
    std::vector<std::string> names(nj);
    for (auto &s : names) s = r.str();
    std::vector<double> jp(nj), jv(nj), je(nj);
    for (auto *vec : {&jp, &jv, &je})
      for (auto &v : *vec) v = r.get<double>();
    if (!r.ok) return;
    publish_state(t, o, tr, m2o, im, flags, std::move(names), std::move(jp), std::move(jv), std::move(je));
  }

  void publish_state(double t, const double *o, const double *tr, const double *m2o, const double *im, uint8_t flags,
                     std::vector<std::string> names, std::vector<double> jp, std::vector<double> jv, std::vector<double> je) {
    if (flags & F_HAS_T) track_offset(t);
    auto stamp = sim_stamp((flags & F_HAS_T) ? t : NAN);

    nav_msgs::msg::Odometry od;
    od.header.stamp = stamp;
    od.header.frame_id = "odom";
    od.child_frame_id = "base_footprint";
    od.pose.pose.position.x = o[0];
    od.pose.pose.position.y = o[1];
    set_quat(od.pose.pose.orientation, o[2]);
    od.twist.twist.linear.x = o[3];
    od.twist.twist.linear.y = o[4];
    od.twist.twist.angular.z = o[5];
    od.pose.covariance.fill(0.0);
    od.pose.covariance[0] = od.pose.covariance[7] = 0.02 * 0.02;
    od.pose.covariance[35] = std::pow(0.5 * M_PI / 180.0, 2);
    od.pose.covariance[14] = od.pose.covariance[21] = od.pose.covariance[28] = 1e6;
    od.twist.covariance.fill(0.0);
    od.twist.covariance[0] = od.twist.covariance[7] = od.twist.covariance[35] = 0.01 * 0.01;
    od.twist.covariance[14] = od.twist.covariance[21] = od.twist.covariance[28] = 1e6;
    odom_pub_->publish(od);

    if (flags & F_IMU) {
      sensor_msgs::msg::Imu m;
      m.header.stamp = stamp;
      m.header.frame_id = "imu_link";
      m.angular_velocity.z = im[0];
      m.linear_acceleration.x = im[1];
      m.linear_acceleration.y = im[2];
      m.linear_acceleration.z = im[3];
      m.orientation_covariance[0] = -1.0;
      m.angular_velocity_covariance[0] = m.angular_velocity_covariance[4] = 1e-4;
      m.angular_velocity_covariance[8] = 0.02 * 0.02;
      m.linear_acceleration_covariance[0] = m.linear_acceleration_covariance[4] = m.linear_acceleration_covariance[8] = 0.02 * 0.02;
      imu_pub_->publish(m);
    }

    nav_msgs::msg::Odometry gt;
    gt.header.stamp = stamp;
    gt.header.frame_id = "map";
    gt.child_frame_id = "base_footprint";
    gt.pose.pose.position.x = tr[0];
    gt.pose.pose.position.y = tr[1];
    set_quat(gt.pose.pose.orientation, tr[2]);
    gt_pub_->publish(gt);

    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    bool own = flags & F_OWN_ODOM;
    if (own) {
      geometry_msgs::msg::TransformStamped ts;
      ts.header.stamp = stamp;
      ts.header.frame_id = "odom";
      ts.child_frame_id = "base_footprint";
      ts.transform.translation.x = o[0];
      ts.transform.translation.y = o[1];
      set_quat(ts.transform.rotation, o[2]);
      tfs.push_back(ts);
    }
    if (flags & F_MAP_ODOM) {
      geometry_msgs::msg::TransformStamped mt;
      mt.header.stamp = stamp;
      mt.header.frame_id = "map";
      mt.child_frame_id = "odom";
      mt.transform.translation.x = m2o[0];
      mt.transform.translation.y = m2o[1];
      set_quat(mt.transform.rotation, m2o[2]);
      tfs.push_back(mt);
    }
    if (!tfs.empty()) tfb_->sendTransform(tfs);
    {
      std::lock_guard<std::mutex> lk(tmu_);
      own_odom_ = own;
      odom_stamp_ = to_sec(stamp);
      has_odom_stamp_ = true;
    }
    if (++n_state_ % 2 == 0) {
      sensor_msgs::msg::JointState js;
      js.header.stamp = stamp;
      js.name = std::move(names);
      js.position = std::move(jp);
      js.velocity = std::move(jv);
      js.effort = std::move(je);
      js_pub_->publish(js);
    }
  }

  builtin_interfaces::msg::Time costmap_stamp(const builtin_interfaces::msg::Time &stamp) {
    std::lock_guard<std::mutex> lk(tmu_);
    if (!has_odom_stamp_ || !clamp_) return stamp;
    double lim = odom_stamp_ - (std::isnan(margin_env_) ? (own_odom_ ? 0.0 : 0.06) : margin_env_);
    if (to_sec(stamp) <= lim) return stamp;
    return to_msg(lim);
  }

  void on_scan(Reader &r) {
    uint8_t kind = r.get<uint8_t>();
    std::string name = r.str(), frame = r.str();
    double t = r.get<double>(), a0 = r.get<double>(), inc = r.get<double>();
    double rmin = r.get<double>(), rmax = r.get<double>(), hz = r.get<double>();
    uint32_t n = r.get<uint32_t>();
    if (!r.ok || r.p + 4ull * n > r.end) return;
    publish_scan(kind, name, frame, t, a0, inc, rmin, rmax, hz, reinterpret_cast<const float *>(r.p), n);
  }

  void publish_scan(uint8_t kind, const std::string &name, const std::string &frame, double t, double a0, double inc,
                    double rmin, double rmax, double hz, const float *ranges, uint32_t n) {
    sensor_msgs::msg::LaserScan s;
    auto stamp = sim_stamp(t);
    s.header.stamp = kind == 0 ? costmap_stamp(stamp) : stamp;
    s.header.frame_id = frame.empty() ? "base_link" : frame;
    s.angle_min = static_cast<float>(a0);
    s.angle_increment = static_cast<float>(inc);
    s.angle_max = static_cast<float>(a0 + inc * (static_cast<double>(n) - 1));
    s.range_min = static_cast<float>(rmin);
    s.range_max = static_cast<float>(rmax);
    s.scan_time = static_cast<float>(1.0 / std::max(1.0, hz));
    s.ranges.resize(n);
    std::memcpy(s.ranges.data(), ranges, 4ull * n);
    if (kind == 1) {
      scan_pub_->publish(s);
      ++n_merged_;
      return;
    }
    auto it = lidar_pubs_.find(name);
    if (it == lidar_pubs_.end())
      it = lidar_pubs_.emplace(name, create_publisher<sensor_msgs::msg::LaserScan>("/scan/" + name, sensor_qos_)).first;
    it->second->publish(s);
    ++n_scan_;
  }

  // ================================================================ 核心模式
  void start_core() {
    const char *u = std::getenv("SIM_API");
    sim_url_ = (u && *u) ? u : "http://127.0.0.1:8090";
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
      on_cmd_vel(m->linear.x, m->linear.y, m->angular.z);
    });
    udp_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    stream_ = std::make_unique<simstream::Stream>(
        sim_url_, [this](uint8_t ty, const uint8_t *b, size_t n) { on_stream_frame(ty, b, n); },
        [this](bool up) {
          stream_up_ = up;
          if (up) refresh_cmd_channel();
          RCLCPP_INFO(get_logger(), "仿真推送流%s: %s", up ? "已连接" : "断开", sim_url_.c_str());
        });
    stream_->start();
    safety_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] { send_safety(false); });
    // ---- 执行进程的其余 ROS 接口 (Python 不再需要 rclpy): Nav2 生命周期、地图、定位栈、ROS 图
    active_cli_ = create_client<std_srvs::srv::Trigger>("/lifecycle_manager_navigation/is_active");
    loadmap_cli_ = create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    savemap_cli_ = create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");
    setpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/set_pose", 10);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).reliable().transient_local(), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { on_map(*m); });
    ros_timer_ = create_wall_timer(std::chrono::seconds(2), [this] { poll_ros(); });
  }

  void refresh_cmd_channel() {
    int st = -1;
    std::string body = simstream::http_get(stream_->url(), "/api/v1/sim", &st);
    jl::Value v;
    int port = 0;
    if (st == 200 && jl::parse(body, v)) port = static_cast<int>(v["cmd_udp_port"].num(0));
    const char *nu = std::getenv("NAV_CMD_UDP");
    if (nu && std::string(nu) == "0") port = 0;
    std::lock_guard<std::mutex> lk(cmu_);
    udp_port_ = port;
    if (port > 0) {
      addrinfo hints{}, *res = nullptr;
      hints.ai_family = AF_INET;
      hints.ai_socktype = SOCK_DGRAM;
      if (getaddrinfo(stream_->url().host.c_str(), std::to_string(port).c_str(), &hints, &res) == 0 && res) {
        std::memcpy(&udp_addr_, res->ai_addr, sizeof(sockaddr_in));
        freeaddrinfo(res);
      } else {
        udp_port_ = 0;
      }
    }
  }

  // 仿真推送流帧: 解析需要的部分，并原样转给 Python
  void on_stream_frame(uint8_t ty, const uint8_t *b, size_t n) {
    if (ty == 1) on_sim_state(b, n);
    else if (ty == 2) on_sim_meta(b, n);
    else if (ty == 3) on_sim_io(b, n);
    else if (ty == 4) { on_sim_scan(b, n); return; }   // 激光按需转发 (on_sim_scan 内)
    relay(ty, b, n);
  }

  void relay(uint8_t ty, const uint8_t *b, size_t n) {
    std::vector<uint8_t> out(MAGIC, MAGIC + 4);
    out.push_back(static_cast<uint8_t>(T_RELAY + ty));
    out.insert(out.end(), b, b + n);
    send(out);
  }

  void on_sim_meta(const uint8_t *b, size_t n) {
    jl::Value v;
    if (!jl::parse(std::string(reinterpret_cast<const char *>(b), n), v)) return;
    std::vector<std::string> names;
    for (const auto &x : v["joint_names"].a) names.push_back(x.str());
    std::lock_guard<std::mutex> lk(smu_);
    joint_names_ = std::move(names);
  }

  // 状态帧 (sim_server STATE_FMT "<Idd6d6d3d5dIBddIH" + 3·nj 个 double)
  void on_sim_state(const uint8_t *b, size_t n) {
    constexpr size_t HEAD = 4 + 8 * 2 + 8 * 6 + 8 * 6 + 8 * 3 + 8 * 5 + 4 + 1 + 8 * 2 + 4 + 2;
    if (n < HEAD) return;
    Reader r{b, b + n};
    r.get<uint32_t>();
    const double t = r.get<double>();
    r.get<double>();
    double tr6[6], od6[6], mo[3], im5[5];
    for (double &v : tr6) v = r.get<double>();
    for (double &v : od6) v = r.get<double>();
    for (double &v : mo) v = r.get<double>();
    for (double &v : im5) v = r.get<double>();
    r.get<uint32_t>();
    const uint8_t sflags = r.get<uint8_t>();
    r.get<double>();
    r.get<double>();
    r.get<uint32_t>();
    const uint16_t nj = r.get<uint16_t>();
    std::vector<double> jp(nj), jv(nj), je(nj);
    for (auto *vec : {&jp, &jv, &je})
      for (auto &v : *vec) v = r.get<double>();
    if (!r.ok) return;
    std::vector<std::string> names;
    uint8_t flags = F_HAS_T | F_IMU;
    double m2o[3] = {0, 0, 0};
    {
      std::lock_guard<std::mutex> lk(smu_);
      names = joint_names_;
      if (mode_own_odom_) flags |= F_OWN_ODOM;
      if (mode_map_odom_) { flags |= F_MAP_ODOM; std::copy(mode_m2o_, mode_m2o_ + 3, m2o); }
      v_meas_ = od6[3];
      w_meas_ = od6[5];
      paused_ = sflags & 4;
      odom_now_ = {od6[0], od6[1], od6[2]};
      odom_hist_.push_back({t, od6[0], od6[1], od6[2]});
      while (odom_hist_.size() > 400) odom_hist_.pop_front();     // 8 s @ 50 Hz
      steer_.clear();
      for (size_t i = 0; i < nj && i < joint_names_.size(); ++i) {
        std::string lo = joint_names_[i];
        for (auto &ch : lo) ch = static_cast<char>(std::tolower(ch));
        if (lo.find("steer") != std::string::npos) steer_.push_back(jp[i]);
      }
    }
    if (names.size() != nj) {
      names.resize(nj);
      for (size_t i = 0; i < nj; ++i) if (names[i].empty()) names[i] = "j" + std::to_string(i);
    }
    const double o[6] = {od6[0], od6[1], od6[2], od6[3], od6[4], od6[5]};
    const double trv[3] = {tr6[0], tr6[1], tr6[2]};
    const double im[4] = {im5[0], im5[1], im5[2], im5[3]};
    publish_state(t, o, trv, m2o, im, flags, std::move(names), std::move(jp), std::move(jv), std::move(je));
    ++n_stream_state_;
  }

  void on_sim_io(const uint8_t *b, size_t n) {
    jl::Value v;
    if (!jl::parse(std::string(reinterpret_cast<const char *>(b), n), v)) return;
    const auto &di = v["io"]["inputs"];
    std::lock_guard<std::mutex> lk(smu_);
    estop_ = di["di_estop"].truthy();
    photo_on_.clear();
    photo_dist_.clear();
    for (const auto &kv : di.o) if (kv.second.truthy()) photo_on_.push_back(kv.first);
    for (const auto &p : v["photos"].a) {
      const auto &d = p["distance_m"];
      photo_dist_[p["name"].str()] = d.is_null() ? -1.0 : d.num(-1.0);
    }
  }

  // 激光帧: <u16 元信息长度><元信息 JSON><float32 ranges>
  void on_sim_scan(const uint8_t *b, size_t n) {
    if (n < 2) return;
    uint16_t ml;
    std::memcpy(&ml, b, 2);
    if (2u + ml > n) return;
    jl::Value m;
    if (!jl::parse(std::string(reinterpret_cast<const char *>(b + 2), ml), m)) return;
    const size_t cnt = (n - 2 - ml) / 4;
    std::vector<float> rs(cnt);
    std::memcpy(rs.data(), b + 2 + ml, cnt * 4);
    const std::string name = m["name"].str();
    const bool merged = name == "merged";
    const double t = m["t"].num(NAN), a0 = m["angle_min"].num(), inc = m["angle_increment"].num();
    const double rmin = m["range_min"].num(0.05), rmax = m["range_max"].num(30.0), hz = m["scan_hz"].num(10.0);
    std::string frame = m["frame_id"].str();
    if (frame.empty()) frame = merged ? "base_link" : name + "_link";
    for (auto &x : rs) if (!std::isfinite(x)) x = std::numeric_limits<float>::infinity();
    publish_scan(merged ? 1 : 0, name, frame, t, a0, inc, rmin, rmax, hz, rs.data(), static_cast<uint32_t>(cnt));
    if (merged) {
      // 融合扫描 (机体系) → 各档防护区走廊最近障碍；点集供原地转向防护
      std::vector<agvsafe::Pt> pts;
      pts.reserve(cnt);
      for (size_t i = 0; i < cnt; ++i) {
        const double r = rs[i];
        if (!(r > 0.02) || !(r < rmax - 1e-3)) continue;
        const double a = a0 + inc * static_cast<double>(i);
        pts.push_back({r * std::cos(a), r * std::sin(a)});
      }
      {
        std::lock_guard<std::mutex> lk(smu_);
        bands_ = agvsafe::compute_bands(cfg_, pts);
        pts_.swap(pts);
      }
      relay(4, b, n);                      // 界面/执行进程用 (10 Hz)
      send_safety(false);
    } else {
      if (want_raw_lidar_) relay(4, b, n);   // 内置 SLAM 需要原始激光帧时才转发
      std::lock_guard<std::mutex> lk(smu_);
      if (!refine_lidar_.empty() && name == refine_lidar_) {        // 精定位用主激光原始帧 (机体系)
        const double sg = std::cos(refine_mount_[3]) >= 0 ? 1.0 : -1.0;     // 倒装 (roll = π) 扫描方向相反
        refine_pts_.clear();
        for (size_t i = 0; i < cnt; ++i) {
          const double r = rs[i];
          if (!std::isfinite(r) || r < rmin + 1e-3 || r >= rmax - 1e-3) continue;
          const double a = refine_mount_[2] + sg * (a0 + inc * static_cast<double>(i));
          refine_pts_.push_back({refine_mount_[0] + r * std::cos(a), refine_mount_[1] + r * std::sin(a)});
        }
        refine_t_ = wall_now();
      }
    }
  }

  void on_config(const std::string &j) {
    jl::Value v;
    if (!jl::parse(j, v)) {
      RCLCPP_WARN(get_logger(), "安全层配置解析失败 (%zu 字节)", j.size());
      return;
    }
    agvsafe::Config c;
    const auto &P = v["prot"];
    c.enabled = P["enabled"].truthy(true);
    for (const auto &f : P["fields"].a) {
      agvsafe::Field F;
      F.name = f["name"].str();
      F.v_max = f["v_max"].num(9.0);
      F.front = f["front"].num(0.3);
      F.rear = f["rear"].num(0.2);
      F.side = f["side"].num(0.08);
      c.fields.push_back(F);
    }
    c.slow_ratio = P["slow_ratio"].num(2.0);
    c.rotate_margin = P["rotate_margin"].num(0.02);
    c.rotate_lookahead = P["rotate_lookahead_rad"].num(0.25);
    c.reaction_s = P["reaction_s"].num(0.3);
    c.docking_front = P["docking"]["front"].num(0.02);
    const auto &ph = P["photo"];
    if (!ph.is_null()) {
      c.photo_mode = ph["mode"].str().empty() ? "field" : ph["mode"].str();
      c.photo_front = ph["front"].num(0.3);
      c.photo_rear = ph["rear"].num(0.3);
      c.photo_side = ph["side"].num(0.1);
      c.mute_near_stop = ph["mute_near_stop"].num(0.1);
    }
    const auto &o = v["outline"];
    c.h = o[0].num(0.6); c.t = o[1].num(0.6); c.l = o[2].num(0.4); c.r = o[3].num(0.4);
    c.max_decel = v["max_decel"].num(0.5);
    c.ang_decel = v["max_ang_decel"].num(1.0);
    c.loc_stale_s = v["loc_stale_s"].num(1.0);
    for (const auto &p : v["photos"].a) {
      agvsafe::Photo ph2;
      ph2.name = p["name"].str();
      ph2.di = p["di"].str();
      ph2.x = p["x"].num(); ph2.y = p["y"].num(); ph2.yaw = p["yaw"].num();
      c.photos.push_back(ph2);
    }
    std::lock_guard<std::mutex> lk(smu_);
    cfg_ = std::move(c);
    have_cfg_ = true;
    if (!have_media_log_ && (!v["media"]["lidars3d"].a.empty() || !v["media"]["cameras"].a.empty())) {
      have_media_log_ = true;
      RCLCPP_INFO(get_logger(), "3D 激光 %zu 台、相机 %zu 台由 C++ 从仿真拉取发布", v["media"]["lidars3d"].a.size(),
                  v["media"]["cameras"].a.size());
    }
    for (const auto &l : v["media"]["lidars3d"].a) start_lidar3d(l["name"].str(), l["topic"].str(), l["frame_id"].str());
    for (const auto &c : v["media"]["cameras"].a) start_camera(c);
    const auto &rl = v["refine_lidar"];
    refine_lidar_ = rl["name"].str();
    refine_mount_[0] = rl["x"].num(); refine_mount_[1] = rl["y"].num(); refine_mount_[2] = rl["yaw"].num(); refine_mount_[3] = rl["roll"].num();
  }

  void on_mode(const std::string &j) {
    jl::Value v;
    if (!jl::parse(j, v)) return;
    std::lock_guard<std::mutex> lk(smu_);
    const bool fwd = v["nav2_forward"].truthy();
    if (!fwd && nav2_fwd_) zone_ = "clear";           // Nav2 任务结束: 防护区状态交还执行进程
    nav2_fwd_ = fwd;
    mode_own_odom_ = v["own_odom"].truthy();
    mode_map_odom_ = v["map_odom"].truthy();
    for (int i = 0; i < 3; ++i) mode_m2o_[i] = v["m2o"][i].num();
    want_raw_lidar_ = v["want_raw_lidar"].truthy();
    loc_ext_ = v["loc_ext"].truthy();
    speed_cap_ = v["speed_cap"].num(0.0);
  }

  // Nav2 /cmd_vel (velocity_smoother 输出) → 安全层 → 仿真
  void on_cmd_vel(double vx, double vy, double wz) {
    {
      std::lock_guard<std::mutex> lk(smu_);
      if (!nav2_fwd_ || !have_cfg_) return;
    }
    const double sd_age = wall_now() - stop_wall_;
    filter_and_send(vx, vy, wz, false, sd_age < 0.5, stop_dist_, true, "nav2");   // 停车点剩余行程: RouteController 20 Hz 以上刷新
  }

  // 安全层 (navigator.safety_filter) → 仿真指令；loc_check: 定位停更检查 (只对 Nav2)
  void filter_and_send(double vx, double vy, double wz, bool in_arc, bool has_left, double left, bool loc_check, const char *source,
                       double rot_left = -1.0) {
    agvsafe::Result r;
    std::string ev_type, ev_level, ev_title, ev_msg, ev_cat = "sensors";
    {
      std::lock_guard<std::mutex> lk(smu_);
      if (!have_cfg_) { send_cmd(vx, vy, wz, source); return; }
      agvsafe::Env e;
      e.estop = estop_;
      const double now_w = wall_now();
      e.loc_check = loc_check && loc_ext_ && last_tf_wall_ > 0.0;
      e.loc_age = now_w - last_tf_wall_;
      e.speed_cap = speed_cap_;
      e.has_left = has_left;
      e.approach_left = left;
      e.in_arc = in_arc;
      e.v_meas = v_meas_;
      e.w_meas = w_meas_;
      e.rot_left = rot_left;
      e.bands = bands_;
      e.pts = &pts_;
      for (const auto &p : cfg_.photos) {
        if (std::find(photo_on_.begin(), photo_on_.end(), p.di) == photo_on_.end()) continue;
        auto it = photo_dist_.find(p.name);
        e.photo_hits.emplace_back(&p, it == photo_dist_.end() ? -1.0 : it->second);
      }
      r = agvsafe::filter(cfg_, e, vx, vy, wz);
      // 定位停更事件
      if (r.loc_stale && !loc_stale_) {
        loc_stale_ = true;
        ev_cat = "localization"; ev_type = "LOC_STALE"; ev_level = "warning"; ev_title = "定位停更，停车等待";
        char b[160];
        std::snprintf(b, sizeof(b), "slam_toolbox 定位已 %.1f s 未更新，Nav2 路径换算不可信", e.loc_age);
        ev_msg = b;
      } else if (!r.loc_stale && loc_stale_ && e.loc_check) {
        loc_stale_ = false;
        ev_cat = "localization"; ev_type = "LOC_RESUME"; ev_level = "info"; ev_title = "定位恢复";
      }
      if (!r.zone.empty()) zone_event(r, ev_type, ev_level, ev_title, ev_msg);
      photo_ignored_ = r.photo_ignored;
      layer_now_ = r.layer;
    }
    if (!ev_type.empty()) send_event(ev_cat, ev_type, ev_level, ev_title, ev_msg);
    send_cmd(r.vx, r.vy, r.wz, source);
  }

  // 防护区状态机 (navigator._zone): 预警 3 s 内只报一次；进入 slow/stop 报事件；恢复到 clear 报解除
  void zone_event(const agvsafe::Result &r, std::string &ty, std::string &lv, std::string &ti, std::string &msg) {
    const std::string prev = zone_, zone = r.zone;
    const double now = wall_now();
    if (zone == "warn") {
      if (prev == "clear" && now - t_warn_ > 3.0) {
        t_warn_ = now;
        ty = "OBS_WARN"; lv = "info"; ti = "近距避障: 预警区有障碍"; msg = "当前档预警区内探测到障碍，保持速度并准备降档";
      }
      zone_ = "warn";
      return;
    }
    if (zone == prev || (prev == "warn" && zone == "clear")) { zone_ = zone; return; }
    zone_ = zone;
    if (zone == "clear" && now - t_zone_ < 0.5) return;
    t_zone_ = now;
    char b[200];
    const char *where = r.layer == "field_front" ? "前向防护区" : (r.layer == "field_rear" ? "后向防护区" : (r.layer == "rotate" ? "转向防护区" : "防护区"));
    if (zone == "slow") {
      ty = "OBS_SLOW"; lv = "warning"; ti = "近距避障: 减速避让";
      std::snprintf(b, sizeof(b), "%s: 障碍 %.2f m < %.2f m，按防护区分档降速", where, r.d_hit, r.need);
      msg = b;
    } else if (zone == "stop") {
      ty = "OBS_STOP"; lv = "danger"; ti = "近距避障: 停车等待";
      if (r.layer == "rotate") std::snprintf(b, sizeof(b), "%s: 原地转向扫掠区 (外扩 %.2f m) 内有障碍，停止转向", where, cfg_.rotate_margin);
      else std::snprintf(b, sizeof(b), "%s: 障碍 %.2f m < 最低档停车距离 %.2f m，停车等待", where, r.d_hit, r.need);
      msg = b;
    } else if (prev == "slow" || prev == "stop") {
      ty = "OBS_CLEAR"; lv = "success"; ti = "障碍解除，恢复巡航";
    }
  }

  void send_event(const std::string &cat, const std::string &type, const std::string &level, const std::string &title,
                  const std::string &msg) {
    send_nav("{\"k\":\"sevent\",\"cat\":\"" + jesc(cat) + "\",\"type\":\"" + jesc(type) + "\",\"level\":\"" + jesc(level) +
             "\",\"title\":\"" + jesc(title) + "\",\"msg\":\"" + jesc(msg) + "\"}");
  }

  void send_cmd(double vx, double vy, double wz, const char *source) {
    std::lock_guard<std::mutex> lk(cmu_);
    if (udp_port_ > 0 && udp_fd_ >= 0) {
      uint8_t pkt[48] = {'A', 'G', 'V', 'C'};
      const uint32_t seq = ++cmd_seq_;
      std::memcpy(pkt + 4, &seq, 4);
      std::memcpy(pkt + 8, &vx, 8);
      std::memcpy(pkt + 16, &vy, 8);
      std::memcpy(pkt + 24, &wz, 8);
      std::memcpy(pkt + 32, source, std::min<size_t>(std::strlen(source), 16));
      if (sendto(udp_fd_, pkt, sizeof(pkt), MSG_DONTWAIT, reinterpret_cast<const sockaddr *>(&udp_addr_), sizeof(udp_addr_)) == 48) {
        ++n_cmd_;
        return;
      }
      ++n_cmd_err_;
    }
    // 无 UDP 通道: REST PUT (短连接)
    int fd = simstream::connect_tcp(stream_->url(), 300);
    if (fd < 0) { ++n_cmd_err_; return; }
    char body[160];
    int bl = std::snprintf(body, sizeof(body), "{\"vx\":%.6f,\"vy\":%.6f,\"wz\":%.6f,\"source\":\"%s\"}", vx, vy, wz, source);
    std::string req = "PUT " + stream_->url().base + "/api/v1/control/cmd_vel HTTP/1.1\r\nHost: " + stream_->url().host +
                      "\r\nContent-Type: application/json\r\nContent-Length: " + std::to_string(bl) + "\r\nConnection: close\r\n\r\n" +
                      std::string(body, bl);
    if (simstream::send_all(fd, req)) { char tmp[256]; recv(fd, tmp, sizeof(tmp), 0); ++n_cmd_; } else ++n_cmd_err_;
    close(fd);
  }

  // 安全层快照 → Python (界面/执行进程导引): 各档走廊最近障碍、当前档、Nav2 执行中的防护区状态、链路统计
  void send_safety(bool) {
    std::string j;
    {
      std::lock_guard<std::mutex> lk(smu_);
      if (!have_cfg_) return;
      const int band = cfg_.fields.empty() ? 0 : agvsafe::field_for_speed(cfg_, v_meas_);
      char b[96];
      j = "{\"bands\":[";
      for (size_t i = 0; i < bands_.size(); ++i) {
        std::snprintf(b, sizeof(b), "%s[%s,%s]", i ? "," : "", bands_[i].first < 0 ? "null" : std::to_string(bands_[i].first).c_str(),
                      bands_[i].second < 0 ? "null" : std::to_string(bands_[i].second).c_str());
        j += b;
      }
      std::snprintf(b, sizeof(b), "],\"band\":%d,\"nav2\":%s,\"zone\":\"", band, nav2_fwd_ ? "true" : "false");
      j += b;
      j += zone_ + "\",\"layer\":\"" + (zone_ == "slow" || zone_ == "stop" ? layer_now_ : std::string()) + "\",\"photo_ignored\":[";
      for (size_t i = 0; i < photo_ignored_.size(); ++i) j += (i ? ",\"" : "\"") + jesc(photo_ignored_[i]) + "\"";
      std::snprintf(b, sizeof(b), "],\"stream\":%s,\"state_frames\":%u,\"cmd_sent\":%u,\"cmd_errors\":%u,\"udp\":%d}",
                    stream_up_ ? "true" : "false", n_stream_state_.load(), n_cmd_.load(), n_cmd_err_.load(), udp_port_);
      j += b;
    }
    std::vector<uint8_t> out(MAGIC, MAGIC + 4);
    out.push_back(T_SAFETY);
    out.insert(out.end(), j.begin(), j.end());
    send(out);
  }

  // ================================================================ 3D 激光 / 相机 (核心模式: Python 不再搬运点云与图像)
  // 3D 激光: 长轮询 /api/v1/sensors/lidars/{name} (二进制: float32[N,4] xyzi + uint8[N] line) → PointCloud2 (livox_ros_driver2 布局)
  void start_lidar3d(const std::string &name, const std::string &topic, const std::string &frame) {
    std::lock_guard<std::mutex> lk(media_mu_);
    if (name.empty() || media_names_.count("lidar:" + name)) return;
    media_names_.insert("lidar:" + name);
    auto pub = create_publisher<sensor_msgs::msg::PointCloud2>(topic.empty() ? "/points/" + name : topic, sensor_qos_);
    media_threads_.emplace_back([this, name, frame, pub] {
      long seq = -1;
      while (!stop_) {
        auto r = simstream::http_get_full(stream_->url(), "/api/v1/sensors/lidars/" + name + "?after_seq=" + std::to_string(seq) + "&wait=0.5",
                                          3000, "application/octet-stream");
        if (r.status != 200) { std::this_thread::sleep_for(std::chrono::milliseconds(300)); continue; }
        const long s2 = std::atol(r.headers["x-seq"].c_str());
        if (s2 == seq) continue;
        seq = s2;
        jl::Value m;
        if (!jl::parse(r.headers["x-meta"], m) || m["type"].str() != "3d") continue;
        const size_t n = static_cast<size_t>(m["count"].num());
        if (r.body.size() < n * 17) continue;
        const auto *xyzi = reinterpret_cast<const float *>(r.body.data());
        const auto *line = reinterpret_cast<const uint8_t *>(r.body.data() + n * 16);
        sensor_msgs::msg::PointCloud2 pc;
        const auto stamp = sim_stamp(m["t"].num(NAN));
        pc.header.stamp = stamp;
        pc.header.frame_id = !m["frame_id"].str().empty() ? m["frame_id"].str() : (frame.empty() ? name + "_link" : frame);
        pc.height = 1;
        pc.width = static_cast<uint32_t>(n);
        auto field = [](const char *nm, uint32_t off, uint8_t dt) {
          sensor_msgs::msg::PointField f; f.name = nm; f.offset = off; f.datatype = dt; f.count = 1; return f; };
        pc.fields = {field("x", 0, 7), field("y", 4, 7), field("z", 8, 7), field("intensity", 12, 7), field("tag", 16, 2),
                     field("line", 17, 2), field("timestamp", 18, 8)};
        pc.is_bigendian = false;
        pc.point_step = 26;
        pc.row_step = 26 * pc.width;
        pc.is_dense = true;
        pc.data.resize(26 * n);
        const double ts = stamp.sec * 1e9 + stamp.nanosec;
        for (size_t i = 0; i < n; ++i) {
          uint8_t *d = pc.data.data() + 26 * i;
          std::memcpy(d, xyzi + 4 * i, 16);
          d[16] = 0;
          d[17] = line[i];
          std::memcpy(d + 18, &ts, 8);
        }
        pub->publish(pc);
        ++n_media_;
      }
    });
  }

  // 相机: 有订阅者才拉 (与 NAV_CAMERAS=auto 相同)；各流 raw 格式 → Image / CameraInfo / PointCloud2
  void start_camera(const jl::Value &c) {
    const std::string name = c["name"].str();
    std::lock_guard<std::mutex> lk(media_mu_);
    if (name.empty() || media_names_.count("cam:" + name)) return;
    media_names_.insert("cam:" + name);
    struct Stream { std::string st, topic; rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img;
                    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info;
                    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pts; };
    auto streams = std::make_shared<std::vector<Stream>>();
    static const std::map<std::string, std::string> TOP = {{"rgb", "image_raw"}, {"left", "left/image_raw"}, {"right", "right/image_raw"},
                                                           {"depth", "depth/image_raw"}, {"amplitude", "amplitude/image_raw"}, {"points", "points"}};
    for (const auto &sv : c["streams"].a) {
      Stream S;
      S.st = sv.str();
      auto it = TOP.find(S.st);
      S.topic = "/" + name + "/" + (it == TOP.end() ? S.st : it->second);
      if (S.st == "points") S.pts = create_publisher<sensor_msgs::msg::PointCloud2>(S.topic, sensor_qos_);
      else {
        S.img = create_publisher<sensor_msgs::msg::Image>(S.topic, sensor_qos_);
        S.info = create_publisher<sensor_msgs::msg::CameraInfo>(S.topic.substr(0, S.topic.rfind('/')) + "/camera_info", sensor_qos_);
      }
      streams->push_back(S);
    }
    std::vector<double> K;
    for (const auto &k : c["K"].a) K.push_back(k.num());
    const double baseline = c["baseline_m"].num(0.0);
    const std::string frame = c["frame_id"].str();
    media_threads_.emplace_back([this, name, streams, K, baseline, frame] {
      long seq = -1;
      while (!stop_) {
        size_t subs = 0;
        for (auto &S : *streams) subs += S.pts ? S.pts->get_subscription_count() : S.img->get_subscription_count() + S.info->get_subscription_count();
        if (!subs || streams->empty()) { std::this_thread::sleep_for(std::chrono::milliseconds(500)); continue; }
        bool first = true;
        const auto stamp = get_clock()->now();
        for (auto &S : *streams) {
          std::string q = "/api/v1/sensors/cameras/" + name + "?stream=" + S.st + "&format=raw";
          if (first) q += "&after_seq=" + std::to_string(seq) + "&wait=1.0";
          auto r = simstream::http_get_full(stream_->url(), q, 4000, "application/octet-stream");
          if (r.status != 200) { std::this_thread::sleep_for(std::chrono::milliseconds(300)); break; }
          if (first) {
            const long s2 = std::atol(r.headers["x-seq"].c_str());
            if (s2 == seq) break;
            seq = s2;
            first = false;
          }
          jl::Value m;
          jl::parse(r.headers["x-meta"], m);
          const uint32_t W = static_cast<uint32_t>(m["width"].num()), H = static_cast<uint32_t>(m["height"].num());
          const std::string fr = S.st == "right" ? name + "_right_optical_frame" : frame;
          if (S.pts) {
            sensor_msgs::msg::PointCloud2 pc;
            pc.header.stamp = stamp;
            pc.header.frame_id = fr;
            const uint32_t n = static_cast<uint32_t>(r.body.size() / 12);
            pc.height = 1; pc.width = n;
            for (auto [nm, off] : {std::pair<const char *, uint32_t>{"x", 0}, {"y", 4}, {"z", 8}}) {
              sensor_msgs::msg::PointField f; f.name = nm; f.offset = off; f.datatype = 7; f.count = 1; pc.fields.push_back(f);
            }
            pc.is_bigendian = false; pc.point_step = 12; pc.row_step = 12 * n; pc.is_dense = true;
            pc.data.assign(r.body.begin(), r.body.begin() + 12 * n);
            S.pts->publish(pc);
            continue;
          }
          sensor_msgs::msg::Image img;
          img.header.stamp = stamp;
          img.header.frame_id = fr;
          img.height = H; img.width = W;
          if (S.st == "rgb" || S.st == "left" || S.st == "right") { img.encoding = "rgb8"; img.step = W * 3; }
          else if (S.st == "amplitude") { img.encoding = "16UC1"; img.step = W * 2; }
          else { img.encoding = "32FC1"; img.step = W * 4; }
          img.is_bigendian = 0;
          if (r.body.size() < static_cast<size_t>(img.step) * H) continue;
          img.data.assign(r.body.begin(), r.body.begin() + static_cast<size_t>(img.step) * H);
          S.img->publish(img);
          sensor_msgs::msg::CameraInfo ci;
          ci.header = img.header;
          ci.height = H; ci.width = W;
          ci.distortion_model = "plumb_bob";
          ci.d.assign(5, 0.0);
          for (size_t i = 0; i < 9 && i < K.size(); ++i) ci.k[i] = K[i];
          ci.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
          const double tx = S.st == "right" ? -ci.k[0] * baseline : 0.0;
          ci.p = {ci.k[0], 0.0, ci.k[2], tx, 0.0, ci.k[4], ci.k[5], 0.0, 0.0, 0.0, 1.0, 0.0};
          S.info->publish(ci);
        }
        ++n_media_;
      }
    });
  }

  // ================================================================ 其余 ROS 接口 (核心模式)
  // Nav2 生命周期 (lifecycle_manager is_active，2 s 一次) 与 ROS 图 → Python
  void poll_ros() {
    if (active_cli_->service_is_ready() && !active_pending_) {
      active_pending_ = true;
      active_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
                                      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
                                        active_pending_ = false;
                                        nav2_active_ = f.get()->success;
                                      });
    } else if (!active_cli_->service_is_ready()) {
      nav2_active_ = false;
    }
    std::string j = "{\"k\":\"ros\",\"nav2_active\":" + std::string(nav2_active_ ? "true" : "false") + ",\"nodes\":[";
    try {
      auto nn = get_node_names();                         // 完整名 (含命名空间)
      std::sort(nn.begin(), nn.end());
      nn.erase(std::unique(nn.begin(), nn.end()), nn.end());
      for (size_t i = 0; i < nn.size(); ++i) j += (i ? ",\"" : "\"") + jesc(nn[i]) + "\"";
      j += "],\"topics\":[";
      auto tt = get_topic_names_and_types();
      bool first = true;
      for (const auto &kv : tt) {
        if (kv.first.rfind("/rosout", 0) == 0) continue;
        j += (first ? "\"" : ",\"") + jesc(kv.first) + "\"";
        first = false;
      }
      j += "]";
    } catch (const std::exception &) {
      j += "],\"topics\":[]";
    }
    send_nav(j + "}");
  }

  // /map (slam_toolbox) → 共享内存文件，通知 Python (界面地图/保存 PGM)
  void on_map(const nav_msgs::msg::OccupancyGrid &m) {
    ++map_rev_;
    const std::string path = "/dev/shm/agv_map_" + std::to_string(getpid()) + ".bin";
    const std::string tmp = path + ".tmp";
    int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) return;
    const uint32_t w = m.info.width, h = m.info.height;
    const double res = m.info.resolution, ox = m.info.origin.position.x, oy = m.info.origin.position.y;
    std::vector<uint8_t> b;
    b.reserve(32 + m.data.size());
    put(b, w); put(b, h); put(b, res); put(b, ox); put(b, oy);
    b.insert(b.end(), reinterpret_cast<const uint8_t *>(m.data.data()), reinterpret_cast<const uint8_t *>(m.data.data()) + m.data.size());
    const bool ok = ::write(fd, b.data(), b.size()) == static_cast<ssize_t>(b.size());
    ::close(fd);
    if (!ok || ::rename(tmp.c_str(), path.c_str()) != 0) return;
    send_nav("{\"k\":\"map\",\"path\":\"" + path + "\",\"rev\":" + std::to_string(map_rev_) + "}");
  }

  void on_roscall(const std::string &j) {
    jl::Value v;
    if (!jl::parse(j, v)) return;
    const std::string op = v["op"].str();
    const long id = static_cast<long>(v["id"].num());
    auto reply = [this, id, op](bool ok, const std::string &err) {
      send_nav("{\"k\":\"roscall\",\"id\":" + std::to_string(id) + ",\"op\":\"" + op + "\",\"ok\":" + (ok ? "true" : "false") +
               ",\"err\":\"" + jesc(err) + "\"}");
    };
    if (op == "set_pose") {                 // EKF odom 系对齐到初始位姿 (ros_slam._align_odom)
      geometry_msgs::msg::PoseWithCovarianceStamped m;
      m.header.frame_id = "odom";
      m.header.stamp = get_clock()->now();
      m.pose.pose.position.x = v["x"].num();
      m.pose.pose.position.y = v["y"].num();
      set_quat(m.pose.pose.orientation, v["yaw"].num());
      m.pose.covariance[0] = m.pose.covariance[7] = m.pose.covariance[35] = 1e-6;
      setpose_pub_->publish(m);
      reply(true, "");
    } else if (op == "load_map") {
      if (!loadmap_cli_->service_is_ready()) { reply(false, "/map_server/load_map 未就绪"); return; }
      auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
      req->map_url = v["url"].str();
      loadmap_cli_->async_send_request(req, [reply](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture f) {
        reply(f.get()->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS, "");
      });
    } else if (op == "save_map") {
      if (!savemap_cli_->wait_for_service(std::chrono::seconds(3))) { reply(false, "slam_toolbox 服务 /slam_toolbox/serialize_map 未就绪"); return; }
      auto req = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();
      req->filename = v["file"].str();
      savemap_cli_->async_send_request(req, [reply](rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedFuture) {
        reply(true, "");
      });
    } else {
      reply(false, "未知操作 " + op);
    }
  }

  // ================================================================ 自研导引 (guidance.hpp)
  // 仿真里程计在仿真时刻 t 的位姿 (线性插值；比最新帧新时按速度外推 ≤ 0.1 s)
  bool odom_at(double t, double *o) const {
    if (odom_hist_.empty()) return false;
    const auto &h = odom_hist_;
    if (t <= h.front()[0]) { o[0] = h.front()[1]; o[1] = h.front()[2]; o[2] = h.front()[3]; return true; }
    if (t >= h.back()[0]) {
      const double dt = std::min(t - h.back()[0], 0.1), th = h.back()[3];
      o[0] = h.back()[1] + (v_meas_ * std::cos(th)) * dt;
      o[1] = h.back()[2] + (v_meas_ * std::sin(th)) * dt;
      o[2] = th + w_meas_ * dt;
      return true;
    }
    size_t lo = 0, hi = h.size() - 1;
    while (hi - lo > 1) { size_t mid = (lo + hi) / 2; if (h[mid][0] < t) lo = mid; else hi = mid; }
    const double u = h[hi][0] <= h[lo][0] ? 0.0 : (t - h[lo][0]) / (h[hi][0] - h[lo][0]);
    o[0] = h[lo][1] + u * (h[hi][1] - h[lo][1]);
    o[1] = h[lo][2] + u * (h[hi][2] - h[lo][2]);
    o[2] = h[lo][3] + u * std::atan2(std::sin(h[hi][3] - h[lo][3]), std::cos(h[hi][3] - h[lo][3]));
    return true;
  }
  static void compose(const double *a, const double *b, double *o) {
    const double c = std::cos(a[2]), s = std::sin(a[2]);
    o[0] = a[0] + c * b[0] - s * b[1];
    o[1] = a[1] + s * b[0] + c * b[1];
    o[2] = std::atan2(std::sin(a[2] + b[2]), std::cos(a[2] + b[2]));
  }
  // 导引用 map 位姿: 精定位修正 > slam_toolbox TF 修正 > 执行进程下发的 map→odom (内置 SLAM / 真值)
  guide::Pose guide_pose() {
    std::lock_guard<std::mutex> lk(smu_);
    const double *M = have_corr_ ? corr_ : (loc_ext_ && have_m_tf_ ? m_tf_ : mode_m2o_);
    double o[3];
    compose(M, odom_now_.data(), o);
    return {o[0], o[1], o[2]};
  }

  void on_guide(const std::string &j) {
    jl::Value v;
    if (!jl::parse(j, v)) return;
    guide::Mission m;
    m.mid = static_cast<long>(v["mid"].num());
    for (const auto &w : v["wps"].a) m.wps.emplace_back(w[0].num(), w[1].num());
    for (const auto &l : v["labels"].a) m.labels.push_back(l.str());
    m.labels.resize(m.wps.size());
    for (const auto &c : v["corners"].a) {
      guide::Corner C;
      if (!c.is_null()) {
        C.kind = static_cast<int>(c["kind"].num());
        C.rot = c["rot"].num(); C.R = c["R"].num(); C.d = c["d"].num(); C.turn = c["turn"].num(); C.heading = c["heading"].num();
        C.v = c["v"].num(0.35); C.cx = c["cx"].num(); C.cy = c["cy"].num(); C.clr = c["clr"].num(9.0);
      }
      m.corners.push_back(C);
    }
    m.corners.resize(m.wps.size());
    m.target_yaw = v["yaw"].num();
    m.replan_left = static_cast<int>(v["replan_left"].num(2));
    m.planner = v["planner"].str();
    m.chassis = v["chassis"].str();
    m.corner_mode = v["corner_mode"].str().empty() ? "auto" : v["corner_mode"].str();
    m.max_v = v["max_v"].num(1.2); m.max_w = v["max_w"].num(1.6); m.max_decel = v["max_decel"].num(0.5);
    m.max_ang_decel = v["max_ang_decel"].num(1.0); m.track_L = v["track_L"].num(0.6);
    m.head = v["head"].num(0.6); m.tail = v["tail"].num(0.6); m.hw = v["hw"].num(0.4);
    m.corner_radius = v["corner_radius"].num(0.9); m.body_margin = v["body_margin"].num(0.05);
    m.rotate_margin = v["rotate_margin"].num(0.02); m.arrive_tol = v["arrive_tol"].num(0.25);
    for (const auto &x : v["segs"].a) m.segs.push_back(x.num());
    for (size_t i = 0; i + 3 < v["refine_segs"].a.size(); i += 4) {
      const auto &a = v["refine_segs"].a;
      m.refine_segs.push_back({a[i].num(), a[i + 1].num(), a[i + 2].num(), a[i + 3].num()});
    }
    m.refine = v["refine"].truthy(true);
    m.refine_dist = v["refine_dist"].num(0.8);
    guide_mid_ = m.mid;
    {
      std::lock_guard<std::mutex> lk(smu_);
      have_corr_ = false;                     // 新任务: 精定位修正从头开始
    }
    if (!guide_) guide_ = std::make_unique<guide::Guidance>(make_guide_io());
    guide_->start(std::move(m));
  }

  guide::Io make_guide_io() {
    guide::Io io;
    io.pose = [this] { return guide_pose(); };
    io.v_meas = [this] { std::lock_guard<std::mutex> lk(smu_); return v_meas_; };
    io.w_meas = [this] { std::lock_guard<std::mutex> lk(smu_); return w_meas_; };
    io.steer = [this] { std::lock_guard<std::mutex> lk(smu_); return steer_; };
    io.hold = [this] { std::lock_guard<std::mutex> lk(smu_); return paused_ || estop_; };
    io.allowed = [this](int sign, bool has_left, double left, double *d_hit) {
      std::lock_guard<std::mutex> lk(smu_);
      if (!cfg_.enabled || cfg_.fields.empty()) return 99.0;
      agvsafe::Env e;
      e.bands = bands_;
      double need;
      return agvsafe::allowed_speed(cfg_, e, sign, sign > 0 && has_left, left, d_hit, &need);
    };
    io.photo_block = [this](bool front, bool has_left, double left) {
      std::lock_guard<std::mutex> lk(smu_);
      agvsafe::Env e;
      e.has_left = has_left;
      e.approach_left = left;
      for (const auto &p : cfg_.photos) {
        if (std::find(photo_on_.begin(), photo_on_.end(), p.di) == photo_on_.end()) continue;
        auto it = photo_dist_.find(p.name);
        e.photo_hits.emplace_back(&p, it == photo_dist_.end() ? -1.0 : it->second);
      }
      agvsafe::Result r;
      if (!cfg_.fields.empty()) agvsafe::photo_sides(cfg_, e, agvsafe::field_for_speed(cfg_, v_meas_), false, r);
      return front ? !r.photo_front.empty() : !r.photo_rear.empty();
    };
    io.rotation_blocked = [this](double dir, double rot_left) {
      std::lock_guard<std::mutex> lk(smu_);
      return cfg_.enabled && agvsafe::rotation_blocked(cfg_, pts_, dir, agvsafe::rotate_look(cfg_, w_meas_, rot_left));
    };
    io.scan_pts = [this] {
      std::lock_guard<std::mutex> lk(smu_);
      std::vector<agv::Pt> out;
      out.reserve(pts_.size());
      for (const auto &p : pts_) out.push_back({p.x, p.y});
      return out;
    };
    io.refine_scan = [this](std::vector<agv::Pt> &out, double t_stop) {
      for (int i = 0; i < 40; ++i) {                  // 最多等 2 s
        {
          std::lock_guard<std::mutex> lk(smu_);
          if (refine_t_ > t_stop + 0.1 && !refine_pts_.empty()) { out = refine_pts_; return true; }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      return false;
    };
    io.cmd = [this](double vx, double vy, double wz, bool in_arc, bool has_left, double left, double rot_left) {
      filter_and_send(vx, vy, wz, in_arc, has_left, left, false, "nav:guide", rot_left);
    };
    io.set_correction = [this](const guide::Pose &p) {       // corr = 精定位位姿 ∘ inv(当前里程计)
      std::lock_guard<std::mutex> lk(smu_);
      const double th = p.th - odom_now_[2], c = std::cos(th), s2 = std::sin(th);
      corr_[0] = p.x - (c * odom_now_[0] - s2 * odom_now_[1]);
      corr_[1] = p.y - (s2 * odom_now_[0] + c * odom_now_[1]);
      corr_[2] = th;
      have_corr_ = true;
    };
    io.event = [this](const std::string &type, const std::string &level, const std::string &title, const std::string &msg) {
      send_event("navigation", type, level, title, msg);
    };
    io.status = [this](const std::string &st, int idx, double rem) {
      const double t = wall_now();
      if (st == guide_st_ && idx == guide_idx_ && t - guide_st_t_ < 0.2) return;
      guide_st_ = st;
      guide_idx_ = idx;
      guide_st_t_ = t;
      char b[160];
      std::snprintf(b, sizeof(b), "{\"k\":\"guide\",\"mid\":%ld,\"st\":\"%s\",\"idx\":%d,\"rem\":%.3f}", guide_mid_, st.c_str(), idx, rem);
      send_nav(b);
    };
    io.done = [this](bool, const std::string &result) {
      const auto p = guide_pose();
      char b[200];
      std::snprintf(b, sizeof(b), "{\"k\":\"guide_done\",\"mid\":%ld,\"result\":\"%s\",\"x\":%.4f,\"y\":%.4f,\"yaw\":%.5f}",
                    guide_mid_, result.c_str(), p.x, p.y, p.th);
      send_nav(b);
      guide_st_.clear();
    };
    return io;
  }

  static double wall_now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  // ---------------------------------------------------------------- 路线导航
  void send_nav(const std::string &json) {
    std::vector<uint8_t> b(MAGIC, MAGIC + 4);
    b.push_back(T_NAV);
    b.insert(b.end(), json.begin(), json.end());
    send(b);
  }

  void on_route(const std::string &j) {
    using NTP = nav2_msgs::action::NavigateToPose;
    const long mid = static_cast<long>(json_nums(j, "mid").empty() ? 0 : json_nums(j, "mid")[0]);
    auto route = json_nums(j, "route"), goal = json_nums(j, "goal"), segs = json_nums(j, "segs");
    const std::string bt = json_str(j, "bt");
    auto fail = [&](const std::string &why) {
      send_nav("{\"k\":\"result\",\"mid\":" + std::to_string(mid) + ",\"result\":\"REJECTED\",\"why\":\"" + jesc(why) + "\"}");
    };
    if (goal.size() < 3 || route.size() < 4) { fail("路线或终点为空"); return; }
    if (!segs.empty()) {
      std_msgs::msg::Float32MultiArray m;
      m.data.assign(segs.begin(), segs.end());
      segs_pub_->publish(m);
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = get_clock()->now();
    for (size_t i = 0; i + 1 < route.size(); i += 2) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = route[i];
      ps.pose.position.y = route[i + 1];
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    route_pub_->publish(path);
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) { fail("Nav2 navigate_to_pose 服务未就绪"); return; }
    NTP::Goal g;
    g.pose.header = path.header;
    g.pose.pose.position.x = goal[0];
    g.pose.pose.position.y = goal[1];
    set_quat(g.pose.pose.orientation, goal[2]);
    g.behavior_tree = bt;
    rclcpp_action::Client<NTP>::SendGoalOptions opt;
    opt.goal_response_callback = [this, mid](const rclcpp_action::ClientGoalHandle<NTP>::SharedPtr &gh) {
      if (!gh) {
        send_nav("{\"k\":\"result\",\"mid\":" + std::to_string(mid) + ",\"result\":\"REJECTED\"}");
        return;
      }
      std::lock_guard<std::mutex> lk(nmu_);
      goal_ = gh;
    };
    opt.feedback_callback = [this, mid](rclcpp_action::ClientGoalHandle<NTP>::SharedPtr,
                                        const std::shared_ptr<const NTP::Feedback> fb) {
      double t = now_s();
      if (t - last_fb_tx_ < 0.2) return;                      // 限 5 Hz
      last_fb_tx_ = t;
      char b[200];
      std::snprintf(b, sizeof(b), "{\"k\":\"fb\",\"mid\":%ld,\"dist\":%.3f,\"t\":%d,\"rec\":%d}", mid,
                    fb->distance_remaining, fb->navigation_time.sec, fb->number_of_recoveries);
      send_nav(b);
    };
    opt.result_callback = [this, mid](const rclcpp_action::ClientGoalHandle<NTP>::WrappedResult &r) {
      const char *txt = r.code == rclcpp_action::ResultCode::SUCCEEDED ? "SUCCEEDED"
                        : r.code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" : "ABORTED";
      send_nav("{\"k\":\"result\",\"mid\":" + std::to_string(mid) + ",\"result\":\"" + txt + "\"}");
    };
    nav_client_->async_send_goal(g, opt);
  }

  void cancel_route() {
    std::lock_guard<std::mutex> lk(nmu_);
    if (goal_) {
      nav_client_->async_cancel_goal(goal_);
      goal_.reset();
    }
  }

  // ---------------------------------------------------------------- TF → Python
  void send(const std::vector<uint8_t> &b) {
    sendto(out_fd_, b.data(), b.size(), MSG_DONTWAIT, reinterpret_cast<const sockaddr *>(&out_addr_), out_len_);
  }

  void poll_tf() {
    double mb[3] = {0, 0, 0}, ob[3] = {0, 0, 0}, stamp = 0.0;
    uint8_t flags = 0;
    try {
      auto tr = buf_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      stamp = to_sec(tr.header.stamp);
      mb[0] = tr.transform.translation.x;
      mb[1] = tr.transform.translation.y;
      mb[2] = yaw_of(tr.transform.rotation);
      if (stamp > last_tf_) flags |= 1;
    } catch (const tf2::TransformException &) {
    }
    try {
      auto tr = buf_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
      ob[0] = tr.transform.translation.x;
      ob[1] = tr.transform.translation.y;
      ob[2] = yaw_of(tr.transform.rotation);
      flags |= 2;
    } catch (const tf2::TransformException &) {
    }
    double off;
    {
      std::lock_guard<std::mutex> lk(tmu_);
      if (!has_off_) return;
      off = off_;
    }
    if (!(flags & 1) && ++idle_ % 25 != 0) return;   // 没有新定位时 2 Hz 回传 odom 位姿 (EKF 对齐用)
    if (flags & 1) {
      last_tf_ = stamp;
      std::lock_guard<std::mutex> lk(smu_);
      last_tf_wall_ = wall_now();
      // 定位修正 M = TF(map→base) ∘ inv(该时刻的仿真里程计) (与 nav_runtime/slam.py set_external 同一算法)
      double od[3];
      if (odom_at(stamp - off, od)) {
        const double th = mb[2] - od[2], c = std::cos(th), s2 = std::sin(th);
        m_tf_[0] = mb[0] - (c * od[0] - s2 * od[1]);
        m_tf_[1] = mb[1] - (s2 * od[0] + c * od[1]);
        m_tf_[2] = th;
        have_m_tf_ = true;
      }
    }
    std::vector<uint8_t> b(MAGIC, MAGIC + 4);
    b.push_back(T_TF);
    put(b, stamp);
    put(b, off);
    for (double v : mb) put(b, v);
    for (double v : ob) put(b, v);
    put(b, flags);
    send(b);
    if (flags & 1) ++n_tf_;
  }

  void send_stats() {
    std::vector<uint8_t> b(MAGIC, MAGIC + 4);
    b.push_back(T_STATS);
    put<uint32_t>(b, n_state_);
    put<uint32_t>(b, n_scan_);
    put<uint32_t>(b, n_merged_);
    put<uint32_t>(b, n_tf_);
    send(b);
  }

  std::string in_path_, out_path_;
  int in_fd_ = -1, out_fd_ = -1;
  sockaddr_un out_addr_{};
  socklen_t out_len_ = sizeof(sockaddr_un);
  std::atomic<bool> stop_{false};
  std::thread rx_;
  bool clamp_ = true;
  double margin_env_ = NAN;
  rclcpp::QoS sensor_qos_{5};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_, gt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> lidar_pubs_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  std::unique_ptr<tf2_ros::Buffer> buf_;
  std::unique_ptr<tf2_ros::TransformListener> tfl_;
  rclcpp::TimerBase::SharedPtr tf_timer_, stats_timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr segs_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  std::mutex nmu_;
  double last_fb_tx_ = 0.0, last_stop_tx_ = 0.0;
  // ---- 核心模式
  bool core_ = false;
  std::string sim_url_;
  std::unique_ptr<simstream::Stream> stream_;
  std::atomic<bool> stream_up_{false};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  std::mutex smu_, cmu_;
  std::vector<std::string> joint_names_;
  agvsafe::Config cfg_;
  bool have_cfg_ = false, nav2_fwd_ = false, mode_own_odom_ = false, mode_map_odom_ = false, loc_ext_ = false;
  std::atomic<bool> want_raw_lidar_{false};
  double mode_m2o_[3] = {0, 0, 0};
  double speed_cap_ = 0.0, v_meas_ = 0.0, stop_dist_ = 0.0, stop_wall_ = -1e9, last_tf_wall_ = 0.0;
  bool estop_ = false, loc_stale_ = false;
  std::vector<std::string> photo_on_, photo_ignored_;
  std::map<std::string, double> photo_dist_;
  std::vector<std::pair<double, double>> bands_;
  std::vector<agvsafe::Pt> pts_;
  std::string zone_ = "clear", layer_now_;
  double t_warn_ = 0.0, t_zone_ = 0.0;
  int udp_fd_ = -1, udp_port_ = 0;
  sockaddr_in udp_addr_{};
  uint32_t cmd_seq_ = 0;
  std::atomic<uint32_t> n_stream_state_{0}, n_cmd_{0}, n_cmd_err_{0};
  // ---- 3D 激光 / 相机
  std::mutex media_mu_;
  bool have_media_log_ = false;
  std::set<std::string> media_names_;
  std::vector<std::thread> media_threads_;
  std::atomic<uint32_t> n_media_{0};
  // ---- 其余 ROS 接口
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr active_cli_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr loadmap_cli_;
  rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedPtr savemap_cli_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr setpose_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr ros_timer_;
  std::atomic<bool> nav2_active_{false}, active_pending_{false};
  uint32_t map_rev_ = 0;
  // ---- 自研导引
  std::unique_ptr<guide::Guidance> guide_;
  long guide_mid_ = 0;
  std::string guide_st_;
  int guide_idx_ = 0;
  double guide_st_t_ = 0;
  double w_meas_ = 0.0;
  bool paused_ = false;
  std::array<double, 3> odom_now_{{0, 0, 0}};
  std::deque<std::array<double, 4>> odom_hist_;
  std::vector<double> steer_;
  double m_tf_[3] = {0, 0, 0}, corr_[3] = {0, 0, 0};
  bool have_m_tf_ = false, have_corr_ = false;
  std::string refine_lidar_;
  double refine_mount_[4] = {0, 0, 0, 0};
  std::vector<agv::Pt> refine_pts_;
  double refine_t_ = 0.0;
  std::mutex tmu_;
  bool has_off_ = false, own_odom_ = false, has_odom_stamp_ = false;
  double off_ = 0.0, odom_stamp_ = 0.0, last_tf_ = 0.0;
  uint32_t idle_ = 0;
  std::atomic<uint32_t> n_state_{0}, n_scan_{0}, n_merged_{0}, n_tf_{0};
};

int main(int argc, char **argv) {
#ifdef __linux__
  // 执行进程 (父进程) 被强杀时一起退出: 否则孤儿桥接仍在同一 ROS_DOMAIN 发布 TF/激光、向仿真发指令
  const pid_t parent = getppid();
  prctl(PR_SET_PDEATHSIG, SIGTERM);
  if (getppid() != parent) return 0;
#endif
  rclcpp::init(argc, argv);
  auto args = rclcpp::remove_ros_arguments(argc, argv);
  std::string in_path, out_path;
  bool core = false;
  for (size_t i = 1; i < args.size(); i++) {
    if (args[i] == "--core") core = true;
    else if (i + 1 < args.size() && args[i] == "--in") in_path = args[++i];
    else if (i + 1 < args.size() && args[i] == "--out") out_path = args[++i];
  }
  if (in_path.empty() || out_path.empty()) {
    std::fprintf(stderr, "usage: agv_ros_bridge --in <sock> --out <sock>\n");
    return 2;
  }
  auto node = std::make_shared<AgvRosBridge>(in_path, out_path, core);
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
