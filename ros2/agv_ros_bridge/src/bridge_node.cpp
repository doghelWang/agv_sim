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
// 用法: agv_ros_bridge --in <本节点接收的 socket 路径> --out <Python 接收的 socket 路径> [--ros-args ...]
// ============================================================================
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace {

constexpr char MAGIC[4] = {'A', 'G', 'V', '1'};
enum : uint8_t { T_STATE = 1, T_SCAN = 2, T_ROUTE = 3, T_CANCEL = 4, T_TF = 10, T_STATS = 11, T_NAV = 12 };
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
  AgvRosBridge(const std::string &in_path, const std::string &out_path)
      : Node("nav_runtime_bridge_cpp"), in_path_(in_path), out_path_(out_path) {
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
    RCLCPP_INFO(get_logger(), "agv_ros_bridge (C++) 就绪: in=%s out=%s clamp=%d", in_path_.c_str(), out_path_.c_str(), clamp_);
  }

  ~AgvRosBridge() override {
    stop_ = true;
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
    std::memcpy(s.ranges.data(), r.p, 4ull * n);
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
    if (flags & 1) last_tf_ = stamp;
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
  std::mutex tmu_;
  bool has_off_ = false, own_odom_ = false, has_odom_stamp_ = false;
  double off_ = 0.0, odom_stamp_ = 0.0, last_tf_ = 0.0;
  uint32_t idle_ = 0;
  std::atomic<uint32_t> n_state_{0}, n_scan_{0}, n_merged_{0}, n_tf_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto args = rclcpp::remove_ros_arguments(argc, argv);
  std::string in_path, out_path;
  for (size_t i = 1; i + 1 < args.size(); i++) {
    if (args[i] == "--in") in_path = args[++i];
    else if (args[i] == "--out") out_path = args[++i];
  }
  if (in_path.empty() || out_path.empty()) {
    std::fprintf(stderr, "usage: agv_ros_bridge --in <sock> --out <sock>\n");
    return 2;
  }
  auto node = std::make_shared<AgvRosBridge>(in_path, out_path);
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
