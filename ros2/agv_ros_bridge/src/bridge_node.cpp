// ============================================================================
// agv_ros_bridge —— 执行进程 (nav_runtime) 的 ROS 2 发布端，C++ 实现 (NAV_ROS_BRIDGE=cpp)
//
//   Python (nav_runtime/cpp_bridge.py)  ──Unix 数据报──▶  本节点
//     STATE  真值/里程计/IMU/关节/map→odom      →  /odom  /ground_truth/odom  /imu  /joint_states
//                                                   TF odom→base_footprint (own_odom)、map→odom (内置定位)
//     SCAN   各 2D 激光 / 融合扫描               →  /scan/<name> (代价地图时间戳钳位)  /scan
//   本节点  ──Unix 数据报──▶  Python
//     TF     50 Hz 查询 map→base_footprint、odom→base_footprint (slam_toolbox / EKF 输出) + 墙钟-仿真时间偏移
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
enum : uint8_t { T_STATE = 1, T_SCAN = 2, T_TF = 10, T_STATS = 11 };
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

double env_d(const char *k, double dflt) {
  const char *v = std::getenv(k);
  return (v && *v) ? std::atof(v) : dflt;
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
    unlink(in_path_.c_str());
    sockaddr_un a{};
    a.sun_family = AF_UNIX;
    std::strncpy(a.sun_path, in_path_.c_str(), sizeof(a.sun_path) - 1);
    if (bind(in_fd_, reinterpret_cast<sockaddr *>(&a), sizeof(a)) != 0) {
      RCLCPP_FATAL(get_logger(), "bind %s 失败: %s", in_path_.c_str(), std::strerror(errno));
      throw std::runtime_error("bind");
    }
    int rcv = 1 << 20;
    setsockopt(in_fd_, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof(rcv));
    timeval tv{0, 200000};
    setsockopt(in_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    std::memset(&out_addr_, 0, sizeof(out_addr_));
    out_addr_.sun_family = AF_UNIX;
    std::strncpy(out_addr_.sun_path, out_path_.c_str(), sizeof(out_addr_.sun_path) - 1);

    rx_ = std::thread([this] { rx_loop(); });
    tf_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { poll_tf(); });
    stats_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { send_stats(); });
    RCLCPP_INFO(get_logger(), "agv_ros_bridge (C++) 就绪: in=%s out=%s clamp=%d", in_path_.c_str(), out_path_.c_str(), clamp_);
  }

  ~AgvRosBridge() override {
    stop_ = true;
    if (rx_.joinable()) rx_.join();
    close(in_fd_);
    close(out_fd_);
    unlink(in_path_.c_str());
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

  // ---------------------------------------------------------------- TF → Python
  void send(const std::vector<uint8_t> &b) {
    sendto(out_fd_, b.data(), b.size(), MSG_DONTWAIT, reinterpret_cast<const sockaddr *>(&out_addr_), sizeof(out_addr_));
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
