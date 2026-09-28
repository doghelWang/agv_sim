// ============================================================================
// RouteController (nav2_core::Controller) —— 路线跟随 + 停车精度控制
//
//   路径 (RoutePlanner 生成) 按尖点切成若干段，逐段执行:
//     ALIGN       段起点原地转向到段方向 (拐点停车转向 / 起步对准)；拐点精度不要求高，±rotate_tol 即可
//     TRACK       段内交给 primary_controller (Regulated Pure Pursuit) 跟线，圆弧过弯不停车
//     CUSP_STOP   到达中间尖点 (±cusp_tol) 停稳 → 下一段 ALIGN
//     LOCALIZE    最后一段剩余 < final_dist: 停车取一帧激光，与场景静态几何 (/agv/world_segments) 做 ICP 精定位
//                 (localize.hpp)，得到场景系车体位姿 → 终点换算到 odom 系冻结；配准失败时退回 map→odom 最近
//                 tf_avg_s 秒平均。slam_toolbox 建图模式的 map 系在贴墙处有 20~30 mm 缓慢偏置，停车误差主要来自它
//     FINAL       按冻结的 odom 终点沿进站直线纯跟踪 (预瞄点在直线上，横向偏差在 final_dist 内收敛) +
//                 剩余行程减速曲线进站，到停止点 (±final_stop_tol) 停车
//     STOPPING    到达停止点 (±final_stop_tol) 停稳
//     GOAL_ALIGN  原地对位到终点朝向 (±yaw_tol)
//     VERIFY      停稳后再做一次精定位: 朝向差 > heading_fix_tol 时按实测差值再转一次 (最多 2 次，消除末段里程计
//                 航向漂移)；位置/朝向残差作为事件发布 (ARRIVE_CHECK)
//     DONE        到位 (SharedState::done → AgvGoalChecker)
//   原地转向前后: 单舵轮车型先用微小指令把舵轮转到位 (steer_settle_s)，避免舵角过渡带偏车体
//   原地转向防护: 实测激光点 (机体系) 对车体外扩 rotate_margin 的扫掠检查；最短方向受阻试反方向；
//     都受阻等 block_wait_s 仍不行 → 发布 /agv/turn_request 并抛异常 → 行为树恢复 (adjust_pose: 摆头 + 后退 + 转向)
// ============================================================================
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "agv_nav2_plugins/common.hpp"
#include "agv_nav2_plugins/geom.hpp"
#include "agv_nav2_plugins/localize.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace agv_nav2_plugins
{

using agv::wrap;

class RouteController : public nav2_core::Controller
{
  enum class Phase { ALIGN, TRACK, CUSP_STOP, LOCALIZE, FINAL, STOPPING, GOAL_ALIGN, VERIFY, DONE };

public:
  RouteController()
  : loader_("nav2_core", "nav2_core::Controller") {}

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    node_ = parent;
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    const std::string p = name + ".";
    primary_name_ = param<std::string>(
      node, p + "primary_controller", "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController");
    body_ = rectParam(node, name);
    rot_w_ = param<double>(node, p + "rotate_max_w", 0.6);
    rot_acc_ = param<double>(node, p + "rotate_accel", 0.6);
    rot_margin_ = param<double>(node, p + "rotate_margin", 0.02);
    rot_tol_ = param<double>(node, p + "rotate_tol", 0.03);
    yaw_tol_ = param<double>(node, p + "yaw_tol", 0.005);
    cusp_tol_ = param<double>(node, p + "cusp_tol", 0.04);
    final_dist_ = param<double>(node, p + "final_dist", 0.30);
    final_stop_tol_ = param<double>(node, p + "final_stop_tol", 0.002);
    final_v_ = param<double>(node, p + "final_max_v", 0.12);
    final_min_v_ = param<double>(node, p + "final_min_v", 0.008);
    final_dec_ = param<double>(node, p + "final_decel", 0.2);
    final_look_ = param<double>(node, p + "final_lookahead", 0.25);
    latency_ = param<double>(node, p + "latency_s", 0.10);
    tf_avg_s_ = param<double>(node, p + "tf_avg_s", 1.0);
    settle_s_ = param<double>(node, p + "steer_settle_s", 0.0);
    block_wait_s_ = param<double>(node, p + "block_wait_s", 2.0);
    scan_max_age_ = param<double>(node, p + "scan_max_age_s", 0.5);
    refine_ = param<bool>(node, p + "refine_localization", true);
    half_thick_ = param<double>(node, p + "wall_half_thickness", 0.025);
    refine_max_corr_ = param<double>(node, p + "refine_max_correction", 0.15);
    refine_settle_s_ = param<double>(node, p + "refine_settle_s", 0.3);
    heading_fix_tol_ = param<double>(node, p + "heading_fix_tol", 0.0026);
    // 精定位用单个 2D 激光的原始帧 (融合扫描按角度分箱取最近点，距离系统性偏短 1~4 cm，不能用于配准)
    refine_topic_ = param<std::string>(node, p + "refine_scan_topic", "");

    primary_ = loader_.createUniqueInstance(primary_name_);
    primary_->configure(parent, name, tf, costmap_ros);   // RPP 参数与本插件同一命名空间

    scan_.init(node, tf, costmap_ros->getBaseFrameID());
    if (!refine_topic_.empty()) {refine_scan_.init(node, tf, costmap_ros->getBaseFrameID(), refine_topic_);}
    events_.init(node);
    stop_pub_ = rclcpp::create_publisher<std_msgs::msg::Float32>(*node, "/agv/stop_distance", rclcpp::QoS(5));
    turn_pub_ = rclcpp::create_publisher<geometry_msgs::msg::PoseStamped>(
      *node, "/agv/turn_request", rclcpp::QoS(1).transient_local().reliable());
    seg_sub_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/agv/world_segments", rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m) {
        std::vector<agv::Seg> v;
        for (size_t i = 0; i + 3 < m->data.size(); i += 4) {v.push_back({m->data[i], m->data[i + 1], m->data[i + 2], m->data[i + 3]});}
        std::lock_guard<std::mutex> lk(seg_mu_);
        segs_.swap(v);
      });
    RCLCPP_INFO(logger_, "[%s] 路线控制器: 主控制器 %s，车体 +%.2f/-%.2f × +%.2f/-%.2f，转向余量 %.3f",
      name_.c_str(), primary_name_.c_str(), body_.head, body_.tail, body_.left, body_.right, rot_margin_);
  }

  void cleanup() override {primary_->cleanup(); primary_.reset();}
  void activate() override {primary_->activate();}
  void deactivate() override {primary_->deactivate();}
  void setSpeedLimit(const double & limit, const bool & pct) override {primary_->setSpeedLimit(limit, pct);}

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    map_frame_ = path.header.frame_id.empty() ? "map" : path.header.frame_id;
    std::vector<agv::Pose2> v;
    for (const auto & ps : path.poses) {
      v.push_back({ps.pose.position.x, ps.pose.position.y, tf2::getYaw(ps.pose.orientation)});
    }
    pieces_ = agv::splitCusps(v);
    // 进站段: 最后一个 ≥ 2 个点的段 (其后若还有单点段，那是终点对位转向的尖点)
    kf_ = pieces_.size() - 1;
    while (kf_ > 0 && pieces_[kf_].size() < 2) {--kf_;}
    // 进站段末尾的直线长度: 精定位停车点不早于直线起点 (否则纯跟踪进站直线会切掉前面的圆弧)
    final_trigger_ = final_dist_;
    {
      const auto & pc = pieces_[kf_];
      double straight = 0.0;
      if (pc.size() >= 2) {
        const double h = std::atan2(pc.back().y - pc[pc.size() - 2].y, pc.back().x - pc[pc.size() - 2].x);
        for (size_t i = pc.size() - 1; i >= 1; --i) {
          const double hi = std::atan2(pc[i].y - pc[i - 1].y, pc[i].x - pc[i - 1].x);
          if (std::fabs(wrap(hi - h)) > 0.02) {break;}
          straight += std::hypot(pc[i].x - pc[i - 1].x, pc[i].y - pc[i - 1].y);
        }
      }
      final_trigger_ = std::min(final_dist_, std::max(0.3, straight));
    }
    header_ = path.header;
    k_ = 0;
    hint_ = 0;
    block_since_ = -1.0;
    rot_dir_ = 0;
    mode_ = 0;
    SharedState::get().done = false;
    block_reported_ = false;
    verify_n_ = 0;
    setPhase(Phase::ALIGN);
    if (!pieces_.empty() && !pieces_[0].empty()) {sendPiece();}
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & vel,
    nav2_core::GoalChecker * gc) override
  {
    geometry_msgs::msg::TwistStamped out;
    out.header = pose.header;
    const double now = clock_->now().seconds();
    sampleMapToOdom(now);
    const double x = pose.pose.position.x, y = pose.pose.position.y, yaw = tf2::getYaw(pose.pose.orientation);
    const double v_meas = vel.linear.x, w_meas = vel.angular.z;
    rx_ = x;
    ry_ = y;
    rx_yaw_ = yaw;
    if (pieces_.empty() || pieces_[0].empty()) {
      throw nav2_core::PlannerException("路线为空");
    }
    const bool last = k_ >= kf_;
    const auto & piece = pieces_[k_];

    switch (phase_) {
      case Phase::ALIGN: {
          const double target = toOdomYaw(agv::pieceHeading(piece));
          double done_e = 0.0;
          if (rotate(target, rot_tol_, yaw, w_meas, now, out, &done_e)) {
            if (piece.size() < 2) {         // 单点段 (终点只差朝向): 直接转入到位流程
              freezeGoal(averagedMapToOdom());
              setPhase(Phase::STOPPING);
            } else {
              setPhase(Phase::TRACK);
              sendPiece();
            }
          }
          publishStop(0.0);
          break;
        }
      case Phase::TRACK: {
          double ox, oy;
          toOdomXY(piece.back().x, piece.back().y, &ox, &oy);
          const auto odom_piece = toOdom(piece);
          const double rem = agv::remaining(odom_piece, x, y, &hint_);
          publishStop(rem);
          if (last && rem < final_trigger_) {
            if (refine_ && haveSegs()) {
              setPhase(Phase::LOCALIZE);
              break;
            }
            freezeGoal(averagedMapToOdom());
            setPhase(Phase::FINAL);
            return computeVelocityCommands(pose, vel, gc);
          }
          if (!last && (rem < cusp_tol_ || std::hypot(ox - x, oy - y) < cusp_tol_)) {
            setPhase(Phase::CUSP_STOP);
            break;
          }
          if (driveSettle(now, out)) {break;}
          out = primary_->computeVelocityCommands(pose, vel, gc);
          mode_ = 1;
          break;
        }
      case Phase::CUSP_STOP: {
          publishStop(0.0);
          if (std::fabs(v_meas) < 0.01 && std::fabs(w_meas) < 0.02) {
            ++k_;
            hint_ = 0;
            setPhase(Phase::ALIGN);
          }
          break;
        }
      case Phase::LOCALIZE: {
          // 停稳后取停车之后到达的激光帧做精定位
          publishStop(final_trigger_);
          if (std::fabs(v_meas) > 0.003 || std::fabs(w_meas) > 0.01) {still_since_ = -1.0; break;}
          if (still_since_ < 0) {still_since_ = now;}
          double age = 1e9;
          const auto pts = refineScan().points(&age);
          if (now - still_since_ < refine_settle_s_ || age > now - still_since_) {
            if (now - phase_t_ > 5.0) {           // 等不到新激光: 退回 map→odom 平均
              freezeGoal(averagedMapToOdom());
              setPhase(Phase::FINAL);
            }
            break;
          }
          agv::Pose2 T = averagedMapToOdom();
          const agv::Pose2 est = odomToMap(cur_, x, y, yaw);
          const auto r = runIcp(pts, est);
          const double corr = std::hypot(r.pose.x - est.x, r.pose.y - est.y);
          char buf[256];
          if (r.ok && corr < refine_max_corr_) {
            // 精定位结果 → map→odom: 使 T(icp 位姿) = 当前 odom 位姿
            const double th = wrap(yaw - r.pose.th), c = std::cos(th), sn = std::sin(th);
            T = {x - (c * r.pose.x - sn * r.pose.y), y - (sn * r.pose.x + c * r.pose.y), th};
            snprintf(buf, sizeof(buf), "停车精定位: 修正 slam 定位 %.1f mm / %.2f°，内点 %d，残差 %.1f mm，约束 %.2f",
              1000.0 * corr, wrap(r.pose.th - est.th) * 180.0 / M_PI, r.inliers, 1000.0 * r.rms, r.min_eig);
            events_.emit("LOC_REFINE", "info", "末段进站前精定位", buf);
          } else {
            snprintf(buf, sizeof(buf), "未采用 (%s，修正量 %.1f mm)，按 slam map→odom 平均进站", r.ok ? "修正量过大" : r.why,
              1000.0 * corr);
            events_.emit("LOC_REFINE", "warning", "末段精定位未采用", buf);
          }
          freezeGoal(T);
          setPhase(Phase::FINAL);
          break;
        }
      case Phase::FINAL: {
          // 终点在 odom 系冻结: e_along 为沿进站方向的剩余，e_lat 左正，e_th 相对进站方向
          const double c = std::cos(appr_), s = std::sin(appr_);
          const double dx = gx_ - x, dy = gy_ - y;
          const double e_along = dx * c + dy * s;
          publishStop(std::max(0.0, e_along));
          if (e_along <= final_stop_tol_) {
            setPhase(Phase::STOPPING);
            break;
          }
          if (driveSettle(now, out)) {break;}
          const double e_pred = e_along - std::max(0.0, v_meas) * latency_;
          double v = std::min({final_v_, std::sqrt(2.0 * final_dec_ * std::max(0.0, e_pred)), 1.5 * std::max(0.0, e_pred)});
          v = std::max(v, final_min_v_);
          // 纯跟踪: 预瞄点 = 进站直线上、车辆投影点前方 final_lookahead 处 (越过终点时沿直线延长)
          const double px = gx_ - e_along * c, py = gy_ - e_along * s;          // 车辆在进站直线上的投影
          const double cx = px + final_look_ * c, cy = py + final_look_ * s;
          const double bx = std::cos(yaw) * (cx - x) + std::sin(yaw) * (cy - y);
          const double by = -std::sin(yaw) * (cx - x) + std::cos(yaw) * (cy - y);
          const double L2 = std::max(bx * bx + by * by, 1e-4);
          double w = v * 2.0 * by / L2;
          w = std::clamp(w, -0.4, 0.4);
          out.twist.linear.x = v;
          out.twist.angular.z = w;
          mode_ = 1;
          break;
        }
      case Phase::STOPPING: {
          publishStop(0.0);
          if (std::fabs(v_meas) < 0.003 && std::fabs(w_meas) < 0.01) {
            if (still_since_ < 0) {still_since_ = now;}
            if (now - still_since_ > 0.3) {
              const double e = wrap(gyaw_ - yaw);
              RCLCPP_INFO(logger_, "[%s] 停止点到位: 距终点 %.1f mm，朝向差 %.2f°", name_.c_str(),
                1000.0 * std::hypot(gx_ - x, gy_ - y), e * 180.0 / M_PI);
              setPhase(Phase::GOAL_ALIGN);
            }
          } else {
            still_since_ = -1.0;
          }
          break;
        }
      case Phase::GOAL_ALIGN: {
          publishStop(0.0);
          if (rotate(gyaw_, yaw_tol_, yaw, w_meas, now, out)) {
            if (refine_ && haveSegs()) {
              setPhase(Phase::VERIFY);
            } else {
              arrived(x, y, yaw, "");
            }
          }
          break;
        }
      case Phase::VERIFY: {
          publishStop(0.0);
          if (std::fabs(v_meas) > 0.003 || std::fabs(w_meas) > 0.01) {still_since_ = -1.0; break;}
          if (still_since_ < 0) {still_since_ = now;}
          double age = 1e9;
          const auto pts = refineScan().points(&age);
          if (now - still_since_ < refine_settle_s_ || age > now - still_since_) {
            if (now - phase_t_ > 5.0) {arrived(x, y, yaw, "；精定位核对: 等不到新激光帧");}
            break;
          }
          const auto r = runIcp(pts, odomToMap(frozen_, x, y, yaw));
          const auto & g = pieces_.back().back();
          char buf[256];
          if (!r.ok) {
            snprintf(buf, sizeof(buf), "；精定位核对未通过 (%s)", r.why);
            arrived(x, y, yaw, buf);
            break;
          }
          const double eh = wrap(g.th - r.pose.th);
          const double ep = std::hypot(r.pose.x - g.x, r.pose.y - g.y);
          if (std::fabs(eh) > heading_fix_tol_ && verify_n_ < 2) {   // 按实测朝向差再对位一次
            ++verify_n_;
            gyaw_ = wrap(yaw + eh);
            setPhase(Phase::GOAL_ALIGN);
            break;
          }
          snprintf(buf, sizeof(buf), "；精定位核对: 距终点 %.1f mm，朝向差 %.2f° (朝向复核修正 %d 次；内点 %d，残差 %.1f mm，约束 %.2f)",
            1000.0 * ep, -eh * 180.0 / M_PI, verify_n_, r.inliers, 1000.0 * r.rms, r.min_eig);
          arrived(x, y, yaw, buf);
          break;
        }
      case Phase::DONE:
        publishStop(0.0);
        break;
    }
    return out;
  }

private:
  void arrived(double x, double y, double yaw, const std::string & extra)
  {
    char buf[160];
    snprintf(buf, sizeof(buf), "控制残差 (odom 冻结终点) %.1f mm / %.2f°", 1000.0 * std::hypot(gx_ - x, gy_ - y),
      wrap(gyaw_ - yaw) * 180.0 / M_PI);
    events_.emit("ARRIVE_CHECK", "info", "到位", std::string(buf) + extra);
    setPhase(Phase::DONE);
    SharedState::get().done = true;
  }

  void setPhase(Phase p)
  {
    phase_ = p;
    phase_t_ = clock_->now().seconds();
    still_since_ = -1.0;
    rot_dir_ = 0;
    block_since_ = -1.0;
  }

  void sendPiece()
  {
    nav_msgs::msg::Path p;
    p.header = header_;
    p.header.stamp = clock_->now();
    for (const auto & q : pieces_[k_]) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = p.header;
      ps.pose.position.x = q.x;
      ps.pose.position.y = q.y;
      tf2::Quaternion qq;
      qq.setRPY(0, 0, q.th);
      ps.pose.orientation.x = qq.x();
      ps.pose.orientation.y = qq.y();
      ps.pose.orientation.z = qq.z();
      ps.pose.orientation.w = qq.w();
      p.poses.push_back(ps);
    }
    primary_->setPlan(p);
  }

  // ---- map → odom
  bool lookupMapToOdom(double * tx, double * ty, double * tth)
  {
    const std::string odom = costmap_ros_->getGlobalFrameID();
    if (odom == map_frame_) {*tx = *ty = *tth = 0.0; return true;}
    try {
      auto t = tf_->lookupTransform(odom, map_frame_, tf2::TimePointZero);   // map 中的点 → odom
      *tx = t.transform.translation.x;
      *ty = t.transform.translation.y;
      *tth = tf2::getYaw(t.transform.rotation);
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }
  void sampleMapToOdom(double now)
  {
    double tx, ty, tth;
    if (!lookupMapToOdom(&tx, &ty, &tth)) {return;}
    cur_ = {tx, ty, tth};
    if (phase_ == Phase::FINAL || phase_ == Phase::STOPPING || phase_ == Phase::GOAL_ALIGN || phase_ == Phase::VERIFY ||
      phase_ == Phase::DONE) {
      return;                                    // 末段: 冻结，不再采样
    }
    m2o_.push_back({now, tx, ty, tth});
    while (!m2o_.empty() && now - m2o_.front()[0] > std::max(tf_avg_s_, 0.05)) {m2o_.pop_front();}
  }
  void toOdomXY(double mx, double my, double * ox, double * oy) const
  {
    const double c = std::cos(cur_.th), s = std::sin(cur_.th);
    *ox = cur_.x + c * mx - s * my;
    *oy = cur_.y + s * mx + c * my;
  }
  double toOdomYaw(double myaw) const {return wrap(myaw + cur_.th);}
  std::vector<agv::Pose2> toOdom(const std::vector<agv::Pose2> & v) const
  {
    std::vector<agv::Pose2> o;
    o.reserve(v.size());
    for (const auto & p : v) {
      double x, y;
      toOdomXY(p.x, p.y, &x, &y);
      o.push_back({x, y, wrap(p.th + cur_.th)});
    }
    return o;
  }

  // map→odom 最近 tf_avg_s 秒的平均 (航向用圆均值)
  agv::Pose2 averagedMapToOdom() const
  {
    if (m2o_.empty()) {return cur_;}
    double sx = 0, sy = 0, sc = 0, ss = 0;
    for (const auto & m : m2o_) {sx += m[1]; sy += m[2]; sc += std::cos(m[3]); ss += std::sin(m[3]);}
    const double n = static_cast<double>(m2o_.size());
    return {sx / n, sy / n, std::atan2(ss, sc)};
  }
  // odom 位姿 → map (T = map→odom，把 map 中的点变到 odom)
  static agv::Pose2 odomToMap(const agv::Pose2 & T, double x, double y, double yaw)
  {
    const double c = std::cos(T.th), s = std::sin(T.th), dx = x - T.x, dy = y - T.y;
    return {c * dx + s * dy, -s * dx + c * dy, wrap(yaw - T.th)};
  }
  ScanPoints & refineScan() {return refine_topic_.empty() ? scan_ : refine_scan_;}
  bool haveSegs()
  {
    std::lock_guard<std::mutex> lk(seg_mu_);
    return !segs_.empty();
  }
  agv::IcpResult runIcp(const std::vector<agv::Pt> & pts, const agv::Pose2 & init)
  {
    std::vector<agv::Seg> segs;
    {
      std::lock_guard<std::mutex> lk(seg_mu_);
      segs = segs_;
    }
    return agv::icp(pts, segs, init, half_thick_, 40, 0.025);
  }

  // 末段终点冻结到 odom 系 (T = map→odom)
  void freezeGoal(const agv::Pose2 & T)
  {
    const auto & last = pieces_[kf_];
    agv::Pose2 g = last.back();
    g.th = pieces_.back().back().th;             // 终点朝向 = 整条路径最后一个位姿的朝向
    // 进站方向: 最后一段的行驶方向 (段末两点)；只有一个点时用其朝向
    double appr_map = g.th;
    for (size_t i = last.size(); i-- > 1; ) {
      const double d = std::hypot(last[i].x - last[i - 1].x, last[i].y - last[i - 1].y);
      if (d > 1e-3) {appr_map = std::atan2(last[i].y - last[i - 1].y, last[i].x - last[i - 1].x); break;}
    }
    frozen_ = T;
    const double c = std::cos(T.th), s = std::sin(T.th);
    gx_ = T.x + c * g.x - s * g.y;
    gy_ = T.y + s * g.x + c * g.y;
    gyaw_ = wrap(g.th + T.th);
    appr_ = wrap(appr_map + T.th);
    RCLCPP_INFO(logger_, "[%s] 末段进站: 冻结 map→odom (与当前相差 %.1f mm / %.2f°)", name_.c_str(),
      1000.0 * std::hypot(cur_.x - T.x, cur_.y - T.y), wrap(cur_.th - T.th) * 180.0 / M_PI);
  }

  void publishStop(double d)
  {
    std_msgs::msg::Float32 m;
    m.data = static_cast<float>(d);
    stop_pub_->publish(m);
  }

  // 由转向切到行驶: 单舵轮先给微小前进指令，舵轮回正后再跟线
  bool driveSettle(double now, geometry_msgs::msg::TwistStamped & out)
  {
    if (settle_s_ <= 0.0 || mode_ == 1) {return false;}
    if (settle_t_ < 0 || mode_ != 3) {settle_t_ = now; mode_ = 3;}
    if (now - settle_t_ < settle_s_) {
      out.twist.linear.x = 0.002;
      return true;
    }
    mode_ = 1;
    settle_t_ = -1.0;
    return false;
  }

  // 原地转向到 target (odom 系)。返回 true 表示已到位 (±tol 且角速度 ~0)
  bool rotate(
    double target, double tol, double yaw, double w_meas, double now, geometry_msgs::msg::TwistStamped & out,
    double * err_out = nullptr)
  {
    double e = wrap(target - yaw);
    if (err_out) {*err_out = e;}
    if (std::fabs(e) < tol && std::fabs(w_meas) < 0.02) {
      if (still_since_ < 0) {still_since_ = now;}
      if (now - still_since_ > 0.15) {return true;}
      return false;
    }
    still_since_ = -1.0;
    // 方向: 最短方向，受阻时试反方向 (整圈扫掠)；已选定的方向保持到本次转完
    double age = 0.0;
    const auto pts = scan_.points(&age);
    const bool scan_ok = age < scan_max_age_;
    if (rot_dir_ != 0 && rot_dir_ * e < 0 && std::fabs(e) > 0.5) {e += rot_dir_ * 2.0 * M_PI;}
    if (rot_dir_ == 0) {
      const double d0 = e, d1 = e - (e > 0 ? 1.0 : -1.0) * 2.0 * M_PI;
      if (!scan_ok || !agv::rotationBlocked(pts, body_, rot_margin_, 0, 0, 0, d0)) {
        rot_dir_ = e > 0 ? 1 : -1;
      } else if (!agv::rotationBlocked(pts, body_, rot_margin_, 0, 0, 0, d1)) {
        rot_dir_ = e > 0 ? -1 : 1;
        e = d1;
        events_.emit("NAV2_ALIGN", "info", "原地转向改走反方向",
          "最短方向的转向扫掠区有障碍，反方向转 " + std::to_string(static_cast<int>(std::fabs(d1) * 180 / M_PI)) + "°");
      }
    }
    // 转动过程中持续检查前方 0.6 rad 的扫掠区
    const double look = (rot_dir_ != 0 ? rot_dir_ : (e > 0 ? 1 : -1)) * std::min(std::fabs(e), 0.6);
    const bool blocked = rot_dir_ == 0 || (scan_ok && agv::rotationBlocked(pts, body_, rot_margin_, 0, 0, 0, look));
    if (blocked) {
      out.twist.angular.z = 0.0;
      if (block_since_ < 0) {block_since_ = now;}
      if (now - block_since_ > block_wait_s_) {
        geometry_msgs::msg::PoseStamped req;
        req.header.frame_id = costmap_ros_->getGlobalFrameID();
        req.header.stamp = clock_->now();
        req.pose.position.x = rx_;
        req.pose.position.y = ry_;
        tf2::Quaternion q;
        q.setRPY(0, 0, target);
        req.pose.orientation.x = q.x();
        req.pose.orientation.y = q.y();
        req.pose.orientation.z = q.z();
        req.pose.orientation.w = q.w();
        turn_pub_->publish(req);
        const double e0 = wrap(target - rx_yaw_);
        const double e1 = e0 - (e0 > 0 ? 1.0 : -1.0) * 2.0 * M_PI;
        char buf[400];
        snprintf(buf, sizeof(buf),
          "阶段 %s，odom 位姿 (%.3f, %.3f, %.1f°) → 目标朝向 %.1f°；扫掠最小净空: 最短方向 %.0f mm / 反方向 %.0f mm "
          "(需 ≥ %.0f mm)；激光 %zu 点，%.2f s 前；车体 +%.2f/-%.2f × ±%.2f",
          phaseName(), rx_, ry_, rx_yaw_ * 180 / M_PI, target * 180 / M_PI,
          1000 * agv::rotationClearance(pts, body_, 0, 0, 0, e0), 1000 * agv::rotationClearance(pts, body_, 0, 0, 0, e1),
          1000 * rot_margin_, pts.size(), age, body_.head, body_.tail, body_.left);
        if (!block_reported_) {
          events_.emit("ROTATE_BLOCKED", "warning", "原地转向受阻，转入恢复", buf);
          block_reported_ = true;
        }
        throw nav2_core::PlannerException("AGV_ROTATION_BLOCKED");
      }
      return false;
    }
    block_since_ = -1.0;
    // 由行驶切到转向: 单舵轮先给微小角速度，舵轮转到位后再转
    if (settle_s_ > 0.0 && mode_ != 2) {
      if (mode_ != 4) {settle_t_ = now; mode_ = 4;}
      if (now - settle_t_ < settle_s_) {
        out.twist.angular.z = (e > 0 ? 1.0 : -1.0) * 0.004;
        return false;
      }
      settle_t_ = -1.0;
    }
    mode_ = 2;
    const double e_pred = e - w_meas * latency_;
    double w = std::min({rot_w_, std::sqrt(2.0 * rot_acc_ * std::fabs(e_pred)), 1.5 * std::fabs(e_pred)});
    w = std::max(w, std::fabs(e) >= tol ? 0.01 : 0.0);
    out.twist.angular.z = std::copysign(w, std::fabs(e_pred) > 1e-4 ? e_pred : e);
    return false;
  }

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_, primary_name_, map_frame_{"map"};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("agv_route_controller")};
  rclcpp::Clock::SharedPtr clock_;
  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  nav2_core::Controller::Ptr primary_;
  ScanPoints scan_, refine_scan_;
  std::string refine_topic_;
  EventPub events_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr stop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr turn_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr seg_sub_;
  std::mutex seg_mu_;
  std::vector<agv::Seg> segs_;
  bool refine_{true};
  double half_thick_{0.025}, refine_max_corr_{0.15}, refine_settle_s_{0.3};

  agv::Rect body_;
  double rot_w_, rot_acc_, rot_margin_, rot_tol_, yaw_tol_, cusp_tol_, final_dist_, final_stop_tol_, final_v_,
    final_min_v_, final_dec_, final_look_, latency_, tf_avg_s_, settle_s_, block_wait_s_, scan_max_age_;

  std_msgs::msg::Header header_;
  std::vector<std::vector<agv::Pose2>> pieces_;
  size_t k_{0}, hint_{0};
  Phase phase_{Phase::ALIGN};
  size_t kf_{0};
  double final_trigger_{0.8};
  double phase_t_{0}, still_since_{-1}, block_since_{-1}, settle_t_{-1};
  int rot_dir_{0};
  int mode_{0};           // 0 未定 1 行驶 2 转向 3 行驶前舵轮回正 4 转向前舵轮到位
  agv::Pose2 cur_{0, 0, 0};
  std::deque<std::array<double, 4>> m2o_;
  double gx_{0}, gy_{0}, gyaw_{0}, appr_{0};
  double rx_{0}, ry_{0}, rx_yaw_{0};
  bool block_reported_{false};
  int verify_n_{0};
  double heading_fix_tol_{0.0026};
  agv::Pose2 frozen_{0, 0, 0};
  const char * phaseName() const
  {
    static const char * n[] = {"ALIGN", "TRACK", "CUSP_STOP", "LOCALIZE", "FINAL", "STOPPING", "GOAL_ALIGN", "VERIFY", "DONE"};
    return n[static_cast<int>(phase_)];
  }
};

}  // namespace agv_nav2_plugins

PLUGINLIB_EXPORT_CLASS(agv_nav2_plugins::RouteController, nav2_core::Controller)
