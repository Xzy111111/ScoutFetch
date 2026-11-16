// SPDX-License-Identifier: Apache-2.0
#include <scoutfetch_reach/scoutfetch_reach_node.hpp>

#include <algorithm>
#include <cmath>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>

#include <moveit/robot_state/conversions.h>
#include <tf2/time.h>

namespace scoutfetch_reach
{

namespace
{
constexpr double kMinQuaternionNorm = 1e-6;

double quatNorm(const geometry_msgs::msg::Quaternion & q)
{
  return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

geometry_msgs::msg::Quaternion normalizeQuat(const geometry_msgs::msg::Quaternion & q)
{
  const double n = quatNorm(q);
  geometry_msgs::msg::Quaternion out = q;
  if (n > kMinQuaternionNorm) {
    out.x /= n;
    out.y /= n;
    out.z /= n;
    out.w /= n;
  }
  return out;
}

geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
}  // namespace

BasePosePlannerNode::BasePosePlannerNode(const rclcpp::NodeOptions & options)
: Node("scoutfetch_reach_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_, this, false)
{
  if (!loadParameters()) {
    RCLCPP_ERROR(this->get_logger(), "参数加载失败");
    throw std::runtime_error("scoutfetch_reach: parameter load failed");
  }

  const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                         .reliability(rclcpp::ReliabilityPolicy::Reliable)
                         .durability(rclcpp::DurabilityPolicy::TransientLocal);
  // Candidate IK can take seconds. Keep TF, live obstacle maps and joint
  // feedback responsive while one target is evaluated; serialize targets
  // with each other, not with sensor callbacks in the default group.
  target_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions target_options;
  target_options.callback_group = target_cb_group_;
  target_pose_map_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/target_pose_map", map_qos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      targetPoseMapCallback(msg);
    }, target_options);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    params_.costmap_topic, map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { costmapCallback(msg); });

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointStateCallback(msg); });

  best_base_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("/best_base_pose", 10);
  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/base_pose_candidates", 10);

  // 见 planCartesian: 服务客户端必须用独立 callback group,
  // 否则与阻塞 wait_for 的订阅回调同组串行 -> 响应永远无法送达。
  service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 等待 move_group 就绪并初始化 MoveGroupInterface (与 pick_node WAIT_MOVEIT 一致)
  const auto node_shared_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
  for (int i = 0; i < 20; ++i) {
    try {
      mgi_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_shared_ptr, params_.planning_group);
      break;
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not ready (%s), retrying...", e.what());
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  if (!mgi_) {
    RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface 初始化失败 (超时 20s)");
    throw std::runtime_error("scoutfetch_reach: MoveGroupInterface init failed");
  }
  RCLCPP_INFO(this->get_logger(), "MoveGroupInterface ready. waiting /target_pose_map ...");
}

bool BasePosePlannerNode::loadParameters()
{
  params_.radius_min = this->declare_parameter<double>("radius_min", 0.30);
  params_.radius_max = this->declare_parameter<double>("radius_max", 0.75);
  params_.radius_step = this->declare_parameter<double>("radius_step", 0.05);
  params_.angle_offsets_deg = this->declare_parameter<std::vector<double>>(
    "angle_offsets_deg", std::vector<double>{-60.0, -45.0, -30.0, 0.0, 30.0, 45.0, 60.0});

  // 用整数索引生成半径, 避免浮点累加在 0.75 边界漏点 (0.30+9*0.05=0.7500000000000001 > 0.75)。
  params_.radii.clear();
  const int n_radii = static_cast<int>(std::lround(
    (params_.radius_max - params_.radius_min) / params_.radius_step));
  for (int i = 0; i <= n_radii; ++i) {
    params_.radii.push_back(params_.radius_min + i * params_.radius_step);
  }

  params_.preferred_distance = this->declare_parameter<double>("preferred_distance", 0.475);
  params_.min_distance = this->declare_parameter<double>("min_distance", 0.25);
  params_.max_distance = this->declare_parameter<double>("max_distance", 0.75);

  // A8-R2: Nav2 终端误差鲁棒检查参数 (与 bringup general_goal_checker 保持一致)
  params_.nav_xy_error_margin = this->declare_parameter<double>("nav_xy_error_margin", 0.05);
  params_.nav_yaw_error_margin = this->declare_parameter<double>("nav_yaw_error_margin", 0.10);
  params_.enable_robust_reachability_check =
    this->declare_parameter<bool>("enable_robust_reachability_check", true);

  params_.costmap_topic = this->declare_parameter<std::string>("costmap_topic", "global_costmap/costmap");

  params_.planning_group = this->declare_parameter<std::string>("planning_group", "arm");
  params_.eef_link = this->declare_parameter<std::string>("eef_link", "gripper_tcp");
  params_.planning_frame = this->declare_parameter<std::string>("planning_frame", "base_link");

  params_.grasp_orientation.x = this->declare_parameter<double>("grasp_orientation.x", 0.0);
  params_.grasp_orientation.y = this->declare_parameter<double>("grasp_orientation.y", 0.819);
  params_.grasp_orientation.z = this->declare_parameter<double>("grasp_orientation.z", 0.0);
  params_.grasp_orientation.w = this->declare_parameter<double>("grasp_orientation.w", 0.574);

  {
    const auto axis = this->declare_parameter<std::vector<double>>("approach_axis", {0.9397, 0.0, -0.3420});
    if (axis.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), "approach_axis 长度必须为 3");
      return false;
    }
    params_.approach_axis = Eigen::Vector3d(axis[0], axis[1], axis[2]);
    if (params_.approach_axis.norm() < kMinQuaternionNorm) {
      RCLCPP_ERROR(this->get_logger(), "approach_axis 不能为零向量");
      return false;
    }
    params_.approach_axis.normalize();
  }
  params_.approach_distance = this->declare_parameter<double>("approach_distance", 0.10);

  {
    const auto axis = this->declare_parameter<std::vector<double>>("lift_axis", {-0.9397, 0.0, 0.3420});
    if (axis.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), "lift_axis 长度必须为 3");
      return false;
    }
    params_.lift_axis = Eigen::Vector3d(axis[0], axis[1], axis[2]);
    if (params_.lift_axis.norm() < kMinQuaternionNorm) {
      RCLCPP_ERROR(this->get_logger(), "lift_axis 不能为零向量");
      return false;
    }
    params_.lift_axis.normalize();
  }
  params_.lift_distance = this->declare_parameter<double>("lift_distance", 0.05);

  params_.cartesian_step = this->declare_parameter<double>("cartesian_step", 0.01);
  params_.grasp_min_z = this->declare_parameter<double>("grasp_min_z", -10.0);
  params_.floor_pick_enabled = this->declare_parameter<bool>("floor_pick_enabled", false);
  params_.floor_threshold_z = this->declare_parameter<double>("floor_threshold_z", 0.05);
  params_.floor_lift_distance = this->declare_parameter<double>("floor_lift_distance", 0.25);
  params_.floor_grasp_z = this->declare_parameter<double>("floor_grasp_z", -0.02);
  params_.floor_min_distance = this->declare_parameter<double>("floor_min_distance", 0.50);
  params_.floor_max_distance = this->declare_parameter<double>("floor_max_distance", 0.60);
  params_.floor_radius_step = this->declare_parameter<double>("floor_radius_step", 0.025);
  params_.grasp_offset = Eigen::Vector3d(
    this->declare_parameter<double>("grasp_offset.x", 0.0),
    this->declare_parameter<double>("grasp_offset.y", 0.0),
    this->declare_parameter<double>("grasp_offset.z", 0.0));
  params_.min_cartesian_fraction = this->declare_parameter<double>("min_cartesian_fraction", 0.99);
  params_.cartesian_avoid_collisions =
    this->declare_parameter<bool>("cartesian_avoid_collisions", false);
  params_.joint_margin = this->declare_parameter<double>("joint_margin", 0.05);

  params_.base_safety_margin = this->declare_parameter<double>("base_safety_margin", 0.05);
  params_.footprint_x = this->declare_parameter<double>("footprint_x", 0.275);
  params_.footprint_y = this->declare_parameter<double>("footprint_y", 0.20);
  params_.distance_weight = this->declare_parameter<double>("distance_weight", 0.6);
  params_.joint_weight = this->declare_parameter<double>("joint_weight", 0.4);
  params_.score_tie_epsilon = this->declare_parameter<double>("score_tie_epsilon", 0.02);

  params_.grasp_orientation = normalizeQuat(params_.grasp_orientation);

  RCLCPP_INFO(this->get_logger(),
              "params: radius[%.3f..%.3f step=%.3f] n=%zu angle_offsets=%zu preferred=%.3f "
              "min=%.3f max=%.3f "
              "nav_xy=%.3f nav_yaw=%.3f robust_check=%d "
              "costmap_topic=%s group=%s eef=%s "
              "cartesian_avoid_collisions=%d joint_margin=%.3f "
              "footprint=(%.3f, %.3f) margin=%.3f weights=(%.2f, %.2f) tie_eps=%.3f",
              params_.radius_min, params_.radius_max, params_.radius_step, params_.radii.size(),
              params_.angle_offsets_deg.size(), params_.preferred_distance,
              params_.min_distance, params_.max_distance,
              params_.nav_xy_error_margin, params_.nav_yaw_error_margin,
              params_.enable_robust_reachability_check, params_.costmap_topic.c_str(),
              params_.planning_group.c_str(),
              params_.eef_link.c_str(), params_.cartesian_avoid_collisions, params_.joint_margin,
              params_.footprint_x, params_.footprint_y, params_.base_safety_margin,
              params_.distance_weight, params_.joint_weight, params_.score_tie_epsilon);
  return true;
}

void BasePosePlannerNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = *msg;
}

void BasePosePlannerNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_mutex_);
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    joint_positions_[msg->name[i]] = msg->position[i];
  }
}

bool BasePosePlannerNode::getSeedState(moveit::core::RobotState & out)
{
  std::map<std::string, double> pos;
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    pos = joint_positions_;
  }
  if (pos.empty()) {
    return false;
  }
  const auto * jmg = mgi_->getRobotModel()->getJointModelGroup(params_.planning_group);
  if (!jmg) {
    return false;
  }
  out = moveit::core::RobotState(mgi_->getRobotModel());
  out.setToDefaultValues();
  std::vector<double> q;
  out.copyJointGroupPositions(jmg, q);
  const auto & vnames = jmg->getVariableNames();
  for (size_t i = 0; i < vnames.size(); ++i) {
    const auto it = pos.find(vnames[i]);
    if (it != pos.end()) {
      q[i] = it->second;
    }
  }
  out.setJointGroupPositions(jmg, q);
  out.update();
  return true;
}

std::vector<BasePosePlannerNode::Candidate> BasePosePlannerNode::generateCandidates(
  const geometry_msgs::msg::PoseStamped & target, double bx, double by) const
{
  std::vector<Candidate> cands;
  const double tx = target.pose.position.x;
  const double ty = target.pose.position.y;

  // 参考方向 = target 指向当前 robot 的方向; 候选在 robot 这一侧围绕目标布点,
  // 避免固定全局角度把候选大量撒到桌子另一侧。
  const double theta_ref = std::atan2(by - ty, bx - tx);

  auto radii = params_.radii;
  if (params_.floor_pick_enabled && target.pose.position.z < params_.floor_threshold_z) {
    radii.clear();
    const int count = static_cast<int>(std::floor(
      (params_.floor_max_distance-params_.floor_min_distance)/params_.floor_radius_step+1e-6));
    for (int i = 0; i <= count; ++i) {
      radii.push_back(params_.floor_min_distance+i*params_.floor_radius_step);
    }
  }
  for (const double r : radii) {
    if (params_.floor_pick_enabled && target.pose.position.z < params_.floor_threshold_z &&
        r < params_.floor_min_distance-1e-6) {
      continue;
    }
    for (const double off_deg : params_.angle_offsets_deg) {
      const double theta = theta_ref + off_deg * M_PI / 180.0;
      Candidate c;
      c.radius = r;
      c.angle_offset_deg = off_deg;
      c.base_pose.header.frame_id = "map";
      c.base_pose.header.stamp = target.header.stamp;  // 结果绑定到本 target request, 不用 now()
      c.base_pose.pose.position.x = tx + r * std::cos(theta);
      c.base_pose.pose.position.y = ty + r * std::sin(theta);
      c.base_pose.pose.position.z = 0.0;
      // yaw: 底盘 +X 始终朝向目标
      const double yaw =
        std::atan2(ty - c.base_pose.pose.position.y, tx - c.base_pose.pose.position.x);
      c.base_pose.pose.orientation = yawToQuat(yaw);
      cands.push_back(std::move(c));
    }
  }
  return cands;
}

bool BasePosePlannerNode::isPassable(const geometry_msgs::msg::PoseStamped & base_pose,
                                     SoftCost * soft_cost)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!latest_costmap_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "costmap 未就绪, 无法判定可通行 (假设不可行)");
    return false;
  }
  const auto & cm = *latest_costmap_;
  const double res = cm.info.resolution;
  const double x0 = cm.info.origin.position.x;
  const double y0 = cm.info.origin.position.y;
  const int w = cm.info.width;
  const int h = cm.info.height;

  const double x = base_pose.pose.position.x;
  const double y = base_pose.pose.position.y;
  const double qw = base_pose.pose.orientation.w;
  const double qz = base_pose.pose.orientation.z;
  const double yaw = std::atan2(2.0 * (qw * qz), 1.0 - 2.0 * qz * qz);
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);

  // A8-R3B corrected 语义 (对齐 Nav2 Smac GridCollisionChecker):
  //   - center cell: unknown/出图 fail-closed; cost >= 99 (raw 253 inscribed/254 lethal) 判碰撞
  //   - footprint 其余 cell: unknown fail-closed; 仅 cost == 100 (raw 254 lethal) 判硬碰撞
  //     (1~99 单独不 hard reject —— 与 Smac full-footprint >=254 一致)
  const int cc = static_cast<int>(std::floor((x - x0) / res));
  const int cr = static_cast<int>(std::floor((y - y0) / res));
  if (cc < 0 || cc >= w || cr < 0 || cr >= h) {
    return false;  // center 出图 fail-closed
  }
  const int8_t center_cost = cm.data[cr * w + cc];
  if (center_cost < 0) {
    return false;  // center 未知 (NO_INFORMATION)
  }
  if (center_cost >= 99) {
    return false;  // center 已落在 inscribed/lethal
  }

  // 真实底盘 footprint (半长/半宽) + 安全边距, 变换到 map
  const double fx = params_.footprint_x + params_.base_safety_margin;
  const double fy = params_.footprint_y + params_.base_safety_margin;
  const double cx[4] = {-fx, fx, fx, -fx};
  const double cy[4] = {-fy, -fy, fy, fy};
  double min_x = std::numeric_limits<double>::max(), max_x = -std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max(), max_y = -std::numeric_limits<double>::max();
  for (int k = 0; k < 4; ++k) {
    const double mx = x + cx[k] * cos_y - cy[k] * sin_y;
    const double my = y + cx[k] * sin_y + cy[k] * cos_y;
    min_x = std::min(min_x, mx);
    max_x = std::max(max_x, mx);
    min_y = std::min(min_y, my);
    max_y = std::max(max_y, my);
  }

  // 覆盖的 AABB 单元格; 只对确实落在旋转矩形内的 cell 判成本
  const int col0 = static_cast<int>(std::floor((min_x - x0) / res));
  const int col1 = static_cast<int>(std::floor((max_x - x0) / res));
  const int row0 = static_cast<int>(std::floor((min_y - y0) / res));
  const int row1 = static_cast<int>(std::floor((max_y - y0) / res));
  for (int row = row0; row <= row1; ++row) {
    for (int col = col0; col <= col1; ++col) {
      if (col < 0 || col >= w || row < 0 || row >= h) {
        return false;  // footprint 覆盖到地图外
      }
      const double mx = x0 + (col + 0.5) * res;
      const double my = y0 + (row + 0.5) * res;
      const double lx = (mx - x) * cos_y + (my - y) * sin_y;
      const double ly = -(mx - x) * sin_y + (my - y) * cos_y;
      if (std::fabs(lx) > fx || std::fabs(ly) > fy) {
        continue;  // cell 中心在旋转矩形外, 跳过
      }
      const int8_t cost = cm.data[row * w + col];
      if (cost < 0) {
        return false;  // 未知空间 (NO_INFORMATION = -1)
      }
      if (cost == 100) {
        return false;  // 仅 lethal (raw 254) 判硬碰撞
      }
      if (soft_cost != nullptr && cost <= 99) {
        soft_cost->max_nonlethal = std::max(soft_cost->max_nonlethal, static_cast<double>(cost));
        soft_cost->sum_nonlethal += cost;
        ++soft_cost->n_nonlethal;
      }
    }
  }
  if (soft_cost != nullptr && soft_cost->n_nonlethal > 0) {
    soft_cost->mean_nonlethal = soft_cost->sum_nonlethal / soft_cost->n_nonlethal;
  }
  return true;
}

bool BasePosePlannerNode::getCurrentPose(double & x, double & y, double & yaw)
{
  try {
    // 非阻塞最新 TF: TimePointZero = 最新可用; 不带 timeout, 避免 tf2_buffer 线程 WARN。
    // TF 缺失时抛异常, 返回 false, 由调用方 fail-closed (跳过本次规划)。
    const auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    const auto & q = tf.transform.rotation;
    yaw = std::atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z);
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "TF map->base_link 不可用 (%s)", e.what());
    return false;
  }
}

GraspFailReason BasePosePlannerNode::checkGraspFeasibility(Candidate & c)
{
  const auto * jmg = mgi_->getRobotModel()->getJointModelGroup(params_.planning_group);
  if (!jmg) {
    RCLCPP_ERROR(this->get_logger(), "规划组 '%s' 不存在", params_.planning_group.c_str());
    return GraspFailReason::IK;
  }

  // 整链明细重置 (Test D 报告); 逐步骤在通过时置位
  c.detail_ik_ok = false;
  c.detail_approach_ok = false;
  c.detail_lift_ok = false;
  c.detail_rev_frac = 0.0;
  c.detail_lift_frac = 0.0;

  // 目标在 candidate base_link 系 = object_pose (z 保持目标离地高度)
  geometry_msgs::msg::Pose object_pose;
  object_pose.position.x = c.target_base_x;
  object_pose.position.y = c.target_base_y;
  object_pose.position.z = c.target_base_z;
  object_pose.orientation.w = 1.0;

  // grasp_pose / pre_grasp_pose / lift_pose
  geometry_msgs::msg::Pose grasp_pose;
  grasp_pose.position = object_pose.position;
  grasp_pose.position.x += params_.grasp_offset.x();
  grasp_pose.position.y += params_.grasp_offset.y();
  grasp_pose.position.z += params_.grasp_offset.z();
  grasp_pose.position.z = std::max(grasp_pose.position.z, params_.grasp_min_z);
  grasp_pose.orientation = params_.grasp_orientation;

  geometry_msgs::msg::Pose pre_grasp_pose = grasp_pose;
  pre_grasp_pose.position.x -= params_.approach_axis.x() * params_.approach_distance;
  pre_grasp_pose.position.y -= params_.approach_axis.y() * params_.approach_distance;
  pre_grasp_pose.position.z -= params_.approach_axis.z() * params_.approach_distance;

  geometry_msgs::msg::Pose lift_pose = grasp_pose;
  lift_pose.position.x += params_.lift_axis.x() * params_.lift_distance;
  lift_pose.position.y += params_.lift_axis.y() * params_.lift_distance;
  lift_pose.position.z += params_.lift_axis.z() * params_.lift_distance;
  if (params_.floor_pick_enabled && c.target_base_z < params_.floor_threshold_z) {
    grasp_pose.position.z = params_.floor_grasp_z;
    pre_grasp_pose = grasp_pose;
    pre_grasp_pose.position.z += params_.approach_distance;
    lift_pose = grasp_pose;
    lift_pose.position.z += params_.floor_lift_distance;
  }

  // 以真实关节反馈为 seed (自维护 feedback/joint_states, 不经 MoveGroupInterface monitor)
  auto start = std::make_shared<moveit::core::RobotState>(mgi_->getRobotModel());
  if (!getSeedState(*start)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "feedback/joint_states 未就绪, 无法做 IK seed");
    return GraspFailReason::IK;
  }

  // 1. grasp_pose IK -> q_grasp
  moveit::core::RobotState ik_state(mgi_->getRobotModel());
  ik_state = *start;
  if (!ik_state.setFromIK(jmg, grasp_pose, params_.eef_link, 0.2)) {
    RCLCPP_INFO(this->get_logger(),
                "  [grasp fail] target_base=(%.3f, %.3f, %.3f) step=IK 无解",
                c.target_base_x, c.target_base_y, c.target_base_z);
    return GraspFailReason::IK;  // 无 IK 解
  }
  ik_state.update();
  c.detail_ik_ok = true;
  // q_grasp 关节居中分 (各关节按上下限归一化, 中心=1 越近限位越低), 供 step 5 加权评分
  c.joint_score = jointCenteredScore(ik_state, jmg);
  std::vector<double> q_grasp;
  ik_state.copyJointGroupPositions(jmg, q_grasp);
  auto q_grasp_state = std::make_shared<moveit::core::RobotState>(mgi_->getRobotModel());
  q_grasp_state->setJointGroupPositions(jmg, q_grasp);
  q_grasp_state->update();

  // 1.5 joint margin (q_grasp)
  double worst = 0.0;
  if (!jointsWithinMargin(*q_grasp_state, jmg, worst)) {
    RCLCPP_INFO(this->get_logger(),
                "  [grasp fail] target_base=(%.3f, %.3f, %.3f) step=joint_margin(q_grasp) worst=%.3f",
                c.target_base_x, c.target_base_y, c.target_base_z, worst);
    return GraspFailReason::JOINT_MARGIN;
  }

  // 2. 反向 Cartesian grasp -> pre_grasp (approach 连续可达)
  moveit_msgs::msg::RobotTrajectory reverse_traj;
  double fraction = 0.0;
  if (!planCartesian(q_grasp_state, {pre_grasp_pose}, reverse_traj, fraction)) {
    RCLCPP_INFO(this->get_logger(),
                "  [grasp fail] target_base=(%.3f, %.3f, %.3f) step=reverse_cartesian 调用失败",
                c.target_base_x, c.target_base_y, c.target_base_z);
    return GraspFailReason::APPROACH;
  }
  if (fraction < params_.min_cartesian_fraction) {
    RCLCPP_INFO(this->get_logger(),
                "  [grasp fail] target_base=(%.3f, %.3f, %.3f) step=reverse_cartesian frac=%.3f<%.2f",
                c.target_base_x, c.target_base_y, c.target_base_z, fraction,
                params_.min_cartesian_fraction);
    return GraspFailReason::APPROACH;
  }
  c.detail_approach_ok = true;
  c.detail_rev_frac = fraction;

  // 2.5 joint margin (q_pre_grasp)
  auto q_pre_grasp_state = trajectoryEndState(q_grasp_state, reverse_traj);
  if (!jointsWithinMargin(*q_pre_grasp_state, jmg, worst)) {
    RCLCPP_INFO(this->get_logger(),
                "  [grasp fail] target_base=(%.3f, %.3f, %.3f) step=joint_margin(pre_grasp) worst=%.3f",
                c.target_base_x, c.target_base_y, c.target_base_z, worst);
    return GraspFailReason::JOINT_MARGIN;
  }

  // 3. (V1 简化) 不再做 current -> q_pre_grasp MoveGroup 关节规划:
  //    候选假设底盘已位于该 Pose, 真实到位后的 current->pre_grasp 轨迹
  //    由导航到位后的 scoutfetch_grasp 最终验证。

  // 4. lift Cartesian (从 q_grasp)
  moveit_msgs::msg::RobotTrajectory lift_traj;
  double lift_fraction = 0.0;
  if (!planCartesian(q_grasp_state, {lift_pose}, lift_traj, lift_fraction)) {
    RCLCPP_INFO(this->get_logger(),
                "  [grasp fail] target_base=(%.3f, %.3f, %.3f) step=lift_cartesian 调用失败",
                c.target_base_x, c.target_base_y, c.target_base_z);
    return GraspFailReason::LIFT;
  }
  if (lift_fraction < params_.min_cartesian_fraction) {
    RCLCPP_INFO(this->get_logger(),
                "  [grasp fail] target_base=(%.3f, %.3f, %.3f) step=lift_cartesian frac=%.3f<%.2f",
                c.target_base_x, c.target_base_y, c.target_base_z, lift_fraction,
                params_.min_cartesian_fraction);
    return GraspFailReason::LIFT;
  }
  c.detail_lift_ok = true;
  c.detail_lift_frac = lift_fraction;

  // 4.5 joint margin (lift 末端)
  auto lift_end_state = trajectoryEndState(q_grasp_state, lift_traj);
  if (!jointsWithinMargin(*lift_end_state, jmg, worst)) {
    RCLCPP_INFO(this->get_logger(),
                "  [grasp fail] target_base=(%.3f, %.3f, %.3f) step=joint_margin(lift) worst=%.3f",
                c.target_base_x, c.target_base_y, c.target_base_z, worst);
    return GraspFailReason::JOINT_MARGIN;
  }

  RCLCPP_INFO(this->get_logger(),
              "  feasible: base(%.3f, %.3f) yaw=%.1f deg  target_base=(%.3f, %.3f, %.3f) "
              "rev_frac=%.3f lift_frac=%.3f",
              c.base_pose.pose.position.x, c.base_pose.pose.position.y,
              std::atan2(2.0 * (c.base_pose.pose.orientation.w * c.base_pose.pose.orientation.z),
                         1.0 - 2.0 * c.base_pose.pose.orientation.z * c.base_pose.pose.orientation.z) *
                180.0 / M_PI,
              c.target_base_x, c.target_base_y, c.target_base_z, fraction, lift_fraction);
  return GraspFailReason::OK;
}

void BasePosePlannerNode::computeTargetInBase(const geometry_msgs::msg::PoseStamped & target,
                                              double bx, double by, double yaw,
                                              double & tx, double & ty) const
{
  // 与 callback 内 nominal 目标->base_link 变换完全一致 (map->base_link 2D 旋转)
  const double dx = target.pose.position.x - bx;
  const double dy = target.pose.position.y - by;
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);
  tx = cos_y * dx + sin_y * dy;
  ty = -sin_y * dx + cos_y * dy;
}

bool BasePosePlannerNode::navErrorCollisionEnvelope(
  const geometry_msgs::msg::PoseStamped & base_pose, RobustDetail & detail)
{
  // Nav2 终端误差碰撞包络: translation radius = nav_xy_error_margin,
  // 平面 8 方向 (0..315 deg) x 3 yaw (nominal, +margin, -margin) = 24 个终端 pose 样本。
  // 复用现有 isPassable (A8-R3B corrected 语义), 任一样本碰撞即淘汰 (fail-closed)。
  const double bx = base_pose.pose.position.x;
  const double by = base_pose.pose.position.y;
  const double qw = base_pose.pose.orientation.w;
  const double qz = base_pose.pose.orientation.z;
  const double nominal_yaw = std::atan2(2.0 * (qw * qz), 1.0 - 2.0 * qz * qz);
  const double eps = params_.nav_xy_error_margin;
  const double dyaws[3] = {0.0, params_.nav_yaw_error_margin, -params_.nav_yaw_error_margin};
  detail.envelope_total = 8 * 3;
  detail.envelope_safe = 0;
  for (int i = 0; i < 8; ++i) {
    const double a = i * M_PI / 4.0;
    const double px = bx + eps * std::cos(a);
    const double py = by + eps * std::sin(a);
    for (const double dyaw : dyaws) {
      geometry_msgs::msg::PoseStamped p;
      p.header = base_pose.header;
      p.pose.position.x = px;
      p.pose.position.y = py;
      p.pose.position.z = 0.0;
      p.pose.orientation = yawToQuat(nominal_yaw + dyaw);
      if (!isPassable(p)) {
        return false;
      }
      ++detail.envelope_safe;
    }
  }
  return true;
}

bool BasePosePlannerNode::navErrorManipulationRobust(
  const Candidate & c, const geometry_msgs::msg::PoseStamped & target, RobustDetail & detail)
{
  // 只检查与"抓不到"最相关的误差最坏情形, 不复制第二套 IK/Cartesian 算法:
  //   Case B: base 沿 away-from-target 偏移 nav_xy_error_margin (yaw nominal)
  //   Case C/D: Case B 位置 + yaw ±nav_yaw_error_margin
  // 每个情形: 重算 target in base 系, 构造候选副本, 复用 checkGraspFeasibility 整链。
  const double tx = target.pose.position.x;
  const double ty = target.pose.position.y;
  const double bx = c.base_pose.pose.position.x;
  const double by = c.base_pose.pose.position.y;
  const double qw = c.base_pose.pose.orientation.w;
  const double qz = c.base_pose.pose.orientation.z;
  const double nominal_yaw = std::atan2(2.0 * (qw * qz), 1.0 - 2.0 * qz * qz);

  // away-from-target 单位向量 (target -> base)
  const double dx = bx - tx;
  const double dy = by - ty;
  const double dist = std::hypot(dx, dy);
  const double ux = dist > 1e-9 ? dx / dist : 1.0;
  const double uy = dist > 1e-9 ? dy / dist : 0.0;

  const double bx_b = bx + params_.nav_xy_error_margin * ux;
  const double by_b = by + params_.nav_xy_error_margin * uy;

  double txb, tyb;
  if (params_.floor_pick_enabled && c.target_base_z < params_.floor_threshold_z) {
    // A floor grasp also has a near reach limit: arriving too close forces
    // the wrist into the chassis. Check this side of the navigation envelope.
    for (double dyaw : {-params_.nav_yaw_error_margin, 0.0, params_.nav_yaw_error_margin}) {
      computeTargetInBase(target, bx-params_.nav_xy_error_margin*ux,
                          by-params_.nav_xy_error_margin*uy, nominal_yaw+dyaw, txb, tyb);
      Candidate near = c;
      near.target_base_x = txb;
      near.target_base_y = tyb;
      if (checkGraspFeasibility(near) != GraspFailReason::OK) {
        return false;
      }
    }
  }
  computeTargetInBase(target, bx_b, by_b, nominal_yaw, txb, tyb);
  detail.away_tdist = std::hypot(txb, tyb);
  {
    Candidate tmp = c;
    tmp.target_base_x = txb;
    tmp.target_base_y = tyb;
    detail.away_reason = checkGraspFeasibility(tmp);
    detail.away_rev_frac = tmp.detail_rev_frac;
    detail.away_lift_frac = tmp.detail_lift_frac;
    if (detail.away_reason != GraspFailReason::OK) {
      return false;
    }
  }

  computeTargetInBase(target, bx_b, by_b, nominal_yaw + params_.nav_yaw_error_margin, txb, tyb);
  {
    Candidate tmp = c;
    tmp.target_base_x = txb;
    tmp.target_base_y = tyb;
    detail.yaw_p_reason = checkGraspFeasibility(tmp);
    if (detail.yaw_p_reason != GraspFailReason::OK) {
      return false;
    }
  }

  computeTargetInBase(target, bx_b, by_b, nominal_yaw - params_.nav_yaw_error_margin, txb, tyb);
  {
    Candidate tmp = c;
    tmp.target_base_x = txb;
    tmp.target_base_y = tyb;
    detail.yaw_n_reason = checkGraspFeasibility(tmp);
    if (detail.yaw_n_reason != GraspFailReason::OK) {
      return false;
    }
  }
  return true;
}

bool BasePosePlannerNode::runNavRobustCheck(const Candidate & c,
                                            const geometry_msgs::msg::PoseStamped & target,
                                            RobustDetail & detail)
{
  detail = RobustDetail{};
  if (!navErrorCollisionEnvelope(c.base_pose, detail)) {
    return false;
  }
  if (!navErrorManipulationRobust(c, target, detail)) {
    return false;
  }
  return true;
}

namespace
{
const char * graspFailName(GraspFailReason r)
{
  switch (r) {
    case GraspFailReason::OK: return "PASS";
    case GraspFailReason::IK: return "FAIL(IK)";
    case GraspFailReason::APPROACH: return "FAIL(APPROACH)";
    case GraspFailReason::LIFT: return "FAIL(LIFT)";
    case GraspFailReason::JOINT_MARGIN: return "FAIL(JOINT_MARGIN)";
  }
  return "FAIL(?)";
}

// 按单原因逐步淘汰模型: IK->approach->lift 逐级判定各步骤是否通过
const char * stepName(GraspFailReason r, int step)  // step: 0=IK 1=approach 2=lift
{
  switch (step) {
    case 0: return (r != GraspFailReason::IK) ? "PASS" : "FAIL(IK)";
    case 1: return (r == GraspFailReason::LIFT || r == GraspFailReason::OK) ? "PASS" : "FAIL";
    case 2: return (r == GraspFailReason::OK) ? "PASS" : "FAIL";
  }
  return "?";
}
}  // namespace

void BasePosePlannerNode::printRobustReport(const Candidate & best) const
{
  const auto & r = best.robust;
  const double nominal_tdist =
    std::hypot(best.target_base_x, best.target_base_y);
  std::cout << "---- Nav 误差鲁棒报告 (Test D) ----\n"
            << "best base (map): (" << best.base_pose.pose.position.x << ", "
            << best.base_pose.pose.position.y << ")  radius=" << best.radius
            << "  yaw=" << std::atan2(2.0 * (best.base_pose.pose.orientation.w *
                                             best.base_pose.pose.orientation.z),
                                      1.0 - 2.0 * best.base_pose.pose.orientation.z *
                                             best.base_pose.pose.orientation.z) *
                              180.0 / M_PI
            << " deg\n"
            << "collision envelope: " << r.envelope_safe << "/" << r.envelope_total << " safe\n"
            << "footprint soft cost: max_nonlethal=" << best.soft_max_nonlethal
            << " mean_nonlethal=" << best.soft_mean_nonlethal << "\n"
            << "nominal:\n"
            << "  collision: PASS\n"
            << "  target distance: " << nominal_tdist << " m\n"
            << "  IK: " << (best.detail_ik_ok ? "PASS" : "FAIL") << "\n"
            << "  approach: " << (best.detail_approach_ok ? "PASS" : "FAIL")
            << " (frac=" << best.detail_rev_frac << ")\n"
            << "  lift: " << (best.detail_lift_ok ? "PASS" : "FAIL")
            << " (frac=" << best.detail_lift_frac << ")\n"
            << "nav-error-away +" << params_.nav_xy_error_margin << " (yaw nominal):\n"
            << "  target distance: " << r.away_tdist << " m\n"
            << "  IK: " << stepName(r.away_reason, 0) << "\n"
            << "  approach: " << stepName(r.away_reason, 1)
            << " (frac=" << r.away_rev_frac << ")\n"
            << "  lift: " << stepName(r.away_reason, 2)
            << " (frac=" << r.away_lift_frac << ")\n"
            << "yaw +" << params_.nav_yaw_error_margin << ":\n"
            << "  IK: " << graspFailName(r.yaw_p_reason) << "\n"
            << "yaw -" << params_.nav_yaw_error_margin << ":\n"
            << "  IK: " << graspFailName(r.yaw_n_reason) << "\n"
            << "------------------------------------\n";
}

bool BasePosePlannerNode::jointsWithinMargin(const moveit::core::RobotState & state,
                                             const moveit::core::JointModelGroup * jmg,
                                             double & worst_margin) const
{
  worst_margin = std::numeric_limits<double>::infinity();
  bool ok = true;
  for (const auto * jm : jmg->getActiveJointModels()) {
    for (const auto & v : jm->getVariableNames()) {
      const auto & vb = jm->getVariableBounds(v);
      if (!vb.position_bounded_) {
        continue;  // 连续关节无界, 不检查
      }
      const double qv = state.getVariablePosition(v);
      const double m = std::min(qv - vb.min_position_, vb.max_position_ - qv);
      worst_margin = std::min(worst_margin, m);
      if (m < params_.joint_margin) {
        ok = false;
      }
    }
  }
  return ok;
}

double BasePosePlannerNode::jointCenteredScore(const moveit::core::RobotState & state,
                                               const moveit::core::JointModelGroup * jmg) const
{
  double sum = 0.0;
  int n = 0;
  for (const auto * jm : jmg->getActiveJointModels()) {
    for (const auto & v : jm->getVariableNames()) {
      const auto & vb = jm->getVariableBounds(v);
      if (!vb.position_bounded_) {
        continue;  // 连续关节无界, 不参与
      }
      const double range = vb.max_position_ - vb.min_position_;
      if (range <= 0.0) {
        continue;
      }
      const double qv = state.getVariablePosition(v);
      const double center = 0.5 * (vb.min_position_ + vb.max_position_);
      const double s = 1.0 - std::fabs(qv - center) / (0.5 * range);
      sum += std::clamp(s, 0.0, 1.0);
      ++n;
    }
  }
  return n > 0 ? sum / n : 1.0;
}

double BasePosePlannerNode::distanceScore(const Candidate & c) const
{
  const double d = std::sqrt(c.target_base_x * c.target_base_x + c.target_base_y * c.target_base_y);
  const double width = params_.max_distance - params_.min_distance;
  if (width <= 0.0) {
    return 1.0;
  }
  const double score = 1.0 - std::fabs(d - params_.preferred_distance) / (0.5 * width);
  return std::clamp(score, 0.0, 1.0);
}

void BasePosePlannerNode::targetPoseMapCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto t0 = std::chrono::steady_clock::now();  // A8-R2 性能记录
  RCLCPP_INFO(this->get_logger(), "收到 /target_pose_map (frame=%s) pos=(%.3f, %.3f, %.3f)",
              msg->header.frame_id.c_str(), msg->pose.position.x, msg->pose.position.y,
              msg->pose.position.z);

  // 当前机器人位姿 (map->base_link): 候选参考方向 + 并列决策 nav_distance 都依赖它。
  // 不可用时 fail-closed: 不规划、不发布, 直接返回, 保持节点存活。
  // 绝不回退到 map 原点伪造机器人位置。
  double cur_x = 0.0, cur_y = 0.0, cur_yaw = 0.0;
  if (!getCurrentPose(cur_x, cur_y, cur_yaw)) {
    RCLCPP_WARN(this->get_logger(),
                "TF map->base_link 不可用: 跳过本次目标规划, 不发布 /best_base_pose");
    clearMarkers();  // 清除旧候选 markers, 避免视觉残留
    return;
  }
  RCLCPP_INFO(this->get_logger(), "当前机器人 (map): (%.3f, %.3f) yaw=%.1f deg",
              cur_x, cur_y, cur_yaw * 180.0 / M_PI);

  // 候选以 target->robot 方向为参考; 仅在真实 robot 位姿可用时生成。
  auto cands = generateCandidates(*msg, cur_x, cur_y);
  const size_t total = cands.size();

  int footprint_rejected = 0;
  int ik_rejected = 0;
  int approach_rejected = 0;
  int lift_rejected = 0;
  int joint_margin_rejected = 0;
  int nav_error_collision_rejected = 0;   // A8-R2
  int nav_error_unreachable_rejected = 0;  // A8-R2

  for (size_t i = 0; i < cands.size(); ++i) {
    Candidate & c = cands[i];
    // 计算目标相对 candidate base_link
    const double dx = msg->pose.position.x - c.base_pose.pose.position.x;
    const double dy = msg->pose.position.y - c.base_pose.pose.position.y;
    const double qw = c.base_pose.pose.orientation.w;
    const double qz = c.base_pose.pose.orientation.z;
    const double yaw = std::atan2(2.0 * (qw * qz), 1.0 - 2.0 * qz * qz);
    const double cos_y = std::cos(yaw);
    const double sin_y = std::sin(yaw);
    c.target_base_x = cos_y * dx + sin_y * dy;
    c.target_base_y = -sin_y * dx + cos_y * dy;
    c.target_base_z = msg->pose.position.z;

    // step 1: 完整底盘 footprint 碰撞检查 (A8-R3B corrected 语义:
    // center>=99 / footprint==100 / unknown fail-closed; 记录软成本, 不改 scoring)
    SoftCost soft;
    c.passable = isPassable(c.base_pose, &soft);
    if (!c.passable) {
      ++footprint_rejected;
      continue;
    }
    c.soft_max_nonlethal = soft.max_nonlethal;
    c.soft_mean_nonlethal = soft.mean_nonlethal;

    // step 2-4: 抓取可行性 (IK/approach/lift + joint margin); OK 时已填充 c.joint_score
    const auto reason = checkGraspFeasibility(c);
    c.fail_reason = static_cast<int>(reason);
    if (reason != GraspFailReason::OK) {
      switch (reason) {
        case GraspFailReason::IK:
          ++ik_rejected;
          break;
        case GraspFailReason::APPROACH:
          ++approach_rejected;
          break;
        case GraspFailReason::LIFT:
          ++lift_rejected;
          break;
        case GraspFailReason::JOINT_MARGIN:
          ++joint_margin_rejected;
          break;
        default:
          break;
      }
      continue;
    }

    // step 2.5: Nav2 终端误差鲁棒检查 (A8-R2, additional filter, 不降低 nominal 任何标准)
    // 仅对 nominal 整链通过的候选: 碰撞包络 (8x3) + 误差最坏操控 (Case B/C/D) 全过才算 valid。
    if (params_.enable_robust_reachability_check) {
      if (!runNavRobustCheck(c, *msg, c.robust)) {
        const bool coll_fail = c.robust.envelope_safe < c.robust.envelope_total;
        c.nav_fail_reason = static_cast<int>(coll_fail
                                               ? NavRobustFailReason::NAV_ERROR_COLLISION
                                               : NavRobustFailReason::NAV_ERROR_UNREACHABLE);
        if (coll_fail) {
          ++nav_error_collision_rejected;
        } else {
          ++nav_error_unreachable_rejected;
        }
        continue;
      }
      c.nav_robust = true;
    }

    // step 5: 评分 (硬淘汰仅 footprint/抓取可行性/joint_margin/Nav误差鲁棒;
    //          距离不再作为硬过滤, 以权重参与总分)
    c.distance_score = distanceScore(c);
    c.total_score = params_.distance_weight * c.distance_score +
                    params_.joint_weight * c.joint_score;
    c.status = 1;
  }

  // step 6: nav_distance (仅并列决策, 不入主评分); cur_x/cur_y 已在函数开头取得 (TF 可用)
  int valid = 0;
  for (auto & c : cands) {
    if (c.status == 1) {
      ++valid;
      c.nav_distance =
        std::hypot(c.base_pose.pose.position.x - cur_x, c.base_pose.pose.position.y - cur_y);
    }
  }

  // best 选择: 先最大化 total_score; |Δtotal| <= score_tie_epsilon 时取 nav_distance 更小
  bool have_best = false;
  size_t best_index = 0;
  for (size_t i = 0; i < cands.size(); ++i) {
    if (cands[i].status != 1) {
      continue;
    }
    if (!have_best) {
      have_best = true;
      best_index = i;
      continue;
    }
    const double delta = cands[i].total_score - cands[best_index].total_score;
    if (delta > params_.score_tie_epsilon) {
      best_index = i;  // 严格更优
    } else if (std::fabs(delta) <= params_.score_tie_epsilon &&
               cands[i].nav_distance < cands[best_index].nav_distance) {
      best_index = i;  // 并列内取 nav_distance 更小
    }
  }

  if (have_best) {
    cands[best_index].status = 2;
    publishBest(cands[best_index]);
    publishMarkers(cands, *msg);
  } else {
    // valid==0 (含 costmap 未就绪导致全淘汰): 不发布 best, 清除旧 markers 避免残留
    clearMarkers();
  }

  // ---- 验收打印 ----
  const int grasp_rejected =
    ik_rejected + approach_rejected + lift_rejected + joint_margin_rejected;
  const int robust_rejected = nav_error_collision_rejected + nav_error_unreachable_rejected;
  const double elapsed_s =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::cout << "\n========== BASE POSE PLANNER REPORT ==========\n"
            << "target (map): (" << msg->pose.position.x << ", " << msg->pose.position.y << ", "
            << msg->pose.position.z << ")\n"
            << "target_stamp: " << msg->header.stamp.sec << "." << std::setw(9)
            << std::setfill('0') << msg->header.stamp.nanosec << "\n"
            << "candidate 总数:                " << total << "\n"
            << "footprint/costmap 淘汰数:      " << footprint_rejected << "\n"
            << "IK 淘汰数:                     " << ik_rejected << "\n"
            << "approach 淘汰数:               " << approach_rejected << "\n"
            << "lift 淘汰数:                   " << lift_rejected << "\n"
            << "joint-margin 淘汰数:           " << joint_margin_rejected << "\n"
            << "grasp 淘汰数 (合计):           " << grasp_rejected << "\n"
            << "Nav误差碰撞淘汰数:             " << nav_error_collision_rejected << "\n"
            << "Nav误差不可达淘汰数:           " << nav_error_unreachable_rejected << "\n"
            << "Nav误差鲁棒淘汰数 (合计):      " << robust_rejected << "\n"
            << "valid 数:                      " << valid << "\n"
            << "planner 单次计算耗时:          " << elapsed_s << " s\n";
  if (have_best) {
    const auto & b = cands[best_index];
    const double yaw =
      std::atan2(2.0 * (b.base_pose.pose.orientation.w * b.base_pose.pose.orientation.z),
                 1.0 - 2.0 * b.base_pose.pose.orientation.z * b.base_pose.pose.orientation.z);
    std::cout << "best candidate:   map (x=" << b.base_pose.pose.position.x << ", y="
              << b.base_pose.pose.position.y << ") yaw=" << yaw * 180.0 / M_PI
              << " deg  target_base=(" << b.target_base_x << ", " << b.target_base_y << ", "
              << b.target_base_z << ")\n"
              << "best_stamp:       " << b.base_pose.header.stamp.sec << "." << std::setw(9)
              << std::setfill('0') << b.base_pose.header.stamp.nanosec << "\n"
              << "distance_score:   " << b.distance_score << "\n"
              << "joint_score:      " << b.joint_score << "\n"
              << "total_score:      " << b.total_score << "\n"
              << "nav_distance:     " << b.nav_distance << "\n";
    if (params_.enable_robust_reachability_check) {
      printRobustReport(b);  // A8-R2 Test D: nominal / away / yaw± / collision 24/24
    }
  } else {
    std::cout << "best candidate:   NONE (无可通行且可抓取候选)\n";
  }

  // 打印全部 valid candidate, 确认 best 非遍历顺序导致
  if (valid > 0) {
    std::cout << "---- 全部 valid candidate (radius angle distance_score joint_score total_score nav_distance) ----\n";
    for (const auto & c : cands) {
      if (c.status >= 1) {
        std::cout << "  r=" << c.radius << "  a=" << c.angle_offset_deg << "  d=" << c.distance_score
                  << "  j=" << c.joint_score << "  total=" << c.total_score
                  << "  nav=" << c.nav_distance << "\n";
      }
    }
    std::cout << "-----------------------------------------------------------------------\n";
  }
  if (valid == 0) {
    printFailReasonStats(cands);
  }

  // 按 radius 汇总: free = footprint/costmap 可通行候选数, grasp-valid = nominal 整链通过数,
  // robust-valid = 通过 Nav 误差鲁棒检查的候选数 (A8-R2)
  std::cout << "---- 按 radius: free / grasp-valid / robust-valid ----\n";
  for (const double r : params_.radii) {
    int free_n = 0, valid_n = 0, robust_n = 0;
    for (const auto & c : cands) {
      if (std::fabs(c.radius - r) < 1e-9) {
        if (c.passable) {
          ++free_n;
        }
        if (c.status >= 1) {
          ++valid_n;
        }
        if (c.nav_robust) {
          ++robust_n;
        }
      }
    }
    std::cout << "  r=" << r << "  free=" << free_n << "  grasp-valid=" << valid_n
              << "  robust-valid=" << robust_n << "\n";
  }
  std::cout << "=============================================\n";
}

void BasePosePlannerNode::printFailReasonStats(const std::vector<Candidate> & cands) const
{
  std::cout << "---- 按 (radius, angle) 主要失败原因 ----\n";
  for (const auto & c : cands) {
    const char * reason = "OK";
    if (!c.passable) {
      reason = "costmap";
    } else if (c.nav_fail_reason != 0) {
      reason = (c.nav_fail_reason == static_cast<int>(NavRobustFailReason::NAV_ERROR_COLLISION))
                 ? "nav_error_collision"
                 : "nav_error_unreachable";
    } else {
      switch (static_cast<GraspFailReason>(c.fail_reason)) {
        case GraspFailReason::IK:
          reason = "IK";
          break;
        case GraspFailReason::APPROACH:
          reason = "approach";
          break;
        case GraspFailReason::LIFT:
          reason = "lift";
          break;
        case GraspFailReason::JOINT_MARGIN:
          reason = "joint_margin";
          break;
        default:
          reason = "OK";
          break;
      }
    }
    std::cout << "  r=" << c.radius << "  a=" << c.angle_offset_deg << "  -> " << reason << "\n";
  }
  std::cout << "------------------------------------------\n";
}

void BasePosePlannerNode::publishBest(const Candidate & best) const
{
  best_base_pose_pub_->publish(best.base_pose);
  RCLCPP_INFO(this->get_logger(), "/best_base_pose 发布 (map): (%.3f, %.3f) total_score=%.3f",
              best.base_pose.pose.position.x, best.base_pose.pose.position.y, best.total_score);
}

void BasePosePlannerNode::publishMarkers(const std::vector<Candidate> & cands,
                                         const geometry_msgs::msg::PoseStamped & target) const
{
  visualization_msgs::msg::MarkerArray markers;

  // 目标参考点
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = rclcpp::Time(0);
    m.ns = "scoutfetch_reach";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = target.pose.position;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.06;
    m.color.r = 1.0;
    m.color.g = 0.2;
    m.color.b = 0.2;
    m.color.a = 0.9;
    m.lifetime = rclcpp::Duration(0, 0);
    markers.markers.push_back(m);
  }

  // 候选底盘 (圆柱, 直径 ~ footprint 0.4m), 按状态着色
  for (size_t i = 0; i < cands.size(); ++i) {
    const auto & c = cands[i];
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = rclcpp::Time(0);
    m.ns = "scoutfetch_reach";
    m.id = static_cast<int>(i) + 1;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = c.base_pose.pose;
    m.pose.position.z = 0.05;
    if (c.status == 2) {  // best
      m.scale.x = m.scale.y = 0.45;
      m.color.r = 0.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.color.a = 0.9;
    } else if (c.status == 1) {  // valid
      m.scale.x = m.scale.y = 0.40;
      m.color.r = 0.0;
      m.color.g = 0.4;
      m.color.b = 1.0;
      m.color.a = 0.6;
    } else {  // invalid
      m.scale.x = m.scale.y = 0.40;
      m.color.r = 0.6;
      m.color.g = 0.6;
      m.color.b = 0.6;
      m.color.a = 0.4;
    }
    m.scale.z = 0.04;
    m.lifetime = rclcpp::Duration(0, 0);
    markers.markers.push_back(m);
  }

  marker_pub_->publish(markers);
}

void BasePosePlannerNode::clearMarkers() const
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = rclcpp::Time(0);
  m.ns = "scoutfetch_reach";
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(m);
  marker_pub_->publish(markers);
}

moveit::core::RobotStatePtr BasePosePlannerNode::trajectoryEndState(
  moveit::core::RobotStatePtr start, const moveit_msgs::msg::RobotTrajectory & traj) const
{
  auto t = std::make_shared<robot_trajectory::RobotTrajectory>(
    mgi_->getRobotModel(), params_.planning_group);
  t->setRobotTrajectoryMsg(*start, traj);
  return std::make_shared<moveit::core::RobotState>(t->getLastWayPoint());
}

bool BasePosePlannerNode::planCartesian(moveit::core::RobotStatePtr start,
                                        const std::vector<geometry_msgs::msg::Pose> & waypoints,
                                        moveit_msgs::msg::RobotTrajectory & out_traj,
                                        double & fraction)
{
  const std::string srv_name = "/compute_cartesian_path";
  if (!cartesian_client_) {
    cartesian_client_ = this->create_client<moveit_msgs::srv::GetCartesianPath>(
      srv_name, rmw_qos_profile_services_default, service_cb_group_);
  }
  if (!cartesian_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "GetCartesianPath 服务不可用: %s", srv_name.c_str());
    return false;
  }

  auto req = std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>();
  req->header.frame_id = params_.planning_frame;
  req->header.stamp = this->now();
  moveit::core::robotStateToRobotStateMsg(*start, req->start_state);
  req->group_name = params_.planning_group;
  req->link_name = params_.eef_link;
  req->waypoints = waypoints;
  req->max_step = params_.cartesian_step;
  req->jump_threshold = 0.0;
  req->prismatic_jump_threshold = 0.0;
  req->revolute_jump_threshold = 0.0;
  req->avoid_collisions = params_.cartesian_avoid_collisions;

  auto fut = cartesian_client_->async_send_request(req);
  const auto status = fut.wait_for(std::chrono::seconds(10));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "GetCartesianPath 服务调用超时");
    return false;
  }
  auto resp = fut.get();
  fraction = resp->fraction;
  out_traj = resp->solution;
  return true;
}

}  // namespace scoutfetch_reach

#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scoutfetch_reach::BasePosePlannerNode>();
  // 回调内会阻塞等待 /compute_cartesian_path 服务响应,
  // 必须多线程 executor, 否则单线程 executor 被回调占住, 服务响应无人处理 -> 超时。
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
