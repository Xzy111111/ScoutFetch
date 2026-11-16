// SPDX-License-Identifier: Apache-2.0
#include <scoutfetch_grasp/pick_node.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

namespace scoutfetch_grasp
{

namespace
{
constexpr double kMinQuaternionNorm = 1e-6;
constexpr int kMaxMoveItWaitCycles = 20;  // 20 * 1.0s
constexpr double kJointStateWaitSeconds = 5.0;  // execute 首读前等待真实 joint state 的时长

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

// resolve_topic_name 返回完整展开名 (如 /control/joint_states), 去掉前导 '/' 便于与配置比较
std::string stripLeadingSlash(std::string s)
{
  if (!s.empty() && s.front() == '/') {
    s.erase(s.begin());
  }
  return s;
}

double maxTrajectoryVelocity(const moveit_msgs::msg::RobotTrajectory & traj)
{
  double max_v = 0.0;
  for (const auto & pt : traj.joint_trajectory.points) {
    for (const auto & v : pt.velocities) {
      max_v = std::max(max_v, std::fabs(v));
    }
  }
  return max_v;
}

double maxTrajectoryAcceleration(const moveit_msgs::msg::RobotTrajectory & traj)
{
  double max_a = 0.0;
  for (const auto & pt : traj.joint_trajectory.points) {
    for (const auto & a : pt.accelerations) {
      max_a = std::max(max_a, std::fabs(a));
    }
  }
  return max_a;
}

double trajectoryDuration(const moveit_msgs::msg::RobotTrajectory & traj)
{
  const auto & pts = traj.joint_trajectory.points;
  if (pts.empty()) {
    return 0.0;
  }
  return rclcpp::Duration(pts.back().time_from_start).seconds();
}

// 把 grasp→pre_grasp 反向轨迹反转为 pre_grasp→grasp 正向 approach:
// 反转 waypoints, 按原时间间隔反向重建单调递增时间戳 (保留速度轮廓), 清零速度/加速度,
// 交给 IPTP 重新时间参数化。不直接沿用反转前的 time_from_start (方向不连续)。
void reverseTrajectory(const moveit_msgs::msg::RobotTrajectory & reverse_in,
                       moveit_msgs::msg::RobotTrajectory & forward_out)
{
  forward_out = reverse_in;
  auto & pts = forward_out.joint_trajectory.points;
  const size_t n = pts.size();
  if (n < 2) {
    return;
  }
  std::vector<double> diffs(n - 1);
  for (size_t i = 0; i + 1 < n; ++i) {
    const double t0 = rclcpp::Duration(pts[i].time_from_start).seconds();
    const double t1 = rclcpp::Duration(pts[i + 1].time_from_start).seconds();
    diffs[i] = std::max(0.0, t1 - t0);
  }
  std::reverse(pts.begin(), pts.end());
  double t = 0.0;
  for (size_t i = 0; i < n; ++i) {
    pts[i].time_from_start = rclcpp::Duration::from_seconds(t);
    pts[i].velocities.assign(pts[i].velocities.size(), 0.0);
    pts[i].accelerations.assign(pts[i].accelerations.size(), 0.0);
    if (i + 1 < n) {
      t += diffs[n - 2 - i];
    }
  }
}

}  // namespace

PickNode::PickNode(const rclcpp::NodeOptions & options)
: Node("pick_node", options)
{
}

PickNode::~PickNode()
{
  stop_spin_.store(true);
  if (spin_exec_) {
    spin_exec_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  stop_marker_repub_.store(true);
  if (marker_repub_thread_.joinable()) {
    marker_repub_thread_.join();
  }
}

bool PickNode::run()
{
  RCLCPP_INFO(this->get_logger(), "=== scoutfetch_grasp pick_node (常驻) ===");

  // ---- 夹爪订阅/发布 (MultiThreadedExecutor 后台线程, 回调不阻塞) ----
  gripper_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("control/gripper_cmd", 10);
  gripper_status_sub_ = this->create_subscription<scoutfetch_interfaces::msg::GripperStatus>(
    "feedback/gripper_status", 10,
    [this](const scoutfetch_interfaces::msg::GripperStatus::SharedPtr msg) { gripperStatusCallback(msg); });

  // 默认 QoS (volatile): 与 RViz MarkerArray display 默认订阅兼容。
  // 迟订阅问题靠"RViz 先启动 + 周期重发 marker"解决, 而不是 transient_local
  // (transient_local pub 与 RViz 默认 volatile sub QoS 不兼容, 反而连不上)。
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("grasp_markers", 10);

  // ---- 夹爪反馈回调线程 ----
  spin_exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  spin_thread_ = std::thread([this]() {
    spin_exec_->add_node(this->get_node_base_interface());
    // 全量 spin(): 线程池派发, reentrant 组 service 阻塞在 pool 线程时,
    // 默认组 (joint_states sub / GetCartesianPath client) 仍由其他 pool 线程调度。
    // (spin_some 在 Humble 中为单线程 inline 执行, 阻塞 service 会饿死默认组回调。)
    spin_exec_->spin();
  });

  // ---- LOAD_PARAMETERS (必须先于 MoveGroupInterface 初始化) ----
  state_ = State::LOAD_PARAMETERS;
  if (!loadParameters()) {
    RCLCPP_ERROR(this->get_logger(), "ABORT: 参数加载失败");
    return false;
  }

  // ---- 启动信息 + 安全检查: execute=true 时禁止 mock 关节 topic ----
  {
    const std::string js_topic =
      stripLeadingSlash(this->get_node_topics_interface()->resolve_topic_name("joint_states"));
    RCLCPP_INFO(this->get_logger(),
                "startup: execute=%s allow_real_execution=%s joint_states_topic=%s "
                "planning_group=%s planning_frame=%s eef_link=%s",
                params_.execute ? "true" : "false",
                params_.allow_real_execution ? "true" : "false", js_topic.c_str(),
                params_.planning_group.c_str(), params_.planning_frame.c_str(),
                params_.eef_link.c_str());
    if (params_.execute && js_topic == "control/joint_states") {
      RCLCPP_ERROR(this->get_logger(),
                   "ABORT: execute=true but mock joint state topic is selected "
                   "(joint_states_topic=%s). 拒绝在 mock 关节 topic 上执行真机抓取。"
                   "请改用 joint_states_topic:=feedback/joint_states 启动。",
                   js_topic.c_str());
      return false;
    }
    // 授权状态提示: AUTHORIZED != AUTO START, 真实执行仍必须由 /execute_pick 显式触发
    if (params_.execute && !params_.allow_real_execution) {
      RCLCPP_WARN(this->get_logger(),
                  "REAL PICK EXECUTION BLOCKED: allow_real_execution=false "
                  "(execute=true 但未授权, /execute_pick 将拒绝真实执行)");
    } else if (params_.execute && params_.allow_real_execution) {
      RCLCPP_INFO(this->get_logger(),
                  "REAL PICK EXECUTION AUTHORIZED (仍等待 /execute_pick 显式触发, 不会自动执行)");
    }
  }

  // ---- WAIT_MOVEIT ----
  state_ = State::WAIT_MOVEIT;
  const auto node_shared_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
  int wait_cycles = 0;
  while (rclcpp::ok() && wait_cycles < kMaxMoveItWaitCycles) {
    try {
      mgi_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_shared_ptr, params_.planning_group);
      break;
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not ready (%s), retrying...", e.what());
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ++wait_cycles;
  }
  if (!mgi_) {
    RCLCPP_ERROR(this->get_logger(), "ABORT: MoveGroupInterface 初始化失败 (超时 %ds)",
                 kMaxMoveItWaitCycles);
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "WAIT_MOVEIT OK");

  // ---- COMPUTE_POSES (启动/standalone: 用 YAML object_pose, 不绑定任务目标) ----
  state_ = State::COMPUTE_POSES;
  if (!computePoses(std::nullopt)) {
    RCLCPP_ERROR(this->get_logger(), "ABORT: Pose 计算失败");
    return false;
  }

  // ---- 启动诊断: TCP +Z 方向 vs approach_axis 夹角 (仅提示) ----
  {
    double deg = 0.0;
    checkGraspApproachAlignment(deg);
  }

  // ---- PUBLISH_MARKERS ----
  state_ = State::PUBLISH_MARKERS;
  publishMarkers();
  startMarkerRepublisher();

  // ---- 常驻输入/触发接口: 目标只缓存, 规划仅由 /execute_pick 显式触发 ----
  grasp_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    params_.target_topic, 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { onGraspTarget(msg); });
  execute_service_cg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  execute_pick_srv_ = this->create_service<std_srvs::srv::Trigger>(
    params_.execute_service,
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      onExecutePick(request, response);
    },
    rmw_qos_profile_services_default, execute_service_cg_);
  recovery_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/prepare_grasp_recovery",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      if (!params_.execute || !params_.allow_real_execution) {
        response->success = false;
        response->message = "recovery execution not enabled";
        return;
      }
      if (pick_in_progress_.exchange(true)) {
        response->success = false;
        response->message = "arm action already running";
        return;
      }
      try {
        moveit::core::RobotStatePtr start;
        moveit_msgs::msg::RobotTrajectory trajectory;
        // Empty-hand observation posture. Wrist tilts down to see the floor
        // near the front of the chassis; shoulder/elbow keep the fingers
        // above the base while the wrist looks down. Check the entire motion.
        const auto & view = params_.recovery_view_joints;
        response->success = openGripper() && readRealState("recovery", start);
        bool at_view = false;
        if (response->success) {
          std::vector<double> joints;
          start->copyJointGroupPositions(start->getJointModelGroup(params_.planning_group), joints);
          at_view = joints.size() == view.size();
          for (size_t i = 0; at_view && i < joints.size(); ++i) {
            at_view = std::abs(joints[i]-view[i]) < .02;
          }
          start->update();
          const auto & tcp = start->getGlobalLinkTransform(params_.eef_link);
          if (!at_view && tcp.translation().z() < 0.15) {
            // Leave a failed low grasp vertically before folding the arm;
            // a direct joint path near the floor can have a narrow escape.
            geometry_msgs::msg::Pose retreat;
            retreat.position.x = tcp.translation().x();
            retreat.position.y = tcp.translation().y();
            retreat.position.z = tcp.translation().z()+params_.floor_lift_distance;
            const Eigen::Quaterniond rotation(tcp.linear());
            retreat.orientation.x = rotation.x();
            retreat.orientation.y = rotation.y();
            retreat.orientation.z = rotation.z();
            retreat.orientation.w = rotation.w();
            double fraction = 0.0;
            response->success = planCartesian(start, {retreat}, "recovery_clearance", trajectory, fraction) &&
              executeTrajectory(trajectory, "recovery_clearance") && readRealState("recovery_clearance", start);
          }
        }
        if (response->success && !at_view) {
          response->success = planToJointTarget(start, view, "recovery_view", trajectory) &&
            executeTrajectory(trajectory, "recovery_view");
        }
        response->message = response->success ? "fresh floor observation ready" :
          "recovery view planning/execution failed";
        if (response->success) {
          std::lock_guard<std::mutex> lock(target_mutex_);
          latest_target_.reset();
        }
      } catch (const std::exception & error) {
        response->success = false;
        response->message = error.what();
      }
      pick_in_progress_.store(false);
    }, rmw_qos_profile_services_default, execute_service_cg_);
  if (params_.execute) {
    // 授权状态已在启动日志打印; 此处仅说明 /execute_pick 入口行为, 不做任何自动执行
    RCLCPP_INFO(this->get_logger(),
                "/execute_pick 入口: %s",
                params_.allow_real_execution ? "real execution authorized (显式触发后执行真机抓取)"
                                             : "real execution blocked (仅允许 dry-run)");
  }
  RCLCPP_INFO(this->get_logger(),
              "pick_node 常驻就绪: 订阅 %s (仅缓存目标), 触发服务 %s",
              params_.target_topic.c_str(), params_.execute_service.c_str());

  // ---- 常驻: 空闲循环保持进程存活, 回调由 spin_thread_ 的 MultiThreadedExecutor 调度。
  //      Ctrl-C / shutdown 后 rclcpp::ok() 为 false, 退出并正常返回。----
  while (rclcpp::ok() && !stop_spin_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return true;
}

// ==================== 真机执行 (execute=true) ====================

bool PickNode::runExecute(const geometry_msgs::msg::PoseStamped & active_pick_target)
{
  RCLCPP_INFO(this->get_logger(),
              "=== execute=true: 真机空抓流程, grasp-anchored IK branch, 每段执行后读真实状态校验 ===");
  last_pick_error_.clear();

  // 任务目标绑定: 本次真实执行目标 = /execute_pick 触发快照 (唯一来源), 与 dry-run 同源。
  // 执行期间即使新 /grasp_target_pose 到达也不改变当前目标; 本函数禁止读取 latest_target_。
  RCLCPP_INFO(this->get_logger(), "REAL PICK TARGET");
  RCLCPP_INFO(this->get_logger(), "  frame: %s", active_pick_target.header.frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "  stamp: %ld.%09ld",
              static_cast<long>(active_pick_target.header.stamp.sec),
              static_cast<long>(active_pick_target.header.stamp.nanosec));
  RCLCPP_INFO(this->get_logger(), "  x: %.4f", active_pick_target.pose.position.x);
  RCLCPP_INFO(this->get_logger(), "  y: %.4f", active_pick_target.pose.position.y);
  RCLCPP_INFO(this->get_logger(), "  z: %.4f", active_pick_target.pose.position.z);
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!computePoses(active_pick_target.pose.position)) {
      last_pick_error_ = "real execution aborted: pose computation failed";
      return false;
    }
  }
  // 刷新 marker 显示本次真实执行目标 (与 dry-run 一致)
  publishMarkers();

  // 0. 首次真实状态读取前, 等待有效 joint state (含 joint1~6, 有限数值)。
  //    超时未就绪即 ABORT, 不进入后续任何动作。
  if (!waitForValidJointState(kJointStateWaitSeconds)) {
    last_pick_error_ = "real execution aborted: valid joint state unavailable";
    return false;
  }

  // 1. 整链预验证 (无运动): q_grasp IK → 反向 Cartesian → q_pre_grasp → joint target
  //    → 正向 approach(反转+retime) → lift → ready 全部可规划才允许动作。
  moveit::core::RobotStatePtr start;
  if (!readRealState("start", start)) {
    last_pick_error_ = "real execution aborted: read start state failed";
    RCLCPP_ERROR(this->get_logger(), "ABORT: 读取起始真实状态失败");
    return false;
  }
  GraspChain chain;
  if (!planFullChain(start, chain)) {
    // planFullChain 内部已记录具体失败原因 (如 grasp IK failed), 未记录时才补通用标签
    if (last_pick_error_.empty()) {
      last_pick_error_ = "real execution aborted: full chain pre-verification failed";
    }
    RCLCPP_ERROR(this->get_logger(),
                 "ABORT: 真机运动前整链预验证失败 (grasp-anchored IK branch), 未执行任何运动");
    return false;
  }

  // 2. 夹爪开启 + 反馈确认 (整链已验证后才动作)
  state_ = State::OPEN_GRIPPER;
  if (!openGripper()) {
    last_pick_error_ = "real execution aborted: open gripper failed";
    RCLCPP_ERROR(this->get_logger(), "ABORT: 夹爪开启失败");
    return false;
  }

  // 3. 执行 current→q_pre_grasp (joint-space 规划轨迹, 已验证)
  state_ = State::EXECUTE_PRE_GRASP;
  if (!executeTrajectory(chain.pre_grasp_traj, "pre_grasp")) {
    last_pick_error_ = "real execution aborted: pre_grasp execution failed";
    RCLCPP_ERROR(this->get_logger(), "ABORT: pre_grasp 执行失败");
    return false;
  }

  // 4. 读真实状态校验 ≈ q_pre_grasp, 执行正向 approach (反转+retime 轨迹)
  {
    state_ = State::EXECUTE_APPROACH;
    moveit::core::RobotStatePtr s;
    if (!readRealState("pre_grasp", s)) {
      last_pick_error_ = "real execution aborted: read pre_grasp state failed";
      RCLCPP_ERROR(this->get_logger(), "ABORT: 读取 pre_grasp 后真实状态失败");
      return false;
    }
    if (!validateStateAgainst(*s, chain.q_pre_grasp, "pre_grasp")) {
      last_pick_error_ = "real execution aborted: pre_grasp state validation failed";
      return false;
    }
    if (!executeTrajectory(chain.approach_traj, "approach")) {
      last_pick_error_ = "real execution aborted: approach execution failed";
      RCLCPP_ERROR(this->get_logger(), "ABORT: approach 执行失败");
      return false;
    }
  }

  // 5. 夹爪闭合 + 反馈确认 (TCP 已到 grasp_pose, 夹住目标)
  state_ = State::CLOSE_GRIPPER;
  if (!closeGripper()) {
    last_pick_error_ = "real execution aborted: close gripper failed";
    RCLCPP_ERROR(this->get_logger(), "ABORT: 夹爪闭合失败");
    return false;
  }

  // 6. 读真实状态校验 ≈ q_grasp, 执行 lift (retime 轨迹)
  {
    state_ = State::EXECUTE_LIFT;
    moveit::core::RobotStatePtr s;
    if (!readRealState("grasp", s)) {
      last_pick_error_ = "real execution aborted: read grasp state failed";
      RCLCPP_ERROR(this->get_logger(), "ABORT: 读取 grasp 后真实状态失败");
      return false;
    }
    if (!validateStateAgainst(*s, chain.q_grasp, "grasp")) {
      last_pick_error_ = "real execution aborted: grasp state validation failed";
      return false;
    }
    if (!executeTrajectory(chain.lift_traj, "lift")) {
      last_pick_error_ = "real execution aborted: lift execution failed";
      RCLCPP_ERROR(this->get_logger(), "ABORT: lift 执行失败");
      return false;
    }
  }

  if (!verifyHeld()) {
    last_pick_error_ = "bottle lost after lift";
    return false;
  }

  // 7. 读真实状态校验 ≈ q_lift_end, 执行收臂 (stow 或 ready)
  if (params_.return_to_stow || params_.return_to_ready) {
    state_ = State::EXECUTE_READY;
    moveit::core::RobotStatePtr s;
    if (!readRealState("lift", s)) {
      last_pick_error_ = "real execution aborted: read lift state failed";
      RCLCPP_ERROR(this->get_logger(), "ABORT: 读取 lift 后真实状态失败");
      return false;
    }
    if (!validateStateAgainst(*s, chain.q_lift_end, "lift")) {
      last_pick_error_ = "real execution aborted: lift state validation failed";
      return false;
    }
    const char * stow_label = params_.return_to_stow ? "stow" : "ready";
    if (!executeTrajectory(chain.ready_traj, stow_label)) {
      last_pick_error_ = std::string("real execution aborted: ") + stow_label +
        " execution failed";
      RCLCPP_ERROR(this->get_logger(), "ABORT: %s 执行失败", stow_label);
      return false;
    }
  }

  if (!verifyHeld()) {
    last_pick_error_ = "bottle lost after stow";
    return false;
  }
  // 8. DONE
  state_ = State::EXECUTE_SUCCESS;
  {
    moveit::core::RobotStatePtr fin;
    if (readRealState("final", fin)) {
      logRealState("final", *fin);
    }
  }
  RCLCPP_INFO(this->get_logger(), "=== 真机空抓全流程完成 ===");
  return true;
}

bool PickNode::retimeTrajectory(moveit::core::RobotStatePtr start,
                                moveit_msgs::msg::RobotTrajectory & traj)
{
  if (traj.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "retime: 轨迹为空");
    return false;
  }
  auto rt = std::make_shared<robot_trajectory::RobotTrajectory>(
    mgi_->getRobotModel(), params_.planning_group);
  rt->setRobotTrajectoryMsg(*start, traj);
  trajectory_processing::IterativeParabolicTimeParameterization time_param;
  if (!time_param.computeTimeStamps(*rt, params_.velocity_scaling, params_.acceleration_scaling)) {
    RCLCPP_ERROR(this->get_logger(), "retime: IterativeParabolicTimeParameterization 失败");
    return false;
  }
  rt->getRobotTrajectoryMsg(traj);
  return true;
}

moveit::core::RobotStatePtr PickNode::trajectoryEndState(
  moveit::core::RobotStatePtr start,
  const moveit_msgs::msg::RobotTrajectory & traj) const
{
  auto t = std::make_shared<robot_trajectory::RobotTrajectory>(
    mgi_->getRobotModel(), params_.planning_group);
  t->setRobotTrajectoryMsg(*start, traj);
  return std::make_shared<moveit::core::RobotState>(t->getLastWayPoint());
}

bool PickNode::planFullChain(moveit::core::RobotStatePtr start, GraspChain & chain)
{
  const auto * jmg = start->getJointModelGroup(params_.planning_group);
  if (!jmg) {
    RCLCPP_ERROR(this->get_logger(), "ABORT: 规划组 '%s' 不存在", params_.planning_group.c_str());
    last_pick_error_ = "planning group missing";
    return false;
  }

  // ===== 1. grasp_pose IK → q_grasp (Task 3), 以当前真实状态为 seed =====
  state_ = State::PLAN_PRE_GRASP;
  {
    moveit::core::RobotState ik_state(mgi_->getRobotModel());
    ik_state = *start;
    if (!ik_state.setFromIK(jmg, grasp_pose_, params_.eef_link, 0.2)) {
      RCLCPP_ERROR(this->get_logger(),
                   "ABORT: grasp_pose (%.3f, %.3f, %.3f) 无 IK 解 (seed=当前状态, 不调整姿态)",
                   grasp_pose_.position.x, grasp_pose_.position.y, grasp_pose_.position.z);
      last_pick_error_ = "grasp IK failed";
      return false;
    }
    if (!ik_state.satisfiesBounds(jmg)) {
      RCLCPP_WARN(this->get_logger(), "q_grasp 超出 joint bounds (IK 仍返回解, 仅记录)");
    }
    ik_state.update();
    ik_state.copyJointGroupPositions(jmg, chain.q_grasp);
    q_grasp_ = chain.q_grasp;
    std::ostringstream oss;
    oss.precision(4);
    for (size_t i = 0; i < chain.q_grasp.size(); ++i) {
      if (i) {
        oss << ", ";
      }
      oss << chain.q_grasp[i];
    }
    RCLCPP_INFO(this->get_logger(), "q_grasp (grasp_pose IK) = [%s]", oss.str().c_str());
  }
  auto q_grasp_state = std::make_shared<moveit::core::RobotState>(*start);
  {
    moveit::core::RobotState gs(mgi_->getRobotModel());
    gs = *start;
    gs.setJointGroupPositions(jmg, chain.q_grasp);
    gs.update();
    q_grasp_state = std::make_shared<moveit::core::RobotState>(gs);
  }

  // ===== 2. 反向 Cartesian: grasp → pre_grasp (Task 4) =====
  moveit_msgs::msg::RobotTrajectory reverse_traj;
  {
    double fraction = 0.0;
    if (!planCartesian(q_grasp_state, {pre_grasp_pose_}, "reverse_approach", reverse_traj,
                       fraction)) {
      RCLCPP_ERROR(this->get_logger(), "ABORT: 反向 Cartesian (grasp→pre_grasp) 失败");
      last_pick_error_ = "approach planning failed";
      return false;
    }
    if (fraction < params_.min_cartesian_fraction) {
      RCLCPP_ERROR(this->get_logger(),
                   "ABORT: 反向 Cartesian fraction=%.4f < %.2f, 该 q_grasp branch 无法连续到 pre_grasp",
                   fraction, params_.min_cartesian_fraction);
      last_pick_error_ = "approach planning failed: fraction insufficient";
      return false;
    }
    chain.reverse_fraction = fraction;
    reverse_approach_fraction_ = fraction;
    RCLCPP_INFO(this->get_logger(),
                "反向 Cartesian grasp→pre_grasp fraction=%.4f points=%zu", fraction,
                reverse_traj.joint_trajectory.points.size());
  }

  // ===== 3. q_pre_grasp = 反向轨迹终点 (Task 5, 与 grasp 同一连续 IK branch) =====
  auto q_pre_grasp_state = trajectoryEndState(q_grasp_state, reverse_traj);
  {
    q_pre_grasp_state->copyJointGroupPositions(jmg, chain.q_pre_grasp);
    q_pre_grasp_ = chain.q_pre_grasp;
    std::ostringstream oss;
    oss.precision(4);
    for (size_t i = 0; i < chain.q_pre_grasp.size(); ++i) {
      if (i) {
        oss << ", ";
      }
      oss << chain.q_pre_grasp[i];
    }
    RCLCPP_INFO(this->get_logger(), "q_pre_grasp (反向 Cartesian 终点) = [%s]", oss.str().c_str());
  }

  // ===== 4. current → q_pre_grasp: joint-space 规划 (Task 6, 不再 setPoseTarget) =====
  {
    state_ = State::PLAN_PRE_GRASP;
    if (!planToJointTarget(start, chain.q_pre_grasp, "pre_grasp", chain.pre_grasp_traj)) {
      RCLCPP_ERROR(this->get_logger(), "ABORT: current→q_pre_grasp 规划失败");
      last_pick_error_ = "pre_grasp planning failed";
      return false;
    }
    pre_grasp_plan_point_count_ =
      static_cast<int>(chain.pre_grasp_traj.joint_trajectory.points.size());
    RCLCPP_INFO(this->get_logger(), "current→q_pre_grasp 规划成功, 轨迹点数=%d",
                pre_grasp_plan_point_count_);
  }

  // ===== 5. 正向 approach: 反转反向轨迹 + IPTP retime (Task 7) =====
  {
    reverseTrajectory(reverse_traj, chain.approach_traj);
    if (chain.approach_traj.joint_trajectory.points.empty()) {
      RCLCPP_ERROR(this->get_logger(), "ABORT: 正向 approach 反转失败 (空轨迹)");
      last_pick_error_ = "approach planning failed: empty trajectory";
      return false;
    }
    if (!retimeTrajectory(q_pre_grasp_state, chain.approach_traj)) {
      RCLCPP_ERROR(this->get_logger(), "ABORT: 正向 approach IPTP retime 失败");
      last_pick_error_ = "approach planning failed: retime";
      return false;
    }
    approach_fraction_ = chain.reverse_fraction;  // 同一几何路径
    RCLCPP_INFO(this->get_logger(),
                "正向 approach (反转+retime): points=%zu duration=%.2fs max_velocity=%.4f rad/s",
                chain.approach_traj.joint_trajectory.points.size(),
                trajectoryDuration(chain.approach_traj), maxTrajectoryVelocity(chain.approach_traj));
  }

  // ===== 6. lift (从 q_grasp, retime) =====
  {
    state_ = State::PLAN_CARTESIAN_LIFT;
    double fraction = 0.0;
    if (!planCartesian(q_grasp_state, {lift_pose_}, "lift", chain.lift_traj, fraction)) {
      RCLCPP_ERROR(this->get_logger(), "ABORT: Cartesian lift 规划失败");
      last_pick_error_ = "lift planning failed";
      return false;
    }
    lift_fraction_ = fraction;
    if (!retimeTrajectory(q_grasp_state, chain.lift_traj)) {
      RCLCPP_ERROR(this->get_logger(), "ABORT: lift IPTP retime 失败");
      last_pick_error_ = "lift planning failed: retime";
      return false;
    }
    auto lift_end = trajectoryEndState(q_grasp_state, chain.lift_traj);
    lift_end->copyJointGroupPositions(jmg, chain.q_lift_end);
    RCLCPP_INFO(this->get_logger(),
                "lift: fraction=%.4f points=%zu (retime)", fraction,
                chain.lift_traj.joint_trajectory.points.size());
  }

  // ===== 7. 收臂: 搬运收臂关节目标 或 ready =====
  if (params_.return_to_stow) {
    state_ = State::PLAN_RETURN_READY;
    auto lift_end = trajectoryEndState(q_grasp_state, chain.lift_traj);
    if (!planToJointTarget(lift_end, params_.carry_stow_joints, "return_stow",
                           chain.ready_traj)) {
      RCLCPP_ERROR(this->get_logger(), "ABORT: return stow 规划失败");
      last_pick_error_ = "stow planning failed";
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "return_stow 规划成功, 轨迹点数=%zu",
                chain.ready_traj.joint_trajectory.points.size());
    return true;
  }
  if (!params_.return_to_ready) {
    RCLCPP_INFO(this->get_logger(),
                "return_to_ready=false: 保持 lift 姿态搬运 (不折臂回 ready)");
    return true;
  }
  {
    state_ = State::PLAN_RETURN_READY;
    auto lift_end = trajectoryEndState(q_grasp_state, chain.lift_traj);
    if (!planToNamed(lift_end, params_.named_ready, "return_ready", chain.ready_traj)) {
      RCLCPP_ERROR(this->get_logger(), "ABORT: return ready 规划失败");
      last_pick_error_ = "ready planning failed";
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "return_ready 规划成功, 轨迹点数=%zu",
                chain.ready_traj.joint_trajectory.points.size());
  }
  return true;
}

bool PickNode::planToJointTarget(moveit::core::RobotStatePtr start,
                                 const std::vector<double> & joint_positions,
                                 const std::string & label,
                                 moveit_msgs::msg::RobotTrajectory & out_traj)
{
  mgi_->setStartState(*start);
  mgi_->setJointValueTarget(joint_positions);
  mgi_->setPlanningTime(5.0);
  mgi_->setMaxVelocityScalingFactor(params_.velocity_scaling);
  mgi_->setMaxAccelerationScalingFactor(params_.acceleration_scaling);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto err = planWithTimeout(plan, label);
  if (!err) {
    return false;
  }
  if (plan.trajectory_.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "%s 返回空轨迹", label.c_str());
    return false;
  }
  out_traj = plan.trajectory_;
  return true;
}

bool PickNode::runIKContinuityDiagnostic(moveit::core::RobotStatePtr start)
{
  RCLCPP_INFO(this->get_logger(),
              "=== 诊断: 原 setPoseTarget 方式 pre_grasp→grasp 逐 waypoint 连续 IK ===");
  moveit_msgs::msg::RobotTrajectory pre_traj;
  if (!planToPose(start, pre_grasp_pose_, pre_traj)) {
    RCLCPP_ERROR(this->get_logger(), "诊断: 原方式 pre_grasp 规划失败, 无法复现 IK branch");
    return false;
  }
  auto pre_end = trajectoryEndState(start, pre_traj);
  const auto * jmg = pre_end->getJointModelGroup(params_.planning_group);
  if (!jmg) {
    RCLCPP_ERROR(this->get_logger(), "诊断: 规划组不存在");
    return false;
  }
  {
    std::vector<double> q;
    pre_end->copyJointGroupPositions(jmg, q);
    std::ostringstream oss;
    oss.precision(4);
    for (size_t i = 0; i < q.size(); ++i) {
      if (i) {
        oss << ", ";
      }
      oss << q[i];
    }
    RCLCPP_INFO(this->get_logger(), "诊断: pre_grasp 终态 joint=[%s]", oss.str().c_str());
  }

  const Eigen::Vector3d approach_delta(grasp_pose_.position.x-pre_grasp_pose_.position.x,
    grasp_pose_.position.y-pre_grasp_pose_.position.y, grasp_pose_.position.z-pre_grasp_pose_.position.z);
  const double approach_length = approach_delta.norm();
  const auto approach_axis = approach_delta.normalized();
  const int n_steps =
    std::max(1, static_cast<int>(std::ceil(approach_length / params_.cartesian_step)));
  RCLCPP_INFO(this->get_logger(),
              "诊断: 沿 approach_axis 离散 %d 段 (step=%.3f m, 共 %d 个 waypoint)", n_steps,
              params_.cartesian_step, n_steps + 1);

  moveit::core::RobotState seed(mgi_->getRobotModel());
  seed = *pre_end;
  int last_good = -1;
  for (int i = 0; i <= n_steps; ++i) {
    const double t = std::min(approach_length, static_cast<double>(i) * params_.cartesian_step);
    geometry_msgs::msg::Pose wp;
    wp.position.x = pre_grasp_pose_.position.x + approach_axis.x() * t;
    wp.position.y = pre_grasp_pose_.position.y + approach_axis.y() * t;
    wp.position.z = pre_grasp_pose_.position.z + approach_axis.z() * t;
    wp.orientation = pre_grasp_pose_.orientation;
    moveit::core::RobotState ik(mgi_->getRobotModel());
    ik = seed;
    const bool ok = ik.setFromIK(jmg, wp, params_.eef_link, 0.1) && ik.satisfiesBounds(jmg);
    if (ok) {
      ik.update();
      seed = ik;
      last_good = i;
      RCLCPP_INFO(this->get_logger(), "  waypoint %2d  xyz=(%.3f, %.3f, %.3f)  IK SUCCESS", i,
                  wp.position.x, wp.position.y, wp.position.z);
    } else {
      RCLCPP_INFO(this->get_logger(), "  waypoint %2d  xyz=(%.3f, %.3f, %.3f)  IK FAILED", i,
                  wp.position.x, wp.position.y, wp.position.z);
      break;
    }
  }
  std::vector<double> q_last;
  seed.copyJointGroupPositions(jmg, q_last);
  std::ostringstream oss;
  oss.precision(4);
  for (size_t i = 0; i < q_last.size(); ++i) {
    if (i) {
      oss << ", ";
    }
    oss << q_last[i];
  }
  if (last_good == n_steps) {
    RCLCPP_INFO(this->get_logger(), "诊断: 全部 %d 个 waypoint IK 连续 (未丢失); 终点 joint=[%s]",
                n_steps + 1, oss.str().c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "诊断: 丢失连续 IK 于 waypoint %d/%d; 最后成功 joint=[%s]",
                last_good + 1, n_steps + 1, oss.str().c_str());
  }
  return true;
}

bool PickNode::runLiftReachabilityDiagnostic() const
{
  RCLCPP_INFO(this->get_logger(), "=== 诊断: grasp→lift 沿 lift_axis 逐 waypoint 连续 IK ===");
  if (q_grasp_.empty()) {
    RCLCPP_WARN(this->get_logger(), "诊断: q_grasp 未就绪, 跳过");
    return false;
  }
  const auto * jmg = mgi_->getRobotModel()->getJointModelGroup(params_.planning_group);
  if (!jmg) {
    RCLCPP_WARN(this->get_logger(), "诊断: 规划组不存在");
    return false;
  }
  moveit::core::RobotState seed(mgi_->getRobotModel());
  seed.setToDefaultValues();
  seed.setJointGroupPositions(jmg, q_grasp_);
  seed.update();

  const Eigen::Vector3d lift_delta(lift_pose_.position.x-grasp_pose_.position.x,
    lift_pose_.position.y-grasp_pose_.position.y, lift_pose_.position.z-grasp_pose_.position.z);
  const double total = lift_delta.norm();
  const auto lift_axis = lift_delta.normalized();
  const int n_steps = std::max(1, static_cast<int>(std::ceil(total / params_.cartesian_step)));
  RCLCPP_INFO(this->get_logger(),
              "诊断: 沿 lift_axis 离散 %d 段 (step=%.3f, 总=%.3f m, 共 %d 个 waypoint)", n_steps,
              params_.cartesian_step, total, n_steps + 1);

  int last_good = -1;
  for (int i = 0; i <= n_steps; ++i) {
    const double t = std::min(total, static_cast<double>(i) * params_.cartesian_step);
    geometry_msgs::msg::Pose wp;
    wp.position.x = grasp_pose_.position.x + lift_axis.x() * t;
    wp.position.y = grasp_pose_.position.y + lift_axis.y() * t;
    wp.position.z = grasp_pose_.position.z + lift_axis.z() * t;
    wp.orientation = grasp_pose_.orientation;
    moveit::core::RobotState ik(mgi_->getRobotModel());
    ik = seed;
    const bool ik_ok = ik.setFromIK(jmg, wp, params_.eef_link, 0.1);
    const bool in_bounds = ik.satisfiesBounds(jmg);
    if (ik_ok && in_bounds) {
      seed = ik;
      last_good = i;
      RCLCPP_INFO(this->get_logger(), "  waypoint %2d  z=%.3f  IK SUCCESS", i, wp.position.z);
    } else {
      RCLCPP_INFO(this->get_logger(), "  waypoint %2d  z=%.3f  IK FAILED (%s)", i, wp.position.z,
                  ik_ok ? "越界" : "无解");
      break;
    }
  }

  std::vector<double> q;
  seed.copyJointGroupPositions(jmg, q);
  std::ostringstream oss;
  oss.precision(4);
  for (size_t i = 0; i < q.size(); ++i) {
    if (i) {
      oss << ", ";
    }
    oss << q[i];
  }
  RCLCPP_INFO(this->get_logger(),
              "诊断: 最后成功 waypoint=%d/%d, 最大连续 lift=%.3f m; joint=[%s]", last_good,
              n_steps, static_cast<double>(std::max(0, last_good)) * params_.cartesian_step,
              oss.str().c_str());

  const auto & jmodels = jmg->getActiveJointModels();
  for (size_t i = 0; i < jmodels.size() && i < q.size(); ++i) {
    const auto & b = jmodels[i]->getVariableBounds()[0];
    if (!b.position_bounded_) {
      continue;
    }
    const double range = b.max_position_ - b.min_position_;
    const double frac =
      (range > 1e-9) ? (q[i] - b.min_position_) / range : 0.0;
    RCLCPP_INFO(this->get_logger(), "  joint[%s] pos=%.4f bounds=[%.3f, %.3f] at %.0f%%",
                jmodels[i]->getName().c_str(), q[i], b.min_position_, b.max_position_,
                frac * 100.0);
  }
  return true;
}

bool PickNode::validateStateAgainst(const moveit::core::RobotState & state,
                                    const std::vector<double> & expected,
                                    const std::string & label) const
{
  const auto * jmg = state.getJointModelGroup(params_.planning_group);
  std::vector<double> q;
  state.copyJointGroupPositions(jmg, q);
  double max_delta = 0.0;
  for (size_t i = 0; i < q.size() && i < expected.size(); ++i) {
    max_delta = std::max(max_delta, std::fabs(q[i] - expected[i]));
  }
  constexpr double kTol = 0.05;  // 约 3°
  RCLCPP_INFO(this->get_logger(), "validate %-9s: max joint delta=%.4f rad (tol=%.4f)",
              label.c_str(), max_delta, kTol);
  if (max_delta > kTol) {
    RCLCPP_ERROR(this->get_logger(),
                 "ABORT: %s 后真实状态偏离预期 >%.4f rad, 拒绝继续执行", label.c_str(), kTol);
    return false;
  }
  return true;
}

bool PickNode::readRealState(const std::string & label, moveit::core::RobotStatePtr & out_state)
{
  auto cur = mgi_->getCurrentState(1.0);
  if (!cur) {
    RCLCPP_ERROR(this->get_logger(), "real state '%s': getCurrentState 返回空", label.c_str());
    return false;
  }
  auto state = std::make_shared<moveit::core::RobotState>(*cur);

  // CurrentStateMonitor 缓存的状态 link transforms 可能是 dirty, 任何 FK 读取前必须显式 update()
  state->update();

  // 校验 arm joint1~6 存在且为有限数值
  const auto * jmg = state->getJointModelGroup(params_.planning_group);
  if (!jmg) {
    RCLCPP_ERROR(this->get_logger(), "real state '%s': 规划组 '%s' 不存在", label.c_str(),
                 params_.planning_group.c_str());
    return false;
  }
  std::vector<double> q;
  state->copyJointGroupPositions(jmg, q);
  const auto & vnames = jmg->getVariableNames();
  for (size_t i = 0; i < q.size(); ++i) {
    if (!std::isfinite(q[i])) {
      RCLCPP_ERROR(this->get_logger(), "real state '%s': 关节 '%s' 位置非有限 (%.4f)",
                   label.c_str(), vnames[i].c_str(), q[i]);
      return false;
    }
  }
  if (q.size() < 6) {
    RCLCPP_WARN(this->get_logger(), "real state '%s': 规划组关节数=%zu (预期 6)", label.c_str(),
                q.size());
  }

  logRealState(label, *state);
  robot_state_ = state;
  out_state = state;
  return true;
}

bool PickNode::waitForValidJointState(double timeout_s)
{
  const std::string topic = this->get_node_topics_interface()->resolve_topic_name("joint_states");
  joint_wait_valid_ = false;
  RCLCPP_INFO(this->get_logger(), "等待有效 joint state on '%s' (%.0fs)...", topic.c_str(),
              timeout_s);
  auto sub = this->create_subscription<sensor_msgs::msg::JointState>(
    topic, 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      // 需要 joint1..joint6 均存在且为有限数值
      bool ok = true;
      for (int j = 1; j <= 6; ++j) {
        const std::string jn = "joint" + std::to_string(j);
        bool found = false;
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (msg->name[i] == jn && i < msg->position.size() && std::isfinite(msg->position[i])) {
            found = true;
            break;
          }
        }
        if (!found) {
          ok = false;
          break;
        }
      }
      if (ok) {
        std::lock_guard<std::mutex> lock(joint_wait_mutex_);
        joint_wait_valid_ = true;
      }
      joint_wait_cv_.notify_all();
    });

  std::unique_lock<std::mutex> lock(joint_wait_mutex_);
  const bool ready = joint_wait_cv_.wait_for(lock, std::chrono::duration<double>(timeout_s),
                                             [this] { return joint_wait_valid_; });
  if (!ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "ABORT: %.0fs 内未收到含 joint1~6 的有效 joint state (topic '%s'), "
                 "检查真机反馈是否发布",
                 timeout_s, topic.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "有效 joint state 已就绪 (joint1~6 有限数值)");
  return true;
}

bool PickNode::executeTrajectory(const moveit_msgs::msg::RobotTrajectory & traj,
                                 const std::string & label)
{
  if (traj.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "%s 轨迹为空, 拒绝执行", label.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(),
              "execute %s: points=%zu duration=%.2fs max_velocity=%.4f rad/s "
              "max_accel=%.4f rad/s^2",
              label.c_str(), traj.joint_trajectory.points.size(), trajectoryDuration(traj),
              maxTrajectoryVelocity(traj), maxTrajectoryAcceleration(traj));
  const auto err = mgi_->execute(traj);
  if (err != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "%s 执行失败: %s", label.c_str(),
                 error_code_to_string(err).c_str());
    return false;
  }
  // 执行完成后短暂等待, 让真实反馈追上指令, 避免用滞后状态规划下一段
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  RCLCPP_INFO(this->get_logger(), "%s 执行成功", label.c_str());
  return true;
}

bool PickNode::openGripper()
{
  RCLCPP_INFO(this->get_logger(), "gripper: 开启 (width=%.4f, force=%.2f)",
              params_.gripper_open_width, params_.gripper_force);
  sendGripperCommand(params_.gripper_open_width, params_.gripper_force);
  double achieved = 0.0;
  if (!waitForGripperTarget(params_.gripper_open_width, params_.gripper_width_tolerance,
                            params_.gripper_timeout, achieved)) {
    RCLCPP_ERROR(this->get_logger(), "gripper 开启等待超时/失败, 当前 width=%.4f", achieved);
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "gripper 开启确认 width=%.4f", achieved);
  return true;
}

bool PickNode::verifyHeld()
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  std::optional<std::chrono::steady_clock::time_point> stable_since;
  std::unique_lock<std::mutex> lock(status_mutex_);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    bool valid = false;
    if (latest_gripper_status_ && last_gripper_status_time_) {
      const auto & s = *latest_gripper_status_;
      valid = (this->now()-*last_gripper_status_time_).seconds() <= 0.5 &&
        s.driver_enable_status && !s.driver_error_status && !s.sensor_status &&
        !s.voltage_too_low && !s.motor_overheating && !s.driver_overcurrent &&
        !s.driver_overheating && std::isfinite(s.width) &&
        s.width >= params_.gripper_grasp_width_min &&
        s.width <= params_.gripper_grasp_width_threshold;
    }
    if (valid) {
      if (!stable_since) {stable_since = now;}
      if (now-*stable_since >= std::chrono::milliseconds(400)) {return true;}
    } else {
      stable_since.reset();
    }
    status_cv_.wait_for(lock, std::chrono::milliseconds(100));
  }
  RCLCPP_ERROR(this->get_logger(), "持续夹持检查失败: 宽度异常或反馈过期");
  return false;
}

bool PickNode::closeGripper()
{
  RCLCPP_INFO(this->get_logger(), "gripper: 闭合 (width=%.4f, force=%.2f)",
              params_.gripper_close_width, params_.gripper_force);
  sendGripperCommand(params_.gripper_close_width, params_.gripper_force);
  double achieved = 0.0;
  // 夹住物体时 width 无法合到 0 (物体厚度), 判定为"夹住"即成功:
  // 闭合容差 = max(width_tolerance, grasp_width_threshold)
  double close_tol = params_.gripper_width_tolerance;
  if (params_.gripper_grasp_width_threshold > close_tol) {
    close_tol = params_.gripper_grasp_width_threshold;
  }
  if (!waitForGripperTarget(params_.gripper_close_width, close_tol,
                            params_.gripper_timeout, achieved)) {
    RCLCPP_ERROR(this->get_logger(), "gripper 闭合等待超时/失败, 当前 width=%.4f", achieved);
    return false;
  }
  // 等待夹爪停稳后再读最终宽度: waitForGripperTarget 在宽度"首次进入"容差带时就返回,
  // 可能取到关闭过程中的瞬时值。停稳后的最终宽度才能区分"夹住物体"(≈物体尺寸)
  // 与"空抓完全合拢"(≈0)。
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  double settled = achieved;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (latest_gripper_status_) {
      settled = latest_gripper_status_->width;
    }
  }
  // 空抓判定: 两指完全合拢到接近 0 说明指间没有任何物体, 即使 <= threshold 也不算夹住。
  if (settled < params_.gripper_grasp_width_min) {
    RCLCPP_ERROR(this->get_logger(),
                 "gripper 最终 width=%.4f < 最小夹持宽度 %.4f, 判定空抓失败",
                 settled, params_.gripper_grasp_width_min);
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "gripper 闭合确认 width=%.4f (夹住物体, 判定成功)", settled);
  return true;
}

void PickNode::logRealState(const std::string & label, moveit::core::RobotState & state) const
{
  // 任何 getGlobalLinkTransform 读取前确保 link transforms 有效
  state.update();

  const auto * jmg = state.getJointModelGroup(params_.planning_group);
  std::vector<double> q;
  state.copyJointGroupPositions(jmg, q);
  std::ostringstream oss;
  oss.precision(4);
  for (size_t i = 0; i < q.size(); ++i) {
    if (i) {
      oss << ", ";
    }
    oss << q[i];
  }
  const auto * link_model = state.getLinkModel(params_.eef_link);
  Eigen::Isometry3d tcp;
  if (link_model) {
    tcp = state.getGlobalLinkTransform(link_model);
  } else {
    RCLCPP_WARN(this->get_logger(), "%s: eef link '%s' 不在模型中, 无法打印 TCP 位姿", label.c_str(),
                params_.eef_link.c_str());
    tcp = Eigen::Isometry3d::Identity();
  }
  RCLCPP_INFO(this->get_logger(), "real state after %-9s: joint_positions=[%s]", label.c_str(),
              oss.str().c_str());
  RCLCPP_INFO(this->get_logger(), "  %s -> base_link: (%.4f, %.4f, %.4f)", params_.eef_link.c_str(),
              tcp.translation().x(), tcp.translation().y(), tcp.translation().z());
}

// ==================== 参数 ====================

bool PickNode::loadParameters()
{
  params_.planning_group = this->declare_parameter<std::string>("planning_group", "arm");
  params_.eef_link = this->declare_parameter<std::string>("eef_link", "gripper_tcp");
  params_.planning_frame = this->declare_parameter<std::string>("planning_frame", "base_link");
  params_.grasp_min_z = this->declare_parameter<double>("grasp_min_z", -10.0);
  params_.floor_pick_enabled = this->declare_parameter<bool>("floor_pick_enabled", false);
  params_.floor_threshold_z = this->declare_parameter<double>("floor_threshold_z", 0.05);
  params_.floor_lift_distance = this->declare_parameter<double>("floor_lift_distance", 0.25);
  params_.floor_grasp_z = this->declare_parameter<double>("floor_grasp_z", -0.02);
  params_.named_ready = this->declare_parameter<std::string>("named_ready", "ready");
  params_.return_to_ready = this->declare_parameter<bool>("return_to_ready", true);
  params_.return_to_stow = this->declare_parameter<bool>("return_to_stow", false);
  params_.carry_stow_joints =
    this->declare_parameter<std::vector<double>>("carry_stow_joints", std::vector<double>{});
  params_.recovery_view_joints = this->declare_parameter<std::vector<double>>(
    "recovery_view_joints", std::vector<double>{0.0, 0.4, -0.5, 0.0, 1.15, 0.0});
  if (params_.recovery_view_joints.size() != 6) {
    RCLCPP_ERROR(this->get_logger(), "recovery_view_joints requires six joints");
    return false;
  }
  if (params_.return_to_stow && params_.carry_stow_joints.size() != 6) {
    RCLCPP_ERROR(this->get_logger(),
                 "return_to_stow=true 但 carry_stow_joints 有 %zu 个值 (需要 6 个)",
                 params_.carry_stow_joints.size());
    return false;
  }
  params_.execute = this->declare_parameter<bool>("execute", false);
  params_.allow_real_execution = this->declare_parameter<bool>("allow_real_execution", false);
  params_.target_topic = this->declare_parameter<std::string>("target_topic", "/grasp_target_pose");
  params_.execute_service =
    this->declare_parameter<std::string>("execute_service", "/execute_pick");

  params_.object_pos.x = this->declare_parameter<double>("object_pose.x", 0.40);
  params_.object_pos.y = this->declare_parameter<double>("object_pose.y", 0.0);
  params_.object_pos.z = this->declare_parameter<double>("object_pose.z", 0.10);

  params_.object_size.x = this->declare_parameter<double>("object_size.x", 0.05);
  params_.object_size.y = this->declare_parameter<double>("object_size.y", 0.05);
  params_.object_size.z = this->declare_parameter<double>("object_size.z", 0.05);

  params_.grasp_offset.x = this->declare_parameter<double>("grasp_offset.x", 0.0);
  params_.grasp_offset.y = this->declare_parameter<double>("grasp_offset.y", 0.0);
  params_.grasp_offset.z = this->declare_parameter<double>("grasp_offset.z", 0.0);

  params_.grasp_orientation.x = this->declare_parameter<double>("grasp_orientation.x", 0.0);
  params_.grasp_orientation.y = this->declare_parameter<double>("grasp_orientation.y", 1.0);
  params_.grasp_orientation.z = this->declare_parameter<double>("grasp_orientation.z", 0.0);
  params_.grasp_orientation.w = this->declare_parameter<double>("grasp_orientation.w", 0.0);

  {
    const auto axis = this->declare_parameter<std::vector<double>>("approach_axis", {0.0, 0.0, -1.0});
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
    const auto axis = this->declare_parameter<std::vector<double>>("lift_axis", {0.0, 0.0, 1.0});
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
  params_.lift_distance = this->declare_parameter<double>("lift_distance", 0.10);

  params_.cartesian_step = this->declare_parameter<double>("cartesian_step", 0.01);
  params_.min_cartesian_fraction = this->declare_parameter<double>("min_cartesian_fraction", 0.99);
  params_.velocity_scaling = this->declare_parameter<double>("velocity_scaling", 0.10);
  params_.acceleration_scaling = this->declare_parameter<double>("acceleration_scaling", 0.10);

  params_.gripper_open_width = this->declare_parameter<double>("gripper.open_width", 0.08);
  params_.gripper_close_width = this->declare_parameter<double>("gripper.close_width", 0.0);
  params_.gripper_force = this->declare_parameter<double>("gripper.force", 1.0);
  params_.gripper_timeout = this->declare_parameter<double>("gripper.command_timeout", 5.0);
  params_.gripper_width_tolerance = this->declare_parameter<double>("gripper.width_tolerance", 0.005);
  params_.gripper_grasp_width_threshold =
      this->declare_parameter<double>("gripper.grasp_width_threshold", 0.05);
  params_.gripper_grasp_width_min =
      this->declare_parameter<double>("gripper.grasp_width_min", 0.0);

  // quaternion 归一化
  const double n = quatNorm(params_.grasp_orientation);
  if (n < kMinQuaternionNorm) {
    RCLCPP_ERROR(this->get_logger(), "grasp_orientation 模长接近 0, 无法归一化");
    return false;
  }
  params_.grasp_orientation = normalizeQuat(params_.grasp_orientation);

  RCLCPP_INFO(this->get_logger(), "planning_frame=%s eef_link=%s group=%s execute=%s",
              params_.planning_frame.c_str(), params_.eef_link.c_str(),
              params_.planning_group.c_str(), params_.execute ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "grasp_orientation (w,x,y,z)=(%.4f,%.4f,%.4f,%.4f)",
              params_.grasp_orientation.w, params_.grasp_orientation.x,
              params_.grasp_orientation.y, params_.grasp_orientation.z);
  return true;
}

// ==================== Pose ====================

bool PickNode::computePoses(std::optional<geometry_msgs::msg::Point> task_target_position)
{
  // object_pose = 本次任务目标位置 (service 触发快照传入) 或 YAML object_pose (启动/standalone 遗留)。
  // 动态任务输入 (active_pick_target) 与静态抓取策略 (grasp_orientation/offset/axis/distance) 分离,
  // 任务目标位置不再经由 params_.object_pos 进入规划链。
  object_pose_.position = task_target_position.value_or(params_.object_pos);
  object_pose_.orientation.w = 1.0;

  // grasp_pose = object_pose + grasp_offset, 姿态 = grasp_orientation
  grasp_pose_.position.x = object_pose_.position.x + params_.grasp_offset.x;
  grasp_pose_.position.y = object_pose_.position.y + params_.grasp_offset.y;
  grasp_pose_.position.z = object_pose_.position.z + params_.grasp_offset.z;
  grasp_pose_.position.z = std::max(grasp_pose_.position.z, params_.grasp_min_z);
  grasp_pose_.orientation = params_.grasp_orientation;

  // pre_grasp_pose = grasp - approach_axis * approach_distance (向量计算)
  pre_grasp_pose_.position.x = grasp_pose_.position.x - params_.approach_axis.x() * params_.approach_distance;
  pre_grasp_pose_.position.y = grasp_pose_.position.y - params_.approach_axis.y() * params_.approach_distance;
  pre_grasp_pose_.position.z = grasp_pose_.position.z - params_.approach_axis.z() * params_.approach_distance;
  pre_grasp_pose_.orientation = grasp_pose_.orientation;

  // lift_pose = grasp + lift_axis * lift_distance, 姿态保持 grasp
  lift_pose_.position.x = grasp_pose_.position.x + params_.lift_axis.x() * params_.lift_distance;
  lift_pose_.position.y = grasp_pose_.position.y + params_.lift_axis.y() * params_.lift_distance;
  lift_pose_.position.z = grasp_pose_.position.z + params_.lift_axis.z() * params_.lift_distance;
  if (params_.floor_pick_enabled && object_pose_.position.z < params_.floor_threshold_z) {
    grasp_pose_.position.z = params_.floor_grasp_z;
    // Keep the calibrated pinch orientation; approach from above instead of
    // retracting the wrist toward the chassis near the floor. Lift clear of
    // the chassis before the normal collision-checked carry stow.
    pre_grasp_pose_ = grasp_pose_;
    pre_grasp_pose_.position.z += params_.approach_distance;
    lift_pose_ = grasp_pose_;
    lift_pose_.position.z += params_.floor_lift_distance;
    RCLCPP_INFO(this->get_logger(), "FLOOR PICK: vertical approach and %.2f m clearance lift",
                params_.floor_lift_distance);
  }
  lift_pose_.orientation = grasp_pose_.orientation;

  RCLCPP_INFO(this->get_logger(), "object    = (%.4f, %.4f, %.4f)",
              object_pose_.position.x, object_pose_.position.y, object_pose_.position.z);
  RCLCPP_INFO(this->get_logger(), "grasp     = (%.4f, %.4f, %.4f)",
              grasp_pose_.position.x, grasp_pose_.position.y, grasp_pose_.position.z);
  RCLCPP_INFO(this->get_logger(), "pre_grasp = (%.4f, %.4f, %.4f)",
              pre_grasp_pose_.position.x, pre_grasp_pose_.position.y, pre_grasp_pose_.position.z);
  RCLCPP_INFO(this->get_logger(), "lift      = (%.4f, %.4f, %.4f)",
              lift_pose_.position.x, lift_pose_.position.y, lift_pose_.position.z);
  return true;
}

// ==================== Marker ====================

void PickNode::publishMarkers() const
{
  // 与 computePoses (触发线程) 互斥, 避免读到半更新位姿
  std::lock_guard<std::mutex> lock(pose_mutex_);
  visualization_msgs::msg::MarkerArray markers;

  // ID 0: object cube
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = params_.planning_frame;
    m.header.stamp = rclcpp::Time(0);
    m.ns = marker_ns_;
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = object_pose_.position;
    m.pose.orientation.w = 1.0;
    m.scale.x = params_.object_size.x;
    m.scale.y = params_.object_size.y;
    m.scale.z = params_.object_size.z;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 0.5;
    m.lifetime = rclcpp::Duration(0, 0);
    markers.markers.push_back(m);
  }

  // ID 1-3: pre_grasp(黄) / grasp(绿) / lift(蓝) spheres
  struct SphereSpec
  {
    int id;
    const geometry_msgs::msg::Point * pos;
    std::array<double, 3> color;
  };
  for (const auto & spec : std::vector<SphereSpec>{
         {1, &pre_grasp_pose_.position, {1.0, 1.0, 0.0}},
         {2, &grasp_pose_.position, {0.0, 1.0, 0.0}},
         {3, &lift_pose_.position, {0.0, 0.0, 1.0}},
       })
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = params_.planning_frame;
    m.header.stamp = rclcpp::Time(0);
    m.ns = marker_ns_;
    m.id = spec.id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = *spec.pos;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.03;
    m.color.r = spec.color[0];
    m.color.g = spec.color[1];
    m.color.b = spec.color[2];
    m.color.a = 1.0;
    m.lifetime = rclcpp::Duration(0, 0);
    markers.markers.push_back(m);
  }

  // ID 4: approach line pre_grasp -> grasp (黄)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = params_.planning_frame;
    m.header.stamp = rclcpp::Time(0);
    m.ns = marker_ns_;
    m.id = 4;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.005;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    m.pose.orientation.w = 1.0;
    m.points = {pre_grasp_pose_.position, grasp_pose_.position};
    m.lifetime = rclcpp::Duration(0, 0);
    markers.markers.push_back(m);
  }
  // ID 5: lift line grasp -> lift (蓝)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = params_.planning_frame;
    m.header.stamp = rclcpp::Time(0);
    m.ns = marker_ns_;
    m.id = 5;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.005;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a = 1.0;
    m.pose.orientation.w = 1.0;
    m.points = {grasp_pose_.position, lift_pose_.position};
    m.lifetime = rclcpp::Duration(0, 0);
    markers.markers.push_back(m);
  }

  // ID 6,7: grasp 处 TCP +Z / +Y 方向箭头
  {
    const Eigen::Isometry3d grasp_transform = Eigen::Translation3d(
      Eigen::Vector3d(grasp_pose_.position.x, grasp_pose_.position.y, grasp_pose_.position.z)) *
      Eigen::Quaterniond(grasp_pose_.orientation.w, grasp_pose_.orientation.x,
                         grasp_pose_.orientation.y, grasp_pose_.orientation.z);
    const Eigen::Vector3d tcp_z = grasp_transform.linear() * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d tcp_y = grasp_transform.linear() * Eigen::Vector3d::UnitY();
    const double arrow_len = 0.08;

    // ID 6: TCP +Z (白)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = params_.planning_frame;
      m.header.stamp = rclcpp::Time(0);
      m.ns = marker_ns_;
      m.id = 6;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.008;
      m.scale.y = 0.02;
      m.scale.z = 0.02;
      m.color.r = 1.0;
      m.color.g = 1.0;
      m.color.b = 1.0;
      m.color.a = 1.0;
      m.pose.orientation.w = 1.0;
      geometry_msgs::msg::Point s = grasp_pose_.position;
      geometry_msgs::msg::Point e = grasp_pose_.position;
      e.x += tcp_z.x() * arrow_len;
      e.y += tcp_z.y() * arrow_len;
      e.z += tcp_z.z() * arrow_len;
      m.points = {s, e};
      m.lifetime = rclcpp::Duration(0, 0);
      markers.markers.push_back(m);
    }
    // ID 7: TCP +Y (绿)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = params_.planning_frame;
      m.header.stamp = rclcpp::Time(0);
      m.ns = marker_ns_;
      m.id = 7;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.008;
      m.scale.y = 0.02;
      m.scale.z = 0.02;
      m.color.r = 0.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      m.pose.orientation.w = 1.0;
      geometry_msgs::msg::Point s = grasp_pose_.position;
      geometry_msgs::msg::Point e = grasp_pose_.position;
      e.x += tcp_y.x() * arrow_len;
      e.y += tcp_y.y() * arrow_len;
      e.z += tcp_y.z() * arrow_len;
      m.points = {s, e};
      m.lifetime = rclcpp::Duration(0, 0);
      markers.markers.push_back(m);
    }
  }

  marker_pub_->publish(markers);
  RCLCPP_INFO(this->get_logger(), "MarkerArray 已发布 (id 0-7)");
}

void PickNode::startMarkerRepublisher()
{
  // RViz 可能晚于本节点启动。周期重发 marker (1Hz), 让迟订阅的 RViz 能收到。
  // volatile QoS 下消息不持久, 只能靠持续重发; 节点存活期间一直重发。
  marker_repub_thread_ = std::thread([this]() {
    while (rclcpp::ok() && !stop_marker_repub_.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      if (!stop_marker_repub_.load() && marker_pub_) {
        publishMarkers();
      }
    }
  });
}

// ==================== 规划 ====================

bool PickNode::planToPose(moveit::core::RobotStatePtr start,
                          const geometry_msgs::msg::Pose & pose,
                          moveit_msgs::msg::RobotTrajectory & out_traj)
{
  mgi_->setStartState(*start);
  mgi_->setPoseTarget(pose, params_.eef_link);
  mgi_->setPlanningTime(5.0);
  mgi_->setMaxVelocityScalingFactor(params_.velocity_scaling);
  mgi_->setMaxAccelerationScalingFactor(params_.acceleration_scaling);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto err = planWithTimeout(plan, "planToPose");
  if (!err) {
    return false;
  }
  if (plan.trajectory_.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "planToPose 返回空轨迹");
    return false;
  }
  pre_grasp_plan_point_count_ = static_cast<int>(plan.trajectory_.joint_trajectory.points.size());
  RCLCPP_INFO(this->get_logger(), "planToPose 成功, 轨迹点数=%d", pre_grasp_plan_point_count_);
  out_traj = plan.trajectory_;
  return true;
}

bool PickNode::planToNamed(moveit::core::RobotStatePtr start,
                           const std::string & name,
                           const std::string & label,
                           moveit_msgs::msg::RobotTrajectory & out_traj) const
{
  mgi_->setStartState(*start);
  mgi_->setNamedTarget(name);
  mgi_->setPlanningTime(5.0);
  mgi_->setMaxVelocityScalingFactor(params_.velocity_scaling);
  mgi_->setMaxAccelerationScalingFactor(params_.acceleration_scaling);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto err = planWithTimeout(plan, "planToNamed");
  if (!err) {
    return false;
  }
  if (plan.trajectory_.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "planToNamed('%s') 返回空轨迹", name.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "%s ('%s') 规划成功, 轨迹点数=%zu", label.c_str(), name.c_str(),
              plan.trajectory_.joint_trajectory.points.size());
  out_traj = plan.trajectory_;
  return true;
}

moveit::core::MoveItErrorCode PickNode::planWithTimeout(
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const std::string & label) const
{
  // move_group 端 KDL 规划偶尔会卡死 (hrtimer_nanosleep 不返回)。
  // 把同步 plan() 放进独立 async 线程, 主线程用 future 超时轮询, 超时即 ABORT。
  // 超时返回后进程随即退出, 残留的 async 线程被 OS 一并终止, 无泄漏。
  constexpr double kPlanWallTimeout = 15.0;
  auto fut = std::async(std::launch::async, [this]() {
    moveit::planning_interface::MoveGroupInterface::Plan local_plan;
    const auto err = mgi_->plan(local_plan);
    return std::make_pair(err, local_plan);
  });

  if (fut.wait_for(std::chrono::duration<double>(kPlanWallTimeout)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "%s 超时 (%0.fs), ABORT", label.c_str(), kPlanWallTimeout);
    return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT);
  }
  auto [err, local_plan] = fut.get();
  plan = std::move(local_plan);
  if (err != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "%s failed: %s", label.c_str(), error_code_to_string(err).c_str());
  }
  return err;
}

bool PickNode::planCartesian(moveit::core::RobotStatePtr start,
                             const std::vector<geometry_msgs::msg::Pose> & waypoints,
                             const std::string & label,
                             moveit_msgs::msg::RobotTrajectory & out_traj,
                             double & fraction)
{
  // GetCartesianPath 服务: 接受显式 start_state 并在 move_group 端做碰撞检查。
  const std::string srv_name = std::string("/compute_cartesian_path");
  if (!client_) {
    client_ = this->create_client<moveit_msgs::srv::GetCartesianPath>(srv_name);
  }
  if (!client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "GetCartesianPath 服务不可用: %s", srv_name.c_str());
    last_pick_error_ = "MoveIt Cartesian service unavailable";
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
  req->jump_threshold = 0.0;  // 禁用关节空间跳变检测
  req->prismatic_jump_threshold = 0.0;
  req->revolute_jump_threshold = 0.0;
  req->avoid_collisions = true;

  auto fut = client_->async_send_request(req);
  const auto status = fut.wait_for(std::chrono::seconds(10));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "GetCartesianPath 服务调用超时");
    last_pick_error_ = "MoveIt Cartesian service timeout";
    return false;
  }
  auto resp = fut.get();
  fraction = resp->fraction;
  const int point_count = static_cast<int>(resp->solution.joint_trajectory.points.size());
  RCLCPP_INFO(this->get_logger(), "Cartesian %-8s fraction=%.4f points=%d (need >= %.2f)",
              label.c_str(), fraction, point_count, params_.min_cartesian_fraction);

  if (resp->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "GetCartesianPath error: %s",
                 error_code_to_string(moveit::core::MoveItErrorCode(resp->error_code)).c_str());
    return false;
  }
  if (fraction < params_.min_cartesian_fraction) {
    std::vector<double> q;
    start->copyJointGroupPositions(start->getJointModelGroup(params_.planning_group), q);
    std::ostringstream oss;
    oss.precision(4);
    for (size_t i = 0; i < q.size(); ++i) {
      if (i) {
        oss << ", ";
      }
      oss << q[i];
    }
    RCLCPP_ERROR(this->get_logger(),
                 "Cartesian %s fraction %.4f < min_cartesian_fraction %.2f, "
                 "ABORT(不执行, 不自动调整). start joint_positions=[%s] error=%s",
                 label.c_str(), fraction, params_.min_cartesian_fraction, oss.str().c_str(),
                 error_code_to_string(moveit::core::MoveItErrorCode(resp->error_code)).c_str());
    return false;
  }
  out_traj = resp->solution;
  return true;
}

// ==================== 诊断 ====================

bool PickNode::checkPoseAxes() const
{
  const Eigen::Isometry3d grasp_transform = Eigen::Translation3d(
    Eigen::Vector3d(grasp_pose_.position.x, grasp_pose_.position.y, grasp_pose_.position.z)) *
    Eigen::Quaterniond(grasp_pose_.orientation.w, grasp_pose_.orientation.x,
                       grasp_pose_.orientation.y, grasp_pose_.orientation.z);
  const Eigen::Vector3d tcp_z = grasp_transform.linear() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d tcp_y = grasp_transform.linear() * Eigen::Vector3d::UnitY();
  RCLCPP_INFO(this->get_logger(), "grasp 处 TCP +Z -> base_link (%.4f, %.4f, %.4f)",
              tcp_z.x(), tcp_z.y(), tcp_z.z());
  RCLCPP_INFO(this->get_logger(), "grasp 处 TCP +Y -> base_link (%.4f, %.4f, %.4f)",
              tcp_y.x(), tcp_y.y(), tcp_y.z());
  return true;
}

bool PickNode::checkGraspApproachAlignment(double & deg_out) const
{
  const Eigen::Isometry3d grasp_transform = Eigen::Translation3d(
    Eigen::Vector3d(grasp_pose_.position.x, grasp_pose_.position.y, grasp_pose_.position.z)) *
    Eigen::Quaterniond(grasp_pose_.orientation.w, grasp_pose_.orientation.x,
                       grasp_pose_.orientation.y, grasp_pose_.orientation.z);
  const Eigen::Vector3d tcp_z = grasp_transform.linear() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d approach(grasp_pose_.position.x-pre_grasp_pose_.position.x,
    grasp_pose_.position.y-pre_grasp_pose_.position.y, grasp_pose_.position.z-pre_grasp_pose_.position.z);
  const double dot = std::clamp(tcp_z.dot(approach.normalized()), -1.0, 1.0);
  const double deg = std::acos(dot) * 180.0 / M_PI;
  deg_out = deg;
  RCLCPP_INFO(this->get_logger(), "诊断: TCP +Z 与 approach_axis 夹角 = %.2f deg (仅提示, 不修改姿态)",
              deg);
  if (deg > 30.0 && !(params_.floor_pick_enabled && object_pose_.position.z < params_.floor_threshold_z)) {
    RCLCPP_WARN(this->get_logger(),
                "WARNING: TCP +Z 与 approach_axis 夹角较大, 请检查 grasp_orientation 与 approach_axis");
  }
  return true;
}

// ==================== 夹爪 ====================

void PickNode::gripperStatusCallback(const scoutfetch_interfaces::msg::GripperStatus::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    latest_gripper_status_ = *msg;
    last_gripper_status_time_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
  }
  status_cv_.notify_all();
}

bool PickNode::waitForGripperTarget(double target_width, double tol, double timeout_s,
                                    double & achieved_width)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_s));
  std::unique_lock<std::mutex> lock(status_mutex_);
  bool reached = false;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (latest_gripper_status_) {
      const auto & s = *latest_gripper_status_;
      if (s.driver_error_status || s.voltage_too_low || s.motor_overheating ||
          s.driver_overcurrent || s.driver_overheating)
      {
        RCLCPP_ERROR(this->get_logger(), "gripper 驱动异常, 停止等待");
        return false;
      }
      const bool fresh = last_gripper_status_time_ &&
        (this->now()-*last_gripper_status_time_).seconds() <= 0.5;
      if (fresh && s.driver_enable_status && !s.sensor_status && std::fabs(s.width - target_width) <= tol) {
        achieved_width = s.width;
        reached = true;
        break;
      }
    }
    if (status_cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
      break;
    }
  }
  if (!reached && latest_gripper_status_) {
    achieved_width = latest_gripper_status_->width;
  }
  return reached;
}

bool PickNode::sendGripperCommand(double width, double force) const
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = {"gripper"};
  msg.position = {width};
  msg.effort = {force};
  gripper_cmd_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "gripper_cmd: width=%.4f force=%.2f", width, force);
  return true;
}

// ==================== 报告 ====================

void PickNode::logFinalReport() const
{
  const geometry_msgs::msg::Point & o = object_pose_.position;
  const geometry_msgs::msg::Point & g = grasp_pose_.position;
  const geometry_msgs::msg::Point & p = pre_grasp_pose_.position;
  const geometry_msgs::msg::Point & l = lift_pose_.position;
  const auto & q = grasp_pose_.orientation;

  auto fmt_vec = [](const std::vector<double> & v) {
    std::ostringstream oss;
    oss.precision(4);
    for (size_t i = 0; i < v.size(); ++i) {
      if (i) {
        oss << ", ";
      }
      oss << v[i];
    }
    return oss.str();
  };

  std::cout
    << "\n========== DRY-RUN REPORT ==========\n"
    << "1.  规划策略:                  grasp-anchored IK branch (不再随机重试 pre_grasp)\n"
    << "2.  piper_control 修改:          NONE (本轮未 touch)\n"
    << "3.  planning_frame / eef_link:  " << params_.planning_frame << " / " << params_.eef_link << "\n"
    << "4.  object / grasp / pre_grasp / lift:\n"
    << "      (" << o.x << ", " << o.y << ", " << o.z << ") / "
    << "(" << g.x << ", " << g.y << ", " << g.z << ") / "
    << "(" << p.x << ", " << p.y << ", " << p.z << ") / "
    << "(" << l.x << ", " << l.y << ", " << l.z << ")\n"
    << "5.  grasp quaternion (w,x,y,z): (" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")\n"
    << "6.  q_grasp (grasp_pose IK):    [" << fmt_vec(q_grasp_) << "]\n"
    << "7.  q_pre_grasp (反向 Cartesian 终点): [" << fmt_vec(q_pre_grasp_) << "]\n"
    << "8.  反向 Cartesian fraction:    " << reverse_approach_fraction_ << " (需 >= 0.99)\n"
    << "9.  pre_grasp Plan (current→q_pre_grasp): "
    << (pre_grasp_plan_point_count_ > 0 ? "SUCCESS" : "FAILED") << ", "
    << pre_grasp_plan_point_count_ << " 点\n"
    << "10. 正向 approach fraction:     " << approach_fraction_ << " (需 >= 0.99)\n"
    << "11. lift fraction:              " << lift_fraction_ << " (需 >= 0.99)\n"
    << "12. return ready Plan:          见上方日志\n"
    << "13. 真机运动 / gripper_cmd:     NONE (dry-run)\n"
    << "14. Marker 发布:                id 0-7 全部发布\n"
    << "15. RViz 重点检查:              object/grasp/pre_grasp/lift 位置; "
    << "ID6 白箭头(TCP+Z) 应朝下(-Z); ID7 绿箭头(TCP+Y) 应朝 +Y\n"
    << "==================================\n";
}

void PickNode::onGraspTarget(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!validateGraspTarget(*msg)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    latest_target_ = *msg;
  }
  RCLCPP_INFO(this->get_logger(),
              "GRASP TARGET STORED\n  frame: %s\n  stamp: %d.%09d\n  x: %.4f y: %.4f z: %.4f",
              msg->header.frame_id.c_str(), msg->header.stamp.sec, msg->header.stamp.nanosec,
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

bool PickNode::validateGraspTarget(const geometry_msgs::msg::PoseStamped & msg) const
{
  if (msg.header.frame_id.empty() || msg.header.frame_id != params_.planning_frame) {
    RCLCPP_WARN(this->get_logger(), "忽略 %s: frame_id='%s' (须为 '%s')",
                params_.target_topic.c_str(), msg.header.frame_id.c_str(),
                params_.planning_frame.c_str());
    return false;
  }
  const auto & p = msg.pose.position;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
    RCLCPP_WARN(this->get_logger(), "忽略 %s: 位置非有限 (%.4f, %.4f, %.4f)",
                params_.target_topic.c_str(), p.x, p.y, p.z);
    return false;
  }
  const double n = quatNorm(msg.pose.orientation);
  if (!std::isfinite(n) || n < kMinQuaternionNorm) {
    RCLCPP_WARN(this->get_logger(), "忽略 %s: 非法 quaternion", params_.target_topic.c_str());
    return false;
  }
  return true;
}

void PickNode::onExecutePick(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  RCLCPP_INFO(this->get_logger(), "EXECUTE PICK REQUEST");

  // ===== 双重安全门禁 (正式、长期保留) =====
  // 门禁 A (execute): false → 始终 dry-run, 与 allow_real_execution 无关。
  // 门禁 B (allow_real_execution): execute=true 时, false → 拒绝真实执行 (不进入 runExecute(),
  //   不产生任何轨迹/夹爪指令); true → 允许进入现有 runExecute() 真机路径。
  // 授权检查集中在 /execute_pick 入口, 不散落到各执行阶段。
  if (params_.execute && !params_.allow_real_execution) {
    response->success = false;
    response->message = "real execution not authorized";
    RCLCPP_WARN(this->get_logger(), "EXECUTE PICK REJECTED: %s", response->message.c_str());
    return;
  }
  const bool real_execution_enabled = params_.execute && params_.allow_real_execution;

  // 情况 2: 无合法目标 → 安全拒绝, 不运行任何规划
  geometry_msgs::msg::PoseStamped active_pick_target;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (!latest_target_) {
      response->success = false;
      response->message = "no grasp target available";
      RCLCPP_WARN(this->get_logger(), "EXECUTE PICK REJECTED: %s", response->message.c_str());
      return;
    }
    // snapshot: 本次 pick 全程使用触发瞬间复制出的目标, 执行中不受新目标影响
    active_pick_target = *latest_target_;
  }

  // 情况 3: 已有 pick 在运行 → 拒绝并行第二次, 不排队不 retry
  if (pick_in_progress_.exchange(true)) {
    response->success = false;
    response->message = "pick already running";
    RCLCPP_WARN(this->get_logger(), "EXECUTE PICK REJECTED: %s", response->message.c_str());
    return;
  }

  // ---- 分支: 真实执行 vs dry-run (仅此入口可达, 无其他自动调用方) ----
  // 无论成功/异常都必须复位 pick_in_progress_, 避免一次异常后永久锁死
  bool ok = false;
  if (real_execution_enabled) {
    try {
      ok = runExecute(active_pick_target);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "real execution exception: %s", e.what());
      last_pick_error_ = "real execution exception";
    }
    pick_in_progress_.store(false);
    if (ok) {
      response->success = true;
      response->message = "pick execution succeeded";
      RCLCPP_INFO(this->get_logger(), "EXECUTE PICK RESPONSE: %s", response->message.c_str());
    } else {
      response->success = false;
      response->message =
        last_pick_error_.empty() ? "real pick execution failed" : last_pick_error_;
      RCLCPP_WARN(this->get_logger(), "EXECUTE PICK RESPONSE: %s", response->message.c_str());
    }
    return;
  }

  try {
    ok = runPickDryRun(active_pick_target);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "dry-run 异常: %s", e.what());
    last_pick_error_ = "dry-run exception";
  }
  pick_in_progress_.store(false);

  if (ok) {
    response->success = true;
    response->message = "dry-run pick planning succeeded";
    RCLCPP_INFO(this->get_logger(), "EXECUTE PICK RESPONSE: %s", response->message.c_str());
  } else {
    response->success = false;
    response->message = last_pick_error_.empty() ? "dry-run pick planning failed" : last_pick_error_;
    RCLCPP_WARN(this->get_logger(), "EXECUTE PICK RESPONSE: %s", response->message.c_str());
  }
}

bool PickNode::runPickDryRun(const geometry_msgs::msg::PoseStamped & active_pick_target)
{
  last_pick_error_.clear();

  // 目标 snapshot → 任务位姿计算; 姿态保持 config grasp_orientation (不自行创造新抓取方向)。
  // 动态任务输入直接传入 computePoses (不写入 params_.object_pos, 不污染静态配置)。
  // computePoses + marker 重发共用 pose_mutex_, 与 marker 重发线程互斥。
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!computePoses(active_pick_target.pose.position)) {
      last_pick_error_ = "pose computation failed";
      return false;
    }
  }
  // 立即重发 marker, 让 RViz 显示本次触发目标
  publishMarkers();

  // 虚拟状态链起点 = 当前真实关节状态 (每次触发重新读取, 底盘/手臂变化后仍正确)
  if (!waitForValidJointState(kJointStateWaitSeconds)) {
    last_pick_error_ = "valid joint state unavailable";
    return false;
  }
  {
    auto current = mgi_->getCurrentState();
    robot_state_ = std::make_shared<moveit::core::RobotState>(*current);
    robot_state_->update();
    RCLCPP_INFO(this->get_logger(), "start state = current (dry-run)");
  }

  // 诊断 + 整链规划 (planFullChain 内部各失败点记录 last_pick_error_)
  runIKContinuityDiagnostic(robot_state_);
  GraspChain chain;
  const bool chain_ok = planFullChain(robot_state_, chain);
  runLiftReachabilityDiagnostic();
  if (!chain_ok) {
    if (last_pick_error_.empty()) {
      last_pick_error_ = "full pick chain planning failed";
    }
    return false;
  }

  state_ = State::DRY_RUN_SUCCESS;
  logFinalReport();
  RCLCPP_INFO(this->get_logger(), "=== dry-run 成功 (service), 无真机运动, 无夹爪命令 ===");
  return true;
}

}  // namespace scoutfetch_grasp

// ==================== main ====================

#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<scoutfetch_grasp::PickNode>();
  if (!node->run()) {
    RCLCPP_ERROR(node->get_logger(), "pick_node 启动失败, 退出码 1");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "pick_node 已退出");
  rclcpp::shutdown();
  return 0;
}
