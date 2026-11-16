// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <scoutfetch_interfaces/msg/gripper_status.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <moveit/utils/moveit_error_code.h>
#include <std_srvs/srv/trigger.hpp>

namespace scoutfetch_grasp
{

class PickNode : public rclcpp::Node
{
public:
  explicit PickNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PickNode() override;

  bool run();
  // 真实执行: 本次任务目标只来自 /execute_pick 触发快照参数,
  // 函数内禁止再读取 latest_target_ / params_.object_pos 作为本次目标。
  bool runExecute(const geometry_msgs::msg::PoseStamped & active_pick_target);

private:
  // ---- 参数 ----
  struct Params
  {
    std::string planning_group;
    std::string eef_link;
    std::string planning_frame;
    std::string named_ready;
    // 抓取后是否回到 ready (折臂收拢)。搬运任务须为 false: 物体仍在两指间,
    // ready 的折臂轨迹会把物体蹭到车身/手臂上掉下来。
    bool return_to_ready = true;
    // 搬运收臂: lift 后折到"夹着物体也安全"的关节目标 (carry_stow_joints)。
    // 与 named_ready 不同, 该目标须保证持有物不碰车体。空数组 = 未配置。
    bool return_to_stow = false;
    std::vector<double> carry_stow_joints;
    std::vector<double> recovery_view_joints;
    bool execute = false;
    // 第二道安全门: 仅 execute=true 且 allow_real_execution=true 才允许真实执行。
    // 默认 false, 真机授权只能通过运行时参数显式打开, 不在任何正式配置里默认 true。
    bool allow_real_execution = false;
    // 常驻输入: 订阅 /grasp_target_pose (A2 target_pose_resolver 输出, base_link 系)
    std::string target_topic = "/grasp_target_pose";
    // 显式触发入口: /execute_pick (std_srvs/srv/Trigger)
    std::string execute_service = "/execute_pick";

    geometry_msgs::msg::Point object_pos;
    geometry_msgs::msg::Vector3 object_size;
    geometry_msgs::msg::Vector3 grasp_offset;
    double grasp_min_z = -10.0;
    bool floor_pick_enabled = false;
    double floor_threshold_z = 0.05;
    double floor_lift_distance = 0.25;
    double floor_grasp_z = -0.02;

    geometry_msgs::msg::Quaternion grasp_orientation;  // 归一化后使用

    Eigen::Vector3d approach_axis{0, 0, -1};
    double approach_distance = 0.10;
    Eigen::Vector3d lift_axis{0, 0, 1};
    double lift_distance = 0.10;

    double cartesian_step = 0.01;
    double min_cartesian_fraction = 0.99;

    double velocity_scaling = 0.1;
    double acceleration_scaling = 0.1;

    double gripper_open_width = 0.08;
    double gripper_close_width = 0.0;
    double gripper_force = 1.0;
    double gripper_timeout = 5.0;
    double gripper_width_tolerance = 0.005;
    // 夹住物体即视为闭合成功: 物体夹在两指间使 width 无法合到 0,
    // 只要 width <= 该阈值即判定已夹住 (而非必须到达 close_width=0)
    double gripper_grasp_width_threshold = 0.05;
    // 判定"真的夹住"所需的最小宽度。空抓时两指会完全合拢到 ~0, 此时
    // width 虽然 <= threshold (满足上面的条件), 但显然没有夹到任何东西。
    // 要求最终 width >= 该下限才能算夹住; 默认 0 (向后兼容, 不做下限检查)。
    double gripper_grasp_width_min = 0.0;
  };
  Params params_;

  // ---- 运行期信息 ----
  double start_velocity_scale_ = 0.0;
  double start_acceleration_scale_ = 0.0;
  std::string arm_planner_id_;

  double approach_fraction_ = -1.0;
  double lift_fraction_ = -1.0;
  double reverse_approach_fraction_ = -1.0;
  int pre_grasp_plan_point_count_ = 0;
  std::vector<double> q_grasp_;
  std::vector<double> q_pre_grasp_;

  rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr client_;

  geometry_msgs::msg::Pose object_pose_;
  geometry_msgs::msg::Pose grasp_pose_;
  geometry_msgs::msg::Pose pre_grasp_pose_;
  geometry_msgs::msg::Pose lift_pose_;

  // ---- 夹爪反馈 ----
  rclcpp::Subscription<scoutfetch_interfaces::msg::GripperStatus>::SharedPtr gripper_status_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_cmd_pub_;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> spin_exec_;
  std::thread spin_thread_;
  std::atomic<bool> stop_spin_{false};

  std::mutex status_mutex_;
  std::condition_variable status_cv_;
  std::optional<scoutfetch_interfaces::msg::GripperStatus> latest_gripper_status_;
  std::optional<rclcpp::Time> last_gripper_status_time_;

  // ---- 真实 joint state 就绪等待 (execute 首读前) ----
  std::mutex joint_wait_mutex_;
  std::condition_variable joint_wait_cv_;
  bool joint_wait_valid_ = false;

  // ---- Marker ----
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  const std::string marker_ns_ = "grasp";

  std::thread marker_repub_thread_;
  std::atomic<bool> stop_marker_repub_{false};

  // ---- 目标来源 / 显式触发 (常驻) ----
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_target_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_pick_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_srv_;
  // /execute_pick 使用独立 Reentrant 回调组: 规划期间阻塞本组线程,
  // 不占默认组, 保证 GetCartesianPath 响应/gripper/target 等默认组回调仍可调度。
  rclcpp::CallbackGroup::SharedPtr execute_service_cg_;
  std::mutex target_mutex_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_target_;
  // 抓取链进行中标志 (一触发一执行, 拒绝重入); 完成后无论成败都必须复位
  std::atomic<bool> pick_in_progress_{false};
  // 最近一次 dry-run 失败阶段 (作为 /execute_pick 响应 message)
  std::string last_pick_error_;
  // 保护 object_pose_/grasp_pose_/pre_grasp_pose_/lift_pose_:
  // computePoses 写入 vs marker 重发线程 publishMarkers 读取
  mutable std::mutex pose_mutex_;

  // ---- MoveIt ----
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
  std::shared_ptr<moveit::core::RobotState> robot_state_;

  // ---- 状态机 ----
  enum class State
  {
    INIT,
    WAIT_MOVEIT,
    LOAD_PARAMETERS,
    COMPUTE_POSES,
    PUBLISH_MARKERS,
    PLAN_PRE_GRASP,
    PLAN_CARTESIAN_APPROACH,
    PLAN_CARTESIAN_LIFT,
    PLAN_RETURN_READY,
    DRY_RUN_SUCCESS,
    OPEN_GRIPPER,
    EXECUTE_PRE_GRASP,
    EXECUTE_APPROACH,
    CLOSE_GRIPPER,
    EXECUTE_LIFT,
    EXECUTE_READY,
    EXECUTE_SUCCESS,
  };

  State state_ = State::INIT;

  // ---- 步骤 ----
  bool loadParameters();
  // 计算抓取规划位姿链 (object→grasp→pre_grasp→lift)。
  // task_target_position: 非空 = 本次任务目标位置 (service 触发快照, 动态输入);
  // 空 = 使用 params_.object_pos (启动/standalone 遗留, 静态配置)。
  bool computePoses(std::optional<geometry_msgs::msg::Point> task_target_position);
  void publishMarkers() const;
  void startMarkerRepublisher();

  bool planToPose(moveit::core::RobotStatePtr start,
                  const geometry_msgs::msg::Pose & pose,
                  moveit_msgs::msg::RobotTrajectory & out_traj);
  bool planToNamed(moveit::core::RobotStatePtr start,
                   const std::string & name,
                   const std::string & label,
                   moveit_msgs::msg::RobotTrajectory & out_traj) const;
  moveit::core::MoveItErrorCode planWithTimeout(
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const std::string & label) const;
  bool planCartesian(moveit::core::RobotStatePtr start,
                     const std::vector<geometry_msgs::msg::Pose> & waypoints,
                     const std::string & label,
                     moveit_msgs::msg::RobotTrajectory & out_traj,
                     double & fraction);

  // ---- 真机执行 (execute=true) ----
  // 读取真实 current state: 等待有效 joint state, 校验 joint1~6 有限, update() FK 后再用。
  bool readRealState(const std::string & label, moveit::core::RobotStatePtr & out_state);
  // 在首次真实状态读取前, 等待 remap 后的 joint_states topic 收到含 joint1~6 的有效消息。
  bool waitForValidJointState(double timeout_s);
  bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory & traj,
                         const std::string & label);
  bool openGripper();
  bool closeGripper();
  bool verifyHeld();
  void logRealState(const std::string & label, moveit::core::RobotState & state) const;

  // GetCartesianPath 返回全速轨迹, 执行前用 MoveIt IterativeParabolicTimeParameterization
  // 按 velocity_scaling/acceleration_scaling 重新时间参数化 (官方轨迹 retiming)。
  bool retimeTrajectory(moveit::core::RobotStatePtr start,
                        moveit_msgs::msg::RobotTrajectory & traj);

  // ---- 整链规划 (dry-run 与 execute 预验证共用) ----
  // grasp-anchored IK branch 策略:
  //   grasp_pose IK → q_grasp → 反向 Cartesian(grasp→pre_grasp) → q_pre_grasp
  //   → current→q_pre_grasp(joint target) → 正向 approach(反转+retime) → lift → ready
  struct GraspChain
  {
    moveit_msgs::msg::RobotTrajectory pre_grasp_traj;  // current -> q_pre_grasp (joint)
    moveit_msgs::msg::RobotTrajectory approach_traj;   // q_pre_grasp -> q_grasp (反转+retime)
    moveit_msgs::msg::RobotTrajectory lift_traj;       // q_grasp -> lift_end (retime)
    moveit_msgs::msg::RobotTrajectory ready_traj;      // lift_end -> ready (joint)
    std::vector<double> q_grasp;                       // grasp_pose 的 IK 关节解
    std::vector<double> q_pre_grasp;                   // 反向 Cartesian 终点 (同一连续 branch)
    std::vector<double> q_lift_end;                    // lift 终点 (execute 校验用)
    double reverse_fraction = -1.0;                    // 反向 Cartesian fraction
  };
  bool planFullChain(moveit::core::RobotStatePtr start, GraspChain & chain);
  moveit::core::RobotStatePtr trajectoryEndState(
    moveit::core::RobotStatePtr start,
    const moveit_msgs::msg::RobotTrajectory & traj) const;

  // ---- grasp-anchored IK branch 相关 ----
  // 诊断: 从 pre_grasp 终态沿 approach 方向逐 waypoint 顺序 IK, 报告丢失连续 IK 的位置 (仅诊断)。
  bool runIKContinuityDiagnostic(moveit::core::RobotStatePtr start);
  // 诊断: 从 q_grasp 沿 lift_axis 逐 waypoint 顺序 IK, 报告 lift 连续可达高度 + 关节限位检查 (仅诊断)。
  bool runLiftReachabilityDiagnostic() const;
  // current -> 指定关节目标 (joint-space 规划, 避免 setPoseTarget 重选 IK branch)。
  bool planToJointTarget(moveit::core::RobotStatePtr start,
                         const std::vector<double> & joint_positions,
                         const std::string & label,
                         moveit_msgs::msg::RobotTrajectory & out_traj);
  // 校验真实状态与预期关节值一致 (execute 每段执行后重新校验)。
  bool validateStateAgainst(const moveit::core::RobotState & state,
                            const std::vector<double> & expected,
                            const std::string & label) const;

  bool checkPoseAxes() const;
  bool checkGraspApproachAlignment(double & deg_out) const;

  void gripperStatusCallback(const scoutfetch_interfaces::msg::GripperStatus::SharedPtr msg);
  bool waitForGripperTarget(double target_width, double tol, double timeout_s,
                            double & achieved_width);
  bool sendGripperCommand(double width, double force) const;

  void logFinalReport() const;

  // ---- 目标来源 / 显式触发 (常驻) ----
  // /grasp_target_pose: 只缓存最新合法目标 (frame 须为 planning_frame, 有限位置, 合法 quaternion),
  // 禁止自动规划/抓取。
  void onGraspTarget(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool validateGraspTarget(const geometry_msgs::msg::PoseStamped & msg) const;
  // /execute_pick: 显式唯一触发入口, 返回 success + message; 无目标/busy/execute=true 均拒绝。
  void onExecutePick(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  // 对触发瞬间的 target snapshot 执行一次 dry-run 抓取规划链, 返回成败。
  bool runPickDryRun(const geometry_msgs::msg::PoseStamped & active_pick_target);
};

}  // namespace scoutfetch_grasp
