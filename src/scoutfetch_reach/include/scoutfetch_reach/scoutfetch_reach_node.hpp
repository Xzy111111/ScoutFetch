// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <mutex>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

namespace scoutfetch_reach
{

// 候选抓取可行性失败原因 (对应验收计数分项)
enum class GraspFailReason
{
  OK = 0,
  IK = 1,          // grasp_pose IK 无解
  APPROACH = 2,    // grasp->pre_grasp 反向 Cartesian 失败/分数不足
  LIFT = 3,        // lift Cartesian 失败/分数不足
  JOINT_MARGIN = 4,  // IK 解任一关节距限位小于 joint_margin
};

// Nav2 终端误差鲁棒检查失败原因 (A8-R2 新增, 叠加在 nominal 整链之上)
enum class NavRobustFailReason
{
  NONE = 0,
  NAV_ERROR_COLLISION = 1,     // 终端误差包络 (8 方向 x 3 yaw) 内任一 footprint 碰撞
  NAV_ERROR_UNREACHABLE = 2,   // 误差最坏情形 (away+0.05 / yaw±0.10) 抓取链不可达
};

class BasePosePlannerNode : public rclcpp::Node
{
public:
  explicit BasePosePlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Params
  {
    double radius_min = 0.30;                 // 候选半径范围 (m), 运行时生成
    double radius_max = 0.75;                 // A8-R2: 恢复 0.75 为搜索空间上限 (由 Nav 误差鲁棒检查决定有效性)
    double radius_step = 0.05;
    std::vector<double> radii;                // 由 radius_min/max/step 生成 (0.30..0.75)
    std::vector<double> angle_offsets_deg;    // 相对 target->robot 方向的偏移角 (deg)

    double preferred_distance = 0.475;        // 优选区 0.45..0.50 (R1 保持, 不变)
    double min_distance = 0.25;
    double max_distance = 0.75;               // A8-R2: 与恢复后的搜索范围一致 (scoring 仅此最小一致性修改)

    // ---- A8-R2: Nav2 终端误差鲁棒检查参数 ----
    // 对应 bringup default.yaml 实际生效的 general_goal_checker:
    //   xy_goal_tolerance=0.05, yaw_goal_tolerance=0.10 (R1 已收紧, 不回退)。
    double nav_xy_error_margin = 0.05;        // 终端 xy 误差裕量 (m): 碰撞包络半径 + Case B away 偏移
    double nav_yaw_error_margin = 0.10;       // 终端 yaw 误差裕量 (rad): 碰撞包络 yaw 采样 + Case C/D
    bool enable_robust_reachability_check = true;  // 关闭则退化为 R1 行为 (仅 nominal 检查)

    std::string costmap_topic = "global_costmap/costmap";

    std::string planning_group = "arm";
    std::string eef_link = "gripper_tcp";
    std::string planning_frame = "base_link";

    geometry_msgs::msg::Quaternion grasp_orientation;
    Eigen::Vector3d grasp_offset{0, 0, 0};
    double grasp_min_z = -10.0;
    bool floor_pick_enabled = false;
    double floor_threshold_z = 0.05;
    double floor_lift_distance = 0.25;
    double floor_grasp_z = -0.02;
    double floor_min_distance = 0.50;
    double floor_max_distance = 0.60;
    double floor_radius_step = 0.025;
    Eigen::Vector3d approach_axis{0, 0, -1};
    double approach_distance = 0.10;
    Eigen::Vector3d lift_axis{0, 0, 1};
    double lift_distance = 0.05;

    double cartesian_step = 0.01;
    double min_cartesian_fraction = 0.99;
    bool cartesian_avoid_collisions = false;  // 候选可行性 Cartesian 不做碰撞检测 (FCL 规避, 仅本包)
    double joint_margin = 0.05;               // 关节限位安全余量 (rad), 过近淘汰

    double base_safety_margin = 0.05;  // 底盘 footprint 外扩安全边距 (m)
    double footprint_x = 0.275;        // 真实底盘半长 (base_link x ±)
    double footprint_y = 0.20;         // 真实底盘半宽 (base_link y ±)

    double distance_weight = 0.6;  // 距离评分权重
    double joint_weight = 0.4;     // 关节构型评分权重
    double score_tie_epsilon = 0.02;  // 并列判定: total_score 差 <= 该值即并列, 取 nav_distance 更小
  };
  Params params_;

  // A8-R3B: footprint 软成本 (仅记录, 不改 scoring)
  // corrected 语义下 footprint 内 1~99 (raw 1..253) 不判硬碰撞; 记录最大/平均非致命成本。
  struct SoftCost
  {
    double max_nonlethal = 0.0;   // footprint 内最大非致命成本 [0,99]
    double mean_nonlethal = 0.0;  // footprint 内平均非致命成本 [0,99]
    double sum_nonlethal = 0.0;
    int n_nonlethal = 0;
  };

  struct RobustDetail
  {
    // 碰撞包络: 8 方向 x 3 yaw = 24 个终端误差采样
    int envelope_total = 0;
    int envelope_safe = 0;
    // Case B: base 沿 away-from-target 偏移 nav_xy_error_margin (yaw nominal)
    double away_tdist = 0.0;                    // Case B 目标平面距离
    GraspFailReason away_reason = GraspFailReason::OK;
    double away_rev_frac = 0.0;                 // Case B 反向 approach 分数
    double away_lift_frac = 0.0;                // Case B lift 分数
    // Case C / D: Case B 位置 + yaw ±nav_yaw_error_margin
    GraspFailReason yaw_p_reason = GraspFailReason::OK;  // yaw +margin
    GraspFailReason yaw_n_reason = GraspFailReason::OK;  // yaw -margin
  };

  struct Candidate
  {
    geometry_msgs::msg::PoseStamped base_pose;  // map frame (position + yaw quat)
    double radius = 0.0;                      // 生成参数 (r)
    double angle_offset_deg = 0.0;            // 相对 target->robot 方向的偏移角 (deg)
    double target_base_x = 0.0;                 // target in candidate base_link
    double target_base_y = 0.0;
    double target_base_z = 0.0;
    bool passable = false;                      // step 1: nav2 passable
    bool feasible = false;                      // step 2-4: grasp feasible
    int fail_reason = 0;                        // GraspFailReason (0=OK)
    double soft_max_nonlethal = 0.0;            // A8-R3B: footprint 最大非致命成本 (仅记录)
    double soft_mean_nonlethal = 0.0;           // A8-R3B: footprint 平均非致命成本 (仅记录)
    int nav_fail_reason = 0;                    // NavRobustFailReason (0=NONE, A8-R2)
    bool nav_robust = false;                    // step 2.5: Nav 误差鲁棒通过
    // 整链明细 (Test D 报告): 各步骤通过与否 + 分数
    bool detail_ik_ok = false;
    bool detail_approach_ok = false;
    bool detail_lift_ok = false;
    double detail_rev_frac = 0.0;
    double detail_lift_frac = 0.0;
    RobustDetail robust;                        // Nav 误差鲁棒明细 (A8-R2)
    double distance_score = 0.0;                // step 5: 距离舒适分 [0,1]
    double joint_score = 0.0;                   // step 5: 关节居中分 [0,1]
    double total_score = 0.0;                   // step 5: 加权总分
    double nav_distance = 0.0;                  // step 6: 距当前 base_link 水平距离 (并列决策用)
    int status = 0;                             // 0=invalid, 1=valid, 2=best
  };

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr best_base_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr cartesian_client_;
  // 服务客户端独立 callback group: 订阅回调阻塞 wait_for 时占用默认组,
  // 若不分离组, 服务响应回调无法在任何 executor 线程执行 -> 永远超时。
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr target_cb_group_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex costmap_mutex_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_costmap_;

  // 真实关节反馈缓存 (feedback/joint_states), 作为 IK seed。
  // 不用 mgi_->getCurrentState(): MoveGroupInterface 的内部 current-state monitor
  // 在其私有节点上订阅, 不受本节点 remap 影响, 拿不到真机状态。
  std::mutex joint_mutex_;
  std::map<std::string, double> joint_positions_;
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool getSeedState(moveit::core::RobotState & out);

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;

  bool loadParameters();
  void targetPoseMapCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  std::vector<Candidate> generateCandidates(const geometry_msgs::msg::PoseStamped & target,
                                            double bx, double by) const;  // bx,by = 当前 robot map 位姿
  bool isPassable(const geometry_msgs::msg::PoseStamped & base_pose,
                  SoftCost * soft_cost = nullptr);  // corrected 语义碰撞检查 + 可选软成本输出
  bool getCurrentPose(double & x, double & y, double & yaw);  // map->base_link, 并列决策
  GraspFailReason checkGraspFeasibility(Candidate & c);
  // A8-R2: target 在给定 base 系 (bx,by,yaw) 的 2D 坐标 (map->base_link 旋转)
  void computeTargetInBase(const geometry_msgs::msg::PoseStamped & target,
                           double bx, double by, double yaw, double & tx, double & ty) const;
  // A8-R2: Nav 终端误差碰撞包络 (8 方向 x 3 yaw, 全通过才 true)
  bool navErrorCollisionEnvelope(const geometry_msgs::msg::PoseStamped & base_pose,
                                 RobustDetail & detail);
  // A8-R2: Nav 终端误差操控鲁棒 (Case B away+margin / Case C/D yaw±margin, 全通过才 true)
  bool navErrorManipulationRobust(const Candidate & c,
                                  const geometry_msgs::msg::PoseStamped & target,
                                  RobustDetail & detail);
  // A8-R2: 组合鲁棒检查 (碰撞包络 + 操控鲁棒), 返回是否通过并填 detail
  bool runNavRobustCheck(const Candidate & c, const geometry_msgs::msg::PoseStamped & target,
                         RobustDetail & detail);
  void printRobustReport(const Candidate & best) const;  // A8-R2 Test D 报告
  bool jointsWithinMargin(const moveit::core::RobotState & state,
                          const moveit::core::JointModelGroup * jmg, double & worst_margin) const;
  double jointCenteredScore(const moveit::core::RobotState & state,
                            const moveit::core::JointModelGroup * jmg) const;
  void printFailReasonStats(const std::vector<Candidate> & cands) const;
  double distanceScore(const Candidate & c) const;
  void publishMarkers(const std::vector<Candidate> & cands,
                      const geometry_msgs::msg::PoseStamped & target) const;
  void clearMarkers() const;  // 失败时清除旧 /base_pose_candidates, 避免视觉残留
  void publishBest(const Candidate & best) const;

  moveit::core::RobotStatePtr trajectoryEndState(
    moveit::core::RobotStatePtr start,
    const moveit_msgs::msg::RobotTrajectory & traj) const;
  bool planCartesian(moveit::core::RobotStatePtr start,
                     const std::vector<geometry_msgs::msg::Pose> & waypoints,
                     moveit_msgs::msg::RobotTrajectory & out_traj,
                     double & fraction);
};

}  // namespace scoutfetch_reach
