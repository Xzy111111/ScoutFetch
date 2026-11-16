# scoutfetch_grasp

基于共享 MoveIt 模型执行 Piper 机械臂抓取。节点缓存 `/grasp_target_pose`，只有收到
`/execute_pick` 才锁定目标并开始整链验证；重复触发会被拒绝。

## 启动与执行

后端和 MoveIt 必须已运行。完整仿真任务会配置本包，单独调试使用：

```bash
ros2 launch scoutfetch_grasp pick_node.launch.py execute:=false use_rviz:=false
```

默认 dry-run。`execute` 与 `allow_real_execution` 均为 true 才允许实际执行。
仿真默认读取 `/joint_states`，实机覆盖 `use_sim_time:=false` 与
`joint_states_topic:=/feedback/joint_states`。

规划先求抓取点 IK，再反向求连续接近路径，使用同一分支验证接近和抬升。
执行时先验证完整链，每段动作后检查实际反馈；携物任务使用 `return_to_stow:=true`
收至携物位，优先于 `return_to_ready`。统一任务另由 FSM 监测返回期间的夹持状态。

### launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `execute` | `false` | `false` = dry-run；`true` = 允许真机执行（仍需 `allow_real_execution`） |
| `allow_real_execution` | `false` | 第二道安全门：`execute=true` 且本参数 `true` 才真正执行 |
| `use_rviz` | `true` | 启动 grasp.rviz（显示 grasp markers） |
| `joint_states_topic` | `/joint_states` | 仿真默认值；实机覆盖为 `/feedback/joint_states` |
| `config_file` | `pick_and_place.yaml` | `config/` 下的配置文件 |

> `pick_node.launch.py` 通过 `MoveItConfigsBuilder` 只读加载 `scoutfetch_moveit_config` 的 `robot_description` 等参数，保证本地机器人模型与 move_group 一致。

## 配置（config/pick_and_place.yaml）

| 参数 | 默认 | 说明 |
|------|------|------|
| `planning_group` / `eef_link` / `planning_frame` | `arm` / `gripper_tcp` / `base_link` | MoveIt 组/末端/规划系 |
| `execute` | `false` | dry-run 开关（launch 可覆盖） |
| `allow_real_execution` | `false` | 真机授权（launch 可覆盖，默认绝不 true） |
| `target_topic` | `/grasp_target_pose` | 常驻输入：base_link 系抓取目标（target_pose_resolver 输出） |
| `execute_service` | `/execute_pick` | 显式触发入口（`std_srvs/srv/Trigger`） |
| `object_pose` | `(0.30, 0, 0.30)` | 保留的几何配置；服务执行仍要求已缓存的合法动态目标 |
| `grasp_orientation` | Ry(110°) | TCP 姿态（纯向下 180° 不可达，用 110° 近似顶抓） |
| `approach_axis` / `approach_distance` | `[0.9397, 0, -0.342]` / `0.10` | 进给方向与距离 |
| `lift_axis` / `lift_distance` | `[-0.9397, 0, 0.342]` / `0.05` | 抬升方向与距离 |
| `cartesian_step` / `min_cartesian_fraction` | `0.01` / `0.99` | 笛卡尔步长 / 最小完成比例 |
| `velocity_scaling` / `acceleration_scaling` | `0.10` / `0.10` | 轨迹速度/加速度缩放 |
| `gripper.open_width` / `close_width` | `0.09` / `0.0` | 夹爪开度（米） |
| `gripper.grasp_width_threshold` / `grasp_width_min` | `0.070` / `0.060` | 当前瓶子闭合验收宽度范围（米），结合新鲜反馈与驱动状态检查 |
| `gripper.force` / `command_timeout` / `width_tolerance` | `1.0` / `5.0` / `0.005` | 夹持力 / 反馈等待超时 / 宽度容差 |

> 所有动态目标 Pose 均在 `planning_frame`（base_link）。任务目标由上游 `scoutfetch_mission/target_pose_resolver` 提供，服务触发时锁定最新合法动态目标，缺少目标时拒绝执行。

## 接口

| 方向 | 话题/服务 | 类型 | 说明 |
|------|-----------|------|------|
| 订阅 | `/grasp_target_pose` | `geometry_msgs/PoseStamped` | 抓取目标（base_link，仅缓存） |
| 服务 | `/execute_pick` | `std_srvs/srv/Trigger` | 唯一触发入口（成功/失败 + message） |
| 订阅 | `joint_states`（按 launch remap） | `sensor_msgs/JointState` | 当前关节状态（IK seed / 执行校验） |
| 订阅 | `feedback/gripper_status` | `scoutfetch_interfaces/GripperStatus` | 夹爪反馈（宽度/驱动状态） |
| 发布 | `control/gripper_cmd` | `sensor_msgs/JointState` | 夹爪命令（name=`gripper`, position=宽度, effort=力） |
| 发布 | `grasp_markers` | `visualization_msgs/MarkerArray` | id 0-7：物体/3 抓取点/两条路径线/TCP+Z+Y 箭头（1Hz 重发） |

服务客户端：`/compute_cartesian_path`（GetCartesianPath）、`/plan_kinematic_path`（MoveGroupInterface 内部）。

## 调试与许可

RViz 订阅 `/grasp_markers` 可查看目标、接近点和抬升点。失败时结合日志中的 IK、
Cartesian fraction、关节限位和夹爪反馈定位原因，几何参数见配置文件。
`/prepare_grasp_recovery` 提供掉落后空手退离和观察姿态准备，由任务层停车后调用。

自编代码采用 [Apache-2.0](LICENSE)，第三方资源见[归属清单](../../THIRD_PARTY_NOTICES.md)。
