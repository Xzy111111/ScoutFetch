# scoutfetch_reach

**抓取感知基站位规划器**。给定地图系目标物体位姿（`/target_pose_map`），围绕目标生成候选底盘站位，按 **Nav2 可通行性（含终端误差鲁棒检查）+ Piper 抓取可行性 + 舒适距离/关节居中评分** 层层过滤，发布最优基站位（`/best_base_pose`，map 系）。**只规划、不下达底盘运动指令**，实际导航由下游 `scoutfetch_mission/nav_goal_executor` 负责。

## 包结构

```
scoutfetch_reach/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── scoutfetch_reach.yaml    # 候选生成/评分/抓取几何参数
├── launch/
│   └── scoutfetch_reach.launch.py   # 节点启动（可带 RViz）
├── include/scoutfetch_reach/
│   └── scoutfetch_reach_node.hpp
└── src/
    └── scoutfetch_reach_node.cpp
```

## 工作原理

收到一条 `/target_pose_map`（一次目标 = 一次规划请求）后执行：

```
① 候选生成   以 target→robot 方向为参考，在机器人这一侧按
             radius [0.30..0.75] × angle_offset [-60..60°] 撒候选，
             底盘 yaw 始终朝向目标
② 可通行判定 完整底盘 footprint（真实 550×400mm + 安全边距）变换到 map，
             对 costmap 做逐格碰撞检查（center≥99 判碰撞,
             footprint==100 判硬碰撞, 1~99 不 reject, unknown fail-closed）
③ 导航误差鲁棒检查 对每条候选按 8 方向 × 3 yaw 采样 Nav2 终端位姿误差包络
             （|Δxy|≤nav_xy_error_margin, |Δyaw|≤nav_yaw_error_margin），
             任一采样 footprint 碰撞即淘汰；抓取可行性按最恶劣 Case B/C/D 可达性判定
④ 抓取可行性 grasp_pose IK → q_grasp → 反向 Cartesian(grasp→pre_grasp)
             → lift Cartesian，每段 fraction ≥ 0.99；q_grasp/pre_grasp/lift
             任一关节距限位 < joint_margin 淘汰
⑤ 评分       total = 0.6×distance_score(距离舒适区居中度) + 0.4×joint_score(关节居中)
⑥ 选最优     先最大化 total_score；总分并列（|Δ|≤epsilon）时取距当前机器人 nav_distance 更小者
             → 发布 /best_base_pose（stamp 绑定到目标请求 stamp）+ 候选 Marker
```

全部候选失败（无解）时**不发布** best，并清除旧 marker，避免视觉残留。机器人位姿（`map→base_link`）不可用时 fail-closed：跳过本次规划、不发布，绝不用 map 原点伪造。

## 接口

| 方向 | 话题/服务 | 类型 | 说明 |
|------|-----------|------|------|
| 订阅 | `/target_pose_map` | `geometry_msgs/PoseStamped` | 目标物体（map 系，transient_local），scoutfetch_vision 发布 |
| 订阅 | `global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Nav2 全局代价地图（可通行判定） |
| 订阅 | `joint_states`（remap→`feedback/joint_states`） | `sensor_msgs/JointState` | 真机关节反馈（IK seed） |
| 发布 | `/best_base_pose` | `geometry_msgs/PoseStamped` | 最优基站位（map 系） |
| 发布 | `/base_pose_candidates` | `visualization_msgs/MarkerArray` | 候选底盘可视化（绿=best / 蓝=valid / 灰=淘汰） |
| 客户端 | `/compute_cartesian_path` | `GetCartesianPath` | MoveIt 反向 Cartesian 可达性检查 |

## 启动

前置：Nav2（`global_costmap`）与 MoveIt（`piper_moveit.launch.py` 的 move_group）需已在运行。

```bash
ros2 launch scoutfetch_reach scoutfetch_reach.launch.py use_rviz:=true
```

launch 通过 `MoveItConfigsBuilder` 只读加载 piper_control 的机器人模型（与 pick_node 一致），不修改 piper_control。

## 关键参数（config/scoutfetch_reach.yaml）

| 参数 | 默认 | 说明 |
|------|------|------|
| `radius_min/max/step` | `0.30/0.75/0.05` | 候选半径范围（0.30 近端贴目标，0.75 远端覆盖可达区） |
| `angle_offsets_deg` | `[-60..60]` | 相对 target→robot 方向的偏移角 |
| `preferred/min/max_distance` | `0.475/0.25/0.75` | 舒适距离评分区间（m） |
| `nav_xy_error_margin` / `nav_yaw_error_margin` | `0.05` / `0.10` | Nav2 终端位姿误差容限（导航误差检查采样包络，m/rad） |
| `enable_robust_reachability_check` | `true` | 开启 导航误差鲁棒可达性检查（false 时仅按名义位姿判定） |
| `costmap_topic` | `global_costmap/costmap` | 代价地图话题（代价地图判定：center≥99 判碰撞, footprint 仅 ==100 判硬碰撞, 1~99 不 reject, unknown fail-closed; 无独立阈值参数） |
| `footprint_x/y` + `base_safety_margin` | `0.275/0.20` + `0.05` | 真实底盘半长/半宽 + 外扩边距 |
| `grasp_orientation` / `approach_axis` / `lift_axis` | 与 pick_and_place.yaml 一致 | 抓取几何 |
| `cartesian_step` / `min_cartesian_fraction` | `0.01` / `0.99` | 反向 Cartesian 步长/最小完成比例 |
| `cartesian_avoid_collisions` | `false` | 候选可行性不做碰撞检测（规避 move_group FCL 0.7.0 自碰撞段误报，仅本包；真实抓取由 scoutfetch_grasp 碰撞感知规划最终验证） |
| `joint_margin` | `0.05` | 关节限位安全余量（rad） |
| `distance_weight` / `joint_weight` | `0.6` / `0.4` | 评分权重 |
| `score_tie_epsilon` | `0.02` | 并列判定阈值 |

## 工程要点

- **多线程 executor**：回调内会阻塞等待 `/compute_cartesian_path` 服务响应，单线程 executor 会被占死导致响应无人处理 → 超时。
- **服务客户端独立 callback group**：订阅回调阻塞 `wait_for` 期间若与响应回调同组串行，响应永远无法送达（死锁）。
- **非阻塞 TF**：`lookupTransform("map","base_link", TimePointZero)`，不带 timeout，TF 缺失时抛异常 fail-closed。
- **浮点边界**：候选半径用整数索引生成，避免浮点累加在 `0.75` 边界漏点。
- 验收输出：每次规划打印候选总数 / 各淘汰分项 / 全部 valid 候选 / 按 radius 汇总，便于真机调参。

## 在整体流水线中的位置

```
scoutfetch_vision /target_pose_map → 本包 /best_base_pose
→ scoutfetch_mission nav_goal_executor → Nav2 → 到位后 target_pose_resolver
→ /grasp_target_pose → scoutfetch_grasp /execute_pick
```

## 许可证

本包自编代码由维护者确认采用 [Apache-2.0](LICENSE)，源码使用
`SPDX-License-Identifier: Apache-2.0` 标注。附带的模型权重、CAD 和其他第三方
资源保留各自许可，见[第三方归属清单](../../THIRD_PARTY_NOTICES.md)。
