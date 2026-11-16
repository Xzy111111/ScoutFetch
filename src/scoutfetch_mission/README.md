# scoutfetch_mission

组织自主搜索、视觉确认、导航、抓取、夹持验证和返回任务，并提供键盘 / 自主模式切换。
完整演示、键盘操作和场景参数见[项目首页](../../README.md)，运行结果见[验收报告](../../docs/results.json)。

## 入口与节点

```bash
# 完整书店任务，默认手动，控制窗口按 2 开始自主
ros2 launch scoutfetch_mission sim_bookstore_search.launch.py

# 实验室随机布局
ros2 launch scoutfetch_mission sim_bottle_search.launch.py radius_min:=1.6 radius_max:=2.4

# 只启动编排节点；后端、感知、站位规划、MoveIt 和 Nav2 需要另行提供
ros2 launch scoutfetch_mission scoutfetch_mission.launch.py enable_navigation:=true search_enabled:=true
```

| 节点 | 职责 |
|---|---|
| `task_fsm` | 任务身份、模式切换、搜索探索、抓取及恢复编排 |
| `nav_goal_executor` | 唯一 Nav2 目标派发者；接收目标及匹配的取消请求 |
| `target_pose_resolver` | 导航到位后，将 map 目标转换为 base_link 抓取目标 |
| `robot_control` | 键盘控制和实时 YOLO 图像窗口 |
| `start_sim_task` | 自动启动选项的就绪检查 |

## 任务行为

开始任务时保存 map 系起点，后续探索或掉落恢复不覆盖该返回点。
默认连续旋转搜索，候选目标须停车并通过新鲜、多帧三维确认；短暂二维线索只用于选择
靠近观察点，不能直接作为抓取坐标。一圈无目标时通过连通代价地图换观察点。

确认后选择可抓取站位，交给 Nav2 导航，到位停稳后解析抓取目标并调用 `/execute_pick`。
真实执行模式通过夹爪宽度、反馈时间和驱动状态检查携物，返回后再次确认。
掉落时先取消导航、准备空手观察，再有限次重试；反馈异常或无法继续时停车报错。

手动接管会取消导航并等待终态，机械臂执行中则等待当前动作结束。
速度链为 `/cmd_vel_search`、`/cmd_vel_nav`、`/cmd_vel_teleop` → twist_mux →
`/cmd_vel_unfiltered` → Collision Monitor → `/cmd_vel`。

## 主要接口

| 类型 | 接口 | 用途 |
|---|---|---|
| 服务 | `/set_control_mode` (`SetBool`) | true 自主，false 手动 |
| 服务 | `/start_task` (`Trigger`) | 显式启动新任务 |
| 话题 | `/control_mode`、`/task_state`、`/grasp_status` | 模式、阶段与夹持状态 |
| 输入 | `/target_pose_map`、`/best_base_pose` | map 目标与规划站位 |
| 导航 | `/navigation_goal`、`/navigation_cancel`、`/navigation_result` | 目标、按 stamp 匹配的取消及结果 |
| 抓取 | `/resolve_grasp_target` → `/grasp_target_pose` → `/execute_pick` | 到位解析及显式抓取 |
| 恢复 | `/prepare_grasp_recovery` | 停车后的空手观察准备 |

输入目标和起点 TF 必须有效；旧任务目标及过期观测不用于新任务。解析请求采用
VOLATILE QoS，目标 stamp 用于区分任务与观测身份。

## 参数

| 参数 | 编排节点默认值 | 说明 |
|---|---|---|
| `enable_navigation` | false | 显式允许向 Nav2 派发实际目标 |
| `search_enabled` / `exploration_enabled` | false / false | 搜索与换观察点；统一仿真入口均开启 |
| `search_continuous_rotation` / `search_angular_speed` | true / 0.25 | 连续旋转及角速度 rad/s |
| `search_max_turns` / `search_timeout_sec` | 1 / 180 | 0 表示持续搜索；统一仿真入口均为 0 |
| `grasp_monitor_enabled` | false | 夹持检查；统一仿真的执行模式开启 |
| `use_sim_time` | true | 必须与后端一致 |

其他检测、停稳、导航超时及恢复参数见 [task_fsm.py](scoutfetch_mission/task_fsm.py) 的
`declare_parameter`，启动参数见 [launch/](launch/)。

```bash
ros2 topic echo /task_state
ros2 topic echo /grasp_status
ros2 run rqt_image_view rqt_image_view /camera/yolo/image_annotated
```

当前地面恢复针对直立瓶标定，侧躺物体不保证可抓。`execute_pick:=false` 时仅验证抓取
规划，底盘仍可能运动；此模式的 DONE 不能作为实际夹持的证明。
