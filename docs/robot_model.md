# 唯一机器人模型规范

## 决策

`scoutfetch_description/urdf/scoutfetch.urdf.xacro` 是唯一权威模型。Gazebo、RViz、
MoveIt 和真机启动都必须从该文件生成 `robot_description`，禁止复制第二份 URDF。

模型基于 RoboMaster 哨兵底盘与松灵机械臂改装，来源归属见 [模型来源与归属](../THIRD_PARTY_NOTICES.md)。

模型直接导入来源：

- 底盘网格、轮位、质量和惯量：真机仓库 `car_sim` 的 SolidWorks 导出模型。
- Piper 关节轴、连杆、质量和惯量：真机仓库导入的松灵 Piper 模型。
- 底盘安装：整机公共根为 `base_link`，`base_link -> BODY` 固定变换 `z=+0.004 m`。
- D435i：当前模型采用便于仿真观察的腕部安装位姿；171 组数据的 Tsai-Lenz
  真机手眼标定结果仅保留为参考，当前没有应用到 Xacro。
- Mid-360：原整机 CAD 固定关节。

## TF 契约

```text
world / odom -> base_link
                ├── BODY
                │   ├── F_B_R_Link / F_B_L_Link / F_F_L_Link / F_F_R_Link
                │   └── mid_Link -> livox_frame -> imu_frame
                └── link1 ... link6
                    ├── camera_link -> optical frames
                    └── gripper -> gripper_tcp
```

`base_link` 是真机和仿真共用的整机根节点；`BODY` 仅保留为底盘 CAD 几何帧。
仿真里程计发布 `odom -> base_link`；模型内部 TF 由 `robot_state_publisher` 发布。

### D435i 当前安装约定

URDF 在 `camera_link` 下包含相机背板、左右连接臂和腕部固定板四个 visual，组成双侧 L 型安装支架。
支架按当前安装位置补全外观，不改变 `camera_mount_joint`、光学帧、质量、惯量或碰撞体；
它不代表已测量的实体支架动力学或碰撞模型。安装位姿与参考手眼标定的关系仍以下文为准。

`camera_mount_joint` 固连 `link6 -> camera_link`：

```text
xyz = [-0.055, 0, 0.040] m
rpy = [0, -pi/2, 0] rad
```

该位姿使 `camera_link +X`（Gazebo 成像方向）与 `link6 +Z` 同向。相机随腕部
运动，它不是固定车载前视相机。ROS 图像消息使用
`camera_color_optical_frame`；Gazebo RGB-D sensor 则挂在 `camera_link`，不要把
optical frame 的轴约定直接当成 Gazebo sensor 的成像轴。

如需恢复真机标定位姿，必须先明确 `link6 -> camera_link` 与
`link6 -> camera_color_optical_frame` 的变换语义并验证图像方向，不能直接把参数
文件中的 optical-frame 标定数值复制到 `camera_mount_joint`。

## 坐标与命名

- REP-103：x 前、y 左、z 上，弧度、米、秒。
- 保留真机代码的名称：`base_link` 是整机公共基准，`BODY` 是底盘 CAD 帧。
- 机械臂关节保持 `joint1..joint6`，避免破坏 Piper 驱动接口。
- 轮关节采用可读名字，旧 CAD 名仅作为来源记录，不作为公共 API。

## 已知需要真机辨识的参数

以下初值可运行，但 sim-to-real 前必须用仓库日志或新实验辨识：

1. `wheel_radius=0.076 m`：直线定距实验辨识有效半径。
2. 麦轮滚子等效摩擦：横移、旋转实验分别拟合。
3. 整机载荷后的质心与惯量：CAD 未包含线缆、电池和控制器。
4. 机械臂位置环带宽、速度/加速度限制及 CAN 延迟。
5. 夹爪接触摩擦、最大夹持力及宽度零偏。
6. D435i 深度噪声、Mid-360 扫描图样和 IMU bias/random walk。

参数更新必须进入 `config/model_parameters.yaml`，并附测量日期和数据来源。

## Gazebo 麦轮接触后端（2026-11-06）

`scoutfetch_sim/scripts/sim_model.py` 从同一 Xacro 生成 URDF，再转换为临时 SDF，
只为四个轮子设置等效接触；外观网格、质量、惯量、机械臂与夹爪接触仍来自原模型。
不维护第二份机器人描述，也不使用模型真值驱动上层任务。

CAD 的右后轮关节帧绕 z 旋转了 π，局部 +Y 与其他三轮相反；在 `sim_mode` 中将其
关节轴改为局部 -Y，使控制器对四轮的正速度具有同一前滚语义。非仿真视图保持原轴。
刚性 CAD 网格中的滚子不会独立自由滚动，因此仿真改用 Gazebo Fortress 官方
麦轮示例的球形接触及车体系固定的 ±45° 摩擦方向：驱动方向 `mu=1`，自由滚子方向
`mu2=0`。这是接触近似，不是对真实滚子动态、轮胎变形或打滑的验证。

接触半径按 CAD 包络取 0.07687545 m，接触中心按轮子 COM 向内偏约 0.03878 m，
控制器的 `lx+ly` 因此为 0.29110 m。原来直接使用关节锚点间距和统一刚性网格摩擦，
会使旋转/横移的实际运动与轮速里程计严重不一致。

实测：指令原地旋转 5 s，里程计约 1.25 rad，Gazebo 约 1.25 rad，车体漂移约
9 mm；随后分别前进和横移 0.30 m，真实位移与里程计基本一致。
原始记录见 `artifacts/validation/base_motion_2026-11-06.jsonl`。

## 模型与接口复用

上层通过公共 TF、传感器话题、`/cmd_vel`、关节轨迹 Action 和夹爪命令/反馈协作。
仿真与实机的驱动差异由后端适配，感知、导航、抓取及任务代码复用同一模型和接口约定。
