# scoutfetch_description

## 这个功能包做什么

`scoutfetch_description` 是机器人的**唯一模型功能包**。它只负责回答一个问题：

> 这台机器人由哪些部件组成，各部件的尺寸、质量、关节、安装位置和坐标系是什么？

底盘、四个麦克纳姆轮、Piper 六轴机械臂、夹爪、D435i、Mid-360 和 IMU
都在同一份 Xacro 中定义。Gazebo、RViz、MoveIt 和未来的真机启动应共同读取这份
模型，不能再各自维护不同的 URDF。

这个包**不负责**物理仿真、导航、运动规划、目标检测或抓取任务。

## 目录说明

```text
scoutfetch_description/
├── config/
│   ├── initial_positions.yaml   仿真启动时机械臂和夹爪的初始位置
│   └── model_parameters.yaml    模型参数、标定值及数据来源
├── launch/
│   └── display.launch.py        只在 RViz 中查看模型，不启动 Gazebo
├── meshes/
│   ├── chassis/                 真机底盘和轮子的 CAD 网格
│   ├── piper/                   Piper 机械臂与夹爪网格
│   └── sensors/                 传感器外观网格及来源说明
├── rviz/
│   └── model.rviz               模型查看界面配置
├── test/
│   └── test_model_contract.py   检查 TF 树、关节和 ros2_control 接口
├── urdf/
│   └── scoutfetch.urdf.xacro        唯一权威整机模型
├── CMakeLists.txt
└── package.xml
```

## 确定的机器人模型

主要 TF 结构为：

```text
base_link                           真机/仿真共用的整机 URDF 根节点
└── BODY                         底盘 CAD 几何帧
├── F_B_R_Link / F_B_L_Link        后轮
├── F_F_L_Link / F_F_R_Link        前轮
├── mid_Link
│   └── livox_frame -> imu_frame
└── base_link                       Piper 安装座
    └── link1 ... link6
        ├── camera_link -> optical frames
        └── flange_link -> gripper_base
                           ├── gripper_link1 / gripper_link2
                           └── gripper_tcp
```

关键命名约定：

- `base_link`：导航、MoveIt、感知和任务层公用的整机根节点。
- `BODY`：保留的底盘 CAD 几何帧。
- `base_link`：Piper 机械臂底座。
- `F_B_R/F_B_L/F_F_L/F_F_R`：真机 CAD 中的四个轮关节。
- `joint1`～`joint6`：Piper 六个关节，名称与真机驱动保持一致。
- `gripper`：夹爪开度控制关节。
- `gripper_tcp`：两指之间的抓取中心，是未来 MoveIt 的末端参考点。

完整参数来源见项目级文档：

- [`docs/robot_model.md`](../../docs/robot_model.md)
- [`config/model_parameters.yaml`](config/model_parameters.yaml)

## Xacro 参数

| 参数 | 默认值 | 作用 |
|---|---|---|
| `sim_mode` | `false` | `true` 使用 Gazebo 硬件后端；`false` 使用 mock hardware |
| `controllers_file` | 空 | Gazebo 模式下加载的控制器 YAML |
| `initial_positions_file` | 包内配置 | 机械臂和夹爪初始关节值 |
| `tcp_offset_xyz` | `0 0 0` | 额外 TCP 平移偏置 |
| `tcp_offset_rpy` | `0 0 0` | 额外 TCP 旋转偏置 |

## 单独查看模型

```bash
cd /home/ubuntu/ScoutFetch
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch scoutfetch_description display.launch.py
```

这个命令只启动：

- `robot_state_publisher`
- `joint_state_publisher_gui`
- RViz

它不会启动 Gazebo，也不会模拟接触、重力和传感器。

## 如何修改模型

1. 几何、关节和 TF 只修改 `urdf/scoutfetch.urdf.xacro`。
2. 可标定参数同时记录到 `config/model_parameters.yaml`。
3. 不要从展开后的 URDF 再复制一份模型。
4. 修改后运行：

```bash
xacro src/scoutfetch_description/urdf/scoutfetch.urdf.xacro \
  sim_mode:=true controllers_file:=/tmp/controllers.yaml >/tmp/mc_arm.urdf
check_urdf /tmp/mc_arm.urdf
colcon test --packages-select scoutfetch_description
```

## 当前限制

- 有效轮半径和麦轮摩擦仍需真机实验辨识。
- CAD 质量未覆盖电池、线缆和控制器等全部载荷。
- 夹爪摩擦、最大夹持力与零位偏差尚未真机标定。
- 基础传感器噪声模型还不是完整的 D435i/Mid-360 硬件模型。
- 当前 D435i 使用便于观察的仿真腕部位姿，并非参数文件中保留的真机手眼标定
  位姿；两者不得混用。
- D435i 外观使用自制简化 box，原无明确授权的网格已移出发布目录，详见 `meshes/sensors/SOURCES.md`。
