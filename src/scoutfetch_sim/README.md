# scoutfetch_sim

## 这个功能包做什么

`scoutfetch_sim` 是机器人的**Gazebo 物理仿真后端功能包**。它读取
`scoutfetch_description` 提供的唯一整机模型，然后负责：

- 启动 Gazebo Fortress；
- 创建机器人和拾取测试场景；
- 加载 `gz_ros2_control`；
- 启动麦轮底盘、机械臂和夹爪控制器；
- 将 Gazebo 传感器消息桥接成 ROS 2 消息；
- 保持 `/cmd_vel` 和 FollowJointTrajectory 等公共控制接口稳定。

这个包**不定义另一套机器人几何模型**，也暂时不负责 Nav2、MoveIt、目标检测、
Point-LIO 或完整抓取任务。

## 与模型包的关系

```text
scoutfetch_description
  提供 robot_description、网格、关节、TF、传感器安装位置
                         ↓
scoutfetch_sim
  提供世界、物理引擎、控制器、传感器数据和 ROS/Gazebo 桥接
                         ↓
未来上层包
  Nav2、MoveIt、感知、抓取、任务编排
```

## 目录说明

```text
scoutfetch_sim/
├── config/
│   ├── controllers.yaml    ros2_control 控制器和底盘运动学参数
│   └── bridge.yaml         Gazebo Transport 到 ROS 2 的传感器桥接
├── launch/
│   └── sim.launch.py       完整仿真入口
├── worlds/
│   └── pick_place.sdf      室内实验室、实心椅子和圆柱抓取物场景
├── test/
│   └── test_config_contract.py  检查控制器和话题契约
├── CMakeLists.txt
└── package.xml
```

## 启动内容

运行 `sim.launch.py` 后会启动：

| 组件 | 作用 |
|---|---|
| Gazebo server | 重力、碰撞、接触和传感器仿真 |
| Gazebo GUI | 显示仿真世界，可用 `gui:=false` 关闭 |
| `robot_state_publisher` | 发布机器人内部 TF |
| `ros_gz_sim create` | 从 `robot_description` 创建机器人 |
| `gz_ros2_control` | 把 ROS 控制命令作用到 Gazebo 关节 |
| `joint_state_broadcaster` | 发布所有关节状态 |
| `base_controller` | 四轮麦克纳姆底盘控制和里程计 |
| `arm_controller` | Piper 六关节轨迹控制 |
| `gripper_controller` | 夹爪开度轨迹控制 |
| `ros_gz_bridge` | 桥接时钟、雷达、IMU、相机和点云 |
| `cmd_vel_adapter` | 将 `/cmd_vel` 转给底盘控制器 |
| `odom_tf_adapter` | 将控制器私有 TF 话题转发到标准 `/tf` |
| RViz | 显示 RobotModel 和 TF，可用 `rviz:=false` 关闭 |

## 启动方法

```bash
cd /home/ubuntu/ScoutFetch
source /opt/ros/humble/setup.bash
source install/setup.bash

# 带 Gazebo GUI 和 RViz
ros2 launch scoutfetch_sim sim.launch.py

# 无界面运行，适合测试
ros2 launch scoutfetch_sim sim.launch.py gui:=false rviz:=false
```

## 对外控制接口

### 1. 麦轮底盘

公共输入保持和真机一致：

```text
/cmd_vel    geometry_msgs/msg/Twist
```

示例：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}"
```

底盘控制器发布里程计和 `odom -> base_link` TF。停止测试时应停止命令发布，
控制器有 0.5 秒超时保护。

### 2. Piper 机械臂

```text
/arm_controller/follow_joint_trajectory
control_msgs/action/FollowJointTrajectory
```

控制关节为 `joint1`～`joint6`。未来 MoveIt 应直接使用这个 Action；切换真机时，
由真机适配层提供相同 Action，而不是改动 MoveIt 或任务代码。

### 3. 夹爪

```text
/gripper_controller/follow_joint_trajectory
control_msgs/action/FollowJointTrajectory
```

控制关节为 `gripper`，范围为 0～0.1 m。当前为位置控制模型，真实夹持力和接触
反馈仍需继续实现。

## 传感器接口

| ROS 2 话题 | 类型 | 当前用途 |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | 仿真时间 |
| `/livox/scan` | `sensor_msgs/LaserScan` | Nav2 二维避障公共输入 |
| `/Odometry` | `nav_msgs/Odometry` | `odom` 中的 `base_link` 位姿和速度 |
| `/livox/imu` | `sensor_msgs/Imu` | 仿真 IMU |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像 |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色相机内参 |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 与彩色像素网格对齐的深度图 |
| `/camera/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | 对齐深度内参 |
| `/camera/camera/depth/color/points` | `sensor_msgs/PointCloud2` | MoveIt OctoMap/三维感知 |

重要限制：当前 `/livox/scan` 是规则 Gazebo GPU LiDAR 的适配输出。它不是 Mid-360 非重复扫描，
也没有提供与真机 `custom_mid360_driver` 完全相同的 `/livox/lidar` 数据。因此目前
可以用于 Nav2 基础验证，但不能声称已完成 Point-LIO 的传感器级 sim-to-real。

麦轮控制器原生把 `odom -> base_link` TF 发布在私有话题
`/base_controller/tf_odometry`。启动文件通过 `odom_tf_adapter` 将其转入标准
`/tf`，供 SLAM Toolbox 和 Nav2 使用；不要再启动第二个 `odom -> base_link` 发布者。

## 仿真场景

`worlds/pick_place.sdf` 当前包含：

- 6 m × 5 m 的开放式机器人实验室；
- 浅灰防滑地面、前墙和两侧墙，观察侧保持开放；
- 一组方向光和两盏室内点光源；
- 一张固定的低矮实心椅子；
- 一个放在椅面上、质量为 0.15 kg 的红色圆柱抓取物；
- 明确定义的 Ogre2 ambient、diffuse、specular 材质；
- 墙面蓝色视觉标识，用于给相机提供可辨认的环境特征。

场景是基础室内验证环境。之后可增加货架、放置区和不同物理属性的物体，
但不要把场景模型写进机器人 Xacro。

## 验证

```bash
colcon test --packages-select scoutfetch_sim
colcon test-result --verbose
```

运行时还应检查：

```bash
ros2 control list_controllers
ros2 topic list
ros2 topic hz /joint_states
ros2 topic hz /livox/scan
ros2 topic hz /livox/imu
```

期望四个控制器均为 `active`：

- `joint_state_broadcaster`
- `base_controller`
- `arm_controller`
- `gripper_controller`

## 关联模块与范围

导航、视觉、机械臂和任务编排已有独立功能包，完整任务使用
[scoutfetch_mission](../scoutfetch_mission/README.md) 的统一入口。
当前雷达仿真为二维扫描转换，物理参数和传感器噪声的范围见
[模型与接口说明](../../docs/robot_model.md)。
