# scoutfetch_navigation

本包提供 MC ARM 的二维建图、地图定位和 Nav2 自主导航配置。机器人几何和物理
后端仍分别属于 `scoutfetch_description` 与 `scoutfetch_sim`。

## 接口契约

```text
TF:       map -> odom -> base_link -> BODY / arm / sensors
扫描:     /livox/scan             sensor_msgs/LaserScan
里程计:   /Odometry              nav_msgs/Odometry
控制:     /cmd_vel                 geometry_msgs/Twist
```

`base_link` 是移动、机械臂规划和任务层的统一整机基准；`BODY` 仅作为底盘 CAD 帧保留。
建图时 SLAM Toolbox 发布 `map -> odom`；加载已有地图时由 AMCL 发布，
二者不能同时启动。

## 构建

```bash
cd /home/ubuntu/ScoutFetch
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 在 Gazebo 中建图

一键启动 Gazebo、SLAM Toolbox 和导航 RViz：

```bash
ros2 launch scoutfetch_navigation sim_mapping.launch.py
```

第一次建图默认不启动 Nav2。另开终端遥控：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ScoutFetch/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

如果桌面环境没有自动弹出 RViz，启动终端现在会打印
`Starting navigation RViz: true`。可在另一个已 source 工作区的桌面终端独立启动同一配置：

```bash
rviz2 -d $(ros2 pkg prefix scoutfetch_navigation)/share/scoutfetch_navigation/rviz/navigation.rviz
```

RViz 配置显示仿真和真机共用的 `/livox/lidar`、`/livox/scan` 和 `/map`。

完成闭环后保存栅格地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/ubuntu/ScoutFetch/src/scoutfetch_navigation/maps/laboratory
```

同时保存可继续编辑/建图的 pose graph：

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/ubuntu/ScoutFetch/src/scoutfetch_navigation/maps/laboratory'}"
```

## 使用已有地图导航

`maps/laboratory.*` 已由 `sim_mapping` 建立（机器人停在原点、朝 +x 时开始建图，
绕原点旋转一周 + 前后移动若干距离）。AMCL 用 `(0,0,0)` 自动初始化
（`config/nav2_params.yaml` 的 `set_initial_pose`），因此无 RViz 的 headless
运行也能直接定位；RViz 的 `2D Pose Estimate` 仍然可以在运行时覆盖。

```bash
ros2 launch scoutfetch_navigation sim_navigation.launch.py \
  map:=/home/ubuntu/ScoutFetch/src/scoutfetch_navigation/maps/laboratory.yaml
```

在 RViz 使用 `2D Pose Estimate` 初始化 AMCL，再使用 `Nav2 Goal` 下发目标。

## 真机 Mid-360 建图

真机参考工程的 `mid360_driver` 发布标准 `sensor_msgs/msg/PointCloud2`：

```text
/livox/lidar -> pointcloud_to_laserscan -> /livox/scan -> slam_toolbox -> /map
```

SLAM Toolbox 本身不负责点云转换。启动底盘驱动、Mid-360 驱动以及提供
`odom -> base_link` 的真机里程计后运行：

```bash
ros2 launch scoutfetch_navigation real_mapping.launch.py
```

可用 `cloud_topic:=...` 覆盖点云输入话题。点云必须是
`sensor_msgs/msg/PointCloud2`；Livox `CustomMsg` 必须先由驱动转换为 PointCloud2。
点云转扫描参数位于 `config/pointcloud_to_laserscan.yaml`。其中高度范围是在
`base_link` 坐标系下计算的，应根据真机安装高度和希望保留的障碍物高度标定。

真机建图入口只启动点云转换、SLAM Toolbox 和 RViz，不会重复启动雷达、底盘或
里程计驱动，避免与真机 bringup 冲突。

真机参考工程当前由 `small_point_lio` 发布 `odom -> base_link` 和 `/Odometry`。
仿真后端现在提供同样的公共契约；上层不应依赖底层控制器的私有里程计话题。

## 建图时同时运行 Nav2

基础建图验证通过后才建议启用：

```bash
ros2 launch scoutfetch_navigation sim_mapping.launch.py navigation:=true
```

## 手动接管仲裁

安装：

```bash
sudo apt install ros-humble-twist-mux
```

启用后，Nav2 发布 `/cmd_vel_nav`，键盘应发布 `/cmd_vel_teleop`，mux 输出公共
`/cmd_vel`。手动控制优先级高于 Nav2：

```bash
ros2 launch scoutfetch_navigation sim_navigation.launch.py \
  map:=/absolute/path/to/map.yaml use_twist_mux:=true

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_teleop
```

当前机器尚未安装 `twist_mux`，因此所有启动文件默认 `use_twist_mux:=false`，基础
建图和导航不受影响。没有启用 mux 时，不要同时运行 Nav2 和键盘遥控。

## 当前边界

- 仿真 `/livox/lidar` 是规则 Gazebo 2D 扫描转换成的平面 `PointCloud2`；话题名和
  消息类型与真机一致，但扫描几何不等价于 Mid-360，不能用于验证 Point-LIO。
- 当前先直接使用麦轮控制器里程计；尚未启用 IMU/轮速 EKF。
- footprint 是保守初值，导航时机械臂应收回运输姿态。
- sim/real 后端都必须维持 `odom -> base_link`、`/Odometry`、`/livox/scan` 契约。
