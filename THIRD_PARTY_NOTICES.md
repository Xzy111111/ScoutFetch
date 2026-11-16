# 许可范围与第三方归属

记录日期：2026-11-16。根目录 [LICENSE](LICENSE) 是 Apache-2.0 正文，适用于
有权按此许可发布的 ScoutFetch 自有贡献。它不覆盖另附许可的内容，也不会把来源未明
的内容自动变成 Apache-2.0。归属和例外同时见 [NOTICE](NOTICE)。

## 逐项清单

| 路径 / 内容 | 来源与已查到的许可 | 当前处理 |
|---|---|---|
| `scoutfetch_description`、`scoutfetch_sim`、`scoutfetch_navigation`、`scoutfetch_moveit_config`、`scoutfetch_mission` 自有代码 | 各包原已声明 Apache-2.0 | 根目录提供完整正文，第三方网格与场景按下列例外处理 |
| `scoutfetch_vision`、`scoutfetch_reach`、`scoutfetch_grasp` 自有代码 | 维护者于 2026-11-16 确认自行编写，按 Apache-2.0 标注 | 已统一 package.xml、源码 SPDX 和各包 LICENSE；模型权重等第三方内容仍按各自条款处理 |
| `src/scoutfetch_interfaces` 六个消息 | 本地实机来源包声明 Apache-2.0；父工程同时附有 MIT 许可 | 保留包声明，并记录父工程许可；不覆盖原始归属 |
| `src/scoutfetch_sim/models/aws_bookstore/` | AWS RoboMaker Bookstore，固定提交 `ed542eb41809d5c5f1c67f32d4ba24e0e90886b3` | 原样保留上游 [LICENSE](src/scoutfetch_sim/models/aws_bookstore/LICENSE) 与 [SOURCE.json](src/scoutfetch_sim/models/aws_bookstore/SOURCE.json)；另存 [正文副本](LICENSES/AWS-Bookstore.txt) |
| `src/scoutfetch_description/meshes/piper/` | 本地 `agx_arm_ros/agx_arm_description/agx_arm_urdf`，附 MIT、Copyright (c) 2026 aalicecc | 22 个网格与该目录下模型逐个 SHA-256 一致，保留 [许可](LICENSES/Piper-Local-Upstream-MIT.txt) 和 [匹配记录](docs/model_provenance.json)；本地来源证据不等于已确认远程原始作者 |
| `src/scoutfetch_description/meshes/chassis/` 与组合 URDF | 维护者说明：基于 RoboMaster 哨兵底盘与松灵机械臂改装；直接导入源为本地 `car_sim` | 原始底盘/机械臂仓库、具体版本和适用授权仍待补充；上游原始链接与版本仍待补充 |
| 相机简化 box visual | ScoutFetch 自制几何 | Apache-2.0；原无明确授权的 D435i 网格已移出项目 |
| `src/scoutfetch_vision/models/yolov8*.onnx` | Ultralytics YOLOv8 模型 | 随附 [AGPL-3.0 正文](LICENSES/AGPL-3.0.txt) 及 n/s 模型元数据，未更改为 Apache |
| COCO 类别表 | YOLO 的 COCO 类别映射 | 只含类别名，不包含 COCO 图像或训练数据集 |

## 上游链接与再分发说明

- AWS 场景：[固定提交的许可证](https://github.com/aws-robotics/aws-robomaker-bookstore-world/blob/ed542eb41809d5c5f1c67f32d4ba24e0e90886b3/LICENSE)。场景生成器会移除部分家具、生成取物台和地图；生成场景及包含这些素材的截图应保留上游归属。
- YOLO：[官方许可说明](https://www.ultralytics.com/license)、[导出所用 v8.4.175 版本的许可](https://github.com/ultralytics/ultralytics/blob/v8.4.175/LICENSE)。提供 ONNX 格式不意味着上游许可消失。使用、分发模型和组合应用时，须按适用条款处理对应源码等义务；本清单不能代替这些义务。没有记录已取得 Enterprise 授权。
- YOLOv8s：[原始检查点、导出环境和摘要](src/scoutfetch_vision/models/yolov8s.metadata.json)；YOLOv8n：[历史导出记录及已知缺项](src/scoutfetch_vision/models/yolov8n.metadata.json)。
- 未在本仓库内分发的 ROS 2、Nav2、MoveIt、Gazebo、ONNX Runtime、OpenCV、Qt 等系统依赖遵守各自许可；制作包含这些依赖的二进制 / 容器发布时，应另外保留对应归属与许可。

## 尚待确认

自编代码的许可标注已确认。RoboMaster 哨兵底盘、松灵机械臂原始模型的具体版本、授权和 Piper 本地来源链仍需进一步核对。
未确认前不将完整仓库描述为“全部 Apache-2.0”或“授权审查已通过”。
核查过程文档由维护者本地保存，本清单列出当前适用范围与待确认项。

## 改装模型与展示素材

组合 URDF 基于维护者提供的 RoboMaster 哨兵底盘与松灵机械臂模型改装，直接导入源为
本地 `car_sim` / `agx_arm_urdf`。项目完成整机安装、传感器、公共坐标系及仿真/规划适配；
上游几何保留各自权利。原始仓库、型号、版本与修改链仍待补充，不根据相邻自编代码的
许可证推定原始网格授权。

README 的机器人图为加载项目 Xacro/URDF 后的 RViz 实际视图截图，相机外观与 L 型安装支架为自制简化几何；定位/导航地图由
场景生成器导出（seed=7、station=2）；书店截图使用上述 AWS 场景素材。
