# scoutfetch_vision

使用 YOLO 检测框或人工像素、对齐深度和 TF 计算目标三维位置。
自动任务使用 YOLO；固定像素和鼠标点击供定位调试。

## 启动

需要后端已提供彩色图、对齐深度、CameraInfo 和完整相机 TF。
以下两条命令分别启动检测和定位；完整任务入口已包含它们，无需重复启动。

```bash
ros2 launch scoutfetch_vision yolo_detector.launch.py use_sim_time:=true
ros2 launch scoutfetch_vision localization.launch.py pixel_source:=yolo use_sim_time:=true

# 定位调试：鼠标选点，或 pixel_source:=yaml 使用固定像素
ros2 launch scoutfetch_vision localization.launch.py pixel_source:=click use_sim_time:=true

# 查看推理使用的实时标注图像
ros2 run rqt_image_view rqt_image_view /camera/yolo/image_annotated
```

`localization.launch.py` 只启动定位节点，点击或检测不会直接触发抓取。
实机连接使用 `use_sim_time:=false`。

## 数据链与接口

`/detections` + 对齐深度 → 相机坐标 → TF → `/target_pose_map` → 站位规划与任务编排。

| 方向 | 话题 | 内容 |
|---|---|---|
| 输入 | `/camera/camera/color/image_raw` | YOLO / 点击模式的彩色图像 |
| 输入 | `/camera/camera/aligned_depth_to_color/image_raw` | 对齐深度，支持 16UC1 毫米和 32FC1 米 |
| 输入 | `/camera/camera/aligned_depth_to_color/camera_info` | 相机内参 |
| 检测输出 / 定位输入 | `/detections` | `Detection2DArray`，原始 RGB 时间戳 |
| 输出 | `/camera/yolo/image_annotated` | 检测框、类别、置信度；空帧不保留旧框 |
| 输出 | `/target_pose_map` | map 系目标，reliable / transient_local / KeepLast(1) |
| 调试输出 | `/target_pose`、`/target_marker` | base_link 系目标和可视化 |

RGB / 深度通过有限时间差匹配，目标保留 RGB 检测时间戳。没有新鲜有效检测、深度
或所需 TF 时，不发布对应目标。`/target_pose` 不作为正式抓取服务的输入。

## 模型与参数

通用检测 launch 默认 YOLOv8n，统一随机搜索入口选择 YOLOv8s。
模型与类别文件从包的 `models/` 加载，也可通过 `model_path`、`classes_path` 覆盖。

| 配置 | 主要参数 |
|---|---|
| [localization.yaml](config/localization.yaml) | pixel_source、相机话题、检测超时、深度范围、抓取像素偏移 |
| [yolo_detector.yaml](config/yolo_detector.yaml) | 类别、置信度、NMS 和输入大小 |
| 定位 launch | config_file、pixel_source、use_sim_time |
| 检测 launch | model_path、classes_path、image_topic、detections_topic、annotated_topic、publish_annotated |

检测器接受标准 YOLOv8 ONNX 输入 `1×3×640×640`、输出 `1×(4+nc)×N`。
换用其他输出格式的模型，需要调整后处理。推理使用 ONNX Runtime：

```bash
python3 -m pip install --user --no-deps onnxruntime==1.16.3
```

配置 `publish_annotated:=false` 可关闭检测器标注输出；`detection_overlay.py` 是可选的
独立叠图工具，默认任务不启动它。

## 排查与许可

定位不输出时依次检查相机数据、对齐深度、检测新鲜度以及 map / base_link / camera TF。
自动导航后不要使用固定像素模式，固定像素不会追踪瓶子。

自编代码采用 [Apache-2.0](LICENSE)。YOLO 权重使用其上游许可，见
[第三方清单](../../THIRD_PARTY_NOTICES.md) 与 `models/` 中的元数据。
