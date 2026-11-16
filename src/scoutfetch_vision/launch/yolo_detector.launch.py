#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""yolo_detector launch: YOLO (ONNX) 目标检测节点。

它是 localization_node `pixel_source:=yolo` 的检测来源: 订阅彩色图, 发布
vision_msgs/Detection2DArray。不启动相机后端 (仿真: scoutfetch_sim; 真机: 真实 D435i)。

模型与类别文件默认取本包 share/models 的安装副本; 真机换模型时覆盖:
    ros2 launch scoutfetch_vision yolo_detector.launch.py \\
        model_path:=/abs/path/to/yolo.onnx classes_path:=/abs/path/to/classes.txt
类别名/阈值在 config/yolo_detector.yaml 里改 (target_classes 等)。

依赖: pip 安装的 onnxruntime (无 rosdep key), numpy, opencv-python。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("scoutfetch_vision")
    models_dir = os.path.join(pkg_dir, "models")

    declared_args = [
        DeclareLaunchArgument(
            "config_file", default_value="yolo_detector.yaml",
            description="Config file name inside scoutfetch_vision/config",
        ),
        DeclareLaunchArgument(
            "model_path", default_value=os.path.join(models_dir, "yolov8n.onnx"),
            description="YOLO ONNX model file (absolute path; real robot: its own export)",
        ),
        DeclareLaunchArgument(
            "classes_path", default_value=os.path.join(models_dir, "coco_classes.txt"),
            description="Class-name file, one per line (real robot: its own export)",
        ),
        DeclareLaunchArgument(
            "image_topic", default_value="/camera/camera/color/image_raw",
            description="Colour image topic; must match localization_node's color_topic",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", choices=["true", "false"],
            description="Use the Gazebo/rosbag simulated clock (sim: true; real: false)",
        ),
        DeclareLaunchArgument("publish_annotated", default_value="true", choices=["true", "false"],
                              description="Publish a live YOLO image with boxes, labels and scores"),
        DeclareLaunchArgument("detections_topic", default_value="/detections"),
        DeclareLaunchArgument("annotated_topic", default_value="/camera/yolo/image_annotated"),
    ]

    detector = Node(
        package="scoutfetch_vision",
        executable="yolo_detector.py",
        name="yolo_detector",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                pkg_dir, "config",
                LaunchConfiguration("config_file"),
            ]),
            {"model_path": LaunchConfiguration("model_path")},
            {"classes_path": LaunchConfiguration("classes_path")},
            {"image_topic": LaunchConfiguration("image_topic")},
            {"detections_topic": LaunchConfiguration("detections_topic")},
            {'publish_annotated': LaunchConfiguration('publish_annotated')},
            {'annotated_topic': LaunchConfiguration('annotated_topic')},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(declared_args + [detector])
