#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""localization.launch.py — object localization node only.

The localization node is the shared (sim/real) upstream of the task pipeline. It
emits /target_pose (base_link frame) and /target_pose_map (map frame), feeding
scoutfetch_reach / scoutfetch_mission. MoveIt, navigation, scoutfetch_grasp and pick are
separate stages and are launched by their own packages.

Prereq: a running backend that publishes the camera topics and the
camera_color_optical_frame -> base_link TF chain (real: hand-eye TF; sim: the
scoutfetch_sim backend).

Usage:
    ros2 launch scoutfetch_vision localization.launch.py pixel_source:=yaml
    ros2 launch scoutfetch_vision localization.launch.py pixel_source:=click
    # yolo: 用 yolo_detector 的 /detections 取目标像素, 与机器人位姿无关
    ros2 launch scoutfetch_vision localization.launch.py pixel_source:=yolo
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("scoutfetch_vision")

    declared_args = [
        DeclareLaunchArgument(
            "config_file", default_value="localization.yaml",
            description="Config file name inside scoutfetch_vision/config",
        ),
        DeclareLaunchArgument(
            "pixel_source", default_value="yaml", choices=["yaml", "click", "yolo"],
            description="yaml = fixed pixel; click = OpenCV window left-click; "
                        "yolo = take the highest-confidence box centre from a "
                        "vision_msgs/Detection2DArray (pose-independent)",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", choices=["true", "false"],
            description="Use the Gazebo/rosbag simulated clock. Must match the backend: "
                        "the node stamps messages and looks up TF on this clock "
                        "(sim: true; real: false)",
        ),
    ]

    localization_node = Node(
        package="scoutfetch_vision",
        executable="localization_node",
        name="localization_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                pkg_dir, "config",
                LaunchConfiguration("config_file"),
            ]),
            {"pixel_source": LaunchConfiguration("pixel_source")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(declared_args + [localization_node])