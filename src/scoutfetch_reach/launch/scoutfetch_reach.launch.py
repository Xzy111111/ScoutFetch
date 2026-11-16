#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""scoutfetch_reach node launch (shared sim/real backend).

Assumes Nav2 (global_costmap) and MoveIt (move_group) are already running. This
launch only starts scoutfetch_reach_node, and initialises MoveGroupInterface
with the shared scoutfetch_moveit_config robot model (read-only).

Usage:
    ros2 launch scoutfetch_reach scoutfetch_reach.launch.py use_rviz:=true
    # real backend: joint_states_topic:=/feedback/joint_states
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _moveit_config():
    # Reuse the shared scoutfetch_moveit_config robot model so this node's local
    # robot model matches move_group exactly (read-only, no geometry duplication).
    description_share = get_package_share_directory("scoutfetch_description")
    model_path = os.path.join(description_share, "urdf", "scoutfetch.urdf.xacro")
    builder = (
        MoveItConfigsBuilder("car_with_piper", package_name="scoutfetch_moveit_config")
        .robot_description(
            file_path=model_path,
            mappings={"sim_mode": "false", "planning_model": "true"},
        )
        .robot_description_semantic(file_path="config/scoutfetch.srdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
    )
    config = builder.to_moveit_configs()
    config.sensors_3d = {}  # no octomap
    return config


def generate_launch_description():
    pkg_dir = get_package_share_directory("scoutfetch_reach")

    declared_args = [
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_dir, 'config', 'scoutfetch_reach.yaml')),
        DeclareLaunchArgument(
            "use_rviz", default_value="true", choices=["true", "false"],
            description="Whether to start RViz",
        ),
        DeclareLaunchArgument(
            "joint_states_topic", default_value="/joint_states",
            description="Joint states topic for the MoveGroup current-state monitor "
                        "(sim: /joint_states; real: /feedback/joint_states)",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", choices=["true", "false"],
            description="Use the Gazebo/rosbag simulated clock. Must match the backend: "
                        "the MoveGroup current-state monitor rejects joint_states stamped "
                        "on a different clock (sim: true; real: false)",
        ),
    ]

    node = Node(
        package="scoutfetch_reach",
        executable="scoutfetch_reach_node",
        name="scoutfetch_reach_node",
        output="screen",
        parameters=[
            LaunchConfiguration('params_file'),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            _moveit_config().to_dict(),
        ],
        remappings=[
            ("joint_states", LaunchConfiguration("joint_states_topic")),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(declared_args + [node, rviz_node])
