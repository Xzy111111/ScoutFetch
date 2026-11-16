#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""pick_node launch (resident node + /execute_pick Trigger service).

Assumes MoveIt (move_group) is already running from a selected backend. This
launch only starts pick_node, and loads the shared scoutfetch_moveit_config robot
model (read-only) so the local model matches move_group exactly.

Triggers:
    ros2 topic pub --once /grasp_target_pose geometry_msgs/msg/PoseStamped "{...}"
    ros2 service call /execute_pick std_srvs/srv/Trigger "{}"

Safety gates:
    execute:=false                   -> /execute_pick is dry-run only
    execute:=true + allow:=false     -> /execute_pick rejects real execution
    execute:=true + allow:=true      -> permit real execution (still explicit trigger)
    sim (default): joint_states_topic=/joint_states
    real:          joint_states_topic=/feedback/joint_states
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
    # Reuse the shared scoutfetch_moveit_config robot model (read-only, no geometry
    # duplication), matching move_group so step validation uses the same model.
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
    pkg_dir = get_package_share_directory("scoutfetch_grasp")

    declared_args = [
        DeclareLaunchArgument(
            "execute", default_value="false", choices=["true", "false"],
            description="false: dry-run (no real motion / no gripper cmd); true: execute",
        ),
        DeclareLaunchArgument(
            "allow_real_execution", default_value="false", choices=["true", "false"],
            description="Second safety gate. Only execute:=true AND allow_real_execution:=true "
                        "permit real execution via /execute_pick. Default false (never true in "
                        "default config)",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true", choices=["true", "false"],
            description="Whether to start RViz with the grasp marker config",
        ),
        DeclareLaunchArgument(
            "config_file", default_value="pick_and_place.yaml",
            description="Config file name inside scoutfetch_grasp/config",
        ),
        DeclareLaunchArgument(
            "joint_states_topic", default_value="/joint_states",
            description="Joint states topic for MoveGroupInterface current-state monitor "
                        "(sim: /joint_states; real: /feedback/joint_states)",
        ),
        DeclareLaunchArgument(
            "return_to_ready", default_value="true", choices=["true", "false"],
            description="true: after lifting, fold the arm back to the 'ready' pose; "
                        "false: keep the lift (carry) pose so a grasped object is not "
                        "dragged against the robot during folding",
        ),
        DeclareLaunchArgument(
            "return_to_stow", default_value="false", choices=["true", "false"],
            description="true: after lifting, fold the arm to carry_stow_joints "
                        "(a carry-safe pose that keeps the held object clear of the "
                        "robot). Takes precedence over return_to_ready",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", choices=["true", "false"],
            description="Use the Gazebo/rosbag simulated clock. Must match the backend: "
                        "the MoveGroupInterface current-state monitor rejects joint_states "
                        "stamped on a different clock (sim: true; real: false)",
        ),
    ]

    pick_node = Node(
        package="scoutfetch_grasp",
        executable="pick_node",
        name="pick_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                pkg_dir, "config",
                LaunchConfiguration("config_file"),
            ]),
            {"execute": LaunchConfiguration("execute")},
            {"allow_real_execution": LaunchConfiguration("allow_real_execution")},
            {"return_to_ready": LaunchConfiguration("return_to_ready")},
            {"return_to_stow": LaunchConfiguration("return_to_stow")},
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
        arguments=["-d", os.path.join(pkg_dir, "rviz", "grasp.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(declared_args + [pick_node, rviz_node])