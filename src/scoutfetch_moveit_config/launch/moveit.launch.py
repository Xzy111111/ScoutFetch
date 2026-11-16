#!/usr/bin/env python3
"""Shared MoveIt launch for an already-running sim or real robot backend.

This launch owns only move_group and optional RViz. The selected backend must
already own robot_state_publisher, joint states, and arm_controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _launch(context):
    backend = LaunchConfiguration('backend').perform(context)
    joint_states_topic = LaunchConfiguration('joint_states_topic').perform(context)
    if not joint_states_topic:
        joint_states_topic = '/joint_states' if backend == 'sim' else '/feedback/joint_states'

    description_share = get_package_share_directory('scoutfetch_description')
    moveit_share = get_package_share_directory('scoutfetch_moveit_config')
    model_path = os.path.join(description_share, 'urdf', 'scoutfetch.urdf.xacro')

    moveit_config = (
        MoveItConfigsBuilder('car_with_piper', package_name='scoutfetch_moveit_config')
        .robot_description(
            file_path=model_path,
            mappings={'sim_mode': 'false', 'planning_model': 'true'})
        .robot_description_semantic(file_path='config/scoutfetch.srdf.xacro')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    common_remaps = [('joint_states', joint_states_topic)]
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': backend == 'sim',
                'planning_frame': 'base_link',
                'allow_trajectory_execution': True,
                'publish_robot_description_semantic': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'moveit_manage_controllers': False,
            },
        ],
        remappings=common_remaps,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        output='screen',
        arguments=['-d', os.path.join(moveit_share, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': backend == 'sim'},
        ],
        remappings=common_remaps,
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend', default_value='sim', choices=['sim', 'real'],
            description='Select time and default joint-state source; backend must already run.'),
        DeclareLaunchArgument(
            'joint_states_topic', default_value='',
            description='Override joint state source. Empty selects the backend default.'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=_launch),
    ])

