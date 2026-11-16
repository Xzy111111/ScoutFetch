#!/usr/bin/env python3
"""scoutfetch_mission launch: the three orchestration nodes.

nav_goal_executor (dry-run by default), target_pose_resolver, task_fsm.

Usage:
    ros2 launch scoutfetch_mission scoutfetch_mission.launch.py
    ros2 launch scoutfetch_mission scoutfetch_mission.launch.py enable_navigation:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    enable_navigation = (
        LaunchConfiguration("enable_navigation").perform(context).lower() == "true"
    )
    search_enabled = LaunchConfiguration("search_enabled").perform(context).lower() == "true"
    exploration_enabled = LaunchConfiguration("exploration_enabled").perform(context).lower() == "true"
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    if search_enabled and not enable_navigation:
        raise RuntimeError("search_enabled requires enable_navigation:=true")
    if exploration_enabled and not search_enabled:
        raise RuntimeError('exploration_enabled requires search_enabled:=true')
    nav_goal_executor = Node(
        package="scoutfetch_mission",
        executable="nav_goal_executor",
        name="nav_goal_executor",
        output="screen",
        parameters=[{"enable_navigation": enable_navigation, "use_sim_time": use_sim_time}],
    )
    target_pose_resolver = Node(
        package="scoutfetch_mission",
        executable="target_pose_resolver",
        name="target_pose_resolver",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    task_fsm = Node(
        package="scoutfetch_mission",
        executable="task_fsm",
        name="task_fsm",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "search_enabled": search_enabled,
                     "center_target_before_confirm": search_enabled,
                     "grasp_monitor_enabled": LaunchConfiguration('grasp_monitor_enabled').perform(context).lower() == 'true',
                     "exploration_enabled": exploration_enabled,
                     "search_max_turns": int(LaunchConfiguration("search_max_turns").perform(context)),
                     "search_continuous_rotation": LaunchConfiguration('search_continuous_rotation').perform(context).lower() == 'true',
                     "search_angular_speed": float(LaunchConfiguration('search_angular_speed').perform(context)),
                     "search_timeout_sec": float(LaunchConfiguration("search_timeout_sec").perform(context))}],
    )
    return [nav_goal_executor, target_pose_resolver, task_fsm]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("search_enabled", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument('grasp_monitor_enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('exploration_enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument("search_max_turns", default_value="1", description="0: keep searching while no target is detected"),
        DeclareLaunchArgument('search_continuous_rotation', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('search_angular_speed', default_value='0.25', description='Search angular speed, rad/s'),
        DeclareLaunchArgument("search_timeout_sec", default_value="180.0", description="0: no overall search deadline; sensor watchdogs remain active"),
        DeclareLaunchArgument(
            "enable_navigation",
            default_value="false",
            choices=["true", "false"],
            description="false: nav_goal_executor dry-run (no Nav2 goal, no motion); "
                        "true: send real NavigateToPose goals",
        ),
        OpaqueFunction(function=_launch),
    ])
