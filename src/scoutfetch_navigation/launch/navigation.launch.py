import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    share = get_package_share_directory('scoutfetch_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    use_twist_mux = LaunchConfiguration('use_twist_mux')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_composition': 'False',
        }.items())

    def navigation_include():
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'use_composition': 'False',
            }.items())
    navigation_via_mux = GroupAction(
        condition=IfCondition(use_twist_mux),
        # Node-specific rules precede Nav2's local remaps. Keep the smoother's
        # input distinct from its output; renaming only /cmd_vel leaves its
        # cmd_vel_smoothed -> cmd_vel rule bypassing the mux.
        actions=[
            SetRemap(src='controller_server:cmd_vel', dst='/cmd_vel_nav_raw'),
            SetRemap(src='velocity_smoother:cmd_vel', dst='/cmd_vel_nav_raw'),
            SetRemap(src='velocity_smoother:cmd_vel_smoothed', dst='/cmd_vel_nav'),
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
            navigation_include()])
    navigation_without_mux = GroupAction(
        condition=UnlessCondition(use_twist_mux), actions=[navigation_include()])

    mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        output='screen', condition=IfCondition(use_twist_mux),
        parameters=[os.path.join(share, 'config', 'twist_mux.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', '/cmd_vel_unfiltered')])
    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen', condition=IfCondition(use_twist_mux),
        parameters=[os.path.join(share, 'config', 'collision_monitor.yaml'),
                    {'use_sim_time': use_sim_time}])
    collision_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_collision', output='screen', condition=IfCondition(use_twist_mux),
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['collision_monitor']}])
    rviz = Node(
        package='rviz2', executable='rviz2', name='navigation_rviz',
        output='screen', condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(share, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to a saved map YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_twist_mux', default_value='false',
                              description='Requires ros-humble-twist-mux'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        localization, navigation_via_mux, navigation_without_mux, mux,
        collision_monitor, collision_lifecycle, rviz,
    ])
