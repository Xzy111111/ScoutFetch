import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    share = get_package_share_directory('scoutfetch_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')
    slam_share = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_navigation = LaunchConfiguration('navigation')
    use_twist_mux = LaunchConfiguration('use_twist_mux')
    slam_params = LaunchConfiguration('slam_params_file')
    nav2_params = LaunchConfiguration('nav2_params_file')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items())

    def navigation_include():
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
            condition=IfCondition(run_navigation),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params,
                'use_composition': 'False',
            }.items())

    navigation_via_mux = GroupAction(
        condition=IfCondition(use_twist_mux),
        actions=[
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
            navigation_include(),
        ])
    navigation_without_mux = GroupAction(
        condition=UnlessCondition(use_twist_mux),
        actions=[navigation_include()])

    mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        output='screen', condition=IfCondition(use_twist_mux),
        parameters=[os.path.join(share, 'config', 'twist_mux.yaml')],
        remappings=[('cmd_vel_out', '/cmd_vel')])

    rviz = Node(
        package='rviz2', executable='rviz2', name='navigation_rviz',
        output='screen', condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(share, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('navigation', default_value='false',
                              description='Run Nav2 while mapping; false is safer for first maps'),
        DeclareLaunchArgument('use_twist_mux', default_value='false',
                              description='Requires ros-humble-twist-mux'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(share, 'config', 'slam_toolbox.yaml')),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        LogInfo(msg=['Starting navigation RViz: ', LaunchConfiguration('rviz')]),
        slam, navigation_via_mux, navigation_without_mux, mux, rviz,
    ])
