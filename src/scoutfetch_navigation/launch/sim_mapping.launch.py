import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_share = get_package_share_directory('scoutfetch_sim')
    navigation_share = get_package_share_directory('scoutfetch_navigation')
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'sim.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui'), 'rviz': 'false'}.items())
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'navigation': LaunchConfiguration('navigation'),
            'use_twist_mux': LaunchConfiguration('use_twist_mux'),
            'rviz': LaunchConfiguration('rviz'),
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('navigation', default_value='false'),
        DeclareLaunchArgument('use_twist_mux', default_value='false'),
        GroupAction(scoped=True, actions=[simulator]),
        GroupAction(scoped=True, actions=[mapping]),
    ])
