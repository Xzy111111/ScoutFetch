"""AWS bookstore adapted for keyboard/autonomous supported-bottle pickup."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('scoutfetch_mission'), 'launch', 'sim_bottle_search.launch.py')),
            launch_arguments={'scene': 'bookstore', 'rviz': LaunchConfiguration('rviz')}.items()),
    ])
