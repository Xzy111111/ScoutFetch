import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('scoutfetch_navigation')
    cloud_topic = LaunchConfiguration('cloud_topic')

    cloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[LaunchConfiguration('pointcloud_params_file')],
        remappings=[('cloud_in', cloud_topic), ('scan', '/livox/scan')],
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'navigation': 'false',
            'use_twist_mux': LaunchConfiguration('use_twist_mux'),
            'rviz': LaunchConfiguration('rviz'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_twist_mux', default_value='false'),
        DeclareLaunchArgument(
            'pointcloud_params_file',
            default_value=os.path.join(
                share, 'config', 'pointcloud_to_laserscan.yaml')),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                share, 'config', 'slam_toolbox_real.yaml')),
        LogInfo(msg='Real mapping: /livox/lidar -> /livox/scan -> slam_toolbox'),
        cloud_to_scan,
        mapping,
    ])
