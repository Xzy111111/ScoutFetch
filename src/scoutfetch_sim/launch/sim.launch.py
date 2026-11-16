import os
import importlib.util
import tempfile
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    desc_share = FindPackageShare('scoutfetch_description')
    sim_share = FindPackageShare('scoutfetch_sim')
    controllers_path = os.path.join(get_package_share_directory('scoutfetch_sim'), 'config', 'controllers.yaml')
    model_path = os.path.join(get_package_share_directory('scoutfetch_description'), 'urdf', 'scoutfetch.urdf.xacro')
    robot_description = xacro.process_file(model_path, mappings={
        'sim_mode': 'true', 'controllers_file': controllers_path,
    }).toxml()
    helper_path = os.path.join(get_package_share_directory('scoutfetch_sim'), 'scripts', 'sim_model.py')
    spec = importlib.util.spec_from_file_location('mc_arm_sim_model', helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    model_sdf = helper.prepare_sim_model(robot_description, tempfile.mkdtemp(prefix='mc_arm_model_'))
    params = {'robot_description': robot_description, 'use_sim_time': True}

    # Fortress's upstream launch uses shell=True. Launch then tracks /bin/sh,
    # and escalating shutdown can kill that shell while leaving the actual
    # Gazebo server alive. Track the Ruby/ign process itself so signals and
    # SIGKILL escalation reach the process which owns physics/controllers.
    # Preserve the upstream launch's plugin search paths: the ros2_control
    # system plugin lives in the sourced ROS library directory.
    plugin_env = {
        key: os.pathsep.join(filter(None, [os.environ.get(key, ''),
                                          os.environ.get('LD_LIBRARY_PATH', '')]))
        for key in ('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', 'GZ_SIM_SYSTEM_PLUGIN_PATH')
    }
    gazebo = ExecuteProcess(
        cmd=['ruby', FindExecutable(name='ign'), 'gazebo', '-r', '-s',
             LaunchConfiguration('world'), '--force-version', '6'],
        name='gazebo_server', output='screen', shell=False, additional_env=plugin_env)
    gazebo_gui = ExecuteProcess(
        cmd=['ruby', FindExecutable(name='ign'), 'gazebo', '-g', '--force-version', '6'],
        name='gazebo_gui', output='screen', shell=False, additional_env=plugin_env,
        condition=IfCondition(LaunchConfiguration('gui')))
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               name='sim_robot_state_publisher', parameters=[params])
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', 'mc_arm', '-file', model_sdf,
                            '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'),
                            '-Y', LaunchConfiguration('spawn_yaw'), '-z', '0.148872'])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': PathJoinSubstitution([sim_share, 'config', 'bridge.yaml'])}])
    lidar_adapter_params = PathJoinSubstitution([sim_share, 'config', 'lidar_adapter.yaml'])
    scan_filter = Node(
        package='scoutfetch_sim', executable='scan_self_filter.py',
        output='screen', parameters=[{'use_sim_time': True}])
    scan_to_cloud = Node(
        package='pointcloud_to_laserscan', executable='laserscan_to_pointcloud_node',
        name='sim_laserscan_to_pointcloud', output='screen',
        parameters=[lidar_adapter_params],
        remappings=[('scan_in', '/livox/scan'), ('cloud', '/livox/lidar')])
    relay = Node(package='topic_tools', executable='relay', name='cmd_vel_adapter',
                 arguments=['/cmd_vel', '/base_controller/reference_unstamped'])
    # The base controller publishes its odom->base_link TF with TRANSIENT_LOCAL
    # durability (TF convention). topic_tools relay defaults to a VOLATILE
    # subscription, which does NOT match a TRANSIENT_LOCAL publisher in Fast-DDS,
    # so odom->base_link silently never reaches /tf. Without it AMCL cannot
    # transform /livox/scan into the odom frame and drops every scan
    # ("timestamp on the message is earlier than all the data in the transform
    # cache"), so the robot never localizes. Pin the relay to the publisher's
    # QoS (transient_local + keep_all + reliable).
    odom_tf_relay = Node(package='topic_tools', executable='relay', name='odom_tf_adapter',
                         arguments=['/base_controller/tf_odometry', '/tf',
                                    '--qos-durability', 'transient_local',
                                    '--qos-history', 'keep_all',
                                    '--qos-reliability', 'reliable'])
    odom_relay = Node(package='topic_tools', executable='relay', name='odometry_adapter',
                      arguments=['/base_controller/odometry', '/Odometry'])
    jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
    base = Node(package='controller_manager', executable='spawner', arguments=['base_controller', '--controller-manager', '/controller_manager'])
    arm = Node(package='controller_manager', executable='spawner', arguments=['arm_controller', '--controller-manager', '/controller_manager'])
    gripper = Node(package='controller_manager', executable='spawner', arguments=['gripper_effort_controller', '--controller-manager', '/controller_manager'])
    # Gazebo 不执行 <mimic>, 两根手指必须各自驱动 (见 controllers.yaml)。
    gripper_fingers = Node(package='controller_manager', executable='spawner', arguments=['gripper_finger_controller', '--controller-manager', '/controller_manager'])
    gripper_adapter = Node(package='scoutfetch_sim', executable='gripper_adapter.py',
                           name='sim_gripper_adapter', output='screen', parameters=[{'use_sim_time': True}])
    after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb, base, arm, gripper, gripper_fingers]))
    rviz = Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', PathJoinSubstitution([desc_share, 'rviz', 'model.rviz'])], parameters=[{'use_sim_time': True}])
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([sim_share, 'worlds', 'pick_place.sdf'])),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'), DeclareLaunchArgument('rviz', default_value='true'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.dirname(get_package_share_directory('scoutfetch_description'))),
        gazebo, gazebo_gui, rsp, spawn, bridge, scan_to_cloud,
        scan_filter, relay, odom_tf_relay, odom_relay, gripper_adapter, after_spawn, rviz])
