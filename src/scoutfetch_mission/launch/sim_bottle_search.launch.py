"""Complete Gazebo-only random bottle search, physical pick, and return demo."""

import os
import random
import tempfile
import math
import yaml
import cv2

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from scoutfetch_mission.random_scene import generate_room_map, generate_scene
from scoutfetch_mission.bookstore_scene import generate_bookstore, base_free
from scoutfetch_mission.simulation_guard import SimulationGuard


def _include(package, filename, **arguments):
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), "launch", filename)),
        launch_arguments=arguments.items())])


def _launch(context):
    seed = int(LaunchConfiguration("seed").perform(context))
    if seed < 0:
        seed = random.SystemRandom().randrange(2**31)
    directory = tempfile.mkdtemp(prefix="scoutfetch_bottle_search_")
    source = os.path.join(get_package_share_directory("scoutfetch_sim"), "worlds", "pick_place.sdf")
    world = os.path.join(directory, "random_bottle.sdf")
    room_map = os.path.join(directory, "room.yaml")
    scene = LaunchConfiguration("scene").perform(context)
    include_bottle = LaunchConfiguration("spawn_bottle").perform(context).lower() == "true"
    if scene == "bookstore":
        audit = generate_bookstore(
            os.path.join(get_package_share_directory("scoutfetch_sim"), "models", "aws_bookstore"),
            source, world, seed, station=int(LaunchConfiguration("station").perform(context)),
            include_bottle=include_bottle)
        room_map = os.path.join(directory, "bookstore.yaml")
        scene_info = f"Bookstore: station={audit['station']}, supported height=0.46 m"
    else:
        generate_scene(source, world, seed,
                       radius_min=float(LaunchConfiguration("radius_min").perform(context)),
                       radius_max=float(LaunchConfiguration("radius_max").perform(context)),
                       include_bottle=include_bottle)
        generate_room_map(source, room_map)
        scene_info = "Laboratory"
    execute = LaunchConfiguration("execute_pick").perform(context)
    spawn_x = float(LaunchConfiguration('spawn_x').perform(context))
    spawn_y = float(LaunchConfiguration('spawn_y').perform(context))
    spawn_yaw = float(LaunchConfiguration('spawn_yaw').perform(context))
    if not all(math.isfinite(v) for v in (spawn_x, spawn_y, spawn_yaw)):
        raise ValueError('initial robot pose must be finite')
    if scene == 'bookstore' and not base_free(cv2.imread(os.path.join(directory, 'bookstore_navigation.pgm'), 0),
                                            spawn_x, spawn_y, spawn_yaw):
        raise ValueError('initial robot pose collides with bookstore obstacles or unknown space')
    nav_source = os.path.join(get_package_share_directory('scoutfetch_navigation'), 'config', 'nav2_params.yaml')
    with open(nav_source) as stream:
        nav_config = yaml.safe_load(stream)
    nav_config['amcl']['ros__parameters']['initial_pose'].update(x=spawn_x, y=spawn_y, yaw=spawn_yaw)
    obstacle_map_nodes = []
    if scene == 'bookstore':
        for costmap in ('local_costmap', 'global_costmap'):
            nav_config[costmap][costmap]['ros__parameters']['static_layer']['map_topic'] = '/navigation_map'
        obstacle_map_nodes = [
            Node(package='nav2_map_server', executable='map_server', name='bookstore_obstacle_map_server',
                 output='screen', parameters=[{'use_sim_time': True, 'frame_id': 'map',
                    'topic_name': '/navigation_map',
                    'yaml_filename': os.path.join(directory, 'bookstore_navigation.yaml')}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_obstacle_map', output='screen',
                 parameters=[{'use_sim_time': True, 'autostart': True,
                              'node_names': ['bookstore_obstacle_map_server']}]),
        ]
    nav_params = os.path.join(directory, 'navigation.yaml')
    with open(nav_params, 'w') as stream:
        yaml.safe_dump(nav_config, stream)
    base_params = os.path.join(get_package_share_directory('scoutfetch_reach'), 'config', 'scoutfetch_reach.yaml')
    if scene == 'bookstore':
        with open(base_params) as stream:
            base_config = yaml.safe_load(stream)
        base_config['scoutfetch_reach_node']['ros__parameters']['angle_offsets_deg'] = [
            float(angle) for angle in range(-180, 181, 10)]
        base_params = os.path.join(directory, 'bookstore_base_pose.yaml')
        with open(base_params, 'w') as stream:
            yaml.safe_dump(base_config, stream)
    return [
        LogInfo(msg=f"{scene_info}; random bottle seed={seed}, files={directory}"),
        _include("scoutfetch_sim", "sim.launch.py", world=world,
                 gui=LaunchConfiguration("gui"), rviz="false",
                 spawn_x=str(spawn_x), spawn_y=str(spawn_y), spawn_yaw=str(spawn_yaw)),
        *obstacle_map_nodes,
        _include("scoutfetch_navigation", "navigation.launch.py", map=room_map,
                 use_sim_time="true", use_twist_mux="true", rviz=LaunchConfiguration("rviz"),
                 params_file=nav_params),
        TimerAction(period=5.0, actions=[
            _include("scoutfetch_moveit_config", "moveit.launch.py", backend="sim", use_rviz="false"),
            _include("scoutfetch_vision", "yolo_detector.launch.py", use_sim_time="true",
                     model_path=os.path.join(get_package_share_directory("scoutfetch_vision"),
                                             "models", "yolov8s.onnx")),
            _include("scoutfetch_vision", "localization.launch.py",
                     pixel_source="yolo", use_sim_time="true"),
            _include("scoutfetch_reach", "scoutfetch_reach.launch.py",
                     use_rviz="false", use_sim_time="true", params_file=base_params),
            _include("scoutfetch_grasp", "pick_node.launch.py", execute=execute,
                     allow_real_execution=execute, return_to_ready="false",
                     return_to_stow="true", use_rviz="false", use_sim_time="true"),
            _include("scoutfetch_mission", "scoutfetch_mission.launch.py", enable_navigation="true",
                     grasp_monitor_enabled=execute,
                     search_enabled="true", use_sim_time="true",
                     exploration_enabled=LaunchConfiguration('exploration_enabled'),
                     search_max_turns=LaunchConfiguration("search_max_turns"),
                     search_continuous_rotation=LaunchConfiguration('search_continuous_rotation'),
                     search_angular_speed=LaunchConfiguration('search_angular_speed'),
                     search_timeout_sec=LaunchConfiguration("search_timeout_sec")),
            Node(package="scoutfetch_mission", executable="start_sim_task", output="screen",
                 parameters=[{"use_sim_time": True}],
                 condition=IfCondition(LaunchConfiguration("auto_start"))),
            Node(package="scoutfetch_mission", executable="robot_control", output="screen",
                 parameters=[{"use_sim_time": True}],
                 condition=IfCondition(LaunchConfiguration("control_gui"))),
        ]),
        LogInfo(msg=("等待就绪后将自动开始任务。" if
                     LaunchConfiguration("auto_start").perform(context).lower() == "true" else
                     "仿真启动后等待 /start_task；可设置 auto_start:=true 自动开始。")),
    ]


def generate_launch_description():
    return LaunchDescription([
        SimulationGuard(),
        DeclareLaunchArgument("scene", default_value="laboratory", choices=["laboratory", "bookstore"]),
        DeclareLaunchArgument('exploration_enabled', default_value='true', choices=['true', 'false'],
                              description='After one empty scan, navigate to another costmap-derived viewpoint'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Initial world/map yaw in radians'),
        DeclareLaunchArgument("station", default_value="-1", description="Bookstore: -1 random; 0..5 supported pickup tables"),
        DeclareLaunchArgument("seed", default_value="-1", description="-1: random each launch; >=0: reproducible layout"),
        DeclareLaunchArgument("radius_min", default_value="0.95", description="Minimum bottle distance from start, metres"),
        DeclareLaunchArgument("radius_max", default_value="1.40", description="Maximum bottle distance, up to 2.40 m; wall clearance is checked"),
        DeclareLaunchArgument("spawn_bottle", default_value="true", choices=["true", "false"],
                              description="false: start without a bottle to verify continued search"),
        DeclareLaunchArgument("search_max_turns", default_value="0", description="0: continue scanning until a target is confirmed"),
        DeclareLaunchArgument('search_continuous_rotation', default_value='true', choices=['true', 'false'],
                              description='Rotate continuously until a target candidate needs confirmation'),
        DeclareLaunchArgument('search_angular_speed', default_value='0.25', description='Search angular speed, rad/s'),
        DeclareLaunchArgument("search_timeout_sec", default_value="0.0", description="0: continuous search, with odometry/camera watchdogs"),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("control_gui", default_value=LaunchConfiguration("gui"),
                              choices=["true", "false"], description="Keyboard/manual-auto control window"),
        DeclareLaunchArgument("auto_start", default_value="false", choices=["true", "false"],
                              description="true: wait for controllers/Nav2/vision/AMCL, then arm once"),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("execute_pick", default_value="true", choices=["true", "false"],
                              description="Gazebo arm/gripper execution; false only validates plans"),
        OpaqueFunction(function=_launch),
    ])
