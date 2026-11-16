import os
import ast
import yaml


ROOT = os.path.dirname(os.path.dirname(__file__))


def test_sim_lidar_exposes_real_machine_pointcloud_contract():
    with open(os.path.join(ROOT, 'config', 'bridge.yaml'), encoding='utf-8') as stream:
        bridge = yaml.safe_load(stream)
    scan_bridge = next(item for item in bridge if item['ros_topic_name'] == '/scan_raw')
    assert scan_bridge['ros_type_name'] == 'sensor_msgs/msg/LaserScan'

    with open(os.path.join(ROOT, 'launch', 'sim.launch.py'), encoding='utf-8') as stream:
        launch_source = stream.read()
    assert "('cloud', '/livox/lidar')" in launch_source
    assert "('scan_in', '/livox/scan')" in launch_source
    assert "executable='scan_self_filter.py'" in launch_source


def test_controller_names_and_interfaces():
    with open(os.path.join(ROOT, 'config', 'controllers.yaml'), encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    cm = data['controller_manager']['ros__parameters']
    assert {'base_controller', 'arm_controller', 'gripper_effort_controller'} <= set(cm)
    assert data['arm_controller']['ros__parameters']['joints'] == [f'joint{i}' for i in range(1, 7)]
    assert data['gripper_effort_controller']['ros__parameters']['joint'] == 'gripper'
    assert data['base_controller']['ros__parameters']['base_frame_id'] == 'base_link'


def test_bridge_has_required_topics():
    with open(os.path.join(ROOT, 'config', 'bridge.yaml'), encoding='utf-8') as stream:
        topics = {item['ros_topic_name'] for item in yaml.safe_load(stream)}
    assert {'/clock', '/scan_raw', '/livox/imu',
            '/camera/camera/color/image_raw',
            '/camera/camera/color/camera_info',
            '/camera/camera/aligned_depth_to_color/image_raw',
            '/camera/camera/aligned_depth_to_color/camera_info',
            '/camera/camera/depth/color/points'} <= topics


def test_sim_launch_adapts_controller_private_topics():
    launch_path = os.path.join(ROOT, 'launch', 'sim.launch.py')
    with open(launch_path, encoding='utf-8') as stream:
        source = stream.read()
    ast.parse(source)
    assert "arguments=['/cmd_vel', '/base_controller/reference_unstamped']" in source
    tree = ast.parse(source)
    relay = next(node for node in ast.walk(tree)
                 if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                 and node.func.id == 'Node'
                 and any(k.arg == 'name' and isinstance(k.value, ast.Constant)
                         and k.value.value == 'odom_tf_adapter' for k in node.keywords))
    args = ast.literal_eval(next(k.value for k in relay.keywords if k.arg == 'arguments'))
    assert args[:2] == ['/base_controller/tf_odometry', '/tf']
    assert dict(zip(args[2::2], args[3::2])) == {
        '--qos-durability': 'transient_local',
        '--qos-history': 'keep_all',
        '--qos-reliability': 'reliable',
    }
    assert "arguments=['/base_controller/odometry', '/Odometry']" in source
