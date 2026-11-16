import os
import py_compile

import yaml


ROOT = os.path.dirname(os.path.dirname(__file__))


def load_config(name):
    with open(os.path.join(ROOT, 'config', name), encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_navigation_frame_and_sensor_contract():
    nav = load_config('nav2_params.yaml')
    assert nav['amcl']['ros__parameters']['base_frame_id'] == 'base_link'
    assert nav['amcl']['ros__parameters']['odom_frame_id'] == 'odom'
    assert nav['amcl']['ros__parameters']['scan_topic'] == '/livox/scan'
    assert nav['bt_navigator']['ros__parameters']['robot_base_frame'] == 'base_link'
    assert nav['bt_navigator']['ros__parameters']['odom_topic'] == '/Odometry'
    assert nav['local_costmap']['local_costmap']['ros__parameters']['robot_base_frame'] == 'base_link'
    assert nav['global_costmap']['global_costmap']['ros__parameters']['robot_base_frame'] == 'base_link'
    assert nav['velocity_smoother']['ros__parameters']['odom_topic'] == '/Odometry'


def test_laser_points_survive_source_height_filter_and_feed_both_costmaps():
    nav = load_config('nav2_params.yaml')
    for name in ('local_costmap', 'global_costmap'):
        costmap = nav[name][name]['ros__parameters']
        scan = costmap['obstacle_layer']['scan']
        assert scan['min_obstacle_height'] <= .014499 <= scan['max_obstacle_height']
        assert scan['marking'] and scan['clearing']
        assert scan['expected_update_rate'] <= .5
        assert 'static_layer' in costmap['plugins']


def test_collision_monitor_guards_mux_output_in_all_directions():
    p = load_config('collision_monitor.yaml')['collision_monitor']['ros__parameters']
    assert p['cmd_vel_in_topic'] == '/cmd_vel_unfiltered'
    assert p['cmd_vel_out_topic'] == '/cmd_vel'
    assert p['source_timeout'] < 1.
    approach = p['FootprintApproach']
    assert approach['action_type'] == 'approach'
    assert approach['footprint_topic'] == '/local_costmap/published_footprint'
    assert approach['time_before_collision'] >= 1.5


def test_navigation_footprint_matches_real_planner_contract():
    nav = load_config('nav2_params.yaml')
    expected = '[[0.275, 0.20], [0.275, -0.20], [-0.275, -0.20], [-0.275, 0.20]]'
    local = nav['local_costmap']['local_costmap']['ros__parameters']
    global_ = nav['global_costmap']['global_costmap']['ros__parameters']
    assert local['footprint'] == expected
    assert global_['footprint'] == expected
    assert local['footprint_padding'] == 0.03
    assert global_['footprint_padding'] == 0.01


def test_slam_owns_map_to_odom_only_in_mapping_mode():
    slam = load_config('slam_toolbox.yaml')['slam_toolbox']['ros__parameters']
    assert slam['mode'] == 'mapping'
    assert slam['map_frame'] == 'map'
    assert slam['odom_frame'] == 'odom'
    assert slam['base_frame'] == 'base_link'
    assert slam['scan_topic'] == '/livox/scan'
    assert slam['transform_publish_period'] > 0.0


def test_amcl_initializes_itself_for_map_based_runs():
    # The task pipeline needs a map frame that does not move between plan,
    # execution and resolve (AD-012). Running against a saved map with AMCL
    # requires an initial pose; the map is built from the robot's origin pose,
    # so (0,0,0) is correct and a headless run needs no RViz 2D Pose Estimate.
    amcl = load_config('nav2_params.yaml')['amcl']['ros__parameters']
    assert amcl['set_initial_pose'] is True
    assert amcl['initial_pose']['x'] == 0.0
    assert amcl['initial_pose']['y'] == 0.0
    assert amcl['initial_pose']['yaw'] == 0.0


def test_saved_map_exists():
    # The grid map is an acceptance artifact, not regenerated build output:
    # sim_navigation consumes it by absolute path.
    maps = os.path.join(ROOT, 'maps')
    with open(os.path.join(maps, 'laboratory.yaml'), encoding='utf-8') as stream:
        meta = yaml.safe_load(stream)
    assert meta['mode'] == 'trinary'
    assert meta['resolution'] > 0.0
    # the image is referenced relatively and must live beside the yaml
    assert os.path.isfile(os.path.join(maps, meta['image']))


def test_real_mid360_cloud_conversion_contract():
    conversion = load_config('pointcloud_to_laserscan.yaml')
    conversion = conversion['pointcloud_to_laserscan']['ros__parameters']
    slam = load_config('slam_toolbox_real.yaml')['slam_toolbox']['ros__parameters']
    assert conversion['target_frame'] == slam['base_frame'] == 'base_link'
    assert conversion['min_height'] < conversion['max_height']
    assert conversion['range_min'] == slam['min_laser_range']
    assert conversion['range_max'] == slam['max_laser_range']
    assert slam['scan_topic'] == '/livox/scan'
    with open(os.path.join(ROOT, 'launch', 'real_mapping.launch.py'), encoding='utf-8') as stream:
        source = stream.read()
    assert "default_value='/livox/lidar'" in source
    assert "('scan', '/livox/scan')" in source


def test_holonomic_controller_and_velocity_limits():
    params = load_config('nav2_params.yaml')
    follow = params['controller_server']['ros__parameters']['FollowPath']
    smooth = params['velocity_smoother']['ros__parameters']
    assert follow['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert follow['motion_model'] == 'Omni'
    assert follow['vy_max'] > 0.0
    assert smooth['max_velocity'][1] > 0.0
    assert smooth['min_velocity'][1] < 0.0


def test_bt_plugins_match_humble_library_names():
    plugins = load_config('nav2_params.yaml')['bt_navigator']['ros__parameters']['plugin_lib_names']
    assert 'nav2_drive_on_heading_bt_node' in plugins
    assert 'nav2_drive_on_heading_action_bt_node' not in plugins


def test_mux_prioritizes_teleop():
    topics = load_config('twist_mux.yaml')['twist_mux']['ros__parameters']['topics']
    assert topics['teleop']['topic'] != topics['navigation']['topic']
    assert topics['teleop']['priority'] > topics['navigation']['priority']


def test_launch_files_are_valid_python():
    launch_dir = os.path.join(ROOT, 'launch')
    for name in os.listdir(launch_dir):
        if name.endswith('.launch.py'):
            py_compile.compile(os.path.join(launch_dir, name), doraise=True)


def test_sim_launches_isolate_child_rviz_arguments():
    for name in ('sim_mapping.launch.py', 'sim_navigation.launch.py'):
        with open(os.path.join(ROOT, 'launch', name), encoding='utf-8') as stream:
            source = stream.read()
        assert source.count('GroupAction(scoped=True, actions=') == 2
