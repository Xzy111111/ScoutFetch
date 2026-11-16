import os
import math
import subprocess
import xml.etree.ElementTree as ET

import yaml


ROOT = os.path.dirname(os.path.dirname(__file__))
XACRO = os.path.join(ROOT, 'urdf', 'scoutfetch.urdf.xacro')


def expanded(planning_model=False):
    return subprocess.check_output([
        'xacro', XACRO, 'sim_mode:=true',
        f'planning_model:={str(planning_model).lower()}',
    ], text=True)


def test_unique_links_and_joints():
    root = ET.fromstring(expanded())
    links = [x.attrib['name'] for x in root.findall('link')]
    joints = [x.attrib['name'] for x in root.findall('joint')]
    assert len(links) == len(set(links))
    assert len(joints) == len(set(joints))
    for required in ('BODY', 'base_link', 'mid_Link', 'link6',
                     'gripper_tcp', 'livox_frame', 'camera_color_optical_frame'):
        assert required in links


def test_control_contract():
    root = ET.fromstring(expanded())
    controlled = {j.attrib['name'] for rc in root.findall('ros2_control') for j in rc.findall('joint')}
    assert {'F_F_L', 'F_F_R', 'F_B_L', 'F_B_R'} <= controlled
    assert {f'joint{i}' for i in range(1, 7)} <= controlled
    assert 'gripper' in controlled


def test_sim_wheel_axes_roll_forward_in_the_same_body_direction():
    root = ET.fromstring(expanded())
    for name in ('F_F_L', 'F_F_R', 'F_B_L', 'F_B_R'):
        joint = root.find(f"joint[@name='{name}']")
        yaw = float(joint.find('origin').attrib['rpy'].split()[2])
        x, y, z = map(float, joint.find('axis').attrib['xyz'].split())
        body_axis = (math.cos(yaw)*x-math.sin(yaw)*y,
                     math.sin(yaw)*x+math.cos(yaw)*y, z)
        assert abs(body_axis[0]) < 1e-5, name
        assert abs(body_axis[1] - 1.0) < 1e-5, name
        assert body_axis[2] == 0.0, name


def test_tree_has_one_parent_per_child():
    root = ET.fromstring(expanded())
    children = [j.find('child').attrib['link'] for j in root.findall('joint')]
    assert len(children) == len(set(children))


def test_public_root_and_chassis_mount_match_real_contract():
    root = ET.fromstring(expanded())
    child_links = {j.find('child').attrib['link'] for j in root.findall('joint')}
    assert 'base_link' not in child_links

    joint = root.find("joint[@name='body_to_base_link']")
    assert joint.find('parent').attrib['link'] == 'base_link'
    assert joint.find('child').attrib['link'] == 'BODY'
    assert [float(v) for v in joint.find('origin').attrib['xyz'].split()] == [0.0, 0.0, 0.004]


def test_active_camera_mount_matches_documented_parameters():
    root = ET.fromstring(expanded())
    joint = root.find("joint[@name='camera_mount_joint']")
    assert joint is not None
    assert joint.find('parent').attrib['link'] == 'link6'
    assert joint.find('child').attrib['link'] == 'camera_link'

    origin = joint.find('origin')
    actual_xyz = [float(value) for value in origin.attrib['xyz'].split()]
    actual_rpy = [float(value) for value in origin.attrib['rpy'].split()]
    parameters_path = os.path.join(ROOT, 'config', 'model_parameters.yaml')
    with open(parameters_path, encoding='utf-8') as stream:
        mount = yaml.safe_load(stream)['model']['camera_mount_link6_to_camera_link']

    assert mount['mode'] == 'convenient_sim_pose'
    assert actual_xyz == mount['xyz']
    assert actual_rpy == mount['rpy']


def test_gazebo_sensor_frames_match_urdf_frames():
    root = ET.fromstring(expanded())
    gazebo_by_reference = {
        element.attrib.get('reference'): element for element in root.findall('gazebo')
    }
    lidar = gazebo_by_reference['livox_frame'].find("sensor[@name='mid360_scan']")
    imu = gazebo_by_reference['imu_frame'].find("sensor[@name='mid360_imu']")
    camera = gazebo_by_reference['camera_link'].find("sensor[@name='d435i_rgbd']")
    assert lidar.findtext('gz_frame_id') == 'livox_frame'
    assert lidar.findtext('topic') == '/scan_raw'
    assert imu.findtext('gz_frame_id') == 'imu_frame'
    assert camera.findtext('gz_frame_id') == 'camera_color_optical_frame'


def test_planning_view_uses_simplified_chassis_collision():
    root = ET.fromstring(expanded(planning_model=True))
    body_collision = root.find("link[@name='BODY']/collision")
    assert body_collision.find('geometry/box') is not None
    for wheel in ('F_F_L_Link', 'F_F_R_Link', 'F_B_L_Link', 'F_B_R_Link', 'mid_Link'):
        assert root.find(f"link[@name='{wheel}']/collision") is None
    # Arm collision geometry remains present for real planning safety.
    assert root.find("link[@name='link1']/collision") is not None
