import ast
import os
import subprocess
import xml.etree.ElementTree as ET

import yaml


ROOT = os.path.dirname(os.path.dirname(__file__))


def test_semantic_arm_contract():
    path = os.path.join(ROOT, 'config', 'scoutfetch.srdf.xacro')
    xml = subprocess.check_output(['xacro', path], text=True)
    root = ET.fromstring(xml)
    group = root.find("group[@name='arm']")
    chain = group.find('chain')
    assert chain.attrib == {'base_link': 'base_link', 'tip_link': 'gripper_tcp'}
    for state_name in ('home', 'ready'):
        state = root.find(f"group_state[@name='{state_name}']")
        assert [j.attrib['name'] for j in state.findall('joint')] == [f'joint{i}' for i in range(1, 7)]


def test_moveit_controller_matches_sim_and_real_public_action():
    with open(os.path.join(ROOT, 'config', 'moveit_controllers.yaml'), encoding='utf-8') as stream:
        data = yaml.safe_load(stream)['moveit_simple_controller_manager']
    assert data['controller_names'] == ['arm_controller']
    controller = data['arm_controller']
    assert controller['type'] == 'FollowJointTrajectory'
    assert controller['action_ns'] == 'follow_joint_trajectory'
    assert controller['joints'] == [f'joint{i}' for i in range(1, 7)]


def test_launch_uses_canonical_description_and_does_not_own_backend():
    path = os.path.join(ROOT, 'launch', 'moveit.launch.py')
    with open(path, encoding='utf-8') as stream:
        source = stream.read()
    ast.parse(source)
    assert "get_package_share_directory('scoutfetch_description')" in source
    assert "'urdf', 'scoutfetch.urdf.xacro'" in source
    assert "package='robot_state_publisher'" not in source
    assert "executable='ros2_control_node'" not in source
    assert "executable='spawner'" not in source
    assert "'/joint_states' if backend == 'sim' else '/feedback/joint_states'" in source

