import os
import ast

import yaml


ROOT = os.path.dirname(os.path.dirname(__file__))
WS_SRC = os.path.dirname(ROOT)


def test_scoutfetch_interfaces_gripper_status_contract():
    msg_path = os.path.join(WS_SRC, 'scoutfetch_interfaces', 'msg', 'GripperStatus.msg')
    with open(msg_path, encoding='utf-8') as stream:
        content = stream.read()
    # Width is the primary state the upper layer gates grasp on; force is present
    # for API compatibility even though the position-controlled sim reports 0.0.
    assert 'float64 width' in content
    assert 'float64 force' in content
    for field in ('voltage_too_low', 'motor_overheating', 'driver_overcurrent',
                  'driver_overheating', 'sensor_status', 'driver_error_status',
                  'driver_enable_status', 'homing_status'):
        assert f'bool {field}' in content


def test_gripper_adapter_exposes_hardware_topics_and_units():
    adapter_path = os.path.join(ROOT, 'scripts', 'gripper_adapter.py')
    with open(adapter_path, encoding='utf-8') as stream:
        source = stream.read()
    ast.parse(source)
    assert "'control/gripper_cmd'" in source
    assert "'feedback/gripper_status'" in source
    assert "'/gripper_effort_controller/gripper_cmd'" in source
    assert "= 'gripper'" in source
    # Hardware command semantics: width in m over [0, 0.1], force in N over [0.5, 3.0].
    assert 'WIDTH_MAX = 0.1' in source
    assert 'FORCE_MIN = 0.5' in source
    assert 'FORCE_MAX = 3.0' in source


def test_sim_gripper_joint_range_matches_hardware_width():
    xacro_path = os.path.join(WS_SRC, 'scoutfetch_description', 'urdf', 'scoutfetch.urdf.xacro')
    with open(xacro_path, encoding='utf-8') as stream:
        source = stream.read()
    # The simulated prismatic gripper joint must span the same [0, 0.1] m range
    # as the hardware width so the adapter maps width 1:1 without rescaling.
    assert '<limit lower="0" upper="0.1"' in source


def test_sim_launch_starts_gripper_adapter():
    launch_path = os.path.join(ROOT, 'launch', 'sim.launch.py')
    with open(launch_path, encoding='utf-8') as stream:
        source = stream.read()
    ast.parse(source)
    assert "executable='gripper_adapter.py'" in source
    assert "name='sim_gripper_adapter'" in source


def test_gripper_controller_is_effort_controlled():
    with open(os.path.join(ROOT, 'config', 'controllers.yaml'), encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    cm = data['controller_manager']['ros__parameters']
    assert cm['gripper_effort_controller']['type'] == 'effort_controllers/GripperActionController'
    gripper = data['gripper_effort_controller']['ros__parameters']
    assert gripper['joint'] == 'gripper'