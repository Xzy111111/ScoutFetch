#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Contract test for scoutfetch_grasp (pick_node).

Asserts the public interface the task pipeline relies on is preserved:

    service    /execute_pick       (std_srvs/Trigger + safety gates)
    subscribes /grasp_target_pose  (target_pose_resolver output, base_link frame)
    publishes  control/gripper_cmd  (sensor_msgs/JointState: name=gripper, width/force)
    subscribes feedback/gripper_status (scoutfetch_interfaces/GripperStatus)

and that the shared grasp model stays aligned with scoutfetch_reach /
scoutfetch_moveit_config (planning_frame=base_link, eef=gripper_tcp, group=arm).
"""

import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src", "pick_node.cpp")
CFG = os.path.join(ROOT, "config", "pick_and_place.yaml")


def _read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def test_execute_pick_trigger_service():
    cfg = _read(CFG)
    assert 'execute_service: "/execute_pick"' in cfg


def test_grasp_target_pose_input():
    cfg = _read(CFG)
    assert 'target_topic: "/grasp_target_pose"' in cfg


def test_gripper_interface_topics_and_types():
    content = _read(SRC)
    # publish JointState commands to the backend adapter
    assert '"control/gripper_cmd"' in content
    # subscribe GripperStatus feedback from the backend adapter
    assert '"feedback/gripper_status"' in content
    assert "scoutfetch_interfaces/msg/gripper_status.hpp" in _read(
        os.path.join(ROOT, "include", "scoutfetch_grasp", "pick_node.hpp")
    )


def test_grasp_model_aligned_with_shared_config():
    cfg = _read(CFG)
    assert 'planning_group: "arm"' in cfg
    assert 'eef_link: "gripper_tcp"' in cfg
    assert 'planning_frame: "base_link"' in cfg


def test_station_planning_uses_the_same_tcp_calibration_as_execution():
    import yaml
    pick = yaml.safe_load(_read(CFG))['pick_node']['ros__parameters']
    base = yaml.safe_load(_read(os.path.join(ROOT, '..', 'scoutfetch_reach',
                                           'config', 'scoutfetch_reach.yaml')))
    base = base['scoutfetch_reach_node']['ros__parameters']
    assert base['grasp_offset'] == pick['grasp_offset']
    assert base['grasp_min_z'] == pick['grasp_min_z']
    for key in ('floor_pick_enabled', 'floor_threshold_z', 'floor_lift_distance', 'floor_grasp_z'):
        assert base[key] == pick[key]


def test_floor_grasp_height_lands_in_the_bottle_body_below_the_neck():
    import xml.etree.ElementTree as ET
    import yaml
    packages = os.path.dirname(ROOT)
    model = ET.parse(os.path.join(packages, 'scoutfetch_description', 'urdf', 'scoutfetch.urdf.xacro'))
    wheel_z = float(model.find("joint[@name='F_F_L']/origin").get('xyz').split()[2])
    body_z = float(model.find("joint[@name='body_to_base_link']/origin").get('xyz').split()[2])
    controls = yaml.safe_load(_read(os.path.join(packages, 'scoutfetch_sim', 'config', 'controllers.yaml')))
    wheel_radius = controls['base_controller']['ros__parameters']['kinematics']['wheels_radius']
    base_height = wheel_radius-wheel_z-body_z
    bottle = ET.parse(os.path.join(packages, 'scoutfetch_sim', 'worlds', 'pick_place.sdf'))
    body = bottle.find(".//model[@name='water_bottle']/link/collision[@name='body_collision']")
    body_z = float(body.findtext('pose').split()[2])
    half_length = float(body.findtext('geometry/cylinder/length'))/2
    pick = yaml.safe_load(_read(CFG))['pick_node']['ros__parameters']
    height = base_height+pick['floor_grasp_z']
    assert body_z-half_length+.02 <= height <= body_z+half_length-.02


def test_launch_safety_gates_and_shared_moveit_config():
    launch = _read(os.path.join(ROOT, "launch", "pick_node.launch.py"))
    assert '"allow_real_execution"' in launch
    assert 'MoveItConfigsBuilder("car_with_piper", package_name="scoutfetch_moveit_config")' in launch
    assert '"joint_states_topic"' in launch
    # ompl-only pipelines: avoids MoveItConfigsBuilder falling back to the default
    # pilz pipeline, whose pilz_cartesian_limits.yaml is not shipped in this config.
    assert '.planning_pipelines(pipelines=["ompl"])' in launch
    # use_sim_time must be settable: the MoveGroupInterface current-state monitor
    # rejects joint_states stamped on a clock other than the node's own.
    assert '"use_sim_time"' in launch
