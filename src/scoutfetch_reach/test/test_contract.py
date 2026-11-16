#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Contract test for scoutfetch_reach.

Asserts the public interface the task pipeline relies on is preserved:

    publishes /best_base_pose        (map frame, geometry_msgs/PoseStamped)
    publishes /base_pose_candidates  (visualization)
    subscribes /target_pose_map      (scoutfetch_vision output, map frame)
    subscribes global_costmap/costmap

and that the shared grasp model (planning_frame=base_link, eef=gripper_tcp,
group=arm) stays aligned with scoutfetch_grasp / scoutfetch_moveit_config.
"""

import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src", "scoutfetch_reach_node.cpp")
CFG = os.path.join(ROOT, "config", "scoutfetch_reach.yaml")


def _read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def test_public_pose_topics():
    content = _read(SRC)
    assert '"/best_base_pose"' in content
    assert '"/base_pose_candidates"' in content
    assert '"/target_pose_map"' in content


def test_costmap_and_grasp_model_params():
    cfg = _read(CFG)
    assert 'costmap_topic: "global_costmap/costmap"' in cfg
    assert 'planning_frame: "base_link"' in cfg
    assert 'eef_link: "gripper_tcp"' in cfg
    assert 'planning_group: "arm"' in cfg


def test_footprint_matches_base():
    cfg = _read(CFG)
    assert "footprint_x: 0.275" in cfg
    assert "footprint_y: 0.20" in cfg


def test_target_subscription_keeps_only_the_freshest_target():
    # KeepLast(1) on /target_pose_map: a single candidate evaluation takes ~2.4 s
    # while the stream runs at 10 Hz, so the node must always work on the newest
    # target rather than build a backlog. task_fsm relies on this when it accepts
    # a best pose stamped at-or-after its task target instead of requiring
    # exact equality.
    content = _read(SRC)
    assert "rclcpp::KeepLast(1)" in content


def test_launch_uses_shared_moveit_config_and_joint_states_arg():
    launch = _read(os.path.join(ROOT, "launch", "scoutfetch_reach.launch.py"))
    assert 'MoveItConfigsBuilder("car_with_piper", package_name="scoutfetch_moveit_config")' in launch
    assert '"joint_states_topic"' in launch
    # ompl-only pipelines: avoids MoveItConfigsBuilder falling back to the default
    # pilz pipeline, whose pilz_cartesian_limits.yaml is not shipped in this config.
    assert '.planning_pipelines(pipelines=["ompl"])' in launch
    # use_sim_time must be settable: the MoveGroup current-state monitor rejects
    # joint_states stamped on a clock other than the node's own.
    assert '"use_sim_time"' in launch