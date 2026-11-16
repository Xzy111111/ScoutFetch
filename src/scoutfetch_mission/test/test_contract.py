#!/usr/bin/env python3
"""Contract test for scoutfetch_mission.

Asserts the orchestration interface connecting the whole pipeline is preserved
by the migration, matching the topic/service wiring in task_fsm /
target_pose_resolver / nav_goal_executor:

    service  /start_task            (task_fsm, std_srvs/Trigger entry point)
    subscribes /target_pose_map     (task_fsm in)
    subscribes /best_base_pose      (task_fsm in)
    publishes /navigation_goal      (task_fsm out: outbound = forwarded
                                      /best_base_pose, return = saved_start_pose)
    subscribes /navigation_goal     (nav_goal_executor in -- the only Nav2
                                      dispatcher, so every /navigation_result
                                      corresponds to the leg the FSM is awaiting)
    action    /navigate_to_pose     (nav_goal_executor -> Nav2 NavigateToPose)
    publishes /navigation_result    (nav_goal_executor out, std_msgs/Bool)
    publishes /resolve_grasp_target (task_fsm out, task event -> target_pose_resolver)
    publishes /grasp_target_pose    (target_pose_resolver out, base_link frame)
    client    /execute_pick         (task_fsm -> scoutfetch_grasp Trigger)
    publishes /task_state           (task_fsm out, std_msgs/String)
"""

import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "scoutfetch_mission")


def _read(*parts):
    with open(os.path.join(PKG, *parts), encoding="utf-8") as f:
        return f.read()


FSM = _read("task_fsm.py")
RESOLVER = _read("target_pose_resolver.py")
EXECUTOR = _read("nav_goal_executor.py")


def test_task_fsm_wiring():
    for topic in (
        "/target_pose_map",
        "/best_base_pose",
        "/navigation_goal",
        "/navigation_result",
        "/resolve_grasp_target",
        "/grasp_target_pose",
        "/execute_pick",
        "/start_task",
        "/task_state",
    ):
        assert topic in FSM, topic


def test_target_pose_resolver_wiring():
    assert 'trigger_topic", "/resolve_grasp_target"' in RESOLVER
    assert 'output_topic", "/grasp_target_pose"' in RESOLVER


def test_nav_goal_executor_wiring():
    # goal_topic is /navigation_goal, NOT /best_base_pose: the FSM must be the
    # only navigation trigger, otherwise the chassis moves before ARM and an
    # unrelated goal's result is misread as the return leg's result.
    assert 'goal_topic", "/navigation_goal"' in EXECUTOR
    assert 'goal_topic", "/best_base_pose"' not in EXECUTOR
    assert 'navigate_action", "/navigate_to_pose"' in EXECUTOR
    assert 'result_topic", "/navigation_result"' in EXECUTOR
    assert "NavigateToPose" in EXECUTOR  # Nav2 action type


def test_fsm_accepts_same_or_newer_best_stamp():
    # scoutfetch_reach evaluates one target per ~2.4 s while /target_pose_map
    # runs at 10 Hz, so its published best always lags the stream: requiring
    # exact stamp equality wedges the FSM in WAIT_BASE_POSE forever.
    assert "def _is_acceptable_best_stamp" in FSM
    assert "return best_stamp >= self.active_target_stamp" in FSM


def test_executor_clears_goal_active_before_publishing_result():
    # The FSM may publish the return goal the instant it sees a result; if
    # goal_active were still True that goal would be dropped as "already
    # active" and the FSM would never see the return result.
    tail = EXECUTOR.split("def result_callback", 1)[1]
    assert tail.index("self.goal_active = False") < tail.index(
        "self.publish_navigation_result")


def test_start_task_is_trigger_service():
    assert "Trigger, self.start_task_service" in FSM
    assert "execute_pick_client = self.create_client(" in FSM
    assert "Trigger, self.execute_pick_service" in FSM


def test_resolver_volatile_event_semantics():
    # /resolve_grasp_target is an event, not state: must not replay after restart
    assert "VOLATILE" in RESOLVER or "volatile_qos" in RESOLVER or "QoSDurabilityPolicy" in RESOLVER