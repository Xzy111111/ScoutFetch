from concurrent.futures import Future

from scoutfetch_interfaces.msg import GripperStatus
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
import numpy as np

from test_search_pipeline import fsm, arm, pose
from scoutfetch_mission.grasp_monitor import GraspMonitor
from scoutfetch_mission.task_fsm import TaskState


def feedback(h, width=.064):
    msg = GripperStatus(width=width, driver_enable_status=True)
    msg.header.stamp = h.get_clock().now().to_msg()
    h._gripper_callback(msg)


def held(h):
    feedback(h)
    h.advance(h.now+.5)
    feedback(h)


def test_cached_width_does_not_prove_hold():
    m = GraspMonitor()
    m.update(.064, True, 100, 0)
    m.update(.064, True, 100, 1.2)
    assert m.state(1.2) == 'FEEDBACK_STALE'
    m.update(float('nan'), True, 101, 1.3)
    assert m.state(1.6) == 'LOST'


def test_pick_success_waits_for_fresh_hold_then_return_and_verifies_arrival(fsm):
    h = fsm
    h.grasp_monitor_enabled = True
    arm(h)
    h.transition_to(TaskState.WAIT_PICK_RESULT)
    h._call_execute_pick()
    h.pick_future.set_result(Trigger.Response(success=True))
    assert not h._carrying_bottle
    assert h.current_task_state == TaskState.VERIFY_CARRY
    h.navigation_goal_publisher.publish.assert_not_called()
    held(h)
    h._search_tick()
    assert h._carrying_bottle and h.current_task_state == TaskState.WAIT_RETURN_NAV_RESULT
    h.navigation_result_callback(Bool(data=True))
    assert h.current_task_state == TaskState.VERIFY_CARRY
    h.advance(h.now+.5)
    feedback(h)
    h._search_tick()
    assert h.current_task_state == TaskState.DONE


def test_drop_cancels_return_before_preparing_and_searches_preserving_home(fsm):
    h = fsm
    h.grasp_monitor_enabled = True
    arm(h)
    home = h.saved_start_pose
    held(h)
    h._carrying_bottle = True
    h.transition_to(TaskState.WAIT_RETURN_NAV_RESULT)
    h._publish_return_goal()
    h.navigation_goal_publisher.publish.reset_mock()
    recovery = Future()
    h.recovery_client.call_async.return_value = recovery
    h.advance(h.now+.01)
    feedback(h, 0.)
    h.advance(h.now+.3)
    feedback(h, 0.)
    h._search_tick()
    assert not h._carrying_bottle and h.current_task_state == TaskState.RECOVERY_STOPPING
    h.navigation_cancel_publisher.publish.assert_called()
    h.recovery_client.call_async.assert_not_called()
    # Success racing with cancellation still must not become DONE.
    h.navigation_result_callback(Bool(data=True))
    assert h.current_task_state == TaskState.RECOVERY_PREPARING
    recovery.set_result(Trigger.Response(success=True))
    assert h.current_task_state == TaskState.SEARCHING
    assert h.saved_start_pose is home and h.active_target_pose is None
    assert h._recovery_attempts == 1
    h.navigation_goal_publisher.publish.assert_not_called()


def test_feedback_loss_stops_without_opening_unknown_held_object(fsm):
    h = fsm
    h.grasp_monitor_enabled = True
    arm(h)
    held(h)
    h._carrying_bottle = True
    h.transition_to(TaskState.WAIT_RETURN_NAV_RESULT)
    h._publish_return_goal()
    h.advance(h.now+1.1)
    h._search_tick()
    h.navigation_result_callback(Bool(data=False))
    assert h.current_task_state == TaskState.FAILED and h.control_mode == 'MANUAL'
    h.recovery_client.call_async.assert_not_called()


def test_manual_during_recovery_waits_for_arm_response(fsm):
    h = fsm
    h.grasp_monitor_enabled = True
    arm(h)
    recovery = Future()
    h.recovery_client.call_async.return_value = recovery
    h._begin_recovery('empty grasp')
    assert h.current_task_state == TaskState.RECOVERY_PREPARING
    assert h.set_control_mode_callback(SetBool.Request(data=False), SetBool.Response()).success
    assert h.control_mode == 'SWITCHING_TO_MANUAL'
    recovery.set_result(Trigger.Response(success=True))
    assert h.control_mode == 'MANUAL' and h.current_task_state == TaskState.IDLE


def test_regrasp_limit_prevents_endless_empty_grabs(fsm):
    h = fsm
    h.grasp_monitor_enabled = True
    arm(h)
    h._recovery_attempts = 2
    h._begin_recovery('empty again')
    assert h.current_task_state == TaskState.FAILED and h.control_mode == 'MANUAL'
    h.recovery_client.call_async.assert_not_called()


def test_new_obstacle_at_grasp_station_cancels_and_requests_fresh_station(fsm):
    h = fsm
    arm(h)
    h.exploration_enabled = True
    grid = np.zeros((100, 100), dtype=np.int16)
    h.explorer.update(grid, .05, (-2., -2., 0.))
    goal = pose(h.now, x=.8)
    h.transition_to(TaskState.WAIT_BASE_POSE)
    h.active_target_stamp = (10, 0)
    h.best_base_pose_callback(goal)
    assert h.current_task_state == TaskState.WAIT_NAV_RESULT
    grid[40, 56] = 100
    h.explorer.update(grid, .05, (-2., -2., 0.))
    h.advance(h.now+.1)
    h._search_tick()
    assert h.current_task_state == TaskState.NAV_REPLANNING
    h.navigation_cancel_publisher.publish.assert_called()
    h.navigation_result_callback(Bool(data=True))
    assert h.current_task_state == TaskState.SEARCHING
    assert h.active_target_pose is None
    h.navigation_goal_publisher.publish.reset_mock()
    h.best_base_pose_callback(goal)  # pre-cancel answer must not be reused
    h.navigation_goal_publisher.publish.assert_not_called()
    h.transition_to(TaskState.WAIT_BASE_POSE)
    h.active_target_stamp = (10, 100000000)
    h.best_base_pose_callback(pose(h.now, x=-.5))
    assert h.current_task_state == TaskState.WAIT_NAV_RESULT


def test_distant_bottle_is_observed_closer_before_grasp_planning(fsm):
    h = fsm
    arm(h)
    h.exploration_enabled = True
    h.explorer.update(np.zeros((160, 160), dtype=np.int16), .1, (-8., -8., 0.))
    home = h.saved_start_pose
    for t in (10.1, 10.95, 11.05, 11.15):
        h.advance(t)
        h.target_pose_map_callback(pose(h.now, x=3.5))
    assert h.current_task_state == TaskState.EXPLORING
    assert h._skip_exploration_candidates
    assert h.active_target_pose is None and h.saved_start_pose is home
    h.target_pose_map_callback(pose(h.now, x=3.5))
    assert h.current_task_state == TaskState.EXPLORING
    h.navigation_result_callback(Bool(data=True))
    assert h.current_task_state == TaskState.SEARCHING
    assert not h._skip_exploration_candidates and h.saved_start_pose is home


def test_floor_coordinate_change_after_arrival_replans_before_resolve(fsm):
    h = fsm
    h.grasp_monitor_enabled = True
    arm(h)
    home = h.saved_start_pose
    h.active_target_pose = pose(h.now)
    h.active_target_pose.pose.position.z = -.04
    h.active_target_stamp = (10, 0)
    h.transition_to(TaskState.WAIT_NAV_RESULT)
    h._begin_settle_then_resolve()
    h._settle_tick()
    h.advance(10.1)
    h._settle_tick()
    h.advance(10.8)
    latest = pose(h.now, x=1.17)
    latest.pose.position.z = -.04
    h.latest_target_pose = latest
    h._settle_tick()
    assert h.current_task_state == TaskState.WAIT_BASE_POSE
    assert h.active_target_pose is latest
    assert h.saved_start_pose is home and h._target_refinements == 1
    h.resolve_trigger_publisher.publish.assert_not_called()
