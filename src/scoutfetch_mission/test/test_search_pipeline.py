"""Exercise actual FSM callbacks with controlled time and ROS message objects.

No DDS, Gazebo ground truth, or duplicated FSM implementation is used here.
"""

from concurrent.futures import Future
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from scoutfetch_mission.bottle_search import BottleSearch
from scoutfetch_mission.bottle_hint import BottleHint
from scoutfetch_mission.search_explorer import SearchExplorer
from scoutfetch_mission.grasp_monitor import GraspMonitor
from scoutfetch_mission.task_fsm import TaskFsm, TaskState


def pose(now, frame="map", x=1.1, y=0.0):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp = Time(seconds=now).to_msg()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, 0.38
    msg.pose.orientation.w = 1.0
    return msg


class Harness(TaskFsm):
    def __init__(self):
        # Transport endpoints are test doubles; every task callback is inherited.
        self.now = 10.0
        self.ros_rate = 1.0
        self.search_enabled = True
        self.center_target_before_confirm = False
        self.exploration_enabled = False
        self.explorer = SearchExplorer()
        self.observe_target_max_range = 2.5
        self._skip_exploration_candidates = False
        self.bottle_hint = BottleHint()
        self._camera_intrinsics = None
        self._exploration_map_received = self.now
        self._exploration_deadline = None
        self._exploration_error = None
        self._exploration_failures = 0
        self._last_exploration_goal_ns = 0
        self._view_stamp_ns = 0
        self.target_max_age = 0.5
        self.detector_timeout = 3.0
        self.phase_timeout = 60.0
        self._state_entered_monotonic = self.now
        self.search = BottleSearch()
        self.search.feedback(0.0, 0.0, 0.0, self.now)
        self._detector_received = self.now
        self._detector_stamp_ns = int(self.now * 1e9)
        self._armed_stamp_ns = None
        self._search_confirm_stamp_ns = None
        self.current_task_state = TaskState.IDLE
        self.control_mode = 'MANUAL'
        self._navigation_pending = False
        self._active_navigation_header = None
        self._carrying_bottle = False
        self.grasp_monitor_enabled = False
        self.grasp_monitor = GraspMonitor()
        self._recovery_attempts = 0
        self._recovery_limit = 2
        self._recovery_error = None
        self._verify_return = False
        self._target_refinements = 0
        self._nav_replans = 0
        self._best_after_ns = 0
        self._navigation_error = None
        self._navigation_started = None
        self._active_navigation_pose = None
        self._grasp_status = 'EMPTY'
        self.grasp_status_publisher = Mock()
        self.recovery_client = Mock()
        self._startup_auto_cancelled = False
        self.mode_publisher = Mock()
        self.autonomy_lock_publisher = Mock()
        self.manual_stop_publisher = Mock()
        self.navigation_cancel_publisher = Mock()
        self.active_target_pose = self.active_target_stamp = self.latest_target_pose = None
        self.saved_start_pose = None
        self.start_pose = pose(self.now, x=0.15, y=-0.2)
        self.start_pose.pose.orientation.z = math.sin(0.3)
        self.start_pose.pose.orientation.w = math.cos(0.3)
        self._settle_timer = self._settle_since = self._settle_prev_xy = None
        self.post_nav_settle_sec = 0.6
        self.settle_epsilon = 0.003
        self.search_velocity_publisher = Mock()
        self.navigation_goal_publisher = Mock()
        self.resolve_trigger_publisher = Mock()
        self.task_state_publisher = Mock()
        self.execute_pick_client = Mock()
        self.pick_future = Future()
        self.execute_pick_client.call_async.return_value = self.pick_future
        self.create_timer = Mock(return_value=Mock())
        self.destroy_timer = Mock()
        self.tf_buffer = Mock()
        self.tf_buffer.lookup_transform.return_value = TransformStamped()
        self.log = Mock()

    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(seconds=10.0 + (self.now - 10.0) * self.ros_rate))

    def get_logger(self):
        return self.log

    def _capture_start_pose(self):
        return self.start_pose

    def advance(self, now):
        self.now = now
        self.search.feedback(0.0, 0.0, 0.0, now)
        self._detector_received = now
        self._detector_stamp_ns = self.get_clock().now().nanoseconds


@pytest.fixture
def fsm(monkeypatch):
    h = Harness()
    monkeypatch.setattr("scoutfetch_mission.task_fsm.time.monotonic", lambda: h.now)
    return h


def arm(h):
    response = h.start_task_callback(Trigger.Request(), Trigger.Response())
    assert response.success
    assert h.current_task_state == TaskState.SEARCHING


def test_edge_detection_does_not_lock_target_and_out_of_order_center_frame_is_replayed(fsm):
    h = fsm
    h.center_target_before_confirm = True
    h._camera_width = 640
    h._centered_detection_frames = {}
    h._pending_search_target = None
    arm(h)
    h.advance(10.1)
    edge = Detection2DArray()
    edge.header.stamp = h.get_clock().now().to_msg()
    detection = Detection2D()
    detection.bbox.center.position.x = 36.
    edge.detections = [detection]
    h._detector_callback(edge)
    h.target_pose_map_callback(pose(h.now))
    assert h.search.mode == 'DWELL'
    h.advance(10.2)
    h.target_pose_map_callback(pose(h.now))  # localization arrives first
    assert h._pending_search_target is not None
    detection.bbox.center.position.x = 320.
    edge.header.stamp = h.get_clock().now().to_msg()
    h._detector_callback(edge)
    assert h.search.mode == 'CONFIRM'
    assert h._pending_search_target is None


def test_search_confirm_navigate_pick_and_return_preserves_original_pose(fsm):
    h = fsm
    h._search_tick()
    h.search_velocity_publisher.publish.assert_not_called()
    arm(h)
    for t in (10.1, 10.95, 11.05):
        h.advance(t)
        h.target_pose_map_callback(pose(t))
        assert h.current_task_state == TaskState.SEARCHING
        h.navigation_goal_publisher.publish.assert_not_called()
    h.advance(11.15)
    h.target_pose_map_callback(pose(h.now))
    assert h.current_task_state == TaskState.WAIT_BASE_POSE
    assert h.search_velocity_publisher.publish.call_args.args[0].angular.z == 0.0
    h.best_base_pose_callback(pose(11.0, x=0.6))  # pre-confirm planner result
    h.navigation_goal_publisher.publish.assert_not_called()
    best = pose(11.15, x=0.6)
    h.best_base_pose_callback(best)
    assert h.current_task_state == TaskState.WAIT_NAV_RESULT
    assert h.navigation_goal_publisher.publish.call_args.args[0] == best
    h.navigation_result_callback(Bool(data=True))
    h._settle_tick()
    h.advance(11.3)
    h._settle_tick()
    h.advance(12.0)
    h.target_pose_map_callback(pose(h.now))
    h._settle_tick()
    assert h.current_task_state == TaskState.WAIT_GRASP_TARGET
    assert h.resolve_trigger_publisher.publish.call_count == 1
    h.grasp_target_pose_callback(pose(h.now, frame="base_link", x=0.5))
    assert h.current_task_state == TaskState.WAIT_PICK_RESULT
    h.pick_future.set_result(Trigger.Response(success=True, message="held bottle"))
    assert h.current_task_state == TaskState.WAIT_RETURN_NAV_RESULT
    returned = h.navigation_goal_publisher.publish.call_args.args[0]
    assert returned.pose == h.start_pose.pose  # includes the pre-search yaw
    h.navigation_result_callback(Bool(data=True))
    assert h.current_task_state == TaskState.DONE
    assert h.navigation_goal_publisher.publish.call_count == 2


def test_stale_or_future_target_never_starts_navigation(fsm):
    arm(fsm)
    for timestamp in (9.9, 0.0, 12.0):
        fsm.target_pose_map_callback(pose(timestamp))
    assert fsm.current_task_state == TaskState.SEARCHING
    assert fsm.search.mode == "DWELL"
    fsm.navigation_goal_publisher.publish.assert_not_called()


def test_confirmation_works_when_simulation_clock_is_ten_times_slower(fsm):
    fsm.ros_rate = 0.1
    arm(fsm)
    for t in (10.1, 10.95, 11.05, 11.15):
        fsm.advance(t)
        stamp = fsm.get_clock().now().nanoseconds / 1e9
        fsm.target_pose_map_callback(pose(stamp))
    assert fsm.current_task_state == TaskState.WAIT_BASE_POSE


def test_detector_loss_stops_and_rearm_does_not_reuse_target(fsm):
    arm(fsm)
    fsm.advance(14.0)
    fsm._detector_received = 10.0
    fsm._search_tick()
    assert fsm.current_task_state == TaskState.FAILED
    assert fsm.search_velocity_publisher.publish.call_args.args[0].angular.z == 0.0
    fsm.latest_target_pose = pose(10.0)
    fsm.advance(14.1)
    arm(fsm)
    assert fsm.latest_target_pose is None


def test_start_rejects_missing_detector_or_odometry(fsm):
    fsm._detector_received = None
    response = fsm.start_task_callback(Trigger.Request(), Trigger.Response())
    assert not response.success and fsm.current_task_state == TaskState.IDLE
    fsm._detector_received = fsm.now
    fsm.search.last_odom = None
    response = fsm.start_task_callback(Trigger.Request(), Trigger.Response())
    assert not response.success and fsm.current_task_state == TaskState.IDLE
    fsm.search_velocity_publisher.publish.assert_not_called()


def test_no_feasible_base_pose_times_out_without_navigation(fsm):
    arm(fsm)
    fsm.transition_to(TaskState.WAIT_BASE_POSE)
    fsm.advance(71.0)
    fsm._search_tick()
    assert fsm.current_task_state == TaskState.FAILED
    fsm.navigation_goal_publisher.publish.assert_not_called()


def set_mode(h, automatic):
    return h.set_control_mode_callback(SetBool.Request(data=automatic), SetBool.Response())


def begin_navigation(h):
    arm(h)
    h.active_target_stamp = (10, 0)
    h.active_target_pose = pose(10.0)
    h.transition_to(TaskState.WAIT_BASE_POSE)
    h.best_base_pose_callback(pose(10.0, x=0.6))
    assert h._navigation_pending


def test_manual_stops_search_and_rejects_late_target(fsm):
    arm(fsm)
    assert set_mode(fsm, False).success
    assert fsm.control_mode == 'MANUAL' and fsm.current_task_state == TaskState.IDLE
    assert not fsm.search.active
    assert fsm.autonomy_lock_publisher.publish.call_args.args[0].data
    fsm.target_pose_map_callback(pose(10.1))
    fsm.navigation_goal_publisher.publish.assert_not_called()


@pytest.mark.parametrize('navigation_success', [False, True])
def test_manual_cancels_navigation_and_waits_for_terminal_result(fsm, navigation_success):
    begin_navigation(fsm)
    assert set_mode(fsm, False).success
    assert fsm.control_mode == 'SWITCHING_TO_MANUAL'
    assert fsm.current_task_state == TaskState.STOPPING
    cancel = fsm.navigation_cancel_publisher.publish.call_args.args[0]
    assert cancel.stamp == fsm.navigation_goal_publisher.publish.call_args.args[0].header.stamp
    assert not set_mode(fsm, True).success
    fsm.navigation_result_callback(Bool(data=navigation_success))
    assert fsm.control_mode == 'MANUAL' and not fsm._navigation_pending
    fsm.resolve_trigger_publisher.publish.assert_not_called()
    assert set_mode(fsm, True).success
    assert fsm.current_task_state == TaskState.SEARCHING


def test_manual_cancels_settle_timer_and_late_timer_cannot_resolve(fsm):
    begin_navigation(fsm)
    fsm.navigation_result_callback(Bool(data=True))
    assert fsm._settle_timer is not None
    assert set_mode(fsm, False).success
    assert fsm._settle_timer is None
    fsm._settle_tick()
    fsm.resolve_trigger_publisher.publish.assert_not_called()


def test_manual_during_pick_waits_for_stow_then_auto_resumes_return(fsm):
    arm(fsm)
    fsm.transition_to(TaskState.WAIT_PICK_RESULT)
    fsm._call_execute_pick()
    assert set_mode(fsm, False).success
    assert fsm.current_task_state == TaskState.WAIT_PICK_RESULT
    assert fsm.control_mode == 'SWITCHING_TO_MANUAL'
    # A late navigation event must not prematurely enable base driving.
    fsm.navigation_result_callback(Bool(data=True))
    assert fsm.control_mode == 'SWITCHING_TO_MANUAL'
    fsm.pick_future.set_result(Trigger.Response(success=True))
    assert fsm.control_mode == 'MANUAL' and fsm._carrying_bottle
    fsm.navigation_goal_publisher.publish.assert_not_called()
    fsm.advance(11.0)
    assert set_mode(fsm, True).success
    returned = fsm.navigation_goal_publisher.publish.call_args.args[0]
    assert returned.pose == fsm.saved_start_pose.pose
    assert fsm.current_task_state == TaskState.WAIT_RETURN_NAV_RESULT
    first_stamp = returned.header.stamp
    assert set_mode(fsm, False).success
    fsm.navigation_result_callback(Bool(data=False))
    fsm.advance(12.0)
    assert set_mode(fsm, True).success
    assert fsm.navigation_goal_publisher.publish.call_args.args[0].header.stamp != first_stamp


def test_manual_after_pick_service_exception_is_released(fsm):
    arm(fsm)
    fsm.transition_to(TaskState.WAIT_PICK_RESULT)
    fsm._call_execute_pick()
    set_mode(fsm, False)
    fsm.pick_future.set_exception(RuntimeError('pick disconnected'))
    assert fsm.control_mode == 'MANUAL'


def test_auto_after_manual_records_current_start_and_ignores_pre_arm_target(fsm):
    arm(fsm)
    set_mode(fsm, False)
    fsm.advance(20.0)
    fsm.start_pose = pose(20.0, x=0.8, y=-0.4)
    assert set_mode(fsm, True).success
    assert fsm.saved_start_pose.pose.position.x == 0.8
    fsm.target_pose_map_callback(pose(10.0))
    assert fsm.current_task_state == TaskState.SEARCHING


def test_operator_manual_choice_cancels_waiting_startup_auto(fsm):
    assert set_mode(fsm, False).success
    response = fsm.auto_start_callback(Trigger.Request(), Trigger.Response())
    assert not response.success and response.message == 'auto-start superseded by operator'
    assert fsm.current_task_state == TaskState.IDLE
    assert set_mode(fsm, True).success


def test_startup_auto_can_retry_readiness_but_cannot_start_a_second_task(fsm):
    fsm._detector_received = None
    assert not fsm.auto_start_callback(Trigger.Request(), Trigger.Response()).success
    fsm._detector_received = fsm.now
    assert fsm.auto_start_callback(Trigger.Request(), Trigger.Response()).success
    set_mode(fsm, False)
    assert not fsm.auto_start_callback(Trigger.Request(), Trigger.Response()).success
    assert fsm.current_task_state == TaskState.IDLE


def explorer_ready(h):
    import numpy as np
    h.exploration_enabled = True
    h.search.max_turns = 0
    grid = np.zeros((120, 120), dtype=np.int16)
    grid[[0, -1], :] = 100; grid[:, [0, -1]] = 100
    h.explorer.update(grid, .1, (-6, -6, 0))
    h._exploration_map_received = h.now


def complete_empty_scan(h):
    h.search.travel = 2*math.pi
    h.search.until = h.now-.1
    h.search.mode = 'DWELL'
    h._search_tick()


def test_empty_full_scan_moves_to_new_view_and_keeps_original_home(fsm):
    explorer_ready(fsm);arm(fsm)
    original = fsm.saved_start_pose
    complete_empty_scan(fsm)
    assert fsm.current_task_state == TaskState.EXPLORING and fsm._navigation_pending
    fsm.start_pose = pose(11.0, x=1.5, y=1.0)
    fsm.advance(11.0)
    fsm.navigation_result_callback(Bool(data=True))
    assert fsm.current_task_state == TaskState.SEARCHING
    assert fsm.saved_start_pose is original
    assert fsm.search.active


def test_exploration_detection_cancels_nav_before_accepting_stationary_target(fsm):
    explorer_ready(fsm);arm(fsm);complete_empty_scan(fsm)
    fsm.advance(10.1)
    fsm.target_pose_map_callback(pose(10.1))
    assert fsm.current_task_state == TaskState.EXPLORATION_STOPPING
    assert fsm.active_target_pose is None
    assert fsm.navigation_cancel_publisher.publish.called
    fsm.advance(10.2)
    fsm.navigation_result_callback(Bool(data=False))
    assert fsm.current_task_state == TaskState.SEARCHING
    # Fresh-by-age but from before stopping is still not a confirmation frame.
    fsm.target_pose_map_callback(pose(10.1))
    assert fsm.search.mode == 'DWELL'
    for now in (10.3, 11.2, 11.3, 11.4):
        fsm.advance(now);fsm.target_pose_map_callback(pose(now))
    assert fsm.current_task_state == TaskState.WAIT_BASE_POSE


def test_manual_takeover_cancels_exploration_navigation(fsm):
    explorer_ready(fsm);arm(fsm);complete_empty_scan(fsm)
    assert set_mode(fsm, False).success
    assert fsm.current_task_state == TaskState.STOPPING
    fsm.navigation_result_callback(Bool(data=False))
    assert fsm.control_mode == 'MANUAL' and fsm.current_task_state == TaskState.IDLE


@pytest.mark.parametrize('missing', ['detector', 'odometry', 'costmap'])
def test_exploration_sensor_failure_brakes_and_waits_for_cancel_ack(fsm, missing):
    explorer_ready(fsm);arm(fsm);complete_empty_scan(fsm)
    if missing == 'detector':
        fsm._detector_received = None
    elif missing == 'odometry':
        fsm.search.last_odom = None
    else:
        fsm._exploration_map_received = None
    fsm._search_tick()
    assert fsm.current_task_state == TaskState.EXPLORATION_STOPPING
    fsm.navigation_result_callback(Bool(data=False))
    assert fsm.control_mode == 'MANUAL' and fsm.current_task_state == TaskState.FAILED


def test_exhausted_search_stops_without_repeated_spins(fsm):
    explorer_ready(fsm);arm(fsm)
    fsm.explorer.max_views = 1;fsm.explorer.goal_count = 1
    complete_empty_scan(fsm)
    assert fsm.current_task_state == TaskState.SEARCH_EXHAUSTED
    assert fsm.control_mode == 'MANUAL'
    fsm.navigation_goal_publisher.publish.assert_not_called()


def test_unreachable_views_are_blacklisted_with_unique_goal_ids(fsm):
    explorer_ready(fsm);arm(fsm);complete_empty_scan(fsm)
    for _ in range(3):
        assert fsm.current_task_state == TaskState.EXPLORING
        fsm.navigation_result_callback(Bool(data=False))
    assert fsm.control_mode == 'MANUAL' and fsm.current_task_state == TaskState.FAILED
    goals = [call.args[0] for call in fsm.navigation_goal_publisher.publish.call_args_list]
    assert len(goals) == 3
    assert len({(g.header.stamp.sec, g.header.stamp.nanosec) for g in goals}) == 3
    assert len(fsm.explorer.failed) == 3


def hint_frame(h, u=320., label='bottle'):
    msg = Detection2DArray()
    msg.header.stamp = h.get_clock().now().to_msg()
    msg.header.frame_id = 'camera_color_optical_frame'
    d = Detection2D()
    d.bbox.center.position.x, d.bbox.center.position.y = u, 240.
    d.bbox.size_x, d.bbox.size_y = 12., 35.
    r = ObjectHypothesisWithPose()
    r.hypothesis.class_id, r.hypothesis.score = label, .4
    d.results = [r]
    msg.detections = [d]
    return msg


def camera_ready(h):
    h._camera_width = 640
    h._camera_intrinsics = (500., 500., 320., 240.)
    transform = TransformStamped()
    q = transform.transform.rotation
    q.x, q.y, q.z, q.w = -.5, .5, -.5, .5
    h.tf_buffer.lookup_transform.return_value = transform


def test_brief_2d_only_glimpse_drives_short_safe_view_then_requires_new_confirmation(fsm):
    explorer_ready(fsm);camera_ready(fsm);arm(fsm)
    fsm.search.continuous_rotation = True
    original = fsm.saved_start_pose
    for t in (10.1, 10.3):
        fsm.advance(t);fsm._detector_callback(hint_frame(fsm))
    assert fsm.search.mode == 'CONFIRM'
    assert fsm.search_velocity_publisher.publish.call_args.args[0].angular.z == 0.
    assert fsm.active_target_pose is None
    # Detection vanishes; no depth / target_pose_map ever arrives.
    fsm.advance(14.4)
    empty = Detection2DArray();empty.header.stamp = fsm.get_clock().now().to_msg()
    fsm._detector_callback(empty)
    fsm._exploration_map_received = fsm.now
    fsm._search_tick()
    assert fsm.current_task_state == TaskState.EXPLORING
    goal = fsm.navigation_goal_publisher.publish.call_args.args[0]
    p = goal.pose.position
    assert p.x > fsm.start_pose.pose.position.x+.6
    assert math.dist((p.x, p.y), (.15, -.2)) <= 1.5
    assert fsm.explorer.pose_free(p.x, p.y, 0.)
    assert not fsm.explorer.visited  # A glimpse is not a completed full scan.
    assert fsm.saved_start_pose is original and fsm.active_target_pose is None
    fsm.advance(15.)
    fsm.start_pose = pose(15., x=p.x, y=p.y)
    fsm.navigation_result_callback(Bool(data=True))
    assert fsm.current_task_state == TaskState.SEARCHING
    assert not fsm.bottle_hint.ready(fsm.now)
    assert fsm.saved_start_pose is original
    fsm.execute_pick_client.call_async.assert_not_called()


@pytest.mark.parametrize('invalid', ['stale', 'wrong_class', 'single', 'no_tf'])
def test_invalid_or_single_hints_never_start_pursuit(fsm, invalid):
    import tf2_ros
    explorer_ready(fsm);camera_ready(fsm);arm(fsm)
    if invalid == 'no_tf':
        fsm.tf_buffer.lookup_transform.side_effect = tf2_ros.LookupException('missing')
    for t in (10.1, 10.3):
        fsm.advance(t)
        msg = hint_frame(fsm, label='chair' if invalid == 'wrong_class' else 'bottle')
        if invalid == 'stale':
            msg.header.stamp = Time(seconds=1.).to_msg()
        if invalid != 'single' or t == 10.1:
            fsm._detector_callback(msg)
    assert not fsm.bottle_hint.ready(fsm.now)
    fsm.navigation_goal_publisher.publish.assert_not_called()


def test_hints_are_cleared_on_manual_takeover(fsm):
    explorer_ready(fsm);camera_ready(fsm);arm(fsm)
    for t in (10.1, 10.3):
        fsm.advance(t);fsm._detector_callback(hint_frame(fsm))
    assert fsm.bottle_hint.ready(fsm.now)
    set_mode(fsm, False)
    assert not fsm.bottle_hint.ready(fsm.now)
