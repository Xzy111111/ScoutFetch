#!/usr/bin/env python3
"""nav_goal_executor: task_fsm 的唯一 Nav2 NavigateToPose 派发者。

/navigation_goal 承载搜索、去抓取站位和返回三种目标; /navigation_result
只对当前航段发布一次。/navigation_cancel 用来源 stamp 指定要取消的航段,
避免迟到的取消请求误伤后续导航。

enable_navigation=false (默认) 时只做检查与打印, 不产生任何底盘运动。
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, Header

_STATUS_LABELS = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}

_QUATERNION_NORM_TOLERANCE = 1e-3
_SERVER_WAIT_TIMEOUT_SEC = 3.0
_SERVER_RETRY_PERIOD_SEC = 0.05


def _yaw_from_orientation(quaternion):
    sin_yaw_cos_pitch = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)


def _is_valid_quaternion(quaternion):
    for component in (quaternion.x, quaternion.y, quaternion.z, quaternion.w):
        if not math.isfinite(component):
            return False
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    return abs(norm - 1.0) <= _QUATERNION_NORM_TOLERANCE


def _stamp_key(stamp):
    return stamp.sec, stamp.nanosec


class NavGoalExecutor(Node):
    def __init__(self):
        super().__init__("nav_goal_executor")

        self.declare_parameter("enable_navigation", False)
        self.declare_parameter("goal_topic", "/navigation_goal")
        self.declare_parameter("cancel_topic", "/navigation_cancel")
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("expected_frame", "map")
        self.declare_parameter("result_topic", "/navigation_result")

        self.enable_navigation = self.get_parameter("enable_navigation").value
        goal_topic = self.get_parameter("goal_topic").value
        cancel_topic = self.get_parameter("cancel_topic").value
        navigate_action = self.get_parameter("navigate_action").value
        self.expected_frame = self.get_parameter("expected_frame").value
        self.result_topic = self.get_parameter("result_topic").value

        goal_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._navigation_goal_sub = self.create_subscription(
            PoseStamped, goal_topic, self.navigation_goal_callback, goal_qos)
        self._navigation_cancel_sub = self.create_subscription(
            Header, cancel_topic, self.navigation_cancel_callback, goal_qos)

        self.navigate_action_client = ActionClient(self, NavigateToPose, navigate_action)

        result_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.navigation_result_publisher = self.create_publisher(
            Bool, self.result_topic, result_qos)

        self.processed_source_stamp = None
        self.goal_active = False
        self.active_source_stamp = None
        self.active_goal_handle = None
        self.cancel_requested = False
        self.cancel_in_flight = False
        self.pending_cancel_stamp = None
        self.pending_goal_msg = None
        self.dispatch_timer = None
        self.dispatch_deadline = None

        self.get_logger().info(
            "nav_goal_executor ready: goal_topic=%s cancel_topic=%s action=%s "
            "expected_frame=%s enable_navigation=%s"
            % (goal_topic, cancel_topic, navigate_action, self.expected_frame,
               self.enable_navigation))

    def navigation_goal_callback(self, msg):
        source_stamp = _stamp_key(msg.header.stamp)
        yaw_deg = math.degrees(_yaw_from_orientation(msg.pose.orientation))
        self.get_logger().info("NAVIGATION GOAL RECEIVED")
        self.get_logger().info(
            "source_stamp: %d.%09d frame: %s x: %.4f y: %.4f yaw: %.2f deg"
            % (source_stamp[0], source_stamp[1], msg.header.frame_id,
               msg.pose.position.x, msg.pose.position.y, yaw_deg))

        if not self._is_valid_pose(msg):
            return
        if self.processed_source_stamp == source_stamp:
            self.get_logger().warn("IGNORE duplicate navigation goal")
            return
        self.processed_source_stamp = source_stamp

        if self.goal_active:
            self.get_logger().warn("IGNORED: navigation goal already active")
            return
        if not self.enable_navigation:
            self.get_logger().warn(
                "DRY RUN (enable_navigation=false): would send goal x=%.4f y=%.4f yaw=%.2f deg"
                % (msg.pose.position.x, msg.pose.position.y, yaw_deg))
            return
        self._send_navigation_goal(msg)

    def navigation_cancel_callback(self, msg):
        source_stamp = _stamp_key(msg.stamp)
        if not self.goal_active:
            if self.processed_source_stamp == source_stamp:
                self.get_logger().warn("IGNORE late cancel for completed goal")
            else:
                self.pending_cancel_stamp = source_stamp
                self.get_logger().info("NAV CANCEL LATCHED for future goal")
            return
        if source_stamp != self.active_source_stamp:
            self.get_logger().warn("IGNORE cancel for unrelated navigation goal")
            return

        self.cancel_requested = True
        self.get_logger().info("NAV CANCEL REQUESTED")
        if self.pending_goal_msg is not None:
            self._finish_navigation(False)
        elif self.active_goal_handle is not None:
            self._request_cancel()

    def _is_valid_pose(self, msg):
        if not msg.header.frame_id:
            self.get_logger().warn("REJECTED: empty frame")
            return False
        if msg.header.frame_id != self.expected_frame:
            self.get_logger().warn(
                "REJECTED: frame=%s (expected %s), 不做自动 TF"
                % (msg.header.frame_id, self.expected_frame))
            return False
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            self.get_logger().warn("REJECTED: empty stamp")
            return False
        if not _is_valid_quaternion(msg.pose.orientation):
            self.get_logger().warn("REJECTED: invalid quaternion")
            return False
        return True

    def _send_navigation_goal(self, msg):
        source_stamp = _stamp_key(msg.header.stamp)
        self.goal_active = True
        self.active_source_stamp = source_stamp
        self.active_goal_handle = None
        self.cancel_requested = False
        self.cancel_in_flight = False
        self.pending_goal_msg = msg

        if self.pending_cancel_stamp == source_stamp:
            self.pending_cancel_stamp = None
            self.cancel_requested = True
            self.get_logger().warn("NAV GOAL CANCELED BEFORE DISPATCH")
            self._finish_navigation(False)
            return

        self.dispatch_deadline = self.get_clock().now() + Duration(
            seconds=_SERVER_WAIT_TIMEOUT_SEC)
        self.dispatch_timer = self.create_timer(
            _SERVER_RETRY_PERIOD_SEC, self._dispatch_tick)
        self._dispatch_tick()

    def _dispatch_tick(self):
        if not self.goal_active or self.pending_goal_msg is None:
            return
        if self.cancel_requested:
            self._finish_navigation(False)
            return
        if self.get_clock().now() >= self.dispatch_deadline:
            self.get_logger().warn("NAV SERVER UNAVAILABLE")
            self._finish_navigation(False)
            return
        if not self.navigate_action_client.server_is_ready():
            return

        msg = self.pending_goal_msg
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.expected_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = msg.pose
        self._stop_dispatch_timer()
        self.pending_goal_msg = None

        self.get_logger().info("NAV GOAL SEND")
        self.get_logger().info(
            "nav_goal_stamp: %d.%09d target=(%.4f, %.4f) yaw=%.2f deg"
            % (goal.pose.header.stamp.sec, goal.pose.header.stamp.nanosec,
               goal.pose.pose.position.x, goal.pose.pose.position.y,
               math.degrees(_yaw_from_orientation(goal.pose.pose.orientation))))
        try:
            send_goal_future = self.navigate_action_client.send_goal_async(goal)
        except Exception as exc:
            self.get_logger().error("NAV GOAL ERROR: %s" % exc)
            self._finish_navigation(False)
            return
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        if not self.goal_active:
            self.get_logger().warn("IGNORE goal response for inactive navigation goal")
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error("NAV GOAL ERROR: %s" % exc)
            self._finish_navigation(False)
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("NAV GOAL REJECTED")
            self._finish_navigation(False)
            return

        self.get_logger().info("NAV GOAL ACCEPTED")
        self.active_goal_handle = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)
        except Exception as exc:
            self.get_logger().error("NAV RESULT REQUEST ERROR: %s" % exc)
            self._finish_navigation(False)
            return
        if self.cancel_requested:
            self._request_cancel()

    def _request_cancel(self):
        if (self.active_goal_handle is None or self.cancel_in_flight
                or not self.cancel_requested):
            return
        self.cancel_in_flight = True
        try:
            cancel_future = self.active_goal_handle.cancel_goal_async()
        except Exception as exc:
            self.cancel_in_flight = False
            self.get_logger().error("NAV CANCEL ERROR: %s" % exc)
            return
        cancel_future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        self.cancel_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error("NAV CANCEL RESPONSE ERROR: %s" % exc)
            return
        self.get_logger().info("NAV CANCEL RESPONSE: %s" % response)

    def result_callback(self, future):
        if not self.goal_active:
            self.get_logger().warn("IGNORE result for inactive navigation goal")
            return
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error("NAV RESULT ERROR: %s" % exc)
            self._finish_navigation(False)
            return
        label = _STATUS_LABELS.get(status, "UNKNOWN")
        self.get_logger().info("NAV RESULT: %s (status=%d)" % (label, status))
        self._finish_navigation(status == GoalStatus.STATUS_SUCCEEDED)

    def _stop_dispatch_timer(self):
        if self.dispatch_timer is not None:
            self.dispatch_timer.cancel()
            self.destroy_timer(self.dispatch_timer)
            self.dispatch_timer = None
        self.dispatch_deadline = None

    def _finish_navigation(self, success):
        if not self.goal_active:
            return
        self._stop_dispatch_timer()
        self.goal_active = False
        self.active_source_stamp = None
        self.active_goal_handle = None
        self.cancel_requested = False
        self.cancel_in_flight = False
        self.pending_goal_msg = None
        self.publish_navigation_result(success)

    def publish_navigation_result(self, success):
        navigation_succeeded = Bool()
        navigation_succeeded.data = success
        self.navigation_result_publisher.publish(navigation_succeeded)
        self.get_logger().info(
            "NAV RESULT EVENT: success=%s" % ("true" if success else "false"))


def main():
    rclpy.init()
    node = NavGoalExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
