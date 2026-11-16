#!/usr/bin/env python3
"""Simulation backend gripper adapter.

The hardware scoutfetch_grasp layer commands the end effector through the
hardware-private topics:

  /control/gripper_cmd       sensor_msgs/msg/JointState
      name=["gripper"], position[0]=width (m), effort[0]=force (N)
  /feedback/gripper_status   scoutfetch_interfaces/msg/GripperStatus

On real hardware these are served directly by ``agx_arm_ctrl_single_node`` and
the driver closes the gripper toward ``width`` with ``force`` as the clamp force.
In Gazebo the gripper is a single prismatic joint ``gripper`` (0..0.1 m, equal
to the hardware width range) driven by ``gripper_effort_controller`` (the
standard ``effort_controllers/GripperActionController``). That controller maps a
``control_msgs/action/GripperCommand`` (position + max_effort) to an effort PID
whose output is clamped to ``max_effort``, so the sim honours the same
``width`` + ``force`` semantics as the hardware: a close that stops at an
object's outer diameter keeps pushing with up to ``force`` N and holds it.

width -> gripper joint position is a 1:1 mapping in metres. This node exposes
the same public command/feedback pair and translates commands into that action,
so shared upper-layer code needs no sim-specific branch.
"""

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import GripperCommand, FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scoutfetch_interfaces.msg import GripperStatus

GRIPPER_JOINT = 'gripper'
# Gazebo 不执行 URDF <mimic>: 两根手指由 gripper_finger_controller 直接驱动。
# 关节值 = ±width/2 (joint1 正方向使左指沿 -Y 移动, joint2 为负)。
FINGER_JOINTS = ('gripper_joint1', 'gripper_joint2')
WIDTH_MIN = 0.0
WIDTH_MAX = 0.1
FORCE_MIN = 0.5
FORCE_MAX = 3.0
DEFAULT_FORCE = 1.0
STATUS_RATE_HZ = 10.0


class GripperAdapter(Node):
    """Bridge hardware-compatible gripper topics to the Gazebo controller."""

    def __init__(self):
        super().__init__('sim_gripper_adapter')

        # Hardware-compatible public contract.
        self._status_pub = self.create_publisher(
            GripperStatus, 'feedback/gripper_status', 1)
        self.create_subscription(JointState, 'control/gripper_cmd', self._on_cmd, 1)

        # Actual gripper position read back from the joint state broadcaster.
        self._last_width = None
        self._joint_stamp = None
        self._joint_received = None
        self.create_subscription(JointState, 'joint_states', self._on_joint_states, 10)

        self._client = ActionClient(
            self, GripperCommand,
            '/gripper_effort_controller/gripper_cmd')
        self._finger_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_finger_controller/follow_joint_trajectory')

        self._status_timer = self.create_timer(
            1.0 / STATUS_RATE_HZ, self._publish_status)

        self.get_logger().info(
            'sim gripper adapter ready; listening on /control/gripper_cmd '
            'and publishing /feedback/gripper_status')

    def _on_cmd(self, msg):
        if GRIPPER_JOINT not in msg.name:
            self.get_logger().warn(
                f"[gripper_cmd] expected name=['{GRIPPER_JOINT}'], got {msg.name}")
            return
        idx = msg.name.index(GRIPPER_JOINT)
        if len(msg.position) <= idx:
            self.get_logger().warn('[gripper_cmd] missing position (width)')
            return

        width = float(msg.position[idx])
        force = float(msg.effort[idx]) if len(msg.effort) > idx else DEFAULT_FORCE
        width = max(WIDTH_MIN, min(WIDTH_MAX, width))
        force = max(FORCE_MIN, min(FORCE_MAX, force))
        self.get_logger().info(
            f'[gripper_cmd] move gripper width={width:.4f} force={force:.2f}')
        self._request_width(width, force)

    def _on_joint_states(self, msg):
        # width 必须来自真实手指位置: 主关节 gripper 只是宽度语义代理,
        # 夹住物体时手指会停在物体外径, 主关节却会照常到位。
        if all(j in msg.name for j in FINGER_JOINTS):
            idx1 = msg.name.index(FINGER_JOINTS[0])
            idx2 = msg.name.index(FINGER_JOINTS[1])
            self._last_width = float(msg.position[idx1] - msg.position[idx2])
            self._joint_stamp = msg.header.stamp
            self._joint_received = time.monotonic()
        elif GRIPPER_JOINT in msg.name:
            idx = msg.name.index(GRIPPER_JOINT)
            self._last_width = float(msg.position[idx])

    def _request_width(self, width, force):
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'gripper_effort_controller action server not available; dropping command')
            return
        goal = GripperCommand.Goal()
        # `force` is the clamp force (N): the effort PID drives the joint to
        # `width` but never exceeds this effort, so a blocked close holds the
        # object with exactly the force the upper layer requested.
        goal.command.position = width
        goal.command.max_effort = force
        self._client.send_goal_async(goal)
        self._request_fingers(width)

    def _request_fingers(self, width):
        if not self._finger_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'gripper_finger_controller action server not available; dropping command')
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(FINGER_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [0.5 * width, -0.5 * width]
        # 闭合放慢到 2 s: 1 s 时首触冲量大, 轻瓶会被先到的一侧手指推离指间。
        closing = self._last_width is not None and width < self._last_width
        pt.time_from_start.sec = 2 if closing else 1
        goal.trajectory.points = [pt]
        self._finger_client.send_goal_async(goal)

    def _publish_status(self):
        status = GripperStatus()
        if self._joint_stamp is None:
            return
        # Preserve the measurement timestamp. Re-publishing cached width must
        # not masquerade as a live gripper sensor when joint feedback stops.
        status.header.stamp = self._joint_stamp
        status.width = self._last_width if self._last_width is not None else 0.0
        # The effort controller applies real clamp force, but the public
        # GripperStatus contract still reports width as the gate; force is left
        # 0.0 for API stability (grasp is gated on width, AD-008).
        status.force = 0.0
        status.voltage_too_low = False
        status.motor_overheating = False
        status.driver_overcurrent = False
        status.driver_overheating = False
        status.sensor_status = self._joint_received is None or time.monotonic()-self._joint_received > 1.0
        status.driver_error_status = False
        status.driver_enable_status = not status.sensor_status
        status.homing_status = True
        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = GripperAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
