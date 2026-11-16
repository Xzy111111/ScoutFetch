#!/usr/bin/env python3
"""target_pose_resolver: 无状态显式 Pose 转换器 (A5c)。

职责边界 (A5c 起):
  1. 收到一条 map 系 PoseStamped (/resolve_grasp_target), 该消息本身就是需要解析的目标
  2. 验证 request (frame==map, stamp!=0, position finite, quaternion valid)
  3. 用触发时刻当前 TF 将 map 目标转换到 base_link (tf2_ros, 不手写矩阵)
  4. 发布 /grasp_target_pose (frame_id = base_link)

无任务状态:
  不再订阅 /target_pose_map, 不再保存 latest_target_, 不知道当前任务是什么。
  目标身份由 /resolve_grasp_target 消息自带, 消除"导航期间新 target 覆盖任务目标"的风险。

时间语义:
  - source_target_stamp: request 的原始 stamp (FSM 锁定的任务目标身份, 保持不变)
  - resolved_pose_stamp:  完成 TF resolve 的当前 node time (输出 header.stamp)
  每次收到 request 都查询当前 TF, 不缓存、不手算、不 fallback 到旧变换。
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose

import tf2_ros

# map->base_link 查询等待上限: 短超时 fail-closed, 不做复杂 retry
_TF_LOOKUP_TIMEOUT_SEC = 1.0
_QUATERNION_NORM_TOLERANCE = 1e-3


class TargetPoseResolver(Node):
    def __init__(self):
        super().__init__("target_pose_resolver")

        self.declare_parameter("trigger_topic", "/resolve_grasp_target")
        self.declare_parameter("output_topic", "/grasp_target_pose")
        self.declare_parameter("source_frame", "map")
        self.declare_parameter("target_frame", "base_link")

        self.trigger_topic = self.get_parameter("trigger_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.source_frame = self.get_parameter("source_frame").value
        self.target_frame = self.get_parameter("target_frame").value

        # /resolve_grasp_target 是任务执行事件, 不是状态: 必须 VOLATILE,
        # 旧任务的 request 不得在 resolver 重启后重新送达 (禁止 TRANSIENT_LOCAL)。
        trigger_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.trigger_sub = self.create_subscription(
            PoseStamped, self.trigger_topic, self.resolve_trigger_callback, trigger_qos)

        self.grasp_target_pub = self.create_publisher(
            PoseStamped, self.output_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            "target_pose_resolver ready (stateless): trigger=%s output=%s "
            "source_frame=%s target_frame=%s"
            % (self.trigger_topic, self.output_topic,
               self.source_frame, self.target_frame))

    def resolve_trigger_callback(self, request):
        if not self._is_valid_request(request):
            return
        resolved = self._resolve_to_base_link(request)
        if resolved is None:
            return
        self.grasp_target_pub.publish(resolved)
        source_stamp = request.header.stamp
        self.get_logger().info(
            "GRASP TARGET RESOLVED\n"
            "  source_stamp: %d.%09d\n"
            "  resolved_stamp: %d.%09d\n"
            "  frame: %s\n"
            "  x: %.4f y: %.4f z: %.4f"
            % (source_stamp.sec, source_stamp.nanosec,
               resolved.header.stamp.sec, resolved.header.stamp.nanosec,
               resolved.header.frame_id,
               resolved.pose.position.x, resolved.pose.position.y, resolved.pose.position.z))

    def _is_valid_request(self, request):
        if request.header.frame_id != self.source_frame:
            self.get_logger().warn(
                "RESOLVE REJECTED: frame=%s (expected %s)"
                % (request.header.frame_id, self.source_frame))
            return False
        if request.header.stamp.sec == 0 and request.header.stamp.nanosec == 0:
            self.get_logger().warn("RESOLVE REJECTED: empty stamp")
            return False
        position = request.pose.position
        for component in (position.x, position.y, position.z):
            if not math.isfinite(component):
                self.get_logger().warn("RESOLVE REJECTED: non-finite position")
                return False
        if not self._is_valid_quaternion(request.pose.orientation):
            self.get_logger().warn("RESOLVE REJECTED: invalid quaternion")
            return False
        return True

    def _is_valid_quaternion(self, quaternion):
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

    def _resolve_to_base_link(self, request):
        try:
            # Time() == 0 -> 取触发时刻可获得的"最新"map->base_link 变换
            transform_stamped = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, Time(),
                timeout=Duration(seconds=_TF_LOOKUP_TIMEOUT_SEC))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                "RESOLVE FAILED: TF unavailable (%s->%s): %s"
                % (self.source_frame, self.target_frame, e))
            return None

        # position + orientation 都由 TF2 完整转换, 不改写 orientation
        resolved_pose = do_transform_pose(request.pose, transform_stamped)

        resolved = PoseStamped()
        resolved.header.frame_id = self.target_frame
        # resolved_stamp: 触发时刻完成 TF resolve 的当前 node time
        resolved.header.stamp = self.get_clock().now().to_msg()
        resolved.pose = resolved_pose
        return resolved


def main():
    rclpy.init()
    node = TargetPoseResolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
