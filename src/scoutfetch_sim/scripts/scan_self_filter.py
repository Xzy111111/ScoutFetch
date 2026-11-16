#!/usr/bin/env python3
"""Remove chassis returns without introducing a half-metre lidar blind zone."""

import math
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


def filter_self(scan, x, y, yaw, half_x=.305, half_y=.23):
    result = deepcopy(scan)
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance):
            continue
        angle = scan.angle_min+i*scan.angle_increment+yaw
        px, py = x+distance*math.cos(angle), y+distance*math.sin(angle)
        if abs(px) <= half_x and abs(py) <= half_y:
            # NaN contributes neither a mark nor a clearing ray. A self return
            # says nothing about the unseen region behind the chassis.
            result.ranges[i] = float('nan')
    return result


class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__('sim_scan_self_filter')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(LaserScan, '/livox/scan', qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.scan, qos_profile_sensor_data)

    def scan(self, message):
        try:
            t = self.buffer.lookup_transform('base_link', message.header.frame_id, Time()).transform
        except TransformException:
            return
        q = t.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.pub.publish(filter_self(message, t.translation.x, t.translation.y, yaw))


def main():
    rclpy.init()
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
