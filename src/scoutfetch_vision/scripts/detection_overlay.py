#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Render existing YOLO results on their exact source images for display."""

from collections import OrderedDict

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


def render_detections(image, detections):
    output = image.copy()
    height, width = output.shape[:2]
    cv2.putText(output, f'YOLO: {len(detections.detections)} detections',
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2, cv2.LINE_AA)
    for detection in detections.detections:
        box = detection.bbox
        x1 = max(0, min(width - 1, round(box.center.position.x - box.size_x / 2)))
        y1 = max(0, min(height - 1, round(box.center.position.y - box.size_y / 2)))
        x2 = max(0, min(width - 1, round(box.center.position.x + box.size_x / 2)))
        y2 = max(0, min(height - 1, round(box.center.position.y + box.size_y / 2)))
        cv2.rectangle(output, (x1, y1), (x2, y2), (0, 220, 0), 2)
        if detection.results:
            best = max(detection.results, key=lambda result: result.hypothesis.score).hypothesis
            text = f'{best.class_id} {best.score:.0%}'
            cv2.putText(output, text, (x1, max(45, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2, cv2.LINE_AA)
    return output


class DetectionOverlay(Node):
    def __init__(self):
        super().__init__('detection_overlay')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('annotated_topic', '/camera/yolo/image_annotated')
        self.bridge = CvBridge()
        self.images = OrderedDict()
        self.detections = OrderedDict()
        self.publisher = self.create_publisher(
            Image, self.get_parameter('annotated_topic').value, 1)
        self.image_sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.image_callback, 10)
        self.detection_sub = self.create_subscription(
            Detection2DArray, self.get_parameter('detections_topic').value,
            self.detection_callback, 10)
        self.get_logger().info('YOLO 标注图像：' + self.get_parameter('annotated_topic').value)

    @staticmethod
    def key(message):
        return (message.header.stamp.sec, message.header.stamp.nanosec, message.header.frame_id)

    def image_callback(self, message):
        self.images[self.key(message)] = message
        self.publish_match(self.key(message))

    def detection_callback(self, message):
        self.detections[self.key(message)] = message
        self.publish_match(self.key(message))

    def publish_match(self, key):
        # DDS may deliver either topic first. Only draw boxes on the exact
        # image used by YOLO; never carry an old box into a newer/empty frame.
        if key in self.images and key in self.detections:
            image = self.images.pop(key)
            detections = self.detections.pop(key)
            bgr = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            annotated = self.bridge.cv2_to_imgmsg(render_detections(bgr, detections), encoding='bgr8')
            annotated.header = image.header
            self.publisher.publish(annotated)
        for cache in (self.images, self.detections):
            while len(cache) > 32:
                cache.popitem(last=False)


def main():
    rclpy.init()
    node = DetectionOverlay()
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
