#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""yolo_detector: YOLO (ONNX) 目标检测节点。

只做检测这一件事: 订阅彩色图, 用 onnxruntime 跑 YOLO ONNX, 按目标类别过滤后
发布 vision_msgs/Detection2DArray。不碰深度、TF、位姿 —— 三维定位仍由
localization_node 负责 (它按 pixel_source:=yolo 消费本节点的检测框中心)。

接口 (仿真与真机完全一致, 仿真只是换掉 image_topic / model_path):
    sub  image_topic      sensor_msgs/Image            默认 /camera/camera/color/image_raw
    pub  detections_topic vision_msgs/Detection2DArray  默认 /detections

依赖: onnxruntime (pip 安装; 无 rosdep key), numpy, opencv-python (cv2)。

模型: 标准 YOLOv8 检测导出, 输入 1x3xSxS float32 0..1 RGB, 输出 1x(4+nc)xN,
      其中前 4 行为框中心/宽高 (letterbox 后的像素单位), 其余为各类别分数。
      COCO 预训练模型的 bottle 类是第 39 号 (classes 文件第 40 行)。
"""

import os

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


def _to_bgr(msg):
    """sensor_msgs/Image -> BGR uint8 ndarray; 未知编码返回 None。"""
    height, width = msg.height, msg.width
    channels = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}.get(msg.encoding)
    if channels is None:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.step != width * channels:
        raw = raw.reshape(height, msg.step)[:, : width * channels]
    image = raw.reshape(height, width, channels)
    if channels == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR if msg.encoding == "bgra8"
                            else cv2.COLOR_RGBA2BGR)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def _letterbox(image, size):
    """等比缩放到 size x size 并居中填充 114; 返回 (图, 比例, 左填充, 上填充)。"""
    height, width = image.shape[:2]
    ratio = min(size / height, size / width)
    new_w, new_h = round(width * ratio), round(height * ratio)
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    pad_w = (size - new_w) / 2.0
    pad_h = (size - new_h) / 2.0
    left, top = round(pad_w - 0.1), round(pad_h - 0.1)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[top:top + new_h, left:left + new_w] = resized
    return canvas, ratio, left, top


def draw_detections(image, detections):
    canvas = image.copy()
    cv2.putText(canvas, f'YOLO: {len(detections)} detections', (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 220, 0), 2, cv2.LINE_AA)
    for name, score, cx, cy, width, height in detections:
        x1, y1 = round(cx-width/2), round(cy-height/2)
        x2, y2 = round(cx+width/2), round(cy+height/2)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 220, 0), 2)
        cv2.putText(canvas, f'{name} {score:.0%}', (x1, max(45, y1-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 220, 0), 2, cv2.LINE_AA)
    return canvas


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self.declare_parameter("model_path", "")
        self.declare_parameter("classes_path", "")
        self.declare_parameter("target_classes", ["bottle"])
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("input_size", 640)
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("inference_rate", 10.0)
        self.declare_parameter("inference_threads", 2)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('annotated_topic', '/camera/yolo/image_annotated')

        model_path = self.get_parameter("model_path").value
        classes_path = self.get_parameter("classes_path").value
        self.target_classes = list(self.get_parameter("target_classes").value)
        self.confidence_threshold = self.get_parameter("confidence_threshold").value
        self.iou_threshold = self.get_parameter("iou_threshold").value
        self.input_size = int(self.get_parameter("input_size").value)
        image_topic = self.get_parameter("image_topic").value
        detections_topic = self.get_parameter("detections_topic").value
        inference_rate = max(float(self.get_parameter("inference_rate").value), 0.1)
        self.min_interval_ns = int(1e9 / inference_rate)

        if not model_path or not os.path.isfile(model_path):
            raise RuntimeError("yolo_detector: model_path 不存在: '%s'" % model_path)

        self.class_names = self._load_classes(classes_path)

        options = ort.SessionOptions()
        options.log_severity_level = 3
        # Keep CPU inference bounded so it does not starve sensor, TF, or control
        # callbacks when Gazebo and the planner run on the same workstation.
        options.intra_op_num_threads = max(1, int(self.get_parameter("inference_threads").value))
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            model_path, options, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        shape = self.session.get_inputs()[0].shape
        if isinstance(shape[2], int) and shape[2] > 0 and shape[2] != self.input_size:
            self.get_logger().warn(
                "模型输入为 %s, 与 input_size=%d 不一致, 采用模型值"
                % (list(shape), self.input_size))
            self.input_size = shape[2]

        # 与仿真相机一致: 图像发布端为 RELIABLE, 订阅端不能直接用 BEST_EFFORT
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, image_qos)
        self.detection_pub = self.create_publisher(
            Detection2DArray, detections_topic, detection_qos)
        self.annotated_pub = (self.create_publisher(
            Image, self.get_parameter('annotated_topic').value, image_qos)
            if self.get_parameter('publish_annotated').value else None)

        self.last_inference_ns = 0
        self.get_logger().info(
            "yolo_detector ready: model=%s input=%dx%d classes=%d target=%s "
            "conf>=%.2f iou<=%.2f image=%s -> %s"
            % (model_path, self.input_size, self.input_size, len(self.class_names),
               self.target_classes, self.confidence_threshold, self.iou_threshold,
               image_topic, detections_topic))

    def _load_classes(self, classes_path):
        if not classes_path or not os.path.isfile(classes_path):
            self.get_logger().warn(
                "classes_path 未设置或不存在, 类别名用序号代替 (target_classes 需写成序号)")
            return []
        with open(classes_path, encoding="utf-8") as stream:
            return [line.strip() for line in stream if line.strip()]

    def _class_name(self, index):
        if 0 <= index < len(self.class_names):
            return self.class_names[index]
        return str(index)

    def image_callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_inference_ns < self.min_interval_ns:
            return
        self.last_inference_ns = now_ns

        image = _to_bgr(msg)
        if image is None:
            self.get_logger().warn(
                "不支持的图像编码 '%s' (需要 bgr8/rgb8/bgra8/rgba8)", msg.encoding,
                throttle_duration_sec=5.0)
            return

        detections = self._infer(image)
        self._publish(msg, detections)
        # Publish the exact inference frame directly: no second RGB subscription
        # or cross-topic synchronizer can drop the displayed result under load.
        if self.annotated_pub is not None:
            annotated = draw_detections(image, detections)
            output = Image()
            output.header = msg.header
            output.height, output.width = annotated.shape[:2]
            output.encoding = 'bgr8'
            output.step = output.width*3
            output.data = annotated.tobytes()
            self.annotated_pub.publish(output)

    def _infer(self, image):
        canvas, ratio, pad_left, pad_top = _letterbox(image, self.input_size)
        blob = canvas[:, :, ::-1].astype(np.float32) / 255.0  # BGR->RGB, 0..1
        blob = np.ascontiguousarray(blob.transpose(2, 0, 1)[None])

        output = self.session.run(None, {self.input_name: blob})[0]
        # YOLOv8 导出: (1, 4+nc, N) -> (N, 4+nc)
        prediction = output[0].transpose()
        if prediction.shape[1] < 5:
            return []

        boxes_xywh = prediction[:, :4]
        scores_all = prediction[:, 4:]
        class_ids = np.argmax(scores_all, axis=1)
        scores = scores_all[np.arange(len(class_ids)), class_ids]

        keep = scores >= self.confidence_threshold
        if not np.any(keep):
            return []
        boxes_xywh, scores, class_ids = boxes_xywh[keep], scores[keep], class_ids[keep]

        # xywh(中心) -> xywh(左上角), 单位仍是 letterbox 后的像素
        corners = np.empty_like(boxes_xywh)
        corners[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2.0
        corners[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2.0
        corners[:, 2] = boxes_xywh[:, 2]
        corners[:, 3] = boxes_xywh[:, 3]

        indices = cv2.dnn.NMSBoxes(
            corners.tolist(), scores.tolist(), self.confidence_threshold, self.iou_threshold)
        if len(indices) == 0:
            return []

        wanted = set(self.target_classes)
        image_h, image_w = image.shape[:2]
        results = []
        for i in np.array(indices).flatten():
            name = self._class_name(int(class_ids[i]))
            if wanted and name not in wanted:
                continue
            # 从 letterbox 坐标还原到原图像素
            x = (corners[i, 0] - pad_left) / ratio
            y = (corners[i, 1] - pad_top) / ratio
            w = corners[i, 2] / ratio
            h = corners[i, 3] / ratio
            x = float(np.clip(x, 0.0, image_w - 1.0))
            y = float(np.clip(y, 0.0, image_h - 1.0))
            w = float(min(w, image_w - x))
            h = float(min(h, image_h - y))
            results.append((name, float(scores[i]), x + w / 2.0, y + h / 2.0, w, h))
        return results

    def _publish(self, image_msg, detections):
        array = Detection2DArray()
        array.header = image_msg.header  # 时间戳/帧与原图一致, 便于与深度对齐
        for name, score, cx, cy, w, h in detections:
            detection = Detection2D()
            detection.header = image_msg.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = name
            hypothesis.hypothesis.score = score
            detection.results.append(hypothesis)
            detection.bbox.center.position.x = cx
            detection.bbox.center.position.y = cy
            detection.bbox.size_x = w
            detection.bbox.size_y = h
            array.detections.append(detection)

        self.detection_pub.publish(array)
        if detections:
            best = max(detections, key=lambda d: d[1])
            self.get_logger().info(
                "detections=%d best=%s score=%.3f center=(%.1f, %.1f) size=%.1fx%.1f"
                % (len(detections), best[0], best[1], best[2], best[3], best[4], best[5]))


def main():
    rclpy.init()
    try:
        node = YoloDetector()
    except Exception as exc:  # 配置/模型错误必须让 launch 立刻看到失败
        rclpy.logging.get_logger("yolo_detector").fatal(str(exc))
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
