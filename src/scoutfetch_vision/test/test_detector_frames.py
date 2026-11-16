# SPDX-License-Identifier: Apache-2.0
"""Regression images captured from actual Gazebo runs, not generated labels."""

import importlib.util
from pathlib import Path

import cv2
import onnxruntime as ort
import pytest
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
FRAMES = ROOT / 'test/data'
SPEC = importlib.util.spec_from_file_location('yolo_frame_test', ROOT / 'scripts/yolo_detector.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture(scope='module')
def detector():
    node = MODULE.YoloDetector.__new__(MODULE.YoloDetector)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    options.inter_op_num_threads = 1
    node.session = ort.InferenceSession(str(ROOT / 'models/yolov8s.onnx'), options,
                                       providers=['CPUExecutionProvider'])
    node.input_name = node.session.get_inputs()[0].name
    node.input_size = 640
    node.confidence_threshold = 0.20
    node.iou_threshold = 0.45
    node.class_names = (ROOT / 'models/coco_classes.txt').read_text().splitlines()
    node.target_classes = ['bottle']
    return node


@pytest.mark.parametrize('filename', ['seed42_camera_before_pick.jpg', 'seed7_camera_before_pick.jpg'])
def test_rendered_bottle_is_detected_at_normal_threshold(detector, filename):
    detections = detector._infer(cv2.imread(str(FRAMES / filename)))
    assert len(detections) == 1
    label, confidence, cx, cy, width, height = detections[0]
    assert label == 'bottle' and confidence > 0.5
    assert 300 < cx < 340 and 245 < cy < 280
    assert 25 < width < 50 and 90 < height < 135


def test_no_bottle_view_does_not_trigger_search_target(detector):
    assert detector._infer(cv2.imread(str(FRAMES / 'no_bottle_initial_view.jpg'))) == []


def test_annotation_is_from_the_inference_frame_and_empty_result_clears_boxes(detector):
    image = cv2.imread(str(FRAMES/'seed42_camera_before_pick.jpg'))
    original = image.copy()
    marked = MODULE.draw_detections(image, detector._infer(image))
    assert np.any(marked != original)
    np.testing.assert_array_equal(image, original)
    empty = MODULE.draw_detections(image, [])
    np.testing.assert_array_equal(empty[50:], original[50:])
