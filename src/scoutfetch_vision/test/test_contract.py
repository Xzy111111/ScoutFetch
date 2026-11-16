#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Contract test for scoutfetch_vision.

Asserts the shared public interface the rest of the pipeline depends on is
preserved by the migration:

    publishes /target_pose      (base_link frame)
    publishes /target_pose_map  (map frame)
    publishes /target_marker    (base_link frame, visualization only)

and that the camera backend contract (D435i aligned-depth topics and the
camera_color_optical_frame -> base_link TF chain) is unchanged.
"""

import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src", "localization_node.cpp")
CFG = os.path.join(ROOT, "config", "localization.yaml")


def _read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def test_public_pose_topics():
    content = _read(SRC)
    assert '"/target_pose"' in content
    assert '"/target_pose_map"' in content
    assert '"/target_marker"' in content


def test_output_frames_are_base_link_and_map():
    cfg = _read(CFG)
    assert "target_frame: base_link" in cfg
    assert "marker_frame: base_link" in cfg
    # image_view 通过 TF camera_color_optical_frame -> base_link 才能转换到 base_link
    assert "optical_frame: camera_color_optical_frame" in cfg


def test_camera_backend_contract():
    cfg = _read(CFG)
    assert "depth_topic: /camera/camera/aligned_depth_to_color/image_raw" in cfg
    assert "depth_info_topic: /camera/camera/aligned_depth_to_color/camera_info" in cfg


def test_launch_has_pixel_source_and_config_args():
    launch = _read(os.path.join(ROOT, "launch", "localization.launch.py"))
    assert '"pixel_source"' in launch
    assert '"config_file"' in launch
    # use_sim_time must be settable: the node stamps /target_pose and looks up TF,
    # so its clock has to match the backend's.
    assert '"use_sim_time"' in launch


def test_yolo_mode_is_pose_independent_and_gated():
    # "yolo" takes the target pixel from a vision_msgs detection, so it keeps
    # working after the chassis has moved — the fixed pixel only matches the
    # pose it was calibrated at. It must publish nothing when there is no fresh
    # detection: task_fsm freezes whatever /target_pose_map it receives as the
    # task target, so a guessed coordinate is worse than no target.
    src = _read(SRC)
    launch = _read(os.path.join(ROOT, "launch", "localization.launch.py"))
    cfg = _read(CFG)
    assert '"yolo"' in src
    assert "detectionsCallback" in src
    assert 'choices=["yaml", "click", "yolo"]' in launch
    assert "detection_topic" in cfg
    assert "detection_timeout" in cfg
    # 抓取点不能直接用检测框中心: 水瓶的框含瓶颈/瓶盖, 框中心在肩部
    assert "grasp_offset_ratio" in cfg
    assert "grasp_offset_ratio" in src


def test_detector_speaks_standard_vision_msgs_and_is_configurable():
    # The detector must use the standard message type and take every
    # environment-specific value from parameters, so the real robot can point
    # the same node at its own model, classes and camera without code changes.
    script = _read(os.path.join(ROOT, "scripts", "yolo_detector.py"))
    cfg = _read(os.path.join(ROOT, "config", "yolo_detector.yaml"))
    launch = _read(os.path.join(ROOT, "launch", "yolo_detector.launch.py"))
    assert "Detection2DArray" in script
    for name in ("model_path", "classes_path", "target_classes",
                 "confidence_threshold", "image_topic", "detections_topic"):
        assert '"%s"' % name in script, name
    assert 'target_classes: ["bottle"]' in cfg
    for name in ("model_path", "classes_path", "image_topic", "use_sim_time"):
        assert '"%s"' % name in launch, name