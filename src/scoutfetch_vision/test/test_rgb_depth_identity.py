# SPDX-License-Identifier: Apache-2.0
"""Run the actual C++ localizer with deliberately offset RGB/depth stamps."""

import os
from pathlib import Path
import subprocess
import sys
import textwrap

from ament_index_python.packages import get_package_prefix


def test_target_keeps_rgb_detection_identity_with_matched_offset_depth():
    executable = Path(get_package_prefix('scoutfetch_vision'))/'lib/scoutfetch_vision/localization_node'
    config = Path(__file__).parents[1]/'config/localization.yaml'
    script = textwrap.dedent('''
        import array, os, signal, subprocess, sys, time
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.duration import Duration
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from geometry_msgs.msg import TransformStamped, PoseStamped
        from sensor_msgs.msg import Image, CameraInfo
        from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        process=subprocess.Popen([sys.argv[1], '--ros-args', '--params-file', sys.argv[2],
            '-p', 'pixel_source:=yolo', '-p', 'use_sim_time:=false'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        rclpy.init(); node=rclpy.create_node('rgb_depth_identity_regression')
        executor=SingleThreadedExecutor();executor.add_node(node)
        depth=node.create_publisher(Image,'/camera/camera/aligned_depth_to_color/image_raw',10)
        info=node.create_publisher(CameraInfo,'/camera/camera/aligned_depth_to_color/camera_info',10)
        detections=node.create_publisher(Detection2DArray,'/detections',10)
        transform=StaticTransformBroadcaster(node)
        transforms=[]
        for parent,child in [('map','base_link'),('base_link','camera_color_optical_frame')]:
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
            t.transform.rotation.w=1.;transforms.append(t)
        transform.sendTransform(transforms)
        received=[]; rgb_stamps=set(); depth_stamps=set()
        node.create_subscription(PoseStamped,'/target_pose_map',lambda m:received.append(m),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        stamp_ns=lambda s:s.sec*10**9+s.nanosec
        try:
            end=time.monotonic()+9;last=0
            while time.monotonic()<end and not received:
                executor.spin_once(timeout_sec=.01)
                if time.monotonic()-last<.1:continue
                last=time.monotonic();now=node.get_clock().now()
                image=Image();image.width=64;image.height=48;image.step=256
                image.encoding='32FC1';image.data=array.array('f',[1.]*64*48).tobytes()
                image.header.frame_id='camera_color_optical_frame';image.header.stamp=now.to_msg()
                camera=CameraInfo();camera.header=image.header;camera.width=64;camera.height=48
                camera.k=[50.,0.,32.,0.,50.,24.,0.,0.,1.]
                frame=Detection2DArray();frame.header.frame_id=image.header.frame_id
                frame.header.stamp=(now-Duration(seconds=.05)).to_msg()
                detection=Detection2D();detection.bbox.center.position.x=32.
                detection.bbox.center.position.y=24.;detection.bbox.size_x=10.;detection.bbox.size_y=12.
                hypothesis=ObjectHypothesisWithPose();hypothesis.hypothesis.class_id='bottle'
                hypothesis.hypothesis.score=.9;detection.results=[hypothesis];frame.detections=[detection]
                rgb_stamps.add(stamp_ns(frame.header.stamp));depth_stamps.add(stamp_ns(image.header.stamp))
                info.publish(camera);depth.publish(image);detections.publish(frame)
            assert received, 'C++ localizer produced no target'
            result=stamp_ns(received[0].header.stamp)
            assert result in rgb_stamps and result not in depth_stamps, result
        finally:
            executor.shutdown();node.destroy_node();rclpy.shutdown()
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=3)
    ''')
    result = subprocess.run([sys.executable, '-c', script, str(executable), str(config)],
                            env=dict(os.environ, ROS_DOMAIN_ID='123'),
                            text=True, capture_output=True, timeout=20)
    assert result.returncode == 0, result.stdout+result.stderr
