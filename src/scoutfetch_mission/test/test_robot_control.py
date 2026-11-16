"""Keyboard release, focus loss and mode acknowledgement must stop motion."""

import math
import os
import time
import subprocess
import sys
import textwrap
from types import SimpleNamespace
from unittest.mock import Mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5 import QtCore, QtGui, QtWidgets
from scoutfetch_mission.robot_control import ControlNode, ControlWindow, keyboard_twist
from std_msgs.msg import String


@pytest.fixture(scope='module')
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def window(app, monkeypatch):
    node = SimpleNamespace(mode='MANUAL', state='IDLE', pending=None, last_mode=time.monotonic(),
                           publisher=Mock(), request_mode=Mock(), image=None, notice='ready')
    monkeypatch.setattr('scoutfetch_mission.robot_control.rclpy.spin_once', lambda *a, **k: None)
    monkeypatch.setattr('scoutfetch_mission.robot_control.rclpy.ok', lambda: True)
    monkeypatch.setattr('scoutfetch_mission.robot_control.SingleThreadedExecutor', Mock)
    widget = ControlWindow(node)
    widget.timer.stop()
    monkeypatch.setattr(widget, 'isActiveWindow', lambda: True)
    yield widget
    widget.deleteLater()
    app.processEvents()


def press(window, key, text):
    window.keyPressEvent(QtGui.QKeyEvent(QtCore.QEvent.KeyPress, key, QtCore.Qt.NoModifier, text))


def test_diagonal_speed_is_bounded_and_opposite_keys_cancel():
    cmd = keyboard_twist({'w', 'a', 'q'})
    assert math.hypot(cmd.linear.x, cmd.linear.y) == pytest.approx(0.15)
    assert cmd.angular.z == pytest.approx(0.4)
    cmd = keyboard_twist({'w', 's', 'a', 'd', 'q', 'e'})
    assert cmd.linear.x == cmd.linear.y == cmd.angular.z == 0


def test_connection_notice_clears_on_external_auto_heartbeat():
    node = SimpleNamespace(mode='CONNECTING', last_mode=0., notice='正在连接机器人…')
    ControlNode.on_mode(node, String(data='AUTO'))
    assert node.mode == 'AUTO' and node.last_mode > 0
    assert '正在连接' not in node.notice
    node.notice = '正在取消导航'
    ControlNode.on_mode(node, String(data='MANUAL'))
    assert node.notice == '正在取消导航'


def test_key_release_stops_motion(window):
    press(window, QtCore.Qt.Key_W, 'w')
    window.tick()
    assert window.node.publisher.publish.call_args.args[0].linear.x > 0
    window.keyReleaseEvent(QtGui.QKeyEvent(QtCore.QEvent.KeyRelease, QtCore.Qt.Key_W,
                                          QtCore.Qt.NoModifier, 'w'))
    window.tick()
    assert window.node.publisher.publish.call_args.args[0].linear.x == 0


def test_focus_loss_and_missing_mode_heartbeat_stop_motion(window, monkeypatch):
    press(window, QtCore.Qt.Key_W, 'w')
    monkeypatch.setattr(window, 'isActiveWindow', lambda: False)
    window.tick()
    assert not window.keys and window.node.publisher.publish.call_args.args[0].linear.x == 0
    monkeypatch.setattr(window, 'isActiveWindow', lambda: True)
    press(window, QtCore.Qt.Key_W, 'w')
    window.node.last_mode = time.monotonic() - 2
    window.tick()
    assert not window.keys and window.node.publisher.publish.call_args.args[0].linear.x == 0


def test_manual_keys_cannot_override_auto_or_pending_handover(window):
    window.node.mode = 'AUTO'
    press(window, QtCore.Qt.Key_W, 'w')
    window.tick()
    assert not window.keys
    window.node.publisher.publish.assert_not_called()
    press(window, QtCore.Qt.Key_1, '1')
    window.node.request_mode.assert_called_once_with(False)
    window.node.mode = 'MANUAL'
    window.node.pending = object()
    press(window, QtCore.Qt.Key_W, 'w')
    window.tick()
    assert not window.keys and window.node.publisher.publish.call_args.args[0].linear.x == 0


def test_live_mode_and_image_callbacks_survive_high_rate_sim_clock():
    # A separate DDS domain ensures this regression never publishes a test
    # /clock or control state into the user's running simulation.
    script = textwrap.dedent('''
        import time, threading
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        from sensor_msgs.msg import Image
        from PyQt5 import QtWidgets
        from scoutfetch_mission.robot_control import ControlNode, ControlWindow
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
        app=QtWidgets.QApplication([])
        node=ControlNode(); window=ControlWindow(node); window.timer.stop()
        source=Node('camera_regression_source')
        clock=source.create_publisher(Clock,'/clock',1)
        mode=source.create_publisher(String,'/control_mode',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        image=source.create_publisher(Image,'/camera/yolo/image_annotated',1)
        stop=threading.Event()
        def publish():
            last=0
            while not stop.is_set():
                now=time.monotonic(); msg=Clock(); msg.clock.sec=int(now); msg.clock.nanosec=int((now-int(now))*1e9); clock.publish(msg)
                if now-last>.05:
                    last=now; mode.publish(String(data='AUTO'))
                    frame=Image(); frame.width=64;frame.height=48;frame.encoding='bgr8';frame.step=192;frame.data=bytes(64*48*3);image.publish(frame)
                time.sleep(.001)
        thread=threading.Thread(target=publish);thread.start()
        try:
            end=time.monotonic()+6
            while time.monotonic()<end and (node.mode!='AUTO' or node.image is None):
                window.tick();app.processEvents();time.sleep(.05)
            assert node.mode=='AUTO', node.mode
            assert node.image is not None
        finally:
            stop.set();thread.join();window.executor.shutdown(timeout_sec=.2)
            node.destroy_node();source.destroy_node();rclpy.shutdown()
    ''')
    env = dict(os.environ, ROS_DOMAIN_ID='121', QT_QPA_PLATFORM='offscreen')
    result = subprocess.run([sys.executable, '-c', script], env=env, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout+result.stderr
