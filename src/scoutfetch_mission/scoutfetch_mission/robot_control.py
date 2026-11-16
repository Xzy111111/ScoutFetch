"""Desktop keyboard control with acknowledged manual/autonomous handover."""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
from PyQt5 import QtCore, QtGui, QtWidgets


def keyboard_twist(keys, linear=0.15, angular=0.4):
    command = Twist()
    x = int('w' in keys) - int('s' in keys)
    y = int('a' in keys) - int('d' in keys)
    scale = max(1.0, math.hypot(x, y))
    command.linear.x, command.linear.y = linear * x / scale, linear * y / scale
    command.angular.z = angular * (int('q' in keys) - int('e' in keys))
    return command


class ControlNode(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.mode = 'CONNECTING'
        self.state = 'IDLE'
        self.grasp = 'EMPTY'
        self.last_mode = 0.0
        self.notice = '正在连接机器人…'
        self.image = None
        self.pending = None
        self.publisher = self.create_publisher(Twist, '/cmd_vel_teleop', 1)
        self.client = self.create_client(SetBool, '/set_control_mode')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_sub = self.create_subscription(String, '/control_mode', self.on_mode, latched)
        self.state_sub = self.create_subscription(String, '/task_state', self.on_state, 1)
        self.grasp_sub = self.create_subscription(String, '/grasp_status', self.on_grasp, latched)
        self.image_sub = self.create_subscription(Image, '/camera/yolo/image_annotated', self.on_image, 1)

    def on_mode(self, message):
        self.mode, self.last_mode = message.data, time.monotonic()
        if self.notice == '正在连接机器人…':
            self.notice = '机器人已连接，可切换键盘或自主模式'

    def on_state(self, message):
        self.state = message.data

    def on_grasp(self, message):
        self.grasp = message.data

    def on_image(self, message):
        if message.encoding not in ('rgb8', 'bgr8'):
            return
        image = QtGui.QImage(message.data, message.width, message.height, message.step,
                             QtGui.QImage.Format_RGB888)
        self.image = image.rgbSwapped() if message.encoding == 'bgr8' else image.copy()

    def request_mode(self, automatic):
        self.publisher.publish(Twist())
        if self.pending is not None:
            return
        if not self.client.service_is_ready():
            self.notice = '模式服务尚未就绪，请等待仿真启动完成'
            return
        self.notice = '正在启动自主任务…' if automatic else '正在切换到键盘模式…'
        self.pending = self.client.call_async(SetBool.Request(data=automatic))
        self.pending.add_done_callback(self.mode_result)

    def mode_result(self, future):
        self.pending = None
        try:
            response = future.result()
            self.notice = response.message if response.success else '暂未切换：' + response.message
        except Exception as exc:
            self.notice = '切换未完成：' + str(exc)


class ControlWindow(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        # Keep the node registered between Qt ticks. Repeated global spin_once
        # add/remove resets callback iteration; a 1 kHz /clock can then starve
        # the later mode/image subscriptions indefinitely.
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(node)
        self.keys = set()
        self.setWindowTitle('MC ARM · 键盘 / 自主找瓶')
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.resize(720, 700)
        layout = QtWidgets.QVBoxLayout(self)
        self.mode_label = QtWidgets.QLabel()
        self.mode_label.setStyleSheet('font-size: 22px; font-weight: bold; padding: 8px;')
        layout.addWidget(self.mode_label)
        controls = QtWidgets.QHBoxLayout()
        for text, automatic in [('键盘模式 [1]', False), ('自主找瓶 / 携瓶返回 [2]', True)]:
            button = QtWidgets.QPushButton(text)
            button.setMinimumHeight(45)
            button.setFocusPolicy(QtCore.Qt.NoFocus)
            button.clicked.connect(lambda _, a=automatic: self.request_mode(a))
            controls.addWidget(button)
        layout.addLayout(controls)
        stop = QtWidgets.QPushButton('停止并转手动 [空格]')
        stop.setMinimumHeight(40)
        stop.setFocusPolicy(QtCore.Qt.NoFocus)
        stop.clicked.connect(lambda: self.request_mode(False))
        layout.addWidget(stop)
        layout.addWidget(QtWidgets.QLabel(
            'W / S 前进后退    A / D 左右横移    Q / E 左右旋转\n'
            '键盘模式下按住运动、松开停止；点击本窗口后操作，切出窗口自动停止。'))
        self.camera = QtWidgets.QLabel('等待 YOLO 相机画面…')
        self.camera.setAlignment(QtCore.Qt.AlignCenter)
        self.camera.setMinimumSize(640, 360)
        self.camera.setStyleSheet('background: #202830; color: white;')
        layout.addWidget(self.camera, 1)
        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    def request_mode(self, automatic):
        self.keys.clear()
        self.node.request_mode(automatic)
        self.setFocus()

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        text = event.text().lower()
        if text == '1' or event.key() == QtCore.Qt.Key_Space:
            self.request_mode(False)
        elif text == '2':
            self.request_mode(True)
        elif (text in ('w', 'a', 's', 'd', 'q', 'e') and
              self.node.mode == 'MANUAL' and self.node.pending is None):
            self.keys.add(text)

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            self.keys.discard(event.text().lower())

    def tick(self):
        if not rclpy.ok():
            self.timer.stop()
            QtWidgets.QApplication.instance().quit()
            return
        try:
            for _ in range(5):
                self.executor.spin_once(timeout_sec=0.0)
        except ExternalShutdownException:
            self.timer.stop()
            QtWidgets.QApplication.instance().quit()
            return
        fresh = time.monotonic() - self.node.last_mode < 1.0
        if not self.isActiveWindow() or not fresh:
            self.keys.clear()
        if self.node.mode == 'MANUAL':
            command = keyboard_twist(self.keys) if fresh and self.node.pending is None else Twist()
            self.node.publisher.publish(command)
        modes = {'MANUAL': '键盘模式', 'AUTO': '自主模式',
                 'SWITCHING_TO_MANUAL': '正在结束自主动作 → 键盘模式'}
        self.mode_label.setText(modes.get(self.node.mode, '正在连接…') if fresh else '等待机器人连接…')
        states = {'IDLE': '等待操作', 'SEARCHING': '旋转寻找瓶子', 'WAIT_TARGET': '等待识别',
                  'WAIT_BASE_POSE': '计算抓取站位', 'WAIT_NAV_RESULT': '前往瓶子',
                  'WAIT_GRASP_TARGET': '确认抓取位置', 'WAIT_PICK_RESULT': '夹取并收臂',
                  'WAIT_RETURN_NAV_RESULT': '携瓶返回', 'DONE': '任务完成',
                  'FAILED': '任务未完成', 'STOPPING': '取消导航中'}
        states.update({'EXPLORING': '自主探索：前往新的观察位置',
                       'EXPLORATION_STOPPING': '探索停车并确认目标',
                       'VERIFY_CARRY': '检查是否仍然夹住瓶子',
                       'RECOVERY_STOPPING': '检测到掉瓶：停车并取消导航',
                       'RECOVERY_PREPARING': '重新观察掉落的瓶子，准备再次夹取',
                       'NAV_REPLANNING': '站位出现障碍：停车并重新选择站位',
                       'SEARCH_EXHAUSTED': '可达观察区域已搜索完，未找到瓶子'})
        grip = {'EMPTY': '未夹持', 'HELD': '已夹持', 'LOST': '瓶子掉落',
                'VERIFYING': '正在确认夹持', 'SEARCHING_AGAIN': '重新观察瓶子，准备重试',
                'FEEDBACK_STALE': '夹爪反馈中断', 'DRIVER_FAULT': '夹爪故障'}
        grasp = getattr(self.node, 'grasp', 'EMPTY')
        self.status.setText('任务：' + states.get(self.node.state, self.node.state) + '\n夹爪：' +
                            grip.get(grasp, grasp) + '\n' + self.node.notice)
        if self.node.image is not None:
            self.camera.setPixmap(QtGui.QPixmap.fromImage(self.node.image).scaled(
                self.camera.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    def closeEvent(self, event):
        self.timer.stop()
        self.keys.clear()
        if not rclpy.ok():
            super().closeEvent(event)
            return
        self.node.request_mode(False)
        deadline = time.monotonic() + 0.3
        while self.node.pending is not None and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
        self.node.publisher.publish(Twist())
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv[:1])
    node = ControlNode()
    window = ControlWindow(node)
    window.show()
    try:
        return app.exec_()
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        window.executor.remove_node(node)
        window.executor.shutdown(timeout_sec=0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
