"""Optional sim-only entry point: wait for readiness and arm one task."""

import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_srvs.srv import Trigger


class StartSimTask(Node):
    def __init__(self):
        super().__init__('start_sim_task')
        self.declare_parameter('startup_timeout_sec', 120.0)
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.navigator = self.create_client(GetState, '/bt_navigator/get_state')
        self.start = self.create_client(Trigger, '/auto_start_task')
        self.required = {'joint_state_broadcaster', 'base_controller', 'arm_controller',
                         'gripper_effort_controller', 'gripper_finger_controller'}
        self.pending = {}
        self.started_at = time.monotonic()
        self.last_log = 0.0
        self.done = False
        self.success = False
        self.timer = self.create_timer(0.5, self.check,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def check(self):
        now = time.monotonic()
        if now - self.started_at > self.get_parameter('startup_timeout_sec').value:
            self.get_logger().error('自动启动超时。请检查控制器、Nav2、相机和 AMCL 日志；任务未自动重试。')
            self.done = True
            return
        if now - self.last_log > 5.0:
            self.get_logger().info('等待控制器、Nav2、视觉和定位就绪，然后自动开始任务 …')
            self.last_log = now
        if 'start' in self.pending:
            f = self.pending['start']
            if not f.done():
                return
            response = f.result()
            if response.message == 'auto-start superseded by operator':
                self.get_logger().info('已按操作窗口的模式选择，取消启动时的自动任务。')
                self.success = self.done = True
            elif response.success:
                self.get_logger().info('任务已自动启动：搜索 → 识别 → 夹取 → 返回。')
                self.success = self.done = True
            else:
                self.get_logger().info('继续等待任务就绪：' + response.message)
            self.pending.clear()
            return
        for key, client, request in (
                ('controllers', self.controllers, ListControllers.Request()),
                ('navigator', self.navigator, GetState.Request())):
            if key not in self.pending and client.service_is_ready():
                self.pending[key] = client.call_async(request)
        if not all(key in self.pending and self.pending[key].done()
                   for key in ('controllers', 'navigator')):
            return
        active = {c.name for c in self.pending['controllers'].result().controller if c.state == 'active'}
        nav_active = self.pending['navigator'].result().current_state.id == 3
        self.pending.clear()
        if self.required <= active and nav_active and self.start.service_is_ready():
            # The task service itself also checks fresh odometry, YOLO frames
            # (including empty frames) and map->base_link before accepting.
            self.pending['start'] = self.start.call_async(Trigger.Request())


def main(args=None):
    rclpy.init(args=args)
    node = StartSimTask()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        success = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1
