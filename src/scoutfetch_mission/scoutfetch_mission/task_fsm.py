#!/usr/bin/env python3
"""task_fsm: shared search -> navigate -> pick -> return orchestration.

只做任务编排, 把已验收的正式 ROS2 接口串起来:
  /target_pose_map -> /best_base_pose -> /navigation_goal -> nav_goal_executor
  -> /navigation_result -> /resolve_grasp_target -> target_pose_resolver
  -> /grasp_target_pose -> /execute_pick -> pick dry-run
  -> 返回 /navigation_goal (saved_start_pose) -> /navigation_result -> DONE

禁止在本节点实现 Nav2 ActionClient / IK / MoveIt / gripper / 规划,
已有模块必须继续复用。搜索阶段独占速度命令, 离开搜索立即停转。
TF 用于记录机器人位姿、将图像拍摄时的相机视线投影为 map 中的搜索方向,
不承担任何 target map->base_link 转换 (仍属 target_pose_resolver)。

安全设计 (显式 ARM):
  /target_pose_map 是 TRANSIENT_LOCAL, 节点启动后可能收到历史 target, 一律忽略。
  用户调用 /start_task (std_srvs/Trigger) 且 TF map->base_link 可用后才进入
  SEARCHING (search_enabled) 或 WAIT_TARGET, 并冻结本任务起点 saved_start_pose。

A6 dry-run 中 DONE 代表软件事件链成功；真实执行仿真还要求持续夹持反馈，
抬升/收臂及返回后验证夹持，掉瓶后停车、重新观察并有限次重试。
搜索有里程计、检测流及总时长 watchdog。
"""

import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String, Header
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
from vision_msgs.msg import Detection2DArray
from scoutfetch_mission.bottle_search import BottleSearch
from scoutfetch_mission.bottle_hint import BottleHint, optical_bearing
from scoutfetch_mission.search_explorer import SearchExplorer
from scoutfetch_mission.grasp_monitor import GraspMonitor
from scoutfetch_interfaces.msg import GripperStatus
from sensor_msgs.msg import CameraInfo

import tf2_ros

from enum import Enum, auto

_QUATERNION_NORM_TOLERANCE = 1e-3
_START_POSE_TF_TIMEOUT_SEC = 1.0


def _yaw_from_orientation(quaternion):
    sin_yaw_cos_pitch = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)


class TaskState(Enum):
    IDLE = auto()
    SEARCHING = auto()
    WAIT_TARGET = auto()
    WAIT_BASE_POSE = auto()
    WAIT_NAV_RESULT = auto()
    WAIT_GRASP_TARGET = auto()
    WAIT_PICK_RESULT = auto()
    WAIT_RETURN_NAV_RESULT = auto()
    DONE = auto()
    FAILED = auto()
    STOPPING = auto()
    EXPLORING = auto()
    EXPLORATION_STOPPING = auto()
    SEARCH_EXHAUSTED = auto()
    VERIFY_CARRY = auto()
    RECOVERY_STOPPING = auto()
    RECOVERY_PREPARING = auto()
    NAV_REPLANNING = auto()


def _transient_local_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _volatile_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class TaskFsm(Node):
    def __init__(self):
        super().__init__("task_fsm")

        self.declare_parameter("target_topic", "/target_pose_map")
        self.declare_parameter("best_pose_topic", "/best_base_pose")
        self.declare_parameter("navigation_goal_topic", "/navigation_goal")
        self.declare_parameter("navigation_result_topic", "/navigation_result")
        self.declare_parameter("resolve_trigger_topic", "/resolve_grasp_target")
        self.declare_parameter("grasp_target_topic", "/grasp_target_pose")
        self.declare_parameter("execute_pick_service", "/execute_pick")
        self.declare_parameter("start_task_service", "/start_task")
        self.declare_parameter("task_state_topic", "/task_state")

        self.target_topic = self.get_parameter("target_topic").value
        self.best_pose_topic = self.get_parameter("best_pose_topic").value
        self.navigation_goal_topic = self.get_parameter("navigation_goal_topic").value
        self.navigation_result_topic = self.get_parameter("navigation_result_topic").value
        self.resolve_trigger_topic = self.get_parameter("resolve_trigger_topic").value
        self.grasp_target_topic = self.get_parameter("grasp_target_topic").value
        self.execute_pick_service = self.get_parameter("execute_pick_service").value
        self.start_task_service = self.get_parameter("start_task_service").value
        self.task_state_topic = self.get_parameter("task_state_topic").value

        # 搜索默认关闭，已有固定目标流程保持兼容。统一 sim launch 显式开启。
        self.search_enabled = self.declare_parameter("search_enabled", False).value
        self.target_max_age = float(self.declare_parameter("target_max_age", 0.5).value)
        self.detector_timeout = float(self.declare_parameter("detector_timeout", 3.0).value)
        self.phase_timeout = float(self.declare_parameter("phase_timeout_sec", 60.0).value)
        self._state_entered_monotonic = time.monotonic()
        self.search = BottleSearch(
            continuous_rotation=self.declare_parameter('search_continuous_rotation', True).value,
            angular_speed=float(self.declare_parameter("search_angular_speed", 0.25).value),
            step_angle=math.radians(float(self.declare_parameter("search_step_deg", 30.0).value)),
            dwell=float(self.declare_parameter("search_dwell_sec", 1.5).value),
            settle=float(self.declare_parameter("search_settle_sec", 0.8).value),
            confirmation_frames=int(self.declare_parameter("search_confirm_frames", 3).value),
            timeout=float(self.declare_parameter("search_timeout_sec", 180.0).value),
            max_turns=int(self.declare_parameter("search_max_turns", 1).value),
        )
        self.exploration_enabled = self.declare_parameter('exploration_enabled', False).value
        self.explorer = SearchExplorer()
        self.observe_target_max_range = float(self.declare_parameter('observe_target_max_range', 2.5).value)
        self._skip_exploration_candidates = False
        self._exploration_map_received = None
        self._exploration_deadline = None
        self._exploration_error = None
        self._exploration_failures = 0
        self._last_exploration_goal_ns = 0
        self._view_stamp_ns = 0
        if self.exploration_enabled:
            self.search.max_turns = 0
            self.exploration_map_sub = self.create_subscription(
                OccupancyGrid, '/global_costmap/costmap', self._exploration_map_callback,
                _transient_local_qos())
        self._detector_received = None
        self._detector_stamp_ns = None
        self.center_target_before_confirm = self.declare_parameter('center_target_before_confirm', False).value
        self._camera_width = None
        self._camera_intrinsics = None
        self.bottle_hint = BottleHint()
        self._centered_detection_frames = {}
        self._pending_search_target = None
        if self.center_target_before_confirm or self.exploration_enabled:
            self.camera_info_sub = self.create_subscription(
                CameraInfo, '/camera/camera/color/camera_info', self._camera_info_callback, 10)
        self._armed_stamp_ns = None
        self._search_confirm_stamp_ns = None
        self.search_velocity_publisher = self.create_publisher(
            Twist, self.declare_parameter("search_cmd_vel_topic", "/cmd_vel_search").value, 1)
        self.odom_sub = self.create_subscription(
            Odometry, "/Odometry", self._search_odom_callback, 10)
        self.detector_sub = self.create_subscription(
            Detection2DArray, "/detections", self._detector_callback, 10)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._search_timer = self.create_timer(
            0.05, self._search_tick, clock=self._steady_clock)

        # ---- 导航到位后的"停稳"等待 (grasp target 解析时机) ----
        # Nav2 报 success 的瞬间底盘可能仍在微动/回稳 (实测到 ~1.4cm 回退),
        # 而 resolver 用"解析时刻"的 map->base_link 把世界系目标转成 base_link。
        # 若立刻解析, 之后底盘再动, 机械臂就会按过时的 base_link 目标去抓, 偏差
        # 可达 1-2cm (足以让两指在瓶身近缘相切滑脱)。故 Nav 成功后先等机器人在
        # map 系停稳 (位移在 settle_epsilon 内持续 post_nav_settle_sec 秒), 再解析,
        # 保证"解析 base_link 目标"与"机械臂执行"之间底盘确实静止。
        self.declare_parameter("post_nav_settle_sec", 0.6)
        self.declare_parameter("settle_epsilon", 0.003)
        self.post_nav_settle_sec = float(self.get_parameter("post_nav_settle_sec").value)
        self.settle_epsilon = float(self.get_parameter("settle_epsilon").value)
        self._settle_timer = None
        self._settle_prev_xy = None
        self._settle_since = None

        # ---- 订阅: 各阶段输入 ----
        # /target_pose_map 发布端 (scoutfetch_vision) 为 TRANSIENT_LOCAL, 必须匹配。
        self.target_pose_map_sub = self.create_subscription(
            PoseStamped, self.target_topic, self.target_pose_map_callback,
            _transient_local_qos())
        # /best_base_pose 发布端 (scoutfetch_reach) 为默认 QoS (reliable/volatile)。
        self.best_base_pose_sub = self.create_subscription(
            PoseStamped, self.best_pose_topic, self.best_base_pose_callback,
            _volatile_qos())
        # /navigation_result 发布端 (nav_goal_executor) 为 VOLATILE。
        self.navigation_result_sub = self.create_subscription(
            Bool, self.navigation_result_topic, self.navigation_result_callback,
            _volatile_qos())
        # /grasp_target_pose 发布端 (target_pose_resolver) 为默认 QoS。
        self.grasp_target_pose_sub = self.create_subscription(
            PoseStamped, self.grasp_target_topic, self.grasp_target_pose_callback,
            _volatile_qos())

        # ---- 发布: 驱动下游 / 观测 ----
        self.navigation_goal_publisher = self.create_publisher(
            PoseStamped, self.navigation_goal_topic, _volatile_qos())
        # 导航成功后才触发 resolver: 发送当前任务锁定的 active_target_pose 副本 (A5c)。
        # /resolve_grasp_target 是任务执行事件, 不是状态: 必须 VOLATILE,
        # 旧任务的 resolve request 不得在 resolver 重启后重新送达 (禁止 TRANSIENT_LOCAL)。
        self.resolve_trigger_publisher = self.create_publisher(
            PoseStamped, self.resolve_trigger_topic, _volatile_qos())
        # 状态观测: 状态变化及心跳发布, VOLATILE 不重放旧任务事件。
        self.task_state_publisher = self.create_publisher(
            String, self.task_state_topic, _volatile_qos())

        # ---- 服务: 显式 ARM / 触发 pick ----
        self.start_task_service_handle = self.create_service(
            Trigger, self.start_task_service, self.start_task_callback)
        self.execute_pick_client = self.create_client(
            Trigger, self.execute_pick_service)
        self.control_mode = "MANUAL"
        self._navigation_pending = False
        self._active_navigation_header = None
        self._carrying_bottle = False
        self.grasp_monitor_enabled = self.declare_parameter('grasp_monitor_enabled', False).value
        self.grasp_monitor = GraspMonitor()
        self._recovery_attempts = 0
        self._recovery_limit = int(self.declare_parameter('regrasp_attempts', 2).value)
        self._recovery_error = None
        self._verify_return = False
        self._target_refinements = 0
        self._nav_replans = 0
        self._best_after_ns = 0
        self._navigation_error = None
        self._navigation_started = None
        self._active_navigation_pose = None
        self._grasp_status = 'EMPTY'
        self.grasp_status_publisher = self.create_publisher(String, '/grasp_status', _transient_local_qos())
        self.gripper_sub = self.create_subscription(
            GripperStatus, '/feedback/gripper_status', self._gripper_callback, 10)
        self.recovery_client = self.create_client(Trigger, '/prepare_grasp_recovery')
        self._startup_auto_cancelled = False
        self.mode_publisher = self.create_publisher(String, "/control_mode", _transient_local_qos())
        self.autonomy_lock_publisher = self.create_publisher(Bool, "/autonomy_lock", 1)
        self.manual_stop_publisher = self.create_publisher(Twist, "/cmd_vel_teleop", 1)
        self.navigation_cancel_publisher = self.create_publisher(Header, "/navigation_cancel", 10)
        self.mode_service = self.create_service(SetBool, "/set_control_mode", self.set_control_mode_callback)
        self.auto_start_service = self.create_service(Trigger, "/auto_start_task", self.auto_start_callback)
        self._mode_timer = self.create_timer(0.1, self._mode_tick, clock=self._steady_clock)

        # ---- 任务身份: 一次任务一个 active_target_stamp ----
        self.active_target_pose = None
        self.active_target_stamp = None  # (sec, nanosec) 元组
        # 最新一帧目标坐标 (仅用于解析时刷新坐标, 不改任务身份)。
        # map 目标和 map->base_link 都由同一个 AMCL 产生; 用同一时刻的一对
        # 才能抵消定位漂移, 用任务开始时的旧 map 坐标配解析时刻的新 TF 会
        # 把漂移直接算进抓取点。
        self.latest_target_pose = None

        # ---- 起点快照: 仅 ARM 时捕获, 一次任务内冻结 (A6) ----
        # TF 只用于记录机器人自身起始位姿, 不承担任何 target 转换。
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.saved_start_pose = None  # PoseStamped, frame=map, stamp=ARM 时刻 node time

        self.current_task_state = TaskState.IDLE
        self._publish_task_state()

        self.get_logger().info(
            "task_fsm ready: state=%s target=%s best=%s nav_goal=%s nav_result=%s "
            "resolve=%s grasp=%s execute_pick=%s start=%s"
            % (self.current_task_state.name, self.target_topic, self.best_pose_topic,
               self.navigation_goal_topic, self.navigation_result_topic,
               self.resolve_trigger_topic, self.grasp_target_topic,
               self.execute_pick_service, self.start_task_service))

    # ==================== 状态转移 ====================

    def transition_to(self, new_state):
        if self.current_task_state == new_state:
            return
        if self.current_task_state == TaskState.SEARCHING:
            self._stop_search()
        self.current_task_state = new_state
        self._state_entered_monotonic = time.monotonic()
        if new_state in (TaskState.FAILED, TaskState.IDLE, TaskState.STOPPING) and self._settle_timer is not None:
            self._settle_timer.cancel()
            self.destroy_timer(self._settle_timer)
            self._settle_timer = None
        self._publish_task_state()
        self.get_logger().info("TASK STATE: %s" % new_state.name)

    def _publish_task_state(self):
        state_message = String()
        state_message.data = self.current_task_state.name
        self.task_state_publisher.publish(state_message)

    # ==================== 显式 ARM ====================

    def _mode_tick(self):
        self.mode_publisher.publish(String(data=self.control_mode))
        self.autonomy_lock_publisher.publish(Bool(data=self.control_mode != "AUTO"))
        # A heartbeat also gives a newly opened control window the current task.
        self._publish_task_state()
        self.grasp_status_publisher.publish(String(data=self._grasp_status))
        if self.control_mode == "SWITCHING_TO_MANUAL":
            self.manual_stop_publisher.publish(Twist())
            if self._navigation_pending and self._active_navigation_header is not None:
                self.navigation_cancel_publisher.publish(self._active_navigation_header)

    def _enter_manual(self):
        self._stop_search()
        self.bottle_hint.clear()
        self.control_mode = "MANUAL"
        self.transition_to(TaskState.IDLE)
        self.active_target_pose = self.active_target_stamp = self.latest_target_pose = None
        self._armed_stamp_ns = None
        self.manual_stop_publisher.publish(Twist())
        self._mode_tick()
        self.get_logger().info("CONTROL MODE: MANUAL")

    def set_control_mode_callback(self, request, response):
        # A deliberate mode choice wins over a still-waiting startup helper.
        self._startup_auto_cancelled = True
        if request.data:
            if self.control_mode == "SWITCHING_TO_MANUAL":
                response.success, response.message = False, "正在结束自主动作，请等待手动模式就绪"
                return response
            if self.control_mode == "AUTO" and self.current_task_state not in (
                    TaskState.IDLE, TaskState.DONE, TaskState.FAILED):
                response.success, response.message = True, "自主任务正在运行"
                return response
            if self._carrying_bottle:
                # Do not try to grasp an already held bottle. Re-entering AUTO
                # after driving manually resumes the saved return destination.
                self.control_mode = "AUTO"
                self._mode_tick()
                self.transition_to(TaskState.WAIT_RETURN_NAV_RESULT)
                self._publish_return_goal()
                response.success, response.message = True, "已夹持瓶子，继续返回本次任务起点"
                return response
            started = self.start_task_callback(Trigger.Request(), Trigger.Response())
            response.success = started.success
            hints = {
                "fresh YOLO detections stream unavailable (including empty frames)": "相机识别流尚未就绪，请稍后再按 2",
                "fresh /Odometry unavailable": "底盘里程计尚未就绪，请稍后再按 2",
                "start pose unavailable": "机器人定位尚未就绪，请稍后再按 2",
                "fresh exploration costmap unavailable": "导航地图尚未就绪，请稍后再按 2",
            }
            response.message = ("自主搜索已启动，返回点已记录" if started.success else
                                hints.get(started.message, started.message))
            return response

        if self.control_mode == "MANUAL":
            response.success, response.message = True, "键盘模式已就绪"
            return response
        self.control_mode = "SWITCHING_TO_MANUAL"
        self._stop_search()
        self.manual_stop_publisher.publish(Twist())
        self.autonomy_lock_publisher.publish(Bool(data=True))
        if self.current_task_state in (TaskState.WAIT_PICK_RESULT, TaskState.RECOVERY_PREPARING):
            response.message = "底盘已停止，夹取/收臂结束后进入键盘模式"
        elif self._navigation_pending:
            self.transition_to(TaskState.STOPPING)
            response.message = "底盘已停止，正在取消导航"
        else:
            self._enter_manual()
            response.message = "键盘模式已就绪"
        self._mode_tick()
        response.success = True
        return response

    def auto_start_callback(self, request, response):
        if self._startup_auto_cancelled:
            response.success, response.message = False, "auto-start superseded by operator"
            return response
        return self.start_task_callback(request, response)

    def start_task_callback(self, request, response):
        if self.control_mode == "SWITCHING_TO_MANUAL" or self._navigation_pending:
            response.success, response.message = False, "previous autonomous action is still stopping"
            return response
        if self._carrying_bottle:
            response.success, response.message = False, "bottle already held; use AUTO mode to return"
            return response
        if self.current_task_state not in (TaskState.IDLE, TaskState.DONE, TaskState.FAILED, TaskState.SEARCH_EXHAUSTED):
            response.success = False
            response.message = "task already running"
            self.get_logger().warn("START REJECTED: %s" % response.message)
            return response
        if self.search_enabled:
            now = time.monotonic()
            if self.exploration_enabled and (self._exploration_map_received is None or
                    now-self._exploration_map_received > 5.0):
                response.success, response.message = False, 'fresh exploration costmap unavailable'
                return response
            if (self._detector_received is None or
                    now - self._detector_received > self.detector_timeout or
                    not self._fresh_stamp(self._detector_stamp_ns)):
                response.success = False
                response.message = "fresh YOLO detections stream unavailable (including empty frames)"
                return response
            if self.search.last_odom is None or now - self.search.last_odom > self.search.odom_timeout:
                response.success = False
                response.message = "fresh /Odometry unavailable"
                return response
        # 每次 ARM 重新捕获当前机器人起点 (map 系), 一次任务内冻结;
        # TF 不可用时拒绝 ARM, 不得进入 WAIT_TARGET, 不得复用旧起点。
        saved_start_pose = self._capture_start_pose()
        if saved_start_pose is None:
            response.success = False
            response.message = "start pose unavailable"
            self.get_logger().warn("TASK ARM REJECTED: start pose unavailable")
            return response
        # 清除上一任务身份与内部临时状态, 不自动复用旧 target。
        self.active_target_pose = None
        self.active_target_stamp = None
        self.latest_target_pose = None
        self._armed_stamp_ns = self.get_clock().now().nanoseconds
        self._view_stamp_ns = self._armed_stamp_ns
        self.explorer.reset()
        self.bottle_hint.clear()
        self._exploration_failures = 0
        self._exploration_error = None
        self._recovery_attempts = 0
        self._nav_replans = 0
        self._best_after_ns = 0
        self.saved_start_pose = saved_start_pose
        self._log_start_pose()
        if self.search_enabled:
            try:
                self.search.start(time.monotonic())
            except ValueError as exc:
                response.success = False
                response.message = str(exc)
                return response
            self._search_confirm_stamp_ns = None
            self.transition_to(TaskState.SEARCHING)
        else:
            self.transition_to(TaskState.WAIT_TARGET)
        response.success = True
        response.message = "task armed"
        self._startup_auto_cancelled = True
        self.control_mode = "AUTO"
        self._mode_tick()
        self.get_logger().info("TASK ARMED")
        return response

    def _capture_start_pose(self):
        try:
            # lookup_transform(target, source) 返回 source 在 target 系中的位姿:
            # map<-base_link 即 base_link 在 map 中的 Pose (T_map_base_link)。
            transform_stamped = self.tf_buffer.lookup_transform(
                "map", "base_link", Time(),
                timeout=Duration(seconds=_START_POSE_TF_TIMEOUT_SEC))
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                "START POSE TF UNAVAILABLE (map->base_link): %s" % exc)
            return None
        if self.search_enabled:
            stamp = transform_stamped.header.stamp
            if not self._fresh_stamp(stamp.sec * 10**9 + stamp.nanosec):
                self.get_logger().warn("START POSE TF is stale")
                return None
        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        # 不能用 transform.header.stamp: 静态/测试 TF 可能 stamp=0,
        # 而 nav_goal_executor 拒绝 stamp==0。用 node time 表示本任务起点快照身份。
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose.position.x = transform_stamped.transform.translation.x
        start_pose.pose.position.y = transform_stamped.transform.translation.y
        start_pose.pose.position.z = transform_stamped.transform.translation.z
        start_pose.pose.orientation.x = transform_stamped.transform.rotation.x
        start_pose.pose.orientation.y = transform_stamped.transform.rotation.y
        start_pose.pose.orientation.z = transform_stamped.transform.rotation.z
        start_pose.pose.orientation.w = transform_stamped.transform.rotation.w
        return start_pose

    def _log_start_pose(self):
        start_pose = self.saved_start_pose
        self.get_logger().info(
            "TASK START POSE SAVED\n"
            "  stamp: %d.%09d\n"
            "  frame: %s\n"
            "  x: %.4f y: %.4f\n"
            "  yaw: %.2f deg"
            % (start_pose.header.stamp.sec, start_pose.header.stamp.nanosec,
               start_pose.header.frame_id,
               start_pose.pose.position.x, start_pose.pose.position.y,
               math.degrees(_yaw_from_orientation(start_pose.pose.orientation))))

    # ==================== WAIT_TARGET ====================

    def _fresh_stamp(self, stamp_ns):
        if stamp_ns is None or stamp_ns <= 0:
            return False
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        return -0.05 <= age <= self.target_max_age

    def _detector_callback(self, msg):
        self._detector_received = time.monotonic()
        self._detector_stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if self.center_target_before_confirm:
            centered = False
            if self._camera_width and msg.detections:
                best = max(msg.detections, key=lambda d: max(
                    (r.hypothesis.score for r in d.results), default=0.0))
                centered = .25*self._camera_width <= best.bbox.center.position.x <= .75*self._camera_width
            self._centered_detection_frames[self._detector_stamp_ns] = centered
            while len(self._centered_detection_frames) > 32:
                self._centered_detection_frames.pop(next(iter(self._centered_detection_frames)))
            pending = self._pending_search_target
            if pending is not None and self._stamp_ns(pending) == self._detector_stamp_ns:
                self._pending_search_target = None
                self.target_pose_map_callback(pending)
        self._remember_detection_hint(msg)

    def _remember_detection_hint(self, msg):
        stamp_ns = self._stamp_ns(msg)
        if (not self.exploration_enabled or self.current_task_state != TaskState.SEARCHING or
                not self._fresh_stamp(stamp_ns) or self._armed_stamp_ns is None or
                stamp_ns < max(self._armed_stamp_ns, self._view_stamp_ns) or
                self._camera_intrinsics is None or not msg.header.frame_id):
            return
        bottles = [(r.hypothesis.score, d) for d in msg.detections for r in d.results
                   if r.hypothesis.class_id in ('bottle', '39') and r.hypothesis.score >= .2]
        if not bottles:
            return  # Empty images do not erase a recent glimpse.
        detection = max(bottles, key=lambda item: item[0])[1]
        u, v = detection.bbox.center.position.x, detection.bbox.center.position.y
        if (not 0 <= u < self._camera_width or not math.isfinite(v) or
                detection.bbox.size_x <= 0 or detection.bbox.size_y <= 0):
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
            q = transform.transform.rotation
            bearing = optical_bearing(u, v, self._camera_intrinsics, (q.x, q.y, q.z, q.w))
        except (tf2_ros.TransformException, ValueError):
            return
        now = time.monotonic()
        ready = self.bottle_hint.observe(bearing, stamp_ns, now)
        if ready and self.search.active and self.search.mode != 'CONFIRM':
            self._search_tick()  # Honor watchdogs before braking for a hint.
            if self.current_task_state != TaskState.SEARCHING or not self.search.active:
                return
            self.search.begin_confirmation(stamp_ns, now)
            self._search_confirm_stamp_ns = self.get_clock().now().nanoseconds
            self.search_velocity_publisher.publish(Twist())
            self.get_logger().info('BOTTLE DIRECTION HINT: braking for fresh confirmation; no grasp target locked')

    def _camera_info_callback(self, msg):
        if msg.width > 0:
            self._camera_width = msg.width
        if msg.k[0] > 0 and msg.k[4] > 0:
            self._camera_intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    @staticmethod
    def _stamp_ns(msg):
        return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec

    def _search_odom_callback(self, msg):
        if msg.header.frame_id != "odom" or msg.child_frame_id != "base_link":
            return
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if not self._fresh_stamp(stamp_ns) or not self._is_valid_pose(msg.pose.pose):
            return
        p = msg.pose.pose.position
        previous_confirm_after = getattr(self.search, "confirm_after", None)
        self.search.feedback(p.x, p.y, _yaw_from_orientation(msg.pose.pose.orientation),
                             time.monotonic())
        if self.search.mode == "CONFIRM" and self.search.confirm_after != previous_confirm_after:
            self._search_confirm_stamp_ns = self.get_clock().now().nanoseconds

    def _stop_search(self):
        self.search_velocity_publisher.publish(Twist())
        self.search.stop()

    def _exploration_map_callback(self, msg):
        if msg.header.frame_id != 'map' or len(msg.data) != msg.info.width*msg.info.height:
            return
        try:
            origin = msg.info.origin
            self.explorer.update(np.asarray(msg.data).reshape(msg.info.height, msg.info.width),
                                 msg.info.resolution,
                                 (origin.position.x, origin.position.y, _yaw_from_orientation(origin.orientation)))
            self._exploration_map_received = time.monotonic()
        except ValueError as exc:
            self.get_logger().warn('EXPLORATION MAP REJECTED: '+str(exc))

    def _exploration_finish(self, reason, exhausted=False):
        self.get_logger().warn('EXPLORATION STOP: '+reason)
        self._enter_manual()
        self.transition_to(TaskState.SEARCH_EXHAUSTED if exhausted else TaskState.FAILED)

    def _explore_next_view(self, mark_scanned=True, target=None, bearing=None):
        self._stop_search()
        current = self._capture_start_pose()  # current observation pose, never overwrite saved_start_pose
        if current is None or self._exploration_map_received is None or time.monotonic()-self._exploration_map_received > 5:
            self._exploration_finish('fresh map or robot pose unavailable')
            return
        p = current.pose.position
        if mark_scanned:
            self.explorer.mark_scanned(p.x, p.y)
        goal = self.explorer.choose(p.x, p.y, target=target, bearing=bearing)
        if goal is None and bearing is not None:
            self.get_logger().info('BOTTLE HINT BLOCKED: continuing connected-area exploration')
            goal = self.explorer.choose(p.x, p.y)
        if goal is None:
            self._exploration_finish('all connected observation points searched; no bottle found', exhausted=True)
            return
        x, y, yaw = goal
        message = PoseStamped()
        message.header.frame_id = 'map'
        stamp = max(self.get_clock().now().nanoseconds, self._last_exploration_goal_ns+1)
        self._last_exploration_goal_ns = stamp
        message.header.stamp = Time(nanoseconds=stamp).to_msg()
        message.pose.position.x, message.pose.position.y = x, y
        message.pose.orientation.z, message.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self._exploration_error = None
        self._skip_exploration_candidates = target is not None or bearing is not None
        self._exploration_deadline = time.monotonic()+90
        self.transition_to(TaskState.EXPLORING)
        self._send_navigation_goal(message)
        self.get_logger().info('EXPLORATION VIEW %d: (%.2f, %.2f), preserving original return pose' %
                               (self.explorer.goal_count, x, y))

    def _restart_scan_here(self):
        self._stop_search()
        self.bottle_hint.clear()
        self._skip_exploration_candidates = False
        try:
            self.search.start(time.monotonic())
        except ValueError as exc:
            self._exploration_finish(str(exc))
            return
        self._view_stamp_ns = self.get_clock().now().nanoseconds
        self._search_confirm_stamp_ns = None
        self.latest_target_pose = None
        self.transition_to(TaskState.SEARCHING)
        self.get_logger().info('EXPLORATION: new viewpoint reached, scanning again')

    def _stop_exploration_navigation(self, error=None):
        self._exploration_error = error
        self.search_velocity_publisher.publish(Twist())
        self.transition_to(TaskState.EXPLORATION_STOPPING)
        if self._active_navigation_header is not None:
            self.navigation_cancel_publisher.publish(self._active_navigation_header)

    def _search_tick(self):
        if self.grasp_monitor_enabled:
            self._grasp_tick()
        if self.current_task_state in (TaskState.VERIFY_CARRY, TaskState.RECOVERY_PREPARING):
            self.search_velocity_publisher.publish(Twist())
            return
        if self.current_task_state == TaskState.RECOVERY_STOPPING:
            self.search_velocity_publisher.publish(Twist())
            if self._navigation_pending and self._active_navigation_header is not None:
                self.navigation_cancel_publisher.publish(self._active_navigation_header)
            return
        if self.current_task_state == TaskState.NAV_REPLANNING:
            self.search_velocity_publisher.publish(Twist())
            if self._navigation_pending and self._active_navigation_header is not None:
                self.navigation_cancel_publisher.publish(self._active_navigation_header)
            return
        if self._navigation_pending and self.current_task_state in (
                TaskState.WAIT_NAV_RESULT, TaskState.WAIT_RETURN_NAV_RESULT):
            if time.monotonic()-self._navigation_started > 180.0:
                self._cancel_for_replan('navigation exceeded 180 seconds', fatal=True)
                return
            if (self.exploration_enabled and self.current_task_state == TaskState.WAIT_NAV_RESULT
                    and self._exploration_map_received is not None
                    and time.monotonic()-self._exploration_map_received < 5.0
                    and not self._goal_free(self._active_navigation_pose)):
                self._cancel_for_replan('new obstacle occupies the grasp station')
                return
        if self.current_task_state == TaskState.EXPLORATION_STOPPING:
            self.search_velocity_publisher.publish(Twist())
            if self._navigation_pending and self._active_navigation_header is not None:
                self.navigation_cancel_publisher.publish(self._active_navigation_header)
            return
        if self.current_task_state == TaskState.EXPLORING:
            now = time.monotonic()
            if self._detector_received is None or now-self._detector_received > self.detector_timeout:
                self._stop_exploration_navigation('YOLO stream stopped during exploration')
            elif self.search.last_odom is None or now-self.search.last_odom > self.search.odom_timeout:
                self._stop_exploration_navigation('odometry stopped during exploration')
            elif self._exploration_map_received is None or now-self._exploration_map_received > 5.0:
                self._stop_exploration_navigation('costmap stopped during exploration')
            elif now > self._exploration_deadline:
                self._stop_exploration_navigation('exploration navigation timeout')
            return
        # These stages command no motion; fail explicitly if no feasible base
        # pose, no fresh arrival observation, or no resolver response arrives.
        waiting = self.current_task_state in (TaskState.WAIT_BASE_POSE, TaskState.WAIT_GRASP_TARGET)
        settling = self.current_task_state == TaskState.WAIT_NAV_RESULT and self._settle_timer is not None
        if (waiting or settling) and time.monotonic() - self._state_entered_monotonic > self.phase_timeout:
            self.get_logger().error("TASK FAILED: phase timeout in %s" % self.current_task_state.name)
            self.transition_to(TaskState.FAILED)
            return
        if self.current_task_state != TaskState.SEARCHING:
            return
        now = time.monotonic()
        if (self._detector_received is None or
                now - self._detector_received > self.detector_timeout):
            self.search.fail("YOLO detections stream stopped")
        previous_turns = self.search.completed_turns
        confirmation_expired = self.search.mode == 'CONFIRM' and now >= self.search.until
        speed = self.search.tick(now)
        if (self.exploration_enabled and not self.search.error and
                (confirmation_expired or self.search.completed_turns > previous_turns)):
            bearing = self.bottle_hint.take(now)
            if bearing is not None:
                self.get_logger().info('FOLLOW BOTTLE HINT: safe short observation move, then detect again')
                self._explore_next_view(mark_scanned=not confirmation_expired, bearing=bearing)
                return
        if self.search.completed_turns > previous_turns and not self.search.error:
            if self.exploration_enabled:
                self.get_logger().info('NO TARGET AFTER FULL SCAN: moving to a new observation point')
                self._explore_next_view()
                return
            self.get_logger().info("NO TARGET: continuing search, turn %d" % (self.search.completed_turns + 1))
        command = Twist()
        command.angular.z = speed
        self.search_velocity_publisher.publish(command)
        if self.search.error:
            self.get_logger().error("SEARCH FAILED: %s" % self.search.error)
            self.transition_to(TaskState.FAILED)

    def target_pose_map_callback(self, msg):
        # 合法目标始终缓存为"最新坐标"; 是否作为任务身份由状态决定。
        valid = self._validate_target(msg)
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if self.search_enabled and (not self._fresh_stamp(stamp_ns) or
                self._armed_stamp_ns is None or stamp_ns < self._armed_stamp_ns):
            return
        if (self.exploration_enabled and self.current_task_state == TaskState.SEARCHING and
                stamp_ns < self._view_stamp_ns):
            return
        if valid:
            self.latest_target_pose = msg
        if self.exploration_enabled and valid and self.current_task_state == TaskState.EXPLORING:
            if self._skip_exploration_candidates:
                return
            self.get_logger().info('BOTTLE CANDIDATE WHILE EXPLORING: cancel navigation, then confirm fresh stationary frames')
            self._stop_exploration_navigation()
            return
        if self.current_task_state == TaskState.EXPLORATION_STOPPING:
            return
        if self.current_task_state == TaskState.SEARCHING:
            if not valid:
                return
            if self.center_target_before_confirm:
                centered = self._centered_detection_frames.get(stamp_ns)
                if centered is None:
                    # DDS can deliver localization before this frame's 2D
                    # detections. Replay only once the matching frame arrives.
                    self._pending_search_target = msg
                    return
                if not centered:
                    return
            now = time.monotonic()
            # Enforce watchdogs before accepting a target between timer ticks.
            self._search_tick()
            if self.current_task_state != TaskState.SEARCHING:
                return
            if self.search.mode != "CONFIRM":
                # Stamp cutoff uses ROS time at stop. Do not add a wall-time
                # settling duration to /clock: slow Gazebo would never confirm.
                self._search_confirm_stamp_ns = self.get_clock().now().nanoseconds
            elif stamp_ns < self._search_confirm_stamp_ns:
                return
            p = msg.pose.position
            found = self.search.observe((p.x, p.y, p.z), stamp_ns, now)
            self.search_velocity_publisher.publish(Twist())
            if not found:
                return
            self.get_logger().info("BOTTLE CONFIRMED: stopped base, fresh consistent observations")
        elif self.current_task_state != TaskState.WAIT_TARGET:
            if self.current_task_state == TaskState.IDLE:
                self.get_logger().warn("TARGET IGNORED: task not armed")
            else:
                self.get_logger().warn(
                    "TARGET IGNORED: state=%s" % self.current_task_state.name)
            return
        if not valid:
            return
        if self.exploration_enabled:
            current = self._capture_start_pose()
            if current is not None and math.hypot(
                    msg.pose.position.x-current.pose.position.x,
                    msg.pose.position.y-current.pose.position.y) > self.observe_target_max_range:
                self.get_logger().info('DISTANT BOTTLE: navigating to a safe closer view before grasp localization')
                self._explore_next_view(target=(msg.pose.position.x, msg.pose.position.y))
                return
        self.active_target_pose = msg
        self._target_refinements = 0
        self.active_target_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.get_logger().info(
            "TASK TARGET ACCEPTED\n"
            "  stamp: %d.%09d\n"
            "  x: %.4f y: %.4f z: %.4f"
            % (msg.header.stamp.sec, msg.header.stamp.nanosec,
               msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        self.transition_to(TaskState.WAIT_BASE_POSE)

    def _validate_target(self, msg):
        if msg.header.frame_id != "map":
            self.get_logger().warn("TARGET REJECTED: frame=%s (expected map)" % msg.header.frame_id)
            return False
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            self.get_logger().warn("TARGET REJECTED: empty stamp")
            return False
        position = msg.pose.position
        for component in (position.x, position.y, position.z):
            if not math.isfinite(component):
                self.get_logger().warn("TARGET REJECTED: non-finite position")
                return False
        if not self._is_valid_quaternion(msg.pose.orientation):
            self.get_logger().warn("TARGET REJECTED: invalid quaternion")
            return False
        return True

    # ==================== WAIT_BASE_POSE ====================

    def _is_acceptable_best_stamp(self, best_stamp):
        """接受"同一或更新"目标的 best pose, 而不是要求 stamp 精确相等。

        scoutfetch_reach 对 /target_pose_map 用 KeepLast(1), 只处理最新目标; 但它的
        单次候选评估约 2.4 s, 所以它发布的 best 永远比 10 Hz 的目标流滞后约一个周期,
        精确相等在实践中永远对不上, FSM 会无限期卡在 WAIT_BASE_POSE。
        在 WAIT_BASE_POSE 期间底盘不动 (导航由本 FSM 派发, 尚未发出), 所以任何不早于
        任务目标的 best 都基于等价观测; 早于任务目标的 best 来自 ARM 前的旧目标, 仍拒绝。
        """
        if self.active_target_stamp is None:
            return False
        return best_stamp >= self.active_target_stamp

    def best_base_pose_callback(self, msg):
        if self.current_task_state != TaskState.WAIT_BASE_POSE:
            self.get_logger().warn(
                "BEST BASE POSE IGNORED: state=%s" % self.current_task_state.name)
            return
        best_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if best_stamp[0]*10**9+best_stamp[1] < self._best_after_ns:
            return
        if self.exploration_enabled and self.explorer.grid is not None and not self._goal_free(msg):
            self.get_logger().warn('BEST BASE POSE REJECTED: latest costmap blocks the footprint')
            return
        if not self._is_acceptable_best_stamp(best_stamp):
            self.get_logger().warn(
                "BEST BASE POSE IGNORED: stamp %d.%09d older than task target %d.%09d"
                % (best_stamp[0], best_stamp[1],
                   self.active_target_stamp[0], self.active_target_stamp[1]))
            return
        # 原样转发 matching best, 不改 x/y/yaw/frame, 不做 TF。
        # 先进入 WAIT_NAV_RESULT 再发布, 避免快速 fake server 在状态切换前返回。
        self.transition_to(TaskState.WAIT_NAV_RESULT)
        self._send_navigation_goal(msg)
        self.get_logger().info(
            "NAV GOAL PUBLISHED: stamp=%d.%09d frame=%s x=%.4f y=%.4f"
            % (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id,
               msg.pose.position.x, msg.pose.position.y))

    # ==================== 导航结果 (去站位 / 返回) ====================

    def _send_navigation_goal(self, msg):
        self._active_navigation_pose = msg
        self._navigation_started = time.monotonic()
        self._navigation_pending = True
        self._active_navigation_header = msg.header
        self.navigation_goal_publisher.publish(msg)

    def navigation_result_callback(self, msg):
        self._navigation_pending = False
        self._active_navigation_header = None
        if self.control_mode == "SWITCHING_TO_MANUAL":
            if self.current_task_state == TaskState.STOPPING:
                self._enter_manual()
            return
        if self.current_task_state == TaskState.RECOVERY_STOPPING:
            self._prepare_recovery()
            return
        if self.current_task_state == TaskState.NAV_REPLANNING:
            if self._navigation_error:
                self._enter_manual()
                self.transition_to(TaskState.FAILED)
            else:
                self._best_after_ns = self.get_clock().now().nanoseconds
                # Navigation may have turned the wrist camera away from the
                # bottle. Reacquire stationary observations before requesting
                # another station, rather than waiting on an invisible target.
                self.active_target_pose = self.active_target_stamp = self.latest_target_pose = None
                self._restart_scan_here()
            return
        if self.current_task_state == TaskState.EXPLORATION_STOPPING:
            if self._exploration_error:
                self._exploration_finish(self._exploration_error)
            else:
                self._restart_scan_here()
            return
        if self.current_task_state == TaskState.EXPLORING:
            if msg.data:
                self._exploration_failures = 0
                self._restart_scan_here()
            else:
                self.explorer.reject_last()
                self._exploration_failures += 1
                if self._exploration_failures >= 3:
                    self._exploration_finish('three observation points could not be reached')
                else:
                    self._explore_next_view(mark_scanned=False)
            return
        if self.current_task_state == TaskState.WAIT_NAV_RESULT:
            if not msg.data:
                self.get_logger().warn("TASK FAILED: navigation failed")
                self.transition_to(TaskState.FAILED)
                return
            # 去抓取站位导航成功后才触发 resolver, 不自行计算 TF。
            # resolve_request 必须是当前任务 active_target_pose 的原始副本,
            # 不得使用最新 /target_pose_map 或任何缓存 (任务身份以 active_target_stamp 为准)。
            # 但 Nav2 success 时底盘未必已停稳: 先等待停稳再解析, 否则 base_link 目标
            # 会基于"到位瞬间"而非"最终静止"的底盘位姿, 导致机械臂抓偏 1-2cm。
            self.get_logger().info("NAV SUCCEEDED -> waiting for base to settle before resolve")
            self._begin_settle_then_resolve()
            return
        if self.current_task_state == TaskState.WAIT_RETURN_NAV_RESULT:
            if not msg.data:
                self.get_logger().warn("TASK FAILED: return navigation failed")
                self.transition_to(TaskState.FAILED)
                return
            if self.grasp_monitor_enabled:
                self._verify_return = True
                self.transition_to(TaskState.VERIFY_CARRY)
            else:
                self.get_logger().info("TASK DONE: returned to start")
                self.transition_to(TaskState.DONE)
            return
        self.get_logger().warn(
            "NAV RESULT IGNORED: state=%s" % self.current_task_state.name)

    def _begin_settle_then_resolve(self):
        """启动停稳检测计时器; 底盘在 map 系位移稳定后解析抓取目标。

        保持状态为 WAIT_NAV_RESULT 直到真正发布 resolve (nav_goal_executor 每个
        goal 只发一条结果, 不会重复触发本回调)。
        """
        self._settle_prev_xy = None
        self._settle_since = None
        self._state_entered_monotonic = time.monotonic()
        if self._settle_timer is None:
            self._settle_timer = self.create_timer(0.1, self._settle_tick)

    def _settle_tick(self):
        if self.current_task_state != TaskState.WAIT_NAV_RESULT or self.control_mode != "AUTO":
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", Time())
        except Exception:  # TF 暂不可用 (AMCL 抖动), 下一拍再试
            return
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        now = self.get_clock().now()
        if self._settle_prev_xy is not None:
            moved = (abs(x - self._settle_prev_xy[0]) > self.settle_epsilon or
                     abs(y - self._settle_prev_xy[1]) > self.settle_epsilon)
            if moved:
                self._settle_since = None
            elif self._settle_since is None:
                self._settle_since = now
            elif (now - self._settle_since) >= Duration(seconds=self.post_nav_settle_sec):
                if self.search_enabled:
                    latest = self.latest_target_pose
                    if latest is None:
                        return
                    stamp = latest.header.stamp
                    if not self._fresh_stamp(stamp.sec * 10**9 + stamp.nanosec):
                        return
                    a, b = latest.pose.position, self.active_target_pose.pose.position
                    if math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) > 0.10:
                        return
                    if (self.grasp_monitor_enabled and a.z < .05 and
                            math.hypot(a.x-b.x, a.y-b.y) > .025):
                        # A very close downward view can localize the cap,
                        # while the arrival view localizes the bottle body.
                        # Recompute the station using this fresh observation
                        # instead of refreshing an old station's grasp point.
                        self._target_refinements += 1
                        if self._target_refinements > 3:
                            self.get_logger().error('FLOOR TARGET DID NOT STABILIZE: stopping')
                            self._enter_manual()
                            self.transition_to(TaskState.FAILED)
                            return
                        if self._settle_timer is not None:
                            self._settle_timer.cancel()
                            self.destroy_timer(self._settle_timer)
                            self._settle_timer = None
                        self.active_target_pose = latest
                        self.active_target_stamp = (latest.header.stamp.sec, latest.header.stamp.nanosec)
                        self._best_after_ns = self.get_clock().now().nanoseconds
                        self.get_logger().warn('FLOOR TARGET REFINED AFTER ARRIVAL: recomputing grasp station')
                        self.transition_to(TaskState.WAIT_BASE_POSE)
                        return
                # 停稳: 到此为止底盘静止, 现在解析能反映最终位姿。
                if self._settle_timer is not None:
                    self._settle_timer.cancel()
                    self.destroy_timer(self._settle_timer)
                    self._settle_timer = None
                self.get_logger().info(
                    "BASE SETTLED at map (%.4f, %.4f) -> triggering resolve" % (x, y))
                resolve_request = self._build_resolve_request()
                # 先进入 WAIT_GRASP_TARGET 再发布, 避免 resolver 快速回复在状态切换前返回。
                self.transition_to(TaskState.WAIT_GRASP_TARGET)
                self.resolve_trigger_publisher.publish(resolve_request)
                return
        self._settle_prev_xy = (x, y)

    def _build_resolve_request(self):
        active_target = self.active_target_pose
        target = active_target
        latest = self.latest_target_pose
        if latest is not None:
            dx = latest.pose.position.x - active_target.pose.position.x
            dy = latest.pose.position.y - active_target.pose.position.y
            dz = latest.pose.position.z - active_target.pose.position.z
            drift = math.sqrt(dx * dx + dy * dy + dz * dz)
            # 身份门限: 最新检测必须仍是同一任务目标附近; 超过门限说明看到的
            # 是别的东西, 退回任务锁定的坐标 (fail-closed)。
            if drift <= 0.10:
                target = latest
                self.get_logger().info(
                    "RESOLVE COORDS REFRESHED: latest target within %.3f m "
                    "of task target (stamp %d.%09d)"
                    % (drift, latest.header.stamp.sec,
                       latest.header.stamp.nanosec))
            else:
                self.get_logger().warn(
                    "RESOLVE USING LOCKED TARGET: latest target %.3f m away "
                    "(identity mismatch)" % drift)
        resolve_request = PoseStamped()
        resolve_request.header.frame_id = target.header.frame_id
        resolve_request.header.stamp = target.header.stamp
        resolve_request.pose.position.x = target.pose.position.x
        resolve_request.pose.position.y = target.pose.position.y
        resolve_request.pose.position.z = target.pose.position.z
        resolve_request.pose.orientation.x = target.pose.orientation.x
        resolve_request.pose.orientation.y = target.pose.orientation.y
        resolve_request.pose.orientation.z = target.pose.orientation.z
        resolve_request.pose.orientation.w = target.pose.orientation.w
        return resolve_request

    # ==================== WAIT_GRASP_TARGET ====================

    def grasp_target_pose_callback(self, msg):
        if self.current_task_state != TaskState.WAIT_GRASP_TARGET:
            self.get_logger().warn(
                "GRASP TARGET IGNORED: state=%s" % self.current_task_state.name)
            return
        if msg.header.frame_id != "base_link":
            self.get_logger().warn(
                "GRASP TARGET REJECTED: frame=%s (expected base_link)" % msg.header.frame_id)
            return
        if not self._is_valid_pose(msg.pose):
            self.get_logger().warn("GRASP TARGET REJECTED: invalid pose")
            return
        # 先进入 WAIT_PICK_RESULT 再发起调用, 期间新的 grasp pose 不再接受。
        if self.grasp_monitor_enabled:
            self._grasp_status = 'VERIFYING'
        self.transition_to(TaskState.WAIT_PICK_RESULT)
        self._call_execute_pick()

    def _is_valid_pose(self, pose):
        position = pose.position
        for component in (position.x, position.y, position.z):
            if not math.isfinite(component):
                return False
        return self._is_valid_quaternion(pose.orientation)

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

    # ==================== WAIT_PICK_RESULT ====================

    def _call_execute_pick(self):
        if not self.execute_pick_client.service_is_ready():
            self.get_logger().warn("EXECUTE PICK REJECTED: service not available")
            self.transition_to(TaskState.FAILED)
            return
        self.get_logger().info("EXECUTE PICK REQUEST SENT")
        # 预检通过后 call_async 内部仍可能因服务瞬间离线抛 ServiceNotReadyException,
        # 必须兜底: 否则异常从回调逃逸被 executor 吞掉, 状态卡死在 WAIT_PICK_RESULT。
        try:
            future = self.execute_pick_client.call_async(Trigger.Request())
        except Exception as exc:
            self.get_logger().error("TASK FAILED: pick request exception: %s" % exc)
            self.transition_to(TaskState.FAILED)
            return
        future.add_done_callback(self.execute_pick_result_callback)

    def execute_pick_result_callback(self, future):
        if self.current_task_state != TaskState.WAIT_PICK_RESULT:
            self.get_logger().warn(
                "PICK RESULT IGNORED: state=%s" % self.current_task_state.name)
            return
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error("TASK FAILED: pick service exception: %s" % exc)
            if self.control_mode == "SWITCHING_TO_MANUAL":
                self._enter_manual()
            else:
                self.transition_to(TaskState.FAILED)
            return
        if self.control_mode == "SWITCHING_TO_MANUAL":
            self._carrying_bottle = response.success and (not self.grasp_monitor_enabled or
                self.grasp_monitor.state(time.monotonic()) == 'HELD')
            self._enter_manual()
            return
        if response.success:
            if self.grasp_monitor_enabled:
                self._verify_return = False
                self._grasp_status = 'VERIFYING'
                self.transition_to(TaskState.VERIFY_CARRY)
                return
            self._carrying_bottle = True
            # A6: dry-run pick 成功不再直接 DONE, 改为发布返回起点导航目标。
            self.get_logger().info("PICK SUCCEEDED: returning to start")
            # 先进入 WAIT_RETURN_NAV_RESULT 再发布返回 goal。
            self.transition_to(TaskState.WAIT_RETURN_NAV_RESULT)
            self._publish_return_goal()
        else:
            if self.grasp_monitor_enabled:
                self._begin_recovery('pick failed: '+response.message)
                return
            self.get_logger().warn(
                "TASK FAILED: pick failed, reason: %s" % response.message)
            self.transition_to(TaskState.FAILED)

    def _gripper_callback(self, msg):
        if not self.grasp_monitor_enabled:
            return
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if not -.05 <= age <= self.grasp_monitor.timeout:
            return
        healthy = (msg.driver_enable_status and not any((msg.driver_error_status,
            msg.voltage_too_low, msg.motor_overheating, msg.driver_overcurrent,
            msg.driver_overheating, msg.sensor_status)))
        self.grasp_monitor.update(msg.width, healthy, stamp, time.monotonic())

    def _goal_free(self, goal):
        p = goal.pose.position
        return self.explorer.pose_free(p.x, p.y, _yaw_from_orientation(goal.pose.orientation))

    def _cancel_for_replan(self, reason, fatal=False):
        self._nav_replans += 1
        self._navigation_error = reason if fatal or self._nav_replans > 3 else None
        self.get_logger().warn('NAVIGATION STOP: '+reason)
        self._stop_search()
        self.transition_to(TaskState.NAV_REPLANNING)
        if self._active_navigation_header is not None:
            self.navigation_cancel_publisher.publish(self._active_navigation_header)

    def _grasp_tick(self):
        now = time.monotonic()
        feedback = self.grasp_monitor.state(now)
        if self._carrying_bottle:
            self._grasp_status = feedback
            if feedback in ('LOST', 'FEEDBACK_STALE', 'DRIVER_FAULT'):
                self._begin_recovery('carry feedback: '+feedback,
                                     retry=feedback == 'LOST')
                return
        if self.current_task_state == TaskState.VERIFY_CARRY:
            self._grasp_status = feedback
            # Require feedback sampled after lift/stow or arrival, not cached
            # width from the instant of closing the fingers.
            if (feedback == 'HELD' and self.grasp_monitor.received > self._state_entered_monotonic
                    and now-self._state_entered_monotonic >= .4):
                self._carrying_bottle = True
                if self._verify_return:
                    self.get_logger().info('TASK DONE: returned to start, bottle hold verified')
                    self.transition_to(TaskState.DONE)
                else:
                    self.get_logger().info('HOLD VERIFIED after lift/stow: returning to start')
                    self.transition_to(TaskState.WAIT_RETURN_NAV_RESULT)
                    self._publish_return_goal()
            elif now-self._state_entered_monotonic > 2.0:
                self._begin_recovery('post-pick/arrival feedback: '+feedback,
                                     retry=feedback == 'LOST')
        if (self.current_task_state == TaskState.RECOVERY_PREPARING and
                now-self._state_entered_monotonic > 60.0):
            # Keep the base locked until the arm service actually ends.
            self.control_mode = 'SWITCHING_TO_MANUAL'
            self._mode_tick()

    def _begin_recovery(self, reason, retry=True):
        self.get_logger().error('GRASP LOST / UNVERIFIED: '+reason)
        self._carrying_bottle = False
        self._grasp_status = ('EMPTY' if reason.startswith('pick failed:') else 'LOST') if retry else self.grasp_monitor.state(time.monotonic())
        self._stop_search()
        if self.control_mode != 'AUTO':
            return
        self._recovery_attempts += 1
        self._recovery_error = None if retry and self._recovery_attempts <= self._recovery_limit else reason
        self.transition_to(TaskState.RECOVERY_STOPPING)
        if self._navigation_pending and self._active_navigation_header is not None:
            self.navigation_cancel_publisher.publish(self._active_navigation_header)
        else:
            self._prepare_recovery()

    def _prepare_recovery(self):
        if self._recovery_error or not self.recovery_client.service_is_ready():
            self.get_logger().error('RECOVERY STOPPED: '+(self._recovery_error or 'arm recovery service unavailable'))
            self._enter_manual()
            self.transition_to(TaskState.FAILED)
            return
        self.transition_to(TaskState.RECOVERY_PREPARING)
        self.get_logger().warn('REGRASP %d/%d: stop, reset wrist view, then observe again; original home preserved' %
                               (self._recovery_attempts, self._recovery_limit))
        try:
            self.recovery_client.call_async(Trigger.Request()).add_done_callback(self._recovery_ready)
        except Exception as exc:
            self._recovery_error = str(exc)
            self._enter_manual()
            self.transition_to(TaskState.FAILED)

    def _recovery_ready(self, future):
        if self.current_task_state != TaskState.RECOVERY_PREPARING:
            return
        if self.control_mode == 'SWITCHING_TO_MANUAL':
            self._enter_manual()
            return
        try:
            result = future.result()
            if not result.success:
                raise RuntimeError(result.message)
        except Exception as exc:
            self.get_logger().error('RECOVERY PREPARATION FAILED: '+str(exc))
            self._enter_manual()
            self.transition_to(TaskState.FAILED)
            return
        self.active_target_pose = self.active_target_stamp = self.latest_target_pose = None
        self._armed_stamp_ns = self.get_clock().now().nanoseconds
        self.explorer.reset()
        self._exploration_failures = 0
        self._nav_replans = 0
        self._best_after_ns = self._armed_stamp_ns
        self._grasp_status = 'SEARCHING_AGAIN'
        self._restart_scan_here()

    def _publish_return_goal(self):
        if self.saved_start_pose is None:
            self.get_logger().error("TASK FAILED: no saved start pose")
            self.transition_to(TaskState.FAILED)
            return
        start_pose = self.saved_start_pose
        # 构造一份新副本交给下游, 不把可被后续修改的成员对象直接传出。
        return_navigation_goal = PoseStamped()
        return_navigation_goal.header.frame_id = start_pose.header.frame_id
        # Each resumed return is a new navigation event, even though its
        # destination stays the original frozen start pose.
        return_navigation_goal.header.stamp = self.get_clock().now().to_msg()
        return_navigation_goal.pose.position.x = start_pose.pose.position.x
        return_navigation_goal.pose.position.y = start_pose.pose.position.y
        return_navigation_goal.pose.position.z = start_pose.pose.position.z
        return_navigation_goal.pose.orientation.x = start_pose.pose.orientation.x
        return_navigation_goal.pose.orientation.y = start_pose.pose.orientation.y
        return_navigation_goal.pose.orientation.z = start_pose.pose.orientation.z
        return_navigation_goal.pose.orientation.w = start_pose.pose.orientation.w
        self._send_navigation_goal(return_navigation_goal)
        self.get_logger().info(
            "RETURN NAV GOAL PUBLISHED\n"
            "  x: %.4f y: %.4f\n"
            "  yaw: %.2f deg"
            % (return_navigation_goal.pose.position.x,
               return_navigation_goal.pose.position.y,
               math.degrees(_yaw_from_orientation(return_navigation_goal.pose.orientation))))


def main():
    rclpy.init()
    node = TaskFsm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._stop_search()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
