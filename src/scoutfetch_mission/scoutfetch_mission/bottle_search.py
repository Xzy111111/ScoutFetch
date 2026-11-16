"""Feedback-driven, bounded in-place search, independent of ROS transport.

All deadlines use monotonic seconds. Camera stamps are checked by the caller
against the ROS clock. This keeps a paused /clock from disabling the watchdog.
"""

import math


class BottleSearch:
    def __init__(self, angular_speed=0.25, step_angle=math.pi / 6,
                 dwell=1.5, settle=0.8, confirmation_frames=3,
                 confirmation_radius=0.08, confirmation_timeout=4.0,
                 timeout=180.0, odom_timeout=0.75, max_drift=0.10, max_turns=1,
                 continuous_rotation=False):
        positive = (angular_speed, step_angle, dwell, settle, confirmation_radius,
                    confirmation_timeout, odom_timeout, max_drift)
        if any(not math.isfinite(v) or v <= 0 for v in positive):
            raise ValueError("search parameters must be finite and positive")
        if angular_speed > 0.5 or step_angle > math.pi / 2:
            raise ValueError("search speed <= 0.5 rad/s and step <= pi/2 required")
        if confirmation_frames < 2 or confirmation_timeout <= settle:
            raise ValueError("need >= 2 confirmation frames and timeout > settle")
        if not math.isfinite(timeout) or timeout < 0 or max_turns < 0:
            raise ValueError("search timeout/turn limit must be non-negative; zero means continuous")
        self.speed = angular_speed
        self.step = step_angle
        self.dwell = dwell
        self.settle = settle
        self.required_frames = confirmation_frames
        self.radius = confirmation_radius
        self.confirmation_timeout = confirmation_timeout
        self.timeout = timeout
        self.odom_timeout = odom_timeout
        self.max_drift = max_drift
        self.max_turns = max_turns
        self.continuous_rotation = continuous_rotation
        self.mode = "IDLE"
        self.error = None
        self.last_odom = None
        self.last_yaw = None
        self.start_xy = None
        self.travel = 0.0
        self.confirmed = False
        self.completed_turns = 0

    @property
    def active(self):
        return self.mode in ("DWELL", "ROTATE", "CONFIRM")

    def feedback(self, x, y, yaw, now):
        if not all(math.isfinite(v) for v in (x, y, yaw, now)):
            if self.active:
                self.fail("invalid odometry")
            return
        if self.active and self.last_yaw is not None:
            # Signed, wrapped yaw deltas: crossing +/-pi is continuous; a stuck
            # robot never completes a turn merely because time elapsed.
            delta = math.atan2(math.sin(yaw - self.last_yaw),
                               math.cos(yaw - self.last_yaw))
            self.travel += delta
            if self.mode == "CONFIRM" and (
                    abs(delta) > 0.002 or math.dist((x, y), self.last_xy) > 0.002):
                self.confirm_after = now + self.settle
                self.points = []
            if math.dist((x, y), self.start_xy) > self.max_drift:
                self.fail("base translated during in-place search")
        self.last_xy = (x, y)
        self.last_yaw = yaw
        self.last_odom = now

    def start(self, now):
        if self.last_odom is None or now - self.last_odom > self.odom_timeout:
            raise ValueError("fresh odometry required before search")
        self.start_xy = self.last_xy
        self.travel = 0.0
        self.started = now
        self.until = now + self.dwell
        self.next_stop = self.step
        self.turn_limit = 2 * math.pi
        self.completed_turns = 0
        self.error = None
        self.confirmed = False
        self.mode = "ROTATE" if self.continuous_rotation else "DWELL"

    def fail(self, reason):
        self.error = reason
        self.mode = "FAILED"

    def stop(self):
        self.mode = "IDLE"

    def tick(self, now):
        if not self.active:
            return 0.0
        if now - self.last_odom > self.odom_timeout:
            self.fail("odometry stopped")
        elif self.timeout > 0 and now - self.started >= self.timeout:
            self.fail("search timeout: no confirmed bottle")
        if not self.active:
            return 0.0
        if self.mode == "CONFIRM":
            if now < self.until:
                return 0.0
            self.mode = "ROTATE" if self.continuous_rotation else "DWELL"
            self.until = now + self.dwell
            if not self.continuous_rotation:
                return 0.0
        if self.continuous_rotation:
            # No angular checkpoints or dwell between empty turns. Completion
            # still comes from measured odometry, never an elapsed-time guess.
            while self.travel >= self.turn_limit - 0.02:
                self.completed_turns += 1
                if self.max_turns and self.completed_turns >= self.max_turns:
                    self.fail("full turn completed: no confirmed bottle")
                    return 0.0
                self.turn_limit += 2 * math.pi
            return self.speed
        if self.mode == "DWELL":
            if now < self.until:
                return 0.0
            if self.travel >= self.turn_limit - 0.02:
                self.completed_turns += 1
                if self.max_turns and self.completed_turns >= self.max_turns:
                    self.fail("full turn completed: no confirmed bottle")
                    return 0.0
                self.turn_limit += 2 * math.pi
            self.next_stop = min(self.travel + self.step, self.turn_limit)
            self.mode = "ROTATE"
        if self.travel >= self.next_stop - 0.015:
            self.mode = "DWELL"
            self.until = now + self.dwell
            return 0.0
        return self.speed

    def begin_confirmation(self, stamp_ns, now):
        """Brake for a visual candidate, including a 2D-only bearing hint."""
        if not self.active or self.mode == 'CONFIRM':
            return
        self.mode = 'CONFIRM'
        self.confirm_after = now + self.settle
        self.until = now + self.confirmation_timeout
        self.points = []
        self.last_stamp = stamp_ns

    def observe(self, point, stamp_ns, now):
        """True only after distinct observations of one stationary target.

        The first observation just stops rotation. Never use its coordinates as
        the grasp target; wait for the base to stop and receive new images.
        """
        if not self.active or not all(math.isfinite(v) for v in point):
            return False
        if self.mode != "CONFIRM":
            self.begin_confirmation(stamp_ns, now)
            return False
        if now < self.confirm_after or stamp_ns <= self.last_stamp:
            return False
        self.last_stamp = stamp_ns
        if self.points and any(math.dist(point, p) > self.radius for p in self.points):
            self.points = []
        self.points.append(point)
        if len(self.points) < self.required_frames:
            return False
        self.confirmed = True
        self.mode = "FOUND"
        return True
