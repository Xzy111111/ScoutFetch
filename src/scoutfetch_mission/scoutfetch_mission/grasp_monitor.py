"""Held-object feedback gate, independent of pick-service success."""

import math


class GraspMonitor:
    def __init__(self, minimum=.060, maximum=.070, settle=.4, loss_delay=.25, timeout=1.0):
        self.minimum, self.maximum = minimum, maximum
        self.settle, self.loss_delay, self.timeout = settle, loss_delay, timeout
        self.received = self.good_since = self.bad_since = None
        self.healthy = False
        self.valid_width = False
        self.stamp_ns = None

    def update(self, width, healthy, stamp_ns, now):
        # Repeated/stale feedback must never prolong a held-object assertion.
        if stamp_ns <= 0 or (self.stamp_ns is not None and stamp_ns <= self.stamp_ns):
            return
        self.stamp_ns, self.received, self.healthy = stamp_ns, now, healthy
        self.valid_width = math.isfinite(width) and self.minimum <= width <= self.maximum
        if self.valid_width and healthy:
            self.bad_since = None
            if self.good_since is None:
                self.good_since = now
        else:
            self.good_since = None
            if self.bad_since is None:
                self.bad_since = now

    def state(self, now):
        if self.received is None or now-self.received > self.timeout:
            return 'FEEDBACK_STALE'
        if not self.healthy:
            return 'DRIVER_FAULT'
        if self.good_since is not None and now-self.good_since >= self.settle:
            return 'HELD'
        if self.bad_since is not None and now-self.bad_since >= self.loss_delay:
            return 'LOST'
        return 'VERIFYING'
