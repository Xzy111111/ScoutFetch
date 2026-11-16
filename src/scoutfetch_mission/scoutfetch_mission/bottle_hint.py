"""Short-lived visual bearings, separate from confirmed grasp targets."""

import math


def optical_bearing(u, v, intrinsics, quaternion):
    """Project an optical camera ray into map XY using the image-time TF."""
    fx, fy, cx, cy = intrinsics
    if not all(math.isfinite(a) for a in (u, v, fx, fy, cx, cy)) or min(fx, fy) <= 0:
        raise ValueError('invalid camera ray')
    x, y, z, w = quaternion
    if not all(math.isfinite(a) for a in quaternion) or abs(x*x+y*y+z*z+w*w-1) > .001:
        raise ValueError('invalid camera rotation')
    a, b = (u-cx)/fx, (v-cy)/fy
    dx = (1-2*(y*y+z*z))*a + 2*(x*y-z*w)*b + 2*(x*z+y*w)
    dy = 2*(x*y+z*w)*a + (1-2*(x*x+z*z))*b + 2*(y*z-x*w)
    if math.hypot(dx, dy) < .1:
        raise ValueError('camera ray points vertically')
    return math.atan2(dy, dx)


class BottleHint:
    def __init__(self, lifetime=20., consistency=math.radians(15)):
        self.lifetime, self.consistency = lifetime, consistency
        self.clear()

    def clear(self):
        self.bearing = self.received = self.first_received = None
        self.stamp_ns = 0
        self.frames = 0

    def observe(self, bearing, stamp_ns, now):
        if not math.isfinite(bearing) or not math.isfinite(now) or stamp_ns <= self.stamp_ns:
            return False
        if (self.bearing is None or now-self.received > 1.5 or
                abs(math.atan2(math.sin(bearing-self.bearing),
                               math.cos(bearing-self.bearing))) > self.consistency):
            self.frames = 0
            self.first_received = now
        self.bearing, self.received, self.stamp_ns = bearing, now, stamp_ns
        self.frames += 1
        return self.ready(now)

    def ready(self, now):
        return (self.frames >= 2 and self.received-self.first_received >= .1 and
                0 <= now-self.received <= self.lifetime)

    def take(self, now):
        bearing = self.bearing if self.ready(now) else None
        self.clear()
        return bearing
