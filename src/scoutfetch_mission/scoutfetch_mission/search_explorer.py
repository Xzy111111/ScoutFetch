"""Choose new search views from the live costmap, without object coordinates."""

from collections import deque
import math

import cv2
import numpy as np


class SearchExplorer:
    def __init__(self, step=3.0, view_range=3.2, clearance=.39, max_views=24):
        self.step, self.view_range, self.clearance, self.max_views = step, view_range, clearance, max_views
        self.grid = None
        self.visited = []
        self.failed = []
        self.goal_count = 0
        self.last_goal = None

    def update(self, values, resolution, origin):
        grid = np.asarray(values, dtype=np.int16)
        if (grid.ndim != 2 or min(grid.shape) < 2 or resolution <= 0 or
                not np.isfinite(resolution) or not np.isfinite(origin).all()):
            raise ValueError('invalid exploration costmap')
        changed = (self.grid is None or self.grid.shape != grid.shape or
                   self.resolution != resolution or self.origin != tuple(origin))
        self.grid, self.resolution, self.origin = grid.copy(), resolution, tuple(origin)
        if changed:
            self.seen = np.zeros(grid.shape, dtype=bool)

    def reset(self):
        self.visited, self.failed = [], []
        self.goal_count = 0
        self.last_goal = None
        if self.grid is not None:
            self.seen.fill(False)

    def cell(self, x, y):
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = x-ox, y-oy
        return int(math.floor((-s*dx+c*dy)/self.resolution)), int(math.floor((c*dx+s*dy)/self.resolution))

    def world(self, row, col):
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        x, y = (col+.5)*self.resolution, (row+.5)*self.resolution
        return ox+c*x-s*y, oy+s*x+c*y

    def pose_free(self, x, y, yaw, half_x=.325, half_y=.25):
        """Check a proposed station against the latest full footprint map."""
        if self.grid is None:
            return False
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [self.cell(x+c*a-s*b, y+s*a+c*b)[::-1] for a, b in
                   ((-half_x, -half_y), (half_x, -half_y),
                    (half_x, half_y), (-half_x, half_y))]
        points = np.asarray(corners, dtype=np.int32)
        h, w = self.grid.shape
        if (points[:, 0].min() < 0 or points[:, 0].max() >= w or
                points[:, 1].min() < 0 or points[:, 1].max() >= h):
            return False
        row, col = self.cell(x, y)
        if not 0 <= self.grid[row, col] < 99:
            return False
        # Match scoutfetch_reach's cell-centre footprint semantics. Filling
        # rounded pixel vertices expands a rotated rectangle by another cell
        # and can reject stations the planner has already robustly validated.
        rows, cols = np.mgrid[points[:, 1].min():points[:, 1].max()+1,
                              points[:, 0].min():points[:, 0].max()+1]
        ox, oy, origin_yaw = self.origin
        gx, gy = (cols+.5)*self.resolution, (rows+.5)*self.resolution
        wx = ox+math.cos(origin_yaw)*gx-math.sin(origin_yaw)*gy
        wy = oy+math.sin(origin_yaw)*gx+math.cos(origin_yaw)*gy
        lx, ly = c*(wx-x)+s*(wy-y), -s*(wx-x)+c*(wy-y)
        inside = (np.abs(lx) <= half_x) & (np.abs(ly) <= half_y)
        values = self.grid[rows[inside], cols[inside]]
        return bool(np.all((values >= 0) & (values < 100)))

    def _visible(self, row, col):
        angles = np.linspace(0, 2*math.pi, 72, endpoint=False)
        steps = np.arange(0, self.view_range, max(.075, self.resolution))/self.resolution
        rows = np.rint(row+np.sin(angles[:, None])*steps).astype(int)
        cols = np.rint(col+np.cos(angles[:, None])*steps).astype(int)
        h, w = self.grid.shape
        inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
        rows, cols = np.clip(rows, 0, h-1), np.clip(cols, 0, w-1)
        values = self.grid[rows, cols]
        blocked = ~inside | (values < 0) | (values >= 100)
        visible = np.cumsum(blocked, axis=1) == 0
        return np.unique(rows[visible]*w+cols[visible])

    def mark_scanned(self, x, y):
        if self.grid is None:
            return
        row, col = self.cell(x, y)
        h, w = self.grid.shape
        if not (0 <= row < h and 0 <= col < w):
            return
        self.seen.flat[self._visible(row, col)] = True
        if not any(math.hypot(x-a, y-b) < .5 for a, b in self.visited):
            self.visited.append((x, y))

    def reject_last(self):
        if self.last_goal is not None:
            self.failed.append(self.last_goal[:2])

    def choose(self, x, y, target=None, bearing=None):
        if self.grid is None or (self.max_views and self.goal_count >= self.max_views):
            return None
        h, w = self.grid.shape
        known = self.grid >= 0
        hard_free = known & (self.grid < 100)
        distances = cv2.distanceTransform(hard_free.astype(np.uint8), cv2.DIST_L2, 5)*self.resolution
        passable = (distances >= self.clearance) & (self.grid < 99)
        row, col = self.cell(x, y)
        if not (0 <= row < h and 0 <= col < w):
            return None
        if not passable[row, col]:
            candidates = np.argwhere(passable)
            if not len(candidates):
                return None
            near = np.argmin(np.sum((candidates-(row, col))**2, axis=1))
            rr, cc = candidates[near]
            if math.hypot(rr-row, cc-col)*self.resolution > .5:
                return None
            row, col = int(rr), int(cc)
        # Connected path distances avoid selecting a point through a wall or
        # disconnected island. Nav2 subsequently plans the actual driving path.
        path = np.full((h, w), np.inf, dtype=np.float32)
        path[row, col] = 0
        queue = deque([(row, col)])
        limit = self.step*1.7
        while queue:
            r, c = queue.popleft()
            distance = path[r, c]+self.resolution
            if distance > limit:
                continue
            for rr, cc in ((r-1, c), (r+1, c), (r, c-1), (r, c+1)):
                if 0 <= rr < h and 0 <= cc < w and passable[rr, cc] and not np.isfinite(path[rr, cc]):
                    path[rr, cc] = distance
                    queue.append((rr, cc))
        stride = max(1, round((.3 if bearing is not None else .6)/self.resolution))
        best = None
        best_score = -math.inf
        for r in range(0, h, stride):
            for c in range(0, w, stride):
                travel = float(path[r, c])
                if not .9 <= travel <= limit:
                    continue
                wx, wy = self.world(r, c)
                if not .9 <= math.hypot(wx-x, wy-y) <= self.step:
                    continue
                if any(math.hypot(wx-a, wy-b) < .8 for a, b in self.visited+self.failed):
                    continue
                if bearing is not None:
                    # A glimpse provides direction, not trustworthy range.
                    # Advance at most 1.5 m, staying in the connected free area.
                    dx, dy = wx-x, wy-y
                    progress = dx*math.cos(bearing)+dy*math.sin(bearing)
                    lateral = abs(-dx*math.sin(bearing)+dy*math.cos(bearing))
                    if math.hypot(dx, dy) > 1.5 or progress < .6 or lateral > .7*progress:
                        continue
                    yaw = bearing
                    if not self.pose_free(wx, wy, yaw):
                        continue
                    score = progress-1.5*lateral-.2*travel
                elif target is not None:
                    distance_to_target = math.hypot(wx-target[0], wy-target[1])
                    if not 1.3 <= distance_to_target <= 2.2:
                        continue
                    score = -3.0*distance_to_target-.35*travel+.1*min(distances[r, c], 1.0)
                    yaw = math.atan2(target[1]-wy, target[0]-wx)
                else:
                    visible = self._visible(r, c)
                    fresh = np.count_nonzero(~self.seen.flat[visible])
                    if fresh*self.resolution**2 < .15:
                        continue
                    score = fresh*self.resolution**2 - .35*travel + .1*min(distances[r, c], 1.0)
                    yaw = math.atan2(wy-y, wx-x)
                if score > best_score:
                    best_score = score
                    best = (wx, wy, yaw)
        if best is not None:
            self.last_goal = best
            self.goal_count += 1
        return best
