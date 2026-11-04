"""Geometry, conservative grid planning and one-shot victim tracking (no ROS)."""
import heapq
import math
from dataclasses import dataclass, field

import numpy as np
from scipy.ndimage import distance_transform_edt


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def speed_limit(clearance, maximum, margin, deceleration, latency):
    """v*latency + v²/(2*a) <= free distance after footprint and margin."""
    if not math.isfinite(clearance) or clearance <= margin:
        return 0.0
    return min(maximum, max(0.0, -deceleration * latency + math.sqrt(
        (deceleration * latency) ** 2 + 2 * deceleration * (clearance - margin))))


@dataclass
class Track:
    samples: list = field(default_factory=list)
    reported: bool = False
    anchor: object = None

    @property
    def center(self):
        return self.anchor if self.reported else np.median([s[1] for s in self.samples], axis=0)


class Victims:
    def __init__(self, limit, association=0.75, confirmations=5, spread=0.06):
        self.limit = limit
        self.association = association
        self.confirmations = confirmations
        self.spread = spread
        self.tracks = []
        self.count = 0

    def observe(self, xy, stamp):
        xy = np.asarray(xy, dtype=float)
        if xy.shape != (2,) or not np.all(np.isfinite(xy)):
            return None
        if self.count >= self.limit:
            return None
        candidates = [t for t in self.tracks if np.linalg.norm(t.center - xy) < self.association]
        # Ambiguous observations must never create another report.
        if len(candidates) > 1:
            return None
        if candidates:
            t = candidates[0]
        else:
            t = Track()
            self.tracks.append(t)
        if t.reported or self.count >= self.limit:
            return None
        if t.samples and stamp <= t.samples[-1][0]:
            return None
        t.samples = [(s, p) for s, p in t.samples if stamp - s < 3.0]
        t.samples.append((stamp, xy))
        if len(t.samples) < self.confirmations or stamp - t.samples[0][0] < 0.4:
            return None
        center = t.center
        if max(np.linalg.norm(p - center) for _, p in t.samples) > self.spread:
            return None
        # Reserve before returning: callers publish at most once, never retry.
        t.anchor = center.copy()
        t.reported = True
        self.count += 1
        return tuple(center)


class Grid:
    def __init__(self, values, resolution, origin, radius):
        self.values = np.asarray(values)
        self.resolution = resolution
        self.origin = origin  # x, y, yaw; rotated OccupancyGrid supported
        self.radius = radius
        self.static = self.values != 0  # unknown cells are blocked
        self.dynamic = {}
        self.refresh(0.0)

    def cell(self, x, y):
        ox, oy, a = self.origin
        dx, dy = x - ox, y - oy
        return (math.floor((-math.sin(a)*dx + math.cos(a)*dy)/self.resolution),
                math.floor((math.cos(a)*dx + math.sin(a)*dy)/self.resolution))

    def point(self, cell):
        r, c = cell
        ox, oy, a = self.origin
        x, y = (c + .5)*self.resolution, (r + .5)*self.resolution
        return (ox + math.cos(a)*x - math.sin(a)*y,
                oy + math.sin(a)*x + math.cos(a)*y)

    def inside(self, cell):
        r, c = cell
        return 0 <= r < self.values.shape[0] and 0 <= c < self.values.shape[1]

    def refresh(self, now):
        self.dynamic = {c: expiry for c, expiry in self.dynamic.items() if expiry > now}
        blocked = self.static.copy()
        for cell in self.dynamic:
            blocked[cell] = True
        # Treat map boundaries as obstacles, including maps with all-free edges.
        padded = np.pad(~blocked, 1, constant_values=False)
        self.clearance = distance_transform_edt(padded)[1:-1, 1:-1]*self.resolution
        self.free = self.clearance > self.radius + self.resolution*.71

    def add_obstacle(self, xy, expiry):
        cell = self.cell(*xy)
        if self.inside(cell) and not self.static[cell]:
            self.dynamic[cell] = expiry

    def paths(self, start):
        if not self.inside(start) or not self.free[start]:
            return {}, {}
        costs, parents, queue = {start: 0.0}, {}, [(0.0, start)]
        while queue:
            cost, cell = heapq.heappop(queue)
            if cost != costs[cell]:
                continue
            r, c = cell
            for dr, dc in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt = (r+dr, c+dc)
                if not self.inside(nxt) or not self.free[nxt]:
                    continue
                if dr and dc and (not self.free[r+dr,c] or not self.free[r,c+dc]):
                    continue
                step = math.hypot(dr, dc)*self.resolution
                new = cost + step*(1.0 + .08/max(.05, self.clearance[nxt]-self.radius))
                if new < costs.get(nxt, math.inf):
                    costs[nxt], parents[nxt] = new, cell
                    heapq.heappush(queue, (new, nxt))
        return costs, parents

    def route(self, start, goal, parents):
        if goal != start and goal not in parents:
            return []
        route = [goal]
        while route[-1] != start:
            route.append(parents[route[-1]])
        return [self.point(c) for c in reversed(route)]
