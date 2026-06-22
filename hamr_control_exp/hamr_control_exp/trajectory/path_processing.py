"""Waypoint pre-processing: pruning and line-of-sight shortcutting.

Pure numpy/scipy — no ROS imports — so it is unit-testable standalone.
"""
import math

import numpy as np


class GridInfo:
    """Minimal occupancy grid wrapper (decoupled from nav_msgs)."""

    def __init__(self, data, width, height, resolution, origin_x, origin_y,
                 occupied_thresh=50, unknown_is_blocked=True):
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        blocked = grid >= occupied_thresh
        if unknown_is_blocked:
            blocked |= grid < 0
        self.blocked = blocked
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.width = int(width)
        self.height = int(height)

    def inflate(self, radius_m):
        if radius_m <= 0.0:
            return
        from scipy import ndimage
        cells = int(math.ceil(radius_m / self.resolution))
        yy, xx = np.ogrid[-cells:cells + 1, -cells:cells + 1]
        disk = (xx * xx + yy * yy) <= cells * cells
        self.blocked = ndimage.binary_dilation(self.blocked, structure=disk)

    def is_blocked(self, x, y):
        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return True
        return bool(self.blocked[iy, ix])

    def line_free(self, p0, p1):
        dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        n = max(2, int(dist / (self.resolution * 0.5)) + 1)
        for t in np.linspace(0.0, 1.0, n):
            if self.is_blocked(p0[0] + t * (p1[0] - p0[0]),
                               p0[1] + t * (p1[1] - p0[1])):
                return False
        return True


def prune_close_points(points, min_spacing):
    """Drop points closer than min_spacing to the previously kept one.
    First and last points are always kept."""
    points = np.asarray(points, dtype=float)
    if len(points) < 3:
        return points
    kept = [points[0]]
    for p in points[1:-1]:
        if np.hypot(*(p - kept[-1])) >= min_spacing:
            kept.append(p)
    kept.append(points[-1])
    return np.array(kept)


def prune_collinear(points, tol=1e-3):
    """Remove interior points that lie (nearly) on the line between their
    neighbors. tol is the max perpendicular deviation in meters."""
    points = np.asarray(points, dtype=float)
    if len(points) < 3:
        return points
    kept = [points[0]]
    for i in range(1, len(points) - 1):
        a, b, c = kept[-1], points[i], points[i + 1]
        ac = c - a
        norm = np.hypot(*ac)
        if norm < 1e-9:
            continue
        deviation = abs(ac[0] * (b[1] - a[1]) - ac[1] * (b[0] - a[0])) / norm
        if deviation > tol:
            kept.append(b)
    kept.append(points[-1])
    return np.array(kept)


def los_shortcut(points, grid: GridInfo):
    """Greedy line-of-sight shortcutting: from each kept point, jump to the
    farthest later point reachable by a straight free line."""
    points = np.asarray(points, dtype=float)
    if grid is None or len(points) < 3:
        return points
    kept = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not grid.line_free(points[i], points[j]):
            j -= 1
        kept.append(points[j])
        i = j
    return np.array(kept)


def simplify_path(points, grid: GridInfo = None,
                  min_spacing=0.10, collinear_tol=1e-3):
    """Full pipeline: collinear pruning -> LOS shortcut -> spacing prune.
    Returns an (N,2) array with N >= 2, or None if degenerate."""
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return None
    points = prune_collinear(points, collinear_tol)
    if grid is not None:
        points = los_shortcut(points, grid)
    points = prune_close_points(points, min_spacing)
    # Collapse a degenerate (zero-length) path
    if len(points) == 2 and np.hypot(*(points[1] - points[0])) < 1e-6:
        return None
    return points
