"""Geometric path smoothing: chord-length parameterized cubic spline."""
import numpy as np
from scipy.interpolate import CubicSpline


class SmoothPath:
    """C2 spline through waypoints, parameterized by (approximate) arc
    length. Evaluate position, unit tangent, and curvature at any s."""

    def __init__(self, points):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
            raise ValueError("points must be (N>=2, 2)")

        d = np.diff(points, axis=0)
        seg_len = np.hypot(d[:, 0], d[:, 1])
        if np.any(seg_len < 1e-9):
            keep = np.concatenate(([True], seg_len >= 1e-9))
            points = points[keep]
            d = np.diff(points, axis=0)
            seg_len = np.hypot(d[:, 0], d[:, 1])
        if len(points) < 2:
            raise ValueError("degenerate path")

        s_knots = np.concatenate(([0.0], np.cumsum(seg_len)))
        # Clamp end derivatives to the first/last segment directions so the
        # robot enters and leaves along the planned headings.
        d0 = d[0] / seg_len[0]
        d1 = d[-1] / seg_len[-1]
        self._sx = CubicSpline(s_knots, points[:, 0],
                               bc_type=((1, d0[0]), (1, d1[0])))
        self._sy = CubicSpline(s_knots, points[:, 1],
                               bc_type=((1, d0[1]), (1, d1[1])))
        self.s_total = float(s_knots[-1])
        self.knots = points

    def position(self, s):
        return np.stack([self._sx(s), self._sy(s)], axis=-1)

    def tangent(self, s):
        """Unit tangent vector(s) at s."""
        tx, ty = self._sx(s, 1), self._sy(s, 1)
        norm = np.maximum(np.hypot(tx, ty), 1e-9)
        return np.stack([tx / norm, ty / norm], axis=-1)

    def curvature(self, s):
        xp, yp = self._sx(s, 1), self._sy(s, 1)
        xpp, ypp = self._sx(s, 2), self._sy(s, 2)
        denom = np.maximum((xp * xp + yp * yp) ** 1.5, 1e-9)
        return (xp * ypp - yp * xpp) / denom
