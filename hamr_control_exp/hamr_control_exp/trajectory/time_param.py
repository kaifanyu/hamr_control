"""Time parameterization: curvature-limited trapezoidal speed profile.

Produces a uniformly sampled trajectory (dt spacing) with continuous
velocity — the property the legacy piecewise-constant-velocity generator
lacks and the reason PID feedforward currently jumps at waypoints.
"""
from dataclasses import dataclass

import numpy as np

from .smoothing import SmoothPath


@dataclass
class SampledTrajectory:
    t: np.ndarray        # (M,) uniform timestamps from 0
    x: np.ndarray
    y: np.ndarray
    x_dot: np.ndarray
    y_dot: np.ndarray
    yaw: float           # constant turret yaw (world frame)
    dt: float
    total_time: float
    s_total: float

    def sample(self, t):
        """Pose/velocity at time t (holds endpoints, zero velocity outside)."""
        if t <= 0.0:
            return self.x[0], self.y[0], self.yaw, 0.0, 0.0, 0.0
        if t >= self.total_time:
            return self.x[-1], self.y[-1], self.yaw, 0.0, 0.0, 0.0
        i = min(int(t / self.dt), len(self.t) - 2)
        a = (t - self.t[i]) / self.dt
        lerp = lambda arr: float(arr[i] + a * (arr[i + 1] - arr[i]))
        return (lerp(self.x), lerp(self.y), self.yaw,
                lerp(self.x_dot), lerp(self.y_dot), 0.0)


def time_parameterize(path: SmoothPath, v_max, a_max, a_lat_max,
                      yaw=0.0, dt=0.02, ds=0.01) -> SampledTrajectory:
    """Forward/backward pass speed profile along the spline:
       v(s) <= min(v_max, sqrt(a_lat_max / |kappa(s)|)),  |dv/dt| <= a_max,
       v = 0 at both ends."""
    n = max(int(np.ceil(path.s_total / ds)) + 1, 2)
    s = np.linspace(0.0, path.s_total, n)
    ds_eff = s[1] - s[0]

    kappa = np.abs(path.curvature(s))
    v_lim = np.minimum(v_max, np.sqrt(a_lat_max / np.maximum(kappa, 1e-9)))

    v = v_lim.copy()
    v[0] = 0.0
    for i in range(n - 1):                      # forward (acceleration) pass
        v[i + 1] = min(v[i + 1], np.sqrt(v[i] ** 2 + 2.0 * a_max * ds_eff))
    v[-1] = 0.0
    for i in range(n - 2, -1, -1):              # backward (braking) pass
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2.0 * a_max * ds_eff))

    v_avg = np.maximum(0.5 * (v[1:] + v[:-1]), 1e-4)
    t_of_s = np.concatenate(([0.0], np.cumsum(ds_eff / v_avg)))
    total_time = float(t_of_s[-1])

    t = np.arange(0.0, total_time + dt, dt)
    s_of_t = np.interp(t, t_of_s, s)
    pos = path.position(s_of_t)
    tan = path.tangent(s_of_t)
    speed = np.interp(s_of_t, s, v)

    return SampledTrajectory(
        t=t, x=pos[:, 0], y=pos[:, 1],
        x_dot=tan[:, 0] * speed, y_dot=tan[:, 1] * speed,
        yaw=float(yaw), dt=float(dt),
        total_time=total_time, s_total=path.s_total)


def build_trajectory(waypoints_xy, v_max, a_max, a_lat_max,
                     yaw=0.0, dt=0.02) -> SampledTrajectory:
    """Convenience: waypoints -> spline -> timed trajectory."""
    return time_parameterize(SmoothPath(waypoints_xy),
                             v_max, a_max, a_lat_max, yaw=yaw, dt=dt)
