"""Shared HAMR kinematics.

The Jacobian here is a copy of hamr_control/hamr_controller.py
compute_velocities() so the experimental controllers command the exact same
hardware mapping as the PID stack. Kept as a copy (not an import) so this
package has no code dependency on the legacy package and can be removed
independently.
"""
import math

import numpy as np


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def jacobian(yaw, r_wheel, a_wheel, b_wheel):
    """J maps joint rates [omega_right, omega_left, omega_turret] to world
    velocities [x_dot, y_dot, yaw_turret_dot]. `yaw` is the *kinematic* base
    yaw (measured yaw + base_yaw_offset)."""
    r_w, a, b = r_wheel, a_wheel, b_wheel
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([
        [r_w / 2 * (c - s * b / a), r_w / 2 * (c + s * b / a), 0],
        [r_w / 2 * (s + c * b / a), r_w / 2 * (s - c * b / a), 0],
        [r_w / (2 * a), -r_w / (2 * a), 1],
    ])


def world_vel_to_joint_omegas(desired_velocity, yaw, r_wheel, a_wheel, b_wheel):
    """Inverse kinematics: desired world velocity [x_dot, y_dot, yaw_dot]
    -> joint rates [omega_right, omega_left, omega_turret]."""
    return np.linalg.solve(jacobian(yaw, r_wheel, a_wheel, b_wheel),
                           np.asarray(desired_velocity, dtype=float))


def joint_rate_matrix(yaw, r_wheel, a_wheel, b_wheel):
    """M = J^-1 so that omegas = M @ world_velocity. Used by MPC to express
    joint-rate limits as linear constraints on world velocity."""
    return np.linalg.inv(jacobian(yaw, r_wheel, a_wheel, b_wheel))


def cap_world_velocity(v, xy_dot_limit, yaw_dot_limit):
    """Apply the same saturation policy as the PID node: scale the xy norm,
    clamp yaw rate. Returns (v_capped, was_saturated)."""
    v = np.array(v, dtype=float)
    saturated = False
    xy_norm = math.hypot(v[0], v[1])
    if xy_norm > xy_dot_limit:
        v[0] *= xy_dot_limit / xy_norm
        v[1] *= xy_dot_limit / xy_norm
        saturated = True
    if abs(v[2]) > yaw_dot_limit:
        v[2] = math.copysign(yaw_dot_limit, v[2])
        saturated = True
    return v, saturated
