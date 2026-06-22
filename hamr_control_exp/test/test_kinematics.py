"""Kinematics must exactly reproduce the legacy controller's math."""
import math

import numpy as np
import pytest

from hamr_control_exp.common.kinematics import (
    cap_world_velocity, jacobian, joint_rate_matrix, wrap_angle,
    world_vel_to_joint_omegas)

# Hardware geometry from hamr_bringup/config/hamr_hw_control_params.yaml
R, A, B = 0.1250, 0.345, 0.301


def legacy_compute_velocities(desired_velocity, yaw, r_w, a, b):
    """Verbatim reproduction of hamr_controller.py compute_velocities()."""
    c, s = np.cos(yaw), np.sin(yaw)
    J = np.array([
        [r_w/2 * (c - s*b/a), r_w/2 * (c + s*b/a), 0],
        [r_w/2 * (s + c*b/a), r_w/2 * (s - c*b/a), 0],
        [r_w/(2*a), -r_w/(2*a), 1]
    ])
    return np.linalg.solve(J, desired_velocity)


@pytest.mark.parametrize("seed", range(20))
def test_matches_legacy_controller(seed):
    rng = np.random.default_rng(seed)
    yaw = rng.uniform(-np.pi, np.pi)
    v = rng.uniform(-0.5, 0.5, size=3)
    ours = world_vel_to_joint_omegas(v, yaw, R, A, B)
    legacy = legacy_compute_velocities(v, yaw, R, A, B)
    np.testing.assert_allclose(ours, legacy, atol=1e-12)


@pytest.mark.parametrize("seed", range(10))
def test_jacobian_round_trip(seed):
    rng = np.random.default_rng(100 + seed)
    yaw = rng.uniform(-np.pi, np.pi)
    v = rng.uniform(-0.5, 0.5, size=3)
    omegas = world_vel_to_joint_omegas(v, yaw, R, A, B)
    np.testing.assert_allclose(jacobian(yaw, R, A, B) @ omegas, v, atol=1e-12)


def test_joint_rate_matrix_is_inverse():
    yaw = 0.7
    M = joint_rate_matrix(yaw, R, A, B)
    np.testing.assert_allclose(M @ jacobian(yaw, R, A, B), np.eye(3),
                               atol=1e-12)


def test_forward_motion_drives_wheels_equally():
    # Pure motion along the kinematic heading -> equal wheel speeds and a
    # turret counter-rotation of zero (no base rotation).
    yaw = 0.3
    v = np.array([0.2 * math.cos(yaw), 0.2 * math.sin(yaw), 0.0])
    w_r, w_l, w_t = world_vel_to_joint_omegas(v, yaw, R, A, B)
    assert w_r == pytest.approx(w_l, abs=1e-9)
    assert w_t == pytest.approx(0.0, abs=1e-9)


def test_turret_compensates_base_rotation():
    # Commanding zero turret yaw rate while the base spins must produce a
    # turret rate exactly canceling the base rotation.
    yaw = -1.1
    v = np.array([0.0, 0.0, 0.0])
    # inject pure base spin via wheels: solve for v with yaw_dot_turret = 0
    # then check third Jacobian row: yaw_t_dot = r/(2a)(w_r - w_l) + w_t = 0
    w_r, w_l, w_t = world_vel_to_joint_omegas(
        np.array([0.05, -0.03, 0.0]), yaw, R, A, B)
    base_spin = R / (2 * A) * (w_r - w_l)
    assert w_t == pytest.approx(-base_spin + 0.0, abs=1e-9)


def test_wrap_angle():
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)
    assert wrap_angle(0.5) == pytest.approx(0.5)


def test_cap_world_velocity_matches_pid_policy():
    v, sat = cap_world_velocity([0.5, 0.0, 3.0], 0.41, 2.0)
    assert sat
    assert math.hypot(v[0], v[1]) == pytest.approx(0.41)
    assert v[2] == pytest.approx(2.0)
    v, sat = cap_world_velocity([0.1, 0.1, 0.5], 0.41, 2.0)
    assert not sat
    np.testing.assert_allclose(v, [0.1, 0.1, 0.5])
