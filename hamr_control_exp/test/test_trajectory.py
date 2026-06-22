"""Trajectory generation invariants (no ROS required)."""
import numpy as np
import pytest

from hamr_control_exp.trajectory.path_processing import (
    prune_close_points, prune_collinear, simplify_path)
from hamr_control_exp.trajectory.time_param import build_trajectory

V_MAX, A_MAX, A_LAT = 0.3, 0.15, 0.1

SQUARE = np.array([
    [0.0, 0.0], [0.0, 3.0], [1.5, 3.0], [1.5, 5.0],
    [-1.5, 5.0], [-1.5, 3.0], [0.0, 3.0], [0.0, 0.0]])


@pytest.fixture(scope="module")
def traj():
    return build_trajectory(SQUARE, V_MAX, A_MAX, A_LAT, yaw=0.4, dt=0.02)


def test_endpoints(traj):
    assert traj.x[0] == pytest.approx(SQUARE[0, 0], abs=1e-6)
    assert traj.y[0] == pytest.approx(SQUARE[0, 1], abs=1e-6)
    assert traj.x[-1] == pytest.approx(SQUARE[-1, 0], abs=1e-3)
    assert traj.y[-1] == pytest.approx(SQUARE[-1, 1], abs=1e-3)


def test_speed_limit(traj):
    speed = np.hypot(traj.x_dot, traj.y_dot)
    assert speed.max() <= V_MAX * 1.01


def test_velocity_continuity(traj):
    # The headline fix over the legacy generator: finite-difference jumps in
    # commanded velocity must be bounded by the accel limit, never the step
    # discontinuities (~2*v_max) the piecewise-constant version produces.
    dvx = np.diff(traj.x_dot) / traj.dt
    dvy = np.diff(traj.y_dot) / traj.dt
    accel = np.hypot(dvx, dvy)
    # allow modest overshoot: lateral+tangential combine on corners
    assert accel.max() <= (A_MAX + A_LAT) * 2.0


def test_starts_and_ends_at_rest(traj):
    assert np.hypot(traj.x_dot[0], traj.y_dot[0]) <= 1e-6
    assert np.hypot(traj.x_dot[-1], traj.y_dot[-1]) <= A_MAX * traj.dt * 2


def test_curvature_slowdown(traj):
    # Speed in the corners must drop below the straightaway speed.
    speed = np.hypot(traj.x_dot, traj.y_dot)
    cruise = np.percentile(speed, 90)
    assert speed[len(speed) // 4: -len(speed) // 4].min() < cruise * 0.9


def test_sample_holds_endpoints(traj):
    x, y, yaw, xd, yd, yawd = traj.sample(-1.0)
    assert (x, y) == (traj.x[0], traj.y[0]) and xd == yd == 0.0
    x, y, yaw, xd, yd, yawd = traj.sample(traj.total_time + 5.0)
    assert (x, y) == (traj.x[-1], traj.y[-1]) and xd == yd == 0.0
    assert yaw == pytest.approx(0.4)


def test_yaw_constant(traj):
    assert traj.yaw == pytest.approx(0.4)
    for t in np.linspace(0, traj.total_time, 17):
        assert traj.sample(t)[2] == pytest.approx(0.4)
        assert traj.sample(t)[5] == 0.0  # yaw_dot


def test_prune_collinear():
    pts = np.array([[0, 0], [1, 0], [2, 0], [2, 1], [2, 2]], dtype=float)
    out = prune_collinear(pts)
    np.testing.assert_allclose(out, [[0, 0], [2, 0], [2, 2]])


def test_prune_close_points():
    pts = np.array([[0, 0], [0.01, 0], [1, 0], [1.01, 0], [2, 0]], dtype=float)
    out = prune_close_points(pts, 0.1)
    np.testing.assert_allclose(out, [[0, 0], [1, 0], [2, 0]])


def test_simplify_degenerate():
    assert simplify_path(np.array([[1.0, 1.0]])) is None
    assert simplify_path(np.array([[1.0, 1.0], [1.0, 1.0]])) is None


def test_dense_grid_path_simplifies():
    # A* outputs grid centers every 5 cm; a straight 3 m leg should collapse
    # to its two endpoints.
    pts = np.stack([np.zeros(61), np.linspace(0, 3.0, 61)], axis=1)
    out = simplify_path(pts, min_spacing=0.10)
    assert len(out) == 2
