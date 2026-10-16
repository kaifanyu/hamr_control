"""Geometric and rolling invariants for the active-ball visualization."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "active_ball_trajectory", PACKAGE / "scripts/active_ball_trajectory.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture(scope="module")
def route():
    return MODULE.DemoTrajectory()


def test_eight_corner_and_midpoint_visits_return_to_center(route):
    visits = [s for s in route.segments if s["phase"] == "target_hold"]
    assert len(visits) == 8
    assert [t["label"] for t in MODULE.TARGETS] == ["NE", "N", "NW", "W", "SW", "S", "SE", "E"]
    expected = [(1.2, 1.2), (0., 1.2), (-1.2, 1.2), (-1.2, 0.),
                (-1.2, -1.2), (0., -1.2), (1.2, -1.2), (1.2, 0.)]
    for index, (visit, xy) in enumerate(zip(visits, expected)):
        state = route.sample(visit["start_time"])
        assert (state["x"], state["y"]) == pytest.approx(xy)
        assert state["completed_targets"] == index + 1
        assert state["target_index"] == index
    returns = [s for s in route.segments if s["phase"] == "center_hold"]
    assert len(returns) == 8
    for segment in returns:
        state = route.sample(segment["start_time"])
        assert (state["x"], state["y"]) == (0., 0.)
    assert route.sample(route.duration)["completed_targets"] == 8


def test_duration_peak_speed_and_fixed_heading(route):
    total_distance = 8. * 1.2 * (1. + math.sqrt(2.))
    assert route.duration == pytest.approx(total_distance * 1.875 / .70 + 6. + 8. * (.5 + .35))
    for segment in route.segments:
        state = route.sample((segment["start_time"] + segment["end_time"]) / 2.)
        speed = math.hypot(state["vx"], state["vy"])
        assert speed == pytest.approx(.70 if segment["phase"] in ("outbound", "return") else 0.)
    for t in np.linspace(0., route.duration, 401):
        state = route.sample(t)
        assert state["yaw"] == state["yaw_rate"] == 0.
        assert abs(state["x"]) <= 1.2 + 1e-12
        assert abs(state["y"]) <= 1.2 + 1e-12
        assert math.hypot(state["vx"], state["vy"]) <= .70 + 1e-12
        assert np.max(np.linalg.norm(state["ball_omega_world"], axis=1)) <= 7. + 1e-12


def test_stops_are_smooth_and_orientations_do_not_jump(route):
    for segment in route.segments:
        for t in (segment["start_time"], segment["end_time"]):
            value = route.sample(t)
            before, after = route.sample(t - 1e-6), route.sample(t + 1e-6)
            assert (value["vx"], value["vy"]) == pytest.approx((0., 0.), abs=1e-12)
            assert (before["x"], before["y"]) == pytest.approx((after["x"], after["y"]), abs=1e-10)
            np.testing.assert_allclose(before["ball_rotations"], after["ball_rotations"], atol=1e-10)
            acceleration = np.array((after["vx"] - before["vx"], after["vy"] - before["vy"])) / 2e-6
            np.testing.assert_allclose(acceleration, 0., atol=3e-6)


def test_ball_rotation_derivative_matches_no_slip_velocity(route):
    dt = 1e-5
    for segment in route.segments:
        t = segment["start_time"] + .37 * segment["duration"]
        state = route.sample(t)
        omega = state["ball_omega_world"]
        body_velocity = np.array((state["vx"], state["vy"], 0.))
        contact_velocity = body_velocity + np.cross(omega, (0., 0., -.10))
        np.testing.assert_allclose(contact_velocity, 0., atol=1e-12)
        rotation = state["ball_rotations"]
        derivative = (route.sample(t + dt)["ball_rotations"] - route.sample(t - dt)["ball_rotations"]) / (2. * dt)
        # Each column of R is a world vector and must satisfy dR/dt = omega x R.
        expected = np.cross(omega[:, None, :], rotation.transpose(0, 2, 1)).transpose(0, 2, 1)
        np.testing.assert_allclose(derivative, expected, atol=3e-8)
        np.testing.assert_allclose(rotation @ rotation.transpose(0, 2, 1), np.repeat(np.eye(3)[None], 4, axis=0), atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(rotation), 1., atol=1e-12)


@pytest.mark.parametrize("twist", [(1., 0., 0.), (0., 1., 0.), (0., 0., 1.3), (.3, -.4, -.7)])
def test_general_holonomic_inverse_and_forward_kinematics(twist):
    omega = MODULE.inverse_kinematics(*twist)
    np.testing.assert_allclose(MODULE.forward_kinematics(omega), twist, atol=1e-12)
    vx, vy, yaw_rate = twist
    offsets = MODULE.BALL_OFFSETS
    center_velocity = np.column_stack((vx - yaw_rate * offsets[:, 1],
                                       vy + yaw_rate * offsets[:, 0], np.zeros(4)))
    np.testing.assert_allclose(center_velocity + np.cross(omega, (0., 0., -.10)), 0., atol=1e-12)


def test_sampling_is_order_independent_and_clamps_boundaries(route):
    t = route.duration * .317
    first = route.sample(t)
    route.sample(route.duration)
    again = route.sample(t)
    np.testing.assert_array_equal(first["ball_rotations"], again["ball_rotations"])
    assert route.sample(-1.)["time"] == 0.
    assert route.sample(route.duration + 1.)["time"] == route.duration
    # Every out-and-back retraces a single axis, restoring its ball orientation.
    np.testing.assert_allclose(route.sample(route.duration)["ball_rotations"], np.repeat(np.eye(3)[None], 4, axis=0), atol=1e-12)
    with pytest.raises(ValueError, match="finite"):
        route.sample(float("nan"))


def test_invalid_geometry_is_rejected():
    with pytest.raises(ValueError, match="positive"):
        MODULE.inverse_kinematics(0., 0., radius=0.)
    with pytest.raises(ValueError, match="shape"):
        MODULE.forward_kinematics(np.zeros((3, 3)))
    with pytest.raises(ValueError, match="unique"):
        MODULE.forward_kinematics(np.zeros((4, 3)), offsets=np.zeros((4, 2)))
