"""Independent geometry, differential consistency and drive-bound checks."""

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "continuous_waypoint", PACKAGE / "scripts/continuous_waypoint.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
POINTS = [(0., 0.), (0., 2.), (-2., 2.), (-2., 4.5), (0., 4.5), (0., 2.), (0., 0.)]
PARAMETERS = dict(
    speed=.25, radius=.1075, half_track=.33072, offset=.27114,
    max_translational_acceleration=.1, planning_wheel_rate=3.9,
    planning_wheel_acceleration=7.5, corner_deviation=.1, max_jerk=.2,
)


def plan(points=POINTS, **changes):
    return MODULE.ContinuousWaypointReference(points, **(PARAMETERS | changes))


@pytest.fixture(scope="module")
def route():
    return plan()


def polyline_distance(positions, points):
    points = np.asarray(points)
    edges = np.diff(points, axis=0)
    delta = np.asarray(positions)[:, None, :]-points[None, :-1, :]
    u = np.clip(np.sum(delta*edges, axis=2)/np.sum(edges*edges, axis=1), 0., 1.)
    return np.min(np.linalg.norm(delta-u[..., None]*edges, axis=2), axis=1)


def test_original_route_becomes_continuous_with_honest_corner_metadata(route):
    metadata = route.metadata()
    assert route.points == POINTS
    assert route.duration == pytest.approx(65.1554610278, abs=1e-8)
    assert 12. < route.length < 13.
    assert [c["waypoint_index"] for c in metadata["corners"]] == [1, 2, 3, 4]
    assert metadata["max_sample_interval_s"] <= .01+1e-12
    assert metadata["linear_chord_error_bound_m"] <= 1.25e-6+1e-12
    for corner in metadata["corners"]:
        assert corner["speed_m_s"] > .15
        assert corner["planned_waypoint_miss_m"] == pytest.approx(math.sqrt(2.)*.1)
        assert corner["max_intended_deviation_m"] == pytest.approx(.1)
        sample = route.sample(corner["closest_pass_time_s"])
        point = POINTS[corner["waypoint_index"]]
        assert math.hypot(sample[0]-point[0], sample[1]-point[1]) == pytest.approx(corner["planned_waypoint_miss_m"])
    # The repeated (0,2) waypoint on the return leg is passed once at full speed;
    # its index cannot accidentally refer to the first, rounded (0,2) corner.
    x, y, vx, vy, index = route.sample(route.starts[5])
    assert (x, y, vx, vy) == pytest.approx((0., 2., 0., -.25), abs=1e-12)
    assert index == 5
    assert route.starts[5] > metadata["corners"][-1]["end_time_s"]
    json.dumps(metadata, allow_nan=False)


def test_geometry_and_no_interior_stops(route):
    samples = route.metadata()["samples"]
    positions = [sample["position_m"] for sample in samples]
    assert np.max(polyline_distance(positions, POINTS)) <= .1+1e-12
    begin, end = route.metadata()["interior_motion_interval_s"]
    speeds = [math.hypot(*s["velocity_m_s"]) for s in samples if begin <= s["time_s"] <= end]
    assert min(speeds) > .15
    assert max(speeds) == pytest.approx(.25)
    assert route.sample_with_jerk(-1.) == (0., 0., 0., 0., 0., 0., 0., 0., 0)
    assert route.sample_with_jerk(route.duration+1.) == (0., 0., 0., 0., 0., 0., 0., 0., 5)


def test_velocity_and_acceleration_are_continuous_at_every_join(route):
    for time_s in [*route._phase_starts, route.duration]:
        before = route.sample_with_acceleration(time_s-1e-6)
        after = route.sample_with_acceleration(time_s+1e-6)
        assert before[:6] == pytest.approx(after[:6], abs=6e-7)


def test_arc_inversion_and_derivatives_agree_with_position_differences(route):
    # Includes locations close to arc lookup-cell boundaries. The lookup table
    # is a Newton seed, not a piecewise-linear position parameterization.
    times = list(np.linspace(.01, route.duration-.01, 301))
    for curve in route._curves.values():
        for distance in curve["arc_grid"][::17]:
            times.append(curve["start_time"]+distance/curve["speed"])
    dt = 1e-4
    for time_s in times:
        before = np.array(route.sample_with_acceleration(time_s-dt)[:6])
        value = np.array(route.sample_with_acceleration(time_s)[:6])
        after = np.array(route.sample_with_acceleration(time_s+dt)[:6])
        np.testing.assert_allclose((after[:2]-before[:2])/(2.*dt), value[2:4], atol=2e-8, rtol=0.)
        np.testing.assert_allclose((after[2:4]-before[2:4])/(2.*dt), value[4:6], atol=5e-6, rtol=0.)
        np.testing.assert_allclose((after[:2]-2.*value[:2]+before[:2])/dt**2,
                                   value[4:6], atol=8e-6, rtol=0.)


@pytest.mark.parametrize("changes", [
    {}, {"max_jerk": .015}, {"planning_wheel_acceleration": .5},
    {"max_translational_acceleration": .025}, {"planning_wheel_rate": .8},
])
def test_certified_and_observed_cartesian_bounds(changes):
    candidate = plan(**changes)
    limits = PARAMETERS | changes
    metadata = candidate.metadata()
    certificate = metadata["certification"]
    for observed, required in (
        ("max_speed_m_s", "speed"), ("max_acceleration_m_s2", "max_translational_acceleration"),
        ("max_jerk_m_s3", "max_jerk"), ("max_wheel_rate_rad_s", "planning_wheel_rate"),
        ("max_wheel_acceleration_rad_s2", "planning_wheel_acceleration"),
    ):
        assert certificate[observed] <= limits[required]*(1.+1e-10)
    for time_s in np.linspace(0., candidate.duration, 3001):
        sample = candidate.sample_with_jerk(float(time_s))
        assert math.hypot(*sample[2:4]) <= limits["speed"]*(1.+1e-10)
        assert math.hypot(*sample[4:6]) <= limits["max_translational_acceleration"]*(1.+1e-10)
        assert math.hypot(*sample[6:8]) <= limits["max_jerk"]*(1.+1e-10)


@pytest.mark.parametrize("angle_deg", [-150., -90., -20., 20., 90., 150.])
def test_blends_of_other_angles_and_short_segments(angle_deg):
    angle = math.radians(angle_deg)
    points = [(0., 0.), (.3, 0.), (.3+.08*math.cos(angle), .08*math.sin(angle))]
    candidate = plan(points)
    samples = [candidate.sample_with_jerk(t) for t in np.linspace(0., candidate.duration, 1001)]
    assert np.max(polyline_distance(np.array(samples)[:, :2], points)) <= .1+1e-12
    assert max(math.hypot(*s[4:6]) for s in samples) <= .1+1e-10
    assert max(math.hypot(*s[6:8]) for s in samples) <= .2+1e-10
    assert min(math.hypot(*s[2:4]) for s in samples[1:-1]) > 0.


@pytest.mark.parametrize("initial_yaw", [0., math.pi/2., math.pi, -math.pi/2.])
def test_independent_offset_drive_heading_and_wheel_derivatives(initial_yaw):
    candidate = plan(points=[(0., 0.), (1., 0.), (1., 1.)], planning_wheel_acceleration=.5)
    r, a, b = PARAMETERS["radius"], PARAMETERS["half_track"], PARAMETERS["offset"]
    theta = initial_yaw
    dt = candidate.duration/2000.
    def theta_dot(time_s, yaw):
        _, _, vx, vy, _ = candidate.sample(time_s)
        return (-math.sin(yaw)*vx+math.cos(yaw)*vy)/b
    for step in range(2000):
        t = step*dt
        k1 = theta_dot(t, theta)
        k2 = theta_dot(t+dt/2., theta+dt*k1/2.)
        k3 = theta_dot(t+dt/2., theta+dt*k2/2.)
        k4 = theta_dot(t+dt, theta+dt*k3)
        theta += dt*(k1+2.*k2+2.*k3+k4)/6.
        _, _, vx, vy, ax, ay, _ = candidate.sample_with_acceleration(t+dt)
        c, s = math.cos(theta), math.sin(theta)
        forward, lateral = c*vx+s*vy, -s*vx+c*vy
        rotation = lateral/b
        # Independently differentiate rotating world-to-body coordinates.
        forward_dot = c*ax+s*ay+rotation*lateral
        lateral_dot = -s*ax+c*ay-rotation*forward
        wheels = [(forward+sign*a*lateral/b)/r for sign in (-1., 1.)]
        wheel_accelerations = [(forward_dot+sign*a*lateral_dot/b)/r for sign in (-1., 1.)]
        assert max(map(abs, wheels)) <= PARAMETERS["planning_wheel_rate"]+1e-9
        assert max(map(abs, wheel_accelerations)) <= .5+1e-9


def test_straight_only_short_route_has_no_false_corner():
    candidate = plan([(0., 0.), (.002, 0.), (.005, 0.)])
    assert candidate.metadata()["corners"] == []
    assert candidate.sample(candidate.starts[1])[:2] == pytest.approx((.002, 0.))
    assert candidate.sample(candidate.starts[1])[2] > 0.


@pytest.mark.parametrize("points", [
    [], [(0., 0.)], [(0., 0.), (0., 0.)], [(0., 0.), (1., 2., 3.)],
    [(0., 0.), (math.inf, 1.)], [(0., 0.), (math.nan, 1.)],
    [(0., 0.), (1., 0.), (0., 0.)],
])
def test_rejects_invalid_or_unsupported_geometry(points):
    with pytest.raises(ValueError):
        plan(points)


@pytest.mark.parametrize("name", PARAMETERS)
@pytest.mark.parametrize("value", [0., -1., math.inf, math.nan])
def test_rejects_invalid_limits(name, value):
    with pytest.raises(ValueError, match="finite and positive"):
        plan(**{name: value})


@pytest.mark.parametrize("time_s", [math.nan, math.inf, -math.inf])
def test_rejects_nonfinite_sample_times(route, time_s):
    with pytest.raises(ValueError, match="Sample time must be finite"):
        route.sample(time_s)
