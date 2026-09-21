"""Feasible reference checks, including an independent moving-heading rollout."""

import importlib.util
import math
from pathlib import Path

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "feasible_waypoint", PACKAGE / "scripts/feasible_waypoint.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

POINTS = [(0., 0.), (0., 2.), (-2., 2.), (-2., 4.5), (0., 4.5), (0., 2.), (0., 0.)]
DEFAULTS = dict(
    speed=.25, radius=.1075, half_track=.33072, offset=.27114,
    max_translational_acceleration=.12,
    planning_wheel_rate=3.9, planning_wheel_acceleration=7.5,
)


def plan(points=POINTS, **overrides):
    return MODULE.FeasibleWaypointReference(points, **(DEFAULTS | overrides))


def test_preserves_route_and_stops_at_every_waypoint():
    route = plan()
    assert route.points == POINTS
    assert route.length == 13.
    # Peak speed remains 0.25 m/s; its average is smaller because of the stops.
    assert route.duration == pytest.approx(97.5)
    assert route.sample(-10.) == (0., 0., 0., 0., 0)
    assert route.sample(route.duration + 10.) == (0., 0., 0., 0., 5)
    for waypoint, arrival in zip(POINTS, route.starts):
        sample = route.sample_with_acceleration(arrival)
        assert sample[:2] == pytest.approx(waypoint, abs=1e-14)
        assert sample[2:6] == pytest.approx([0.] * 4, abs=1e-14)
        # C2 across turns (including the collinear waypoint), not just sampled rest.
        before = route.sample_with_acceleration(arrival - 1e-5)
        after = route.sample_with_acceleration(arrival + 1e-5)
        assert before[:6] == pytest.approx(after[:6], abs=2e-6)


def test_progress_is_monotone_on_original_straight_segments():
    route = plan()
    for index, segment in enumerate(route.segments):
        first, last = POINTS[index:index + 2]
        dx, dy = last[0] - first[0], last[1] - first[1]
        last_progress = -1.
        for fraction in range(1001):
            t = route.starts[index] + segment["duration_s"] * fraction / 1000.
            x, y, vx, vy, _ = route.sample(t)
            progress = ((x-first[0])*dx + (y-first[1])*dy) / (dx*dx + dy*dy)
            assert last_progress - 1e-12 <= progress <= 1. + 1e-12
            assert (x-first[0])*dy - (y-first[1])*dx == pytest.approx(0., abs=1e-13)
            assert vx*dy - vy*dx == pytest.approx(0., abs=1e-13)
            last_progress = progress


@pytest.mark.parametrize("overrides", [
    {}, {"speed": .03}, {"max_translational_acceleration": .005},
    {"planning_wheel_rate": .5}, {"planning_wheel_acceleration": .1},
])
def test_each_planning_limit_is_respected(overrides):
    route = plan(**overrides)
    limits = DEFAULTS | overrides
    for index, segment in enumerate(route.segments):
        assert segment["wheel_speed_bound_rad_s"] <= limits["planning_wheel_rate"] * (1.+1e-12)
        assert segment["wheel_acceleration_bound_rad_s2"] <= limits["planning_wheel_acceleration"] * (1.+1e-12)
        for fraction in range(1001):
            t = route.starts[index] + segment["duration_s"] * fraction / 1000.
            _, _, vx, vy, ax, ay, _ = route.sample_with_acceleration(t)
            assert math.hypot(vx, vy) <= limits["speed"] * (1.+1e-12)
            assert math.hypot(ax, ay) <= limits["max_translational_acceleration"] * (1.+1e-12)


@pytest.mark.parametrize("initial_yaw", [i * math.pi / 4. for i in range(8)])
def test_wheel_bounds_during_independent_heading_dynamics(initial_yaw):
    """Differentiate inverse-kinematic wheels along a dynamically rotating base.

    This would expose omitting the v^2/b wheel-acceleration contribution: a
    constant world velocity can still require accelerating wheels as yaw moves.
    """
    # Tight acceleration limits deliberately exercise timing from the rotational term.
    route = plan(points=[(0., 0.), (.6, .8)], planning_wheel_acceleration=.7)
    radius, half_track, offset = DEFAULTS["radius"], DEFAULTS["half_track"], DEFAULTS["offset"]
    yaw = initial_yaw
    steps = 4000
    dt = route.duration / steps
    previous_wheels = (0., 0.)

    def heading_rate(time_s, angle):
        _, _, vx, vy, _ = route.sample(time_s)
        return (-math.sin(angle)*vx + math.cos(angle)*vy) / offset

    for step in range(steps):
        t = step * dt
        k1 = heading_rate(t, yaw)
        k2 = heading_rate(t + dt/2., yaw + dt*k1/2.)
        k3 = heading_rate(t + dt/2., yaw + dt*k2/2.)
        k4 = heading_rate(t + dt, yaw + dt*k3)
        yaw += dt * (k1 + 2*k2 + 2*k3 + k4) / 6.
        _, _, vx, vy, _ = route.sample(t + dt)
        forward = math.cos(yaw)*vx + math.sin(yaw)*vy
        omega = (-math.sin(yaw)*vx + math.cos(yaw)*vy) / offset
        wheels = ((forward-half_track*omega)/radius, (forward+half_track*omega)/radius)
        assert max(abs(w) for w in wheels) <= DEFAULTS["planning_wheel_rate"] + 1e-9
        assert max(abs(w-p)/dt for w, p in zip(wheels, previous_wheels)) <= .7 + 1e-8
        previous_wheels = wheels


@pytest.mark.parametrize("name", DEFAULTS)
@pytest.mark.parametrize("value", [0., -1., math.inf, math.nan])
def test_rejects_invalid_limits(name, value):
    with pytest.raises(ValueError, match="finite and positive"):
        plan(**{name: value})


@pytest.mark.parametrize("points", [
    [], [(0., 0.)], [(0., 0.), (1., 2., 3.)],
    [(0., 0.), (math.nan, 1.)], [(0., 0.), (math.inf, 1.)],
    [(0., 0.), (0., 0.), (1., 0.)],
])
def test_rejects_invalid_waypoints(points):
    with pytest.raises(ValueError):
        plan(points=points)


@pytest.mark.parametrize("time_s", [math.inf, -math.inf, math.nan])
def test_rejects_nonfinite_sample_time(time_s):
    with pytest.raises(ValueError, match="Sample time must be finite"):
        plan().sample(time_s)
