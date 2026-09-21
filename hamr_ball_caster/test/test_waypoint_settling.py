"""A delayed vehicle must hold the endpoint without advancing the reference."""
import importlib.util
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1]/"scripts"
spec = importlib.util.spec_from_file_location("schedule_runner", SCRIPTS/"run_waypoint_sim.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def schedule():
    route = runner.PolylineReference([[0, 0], [0, 1], [1, 1]], 1.)
    return runner.SettledWaypointSchedule(route, .001, .005, .2, 3.)


def test_reference_waits_for_both_position_and_speed_and_continuous_dwell():
    s = schedule()
    s.update(1., [0, .95], 0.)
    assert s.state == "waypoint_settle"
    assert s.sample(1.5)[:4] == (0., 1., 0., 0.)
    s.update(1.5, [0, 1], .1)
    s.update(1.6, [0, 1], 0.)
    s.update(1.7, [0, 1], .1)  # Disturbance resets dwell.
    s.update(1.8, [0, 1], 0.)
    s.update(1.99, [0, 1], 0.)
    assert len(s.departures) == 1
    s.update(2.01, [0, 1], 0.)
    assert s.departures == [0., 2.01]
    assert s.arrivals[:2] == [0., 1.]
    # Historical sampling must keep the held reference even after release.
    assert s.sample(1.9)[:4] == (0., 1., 0., 0.)
    assert s.sample(2.51)[:2] == pytest.approx((.5, 1.))


def test_last_waypoint_must_settle_before_completion():
    s = schedule()
    s.update(1., [0, 1], 0.)
    s.update(1.21, [0, 1], 0.)
    s.update(2.21, [.9, 1], 0.)
    assert s.completed_at is None
    s.update(2.3, [1, 1], 0.)
    s.update(2.51, [1, 1], 0.)
    assert s.completed_at == 2.51
    assert s.sample(3.)[:4] == (1., 1., 0., 0.)


def test_failed_settling_has_bounded_wait():
    s = schedule()
    with pytest.raises(RuntimeError, match="did not settle"):
        s.update(4.1, [0, .9], .1)
