"""Verify scan feedback timing independently of reference-publisher timing."""

import pytest

from reference_trajectory.continuous_execution import ContinuousExecution, Feedback, build_plan
from reference_trajectory.continuous_waypoint_node import validate_runtime_config


def session_at(x=0.0, y=0.0):
    cfg = validate_runtime_config({
        'points_m_flat': [0., 0., 0., 0.1],
        'origin_mode': 'vicon_absolute', 'odom_timeout_s': 0.20,
    })
    session = ContinuousExecution(cfg, build_plan(cfg))
    for i in range(40):
        session.observe(Feedback(x, y, 0., 9000. + i * .01, i * .01, 'world'))
    return session


def test_scan_waits_200_ms_for_feedback_but_retains_80_ms_reference_guard():
    session = session_at()
    assert session.start(.39, subscribers=1, publishers=1)[0]
    for now in (.44, .49, .54, .58):
        assert session.step(now) is not None
    assert session.step(.591) is None
    assert 'Odometry became stale' in session.reason
    session = session_at()
    assert session.start(.39, subscribers=1, publishers=1)[0]
    assert session.step(.471) is None
    assert 'Reference callback gap' in session.reason


def test_start_rejection_reports_actual_target_distance_and_tolerance():
    session = session_at(-.057, .031)
    accepted, message = session.start(.39, subscribers=1, publishers=1)
    assert not accepted
    for text in ('current=(-0.057, 0.031)', 'required=(0.000, 0.000)',
                 'distance=0.065', 'tolerance=0.030'):
        assert text in message


def test_feedback_timeout_above_200_ms_is_rejected():
    with pytest.raises(ValueError, match='odom_timeout_s'):
        validate_runtime_config({'odom_timeout_s': .201})
