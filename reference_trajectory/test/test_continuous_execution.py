"""Verify real-car arming, frame placement and playback silence."""

import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from reference_trajectory.continuous_execution import (
    ContinuousExecution,
    Feedback,
    build_plan,
    validate_config,
)
from reference_trajectory.continuous_waypoint_node import (
    finite_feedback,
    load_config,
    plan_document,
)


@pytest.fixture
def session():
    """Return a short route with the same guarded hardware scheduler."""
    cfg = validate_config({'points_m_flat': [0.0, 0.0, 0.0, 0.10]})
    return ContinuousExecution(cfg, build_plan(cfg))


def observe(session, now, x=3.0, y=-2.0, yaw=0.0, frame='world'):
    """Use a deliberately unsynchronized source timestamp."""
    return session.observe(Feedback(x, y, yaw, 10000.0 + now, now, frame))


def ready(session):
    """Supply stationary feedback for longer than the readiness window."""
    for index in range(33):
        observe(session, index * 0.01)
    return 0.32


def arm(session):
    """Arm a fresh stationary session."""
    now = ready(session)
    assert session.start(now, subscribers=1, publishers=1)[0]
    return now


def test_no_reference_without_start_and_start_has_no_jump(session):
    """Idle never commands motion; capturing an origin is continuous."""
    now = ready(session)
    assert session.step(now) is None
    assert session.start(now, subscribers=1, publishers=1)[0]
    assert session.step(now) == pytest.approx((3.0, -2.0, 0.0, 0.0, 0.0, 0.0))
    assert session.translation == (3.0, -2.0)


@pytest.mark.parametrize(
    'subscribers,publishers', [(0, 1), (2, 1), (1, 0), (1, 2)]
)
def test_start_requires_reference_ownership(session, subscribers, publishers):
    """A missing controller or competing reference blocks arming."""
    now = ready(session)
    assert not session.start(
        now, subscribers=subscribers, publishers=publishers
    )[0]
    assert not session.active


def test_start_requires_fresh_stationary_pose(session):
    """A new sample alone and stale feedback never satisfy the start guard."""
    observe(session, 0.1)
    assert not session.start(0.1, subscribers=1, publishers=1)[0]
    now = ready(session)
    assert not session.start(now + 0.081, subscribers=1, publishers=1)[0]
    for index in range(34, 65):
        observe(session, index * 0.01, x=3.0 + 0.001 * (index - 33))
    assert not session.start(0.64, subscribers=1, publishers=1)[0]


def test_wrong_base_yaw_blocks_start(session):
    """Fixed turret yaw never substitutes for raw base start alignment."""
    for index in range(33):
        observe(session, index * 0.01, yaw=0.3)
    assert not session.start(0.32, subscribers=1, publishers=1)[0]


def test_absolute_mode_requires_actual_start_position():
    """The absolute alternative cannot silently jump from a captured origin."""
    cfg = validate_config({'origin_mode': 'vicon_absolute'})
    run = ContinuousExecution(cfg, build_plan(cfg))
    now = ready(run)
    assert not run.start(now, subscribers=1, publishers=1)[0]
    for index in range(34, 70):
        observe(run, index * 0.01, x=0.0, y=0.0)
    assert run.start(run.feedback.received_s, subscribers=1, publishers=1)[0]
    assert run.translation == (0.0, 0.0)


def test_callback_gap_aborts_without_catching_up(session):
    """A stalled process must not jump forward and resume a missed path."""
    now = arm(session)
    for index in range(1, 10):
        observe(session, now + index * 0.01)
    assert session.step(now + 0.09) is None
    assert session.state == 'aborted'
    for index in range(42, 75):
        observe(session, index * 0.01)
        assert session.step(index * 0.01) is None
    assert session.start(0.74, subscribers=1, publishers=1)[0]
    assert session.step(0.74)[3:] == (0.0, 0.0, 0.0)


def test_stale_odom_aborts_and_new_feedback_never_auto_restarts(session):
    """Feedback recovery alone cannot rearm a failed run."""
    now = arm(session)
    for dt in (0.02, 0.04, 0.06, 0.081):
        result = session.step(now + dt)
    assert result is None
    assert session.state == 'aborted'
    observe(session, now + 0.09)
    assert session.step(now + 0.09) is None


@pytest.mark.parametrize(
    'fault',
    ['nan', 'duplicate', 'backwards', 'frame', 'jump', 'yaw', 'source_gap'],
)
def test_invalid_odom_aborts(session, fault):
    """Malformed, out-of-order and discontinuous poses cause silence."""
    now = arm(session)
    x, y, yaw, stamp, frame = 3.0, -2.0, 0.0, 10000.0 + now + 0.01, 'world'
    if fault == 'nan':
        x = math.nan
    elif fault == 'duplicate':
        stamp = 10000.0 + now
    elif fault == 'backwards':
        stamp = 9999.0
    elif fault == 'frame':
        frame = 'other'
    elif fault == 'jump':
        x += 0.10
    elif fault == 'yaw':
        yaw = 0.4
    elif fault == 'source_gap':
        stamp += 0.10
    session.observe(Feedback(x, y, yaw, stamp, now + 0.01, frame))
    assert session.state == 'aborted'
    assert session.step(now + 0.01) is None


def test_new_publisher_mid_run_causes_latched_abort(session):
    """Ownership is checked throughout playback, not just at arming."""
    now = arm(session)
    assert session.step(now, publishers=2) is None
    assert session.state == 'aborted'
    assert session.step(now, publishers=1) is None


def test_stop_is_silence_and_requires_explicit_restart(session):
    """No zero-position reference is emitted as a stop command."""
    now = arm(session)
    session.stop()
    assert session.step(now) is None
    assert session.state == 'stopped'
    assert session.start(now, subscribers=1, publishers=1)[0]
    assert session.run_id == 2


def test_complete_one_shot_with_endpoint_hold_then_silence(session):
    """Motion uses world velocity; completion holds once and never loops."""
    now = arm(session)
    references = []
    states = []
    end = (
        now
        + session.cfg['startup_hold_s']
        + session.plan.duration
        + session.cfg['final_hold_s']
        + 0.1
    )
    step = 1
    while now + step * 0.01 < end:
        current = now + step * 0.01
        observe(session, current)
        value = session.step(current)
        references.append(value)
        states.append(session.state)
        step += 1
    assert session.state == 'complete'
    assert {'startup_hold', 'motion', 'final_hold', 'complete'} <= set(states)
    nonzero = [value for value in references if value and value[4] > 0]
    assert nonzero and all(value[3] == 0 for value in nonzero)
    endpoint = next(
        value
        for value, state in zip(references, states)
        if state == 'final_hold'
    )
    assert endpoint == pytest.approx((3.0, -1.9, 0.0, 0.0, 0.0, 0.0))
    assert session.step(end + 10) is None


@pytest.mark.parametrize(
    'overrides',
    [
        {'speed_m_s': float('nan')},
        {'speed_m_s': True},
        {'max_wheel_rate_rad_s': 3.0},
        {'planning_wheel_rate_fraction': 1.0},
        {'max_publish_gap_s': 0.12},
        {'odom_timeout_s': 0.12},
        {'points_m_flat': [0.0, 0.0, 1.0]},
        {'points_m_flat': [0.0, 0.0, True, 1.0]},
        {'origin_mode': 'body_rotated'},
        {'odom_min_z_m': 0.5},
    ],
)
def test_invalid_parameters_are_rejected(overrides):
    """Reject unsafe timing, hardware limits and ambiguous coordinates."""
    with pytest.raises(ValueError):
        validate_config(overrides)


def odom(z=0.30, quaternion=(0.0, 0.0, 0.0, 1.0)):
    """Build a ROS-independent pose-shaped object."""
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=z),
                orientation=SimpleNamespace(
                    **dict(zip(('x', 'y', 'z', 'w'), quaternion))
                ),
            )
        ),
        header=SimpleNamespace(
            frame_id='world', stamp=SimpleNamespace(sec=7, nanosec=0)
        ),
    )


@pytest.mark.parametrize(
    'message',
    [
        odom(z=0.2),
        odom(z=0.5),
        odom(quaternion=(0.0, 0.0, 0.0, 0.0)),
        odom(quaternion=(0.5, 0.0, 0.0, 0.8660254)),
        odom(z=math.nan),
    ],
)
def test_vicon_validity_matches_controller_height_and_tilt(message):
    """The planner aborts when the hardware controller would reject a pose."""
    with pytest.raises(ValueError):
        finite_feedback(message, 1.0, validate_config({}))


def test_normalized_vicon_quaternion_and_unsynchronized_stamp():
    """Receipt time controls freshness; valid nonunit quaternions normalize."""
    sample = finite_feedback(
        odom(quaternion=(0.0, 0.0, 0.0, 2.0)), 100.0, validate_config({})
    )
    assert sample == Feedback(1.0, 2.0, 0.0, 7.0, 100.0, 'world')


def test_installed_hardware_config_plan_has_headroom_and_explicit_placement():
    """Hardware planning retains conservative headroom."""
    cfg = load_config(
        Path(__file__).parents[1] / 'config/continuous_waypoint_hw.yaml'
    )
    plan = build_plan(cfg)
    report = plan_document(cfg, plan)
    assert cfg['speed_m_s'] == 0.15
    assert cfg['corner_deviation_m'] == 0.02
    assert (
        plan.metadata()['certification']['max_wheel_rate_rad_s']
        < cfg['max_wheel_rate_rad_s']
    )
    assert (
        plan.metadata()['certification']['max_wheel_acceleration_rad_s2']
        <= 2.0 + 1e-9
    )
    assert report['placement']['translation_m'] is None
    assert report['execution']['requires_explicit_start']


@pytest.mark.parametrize('kind', ['height_jump', 'roll_jump'])
def test_full_pose_jumps_match_hardware_controller_guard(session, kind):
    """Valid height and tilt can still have invalid intersample jumps."""
    now = arm(session)
    if kind == 'height_jump':
        sample = Feedback(
            3.0, -2.0, 0.0, 10000.0 + now + 0.01, now + 0.01, 'world', 0.40
        )
    else:
        session.feedback = Feedback(
            3.0,
            -2.0,
            0.0,
            10000.0 + now,
            now,
            'world',
            0.30,
            (math.sin(-0.15), 0.0, 0.0, math.cos(-0.15)),
        )
        sample = Feedback(
            3.0,
            -2.0,
            0.0,
            10000.0 + now + 0.01,
            now + 0.01,
            'world',
            0.30,
            (math.sin(0.15), 0.0, 0.0, math.cos(0.15)),
        )
    session.observe(sample)
    assert session.state == 'aborted'
    assert session.step(now + 0.01) is None


@pytest.mark.parametrize('stamp', [0.0, -1.0])
def test_source_stamps_must_be_positive(session, stamp):
    """Uninitialized or negative stamps cannot establish readiness."""
    assert not session.observe(Feedback(0.0, 0.0, 0.0, stamp, 1.0, 'world'))
    assert session.feedback is None


def test_overflowing_quaternion_is_invalid():
    """Finite components must not normalize through an infinite norm."""
    with pytest.raises(ValueError):
        finite_feedback(
            odom(quaternion=(1e308, 0.0, 0.0, 1e308)), 1.0, validate_config({})
        )


@pytest.mark.parametrize(
    'sec,nanosec', [(0, 0), (-1, 1), (1, -1), (1, 1000000000)]
)
def test_malformed_ros_stamp_is_rejected(sec, nanosec):
    """Preserve the source timestamp contract at the ROS adapter boundary."""
    msg = odom()
    msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nanosec
    with pytest.raises(ValueError):
        finite_feedback(msg, 1.0, validate_config({}))


def test_default_preview_uses_shipped_yaml_without_ros(tmp_path, monkeypatch):
    """Default previews match the launch YAML and never initialize ROS."""
    import json
    import sys

    from reference_trajectory.continuous_waypoint_node import (
        default_config_path,
        main,
    )

    monkeypatch.setitem(sys.modules, 'rclpy', None)
    output = tmp_path / 'plan.json'
    main(['--plan-only', str(output)])
    report = json.loads(output.read_text())
    assert report['config'] == load_config(default_config_path())
    assert report['placement']['translation_m'] is None
    assert report['execution']['requires_explicit_start']


def test_recovery_waits_for_controller_sample_count_at_low_rate(session):
    """Elapsed stability cannot bypass the controller's ten-sample recovery."""
    for index in range(7):
        observe(session, index * 0.05)
    assert session.start_ready_since == pytest.approx(0.05)
    assert not session.start(0.30, subscribers=1, publishers=1)[0]
    for index in range(7, 10):
        observe(session, index * 0.05)
    assert session.start(0.45, subscribers=1, publishers=1)[0]
    session.invalidate_feedback('Test invalid sample')
    assert session.recovery_samples == 0
    for index in range(10, 19):
        observe(session, index * 0.05)
    assert session.recovery_samples == 9
    assert not session.start(0.9, subscribers=1, publishers=1)[0]
    observe(session, 0.95)
    assert session.start(0.95, subscribers=1, publishers=1)[0]


@pytest.mark.parametrize('count', [9, 0, 10.0, True, 1001])
def test_recovery_count_cannot_undercut_controller_or_use_float(count):
    """Preserve the ten-sample hardware controller recovery contract."""
    with pytest.raises(ValueError):
        validate_config({'odom_recovery_samples': count})
