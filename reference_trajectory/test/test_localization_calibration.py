"""Verify calibration excitation, continuity, holds and hardware contracts."""

import json
import math
import sys
from pathlib import Path

import numpy as np

import pytest

from reference_trajectory.continuous_execution import (
    ContinuousExecution,
    Feedback,
)
from reference_trajectory.localization_calibration import (
    CalibrationPlan,
    default_config_path,
    load_config,
    main,
    validate_config,
    validate_runtime,
)

import yaml


@pytest.fixture(scope='module')
def cfg():
    """Load the shipped hardware calibration profile."""
    return load_config(default_config_path())


@pytest.fixture(scope='module')
def plan(cfg):
    """Share one immutable plan across contract checks."""
    return CalibrationPlan(cfg)


def test_both_turn_directions_and_reversals_at_each_speed(cfg, plan):
    """Check useful excitation, closure, and the expected run duration."""
    motion = [s for s in plan.stages if s['plan'] is not None]
    assert [s['name'] for s in motion] == [
        'fast_forward',
        'fast_reverse',
        'fast_ccw',
        'fast_cw',
        'slow_forward',
        'slow_reverse',
        'slow_ccw',
        'slow_cw',
    ]
    assert len(plan.stages) == 15
    for stage in motion:
        segment = stage['plan']
        t = np.linspace(0, segment.duration, 401)
        states = np.array([segment.sample(float(x)) for x in t])
        if stage['name'].endswith('forward'):
            assert np.all(states[:, 3] >= 0) and max(states[:, 3]) > 0.09
        elif stage['name'].endswith('reverse'):
            assert np.all(states[:, 3] <= 0) and min(states[:, 3]) < -0.09
        else:
            pts = np.array(segment.points)
            area = np.sum(pts[:-1, 0] * pts[1:, 1] - pts[1:, 0] * pts[:-1, 1])
            assert (area > 0) == stage['name'].endswith('_ccw')
    assert plan.points[0] == plan.points[-1] == (0.0, 0.0)
    assert plan.metadata()['total_duration_with_holds_s'] == pytest.approx(
        345.2283966975281
    )


def test_every_join_is_continuous_and_dwell_velocities_are_zero(plan):
    """Prevent reference jumps at motion/hold boundaries."""
    for stage in plan.stages:
        t = stage['start_time_s']
        before = plan.sample_with_acceleration(t - 1e-7)
        after = plan.sample_with_acceleration(t + 1e-7)
        np.testing.assert_allclose(before[:6], after[:6], atol=1e-6)
        if stage['plan'] is None:
            a = plan.sample_with_acceleration(t + 0.01)
            b = plan.sample_with_acceleration(stage['end_time_s'] - 0.01)
            assert a[:2] == b[:2] == stage['hold_position']
            assert a[2:6] == b[2:6] == (0.0, 0.0, 0.0, 0.0)


def test_bounds_and_position_velocity_consistency(cfg, plan):
    """Compare analytic feedforward to position derivatives and limits."""
    for stage in plan.stages:
        if stage['plan'] is None:
            continue
        segment = stage['plan']
        for t in np.linspace(0.01, segment.duration - 0.01, 110):
            dt = 1e-4
            a = np.array(segment.sample_with_jerk(t - dt))
            b = np.array(segment.sample_with_jerk(t + dt))
            mid = np.array(segment.sample_with_jerk(t))
            np.testing.assert_allclose(
                (b[:2] - a[:2]) / (2 * dt), mid[2:4], atol=1e-5
            )
            assert np.linalg.norm(mid[2:4]) <= 0.15 + 1e-9
            assert np.linalg.norm(mid[4:6]) <= 0.10 + 1e-8
            assert np.linalg.norm(mid[6:8]) <= 0.20 + 1e-8
            assert -1.5 - 1e-8 <= mid[0] <= 1e-8
            assert -1e-8 <= mid[1] <= 2.0 + 1e-8
    bounds = plan.metadata()['certification']
    assert bounds['max_wheel_rate_rad_s'] < cfg['max_wheel_rate_rad_s']
    assert bounds['max_wheel_acceleration_rad_s2'] <= 2.0 + 1e-9


@pytest.mark.parametrize('speed_order', [['fast'], ['slow'], ['fast', 'slow']])
def test_full_execution_waits_ten_seconds_and_holds_end(cfg, speed_order):
    """Drive the complete execution clock with translated feedback."""
    cfg = validate_config(dict(cfg, speed_order=speed_order))
    plan = CalibrationPlan(cfg)
    session = ContinuousExecution(cfg, plan)
    for k in range(40):
        t = k * 0.01
        session.observe(Feedback(3.0, -2.0, 0.0, t + 1000.0, t, 'mocap'))
    assert session.start(0.39, subscribers=1, publishers=1)[0]
    origin = np.array([3.0, -2.0])
    seen = set()
    for k in range(1, math.ceil((20 + plan.duration + 0.1) / 0.01)):
        elapsed = k * 0.01
        now = 0.39 + elapsed
        reference = plan.sample(max(0.0, elapsed - 10.0))
        xy = origin + np.array(reference[:2])
        session.observe(Feedback(*xy, 0.0, now + 1000.0, now, 'mocap'))
        sample = session.step(now)
        seen.add(session.state)
        if elapsed < 10:
            assert sample == pytest.approx((3.0, -2.0, 0.0, 0.0, 0.0, 0.0))
        if session.state == 'final_hold':
            assert sample == pytest.approx((3.0, -2.0, 0.0, 0.0, 0.0, 0.0))
    assert seen >= {'startup_hold', 'motion', 'final_hold', 'complete'}
    assert session.step(1000.0) is None


@pytest.fixture
def armed_session(cfg, plan):
    """Start the calibration with stationary, valid feedback."""
    session = ContinuousExecution(cfg, plan)
    for k in range(40):
        t = k * 0.01
        session.observe(Feedback(3.0, -2.0, 0.0, t + 1000.0, t, 'mocap'))
    assert session.start(0.39, subscribers=1, publishers=1)[0]
    return session


@pytest.mark.parametrize('gap_s', [0.097405, 0.117145, 0.15, 0.199])
def test_vicon_gaps_below_limit_are_tolerated(armed_session, gap_s):
    """Allow delivery gaps below 200 ms while callbacks stay timely."""
    session = armed_session
    before = session.feedback
    for k in range(1, math.ceil(gap_s / 0.01)):
        assert session.step(before.received_s + k * 0.01) is not None
    now = before.received_s + gap_s
    assert session.observe(
        Feedback(3.0, -2.0, 0.0, now + 1000.0, now, 'mocap')
    )
    assert session.step(now) is not None
    assert session.active


@pytest.mark.parametrize('fault', ['age', 'receipt', 'source', 'callback'])
def test_calibration_still_aborts_on_longer_gaps(armed_session, fault):
    """Retain the 200 ms feedback and independent 80 ms callback guards."""
    session = armed_session
    before = session.feedback
    t = before.received_s
    if fault == 'age':
        for delta in (0.04, 0.08, 0.12, 0.16, 0.199):
            assert session.step(t + delta) is not None
        assert session.step(t + 0.201) is None
    elif fault == 'callback':
        for k in range(1, 10):
            now = t + k * 0.01
            session.observe(Feedback(3.0, -2.0, 0.0, now + 1000.0, now, 'mocap'))
        assert session.step(t + 0.09) is None
        assert session.reason.startswith('Reference callback gap')
    else:
        receive_delta = 0.201 if fault == 'receipt' else 0.01
        assert not session.observe(
            Feedback(3.0, -2.0, 0.0, before.source_stamp_s + 0.201,
                     t + receive_delta, 'mocap')
        )
    assert session.state == 'aborted'


@pytest.mark.parametrize(
    'bad',
    [
        {'slow_speed_m_s': 0.15},
        {'speed_m_s': 0.2},
        {'startup_hold_s': 1.0},
        {'final_hold_s': 1.0},
        {'loop_width_m': 0.0},
        {'straight_distance_m': True},
        {'loop_height_m': math.inf},
        {'segment_hold_s': 20.0},
        {'odom_topic': '/local_HAMR/odom'},
        {'origin_mode': 'vicon_absolute'},
        {'odom_timeout_s': 0.201},
        {'max_publish_gap_s': 0.12},
        {'speed_order': []},
        {'speed_order': 'fast'},
        {'speed_order': ['fast', 'fast']},
        {'speed_order': ['medium']},
        {'speed_order': ['slow', 1]},
    ],
)
def test_invalid_calibration_configuration(cfg, bad):
    """Reject profiles that invalidate calibration or hardware limits."""
    with pytest.raises(ValueError):
        validate_config(dict(cfg, **bad))


def test_preview_never_initializes_ros(tmp_path, monkeypatch):
    """Make preview useful even when ROS cannot be imported."""
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    output = tmp_path / 'plan.json'
    main(['--plan-only', str(output)])
    doc = json.loads(output.read_text())
    assert (
        doc['config']['startup_hold_s']
        == doc['config']['final_hold_s']
        == 10.0
    )
    assert doc['placement']['translation_m'] is None
    assert len(doc['continuous_plan']['stages']) == 15
    assert doc['continuous_plan']['stages'][0]['start_from_run_s'] == 10.0


@pytest.mark.parametrize('speed_set', ['fast', 'slow', 'both'])
def test_selected_speed_preview_is_complete_and_recorded(
    tmp_path, monkeypatch, speed_set
):
    """Check CLI selection, both turn directions, holds and provenance."""
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    output = tmp_path / 'selected.json'
    main(['--speed-set', speed_set, '--plan-only', str(output)])
    doc = json.loads(output.read_text())
    order = ['fast', 'slow'] if speed_set == 'both' else [speed_set]
    assert doc['config']['speed_order'] == order
    assert doc['continuous_plan']['speed_order'] == order
    motion = [
        stage
        for stage in doc['continuous_plan']['stages']
        if stage['kind'] == 'motion'
    ]
    assert [stage['name'] for stage in motion] == [
        f'{speed}_{leg}'
        for speed in order
        for leg in ('forward', 'reverse', 'ccw', 'cw')
    ]
    for stage in motion:
        expected_speed = 0.15 if stage['name'].startswith('fast') else 0.10
        assert stage['requested_speed_m_s'] == expected_speed
    assert len(doc['continuous_plan']['stages']) == len(motion) * 2 - 1
    assert doc['continuous_plan']['points_m'][0] == [0.0, 0.0]
    assert doc['continuous_plan']['points_m'][-1] == [0.0, 0.0]
    assert (
        doc['config']['startup_hold_s']
        == doc['config']['final_hold_s']
        == 10.0
    )
    assert doc['execution']['loop'] is False
    assert doc['config']['odom_timeout_s'] == 0.20
    assert doc['config']['max_publish_gap_s'] == 0.08


def test_legacy_slow_first_profile_is_still_supported(cfg):
    """Allow explicit reproduction of the original speed order."""
    cfg = validate_config(dict(cfg, speed_order=['slow', 'fast']))
    plan = CalibrationPlan(cfg)
    assert plan.stages[0]['name'] == 'slow_forward'
    assert plan.stages[-1]['name'] == 'fast_cw'


def runtime_snapshot(odom_timeout_s=0.20):
    """Read hardware defaults with the calibration launch timeout override."""
    repo = Path(__file__).resolve().parents[2]
    controller = yaml.safe_load(
        (repo / 'hamr_bringup/config/hamr_hw_control_params.yaml').read_text()
    )['/hamr_controller_node']['ros__parameters']
    controller['odom_timeout_s'] = odom_timeout_s
    ekf = yaml.safe_load(
        (repo / 'hamr_bringup/config/ekf_calibrated.yaml').read_text()
    )['/**']['ros__parameters']
    return {'hamr_controller_node': controller, 'ekf_filter_node': ekf}


def test_current_hardware_parameters_match_plan(cfg):
    """Allow hardware with the matching 200 ms Vicon timeout override."""
    validate_runtime(cfg, runtime_snapshot())


def test_default_controller_needs_matching_timeout_override(cfg):
    """Catch a launch left at 120 ms before arming a 200 ms trajectory."""
    with pytest.raises(ValueError, match='controller_odom_timeout_s:=0.20'):
        validate_runtime(cfg, runtime_snapshot(odom_timeout_s=0.12))
    # Existing 120 ms calibration profiles still work with normal hardware.
    validate_runtime(dict(cfg, odom_timeout_s=0.12),
                     runtime_snapshot(odom_timeout_s=0.12))


@pytest.mark.parametrize(
    'key,value',
    [
        ('r_wheel', 0.125),
        ('mode', 'manual'),
        ('simulating', True),
        ('vicon_pose_guard_enabled', False),
        ('odom_timeout_s', 0.0),
        ('odom_timeout_s', 0.10),
        ('odom_timeout_s', 0.21),
        ('reference_timeout_s', 0.20),
        ('xy_velocity_source', 'odom_twist_body'),
    ],
)
def test_runtime_mismatch_prevents_calibration(cfg, key, value):
    """Reject a running controller inconsistent with planning assumptions."""
    snapshot = runtime_snapshot()
    snapshot['hamr_controller_node'][key] = value
    with pytest.raises(ValueError):
        validate_runtime(cfg, snapshot)


def test_external_pose_must_not_be_fused_into_evaluated_filter(cfg):
    """Keep the scoring reference independent of filter measurements."""
    snapshot = runtime_snapshot()
    snapshot['ekf_filter_node']['pose0'] = '/HAMR_base/pose'
    with pytest.raises(ValueError):
        validate_runtime(cfg, snapshot)


def test_recorder_captures_calibration_metadata_and_status():
    """Ensure late recorder discovery still captures the reference plan."""
    repo = Path(__file__).resolve().parents[2]
    recorder = (repo / 'hamr_bringup/scripts/record_hamr_test_bag').read_text()
    assert '^/localization_calibration/(metadata|status|path)$' in recorder
    qos = yaml.safe_load(
        (repo / 'hamr_bringup/config/record_qos.yaml').read_text()
    )
    for suffix in ('metadata', 'status', 'path'):
        assert (
            qos[f'/localization_calibration/{suffix}']['durability']
            == 'transient_local'
        )
