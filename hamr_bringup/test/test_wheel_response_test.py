"""Focused safety regressions for the direct wheel-response harness."""

import ast
from importlib.machinery import SourceFileLoader
import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'wheel_response_test'
SPEC = importlib.util.spec_from_loader(
    'wheel_response_test',
    SourceFileLoader('wheel_response_test', str(SCRIPT)),
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_odom(
    position=(0.0, 0.0, 0.32),
    quaternion=(0.0, 0.0, 0.0, 1.0),
    stamp=(9, 950_000_000),
):
    msg = MODULE.Odometry()
    msg.header.stamp.sec, msg.header.stamp.nanosec = stamp
    (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
    ) = position
    (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    ) = quaternion
    return msg


def make_pose_guard_node(now_ns=10_000_000_000):
    node = object.__new__(MODULE.WheelResponseTest)
    node.last_odom_time = None
    node.last_odom_source_stamp_ns = None
    node.pose = None
    node.latched_odom_guard_error = None
    node.vicon_pose_guard = MODULE.ViconPosePlausibilityGuard(
        MODULE.VICON_MAX_POSITION_STEP_M,
        MODULE.VICON_MAX_ORIENTATION_STEP_RAD,
        1,
        MODULE.VICON_MIN_Z_M,
        MODULE.VICON_MAX_Z_M,
        MODULE.VICON_MAX_TILT_RAD,
    )
    node.last_raw_yaw = None
    node.unwrapped_yaw = None
    node.odom_samples = []
    node.total_path_m = 0.0
    node.last_path_pose = None
    node.get_clock = lambda: type('Clock', (), {
        'now': lambda self: type('Now', (), {
            'nanoseconds': now_ns,
        })(),
    })()
    node.trim_samples = lambda _now: None
    return node


def make_status_row(timestamp, **overrides):
    values = {field: 0.0 for field in MODULE.CONTROLLER_STATUS_FIELDS}
    values.update(overrides)
    return (
        float(timestamp),
        *(float(values[field]) for field in MODULE.CONTROLLER_STATUS_FIELDS),
    )


def make_diag_row(
    timestamp,
    left_legacy=0,
    left_quadrature=0,
    right_legacy=0,
    right_quadrature=0,
    valid_positive=0,
    valid_negative=0,
    invalid_transitions=0,
):
    def wheel(legacy, quadrature):
        return (
            legacy,
            quadrature,
            valid_positive,
            valid_positive,
            valid_positive,
            valid_positive,
            valid_positive,
            valid_negative,
            invalid_transitions,
            0,
            0,
        )

    return (
        float(timestamp),
        *wheel(left_legacy, left_quadrature),
        *wheel(right_legacy, right_quadrature),
    )


def make_valid_firmware_pid_config():
    node = object.__new__(MODULE.WheelResponseTest)
    node.enable = True
    node.profile = MODULE.FIRMWARE_PID_STRAIGHT_PROFILE
    node.encoder_only_mode = True
    node.allow_encoder_only = True
    node.level = 1.0
    node.repeat_count = 1
    node.hold_s = 1.0
    node.measure_s = 0.5
    node.ramp_s = 0.5
    node.zero_s = 0.5
    node.start_delay_s = 0.0
    node.rate_hz = 50.0
    node.max_abs_wheel_cmd = MODULE.FIRMWARE_PID_MAX_LEVEL_RAD_S
    node.stale_odom_s = 0.35
    node.stale_controller_status_s = 0.20
    node.stale_encoder_diag_s = 0.20
    node.startup_timeout_s = 5.0
    node.max_trial_path_m = 0.5
    node.max_trial_yaw_rad = 1.0
    node.max_total_displacement_m = 1.0
    node.max_total_path_m = 3.0
    node.r_wheel = 0.122
    node.a_wheel = 0.350
    node.vicon_source_stamp_policy = MODULE.VICON_SOURCE_STAMP_POLICY
    return node


def test_preflight_publishes_only_zero_heartbeats_until_inputs_are_ready(
    monkeypatch,
):
    node = object.__new__(MODULE.WheelResponseTest)
    node.startup_timeout_s = 5.0
    node.latched_odom_guard_error = None
    commands = []
    readiness_checks = []

    node.publish_cmd = lambda left, right: commands.append((left, right))
    node.assert_ready = lambda: readiness_checks.append(True)
    node.assert_zero_heartbeat_acknowledged = lambda: None
    monkeypatch.setattr(MODULE.time, 'monotonic', iter((0.0, 0.1)).__next__)
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(MODULE.rclpy, 'spin_once', lambda *_args, **_kwargs: None)

    node.wait_for_inputs()

    assert commands == [(0.0, 0.0)]
    assert readiness_checks == [True]


def test_odom_monitor_qos_and_pose_guard_limits_are_fixed():
    qos = MODULE.ODOM_MONITOR_QOS
    assert qos.depth == 1
    assert qos.history == MODULE.HistoryPolicy.KEEP_LAST
    assert qos.reliability == MODULE.ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == MODULE.DurabilityPolicy.VOLATILE

    tree = ast.parse(SCRIPT.read_text(encoding='utf-8'))
    odom_subscriptions = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == 'create_subscription'
        and len(call.args) == 4
        and isinstance(call.args[2], ast.Attribute)
        and call.args[2].attr == 'odom_cb'
    ]
    assert len(odom_subscriptions) == 1
    assert isinstance(odom_subscriptions[0].args[3], ast.Name)
    assert odom_subscriptions[0].args[3].id == 'ODOM_MONITOR_QOS'
    policy_declarations = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == 'declare_parameter'
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == 'vicon_source_stamp_policy'
    ]
    assert len(policy_declarations) == 1
    assert ast.unparse(policy_declarations[0].args[1]) == (
        'VICON_SOURCE_STAMP_POLICY'
    )

    node = make_pose_guard_node()
    guard = node.vicon_pose_guard
    assert guard.max_position_jump_m == 0.08
    assert guard.max_orientation_jump_rad == 0.35
    assert guard.min_z_m == 0.25
    assert guard.max_z_m == 0.40
    assert guard.max_tilt_rad == 0.35
    assert MODULE.VICON_SOURCE_STAMP_POLICY == 'receipt_monotonic'
    assert MODULE.validated_vicon_source_stamp_policy(
        ' RECEIPT_MONOTONIC '
    ) == MODULE.VICON_SOURCE_STAMP_POLICY


@pytest.mark.parametrize(
    ('stamp', 'reason'),
    (
        ((0, 0), 'zero header timestamp'),
        ((-1, 0), 'invalid header timestamp'),
        ((9, 1_000_000_000), 'invalid header timestamp'),
    ),
)
def test_invalid_source_stamp_latches_before_refresh(stamp, reason):
    node = make_pose_guard_node()
    node.odom_cb(make_odom(stamp=stamp))

    assert reason in node.latched_odom_guard_error
    assert node.pose is None
    assert node.last_odom_time is None
    assert node.last_odom_source_stamp_ns is None
    assert node.odom_samples == []

    # A following valid sample cannot recover or establish a baseline in this
    # one-shot process.
    latched_error = node.latched_odom_guard_error
    node.odom_cb(make_odom())
    assert node.latched_odom_guard_error == latched_error
    assert node.pose is None


def test_absolute_clock_offset_does_not_affect_fully_valid_preflight_sample(
    monkeypatch,
):
    node = make_pose_guard_node()
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: 123.0)

    # A source clock behind the receiver may establish the first pose.
    node.odom_cb(make_odom(stamp=(1, 0)))
    assert node.latched_odom_guard_error is None
    assert node.pose == pytest.approx((0.0, 0.0, 0.0))
    assert node.last_odom_time == 123.0
    assert node.last_odom_source_stamp_ns == 1_000_000_000

    # A source clock far ahead of the receiver is equally valid; freshness is
    # enforced only by the local callback-receipt watchdog.
    future_node = make_pose_guard_node()
    future_node.odom_cb(make_odom(stamp=(5000, 0)))
    assert future_node.latched_odom_guard_error is None
    assert future_node.pose is not None
    assert future_node.last_odom_source_stamp_ns == 5_000_000_000_000


@pytest.mark.parametrize('stamp', ((9, 950_000_000), (9, 940_000_000)))
def test_duplicate_or_reversed_source_stamp_latches_without_refresh(
    monkeypatch,
    stamp,
):
    node = make_pose_guard_node()
    monotonic_now = [100.0]
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: monotonic_now[0])
    node.odom_cb(make_odom(stamp=(9, 950_000_000)))
    accepted_time = node.last_odom_time
    accepted_pose = node.pose

    monotonic_now[0] = 100.1
    node.odom_cb(make_odom(position=(0.01, 0.0, 0.32), stamp=stamp))

    assert 'non-monotonic source timestamp' in node.latched_odom_guard_error
    assert node.last_odom_time == accepted_time
    assert node.pose == accepted_pose
    assert node.last_odom_source_stamp_ns == 9_950_000_000


def test_pose_guard_boundaries_are_inclusive_normalized_and_yaw_invariant():
    guard = MODULE.ViconPosePlausibilityGuard(
        0.08, 0.35, 1, 0.25, 0.40, 0.35
    )
    yaw = 1.2
    tilt = 0.35
    cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    ct, st = math.cos(0.5 * tilt), math.sin(0.5 * tilt)
    scaled_yaw_tilt = tuple(
        3.0 * value for value in (cy * st, sy * st, sy * ct, cy * ct)
    )
    assert guard.observe((0.0, 0.0, 0.25), scaled_yaw_tilt).accepted
    assert math.sqrt(sum(q * q for q in guard.last_quaternion)) == pytest.approx(1.0)

    upper_guard = MODULE.ViconPosePlausibilityGuard(
        0.08, 0.35, 1, 0.25, 0.40, 0.35
    )
    assert upper_guard.observe((0.0, 0.0, 0.40), (0.0, 0.0, 0.0, 2.0)).accepted

    step_guard = MODULE.ViconPosePlausibilityGuard(
        0.08, 0.35, 1, 0.25, 0.40, 0.35
    )
    assert step_guard.observe((0.0, 0.0, 0.32), (0.0, 0.0, 0.0, 1.0)).accepted
    half_angle = 0.5 * 0.35
    boundary_q = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))
    result = step_guard.observe((0.048, 0.0, 0.384), boundary_q)
    assert result.accepted
    assert result.position_jump_m == pytest.approx(0.08)
    assert result.orientation_jump_rad == pytest.approx(0.35)
    sign_flipped = tuple(-value for value in boundary_q)
    result = step_guard.observe((0.048, 0.0, 0.384), sign_flipped)
    assert result.accepted
    assert result.orientation_jump_rad == pytest.approx(0.0)


def test_observed_pose_jump_latches_stops_and_never_recovers(monkeypatch):
    node = make_pose_guard_node()
    monotonic_now = [100.0]
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: monotonic_now[0])

    node.odom_cb(make_odom())
    accepted_state = (
        node.pose,
        node.last_odom_time,
        tuple(node.odom_samples),
        node.total_path_m,
        node.last_path_pose,
        node.last_raw_yaw,
        node.vicon_pose_guard.last_position,
        node.vicon_pose_guard.last_quaternion,
    )

    half_angle = 0.5 * 0.575
    observed_outlier = make_odom(
        position=(0.142, 0.0, 0.32),
        quaternion=(0.0, 0.0, math.sin(half_angle), math.cos(half_angle)),
        stamp=(9, 960_000_000),
    )
    monotonic_now[0] = 100.1
    node.odom_cb(observed_outlier)

    assert '3-D position jump' in node.latched_odom_guard_error
    assert 'quaternion rotation jump' in node.latched_odom_guard_error
    assert '3-D step=0.142 m' in node.latched_odom_guard_error
    assert 'quaternion angle=0.575 rad' in node.latched_odom_guard_error
    assert (
        node.pose,
        node.last_odom_time,
        tuple(node.odom_samples),
        node.total_path_m,
        node.last_path_pose,
        node.last_raw_yaw,
        node.vicon_pose_guard.last_position,
        node.vicon_pose_guard.last_quaternion,
    ) == accepted_state

    latched_error = node.latched_odom_guard_error
    node.odom_cb(make_odom(stamp=(9, 970_000_000)))
    assert node.latched_odom_guard_error == latched_error
    with pytest.raises(RuntimeError, match='Vicon guard is latched'):
        node.assert_ready()

    commands = []
    node.rate_hz = 50.0
    node.publish_cmd = lambda left, right: commands.append((left, right))
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(MODULE.rclpy, 'spin_once', lambda *_args, **_kwargs: None)
    monotonic_now[0] = 101.0
    with pytest.raises(RuntimeError, match='Vicon guard is latched'):
        node.run_for(0.5, lambda _elapsed: (1.0, 0.5))
    assert commands == [(0.0, 0.0)]


def test_preflight_requires_firmware_acknowledgement_of_uart_zero():
    node = object.__new__(MODULE.WheelResponseTest)
    row = [0.0] * (1 + len(MODULE.CONTROLLER_STATUS_FIELDS))
    source_offset = 1 + MODULE.CONTROLLER_STATUS_FIELDS.index('source')

    row[source_offset] = 5.0
    node.controller_status_samples = [tuple(row)]
    node.assert_zero_heartbeat_acknowledged()

    for rejected_source in (6.0, 7.0):
        row[source_offset] = rejected_source
        node.controller_status_samples = [tuple(row)]
        with pytest.raises(RuntimeError, match='Motion remains disabled'):
            node.assert_zero_heartbeat_acknowledged()


def test_active_motion_rejects_firmware_stop_and_external_sources():
    node = object.__new__(MODULE.WheelResponseTest)
    row = [0.0] * (1 + len(MODULE.CONTROLLER_STATUS_FIELDS))
    source_offset = 1 + MODULE.CONTROLLER_STATUS_FIELDS.index('source')

    for accepted_source in (1.0, 5.0):
        row[source_offset] = accepted_source
        node.controller_status_samples = [tuple(row)]
        node.assert_active_controller_source_safe()

    for rejected_source in (0.0, 2.0, 3.0, 4.0, 6.0, 7.0):
        row[source_offset] = rejected_source
        node.controller_status_samples = [tuple(row)]
        with pytest.raises(RuntimeError, match='Safety stop'):
            node.assert_active_controller_source_safe()


def test_firmware_pid_plan_is_one_equal_forward_trial_with_hard_command_cap():
    level = MODULE.FIRMWARE_PID_MAX_LEVEL_RAD_S
    assert MODULE.FIRMWARE_PID_V020_LEVEL_RAD_S == pytest.approx(
        1.639344262295082
    )
    assert MODULE.FIRMWARE_PID_V020_LEVEL_RAD_S < level
    assert level == pytest.approx(1.65)
    assert MODULE.FIRMWARE_PID_MAX_WHEEL_TICKS == 1200
    assert MODULE.build_plan(MODULE.FIRMWARE_PID_STRAIGHT_PROFILE, level) == [
        ('equal_positive_pid', level, level),
    ]
    assert MODULE.require_firmware_pid_straight_command(level, level) == (
        level,
        level,
    )

    for command in (
        (-0.1, -0.1),
        (0.5, 0.4),
        (level + 0.01, level + 0.01),
        (float('nan'), float('nan')),
    ):
        with pytest.raises(RuntimeError):
            MODULE.require_firmware_pid_straight_command(*command)


def test_firmware_pid_tick_bounds_are_inclusive_and_fail_closed():
    assert MODULE.require_firmware_pid_tick_travel(
        100, 200, 1300, 1280
    ) == (1200.0, 1080.0)

    for ticks in (
        (100, 200, 99, 200),
        (100, 200, 1301, 1280),
        (100, 200, 500, 479),
        (100, 200, float('inf'), 200),
    ):
        with pytest.raises(RuntimeError, match='Encoder|encoder|tick'):
            MODULE.require_firmware_pid_tick_travel(*ticks)


def test_firmware_pid_profile_requires_explicit_opt_in_and_fixed_envelope():
    node = make_valid_firmware_pid_config()
    plan = MODULE.build_plan(node.profile, node.level)
    node.validate(plan)

    node.allow_encoder_only = False
    with pytest.raises(RuntimeError, match='two explicit opt-ins'):
        node.validate(plan)

    node = make_valid_firmware_pid_config()
    node.repeat_count = 2
    with pytest.raises(RuntimeError, match='exactly one trial'):
        node.validate(plan * 2)

    for attribute, rejected in (
        ('level', MODULE.FIRMWARE_PID_MAX_LEVEL_RAD_S + 0.01),
        ('ramp_s', MODULE.FIRMWARE_PID_MAX_RAMP_S + 0.01),
        ('hold_s', MODULE.FIRMWARE_PID_MIN_HOLD_S - 0.01),
        ('hold_s', MODULE.FIRMWARE_PID_MAX_HOLD_S + 0.01),
        ('rate_hz', MODULE.FIRMWARE_PID_MIN_RATE_HZ - 0.01),
        (
            'stale_controller_status_s',
            MODULE.FIRMWARE_PID_MAX_TELEMETRY_AGE_S + 0.01,
        ),
    ):
        node = make_valid_firmware_pid_config()
        setattr(node, attribute, rejected)
        rejected_plan = MODULE.build_plan(node.profile, node.level)
        with pytest.raises(RuntimeError):
            node.validate(rejected_plan)


def test_firmware_pid_v020_dry_run_validates_without_motion():
    node = make_valid_firmware_pid_config()
    node.enable = False
    node.allow_encoder_only = False
    node.level = MODULE.FIRMWARE_PID_V020_LEVEL_RAD_S
    node.csv_path = Path('/tmp/firmware_pid_v020_dry_run.csv')

    class Logger:
        def warn(self, _message):
            pass

        def info(self, _message):
            pass

    node.get_logger = Logger
    node.wait_for_inputs = lambda: pytest.fail(
        'dry run must return before hardware preflight'
    )

    assert node.run() == 0


def test_encoder_only_readiness_does_not_depend_on_vicon(monkeypatch):
    node = make_pose_guard_node()
    node.encoder_only_mode = True
    node.odom_cb(make_odom(stamp=(0, 0)))
    assert node.pose is None
    assert node.latched_odom_guard_error is None

    now = 10.0
    node.left_tick_samples = [(9.95, 100.0)]
    node.right_tick_samples = [(9.95, 100.0)]
    node.last_left_tick_time = 9.95
    node.last_right_tick_time = 9.95
    node.controller_status_samples = [make_status_row(9.95, source=5)]
    node.last_controller_status_time = 9.95
    node.encoder_diag_samples = [make_diag_row(9.95)]
    node.last_encoder_diag_time = 9.95
    node.controller_status_schema_error = None
    node.encoder_diag_schema_error = None
    node.stale_controller_status_s = 0.20
    node.stale_encoder_diag_s = 0.20
    node.left_topic = '/left'
    node.right_topic = '/right'
    node.turret_topic = '/turret'
    node.count_publishers = lambda _topic: 1
    subscriber = type('Publisher', (), {
        'get_subscription_count': lambda self: 1,
    })()
    node.pub_left = subscriber
    node.pub_right = subscriber
    node.pub_turret = subscriber
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: now)

    node.assert_ready()
    node.assert_zero_heartbeat_acknowledged()


def test_firmware_pid_status_guard_rejects_source_saturation_and_reverse():
    node = object.__new__(MODULE.WheelResponseTest)
    node.firmware_pid_motion_start_time = 10.0
    good_early = make_status_row(10.05, source=5)
    good_active = make_status_row(
        10.25,
        left_target_rpm=10.0,
        right_target_rpm=10.0,
        left_measured_rpm=9.0,
        right_measured_rpm=9.0,
        left_ff_pwm=100.0,
        right_ff_pwm=100.0,
        left_pid_pwm=5.0,
        right_pid_pwm=5.0,
        left_output_pwm=105.0,
        right_output_pwm=105.0,
        source=1,
    )
    node.controller_status_samples = [good_early, good_active]
    node.assert_firmware_pid_status_safe(10.40)

    rejected = (
        make_status_row(10.25, source=6),
        make_status_row(10.25, source=1, left_saturated=1),
        make_status_row(
            10.25, source=1, left_target_rpm=-1, right_target_rpm=-1
        ),
        make_status_row(
            10.25, source=1, left_target_rpm=1, right_target_rpm=2
        ),
        make_status_row(
            10.40,
            source=1,
            left_target_rpm=1,
            right_target_rpm=1,
            left_output_pwm=-1,
        ),
        make_status_row(
            10.40,
            source=1,
            left_target_rpm=1,
            right_target_rpm=1,
            left_measured_rpm=-1,
        ),
    )
    for row in rejected:
        node.controller_status_samples = [row]
        with pytest.raises(RuntimeError, match='Safety stop'):
            node.assert_firmware_pid_status_safe(10.40)


def test_firmware_pid_plateau_target_must_match_requested_speed():
    node = object.__new__(MODULE.WheelResponseTest)
    node.firmware_pid_motion_start_time = 10.0
    node.firmware_pid_plateau_start_time = 10.10
    expected = MODULE.wheel_rad_s_to_rpm(1.0)
    node.firmware_pid_plateau_target_rpm = (expected, expected)

    boundary = MODULE.FIRMWARE_PID_PLATEAU_TARGET_TOLERANCE_RPM
    node.controller_status_samples = [make_status_row(
        10.31,
        source=1,
        left_target_rpm=expected + boundary,
        right_target_rpm=expected + boundary,
    )]
    node.assert_firmware_pid_status_safe(10.40)

    node.controller_status_samples = [make_status_row(
        10.31,
        source=1,
        left_target_rpm=expected + boundary + 0.001,
        right_target_rpm=expected + boundary + 0.001,
    )]
    with pytest.raises(RuntimeError, match='plateau target'):
        node.assert_firmware_pid_status_safe(10.40)


def test_firmware_pid_source_ack_is_required_at_grace_boundary():
    node = object.__new__(MODULE.WheelResponseTest)
    node.firmware_pid_motion_start_time = 10.0
    boundary = 10.0 + MODULE.FIRMWARE_PID_SOURCE_GRACE_S
    node.controller_status_samples = [make_status_row(
        boundary,
        source=1,
        left_target_rpm=1.0,
        right_target_rpm=1.0,
    )]
    node.assert_firmware_pid_status_safe(boundary + 0.001)

    node.controller_status_samples = [make_status_row(boundary, source=5)]
    with pytest.raises(RuntimeError, match='source acknowledgement'):
        node.assert_firmware_pid_status_safe(boundary + 0.001)


def test_firmware_pid_diagnostic_guard_requires_fresh_forward_counts():
    node = object.__new__(MODULE.WheelResponseTest)
    node.firmware_pid_motion_start_time = 10.0
    baseline = make_diag_row(10.0)
    node.firmware_pid_diag_start = baseline
    node.encoder_diag_samples = [
        baseline,
        make_diag_row(
            10.40,
            left_legacy=10,
            left_quadrature=20,
            right_legacy=10,
            right_quadrature=20,
            valid_positive=20,
        ),
    ]
    node.assert_firmware_pid_diagnostics_safe(10.40)

    tolerated = make_diag_row(
        10.40,
        left_legacy=10,
        left_quadrature=20,
        right_legacy=10,
        right_quadrature=20,
        valid_positive=20,
        valid_negative=MODULE.FIRMWARE_PID_MAX_VALID_NEGATIVE_TRANSITIONS,
        invalid_transitions=MODULE.FIRMWARE_PID_MAX_INVALID_TRANSITIONS,
    )
    node.encoder_diag_samples = [baseline, tolerated]
    node.assert_firmware_pid_diagnostics_safe(10.40)

    reverse = make_diag_row(
        10.40,
        left_legacy=10,
        left_quadrature=20,
        right_legacy=10,
        right_quadrature=20,
        valid_positive=20,
        valid_negative=(
            MODULE.FIRMWARE_PID_MAX_VALID_NEGATIVE_TRANSITIONS + 1
        ),
    )
    node.encoder_diag_samples = [baseline, reverse]
    with pytest.raises(RuntimeError, match='reverse'):
        node.assert_firmware_pid_diagnostics_safe(10.40)

    invalid = make_diag_row(
        10.40,
        left_legacy=10,
        left_quadrature=20,
        right_legacy=10,
        right_quadrature=20,
        valid_positive=20,
        invalid_transitions=MODULE.FIRMWARE_PID_MAX_INVALID_TRANSITIONS + 1,
    )
    node.encoder_diag_samples = [baseline, invalid]
    with pytest.raises(RuntimeError, match='invalid'):
        node.assert_firmware_pid_diagnostics_safe(10.40)

    node.encoder_diag_samples = [baseline]
    with pytest.raises(RuntimeError, match='no post-command'):
        node.assert_firmware_pid_diagnostics_safe(10.40)


def test_firmware_zero_ack_requires_zero_actuator_state():
    zero = make_status_row(1.0, source=5)
    assert MODULE.firmware_zero_status_is_acknowledged(zero)
    assert not MODULE.firmware_zero_status_is_acknowledged(
        make_status_row(1.0, source=1)
    )
    assert not MODULE.firmware_zero_status_is_acknowledged(
        make_status_row(1.0, source=5, left_output_pwm=1.0)
    )
    assert not MODULE.firmware_zero_status_is_acknowledged(
        make_status_row(1.0, source=5, right_measured_rpm=0.1)
    )
    assert not MODULE.firmware_zero_status_is_acknowledged(
        make_status_row(1.0, source=5, left_integral_rpm_s=0.1)
    )
    quantized_rebound = make_status_row(
        1.0,
        source=5,
        left_measured_rpm=1.559,
        right_measured_rpm=-1.893,
        left_error_rpm=-1.559,
        right_error_rpm=1.893,
    )
    assert MODULE.firmware_status_reports_zero_actuation(quantized_rebound)
    assert not MODULE.firmware_zero_status_is_acknowledged(quantized_rebound)


def test_firmware_pid_requires_harness_to_be_only_command_publisher():
    node = object.__new__(MODULE.WheelResponseTest)
    node.encoder_only_mode = True
    node.left_topic = '/left'
    node.right_topic = '/right'
    node.turret_topic = '/turret'
    counts = {'/left': 1, '/right': 1, '/turret': 1}
    node.count_publishers = counts.__getitem__
    node.assert_exclusive_publishers()

    counts['/left'] = 2
    with pytest.raises(RuntimeError, match='run_controller:=false'):
        node.assert_exclusive_publishers()


def test_firmware_pid_trial_publishes_zero_immediately_on_guard_failure():
    node = object.__new__(MODULE.WheelResponseTest)
    node.encoder_only_mode = True
    node.zero_s = 0.5
    node.pose = None
    node.total_path_m = 0.0
    node.ramp_s = 0.5
    node.hold_s = 1.0
    node.get_logger = lambda: type('Logger', (), {
        'warn': lambda self, _message: None,
    })()
    node.zero_for = lambda _duration: None
    node.begin_firmware_pid_motion = lambda: None
    node.service_callbacks = lambda timeout_sec=0.0: None
    node.left_tick_samples = []
    node.right_tick_samples = []
    published = []
    node.publish_cmd = lambda left, right: published.append((left, right))

    def fail_motion(_duration, command_fn):
        published.append(command_fn(0.25))
        raise RuntimeError('simulated live guard')

    node.run_for = fail_motion
    with pytest.raises(RuntimeError, match='simulated live guard'):
        node.run_trial(1, 'equal_positive_pid', 1.0, 1.0)

    assert published[-1] == (0.0, 0.0)


def test_firmware_pid_stop_drains_latest_ticks_before_coast_baseline(
    monkeypatch,
):
    node = object.__new__(MODULE.WheelResponseTest)
    node.encoder_only_mode = True
    # This reproduces the failed 0.20-equivalent run: a callback-lagged
    # baseline would count 51.5 ticks, while the latest pre-zero samples show
    # that physical coast was only 34 ticks.
    node.left_tick_samples = [(9.95, 2650.0)]
    node.right_tick_samples = [(9.95, 2778.0)]
    events = []

    def drain(timeout_sec=1.0):
        assert timeout_sec == 0.0
        events.append('drain')
        node.left_tick_samples.append((9.99, 2667.0))
        node.right_tick_samples.append((9.99, 2796.0))

    def publish(left, right):
        events.append(('publish', left, right))

    node.service_callbacks = drain
    node.publish_cmd = publish
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: 10.0)

    with pytest.raises(RuntimeError, match='average post-zero'):
        MODULE.require_firmware_pid_post_stop_coast(
            2650.0, 2778.0, 2700.0, 2831.0
        )

    node.publish_firmware_pid_stop()

    assert events == ['drain', ('publish', 0.0, 0.0)]
    assert node.firmware_pid_stop_time == pytest.approx(10.0)
    assert node.firmware_pid_stop_tick_start == (2667.0, 2796.0)
    assert MODULE.require_firmware_pid_post_stop_coast(
        *node.firmware_pid_stop_tick_start, 2700.0, 2831.0
    ) == (33.0, 35.0, 34.0, 2.0)


def test_firmware_pid_stop_attempts_zero_even_if_callback_drain_fails(
    monkeypatch,
):
    node = object.__new__(MODULE.WheelResponseTest)
    node.encoder_only_mode = True
    node.left_tick_samples = [(9.99, 10.0)]
    node.right_tick_samples = [(9.99, 11.0)]
    published = []

    def fail_drain(timeout_sec=1.0):
        assert timeout_sec == 0.0
        raise RuntimeError('simulated executor failure')

    node.service_callbacks = fail_drain
    node.publish_cmd = lambda left, right: published.append((left, right))
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: 10.0)

    with pytest.raises(RuntimeError, match='simulated executor failure'):
        node.publish_firmware_pid_stop()

    assert published == [(0.0, 0.0)]
    assert node.firmware_pid_stop_tick_start == (10.0, 11.0)


def test_firmware_pid_final_zero_waits_for_new_complete_ack(monkeypatch):
    node = object.__new__(MODULE.WheelResponseTest)
    node.encoder_only_mode = True
    node.controller_status_samples = [make_status_row(-1.0, source=1)]
    node.last_controller_status_time = -1.0
    node.assert_ready = lambda: None
    published = []
    node.publish_cmd = lambda left, right: published.append((left, right))

    monotonic_values = iter((0.0, 0.01))
    monkeypatch.setattr(MODULE.time, 'monotonic', monotonic_values.__next__)
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)

    def acknowledge_zero(*_args, **_kwargs):
        node.controller_status_samples = [make_status_row(0.02, source=5)]
        node.last_controller_status_time = 0.02

    monkeypatch.setattr(MODULE.rclpy, 'spin_once', acknowledge_zero)
    node.wait_for_firmware_pid_final_zero()

    assert published == [(0.0, 0.0)]


def test_firmware_pid_hard_elapsed_duration_is_independent_of_parameters(
    monkeypatch,
):
    node = object.__new__(MODULE.WheelResponseTest)
    node.firmware_pid_motion_start_time = 10.0
    node.firmware_pid_tick_start = (0.0, 0.0)
    monkeypatch.setattr(
        MODULE.time,
        'monotonic',
        lambda: 10.0 + MODULE.FIRMWARE_PID_MAX_MOTION_S + 0.001,
    )

    with pytest.raises(RuntimeError, match='hard motion duration'):
        node.assert_firmware_pid_motion_safety()


def test_firmware_pid_post_stop_coast_bounds_are_inclusive():
    assert MODULE.require_firmware_pid_post_stop_coast(
        100, 200, 160, 240
    ) == (60.0, 40.0, 50.0, 20.0)

    for ticks in (
        (100, 200, 161, 241),
        (100, 200, 151, 230),
        (100, 200, float('nan'), 200),
    ):
        with pytest.raises(RuntimeError, match='post-zero|Post-stop'):
            MODULE.require_firmware_pid_post_stop_coast(*ticks)


def test_firmware_pid_nonzero_gap_sends_zero_before_failing(monkeypatch):
    node = object.__new__(MODULE.WheelResponseTest)
    published = []
    node.publish_cmd = lambda left, right: published.append((left, right))
    node.last_wheel_command_publish_time = 0.0

    monkeypatch.setattr(
        MODULE.time,
        'monotonic',
        lambda: MODULE.FIRMWARE_PID_MAX_NONZERO_PUBLICATION_GAP_S,
    )
    node.assert_firmware_pid_publication_gap_safe('boundary')

    monkeypatch.setattr(
        MODULE.time,
        'monotonic',
        lambda: (
            MODULE.FIRMWARE_PID_MAX_NONZERO_PUBLICATION_GAP_S
            + 0.001
        ),
    )
    with pytest.raises(RuntimeError, match='refusing to resume'):
        node.assert_firmware_pid_publication_gap_safe('delayed graph query')
    assert published == [(0.0, 0.0)]


def test_post_stop_allows_measured_rebound_then_requires_final_full_zero(
    monkeypatch,
):
    node = object.__new__(MODULE.WheelResponseTest)
    node.encoder_only_mode = True
    node.firmware_pid_stop_time = 0.0
    node.firmware_pid_stop_tick_start = (0.0, 0.0)
    node.firmware_pid_tick_start = (0.0, 0.0)
    node.firmware_pid_stop_proof = None
    node.left_tick_samples = [(0.0, 0.0)]
    node.right_tick_samples = [(0.0, 0.0)]
    node.controller_status_samples = [make_status_row(-1.0, source=1)]
    node.last_controller_status_time = -1.0
    node.last_wheel_command_publish_time = 0.0
    node.assert_ready = lambda: None
    clock = [0.0]
    published = []

    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)

    def publish(left, right):
        published.append((left, right))
        node.last_wheel_command_publish_time = clock[0]

    def service_callbacks(timeout_sec=0.0):
        del timeout_sec
        clock[0] += 0.025
        # Two short coast observations, followed by exact encoder stability.
        step = len(node.left_tick_samples)
        left = 20.0 if step == 1 else 30.0
        right = 15.0 if step == 1 else 20.0
        node.left_tick_samples.append((clock[0], left))
        node.right_tick_samples.append((clock[0], right))
        # Historical clean stops can emit a few nonzero measured/error fields
        # from encoder quantization after the first complete-zero row. Targets,
        # controller state, and outputs remain exactly zero throughout.
        if 0.05 <= clock[0] < 0.20:
            status = make_status_row(
                clock[0],
                source=5,
                left_measured_rpm=1.559,
                right_measured_rpm=-1.893,
                left_error_rpm=-1.559,
                right_error_rpm=1.893,
            )
        else:
            status = make_status_row(clock[0], source=5)
        node.controller_status_samples.append(status)
        node.last_controller_status_time = clock[0]

    node.publish_cmd = publish
    node.service_callbacks = service_callbacks
    node.wait_for_firmware_pid_stop_proof()

    proof = node.firmware_pid_stop_proof
    assert proof['post_stop_average_abs_coast_ticks'] == 25.0
    assert proof['post_stop_differential_growth_ticks'] == 10.0
    assert (
        proof['post_stop_encoder_freeze_s']
        >= MODULE.FIRMWARE_PID_POST_STOP_FREEZE_S
    )
    assert published and set(published) == {(0.0, 0.0)}


def test_encoder_preflight_rejects_publisher_conflict_before_first_heartbeat(
    monkeypatch,
):
    node = object.__new__(MODULE.WheelResponseTest)
    node.encoder_only_mode = True
    node.startup_timeout_s = 5.0
    node.left_topic = '/left'
    node.right_topic = '/right'
    node.turret_topic = '/turret'
    counts = {'/left': 2, '/right': 1, '/turret': 1}
    node.count_publishers = counts.__getitem__
    published = []
    node.publish_cmd = lambda left, right: published.append((left, right))
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: 0.0)
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)

    with pytest.raises(RuntimeError, match='run_controller:=false'):
        node.wait_for_inputs()
    assert published == []


@pytest.mark.parametrize(
    'status',
    (
        make_status_row(0.0, source=2),
        make_status_row(0.0, source=5, left_output_pwm=1.0),
    ),
)
def test_encoder_preflight_rejects_unsafe_status_before_first_heartbeat(
    status,
    monkeypatch,
):
    node = object.__new__(MODULE.WheelResponseTest)
    node.encoder_only_mode = True
    node.startup_timeout_s = 5.0
    node.left_topic = '/left'
    node.right_topic = '/right'
    node.turret_topic = '/turret'
    node.count_publishers = lambda _topic: 1
    node.controller_status_schema_error = None
    node.encoder_diag_schema_error = None
    node.controller_status_samples = [status]
    published = []
    node.publish_cmd = lambda left, right: published.append((left, right))
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: 0.0)
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)

    with pytest.raises(RuntimeError, match='Preflight'):
        node.wait_for_inputs()
    assert published == []


def test_encoder_callback_service_is_bounded_burst(monkeypatch):
    node = object.__new__(MODULE.WheelResponseTest)
    node.encoder_only_mode = True
    timeouts = []
    monkeypatch.setattr(
        MODULE.rclpy,
        'spin_once',
        lambda _node, timeout_sec: timeouts.append(timeout_sec),
    )

    node.service_callbacks(timeout_sec=0.05)

    assert len(timeouts) == MODULE.FIRMWARE_PID_CALLBACK_DRAIN_LIMIT
    assert timeouts[0] == 0.05
    assert set(timeouts[1:]) == {0.0}


def test_post_stop_coast_cannot_exceed_original_motion_tick_envelope(
    monkeypatch,
):
    node = object.__new__(MODULE.WheelResponseTest)
    node.encoder_only_mode = True
    node.firmware_pid_stop_time = 0.0
    node.firmware_pid_stop_tick_start = (1190.0, 1190.0)
    node.firmware_pid_tick_start = (0.0, 0.0)
    node.left_tick_samples = [(0.0, 1190.0)]
    node.right_tick_samples = [(0.0, 1190.0)]
    node.controller_status_samples = [make_status_row(-1.0, source=1)]
    node.last_controller_status_time = -1.0
    node.last_wheel_command_publish_time = 0.0
    node.assert_ready = lambda: None
    clock = [0.0]
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)

    def publish(_left, _right):
        node.last_wheel_command_publish_time = clock[0]

    def service_callbacks(timeout_sec=0.0):
        del timeout_sec
        clock[0] += 0.025
        node.left_tick_samples.append((clock[0], 1210.0))
        node.right_tick_samples.append((clock[0], 1210.0))
        node.controller_status_samples.append(
            make_status_row(clock[0], source=5)
        )
        node.last_controller_status_time = clock[0]

    node.publish_cmd = publish
    node.service_callbacks = service_callbacks
    with pytest.raises(RuntimeError, match='tick-travel limit'):
        node.wait_for_firmware_pid_stop_proof()


def test_post_stop_requires_prompt_complete_zero_ack(monkeypatch):
    node = object.__new__(MODULE.WheelResponseTest)
    node.encoder_only_mode = True
    node.firmware_pid_stop_time = 0.0
    node.firmware_pid_stop_tick_start = (0.0, 0.0)
    node.firmware_pid_tick_start = (0.0, 0.0)
    node.left_tick_samples = [(0.0, 0.0)]
    node.right_tick_samples = [(0.0, 0.0)]
    node.controller_status_samples = [make_status_row(-1.0, source=1)]
    node.last_controller_status_time = -1.0
    node.last_wheel_command_publish_time = 0.0
    node.assert_ready = lambda: None
    clock = [0.0]
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)

    def publish(_left, _right):
        node.last_wheel_command_publish_time = clock[0]

    def service_callbacks(timeout_sec=0.0):
        del timeout_sec
        clock[0] += 0.025
        node.left_tick_samples.append((clock[0], 0.0))
        node.right_tick_samples.append((clock[0], 0.0))
        source = (
            5
            if clock[0] > MODULE.FIRMWARE_PID_POST_STOP_ZERO_ACK_MAX_S
            else 1
        )
        node.controller_status_samples.append(
            make_status_row(clock[0], source=source)
        )
        node.last_controller_status_time = clock[0]

    node.publish_cmd = publish
    node.service_callbacks = service_callbacks
    with pytest.raises(RuntimeError, match='acknowledgement'):
        node.wait_for_firmware_pid_stop_proof()


def test_post_stop_rejects_zero_state_regression_after_ack(monkeypatch):
    node = object.__new__(MODULE.WheelResponseTest)
    node.encoder_only_mode = True
    node.firmware_pid_stop_time = 0.0
    node.firmware_pid_stop_tick_start = (0.0, 0.0)
    node.firmware_pid_tick_start = (0.0, 0.0)
    node.left_tick_samples = [(0.0, 0.0)]
    node.right_tick_samples = [(0.0, 0.0)]
    node.controller_status_samples = [make_status_row(-1.0, source=1)]
    node.last_controller_status_time = -1.0
    node.last_wheel_command_publish_time = 0.0
    node.assert_ready = lambda: None
    clock = [0.0]
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)

    def publish(_left, _right):
        node.last_wheel_command_publish_time = clock[0]

    def service_callbacks(timeout_sec=0.0):
        del timeout_sec
        clock[0] += 0.025
        node.left_tick_samples.append((clock[0], 0.0))
        node.right_tick_samples.append((clock[0], 0.0))
        status = (
            make_status_row(clock[0], source=5)
            if clock[0] < 0.05
            else make_status_row(
                clock[0],
                source=1,
                left_target_rpm=1.0,
                right_target_rpm=1.0,
                left_output_pwm=100.0,
                right_output_pwm=100.0,
            )
        )
        node.controller_status_samples.append(status)
        node.last_controller_status_time = clock[0]

    node.publish_cmd = publish
    node.service_callbacks = service_callbacks
    with pytest.raises(RuntimeError, match='zero state regressed'):
        node.wait_for_firmware_pid_stop_proof()
