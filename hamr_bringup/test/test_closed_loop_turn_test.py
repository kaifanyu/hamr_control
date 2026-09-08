"""Pure-math regression tests for the supervised closed-loop turn harness."""

import ast
from collections import deque
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
import math
from pathlib import Path

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'closed_loop_turn_test'
)
WAYPOINT_SIMPLE = (
    Path(__file__).resolve().parents[2]
    / 'reference_trajectory'
    / 'reference_trajectory'
    / 'waypoint_traj_simple.py'
)
LOADER = SourceFileLoader('closed_loop_turn_test_module', str(SCRIPT))
SPEC = spec_from_loader(LOADER.name, LOADER)
MODULE = module_from_spec(SPEC)
LOADER.exec_module(MODULE)


def test_monitoring_qos_and_callback_service_until_deadline(
    monkeypatch,
):
    assert MODULE.MONITOR_QOS_DEPTH == 1
    assert MODULE.ODOM_MONITOR_QOS.depth == MODULE.MONITOR_QOS_DEPTH
    assert (
        MODULE.ODOM_MONITOR_QOS.history
        == MODULE.HistoryPolicy.KEEP_LAST
    )
    assert (
        MODULE.ODOM_MONITOR_QOS.reliability
        == MODULE.ReliabilityPolicy.BEST_EFFORT
    )
    tree = ast.parse(SCRIPT.read_text(encoding='utf-8'))
    subscriptions = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == 'create_subscription'
    ]
    assert len(subscriptions) == 5
    odom_subscriptions = [
        call
        for call in subscriptions
        if isinstance(call.args[2], ast.Attribute)
        and call.args[2].attr == 'odom_cb'
    ]
    assert len(odom_subscriptions) == 1
    assert (
        isinstance(odom_subscriptions[0].args[3], ast.Name)
        and odom_subscriptions[0].args[3].id == 'ODOM_MONITOR_QOS'
    )
    other_subscriptions = [
        call for call in subscriptions if call not in odom_subscriptions
    ]
    assert all(
        len(call.args) == 4
        and isinstance(call.args[3], ast.Name)
        and call.args[3].id == 'MONITOR_QOS_DEPTH'
        for call in other_subscriptions
    )

    now = [0.0]
    pending_callbacks = list(range(8))
    spin_timeouts = []

    def spin_once(node, timeout_sec):
        spin_timeouts.append(timeout_sec)
        if pending_callbacks:
            pending_callbacks.pop()
            now[0] += 0.001
        else:
            now[0] += timeout_sec

    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: now[0])
    node = object.__new__(MODULE.ClosedLoopTurnTest)
    node.armed = False
    node.latched_odom_source_error = None
    node.latched_odom_pose_envelope_error = None
    node.callback_executor = type('Executor', (), {
        'spin_once': lambda self, timeout_sec: spin_once(
            node, timeout_sec
        ),
    })()
    node._service_callbacks_until(0.020)

    assert pending_callbacks == []
    assert len(spin_timeouts) == 9
    assert spin_timeouts[0] == pytest.approx(0.020)
    assert spin_timeouts[-1] == pytest.approx(0.012)
    assert now[0] == pytest.approx(0.020)


def test_supervisor_and_controller_odom_timeouts_are_profile_bounded_and_match():
    assert MODULE.DEFAULT_STALE_ODOM_S == pytest.approx(0.12)
    assert MODULE.MIRRORED_90_STALE_ODOM_S == pytest.approx(0.45)
    assert MODULE.MIRRORED_90_FULL_SPEED_M_S == pytest.approx(0.20)
    assert MODULE.MIRRORED_90_FULL_SPEED_STALE_ODOM_S == pytest.approx(0.20)
    assert MODULE.MAX_CONTROLLER_ODOM_TIMEOUT_S == pytest.approx(0.45)
    for value in (0.001, 0.10, 0.12, 0.20, 0.45):
        assert MODULE.require_controller_odom_timeout(
            value, value
        ) == pytest.approx(value)

    for value, expected in (
        (0.12, 0.45),
        (0.45, 0.12),
        (0.450001, 0.45),
        (None, 0.12),
        (True, 0.12),
        (0.0, 0.12),
        (-0.1, 0.12),
        (math.nan, 0.12),
        (math.inf, 0.12),
    ):
        with pytest.raises(RuntimeError, match='odom_timeout_s'):
            MODULE.require_controller_odom_timeout(value, expected)

    for expected in (None, True, 0.0, -0.1, 0.450001, math.nan, math.inf):
        with pytest.raises(RuntimeError, match='stale_odom_s'):
            MODULE.require_controller_odom_timeout(0.12, expected)

    ordinary = MODULE.get_profile_config('vicon_straight')
    mirrored = MODULE.get_profile_config('mirrored_90')
    assert ordinary['default_stale_odom_s'] == pytest.approx(0.12)
    assert ordinary['max_stale_odom_s'] == pytest.approx(0.12)
    assert mirrored['default_stale_odom_s'] == pytest.approx(0.12)
    assert mirrored['max_stale_odom_s'] == pytest.approx(0.12)
    assert mirrored['required_stale_odom_s_by_speed'] == (
        (0.15, 0.45),
        (0.20, 0.20),
    )

    assert MODULE.require_profile_stale_odom_timeout(
        'mirrored_90', 0.15, 0.45, mirrored
    ) == pytest.approx(0.45)
    with pytest.raises(ValueError, match=r'requires explicit.*0\.45s'):
        MODULE.require_profile_stale_odom_timeout(
            'mirrored_90', 0.15, 0.12, mirrored
        )
    with pytest.raises(ValueError, match=r"mirrored_90.*\(0, 0\.45\]"):
        MODULE.require_profile_stale_odom_timeout(
            'mirrored_90', 0.15, 0.450001, mirrored
        )
    with pytest.raises(ValueError, match=r"mirrored_90.*\(0, 0\.20\]"):
        MODULE.require_profile_stale_odom_timeout(
            'mirrored_90', 0.20, 0.45, mirrored
        )
    with pytest.raises(ValueError, match=r'requires explicit.*0\.20s'):
        MODULE.require_profile_stale_odom_timeout(
            'mirrored_90', 0.20, 0.12, mirrored
        )
    assert MODULE.require_profile_stale_odom_timeout(
        'mirrored_90', 0.20, 0.20, mirrored
    ) == pytest.approx(0.20)
    assert MODULE.require_profile_stale_odom_timeout(
        'mirrored_90', 0.18, 0.12, mirrored
    ) == pytest.approx(0.12)

    tree = ast.parse(SCRIPT.read_text(encoding='utf-8'))
    stale_declarations = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == 'declare_parameter'
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == 'stale_odom_s'
    ]
    assert len(stale_declarations) == 1
    default_argument = stale_declarations[0].args[1]
    assert ast.unparse(default_argument) == (
        "self.profile_config['default_stale_odom_s']"
    )


def test_controller_reference_timeout_is_exactly_120ms_and_ordered_after_gap():
    assert MODULE.MAX_REFERENCE_PUBLICATION_GAP_S == pytest.approx(0.10)
    assert MODULE.REQUIRED_CONTROLLER_REFERENCE_TIMEOUT_S == pytest.approx(
        0.12
    )
    assert (
        MODULE.MAX_REFERENCE_PUBLICATION_GAP_S
        < MODULE.REQUIRED_CONTROLLER_REFERENCE_TIMEOUT_S
    )
    assert MODULE.require_controller_reference_timeout(0.12) == pytest.approx(
        0.12
    )

    for value in (
        0.119999,
        0.120001,
        None,
        True,
        0.0,
        -0.1,
        math.nan,
        math.inf,
    ):
        with pytest.raises(RuntimeError, match='reference_timeout_s'):
            MODULE.require_controller_reference_timeout(value)


def test_controller_and_supervisor_require_the_28_rpm_operating_cap():
    required = MODULE.HARDWARE_WHEEL_SPEED_LIMIT_RAD_S
    assert required == pytest.approx(2.93215314335)
    assert required * 60.0 / (2.0 * math.pi) == pytest.approx(28.0)
    assert MODULE.require_controller_wheel_speed_limit(
        required
    ) == pytest.approx(required)

    for value in (
        required - 0.000001,
        required + 0.000001,
        2.09439510239,
        None,
        True,
        0.0,
        math.nan,
        math.inf,
    ):
        with pytest.raises(RuntimeError, match='wheel_speed_limit_rad_s'):
            MODULE.require_controller_wheel_speed_limit(value)

    tree = ast.parse(SCRIPT.read_text(encoding='utf-8'))
    declarations = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == 'declare_parameter'
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == 'max_abs_wheel_cmd'
    ]
    assert len(declarations) == 1
    assert ast.unparse(declarations[0].args[1]) == (
        'HARDWARE_WHEEL_SPEED_LIMIT_RAD_S'
    )
    controller_class = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef)
        and item.name == 'ClosedLoopTurnTest'
    )
    preflight = next(
        item
        for item in controller_class.body
        if isinstance(item, ast.FunctionDef)
        and item.name == 'read_controller_parameters'
    )
    limit_checks = [
        call
        for call in ast.walk(preflight)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'require_controller_wheel_speed_limit'
    ]
    assert len(limit_checks) == 1
    assert ast.unparse(limit_checks[0].args[0]) == (
        "values['wheel_speed_limit_rad_s']"
    )


def test_mirrored_receipt_grace_accepts_observed_gaps_but_not_outage():
    node = object.__new__(MODULE.ClosedLoopTurnTest)
    accepted_time = 10.0

    assert node._fresh(accepted_time, 0.45, accepted_time + 0.400)
    assert node._fresh(accepted_time, 0.45, accepted_time + 0.450)
    assert not node._fresh(accepted_time, 0.45, accepted_time + 0.450001)
    assert not node._fresh(accepted_time, 0.45, accepted_time + 0.667)

    assert node._fresh(accepted_time, 0.20, accepted_time + 0.14545)
    assert node._fresh(accepted_time, 0.20, accepted_time + 0.200)
    assert not node._fresh(accepted_time, 0.20, accepted_time + 0.200001)

    # Nominal travel inside either exact grace remains below the controller's
    # independently enforced 80 mm pose-jump guard.
    assert 0.15 * 0.45 == pytest.approx(0.0675)
    assert 0.15 * 0.45 < 0.08
    assert 0.20 * 0.20 == pytest.approx(0.04)
    assert 0.20 * 0.20 < 0.08
    assert 0.20 * 0.45 > 0.08

def test_source_stamp_policy_is_shared_and_not_profile_or_speed_bounded():
    assert MODULE.VICON_SOURCE_STAMP_POLICY == 'receipt_monotonic'
    assert MODULE.require_vicon_source_stamp_policy(
        ' RECEIPT_MONOTONIC '
    ) == MODULE.VICON_SOURCE_STAMP_POLICY

    for profile in ('s_curve', 'vicon_straight', 'mirrored_90', 'legacy_90'):
        config = MODULE.get_profile_config(profile)
        assert not any('source_age' in name for name in config)

    for value in ('absolute_age', 'receipt', '', None, True, 0.25):
        with pytest.raises(RuntimeError, match='vicon_source_stamp_policy'):
            MODULE.require_vicon_source_stamp_policy(value)


def test_callback_service_returns_as_soon_as_source_failure_latches(
    monkeypatch,
):
    now = [0.0]
    spin_calls = []

    def spin_once(node, timeout_sec):
        spin_calls.append(timeout_sec)
        node.latched_odom_source_error = (
            'base Vicon non-monotonic source timestamp'
        )
        now[0] += 0.001

    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: now[0])
    node = object.__new__(MODULE.ClosedLoopTurnTest)
    node.armed = True
    node.latched_odom_source_error = None
    node.latched_odom_pose_envelope_error = None
    node.callback_executor = type('Executor', (), {
        'spin_once': lambda self, timeout_sec: spin_once(
            node, timeout_sec
        ),
    })()

    node._service_callbacks_until(0.020)

    assert spin_calls == [pytest.approx(0.020)]
    assert now[0] == pytest.approx(0.001)


def test_absolute_reference_deadline_skips_missed_slots():
    start = 10.0
    period = 0.02

    assert MODULE.next_reference_deadline(
        start, period, 10.0001
    ) == pytest.approx(10.02)
    assert MODULE.next_reference_deadline(
        start, period, 10.039
    ) == pytest.approx(10.04)
    assert MODULE.next_reference_deadline(
        start, period, 10.041
    ) == pytest.approx(10.06)
    # A 349 ms overrun advances to one future wall-clock slot. It never asks
    # the caller to replay the seventeen missed 20 ms publications.
    assert MODULE.next_reference_deadline(
        start, period, 10.349
    ) == pytest.approx(10.36)


def test_349ms_executor_overrun_aborts_before_late_reference(monkeypatch):
    now = [0.020]
    spin_calls = []
    safety_calls = []

    class Executor:
        def spin_once(self, timeout_sec):
            spin_calls.append(timeout_sec)
            if len(spin_calls) == 1:
                now[0] += 0.349

    node = object.__new__(MODULE.ClosedLoopTurnTest)
    node.armed = True
    node.latched_odom_source_error = None
    node.latched_odom_pose_envelope_error = None
    node.callback_executor = Executor()
    node.assert_active_safety = lambda current, elapsed: safety_calls.append(
        (current, elapsed)
    )
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: now[0])

    with pytest.raises(
        RuntimeError,
        match=r'reference publication gap 0\.349s exceeded 0\.100s',
    ):
        node._prepare_reference_cycle(
            run_start=0.0,
            last_publish_time=0.020,
        )

    assert spin_calls == [0.0] * MODULE.MONITOR_STREAM_COUNT
    assert safety_calls == []


def test_normal_reference_jitter_drains_then_checks_current_time(
    monkeypatch,
):
    now = [0.039]
    spin_calls = []
    safety_calls = []

    class Executor:
        def spin_once(self, timeout_sec):
            spin_calls.append(timeout_sec)

    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: now[0])
    node = object.__new__(MODULE.ClosedLoopTurnTest)
    node.armed = True
    node.latched_odom_source_error = None
    node.latched_odom_pose_envelope_error = None
    node.callback_executor = Executor()
    node.assert_active_safety = lambda current, elapsed: safety_calls.append(
        (current, elapsed)
    )

    current, elapsed = node._prepare_reference_cycle(
        run_start=0.0,
        last_publish_time=0.020,
    )

    assert current == pytest.approx(0.039)
    assert elapsed == pytest.approx(0.039)
    assert safety_calls == [(pytest.approx(0.039), pytest.approx(0.039))]
    assert spin_calls == [0.0] * MODULE.MONITOR_STREAM_COUNT
    assert MODULE.next_reference_deadline(
        0.0, 0.02, current
    ) == pytest.approx(0.04)


def test_active_safety_never_queries_blocking_dds_graph():
    node = object.__new__(MODULE.ClosedLoopTurnTest)
    node.hard_timeout_s = 10.0
    node.latched_odom_source_error = None
    node.odom_source_error = None
    node.latched_odom_pose_envelope_error = None
    node.odom_pose_envelope_error = None
    node.last_odom_time = 1.0
    node.stale_odom_s = 0.12
    node.last_status_time = 1.0
    node.stale_status_s = 0.35
    node.status_schema_error = None
    node.assert_saturation_safety = lambda now: None
    node._graph_ready = lambda: pytest.fail(
        'active safety must not synchronously query the DDS graph'
    )
    node.command_startup_grace_s = 1.0
    node.left_cmd = 0.0
    node.right_cmd = 0.0
    node.max_abs_wheel_cmd = MODULE.HARDWARE_WHEEL_SPEED_LIMIT_RAD_S
    node.pose = (0.0, 0.0)
    node.run_start_pose = (0.0, 0.0)
    node.unwrapped_yaw = 0.0
    node.run_start_yaw = 0.0
    node.active_vicon_bounds = None
    node.max_displacement_m = 1.0
    node.total_path_m = 0.0
    node.max_total_path_m = 1.0
    node.max_yaw_change_rad = 1.0

    node.assert_active_safety(now=1.01, armed_elapsed=0.1)


def test_persistent_executor_replaces_global_spin_once_calls():
    tree = ast.parse(SCRIPT.read_text(encoding='utf-8'))
    global_spin_calls = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == 'rclpy'
        and call.func.attr == 'spin_once'
    ]
    assert global_spin_calls == []

    constructors = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'SingleThreadedExecutor'
    ]
    assert len(constructors) == 1


def test_path_is_bounded_and_mirrored():
    points = MODULE.LOCAL_PATH_OFFSETS

    assert points[0] == pytest.approx((0.0, 0.0))
    assert points[-1] == pytest.approx((0.0, 0.8))
    assert max(abs(x) for x, _ in points) == pytest.approx(0.07)
    assert all(after[1] > before[1] for before, after in zip(points, points[1:]))

    for point, mirror in zip(points, reversed(points)):
        assert point[0] == pytest.approx(-mirror[0])
        assert point[1] + mirror[1] == pytest.approx(0.8)

    _, length = MODULE.path_timing(points, 0.08)
    assert length == pytest.approx(0.8696035155901702)


def test_transform_path_is_relative_to_position_and_heading():
    transformed = MODULE.transform_path(
        ((0.0, 0.0), (0.0, 0.8)),
        start_x=1.0,
        start_y=2.0,
        start_yaw=math.pi / 2.0,
    )

    assert transformed[0] == pytest.approx((1.0, 2.0))
    assert transformed[1] == pytest.approx((0.2, 2.0))


def test_translate_path_moves_origin_without_rotating_vicon_axes():
    translated = MODULE.translate_path(
        ((0.0, 0.0), (0.0, 2.0), (-2.0, 2.0)),
        start_x=3.5,
        start_y=-1.25,
    )

    expected = ((3.5, -1.25), (3.5, 0.75), (1.5, 0.75))
    for actual, target in zip(translated, expected):
        assert actual == pytest.approx(target)


def test_path_sampling_preserves_speed_and_stops_at_end():
    points = MODULE.LOCAL_PATH_OFFSETS
    speed = 0.08
    timing, _ = MODULE.path_timing(points, speed)

    for segment in range(len(points) - 1):
        midpoint_t = 0.5 * (timing[segment] + timing[segment + 1])
        _, _, x_dot, y_dot = MODULE.sample_path(
            points, timing, midpoint_t
        )
        assert math.hypot(x_dot, y_dot) == pytest.approx(speed)

    x, y, x_dot, y_dot = MODULE.sample_path(points, timing, timing[-1])
    assert (x, y) == pytest.approx(points[-1])
    assert (x_dot, y_dot) == pytest.approx((0.0, 0.0))


def test_default_schedule_has_margin_inside_absolute_hard_stop():
    runtime = MODULE.scheduled_runtime_s(
        MODULE.LOCAL_PATH_OFFSETS,
        speed_m_s=0.08,
        start_hold_s=2.0,
        final_hold_s=3.0,
    )

    assert runtime == pytest.approx(15.870043944877128)
    assert runtime <= MODULE.ABSOLUTE_HARD_TIMEOUT_S - 0.25


def test_profile_defaults_preserve_compact_s_curve():
    config = MODULE.get_profile_config('s_curve')

    assert config['path_offsets'] is MODULE.S_CURVE_PATH_OFFSETS
    assert config['default_speed_m_s'] == pytest.approx(0.08)
    assert config['default_hard_timeout_s'] == pytest.approx(18.0)
    assert config['default_max_displacement_m'] == pytest.approx(1.0)
    assert config['default_max_total_path_m'] == pytest.approx(1.5)
    assert config['default_max_yaw_change_rad'] == pytest.approx(1.0)
    assert config['default_saturation_timeout_s'] == pytest.approx(0.0)
    assert config['frame_mode'] == 'relative'
    assert config['vicon_bounds_offsets'] is None


def test_vicon_straight_profile_exact_geometry_config_and_timing():
    expected = ((0.0, 0.0), (0.0, 0.8))
    config = MODULE.get_profile_config('vicon_straight')

    assert MODULE.VICON_STRAIGHT_PATH_OFFSETS == expected
    assert config['path_offsets'] is MODULE.VICON_STRAIGHT_PATH_OFFSETS
    for speed, motion_s, runtime_s in (
        (0.05, 16.0, 21.0),
        (0.08, 10.0, 15.0),
        (0.10, 8.0, 13.0),
        (0.15, 16.0 / 3.0, 31.0 / 3.0),
    ):
        timing, length = MODULE.path_timing(expected, speed)
        assert length == pytest.approx(0.8)
        assert timing == pytest.approx((0.0, motion_s))
        assert MODULE.scheduled_runtime_s(
            expected, speed, 2.0, 3.0
        ) == pytest.approx(runtime_s)
        assert runtime_s <= 24.0 - 0.25
    assert config['default_speed_m_s'] == pytest.approx(0.05)
    assert config['max_speed_m_s'] == pytest.approx(0.15)
    assert MODULE.require_profile_speed(
        'vicon_straight', 0.15, config
    ) == pytest.approx(0.15)
    with pytest.raises(ValueError, match=r"vicon_straight.*\(0, 0\.15\]"):
        MODULE.require_profile_speed(
            'vicon_straight', 0.150001, config
        )
    # Worst ideal chassis direction coefficient at the requested speed remains
    # feasible below the supervised 28 RPM operating ceiling.
    assert 0.15 * 12.570901 == pytest.approx(1.88563515)
    assert 0.15 * 12.570901 < MODULE.HARDWARE_WHEEL_SPEED_LIMIT_RAD_S
    assert 1.0 - (
        0.15 * 12.570901 / MODULE.HARDWARE_WHEEL_SPEED_LIMIT_RAD_S
    ) == pytest.approx(
        0.356911096449
    )
    assert config['default_hard_timeout_s'] == pytest.approx(24.0)
    assert config['max_hard_timeout_s'] == pytest.approx(24.0)
    assert config['default_max_displacement_m'] == pytest.approx(1.0)
    assert config['max_displacement_m'] == pytest.approx(1.0)
    assert config['default_max_total_path_m'] == pytest.approx(1.1)
    assert config['max_total_path_m'] == pytest.approx(1.1)
    assert config['default_max_yaw_change_rad'] == pytest.approx(0.5)
    assert config['max_yaw_change_rad'] == pytest.approx(0.5)
    assert config['default_saturation_timeout_s'] == pytest.approx(0.0)
    assert config['max_saturation_timeout_s'] == pytest.approx(0.0)
    assert config['explicit_saturation_speed_m_s'] == pytest.approx(0.15)
    assert config['explicit_saturation_timeout_s'] == pytest.approx(0.05)
    assert config['explicit_clean_preflight_speed_m_s'] == pytest.approx(0.15)
    assert config['required_clean_preflight_s'] == pytest.approx(12.0)
    assert config['required_startup_timeout_s'] == pytest.approx(30.0)
    assert config['frame_mode'] == 'translated_vicon'
    assert config['max_start_position_error_m'] is None
    assert config['max_start_yaw_error_rad'] == pytest.approx(0.15)
    assert config['vicon_bounds_offsets'] == pytest.approx(
        (-0.20, 0.20, -0.20, 1.0)
    )
    assert MODULE.scheduled_runtime_s(
        expected, 0.05, 2.0, 3.0
    ) <= config['default_hard_timeout_s'] - 0.25


def test_vicon_straight_translation_bounds_and_schedule_are_exact():
    config = MODULE.get_profile_config('vicon_straight')
    points = MODULE.translate_path(
        config['path_offsets'], start_x=1.25, start_y=-2.0
    )
    bounds = MODULE.translate_bounds(
        config['vicon_bounds_offsets'], start_x=1.25, start_y=-2.0
    )
    timing, _ = MODULE.path_timing(points, 0.05)

    for actual, expected in zip(points, ((1.25, -2.0), (1.25, -1.2))):
        assert actual == pytest.approx(expected)
    assert bounds == pytest.approx((1.05, 1.45, -2.20, -1.0))
    assert MODULE.point_in_bounds(*points[0], bounds)
    assert MODULE.point_in_bounds(*points[-1], bounds)
    assert not MODULE.point_in_bounds(1.049, -1.5, bounds)
    assert not MODULE.point_in_bounds(1.25, -0.999, bounds)

    assert MODULE.sample_reference_schedule(
        points, timing, 2.0 - 1e-6, 2.0, 3.0
    ) == pytest.approx((*points[0], 0.0, 0.0))
    assert MODULE.sample_reference_schedule(
        points, timing, 2.0, 2.0, 3.0
    ) == pytest.approx((*points[0], 0.0, 0.05))
    assert MODULE.sample_reference_schedule(
        points, timing, 18.0, 2.0, 3.0
    ) == pytest.approx((*points[-1], 0.0, 0.0))
    assert MODULE.sample_reference_schedule(
        points, timing, 21.0 - 1e-6, 2.0, 3.0
    ) == pytest.approx((*points[-1], 0.0, 0.0))
    assert MODULE.sample_reference_schedule(
        points, timing, 21.0, 2.0, 3.0
    ) is None


def test_mirrored_90_profile_exact_geometry_runtime_and_bounds():
    expected = (
        (0.0, 0.0),
        (0.0, 1.0),
        (-1.0, 1.0),
        (-1.0, 2.0),
        (0.0, 2.0),
        (0.0, 3.0),
    )
    config = MODULE.get_profile_config('mirrored_90')

    assert MODULE.MIRRORED_90_PATH_OFFSETS == expected
    assert config['path_offsets'] is MODULE.MIRRORED_90_PATH_OFFSETS
    timing, length = MODULE.path_timing(expected, 0.20)
    assert length == pytest.approx(5.0)
    assert timing == pytest.approx((0.0, 5.0, 10.0, 15.0, 20.0, 25.0))
    assert MODULE.scheduled_runtime_s(
        expected, 0.20, 2.0, 3.0
    ) == pytest.approx(30.0)
    assert config['default_speed_m_s'] == pytest.approx(0.20)
    assert config['max_speed_m_s'] == pytest.approx(0.20)
    # The worst ideal chassis direction remains below the matched controller
    # and firmware command ceiling. PWM saturation is checked independently.
    assert 0.20 * 12.570901 == pytest.approx(2.5141802)
    assert 0.20 * 12.570901 < MODULE.HARDWARE_WHEEL_SPEED_LIMIT_RAD_S
    assert 1.0 - (
        0.20 * 12.570901 / MODULE.HARDWARE_WHEEL_SPEED_LIMIT_RAD_S
    ) == pytest.approx(0.142548128599)
    assert config['default_hard_timeout_s'] == pytest.approx(33.0)
    assert config['max_hard_timeout_s'] == pytest.approx(42.0)
    assert config['default_max_displacement_m'] == pytest.approx(3.3)
    assert config['max_displacement_m'] == pytest.approx(3.3)
    assert config['default_max_total_path_m'] == pytest.approx(5.8)
    assert config['max_total_path_m'] == pytest.approx(5.8)
    assert config['default_max_yaw_change_rad'] == pytest.approx(2.5)
    assert config['max_yaw_change_rad'] == pytest.approx(2.5)
    assert config['default_saturation_timeout_s'] == pytest.approx(0.5)
    assert config['max_saturation_timeout_s'] == pytest.approx(0.5)
    assert config['frame_mode'] == 'translated_vicon'
    assert config['max_start_position_error_m'] is None
    assert config['max_start_yaw_error_rad'] == pytest.approx(0.15)
    assert config['vicon_bounds_offsets'] == pytest.approx(
        (-1.3, 0.3, -0.3, 3.3)
    )


def test_mirrored_90_translation_and_all_corners_are_continuous():
    config = MODULE.get_profile_config('mirrored_90')
    points = MODULE.translate_path(
        config['path_offsets'], start_x=2.5, start_y=-1.0
    )
    expected = (
        (2.5, -1.0),
        (2.5, 0.0),
        (1.5, 0.0),
        (1.5, 1.0),
        (2.5, 1.0),
        (2.5, 2.0),
    )
    for actual, target in zip(points, expected):
        assert actual == pytest.approx(target)

    bounds = MODULE.translate_bounds(
        config['vicon_bounds_offsets'], start_x=2.5, start_y=-1.0
    )
    assert bounds == pytest.approx((1.2, 2.8, -1.3, 2.3))
    assert all(MODULE.point_in_bounds(*point, bounds) for point in points)

    timing, _ = MODULE.path_timing(points, 0.20)
    epsilon = 1e-6
    expected_incoming = (
        (0.0, 0.20),
        (-0.20, 0.0),
        (0.0, 0.20),
        (0.20, 0.0),
    )
    expected_outgoing = (
        (-0.20, 0.0),
        (0.0, 0.20),
        (0.20, 0.0),
        (0.0, 0.20),
    )
    turn_signs = []
    for index in range(1, len(points) - 1):
        before = MODULE.sample_path(
            points, timing, timing[index] - epsilon
        )
        at_boundary = MODULE.sample_path(points, timing, timing[index])
        assert before[:2] == pytest.approx(points[index], abs=1e-6)
        assert before[2:] == pytest.approx(expected_incoming[index - 1])
        assert at_boundary[:2] == pytest.approx(points[index])
        assert at_boundary[2:] == pytest.approx(
            expected_outgoing[index - 1]
        )
        assert math.hypot(*at_boundary[2:]) == pytest.approx(0.20)
        vx_in, vy_in = before[2:]
        vx_out, vy_out = at_boundary[2:]
        turn_signs.append(math.copysign(
            1.0, vx_in * vy_out - vy_in * vx_out
        ))

    assert turn_signs == [1.0, -1.0, -1.0, 1.0]


def test_mirrored_90_point_15_runtime_requires_bounded_timeout_override():
    config = MODULE.get_profile_config('mirrored_90')
    runtime = MODULE.scheduled_runtime_s(
        config['path_offsets'], 0.15, 2.0, 3.0
    )

    assert runtime == pytest.approx(38.333333333333336)
    assert runtime > config['default_hard_timeout_s'] - 0.25
    assert runtime <= config['max_hard_timeout_s'] - 0.25


def test_mirrored_90_accepts_point_15_and_point_20_but_rejects_point_25():
    config = MODULE.get_profile_config('mirrored_90')

    assert MODULE.require_profile_speed(
        'mirrored_90', 0.15, config
    ) == pytest.approx(0.15)
    assert MODULE.require_profile_speed(
        'mirrored_90', 0.20, config
    ) == pytest.approx(0.20)
    with pytest.raises(ValueError, match=r"mirrored_90.*\(0, 0\.2\]"):
        MODULE.require_profile_speed('mirrored_90', 0.25, config)


def test_legacy_90_profile_exact_route_runtime_and_bounds():
    expected = (
        (0.0, 0.0),
        (0.0, 2.0),
        (-2.0, 2.0),
        (-2.0, 4.5),
        (0.0, 4.5),
        (0.0, 2.0),
        (0.0, 0.0),
    )
    config = MODULE.get_profile_config('legacy_90')

    assert MODULE.LEGACY_90_PATH_OFFSETS == expected
    assert config['path_offsets'] is MODULE.LEGACY_90_PATH_OFFSETS
    timing, length = MODULE.path_timing(expected, 0.15)
    assert length == pytest.approx(13.0)
    assert timing[-1] == pytest.approx(86.66666666666667)
    assert MODULE.scheduled_runtime_s(
        expected, 0.15, 2.0, 3.0
    ) == pytest.approx(91.66666666666667)
    assert config['default_speed_m_s'] == pytest.approx(0.15)
    assert config['max_speed_m_s'] == pytest.approx(0.15)
    assert config['default_hard_timeout_s'] == pytest.approx(95.0)
    assert config['default_max_displacement_m'] == pytest.approx(5.5)
    assert config['default_max_total_path_m'] == pytest.approx(15.0)
    assert config['default_max_yaw_change_rad'] == pytest.approx(4.0)
    assert config['default_saturation_timeout_s'] == pytest.approx(0.5)
    assert config['frame_mode'] == 'translated_vicon'
    assert config['max_start_position_error_m'] is None
    assert config['max_start_yaw_error_rad'] == pytest.approx(0.15)
    assert config['vicon_bounds_offsets'] == pytest.approx(
        (-2.3, 0.3, -0.3, 4.8)
    )
    assert MODULE.scheduled_runtime_s(
        expected, 0.15, 2.0, 3.0
    ) <= config['default_hard_timeout_s'] - 0.25


def test_legacy_90_all_boundaries_switch_on_time_without_zero_sample():
    points = MODULE.LEGACY_90_PATH_OFFSETS
    timing, _ = MODULE.path_timing(points, 0.15)
    epsilon = 1e-6

    for index in range(1, len(points) - 1):
        before = MODULE.sample_path(points, timing, timing[index] - epsilon)
        at_boundary = MODULE.sample_path(points, timing, timing[index])

        assert before[:2] == pytest.approx(points[index], abs=1e-6)
        assert math.hypot(*before[2:]) == pytest.approx(0.15)
        assert at_boundary[:2] == pytest.approx(points[index])
        assert math.hypot(*at_boundary[2:]) == pytest.approx(0.15)

    # The first 90-degree corner switches directly from +Y to -X.
    assert MODULE.sample_path(
        points, timing, timing[1] - epsilon
    )[2:] == pytest.approx((0.0, 0.15))
    assert MODULE.sample_path(
        points, timing, timing[1]
    )[2:] == pytest.approx((-0.15, 0.0))


def test_reference_schedule_has_only_start_and_final_zero_holds():
    points = MODULE.LEGACY_90_PATH_OFFSETS
    timing, _ = MODULE.path_timing(points, 0.15)
    start_hold = 2.0
    final_hold = 3.0
    motion_end = start_hold + timing[-1]
    publication_end = motion_end + final_hold

    assert MODULE.sample_reference_schedule(
        points, timing, start_hold - 1e-6, start_hold, final_hold
    )[2:] == pytest.approx((0.0, 0.0))
    assert MODULE.sample_reference_schedule(
        points, timing, start_hold, start_hold, final_hold
    )[2:] == pytest.approx((0.0, 0.15))
    assert MODULE.sample_reference_schedule(
        points, timing, start_hold + timing[1], start_hold, final_hold
    )[2:] == pytest.approx((-0.15, 0.0))
    assert MODULE.sample_reference_schedule(
        points, timing, motion_end, start_hold, final_hold
    ) == pytest.approx((*points[-1], 0.0, 0.0))
    assert MODULE.sample_reference_schedule(
        points, timing, publication_end - 1e-6, start_hold, final_hold
    ) == pytest.approx((*points[-1], 0.0, 0.0))
    assert MODULE.sample_reference_schedule(
        points, timing, publication_end, start_hold, final_hold
    ) is None


def test_legacy_profile_matches_waypoint_traj_simple_active_points():
    tree = ast.parse(WAYPOINT_SIMPLE.read_text(encoding='utf-8'))
    waypoint_literals = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(
            isinstance(target, ast.Name) and target.id == 'waypoints'
            for target in node.targets
        ):
            continue
        if not isinstance(node.value, ast.Call) or not node.value.args:
            continue
        waypoint_literals.append(ast.literal_eval(node.value.args[0]))

    assert len(waypoint_literals) == 1
    active_points = tuple(
        (float(x), float(y)) for x, y, yaw in waypoint_literals[0]
    )
    active_yaws = tuple(float(row[2]) for row in waypoint_literals[0])
    assert active_points == MODULE.LEGACY_90_PATH_OFFSETS
    assert active_yaws == pytest.approx((0.0,) * len(active_yaws))


def test_unknown_profile_is_rejected():
    with pytest.raises(ValueError, match='profile must be one of'):
        MODULE.get_profile_config('unknown')


def test_saturation_policy_is_strict_for_diagnostics_and_bounded_for_legacy():
    assert MODULE.get_profile_config(
        'vicon_straight'
    )['default_saturation_timeout_s'] == pytest.approx(0.0)
    assert MODULE.get_profile_config(
        'mirrored_90'
    )['default_saturation_timeout_s'] == pytest.approx(0.5)
    assert MODULE.saturation_timeout_exceeded(0.0, 0.0)
    assert not MODULE.saturation_timeout_exceeded(0.01, 0.5)
    assert not MODULE.saturation_timeout_exceeded(0.5, 0.5)
    assert MODULE.saturation_timeout_exceeded(0.501, 0.5)


def test_cleared_saturation_episode_enforces_timeout_inclusively():
    warnings = []
    logger = type('Logger', (), {
        'warn': lambda self, message: warnings.append(message),
    })()
    node = object.__new__(MODULE.ClosedLoopTurnTest)
    node.status_saturated = False
    node.saturation_started_time = 0.0
    node.max_saturation_episode_s = 0.0
    node.saturation_timeout_s = 0.05
    node.profile = 'vicon_straight'
    node.get_logger = lambda: logger

    node.assert_saturation_safety(0.05)

    assert node.saturation_started_time is None
    assert node.max_saturation_episode_s == pytest.approx(0.05)
    assert warnings == [
        'Wheel saturation cleared after 0.050s '
        '(profile=vicon_straight).'
    ]

    node.saturation_started_time = 1.0
    with pytest.raises(
        RuntimeError,
        match=r'saturation episode cleared after 0\.050s \(limit 0\.050s\)',
    ):
        node.assert_saturation_safety(1.050001)

    assert node.saturation_started_time is None
    assert node.max_saturation_episode_s == pytest.approx(0.050001)
    assert len(warnings) == 1


def test_vicon_straight_saturation_allowance_is_exact_and_speed_specific():
    config = MODULE.get_profile_config('vicon_straight')

    assert MODULE.require_profile_saturation_timeout(
        'vicon_straight', 0.15, 0.0, config
    ) == pytest.approx(0.0)
    assert MODULE.require_profile_saturation_timeout(
        'vicon_straight', 0.15, 0.05, config
    ) == pytest.approx(0.05)

    for speed, timeout in (
        (0.05, 0.05),
        (0.10, 0.05),
        (0.149, 0.05),
        (0.15, 0.049),
        (0.15, 0.050001),
        (0.15, -0.001),
        (0.15, math.nan),
        (0.15, math.inf),
    ):
        with pytest.raises(ValueError, match='saturation_timeout_s'):
            MODULE.require_profile_saturation_timeout(
                'vicon_straight', speed, timeout, config
            )

    mirrored = MODULE.get_profile_config('mirrored_90')
    assert MODULE.require_profile_saturation_timeout(
        'mirrored_90', 0.20, 0.5, mirrored
    ) == pytest.approx(0.5)


def test_vicon_straight_clean_preflight_is_exact_stage_only():
    config = MODULE.get_profile_config('vicon_straight')

    assert MODULE.require_profile_clean_preflight(
        'vicon_straight', 0.15, 30.0, config
    ) == pytest.approx(12.0)
    assert MODULE.require_profile_clean_preflight(
        'vicon_straight', 0.10, 6.0, config
    ) == pytest.approx(0.0)

    for startup_timeout_s in (29.999999, -1.0, math.nan, math.inf):
        with pytest.raises(ValueError, match=r'startup_timeout_s.*30\.0s'):
            MODULE.require_profile_clean_preflight(
                'vicon_straight', 0.15, startup_timeout_s, config
            )

    mirrored = MODULE.get_profile_config('mirrored_90')
    assert MODULE.require_profile_clean_preflight(
        'mirrored_90', 0.15, 6.0, mirrored
    ) == pytest.approx(0.0)


def test_exact_stage_clean_preflight_boundary_and_readiness_resets():
    node = object.__new__(MODULE.ClosedLoopTurnTest)
    node.clean_preflight_window_s = 12.0
    node.preflight_ready_since = None
    node.status_saturated = False
    node.odom_history = deque(((99.0, 0.0, 0.0, 0.0),))
    readiness = {'instantaneous': True, 'stationary': True}
    node._instantaneous_preflight_ready = (
        lambda now: readiness['instantaneous']
    )
    node._stationary = lambda now: readiness['stationary']

    assert not node._preflight_ready(100.0)
    assert node.preflight_ready_since == pytest.approx(100.0)
    assert not node._preflight_ready(111.999999)
    assert node._preflight_ready(112.0)

    readiness['instantaneous'] = False
    assert not node._preflight_ready(112.1)
    assert node.preflight_ready_since is None
    assert node.odom_history == deque()

    readiness['instantaneous'] = True
    assert not node._preflight_ready(113.0)
    readiness['stationary'] = False
    assert not node._preflight_ready(113.1)
    assert node.preflight_ready_since is None

    readiness['stationary'] = True
    assert not node._preflight_ready(114.0)
    node.status_saturated = True
    assert not node._preflight_ready(114.1)
    assert node.preflight_ready_since is None


def test_exact_stage_reuses_one_fixed_deadline_across_both_preflights():
    tree = ast.parse(SCRIPT.read_text(encoding='utf-8'))
    run_enabled = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == 'run_enabled'
    )
    waits = [
        node
        for node in ast.walk(run_enabled)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'wait_for_preflight'
    ]

    assert len(waits) == 2
    for call in waits:
        deadline = next(
            keyword.value
            for keyword in call.keywords
            if keyword.arg == 'deadline'
        )
        assert isinstance(deadline, ast.Name)
        assert deadline.id == 'clean_preflight_deadline'


def test_translated_vicon_bounds_are_inclusive_and_reject_escape():
    offsets = MODULE.get_profile_config(
        'legacy_90'
    )['vicon_bounds_offsets']
    bounds = MODULE.translate_bounds(offsets, start_x=5.0, start_y=-2.0)

    assert bounds == pytest.approx((2.7, 5.3, -2.3, 2.8))
    assert MODULE.point_in_bounds(2.7, -2.3, bounds)
    assert MODULE.point_in_bounds(5.3, 2.8, bounds)
    assert MODULE.point_in_bounds(4.0, 0.0, bounds)
    assert not MODULE.point_in_bounds(2.699, 0.0, bounds)
    assert not MODULE.point_in_bounds(5.301, 0.0, bounds)
    assert not MODULE.point_in_bounds(4.0, -2.301, bounds)
    assert not MODULE.point_in_bounds(4.0, 2.801, bounds)
    assert MODULE.point_in_bounds(1e9, -1e9, None)


def test_translated_profile_accepts_any_position_but_requires_vicon_yaw():
    for profile in ('vicon_straight', 'mirrored_90', 'legacy_90'):
        node = object.__new__(MODULE.ClosedLoopTurnTest)
        node.profile_config = MODULE.get_profile_config(profile)
        node.pose = (123.0, -87.0)
        node.raw_yaw = 0.14

        assert node._profile_start_ready()
        node.raw_yaw = 0.16
        assert not node._profile_start_ready()
        node.raw_yaw = None
        assert not node._profile_start_ready()


def test_source_stamp_status_requires_valid_nonzero_strictly_monotonic_data():
    valid, stamp_ns, reason = MODULE.source_stamp_status(9, 900_000_000)
    assert valid
    assert stamp_ns == 9_900_000_000
    assert reason == ''

    # Absolute source time can be far behind or ahead of the receiver clock;
    # only fields and source ordering are meaningful across these hosts.
    valid, stamp_ns, reason = MODULE.source_stamp_status(
        5000, 0, 9_900_000_000
    )
    assert valid
    assert stamp_ns == 5_000_000_000_000
    assert reason == ''

    for sec, nanosec, previous, expected_reason in (
        (0, 0, None, 'zero header timestamp'),
        (-1, 0, None, 'invalid header timestamp'),
        (1, 1_000_000_000, None, 'invalid header timestamp'),
        (2, 0, 2_000_000_000, 'non-monotonic source timestamp'),
        (1, 999_999_999, 2_000_000_000, 'non-monotonic source timestamp'),
    ):
        valid, _, reason = MODULE.source_stamp_status(
            sec, nanosec, previous
        )
        assert not valid
        assert reason == expected_reason


def test_base_pose_envelope_is_inclusive_normalized_and_yaw_invariant():
    pose = MODULE.Odometry().pose.pose
    pose.position.z = MODULE.DEFAULT_VICON_MIN_Z_M
    tilt = MODULE.DEFAULT_VICON_MAX_TILT_RAD
    yaw = 1.70
    sin_tilt = math.sin(0.5 * tilt)
    cos_tilt = math.cos(0.5 * tilt)
    sin_yaw = math.sin(0.5 * yaw)
    cos_yaw = math.cos(0.5 * yaw)
    scale = 3.0
    # q = q_yaw * q_roll, deliberately scaled away from unit length.
    pose.orientation.x = scale * cos_yaw * sin_tilt
    pose.orientation.y = scale * sin_yaw * sin_tilt
    pose.orientation.z = scale * sin_yaw * cos_tilt
    pose.orientation.w = scale * cos_yaw * cos_tilt

    valid, state, reason = MODULE.base_pose_envelope_status(
        pose,
        MODULE.DEFAULT_VICON_MIN_Z_M,
        MODULE.DEFAULT_VICON_MAX_Z_M,
        MODULE.DEFAULT_VICON_MAX_TILT_RAD,
    )
    assert valid
    assert state[0] == pytest.approx(tilt)
    assert state[1] == pytest.approx(yaw)
    assert reason == ''

    pose.position.z = MODULE.DEFAULT_VICON_MAX_Z_M
    assert MODULE.base_pose_envelope_status(
        pose,
        MODULE.DEFAULT_VICON_MIN_Z_M,
        MODULE.DEFAULT_VICON_MAX_Z_M,
        MODULE.DEFAULT_VICON_MAX_TILT_RAD,
    )[0]


@pytest.mark.parametrize(
    ('mutate', 'reason'),
    (
        (lambda pose: setattr(pose.position, 'z', 0.249), 'pose z='),
        (lambda pose: setattr(pose.position, 'z', 0.401), 'pose z='),
        (lambda pose: setattr(pose.position, 'x', math.inf), 'non-finite'),
        (lambda pose: setattr(pose.orientation, 'w', 0.0), 'quaternion'),
    ),
)
def test_base_pose_envelope_rejects_absolute_impossibilities(mutate, reason):
    pose = MODULE.Odometry().pose.pose
    pose.position.z = 0.32
    pose.orientation.w = 1.0
    mutate(pose)

    valid, state, detail = MODULE.base_pose_envelope_status(
        pose,
        MODULE.DEFAULT_VICON_MIN_Z_M,
        MODULE.DEFAULT_VICON_MAX_Z_M,
        MODULE.DEFAULT_VICON_MAX_TILT_RAD,
    )
    assert not valid
    assert state is None
    assert reason in detail


def test_base_pose_envelope_rejects_tilt_just_beyond_limit():
    pose = MODULE.Odometry().pose.pose
    pose.position.z = 0.32
    tilt = MODULE.DEFAULT_VICON_MAX_TILT_RAD + 0.001
    pose.orientation.x = math.sin(0.5 * tilt)
    pose.orientation.w = math.cos(0.5 * tilt)

    valid, _, reason = MODULE.base_pose_envelope_status(
        pose,
        MODULE.DEFAULT_VICON_MIN_Z_M,
        MODULE.DEFAULT_VICON_MAX_Z_M,
        MODULE.DEFAULT_VICON_MAX_TILT_RAD,
    )
    assert not valid
    assert 'tilt=' in reason


def test_controller_pose_envelope_must_exactly_match_supervisor():
    expected = {
        'vicon_min_z_m': 0.25,
        'vicon_max_z_m': 0.40,
        'vicon_max_tilt_rad': 0.35,
    }
    assert MODULE.VICON_POSE_ENVELOPE_PARAMETER_NAMES == tuple(expected)
    assert MODULE.require_matching_controller_pose_envelope(
        dict(expected), expected
    ) is None

    for name in expected:
        mismatch = dict(expected)
        mismatch[name] += 1e-6
        with pytest.raises(RuntimeError, match=name):
            MODULE.require_matching_controller_pose_envelope(
                mismatch, expected
            )
    with pytest.raises(RuntimeError, match='vicon_max_z_m'):
        MODULE.require_matching_controller_pose_envelope(
            {'vicon_min_z_m': 0.25}, expected
        )


def test_hardware_supervisor_requires_world_odom_twist_velocity_source():
    assert MODULE.REQUIRED_XY_VELOCITY_SOURCE == 'odom_twist_world'
    assert MODULE.require_controller_xy_velocity_source(
        'odom_twist_world'
    ) is None

    for value in ('pose_delta_world', '', None, True):
        with pytest.raises(
            RuntimeError,
            match='controller xy_velocity_source must be',
        ):
            MODULE.require_controller_xy_velocity_source(value)

    tree = ast.parse(SCRIPT.read_text(encoding='utf-8'))
    queried_parameter_names = {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant)
        and isinstance(node.value, str)
    }
    assert 'xy_velocity_source' in queried_parameter_names
    assert 'vicon_source_stamp_policy' in queried_parameter_names
    assert set(MODULE.VICON_POSE_ENVELOPE_PARAMETER_NAMES).issubset(
        queried_parameter_names
    )


def test_supervisor_accepts_absolute_offset_but_not_nonmonotonic_source_stamp():
    node = object.__new__(MODULE.ClosedLoopTurnTest)
    node.vicon_source_stamp_policy = MODULE.VICON_SOURCE_STAMP_POLICY
    node.last_odom_source_stamp_ns = None
    node.odom_source_error = None
    node.latched_odom_source_error = None
    node.vicon_min_z_m = MODULE.DEFAULT_VICON_MIN_Z_M
    node.vicon_max_z_m = MODULE.DEFAULT_VICON_MAX_Z_M
    node.vicon_max_tilt_rad = MODULE.DEFAULT_VICON_MAX_TILT_RAD
    node.odom_pose_envelope_error = None
    node.latched_odom_pose_envelope_error = None
    node.base_z = None
    node.base_tilt = None
    node.last_raw_yaw = None
    node.unwrapped_yaw = None
    node.raw_yaw = None
    node.pose = None
    node.last_odom_time = None
    node.odom_history = deque()
    node.stationary_window_s = 1.0
    node.armed = False
    node.get_clock = lambda: type('Clock', (), {
        'now': lambda self: type('Time', (), {
            'nanoseconds': 10_000_000_000,
        })(),
    })()

    msg = MODULE.Odometry()
    msg.header.stamp.sec = 9
    msg.header.stamp.nanosec = 800_000_000
    msg.pose.pose.orientation.w = 1.0
    msg.pose.pose.position.z = 0.32
    node.odom_cb(msg)

    # This source clock is 200 ms behind the synthetic receiver clock, but the
    # callback is accepted because absolute cross-host age is not consulted.
    assert node.pose == pytest.approx((0.0, 0.0))
    assert node.last_odom_time is not None
    assert node.odom_source_error is None
    assert node.latched_odom_source_error is None

    msg.header.stamp.sec = 9
    msg.header.stamp.nanosec = 950_000_000
    msg.pose.pose.position.x = 1.25
    node.odom_cb(msg)

    assert node.pose == pytest.approx((1.25, 0.0))
    assert node.last_odom_time is not None
    assert node.odom_source_error is None
    assert node.latched_odom_source_error is None

    accepted_pose = node.pose
    accepted_time = node.last_odom_time
    history = tuple(node.odom_history)
    node.armed = True
    node.last_path_pose = accepted_pose
    node.total_path_m = 0.0
    msg.header.stamp.sec = 9
    msg.header.stamp.nanosec = 800_000_000
    msg.pose.pose.position.x = 99.0
    node.odom_cb(msg)

    assert node.pose == accepted_pose
    assert node.last_odom_time == accepted_time
    assert tuple(node.odom_history) == history
    assert node.total_path_m == 0.0
    assert node.odom_source_error == (
        'base Vicon non-monotonic source timestamp'
    )
    assert node.latched_odom_source_error == node.odom_source_error
    assert node._fresh(accepted_time, 0.12, accepted_time + 0.12)
    assert not node._fresh(
        accepted_time,
        0.12,
        accepted_time + 0.120001,
    )


def test_supervisor_receipt_watchdog_aborts_from_last_accepted_time():
    node = object.__new__(MODULE.ClosedLoopTurnTest)
    node.hard_timeout_s = 10.0
    node.latched_odom_source_error = None
    node.odom_source_error = None
    node.latched_odom_pose_envelope_error = None
    node.odom_pose_envelope_error = None
    node.last_odom_time = 1.0
    node.stale_odom_s = 0.12
    with pytest.raises(
        RuntimeError,
        match='base Vicon odometry receipt became stale',
    ):
        node.assert_active_safety(now=1.120001, armed_elapsed=0.5)


def test_stable_wrong_pose_cannot_establish_preflight_baseline():
    node = object.__new__(MODULE.ClosedLoopTurnTest)
    node.vicon_source_stamp_policy = MODULE.VICON_SOURCE_STAMP_POLICY
    node.vicon_min_z_m = MODULE.DEFAULT_VICON_MIN_Z_M
    node.vicon_max_z_m = MODULE.DEFAULT_VICON_MAX_Z_M
    node.vicon_max_tilt_rad = MODULE.DEFAULT_VICON_MAX_TILT_RAD
    node.last_odom_source_stamp_ns = None
    node.odom_source_error = None
    node.latched_odom_source_error = None
    node.odom_pose_envelope_error = None
    node.latched_odom_pose_envelope_error = None
    node.base_z = None
    node.base_tilt = None
    node.last_raw_yaw = None
    node.unwrapped_yaw = None
    node.raw_yaw = None
    node.pose = None
    node.last_odom_time = None
    node.odom_history = deque()
    node.stationary_window_s = 1.0
    node.armed = False
    node.get_clock = lambda: type('Clock', (), {
        'now': lambda self: type('Time', (), {
            'nanoseconds': 10_000_000_000,
        })(),
    })()

    wrong = MODULE.Odometry()
    wrong.header.stamp.sec = 9
    wrong.header.stamp.nanosec = 950_000_000
    wrong.pose.pose.position.z = 0.44287
    wrong.pose.pose.orientation.w = 1.0
    for index in range(20):
        wrong.header.stamp.nanosec = 950_000_000 + index * 1_000_000
        node.odom_cb(wrong)

    assert node.pose is None
    assert node.last_odom_time is None
    assert node.odom_history == deque()
    assert 'pose z=' in node.odom_pose_envelope_error
    assert node.latched_odom_pose_envelope_error is None

    recovered = MODULE.Odometry()
    recovered.header.stamp.sec = 9
    recovered.header.stamp.nanosec = 970_000_000
    recovered.pose.pose.position.z = 0.326
    recovered.pose.pose.orientation.w = 1.0
    node.odom_cb(recovered)

    assert node.pose == pytest.approx((0.0, 0.0))
    assert node.last_odom_time is not None
    assert node.odom_pose_envelope_error is None


def test_clean_preflight_resets_on_reject_gap_and_heading_misalignment(
    monkeypatch,
):
    monotonic_now = [100.0]
    monkeypatch.setattr(
        MODULE.time, 'monotonic', lambda: monotonic_now[0]
    )
    node = object.__new__(MODULE.ClosedLoopTurnTest)
    node.profile_config = MODULE.get_profile_config('vicon_straight')
    node.clean_preflight_window_s = 12.0
    node.preflight_ready_since = 88.1
    node.stale_odom_s = 0.12
    node.vicon_source_stamp_policy = MODULE.VICON_SOURCE_STAMP_POLICY
    node.vicon_min_z_m = MODULE.DEFAULT_VICON_MIN_Z_M
    node.vicon_max_z_m = MODULE.DEFAULT_VICON_MAX_Z_M
    node.vicon_max_tilt_rad = MODULE.DEFAULT_VICON_MAX_TILT_RAD
    node.last_odom_source_stamp_ns = 9_900_000_000
    node.odom_source_error = None
    node.latched_odom_source_error = None
    node.odom_pose_envelope_error = None
    node.latched_odom_pose_envelope_error = None
    node.base_z = None
    node.base_tilt = None
    node.last_raw_yaw = None
    node.unwrapped_yaw = None
    node.raw_yaw = None
    node.pose = None
    node.last_odom_time = None
    node.odom_history = deque(((99.0, 0.0, 0.0, 0.0),))
    node.stationary_window_s = 1.0
    node.armed = False
    node.get_clock = lambda: type('Clock', (), {
        'now': lambda self: type('Time', (), {
            'nanoseconds': 10_000_000_000,
        })(),
    })()

    next_stamp_ns = [9_950_000_000]

    def odom(stamp_ns=None, z=0.32, yaw=0.0):
        if stamp_ns is None:
            stamp_ns = next_stamp_ns[0]
            next_stamp_ns[0] += 10_000_000
        msg = MODULE.Odometry()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
            stamp_ns, 1_000_000_000
        )
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.pose.orientation.w = math.cos(0.5 * yaw)
        return msg

    # A reversed source stamp discards both accumulated windows.
    node.odom_cb(odom(stamp_ns=9_800_000_000))
    assert node.preflight_ready_since is None
    assert node.odom_history == deque()

    node.odom_cb(odom())
    assert node.last_odom_time == pytest.approx(100.0)
    node.preflight_ready_since = 88.1

    # The receipt watchdog boundary is inclusive.
    monotonic_now[0] = 100.12
    node.odom_cb(odom())
    assert node.preflight_ready_since == pytest.approx(88.1)
    assert len(node.odom_history) == 2

    # Just beyond the boundary starts over and cannot reuse old history.
    monotonic_now[0] = 100.240002
    node.odom_cb(odom())
    assert node.preflight_ready_since is None
    assert len(node.odom_history) == 1

    node.preflight_ready_since = 88.4
    node.odom_cb(odom(z=0.45))
    assert node.preflight_ready_since is None
    assert node.odom_history == deque()

    node.preflight_ready_since = 88.4
    node.odom_cb(odom(yaw=0.16))
    assert node.preflight_ready_since is None
    assert len(node.odom_history) == 1

    # Accepted-gap bookkeeping is preflight-only; active safety semantics are
    # left to the existing active guards and are not replaced by this reset.
    node.armed = True
    node.last_path_pose = node.pose
    node.total_path_m = 0.0
    node.preflight_ready_since = 90.0
    monotonic_now[0] = 100.5
    node.odom_cb(odom(yaw=0.0))
    assert node.preflight_ready_since == pytest.approx(90.0)


def test_active_pose_envelope_failure_survives_fresh_burst_and_aborts():
    node = object.__new__(MODULE.ClosedLoopTurnTest)
    node.vicon_source_stamp_policy = MODULE.VICON_SOURCE_STAMP_POLICY
    node.vicon_min_z_m = MODULE.DEFAULT_VICON_MIN_Z_M
    node.vicon_max_z_m = MODULE.DEFAULT_VICON_MAX_Z_M
    node.vicon_max_tilt_rad = MODULE.DEFAULT_VICON_MAX_TILT_RAD
    node.last_odom_source_stamp_ns = None
    node.odom_source_error = None
    node.latched_odom_source_error = None
    node.odom_pose_envelope_error = None
    node.latched_odom_pose_envelope_error = None
    node.base_z = None
    node.base_tilt = None
    node.last_raw_yaw = None
    node.unwrapped_yaw = None
    node.raw_yaw = None
    node.pose = None
    node.last_odom_time = None
    node.odom_history = deque()
    node.stationary_window_s = 1.0
    node.armed = True
    node.last_path_pose = None
    node.total_path_m = 0.0
    node.hard_timeout_s = 10.0
    node.get_clock = lambda: type('Clock', (), {
        'now': lambda self: type('Time', (), {
            'nanoseconds': 10_000_000_000,
        })(),
    })()

    wrong = MODULE.Odometry()
    wrong.header.stamp.sec = 9
    wrong.header.stamp.nanosec = 950_000_000
    wrong.pose.pose.position.z = 0.44287
    wrong.pose.pose.orientation.w = 1.0
    node.odom_cb(wrong)

    latched_error = node.latched_odom_pose_envelope_error
    assert latched_error is not None
    assert 'pose z=' in latched_error

    fresh = MODULE.Odometry()
    fresh.header.stamp.sec = 9
    fresh.header.stamp.nanosec = 960_000_000
    fresh.pose.pose.position.z = 0.326
    fresh.pose.pose.orientation.w = 1.0
    node.odom_cb(fresh)

    assert node.odom_pose_envelope_error is None
    assert node.latched_odom_pose_envelope_error == latched_error
    with pytest.raises(RuntimeError, match='base Vicon pose z='):
        node.assert_active_safety(now=0.0, armed_elapsed=0.0)


def test_active_source_rejection_survives_fresh_burst_and_aborts():
    node = object.__new__(MODULE.ClosedLoopTurnTest)
    node.vicon_source_stamp_policy = MODULE.VICON_SOURCE_STAMP_POLICY
    node.last_odom_source_stamp_ns = None
    node.odom_source_error = None
    node.latched_odom_source_error = None
    node.vicon_min_z_m = MODULE.DEFAULT_VICON_MIN_Z_M
    node.vicon_max_z_m = MODULE.DEFAULT_VICON_MAX_Z_M
    node.vicon_max_tilt_rad = MODULE.DEFAULT_VICON_MAX_TILT_RAD
    node.odom_pose_envelope_error = None
    node.latched_odom_pose_envelope_error = None
    node.base_z = None
    node.base_tilt = None
    node.last_raw_yaw = None
    node.unwrapped_yaw = None
    node.raw_yaw = None
    node.pose = None
    node.last_odom_time = None
    node.odom_history = deque()
    node.stationary_window_s = 1.0
    node.armed = True
    node.last_path_pose = None
    node.total_path_m = 0.0
    node.hard_timeout_s = 10.0
    node.get_clock = lambda: type('Clock', (), {
        'now': lambda self: type('Time', (), {
            'nanoseconds': 10_000_000_000,
        })(),
    })()

    baseline = MODULE.Odometry()
    baseline.header.stamp.sec = 9
    baseline.header.stamp.nanosec = 950_000_000
    baseline.pose.pose.orientation.w = 1.0
    baseline.pose.pose.position.z = 0.32
    node.odom_cb(baseline)

    duplicate = MODULE.Odometry()
    duplicate.header.stamp.sec = 9
    duplicate.header.stamp.nanosec = 950_000_000
    duplicate.pose.pose.orientation.w = 1.0
    duplicate.pose.pose.position.z = 0.32
    node.odom_cb(duplicate)

    latched_error = node.latched_odom_source_error
    assert latched_error is not None
    assert 'non-monotonic' in latched_error

    fresh = MODULE.Odometry()
    fresh.header.stamp.sec = 9
    fresh.header.stamp.nanosec = 960_000_000
    fresh.pose.pose.orientation.w = 1.0
    fresh.pose.pose.position.z = 0.32
    node.odom_cb(fresh)

    assert node.odom_source_error is None
    assert node.latched_odom_source_error == latched_error
    with pytest.raises(RuntimeError, match='base Vicon.*non-monotonic'):
        node.assert_active_safety(now=0.0, armed_elapsed=0.0)


def test_path_timing_rejects_unsafe_inputs():
    with pytest.raises(ValueError):
        MODULE.path_timing(((0.0, 0.0), (1.0, 0.0)), 0.0)
    with pytest.raises(ValueError):
        MODULE.path_timing(((0.0, 0.0),), 0.08)
    with pytest.raises(ValueError):
        MODULE.path_timing(((0.0, 0.0), (0.0, 0.0)), 0.08)
