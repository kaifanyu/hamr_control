import ast
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

from hamr_control.hamr_controller import (
    HARDWARE_ODOM_QOS,
    HamrControlNode,
    apply_turret_command_policy,
    timestamp_is_fresh,
    turret_state_is_ready,
)


def test_hardware_odom_subscriptions_use_latest_only_best_effort_qos():
    assert HARDWARE_ODOM_QOS.depth == 1
    assert HARDWARE_ODOM_QOS.history == QoSHistoryPolicy.KEEP_LAST
    assert HARDWARE_ODOM_QOS.reliability == QoSReliabilityPolicy.BEST_EFFORT
    assert HARDWARE_ODOM_QOS.durability == QoSDurabilityPolicy.VOLATILE

    controller_path = (
        Path(__file__).resolve().parents[1]
        / 'hamr_control'
        / 'hamr_controller.py'
    )
    tree = ast.parse(controller_path.read_text(encoding='utf-8'))
    subscriptions = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == 'create_subscription'
        and len(call.args) == 4
        and isinstance(call.args[1], ast.Constant)
    ]
    by_topic = {call.args[1].value: call for call in subscriptions}

    for topic in ('HAMR_base/odom', 'HAMR_turret/odom'):
        qos_argument = by_topic[topic].args[3]
        assert isinstance(qos_argument, ast.Name)
        assert qos_argument.id == 'HARDWARE_ODOM_QOS'

    simulation_qos = by_topic['/hamr/odom'].args[3]
    assert isinstance(simulation_qos, ast.Constant)
    assert simulation_qos.value == 1


def test_timestamp_freshness_requires_a_sample_and_rejects_clock_rewind():
    assert not timestamp_is_fresh(1_000, None, 0.25)
    assert not timestamp_is_fresh(999, 1_000, 0.25)


def test_timestamp_freshness_boundary_and_disabled_timeout():
    last_update_ns = 1_000_000_000

    assert timestamp_is_fresh(1_250_000_000, last_update_ns, 0.25)
    assert not timestamp_is_fresh(1_250_000_001, last_update_ns, 0.25)
    assert timestamp_is_fresh(20_000_000_000, last_update_ns, 0.0)
    assert not timestamp_is_fresh(20_000_000_000, last_update_ns, math.nan)


def test_disabled_turret_needs_no_feedback():
    assert turret_state_is_ready(False, False, None, None)
    assert turret_state_is_ready(False, True, None, None)


def test_enabled_turret_requires_feedback_for_the_active_mode():
    orientation = object()

    assert turret_state_is_ready(True, True, orientation, None)
    assert not turret_state_is_ready(True, True, None, orientation)
    assert turret_state_is_ready(True, False, None, orientation)
    assert not turret_state_is_ready(True, False, orientation, None)


def test_disabled_turret_command_is_hard_zero_without_changing_wheels():
    original = np.array([2.0, -1.0, 7.0])

    disabled = apply_turret_command_policy(original, False)
    enabled = apply_turret_command_policy(original, True)

    assert np.array_equal(disabled, np.array([2.0, -1.0, 0.0]))
    assert np.array_equal(enabled, original)
    assert np.array_equal(original, np.array([2.0, -1.0, 7.0]))


@pytest.mark.parametrize('nonfinite', (math.nan, math.inf, -math.inf))
def test_joint_command_boundary_publishes_only_zeros_for_nonfinite_output(
    nonfinite,
):
    class CapturingPublisher:
        def __init__(self):
            self.values = []

        def publish(self, message):
            self.values.append(message.data)

    controller = object.__new__(HamrControlNode)
    controller.right_wheel_vel_ = CapturingPublisher()
    controller.left_wheel_vel_ = CapturingPublisher()
    controller.turret_vel_ = CapturingPublisher()
    controller.compute_velocities = lambda *_: np.array(
        [nonfinite, 1.0, 0.0]
    )
    controller.turret_enabled = True
    controller.wheel_speed_limit_rad_s = 3.0
    controller.last_invalid_command_log_ns = None
    controller.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)
    )
    errors = []
    controller.get_logger = lambda: SimpleNamespace(
        error=lambda message: errors.append(message)
    )
    resets = []
    controller.reset_pid_state = lambda: resets.append('reset')

    assert not controller.publish_joint_cmd((0.2, 0.0, 0.0), 0.0)

    assert controller.right_wheel_vel_.values == [0.0]
    assert controller.left_wheel_vel_.values == [0.0]
    assert controller.turret_vel_.values == [0.0]
    assert resets == ['reset']
    assert len(errors) == 1


def test_turret_command_policy_rejects_malformed_joint_vectors():
    try:
        apply_turret_command_policy([1.0, 2.0], False)
    except ValueError as exc:
        assert "three joint" in str(exc)
    else:
        raise AssertionError("malformed joint commands must be rejected")


def test_shutdown_is_idempotent_and_repeats_zero_command():
    class FakeTimer:
        def __init__(self):
            self.cancel_count = 0

        def cancel(self):
            self.cancel_count += 1

    controller = object.__new__(HamrControlNode)
    controller.shutdown_started = False
    controller.control_timer_ = FakeTimer()
    events = []
    controller.reset_pid_state = lambda: events.append("reset")
    controller.publish_zero_cmd = lambda: events.append("zero")

    controller.shutdown_controller()
    controller.shutdown_controller()

    assert controller.control_timer_.cancel_count == 1
    assert events == ["reset", "zero", "zero", "zero"]
