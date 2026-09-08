import math
from types import SimpleNamespace

import numpy as np
import pytest

from hamr_control.hamr_controller import (
    HamrControlNode,
    PIAccumulator,
    WorldVelocityEstimator,
    compensated_xy_feedforward,
    time_scaled_filter_alpha,
    tracking_velocity_error,
    validated_xy_feedforward_config,
)


def make_pid_controller(
    reference_position=(0.1, 0.0),
    reference_velocity=(0.0, 0.0),
    measured_position=(0.0, 0.0),
    measured_velocity=(0.0, 0.0),
):
    """Build the non-ROS state needed to exercise ``pid_step`` directly."""
    controller = object.__new__(HamrControlNode)
    controller.reference_ = SimpleNamespace(
        x=float(reference_position[0]),
        y=float(reference_position[1]),
        yaw=0.0,
        x_dot=float(reference_velocity[0]),
        y_dot=float(reference_velocity[1]),
        yaw_dot=0.0,
    )
    controller.pose_base_ = SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(
                x=float(measured_position[0]),
                y=float(measured_position[1]),
            ),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )
    controller.measured_world_velocity = measured_velocity
    controller.world_velocity_estimator = WorldVelocityEstimator()
    controller.hamr_config = {
        "base_yaw_offset": 0.0,
        "simulating": False,
    }
    controller.turret_enabled = False
    controller.gains = {
        "x": {"P": 0.7, "I": 0.0, "D": 0.7},
        "y": {"P": 0.7, "I": 0.0, "D": 0.7},
        "yaw": {"P": 0.8, "I": 1.0, "D": 0.05},
    }
    controller.control_rate_hz = 100.0
    controller.dt = 0.01
    controller.d_alpha = 0.2
    controller.xy_feedforward_gain = 1.0
    controller.xy_feedforward_max_extra_m_s = 0.03
    controller.last_xy_feedforward_extra = (0.0, 0.0)
    controller.pid_needs_prime = True
    controller.err_x_prev = 0.0
    controller.err_y_prev = 0.0
    controller.err_yaw_prev = 0.0
    controller.d_err_x_filt = 0.0
    controller.d_err_y_filt = 0.0
    controller.d_err_yaw_filt = 0.0
    controller.I_x = PIAccumulator(limit=0.5)
    controller.I_y = PIAccumulator(limit=0.5)
    controller.I_yaw = PIAccumulator(limit=1.0)
    controller.threshold_x_y = 0.02
    controller.threshold_yaw = 0.1
    controller.xy_dot_limit = 0.8
    controller.yaw_dot_limit = 2.0
    controller.gain_reports = []
    controller.body_commands = []
    controller.get_logger = lambda: SimpleNamespace(warn=lambda _message: None)
    controller.publish_live_gains = (
        lambda *terms: controller.gain_reports.append(tuple(terms))
    )
    controller.publish_joint_cmd = (
        lambda desired, yaw: controller.body_commands.append(
            np.asarray(desired, dtype=float).copy()
        )
    )
    return controller


def run_pid_step(
    controller,
    *,
    reference_position=None,
    reference_velocity=None,
    measured_position=None,
    measured_velocity=None,
    dt=None,
):
    if reference_position is not None:
        controller.reference_.x = float(reference_position[0])
        controller.reference_.y = float(reference_position[1])
    if reference_velocity is not None:
        controller.reference_.x_dot = float(reference_velocity[0])
        controller.reference_.y_dot = float(reference_velocity[1])
    if measured_position is not None:
        controller.pose_base_.pose.position.x = float(measured_position[0])
        controller.pose_base_.pose.position.y = float(measured_position[1])
    if measured_velocity is not None:
        controller.measured_world_velocity = tuple(measured_velocity)
    if dt is not None:
        controller.dt = float(dt)
    controller.pid_step()
    return controller.gain_reports[-1], controller.body_commands[-1]


def test_time_scaled_filter_alpha_preserves_nominal_smoothing_over_time():
    nominal = time_scaled_filter_alpha(0.2, 0.01, 0.01)
    half_step = time_scaled_filter_alpha(0.2, 0.005, 0.01)
    double_step = time_scaled_filter_alpha(0.2, 0.02, 0.01)

    assert nominal == pytest.approx(0.2)
    assert 1.0 - (1.0 - half_step) ** 2 == pytest.approx(nominal)
    assert double_step == pytest.approx(1.0 - (1.0 - nominal) ** 2)
    assert time_scaled_filter_alpha(0.2, 0.0, 0.01) == 0.0
    assert time_scaled_filter_alpha(0.2, math.nan, 0.01) == 0.0


def test_world_velocity_estimator_uses_irregular_source_intervals():
    estimator = WorldVelocityEstimator()
    velocity = (0.23, -0.11)
    stamp_ns = 1_000_000_000
    elapsed_s = 0.0

    assert estimator.observe(stamp_ns, (0.0, 0.0)) is None
    for interval_ms in (8, 17, 11, 15, 9):
        elapsed_s += interval_ms * 1e-3
        stamp_ns += interval_ms * 1_000_000
        estimate = estimator.observe(
            stamp_ns,
            (velocity[0] * elapsed_s, velocity[1] * elapsed_s),
        )
        assert estimate == pytest.approx(velocity, abs=1e-12)


def test_world_velocity_estimator_reprimes_repeated_or_backward_stamps():
    estimator = WorldVelocityEstimator()
    estimator.observe(1_000_000_000, (0.0, 0.0))
    assert estimator.observe(1_010_000_000, (0.01, -0.02)) == pytest.approx(
        (1.0, -2.0)
    )

    assert estimator.observe(1_010_000_000, (4.0, 5.0)) is None
    assert estimator.velocity is None
    assert estimator.observe(1_020_000_000, (4.01, 4.98)) == pytest.approx(
        (1.0, -2.0)
    )

    assert estimator.observe(1_005_000_000, (7.0, 8.0)) is None
    assert estimator.velocity is None
    assert estimator.observe(1_015_000_000, (7.01, 7.98)) == pytest.approx(
        (1.0, -2.0)
    )


def test_tracking_velocity_error_rejects_unknown_or_nonfinite_inputs():
    assert tracking_velocity_error((0.15, -0.02), (0.10, 0.03)) == (
        pytest.approx(0.05),
        pytest.approx(-0.05),
    )
    assert tracking_velocity_error((0.15, 0.0), None) is None
    assert tracking_velocity_error((math.nan, 0.0), (0.0, 0.0)) is None


def test_xy_feedforward_neutral_gain_does_not_change_reference():
    assert compensated_xy_feedforward((0.15, -0.06), 1.0, 0.03) == (
        pytest.approx(0.15),
        pytest.approx(-0.06),
    )
    assert compensated_xy_feedforward((0.0, 0.0), 1.5, 0.03) == (
        pytest.approx(0.0),
        pytest.approx(0.0),
    )


def test_xy_feedforward_applies_gain_without_changing_vector_direction():
    compensated = compensated_xy_feedforward((0.12, -0.05), 1.1, 0.03)

    assert compensated == pytest.approx((0.132, -0.055), abs=1e-15)
    assert compensated[1] / compensated[0] == pytest.approx(-0.05 / 0.12)


def test_xy_feedforward_caps_added_vector_norm_not_each_axis():
    reference = (0.3, 0.4)
    compensated = compensated_xy_feedforward(reference, 1.5, 0.03)
    extra = (
        compensated[0] - reference[0],
        compensated[1] - reference[1],
    )

    assert math.hypot(*extra) == pytest.approx(0.03, abs=1e-15)
    assert extra == pytest.approx((0.018, 0.024), abs=1e-15)


@pytest.mark.parametrize(
    ("gain", "max_extra_m_s"),
    [
        (0.99, 0.03),
        (1.51, 0.03),
        (1.1, -0.001),
        (1.1, 0.051),
        (math.nan, 0.03),
        (1.1, math.inf),
    ],
)
def test_xy_feedforward_configuration_rejects_unsafe_values(
    gain, max_extra_m_s
):
    with pytest.raises(ValueError):
        validated_xy_feedforward_config(gain, max_extra_m_s)


def test_xy_feedforward_runtime_parameter_validation_is_transactional():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard = SimpleNamespace(
        min_z_m=0.25,
        max_z_m=0.40,
        max_tilt_rad=0.35,
    )
    controller.xy_feedforward_gain = 1.0
    controller.xy_feedforward_max_extra_m_s = 0.03

    accepted = controller.validate_safety_parameters_callback(
        [
            SimpleNamespace(name="xy_feedforward_gain", value=1.1),
            SimpleNamespace(
                name="xy_feedforward_max_extra_m_s", value=0.025
            ),
        ]
    )
    rejected = controller.validate_safety_parameters_callback(
        [
            SimpleNamespace(name="xy_feedforward_gain", value=1.1),
            SimpleNamespace(
                name="xy_feedforward_max_extra_m_s", value=0.25
            ),
        ]
    )

    assert accepted.successful
    assert not rejected.successful
    assert "xy_feedforward_max_extra_m_s" in rejected.reason


def test_xy_feedforward_runtime_change_resets_pid_history_once():
    class FakeLogger:
        def __init__(self):
            self.infos = []
            self.errors = []

        def info(self, message):
            self.infos.append(message)

        def error(self, message):
            self.errors.append(message)

    controller = object.__new__(HamrControlNode)
    controller.xy_feedforward_gain = 1.0
    controller.xy_feedforward_max_extra_m_s = 0.03
    resets = []
    logger = FakeLogger()
    controller.reset_pid_state = lambda: resets.append("reset")
    controller.get_logger = lambda: logger

    controller.parameters_callback(
        [
            SimpleNamespace(name="xy_feedforward_gain", value=1.1),
            SimpleNamespace(
                name="xy_feedforward_max_extra_m_s", value=0.025
            ),
        ]
    )

    assert controller.xy_feedforward_gain == pytest.approx(1.1)
    assert controller.xy_feedforward_max_extra_m_s == pytest.approx(0.025)
    assert resets == ["reset"]
    assert not logger.errors

    # Direct invocation with an invalid value keeps the complete previous
    # configuration and does not disturb the running controller state.
    controller.parameters_callback(
        [SimpleNamespace(name="xy_feedforward_gain", value=2.0)]
    )
    assert controller.xy_feedforward_gain == pytest.approx(1.1)
    assert controller.xy_feedforward_max_extra_m_s == pytest.approx(0.025)
    assert resets == ["reset"]
    assert "Ignoring invalid" in logger.errors[-1]


def test_pid_derivative_is_unchanged_by_position_setpoint_steps():
    controller = make_pid_controller(
        reference_position=(0.1, 0.0),
        reference_velocity=(0.0, 0.0),
        measured_velocity=(0.0, 0.0),
    )

    run_pid_step(controller)
    gains, _ = run_pid_step(controller, reference_position=(0.6, -0.4))

    assert gains[1] == pytest.approx(0.0, abs=1e-15)  # D_x
    assert gains[4] == pytest.approx(0.0, abs=1e-15)  # D_y


def test_stationary_robot_velocity_feedback_is_smooth_under_50hz_holds():
    stepped = make_pid_controller(
        reference_position=(0.1, 0.0),
        reference_velocity=(0.15, 0.0),
        measured_velocity=(0.0, 0.0),
    )
    held = make_pid_controller(
        reference_position=(0.1, 0.0),
        reference_velocity=(0.15, 0.0),
        measured_velocity=(0.0, 0.0),
    )
    stepped_d = []
    held_d = []

    for tick in range(31):
        if tick and tick % 2 == 0:
            stepped.reference_.x += 0.003
        stepped_gains, _ = run_pid_step(stepped)
        held_gains, _ = run_pid_step(held)
        stepped_d.append(stepped_gains[1])
        held_d.append(held_gains[1])

    assert stepped_d == pytest.approx(held_d, abs=1e-15)
    assert all(
        later >= earlier
        for earlier, later in zip(stepped_d, stepped_d[1:])
    )
    expected = 0.7 * 0.15 * (1.0 - 0.8 ** 30)
    assert stepped_d[-1] == pytest.approx(expected, abs=1e-12)


def test_async_50_82_100hz_perfect_tracking_has_no_derivative_chatter():
    velocity = (0.15, -0.06)
    controller = make_pid_controller(
        reference_position=(0.0, 0.0),
        reference_velocity=velocity,
        measured_position=(0.0, 0.0),
        measured_velocity=(0.0, 0.0),
    )
    estimator = WorldVelocityEstimator()
    source_origin_ns = 1_000_000_000
    assert estimator.observe(source_origin_ns, (0.0, 0.0)) is None
    controller.measured_world_velocity = None

    next_reference_ns = 0
    next_odom_ns = 12_000_000
    odom_intervals_ns = (12_000_000, 12_000_000, 13_000_000, 12_000_000)
    interval_index = 0
    derivative_terms = []

    for control_ns in range(0, 1_000_000_001, 10_000_000):
        while next_reference_ns <= control_ns:
            reference_time_s = next_reference_ns * 1e-9
            controller.reference_.x = velocity[0] * reference_time_s
            controller.reference_.y = velocity[1] * reference_time_s
            next_reference_ns += 20_000_000

        while next_odom_ns <= control_ns:
            odom_time_s = next_odom_ns * 1e-9
            position = (
                velocity[0] * odom_time_s,
                velocity[1] * odom_time_s,
            )
            controller.pose_base_.pose.position.x = position[0]
            controller.pose_base_.pose.position.y = position[1]
            controller.measured_world_velocity = estimator.observe(
                source_origin_ns + next_odom_ns,
                position,
            )
            next_odom_ns += odom_intervals_ns[interval_index]
            interval_index = (interval_index + 1) % len(odom_intervals_ns)

        gains, _ = run_pid_step(controller, dt=0.01)
        derivative_terms.append((gains[1], gains[4]))

    # Ignore initial estimator priming; all later source-time pose differences
    # exactly match the analytic trajectory velocity despite asynchronous holds.
    assert np.max(np.abs(np.asarray(derivative_terms[3:]))) < 1e-12


def test_pid_filter_response_is_independent_of_control_tick_partitioning():
    one_step = make_pid_controller(
        reference_velocity=(0.15, 0.0), measured_velocity=(0.0, 0.0)
    )
    two_steps = make_pid_controller(
        reference_velocity=(0.15, 0.0), measured_velocity=(0.0, 0.0)
    )
    run_pid_step(one_step)
    run_pid_step(two_steps)

    one_gains, _ = run_pid_step(one_step, dt=0.01)
    run_pid_step(two_steps, dt=0.005)
    two_gains, _ = run_pid_step(two_steps, dt=0.005)

    assert two_gains[1] == pytest.approx(one_gains[1], abs=1e-15)


def test_pid_uses_compensated_feedforward_but_original_velocity_objective():
    controller = make_pid_controller(
        reference_position=(0.0, 0.0),
        reference_velocity=(0.15, 0.0),
        measured_position=(0.0, 0.0),
        measured_velocity=(0.15, 0.0),
    )
    controller.xy_feedforward_gain = 1.1
    controller.xy_feedforward_max_extra_m_s = 0.03

    _, first_command = run_pid_step(controller)
    gains, second_command = run_pid_step(controller)

    assert first_command[:2] == pytest.approx((0.165, 0.0), abs=1e-15)
    assert second_command[:2] == pytest.approx((0.165, 0.0), abs=1e-15)
    assert gains[1] == pytest.approx(0.0, abs=1e-15)
    assert gains[4] == pytest.approx(0.0, abs=1e-15)
    assert controller.last_xy_feedforward_extra == pytest.approx(
        (0.015, 0.0), abs=1e-15
    )


def test_pid_feedforward_extra_and_final_xy_command_remain_bounded():
    controller = make_pid_controller(
        reference_position=(0.0, 0.0),
        reference_velocity=(0.8, 0.0),
        measured_position=(0.0, 0.0),
        measured_velocity=(0.8, 0.0),
    )
    controller.xy_feedforward_gain = 1.5
    controller.xy_feedforward_max_extra_m_s = 0.03

    _, command = run_pid_step(controller)

    assert controller.last_xy_feedforward_extra == pytest.approx(
        (0.03, 0.0), abs=1e-15
    )
    assert math.hypot(command[0], command[1]) == pytest.approx(
        controller.xy_dot_limit, abs=1e-15
    )


def test_xy_pid_is_continuous_across_two_centimeter_integral_threshold():
    controller = make_pid_controller(
        reference_position=(0.0199, 0.0),
        reference_velocity=(0.15, 0.0),
        measured_velocity=(0.15, 0.0),
    )

    first_gains, first_command = run_pid_step(controller)
    second_gains, second_command = run_pid_step(
        controller, reference_position=(0.0201, 0.0)
    )
    third_gains, third_command = run_pid_step(
        controller, reference_position=(0.0199, 0.0)
    )

    assert [first_gains[1], second_gains[1], third_gains[1]] == pytest.approx(
        [0.0, 0.0, 0.0], abs=1e-15
    )
    expected_step = 0.7 * 0.0002
    assert second_command[0] - first_command[0] == pytest.approx(
        expected_step, abs=1e-15
    )
    assert third_command[0] - second_command[0] == pytest.approx(
        -expected_step, abs=1e-15
    )


def test_reset_reinitializes_velocity_and_primes_first_derivative_step():
    controller = make_pid_controller(
        reference_velocity=(0.15, 0.0), measured_velocity=(0.0, 0.0)
    )
    controller.world_velocity_estimator.observe(1_000_000_000, (0.0, 0.0))
    controller.world_velocity_estimator.observe(1_010_000_000, (0.001, 0.0))
    run_pid_step(controller)
    for _ in range(5):
        gains, _ = run_pid_step(controller)
    assert gains[1] > 0.0

    controller.reset_pid_state()

    assert controller.pid_needs_prime
    assert controller.measured_world_velocity is None
    assert controller.world_velocity_estimator.last_stamp_ns is None
    assert controller.d_err_x_filt == 0.0
    assert controller.last_xy_feedforward_extra == (0.0, 0.0)

    first_gains, _ = run_pid_step(
        controller, measured_velocity=(0.0, 0.0)
    )
    second_gains, _ = run_pid_step(controller)
    assert first_gains[1] == 0.0
    assert second_gains[1] > 0.0
