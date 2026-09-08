import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from hamr_control.hamr_controller import (
    HamrControlNode,
    VICON_SOURCE_STAMP_POLICY,
    ViconPosePlausibilityGuard,
    absolute_vicon_pose_is_plausible,
    quaternion_angular_distance,
    quaternion_tilt_rad,
    source_timestamp_is_valid_and_monotonic,
    validated_vicon_source_stamp_policy,
)


MAX_POSITION_JUMP_M = 0.08
MAX_ORIENTATION_JUMP_RAD = 0.35
RECOVERY_SAMPLES = 10
MIN_Z_M = 0.25
MAX_Z_M = 0.40
MAX_TILT_RAD = 0.35


def make_guard(recovery_samples=RECOVERY_SAMPLES):
    """Relative-only guard used by the pre-existing step-guard tests."""
    return ViconPosePlausibilityGuard(
        MAX_POSITION_JUMP_M,
        MAX_ORIENTATION_JUMP_RAD,
        recovery_samples,
        min_z_m=-10.0,
        max_z_m=10.0,
        max_tilt_rad=math.pi,
    )


def make_ground_guard(recovery_samples=RECOVERY_SAMPLES):
    return ViconPosePlausibilityGuard(
        MAX_POSITION_JUMP_M,
        MAX_ORIENTATION_JUMP_RAD,
        recovery_samples,
        min_z_m=MIN_Z_M,
        max_z_m=MAX_Z_M,
        max_tilt_rad=MAX_TILT_RAD,
    )


def yaw_quaternion(yaw_rad):
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def yaw_then_tilt_quaternion(yaw_rad, tilt_rad):
    """Quaternion for world-Z yaw followed by body-X gravity tilt."""
    yaw_sine = math.sin(yaw_rad / 2.0)
    yaw_cosine = math.cos(yaw_rad / 2.0)
    tilt_sine = math.sin(tilt_rad / 2.0)
    tilt_cosine = math.cos(tilt_rad / 2.0)
    return (
        yaw_cosine * tilt_sine,
        yaw_sine * tilt_sine,
        yaw_sine * tilt_cosine,
        yaw_cosine * tilt_cosine,
    )


def make_odom(position, quaternion, stamp_ns=1_000_000_000):
    msg = Odometry()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
        stamp_ns, 1_000_000_000
    )
    pose = msg.pose.pose
    pose.position.x, pose.position.y, pose.position.z = position
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = quaternion
    return msg


class FakeDuration:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class FakeTime:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        return FakeDuration(self.nanoseconds - other.nanoseconds)


class FakeClock:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def now(self):
        return FakeTime(self.nanoseconds)


class FakeLogger:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.infos = []

    def error(self, message):
        self.errors.append(message)

    def warn(self, message):
        self.warnings.append(message)

    def info(self, message):
        self.infos.append(message)


def configure_source_time_guard(controller):
    controller.vicon_source_stamp_policy = VICON_SOURCE_STAMP_POLICY
    controller.last_odom_source_stamp_ns = None


def test_source_timestamp_accepts_any_absolute_offset_when_monotonic():
    old_clock = source_timestamp_is_valid_and_monotonic(1, 0)
    far_future_clock = source_timestamp_is_valid_and_monotonic(
        5000, 0, old_clock.stamp_ns
    )

    assert old_clock.accepted and old_clock.stamp_ns == 1_000_000_000
    assert far_future_clock.accepted
    assert far_future_clock.stamp_ns == 5_000_000_000_000


def test_source_timestamp_rejects_invalid_zero_and_nonmonotonic_stamps():
    zero = source_timestamp_is_valid_and_monotonic(0, 0)
    duplicate = source_timestamp_is_valid_and_monotonic(
        2, 0, 2_000_000_000
    )
    reversed_stamp = source_timestamp_is_valid_and_monotonic(
        1, 999_999_999, 2_000_000_000
    )
    malformed = source_timestamp_is_valid_and_monotonic(1, 1_000_000_000)

    assert not zero.accepted and zero.reason == "zero header timestamp"
    assert not duplicate.accepted
    assert duplicate.reason == "non-monotonic source timestamp"
    assert not reversed_stamp.accepted
    assert reversed_stamp.reason == "non-monotonic source timestamp"
    assert not malformed.accepted
    assert malformed.reason == "invalid header timestamp"


def test_hardware_source_stamp_policy_is_explicit_and_single_mode():
    assert validated_vicon_source_stamp_policy(
        " RECEIPT_MONOTONIC "
    ) == VICON_SOURCE_STAMP_POLICY
    for value in ("absolute_age", "receipt", "", None):
        with pytest.raises(ValueError, match="vicon_source_stamp_policy"):
            validated_vicon_source_stamp_policy(value)


def test_absolute_ground_pose_helper_accepts_inclusive_height_and_tilt_bounds():
    for z_m in (MIN_Z_M, 0.323, MAX_Z_M):
        result = absolute_vicon_pose_is_plausible(
            (4.0, -3.0, z_m),
            yaw_then_tilt_quaternion(2.4, MAX_TILT_RAD),
            MIN_Z_M,
            MAX_Z_M,
            MAX_TILT_RAD,
        )

        assert result.accepted
        assert math.isclose(result.z_m, z_m)
        assert math.isclose(result.tilt_rad, MAX_TILT_RAD, abs_tol=1e-12)


def test_gravity_tilt_is_yaw_invariant():
    expected_tilt = 0.12

    tilts = [
        quaternion_tilt_rad(yaw_then_tilt_quaternion(yaw, expected_tilt))
        for yaw in (-math.pi, -1.0, 0.0, 2.0, math.pi)
    ]

    assert all(
        math.isclose(tilt, expected_tilt, abs_tol=1e-12)
        for tilt in tilts
    )


def test_absolute_ground_pose_helper_rejects_height_tilt_and_invalid_inputs():
    high = absolute_vicon_pose_is_plausible(
        (0.0, 0.0, MAX_Z_M + 1e-6),
        yaw_quaternion(0.0),
        MIN_Z_M,
        MAX_Z_M,
        MAX_TILT_RAD,
    )
    tilted = absolute_vicon_pose_is_plausible(
        (0.0, 0.0, 0.323),
        yaw_then_tilt_quaternion(1.3, MAX_TILT_RAD + 1e-6),
        MIN_Z_M,
        MAX_Z_M,
        MAX_TILT_RAD,
    )
    invalid = absolute_vicon_pose_is_plausible(
        (0.0, math.nan, 0.323),
        (0.0, 0.0, 0.0, 0.0),
        MIN_Z_M,
        MAX_Z_M,
        MAX_TILT_RAD,
    )

    assert not high.accepted and "absolute z" in high.reason
    assert not tilted.accepted and "absolute tilt" in tilted.reason
    assert not invalid.accepted
    assert invalid.reason == "non-finite position or invalid quaternion"


def test_recorded_stable_wrong_startup_pose_fails_absolute_envelope():
    result = absolute_vicon_pose_is_plausible(
        (0.1308571146373248, 4.006006442983119, 0.44293490791279283),
        (
            0.6565229817923777,
            -0.5220124760105851,
            -0.4994343885938946,
            -0.2169005319444198,
        ),
        MIN_Z_M,
        MAX_Z_M,
        MAX_TILT_RAD,
    )

    assert not result.accepted
    assert "absolute z" in result.reason
    assert "absolute tilt" in result.reason
    assert result.z_m > 0.442
    assert result.tilt_rad > 1.9


def test_stable_wrong_startup_never_becomes_baseline_or_recovery_candidate():
    guard = make_ground_guard()
    wrong_position = (0.13, 4.01, 0.443)
    wrong_quaternion = (
        0.6565229817923777,
        -0.5220124760105851,
        -0.4994343885938946,
        -0.2169005319444198,
    )

    for index in range(2 * RECOVERY_SAMPLES):
        result = guard.observe(wrong_position, wrong_quaternion)
        assert not result.accepted
        assert result.latched
        assert result.recovery_count == 0
        assert result.newly_latched == (index == 0)

    assert guard.last_position is None


def test_absolute_startup_failure_requires_ten_valid_ground_samples_to_recover():
    guard = make_ground_guard()
    guard.observe((0.0, 0.0, 0.443), yaw_quaternion(0.0))

    for index in range(RECOVERY_SAMPLES - 1):
        result = guard.observe(
            (0.001 * index, 0.0, 0.323),
            yaw_then_tilt_quaternion(0.1, 0.02),
        )
        assert not result.accepted
        assert result.recovery_count == index + 1

    recovered = guard.observe(
        (0.001 * RECOVERY_SAMPLES, 0.0, 0.323),
        yaw_then_tilt_quaternion(0.1, 0.02),
    )

    assert recovered.accepted and recovered.recovered
    assert not guard.latched


def test_normal_physical_translation_and_turning_are_accepted():
    guard = make_guard()

    assert guard.observe((0.0, 0.0, 0.33), yaw_quaternion(0.0)).accepted
    # These are deliberately larger than ordinary 60--90 Hz sharp-turn steps
    # in the two bags, while remaining inside the configured safety margins.
    result = guard.observe((0.04, 0.03, 0.335), yaw_quaternion(0.15))

    assert result.accepted
    assert not result.latched
    assert math.isclose(result.position_jump_m, 0.0502493781, rel_tol=1e-6)
    assert math.isclose(result.orientation_jump_rad, 0.15, abs_tol=1e-12)


def test_quaternion_sign_flip_is_the_same_orientation():
    guard = make_guard()
    quaternion = yaw_quaternion(1.2)

    assert guard.observe((0.0, 0.0, 0.33), quaternion).accepted
    result = guard.observe(
        (0.001, -0.001, 0.33), tuple(-value for value in quaternion)
    )

    assert result.accepted
    assert result.orientation_jump_rad == 0.0
    assert quaternion_angular_distance(quaternion, tuple(-v for v in quaternion)) == 0.0


def test_three_dimensional_position_jump_latches_including_vertical_motion():
    guard = make_guard()
    guard.observe((0.0, 0.0, 0.33), yaw_quaternion(0.0))

    result = guard.observe((0.0, 0.0, 0.411), yaw_quaternion(0.0))

    assert not result.accepted
    assert result.latched
    assert result.newly_latched
    assert "3-D position jump" in result.reason
    assert math.isclose(result.position_jump_m, 0.081, abs_tol=1e-12)


def test_first_bag_marker_flip_sequence_is_rejected_against_clean_baseline():
    guard = make_guard()
    # /HAMR_base/odom at 17:24:48.627, immediately before the first bad solve.
    clean_position = (-0.017068917091282214, 1.2988566176888898, 0.3197499296648777)
    clean_quaternion = (
        -0.008098173098388728,
        -0.0044110598656034875,
        0.010624415600344076,
        0.9999010370729714,
    )
    guard.observe(clean_position, clean_quaternion)

    # The next two recorded samples contained the 10 cm/83 deg onset followed
    # by a roughly 93 deg attitude step toward the upside-down marker solution.
    outliers = [
        (
            (-0.03479222932551683, 1.3319349767133621, 0.41344994783535244),
            (-0.0019239162105166479, 0.6378520973938475,
             -0.15557208994550797, 0.7542800045273595),
        ),
        (
            (-0.037972880379560446, 1.34179978939433, 0.4328655130516488),
            (0.004633908412740542, 0.9685624744463935,
             -0.24485810308763908, 0.043700907800029616),
        ),
    ]

    first = guard.observe(*outliers[0])
    second = guard.observe(*outliers[1])

    assert not first.accepted and first.newly_latched
    assert first.position_jump_m > 0.10
    assert first.orientation_jump_rad > 1.4
    assert not second.accepted and second.latched
    assert guard.last_position == clean_position
    assert quaternion_angular_distance(
        guard.last_quaternion, clean_quaternion
    ) == 0.0


def test_second_bag_roll_flip_sequence_is_rejected_despite_sane_position():
    guard = make_guard()
    # /HAMR_base/odom at 18:38:53.706, immediately before the dog-free failure.
    clean_position = (-0.2006326708637598, 3.672915479910363, 0.3333407026174254)
    clean_quaternion = (
        -0.02563182335809201,
        0.0035198686849512223,
        -0.9040277504075153,
        0.4266901061063344,
    )
    guard.observe(clean_position, clean_quaternion)

    # Position moved only 1.9 cm, but the full quaternion rotated about 80 deg;
    # a yaw-only plausibility check would miss part of this rigid-body failure.
    result = guard.observe(
        (-0.19558670943797532, 3.654906405111442, 0.33390372025628956),
        (0.16432158701982394, 0.6195664716904415,
         -0.6870942687112012, 0.34210710179809284),
    )

    assert not result.accepted
    assert result.position_jump_m < 0.02
    assert result.orientation_jump_rad > 1.39
    assert "quaternion rotation jump" in result.reason
    assert guard.last_position == clean_position


def test_recovery_requires_consecutive_samples_close_to_last_accepted_pose():
    guard = make_guard(recovery_samples=3)
    baseline_position = (0.0, 1.0, 0.33)
    baseline_quaternion = yaw_quaternion(0.1)
    guard.observe(baseline_position, baseline_quaternion)
    guard.observe((0.0, 1.0, 0.45), yaw_quaternion(2.0))

    first = guard.observe((0.01, 1.0, 0.33), yaw_quaternion(0.11))
    second = guard.observe((0.02, 1.0, 0.33), yaw_quaternion(0.12))

    assert not first.accepted and first.recovery_count == 1
    assert not second.accepted and second.recovery_count == 2
    assert guard.last_position == baseline_position

    # A new bad sample breaks the consecutive run.
    reset = guard.observe((0.20, 1.0, 0.33), yaw_quaternion(0.13))
    assert not reset.accepted and reset.recovery_count == 0

    guard.observe((0.01, 1.0, 0.33), yaw_quaternion(0.11))
    guard.observe((0.02, 1.0, 0.33), yaw_quaternion(0.12))
    recovered = guard.observe((0.03, 1.0, 0.33), yaw_quaternion(0.13))

    assert recovered.accepted and recovered.recovered
    assert not guard.latched
    assert guard.last_position == (0.03, 1.0, 0.33)


def test_invalid_first_sample_requires_full_stable_validation_run():
    guard = make_guard()

    invalid = guard.observe((math.nan, 0.0, 0.33), (0.0, 0.0, 0.0, 0.0))
    assert not invalid.accepted and invalid.latched

    for index in range(RECOVERY_SAMPLES - 1):
        result = guard.observe(
            (0.001 * index, 0.0, 0.33), yaw_quaternion(0.001 * index)
        )
        assert not result.accepted
        assert result.recovery_count == index + 1

    recovered = guard.observe(
        (0.001 * RECOVERY_SAMPLES, 0.0, 0.33),
        yaw_quaternion(0.001 * RECOVERY_SAMPLES),
    )
    assert recovered.accepted and recovered.recovered


def test_recovery_samples_must_also_be_stable_relative_to_each_other():
    guard = make_guard(recovery_samples=3)
    guard.observe((0.0, 0.0, 0.33), yaw_quaternion(0.0))
    guard.observe((0.0, 0.0, 0.50), yaw_quaternion(math.pi))

    first = guard.observe((0.079, 0.0, 0.33), yaw_quaternion(0.0))
    second = guard.observe((-0.079, 0.0, 0.33), yaw_quaternion(0.0))
    third = guard.observe((0.079, 0.0, 0.33), yaw_quaternion(0.0))

    assert first.recovery_count == 1
    assert not second.accepted and second.recovery_count == 1
    assert second.reason == "unstable recovery step"
    assert not third.accepted and third.recovery_count == 1
    assert guard.latched


def test_odom_callback_stops_immediately_without_accepting_or_refreshing_outlier():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard_enabled = True
    controller.vicon_pose_guard = make_guard(recovery_samples=2)
    configure_source_time_guard(controller)
    controller.pose_base_ = None
    controller.last_odom_time_ns = None
    cached_reference = object()
    controller.reference_ = cached_reference
    controller.last_reference_time_ns = 1_000_000_000
    clock = FakeClock(1_000_000_000)
    logger = FakeLogger()
    events = []
    controller.get_clock = lambda: clock
    controller.get_logger = lambda: logger
    controller.publish_zero_cmd = lambda: events.append("zero")
    controller.reset_pid_state = lambda: events.append("reset")

    clean = make_odom(
        (0.0, 0.0, 0.33),
        yaw_quaternion(0.0),
        stamp_ns=1_000_000_000,
    )
    controller.callback_odom(clean)
    accepted_pose = controller.pose_base_
    accepted_time = controller.last_odom_time_ns

    clock.nanoseconds += 10_000_000
    bad = make_odom(
        (0.0, 0.0, 0.33),
        yaw_quaternion(math.pi),
        stamp_ns=1_010_000_000,
    )
    controller.callback_odom(bad)

    assert events == ["zero", "reset"]
    assert controller.pose_base_ is accepted_pose
    assert controller.last_odom_time_ns == accepted_time
    assert controller.vicon_pose_guard.latched
    assert controller.vicon_motion_inhibited
    assert controller.reference_ is None
    assert controller.last_reference_time_ns is None
    assert len(logger.errors) == 1

    # The first sane recovery observation still cannot refresh odom freshness.
    clock.nanoseconds += 10_000_000
    recovering = make_odom(
        (0.0, 0.0, 0.33),
        yaw_quaternion(0.0),
        stamp_ns=1_020_000_000,
    )
    controller.callback_odom(recovering)
    assert controller.last_odom_time_ns == accepted_time
    assert events[-2:] == ["zero", "reset"]

    clock.nanoseconds += 10_000_000
    recovered = make_odom(
        (0.01, 0.0, 0.33),
        yaw_quaternion(0.01),
        stamp_ns=1_030_000_000,
    )
    controller.callback_odom(recovered)
    assert controller.pose_base_ is recovered.pose
    assert controller.last_odom_time_ns == clock.nanoseconds
    assert not controller.vicon_pose_guard.latched
    assert logger.warnings


def test_odom_callback_immediately_stops_on_stable_wrong_absolute_pose():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard_enabled = True
    controller.vicon_pose_guard = make_ground_guard()
    configure_source_time_guard(controller)
    controller.pose_base_ = None
    controller.last_odom_time_ns = None
    cached_reference = object()
    controller.reference_ = cached_reference
    controller.last_reference_time_ns = 2_000_000_000
    controller.get_clock = lambda: FakeClock(1_000_000_000)
    logger = FakeLogger()
    events = []
    controller.get_logger = lambda: logger
    controller.publish_zero_cmd = lambda: events.append("zero")
    controller.reset_pid_state = lambda: events.append("reset")

    wrong = make_odom(
        (0.1308571146373248, 4.006006442983119, 0.44293490791279283),
        (
            0.6565229817923777,
            -0.5220124760105851,
            -0.4994343885938946,
            -0.2169005319444198,
        ),
    )
    controller.callback_odom(wrong)

    assert events == ["zero", "reset"]
    assert controller.pose_base_ is None
    assert controller.last_odom_time_ns is None
    assert controller.vicon_pose_guard.latched
    assert "absolute z" in logger.errors[0]
    assert "absolute tilt" in logger.errors[0]


def test_absolute_clock_offset_does_not_block_monotonic_vicon_receipts():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard_enabled = True
    controller.vicon_pose_guard = make_guard()
    configure_source_time_guard(controller)
    controller.pose_base_ = None
    controller.last_odom_time_ns = None
    controller.odom_timeout_s = 0.12
    controller.odom_timed_out = False
    controller.hamr_config = {"mode": "auto"}
    controller.reference_timeout_s = 0.5
    controller.reference_timed_out = False
    controller.vicon_motion_inhibited = False
    controller.vicon_reference_rearm_ready = False
    controller.vicon_inhibit_last_reference_time_ns = None
    clock = FakeClock(2_000_000_000)
    logger = FakeLogger()
    events = []
    controller.get_clock = lambda: clock
    controller.get_logger = lambda: logger
    controller.publish_zero_cmd = lambda: events.append("zero")
    controller.reset_pid_state = lambda: events.append("reset")

    first = make_odom(
        (0.0, 0.0, 0.33),
        yaw_quaternion(0.0),
        stamp_ns=1_000_000_000,
    )
    controller.callback_odom(first)
    cached_reference = object()
    controller.reference_ = cached_reference
    controller.last_reference_time_ns = clock.nanoseconds

    clock.nanoseconds += 10_000_000
    second = make_odom(
        (0.01, 0.0, 0.33),
        yaw_quaternion(0.01),
        stamp_ns=1_010_000_000,
    )
    controller.callback_odom(second)

    assert events == []
    assert controller.pose_base_ is second.pose
    assert controller.last_odom_time_ns == clock.nanoseconds
    assert controller.last_odom_source_stamp_ns == 1_010_000_000
    assert not controller.vicon_pose_guard.latched
    assert not controller.vicon_motion_inhibited
    assert controller.reference_ is cached_reference
    assert controller.last_reference_time_ns == 2_000_000_000
    assert not logger.errors
    assert controller.measured_world_velocity == pytest.approx((1.0, 0.0))


def test_stale_odom_uses_exact_450ms_accepted_receipt_watchdog_boundary():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard_enabled = True
    controller.vicon_pose_guard = make_guard()
    configure_source_time_guard(controller)
    baseline_ns = 2_000_000_000
    clock = FakeClock(baseline_ns)
    logger = FakeLogger()
    events = []
    controller.pose_base_ = None
    controller.last_odom_time_ns = None
    controller.odom_timeout_s = 0.45
    controller.odom_timed_out = False
    controller.hamr_config = {"mode": "auto"}
    controller.reference_timeout_s = 0.5
    controller.reference_timed_out = False
    controller.vicon_motion_inhibited = False
    controller.vicon_reference_rearm_ready = False
    controller.vicon_inhibit_last_reference_time_ns = None
    controller.get_clock = lambda: clock
    controller.get_logger = lambda: logger
    controller.publish_zero_cmd = lambda: events.append("zero")
    controller.reset_pid_state = lambda: events.append("reset")

    controller.callback_odom(
        make_odom(
            (0.0, 0.0, 0.33),
            yaw_quaternion(0.0),
            stamp_ns=baseline_ns,
        )
    )
    cached_reference = object()
    controller.reference_ = cached_reference
    controller.last_reference_time_ns = baseline_ns

    for receipt_age_ns in (449_999_999, 450_000_000):
        clock.nanoseconds = baseline_ns + receipt_age_ns
        assert controller.ensure_base_odom_ready(clock.nanoseconds)
        assert events == []
        assert controller.reference_ is cached_reference
        assert not controller.vicon_motion_inhibited

    clock.nanoseconds = baseline_ns + 450_000_001
    assert not controller.ensure_base_odom_ready(clock.nanoseconds)

    assert events == ["zero", "reset"]
    assert controller.vicon_pose_guard.latched
    assert controller.vicon_motion_inhibited
    assert controller.reference_ is None
    assert controller.last_reference_time_ns is None
    assert controller.odom_timed_out


def test_zero_header_callback_latches_before_pose_validation():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard_enabled = True
    controller.vicon_pose_guard = make_guard()
    configure_source_time_guard(controller)
    controller.pose_base_ = None
    controller.last_odom_time_ns = None
    controller.get_clock = lambda: FakeClock(2_000_000_000)
    logger = FakeLogger()
    events = []
    controller.get_logger = lambda: logger
    controller.publish_zero_cmd = lambda: events.append("zero")
    controller.reset_pid_state = lambda: events.append("reset")

    # The invalid quaternion would also fail the pose guard, so the logged
    # timestamp reason proves source fields are checked first.
    message = make_odom(
        (0.0, 0.0, 0.33),
        (0.0, 0.0, 0.0, 0.0),
        stamp_ns=0,
    )
    controller.callback_odom(message)

    assert events == ["zero", "reset"]
    assert controller.pose_base_ is None
    assert controller.last_odom_time_ns is None
    assert controller.vicon_pose_guard.latched
    assert "zero header timestamp" in logger.errors[0]


def test_timestamp_fault_recovery_requires_ten_consecutive_monotonic_samples():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard_enabled = True
    controller.vicon_pose_guard = make_guard(recovery_samples=RECOVERY_SAMPLES)
    configure_source_time_guard(controller)
    controller.pose_base_ = None
    controller.last_odom_time_ns = None
    clock = FakeClock(2_000_000_000)
    logger = FakeLogger()
    events = []
    controller.get_clock = lambda: clock
    controller.get_logger = lambda: logger
    controller.publish_zero_cmd = lambda: events.append("zero")
    controller.reset_pid_state = lambda: events.append("reset")

    baseline = make_odom(
        (0.0, 0.0, 0.33), yaw_quaternion(0.0), stamp_ns=clock.nanoseconds
    )
    controller.callback_odom(baseline)
    accepted_receipt_ns = controller.last_odom_time_ns

    clock.nanoseconds += 10_000_000
    duplicate = make_odom(
        (0.001, 0.0, 0.33),
        yaw_quaternion(0.001),
        stamp_ns=baseline.header.stamp.sec * 1_000_000_000
        + baseline.header.stamp.nanosec,
    )
    controller.callback_odom(duplicate)

    for index in range(4):
        clock.nanoseconds += 10_000_000
        fresh = make_odom(
            (0.001 * (index + 1), 0.0, 0.33),
            yaw_quaternion(0.001 * (index + 1)),
            stamp_ns=2_010_000_000 + index * 10_000_000,
        )
        controller.callback_odom(fresh)
        assert controller.vicon_pose_guard.latched
        assert controller.last_odom_time_ns == accepted_receipt_ns

    # A duplicate cannot count toward, or bridge, the required run of
    # consecutive monotonic recovery observations.
    clock.nanoseconds += 10_000_000
    duplicate = make_odom(
        (0.005, 0.0, 0.33),
        yaw_quaternion(0.005),
        stamp_ns=2_040_000_000,
    )
    controller.callback_odom(duplicate)
    assert controller.vicon_pose_guard.recovery_count == 0

    for index in range(RECOVERY_SAMPLES - 1):
        clock.nanoseconds += 10_000_000
        fresh = make_odom(
            (0.001 * (index + 1), 0.0, 0.33),
            yaw_quaternion(0.001 * (index + 1)),
            stamp_ns=2_050_000_000 + index * 10_000_000,
        )
        controller.callback_odom(fresh)
        assert controller.vicon_pose_guard.latched
        assert controller.last_odom_time_ns == accepted_receipt_ns

    clock.nanoseconds += 10_000_000
    recovered = make_odom(
        (0.01, 0.0, 0.33),
        yaw_quaternion(0.01),
        stamp_ns=2_140_000_000,
    )
    controller.callback_odom(recovered)

    assert not controller.vicon_pose_guard.latched
    assert controller.pose_base_ is recovered.pose
    assert controller.last_odom_time_ns == clock.nanoseconds
    assert logger.warnings


def test_simulation_compatibility_when_vicon_guard_is_disabled():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard_enabled = False
    controller.pose_base_ = None
    controller.last_odom_time_ns = None
    controller.get_clock = lambda: FakeClock(2_000_000_000)

    zero_stamped_sim_odom = make_odom(
        (0.0, 0.0, 0.0), yaw_quaternion(0.0), stamp_ns=0
    )
    controller.callback_odom(zero_stamped_sim_odom)

    assert controller.pose_base_ is zero_stamped_sim_odom.pose
    assert controller.last_odom_time_ns == 2_000_000_000
    assert not getattr(controller, "vicon_motion_inhibited", False)


def test_waypoint_unprotected_mode_accepts_vicon_outlier_and_reversed_stamp():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard_enabled = False
    controller.pose_base_ = None
    controller.last_odom_time_ns = None
    controller.xy_velocity_source = "odom_twist_world"
    clock = FakeClock(2_000_000_000)
    controller.get_clock = lambda: clock

    clean = make_odom(
        (0.0, 0.0, 0.33), yaw_quaternion(0.0), stamp_ns=2_000_000_000
    )
    controller.callback_odom(clean)

    clock.nanoseconds += 10_000_000
    outlier = make_odom(
        (0.20, 0.0, 0.50),
        yaw_quaternion(math.pi),
        stamp_ns=1_000_000_000,
    )
    controller.callback_odom(outlier)

    # With the explicit waypoint bypass, neither reversed source time nor a
    # pose outside every normal Vicon envelope enters the inhibit path.
    assert controller.pose_base_ is outlier.pose
    assert controller.last_odom_time_ns == clock.nanoseconds
    assert not getattr(controller, "vicon_motion_inhibited", False)


def test_manual_mode_cannot_command_motion_while_vicon_guard_is_latched():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard_enabled = True
    controller.vicon_pose_guard = make_guard()
    controller.vicon_pose_guard.latched = True
    controller.get_clock = lambda: FakeClock(2_000_000_000)
    events = []
    controller.publish_zero_cmd = lambda: events.append("zero")
    controller.publish_joint_cmd = lambda *_: events.append("motion")

    controller.manual_mode_callback(Twist())

    assert events == ["zero"]


def test_manual_watchdog_zeros_a_held_command_when_vicon_goes_silent():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard_enabled = True
    controller.vicon_pose_guard = make_guard()
    controller.last_odom_time_ns = 1_000_000_000
    controller.odom_timeout_s = 0.25
    controller.odom_timed_out = False
    controller.get_clock = lambda: FakeClock(2_000_000_000)
    controller.get_logger = lambda: FakeLogger()
    events = []
    controller.publish_zero_cmd = lambda: events.append("zero")
    controller.reset_pid_state = lambda: events.append("reset")

    controller.manual_safety_tick()

    assert events == ["reset", "zero"]
    assert controller.odom_timed_out


def test_auto_mode_cannot_run_pid_while_vicon_guard_is_latched():
    controller = object.__new__(HamrControlNode)
    now_ns = 2_000_000_000
    controller.get_clock = lambda: FakeClock(now_ns)
    controller.last_control_time = FakeTime(now_ns - 10_000_000)
    controller.last_reference_time_ns = now_ns
    controller.reference_timeout_s = 0.5
    controller.reference_ = object()
    controller.reference_timed_out = False
    controller.vicon_pose_guard_enabled = True
    controller.vicon_pose_guard = make_guard()
    controller.vicon_pose_guard.latched = True
    events = []
    controller.publish_zero_cmd = lambda: events.append("zero")
    controller.pid_step = lambda: events.append("pid")

    controller.control_tick()

    assert events == ["zero"]


def test_enabling_guard_invalidates_unchecked_odom_and_starts_validation():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard_enabled = False
    controller.vicon_pose_guard = make_guard()
    controller.pose_base_ = object()
    controller.last_odom_time_ns = 1_000_000_000
    controller.reference_ = object()
    controller.last_reference_time_ns = 1_000_000_000
    controller.get_clock = lambda: FakeClock(1_000_000_000)
    controller.get_logger = lambda: FakeLogger()
    events = []
    controller.reset_pid_state = lambda: events.append("reset")
    controller.publish_zero_cmd = lambda: events.append("zero")

    controller.parameters_callback(
        [SimpleNamespace(name="vicon_pose_guard_enabled", value=True)]
    )

    assert controller.vicon_pose_guard_enabled
    assert controller.vicon_pose_guard.latched
    assert controller.pose_base_ is None
    assert controller.last_odom_time_ns is None
    assert controller.reference_ is None
    assert controller.vicon_motion_inhibited
    assert events == ["zero", "reset"]


def test_absolute_limit_validation_rejects_nonfinite_inverted_and_invalid_tilt():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard = make_ground_guard()

    cases = (
        [SimpleNamespace(name="vicon_min_z_m", value=math.nan)],
        [
            SimpleNamespace(name="vicon_min_z_m", value=0.41),
            SimpleNamespace(name="vicon_max_z_m", value=0.40),
        ],
        [SimpleNamespace(name="vicon_max_tilt_rad", value=math.pi + 1e-6)],
    )

    for parameters in cases:
        result = controller.validate_safety_parameters_callback(parameters)
        assert not result.successful
        assert result.reason


def test_runtime_absolute_limit_change_stops_and_requires_fresh_validation():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard_enabled = True
    controller.vicon_pose_guard = make_ground_guard()
    controller.vicon_pose_guard.observe(
        (0.0, 0.0, 0.323), yaw_quaternion(0.0)
    )
    controller.pose_base_ = object()
    controller.last_odom_time_ns = 1_000_000_000
    controller.reference_ = object()
    controller.last_reference_time_ns = 1_000_000_000
    controller.get_clock = lambda: FakeClock(1_000_000_000)
    logger = FakeLogger()
    controller.get_logger = lambda: logger
    events = []
    controller.reset_pid_state = lambda: events.append("reset")
    controller.publish_zero_cmd = lambda: events.append("zero")

    controller.parameters_callback(
        [SimpleNamespace(name="vicon_max_z_m", value=0.39)]
    )

    assert controller.vicon_pose_guard.max_z_m == 0.39
    assert controller.vicon_pose_guard.latched
    assert controller.vicon_pose_guard.last_position is None
    assert controller.pose_base_ is None
    assert controller.last_odom_time_ns is None
    assert controller.reference_ is None
    assert controller.vicon_motion_inhibited
    assert events == ["zero", "reset"]

    for index in range(RECOVERY_SAMPLES - 1):
        result = controller.vicon_pose_guard.observe(
            (0.001 * index, 0.0, 0.323), yaw_quaternion(0.0)
        )
        assert not result.accepted
    recovered = controller.vicon_pose_guard.observe(
        (0.01, 0.0, 0.323), yaw_quaternion(0.0)
    )
    assert recovered.accepted and recovered.recovered


def test_direct_invalid_runtime_absolute_update_fails_closed():
    controller = object.__new__(HamrControlNode)
    controller.vicon_pose_guard_enabled = True
    controller.vicon_pose_guard = make_ground_guard()
    controller.pose_base_ = object()
    controller.last_odom_time_ns = 1_000_000_000
    controller.reference_ = object()
    controller.last_reference_time_ns = 1_000_000_000
    controller.get_clock = lambda: FakeClock(1_000_000_000)
    logger = FakeLogger()
    controller.get_logger = lambda: logger
    events = []
    controller.reset_pid_state = lambda: events.append("reset")
    controller.publish_zero_cmd = lambda: events.append("zero")

    controller.parameters_callback(
        [SimpleNamespace(name="vicon_max_z_m", value=0.20)]
    )

    assert controller.vicon_pose_guard.latched
    assert controller.pose_base_ is None
    assert controller.last_odom_time_ns is None
    assert controller.reference_ is None
    assert controller.vicon_motion_inhibited
    assert events == ["zero", "reset"]
    assert "latched at zero" in logger.errors[0]
