import math
import random

from hamr_control.hamr_controller import reference_is_fresh, scale_wheel_pair


HARDWARE_28_RPM_RAD_S = 2.93215314335


def test_commands_inside_limit_are_unchanged():
    right, left, scale = scale_wheel_pair(1.25, -2.5, 3.0)

    assert right == 1.25
    assert left == -2.5
    assert scale == 1.0


def test_overspeed_pair_uses_one_common_scale():
    right, left, scale = scale_wheel_pair(6.0, -3.0, 3.0)

    assert right == 3.0
    assert left == -1.5
    assert scale == 0.5


def test_28_rpm_operating_cap_preserves_pair_ratio():
    original_right = 3.5
    original_left = 1.75
    right, left, scale = scale_wheel_pair(
        original_right, original_left, HARDWARE_28_RPM_RAD_S
    )

    assert right == HARDWARE_28_RPM_RAD_S
    assert left == HARDWARE_28_RPM_RAD_S / 2.0
    assert scale == HARDWARE_28_RPM_RAD_S / original_right
    assert right / left == original_right / original_left


def test_mirrored_turns_are_limited_symmetrically():
    clockwise = scale_wheel_pair(7.0, 2.0, 3.0)
    counterclockwise = scale_wheel_pair(-7.0, -2.0, 3.0)

    assert counterclockwise[0] == -clockwise[0]
    assert counterclockwise[1] == -clockwise[1]
    assert counterclockwise[2] == clockwise[2]


def test_random_pairs_preserve_ratio_and_bound():
    generator = random.Random(20260818)

    for _ in range(1000):
        original_right = generator.uniform(-20.0, 20.0)
        original_left = generator.uniform(-20.0, 20.0)
        limit = generator.uniform(0.01, 5.0)
        right, left, scale = scale_wheel_pair(
            original_right, original_left, limit
        )

        assert max(abs(right), abs(left)) <= limit + 1e-12
        assert 0.0 < scale <= 1.0
        assert math.isclose(right, original_right * scale, abs_tol=1e-12)
        assert math.isclose(left, original_left * scale, abs_tol=1e-12)


def test_reference_timeout_requires_a_received_reference():
    assert not reference_is_fresh(1_000_000_000, None, 0.5)


def test_reference_timeout_boundary_and_expiry():
    last_reference_ns = 1_000_000_000

    assert reference_is_fresh(1_500_000_000, last_reference_ns, 0.5)
    assert not reference_is_fresh(1_500_000_001, last_reference_ns, 0.5)


def test_nonpositive_reference_timeout_disables_expiry():
    assert reference_is_fresh(10_000_000_000, 1, 0.0)
