import math
from types import SimpleNamespace

import pytest

from hamr_control.hamr_controller import HamrControlNode


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
        self.warnings = []
        self.errors = []

    def warn(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


class FakeViconGuard:
    def __init__(self):
        self.latched = False

    def latch(self):
        newly_latched = not self.latched
        self.latched = True
        return newly_latched


def make_controller(now_ns=1_000_000_000):
    controller = object.__new__(HamrControlNode)
    clock = FakeClock(now_ns)
    logger = FakeLogger()
    events = []

    controller.get_clock = lambda: clock
    controller.get_logger = lambda: logger
    controller.publish_zero_cmd = lambda: events.append("zero")
    controller.reset_pid_state = lambda: events.append("reset")
    controller.pid_step = lambda: events.append("pid")
    controller.vicon_pose_guard_enabled = True
    controller.vicon_pose_guard = FakeViconGuard()
    controller.hamr_config = {"mode": "auto"}
    controller.reference_timeout_s = 0.5
    controller.odom_timeout_s = 0.25
    controller.last_odom_time_ns = now_ns
    controller.last_reference_time_ns = None
    controller.reference_ = None
    controller.reference_timed_out = False
    controller.odom_timed_out = False
    controller.vicon_motion_inhibited = False
    controller.vicon_reference_rearm_ready = False
    controller.vicon_inhibit_last_reference_time_ns = None
    controller.last_control_time = FakeTime(now_ns - 10_000_000)

    return controller, clock, logger, events


def recover_tracking_at(controller, clock, now_ns):
    clock.nanoseconds = now_ns
    controller.vicon_pose_guard.latched = False
    controller.last_odom_time_ns = now_ns


def test_fault_invalidates_cached_reference_and_recovery_cannot_restart_it():
    controller, clock, _, events = make_controller()
    cached_reference = object()
    controller.reference_ = cached_reference
    controller.last_reference_time_ns = clock.nanoseconds

    controller._latch_vicon_motion_inhibit(clock.nanoseconds)

    assert events == ["zero", "reset"]
    assert controller.vicon_motion_inhibited
    assert controller.reference_ is None
    assert controller.last_reference_time_ns is None

    # Even after valid Vicon and the complete quiet interval, a timer tick may
    # only mark rearm readiness; it cannot reuse the pre-fault reference.
    recover_tracking_at(controller, clock, 1_500_000_000)
    controller.last_control_time = FakeTime(clock.nanoseconds - 10_000_000)
    controller.control_tick()

    assert events[-1] == "zero"
    assert "pid" not in events
    assert controller.vicon_motion_inhibited
    assert controller.vicon_reference_rearm_ready
    assert controller.reference_ is None


def test_inhibit_refreshes_zero_even_when_control_clock_does_not_advance():
    controller, clock, _, events = make_controller()
    controller._latch_vicon_motion_inhibit(clock.nanoseconds)
    controller.last_control_time = FakeTime(clock.nanoseconds)

    controller.control_tick()

    assert events == ["zero", "reset", "zero"]
    assert controller.vicon_motion_inhibited
    assert "pid" not in events


def test_continuously_publishing_references_never_rearm_after_fault():
    controller, clock, _, events = make_controller()
    controller._latch_vicon_motion_inhibit(clock.nanoseconds)

    # Run for several watchdog intervals. Each rejected callback moves the
    # quiet anchor forward, so elapsed wall time alone cannot re-enable motion.
    for index in range(1, 21):
        recover_tracking_at(
            controller,
            clock,
            1_000_000_000 + index * 100_000_000,
        )
        controller.callback_reference(SimpleNamespace(sequence=index))
        assert controller.reference_ is None
        assert controller.last_reference_time_ns is None
        assert controller.vicon_motion_inhibited
        assert not controller.vicon_reference_rearm_ready

    assert "pid" not in events


def test_quiet_interval_then_subsequent_new_reference_rearms():
    controller, clock, logger, events = make_controller()
    controller._latch_vicon_motion_inhibit(clock.nanoseconds)
    new_reference = SimpleNamespace(sequence=1)

    # Inclusive boundary: the callback arriving after exactly one configured
    # quiet interval is itself the required subsequent reference.
    recover_tracking_at(controller, clock, 1_500_000_000)
    controller.callback_reference(new_reference)

    assert not controller.vicon_motion_inhibited
    assert not controller.vicon_reference_rearm_ready
    assert controller.vicon_inhibit_last_reference_time_ns is None
    assert controller.reference_ is new_reference
    assert controller.last_reference_time_ns == clock.nanoseconds
    assert events == ["zero", "reset", "reset"]
    assert len(logger.warnings) == 2


def test_repeated_fault_restarts_the_entire_rearm_sequence():
    controller, clock, _, _ = make_controller()
    controller._latch_vicon_motion_inhibit(clock.nanoseconds)

    recover_tracking_at(controller, clock, 1_500_000_000)
    assert controller._update_vicon_reference_rearm(clock.nanoseconds)
    assert controller.vicon_reference_rearm_ready

    # A second fault cancels readiness and starts a new quiet interval.
    clock.nanoseconds = 1_510_000_000
    controller._latch_vicon_motion_inhibit(clock.nanoseconds)
    assert not controller.vicon_reference_rearm_ready
    assert controller.vicon_inhibit_last_reference_time_ns == clock.nanoseconds

    recover_tracking_at(controller, clock, 2_000_000_000)
    too_early = SimpleNamespace(sequence=2)
    controller.callback_reference(too_early)
    assert controller.vicon_motion_inhibited
    assert controller.reference_ is None

    # The rejected early reference becomes the new quiet-period anchor.
    new_reference = SimpleNamespace(sequence=3)
    recover_tracking_at(controller, clock, 2_500_000_000)
    controller.callback_reference(new_reference)
    assert not controller.vicon_motion_inhibited
    assert controller.reference_ is new_reference


def test_startup_without_reference_stays_stopped_and_normal_start_is_unchanged():
    controller, clock, _, events = make_controller()
    controller.last_control_time = FakeTime(clock.nanoseconds - 10_000_000)

    controller.control_tick()

    assert events == ["zero"]
    assert "pid" not in events
    assert not controller.vicon_motion_inhibited

    first_reference = SimpleNamespace(sequence=1)
    controller.callback_reference(first_reference)
    assert controller.reference_ is first_reference
    assert controller.last_reference_time_ns == clock.nanoseconds


def test_fault_before_first_reference_still_requires_quiet_then_new_reference():
    controller, clock, _, _ = make_controller()
    controller._latch_vicon_motion_inhibit(clock.nanoseconds)

    recover_tracking_at(controller, clock, 1_500_000_000)
    assert controller._update_vicon_reference_rearm(clock.nanoseconds)
    assert controller.vicon_motion_inhibited
    assert controller.reference_ is None

    first_reference = SimpleNamespace(sequence=1)
    controller.callback_reference(first_reference)
    assert not controller.vicon_motion_inhibited
    assert controller.reference_ is first_reference


def test_disabled_reference_watchdog_fails_closed_instead_of_auto_rearming():
    controller, clock, _, _ = make_controller()
    controller.reference_timeout_s = 0.0
    controller._latch_vicon_motion_inhibit(clock.nanoseconds)

    recover_tracking_at(controller, clock, 10_000_000_000)
    controller.callback_reference(SimpleNamespace(sequence=1))

    assert controller.vicon_motion_inhibited
    assert not controller.vicon_reference_rearm_ready
    assert controller.reference_ is None


def test_total_vicon_silence_invalidates_an_active_auto_reference():
    controller, clock, _, events = make_controller()
    cached_reference = SimpleNamespace(sequence=1)
    controller.reference_ = cached_reference
    controller.last_reference_time_ns = clock.nanoseconds

    clock.nanoseconds = 1_250_000_001
    assert not controller.ensure_base_odom_ready(clock.nanoseconds)

    assert events == ["zero", "reset"]
    assert controller.vicon_pose_guard.latched
    assert controller.vicon_motion_inhibited
    assert controller.reference_ is None
    assert controller.last_reference_time_ns is None
    assert controller.odom_timed_out


def test_waypoint_unprotected_mode_requires_one_odom_then_ignores_its_age():
    controller, clock, _, events = make_controller()
    controller.vicon_pose_guard_enabled = False
    controller.odom_timeout_s = 0.0
    controller.reference_ = SimpleNamespace(sequence=1)
    controller.last_reference_time_ns = clock.nanoseconds
    controller.last_odom_time_ns = None

    # Disabling the age watchdog does not permit control before the first pose.
    assert not controller.ensure_base_odom_ready(clock.nanoseconds)
    assert events == ["reset", "zero"]
    assert not controller.vicon_pose_guard.latched
    assert not controller.vicon_motion_inhibited
    assert controller.reference_ is not None

    # Once one pose has arrived, its age cannot trigger a Vicon timeout/latch.
    controller.last_odom_time_ns = clock.nanoseconds
    controller.odom_timed_out = False
    events.clear()
    clock.nanoseconds += 60_000_000_000

    assert controller.ensure_base_odom_ready(clock.nanoseconds)
    assert events == []
    assert not controller.vicon_pose_guard.latched
    assert not controller.vicon_motion_inhibited


def test_waypoint_unprotected_mode_still_enforces_reference_timeout():
    controller, clock, _, events = make_controller()
    controller.vicon_pose_guard_enabled = False
    controller.odom_timeout_s = 0.0
    controller.reference_timeout_s = 0.12
    controller.reference_ = SimpleNamespace(sequence=1)
    controller.last_reference_time_ns = clock.nanoseconds - 120_000_001
    controller.last_control_time = FakeTime(clock.nanoseconds - 10_000_000)

    controller.control_tick()

    assert events == ["reset", "zero"]
    assert controller.reference_timed_out
    assert "pid" not in events
    assert not controller.vicon_pose_guard.latched
    assert not controller.vicon_motion_inhibited


@pytest.mark.parametrize("invalid_timeout", (0.0, -0.1, math.nan, math.inf))
def test_guarded_hardware_fails_closed_without_a_positive_odom_timeout(
    invalid_timeout,
):
    controller, clock, _, events = make_controller()
    controller.odom_timeout_s = invalid_timeout
    controller.reference_ = SimpleNamespace(sequence=1)
    controller.last_reference_time_ns = clock.nanoseconds

    assert not controller.ensure_base_odom_ready(clock.nanoseconds)

    assert events == ["zero", "reset"]
    assert controller.vicon_pose_guard.latched
    assert controller.vicon_motion_inhibited
    assert controller.reference_ is None


def test_manual_mode_remains_fail_closed_after_a_rejected_vicon_sample():
    controller, clock, _, events = make_controller()
    controller.hamr_config = {"mode": "manual"}
    controller._latch_vicon_motion_inhibit(clock.nanoseconds)

    recover_tracking_at(controller, clock, 1_500_000_000)
    assert not controller.ensure_base_odom_ready(clock.nanoseconds)

    assert controller.vicon_reference_rearm_ready
    assert controller.vicon_motion_inhibited
    assert events[-1] == "zero"
    assert "pid" not in events
