"""Offline tests for waypoint trajectory publication scheduling."""

import math

import numpy as np
import pytest

from reference_trajectory.waypoint_traj_simple import MAX_HOLD_SECONDS
from reference_trajectory.waypoint_traj_simple import PHASE_DONE
from reference_trajectory.waypoint_traj_simple import PHASE_FINAL_HOLD
from reference_trajectory.waypoint_traj_simple import PHASE_MOTION
from reference_trajectory.waypoint_traj_simple import PHASE_STARTUP_HOLD
from reference_trajectory.waypoint_traj_simple import PlaybackStep
from reference_trajectory.waypoint_traj_simple import scheduled_reference
from reference_trajectory.waypoint_traj_simple import SubscriberReadinessGate
from reference_trajectory.waypoint_traj_simple import TrajectoryPlayback
from reference_trajectory.waypoint_traj_simple import validated_bounded_float
from reference_trajectory.waypoint_traj_simple import validated_loop
from reference_trajectory.waypoint_traj_simple import (
    validated_required_subscribers,
)
from reference_trajectory.waypoint_traj_simple import WaypointTraj


def test_subscriber_gate_requires_an_uninterrupted_stable_interval():
    gate = SubscriberReadinessGate(required_subscribers=2, stable_s=1.0)

    assert not gate.observe(0.0, 1)
    assert not gate.observe(0.1, 2)
    assert not gate.observe(0.9, 2)
    assert not gate.observe(1.0, 1)
    assert not gate.observe(1.1, 2)
    assert not gate.observe(2.09, 2)
    assert gate.observe(2.11, 2)

    # Readiness is intentionally latched; this gate controls startup only.
    assert gate.observe(2.12, 0)


def test_subscriber_gate_resets_stability_after_clock_regression():
    gate = SubscriberReadinessGate(required_subscribers=2, stable_s=0.5)

    assert not gate.observe(10.0, 2)
    assert not gate.observe(9.0, 2)
    assert not gate.observe(9.49, 2)
    assert gate.observe(9.51, 2)
    assert SubscriberReadinessGate(0, 0.5).observe(0.0, 0)


def test_playback_clock_is_not_anchored_while_subscribers_are_unready():
    gate = SubscriberReadinessGate(required_subscribers=2, stable_s=1.0)
    playback = TrajectoryPlayback(
        trajectory_duration_s=5.0,
        startup_hold_s=2.0,
        final_hold_s=3.0,
        loop=False,
    )

    assert not gate.observe(100.0, 1)
    assert not gate.observe(101.0, 2)
    assert playback.started_at_s is None
    assert gate.observe(102.0, 2)

    first_publication = playback.step(102.0)
    assert first_publication.phase == PHASE_STARTUP_HOLD
    assert playback.started_at_s == 102.0
    assert playback.step(104.0).trajectory_time_s == 0.0


def test_one_shot_playback_holds_both_endpoints_and_never_restarts():
    playback = TrajectoryPlayback(
        trajectory_duration_s=5.0,
        startup_hold_s=2.0,
        final_hold_s=3.0,
        loop=False,
    )

    first = playback.step(10.0)
    assert first == PlaybackStep(PHASE_STARTUP_HOLD, 0.0, True, 0)
    assert playback.step(11.999).phase == PHASE_STARTUP_HOLD

    motion_start = playback.step(12.0)
    assert motion_start.phase == PHASE_MOTION
    assert motion_start.trajectory_time_s == pytest.approx(0.0)
    assert playback.step(16.999).phase == PHASE_MOTION

    endpoint = playback.step(17.0)
    assert endpoint == PlaybackStep(PHASE_FINAL_HOLD, 5.0, True, 0)
    assert playback.step(19.999).phase == PHASE_FINAL_HOLD

    done = playback.step(20.0)
    assert done == PlaybackStep(PHASE_DONE, 5.0, False, 0)
    assert playback.step(21.0) == done
    assert playback.cycle_index == 0


def test_delayed_callback_cannot_skip_the_full_final_hold():
    playback = TrajectoryPlayback(
        trajectory_duration_s=5.0,
        startup_hold_s=0.0,
        final_hold_s=3.0,
        loop=False,
    )

    assert playback.step(100.0).phase == PHASE_MOTION
    assert playback.step(200.0).phase == PHASE_FINAL_HOLD
    assert playback.step(202.999).phase == PHASE_FINAL_HOLD
    assert playback.step(203.0).phase == PHASE_DONE


def test_loop_restarts_only_after_publishing_the_endpoint():
    playback = TrajectoryPlayback(
        trajectory_duration_s=1.0,
        startup_hold_s=0.0,
        final_hold_s=0.0,
        loop=True,
    )

    assert playback.step(0.0).phase == PHASE_MOTION
    endpoint = playback.step(1.0)
    assert endpoint.phase == PHASE_FINAL_HOLD
    assert endpoint.cycle_index == 0

    restarted = playback.step(1.01)
    assert restarted.phase == PHASE_MOTION
    assert restarted.trajectory_time_s == 0.0
    assert restarted.cycle_index == 1


def test_recommended_hardware_schedule_has_expected_70_second_lifecycle():
    # The active path is 13 m long, hence 65 s at 0.20 m/s, plus the 2 s and
    # 3 s endpoint holds used by the opt-in hardware wrapper.
    playback = TrajectoryPlayback(
        trajectory_duration_s=65.0,
        startup_hold_s=2.0,
        final_hold_s=3.0,
        loop=False,
    )

    assert playback.step(0.0).phase == PHASE_STARTUP_HOLD
    assert playback.step(1.999).phase == PHASE_STARTUP_HOLD
    assert playback.step(2.0).phase == PHASE_MOTION
    assert playback.step(66.999).phase == PHASE_MOTION
    assert playback.step(67.0).phase == PHASE_FINAL_HOLD
    assert playback.step(69.999).phase == PHASE_FINAL_HOLD
    assert playback.step(70.0).phase == PHASE_DONE


def test_hold_references_have_zero_velocity():
    trajectory = WaypointTraj(
        np.array([[1.0, 2.0, 0.1], [1.0, 3.0, 0.1]]),
        v_lin=0.2,
        w_yaw=0.5,
    )

    startup = scheduled_reference(
        trajectory,
        PlaybackStep(PHASE_STARTUP_HOLD, 0.0, True, 0),
    )
    final = scheduled_reference(
        trajectory,
        PlaybackStep(
            PHASE_FINAL_HOLD, trajectory.total_time, True, 0
        ),
    )
    motion = scheduled_reference(
        trajectory,
        PlaybackStep(PHASE_MOTION, 1.0, True, 0),
    )

    assert startup == (1.0, 2.0, 0.1, 0.0, 0.0, 0.0)
    assert final == (1.0, 3.0, 0.1, 0.0, 0.0, 0.0)
    assert motion[:3] == pytest.approx((1.0, 2.2, 0.1))
    assert motion[3:] == pytest.approx((0.0, 0.2, 0.0))


@pytest.mark.parametrize("bad", [-0.1, math.nan, math.inf, -math.inf])
def test_hold_validation_rejects_nonfinite_or_negative_values(bad):
    with pytest.raises(ValueError):
        validated_bounded_float(
            "startup_hold_s", bad, 0.0, MAX_HOLD_SECONDS
        )


def test_parameter_type_and_range_validation():
    with pytest.raises(ValueError):
        validated_bounded_float(
            "final_hold_s",
            MAX_HOLD_SECONDS + 0.1,
            0.0,
            MAX_HOLD_SECONDS,
        )
    with pytest.raises(ValueError):
        validated_loop("false")
    with pytest.raises(ValueError):
        validated_required_subscribers(2.0)
    with pytest.raises(ValueError):
        validated_required_subscribers(-1)


def test_playback_rejects_a_backward_clock():
    playback = TrajectoryPlayback(trajectory_duration_s=1.0)
    playback.step(4.0)
    with pytest.raises(ValueError, match="must not move backwards"):
        playback.step(3.0)
