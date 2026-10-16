"""Physical startup requires rear-axle rotation and observed caster alignment."""
import importlib.util
import math
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location("startup_alignment",
    Path(__file__).resolve().parents[1] / "scripts/startup_alignment.py")
ALIGN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ALIGN)


def make(**config):
    return ALIGN.StartupAlignment({"alignment_dwell_s": .10, **config},
        [[0., 0.], [0., 2.], [1., 2.]], .1075, .33072, .27114)


def test_opposite_heading_commands_real_positive_half_turn():
    recovery = make()
    joints = {"left_caster_swivel_joint": 0., "right_caster_swivel_joint": 0.}
    left, right, reference, status = recovery.update(0., (0., 0.), -math.pi/2, joints, 0.)
    assert left < 0. < right
    assert recovery.target_unwrapped-recovery.initial_yaw == pytest.approx(math.pi)
    # Differential rotation displaces the base by twice its axle offset.
    assert reference == pytest.approx((0., 2*.27114))
    assert joints == {"left_caster_swivel_joint": 0., "right_caster_swivel_joint": 0.}
    assert status["phase"] == "startup_heading_recovery"


def test_270_degree_recovery_with_independent_rolling_caster_kinematics():
    recovery = make()
    x, y, yaw = 0., 0., -math.pi/2
    q = {"left": 0., "right": 0.}
    dt = .005
    speed = 0.
    phases = set()
    for step in range(10000):
        joints = {f"{side}_caster_swivel_joint": angle for side, angle in q.items()}
        left, right, _, status = recovery.update(step*dt, (x, y), ALIGN.wrap(yaw), joints, speed)
        phases.add(status["phase"])
        if recovery.done:
            break
        forward = .1075*(left+right)/2
        omega = .1075*(right-left)/(2*.33072)
        vx = forward*math.cos(yaw)-.27114*omega*math.sin(yaw)
        vy = forward*math.sin(yaw)+.27114*omega*math.cos(yaw)
        x += vx*dt
        y += vy*dt
        yaw += omega*dt
        speed = math.hypot(vx, vy)
        # A passive trailing wheel aligns from its pivot's velocity. This
        # independently models rolling geometry; no caster command is applied.
        for side, lateral in (("left", .33), ("right", -.33)):
            pivot_vx, pivot_vy = forward-omega*lateral, omega*.60
            q[side] += ((-math.sin(q[side])*pivot_vx+math.cos(q[side])*pivot_vy)/.045-omega)*dt
    report = recovery.report()
    assert report["passed"]
    assert report["initial_heading_deg"] == pytest.approx(270.)
    assert report["final_heading_unwrapped_deg"] == pytest.approx(450., abs=3.)
    assert report["actual_chassis_yaw_change_rad"] == pytest.approx(math.pi, abs=.05)
    assert .27114*2+.15 < y < 1.3
    assert phases >= {"startup_heading_recovery", "startup_caster_lead_in",
                      "startup_alignment_settle", "startup_alignment_complete"}
    assert max(abs(v) for v in report["final_caster_heading_errors_rad"].values()) < math.radians(15.)


def test_stuck_passive_caster_prevents_release_and_fails_boundedly():
    recovery = make(timeout_s=2.)
    joints = {"left_caster_swivel_joint": math.pi/2, "right_caster_swivel_joint": 0.}
    for now in (0., .11, .5, 1., 1.9):
        recovery.update(now, (0., .1*now), math.pi/2, joints, .1)
        assert not recovery.done
    with pytest.raises(RuntimeError, match="timed out"):
        recovery.update(2.1, (0., .21), math.pi/2, joints, .1)
    assert not recovery.report()["passed"]


def test_first_destination_cannot_be_consumed_by_preparation():
    recovery = make()
    joints = {"left_caster_swivel_joint": 1., "right_caster_swivel_joint": 1.}
    with pytest.raises(RuntimeError, match="first destination"):
        recovery.update(0., (0., 1.90), math.pi/2, joints, 0.)


def test_origin_angle_offset_is_added_to_measured_joint_position():
    recovery = make(caster_zero_offset_rad=math.pi/2)
    joints = {"left_caster_swivel_joint": -math.pi/2, "right_caster_swivel_joint": 0.}
    *_, status = recovery.update(0., (0., 0.), math.pi/2, joints, 0.)
    assert status["caster_heading_errors_rad"] == pytest.approx({"left": 0., "right": math.pi/2})


def test_ball_startup_rotates_physically_without_fictional_swivel_feedback():
    recovery = make(caster_model="ball")
    x, y, yaw = 0., 0., -math.pi/2
    speed = 0.
    dt = .005
    assert recovery.required_joint_names == ()
    for step in range(10000):
        left, right, _, status = recovery.update(step*dt, (x, y), ALIGN.wrap(yaw), {}, speed)
        assert status["caster_heading_errors_rad"] == {}
        if recovery.done:
            break
        forward = .1075*(left+right)/2
        omega = .1075*(right-left)/(2*.33072)
        vx = forward*math.cos(yaw)-.27114*omega*math.sin(yaw)
        vy = forward*math.sin(yaw)+.27114*omega*math.cos(yaw)
        x += vx*dt
        y += vy*dt
        yaw += omega*dt
        speed = math.hypot(vx, vy)
    report = recovery.report()
    assert report["passed"]
    assert not report["caster_orientation_gate_applied"]
    assert report["caster_model"] == "ball"
    assert report["actual_chassis_yaw_change_rad"] == pytest.approx(math.pi, abs=.05)
    assert report["lead_in_distance_m"] >= recovery.min_lead_in_distance_m
    assert report["final_caster_heading_errors_rad"] == {}
    assert .27114*2+.15 < y < 1.3


def test_unknown_caster_startup_model_is_rejected():
    with pytest.raises(ValueError, match="caster_model"):
        make(caster_model="omni")
