"""Analytic and paired regressions for physically distinct caster models."""
import importlib.util
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/"scripts"))
import analyze_ball_stress as BALL
import compare_caster_runs as COMPARE


def robot():
    return ET.parse(PACKAGE/"urdf/compa_ball_caster.urdf").getroot()


def trajectory(error=.01):
    model = robot()
    joint_names = [j.get("name") for j in model.findall("joint") if j.get("type") != "fixed"]
    data = {"completed": True, "summary": {"passed": True}, "configuration": {
        "points_m": [[0, 0], [.2, 0], [.4, 0]], "speed_m_s": .2,
        "wheel_radius_m": .1075, "half_track_m": .33072, "base_ahead_of_axle_m": .27114}, "samples": []}
    for t in np.linspace(0, 2, 101):
        positions = {name: 0. for name in joint_names}
        rates = {name: 0. for name in joint_names}
        for name in BALL.DRIVE_JOINTS:
            positions[name], rates[name] = .2/.1075*t, .2/.1075
        for side in ("left", "right"):
            positions[side+"_caster_carrier_joint"] = 2*t
            rates[side+"_caster_carrier_joint"] = 2.
        data["samples"].append({"elapsed_s": t, "odometry_stamp_s": 10+t,
            "joint_state_stamp_s": 10+t, "phase": "motion", "segment": int(t >= 1),
            "position_m": [.2*t+error, 0, .001], "reference_m": [.2*t, 0],
            "reference_velocity_m_s": [.2, 0], "base_rpy_rad": [0, 0, 0],
            "joint_positions_rad": positions, "joint_velocities_rad_s": rates,
            "raw_wheels_rad_s": [.2/.1075]*2, "commands_rad_s": [.2/.1075]*2,
            "wheel_speed_scale": 1.})
    return data


def test_ball_activity_has_no_swivel_heading_or_sphere_slip_score():
    result, _ = BALL.analyze_trajectory(trajectory(), robot())
    assert result["caster_heading_alignment"] is None
    assert result["contact_slip"] is None
    assert len(result["joint_activity"]) == 10
    activity = result["joint_activity"]["left_caster_carrier_joint"]["regions"]["motion"]
    assert activity["rate_rad_s"]["rms"] == pytest.approx(2.)
    assert activity["total_absolute_rotation_rad"] == pytest.approx(4.)
    assert result["mass_kg"]["left_caster"] > 0.


def test_common_tracking_matches_existing_swivel_weighting():
    path = PACKAGE.parent/"hamr_swivel_caster/scripts/analyze_caster_stress.py"
    spec = importlib.util.spec_from_file_location("swivel_comparison_test", path)
    swivel = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(swivel)
    model = ET.parse(PACKAGE.parent/"hamr_swivel_caster/urdf/compa_swivel_caster.urdf").getroot()
    data = trajectory()
    for sample in data["samples"]:
        for joint in model.findall("joint"):
            if joint.get("type") != "fixed":
                sample["joint_positions_rad"].setdefault(joint.get("name"), 0.)
                sample["joint_velocities_rad_s"].setdefault(joint.get("name"), 0.)
        sample["position_m"][1] = .01*sample["elapsed_s"]
        sample["wheel_speed_scale"] = .8 if sample["elapsed_s"] > 1. else 1.
    common, _, _, _ = BALL.common_metrics(data)
    expected, _ = swivel.analyze_trajectory(data, model)
    for region in ("motion", "first_leg", "later_legs", "startup", "hold"):
        assert common["regions"][region]["tracking_error_m"] == expected["tracking"][region]
        assert common["regions"][region]["either_limited_fraction"] == expected["actuator_limits"]["sampled_regions"][region]["either_limited_fraction"]


def test_ideal_straight_drive_residual_is_zero():
    result, _, _, _ = BALL.common_metrics(trajectory())
    for stats in result["regions"]["motion"]["drive_residual"].values():
        assert stats["max_abs"] < 1e-10
    assert result["regions"]["motion"]["along_track_error_m"]["rms"] == pytest.approx(.01)
    assert result["regions"]["motion"]["cross_track_error_m"]["rms"] == pytest.approx(0.)


def test_offset_pivot_rotation_is_not_falsely_reported_as_lateral_sliding():
    data = trajectory(error=0)
    a, b, r, omega = .27114, .33072, .1075, .3
    for sample in data["samples"]:
        t = sample["elapsed_s"]
        sample["position_m"] = [a*math.cos(omega*t), a*math.sin(omega*t), .001]
        sample["base_rpy_rad"][2] = omega*t
        for name, sign in zip(BALL.DRIVE_JOINTS, (-1, 1)):
            sample["joint_velocities_rad_s"][name] = sign*b*omega/r
    result, _, _, _ = BALL.common_metrics(data)
    for stats in result["regions"]["motion"]["drive_residual"].values():
        assert stats["max_abs"] < 2e-6


def test_duplicate_samples_preserve_weighted_metrics():
    original = trajectory()
    duplicate = {**original, "samples": [s for sample in original["samples"] for s in (sample, sample)]}
    expected, _, _, _ = BALL.common_metrics(original)
    actual, _, _, _ = BALL.common_metrics(duplicate)
    assert actual["regions"] == expected["regions"]
    assert actual["duplicate_odometry_samples_removed"] == 101


def test_pair_quantifies_difference_without_claiming_superiority():
    result, _ = COMPARE.compare_trajectories(trajectory(.02), trajectory(.01))
    assert result["primary_region"] == "motion"
    assert result["regions"]["motion"]["tracking_rms_change_percent"] == pytest.approx(-50.)
    assert result["comparability"]["executed_waypoints_match"]
    assert result["joint_alignment_comparison"] is None
    assert "does not establish statistical superiority" in result["assessment"]


def test_startup_shift_prioritizes_later_legs_and_flags_source_mismatch():
    swivel, ball = trajectory(), trajectory()
    swivel["startup_alignment"] = {"enabled": True, "passed": True}
    ball["startup_alignment"] = {"enabled": True, "passed": True, "caster_orientation_gate_applied": False}
    swivel["executed_route_points_m"] = [[.08, .1], [.2, 0], [.4, 0]]
    result, _ = COMPARE.compare_trajectories(swivel, ball)
    assert result["primary_region"] == "later_legs"
    assert result["comparability"]["requested_waypoints_match"]
    assert not result["comparability"]["executed_waypoints_match"]
    assert result["comparability"]["later_waypoints_match"]
    assert result["comparability"]["executed_source_separation_m"] == pytest.approx(math.hypot(.08, .1))


def test_missing_ball_joint_is_not_silently_zero():
    data = trajectory()
    del data["samples"][1]["joint_positions_rad"]["left_caster_carrier_joint"]
    with pytest.raises(ValueError, match="Missing passive ball joint"):
        BALL.analyze_trajectory(data, robot())


def test_lower_error_on_failed_run_does_not_hide_outcome():
    swivel, ball = trajectory(.02), trajectory(.01)
    ball["completed"] = False
    ball["summary"]["passed"] = False
    result, _ = COMPARE.compare_trajectories(swivel, ball,
        context={"ball_role": "nominal repeated baseline for swivel-only sensitivity"})
    assert not result["run_status"]["ball"]["completed"]
    assert "lower sampled error alone cannot" in result["assessment"]
    assert "not an equivalent parameter intervention" in result["assessment"]


def test_fast_polar_roller_accumulated_coordinates_are_not_modulo_aliased():
    data = trajectory()
    name = "left_caster_positive_polar_roller_joint"
    for sample in data["samples"]:
        sample["joint_positions_rad"][name] = 200*sample["elapsed_s"]
        sample["joint_velocities_rad_s"][name] = 200.
    result, rows = BALL.analyze_trajectory(data, robot())
    activity = result["joint_activity"][name]["regions"]["motion"]
    assert activity["total_absolute_rotation_rad"] == pytest.approx(400.)
    assert rows[name+"_position_rad"][-1] == pytest.approx(400.)
