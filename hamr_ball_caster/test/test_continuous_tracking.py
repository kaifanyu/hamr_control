"""Independent geometric, timing, and motion-continuity acceptance checks."""
import importlib.util
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("continuous_tracking", PACKAGE/"tools/analyze_continuous_tracking.py")
ANALYSIS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYSIS)


def curve(time_s):
    """An analytic rounded quarter circle with smooth zero-speed endpoints."""
    u = np.clip(np.asarray(time_s)/10., 0., 1.)
    angle = np.pi/2 * (10*u**3-15*u**4+6*u**5)
    rate = np.pi/2 * 30*u**2*(1-u)**2/10.
    acceleration = np.pi/2 * 60*u*(1-u)*(1-2*u)/100.
    normal = np.column_stack((np.cos(angle), np.sin(angle)))
    tangent = np.column_stack((-np.sin(angle), np.cos(angle)))
    return normal, tangent*rate[:, None], tangent*acceleration[:, None]-normal*rate[:, None]**2


def report():
    measured_time = np.linspace(-.1, 10.2, 516)
    plan_time = np.linspace(0., 10., 1001)
    position, velocity, _ = curve(measured_time)
    plan_position, plan_velocity, plan_acceleration = curve(plan_time)
    return {
        "completed": True, "trajectory_profile": "continuous",
        "configuration": {"points_m": [[1., 0.], [1., 1.], [0., 1.]]},
        "continuous_plan": {
            "duration_s": 10., "max_acceleration_m_s2": .2,
            "interior_motion_interval_s": [2., 8.],
            "corners": [{"waypoint_index": 1, "start_time_s": 3., "end_time_s": 7.}],
            "samples": [{"time_s": float(t), "position_m": p.tolist(), "velocity_m_s": v.tolist(),
                         "acceleration_m_s2": a.tolist()}
                        for t, p, v, a in zip(plan_time, plan_position, plan_velocity, plan_acceleration)],
        },
        "command_statistics": {"updates": 1000, "speed_limited_updates": 0, "acceleration_limited_updates": 0},
        "samples": [{"trajectory_time_s": float(t), "position_m": [*p, 0.], "reference_m": p.tolist(),
                     "estimated_world_velocity_m_s": v.tolist()}
                    for t, p, v in zip(measured_time, position, velocity)],
    }


def test_intended_rounding_is_not_mistaken_for_tracking_error():
    data = report()
    result = ANALYSIS.analyze_report(data)
    assert result["passed"], result["failures"]
    assert result["motion_time_aligned_error"]["max_m"] == pytest.approx(0.)
    assert result["conservative_max_curve_error_m"] < .00001
    midpoint = data["continuous_plan"]["samples"][500]["position_m"]
    # Deliberate deviation from the original sharp L exceeds 290 mm.
    assert min(1-midpoint[0], 1-midpoint[1]) > .29
    assert result["intended_rounding"]["sampled_max_deviation_from_original_polyline_m"] > .29
    assert result["source_time_reference_consistency"]["max_m"] <= result["source_time_reference_consistency_tolerance_m"]


def test_reference_clock_mismatch_fails_even_below_tracking_error_threshold():
    data = report()
    sample = min(data["samples"], key=lambda sample: abs(sample["trajectory_time_s"]-5.))
    sample["reference_m"][0] += .001
    result = ANALYSIS.analyze_report(data)
    assert not result["passed"]
    assert result["motion_time_aligned_error"]["max_m"] == pytest.approx(.001)
    assert any("reference is inconsistent with source time" in failure for failure in result["failures"])


def test_late_curve_deviation_is_checked_through_the_entire_corner():
    data = report()
    sample = min(data["samples"], key=lambda sample: abs(sample["trajectory_time_s"]-6.8))
    sample["position_m"][0] += .006
    result = ANALYSIS.analyze_report(data)
    assert not result["passed"]
    assert result["corners"][0]["tracking_error"]["max_m"] == pytest.approx(.006)
    assert any("max_tracking_error_m" in failure for failure in result["failures"])


def test_interior_stop_between_corner_intervals_cannot_hide():
    data = report()
    sample = min(data["samples"], key=lambda sample: abs(sample["trajectory_time_s"]-2.5))
    sample["estimated_world_velocity_m_s"] = [0., 0.]
    result = ANALYSIS.analyze_report(data)
    assert not result["passed"]
    assert result["interior_actual_speed"]["min_m_s"] == 0.
    assert any("Interior motion" in failure for failure in result["failures"])
    assert result["corners"][0]["actual_speed"]["min_m_s"] > .03


def test_corner_slowdown_relative_to_plan_fails_even_while_moving():
    data = report()
    sample = min(data["samples"], key=lambda sample: abs(sample["trajectory_time_s"]-5.))
    sample["estimated_world_velocity_m_s"] = (np.asarray(sample["estimated_world_velocity_m_s"])*.6).tolist()
    result = ANALYSIS.analyze_report(data)
    assert not result["passed"]
    assert result["corners"][0]["max_slowdown_fraction"] == pytest.approx(.4)
    assert any("slowdown fraction" in failure for failure in result["failures"])


@pytest.mark.parametrize("mutation", ["incomplete", "gapped", "nonfinite", "missing_final", "limiter", "coarse_curve", "unbounded_curve"])
def test_missing_evidence_or_violated_limits_cannot_pass(mutation):
    data = report()
    if mutation == "incomplete":
        data["completed"] = False
    elif mutation == "gapped":
        del data["samples"][200:210]
    elif mutation == "nonfinite":
        data["samples"][200]["position_m"][0] = float("nan")
    elif mutation == "missing_final":
        data["samples"] = data["samples"][:200]
    elif mutation == "limiter":
        data["command_statistics"]["acceleration_limited_updates"] = 1
    elif mutation == "coarse_curve":
        data["continuous_plan"]["samples"] = data["continuous_plan"]["samples"][::100]
    else:
        del data["continuous_plan"]["max_acceleration_m_s2"]
    result = ANALYSIS.analyze_report(data)
    assert not result["passed"]
    assert result["failures"]


def test_local_progress_window_prevents_distant_route_crossing_from_hiding_error():
    # The route revisits the origin much later; measuring there at t=1 cannot use
    # the distant t=10 branch to claim zero local path deviation.
    points = np.array([[0., 0.], [1., 0.], [2., 0.], [0., 0.], [0., 1.]])
    times = np.array([0., 1., 2., 10., 11.])
    actual = ANALYSIS.local_curve_distance(np.array([[0., 1.]]), np.array([1.]), points, times, .1)
    assert actual[0] >= 1.


def test_recorded_thresholds_and_explicit_overrides_are_preserved():
    data = report()
    data["configuration"]["acceptance"] = {"max_tracking_error_m": .003, "min_corner_speed_m_s": .04}
    limits = ANALYSIS.resolved_limits(data, {"max_tracking_error_m": .004, "min_corner_speed_m_s": None})
    assert limits["max_tracking_error_m"] == .004
    assert limits["min_corner_speed_m_s"] == .04
    assert limits["max_cross_track_m"] == .002


def test_cli_cannot_overwrite_raw_measurements(tmp_path):
    source = tmp_path/"trajectory.json"
    original = '{"raw_measurements": "must remain unchanged"}\n'
    source.write_text(original)
    result = subprocess.run([sys.executable, str(PACKAGE/"tools/analyze_continuous_tracking.py"),
                             str(source), "--output", str(source.with_suffix("")), "--no-plot"],
                            capture_output=True, text=True)
    assert result.returncode == 2
    assert "must differ from the raw input report" in result.stderr
    assert source.read_text() == original
