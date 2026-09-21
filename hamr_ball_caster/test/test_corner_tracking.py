"""Accuracy acceptance must distinguish path shape, timing, and corner overrun."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("corner_tracking", PACKAGE/"tools/analyze_corner_tracking.py")
ANALYSIS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYSIS)


def report(points, arrivals, dt=.05):
    points = np.array(points, float)
    times = np.arange(-.1, arrivals[-1]+.21, dt)
    positions = np.column_stack([np.interp(times, arrivals, points[:, dim]) for dim in (0, 1)])
    return {
        "configuration": {"points_m": points.tolist(), "speed_m_s": 1.},
        "reference_schedule_s": arrivals, "reference_departure_schedule_s": arrivals,
        "startup_hold_s": .1, "completed": True,
        "summary": {"reference_duration_s": arrivals[-1]},
        "samples": [{"elapsed_s": float(time+.1), "trajectory_time_s": float(max(0., time)),
                     "phase": "startup_hold" if time < 0. else "motion",
                     "reference_m": position.tolist(), "position_m": [*position, 0.]}
                    for time, position in zip(times, positions)],
    }


def test_finite_polyline_distance_includes_endpoint_caps():
    distances = ANALYSIS.point_segment_distances([[.5, .2], [1.2, 1.3], [-.2, 0.]],
                                                 [[0., 0.], [1., 0.], [1., 1.]])
    np.testing.assert_allclose(distances, [.2, np.hypot(.2, .3), .2])


def test_on_path_lag_fails_tracking_despite_zero_geometric_error():
    data = report([[0., 0.], [2., 0.]], [0., 2.])
    for sample in data["samples"]:
        if .3 < sample["trajectory_time_s"] < 1.8:
            sample["position_m"][0] -= .05
    result = ANALYSIS.analyze_report(data)
    assert result["motion_nearest_route_error"]["max_m"] == pytest.approx(0.)
    assert result["motion_time_aligned_error"]["max_m"] == pytest.approx(.05)
    assert not result["passed"]
    assert any("max_tracking_error_m" in failure for failure in result["failures"])
    assert not any("max_cross_track_m" in failure for failure in result["failures"])


def test_collinear_waypoint_not_classified_as_corner():
    data = report([[0., 0.], [1., 0.], [1., 1.], [1., 2.]], [0., 1., 2., 3.])
    result = ANALYSIS.analyze_report(data)
    assert result["passed"]
    assert len(result["corners"]) == 1
    assert result["corners"][0]["waypoint_index"] == 1
    assert result["waypoints"][2]["is_corner"] is False
    assert result["corner_max_incoming_direction_overshoot_m"] == pytest.approx(0.)


def test_incoming_direction_overrun_detected_even_at_a_stop():
    data = report([[0., 0.], [1., 0.], [1., 1.]], [0., 1., 2.])
    sample = min(data["samples"], key=lambda sample: abs(sample["trajectory_time_s"]-1.))
    sample["position_m"][0] += .003
    result = ANALYSIS.analyze_report(data)
    assert result["corner_max_incoming_direction_overshoot_m"] == pytest.approx(.003)
    assert not result["passed"]
    assert any("max_corner_overshoot_m" in failure for failure in result["failures"])


def test_delayed_reorientation_cannot_hide_outside_local_corner_window():
    data = report([[0., 0.], [1., 0.], [1., 6.]], [0., 1., 7.])
    sample = min(data["samples"], key=lambda sample: abs(sample["trajectory_time_s"]-4.5))
    sample["position_m"][0] += .003
    result = ANALYSIS.analyze_report(data)
    corner = result["corners"][0]
    assert corner["local_window_incoming_direction_overshoot_m"] == pytest.approx(0.)
    assert corner["entire_outgoing_leg_incoming_direction_overshoot_m"] == pytest.approx(.003)
    assert corner["incoming_direction_overshoot_m"] == pytest.approx(.003)
    assert corner["entire_outgoing_leg_line_error_after_departure"]["max_m"] == pytest.approx(.003)
    assert not result["passed"]
    assert any("max_corner_overshoot_m" in failure for failure in result["failures"])
    assert not any("max_cross_track_m" in failure for failure in result["failures"])


def test_dwell_schedules_use_actual_departure_for_outgoing_cross_track():
    data = report([[0., 0.], [1., 0.], [1., 1.]], [0., 1., 3.])
    data["reference_departure_schedule_s"] = [0., 2., 3.]
    for sample in data["samples"]:
        t = sample["trajectory_time_s"]
        position = [min(t, 1.), max(0., min(1., t-2.))]
        sample["position_m"] = [*position, 0.]
        sample["reference_m"] = position
    result = ANALYSIS.analyze_report(data)
    assert result["passed"]
    assert result["waypoints"][1]["hold_duration_s"] == pytest.approx(1.)
    assert result["corners"][0]["outgoing_line_error_after_departure"]["max_m"] == pytest.approx(0.)


@pytest.mark.parametrize("mutation", ["incomplete", "nonfinite", "missing_end", "bad_schedule"])
def test_invalid_or_incomplete_run_cannot_pass(mutation):
    data = report([[0., 0.], [1., 0.], [1., 1.]], [0., 1., 2.])
    if mutation == "incomplete":
        data["completed"] = False
    elif mutation == "nonfinite":
        data["samples"][3]["position_m"][0] = float("nan")
    elif mutation == "missing_end":
        data["samples"] = data["samples"][:10]
    else:
        data["reference_schedule_s"] = [0., 1.]
    result = ANALYSIS.analyze_report(data)
    assert not result["passed"]
    assert result["failures"]


def test_legacy_schedule_fallback_preserves_original_waypoint_timing():
    data = report([[0., 0.], [2., 0.], [2., 1.]], [0., 2., 3.])
    del data["reference_schedule_s"]
    del data["reference_departure_schedule_s"]
    del data["completed"]
    data["summary"]["passed"] = True
    result = ANALYSIS.analyze_report(data)
    assert result["passed"]
    assert [waypoint["scheduled_arrival_s"] for waypoint in result["waypoints"]] == [0., 2., 3.]


def test_shallow_turn_does_not_call_intended_forward_motion_overshoot():
    data = report([[0., 0.], [1., 0.], [2., 1.]], [0., 1., 2.])
    result = ANALYSIS.analyze_report(data)
    assert result["passed"]
    assert result["corners"][0]["incoming_direction_overshoot_m"] is None
    assert result["corners"][0]["overshoot_applicable"] is False


def test_configured_strict_limits_are_applied():
    data = report([[0., 0.], [1., 0.]], [0., 1.])
    for sample in data["samples"]:
        if .2 < sample["trajectory_time_s"] < .8:
            sample["position_m"][1] += .003
    assert ANALYSIS.analyze_report(data)["passed"]
    strict = ANALYSIS.analyze_report(data, {"max_cross_track_m": .002})
    assert not strict["passed"]
    assert strict["acceptance"]["limits_m"]["max_cross_track_m"] == .002


def test_cli_inherits_recorded_limits_and_overrides_only_explicit_values():
    data = report([[0., 0.], [1., 0.]], [0., 1.])
    assert ANALYSIS.cli_limits(data) == ANALYSIS.DEFAULT_LIMITS
    strict = {"max_tracking_error_m": .005, "max_cross_track_m": .002,
              "max_corner_overshoot_m": .001, "max_final_error_m": .0005}
    data["configuration"]["acceptance"] = strict.copy()
    assert ANALYSIS.cli_limits(data) == strict
    actual = ANALYSIS.cli_limits(data, {"max_tracking_error_m": None, "max_corner_overshoot_m": .0007})
    assert actual == {**strict, "max_corner_overshoot_m": .0007}
    assert data["configuration"]["acceptance"] == strict
