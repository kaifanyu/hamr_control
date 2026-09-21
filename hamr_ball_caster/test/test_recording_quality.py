"""Reject rendered-image stalls while preserving genuine startup/final holds."""
import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1]/"scripts/record_waypoint_run.py"
spec = importlib.util.spec_from_file_location("recording_quality_runner", SCRIPT)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def trajectory():
    # The controller clock differs from the raw pose timestamp. Motion begins
    # at source time 10, so quality must use source time rather than this clock.
    return {"samples": [{"phase": "motion", "odometry_stamp_s": 10.05,
                          "trajectory_time_s": .05, "simulation_time_s": 10.30}],
            "summary": {"reference_duration_s": 1.}}


def source_images():
    return [0., 9.92] + [10.+index*.04 for index in range(26)] + [11.08, 20.]


def test_large_holds_outside_motion_do_not_fail_video_quality():
    result = runner.continuous_video_quality(trajectory(), {
        "encoded_source_timestamps_s": source_images()})
    assert result["passed"]
    assert result["motion_interval_sim_s"] == pytest.approx([10., 11.])
    assert result["max_motion_image_gap_s"] == pytest.approx(.04)
    assert not result["motion_gaps_over_limit"]


def test_encoding_duplicates_cannot_hide_a_missing_source_image_interval():
    stamps = [stamp for stamp in source_images() if not 10.2 < stamp < 10.6]
    result = runner.continuous_video_quality(trajectory(), {
        "encoded_source_timestamps_s": stamps,
        "frames_encoded": 99999, "frames_duplicated": 99000,
        "frames_queue_dropped": 0})
    assert not result["passed"]
    assert result["max_motion_image_gap_s"] > .3
    assert len(result["motion_gaps_over_limit"]) == 1


def test_exact_threshold_is_accepted_with_floating_point_tolerance():
    stamps = [0., 9.92, 10.] + [10.16+index*.04 for index in range(22)] + [11.04, 20.]
    result = runner.continuous_video_quality(trajectory(), {
        "encoded_source_timestamps_s": stamps})
    assert result["passed"]
    assert result["max_motion_image_gap_s"] == pytest.approx(.16)


def test_gap_crossing_motion_start_is_checked():
    stamps = [0., 9.70] + [10.05+index*.04 for index in range(26)] + [20.]
    result = runner.continuous_video_quality(trajectory(), {
        "encoded_source_timestamps_s": stamps})
    assert not result["passed"]
    assert result["max_motion_image_gap_s"] == pytest.approx(.35)


@pytest.mark.parametrize("recording", [
    {},
    {"encoded_source_timestamps_s": []},
    {"encoded_source_timestamps_s": [10.2, 10.4, 10.6]},
    {"encoded_source_timestamps_s": [0., 10., 10., 11.1]},
    {"encoded_source_timestamps_s": [0., float("nan"), 11.1]},
    {"encoded_source_timestamps_s": [0., 11., 10., 11.1]},
])
def test_missing_coverage_or_invalid_timestamps_fail(recording):
    result = runner.continuous_video_quality(trajectory(), recording)
    assert not result["passed"]
    assert result["failure"]


def test_missing_motion_cannot_be_reported_as_good_video():
    report = {"samples": [{"phase": "startup_hold"}],
              "summary": {"reference_duration_s": 1.}}
    result = runner.continuous_video_quality(report, {
        "encoded_source_timestamps_s": source_images()})
    assert not result["passed"]
