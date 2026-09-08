"""Pure/offline tests for study bag validation and aggregation."""

import csv
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml


SCRIPT = (
    Path(__file__).resolve().parents[2]
    / "rosbags"
    / "analyze_hamr_study.py"
)
SPEC = importlib.util.spec_from_file_location("analyze_hamr_study", SCRIPT)
analyzer = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = analyzer
SPEC.loader.exec_module(analyzer)


def make_bag(root: Path, name: str, missing_topic: str | None = None) -> Path:
    bag = root / name
    bag.mkdir()
    storage = bag / "data_0.mcap"
    storage.write_bytes(b"nonempty")
    topics = {
        topic: (0 if topic == missing_topic else 1) for topic in analyzer.CORE_TOPICS
    }
    metadata = {
        "rosbag2_bagfile_information": {
            "storage_identifier": "mcap",
            "relative_file_paths": [storage.name],
            "topics_with_message_count": [
                {
                    "topic_metadata": {"name": topic},
                    "message_count": count,
                }
                for topic, count in topics.items()
            ],
        }
    }
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump(metadata), encoding="utf-8"
    )
    return bag


def complete_reference():
    timestamps = np.linspace(0.0, 10.0, 501)
    velocity = np.zeros_like(timestamps)
    velocity[(timestamps >= 2.0) & (timestamps < 7.0)] = 0.2
    return {
        "t": timestamps,
        "x": np.zeros_like(timestamps),
        "y": np.zeros_like(timestamps),
        "yaw": np.zeros_like(timestamps),
        "vx": velocity,
        "vy": np.zeros_like(timestamps),
    }


def manifest(profile="triangle"):
    return {
        "profile": profile,
        "closed": True,
        "origin_x_m": 0.0,
        "origin_y_m": 0.0,
        "expected_total_duration_s": 10.0,
        "expected_motion_duration_s": 5.0,
        "reference_timer_hz": 50.0,
        "final_hold_s": 3.0,
    }


def planned_trial(
    repetition=1,
    *,
    profile="straight_forward",
    planned_id=None,
    planned_order=None,
    profile_sha256=None,
    gate_added="1",
    matched_block_id=None,
):
    return {
        "planned_id": planned_id or f"{profile}-{repetition}",
        "planned_order": str(
            repetition if planned_order is None else planned_order
        ),
        "gate_added": str(gate_added),
        "matched_block_id": matched_block_id or f"G{gate_added}-R{repetition}",
        "hardware_block_id": f"H-{profile}-{repetition}",
        "profile": profile,
        "profile_sha256": profile_sha256 or f"{profile}-hash",
        "study_stack_contract_version": "2",
        "controller_config_sha256": "controller-hash",
        "wheel_speed_limit_rad_s": "3.386636",
        "turret_control_enabled": "false",
        "nominal_start_base_yaw_rad": "0.0",
        "start_base_yaw_tolerance_rad": "0.15",
        "speed_m_s": "0.20",
        "caster_type": "traditional",
        "terrain": "flat",
        "initial_caster_orientation_deg": "0",
        "repetition": str(repetition),
    }


def test_preflight_requires_closed_storage_and_core_topics(tmp_path):
    valid_bag = make_bag(tmp_path, "hamr_study_valid")
    missing_bag = make_bag(
        tmp_path, "hamr_study_missing", analyzer.METADATA_TOPIC
    )

    valid, _reason, counts = analyzer.metadata_preflight(valid_bag)
    missing, reason, _counts = analyzer.metadata_preflight(missing_bag)

    assert valid
    assert counts[analyzer.REFERENCE_TOPIC] == 1
    assert not missing
    assert analyzer.METADATA_TOPIC in reason


def test_latest_selection_skips_malformed_newer_bag(tmp_path):
    older = make_bag(tmp_path, "hamr_study_triangle_old")
    newer = make_bag(
        tmp_path, "hamr_study_triangle_new", analyzer.REFERENCE_TOPIC
    )
    (older / "metadata.yaml").touch()
    (newer / "metadata.yaml").touch()
    newer_mtime = (newer / "metadata.yaml").stat().st_mtime + 5.0
    import os

    os.utime(newer / "metadata.yaml", (newer_mtime, newer_mtime))

    selected, rejected = analyzer.select_latest_study_bag(
        tmp_path, manifest_reader=lambda _bag: manifest()
    )

    assert selected == older.resolve()
    assert rejected and analyzer.REFERENCE_TOPIC in rejected[0]


def test_completion_is_relative_to_manifest_and_first_last_reference():
    lifecycle, reasons = analyzer.reference_lifecycle(
        complete_reference(), manifest()
    )

    assert lifecycle["complete"]
    assert lifecycle["completion_fraction"] == pytest.approx(1.0)
    assert lifecycle["endpoint_delta_m"] == 0.0
    assert lifecycle["final_zero_s"] >= 3.0
    assert reasons == []


def test_time_reference_metadata_exposes_video_alignment_clock_and_motion():
    epoch_s = 1_787_781_500.0
    reference = complete_reference()
    reference["t"] = reference["t"] + epoch_s
    bag_start_ns = int((epoch_s - 1.25) * 1e9)

    result = analyzer.build_time_reference_metadata(
        {"starting_time": {"nanoseconds_since_epoch": bag_start_ns}},
        reference,
    )

    assert result["clock"] == "rosbag storage/receipt timestamp"
    assert result["first_reference"][
        "elapsed_from_first_reference_s"
    ] == pytest.approx(0.0)
    assert result["first_reference"]["receipt_time_utc"].endswith("+00:00")
    assert result["first_reference"]["receipt_time_local"]
    assert result["bag_start"][
        "elapsed_from_first_reference_s"
    ] == pytest.approx(-1.25, abs=1e-6)
    assert result["last_reference"][
        "elapsed_from_first_reference_s"
    ] == pytest.approx(10.0)
    assert len(result["active_motion_phases"]) == 1
    assert result["plot_motion_onset_s"] == pytest.approx(2.0)
    assert result["video_alignment_formula"] == (
        "video_time_s = plot_elapsed_s + video_motion_onset_s "
        "- plot_motion_onset_s"
    )
    phase = result["active_motion_phases"][0]
    assert phase["active_start"][
        "elapsed_from_first_reference_s"
    ] == pytest.approx(2.0)
    assert phase["last_active_sample"][
        "elapsed_from_first_reference_s"
    ] == pytest.approx(6.98)
    assert phase["active_stop"][
        "elapsed_from_first_reference_s"
    ] == pytest.approx(7.0)


def test_elapsed_tick_spacing_keeps_one_second_detail_for_study_run():
    assert analyzer.elapsed_tick_spacing(67.84) == (5.0, 1.0)
    assert analyzer.elapsed_tick_spacing(10.0) == (1.0, 0.2)


def test_video_timeline_exports_exact_primary_and_delivery_diagnostics(tmp_path):
    epoch_s = 1_787_781_500.0
    timestamps = epoch_s + np.arange(4, dtype=float)
    moving = np.asarray([False, True, True, False])
    reference = {
        "t": timestamps,
        "x": np.asarray([0.0, 0.0, 0.2, 0.4]),
        "y": np.zeros(4),
        "yaw": np.zeros(4),
        "vx": np.asarray([0.0, 0.2, 0.2, 0.0]),
        "vy": np.zeros(4),
    }
    time_reference = analyzer.build_time_reference_metadata({}, reference)
    traces = {
        "t": timestamps,
        "moving": moving,
        "ref_x": reference["x"],
        "ref_y": reference["y"],
        "actual_x": np.asarray([0.0, 0.01, 0.21, 0.40]),
        "actual_y": np.asarray([0.0, 0.002, 0.003, 0.0]),
        "receipt_actual_x": np.asarray([0.0, 0.03, 0.24, 0.40]),
        "receipt_actual_y": np.asarray([0.0, 0.004, 0.006, 0.0]),
        "primary_error_x": np.asarray([0.0, 0.01, 0.01, 0.0]),
        "primary_error_y": np.asarray([0.0, 0.002, 0.003, 0.0]),
        "primary_error_xy": np.asarray([0.0, 0.0102, 0.0104, 0.0]),
        "receipt_error_xy": np.asarray([0.0, 0.0303, 0.0404, 0.0]),
        "primary_along_error": np.asarray([0.01, 0.01]),
        "primary_cross_error": np.asarray([0.002, 0.003]),
        "receipt_along_error": np.asarray([0.03, 0.04]),
        "receipt_cross_error": np.asarray([0.004, 0.006]),
        "command_speed": np.asarray([0.2, 0.2]),
        "primary_actual_tangent_speed": np.asarray([0.19, 0.20]),
        "receipt_actual_tangent_speed": np.asarray([0.31, 0.08]),
        "source_receipt_residual": np.asarray([0.0, 0.012, -0.018, 0.0]),
    }

    path = analyzer.write_video_timeline(
        tmp_path / "video_timeline.csv", traces, time_reference
    )
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    assert [row["phase"] for row in rows] == [
        "startup_hold",
        "active_motion",
        "active_motion",
        "final_hold",
    ]
    assert float(rows[1]["elapsed_from_first_reference_s"]) == pytest.approx(1.0)
    assert float(rows[1]["elapsed_from_first_motion_s"]) == pytest.approx(0.0)
    assert float(rows[1]["source_time_tangent_speed_m_s"]) == pytest.approx(0.19)
    assert float(
        rows[1]["receipt_time_tangent_speed_diagnostic_m_s"]
    ) == pytest.approx(0.31)
    assert float(rows[2]["source_aligned_xy_error_m"]) == pytest.approx(0.0104)
    assert float(
        rows[2]["receipt_aligned_xy_error_diagnostic_m"]
    ) == pytest.approx(0.0404)
    assert rows[0]["source_time_tangent_speed_m_s"] == ""
    assert rows[0]["reference_receipt_time_utc"].endswith("+00:00")


def test_partial_reference_is_analyzable_but_not_complete():
    reference = complete_reference()
    partial = {key: value[:300] for key, value in reference.items()}

    lifecycle, reasons = analyzer.reference_lifecycle(partial, manifest())

    assert not lifecycle["complete"]
    assert lifecycle["completion_fraction"] < 1.0
    assert reasons


def test_lifecycle_rejects_extra_cycle_and_excess_duration():
    reference = complete_reference()
    extra = {
        key: np.concatenate((value, value[1:] if key != "t" else value[1:] + 10.0))
        for key, value in reference.items()
    }

    lifecycle, reasons = analyzer.reference_lifecycle(extra, manifest())

    assert not lifecycle["complete"]
    assert lifecycle["motion_cycle_count"] == 2
    assert any("multiple motion cycles" in reason for reason in reasons)
    assert any("duration disagrees" in reason for reason in reasons)


def test_primary_tracking_error_uses_motion_not_endpoint_holds():
    reference = complete_reference()
    moving = np.hypot(reference["vx"], reference["vy"]) > 0.02
    actual_x = np.full(reference["t"].shape, 10.0)
    actual_x[moving] = 0.10
    base = {
        "t": reference["t"],
        "x": actual_x,
        "y": np.zeros_like(actual_x),
        "z": np.full_like(actual_x, 0.31),
        "yaw": np.zeros_like(actual_x),
    }

    metrics, _traces = analyzer.tracking_metrics(reference, base, None)

    assert metrics["xy_error_m"]["rmse"] == pytest.approx(0.10)
    assert metrics["all_lifecycle_xy_error_m"]["rmse"] > 5.0
    assert metrics["final_hold_xy_error_m"] == pytest.approx(10.0)
    assert metrics["first_reference_match_dt_s"] == 0.0
    assert metrics["last_reference_match_dt_s"] == 0.0


def test_final_vicon_coverage_is_reported_separately_from_fraction():
    reference = complete_reference()
    keep = reference["t"] <= 7.5
    base = {
        "t": reference["t"][keep],
        "x": np.zeros(np.count_nonzero(keep)),
        "y": np.zeros(np.count_nonzero(keep)),
        "z": np.full(np.count_nonzero(keep), 0.31),
        "yaw": np.zeros(np.count_nonzero(keep)),
    }

    metrics, _traces = analyzer.tracking_metrics(reference, base, None)

    assert metrics["first_reference_match_dt_s"] == 0.0
    assert metrics["last_reference_match_dt_s"] > 2.0
    assert metrics["final_hold_xy_error_m"] is None


def test_circle_metrics_report_radial_bias_and_phase_lag():
    angle = np.linspace(-np.pi / 2.0, 3.0 * np.pi / 2.0, 400)
    reference = {
        "x": np.cos(angle),
        "y": np.sin(angle),
        "vx": -np.sin(angle) * 0.2,
        "vy": np.cos(angle) * 0.2,
    }
    actual_angle = angle - 0.1
    traces = {
        "moving": np.ones(angle.size, dtype=bool),
        "ref_x": reference["x"],
        "ref_y": reference["y"],
        "actual_x": 0.9 * np.cos(actual_angle),
        "actual_y": 0.9 * np.sin(actual_angle),
        "moving_t": np.linspace(0.0, 10.0, angle.size),
    }

    metrics, _traces = analyzer.circle_metrics(
        reference, traces, {"direction": "ccw"}
    )

    assert metrics["radius_m"] == pytest.approx(1.0, abs=1e-3)
    assert metrics["radial_error_m"]["mean"] == pytest.approx(-0.1, abs=1e-3)
    assert metrics["phase_progress_error_rad"]["mean"] == pytest.approx(
        -0.1, abs=1e-3
    )
    assert metrics["lap_completion_fraction"] == pytest.approx(1.0, abs=1e-3)


def test_triangle_metrics_report_interior_and_commanded_turn_angles():
    root_three_over_four = 3.0 * np.sqrt(3.0) / 4.0
    directions = np.asarray(
        (
            (0.0, 1.0),
            (-0.5, -np.sqrt(3.0) / 2.0),
            (0.5, -np.sqrt(3.0) / 2.0),
            (0.0, 1.0),
        )
    )
    timestamps = np.arange(4.0)
    x = np.asarray((0.0, 0.0, -0.75, 0.0))
    y = np.asarray((0.0, root_three_over_four, 0.0, -root_three_over_four))
    reference = {
        "t": timestamps,
        "x": x,
        "y": y,
        "vx": 0.15 * directions[:, 0],
        "vy": 0.15 * directions[:, 1],
    }
    traces = {
        "t": timestamps,
        "actual_x": x.copy(),
        "actual_y": y.copy(),
        "moving_t": timestamps,
        "tangent_x": directions[:, 0],
        "tangent_y": directions[:, 1],
        "cross_error": np.zeros(4),
    }

    metrics = analyzer.detect_triangle_corners(reference, traces)

    assert metrics["corner_count"] == 3
    assert metrics["leg_count"] == 4
    assert [item["turn"] for item in metrics["corners"]] == [
        "left", "left", "left"
    ]
    assert [
        item["commanded_turn_angle_deg"] for item in metrics["corners"]
    ] == pytest.approx((150.0, 60.0, 150.0))
    assert [
        item["interior_angle_deg"] for item in metrics["corners"]
    ] == pytest.approx((30.0, 120.0, 30.0))
    assert [
        item["interior_angle_deg"]
        for item in metrics["corner_angle_groups"]
    ] == pytest.approx((30.0, 120.0))
    assert [
        item["corner_count"] for item in metrics["corner_angle_groups"]
    ] == [2, 1]


def test_manifest_rows_must_be_stable_json():
    rows = [
        (0.0, SimpleNamespace(data='{"profile":"triangle","run_id":"r1"}')),
        (1.0, SimpleNamespace(data='{"run_id":"r1","profile":"triangle"}')),
    ]

    decoded = analyzer.parse_manifest_rows(rows)

    assert decoded["run_id"] == "r1"
    rows.append((2.0, SimpleNamespace(data='{"profile":"circle_ccw"}')))
    with pytest.raises(RuntimeError, match="changed"):
        analyzer.parse_manifest_rows(rows)


def test_existing_operator_exclusion_is_preserved_until_explicit_include(tmp_path):
    validation = tmp_path / "analysis" / "validation.json"
    validation.parent.mkdir()
    validation.write_text(
        '{"status":"excluded","exclude_reason":"person entered corridor"}',
        encoding="utf-8",
    )

    assert analyzer.existing_exclusion(validation) == "person entered corridor"
    assert analyzer.existing_exclusion(tmp_path / "missing.json") == ""


def test_aggregate_update_is_idempotent_by_run_id(tmp_path):
    row = {field: "" for field in analyzer.RUN_FIELDS}
    row.update(
        {
            "run_id": "run-001",
            "profile": "triangle",
            "speed_m_s": "0.15",
            "caster_type": "trailing",
            "terrain": "flat",
            "initial_caster_orientation_deg": "90",
            "valid_for_aggregate": "true",
            "xy_rmse_m": "0.05",
            "cross_track_rmse_m": "0.04",
            "speed_bias_m_s": "-0.01",
            "final_xy_error_m": "0.02",
        }
    )

    analyzer.update_study_results(tmp_path, row)
    analyzer.update_study_results(tmp_path, row)

    with (tmp_path / "runs.csv").open(newline="") as stream:
        runs = list(csv.DictReader(stream))
    with (tmp_path / "aggregate_metrics.csv").open(newline="") as stream:
        aggregate = list(csv.DictReader(stream))
    assert len(runs) == 1
    assert len(aggregate) == 1
    assert aggregate[0]["valid_run_count"] == "1"
    assert float(aggregate[0]["xy_rmse_mean_m"]) == pytest.approx(0.05)


def test_profile_hash_revisions_are_never_pooled(tmp_path):
    base = {field: "" for field in analyzer.RUN_FIELDS}
    base.update(
        {
            "profile": "triangle",
            "speed_m_s": "0.15",
            "caster_type": "trailing",
            "terrain": "flat",
            "initial_caster_orientation_deg": "0",
            "valid_for_aggregate": "true",
            "xy_rmse_m": "0.05",
        }
    )
    first = {**base, "run_id": "run-a", "profile_sha256": "aaa"}
    second = {**base, "run_id": "run-b", "profile_sha256": "bbb"}

    analyzer.update_study_results(tmp_path, first)
    analyzer.update_study_results(tmp_path, second)

    with (tmp_path / "aggregate_metrics.csv").open(newline="") as stream:
        aggregate = list(csv.DictReader(stream))
    assert len(aggregate) == 2
    assert {item["profile_sha256"] for item in aggregate} == {"aaa", "bbb"}



def forward_reverse_reference():
    timestamps = np.arange(0.0, 8.001, 0.02)
    forward = (timestamps >= 1.0) & (timestamps < 3.0)
    reverse = (timestamps >= 4.0) & (timestamps < 6.0)
    velocity_y = np.zeros_like(timestamps)
    velocity_y[forward] = 0.2
    velocity_y[reverse] = -0.2
    position_y = np.zeros_like(timestamps)
    position_y[forward] = 0.2 * (timestamps[forward] - 1.0)
    position_y[(timestamps >= 3.0) & (timestamps < 4.0)] = 0.4
    position_y[reverse] = 0.4 - 0.2 * (timestamps[reverse] - 4.0)
    return {
        "t": timestamps,
        "x": np.zeros_like(timestamps),
        "y": position_y,
        "yaw": np.zeros_like(timestamps),
        "vx": np.zeros_like(timestamps),
        "vy": velocity_y,
    }


def base_samples(reference, keep=None):
    if keep is None:
        keep = np.ones(reference["t"].shape, dtype=bool)
    timestamps = reference["t"][keep]
    return {
        "t": timestamps,
        "x": reference["x"][keep],
        "y": reference["y"][keep],
        "z": np.full(timestamps.shape, 0.31),
        "yaw": np.zeros(timestamps.shape),
        "tilt": np.zeros(timestamps.shape),
    }


def test_lifecycle_accepts_declared_forward_dwell_reverse_phases():
    reference = forward_reverse_reference()
    run_manifest = {
        "profile": "forward_reverse",
        "closed": True,
        "origin_x_m": 0.0,
        "origin_y_m": 0.0,
        "expected_total_duration_s": 8.0,
        "expected_motion_duration_s": 5.0,
        "expected_motion_phase_count": 2,
        "expected_dwell_duration_s": 1.0,
        "startup_hold_s": 1.0,
        "final_hold_s": 2.0,
        "reference_timer_hz": 50.0,
    }

    lifecycle, reasons = analyzer.reference_lifecycle(reference, run_manifest)

    assert lifecycle["complete"]
    assert lifecycle["motion_cycle_count"] == 2
    assert lifecycle["motion_phase_count"] == 2
    assert lifecycle["observed_internal_dwell_s"] == pytest.approx(1.02)
    assert reasons == []


def test_translation_primary_window_gap_is_reference_clock_anchored_and_invalid():
    timestamps = np.arange(0.0, 5.001, 0.02)
    moving = (timestamps >= 1.0) & (timestamps < 4.0)
    reference = {
        "t": timestamps,
        "x": np.zeros_like(timestamps),
        "y": np.where(moving, 0.2 * (timestamps - 1.0), 0.0),
        "yaw": np.zeros_like(timestamps),
        "vx": np.zeros_like(timestamps),
        "vy": np.where(moving, 0.2, 0.0),
    }
    onset_gap = ~((timestamps >= 1.35) & (timestamps <= 1.65))

    quality = analyzer.primary_window_data_quality(
        reference,
        base_samples(reference, onset_gap),
        {"profile": "lateral_left"},
    )
    onset = next(
        item
        for item in quality["windows"]
        if item["label"] == "translation_onset_2s"
    )

    assert onset["start_time_s"] == pytest.approx(1.0)
    assert onset["end_time_s"] == pytest.approx(3.0)
    assert onset["coverage_fraction"] < 1.0
    assert onset["max_effective_vicon_gap_s"] > 0.10
    assert not onset["valid"]
    assert not quality["valid"]


def test_common_metric_lateral_gap_after_onset_is_formally_invalid():
    timestamps = np.arange(0.0, 5.001, 0.02)
    moving = (timestamps >= 1.0) & (timestamps < 4.0)
    reference = {
        "t": timestamps,
        "x": np.where(moving, 0.2 * (timestamps - 1.0), 0.0),
        "y": np.zeros_like(timestamps),
        "yaw": np.zeros_like(timestamps),
        "vx": np.where(moving, 0.2, 0.0),
        "vy": np.zeros_like(timestamps),
    }
    late_gap = ~((timestamps >= 3.30) & (timestamps <= 3.60))

    quality = analyzer.primary_window_data_quality(
        reference,
        base_samples(reference, late_gap),
        {"profile": "lateral_right"},
    )
    windows = {item["label"]: item for item in quality["windows"]}

    assert windows["translation_onset_2s"]["valid"]
    assert not windows["common_metric_active_phase_1"]["valid"]
    assert not quality["valid"]


def test_reversal_post_window_and_outbound_common_gaps_are_invalid():
    reference = forward_reverse_reference()
    post_gap = ~((reference["t"] >= 4.40) & (reference["t"] <= 4.70))
    outbound_gap = ~((reference["t"] >= 2.00) & (reference["t"] <= 2.30))
    dwell_gap = ~((reference["t"] >= 3.30) & (reference["t"] <= 3.60))

    post_quality = analyzer.primary_window_data_quality(
        reference,
        base_samples(reference, post_gap),
        {"profile": "forward_reverse"},
    )
    outbound_quality = analyzer.primary_window_data_quality(
        reference,
        base_samples(reference, outbound_gap),
        {"profile": "forward_reverse"},
    )
    dwell_quality = analyzer.primary_window_data_quality(
        reference,
        base_samples(reference, dwell_gap),
        {"profile": "forward_reverse"},
    )
    post_windows = {item["label"]: item for item in post_quality["windows"]}
    outbound_windows = {
        item["label"]: item for item in outbound_quality["windows"]
    }
    dwell_windows = {
        item["label"]: item for item in dwell_quality["windows"]
    }

    assert post_windows["post_reverse_2s"]["start_time_s"] == pytest.approx(4.0)
    assert post_windows["post_reverse_2s"]["end_time_s"] == pytest.approx(6.0)
    assert not post_windows["post_reverse_2s"]["valid"]
    assert not post_quality["valid"]
    assert outbound_windows["post_reverse_2s"]["valid"]
    assert not outbound_windows["common_metric_active_phase_1"]["valid"]
    assert not outbound_quality["valid"]
    assert dwell_windows["post_reverse_2s"]["valid"]
    assert dwell_windows["common_metric_active_phase_1"]["valid"]
    assert dwell_windows["common_metric_active_phase_2"]["valid"]
    assert not dwell_windows["reversal_event_window"]["valid"]
    assert not dwell_quality["valid"]


def test_translation_metrics_are_axis_specific_and_report_step_response():
    timestamps = np.linspace(0.0, 5.0, 251)
    actual_speed = 0.2 * (1.0 - np.exp(-timestamps / 0.25))
    reference_y = 0.2 * timestamps
    traces = {
        "t": timestamps,
        "ref_x": np.zeros_like(timestamps),
        "ref_y": reference_y,
        "actual_x": np.full_like(timestamps, 0.03),
        "actual_y": reference_y,
        "error_x": np.full_like(timestamps, 0.03),
        "error_y": np.zeros_like(timestamps),
        "moving": np.ones(timestamps.size, dtype=bool),
        "moving_t": timestamps,
        "tangent_x": np.zeros_like(timestamps),
        "tangent_y": np.ones_like(timestamps),
        "command_speed": np.full_like(timestamps, 0.2),
        "actual_tangent_speed": actual_speed,
        "along_error": np.full_like(timestamps, -0.05),
    }

    metrics, response = analyzer.translation_metrics(
        traces, {"profile": "straight_forward"}
    )

    assert metrics["command_axis"] == "y"
    assert metrics["cross_axis"] == "x"
    assert metrics["cross_axis_error_m"]["rmse"] == pytest.approx(0.03)
    assert metrics["speed_response"]["rise_time_s"] == pytest.approx(
        0.58, abs=0.12
    )
    assert metrics["speed_response"]["steady_state_speed_error_m_s"][
        "mean"
    ] == pytest.approx(0.0, abs=1e-4)
    assert response["actual_speed_smoothed"].shape == timestamps.shape
    assert metrics["onset_response"][
        "along_track_abs_integral_m_s"
    ] == pytest.approx(0.10, abs=0.002)


def test_reversal_metrics_exclude_dwell_and_measure_delay_and_overshoot():
    timestamps = np.arange(0.0, 12.001, 0.02)
    moving = ((timestamps >= 1.0) & (timestamps < 5.0)) | (
        (timestamps >= 6.0) & (timestamps < 10.0)
    )
    pre = (timestamps >= 1.0) & (timestamps < 5.0)
    post = (timestamps >= 6.0) & (timestamps < 10.0)
    reference_y = np.zeros_like(timestamps)
    reference_y[pre] = 0.2 * (timestamps[pre] - 1.0)
    reference_y[(timestamps >= 5.0) & (timestamps < 6.0)] = 0.8
    reference_y[post] = 0.8 - 0.2 * (timestamps[post] - 6.0)
    actual_vy = np.zeros_like(timestamps)
    actual_vy[pre] = 0.2
    actual_vy[(timestamps >= 5.0) & (timestamps < 5.30)] = 0.2
    actual_vy[(timestamps >= 6.30) & (timestamps < 10.0)] = -0.2
    actual_y = np.cumsum(actual_vy) * 0.02
    tangent_y = np.r_[np.ones(np.count_nonzero(pre)), -np.ones(np.count_nonzero(post))]
    traces = {
        "t": timestamps,
        "ref_x": np.zeros_like(timestamps),
        "ref_y": reference_y,
        "actual_x": np.zeros_like(timestamps),
        "actual_y": actual_y,
        "actual_vx": np.zeros_like(timestamps),
        "actual_vy": actual_vy,
        "error_x": np.zeros_like(timestamps),
        "error_y": actual_y - reference_y,
        "moving": moving,
        "moving_t": timestamps[moving],
        "tangent_x": np.zeros(np.count_nonzero(moving)),
        "tangent_y": tangent_y,
        "command_speed": np.full(np.count_nonzero(moving), 0.2),
        "actual_tangent_speed": np.r_[
            actual_vy[pre], -actual_vy[post]
        ],
        "cross_error": np.zeros(np.count_nonzero(moving)),
    }

    metrics, _profile_traces = analyzer.reversal_metrics(
        traces, {"expected_dwell_duration_s": 1.0}
    )

    assert metrics["available"]
    assert metrics["commanded_dwell_s"] == pytest.approx(1.0)
    assert metrics["stop_delay_from_dwell_start_s"] == pytest.approx(
        0.30, abs=0.03
    )
    assert metrics["reversal_delay_s"] == pytest.approx(0.30, abs=0.03)
    assert metrics["turnaround_overshoot_m"] == pytest.approx(0.06, abs=0.01)
    assert metrics["dwell_excluded_from_speed_error"] is True
    assert metrics["primary_paper_metric"] == (
        "post_reverse_response.along_track_abs_integral_m_s"
    )
    assert metrics["post_reverse_response"][
        "along_track_abs_integral_m_s"
    ] > 0.0


def wheel_status_message(data, label="wheel_control_status_v1"):
    return SimpleNamespace(
        data=list(data),
        layout=SimpleNamespace(
            dim=[SimpleNamespace(label=label, size=17, stride=17)],
            data_offset=0,
        ),
    )


def test_actuator_diagnostics_separate_cap_saturation_and_rpm_error():
    timestamps = np.linspace(0.0, 1.0, 11)
    reference = {
        "t": timestamps,
        "vx": np.full_like(timestamps, 0.2),
        "vy": np.zeros_like(timestamps),
    }
    rows = {
        analyzer.WHEEL_STATUS_TOPIC: [],
        analyzer.LEFT_COMMAND_TOPIC: [],
        analyzer.RIGHT_COMMAND_TOPIC: [],
    }
    for index, timestamp in enumerate(timestamps):
        status = np.zeros(17)
        status[[0, 8]] = 10.0
        status[[1, 9]] = 9.0
        if index < 2:
            status[7] = 1.0
            status[16] = 7.0
        rows[analyzer.WHEEL_STATUS_TOPIC].append(
            (timestamp, wheel_status_message(status))
        )
        command = analyzer.DEFAULT_WHEEL_CAP_RAD_S if index < 5 else 1.0
        rows[analyzer.LEFT_COMMAND_TOPIC].append(
            (timestamp, SimpleNamespace(data=command))
        )
        rows[analyzer.RIGHT_COMMAND_TOPIC].append(
            (timestamp, SimpleNamespace(data=command))
        )

    metrics = analyzer.actuator_diagnostics(rows, reference)

    assert metrics["available"]
    assert metrics["active_pair_cap_fraction"] == pytest.approx(5.0 / 11.0)
    assert metrics["firmware_saturation_sample_count"] == 2
    assert metrics["firmware_saturation_fraction"] == pytest.approx(2.0 / 11.0)
    assert metrics["wheel_rpm_error"]["mean"] == pytest.approx(-1.0)
    assert metrics["wheel_rpm_error"]["rmse"] == pytest.approx(1.0)
    assert metrics["wheel_status_schema"] == "wheel_control_status_v1"
    assert metrics["wheel_status_source_value_counts"] == {"0": 9, "7": 2}
    assert metrics["communications_timeout_source_sample_count"] == 2
    assert metrics["communications_timeout_source_fraction"] == pytest.approx(
        2.0 / 11.0
    )


def test_actuator_cap_is_independent_and_unknown_status_layout_is_not_decoded():
    timestamps = np.linspace(0.0, 1.0, 11)
    reference = {
        "t": timestamps,
        "vx": np.full_like(timestamps, 0.2),
        "vy": np.zeros_like(timestamps),
    }
    rows = {
        analyzer.WHEEL_STATUS_TOPIC: [],
        analyzer.LEFT_COMMAND_TOPIC: [
            (timestamp, SimpleNamespace(data=analyzer.DEFAULT_WHEEL_CAP_RAD_S))
            for timestamp in timestamps
        ],
        analyzer.RIGHT_COMMAND_TOPIC: [
            (timestamp, SimpleNamespace(data=analyzer.DEFAULT_WHEEL_CAP_RAD_S))
            for timestamp in timestamps
        ],
    }

    missing_status = analyzer.actuator_diagnostics(rows, reference)
    rows[analyzer.WHEEL_STATUS_TOPIC] = [
        (
            timestamp,
            wheel_status_message(np.zeros(17), label="unknown_status_v9"),
        )
        for timestamp in timestamps
    ]
    unknown_status = analyzer.actuator_diagnostics(rows, reference)

    assert missing_status["command_pair_cap_available"]
    assert missing_status["active_pair_cap_fraction"] == pytest.approx(1.0)
    assert not missing_status["wheel_status_diagnostics_available"]
    assert missing_status["firmware_saturation_fraction"] is None
    assert unknown_status["command_pair_cap_available"]
    assert not unknown_status["wheel_status_diagnostics_available"]
    assert unknown_status["wheel_status_schema"] == ""
    assert unknown_status["wheel_rpm_error"] is None
    assert "not wheel_control_status_v1" in unknown_status[
        "wheel_status_unavailable_reason"
    ]


def test_noninteger_wheel_status_source_is_not_coerced_to_timeout():
    timestamps = np.linspace(0.0, 1.0, 11)
    reference = {
        "t": timestamps,
        "vx": np.full_like(timestamps, 0.2),
        "vy": np.zeros_like(timestamps),
    }
    status_rows = []
    for timestamp in timestamps:
        status = np.zeros(17)
        status[[0, 8]] = 10.0
        status[[1, 9]] = 9.0
        status[16] = 7.4
        status_rows.append((timestamp, wheel_status_message(status)))
    rows = {
        analyzer.WHEEL_STATUS_TOPIC: status_rows,
        analyzer.LEFT_COMMAND_TOPIC: [],
        analyzer.RIGHT_COMMAND_TOPIC: [],
    }

    metrics = analyzer.actuator_diagnostics(rows, reference)

    assert metrics["wheel_status_diagnostics_available"]
    assert metrics["wheel_rpm_error"]["rmse"] == pytest.approx(1.0)
    assert not metrics["wheel_status_source_diagnostics_available"]
    assert metrics["wheel_status_source_value_counts"] is None
    assert metrics["communications_timeout_source_sample_count"] is None
    assert "integer codes in 0..7" in metrics[
        "wheel_status_source_unavailable_reason"
    ]


def test_sparse_active_actuator_topics_do_not_report_zero_exposure():
    timestamps = np.arange(0.0, 2.001, 0.02)
    reference = {
        "t": timestamps,
        "vx": np.full_like(timestamps, 0.2),
        "vy": np.zeros_like(timestamps),
    }
    sparse_t = timestamps[:5]
    rows = {
        analyzer.LEFT_COMMAND_TOPIC: [
            (timestamp, SimpleNamespace(data=1.0))
            for timestamp in timestamps
        ],
        analyzer.RIGHT_COMMAND_TOPIC: [
            (timestamp, SimpleNamespace(data=1.0))
            for timestamp in sparse_t
        ],
        analyzer.WHEEL_STATUS_TOPIC: [
            (timestamp, wheel_status_message(np.zeros(17)))
            for timestamp in sparse_t
        ],
    }

    metrics = analyzer.actuator_diagnostics(rows, reference)

    command_support = metrics["command_pair_support"]
    status_support = metrics["wheel_status_support"]
    assert not command_support["valid"]
    assert command_support["coverage_fraction"] < 1.0
    assert command_support["max_effective_topic_gap_s"] > 0.10
    assert "coverage is" in command_support["failure_reason"]
    assert metrics["active_pair_cap_fraction"] is None
    assert metrics["active_pair_cap_sample_count"] is None
    assert not metrics["command_pair_cap_available"]
    assert not status_support["valid"]
    assert status_support["coverage_fraction"] < 1.0
    assert status_support["max_effective_topic_gap_s"] > 0.10
    assert metrics["observed_active_status_sample_count"] == 5
    assert metrics["firmware_saturation_fraction"] is None
    assert metrics["firmware_saturation_sample_count"] is None
    assert metrics["wheel_rpm_error"] is None
    assert metrics["wheel_status_source_value_counts"] is None
    assert metrics["communications_timeout_source_fraction"] is None


def test_forward_reverse_actuator_support_requires_midpoint_dwell():
    reference = forward_reverse_reference()
    moving = np.hypot(reference["vx"], reference["vy"]) > 0.02
    active_t = reference["t"][moving]
    rows = {
        analyzer.LEFT_COMMAND_TOPIC: [
            (timestamp, SimpleNamespace(data=1.0)) for timestamp in active_t
        ],
        analyzer.RIGHT_COMMAND_TOPIC: [
            (timestamp, SimpleNamespace(data=1.0)) for timestamp in active_t
        ],
        analyzer.WHEEL_STATUS_TOPIC: [
            (timestamp, wheel_status_message(np.zeros(17)))
            for timestamp in active_t
        ],
    }

    phase_only = analyzer.actuator_diagnostics(rows, reference)
    with_dwell = analyzer.actuator_diagnostics(
        rows, reference, {"profile": "forward_reverse"}
    )

    assert phase_only["command_pair_support"]["valid"]
    assert phase_only["wheel_status_support"]["valid"]
    assert phase_only["active_pair_cap_fraction"] == pytest.approx(0.0)
    assert phase_only["firmware_saturation_fraction"] == pytest.approx(0.0)
    assert not with_dwell["command_pair_support"]["valid"]
    assert not with_dwell["wheel_status_support"]["valid"]
    assert with_dwell["command_pair_support"]["coverage_fraction"] < 1.0
    assert with_dwell["wheel_status_support"]["coverage_fraction"] < 1.0
    assert with_dwell["active_pair_cap_fraction"] is None
    assert with_dwell["firmware_saturation_fraction"] is None
    required = analyzer._required_actuator_reference_mask(reference, True)
    assert required[np.argmin(np.abs(reference["t"] - 3.5))]


def test_actuator_plot_invocation_uses_manifest_wheel_limit(
    tmp_path, monkeypatch
):
    invocation = {}

    def capture(command, check):
        invocation["command"] = command
        invocation["check"] = check

    monkeypatch.setattr(analyzer.subprocess, "run", capture)
    counts = {topic: 1 for topic in analyzer.ACTUATOR_TOPICS}

    artifacts, warning = analyzer.run_actuator_analysis(
        tmp_path / "bag",
        tmp_path / "analysis",
        counts,
        {
            "wheel_speed_limit_rad_s": 3.386636,
            "profile": "forward_reverse",
        },
    )

    assert warning is None
    assert len(artifacts) == 2
    option_index = invocation["command"].index("--wheel-cap-rad-s")
    assert float(invocation["command"][option_index + 1]) == pytest.approx(
        3.386636
    )
    profile_index = invocation["command"].index("--profile")
    assert invocation["command"][profile_index + 1] == "forward_reverse"
    assert invocation["check"] is True


def test_imu_disturbance_proxy_requires_quality_for_high_frequency_metric():
    timestamps = np.arange(0.0, 2.0, 0.01)
    reference = {
        "t": timestamps,
        "vx": np.full_like(timestamps, 0.2),
        "vy": np.zeros_like(timestamps),
    }
    rows = []
    for timestamp in timestamps:
        rows.append(
            (
                timestamp,
                SimpleNamespace(
                    linear_acceleration=SimpleNamespace(
                        x=0.1 * np.sin(2.0 * np.pi * 20.0 * timestamp),
                        y=0.0,
                        z=9.81,
                    ),
                    angular_velocity=SimpleNamespace(x=0.0, y=0.0, z=0.02),
                ),
            )
        )

    metrics = analyzer.imu_disturbance_metrics(rows, reference)

    assert metrics["available"]
    assert metrics["high_frequency_available"]
    assert metrics["high_frequency_acceleration_residual_m_s2"][
        "rmse"
    ] > 0.05
    assert metrics["angular_rate_norm_rad_s"]["rmse"] == pytest.approx(0.02)
    assert not analyzer.imu_disturbance_metrics([], reference)["available"]


def test_matched_comparisons_use_repetitions_and_preserve_profile_hash():
    runs = []
    for repetition, first, second in ((1, 0.04, 0.05), (2, 0.05, 0.07), (3, 0.06, 0.09)):
        for caster, value in (
            ("traditional", first),
            ("split_spherical", second),
        ):
            row = {field: "" for field in analyzer.RUN_FIELDS}
            row.update(
                {
                    "run_id": f"{caster}-{repetition}",
                    "matched_block_id": f"G1-R{repetition}",
                    "valid_for_aggregate": "true",
                    "profile": "straight_forward",
                    "profile_sha256": "same-hash",
                    "speed_m_s": "0.15",
                    "caster_type": caster,
                    "terrain": "flat",
                    "initial_caster_orientation_deg": "0",
                    "repetition": str(repetition),
                    "xy_rmse_m": str(value),
                }
            )
            runs.append(row)
    different_revision = dict(runs[0])
    different_revision.update(
        {"run_id": "other-hash", "profile_sha256": "other", "xy_rmse_m": "9"}
    )
    runs.append(different_revision)

    comparisons = analyzer.build_matched_comparisons(
        runs, planned=[planned_trial()]
    )
    caster_xy = [
        item
        for item in comparisons
        if item["comparison_type"] == "caster"
        and item["metric"] == "xy_rmse_m"
        and item["profile_sha256_a"] == "same-hash"
    ]

    assert len(caster_xy) == 1
    comparison = caster_xy[0]
    assert comparison["paired_n"] == 3
    assert comparison["paired_difference_b_minus_a"] == pytest.approx(0.02)
    assert comparison["ci95_lower"] is not None
    assert comparison["ci95_upper"] is not None
    assert comparison["profile_sha256_b"] == "same-hash"


def test_planned_trial_coverage_counts_unrecorded_attempts():
    planned = []
    runs = []
    for repetition in (1, 2, 3):
        planned.append(planned_trial(repetition))
    for repetition, valid in ((1, True), (2, False)):
        runs.append(
            {
                **planned[repetition - 1],
                "valid_for_aggregate": str(valid).lower(),
            }
        )

    coverage = analyzer.planned_trial_coverage(planned, runs)

    assert coverage[0]["planned_trial_count"] == 3
    assert coverage[0]["recorded_trial_count"] == 2
    assert coverage[0]["valid_trial_count"] == 1



def test_profile_hash_and_formal_condition_checks_are_explicit():
    source = "schema_version: 1\nname: straight_forward\n"
    digest = __import__("hashlib").sha256(source.encode()).hexdigest()

    valid_hash = analyzer.profile_hash_validation(
        {"profile_yaml": source, "profile_sha256": digest}
    )
    invalid_hash = analyzer.profile_hash_validation(
        {"profile_yaml": source + "changed: true\n", "profile_sha256": digest}
    )
    valid_condition = analyzer.study_condition_validation(
        {
            "captured_start_yaw_rad": 0.10,
            "nominal_start_base_yaw_rad": 0.0,
            "start_base_yaw_tolerance_rad": 0.15,
            "caster_type": "split_spherical",
            "terrain": "flat",
        }
    )
    invalid_condition = analyzer.study_condition_validation(
        {
            "captured_start_yaw_rad": 0.20,
            "caster_type": "unspecified",
            "terrain": "uneven",
        }
    )

    assert valid_hash["valid"]
    assert not invalid_hash["valid"]
    assert valid_condition["captured_start_yaw_valid"]
    assert valid_condition["caster_type_valid"]
    assert valid_condition["terrain_rule_valid"]
    assert not invalid_condition["captured_start_yaw_valid"]
    assert not invalid_condition["caster_type_valid"]
    assert not invalid_condition["terrain_rule_valid"]


def test_quaternion_tilt_and_pose_envelope_are_formal_validity_checks():
    half = 0.40 / 2.0
    quaternion = SimpleNamespace(
        x=np.sin(half), y=0.0, z=0.0, w=np.cos(half)
    )
    assert analyzer.quaternion_tilt(quaternion) == pytest.approx(0.40)
    good = {
        "nonfinite_sample_count": 0,
        "xy_jump_over_80mm_count": 0,
        "yaw_jump_over_0_35rad_count": 0,
        "z_outside_0_25_to_0_40m_count": 0,
        "tilt_over_0_35rad_count": 0,
    }
    assert analyzer.pose_quality_valid_for_aggregate(good)
    assert not analyzer.pose_quality_valid_for_aggregate(
        {**good, "z_outside_0_25_to_0_40m_count": 1}
    )
    assert not analyzer.pose_quality_valid_for_aggregate(
        {**good, "tilt_over_0_35rad_count": 1}
    )


def odom_message(source_time_s):
    seconds = int(np.floor(source_time_s))
    nanoseconds = int(round((source_time_s - seconds) * 1e9))
    if nanoseconds == 1_000_000_000:
        seconds += 1
        nanoseconds = 0
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)
        ),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.31),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
    )


def test_vicon_source_stamps_are_extracted_and_quality_checked():
    receipt_t = np.asarray([100.00, 100.02, 100.04, 100.06])
    source_t = np.asarray([5.00, 5.02, 5.04, 5.06])
    rows = [
        (receipt, odom_message(source))
        for receipt, source in zip(receipt_t, source_t)
    ]

    base = analyzer.odom_arrays(rows)
    quality = analyzer.vicon_source_stamp_quality(base)

    assert np.allclose(base["t"], receipt_t)
    assert np.allclose(base["source_t"], source_t)
    assert quality["valid"]
    assert quality["max_gap_s"] == pytest.approx(0.02)
    assert not quality["absolute_source_receipt_lag_checked"]
    assert "cannot detect a replay" in quality["interpretation"]
    assert analyzer.ANALYSIS_REVISION == "hamr-study-analysis-v4"


def test_source_clock_mapping_rejects_delivery_burst_from_primary_tracking():
    relative_t = np.arange(0.0, 4.001, 0.02)
    reference_t = 100.005 + relative_t
    source_t = 5.0 + relative_t
    receipt_t = reference_t.copy()
    burst = (relative_t >= 1.60) & (relative_t < 2.00)
    receipt_t[burst] = 102.001 + 0.0002 * np.arange(
        np.count_nonzero(burst)
    )
    reference = {
        "t": reference_t,
        "x": 0.2 * relative_t,
        "y": np.zeros_like(relative_t),
        "yaw": np.zeros_like(relative_t),
        "vx": np.full_like(relative_t, 0.2),
        "vy": np.zeros_like(relative_t),
    }
    base = {
        "t": receipt_t,
        "source_t": source_t,
        "x": reference["x"].copy(),
        "y": reference["y"].copy(),
        "z": np.full_like(relative_t, 0.31),
        "yaw": np.zeros_like(relative_t),
    }

    metrics, traces = analyzer.tracking_metrics(reference, base, None)

    assert metrics["pose_alignment_time_basis"] == (
        "vicon_source_stamp_mapped_to_bag_time"
    )
    assert metrics["velocity_time_basis"] == "vicon_source_stamp"
    assert metrics["source_stamp_clock_mapping"][
        "source_to_bag_offset_s"
    ] == pytest.approx(95.005)
    assert metrics["source_stamp_clock_mapping"][
        "receipt_minus_mapped_source_s"
    ]["abs_max"] > 0.30
    assert metrics["source_stamp_clock_mapping"][
        "receipt_residual_over_50ms_count"
    ] > 0
    assert metrics["xy_error_m"]["abs_max"] == pytest.approx(0.0, abs=1e-12)
    assert metrics["receipt_time_alignment_diagnostic"]["xy_error_m"][
        "abs_max"
    ] > 0.03
    after_filter_fill = traces["moving_t"] >= reference_t[0] + 0.25
    assert np.max(
        np.abs(
            traces["primary_actual_tangent_speed"][after_filter_fill] - 0.2
        )
    ) < 1e-10
    assert np.max(traces["receipt_actual_tangent_speed"]) > 0.30
    assert np.nanmax(np.abs(traces["source_receipt_residual"])) > 0.30


def test_tracking_falls_back_cleanly_when_source_stamps_are_missing():
    reference = complete_reference()
    base = {
        "t": reference["t"],
        "x": reference["x"],
        "y": reference["y"],
        "z": np.full_like(reference["t"], 0.31),
        "yaw": np.zeros_like(reference["t"]),
    }

    metrics, traces = analyzer.tracking_metrics(reference, base, None)

    assert metrics["pose_alignment_time_basis"] == "bag_receipt_time_fallback"
    assert metrics["velocity_time_basis"] == "bag_receipt_time_fallback"
    assert not metrics["source_stamp_clock_mapping"]["available"]
    assert np.allclose(traces["actual_x"], traces["receipt_actual_x"])
    assert np.allclose(
        traces["actual_tangent_speed"],
        traces["receipt_actual_tangent_speed"],
    )


@pytest.mark.parametrize("fault", ("duplicate", "reversed", "nonfinite"))
def test_tracking_labels_invalid_source_stamp_fallback(fault):
    reference = complete_reference()
    source_t = 5.0 + reference["t"]
    source_t = source_t.copy()
    if fault == "duplicate":
        source_t[100] = source_t[99]
    elif fault == "reversed":
        source_t[100] = source_t[99] - 0.01
    else:
        source_t[100] = np.nan
    base = {
        "t": reference["t"],
        "source_t": source_t,
        "x": reference["x"],
        "y": reference["y"],
        "z": np.full_like(reference["t"], 0.31),
        "yaw": np.zeros_like(reference["t"]),
    }

    metrics, _traces = analyzer.tracking_metrics(reference, base, None)

    assert metrics["pose_alignment_time_basis"] == "bag_receipt_time_fallback"
    assert not metrics["source_stamp_clock_mapping"]["available"]


def test_causal_trailing_chord_handles_irregular_source_intervals():
    timestamps = np.asarray([0.0, 0.03, 0.08, 0.21, 0.27, 0.45, 0.50])
    position_x = 0.2 * timestamps
    position_y = -0.1 * timestamps

    velocity_x, velocity_y = analyzer._causal_trailing_chord_velocity(
        timestamps, position_x, position_y
    )

    assert np.allclose(velocity_x[1:], 0.2)
    assert np.allclose(velocity_y[1:], -0.1)


def test_causal_trailing_chord_does_not_use_future_positions():
    timestamps = np.arange(0.0, 1.001, 0.02)
    position_x = 0.2 * timestamps
    baseline_x, baseline_y = analyzer._causal_trailing_chord_velocity(
        timestamps, position_x, np.zeros_like(timestamps)
    )
    changed_x = position_x.copy()
    changed_x[timestamps >= 0.60] += 10.0

    revised_x, revised_y = analyzer._causal_trailing_chord_velocity(
        timestamps, changed_x, np.zeros_like(timestamps)
    )

    before_change = timestamps < 0.60
    assert np.array_equal(revised_x[before_change], baseline_x[before_change])
    assert np.array_equal(revised_y[before_change], baseline_y[before_change])


@pytest.mark.parametrize(
    ("source_t", "reason_fragment"),
    (
        ([5.0, 5.0, 5.0], "duplicate/frozen"),
        ([5.0, 5.02, 4.90], "reversed"),
        ([5.0, 5.02, 5.25], "gaps exceed"),
        ([0.0, 0.02, 0.04], "zero or negative"),
    ),
)
def test_vicon_source_stamp_freeze_reverse_gap_and_zero_are_invalid(
    source_t, reason_fragment
):
    quality = analyzer.vicon_source_stamp_quality(
        {
            "t": np.arange(len(source_t), dtype=float) * 0.02 + 100.0,
            "source_t": np.asarray(source_t, dtype=float),
        }
    )

    assert not quality["valid"]
    assert reason_fragment in quality["failure_reason"]


def test_missing_vicon_source_stamps_are_invalid():
    quality = analyzer.vicon_source_stamp_quality(
        {"t": np.asarray([1.0, 1.02]), "source_t": np.asarray([])}
    )

    assert not quality["valid"]
    assert "unavailable" in quality["failure_reason"]


def test_duplicate_valid_repetition_is_flagged_and_not_averaged():
    rows = []
    for caster, repetition, value, suffix in (
        ("traditional", "1", "0.10", "a"),
        ("split_spherical", "1", "0.20", "a"),
        ("split_spherical", "1", "9.00", "duplicate"),
        ("traditional", "2", "0.11", "a"),
        ("split_spherical", "2", "0.21", "a"),
    ):
        row = {field: "" for field in analyzer.RUN_FIELDS}
        row.update(
            {
                "run_id": f"{caster}-{repetition}-{suffix}",
                "matched_block_id": f"G1-R{repetition}",
                "valid_for_aggregate": "true",
                "profile": "straight_forward",
                "profile_sha256": "hash",
                "analysis_revision": "revision",
                "analyzer_sha256": "analyzer",
                "speed_m_s": "0.15",
                "caster_type": caster,
                "terrain": "flat",
                "initial_caster_orientation_deg": "0",
                "repetition": repetition,
                "xy_rmse_m": value,
            }
        )
        rows.append(row)

    comparison = next(
        item
        for item in analyzer.build_matched_comparisons(
            rows, planned=[planned_trial()]
        )
        if item["comparison_type"] == "caster"
        and item["metric"] == "xy_rmse_m"
    )

    assert comparison["paired_n"] == 1
    assert comparison["paired_difference_b_minus_a"] == pytest.approx(0.10)
    assert comparison["excluded_ambiguous_block_count"] == 1
    assert comparison["excluded_ambiguous_block_ids"] == "G1-R1"
    assert comparison["signed_definition"] == (
        "spherical minus traditional/trailing caster"
    )


def test_directional_asymmetry_has_declared_left_minus_right_sign():
    rows = []
    for profile, value in (("lateral_right", 0.02), ("lateral_left", 0.05)):
        row = {field: "" for field in analyzer.RUN_FIELDS}
        row.update(
            {
                "run_id": profile,
                "matched_block_id": "G1-R1",
                "valid_for_aggregate": "true",
                "profile": profile,
                "profile_sha256": profile + "-hash",
                "speed_m_s": "0.15",
                "caster_type": "traditional",
                "terrain": "flat",
                "initial_caster_orientation_deg": "0",
                "repetition": "1",
                "xy_rmse_m": str(value),
            }
        )
        rows.append(row)

    comparison = next(
        item
        for item in analyzer.build_matched_comparisons(
            rows, planned=[planned_trial()]
        )
        if item["comparison_type"] == "direction"
        and item["metric"] == "xy_rmse_m"
    )

    assert comparison["signed_definition"] == (
        "lateral_left minus lateral_right"
    )
    assert comparison["paired_difference_b_minus_a"] == pytest.approx(0.03)


def test_tracking_to_response_uses_one_causal_filter_without_future_leakage():
    timestamps = np.arange(0.0, 7.001, 0.02)
    moving = (timestamps >= 1.0) & (timestamps < 5.0)
    reference_x = np.zeros_like(timestamps)
    reference_x[moving] = 0.2 * (timestamps[moving] - 1.0)
    reference_x[timestamps >= 5.0] = 0.8
    reference = {
        "t": timestamps,
        "x": reference_x,
        "y": np.zeros_like(timestamps),
        "yaw": np.zeros_like(timestamps),
        "vx": np.where(moving, 0.2, 0.0),
        "vy": np.zeros_like(timestamps),
    }
    actual_x = np.zeros_like(timestamps)
    delayed = (timestamps >= 1.5) & (timestamps < 5.5)
    actual_x[delayed] = 0.2 * (timestamps[delayed] - 1.5)
    actual_x[timestamps >= 5.5] = 0.8
    base = {
        "t": timestamps,
        "x": actual_x,
        "y": np.zeros_like(timestamps),
        "z": np.full_like(timestamps, 0.31),
        "yaw": np.zeros_like(timestamps),
    }

    tracking, traces = analyzer.tracking_metrics(reference, base, None)
    translation, _response = analyzer.translation_metrics(
        traces, {"profile": "straight_forward"}
    )

    observed = translation["speed_response"]["rise_time_s"]
    assert observed >= 0.50
    assert observed <= 0.75
    assert tracking["velocity_filter_future_leakage"] is False
    assert translation["speed_response"]["velocity_filter_applied_here"] is False


def test_explicit_plan_is_read_only_and_hashed_in_summary(tmp_path):
    plan = tmp_path / "gate2_plan.csv"
    trial = planned_trial(1, planned_id="gate2-1", profile_sha256="hash")
    analyzer.write_csv_rows(
        plan,
        tuple(trial),
        [trial],
    )
    original = plan.read_bytes()
    results = tmp_path / "results"
    row = {field: "" for field in analyzer.RUN_FIELDS}
    row.update(
        {
            "run_id": "run-plan-1",
            "planned_id": "gate2-1",
            "profile": "straight_forward",
            "profile_sha256": "hash",
            "analysis_revision": "revision",
            "analyzer_sha256": "analyzer",
            "study_stack_contract_version": "2.0",
            "controller_config_sha256": "controller-hash",
            "wheel_speed_limit_rad_s": "3.3866360",
            "turret_control_enabled": "false",
            "nominal_start_base_yaw_rad": "0",
            "start_base_yaw_tolerance_rad": "0.1500",
            "speed_m_s": "0.2",
            "caster_type": "traditional",
            "terrain": "flat",
            "initial_caster_orientation_deg": "0.0",
            "repetition": "1.0",
            "valid_for_aggregate": "true",
        }
    )

    analyzer.update_study_results(results, row, plan_path=plan)

    assert plan.read_bytes() == original
    summary = (results / "study_summary.md").read_text(encoding="utf-8")
    digest = __import__("hashlib").sha256(original).hexdigest()
    assert str(plan.resolve()) in summary
    assert digest in summary
    with (results / "planned_trial_coverage.csv").open(newline="") as stream:
        coverage = list(csv.DictReader(stream))
    assert coverage[0]["recorded_trial_count"] == "1"
    assert coverage[0]["valid_trial_count"] == "1"


def test_detached_formal_csv_rows_include_selected_plan_provenance(tmp_path):
    first = planned_trial(
        1,
        planned_id="traditional-1",
        planned_order=1,
        matched_block_id="G1-M1",
    )
    second = {
        **planned_trial(
            1,
            planned_id="spherical-1",
            planned_order=2,
            matched_block_id="G1-M1",
        ),
        "caster_type": "split_spherical",
        "hardware_block_id": "H-spherical-1",
    }
    plan = tmp_path / "gate1_plan.csv"
    analyzer.write_csv_rows(plan, tuple(first), [first, second])
    digest = __import__("hashlib").sha256(plan.read_bytes()).hexdigest()
    results = tmp_path / "results"

    for trial, value in ((first, "0.10"), (second, "0.20")):
        row = {field: "" for field in analyzer.RUN_FIELDS}
        row.update(
            {
                **trial,
                "run_id": f"run-{trial['planned_id']}",
                "analysis_revision": "revision",
                "analyzer_sha256": "analyzer",
                "valid_for_aggregate": "true",
                "xy_rmse_m": value,
            }
        )
        analyzer.update_study_results(results, row, plan_path=plan)

    for filename in (
        "aggregate_metrics.csv",
        "planned_run_classification.csv",
        "planned_trial_coverage.csv",
        "paper_matched_comparisons.csv",
        "paper_condition_statistics.csv",
    ):
        with (results / filename).open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        assert rows, filename
        assert {item["plan_basename"] for item in rows} == {plan.name}
        assert {item["plan_sha256"] for item in rows} == {digest}


def test_no_plan_rewrites_stale_plan_outputs_and_labels_results_exploratory(
    tmp_path, monkeypatch
):
    monkeypatch.delenv("HAMR_STUDY_PLAN", raising=False)
    trial = planned_trial(1, planned_id="planned-1")
    plan = tmp_path / "selected_plan.csv"
    analyzer.write_csv_rows(plan, tuple(trial), [trial])
    results = tmp_path / "results"
    planned_row = {field: "" for field in analyzer.RUN_FIELDS}
    planned_row.update(
        {
            **trial,
            "run_id": "planned-run",
            "analysis_revision": "revision",
            "analyzer_sha256": "analyzer",
            "valid_for_aggregate": "true",
            "xy_rmse_m": "0.10",
        }
    )
    analyzer.update_study_results(results, planned_row, plan_path=plan)
    assert analyzer.read_csv_rows(
        results / "planned_run_classification.csv"
    )
    assert analyzer.read_csv_rows(results / "planned_trial_coverage.csv")

    exploratory_row = {
        **planned_row,
        "run_id": "exploratory-run",
        "planned_id": "",
        "xy_rmse_m": "0.20",
    }
    artifacts = analyzer.update_study_results(results, exploratory_row)

    plan_outputs = (
        (
            results / "planned_run_classification.csv",
            analyzer.PLAN_CLASSIFICATION_FIELDS,
        ),
        (
            results / "planned_trial_coverage.csv",
            analyzer.PLAN_COVERAGE_FIELDS,
        ),
    )
    for path, expected_fields in plan_outputs:
        with path.open(newline="") as stream:
            reader = csv.DictReader(stream)
            assert tuple(reader.fieldnames or ()) == expected_fields
            assert list(reader) == []
        assert path in artifacts

    with (results / "aggregate_metrics.csv").open(newline="") as stream:
        aggregate = list(csv.DictReader(stream))
    assert aggregate
    assert {item["plan_basename"] for item in aggregate} == {""}
    assert {item["plan_sha256"] for item in aggregate} == {""}

    study_summary = (results / "study_summary.md").read_text(encoding="utf-8")
    paper_summary = (results / "paper_summary.md").read_text(encoding="utf-8")
    assert "valid exploratory recorded rows" in study_summary
    assert "formal plan-matched runs" not in study_summary
    assert "valid exploratory rows" in paper_summary
    assert "match the selected plan/factors" not in paper_summary
    assert "Blank plan_basename and plan_sha256" in paper_summary
    assert "triangle profiles use mean corner error" in paper_summary
    assert "circle_cw and circle_ccw use radial RMSE" in paper_summary


def test_malformed_selected_plan_does_not_mutate_existing_runs_csv(tmp_path):
    results = tmp_path / "results"
    existing = {field: "" for field in analyzer.RUN_FIELDS}
    existing.update({"run_id": "existing", "valid_for_aggregate": "true"})
    runs_path = results / "runs.csv"
    analyzer.write_csv_rows(runs_path, analyzer.RUN_FIELDS, [existing])
    original = runs_path.read_bytes()

    malformed = planned_trial(1, planned_id="bad-plan")
    malformed.pop("hardware_block_id")
    plan = tmp_path / "malformed.csv"
    analyzer.write_csv_rows(plan, tuple(malformed), [malformed])
    candidate = {**existing, "run_id": "must-not-be-committed"}

    with pytest.raises(RuntimeError, match="hardware_block_id"):
        analyzer.update_study_results(results, candidate, plan_path=plan)

    assert runs_path.read_bytes() == original


def test_analysis_revisions_are_not_pooled(tmp_path):
    base = {field: "" for field in analyzer.RUN_FIELDS}
    base.update(
        {
            "profile": "straight_forward",
            "profile_sha256": "same-profile",
            "speed_m_s": "0.15",
            "caster_type": "traditional",
            "terrain": "flat",
            "initial_caster_orientation_deg": "0",
            "valid_for_aggregate": "true",
            "xy_rmse_m": "0.05",
        }
    )
    analyzer.update_study_results(
        tmp_path,
        {
            **base,
            "run_id": "old",
            "analysis_revision": "v1",
            "analyzer_sha256": "aaa",
        },
    )
    analyzer.update_study_results(
        tmp_path,
        {
            **base,
            "run_id": "new",
            "analysis_revision": "v2",
            "analyzer_sha256": "bbb",
        },
    )

    with (tmp_path / "aggregate_metrics.csv").open(newline="") as stream:
        aggregate = list(csv.DictReader(stream))
    assert len(aggregate) == 2
    assert {item["analysis_revision"] for item in aggregate} == {"v1", "v2"}


def test_plan_contract_mismatch_is_not_formal_and_numeric_versions_match():
    trial = planned_trial(1, planned_id="p1")
    matching = {
        **trial,
        "run_id": "matching",
        "study_stack_contract_version": "2.0",
        "valid_for_aggregate": "true",
    }
    mismatch = {
        **trial,
        "run_id": "mismatch",
        "study_stack_contract_version": "3",
        "valid_for_aggregate": "true",
    }

    formal, classifications = analyzer.classify_runs_against_plan(
        [trial], [matching, mismatch]
    )

    assert [item["run_id"] for item in formal] == ["matching"]
    by_run = {item["run_id"]: item for item in classifications}
    assert by_run["matching"]["status"] == "planned_valid"
    assert by_run["mismatch"]["status"] == "plan_factor_mismatch"
    assert "study_stack_contract_version" in by_run["mismatch"]["reason"]


@pytest.mark.parametrize(
    "field",
    (
        "study_stack_contract_version",
        "controller_config_sha256",
        "wheel_speed_limit_rad_s",
        "turret_control_enabled",
        "nominal_start_base_yaw_rad",
        "start_base_yaw_tolerance_rad",
    ),
)
def test_missing_required_provenance_cannot_match_selected_plan(field):
    trial = planned_trial(1, planned_id="p1")
    run = {**trial, "run_id": "missing", "valid_for_aggregate": "true"}
    run.pop(field)

    formal, classifications = analyzer.classify_runs_against_plan(
        [trial], [run]
    )

    assert formal == []
    assert classifications[0]["status"] == "plan_factor_mismatch"
    assert field in classifications[0]["reason"]


def test_turret_provenance_serializes_only_explicit_boole():
    assert analyzer._serialize_explicit_bool(False) == "false"
    assert analyzer._serialize_explicit_bool(True) == "true"
    assert analyzer._serialize_explicit_bool(None) == ""
    assert analyzer._serialize_explicit_bool("false") == ""


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("hardware_block_id", ""),
        ("planned_order", ""),
        ("planned_order", "0"),
        ("planned_order", "1.0"),
    ),
)
def test_selected_plan_requires_hardware_block_and_positive_integer_order(
    field, value
):
    trial = planned_trial(1, planned_id="p1")
    trial[field] = value

    with pytest.raises(RuntimeError, match=field):
        analyzer.classify_runs_against_plan([trial], [])


def test_selected_plan_requires_unique_planned_order():
    first = planned_trial(1, planned_id="p1", planned_order=1)
    second = planned_trial(2, planned_id="p2", planned_order=1)

    with pytest.raises(RuntimeError, match="duplicate planned_order"):
        analyzer.classify_runs_against_plan([first, second], [])


def test_selected_plan_excludes_unplanned_pilot_from_formal_estimates(tmp_path):
    trial = planned_trial(1, planned_id="formal-1")
    plan = tmp_path / "plan.csv"
    analyzer.write_csv_rows(plan, tuple(trial), [trial])
    results = tmp_path / "results"

    common = {field: "" for field in analyzer.RUN_FIELDS}
    common.update(
        {
            **trial,
            "analysis_revision": "revision",
            "analyzer_sha256": "analyzer",
            "wheel_status_schema": "float64_multiarray_16",
            "valid_for_aggregate": "true",
        }
    )
    pilot = {
        **common,
        "run_id": "pilot",
        "planned_id": "pilot-not-in-plan",
        "xy_rmse_m": "9.0",
    }
    formal = {
        **common,
        "run_id": "formal",
        "planned_id": "formal-1",
        "xy_rmse_m": "0.1",
    }

    analyzer.update_study_results(results, pilot, plan_path=plan)
    analyzer.update_study_results(results, formal, plan_path=plan)

    with (results / "aggregate_metrics.csv").open(newline="") as stream:
        aggregate = list(csv.DictReader(stream))
    assert len(aggregate) == 1
    assert float(aggregate[0]["xy_rmse_mean_m"]) == pytest.approx(0.1)
    with (results / "planned_run_classification.csv").open(
        newline=""
    ) as stream:
        classification = {item["run_id"]: item for item in csv.DictReader(stream)}
    assert classification["pilot"]["status"] == "unplanned"
    assert classification["formal"]["status"] == "planned_valid"


def test_duplicate_valid_attempts_for_planned_id_are_all_excluded():
    trial = planned_trial(1, planned_id="p1")
    runs = [
        {
            **trial,
            "run_id": run_id,
            "valid_for_aggregate": "true",
        }
        for run_id in ("attempt-a", "attempt-b")
    ]

    formal, classifications = analyzer.classify_runs_against_plan([trial], runs)
    coverage = analyzer.planned_trial_coverage([trial], runs)

    assert all(item["valid_for_aggregate"] == "false" for item in formal)
    assert {
        item["status"] for item in classifications
    } == {"ambiguous_valid_duplicate"}
    assert all(
        item["valid_for_aggregate"] == "false" for item in classifications
    )
    assert coverage[0]["valid_trial_count"] == 0
    assert coverage[0]["ambiguous_valid_trial_count"] == 1


def _comparison_run(profile, repetition, **metrics):
    row = {field: "" for field in analyzer.RUN_FIELDS}
    row.update(
        {
            "run_id": f"{profile}-{repetition}",
            "matched_block_id": f"G1-R{repetition}",
            "valid_for_aggregate": "true",
            "profile": profile,
            "profile_sha256": f"{profile}-hash",
            "analysis_revision": "revision",
            "analyzer_sha256": "analyzer",
            "study_stack_contract_version": "2",
            "controller_config_sha256": "controller-hash",
            "wheel_speed_limit_rad_s": "3.386636",
            "turret_control_enabled": "false",
            "wheel_status_schema": "float64_multiarray_16",
            "speed_m_s": "0.20",
            "nominal_start_base_yaw_rad": "0",
            "start_base_yaw_tolerance_rad": "0.15",
            "caster_type": "traditional",
            "terrain": "flat",
            "initial_caster_orientation_deg": "0",
            "repetition": str(repetition),
            **{key: str(value) for key, value in metrics.items()},
        }
    )
    return row


def test_reorientation_penalties_use_same_two_second_integral_and_sign():
    rows = [
        _comparison_run(
            "straight_forward",
            1,
            translation_onset_along_abs_integral_m_s=0.10,
        ),
        _comparison_run(
            "lateral_left",
            1,
            translation_onset_along_abs_integral_m_s=0.18,
        ),
        _comparison_run(
            "forward_reverse",
            1,
            reversal_post_reverse_along_abs_integral_m_s=0.25,
        ),
    ]

    penalties = {
        item["profile_b"]: item
        for item in analyzer.build_matched_comparisons(
            rows, planned=[planned_trial()]
        )
        if item["comparison_type"] == "reorientation_penalty"
    }

    assert penalties["lateral_left"]["paired_difference_b_minus_a"] == pytest.approx(
        0.08
    )
    assert penalties["forward_reverse"][
        "paired_difference_b_minus_a"
    ] == pytest.approx(0.15)
    assert penalties["forward_reverse"]["metric_a"] == (
        "translation_onset_along_abs_integral_m_s"
    )
    assert penalties["forward_reverse"]["metric_b"] == (
        "reversal_post_reverse_along_abs_integral_m_s"
    )
    assert penalties["forward_reverse"]["metric_role"] == "primary"
    assert "positive means worse than straight" in penalties[
        "forward_reverse"
    ]["signed_definition"]


def test_reorientation_penalty_reports_missing_planned_baseline():
    trial = planned_trial(
        1,
        profile="lateral_left",
        planned_id="lateral-1",
    )
    run = _comparison_run(
        "lateral_left",
        1,
        translation_onset_along_abs_integral_m_s=0.18,
    )
    run["planned_id"] = "lateral-1"

    penalty = next(
        item
        for item in analyzer.build_matched_comparisons([run], planned=[trial])
        if item["comparison_type"] == "reorientation_penalty"
    )

    assert penalty["availability"] == "unavailable"
    assert penalty["paired_n"] == 0
    assert "plan contains no straight_forward baseline" in penalty[
        "unavailable_reason"
    ]


def test_speed_comparisons_require_a_shared_predeclared_matched_block():
    gate1 = _comparison_run("straight_forward", 1, xy_rmse_m=0.15)
    gate1.update({"speed_m_s": "0.15", "matched_block_id": "G1-R1"})
    low = _comparison_run("straight_forward", 1, xy_rmse_m=0.10)
    low.update({"speed_m_s": "0.10", "matched_block_id": "G3-R1"})
    high = _comparison_run("straight_forward", 1, xy_rmse_m=0.20)
    high.update({"speed_m_s": "0.20", "matched_block_id": "G3-R1"})
    rows = [gate1, low, high]

    speed_xy = [
        item
        for item in analyzer.build_matched_comparisons(
            rows, planned=[planned_trial()]
        )
        if item["comparison_type"] == "speed"
        and item["metric"] == "xy_rmse_m"
    ]

    assert len(speed_xy) == 1
    assert speed_xy[0]["level_a"] == "0.10"
    assert speed_xy[0]["level_b"] == "0.20"
    assert speed_xy[0]["matched_block_ids"] == "G3-R1"
    assert analyzer.build_matched_comparisons(rows, planned=None) == []


def test_condition_statistics_label_roles_and_report_small_n_descriptives():
    rows = [
        _comparison_run(
            "lateral_left",
            repetition,
            translation_onset_along_abs_integral_m_s=primary,
            active_time_weighted_xy_error_mean_m=common,
        )
        for repetition, primary, common in ((1, 0.1, 0.05), (2, 0.3, 0.07))
    ]

    statistics_rows = analyzer.build_condition_statistics(rows)
    by_metric = {item["metric"]: item for item in statistics_rows}

    primary = by_metric["translation_onset_along_abs_integral_m_s"]
    assert primary["metric_role"] == "primary"
    assert primary["n"] == 2
    assert primary["mean"] == pytest.approx(0.2)
    assert primary["sd"] == pytest.approx(np.sqrt(0.02))
    assert primary["median"] == pytest.approx(0.2)
    assert primary["minimum"] == pytest.approx(0.1)
    assert primary["maximum"] == pytest.approx(0.3)
    assert by_metric["active_time_weighted_xy_error_mean_m"][
        "metric_role"
    ] == "secondary"
    assert analyzer.paper_metric_role("straight_forward", "cross_track_rmse_m") == (
        "primary"
    )


def test_single_run_sample_sd_is_unavailable_in_condition_and_aggregate(
    tmp_path,
):
    row = _comparison_run(
        "straight_forward", 1, xy_rmse_m=0.10, cross_track_rmse_m=0.04
    )

    by_metric = {
        item["metric"]: item
        for item in analyzer.build_condition_statistics([row])
    }
    assert by_metric["cross_track_rmse_m"]["n"] == 1
    assert by_metric["cross_track_rmse_m"]["sd"] is None
    assert analyzer.numeric_std([row], "xy_rmse_m") is None

    analyzer.update_study_results(tmp_path, row)
    with (tmp_path / "aggregate_metrics.csv").open(newline="") as stream:
        aggregate = list(csv.DictReader(stream))
    assert aggregate[0]["xy_rmse_std_m"] == ""


def test_paper_summary_does_not_treat_missing_actuator_topics_as_zero(tmp_path):
    row = _comparison_run("straight_forward", 1, xy_rmse_m=0.10)

    analyzer.update_study_results(tmp_path, row)
    summary = (tmp_path / "paper_summary.md").read_text(encoding="utf-8")

    assert "pair-cap exposure is unavailable for all valid runs" in summary
    assert "saturation exposure is unavailable for all valid runs" in summary
    assert "0/1 valid runs" not in summary


def test_common_time_weighted_xy_metric_does_not_bridge_midpoint_dwell():
    timestamps = np.asarray([0.0, 0.02, 0.04, 0.06, 0.08, 0.50, 0.52, 0.54])
    error = np.asarray([1.0, 1.0, 1.0, 100.0, 100.0, 3.0, 3.0, 3.0])
    active = np.asarray([True, True, True, False, False, True, True, True])

    result = analyzer._masked_time_weighted_mean(timestamps, error, active)

    assert result == pytest.approx(2.0)


def test_common_time_weighted_xy_metric_does_not_bridge_active_data_gap():
    timestamps = np.asarray([0.0, 0.02, 0.04, 0.30, 0.32, 0.34])
    error = np.asarray([1.0, 1.0, 1.0, 3.0, 3.0, 3.0])
    active = np.ones(timestamps.shape, dtype=bool)

    result = analyzer._masked_time_weighted_mean(timestamps, error, active)

    assert result == pytest.approx(2.0)
