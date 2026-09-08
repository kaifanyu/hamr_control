#!/usr/bin/env python3
"""Validate, analyze, and aggregate HAMR independent-study bags."""

from __future__ import annotations

import argparse
import csv
import hashlib
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
from typing import Any, Callable

import numpy as np
import yaml


DEFAULT_PATTERN = "hamr_study_*"
ANALYSIS_REVISION = "hamr-study-analysis-v4"
METADATA_TOPIC = "/hamr_test/trajectory_metadata"
REFERENCE_TOPIC = "/reference_trajectory"
BASE_TOPIC = "/HAMR_base/odom"
TURRET_TOPIC = "/HAMR_turret/odom"
IMU_TOPIC = "/imu/data"
WHEEL_STATUS_TOPIC = "/wheel_control/status"
LEFT_COMMAND_TOPIC = "/left_wheel/cmd_vel"
RIGHT_COMMAND_TOPIC = "/right_wheel/cmd_vel"
CORE_TOPICS = (METADATA_TOPIC, REFERENCE_TOPIC, BASE_TOPIC)
ACTUATOR_TOPICS = (
    REFERENCE_TOPIC,
    "/left_wheel/cmd_vel",
    "/right_wheel/cmd_vel",
    "/turret/cmd_vel",
    "/live_gains",
    "/wheel_control/status",
    "/left_wheel/encoder_ticks",
    "/right_wheel/encoder_ticks",
    "/turret/encoder_ticks",
)
MAX_POSE_MATCH_DT_S = 0.05
PRIMARY_WINDOW_REQUIRED_COVERAGE = 1.0
PRIMARY_WINDOW_MAX_EFFECTIVE_GAP_S = 0.10
ACTUATOR_SUPPORT_REQUIRED_COVERAGE = 1.0
ACTUATOR_SUPPORT_MAX_MATCH_DT_S = 0.05
ACTUATOR_SUPPORT_MAX_EFFECTIVE_GAP_S = 0.10
VICON_SOURCE_STAMP_MAX_GAP_S = 0.10
REFERENCE_SPEED_EPS = 1e-6
ACTIVE_TRANSLATION_SPEED_M_S = 0.02
DEFAULT_WHEEL_CAP_RPM = 28.0
DEFAULT_WHEEL_CAP_RAD_S = DEFAULT_WHEEL_CAP_RPM * 2.0 * math.pi / 60.0
RESPONSE_SUSTAIN_S = 0.20
VELOCITY_SMOOTHING_S = 0.20
TRANSLATION_ONSET_WINDOW_S = 2.0
REVERSAL_EVENT_PRE_S = 0.50
REVERSAL_EVENT_POST_S = 1.50
REVERSAL_POST_WINDOW_S = 2.0
TRANSLATION_PROFILES = frozenset(
    (
        "straight_forward",
        "straight_backward",
        "lateral_left",
        "lateral_right",
    )
)
REVERSAL_PROFILES = frozenset(("forward_reverse",))

RUN_FIELDS = (
    "run_id",
    "planned_id",
    "planned_order",
    "gate_added",
    "matched_block_id",
    "hardware_block_id",
    "bag_path",
    "analyzed_at_utc",
    "status",
    "valid_for_aggregate",
    "profile",
    "profile_sha256",
    "analysis_revision",
    "analyzer_sha256",
    "study_stack_contract_version",
    "controller_config_sha256",
    "wheel_speed_limit_rad_s",
    "turret_control_enabled",
    "wheel_status_schema",
    "kind",
    "speed_m_s",
    "origin_x_m",
    "origin_y_m",
    "captured_start_yaw_rad",
    "nominal_start_base_yaw_rad",
    "start_base_yaw_tolerance_rad",
    "caster_type",
    "terrain",
    "initial_caster_orientation_deg",
    "repetition",
    "video_id",
    "notes",
    "exclude_reason",
    "reference_duration_s",
    "reference_completion_fraction",
    "reference_rate_hz",
    "reference_max_gap_s",
    "final_zero_s",
    "vicon_coverage_fraction",
    "vicon_first_reference_match_dt_s",
    "vicon_last_reference_match_dt_s",
    "vicon_rate_hz",
    "vicon_max_gap_s",
    "vicon_gap_over_100ms_count",
    "vicon_source_stamp_valid",
    "vicon_source_stamp_sample_count",
    "vicon_source_stamp_nonpositive_count",
    "vicon_source_stamp_duplicate_count",
    "vicon_source_stamp_reversed_count",
    "vicon_source_stamp_max_gap_s",
    "vicon_source_stamp_failure_reason",
    "primary_window_quality_valid",
    "primary_window_min_coverage_fraction",
    "primary_window_max_effective_gap_s",
    "primary_window_failure_reason",
    "vicon_z_envelope_violation_count",
    "vicon_tilt_violation_count",
    "xy_rmse_m",
    "xy_p95_m",
    "xy_max_m",
    "active_time_weighted_xy_error_mean_m",
    "final_xy_error_m",
    "cross_track_rmse_m",
    "along_track_rmse_m",
    "startup_hold_xy_jitter_p95_m",
    "final_hold_xy_jitter_p95_m",
    "commanded_speed_mean_m_s",
    "actual_tangent_speed_mean_m_s",
    "speed_bias_m_s",
    "speed_rmse_m_s",
    "turret_yaw_rmse_rad",
    "turret_reference_difference_rmse_rad",
    "base_yaw_drift_rmse_rad",
    "base_yaw_net_change_rad",
    "translation_axis",
    "translation_cross_axis",
    "translation_cross_axis_bias_m",
    "translation_cross_axis_rmse_m",
    "translation_onset_along_rmse_m",
    "translation_onset_along_peak_m",
    "translation_onset_along_abs_integral_m_s",
    "speed_rise_time_s",
    "speed_settling_time_s",
    "steady_state_speed_bias_m_s",
    "speed_overshoot_m_s",
    "reversal_commanded_dwell_s",
    "reversal_stop_delay_s",
    "reversal_delay_s",
    "reversal_overshoot_m",
    "reversal_dwell_drift_m",
    "reversal_midpoint_error_m",
    "reversal_event_xy_rmse_m",
    "reversal_event_xy_peak_m",
    "reversal_event_xy_abs_integral_m_s",
    "reversal_event_along_rmse_m",
    "reversal_event_along_peak_m",
    "reversal_event_along_abs_integral_m_s",
    "reversal_post_reverse_along_abs_integral_m_s",
    "reversal_event_cross_rmse_m",
    "reversal_event_cross_peak_m",
    "reversal_event_cross_abs_integral_m_s",
    "reversal_pre_speed_bias_m_s",
    "reversal_post_speed_bias_m_s",
    "reversal_closure_error_m",
    "actuator_command_support_valid",
    "actuator_command_support_coverage_fraction",
    "actuator_command_support_max_effective_gap_s",
    "actuator_command_support_reason",
    "wheel_status_support_valid",
    "wheel_status_support_coverage_fraction",
    "wheel_status_support_max_effective_gap_s",
    "wheel_status_support_reason",
    "actuator_pair_cap_fraction",
    "firmware_saturation_fraction",
    "firmware_saturation_count",
    "wheel_rpm_error_bias",
    "wheel_rpm_error_rmse",
    "wheel_status_source_counts_json",
    "wheel_status_communications_timeout_count",
    "wheel_status_communications_timeout_fraction",
    "imu_linear_accel_norm_rms_m_s2",
    "imu_linear_accel_norm_p95_m_s2",
    "imu_high_frequency_accel_rms_m_s2",
    "imu_angular_rate_norm_rms_rad_s",
    "imu_angular_rate_norm_p95_rad_s",
    "triangle_corner_count",
    "triangle_corner_error_mean_m",
    "triangle_corner_error_max_m",
    "triangle_leg_cross_track_rmse_mean_m",
    "circle_radius_m",
    "circle_radial_bias_m",
    "circle_radial_rmse_m",
    "circle_radial_max_m",
    "circle_along_lag_mean_m",
    "circle_lap_completion_fraction",
    "warning_count",
)

AGGREGATE_FIELDS = (
    "plan_basename",
    "plan_sha256",
    "profile",
    "profile_sha256",
    "analysis_revision",
    "analyzer_sha256",
    "study_stack_contract_version",
    "controller_config_sha256",
    "wheel_speed_limit_rad_s",
    "turret_control_enabled",
    "speed_m_s",
    "nominal_start_base_yaw_rad",
    "start_base_yaw_tolerance_rad",
    "caster_type",
    "terrain",
    "initial_caster_orientation_deg",
    "run_count",
    "valid_run_count",
    "valid_rate",
    "xy_rmse_mean_m",
    "xy_rmse_std_m",
    "active_time_weighted_xy_error_mean_m",
    "cross_track_rmse_mean_m",
    "startup_hold_xy_jitter_p95_mean_m",
    "final_hold_xy_jitter_p95_mean_m",
    "speed_bias_mean_m_s",
    "final_xy_error_mean_m",
    "triangle_corner_error_mean_m",
    "circle_radial_rmse_mean_m",
    "base_yaw_drift_rmse_mean_rad",
    "translation_cross_axis_rmse_mean_m",
    "translation_onset_along_peak_mean_m",
    "translation_onset_along_abs_integral_mean_m_s",
    "speed_rise_time_mean_s",
    "speed_settling_time_mean_s",
    "steady_state_speed_bias_mean_m_s",
    "reversal_stop_delay_mean_s",
    "reversal_delay_mean_s",
    "reversal_overshoot_mean_m",
    "reversal_event_along_peak_mean_m",
    "reversal_event_along_abs_integral_mean_m_s",
    "reversal_post_reverse_along_abs_integral_mean_m_s",
    "reversal_closure_error_mean_m",
    "actuator_pair_cap_fraction_mean",
    "firmware_saturation_fraction_mean",
    "wheel_rpm_error_rmse_mean",
    "imu_high_frequency_accel_rms_mean_m_s2",
    "imu_angular_rate_norm_rms_mean_rad_s",
)

COMPARISON_FIELDS = (
    "comparison_type",
    "plan_basename",
    "plan_sha256",
    "analysis_revision",
    "analyzer_sha256",
    "study_stack_contract_version",
    "condition",
    "matched_block_ids",
    "profile_a",
    "profile_sha256_a",
    "level_a",
    "profile_b",
    "profile_sha256_b",
    "level_b",
    "metric",
    "metric_a",
    "metric_b",
    "metric_role",
    "unit",
    "availability",
    "unavailable_reason",
    "paired_n",
    "mean_a",
    "mean_b",
    "paired_difference_b_minus_a",
    "paired_difference_sd",
    "paired_difference_sem",
    "ci95_lower",
    "ci95_upper",
    "cohen_dz",
    "excluded_ambiguous_block_count",
    "excluded_ambiguous_block_ids",
    "signed_definition",
)

CONDITION_GROUP_FIELDS = (
    "profile",
    "profile_sha256",
    "analysis_revision",
    "analyzer_sha256",
    "study_stack_contract_version",
    "controller_config_sha256",
    "wheel_speed_limit_rad_s",
    "turret_control_enabled",
    "speed_m_s",
    "nominal_start_base_yaw_rad",
    "start_base_yaw_tolerance_rad",
    "caster_type",
    "terrain",
    "initial_caster_orientation_deg",
)

CONDITION_STAT_FIELDS = (
    "plan_basename",
    "plan_sha256",
    *CONDITION_GROUP_FIELDS,
    "metric",
    "metric_role",
    "unit",
    "n",
    "mean",
    "sd",
    "median",
    "minimum",
    "maximum",
)

PLAN_FACTOR_FIELDS = (
    "profile",
    "profile_sha256",
    "study_stack_contract_version",
    "controller_config_sha256",
    "wheel_speed_limit_rad_s",
    "turret_control_enabled",
    "nominal_start_base_yaw_rad",
    "start_base_yaw_tolerance_rad",
    "speed_m_s",
    "caster_type",
    "terrain",
    "initial_caster_orientation_deg",
    "repetition",
)

PLAN_CLASSIFICATION_FIELDS = (
    "plan_basename",
    "plan_sha256",
    "run_id",
    "planned_id",
    "status",
    "reason",
    "valid_for_aggregate",
)

PLAN_COVERAGE_FIELDS = (
    "plan_basename",
    "plan_sha256",
    "profile",
    "profile_sha256",
    "speed_m_s",
    "caster_type",
    "terrain",
    "initial_caster_orientation_deg",
    "planned_trial_count",
    "recorded_trial_count",
    "recorded_attempt_count",
    "valid_trial_count",
    "ambiguous_valid_trial_count",
    "factor_mismatch_attempt_count",
    "unplanned_run_count",
)

PAPER_METRICS = {
    "active_time_weighted_xy_error_mean_m": "m",
    "xy_rmse_m": "m",
    "cross_track_rmse_m": "m",
    "speed_bias_m_s": "m/s",
    "final_xy_error_m": "m",
    "base_yaw_drift_rmse_rad": "rad",
    "translation_cross_axis_rmse_m": "m",
    "translation_onset_along_peak_m": "m",
    "translation_onset_along_abs_integral_m_s": "m*s",
    "speed_rise_time_s": "s",
    "speed_settling_time_s": "s",
    "steady_state_speed_bias_m_s": "m/s",
    "reversal_stop_delay_s": "s",
    "reversal_delay_s": "s",
    "reversal_overshoot_m": "m",
    "reversal_event_along_peak_m": "m",
    "reversal_event_along_abs_integral_m_s": "m*s",
    "reversal_post_reverse_along_abs_integral_m_s": "m*s",
    "reversal_closure_error_m": "m",
    "actuator_pair_cap_fraction": "fraction",
    "firmware_saturation_fraction": "fraction",
    "wheel_rpm_error_rmse": "rpm",
    "imu_high_frequency_accel_rms_m_s2": "m/s^2",
    "imu_angular_rate_norm_rms_rad_s": "rad/s",
    "triangle_corner_error_mean_m": "m",
    "circle_radial_rmse_m": "m",
}

REORIENTATION_PENALTY_METRIC = (
    "reorientation_penalty_along_abs_integral_m_s"
)
PAPER_PRIMARY_METRICS = {
    "straight_forward": "cross_track_rmse_m",
    "straight_backward": "cross_track_rmse_m",
    "lateral_left": "translation_onset_along_abs_integral_m_s",
    "lateral_right": "translation_onset_along_abs_integral_m_s",
    "forward_reverse": "reversal_post_reverse_along_abs_integral_m_s",
    "triangle": "triangle_corner_error_mean_m",
    "circle": "circle_radial_rmse_m",
}



def analyzer_sha256() -> str:
    """Return the exact offline analyzer source identity."""
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()

def default_bag_root() -> Path:
    configured = os.environ.get("HAMR_BAG_ROOT")
    if configured:
        return Path(configured).expanduser()
    return Path.home() / "hamster_ws/src/hamr_control/rosbags"


def load_bag_metadata(bag: Path) -> dict[str, Any]:
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file():
        raise RuntimeError("metadata.yaml is missing; close the recorder first")
    document = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    metadata = document.get("rosbag2_bagfile_information")
    if not isinstance(metadata, dict):
        raise RuntimeError("metadata.yaml has no rosbag2_bagfile_information")
    return metadata


def storage_id(bag: Path) -> str:
    metadata = load_bag_metadata(bag)
    value = metadata.get("storage_identifier")
    if value:
        return str(value)
    return "mcap" if any(bag.glob("*.mcap")) else "sqlite3"


def topic_counts(metadata: dict[str, Any]) -> dict[str, int]:
    result: dict[str, int] = {}
    for item in metadata.get("topics_with_message_count", ()):
        name = item.get("topic_metadata", {}).get("name")
        if name:
            result[str(name)] = int(item.get("message_count", 0))
    return result


def metadata_preflight(bag: Path) -> tuple[bool, str, dict[str, int]]:
    try:
        metadata = load_bag_metadata(bag)
    except (OSError, RuntimeError, yaml.YAMLError) as exc:
        return False, str(exc), {}
    relative_files = metadata.get("relative_file_paths", ())
    if not relative_files:
        relative_files = [
            path.name for path in (*bag.glob("*.mcap"), *bag.glob("*.db3"))
        ]
    if not relative_files or any(
        not (bag / str(name)).is_file()
        or (bag / str(name)).stat().st_size <= 0
        for name in relative_files
    ):
        return False, "storage file is missing or empty", {}
    counts = topic_counts(metadata)
    missing = [topic for topic in CORE_TOPICS if counts.get(topic, 0) <= 0]
    if missing:
        return False, "missing recorded data on " + ", ".join(missing), counts
    return True, "closed bag and core topic counts are present", counts


def _read_rows(
    bag: Path, requested_topics: set[str]
) -> tuple[dict[str, list[tuple[float, Any]]], dict[str, str]]:
    # Keep ROS imports lazy so discovery, --help, and unit tests stay light.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id(bag)),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    selected = requested_topics & set(topic_types)
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(selected)))
    message_types = {
        topic: get_message(topic_types[topic]) for topic in selected
    }
    rows = {topic: [] for topic in requested_topics}
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic in selected:
            rows[topic].append(
                (
                    timestamp_ns * 1e-9,
                    deserialize_message(raw, message_types[topic]),
                )
            )
    return rows, topic_types


def parse_manifest_rows(rows: list[tuple[float, Any]]) -> dict[str, Any]:
    if not rows:
        raise RuntimeError(f"{METADATA_TOPIC} is empty")
    decoded = []
    for _timestamp, message in rows:
        try:
            value = json.loads(str(message.data))
        except (AttributeError, json.JSONDecodeError) as exc:
            raise RuntimeError("trajectory metadata is not valid JSON") from exc
        if not isinstance(value, dict):
            raise RuntimeError("trajectory metadata JSON must be an object")
        decoded.append(value)
    canonical = json.dumps(decoded[0], sort_keys=True)
    if any(json.dumps(item, sort_keys=True) != canonical for item in decoded[1:]):
        raise RuntimeError("trajectory metadata changed during one bag")
    return decoded[0]


def read_manifest_only(bag: Path) -> dict[str, Any]:
    rows, _types = _read_rows(bag, {METADATA_TOPIC})
    return parse_manifest_rows(rows[METADATA_TOPIC])


def select_latest_study_bag(
    root: Path,
    pattern: str = DEFAULT_PATTERN,
    manifest_reader: Callable[[Path], dict[str, Any]] = read_manifest_only,
) -> tuple[Path, list[str]]:
    rejected: list[str] = []
    candidates = sorted(
        (path for path in root.glob(pattern) if path.is_dir()),
        key=lambda path: (
            (path / "metadata.yaml").stat().st_mtime
            if (path / "metadata.yaml").exists()
            else path.stat().st_mtime
        ),
        reverse=True,
    )
    for bag in candidates:
        valid, reason, _counts = metadata_preflight(bag)
        if not valid:
            rejected.append(f"{bag.name}: {reason}")
            continue
        try:
            manifest_reader(bag)
        except Exception as exc:
            rejected.append(f"{bag.name}: metadata read failed: {exc}")
            continue
        return bag.resolve(), rejected
    suffix = "; skipped " + " | ".join(rejected) if rejected else ""
    raise RuntimeError(
        f"no closed study bag matching {pattern!r} in {root.resolve()}{suffix}"
    )


def _normalized_quaternion(quaternion: Any) -> tuple[float, ...] | None:
    values = tuple(
        float(value)
        for value in (
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
    )
    norm = math.sqrt(sum(value * value for value in values))
    if not all(math.isfinite(value) for value in values) or norm <= 1e-12:
        return None
    return tuple(value / norm for value in values)


def quaternion_yaw(quaternion: Any) -> float:
    normalized = _normalized_quaternion(quaternion)
    if normalized is None:
        return math.nan
    x, y, z, w = normalized
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def quaternion_tilt(quaternion: Any) -> float:
    """Return body-up tilt from world-up, matching the pose-guard geometry."""
    normalized = _normalized_quaternion(quaternion)
    if normalized is None:
        return math.nan
    x, y, _z, _w = normalized
    cosine = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
    return math.acos(cosine)



def profile_hash_validation(manifest: dict[str, Any]) -> dict[str, Any]:
    """Recompute immutable trajectory YAML identity before formal pooling."""
    declared = manifest.get("profile_sha256")
    source = manifest.get("profile_yaml")
    if not isinstance(declared, str) or not declared:
        return {
            "valid": False,
            "declared_sha256": declared,
            "computed_sha256": None,
            "reason": "manifest profile_sha256 is missing",
        }
    if not isinstance(source, str):
        return {
            "valid": False,
            "declared_sha256": declared,
            "computed_sha256": None,
            "reason": "manifest profile_yaml is missing or not text",
        }
    computed = hashlib.sha256(source.encode("utf-8")).hexdigest()
    return {
        "valid": computed == declared,
        "declared_sha256": declared,
        "computed_sha256": computed,
        "reason": "match" if computed == declared else "profile YAML hash mismatch",
    }


def study_condition_validation(manifest: dict[str, Any]) -> dict[str, Any]:
    """Validate predeclared fixed-axis, caster, and terrain study conditions."""
    captured_start_yaw = as_float(manifest.get("captured_start_yaw_rad"))
    nominal_declared = "nominal_start_base_yaw_rad" in manifest
    tolerance_declared = "start_base_yaw_tolerance_rad" in manifest
    nominal_start_yaw = as_float(
        manifest.get("nominal_start_base_yaw_rad", 0.0)
    )
    start_yaw_tolerance = as_float(
        manifest.get("start_base_yaw_tolerance_rad", 0.15)
    )
    protocol_metadata_explicit = bool(
        nominal_declared
        and tolerance_declared
        and nominal_start_yaw is not None
        and start_yaw_tolerance is not None
    )
    start_yaw_error = None
    if captured_start_yaw is not None and nominal_start_yaw is not None:
        start_yaw_error = math.atan2(
            math.sin(captured_start_yaw - nominal_start_yaw),
            math.cos(captured_start_yaw - nominal_start_yaw),
        )
    start_yaw_ok = bool(
        start_yaw_error is not None
        and start_yaw_tolerance is not None
        and 0.0 < start_yaw_tolerance <= math.pi
        and abs(start_yaw_error) <= start_yaw_tolerance
    )
    caster_label = str(manifest.get("caster_type", "")).strip().lower()
    caster_ok = bool(caster_label and caster_label != "unspecified")
    terrain_label = str(manifest.get("terrain", "")).strip().lower()
    terrain_rule_ok = terrain_label == "flat"
    return {
        "captured_start_yaw_rad": captured_start_yaw,
        "captured_start_yaw_nominal_rad": nominal_start_yaw,
        "captured_start_yaw_tolerance_rad": start_yaw_tolerance,
        "captured_start_yaw_error_rad": start_yaw_error,
        "captured_start_yaw_valid": start_yaw_ok,
        "start_yaw_protocol_metadata_explicit": protocol_metadata_explicit,
        "formal_start_yaw_valid": start_yaw_ok and protocol_metadata_explicit,
        "legacy_default_used": not protocol_metadata_explicit,
        "caster_type_valid": caster_ok,
        "terrain_rule": "flat_v2_z0.25..0.40m_tilt<=0.35rad",
        "terrain_rule_valid": terrain_rule_ok,
        "terrain_rule_limitation": (
            "non-flat trials require a new predeclared and validated "
            "terrain-specific pose-quality rule/analyzer revision"
        ),
    }

def reference_arrays(rows: list[tuple[float, Any]]) -> dict[str, np.ndarray]:
    return {
        "t": np.asarray([time_s for time_s, _message in rows], dtype=float),
        "x": np.asarray([message.x for _time, message in rows], dtype=float),
        "y": np.asarray([message.y for _time, message in rows], dtype=float),
        "yaw": np.asarray([message.yaw for _time, message in rows], dtype=float),
        "vx": np.asarray([message.x_dot for _time, message in rows], dtype=float),
        "vy": np.asarray([message.y_dot for _time, message in rows], dtype=float),
        "yaw_rate": np.asarray(
            [getattr(message, "yaw_dot", 0.0) for _time, message in rows],
            dtype=float,
        ),
    }


def _header_stamp_seconds(message: Any) -> float:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return math.nan
    seconds = getattr(stamp, "sec", getattr(stamp, "secs", None))
    nanoseconds = getattr(
        stamp, "nanosec", getattr(stamp, "nsecs", None)
    )
    try:
        seconds_value = float(seconds)
        nanoseconds_value = float(nanoseconds)
    except (TypeError, ValueError):
        return math.nan
    if (
        not math.isfinite(seconds_value)
        or not math.isfinite(nanoseconds_value)
        or nanoseconds_value < 0.0
        or nanoseconds_value >= 1e9
    ):
        return math.nan
    return seconds_value + nanoseconds_value * 1e-9


def odom_arrays(rows: list[tuple[float, Any]]) -> dict[str, np.ndarray]:
    return {
        "t": np.asarray([time_s for time_s, _message in rows], dtype=float),
        "source_t": np.asarray(
            [_header_stamp_seconds(message) for _time, message in rows],
            dtype=float,
        ),
        "x": np.asarray(
            [message.pose.pose.position.x for _time, message in rows],
            dtype=float,
        ),
        "y": np.asarray(
            [message.pose.pose.position.y for _time, message in rows],
            dtype=float,
        ),
        "z": np.asarray(
            [message.pose.pose.position.z for _time, message in rows],
            dtype=float,
        ),
        "yaw": np.asarray(
            [
                quaternion_yaw(message.pose.pose.orientation)
                for _time, message in rows
            ],
            dtype=float,
        ),
        "tilt": np.asarray(
            [
                quaternion_tilt(message.pose.pose.orientation)
                for _time, message in rows
            ],
            dtype=float,
        ),
    }


def vicon_source_stamp_quality(base: dict[str, np.ndarray]) -> dict[str, Any]:
    """Check source-clock freshness without comparing unsynchronized clocks."""
    receipt_t = np.asarray(base.get("t", np.asarray([])), dtype=float)
    source_t = np.asarray(base.get("source_t", np.asarray([])), dtype=float)
    reasons = []
    if source_t.size != receipt_t.size:
        reasons.append("Vicon source stamps are unavailable for some poses")
    sample_count = int(source_t.size)
    finite = np.isfinite(source_t)
    nonfinite_count = int(np.count_nonzero(~finite))
    nonpositive_count = int(np.count_nonzero(finite & (source_t <= 0.0)))
    if sample_count < 2:
        reasons.append("fewer than two Vicon source stamps are available")
    if nonfinite_count:
        reasons.append(
            f"{nonfinite_count} Vicon source stamps are missing or non-finite"
        )
    if nonpositive_count:
        reasons.append(
            f"{nonpositive_count} Vicon source stamps are zero or negative"
        )

    differences = np.diff(source_t)
    finite_differences = differences[np.isfinite(differences)]
    duplicate_count = int(np.count_nonzero(finite_differences == 0.0))
    reversed_count = int(np.count_nonzero(finite_differences < 0.0))
    positive_differences = finite_differences[finite_differences > 0.0]
    max_gap = (
        float(np.max(positive_differences))
        if positive_differences.size
        else None
    )
    gap_over_limit_count = int(
        np.count_nonzero(
            positive_differences > VICON_SOURCE_STAMP_MAX_GAP_S + 1e-9
        )
    )
    if duplicate_count:
        reasons.append(
            f"{duplicate_count} duplicate/frozen Vicon source-stamp steps"
        )
    if reversed_count:
        reasons.append(f"{reversed_count} reversed Vicon source-stamp steps")
    if gap_over_limit_count:
        reasons.append(
            f"{gap_over_limit_count} Vicon source-stamp gaps exceed 0.100 s"
        )
    return {
        "valid": not reasons,
        "sample_count": sample_count,
        "nonfinite_count": nonfinite_count,
        "nonpositive_count": nonpositive_count,
        "duplicate_count": duplicate_count,
        "reversed_count": reversed_count,
        "gap_over_100ms_count": gap_over_limit_count,
        "max_gap_s": max_gap,
        "failure_reasons": reasons,
        "failure_reason": "; ".join(reasons),
        "scope": "full reference lifecycle using base odometry poses",
        "offline_only": True,
        "absolute_source_receipt_lag_checked": False,
        "interpretation": (
            "Source stamps are required to be nonzero and strictly increasing "
            "with <=100 ms gaps. Analysis v4 also maps usable source stamps onto "
            "bag time with a robust constant offset for primary pose alignment "
            "and velocity estimation, while retaining receipt-time diagnostics. "
            "The offset includes unknown clock offset and typical delivery delay, "
            "so it cannot detect a replay whose source stamps advance normally "
            "or provide absolute latency."
        ),
    }



def _scalar_arrays(rows: list[tuple[float, Any]]) -> tuple[np.ndarray, np.ndarray]:
    return (
        np.asarray([time_s for time_s, _message in rows], dtype=float),
        np.asarray([float(message.data) for _time, message in rows], dtype=float),
    )


def _reference_active_at(
    timestamps: np.ndarray,
    reference: dict[str, np.ndarray],
    include_internal_dwell: bool = False,
) -> np.ndarray:
    if timestamps.size == 0 or reference["t"].size == 0:
        return np.zeros(timestamps.shape, dtype=bool)
    indices = np.searchsorted(reference["t"], timestamps, side="right") - 1
    inside = (indices >= 0) & (indices < reference["t"].size)
    indices = np.clip(indices, 0, reference["t"].size - 1)
    reference_speed = np.hypot(reference["vx"], reference["vy"])
    reference_active = reference_speed > ACTIVE_TRANSLATION_SPEED_M_S
    if include_internal_dwell and np.any(reference_active):
        active_indices = np.flatnonzero(reference_active)
        reference_active[active_indices[0] : active_indices[-1] + 1] = True
    return inside & reference_active[indices]


def _required_actuator_reference_mask(
    reference: dict[str, np.ndarray], include_internal_dwell: bool
) -> np.ndarray:
    """Return the frozen reference-clock denominator for actuator exposure."""
    timestamps = np.asarray(reference["t"], dtype=float)
    speed = np.hypot(reference["vx"], reference["vy"])
    required = np.isfinite(timestamps) & np.isfinite(speed) & (
        speed > ACTIVE_TRANSLATION_SPEED_M_S
    )
    if include_internal_dwell and np.any(required):
        indices = np.flatnonzero(required)
        required[indices[0] : indices[-1] + 1] = np.isfinite(
            timestamps[indices[0] : indices[-1] + 1]
        )
    return required


def _actuator_stream_support(
    reference: dict[str, np.ndarray],
    sample_t: np.ndarray,
    include_internal_dwell: bool,
    stream_label: str,
) -> dict[str, Any]:
    """Validate reference-clock coverage and topic gaps for one stream."""
    epsilon = 1e-9
    required = _required_actuator_reference_mask(
        reference, include_internal_dwell
    )
    phase_runs = _active_runs(required)
    finite_samples = np.asarray(sample_t, dtype=float)
    finite_samples = np.unique(finite_samples[np.isfinite(finite_samples)])
    windows = []
    for phase_index, (start, stop) in enumerate(phase_runs, start=1):
        query_t = np.asarray(reference["t"][start : stop + 1], dtype=float)
        if finite_samples.size:
            nearest = nearest_time_indices(query_t, finite_samples)
            match_dt = np.abs(query_t - finite_samples[nearest])
        else:
            nearest = np.zeros(query_t.shape, dtype=int)
            match_dt = np.full(query_t.shape, np.inf)
        matched = match_dt <= ACTUATOR_SUPPORT_MAX_MATCH_DT_S + epsilon
        expected_count = int(query_t.size)
        matched_count = int(np.count_nonzero(matched))
        coverage = matched_count / expected_count if expected_count else 0.0
        effective_gap = None
        if matched_count:
            observed_t = np.unique(finite_samples[nearest[matched]])
            observed_t = np.clip(observed_t, query_t[0], query_t[-1])
            support_t = np.unique(np.r_[query_t[0], observed_t, query_t[-1]])
            if support_t.size >= 2:
                effective_gap = float(np.max(np.diff(support_t)))
        reasons = []
        if expected_count < 2:
            reasons.append("fewer than two required reference samples")
        if coverage < ACTUATOR_SUPPORT_REQUIRED_COVERAGE - epsilon:
            reasons.append(
                f"coverage is {coverage:.3f}, below the required "
                f"{ACTUATOR_SUPPORT_REQUIRED_COVERAGE:.3f}"
            )
        if (
            effective_gap is None
            or effective_gap
            > ACTUATOR_SUPPORT_MAX_EFFECTIVE_GAP_S + epsilon
        ):
            gap_text = (
                "unavailable"
                if effective_gap is None
                else f"{effective_gap:.3f} s"
            )
            reasons.append(
                f"effective topic support gap is {gap_text}, above the "
                "0.100 s limit"
            )
        finite_match_dt = match_dt[np.isfinite(match_dt)]
        windows.append(
            {
                "label": f"active_phase_{phase_index}",
                "start_time_s": float(query_t[0]),
                "end_time_s": float(query_t[-1]),
                "required_reference_sample_count": expected_count,
                "matched_reference_sample_count": matched_count,
                "coverage_fraction": coverage,
                "max_reference_match_dt_s": (
                    float(np.max(finite_match_dt))
                    if finite_match_dt.size
                    else None
                ),
                "max_effective_topic_gap_s": effective_gap,
                "valid": not reasons,
                "failure_reasons": reasons,
            }
        )
    reasons = [
        f"{stream_label} {window['label']}: {reason}"
        for window in windows
        for reason in window["failure_reasons"]
    ]
    if not phase_runs:
        reasons.append(
            f"{stream_label}: reference has no required active-motion samples"
        )
    expected_total = sum(
        window["required_reference_sample_count"] for window in windows
    )
    matched_total = sum(
        window["matched_reference_sample_count"] for window in windows
    )
    gaps = [
        window["max_effective_topic_gap_s"]
        for window in windows
        if window["max_effective_topic_gap_s"] is not None
    ]
    return {
        "stream": stream_label,
        "valid": bool(windows) and all(window["valid"] for window in windows),
        "required_reference_sample_count": expected_total,
        "matched_reference_sample_count": matched_total,
        "coverage_fraction": (
            matched_total / expected_total if expected_total else 0.0
        ),
        "max_effective_topic_gap_s": max(gaps) if gaps else None,
        "source_observation_count": int(finite_samples.size),
        "include_internal_dwell": include_internal_dwell,
        "windows": windows,
        "failure_reasons": reasons,
        "failure_reason": "; ".join(reasons),
        "definition": (
            "Every required reference sample must match a finite topic sample "
            "within 50 ms, with no phasewise effective topic gap over 100 ms."
        ),
    }


def _combined_actuator_support(
    stream_support: list[tuple[str, dict[str, Any]]]
) -> dict[str, Any]:
    """Combine independently checked streams without hiding missing support."""
    supports = [support for _label, support in stream_support]
    reasons = [
        reason
        for _label, support in stream_support
        for reason in support["failure_reasons"]
    ]
    gaps = [support["max_effective_topic_gap_s"] for support in supports]
    return {
        "valid": bool(supports) and all(support["valid"] for support in supports),
        "required_reference_sample_count": (
            supports[0]["required_reference_sample_count"] if supports else 0
        ),
        "coverage_fraction": (
            min(support["coverage_fraction"] for support in supports)
            if supports
            else 0.0
        ),
        "max_effective_topic_gap_s": (
            max(float(gap) for gap in gaps)
            if gaps and all(gap is not None for gap in gaps)
            else None
        ),
        "streams": {
            label: support for label, support in stream_support
        },
        "failure_reasons": reasons,
        "failure_reason": "; ".join(reasons),
    }


def _sorted_unique_scalar_rows(
    rows: list[tuple[float, Any]],
) -> tuple[np.ndarray, np.ndarray, str]:
    if not rows:
        return np.asarray([]), np.asarray([]), "topic is absent"
    try:
        timestamps, values = _scalar_arrays(rows)
    except (AttributeError, TypeError, ValueError) as exc:
        return np.asarray([]), np.asarray([]), f"invalid scalar topic: {exc}"
    finite = np.isfinite(timestamps) & np.isfinite(values)
    timestamps, values = timestamps[finite], values[finite]
    if not timestamps.size:
        return timestamps, values, "topic has no finite samples"
    order = np.argsort(timestamps, kind="stable")
    timestamps, values = timestamps[order], values[order]
    keep = np.r_[True, np.diff(timestamps) > 0.0]
    return timestamps[keep], values[keep], ""


def _wheel_status_v1_layout(message: Any) -> tuple[bool, str]:
    layout = getattr(message, "layout", None)
    dimensions = getattr(layout, "dim", ()) if layout is not None else ()
    data_offset = getattr(layout, "data_offset", None)
    if len(dimensions) != 1:
        return False, "wheel status layout must have exactly one dimension"
    dimension = dimensions[0]
    if (
        getattr(dimension, "label", "") != "wheel_control_status_v1"
        or getattr(dimension, "size", None) != 17
        or getattr(dimension, "stride", None) != 17
        or data_offset != 0
    ):
        return (
            False,
            "wheel status layout is not wheel_control_status_v1 "
            "(size=17, stride=17, data_offset=0)",
        )
    if len(getattr(message, "data", ())) != 17:
        return False, "wheel_control_status_v1 data length is not 17"
    return True, ""


def actuator_diagnostics(
    rows: dict[str, list[tuple[float, Any]]],
    reference: dict[str, np.ndarray],
    manifest: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Separate controller/firmware limiting from physical tracking error."""
    include_dwell = str((manifest or {}).get("profile", "")) in REVERSAL_PROFILES
    declared_limit = as_float((manifest or {}).get("wheel_speed_limit_rad_s"))
    declared_limit_valid = bool(declared_limit is not None and declared_limit > 0.0)
    wheel_limit = (
        declared_limit if declared_limit_valid else DEFAULT_WHEEL_CAP_RAD_S
    )
    required_mask = _required_actuator_reference_mask(reference, include_dwell)
    required_t = np.asarray(reference["t"][required_mask], dtype=float)
    left_rows = rows.get(LEFT_COMMAND_TOPIC, [])
    right_rows = rows.get(RIGHT_COMMAND_TOPIC, [])
    left_t, left_command, left_parse_reason = _sorted_unique_scalar_rows(
        left_rows
    )
    right_t, right_command, right_parse_reason = _sorted_unique_scalar_rows(
        right_rows
    )
    left_support = _actuator_stream_support(
        reference, left_t, include_dwell, "left wheel command"
    )
    right_support = _actuator_stream_support(
        reference, right_t, include_dwell, "right wheel command"
    )
    for support, parse_reason in (
        (left_support, left_parse_reason),
        (right_support, right_parse_reason),
    ):
        if parse_reason:
            support["valid"] = False
            support["failure_reasons"].insert(
                0, f"{support['stream']}: {parse_reason}"
            )
            support["failure_reason"] = "; ".join(
                support["failure_reasons"]
            )
    command_support = _combined_actuator_support(
        (("left", left_support), ("right", right_support))
    )
    cap_count = None
    cap_fraction = None
    command_count = 0
    if command_support["valid"] and required_t.size:
        left_index = nearest_time_indices(required_t, left_t)
        right_index = nearest_time_indices(required_t, right_t)
        pair = np.column_stack(
            (left_command[left_index], right_command[right_index])
        )
        tolerance = max(1e-8, wheel_limit * 1e-8)
        capped = np.any(
            np.abs(pair) >= wheel_limit - tolerance,
            axis=1,
        )
        command_count = int(pair.shape[0])
        cap_count = int(np.count_nonzero(capped))
        cap_fraction = float(cap_count / command_count)
    cap_reason = command_support["failure_reason"]
    observed_left_count = int(
        np.count_nonzero(
            _reference_active_at(
                left_t, reference, include_internal_dwell=include_dwell
            )
        )
    )
    observed_right_count = int(
        np.count_nonzero(
            _reference_active_at(
                right_t, reference, include_internal_dwell=include_dwell
            )
        )
    )

    status_rows = rows.get(WHEEL_STATUS_TOPIC, [])
    status_available = False
    status_reason = "wheel status topic is absent"
    status_sample_count = 0
    status_field_count = None
    status_schema = ""
    saturation_count = None
    saturation_fraction = None
    rpm_error_summary = None
    source_diagnostics_available = False
    source_unavailable_reason = "wheel status v1 diagnostics are unavailable"
    source_counts = None
    communications_timeout_count = None
    communications_timeout_fraction = None
    status_support = _actuator_stream_support(
        reference, np.asarray([]), include_dwell, "wheel status"
    )
    observed_status_count = 0
    if not status_rows:
        status_support["failure_reasons"].insert(
            0, "wheel status: topic is absent"
        )
        status_support["failure_reason"] = "; ".join(
            status_support["failure_reasons"]
        )
    else:
        layout_results = [
            _wheel_status_v1_layout(message)
            for _time, message in status_rows
        ]
        layout_failures = [reason for valid, reason in layout_results if not valid]
        if layout_failures:
            status_reason = layout_failures[0]
            status_support["failure_reasons"].insert(0, status_reason)
            status_support["failure_reason"] = "; ".join(
                status_support["failure_reasons"]
            )
        else:
            status_schema = "wheel_control_status_v1"
            status_field_count = 17
            status_t = np.asarray(
                [time_s for time_s, _message in status_rows], dtype=float
            )
            try:
                status = np.asarray(
                    [
                        np.asarray(message.data, dtype=float)
                        for _time, message in status_rows
                    ]
                )
            except (TypeError, ValueError) as exc:
                status_reason = f"invalid wheel status: {exc}"
                status_support["failure_reasons"].insert(0, status_reason)
                status_support["failure_reason"] = "; ".join(
                    status_support["failure_reasons"]
                )
            else:
                finite_status = np.isfinite(status_t) & np.all(
                    np.isfinite(status[:, :16]), axis=1
                )
                order = np.argsort(status_t[finite_status], kind="stable")
                valid_status_t = status_t[finite_status][order]
                valid_status = status[finite_status][order]
                if valid_status_t.size:
                    keep = np.r_[True, np.diff(valid_status_t) > 0.0]
                    valid_status_t = valid_status_t[keep]
                    valid_status = valid_status[keep]
                observed_status_count = int(
                    np.count_nonzero(
                        _reference_active_at(
                            valid_status_t,
                            reference,
                            include_internal_dwell=include_dwell,
                        )
                    )
                )
                status_support = _actuator_stream_support(
                    reference,
                    valid_status_t,
                    include_dwell,
                    "wheel status",
                )
                status_reason = status_support["failure_reason"]
                if status_support["valid"]:
                    status_index = nearest_time_indices(
                        required_t, valid_status_t
                    )
                    active = valid_status[status_index]
                    status_sample_count = int(active.shape[0])
                    saturation = (active[:, 7] != 0.0) | (
                        active[:, 15] != 0.0
                    )
                    rpm_error = np.concatenate(
                        (
                            active[:, 1] - active[:, 0],
                            active[:, 9] - active[:, 8],
                        )
                    )
                    status_available = True
                    status_reason = ""
                    saturation_count = int(np.count_nonzero(saturation))
                    saturation_fraction = float(np.mean(saturation))
                    rpm_error_summary = _metric_summary(rpm_error)
                    raw_sources = active[:, 16]
                    rounded_sources = np.rint(raw_sources)
                    valid_sources = (
                        np.isfinite(raw_sources)
                        & (np.abs(raw_sources - rounded_sources) <= 1e-9)
                        & (rounded_sources >= 0.0)
                        & (rounded_sources <= 7.0)
                    )
                    if np.all(valid_sources):
                        source_values = rounded_sources.astype(int)
                        unique_sources, source_sample_counts = np.unique(
                            source_values, return_counts=True
                        )
                        source_counts = {
                            str(source): int(count)
                            for source, count in zip(
                                unique_sources, source_sample_counts
                            )
                        }
                        communications_timeout = source_values == 7
                        communications_timeout_count = int(
                            np.count_nonzero(communications_timeout)
                        )
                        communications_timeout_fraction = float(
                            np.mean(communications_timeout)
                        )
                        source_diagnostics_available = True
                        source_unavailable_reason = ""
                    else:
                        source_unavailable_reason = (
                            "wheel_control_status_v1 source values must be "
                            "finite integer codes in 0..7"
                        )

    return {
        "available": cap_fraction is not None or status_available,
        "command_pair_cap_available": cap_fraction is not None,
        "command_pair_cap_unavailable_reason": cap_reason,
        "command_pair_support": command_support,
        "wheel_status_diagnostics_available": status_available,
        "wheel_status_unavailable_reason": status_reason,
        "wheel_status_support": status_support,
        "active_status_sample_count": status_sample_count,
        "observed_active_status_sample_count": observed_status_count,
        "wheel_status_field_count": status_field_count,
        "wheel_status_schema": status_schema,
        "analysis_window": (
            "outbound-through-reverse including planned dwell"
            if include_dwell
            else "active reference translation only"
        ),
        "active_pair_cap_sample_count": cap_count,
        "active_pair_cap_fraction": cap_fraction,
        "active_command_sample_count": command_count,
        "observed_active_left_command_sample_count": observed_left_count,
        "observed_active_right_command_sample_count": observed_right_count,
        "wheel_speed_limit_rad_s": wheel_limit,
        "wheel_speed_limit_rpm": wheel_limit * 60.0 / (2.0 * math.pi),
        "wheel_speed_limit_source": (
            "manifest"
            if declared_limit_valid
            else "legacy_28rpm_fallback_nonformal"
        ),
        "firmware_saturation_sample_count": saturation_count,
        "firmware_saturation_fraction": saturation_fraction,
        "wheel_rpm_error_sign_convention": "measured_minus_target",
        "wheel_rpm_error": rpm_error_summary,
        "wheel_status_source_value_counts": source_counts,
        "wheel_status_source_diagnostics_available": (
            source_diagnostics_available
        ),
        "wheel_status_source_unavailable_reason": source_unavailable_reason,
        "communications_timeout_source_sample_count": (
            communications_timeout_count
        ),
        "communications_timeout_source_fraction": (
            communications_timeout_fraction
        ),
        "wheel_status_source_interpretation": (
            "Values come from wheel_control_status_v1 field 16 and counts use "
            "the required reference-clock denominator; source==7 indicates the "
            "firmware communications-timeout source state, not proof that every "
            "possible watchdog or interruption was captured."
        ),
        "interpretation": (
            "Exposure fractions use the required reference-clock denominator "
            "and are unavailable unless active-window support passes. Use cap "
            "and saturation exposure to distinguish actuator-limited runs from "
            "candidate caster/contact effects."
        ),
    }


def imu_arrays(rows: list[tuple[float, Any]]) -> dict[str, np.ndarray]:
    return {
        "t": np.asarray([time_s for time_s, _message in rows], dtype=float),
        "acceleration": np.asarray(
            [
                (
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                )
                for _time, message in rows
            ],
            dtype=float,
        ),
        "angular_rate": np.asarray(
            [
                (
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                )
                for _time, message in rows
            ],
            dtype=float,
        ),
    }


def imu_disturbance_metrics(
    rows: list[tuple[float, Any]],
    reference: dict[str, np.ndarray],
    manifest: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Return active-motion IMU proxies; never claim direct caster vibration."""
    if not rows:
        return {"available": False, "reason": "IMU topic is absent"}
    imu = imu_arrays(rows)
    finite = (
        np.isfinite(imu["t"])
        & np.all(np.isfinite(imu["acceleration"]), axis=1)
        & np.all(np.isfinite(imu["angular_rate"]), axis=1)
    )
    include_dwell = str((manifest or {}).get("profile", "")) in REVERSAL_PROFILES
    active = _reference_active_at(
        imu["t"], reference, include_internal_dwell=include_dwell
    )
    selected = finite & active
    timestamps = imu["t"][selected]
    if timestamps.size < 10:
        return {"available": False, "reason": "fewer than 10 active IMU samples"}
    acceleration = imu["acceleration"][selected]
    angular_rate = imu["angular_rate"][selected]
    quality = stream_quality(timestamps)
    acceleration_norm = np.linalg.norm(acceleration, axis=1)
    angular_rate_norm = np.linalg.norm(angular_rate, axis=1)
    high_frequency_available = bool(
        quality["duration_s"] >= 1.0
        and quality["mean_rate_hz"] >= 40.0
        and quality["max_gap_s"] is not None
        and quality["max_gap_s"] <= 0.05
    )
    high_frequency_summary = None
    if high_frequency_available:
        sample_period = 1.0 / quality["mean_rate_hz"]
        window_samples = max(3, int(round(0.25 / sample_period)))
        low_frequency = np.column_stack(
            [
                _centered_moving_average(acceleration[:, axis], window_samples)
                for axis in range(3)
            ]
        )
        residual_norm = np.linalg.norm(acceleration - low_frequency, axis=1)
        high_frequency_summary = _metric_summary(residual_norm)
    return {
        "available": True,
        "active_sample_count": int(timestamps.size),
        "analysis_window": (
            "outbound-through-reverse including planned dwell"
            if include_dwell
            else "active reference translation only"
        ),
        "stream_quality": quality,
        "linear_acceleration_norm_m_s2": _metric_summary(acceleration_norm),
        "angular_rate_norm_rad_s": _metric_summary(angular_rate_norm),
        "high_frequency_available": high_frequency_available,
        "high_frequency_acceleration_residual_m_s2": high_frequency_summary,
        "high_frequency_definition": (
            "vector residual after subtracting a centered 0.25 s moving "
            "average; requires >=40 Hz, <=50 ms max gap, and >=1 s"
        ),
        "interpretation": (
            "Chassis-disturbance proxies only; they do not prove caster "
            "vibration, slip, or contact transitions."
        ),
    }

def unique_time_series(
    timestamps: np.ndarray, *values: np.ndarray
) -> tuple[np.ndarray, ...]:
    if timestamps.size == 0:
        return (timestamps, *values)
    _unique, indices = np.unique(timestamps, return_index=True)
    indices.sort()
    return (timestamps[indices], *(value[indices] for value in values))


def nearest_time_distance(query: np.ndarray, source: np.ndarray) -> np.ndarray:
    indices = nearest_time_indices(query, source)
    if source.size == 0:
        return np.full(query.shape, np.inf)
    return np.abs(query - source[indices])


def nearest_time_indices(query: np.ndarray, source: np.ndarray) -> np.ndarray:
    """Return the nearest source index for each query timestamp."""
    if source.size == 0:
        return np.zeros(query.shape, dtype=int)
    positions = np.searchsorted(source, query)
    left = np.clip(positions - 1, 0, source.size - 1)
    right = np.clip(positions, 0, source.size - 1)
    choose_right = np.abs(query - source[right]) < np.abs(query - source[left])
    return np.where(choose_right, right, left)


def _reference_window_pose_quality(
    reference_t: np.ndarray,
    base_t: np.ndarray,
    start_t: float,
    end_t: float,
    label: str,
    metric: str,
) -> dict[str, Any]:
    """Check complete, contiguous Vicon support for one frozen window."""
    epsilon = 1e-9
    inside = (reference_t >= start_t - epsilon) & (
        reference_t <= end_t + epsilon
    )
    query_t = reference_t[inside]
    window_complete = bool(
        reference_t.size
        and reference_t[0] <= start_t + epsilon
        and reference_t[-1] >= end_t - epsilon
    )
    nearest = nearest_time_indices(query_t, base_t)
    match_dt = (
        np.abs(query_t - base_t[nearest])
        if base_t.size
        else np.full(query_t.shape, np.inf)
    )
    matched = match_dt <= MAX_POSE_MATCH_DT_S + epsilon
    expected_count = int(query_t.size)
    matched_count = int(np.count_nonzero(matched))
    coverage = matched_count / expected_count if expected_count else 0.0

    effective_gap = None
    if matched_count:
        observed_t = np.unique(base_t[nearest[matched]])
        observed_t = np.clip(observed_t, start_t, end_t)
        support_t = np.unique(np.r_[start_t, observed_t, end_t])
        if support_t.size >= 2:
            effective_gap = float(np.max(np.diff(support_t)))
    reasons = []
    if not window_complete:
        reasons.append("reference does not span the complete frozen window")
    if expected_count < 2:
        reasons.append("fewer than two reference samples occur in the window")
    if coverage < PRIMARY_WINDOW_REQUIRED_COVERAGE - epsilon:
        reasons.append(
            f"Vicon/reference coverage is {coverage:.3f}, below the required "
            f"{PRIMARY_WINDOW_REQUIRED_COVERAGE:.3f}"
        )
    if (
        effective_gap is None
        or effective_gap > PRIMARY_WINDOW_MAX_EFFECTIVE_GAP_S + epsilon
    ):
        gap_text = "unavailable" if effective_gap is None else f"{effective_gap:.3f} s"
        reasons.append(
            f"effective Vicon support gap is {gap_text}, above the 0.100 s limit"
        )
    finite_match_dt = match_dt[np.isfinite(match_dt)]
    return {
        "label": label,
        "metric": metric,
        "start_time_s": start_t,
        "end_time_s": end_t,
        "duration_s": end_t - start_t,
        "reference_sample_count": expected_count,
        "matched_reference_sample_count": matched_count,
        "coverage_fraction": coverage,
        "max_reference_match_dt_s": (
            float(np.max(finite_match_dt)) if finite_match_dt.size else None
        ),
        "max_effective_vicon_gap_s": effective_gap,
        "complete_reference_window": window_complete,
        "valid": not reasons,
        "failure_reasons": reasons,
    }


def primary_window_data_quality(
    reference: dict[str, np.ndarray],
    base: dict[str, np.ndarray],
    manifest: dict[str, Any],
) -> dict[str, Any]:
    """Apply offline-only Vicon quality rules to frozen primary windows."""
    finite_base = (
        np.isfinite(base["t"])
        & np.isfinite(base["x"])
        & np.isfinite(base["y"])
        & np.isfinite(base["z"])
        & np.isfinite(base["yaw"])
        & np.isfinite(base["tilt"])
    )
    base_t = np.unique(base["t"][finite_base])
    speed = np.hypot(reference["vx"], reference["vy"])
    active_runs = _active_runs(speed > ACTIVE_TRANSLATION_SPEED_M_S)
    profile = str(manifest.get("profile", ""))
    specifications: list[tuple[float, float, str, str]] = [
        (
            float(reference["t"][start]),
            float(reference["t"][stop]),
            f"common_metric_active_phase_{index}",
            "active_time_weighted_xy_error_mean_m",
        )
        for index, (start, stop) in enumerate(active_runs, start=1)
    ]
    if profile in TRANSLATION_PROFILES and active_runs:
        onset_t = float(reference["t"][active_runs[0][0]])
        specifications.append(
            (
                onset_t,
                onset_t + TRANSLATION_ONSET_WINDOW_S,
                "translation_onset_2s",
                "translation_onset_along_abs_integral_m_s",
            )
        )
    elif profile in REVERSAL_PROFILES and len(active_runs) >= 2:
        dwell_start_t = float(reference["t"][active_runs[0][1] + 1])
        reverse_t = float(reference["t"][active_runs[1][0]])
        specifications.append(
            (
                dwell_start_t - REVERSAL_EVENT_PRE_S,
                reverse_t + REVERSAL_EVENT_POST_S,
                "reversal_event_window",
                "reversal_event_along_abs_integral_m_s",
            )
        )
        specifications.append(
            (
                reverse_t,
                reverse_t + REVERSAL_POST_WINDOW_S,
                "post_reverse_2s",
                "reversal_post_reverse_along_abs_integral_m_s",
            )
        )
    windows = [
        _reference_window_pose_quality(
            reference["t"], base_t, start_t, end_t, label, metric
        )
        for start_t, end_t, label, metric in specifications
    ]
    reasons = [
        f"{window['label']}: {reason}"
        for window in windows
        for reason in window["failure_reasons"]
    ]
    if not specifications:
        reasons.append(
            "could not derive the required primary window from reference motion"
        )
    return {
        "valid": bool(windows) and all(window["valid"] for window in windows),
        "policy": {
            "offline_only": True,
            "reference_clock_anchored": True,
            "required_coverage_fraction": PRIMARY_WINDOW_REQUIRED_COVERAGE,
            "max_reference_match_dt_s": MAX_POSE_MATCH_DT_S,
            "max_effective_vicon_gap_s": PRIMARY_WINDOW_MAX_EFFECTIVE_GAP_S,
            "interpretation": (
                "Every reference sample in each required window must match a "
                "finite Vicon pose within 50 ms, with no effective observation "
                "gap over 100 ms. This never stops a live test."
            ),
        },
        "windows": windows,
        "failure_reasons": reasons,
    }


def stream_quality(timestamps: np.ndarray) -> dict[str, Any]:
    if timestamps.size < 2:
        return {
            "sample_count": int(timestamps.size),
            "duration_s": 0.0,
            "mean_rate_hz": 0.0,
            "max_gap_s": None,
            "gap_over_50ms_count": 0,
            "gap_over_100ms_count": 0,
            "gap_over_200ms_count": 0,
        }
    gaps = np.diff(timestamps)
    duration = float(timestamps[-1] - timestamps[0])
    return {
        "sample_count": int(timestamps.size),
        "duration_s": duration,
        "mean_rate_hz": float((timestamps.size - 1) / duration)
        if duration > 0.0
        else 0.0,
        "max_gap_s": float(np.max(gaps)),
        "gap_over_50ms_count": int(np.count_nonzero(gaps > 0.05)),
        "gap_over_100ms_count": int(np.count_nonzero(gaps > 0.10)),
        "gap_over_200ms_count": int(np.count_nonzero(gaps > 0.20)),
    }


def vicon_pose_quality(base: dict[str, np.ndarray]) -> dict[str, Any]:
    """Summarize pose corruption without imposing a live controller guard."""
    finite = (
        np.isfinite(base["x"])
        & np.isfinite(base["y"])
        & np.isfinite(base["z"])
        & np.isfinite(base["yaw"])
        & np.isfinite(base["tilt"])
    )
    x = base["x"][finite]
    y = base["y"][finite]
    yaw = np.unwrap(base["yaw"][finite])
    jumps = np.hypot(np.diff(x), np.diff(y)) if x.size >= 2 else np.asarray([])
    yaw_jumps = np.abs(np.diff(yaw)) if yaw.size >= 2 else np.asarray([])
    finite_z = base["z"][np.isfinite(base["z"])]
    finite_tilt = base["tilt"][np.isfinite(base["tilt"])]
    return {
        "nonfinite_sample_count": int(np.count_nonzero(~finite)),
        "xy_jump_over_80mm_count": int(np.count_nonzero(jumps > 0.08)),
        "max_xy_jump_m": float(np.max(jumps)) if jumps.size else None,
        "yaw_jump_over_0_35rad_count": int(
            np.count_nonzero(yaw_jumps > 0.35)
        ),
        "max_yaw_jump_rad": float(np.max(yaw_jumps))
        if yaw_jumps.size
        else None,
        "z_outside_0_25_to_0_40m_count": int(
            np.count_nonzero((finite_z < 0.25) | (finite_z > 0.40))
        ),
        "tilt_over_0_35rad_count": int(
            np.count_nonzero(finite_tilt > 0.35)
        ),
        "max_tilt_rad": float(np.max(finite_tilt))
        if finite_tilt.size
        else None,
        "tilt_checked": True,
        "tilt_definition": "body-up angle from Vicon world-up",
    }


def pose_quality_valid_for_aggregate(quality: dict[str, Any]) -> bool:
    """Apply the predefined Vicon geometry rules used for formal pooling."""
    return bool(
        quality.get("nonfinite_sample_count", 0) == 0
        and quality.get("xy_jump_over_80mm_count", 0) == 0
        and quality.get("yaw_jump_over_0_35rad_count", 0) == 0
        and quality.get("z_outside_0_25_to_0_40m_count", 0) == 0
        and quality.get("tilt_over_0_35rad_count", 0) == 0
    )


def reference_lifecycle(
    reference: dict[str, np.ndarray], manifest: dict[str, Any]
) -> tuple[dict[str, Any], list[str]]:
    timestamps = reference["t"]
    speed = np.hypot(reference["vx"], reference["vy"])
    moving_indices = np.flatnonzero(speed > REFERENCE_SPEED_EPS)
    moving_mask = speed > REFERENCE_SPEED_EPS
    moving_starts = int(
        np.count_nonzero(moving_mask & ~np.r_[False, moving_mask[:-1]])
    )
    moving_directions = np.column_stack((reference["vx"], reference["vy"]))
    phase_starts = moving_mask & ~np.r_[False, moving_mask[:-1]]
    adjacent_motion = moving_mask & np.r_[False, moving_mask[:-1]]
    direction_change = np.zeros(moving_mask.shape, dtype=bool)
    if np.any(adjacent_motion):
        current = moving_directions[adjacent_motion]
        previous = moving_directions[np.flatnonzero(adjacent_motion) - 1]
        current /= np.linalg.norm(current, axis=1)[:, None]
        previous /= np.linalg.norm(previous, axis=1)[:, None]
        direction_change[adjacent_motion] = (
            np.sum(current * previous, axis=1)
            < math.cos(math.radians(10.0))
        )
    motion_phase_count = int(np.count_nonzero(phase_starts | direction_change))
    internal_dwell_s = 0.0
    if moving_starts > 1:
        active_runs = np.flatnonzero(
            moving_mask & ~np.r_[False, moving_mask[:-1]]
        )
        prior_ends = np.flatnonzero(
            moving_mask & ~np.r_[moving_mask[1:], False]
        )
        for prior_end, next_start in zip(prior_ends[:-1], active_runs[1:]):
            internal_dwell_s += float(
                timestamps[next_start] - timestamps[prior_end]
            )
    quality = stream_quality(timestamps)
    reasons: list[str] = []
    if timestamps.size == 0:
        reasons.append("reference topic is empty")
        return {**quality, "complete": False}, reasons

    if moving_indices.size:
        startup_zero_s = float(timestamps[moving_indices[0]] - timestamps[0])
        final_zero_s = float(timestamps[-1] - timestamps[moving_indices[-1]])
        observed_motion_s = float(
            timestamps[moving_indices[-1]] - timestamps[moving_indices[0]]
        )
    else:
        startup_zero_s = quality["duration_s"]
        final_zero_s = 0.0
        observed_motion_s = 0.0
        reasons.append("reference never commanded motion")

    endpoint_delta = float(
        math.hypot(
            reference["x"][-1] - reference["x"][0],
            reference["y"][-1] - reference["y"][0],
        )
    )
    origin_x = float(manifest.get("origin_x_m", reference["x"][0]))
    origin_y = float(manifest.get("origin_y_m", reference["y"][0]))
    origin_error = float(
        math.hypot(reference["x"][0] - origin_x, reference["y"][0] - origin_y)
    )
    expected_total = float(manifest.get("expected_total_duration_s", 0.0) or 0.0)
    expected_motion = float(
        manifest.get("expected_motion_duration_s", 0.0) or 0.0
    )
    configured_rate = float(manifest.get("reference_timer_hz", 50.0) or 50.0)
    configured_startup_hold = float(
        manifest.get("startup_hold_s", 2.0) or 0.0
    )
    configured_final_hold = float(manifest.get("final_hold_s", 3.0) or 3.0)
    duration_tolerance = max(0.25, 5.0 / max(configured_rate, 1.0))
    completion_fraction = (
        min(1.0, quality["duration_s"] / expected_total)
        if expected_total > 0.0
        else 1.0
    )

    if expected_total > 0.0 and abs(
        quality["duration_s"] - expected_total
    ) > duration_tolerance:
        reasons.append("reference duration disagrees with metadata")
    if abs(startup_zero_s - configured_startup_hold) > duration_tolerance:
        reasons.append("startup zero-velocity hold disagrees with metadata")
    if expected_motion > 0.0 and abs(
        observed_motion_s - expected_motion
    ) > duration_tolerance:
        reasons.append("observed motion duration disagrees with metadata")
    if abs(final_zero_s - configured_final_hold) > duration_tolerance:
        reasons.append("final zero-velocity hold disagrees with metadata")
    expected_phases_value = manifest.get("expected_motion_phase_count")
    if expected_phases_value is None:
        if moving_starts > 1:
            reasons.append("reference contains multiple motion cycles")
        expected_motion_phases = 1
    else:
        expected_motion_phases = int(expected_phases_value)
        if expected_motion_phases < 1:
            reasons.append("metadata expected_motion_phase_count is invalid")
        elif motion_phase_count != expected_motion_phases:
            reasons.append("reference motion phase count disagrees with metadata")
    expected_dwell_value = manifest.get("expected_dwell_duration_s")
    expected_dwell_s = float(expected_dwell_value or 0.0)
    if expected_dwell_value is not None and abs(
        internal_dwell_s - expected_dwell_s
    ) > duration_tolerance:
        reasons.append("reference internal dwell duration disagrees with metadata")
    if bool(manifest.get("closed", False)) and endpoint_delta > 0.03:
        reasons.append("closed trajectory did not return to its first reference")
    if origin_error > 0.03:
        reasons.append("first reference does not match the declared reference origin")
    if quality["max_gap_s"] is not None and quality["max_gap_s"] > 0.10:
        reasons.append("reference publication contains a gap over 100 ms")

    return (
        {
            **quality,
            "configured_rate_hz": configured_rate,
            "configured_startup_hold_s": configured_startup_hold,
            "expected_total_duration_s": expected_total,
            "expected_motion_duration_s": expected_motion,
            "observed_motion_duration_s": observed_motion_s,
            "motion_cycle_count": moving_starts,
            "motion_phase_count": motion_phase_count,
            "expected_motion_phase_count": expected_motion_phases,
            "observed_internal_dwell_s": internal_dwell_s,
            "expected_internal_dwell_s": expected_dwell_s,
            "completion_fraction": completion_fraction,
            "startup_zero_s": startup_zero_s,
            "final_zero_s": final_zero_s,
            "endpoint_delta_m": endpoint_delta,
            "origin_error_m": origin_error,
            "complete": not reasons,
        },
        reasons,
    )


def _metric_summary(values: np.ndarray) -> dict[str, float | None]:
    if values.size == 0:
        return {"mean": None, "rmse": None, "p95_abs": None, "abs_max": None}
    return {
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values**2))),
        "p95_abs": float(np.percentile(np.abs(values), 95.0)),
        "abs_max": float(np.max(np.abs(values))),
    }


def vicon_source_clock_mapping(
    receipt_t: np.ndarray, source_t: np.ndarray
) -> tuple[dict[str, Any], np.ndarray]:
    """Map a usable Vicon source clock to bag time with one robust offset."""
    receipt_t = np.asarray(receipt_t, dtype=float)
    source_t = np.asarray(source_t, dtype=float)
    unavailable = np.full(receipt_t.shape, np.nan, dtype=float)
    if source_t.size != receipt_t.size:
        return (
            {
                "available": False,
                "reason": "source and receipt timestamp counts differ",
            },
            unavailable,
        )
    if source_t.size < 2:
        return (
            {
                "available": False,
                "reason": "fewer than two source timestamps are available",
            },
            unavailable,
        )
    if not np.all(np.isfinite(receipt_t)):
        return (
            {"available": False, "reason": "receipt timestamps are non-finite"},
            unavailable,
        )
    if not np.all(np.isfinite(source_t)):
        return (
            {"available": False, "reason": "source timestamps are non-finite"},
            unavailable,
        )
    if np.any(source_t <= 0.0):
        return (
            {"available": False, "reason": "source timestamps are nonpositive"},
            unavailable,
        )
    if np.any(np.diff(source_t) <= 0.0):
        return (
            {
                "available": False,
                "reason": "source timestamps are duplicated or nonmonotonic",
            },
            unavailable,
        )

    source_to_bag_offset = float(np.median(receipt_t - source_t))
    mapped_source_t = source_t + source_to_bag_offset
    receipt_residual = receipt_t - mapped_source_t
    residual_summary = _metric_summary(receipt_residual)
    edge_count = max(1, int(math.ceil(0.10 * source_t.size)))
    residual_drift = float(
        np.median(receipt_residual[-edge_count:])
        - np.median(receipt_residual[:edge_count])
    )
    mapped_duration = float(mapped_source_t[-1] - mapped_source_t[0])
    return (
        {
            "available": True,
            "mapping": "mapped_source_t = source_t + median(receipt_t - source_t)",
            "source_to_bag_offset_s": source_to_bag_offset,
            "receipt_minus_mapped_source_s": residual_summary,
            "receipt_minus_mapped_source_median_s": float(
                np.median(receipt_residual)
            ),
            "receipt_residual_over_50ms_count": int(
                np.count_nonzero(np.abs(receipt_residual) > 0.05)
            ),
            "receipt_residual_end_minus_start_s": residual_drift,
            "receipt_residual_drift_rate_ppm": (
                residual_drift / mapped_duration * 1e6
                if mapped_duration > 0.0
                else None
            ),
            "residual_drift_definition": (
                "last-decile median residual minus first-decile median "
                "residual; no clock-rate correction is applied"
            ),
            "absolute_delivery_latency_available": False,
            "interpretation": (
                "The median constant offset places source-stamped captures on "
                "the bag/reference clock without allowing delivery bursts to "
                "compress capture time. Receipt-minus-mapped-source residual is "
                "relative delivery jitter: positive means later than the run's "
                "typical receipt timing. The offset combines unknown clock offset "
                "and typical transport delay, so neither it nor the residual is "
                "an absolute latency measurement."
            ),
        },
        mapped_source_t,
    )


def _causal_trailing_chord_velocity(
    timestamps: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    window_s: float = VELOCITY_SMOOTHING_S,
) -> tuple[np.ndarray, np.ndarray]:
    """Estimate causal velocity from displacement over a trailing time window."""
    timestamps = np.asarray(timestamps, dtype=float)
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if timestamps.size != x.size or timestamps.size != y.size:
        raise ValueError("velocity timestamps and positions must have equal sizes")
    if timestamps.size == 0:
        return np.asarray([], dtype=float), np.asarray([], dtype=float)
    if window_s <= 0.0 or not math.isfinite(window_s):
        raise ValueError("velocity window must be finite and positive")
    if (
        not np.all(np.isfinite(timestamps))
        or not np.all(np.isfinite(x))
        or not np.all(np.isfinite(y))
        or np.any(np.diff(timestamps) <= 0.0)
    ):
        raise ValueError("velocity inputs must be finite with increasing time")

    velocity_x = np.zeros(timestamps.shape, dtype=float)
    velocity_y = np.zeros(timestamps.shape, dtype=float)
    for index in range(1, timestamps.size):
        cutoff = float(timestamps[index] - window_s)
        right = int(np.searchsorted(timestamps, cutoff, side="left"))
        if right == 0:
            start_t = float(timestamps[0])
            start_x = float(x[0])
            start_y = float(y[0])
        elif timestamps[right] == cutoff:
            start_t = cutoff
            start_x = float(x[right])
            start_y = float(y[right])
        else:
            left = right - 1
            interval_fraction = (cutoff - timestamps[left]) / (
                timestamps[right] - timestamps[left]
            )
            start_t = cutoff
            start_x = float(
                x[left] + interval_fraction * (x[right] - x[left])
            )
            start_y = float(
                y[left] + interval_fraction * (y[right] - y[left])
            )
        elapsed = float(timestamps[index] - start_t)
        velocity_x[index] = (x[index] - start_x) / elapsed
        velocity_y[index] = (y[index] - start_y) / elapsed
    return velocity_x, velocity_y


def _causal_step_resample(
    query_t: np.ndarray, sample_t: np.ndarray, values: np.ndarray
) -> np.ndarray:
    """Resample from the newest observation at or before each query time."""
    if sample_t.size == 0 or sample_t.size != values.size:
        raise ValueError("causal resampling requires equal nonempty samples")
    indices = np.searchsorted(sample_t, query_t, side="right") - 1
    indices = np.clip(indices, 0, sample_t.size - 1)
    return values[indices]


def _contiguous_time_runs(
    timestamps: np.ndarray,
    mask: np.ndarray,
    max_gap_s: float = PRIMARY_WINDOW_MAX_EFFECTIVE_GAP_S,
) -> list[tuple[int, int]]:
    """Split selected samples at holds, nonpositive time, or data gaps."""
    result = []
    for start, stop in _active_runs(mask):
        gaps = np.diff(timestamps[start : stop + 1])
        breaks = np.flatnonzero((gaps <= 0.0) | (gaps > max_gap_s)) + start
        segment_start = start
        for before_gap in breaks:
            if before_gap >= segment_start:
                result.append((segment_start, int(before_gap)))
            segment_start = int(before_gap + 1)
        if stop >= segment_start:
            result.append((segment_start, stop))
    return result


def _masked_time_weighted_mean(
    timestamps: np.ndarray, values: np.ndarray, mask: np.ndarray
) -> float | None:
    """Integrate only contiguous selected spans, without bridging holds."""
    integral = 0.0
    duration = 0.0
    finite_mask = mask & np.isfinite(timestamps) & np.isfinite(values)
    for start, stop in _contiguous_time_runs(timestamps, finite_mask):
        if stop <= start:
            continue
        segment_t = timestamps[start : stop + 1]
        segment_values = values[start : stop + 1]
        integral += float(np.trapz(segment_values, segment_t))
        duration += float(segment_t[-1] - segment_t[0])
    return integral / duration if duration > 0.0 else None



def _hold_xy_jitter(
    actual_x: np.ndarray, actual_y: np.ndarray, mask: np.ndarray
) -> dict[str, Any]:
    sample_count = int(np.count_nonzero(mask))
    if sample_count < 5:
        return {"sample_count": sample_count, "displacement_m": None}
    center_x = float(np.median(actual_x[mask]))
    center_y = float(np.median(actual_y[mask]))
    displacement = np.hypot(
        actual_x[mask] - center_x,
        actual_y[mask] - center_y,
    )
    return {
        "sample_count": sample_count,
        "median_x_m": center_x,
        "median_y_m": center_y,
        "displacement_m": _metric_summary(displacement),
        "definition": "Vicon XY displacement from the hold's median position",
    }

def tracking_metrics(
    reference: dict[str, np.ndarray],
    base: dict[str, np.ndarray],
    turret: dict[str, np.ndarray] | None,
    manifest: dict[str, Any] | None = None,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    full_command_speed = np.hypot(reference["vx"], reference["vy"])
    reference_motion_runs = _active_runs(
        full_command_speed > ACTIVE_TRANSLATION_SPEED_M_S
    )
    reference_phase_starts_t = np.asarray(
        [reference["t"][start] for start, _stop in reference_motion_runs],
        dtype=float,
    )
    reference_phase_stops_t = np.asarray(
        [reference["t"][stop] for _start, stop in reference_motion_runs],
        dtype=float,
    )
    reference_internal_dwell_starts_t = np.asarray(
        [
            reference["t"][stop + 1]
            for (_start, stop), (next_start, _next_stop) in zip(
                reference_motion_runs[:-1], reference_motion_runs[1:]
            )
            if stop + 1 < next_start
        ],
        dtype=float,
    )
    finite_base = (
        np.isfinite(base["t"])
        & np.isfinite(base["x"])
        & np.isfinite(base["y"])
        & np.isfinite(base["yaw"])
    )
    source_t_value = np.asarray(
        base.get("source_t", np.asarray([])), dtype=float
    )
    if source_t_value.size != base["t"].size:
        source_t_value = np.full(base["t"].shape, np.nan, dtype=float)
    base_t, base_x, base_y, base_yaw, base_source_t = unique_time_series(
        base["t"][finite_base],
        base["x"][finite_base],
        base["y"][finite_base],
        base["yaw"][finite_base],
        source_t_value[finite_base],
    )
    clock_mapping, mapped_source_t = vicon_source_clock_mapping(
        base_t, base_source_t
    )
    source_alignment_available = bool(clock_mapping["available"])
    if source_alignment_available:
        alignment_t = mapped_source_t
        pose_alignment_time_basis = "vicon_source_stamp_mapped_to_bag_time"
        velocity_time_basis = "vicon_source_stamp"
    else:
        alignment_t = base_t
        pose_alignment_time_basis = "bag_receipt_time_fallback"
        velocity_time_basis = "bag_receipt_time_fallback"

    match_distance = nearest_time_distance(reference["t"], alignment_t)
    matched = match_distance <= MAX_POSE_MATCH_DT_S
    if np.count_nonzero(matched) < 10:
        raise RuntimeError("fewer than 10 time-aligned Vicon/reference samples")

    time_s = reference["t"][matched]
    ref_x = reference["x"][matched]
    ref_y = reference["y"][matched]
    ref_yaw = reference["yaw"][matched]
    ref_vx = reference["vx"][matched]
    ref_vy = reference["vy"][matched]
    actual_x = np.interp(time_s, alignment_t, base_x)
    actual_y = np.interp(time_s, alignment_t, base_y)
    actual_yaw = np.interp(time_s, alignment_t, np.unwrap(base_yaw))
    receipt_actual_x = np.interp(time_s, base_t, base_x)
    receipt_actual_y = np.interp(time_s, base_t, base_y)
    receipt_actual_yaw = np.interp(time_s, base_t, np.unwrap(base_yaw))
    error_x = actual_x - ref_x
    error_y = actual_y - ref_y
    error_xy = np.hypot(error_x, error_y)
    receipt_error_x = receipt_actual_x - ref_x
    receipt_error_y = receipt_actual_y - ref_y
    receipt_error_xy = np.hypot(receipt_error_x, receipt_error_y)
    command_speed = np.hypot(ref_vx, ref_vy)
    moving = command_speed > ACTIVE_TRANSLATION_SPEED_M_S
    if np.count_nonzero(moving) < 10:
        raise RuntimeError("fewer than 10 active-motion tracking samples")
    tangent_x = np.zeros_like(command_speed)
    tangent_y = np.zeros_like(command_speed)
    tangent_x[moving] = ref_vx[moving] / command_speed[moving]
    tangent_y[moving] = ref_vy[moving] / command_speed[moving]
    along_error = (
        error_x[moving] * tangent_x[moving]
        + error_y[moving] * tangent_y[moving]
    )
    cross_error = (
        -error_x[moving] * tangent_y[moving]
        + error_y[moving] * tangent_x[moving]
    )
    receipt_along_error = (
        receipt_error_x[moving] * tangent_x[moving]
        + receipt_error_y[moving] * tangent_y[moving]
    )
    receipt_cross_error = (
        -receipt_error_x[moving] * tangent_y[moving]
        + receipt_error_y[moving] * tangent_x[moving]
    )

    if np.any(np.diff(time_s) <= 0.0):
        raise RuntimeError("matched reference timestamps must strictly increase")
    base_actual_vx, base_actual_vy = _causal_trailing_chord_velocity(
        alignment_t, base_x, base_y
    )
    receipt_base_actual_vx, receipt_base_actual_vy = (
        _causal_trailing_chord_velocity(base_t, base_x, base_y)
    )
    actual_vx = _causal_step_resample(
        time_s, alignment_t, base_actual_vx
    )
    actual_vy = _causal_step_resample(
        time_s, alignment_t, base_actual_vy
    )
    receipt_actual_vx = _causal_step_resample(
        time_s, base_t, receipt_base_actual_vx
    )
    receipt_actual_vy = _causal_step_resample(
        time_s, base_t, receipt_base_actual_vy
    )
    actual_tangent_speed = (
        actual_vx[moving] * tangent_x[moving]
        + actual_vy[moving] * tangent_y[moving]
    )
    receipt_actual_tangent_speed = (
        receipt_actual_vx[moving] * tangent_x[moving]
        + receipt_actual_vy[moving] * tangent_y[moving]
    )
    speed_error = actual_tangent_speed - command_speed[moving]
    receipt_speed_error = (
        receipt_actual_tangent_speed - command_speed[moving]
    )
    moving_yaw = actual_yaw[moving]
    base_yaw_drift = moving_yaw - moving_yaw[0]
    moving_indices = np.flatnonzero(moving)
    startup_hold = np.arange(time_s.size) < moving_indices[0]
    final_hold = np.arange(time_s.size) > moving_indices[-1]
    startup_jitter = _hold_xy_jitter(actual_x, actual_y, startup_hold)
    final_jitter = _hold_xy_jitter(actual_x, actual_y, final_hold)

    first_reference_match_dt_s = float(match_distance[0])
    last_reference_match_dt_s = float(match_distance[-1])
    final_xy_error_m = None
    if last_reference_match_dt_s <= MAX_POSE_MATCH_DT_S:
        final_actual_x = float(
            np.interp(reference["t"][-1], alignment_t, base_x)
        )
        final_actual_y = float(
            np.interp(reference["t"][-1], alignment_t, base_y)
        )
        final_xy_error_m = float(
            math.hypot(
                final_actual_x - reference["x"][-1],
                final_actual_y - reference["y"][-1],
            )
        )

    receipt_reference_match_distance = nearest_time_distance(
        reference["t"], base_t
    )
    receipt_reference_matched = (
        receipt_reference_match_distance <= MAX_POSE_MATCH_DT_S
    )
    if source_alignment_available:
        native_receipt_residual = base_t - mapped_source_t
        source_receipt_residual = np.interp(
            time_s, mapped_source_t, native_receipt_residual
        )
    else:
        native_receipt_residual = np.full(base_t.shape, np.nan, dtype=float)
        source_receipt_residual = np.full(time_s.shape, np.nan, dtype=float)

    yaw_summary = None
    if turret is not None and turret["t"].size >= 2:
        turret_t, turret_yaw = unique_time_series(turret["t"], turret["yaw"])
        yaw_match = nearest_time_distance(time_s, turret_t) <= MAX_POSE_MATCH_DT_S
        if np.count_nonzero(yaw_match) >= 10:
            actual_turret_yaw = np.interp(
                time_s[yaw_match], turret_t, np.unwrap(turret_yaw)
            )
            reference_yaw = ref_yaw[yaw_match]
            yaw_error = np.arctan2(
                np.sin(actual_turret_yaw - reference_yaw),
                np.cos(actual_turret_yaw - reference_yaw),
            )
            yaw_summary = _metric_summary(yaw_error)

    turret_setting = (manifest or {}).get(
        "turret_control_enabled", (manifest or {}).get("turret_enabled", False)
    )
    turret_enabled = bool(turret_setting is True)
    receipt_diagnostic = {
        "definition": (
            "Diagnostic replay of the same Vicon poses on rosbag receipt time. "
            "Delivery gaps and catch-up bursts remain visible and these values "
            "are not used for analysis-v4 primary tracking outcomes."
        ),
        "matched_sample_count": int(
            np.count_nonzero(receipt_reference_matched)
        ),
        "coverage_fraction": float(
            np.count_nonzero(receipt_reference_matched)
            / max(reference["t"].size, 1)
        ),
        "max_pose_match_dt_s": (
            float(
                np.max(
                    receipt_reference_match_distance[
                        receipt_reference_matched
                    ]
                )
            )
            if np.any(receipt_reference_matched)
            else None
        ),
        "first_reference_match_dt_s": float(
            receipt_reference_match_distance[0]
        ),
        "last_reference_match_dt_s": float(
            receipt_reference_match_distance[-1]
        ),
        "xy_error_m": _metric_summary(receipt_error_xy[moving]),
        "x_error_m": _metric_summary(receipt_error_x[moving]),
        "y_error_m": _metric_summary(receipt_error_y[moving]),
        "along_track_error_m": _metric_summary(receipt_along_error),
        "cross_track_error_m": _metric_summary(receipt_cross_error),
        "actual_tangent_speed_m_s": _metric_summary(
            receipt_actual_tangent_speed
        ),
        "speed_error_m_s": _metric_summary(receipt_speed_error),
        "velocity_estimator": (
            "causal Vicon position chord over up to the trailing 0.20 s of "
            "receipt time, resampled with previous-observation hold"
        ),
    }
    metrics = {
        "matched_sample_count": int(time_s.size),
        "coverage_fraction": float(time_s.size / max(reference["t"].size, 1)),
        "max_pose_match_dt_s": float(np.max(match_distance[matched])),
        "first_reference_match_dt_s": first_reference_match_dt_s,
        "last_reference_match_dt_s": last_reference_match_dt_s,
        "pose_alignment_time_basis": pose_alignment_time_basis,
        "velocity_time_basis": velocity_time_basis,
        "source_stamp_clock_mapping": clock_mapping,
        "source_receipt_residual_at_reference_s": _metric_summary(
            source_receipt_residual[np.isfinite(source_receipt_residual)]
        ),
        "receipt_time_alignment_diagnostic": receipt_diagnostic,
        "primary_tracking_interpretation": (
            "Primary pose and velocity use mapped Vicon capture time when "
            "source stamps are usable; receipt-time results are retained only "
            "as delivery diagnostics. Missing, nonfinite, nonpositive, frozen, "
            "or reversed source stamps cause an explicit receipt-time fallback."
        ),
        "xy_error_m": _metric_summary(error_xy[moving]),
        "active_time_weighted_xy_error_mean_m": _masked_time_weighted_mean(
            time_s, error_xy, moving
        ),
        "active_time_weighted_xy_error_definition": (
            "time integral of Euclidean XY tracking error divided by active "
            "reference-motion duration; zero-command holds/dwells are excluded "
            "and numerical integration splits gaps over 100 ms; formal use "
            "also requires complete <=50 ms matching and <=100 ms effective "
            "Vicon support in every active phase"
        ),
        "x_error_m": _metric_summary(error_x[moving]),
        "y_error_m": _metric_summary(error_y[moving]),
        "all_lifecycle_xy_error_m": _metric_summary(error_xy),
        "along_track_error_m": _metric_summary(along_error),
        "cross_track_error_m": _metric_summary(cross_error),
        "startup_hold_xy_jitter": startup_jitter,
        "final_hold_xy_jitter": final_jitter,
        "final_hold_xy_error_m": final_xy_error_m,
        "actual_closure_error_m": float(
            math.hypot(actual_x[-1] - actual_x[0], actual_y[-1] - actual_y[0])
        ),
        "commanded_speed_mean_m_s": float(np.mean(command_speed[moving])),
        "actual_tangent_speed_mean_m_s": float(np.mean(actual_tangent_speed)),
        "actual_tangent_speed_m_s": _metric_summary(actual_tangent_speed),
        "speed_error_m_s": _metric_summary(speed_error),
        "velocity_estimator": (
            "causal Vicon position chord over up to the trailing 0.20 s of the "
            "selected timestamp basis, resampled with previous-observation hold"
        ),
        "velocity_filter_future_leakage": False,
        "response_timing_filter_delay_bound_s": VELOCITY_SMOOTHING_S,
        "turret_control_enabled": turret_enabled,
        "turret_reference_difference_rad": yaw_summary,
        "turret_yaw_error_rad": yaw_summary if turret_enabled else None,
        "turret_reference_interpretation": (
            "Descriptive turret-minus-reference difference; it is a tracking "
            "error only when metadata explicitly records turret_enabled=true."
        ),
        "base_yaw_drift_rad": _metric_summary(base_yaw_drift),
        "base_yaw_net_change_rad": float(base_yaw_drift[-1]),
        "base_yaw_peak_to_peak_rad": float(np.ptp(moving_yaw)),
        "base_yaw_interpretation": (
            "Measured chassis/base yaw drift relative to motion onset; this is "
            "not reference-yaw tracking."
        ),
        "error_sign_convention": "actual_minus_reference",
    }
    traces = {
        "t": time_s,
        "ref_x": ref_x,
        "ref_y": ref_y,
        "ref_yaw": ref_yaw,
        "actual_x": actual_x,
        "actual_y": actual_y,
        "actual_yaw": actual_yaw,
        "actual_vx": actual_vx,
        "actual_vy": actual_vy,
        "error_x": error_x,
        "error_y": error_y,
        "error_xy": error_xy,
        "primary_error_x": error_x,
        "primary_error_y": error_y,
        "primary_error_xy": error_xy,
        "moving": moving,
        "along_error": along_error,
        "cross_error": cross_error,
        "primary_along_error": along_error,
        "primary_cross_error": cross_error,
        "moving_t": time_s[moving],
        "tangent_x": tangent_x[moving],
        "tangent_y": tangent_y[moving],
        "command_speed": command_speed[moving],
        "actual_tangent_speed": actual_tangent_speed,
        "primary_actual_tangent_speed": actual_tangent_speed,
        "receipt_actual_x": receipt_actual_x,
        "receipt_actual_y": receipt_actual_y,
        "receipt_actual_yaw": receipt_actual_yaw,
        "receipt_actual_vx": receipt_actual_vx,
        "receipt_actual_vy": receipt_actual_vy,
        "receipt_error_x": receipt_error_x,
        "receipt_error_y": receipt_error_y,
        "receipt_error_xy": receipt_error_xy,
        "receipt_along_error": receipt_along_error,
        "receipt_cross_error": receipt_cross_error,
        "receipt_actual_tangent_speed": receipt_actual_tangent_speed,
        "source_receipt_residual": source_receipt_residual,
        "vicon_receipt_t": base_t,
        "vicon_source_t": base_source_t,
        "vicon_mapped_source_t": mapped_source_t,
        "vicon_receipt_minus_mapped_source": native_receipt_residual,
        "reference_motion_phase_starts_t": reference_phase_starts_t,
        "reference_motion_phase_stops_t": reference_phase_stops_t,
        "reference_internal_dwell_starts_t": (
            reference_internal_dwell_starts_t
        ),
    }
    return metrics, traces

def detect_triangle_corners(
    reference: dict[str, np.ndarray], traces: dict[str, np.ndarray]
) -> dict[str, Any]:
    speed = np.hypot(reference["vx"], reference["vy"])
    moving_indices = np.flatnonzero(speed > 0.02)
    corners: list[dict[str, Any]] = []
    previous_direction: np.ndarray | None = None
    previous_index: int | None = None
    for index in moving_indices:
        direction = np.asarray(
            [reference["vx"][index], reference["vy"][index]], dtype=float
        ) / speed[index]
        direction_dot = (
            float(np.clip(np.dot(previous_direction, direction), -1.0, 1.0))
            if previous_direction is not None
            else 1.0
        )
        if previous_direction is not None and direction_dot < math.cos(
            math.radians(10.0)
        ):
            corner_t = float(reference["t"][index])
            corner_x = float(reference["x"][index])
            corner_y = float(reference["y"][index])
            actual_x = float(np.interp(corner_t, traces["t"], traces["actual_x"]))
            actual_y = float(np.interp(corner_t, traces["t"], traces["actual_y"]))
            corner_error = math.hypot(actual_x - corner_x, actual_y - corner_y)
            window = np.abs(traces["t"] - corner_t) <= 1.5
            distances = np.hypot(
                traces["actual_x"][window] - corner_x,
                traces["actual_y"][window] - corner_y,
            )
            window_times = traces["t"][window]
            closest_index = int(np.argmin(distances)) if distances.size else 0
            cross = float(
                previous_direction[0] * direction[1]
                - previous_direction[1] * direction[0]
            )
            commanded_turn_angle_deg = math.degrees(
                math.acos(direction_dot)
            )
            corners.append(
                {
                    "index": len(corners) + 1,
                    "turn": "left" if cross > 0.0 else "right",
                    "commanded_turn_angle_deg": commanded_turn_angle_deg,
                    "interior_angle_deg": 180.0 - commanded_turn_angle_deg,
                    "reference_time_s": corner_t,
                    "reference_x_m": corner_x,
                    "reference_y_m": corner_y,
                    "time_aligned_error_m": float(corner_error),
                    "closest_distance_m": float(distances[closest_index])
                    if distances.size
                    else None,
                    "closest_point_delay_s": float(
                        window_times[closest_index] - corner_t
                    )
                    if distances.size
                    else None,
                }
            )
        previous_direction = direction
        previous_index = int(index)
    del previous_index
    errors = np.asarray(
        [item["time_aligned_error_m"] for item in corners], dtype=float
    )
    angle_groups: list[dict[str, Any]] = []
    grouped_angles = sorted(
        {round(item["interior_angle_deg"], 6) for item in corners}
    )
    for interior_angle_deg in grouped_angles:
        group = [
            item
            for item in corners
            if math.isclose(
                item["interior_angle_deg"], interior_angle_deg, abs_tol=1e-6
            )
        ]
        group_errors = np.asarray(
            [item["time_aligned_error_m"] for item in group], dtype=float
        )
        group_closest = np.asarray(
            [
                item["closest_distance_m"]
                for item in group
                if item["closest_distance_m"] is not None
            ],
            dtype=float,
        )
        group_delays = np.asarray(
            [
                item["closest_point_delay_s"]
                for item in group
                if item["closest_point_delay_s"] is not None
            ],
            dtype=float,
        )
        angle_groups.append(
            {
                "interior_angle_deg": interior_angle_deg,
                "commanded_turn_angle_deg": 180.0 - interior_angle_deg,
                "corner_count": len(group),
                "time_aligned_error_m": _metric_summary(group_errors),
                "closest_distance_m": _metric_summary(group_closest),
                "closest_point_delay_s": _metric_summary(group_delays),
            }
        )
    tangents = np.column_stack((traces["tangent_x"], traces["tangent_y"]))
    changes = (
        np.flatnonzero(
            np.sum(tangents[1:] * tangents[:-1], axis=1)
            < math.cos(math.radians(10.0))
        )
        + 1
    )
    boundaries = [0, *changes.tolist(), len(tangents)]
    legs: list[dict[str, Any]] = []
    for leg_index, (start, stop) in enumerate(
        zip(boundaries[:-1], boundaries[1:]), start=1
    ):
        cross = traces["cross_error"][start:stop]
        if cross.size == 0:
            continue
        legs.append(
            {
                "index": leg_index,
                "sample_count": int(cross.size),
                "direction_x": float(tangents[start, 0]),
                "direction_y": float(tangents[start, 1]),
                "cross_track_error_m": _metric_summary(cross),
            }
        )
    leg_rmses = np.asarray(
        [item["cross_track_error_m"]["rmse"] for item in legs], dtype=float
    )
    return {
        "corner_count": len(corners),
        "corner_error_mean_m": float(np.mean(errors)) if errors.size else None,
        "corner_error_max_m": float(np.max(errors)) if errors.size else None,
        "corners": corners,
        "corner_angle_groups": angle_groups,
        "leg_count": len(legs),
        "leg_cross_track_rmse_mean_m": float(np.mean(leg_rmses))
        if leg_rmses.size
        else None,
        "leg_cross_track_rmse_max_m": float(np.max(leg_rmses))
        if leg_rmses.size
        else None,
        "legs": legs,
    }




def _causal_moving_average(values: np.ndarray, sample_count: int) -> np.ndarray:
    """Return a trailing moving average with no future-sample leakage."""
    if values.size == 0 or sample_count <= 1:
        return values.copy()
    width = min(int(sample_count), int(values.size))
    cumulative = np.r_[0.0, np.cumsum(values, dtype=float)]
    indices = np.arange(values.size)
    starts = np.maximum(0, indices - width + 1)
    totals = cumulative[indices + 1] - cumulative[starts]
    return totals / (indices - starts + 1)

def _centered_moving_average(values: np.ndarray, sample_count: int) -> np.ndarray:
    """Smooth a one-dimensional trace without zero-padding its edges."""
    if values.size == 0 or sample_count <= 1:
        return values.copy()
    width = min(int(sample_count), int(values.size))
    if width % 2 == 0:
        width = max(1, width - 1)
    pad = width // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    return np.convolve(padded, np.ones(width) / width, mode="valid")


def _first_sustained(
    timestamps: np.ndarray, condition: np.ndarray, duration_s: float
) -> float | None:
    """Return the start of the first continuously true interval."""
    if timestamps.size == 0:
        return None
    start: int | None = None
    for index, value in enumerate(condition):
        if value and start is None:
            start = index
        if (not value or index == condition.size - 1) and start is not None:
            stop = index if value else index - 1
            if timestamps[stop] - timestamps[start] >= duration_s:
                return float(timestamps[start])
            start = None
    return None


def speed_response_metrics(
    timestamps: np.ndarray,
    commanded_speed: np.ndarray,
    actual_speed: np.ndarray,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    """Measure a constant-speed step response using smoothed Vicon velocity."""
    finite = (
        np.isfinite(timestamps)
        & np.isfinite(commanded_speed)
        & np.isfinite(actual_speed)
    )
    timestamps = timestamps[finite]
    commanded_speed = commanded_speed[finite]
    actual_speed = actual_speed[finite]
    if timestamps.size < 10:
        return {"available": False, "reason": "fewer than 10 speed samples"}, {}
    gaps = np.diff(timestamps)
    positive_gaps = gaps[gaps > 0.0]
    if positive_gaps.size == 0:
        return {"available": False, "reason": "speed timestamps do not advance"}, {}
    smoothed = actual_speed.copy()
    target = float(np.median(commanded_speed))
    if target <= ACTIVE_TRANSLATION_SPEED_M_S:
        return {"available": False, "reason": "commanded speed is too small"}, {}

    rise_at = _first_sustained(
        timestamps,
        smoothed >= 0.90 * target,
        RESPONSE_SUSTAIN_S,
    )
    rise_time = rise_at - timestamps[0] if rise_at is not None else None
    tolerance = max(0.10 * target, 0.005)
    settled = np.abs(smoothed - target) <= tolerance
    edge_safe = timestamps <= timestamps[-1] - RESPONSE_SUSTAIN_S
    eligible = np.flatnonzero(edge_safe)
    settling_time = None
    if eligible.size:
        last = int(eligible[-1])
        suffix_good = np.logical_and.accumulate(
            settled[: last + 1][::-1]
        )[::-1]
        enough_duration = (
            timestamps[: last + 1]
            <= timestamps[last] - RESPONSE_SUSTAIN_S
        )
        candidates = np.flatnonzero(suffix_good & enough_duration)
        if candidates.size:
            settling_time = float(timestamps[candidates[0]] - timestamps[0])

    steady_start = timestamps[0] + 0.50 * (timestamps[-1] - timestamps[0])
    steady = timestamps >= steady_start
    steady_error = smoothed[steady] - commanded_speed[steady]
    metrics = {
        "available": True,
        "target_speed_m_s": target,
        "velocity_filter_window_s": VELOCITY_SMOOTHING_S,
        "velocity_filter_applied_here": False,
        "filter_delay_bound_s": VELOCITY_SMOOTHING_S,
        "timing_interpretation": (
            "Conservative observed timing; the single causal velocity filter "
            "can add up to 0.20 s and is not delay-corrected"
        ),
        "rise_definition": "first sustained 0.20 s at or above 90% of target",
        "settling_definition": (
            "first time smoothed speed remains within +/-10% (minimum "
            "0.005 m/s), excluding the final 0.20 s command edge"
        ),
        "rise_time_s": rise_time,
        "settling_time_s": settling_time,
        "steady_state_speed_error_m_s": _metric_summary(steady_error),
        "overshoot_m_s": float(max(0.0, np.max(smoothed - target))),
    }
    return metrics, {"t": timestamps, "actual_speed_smoothed": smoothed}



def _absolute_time_integral(
    timestamps: np.ndarray, values: np.ndarray
) -> float | None:
    if timestamps.size < 2 or values.size != timestamps.size:
        return None
    finite = np.isfinite(timestamps) & np.isfinite(values)
    total = 0.0
    integrated = False
    for start, stop in _contiguous_time_runs(timestamps, finite):
        if stop <= start:
            continue
        total += float(
            np.trapz(np.abs(values[start : stop + 1]), timestamps[start : stop + 1])
        )
        integrated = True
    return total if integrated else None

def translation_metrics(
    traces: dict[str, np.ndarray], manifest: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    """Report axis-specific tracking and speed response for straight motion."""
    moving = traces["moving"]
    ref_dx = float(traces["ref_x"][moving][-1] - traces["ref_x"][moving][0])
    ref_dy = float(traces["ref_y"][moving][-1] - traces["ref_y"][moving][0])
    axis = "x" if abs(ref_dx) >= abs(ref_dy) else "y"
    cross_axis = "y" if axis == "x" else "x"
    cross_error = traces[f"error_{cross_axis}"][moving]
    speed_response, response_traces = speed_response_metrics(
        traces["moving_t"],
        traces["command_speed"],
        traces["actual_tangent_speed"],
    )
    phase_starts = traces.get("reference_motion_phase_starts_t", np.asarray([]))
    command_onset_t = float(
        phase_starts[0] if phase_starts.size else traces["moving_t"][0]
    )
    onset_end = command_onset_t + TRANSLATION_ONSET_WINDOW_S
    onset = (traces["moving_t"] >= command_onset_t) & (
        traces["moving_t"] <= onset_end
    )
    onset_along = traces["along_error"][onset]
    onset_cross = cross_error[onset]
    onset_metrics = {
        "window_s": TRANSLATION_ONSET_WINDOW_S,
        "command_onset_time_s": command_onset_t,
        "window_end_time_s": onset_end,
        "along_track_error_m": _metric_summary(onset_along),
        "along_track_abs_integral_m_s": _absolute_time_integral(
            traces["moving_t"][onset], onset_along
        ),
        "cross_axis_error_m": _metric_summary(onset_cross),
        "cross_axis_abs_integral_m_s": _absolute_time_integral(
            traces["moving_t"][onset], onset_cross
        ),
        "definition": (
            "first 2.0 s after translation command onset, anchored to the "
            "reference clock; integration splits gaps over 100 ms and formal "
            "use requires the offline primary-window support rule"
        ),
    }
    metrics = {
        "profile": str(manifest.get("profile", "")),
        "command_axis": axis,
        "command_direction_sign": 1 if (ref_dx if axis == "x" else ref_dy) >= 0.0 else -1,
        "cross_axis": cross_axis,
        "cross_axis_error_m": _metric_summary(cross_error),
        "along_track_error_m": _metric_summary(traces["along_error"]),
        "onset_response": onset_metrics,
        "speed_response": speed_response,
        "coordinate_sign_convention": "actual_minus_reference in fixed Vicon axes",
    }
    profile_traces = {
        "t": traces["moving_t"],
        "cross_axis_error": cross_error,
        **response_traces,
    }
    return metrics, profile_traces


def _active_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    starts = np.flatnonzero(mask & ~np.r_[False, mask[:-1]])
    stops = np.flatnonzero(mask & ~np.r_[mask[1:], False])
    return [(int(start), int(stop)) for start, stop in zip(starts, stops)]


def reversal_metrics(
    traces: dict[str, np.ndarray], manifest: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    """Measure braking, dwell, and reverse response without scoring the dwell."""
    timestamps = traces["t"]
    phase_starts = traces.get("reference_motion_phase_starts_t", np.asarray([]))
    dwell_starts = traces.get(
        "reference_internal_dwell_starts_t", np.asarray([])
    )
    if phase_starts.size == 2 and dwell_starts.size == 1:
        dwell_start_t = float(dwell_starts[0])
        reverse_command_t = float(phase_starts[1])
    else:
        runs = _active_runs(traces["moving"])
        if len(runs) != 2:
            return (
                {
                    "available": False,
                    "reason": f"expected two moving phases, observed {len(runs)}",
                },
                {},
            )
        (_pre_start, pre_stop), (post_start, _post_stop) = runs
        if pre_stop + 1 >= post_start:
            return {"available": False, "reason": "no midpoint dwell observed"}, {}
        dwell_start_t = float(timestamps[pre_stop + 1])
        reverse_command_t = float(timestamps[post_start])
    commanded_dwell_s = reverse_command_t - dwell_start_t
    moving_t = traces["moving_t"]
    pre_moving = moving_t < reverse_command_t
    post_moving = ~pre_moving
    if np.count_nonzero(pre_moving) < 5 or np.count_nonzero(post_moving) < 5:
        return {"available": False, "reason": "reversal phases are too short"}, {}

    old_direction = np.asarray(
        [
            np.median(traces["tangent_x"][pre_moving]),
            np.median(traces["tangent_y"][pre_moving]),
        ]
    )
    new_direction = np.asarray(
        [
            np.median(traces["tangent_x"][post_moving]),
            np.median(traces["tangent_y"][post_moving]),
        ]
    )
    old_direction /= np.linalg.norm(old_direction)
    new_direction /= np.linalg.norm(new_direction)
    target_pre = traces["command_speed"][pre_moving]
    target_post = traces["command_speed"][post_moving]
    actual_pre = traces["actual_tangent_speed"][pre_moving]
    actual_post = traces["actual_tangent_speed"][post_moving]

    target = float(np.median(target_post))
    after_reverse = timestamps >= reverse_command_t
    speed_new_direction = (
        traces["actual_vx"] * new_direction[0]
        + traces["actual_vy"] * new_direction[1]
    )
    reverse_at = _first_sustained(
        timestamps[after_reverse],
        speed_new_direction[after_reverse] >= max(0.10 * target, 0.005),
        RESPONSE_SUSTAIN_S,
    )
    reversal_delay = (
        reverse_at - reverse_command_t if reverse_at is not None else None
    )

    during_dwell = (timestamps >= dwell_start_t) & (
        timestamps < reverse_command_t
    )
    actual_speed_norm = np.hypot(traces["actual_vx"], traces["actual_vy"])
    stopped_at = _first_sustained(
        timestamps[during_dwell],
        actual_speed_norm[during_dwell] <= max(0.10 * target, 0.005),
        RESPONSE_SUSTAIN_S,
    )
    stop_delay = stopped_at - dwell_start_t if stopped_at is not None else None

    midpoint_ref = np.asarray(
        [
            np.interp(reverse_command_t, timestamps, traces["ref_x"]),
            np.interp(reverse_command_t, timestamps, traces["ref_y"]),
        ]
    )
    actual_xy = np.column_stack((traces["actual_x"], traces["actual_y"]))
    midpoint_actual = np.asarray(
        [
            np.interp(reverse_command_t, timestamps, traces["actual_x"]),
            np.interp(reverse_command_t, timestamps, traces["actual_y"]),
        ]
    )
    after_dwell_xy = actual_xy[timestamps >= dwell_start_t]
    overshoot_projection = (after_dwell_xy - midpoint_ref) @ old_direction
    overshoot = float(max(0.0, np.max(overshoot_projection)))
    actual_dwell_start = np.asarray(
        [
            np.interp(dwell_start_t, timestamps, traces["actual_x"]),
            np.interp(dwell_start_t, timestamps, traces["actual_y"]),
        ]
    )
    dwell_drift = float(np.linalg.norm(midpoint_actual - actual_dwell_start))
    midpoint_error = float(np.linalg.norm(midpoint_actual - midpoint_ref))
    closure = float(np.linalg.norm(actual_xy[-1] - actual_xy[0]))
    event_start_t = dwell_start_t - REVERSAL_EVENT_PRE_S
    event_end_t = reverse_command_t + REVERSAL_EVENT_POST_S
    event = (timestamps >= event_start_t) & (timestamps <= event_end_t)
    event_error_x = traces["error_x"][event]
    event_error_y = traces["error_y"][event]
    event_xy = np.hypot(event_error_x, event_error_y)
    event_along = (
        event_error_x * old_direction[0]
        + event_error_y * old_direction[1]
    )
    event_cross = (
        -event_error_x * old_direction[1]
        + event_error_y * old_direction[0]
    )
    event_t = timestamps[event]
    event_metrics = {
        "start_time_s": event_start_t,
        "end_time_s": event_end_t,
        "definition": (
            "0.50 s before commanded dwell through 1.50 s after reverse command; "
            "reference-clock anchored, along/cross axes remain fixed to the "
            "outbound direction, integration splits gaps over 100 ms, and "
            "formal use requires the offline window-support rule"
        ),
        "xy_error_m": _metric_summary(event_xy),
        "xy_abs_integral_m_s": _absolute_time_integral(event_t, event_xy),
        "along_track_error_m": _metric_summary(event_along),
        "along_track_abs_integral_m_s": _absolute_time_integral(
            event_t, event_along
        ),
        "cross_track_error_m": _metric_summary(event_cross),
        "cross_track_abs_integral_m_s": _absolute_time_integral(
            event_t, event_cross
        ),
    }
    post_reverse = (timestamps >= reverse_command_t) & (
        timestamps <= reverse_command_t + REVERSAL_POST_WINDOW_S
    )
    post_reverse_along = (
        traces["error_x"][post_reverse] * old_direction[0]
        + traces["error_y"][post_reverse] * old_direction[1]
    )
    post_reverse_metrics = {
        "window_s": REVERSAL_POST_WINDOW_S,
        "along_track_error_m": _metric_summary(post_reverse_along),
        "along_track_abs_integral_m_s": _absolute_time_integral(
            timestamps[post_reverse], post_reverse_along
        ),
        "definition": (
            "first 2.0 s beginning at reverse command; absolute along-axis "
            "error integral uses the fixed outbound axis (absolute value makes "
            "axis sign immaterial), is reference-clock anchored, and never "
            "bridges gaps over 100 ms; formal use requires the offline "
            "primary-window support rule"
        ),
    }

    metrics = {
        "available": True,
        "phase_count": 2,
        "dwell_start_time_s": dwell_start_t,
        "reverse_command_time_s": reverse_command_t,
        "commanded_dwell_s": commanded_dwell_s,
        "metadata_expected_dwell_s": manifest.get("expected_dwell_duration_s"),
        "stop_delay_from_dwell_start_s": stop_delay,
        "reversal_delay_s": reversal_delay,
        "reversal_delay_definition": (
            "time after reverse command until Vicon velocity in the new "
            "direction is at least 10% of target for 0.20 s"
        ),
        "turnaround_overshoot_m": overshoot,
        "dwell_position_drift_m": dwell_drift,
        "midpoint_error_at_reverse_command_m": midpoint_error,
        "event_window": event_metrics,
        "post_reverse_response": post_reverse_metrics,
        "primary_paper_metric": (
            "post_reverse_response.along_track_abs_integral_m_s"
        ),
        "velocity_filter_definition": (
            "causal Vicon position chord over up to the trailing 0.20 s, using "
            "mapped source time when usable and receipt time only as fallback; "
            "previous-observation resampling has no future-position leakage"
        ),
        "timing_filter_delay_bound_s": VELOCITY_SMOOTHING_S,
        "pre_reversal_speed_error_m_s": _metric_summary(actual_pre - target_pre),
        "post_reversal_speed_error_m_s": _metric_summary(actual_post - target_post),
        "pre_reversal_cross_track_error_m": _metric_summary(
            traces["cross_error"][pre_moving]
        ),
        "post_reversal_cross_track_error_m": _metric_summary(
            traces["cross_error"][post_moving]
        ),
        "actual_closure_error_m": closure,
        "dwell_excluded_from_speed_error": True,
    }
    profile_traces = {
        "t": timestamps,
        "speed_new_direction": speed_new_direction,
        "actual_speed_norm": actual_speed_norm,
        "dwell_start_t": np.asarray([dwell_start_t]),
        "reverse_command_t": np.asarray([reverse_command_t]),
        "event_t": event_t,
        "event_along_error": event_along,
        "event_cross_error": event_cross,
        "event_xy_error": event_xy,
    }
    return metrics, profile_traces

def fitted_circle(reference_x: np.ndarray, reference_y: np.ndarray) -> tuple[float, float, float]:
    matrix = np.column_stack((2.0 * reference_x, 2.0 * reference_y, np.ones(reference_x.size)))
    target = reference_x**2 + reference_y**2
    center_x, center_y, constant = np.linalg.lstsq(matrix, target, rcond=None)[0]
    radius = math.sqrt(max(0.0, constant + center_x**2 + center_y**2))
    return float(center_x), float(center_y), float(radius)


def circle_metrics(
    reference: dict[str, np.ndarray],
    traces: dict[str, np.ndarray],
    manifest: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    moving_ref = np.hypot(reference["vx"], reference["vy"]) > 0.02
    center_x, center_y, fitted_radius = fitted_circle(
        reference["x"][moving_ref], reference["y"][moving_ref]
    )
    moving = traces["moving"]
    ref_x = traces["ref_x"][moving]
    ref_y = traces["ref_y"][moving]
    actual_x = traces["actual_x"][moving]
    actual_y = traces["actual_y"][moving]
    reference_radius = np.hypot(ref_x - center_x, ref_y - center_y)
    actual_radius = np.hypot(actual_x - center_x, actual_y - center_y)
    radial_error = actual_radius - reference_radius
    reference_angle = np.arctan2(ref_y - center_y, ref_x - center_x)
    actual_angle = np.arctan2(actual_y - center_y, actual_x - center_x)
    direction = str(manifest.get("direction") or "ccw").lower()
    direction_sign = -1.0 if direction == "cw" else 1.0
    phase_error = direction_sign * np.arctan2(
        np.sin(actual_angle - reference_angle),
        np.cos(actual_angle - reference_angle),
    )
    along_lag = fitted_radius * phase_error
    actual_unwrapped = np.unwrap(actual_angle)
    actual_progress_rad = direction_sign * float(
        actual_unwrapped[-1] - actual_unwrapped[0]
    )
    actual_laps_completed = actual_progress_rad / (2.0 * math.pi)
    commanded_laps = float(manifest.get("laps") or 1.0)
    lap_completion_fraction = (
        actual_laps_completed / commanded_laps if commanded_laps > 0.0 else None
    )
    metrics = {
        "center_x_m": center_x,
        "center_y_m": center_y,
        "radius_m": fitted_radius,
        "direction": direction,
        "radial_error_m": _metric_summary(radial_error),
        "phase_progress_error_rad": _metric_summary(phase_error),
        "along_lag_m": _metric_summary(along_lag),
        "commanded_laps": commanded_laps,
        "actual_laps_completed": actual_laps_completed,
        "lap_completion_fraction": lap_completion_fraction,
    }
    profile_traces = {
        "t": traces["moving_t"],
        "radial_error": radial_error,
        "along_lag": along_lag,
    }
    return metrics, profile_traces


def finite_json_value(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): finite_json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json_value(item) for item in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        result = float(value)
        return result if math.isfinite(result) else None
    return value


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(finite_json_value(value), indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def existing_exclusion(validation_path: Path) -> str:
    """Return a prior operator exclusion so re-analysis is idempotent."""
    if not validation_path.is_file():
        return ""
    try:
        document = json.loads(validation_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(
            f"cannot preserve exclusion from {validation_path}: {exc}"
        ) from exc
    if not isinstance(document, dict):
        raise RuntimeError(f"existing validation is not a JSON object: {validation_path}")
    value = document.get("exclude_reason", "")
    if not isinstance(value, str):
        raise RuntimeError(
            f"existing validation exclude_reason is not a string: {validation_path}"
        )
    return value


def _timestamp_metadata(timestamp_s: float, elapsed_origin_s: float) -> dict[str, Any]:
    """Describe one rosbag receipt timestamp for plots and video alignment."""
    utc_time = datetime.fromtimestamp(timestamp_s, timezone.utc)
    local_time = utc_time.astimezone()
    return {
        "elapsed_from_first_reference_s": float(timestamp_s - elapsed_origin_s),
        "receipt_time_unix_s": float(timestamp_s),
        "receipt_time_utc": utc_time.isoformat(timespec="milliseconds"),
        "receipt_time_local": local_time.isoformat(timespec="milliseconds"),
        "local_timezone": str(local_time.tzname() or local_time.tzinfo or "local"),
    }


def build_time_reference_metadata(
    bag_metadata: dict[str, Any],
    reference: dict[str, np.ndarray],
) -> dict[str, Any]:
    """Build one explicit receipt-clock timeline shared by every run plot."""
    timestamps = np.asarray(reference.get("t", np.asarray([])), dtype=float)
    vx = np.asarray(reference.get("vx", np.asarray([])), dtype=float)
    vy = np.asarray(reference.get("vy", np.asarray([])), dtype=float)
    if timestamps.size == 0:
        raise RuntimeError("cannot build plot time reference without reference samples")
    if vx.size != timestamps.size or vy.size != timestamps.size:
        raise RuntimeError("reference timestamp and velocity arrays disagree")
    if not np.all(np.isfinite(timestamps)) or np.any(np.diff(timestamps) <= 0.0):
        raise RuntimeError("plot time reference requires finite increasing timestamps")

    first_reference_s = float(timestamps[0])
    bag_start_document = bag_metadata.get("starting_time", {})
    bag_start_ns = (
        bag_start_document.get("nanoseconds_since_epoch")
        if isinstance(bag_start_document, dict)
        else None
    )
    try:
        bag_start_s = float(bag_start_ns) * 1e-9
    except (TypeError, ValueError):
        bag_start_s = math.nan

    speed = np.hypot(vx, vy)
    phases = []
    for phase_index, (start, last_active) in enumerate(
        _active_runs(speed > ACTIVE_TRANSLATION_SPEED_M_S), start=1
    ):
        # The stop marker is the first inactive command after an active run.
        # Preserve the last active sample as well so the half-open definition
        # is explicit and frame-level comparisons do not hide one sample.
        stop = min(last_active + 1, timestamps.size - 1)
        phases.append(
            {
                "phase_index": phase_index,
                "active_start": _timestamp_metadata(
                    float(timestamps[start]), first_reference_s
                ),
                "last_active_sample": _timestamp_metadata(
                    float(timestamps[last_active]), first_reference_s
                ),
                "active_stop": _timestamp_metadata(
                    float(timestamps[stop]), first_reference_s
                ),
            }
        )

    return {
        "clock": "rosbag storage/receipt timestamp",
        "elapsed_origin": "first /reference_trajectory receipt",
        "first_reference": _timestamp_metadata(
            first_reference_s, first_reference_s
        ),
        "last_reference": _timestamp_metadata(
            float(timestamps[-1]), first_reference_s
        ),
        "bag_start": (
            _timestamp_metadata(bag_start_s, first_reference_s)
            if math.isfinite(bag_start_s)
            else None
        ),
        "active_motion_threshold_m_s": ACTIVE_TRANSLATION_SPEED_M_S,
        "active_motion_interval_definition": (
            "active_start is the first reference sample above the threshold; "
            "active_stop is the first sample at or below the threshold after "
            "that run, and last_active_sample preserves the preceding sample"
        ),
        "active_motion_phases": phases,
        "video_alignment_formula": (
            "video_time_s = plot_elapsed_s + video_motion_onset_s "
            "- plot_motion_onset_s"
        ),
        "plot_motion_onset_s": (
            phases[0]["active_start"]["elapsed_from_first_reference_s"]
            if phases
            else None
        ),
        "video_alignment_note": (
            "Use elapsed seconds directly only after aligning the video to a "
            "shared-clock timestamp or a visible synchronization event, such "
            "as first commanded/visible motion. External camera start time is "
            "not recorded by this rosbag."
        ),
    }


VIDEO_TIMELINE_FIELDS = (
    "sample_index",
    "phase",
    "elapsed_from_first_reference_s",
    "elapsed_from_first_motion_s",
    "reference_receipt_time_unix_s",
    "reference_receipt_time_local",
    "reference_receipt_time_utc",
    "reference_x_m",
    "reference_y_m",
    "source_aligned_actual_x_m",
    "source_aligned_actual_y_m",
    "receipt_aligned_actual_x_diagnostic_m",
    "receipt_aligned_actual_y_diagnostic_m",
    "source_aligned_x_error_m",
    "source_aligned_y_error_m",
    "source_aligned_xy_error_m",
    "receipt_aligned_xy_error_diagnostic_m",
    "source_aligned_along_track_error_m",
    "source_aligned_cross_track_error_m",
    "receipt_aligned_along_track_error_diagnostic_m",
    "receipt_aligned_cross_track_error_diagnostic_m",
    "commanded_tangent_speed_m_s",
    "source_time_tangent_speed_m_s",
    "receipt_time_tangent_speed_diagnostic_m_s",
    "source_time_speed_error_m_s",
    "receipt_time_speed_error_diagnostic_m_s",
    "relative_vicon_delivery_jitter_s",
)


def _timeline_number(value: float) -> str:
    """Format one finite timeline number; unavailable values remain blank."""
    number = float(value)
    return f"{number:.9f}" if math.isfinite(number) else ""


def write_video_timeline(
    path: Path,
    traces: dict[str, np.ndarray],
    time_reference: dict[str, Any],
) -> Path:
    """Write exact plot/video lookup rows instead of requiring pixel estimates."""
    timestamps = np.asarray(traces["t"], dtype=float)
    sample_count = int(timestamps.size)
    moving = np.asarray(traces["moving"], dtype=bool)
    if moving.size != sample_count:
        raise RuntimeError("timeline moving mask does not match tracking samples")
    t0 = float(time_reference["first_reference"]["receipt_time_unix_s"])
    moving_indices = np.flatnonzero(moving)
    motion_start_t = (
        float(timestamps[moving_indices[0]]) if moving_indices.size else t0
    )

    def full_motion_values(key: str, default: float = math.nan) -> np.ndarray:
        values = np.asarray(traces[key], dtype=float)
        if values.size != moving_indices.size:
            raise RuntimeError(f"timeline trace {key!r} does not match active samples")
        result = np.full(sample_count, default, dtype=float)
        result[moving] = values
        return result

    command_speed = full_motion_values("command_speed", default=0.0)
    primary_speed = full_motion_values("primary_actual_tangent_speed")
    receipt_speed = full_motion_values("receipt_actual_tangent_speed")
    primary_along = full_motion_values("primary_along_error")
    primary_cross = full_motion_values("primary_cross_error")
    receipt_along = full_motion_values("receipt_along_error")
    receipt_cross = full_motion_values("receipt_cross_error")
    primary_speed_error = primary_speed - command_speed
    receipt_speed_error = receipt_speed - command_speed
    reference_x = np.asarray(traces["ref_x"], dtype=float)
    reference_y = np.asarray(traces["ref_y"], dtype=float)
    primary_x = np.asarray(traces["actual_x"], dtype=float)
    primary_y = np.asarray(traces["actual_y"], dtype=float)
    receipt_x = np.asarray(traces["receipt_actual_x"], dtype=float)
    receipt_y = np.asarray(traces["receipt_actual_y"], dtype=float)
    primary_error_x = np.asarray(traces["primary_error_x"], dtype=float)
    primary_error_y = np.asarray(traces["primary_error_y"], dtype=float)
    primary_error_xy = np.asarray(traces["primary_error_xy"], dtype=float)
    receipt_error_xy = np.asarray(traces["receipt_error_xy"], dtype=float)
    delivery_jitter = np.asarray(traces["source_receipt_residual"], dtype=float)
    full_series = (
        reference_x,
        reference_y,
        primary_x,
        primary_y,
        receipt_x,
        receipt_y,
        primary_error_x,
        primary_error_y,
        primary_error_xy,
        receipt_error_xy,
        delivery_jitter,
    )
    if any(values.size != sample_count for values in full_series):
        raise RuntimeError("timeline full-lifecycle traces have inconsistent sizes")

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=VIDEO_TIMELINE_FIELDS)
        writer.writeheader()
        for index, timestamp_s in enumerate(timestamps):
            if moving[index]:
                phase = "active_motion"
            elif not moving_indices.size or index < moving_indices[0]:
                phase = "startup_hold"
            elif index > moving_indices[-1]:
                phase = "final_hold"
            else:
                phase = "internal_dwell"
            utc_time = datetime.fromtimestamp(float(timestamp_s), timezone.utc)
            local_time = utc_time.astimezone()
            writer.writerow(
                {
                    "sample_index": index,
                    "phase": phase,
                    "elapsed_from_first_reference_s": _timeline_number(
                        timestamp_s - t0
                    ),
                    "elapsed_from_first_motion_s": _timeline_number(
                        timestamp_s - motion_start_t
                    ),
                    "reference_receipt_time_unix_s": _timeline_number(timestamp_s),
                    "reference_receipt_time_local": local_time.isoformat(
                        timespec="milliseconds"
                    ),
                    "reference_receipt_time_utc": utc_time.isoformat(
                        timespec="milliseconds"
                    ),
                    "reference_x_m": _timeline_number(reference_x[index]),
                    "reference_y_m": _timeline_number(reference_y[index]),
                    "source_aligned_actual_x_m": _timeline_number(primary_x[index]),
                    "source_aligned_actual_y_m": _timeline_number(primary_y[index]),
                    "receipt_aligned_actual_x_diagnostic_m": _timeline_number(
                        receipt_x[index]
                    ),
                    "receipt_aligned_actual_y_diagnostic_m": _timeline_number(
                        receipt_y[index]
                    ),
                    "source_aligned_x_error_m": _timeline_number(
                        primary_error_x[index]
                    ),
                    "source_aligned_y_error_m": _timeline_number(
                        primary_error_y[index]
                    ),
                    "source_aligned_xy_error_m": _timeline_number(
                        primary_error_xy[index]
                    ),
                    "receipt_aligned_xy_error_diagnostic_m": _timeline_number(
                        receipt_error_xy[index]
                    ),
                    "source_aligned_along_track_error_m": _timeline_number(
                        primary_along[index]
                    ),
                    "source_aligned_cross_track_error_m": _timeline_number(
                        primary_cross[index]
                    ),
                    "receipt_aligned_along_track_error_diagnostic_m": (
                        _timeline_number(receipt_along[index])
                    ),
                    "receipt_aligned_cross_track_error_diagnostic_m": (
                        _timeline_number(receipt_cross[index])
                    ),
                    "commanded_tangent_speed_m_s": _timeline_number(
                        command_speed[index]
                    ),
                    "source_time_tangent_speed_m_s": _timeline_number(
                        primary_speed[index]
                    ),
                    "receipt_time_tangent_speed_diagnostic_m_s": _timeline_number(
                        receipt_speed[index]
                    ),
                    "source_time_speed_error_m_s": _timeline_number(
                        primary_speed_error[index]
                    ),
                    "receipt_time_speed_error_diagnostic_m_s": _timeline_number(
                        receipt_speed_error[index]
                    ),
                    "relative_vicon_delivery_jitter_s": _timeline_number(
                        delivery_jitter[index]
                    ),
                }
            )
    return path


def elapsed_tick_spacing(duration_s: float) -> tuple[float, float]:
    """Choose readable major/minor elapsed-time ticks, including 1 s detail."""
    duration_s = max(float(duration_s), 0.0)
    major_candidates = (
        0.5,
        1.0,
        2.0,
        5.0,
        10.0,
        15.0,
        20.0,
        30.0,
        60.0,
        120.0,
        300.0,
        600.0,
    )
    target_major = duration_s / 14.0
    major = next(
        (candidate for candidate in major_candidates if candidate >= target_major),
        major_candidates[-1],
    )
    if major in (2.0, 20.0):
        minor = major / 4.0
    elif major in (15.0, 30.0):
        minor = major / 3.0 if major == 15.0 else major / 6.0
    else:
        minor = major / 5.0
    return major, minor


def _elapsed_axis_label(
    time_reference: dict[str, Any], major_s: float, minor_s: float
) -> str:
    first_reference = time_reference["first_reference"]
    bag_start = time_reference.get("bag_start")
    bag_text = "bag start unavailable"
    if isinstance(bag_start, dict):
        bag_text = (
            "bag start "
            f"t={float(bag_start['elapsed_from_first_reference_s']):+.3f} s"
        )
    return (
        "Elapsed from first reference receipt (s)  "
        f"[major {major_s:g} s; minor {minor_s:g} s]\n"
        f"t=0: {first_reference['receipt_time_local']} "
        f"{first_reference['local_timezone']} | "
        f"{first_reference['receipt_time_utc']} UTC | {bag_text}"
    )


def _configure_elapsed_axis(
    axis: Any, time_reference: dict[str, Any], show_label: bool = True
) -> None:
    """Apply a consistent, video-friendly elapsed-time axis to one plot."""
    from matplotlib.ticker import MultipleLocator

    duration_s = float(
        time_reference["last_reference"]["elapsed_from_first_reference_s"]
    )
    major_s, minor_s = elapsed_tick_spacing(duration_s)
    axis.set_xlim(0.0, max(duration_s, minor_s))
    axis.xaxis.set_major_locator(MultipleLocator(major_s))
    axis.xaxis.set_minor_locator(MultipleLocator(minor_s))
    axis.tick_params(axis="x", which="minor", length=3.0)
    axis.grid(True, which="major", alpha=0.35)
    axis.grid(True, which="minor", axis="x", alpha=0.14, linewidth=0.65)
    if show_label:
        axis.set_xlabel(_elapsed_axis_label(time_reference, major_s, minor_s))


def _add_motion_phase_markers(
    axes: Any, time_reference: dict[str, Any], legend_axis: Any
) -> None:
    """Draw the same command-motion boundaries on all supplied axes."""
    axis_list = list(axes) if isinstance(axes, (list, tuple, np.ndarray)) else [axes]
    phases = time_reference.get("active_motion_phases", [])
    for axis in axis_list:
        for phase_index, phase in enumerate(phases):
            start_s = float(phase["active_start"]["elapsed_from_first_reference_s"])
            stop_s = float(phase["active_stop"]["elapsed_from_first_reference_s"])
            axis.axvline(
                start_s,
                color="#238b45",
                linestyle=":",
                linewidth=1.15,
                alpha=0.9,
                label=(
                    "command motion start"
                    if axis is legend_axis and phase_index == 0
                    else "_nolegend_"
                ),
            )
            axis.axvline(
                stop_s,
                color="#c43b32",
                linestyle=":",
                linewidth=1.15,
                alpha=0.9,
                label=(
                    "command motion stop"
                    if axis is legend_axis and phase_index == 0
                    else "_nolegend_"
                ),
            )


def write_plots(
    output: Path,
    bag: Path,
    manifest: dict[str, Any],
    traces: dict[str, np.ndarray],
    profile_metrics: dict[str, Any] | None,
    profile_traces: dict[str, np.ndarray] | None,
    time_reference: dict[str, Any],
) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    artifacts: list[Path] = []
    profile = str(manifest.get("profile", "unknown"))

    path_plot = output / "path_reference.png"
    fig, axis = plt.subplots(figsize=(7.5, 7.0))
    axis.plot(
        traces["ref_x"], traces["ref_y"], "k--", linewidth=2.0, label="Reference"
    )
    axis.plot(
        traces["actual_x"],
        traces["actual_y"],
        color="#1764ab",
        linewidth=2.0,
        label="Vicon base",
    )
    axis.scatter(
        [traces["ref_x"][0]], [traces["ref_y"][0]], color="#238b45", label="Start"
    )
    axis.scatter(
        [traces["actual_x"][-1]],
        [traces["actual_y"][-1]],
        color="#c43b32",
        marker="x",
        label="Actual end",
    )
    axis.set_title(f"{profile} — Vicon tracking\n{bag.name}")
    axis.set_xlabel("Vicon x (m)")
    axis.set_ylabel("Vicon y (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.35)
    axis.legend()
    fig.tight_layout()
    fig.savefig(path_plot, dpi=170)
    plt.close(fig)
    artifacts.append(path_plot)

    tracking_plot = output / "tracking_error.png"
    fig, axes = plt.subplots(2, 1, figsize=(10.5, 7.0), sharex=True)
    t0 = float(time_reference["first_reference"]["receipt_time_unix_s"])
    axes[0].plot(
        traces["t"] - t0,
        1000.0 * traces["primary_error_xy"],
        color="#1764ab",
        linewidth=1.8,
        label="source-time aligned (primary)",
    )
    axes[0].plot(
        traces["t"] - t0,
        1000.0 * traces["receipt_error_xy"],
        color="#7a7a7a",
        linestyle="--",
        linewidth=1.1,
        alpha=0.85,
        label="receipt-time aligned (delivery diagnostic)",
    )
    axes[0].set_ylabel("XY error (mm)")
    moving_t = traces["moving_t"] - t0
    axes[1].plot(
        moving_t,
        1000.0 * traces["along_error"],
        label="along-track",
        color="#e88b00",
    )
    axes[1].plot(
        moving_t,
        1000.0 * traces["cross_error"],
        label="cross-track",
        color="#238b45",
    )
    axes[1].set_ylabel("Signed error (mm)")
    _add_motion_phase_markers(axes, time_reference, axes[1])
    _configure_elapsed_axis(axes[0], time_reference, show_label=False)
    _configure_elapsed_axis(axes[1], time_reference)
    axes[0].legend(ncol=2)
    axes[1].legend(ncol=2)
    fig.suptitle(f"{profile} tracking error")
    fig.tight_layout()
    fig.savefig(tracking_plot, dpi=170)
    plt.close(fig)
    artifacts.append(tracking_plot)

    speed_plot = output / "speed_tracking.png"
    fig, axis = plt.subplots(figsize=(10.5, 4.8))
    axis.plot(
        moving_t,
        traces["command_speed"],
        "k--",
        linewidth=1.8,
        label="Commanded tangent speed",
    )
    axis.plot(
        moving_t,
        traces["primary_actual_tangent_speed"],
        color="#1764ab",
        linewidth=1.8,
        label="Vicon source-time speed (primary)",
    )
    axis.plot(
        moving_t,
        traces["receipt_actual_tangent_speed"],
        color="#7a7a7a",
        linestyle="--",
        linewidth=1.1,
        alpha=0.85,
        label="Vicon receipt-time speed (delivery diagnostic)",
    )
    axis.set_ylabel("Speed (m/s)")
    axis.set_title(f"{profile} speed tracking")
    _add_motion_phase_markers(axis, time_reference, axis)
    _configure_elapsed_axis(axis, time_reference)
    axis.legend(ncol=2)
    fig.tight_layout()
    fig.savefig(speed_plot, dpi=170)
    plt.close(fig)
    artifacts.append(speed_plot)

    delivery_plot = output / "vicon_delivery_timing.png"
    fig, axis = plt.subplots(figsize=(10.5, 4.8))
    delivery_residual = np.asarray(
        traces["source_receipt_residual"], dtype=float
    )
    finite_delivery = np.isfinite(delivery_residual)
    if np.any(finite_delivery):
        axis.plot(
            (traces["t"] - t0)[finite_delivery],
            1000.0 * delivery_residual[finite_delivery],
            color="#7a3db8",
            linewidth=1.25,
            label="receipt minus mapped source time",
        )
        axis.axhline(0.0, color="black", linewidth=0.9)
        axis.axhline(50.0, color="#c43b32", linestyle="--", linewidth=1.0)
        axis.axhline(-50.0, color="#c43b32", linestyle="--", linewidth=1.0)
    else:
        axis.text(
            0.5,
            0.5,
            "Vicon source stamps unavailable; receipt-time fallback used",
            ha="center",
            va="center",
            transform=axis.transAxes,
        )
    axis.set_ylabel("Relative delivery residual (ms)")
    axis.set_title(
        f"{profile} Vicon delivery timing\n"
        "relative jitter only — unknown clock offset prevents absolute latency"
    )
    _add_motion_phase_markers(axis, time_reference, axis)
    _configure_elapsed_axis(axis, time_reference)
    if np.any(finite_delivery):
        axis.legend(ncol=2)
    fig.tight_layout()
    fig.savefig(delivery_plot, dpi=170)
    plt.close(fig)
    artifacts.append(delivery_plot)

    if profile_metrics and profile_traces and profile in TRANSLATION_PROFILES:
        profile_plot = output / "translation_response.png"
        fig, axes = plt.subplots(2, 1, figsize=(10.0, 6.8), sharex=True)
        response_t = profile_traces["t"] - t0
        axes[0].plot(
            response_t,
            1000.0 * profile_traces["cross_axis_error"],
            color="#238b45",
        )
        axes[0].axhline(0.0, color="black", linewidth=0.9)
        axes[0].set_ylabel(
            f"{profile_metrics['cross_axis']} error (mm)"
        )
        if "actual_speed_smoothed" in profile_traces:
            axes[1].plot(
                response_t,
                traces["command_speed"],
                "k--",
                label="commanded",
            )
            axes[1].plot(
                response_t,
                profile_traces["actual_speed_smoothed"],
                color="#1764ab",
                label="Vicon (single 0.20 s causal filter)",
            )
        axes[1].set_ylabel("Along-axis speed (m/s)")
        _add_motion_phase_markers(axes, time_reference, axes[1])
        _configure_elapsed_axis(axes[0], time_reference, show_label=False)
        _configure_elapsed_axis(axes[1], time_reference)
        axes[1].legend(ncol=2)
        fig.suptitle("Straight translation response")
        fig.tight_layout()
        fig.savefig(profile_plot, dpi=170)
        plt.close(fig)
        artifacts.append(profile_plot)
    elif profile_metrics and profile_traces and profile in REVERSAL_PROFILES:
        profile_plot = output / "reversal_response.png"
        fig, axis = plt.subplots(figsize=(10.0, 4.8))
        response_t = profile_traces["t"] - t0
        axis.plot(
            response_t,
            profile_traces["actual_speed_norm"],
            color="#1764ab",
            label="Vicon speed magnitude",
        )
        axis.plot(
            response_t,
            profile_traces["speed_new_direction"],
            color="#e88b00",
            label="Vicon speed in reverse direction",
        )
        for key, label in (
            ("dwell_start_t", "dwell starts"),
            ("reverse_command_t", "reverse command"),
        ):
            axis.axvline(
                float(profile_traces[key][0] - t0),
                linestyle="--",
                linewidth=1.0,
                label=label,
            )
        axis.set_ylabel("Speed (m/s)")
        axis.set_title("Forward-dwell-reverse response")
        _add_motion_phase_markers(axis, time_reference, axis)
        _configure_elapsed_axis(axis, time_reference)
        axis.legend(ncol=2)
        fig.tight_layout()
        fig.savefig(profile_plot, dpi=170)
        plt.close(fig)
        artifacts.append(profile_plot)
    elif profile_metrics and profile.startswith("triangle"):
        corners = profile_metrics.get("corners", [])
        if corners:
            profile_plot = output / "triangle_metrics.png"
            labels = [
                f"{item['index']}\n"
                f"{item['interior_angle_deg']:.0f}° interior\n"
                f"{item['commanded_turn_angle_deg']:.0f}° direction change"
                for item in corners
            ]
            errors = [1000.0 * item["time_aligned_error_m"] for item in corners]
            closest = [1000.0 * item["closest_distance_m"] for item in corners]
            positions = np.arange(len(corners))
            fig, axis = plt.subplots(figsize=(8.0, 4.8))
            axis.bar(positions - 0.18, errors, width=0.36, label="At commanded corner")
            axis.bar(positions + 0.18, closest, width=0.36, label="Closest approach")
            axis.set_xticks(positions, labels)
            axis.set_xlabel("Triangle corner geometry")
            axis.set_ylabel("Distance to corner (mm)")
            axis.set_title("Triangle corner tracking")
            axis.grid(True, axis="y", alpha=0.35)
            axis.legend()
            fig.tight_layout()
            fig.savefig(profile_plot, dpi=170)
            plt.close(fig)
            artifacts.append(profile_plot)
    elif profile_metrics and profile_traces and profile.startswith("circle"):
        profile_plot = output / "circle_metrics.png"
        fig, axes = plt.subplots(2, 1, figsize=(10.0, 6.8), sharex=True)
        circle_t = profile_traces["t"] - t0
        axes[0].plot(circle_t, 1000.0 * profile_traces["radial_error"], color="#1764ab")
        axes[0].axhline(0.0, color="black", linewidth=0.9)
        axes[0].set_ylabel("Radial error (mm)")
        axes[1].plot(circle_t, 1000.0 * profile_traces["along_lag"], color="#e88b00")
        axes[1].axhline(0.0, color="black", linewidth=0.9)
        axes[1].set_ylabel("Along-path lag (mm)")
        _add_motion_phase_markers(axes, time_reference, axes[1])
        _configure_elapsed_axis(axes[0], time_reference, show_label=False)
        _configure_elapsed_axis(axes[1], time_reference)
        axes[1].legend(ncol=2)
        fig.suptitle("Circle radial and phase tracking")
        fig.tight_layout()
        fig.savefig(profile_plot, dpi=170)
        plt.close(fig)
        artifacts.append(profile_plot)

    return artifacts


def _nested(metrics: dict[str, Any], *keys: str) -> Any:
    value: Any = metrics
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _serialize_explicit_bool(value: Any) -> str:
    """Serialize only an explicitly recorded boolean provenance value."""
    return str(value).lower() if isinstance(value, bool) else ""


def build_summary_row(
    bag: Path,
    manifest: dict[str, Any],
    validation: dict[str, Any],
    metrics: dict[str, Any],
) -> dict[str, Any]:
    lifecycle = validation["reference_lifecycle"]
    vicon = validation["vicon_stream"]
    source_stamp = validation.get("vicon_source_stamp_quality", {})
    pose_quality = validation["vicon_pose_quality"]
    primary_quality = validation.get("vicon_primary_window_quality", {})
    primary_windows = primary_quality.get("windows", [])
    tracking = metrics["tracking"]
    translation = metrics.get("translation", {})
    reversal = metrics.get("reversal", {})
    actuator = metrics.get("actuator_limits", {})
    command_support = actuator.get("command_pair_support", {})
    status_support = actuator.get("wheel_status_support", {})
    imu = metrics.get("imu_disturbance_proxies", {})
    triangle = metrics.get("triangle", {})
    circle = metrics.get("circle", {})
    turret_setting = manifest.get("turret_control_enabled")
    serialized_turret_setting = _serialize_explicit_bool(turret_setting)
    return {
        "run_id": manifest.get("run_id", bag.name),
        "planned_id": manifest.get("planned_id", ""),
        "planned_order": manifest.get("planned_order", ""),
        "gate_added": manifest.get("gate_added", ""),
        "matched_block_id": manifest.get("matched_block_id", ""),
        "hardware_block_id": manifest.get("hardware_block_id", ""),
        "bag_path": str(bag),
        "analyzed_at_utc": validation["analyzed_at_utc"],
        "status": validation["status"],
        "valid_for_aggregate": str(validation["valid_for_aggregate"]).lower(),
        "profile": manifest.get("profile", ""),
        "profile_sha256": manifest.get("profile_sha256", ""),
        "analysis_revision": metrics.get("analysis_revision", ""),
        "analyzer_sha256": metrics.get("analyzer_sha256", ""),
        "study_stack_contract_version": manifest.get(
            "study_stack_contract_version", ""
        ),
        "controller_config_sha256": manifest.get(
            "controller_config_sha256", ""
        ),
        "wheel_speed_limit_rad_s": manifest.get(
            "wheel_speed_limit_rad_s", ""
        ),
        "turret_control_enabled": serialized_turret_setting,
        "wheel_status_schema": actuator.get("wheel_status_schema", ""),
        "kind": manifest.get("kind", ""),
        "speed_m_s": manifest.get("speed_m_s", ""),
        "origin_x_m": manifest.get("origin_x_m", ""),
        "origin_y_m": manifest.get("origin_y_m", ""),
        "captured_start_yaw_rad": manifest.get("captured_start_yaw_rad", ""),
        "nominal_start_base_yaw_rad": manifest.get(
            "nominal_start_base_yaw_rad", ""
        ),
        "start_base_yaw_tolerance_rad": manifest.get(
            "start_base_yaw_tolerance_rad", ""
        ),
        "caster_type": manifest.get("caster_type", ""),
        "terrain": manifest.get("terrain", ""),
        "initial_caster_orientation_deg": manifest.get(
            "initial_caster_orientation_deg", ""
        ),
        "repetition": manifest.get("repetition", ""),
        "video_id": manifest.get("video_id", ""),
        "notes": manifest.get("notes", ""),
        "exclude_reason": validation.get("exclude_reason", ""),
        "reference_duration_s": lifecycle.get("duration_s"),
        "reference_completion_fraction": lifecycle.get("completion_fraction"),
        "reference_rate_hz": lifecycle.get("mean_rate_hz"),
        "reference_max_gap_s": lifecycle.get("max_gap_s"),
        "final_zero_s": lifecycle.get("final_zero_s"),
        "vicon_coverage_fraction": tracking.get("coverage_fraction"),
        "vicon_first_reference_match_dt_s": tracking.get(
            "first_reference_match_dt_s"
        ),
        "vicon_last_reference_match_dt_s": tracking.get(
            "last_reference_match_dt_s"
        ),
        "vicon_rate_hz": vicon.get("mean_rate_hz"),
        "vicon_max_gap_s": vicon.get("max_gap_s"),
        "vicon_gap_over_100ms_count": vicon.get("gap_over_100ms_count"),
        "vicon_source_stamp_valid": str(
            source_stamp.get("valid", False)
        ).lower(),
        "vicon_source_stamp_sample_count": source_stamp.get(
            "sample_count", ""
        ),
        "vicon_source_stamp_nonpositive_count": source_stamp.get(
            "nonpositive_count", ""
        ),
        "vicon_source_stamp_duplicate_count": source_stamp.get(
            "duplicate_count", ""
        ),
        "vicon_source_stamp_reversed_count": source_stamp.get(
            "reversed_count", ""
        ),
        "vicon_source_stamp_max_gap_s": source_stamp.get("max_gap_s", ""),
        "vicon_source_stamp_failure_reason": source_stamp.get(
            "failure_reason", ""
        ),
        "primary_window_quality_valid": str(
            primary_quality.get("valid", False)
        ).lower(),
        "primary_window_min_coverage_fraction": (
            min(window["coverage_fraction"] for window in primary_windows)
            if primary_windows
            else ""
        ),
        "primary_window_max_effective_gap_s": (
            max(
                window["max_effective_vicon_gap_s"]
                for window in primary_windows
                if window["max_effective_vicon_gap_s"] is not None
            )
            if any(
                window["max_effective_vicon_gap_s"] is not None
                for window in primary_windows
            )
            else ""
        ),
        "primary_window_failure_reason": "; ".join(
            primary_quality.get("failure_reasons", [])
        ),
        "vicon_z_envelope_violation_count": pose_quality.get(
            "z_outside_0_25_to_0_40m_count"
        ),
        "vicon_tilt_violation_count": pose_quality.get(
            "tilt_over_0_35rad_count"
        ),
        "xy_rmse_m": _nested(tracking, "xy_error_m", "rmse"),
        "xy_p95_m": _nested(tracking, "xy_error_m", "p95_abs"),
        "xy_max_m": _nested(tracking, "xy_error_m", "abs_max"),
        "active_time_weighted_xy_error_mean_m": tracking.get(
            "active_time_weighted_xy_error_mean_m"
        ),
        "final_xy_error_m": tracking.get("final_hold_xy_error_m"),
        "cross_track_rmse_m": _nested(
            tracking, "cross_track_error_m", "rmse"
        ),
        "along_track_rmse_m": _nested(
            tracking, "along_track_error_m", "rmse"
        ),
        "startup_hold_xy_jitter_p95_m": _nested(
            tracking, "startup_hold_xy_jitter", "displacement_m", "p95_abs"
        ),
        "final_hold_xy_jitter_p95_m": _nested(
            tracking, "final_hold_xy_jitter", "displacement_m", "p95_abs"
        ),
        "commanded_speed_mean_m_s": tracking.get(
            "commanded_speed_mean_m_s"
        ),
        "actual_tangent_speed_mean_m_s": tracking.get(
            "actual_tangent_speed_mean_m_s"
        ),
        "speed_bias_m_s": _nested(tracking, "speed_error_m_s", "mean"),
        "speed_rmse_m_s": _nested(tracking, "speed_error_m_s", "rmse"),
        "turret_yaw_rmse_rad": _nested(
            tracking, "turret_yaw_error_rad", "rmse"
        ),
        "turret_reference_difference_rmse_rad": _nested(
            tracking, "turret_reference_difference_rad", "rmse"
        ),
        "base_yaw_drift_rmse_rad": _nested(
            tracking, "base_yaw_drift_rad", "rmse"
        ),
        "base_yaw_net_change_rad": tracking.get(
            "base_yaw_net_change_rad", ""
        ),
        "translation_axis": translation.get("command_axis", ""),
        "translation_cross_axis": translation.get("cross_axis", ""),
        "translation_cross_axis_bias_m": _nested(
            translation, "cross_axis_error_m", "mean"
        ),
        "translation_cross_axis_rmse_m": _nested(
            translation, "cross_axis_error_m", "rmse"
        ),
        "translation_onset_along_rmse_m": _nested(
            translation, "onset_response", "along_track_error_m", "rmse"
        ),
        "translation_onset_along_peak_m": _nested(
            translation, "onset_response", "along_track_error_m", "abs_max"
        ),
        "translation_onset_along_abs_integral_m_s": _nested(
            translation, "onset_response", "along_track_abs_integral_m_s"
        ),
        "speed_rise_time_s": _nested(
            translation, "speed_response", "rise_time_s"
        ),
        "speed_settling_time_s": _nested(
            translation, "speed_response", "settling_time_s"
        ),
        "steady_state_speed_bias_m_s": _nested(
            translation,
            "speed_response",
            "steady_state_speed_error_m_s",
            "mean",
        ),
        "speed_overshoot_m_s": _nested(
            translation, "speed_response", "overshoot_m_s"
        ),
        "reversal_commanded_dwell_s": reversal.get("commanded_dwell_s", ""),
        "reversal_stop_delay_s": reversal.get(
            "stop_delay_from_dwell_start_s", ""
        ),
        "reversal_delay_s": reversal.get("reversal_delay_s", ""),
        "reversal_overshoot_m": reversal.get("turnaround_overshoot_m", ""),
        "reversal_dwell_drift_m": reversal.get("dwell_position_drift_m", ""),
        "reversal_midpoint_error_m": reversal.get(
            "midpoint_error_at_reverse_command_m", ""
        ),
        "reversal_event_xy_rmse_m": _nested(
            reversal, "event_window", "xy_error_m", "rmse"
        ),
        "reversal_event_xy_peak_m": _nested(
            reversal, "event_window", "xy_error_m", "abs_max"
        ),
        "reversal_event_xy_abs_integral_m_s": _nested(
            reversal, "event_window", "xy_abs_integral_m_s"
        ),
        "reversal_event_along_rmse_m": _nested(
            reversal, "event_window", "along_track_error_m", "rmse"
        ),
        "reversal_event_along_peak_m": _nested(
            reversal, "event_window", "along_track_error_m", "abs_max"
        ),
        "reversal_event_along_abs_integral_m_s": _nested(
            reversal, "event_window", "along_track_abs_integral_m_s"
        ),
        "reversal_post_reverse_along_abs_integral_m_s": _nested(
            reversal,
            "post_reverse_response",
            "along_track_abs_integral_m_s",
        ),
        "reversal_event_cross_rmse_m": _nested(
            reversal, "event_window", "cross_track_error_m", "rmse"
        ),
        "reversal_event_cross_peak_m": _nested(
            reversal, "event_window", "cross_track_error_m", "abs_max"
        ),
        "reversal_event_cross_abs_integral_m_s": _nested(
            reversal, "event_window", "cross_track_abs_integral_m_s"
        ),
        "reversal_pre_speed_bias_m_s": _nested(
            reversal, "pre_reversal_speed_error_m_s", "mean"
        ),
        "reversal_post_speed_bias_m_s": _nested(
            reversal, "post_reversal_speed_error_m_s", "mean"
        ),
        "reversal_closure_error_m": reversal.get("actual_closure_error_m", ""),
        "actuator_command_support_valid": str(
            command_support.get("valid", False)
        ).lower(),
        "actuator_command_support_coverage_fraction": command_support.get(
            "coverage_fraction", ""
        ),
        "actuator_command_support_max_effective_gap_s": command_support.get(
            "max_effective_topic_gap_s", ""
        ),
        "actuator_command_support_reason": command_support.get(
            "failure_reason", ""
        ),
        "wheel_status_support_valid": str(
            status_support.get("valid", False)
        ).lower(),
        "wheel_status_support_coverage_fraction": status_support.get(
            "coverage_fraction", ""
        ),
        "wheel_status_support_max_effective_gap_s": status_support.get(
            "max_effective_topic_gap_s", ""
        ),
        "wheel_status_support_reason": status_support.get(
            "failure_reason", ""
        ),
        "actuator_pair_cap_fraction": actuator.get(
            "active_pair_cap_fraction", ""
        ),
        "firmware_saturation_fraction": actuator.get(
            "firmware_saturation_fraction", ""
        ),
        "firmware_saturation_count": actuator.get(
            "firmware_saturation_sample_count", ""
        ),
        "wheel_rpm_error_bias": _nested(actuator, "wheel_rpm_error", "mean"),
        "wheel_rpm_error_rmse": _nested(actuator, "wheel_rpm_error", "rmse"),
        "wheel_status_source_counts_json": (
            json.dumps(
                actuator.get("wheel_status_source_value_counts"),
                sort_keys=True,
                separators=(",", ":"),
            )
            if actuator.get("wheel_status_source_value_counts") is not None
            else ""
        ),
        "wheel_status_communications_timeout_count": actuator.get(
            "communications_timeout_source_sample_count", ""
        ),
        "wheel_status_communications_timeout_fraction": actuator.get(
            "communications_timeout_source_fraction", ""
        ),
        "imu_linear_accel_norm_rms_m_s2": _nested(
            imu, "linear_acceleration_norm_m_s2", "rmse"
        ),
        "imu_linear_accel_norm_p95_m_s2": _nested(
            imu, "linear_acceleration_norm_m_s2", "p95_abs"
        ),
        "imu_high_frequency_accel_rms_m_s2": _nested(
            imu, "high_frequency_acceleration_residual_m_s2", "rmse"
        ),
        "imu_angular_rate_norm_rms_rad_s": _nested(
            imu, "angular_rate_norm_rad_s", "rmse"
        ),
        "imu_angular_rate_norm_p95_rad_s": _nested(
            imu, "angular_rate_norm_rad_s", "p95_abs"
        ),
        "triangle_corner_count": triangle.get("corner_count", ""),
        "triangle_corner_error_mean_m": triangle.get(
            "corner_error_mean_m", ""
        ),
        "triangle_corner_error_max_m": triangle.get("corner_error_max_m", ""),
        "triangle_leg_cross_track_rmse_mean_m": triangle.get(
            "leg_cross_track_rmse_mean_m", ""
        ),
        "circle_radius_m": circle.get("radius_m", ""),
        "circle_radial_bias_m": _nested(circle, "radial_error_m", "mean"),
        "circle_radial_rmse_m": _nested(circle, "radial_error_m", "rmse"),
        "circle_radial_max_m": _nested(circle, "radial_error_m", "abs_max"),
        "circle_along_lag_mean_m": _nested(circle, "along_lag_m", "mean"),
        "circle_lap_completion_fraction": circle.get(
            "lap_completion_fraction", ""
        ),
        "warning_count": len(validation.get("warnings", [])),
    }


def write_csv_rows(path: Path, fields: tuple[str, ...], rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})
    temporary.replace(path)


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def as_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def numeric_mean(rows: list[dict[str, Any]], field: str) -> float | None:
    values = [as_float(row.get(field)) for row in rows]
    clean = [value for value in values if value is not None]
    return statistics.fmean(clean) if clean else None


def numeric_std(rows: list[dict[str, Any]], field: str) -> float | None:
    values = [as_float(row.get(field)) for row in rows]
    clean = [value for value in values if value is not None]
    if len(clean) < 2:
        return None
    return statistics.stdev(clean)


def paper_metric_role(profile: str, metric: str) -> str:
    """Return the frozen primary/secondary role for a profile metric."""
    profile_key = str(profile)
    if profile_key.startswith("triangle"):
        profile_key = "triangle"
    elif profile_key.startswith("circle"):
        profile_key = "circle"
    return (
        "primary"
        if PAPER_PRIMARY_METRICS.get(profile_key) == metric
        else "secondary"
    )



def _t_critical_95(degrees_freedom: int) -> float:
    """Two-sided 95% Student-t critical value without a SciPy dependency."""
    values = (
        12.706,
        4.303,
        3.182,
        2.776,
        2.571,
        2.447,
        2.365,
        2.306,
        2.262,
        2.228,
        2.201,
        2.179,
        2.160,
        2.145,
        2.131,
        2.120,
        2.110,
        2.101,
        2.093,
        2.086,
        2.080,
        2.074,
        2.069,
        2.064,
        2.060,
        2.056,
        2.052,
        2.048,
        2.045,
        2.042,
    )
    if degrees_freedom < 1:
        raise ValueError("degrees_freedom must be positive")
    return values[degrees_freedom - 1] if degrees_freedom <= 30 else 1.96


def _paired_repetition_values(
    rows_a: list[dict[str, Any]],
    rows_b: list[dict[str, Any]],
    metric_a: str,
    metric_b: str | None = None,
) -> tuple[np.ndarray, np.ndarray, list[str], list[str]]:
    """Pair only unique rows sharing a predeclared matched block."""
    metric_b = metric_a if metric_b is None else metric_b

    def by_matched_block(
        rows: list[dict[str, Any]],
    ) -> dict[str, list[dict[str, Any]]]:
        result: dict[str, list[dict[str, Any]]] = {}
        for item in rows:
            block_id = str(item.get("matched_block_id", "")).strip()
            if block_id:
                result.setdefault(block_id, []).append(item)
        return result

    first = by_matched_block(rows_a)
    second = by_matched_block(rows_b)
    common = sorted(set(first) & set(second))
    ambiguous = [
        key
        for key in common
        if len(first[key]) != 1 or len(second[key]) != 1
    ]
    clean = []
    paired_blocks = []
    for key in common:
        if key in ambiguous:
            continue
        first_value = as_float(first[key][0].get(metric_a))
        second_value = as_float(second[key][0].get(metric_b))
        if first_value is not None and second_value is not None:
            clean.append((first_value, second_value))
            paired_blocks.append(key)
    return (
        np.asarray([item[0] for item in clean]),
        np.asarray([item[1] for item in clean]),
        ambiguous,
        paired_blocks,
    )


def _comparison_row(
    *,
    comparison_type: str,
    condition: str,
    profile_a: str,
    profile_sha256_a: str,
    level_a: str,
    rows_a: list[dict[str, Any]],
    profile_b: str,
    profile_sha256_b: str,
    level_b: str,
    rows_b: list[dict[str, Any]],
    metric: str,
    metric_a: str | None = None,
    metric_b: str | None = None,
    unit: str | None = None,
    metric_role: str | None = None,
    emit_unavailable: bool = False,
    unavailable_reason: str = "",
    signed_definition: str | None = None,
) -> dict[str, Any] | None:
    metric_a = metric if metric_a is None else metric_a
    metric_b = metric if metric_b is None else metric_b
    first, second, ambiguous, paired_blocks = _paired_repetition_values(
        rows_a, rows_b, metric_a, metric_b
    )
    if first.size == 0 and not ambiguous and not emit_unavailable:
        return None
    differences = second - first
    paired_n = int(differences.size)
    mean_difference = float(np.mean(differences)) if paired_n else None
    difference_sd = None
    difference_sem = None
    ci_low = None
    ci_high = None
    cohen_dz = None
    if paired_n >= 2:
        difference_sd = float(np.std(differences, ddof=1))
        difference_sem = difference_sd / math.sqrt(paired_n)
        half_width = _t_critical_95(paired_n - 1) * difference_sem
        ci_low = mean_difference - half_width
        ci_high = mean_difference + half_width
        if difference_sd > 0.0:
            cohen_dz = mean_difference / difference_sd
    provenance = rows_a[0] if rows_a else (rows_b[0] if rows_b else {})
    if metric_role is None:
        metric_role = (
            "primary"
            if paper_metric_role(profile_a, metric_a) == "primary"
            and paper_metric_role(profile_b, metric_b) == "primary"
            else "secondary"
        )
    if not paired_n and not unavailable_reason:
        unavailable_reason = (
            "all common matched blocks are ambiguous duplicates"
            if ambiguous
            else (
                "no unique valid rows share a nonempty predeclared "
                "matched_block_id and both metrics"
            )
        )
    return {
        "comparison_type": comparison_type,
        "analysis_revision": provenance.get("analysis_revision", ""),
        "analyzer_sha256": provenance.get("analyzer_sha256", ""),
        "study_stack_contract_version": provenance.get(
            "study_stack_contract_version", ""
        ),
        "condition": condition,
        "matched_block_ids": ";".join(paired_blocks),
        "profile_a": profile_a,
        "profile_sha256_a": profile_sha256_a,
        "level_a": level_a,
        "profile_b": profile_b,
        "profile_sha256_b": profile_sha256_b,
        "level_b": level_b,
        "metric": metric,
        "metric_a": metric_a,
        "metric_b": metric_b,
        "metric_role": metric_role,
        "unit": unit or PAPER_METRICS[metric],
        "availability": "available" if paired_n else "unavailable",
        "unavailable_reason": unavailable_reason if not paired_n else "",
        "paired_n": paired_n,
        "mean_a": float(np.mean(first)) if paired_n else None,
        "mean_b": float(np.mean(second)) if paired_n else None,
        "paired_difference_b_minus_a": mean_difference,
        "paired_difference_sd": difference_sd,
        "paired_difference_sem": difference_sem,
        "ci95_lower": ci_low,
        "ci95_upper": ci_high,
        "cohen_dz": cohen_dz,
        "excluded_ambiguous_block_count": len(ambiguous),
        "excluded_ambiguous_block_ids": ";".join(ambiguous),
        "signed_definition": signed_definition or f"{level_b} minus {level_a}",
    }


def _caster_sort_key(name: str) -> tuple[int, str]:
    normalized = name.lower().replace("-", "_")
    if any(
        label in normalized
        for label in ("traditional", "trailing", "conventional", "standard")
    ):
        return 0, normalized
    if "spherical" in normalized:
        return 1, normalized
    return 2, normalized


def _caster_signed_definition(level_a: str, level_b: str) -> str:
    first_role = _caster_sort_key(level_a)[0]
    second_role = _caster_sort_key(level_b)[0]
    if first_role == 0 and second_role == 1:
        return "spherical minus traditional/trailing caster"
    return f"{level_b} minus {level_a}"

def build_matched_comparisons(
    runs: list[dict[str, Any]],
    planned: list[dict[str, Any]] | None = None,
    plan_basename: str = "",
    plan_sha256: str = "",
) -> list[dict[str, Any]]:
    """Build valid-only paired contrasts, including reorientation penalties."""
    if not planned:
        return []
    valid = [
        item
        for item in runs
        if str(item.get("valid_for_aggregate", "")).lower() == "true"
    ]
    comparisons: list[dict[str, Any]] = []

    caster_groups: dict[tuple[str, ...], dict[str, list[dict[str, Any]]]] = {}
    for item in valid:
        key = tuple(
            str(item.get(field, ""))
            for field in (
                "profile",
                "profile_sha256",
                "analysis_revision",
                "analyzer_sha256",
                "study_stack_contract_version",
                "controller_config_sha256",
                "wheel_speed_limit_rad_s",
                "turret_control_enabled",
                "speed_m_s",
                "nominal_start_base_yaw_rad",
                "start_base_yaw_tolerance_rad",
                "terrain",
                "initial_caster_orientation_deg",
            )
        )
        caster_groups.setdefault(key, {}).setdefault(
            str(item.get("caster_type", "")), []
        ).append(item)
    for key, levels in sorted(caster_groups.items()):
        (
            profile,
            profile_hash,
            analysis_revision,
            analysis_hash,
            contract_version,
            controller_hash,
            wheel_limit,
            turret_enabled,
            speed,
            nominal_yaw,
            yaw_tolerance,
            terrain,
            orientation,
        ) = key
        names = sorted(
            (name for name in levels if name), key=_caster_sort_key
        )
        for index, first_name in enumerate(names):
            for second_name in names[index + 1 :]:
                condition = (
                    f"profile={profile}; speed={speed}; terrain={terrain}; "
                    f"initial_caster_deg={orientation}; "
                    f"start_yaw={nominal_yaw}+/-{yaw_tolerance}rad; "
                    f"analysis={analysis_revision}:{analysis_hash[:12]}; "
                    f"contract={contract_version}; "
                    f"controller={controller_hash[:12]}; limit={wheel_limit}; "
                    f"turret={turret_enabled}"
                )
                for metric in PAPER_METRICS:
                    result = _comparison_row(
                        comparison_type="caster",
                        condition=condition,
                        profile_a=profile,
                        profile_sha256_a=profile_hash,
                        level_a=first_name,
                        rows_a=levels[first_name],
                        profile_b=profile,
                        profile_sha256_b=profile_hash,
                        level_b=second_name,
                        rows_b=levels[second_name],
                        metric=metric,
                        signed_definition=_caster_signed_definition(
                            first_name, second_name
                        ),
                    )
                    if result is not None:
                        comparisons.append(result)

    speed_groups: dict[tuple[str, ...], dict[str, list[dict[str, Any]]]] = {}
    for item in valid:
        key = tuple(
            str(item.get(field, ""))
            for field in (
                "profile",
                "profile_sha256",
                "analysis_revision",
                "analyzer_sha256",
                "study_stack_contract_version",
                "controller_config_sha256",
                "wheel_speed_limit_rad_s",
                "turret_control_enabled",
                "caster_type",
                "nominal_start_base_yaw_rad",
                "start_base_yaw_tolerance_rad",
                "terrain",
                "initial_caster_orientation_deg",
            )
        )
        speed_groups.setdefault(key, {}).setdefault(
            str(item.get("speed_m_s", "")), []
        ).append(item)
    for key, levels in sorted(speed_groups.items()):
        (
            profile,
            profile_hash,
            analysis_revision,
            analysis_hash,
            contract_version,
            controller_hash,
            wheel_limit,
            turret_enabled,
            caster,
            nominal_yaw,
            yaw_tolerance,
            terrain,
            orientation,
        ) = key
        names = sorted(
            (name for name in levels if as_float(name) is not None),
            key=float,
        )
        for index, first_name in enumerate(names):
            for second_name in names[index + 1 :]:
                condition = (
                    f"profile={profile}; caster={caster}; terrain={terrain}; "
                    f"initial_caster_deg={orientation}; "
                    f"start_yaw={nominal_yaw}+/-{yaw_tolerance}rad; "
                    f"analysis={analysis_revision}:{analysis_hash[:12]}; "
                    f"contract={contract_version}; "
                    f"controller={controller_hash[:12]}; limit={wheel_limit}; "
                    f"turret={turret_enabled}"
                )
                for metric in PAPER_METRICS:
                    result = _comparison_row(
                        comparison_type="speed",
                        condition=condition,
                        profile_a=profile,
                        profile_sha256_a=profile_hash,
                        level_a=first_name,
                        rows_a=levels[first_name],
                        profile_b=profile,
                        profile_sha256_b=profile_hash,
                        level_b=second_name,
                        rows_b=levels[second_name],
                        metric=metric,
                        signed_definition=(
                            f"{second_name} m/s minus {first_name} m/s"
                        ),
                    )
                    if result is not None:
                        comparisons.append(result)

    direction_pairs = (
        (
            "longitudinal",
            "straight_forward",
            "straight_backward",
            "straight_backward minus straight_forward",
        ),
        (
            "lateral",
            "lateral_right",
            "lateral_left",
            "lateral_left minus lateral_right",
        ),
        (
            "circle",
            "circle_ccw",
            "circle_cw",
            "circle_cw minus circle_ccw",
        ),
    )
    for family, profile_a, profile_b, signed_definition in direction_pairs:
        first = [item for item in valid if item.get("profile") == profile_a]
        second = [item for item in valid if item.get("profile") == profile_b]
        first_groups: dict[tuple[str, ...], list[dict[str, Any]]] = {}
        second_groups: dict[tuple[str, ...], list[dict[str, Any]]] = {}
        condition_fields = (
            "study_stack_contract_version",
            "controller_config_sha256",
            "wheel_speed_limit_rad_s",
            "turret_control_enabled",
            "speed_m_s",
            "caster_type",
            "nominal_start_base_yaw_rad",
            "start_base_yaw_tolerance_rad",
            "terrain",
            "initial_caster_orientation_deg",
        )
        for item in first:
            key = (
                str(item.get("profile_sha256", "")),
                str(item.get("analysis_revision", "")),
                str(item.get("analyzer_sha256", "")),
                *(str(item.get(field, "")) for field in condition_fields),
            )
            first_groups.setdefault(key, []).append(item)
        for item in second:
            key = (
                str(item.get("profile_sha256", "")),
                str(item.get("analysis_revision", "")),
                str(item.get("analyzer_sha256", "")),
                *(str(item.get(field, "")) for field in condition_fields),
            )
            second_groups.setdefault(key, []).append(item)
        for first_key, rows_a in sorted(first_groups.items()):
            (
                hash_a,
                analysis_revision,
                analysis_hash,
                contract_version,
                controller_hash,
                wheel_limit,
                turret_enabled,
                speed,
                caster,
                nominal_yaw,
                yaw_tolerance,
                terrain,
                orientation,
            ) = first_key
            for second_key, rows_b in sorted(second_groups.items()):
                (
                    hash_b,
                    other_analysis_revision,
                    other_analysis_hash,
                    other_contract_version,
                    other_controller_hash,
                    other_wheel_limit,
                    other_turret_enabled,
                    other_speed,
                    other_caster,
                    other_nominal_yaw,
                    other_yaw_tolerance,
                    other_terrain,
                    other_orientation,
                ) = second_key
                if (
                    analysis_revision,
                    analysis_hash,
                    contract_version,
                    controller_hash,
                    wheel_limit,
                    turret_enabled,
                    speed,
                    caster,
                    nominal_yaw,
                    yaw_tolerance,
                    terrain,
                    orientation,
                ) != (
                    other_analysis_revision,
                    other_analysis_hash,
                    other_contract_version,
                    other_controller_hash,
                    other_wheel_limit,
                    other_turret_enabled,
                    other_speed,
                    other_caster,
                    other_nominal_yaw,
                    other_yaw_tolerance,
                    other_terrain,
                    other_orientation,
                ):
                    continue
                condition = (
                    f"family={family}; speed={speed}; caster={caster}; "
                    f"terrain={terrain}; initial_caster_deg={orientation}; "
                    f"start_yaw={nominal_yaw}+/-{yaw_tolerance}rad; "
                    f"analysis={analysis_revision}:{analysis_hash[:12]}; "
                    f"contract={contract_version}; "
                    f"controller={controller_hash[:12]}; limit={wheel_limit}; "
                    f"turret={turret_enabled}"
                )
                for metric in PAPER_METRICS:
                    result = _comparison_row(
                        comparison_type="direction",
                        condition=condition,
                        profile_a=profile_a,
                        profile_sha256_a=hash_a,
                        level_a=profile_a,
                        rows_a=rows_a,
                        profile_b=profile_b,
                        profile_sha256_b=hash_b,
                        level_b=profile_b,
                        rows_b=rows_b,
                        metric=metric,
                        signed_definition=signed_definition,
                    )
                    if result is not None:
                        comparisons.append(result)
    comparisons.extend(_build_reorientation_penalties(runs, planned or []))
    for item in comparisons:
        item["plan_basename"] = plan_basename
        item["plan_sha256"] = plan_sha256
    return comparisons


def _reorientation_condition_key(item: dict[str, Any]) -> tuple[str, ...]:
    fields = (
        "analysis_revision",
        "analyzer_sha256",
        "study_stack_contract_version",
        "controller_config_sha256",
        "wheel_speed_limit_rad_s",
        "turret_control_enabled",
        "speed_m_s",
        "caster_type",
        "nominal_start_base_yaw_rad",
        "start_base_yaw_tolerance_rad",
        "terrain",
        "initial_caster_orientation_deg",
    )
    return tuple(str(item.get(field, "")) for field in fields)


def _planned_baseline_hashes(
    planned: list[dict[str, Any]], target_rows: list[dict[str, Any]]
) -> set[str]:
    """Return planned straight hashes matching target factors/repetitions."""
    if not planned or not target_rows:
        return set()
    comparison_fields = (
        "study_stack_contract_version",
        "controller_config_sha256",
        "wheel_speed_limit_rad_s",
        "turret_control_enabled",
        "nominal_start_base_yaw_rad",
        "start_base_yaw_tolerance_rad",
        "speed_m_s",
        "caster_type",
        "terrain",
        "initial_caster_orientation_deg",
    )
    targets = {
        tuple(
            _normalized_trial_value(field, row.get(field, ""))
            for field in (*comparison_fields, "repetition")
        )
        for row in target_rows
    }
    return {
        str(item.get("profile_sha256", "")).strip()
        for item in planned
        if str(item.get("profile", "")).strip().lower()
        == "straight_forward"
        and tuple(
            _normalized_trial_value(field, item.get(field, ""))
            for field in (*comparison_fields, "repetition")
        )
        in targets
    }


def _build_reorientation_penalties(
    runs: list[dict[str, Any]], planned: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """Compare each reorientation event with a matched straight onset."""
    targets = {
        "lateral_left": "translation_onset_along_abs_integral_m_s",
        "lateral_right": "translation_onset_along_abs_integral_m_s",
        "forward_reverse": "reversal_post_reverse_along_abs_integral_m_s",
    }
    grouped: dict[
        str, dict[tuple[str, tuple[str, ...]], list[dict[str, Any]]]
    ] = {}
    for item in runs:
        profile = str(item.get("profile", ""))
        if profile != "straight_forward" and profile not in targets:
            continue
        group_key = (
            str(item.get("profile_sha256", "")),
            _reorientation_condition_key(item),
        )
        grouped.setdefault(profile, {}).setdefault(group_key, []).append(item)

    baseline_groups = grouped.get("straight_forward", {})
    results: list[dict[str, Any]] = []
    for profile_b, metric_b in targets.items():
        for (hash_b, condition_key), all_rows_b in sorted(
            grouped.get(profile_b, {}).items()
        ):
            valid_rows_b = [
                item
                for item in all_rows_b
                if str(item.get("valid_for_aggregate", "")).lower() == "true"
            ]
            matching_baselines = [
                (hash_a, all_rows_a)
                for (hash_a, baseline_key), all_rows_a in baseline_groups.items()
                if baseline_key == condition_key
            ]
            (
                analysis_revision,
                analysis_hash,
                contract_version,
                controller_hash,
                wheel_limit,
                turret_enabled,
                speed,
                caster,
                nominal_yaw,
                yaw_tolerance,
                terrain,
                orientation,
            ) = condition_key
            condition = (
                f"target={profile_b}; speed={speed}; caster={caster}; "
                f"terrain={terrain}; initial_caster_deg={orientation}; "
                f"start_yaw={nominal_yaw}+/-{yaw_tolerance}rad; "
                f"analysis={analysis_revision}:{analysis_hash[:12]}; "
                f"contract={contract_version}; "
                f"controller={controller_hash[:12]}; limit={wheel_limit}; "
                f"turret={turret_enabled}"
            )
            planned_hashes = _planned_baseline_hashes(planned, all_rows_b)
            if not matching_baselines:
                if planned and not planned_hashes:
                    reason = (
                        "selected plan contains no straight_forward baseline "
                        "with matching factors and repetition"
                    )
                elif planned:
                    reason = (
                        "matched straight_forward baseline is planned but has "
                        "no analyzed run in this condition"
                    )
                else:
                    reason = "no valid recorded matched straight_forward baseline"
                result = _comparison_row(
                    comparison_type="reorientation_penalty",
                    condition=condition,
                    profile_a="straight_forward",
                    profile_sha256_a=";".join(sorted(planned_hashes)),
                    level_a="straight_forward 2.0 s onset",
                    rows_a=[],
                    profile_b=profile_b,
                    profile_sha256_b=hash_b,
                    level_b=f"{profile_b} 2.0 s event response",
                    rows_b=valid_rows_b,
                    metric=REORIENTATION_PENALTY_METRIC,
                    metric_a="translation_onset_along_abs_integral_m_s",
                    metric_b=metric_b,
                    unit="m*s",
                    metric_role="primary",
                    emit_unavailable=True,
                    unavailable_reason=reason,
                    signed_definition=(
                        f"{profile_b} event minus matched straight_forward "
                        "onset; positive means worse than straight"
                    ),
                )
                assert result is not None
                results.append(result)
                continue

            for hash_a, all_rows_a in matching_baselines:
                valid_rows_a = [
                    item
                    for item in all_rows_a
                    if str(item.get("valid_for_aggregate", "")).lower()
                    == "true"
                ]
                result = _comparison_row(
                    comparison_type="reorientation_penalty",
                    condition=condition,
                    profile_a="straight_forward",
                    profile_sha256_a=hash_a,
                    level_a="straight_forward 2.0 s onset",
                    rows_a=valid_rows_a,
                    profile_b=profile_b,
                    profile_sha256_b=hash_b,
                    level_b=f"{profile_b} 2.0 s event response",
                    rows_b=valid_rows_b,
                    metric=REORIENTATION_PENALTY_METRIC,
                    metric_a="translation_onset_along_abs_integral_m_s",
                    metric_b=metric_b,
                    unit="m*s",
                    metric_role="primary",
                    emit_unavailable=True,
                    signed_definition=(
                        f"{profile_b} event minus matched straight_forward "
                        "onset; positive means worse than straight"
                    ),
                )
                assert result is not None
                results.append(result)
    return results


def _normalized_trial_value(field: str, value: Any) -> str:
    if field in (
        "wheel_speed_limit_rad_s",
        "nominal_start_base_yaw_rad",
        "start_base_yaw_tolerance_rad",
        "speed_m_s",
        "initial_caster_orientation_deg",
        "repetition",
        "study_stack_contract_version",
    ):
        number = as_float(value)
        return f"{number:.9g}" if number is not None else str(value).strip()
    return str(value).strip().lower()


def classify_runs_against_plan(
    planned: list[dict[str, Any]],
    runs: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Filter formal rows by unique planned_id and exact declared factors."""
    if not planned:
        raise RuntimeError("selected study plan is empty")
    plan_by_id: dict[str, dict[str, Any]] = {}
    planned_orders: set[int] = set()
    for item in planned:
        planned_id = str(item.get("planned_id", "")).strip()
        if not planned_id:
            raise RuntimeError("every selected plan row must have planned_id")
        if planned_id in plan_by_id:
            raise RuntimeError(f"duplicate planned_id in selected plan: {planned_id}")
        if not str(item.get("gate_added", "")).strip():
            raise RuntimeError(f"plan row {planned_id} is missing gate_added")
        if not str(item.get("matched_block_id", "")).strip():
            raise RuntimeError(
                f"plan row {planned_id} is missing matched_block_id"
            )
        if not str(item.get("hardware_block_id", "")).strip():
            raise RuntimeError(
                f"plan row {planned_id} is missing hardware_block_id"
            )
        raw_order = str(item.get("planned_order", "")).strip()
        try:
            planned_order = int(raw_order)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                f"plan row {planned_id} has invalid planned_order {raw_order!r}; "
                "expected a positive integer"
            ) from exc
        if planned_order <= 0 or str(planned_order) != raw_order:
            raise RuntimeError(
                f"plan row {planned_id} has invalid planned_order {raw_order!r}; "
                "expected a canonical positive integer"
            )
        if planned_order in planned_orders:
            raise RuntimeError(
                f"duplicate planned_order in selected plan: {planned_order}"
            )
        planned_orders.add(planned_order)
        missing = [
            field
            for field in PLAN_FACTOR_FIELDS
            if not str(item.get(field, "")).strip()
        ]
        if missing:
            raise RuntimeError(
                f"plan row {planned_id} is missing exact factors: "
                + ", ".join(missing)
            )
        plan_by_id[planned_id] = item

    classifications: list[dict[str, Any]] = []
    factor_matched: list[tuple[dict[str, Any], dict[str, Any]]] = []
    valid_by_plan: dict[str, list[int]] = {}
    for item in runs:
        planned_id = str(item.get("planned_id", "")).strip()
        classification = {
            "run_id": item.get("run_id", ""),
            "planned_id": planned_id,
            "status": "unplanned",
            "reason": "run has no planned_id",
            "valid_for_aggregate": item.get("valid_for_aggregate", ""),
        }
        planned_item = plan_by_id.get(planned_id)
        if planned_id and planned_item is None:
            classification["reason"] = "planned_id is absent from selected plan"
        elif planned_item is not None:
            mismatches = []
            for field in PLAN_FACTOR_FIELDS:
                planned_value = _normalized_trial_value(
                    field, planned_item.get(field, "")
                )
                run_value = _normalized_trial_value(field, item.get(field, ""))
                if planned_value != run_value:
                    mismatches.append(
                        f"{field}: plan={planned_value!r}, run={run_value!r}"
                    )
            if mismatches:
                classification["status"] = "plan_factor_mismatch"
                classification["reason"] = "; ".join(mismatches)
            else:
                formal_copy = dict(item)
                formal_copy.update(
                    {
                        "planned_order": planned_item.get(
                            "planned_order", ""
                        ),
                        "gate_added": planned_item.get("gate_added", ""),
                        "matched_block_id": planned_item.get(
                            "matched_block_id", ""
                        ),
                        "hardware_block_id": planned_item.get(
                            "hardware_block_id", ""
                        ),
                    }
                )
                factor_matched.append((formal_copy, classification))
                if str(item.get("valid_for_aggregate", "")).lower() == "true":
                    classification["status"] = "planned_valid_candidate"
                    classification["reason"] = "exact plan and factor match"
                    valid_by_plan.setdefault(planned_id, []).append(
                        len(factor_matched) - 1
                    )
                else:
                    classification["status"] = "planned_recorded_invalid"
                    classification["reason"] = (
                        "exact plan match but run failed data-quality/exclusion rules"
                    )
        classifications.append(classification)

    for planned_id, indices in valid_by_plan.items():
        if len(indices) == 1:
            _formal, classification = factor_matched[indices[0]]
            classification["status"] = "planned_valid"
            continue
        for index in indices:
            formal, classification = factor_matched[index]
            formal["valid_for_aggregate"] = "false"
            classification["status"] = "ambiguous_valid_duplicate"
            classification["valid_for_aggregate"] = "false"
            classification["reason"] = (
                f"{len(indices)} valid attempts share planned_id {planned_id}; "
                "exclude all until the operator excludes superseded attempts"
            )

    formal_runs = [item for item, _classification in factor_matched]
    return formal_runs, classifications


def planned_trial_coverage(
    planned: list[dict[str, Any]], runs: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """Report exact-plan recorded-data yield and mismatch counts by cell."""
    _formal_runs, classifications = classify_runs_against_plan(planned, runs)
    by_planned_id: dict[str, list[dict[str, Any]]] = {}
    for item in classifications:
        by_planned_id.setdefault(str(item.get("planned_id", "")), []).append(item)
    cell_fields = (
        "profile",
        "profile_sha256",
        "speed_m_s",
        "caster_type",
        "terrain",
        "initial_caster_orientation_deg",
    )

    def cell_key(item: dict[str, Any]) -> tuple[str, ...]:
        return tuple(
            _normalized_trial_value(field, item.get(field, ""))
            for field in cell_fields
        )

    cells: dict[tuple[str, ...], list[dict[str, Any]]] = {}
    for item in planned:
        cells.setdefault(cell_key(item), []).append(item)
    unplanned_count = sum(
        item["status"] == "unplanned" for item in classifications
    )
    result = []
    for cell, items in sorted(cells.items()):
        plan_ids = [str(item["planned_id"]).strip() for item in items]
        attempts = [
            attempt
            for planned_id in plan_ids
            for attempt in by_planned_id.get(planned_id, [])
        ]
        factor_matched_statuses = {
            "planned_valid",
            "planned_recorded_invalid",
            "ambiguous_valid_duplicate",
        }
        result.append(
            {
                **dict(zip(cell_fields, cell)),
                "planned_trial_count": len(plan_ids),
                "recorded_trial_count": sum(
                    any(
                        attempt["status"] in factor_matched_statuses
                        for attempt in by_planned_id.get(planned_id, [])
                    )
                    for planned_id in plan_ids
                ),
                "recorded_attempt_count": len(attempts),
                "valid_trial_count": sum(
                    any(
                        attempt["status"] == "planned_valid"
                        for attempt in by_planned_id.get(planned_id, [])
                    )
                    for planned_id in plan_ids
                ),
                "ambiguous_valid_trial_count": sum(
                    any(
                        attempt["status"] == "ambiguous_valid_duplicate"
                        for attempt in by_planned_id.get(planned_id, [])
                    )
                    for planned_id in plan_ids
                ),
                "factor_mismatch_attempt_count": sum(
                    attempt["status"] == "plan_factor_mismatch"
                    for attempt in attempts
                ),
                "unplanned_run_count": unplanned_count,
            }
        )
    return result

def _format_estimate(value: Any, digits: int = 4) -> str:
    number = as_float(value)
    return f"{number:.{digits}g}" if number is not None else "--"


def build_condition_statistics(
    runs: list[dict[str, Any]],
    plan_basename: str = "",
    plan_sha256: str = "",
) -> list[dict[str, Any]]:
    """Build small-n descriptive summaries for each exact formal condition."""
    group_fields = CONDITION_GROUP_FIELDS
    groups: dict[tuple[str, ...], list[dict[str, Any]]] = {}
    for item in runs:
        if str(item.get("valid_for_aggregate", "")).lower() != "true":
            continue
        key = tuple(str(item.get(field, "")) for field in group_fields)
        groups.setdefault(key, []).append(item)

    results: list[dict[str, Any]] = []
    for key, rows in sorted(groups.items()):
        condition = dict(zip(group_fields, key))
        profile = condition["profile"]
        for metric, unit in PAPER_METRICS.items():
            values = [as_float(item.get(metric)) for item in rows]
            clean = [value for value in values if value is not None]
            if not clean:
                continue
            results.append(
                {
                    "plan_basename": plan_basename,
                    "plan_sha256": plan_sha256,
                    **condition,
                    "metric": metric,
                    "metric_role": paper_metric_role(profile, metric),
                    "unit": unit,
                    "n": len(clean),
                    "mean": statistics.fmean(clean),
                    "sd": statistics.stdev(clean) if len(clean) > 1 else None,
                    "median": statistics.median(clean),
                    "minimum": min(clean),
                    "maximum": max(clean),
                }
            )
    return results

def update_study_results(
    results_dir: Path,
    row: dict[str, Any],
    plan_path: Path | None = None,
) -> tuple[Path, ...]:
    runs_path = results_dir / "runs.csv"
    existing = read_csv_rows(runs_path)
    by_id = {item.get("run_id", ""): item for item in existing}
    by_id[str(row["run_id"])] = {field: row.get(field, "") for field in RUN_FIELDS}
    runs = sorted(by_id.values(), key=lambda item: item.get("run_id", ""))

    configured_plan = os.environ.get("HAMR_STUDY_PLAN")
    selected_plan = (
        plan_path.expanduser().resolve()
        if plan_path is not None
        else (
            Path(configured_plan).expanduser().resolve()
            if configured_plan
            else (results_dir / "planned_trials.csv").resolve()
        )
    )
    explicit_plan = plan_path is not None or bool(configured_plan)
    if explicit_plan and not selected_plan.is_file():
        raise RuntimeError(f"study plan does not exist: {selected_plan}")
    plan_sha256 = None
    planned: list[dict[str, Any]] = []
    classifications: list[dict[str, Any]] = []
    classification_path = results_dir / "planned_run_classification.csv"
    if selected_plan.is_file():
        plan_sha256 = hashlib.sha256(selected_plan.read_bytes()).hexdigest()
        planned = read_csv_rows(selected_plan)
        formal_runs, classifications = classify_runs_against_plan(planned, runs)
        plan_provenance = {
            "plan_basename": selected_plan.name,
            "plan_sha256": plan_sha256,
        }
        for item in classifications:
            item.update(plan_provenance)
    else:
        formal_runs = runs

    # Do not mutate any study product until the selected immutable plan has
    # been read, validated, and applied to the complete in-memory candidate.
    write_csv_rows(runs_path, RUN_FIELDS, runs)
    write_csv_rows(
        classification_path,
        PLAN_CLASSIFICATION_FIELDS,
        classifications,
    )

    group_fields = (
        "profile",
        "profile_sha256",
        "analysis_revision",
        "analyzer_sha256",
        "study_stack_contract_version",
        "controller_config_sha256",
        "wheel_speed_limit_rad_s",
        "turret_control_enabled",
        "speed_m_s",
        "nominal_start_base_yaw_rad",
        "start_base_yaw_tolerance_rad",
        "caster_type",
        "terrain",
        "initial_caster_orientation_deg",
    )
    groups: dict[tuple[str, ...], list[dict[str, Any]]] = {}
    for item in formal_runs:
        key = tuple(str(item.get(field, "")) for field in group_fields)
        groups.setdefault(key, []).append(item)
    aggregate_rows: list[dict[str, Any]] = []
    for key in sorted(groups):
        group = groups[key]
        valid = [
            item
            for item in group
            if str(item.get("valid_for_aggregate", "")).lower() == "true"
        ]
        aggregate_rows.append(
            {
                "plan_basename": selected_plan.name if planned else "",
                "plan_sha256": plan_sha256 or "",
                **dict(zip(group_fields, key)),
                "run_count": len(group),
                "valid_run_count": len(valid),
                "valid_rate": len(valid) / len(group),
                "xy_rmse_mean_m": numeric_mean(valid, "xy_rmse_m"),
                "xy_rmse_std_m": numeric_std(valid, "xy_rmse_m"),
                "active_time_weighted_xy_error_mean_m": numeric_mean(
                    valid, "active_time_weighted_xy_error_mean_m"
                ),
                "cross_track_rmse_mean_m": numeric_mean(
                    valid, "cross_track_rmse_m"
                ),
                "startup_hold_xy_jitter_p95_mean_m": numeric_mean(
                    valid, "startup_hold_xy_jitter_p95_m"
                ),
                "final_hold_xy_jitter_p95_mean_m": numeric_mean(
                    valid, "final_hold_xy_jitter_p95_m"
                ),
                "speed_bias_mean_m_s": numeric_mean(valid, "speed_bias_m_s"),
                "final_xy_error_mean_m": numeric_mean(
                    valid, "final_xy_error_m"
                ),
                "triangle_corner_error_mean_m": numeric_mean(
                    valid, "triangle_corner_error_mean_m"
                ),
                "circle_radial_rmse_mean_m": numeric_mean(
                    valid, "circle_radial_rmse_m"
                ),
                "base_yaw_drift_rmse_mean_rad": numeric_mean(
                    valid, "base_yaw_drift_rmse_rad"
                ),
                "translation_cross_axis_rmse_mean_m": numeric_mean(
                    valid, "translation_cross_axis_rmse_m"
                ),
                "translation_onset_along_peak_mean_m": numeric_mean(
                    valid, "translation_onset_along_peak_m"
                ),
                "translation_onset_along_abs_integral_mean_m_s": numeric_mean(
                    valid, "translation_onset_along_abs_integral_m_s"
                ),
                "speed_rise_time_mean_s": numeric_mean(
                    valid, "speed_rise_time_s"
                ),
                "speed_settling_time_mean_s": numeric_mean(
                    valid, "speed_settling_time_s"
                ),
                "steady_state_speed_bias_mean_m_s": numeric_mean(
                    valid, "steady_state_speed_bias_m_s"
                ),
                "reversal_stop_delay_mean_s": numeric_mean(
                    valid, "reversal_stop_delay_s"
                ),
                "reversal_delay_mean_s": numeric_mean(
                    valid, "reversal_delay_s"
                ),
                "reversal_overshoot_mean_m": numeric_mean(
                    valid, "reversal_overshoot_m"
                ),
                "reversal_event_along_peak_mean_m": numeric_mean(
                    valid, "reversal_event_along_peak_m"
                ),
                "reversal_event_along_abs_integral_mean_m_s": numeric_mean(
                    valid, "reversal_event_along_abs_integral_m_s"
                ),
                "reversal_post_reverse_along_abs_integral_mean_m_s": numeric_mean(
                    valid, "reversal_post_reverse_along_abs_integral_m_s"
                ),
                "reversal_closure_error_mean_m": numeric_mean(
                    valid, "reversal_closure_error_m"
                ),
                "actuator_pair_cap_fraction_mean": numeric_mean(
                    valid, "actuator_pair_cap_fraction"
                ),
                "firmware_saturation_fraction_mean": numeric_mean(
                    valid, "firmware_saturation_fraction"
                ),
                "wheel_rpm_error_rmse_mean": numeric_mean(
                    valid, "wheel_rpm_error_rmse"
                ),
                "imu_high_frequency_accel_rms_mean_m_s2": numeric_mean(
                    valid, "imu_high_frequency_accel_rms_m_s2"
                ),
                "imu_angular_rate_norm_rms_mean_rad_s": numeric_mean(
                    valid, "imu_angular_rate_norm_rms_rad_s"
                ),
            }
        )
    aggregate_path = results_dir / "aggregate_metrics.csv"
    write_csv_rows(aggregate_path, AGGREGATE_FIELDS, aggregate_rows)

    summary_path = results_dir / "study_summary.md"
    study_index_description = (
        f"Runs indexed: {len(runs)}; formal plan-matched runs: "
        f"{len(formal_runs)}; valid unique formal runs: "
        f"{sum(str(item.get('valid_for_aggregate', '')).lower() == 'true' for item in formal_runs)}."
        if planned
        else (
            f"Runs indexed: {len(runs)}; valid exploratory recorded rows: "
            f"{sum(str(item.get('valid_for_aggregate', '')).lower() == 'true' for item in formal_runs)}. "
            "No selected plan is attached to these aggregates."
        )
    )
    lines = [
        "# HAMR Independent-Study Summary",
        "",
        study_index_description,
        "",
        "Valid/total is recorded-data yield, not operational reliability. "
        "Operational reliability requires the external append-only attempt log "
        "with structured failure categories and no-bag attempts.",
        "",
        "| Profile | Profile SHA-256 | Speed (m/s) | Caster | Terrain | Initial caster (deg) | Valid / total | XY RMSE mean (m) |",
        "|---|---|---:|---|---|---:|---:|---:|",
    ]
    for item in aggregate_rows:
        xy_mean = as_float(item.get("xy_rmse_mean_m"))
        xy_text = f"{xy_mean:.4f}" if xy_mean is not None else "—"
        lines.append(
            f"| {item['profile']} | {str(item['profile_sha256'])[:12]} | {item['speed_m_s']} | {item['caster_type']} "
            f"| {item['terrain']} | {item['initial_caster_orientation_deg']} "
            f"| {item['valid_run_count']} / {item['run_count']} | {xy_text} |"
        )
    comparisons = build_matched_comparisons(
        formal_runs,
        planned=planned,
        plan_basename=selected_plan.name if planned else "",
        plan_sha256=plan_sha256 or "",
    )
    comparisons_path = results_dir / "paper_matched_comparisons.csv"
    write_csv_rows(comparisons_path, COMPARISON_FIELDS, comparisons)
    condition_statistics = build_condition_statistics(
        formal_runs,
        plan_basename=selected_plan.name if planned else "",
        plan_sha256=plan_sha256 or "",
    )
    condition_statistics_path = results_dir / "paper_condition_statistics.csv"
    write_csv_rows(
        condition_statistics_path,
        CONDITION_STAT_FIELDS,
        condition_statistics,
    )

    coverage_rows: list[dict[str, Any]] = []
    coverage_path = results_dir / "planned_trial_coverage.csv"
    if planned:
        coverage_rows = planned_trial_coverage(planned, runs)
        for item in coverage_rows:
            item.update(
                {
                    "plan_basename": selected_plan.name,
                    "plan_sha256": plan_sha256,
                }
            )
    write_csv_rows(coverage_path, PLAN_COVERAGE_FIELDS, coverage_rows)

    lines.extend(
        [
            "",
            "Plan source: "
            + (
                f"`{selected_plan}` (SHA-256 `{plan_sha256}`; read-only)."
                if plan_sha256 is not None
                else "none (default path was absent)."
            ),
        ]
    )
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = summary_path.with_suffix(".md.tmp")
    temporary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    temporary.replace(summary_path)

    valid_runs = [
        item
        for item in formal_runs
        if str(item.get("valid_for_aggregate", "")).lower() == "true"
    ]
    cap_values = [
        value
        for item in valid_runs
        if (value := as_float(item.get("actuator_pair_cap_fraction")))
        is not None
    ]
    saturation_values = [
        value
        for item in valid_runs
        if (value := as_float(item.get("firmware_saturation_fraction")))
        is not None
    ]
    cap_summary = (
        f"Controller pair-cap exposure was nonzero in "
        f"{sum(value > 0.0 for value in cap_values)}/{len(cap_values)} valid "
        "runs with paired command diagnostics."
        if cap_values
        else "Controller pair-cap exposure is unavailable for all valid runs."
    )
    saturation_summary = (
        f"Firmware saturation was nonzero in "
        f"{sum(value > 0.0 for value in saturation_values)}/"
        f"{len(saturation_values)} valid runs with exact "
        "wheel_control_status_v1 diagnostics."
        if saturation_values
        else (
            "Firmware saturation exposure is unavailable for all valid runs; "
            "no zero-exposure conclusion is made."
        )
    )
    paper_index_description = (
        f"This index contains {len(runs)} recorded runs; {len(formal_runs)} "
        f"match the selected plan/factors and {len(valid_runs)} are unique valid "
        "formal runs."
        if planned
        else (
            f"This index contains {len(runs)} recorded runs; {len(valid_runs)} "
            "are valid exploratory rows. No selected plan is attached, so these "
            "rows are not described as formal plan-matched trials."
        )
    )
    paper_lines = [
        "# Paper-ready descriptive results",
        "",
        paper_index_description,
        "",
        "## Interpretation limits",
        "",
        "All contrasts below are descriptive paired estimates using shared "
        "nonempty matched_block_id values from the immutable plan. Difference "
        "means B minus A. A two-sided 95% "
        "Student-t interval is shown only when at least two pairs exist; no "
        "p-values or population-level causal claims are produced. Duplicate "
        "valid attempts within one matched block are excluded and flagged. "
        "Profile revisions remain separate by SHA-256.",
        "Primary metrics are frozen by profile in "
        "`paper_condition_statistics.csv`: straight_forward and "
        "straight_backward use cross-track RMSE; lateral_left and lateral_right "
        "use the 2.0 s onset integral of absolute along-track error; "
        "forward_reverse uses the 2.0 s post-reverse integral; triangle profiles "
        "use mean corner error; and circle_cw and circle_ccw use radial RMSE. "
        "Other emitted metrics are labeled secondary for the applicable profile.",
        "For descriptive ranking across different trajectory profiles, use "
        "`active_time_weighted_xy_error_mean_m`, the time-weighted Euclidean "
        "XY error over active motion. Do not rank profiles by their different "
        "profile-specific primary metrics or mix metre and metre-second units.",
        "Formal primary/common outcomes require every reference sample in each "
        "required window to match finite Vicon within 50 ms and no effective "
        "support gap over 100 ms. This is an offline validity rule only and "
        "never interrupts a live test.",
        "Actuator exposure fractions are secondary and are emitted only when "
        "each required active reference sample (including the forward_reverse "
        "dwell) has command/status support within 50 ms and no phasewise "
        "effective topic gap exceeds 100 ms. Sparse streams are reported as "
        "unavailable, never as observed zero exposure.",
        "Formal Vicon data also require nonzero, strictly increasing source "
        "stamps with no source-stamp gap over 100 ms across the recorded "
        "reference lifecycle. Analysis v4 maps usable capture stamps onto bag "
        "time with the median receipt-minus-source offset for primary pose and "
        "velocity metrics, while retaining receipt-time delivery diagnostics. "
        "This offset combines unknown clock offset and typical delivery delay; "
        "it is not absolute latency, and a replay with normally advancing stamps "
        "cannot be detected by this check.",
        "",
        "This analysis revision's formal pose envelope is flat-ground-only; "
        "non-flat trials remain analyzable but are excluded until a predeclared "
        "terrain-specific quality rule is implemented. "
        "Caster attribution requires checking actuator-limit exposure and IMU "
        "data alongside path error. Pair-cap or firmware-saturation exposure "
        "can explain speed mismatch independently of caster behavior. IMU "
        "high-frequency acceleration and angular-rate values are chassis-"
        "disturbance proxies, not direct proof of caster vibration, slip, or "
        "contact transitions.",
        "Actuator and IMU summaries for forward_reverse are run-level secondary "
        "diagnostics over outbound-through-return (including the dwell), not "
        "event-localized mechanism measures. Event-window attribution therefore "
        "requires a separate predeclared analysis revision. Wheel-status schema "
        "is not a flashed-firmware revision; firmware identity must come from "
        "the operator ledger/provenance record.",
        "`planned_valid` verifies automated plan factors and bag quality only. "
        "This analyzer does not join the attempt log or condition on session, "
        "hardware, flashed-firmware, workspace revisions, or matched start "
        "placement. Audit the external ledger and captured origins before any "
        "causal caster claim.",
        "",
        cap_summary,
        saturation_summary,
        "",
        "## Recorded-data yield and completeness",
        "",
    ]
    if coverage_rows:
        planned_total = sum(int(item["planned_trial_count"]) for item in coverage_rows)
        recorded_total = sum(
            int(item["recorded_trial_count"]) for item in coverage_rows
        )
        valid_total = sum(int(item["valid_trial_count"]) for item in coverage_rows)
        ambiguous_total = sum(
            int(item["ambiguous_valid_trial_count"])
            for item in coverage_rows
        )
        paper_lines.extend(
            [
                f"Plan source: `{selected_plan}`; SHA-256 `{plan_sha256}` "
                "(read-only).",
                f"The planned run sheet contains {planned_total} trials: "
                f"{recorded_total} were recorded and {valid_total} had exactly "
                "one valid attempt.",
                f"{ambiguous_total} planned trials had multiple valid attempts "
                "for the same planned_id and are excluded as ambiguous.",
                "See `planned_trial_coverage.csv` for cell-level denominators.",
            ]
        )
    else:
        paper_lines.extend(
            [
                "No plan CSV was found. Aggregate and comparison outputs are "
                "exploratory/non-predeclared and use recorded bags; they cannot "
                "count attempts that produced no bag. Blank plan_basename and "
                "plan_sha256 columns mean no selected plan was attached.",
            ]
        )
    paper_lines.extend(
        [
            "",
            "These automated counts are data yield/completeness, not operational "
            "reliability. Operational reliability requires the external append-"
            "only attempt log with structured sensing, setup, corridor, controller, "
            "and mechanical failure categories, including no-bag attempts.",
            "",
            "## Matched comparisons",
            "",
            "| Type | Role | Signed contrast (B - A) | Metric | n pairs | Estimate | 95% t interval | dz | Availability | Excluded ambiguous blocks |",
            "|---|---|---|---|---:|---:|---:|---:|---|---:|",
        ]
    )
    if comparisons:
        for item in comparisons:
            low = as_float(item.get("ci95_lower"))
            high = as_float(item.get("ci95_upper"))
            interval = (
                f"[{_format_estimate(low)}, {_format_estimate(high)}]"
                if low is not None and high is not None
                else "--"
            )
            paper_lines.append(
                f"| {item['comparison_type']} | {item['metric_role']} | "
                f"{item['signed_definition']} | "
                f"{item['metric']} ({item['unit']}) | {item['paired_n']} | "
                f"{_format_estimate(item['paired_difference_b_minus_a'])} | "
                f"{interval} | {_format_estimate(item['cohen_dz'])} | "
                f"{item['availability']} | "
                f"{item['excluded_ambiguous_block_count']} |"
            )
    else:
        paper_lines.append(
            "| -- | -- | No valid matched blocks yet | -- | 0 | -- | -- | -- | unavailable | 0 |"
        )
    paper_lines.extend(
        [
            "",
            "Use `paper_matched_comparisons.csv` for exact condition labels, "
            "both profile hashes, paired means, uncertainty, effect sizes, and "
            "explicit unavailable reasons. Use `paper_condition_statistics.csv` "
            "for n, mean, SD, median, and range by exact condition and metric role.",
        ]
    )
    paper_path = results_dir / "paper_summary.md"
    temporary = paper_path.with_suffix(".md.tmp")
    temporary.write_text("\n".join(paper_lines) + "\n", encoding="utf-8")
    temporary.replace(paper_path)
    artifacts = [
        runs_path,
        aggregate_path,
        summary_path,
        comparisons_path,
        condition_statistics_path,
        paper_path,
    ]
    artifacts.append(coverage_path)
    artifacts.append(classification_path)
    return tuple(artifacts)


def find_actuator_helper() -> Path:
    sibling = Path(__file__).resolve().parent / "plot_hamr_actuators.py"
    if sibling.is_file():
        return sibling
    raise RuntimeError("plot_hamr_actuators.py is not installed beside analyzer")


def run_actuator_analysis(
    bag: Path,
    output: Path,
    counts: dict[str, int],
    manifest: dict[str, Any] | None = None,
) -> tuple[list[Path], str | None]:
    missing = [topic for topic in ACTUATOR_TOPICS if counts.get(topic, 0) <= 0]
    if missing:
        return [], "actuator plots skipped; missing data on " + ", ".join(missing)
    command = [
        sys.executable,
        str(find_actuator_helper()),
        str(bag),
        "--output-prefix",
        str(output / "actuator"),
    ]
    declared_limit = as_float((manifest or {}).get("wheel_speed_limit_rad_s"))
    if declared_limit is not None and declared_limit > 0.0:
        command.extend(("--wheel-cap-rad-s", f"{declared_limit:.12g}"))
    profile = str((manifest or {}).get("profile", "")).strip()
    if profile:
        command.extend(("--profile", profile))
    try:
        subprocess.run(command, check=True)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        return [], f"actuator plot failed: {exc}"
    return [output / "actuator_overview.png", output / "actuator_stop_zoom.png"], None


def analyze_bag(
    bag: Path,
    output: Path,
    exclude_reason: str = "",
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], list[Path]]:
    valid, reason, counts = metadata_preflight(bag)
    if not valid:
        raise RuntimeError(f"bag is not analyzable: {reason}")
    requested = (
        set(CORE_TOPICS) | {TURRET_TOPIC, IMU_TOPIC} | set(ACTUATOR_TOPICS)
    )
    rows, _types = _read_rows(bag, requested)
    manifest = parse_manifest_rows(rows[METADATA_TOPIC])
    profile_integrity = profile_hash_validation(manifest)
    manifest_run_id = manifest.get("run_id")
    run_id = str(manifest_run_id or bag.name)
    reference = reference_arrays(rows[REFERENCE_TOPIC])
    time_reference = build_time_reference_metadata(
        load_bag_metadata(bag), reference
    )
    base = odom_arrays(rows[BASE_TOPIC])
    turret = odom_arrays(rows[TURRET_TOPIC]) if rows[TURRET_TOPIC] else None

    lifecycle, incomplete_reasons = reference_lifecycle(reference, manifest)
    lifecycle_mask = (
        (base["t"] >= reference["t"][0])
        & (base["t"] <= reference["t"][-1])
    )
    lifecycle_base = {
        key: value[lifecycle_mask] for key, value in base.items()
    }
    vicon_quality = stream_quality(lifecycle_base["t"])
    pose_quality = vicon_pose_quality(lifecycle_base)
    source_stamp_quality = vicon_source_stamp_quality(lifecycle_base)
    whole_bag_vicon_quality = stream_quality(base["t"])
    whole_bag_pose_quality = vicon_pose_quality(base)
    primary_window_quality = primary_window_data_quality(
        reference, base, manifest
    )
    tracking, traces = tracking_metrics(reference, base, turret, manifest)
    warnings: list[str] = []
    if not manifest_run_id:
        warnings.append("manifest has no immutable run_id")
    if not profile_integrity["valid"]:
        warnings.append(str(profile_integrity["reason"]))
    if run_id != bag.name:
        warnings.append(
            f"manifest run_id {run_id!r} disagrees with bag directory {bag.name!r}"
        )
    if vicon_quality.get("gap_over_100ms_count", 0):
        warnings.append(
            "Vicon contains receipt gaps over 100 ms; the controller did not abort, "
            "and affected samples were excluded from time alignment"
        )
    if tracking["coverage_fraction"] < 0.90:
        warnings.append("less than 90% of reference samples have nearby Vicon data")
    if tracking["first_reference_match_dt_s"] > MAX_POSE_MATCH_DT_S:
        warnings.append("Vicon does not cover the first reference sample")
    if tracking["last_reference_match_dt_s"] > MAX_POSE_MATCH_DT_S:
        warnings.append("Vicon does not cover the final reference hold")
    if pose_quality["nonfinite_sample_count"]:
        warnings.append("Vicon contains non-finite pose samples")
    if pose_quality["xy_jump_over_80mm_count"]:
        warnings.append("Vicon contains XY jumps over 80 mm")
    if pose_quality["yaw_jump_over_0_35rad_count"]:
        warnings.append("Vicon contains yaw jumps over 0.35 rad")
    if pose_quality["z_outside_0_25_to_0_40m_count"]:
        warnings.append("Vicon base height leaves the nominal 0.25..0.40 m range")
    if pose_quality["tilt_over_0_35rad_count"]:
        warnings.append("Vicon base tilt exceeds 0.35 rad")
    if not source_stamp_quality["valid"]:
        warnings.append(
            "offline Vicon source-stamp validity failed: "
            + source_stamp_quality["failure_reason"]
        )
    if not primary_window_quality["valid"]:
        warnings.append(
            "offline Vicon quality failed in a frozen primary/event window: "
            + "; ".join(primary_window_quality["failure_reasons"])
        )

    actuator = actuator_diagnostics(rows, reference, manifest)
    imu = imu_disturbance_metrics(rows[IMU_TOPIC], reference, manifest)
    profile = str(manifest.get("profile", ""))
    profile_metrics: dict[str, Any] | None = None
    profile_traces: dict[str, np.ndarray] | None = None
    metrics: dict[str, Any] = {
        "schema_version": 4,
        "analysis_revision": ANALYSIS_REVISION,
        "analyzer_sha256": analyzer_sha256(),
        "run_id": run_id,
        "time_reference": time_reference,
        "tracking": tracking,
        "vicon_source_stamp_quality": source_stamp_quality,
        "vicon_primary_window_quality": primary_window_quality,
        "actuator_limits": actuator,
        "imu_disturbance_proxies": imu,
    }
    if profile in TRANSLATION_PROFILES:
        profile_metrics, profile_traces = translation_metrics(traces, manifest)
        metrics["translation"] = profile_metrics
    elif profile in REVERSAL_PROFILES:
        profile_metrics, profile_traces = reversal_metrics(traces, manifest)
        metrics["reversal"] = profile_metrics
    elif profile.startswith("triangle"):
        profile_metrics = detect_triangle_corners(reference, traces)
        metrics["triangle"] = profile_metrics
    elif profile.startswith("circle"):
        profile_metrics, profile_traces = circle_metrics(
            reference, traces, manifest
        )
        metrics["circle"] = profile_metrics
    else:
        warnings.append(f"no profile-specific metrics implemented for {profile!r}")

    complete = bool(lifecycle["complete"])
    endpoint_coverage_ok = (
        tracking["first_reference_match_dt_s"] <= MAX_POSE_MATCH_DT_S
        and tracking["last_reference_match_dt_s"] <= MAX_POSE_MATCH_DT_S
    )
    pose_quality_ok = pose_quality_valid_for_aggregate(pose_quality)
    identity_ok = bool(manifest_run_id) and run_id == bag.name
    condition_checks = study_condition_validation(manifest)
    start_yaw_ok = condition_checks["formal_start_yaw_valid"]
    caster_ok = condition_checks["caster_type_valid"]
    terrain_rule_ok = condition_checks["terrain_rule_valid"]
    if not start_yaw_ok:
        warnings.append(
            "captured/nominal base start-yaw protocol metadata is missing or outside tolerance"
        )
    if not caster_ok:
        warnings.append("caster_type must be specified for formal aggregation")
    if not terrain_rule_ok:
        warnings.append(
            "analysis v4 formal pose envelope is validated only for flat terrain; "
            "use a predeclared terrain-specific rule before pooling other terrain"
        )
    valid_for_aggregate = (
        complete
        and tracking["coverage_fraction"] >= 0.90
        and endpoint_coverage_ok
        and primary_window_quality["valid"]
        and source_stamp_quality["valid"]
        and pose_quality_ok
        and identity_ok
        and profile_integrity["valid"]
        and start_yaw_ok
        and caster_ok
        and terrain_rule_ok
        and not exclude_reason
    )
    status = "excluded" if exclude_reason else ("complete" if complete else "partial")
    validation = {
        "schema_version": 4,
        "analysis_revision": ANALYSIS_REVISION,
        "analyzer_sha256": metrics["analyzer_sha256"],
        "run_id": run_id,
        "bag_path": str(bag),
        "analyzed_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "complete": complete,
        "valid_for_aggregate": valid_for_aggregate,
        "exclude_reason": exclude_reason,
        "incomplete_reasons": incomplete_reasons,
        "warnings": warnings,
        "metadata_preflight": reason,
        "topic_counts": counts,
        "profile_integrity": profile_integrity,
        "study_condition_checks": condition_checks,
        "reference_lifecycle": lifecycle,
        "vicon_stream": vicon_quality,
        "vicon_source_stamp_quality": source_stamp_quality,
        "vicon_pose_quality": pose_quality,
        "vicon_primary_window_quality": primary_window_quality,
        "whole_bag_vicon_stream_diagnostic": whole_bag_vicon_quality,
        "whole_bag_vicon_pose_diagnostic": whole_bag_pose_quality,
    }

    output.mkdir(parents=True, exist_ok=True)
    write_json(output / "manifest.json", manifest)
    write_json(output / "metrics.json", metrics)
    artifacts = write_plots(
        output,
        bag,
        manifest,
        traces,
        profile_metrics,
        profile_traces,
        time_reference,
    )
    artifacts.append(
        write_video_timeline(
            output / "video_timeline.csv", traces, time_reference
        )
    )
    actuator_artifacts, actuator_warning = run_actuator_analysis(
        bag, output, counts, manifest
    )
    artifacts.extend(actuator_artifacts)
    if actuator_warning:
        validation["warnings"].append(actuator_warning)
    write_json(output / "validation.json", validation)

    row = build_summary_row(bag, manifest, validation, metrics)
    write_csv_rows(output / "summary.csv", RUN_FIELDS, [row])
    return manifest, validation, metrics, artifacts


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bag",
        nargs="?",
        type=Path,
        help="specific bag directory (default: newest closed hamr_study_* bag)",
    )
    parser.add_argument("--bag-root", type=Path, default=default_bag_root())
    parser.add_argument("--pattern", default=DEFAULT_PATTERN)
    parser.add_argument(
        "--study-results",
        type=Path,
        help="aggregate output directory (default: BAG_ROOT/study_results)",
    )
    parser.add_argument(
        "--plan",
        type=Path,
        help=(
            "immutable planned-trials CSV (default: HAMR_STUDY_PLAN, then "
            "STUDY_RESULTS/planned_trials.csv)"
        ),
    )
    parser.add_argument(
        "--no-aggregate",
        action="store_true",
        help="write only this bag's analysis directory",
    )
    exclusion = parser.add_mutually_exclusive_group()
    exclusion.add_argument(
        "--exclude-reason",
        default=None,
        help=(
            "operator reason to retain but exclude this run from aggregate "
            "metrics (for example, corridor intervention)"
        ),
    )
    exclusion.add_argument(
        "--include",
        action="store_true",
        help="explicitly clear a previously recorded operator exclusion",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.exclude_reason is not None and not args.exclude_reason.strip():
        raise RuntimeError("--exclude-reason must be nonempty; use --include to clear")
    if args.exclude_reason is not None and len(args.exclude_reason) > 1000:
        raise RuntimeError("--exclude-reason must be at most 1000 characters")
    bag_root = args.bag_root.expanduser().resolve()
    if args.bag is None:
        bag, rejected = select_latest_study_bag(bag_root, args.pattern)
        for item in rejected:
            print(f"Skipped newer bag: {item}", file=sys.stderr)
    else:
        bag = args.bag.expanduser().resolve()
        if not bag.is_dir():
            raise RuntimeError(f"bag directory does not exist: {bag}")
    output = bag / "analysis"
    if args.include:
        exclude_reason = ""
    elif args.exclude_reason is not None:
        exclude_reason = args.exclude_reason
    else:
        exclude_reason = existing_exclusion(output / "validation.json")
    manifest, validation, metrics, artifacts = analyze_bag(
        bag, output, exclude_reason=exclude_reason
    )
    row = build_summary_row(bag, manifest, validation, metrics)

    aggregate_artifacts: list[Path] = []
    if not args.no_aggregate:
        results_dir = (
            args.study_results.expanduser().resolve()
            if args.study_results
            else bag_root / "study_results"
        )
        aggregate_artifacts.extend(
            update_study_results(results_dir, row, plan_path=args.plan)
        )

    print(f"Selected bag: {bag}")
    print(
        "Validation: "
        f"{validation['status']}; "
        f"valid_for_aggregate={str(validation['valid_for_aggregate']).lower()}"
    )
    for reason in validation["incomplete_reasons"]:
        print(f"Incomplete: {reason}", file=sys.stderr)
    for warning in validation["warnings"]:
        print(f"Warning: {warning}", file=sys.stderr)
    print("Analysis artifacts:")
    for path in (
        output / "manifest.json",
        output / "validation.json",
        output / "metrics.json",
        output / "summary.csv",
        *artifacts,
        *aggregate_artifacts,
    ):
        print(f"  {path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as exc:
        print(f"analyze_hamr_study: {exc}", file=sys.stderr)
        raise SystemExit(1)
