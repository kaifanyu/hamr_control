#!/usr/bin/env python3
"""Offline acceptance analysis for a ``firmware_pid_straight`` HAMR bag.

The test profile intentionally does not depend on Vicon, a reference
trajectory, or live outer-loop gains.  Firmware acceptance therefore uses only
the direct wheel/turret commands, physical wheel encoder streams, firmware
wheel status, and protocol-v4 encoder diagnostics recorded by
``record_hamr_test_bag``.  When Vicon base odometry happens to be present, it is
treated as measurement-only: source-stamp and pose-geometry checks must pass
before ground speed or effective rolling radius is reported, and its result
never changes the encoder-only acceptance verdict.

It writes a JSON report and a response/stop PNG.  By default a completed
analysis exits successfully even when a controller performance check fails;
use ``--strict`` when a pass/fail exit status is useful in automation.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


TOPICS = {
    "left_cmd": "/left_wheel/cmd_vel",
    "right_cmd": "/right_wheel/cmd_vel",
    "turret_cmd": "/turret/cmd_vel",
    "left_ticks": "/left_wheel/encoder_ticks",
    "right_ticks": "/right_wheel/encoder_ticks",
    "status": "/wheel_control/status",
    "diagnostics": "/wheel_encoder/diagnostics",
}
OPTIONAL_TOPICS = {
    "vicon": "/HAMR_base/odom",
}

STATUS_SCHEMA = "wheel_control_status_v1"
STATUS_FIELDS = (
    "left_target_rpm",
    "left_measured_rpm",
    "left_ff_pwm",
    "left_pid_pwm",
    "left_output_pwm",
    "left_error_rpm",
    "left_integral_rpm_s",
    "left_saturated",
    "right_target_rpm",
    "right_measured_rpm",
    "right_ff_pwm",
    "right_pid_pwm",
    "right_output_pwm",
    "right_error_rpm",
    "right_integral_rpm_s",
    "right_saturated",
    "source",
)
STATUS_INDEX = {name: index for index, name in enumerate(STATUS_FIELDS)}

DIAGNOSTIC_SCHEMA = "wheel_encoder_diag_v1"
DIAGNOSTIC_WHEEL_FIELDS = (
    "legacy_ticks",
    "quadrature_count",
    "a_rise",
    "a_fall",
    "b_rise",
    "b_fall",
    "valid_positive",
    "valid_negative",
    "invalid_transitions",
    "duplicate_samples",
    "ab_state",
)
DIAGNOSTIC_WIDTH = 2 * len(DIAGNOSTIC_WHEEL_FIELDS)

SOURCE_NAMES = {
    0: "NONE",
    1: "UART_ROS",
    2: "USB_SERIAL",
    3: "UDP",
    4: "WEB",
    5: "UART_ZERO",
    6: "TOF_STOP",
    7: "COMMS_TIMEOUT",
}

LEFT = "#1764ab"
RIGHT = "#c43b32"
TURRET = "#6f4e9c"
GREEN = "#238b45"
ORANGE = "#e88b00"


@dataclass(frozen=True)
class Config:
    expected_level_rad_s: float = 1.35
    expected_ramp_s: float = 0.50
    expected_hold_s: float = 1.50
    measure_s: float = 1.00
    ticks_per_rev: float = 2263.7
    forward_ff_pwm_per_rpm: float = 175.0
    command_pair_tolerance_s: float = 0.012
    telemetry_freshness_s: float = 0.20
    source_grace_s: float = 0.20
    direction_grace_s: float = 0.35
    final_zero_timeout_s: float = 1.00
    max_wheel_ticks: float = 1200.0
    max_differential_ticks: float = 120.0
    speed_tolerance_fraction: float = 0.10
    wheel_mismatch_fraction: float = 0.05
    overshoot_fraction: float = 0.15
    rolling_window_s: float = 0.20
    settle_limit_s: float = 0.50
    max_median_pid_pwm: float = 250.0
    max_median_integral_rpm_s: float = 5.0
    max_pwm_step_p95: float = 250.0
    max_pwm_step: float = 400.0
    assumed_wheel_radius_m: float = 0.122
    vicon_live_source_age_limit_s: float = 0.10
    vicon_max_abs_receipt_age_s: float = 0.50
    vicon_min_samples: int = 20
    vicon_min_span_s: float = 0.75
    vicon_max_source_gap_s: float = 0.05
    vicon_min_z_m: float = 0.25
    vicon_max_z_m: float = 0.40
    vicon_max_tilt_rad: float = 0.35
    vicon_quaternion_norm_tolerance: float = 0.05
    vicon_max_position_step_m: float = 0.08
    vicon_max_fit_residual_m: float = 0.03
    vicon_max_yaw_change_rad: float = 0.15
    vicon_min_dynamic_samples: int = 50
    vicon_dynamic_pre_s: float = 0.50
    vicon_max_response_lag_s: float = 0.30
    vicon_lag_grid_s: float = 0.001

    def validate(self) -> None:
        values = tuple(self.__dict__.values())
        if not all(math.isfinite(float(value)) and float(value) > 0.0 for value in values):
            raise ValueError("all analyzer configuration values must be finite and positive")
        if self.vicon_min_z_m >= self.vicon_max_z_m:
            raise ValueError("Vicon minimum z must be below maximum z")
        if self.expected_ramp_s + self.expected_hold_s > 2.0 + 1e-12:
            raise ValueError("expected firmware-PID motion may not exceed 2.00 s")
        if self.measure_s > self.expected_hold_s:
            raise ValueError("measure_s may not exceed expected_hold_s")


def storage_id(bag: Path) -> str:
    metadata = bag / "metadata.yaml"
    if metadata.exists():
        for line in metadata.read_text(encoding="utf-8").splitlines():
            stripped = line.strip()
            if stripped.startswith("storage_identifier:"):
                return stripped.split(":", 1)[1].strip()
            if stripped.startswith("storage_id:"):
                return stripped.split(":", 1)[1].strip()
    if any(bag.glob("*.mcap")):
        return "mcap"
    if any(bag.glob("*.db3")):
        return "sqlite3"
    raise RuntimeError(f"cannot determine rosbag storage type for {bag}")


def _layout_label(message: object) -> str:
    layout = getattr(message, "layout", None)
    dimensions = getattr(layout, "dim", ()) if layout is not None else ()
    return str(dimensions[0].label) if dimensions else ""


def read_bag(bag: Path) -> dict[str, Any]:
    """Read required profile topics plus optional measurement-only Vicon."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:  # pragma: no cover - depends on ROS environment
        raise RuntimeError("source the ROS 2 Jazzy environment before analysis") from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id(bag)),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = sorted(set(TOPICS.values()) - set(topic_types))
    if missing:
        raise RuntimeError("bag is missing required topics: " + ", ".join(missing))

    available_optional = {
        name: topic
        for name, topic in OPTIONAL_TOPICS.items()
        if topic in topic_types
    }
    selected_topics = list(TOPICS.values()) + list(available_optional.values())
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected_topics))
    message_types = {
        topic: get_message(topic_types[topic]) for topic in selected_topics
    }
    names = {topic: name for name, topic in TOPICS.items()}
    names.update({topic: name for name, topic in available_optional.items()})
    rows: dict[str, list[tuple[int, object]]] = {name: [] for name in names.values()}
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        if topic not in names:
            continue
        rows[names[topic]].append(
            (timestamp, deserialize_message(raw, message_types[topic]))
        )

    empty = [name for name in TOPICS if not rows.get(name)]
    if empty:
        raise RuntimeError("required topics are empty: " + ", ".join(empty))

    traces: dict[str, Any] = {}
    for name in ("left_cmd", "right_cmd", "turret_cmd", "left_ticks", "right_ticks"):
        traces[name] = {
            "t": np.asarray([timestamp * 1e-9 for timestamp, _ in rows[name]], dtype=float),
            "v": np.asarray([float(message.data) for _, message in rows[name]], dtype=float),
        }

    for name in ("status", "diagnostics"):
        widths = [len(message.data) for _, message in rows[name]]
        if len(set(widths)) != 1:
            raise RuntimeError(f"{TOPICS[name]} changed width within the bag")
        traces[name] = {
            "t": np.asarray([timestamp * 1e-9 for timestamp, _ in rows[name]], dtype=float),
            "v": np.asarray([message.data for _, message in rows[name]], dtype=float),
            "labels": [_layout_label(message) for _, message in rows[name]],
        }
    if rows.get("vicon"):
        receipt_t = []
        source_t = []
        positions = []
        quaternions = []
        for timestamp, message in rows["vicon"]:
            pose = message.pose.pose
            stamp = message.header.stamp
            receipt_t.append(timestamp * 1e-9)
            source_t.append(float(stamp.sec) + float(stamp.nanosec) * 1e-9)
            positions.append(
                (pose.position.x, pose.position.y, pose.position.z)
            )
            quaternions.append(
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
            )
        traces["vicon"] = {
            "receipt_t": np.asarray(receipt_t, dtype=float),
            "source_t": np.asarray(source_t, dtype=float),
            "position": np.asarray(positions, dtype=float),
            "quaternion": np.asarray(quaternions, dtype=float),
        }
    return traces


def pair_commands(
    left: dict[str, np.ndarray],
    right: dict[str, np.ndarray],
    tolerance_s: float,
) -> tuple[np.ndarray, np.ndarray, int, int]:
    """Pair one left and one right publication without reusing a sample."""
    left_t, left_v = left["t"], left["v"]
    right_t, right_v = right["t"], right["v"]
    pairs: list[tuple[float, float, float]] = []
    i = j = 0
    dropped_left = dropped_right = 0
    while i < left_t.size and j < right_t.size:
        delta = float(left_t[i] - right_t[j])
        if abs(delta) <= tolerance_s:
            pairs.append((max(float(left_t[i]), float(right_t[j])), left_v[i], right_v[j]))
            i += 1
            j += 1
        elif delta < 0.0:
            dropped_left += 1
            i += 1
        else:
            dropped_right += 1
            j += 1
    dropped_left += int(left_t.size - i)
    dropped_right += int(right_t.size - j)
    if not pairs:
        return np.empty(0), np.empty((0, 2)), dropped_left, dropped_right
    values = np.asarray(pairs, dtype=float)
    return values[:, 0], values[:, 1:], dropped_left, dropped_right


def fit_slope(t: np.ndarray, values: np.ndarray) -> float:
    if t.size < 3 or values.size != t.size:
        return math.nan
    shifted = t - t[0]
    centered_t = shifted - np.mean(shifted)
    denominator = float(np.dot(centered_t, centered_t))
    if denominator <= 1e-12:
        return math.nan
    return float(np.dot(centered_t, values - np.mean(values)) / denominator)


def rolling_mean(t: np.ndarray, values: np.ndarray, window_s: float) -> np.ndarray:
    result = np.empty(values.shape, dtype=float)
    for index, timestamp in enumerate(t):
        first = int(np.searchsorted(t, timestamp - window_s, side="left"))
        result[index] = np.mean(values[first:index + 1], axis=0)
    return result


def interval_max_gap(t: np.ndarray, start: float, end: float) -> float:
    inside = t[(t >= start) & (t <= end)]
    if not inside.size:
        return math.inf
    points = np.concatenate(([start], inside, [end]))
    return float(np.max(np.diff(points)))


def median_or_nan(values: np.ndarray) -> float:
    return float(np.median(values)) if values.size else math.nan


def normalized_mismatch(left: float, right: float) -> float:
    denominator = 0.5 * (abs(left) + abs(right))
    return abs(left - right) / denominator if denominator > 0.0 else math.nan


def fit_slope_with_standard_error(
    t: np.ndarray, values: np.ndarray
) -> tuple[float, float, np.ndarray]:
    """Return OLS slope, its standard error, and fitted values."""
    if t.size < 3 or values.size != t.size:
        return math.nan, math.nan, np.full(values.shape, math.nan)
    centered_t = t - np.mean(t)
    denominator = float(np.dot(centered_t, centered_t))
    if denominator <= 1e-12:
        return math.nan, math.nan, np.full(values.shape, math.nan)
    centered_values = values - np.mean(values)
    slope = float(np.dot(centered_t, centered_values) / denominator)
    fitted = np.mean(values) + slope * centered_t
    residual = values - fitted
    variance = float(np.dot(residual, residual) / max(1, t.size - 2))
    standard_error = math.sqrt(max(0.0, variance / denominator))
    return slope, standard_error, fitted


def quaternion_yaw(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion.T
    return np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def analyze_opportunistic_vicon(
    traces: dict[str, Any],
    config: Config,
    measure_start: float,
    measure_end: float,
    left_tick_rate: float,
    right_tick_rate: float,
    command_t: np.ndarray,
    command_rad_s: np.ndarray,
    motion_start: float,
    command_zero: float,
) -> dict[str, Any]:
    """Validate optional Vicon and estimate loaded effective rolling radius.

    Source stamps define the measurement window.  A constant receipt delay may
    exceed the live controller's freshness limit without invalidating an
    offline slope, but the stamps still must be close enough to the recorder's
    clock to align them with the un-stamped wheel commands.
    """
    result: dict[str, Any] = {
        "available": "vicon" in traces,
        "usable": False,
        "measurement_only": True,
        "affects_firmware_verdict": False,
        "reasons": [],
        "ground_speed_m_s": None,
        "ground_speed_95pct_ci_m_s": None,
        "effective_radius_m": None,
        "effective_radius_95pct_ci_m": None,
        "ground_gain_vs_assumed_radius": None,
        "command_to_ground_lag_s": None,
        "lag_usable": False,
        "lag_reasons": [],
        "lag_checks": {},
        "assumed_wheel_radius_m": config.assumed_wheel_radius_m,
        "source_measurement_window_s": [measure_start, measure_end],
        "checks": {},
    }
    if "vicon" not in traces:
        result["reasons"] = ["optional Vicon topic is absent"]
        return result

    trace = traces["vicon"]
    receipt_t = np.asarray(trace.get("receipt_t", []), dtype=float)
    source_t = np.asarray(trace.get("source_t", []), dtype=float)
    position = np.asarray(trace.get("position", []), dtype=float)
    quaternion = np.asarray(trace.get("quaternion", []), dtype=float)
    count = source_t.size
    structure_ok = (
        receipt_t.shape == (count,)
        and position.shape == (count, 3)
        and quaternion.shape == (count, 4)
    )
    result["topic_sample_count"] = int(count)
    result["checks"]["trace_structure"] = make_check(
        {
            "receipt": list(receipt_t.shape),
            "source": list(source_t.shape),
            "position": list(position.shape),
            "quaternion": list(quaternion.shape),
        },
        structure_ok,
        "equal-length receipt/source, Nx3 position, and Nx4 quaternion arrays",
    )
    if not structure_ok:
        result["reasons"] = ["Vicon trace arrays have inconsistent shapes"]
        return result

    finite_all = bool(
        count
        and np.all(np.isfinite(receipt_t))
        and np.all(np.isfinite(source_t))
        and np.all(np.isfinite(position))
        and np.all(np.isfinite(quaternion))
    )
    result["checks"]["finite_trace"] = make_check(
        finite_all, finite_all, "all optional Vicon fields finite"
    )
    if not finite_all:
        result["reasons"] = ["Vicon trace is empty or contains non-finite fields"]
        return result

    selected = (source_t >= measure_start) & (source_t <= measure_end)
    selected_count = int(np.sum(selected))
    t = source_t[selected]
    receipt = receipt_t[selected]
    xyz = position[selected]
    quat = quaternion[selected]
    result["selected_sample_count"] = selected_count
    result["source_topic_range_s"] = [float(source_t[0]), float(source_t[-1])]
    result["checks"]["sample_count"] = make_check(
        selected_count,
        selected_count >= config.vicon_min_samples,
        f"at least {config.vicon_min_samples} source-stamped samples",
    )
    if selected_count < 3:
        result["reasons"] = [
            "fewer than three Vicon source-stamped samples overlap the measurement window"
        ]
        return _safe_json(result)

    source_delta = np.diff(t)
    receipt_delta = np.diff(receipt)
    source_monotonic = bool(np.all(source_delta > 0.0))
    receipt_monotonic = bool(np.all(receipt_delta >= 0.0))
    source_span = float(t[-1] - t[0])
    max_source_gap = float(np.max(source_delta)) if source_delta.size else math.inf
    result.update({
        "source_span_s": source_span,
        "max_source_gap_s": max_source_gap,
        "source_stamp_strictly_monotonic": source_monotonic,
        "receipt_stamp_monotonic": receipt_monotonic,
    })
    result["checks"].update({
        "source_stamp_monotonic": make_check(
            source_monotonic, source_monotonic, "strictly increasing source stamps"
        ),
        "receipt_stamp_monotonic": make_check(
            receipt_monotonic, receipt_monotonic, "nondecreasing bag receipt stamps"
        ),
        "source_span": make_check(
            source_span,
            source_span >= config.vicon_min_span_s,
            f"source-stamped span >= {config.vicon_min_span_s:.2f} s",
        ),
        "source_gap": make_check(
            max_source_gap,
            source_monotonic and max_source_gap <= config.vicon_max_source_gap_s,
            f"maximum source-stamp gap <= {config.vicon_max_source_gap_s:.3f} s",
        ),
    })

    receipt_age = receipt - t
    receipt_age_median = float(np.median(receipt_age))
    receipt_age_p95 = float(np.percentile(receipt_age, 95))
    receipt_age_abs_max = float(np.max(np.abs(receipt_age)))
    live_fresh_fraction = float(
        np.mean(receipt_age <= config.vicon_live_source_age_limit_s)
    )
    result.update({
        "receipt_age_median_s": receipt_age_median,
        "receipt_age_p95_s": receipt_age_p95,
        "receipt_age_abs_max_s": receipt_age_abs_max,
        "live_source_age_limit_s": config.vicon_live_source_age_limit_s,
        "live_fresh_fraction": live_fresh_fraction,
        "stale_for_live_control": live_fresh_fraction < 1.0,
    })
    result["checks"]["source_receipt_alignment"] = make_check(
        receipt_age_abs_max,
        receipt_age_abs_max <= config.vicon_max_abs_receipt_age_s,
        "absolute source/receipt offset <= "
        f"{config.vicon_max_abs_receipt_age_s:.2f} s for command-window alignment",
    )

    quaternion_norm = np.linalg.norm(quat, axis=1)
    nonzero_quaternion = bool(np.all(quaternion_norm > 1e-9))
    normalized_quaternion = quat / np.maximum(quaternion_norm[:, None], 1e-12)
    norm_error = float(np.max(np.abs(quaternion_norm - 1.0)))
    xq, yq, _, _ = normalized_quaternion.T
    body_z_dot_world_z = np.clip(1.0 - 2.0 * (xq * xq + yq * yq), -1.0, 1.0)
    tilt = np.arccos(body_z_dot_world_z)
    max_tilt = float(np.max(tilt))
    yaw = np.unwrap(quaternion_yaw(normalized_quaternion))
    yaw_change = float(np.max(yaw) - np.min(yaw))
    z_min = float(np.min(xyz[:, 2]))
    z_max = float(np.max(xyz[:, 2]))
    position_steps = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    max_position_step = (
        float(np.max(position_steps)) if position_steps.size else math.inf
    )
    result["geometry"] = {
        "quaternion_norm_max_error": norm_error,
        "z_range_m": [z_min, z_max],
        "max_tilt_rad": max_tilt,
        "yaw_change_rad": yaw_change,
        "max_3d_position_step_m": max_position_step,
    }
    result["checks"].update({
        "quaternion_norm": make_check(
            norm_error,
            nonzero_quaternion
            and norm_error <= config.vicon_quaternion_norm_tolerance,
            "nonzero quaternion with max norm error <= "
            f"{config.vicon_quaternion_norm_tolerance:.3f}",
        ),
        "z_envelope": make_check(
            [z_min, z_max],
            z_min >= config.vicon_min_z_m and z_max <= config.vicon_max_z_m,
            f"z in [{config.vicon_min_z_m:.2f}, {config.vicon_max_z_m:.2f}] m",
        ),
        "tilt_envelope": make_check(
            max_tilt,
            max_tilt <= config.vicon_max_tilt_rad,
            f"tilt <= {config.vicon_max_tilt_rad:.2f} rad",
        ),
        "straight_yaw": make_check(
            yaw_change,
            yaw_change <= config.vicon_max_yaw_change_rad,
            f"yaw range <= {config.vicon_max_yaw_change_rad:.2f} rad",
        ),
        "position_step": make_check(
            max_position_step,
            max_position_step <= config.vicon_max_position_step_m,
            f"3-D source-to-source step <= {config.vicon_max_position_step_m:.2f} m",
        ),
    })

    vx, vx_se, fit_x = fit_slope_with_standard_error(t, xyz[:, 0])
    vy, vy_se, fit_y = fit_slope_with_standard_error(t, xyz[:, 1])
    ground_speed = math.hypot(vx, vy)
    if ground_speed > 1e-12:
        ground_speed_se = math.sqrt(
            (vx * vx_se) ** 2 + (vy * vy_se) ** 2
        ) / ground_speed
    else:
        ground_speed_se = math.hypot(vx_se, vy_se)
    ground_ci = [
        max(0.0, ground_speed - 1.96 * ground_speed_se),
        ground_speed + 1.96 * ground_speed_se,
    ]
    horizontal_residual = np.hypot(xyz[:, 0] - fit_x, xyz[:, 1] - fit_y)
    fit_residual_rms = float(math.sqrt(np.mean(horizontal_residual ** 2)))
    fit_residual_max = float(np.max(horizontal_residual))
    result["geometry"].update({
        "horizontal_fit_residual_rms_m": fit_residual_rms,
        "horizontal_fit_residual_max_m": fit_residual_max,
    })
    travel_direction = (
        np.asarray([vx, vy], dtype=float) / ground_speed
        if ground_speed > 1e-12 else np.asarray([math.nan, math.nan])
    )
    result["travel_direction_xy"] = travel_direction
    result["checks"]["straight_line_fit"] = make_check(
        {"rms_m": fit_residual_rms, "max_m": fit_residual_max},
        fit_residual_max <= config.vicon_max_fit_residual_m,
        f"maximum horizontal linear-fit residual <= {config.vicon_max_fit_residual_m:.2f} m",
    )

    mean_tick_rate = 0.5 * (left_tick_rate + right_tick_rate)
    mean_wheel_omega = mean_tick_rate * 2.0 * math.pi / config.ticks_per_rev
    encoder_rate_ok = bool(
        math.isfinite(mean_wheel_omega) and mean_wheel_omega > 1e-6
    )
    result["mean_encoder_wheel_speed_rad_s"] = mean_wheel_omega
    result["checks"]["encoder_rate"] = make_check(
        mean_wheel_omega,
        encoder_rate_ok,
        "finite positive mean encoder wheel speed",
    )

    failed = [name for name, check in result["checks"].items() if not check["pass"]]
    result["reasons"] = [f"failed {name}: {result['checks'][name]['requirement']}" for name in failed]
    result["usable"] = not failed
    if result["usable"]:
        effective_radius = ground_speed / mean_wheel_omega
        effective_radius_ci = [value / mean_wheel_omega for value in ground_ci]
        assumed_ground_speed = config.assumed_wheel_radius_m * mean_wheel_omega
        result.update({
            "ground_speed_m_s": ground_speed,
            "ground_speed_95pct_ci_m_s": ground_ci,
            "effective_radius_m": effective_radius,
            "effective_radius_95pct_ci_m": effective_radius_ci,
            "assumed_radius_encoder_speed_m_s": assumed_ground_speed,
            "ground_gain_vs_assumed_radius": (
                ground_speed / assumed_ground_speed
                if assumed_ground_speed > 0.0 else math.nan
            ),
        })

        # Estimate command-to-ground delay from the full ramp. This is a
        # deliberately separate, quality-gated metric: steady-state speed and
        # radius remain usable even if the short dynamic fit is inconclusive.
        dynamic = (
            (source_t >= motion_start - config.vicon_dynamic_pre_s)
            & (source_t <= command_zero)
        )
        dynamic_t = source_t[dynamic]
        dynamic_xyz = position[dynamic]
        dynamic_quat = quaternion[dynamic]
        lag_checks = result["lag_checks"]
        lag_checks["sample_count"] = make_check(
            int(dynamic_t.size),
            dynamic_t.size >= config.vicon_min_dynamic_samples,
            f"at least {config.vicon_min_dynamic_samples} dynamic-window samples",
        )
        dynamic_delta = np.diff(dynamic_t)
        dynamic_stamps_ok = bool(
            dynamic_t.size >= 3
            and np.all(dynamic_delta > 0.0)
            and np.max(dynamic_delta) <= config.vicon_max_source_gap_s
        )
        lag_checks["source_stamp_continuity"] = make_check(
            float(np.max(dynamic_delta)) if dynamic_delta.size else math.inf,
            dynamic_stamps_ok,
            "strictly monotonic dynamic source stamps with maximum gap <= "
            f"{config.vicon_max_source_gap_s:.3f} s",
        )
        dynamic_geometry_ok = False
        if dynamic_t.size >= 3:
            dynamic_norm = np.linalg.norm(dynamic_quat, axis=1)
            dynamic_normalized = dynamic_quat / np.maximum(
                dynamic_norm[:, None], 1e-12
            )
            dxq, dyq, _, _ = dynamic_normalized.T
            dynamic_tilt = np.arccos(np.clip(
                1.0 - 2.0 * (dxq * dxq + dyq * dyq), -1.0, 1.0
            ))
            dynamic_yaw = np.unwrap(quaternion_yaw(dynamic_normalized))
            dynamic_steps = np.linalg.norm(np.diff(dynamic_xyz, axis=0), axis=1)
            dynamic_geometry_ok = bool(
                np.all(dynamic_norm > 1e-9)
                and np.max(np.abs(dynamic_norm - 1.0))
                <= config.vicon_quaternion_norm_tolerance
                and np.min(dynamic_xyz[:, 2]) >= config.vicon_min_z_m
                and np.max(dynamic_xyz[:, 2]) <= config.vicon_max_z_m
                and np.max(dynamic_tilt) <= config.vicon_max_tilt_rad
                and np.ptp(dynamic_yaw) <= config.vicon_max_yaw_change_rad
                and np.max(dynamic_steps) <= config.vicon_max_position_step_m
            )
        lag_checks["geometry"] = make_check(
            dynamic_geometry_ok,
            dynamic_geometry_ok,
            "valid quaternion/z/tilt/yaw/position geometry throughout dynamic window",
        )

        if all(check["pass"] for check in lag_checks.values()):
            projected = dynamic_xyz[:, :2] @ travel_direction
            best: tuple[float, float, float, float] | None = None
            lag_values = np.arange(
                0.0,
                config.vicon_max_response_lag_s + 0.5 * config.vicon_lag_grid_s,
                config.vicon_lag_grid_s,
            )
            for lag in lag_values:
                shifted_command = np.interp(
                    dynamic_t - lag,
                    command_t,
                    command_rad_s,
                    left=0.0,
                    right=0.0,
                )
                wheel_angle = np.concatenate((
                    [0.0],
                    np.cumsum(
                        0.5
                        * (shifted_command[1:] + shifted_command[:-1])
                        * dynamic_delta
                    ),
                ))
                design = np.column_stack((np.ones(dynamic_t.size), wheel_angle))
                offset, dynamic_gain = np.linalg.lstsq(
                    design, projected, rcond=None
                )[0]
                residual = projected - (offset + dynamic_gain * wheel_angle)
                rms = float(math.sqrt(np.mean(residual * residual)))
                candidate = (rms, float(lag), float(dynamic_gain),
                             float(np.max(np.abs(residual))))
                if best is None or candidate[0] < best[0]:
                    best = candidate
            assert best is not None
            fit_rms, response_lag, dynamic_gain, fit_max = best
            fit_ok = bool(
                math.isfinite(dynamic_gain)
                and dynamic_gain > 0.0
                and response_lag < config.vicon_max_response_lag_s
                and fit_max <= config.vicon_max_fit_residual_m
            )
            lag_checks["integrated_command_fit"] = make_check(
                {
                    "rms_m": fit_rms,
                    "max_m": fit_max,
                    "gain_m_per_rad": dynamic_gain,
                    "best_lag_s": response_lag,
                },
                fit_ok,
                "positive gain, non-boundary lag, and maximum residual <= "
                f"{config.vicon_max_fit_residual_m:.2f} m",
            )
            result["lag_usable"] = fit_ok
            if fit_ok:
                result["command_to_ground_lag_s"] = response_lag
                result["dynamic_fit_gain_m_per_rad"] = dynamic_gain
                result["dynamic_fit_residual_rms_m"] = fit_rms
                result["dynamic_fit_residual_max_m"] = fit_max
                result["lag_method"] = (
                    "source-stamped projected displacement fit to the integrated "
                    "wheel-command ramp; coarse dynamic estimate"
                )
        result["lag_reasons"] = [
            f"failed {name}: {check['requirement']}"
            for name, check in lag_checks.items() if not check["pass"]
        ]
    return _safe_json(result)


def exact_zero_status(row: np.ndarray) -> bool:
    if row.size < len(STATUS_FIELDS):
        return False
    if int(round(row[STATUS_INDEX["source"]])) != 5:
        return False
    fields = (
        "left_target_rpm",
        "left_ff_pwm",
        "left_pid_pwm",
        "left_output_pwm",
        "left_saturated",
        "right_target_rpm",
        "right_ff_pwm",
        "right_pid_pwm",
        "right_output_pwm",
        "right_saturated",
    )
    return all(abs(float(row[STATUS_INDEX[field]])) <= 1e-9 for field in fields)


def _safe_json(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _safe_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_safe_json(item) for item in value]
    if isinstance(value, np.ndarray):
        return _safe_json(value.tolist())
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, (np.integer, int)):
        return int(value)
    return value


def make_check(actual: Any, passed: bool, requirement: str) -> dict[str, Any]:
    return {
        "pass": bool(passed),
        "actual": _safe_json(actual),
        "requirement": requirement,
    }


def counter_delta(values: np.ndarray) -> tuple[int, bool]:
    integers = np.rint(values).astype(np.int64)
    monotonic = bool(np.all(np.diff(integers) >= 0))
    return int(integers[-1] - integers[0]), monotonic


def diagnostic_metrics(
    diag_t: np.ndarray,
    diag: np.ndarray,
    start: float,
    end: float,
    include_prestart_baseline: bool = False,
) -> dict[str, Any]:
    indices = np.flatnonzero((diag_t >= start) & (diag_t <= end))
    if include_prestart_baseline:
        baseline = np.flatnonzero(diag_t <= start)
        if baseline.size:
            indices = np.unique(np.concatenate(([baseline[-1]], indices)))
    if indices.size < 2:
        return {"sample_count": int(indices.size), "span_s": math.nan}
    selected_t = diag_t[indices]
    selected = diag[indices]
    result: dict[str, Any] = {
        "sample_count": int(indices.size),
        "span_s": float(selected_t[-1] - selected_t[0]),
    }
    width = len(DIAGNOSTIC_WHEEL_FIELDS)
    for wheel_index, wheel in enumerate(("left", "right")):
        offset = wheel_index * width
        fields = {
            name: selected[:, offset + field_index]
            for field_index, name in enumerate(DIAGNOSTIC_WHEEL_FIELDS)
        }
        legacy = int(round(fields["legacy_ticks"][-1] - fields["legacy_ticks"][0]))
        quadrature = int(
            round(fields["quadrature_count"][-1] - fields["quadrature_count"][0])
        )
        counters: dict[str, int] = {}
        monotonic = True
        for name in DIAGNOSTIC_WHEEL_FIELDS[2:10]:
            delta, field_monotonic = counter_delta(fields[name])
            counters[name] = delta
            monotonic = monotonic and field_monotonic
        valid = counters["valid_positive"] + counters["valid_negative"]
        classified = valid + counters["invalid_transitions"]
        sampled = classified + counters["duplicate_samples"]
        result[wheel] = {
            "legacy_delta": legacy,
            "legacy_rate_ticks_s": legacy / result["span_s"],
            "quadrature_delta": quadrature,
            "quadrature_rate_counts_s": quadrature / result["span_s"],
            "valid_positive_delta": counters["valid_positive"],
            "valid_negative_delta": counters["valid_negative"],
            "invalid_transition_delta": counters["invalid_transitions"],
            "duplicate_sample_delta": counters["duplicate_samples"],
            "direction_correct_fraction": (
                counters["valid_positive"] / valid if valid else math.nan
            ),
            "invalid_fraction": (
                counters["invalid_transitions"] / classified if classified else 0.0
            ),
            "duplicate_fraction": (
                counters["duplicate_samples"] / sampled if sampled else 0.0
            ),
            "quadrature_to_twice_legacy": (
                quadrature / (2.0 * legacy) if legacy else math.nan
            ),
            "counter_monotonic": monotonic,
        }
    return result


def analyze_traces(
    traces: dict[str, Any],
    config: Config,
    bag_name: str = "synthetic",
) -> dict[str, Any]:
    config.validate()
    status_t = np.asarray(traces["status"]["t"], dtype=float)
    status = np.asarray(traces["status"]["v"], dtype=float)
    diag_t = np.asarray(traces["diagnostics"]["t"], dtype=float)
    diagnostics = np.asarray(traces["diagnostics"]["v"], dtype=float)
    if status.ndim != 2 or status.shape[1] < len(STATUS_FIELDS):
        raise RuntimeError(
            f"{TOPICS['status']} must contain at least {len(STATUS_FIELDS)} fields"
        )
    if diagnostics.ndim != 2 or diagnostics.shape[1] < DIAGNOSTIC_WIDTH:
        raise RuntimeError(
            f"{TOPICS['diagnostics']} must contain at least {DIAGNOSTIC_WIDTH} fields"
        )
    if not np.all(np.isfinite(status[:, :len(STATUS_FIELDS)])):
        raise RuntimeError("wheel-controller status contains non-finite values")
    if not np.all(np.isfinite(diagnostics[:, :DIAGNOSTIC_WIDTH])):
        raise RuntimeError("encoder diagnostics contain non-finite values")

    pair_t, pair_v, dropped_left, dropped_right = pair_commands(
        traces["left_cmd"], traces["right_cmd"], config.command_pair_tolerance_s
    )
    if not pair_t.size:
        raise RuntimeError("no left/right wheel commands could be paired")
    active = np.max(np.abs(pair_v), axis=1) > 1e-6
    active_indices = np.flatnonzero(active)
    if not active_indices.size:
        raise RuntimeError("bag contains no nonzero wheel command")
    first_active = int(active_indices[0])
    last_active = int(active_indices[-1])
    motion_start = float(pair_t[first_active])
    last_active_time = float(pair_t[last_active])

    post_indices = np.flatnonzero((np.arange(pair_t.size) > last_active) & ~active)
    command_zero = float(pair_t[post_indices[0]]) if post_indices.size else math.nan
    analysis_end = command_zero if math.isfinite(command_zero) else float(pair_t[last_active])

    active_gaps = np.diff(pair_t[active_indices])
    episode_count = 1 + int(np.sum(active_gaps > 0.10))
    level_tolerance = max(0.01, 0.01 * config.expected_level_rad_s)
    at_level = np.all(
        np.abs(pair_v - config.expected_level_rad_s) <= level_tolerance,
        axis=1,
    ) & active
    plateau_start = math.nan
    if at_level[last_active]:
        plateau_first = last_active
        while plateau_first > first_active and at_level[plateau_first - 1]:
            plateau_first -= 1
        plateau_start = float(pair_t[plateau_first])

    motion_duration = analysis_end - motion_start
    ramp_duration = plateau_start - motion_start if math.isfinite(plateau_start) else math.nan
    hold_duration = analysis_end - plateau_start if math.isfinite(plateau_start) else math.nan
    active_values = pair_v[active]
    max_pair_difference = float(np.max(np.abs(active_values[:, 0] - active_values[:, 1])))
    min_active_command = float(np.min(active_values))

    turret_v = np.asarray(traces["turret_cmd"]["v"], dtype=float)
    turret_max = float(np.max(np.abs(turret_v)))
    expected_rpm = config.expected_level_rad_s * 60.0 / (2.0 * math.pi)
    expected_tick_rate = expected_rpm * config.ticks_per_rev / 60.0
    expected_quadrature_rate = 2.0 * expected_tick_rate
    expected_ff_pwm = expected_rpm * config.forward_ff_pwm_per_rpm

    status_labels = traces["status"].get("labels", [])
    diag_labels = traces["diagnostics"].get("labels", [])
    status_schema_ok = bool(status_labels) and all(
        label == STATUS_SCHEMA for label in status_labels
    )
    diag_schema_ok = bool(diag_labels) and all(
        label == DIAGNOSTIC_SCHEMA for label in diag_labels
    )

    # The paired zero timestamp is the later of the sequential left/right ROS
    # publications. Firmware can acknowledge the first zero a few milliseconds
    # before that timestamp, so exclude post-stop rows from active-response
    # checks by ending at the final paired nonzero command.
    active_status_mask = (status_t >= motion_start) & (status_t <= last_active_time)
    active_status_t = status_t[active_status_mask]
    active_status = status[active_status_mask, :len(STATUS_FIELDS)]
    if not active_status_t.size:
        raise RuntimeError("no firmware status samples overlap motion")
    sources = np.rint(active_status[:, STATUS_INDEX["source"]]).astype(int)
    elapsed_status = active_status_t - motion_start
    source_allowed = np.isin(sources, (1, 5))
    source_after_grace = sources[elapsed_status > config.source_grace_s]
    active_saturation_count = int(
        np.sum(
            (active_status[:, STATUS_INDEX["left_saturated"]] > 0.5)
            | (active_status[:, STATUS_INDEX["right_saturated"]] > 0.5)
        )
    )
    status_target_difference = float(
        np.max(
            np.abs(
                active_status[:, STATUS_INDEX["left_target_rpm"]]
                - active_status[:, STATUS_INDEX["right_target_rpm"]]
            )
        )
    )
    reverse_target_count = int(
        np.sum(
            (active_status[:, STATUS_INDEX["left_target_rpm"]] < -1e-9)
            | (active_status[:, STATUS_INDEX["right_target_rpm"]] < -1e-9)
        )
    )
    reverse_output_count = int(
        np.sum(
            (active_status[:, STATUS_INDEX["left_output_pwm"]] < -1e-9)
            | (active_status[:, STATUS_INDEX["right_output_pwm"]] < -1e-9)
        )
    )
    after_direction_grace = elapsed_status > config.direction_grace_s
    reverse_measured_count = int(
        np.sum(
            after_direction_grace
            & (
                (active_status[:, STATUS_INDEX["left_measured_rpm"]] < -0.5)
                | (active_status[:, STATUS_INDEX["right_measured_rpm"]] < -0.5)
            )
        )
    )

    pre_status_indices = np.flatnonzero(status_t < motion_start)
    prezero_ack = bool(
        pre_status_indices.size and exact_zero_status(status[pre_status_indices[-1]])
    )
    final_ack_t = math.nan
    if math.isfinite(command_zero):
        for index in np.flatnonzero(status_t >= command_zero):
            if exact_zero_status(status[index]):
                final_ack_t = float(status_t[index])
                break
    final_zero_latency = final_ack_t - command_zero if math.isfinite(final_ack_t) else math.nan

    left_tick_t = np.asarray(traces["left_ticks"]["t"], dtype=float)
    left_ticks = np.asarray(traces["left_ticks"]["v"], dtype=float)
    right_tick_t = np.asarray(traces["right_ticks"]["t"], dtype=float)
    right_ticks = np.asarray(traces["right_ticks"]["v"], dtype=float)

    def tick_delta(t: np.ndarray, values: np.ndarray) -> float:
        before = np.flatnonzero(t <= motion_start)
        through_end = np.flatnonzero(t <= analysis_end)
        if not before.size or not through_end.size:
            return math.nan
        return float(values[through_end[-1]] - values[before[-1]])

    left_tick_delta = tick_delta(left_tick_t, left_ticks)
    right_tick_delta = tick_delta(right_tick_t, right_ticks)
    tick_difference = abs(left_tick_delta - right_tick_delta)

    full_diag = diagnostic_metrics(
        diag_t,
        diagnostics,
        motion_start,
        analysis_end,
        include_prestart_baseline=True,
    )
    measure_end = last_active_time
    measure_start = max(
        plateau_start if math.isfinite(plateau_start) else motion_start,
        measure_end - config.measure_s,
    )
    measure_diag = diagnostic_metrics(
        diag_t, diagnostics, measure_start, measure_end
    )

    left_measure = (left_tick_t >= measure_start) & (left_tick_t <= measure_end)
    right_measure = (right_tick_t >= measure_start) & (right_tick_t <= measure_end)
    left_tick_rate = fit_slope(left_tick_t[left_measure], left_ticks[left_measure])
    right_tick_rate = fit_slope(right_tick_t[right_measure], right_ticks[right_measure])

    measure_status_mask = (status_t >= measure_start) & (status_t <= measure_end)
    measure_status_t = status_t[measure_status_mask]
    measure_status = status[measure_status_mask, :len(STATUS_FIELDS)]
    if measure_status_t.size < 3:
        raise RuntimeError("fewer than three firmware status samples in measurement window")
    rolling = rolling_mean(
        active_status_t,
        active_status[:, [
            STATUS_INDEX["left_measured_rpm"],
            STATUS_INDEX["right_measured_rpm"],
        ]],
        config.rolling_window_s,
    )
    rolling_ready = active_status_t >= plateau_start + config.rolling_window_s
    rolling_plateau = rolling_ready & (active_status_t <= analysis_end)
    speed_low = expected_rpm * (1.0 - config.speed_tolerance_fraction)
    speed_high = expected_rpm * (1.0 + config.speed_tolerance_fraction)
    rolling_in_band = np.all((rolling >= speed_low) & (rolling <= speed_high), axis=1)
    settling_time = math.nan
    candidates = np.flatnonzero(rolling_plateau & rolling_in_band)
    for candidate in candidates:
        remainder = rolling_plateau & (np.arange(active_status_t.size) >= candidate)
        if np.all(rolling_in_band[remainder]):
            settling_time = float(active_status_t[candidate] - plateau_start)
            break
    rolling_peak = (
        float(np.max(rolling[rolling_plateau])) if np.any(rolling_plateau) else math.nan
    )

    controller: dict[str, Any] = {
        "measurement_window_s": [measure_start - motion_start, measure_end - motion_start],
        "status_samples": int(measure_status_t.size),
        "status_max_gap_s": interval_max_gap(
            status_t, motion_start, last_active_time
        ),
        "active_source_counts": {
            SOURCE_NAMES.get(int(source), f"UNKNOWN_{source}"): int(np.sum(sources == source))
            for source in np.unique(sources)
        },
        "active_saturation_count": active_saturation_count,
        "max_target_difference_rpm": status_target_difference,
        "reverse_target_count": reverse_target_count,
        "reverse_output_count": reverse_output_count,
        "reverse_measured_count_after_grace": reverse_measured_count,
        "prezero_acknowledged": prezero_ack,
        "final_zero_ack_s_after_command": final_zero_latency,
        "rolling_window_s": config.rolling_window_s,
        "rolling_peak_rpm": rolling_peak,
        "settling_time_s_after_plateau": settling_time,
    }
    for wheel in ("left", "right"):
        def column(suffix: str) -> np.ndarray:
            return measure_status[:, STATUS_INDEX[f"{wheel}_{suffix}"]]

        output_steps = np.abs(np.diff(column("output_pwm")))
        controller[wheel] = {
            "target_rpm_median": median_or_nan(column("target_rpm")),
            "measured_rpm_median": median_or_nan(column("measured_rpm")),
            "ff_pwm_median": median_or_nan(column("ff_pwm")),
            "pid_pwm_median": median_or_nan(column("pid_pwm")),
            "abs_pid_pwm_median": median_or_nan(np.abs(column("pid_pwm"))),
            "output_pwm_median": median_or_nan(column("output_pwm")),
            "error_rpm_median": median_or_nan(column("error_rpm")),
            "integral_rpm_s_median": median_or_nan(column("integral_rpm_s")),
            "abs_integral_rpm_s_median": median_or_nan(
                np.abs(column("integral_rpm_s"))
            ),
            "output_step_pwm_p95": (
                float(np.percentile(output_steps, 95)) if output_steps.size else math.nan
            ),
            "output_step_pwm_max": (
                float(np.max(output_steps)) if output_steps.size else math.nan
            ),
        }

    encoder = {
        "left_motion_delta_ticks": left_tick_delta,
        "right_motion_delta_ticks": right_tick_delta,
        "motion_differential_ticks": tick_difference,
        "left_measure_rate_ticks_s": left_tick_rate,
        "right_measure_rate_ticks_s": right_tick_rate,
        "measure_rate_mismatch_fraction": normalized_mismatch(
            left_tick_rate, right_tick_rate
        ),
        "left_max_gap_s": interval_max_gap(left_tick_t, motion_start, analysis_end),
        "right_max_gap_s": interval_max_gap(right_tick_t, motion_start, analysis_end),
    }
    opportunistic_vicon = analyze_opportunistic_vicon(
        traces,
        config,
        measure_start,
        measure_end,
        left_tick_rate,
        right_tick_rate,
        pair_t,
        np.mean(pair_v, axis=1),
        motion_start,
        command_zero,
    )

    acquisition_checks = {
        "status_schema": make_check(status_schema_ok, status_schema_ok, STATUS_SCHEMA),
        "diagnostic_schema": make_check(diag_schema_ok, diag_schema_ok, DIAGNOSTIC_SCHEMA),
        "one_motion_episode": make_check(episode_count, episode_count == 1, "exactly 1"),
        "paired_command_samples": make_check(
            int(active_indices.size), active_indices.size >= 50, "at least 50 active pairs"
        ),
        "equal_forward_commands": make_check(
            {"max_difference_rad_s": max_pair_difference, "minimum_rad_s": min_active_command},
            max_pair_difference <= 1e-6 and min_active_command >= 0.0,
            "equal, forward-positive wheel commands",
        ),
        "turret_zero": make_check(turret_max, turret_max <= 1e-9, "max |command| <= 1e-9"),
        "motion_duration": make_check(
            motion_duration,
            abs(motion_duration - (config.expected_ramp_s + config.expected_hold_s)) <= 0.12,
            f"{config.expected_ramp_s + config.expected_hold_s:.2f} +/- 0.12 s",
        ),
        "ramp_duration": make_check(
            ramp_duration,
            math.isfinite(ramp_duration) and abs(ramp_duration - config.expected_ramp_s) <= 0.08,
            f"{config.expected_ramp_s:.2f} +/- 0.08 s",
        ),
        "hold_duration": make_check(
            hold_duration,
            math.isfinite(hold_duration) and abs(hold_duration - config.expected_hold_s) <= 0.08,
            f"{config.expected_hold_s:.2f} +/- 0.08 s",
        ),
        "prezero_ack": make_check(prezero_ack, prezero_ack, "fresh complete source=5 zero"),
        "active_sources": make_check(
            controller["active_source_counts"],
            bool(np.all(source_allowed))
            and bool(source_after_grace.size)
            and bool(np.all(source_after_grace == 1)),
            "only source 1/5; source 1 exclusively after 0.20 s",
        ),
        "status_freshness": make_check(
            controller["status_max_gap_s"],
            controller["status_max_gap_s"] <= config.telemetry_freshness_s,
            f"max active gap <= {config.telemetry_freshness_s:.2f} s",
        ),
        "encoder_freshness": make_check(
            max(encoder["left_max_gap_s"], encoder["right_max_gap_s"]),
            max(encoder["left_max_gap_s"], encoder["right_max_gap_s"])
            <= config.telemetry_freshness_s,
            f"max active gap <= {config.telemetry_freshness_s:.2f} s",
        ),
        "diagnostic_freshness": make_check(
            interval_max_gap(diag_t, motion_start, analysis_end),
            interval_max_gap(diag_t, motion_start, analysis_end)
            <= config.telemetry_freshness_s,
            f"max active gap <= {config.telemetry_freshness_s:.2f} s",
        ),
        "no_saturation": make_check(
            active_saturation_count, active_saturation_count == 0, "0 active samples"
        ),
        "forward_firmware_state": make_check(
            {
                "reverse_targets": reverse_target_count,
                "reverse_outputs": reverse_output_count,
                "reverse_measured_after_grace": reverse_measured_count,
                "max_target_difference_rpm": status_target_difference,
            },
            reverse_target_count == 0
            and reverse_output_count == 0
            and reverse_measured_count == 0
            and status_target_difference <= 0.25,
            "no reverse state and target pair difference <= 0.25 rpm",
        ),
        "bounded_tick_travel": make_check(
            {
                "left": left_tick_delta,
                "right": right_tick_delta,
                "difference": tick_difference,
            },
            left_tick_delta >= 0.0
            and right_tick_delta >= 0.0
            and max(left_tick_delta, right_tick_delta) <= config.max_wheel_ticks
            and tick_difference <= config.max_differential_ticks,
            "forward; each <= 1200 ticks; difference <= 120 ticks",
        ),
        "encoder_diagnostic_integrity": make_check(
            {
                wheel: {
                    key: full_diag.get(wheel, {}).get(key)
                    for key in (
                        "valid_negative_delta",
                        "invalid_transition_delta",
                        "counter_monotonic",
                    )
                }
                for wheel in ("left", "right")
            },
            all(
                full_diag.get(wheel, {}).get("valid_negative_delta") == 0
                and full_diag.get(wheel, {}).get("invalid_transition_delta") == 0
                and full_diag.get(wheel, {}).get("counter_monotonic") is True
                for wheel in ("left", "right")
            ),
            "0 reverse/invalid transitions and monotonic counters",
        ),
        "final_zero_ack": make_check(
            final_zero_latency,
            math.isfinite(final_zero_latency)
            and 0.0 <= final_zero_latency <= config.final_zero_timeout_s,
            f"complete source=5 zero within {config.final_zero_timeout_s:.2f} s",
        ),
    }

    speed_rate_low = expected_tick_rate * (1.0 - config.speed_tolerance_fraction)
    speed_rate_high = expected_tick_rate * (1.0 + config.speed_tolerance_fraction)
    diagnostic_rates = [
        measure_diag.get(wheel, {}).get("legacy_rate_ticks_s", math.nan)
        for wheel in ("left", "right")
    ]
    all_tick_rates = [left_tick_rate, right_tick_rate, *diagnostic_rates]
    diagnostic_mismatch = normalized_mismatch(*diagnostic_rates)
    target_medians = [controller[wheel]["target_rpm_median"] for wheel in ("left", "right")]
    ff_medians = [controller[wheel]["ff_pwm_median"] for wheel in ("left", "right")]
    pid_medians = [controller[wheel]["abs_pid_pwm_median"] for wheel in ("left", "right")]
    integral_medians = [
        controller[wheel]["abs_integral_rpm_s_median"] for wheel in ("left", "right")
    ]
    pwm_step_p95 = [controller[wheel]["output_step_pwm_p95"] for wheel in ("left", "right")]
    pwm_step_max = [controller[wheel]["output_step_pwm_max"] for wheel in ("left", "right")]
    diagnostic_ratios = [
        measure_diag.get(wheel, {}).get("quadrature_to_twice_legacy", math.nan)
        for wheel in ("left", "right")
    ]
    direction_fractions = [
        measure_diag.get(wheel, {}).get("direction_correct_fraction", math.nan)
        for wheel in ("left", "right")
    ]

    performance_checks = {
        "target_rpm": make_check(
            target_medians,
            all(math.isfinite(value) and abs(value - expected_rpm) <= 0.05 for value in target_medians),
            f"each {expected_rpm:.6f} +/- 0.05 rpm",
        ),
        "feedforward_pwm": make_check(
            ff_medians,
            all(math.isfinite(value) and abs(value - expected_ff_pwm) <= 2.0 for value in ff_medians),
            f"each {expected_ff_pwm:.3f} +/- 2 pwm",
        ),
        "encoder_speed": make_check(
            {
                "topic_left": left_tick_rate,
                "topic_right": right_tick_rate,
                "diagnostic_left": diagnostic_rates[0],
                "diagnostic_right": diagnostic_rates[1],
            },
            all(math.isfinite(value) and speed_rate_low <= value <= speed_rate_high for value in all_tick_rates),
            f"each {expected_tick_rate:.3f} +/- {100 * config.speed_tolerance_fraction:.0f}% ticks/s",
        ),
        "wheel_rate_match": make_check(
            {"topic": encoder["measure_rate_mismatch_fraction"], "diagnostic": diagnostic_mismatch},
            math.isfinite(encoder["measure_rate_mismatch_fraction"])
            and math.isfinite(diagnostic_mismatch)
            and encoder["measure_rate_mismatch_fraction"] <= config.wheel_mismatch_fraction
            and diagnostic_mismatch <= config.wheel_mismatch_fraction,
            f"normalized L/R mismatch <= {100 * config.wheel_mismatch_fraction:.0f}%",
        ),
        "rolling_overshoot": make_check(
            rolling_peak,
            math.isfinite(rolling_peak)
            and rolling_peak <= expected_rpm * (1.0 + config.overshoot_fraction),
            f"200 ms rolling peak <= {expected_rpm * (1.0 + config.overshoot_fraction):.3f} rpm",
        ),
        "settling_time": make_check(
            settling_time,
            math.isfinite(settling_time) and settling_time <= config.settle_limit_s,
            f"both 200 ms rolling speeds remain within +/-10% by {config.settle_limit_s:.2f} s",
        ),
        "pid_correction": make_check(
            pid_medians,
            all(math.isfinite(value) and value <= config.max_median_pid_pwm for value in pid_medians),
            f"median |PID| <= {config.max_median_pid_pwm:.0f} pwm",
        ),
        "integral_state": make_check(
            integral_medians,
            all(
                math.isfinite(value) and value <= config.max_median_integral_rpm_s
                for value in integral_medians
            ),
            f"median |integral| <= {config.max_median_integral_rpm_s:.1f} rpm*s",
        ),
        "pwm_step": make_check(
            {"p95": pwm_step_p95, "max": pwm_step_max},
            all(math.isfinite(value) and value <= config.max_pwm_step_p95 for value in pwm_step_p95)
            and all(math.isfinite(value) and value <= config.max_pwm_step for value in pwm_step_max),
            f"p95 <= {config.max_pwm_step_p95:.0f}, max <= {config.max_pwm_step:.0f} pwm",
        ),
        "quadrature_ratio": make_check(
            diagnostic_ratios,
            all(math.isfinite(value) and 0.98 <= value <= 1.02 for value in diagnostic_ratios),
            "quadrature/(2*legacy) in [0.98, 1.02]",
        ),
        "forward_direction": make_check(
            direction_fractions,
            all(math.isfinite(value) and value >= 0.99 for value in direction_fractions),
            "requested-direction valid fraction >= 0.99",
        ),
    }

    acquisition_pass = all(item["pass"] for item in acquisition_checks.values())
    performance_pass = all(item["pass"] for item in performance_checks.values())
    return _safe_json({
        "bag": bag_name,
        "profile": "firmware_pid_straight",
        "expected": {
            "level_rad_s": config.expected_level_rad_s,
            "target_rpm": expected_rpm,
            "legacy_tick_rate_ticks_s": expected_tick_rate,
            "quadrature_rate_counts_s": expected_quadrature_rate,
            "feedforward_pwm": expected_ff_pwm,
            "assumed_wheel_radius_m": config.assumed_wheel_radius_m,
            "assumed_radius_ground_speed_m_s": (
                config.assumed_wheel_radius_m * config.expected_level_rad_s
            ),
            "ideal_ramp_hold_tick_travel": expected_tick_rate
            * (0.5 * config.expected_ramp_s + config.expected_hold_s),
        },
        "metrics": {
            "command": {
                "paired_samples": int(pair_t.size),
                "active_paired_samples": int(active_indices.size),
                "dropped_left_samples": dropped_left,
                "dropped_right_samples": dropped_right,
                "motion_episodes": episode_count,
                "motion_start_bag_s": motion_start,
                "plateau_start_bag_s": plateau_start,
                "command_zero_bag_s": command_zero,
                "motion_duration_s": motion_duration,
                "ramp_duration_s": ramp_duration,
                "hold_duration_s": hold_duration,
                "max_pair_difference_rad_s": max_pair_difference,
                "turret_max_abs_rad_s": turret_max,
            },
            "controller": controller,
            "encoder": encoder,
            "opportunistic_vicon": opportunistic_vicon,
            "diagnostics_motion": full_diag,
            "diagnostics_measurement": measure_diag,
        },
        "checks": {
            "acquisition": acquisition_checks,
            "performance": performance_checks,
        },
        "verdict": {
            "acquisition_pass": acquisition_pass,
            "performance_pass": performance_pass,
            "overall_pass": acquisition_pass and performance_pass,
        },
    })


def plot_report(
    traces: dict[str, Any],
    report: dict[str, Any],
    output: Path,
) -> None:
    command = report["metrics"]["command"]
    motion_start = float(command["motion_start_bag_s"])
    plateau_start = command["plateau_start_bag_s"]
    command_zero = command["command_zero_bag_s"]
    final_latency = report["metrics"]["controller"]["final_zero_ack_s_after_command"]
    final_ack = (
        float(command_zero) + float(final_latency)
        if command_zero is not None and final_latency is not None else None
    )
    expected = report["expected"]

    status_t = np.asarray(traces["status"]["t"], dtype=float)
    status = np.asarray(traces["status"]["v"], dtype=float)
    rolling = rolling_mean(
        status_t,
        status[:, [STATUS_INDEX["left_measured_rpm"], STATUS_INDEX["right_measured_rpm"]]],
        0.20,
    )
    pair_t, pair_v, _, _ = pair_commands(
        traces["left_cmd"], traces["right_cmd"], 0.012
    )

    left_tick_t = np.asarray(traces["left_ticks"]["t"], dtype=float)
    left_ticks = np.asarray(traces["left_ticks"]["v"], dtype=float)
    right_tick_t = np.asarray(traces["right_ticks"]["t"], dtype=float)
    right_ticks = np.asarray(traces["right_ticks"]["v"], dtype=float)
    left_base_index = max(0, int(np.searchsorted(left_tick_t, motion_start, side="right")) - 1)
    right_base_index = max(0, int(np.searchsorted(right_tick_t, motion_start, side="right")) - 1)

    fig, axes_array = plt.subplots(6, 1, figsize=(15, 17), sharex=True)
    axes = list(axes_array)
    axes[0].plot(pair_t - motion_start, pair_v[:, 0], color=LEFT, lw=1.2, label="left")
    axes[0].plot(pair_t - motion_start, pair_v[:, 1], color=RIGHT, lw=1.2, label="right")
    turret_t = np.asarray(traces["turret_cmd"]["t"], dtype=float)
    turret_v = np.asarray(traces["turret_cmd"]["v"], dtype=float)
    axes[0].plot(turret_t - motion_start, turret_v, color=TURRET, lw=0.8, label="turret")
    axes[0].axhline(expected["level_rad_s"], color=ORANGE, ls="--", lw=0.8, label="expected")
    axes[0].set_ylabel("command\n(rad/s)")
    axes[0].legend(loc="upper right", ncol=4)

    st = status_t - motion_start
    axes[1].plot(st, status[:, STATUS_INDEX["left_target_rpm"]], color=LEFT, ls="--", lw=0.8, label="L target")
    axes[1].plot(st, status[:, STATUS_INDEX["right_target_rpm"]], color=RIGHT, ls="--", lw=0.8, label="R target")
    axes[1].plot(st, status[:, STATUS_INDEX["left_measured_rpm"]], color=LEFT, alpha=0.22, lw=0.7, label="L raw")
    axes[1].plot(st, status[:, STATUS_INDEX["right_measured_rpm"]], color=RIGHT, alpha=0.22, lw=0.7, label="R raw")
    axes[1].plot(st, rolling[:, 0], color=LEFT, lw=1.4, label="L 200 ms mean")
    axes[1].plot(st, rolling[:, 1], color=RIGHT, lw=1.4, label="R 200 ms mean")
    low = expected["target_rpm"] * 0.90
    high = expected["target_rpm"] * 1.10
    axes[1].axhspan(low, high, color=GREEN, alpha=0.08, label="+/-10% band")
    axes[1].set_ylabel("firmware speed\n(rpm)")
    axes[1].legend(loc="upper right", ncol=4, fontsize=8)

    axes[2].plot(st, status[:, STATUS_INDEX["left_output_pwm"]], color=LEFT, lw=1.0, label="L output")
    axes[2].plot(st, status[:, STATUS_INDEX["right_output_pwm"]], color=RIGHT, lw=1.0, label="R output")
    axes[2].plot(st, status[:, STATUS_INDEX["left_ff_pwm"]], color=LEFT, ls=":", lw=0.9, label="L FF")
    axes[2].plot(st, status[:, STATUS_INDEX["right_ff_pwm"]], color=RIGHT, ls=":", lw=0.9, label="R FF")
    axes[2].plot(st, status[:, STATUS_INDEX["left_pid_pwm"]], color=LEFT, alpha=0.45, lw=0.7, label="L PID")
    axes[2].plot(st, status[:, STATUS_INDEX["right_pid_pwm"]], color=RIGHT, alpha=0.45, lw=0.7, label="R PID")
    saturation = (status[:, STATUS_INDEX["left_saturated"]] > 0.5) | (status[:, STATUS_INDEX["right_saturated"]] > 0.5)
    if np.any(saturation):
        axes[2].scatter(st[saturation], status[saturation, STATUS_INDEX["left_output_pwm"]], marker="v", color="black", s=20, label="saturation")
    axes[2].set_ylabel("PWM")
    axes[2].legend(loc="upper right", ncol=4, fontsize=8)

    axes[3].plot(left_tick_t - motion_start, left_ticks - left_ticks[left_base_index], color=LEFT, lw=1.1, label="left")
    axes[3].plot(right_tick_t - motion_start, right_ticks - right_ticks[right_base_index], color=RIGHT, lw=1.1, label="right")
    axes[3].axhline(1200, color=ORANGE, ls="--", lw=0.8, label="hard wheel limit")
    axes[3].set_ylabel("encoder\nDelta ticks")
    axes[3].legend(loc="upper left", ncol=3)

    source = np.rint(status[:, STATUS_INDEX["source"]]).astype(int)
    axes[4].step(st, source, where="post", color="#444444", lw=1.0, label="source")
    axes[4].set_yticks(sorted(SOURCE_NAMES))
    axes[4].set_yticklabels([SOURCE_NAMES[key] for key in sorted(SOURCE_NAMES)], fontsize=7)
    axes[4].set_ylabel("firmware source")

    vicon = report["metrics"]["opportunistic_vicon"]
    if vicon["available"] and "vicon" in traces:
        vicon_t = np.asarray(traces["vicon"]["source_t"], dtype=float)
        vicon_xy = np.asarray(traces["vicon"]["position"], dtype=float)[:, :2]
        direction = np.asarray(vicon.get("travel_direction_xy", []), dtype=float)
        if direction.shape == (2,) and np.all(np.isfinite(direction)):
            projected = vicon_xy @ direction
            delta_t = np.diff(vicon_t)
            valid = delta_t > 0.0
            speed_t = 0.5 * (vicon_t[1:] + vicon_t[:-1])
            projected_speed = np.full(delta_t.shape, np.nan)
            projected_speed[valid] = np.diff(projected)[valid] / delta_t[valid]
            finite = np.isfinite(projected_speed)
            if np.any(finite):
                smoothed = rolling_mean(
                    speed_t[finite], projected_speed[finite, None], 0.20
                )[:, 0]
                axes[5].plot(
                    speed_t[finite] - motion_start,
                    projected_speed[finite],
                    color="#888888", alpha=0.22, lw=0.6,
                    label="source-time raw",
                )
                axes[5].plot(
                    speed_t[finite] - motion_start,
                    smoothed,
                    color=GREEN, lw=1.3, label="200 ms mean",
                )
        axes[5].axhline(
            expected["assumed_radius_ground_speed_m_s"],
            color=ORANGE, ls="--", lw=0.9, label="commanded ground speed",
        )
        if vicon["usable"]:
            axes[5].axhline(
                vicon["ground_speed_m_s"], color=TURRET, ls=":", lw=1.1,
                label="Vicon plateau fit",
            )
            lag_text = (
                f"; dynamic lag ~{1000 * vicon['command_to_ground_lag_s']:.0f} ms"
                if vicon.get("lag_usable") else "; dynamic lag unavailable"
            )
            axes[5].text(
                0.01, 0.95,
                f"r_eff={vicon['effective_radius_m']:.5f} m; "
                f"receipt age={1000 * vicon['receipt_age_median_s']:.0f} ms"
                f"{lag_text}",
                transform=axes[5].transAxes, ha="left", va="top", fontsize=8,
            )
        else:
            reason = "; ".join(vicon.get("reasons", [])[:2])
            axes[5].text(
                0.01, 0.95, f"Vicon UNUSABLE: {reason}",
                transform=axes[5].transAxes, ha="left", va="top", fontsize=8,
                color=RIGHT,
            )
    else:
        axes[5].text(
            0.01, 0.95, "optional Vicon absent",
            transform=axes[5].transAxes, ha="left", va="top", fontsize=8,
        )
    axes[5].set_ylabel("Vicon ground\nspeed (m/s)")
    axes[5].set_xlabel("seconds from first nonzero wheel command")
    axes[5].legend(loc="upper right", ncol=4, fontsize=8)

    marks = [(0.0, "motion start", "#444444")]
    if plateau_start is not None:
        marks.append((float(plateau_start) - motion_start, "plateau", ORANGE))
    if command_zero is not None:
        marks.append((float(command_zero) - motion_start, "command zero", GREEN))
    if final_ack is not None:
        marks.append((final_ack - motion_start, "firmware zero ack", TURRET))
    for value, label, color in marks:
        for axis in axes:
            axis.axvline(value, color=color, ls="--", lw=0.9, alpha=0.75)
        axes[0].text(
            value,
            0.98,
            label,
            color=color,
            rotation=90,
            ha="right",
            va="top",
            fontsize=8,
            transform=axes[0].get_xaxis_transform(),
        )
    for axis in axes:
        axis.grid(True, alpha=0.22)
    plot_end = (
        float(command_zero) - motion_start + 1.0
        if command_zero is not None else float(pair_t[-1] - motion_start)
    )
    if final_ack is not None:
        plot_end = max(plot_end, final_ack - motion_start + 0.50)
    for axis in axes:
        axis.set_xlim(-1.0, plot_end)

    acquisition = report["verdict"]["acquisition_pass"]
    performance = report["verdict"]["performance_pass"]
    vicon_state = (
        "usable offline / stale live"
        if vicon.get("usable") and vicon.get("stale_for_live_control")
        else "usable" if vicon.get("usable")
        else "unusable" if vicon.get("available") else "absent"
    )
    fig.suptitle(
        f"HAMR encoder-only firmware PI response - {Path(report['bag']).name}\n"
        f"acquisition {'PASS' if acquisition else 'FAIL'} | "
        f"controller performance {'PASS' if performance else 'FAIL'} | "
        f"Vicon {vicon_state}",
        fontsize=12,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170)
    plt.close(fig)


def failed_checks(report: dict[str, Any], group: str) -> list[str]:
    return [
        name
        for name, result in report["checks"][group].items()
        if not result["pass"]
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory")
    parser.add_argument(
        "--output-prefix",
        type=Path,
        help="output prefix (default: next to bag, suffixed _firmware_pid)",
    )
    parser.add_argument("--expected-level-rad-s", type=float, default=1.35)
    parser.add_argument("--expected-ramp-s", type=float, default=0.50)
    parser.add_argument("--expected-hold-s", type=float, default=1.50)
    parser.add_argument("--measure-s", type=float, default=1.00)
    parser.add_argument("--ticks-per-rev", type=float, default=2263.7)
    parser.add_argument("--forward-ff-pwm-per-rpm", type=float, default=175.0)
    parser.add_argument("--assumed-wheel-radius-m", type=float, default=0.122)
    parser.add_argument(
        "--strict",
        action="store_true",
        help="exit 2 for acquisition failure or 3 for performance failure",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag = args.bag.expanduser().resolve()
    prefix = (
        args.output_prefix.expanduser().resolve()
        if args.output_prefix
        else bag.parent / f"{bag.name}_firmware_pid"
    )
    json_path = Path(f"{prefix}_metrics.json")
    png_path = Path(f"{prefix}_response.png")
    config = Config(
        expected_level_rad_s=args.expected_level_rad_s,
        expected_ramp_s=args.expected_ramp_s,
        expected_hold_s=args.expected_hold_s,
        measure_s=args.measure_s,
        ticks_per_rev=args.ticks_per_rev,
        forward_ff_pwm_per_rpm=args.forward_ff_pwm_per_rpm,
        assumed_wheel_radius_m=args.assumed_wheel_radius_m,
    )
    try:
        traces = read_bag(bag)
        report = analyze_traces(traces, config, str(bag))
        plot_report(traces, report, png_path)
    except Exception as exc:
        report = {
            "bag": str(bag),
            "profile": "firmware_pid_straight",
            "analysis_error": str(exc),
            "verdict": {
                "acquisition_pass": False,
                "performance_pass": False,
                "overall_pass": False,
            },
        }
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"analysis failed: {exc}")
        print(f"JSON: {json_path}")
        return 2

    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    acquisition_failures = failed_checks(report, "acquisition")
    performance_failures = failed_checks(report, "performance")
    print(
        "acquisition %s; controller performance %s"
        % (
            "PASS" if not acquisition_failures else "FAIL",
            "PASS" if not performance_failures else "FAIL",
        )
    )
    if acquisition_failures:
        print("failed acquisition checks: " + ", ".join(acquisition_failures))
    if performance_failures:
        print("failed performance checks: " + ", ".join(performance_failures))
    vicon = report["metrics"]["opportunistic_vicon"]
    if vicon["usable"]:
        lag = (
            f", dynamic lag ~{1000 * vicon['command_to_ground_lag_s']:.0f} ms"
            if vicon.get("lag_usable") else ", dynamic lag unavailable"
        )
        print(
            f"Vicon offline usable: {vicon['ground_speed_m_s']:.4f} m/s, "
            f"effective radius {vicon['effective_radius_m']:.5f} m{lag}; "
            f"median receipt age {1000 * vicon['receipt_age_median_s']:.1f} ms"
            + (" (stale for live control)" if vicon["stale_for_live_control"] else "")
        )
    elif vicon["available"]:
        print("Vicon offline UNUSABLE: " + "; ".join(vicon["reasons"]))
    else:
        print("Vicon optional measurement absent")
    print(f"JSON: {json_path}")
    print(f"PNG:  {png_path}")
    if args.strict:
        if acquisition_failures:
            return 2
        if performance_failures:
            return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
