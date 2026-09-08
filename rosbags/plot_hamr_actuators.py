#!/usr/bin/env python3
"""Create actuator and stop-safety plots from an offline HAMR rosbag."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "reference": "/reference_trajectory",
    "left_cmd": "/left_wheel/cmd_vel",
    "right_cmd": "/right_wheel/cmd_vel",
    "turret_cmd": "/turret/cmd_vel",
    "gains": "/live_gains",
    "status": "/wheel_control/status",
    "left_enc": "/left_wheel/encoder_ticks",
    "right_enc": "/right_wheel/encoder_ticks",
    "turret_enc": "/turret/encoder_ticks",
}

LEFT = "#1764ab"
RIGHT = "#c43b32"
TURRET = "#6f4e9c"
ORANGE = "#e88b00"
GREEN = "#238b45"

# Keep the offline cap annotation aligned with the hardware bringup default.
# Historical 20 RPM bags can still be plotted with --wheel-cap-rad-s.
DEFAULT_WHEEL_CAP_RPM = 28.0
DEFAULT_WHEEL_CAP_RAD_S = DEFAULT_WHEEL_CAP_RPM * 2.0 * math.pi / 60.0
ACTIVE_TRANSLATION_SPEED_M_S = 0.02
ACTUATOR_SUPPORT_MAX_MATCH_DT_S = 0.05
ACTUATOR_SUPPORT_MAX_EFFECTIVE_GAP_S = 0.10
REVERSAL_PROFILES = frozenset(("forward_reverse",))
WHEEL_STATUS_SCHEMA = "wheel_control_status_v1"
WHEEL_STATUS_FIELD_COUNT = 17
WHEEL_STATUS_SOURCE_MIN = 0
WHEEL_STATUS_SOURCE_MAX = 7


def storage_id(bag: Path) -> str:
    metadata = bag / "metadata.yaml"
    if metadata.exists():
        for line in metadata.read_text().splitlines():
            if "storage_identifier:" in line:
                return line.split(":", 1)[1].strip()
    return "mcap" if any(bag.glob("*.mcap")) else "sqlite3"


def load_bag(bag: Path) -> dict[str, list[tuple[int, object]]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id(bag)),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = set(TOPICS.values())
    missing = sorted(wanted - set(topic_types))
    if missing:
        raise RuntimeError(f"bag is missing required topics: {', '.join(missing)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))
    message_types = {topic: get_message(topic_types[topic]) for topic in wanted}
    data = {name: [] for name in TOPICS}
    names_by_topic = {topic: name for name, topic in TOPICS.items()}
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        data[names_by_topic[topic]].append(
            (timestamp, deserialize_message(raw, message_types[topic]))
        )
    return data


def scalar_series(rows: list[tuple[int, object]]) -> tuple[np.ndarray, np.ndarray]:
    return (
        np.asarray([timestamp * 1e-9 for timestamp, _ in rows], dtype=float),
        np.asarray([float(msg.data) for _, msg in rows], dtype=float),
    )


def command_series(rows: list[tuple[int, object]]) -> tuple[np.ndarray, np.ndarray]:
    return (
        np.asarray([timestamp * 1e-9 for timestamp, _ in rows], dtype=float),
        np.asarray([float(msg.data) for _, msg in rows], dtype=float),
    )


def direction_name(vector: np.ndarray) -> str:
    x, y = vector
    if abs(x) >= abs(y):
        return "E" if x >= 0.0 else "W"
    return "N" if y >= 0.0 else "S"


def reference_events(
    timestamps: np.ndarray, velocity: np.ndarray
) -> list[tuple[float, str, str]]:
    changed = np.flatnonzero(np.any(np.abs(np.diff(velocity, axis=0)) > 1e-9, axis=1)) + 1
    events: list[tuple[float, str, str]] = []
    left_count = 0
    right_count = 0
    for index in changed:
        old = velocity[index - 1]
        new = velocity[index]
        old_moving = np.linalg.norm(old) > 1e-9
        new_moving = np.linalg.norm(new) > 1e-9
        if not old_moving and new_moving:
            label, color = f"start {direction_name(new)}", "#4d4d4d"
        elif old_moving and not new_moving:
            label, color = "motion end", "#4d4d4d"
        elif old_moving and new_moving:
            cross = old[0] * new[1] - old[1] * new[0]
            if cross > 0.0:
                left_count += 1
                label, color = f"L{left_count} → {direction_name(new)}", GREEN
            else:
                right_count += 1
                label, color = f"R{right_count} → {direction_name(new)}", RIGHT
        else:
            continue
        events.append((timestamps[index], label, color))
    return events


def nearest_time_distance(
    query: np.ndarray,
    source: np.ndarray,
) -> np.ndarray:
    """Return the distance from each query timestamp to its nearest source."""
    indices = nearest_time_indices(query, source)
    if source.size == 0:
        return np.full(query.shape, np.inf)
    return np.abs(query - source[indices])


def nearest_time_indices(
    query: np.ndarray,
    source: np.ndarray,
) -> np.ndarray:
    """Return nearest source indices, preferring the earlier sample on ties."""
    if source.size == 0:
        return np.zeros(query.shape, dtype=int)
    positions = np.searchsorted(source, query)
    left = np.clip(positions - 1, 0, source.size - 1)
    right = np.clip(positions, 0, source.size - 1)
    choose_right = np.abs(query - source[right]) < np.abs(query - source[left])
    return np.where(choose_right, right, left)


def align_commands(
    left_t: np.ndarray,
    left_v: np.ndarray,
    right_t: np.ndarray,
    right_v: np.ndarray,
    max_delay: float = 0.02,
) -> tuple[np.ndarray, np.ndarray]:
    """Pair commands exactly as the study analyzer's cap diagnostic does."""
    finite_left = np.isfinite(left_t) & np.isfinite(left_v)
    finite_right = np.isfinite(right_t) & np.isfinite(right_v)
    left_t, left_v = left_t[finite_left], left_v[finite_left]
    right_t, right_v = right_t[finite_right], right_v[finite_right]
    if not left_t.size or not right_t.size:
        return np.empty(0), np.empty((0, 2))
    aligned = nearest_time_distance(left_t, right_t) <= max_delay
    if not np.any(aligned):
        return np.empty(0), np.empty((0, 2))
    right_aligned = np.interp(left_t, right_t, right_v)
    return (
        left_t[aligned],
        np.column_stack((left_v, right_aligned))[aligned],
    )


def reference_active_mask(
    timestamps: np.ndarray,
    velocity: np.ndarray,
    include_internal_dwell: bool,
) -> np.ndarray:
    """Return the frozen reference-clock samples required for exposure."""
    if velocity.ndim != 2 or velocity.shape[1] != 2:
        raise ValueError("reference velocity must have shape (N, 2)")
    speed = np.linalg.norm(velocity, axis=1)
    active = (
        np.isfinite(timestamps)
        & np.isfinite(speed)
        & (speed > ACTIVE_TRANSLATION_SPEED_M_S)
    )
    if include_internal_dwell and np.any(active):
        active_indices = np.flatnonzero(active)
        active[active_indices[0]:active_indices[-1] + 1] = np.isfinite(
            timestamps[active_indices[0]:active_indices[-1] + 1]
        )
    return active


def active_reference_phases(active: np.ndarray) -> list[np.ndarray]:
    """Split required reference samples into contiguous clock phases."""
    indices = np.flatnonzero(active)
    if not indices.size:
        return []
    breaks = np.flatnonzero(np.diff(indices) > 1) + 1
    return [phase for phase in np.split(indices, breaks) if phase.size]


def reference_active_at(
    timestamps: np.ndarray,
    reference_t: np.ndarray,
    reference_v: np.ndarray,
    include_internal_dwell: bool,
) -> np.ndarray:
    """Map command timestamps onto the analyzer's held reference window."""
    if timestamps.size == 0 or reference_t.size == 0:
        return np.zeros(timestamps.shape, dtype=bool)
    indices = np.searchsorted(reference_t, timestamps, side="right") - 1
    inside = (indices >= 0) & (indices < reference_t.size)
    indices = np.clip(indices, 0, reference_t.size - 1)
    reference_active = reference_active_mask(
        reference_t,
        reference_v,
        include_internal_dwell=include_internal_dwell,
    )
    return inside & reference_active[indices]


def actuator_time_support(
    reference_t: np.ndarray,
    reference_v: np.ndarray,
    observed_t: np.ndarray,
    include_internal_dwell: bool,
    stream_label: str,
) -> dict[str, object]:
    """Validate one finite stream against each frozen active phase."""
    epsilon = 1e-9
    active = reference_active_mask(
        reference_t,
        reference_v,
        include_internal_dwell,
    )
    phases = active_reference_phases(active)
    reference_indices = np.concatenate(phases) if phases else np.empty(0, dtype=int)
    finite_samples = np.asarray(observed_t, dtype=float)
    finite_samples = np.unique(finite_samples[np.isfinite(finite_samples)])
    windows = []
    for phase_number, phase in enumerate(phases, start=1):
        query_t = reference_t[phase]
        nearest = nearest_time_indices(query_t, finite_samples)
        match_dt = (
            np.abs(query_t - finite_samples[nearest])
            if finite_samples.size
            else np.full(query_t.shape, np.inf)
        )
        matched = match_dt <= ACTUATOR_SUPPORT_MAX_MATCH_DT_S + epsilon
        matched_count = int(np.count_nonzero(matched))
        expected_count = int(query_t.size)
        coverage = matched_count / expected_count if expected_count else 0.0
        effective_gap = None
        if matched_count:
            matched_observations = np.unique(finite_samples[nearest[matched]])
            matched_observations = np.clip(matched_observations, query_t[0], query_t[-1])
            support_t = np.unique(
                np.r_[query_t[0], matched_observations, query_t[-1]]
            )
            if support_t.size >= 2:
                effective_gap = float(np.max(np.diff(support_t)))
        reasons = []
        if expected_count < 2:
            reasons.append("fewer than two required reference samples")
        if coverage < 1.0 - epsilon:
            reasons.append(
                f"coverage is {coverage:.3f}, below the required 1.000"
            )
        if (
            effective_gap is None
            or effective_gap > ACTUATOR_SUPPORT_MAX_EFFECTIVE_GAP_S + epsilon
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
        windows.append(
            {
                "label": f"active_phase_{phase_number}",
                "required_reference_sample_count": expected_count,
                "matched_reference_sample_count": matched_count,
                "coverage_fraction": coverage,
                "max_reference_match_dt_s": (
                    float(np.max(match_dt[np.isfinite(match_dt)]))
                    if np.any(np.isfinite(match_dt))
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
    if not phases:
        reasons.append(
            f"{stream_label}: reference has no required active-motion samples"
        )
    required_t = reference_t[reference_indices]
    return {
        "stream": stream_label,
        "valid": bool(windows) and all(window["valid"] for window in windows),
        "reason": "; ".join(reasons),
        "failure_reason": "; ".join(reasons),
        "failure_reasons": reasons,
        "reference_indices": reference_indices,
        "nearest_indices": nearest_time_indices(required_t, finite_samples),
        "phase_support": windows,
        "windows": windows,
    }


def command_cap_exposure(
    left_t: np.ndarray,
    left_command: np.ndarray,
    right_t: np.ndarray,
    right_command: np.ndarray,
    reference_t: np.ndarray,
    reference_v: np.ndarray,
    wheel_cap: float,
    profile: str = "",
) -> dict[str, object]:
    """Return reference-clock-weighted cap exposure when support is valid."""
    finite_left = np.isfinite(left_t) & np.isfinite(left_command)
    finite_right = np.isfinite(right_t) & np.isfinite(right_command)
    left_t = left_t[finite_left]
    left_command = left_command[finite_left]
    right_t = right_t[finite_right]
    right_command = right_command[finite_right]
    left_order = np.argsort(left_t, kind="stable")
    right_order = np.argsort(right_t, kind="stable")
    left_t, left_command = left_t[left_order], left_command[left_order]
    right_t, right_command = right_t[right_order], right_command[right_order]
    if left_t.size:
        keep = np.r_[True, np.diff(left_t) > 0.0]
        left_t, left_command = left_t[keep], left_command[keep]
    if right_t.size:
        keep = np.r_[True, np.diff(right_t) > 0.0]
        right_t, right_command = right_t[keep], right_command[keep]
    include_dwell = profile in REVERSAL_PROFILES
    left_support = actuator_time_support(
        reference_t,
        reference_v,
        left_t,
        include_internal_dwell=include_dwell,
        stream_label="left wheel command",
    )
    right_support = actuator_time_support(
        reference_t,
        reference_v,
        right_t,
        include_internal_dwell=include_dwell,
        stream_label="right wheel command",
    )
    support_valid = bool(left_support["valid"] and right_support["valid"])
    reasons = [
        str(support["reason"])
        for support in (left_support, right_support)
        if not support["valid"]
    ]
    reference_indices = np.asarray(left_support["reference_indices"], dtype=int)
    sample_count = int(reference_indices.size)
    if not support_valid:
        return {
            "available": False,
            "reason": "; ".join(reasons),
            "include_internal_dwell": include_dwell,
            "sample_count": sample_count,
            "cap_count": None,
            "cap_fraction": None,
            "timestamps": np.empty(0),
            "wheel_commands": np.empty((0, 2)),
            "capped": np.empty(0, dtype=bool),
            "left_support": left_support,
            "right_support": right_support,
        }
    left_nearest = np.asarray(left_support["nearest_indices"], dtype=int)
    right_nearest = np.asarray(right_support["nearest_indices"], dtype=int)
    wheel_commands = np.column_stack(
        (left_command[left_nearest], right_command[right_nearest])
    )
    tolerance = max(1e-8, wheel_cap * 1e-8)
    capped = np.any(
        np.abs(wheel_commands) >= wheel_cap - tolerance,
        axis=1,
    )
    cap_count = int(np.count_nonzero(capped))
    return {
        "available": True,
        "reason": "",
        "include_internal_dwell": include_dwell,
        "sample_count": sample_count,
        "cap_count": cap_count,
        "cap_fraction": cap_count / sample_count,
        "timestamps": reference_t[reference_indices],
        "wheel_commands": wheel_commands,
        "capped": capped,
        "left_support": left_support,
        "right_support": right_support,
    }


def wheel_status_v1_layout(message: object) -> tuple[bool, str]:
    """Validate the exact firmware status array layout contract."""
    layout = getattr(message, "layout", None)
    dimensions = getattr(layout, "dim", ()) if layout is not None else ()
    data_offset = getattr(layout, "data_offset", None)
    if len(dimensions) != 1:
        return False, "wheel status layout must have exactly one dimension"
    dimension = dimensions[0]
    if (
        getattr(dimension, "label", "") != WHEEL_STATUS_SCHEMA
        or getattr(dimension, "size", None) != WHEEL_STATUS_FIELD_COUNT
        or getattr(dimension, "stride", None) != WHEEL_STATUS_FIELD_COUNT
        or data_offset != 0
    ):
        return (
            False,
            "wheel status layout is not wheel_control_status_v1 "
            "(size=17, stride=17, data_offset=0)",
        )
    if len(getattr(message, "data", ())) != WHEEL_STATUS_FIELD_COUNT:
        return False, "wheel_control_status_v1 data length is not 17"
    return True, ""


def wheel_status_v1_array(
    rows: list[tuple[int, object]],
) -> tuple[np.ndarray, np.ndarray | None, str]:
    """Decode actuator fields; report an invalid source enum separately."""
    if not rows:
        raise RuntimeError("wheel status topic is empty")
    for index, (_timestamp, message) in enumerate(rows):
        valid, reason = wheel_status_v1_layout(message)
        if not valid:
            raise RuntimeError(
                f"wheel status sample {index} is unavailable: {reason}"
            )
    try:
        status = np.asarray(
            [np.asarray(message.data, dtype=float) for _, message in rows],
            dtype=float,
        )
    except (TypeError, ValueError) as error:
        raise RuntimeError(
            f"invalid wheel_control_status_v1 data: {error}"
        ) from error
    if status.shape != (len(rows), WHEEL_STATUS_FIELD_COUNT):
        raise RuntimeError(
            "wheel_control_status_v1 samples do not form an N x 17 array"
        )
    if not np.all(np.isfinite(status[:, :16])):
        raise RuntimeError(
            "wheel_control_status_v1 actuator fields 0..15 must be finite"
        )
    sources, source_reason = wheel_status_source_values(status)
    return status, sources, source_reason


def wheel_status_source_values(
    status: np.ndarray,
) -> tuple[np.ndarray | None, str]:
    """Validate source enums for the selected reference-clock samples."""
    raw_sources = status[:, 16]
    rounded_sources = np.rint(raw_sources)
    valid_sources = (
        np.isfinite(raw_sources)
        & (np.abs(raw_sources - rounded_sources) <= 1e-9)
        & (rounded_sources >= WHEEL_STATUS_SOURCE_MIN)
        & (rounded_sources <= WHEEL_STATUS_SOURCE_MAX)
    )
    if not np.all(valid_sources):
        return (
            None,
            "wheel_control_status_v1 source values must be finite integer "
            "codes in 0..7",
        )
    return rounded_sources.astype(int), ""


def wheel_status_exposure(
    status_t: np.ndarray,
    status: np.ndarray,
    reference_t: np.ndarray,
    reference_v: np.ndarray,
    profile: str = "",
) -> dict[str, object]:
    """Return reference-clock status exposure only with complete support."""
    finite = np.isfinite(status_t) & np.all(np.isfinite(status[:, :16]), axis=1)
    finite_t = status_t[finite]
    finite_status = status[finite]
    order = np.argsort(finite_t, kind="stable")
    finite_t, finite_status = finite_t[order], finite_status[order]
    if finite_t.size:
        keep = np.r_[True, np.diff(finite_t) > 0.0]
        finite_t, finite_status = finite_t[keep], finite_status[keep]
    include_dwell = profile in REVERSAL_PROFILES
    support = actuator_time_support(
        reference_t,
        reference_v,
        finite_t,
        include_internal_dwell=include_dwell,
        stream_label="wheel status",
    )
    reference_indices = np.asarray(support["reference_indices"], dtype=int)
    sample_count = int(reference_indices.size)
    if not support["valid"]:
        reason = str(support["reason"])
        return {
            "available": False,
            "reason": reason,
            "include_internal_dwell": include_dwell,
            "sample_count": sample_count,
            "saturation_count": None,
            "saturation_fraction": None,
            "source_values": None,
            "source_reason": (
                "wheel status exposure unavailable: "
                f"{compact_support_reason(reason)}"
            ),
            "support": support,
        }
    nearest = np.asarray(support["nearest_indices"], dtype=int)
    selected = finite_status[nearest]
    saturation = (selected[:, 7] != 0.0) | (selected[:, 15] != 0.0)
    saturation_count = int(np.count_nonzero(saturation))
    source_values, source_reason = wheel_status_source_values(selected)
    return {
        "available": True,
        "reason": "",
        "include_internal_dwell": include_dwell,
        "sample_count": sample_count,
        "saturation_count": saturation_count,
        "saturation_fraction": saturation_count / sample_count,
        "source_values": source_values,
        "source_reason": source_reason,
        "support": support,
    }


def compact_support_reason(reason: str) -> str:
    """Condense exact support failures for readable plot annotations."""
    details = []
    if "coverage is" in reason:
        details.append("incomplete <=50 ms reference coverage")
    if "effective topic support gap" in reason:
        details.append("topic support gap >100 ms")
    if "fewer than two required reference samples" in reason:
        details.append("fewer than two required reference samples")
    if "no required active-motion samples" in reason:
        details.append("no required active-motion samples")
    return "; ".join(details) if details else reason


def cap_exposure_text(
    exposure: dict[str, object],
    wheel_cap: float,
) -> str:
    """Format a cap label without turning missing support into zero percent."""
    wheel_cap_rpm = wheel_cap * 60.0 / (2.0 * math.pi)
    prefix = f"pair cap {wheel_cap:.6f} rad/s ({wheel_cap_rpm:.1f} RPM): "
    dwell = (
        " incl. planned dwell"
        if exposure["include_internal_dwell"]
        else ""
    )
    if not exposure["available"]:
        reason = compact_support_reason(str(exposure["reason"]))
        return prefix + f"exposure unavailable ({reason})" + dwell
    percent = 100.0 * float(exposure["cap_fraction"])
    return (
        prefix
        + f"{exposure['cap_count']}/{exposure['sample_count']} "
        + f"active reference samples ({percent:.1f}%)"
        + dwell
    )


def saturation_exposure_text(exposure: dict[str, object]) -> str:
    """Format status exposure without making a false zero-percent claim."""
    if not exposure["available"]:
        reason = compact_support_reason(str(exposure["reason"]))
        return f"firmware saturation exposure unavailable ({reason})"
    percent = 100.0 * float(exposure["saturation_fraction"])
    return (
        f"firmware saturation {exposure['saturation_count']}/"
        f"{exposure['sample_count']} active reference samples "
        f"({percent:.1f}%)"
    )


def first_exact_zero_suffix(timestamps: np.ndarray, values: np.ndarray) -> float | None:
    if not timestamps.size:
        return None
    nonzero = np.any(values != 0.0, axis=1) if values.ndim > 1 else values != 0.0
    indices = np.flatnonzero(nonzero)
    if not indices.size:
        return float(timestamps[0])
    first = int(indices[-1] + 1)
    return float(timestamps[first]) if first < timestamps.size else None


def gross_motion(values: np.ndarray) -> float:
    return float(np.abs(np.diff(values)).sum())


def add_event_lines(
    axes: list[plt.Axes], events: list[tuple[float, str, str]], t0: float
) -> None:
    for timestamp, _, color in events:
        for axis in axes:
            axis.axvline(timestamp - t0, color=color, lw=0.9, alpha=0.65, ls=":")
    top = axes[0]
    for timestamp, label, color in events:
        top.text(
            timestamp - t0,
            0.98,
            label,
            color=color,
            fontsize=8,
            rotation=90,
            ha="right",
            va="top",
            transform=top.get_xaxis_transform(),
        )


def add_stop_lines(
    axes: list[plt.Axes],
    last_reference: float,
    command_zero: float | None,
    firmware_zero: float | None,
    t0: float,
) -> None:
    marks = [(last_reference, "last reference", "#4d4d4d")]
    if command_zero is not None:
        marks.append((command_zero, "both commands zero", GREEN))
    if firmware_zero is not None:
        marks.append((firmware_zero, "firmware exact zero", TURRET))
    for timestamp, _, color in marks:
        for axis in axes:
            axis.axvline(timestamp - t0, color=color, lw=1.2, ls="--", alpha=0.9)


def plot(
    bag: Path,
    output_prefix: Path,
    wheel_cap: float,
    profile: str = "",
) -> tuple[Path, Path, str]:
    if not math.isfinite(wheel_cap) or wheel_cap <= 0.0:
        raise ValueError("wheel cap must be a finite positive value")
    data = load_bag(bag)

    ref_t = np.asarray([t * 1e-9 for t, _ in data["reference"]], dtype=float)
    ref_v = np.asarray(
        [[msg.x_dot, msg.y_dot] for _, msg in data["reference"]],
        dtype=float,
    )
    left_t, left_cmd = command_series(data["left_cmd"])
    right_t, right_cmd = command_series(data["right_cmd"])
    turret_t, turret_cmd = command_series(data["turret_cmd"])
    gains_t = np.asarray([t * 1e-9 for t, _ in data["gains"]], dtype=float)
    derivative = np.asarray(
        [[msg.d_x, msg.d_y] for _, msg in data["gains"]], dtype=float
    )

    status_t = np.asarray([t * 1e-9 for t, _ in data["status"]], dtype=float)
    status, _raw_status_sources, _raw_source_reason = wheel_status_v1_array(
        data["status"]
    )
    target_left, measured_left = status[:, 0], status[:, 1]
    target_right, measured_right = status[:, 8], status[:, 9]
    pwm_left, pwm_right = status[:, 4], status[:, 12]
    saturated_left, saturated_right = status[:, 7] != 0.0, status[:, 15] != 0.0

    left_enc_t, left_enc = scalar_series(data["left_enc"])
    right_enc_t, right_enc = scalar_series(data["right_enc"])
    turret_enc_t, turret_enc = scalar_series(data["turret_enc"])

    if not ref_t.size:
        raise RuntimeError("reference topic is empty")
    t0 = float(ref_t[0])
    last_reference = float(ref_t[-1])
    events = reference_events(ref_t, ref_v)

    cycle_t, wheel = align_commands(
        left_t, left_cmd, right_t, right_cmd
    )
    cap_exposure = command_cap_exposure(
        left_t,
        left_cmd,
        right_t,
        right_cmd,
        ref_t,
        ref_v,
        wheel_cap,
        profile=profile,
    )
    status_exposure = wheel_status_exposure(
        status_t,
        status,
        ref_t,
        ref_v,
        profile=profile,
    )

    wheel_step_t = cycle_t[1:]
    wheel_step = np.linalg.norm(np.diff(wheel, axis=0), axis=1)
    both_zero_t = max(
        value
        for value in (
            first_exact_zero_suffix(left_t, left_cmd),
            first_exact_zero_suffix(right_t, right_cmd),
        )
        if value is not None
    )
    firmware_zero_t = first_exact_zero_suffix(status_t, status[:, :16])
    firmware_zero_tail = (
        float(status_t[-1] - firmware_zero_t) if firmware_zero_t is not None else math.nan
    )

    saturation = saturated_left | saturated_right
    raw_saturation_count = int(saturation.sum())
    status_sources = status_exposure["source_values"]
    if status_sources is None:
        source_summary = f"unavailable ({status_exposure['source_reason']})"
    else:
        source_values, source_counts = np.unique(
            status_sources, return_counts=True
        )
        source_summary = ", ".join(
            f"{int(source)}:{int(count)}"
            for source, count in zip(source_values, source_counts)
        )
    left_gross = gross_motion(left_enc)
    right_gross = gross_motion(right_enc)
    encoder_bias = (
        100.0 * (right_gross - left_gross) / ((right_gross + left_gross) / 2.0)
        if left_gross + right_gross
        else math.nan
    )

    x_end = max(left_t[-1], right_t[-1], status_t[-1], left_enc_t[-1]) - t0
    fig, axes_array = plt.subplots(6, 1, figsize=(16, 18), sharex=True)
    axes = list(axes_array)

    axes[0].plot(ref_t - t0, ref_v[:, 0], label="$v_x$", color=LEFT, lw=1.4)
    axes[0].plot(ref_t - t0, ref_v[:, 1], label="$v_y$", color=RIGHT, lw=1.4)
    axes[0].set_ylabel("reference\n(m/s)")
    axes[0].legend(loc="lower left", ncol=2)

    axes[1].plot(left_t - t0, left_cmd, label="left", color=LEFT, lw=1.0)
    axes[1].plot(right_t - t0, right_cmd, label="right", color=RIGHT, lw=1.0)
    wheel_cap_rpm = wheel_cap * 60.0 / (2.0 * math.pi)
    axes[1].axhline(
        wheel_cap,
        color=ORANGE,
        lw=0.8,
        ls="--",
        label=f"controller cap ({wheel_cap_rpm:.1f} RPM)",
    )
    axes[1].axhline(-wheel_cap, color=ORANGE, lw=0.8, ls="--")
    if cap_exposure["available"]:
        cap_t = np.asarray(cap_exposure["timestamps"], dtype=float)
        cap_wheel = np.asarray(cap_exposure["wheel_commands"], dtype=float)
        capped = np.asarray(cap_exposure["capped"], dtype=bool)
        cap_percent = 100.0 * float(cap_exposure["cap_fraction"])
        axes[1].scatter(
            cap_t[capped] - t0,
            np.max(np.abs(cap_wheel[capped]), axis=1),
            s=4,
            color=ORANGE,
            label=f"pair capped ({cap_percent:.1f}% supported active)",
            zorder=3,
        )
    else:
        cap_reason = compact_support_reason(str(cap_exposure["reason"]))
        axes[1].plot(
            [],
            [],
            color=ORANGE,
            label=f"cap exposure unavailable ({cap_reason})",
        )
    axes[1].set_ylabel("wheel command\n(rad/s)")
    axes[1].legend(loc="upper right", ncol=3)

    axes[2].plot(status_t - t0, target_left, color=LEFT, lw=0.9, ls="--", label="L target")
    axes[2].plot(status_t - t0, measured_left, color=LEFT, lw=0.9, alpha=0.65, label="L measured")
    axes[2].plot(status_t - t0, target_right, color=RIGHT, lw=0.9, ls="--", label="R target")
    axes[2].plot(status_t - t0, measured_right, color=RIGHT, lw=0.9, alpha=0.65, label="R measured")
    axes[2].set_ylabel("firmware speed\n(rpm)")
    axes[2].legend(loc="upper right", ncol=4)

    axes[3].plot(status_t - t0, pwm_left, color=LEFT, lw=0.8, label="L PWM")
    axes[3].plot(status_t - t0, pwm_right, color=RIGHT, lw=0.8, label="R PWM")
    axes[3].axhline(4095, color="#777777", lw=0.7, ls=":")
    axes[3].axhline(-4095, color="#777777", lw=0.7, ls=":")
    if raw_saturation_count:
        sat_y = np.where(np.abs(pwm_left[saturation]) >= np.abs(pwm_right[saturation]), pwm_left[saturation], pwm_right[saturation])
        axes[3].scatter(status_t[saturation] - t0, sat_y, marker="v", s=24, color="black", label="raw firmware saturation markers")
    if not status_exposure["available"]:
        status_reason = compact_support_reason(str(status_exposure["reason"]))
        axes[3].plot(
            [],
            [],
            color="black",
            label=(
                "saturation exposure unavailable "
                f"({status_reason})"
            ),
        )
    axes[3].set_ylabel("motor output\n(PWM)")
    axes[3].legend(loc="upper right", ncol=3)

    axes[4].plot(gains_t - t0, derivative[:, 0], color=LEFT, lw=0.8, label="$D_x$")
    axes[4].plot(gains_t - t0, derivative[:, 1], color=RIGHT, lw=0.8, label="$D_y$")
    axes[4].set_ylabel("D term")
    step_axis = axes[4].twinx()
    step_axis.plot(wheel_step_t - t0, wheel_step, color="#555555", alpha=0.38, lw=0.7, label="$|Δω|$")
    step_axis.set_ylabel("command step (rad/s)", color="#555555")
    handles1, labels1 = axes[4].get_legend_handles_labels()
    handles2, labels2 = step_axis.get_legend_handles_labels()
    axes[4].legend(handles1 + handles2, labels1 + labels2, loc="upper right", ncol=3)

    axes[5].plot(left_enc_t - t0, left_enc - left_enc[0], color=LEFT, lw=0.9, label=f"L (gross {left_gross:.0f})")
    axes[5].plot(right_enc_t - t0, right_enc - right_enc[0], color=RIGHT, lw=0.9, label=f"R (gross {right_gross:.0f})")
    axes[5].set_ylabel("wheel encoder\nΔ ticks")
    axes[5].set_xlabel("seconds from first reference")
    turret_axis = axes[5].twinx()
    turret_axis.plot(turret_enc_t - t0, turret_enc - turret_enc[0], color=TURRET, lw=0.8, alpha=0.7, label="turret passive")
    turret_axis.set_ylabel("turret Δ ticks", color=TURRET)
    handles1, labels1 = axes[5].get_legend_handles_labels()
    handles2, labels2 = turret_axis.get_legend_handles_labels()
    axes[5].legend(handles1 + handles2, labels1 + labels2, loc="upper left", ncol=3)

    add_event_lines(axes, events, t0)
    add_stop_lines(axes, last_reference, both_zero_t, firmware_zero_t, t0)
    for axis in axes:
        axis.grid(True, alpha=0.22)
        axis.set_xlim(-0.25, x_end + 0.1)
        axis.axvspan(last_reference - t0, x_end + 0.1, color="#d9d9d9", alpha=0.18, zorder=-10)

    turret_max = float(np.nanmax(np.abs(turret_cmd)))
    summary = (
        f"{cap_exposure_text(cap_exposure, wheel_cap)}   |   "
        f"{saturation_exposure_text(status_exposure)}   |   "
        f"firmware sources {source_summary}   |   "
        f"encoder gross L {left_gross:.0f}, R {right_gross:.0f} "
        f"({encoder_bias:+.2f}% R−L)\n"
        f"turret command max |ω| = {turret_max:g} rad/s   |   "
        f"exact-zero firmware tail = {firmware_zero_tail:.3f} s"
    )
    fig.suptitle(f"HAMR actuator overview — {bag.name}\n{summary}", fontsize=12, y=0.998)
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    overview = Path(f"{output_prefix}_overview.png")
    overview.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(overview, dpi=170)
    plt.close(fig)

    zoom_start = last_reference - 0.25
    zoom_end = max(
        last_reference + 1.25,
        (firmware_zero_t + 0.75) if firmware_zero_t is not None else last_reference + 1.25,
    )
    left_zoom = (left_t >= zoom_start) & (left_t <= zoom_end)
    right_zoom = (right_t >= zoom_start) & (right_t <= zoom_end)
    turret_zoom = (turret_t >= zoom_start) & (turret_t <= zoom_end)
    status_zoom = (status_t >= zoom_start) & (status_t <= zoom_end)
    left_enc_zoom = (left_enc_t >= zoom_start) & (left_enc_t <= zoom_end)
    right_enc_zoom = (right_enc_t >= zoom_start) & (right_enc_t <= zoom_end)
    turret_enc_zoom = (turret_enc_t >= zoom_start) & (turret_enc_t <= zoom_end)
    fig, zoom_array = plt.subplots(4, 1, figsize=(15, 10), sharex=True)
    zoom_axes = list(zoom_array)
    zoom_axes[0].plot(left_t[left_zoom] - t0, left_cmd[left_zoom], color=LEFT, lw=1.2, label="left command")
    zoom_axes[0].plot(right_t[right_zoom] - t0, right_cmd[right_zoom], color=RIGHT, lw=1.2, label="right command")
    zoom_axes[0].plot(turret_t[turret_zoom] - t0, turret_cmd[turret_zoom], color=TURRET, lw=0.9, label="turret command")
    zoom_axes[0].set_ylabel("command\n(rad/s)")
    zoom_axes[0].legend(loc="upper right", ncol=3)

    zoom_axes[1].plot(status_t[status_zoom] - t0, target_left[status_zoom], color=LEFT, ls="--", lw=1.0, label="L target")
    zoom_axes[1].plot(status_t[status_zoom] - t0, measured_left[status_zoom], color=LEFT, lw=1.0, alpha=0.65, label="L measured")
    zoom_axes[1].plot(status_t[status_zoom] - t0, target_right[status_zoom], color=RIGHT, ls="--", lw=1.0, label="R target")
    zoom_axes[1].plot(status_t[status_zoom] - t0, measured_right[status_zoom], color=RIGHT, lw=1.0, alpha=0.65, label="R measured")
    zoom_axes[1].set_ylabel("speed (rpm)")
    zoom_axes[1].legend(loc="upper right", ncol=4)

    zoom_axes[2].plot(status_t[status_zoom] - t0, pwm_left[status_zoom], color=LEFT, lw=1.0, label="L PWM")
    zoom_axes[2].plot(status_t[status_zoom] - t0, pwm_right[status_zoom], color=RIGHT, lw=1.0, label="R PWM")
    zoom_axes[2].set_ylabel("PWM")
    zoom_axes[2].legend(loc="upper right", ncol=2)

    left_base = left_enc[np.searchsorted(left_enc_t, zoom_start, side="left")]
    right_base = right_enc[np.searchsorted(right_enc_t, zoom_start, side="left")]
    turret_base = turret_enc[np.searchsorted(turret_enc_t, zoom_start, side="left")]
    zoom_axes[3].plot(left_enc_t[left_enc_zoom] - t0, left_enc[left_enc_zoom] - left_base, color=LEFT, lw=1.0, label="L encoder")
    zoom_axes[3].plot(right_enc_t[right_enc_zoom] - t0, right_enc[right_enc_zoom] - right_base, color=RIGHT, lw=1.0, label="R encoder")
    zoom_axes[3].plot(turret_enc_t[turret_enc_zoom] - t0, turret_enc[turret_enc_zoom] - turret_base, color=TURRET, lw=0.9, label="turret encoder")
    zoom_axes[3].set_ylabel("Δ ticks")
    zoom_axes[3].set_xlabel("seconds from first reference")
    zoom_axes[3].legend(loc="upper right", ncol=3)

    add_stop_lines(zoom_axes, last_reference, both_zero_t, firmware_zero_t, t0)
    for axis in zoom_axes:
        axis.set_xlim(zoom_start - t0, zoom_end - t0)
        axis.grid(True, alpha=0.25)
    labels = [(last_reference, "last reference", "#4d4d4d")]
    if both_zero_t is not None:
        labels.append((both_zero_t, "both commands zero", GREEN))
    if firmware_zero_t is not None:
        labels.append((firmware_zero_t, "firmware exact zero", TURRET))
    for timestamp, label, color in labels:
        zoom_axes[0].text(
            timestamp - t0,
            0.98,
            label,
            color=color,
            fontsize=8,
            rotation=90,
            ha="right",
            va="top",
            transform=zoom_axes[0].get_xaxis_transform(),
        )
    fig.suptitle(
        f"HAMR stop-window proof — {bag.name}\n"
        f"reference→both commands zero {(both_zero_t - last_reference):.3f} s; "
        f"reference→firmware exact zero {(firmware_zero_t - last_reference):.3f} s; "
        f"zero tail {firmware_zero_tail:.3f} s",
        fontsize=12,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    stop_zoom = Path(f"{output_prefix}_stop_zoom.png")
    fig.savefig(stop_zoom, dpi=170)
    plt.close(fig)

    return overview, stop_zoom, summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory")
    parser.add_argument(
        "--output-prefix",
        type=Path,
        help="output prefix (default: next to the bag)",
    )
    parser.add_argument(
        "--wheel-cap-rad-s",
        type=float,
        default=DEFAULT_WHEEL_CAP_RAD_S,
        help=(
            "controller wheel cap (default: 2.932153 rad/s = 28 RPM); "
            "override this when plotting historical bags with another cap"
        ),
    )
    parser.add_argument(
        "--profile",
        default="",
        help=(
            "study profile name; forward_reverse includes its planned "
            "midpoint dwell in actuator exposure support"
        ),
    )
    args = parser.parse_args()
    bag = args.bag.resolve()
    prefix = args.output_prefix or bag.parent / f"{bag.name}_actuator"
    overview, stop_zoom, summary = plot(
        bag,
        prefix.resolve(),
        args.wheel_cap_rad_s,
        profile=args.profile,
    )
    print(summary)
    print(f"overview: {overview}")
    print(f"stop zoom: {stop_zoom}")


if __name__ == "__main__":
    main()
