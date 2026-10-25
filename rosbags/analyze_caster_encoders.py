#!/usr/bin/env python3
"""Export caster telemetry, reception quality, and optional independent angle error.

Reads one wheel's /caster_encoder/<wheel>/packets (std_msgs/String), or the
legacy /caster_encoder/packets, from sqlite3/MCAP ROS 2 bags. Also accepts the
encoder logger's packets.jsonl without ROS. No clock, sign, or phase
alignment is estimated from the measurements being evaluated.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import csv
from decimal import Decimal, InvalidOperation
import json
import math
from pathlib import Path
import statistics
import sys

try:
    import caster_encoder_protocol as protocol
except ModuleNotFoundError as error:
    if error.name != "caster_encoder_protocol":
        raise
    # Installed scripts sit beside the helper; this also supports source checkout use.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hamr_bringup" / "scripts"))
    import caster_encoder_protocol as protocol

from caster_recording_format import LOCAL_ORIGIN, validate_local_envelope


from caster_encoder_selection import (
    WHEELS, WheelIdentityGuard, metadata_topic, resolve_selection,
)


TOPIC = "/caster_encoder/packets"
METADATA_TOPIC = "/caster_encoder/metadata"
ROLES = {1: "H1", 2: "H2", 3: "CENTER"}
CSV_FIELDS = (
    "receive_epoch_ns", "receive_monotonic_ns", "source_ip", "source_port",
    "bag_timestamp_ns", "wheel_id", "device_id", "node_id", "boot_id", *protocol.SAMPLE_FIELDS,
    *protocol.FLAG_NAMES, *protocol.DIAGNOSTIC_FIELDS,
    "sequence_status", "conflicting_duplicate", "segment_id",
    "mapped_epoch_ns", "mapped_clock_sync_valid", "comparison_time_basis",
    "angle_eligible", "velocity_eligible", "comparison_exclusion",
    "reference_time_epoch_ns", "reference_angle_rad", "reference_match_dt_ns",
    "angle_error_rad",
    "origin", "timing_estimated", "diagnostics_known",
)


def validate_envelope(value):
    """Keep exact integers and unknown packet fields; reject malformed recordings."""
    if not isinstance(value, dict):
        raise ValueError("packet envelope must be an object")
    packet = value.get("packet")
    protocol.decode_packet(json.dumps(packet, allow_nan=False).encode("utf-8"))
    if value.get("origin") == LOCAL_ORIGIN:
        return validate_local_envelope(value)
    if value.get("metadata_source_topic") is not None:
        if not isinstance(value["metadata_source_topic"], str) or not value["metadata_source_topic"].endswith("/metadata"):
            raise ValueError("retained metadata requires a metadata source topic")
        if packet["type"] != "metadata":
            raise ValueError("retained metadata must not contain samples")
        protocol.integer(value.get("snapshot_bag_timestamp_ns"), "snapshot_bag_timestamp_ns", 0, (1 << 127) - 1)
        if any(value.get(name) is not None for name in CSV_FIELDS[:4]):
            raise ValueError("retained metadata must not invent original receive timestamps/source")
        return value
    for name in ("receive_epoch_ns", "receive_monotonic_ns"):
        protocol.integer(value.get(name), name, 0, (1 << 127) - 1)
    protocol.integer(value.get("source_port"), "source_port", 0, 65535)
    if not isinstance(value.get("source_ip"), str) or not value["source_ip"]:
        raise ValueError("source_ip must be a nonempty string")
    return value


def read_jsonl(path):
    with Path(path).open(encoding="utf-8") as handle:
        for number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                yield validate_envelope(protocol.parse_json(line)), None
            except (ValueError, TypeError) as error:
                raise ValueError(f"{path}:{number}: {error}") from error


def storage_id(path):
    path = Path(path)
    if path.suffix.lower() in (".db3", ".sqlite3"):
        return "sqlite3"
    if path.suffix.lower() == ".mcap":
        return "mcap"
    metadata = path / "metadata.yaml"
    if metadata.is_file():
        for line in metadata.read_text(encoding="utf-8").splitlines():
            if line.strip().startswith("storage_identifier:"):
                return line.split(":", 1)[1].strip().strip("\"'")
    return "mcap" if any(path.glob("*.mcap")) else "sqlite3"


def read_bag(path, topic=None, wheel_id=None):
    """Import ROS only when needed; the JSONL analysis path is portable."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import String
    except ImportError as error:
        raise RuntimeError(
            "ROS bag input needs rosbag2_py, rclpy and std_msgs. Source your ROS 2 "
            "installation and workspace; install ros-$ROS_DISTRO-rosbag2-py and "
            "ros-$ROS_DISTRO-rosbag2-storage-mcap for MCAP. Alternatively use "
            "--packets-jsonl recordings/run-.../packets.jsonl without ROS."
        ) from error
    path = Path(path)
    if not path.exists():
        raise ValueError(f"bag does not exist: {path}")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id(path)),
                rosbag2_py.ConverterOptions("", ""))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wheel_id, topic = resolve_selection(wheel_id, topic, types)
    retained_topic = metadata_topic(topic)
    guard = WheelIdentityGuard(wheel_id)
    if topic not in types:
        raise ValueError(f"bag has no {topic}; enable caster encoder recording for a new run")
    if types[topic] != "std_msgs/msg/String":
        raise ValueError(f"{topic} has unexpected message type {types[topic]}")
    topics = [topic]
    if retained_topic in types:
        if types[retained_topic] != "std_msgs/msg/String":
            raise ValueError(f"{retained_topic} must be std_msgs/msg/String")
        topics.append(retained_topic)
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while reader.has_next():
        current_topic, data, timestamp = reader.read_next()
        try:
            decoded = protocol.parse_json(deserialize_message(data, String).data)
            if current_topic == retained_topic:
                if not isinstance(decoded, dict) or not isinstance(decoded.get("nodes"), dict):
                    raise ValueError("retained metadata requires nodes mapping")
                for packet in decoded["nodes"].values():
                    envelope = {
                        "receive_epoch_ns": None, "receive_monotonic_ns": None,
                        "source_ip": None, "source_port": None, "packet": packet,
                        "metadata_source_topic": retained_topic,
                        "snapshot_bag_timestamp_ns": int(timestamp),
                    }
                    validate_envelope(envelope)
                    guard.accept(packet)
                    yield envelope, int(timestamp)
                continue
            envelope = validate_envelope(decoded)
            guard.accept(envelope["packet"])
        except (ValueError, TypeError) as error:
            raise ValueError(f"invalid encoder envelope at bag timestamp {timestamp}: {error}") from error
        yield envelope, int(timestamp)


def finite(value, name):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be finite") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def seconds_to_ns(value):
    try:
        result = Decimal(str(value))
    except InvalidOperation as error:
        raise ValueError("time offset must be finite") from error
    if not result.is_finite():
        raise ValueError("time offset must be finite")
    return int((result * 1_000_000_000).to_integral_value())


def read_reference(path, offset_s=0.0, signs=None, zeros=None):
    """CSV timestamps are exact integer epoch ns, never floating point seconds."""
    signs, zeros = signs or {}, zeros or {}
    result = defaultdict(list)
    seen = set()
    offset_ns = seconds_to_ns(offset_s)
    with Path(path).open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"node_id", "boot_id", "time_epoch_ns", "angle_rad"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"reference CSV requires columns: {', '.join(sorted(required))}")
        for line, record in enumerate(reader, 2):
            try:
                node = int(record["node_id"])
                boot = record["boot_id"].lower()
                protocol.identity({"node_id": node, "boot_id": boot})
                if not record["time_epoch_ns"].isdigit():
                    raise ValueError("time_epoch_ns must be a nonnegative decimal integer")
                timestamp = int(record["time_epoch_ns"]) + offset_ns
                if timestamp < 0:
                    raise ValueError("adjusted reference time is negative")
                # Optional externally supplied visibility/tracking gate.
                if record.get("valid", "1") not in ("0", "1"):
                    raise ValueError("optional reference valid column must be 0 or 1")
                if record.get("valid", "1") == "0":
                    continue
                angle = finite(record["angle_rad"], "angle_rad")
                if record.get("wheel_id") or record.get("device_id"):
                    identity_packet = {"v": 2, "wheel_id": record.get("wheel_id"),
                                       "device_id": record.get("device_id"), "node_id": node, "boot_id": boot}
                    protocol.wheel_device_identity(identity_packet)
                    key = protocol.packet_identity(identity_packet)
                else:
                    key = (node, boot)
                identity = (*key, timestamp)
                if identity in seen:
                    raise ValueError("duplicate reference timestamp for one node/boot")
                seen.add(identity)
                angle = signs.get(node, 1) * (angle - zeros.get(node, 0.0))
                result[key].append((timestamp, angle))
            except (ValueError, KeyError, TypeError) as error:
                raise ValueError(f"{path}:{line}: {error}") from error
    for values in result.values():
        values.sort()
    return dict(result)


def angle_exclusion(row):
    if row["sequence_status"] == "duplicate":
        return "duplicate"
    if row["conflicting_duplicate"]:
        return "conflicting_duplicate"
    if not row["ready"]:
        return "not_ready"
    if not row["sample_valid"]:
        return "sample_invalid"
    field = "calibrated_alpha_rad" if row["node_id"] == 3 else "angle_rad"
    if row[field] is None:
        return "angle_unavailable"
    if row["parity_error"] or row["spi_io_error"]:
        return "sensor_error"
    if row["node_id"] != 3 and (not row["unwrap_continuous"] or row["continuity_lost"]):
        return "continuity_unverified"
    return ""


def error_statistics(values):
    if not values:
        return {"count": 0, "rmse_rad": None, "mae_rad": None,
                "bias_rad": None, "p95_abs_error_rad": None, "max_abs_error_rad": None}
    absolute = sorted(abs(value) for value in values)
    position = .95 * (len(absolute) - 1)
    low, high = math.floor(position), math.ceil(position)
    return {
        "count": len(values),
        "rmse_rad": math.sqrt(statistics.fmean(value * value for value in values)),
        "mae_rad": statistics.fmean(absolute),
        "bias_rad": statistics.fmean(values),
        "p95_abs_error_rad": absolute[low] + (position - low) * (absolute[high] - absolute[low]),
        "max_abs_error_rad": absolute[-1],
    }


def index_consistency(rows, metadata):
    """Report net count differences, never treat index pulses as external truth."""
    configured = {packet.get("counts_per_rev") for packet in metadata
                  if type(packet.get("counts_per_rev")) in (int, float)
                  and math.isfinite(packet["counts_per_rev"]) and packet["counts_per_rev"] > 0}
    configured_cpr = next(iter(configured)) if len(configured) == 1 else None
    deltas, skipped = [], 0
    previous = None
    for row in rows:
        if not row["angle_eligible"] or not row["index_valid"] or row["index_raw_count"] is None:
            previous = None
            continue
        if previous is not None and row["segment_id"] == previous["segment_id"]:
            event_step = row["index_event_count"] - previous["index_event_count"]
            if event_step == 1:
                deltas.append(int(row["index_raw_count"]) - int(previous["index_raw_count"]))
            elif event_step > 1 or event_step < 0:
                skipped += 1
        previous = row
    residuals = [abs(delta) - configured_cpr for delta in deltas] if configured_cpr else []
    return {
        "configured_counts_per_rev": configured_cpr,
        "metadata_configuration_consistent": len(configured) == 1,
        "consecutive_index_intervals": len(deltas),
        "skipped_index_event_intervals": skipped,
        "net_count_delta_min": min(deltas) if deltas else None,
        "net_count_delta_max": max(deltas) if deltas else None,
        "mean_abs_delta_minus_configured_cpr": statistics.fmean(residuals) if residuals else None,
        "interpretation": "Internal consistency only. Reversing across the same index can yield zero net counts; this does not establish PPR or physical accuracy.",
    }


def analyze(envelopes, reference=None, clock_mapper=None, max_match_dt_s=.02,
            max_continuity_gap_s=.1, wheel_id=None):
    """Pure analysis entry point; returns rows, metadata envelopes, and summary."""
    if not math.isfinite(max_match_dt_s) or max_match_dt_s < 0:
        raise ValueError("max_match_dt_s must be finite and nonnegative")
    if not math.isfinite(max_continuity_gap_s) or max_continuity_gap_s <= 0:
        raise ValueError("max_continuity_gap_s must be finite and positive")
    clock_mapper = clock_mapper or protocol.ClockMapper()
    max_match_ns = seconds_to_ns(max_match_dt_s)
    groups, metadata = defaultdict(list), []
    rows = []
    seen, highest, conflicts = {}, {}, set()
    packet_counts = Counter()
    guard = WheelIdentityGuard(wheel_id)
    for envelope, bag_timestamp_ns in envelopes:
        validate_envelope(envelope)
        packet = envelope["packet"]
        # A local run intentionally contains both casters. An explicit selector
        # filters those exports; live single-topic captures still reject a
        # conflicting identity instead of silently masking routing mistakes.
        if envelope.get("origin") == LOCAL_ORIGIN and wheel_id is not None and packet["wheel_id"] != wheel_id:
            continue
        key = guard.accept(packet)
        packet_counts[key] += 1
        groups[key]  # Retain metadata-only streams in the report.
        if packet["type"] == "metadata":
            metadata.append(envelope)
            continue
        for sample in packet["samples"]:
            sample_key = (*key, sample["seq"])
            if sample_key in seen:
                status = "duplicate"
                if sample != seen[sample_key]:
                    conflicts.add(sample_key)
            else:
                status = "reordered" if sample["seq"] < highest.get(key, -1) else "in_order"
                seen[sample_key] = sample
                highest[key] = max(highest.get(key, -1), sample["seq"])
            mapped = clock_mapper.map(key[2], key[3], sample["t_us"],
                                      wheel_id=key[0], device_id=key[1])
            local = envelope.get("origin") == LOCAL_ORIGIN
            estimated = mapped is not None and key in getattr(clock_mapper, "estimated_keys", set())
            if local and envelope.get("clock_mapping"):
                if mapped is None:
                    local_mapper = protocol.ClockMapper({"version": 1, "mappings": [envelope["clock_mapping"]]})
                    mapped = local_mapper.map(key[2], key[3], sample["t_us"], key[0], key[1])
                estimated = mapped is not None
            diagnostics_known = envelope.get("diagnostics_known", True)
            row = {name: envelope[name] for name in CSV_FIELDS[:4]}
            row.update(wheel_id=key[0], device_id=key[1], node_id=key[2],
                       boot_id=key[3], bag_timestamp_ns=bag_timestamp_ns)
            row.update({name: sample[name] for name in protocol.SAMPLE_FIELDS})
            row.update({name: int(bool(sample["flags"] & (1 << bit)))
                        for bit, name in enumerate(protocol.FLAG_NAMES)})
            row.update({name: packet["diagnostics"][name] if diagnostics_known else None
                        for name in protocol.DIAGNOSTIC_FIELDS})
            row.update(sequence_status=status, mapped_epoch_ns=mapped,
                       mapped_clock_sync_valid=int(mapped is not None and not estimated),
                       comparison_time_basis=("estimated_acquisition" if estimated else "externally_calibrated_acquisition")
                           if mapped is not None else ("unmapped_local_acquisition" if local else "host_receive"),
                       origin=LOCAL_ORIGIN if local else "udp_receive", timing_estimated=int(estimated),
                       diagnostics_known=int(diagnostics_known),
                       conflicting_duplicate=0, segment_id=None,
                       reference_time_epoch_ns=None, reference_angle_rad=None,
                       reference_match_dt_ns=None, angle_error_rad=None)
            rows.append(row)
            groups[key].append(row)
    if not packet_counts:
        raise ValueError("no caster encoder packets found")
    summaries = []
    for key, group in sorted(groups.items()):
        unique = sorted((row for row in group if row["sequence_status"] != "duplicate"),
                        key=lambda row: row["seq"])
        for row in group:
            row["conflicting_duplicate"] = int((*key, row["seq"]) in conflicts)
            row["comparison_exclusion"] = angle_exclusion(row)
            row["angle_eligible"] = int(not row["comparison_exclusion"])
            row["velocity_eligible"] = int(bool(row["angle_eligible"] and row["velocity_valid"]
                                                   and row["unwrap_continuous"] and not row["continuity_lost"]
                                                   and row["velocity_rad_s"] is not None))
        previous, segment = None, 0
        for row in unique:
            if previous is not None:
                gap_us = row["t_us"] - previous["t_us"]
                if (row["seq"] != previous["seq"] + 1 or gap_us <= 0
                        or gap_us > max_continuity_gap_s * 1e6
                        or not previous["angle_eligible"] or not row["angle_eligible"]):
                    segment += 1
            row["segment_id"] = segment
            previous = row
        references = (reference or {}).get(key, [])
        if key[0] == "legacy" and not references:
            references = (reference or {}).get(key[2:], [])
        reference_times = [item[0] for item in references]
        matched = []
        for row in unique:
            if not row["angle_eligible"] or not references:
                continue
            timestamp = row["mapped_epoch_ns"] if row["mapped_epoch_ns"] is not None else row["receive_epoch_ns"]
            if timestamp is None:
                continue
            insertion = bisect_left(reference_times, timestamp)
            candidates = [index for index in (insertion - 1, insertion) if 0 <= index < len(references)]
            index = min(candidates, key=lambda i: (abs(reference_times[i] - timestamp), reference_times[i]))
            ref_time, ref_angle = references[index]
            if abs(ref_time - timestamp) > max_match_ns:
                continue
            angle = row["calibrated_alpha_rad"] if key[2] == 3 else row["angle_rad"]
            error = angle - ref_angle
            if key[2] == 3:
                error = math.atan2(math.sin(error), math.cos(error))
            row.update(reference_time_epoch_ns=ref_time, reference_angle_rad=ref_angle,
                       reference_match_dt_ns=ref_time - timestamp, angle_error_rad=error)
            matched.append(row)
        sequence_min = unique[0]["seq"] if unique else None
        sequence_max = unique[-1]["seq"] if unique else None
        expected = sequence_max - sequence_min + 1 if unique else 0
        missing = expected - len(unique)
        local_span = (max(row["t_us"] for row in unique) - min(row["t_us"] for row in unique)) * 1e-6 if unique else 0.0
        receive_times = [row["receive_monotonic_ns"] for row in unique if row["receive_monotonic_ns"] is not None]
        receive_span = (max(receive_times) - min(receive_times)) * 1e-9 if receive_times else 0.0
        stream_metadata = [item["packet"] for item in metadata if
                           protocol.packet_identity(item["packet"]) == key]
        comparison = error_statistics([row["angle_error_rad"] for row in matched])
        comparison.update(
            status="reference_agreement" if matched else "accuracy_unavailable",
            reason=("Independent angle reference matched; interpret calibration and timing qualifications below."
                    if matched else "No matching valid independent angle reference; reception/health quality alone cannot measure accuracy."),
            angle_field="calibrated_alpha_rad" if key[2] == 3 else "angle_rad",
            error_definition="circular encoder minus reference" if key[2] == 3 else "unwrapped relative encoder minus reference; missed turns retained",
            eligible_samples=sum(row["angle_eligible"] for row in unique),
            unmatched_eligible_samples=sum(row["angle_eligible"] for row in unique) - len(matched),
            unique_reference_points_matched=len({row["reference_time_epoch_ns"] for row in matched}),
            matched_with_physical_phase_verified=sum(row["phase_reference_valid"] for row in matched),
            matched_with_ppr_verified=sum(row["ppr_verified"] for row in matched) if key[2] != 3 else None,
            matched_using_host_receive_time=sum(row["mapped_epoch_ns"] is None for row in matched),
            matched_using_calibrated_acquisition_time=sum(row["mapped_clock_sync_valid"] for row in matched),
            matched_using_estimated_acquisition_time=sum(row["timing_estimated"] for row in matched),
            absolute_accuracy_established=False,
            calibration_note="No automatic sign/zero/clock fit. H1/H2 are relative angles. CENTER alpha requires a verified physical phase for an absolute claim. Reference and clock uncertainty require independent validation.",
            timing_note="Host receipt includes unknown Wi-Fi/batching/scheduling latency; matching tolerance does not bound acquisition-time error. External clock maps retain their independently assessed uncertainty.",
        )
        summaries.append({
            "wheel_id": key[0], "device_id": key[1],
            "node_id": key[2], "role": ROLES[key[2]], "boot_id": key[3],
            "packets_and_retained_metadata_entries": packet_counts[key], "metadata_entries": len(stream_metadata),
            "received_sample_rows": len(group), "unique_samples": len(unique),
            "duplicate_samples": len(group) - len(unique),
            "conflicting_duplicate_sequences": sum((*key, row["seq"]) in conflicts for row in unique),
            "reordered_samples": sum(row["sequence_status"] == "reordered" for row in unique),
            "first_observed_sequence": sequence_min, "last_observed_sequence": sequence_max,
            "samples_before_observed_range": sequence_min,
            "missing_between_observed_sequences": missing,
            "missing_fraction_observed_range": missing / expected if expected else None,
            "loss_note": "Gaps combine acquisition, queue, radio, bridge, and bag loss; not radio loss alone. Earlier and trailing unobserved samples cannot be classified from this recording.",
            "local_acquisition_span_s": local_span,
            "host_reception_span_s": receive_span,
            "observed_samples_per_local_second": (len(unique) - 1) / local_span if local_span > 0 else None,
            "sequence_rate_per_local_second": (sequence_max - sequence_min) / local_span if local_span > 0 else None,
            "observed_samples_per_receive_second": (len(unique) - 1) / receive_span if receive_span > 0 else None,
            "continuity_segments": segment + 1 if unique else 0,
            "flag_counts_unique": {name: sum(row[name] for row in unique) for name in protocol.FLAG_NAMES},
            "flag_fraction_unique": {name: sum(row[name] for row in unique) / len(unique) if unique else None for name in protocol.FLAG_NAMES},
            "firmware_counter_maxima": {name: max((row[name] for row in group if row[name] is not None), default=None) for name in protocol.DIAGNOSTIC_FIELDS},
            "firmware_counter_note": "Cumulative boot counters are maxima, never sums of repeated per-sample values; maxima can include events before recording.",
            "comparison_exclusion_counts": dict(Counter(row["comparison_exclusion"] for row in unique if row["comparison_exclusion"])),
            "reference_comparison": comparison,
            "index_consistency": index_consistency(unique, stream_metadata) if key[2] != 3 else None,
        })
    summary = {
        "schema_version": 2, "wheel_id": guard.wheel_id, "streams": summaries,
        "reference_supplied": reference is not None,
        "max_match_dt_s": max_match_dt_s,
        "max_continuity_gap_s": max_continuity_gap_s,
        "scope": "Caster H1/H2 hemisphere and CENTER swivel encoders. These are not HAMR drive-wheel ground truth.",
        "accuracy_status": "reference_agreement" if any(s["reference_comparison"]["count"] for s in summaries) else "accuracy_unavailable",
        "integer_precision_note": "Load nanosecond timestamps, sequence/count and index columns as integers or strings, not spreadsheet floating point.",
    }
    return rows, metadata, summary


def plot_report(rows, output):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError("Plots need a working matplotlib installation; install python3-matplotlib or use --no-plots for CSV/JSON only.") from error
    groups = defaultdict(list)
    for row in rows:
        if row["sequence_status"] != "duplicate":
            groups[(row["wheel_id"], row["device_id"], row["node_id"], row["boot_id"])].append(row)
    outputs = []
    for (wheel, device, node, boot), group in sorted(groups.items()):
        group.sort(key=lambda row: row["seq"])
        origin = min(row["t_us"] for row in group)
        x = [(row["t_us"] - origin) * 1e-6 for row in group]
        stem = f"{wheel}_{device.replace(chr(58), chr(45))}_node{node}_{boot}"
        title = f"{wheel} / {ROLES[node]} / {device} / boot {boot}"

        def series(ax, field, gate, label, wrap=False):
            sx, sy = [], []
            previous = None
            for row, time_s in zip(group, x):
                value = row[field] if row[gate] else None
                if previous is not None and (row["segment_id"] != previous["segment_id"] or
                        (wrap and value is not None and previous[field] is not None
                         and abs(value - previous[field]) > math.pi)):
                    sx.append(float("nan"))
                    sy.append(float("nan"))
                sx.append(time_s)
                sy.append(value if value is not None else float("nan"))
                previous = row
            ax.plot(sx, sy, ".-", markersize=2, linewidth=.8, label=label)

        fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
        field = "calibrated_alpha_rad" if node == 3 else "angle_rad"
        series(axes[0], field, "angle_eligible", "CENTER alpha (wrapped)" if node == 3 else "relative angle", node == 3)
        valid_reference = [i for i, row in enumerate(group) if row["reference_angle_rad"] is not None]
        if valid_reference:
            axes[0].scatter([x[i] for i in valid_reference],
                            [group[i]["reference_angle_rad"] for i in valid_reference],
                            s=4, alpha=.65, label="independent reference at matched time")
        series(axes[1], "velocity_rad_s", "velocity_eligible", "raw velocity")
        series(axes[1], "filtered_velocity_rad_s", "velocity_eligible", "filtered velocity")
        axes[0].set(title=title, ylabel="angle (rad)")
        axes[1].set(ylabel="velocity (rad/s)", xlabel="node acquisition time since first observed sample (s)")
        for ax in axes:
            ax.grid(alpha=.25)
            ax.legend(loc="best")
        fig.tight_layout()
        path = output / f"{stem}_angles.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        outputs.append(path.name)

        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        flags = ("ready", "sample_valid", "unwrap_continuous", "continuity_lost", "velocity_valid", "phase_reference_valid", "ppr_verified", "parity_error", "spi_io_error")
        for index, name in enumerate(flags):
            active = [i for i, row in enumerate(group) if row[name]]
            axes[0].scatter([x[i] for i in active], [index] * len(active), s=3)
        axes[0].set_yticks(list(range(len(flags))))
        axes[0].set_yticklabels(flags, fontsize=8)
        axes[0].set_title(title + " — unique sample quality (dots = flag set)")
        missing = [max(0, group[i]["seq"] - group[i - 1]["seq"] - 1) for i in range(1, len(group))]
        axes[1].scatter(x[1:], missing, s=8, label="unreceived sequences before sample")
        axes[1].set(ylabel="gap size (samples)")
        for name in protocol.DIAGNOSTIC_FIELDS:
            # Dots deliberately do not connect missing intervals or reboots.
            axes[2].plot(x, [row[name] for row in group], ".", markersize=2, label=name)
        axes[2].set(ylabel="cumulative firmware count", xlabel="node acquisition time since first observed sample (s)")
        for ax in axes:
            ax.grid(alpha=.25)
        axes[1].legend(loc="best")
        axes[2].legend(loc="best", fontsize=8)
        fig.tight_layout()
        path = output / f"{stem}_quality.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        outputs.append(path.name)
        matched = [i for i, row in enumerate(group) if row["angle_error_rad"] is not None]
        if matched:
            fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
            axes[0].scatter([x[i] for i in matched], [group[i]["angle_error_rad"] for i in matched], s=5)
            axes[0].axhline(0, color="gray", linewidth=.8)
            axes[0].set(ylabel="encoder − reference (rad)", title=title + " — reference agreement\nSee timing/calibration qualifications in summary.json")
            axes[1].scatter([x[i] for i in matched], [group[i]["reference_match_dt_ns"] / 1e6 for i in matched], s=5)
            axes[1].set(ylabel="reference time − match time (ms)", xlabel="node acquisition time since first observed sample (s)")
            for ax in axes:
                ax.grid(alpha=.25)
            fig.tight_layout()
            path = output / f"{stem}_error.png"
            fig.savefig(path, dpi=150)
            plt.close(fig)
            outputs.append(path.name)
    return outputs


def node_values(values, name, sign=False):
    result = {}
    for value in values:
        try:
            node_text, number_text = value.split("=", 1)
            node = int(node_text)
            number = finite(number_text, name)
            if node not in ROLES or node in result or (sign and number not in (-1, 1)):
                raise ValueError("node must be 1, 2 or 3 once; sign must be +1 or -1")
            result[node] = number
        except (ValueError, TypeError) as error:
            raise ValueError(f"{name} expects NODE=VALUE: {error}") from error
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", nargs="?", type=Path, help="ROS 2 bag directory or .mcap/.db3 file")
    parser.add_argument("--packets-jsonl", type=Path, help="logger packets.jsonl instead of a ROS bag")
    parser.add_argument("--output", type=Path, default=Path("caster_encoder_report"))
    parser.add_argument("--topic", help="packet topic; inferred from --wheel or unique bag topic")
    parser.add_argument("--wheel", choices=WHEELS, help="physical wheel; legacy selects old v1 recordings")
    parser.add_argument("--reference-csv", type=Path, help="independent reference: node_id,boot_id,time_epoch_ns,angle_rad; v2 requires wheel_id,device_id; optional valid=0/1")
    parser.add_argument("--max-match-dt-s", type=float, default=.02)
    parser.add_argument("--reference-time-offset-s", type=float, default=0,
                        help="fixed independent offset ADDED to reference timestamps; never fitted")
    parser.add_argument("--reference-sign", action="append", default=[], metavar="NODE=SIGN")
    parser.add_argument("--reference-zero-rad", action="append", default=[], metavar="NODE=ANGLE",
                        help="reference becomes sign * (angle - zero); repeat per node; never fitted")
    parser.add_argument("--clock-map", type=Path, help="independently calibrated per-node/boot logger clock-map JSON")
    parser.add_argument("--max-continuity-gap-s", type=float, default=.1)
    parser.add_argument("--no-plots", action="store_true", help="write CSV/JSON without matplotlib")
    args = parser.parse_args(argv)
    if (args.bag is None) == (args.packets_jsonl is None):
        parser.error("provide exactly one of BAG or --packets-jsonl")
    try:
        signs = node_values(args.reference_sign, "--reference-sign", sign=True)
        zeros = node_values(args.reference_zero_rad, "--reference-zero-rad")
        seconds_to_ns(args.reference_time_offset_s)
        if not args.reference_csv and (signs or zeros or args.reference_time_offset_s):
            raise ValueError("reference transforms require --reference-csv")
        reference = read_reference(args.reference_csv, args.reference_time_offset_s, signs, zeros) if args.reference_csv else None
        clock_document = protocol.parse_json(args.clock_map.read_text(encoding="utf-8")) if args.clock_map else None
        mapper = protocol.ClockMapper(clock_document)
        mapper.estimated_keys = {protocol.packet_identity(entry) for entry in
                                 (clock_document or {}).get("mappings", [])
                                 if entry.get("timing_estimated") or entry.get("accuracy_validated") is False}
        wheel_id, selected_topic = resolve_selection(args.wheel, args.topic)
        source = read_jsonl(args.packets_jsonl) if args.packets_jsonl else read_bag(args.bag, args.topic, args.wheel)
        # Preserve original envelopes, including extra metadata/configuration fields.
        packets = list(source)
        rows, metadata, summary = analyze(packets, reference, mapper, args.max_match_dt_s, args.max_continuity_gap_s, wheel_id)
        _, selected_topic = resolve_selection(summary["wheel_id"], args.topic)
        summary.update(source=str(args.packets_jsonl or args.bag), topic=selected_topic,
                       reference_csv=str(args.reference_csv) if args.reference_csv else None,
                       reference_time_offset_s=args.reference_time_offset_s,
                       reference_sign_by_node=signs, reference_zero_rad_by_node=zeros,
                       clock_map=str(args.clock_map) if args.clock_map else None)
        args.output.mkdir(parents=True, exist_ok=True)
        # A report directory is a reproducible artifact, not an appendable session.
        if any(args.output.iterdir()):
            raise ValueError(f"report directory must be empty to avoid mixing runs: {args.output}")
        with (args.output / "samples.csv").open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        for name, entries in (("packets.jsonl", [item[0] for item in packets if "metadata_source_topic" not in item[0]]), ("metadata.jsonl", metadata)):
            with (args.output / name).open("w", encoding="utf-8") as handle:
                for envelope in entries:
                    handle.write(json.dumps(envelope, allow_nan=False, separators=(",", ":")) + "\n")
        if clock_document is not None:
            (args.output / "clock-map.json").write_text(json.dumps(clock_document, indent=2) + "\n", encoding="utf-8")
        summary["plots"] = []
        plot_error = None
        if not args.no_plots:
            try:
                summary["plots"] = plot_report(rows, args.output)
            except RuntimeError as error:
                plot_error = str(error)
                summary["plot_error"] = plot_error
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(f"Saved {len(rows)} sample rows across {len(summary['streams'])} node/boot streams to {args.output}")
        print(f"Accuracy status: {summary['accuracy_status']}. See summary.json for timing and calibration qualifications.")
        if plot_error:
            print(plot_error, file=sys.stderr)
            return 1
        return 0
    except (ValueError, OSError, RuntimeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
