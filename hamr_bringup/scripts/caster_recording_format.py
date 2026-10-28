#!/usr/bin/env python3
"""Strict, ROS-independent decoder and time estimator for onboard caster logs.

Binary input is retained as the authority. Exports explicitly identify local
acquisition: HTTP download timestamps are never substituted for sample times.
Two-way clock exchanges estimate alignment, not independently proven accuracy.
"""

from __future__ import annotations

import csv
from decimal import Decimal, localcontext
import json
import math
import os
from pathlib import Path
import re
import struct
import zlib

import caster_encoder_protocol as protocol


HEADER_BYTES = 4096
HEADER = struct.Struct("<8sIIII")
RECORD = struct.Struct("<QqqHhI5dQqqI")
MAGIC = b"CASTLOG1"
RUN_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}\Z")
LOCAL_ORIGIN = "encoder_local_recording"
CSV_FIELDS = (*protocol.CSV_FIELDS, "origin", "run_id", "timing_estimated",
              "timing_uncertainty_ns", "diagnostics_known")


def ordinary_recording():
    protocol.ordinary_protocol()  # Makes the source/installed package available.
    from ordinary_caster import recording
    return recording


def _layout(metadata):
    if metadata["packet"].get("caster_type") == "ordinary":
        ordinary = ordinary_recording()
        return ordinary.RECORD, ordinary.decode_record
    return RECORD, decode_record


def _timestamps(sample):
    return (sample["t_us"], sample["wheel"]["t_us"]) if "wheel" in sample else (sample["t_us"],)


def _check_order(previous, sample):
    if previous is not None and (sample["seq"] <= previous["seq"] or any(
            now <= before for now, before in zip(_timestamps(sample), _timestamps(previous)))):
        raise ValueError("sample sequence and each channel acquisition time must strictly increase")


def _json(value):
    return json.dumps(value, allow_nan=False, separators=(",", ":"))


def _run_id(value):
    if not isinstance(value, str) or not RUN_ID.fullmatch(value):
        raise ValueError("run_id must be 1..64 safe ASCII filename characters")
    return value


def read_header(handle):
    data = handle.read(HEADER_BYTES)
    if len(data) != HEADER_BYTES:
        raise ValueError("truncated recording header")
    magic, version, record_size, length, checksum = HEADER.unpack_from(data)
    ordinary = ordinary_recording()
    ball_layout = magic == MAGIC and version == 1 and record_size == RECORD.size
    ordinary_layout = magic == ordinary.MAGIC and version == ordinary.VERSION and record_size == ordinary.RECORD.size
    if not ball_layout and not ordinary_layout:
        raise ValueError("unsupported recording magic/version/record size")
    if not 0 < length <= HEADER_BYTES - HEADER.size:
        raise ValueError("invalid metadata length")
    payload = data[HEADER.size:HEADER.size + length]
    if zlib.crc32(payload) != checksum:
        raise ValueError("metadata CRC mismatch")
    if any(data[HEADER.size + length:]):
        raise ValueError("nonzero header padding")
    try:
        metadata = protocol.parse_json(payload.decode("utf-8"))
        if not isinstance(metadata, dict):
            raise ValueError("recording metadata must be an object")
        _run_id(metadata.get("run_id"))
        packet = protocol.decode_packet(_json(metadata.get("packet")).encode())
        if packet["v"] not in ((3, 4) if ordinary_layout else (2,)) or packet["type"] != "metadata":
            raise ValueError("recording header version must match its metadata packet")
    except (UnicodeError, TypeError) as error:
        raise ValueError(f"invalid recording metadata: {error}") from error
    return metadata


def decode_record(data, node_id):
    if len(data) != RECORD.size:
        raise ValueError("truncated sample record")
    values = RECORD.unpack(data)
    if zlib.crc32(data[:-4]) != values[-1]:
        raise ValueError("sample CRC mismatch")
    (seq, timestamp, raw, spi, position, flags, angle, absolute, calibrated,
     velocity, filtered, index_events, index_timestamp, index_raw, _) = values
    protocol.integer(node_id, "node_id", 1, 3)
    protocol.integer(timestamp, "t_us", 0, protocol.INT64_MAX)
    protocol.integer(index_timestamp, "index_timestamp_us", 0, protocol.INT64_MAX)
    protocol.integer(flags, "flags", 0, (1 << 13) - 1)
    protocol.integer(position, "position14", -1, 16383)
    sample = dict(seq=seq, t_us=timestamp, flags=flags,
                  raw_count=str(raw) if node_id != 3 and flags & protocol.SAMPLE_VALID else None,
                  raw_spi_word=spi if node_id == 3 else None,
                  position14=position if node_id == 3 and position >= 0 else None,
                  index_event_count=index_events, index_timestamp_us=index_timestamp,
                  index_raw_count=str(index_raw) if node_id != 3 else None)
    # Firmware serializes all non-finite readings as JSON null, including the
    # intentionally unavailable NaNs in fields belonging to the other role.
    for name, value in zip(("angle_rad", "absolute_angle_rad", "calibrated_alpha_rad",
                            "velocity_rad_s", "filtered_velocity_rad_s"),
                           (angle, absolute, calibrated, velocity, filtered)):
        sample[name] = value if math.isfinite(value) else None
    return sample


def iter_samples(path):
    """Yield samples after header/record validation, retaining exact int64 values."""
    with Path(path).open("rb") as handle:
        metadata = read_header(handle)
        record, decoder = _layout(metadata)
        previous = None
        while data := handle.read(record.size):
            sample = decoder(data, metadata["packet"]["node_id"])
            _check_order(previous, sample)
            previous = sample
            yield sample


def verify_log(path, expected_status=None, expected_run_id=None):
    """Validate every byte plus the immutable download status; return a summary.

    Sequence gaps are reported, not silently filled. The first sequence need
    not be zero: acquisition runs while local recording is idle.
    """
    count, checksum, missing, gaps, max_gap_us = 0, 0, 0, [], 0
    first = previous = None
    with Path(path).open("rb") as handle:
        metadata = read_header(handle)
        packet = metadata["packet"]
        record, decoder = _layout(metadata)
        is_ordinary = packet.get("caster_type") == "ordinary"
        flag_names = protocol.ordinary_protocol().FLAG_NAMES if is_ordinary else protocol.FLAG_NAMES
        flag_counts = ({channel: {name: 0 for name in flag_names} for channel in protocol.ordinary_protocol().channels(packet)}
                       if is_ordinary else {name: 0 for name in flag_names})
        if expected_run_id is not None and metadata["run_id"] != expected_run_id:
            raise ValueError("recording run_id mismatch")
        while data := handle.read(record.size):
            sample = decoder(data, packet["node_id"])
            checksum = zlib.crc32(data, checksum)
            count += 1
            for bit, name in enumerate(flag_names):
                if is_ordinary:
                    for channel in protocol.ordinary_protocol().channels(packet):
                        flag_counts[channel][name] += int(bool(sample[channel]["flags"] & (1 << bit)))
                else:
                    flag_counts[name] += int(bool(sample["flags"] & (1 << bit)))
            _check_order(previous, sample)
            if previous is not None:
                max_gap_us = max(max_gap_us, *(now - before for now, before in
                                             zip(_timestamps(sample), _timestamps(previous))))
                if sample["seq"] > previous["seq"] + 1:
                    gap = [previous["seq"] + 1, sample["seq"] - 1]
                    gaps.append(gap)
                    missing += gap[1] - gap[0] + 1
            if first is None:
                first = sample
            previous = sample
    result = {"metadata": metadata, "run_id": metadata["run_id"],
              **{key: packet[key] for key in ("wheel_id", "device_id", "node_id", "boot_id")},
              "sample_count": count, "data_bytes": HEADER_BYTES + count * record.size,
              "crc32": checksum, "missing_samples": missing, "gaps": gaps,
              "max_acquisition_gap_us": max_gap_us, "flag_counts": flag_counts,
              "integrity_note": "Byte integrity and observed sequence gaps do not prove uninterrupted acquisition; inspect overrun flags and acquisition time gaps.",
              "first_seq": first["seq"] if first else None,
              "last_seq": previous["seq"] if previous else None,
              "first_t_us": min(_timestamps(first)) if first else None,
              "last_t_us": max(_timestamps(previous)) if previous else None}
    if is_ordinary:
        result.update(caster_type="ordinary", record_version=2, record_bytes=record.size,
                      first_swivel_t_us=first["t_us"] if first and "swivel" in first else None,
                      first_wheel_t_us=first["wheel"]["t_us"] if first and "wheel" in first else None,
                      last_swivel_t_us=previous["t_us"] if previous and "swivel" in previous else None,
                      last_wheel_t_us=previous["wheel"]["t_us"] if previous and "wheel" in previous else None)
    if expected_status is not None:
        if not isinstance(expected_status, dict):
            raise ValueError("expected recording status must be an object")
        if expected_status.get("caster_type", "ball") != packet.get("caster_type", "ball"):
            raise ValueError("recording caster_type differs from device status")
        if is_ordinary:
            for name, value in (("record_version", 2), ("record_bytes", record.size)):
                if name in expected_status and expected_status[name] != value:
                    raise ValueError(f"recording {name} differs from device status")
        for name in ("sample_count", "data_bytes", "crc32"):
            protocol.integer(expected_status.get(name), name)
            if expected_status[name] != result[name]:
                raise ValueError(f"recording {name} differs from device status")
        for name in ("wheel_id", "device_id", "node_id", "run_id"):
            if expected_status.get(name) != result[name]:
                raise ValueError(f"recording {name} differs from device status")
        # Current device boot may differ after recovery; the log's boot is the
        # only correct identity for sample timestamps and synchronization.
        log_boot = expected_status.get("log_boot_id")
        if not isinstance(log_boot, str) or log_boot.lower() != packet["boot_id"].lower():
            raise ValueError("recording log_boot_id differs from device status")
        bounds = {}
        for name in ("start_us", "stop_us"):
            value = expected_status.get(name)
            if value is None:
                continue  # Older captures and interrupted recovery may lack bounds.
            protocol.integer(value, name, -1, protocol.INT64_MAX)
            if value >= 0:
                bounds[name] = value
        if "start_us" in bounds and "stop_us" in bounds and bounds["stop_us"] < bounds["start_us"]:
            raise ValueError("recording stop_us precedes start_us")
        if first is not None and "start_us" in bounds and min(_timestamps(first)) < bounds["start_us"]:
            raise ValueError("recording sample precedes requested start_us")
        if previous is not None and "stop_us" in bounds and max(_timestamps(previous)) > bounds["stop_us"]:
            raise ValueError("recording sample exceeds requested stop_us")
    return result


def _anchor(exchange, expected_identity, max_clock_step_ns):
    if not isinstance(exchange, dict) or not isinstance(exchange.get("device"), dict):
        raise ValueError("clock exchange requires a device timestamp response")
    device = exchange["device"]
    protocol.identity(device)
    protocol.wheel_device_identity(device)
    if protocol.packet_identity(device) != protocol.packet_identity(expected_identity):
        raise ValueError("clock exchange identity/boot differs from recording")
    for key in ("host_send_epoch_ns", "host_recv_epoch_ns", "host_send_monotonic_ns", "host_recv_monotonic_ns"):
        protocol.integer(exchange.get(key), key, 0, (1 << 127) - 1)
    for key in ("rx_us", "tx_us"):
        protocol.integer(device.get(key), key, 0, protocol.INT64_MAX)
    sent, received = exchange["host_send_epoch_ns"], exchange["host_recv_epoch_ns"]
    mono_sent, mono_received = exchange["host_send_monotonic_ns"], exchange["host_recv_monotonic_ns"]
    elapsed = mono_received - mono_sent
    processing = (device["tx_us"] - device["rx_us"]) * 1000
    if elapsed <= 0 or received <= sent or processing < 0:
        raise ValueError("clock exchange has non-increasing timestamps")
    if abs((received - sent) - elapsed) > max_clock_step_ns:
        raise ValueError("host epoch clock stepped during clock exchange")
    # A 0.5% rate allowance permits clocks within the configured drift gate;
    # processing cannot take longer than the measured request round trip.
    if processing > elapsed * 1005 // 1000 + 1000:
        raise ValueError("device processing time exceeds clock exchange round trip")
    return {"device_us": Decimal(device["rx_us"] + device["tx_us"]) / 2,
            "epoch_ns": Decimal(sent + received) / 2,
            "rtt_ns": elapsed, "processing_ns": processing,
            "epoch_monotonic_offset_ns": (sent - mono_sent, received - mono_received)}


def fit_clock_map(sync_pre, sync_post, identity, valid_from_us, valid_to_us,
                  max_drift_ppm=2000, max_clock_step_ns=5_000_000,
                  minimum_span_us=1_000_000, max_anchor_rtt_ns=1_000_000_000):
    """Fit the lowest-RTT pre/post anchors, with explicit unproven uncertainty.

    ``calibrated`` is ClockMapper's legacy apply-map gate; accuracy_validated
    remains false. Exporters/analyzers must not turn this into a sync-valid flag.
    """
    protocol.identity(identity)
    protocol.wheel_device_identity(identity)
    protocol.integer(valid_from_us, "valid_from_us", 0, protocol.INT64_MAX)
    protocol.integer(valid_to_us, "valid_to_us", valid_from_us, protocol.INT64_MAX)
    if not sync_pre or not sync_post:
        raise ValueError("both pre-recording and post-recording clock exchanges are required")
    if not math.isfinite(max_drift_ppm) or not 0 < max_drift_ppm <= 5000:
        raise ValueError("max_drift_ppm must be in (0, 5000]")
    protocol.integer(max_clock_step_ns, "max_clock_step_ns", 1)
    protocol.integer(minimum_span_us, "minimum_span_us", 1)
    protocol.integer(max_anchor_rtt_ns, "max_anchor_rtt_ns", 1)
    with localcontext() as context:
        context.prec = 60
        before = [_anchor(item, identity, max_clock_step_ns) for item in sync_pre]
        after = [_anchor(item, identity, max_clock_step_ns) for item in sync_post]
        offsets = [offset for anchor in before + after for offset in anchor["epoch_monotonic_offset_ns"]]
        if max(offsets) - min(offsets) > max_clock_step_ns:
            raise ValueError("host epoch clock stepped or slewed excessively across recording")
        left, right = (min(group, key=lambda anchor: anchor["rtt_ns"] - anchor["processing_ns"])
                       for group in (before, after))
        if max(left["rtt_ns"], right["rtt_ns"]) > max_anchor_rtt_ns:
            raise ValueError("clock anchors exceed maximum round-trip latency")
        span = right["device_us"] - left["device_us"]
        if span < minimum_span_us:
            raise ValueError("clock anchors have insufficient or reversed time span")
        if left["device_us"] > valid_from_us or right["device_us"] < valid_to_us:
            raise ValueError("clock anchors do not bracket recorded samples")
        slope = (right["epoch_ns"] - left["epoch_ns"]) / span
        drift = (slope / 1000 - 1) * 1_000_000
        if abs(drift) > Decimal(str(max_drift_ppm)):
            raise ValueError("estimated clock drift exceeds configured limit")
        offset = int((left["epoch_ns"] - slope * left["device_us"]).to_integral_value())
        # Conservative latency envelope for each selected anchor; no claim
        # about oscillator changes between anchors or asymmetric network paths.
        uncertainty = max(left["rtt_ns"], right["rtt_ns"]) // 2 + 1000
        residual = max(abs(anchor["epoch_ns"] - (slope * anchor["device_us"] + offset))
                       for anchor in before + after)
        mapping = {**{name: identity[name] for name in ("wheel_id", "device_id", "node_id", "boot_id")},
                   "calibrated": True, "slope_ns_per_us": str(slope), "offset_ns": offset,
                   "valid_from_us": valid_from_us, "valid_to_us": valid_to_us,
                   "method": "two_way_midpoint_estimate", "accuracy_validated": False,
                   "timing_estimated": True, "drift_ppm": str(drift),
                   "anchor_span_us": str(span), "uncertainty_ns": uncertainty,
                   "max_probe_residual_ns": int(residual.to_integral_value()),
                   "pre_anchor_rtt_ns": left["rtt_ns"], "post_anchor_rtt_ns": right["rtt_ns"],
                   "uncertainty_note": "RTT midpoint estimate; asymmetric transport, clock wander and host clock discipline need independent validation. This is not a proven accuracy bound."}
        if identity.get("caster_type") == "ordinary":
            mapping["caster_type"] = "ordinary"
        protocol.ClockMapper({"version": 1, "mappings": [mapping]})
        return mapping


def validate_local_envelope(value):
    """Validate offline provenance separately from live UDP receive envelopes."""
    if not isinstance(value, dict):
        raise ValueError("local recording envelope must be an object")
    if value.get("origin") != LOCAL_ORIGIN:
        raise ValueError("unknown local recording origin")
    if any(name not in value or value[name] is not None for name in protocol.CSV_FIELDS[:4]):
        raise ValueError("local recording cannot invent receive timestamps or source address")
    recording = value.get("recording")
    if not isinstance(recording, dict):
        raise ValueError("local recording requires recording provenance")
    _run_id(recording.get("run_id"))
    if not isinstance(recording.get("file"), str) or not recording["file"]:
        raise ValueError("local recording requires a source filename")
    for name in ("diagnostics_known", "timing_estimated"):
        if type(value.get(name)) is not bool:
            raise ValueError(f"local recording requires explicit {name}")
    packet = protocol.decode_packet(_json(value.get("packet")).encode())
    if packet["v"] not in (2, 3, 4):
        raise ValueError("local recording requires identified v2/v3/v4 packets")
    mapping = value.get("clock_mapping")
    if mapping is not None:
        if not isinstance(mapping, dict):
            raise ValueError("local clock mapping must be an object")
        if protocol.packet_identity(mapping) != protocol.packet_identity(packet):
            raise ValueError("local recording clock mapping identity mismatch")
        protocol.ClockMapper({"version": 1, "mappings": [mapping]})
        if mapping.get("method") != "two_way_midpoint_estimate" or mapping.get("accuracy_validated") is not False:
            raise ValueError("local clock estimate must explicitly disclaim validated accuracy")
        if not value["timing_estimated"] or mapping.get("calibrated") is not True:
            raise ValueError("local clock estimate must be accepted and marked estimated")
    elif value["timing_estimated"]:
        raise ValueError("timing_estimated requires a clock mapping")
    return value


def _atomic_json(path, document):
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write(json.dumps(document, indent=2, allow_nan=False) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)


def export_run(run_directory, manifest):
    """Export verified device files together; missing clock sync is nonfatal.

    The raw binary plus manifest remain sufficient to re-estimate clock maps
    later. Paths are resolved inside this run to avoid accidental cross-run mix.
    """
    root = Path(run_directory).resolve()
    run_id = _run_id(manifest.get("run_id"))
    devices = manifest.get("devices")
    if not isinstance(devices, list):
        raise ValueError("manifest requires devices array")
    prepared, identities, clock_errors, mappings, incomplete = [], set(), [], [], []
    ordinary_guard = protocol.ordinary_protocol().IdentityGuard()
    for device in devices:
        if not device.get("log_path"):
            incomplete.append({"device_id": device.get("device_id"), "error": "no downloaded log path"})
            continue
        path = (root / device["log_path"]).resolve()
        if root not in path.parents:
            raise ValueError("device log_path must remain inside run directory")
        if not path.is_file():
            incomplete.append({"device_id": device.get("device_id"), "error": "log file has not been downloaded"})
            continue
        try:
            verified = verify_log(path, device.get("status"), run_id)
        except (ValueError, OSError) as error:
            incomplete.append({"device_id": device.get("device_id"), "error": f"log verification failed: {error}"})
            continue
        packet = verified["metadata"]["packet"]
        if packet.get("caster_type", "ball") != manifest.get("caster_type", "ball"):
            raise ValueError("manifest caster_type differs from recording")
        if packet.get("caster_type") == "ordinary":
            ordinary_guard.accept(packet)
        key = protocol.packet_identity(packet)
        if key in identities:
            raise ValueError("duplicate device/boot recording in manifest")
        identities.add(key)
        for name in ("wheel_id", "device_id", "node_id"):
            if device.get(name) != packet[name]:
                raise ValueError(f"manifest {name} differs from recording")
        mapping = None
        if verified["sample_count"]:
            try:
                mapping = fit_clock_map(device.get("sync_pre", []), device.get("sync_post", []), packet,
                                        verified["first_t_us"], verified["last_t_us"])
                mappings.append(mapping)
            except (ValueError, TypeError, KeyError) as error:
                clock_errors.append({"device_id": packet["device_id"], "error": str(error)})
        prepared.append((device, path, verified, mapping))
    if not prepared:
        detail = "; ".join(item["error"] for item in incomplete)
        raise ValueError(f"no verified downloaded local recordings to export: {detail}")
    is_ordinary = manifest.get("caster_type") == "ordinary"
    csv_fields = ((*protocol.ordinary_protocol().CSV_FIELDS, "mapped_epoch_ns", "mapped_clock_sync_valid",
                   "origin", "run_id", "timing_estimated", "timing_uncertainty_ns", "diagnostics_known")
                  if is_ordinary else CSV_FIELDS)
    names = ("packets.jsonl", "metadata.jsonl", "samples.csv")
    temporary_paths = [root / (name + ".tmp") for name in names]
    try:
        with temporary_paths[0].open("w", encoding="utf-8", newline="\n") as packets, \
                temporary_paths[1].open("w", encoding="utf-8", newline="\n") as metadata_output, \
                temporary_paths[2].open("w", encoding="utf-8", newline="") as csv_output:
            writer = csv.DictWriter(csv_output, fieldnames=csv_fields)
            writer.writeheader()
            for device, path, verified, mapping in prepared:
                metadata = verified["metadata"]["packet"]
                identity = {name: metadata[name] for name in ("v", "wheel_id", "device_id", "node_id", "boot_id")}
                if is_ordinary:
                    identity["caster_type"] = "ordinary"
                raw_diagnostics = (device.get("status") or {}).get("diagnostics", {})
                known = (device.get("status") or {}).get("diagnostics_known", True) is True and all(
                    type(raw_diagnostics.get(name)) is int and raw_diagnostics[name] >= 0
                    for name in protocol.DIAGNOSTIC_FIELDS)
                diagnostics = {name: raw_diagnostics[name] if known else 0 for name in protocol.DIAGNOSTIC_FIELDS}
                base = {"origin": LOCAL_ORIGIN, "recording": {"run_id": run_id, "file": str(path.relative_to(root))},
                        **{name: None for name in protocol.CSV_FIELDS[:4]},
                        "clock_mapping": mapping, "timing_estimated": mapping is not None,
                        "diagnostics_known": known, "diagnostics_basis": "final_boot_counters" if known else "unavailable"}
                envelope = {**base, "packet": metadata}
                validate_local_envelope(envelope)
                packets.write(_json(envelope) + "\n")
                metadata_output.write(_json(envelope) + "\n")
                mapper = protocol.ClockMapper({"version": 1, "mappings": [mapping]} if mapping else None)
                batch = []
                for sample in iter_samples(path):
                    batch.append(sample)
                    rows = (protocol.ordinary_protocol().channel_rows({**metadata, "diagnostics": diagnostics}, sample)
                            if is_ordinary else [{
                                **{name: metadata[name] for name in ("wheel_id", "device_id", "node_id", "boot_id")},
                                **sample, **{name: int(bool(sample["flags"] & (1 << bit)))
                                            for bit, name in enumerate(protocol.FLAG_NAMES)}}])
                    for row in rows:
                        mapped = mapper.map(metadata["node_id"], metadata["boot_id"], row["t_us"],
                                            metadata["wheel_id"], metadata["device_id"])
                        row.update({**{name: None for name in protocol.CSV_FIELDS[:4]},
                                    **{name: diagnostics[name] if known else None for name in protocol.DIAGNOSTIC_FIELDS},
                                    "sequence_status": "in_order", "mapped_epoch_ns": mapped,
                                    "mapped_clock_sync_valid": 0, "timing_estimated": int(mapped is not None),
                                    "timing_uncertainty_ns": mapping["uncertainty_ns"] if mapping else None,
                                    "origin": LOCAL_ORIGIN, "run_id": run_id, "diagnostics_known": int(known)})
                        writer.writerow(row)
                    if len(batch) == 32:
                        packets.write(_json({**base, "packet": {**identity, "type": "samples", "samples": batch,
                                                              "diagnostics": diagnostics}}) + "\n")
                        batch = []
                if batch:
                    packets.write(_json({**base, "packet": {**identity, "type": "samples", "samples": batch,
                                                          "diagnostics": diagnostics}}) + "\n")
            for handle in (packets, metadata_output, csv_output):
                handle.flush()
                os.fsync(handle.fileno())
        for temporary, name in zip(temporary_paths, names):
            temporary.replace(root / name)
    finally:
        for path in temporary_paths:
            if path.exists():
                path.unlink()
    _atomic_json(root / "clock-map.json", {"version": 1, "mappings": mappings,
                                          "accuracy_validated": False, "clock_errors": clock_errors})
    summary = {"version": 1, "run_id": run_id, "files": [*names, "clock-map.json"],
               "nodes": [{key: value for key, value in item[2].items() if key != "metadata"} for item in prepared],
               "clock_errors": clock_errors, "incomplete_devices": incomplete,
               "timing_note": "Mapped timestamps are two-way midpoint estimates; firmware clock flags remain unchanged. Unknown diagnostics remain blank in CSV and are explicitly marked unavailable in packet envelopes."}
    _atomic_json(root / "export-summary.json", summary)
    return summary
