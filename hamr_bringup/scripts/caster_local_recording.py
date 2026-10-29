#!/usr/bin/env python3
"""Convert a downloaded local caster recording and render its measured motion.

The Bash entry point sources ROS. This helper keeps conversion testable without
VTK, ffmpeg, a running robot, or a ROS graph. Source exports are never modified.
"""

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib
import json
import math
from pathlib import Path
import shutil

from caster_encoder_protocol import ClockMapper, INT64_MAX, ROLES, integer, packet_identity, parse_json
from caster_encoder_selection import WheelIdentityGuard, topic_for_wheel
from caster_recording_format import validate_local_envelope
from caster_replay_model import ReplayModel


PROVENANCE_FILE = "local-recording-replay.json"
TIME_BASIS = (
    "Estimated acquisition time from the exported per-device clock maps; "
    "synthetic replay timestamps, not live UDP receipt times. "
    "Synchronization accuracy has not been independently validated."
)


def load_recording(directory, wheel=None, packet_rows=None):
    """Unbatch and interleave samples without losing integer timestamp precision."""
    directory = Path(directory).expanduser().resolve()
    source = directory / "packets.jsonl"
    if packet_rows is None and not source.is_file():
        raise ValueError(f"Missing {source}; pass the downloaded encoder_<run> folder")
    payload = source.read_bytes() if packet_rows is None else None
    envelopes = []
    for line_number, line in enumerate(payload.splitlines() if payload is not None else packet_rows, 1):
        if isinstance(line, bytes) and not line.strip():
            continue
        try:
            envelope = parse_json(line) if isinstance(line, bytes) else line
            validate_local_envelope(envelope)
        except (ValueError, KeyError, TypeError) as error:
            raise ValueError(f"{source}:{line_number}: invalid packet envelope: {error}") from error
        envelopes.append(envelope)
    wheels = {packet_identity(e["packet"])[0] for e in envelopes}
    if wheel is None:
        if len(wheels) != 1:
            raise ValueError("Select --wheel front_left or front_right; recording must identify one wheel")
        wheel = next(iter(wheels))
    if wheel not in wheels:
        raise ValueError(f"Recording contains no packets for {wheel}")

    guard = WheelIdentityGuard(wheel)
    events, metadata, first_sample, mappings, mappers, previous = [], {}, {}, {}, {}, {}
    counts, runs = Counter(), set()
    for envelope in envelopes:
        packet = envelope["packet"]
        if packet_identity(packet)[0] != wheel:
            continue
        key = guard.accept(packet)
        runs.add(envelope["recording"]["run_id"])
        mapping = envelope.get("clock_mapping")
        if key in mappings and mappings[key] != mapping:
            raise ValueError(f"Clock mapping changed within recording for {key}")
        mappings[key] = mapping
        if packet["type"] == "metadata":
            if key in metadata and metadata[key] != envelope:
                raise ValueError(f"Conflicting metadata for {key}")
            metadata[key] = envelope
            continue
        if packet["type"] != "samples":
            raise ValueError(f"Unexpected local packet type: {packet['type']}")
        if mapping is None:
            raise ValueError(f"Missing clock estimate for {key}; cannot align the encoder clocks for replay")
        if key not in mappers:
            mappers[key] = ClockMapper({"version": 1, "mappings": [mapping]})
        for sample in packet["samples"]:
            last = previous.get(key)
            if last and (sample["seq"] <= last[0] or sample["t_us"] <= last[1]):
                raise ValueError(f"Duplicate or out-of-order sample for {key}")
            previous[key] = sample["seq"], sample["t_us"]
            stamp = mappers[key].map(packet["node_id"], packet["boot_id"], sample["t_us"],
                                     packet["wheel_id"], packet["device_id"])
            if stamp is None:
                raise ValueError(f"Sample {sample['seq']} is outside the clock estimate for {key}")
            integer(stamp, "mapped acquisition timestamp", 1, INT64_MAX)
            first_sample.setdefault(key, stamp)
            counts[ROLES[packet["node_id"]]] += 1
            events.append((stamp, {**envelope, "packet": {**packet, "samples": [sample]}}))
    if len(runs) != 1:
        raise ValueError("Selected packets must belong to a single recording run")
    if set(counts) != set(ROLES.values()):
        raise ValueError("Replay needs samples from H1, H2, and CENTER on the selected wheel")
    for key, stamp in first_sample.items():
        if key not in metadata:
            raise ValueError(f"Missing metadata for {key}")
        events.append((stamp - 1, metadata[key]))
    events.sort(key=lambda item: item[0])
    start = events[0][0]
    model = ReplayModel(wheel_id=wheel)
    adapted = []
    for stamp, envelope in events:
        # Original exports retain their null receive fields. Only this clearly
        # identified derivative supplies the legacy renderer's required fields.
        replay = {**envelope, "origin": "encoder_local_recording_replay_adapter",
                  "replay_time_basis": TIME_BASIS,
                  "receive_epoch_ns": stamp, "receive_monotonic_ns": stamp - start,
                  "source_ip": "offline-local-recording", "source_port": 0}
        model.receive(replay, stamp)
        adapted.append((stamp, replay))
    if not model.snapshot(events[-1][0])[2]["complete_pose_available"]:
        raise ValueError("Recording has no complete valid H1/H2/CENTER pose")
    provenance = {
        "version": 1, "source_directory": str(directory), "run_id": next(iter(runs)),
        "source_packets_sha256": hashlib.sha256(payload).hexdigest() if payload is not None else None,
        "wheel_id": wheel, "time_basis": TIME_BASIS,
        "timing_estimated": True, "accuracy_validated": False,
        "synthetic_envelope_fields": ["receive_epoch_ns", "receive_monotonic_ns", "source_ip", "source_port"],
        "original_receive_fields": None, "sample_counts": dict(counts),
        "clock_mappings": list(mappings.values()),
        "start_ns": start, "end_ns": events[-1][0],
    }
    return adapted, provenance


def write_replay_bag(destination, events, provenance):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String

    destination = Path(destination)
    if destination.exists():
        raise ValueError(f"Refusing to overwrite {destination}; choose another output directory")
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(destination), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    try:
        topic = topic_for_wheel(provenance["wheel_id"])
        fields = dict(name=topic, type="std_msgs/msg/String", serialization_format="cdr")
        try:
            metadata = rosbag2_py.TopicMetadata(id=0, **fields)
        except TypeError:
            # Humble assigns topic IDs internally; Jazzy accepts an explicit ID.
            metadata = rosbag2_py.TopicMetadata(**fields)
        writer.create_topic(metadata)
        for stamp, envelope in events:
            message = String(data=json.dumps(envelope, allow_nan=False, separators=(",", ":")))
            writer.write(topic, serialize_message(message), stamp)
    finally:
        writer.close()
    (destination / PROVENANCE_FILE).write_text(json.dumps(provenance, indent=2) + "\n")


def check_dependencies(plots, video=True):
    missing = []
    for module in ("rosbag2_py", "rclpy", "std_msgs.msg", "numpy", "PIL", "vtk", "yaml") + (
            ("matplotlib",) if plots else ()):
        try:
            importlib.import_module(module)
        except ImportError:
            missing.append(module)
    if video and not shutil.which("ffmpeg"):
        missing.append("ffmpeg")
    if missing:
        raise RuntimeError(
            "Missing rendering dependencies: " + ", ".join(missing) +
            ". See docs/LOCAL_CASTER_RECORDING.md (Render a recording on Ubuntu).")
