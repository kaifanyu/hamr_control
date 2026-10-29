"""Load both recorded ball casters onto one acquisition-time replay clock.

This is an offline adapter: input files, firmware validity flags, and accepted
clock estimates are never changed. A caller may explicitly opt into recovering
missing estimates from the recording's original clock exchanges at a relaxed
drift gate, for approximate visualization only.
"""

from collections import Counter
import hashlib
import json
from pathlib import Path

from caster_encoder_protocol import ClockMapper, INT64_MAX, ROLES, WHEELS, integer, packet_identity, parse_json
from caster_encoder_selection import WheelIdentityGuard, topic_for_wheel
from caster_recording_format import fit_clock_map, validate_local_envelope
from caster_replay_model import ReplayModel


PROVENANCE_FILE = "local-recording-replay.json"
RECOVERY_MAX_DRIFT_PPM = 5000
TIME_BASIS = (
    "One shared estimated acquisition clock from per-device two-way midpoint maps; "
    "synthetic replay timestamps, not live UDP receipt times. Synchronization "
    "accuracy has not been independently validated."
)


def _recover_maps(directory, run_id, missing, bounds):
    path = directory / "manifest.json"
    if not path.is_file():
        raise ValueError(f"Missing {path}; timing recovery requires original clock exchanges")
    payload = path.read_bytes()
    manifest = parse_json(payload)
    if not isinstance(manifest, dict) or manifest.get("run_id") != run_id:
        raise ValueError("Timing recovery manifest must belong to the same recording run")
    devices = manifest.get("devices")
    if not isinstance(devices, list):
        raise ValueError("Timing recovery manifest requires a devices list")
    recovered = {}
    for key in missing:
        matches = [device for device in devices if isinstance(device, dict)
                   and packet_identity(device) == key]
        if len(matches) != 1:
            raise ValueError(f"Timing recovery needs exactly one manifest identity for {key}")
        device = matches[0]
        try:
            mapping = fit_clock_map(
                device.get("sync_pre"), device.get("sync_post"), device,
                *bounds[key], max_drift_ppm=RECOVERY_MAX_DRIFT_PPM)
        except (ValueError, TypeError, KeyError) as error:
            raise ValueError(f"Cannot recover approximate timing for {key}: {error}") from error
        mapping.update(
            recovery="offline_visualization_relaxed_drift_gate",
            recovery_max_drift_ppm=RECOVERY_MAX_DRIFT_PPM,
            original_clock_mapping_available=False,
        )
        recovered[key] = mapping
    return recovered, hashlib.sha256(payload).hexdigest()


def load_pair_recording(directory, recover_timing=False, packet_rows=None):
    """Return ``({wheel: [(epoch_ns, envelope), ...]}, provenance)``.

    Keep the complete union of both recordings, including staggered beginnings
    and endings. Consumers must display unknown/held poses where a stream has
    not started or has ended; independent per-wheel zeroing would erase offsets.
    """
    directory = Path(directory).expanduser().resolve()
    source = directory / "packets.jsonl"
    if packet_rows is None and not source.is_file():
        raise ValueError(f"Missing {source}; pass the downloaded encoder_<run> folder")
    payload = source.read_bytes() if packet_rows is None else None
    inputs = payload.splitlines() if payload is not None else packet_rows
    guards = {wheel: WheelIdentityGuard(wheel) for wheel in WHEELS}
    counts = {wheel: Counter() for wheel in WHEELS}
    envelopes, mappings, metadata, bounds, previous, runs = [], {}, {}, {}, {}, set()
    for line_number, line in enumerate(inputs, 1):
        if isinstance(line, bytes) and not line.strip():
            continue
        try:
            envelope = parse_json(line) if isinstance(line, bytes) else line
            validate_local_envelope(envelope)
        except (ValueError, KeyError, TypeError) as error:
            raise ValueError(f"{source}:{line_number}: invalid packet envelope: {error}") from error
        packet = envelope["packet"]
        if packet.get("v") != 2 or packet.get("caster_type", "ball") != "ball":
            raise ValueError("Paired ball-caster replay requires identified v2 ball-caster packets")
        key = guards[packet["wheel_id"]].accept(packet)
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
        for sample in packet["samples"]:
            last = previous.get(key)
            if last and (sample["seq"] <= last[0] or sample["t_us"] <= last[1]):
                raise ValueError(f"Duplicate or out-of-order sample for {key}")
            previous[key] = sample["seq"], sample["t_us"]
            bounds.setdefault(key, [sample["t_us"], sample["t_us"]])[1] = sample["t_us"]
            counts[key[0]][ROLES[key[2]]] += 1
            envelopes.append((key, sample, envelope))
    if len(runs) != 1:
        raise ValueError("Selected packets must belong to a single recording run")
    run_id = next(iter(runs))
    for wheel in WHEELS:
        if set(counts[wheel]) != set(ROLES.values()):
            raise ValueError(f"Paired replay needs H1, H2, and CENTER samples on {wheel}")
    for key in bounds:
        if key not in metadata:
            raise ValueError(f"Missing metadata for {key}")
    missing = [key for key in bounds if mappings[key] is None]
    recovered, manifest_hash = {}, None
    if missing:
        if not recover_timing:
            raise ValueError(
                "Missing clock estimate for " + ", ".join(f"{key[0]} {ROLES[key[2]]}" for key in missing)
                + "; --recover-timing permits approximate visualization from original manifest probes")
        recovered, manifest_hash = _recover_maps(directory, run_id, missing, bounds)
        mappings.update(recovered)

    mappers = {key: ClockMapper({"version": 1, "mappings": [mappings[key]]}) for key in bounds}
    events = {wheel: [] for wheel in WHEELS}
    coverage = {}
    for key, sample, envelope in envelopes:
        wheel, device, node, boot = key
        stamp = mappers[key].map(node, boot, sample["t_us"], wheel, device)
        if stamp is None:
            raise ValueError(f"Sample {sample['seq']} is outside the clock estimate for {key}")
        integer(stamp, "mapped acquisition timestamp", 1, INT64_MAX)
        coverage.setdefault(key, [stamp, stamp])[1] = stamp
        events[wheel].append((stamp, {**envelope, "packet": {**envelope["packet"], "samples": [sample]}}))
    for key, (first, _) in coverage.items():
        events[key[0]].append((first - 1, metadata[key]))
    start = min(stamp for wheel_events in events.values() for stamp, _ in wheel_events)
    end = max(stamp for wheel_events in events.values() for stamp, _ in wheel_events)
    for wheel in WHEELS:
        events[wheel].sort(key=lambda event: event[0])
        adapted, model = [], ReplayModel(wheel_id=wheel)
        for stamp, envelope in events[wheel]:
            key = packet_identity(envelope["packet"])
            replay = {
                **envelope,
                "origin": "encoder_local_recording_replay_adapter",
                "clock_mapping": mappings[key], "timing_estimated": True,
                "replay_time_basis": TIME_BASIS,
                "receive_epoch_ns": stamp, "receive_monotonic_ns": stamp - start,
                "source_ip": "offline-local-recording", "source_port": 0,
            }
            if key in recovered:
                replay["original_clock_mapping"] = None
                replay["timing_recovered_for_visualization"] = True
            model.receive(replay, stamp)
            adapted.append((stamp, replay))
        if not model.snapshot(end)[2]["complete_pose_available"]:
            raise ValueError(f"Recording has no complete valid H1/H2/CENTER pose for {wheel}")
        events[wheel] = adapted

    errors_path = directory / "export-summary.json"
    errors = []
    if errors_path.is_file():
        errors = parse_json(errors_path.read_bytes()).get("clock_errors", [])
    provenance = {
        "version": 1, "source_directory": str(directory), "run_id": run_id,
        "source_packets_sha256": hashlib.sha256(payload).hexdigest() if payload is not None else None,
        "source_manifest_sha256": manifest_hash,
        "wheel_ids": list(WHEELS), "time_basis": TIME_BASIS, "timing_note": TIME_BASIS,
        "timing_estimated": True, "accuracy_validated": False,
        "synthetic_envelope_fields": ["receive_epoch_ns", "receive_monotonic_ns", "source_ip", "source_port"],
        "original_receive_fields": None,
        "sample_counts": {wheel: dict(counts[wheel]) for wheel in WHEELS},
        "clock_mappings": [mappings[key] for key in bounds],
        "recovered_clock_mappings": list(recovered.values()),
        "original_clock_errors": errors,
        "recovery_max_drift_ppm": RECOVERY_MAX_DRIFT_PPM if recovered else None,
        "start_ns": start, "end_ns": end,
        "streams": [
            {"wheel_id": key[0], "device_id": key[1], "role": ROLES[key[2]], "boot_id": key[3],
             "start_ns": first, "end_ns": last,
             "timing_recovered_for_visualization": key in recovered}
            for key, (first, last) in coverage.items()
        ],
    }
    return events, provenance


def write_pair_replay_bag(destination, events_by_wheel, provenance):
    """Write both namespaced topics in timestamp order, with timing provenance."""
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
        for index, wheel in enumerate(WHEELS):
            fields = dict(name=topic_for_wheel(wheel), type="std_msgs/msg/String", serialization_format="cdr")
            try:
                metadata = rosbag2_py.TopicMetadata(id=index, **fields)
            except TypeError:
                metadata = rosbag2_py.TopicMetadata(**fields)
            writer.create_topic(metadata)
        events = sorted(((stamp, wheel, envelope) for wheel in WHEELS
                         for stamp, envelope in events_by_wheel[wheel]), key=lambda event: event[:2])
        for stamp, wheel, envelope in events:
            message = String(data=json.dumps(envelope, allow_nan=False, separators=(",", ":")))
            writer.write(topic_for_wheel(wheel), serialize_message(message), stamp)
    finally:
        writer.close()
    (destination / PROVENANCE_FILE).write_text(json.dumps(provenance, indent=2) + "\n")
