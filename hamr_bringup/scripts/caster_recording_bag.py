"""Merge verified onboard logs with a finalized robot bag, without live replay."""

import heapq
import json
import os
from pathlib import Path
import tempfile

import yaml

import caster_encoder_protocol as protocol
import caster_recording_format as binary
from caster_recording_client import RecordingError, fsync_directory, write_json


PREFIX = "/caster_recording"


def bag_counts(path):
    metadata = yaml.safe_load((Path(path) / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    return {entry["topic_metadata"]["name"]: entry["message_count"]
            for entry in metadata["topics_with_message_count"]}


def merge_session(session, state):
    """Keep source bytes/timestamps through rosbag2's native multi-bag rewrite.

    All checks and output are offline. Failed attempts remain in their own
    temporary directory; originals are never overwritten or deleted.
    """
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    robot = Path(session.manifest["bag_path"])
    destination = session.directory / "bags"
    if destination.exists():
        report = json.loads((destination / "merge-result.json").read_text())
        if (report["run_id"] != session.manifest["run_id"] or
                report["robot_bag"] != str(robot) or
                bag_counts(destination / "combined") != report["message_counts"]):
            raise RecordingError("Existing merged bag does not match this session; preserved without overwrite")
        return destination / "combined"
    source_counts = bag_counts(robot)
    if any(name == PREFIX or name.startswith(PREFIX + "/") for name in source_counts):
        raise RecordingError(f"Robot bag already contains {PREFIX} topics; refusing ambiguous merge")
    if not all(item.get("verified") and item.get("fetch_validated") and item.get("acknowledged")
               for item in session.manifest["devices"]):
        raise RecordingError("Not every selected log has been downloaded, verified, and acknowledged; rerun stop to recover")

    # Regenerate from verified binaries and recorded clock probes, never from
    # download arrival times or live receiver state.
    exports = binary.export_run(session.directory, session.manifest)
    if exports["clock_errors"] or exports["incomplete_devices"]:
        raise RecordingError("Cannot align every selected board to Pi time. Raw logs are saved; see "
                             f"{session.directory / 'export-summary.json'}")
    envelopes = [json.loads(line) for line in (session.directory / "metadata.jsonl").read_text().splitlines()]
    prepared = []
    for envelope in envelopes:
        binary.validate_local_envelope(envelope)
        packet = envelope["packet"]
        mapping = envelope["clock_mapping"]
        if mapping is None:
            raise RecordingError(f"No timed samples for {packet['wheel_id']}/{protocol.ROLES[packet['node_id']]}")
        mapper = protocol.ClockMapper({"version": 1, "mappings": [mapping]})
        prepared.append((envelope, mapper))

    def encoded(value):
        return serialize_message(String(data=json.dumps(value, allow_nan=False, separators=(",", ":"))))

    def device_messages(envelope, mapper):
        packet = envelope["packet"]
        wheel, node = packet["wheel_id"], packet["node_id"]
        base = f"{PREFIX}/{wheel}"
        diagnostics = next(item["status"].get("diagnostics", {}) for item in session.manifest["devices"]
                           if item["device_id"] == packet["device_id"])
        diagnostics = {key: diagnostics.get(key, 0) for key in protocol.DIAGNOSTIC_FIELDS}
        first = True
        for sample in binary.iter_samples(session.directory / envelope["recording"]["file"]):
            stamp = mapper.map(node, packet["boot_id"], sample["t_us"], wheel, packet["device_id"])
            if stamp is None or not 0 < stamp < 2**63:
                raise RecordingError("Sample lies outside the verified clock mapping")
            if first:
                yield stamp, base + "/metadata", encoded(envelope)
                first = False
            # Keep invalid samples and original flags/64-bit counts in packets.
            # Only valid measurements become displayable JointState messages.
            payload = {key: packet[key] for key in ("v", "wheel_id", "device_id", "node_id", "boot_id")}
            payload.update(type="samples", samples=[sample], diagnostics=diagnostics)
            yield stamp, base + "/packets", encoded({**envelope, "packet": payload})
            reading = protocol.display_reading(node, sample, wheel, packet["device_id"])
            if reading is not None:
                joint = JointState()
                joint.header.stamp.sec, joint.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                joint.name, joint.position = [reading["name"]], [reading["position"]]
                if reading["velocity"] is not None:
                    joint.velocity = [reading["velocity"]]
                yield stamp, base + f"/{protocol.ROLES[node]}/joint_states", serialize_message(joint)

    stage = Path(tempfile.mkdtemp(prefix=".merge-", dir=session.directory))
    encoder = stage / "encoders"
    combined = stage / "combined"
    writer = rosbag2_py.SequentialWriter()
    created = set()
    encoder_counts = {}
    try:
        writer.open(rosbag2_py.StorageOptions(uri=str(encoder), storage_id="mcap"),
                    rosbag2_py.ConverterOptions("", ""))
        def write(topic, data, stamp):
            if topic not in created:
                message_type = "sensor_msgs/msg/JointState" if topic.endswith("/joint_states") else "std_msgs/msg/String"
                writer.create_topic(rosbag2_py.TopicMetadata(
                    id=0, name=topic, type=message_type, serialization_format="cdr"))
                created.add(topic)
            writer.write(topic, data, stamp, stamp)
            encoder_counts[topic] = encoder_counts.get(topic, 0) + 1
        provenance = {"version": 1, "run_id": session.manifest["run_id"], "robot_bag": str(robot),
                      "timing_estimated": True, "accuracy_validated": False,
                      "timing_note": "ESP acquisition times mapped by two-way clock probes; no download timestamps substituted. Robot messages retain original timestamps.",
                      "session": state, "encoder_manifest": session.manifest,
                      "clock_maps": [envelope["clock_mapping"] for envelope, _ in prepared]}
        streams = [device_messages(*item) for item in prepared]
        first = True
        for stamp, topic, data in heapq.merge(*streams, key=lambda item: item[0]):
            if first:
                write(PREFIX + "/session", encoded(provenance), stamp)
                first = False
            write(topic, data, stamp)
    finally:
        writer.close()
    # The native merge retains serialized message contents and both rosbag
    # receive/send timestamps, including messages with custom ROS interfaces.
    options = stage / "merge.yaml"
    options.write_text(yaml.safe_dump({"output_bags": [{"uri": str(combined), "storage_id": "mcap",
                                                       "all_topics": True, "all_services": True,
                                                       "include_hidden_topics": True}]}))
    rosbag2_py.bag_rewrite([rosbag2_py.StorageOptions(uri=str(robot)),
                           rosbag2_py.StorageOptions(uri=str(encoder))], str(options))
    expected = {**source_counts, **encoder_counts}
    actual = bag_counts(combined)
    # Some storage versions omit registered topics with zero messages.
    if {k: v for k, v in actual.items() if v} != {k: v for k, v in expected.items() if v}:
        raise RecordingError(f"Merged message counts differ from inputs; source files preserved, inspect {stage}")
    report = {"version": 1, "run_id": session.manifest["run_id"], "robot_bag": str(robot),
              "message_counts": actual, "robot_message_count": sum(source_counts.values()),
              "encoder_message_count": sum(encoder_counts.values()), "timing_estimated": True}
    write_json(stage / "merge-result.json", report)
    # Flush the completed artifact before publishing its final name.
    for path in stage.rglob("*"):
        if path.is_file():
            with path.open("rb") as handle:
                os.fsync(handle.fileno())
    for directory in sorted((path for path in stage.rglob("*") if path.is_dir()),
                            key=lambda path: len(path.parts), reverse=True):
        fsync_directory(directory)
    fsync_directory(stage)
    os.rename(stage, destination)
    fsync_directory(session.directory)
    return destination / "combined"
