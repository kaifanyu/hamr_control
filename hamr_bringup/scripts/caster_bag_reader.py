"""One offline reader for recording folders, split bags and standalone MCAPs."""
from collections import Counter, defaultdict
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
import struct

import yaml

from caster_encoder_protocol import ROLES, packet_identity, parse_json
from caster_packet_analysis import validate_envelope

WHEELS = ("front_left", "front_right")
SUBJECTS = ("HAMR_base", "HAMR_turret", "Left_Caster", "Right_Caster")
REFERENCE_FIELDS = "x y roll pitch yaw x_dot y_dot roll_dot pitch_dot yaw_dot".split()
STATUS_FIELDS = [f"{wheel}_{name}" for wheel in ("left", "right") for name in
                 ("target_rpm", "measured_rpm", "ff_pwm", "pid_pwm", "output_pwm",
                  "error_rpm", "integral_rpm_s", "saturated")] + ["source"]


def digest(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            result.update(block)
    return result.hexdigest()


@dataclass
class Recording:
    source: Path
    root: Path
    files: list = field(default_factory=list)
    metadata: dict = field(default_factory=dict)
    types: dict = field(default_factory=dict)
    times: dict = field(default_factory=lambda: defaultdict(list))
    poses: dict = field(default_factory=lambda: defaultdict(list))
    numeric: dict = field(default_factory=dict)
    packets: dict = field(default_factory=lambda: defaultdict(list))
    paths: dict = field(default_factory=dict)
    frames: dict = field(default_factory=lambda: defaultdict(set))
    issues: list = field(default_factory=list)
    sidecars: dict = field(default_factory=dict)
    start_ns: int = 0
    end_ns: int = 0
    counts_verified: bool = False

    def add_numeric(self, topic, stamp, values):
        series = self.numeric.setdefault(topic, [])
        series.append({"stamp_ns": stamp, **values})

    def encoder_topics(self):
        selected = {}
        for wheel in WHEELS:
            for role in ROLES.values():
                for prefix in ("caster_recording", "caster_encoder"):
                    topic = f"/{prefix}/{wheel}/{role}/joint_states"
                    if self.numeric.get(topic):
                        selected[wheel, role] = topic
                        break
        return selected

    def packet_streams(self):
        selected = {}
        for wheel in (*WHEELS, "legacy"):
            candidates = ([f"/caster_recording/{wheel}/packets", f"/caster_encoder/{wheel}/packets"]
                          if wheel != "legacy" else ["/caster_encoder/packets"])
            for topic in candidates:
                if topic in self.packets:
                    selected[wheel] = self.packets[topic] + self.packets.get(topic.rsplit("/", 1)[0]+"/metadata", [])
                    break
        return {wheel: sorted(rows, key=lambda row: row[1]) for wheel, rows in selected.items()}


def resolve_source(source):
    source = Path(source).expanduser().resolve()
    if not source.exists():
        raise ValueError(f"Input does not exist: {source}")
    root = source if source.is_dir() else source.parent
    bag = source
    if source.is_dir() and (source / "bags/combined").is_dir():
        bag = source / "bags/combined"
    elif source.is_dir() and (source / "bag/metadata.yaml").is_file():
        bag = source / "bag"
    elif source.is_file() and source.name == "packets.jsonl":
        return Recording(source, root)
    directory = bag if bag.is_dir() else bag.parent
    if directory.name == "combined" and directory.parent.name == "bags":
        root = directory.parent.parent
    metadata = {}
    metadata_path = directory / "metadata.yaml"
    if metadata_path.is_file():
        metadata = yaml.safe_load(metadata_path.read_text())["rosbag2_bagfile_information"]
        files = [(directory / name).resolve() for name in metadata["relative_file_paths"]]
        if not files or any(directory not in path.parents or not path.is_file()
                            or path.suffix not in (".mcap", ".db3") for path in files):
            raise ValueError("Bag metadata must identify complete local MCAP/SQLite shards")
        # A file inside a split bag always selects the entire metadata-described
        # recording, keeping global counts and time coverage honest.
        if bag.is_file() and bag not in files:
            raise ValueError("Selected file is not listed in its bag metadata")
    elif bag.is_file() and bag.suffix in (".mcap", ".db3"):
        files = [bag]
    elif bag.is_dir():
        files = sorted([*bag.glob("*.mcap"), *bag.glob("*.db3")])
        if len(files) > 1:
            raise ValueError("Multiple storage files require metadata.yaml to identify one recording")
        if not files and not (root / "packets.jsonl").is_file():
            raise ValueError(f"No MCAP/DB3 bag or packets.jsonl found in {source}")
    else:
        raise ValueError("Input must be a recording folder, rosbag directory, .mcap, .db3 or packets.jsonl")
    recording = Recording(source, root, files, metadata)
    for name in ("export-summary.json", "clock-map.json"):
        path = root / name
        if path.is_file():
            recording.sidecars[name] = json.loads(path.read_text())
    return recording


def stamp_ns(header):
    return int(header.stamp.sec)*1_000_000_000 + int(header.stamp.nanosec)


def vector(value, prefix, axes="xyz"):
    return {f"{prefix}_{axis}": getattr(value, axis) for axis in axes}


def retain_message(recording, topic, kind, message, stamp):
    """Decode typed numeric channels; retain large sensor payloads only in source."""
    values = {}
    if kind in ("geometry_msgs/msg/PoseStamped", "geometry_msgs/msg/PoseWithCovarianceStamped", "nav_msgs/msg/Odometry"):
        pose = message.pose if kind == "geometry_msgs/msg/PoseStamped" else message.pose.pose
        values = {**vector(pose.position, "position_m"), **vector(pose.orientation, "quaternion", "xyzw")}
        recording.poses[topic].append(dict(stamp_ns=stamp, header_stamp_ns=stamp_ns(message.header),
                                         frame_id=message.header.frame_id,
                                         **{axis: getattr(pose.position, axis) for axis in "xyz"},
                                         **{f"q{axis}": getattr(pose.orientation, axis) for axis in "xyzw"}))
        if kind == "nav_msgs/msg/Odometry":
            values.update(vector(message.twist.twist.linear, "linear_m_s"))
            values.update(vector(message.twist.twist.angular, "angular_rad_s"))
    elif kind == "sensor_msgs/msg/JointState":
        if len(message.position) == 1:
            values = dict(angle_rad=message.position[0], velocity_rad_s=message.velocity[0] if message.velocity else None)
        else:
            values = {f"position_{name}": v for name, v in zip(message.name, message.position)}
            values.update({f"velocity_{name}": v for name, v in zip(message.name, message.velocity)})
    elif kind == "sensor_msgs/msg/Imu":
        values = {**vector(message.orientation, "quaternion", "xyzw"),
                  **vector(message.angular_velocity, "gyro_rad_s"),
                  **vector(message.linear_acceleration, "acceleration_m_s2")}
    elif kind == "sensor_msgs/msg/MagneticField":
        values = vector(message.magnetic_field, "field_tesla")
    elif kind == "nav_msgs/msg/Path":
        recording.paths[topic] = dict(frame_id=message.header.frame_id,
                                      points=[[p.pose.position.x, p.pose.position.y] for p in message.poses])
    elif kind == "std_msgs/msg/String":
        if topic.endswith(("/packets", "/metadata")) and topic.startswith(("/caster_recording/", "/caster_encoder/")):
            envelope = parse_json(message.data)
            if isinstance(envelope, dict) and isinstance(envelope.get("nodes"), dict):
                from types import SimpleNamespace
                for packet in envelope["nodes"].values():
                    retained = dict(packet=packet, metadata_source_topic=topic, snapshot_bag_timestamp_ns=stamp,
                                    receive_epoch_ns=None, receive_monotonic_ns=None, source_ip=None, source_port=None)
                    retain_message(recording, topic, kind, SimpleNamespace(data=json.dumps(retained)), stamp)
                return
            if "packet" not in envelope:  # Live bridge retained metadata snapshot.
                from caster_encoder_protocol import decode_packet
                packet = decode_packet(json.dumps(envelope).encode())
                envelope = dict(packet=packet, metadata_source_topic=topic, snapshot_bag_timestamp_ns=stamp,
                                receive_epoch_ns=None, receive_monotonic_ns=None, source_ip=None, source_port=None)
            validate_envelope(envelope)
            wheel = packet_identity(envelope["packet"])[0]
            expected = topic.split("/")[2]
            if expected in WHEELS and wheel != expected:
                raise ValueError(f"Packet wheel {wheel} conflicts with topic {topic}")
            recording.packets[topic].append((envelope, stamp))
    elif hasattr(message, "data") and isinstance(message.data, (float, int, bool)):
        values = {"value": message.data}
    elif kind.endswith("MultiArray"):
        if len(message.data) > 64:
            return
        fields = [f"channel_{i}" for i in range(len(message.data))]
        if topic == "/wheel_control/status" and len(message.data) == 17:
            if not message.layout.dim or message.layout.dim[0].label != "wheel_control_status_v1":
                raise ValueError("Unsupported wheel status schema")
            fields = STATUS_FIELDS
        elif topic == "/imu/calib_status" and len(message.data) == 4:
            fields = ["system", "gyro", "accelerometer", "magnetometer"]
        values = dict(zip(fields, message.data))
    elif kind.startswith("hamr_interfaces/msg/"):
        values = {name: getattr(message, name) for name in message.get_fields_and_field_types()
                  if isinstance(getattr(message, name), (int, float, bool))}
    if hasattr(message, "header"):
        recording.frames[topic].add(message.header.frame_id)
        if values:
            values["header_stamp_ns"] = stamp_ns(message.header)
    if values:
        recording.add_numeric(topic, stamp, values)


def read_recording(source, clock_mapper=None):
    recording = resolve_source(source)
    sample_bounds = []
    if not recording.files:
        from caster_packet_analysis import read_jsonl
        from caster_encoder_protocol import ClockMapper
        path = recording.root / "packets.jsonl"
        for envelope, _ in read_jsonl(path):
            wheel = packet_identity(envelope["packet"])[0]
            packet = envelope["packet"]
            mapping = envelope.get("clock_mapping")
            samples = packet.get("samples", [])
            mapped = None
            mapper = None
            if mapping and samples:
                mapper = ClockMapper({"version": 1, "mappings": [mapping]})
            elif clock_mapper and samples:
                mapper = clock_mapper
            if mapper:
                mapped_stamps = [mapper.map(packet["node_id"], packet["boot_id"], sample["t_us"],
                                            wheel, packet.get("device_id")) for sample in samples]
                mapped = mapped_stamps[0]
                sample_bounds.extend(stamp for stamp in mapped_stamps if stamp is not None)
            stamp = mapped or envelope.get("receive_epoch_ns") or 0
            topic = f"/caster_recording/{wheel}/packets" if wheel in WHEELS else "/caster_encoder/packets"
            recording.packets[topic].append((envelope, stamp))
            recording.types[topic] = "std_msgs/msg/String"
            recording.times[topic].append(stamp)
        if not recording.packets:
            raise ValueError(f"No encoder packets in {path}")
    else:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        classes = {}
        decode_counts = Counter()
        for path in recording.files:
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap" if path.suffix == ".mcap" else "sqlite3"),
                        rosbag2_py.ConverterOptions("", ""))
            for item in reader.get_all_topics_and_types():
                if item.name in recording.types and recording.types[item.name] != item.type:
                    raise ValueError(f"Topic type changes across shards: {item.name}")
                recording.types[item.name] = item.type
            while reader.has_next():
                topic, raw, stamp = reader.read_next()
                recording.times[topic].append(stamp)
                kind = recording.types[topic]
                supported = (kind.startswith(("std_msgs/msg/", "hamr_interfaces/msg/")) or kind in (
                    "geometry_msgs/msg/PoseStamped", "geometry_msgs/msg/PoseWithCovarianceStamped",
                    "nav_msgs/msg/Odometry", "nav_msgs/msg/Path", "sensor_msgs/msg/JointState",
                    "sensor_msgs/msg/Imu", "sensor_msgs/msg/MagneticField"))
                if not supported:
                    continue
                try:
                    if kind in ("hamr_interfaces/msg/ReferenceTraj", "hamr_interfaces/msg/LiveGains", "hamr_interfaces/msg/StateError"):
                        fields = {"hamr_interfaces/msg/ReferenceTraj": REFERENCE_FIELDS,
                                  "hamr_interfaces/msg/LiveGains": "p_x d_x i_x p_y d_y i_y p_yaw d_yaw i_yaw".split(),
                                  "hamr_interfaces/msg/StateError": "err_x err_y err_yaw".split()}[kind]
                        if len(raw) != 4+8*len(fields) or raw[:2] not in (b"\x00\x01", b"\x00\x00"):
                            raise ValueError("Unsupported HAMR scalar CDR schema")
                        recording.add_numeric(topic, stamp, dict(zip(fields, struct.unpack(
                            ("<" if raw[1] == 1 else ">") + f"{len(fields)}d", raw[4:]))))
                    else:
                        if kind not in classes:
                            classes[kind] = get_message(kind)
                        retain_message(recording, topic, kind, deserialize_message(raw, classes[kind]), stamp)
                except (ValueError, TypeError, KeyError, AttributeError, ImportError, RuntimeError) as error:
                    decode_counts[topic] += 1
                    if decode_counts[topic] <= 3:
                        recording.issues.append(f"{topic} at {stamp}: {error}")
        for topic, count in decode_counts.items():
            recording.issues.append(f"{topic}: {count} messages could not be decoded; inspect source bag")
        if recording.metadata:
            expected = {item["topic_metadata"]["name"]: item["message_count"]
                        for item in recording.metadata["topics_with_message_count"]}
            actual = {topic: len(times) for topic, times in recording.times.items()}
            if ({k: v for k, v in expected.items() if v} != {k: v for k, v in actual.items() if v}
                    or sum(actual.values()) != recording.metadata["message_count"]):
                raise ValueError("Recorded message counts do not match metadata; incomplete or mixed bag")
            recording.counts_verified = True
    stamps = sample_bounds + [stamp for times in recording.times.values() for stamp in times if stamp > 0]
    if not stamps:
        if recording.packets:
            recording.issues.append("No global timestamps: only device-local packet quality/angle plots are available; supply --clock-map or saved clock probes")
            return recording
        raise ValueError("Recording has no usable timestamps")
    recording.start_ns, recording.end_ns = min(stamps), max(stamps)
    for rows in (*recording.poses.values(), *recording.numeric.values()):
        rows.sort(key=lambda row: row["stamp_ns"])
    return recording


def replay_events(recording, recover_timing=False, wheels=WHEELS):
    """Retain acquisition time for onboard logs and receipt time for live logs."""
    streams = recording.packet_streams()
    if not all(wheel in streams for wheel in wheels):
        raise ValueError(f"CAD replay needs packet history for {', '.join(wheels)}; joint-state-only bags still produce plots")
    envelopes = [e for wheel in wheels for e, _ in streams[wheel]]
    local = [e.get("origin") == "encoder_local_recording" for e in envelopes]
    if any(local) and not all(local):
        raise ValueError("Cannot mix local acquisition and live receipt clocks in one CAD replay")
    if all(e.get("origin") == "encoder_local_recording" for e in envelopes):
        if len(wheels) == 1:
            from caster_local_recording import load_recording
            events, provenance = load_recording(recording.root, wheels[0], packet_rows=envelopes)
            return {wheels[0]: events}, provenance
        from caster_pair_recording import load_pair_recording
        return load_pair_recording(recording.root, recover_timing, packet_rows=envelopes)
    from caster_replay_model import ReplayModel
    events, counts = {}, {}
    for wheel in wheels:
        model = ReplayModel(wheel_id=wheel)
        events[wheel], counts[wheel] = [], Counter()
        for envelope, stamp in streams[wheel]:
            event = dict(envelope, receive_epoch_ns=stamp,
                         receive_monotonic_ns=stamp-recording.start_ns,
                         source_ip="offline-bag-replay", source_port=0)
            counts[wheel].update(ROLES[event["packet"]["node_id"]] for _ in event["packet"].get("samples", []))
            model.receive(event, stamp)
            events[wheel].append((stamp, event))
        if not model.snapshot(recording.end_ns)[2]["complete_pose_available"]:
            raise ValueError(f"No complete valid H1/H2/CENTER pose for {wheel}")
    start = min(stamp for rows in events.values() for stamp, _ in rows)
    end = max(stamp for rows in events.values() for stamp, _ in rows)
    return events, dict(start_ns=start, end_ns=end, wheel_ids=list(wheels), accuracy_validated=False,
                        sample_counts={wheel: dict(value) for wheel, value in counts.items()},
                        time_basis="Recorded bag receipt time; batched samples share receipt time. Acquisition clocks are not aligned.",
                        recovered_clock_mappings=[])
