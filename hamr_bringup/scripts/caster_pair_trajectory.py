"""Read a recorded HAMR pose on the encoder replay's existing epoch clock.

Robot bag timestamps are retained exactly. This adapter neither shifts another
run onto the encoder timeline nor assumes Vicon and encoder latency is calibrated.
"""

import math
from pathlib import Path


POSE_TYPES = (
    "nav_msgs/msg/Odometry", "geometry_msgs/msg/PoseStamped",
    "geometry_msgs/msg/PoseWithCovarianceStamped", "geometry_msgs/msg/TransformStamped",
)
PREFERRED_TOPICS = (
    "/HAMR_base/odom", "/HAMR_base/pose", "/vicon/odom", "/mocap/odom",
    "/local_HAMR/odom", "/wheel_odom", "/odometry/filtered", "/odom",
)
TIME_BASIS = (
    "Recorded robot bag epoch timestamps aligned to the existing estimated encoder "
    "epoch clock, without a per-run time shift. Sensor/transport latency and "
    "cross-system synchronization accuracy have not been independently validated."
)


def _bag_files(path):
    """Open storage shards directly, including Jazzy bags on Humble readers."""
    if path.is_file():
        if path.suffix not in (".mcap", ".db3"):
            raise ValueError("Trajectory bag file must end in .mcap or .db3")
        return [path]
    if not path.is_dir():
        raise ValueError(f"Trajectory bag does not exist: {path}")
    metadata = path / "metadata.yaml"
    if metadata.is_file():
        import yaml
        information = yaml.safe_load(metadata.read_text())["rosbag2_bagfile_information"]
        names = information.get("relative_file_paths", [])
        files = [path / name for name in names]
    else:
        files = sorted([*path.glob("*.mcap"), *path.glob("*.db3")])
    if not files or any(not item.is_file() or item.suffix not in (".mcap", ".db3") for item in files):
        raise ValueError(f"Trajectory bag has no complete MCAP/SQLite storage files: {path}")
    return files


def pose_row(message, stamp_ns, message_type):
    """Convert one supported message; invalid coordinates cannot become a path."""
    if message_type == "nav_msgs/msg/Odometry" or message_type == "geometry_msgs/msg/PoseWithCovarianceStamped":
        pose = message.pose.pose
        position, quaternion = pose.position, pose.orientation
    elif message_type == "geometry_msgs/msg/PoseStamped":
        position, quaternion = message.pose.position, message.pose.orientation
    elif message_type == "geometry_msgs/msg/TransformStamped":
        position, quaternion = message.transform.translation, message.transform.rotation
    else:
        raise ValueError(f"Unsupported trajectory message type: {message_type}")
    values = [float(getattr(position, axis)) for axis in "xyz"]
    quaternion_values = [float(getattr(quaternion, axis)) for axis in "xyzw"]
    if not all(math.isfinite(value) for value in values + quaternion_values):
        raise ValueError("Nonfinite recorded pose")
    norm = math.sqrt(sum(value * value for value in quaternion_values))
    if norm < 1e-9:
        raise ValueError("Recorded pose has no valid orientation quaternion")
    qx, qy, qz, qw = [value / norm for value in quaternion_values]
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return {
        "stamp_ns": int(stamp_ns), "x": values[0], "y": values[1], "z": values[2],
        "qx": quaternion_values[0], "qy": quaternion_values[1],
        "qz": quaternion_values[2], "qw": quaternion_values[3], "yaw": yaw,
        "header_stamp_ns": int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec),
        "frame_id": message.header.frame_id,
        "child_frame_id": getattr(message, "child_frame_id", ""),
    }


def load_pair_trajectory(source_directory, start_ns, end_ns, trajectory_bag=None, topic=None):
    """Return ``(rows, metadata)``; absent or nonoverlapping poses return no rows.

    Automatic discovery is limited to this recording's ``bags/combined`` and
    known robot-base topics, so caster motion is never mistaken for body motion.
    Explicit ``trajectory_bag`` and ``topic`` select other recorded sources.
    """
    if type(start_ns) is not int or type(end_ns) is not int or start_ns > end_ns:
        raise ValueError("Trajectory interval needs ordered integer epoch timestamps")
    source = Path(source_directory).expanduser().resolve()
    path = Path(trajectory_bag).expanduser().resolve() if trajectory_bag else source / "bags/combined"
    metadata = {
        "available": False, "source_bag": str(path), "topic": topic,
        "time_basis": TIME_BASIS, "encoder_start_ns": start_ns, "encoder_end_ns": end_ns,
        "time_shift_ns": 0, "discovered_topics": [], "reason": "", "invalid_pose_count": 0,
        "coordinate_transform": "none; recorded pose frame and origin retained",
    }
    if not path.exists():
        metadata["reason"] = f"No trajectory bag found at {path}"
        return [], metadata
    files = _bag_files(path)
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    readers, types = [], {}
    for filename in files:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(
            uri=str(filename), storage_id="mcap" if filename.suffix == ".mcap" else "sqlite3"),
            rosbag2_py.ConverterOptions("", ""))
        for value in reader.get_all_topics_and_types():
            if value.name in types and types[value.name] != value.type:
                raise ValueError(f"Trajectory topic changes message type across bag shards: {value.name}")
            types[value.name] = value.type
        readers.append(reader)
    if topic is None:
        topic = next((candidate for candidate in PREFERRED_TOPICS if types.get(candidate) in POSE_TYPES), None)
    metadata["topic"] = topic
    selected_type = types.get(topic)
    selected_class = get_message(selected_type) if selected_type in POSE_TYPES else None
    stats = {name: {"name": name, "type": value, "message_count": 0, "start_ns": None, "end_ns": None}
             for name, value in types.items()}
    rows, frames, child_frames, delays = [], set(), set(), []
    for reader in readers:
        while reader.has_next():
            name, data, stamp = reader.read_next()
            entry = stats[name]
            entry["message_count"] += 1
            entry["start_ns"] = stamp if entry["start_ns"] is None else min(entry["start_ns"], stamp)
            entry["end_ns"] = stamp if entry["end_ns"] is None else max(entry["end_ns"], stamp)
            if name != topic or selected_class is None:
                continue
            try:
                row = pose_row(deserialize_message(data, selected_class), stamp, selected_type)
            except (ValueError, TypeError, RuntimeError):
                metadata["invalid_pose_count"] += 1
                continue
            frames.add(row["frame_id"])
            child_frames.add(row["child_frame_id"])
            if start_ns <= stamp <= end_ns:
                rows.append(row)
                if row["header_stamp_ns"]:
                    delays.append(stamp - row["header_stamp_ns"])
    metadata["discovered_topics"] = [stats[name] for name in sorted(stats)]
    if selected_class is None:
        metadata["reason"] = (f"Selected trajectory topic is absent or unsupported: {topic}" if topic
                              else "No recognized robot-base pose topic; select --trajectory-topic explicitly")
        return [], metadata
    metadata.update(
        message_type=selected_type, source_start_ns=stats[topic]["start_ns"],
        source_end_ns=stats[topic]["end_ns"], source_message_count=stats[topic]["message_count"],
        frame_ids=sorted(frames), child_frame_ids=sorted(child_frames),
    )
    if len(frames) != 1 or len(child_frames) != 1 or "" in frames:
        metadata["reason"] = "Selected trajectory mixes coordinate/child frames or has no valid frame; choose a consistent robot pose topic"
        return [], metadata
    metadata.update(frame_id=next(iter(frames)), child_frame_id=next(iter(child_frames)))
    if not rows:
        metadata["reason"] = "No valid trajectory samples overlap the encoder epoch interval; the bag may belong to another recording run"
        return [], metadata
    rows.sort(key=lambda row: row["stamp_ns"])
    metadata.update(
        available=True, reason="", sample_count=len(rows), start_ns=rows[0]["stamp_ns"], end_ns=rows[-1]["stamp_ns"],
        coverage={"start_gap_s": (rows[0]["stamp_ns"] - start_ns) / 1e9,
                  "end_gap_s": (end_ns - rows[-1]["stamp_ns"]) / 1e9,
                  "max_sample_gap_s": max((right["stamp_ns"] - left["stamp_ns"] for left, right in zip(rows, rows[1:])), default=0) / 1e9},
        header_to_bag_delay_ns={"min": min(delays), "max": max(delays)} if delays else None,
    )
    return rows, metadata
