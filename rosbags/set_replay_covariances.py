#!/usr/bin/env python3
"""Inspect a ROS 2 bag's covariances, or prepare a configured offline replay.

Run in a sourced ROS 2 environment with rosbag2_py and the source storage plugin.
Use INPUT --show to print recorded values as Python assignments without copying.
Use INPUT OUTPUT --config FILE to set matrices and save OUTPUT/ekf.yaml.
When copying, the original is read-only; output is an uncompressed sqlite3 bag.
"""

import argparse
import math
from pathlib import Path
import tempfile

import yaml


TARGET_TYPES = {
    "/wheel_odom": "nav_msgs/msg/Odometry",
    "/imu/data": "sensor_msgs/msg/Imu",
}

MATRIX_FIELDS = {
    "/wheel_odom": {
        "wheel_pose": ("pose", "covariance"),
        "wheel_twist": ("twist", "covariance"),
    },
    "/imu/data": {
        "imu_orientation": ("orientation_covariance",),
        "imu_angular_velocity": ("angular_velocity_covariance",),
        "imu_linear_acceleration": ("linear_acceleration_covariance",),
    },
}


def apply_profile(name, msg, matrices, ekf_parameters):
    """Replace complete validated matrices, preserving unavailable IMU markers."""
    imu_ranges = {
        "imu_orientation": slice(3, 6),
        "imu_angular_velocity": slice(9, 12),
        "imu_linear_acceleration": slice(12, 15),
    }
    for key, field_path in MATRIX_FIELDS[name].items():
        owner = msg
        for part in field_path[:-1]:
            owner = getattr(owner, part)
        field = field_path[-1]
        if name == "/imu/data" and getattr(owner, field)[0] == -1:
            selected = any(ekf_parameters["imu0_config"][imu_ranges[key]])
            if selected or key in matrices:
                raise ValueError(
                    f"{key} is unavailable in a recorded IMU message; disable its "
                    "EKF selection and set its covariance to null in the profile"
                )
        if key in matrices:
            setattr(owner, field, list(matrices[key]))


def positive_variance(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("variance must be positive and finite")
    return value


def replace_diagonal(covariance, size, variances):
    """Reject coupled variables instead of silently invalidating covariance."""
    if len(covariance) != size * size:
        raise ValueError("Unexpected covariance matrix size")
    for axis in variances:
        for other in range(size):
            if other != axis and (
                covariance[axis * size + other] != 0.0
                or covariance[other * size + axis] != 0.0
            ):
                raise ValueError("Affected covariance has nonzero cross terms")
    for axis, variance in variances.items():
        covariance[axis * size + axis] = variance


def open_source(source):
    """Validate and open the input without creating or modifying any files."""
    import rosbag2_py

    source = Path(source).expanduser().resolve()
    metadata_path = source / "metadata.yaml"
    if not metadata_path.is_file():
        raise ValueError(f"Missing input metadata: {metadata_path}")
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))[
        "rosbag2_bagfile_information"
    ]
    storage_id = metadata["storage_identifier"]
    if metadata.get("compression_format"):
        raise ValueError("Compressed input bags are not supported by this helper")
    files = metadata["relative_file_paths"]
    if not files or any(not (source / name).is_file() for name in files):
        raise ValueError("An input bag data file is missing")
    recorded_counts = {
        item["topic_metadata"]["name"]: item["message_count"]
        for item in metadata["topics_with_message_count"]
    }
    if any(recorded_counts.get(name, 0) <= 0 for name in TARGET_TYPES):
        raise ValueError("Input must contain nonempty /wheel_odom and /imu/data")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(source), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = reader.get_all_topics_and_types()
    by_name = {topic.name: topic for topic in topics}
    for name, expected_type in TARGET_TYPES.items():
        topic = by_name.get(name)
        if topic is None or topic.type != expected_type or topic.serialization_format != "cdr":
            raise ValueError(f"Expected {name}: {expected_type}, serialized as cdr")
    return reader, topics, recorded_counts


def covariance_snapshot(name, msg):
    if name == "/wheel_odom":
        return (
            ("odom.pose.covariance", tuple(msg.pose.covariance)),
            ("odom.twist.covariance", tuple(msg.twist.covariance)),
        )
    return tuple(
        (f"imu.{field}", tuple(getattr(msg, field)))
        for field in (
            "orientation_covariance",
            "angular_velocity_covariance",
            "linear_acceleration_covariance",
        )
    )


def print_covariances(name, snapshot):
    """Show diagonal entries plus any nonzero cross terms; omit zero cross terms."""
    print(f"\n# {name} -- first recorded message")
    for field, values in snapshot:
        if field == "odom.pose.covariance":
            labels = ("x", "y", "z", "roll", "pitch", "yaw")
        elif field == "odom.twist.covariance":
            labels = ("vx", "vy", "vz", "vroll", "vpitch", "vyaw")
        elif field == "imu.orientation_covariance":
            labels = ("roll", "pitch", "yaw")
        elif field == "imu.angular_velocity_covariance":
            labels = ("wx", "wy", "wz")
        else:
            labels = ("ax", "ay", "az")
        size = len(labels)
        print()
        for axis, label in enumerate(labels):
            index = axis * (size + 1)
            print(f"{field}[{index}] = {float(values[index])!r}  # {label}")
        for index, value in enumerate(values):
            row, col = divmod(index, size)
            if row != col and value != 0:
                print(f"{field}[{index}] = {float(value)!r}  # {labels[row]}/{labels[col]}")


def show_bag(source):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader, _, recorded_counts = open_source(source)
    types = {name: get_message(kind) for name, kind in TARGET_TYPES.items()}
    first, varied = {}, set()
    counts = dict.fromkeys(TARGET_TYPES, 0)
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name not in types:
            continue
        snapshot = covariance_snapshot(name, deserialize_message(data, types[name]))
        counts[name] += 1
        if name not in first:
            first[name] = snapshot
        elif snapshot != first[name]:
            varied.add(name)
    if any(counts[name] != recorded_counts[name] for name in TARGET_TYPES):
        raise ValueError("Read message counts do not match input metadata")
    print(f"# Sensor covariance stored in {Path(source).expanduser().resolve()}")
    for name in TARGET_TYPES:
        print_covariances(name, first[name])
        if name in varied:
            print(f"# WARNING: covariance changes across {counts[name]} messages; shown above is the first.")
        else:
            print(f"# Covariance is constant across all {counts[name]} messages.")


def copy_bag(source, output, wheel_vx=None, wheel_vy=None, imu_wz=None, config=None):
    matrices, ekf_document = None, None
    if config is not None:
        if any(value is not None for value in (wheel_vx, wheel_vy, imu_wz)):
            raise ValueError("Use --config or the three variance flags, not both")
        from offline_replay_config import load_profile

        matrices, ekf_document = load_profile(config)
    else:
        for value in (wheel_vx, wheel_vy, imu_wz):
            positive_variance(value)

    from rclpy.serialization import deserialize_message, serialize_message
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message

    source = Path(source).expanduser().resolve()
    output = Path(output).expanduser().resolve()
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if source == output or source in output.parents:
        raise ValueError("Output must be outside the original bag directory")
    reader, topics, recorded_counts = open_source(source)
    types = {name: get_message(kind) for name, kind in TARGET_TYPES.items()}

    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.partial-", dir=output.parent))
    staged_bag = staging / "bag"
    writer = rosbag2_py.SequentialWriter()
    counts = dict.fromkeys(TARGET_TYPES, 0)
    first = {}
    total = 0
    try:
        writer.open(
            rosbag2_py.StorageOptions(uri=str(staged_bag), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""),
        )
        for topic in topics:
            writer.create_topic(topic)  # Preserve names, types, serialization and QoS.
        while reader.has_next():
            name, data, timestamp = reader.read_next()
            if name in TARGET_TYPES:
                msg = deserialize_message(data, types[name])
                if matrices is not None:
                    apply_profile(name, msg, matrices, ekf_document["/**"]["ros__parameters"])
                elif name == "/wheel_odom":
                    replace_diagonal(msg.twist.covariance, 6, {0: wheel_vx, 1: wheel_vy})
                else:
                    if msg.angular_velocity_covariance[0] == -1:
                        raise ValueError("IMU marks angular velocity as unavailable")
                    replace_diagonal(msg.angular_velocity_covariance, 3, {2: imu_wz})
                data = serialize_message(msg)
                counts[name] += 1
                if name not in first:
                    first[name] = covariance_snapshot(name, msg)
            writer.write(name, data, timestamp)
            total += 1
        if any(counts[name] != recorded_counts[name] for name in TARGET_TYPES):
            raise ValueError("Read message counts do not match input metadata")
        del writer  # Flush sqlite3 and metadata before publishing the completed copy.
        if ekf_document is not None:
            (staged_bag / "ekf.yaml").write_text(
                yaml.safe_dump(ekf_document, sort_keys=False), encoding="utf-8"
            )
            # Freeze a reusable profile alongside the exact EKF config used by this trial.
            saved_matrices = {}
            for key, values in matrices.items():
                size = 6 if key.startswith("wheel_") else 3
                saved_matrices[key] = [values[i:i + size] for i in range(0, len(values), size)]
            (staged_bag / "offline_replay.yaml").write_text(
                yaml.safe_dump({
                    "base_ekf_config": "ekf.yaml", "ekf": {}, "covariances": saved_matrices,
                }, sort_keys=False), encoding="utf-8"
            )
        if output.exists():
            raise ValueError(f"Output appeared during copying: {output}")
        staged_bag.rename(output)
        staging.rmdir()
    except Exception as exc:
        raise RuntimeError(f"Copy failed; incomplete files retained at {staging}: {exc}") from exc
    print(f"Created {output} ({total} messages; updated {counts})")
    if ekf_document is not None:
        print(f"EKF settings for this trial: {output / 'ekf.yaml'}")
    print("# Copied input sensor values (not EKF output covariance):")
    for name in TARGET_TYPES:
        print_covariances(name, first[name])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path, nargs="?")
    parser.add_argument("--show", action="store_true", help="Read and display input covariances; do not copy")
    parser.add_argument("--config", type=Path, help="Offline profile YAML: sensor selections and full covariance matrices")
    parser.add_argument("--wheel-vx", type=positive_variance, help="(m/s)^2")
    parser.add_argument("--wheel-vy", type=positive_variance, help="(m/s)^2")
    parser.add_argument("--imu-wz", type=positive_variance, help="(rad/s)^2")
    args = parser.parse_args()
    variances = (args.wheel_vx, args.wheel_vy, args.imu_wz)
    copy_args = (args.output, args.config, *variances)
    if args.show and any(value is not None for value in copy_args):
        parser.error("--show takes only INPUT; omit OUTPUT, --config and variance overrides")
    if not args.show:
        if args.output is None:
            parser.error("copying requires OUTPUT")
        if args.config is not None and any(value is not None for value in variances):
            parser.error("use --config or the three variance flags, not both")
        if args.config is None and any(value is None for value in variances):
            parser.error("copying requires --config FILE or all three variance flags")
    try:
        if args.show:
            show_bag(args.input)
        else:
            copy_bag(args.input, args.output, args.wheel_vx, args.wheel_vy, args.imu_wz, config=args.config)
    except (OSError, ValueError, KeyError, ImportError, RuntimeError) as exc:
        parser.exit(1, f"Error: {exc}\n")


if __name__ == "__main__":
    main()
