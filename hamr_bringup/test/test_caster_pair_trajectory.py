"""Recorded body paths must retain their frame, epoch, and recording overlap."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from caster_pair_trajectory import load_pair_trajectory, pose_row

EPOCH = 1_800_000_000_000_000_001


def write_bag(path, topic="/HAMR_base/odom", frames=("mocap",) * 4):
    rosbag2_py = pytest.importorskip("rosbag2_py")
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    fields = dict(name=topic, type="nav_msgs/msg/Odometry", serialization_format="cdr")
    try:
        metadata = rosbag2_py.TopicMetadata(id=0, **fields)
    except TypeError:
        metadata = rosbag2_py.TopicMetadata(**fields)
    writer.create_topic(metadata)
    for index, frame in enumerate(frames):
        message = Odometry()
        message.header.frame_id = frame
        message.child_frame_id = "HAMR_base/base_link"
        message.pose.pose.position.x = 4.0 + index
        message.pose.pose.position.y = -2.0
        message.pose.pose.orientation.z = math.sin(math.pi / 4)
        message.pose.pose.orientation.w = math.cos(math.pi / 4)
        stamp = EPOCH + index * 1_000_000_000
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp - 10_000_000, 1_000_000_000)
        writer.write(topic, serialize_message(message), stamp)
    writer.close()


def test_absent_auto_bag_returns_reason_without_ros(tmp_path):
    rows, metadata = load_pair_trajectory(tmp_path, EPOCH, EPOCH + 10)
    assert rows == []
    assert metadata["available"] is False
    assert "No trajectory bag" in metadata["reason"]
    assert metadata["time_shift_ns"] == 0


def test_pose_conversion_preserves_xyz_and_reports_invalid_orientation():
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id="mocap", stamp=SimpleNamespace(sec=1, nanosec=5)),
        transform=SimpleNamespace(translation=SimpleNamespace(x=4, y=5, z=6),
                                  rotation=SimpleNamespace(x=0, y=0, z=2, w=2)), child_frame_id="robot")
    row = pose_row(message, EPOCH, "geometry_msgs/msg/TransformStamped")
    assert (row["x"], row["y"], row["z"]) == (4, 5, 6)
    assert row["stamp_ns"] == EPOCH
    assert row["yaw"] == pytest.approx(math.pi / 2)
    assert row["qz"] == 2  # Preserve raw quaternion; normalize only for derived yaw.
    message.transform.rotation.z = message.transform.rotation.w = 0
    with pytest.raises(ValueError, match="orientation"):
        pose_row(message, EPOCH, "geometry_msgs/msg/TransformStamped")


def test_auto_combined_bag_clips_to_existing_encoder_epoch_and_preserves_coordinates(tmp_path):
    bag = tmp_path / "bags/combined"
    bag.parent.mkdir()
    write_bag(bag)
    rows, metadata = load_pair_trajectory(tmp_path, EPOCH + 500_000_000, EPOCH + 2_500_000_000)
    assert [row["stamp_ns"] for row in rows] == [EPOCH + 1_000_000_000, EPOCH + 2_000_000_000]
    assert [row["x"] for row in rows] == [5, 6]
    assert metadata["available"] is True
    assert metadata["frame_id"] == "mocap"
    assert metadata["source_message_count"] == 4
    assert metadata["header_to_bag_delay_ns"] == {"min": 10_000_000, "max": 10_000_000}
    assert metadata["discovered_topics"][0]["start_ns"] == EPOCH


def test_other_run_is_not_shifted_to_match_encoder_clock(tmp_path):
    bag = tmp_path / "bag"
    write_bag(bag)
    rows, metadata = load_pair_trajectory(tmp_path, EPOCH + 100_000_000_000, EPOCH + 130_000_000_000, trajectory_bag=bag)
    assert rows == []
    assert "another recording run" in metadata["reason"]
    assert metadata["source_start_ns"] == EPOCH
    assert metadata["time_shift_ns"] == 0


def test_mixed_frames_are_rejected_instead_of_joined_into_a_path(tmp_path):
    bag = tmp_path / "bag"
    write_bag(bag, frames=("mocap", "odom", "mocap", "odom"))
    rows, metadata = load_pair_trajectory(tmp_path, EPOCH, EPOCH + 4_000_000_000, trajectory_bag=bag)
    assert rows == []
    assert "mixes coordinate" in metadata["reason"]
    assert metadata["frame_ids"] == ["mocap", "odom"]


def test_caster_topic_is_never_automatically_used_for_robot_body(tmp_path):
    bag = tmp_path / "bag"
    write_bag(bag, topic="/Left_Caster/odom")
    rows, metadata = load_pair_trajectory(tmp_path, EPOCH, EPOCH + 4_000_000_000, trajectory_bag=bag)
    assert rows == []
    assert "No recognized robot-base" in metadata["reason"]
    rows, metadata = load_pair_trajectory(tmp_path, EPOCH, EPOCH + 4_000_000_000,
                                          trajectory_bag=bag, topic="/Left_Caster/odom")
    assert len(rows) == 4
