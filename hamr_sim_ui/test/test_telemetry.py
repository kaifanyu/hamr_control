"""Protocol compatibility, image layout and real ROS bag round-trip checks."""

import importlib.util
import json
import math
from pathlib import Path
import shutil
import sys
import threading
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hamr_sim_ui.telemetry import EncoderAdapter, JOINT_MAPPING, image_frame, joint_readings


def hardware_module(name):
    scripts = Path(__file__).resolve().parents[2] / "hamr_bringup/scripts"
    sys.path.insert(0, str(scripts))
    spec = importlib.util.spec_from_file_location(name, scripts / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_simulated_packets_replay_through_existing_hardware_parser():
    protocol = hardware_module("caster_encoder_protocol")
    replay = hardware_module("caster_replay_model").ReplayModel()
    adapter = EncoderAdapter("ball")
    values = {name: {"position": float(node) / 3, "velocity": -0.25 * node}
              for node, name in JOINT_MAPPING["ball"].items()}
    envelopes = adapter.envelopes(values, 3_100_000_000)
    for envelope in envelopes:
        packet = protocol.decode_packet(json.dumps(envelope["packet"]).encode())
        assert packet["samples"][0]["raw_count"] is None
        assert envelope["source"] == "gazebo_simulation"
        assert envelope["receive_ros_ns"] == 3_100_000_000
        replay.receive(envelope, 3_100_000_000)
    names, positions, status = replay.snapshot(3_100_000_000)
    assert names[:3] == ["caster_positive_hemisphere_joint", "caster_negative_hemisphere_joint",
                         "caster_carrier_joint"]
    assert positions[:3] == pytest.approx([1 / 3, 2 / 3, 1])
    assert status["complete_pose_available"]


def test_missing_joint_does_not_invent_samples_and_clock_reset_creates_segment():
    adapter = EncoderAdapter()
    name = JOINT_MAPPING["ball"][1]
    readings = {name: {"position": 0.4, "velocity": None}}
    before = adapter.envelopes(readings, 9_000_000_000)
    assert len(before) == 1
    assert before[0]["packet"]["samples"][0]["velocity_rad_s"] is None
    assert before[0]["packet"]["samples"][0]["flags"] & (1 << 6) == 0
    after = adapter.envelopes(readings, 100_000_000)
    assert after[0]["packet"]["boot_id"] != before[0]["packet"]["boot_id"]
    assert after[0]["packet"]["samples"][0]["seq"] == 0


def test_swivel_transport_slots_are_explicitly_mapped():
    metadata = EncoderAdapter("swivel").metadata()
    assert metadata["nodes"]["1"]["source_joint"] == "left_caster_swivel_joint"
    assert metadata["nodes"]["2"]["source_joint"] == "right_caster_swivel_joint"
    assert metadata["synthetic"] and not metadata["hardware_calibration"]


def test_nonfinite_or_missing_joint_values_are_rejected():
    result = joint_readings(["present", "nan", "absent"], [0.3, math.nan], [math.inf])
    assert result == {"present": {"position": 0.3, "velocity": None}}


def test_bgr_padding_is_removed_and_pixels_are_rgb():
    pytest.importorskip("numpy")
    message = SimpleNamespace(encoding="bgr8", width=1, height=2, step=4,
                              data=bytes([3, 2, 1, 99, 6, 5, 4, 99]),
                              header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=20)))
    result = image_frame(message)
    assert result["data"] == bytes([1, 2, 3, 4, 5, 6])
    assert result["step"] == 3
    assert result["stamp_ns"] == 1_000_000_020
    message.data = b"short"
    with pytest.raises(ValueError, match="truncated"):
        image_frame(message)


def test_ros_recording_roundtrip(tmp_path, monkeypatch):
    """Exercise actual DDS receipt, CDR serialization, SQLite bag and CSV files."""
    rclpy = pytest.importorskip("rclpy")
    rosbag2_py = pytest.importorskip("rosbag2_py")
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    from hamr_sim_ui.telemetry import RosTelemetry

    monkeypatch.setenv("ROS_DOMAIN_ID", "178")
    telemetry = RosTelemetry()
    try:
        source = rclpy.create_node("hamr_ui_test_source", context=telemetry._context)
        odom_pub = source.create_publisher(Odometry, "/compa/odom", 10)
        joint_pub = source.create_publisher(JointState, "/joint_states", 10)
        clock_pub = source.create_publisher(Clock, "/clock", 10)
        deadline = time.monotonic() + 5
        while joint_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.03)
        telemetry.start_recording(tmp_path, video=False, metadata={"terrain": "flat", "gazebo_log": "/tmp/test-log"})
        clock = Clock()
        clock.clock.sec = 7
        odom = Odometry()
        odom.header.stamp.sec = 7
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.position.x = 2.5
        odom.twist.twist.linear.x = 0.3
        odom.twist.twist.linear.y = 0.4
        joints = JointState()
        joints.header.stamp.sec = 7
        joints.name = list(JOINT_MAPPING["ball"].values())
        joints.position = [0.5, -1.2, 2.4]
        joints.velocity = [0.1, 0.2, 0.3]
        clock_pub.publish(clock)
        odom_pub.publish(odom)
        joint_pub.publish(joints)
        deadline = time.monotonic() + 5
        while (not telemetry.snapshot()["joints_received"] or
               not telemetry.snapshot()["odom_received"]) and time.monotonic() < deadline:
            time.sleep(0.03)
        assert telemetry.snapshot()["linear_speed"] == pytest.approx(0.5)
        assert telemetry.snapshot()["casters"][joints.name[1]]["position"] == -1.2
        summary = telemetry.stop_recording()
        assert summary["message_counts"]["/caster_encoder/packets"] == 3
        assert summary["session"]["terrain"] == "flat"
        assert "0.3,0.4,0.5" in (tmp_path / "velocity.csv").read_text()
        assert "right_caster_negative_hemisphere_joint,-1.2,0.2" in (tmp_path / "joints.csv").read_text()
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(tmp_path / "bag"), storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        packets = []
        while reader.has_next():
            topic, serialized, timestamp = reader.read_next()
            if topic == "/caster_encoder/packets":
                assert timestamp == 7_000_000_000
                packets.append(json.loads(deserialize_message(serialized, String).data))
        assert len(packets) == 3
        assert packets[2]["packet"]["samples"][0]["calibrated_alpha_rad"] == 2.4
        with pytest.raises(FileExistsError):
            telemetry.start_recording(tmp_path, video=False)
        telemetry.reset()
        assert not telemetry.snapshot()["odom_received"]
        assert telemetry.snapshot()["x"] is None
        assert telemetry.snapshot()["history"] == []
        assert telemetry.snapshot()["casters"] == {}
        source.destroy_node()
    finally:
        telemetry.close()


def test_failed_video_is_not_reported_as_complete(tmp_path):
    pytest.importorskip("rosbag2_py")
    from hamr_sim_ui.telemetry import BagRecording
    recording = BagRecording(tmp_path, EncoderAdapter().metadata())
    report = recording.close([{"camera": "closeup", "success": False, "returncode": 2}])
    assert report["recording_complete"] is False
    assert "closeup video failed" in report["errors"][0]
    assert report["video_results"]["closeup"]["success"] is False


def test_caster_and_cameras_switch_without_replacing_ros_node(tmp_path, monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("rosbag2_py")
    from sensor_msgs.msg import JointState
    from hamr_sim_ui.telemetry import RosTelemetry

    monkeypatch.setenv("ROS_DOMAIN_ID", "176")
    telemetry = RosTelemetry(cameras=False)
    source = rclpy.create_node("hamr_ui_test_caster_switch", context=telemetry._context)
    publisher = source.create_publisher(JointState, "/joint_states", 10)
    node = telemetry.node
    try:
        assert telemetry.camera_topics == {}
        assert not telemetry.snapshot()["cameras_enabled"]
        with pytest.raises(RuntimeError, match="camera sensors"):
            telemetry.start_recording(tmp_path / "no-video", video=True)
        assert not (tmp_path / "no-video").exists()
        for caster, cameras in (("ball", True), ("swivel", False), ("swivel", True)):
            previous_boot_id = telemetry.encoder.boot_id
            telemetry.configure(caster, cameras=cameras)
            assert telemetry.node is node
            assert telemetry.encoder.caster == caster
            assert telemetry.encoder.boot_id != previous_boot_id
            topics = {subscription.topic_name for subscription in node.subscriptions}
            image_topics = {topic for topic in topics if topic.endswith("/image")}
            expected = ({f"/hamr/{caster}_caster/closeup/image", "/hamr/recording_camera/image"}
                        if cameras else set())
            assert image_topics == expected
            assert telemetry.snapshot()["history"] == []
            output = tmp_path / f"{caster}-{cameras}"
            telemetry.start_recording(output, video=False)
            with pytest.raises(RuntimeError, match="Stop recording"):
                telemetry.configure("ball", cameras=False)
            joints = JointState()
            joints.header.stamp.sec = 7
            joints.name = list(JOINT_MAPPING[caster].values())
            joints.position = [0.5, -1.2, 2.4]
            joints.velocity = [0.1, 0.2, 0.3]
            deadline = time.monotonic() + 5
            while not telemetry.snapshot()["joints_received"] and time.monotonic() < deadline:
                publisher.publish(joints)
                time.sleep(0.05)
            summary = telemetry.stop_recording()
            assert telemetry.snapshot()["joints_received"]
            assert summary["caster"] == caster
            packets = [json.loads(line) for line in (output / "encoder_packets.jsonl").read_text().splitlines()]
            assert {packet["source_joint"] for packet in packets} == set(JOINT_MAPPING[caster].values())
            assert {packet["packet"]["boot_id"] for packet in packets} == {telemetry.encoder.boot_id}
    finally:
        source.destroy_node()
        telemetry.close()


@pytest.mark.parametrize("preview_cameras", [None, ("closeup",), ()])
def test_both_video_encoders_ready_and_verified(tmp_path, monkeypatch, preview_cameras):
    """Exercise the installed camera recorders and ffmpeg using ROS image frames."""
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("rosbag2_py")
    if not shutil.which("ffmpeg"):
        pytest.skip("ffmpeg is not on PATH")
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    try:
        recorder = Path(get_package_prefix("hamr_ball_caster")) / "lib/hamr_ball_caster/record_simulation.py"
    except PackageNotFoundError:
        pytest.skip("HAMR camera recorder is not installed")
    if not recorder.is_file():
        pytest.skip("HAMR camera recorder is not installed")
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock
    from hamr_sim_ui.telemetry import RosTelemetry

    monkeypatch.setenv("ROS_DOMAIN_ID", "177")
    telemetry = RosTelemetry(preview_cameras=preview_cameras)
    source = rclpy.create_node("hamr_ui_test_camera", context=telemetry._context)
    publishers = [source.create_publisher(Image, topic, 10) for topic in telemetry.camera_topics.values()]
    clock_pub = source.create_publisher(Clock, "/clock", 10)
    stopping = threading.Event()

    def frames():
        count = 0
        while not stopping.wait(0.04):
            image = Image()
            timestamp = 10_000_000_000 + count * 40_000_000
            image.header.stamp.sec, image.header.stamp.nanosec = divmod(timestamp, 1_000_000_000)
            image.width, image.height, image.step, image.encoding = 64, 48, 192, "rgb8"
            image.data = bytes([count % 256, 40, 200]) * (64 * 48)
            clock = Clock(clock=image.header.stamp)
            clock_pub.publish(clock)
            for publisher in publishers:
                publisher.publish(image)
            count += 1

    thread = threading.Thread(target=frames, daemon=True)
    thread.start()
    try:
        actual_topics = {subscription.topic_name for subscription in telemetry.node.subscriptions
                         if subscription.topic_name.endswith("/image")}
        expected_cameras = set(telemetry.camera_topics) if preview_cameras is None else set(preview_cameras)
        assert actual_topics == {telemetry.camera_topics[camera] for camera in expected_cameras}
        telemetry.start_recording(tmp_path, video=True)
        assert all((tmp_path / f"{camera}.ready.json").is_file() for camera in telemetry.camera_topics)
        time.sleep(0.3)
        report = telemetry.stop_recording()
        assert report["recording_complete"], report["errors"]
        assert set(report["video_results"]) == {"closeup", "overview"}
        assert all(result["success"] and result["frames_encoded"] > 0 for result in report["video_results"].values())
        snapshot = telemetry.snapshot()
        assert set(snapshot["cameras"]) == expected_cameras
        assert all(stats["frames"] > 0 for stats in snapshot["cameras"].values())
        assert snapshot["real_time_factor"] is not None
    finally:
        stopping.set()
        thread.join(2)
        source.destroy_node()
        telemetry.close()
