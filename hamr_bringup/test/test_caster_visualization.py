"""Real MCAP coverage for the one-command report and source preservation."""
import csv
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from caster_bag_reader import read_recording, replay_events
from caster_visualization import main
from test_render_caster_recording import local_packets, save_packets, EPOCH


def full_bag(path, include_packets=True, include_forks=True):
    rosbag = pytest.importorskip("rosbag2_py")
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState, Imu, MagneticField
    from std_msgs.msg import String, Float64, Float64MultiArray, MultiArrayDimension, Int32
    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(path), storage_id="mcap"), rosbag.ConverterOptions("", ""))
    created, messages = set(), []
    def add(topic, kind, message, stamp):
        if topic not in created:
            writer.create_topic(rosbag.TopicMetadata(id=len(created), name=topic, type=kind, serialization_format="cdr"))
            created.add(topic)
        messages.append((stamp, topic, message if isinstance(message, bytes) else serialize_message(message)))
    subjects = ["HAMR_base", "HAMR_turret"] + (["Left_Caster", "Right_Caster"] if include_forks else [])
    for i in range(101):
        stamp = EPOCH + 250_000_000 + i*10_000_000
        for j, subject in enumerate(subjects):
            pose = PoseStamped()
            pose.header.frame_id = "mocap"
            pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(stamp-1_000_000, 1_000_000_000)
            pose.pose.position.x, pose.pose.position.y = i*.002, j*.1
            pose.pose.orientation.w = 1.
            add(f"/{subject}/pose", "geometry_msgs/msg/PoseStamped", pose, stamp)
        imu = Imu()
        imu.header = pose.header
        imu.orientation.w, imu.linear_acceleration.z = 1., 9.81
        add("/imu/data", "sensor_msgs/msg/Imu", imu, stamp)
        mag = MagneticField()
        mag.header = pose.header
        mag.magnetic_field.x = .00002
        add("/imu/mag", "sensor_msgs/msg/MagneticField", mag, stamp)
        add("/left_wheel/cmd_vel", "std_msgs/msg/Float64", Float64(data=1.25), stamp)
        add("/left_wheel/encoder_ticks", "std_msgs/msg/Int32", Int32(data=i*100), stamp)
        status = Float64MultiArray(data=[0.]*17)
        status.layout.dim = [MultiArrayDimension(label="wheel_control_status_v1", size=17, stride=17)]
        add("/wheel_control/status", "std_msgs/msg/Float64MultiArray", status, stamp)
        add("/reference_trajectory", "hamr_interfaces/msg/ReferenceTraj", b"\x00\x01\x00\x00" + struct.pack("<10d", i*.002, 0., 0., 0., 0., .2, 0., 0., 0., 0.), stamp)
        odom = Odometry()
        odom.header = pose.header
        odom.pose.pose.orientation.w, odom.pose.pose.position.x = 1., i*.002
        add("/local_HAMR/odom", "nav_msgs/msg/Odometry", odom, stamp)
    for wheel in ("front_left", "front_right"):
        for envelope in local_packets(wheel):
            envelope["clock_mapping"]["valid_to_us"] = 1250000
            packet = envelope["packet"]
            prefix = f"/caster_recording/{wheel}"
            if packet["type"] == "metadata":
                if include_packets:
                    add(prefix+"/metadata", "std_msgs/msg/String", String(data=json.dumps(envelope)), EPOCH+250_000_000)
                continue
            sample = packet["samples"][0]
            for i in range(201):
                current = {**sample, "seq": sample["seq"]+i, "t_us": 250000+i*5000}
                if packet["node_id"] != 3:
                    current["angle_rad"] = i*.01
                value = {**envelope, "packet": {**packet, "samples": [current]}}
                stamp = EPOCH+250_000_000+i*5_000_000+packet["node_id"]*100
                if include_packets:
                    add(prefix+"/packets", "std_msgs/msg/String", String(data=json.dumps(value)), stamp)
                joint = JointState()
                joint.header.stamp.sec, joint.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                role = {1:"H1", 2:"H2", 3:"CENTER"}[packet["node_id"]]
                joint.name, joint.velocity = [role], [2.]
                joint.position = [current["angle_rad"] if role != "CENTER" else current["calibrated_alpha_rad"]]
                add(prefix+f"/{role}/joint_states", "sensor_msgs/msg/JointState", joint, stamp)
    for stamp, topic, data in sorted(messages, key=lambda item: item[0]):
        writer.write(topic, data, stamp)
    writer.close()
    return next(path.glob("*.mcap"))


def test_full_combined_bag_and_bare_mcap_read_all_channels_without_sidecars(tmp_path):
    bag = tmp_path / "recording with spaces"
    source = full_bag(bag)
    original = hashlib.sha256(source.read_bytes()).hexdigest()
    recording = read_recording(source)
    assert recording.counts_verified
    assert len(recording.encoder_topics()) == 6
    assert set(recording.packet_streams()) == {"front_left", "front_right"}
    assert "/imu/data" in recording.numeric
    assert "left_target_rpm" in recording.numeric["/wheel_control/status"][0]
    assert recording.numeric["/reference_trajectory"][0]["x_dot"] == .2
    events, provenance = replay_events(recording)
    assert provenance["sample_counts"]["front_right"] == {"H1": 201, "H2": 201, "CENTER": 201}
    assert all(len(events[wheel]) == 606 for wheel in events)
    (bag / "metadata.yaml").unlink()
    bare = read_recording(source)
    assert not bare.counts_verified
    assert bare.start_ns == recording.start_ns
    assert bare.end_ns == recording.end_ns
    assert hashlib.sha256(source.read_bytes()).hexdigest() == original


def test_one_command_exports_both_wheels_and_estimates_with_exact_time(tmp_path):
    source = full_bag(tmp_path / "bag")
    target = tmp_path / "report"
    before = source.read_bytes()
    assert main([str(source), "--output", str(target), "--no-plots"]) == 0
    report = json.loads((target / "report.json").read_text())
    assert report["stages"]["fork_comparison"]["status"] == "complete"
    assert (target / "index.html").exists()
    with (target / "comparison/front_right_aligned.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert int(rows[1]["epoch_ns"]) == EPOCH+270_000_000
    assert any(row["fork_estimate_valid"] == "True" for row in rows)
    for wheel in ("front_left", "front_right"):
        with (target / "analysis/encoder_packets" / wheel / "samples.csv").open() as stream:
            packets = list(csv.DictReader(stream))
        assert len(packets) == 603
        assert packets[0]["seq"] == str(2**53+1)
        assert {p["wheel_id"] for p in packets} == {wheel}
    assert source.read_bytes() == before
    with pytest.raises(SystemExit):
        main([str(source), "--output", str(target)])


def test_missing_forks_or_packets_keeps_available_reports_and_explains_omissions(tmp_path):
    source = full_bag(tmp_path / "partial", include_packets=False, include_forks=False)
    target = tmp_path / "report"
    assert main([str(source), "--output", str(target), "--no-plots"]) == 0
    report = json.loads((target / "report.json").read_text())
    assert report["stages"]["fork_comparison"]["status"] == "unavailable"
    assert (target / "analysis/encoder_signals.csv").is_file()
    assert len(json.loads((target / "analysis/summary.json").read_text())["encoders"]) == 6
    with pytest.raises(ValueError, match="packet history"):
        replay_events(read_recording(source))


def test_help_works_outside_checkout_and_invalid_input_creates_nothing(tmp_path):
    result = subprocess.run(["bash", str(SCRIPTS/"visualize_caster_bag"), "--help"], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0 and ".mcap" in result.stdout
    assert "--render" in result.stdout and "index.html" in result.stdout
    with pytest.raises(SystemExit):
        main([str(tmp_path/"missing.mcap"), "--output", str(tmp_path/"report")])
    assert not (tmp_path/"report").exists()


def test_packet_only_duration_includes_last_batched_sample_and_single_wheel_selection(tmp_path):
    save_packets(tmp_path, local_packets()+local_packets("front_right"))
    recording = read_recording(tmp_path)
    assert recording.start_ns == EPOCH+250_000_000+100
    assert recording.end_ns == EPOCH+255_000_000+300
    events, provenance = replay_events(recording, wheels=("front_right",))
    assert set(events) == {"front_right"}
    assert provenance["sample_counts"] == {"H1": 2, "H2": 2, "CENTER": 2}


def test_mixing_local_and_live_packet_clocks_is_not_silently_replayed(tmp_path):
    save_packets(tmp_path, local_packets()+local_packets("front_right"))
    recording = read_recording(tmp_path)
    recording.packets["/caster_recording/front_right/packets"][0][0]["origin"] = "udp"
    with pytest.raises(ValueError, match="Cannot mix"):
        replay_events(recording)
