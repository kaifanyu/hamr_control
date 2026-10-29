"""Exercise six UDP devices through real ROS bridges, recording and CSV export.

Runs on loopback in a separate DDS domain. No physical interfaces or motors are
used. The test skips on development hosts without ROS bag/MCAP dependencies.
"""

import csv
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip("rclpy")
rosbag2_py = pytest.importorskip("rosbag2_py")
pytest.importorskip("ros2bag")
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import String

PACKAGE = Path(__file__).resolve().parents[1]
WHEELS = ("front_left", "front_right")
ROLES = {1: "H1", 2: "H2", 3: "CENTER"}
BOOT = "0123456789abcdef"
NEXT_BOOT = "fedcba9876543210"
COUNTERS = ("acquisition_overruns", "queue_drops", "invalid_spi_responses",
            "index_queue_drops", "radio_failures")


def device(wheel, role):
    return f"02:00:00:00:{WHEELS.index(wheel) + 1:02x}:{role:02x}"


def packet(wheel, role, seq=0, boot=BOOT, metadata=False):
    value = {"v": 2, "type": "metadata" if metadata else "samples",
             "wheel_id": wheel, "device_id": device(wheel, role),
             "node_id": role, "boot_id": boot}
    if metadata:
        value.update(role=ROLES[role], firmware_version="integration-test",
                     idf_version="v5.4.2", encoder_ppr=2048, ppr_verified=False)
    else:
        position = WHEELS.index(wheel) + role / 10 + seq / 1000
        value.update(diagnostics=dict.fromkeys(COUNTERS, 0), samples=[{
            "seq": seq, "t_us": 250000 + seq * 5000,
            "raw_count": str(seq) if role != 3 else None,
            "raw_spi_word": 123 if role == 3 else None,
            "position14": 123 if role == 3 else None,
            "angle_rad": position, "absolute_angle_rad": position if role == 3 else None,
            "calibrated_alpha_rad": position if role == 3 else None,
            "velocity_rad_s": .2, "filtered_velocity_rad_s": .2,
            "index_event_count": 0, "index_timestamp_us": 0,
            "index_raw_count": "0" if role != 3 else None,
            "flags": 1 | 2 | 16 | 64,
        }])
    return value


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def test_six_devices_remain_separate_through_mcap_and_csv(tmp_path, monkeypatch):
    if not shutil.which("ros2") or "mcap" not in rosbag2_py.get_registered_writers():
        pytest.skip("ROS CLI and MCAP storage are required")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", str(180 + os.getpid() % 40))
    context = Context()
    context.init(args=[])
    node = Node("dual_caster_pipeline_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    processes, logs, subscriptions = [], [], []
    senders = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2)]
    senders[0].bind(("127.0.0.1", 0))
    senders[1].bind(("127.0.0.2", 0))
    messages, ports = {}, {}
    topics = ("packets", "metadata", "diagnostics", "H1/joint_states",
              "H2/joint_states", "CENTER/joint_states")

    def wait_for(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert all(p.poll() is None for p in processes), [p.poll() for p in processes]
            assert time.monotonic() < deadline, "Timed out waiting for ROS pipeline"
            executor.spin_once(timeout_sec=.02)

    def send(wheel, role, seq=0, **kwargs):
        data = packet(wheel, role, seq, **kwargs)
        senders[0].sendto(json.dumps(data).encode(), ("127.0.0.1", ports[wheel]))

    def start(arguments, filename, env=None):
        handle = (tmp_path / filename).open("w")
        logs.append(handle)
        process = subprocess.Popen(arguments, stdout=handle, stderr=subprocess.STDOUT, env=env)
        processes.append(process)
        return process

    try:
        for wheel in WHEELS:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind(("127.0.0.1", 0))
                ports[wheel] = probe.getsockname()[1]
            for suffix in topics:
                topic = f"/caster_encoder/{wheel}/{suffix}"
                messages[topic] = []
                qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL if suffix == "metadata"
                                 else DurabilityPolicy.VOLATILE)
                subscriptions.append(node.create_subscription(
                    JointState if suffix.endswith("joint_states") else String,
                    topic, lambda message, topic=topic: messages[topic].append(message), qos))
            start([sys.executable, str(PACKAGE / "scripts/caster_encoder_bridge"),
                   "--ros-args", "-r", f"__node:=caster_encoder_{wheel}",
                   "-p", f"wheel_id:={wheel}", "-p", "bind_address:=127.0.0.1",
                   "-p", f"udp_port:={ports[wheel]}", "-p", "stale_timeout_s:=0.3",
                   "-p", "expected_device_ids:=" + json.dumps([device(wheel, r) for r in ROLES]),
                   "-p", 'expected_source_ips:=["127.0.0.1", "127.0.0.1", "127.0.0.1"]'],
                  f"{wheel}.log")
        wait_for(lambda: all(node.count_publishers(topic) for topic in messages))
        env = dict(os.environ, HAMR_BAG_ROOT=str(tmp_path), HAMR_BAG_NAME="dual", HAMR_BAG_STORAGE_ID="mcap")
        env.pop("HAMR_BAG_QOS_OVERRIDES", None)
        recorder = start(["bash", str(PACKAGE / "scripts/record_hamr_test_bag")], "recorder.log", env)
        wait_for(lambda: all(node.count_subscribers(topic) >= 2 for topic in messages))
        for wheel in WHEELS:
            for role in ROLES:
                send(wheel, role, metadata=True)
                send(wheel, role)
        wait_for(lambda: all(messages[t] for t in messages if not t.endswith("diagnostics")))
        # Wrong wheel, wrong physical device, and wrong source address must never
        # enter the left packet topic or alter its stream/metadata identity.
        bad = packet("front_right", 1, 9000)
        senders[0].sendto(json.dumps(bad).encode(), ("127.0.0.1", ports["front_left"]))
        bad = packet("front_left", 1, 9001)
        bad["device_id"] = "02:00:00:ff:ff:ff"
        senders[0].sendto(json.dumps(bad).encode(), ("127.0.0.1", ports["front_left"]))
        senders[1].sendto(json.dumps(packet("front_left", 1, 9002)).encode(),
                          ("127.0.0.1", ports["front_left"]))
        # Reboot only left H1; right H1 keeps the original boot and advances.
        send("front_left", 1, metadata=True, boot=NEXT_BOOT)
        send("front_left", 1, boot=NEXT_BOOT)
        for wheel in WHEELS:
            for role in ROLES:
                send(wheel, role, 1, boot=NEXT_BOOT if (wheel, role) == ("front_left", 1) else BOOT)
        wait_for(lambda: all(len(messages[f"/caster_encoder/{w}/{r}/joint_states"]) >= 2
                            for w in WHEELS for r in ROLES.values()))
        left_diag = "/caster_encoder/front_left/diagnostics"
        wait_for(lambda: messages[left_diag]
                 and json.loads(messages[left_diag][-1].data)["rejected_packets"] >= 3
                 and all(messages[f"/caster_encoder/{wheel}/diagnostics"]
                         and json.loads(messages[f"/caster_encoder/{wheel}/diagnostics"][-1].data)
                         ["inventory_verified"] for wheel in WHEELS))
        for wheel in WHEELS:
            for role in ROLES:
                joint = messages[f"/caster_encoder/{wheel}/{ROLES[role]}/joint_states"][-1]
                assert wheel in joint.name[0]
                assert joint.position[0] == pytest.approx(WHEELS.index(wheel) + role / 10 + .001)
        stop(recorder)
        processes.remove(recorder)
        assert recorder.returncode == 0, (tmp_path / "recorder.log").read_text()
    finally:
        for process in reversed(processes):
            stop(process)
        for handle in logs:
            handle.close()
        for sender in senders:
            sender.close()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(tmp_path / "dual"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    recorded = set()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        recorded.add(topic)
        if topic.endswith("/packets"):
            envelope = json.loads(deserialize_message(data, String).data)
            assert envelope["packet"]["wheel_id"] == topic.split("/")[2]
            assert all(sample["seq"] < 9000 for sample in envelope["packet"].get("samples", []))
    assert set(messages) <= recorded, sorted(set(messages) - recorded)
    for wheel in WHEELS:
        report = tmp_path / wheel
        result = subprocess.run([sys.executable, str(PACKAGE / "scripts/caster_visualization.py"),
                                 str(tmp_path / "dual"), "--wheel", wheel,
                                 "--output", str(report), "--no-plots"],
                                text=True, capture_output=True, timeout=15)
        assert result.returncode == 0, result.stderr + result.stdout
        rows = list(csv.DictReader((report / "analysis/encoder_packets" / wheel / "samples.csv").open()))
        assert rows and {row["wheel_id"] for row in rows} == {wheel}
        assert {row["device_id"] for row in rows} == {device(wheel, r) for r in ROLES}
        h1_boots = {row["boot_id"] for row in rows if row["node_id"] == "1"}
        assert h1_boots == ({BOOT, NEXT_BOOT} if wheel == "front_left" else {BOOT})
