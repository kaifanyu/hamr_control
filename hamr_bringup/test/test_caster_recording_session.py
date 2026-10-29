"""Real HTTP fake ESPs and real MCAP merges, with no robot hardware involved."""

from contextlib import ExitStack, contextmanager
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import caster_recording_client as client
import caster_recording_session as workflow
from caster_recording_bag import merge_session
from test_caster_recording_client import Recorder, make_log
from test_caster_recording_format import EPOCH, exchange

rosbag = pytest.importorskip("rosbag2_py")
from rclpy.serialization import deserialize_message, serialize_message
from nav_msgs.msg import Odometry
from hamr_interfaces.msg import ReferenceTraj
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def make_bag(path):
    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(path), storage_id="mcap"), rosbag.ConverterOptions("", ""))
    for name, kind in [("/local_HAMR/odom", "nav_msgs/msg/Odometry"),
                       ("/reference_trajectory", "hamr_interfaces/msg/ReferenceTraj")]:
        writer.create_topic(rosbag.TopicMetadata(id=0, name=name, type=kind, serialization_format="cdr"))
    odom = Odometry()
    odom.header.stamp.sec = EPOCH // 1_000_000_000
    odom.pose.pose.position.x = 1.23
    writer.write("/local_HAMR/odom", serialize_message(odom), EPOCH + 15_000_000, EPOCH + 14_000_000)
    writer.write("/reference_trajectory", serialize_message(ReferenceTraj(x=1.5, yaw=0.3)),
                 EPOCH + 25_000_000, EPOCH + 24_000_000)
    writer.close()


def read_bag(path):
    reader = rosbag.SequentialReader()
    reader.open(rosbag.StorageOptions(uri=str(path)), rosbag.ConverterOptions("", ""))
    result = []
    while reader.has_next():
        result.append(reader.read_next())
    reader.close()
    return result


def mcap_messages(data):
    """Read uncompressed MCAP Message records to verify send AND receive times.

    Test fixtures deliberately use default uncompressed chunks. The production
    merge delegates all storage parsing/compression to native rosbag2.
    """
    offset = 0
    while offset < len(data):
        opcode, size = struct.unpack_from("<BQ", data, offset)
        payload = data[offset + 9:offset + 9 + size]
        offset += 9 + size
        if opcode == 5:
            yield struct.unpack_from("<HIQQ", payload), payload[22:]
        elif opcode == 6:
            compression_size, = struct.unpack_from("<I", payload, 28)
            assert compression_size == 0
            length, = struct.unpack_from("<Q", payload, 32)
            yield from mcap_messages(payload[40:40 + length])


def mcap_timestamps(path):
    data = next(path.glob("*.mcap")).read_bytes()
    return sorted((header[2], header[3], message) for header, message in mcap_messages(data[8:-8]))


@contextmanager
def dummy_recorder(root, bag):
    cli = root / "ros2"
    cli.write_text("import signal,sys,time\nsignal.signal(signal.SIGINT, lambda *_: sys.exit(0))\nprint('ready',flush=True)\nwhile True: time.sleep(.1)\n")
    process = subprocess.Popen([sys.executable, str(cli), "bag", "record", "--output", str(bag)],
                               stdout=subprocess.PIPE, text=True)
    assert process.stdout.readline().strip() == "ready"
    try:
        yield process
    finally:
        if process.poll() is None:
            process.terminate()
        process.wait(timeout=5)
        process.stdout.close()


def configure(root, monkeypatch, devices):
    bag = root / "robot"
    make_bag(bag)
    client.write_json(root / ".latest_hamr_bag.json", {"version": 1, "bag_path": str(bag)})
    monkeypatch.setattr(client, "discover", lambda *args: devices)
    original_transport = client.Transport
    class TimedTransport(original_transport):
        def sync(self, device):
            status = self.request(device, "/api/status")
            timestamp = 1000 if status["state"] in ("prepared", "recording") else 2_000_000
            return exchange(timestamp, identity={key: device[key] for key in (
                "v", "wheel_id", "node_id", "device_id", "boot_id")})
    monkeypatch.setattr(client, "Transport", TimedTransport)
    return bag


@pytest.mark.parametrize("roles", [(("front_right", 3),),
    tuple((wheel, node) for wheel in client.WHEELS for node in client.ROLES)])
def test_manual_start_stop_merges_all_available_and_preserves_robot_data(tmp_path, monkeypatch, roles):
    with ExitStack() as stack:
        boards = [stack.enter_context(Recorder(node, wheel)) for wheel, node in roles]
        bag = configure(tmp_path, monkeypatch, [board.device for board in boards])
        original = read_bag(bag)
        timestamps = mcap_timestamps(bag)
        original_hash = hashlib.sha256(next(bag.glob("*.mcap")).read_bytes()).digest()
        process = stack.enter_context(dummy_recorder(tmp_path, bag))
        args = ["--root", str(tmp_path)]
        assert workflow.main("start", ["trial", *args]) == 0
        assert all(board.state == "recording" for board in boards)
        assert process.poll() is None
        manifest = client.Session.load(tmp_path, "trial").manifest
        assert len(manifest["devices"]) == len(boards)
        # A duplicate start cannot replace the identity used by the stop command.
        assert workflow.main("start", ["duplicate", *args]) == 2
        assert json.loads((tmp_path / workflow.MARKER).read_text())["run_id"] == "trial"
        assert workflow.main("stop", args) == 0
        process.wait(timeout=5)
        assert all(board.acknowledged and board.state == "stopped" for board in boards)
        directory = tmp_path / "encoder_trial"
        combined = directory / "bags/combined"
        messages = read_bag(combined)
        assert [msg for msg in messages if not msg[0].startswith("/caster_recording/")] == original
        assert all(item in mcap_timestamps(combined) for item in timestamps)
        assert hashlib.sha256(next(bag.glob("*.mcap")).read_bytes()).digest() == original_hash
        assert [msg[2] for msg in messages] == sorted(msg[2] for msg in messages)
        packets = [msg for msg in messages if msg[0].endswith("/packets")]
        assert len(packets) == 3 * len(boards)
        for topic, data, stamp in packets:
            envelope = json.loads(deserialize_message(data, String).data)
            assert envelope["receive_epoch_ns"] is None
            assert envelope["timing_estimated"] is True
            assert envelope["clock_mapping"]["accuracy_validated"] is False
            assert envelope["clock_mapping"]["uncertainty_ns"] > 0
            assert stamp == EPOCH + envelope["packet"]["samples"][0]["t_us"] * 1000
            assert envelope["packet"]["wheel_id"] in topic
        for topic, data, stamp in messages:
            if topic.endswith("/joint_states"):
                joint = deserialize_message(data, JointState)
                assert stamp == joint.header.stamp.sec * 1_000_000_000 + joint.header.stamp.nanosec
        state = json.loads((directory / workflow.STATE).read_text())
        assert len(state["missing_at_start"]) == 6 - len(boards)
        assert state["state"] == "complete"
        requests = [list(board.requests) for board in boards]
        assert workflow.main("stop", args) == 0
        assert requests == [board.requests for board in boards]  # offline, idempotent stop


def test_stale_bag_or_no_devices_never_starts_recording(tmp_path, monkeypatch):
    with Recorder() as board:
        bag = configure(tmp_path, monkeypatch, [board.device])
        args = ["--root", str(tmp_path)]
        assert workflow.main("start", ["stale", *args]) == 2
        assert not board.requests
        with dummy_recorder(tmp_path, bag):
            monkeypatch.setattr(client, "discover", lambda *args: [])
            assert workflow.main("start", ["none", *args]) == 2
        assert not list(tmp_path.glob("encoder_*"))


def test_reused_pid_is_never_signalled(tmp_path, monkeypatch):
    make_bag(tmp_path / "robot")
    saved = {"pid": os.getpid(), "start_ticks": 1, "bag_path": str(tmp_path / "robot"), "kernel_boot_id": "old"}
    monkeypatch.setattr(workflow, "recorder_process", lambda pid: None)
    monkeypatch.setattr(os, "pidfd_open", lambda *args: pytest.fail("Must not open a replaced PID"))
    workflow.finalize_robot_bag(saved)


def test_real_rosbag_recorder_finalizes_without_stopping_other_processes(tmp_path):
    bag = tmp_path / "real_robot"
    environment = dict(os.environ, ROS_DOMAIN_ID="215", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    with (tmp_path / "recorder.log").open("w") as output:
        process = subprocess.Popen(["ros2", "bag", "record", "--storage", "mcap", "--output", str(bag),
                                    "/caster_session_test_fixture"], env=environment,
                                   stdout=output, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 15
            while not list(bag.glob("*.mcap")):
                assert process.poll() is None, (tmp_path / "recorder.log").read_text()
                assert time.monotonic() < deadline
                time.sleep(.1)
            saved = workflow.active_recorder(bag)
            assert saved["pid"] == process.pid
            workflow.finalize_robot_bag(saved)
            process.wait(timeout=5)
            assert (bag / "metadata.yaml").is_file()
        finally:
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=5)


def test_missing_clock_mapping_preserves_raw_logs_and_refuses_false_alignment(tmp_path, monkeypatch):
    with Recorder() as board:
        bag = configure(tmp_path, monkeypatch, [board.device])
        with dummy_recorder(tmp_path, bag):
            args = ["--root", str(tmp_path)]
            assert workflow.main("start", ["trial", *args]) == 0
            monkeypatch.setattr(client.Transport, "sync", lambda self, device: exchange(1000, identity={
                key: device[key] for key in ("v", "wheel_id", "node_id", "device_id", "boot_id")}))
            assert workflow.main("stop", args) == 2
        directory = tmp_path / "encoder_trial"
        assert list(directory.glob("*.bin"))
        assert (directory / "samples.csv").exists()
        assert not (directory / "bags").exists()
        assert json.loads((directory / workflow.STATE).read_text())["state"] == "needs_recovery"


def test_download_failure_is_retryable_without_restarting_boards(tmp_path, monkeypatch):
    with Recorder() as board:
        bag = configure(tmp_path, monkeypatch, [board.device])
        with dummy_recorder(tmp_path, bag):
            args = ["--root", str(tmp_path)]
            assert workflow.main("start", ["trial", *args]) == 0
            board.bad_crc = True
            assert workflow.main("stop", args) == 2
            assert not board.acknowledged
            board.bad_crc = False
            assert workflow.main("stop", args) == 0
            assert board.acknowledged
            assert board.requests.count("/api/start") == 1
            assert (tmp_path / "encoder_trial/bags/combined/metadata.yaml").exists()


def test_capacity_stop_still_downloads_and_merges_but_reports_incomplete(tmp_path, monkeypatch, capsys):
    with Recorder() as board:
        bag = configure(tmp_path, monkeypatch, [board.device])
        with dummy_recorder(tmp_path, bag):
            args = ["--root", str(tmp_path)]
            assert workflow.main("start", ["trial", *args]) == 0
            board.stop_reason = "capacity"
            assert workflow.main("stop", args) == 2
            directory = tmp_path / "encoder_trial"
            assert (directory / "bags/combined/metadata.yaml").exists()
            assert json.loads((directory / workflow.STATE).read_text())["state"] == "complete_with_errors"
            assert "recording incomplete" in capsys.readouterr().err


@pytest.mark.parametrize("old_state", ["stopped", "recording", "error"])
def test_start_discards_unread_onboard_logs_and_preserves_pi_files(tmp_path, monkeypatch, old_state):
    with Recorder() as board:
        board.state, board.run = old_state, "old-run"
        board.data, board.crc = make_log(board.device, board.run)
        board.log_boot = board.device["boot_id"]
        old_data = board.data
        saved = tmp_path / "previous_download.bin"
        saved.write_bytes(old_data)
        bag = configure(tmp_path, monkeypatch, [board.device])
        with dummy_recorder(tmp_path, bag):
            assert workflow.main("start", ["new-run", "--root", str(tmp_path)]) == 0
            assert board.state == "recording" and board.run == "new-run"
            assert board.data != old_data
            assert saved.read_bytes() == old_data
            assert "/api/log" not in board.requests and "/api/ack" not in board.requests
            prepare = next(body for route, body in board.bodies if route == "/api/prepare")
            assert prepare == {"run_id": "new-run", "discard_existing": True,
                               "expected_device_id": board.device["device_id"],
                               "expected_boot_id": board.device["boot_id"], "expected_run_id": "old-run"}
            if old_state == "recording":
                assert board.requests.index("/api/stop") < board.requests.index("/api/prepare")
            manifest = client.Session.load(tmp_path, "new-run").manifest
            assert manifest["discard_existing"] is True


def test_start_with_unsupported_firmware_leaves_every_board_untouched(tmp_path, monkeypatch):
    with Recorder(1) as first, Recorder(2) as second:
        first.state, first.run = "recording", "old-run"
        second.supports_discard_existing = False
        bag = configure(tmp_path, monkeypatch, [first.device, second.device])
        with dummy_recorder(tmp_path, bag):
            assert workflow.main("start", ["new-run", "--root", str(tmp_path)]) == 2
        assert first.bodies == second.bodies == []
        assert first.state == "recording" and first.run == "old-run"


def test_new_start_can_discard_previous_session_after_failed_download(tmp_path, monkeypatch):
    with Recorder() as board:
        bag = configure(tmp_path, monkeypatch, [board.device])
        args = ["--root", str(tmp_path)]
        with dummy_recorder(tmp_path, bag):
            assert workflow.main("start", ["old-run", *args]) == 0
            board.bad_crc = True
            assert workflow.main("stop", args) == 2
        assert not board.acknowledged and board.run == "old-run"
        old_manifest = (tmp_path / "encoder_old-run/manifest.json").read_bytes()
        new_bag = tmp_path / "robot_next"
        make_bag(new_bag)
        client.write_json(tmp_path / ".latest_hamr_bag.json", {"version": 1, "bag_path": str(new_bag)})
        board.bad_crc = False
        with dummy_recorder(tmp_path, new_bag):
            assert workflow.main("start", ["new-run", *args]) == 0
            assert board.run == "new-run" and board.state == "recording"
            assert (tmp_path / "encoder_old-run/manifest.json").read_bytes() == old_manifest
            assert workflow.main("stop", args) == 0
            assert (tmp_path / "encoder_new-run/bags/combined/metadata.yaml").exists()
