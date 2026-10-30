"""Readiness failures, real HTTP GET-only probes, and isolated ROS reception."""
from contextlib import ExitStack
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace as NS

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
import hamr_readiness as check
from test_caster_recording_client import Recorder


def pose_message(stamp=100):
    return NS(header=NS(stamp=NS(sec=stamp, nanosec=0), frame_id="mocap"),
              pose=NS(position=NS(x=0., y=0., z=.3), orientation=NS(x=0., y=0., z=0., w=1.)))


def test_receipt_freshness_accepts_remote_clock_offset_but_rejects_repeated_stamps():
    stream = check.Stream(check.SPECS[1])
    stream.receive(pose_message(100), 50000.)
    stream.receive(pose_message(101), 50000.1)
    assert stream.report(50000.2, 1)["status"] == "PASS"
    stream.receive(pose_message(101), 50000.3)
    assert stream.report(50000.3, 1)["status"] == "FAIL"
    assert stream.last_good == 50000.1
    stream.receive(pose_message(102), 50000.4)
    assert stream.report(50000.4, 1)["status"] == "WARN"
    assert stream.report(50002., 1)["status"] == "FAIL"


@pytest.mark.parametrize("fault", ["zero_quaternion", "nan_position", "no_frame", "zero_stamp"])
def test_bad_pose_is_not_ready(fault):
    stream = check.Stream(check.SPECS[1])
    message = pose_message()
    if fault == "zero_quaternion":
        message.pose.orientation.w = 0.
    elif fault == "nan_position":
        message.pose.position.x = math.nan
    elif fault == "no_frame":
        message.header.frame_id = ""
    else:
        message.header.stamp.sec = 0
    stream.receive(message, 1.)
    assert stream.report(1., 1)["status"] == "FAIL"
    assert stream.good_count == 0


def test_stationary_zero_ticks_are_valid_and_conflicting_publishers_are_not():
    spec = next(s for s in check.SPECS if s.kind == "std_msgs/msg/Int32")
    stream = check.Stream(spec)
    stream.receive(NS(data=0), 1.)
    stream.receive(NS(data=0), 1.1)
    assert stream.report(1.2, 1)["status"] == "PASS"
    assert stream.report(1.2, 0)["status"] == "FAIL"
    assert stream.report(1.2, 2)["status"] == "FAIL"
    assert check.Stream(spec).report(1., 1)["status"] == "FAIL"


def replies(state="idle", count=0, when=1., run=None):
    values = []
    for wheel, node in check.EXPECTED:
        device = dict(v=1, wheel_id=wheel, node_id=node, boot_id="0123456789abcdef",
                      device_id=f"aa:bb:cc:dd:{0 if wheel == 'front_left' else 1:02x}:{node:02x}",
                      host="127.0.0.1", http_port=8080)
        status = dict(device, state=state, sample_count=count, sample_hz=200,
                      capacity_samples=20000, supports_discard_existing=True,
                      finalizing=False, run_id=run, acknowledged=False, diagnostics={"queue_drops": 0})
        values.append((device, status, None, when))
    return values


def pinned(values):
    return [item[0] for item in values]


def test_all_six_idle_boards_can_be_ready_without_claiming_received_sensor_samples():
    values = replies()
    state = check.Recorders(pinned(values))
    state.update(values)
    report = check.summarize(state.report(1.1), 10.)
    assert report["ready"]
    assert not report["all_six_recorder_counts_advancing"]
    assert "does not expose live encoder measurements" in report["sample_validation"]
    state.update(values[:-1])
    assert not check.summarize(state.report(1.2), 10.)["ready"]


def test_active_recording_requires_same_run_and_advancing_counts_on_every_board():
    values = replies("recording", 100, run="run_1")
    state = check.Recorders(pinned(values))
    state.update(values)
    assert not check.summarize(state.report(1.1), 10.)["ready"]
    state.update(replies("recording", 300, when=2., run="run_1"))
    report = check.summarize(state.report(2.1), 10.)
    assert report["ready"] and report["all_six_recorder_counts_advancing"]
    state.update(replies("recording", 300, when=3., run="run_1"))
    assert not check.summarize(state.report(3.1), 10.)["ready"]


@pytest.mark.parametrize("fault", ["different_run", "reboot", "idle", "prepared", "error",
                                   "old_firmware", "bad_capacity", "count_overflow", "bad_diagnostics",
                                   "finalizing", "http_error"])
def test_partial_or_failed_recordings_never_pass(fault):
    values = replies("recording", 100, run="run_1")
    state = check.Recorders(pinned(values))
    state.update(values)
    current = replies("recording", 300, when=2., run="run_1")
    device, status, _, now = current[-1]
    if fault == "different_run":
        status["run_id"] = "another_run"
    elif fault == "reboot":
        status["boot_id"] = "fedcba9876543210"
    elif fault in ("idle", "prepared", "error"):
        status["state"] = fault
    elif fault == "old_firmware":
        status["supports_discard_existing"] = False
    elif fault == "bad_capacity":
        status["capacity_samples"] = None
    elif fault == "count_overflow":
        status["sample_count"] = status["capacity_samples"] + 1
    elif fault == "bad_diagnostics":
        status["diagnostics"] = []
    elif fault == "finalizing":
        status["finalizing"] = True
    else:
        current[-1] = (device, None, "HTTP unavailable", now)
    state.update(current)
    assert not check.summarize(state.report(2.1), 10.)["ready"]


def test_storage_limit_stale_status_and_unpinned_inventory_are_visible():
    state = check.Recorders([], duration=100.)
    state.update(replies())
    assert all(r["status"] == "FAIL" for r in state.report(1.1))
    state.duration = None
    assert all(r["status"] == "WARN" for r in state.report(1.1))
    assert all(r["status"] == "FAIL" for r in state.report(10.))


def test_diagnostic_counters_report_deltas_not_historical_faults():
    values = replies("recording", 100, run="run_1")
    values[0][1]["diagnostics"]["queue_drops"] = 100
    state = check.Recorders(pinned(values))
    state.update(copy.deepcopy(values))
    values[0][1]["sample_count"] = 300
    state.update(copy.deepcopy(values))
    assert state.report(1.1)[0]["status"] == "PASS"
    values[0][1]["sample_count"] = 500
    values[0][1]["diagnostics"]["queue_drops"] = 101
    state.update(values)
    assert state.report(1.1)[0]["status"] == "WARN"


def test_probe_six_real_http_endpoints_is_get_only_and_checks_mac_inventory():
    with ExitStack() as stack:
        boards = [stack.enter_context(Recorder(node, wheel)) for wheel, node in check.EXPECTED]
        args = check.parser().parse_args(sum(([
            "--node", f"{b.device['wheel_id']}:{check.client.ROLES[b.device['node_id']]}@127.0.0.1:{b.device['http_port']}"
        ] for b in boards), []))
        inventory = [b.device for b in boards]
        result = check.probe_recorders(args, {}, inventory)
        assert len(result) == 6
        assert all(error is None for _, _, error, _ in result)
        assert all(b.requests == ["/api/status", "/api/status"] and not b.bodies for b in boards)
        wrong = copy.deepcopy(inventory)
        wrong[0]["device_id"] = "aa:bb:cc:dd:ff:ff"
        with pytest.raises(ValueError, match="MAC inventory mismatch"):
            check.probe_recorders(args, {}, wrong)


def test_robot_bag_requires_an_active_matching_process(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "bag_0.mcap").touch()
    (tmp_path / ".latest_hamr_bag.json").write_text(json.dumps({"version": 1, "bag_path": str(bag)}))
    assert check.bag_report(tmp_path, Path("latest"))["status"] == "FAIL"
    cli = tmp_path / "ros2"
    cli.write_text("import time\nprint('ready', flush=True)\ntime.sleep(30)\n")
    process = subprocess.Popen([sys.executable, str(cli), "bag", "record", "--output", str(bag)],
                               stdout=subprocess.PIPE, text=True)
    try:
        assert process.stdout.readline().strip() == "ready"
        report = check.bag_report(tmp_path, Path("latest"))
        assert report["status"] == "PASS" and report["pid"] == process.pid
    finally:
        process.terminate()
        process.wait(timeout=5)
        process.stdout.close()


def test_real_ros_subscriptions_receive_all_streams_and_detect_missing_caster_pose(tmp_path, monkeypatch, capsys):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message

    monkeypatch.setenv("ROS_DOMAIN_ID", str(180 + os.getpid() % 40))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    config = tmp_path / "recorders.yaml"
    config.write_text('{"version": 1, "devices": []}')
    pins = tmp_path / "casters.yaml"
    pins.write_text(json.dumps({f"/caster_encoder_{w}": {"ros__parameters": {
        "wheel_id": w, "configured_wheel_id": w}} for w in check.client.WHEELS}))
    args = check.parser().parse_args(["--timeout", "1.8", "--config", str(config),
                                      "--caster-config", str(pins), "--report", str(tmp_path/"report.json")])
    monkeypatch.setattr(check, "probe_recorders", lambda *args: replies(when=time.monotonic()))
    monkeypatch.setattr(check, "bag_report", lambda *args: check.row("Robot bag recorder", "PASS", "test"))
    context = Context()
    context.init(args=[])
    producer = Node("readiness_test_source", context=context)
    outputs = [(s, get_message(s.kind), producer.create_publisher(get_message(s.kind), s.topic, qos_profile_sensor_data))
               for s in check.SPECS]
    stopped = threading.Event()
    def publish():
        while not stopped.wait(.02):
            for spec, kind, publisher in outputs:
                msg = kind()
                if hasattr(msg, "header"):
                    msg.header.frame_id = "mocap"
                    msg.header.stamp = producer.get_clock().now().to_msg()
                    msg.header.stamp.sec -= 3600  # Deliberately unsynchronized remote clock.
                if spec.kind in ("nav_msgs/msg/Odometry", "geometry_msgs/msg/PoseStamped"):
                    pose = msg.pose.pose if spec.kind == "nav_msgs/msg/Odometry" else msg.pose
                    pose.orientation.w, pose.position.z = 1., .3
                if spec.kind == "std_msgs/msg/Float64MultiArray":
                    msg.data = [0.] * (17 if spec.topic == "/wheel_control/status" else 22)
                publisher.publish(msg)
    thread = threading.Thread(target=publish)
    thread.start()
    try:
        assert check.run(args) == 0
        report = json.loads(args.report.read_text())
        assert report["ready"]
        assert all(r["hz"] > 0 for r in report["checks"] if "hz" in r)
        stopped.set()
        thread.join()
        missing = next(v for v in outputs if v[0].topic == "/Left_Caster/pose")
        outputs.remove(missing)
        producer.destroy_publisher(missing[2])
        stopped.clear()
        thread = threading.Thread(target=publish)
        thread.start()
        assert check.run(args) == 1
        report = json.loads(args.report.read_text())
        failed = [r for r in report["checks"] if r["status"] == "FAIL"]
        assert [r["name"] for r in failed] == ["Vicon left caster"]
    finally:
        stopped.set()
        thread.join(timeout=5)
        producer.destroy_node()
        context.shutdown()
