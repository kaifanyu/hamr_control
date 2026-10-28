import math
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from test_ordinary_history import make_board, write_run, load_history
from ordinary_caster.replay import frame_times, replay_frame, validate_replay


def test_replay_timestamps_joint_names_and_frame_bounds(tmp_path):
    history = load_history(write_run(tmp_path, make_board(), make_board("front_right")), require_clock=True)
    validate_replay(history)
    times = list(frame_times(history, 200))
    assert times[0] == history.start_ns
    assert times[-1] == history.end_ns
    assert all(a < b for a, b in zip(times, times[1:]))
    frame = replay_frame(history, times[1])
    assert frame["replay_time_ns"] == 1_000_000_000 + times[1] - times[0]
    assert set(frame["joint_names"]) == {f"front_{side}_caster_{channel}_joint"
                                         for side in ("left", "right") for channel in ("swivel", "wheel")}
    assert frame["complete_pose_available"]
    assert frame["accuracy_validated"] is False


def test_uncalibrated_preview_must_be_explicit(tmp_path):
    history = load_history(write_run(tmp_path, make_board(mapped=False, calibrated=False)), device_local=True)
    with pytest.raises(ValueError, match="no eligible"):
        validate_replay(history)
    validate_replay(history, True)
    frame = replay_frame(history, history.streams[0].times[1], True)
    assert frame["complete_pose_available"]
    assert all(item["uncalibrated_override"] for item in frame["channels"])


@pytest.mark.parametrize("fps", [0, -1, float("nan"), float("inf"), 1001])
def test_invalid_frame_rate(tmp_path, fps):
    history = load_history(write_run(tmp_path, make_board()))
    with pytest.raises(ValueError, match="Frame rate"):
        list(frame_times(history, fps))


def test_offline_preview_uses_same_history_and_reports_missing_data(tmp_path):
    pytest.importorskip("matplotlib")
    from ordinary_caster.render import render_history
    board = make_board()
    board[1]["samples"][2]["wheel"].update(flags=1 | 256, raw_count=None)
    history = load_history(write_run(tmp_path / "run", board))
    provenance = render_history(history, tmp_path / "render")
    assert provenance["geometry"] == "illustrative schematic"
    assert (tmp_path / "render/preview.png").stat().st_size > 1000
    assert (tmp_path / "render/trajectory.png").stat().st_size > 1000
    assert not provenance["accuracy_validated"]
    assert provenance["preview"]["complete_pose_available"]


def test_short_recording_ros_publishes_joints_clock_and_fault_status(tmp_path, monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    context = Context()
    rclpy.init(context=context)
    node = Node("ordinary_replay_test_subscriber", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    joints, statuses, clocks = [], [], []
    subscriptions = [node.create_subscription(JointState, "/ordinary_caster/replay/joint_states", joints.append, 100),
                     node.create_subscription(String, "/ordinary_caster/replay/status", lambda m: statuses.append(json.loads(m.data)), 100),
                     node.create_subscription(Clock, "/clock", clocks.append, 100)]
    run = write_run(tmp_path / "run", make_board(mapped=False, calibrated=False))
    script = Path(__file__).resolve().parents[1] / "scripts/replay_ordinary_caster_joints"
    process = subprocess.Popen([sys.executable, str(script), str(run), "--device-local", "--allow-uncalibrated",
                                "--start-delay", "1", "--exit-on-end", "--ros-args", "-r", "__node:=ordinary_replay_smoke"],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 8
        while process.poll() is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert process.poll() is not None, "Replay did not finish"
        output = process.communicate()[0]
        assert process.returncode == 0, output
        assert joints and clocks and statuses, output
        assert any("front_left_caster_wheel_joint" in message.name for message in joints)
        assert all(message.header.stamp.sec >= 1 for message in joints)
        assert any(status["end_of_history"] for status in statuses)
        assert all("tf_note" in status for status in statuses)
        assert all(status["time_basis"] == "device_local" for status in statuses)
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate(timeout=5)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown(context=context)
