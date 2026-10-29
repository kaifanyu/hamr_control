"""Validate one-turn history, not just matching absolute endpoints."""
import importlib.machinery
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from test_caster_center_workflow import SCRIPTS, packet, row

loader = importlib.machinery.SourceFileLoader("center_turn_check", str(SCRIPTS / "center_turn_check"))
spec = importlib.util.spec_from_loader(loader.name, loader)
turn = importlib.util.module_from_spec(spec)
loader.exec_module(turn)


def history(path, revolutions=1, return_back=False, gap=False, invalid=False, wheel="front_left"):
    with path.open("w") as stream:
        stream.write(json.dumps(dict(format="center_turn_v1", wheel_id=wheel)) + "\n")
        for seq, t in enumerate(np.arange(0, 12.01, .01)):
            if gap and 420 <= seq <= 490:
                continue
            phase = np.clip((t - 3) / 4, 0, 1) * revolutions
            if return_back:
                phase = np.clip((t - 3) / 2, 0, 1) - np.clip((t - 5) / 2, 0, 1)
            sample = row(wheel, 16382 + phase * turn.N, seq, t)
            envelope = dict(receive_epoch_ns=sample["receive_ns"],
                            packet=packet(wheel, sample, invalid=invalid and 500 <= seq < 503))
            stream.write(json.dumps(dict(envelope=envelope)) + "\n")


@pytest.mark.parametrize("revs,passed", [(1, True), (-1, True), (0, False), (2, False), (1.05, False)])
@pytest.mark.parametrize("wheel", ["front_left", "front_right"])
def test_rotation_count_and_return_error(tmp_path, revs, passed, wheel):
    path = tmp_path / "turn.jsonl"
    history(path, revolutions=revs, wheel=wheel)
    result = turn.evaluate(path, 60)
    assert (result["result"] == "PASS") is passed
    assert result["signed_revolutions"] == pytest.approx(revs, abs=1/16384)
    assert result["start_position14_mean"] == pytest.approx(16382)
    assert result["wheel_id"] == wheel


def test_recording_header_cannot_relabel_other_wheel(tmp_path):
    path = tmp_path / "turn.jsonl"
    history(path, wheel="front_right")
    lines = path.read_text().splitlines()
    lines[0] = json.dumps(dict(format="center_turn_v1", wheel_id="front_left"))
    path.write_text("\n".join(lines) + "\n")
    with pytest.raises(ValueError, match="Expected identified front_left"):
        turn.evaluate(path, 60)


def test_turn_then_reverse_back_is_not_one_monotonic_turn(tmp_path):
    path = tmp_path / "turn.jsonl"
    history(path, return_back=True)
    result = turn.evaluate(path, 60)
    assert result["result"] == "FAIL"
    assert result["return_error_deg"] == pytest.approx(0)
    assert result["maximum_backtrack_deg"] == pytest.approx(360)


def test_ambiguous_gap_rejected_even_when_endpoints_match(tmp_path):
    path = tmp_path / "turn.jsonl"
    history(path, gap=True)
    with pytest.raises(ValueError, match="ambiguous CENTER gap"):
        turn.evaluate(path, 60)


def test_invalid_samples_preserved_but_not_used_and_report_is_new(tmp_path):
    path = tmp_path / "turn.jsonl"
    history(path, invalid=True)
    report = tmp_path / "report.json"
    assert turn.main(["check", str(path), "--max-rpm", "60", "--report", str(report)]) == 0
    result = json.loads(report.read_text())
    assert result["rejected_samples"] == 3
    assert turn.main(["check", str(path), "--max-rpm", "60", "--report", str(report)]) == 2


@pytest.mark.parametrize("wheel", ["front_left", "front_right"])
def test_real_ros_record_and_evaluate(tmp_path, wheel):
    pytest.importorskip("rclpy")
    env = dict(os.environ, ROS_DOMAIN_ID=str(180 + os.getpid() % 40),
               ROS_LOCALHOST_ONLY="1", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    code = r'''
import json
import sys
import threading
import time
sys.path.insert(0, sys.argv[1])
from test_center_turn_check import turn, packet, row
from pathlib import Path
import rclpy
from rclpy.context import Context
from std_msgs.msg import String
ctx = Context()
rclpy.init(context=ctx)
node = rclpy.create_node("synthetic_center_turn", context=ctx)
wheel = sys.argv[3]
pub = node.create_publisher(String, f"/caster_encoder/{wheel}/packets", 10)
stop = threading.Event()
def publish():
    started = time.monotonic()
    seq = 0
    while not stop.is_set():
        t = time.monotonic() - started
        phase = min(1., max(0., (t - 3.6) / 2))
        sample = row(wheel, 14000 + phase * turn.N, seq, t)
        envelope = dict(receive_epoch_ns=time.time_ns(), packet=packet(wheel, sample))
        pub.publish(String(data=json.dumps(envelope)))
        seq += 1
        stop.wait(.01)
worker = threading.Thread(target=publish)
worker.start()
try:
    path = Path(sys.argv[2])
    assert turn.main(["record", str(path), "--seconds", "10", "--wheel", wheel]) == 0
    report = turn.evaluate(path, 60)
    assert report["result"] == "PASS", report
    assert report["wheel_id"] == wheel
finally:
    stop.set()
    worker.join()
    node.destroy_node()
    rclpy.shutdown(context=ctx)
'''
    result = subprocess.run([sys.executable, "-c", code, str(Path(__file__).parent), str(tmp_path / "live.jsonl"), wheel],
                            env=env, capture_output=True, text=True, timeout=25)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Starting raw position" in result.stdout
    assert "ONE full revolution" in result.stdout
