"""Native 12-bit calibration, turn ambiguity, and live ROS capture contracts."""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ordinary_caster import calibration as cal

FIXTURE = ROOT / "test/fixtures/ordinary_caster/front_left_swivel.jsonl"
BASE = json.loads(FIXTURE.read_text().splitlines()[0])


def packet(count=4094, seq=0, t=0, wheel="front_left", invalid=False):
    result = copy.deepcopy(BASE)
    count = round(count) % cal.N
    bits = f"{count << 2:014b}"
    odd = 1 ^ (bits[::2].count("1") % 2)
    even = 1 ^ (bits[1::2].count("1") % 2)
    word = (odd << 15) | (even << 14) | (count << 2)
    result["wheel_id"] = wheel
    sample = result["samples"][0]
    sample.update(seq=seq, t_us=round(t * 1e6))
    # Valid raw phase with no verified rate, and a deliberately different firmware zero.
    sample["swivel"].update(raw_spi_word=word, position12=count, absolute_angle_rad=count * cal.TAU / cal.N,
                            calibrated_angle_rad=1.234, angle_rad=None, velocity_rad_s=None,
                            filtered_velocity_rad_s=None, flags=3 | (1 << 8))
    if invalid:
        sample["swivel"].update(raw_spi_word=0, position12=None, absolute_angle_rad=None,
                                calibrated_angle_rad=None, flags=1 | (1 << 7))
    return result


def history(path, turns=1, wheel="front_left", reverse=False, gap=False, invalid=False):
    with path.open("w") as stream:
        stream.write(json.dumps(dict(format="ordinary_swivel_turn_v1", wheel_id=wheel)) + "\n")
        for seq in range(2001):
            t = seq / 200
            if gap and 4 < t < 5:
                continue
            phase = min(1., max(0., (t - 3) / 4))
            if reverse:
                phase = min(1., max(0., (t - 3) / 2)) - min(1., max(0., (t - 5) / 2))
            p = packet(4094 + turns * phase * cal.N, seq, t, wheel, invalid and 900 <= seq < 903)
            stream.write(json.dumps(dict(envelope=dict(packet=p, receive_epoch_ns=0))) + "\n")


def observations(sign=1, wheel="front_left", zero=4094):
    poses = {}
    for index, ((name, _), offset) in enumerate(zip(cal.POSES, (0, 1024, 2048, 0))):
        rows = []
        for seq in range(401):
            rows.extend(cal.sample_rows(packet(zero + sign * offset, index * 1000 + seq,
                                                index * 10 + seq / 200, wheel)))
        poses[name] = cal.pose_summary(rows)
    return poses


@pytest.mark.parametrize("wheel", cal.protocol.WHEELS)
@pytest.mark.parametrize("turns,passed", [(1, True), (-1, True), (0, False), (2, False), (1.02, False)])
def test_turn_and_return(tmp_path, wheel, turns, passed):
    path = tmp_path / "turn.jsonl"
    history(path, turns=turns, wheel=wheel)
    report = cal.evaluate(path, 60)
    assert (report["result"] == "PASS") is passed
    assert report["signed_revolutions"] == pytest.approx(turns, abs=1 / cal.N)
    assert report["start_position12_mean"] == pytest.approx(4094)
    assert report["wheel_id"] == wheel


@pytest.mark.parametrize("wheel", cal.protocol.WHEELS)
@pytest.mark.parametrize("sign", (-1, 1))
def test_reference_wrap_and_physical_direction(wheel, sign):
    result = cal.solve_reference(wheel, observations(sign, wheel))
    assert result["zero_position12"] == 4094
    assert result["direction_sign"] == sign
    assert result["firmware_settings"] == dict(SWIVEL_ZERO_RAD=4094 * cal.TAU / cal.N,
                                               SWIVEL_SIGN=sign, SWIVEL_PHASE_REFERENCE_VALID=True)
    assert not result["accuracy_validated"] and not result["max_rpm_verified"]
    for phase in (0, math.pi / 2, math.pi, -math.pi / 2):
        raw = (4094 * cal.TAU / cal.N + sign * phase) % cal.TAU
        assert float(cal.wrap(sign * (raw - result["zero_absolute_angle_rad"]))) == pytest.approx(float(cal.wrap(phase)))


def test_stationary_mean_across_zero():
    rows = [cal.sample_rows(packet(4095 if seq % 2 else 0, seq, seq / 200))[0] for seq in range(401)]
    pose = cal.pose_summary(rows)
    assert min(pose["mean_position12"], cal.N - pose["mean_position12"]) < 1


@pytest.mark.parametrize("name,offset", [("quarter", 2048), ("opposite", 1024), ("return_zero", 100)])
def test_bad_reference_poses_rejected(name, offset):
    poses = observations()
    poses[name]["mean_raw_rad"] = (4094 + offset) % cal.N * cal.TAU / cal.N
    with pytest.raises(ValueError, match="check failed"):
        cal.solve_reference("front_left", poses)


def test_reverse_and_ambiguous_gap(tmp_path):
    path = tmp_path / "turn.jsonl"
    history(path, reverse=True)
    report = cal.evaluate(path, 60)
    assert report["result"] == "FAIL"
    assert report["maximum_backtrack_deg"] == pytest.approx(360)
    history(path, gap=True)
    with pytest.raises(ValueError, match="Ambiguous swivel gap"):
        cal.evaluate(path, 60)
    history(path)
    with pytest.raises(ValueError, match="exceeds supplied"):
        cal.evaluate(path, 1)


def test_invalid_preserved_report_exclusive_and_wrong_wheel(tmp_path):
    path, report = tmp_path / "turn.jsonl", tmp_path / "report.json"
    history(path, invalid=True)
    assert cal.main(["check", str(path), "--max-rpm", "60", "--report", str(report)]) == 0
    assert json.loads(report.read_text())["rejected_samples"] == 3
    assert cal.main(["check", str(path), "--max-rpm", "60", "--report", str(report)]) == 2
    lines = path.read_text().splitlines()
    lines[0] = json.dumps(dict(format="ordinary_swivel_turn_v1", wheel_id="front_right"))
    path.write_text("\n".join(lines) + "\n")
    with pytest.raises(ValueError, match="Expected identified front_right"):
        cal.evaluate(path, 60)


@pytest.mark.parametrize("field,value", [("boot_id", "fedcba9876543210"), ("device_id", "aa:bb:cc:dd:ee:12")])
def test_identity_changes_even_on_invalid_read(field, value):
    guard = cal.StreamGuard()
    guard.accept(packet())
    p = packet(seq=1, t=.005, invalid=True)
    p[field] = value
    with pytest.raises(ValueError, match="changed or rebooted"):
        guard.accept(p)


def test_duplicate_clock_and_raw_disagreement():
    guard = cal.StreamGuard()
    p = packet()
    assert len(guard.accept(p)) == 1
    assert guard.accept(p) == []
    with pytest.raises(ValueError, match="Conflicting duplicate"):
        guard.accept(packet(123))
    with pytest.raises(ValueError, match="clock is not increasing"):
        cal.ordered_valid(cal.sample_rows(p) + cal.sample_rows(packet(seq=1, t=0)))
    p["samples"][0]["swivel"]["absolute_angle_rad"] = 0
    with pytest.raises(ValueError, match="disagrees"):
        cal.sample_rows(p)
    p = packet()
    p["samples"][0]["swivel"]["raw_spi_word"] ^= 0x8000
    with pytest.raises(ValueError, match="parity"):
        cal.sample_rows(p)


def test_real_firmware_fixtures_and_wheel_channel_isolation():
    guard = cal.StreamGuard()
    for line in FIXTURE.read_text().splitlines():
        p = cal.swivel_packet(dict(packet=json.loads(line)), "front_left")
        guard.accept(p)
    rows, rejected = cal.ordered_valid(list(guard.seen.values()))
    assert rows and rejected
    p = json.loads((FIXTURE.parent / "front_left_wheel.jsonl").read_text().splitlines()[0])
    assert cal.swivel_packet(dict(packet=p), "front_left") is None
    legacy = json.loads((FIXTURE.parent / "front_left.jsonl").read_text().splitlines()[0])
    assert cal.sample_rows(cal.swivel_packet(dict(packet=legacy), "front_left"))[0]["node_id"] == 4


def test_reference_command_writes_reviewable_settings_and_preserves_history(tmp_path, monkeypatch):
    poses = observations(-1, "front_right")

    class Capture:
        def __init__(self, wheel, output):
            self.output = output

        def pose(self, name, duration):
            self.output.write(json.dumps(dict(phase=name)) + "\n")
            return poses[name]

        def close(self):
            pass

    monkeypatch.setattr(cal, "LiveCapture", Capture)
    monkeypatch.setattr("builtins.input", lambda prompt: "")
    args = ["zero", "--wheel", "front_right", "--output", str(tmp_path)]
    assert cal.main(args) == 0
    result = json.loads((tmp_path / "front_right_swivel_calibration.json").read_text())
    assert result["direction_sign"] == -1
    fragment = (tmp_path / "front_right_swivel_calibration.h").read_text()
    assert "#define SWIVEL_SIGN -1" in fragment
    assert "SWIVEL_MAX_RPM_VERIFIED" not in fragment
    assert cal.main(args) == 2


@pytest.mark.parametrize("wheel", cal.protocol.WHEELS)
def test_real_ros_capture(tmp_path, wheel):
    pytest.importorskip("rclpy")
    env = dict(os.environ, ROS_DOMAIN_ID=str(180 + os.getpid() % 40),
               ROS_LOCALHOST_ONLY="1", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    code = r'''
import json, sys, threading, time
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from test_ordinary_swivel_calibration import cal, packet
import rclpy
from rclpy.context import Context
from std_msgs.msg import String
ctx = Context()
rclpy.init(context=ctx)
node = rclpy.create_node("synthetic_swivel_turn", context=ctx)
wheel = sys.argv[3]
pub = node.create_publisher(String, f"/ordinary_caster/{wheel}/packets", 100)
stop = threading.Event()
def publish():
    started = time.monotonic()
    seq = 0
    while not stop.is_set():
        t = time.monotonic() - started
        phase = min(1., max(0., (t - 4.6) / 2))
        p = packet(4094 - phase * cal.N, seq, t, wheel)
        pub.publish(String(data=json.dumps(dict(packet=p, receive_epoch_ns=time.time_ns()))))
        seq += 1
        stop.wait(.005)
worker = threading.Thread(target=publish)
worker.start()
try:
    path = Path(sys.argv[2])
    assert cal.main(["record", str(path), "--seconds", "10", "--wheel", wheel]) == 0
    report = cal.evaluate(path, 60)
    assert report["result"] == "PASS", report
    assert abs(report["signed_revolutions"] + 1) < 1e-9, report
    with cal.open_history(path.with_suffix(".poses"), wheel, "ordinary_swivel_poses_v1") as output:
        capture = cal.LiveCapture(wheel, output)
        try:
            assert capture.pose("zero", 2)["mean_position12"] > 4093
        finally:
            capture.close()
finally:
    stop.set()
    worker.join()
    node.destroy_node()
    rclpy.shutdown(context=ctx)
'''
    result = subprocess.run([sys.executable, "-c", code, str(Path(__file__).parent),
                             str(tmp_path / "live.jsonl"), wheel],
                            env=env, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Starting raw position" in result.stdout
    assert "ONE full revolution" in result.stdout
