"""Known mechanical poses and dual-wheel rolling recordings, including failures."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import caster_center_workflow as workflow
from caster_center_calibration import load_calibration, orientation

EPOCH = 1_800_000_000_000_000_000
BOOT = "0123456789abcdef"
WHEELS = workflow.protocol.WHEELS


def device(wheel):
    return "02:00:00:00:00:01" if wheel == "front_left" else "02:00:00:00:00:02"


def row(wheel, count, seq, t):
    return dict(wheel_id=wheel, device_id=device(wheel), boot_id=BOOT,
                seq=seq, t_us=round(t * 1e6), receive_ns=EPOCH + round(t * 1e9),
                position14=int(round(count)) % workflow.N)


def reference(wheel, zero=16382, sign=1):
    observations = {}
    for (name, _), phase in zip(workflow.POSES, (0, 90, 180, 0)):
        rows = [row(wheel, zero + sign * phase * workflow.N / 360 + (i % 3 - 1), i, i * .01)
                for i in range(301)]
        observations[name] = workflow.pose_summary(rows, wheel)
    return workflow.solve_reference(wheel, observations)


@pytest.mark.parametrize("sign", [-1, 1])
def test_reference_wrap_sign_and_existing_monitor_compatibility(sign, tmp_path):
    cal = reference("front_left", sign=sign)
    assert cal["zero_position14"] == 16382
    assert cal["direction_sign"] == sign
    assert cal["physical_direction_verified"] is True
    assert cal["accuracy_validated"] is False
    path = tmp_path / "cal.json"
    workflow.write_new(path, cal)
    loaded = load_calibration(path)
    assert orientation((16382 + sign * 4096) % 16384, loaded)["orientation_deg"] == 90
    with pytest.raises(FileExistsError):
        workflow.write_new(path, cal)


@pytest.mark.parametrize("pose,shift,error", [
    ("quarter", 90, "Quarter-turn"),  # opposite pose is not a sign check
    ("opposite", 12, "Opposite-pose"),
    ("return_zero", 4, "Return-to-zero"),
])
def test_pose_checks_reject_bad_positions(pose, shift, error):
    observations = reference("front_left")["observations"]
    observations[pose]["mean_raw_rad"] += math.radians(shift)
    with pytest.raises(ValueError, match=error):
        workflow.solve_reference("front_left", observations)


def test_pose_must_be_stationary_and_not_replayed_duplicates():
    rows = [row("front_left", i * 4, i, i * .01) for i in range(301)]
    with pytest.raises(ValueError, match="moved"):
        workflow.pose_summary(rows, "front_left")
    with pytest.raises(ValueError, match="insufficient fresh"):
        workflow.pose_summary([rows[0]] * 301, "front_left")


def recording(reverse=False, offset=90):
    poses, centers = [], {w: [] for w in WHEELS}
    calibrations = {w: reference(w, zero=20 + i * 2000, sign=(-1 if i else 1)) for i, w in enumerate(WHEELS)}
    radii = {"front_left": .1, "front_right": .103}
    forward = np.array([math.cos(math.radians(offset)), math.sin(math.radians(offset))])
    for i, t in enumerate(np.arange(0, 26.01, .01)):
        # Five seconds stationary, 2.4 m travel in 16 s, five seconds stationary.
        s = np.clip(t - 5, 0, 16) * .15 * (-1 if reverse else 1)
        poses.append(dict(receive_ns=EPOCH + round(t * 1e9), xyz=[*(s * forward), .3], rpy=[0., 0., 0.]))
        for wheel in WHEELS:
            cal = calibrations[wheel]
            count = cal["zero_position14"] + cal["direction_sign"] * s / radii[wheel] * workflow.N / workflow.TAU
            sample = row(wheel, count, i, t)
            # Variable packet delay: deliberately unsuitable for instantaneous
            # velocity alignment, harmless during long stationary endpoints.
            sample["receive_ns"] += round((.03 + .01 * math.sin(t)) * 1e9)
            centers[wheel].append(sample)
    return poses, centers, calibrations


def fit(data, **kwargs):
    options = dict(max_rpm=120, forward_offset_deg=90)
    options.update(kwargs)
    return workflow.estimate_radii(*data, **options)


@pytest.mark.parametrize("reverse", [False, True])
def test_dual_radius_with_wraps_opposite_signs_and_transport_delay(reverse):
    result = fit(recording(reverse), direction="reverse" if reverse else "forward")
    assert result["signed_distance_m"] == pytest.approx(-2.4 if reverse else 2.4)
    assert result["wheels"]["front_left"]["radius_m"] == pytest.approx(.1, abs=2e-6)
    assert result["wheels"]["front_right"]["radius_m"] == pytest.approx(.103, abs=2e-6)
    assert result["wheels"]["front_left"]["revolutions"] > 3.8
    assert result["accuracy_validated"] is False


@pytest.mark.parametrize("fault,message", [
    ("gap", "ambiguous CENTER gap"),
    ("reboot", "reboot"),
    ("device", "device identity"),
    ("sign", "direction disagrees"),
    ("moving_finish", "not stationary"),
    ("duplicate", "conflicting duplicate"),
    ("clock", "clock is not increasing"),
    ("vicon_gap", "Vicon data"),
    ("moving_vicon", "Vicon finish is moving"),
    ("lateral", "Lateral"),
    ("yaw", "orientation changed"),
    ("nan", "Non-finite"),
])
def test_bad_recordings_do_not_produce_radius(fault, message):
    poses, centers, cals = recording()
    left = centers["front_left"]
    if fault == "gap":
        del left[800:850]
    elif fault == "reboot":
        left[1000]["boot_id"] = "1111111111111111"
    elif fault == "device":
        cals["front_left"]["device_id"] = "02:00:00:00:00:03"
    elif fault == "sign":
        cals["front_left"]["direction_sign"] *= -1
    elif fault == "moving_finish":
        for i, r in enumerate(left[-200:]):
            r["position14"] = (r["position14"] + i) % workflow.N
    elif fault == "duplicate":
        left.append(dict(left[1000], position14=(left[1000]["position14"] + 1) % workflow.N))
    elif fault == "clock":
        left[1000]["t_us"] = left[999]["t_us"]
    elif fault == "vicon_gap":
        del poses[700:750]
    elif fault == "moving_vicon":
        for i, p in enumerate(poses[-200:]):
            p["xyz"][1] += i * .0002
    elif fault == "lateral":
        poses[1000]["xyz"][0] += .03
    elif fault == "yaw":
        poses[1000]["rpy"][2] = .05
    elif fault == "nan":
        poses[1000]["xyz"][0] = float("nan")
    with pytest.raises(ValueError, match=message):
        fit((poses, centers, cals))


def test_wrong_forward_frame_and_rate_bound_are_rejected():
    with pytest.raises(ValueError, match="forward offset"):
        fit(recording(), forward_offset_deg=0)
    with pytest.raises(ValueError, match="exceeds supplied maximum"):
        fit(recording(), max_rpm=1)


def packet(wheel, sample, invalid=False):
    value = dict(sample)
    value.pop("receive_ns")
    for key in ("wheel_id", "device_id", "boot_id"):
        value.pop(key)
    value.update(raw_count=None, raw_spi_word=sample["position14"],
                 absolute_angle_rad=sample["position14"] * workflow.TAU / workflow.N,
                 calibrated_alpha_rad=.123, angle_rad=None, velocity_rad_s=None,
                 filtered_velocity_rad_s=None, index_event_count=0,
                 index_timestamp_us=0, index_raw_count=None,
                 flags=1 if invalid else 3)
    return dict(v=2, type="samples", wheel_id=wheel, device_id=device(wheel),
                node_id=3, boot_id=BOOT, samples=[value],
                diagnostics=dict.fromkeys(workflow.protocol.DIAGNOSTIC_FIELDS, 0))


def test_raw_counts_used_without_firmware_zero_or_unwrap_flags():
    sample = row("front_left", 100, 1, .01)
    envelope = {"packet": packet("front_left", sample)}
    result = workflow.center_rows(envelope, "front_left", sample["receive_ns"])
    assert result[0]["position14"] == 100
    envelope["packet"]["samples"][0]["flags"] |= 1 << 7
    assert workflow.center_rows(envelope, "front_left", sample["receive_ns"]) == []
    with pytest.raises(ValueError, match="Expected identified"):
        workflow.center_rows(envelope, "front_right", sample["receive_ns"])


def test_real_mcap_reader_and_cli(tmp_path):
    bag = pytest.importorskip("rosbag2_py")
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    if "mcap" not in bag.get_registered_writers():
        pytest.skip("MCAP writer unavailable")
    poses, centers, cals = recording()
    bag_path = tmp_path / "straight"
    writer = bag.SequentialWriter()
    writer.open(bag.StorageOptions(uri=str(bag_path), storage_id="mcap"), bag.ConverterOptions("", ""))
    vtopic = "/HAMR_base/odom"
    topic_types = {vtopic: "nav_msgs/msg/Odometry", **{f"/caster_encoder/{w}/packets": "std_msgs/msg/String" for w in WHEELS}}
    for i, (name, type_name) in enumerate(topic_types.items()):
        writer.create_topic(bag.TopicMetadata(id=i, name=name, type=type_name, serialization_format="cdr"))
    messages = []
    for p in poses:
        msg = Odometry()
        ns = p["receive_ns"]
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        msg.header.frame_id = "world"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = p["xyz"]
        msg.pose.pose.orientation.w = 1.
        messages.append((ns, vtopic, serialize_message(msg)))
    for wheel in WHEELS:
        for r in centers[wheel]:
            msg = String(data=json.dumps({"packet": packet(wheel, r)}))
            messages.append((r["receive_ns"], f"/caster_encoder/{wheel}/packets", serialize_message(msg)))
    for ns, topic, data in sorted(messages):
        writer.write(topic, data, ns)
    del writer
    for wheel, cal in cals.items():
        workflow.write_new(tmp_path / f"{wheel}_center_calibration.json", cal)
    report = tmp_path / "radius.json"
    assert workflow.main(["radius", "--bag", str(bag_path), "--calibration-dir", str(tmp_path),
                          "--output", str(report), "--max-rpm", "120", "--vicon-forward-offset-deg", "90"]) == 0
    result = json.loads(report.read_text())
    assert result["wheels"]["front_right"]["radius_m"] == pytest.approx(.103, abs=2e-6)
    assert workflow.main(["radius", "--bag", str(bag_path), "--calibration-dir", str(tmp_path),
                          "--output", str(report), "--max-rpm", "120", "--vicon-forward-offset-deg", "90"]) == 2


def test_guided_both_wheels_creates_independent_files(tmp_path, monkeypatch):
    calls = []
    data = {w: reference(w, sign=1 if w == WHEELS[0] else -1)["observations"] for w in WHEELS}
    def capture(wheel, duration, timeout):
        names = [name for name, _ in workflow.POSES]
        result = data[wheel][names[len(calls) % 4]]
        calls.append(wheel)
        return result
    monkeypatch.setattr(workflow, "capture_live", capture)
    monkeypatch.setattr("builtins.input", lambda prompt: "")
    assert workflow.main(["zero", "--output", str(tmp_path)]) == 0
    assert calls == [WHEELS[0]] * 4 + [WHEELS[1]] * 4
    assert load_calibration(tmp_path / "front_right_center_calibration.json")["direction_sign"] == -1


def test_live_capture_with_isolated_ros_publisher():
    pytest.importorskip("rclpy")
    env = dict(os.environ, ROS_DOMAIN_ID=str(180 + os.getpid() % 40),
               ROS_LOCALHOST_ONLY="1", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    code = r'''
import json
import sys
import threading
import time
sys.path.insert(0, sys.argv[1])
from test_caster_center_workflow import workflow, packet, row
import rclpy
from rclpy.context import Context
from std_msgs.msg import String
ctx = Context()
rclpy.init(context=ctx)
node = rclpy.create_node("synthetic_center_source", context=ctx)
pub = node.create_publisher(String, "/caster_encoder/front_left/packets", 10)
stop = threading.Event()
def publish():
    started = time.monotonic()
    seq = 0
    while not stop.is_set():
        sample = row("front_left", 16383, seq, time.monotonic() - started)
        envelope = dict(receive_epoch_ns=time.time_ns(), packet=packet("front_left", sample))
        pub.publish(String(data=json.dumps(envelope)))
        seq += 1
        stop.wait(.01)
thread = threading.Thread(target=publish)
thread.start()
try:
    result = workflow.capture_live("front_left", duration=2., timeout=3.)
    assert result["sample_count"] >= 100
    assert abs(result["mean_raw_rad"] * workflow.N / workflow.TAU - 16383) < 1e-6
finally:
    stop.set()
    thread.join()
    node.destroy_node()
    rclpy.shutdown(context=ctx)
'''
    result = subprocess.run([sys.executable, "-c", code, str(Path(__file__).parent)],
                            env=env, capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
