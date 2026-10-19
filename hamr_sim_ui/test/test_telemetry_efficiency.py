"""Display-only optimizations must preserve the full underlying telemetry."""

from collections import deque
import csv
import io
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hamr_sim_ui.telemetry import EncoderAdapter, JOINT_MAPPING, RosTelemetry


def telemetry_without_ros():
    telemetry = RosTelemetry.__new__(RosTelemetry)
    telemetry._lock = threading.RLock()
    telemetry._state = {"sim_time": 2., "joints": {"wheel": {"position": 1.}}, "casters": {}}
    telemetry._trail = deque([(1., 2.), (3., 4.)])
    telemetry._history = deque([{"sim_time": 1., "caster": .2}])
    telemetry._camera_stats = {}
    telemetry._clock_samples = deque()
    telemetry._video_processes = []
    telemetry.encoder = EncoderAdapter()
    telemetry.cameras_enabled = True
    telemetry._last_preview_wall = {}
    telemetry._preview_interval = 1. / 15
    telemetry._preview_cameras = frozenset({"closeup"})
    return telemetry


class Publisher:
    def __init__(self):
        self.subscribers = 0
        self.messages = []

    def get_subscription_count(self):
        return self.subscribers

    def publish(self, message):
        self.messages.append(message)


def joint_callback_telemetry():
    telemetry = telemetry_without_ros()
    telemetry._state["joints"] = {}
    telemetry._recording = None
    telemetry._last_history_ns = None
    telemetry._packet_pub = Publisher()
    telemetry._encoder_pubs = {node: Publisher() for node in JOINT_MAPPING["ball"]}
    telemetry._string = SimpleNamespace
    telemetry._joint_type = lambda: SimpleNamespace(header=SimpleNamespace(stamp=None))
    telemetry._notify = lambda: None
    telemetry.raw_messages = []
    telemetry._raw = lambda *args: telemetry.raw_messages.append(args)
    telemetry.metadata_boots = []
    telemetry._publish_metadata = lambda: telemetry.metadata_boots.append(telemetry.encoder.boot_id)
    return telemetry


def joint_message(second):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=second, nanosec=0)),
                           name=list(JOINT_MAPPING["ball"].values()),
                           position=[.5, -.3, 1.2], velocity=[.1, -.1, .2])


def test_unused_encoder_publications_skip_serialization_but_keep_sample_sequences(monkeypatch):
    telemetry = joint_callback_telemetry()
    monkeypatch.setattr(telemetry, "_json", lambda _: pytest.fail("Unused packet was serialized"))
    telemetry._joints(joint_message(2))
    assert telemetry.encoder.sequence == {1: 1, 2: 1, 3: 1}
    assert telemetry._packet_pub.messages == []
    assert all(publisher.messages == [] for publisher in telemetry._encoder_pubs.values())
    assert [message[0] for message in telemetry.raw_messages] == ["/joint_states"]
    previous_boot = telemetry.encoder.boot_id
    telemetry._joints(joint_message(1))
    assert telemetry.encoder.boot_id != previous_boot
    assert telemetry.metadata_boots == [telemetry.encoder.boot_id]
    assert telemetry.encoder.sequence == {1: 1, 2: 1, 3: 1}


def test_new_packet_and_single_joint_subscribers_receive_the_next_current_sample():
    telemetry = joint_callback_telemetry()
    telemetry._joints(joint_message(1))
    telemetry._packet_pub.subscribers = 1
    telemetry._encoder_pubs[2].subscribers = 1
    telemetry._joints(joint_message(2))
    packets = [json.loads(message.data)["packet"] for message in telemetry._packet_pub.messages]
    assert len(packets) == 3
    assert {packet["samples"][0]["seq"] for packet in packets} == {1}
    assert {packet["samples"][0]["t_us"] for packet in packets} == {2_000_000}
    assert [len(publisher.messages) for publisher in telemetry._encoder_pubs.values()] == [0, 1, 0]
    joint = telemetry._encoder_pubs[2].messages[0]
    assert joint.header.stamp.sec == 2 and joint.position == [-.3]


def test_recording_retains_all_encoder_outputs_even_without_subscribers():
    telemetry = joint_callback_telemetry()
    telemetry._joints(joint_message(1))
    joint_csv, packet_json = io.StringIO(), io.StringIO()
    telemetry._recording = SimpleNamespace(joints=csv.writer(joint_csv), packet_file=packet_json)
    telemetry._joints(joint_message(2))
    telemetry._joints(joint_message(3))
    packets = [json.loads(line)["packet"] for line in packet_json.getvalue().splitlines()]
    assert len(packets) == 6
    assert [packet["samples"][0]["seq"] for packet in packets] == [1, 1, 1, 2, 2, 2]
    assert len(telemetry._packet_pub.messages) == 6
    assert [len(publisher.messages) for publisher in telemetry._encoder_pubs.values()] == [2, 2, 2]
    assert len(list(csv.reader(io.StringIO(joint_csv.getvalue())))) == 6
    assert sum(message[0] == "/caster_encoder/packets" for message in telemetry.raw_messages) == 6


def test_snapshot_can_skip_display_history_without_discarding_samples():
    telemetry = telemetry_without_ros()
    compact = telemetry.snapshot(include_history=False, include_trail=False)
    assert "trail" not in compact and "history" not in compact
    assert compact["joints"]["wheel"]["position"] == 1.
    complete = telemetry.snapshot()
    assert complete["trail"] == [(1., 2.), (3., 4.)]
    assert complete["history"] == [{"sim_time": 1., "caster": .2}]
    complete["history"][0]["caster"] = 50.
    complete["joints"]["wheel"]["position"] = 50.
    assert telemetry.snapshot()["history"][0]["caster"] == .2
    assert telemetry.snapshot()["joints"]["wheel"]["position"] == 1.


@pytest.mark.parametrize("preserve_clock", [False, True])
def test_reset_clears_pose_and_encoder_segment_with_optional_clock_preservation(preserve_clock):
    telemetry = telemetry_without_ros()
    telemetry._state.update(x=1., y=2., yaw=.4, linear_speed=.1, odom_received=True,
                            joints_received=True)
    telemetry._recording = None
    telemetry._cached = {"/compa/odom": object()}
    published = []
    telemetry._publish_metadata = lambda: published.append(telemetry.encoder.boot_id)
    previous_boot = telemetry.encoder.boot_id
    telemetry.reset(preserve_clock=preserve_clock)
    state = telemetry.snapshot()
    assert state["sim_time"] == (2. if preserve_clock else 0.)
    assert state["x"] is None and state["y"] is None and state["yaw"] is None
    assert state["linear_speed"] is None
    assert not state["odom_received"] and not state["joints_received"]
    assert state["trail"] == [] and state["history"] == [] and state["joints"] == {}
    assert telemetry._cached == {}
    assert telemetry.encoder.boot_id != previous_boot
    assert published == [telemetry.encoder.boot_id]


@pytest.mark.parametrize("preserve_clock", [False, True])
def test_reset_refuses_active_recording_before_mutating_state(preserve_clock):
    telemetry = telemetry_without_ros()
    telemetry._recording = object()
    previous_boot = telemetry.encoder.boot_id
    with pytest.raises(RuntimeError, match="Stop recording"):
        telemetry.reset(preserve_clock=preserve_clock)
    assert telemetry.encoder.boot_id == previous_boot
    assert telemetry.snapshot()["sim_time"] == 2.
    assert telemetry.snapshot()["trail"] == [(1., 2.), (3., 4.)]


def test_only_visible_preview_is_converted_and_wall_rate_limited(monkeypatch):
    telemetry = telemetry_without_ros()
    delivered, converted = [], []
    telemetry._on_image = lambda camera, frame: delivered.append((camera, frame))
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0)))
    monkeypatch.setattr("hamr_sim_ui.telemetry.image_frame", lambda msg: converted.append(msg) or "pixels")
    wall = [10.]
    monkeypatch.setattr("hamr_sim_ui.telemetry.time.monotonic", lambda: wall[0])
    for _ in range(3):
        telemetry._image("closeup", message)
        telemetry._image("overview", message)
        wall[0] += .01
    assert delivered == [("closeup", "pixels")]
    assert len(converted) == 1
    assert telemetry.snapshot()["cameras"]["closeup"]["frames"] == 3
    assert telemetry.snapshot()["cameras"]["overview"]["frames"] == 3
    wall[0] += .1
    telemetry._image("closeup", message)
    assert len(delivered) == 2
    telemetry.cameras_enabled = False
    telemetry._image("closeup", message)
    assert len(delivered) == 2
    assert telemetry.snapshot()["cameras"]["closeup"]["frames"] == 4


@pytest.mark.parametrize("value", [0, -1., float("nan"), float("inf")])
def test_preview_rate_must_be_positive_and_finite(value):
    with pytest.raises(ValueError, match="preview_fps"):
        RosTelemetry(preview_fps=value)
