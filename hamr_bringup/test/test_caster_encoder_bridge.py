"""Exercise the actual wire receiver and bridge contract without ROS/hardware."""

import importlib.machinery
import importlib.util
import json
from pathlib import Path
import socket
import sys
import time
from types import SimpleNamespace

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
SPEC = importlib.util.spec_from_file_location(
    "caster_encoder_protocol_under_test", SCRIPTS / "caster_encoder_protocol.py")
protocol = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(protocol)
BOOT = "0123456789abcdef"
NEXT_BOOT = "fedcba9876543210"


def packet(*sequences, node_id=1, boot_id=BOOT):
    return {
        "v": 1, "type": "samples", "node_id": node_id, "boot_id": boot_id,
        "samples": [{
            "seq": sequence, "t_us": 250000 + sequence * 5000,
            "raw_count": "9223372036854775807", "raw_spi_word": None,
            "position14": None, "angle_rad": sequence * 0.01,
            "absolute_angle_rad": None, "calibrated_alpha_rad": None,
            "velocity_rad_s": 2.0, "filtered_velocity_rad_s": 1.9,
            "index_event_count": 0, "index_timestamp_us": 0,
            "index_raw_count": "-9223372036854775808", "flags": 1 | 2 | 16 | 64,
        } for sequence in (sequences or (0,))],
        "diagnostics": {name: 0 for name in protocol.DIAGNOSTIC_FIELDS},
    }


def metadata(boot_id=BOOT):
    return {
        "v": 1, "type": "metadata", "node_id": 1, "boot_id": boot_id,
        "role": "H1", "firmware_version": "0.1.0", "idf_version": "v5.4.2",
        "encoder_ppr": 2048, "ppr_verified": False, "custom": {"retain": True},
    }


def encoded(value):
    return json.dumps(value).encode("utf-8")


def receive(state, value, monotonic_ns=1000000000):
    return state.receive(encoded(value), ("192.168.1.11", 43210),
                         1791480000123456789, monotonic_ns, 1791480000123456791)


def test_wire_values_envelope_and_nanosecond_precision_are_preserved():
    value = packet(0, 1)
    value["samples"][1]["angle_rad"] = None
    value["samples"][1]["flags"] = 1
    envelope, readings, _ = receive(protocol.TelemetryState(), value)
    assert envelope["packet"] == value
    assert envelope["receive_epoch_ns"] == 1791480000123456789
    assert envelope["receive_ros_ns"] == 1791480000123456791
    assert envelope["source_ip"] == "192.168.1.11"
    assert readings[0]["name"] == "caster_h1_rotation"
    assert len(readings) == 1
    assert set(envelope) == {
        "receive_epoch_ns", "receive_monotonic_ns", "source_ip", "source_port", "packet", "receive_ros_ns",
    }
    assert protocol.decode_packet(encoded(value)) == value


@pytest.mark.parametrize("field,value", [
    ("raw_count", 9007199254740993), ("raw_count", "9223372036854775808"),
    ("raw_count", "+1"), ("index_raw_count", "01"),
    ("seq", -1), ("seq", True), ("t_us", 1 << 63), ("flags", 1 << 13),
    ("raw_spi_word", 65536), ("position14", 16384),
    ("angle_rad", float("nan")), ("angle_rad", float("inf")),
    ("velocity_rad_s", True), ("calibrated_alpha_rad", "0.0"),
])
def test_malformed_sample_rejected(field, value):
    source = packet()
    source["samples"][0][field] = value
    state = protocol.TelemetryState()
    with pytest.raises(ValueError):
        receive(state, source)
    assert state.invalid_packets == 1
    assert state.valid_packets == 0


@pytest.mark.parametrize("data", [
    b"", b"\xff", b"[]", b"null", b'{"v":1,"v":1}',
    b"[" * 2000 + b"]" * 2000, b"x" * 65508,
])
def test_invalid_datagram_does_not_poison_next_receive(data):
    state = protocol.TelemetryState()
    with pytest.raises(ValueError):
        state.receive(data, ("127.0.0.1", 1), 1, 1)
    assert receive(state, packet())[0]["packet"] == packet()


def test_unknown_metadata_fields_retained_and_nested_payload_rejected():
    assert protocol.decode_packet(encoded(metadata())) == metadata()
    value = metadata()
    extra = []
    for _ in range(20):
        extra = [extra]
    value["extra"] = extra
    with pytest.raises(ValueError):
        protocol.decode_packet(encoded(value))


def test_loss_duplicates_late_arrivals_and_capture_start_accounting():
    state = protocol.TelemetryState()
    assert len(receive(state, packet(100, 102))[1]) == 2
    # Preserve late/duplicate raw packets, but never move a live display backward.
    assert receive(state, packet(101))[1] == []
    assert receive(state, packet(102))[1] == []
    report = state.diagnostics_snapshot(1000000000)["streams"][0]
    assert report["unique_samples"] == 3
    assert report["missing_samples"] == 100  # upstream since-boot semantics
    assert report["missing_samples_in_recording"] == 0
    assert report["duplicate_samples"] == 1
    assert report["reordered_samples"] == report["recovered_samples"] == 1
    receive(state, packet(105))
    assert state.diagnostics_snapshot(1000000000)["streams"][0]["missing_samples_in_recording"] == 2


def test_reboot_invalidates_metadata_and_old_packets_cannot_reactivate_it():
    state = protocol.TelemetryState()
    receive(state, metadata())
    receive(state, packet(200))
    _, readings, changed = receive(state, packet(0, boot_id=NEXT_BOOT))
    assert changed and readings[0]["seq"] == 0
    assert state.metadata_snapshot()["nodes"] == {}
    assert receive(state, metadata())[2] is False
    assert receive(state, packet(201))[1] == []
    assert state.metadata_snapshot()["nodes"] == {}
    receive(state, metadata(NEXT_BOOT))
    assert state.metadata_snapshot()["nodes"]["1"]["boot_id"] == NEXT_BOOT
    streams = state.diagnostics_snapshot(1000000000)["streams"]
    assert len(streams) == 2
    assert sum(s["current_boot"] for s in streams) == 1


def test_center_wrapped_angle_survives_untrusted_unwrap():
    value = packet(node_id=3)
    value["samples"][0].update(
        raw_count=None, raw_spi_word=0x61AB, position14=8619,
        absolute_angle_rad=3.305, calibrated_alpha_rad=3.105,
        angle_rad=None, velocity_rad_s=None, filtered_velocity_rad_s=None,
        flags=1 | 2 | 256,
    )
    reading = receive(protocol.TelemetryState(), value)[1][0]
    assert reading["name"] == "caster_center_alpha"
    assert reading["position"] == 3.105 and reading["velocity"] is None
    value["samples"][0]["flags"] |= protocol.PARITY_ERROR
    assert receive(protocol.TelemetryState(), value)[1] == []


@pytest.mark.parametrize("flags", [0, 1, 2, 1 | 2, 1 | 2 | 16 | 256, 1 | 2 | 16 | 4096])
def test_bad_hemisphere_samples_never_publish_display_values(flags):
    value = packet()
    value["samples"][0]["flags"] = flags
    envelope, readings, _ = receive(protocol.TelemetryState(), value)
    assert envelope["packet"] == value
    assert readings == []


def test_velocity_requires_its_own_validity_flag():
    value = packet()
    value["samples"][0]["flags"] &= ~protocol.VELOCITY_VALID
    assert receive(protocol.TelemetryState(), value)[1][0]["velocity"] is None


def test_expected_never_seen_and_stale_nodes_are_explicit():
    state = protocol.TelemetryState()
    receive(state, metadata())
    report = state.diagnostics_snapshot(1000000000)
    assert all(n["never_seen"] and n["stale"] for n in report["nodes"])
    receive(state, packet())
    report = state.diagnostics_snapshot(1500000000)
    assert not report["nodes"][0]["stale"]
    assert report["nodes"][0]["sample_age_s"] == 0.5
    assert report["nodes"][1]["never_seen"]
    assert state.diagnostics_snapshot(3000000000)["nodes"][0]["stale"]


def test_duplicates_and_late_samples_do_not_hide_stalled_acquisition_or_regress_counters():
    state = protocol.TelemetryState()
    value = packet(100, 102)
    value["diagnostics"]["radio_failures"] = 20
    receive(state, value, monotonic_ns=1000000000)
    # These packets arrive much later, with older cumulative diagnostics.
    receive(state, packet(102), monotonic_ns=3000000000)
    receive(state, packet(101), monotonic_ns=3000000001)
    report = state.diagnostics_snapshot(3000000001)
    assert report["nodes"][0]["stale"]
    assert report["nodes"][0]["sample_age_s"] > 2.0
    assert report["streams"][0]["firmware_diagnostics"]["radio_failures"] == 20
    assert report["streams"][0]["missing_samples_in_recording"] == 0
    receive(state, packet(103), monotonic_ns=3000000002)
    assert not state.diagnostics_snapshot(3000000002)["nodes"][0]["stale"]


def test_clock_mapper_is_explicit_per_boot_and_preserves_epoch_precision():
    document = {"version": 1, "mappings": [{
        "node_id": 1, "boot_id": BOOT, "calibrated": True,
        "slope_ns_per_us": "1000.125", "offset_ns": 1791480000123456789,
        "valid_from_us": 250000, "valid_to_us": 1000000,
    }]}
    mapper = protocol.ClockMapper(document)
    assert mapper.map(1, BOOT, 250000) == 1791480000373488039
    assert mapper.map(1, NEXT_BOOT, 250000) is None
    assert mapper.map(1, BOOT, 1000001) is None
    assert protocol.ClockMapper().map(1, BOOT, 250000) is None


def test_actual_udp_round_trip_exclusive_bind_and_bounded_drain():
    receiver = protocol.UDPReceiver("127.0.0.1", 0)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        address = receiver.socket.getsockname()
        with pytest.raises(RuntimeError, match="Cannot listen on UDP"):
            protocol.UDPReceiver(*address)
        sender.sendto(encoded(packet(0, 1)), address)
        sender.sendto(b"bad", address)
        received = []
        deadline = time.monotonic() + 2.0
        while len(received) < 2 and time.monotonic() < deadline:
            batch = list(receiver.drain(max_datagrams=1))
            assert len(batch) <= 1
            received.extend(batch)
            if not batch:
                time.sleep(0.001)
        assert len(received) == 2
        assert protocol.decode_packet(received[0][0]) == packet(0, 1)
        assert received[0][2] > 0 and received[0][3] > 0
        assert list(receiver.drain()) == []
    finally:
        sender.close()
        receiver.close()


def load_bridge_with_fake_ros(monkeypatch, use_sim_time=False, parameters=None):
    class FakePublisher:
        def __init__(self, topic, qos):
            self.topic, self.qos, self.messages = topic, qos, []

        def publish(self, message):
            self.messages.append(message)

    class FakeNode:
        def __init__(self, name):
            self.parameters = {"use_sim_time": use_sim_time, **(parameters or {})}
            self.publishers = {}
            self.descriptors = {}
            self.warnings = []

        def get_parameter(self, name):
            return SimpleNamespace(value=self.parameters[name])

        def declare_parameter(self, name, default, descriptor=None):
            self.parameters.setdefault(name, default)
            self.descriptors[name] = descriptor

        def set_descriptor(self, name, descriptor):
            self.descriptors[name] = descriptor

        def create_publisher(self, message_type, topic, qos):
            publisher = FakePublisher(topic, qos)
            self.publishers[topic] = publisher
            return publisher

        def create_timer(self, interval, callback):
            return None

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(
                nanoseconds=1234567890123456789, to_msg=lambda: "receipt_stamp"))

        def get_logger(self):
            return SimpleNamespace(info=lambda text: None, warning=self.warnings.append)

        def destroy_node(self):
            return True

    class FakeReceiver:
        def __init__(self, address, port):
            self.queue = []

        def drain(self):
            queue, self.queue = self.queue, []
            return iter(queue)

        def close(self):
            pass

    class FakeJointState:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None)
            self.name, self.position, self.velocity = [], [], []

    monkeypatch.setitem(sys.modules, "rcl_interfaces.msg", SimpleNamespace(
        ParameterDescriptor=SimpleNamespace, ParameterType=SimpleNamespace(PARAMETER_BOOL=1)))
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "rclpy.node", SimpleNamespace(Node=FakeNode))
    monkeypatch.setitem(sys.modules, "rclpy.qos", SimpleNamespace(
        QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs),
        DurabilityPolicy=SimpleNamespace(VOLATILE="volatile", TRANSIENT_LOCAL="transient_local"),
        HistoryPolicy=SimpleNamespace(KEEP_LAST="keep_last"),
        ReliabilityPolicy=SimpleNamespace(RELIABLE="reliable"),
    ))
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", SimpleNamespace(JointState=FakeJointState))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(String=SimpleNamespace))
    monkeypatch.setitem(sys.modules, "caster_encoder_protocol", protocol)
    loader = importlib.machinery.SourceFileLoader("caster_encoder_bridge_under_test", str(SCRIPTS / "caster_encoder_bridge"))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "UDPReceiver", FakeReceiver)
    return module


def test_actual_bridge_publishes_receipt_stamps_qos_and_all_raw_packets(monkeypatch):
    module = load_bridge_with_fake_ros(monkeypatch, parameters={"wheel_id": "", "legacy_mode": True})
    node = module.CasterEncoderBridge()
    node.receiver.queue = [
        (encoded(packet(0, 1)), ("192.168.1.11", 1234), 11, 12),
        (encoded(packet(0)), ("192.168.1.11", 1234), 13, 14),
        (b"malformed", ("192.168.1.11", 1234), 15, 16),
    ]
    node.receive()
    assert len(node.packets.messages) == 2
    assert len(node.joints[1].messages) == 2  # duplicate is raw-only
    assert node.joints[1].messages[0].header.stamp == "receipt_stamp"
    envelope = json.loads(node.packets.messages[0].data)
    assert envelope["receive_ros_ns"] == 1234567890123456789
    assert envelope["packet"]["samples"][0]["t_us"] == 250000
    assert node.packets.qos.depth >= 1000
    assert node.packets.qos.reliability == "reliable"
    assert node.packets.qos.durability == "volatile"
    assert node.metadata.qos.durability == "transient_local" and node.metadata.qos.depth == 1
    node.publish_diagnostics()
    assert json.loads(node.diagnostics.messages[-1].data)["invalid_packets"] == 1
    assert node.warnings


def test_bridge_rejects_simulation_clock_at_startup_and_runtime(monkeypatch):
    module = load_bridge_with_fake_ros(monkeypatch, use_sim_time=True)
    with pytest.raises(RuntimeError, match="use_sim_time=false"):
        module.CasterEncoderBridge()
    module = load_bridge_with_fake_ros(monkeypatch)
    node = module.CasterEncoderBridge()
    node.parameters["use_sim_time"] = True
    with pytest.raises(RuntimeError, match="use_sim_time=true"):
        node.receive()


def identified(value, wheel="front_left", device=None):
    """Give a wire fixture the stable identity of one of six physical boards."""
    value = json.loads(json.dumps(value))
    value.update(v=2, wheel_id=wheel,
                 device_id=device or f"02:00:00:00:{1 if wheel == 'front_left' else 2:02x}:{value['node_id']:02x}")
    return value


def metadata_for(node_id, wheel="front_left", boot_id=BOOT):
    value = metadata(boot_id)
    value.update(node_id=node_id, role=protocol.ROLES[node_id])
    return identified(value, wheel)


def test_all_six_shared_boot_ids_keep_metadata_loss_and_readings_independent():
    # Deliberately reuse boot IDs and sequence ranges: only full identity separates these.
    state = protocol.TelemetryState()
    names = set()
    for wheel in protocol.WHEELS:
        for node in protocol.ROLES:
            receive(state, metadata_for(node, wheel))
            value = identified(packet(10, 12, node_id=node), wheel)
            if node == 3:
                for sample in value["samples"]:
                    sample["calibrated_alpha_rad"] = 0.25
            _, readings, _ = receive(state, value)
            assert len(readings) == 2
            names.add(readings[0]["name"])
    assert len(names) == 6
    assert len(state.streams) == len(state.current_boot) == len(state.metadata) == 6
    report = state.diagnostics_snapshot(1000000000)
    assert all(stream["missing_samples_in_recording"] == 1 for stream in report["streams"])
    receive(state, identified(packet(11), "front_left"))
    report = state.diagnostics_snapshot(1000000000)
    recovered = [stream for stream in report["streams"] if stream["missing_samples_in_recording"] == 0]
    assert len(recovered) == 1 and recovered[0]["wheel_id"] == "front_left" and recovered[0]["node_id"] == 1
    # Reboot left H1; the five other metadata entries survive untouched.
    receive(state, identified(packet(0, boot_id=NEXT_BOOT), "front_left"))
    assert len(state.metadata) == 5
    assert receive(state, identified(packet(13), "front_left"))[1] == []
    assert len(receive(state, identified(packet(13), "front_right"))[1]) == 1
    assert len(state.streams) == 7


@pytest.mark.parametrize("edit", [
    {"wheel_id": "rear_left"}, {"device_id": "02:00:00:00:AA:01"},
    {"device_id": "bad"}, {"device_id": None}, {"wheel_id": None},
])
def test_v2_rejects_incomplete_or_noncanonical_identity(edit):
    value = identified(packet())
    value.update(edit)
    with pytest.raises(ValueError):
        protocol.decode_packet(encoded(value))


def test_legacy_cannot_claim_an_identified_wheel():
    value = identified(packet())
    value["v"] = 1
    with pytest.raises(ValueError, match="legacy"):
        protocol.decode_packet(encoded(value))


def test_live_route_rejections_leave_all_accepted_state_untouched():
    device = "02:00:00:00:01:01"
    state = protocol.TelemetryState(wheel_id="front_left",
        expected_device_ids=[device, "", ""], expected_source_ips=["192.168.1.11", "", ""])
    receive(state, metadata_for(1))
    receive(state, identified(packet(100, 102)))
    before = json.dumps(state.metadata_snapshot(), sort_keys=True)
    stream_before = state.diagnostics_snapshot(1000000000)["streams"]
    wrong_packets = [packet(200), identified(packet(200), "front_right"),
                     identified(packet(200), device="02:00:00:00:01:99")]
    for value in wrong_packets:
        with pytest.raises(ValueError):
            receive(state, value)
    with pytest.raises(ValueError, match="source IP"):
        state.receive(encoded(identified(packet(200))), ("192.168.1.99", 43210), 1, 1000000000)
    assert json.dumps(state.metadata_snapshot(), sort_keys=True) == before
    report = state.diagnostics_snapshot(1000000000)
    assert report["streams"] == stream_before
    assert report["valid_packets"] == 2
    assert report["rejected_packets"] == report["invalid_packets"] == 4
    assert report["nodes"][0]["inventory_verified"]
    assert not report["inventory_verified"]  # H2 and CENTER still unconfigured
    assert state.pinned_devices == {1: device}


def test_commissioning_pins_first_device_per_role_and_detects_device_reuse():
    state = protocol.TelemetryState(wheel_id="front_left")
    receive(state, identified(packet()))
    with pytest.raises(ValueError, match="duplicate device"):
        receive(state, identified(packet(1), device="02:00:00:00:01:99"))
    with pytest.raises(ValueError, match="already belongs"):
        receive(state, identified(packet(1, node_id=2), device="02:00:00:00:01:01"))
    assert len(receive(state, identified(packet(0, boot_id=NEXT_BOOT)))[1]) == 1
    report = state.diagnostics_snapshot(1000000000)
    assert not report["inventory_verified"]
    assert len(report["streams"]) == 2


def test_inventory_fully_verified_only_after_all_configured_devices_seen():
    devices = [f"02:00:00:00:01:{node:02x}" for node in protocol.ROLES]
    state = protocol.TelemetryState(wheel_id="front_left", expected_device_ids=devices)
    for node in protocol.ROLES:
        receive(state, metadata_for(node))
    assert state.diagnostics_snapshot(1000000000)["inventory_verified"]


@pytest.mark.parametrize("kwargs", [
    {"wheel_id": "left"}, {"wheel_id": "front_left", "allow_legacy": True},
    {"expected_device_ids": ["", ""]},
    {"wheel_id": "front_left", "expected_device_ids": ["bad", "", ""]},
    {"wheel_id": "front_left", "expected_device_ids": ["02:00:00:00:00:01"] * 3},
    {"wheel_id": "front_left", "expected_source_ips": ["invalid", "", ""]},
    {"expected_source_ips": ["192.168.1.1", "", ""]},
])
def test_invalid_receiver_inventory_fails_before_receiving(kwargs):
    with pytest.raises(ValueError):
        protocol.TelemetryState(**kwargs)


def test_clock_maps_never_cross_wheel_device_or_legacy_identity():
    base = {"node_id": 1, "boot_id": BOOT, "calibrated": True,
            "slope_ns_per_us": "1000", "offset_ns": 1}
    left = dict(base, wheel_id="front_left", device_id="02:00:00:00:01:01", offset_ns=2)
    right = dict(base, wheel_id="front_right", device_id="02:00:00:00:02:01", offset_ns=3)
    mapper = protocol.ClockMapper({"version": 1, "mappings": [base, left, right]})
    assert mapper.map(1, BOOT, 10) == 10001
    assert mapper.map(1, BOOT, 10, left["wheel_id"], left["device_id"]) == 10002
    assert mapper.map(1, BOOT, 10, right["wheel_id"], right["device_id"]) == 10003
    assert mapper.map(1, BOOT, 10, "front_left", right["device_id"]) is None
    with pytest.raises(ValueError):
        protocol.ClockMapper({"version": 1, "mappings": [dict(base, wheel_id="front_left")]})


@pytest.mark.parametrize("wheel", protocol.WHEELS)
def test_actual_bridge_namespaces_topics_and_joints_and_rejects_other_wheel(monkeypatch, wheel):
    module = load_bridge_with_fake_ros(monkeypatch, parameters={"wheel_id": wheel})
    node = module.CasterEncoderBridge()
    other = "front_right" if wheel == "front_left" else "front_left"
    node.receiver.queue = [
        (encoded(identified(packet(), wheel)), ("192.168.1.11", 1234), 11, 12),
        (encoded(identified(packet(1), other)), ("192.168.1.12", 1234), 13, 14),
    ]
    node.receive()
    expected = {f"/caster_encoder/{wheel}/{topic}" for topic in
                ("packets", "metadata", "diagnostics", "H1/joint_states", "H2/joint_states", "CENTER/joint_states")}
    assert set(node.publishers) == expected
    assert len(node.packets.messages) == 1
    assert node.joints[1].messages[0].name == [f"{wheel}_caster_h1_rotation"]
    assert node.state.rejected_packets == 1


def test_explicit_legacy_bridge_rejects_v2(monkeypatch):
    module = load_bridge_with_fake_ros(monkeypatch, parameters={"wheel_id": "", "legacy_mode": True})
    node = module.CasterEncoderBridge()
    node.receiver.queue = [(encoded(identified(packet())), ("192.168.1.11", 1), 1, 1)]
    node.receive()
    assert node.state.rejected_packets == 1
    assert not node.packets.messages


@pytest.mark.parametrize("configured_wheel", ["", "front_right"])
def test_dual_launch_requires_a_matching_loaded_configuration(monkeypatch, configured_wheel):
    module = load_bridge_with_fake_ros(monkeypatch, parameters={
        "wheel_id": "front_left", "require_config": True,
        "configured_wheel_id": configured_wheel,
    })
    with pytest.raises(ValueError, match="Missing or mismatched caster configuration"):
        module.CasterEncoderBridge()


def test_bridge_configuration_requires_restart_including_simulation_clock(monkeypatch):
    module = load_bridge_with_fake_ros(monkeypatch, parameters={
        "wheel_id": "front_left", "require_config": True,
        "configured_wheel_id": "front_left",
    })
    node = module.CasterEncoderBridge()
    assert all(descriptor.read_only for descriptor in node.descriptors.values())
    assert "use_sim_time" in node.descriptors


def test_packet_identity_rejects_partial_v2_fields_before_grouping():
    with pytest.raises(ValueError):
        protocol.packet_identity({"v": 2, "node_id": 1, "boot_id": BOOT})
    with pytest.raises(ValueError):
        protocol.packet_identity({"node_id": 1, "boot_id": BOOT, "wheel_id": "front_left"})


def test_real_ros_named_configuration_and_read_only_parameters(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter

    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "177")
    monkeypatch.syspath_prepend(str(SCRIPTS))
    loader = importlib.machinery.SourceFileLoader("caster_encoder_bridge_real_ros", str(SCRIPTS / "caster_encoder_bridge"))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    config = SCRIPTS.parent / "config/caster_encoders.yaml"
    rclpy.init(args=["--ros-args", "-r", "__node:=caster_encoder_front_right",
                     "--params-file", str(config), "-p", "require_config:=true",
                     "-p", "bind_address:=127.0.0.1", "-p", f"udp_port:={port}"])
    node = None
    try:
        node = module.CasterEncoderBridge()
        assert node.state.wheel_id == "front_right"
        assert node.get_parameter("use_sim_time").value is False
        for name, value in (("wheel_id", "front_left"), ("udp_port", port), ("use_sim_time", True)):
            result = node.set_parameters([Parameter(name, value=value)])[0]
            assert result.successful is False
            assert "read-only" in result.reason
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
