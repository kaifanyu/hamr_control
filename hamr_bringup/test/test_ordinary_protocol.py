"""Read the actual firmware serializer output through the ROS wire contract."""

from copy import deepcopy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import caster_encoder_protocol as shared

ordinary = shared.ordinary_protocol()
FIXTURES = Path(__file__).parent / "fixtures/ordinary_caster"


def packets(wheel="front_left"):
    return [ordinary.decode_packet(line) for line in (FIXTURES / (wheel + ".jsonl")).read_bytes().splitlines()]


@pytest.mark.parametrize("wheel", shared.WHEELS)
def test_actual_cpp_serializer_is_lossless(wheel):
    values = packets(wheel)
    assert len(values) == 4
    samples = [p["samples"][0] for p in values if p["type"] == "samples"]
    assert samples[0]["seq"] == 2**53
    assert samples[0]["wheel"]["raw_count"] == str(-(2**63))
    assert samples[1]["swivel"]["position12"] is None
    assert samples[1]["wheel"]["raw_count"] == str(-(2**63) + 1)
    assert samples[2]["wheel"]["raw_count"] is None
    for value in values:
        assert shared.decode_packet(json.dumps(value).encode()) == value
    rows = list(ordinary.channel_rows(values[0], samples[0]))
    assert [row["channel"] for row in rows] == ["swivel", "wheel"]
    assert rows[1]["t_us"] - rows[0]["t_us"] == 100


@pytest.mark.parametrize("path,bad", [
    (("v",), True), (("caster_type",), "ball"), (("node_id",), 1),
    (("samples", 0, "seq"), 1.5), (("samples", 0, "wheel", "raw_count"), -(2**63)),
    (("samples", 0, "wheel", "raw_count"), str(2**63)),
    (("samples", 0, "wheel", "t_us"), True),
    (("samples", 0, "swivel", "position12"), 4096),
    (("samples", 0, "swivel", "flags"), 16384),
    (("samples", 0, "swivel", "angle_rad"), float("inf")),
])
def test_rejects_malformed_fields(path, bad):
    value = deepcopy(packets()[0])
    parent = value
    for key in path[:-1]:
        parent = parent[key]
    parent[path[-1]] = bad
    with pytest.raises(ValueError):
        ordinary.decode_packet(json.dumps(value).encode())


def test_metadata_configuration_is_validated():
    metadata = packets()[-1]
    metadata["swivel"]["resolution_bits"] = 14
    with pytest.raises(ValueError, match="resolution_bits"):
        ordinary.decode_packet(json.dumps(metadata).encode())


def test_identity_guard_allows_reboot_not_reassignment():
    left, right = packets()[0], packets("front_right")[0]
    guard = ordinary.IdentityGuard()
    guard.accept(left)
    guard.accept(right)
    guard.accept(dict(left, boot_id="1111111111111111"))
    with pytest.raises(ValueError, match="assignment"):
        guard.accept(dict(left, device_id=right["device_id"]))
    with pytest.raises(ValueError, match="assignment"):
        guard.accept(dict(right, device_id=left["device_id"]))


def test_ball_state_rejects_ordinary_without_changing_streams():
    state = shared.TelemetryState()
    value = json.dumps(packets()[0]).encode()
    with pytest.raises(ValueError, match="ordinary caster bridge"):
        state.receive(value, ("127.0.0.1", 5007), 1, 1)
    assert not state.streams
