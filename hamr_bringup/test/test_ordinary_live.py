"""Per-axis acquisition validity, inventory and boot ordering at the live boundary."""
import copy
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "scripts")]
from ordinary_caster.live import LiveState
from ordinary_caster.protocol import DIAGNOSTIC_FIELDS


def packet(seq=0, boot="0123456789abcdef"):
    return dict(v=3, type="samples", caster_type="ordinary", wheel_id="front_left",
        device_id="aa:bb:cc:dd:ee:01", node_id=4, boot_id=boot,
        samples=[dict(seq=seq, t_us=1000 + seq * 5000,
            swivel=dict(raw_spi_word=0, position12=0, absolute_angle_rad=0.0,
                calibrated_angle_rad=0.0, angle_rad=None, velocity_rad_s=None,
                filtered_velocity_rad_s=None, flags=3),
            wheel=dict(t_us=990 + seq * 5000, raw_count="-9223372036854775808",
                angle_rad=0.0, velocity_rad_s=0.0, filtered_velocity_rad_s=0.0, flags=67))],
        diagnostics={name: 0 for name in DIAGNOSTIC_FIELDS})


def receive(state, value, now=1000000000):
    return state.receive(json.dumps(value).encode(), ("127.0.0.1", 5000),
                         1791480000123456789, now, 1791480000123456791)


def test_exact_raw_envelope_independent_timestamps_and_zero_angles():
    state = LiveState("front_left")
    value = packet()
    envelope, readings, changed = receive(state, value)
    assert envelope["packet"] == value
    assert envelope["receive_epoch_ns"] == 1791480000123456789
    assert changed and len(readings) == 2
    assert [r["acquisition_t_us"] for r in readings] == [1000, 990]
    assert [r["position"] for r in readings] == [0, 0]
    assert readings[0]["velocity"] is None
    assert not state.metadata_snapshot()["clocks_synchronized"]
    assert not state.metadata_snapshot()["inventory_verified"]


@pytest.mark.parametrize("bad_channel", ["swivel", "wheel"])
def test_one_failed_channel_keeps_other_and_marks_diagnostics(bad_channel):
    state = LiveState("front_left")
    value = packet()
    value["samples"][0][bad_channel]["flags"] = 1
    _, readings, _ = receive(state, value)
    assert [r["channel"] for r in readings] == ["wheel" if bad_channel == "swivel" else "swivel"]
    assert not state.diagnostics_snapshot(1000000001)["channels"][bad_channel]["valid"]


def test_duplicate_late_samples_and_old_boot_never_refresh_live_pose():
    state = LiveState("front_left")
    receive(state, packet(10))
    receive(state, packet(12), now=2000000000)
    assert receive(state, packet(11), now=3000000000)[1] == []
    assert receive(state, packet(12), now=3000000000)[1] == []
    report = state.diagnostics_snapshot(4000000000)
    assert all(ch["stale"] for ch in report["channels"].values())
    assert report["streams"]["0123456789abcdef"]["missing_samples"] == 0
    receive(state, packet(0, "fedcba9876543210"), now=5000000000)
    assert receive(state, packet(13), now=6000000000)[1] == []
    assert state.boot_id == "fedcba9876543210"
    assert state.diagnostics_snapshot(7000000000)["channels"]["wheel"]["stale"]


def test_separate_wheel_timestamp_must_advance():
    state = LiveState("front_left")
    receive(state, packet())
    value = packet(1)
    value["samples"][0]["wheel"]["t_us"] = 990
    assert [r["channel"] for r in receive(state, value)[1]] == ["swivel"]


@pytest.mark.parametrize("field,value", [("wheel_id", "front_right"),
    ("device_id", "aa:bb:cc:dd:ee:02"), ("caster_type", "ball"), ("node_id", 3)])
def test_inventory_rejection_retains_state(field, value):
    state = LiveState("front_left", expected_device_id="aa:bb:cc:dd:ee:01")
    receive(state, packet())
    invalid = copy.deepcopy(packet(1))
    invalid[field] = value
    with pytest.raises(ValueError):
        receive(state, invalid)
    assert state.channels["wheel"]["seq"] == 0
    assert state.rejected_packets == 1
    assert state.metadata_snapshot()["inventory_verified"]


def test_unconfigured_inventory_pins_first_device_and_optional_source():
    state = LiveState("front_left", expected_source_ip="127.0.0.1")
    receive(state, packet())
    other = packet(1)
    other["device_id"] = "aa:bb:cc:dd:ee:02"
    with pytest.raises(ValueError):
        receive(state, other)
    with pytest.raises(ValueError):
        state.receive(json.dumps(packet(1)).encode(), ("127.0.0.2", 1), 1, 1)
