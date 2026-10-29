"""Paired replay preserves inter-device offsets and discloses timing recovery."""

import copy
import json
from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from caster_pair_recording import load_pair_recording, write_pair_replay_bag, PROVENANCE_FILE
from caster_recording_format import fit_clock_map
from test_render_caster_recording import EPOCH, local_packets, save_packets


def pair_packets():
    left, right = local_packets(), local_packets("front_right")
    for envelope in right:
        envelope["clock_mapping"] = {**envelope["clock_mapping"],
                                     "offset_ns": envelope["clock_mapping"]["offset_ns"] + 2_000_000}
    return left + right


def recovery_manifest(packets, slope=1003):
    packet = packets[0]["packet"]
    identity = {key: packet[key] for key in ("wheel_id", "device_id", "node_id", "boot_id")}

    def probe(device_us):
        midpoint = (device_us + 100) * slope
        return {
            "host_send_epoch_ns": EPOCH + midpoint - 5_000_000,
            "host_recv_epoch_ns": EPOCH + midpoint + 5_000_000,
            "host_send_monotonic_ns": midpoint - 5_000_000,
            "host_recv_monotonic_ns": midpoint + 5_000_000,
            "device": {**identity, "rx_us": device_us, "tx_us": device_us + 200},
        }

    return {"run_id": "test", "devices": [{**identity,
            "sync_pre": [probe(100_000)], "sync_post": [probe(1_100_000)]}]}


def test_pair_keeps_full_union_and_exact_cross_wheel_offsets(tmp_path):
    packets = pair_packets()
    save_packets(tmp_path, packets)
    original = (tmp_path / "packets.jsonl").read_bytes()
    events, provenance = load_pair_recording(tmp_path)
    assert set(events) == {"front_left", "front_right"}
    for wheel, wheel_events in events.items():
        assert len(wheel_events) == 9
        assert {e["packet"]["wheel_id"] for _, e in wheel_events} == {wheel}
        assert provenance["sample_counts"][wheel] == {"H1": 2, "H2": 2, "CENTER": 2}
        assert all(e["receive_monotonic_ns"] == stamp - provenance["start_ns"] for stamp, e in wheel_events)
    assert events["front_right"][0][0] - events["front_left"][0][0] == 2_000_000
    assert provenance["start_ns"] == EPOCH + 250_000_000 + 100 - 1
    assert provenance["end_ns"] == EPOCH + 255_000_000 + 300 + 2_000_000
    assert provenance["recovered_clock_mappings"] == []
    assert provenance["accuracy_validated"] is False
    assert (tmp_path / "packets.jsonl").read_bytes() == original
    assert len(list(tmp_path.iterdir())) == 1


def test_missing_map_needs_explicit_recovery_and_preserves_other_maps(tmp_path):
    packets = pair_packets()
    original_maps = [copy.deepcopy(e["clock_mapping"]) for e in packets[2::2]]
    manifest = recovery_manifest(packets)
    for envelope in packets[:2]:
        envelope.update(clock_mapping=None, timing_estimated=False)
    save_packets(tmp_path, packets)
    (tmp_path / "manifest.json").write_text(json.dumps(manifest))
    before = {path.name: path.read_bytes() for path in tmp_path.iterdir()}
    with pytest.raises(ValueError, match="--recover-timing"):
        load_pair_recording(tmp_path)
    with pytest.raises(ValueError, match="drift exceeds"):
        device = manifest["devices"][0]
        fit_clock_map(device["sync_pre"], device["sync_post"], device, 250_000, 255_000)
    events, provenance = load_pair_recording(tmp_path, recover_timing=True)
    recovered = provenance["recovered_clock_mappings"]
    assert len(recovered) == 1
    assert float(recovered[0]["drift_ppm"]) == pytest.approx(3000)
    assert recovered[0]["recovery_max_drift_ppm"] == 5000
    assert recovered[0]["accuracy_validated"] is False
    assert recovered[0]["method"] == "two_way_midpoint_estimate"
    assert provenance["clock_mappings"][1:] == original_maps
    for stamp, envelope in events["front_left"]:
        if envelope["packet"]["node_id"] != 1:
            continue
        assert envelope["original_clock_mapping"] is None
        assert envelope["timing_recovered_for_visualization"] is True
        if envelope["packet"]["type"] == "samples":
            sample = envelope["packet"]["samples"][0]
            assert stamp == EPOCH + sample["t_us"] * 1003
            assert sample in packets[1]["packet"]["samples"]
    assert {path.name: path.read_bytes() for path in tmp_path.iterdir()} == before


@pytest.mark.parametrize("case,match", [
    ("no_manifest", "original clock exchanges"),
    ("wrong_run", "same recording run"),
    ("wrong_boot", "exactly one manifest identity"),
    ("high_drift", "drift exceeds"),
    ("clock_step", "epoch clock stepped"),
    ("unbracketed", "bracket recorded samples"),
])
def test_recovery_retains_identity_and_clock_sanity_gates(tmp_path, case, match):
    packets = pair_packets()
    manifest = recovery_manifest(packets, slope=1006 if case == "high_drift" else 1003)
    device = manifest["devices"][0]
    if case == "wrong_run":
        manifest["run_id"] = "another-run"
    elif case == "wrong_boot":
        device["boot_id"] = "1111111111111111"
    elif case == "clock_step":
        device["sync_post"][0]["host_recv_epoch_ns"] += 10_000_000
    elif case == "unbracketed":
        device["sync_pre"][0]["device"].update(rx_us=300_000, tx_us=300_200)
        device["sync_post"][0]["device"].update(rx_us=1_400_000, tx_us=1_400_200)
    for envelope in packets[:2]:
        envelope.update(clock_mapping=None, timing_estimated=False)
    save_packets(tmp_path, packets)
    if case != "no_manifest":
        (tmp_path / "manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match=match):
        load_pair_recording(tmp_path, recover_timing=True)


@pytest.mark.parametrize("case,match", [
    ("missing_wheel", "H1, H2, and CENTER samples on front_right"),
    ("missing_role", "H1, H2, and CENTER samples on front_left"),
    ("mixed_run", "single recording run"),
    ("duplicate", "Duplicate or out-of-order"),
    ("changed_device", "changed device identity"),
])
def test_pair_rejects_incomplete_or_conflicting_data(tmp_path, case, match):
    packets = pair_packets()
    if case == "missing_wheel":
        packets = packets[:6]
    elif case == "missing_role":
        packets = packets[2:]
    elif case == "mixed_run":
        packets[-1]["recording"]["run_id"] = "other"
    elif case == "duplicate":
        packets.append(packets[-1])
    elif case == "changed_device":
        packet = copy.deepcopy(packets[-1])
        packet["packet"]["device_id"] = "00:00:00:00:00:00"
        packet["clock_mapping"]["device_id"] = packet["packet"]["device_id"]
        packets.append(packet)
    save_packets(tmp_path, packets)
    with pytest.raises(ValueError, match=match):
        load_pair_recording(tmp_path)


def test_pair_mcap_round_trip_keeps_both_wheels_on_one_clock(tmp_path):
    pytest.importorskip("rosbag2_py")
    from render_caster_encoder_video import read_bag
    source, destination = tmp_path / "source", tmp_path / "bag"
    save_packets(source, pair_packets())
    events, provenance = load_pair_recording(source)
    write_pair_replay_bag(destination, events, provenance)
    for wheel in events:
        loaded, start, end = read_bag(destination, wheel_id=wheel)
        assert (start, end) == (provenance["start_ns"], provenance["end_ns"])
        assert [stamp for stamp, _ in loaded] == [stamp for stamp, _ in events[wheel]]
    assert json.loads((destination / PROVENANCE_FILE).read_text()) == provenance
    with pytest.raises(ValueError, match="overwrite"):
        write_pair_replay_bag(destination, events, provenance)
