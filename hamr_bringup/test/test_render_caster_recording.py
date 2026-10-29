"""Local recordings must retain sample timing, wheel identity, and provenance."""

import json
from pathlib import Path
import subprocess
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from caster_encoder_protocol import ROLES
from caster_local_recording import load_recording, write_replay_bag, PROVENANCE_FILE, TIME_BASIS
from test_caster_encoder_replay import wheel_envelope

EPOCH = 1_800_000_000_000_000_001


def local_packets(wheel="front_left"):
    result = []
    for node in (1, 2, 3):
        envelope = wheel_envelope(node, wheel=wheel, angle=node / 10)
        packet = envelope["packet"]
        sample = packet["samples"][0]
        sample["seq"] = 2**53 + 1
        packet["samples"].append({**sample, "seq": 2**53 + 2, "t_us": 255000})
        identity = {key: packet[key] for key in ("v", "wheel_id", "device_id", "node_id", "boot_id")}
        envelope.update(
            origin="encoder_local_recording", recording={"run_id": "test", "file": f"{node}.bin"},
            receive_epoch_ns=None, receive_monotonic_ns=None, source_ip=None, source_port=None,
            timing_estimated=True, diagnostics_known=True,
            clock_mapping={**identity, "calibrated": True, "slope_ns_per_us": "1000",
                           "offset_ns": EPOCH + node * 100, "valid_from_us": 250000,
                           "valid_to_us": 255000, "method": "two_way_midpoint_estimate",
                           "accuracy_validated": False})
        result.append({**envelope, "packet": {**identity, "type": "metadata", "role": ROLES[node],
                                             "firmware_version": "test", "idf_version": "test"}})
        result.append(envelope)
    return result


def save_packets(path, packets):
    path.mkdir(parents=True, exist_ok=True)
    (path / "packets.jsonl").write_text("".join(json.dumps(p) + "\n" for p in packets))


def test_unbatching_interleaves_exact_times_and_keeps_originals_unchanged(tmp_path):
    save_packets(tmp_path, local_packets())
    before = (tmp_path / "packets.jsonl").read_bytes()
    events, provenance = load_recording(tmp_path)
    samples = [(stamp, e) for stamp, e in events if e["packet"]["type"] == "samples"]
    assert [e["packet"]["node_id"] for _, e in samples] == [1, 2, 3, 1, 2, 3]
    assert [stamp for stamp, _ in samples] == [
        EPOCH + timestamp * 1000 + node * 100 for timestamp in (250000, 255000) for node in (1, 2, 3)]
    assert all(len(e["packet"]["samples"]) == 1 for _, e in samples)
    assert samples[0][1]["packet"]["samples"][0]["seq"] == 2**53 + 1
    assert samples[0][1]["packet"]["samples"][0]["raw_count"] == "9007199254740993"
    for stamp, e in events:
        assert e["receive_epoch_ns"] == stamp
        assert e["replay_time_basis"] == TIME_BASIS
        assert e["origin"] == "encoder_local_recording_replay_adapter"
    assert provenance["sample_counts"] == {"H1": 2, "H2": 2, "CENTER": 2}
    assert provenance["accuracy_validated"] is False
    assert (tmp_path / "packets.jsonl").read_bytes() == before
    assert list(tmp_path.iterdir()) == [tmp_path / "packets.jsonl"]


def test_two_wheels_require_selection_and_never_mix_roles(tmp_path):
    save_packets(tmp_path, local_packets() + local_packets("front_right"))
    with pytest.raises(ValueError, match="Select --wheel"):
        load_recording(tmp_path)
    events, provenance = load_recording(tmp_path, "front_right")
    assert provenance["wheel_id"] == "front_right"
    assert len(events) == 9
    assert {e["packet"]["wheel_id"] for _, e in events} == {"front_right"}


@pytest.mark.parametrize("case,match", [
    ("missing_role", "H1, H2, and CENTER"),
    ("missing_map", "Missing clock estimate"),
    ("out_of_bounds", "outside the clock estimate"),
    ("duplicate", "Duplicate or out-of-order"),
    ("mixed_run", "single recording run"),
    ("wrong_clock_identity", "identity mismatch"),
    ("malformed", "packet envelope"),
])
def test_unreplayable_exports_fail_before_writing_anything(tmp_path, case, match):
    packets = local_packets()
    if case == "missing_role":
        packets = packets[:4]
    elif case == "missing_map":
        for p in packets[:2]:
            p.update(clock_mapping=None, timing_estimated=False)
    elif case == "out_of_bounds":
        packets[1]["packet"]["samples"][-1]["t_us"] = 260000
    elif case == "duplicate":
        packets[1]["packet"]["samples"].append(packets[1]["packet"]["samples"][0])
    elif case == "mixed_run":
        packets[-1]["recording"]["run_id"] = "other-run"
    elif case == "wrong_clock_identity":
        packets[-1]["clock_mapping"]["boot_id"] = "1111111111111111"
    elif case == "malformed":
        packets[0] = {"packet": []}
    save_packets(tmp_path, packets)
    with pytest.raises(ValueError, match=match):
        load_recording(tmp_path)
    assert list(tmp_path.iterdir()) == [tmp_path / "packets.jsonl"]


def test_mcap_round_trip_is_accepted_by_existing_renderer(tmp_path):
    pytest.importorskip("rosbag2_py")
    from render_caster_encoder_video import read_bag, sample_frames
    src, dst = tmp_path / "input with spaces", tmp_path / "replay with spaces"
    save_packets(src, local_packets())
    events, provenance = load_recording(src)
    write_replay_bag(dst, events, provenance)
    loaded, start, end = read_bag(dst, wheel_id="front_left")
    assert (start, end) == (events[0][0], events[-1][0])
    assert len(loaded) == len(events)
    frame = sample_frames(loaded, [end], wheel_id="front_left")[0]
    assert frame["status"]["complete_pose_available"]
    assert frame["status"]["invalid_envelopes"] == 0
    assert json.loads((dst / PROVENANCE_FILE).read_text()) == provenance
    with pytest.raises(ValueError, match="overwrite"):
        write_replay_bag(dst, events, provenance)


def test_bash_wrapper_handles_help_and_paths_from_another_directory(tmp_path):
    wrapper = SCRIPTS / "visualize_caster_bag"
    result = subprocess.run(["bash", str(wrapper), "--help"], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0
    assert "source" in result.stdout
    assert "--render {auto,video,preview,none}" in result.stdout
    assert "--preview-time" in result.stdout
    missing = tmp_path / "missing folder"
    result = subprocess.run([sys.executable, str(SCRIPTS / "caster_visualization.py"), str(missing)],
                            cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 2
    assert str(missing) in result.stderr
    assert not list(tmp_path.iterdir())
