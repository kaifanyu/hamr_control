"""History tests exercise the firmware packet shape and independent channels."""

import copy
import json
import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from ordinary_caster.history import load_history

FIXTURES = ROOT / "test/fixtures/ordinary_caster"
GOOD_FLAGS = 1 | 2 | 4 | 8 | 16 | 64 | 1024


def fixture(wheel="front_left"):
    return [json.loads(line) for line in (FIXTURES / f"{wheel}.jsonl").read_text().splitlines()]


def make_board(wheel="front_left", boot="0123456789abcdef", offset_ns=1_700_000_000_000_000_000,
               mapped=True, calibrated=True, count=(1 << 63) - 100):
    packets = fixture(wheel)
    metadata = copy.deepcopy(packets[-1])
    metadata["boot_id"] = boot
    metadata["device_id"] = "aa:bb:cc:dd:ee:01" if wheel == "front_left" else "aa:bb:cc:dd:ee:02"
    metadata["swivel"]["phase_reference_valid"] = calibrated
    metadata["swivel"]["max_rpm_verified"] = calibrated
    metadata["wheel"]["cpr_verified"] = calibrated
    mapping = {key: metadata[key] for key in ("caster_type", "wheel_id", "device_id", "node_id", "boot_id")}
    mapping.update(calibrated=True, slope_ns_per_us="1000", offset_ns=offset_ns,
                   valid_from_us=0, valid_to_us=10_000_000, method="two_way_midpoint_estimate",
                   accuracy_validated=False, uncertainty_ns=1000, max_probe_residual_ns=1000)
    samples = []
    for index in range(5):
        sample = copy.deepcopy(packets[0]["samples"][0])
        sample.update(seq=index, t_us=1_000_000 + index * 5000)
        sample["swivel"].update(flags=GOOD_FLAGS, angle_rad=index * .1, calibrated_angle_rad=index * .1)
        sample["wheel"].update(flags=GOOD_FLAGS, t_us=sample["t_us"] - 100, raw_count=str(count + index))
        samples.append(sample)
    packet = {key: metadata[key] for key in ("v", "caster_type", "wheel_id", "device_id", "node_id", "boot_id")}
    packet.update(type="samples", samples=samples, diagnostics=packets[0]["diagnostics"])
    return metadata, packet, mapping if mapped else None


def write_run(path, *boards):
    path.mkdir(parents=True, exist_ok=True)
    envelopes = []
    for metadata, packet, mapping in boards:
        base = dict(origin="encoder_local_recording", recording=dict(run_id="history-test", file="device.bin"),
                    receive_epoch_ns=None, receive_monotonic_ns=None, source_ip=None, source_port=None,
                    clock_mapping=mapping, timing_estimated=mapping is not None, diagnostics_known=True)
        envelopes.extend(({**base, "packet": metadata}, {**base, "packet": packet}))
    (path / "packets.jsonl").write_text("".join(json.dumps(value) + "\n" for value in envelopes))
    return path


def test_exact_counts_and_per_channel_timestamps(tmp_path):
    history = load_history(write_run(tmp_path, make_board()), require_clock=True)
    swivel, wheel = history.streams
    assert swivel.observations[0].time_ns - wheel.observations[0].time_ns == 100_000
    assert wheel.observations[0].values["raw_count"] == str((1 << 63) - 100)
    assert wheel.observations[2].position == pytest.approx(math.tau * 2 / 128)
    assert history.pose_at(wheel.times[0])["joint_names"] == ["front_left_caster_wheel_joint"]
    assert history.pose_at(wheel.times[0])["channels"][0]["state"] == "before_history"


def test_independent_spi_fault_and_gap(tmp_path):
    board = make_board()
    board[1]["samples"][2]["swivel"].update(flags=1 | 128, angle_rad=None, calibrated_angle_rad=None)
    history = load_history(write_run(tmp_path, board), require_clock=True)
    swivel, wheel = history.streams
    pose = history.pose_at(swivel.times[2])
    assert "front_left_caster_swivel_joint" not in pose["joint_names"]
    assert "front_left_caster_wheel_joint" in pose["joint_names"]
    assert swivel.at(swivel.times[1] + 1)["state"] == "gap_or_invalid_boundary"
    assert wheel.at(wheel.times[1] + 1)["state"] == "interpolated"


def test_no_interpolation_over_missing_sequence_or_end(tmp_path):
    board = make_board()
    del board[1]["samples"][2]
    history = load_history(write_run(tmp_path, board))
    swivel = history.streams[0]
    assert swivel.at(swivel.times[1] + 1)["state"] == "gap_or_invalid_boundary"
    assert not swivel.at(swivel.times[-1] + 1)["valid"]


def test_wrapped_pose_does_not_invent_turns(tmp_path):
    board = make_board(calibrated=False)
    for index, sample in enumerate(board[1]["samples"]):
        sample["swivel"].update(flags=1 | 2 | 256, angle_rad=None,
                                calibrated_angle_rad=3.1 if index % 2 == 0 else -3.1)
    history = load_history(write_run(tmp_path, board))
    swivel = history.streams[0]
    assert not swivel.at(swivel.times[0])["valid"]
    value = swivel.at(swivel.times[0] + 1, True)
    assert value["angle_rad"] == 3.1
    assert value["state"] == "held_wrapped_observation"
    assert not swivel.observations[0].continuous


def test_verified_swivel_wrap_interpolates_unwrapped_value(tmp_path):
    board = make_board()
    for index, sample in enumerate(board[1]["samples"]):
        sample["swivel"]["angle_rad"] = 3.13 + index * .02
        sample["swivel"]["calibrated_angle_rad"] = math.remainder(3.13 + index * .02, math.tau)
    history = load_history(write_run(tmp_path, board))
    swivel = history.streams[0]
    middle = swivel.at((swivel.times[0] + swivel.times[1]) // 2)
    assert middle["state"] == "interpolated"
    assert middle["angle_rad"] == pytest.approx(3.14)


def test_dual_requires_usable_maps_and_explicit_single_board_local(tmp_path):
    path = write_run(tmp_path, make_board(mapped=False), make_board("front_right", mapped=False))
    history = load_history(path)
    assert history.time_basis == "separate_device_clocks"
    with pytest.raises(ValueError, match="Cannot combine"):
        history.pose_at(history.start_ns)
    with pytest.raises(ValueError, match="Cannot align"):
        load_history(path, require_clock=True)
    with pytest.raises(ValueError, match="one sensor board and one boot"):
        load_history(path, device_local=True)
    assert load_history(path, wheel="front_right", device_local=True, require_clock=True).time_basis == "device_local"


def test_clock_coverage_and_uncertainty_gate_each_channel(tmp_path):
    board = make_board()
    board[2]["valid_from_us"] = 1_000_000  # Excludes first wheel observation.
    path = write_run(tmp_path, board)
    with pytest.raises(ValueError, match="does not cover every channel"):
        load_history(path, require_clock=True)
    board[2]["valid_from_us"] = 0
    board[2]["uncertainty_ns"] = 11_000_000
    write_run(path, board)
    with pytest.raises(ValueError, match="excessive clock uncertainty"):
        load_history(path, require_clock=True)
    assert load_history(path, require_clock=True, max_clock_uncertainty_ms=12).time_basis == "estimated_acquisition_epoch"


def test_bad_probe_residual_rejected(tmp_path):
    board = make_board()
    board[2]["max_probe_residual_ns"] = 100_000_000
    with pytest.raises(ValueError, match="probe residual"):
        load_history(write_run(tmp_path, board), require_clock=True)


def test_reboot_is_separate_and_overlapping_clocks_rejected(tmp_path):
    first = make_board()
    second = make_board(boot="fedcba9876543210", offset_ns=1_700_000_001_000_000_000)
    path = write_run(tmp_path, first, second)
    history = load_history(path, require_clock=True)
    assert len(history.streams) == 4
    gap = history.streams[0].times[-1] + 100_000_000
    assert history.pose_at(gap)["joint_names"] == []
    with pytest.raises(ValueError, match="one sensor board and one boot"):
        load_history(path, device_local=True)
    second[2]["offset_ns"] = first[2]["offset_ns"]
    with pytest.raises(ValueError, match="Overlapping boot clocks"):
        load_history(write_run(path, first, second))


def test_nonmonotonic_and_live_receipt_envelopes_rejected(tmp_path):
    board = make_board()
    board[1]["samples"][2]["wheel"]["t_us"] = board[1]["samples"][0]["wheel"]["t_us"]
    with pytest.raises(ValueError, match="nonmonotonic"):
        load_history(write_run(tmp_path, board))
    write_run(tmp_path, make_board())
    path = tmp_path / "packets.jsonl"
    path.write_text(path.read_text().replace('"encoder_local_recording"', '"live_udp"'))
    with pytest.raises(ValueError, match="unknown local recording origin"):
        load_history(path)


def test_real_firmware_fixture_imported_without_discarding_invalid_channel(tmp_path):
    packets = fixture()
    metadata = packets[-1]
    sample_packet = {**packets[0], "samples": [packet["samples"][0] for packet in packets[:3]]}
    history = load_history(write_run(tmp_path, (metadata, sample_packet, None)), device_local=True)
    swivel, wheel = history.streams
    assert [point.valid for point in swivel.observations] == [True, False, True]
    assert [point.valid for point in wheel.observations] == [True, True, False]
    assert wheel.observations[1].values["raw_count"] == str(-(1 << 63) + 1)


def test_real_dual_firmware_binary_export_readable_as_history(tmp_path):
    from test_ordinary_recording_format import export_fixture
    export_fixture(tmp_path)
    history = load_history(tmp_path, require_clock=True)
    assert history.wheels == ["front_left", "front_right"]
    assert history.time_basis == "estimated_acquisition_epoch"
    assert len(history.streams) == 4
    assert all(len(stream.observations) == 3 for stream in history.streams)
    assert history.streams[1].origin_raw_count == -(1 << 63)
