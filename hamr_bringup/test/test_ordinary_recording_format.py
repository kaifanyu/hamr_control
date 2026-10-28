"""Binary, timing, and two-channel export conformance from actual C++ output."""

import csv
import json
from pathlib import Path
import shutil
import sys
import zlib

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import caster_recording_format as binary
ordinary = binary.ordinary_recording()
FIXTURES = Path(__file__).parent / "fixtures/ordinary_caster"


def fixture(wheel="front_left"):
    return FIXTURES / (wheel + ".bin")


def status_for(path):
    result = binary.verify_log(path)
    return {**result, "log_boot_id": result["boot_id"], "start_us": result["first_t_us"],
            "stop_us": result["last_t_us"], "state": "stopped"}


def anchor(packet, device_us):
    center = 1_700_000_000_000_000_000 + device_us * 1000
    return {"host_send_epoch_ns": center - 100000, "host_recv_epoch_ns": center + 100000,
            "host_send_monotonic_ns": device_us * 1000 - 100000,
            "host_recv_monotonic_ns": device_us * 1000 + 100000,
            "device": dict(packet, rx_us=device_us - 1, tx_us=device_us + 1)}


@pytest.mark.parametrize("wheel", binary.protocol.WHEELS)
def test_actual_cpp_binary_exactly_matches_json(wheel):
    packets = [json.loads(line) for line in (FIXTURES / (wheel + ".jsonl")).read_text().splitlines()]
    samples = [packet["samples"][0] for packet in packets if packet["type"] == "samples"]
    assert list(binary.iter_samples(fixture(wheel))) == samples
    verified = ordinary.verify_log(fixture(wheel))
    assert verified["sample_count"] == 3
    assert verified["data_bytes"] == 4096 + 3 * 116
    assert verified["record_version"] == 2
    assert verified["first_seq"] == 2**53
    assert verified["last_wheel_t_us"] - verified["last_swivel_t_us"] == 100
    assert verified["flag_counts"]["swivel"]["parity_error"] == 1
    assert verified["flag_counts"]["wheel"]["continuity_lost"] == 1
    binary.verify_log(fixture(wheel), status_for(fixture(wheel)))


def changed_record(tmp_path, record_index, field_index, value, fix_crc=True):
    data = bytearray(fixture().read_bytes())
    start = binary.HEADER_BYTES + record_index * ordinary.RECORD.size
    values = list(ordinary.RECORD.unpack_from(data, start))
    values[field_index] = value
    changed = ordinary.RECORD.pack(*values)
    if fix_crc:
        changed = changed[:-4] + zlib.crc32(changed[:-4]).to_bytes(4, "little")
    data[start:start + ordinary.RECORD.size] = changed
    path = tmp_path / "changed.bin"
    path.write_bytes(data)
    return path


@pytest.mark.parametrize("index,value,message", [(8, 1, "reserved"), (2, 1, "strictly increase"),
                                                (1, 1, "strictly increase"), (0, 0, "strictly increase"),
                                                (5, 4096, "position12"), (6, 16384, "flags")])
def test_rejects_corrupt_layout_and_independent_timestamp_reversal(tmp_path, index, value, message):
    path = changed_record(tmp_path, 1, index, value)
    with pytest.raises(ValueError, match=message):
        binary.verify_log(path)


def test_crc_and_both_channel_recording_window(tmp_path):
    with pytest.raises(ValueError, match="CRC"):
        binary.verify_log(changed_record(tmp_path, 0, 0, 1, fix_crc=False))
    status = status_for(fixture())
    status["stop_us"] = status["last_swivel_t_us"]
    with pytest.raises(ValueError, match="exceeds requested stop"):
        binary.verify_log(fixture(), status)
    earlier_wheel = changed_record(tmp_path, 0, 2, 999900)
    status = status_for(earlier_wheel)
    status["start_us"] = status["first_swivel_t_us"]
    with pytest.raises(ValueError, match="precedes requested start"):
        binary.verify_log(earlier_wheel, status)


def export_fixture(tmp_path, include_clocks=True):
    devices = []
    for wheel in binary.protocol.WHEELS:
        destination = tmp_path / (wheel + ".bin")
        shutil.copyfile(fixture(wheel), destination)
        status = status_for(destination)
        packet = status["metadata"]["packet"]
        devices.append(dict(packet, log_path=destination.name, status=status,
                            sync_pre=[anchor(packet, 500000)] if include_clocks else [],
                            sync_post=[anchor(packet, 2000000)] if include_clocks else []))
    manifest = {"version": 1, "caster_type": "ordinary", "run_id": devices[0]["status"]["run_id"], "devices": devices}
    summary = binary.export_run(tmp_path, manifest)
    return summary, manifest


def test_export_keeps_exact_count_and_independent_mapped_timestamps(tmp_path):
    summary, manifest = export_fixture(tmp_path)
    assert not summary["clock_errors"]
    with (tmp_path / "samples.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 12
    assert rows[1]["raw_count"] == str(-(2**63))
    assert int(rows[1]["mapped_epoch_ns"]) - int(rows[0]["mapped_epoch_ns"]) == 100000
    assert rows[2]["sample_valid"] == "0" and rows[3]["sample_valid"] == "1"
    assert all(row["mapped_clock_sync_valid"] == "0" and row["timing_estimated"] == "1" for row in rows)
    for line in (tmp_path / "packets.jsonl").read_text().splitlines():
        value = json.loads(line)
        binary.validate_local_envelope(value)
        assert value["clock_mapping"]["caster_type"] == "ordinary"
        assert value["receive_epoch_ns"] is None
        assert value["clock_mapping"]["accuracy_validated"] is False
    manifest.pop("caster_type")
    with pytest.raises(ValueError, match="caster_type"):
        binary.export_run(tmp_path, manifest)


def test_missing_clocks_preserve_data_without_invented_timing(tmp_path):
    summary, _ = export_fixture(tmp_path, include_clocks=False)
    assert len(summary["clock_errors"]) == 2
    with (tmp_path / "samples.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 12
    assert all(row["mapped_epoch_ns"] == "" and row["timing_estimated"] == "0" for row in rows)
