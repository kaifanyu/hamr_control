"""Binary integrity, precise time estimation, and honest offline provenance."""

import csv
import importlib.util
import json
from pathlib import Path
import struct
import sys
import zlib

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import caster_recording_format as recording


IDENTITY = {"v": 2, "wheel_id": "front_left", "device_id": "02:00:00:00:00:01",
            "node_id": 1, "boot_id": "1234567890abcdef"}
EPOCH = 1_800_000_000_000_000_000


def metadata(node=1):
    return {"run_id": "run-test", "packet": {**IDENTITY, "node_id": node, "type": "metadata",
            "role": {1: "H1", 2: "H2", 3: "CENTER"}[node], "firmware_version": "test", "idf_version": "test"}}


def record(seq=2**53 + 10, timestamp=2_000_000, flags=3 | 16, position=-1):
    data = recording.RECORD.pack(seq, timestamp, -(2**63), 65535, position, flags,
                                 0.25, float("nan"), float("nan"), 1.5, 1.4,
                                 2**53 + 1, 12345, 2**63 - 1, 0)
    return data[:-4] + struct.pack("<I", zlib.crc32(data[:-4]))


def write_log(path, records=None, node=1):
    doc = metadata(node)
    payload = json.dumps(doc).encode()
    header = recording.HEADER.pack(recording.MAGIC, 1, 100, len(payload), zlib.crc32(payload)) + payload
    data = b"".join([record(), record(2**53 + 12, 4_000_000)] if records is None else records)
    path.write_bytes(header.ljust(4096, b"\0") + data)
    return {**IDENTITY, "node_id": node, "run_id": doc["run_id"], "log_boot_id": IDENTITY["boot_id"],
            "sample_count": len(data) // 100, "data_bytes": 4096 + len(data), "crc32": zlib.crc32(data)}


def exchange(timestamp, slope=1000, round_trip=2_100_000, identity=None):
    # Symmetric network plus 100us of device request processing.
    receive, transmit = timestamp - 50, timestamp + 50
    midpoint = EPOCH + int(timestamp * slope)
    sent, received = midpoint - round_trip // 2, midpoint + round_trip // 2
    return {"host_send_epoch_ns": sent, "host_recv_epoch_ns": received,
            "host_send_monotonic_ns": sent - EPOCH + 100_000_000_000,
            "host_recv_monotonic_ns": received - EPOCH + 100_000_000_000,
            "device": {**(identity or IDENTITY), "rx_us": receive, "tx_us": transmit}}


def fit(before=None, after=None, **kwargs):
    return recording.fit_clock_map(before or [exchange(1_000_000)], after or [exchange(5_000_000)],
                                   IDENTITY, 2_000_000, 4_000_000, **kwargs)


def test_binary_round_trip_exact_integers_gaps_and_nan(tmp_path):
    path = tmp_path / "log.bin"
    status = write_log(path)
    result = recording.verify_log(path, status, "run-test")
    assert result["sample_count"] == 2
    assert result["missing_samples"] == 1
    assert result["max_acquisition_gap_us"] == 2_000_000
    assert result["flag_counts"]["sample_valid"] == 2
    assert result["gaps"] == [[2**53 + 11, 2**53 + 11]]
    sample = next(recording.iter_samples(path))
    assert sample["seq"] == 2**53 + 10
    assert sample["raw_count"] == str(-(2**63))
    assert sample["index_raw_count"] == str(2**63 - 1)
    assert sample["index_event_count"] == 2**53 + 1
    assert sample["absolute_angle_rad"] is None
    assert sample["raw_spi_word"] is None


def test_binary_role_and_validity_match_firmware_serializer():
    center = recording.decode_record(record(position=1234), 3)
    assert center["raw_count"] is None and center["index_raw_count"] is None
    assert center["position14"] == 1234 and center["raw_spi_word"] == 65535
    invalid = recording.decode_record(record(flags=1), 1)
    assert invalid["raw_count"] is None
    assert invalid["index_raw_count"] == str(2**63 - 1)


@pytest.mark.parametrize("mutation", ["header_crc", "record_crc", "truncated_header", "truncated_record", "padding", "record_size"])
def test_binary_corruption_fails(tmp_path, mutation):
    path = tmp_path / "log.bin"
    write_log(path)
    data = bytearray(path.read_bytes())
    if mutation == "header_crc":
        data[30] ^= 1
    elif mutation == "record_crc":
        data[4100] ^= 1
    elif mutation == "truncated_header":
        data = data[:2000]
    elif mutation == "truncated_record":
        data = data[:-1]
    elif mutation == "padding":
        data[4095] = 1
    elif mutation == "record_size":
        data[12] = 99
    path.write_bytes(data)
    with pytest.raises(ValueError):
        recording.verify_log(path)


@pytest.mark.parametrize("field,value", [("wheel_id", "front_right"), ("device_id", "02:00:00:00:00:02"),
    ("node_id", 2), ("log_boot_id", "fedcba0987654321"), ("run_id", "other-run"),
    ("crc32", 0), ("data_bytes", 5000), ("sample_count", 3)])
def test_status_mismatch_fails(tmp_path, field, value):
    path = tmp_path / "log.bin"
    status = write_log(path)
    status[field] = value
    with pytest.raises(ValueError):
        recording.verify_log(path, status)


def test_reboot_current_status_does_not_relabel_old_log(tmp_path):
    path = tmp_path / "log.bin"
    status = write_log(path)
    status["boot_id"] = "fedcba0987654321"
    assert recording.verify_log(path, status)["boot_id"] == IDENTITY["boot_id"]


@pytest.mark.parametrize("second", [record(2**53 + 10, 4_000_000), record(2**53 + 9, 4_000_000),
                                    record(2**53 + 11, 2_000_000), record(2**53 + 11, -1),
                                    record(2**53 + 11, 4_000_000, flags=1 << 13),
                                    record(2**53 + 11, 4_000_000, position=16384)])
def test_invalid_order_or_fields_rejected_even_with_valid_crc(tmp_path, second):
    path = tmp_path / "log.bin"
    write_log(path, [record(), second])
    with pytest.raises(ValueError):
        recording.verify_log(path)


def test_empty_valid_log(tmp_path):
    path = tmp_path / "log.bin"
    status = write_log(path, [])
    result = recording.verify_log(path, status)
    assert result["sample_count"] == 0 and result["first_seq"] is None


@pytest.mark.parametrize("bounds", [{"start_us": 2_000_001}, {"stop_us": 3_999_999},
                                    {"start_us": 5_000_000, "stop_us": 1_000_000}])
def test_crc_valid_samples_outside_requested_window_are_rejected(tmp_path, bounds):
    path = tmp_path / "log.bin"
    status = write_log(path)
    status.update(bounds)
    with pytest.raises(ValueError, match="(start_us|stop_us)"):
        recording.verify_log(path, status)


@pytest.mark.parametrize("records,bounds", [
    (None, {"start_us": 2_000_000, "stop_us": 4_000_000}),
    (None, {"start_us": None, "stop_us": None}),
    (None, {"start_us": None, "stop_us": 4_000_000}),
    ([], {"start_us": 2_000_000, "stop_us": 2_000_000}),
    ([], {"start_us": None, "stop_us": 2_000_000}),
    ([], {"start_us": -1, "stop_us": 2_000_000}),
])
def test_window_boundaries_empty_and_recovered_unknown_bounds_accepted(tmp_path, records, bounds):
    path = tmp_path / "log.bin"
    status = write_log(path, records)
    status.update(bounds)
    assert recording.verify_log(path, status)["sample_count"] == status["sample_count"]


def test_precise_affine_fit_and_unproven_accuracy():
    mapping = fit([exchange(1_000_000, 1000.05)], [exchange(5_000_000, 1000.05)])
    mapper = recording.protocol.ClockMapper({"version": 1, "mappings": [mapping]})
    assert mapper.map(1, IDENTITY["boot_id"], 3_000_000, IDENTITY["wheel_id"], IDENTITY["device_id"]) == EPOCH + 3_000_150_000
    assert mapping["accuracy_validated"] is False and mapping["timing_estimated"] is True
    assert float(mapping["drift_ppm"]) == pytest.approx(50)
    assert mapping["uncertainty_ns"] > 0


def test_low_rtt_anchor_rejects_slow_network_influence():
    left = exchange(1_000_000, round_trip=50_000_000)
    # A heavily asymmetric request moves the midpoint, but retains a valid
    # host epoch/monotonic relation and is discarded in favor of low RTT.
    for name in ("host_send_epoch_ns", "host_recv_epoch_ns", "host_send_monotonic_ns", "host_recv_monotonic_ns"):
        left[name] += 10_000_000
    mapping = fit([left, exchange(1_100_000)], [exchange(5_000_000)])
    assert mapping["slope_ns_per_us"] == "1000"
    assert mapping["max_probe_residual_ns"] == 10_000_000


@pytest.mark.parametrize("problem", ["boot", "identity", "host_step_during", "host_step_across", "impossible_latency", "negative", "short_span", "drift", "unbracketed", "large_rtt"])
def test_invalid_clock_exchange_rejected(problem):
    before, after = [exchange(1_000_000)], [exchange(5_000_000)]
    if problem == "boot":
        after[0]["device"]["boot_id"] = "fedcba0987654321"
    elif problem == "identity":
        after[0]["device"]["node_id"] = 2
    elif problem == "host_step_during":
        after[0]["host_recv_epoch_ns"] += 100_000_000
    elif problem == "host_step_across":
        after[0]["host_send_epoch_ns"] += 100_000_000
        after[0]["host_recv_epoch_ns"] += 100_000_000
    elif problem == "impossible_latency":
        after[0]["device"]["tx_us"] += 10_000
    elif problem == "negative":
        after[0]["device"]["rx_us"] = -1
    elif problem == "short_span":
        after = [exchange(1_500_000)]
    elif problem == "drift":
        before, after = [exchange(1_000_000, 1010)], [exchange(5_000_000, 1010)]
    elif problem == "unbracketed":
        before = [exchange(3_000_000)]
    elif problem == "large_rtt":
        after = [exchange(5_000_000, round_trip=2_000_000_000)]
    with pytest.raises(ValueError):
        fit(before, after)


def manifest(path, status, sync=True):
    return {"version": 1, "run_id": "run-test", "devices": [{**IDENTITY, "log_path": path.name,
            "status": status, "sync_pre": [exchange(1_000_000)] if sync else [],
            "sync_post": [exchange(5_000_000)] if sync else []}]}


def load_analyzer():
    path = SCRIPTS.parents[1] / "rosbags/analyze_caster_encoders.py"
    spec = importlib.util.spec_from_file_location("local_recording_analyzer", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("sync", [True, False])
def test_export_and_existing_analysis_preserve_timing_provenance(tmp_path, sync):
    path = tmp_path / "log.bin"
    status = write_log(path)
    summary = recording.export_run(tmp_path, manifest(path, status, sync))
    assert bool(summary["clock_errors"]) is not sync
    rows = list(csv.DictReader((tmp_path / "samples.csv").open(newline="")))
    assert rows[0]["raw_count"] == str(-(2**63))
    assert rows[0]["receive_epoch_ns"] == "" and rows[0]["source_ip"] == ""
    assert rows[0]["timing_estimated"] == str(int(sync))
    assert rows[0]["mapped_clock_sync_valid"] == "0" and rows[0]["clock_sync_valid"] == "0"
    assert bool(rows[0]["mapped_epoch_ns"]) is sync
    assert rows[0]["queue_drops"] == ""  # unknown is not a measured zero
    analyzer = load_analyzer()
    reference = {recording.protocol.packet_identity(IDENTITY): [(EPOCH + 2_000_000_000, .25)]}
    analyzed, metadata_rows, analyzed_summary = analyzer.analyze(
        analyzer.read_jsonl(tmp_path / "packets.jsonl"), reference=reference)
    assert analyzed[0]["mapped_clock_sync_valid"] == 0
    assert analyzed[0]["timing_estimated"] == int(sync)
    assert analyzed[0]["comparison_time_basis"] == ("estimated_acquisition" if sync else "unmapped_local_acquisition")
    assert analyzed_summary["streams"][0]["firmware_counter_maxima"]["queue_drops"] is None
    assert bool(analyzed[0]["reference_time_epoch_ns"]) is sync
    assert metadata_rows[0]["origin"] == recording.LOCAL_ORIGIN


def test_local_envelope_cannot_fabricate_receive_time(tmp_path):
    path = tmp_path / "log.bin"
    recording.export_run(tmp_path, manifest(path, write_log(path)))
    value = json.loads((tmp_path / "packets.jsonl").read_text().splitlines()[0])
    value["receive_epoch_ns"] = EPOCH
    with pytest.raises(ValueError, match="invent"):
        recording.validate_local_envelope(value)


def test_export_rejects_cross_run_and_path_escape(tmp_path):
    path = tmp_path / "log.bin"
    document = manifest(path, write_log(path))
    document["run_id"] = "other-run"
    with pytest.raises(ValueError, match="run_id"):
        recording.export_run(tmp_path, document)
    document["devices"][0]["log_path"] = "../outside.bin"
    with pytest.raises(ValueError, match="inside"):
        recording.export_run(tmp_path, document)


def test_partial_export_retains_verified_device_and_reports_missing_peer(tmp_path):
    path = tmp_path / "log.bin"
    document = manifest(path, write_log(path))
    document["devices"].append({**IDENTITY, "node_id": 2, "device_id": "02:00:00:00:00:02",
                                 "log_path": "missing.bin"})
    summary = recording.export_run(tmp_path, document)
    assert len(summary["nodes"]) == 1
    assert len(summary["incomplete_devices"]) == 1
    assert summary["incomplete_devices"][0]["device_id"] == "02:00:00:00:00:02"


def test_reboot_unknown_diagnostics_override_zero_placeholder_values(tmp_path):
    path = tmp_path / "log.bin"
    status = write_log(path)
    status["diagnostics"] = {name: 0 for name in recording.protocol.DIAGNOSTIC_FIELDS}
    status["diagnostics_known"] = False
    recording.export_run(tmp_path, manifest(path, status))
    rows = list(csv.DictReader((tmp_path / "samples.csv").open(newline="")))
    assert rows[0]["queue_drops"] == "" and rows[0]["diagnostics_known"] == "0"


def test_analyzer_cli_external_estimated_map_cannot_claim_sync_valid(tmp_path, capsys):
    path = tmp_path / "log.bin"
    recording.export_run(tmp_path, manifest(path, write_log(path)))
    envelopes = [json.loads(line) for line in (tmp_path / "packets.jsonl").read_text().splitlines()]
    for envelope in envelopes:
        envelope["clock_mapping"] = None
        envelope["timing_estimated"] = False
    (tmp_path / "packets.jsonl").write_text("\n".join(json.dumps(item) for item in envelopes) + "\n")
    output = tmp_path / "analysis"
    result = load_analyzer().main(["--packets-jsonl", str(tmp_path / "packets.jsonl"),
                                  "--clock-map", str(tmp_path / "clock-map.json"),
                                  "--output", str(output), "--no-plots"])
    assert result == 0
    rows = list(csv.DictReader((output / "samples.csv").open(newline="")))
    assert rows[0]["mapped_clock_sync_valid"] == "0"
    assert rows[0]["timing_estimated"] == "1"


def test_combined_local_export_supports_explicit_wheel_selection(tmp_path):
    path = tmp_path / "log.bin"
    recording.export_run(tmp_path, manifest(path, write_log(path)))
    analyzer = load_analyzer()
    original = list(analyzer.read_jsonl(tmp_path / "packets.jsonl"))
    other = []
    for value, timestamp in original:
        value = json.loads(json.dumps(value))
        value["packet"]["wheel_id"] = "front_right"
        value["clock_mapping"]["wheel_id"] = "front_right"
        other.append((value, timestamp))
    rows, _, summary = analyzer.analyze(original + other, wheel_id="front_left")
    assert len(rows) == 2 and summary["wheel_id"] == "front_left"
    with pytest.raises(ValueError, match="Expected wheel"):
        analyzer.analyze(original + other)
