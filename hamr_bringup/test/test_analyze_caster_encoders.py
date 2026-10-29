"""Portable caster replay tests with independent known-error reference fixtures."""

import csv
from importlib.util import module_from_spec, spec_from_file_location
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "caster_packet_analysis.py"
SPEC = spec_from_file_location("analyze_caster_encoders", SCRIPT)
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
BOOT = "0123456789abcdef"
BOOT2 = "fedcba9876543210"
EPOCH = 1_800_000_000_000_000_123
GOOD = sum(1 << bit for bit in (0, 1, 2, 3, 4, 6, 10))


def sample(seq, angle=0.0, flags=GOOD, t_us=None, **values):
    result = {
        "seq": seq, "t_us": seq * 10_000 if t_us is None else t_us,
        "raw_count": str(2 ** 53 + 123 + seq), "raw_spi_word": None,
        "position14": None, "angle_rad": angle, "absolute_angle_rad": None,
        "calibrated_alpha_rad": None, "velocity_rad_s": 1.0,
        "filtered_velocity_rad_s": .9, "index_event_count": 0,
        "index_timestamp_us": 0, "index_raw_count": None, "flags": flags,
    }
    result.update(values)
    return result


def envelope(samples, node=1, boot=BOOT, receive_ns=None, counter=4):
    receive_ns = EPOCH + samples[-1]["t_us"] * 1000 if receive_ns is None else receive_ns
    return {
        "receive_epoch_ns": receive_ns,
        "receive_monotonic_ns": receive_ns - EPOCH + 1_000_000_000,
        "source_ip": "192.0.2.1", "source_port": 5005,
        "packet": {"v": 1, "type": "samples", "node_id": node, "boot_id": boot,
                   "samples": samples,
                   "diagnostics": {name: counter for name in MODULE.protocol.DIAGNOSTIC_FIELDS}},
    }


def analyze(packets, **kwargs):
    return MODULE.analyze([(packet, None) for packet in packets], **kwargs)


def reference_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["node_id", "boot_id", "time_epoch_ns", "angle_rad"])
        writer.writerows(rows)
    return path


def test_known_error_without_hidden_fitting():
    packets = [envelope([sample(i, i * .01 + .125)]) for i in range(5)]
    reference = {(1, BOOT): [(EPOCH + i * 10_000_000, i * .01) for i in range(5)]}
    rows, _, report = analyze(packets, reference=reference)
    result = report["streams"][0]["reference_comparison"]
    assert result["count"] == 5
    assert result["rmse_rad"] == pytest.approx(.125)
    assert result["mae_rad"] == pytest.approx(.125)
    assert result["bias_rad"] == pytest.approx(.125)
    assert result["p95_abs_error_rad"] == pytest.approx(.125)
    assert result["max_abs_error_rad"] == pytest.approx(.125)
    assert result["matched_using_host_receive_time"] == 5
    assert not result["absolute_accuracy_established"]
    assert all(row["reference_match_dt_ns"] == 0 for row in rows)


def test_center_circular_error_remains_valid_without_unwrap():
    s = sample(0, None, flags=(1 << 0) | (1 << 1) | (1 << 8),
               raw_count=None, raw_spi_word=123, position14=123,
               absolute_angle_rad=math.pi - .01, calibrated_alpha_rad=math.pi - .01)
    rows, _, report = analyze([envelope([s], node=3)], reference={(3, BOOT): [(EPOCH, -math.pi + .01)]})
    assert rows[0]["angle_eligible"] == 1
    assert rows[0]["velocity_eligible"] == 0
    assert rows[0]["angle_error_rad"] == pytest.approx(-.02)
    assert report["streams"][0]["reference_comparison"]["matched_with_physical_phase_verified"] == 0


def test_hemisphere_missed_revolution_is_not_wrapped_away():
    rows, _, report = analyze([envelope([sample(0, 2 * math.pi + .1)])],
                              reference={(1, BOOT): [(EPOCH, .1)]})
    assert rows[0]["angle_error_rad"] == pytest.approx(2 * math.pi)
    assert report["streams"][0]["reference_comparison"]["rmse_rad"] == pytest.approx(2 * math.pi)


def test_loss_reordering_duplicates_and_reboots_are_separate():
    packets = [envelope([sample(i)], receive_ns=EPOCH + index * 10_000_000)
               for index, i in enumerate((10, 13, 11, 11))]
    packets += [envelope([sample(0)], boot=BOOT2)]
    rows, _, report = analyze(packets)
    stream, reboot = report["streams"]
    assert stream["unique_samples"] == 3
    assert stream["samples_before_observed_range"] == 10
    assert stream["missing_between_observed_sequences"] == 1
    assert stream["missing_fraction_observed_range"] == .25
    assert stream["reordered_samples"] == 1
    assert stream["duplicate_samples"] == 1
    assert stream["firmware_counter_maxima"]["queue_drops"] == 4
    assert stream["continuity_segments"] == 2
    assert reboot["missing_between_observed_sequences"] == 0
    assert rows[3]["sequence_status"] == "duplicate"
    assert rows[3]["angle_eligible"] == 0


def test_conflicting_duplicates_exclude_both_values():
    rows, _, report = analyze([envelope([sample(0, .1)]), envelope([sample(0, .9)])],
                              reference={(1, BOOT): [(EPOCH, 0)]})
    assert not any(row["angle_eligible"] for row in rows)
    assert report["streams"][0]["conflicting_duplicate_sequences"] == 1
    assert report["streams"][0]["reference_comparison"]["count"] == 0


@pytest.mark.parametrize("flags,reason", [
    (GOOD & ~(1 << 0), "not_ready"),
    (GOOD & ~(1 << 1), "sample_invalid"),
    (GOOD & ~(1 << 4), "continuity_unverified"),
    (GOOD | (1 << 8), "continuity_unverified"),
    (GOOD | (1 << 7), "sensor_error"),
    (GOOD | (1 << 12), "sensor_error"),
])
def test_invalid_measurements_retained_but_not_compared(flags, reason):
    rows, _, report = analyze([envelope([sample(0, 1, flags)])],
                              reference={(1, BOOT): [(EPOCH, 0)]})
    assert rows[0]["angle_rad"] == 1
    assert rows[0]["flags"] == flags
    assert rows[0]["comparison_exclusion"] == reason
    assert report["streams"][0]["reference_comparison"]["count"] == 0


def test_no_reference_explicitly_reports_accuracy_unavailable():
    _, _, report = analyze([envelope([sample(0)])])
    assert report["accuracy_status"] == "accuracy_unavailable"
    assert report["reference_supplied"] is False
    assert report["streams"][0]["reference_comparison"]["rmse_rad"] is None


def test_matching_tolerance_and_boot_identity_gate_reference():
    _, _, report = analyze([envelope([sample(0)])], reference={
        (1, BOOT): [(EPOCH + 20_000_001, 0)], (1, BOOT2): [(EPOCH, 0)]})
    assert report["streams"][0]["reference_comparison"]["count"] == 0


def test_reference_csv_integer_ns_and_explicit_transforms(tmp_path):
    path = reference_csv(tmp_path / "ref.csv", [(1, BOOT, EPOCH, 2.5)])
    reference = MODULE.read_reference(path, offset_s=.01, signs={1: -1}, zeros={1: .5})
    assert reference[(1, BOOT)] == [(EPOCH + 10_000_000, -2.0)]


def test_excluded_reference_row_can_have_no_angle_but_invalid_flag_is_rejected(tmp_path):
    path = tmp_path / "visibility.csv"
    header = "node_id,boot_id,time_epoch_ns,angle_rad,valid\n"
    path.write_text(header + f"1,{BOOT},{EPOCH},,0\n"
                    + f"1,{BOOT},{EPOCH + 1},0.25,1\n", encoding="utf-8")
    assert MODULE.read_reference(path) == {(1, BOOT): [(EPOCH + 1, .25)]}
    path.write_text(header + f"1,{BOOT},{EPOCH},,maybe\n", encoding="utf-8")
    with pytest.raises(ValueError, match="valid column must be 0 or 1"):
        MODULE.read_reference(path)


def test_header_only_and_wrong_boot_references_report_accuracy_unavailable(tmp_path):
    path = reference_csv(tmp_path / "empty.csv", [])
    reference = MODULE.read_reference(path)
    _, _, report = analyze([envelope([sample(0)])], reference=reference)
    assert report["reference_supplied"] is True
    assert report["accuracy_status"] == "accuracy_unavailable"
    path = reference_csv(tmp_path / "wrong_boot.csv", [(1, BOOT2, EPOCH, 0)])
    _, _, report = analyze([envelope([sample(0)])], reference=MODULE.read_reference(path))
    assert report["accuracy_status"] == "accuracy_unavailable"
    assert report["streams"][0]["reference_comparison"]["count"] == 0


@pytest.mark.parametrize("contents,match", [
    ("time,angle\n1,2\n", "requires columns"),
    (f"node_id,boot_id,time_epoch_ns,angle_rad\n1,{BOOT},1.8e18,0\n", "decimal integer"),
    (f"node_id,boot_id,time_epoch_ns,angle_rad\n1,{BOOT},{EPOCH},nan\n", "finite"),
    (f"node_id,boot_id,time_epoch_ns,angle_rad\n4,{BOOT},{EPOCH},0\n", "node_id"),
    (f"node_id,boot_id,time_epoch_ns,angle_rad\n1,{BOOT},{EPOCH},0\n1,{BOOT},{EPOCH},1\n", "duplicate"),
])
def test_reference_csv_validation(tmp_path, contents, match):
    path = tmp_path / "bad.csv"
    path.write_text(contents, encoding="utf-8")
    with pytest.raises(ValueError, match=match):
        MODULE.read_reference(path)


def test_calibrated_clock_map_preserves_ns_and_respects_validity():
    document = {"version": 1, "mappings": [{
        "node_id": 1, "boot_id": BOOT, "calibrated": True,
        "slope_ns_per_us": "1000.125", "offset_ns": EPOCH,
        "valid_from_us": 8, "valid_to_us": 8,
    }]}
    mapped = EPOCH + 8001
    rows, _, report = analyze([envelope([sample(0, t_us=8)], receive_ns=EPOCH + 1_000_000_000)],
                              clock_mapper=MODULE.protocol.ClockMapper(document),
                              reference={(1, BOOT): [(mapped, 0)]})
    assert rows[0]["mapped_epoch_ns"] == mapped
    assert rows[0]["reference_match_dt_ns"] == 0
    assert report["streams"][0]["reference_comparison"]["matched_using_calibrated_acquisition_time"] == 1
    assert MODULE.protocol.ClockMapper(document).map(1, BOOT, 9) is None


def test_timestamp_gaps_and_nonmonotonic_node_clock_split_segments():
    rows, _, report = analyze([envelope([
        sample(0, t_us=1000), sample(1, t_us=900), sample(2, t_us=200_000)])])
    assert [row["segment_id"] for row in rows] == [0, 1, 2]
    assert report["streams"][0]["continuity_segments"] == 3


def test_index_count_consistency_uses_integer_subtraction():
    packet = envelope([sample(0)])["packet"]
    packet = {"v": 1, "type": "metadata", "node_id": 1, "boot_id": BOOT,
              "role": "H1", "firmware_version": "test", "idf_version": "test", "counts_per_rev": 8192}
    meta = envelope([sample(0)])
    meta["packet"] = packet
    base = 2 ** 53 + 123
    packets = [meta] + [envelope([sample(i, flags=GOOD | (1 << 9),
                                        index_event_count=i + 1, index_raw_count=str(base + i * 8192))])
                        for i in range(3)]
    _, _, report = analyze(packets)
    result = report["streams"][0]["index_consistency"]
    assert result["consecutive_index_intervals"] == 2
    assert result["net_count_delta_min"] == 8192
    assert result["net_count_delta_max"] == 8192
    assert result["mean_abs_delta_minus_configured_cpr"] == 0


def test_jsonl_packet_export_preserves_raw_and_integer_precision(tmp_path):
    from caster_bag_reader import read_recording
    from caster_bag_report import packet_reports
    packet = envelope([sample(0), sample(1, None, flags=0)])
    source, output = tmp_path / "packets.jsonl", tmp_path / "report"
    source.write_text(json.dumps(packet) + "\n", encoding="utf-8")
    packet_reports(read_recording(source), output, plots=False)
    packet_output = output / "encoder_packets/legacy"
    with (packet_output / "samples.csv").open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["raw_count"] == str(2 ** 53 + 123)
    assert rows[0]["receive_epoch_ns"] == str(EPOCH + 10_000_000)
    assert rows[1]["angle_rad"] == ""
    assert rows[1]["sample_valid"] == "0"
    assert json.loads((packet_output / "packets.jsonl").read_text()) == packet
    report = json.loads((packet_output / "summary.json").read_text())
    assert report["accuracy_status"] == "accuracy_unavailable"
    assert json.loads(source.read_text()) == packet


def test_help_runs_without_ros():
    result = subprocess.run([sys.executable, str(SCRIPT.with_name("caster_visualization.py")), "--help"], capture_output=True, text=True)
    assert result.returncode == 0
    assert "--ekf-alignment" in result.stdout


def test_missing_ros_has_actionable_error(monkeypatch, tmp_path):
    monkeypatch.setitem(sys.modules, "rosbag2_py", None)
    with pytest.raises(RuntimeError, match="packets.jsonl --render none"):
        list(MODULE.read_bag(tmp_path))


def test_bag_reader_retained_metadata_is_labeled_without_invented_receive_time(monkeypatch, tmp_path):
    metadata = {"v": 1, "type": "metadata", "node_id": 1, "boot_id": BOOT,
                "role": "H1", "firmware_version": "test", "idf_version": "test", "counts_per_rev": 8192}
    messages = [
        (MODULE.METADATA_TOPIC, json.dumps({"nodes": {"1": metadata}}), EPOCH),
        (MODULE.TOPIC, json.dumps(envelope([sample(0)])), EPOCH + 1),
    ]

    class Reader:
        def open(self, *args):
            pass

        def get_all_topics_and_types(self):
            return [SimpleNamespace(name=name, type="std_msgs/msg/String")
                    for name in (MODULE.TOPIC, MODULE.METADATA_TOPIC)]

        def set_filter(self, *args):
            pass

        def has_next(self):
            return bool(messages)

        def read_next(self):
            return messages.pop(0)

    monkeypatch.setitem(sys.modules, "rosbag2_py", SimpleNamespace(
        SequentialReader=Reader, StorageOptions=lambda **kw: kw,
        ConverterOptions=lambda *args: args, StorageFilter=lambda **kw: kw))
    monkeypatch.setitem(sys.modules, "rclpy.serialization", SimpleNamespace(
        deserialize_message=lambda value, _: SimpleNamespace(data=value)))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(String=object))
    records = list(MODULE.read_bag(tmp_path))
    assert len(records) == 2
    assert records[0][0]["receive_epoch_ns"] is None
    assert records[0][0]["snapshot_bag_timestamp_ns"] == EPOCH
    _, metadata_rows, report = MODULE.analyze(records)
    assert metadata_rows[0]["packet"] == metadata
    assert report["streams"][0]["index_consistency"]["configured_counts_per_rev"] == 8192


def test_plots_render_complete_output_when_matplotlib_available(tmp_path):
    try:
        import matplotlib
    except ImportError:
        pytest.skip("matplotlib unavailable or incompatible in this interpreter")
    rows, _, _ = analyze([envelope([sample(i, .1 + i * .01)]) for i in range(3)],
                          reference={(1, BOOT): [(EPOCH + i * 10_000_000, i * .01) for i in range(3)]})
    outputs = MODULE.plot_report(rows, tmp_path)
    assert len(outputs) == 3
    for name in outputs:
        assert (tmp_path / name).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def v2(value, wheel="front_left", device="aa:bb:cc:dd:ee:01"):
    value["packet"].update(v=2, wheel_id=wheel, device_id=device)
    return value


def test_v2_analysis_exports_identity_and_rejects_mixed_wheels_or_devices():
    left = v2(envelope([sample(0)]))
    rows, _, summary = analyze([left], wheel_id="front_left")
    assert rows[0]["wheel_id"] == summary["wheel_id"] == "front_left"
    assert rows[0]["device_id"] == summary["streams"][0]["device_id"] == "aa:bb:cc:dd:ee:01"
    with pytest.raises(ValueError, match="Expected wheel front_left"):
        analyze([left, v2(envelope([sample(0)]), wheel="front_right")])
    with pytest.raises(ValueError, match="changed device identity"):
        analyze([left, v2(envelope([sample(1)]), device="aa:bb:cc:dd:ee:02")])
    with pytest.raises(ValueError, match="Expected wheel front_right"):
        analyze([left], wheel_id="front_right")


def test_v2_reference_and_clock_maps_require_the_entire_physical_identity(tmp_path):
    packet = v2(envelope([sample(0, angle=.5)]))
    path = tmp_path / "reference.csv"
    path.write_text("wheel_id,device_id,node_id,boot_id,time_epoch_ns,angle_rad\n"
                    f"front_right,aa:bb:cc:dd:ee:01,1,{BOOT},{EPOCH},.5\n"
                    f"front_left,aa:bb:cc:dd:ee:01,1,{BOOT},{EPOCH},.25\n")
    reference = MODULE.read_reference(path)
    mapping = {"node_id": 1, "boot_id": BOOT, "calibrated": True,
               "slope_ns_per_us": "1000", "offset_ns": EPOCH,
               "valid_from_us": 0, "valid_to_us": 1000000}
    mapper = MODULE.protocol.ClockMapper({"version": 1, "mappings": [mapping]})
    rows, _, _ = analyze([packet], reference=reference, clock_mapper=mapper)
    assert rows[0]["angle_error_rad"] == .25
    assert rows[0]["mapped_epoch_ns"] is None
    rows, _, _ = analyze([packet], reference={(1, BOOT): [(EPOCH, .5)]})
    assert rows[0]["reference_angle_rad"] is None
    mapping.update(wheel_id="front_left", device_id="aa:bb:cc:dd:ee:01")
    rows, _, _ = analyze([packet], clock_mapper=MODULE.protocol.ClockMapper({"version": 1, "mappings": [mapping]}))
    assert rows[0]["mapped_epoch_ns"] == EPOCH


def test_v2_bag_reads_only_sibling_retained_metadata(monkeypatch, tmp_path):
    packet_topic = "/caster_encoder/front_right/packets"
    retained_topic = "/caster_encoder/front_right/metadata"
    metadata = {"v": 2, "wheel_id": "front_right", "device_id": "aa:bb:cc:dd:ee:01",
                "type": "metadata", "node_id": 1, "boot_id": BOOT, "role": "H1",
                "firmware_version": "test", "idf_version": "test"}
    messages = [
        ("/caster_encoder/front_left/metadata", json.dumps({"nodes": {"1": {**metadata, "wheel_id": "front_left"}}}), EPOCH),
        (retained_topic, json.dumps({"nodes": {"1": metadata}}), EPOCH),
        (packet_topic, json.dumps(v2(envelope([sample(0)]), wheel="front_right")), EPOCH + 1),
    ]
    selected = []
    class Reader:
        def open(self, *args): pass
        def get_all_topics_and_types(self):
            return [SimpleNamespace(name=name, type="std_msgs/msg/String")
                    for name in [packet_topic, retained_topic, "/caster_encoder/front_left/packets", "/caster_encoder/front_left/metadata"]]
        def set_filter(self, value):
            selected.extend(value["topics"])
            messages[:] = [message for message in messages if message[0] in selected]
        def has_next(self): return bool(messages)
        def read_next(self): return messages.pop(0)
    monkeypatch.setitem(sys.modules, "rosbag2_py", SimpleNamespace(
        SequentialReader=Reader, StorageOptions=lambda **kw: kw,
        ConverterOptions=lambda *args: args, StorageFilter=lambda **kw: kw))
    monkeypatch.setitem(sys.modules, "rclpy.serialization", SimpleNamespace(
        deserialize_message=lambda value, _: SimpleNamespace(data=value)))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(String=object))
    records = list(MODULE.read_bag(tmp_path, wheel_id="front_right"))
    assert selected == [packet_topic, retained_topic]
    rows, metadata_rows, summary = MODULE.analyze(records, wheel_id="front_right")
    assert len(rows) == len(metadata_rows) == 1
    assert metadata_rows[0]["metadata_source_topic"] == retained_topic
    assert summary["streams"][0]["metadata_entries"] == 1
