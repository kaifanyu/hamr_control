"""Physical-turn analysis uses raw counts and device time, without ROS."""
import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location("rotation_recorder", SCRIPTS / "record_caster_rotation.py")
rotation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(rotation)
BOOT = "0123456789abcdef"
Q = math.tau / 8192


def envelope(count, seq, *, node=1, boot=BOOT, timestamp=None, flags=83,
             index_event=0, index_time=0, index_count=None):
    sample = {
        "seq": seq, "t_us": timestamp if timestamp is not None else seq * 5000,
        "raw_count": str(count) if count is not None else None,
        "raw_spi_word": None, "position14": None,
        "angle_rad": count * Q if count is not None else None,
        "absolute_angle_rad": None, "calibrated_alpha_rad": None,
        "velocity_rad_s": Q / .005, "filtered_velocity_rad_s": Q / .005,
        "flags": flags, "index_event_count": index_event,
        "index_timestamp_us": index_time,
        "index_raw_count": str(index_count) if index_count is not None else None,
    }
    return json.dumps({"receive_epoch_ns": 100, "receive_ros_ns": 100,
                       "source_ip": "127.0.0.1", "packet": {
                           "v": 1, "type": "samples", "node_id": node, "boot_id": boot,
                           "samples": [sample],
                           "diagnostics": dict.fromkeys(rotation.DIAGNOSTIC_FIELDS, 0)}})


def metadata(ppr=2048, node=1):
    return json.dumps({"packet": {
        "v": 1, "type": "metadata", "node_id": node, "boot_id": BOOT,
        "role": "H1" if node == 1 else "H2", "firmware_version": "0.1.0",
        "idf_version": "v5.4.2", "encoder_ppr": ppr, "coordinate_sign": 1,
        "software_zero_count": "0", "ppr_verified": False}})


class RotationTests(unittest.TestCase):
    def make(self, role="H1", turns=1):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        r = rotation.RotationRecording(role, turns, directory.name)
        self.addCleanup(r.raw_file.close)
        self.addCleanup(r.csv_file.close)
        return r

    def capture(self, counts, *, role="H1", turns=1, ppr=2048):
        r = self.make(role, turns)
        r.ingest(metadata(ppr, r.node_id))
        # Joining a stream at sequence 1000 must not count previous samples as lost.
        for i, count in enumerate(counts):
            r.ingest(envelope(count, i + 1000, node=r.node_id), now=i)
            if i == 0:
                r.mark(now=i)
        r.mark(now=len(counts) - 1)
        return r

    def test_forward_reverse_and_multiple_turns(self):
        for role, sign, turns in (("H1", 1, 1), ("H2", -1, 1), ("H1", 1, 3)):
            with self.subTest(role=role, sign=sign, turns=turns):
                r = self.capture([120, 120 + sign * 8192 * turns], role=role, turns=turns)
                s, _ = r.close()
                self.assertTrue(s["measurement_valid"])
                self.assertEqual(s["observed_counts_per_rev"], 8192)
                self.assertEqual(s["count_scale_ratio"], 1)
                self.assertEqual(s["missing_sequences"], 0)
                self.assertEqual(s["reverse_counts"], 0)
                self.assertAlmostEqual(s["angle_change_rad"], sign * turns * math.tau)

    def test_half_scale_and_metadata_expected_ppr(self):
        for ppr, ratio in ((2048, .5), (1024, 1)):
            r = self.capture([0, 4096], ppr=ppr)
            s, _ = r.close()
            self.assertEqual(s["observed_counts_per_rev"], 4096)
            self.assertEqual(s["observed_ppr_equivalent"], 1024)
            self.assertEqual(s["count_scale_ratio"], ratio)

    def test_exact_int64_counts(self):
        start = 2**60
        r = self.capture([start, start + 8192])
        s, _ = r.close()
        self.assertEqual(s["net_counts"], 8192)

    def test_reordered_duplicates_and_missing_sequences(self):
        r = self.make()
        r.ingest(envelope(0, 1000), now=0)
        r.mark(now=0)
        r.ingest(envelope(8192, 1004), now=1)
        r.ingest(envelope(4096, 1002), now=1)
        r.ingest(envelope(4096, 1002), now=1)
        r.mark(now=1)
        s, _ = r.close()
        self.assertTrue(s["measurement_valid"])
        self.assertEqual(s["net_counts"], 8192)
        self.assertEqual(s["missing_sequences"], 2)
        self.assertEqual(s["duplicate_samples"], 1)
        self.assertEqual(s["reverse_counts"], 0)

    def test_other_role_does_not_change_markers(self):
        r = self.capture([0, 8192])
        r.ingest(envelope(123, 9000, node=2), now=2)
        self.assertEqual(r.latest["raw_count"], "8192")
        self.assertEqual(len(r.rows), 2)

    def test_device_time_not_identical_ros_receipt_time(self):
        r = self.capture([0, 1])
        s, _ = r.close()
        self.assertAlmostEqual(s["mean_velocity_rad_s"], .15339807878856412)
        self.assertEqual(s["sample_interval_median_ms"], 5)

    def test_invalid_snapshot_cannot_be_an_endpoint(self):
        r = self.make()
        r.ingest(envelope(None, 1000, flags=1), now=0)
        with self.assertRaises(ValueError):
            r.mark(now=0)
        r.ingest(envelope(0, 1001), now=1)
        r.mark(now=1)
        r.ingest(envelope(None, 1002, flags=1), now=2)
        with self.assertRaisesRegex(ValueError, "newest sample"):
            r.mark(now=2)
        r.ingest(envelope(8192, 1003), now=3)
        r.mark(now=3)
        s, _ = r.close()
        self.assertEqual(s["invalid_count_samples"], 1)
        self.assertEqual(s["net_counts"], 8192)

    def test_stale_duplicate_does_not_refresh_endpoint(self):
        r = self.make()
        r.ingest(envelope(0, 1000), now=0)
        r.mark(now=0)
        r.ingest(envelope(0, 1000), now=5)
        with self.assertRaisesRegex(ValueError, "fresh"):
            r.mark(now=5)

    def test_reboot_invalidates_measurement(self):
        r = self.capture([0, 8192])
        r.ingest(envelope(0, 0, boot="fedcba9876543210"), now=2)
        s, _ = r.close()
        self.assertFalse(s["measurement_valid"])
        self.assertIsNone(s["observed_counts_per_rev"])

    def test_continuity_loss_and_conflicting_samples(self):
        for bad in (envelope(1, 1001, flags=339), envelope(7, 1000)):
            r = self.capture([0, 1, 8192])
            r.ingest(bad, now=2)
            s, _ = r.close()
            self.assertFalse(s["measurement_valid"])

    def test_reverse_counts_and_zero_motion_are_visible(self):
        r = self.capture([0, 1, 0, 2, 0])
        s, _ = r.close()
        self.assertEqual(s["reverse_counts"], 3)
        self.assertEqual(s["net_counts"], 0)
        self.assertTrue(any("No net counts" in w for w in s["warnings"]))

    def test_index_to_index_measurement(self):
        r = self.make()
        r.ingest(envelope(0, 1000), now=0)
        r.mark(now=0)
        r.ingest(envelope(100, 1001, flags=595, index_event=1,
                          index_time=5005000, index_count=100), now=1)
        r.ingest(envelope(8292, 1002, flags=595, index_event=2,
                          index_time=5010000, index_count=8292), now=2)
        r.mark(now=2)
        s, _ = r.close()
        self.assertEqual(s["index_intervals"], [{"events": 1, "counts": 8192}])

    def test_partial_run_preserves_logs_without_calibration(self):
        r = self.make()
        r.ingest(envelope(0, 1000), now=0)
        r.mark(now=0)
        s, _ = r.close()
        self.assertFalse(s["measurement_valid"])
        for name in ("packets.jsonl", "samples.csv", "markers.json", "summary.json", "summary.txt"):
            self.assertTrue((r.path / name).exists())
        self.assertNotIn("observed_counts_per_rev", s)

    def test_compare_saved_runs(self):
        a, b = self.capture([0, 4096]), self.capture([0, 8192], role="H2")
        a.close()
        b.close()
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(rotation.main(["compare", str(a.path), str(b.path)]), 0)
        self.assertIn("H1/H2 counts-per-revolution ratio: 0.5", output.getvalue())

    def test_bad_json_does_not_poison_next_packet(self):
        r = self.make()
        r.ingest("not JSON")
        r.ingest(envelope(0, 1000), now=0)
        self.assertEqual(r.invalid_messages, 1)
        self.assertIsNotNone(r.latest)

    def test_interactive_ros_capture_flow(self):
        # Exercise the actual record loop with an injected ROS transport and
        # operator Enter events, including two samples sharing a receipt stamp.
        with tempfile.TemporaryDirectory() as directory:
            messages = [metadata(), envelope(0, 1000), envelope(8192, 1001)]
            fake_node = mock.Mock()
            callbacks = []
            fake_node.create_subscription.side_effect = lambda kind, topic, callback, qos: callbacks.append(callback)

            def spin_once(node, timeout_sec):
                callbacks[0](SimpleNamespace(data=messages.pop(0)))

            fake_ros = SimpleNamespace(init=mock.Mock(), create_node=lambda name: fake_node,
                                       spin_once=spin_once, ok=lambda: True, shutdown=mock.Mock())
            fake_qos = SimpleNamespace(
                QoSProfile=lambda **kwargs: kwargs,
                ReliabilityPolicy=SimpleNamespace(RELIABLE=1),
                DurabilityPolicy=SimpleNamespace(VOLATILE=0),
                HistoryPolicy=SimpleNamespace(KEEP_LAST=1))
            modules = {"rclpy": fake_ros, "rclpy.qos": fake_qos,
                       "std_msgs": SimpleNamespace(), "std_msgs.msg": SimpleNamespace(String=object)}
            with mock.patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()), \
                    mock.patch.object(rotation, "request_enter", side_effect=lambda commands, prompt: commands.put("enter")):
                status = rotation.main(["record", "H1", "--output", directory])
            self.assertEqual(status, 0)
            path = next(Path(directory).glob("*/summary.json"))
            self.assertEqual(json.loads(path.read_text())["observed_counts_per_rev"], 8192)
            fake_ros.shutdown.assert_called_once()

    def test_nonmonotonic_device_time_prevents_calibration(self):
        r = self.make()
        r.ingest(envelope(0, 1000, timestamp=5000), now=0)
        r.mark(now=0)
        r.ingest(envelope(8192, 1001, timestamp=5000), now=1)
        r.mark(now=1)
        s, _ = r.close()
        self.assertFalse(s["measurement_valid"])
        self.assertIsNone(s["observed_counts_per_rev"])


if __name__ == "__main__":
    unittest.main()
