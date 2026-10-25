"""Exercise real HTTP/download lifecycle against isolated loopback recorders."""

from contextlib import ExitStack
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import time
import urllib.parse
import zlib

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import caster_recording_client as client
import caster_recording_format as binary


def make_log(device, run="trial"):
    metadata = {"run_id": run, "packet": {
        "v": 2, "type": "metadata", "wheel_id": device["wheel_id"],
        "device_id": device["device_id"], "node_id": device["node_id"],
        "boot_id": device["boot_id"], "role": client.ROLES[device["node_id"]],
        "firmware_version": "test", "idf_version": "test", "sample_hz": 200}}
    encoded = json.dumps(metadata, separators=(",", ":")).encode()
    header = binary.HEADER.pack(binary.MAGIC, 1, binary.RECORD.size, len(encoded), zlib.crc32(encoded)) + encoded
    header += b"\x00" * (binary.HEADER_BYTES - len(header))
    records = []
    for index in range(3):
        row = binary.RECORD.pack(index, 10000 + index * 5000, index, 0, -1, 3 | 16 | 64,
                                 index * .1, float("nan"), float("nan"), .2, .2,
                                 0, 0, 0, 0)
        records.append(row[:-4] + zlib.crc32(row[:-4]).to_bytes(4, "little"))
    payload = b"".join(records)
    return header + payload, zlib.crc32(payload)


class Recorder:
    def __init__(self, number=1, wheel="front_left"):
        self.device = dict(v=1, wheel_id=wheel, node_id=number,
                           device_id=f"aa:bb:cc:dd:{0 if wheel == 'front_left' else 1:02x}:{number:02x}",
                           boot_id="0123456789abcdef")
        self.state, self.run, self.acknowledged = "idle", None, False
        self.data, self.crc = b"", 0
        self.requests = []
        self.fail_start = self.fail_ack = self.bad_crc = self.wrong_identity = False
        self.fail_downloads = 0
        self.incomplete_downloads = 0
        self.footer_error = False
        self.before_ack = None
        self.log_boot = None
        self.stop_reason = "command"
        self.server = None

    def status(self):
        result = dict(self.device, state=self.state, run_id=self.run,
                      log_boot_id=self.log_boot,
                      sample_count=3 if self.data else 0, data_bytes=len(self.data),
                      crc32=self.crc, capacity_samples=20000, sample_hz=200,
                      start_us=10000 if self.run else None, stop_us=20000 if self.state == "stopped" else None,
                      stop_reason=self.stop_reason, acknowledged=self.acknowledged,
                      diagnostics={name: 0 for name in binary.protocol.DIAGNOSTIC_FIELDS})
        if self.wrong_identity:
            result["device_id"] = "00:11:22:33:44:55"
        return result

    def __enter__(self):
        recorder = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def send(self, data, status=200, binary_data=False):
                body = data if binary_data else json.dumps(data).encode()
                self.send_response(status)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                route = urllib.parse.urlparse(self.path).path
                recorder.requests.append(route)
                if route == "/api/log":
                    if recorder.fail_downloads:
                        recorder.fail_downloads -= 1
                        return self.send({"error": "retry"}, 503)
                    if recorder.incomplete_downloads:
                        recorder.incomplete_downloads -= 1
                        self.send_response(200)
                        self.send_header("Content-Length", str(len(recorder.data)))
                        self.end_headers()
                        self.wfile.write(recorder.data[:20])
                        self.close_connection = True
                        return
                    data = recorder.data[:-1] + bytes([recorder.data[-1] ^ 1]) if recorder.bad_crc else recorder.data
                    return self.send(data, binary_data=True)
                if route == "/api/sync":
                    now = time.monotonic_ns() // 1000
                    return self.send(dict(recorder.device, rx_us=now, tx_us=now + 1))
                self.send(recorder.status())

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                recorder.requests.append(self.path)
                if self.path == "/api/prepare":
                    recorder.run, recorder.state = body["run_id"], "prepared"
                    recorder.log_boot = recorder.device["boot_id"]
                    recorder.data, recorder.crc = make_log(recorder.device, recorder.run)
                elif self.path == "/api/start":
                    # Model a lost/error response after the command took effect.
                    recorder.state = "recording"
                    if recorder.fail_start:
                        return self.send({"error": "start response failed"}, 503)
                elif self.path == "/api/stop":
                    recorder.state = "error" if recorder.footer_error else "stopped"
                elif self.path == "/api/ack":
                    if recorder.fail_ack:
                        return self.send({"error": "ack unavailable"}, 503)
                    assert body["crc32"] == recorder.crc
                    if recorder.before_ack:
                        recorder.before_ack()
                    recorder.acknowledged = True
                self.send(recorder.status())

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.daemon_threads = True
        self.device.update(host="127.0.0.1", http_port=self.server.server_port)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        return self

    def __exit__(self, *args):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()


def session(tmp_path, recorders, **kwargs):
    return client.Session.create(tmp_path, "trial", [recorder.device for recorder in recorders],
                                 transport=client.Transport(timeout=2, retries=2), sync_samples=1, **kwargs)


def test_full_six_board_start_stop_download_export_and_repeat(tmp_path):
    with ExitStack() as stack:
        recorders = [stack.enter_context(Recorder(node, wheel)) for wheel in client.WHEELS for node in client.ROLES]
        devices = client.validate_inventory([recorder.device for recorder in recorders])
        assert len(devices) == 6
        run = session(tmp_path, recorders)
        run.start()
        assert all(recorder.state == "recording" for recorder in recorders)
        run.stop()
        assert run.manifest["state"] == "complete"
        assert all(item["verified"]["sample_count"] == 3 and item["acknowledged"] for item in run.manifest["devices"])
        assert (run.directory / "samples.csv").exists()
        requests = [len(recorder.requests) for recorder in recorders]
        # Complete local recordings can be re-exported without touching a newer
        # recorder run or pretending download times were acquisition times.
        loaded = client.Session.load(tmp_path, "trial", sync_samples=1)
        loaded.fetch()
        assert requests == [len(recorder.requests) for recorder in recorders]
        lines = [json.loads(line) for line in (run.directory / "packets.jsonl").read_text().splitlines()]
        assert all(line.get("receive_epoch_ns") is None for line in lines)


def test_sidecar_preserves_existing_robot_bag_and_start_is_exclusive(tmp_path):
    bag = tmp_path / "robot_bag"
    bag.mkdir()
    marker = bag / "metadata.yaml"
    marker.write_text("original")
    with Recorder() as recorder:
        run = session(tmp_path, [recorder], bag=bag)
        assert run.directory == tmp_path / "encoder_trial"
        assert run.manifest["bag_path"] == str(bag.resolve())
        with pytest.raises(FileExistsError):
            session(tmp_path, [recorder])
        assert marker.read_text() == "original"


def test_partial_start_attempts_stop_on_every_board_and_preserves_manifest(tmp_path):
    with Recorder(1) as first, Recorder(2) as second:
        second.fail_start = True
        run = session(tmp_path, [first, second])
        with pytest.raises(client.RecordingError, match="start failed"):
            run.start()
        assert first.state == second.state == "stopped"
        assert "/api/stop" in first.requests and "/api/stop" in second.requests
        saved = json.loads((run.directory / "manifest.json").read_text())
        assert saved["state"] == "start_failed"
        assert all(item["sync_pre"] for item in saved["devices"])
        # Retrying start cannot wipe the first trace. Recovery stop still fetches.
        with pytest.raises(client.RecordingError, match="already exists"):
            run.start()
        run.stop()
        assert run.manifest["state"] == "complete"


def test_crc_failure_never_acknowledges_then_fetch_recovers(tmp_path):
    with Recorder() as recorder:
        run = session(tmp_path, [recorder])
        run.start()
        recorder.bad_crc = True
        with pytest.raises(client.RecordingError, match="CRC mismatch"):
            run.stop()
        assert not recorder.acknowledged
        assert "/api/ack" not in recorder.requests
        assert not (run.directory / run.manifest["devices"][0]["log_path"]).exists()
        recorder.bad_crc = False
        client.Session.load(tmp_path, "trial", transport=client.Transport(timeout=2)).fetch()
        assert recorder.acknowledged


def test_download_retry_and_ack_retry_reuse_verified_file(tmp_path):
    with Recorder() as recorder:
        run = session(tmp_path, [recorder])
        run.start()
        recorder.fail_downloads, recorder.fail_ack = 1, True
        with pytest.raises(client.RecordingError, match="ack unavailable"):
            run.stop()
        assert recorder.requests.count("/api/log") == 2
        assert run.manifest["devices"][0]["verified"]
        recorder.fail_ack = False
        run.fetch()
        assert recorder.requests.count("/api/log") == 2
        assert recorder.acknowledged


def test_changed_device_identity_rejected_before_download_or_ack(tmp_path):
    with Recorder() as recorder:
        run = session(tmp_path, [recorder])
        run.start()
        recorder.wrong_identity = True
        with pytest.raises(client.RecordingError, match="identity changed"):
            run.stop()
        assert "/api/log" not in recorder.requests and "/api/ack" not in recorder.requests


def test_capacity_failure_never_prepares_or_starts(tmp_path):
    with Recorder() as recorder:
        run = session(tmp_path, [recorder], planned_duration=101)
        with pytest.raises(client.RecordingError, match="does not fit"):
            run.start()
        assert "/api/start" not in recorder.requests
        assert "/api/prepare" not in recorder.requests
        assert "/api/stop" not in recorder.requests


def test_early_capacity_stop_preserves_download_but_is_not_success(tmp_path):
    with Recorder() as recorder:
        run = session(tmp_path, [recorder])
        run.start()
        recorder.stop_reason = "capacity"
        with pytest.raises(client.RecordingError, match="stopped early: capacity"):
            run.stop()
        assert recorder.acknowledged
        assert run.manifest["devices"][0]["verified"]
        assert run.manifest["state"] == "incomplete"


def test_inventory_rejects_ambiguity_missing_roles_and_wrong_pins():
    first = dict(Recorder().device, host="127.0.0.1", http_port=8080)
    second = dict(first, device_id="01:02:03:04:05:06")
    with pytest.raises(ValueError, match="ambiguous"):
        client.validate_inventory([first, second], require_all=False)
    with pytest.raises(ValueError, match="missing recorders"):
        client.validate_inventory([first])
    with pytest.raises(ValueError, match="MAC inventory mismatch"):
        client.validate_inventory([first], inventory=[dict(second)], require_all=False)
    assert len(client.validate_inventory([first, first], require_all=False)) == 1


@pytest.mark.parametrize("value", ["../escape", "abc/def", "abc\\def", "", ".hidden", "a" * 65])
def test_run_id_cannot_escape_sidecar(value):
    with pytest.raises(ValueError):
        client.run_id(value)


def test_durable_manifest_exists_before_first_mutating_request(tmp_path):
    class InspectingTransport:
        def request(self, device, path, body=None):
            saved = json.loads((tmp_path / "encoder_trial/manifest.json").read_text())
            assert saved["run_id"] == "trial"
            assert saved["devices"][0]["device_id"] == device["device_id"]
            if path == "/api/status":
                return dict(device, state="idle", run_id=None, sample_hz=200, capacity_samples=20000)
            assert saved["state"] in ("preparing", "start_failed_stopping")
            raise client.RecordingError("offline")
    device = dict(Recorder().device, host="127.0.0.1", http_port=8080)
    run = client.Session.create(tmp_path, "trial", [device], transport=InspectingTransport())
    with pytest.raises(client.RecordingError):
        run.start()


def test_endpoint_refresh_keeps_recording_boot_and_rejects_replacement(tmp_path):
    with Recorder() as recorder:
        run = session(tmp_path, [recorder])
        changed = dict(recorder.device, host="127.0.0.2", http_port=8765, boot_id="fedcba9876543210")
        run.refresh_endpoints([changed])
        saved = run.manifest["devices"][0]
        assert saved["host"] == "127.0.0.2" and saved["http_port"] == 8765
        assert saved["boot_id"] == "0123456789abcdef"
        with pytest.raises(ValueError, match="identity changed"):
            run.refresh_endpoints([dict(changed, device_id="00:11:22:33:44:55")])


def test_run_lock_rejects_concurrent_file_mutation(tmp_path):
    with client.run_lock(tmp_path):
        with pytest.raises(client.RecordingError, match="another caster_recording"):
            with client.run_lock(tmp_path):
                pytest.fail("second operation obtained the same lock")
    with client.run_lock(tmp_path):
        pass


def test_error_footer_recovers_and_acknowledges_records_but_reports_failure(tmp_path):
    with Recorder() as recorder:
        run = session(tmp_path, [recorder])
        run.start()
        recorder.footer_error, recorder.stop_reason = True, "footer_write_error"
        with pytest.raises(client.RecordingError, match="footer_write_error"):
            run.stop()
        assert recorder.acknowledged
        assert run.manifest["devices"][0]["verified"]["sample_count"] == 3


def test_reboot_recovers_previous_boot_without_claiming_clock_alignment(tmp_path):
    with Recorder() as recorder:
        run = session(tmp_path, [recorder])
        run.start()
        recorder.device["boot_id"] = "fedcba9876543210"
        recorder.stop_reason = "interrupted"
        with pytest.raises(client.RecordingError, match="interrupted"):
            run.stop()
        item = run.manifest["devices"][0]
        assert item["verified"]["boot_id"] == "0123456789abcdef"
        assert item["sync_post"] == [] and item["acknowledged"]
        assert run.manifest["exports"]["clock_errors"]


def test_partial_download_still_exports_successful_device(tmp_path):
    with Recorder(1) as first, Recorder(2) as second:
        run = session(tmp_path, [first, second])
        run.start()
        second.bad_crc = True
        with pytest.raises(client.RecordingError, match="CRC mismatch"):
            run.stop()
        assert first.acknowledged and not second.acknowledged
        assert len(run.manifest["exports"]["nodes"]) == 1
        assert len(run.manifest["exports"]["incomplete_devices"]) == 1


def test_truncated_transfer_retries_and_ack_sees_durable_status_and_file(tmp_path):
    with Recorder() as recorder:
        run = session(tmp_path, [recorder])
        run.start()
        recorder.incomplete_downloads = 1
        seen = []
        def before_ack():
            saved = json.loads((run.directory / "manifest.json").read_text())
            item = saved["devices"][0]
            assert item["verified"]["sample_count"] == 3
            assert item["status"]["crc32"] == recorder.crc
            assert (run.directory / item["log_path"]).read_bytes() == recorder.data
            seen.append(True)
        recorder.before_ack = before_ack
        run.stop()
        assert seen and recorder.requests.count("/api/log") == 2
