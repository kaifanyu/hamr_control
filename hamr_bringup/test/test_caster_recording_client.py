"""Exercise real HTTP/download lifecycle against isolated loopback recorders."""

from contextlib import ExitStack
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
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


def saved_run(root, name, created, wheel="front_left"):
    run = client.Session.create(root, name, [Recorder(wheel=wheel).device])
    run.manifest["created_epoch_ns"] = created
    run.save()
    return run


def test_latest_uses_creation_time_including_failed_attempts_and_wheel_scope(tmp_path):
    old = saved_run(tmp_path, "trial_999", 100)
    new = saved_run(tmp_path, "trial_001", 200)
    new.phase("start_failed")
    saved_run(tmp_path, "other_wheel", 300, "front_right")
    os.utime(old.directory / "manifest.json", (time.time() + 1000,) * 2)
    assert client.latest_run(tmp_path) == "other_wheel"
    assert client.latest_run(tmp_path, "front_left") == "trial_001"


def test_latest_refuses_empty_corrupt_or_ambiguous_selection(tmp_path):
    with pytest.raises(client.RecordingError, match="No saved"):
        client.latest_run(tmp_path)
    saved_run(tmp_path, "one", 100)
    bad = tmp_path / "encoder_bad"
    bad.mkdir()
    with pytest.raises(client.RecordingError, match="specify a run ID"):
        client.latest_run(tmp_path)
    bad.rmdir()
    saved_run(tmp_path, "two", 100)
    with pytest.raises(client.RecordingError, match="ambiguous"):
        client.latest_run(tmp_path)


@pytest.mark.parametrize("command", ["stop", "fetch"])
def test_omitted_run_uses_latest_manifest_without_new_discovery_for_complete_runs(tmp_path, monkeypatch, command):
    saved_run(tmp_path, "old", 100)
    new = saved_run(tmp_path, "new", 200)
    new.manifest["devices"][0]["acknowledged"] = True
    new.save()
    monkeypatch.setattr(client, "discover", lambda *a, **kw: pytest.fail("unexpected discovery"))
    monkeypatch.setattr(client, "load_config", lambda path: {})
    selected = []
    monkeypatch.setattr(client.Session, command, lambda self: selected.append(self.manifest["run_id"]))
    assert client.main(["--root", str(tmp_path), command]) == 0
    assert selected == ["new"]


def test_latest_cli_is_offline_and_shows_bag(tmp_path, monkeypatch, capsys):
    run = saved_run(tmp_path, "new", 100)
    monkeypatch.setattr(client, "load_config", lambda *a: pytest.fail("offline command loaded network config"))
    assert client.main(["--root", str(tmp_path), "latest"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["run_id"] == "new"
    assert report["directory"] == str(run.directory)


def test_automatic_start_associates_latest_actual_bag_path(tmp_path, monkeypatch):
    bag = tmp_path / "bag with spaces_001"
    bag.mkdir()
    (bag / "recording.mcap").touch()
    client.write_json(tmp_path / ".latest_hamr_bag.json", {"version": 1, "bag_path": str(bag)})
    devices = [Recorder(number).device for number in (1, 2, 3)]
    monkeypatch.setattr(client, "discover", lambda *a, **kw: devices)
    monkeypatch.setattr(client, "load_config", lambda path: {})
    selected = []
    monkeypatch.setattr(client.Session, "start", lambda self: selected.append(self.manifest))
    for _ in range(2):
        assert client.main(["--root", str(tmp_path), "--wheel", "front_left", "start", "--bag", "latest"]) == 0
    assert selected[0]["run_id"] != selected[1]["run_id"]
    assert all(item["bag_path"] == str(bag) for item in selected)
    assert all(client.run_id(item["run_id"]) for item in selected)


def test_latest_bag_does_not_fall_back_to_older_bag_if_latest_launch_failed(tmp_path):
    old = tmp_path / "old"
    old.mkdir()
    (old / "metadata.yaml").touch()
    with pytest.raises(client.RecordingError, match="No latest robot bag marker"):
        client.resolve_bag(tmp_path, Path("latest"))
    latest = tmp_path / "failed"
    client.write_json(tmp_path / ".latest_hamr_bag.json", {"version": 1, "bag_path": str(latest)})
    with pytest.raises(client.RecordingError, match="not ready"):
        client.resolve_bag(tmp_path, Path("latest"))
    latest.mkdir()
    with pytest.raises(client.RecordingError, match="not ready"):
        client.resolve_bag(tmp_path, Path("latest"))
    (latest / "bag.db3").touch()
    assert client.resolve_bag(tmp_path, Path("latest")) == latest


@pytest.mark.parametrize("reference_count", [0, 100])
def test_timing_reports_estimates_without_claiming_synchronization(tmp_path, monkeypatch, capsys, reference_count):
    run = saved_run(tmp_path, "trial", 100)
    bag = tmp_path / "robot"
    bag.mkdir()
    run.manifest["bag_path"] = str(bag)
    run.save()
    # JSON is also YAML; keep fixture data small and independent of ROS.
    client.write_json(bag / "metadata.yaml", {"rosbag2_bagfile_information": {
        "topics_with_message_count": [{"topic_metadata": {"name": "/reference_trajectory"},
                                       "message_count": reference_count}]}})
    mapping = dict(run.manifest["devices"][0], calibrated=True, uncertainty_ns=16000000,
                   max_probe_residual_ns=30000000, accuracy_validated=False)
    client.write_json(run.directory / "clock-map.json", {"mappings": [mapping], "clock_errors": []})
    monkeypatch.setattr(client, "load_config", lambda *a: pytest.fail("offline command loaded network config"))
    assert client.main(["--root", str(tmp_path), "timing"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["reference_message_count"] == reference_count
    assert report["accuracy_validated"] is False
    assert report["devices"][0]["uncertainty_indicator_ms"] == 16
    assert report["devices"][0]["max_probe_residual_ms"] == 30
    assert any("No /reference_trajectory" in item for item in report["warnings"]) == (reference_count == 0)
    assert "does not prove full time coverage" in report["timing_note"]


def test_timing_handles_missing_exports_and_unfinalized_bag(tmp_path):
    run = saved_run(tmp_path, "trial", 100)
    run.manifest["bag_path"] = str(tmp_path / "active_bag")
    report = client.timing_report(run)
    assert report["reference_message_count"] is None
    assert report["devices"][0]["clock_estimate_available"] is False
    assert report["devices"][0]["uncertainty_indicator_ms"] is None
    assert len(report["warnings"]) == 2


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
        self.bodies = []
        self.fail_start = self.fail_ack = self.bad_crc = self.wrong_identity = False
        self.fail_stop = self.finalizing = False
        self.supports_discard_existing = True
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
                      finalizing=self.finalizing, supports_discard_existing=self.supports_discard_existing,
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
                recorder.bodies.append((self.path, body))
                if self.path == "/api/prepare":
                    discard = body.get("discard_existing", False)
                    if discard:
                        if (not recorder.supports_discard_existing or
                                body.get("expected_device_id") != recorder.device["device_id"] or
                                body.get("expected_boot_id") != recorder.device["boot_id"]):
                            return self.send({"error": "discard unsupported or identity changed"}, 409)
                    if recorder.run == body["run_id"]:
                        return self.send(recorder.status())
                    if recorder.state == "recording" or recorder.finalizing:
                        return self.send({"error": "still recording/finalizing"}, 409)
                    if discard:
                        if "expected_run_id" not in body or body["expected_run_id"] != recorder.run:
                            return self.send({"error": "retained run changed"}, 409)
                    elif recorder.state != "idle" and not recorder.acknowledged:
                        return self.send({"error": "unread recording"}, 409)
                    recorder.run, recorder.state = body["run_id"], "prepared"
                    recorder.acknowledged = False
                    recorder.log_boot = recorder.device["boot_id"]
                    recorder.data, recorder.crc = make_log(recorder.device, recorder.run)
                elif self.path == "/api/start":
                    # Model a lost/error response after the command took effect.
                    recorder.state = "recording"
                    if recorder.fail_start:
                        return self.send({"error": "start response failed"}, 503)
                elif self.path == "/api/stop":
                    if "expected_device_id" in body or "expected_boot_id" in body:
                        if (body.get("expected_device_id") != recorder.device["device_id"] or
                                body.get("expected_boot_id") != recorder.device["boot_id"]):
                            return self.send({"error": "stop identity changed"}, 409)
                    if body["run_id"] != recorder.run:
                        return self.send({"error": "wrong run"}, 409)
                    if recorder.fail_stop:
                        return self.send({"error": "stop unavailable"}, 503)
                    recorder.state = "error" if recorder.footer_error else "stopped"
                    recorder.finalizing = False
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


def test_invalid_header_diagnosis_is_preserved_and_default_never_mutates(tmp_path):
    with Recorder() as recorder:
        recorder.state, recorder.stop_reason = "error", "invalid_header"
        run = session(tmp_path, [recorder])
        with pytest.raises(client.RecordingError, match="invalid_header"):
            run.start()
        assert recorder.bodies == []
        saved = json.loads((run.directory / "manifest.json").read_text())
        assert saved["devices"][0]["preflight_status"]["stop_reason"] == "invalid_header"


@pytest.mark.parametrize("state,old_run", [("error", None), ("stopped", "unread"), ("idle", None)])
def test_explicit_discard_recovers_storage_and_saves_authorization(tmp_path, state, old_run):
    with Recorder() as recorder:
        recorder.state, recorder.run = state, old_run
        recorder.stop_reason = "invalid_header" if old_run is None and state == "error" else "command"
        run = session(tmp_path, [recorder], discard_existing=True)
        run.start()
        assert recorder.state == "recording" and recorder.run == "trial"
        prepare = next(body for path, body in recorder.bodies if path == "/api/prepare")
        assert prepare == dict(run_id="trial", discard_existing=True,
                               expected_device_id=recorder.device["device_id"],
                               expected_boot_id=recorder.device["boot_id"], expected_run_id=old_run)
        assert "/api/stop" not in recorder.requests
        saved = json.loads((run.directory / "manifest.json").read_text())
        assert saved["discard_existing"] is True
        assert saved["devices"][0]["preflight_status"]["run_id"] == old_run


@pytest.mark.parametrize("old_state,finalizing", [("recording", False), ("prepared", False), ("recording", True)])
def test_discard_stops_old_run_before_prepare_without_fetching(tmp_path, old_state, finalizing):
    with Recorder() as recorder:
        recorder.state, recorder.run, recorder.finalizing = old_state, "old-run", finalizing
        run = session(tmp_path, [recorder], discard_existing=True)
        run.start()
        assert recorder.bodies[0] == ("/api/stop", {
            "run_id": "old-run", "expected_device_id": recorder.device["device_id"],
            "expected_boot_id": recorder.device["boot_id"]})
        assert [path for path, _ in recorder.bodies] == ["/api/stop", "/api/prepare", "/api/start"]
        assert recorder.bodies[1][1]["expected_run_id"] == "old-run"
        assert "/api/log" not in recorder.requests and "/api/ack" not in recorder.requests


@pytest.mark.parametrize("failure", ["old_firmware", "capacity", "same_run", "invalid_active_run",
                                     "missing_log_partition", "journal_unavailable", "journal_read_error"])
def test_discard_fleet_preflight_failure_never_stops_or_erases_any_board(tmp_path, failure):
    with Recorder(1) as first, Recorder(2) as second:
        first.state, first.run = "recording", "old-run"
        options = {"discard_existing": True}
        if failure == "old_firmware":
            second.supports_discard_existing = False
        elif failure == "capacity":
            options["planned_duration"] = 101
        elif failure == "same_run":
            second.run = "trial"
        elif failure == "invalid_active_run":
            second.state, second.run = "recording", None
        else:
            second.state, second.stop_reason = "error", failure
        run = session(tmp_path, [first, second], **options)
        with pytest.raises(client.RecordingError, match="preflight failed"):
            run.start()
        assert first.bodies == second.bodies == []
        assert first.state == "recording"


def test_discard_stop_failure_never_erases_any_board(tmp_path):
    with Recorder(1) as first, Recorder(2) as second:
        first.state, first.run, first.fail_stop = "recording", "old-run", True
        second.state, second.stop_reason = "error", "invalid_header"
        run = session(tmp_path, [first, second], discard_existing=True)
        with pytest.raises(client.RecordingError, match="no prepare/erase was requested"):
            run.start()
        assert first.bodies == [("/api/stop", {
            "run_id": "old-run", "expected_device_id": first.device["device_id"],
            "expected_boot_id": first.device["boot_id"]})]
        assert second.bodies == []
        assert run.manifest["state"] == "start_failed"


def test_discard_guards_reject_run_that_changed_after_preflight(tmp_path):
    with Recorder() as recorder:
        recorder.state, recorder.run = "stopped", "old-run"
        run = session(tmp_path, [recorder], discard_existing=True)
        original_request = run.transport.request

        def change_run(device, path, body=None):
            if path == "/api/prepare":
                recorder.run = "someone-elses-run"
            return original_request(device, path, body)

        run.transport.request = change_run
        with pytest.raises(client.RecordingError, match="retained run changed"):
            run.start()
        assert recorder.run == "someone-elses-run"
        assert "/api/start" not in recorder.requests


def test_discard_partial_prepare_preserves_evidence_and_only_cancels_new_run(tmp_path):
    with Recorder(1) as first, Recorder(2) as second:
        for recorder in (first, second):
            recorder.state, recorder.run = "stopped", "old-run"
            recorder.data, recorder.crc = make_log(recorder.device, recorder.run)
        old_first = first.data
        run = session(tmp_path, [first, second], discard_existing=True)
        original_request = run.transport.request
        competing_data = []

        def competing_prepare(device, path, body=None):
            if path == "/api/prepare" and device["node_id"] == 2:
                second.run = "competing-run"
                second.data, second.crc = make_log(second.device, second.run)
                competing_data.append(second.data)
            return original_request(device, path, body)

        run.transport.request = competing_prepare
        with pytest.raises(client.RecordingError, match="onboard logs may already have been erased"):
            run.start()
        assert first.run == "trial" and first.state == "stopped" and first.data != old_first
        assert second.run == "competing-run" and second.data == competing_data[0]
        for recorder in (first, second):
            assert "/api/start" not in recorder.requests
            assert [body["run_id"] for path, body in recorder.bodies if path == "/api/stop"] == ["trial"]
        saved = json.loads((run.directory / "manifest.json").read_text())
        assert saved["state"] == "start_failed" and saved["discard_existing"] is True
        assert all(item["preflight_status"]["run_id"] == "old-run" for item in saved["devices"])


@pytest.mark.parametrize("changed", ["device_id", "boot_id"])
def test_discard_guards_do_not_stop_a_changed_device_or_boot(tmp_path, changed):
    with Recorder() as recorder:
        recorder.state, recorder.run = "recording", "old-run"
        run = session(tmp_path, [recorder], discard_existing=True)
        original_request = run.transport.request

        def change_identity(device, path, body=None):
            if path == "/api/stop":
                recorder.device[changed] = ("aa:bb:cc:dd:ee:ff" if changed == "device_id"
                                            else "fedcba9876543210")
            return original_request(device, path, body)

        run.transport.request = change_identity
        with pytest.raises(client.RecordingError, match="stop identity changed"):
            run.start()
        assert recorder.state == "recording" and recorder.run == "old-run"
        assert "/api/prepare" not in recorder.requests


def test_discard_cannot_overwrite_existing_local_sidecar(tmp_path):
    with Recorder() as recorder:
        original = session(tmp_path, [recorder])
        saved = (original.directory / "manifest.json").read_bytes()
        with pytest.raises(FileExistsError):
            session(tmp_path, [recorder], discard_existing=True)
        assert (original.directory / "manifest.json").read_bytes() == saved
        assert recorder.requests == []


def test_discard_cli_passes_flag_and_existing_bag_to_session(tmp_path, monkeypatch):
    devices = [dict(Recorder(node).device, host="127.0.0.1", http_port=8080) for node in client.ROLES]
    bag = tmp_path / "robot-bag"
    bag.mkdir()
    monkeypatch.setattr(client, "discover", lambda *args, **kwargs: devices)
    monkeypatch.setattr(client, "load_config", lambda path: {})
    observed = []
    monkeypatch.setattr(client.Session, "start", lambda self: observed.append(self.manifest))
    assert client.main(["--root", str(tmp_path), "--wheel", "front_left", "start", "fresh",
                        "--bag", str(bag), "--duration", "60", "--discard-existing"]) == 0
    assert observed[0]["discard_existing"] is True
    assert observed[0]["bag_path"] == str(bag.resolve())
    assert len(observed[0]["devices"]) == 3


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
