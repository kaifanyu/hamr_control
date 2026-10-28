"""Ordinary family selection and real loopback HTTP recording/recovery."""

from contextlib import ExitStack
import json
from pathlib import Path
import sys
import zlib

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import caster_recording_client as client
import caster_recording_format as binary
import test_caster_recording_client as ball_tests
from test_ordinary_recording_format import fixture


class Recorder(ball_tests.Recorder):
    def __init__(self, wheel="front_left"):
        super().__init__(4, wheel)
        with fixture(wheel).open("rb") as stream:
            metadata = binary.read_header(stream)["packet"]
        self.device.update({key: metadata[key] for key in ("caster_type", "device_id", "boot_id")},
                           record_version=2, record_bytes=116)

    def status(self):
        result = super().status()
        result.update(start_us=1000000 if self.run else None,
                      stop_us=1010100 if self.state == "stopped" else None)
        return result


def make_log(device, run="trial"):
    original = fixture(device["wheel_id"]).read_bytes()
    with fixture(device["wheel_id"]).open("rb") as stream:
        metadata = binary.read_header(stream)
    metadata["run_id"] = run
    metadata["packet"].update({key: device[key] for key in ("device_id", "boot_id")})
    encoded = json.dumps(metadata, separators=(",", ":")).encode()
    header = binary.HEADER.pack(b"CASTLOG2", 2, 116, len(encoded), zlib.crc32(encoded)) + encoded
    header += b"\0" * (4096 - len(header))
    records = original[4096:]
    return header + records, zlib.crc32(records)


def test_family_filter_happens_before_inventory_checks():
    ordinary = [Recorder(wheel).device | {"host": "127.0.0.1", "http_port": 8080}
                for wheel in client.WHEELS]
    # Invalid and duplicate other-family roles cannot poison selected inventory.
    balls = [ball_tests.Recorder().device | {"host": "127.0.0.1", "http_port": 8080}] * 2
    selected = client.validate_inventory(balls + ordinary, caster_type="ordinary", ordinary_layout="combined")
    assert len(selected) == 2 and all(item["node_id"] == 4 for item in selected)
    assert len(client.validate_inventory(balls + ordinary, require_all=False)) == 1
    with pytest.raises(ValueError, match="missing recorders.*front_right/ORDINARY"):
        client.validate_inventory(ordinary[:1], caster_type="ordinary", ordinary_layout="combined")
    assert len(client.validate_inventory(ordinary[:1], ("front_left",), caster_type="ordinary", ordinary_layout="combined")) == 1
    shared_mac = dict(ordinary[1], device_id=ordinary[0]["device_id"])
    with pytest.raises(ValueError, match="multiple wheel"):
        client.validate_inventory([ordinary[0], shared_mac], caster_type="ordinary")


def test_real_two_board_lifecycle_download_verification_and_recovery(tmp_path, monkeypatch):
    monkeypatch.setattr(ball_tests, "make_log", make_log)
    with ExitStack() as stack:
        recorders = [stack.enter_context(Recorder(wheel)) for wheel in client.WHEELS]
        session = client.Session.create(tmp_path, "trial", [item.device for item in recorders],
                                        caster_type="ordinary", sync_samples=1)
        session.start()
        assert session.manifest["state"] == "recording"
        recorders[0].fail_downloads = 1
        recorders[1].fail_ack = True
        with pytest.raises(client.RecordingError, match="incomplete"):
            session.stop()
        assert all(item.get("verified") for item in session.manifest["devices"])
        assert all(item["verified"]["record_bytes"] == 116 for item in session.manifest["devices"])
        recorders[1].fail_ack = False
        restored = client.Session.load(tmp_path, "trial", caster_type="ordinary", sync_samples=1)
        restored.fetch()
        assert restored.manifest["state"] == "complete"
        assert all(item["acknowledged"] for item in restored.manifest["devices"])
        rows = (restored.directory / "samples.csv").read_text().splitlines()
        assert len(rows) == 13
        assert "-9223372036854775808" in rows[2]
        # A completed durable run no longer contacts devices.
        counts = [len(item.requests) for item in recorders]
        restored.fetch()
        assert [len(item.requests) for item in recorders] == counts


def test_wrong_family_cannot_create_or_recover_or_mutate(tmp_path, monkeypatch):
    with Recorder() as ordinary, ball_tests.Recorder() as ball:
        with pytest.raises(ValueError, match="selected caster family"):
            client.Session.create(tmp_path, "bad", [ball.device], caster_type="ordinary")
        assert not (tmp_path / "encoder_bad").exists()
        saved = client.Session.create(tmp_path, "good", [ordinary.device], caster_type="ordinary")
        with pytest.raises(ValueError, match="different caster family"):
            client.Session.load(tmp_path, "good", caster_type="ball")
        with pytest.raises(ValueError, match="different wheel/role"):
            client.explicit_nodes([f"front_left:ORDINARY@127.0.0.1:{ball.device['http_port']}"],
                                  client.Transport(), caster_type="ordinary")
        stale_endpoint = dict(ordinary.device, host=ball.device["host"], http_port=ball.device["http_port"])
        with pytest.raises(ValueError, match="identity changed"):
            client.Transport().request(stale_endpoint, "/api/stop", {"run_id": "good"})
        monkeypatch.setattr(client, "load_config", lambda _: {})
        assert client.main(["--root", str(tmp_path), "stop", "good"]) == 2
        assert ball.bodies == [] and ordinary.bodies == []
        saved.manifest["devices"].append(ball.device)
        saved.save()
        with pytest.raises(ValueError, match="mixes caster families"):
            client.Session.load(tmp_path, "good")


def test_latest_and_timing_remain_family_scoped_and_offline(tmp_path, capsys):
    with Recorder() as ordinary, ball_tests.Recorder() as ball:
        old = client.Session.create(tmp_path, "ordinary_trial", [ordinary.device], caster_type="ordinary")
        old.manifest["created_epoch_ns"] = 100
        old.save()
        new = client.Session.create(tmp_path, "ball_trial", [ball.device])
        new.manifest["created_epoch_ns"] = 200
        new.save()
        assert client.latest_run(tmp_path) == "ball_trial"
        assert client.latest_run(tmp_path, caster_type="ordinary") == "ordinary_trial"
        assert client.main(["--root", str(tmp_path), "--caster-type", "ordinary", "latest"]) == 0
        assert json.loads(capsys.readouterr().out)["run_id"] == "ordinary_trial"
        assert client.main(["--root", str(tmp_path), "--caster-type", "ordinary", "timing"]) == 0
        report = json.loads(capsys.readouterr().out)
        assert report["devices"][0]["role"] == "ORDINARY"
        assert not ordinary.requests and not ball.requests


def test_corrupt_binary_is_never_acknowledged(tmp_path, monkeypatch):
    monkeypatch.setattr(ball_tests, "make_log", make_log)
    with Recorder() as recorder:
        session = client.Session.create(tmp_path, "trial", [recorder.device], caster_type="ordinary", sync_samples=1,
                                        transport=client.Transport(retries=1))
        session.start()
        recorder.bad_crc = True
        with pytest.raises(client.RecordingError):
            session.stop()
        assert "/api/ack" not in recorder.requests
        recorder.bad_crc = False
        session.fetch()
        assert session.manifest["state"] == "complete"


def test_default_ordinary_storage_is_separate_but_latest_bag_marker_shared(tmp_path, monkeypatch):
    bag = tmp_path / "robot_bag"
    bag.mkdir()
    (bag / "data.mcap").touch()
    client.write_json(tmp_path / ".latest_hamr_bag.json", {"version": 1, "bag_path": str(bag)})
    monkeypatch.setattr(client, "default_root", lambda: tmp_path)
    monkeypatch.setattr(client, "load_config", lambda _: {})
    devices = [Recorder(wheel).device | {"host": "127.0.0.1", "http_port": 8080}
               for wheel in client.WHEELS]
    monkeypatch.setattr(client, "discover", lambda *args: devices)
    selected = []
    monkeypatch.setattr(client.Session, "start", lambda self: selected.append(self))
    assert client.main(["--caster-type", "ordinary", "--ordinary-layout", "combined", "start", "trial", "--bag", "latest"]) == 0
    assert selected[0].directory == tmp_path / "ordinary_caster/encoder_trial"
    assert selected[0].manifest["bag_path"] == str(bag)
