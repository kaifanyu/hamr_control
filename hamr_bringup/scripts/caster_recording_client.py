"""Coordinate durable per-ESP recordings without streaming encoder samples.

The robot trajectory is deliberately independent. This client controls only the
encoder recorders and saves a recoverable sidecar beside the robot's ROS bags.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import contextmanager
from datetime import datetime, timezone
import http.client
import json
import math
import os
from pathlib import Path
import re
import socket
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import zlib


ROLES = {1: "H1", 2: "H2", 3: "CENTER"}
WHEELS = ("front_left", "front_right")
RUN_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}\Z")
MAC = re.compile(r"(?:[0-9a-f]{2}:){5}[0-9a-f]{2}\Z")
BOOT = re.compile(r"[0-9a-fA-F]{16}\Z")


class RecordingError(RuntimeError):
    """A failed operation whose durable manifest can be used for recovery."""


def run_id(value):
    if not isinstance(value, str) or not RUN_ID.fullmatch(value):
        raise ValueError("run_id must be 1..64 letters, digits, underscores, hyphens or dots, starting with a letter/digit")
    return value


def identity(value):
    if not isinstance(value, dict) or value.get("wheel_id") not in WHEELS:
        raise ValueError("invalid recorder wheel identity")
    if type(value.get("node_id")) is not int or value["node_id"] not in ROLES:
        raise ValueError("invalid recorder node identity")
    if not isinstance(value.get("device_id"), str) or not MAC.fullmatch(value["device_id"]):
        raise ValueError("invalid recorder MAC identity")
    if not isinstance(value.get("boot_id"), str) or not BOOT.fullmatch(value["boot_id"]):
        raise ValueError("invalid recorder boot identity")
    return value["wheel_id"], value["node_id"], value["device_id"]


def check_identity(actual, expected, same_boot=False):
    if identity(actual) != identity(expected):
        raise ValueError("recorder identity changed; refusing commands or imported data")
    if same_boot and actual["boot_id"].lower() != expected["boot_id"].lower():
        raise ValueError("recorder rebooted during this operation")


def fsync_directory(path):
    # Linux/Pi requires the containing directory to be durable after rename.
    if os.name != "nt":
        descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    fsync_directory(path.parent)


@contextmanager
def run_lock(directory):
    """Serialize a complete operation, including file verification and ACK."""
    with (Path(directory) / ".recording.lock").open("a+b") as handle:
        handle.seek(0, os.SEEK_END)
        if handle.tell() == 0:
            handle.write(b"0")
            handle.flush()
        handle.seek(0)
        try:
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            raise RecordingError("another caster_recording operation is using this run") from error
        try:
            yield
        finally:
            handle.seek(0)
            if os.name == "nt":
                msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def default_root():
    return Path(os.environ.get("HAMR_BAG_ROOT", str(Path.home() / "hamster_ws/src/hamr_control/rosbags")))


def config_path():
    source = Path(__file__).resolve().parent.parent / "config/caster_recording.yaml"
    if source.is_file():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("hamr_bringup")) / "config/caster_recording.yaml"
    except (ImportError, LookupError):
        return None


def load_config(path):
    if path is None:
        return {}
    import yaml
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("version") != 1:
        raise ValueError("recorder config must be a version: 1 mapping")
    return value


def validate_inventory(devices, wheels=WHEELS, inventory=(), require_all=True):
    selected = [dict(item) for item in devices if item.get("wheel_id") in wheels]
    roles, macs = {}, {}
    for item in selected:
        wheel, node, mac = identity(item)
        key = wheel, node
        if key in roles and roles[key]["device_id"] != mac:
            raise ValueError(f"ambiguous devices for {wheel}/{ROLES[node]}")
        if mac in macs and macs[mac] != key:
            raise ValueError(f"device {mac} advertises multiple wheel/role identities")
        if key in roles and (roles[key]["host"], roles[key]["http_port"]) != (item["host"], item["http_port"]):
            raise ValueError(f"device {mac} advertises multiple endpoints")
        if key in roles and roles[key]["boot_id"].lower() != item["boot_id"].lower():
            raise ValueError(f"device {mac} rebooted during discovery; discover again")
        roles[key], macs[mac] = item, key
    configured_keys = set()
    for expected in inventory:
        wheel, node = expected.get("wheel_id"), expected.get("node_id")
        if wheel not in WHEELS or node not in ROLES or (wheel, node) in configured_keys:
            raise ValueError("inventory has an invalid or duplicate wheel/role")
        configured_keys.add((wheel, node))
        actual = roles.get((wheel, node))
        if actual is None:
            continue
        if expected.get("device_id") and expected["device_id"] != actual["device_id"]:
            raise ValueError(f"MAC inventory mismatch for {wheel}/{ROLES[node]}")
        if expected.get("host") and expected["host"] != actual["host"]:
            raise ValueError(f"host inventory mismatch for {wheel}/{ROLES[node]}")
    missing = [(wheel, ROLES[node]) for wheel in wheels for node in ROLES if (wheel, node) not in roles]
    if missing and require_all:
        raise ValueError("missing recorders: " + ", ".join(f"{wheel}/{role}" for wheel, role in missing))
    return [roles[key] for key in sorted(roles)]


def discover(address="255.255.255.255", port=5010, timeout=2.0):
    request_id = uuid.uuid4().hex
    request = json.dumps({"v": 1, "type": "caster_recorder_discover", "request_id": request_id}).encode()
    devices = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        receiver.bind(("0.0.0.0", 0))
        # Repeat discovery to tolerate loss without touching live telemetry ports.
        deadline, next_send = time.monotonic() + timeout, 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                receiver.sendto(request, (address, port))
                next_send = now + 0.5
            receiver.settimeout(max(.001, min(.2, deadline - now)))
            try:
                data, source = receiver.recvfrom(4096)
            except socket.timeout:
                continue
            try:
                value = json.loads(data)
                if value.get("v") != 1 or value.get("type") != "caster_recorder" or value.get("request_id") != request_id:
                    continue
                identity(value)
                if type(value.get("http_port")) is not int or not 1 <= value["http_port"] <= 65535:
                    continue
                value["host"] = source[0]
                devices.append(value)
            except (ValueError, TypeError, AttributeError, UnicodeError):
                continue
    return devices


class Transport:
    def __init__(self, timeout=10.0, prepare_timeout=30.0, retries=3):
        self.timeout, self.prepare_timeout, self.retries = timeout, prepare_timeout, retries

    @staticmethod
    def url(device, path):
        host = device["host"]
        if not re.fullmatch(r"[A-Za-z0-9.-]+", host):
            raise ValueError("recorder host must be an IPv4 address or hostname")
        port = device.get("http_port", 8080)
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("invalid recorder HTTP port")
        return f"http://{host}:{port}{path}"

    def request(self, device, path, body=None):
        payload = None if body is None else json.dumps(body, allow_nan=False).encode()
        request = urllib.request.Request(self.url(device, path), data=payload,
                                         headers={"Content-Type": "application/json"})
        timeout = self.prepare_timeout if path in ("/api/prepare", "/api/stop") else self.timeout
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                data = response.read(65537)
                if len(data) > 65536:
                    raise ValueError("oversized recorder JSON response")
                value = json.loads(data)
        except urllib.error.HTTPError as error:
            detail = error.read(1024).decode("utf-8", errors="replace")
            raise RecordingError(f"{path}: HTTP {error.code}: {detail}") from error
        if not isinstance(value, dict) or value.get("v") != 1:
            raise ValueError("invalid recorder API response")
        check_identity(value, device)
        return value

    def sync(self, device):
        send_mono = time.monotonic_ns()
        send_epoch = time.time_ns()
        response = self.request(device, "/api/sync")
        recv_epoch = time.time_ns()
        recv_mono = time.monotonic_ns()
        check_identity(response, device, same_boot=True)
        if any(type(response.get(key)) is not int or response[key] < 0 for key in ("rx_us", "tx_us")) or response["tx_us"] < response["rx_us"]:
            raise ValueError("invalid recorder sync timestamps")
        return {"host_send_epoch_ns": send_epoch, "host_recv_epoch_ns": recv_epoch,
                "host_send_monotonic_ns": send_mono, "host_recv_monotonic_ns": recv_mono,
                "device": response}

    def download(self, device, run, path, expected):
        from caster_recording_format import HEADER_BYTES, verify_log
        if expected.get("run_id") != run or expected.get("state") not in ("stopped", "error") or expected.get("finalizing", False):
            raise ValueError("download requires a finalized stopped/error recording with the requested run_id")
        for field in ("data_bytes", "sample_count", "crc32"):
            if type(expected.get(field)) is not int or expected[field] < 0:
                raise ValueError(f"invalid download status {field}")
        if expected["data_bytes"] <= 0 or expected["crc32"] > 0xffffffff:
            raise ValueError("invalid download size or CRC")
        if path.exists():
            return verify_log(path, expected_status=expected, expected_run_id=run)
        partial = path.with_name(path.name + ".part")
        last_error = None
        # A stopped file is immutable. Resume means retry a whole file or reuse a
        # verified complete file; no server-side HTTP Range support is assumed.
        for attempt in range(self.retries):
            try:
                request = urllib.request.Request(self.url(device, "/api/log?" + urllib.parse.urlencode({"run_id": run})))
                count, crc = 0, 0
                with urllib.request.urlopen(request, timeout=self.timeout) as response, partial.open("wb") as stream:
                    while True:
                        chunk = response.read(65536)
                        if not chunk:
                            break
                        offset = max(0, HEADER_BYTES - count)
                        count += len(chunk)
                        if count > expected["data_bytes"]:
                            raise ValueError("download exceeds advertised data_bytes")
                        stream.write(chunk)
                        # The API CRC covers records only; the header has its
                        # own metadata CRC checked by verify_log below.
                        crc = zlib.crc32(chunk[offset:], crc)
                    stream.flush()
                    os.fsync(stream.fileno())
                if count != expected["data_bytes"] or crc != expected["crc32"]:
                    raise ValueError("download length or record-stream CRC mismatch")
                verified = verify_log(partial, expected_status=expected, expected_run_id=run)
                os.replace(partial, path)
                fsync_directory(path.parent)
                return verified
            except (OSError, ValueError, RecordingError, http.client.HTTPException) as error:
                last_error = error
                if attempt + 1 < self.retries:
                    time.sleep(min(.2 * (attempt + 1), .6))
        raise RecordingError(f"download failed after {self.retries} attempts: {last_error}")


class Session:
    def __init__(self, directory, manifest, transport=None, sync_samples=6, planned_duration=None):
        self.directory, self.manifest = Path(directory), manifest
        self.transport, self.sync_samples = transport or Transport(), sync_samples
        self.planned_duration = planned_duration

    @classmethod
    def create(cls, root, run, devices, bag=None, discard_existing=False, **options):
        run_id(run)
        root = Path(root).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        directory = root / ("encoder_" + run)
        # Exclusive directory creation prevents repeated start from erasing logs.
        directory.mkdir()
        fsync_directory(root)
        entries = []
        for device in devices:
            entry = dict(device)
            entry.update(sync_pre=[], sync_post=[], errors=[], acknowledged=False)
            entry["log_path"] = f"{device['wheel_id']}_{ROLES[device['node_id']]}_{device['device_id'].replace(':', '')}.bin"
            entries.append(entry)
        manifest = {"version": 1, "run_id": run, "bag_path": str(Path(bag).expanduser().resolve()) if bag else None,
                    "discard_existing": discard_existing,
                    "state": "created", "created_epoch_ns": time.time_ns(), "devices": entries, "errors": []}
        session = cls(directory, manifest, **options)
        session.save()
        return session

    @classmethod
    def load(cls, root, run, **options):
        directory = Path(root).expanduser().resolve() / ("encoder_" + run_id(run))
        manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
        if manifest.get("version") != 1 or manifest.get("run_id") != run:
            raise ValueError("invalid saved recording manifest")
        devices = manifest.get("devices", [])
        if not devices:
            raise ValueError("manifest has no devices")
        validate_inventory(devices, require_all=False)
        for device in devices:
            filename = device.get("log_path", "")
            if not filename or Path(filename).name != filename or "\\" in filename:
                raise ValueError("unsafe log_path in recording manifest")
        return cls(directory, manifest, **options)

    def save(self):
        self.manifest["updated_epoch_ns"] = time.time_ns()
        write_json(self.directory / "manifest.json", self.manifest)

    def refresh_endpoints(self, discovered):
        available = validate_inventory(discovered, require_all=False)
        by_role = {(item["wheel_id"], item["node_id"]): item for item in available}
        for item in self.manifest["devices"]:
            if item.get("acknowledged") and item.get("verified"):
                continue
            current = by_role.get((item["wheel_id"], item["node_id"]))
            if current is None:
                continue  # Saved endpoint remains a fallback if broadcast is blocked.
            check_identity(current, item)
            item.update(host=current["host"], http_port=current["http_port"])
        self.save()

    def phase(self, state):
        self.manifest["state"] = state
        self.save()

    def parallel(self, function, devices=None):
        errors = []
        selected = self.manifest["devices"] if devices is None else devices
        with ThreadPoolExecutor(max_workers=max(1, len(selected))) as pool:
            futures = {pool.submit(function, item): item for item in selected}
            for future in as_completed(futures):
                item = futures[future]
                try:
                    result = future.result()
                    if result is not None:
                        item.update(result)
                except Exception as error:
                    message = f"{item['wheel_id']}/{ROLES[item['node_id']]}: {error}"
                    item.setdefault("errors", []).append(message)
                    errors.append(message)
                self.save()
        return errors

    def command(self, item, action, required_state=None, same_boot=False):
        body = {"run_id": self.manifest["run_id"]}
        if action == "prepare" and self.manifest.get("discard_existing", False):
            # Bind destructive preparation to the physical board, its boot and
            # the old run observed during preflight. A changed endpoint or a
            # competing session must not turn this into an unrelated erase.
            body.update(discard_existing=True, expected_device_id=item["device_id"],
                        expected_boot_id=item["boot_id"],
                        expected_run_id=item["preflight_status"].get("run_id"))
        status = self.transport.request(item, "/api/" + action, body)
        check_identity(status, item, same_boot=same_boot)
        if status.get("run_id") != self.manifest["run_id"]:
            raise ValueError(f"{action} response has wrong run_id")
        if required_state and status.get("state") != required_state:
            raise ValueError(f"{action} returned state {status.get('state')!r}, expected {required_state}")
        return {"status": status}

    def anchors(self, key, devices=None):
        # Each round is saved before the next round; an interrupted start retains
        # every completed anchor and the exact identities needed for recovery.
        errors = []
        for _ in range(self.sync_samples):
            def sample(item):
                anchor = self.transport.sync(item)
                return {key: [*item.get(key, []), anchor]}
            errors.extend(self.parallel(sample, devices))
        return errors

    def start(self):
        if self.manifest["state"] != "created":
            raise RecordingError("this run already exists; use stop/fetch to recover, or choose a new run_id")
        self.phase("preflight")
        discard = self.manifest.get("discard_existing", False)
        def inspect(item):
            status = self.transport.request(item, "/api/status")
            check_identity(status, item, same_boot=True)
            # Retain the actual device diagnosis even when preflight fails.
            item["preflight_status"] = status
            state, previous_run = status.get("state"), status.get("run_id")
            if state not in ("idle", "prepared", "recording", "stopped", "error"):
                raise ValueError(f"unrecognized recorder state {state!r}")
            if status.get("run_id") == self.manifest["run_id"]:
                raise ValueError("device already retains this run_id; recover it instead of restarting")
            if discard:
                if status.get("supports_discard_existing") is not True:
                    raise ValueError("firmware does not support --discard-existing; flash the updated recorder firmware first")
                if state == "error" and status.get("stop_reason") in (
                        "missing_log_partition", "journal_unavailable", "journal_read_error"):
                    raise ValueError(f"recording storage is unavailable ({status['stop_reason']}); repair it before discarding")
                if previous_run is not None:
                    run_id(previous_run)
                if (state in ("prepared", "recording") or status.get("finalizing", False)) and previous_run is None:
                    raise ValueError("active recorder has no valid run_id; cannot stop it before discarding")
            elif (state != "idle" and not (state in ("stopped", "error") and status.get("acknowledged"))) or status.get("finalizing", False):
                raise ValueError(
                    f"device is not ready (state={state}, stop_reason={status.get('stop_reason')!r}, "
                    f"run_id={previous_run!r}); stop/fetch an existing run, or use --discard-existing "
                    "with updated firmware to erase its onboard data")
            return {"status": status}
        errors = self.parallel(inspect)
        if not errors:
            for item in self.manifest["devices"]:
                status = item["status"]
                hz, capacity = status.get("sample_hz"), status.get("capacity_samples")
                if type(hz) not in (int, float) or not math.isfinite(hz) or hz <= 0 or type(capacity) is not int or capacity <= 0:
                    errors.append(f"{item['wheel_id']}/{ROLES[item['node_id']]}: missing valid sample_hz/capacity_samples")
                    continue
                duration = capacity / hz
                item["capacity_duration_s"] = duration
                print(f"{item['wheel_id']}/{ROLES[item['node_id']]}: {capacity} samples at {hz:g} Hz; maximum {duration:.1f} s", flush=True)
                if self.planned_duration is not None and self.planned_duration >= duration:
                    errors.append(f"planned {self.planned_duration:g} s recording does not fit {duration:.1f} s capacity")
        if errors:
            self.manifest["errors"].extend(errors)
            self.phase("start_failed")
            raise RecordingError("recording preflight failed before prepare/start: " + "; ".join(errors))
        if discard:
            # All devices have passed identity, capability and capacity checks
            # before even stopping an old run. Never erase while its writer is
            # active, and do not download data the operator chose to discard.
            active = [item for item in self.manifest["devices"]
                      if item["status"]["state"] in ("prepared", "recording")
                      or item["status"].get("finalizing", False)]
            if active:
                self.phase("stopping_previous")
                def stop_previous(item):
                    previous_run = item["preflight_status"]["run_id"]
                    status = self.transport.request(item, "/api/stop", {
                        "run_id": previous_run, "expected_device_id": item["device_id"],
                        "expected_boot_id": item["boot_id"]})
                    check_identity(status, item, same_boot=True)
                    if (status.get("run_id") != previous_run or
                            status.get("state") not in ("stopped", "error") or
                            status.get("finalizing", False)):
                        raise ValueError("previous recording did not finish stopping; refusing to erase")
                    return {"status": status}
                errors = self.parallel(stop_previous, active)
                if errors:
                    self.manifest["errors"].extend(errors)
                    self.phase("start_failed")
                    raise RecordingError("could not stop previous recordings; no prepare/erase was requested: " + "; ".join(errors))
        self.phase("preparing")
        errors = self.parallel(lambda item: self.command(item, "prepare", "prepared", same_boot=True))
        if not errors:
            self.phase("synchronizing_before")
            errors = self.anchors("sync_pre")
        if not errors:
            self.phase("starting")
            errors = self.parallel(lambda item: self.command(item, "start", "recording", same_boot=True))
        if errors:
            self.manifest["errors"].extend(errors)
            self.phase("start_failed_stopping")
            cleanup = self.parallel(lambda item: self.command(item, "stop", "stopped"))
            self.manifest["errors"].extend(cleanup)
            self.phase("start_failed")
            discarded = " Existing onboard logs may already have been erased." if discard else ""
            raise RecordingError("recording start failed; stop was attempted on every selected device."
                                 + discarded + " Run stop again to recover/download. " + "; ".join(errors))
        self.phase("recording")

    def stop(self):
        if all(item.get("acknowledged") for item in self.manifest["devices"]):
            return self.fetch()
        self.phase("stopping")
        active = [item for item in self.manifest["devices"] if not item.get("acknowledged")]
        errors = self.parallel(lambda item: self.command(item, "stop", "stopped"), active)
        self.manifest["errors"].extend(errors)
        self.phase("synchronizing_after")
        # A reboot invalidates new anchors for the saved log but must not prevent
        # recovering and acknowledging a correctly verified binary recording.
        sync_errors = self.anchors("sync_post", active)
        self.manifest["errors"].extend(sync_errors)
        return self.fetch(prior_errors=errors)

    def fetch(self, prior_errors=()):
        self.phase("fetching")
        errors = list(prior_errors)
        for item in self.manifest["devices"]:
            item["fetch_validated"] = False
        self.save()
        def fetch_one(item):
            if item.get("acknowledged") and item.get("verified"):
                # The board may already be preparing a later run. Revalidate
                # this run's durable file without touching that new recording.
                from caster_recording_format import verify_log
                verified = verify_log(self.directory / item["log_path"], expected_status=item["status"], expected_run_id=self.manifest["run_id"])
                return {"verified": verified, "fetch_validated": True}
            status = self.transport.request(item, "/api/status")
            if status.get("run_id") != self.manifest["run_id"] or status.get("state") not in ("stopped", "error") or status.get("finalizing", False):
                raise ValueError("requested run is not retained and finalized; run stop first")
            verified = self.transport.download(item, self.manifest["run_id"], self.directory / item["log_path"], status)
            return {"status": status, "verified": verified, "fetch_validated": True, "downloaded_epoch_ns": time.time_ns()}
        errors.extend(self.parallel(fetch_one))
        # All verified binary files and their statuses are durable before any ACK.
        self.save()
        def acknowledge(item):
            result = self.transport.request(item, "/api/ack", {"run_id": self.manifest["run_id"], "crc32": item["status"]["crc32"]})
            if result.get("run_id") != self.manifest["run_id"] or not result.get("acknowledged"):
                raise ValueError("recorder did not acknowledge durable download")
            return {"acknowledged": True, "acknowledged_epoch_ns": time.time_ns()}
        # Verification from a previous attempt is usable only if this fetch also
        # returned and verified the current retained status.
        candidates = [item for item in self.manifest["devices"] if item.get("fetch_validated") and not item.get("acknowledged")]
        errors.extend(self.parallel(acknowledge, candidates))
        for item in self.manifest["devices"]:
            reason = item.get("status", {}).get("stop_reason")
            if reason and reason not in ("command", "requested", "user", "stopped", "api"):
                # Retain/download the trace, but do not advertise a capacity,
                # acquisition or storage failure as a successful commanded run.
                errors.append(f"{item['wheel_id']}/{ROLES[item['node_id']]} stopped early: {reason}")
        try:
            from caster_recording_format import export_run
            self.manifest["exports"] = export_run(self.directory, self.manifest)
            for issue in self.manifest["exports"].get("clock_errors", []):
                print(f"Timing unavailable for {issue.get('device_id', 'recorder')}: {issue.get('error', issue)}; local timestamps preserved", file=sys.stderr)
        except Exception as error:
            errors.append(f"offline export failed (verified binaries preserved): {error}")
        self.manifest["errors"].extend(errors)
        complete = all(item.get("verified") and item.get("acknowledged") for item in self.manifest["devices"])
        self.phase("complete" if complete and not errors else "incomplete")
        if errors or not complete:
            raise RecordingError("recording recovery incomplete; rerun stop/fetch. " + "; ".join(errors))
        return self.manifest


def explicit_nodes(specifications, transport):
    devices = []
    for specification in specifications:
        try:
            identity_text, endpoint = specification.split("@", 1)
            wheel, role = identity_text.split(":", 1)
            host, separator, port = endpoint.partition(":")
            node = next(key for key, value in ROLES.items() if value == role.upper())
            if wheel not in WHEELS:
                raise ValueError()
            endpoint_data = {"host": host, "http_port": int(port) if separator else 8080}
        except (ValueError, StopIteration):
            raise ValueError("--node requires front_left:H1@192.168.1.20[:8080] (or H2/CENTER/front_right)") from None
        # No expected MAC exists until initial discovery/status; explicitly check
        # the physical role requested instead of accepting a response silently.
        request = urllib.request.Request(transport.url(endpoint_data, "/api/status"))
        with urllib.request.urlopen(request, timeout=transport.timeout) as response:
            status = json.loads(response.read(65537))
        identity(status)
        if status.get("v") != 1 or (status["wheel_id"], status["node_id"]) != (wheel, node):
            raise ValueError("explicit endpoint returned a different wheel/role")
        devices.append(dict(status, **endpoint_data))
    return devices


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=default_root(), help="Pi rosbag root; creates encoder_RUN_ID sidecar")
    parser.add_argument("--config", type=Path, default=config_path())
    parser.add_argument("--wheel", choices=("both", *WHEELS), default="both")
    parser.add_argument("--node", action="append", default=[], metavar="WHEEL:ROLE@HOST[:PORT]", help="explicit endpoint; repeat for every selected role")
    parser.add_argument("--discovery-address", default=None)
    parser.add_argument("--discovery-timeout", type=float, default=2.0)
    parser.add_argument("--http-timeout", type=float, default=10.0)
    parser.add_argument("--prepare-timeout", type=float, default=35.0, help="prepare/stop HTTP timeout, including flash finalization")
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--sync-samples", type=int, default=6)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("discover", aliases=["discovery"])
    commands.add_parser("status")
    start = commands.add_parser("start")
    start.add_argument("run_id", nargs="?", default=None)
    start.add_argument("--bag", type=Path, help="associate an existing robot bag without modifying it")
    start.add_argument("--duration", type=float, default=None, help="planned total recording seconds, including startup and stop delay; preflight capacity check only")
    start.add_argument("--discard-existing", action="store_true",
                       help="erase selected boards' existing logs, including unread/corrupt data; stop active logs first; requires updated firmware")
    for name in ("stop", "fetch"):
        command = commands.add_parser(name)
        command.add_argument("run_id")
    args = parser.parse_args(argv)
    try:
        if not 1 <= args.sync_samples <= 100 or not 1 <= args.retries <= 20:
            raise ValueError("sync-samples must be 1..100 and retries 1..20")
        if not all(0 < item <= 300 for item in (args.discovery_timeout, args.http_timeout, args.prepare_timeout)):
            raise ValueError("timeouts must be positive and at most 300 seconds")
        configuration = load_config(args.config)
        transport = Transport(args.http_timeout, args.prepare_timeout, args.retries)
        options = {"transport": transport, "sync_samples": args.sync_samples}
        if args.command in ("stop", "fetch"):
            directory = Path(args.root).expanduser().resolve() / ("encoder_" + run_id(args.run_id))
            with run_lock(directory):
                session = Session.load(args.root, args.run_id, **options)
                # Reuse local complete data without contacting recorders that
                # may have moved on to a different recording session.
                if not all(item.get("acknowledged") for item in session.manifest["devices"]):
                    if args.node:
                        available = explicit_nodes(args.node, transport)
                    else:
                        try:
                            available = discover(args.discovery_address or configuration.get("discovery_address", "255.255.255.255"),
                                                 configuration.get("discovery_port", 5010), args.discovery_timeout)
                        except OSError as error:
                            print(f"Discovery unavailable; using saved endpoints: {error}", file=sys.stderr)
                            available = []
                    session.refresh_endpoints(available)
                getattr(session, args.command)()
            print(f"Saved encoder recording: {session.directory}")
            return 0
        raw = explicit_nodes(args.node, transport) if args.node else discover(
            args.discovery_address or configuration.get("discovery_address", "255.255.255.255"),
            configuration.get("discovery_port", 5010), args.discovery_timeout)
        wheels = WHEELS if args.wheel == "both" else (args.wheel,)
        devices = validate_inventory(raw, wheels, configuration.get("devices", []), require_all=args.command == "start")
        if args.command in ("discover", "discovery", "status"):
            if args.command == "status":
                devices = [dict(transport.request(item, "/api/status"), host=item["host"], http_port=item["http_port"]) for item in devices]
            print(json.dumps(devices, indent=2))
            return 0 if devices else 1
        run = args.run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid.uuid4().hex[:8]
        if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
            raise ValueError("--duration must be finite and positive")
        if args.bag and not args.bag.exists():
            raise ValueError("--bag must identify an existing robot bag path")
        session = Session.create(args.root, run, devices, bag=args.bag, discard_existing=args.discard_existing,
                                 planned_duration=args.duration, **options)
        print(f"Recording run {run}; recovery manifest: {session.directory / 'manifest.json'}", flush=True)
        if args.discard_existing:
            print("Discard mode: existing onboard logs on the selected boards will be erased; saved host files are retained.", flush=True)
        with run_lock(session.directory):
            session.start()
        print(f"All {len(devices)} recorders are recording locally. Stop/download: caster_recording --root {args.root} stop {run}")
        return 0
    except (RecordingError, OSError, ValueError, KeyError, TypeError, http.client.HTTPException) as error:
        print(f"caster_recording: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
