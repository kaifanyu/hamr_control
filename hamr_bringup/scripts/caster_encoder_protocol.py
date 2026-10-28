#!/usr/bin/env python3
"""ESP encoder protocol and ROS-independent receiver helpers.

Protocol parser, constants, SequenceTracker and ClockMapper vendored from
caster-encoder-controller/host/logger.py at commit
58381b32a11d4495076f8889a0864c6b13aa581c (2026-10-14).
Extended here for identified v2 wheels while retaining legacy v1 decoding.
Keep wire validation consistent with that repository's docs/protocol.md.
No sibling repository or ROS installation is required to import this module.
"""

from decimal import Decimal, InvalidOperation, localcontext
import ipaddress
import json
import math
import re
import socket
import time


def ordinary_protocol():
    """Load the separate ordinary contract in source or installed layouts."""
    try:
        from ordinary_caster import protocol
    except ModuleNotFoundError as error:
        if error.name != "ordinary_caster":
            raise
        from pathlib import Path
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from ordinary_caster import protocol
    return protocol


MAX_DATAGRAM = 65507
INT64_MAX = (1 << 63) - 1
UINT64_MAX = (1 << 64) - 1
WHEELS = ("front_left", "front_right")
DEVICE_ID = re.compile(r"(?:[0-9a-f]{2}:){5}[0-9a-f]{2}\Z")
BOOT_ID = re.compile(r"[0-9a-fA-F]{16}\Z")
INTEGER_TEXT = re.compile(r"-?(0|[1-9][0-9]*)\Z")
SAMPLE_FIELDS = (
    "seq", "t_us", "raw_count", "raw_spi_word", "position14", "angle_rad",
    "absolute_angle_rad", "calibrated_alpha_rad", "velocity_rad_s", "filtered_velocity_rad_s",
    "index_event_count", "index_timestamp_us", "index_raw_count", "flags",
)
DIAGNOSTIC_FIELDS = (
    "acquisition_overruns", "queue_drops", "invalid_spi_responses",
    "index_queue_drops", "radio_failures",
)
FLAG_NAMES = (
    "ready", "sample_valid", "ppr_verified", "phase_reference_valid",
    "unwrap_continuous", "clock_sync_valid", "velocity_valid", "parity_error",
    "continuity_lost", "index_valid", "rate_bound_verified",
    "acquisition_overrun", "spi_io_error",
)
CSV_FIELDS = (
    "receive_epoch_ns", "receive_monotonic_ns", "source_ip", "source_port",
    "wheel_id", "device_id", "node_id", "boot_id", *SAMPLE_FIELDS, *FLAG_NAMES,
    "sequence_status", "mapped_epoch_ns", "mapped_clock_sync_valid",
    *DIAGNOSTIC_FIELDS,
)


def integer(value, name, minimum=0, maximum=UINT64_MAX):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")
    return value


def finite_float(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("non-finite JSON number")
    return number


def reject_constant(value):
    raise ValueError(f"invalid JSON constant: {value}")


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def parse_json(data):
    result = json.loads(data, parse_constant=reject_constant,
                        parse_float=finite_float, object_pairs_hook=unique_object)
    # Reserve ample recursion headroom for the recording envelope and reject
    # gratuitous nesting in unknown metadata fields before writing to disk.
    pending = [(result, 0)]
    while pending:
        value, depth = pending.pop()
        if depth > 16:
            raise ValueError("JSON nesting exceeds 16 levels")
        if isinstance(value, dict):
            pending.extend((item, depth + 1) for item in value.values())
        elif isinstance(value, list):
            pending.extend((item, depth + 1) for item in value)
    return result


def identity(packet):
    if packet.get("caster_type") == "ordinary":
        ordinary_protocol().identity(packet)
        return
    if packet.get("caster_type", "ball") != "ball":
        raise ValueError("unknown caster family")
    integer(packet.get("node_id"), "node_id", 1, 3)
    if not isinstance(packet.get("boot_id"), str) or not BOOT_ID.fullmatch(packet["boot_id"]):
        raise ValueError("boot_id must be 16 hexadecimal characters")


def wheel_device_identity(value):
    """Validate the pair used by v2 packets and identity-scoped clock maps."""
    if value.get("wheel_id") not in WHEELS:
        raise ValueError("wheel_id must be front_left or front_right")
    if not isinstance(value.get("device_id"), str) or not DEVICE_ID.fullmatch(value["device_id"]):
        raise ValueError("device_id must be a lowercase colon-separated MAC address")


def packet_identity(packet):
    """Return a collision-free stream key; legacy captures have no wheel identity."""
    if packet.get("node_id") in (4, 5, 6) or packet.get("caster_type") == "ordinary":
        return ordinary_protocol().identity(packet)
    if packet.get("v") == 2 or "wheel_id" in packet or "device_id" in packet:
        wheel_device_identity(packet)
    return (packet.get("wheel_id", "legacy"), packet.get("device_id", ""),
            packet["node_id"], packet["boot_id"].lower())


def count_text(value, name):
    if value is None:
        return
    if not isinstance(value, str) or len(value) > 20 or not INTEGER_TEXT.fullmatch(value):
        raise ValueError(f"{name} must be a signed int64 decimal string or null")
    integer(int(value), name, -(1 << 63), INT64_MAX)


def decode_packet(data):
    """Validate one complete datagram; malformed input raises ValueError."""
    try:
        if not 0 < len(data) <= MAX_DATAGRAM:
            raise ValueError("empty or oversized datagram")
        packet = parse_json(data.decode("utf-8"))
        if not isinstance(packet, dict):
            raise ValueError("packet must be a JSON object")
        if type(packet.get("v")) is int and packet["v"] in (3, 4):
            return ordinary_protocol().decode_packet(data)
        if type(packet.get("v")) is not int or packet["v"] not in (1, 2):
            raise ValueError("unsupported protocol version")
        if packet.get("caster_type", "ball") != "ball" or packet.get("node_id") in (4, 5, 6):
            raise ValueError("v1/v2 telemetry is reserved for ball casters")
        identity(packet)
        if packet["v"] == 2:
            wheel_device_identity(packet)
        elif "wheel_id" in packet or "device_id" in packet:
            raise ValueError("legacy v1 packets cannot declare v2 identity fields")
        if packet.get("type") == "metadata":
            expected_role = {1: "H1", 2: "H2", 3: "CENTER"}[packet["node_id"]]
            if packet.get("role") != expected_role:
                raise ValueError("metadata role disagrees with node_id")
            for name in ("firmware_version", "idf_version"):
                if not isinstance(packet.get(name), str) or not 1 <= len(packet[name]) <= 128:
                    raise ValueError(f"metadata requires {name}")
        elif packet.get("type") == "samples":
            samples = packet.get("samples")
            if not isinstance(samples, list) or not 1 <= len(samples) <= 32:
                raise ValueError("samples must contain 1 to 32 records")
            diagnostics = packet.get("diagnostics")
            if not isinstance(diagnostics, dict):
                raise ValueError("diagnostics must be an object")
            for name in DIAGNOSTIC_FIELDS:
                integer(diagnostics.get(name), name)
            for sample in samples:
                if not isinstance(sample, dict) or not all(k in sample for k in SAMPLE_FIELDS):
                    raise ValueError("sample is missing required fields")
                integer(sample["seq"], "seq")
                integer(sample["t_us"], "t_us", 0, INT64_MAX)
                integer(sample["flags"], "flags", 0, (1 << 13) - 1)
                integer(sample["index_event_count"], "index_event_count")
                integer(sample["index_timestamp_us"], "index_timestamp_us", 0, INT64_MAX)
                for name in ("raw_count", "index_raw_count"):
                    count_text(sample[name], name)
                for name, upper in (("raw_spi_word", 65535), ("position14", 16383)):
                    if sample[name] is not None:
                        integer(sample[name], name, 0, upper)
                for name in ("angle_rad", "absolute_angle_rad", "calibrated_alpha_rad", "velocity_rad_s", "filtered_velocity_rad_s"):
                    value = sample[name]
                    if value is not None:
                        if type(value) not in (int, float):
                            raise ValueError(f"{name} must be a finite number or null")
                        finite_float(value)
        else:
            raise ValueError("unknown packet type")
        return packet
    except (UnicodeError, RecursionError, OverflowError, TypeError) as error:
        raise ValueError(f"invalid packet: {error}") from error


class SequenceTracker:
    """Track gaps as intervals, including late recovery without storing every sample."""

    def __init__(self):
        self.highest_seq = -1
        self.unique_samples = 0
        self.duplicate_samples = 0
        self.reordered_samples = 0
        self.recovered_samples = 0
        self.missing_samples = 0
        self.gaps = []

    def observe(self, sequence):
        if sequence > self.highest_seq:
            gap = sequence - self.highest_seq - 1
            if gap:
                self.gaps.append((self.highest_seq + 1, sequence - 1))
                self.missing_samples += gap
            self.highest_seq = sequence
            self.unique_samples += 1
            return "in_order"
        for index, (start, end) in enumerate(self.gaps):
            if start <= sequence <= end:
                replacement = []
                if start < sequence:
                    replacement.append((start, sequence - 1))
                if sequence < end:
                    replacement.append((sequence + 1, end))
                self.gaps[index:index + 1] = replacement
                self.unique_samples += 1
                self.reordered_samples += 1
                self.recovered_samples += 1
                self.missing_samples -= 1
                return "recovered"
            if start > sequence:
                break
        self.duplicate_samples += 1
        return "duplicate"

    def summary(self):
        return {name: getattr(self, name) for name in (
            "highest_seq", "unique_samples", "duplicate_samples", "reordered_samples",
            "recovered_samples", "missing_samples",
        )}


class ClockMapper:
    """Apply externally calibrated affine maps; never estimate from UDP arrival time."""

    def __init__(self, document=None):
        self.maps = {}
        if document is None:
            return
        if not isinstance(document, dict) or type(document.get("version")) is not int or document["version"] != 1:
            raise ValueError("clock map requires version: 1")
        if not isinstance(document.get("mappings"), list):
            raise ValueError("clock map requires mappings array")
        for entry in document["mappings"]:
            if not isinstance(entry, dict):
                raise ValueError("clock mapping must be an object")
            identity(entry)
            if type(entry.get("calibrated")) is not bool:
                raise ValueError("clock mapping requires explicit calibrated boolean")
            try:
                slope = Decimal(str(entry["slope_ns_per_us"]))
            except (KeyError, InvalidOperation) as error:
                raise ValueError("invalid clock slope") from error
            if not slope.is_finite() or not Decimal("1") <= slope <= Decimal("1000000"):
                raise ValueError("clock slope must be finite in [1, 1000000] ns/us")
            offset = integer(entry.get("offset_ns"), "offset_ns", -(1 << 127), (1 << 127) - 1)
            start = integer(entry.get("valid_from_us", 0), "valid_from_us", 0, INT64_MAX)
            end = integer(entry.get("valid_to_us", INT64_MAX), "valid_to_us", start, INT64_MAX)
            if "wheel_id" in entry or "device_id" in entry:
                wheel_device_identity(entry)
            key = packet_identity(entry)
            if key in self.maps:
                raise ValueError("duplicate wheel/device/node/boot clock mapping")
            self.maps[key] = (entry["calibrated"], slope, offset, start, end)

    def map(self, node_id, boot_id, timestamp_us, wheel_id=None, device_id=None):
        if wheel_id in (None, "legacy") and device_id in (None, ""):
            wheel_id, device_id = "legacy", ""
        else:
            wheel_device_identity({"wheel_id": wheel_id, "device_id": device_id})
        mapping = self.maps.get((wheel_id, device_id, node_id, boot_id.lower()))
        if mapping is None:
            return None
        calibrated, slope, offset, start, end = mapping
        if not calibrated or not start <= timestamp_us <= end:
            return None
        with localcontext() as context:
            context.prec = 60
            return int((slope * timestamp_us + offset).to_integral_value())


ROLES = {1: "H1", 2: "H2", 3: "CENTER"}
JOINT_NAMES = {
    1: "caster_h1_rotation", 2: "caster_h2_rotation", 3: "caster_center_alpha",
}
READY = 1 << 0
SAMPLE_VALID = 1 << 1
UNWRAP_CONTINUOUS = 1 << 4
VELOCITY_VALID = 1 << 6
PARITY_ERROR = 1 << 7
CONTINUITY_LOST = 1 << 8
SPI_IO_ERROR = 1 << 12


def display_reading(node_id, sample, wheel_id="legacy", device_id=""):
    """Return a valid display value; CENTER needs no multi-turn continuity."""
    flags = sample["flags"]
    if flags & (READY | SAMPLE_VALID) != (READY | SAMPLE_VALID):
        return None
    if flags & (PARITY_ERROR | SPI_IO_ERROR):
        return None
    if node_id != 3 and (not flags & UNWRAP_CONTINUOUS or flags & CONTINUITY_LOST):
        return None
    position = sample["calibrated_alpha_rad"] if node_id == 3 else sample["angle_rad"]
    if position is None:
        return None
    velocity = sample["velocity_rad_s"] if flags & VELOCITY_VALID else None
    return {
        "node_id": node_id, "role": ROLES[node_id],
        "wheel_id": wheel_id, "device_id": device_id,
        "name": ("" if wheel_id == "legacy" else wheel_id + "_") + JOINT_NAMES[node_id],
        "position": position, "velocity": velocity, "seq": sample["seq"],
    }


class TelemetryState:
    """Keep raw and live state isolated by wheel, device, role and boot.

    A wheel-scoped live receiver rejects mismatches before touching stream state.
    Without a wheel scope this class can inspect legacy and multi-wheel captures.
    """

    def __init__(self, expected_node_ids=(1, 2, 3), stale_timeout_s=1.0,
                 wheel_id=None, expected_device_ids=("", "", ""),
                 expected_source_ips=("", "", ""), allow_legacy=False):
        if not math.isfinite(stale_timeout_s) or stale_timeout_s <= 0:
            raise ValueError("stale_timeout_s must be finite and positive")
        self.expected_node_ids = tuple(sorted(set(expected_node_ids)))
        if not self.expected_node_ids or any(type(n) is not int or n not in ROLES
                                             for n in self.expected_node_ids):
            raise ValueError("expected_node_ids must contain IDs 1, 2, or 3")
        if wheel_id is not None and wheel_id not in WHEELS:
            raise ValueError("wheel_id must be front_left or front_right")
        if allow_legacy and wheel_id is not None:
            raise ValueError("legacy mode cannot assign a wheel identity")
        self.wheel_id = wheel_id
        self.allow_legacy = allow_legacy
        self.expected_device_ids = self._inventory(expected_device_ids, "expected_device_ids")
        self.expected_source_ips = self._inventory(expected_source_ips, "expected_source_ips")
        for value in self.expected_device_ids:
            if value and not DEVICE_ID.fullmatch(value):
                raise ValueError("expected_device_ids must contain lowercase MACs or empty strings")
        configured = [value for value in self.expected_device_ids if value]
        if len(configured) != len(set(configured)):
            raise ValueError("each expected device must have a unique role")
        for value in self.expected_source_ips:
            if value:
                try:
                    ipaddress.IPv4Address(value)
                except ipaddress.AddressValueError as error:
                    raise ValueError("expected_source_ips must contain IPv4 addresses or empty strings") from error
        if wheel_id is None and (any(self.expected_device_ids) or any(self.expected_source_ips)):
            raise ValueError("device/source inventory requires a wheel_id")
        self.pinned_devices = {}
        self.stale_timeout_s = stale_timeout_s
        self.streams = {}
        self.current_boot = {}
        self.metadata = {}
        self.valid_packets = 0
        self.invalid_packets = 0
        self.rejected_packets = 0
        self.last_rejection = None

    @staticmethod
    def _inventory(values, name):
        if not isinstance(values, (list, tuple)) or len(values) != 3 or any(
                not isinstance(value, str) for value in values):
            raise ValueError(f"{name} must contain three strings in H1, H2, CENTER order")
        return tuple(values)

    def _validate_route(self, packet, source):
        if packet["v"] in (3, 4):
            raise ValueError("ordinary telemetry requires the ordinary caster bridge")
        if self.wheel_id is None:
            if self.allow_legacy and packet["v"] != 1:
                raise ValueError("legacy bridge accepts only protocol v1")
            return
        if packet["v"] != 2:
            raise ValueError("wheel-scoped bridge requires protocol v2")
        if packet["wheel_id"] != self.wheel_id:
            raise ValueError(f"wrong wheel_id: expected {self.wheel_id}, received {packet['wheel_id']}")
        node, device = packet["node_id"], packet["device_id"]
        if node not in self.expected_node_ids:
            raise ValueError(f"unexpected node_id: {node}")
        expected = self.expected_device_ids[node - 1]
        if expected and device != expected:
            raise ValueError(f"device_id mismatch for {ROLES[node]}: expected {expected}, received {device}")
        expected_ip = self.expected_source_ips[node - 1]
        if expected_ip and source[0] != expected_ip:
            raise ValueError(f"source IP mismatch for {ROLES[node]}: expected {expected_ip}, received {source[0]}")
        pinned = self.pinned_devices.get(node)
        if pinned is not None and device != pinned:
            raise ValueError(f"duplicate device for {ROLES[node]}: pinned {pinned}, received {device}")
        for other_node, other_device in self.pinned_devices.items():
            if other_node != node and other_device == device:
                raise ValueError(f"device {device} already belongs to {ROLES[other_node]}")
        for other_node, other_device in enumerate(self.expected_device_ids, start=1):
            if other_node != node and other_device == device:
                raise ValueError(f"device {device} is configured for {ROLES[other_node]}")

    def receive(self, data, source, receive_epoch_ns, receive_monotonic_ns,
                receive_ros_ns=None):
        """Return (raw envelope, readings, metadata_changed), or reject atomically."""
        try:
            packet = decode_packet(data)
        except ValueError:
            self.invalid_packets += 1
            raise
        try:
            self._validate_route(packet, source)
        except ValueError as error:
            self.invalid_packets += 1
            self.rejected_packets += 1
            self.last_rejection = str(error)
            raise
        self.valid_packets += 1
        envelope = {
            "receive_epoch_ns": receive_epoch_ns,
            "receive_monotonic_ns": receive_monotonic_ns,
            "source_ip": source[0], "source_port": source[1], "packet": packet,
        }
        if receive_ros_ns is not None:
            envelope["receive_ros_ns"] = receive_ros_ns
        key = packet_identity(packet)
        wheel, device, node_id, boot_id = key
        device_key = key[:3]
        if self.wheel_id is not None:
            self.pinned_devices.setdefault(node_id, device)
        metadata_changed = False
        if key not in self.streams:
            self.streams[key] = {
                "tracker": SequenceTracker(), "minimum_seq": None,
                "last_packet_monotonic_ns": None, "last_sample_monotonic_ns": None,
                "highest_display_seq": -1, "diagnostics": {}, "last_flags": None,
                "source_ip": source[0],
            }
            # Delayed packets from an already observed old boot cannot reactivate it.
            self.current_boot[device_key] = boot_id
            if device_key in self.metadata:
                del self.metadata[device_key]
                metadata_changed = True
        stream = self.streams[key]
        stream["source_ip"] = source[0]
        stream["last_packet_monotonic_ns"] = receive_monotonic_ns
        is_current = self.current_boot[device_key] == boot_id
        readings = []
        if packet["type"] == "metadata":
            if is_current:
                self.metadata[device_key] = packet
                metadata_changed = True
        else:
            stream["diagnostics"] = {
                name: max(stream["diagnostics"].get(name, 0), packet["diagnostics"][name])
                for name in DIAGNOSTIC_FIELDS
            }
            for sample in packet["samples"]:
                if sample["seq"] > stream["tracker"].highest_seq:
                    stream["last_sample_monotonic_ns"] = receive_monotonic_ns
                stream["tracker"].observe(sample["seq"])
                minimum = stream["minimum_seq"]
                stream["minimum_seq"] = sample["seq"] if minimum is None else min(minimum, sample["seq"])
                if not is_current or sample["seq"] <= stream["highest_display_seq"]:
                    continue
                stream["highest_display_seq"] = sample["seq"]
                stream["last_flags"] = sample["flags"]
                reading = display_reading(node_id, sample, wheel, device)
                if reading is not None:
                    readings.append(reading)
        return envelope, readings, metadata_changed

    def metadata_snapshot(self):
        nodes = {}
        for (wheel, device, node), packet in sorted(self.metadata.items()):
            # Preserve the historical single-wheel snapshot shape. An unscoped
            # multi-wheel reader gets explicit composite keys instead of collisions.
            key = str(node) if self.wheel_id is not None or wheel == "legacy" else f"{wheel}/{device}/{node}"
            nodes[key] = packet
        return {"version": 2, "wheel_id": self._snapshot_wheel(), "nodes": nodes}

    def _snapshot_wheel(self):
        if self.wheel_id is not None:
            return self.wheel_id
        wheels = {key[0] for key in self.current_boot}
        return next(iter(wheels)) if len(wheels) == 1 else ("mixed" if wheels else "legacy")

    def diagnostics_snapshot(self, now_monotonic_ns):
        streams = []
        for (wheel, device, node, boot), stream in sorted(self.streams.items()):
            summary = stream["tracker"].summary()
            minimum = stream["minimum_seq"]
            summary["first_observed_sequence"] = minimum
            summary["missing_samples_in_recording"] = (
                0 if minimum is None else summary["highest_seq"] - minimum + 1 - summary["unique_samples"]
            )
            last_sample = stream["last_sample_monotonic_ns"]
            age = None if last_sample is None else max(0.0, (now_monotonic_ns - last_sample) / 1e9)
            streams.append({
                "wheel_id": wheel, "device_id": device,
                "node_id": node, "role": ROLES[node], "boot_id": boot,
                "source_ip": stream["source_ip"],
                "current_boot": self.current_boot[(wheel, device, node)] == boot,
                "sample_age_s": age, "stale": age is None or age > self.stale_timeout_s,
                "last_flags": stream["last_flags"],
                "firmware_diagnostics": dict(stream["diagnostics"]), **summary,
            })
        nodes = []
        identities = set(self.current_boot)
        for node in self.expected_node_ids:
            if not any(key[2] == node for key in identities):
                identities.add((self.wheel_id or "legacy", self.expected_device_ids[node - 1], node))
        for wheel, device, node in sorted(identities, key=lambda key: (key[0], key[2], key[1])):
            key = (wheel, device, node)
            boot = self.current_boot.get(key)
            stream = self.streams.get((*key, boot))
            last_sample = None if stream is None else stream["last_sample_monotonic_ns"]
            age = None if last_sample is None else max(0.0, (now_monotonic_ns - last_sample) / 1e9)
            nodes.append({
                "wheel_id": wheel, "device_id": device,
                "node_id": node, "role": ROLES[node], "boot_id": boot,
                "source_ip": None if stream is None else stream["source_ip"],
                "expected_device_id": self.expected_device_ids[node - 1],
                "expected_source_ip": self.expected_source_ips[node - 1],
                "inventory_verified": bool(self.expected_device_ids[node - 1]) and boot is not None,
                "expected": node in self.expected_node_ids,
                "never_seen": last_sample is None, "sample_age_s": age,
                "stale": age is None or age > self.stale_timeout_s,
                "metadata_received": key in self.metadata,
            })
        return {
            "version": 2, "wheel_id": self._snapshot_wheel(),
            "timestamp_basis": "host_receive_unsynchronized",
            "freshness_basis": "latest_advancing_sample_sequence",
            "valid_packets": self.valid_packets, "invalid_packets": self.invalid_packets,
            "rejected_packets": self.rejected_packets, "last_rejection": self.last_rejection,
            "inventory_verified": all(n["inventory_verified"] for n in nodes if n["expected"]),
            "stale_timeout_s": self.stale_timeout_s, "nodes": nodes, "streams": streams,
        }


class UDPReceiver:
    """An exclusive, nonblocking UDP socket with a bounded drain operation."""

    def __init__(self, bind_address="0.0.0.0", udp_port=5005):
        if type(udp_port) is not int or not 0 <= udp_port <= 65535:
            raise ValueError("udp_port must be an integer in 0..65535")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            self.socket.bind((bind_address, udp_port))
            self.socket.setblocking(False)
        except OSError as error:
            self.socket.close()
            raise RuntimeError(
                f"Cannot listen on UDP {bind_address}:{udp_port}: {error}. "
                "Stop another encoder logger/bridge using this port, or check the bind address."
            ) from error

    def drain(self, max_datagrams=200, max_duration_s=0.005):
        deadline = time.monotonic() + max_duration_s
        for _ in range(max_datagrams):
            if time.monotonic() >= deadline:
                break
            try:
                data, source = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            # Some systems report an asynchronous ICMP error on a UDP socket.
            # Let other receive errors surface instead of silently stopping data.
            yield data, source, time.time_ns(), time.monotonic_ns()

    def close(self):
        self.socket.close()

