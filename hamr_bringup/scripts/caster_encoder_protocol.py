#!/usr/bin/env python3
"""ESP encoder protocol and ROS-independent receiver helpers.

Protocol parser, constants, SequenceTracker and ClockMapper vendored from
caster-encoder-controller/host/logger.py at commit
58381b32a11d4495076f8889a0864c6b13aa581c (2026-10-14).
Keep wire validation consistent with that repository's docs/protocol.md.
No sibling repository or ROS installation is required to import this module.
"""

from decimal import Decimal, InvalidOperation, localcontext
import json
import math
import re
import socket
import time


MAX_DATAGRAM = 65507
INT64_MAX = (1 << 63) - 1
UINT64_MAX = (1 << 64) - 1
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
    "node_id", "boot_id", *SAMPLE_FIELDS, *FLAG_NAMES,
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
    integer(packet.get("node_id"), "node_id", 1, 3)
    if not isinstance(packet.get("boot_id"), str) or not BOOT_ID.fullmatch(packet["boot_id"]):
        raise ValueError("boot_id must be 16 hexadecimal characters")


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
        if type(packet.get("v")) is not int or packet["v"] != 1:
            raise ValueError("unsupported protocol version")
        identity(packet)
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
            key = (entry["node_id"], entry["boot_id"].lower())
            if key in self.maps:
                raise ValueError("duplicate node/boot clock mapping")
            self.maps[key] = (entry["calibrated"], slope, offset, start, end)

    def map(self, node_id, boot_id, timestamp_us):
        mapping = self.maps.get((node_id, boot_id.lower()))
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


def display_reading(node_id, sample):
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
        "node_id": node_id, "role": ROLES[node_id], "name": JOINT_NAMES[node_id],
        "position": position, "velocity": velocity, "seq": sample["seq"],
    }


class TelemetryState:
    """Keep all raw samples, with separate conservative live-display state.

    Packet reception time is deliberately never called acquisition time. Boot
    IDs already superseded cannot reactivate an old display or old metadata.
    """

    def __init__(self, expected_node_ids=(1, 2, 3), stale_timeout_s=1.0):
        if not math.isfinite(stale_timeout_s) or stale_timeout_s <= 0:
            raise ValueError("stale_timeout_s must be finite and positive")
        self.expected_node_ids = tuple(sorted(set(expected_node_ids)))
        if not self.expected_node_ids or any(type(n) is not int or n not in ROLES
                                             for n in self.expected_node_ids):
            raise ValueError("expected_node_ids must contain IDs 1, 2, or 3")
        self.stale_timeout_s = stale_timeout_s
        self.streams = {}
        self.current_boot = {}
        self.metadata = {}
        self.valid_packets = 0
        self.invalid_packets = 0

    def receive(self, data, source, receive_epoch_ns, receive_monotonic_ns,
                receive_ros_ns=None):
        """Return (raw envelope, display readings, metadata_changed).

        Invalid input increments diagnostics and raises ValueError. The caller
        must not publish it as valid encoder telemetry.
        """
        try:
            packet = decode_packet(data)
        except ValueError:
            self.invalid_packets += 1
            raise
        self.valid_packets += 1
        envelope = {
            "receive_epoch_ns": receive_epoch_ns,
            "receive_monotonic_ns": receive_monotonic_ns,
            "source_ip": source[0], "source_port": source[1], "packet": packet,
        }
        if receive_ros_ns is not None:
            envelope["receive_ros_ns"] = receive_ros_ns
        node_id, boot_id = packet["node_id"], packet["boot_id"].lower()
        key = (node_id, boot_id)
        metadata_changed = False
        if key not in self.streams:
            self.streams[key] = {
                "tracker": SequenceTracker(), "minimum_seq": None,
                "last_packet_monotonic_ns": None, "last_sample_monotonic_ns": None,
                "highest_display_seq": -1, "diagnostics": {}, "last_flags": None,
            }
            # A newly observed boot supersedes the previous stream. A delayed
            # packet from a known old boot never changes this selection.
            self.current_boot[node_id] = boot_id
            if node_id in self.metadata:
                del self.metadata[node_id]
                metadata_changed = True
        stream = self.streams[key]
        stream["last_packet_monotonic_ns"] = receive_monotonic_ns
        is_current = self.current_boot[node_id] == boot_id
        readings = []
        if packet["type"] == "metadata":
            if is_current:
                self.metadata[node_id] = packet
                metadata_changed = True
        else:
            # Late packets cannot decrease cumulative per-boot counters. Their
            # unmodified counter values are still preserved in the raw envelope.
            stream["diagnostics"] = {
                name: max(stream["diagnostics"].get(name, 0), packet["diagnostics"][name])
                for name in DIAGNOSTIC_FIELDS
            }
            for sample in packet["samples"]:
                # Receiving old data does not prove acquisition is progressing.
                # Duplicates or reordered packets must not hide a stalled node.
                if sample["seq"] > stream["tracker"].highest_seq:
                    stream["last_sample_monotonic_ns"] = receive_monotonic_ns
                stream["tracker"].observe(sample["seq"])
                minimum = stream["minimum_seq"]
                stream["minimum_seq"] = sample["seq"] if minimum is None else min(minimum, sample["seq"])
                # Invalid new samples also advance this watermark. Otherwise a
                # delayed older valid sample could make the display go backward.
                if not is_current or sample["seq"] <= stream["highest_display_seq"]:
                    continue
                stream["highest_display_seq"] = sample["seq"]
                stream["last_flags"] = sample["flags"]
                reading = display_reading(node_id, sample)
                if reading is not None:
                    readings.append(reading)
        return envelope, readings, metadata_changed

    def metadata_snapshot(self):
        return {
            "version": 1,
            "nodes": {str(node): packet for node, packet in sorted(self.metadata.items())},
        }

    def diagnostics_snapshot(self, now_monotonic_ns):
        streams = []
        for (node, boot), stream in sorted(self.streams.items()):
            summary = stream["tracker"].summary()
            minimum = stream["minimum_seq"]
            summary["first_observed_sequence"] = minimum
            # Unlike upstream missing_samples, this excludes samples sent before
            # this receiver started. Late arrivals shrink/recover the range.
            summary["missing_samples_in_recording"] = (
                0 if minimum is None else summary["highest_seq"] - minimum + 1 - summary["unique_samples"]
            )
            last_sample = stream["last_sample_monotonic_ns"]
            age = None if last_sample is None else max(0.0, (now_monotonic_ns - last_sample) / 1e9)
            streams.append({
                "node_id": node, "role": ROLES[node], "boot_id": boot,
                "current_boot": self.current_boot[node] == boot,
                "sample_age_s": age, "stale": age is None or age > self.stale_timeout_s,
                "last_flags": stream["last_flags"],
                "firmware_diagnostics": dict(stream["diagnostics"]), **summary,
            })
        nodes = []
        for node in sorted(set(self.expected_node_ids) | set(self.current_boot)):
            boot = self.current_boot.get(node)
            stream = self.streams.get((node, boot))
            last_sample = None if stream is None else stream["last_sample_monotonic_ns"]
            age = None if last_sample is None else max(0.0, (now_monotonic_ns - last_sample) / 1e9)
            nodes.append({
                "node_id": node, "role": ROLES[node], "boot_id": boot,
                "expected": node in self.expected_node_ids,
                "never_seen": last_sample is None, "sample_age_s": age,
                "stale": age is None or age > self.stale_timeout_s,
                "metadata_received": node in self.metadata,
            })
        return {
            "version": 1, "timestamp_basis": "host_receive_unsynchronized",
            "freshness_basis": "latest_advancing_sample_sequence",
            "valid_packets": self.valid_packets, "invalid_packets": self.invalid_packets,
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

