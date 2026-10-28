"""Strict v4 split-board and legacy v3 ordinary-caster wire contract, independent of ROS and hardware.

Each split board owns one channel, sequence, boot identity and clock.
Legacy v3 paired records retain their two acquisition timestamps. A valid packet is not proof
of physical calibration, sensor presence, or synchronization.
"""

import math

from pathlib import Path
import sys

try:
    import caster_encoder_protocol
except ModuleNotFoundError as error:
    if error.name != "caster_encoder_protocol":
        raise
    # In source, ordinary_caster and scripts are siblings. Installation places
    # this package beside the executable helpers in lib/hamr_bringup.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from caster_encoder_protocol import (
    BOOT_ID, DEVICE_ID, DIAGNOSTIC_FIELDS, FLAG_NAMES as BALL_FLAG_NAMES,
    INT64_MAX, MAX_DATAGRAM, UINT64_MAX, WHEELS, count_text, integer, parse_json,
)

FLAG_NAMES = (*BALL_FLAG_NAMES, "format_error")
SAMPLE_VALID = 1 << 1
UNWRAP_CONTINUOUS = 1 << 4
VELOCITY_VALID = 1 << 6
CONTINUITY_LOST = 1 << 8
FORMAT_ERROR = 1 << 13
ROLES = {4: "ORDINARY", 5: "SWIVEL", 6: "WHEEL"}
CHANNELS = {4: ("swivel", "wheel"), 5: ("swivel",), 6: ("wheel",)}


def channels(value):
    node = value["node_id"] if isinstance(value, dict) else value
    if type(node) is not int or node not in CHANNELS:
        raise ValueError("ordinary node_id must be 4 (legacy), 5 (SWIVEL), or 6 (WHEEL)")
    return CHANNELS[node]


def channel_timestamps(packet, sample):
    return tuple(sample["t_us"] if channel == "swivel" else sample[channel]["t_us"]
                 for channel in channels(packet))
SWIVEL_FIELDS = (
    "raw_spi_word", "position12", "absolute_angle_rad", "calibrated_angle_rad",
    "angle_rad", "velocity_rad_s", "filtered_velocity_rad_s", "flags",
)
WHEEL_FIELDS = (
    "t_us", "raw_count", "angle_rad", "velocity_rad_s",
    "filtered_velocity_rad_s", "flags",
)
VALUE_FIELDS = (
    "raw_count", "raw_spi_word", "position12", "angle_rad", "absolute_angle_rad",
    "calibrated_angle_rad", "velocity_rad_s", "filtered_velocity_rad_s", "flags",
)
CSV_FIELDS = (
    "receive_epoch_ns", "receive_monotonic_ns", "source_ip", "source_port",
    "caster_type", "wheel_id", "device_id", "node_id", "boot_id", "channel",
    "seq", "t_us", *VALUE_FIELDS, *FLAG_NAMES, "sequence_status", *DIAGNOSTIC_FIELDS,
)


def number(value, name, minimum=None, maximum=None, nullable=False):
    if nullable and value is None:
        return value
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number" + (" or null" if nullable else ""))
    if minimum is not None and value < minimum or maximum is not None and value > maximum:
        raise ValueError(f"{name} is outside its permitted range")
    return value


def boolean(value, name):
    if type(value) is not bool:
        raise ValueError(f"{name} must be a boolean")
    return value


def identity(value):
    """Validate ordinary recorder identity (also usable for status/clock replies)."""
    if not isinstance(value, dict) or value.get("caster_type") != "ordinary":
        raise ValueError("caster_type must be ordinary")
    channels(value.get("node_id"))
    if value.get("wheel_id") not in WHEELS:
        raise ValueError("wheel_id must be front_left or front_right")
    if not isinstance(value.get("device_id"), str) or not DEVICE_ID.fullmatch(value["device_id"]):
        raise ValueError("device_id must be a lowercase colon-separated MAC")
    if not isinstance(value.get("boot_id"), str) or not BOOT_ID.fullmatch(value["boot_id"]):
        raise ValueError("boot_id must be 16 hexadecimal characters")
    return value["wheel_id"], value["device_id"], value["node_id"], value["boot_id"].lower()


def validate_metadata(packet):
    if packet.get("role") != ROLES[packet["node_id"]]:
        raise ValueError("ordinary metadata role must agree with node_id")
    for name in ("firmware_version", "idf_version"):
        if not isinstance(packet.get(name), str) or not 1 <= len(packet[name]) <= 128:
            raise ValueError(f"metadata requires {name}")
    integer(packet.get("sample_hz"), "sample_hz", 2, 1000)
    integer(packet.get("transmit_hz"), "transmit_hz", 1, 1000)
    integer(packet.get("samples_per_packet"), "samples_per_packet", 1, 1)
    integer(packet.get("record_version"), "record_version", 2, 2)
    integer(packet.get("record_bytes"), "record_bytes", 116, 116)
    active = channels(packet)
    for channel in ("swivel", "wheel"):
        if channel not in active:
            if channel in packet:
                raise ValueError("single-sensor metadata contains an inactive channel")
            continue
        config = packet.get(channel)
        if not isinstance(config, dict):
            raise ValueError(f"metadata requires {channel} configuration")
        sign = integer(config.get("coordinate_sign"), f"{channel}.coordinate_sign", -1, 1)
        if sign == 0:
            raise ValueError(f"{channel}.coordinate_sign must be +1 or -1")
        boolean(config.get("phase_reference_valid"), f"{channel}.phase_reference_valid")
        number(config.get("velocity_filter_tau_s"), f"{channel}.velocity_filter_tau_s", 0)
        if channel == "swivel":
            if config.get("encoder_type") != "AMT222A-V":
                raise ValueError("swivel encoder_type must be AMT222A-V")
            integer(config.get("resolution_bits"), "swivel.resolution_bits", 12, 12)
            integer(config.get("counts_per_rev"), "swivel.counts_per_rev", 4096, 4096)
            number(config.get("zero_rad"), "swivel.zero_rad")
            if number(config.get("max_rpm"), "swivel.max_rpm", 0) == 0:
                raise ValueError("swivel.max_rpm must be positive")
            boolean(config.get("max_rpm_verified"), "swivel.max_rpm_verified")
        else:
            integer(config.get("counts_per_rev"), "wheel.counts_per_rev", 1, (1 << 31) - 1)
            boolean(config.get("cpr_verified"), "wheel.cpr_verified")
            count_text(config.get("software_zero_count"), "wheel.software_zero_count")
            if config.get("software_zero_count") is None:
                raise ValueError("wheel.software_zero_count must be an int64 string")
            integer(config.get("velocity_window_us"), "wheel.velocity_window_us", 1, INT64_MAX)
            integer(config.get("glitch_filter_ns"), "wheel.glitch_filter_ns", 0, (1 << 31) - 1)
            for name in ("index_available", "invalid_transition_detection"):
                boolean(config.get(name), f"wheel.{name}")


def validate_sample(sample, node_id=4):
    if not isinstance(sample, dict):
        raise ValueError("sample must be an object")
    integer(sample.get("seq"), "seq", 0, UINT64_MAX)
    integer(sample.get("t_us"), "t_us", 0, INT64_MAX)
    active = channels(node_id)
    for channel, fields in (("swivel", SWIVEL_FIELDS), ("wheel", WHEEL_FIELDS)):
        if channel not in active:
            if channel in sample:
                raise ValueError("single-sensor sample contains an inactive channel")
            continue
        values = sample.get(channel)
        if not isinstance(values, dict) or any(field not in values for field in fields):
            raise ValueError(f"missing {channel} sample fields")
        integer(values["flags"], f"{channel}.flags", 0, (1 << 14) - 1)
        for name in ("angle_rad", "velocity_rad_s", "filtered_velocity_rad_s"):
            number(values[name], f"{channel}.{name}", nullable=True)
        if channel == "swivel":
            integer(values["raw_spi_word"], "swivel.raw_spi_word", 0, 65535)
            if values["position12"] is not None:
                integer(values["position12"], "swivel.position12", 0, 4095)
            for name in ("absolute_angle_rad", "calibrated_angle_rad"):
                number(values[name], f"swivel.{name}", nullable=True)
        else:
            integer(values["t_us"], "wheel.t_us", 0, INT64_MAX)
            count_text(values["raw_count"], "wheel.raw_count")
            if node_id == 6 and values["t_us"] != sample["t_us"]:
                raise ValueError("wheel-only envelope must use the wheel acquisition timestamp")
    return sample


def decode_packet(data):
    """Validate an ordinary v3/v4 datagram without mutating or discarding fields."""
    try:
        if not 0 < len(data) <= MAX_DATAGRAM:
            raise ValueError("empty or oversized datagram")
        packet = parse_json(data.decode("utf-8"))
        if not isinstance(packet, dict) or type(packet.get("v")) is not int or packet["v"] not in (3, 4):
            raise ValueError("ordinary telemetry requires protocol v3 or v4")
        identity(packet)
        if packet["v"] != (3 if packet["node_id"] == 4 else 4):
            raise ValueError("ordinary protocol version does not match board role")
        if packet.get("type") == "metadata":
            validate_metadata(packet)
        elif packet.get("type") == "samples":
            samples = packet.get("samples")
            # Firmware transmits one sample per UDP packet to stay under the MTU.
            # Bounded offline batches remain accepted for lossless log exports.
            if not isinstance(samples, list) or not 1 <= len(samples) <= 32:
                raise ValueError("samples must contain 1 to 32 records")
            diagnostics = packet.get("diagnostics")
            if not isinstance(diagnostics, dict):
                raise ValueError("diagnostics must be an object")
            for name in DIAGNOSTIC_FIELDS:
                integer(diagnostics.get(name), name)
            for sample in samples:
                validate_sample(sample, packet["node_id"])
        else:
            raise ValueError("unknown packet type")
        return packet
    except (UnicodeError, RecursionError, OverflowError, TypeError) as error:
        raise ValueError(f"invalid ordinary packet: {error}") from error


def channel_rows(packet, sample):
    """Yield only physically acquired channels, preserving identity and time."""
    for channel in channels(packet):
        values = sample[channel]
        row = {name: packet[name] for name in ("caster_type", "wheel_id", "device_id", "node_id", "boot_id")}
        row.update(channel=channel, seq=sample["seq"],
                   t_us=sample["t_us"] if channel == "swivel" else values["t_us"])
        row.update({name: values.get(name) for name in VALUE_FIELDS})
        row.update({name: int(bool(values["flags"] & (1 << bit)))
                    for bit, name in enumerate(FLAG_NAMES)})
        row.update(packet["diagnostics"])
        yield row


class IdentityGuard:
    """One MAC per wheel/role; refuse overlapping combined and split hardware."""

    def __init__(self):
        self.assignments = {}
        self.device_assignments = {}

    def accept(self, packet):
        wheel, device, node, _ = identity(packet)
        key = wheel, node
        if (key in self.assignments and self.assignments[key] != device or
                device in self.device_assignments and self.device_assignments[device] != key):
            raise ValueError("ordinary caster device/wheel/role assignment conflict")
        if any(other_wheel == wheel and bool(set(channels(other_node)) & set(channels(node)))
               and other_node != node for other_wheel, other_node in self.assignments):
            raise ValueError("overlapping combined/split ordinary sensor assignments")
        self.assignments[key] = device
        self.device_assignments[device] = key
