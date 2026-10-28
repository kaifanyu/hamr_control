"""Ordinary CASTLOG2 format: two independently timestamped channels per record.

Binary decode is separate from ball CASTLOG1. Shared lifecycle, CRC validation,
durable exports, and clock estimation are implemented by caster_recording_format.
"""

import math
from pathlib import Path
import struct
import zlib

from . import protocol

MAGIC = b"CASTLOG2"
VERSION = 2
RECORD = struct.Struct("<QqqqHhIII8dI")


def decode_record(data, node_id=4):
    active = protocol.channels(node_id)
    if len(data) != RECORD.size:
        raise ValueError("truncated ordinary sample record")
    values = RECORD.unpack(data)
    if zlib.crc32(data[:-4]) != values[-1]:
        raise ValueError("sample CRC mismatch")
    (seq, stamp, wheel_stamp, count, spi, position, swivel_flags, wheel_flags, reserved,
     absolute, calibrated, angle, velocity, filtered, wheel_angle, wheel_velocity,
     wheel_filtered, _) = values
    if reserved != 0:
        raise ValueError("nonzero ordinary record reserved bytes")
    protocol.integer(position, "position12", -1, 4095)
    def finite(value):
        return value if math.isfinite(value) else None
    sample = dict(
        seq=seq, t_us=stamp,
        swivel=dict(raw_spi_word=spi, position12=position if position >= 0 else None,
                    flags=swivel_flags, absolute_angle_rad=finite(absolute),
                    calibrated_angle_rad=finite(calibrated), angle_rad=finite(angle),
                    velocity_rad_s=finite(velocity), filtered_velocity_rad_s=finite(filtered)),
        wheel=dict(t_us=wheel_stamp, raw_count=str(count) if wheel_flags & protocol.SAMPLE_VALID else None,
                   flags=wheel_flags, angle_rad=finite(wheel_angle),
                   velocity_rad_s=finite(wheel_velocity), filtered_velocity_rad_s=finite(wheel_filtered)),
    )
    for channel in ("swivel", "wheel"):
        if channel not in active:
            del sample[channel]
    return protocol.validate_sample(sample, node_id)


def iter_samples(path):
    from caster_recording_format import iter_samples as shared, read_header
    with Path(path).open("rb") as handle:
        if read_header(handle)["packet"].get("caster_type") != "ordinary":
            raise ValueError("expected an ordinary caster log")
    yield from shared(path)


def verify_log(path, expected_status=None, expected_run_id=None):
    from caster_recording_format import verify_log as shared
    result = shared(path, expected_status, expected_run_id)
    if result.get("caster_type") != "ordinary":
        raise ValueError("expected an ordinary caster log")
    return result


def channel_rows(packet, sample):
    yield from protocol.channel_rows(packet, sample)
