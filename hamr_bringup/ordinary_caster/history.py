"""Lossless ordinary-caster acquisition history and conservative pose lookup.

No receipt timestamp is used as an acquisition timestamp. Interpolation is
limited to consecutive, valid observations from the same boot and channel.
"""

from bisect import bisect_right
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

from . import protocol
from caster_encoder_protocol import ClockMapper, parse_json
from caster_recording_format import validate_local_envelope

READY = 1 << 0
CPR_VERIFIED = 1 << 2
PHASE_REFERENCE_VALID = 1 << 3
RATE_BOUND_VERIFIED = 1 << 10
SWIVEL_ERRORS = (1 << 7) | (1 << 12) | protocol.FORMAT_ERROR
TIME_NOTE = ("Device acquisition time mapped by explicit two-way clock estimates. "
             "Reported uncertainty is not an independently validated accuracy bound.")


@dataclass
class Observation:
    seq: int
    t_us: int
    time_ns: int
    flags: int
    values: dict
    mapped_epoch_ns: int | None = None
    segment: int = 0
    valid: bool = False
    position: float | None = None
    continuous: bool = False
    calibrated: bool = False
    reason: str = "invalid_sample"


@dataclass
class ChannelHistory:
    identity: tuple
    channel: str
    metadata: dict
    observations: list = field(default_factory=list)
    clock_mapping: dict | None = None
    clock_problem: str | None = None
    gap_ns: int = 0
    times: list = field(default_factory=list)
    origin_raw_count: int | None = None

    @property
    def wheel_id(self):
        return self.identity[0]

    @property
    def joint_name(self):
        return f"{self.wheel_id}_caster_{self.channel}_joint"

    def at(self, time_ns, allow_uncalibrated=False):
        """Return a joint state and its provenance; missing states have no angle."""
        status = {"wheel_id": self.wheel_id, "channel": self.channel,
                  "device_id": self.identity[1], "node_id": self.identity[2], "boot_id": self.identity[3],
                  "joint": self.joint_name, "valid": False, "angle_rad": None,
                  "wheel_phase_relative": self.channel == "wheel",
                  "origin_raw_count": str(self.origin_raw_count) if self.origin_raw_count is not None else None}
        index = bisect_right(self.times, time_ns) - 1
        if index < 0:
            return {**status, "state": "before_history"}
        current = self.observations[index]
        status.update(seq=current.seq, t_us=current.t_us, flags=current.flags,
                      segment=current.segment, calibrated=current.calibrated)
        if time_ns > current.time_ns and index + 1 == len(self.observations):
            return {**status, "state": "after_history"}
        if not current.valid:
            return {**status, "state": current.reason}
        if not current.calibrated and not allow_uncalibrated:
            return {**status, "state": "calibration_required"}
        angle, state = current.position, "measured"
        if time_ns != current.time_ns:
            following = self.observations[index + 1]
            if following.segment != current.segment:
                return {**status, "state": "gap_or_invalid_boundary"}
            fraction = (time_ns - current.time_ns) / (following.time_ns - current.time_ns)
            if current.continuous and following.continuous:
                angle += fraction * (following.position - current.position)
                state = "interpolated"
            else:
                # Wrapped absolute orientation remains observable after turn
                # continuity is lost. Do not invent the direction through a wrap.
                state = "held_wrapped_observation"
        return {**status, "state": state, "valid": True, "angle_rad": angle,
                "uncalibrated_override": not current.calibrated}


@dataclass
class History:
    source: Path
    run_id: str
    streams: list
    time_basis: str
    warnings: list

    @property
    def start_ns(self):
        return min(stream.times[0] for stream in self.streams)

    @property
    def end_ns(self):
        return max(stream.times[-1] for stream in self.streams)

    @property
    def wheels(self):
        return sorted({stream.wheel_id for stream in self.streams})

    def pose_at(self, time_ns, allow_uncalibrated=False):
        if self.time_basis == "separate_device_clocks" and len({s.identity for s in self.streams}) > 1:
            raise ValueError("Cannot combine device-local clocks; use clock maps or select one board and boot")
        # Multiple boot histories never interpolate into one another. A boot
        # outside its range does not mask another boot active at this time.
        choices = {}
        for stream in self.streams:
            result = stream.at(time_ns, allow_uncalibrated)
            old = choices.get(stream.joint_name)
            if old is None or result["state"] not in ("before_history", "after_history"):
                if old is not None and old["state"] not in ("before_history", "after_history"):
                    raise ValueError(f"Overlapping boot histories for {stream.joint_name}")
                choices[stream.joint_name] = result
        for wheel in self.wheels:
            for channel in ("swivel", "wheel"):
                joint = f"{wheel}_caster_{channel}_joint"
                choices.setdefault(joint, dict(wheel_id=wheel, channel=channel, joint=joint,
                    valid=False, angle_rad=None, state="sensor_history_missing",
                    wheel_phase_relative=channel == "wheel"))
        channels = list(choices.values())
        valid = [item for item in channels if item["valid"]]
        return {"time_ns": time_ns, "time_basis": self.time_basis,
                "joint_names": [item["joint"] for item in valid],
                "positions": [item["angle_rad"] for item in valid],
                "channels": channels, "complete_pose_available": all(item["valid"] for item in channels),
                "accuracy_validated": False, "warnings": self.warnings}


def _clock_problem(mapping, identity, observations, maximum_ns):
    if mapping is None:
        return "missing clock mapping"
    if protocol.identity(mapping) != identity:
        raise ValueError("Clock mapping identity differs from packet")
    mapper = ClockMapper({"version": 1, "mappings": [mapping]})
    if mapping.get("calibrated") is not True:
        return "clock mapping has not been accepted"
    uncertainty = mapping.get("uncertainty_ns")
    if type(uncertainty) is not int or not 0 <= uncertainty <= maximum_ns:
        return "missing or excessive clock uncertainty"
    residual = mapping.get("max_probe_residual_ns", 0)
    if type(residual) is not int or not 0 <= residual <= maximum_ns:
        return "excessive clock probe residual"
    if abs(float(mapping["slope_ns_per_us"]) / 1000 - 1) > 0.002000001:
        return "clock drift exceeds 2000 ppm"
    for stamp in observations:
        if mapper.map(identity[2], identity[3], stamp, identity[0], identity[1]) is None:
            return "clock mapping does not cover every channel timestamp"
    return None


def _prepare(stream):
    meta = stream.metadata[stream.channel]
    previous, origin_count, segment = None, None, 0
    for point in stream.observations:
        flags = point.flags
        point.valid = flags & (READY | protocol.SAMPLE_VALID) == (READY | protocol.SAMPLE_VALID)
        point.continuous = bool(flags & protocol.UNWRAP_CONTINUOUS) and not flags & protocol.CONTINUITY_LOST
        if stream.channel == "swivel":
            point.valid &= not bool(flags & SWIVEL_ERRORS)
            point.valid &= point.values["calibrated_angle_rad"] is not None and point.values["position12"] is not None
            point.calibrated = meta["phase_reference_valid"] and bool(flags & PHASE_REFERENCE_VALID)
            point.continuous &= meta["max_rpm_verified"] and bool(flags & RATE_BOUND_VERIFIED)
            point.continuous &= point.values["angle_rad"] is not None
            point.position = (point.values["angle_rad"] if point.continuous
                              else point.values["calibrated_angle_rad"])
        else:
            count = point.values["raw_count"]
            point.valid &= point.continuous and count is not None
            point.calibrated = meta["cpr_verified"] and bool(flags & CPR_VERIFIED)
            if point.valid:
                count = int(count)  # int64 text remains exact until subtraction.
                if origin_count is None:
                    origin_count = count
                    stream.origin_raw_count = count
                point.position = (count - origin_count) * (math.tau / meta["counts_per_rev"]) * meta["coordinate_sign"]
        point.reason = "valid" if point.valid else "invalid_sample_or_continuity"
        boundary = (previous is None or not point.valid or not previous.valid or
                    point.seq != previous.seq + 1 or point.time_ns - previous.time_ns > stream.gap_ns or
                    point.continuous != previous.continuous or point.calibrated != previous.calibrated)
        if boundary:
            segment += 1
        point.segment = segment
        previous = point
    stream.times = [point.time_ns for point in stream.observations]


def load_history(source, wheel=None, device_local=False, max_clock_uncertainty_ms=10.0,
                 require_clock=False, max_gap_ms=None, clock_map=None, role=None):
    """Read a durable export or run folder, retaining every channel observation.

    Analysis may inspect separate device clocks. Replay sets require_clock=True
    and needs accepted maps, or explicit device_local=True with one board/boot.
    Run exports carry manifest-derived maps in each envelope; clock-map.json or
    an explicit --clock-map may supply a later independently reviewed mapping.
    """
    source = Path(source).expanduser().resolve()
    source = source / "packets.jsonl" if source.is_dir() else source
    if not math.isfinite(max_clock_uncertainty_ms) or max_clock_uncertainty_ms <= 0:
        raise ValueError("Clock uncertainty limit must be positive and finite")
    if max_gap_ms is not None and (not math.isfinite(max_gap_ms) or max_gap_ms <= 0):
        raise ValueError("Gap limit must be positive and finite")
    if wheel is not None and wheel not in protocol.WHEELS:
        raise ValueError("Unknown wheel selection")
    if role is not None and role not in protocol.ROLES.values():
        raise ValueError("Unknown ordinary role")
    supplemental = {}
    map_path = Path(clock_map) if clock_map else source.parent / "clock-map.json"
    if clock_map or map_path.is_file():
        document = parse_json(map_path.read_text(encoding="utf-8"))
        ClockMapper(document)
        supplemental = {protocol.identity(item): item for item in document["mappings"]
                        if item.get("caster_type") == "ordinary"}
    metadata, samples, mappings, runs = {}, {}, {}, set()
    guard = protocol.IdentityGuard()
    with source.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                envelope = parse_json(line)
                validate_local_envelope(envelope)
                packet = envelope["packet"]
                key = protocol.identity(packet)
                if (wheel and key[0] != wheel) or (role and protocol.ROLES[key[2]] != role):
                    continue
                guard.accept(packet)
                runs.add(envelope["recording"]["run_id"])
                mapping = supplemental.get(key, envelope.get("clock_mapping"))
                if key in mappings and mappings[key] != mapping:
                    raise ValueError("Clock mapping changes within one board boot")
                mappings[key] = mapping
                if packet["type"] == "metadata":
                    if key in metadata and metadata[key] != packet:
                        raise ValueError("Conflicting metadata for one board boot")
                    metadata[key] = packet
                else:
                    samples.setdefault(key, []).extend(packet["samples"])
            except (ValueError, KeyError, TypeError) as error:
                raise ValueError(f"{source}:{line_number}: {error}") from error
    if len(runs) != 1 or not samples:
        raise ValueError("Select one nonempty ordinary recording run")
    if device_local and len(samples) != 1:
        raise ValueError("--device-local requires one sensor board and one boot; select --wheel and --role or a separate run")
    problems = {}
    for key, values in samples.items():
        if key not in metadata:
            raise ValueError(f"Missing metadata for {key}")
        for previous, current in zip(values, values[1:]):
            if (current["seq"] <= previous["seq"] or current["t_us"] <= previous["t_us"] or
                    any(now <= before for now, before in zip(protocol.channel_timestamps(key[2], current), protocol.channel_timestamps(key[2], previous)))):
                raise ValueError(f"Duplicate or nonmonotonic acquisition history for {key}")
        stamps = [stamp for value in values for stamp in protocol.channel_timestamps(key[2], value)]
        problems[key] = _clock_problem(mappings[key], key, stamps, int(max_clock_uncertainty_ms * 1e6))
    use_maps = not device_local and not any(problems.values())
    if require_clock and not device_local and not use_maps:
        detail = "; ".join(f"{key[0]}/{key[3]}: {problem}" for key, problem in problems.items() if problem)
        raise ValueError(f"Cannot align ordinary caster histories: {detail}. Use --device-local with one board explicitly.")
    warnings = [f"{key[0]}/{key[3]}: {problem}" for key, problem in problems.items() if problem]
    if use_maps:
        warnings.append(TIME_NOTE)
    else:
        warnings.append("Device-local time only; separate boards or boots have no common time origin.")
    warnings.append("Wheel phase is relative to the first valid count in each recording/boot, not the configured boot zero; Hall A/B has no absolute wheel phase or presence detection.")
    streams = []
    for key, values in samples.items():
        mapper = ClockMapper({"version": 1, "mappings": [mappings[key]]}) if mappings[key] else None
        gap = int(max_gap_ms * 1e6) if max_gap_ms is not None else 3_000_000_000 // metadata[key]["sample_hz"]
        for channel in protocol.channels(key[2]):
            stream = ChannelHistory(key, channel, metadata[key], clock_mapping=mappings[key],
                                    clock_problem=problems[key], gap_ns=gap)
            for sample in values:
                stamp = sample["t_us"] if channel == "swivel" else sample["wheel"]["t_us"]
                mapped = mapper.map(key[2], key[3], stamp, key[0], key[1]) if mapper else None
                stream.observations.append(Observation(sample["seq"], stamp, mapped if use_maps else stamp * 1000,
                                                       sample[channel]["flags"], dict(sample[channel]), mapped))
            _prepare(stream)
            streams.append(stream)
    # A shared mapped clock must not give one physical joint two active boots.
    if use_maps:
        for first in streams:
            for second in streams:
                if (first.identity < second.identity and first.joint_name == second.joint_name and
                        max(first.times[0], second.times[0]) <= min(first.times[-1], second.times[-1])):
                    raise ValueError(f"Overlapping boot clocks for {first.joint_name}")
    return History(source, next(iter(runs)), streams,
                   "estimated_acquisition_epoch" if use_maps else
                   "device_local" if device_local else "separate_device_clocks", warnings)


def add_history_arguments(parser):
    parser.add_argument("input", help="Downloaded ordinary run folder or exported packets.jsonl")
    parser.add_argument("--wheel", choices=protocol.WHEELS)
    parser.add_argument("--role", choices=tuple(protocol.ROLES.values()), help="Select one sensor board; needed for split-board device-local replay")
    parser.add_argument("--device-local", action="store_true", help="Explicit single-board, single-boot clock")
    parser.add_argument("--clock-map", help="Explicit replacement clock-map.json, keyed by wheel/MAC/boot")
    parser.add_argument("--max-clock-uncertainty-ms", type=float, default=10.0)
    parser.add_argument("--max-gap-ms", type=float, help="Interpolation gap limit; default three acquisition periods")


def history_from_args(args, require_clock=False):
    return load_history(args.input, wheel=args.wheel, device_local=args.device_local,
                        role=args.role, clock_map=args.clock_map, max_clock_uncertainty_ms=args.max_clock_uncertainty_ms,
                        max_gap_ms=args.max_gap_ms, require_clock=require_clock)
