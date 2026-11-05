"""Manual ordinary-swivel calibration using native AMT222A-V samples.

ROS is needed only for capture. Never commands motion or edits firmware.
Unwrapping is conditional on an operator-supplied speed bound, independently
of firmware's optional (and possibly unavailable) accumulated angle.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

from . import protocol

N = 4096
TAU = 2 * math.pi
POSES = (
    ("zero", "Fork upright; wheel axle BEHIND the vertical swivel axis (robot rear, -X)"),
    ("quarter", "Turn +90 degrees CCW viewed from ABOVE: axle on robot RIGHT (-Y)"),
    ("opposite", "Continue to +180 degrees: axle in FRONT of the swivel axis (+X)"),
    ("return_zero", "Return to zero: axle BEHIND the swivel axis (-X)"),
)


def wrap(angle):
    return (np.asarray(angle) + math.pi) % TAU - math.pi


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def write_new(path, value):
    payload = json.dumps(value, indent=2, allow_nan=False) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        stream.write(payload)


def swivel_packet(envelope, wheel):
    packet = protocol.decode_packet(json.dumps(envelope["packet"], allow_nan=False).encode())
    if packet["wheel_id"] != wheel:
        raise ValueError(f"Expected identified {wheel} ordinary packets")
    return packet if "swivel" in protocol.channels(packet) else None


def sample_rows(packet):
    """Retain invalid observations for identity/time checks, never as angles."""
    if packet["type"] != "samples":
        return []
    rows = []
    for sample in packet["samples"]:
        values = sample["swivel"]
        flags, count, angle = values["flags"], values["position12"], values["absolute_angle_rad"]
        valid = flags & 3 == 3 and not flags & ((1 << 7) | (1 << 12) | protocol.FORMAT_ERROR)
        valid = bool(valid and count is not None and angle is not None)
        if valid:
            word = values["raw_spi_word"]
            even = sum((word >> bit) & 1 for bit in range(0, 14, 2)) % 2
            odd = sum((word >> bit) & 1 for bit in range(1, 14, 2)) % 2
            if (word & 3 or (word >> 14) & 1 != even ^ 1 or (word >> 15) & 1 != odd ^ 1
                    or (word & 0x3fff) >> 2 != count):
                raise ValueError("Valid swivel sample has inconsistent raw SPI parity/format/position12")
            if not math.isclose(angle, count * TAU / N, rel_tol=0, abs_tol=1e-12):
                raise ValueError("Raw swivel angle disagrees with position12")
        rows.append(dict(wheel_id=packet["wheel_id"], device_id=packet["device_id"],
                         node_id=packet["node_id"], boot_id=packet["boot_id"],
                         seq=sample["seq"], t_us=sample["t_us"], valid=valid, **values))
    return rows


class StreamGuard:
    """Pin even invalid packets to one device/role/boot; verify duplicates."""
    def __init__(self):
        self.identity = None
        self.seen = {}

    def accept(self, packet):
        identity = protocol.identity(packet)
        if self.identity is not None and identity != self.identity:
            raise ValueError("SWIVEL device/role changed or rebooted; use a new capture")
        self.identity = identity
        fresh = []
        for row in sample_rows(packet):
            old = self.seen.get(row["seq"])
            if old is not None:
                if old != row:
                    raise ValueError("Conflicting duplicate swivel sequence")
            else:
                self.seen[row["seq"]] = row
                fresh.append(row)
        return fresh


def ordered_valid(rows):
    if not rows:
        raise ValueError("No swivel samples")
    identities = {(r["wheel_id"], r["device_id"], r["node_id"], r["boot_id"]) for r in rows}
    if len(identities) != 1:
        raise ValueError("SWIVEL identity changed between poses")
    seen = {}
    for row in rows:
        if row["seq"] in seen and seen[row["seq"]] != row:
            raise ValueError("Conflicting duplicate swivel sequence")
        seen[row["seq"]] = row
    ordered = sorted(seen.values(), key=lambda row: row["seq"])
    if any(b["t_us"] <= a["t_us"] for a, b in zip(ordered, ordered[1:])):
        raise ValueError("Swivel acquisition clock is not increasing")
    valid = [row for row in ordered if row["valid"]]
    if not valid:
        raise ValueError("No valid swivel samples; check SPI diagnostics")
    return valid, len(ordered) - len(valid)


def pose_summary(rows, min_duration_s=1.5):
    valid, rejected = ordered_valid(rows)
    times = np.array([row["t_us"] for row in valid], dtype=np.int64)
    if len(valid) < 20 or (times[-1] - times[0]) / 1e6 < min_duration_s:
        raise ValueError("Insufficient fresh samples for stationary swivel pose")
    if np.max(np.diff(times)) > 250000:
        raise ValueError("Stationary swivel capture has a gap above 0.25 s")
    angles = np.array([row["position12"] for row in valid]) * TAU / N
    mean = float(np.angle(np.mean(np.exp(1j * angles))) % TAU)
    spread = float(np.degrees(np.max(np.abs(wrap(angles - mean)))))
    if spread > .5:
        raise ValueError(f"Swivel moved during stationary pose ({spread:.3f} deg)")
    return dict(mean_raw_rad=mean, mean_position12=mean * N / TAU,
                max_deviation_deg=spread, valid_samples=len(valid), rejected_samples=rejected,
                samples=rows)


def solve_reference(wheel, observations):
    if set(observations) != {name for name, _ in POSES}:
        raise ValueError("Require zero, quarter, opposite, and return_zero captures")
    valid, _ = ordered_valid([row for pose in observations.values() for row in pose["samples"]])
    if valid[0]["wheel_id"] != wheel:
        raise ValueError("Calibration wheel does not match observations")
    z, q, h, r = [observations[name]["mean_raw_rad"] for name, _ in POSES]
    quarter = float(wrap(q - z))
    sign = 1 if quarter > 0 else -1
    checks = dict(quarter_error_deg=abs(math.degrees(quarter) - sign * 90),
                  half_turn_error_deg=abs(float(np.degrees(wrap(h - z - math.pi)))),
                  return_zero_error_deg=abs(float(np.degrees(wrap(r - z)))))
    if checks["quarter_error_deg"] > 5:
        raise ValueError("Quarter-turn check failed; a half/full turn cannot determine physical sign")
    if checks["half_turn_error_deg"] > 5:
        raise ValueError("Opposite-pose check failed")
    if checks["return_zero_error_deg"] > 2:
        raise ValueError("Return-to-zero check failed")
    # Equal pose weighting avoids bias toward whichever dwell delivered more packets.
    mean = float(np.angle(np.exp(1j * z) + np.exp(1j * r)) % TAU)
    count = int(round(mean * N / TAU)) % N
    zero = count * TAU / N
    return dict(format="ordinary_swivel_calibration_v1", caster_type="ordinary", wheel_id=wheel,
                device_id=valid[0]["device_id"], node_id=valid[0]["node_id"], boot_id=valid[0]["boot_id"],
                encoder_type="AMT222A-V", counts_per_revolution=N,
                zero_position12=count, zero_absolute_angle_rad=zero, direction_sign=sign,
                physical_direction_verified=True, accuracy_validated=False,
                reference_pose=POSES[0][1], direction_convention="Positive about +Z: CCW viewed from above",
                angle_formula="wrap_signed(direction_sign * (position12 * 2*pi/4096 - zero_absolute_angle_rad))",
                angle_source="Native position12, before firmware sign/zero; scale unchanged",
                firmware_settings=dict(SWIVEL_ZERO_RAD=zero, SWIVEL_SIGN=sign, SWIVEL_PHASE_REFERENCE_VALID=True),
                max_rpm_verified=False,
                reuse_conditions="Same encoder and mechanical mounting; repeat after shaft/encoder remount or hardware zero reset",
                created_utc=datetime.now(timezone.utc).isoformat(), checks=checks, observations=observations)


class LiveCapture:
    def __init__(self, wheel, output):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        self.ros, self.output, self.wheel = rclpy, output, wheel
        self.guard, self.rows, self.errors = StreamGuard(), [], []
        self.ignore_until, self.phase = time.monotonic() + 1, "turn"
        rclpy.init(args=[])
        self.node = rclpy.create_node("calibrate_ordinary_swivel")
        self.node.create_subscription(String, f"/ordinary_caster/{wheel}/packets", self.callback,
                                      QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE))

    def callback(self, message):
        if time.monotonic() < self.ignore_until:
            return
        try:
            envelope = protocol.parse_json(message.data)
            packet = swivel_packet(envelope, self.wheel)
            if packet is None:
                return
            self.output.write(json.dumps(dict(phase=self.phase, envelope=envelope), allow_nan=False) + "\n")
            self.output.flush()
            received = envelope.get("receive_epoch_ns")
            if type(received) is not int or abs(time.time_ns() - received) > 1_000_000_000:
                raise ValueError("Stale/clock-mismatched packet; use live telemetry, not playback")
            self.rows.extend(self.guard.accept(packet))
        except (ValueError, TypeError, KeyError) as error:
            self.errors.append(error)

    def poll(self):
        if not self.ros.ok():
            raise RuntimeError("ROS shutdown during capture")
        self.ros.spin_once(self.node, timeout_sec=.05)
        if self.errors:
            raise self.errors[0]
        rows, self.rows = self.rows, []
        return rows

    def pose(self, name, duration):
        self.phase, self.rows = name, []
        self.ignore_until = time.monotonic() + 1  # Drain traffic queued while operator positioned fork.
        rows = []
        while time.monotonic() < self.ignore_until + duration:
            rows.extend(self.poll())
        return pose_summary(rows, min_duration_s=.75 * duration)

    def close(self):
        self.node.destroy_node()
        if self.ros.ok():
            self.ros.shutdown()


def open_history(path, wheel, kind, **extra):
    path.parent.mkdir(parents=True, exist_ok=True)
    stream = path.open("x", encoding="utf-8")
    stream.write(json.dumps(dict(format=kind, wheel_id=wheel, caster_type="ordinary",
                                counts_per_revolution=N, topic=f"/ordinary_caster/{wheel}/packets",
                                created_utc=datetime.now(timezone.utc).isoformat(), **extra)) + "\n")
    stream.flush()
    return stream


def capture_reference(args):
    for wheel in protocol.WHEELS if args.wheel == "both" else (args.wheel,):
        target = args.output / f"{wheel}_swivel_calibration.json"
        history = args.output / f"{wheel}_swivel_observations.jsonl"
        fragment = args.output / f"{wheel}_swivel_calibration.h"
        if any(path.exists() for path in (target, history, fragment)):
            raise ValueError(f"{wheel} output exists; choose a new calibration directory")
        observations = {}
        print(f"\n{wheel}: keep the mount fixed and upright; rotate the fork around its vertical shaft.\n"
              "Use robot front/right/rear directions, not your viewing direction. Wheel spin is irrelevant.", flush=True)
        with open_history(history, wheel, "ordinary_swivel_poses_v1") as output:
            capture = LiveCapture(wheel, output)
            try:
                for name, instruction in POSES:
                    input(f"\n{name}: {instruction}. Hold still, then press Enter: ")
                    observations[name] = capture.pose(name, args.duration)
                    print(f"Raw mean: {observations[name]['mean_position12']:.3f} / {N} counts", flush=True)
            finally:
                capture.close()
        result = solve_reference(wheel, observations)
        result["source_history"] = str(history.resolve())
        result["source_sha256"] = hashlib.sha256(history.read_bytes()).hexdigest()
        write_new(target, result)
        with fragment.open("x", encoding="utf-8") as stream:
            stream.write(f"// {wheel} / {result['device_id']}; measured zero/sign, not a rate qualification.\n"
                         "// Replace these definitions in this wheel's ordinary_caster/config/*.local.h.\n"
                         f"#define SWIVEL_ZERO_RAD {result['zero_absolute_angle_rad']:.17g}\n"
                         f"#define SWIVEL_SIGN {result['direction_sign']}\n"
                         "#define SWIVEL_PHASE_REFERENCE_VALID true\n")
        print(f"Saved {target}\nFirmware settings: {fragment}\n"
              "Apply these three settings to the matching private header and rebuild/reflash SWIVEL to use them.\n"
              "This does not qualify SWIVEL_MAX_RPM_VERIFIED.", flush=True)


def record(path, seconds, wheel):
    if seconds < 10:
        raise ValueError("Allow at least 10 seconds including stationary endpoints")
    with open_history(path, wheel, "ordinary_swivel_turn_v1", requested_seconds=seconds,
                      reference="User-positioned mark; no physical zero assigned") as output:
        capture = LiveCapture(wheel, output)
        started, last_valid, go, stop = None, time.monotonic(), False, False
        print(f"Listening to /ordinary_caster/{wheel}/packets. Hold the swivel at your mark.", flush=True)
        try:
            while True:
                rows = [row for row in capture.poll() if row["valid"]]
                now = time.monotonic()
                if rows:
                    last_valid = now
                    if started is None:
                        started = now
                        print(f"Starting raw position: {rows[0]['position12']} counts. HOLD STILL for 3 seconds.", flush=True)
                if now - last_valid > 5:
                    raise ValueError("No fresh valid SWIVEL samples for 5 seconds; check live telemetry/SPI diagnostics")
                if started is None:
                    continue
                elapsed = now - started
                if elapsed >= 3 and not go:
                    print("GO: turn the fork ONE full revolution around the vertical shaft in one direction, "
                          "back to your mark; then HOLD STILL. Do not reverse through another revolution.", flush=True)
                    go = True
                if elapsed >= seconds - 3 and not stop:
                    print("Final 3 seconds: HOLD STILL at the starting mark.", flush=True)
                    stop = True
                if elapsed >= seconds:
                    break
        finally:
            capture.close()
            print(f"History saved: {path}; check evaluates the complete history.", flush=True)


def evaluate(path, max_rpm):
    if not math.isfinite(max_rpm) or max_rpm <= 0:
        raise ValueError("Maximum RPM must be finite and positive")
    guard = StreamGuard()
    with path.open(encoding="utf-8") as source:
        header = protocol.parse_json(next(source, "{}"))
        wheel = header.get("wheel_id")
        if header.get("format") != "ordinary_swivel_turn_v1" or wheel not in protocol.WHEELS:
            raise ValueError("Not an identified ordinary_swivel_turn_v1 recording")
        for number, line in enumerate(source, 2):
            try:
                packet = swivel_packet(protocol.parse_json(line)["envelope"], wheel)
                if packet is not None:
                    guard.accept(packet)
            except (ValueError, KeyError, TypeError) as error:
                raise ValueError(f"Line {number}: {error}") from error
    rows, rejected = ordered_valid(list(guard.seen.values()))
    times = np.array([row["t_us"] - rows[0]["t_us"] for row in rows]) * 1e-6
    if times[-1] < 6:
        raise ValueError("Need at least 6 seconds including stationary endpoints")
    counts = np.array([row["position12"] for row in rows], dtype=float)
    allowed = max_rpm / 60 * N * np.diff(times) + 2
    if np.any(allowed >= N / 2):
        raise ValueError("Ambiguous swivel gap at supplied maximum RPM; lost turns cannot be recovered")
    delta = (np.diff(counts) + N / 2) % N - N / 2
    if np.any(np.abs(delta) > allowed):
        raise ValueError("Observed swivel motion exceeds supplied maximum RPM")
    angle = np.r_[0., np.cumsum(delta)] * TAU / N
    first, last = times <= 2, times >= times[-1] - 2
    start = pose_summary([row for row, keep in zip(rows, first) if keep])
    finish = pose_summary([row for row, keep in zip(rows, last) if keep])
    rotation = float(np.mean(angle[last]) - np.mean(angle[first]))
    relative = angle - np.mean(angle[first])
    sign = 1 if relative[np.argmax(np.abs(relative))] >= 0 else -1
    progress = sign * relative
    backtrack = float(np.degrees(np.max(np.maximum.accumulate(progress) - progress)))
    error = float(np.degrees(wrap(finish["mean_raw_rad"] - start["mean_raw_rad"])))
    turn_error = abs(abs(math.degrees(rotation)) - 360)
    passed = turn_error <= 2 and abs(error) <= 2 and backtrack <= 2
    return dict(result="PASS" if passed else "FAIL", wheel_id=wheel, caster_type="ordinary",
                device_id=rows[0]["device_id"], node_id=rows[0]["node_id"], boot_id=rows[0]["boot_id"],
                counts_per_revolution=N, start_position12_mean=start["mean_position12"],
                finish_position12_mean=finish["mean_position12"], signed_rotation_deg=math.degrees(rotation),
                signed_revolutions=rotation / TAU, return_error_deg=error, full_turn_error_deg=turn_error,
                maximum_backtrack_deg=backtrack, tolerance_deg=2, valid_unique_samples=len(rows),
                rejected_samples=rejected, duration_s=float(times[-1]), max_rpm_assumption=max_rpm,
                max_valid_sample_gap_s=float(np.max(np.diff(times))),
                note="Conditional on supplied speed bound and physical mark. Checks turn history/return repeatability; "
                     "does not establish physical zero/sign, angular accuracy, or a verified maximum rate.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    zero = commands.add_parser("zero", help="Guided zero, +90 CCW, opposite, return-zero captures")
    zero.add_argument("--wheel", choices=["both", *protocol.WHEELS], default="both")
    zero.add_argument("--output", type=Path, required=True)
    zero.add_argument("--duration", type=positive, default=3., help="Stationary seconds per pose (minimum 2)")
    capture = commands.add_parser("record", help="Record one manual full swivel turn with stationary endpoints")
    capture.add_argument("file", type=Path)
    capture.add_argument("--wheel", choices=protocol.WHEELS, default="front_left")
    capture.add_argument("--seconds", type=positive, default=30.)
    check = commands.add_parser("check", help="Evaluate saved turn history without ROS")
    check.add_argument("file", type=Path)
    check.add_argument("--max-rpm", type=positive, required=True,
                       help="Credible physical upper speed bound for the entire turn; not a fitted rate")
    check.add_argument("--report", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "zero":
            if args.duration < 2:
                raise ValueError("Stationary duration must be at least 2 seconds")
            capture_reference(args)
        elif args.command == "record":
            record(args.file, args.seconds, args.wheel)
        else:
            result = evaluate(args.file, args.max_rpm)
            if args.report:
                write_new(args.report, result)
            print(json.dumps(result, indent=2, allow_nan=False))
            return 0 if result["result"] == "PASS" else 1
        return 0
    except (ValueError, RuntimeError, OSError, TypeError, KeyError, ImportError, EOFError) as error:
        print(f"calibrate_ordinary_swivel: {error}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Interrupted; captured history retained.", file=sys.stderr)
        return 130
