#!/usr/bin/env python3
"""Record a marked H1/H2 rotation from ROS packets and compare saved runs."""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import queue
import statistics
import sys
import tempfile
import threading
import time

from caster_encoder_protocol import DIAGNOSTIC_FIELDS, SAMPLE_FIELDS, decode_packet


ROLES = {"H1": 1, "H2": 2}
READY, SAMPLE_VALID, CONTINUOUS = 1, 2, 16
VELOCITY_VALID, CONTINUITY_LOST, INDEX_VALID = 64, 256, 512
CSV_FIELDS = ("phase", "receive_epoch_ns", "receive_ros_ns", "source_ip",
              "node_id", "boot_id", *SAMPLE_FIELDS, *DIAGNOSTIC_FIELDS)


def valid_count(sample):
    needed = READY | SAMPLE_VALID | CONTINUOUS
    return (sample["raw_count"] is not None and sample["flags"] & needed == needed
            and not sample["flags"] & CONTINUITY_LOST)


def marker(row):
    return {key: row[key] for key in ("boot_id", "seq", "t_us", "raw_count", "angle_rad")}


class RotationRecording:
    """Single-threaded recording state; only stdin runs in a background thread."""

    def __init__(self, role, turns, output, expected_ppr=None):
        self.role, self.node_id = role, ROLES[role]
        self.turns, self.expected_ppr = turns, expected_ppr
        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)
        prefix = role + "-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-")
        self.path = Path(tempfile.mkdtemp(prefix=prefix, dir=output)).resolve()
        self.raw_file = (self.path / "packets.jsonl").open("w", encoding="utf-8")
        self.csv_file = (self.path / "samples.csv").open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.rows, self.metadata = [], []
        self.latest = None
        self.latest_observed = None
        self.last_progress = None
        self.start = self.end = None
        self.abort_reason = None
        self.invalid_messages = 0

    def ingest(self, text, now=None):
        now = time.monotonic() if now is None else now
        try:
            envelope = json.loads(text)
            packet = envelope["packet"]
            # Validate using the same protocol parser as the live bridge.
            packet = decode_packet(json.dumps(packet, allow_nan=False).encode("utf-8"))
        except (KeyError, TypeError, ValueError, RecursionError):
            self.invalid_messages += 1
            return
        if packet["node_id"] != self.node_id:
            return
        boot = packet["boot_id"].lower()
        self.raw_file.write(json.dumps(envelope, allow_nan=False) + "\n")
        if self.start and boot != self.start["boot_id"]:
            self.abort_reason = "A different boot ID appeared during the rotation; repeat the run."
        if packet["type"] == "metadata":
            self.metadata.append(packet)
            return
        for sample in packet["samples"]:
            row = {"phase": "rotation" if self.start else "waiting",
                   "receive_epoch_ns": envelope.get("receive_epoch_ns"),
                   "receive_ros_ns": envelope.get("receive_ros_ns"),
                   "source_ip": envelope.get("source_ip"), "node_id": self.node_id,
                   "boot_id": boot, **sample, **packet["diagnostics"]}
            self.writer.writerow(row)
            self.rows.append(row)
            if (self.latest_observed is None or boot != self.latest_observed["boot_id"]
                    or row["seq"] > self.latest_observed["seq"]):
                self.latest_observed = row
            if valid_count(row) and (self.latest is None or boot != self.latest["boot_id"]
                                     or row["seq"] > self.latest["seq"]):
                self.latest = row
                self.last_progress = now

    def mark(self, now=None, timeout=1.0):
        now = time.monotonic() if now is None else now
        if self.latest is None or now - self.last_progress > timeout:
            raise ValueError("No fresh valid count. Keep the shaft still and wait for telemetry.")
        if not valid_count(self.latest_observed):
            raise ValueError("The newest sample has an invalid count; cannot use an older count as an endpoint.")
        if self.start is None:
            self.start = marker(self.latest)
        else:
            if self.latest["boot_id"] != self.start["boot_id"]:
                raise ValueError("Encoder rebooted during the rotation.")
            if self.latest["seq"] <= self.start["seq"]:
                raise ValueError("No new acquisition since the start marker.")
            self.end = marker(self.latest)

    def flush(self):
        self.raw_file.flush()
        self.csv_file.flush()
        # Preserve operator markers even if the process subsequently stops.
        (self.path / "markers.json").write_text(json.dumps({
            "role": self.role, "physical_turns": self.turns,
            "start": self.start, "end": self.end,
        }, indent=2) + "\n", encoding="utf-8")

    def close(self):
        try:
            self.flush()
            summary = summarize(self)
            (self.path / "summary.json").write_text(
                json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
            report = format_summary(summary)
            (self.path / "summary.txt").write_text(report + "\n", encoding="utf-8")
            return summary, report
        finally:
            self.raw_file.close()
            self.csv_file.close()


def summarize(recording):
    r = recording
    result = {"format_version": 1, "role": r.role, "physical_turns": r.turns,
              "measurement_valid": False, "start": r.start, "end": r.end,
              "invalid_messages": r.invalid_messages, "warnings": [],
              "metadata": r.metadata[-1] if r.metadata else None}
    warnings = result["warnings"]
    if r.abort_reason:
        warnings.append(r.abort_reason)
    if r.start is None or r.end is None:
        warnings.append("Incomplete run: both operator markers are required; no calibration computed.")
        return result
    selected = [row for row in r.rows if row["boot_id"] == r.start["boot_id"]
                and r.start["seq"] <= row["seq"] <= r.end["seq"]]
    unique = {}
    conflicts = 0
    for row in selected:
        prior = unique.setdefault(row["seq"], row)
        if any(prior[key] != row[key] for key in SAMPLE_FIELDS):
            conflicts += 1
    rows = [unique[seq] for seq in sorted(unique)]
    valid = [row for row in rows if valid_count(row)]
    missing = r.end["seq"] - r.start["seq"] + 1 - len(rows)
    result.update(samples=len(rows), duplicate_samples=len(selected) - len(rows),
                  conflicting_samples=conflicts, missing_sequences=missing,
                  invalid_count_samples=len(rows) - len(valid))
    metadata = [m for m in r.metadata if m["boot_id"].lower() == r.start["boot_id"]]
    config = metadata[-1] if metadata else {}
    result["metadata"] = config or None
    configured_ppr = config.get("encoder_ppr")
    expected_ppr = r.expected_ppr or configured_ppr or 2048
    if type(expected_ppr) not in (int, float) or not math.isfinite(expected_ppr) or expected_ppr <= 0:
        expected_ppr = 2048
        warnings.append("Invalid metadata PPR; comparison uses assumed 2048 PPR.")
    result["expected_ppr"] = expected_ppr
    result["expected_ppr_source"] = ("argument" if r.expected_ppr else
                                     "firmware_metadata" if configured_ppr else "assumed_2048")
    if not metadata:
        warnings.append("No matching firmware metadata received; verify configured PPR/sign separately.")
    else:
        keys = ("encoder_ppr", "coordinate_sign", "software_zero_count")
        if len({tuple(m.get(k) for k in keys) for m in metadata}) > 1:
            warnings.append("Firmware scale/sign/zero metadata changed during this recording.")
            conflicts += 1
        if not config.get("ppr_verified"):
            warnings.append("Firmware PPR is marked unverified.")
    elapsed = (r.end["t_us"] - r.start["t_us"]) / 1e6
    monotonic = all(b["t_us"] > a["t_us"] for a, b in zip(rows, rows[1:]))
    continuity_lost = any(row["flags"] & CONTINUITY_LOST for row in rows)
    if elapsed <= 0 or not monotonic:
        warnings.append("Acquisition timestamps did not advance monotonically.")
    if continuity_lost:
        warnings.append("Firmware reported lost count continuity; this run cannot calibrate resolution.")
    if conflicts:
        warnings.append("Conflicting samples or configuration; this run cannot calibrate resolution.")
    if missing:
        warnings.append("Missing sequences include skipped acquisition or transport loss. Endpoint count change may still be usable; pulse capture is a separate question.")
    if len(valid) != len(rows):
        warnings.append("Some count snapshots were invalid; inspect samples.csv and flags.")
    usable = not r.abort_reason and not conflicts and not continuity_lost and elapsed > 0 and monotonic
    net = int(r.end["raw_count"]) - int(r.start["raw_count"])
    angle_delta = (r.end["angle_rad"] - r.start["angle_rad"]
                   if r.end["angle_rad"] is not None and r.start["angle_rad"] is not None else None)
    deltas = [int(b["raw_count"]) - int(a["raw_count"]) for a, b in zip(valid, valid[1:])]
    direction = 1 if net >= 0 else -1
    reverse = sum(abs(d) for d in deltas if d * direction < 0)
    dts = [(b["t_us"] - a["t_us"]) / 1e6 for a, b in zip(rows, rows[1:])]
    velocities = [row["velocity_rad_s"] for row in rows
                  if row["flags"] & VELOCITY_VALID and row["velocity_rad_s"] is not None]
    index_events = {}
    for row in rows:
        if (row["flags"] & INDEX_VALID and row["index_raw_count"] is not None
                and r.start["t_us"] <= row["index_timestamp_us"] <= r.end["t_us"]):
            index_events[row["index_event_count"]] = row
    indexes = [index_events[key] for key in sorted(index_events)]
    index_intervals = [{"events": b["index_event_count"] - a["index_event_count"],
                        "counts": int(b["index_raw_count"]) - int(a["index_raw_count"])}
                       for a, b in zip(indexes, indexes[1:])]
    result.update(
        measurement_valid=usable, elapsed_acquisition_s=elapsed, net_counts=net,
        observed_counts_per_rev=abs(net) / r.turns if usable else None,
        observed_ppr_equivalent=abs(net) / (4 * r.turns) if usable else None,
        count_scale_ratio=abs(net) / (4 * expected_ppr * r.turns) if usable else None,
        angle_change_rad=angle_delta,
        angle_change_deg=math.degrees(angle_delta) if angle_delta is not None else None,
        reported_revolutions=angle_delta / math.tau if angle_delta is not None else None,
        mean_velocity_rad_s=angle_delta / elapsed if angle_delta is not None and elapsed > 0 else None,
        peak_abs_velocity_rad_s=max(map(abs, velocities), default=None),
        zero_velocity_fraction=(sum(v == 0 for v in velocities) / len(velocities) if velocities else None),
        reverse_counts=reverse, observed_absolute_count_travel=sum(map(abs, deltas)),
        sample_interval_median_ms=statistics.median(dts) * 1000 if dts else None,
        sample_interval_max_ms=max(dts) * 1000 if dts else None,
        observed_index_events=len(indexes), index_intervals=index_intervals,
        diagnostic_deltas={key: rows[-1][key] - rows[0][key] for key in DIAGNOSTIC_FIELDS},
    )
    if reverse:
        warnings.append("Counts reversed direction during the turn (or jittered); inspect raw samples and repeat monotonically.")
    if net == 0:
        warnings.append("No net counts despite the declared physical turns: check capture and coupling before changing PPR.")
    elif usable and abs(result["count_scale_ratio"] - 1) > 0.02:
        warnings.append("Count scale differs by more than 2% from the expected PPR. A different programmed resolution, lost pulses, or mechanical motion/coupling can cause this; this is not an automatic PPR calibration.")
    return result


def display(value):
    return "n/a" if value is None else f"{value:.9g}" if isinstance(value, float) else str(value)


def format_summary(s):
    fields = (
        ("Measurement usable", "measurement_valid"), ("Physical turns entered", "physical_turns"),
        ("Expected PPR", "expected_ppr"), ("Samples", "samples"),
        ("Net counts", "net_counts"), ("Observed counts / physical rev", "observed_counts_per_rev"),
        ("Observed PPR equivalent (counts / 4)", "observed_ppr_equivalent"),
        ("Count scale ratio (expected = 1)", "count_scale_ratio"),
        ("Angle change [rad]", "angle_change_rad"), ("Angle change [deg]", "angle_change_deg"),
        ("Acquisition elapsed [s]", "elapsed_acquisition_s"),
        ("Mean reported velocity incl. holds [rad/s]", "mean_velocity_rad_s"),
        ("Peak abs. reported velocity [rad/s]", "peak_abs_velocity_rad_s"),
        ("Zero-velocity sample fraction", "zero_velocity_fraction"),
        ("Reverse counts", "reverse_counts"), ("Missing sequences", "missing_sequences"),
        ("Duplicate samples", "duplicate_samples"), ("Invalid count samples", "invalid_count_samples"),
        ("Median sample interval [ms]", "sample_interval_median_ms"),
        ("Max sample interval [ms]", "sample_interval_max_ms"),
    )
    lines = [f"{s['role']} marked-rotation report"]
    lines += [f"{label}: {display(s.get(key))}" for label, key in fields]
    if "diagnostic_deltas" in s:
        lines += ["Firmware counter changes: " + json.dumps(s["diagnostic_deltas"], sort_keys=True),
                  "Observed index intervals (events, signed counts): " + json.dumps(s["index_intervals"])]
    lines += ["PPR equivalent assumes the entered physical turns and correct x4 pulse capture.",
              "Separate hand turns need not have equal velocities; compare counts per revolution."]
    lines += ["NOTE: " + warning for warning in s["warnings"]]
    return "\n".join(lines)


def compare(paths):
    summaries = []
    for path in paths:
        path = Path(path)
        s = json.loads((path / "summary.json" if path.is_dir() else path).read_text(encoding="utf-8"))
        if s.get("format_version") != 1 or s.get("role") not in ROLES:
            raise ValueError(f"Not a marked-rotation summary: {path}")
        summaries.append(s)
    keys = ("measurement_valid", "physical_turns", "expected_ppr", "observed_counts_per_rev",
            "count_scale_ratio", "angle_change_deg", "peak_abs_velocity_rad_s", "reverse_counts",
            "missing_sequences", "invalid_count_samples")
    print(f"{'Metric':32} {summaries[0]['role']:>18} {summaries[1]['role']:>18}")
    for key in keys:
        print(f"{key:32} {display(summaries[0].get(key)):>18} {display(summaries[1].get(key)):>18}")
    a, b = summaries
    if all(s.get("measurement_valid") for s in summaries) and b.get("observed_counts_per_rev"):
        print(f"{a['role']}/{b['role']} counts-per-revolution ratio: "
              f"{a['observed_counts_per_rev'] / b['observed_counts_per_rev']:.9g}")
    else:
        print("No ratio: a run is incomplete/invalid or the second run has zero count travel.")
    print("Compare identical physical turns. Hand-rotation speeds may differ.")
    for s in summaries:
        for warning in s["warnings"]:
            print(f"{s['role']}: {warning}")


def request_enter(commands, prompt):
    def read():
        try:
            input(prompt)
            commands.put("enter")
        except EOFError:
            commands.put("eof")
    threading.Thread(target=read, daemon=True).start()


def record(args):
    # Offline comparisons and analysis tests work without ROS installed.
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from std_msgs.msg import String
    except ImportError as error:
        raise RuntimeError("Source your ROS 2 environment first; recording needs rclpy and std_msgs.") from error
    rclpy.init(args=[])
    node = rclpy.create_node("record_caster_rotation")
    recording = RotationRecording(args.role, args.turns, args.output, args.expected_ppr)
    commands = queue.Queue()
    qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
    subscription = node.create_subscription(String, args.topic, lambda msg: recording.ingest(msg.data), qos)
    print(f"Saving {args.role} to {recording.path}", flush=True)
    print(f"Waiting for valid {args.role} samples on {args.topic}. Keep the encoder still at your mark.", flush=True)
    started = last_flush = time.monotonic()
    prompted = False
    try:
        while rclpy.ok() and recording.end is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if recording.abort_reason:
                break
            if now - (recording.last_progress if recording.last_progress is not None else started) > args.timeout:
                recording.abort_reason = "Timed out waiting for progressing valid samples; check the bridge/topic and save a new complete run."
                break
            if not prompted and recording.latest is not None:
                request_enter(commands, "Hold the shaft still at the mark. Press ENTER to set START: ")
                prompted = True
            try:
                command = commands.get_nowait()
            except queue.Empty:
                command = None
            if command == "eof":
                recording.abort_reason = "Interactive stdin closed before both markers were set."
                break
            if command == "enter":
                try:
                    recording.mark(now)
                except ValueError as error:
                    print(str(error), flush=True)
                    request_enter(commands, "Press ENTER again after fresh data arrives: ")
                    continue
                recording.flush()
                if recording.end is None:
                    print(f"START count={recording.start['raw_count']}; angle={recording.start['angle_rad']} rad", flush=True)
                    request_enter(commands, f"Rotate the encoder shaft exactly {args.turns:g} full turn(s) in ONE direction relative to its housing.\nStop at the mark, hold still for 1 second, then press ENTER to FINISH: ")
            if now - last_flush >= 1:
                recording.flush()
                last_flush = now
    except KeyboardInterrupt:
        recording.abort_reason = "Interrupted before FINISH; recording preserved without a calibration."
    except Exception as error:
        recording.abort_reason = f"Recording failed: {error}"
    finally:
        if recording.end is None and not recording.abort_reason:
            recording.abort_reason = "ROS shut down before FINISH."
        summary, report = recording.close()
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("\n" + report)
    print(f"\nSaved: {recording.path}\nShare summary.json and samples.csv for diagnosis.")
    return 0 if summary["measurement_valid"] else 2


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    capture = sub.add_parser("record", help="interactively mark and record a physical rotation")
    capture.add_argument("role", choices=tuple(ROLES))
    capture.add_argument("--turns", type=positive, default=1.0, help="actual complete physical turns (default: 1)")
    capture.add_argument("--output", type=Path, default=Path("recordings/caster_rotations"))
    capture.add_argument("--topic", default="/caster_encoder/packets")
    capture.add_argument("--timeout", type=positive, default=10.0, help="seconds without new valid samples before aborting")
    capture.add_argument("--expected-ppr", type=positive, help="comparison reference; default firmware metadata, else assumed 2048")
    comparison = sub.add_parser("compare", help="compare two recording folders or summary.json files; ROS not required")
    comparison.add_argument("runs", type=Path, nargs=2)
    args = parser.parse_args(argv)
    try:
        if args.command == "compare":
            compare(args.runs)
            return 0
        return record(args)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"record_caster_rotation: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
