"""Unified numeric exports and plots for all available HAMR/caster telemetry."""
import csv
import json
import math
from pathlib import Path
import re

import numpy as np
from scipy.spatial.transform import Rotation

from caster_bag_reader import SUBJECTS, WHEELS, digest
from caster_encoder_protocol import ROLES
import caster_packet_analysis as packet_analysis


def safe_json(value):
    if isinstance(value, dict):
        return {str(k): safe_json(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe_json(v) for v in value]
    if isinstance(value, np.generic):
        return safe_json(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(safe_json(value), indent=2, allow_nan=False)+"\n")


def write_csv(path, rows, fields=None):
    rows = list(rows)
    fields = fields or list(dict.fromkeys(key for row in rows for key in row))
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def slug(topic):
    import hashlib
    return re.sub(r"[^a-zA-Z0-9_-]", "_", topic.strip("/")) + "_" + hashlib.sha256(topic.encode()).hexdigest()[:8]


def finite_values(rows, key):
    return np.array([float(row[key]) if isinstance(row.get(key), (int, float)) else np.nan for row in rows])


def quaternion_steps_deg(q):
    q = np.asarray(q, dtype=float)
    norms = np.linalg.norm(q, axis=1)
    if not np.all(np.isfinite(q)) or np.any(norms < 1e-8):
        raise ValueError("Quaternion orientation is undefined or nonfinite")
    q = q / norms[:, None]
    return np.degrees(2*np.arccos(np.clip(np.abs(np.sum(q[1:]*q[:-1], axis=1)), 0, 1)))


def segmented_change(rows, role, max_gap_s=.025):
    """Unwrap only continuous, finite runs; retain blanks and reset after gaps."""
    delta, segment = [], -1
    previous, total = None, 0.
    for row in rows:
        angle = row.get("angle_rad")
        valid = isinstance(angle, (int, float)) and math.isfinite(angle)
        fresh = (valid and previous is not None
                 and 0 < row["stamp_ns"]-previous["stamp_ns"] <= round(max_gap_s*1e9)
                 and row.get("segment") == previous.get("segment"))
        if not valid:
            delta.append((None, None))
            previous = None
            continue
        if not fresh:
            segment += 1
            total = 0.
        else:
            step = angle-previous["angle_rad"]
            total += math.atan2(math.sin(step), math.cos(step)) if role == "CENTER" else step
        delta.append((total, segment))
        previous = row
    return delta


def packet_reports(recording, output, plots=True, wheel=None, reference=None, mapper=None,
                   max_match_dt_s=.02, max_continuity_gap_s=.1):
    result = {}
    for identity, packets in recording.packet_streams().items():
        if wheel and wheel != "both" and identity != wheel:
            continue
        target = output / "encoder_packets" / identity
        target.mkdir(parents=True)
        rows, metadata, summary = packet_analysis.analyze(packets, reference=reference,
                                                        clock_mapper=mapper, wheel_id=identity,
                                                        max_match_dt_s=max_match_dt_s,
                                                        max_continuity_gap_s=max_continuity_gap_s)
        write_csv(target / "samples.csv", rows, packet_analysis.CSV_FIELDS)
        for filename, values in (("packets.jsonl", [entry for entry, _ in packets]), ("metadata.jsonl", metadata)):
            with (target / filename).open("w") as stream:
                for entry in values:
                    stream.write(json.dumps(entry, allow_nan=False, separators=(",", ":"))+"\n")
        summary["plots"] = packet_analysis.plot_report(rows, target) if plots else []
        write_json(target / "summary.json", summary)
        result[identity] = summary
        # Packet-only bags/local logs also yield source-angle plots. Never
        # replace an existing published JointState stream with derived values.
        existing = recording.encoder_topics()
        for node, role in ROLES.items():
            if (identity, role) in existing or identity not in WHEELS:
                continue
            field = "calibrated_alpha_rad" if node == 3 else "angle_rad"
            topic = f"/caster_recording/{identity}/{role}/joint_states"
            for row in rows:
                stamp = row["mapped_epoch_ns"] or row["bag_timestamp_ns"] or row["receive_epoch_ns"]
                if row["node_id"] != node or not stamp or row["sequence_status"] == "duplicate":
                    continue
                recording.add_numeric(topic, stamp, dict(angle_rad=row[field] if row["angle_eligible"] else None,
                                                        velocity_rad_s=row["velocity_rad_s"] if row["velocity_eligible"] else None,
                                                        segment=row["segment_id"]))
            if topic in recording.numeric:
                recording.numeric[topic].sort(key=lambda row: row["stamp_ns"])
    return result


def export_report(recording, output, plots=True, **packet_options):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    packet_quality = packet_reports(recording, output, plots, **packet_options)
    inventory = []
    for topic, kind in sorted(recording.types.items()):
        stamps = recording.times.get(topic, [])
        known_stamps = [stamp for stamp in stamps if stamp > 0]
        dt = np.diff(known_stamps) / 1e6
        inventory.append(dict(topic=topic, type=kind, count=len(stamps),
                              start_s=(min(known_stamps)-recording.start_ns)/1e9 if known_stamps else None,
                              end_s=(max(known_stamps)-recording.start_ns)/1e9 if known_stamps else None,
                              unknown_timestamps=len(stamps)-len(known_stamps),
                              median_interval_ms=float(np.median(dt)) if len(dt) else None,
                              max_interval_ms=float(np.max(dt)) if len(dt) else None,
                              nonpositive_intervals=int(np.sum(dt <= 0)),
                              numeric_export=topic in recording.numeric))
    write_csv(output / "topics.csv", inventory)
    files = recording.files or [recording.root / "packets.jsonl"]
    report = dict(version=3, source=str(recording.source),
                  source_sha256={str(path): digest(path) for path in files},
                  sha256=digest(files[0]) if len(files) == 1 else None,
                  start_epoch_ns=recording.start_ns,
                  duration_s=(recording.end_ns-recording.start_ns)/1e9,
                  total_messages=sum(len(times) for times in recording.times.values()),
                  metadata_counts_verified=recording.counts_verified,
                  frames={topic: sorted(frames) for topic, frames in recording.frames.items()},
                  topics=inventory, decode_issues=recording.issues,
                  encoder_export_quality=recording.sidecars.get("export-summary.json", {}),
                  clock_mapping=recording.sidecars.get("clock-map.json", {}),
                  sidecar_sha256={name: digest(recording.root/name) for name in recording.sidecars},
                  packet_quality=packet_quality, poses={}, encoders={}, plots=[],
                  notes=["Recorded bag time is the shared plot clock; embedded acquisition maps are used for local packet replay.",
                         "Uncalibrated fork-marker motion is distinct from CENTER carrier rotation and H1/H2 shell spin.",
                         "No clock shift, marker transform, rolling model, or fitted encoder accuracy is inferred.",
                         "Missing/unsupported topics remain listed in topics.csv; source MCAP bytes are unchanged."])
    numeric_dir = output / "topics"
    numeric_dir.mkdir()
    for topic, rows in recording.numeric.items():
        write_csv(numeric_dir / f"{slug(topic)}.csv", [dict(
            bag_time_s=f"{(row['stamp_ns']-recording.start_ns)/1e9:.9f}", **row) for row in rows])
    for subject in SUBJECTS:
        topic = next((f"/{subject}/{kind}" for kind in ("pose", "odom") if recording.poses.get(f"/{subject}/{kind}")), None)
        if topic is None:
            continue
        rows = recording.poses[topic]
        exported = [dict(bag_time_s=f"{(row['stamp_ns']-recording.start_ns)/1e9:.9f}",
                         header_time_s=f"{(row['header_stamp_ns']-recording.start_ns)/1e9:.9f}",
                         **{f"{axis}_m": row[axis] for axis in "xyz"},
                         **{f"q{axis}": row[f"q{axis}"] for axis in "xyzw"}) for row in rows]
        write_csv(output / f"{subject}_pose.csv", exported)
        report["frames"][f"/{subject}/pose"] = sorted(recording.frames[topic])
        q = np.array([[row[f"q{axis}"] for axis in "xyzw"] for row in rows])
        valid = np.all(np.isfinite(q), axis=1) & (np.linalg.norm(q, axis=1) > 1e-8)
        jumps = np.full(max(0, len(rows)-1), np.nan)
        for i in range(1, len(rows)):
            if valid[i-1] and valid[i]:
                jumps[i-1] = quaternion_steps_deg(q[i-1:i+1])[0]
        report["poses"][subject] = dict(source_topic=topic, samples=len(rows), invalid_quaternions=int(np.sum(~valid)),
                                        orientation_steps_above_10deg=int(np.sum(jumps > 10)))
    encoder_rows = []
    for (wheel, role), topic in recording.encoder_topics().items():
        rows = recording.numeric[topic]
        changes = segmented_change(rows, role)
        for row, (change, segment) in zip(rows, changes):
            encoder_rows.append(dict(topic=f"/caster_recording/{wheel}/{role}/joint_states",
                                     bag_time_s=f"{(row['stamp_ns']-recording.start_ns)/1e9:.9f}",
                                     recorded_angle_rad=row.get("angle_rad"),
                                     unwrapped_change_from_first_sample_rad=change,
                                     reported_velocity_rad_s=row.get("velocity_rad_s"), segment=segment))
        report["encoders"][f"{wheel}/{role}"] = dict(source_topic=topic, samples=len(rows),
                                                    segments=max((s for _, s in changes if s is not None), default=-1)+1)
    if encoder_rows:
        write_csv(output / "encoder_signals.csv", encoder_rows)
    write_json(output / "summary.json", report)
    if plots:
        plot_report(recording, output, report)
        write_json(output / "summary.json", report)
    return report


def plot_report(recording, output, report):
    from contextlib import nullcontext
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    plt.rcParams.update({"font.size": 9, "lines.linewidth": 1., "pdf.fonttype": 42})
    def save(fig, name, pdf):
        for axis in fig.axes:
            axis.grid(alpha=.2)
        fig.savefig(output / f"{name}.png", dpi=140)
        if pdf is not None:
            pdf.savefig(fig)
        plt.close(fig)
        report["plots"].append(f"{name}.png")

    def times(rows):
        return np.array([(row["stamp_ns"]-recording.start_ns)/1e9 for row in rows])

    def gapped(rows, field, wrap=False):
        t, values = times(rows), finite_values(rows, field)
        broken = np.r_[False, (np.diff(t) <= 0) | (np.diff(t) > .1)]
        broken |= np.array([False] + [current.get("segment") != previous.get("segment")
                                      for previous, current in zip(rows, rows[1:])])
        if wrap:
            broken |= np.r_[False, np.abs(np.diff(values)) > math.pi]
        # Insert breaks without discarding the first observed value after a gap.
        return (np.insert(t, np.flatnonzero(broken), np.nan),
                np.insert(values, np.flatnonzero(broken), np.nan))

    with nullcontext(None) as pdf:
        fig, axes = plt.subplots(1, 2, figsize=(14, 6), layout="constrained")
        for topic, rows in recording.poses.items():
            if topic.endswith("/odom") and recording.poses.get(topic.removesuffix("/odom")+"/pose"):
                continue
            frames = recording.frames[topic]
            label = f"{topic} [{', '.join(sorted(frames))}]"
            if len(frames) != 1:
                continue
            # Separate raw coordinate frames; never align independent origins silently.
            x, y = finite_values(rows, "x"), finite_values(rows, "y")
            jumps = np.r_[False, (np.diff(times(rows)) > .1) | (np.linalg.norm(np.diff(np.c_[x, y], axis=0), axis=1) > .08)]
            x = np.insert(x, np.flatnonzero(jumps), np.nan)
            y = np.insert(y, np.flatnonzero(jumps), np.nan)
            axes[0].plot(x, y, label=label)
            axes[1].plot(*gapped(rows, "z"), label=label)
        for topic, path in recording.paths.items():
            if path["points"]:
                points = np.asarray(path["points"])
                axes[0].plot(points[:, 0], points[:, 1], "k--", label=f"{topic} [{path['frame_id']}]")
        reference = recording.numeric.get("/reference_trajectory", [])
        if reference:
            axes[0].plot(finite_values(reference, "x"), finite_values(reference, "y"), ":", label="Reference [frame unspecified]")
        axes[0].set(xlabel="Recorded x (m)", ylabel="Recorded y (m)", title="Paths in recorded coordinates; no frame alignment")
        axes[0].set_aspect("equal", adjustable="datalim")
        axes[1].set(xlabel="Bag time (s)", ylabel="Recorded z (m)", title="Pose height")
        for ax in axes:
            if ax.lines:
                ax.legend(fontsize=6)
            else:
                ax.text(.5, .5, "No recorded pose/path data", transform=ax.transAxes, ha="center")
        fig.suptitle("Robot, turret and fork trajectories")
        save(fig, "01_trajectories", pdf)

        for field, name, ylabel in (("angle_rad", "02_encoder_angles", "Source angle (rad)"),
                                    ("velocity_rad_s", "03_encoder_rates", "Reported rate (rad/s)")):
            fig, axes = plt.subplots(3, 2, figsize=(14, 9), layout="constrained", sharex=True)
            for column, wheel in enumerate(WHEELS):
                for row, role in enumerate(ROLES.values()):
                    ax = axes[row, column]
                    topic = recording.encoder_topics().get((wheel, role))
                    if topic:
                        ax.plot(*gapped(recording.numeric[topic], field, wrap=role == "CENTER" and field == "angle_rad"))
                    else:
                        ax.text(.5, .5, "Not recorded", transform=ax.transAxes, ha="center")
                    ax.set(title=f"{wheel} / {role}", ylabel=ylabel, xlabel="Bag time (s)")
            fig.suptitle("Caster encoder history — source signs and phases retained")
            save(fig, name, pdf)

        pose_topics = [f"/{subject}/{kind}" for subject in SUBJECTS for kind in ("pose", "odom")
                       if recording.poses.get(f"/{subject}/{kind}")
                       and (kind == "pose" or not recording.poses.get(f"/{subject}/pose"))]
        if pose_topics:
            fig, axes = plt.subplots(len(pose_topics), 2, figsize=(14, 3*len(pose_topics)), layout="constrained", squeeze=False)
            for index, topic in enumerate(pose_topics):
                rows = recording.poses[topic]
                q = np.array([[row[f"q{axis}"] for axis in "xyzw"] for row in rows])
                valid = np.all(np.isfinite(q), axis=1) & (np.linalg.norm(q, axis=1) > 1e-8)
                euler = np.full((len(rows), 3), np.nan)
                import warnings
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    if np.any(valid):
                        euler[valid] = Rotation.from_quat(q[valid]).as_euler("xyz", degrees=True)
                for j, label in enumerate(("roll", "pitch", "yaw")):
                    axes[index, 0].plot(times(rows), euler[:, j], label=label)
                jumps = [quaternion_steps_deg(q[i-1:i+1])[0] if valid[i-1] and valid[i] else np.nan for i in range(1, len(rows))]
                axes[index, 1].plot(times(rows)[1:], jumps)
                axes[index, 1].axhline(10, color="red", ls="--")
                axes[index, 0].set(title=topic, ylabel="Euler angle (degrees)", xlabel="Bag time (s)")
                axes[index, 0].legend()
                axes[index, 1].set(title="Shortest 3D orientation step (q/-q invariant)", ylabel="Degrees", xlabel="Bag time (s)")
            save(fig, "04_vicon_orientation_quality", pdf)

        # Every decoded numeric channel gets its own topic plot(s), so drive
        # commands, ticks, PWM/PI terms, gains, IMU, magnetometer, calibration,
        # odometry twist and reference velocities cannot disappear from a fixed
        # seven-page template. CSV exports retain exact integer counters.
        for topic, rows in sorted(recording.numeric.items()):
            if topic.endswith("/joint_states") or topic.endswith("/pose"):
                continue
            fields = [key for key in dict.fromkeys(k for row in rows for k in row)
                      if key not in ("stamp_ns", "header_stamp_ns", "segment")]
            for page, start in enumerate(range(0, len(fields), 6)):
                selected = fields[start:start+6]
                fig, axes = plt.subplots(len(selected), 1, figsize=(13, max(3, 1.8*len(selected))), layout="constrained", squeeze=False, sharex=True)
                for ax, field in zip(axes.flat, selected):
                    ax.plot(*gapped(rows, field))
                    ax.set(ylabel=field)
                    ax.ticklabel_format(axis="y", style="plain", useOffset=False)
                axes[-1, 0].set_xlabel("Bag elapsed time (s)")
                fig.suptitle(topic)
                save(fig, f"topic_{slug(topic)}_{page+1:02d}", pdf)

        fig, axes = plt.subplots(2, 1, figsize=(14, 9), layout="constrained")
        inventory = report["topics"]
        for index, item in enumerate(inventory):
            if item["start_s"] is not None:
                axes[0].plot([item["start_s"], item["end_s"]], [index, index], lw=3)
        axes[0].set_yticks(range(len(inventory)), [item["topic"] for item in inventory], fontsize=5)
        axes[0].set(xlabel="Bag time (s)", title="Coverage of every recorded topic")
        for topic in pose_topics:
            rows = recording.poses[topic]
            delay = [(row["stamp_ns"]-row["header_stamp_ns"])/1e6 for row in rows]
            axes[1].plot(times(rows), delay, label=topic)
        axes[1].set(xlabel="Bag time (s)", ylabel="Bag minus header (ms)", title="Timestamp difference; not calibrated capture latency")
        if axes[1].lines:
            axes[1].legend(fontsize=7)
        save(fig, "05_timing_and_coverage", pdf)
