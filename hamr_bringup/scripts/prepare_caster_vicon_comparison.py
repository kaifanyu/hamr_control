#!/usr/bin/env python3
"""Align exported fork poses and encoder history for a later physical comparison.

This preparation is read-only with respect to the recording and analysis. It
uses the common recorded bag clock, preserves raw poses, and never treats fork
orientation as a measurement of CENTER or hemisphere spin. No ROS is required.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
import csv
from decimal import Decimal, ROUND_HALF_EVEN
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

WHEELS = ("front_left", "front_right")
ROLES = ("H1", "H2", "CENTER")
POSE_FILES = {"base": "HAMR_base_pose.csv", "front_left": "Left_Caster_pose.csv",
              "front_right": "Right_Caster_pose.csv"}
POSE_TOPICS = {"base": "/HAMR_base/pose", "front_left": "/Left_Caster/pose",
               "front_right": "/Right_Caster/pose"}
DEFAULT_QUALITY = dict(max_encoder_age_s=.025, max_base_age_s=.050,
                       max_pose_gap_s=.100, orientation_jump_deg=10., jump_guard_s=.100)
REQUIREMENTS = [
    "Resolve Vicon fork orientation ambiguity; smooth segments alone do not establish correct orientation.",
    "Measure marker-to-fork and marker-to-ball-center transforms, plus the base marker-to-robot transform.",
    "Validate cross-system timing, encoder PPR, axes, direction signs and physical phase references.",
    "Specify and validate a rolling/contact model to predict internal ball and hemisphere motion from fork travel.",
    "Fork marker orientation is not CENTER shaft angle or H1/H2 hemisphere rotation; no direct angular error is defined here.",
]


def elapsed_ns(value):
    """Recover integer relative nanoseconds without rounding a large epoch float."""
    number = Decimal(str(value))
    if not number.is_finite():
        raise ValueError("Nonfinite bag timestamp")
    return int((number * 1_000_000_000).to_integral_value(rounding=ROUND_HALF_EVEN))


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or norm < 1e-12:
        raise ValueError("Invalid orientation quaternion")
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion_step_deg(a, b):
    """Shortest actual 3D rotation: q and -q are equivalent."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    quaternion_matrix(a)
    quaternion_matrix(b)
    dot = abs(float(np.dot(a / np.linalg.norm(a), b / np.linalg.norm(b))))
    return math.degrees(2 * math.acos(min(1., dot)))


def relative_quaternion(base, fork):
    """Inverse(base) * fork; output is diagnostic marker-frame orientation."""
    base, fork = np.asarray(base, dtype=float), np.asarray(fork, dtype=float)
    base, fork = base / np.linalg.norm(base), fork / np.linalg.norm(fork)
    av, aw = -base[:3], base[3]
    bv, bw = fork[:3], fork[3]
    return np.r_[aw*bv + bw*av + np.cross(av, bv), aw*bw - np.dot(av, bv)]


def read_poses(path, quality):
    poses = []
    with Path(path).open(newline="") as stream:
        for source in csv.DictReader(stream):
            pose = {"time_ns": elapsed_ns(source["bag_time_s"]),
                    "header_time_ns": elapsed_ns(source["header_time_s"]),
                    "p": [float(source[f"{axis}_m"]) for axis in "xyz"],
                    "q": [float(source[f"q{axis}"]) for axis in "xyzw"]}
            pose["position_valid"] = all(math.isfinite(v) for v in pose["p"])
            try:
                quaternion_matrix(pose["q"])
                pose["orientation_valid"] = True
            except ValueError:
                pose["orientation_valid"] = False
            poses.append(pose)
    poses.sort(key=lambda row: row["time_ns"])
    if not poses or any(b["time_ns"] <= a["time_ns"] for a, b in zip(poses, poses[1:])):
        raise ValueError(f"Pose timestamps must be nonempty and strictly increasing: {path}")
    segment, jumps = 0, []
    for index, pose in enumerate(poses):
        previous = poses[index-1] if index else None
        gap = (pose["time_ns"] - previous["time_ns"]) / 1e9 if previous else None
        step = (quaternion_step_deg(previous["q"], pose["q"])
                if previous and previous["orientation_valid"] and pose["orientation_valid"] else None)
        jump = step is not None and step > quality["orientation_jump_deg"]
        discontinuity = bool(previous and (jump or gap > quality["max_pose_gap_s"]
                                          or not previous["orientation_valid"] or not pose["orientation_valid"]))
        segment += int(discontinuity)
        pose.update(gap_s=gap, rotation_step_deg=step, orientation_jump=jump,
                    segment=segment, gap_ok=gap is not None and 0 < gap <= quality["max_pose_gap_s"])
        if jump:
            jumps.append(pose["time_ns"])
    guard = elapsed_ns(quality["jump_guard_s"])
    for pose in poses:
        index = bisect_right(jumps, pose["time_ns"])
        near = any(abs(pose["time_ns"] - stamp) <= guard for stamp in jumps[max(0, index-1):index+1])
        pose["jump_neighborhood"] = near
        pose["orientation_continuity_ok"] = pose["orientation_valid"] and pose["gap_ok"] and not near
    return poses


def read_encoders(path):
    streams = {(wheel, role): [] for wheel in WHEELS for role in ROLES}
    with Path(path).open(newline="") as stream:
        for row in csv.DictReader(stream):
            parts = row["topic"].strip("/").split("/")
            if len(parts) != 4 or parts[0] != "caster_recording" or parts[3] != "joint_states":
                raise ValueError(f"Unexpected encoder topic: {row['topic']}")
            key = parts[1], parts[2]
            if key not in streams:
                raise ValueError(f"Unknown encoder identity: {row['topic']}")
            streams[key].append({"time_ns": elapsed_ns(row["bag_time_s"]),
                                 "angle_rad": float(row["recorded_angle_rad"]),
                                 "change_rad": float(row["unwrapped_change_from_first_sample_rad"]),
                                 "velocity_rad_s": float(row["reported_velocity_rad_s"])})
    for key, rows in streams.items():
        rows.sort(key=lambda row: row["time_ns"])
        if not rows or any(b["time_ns"] <= a["time_ns"] for a, b in zip(rows, rows[1:])):
            raise ValueError(f"Encoder timestamps must be nonempty and strictly increasing: {key}")
    return streams


class PastSamples:
    def __init__(self, rows):
        self.rows = rows
        self.times = [row["time_ns"] for row in rows]

    def at(self, stamp):
        index = bisect_right(self.times, stamp) - 1
        return self.rows[index] if index >= 0 else None


def align_wheel(wheel, forks, base, streams, quality, start_ns, end_ns, epoch=None):
    base_lookup = PastSamples(base)
    encoders = {role: PastSamples(streams[wheel, role]) for role in ROLES}
    output = []
    for fork in forks:
        stamp = fork["time_ns"]
        if not start_ns <= stamp <= end_ns:
            continue
        body = base_lookup.at(stamp)
        age = (stamp - body["time_ns"]) / 1e9 if body else None
        body_fresh = body is not None and age <= quality["max_base_age_s"]
        row = dict(wheel_id=wheel, bag_elapsed_ns=stamp, bag_time_s=stamp/1e9,
                   epoch_ns=epoch+stamp if epoch is not None else "",
                   fork_header_elapsed_ns=fork["header_time_ns"],
                   fork_gap_s=fork["gap_s"], fork_rotation_step_deg=fork["rotation_step_deg"],
                   fork_orientation_jump=fork["orientation_jump"],
                   fork_jump_neighborhood=fork["jump_neighborhood"], fork_segment=fork["segment"],
                   fork_orientation_continuity_ok=fork["orientation_continuity_ok"],
                   base_elapsed_ns=body["time_ns"] if body else None, base_age_s=age,
                   base_fresh=body_fresh, base_segment=body["segment"] if body else None,
                   base_orientation_continuity_ok=body["orientation_continuity_ok"] if body else False,
                   position_pair_usable=bool(body_fresh and body["position_valid"] and fork["position_valid"]),
                   orientation_pair_continuous=bool(body_fresh and body["orientation_continuity_ok"]
                                                    and fork["orientation_continuity_ok"]),
                   physical_prediction_ready=False)
        for label, pose in (("fork", fork), ("base", body)):
            for axis, value in zip("xyz", pose["p"] if pose else [None]*3):
                row[f"{label}_{axis}_m"] = value
            for axis, value in zip("xyzw", pose["q"] if pose else [None]*4):
                row[f"{label}_q{axis}"] = value
        # A raw relative quaternion is diagnostic only; always retain its masks.
        relative_p, relative_q = [None]*3, [None]*4
        if body_fresh and body["orientation_valid"] and fork["orientation_valid"]:
            relative_q = relative_quaternion(body["q"], fork["q"])
            if row["position_pair_usable"]:
                relative_p = quaternion_matrix(body["q"]).T @ (np.asarray(fork["p"])-body["p"])
        for axis, value in zip("xyz", relative_p):
            row[f"fork_marker_in_base_marker_{axis}_m"] = value
        for axis, value in zip("xyzw", relative_q):
            row[f"fork_relative_q{axis}"] = value
        all_fresh = True
        for role in ROLES:
            sample = encoders[role].at(stamp)
            sample_age = (stamp-sample["time_ns"])/1e9 if sample else None
            usable = (sample is not None and sample_age <= quality["max_encoder_age_s"]
                      and math.isfinite(sample["angle_rad"]) and math.isfinite(sample["change_rad"]))
            all_fresh = all_fresh and usable
            prefix = role.lower()
            row.update({f"{prefix}_elapsed_ns": sample["time_ns"] if sample else None,
                        f"{prefix}_age_s": sample_age, f"{prefix}_fresh": usable,
                        f"{prefix}_recorded_angle_rad": sample["angle_rad"] if sample else None,
                        f"{prefix}_unwrapped_change_rad": sample["change_rad"] if sample else None,
                        f"{prefix}_reported_velocity_rad_s": sample["velocity_rad_s"] if sample else None})
        row["all_encoders_fresh"] = all_fresh
        output.append(row)
    return output


def plot_preparation(output, aligned):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), layout="constrained")
    for column, wheel in enumerate(WHEELS):
        rows = aligned[wheel]
        t = np.array([row["bag_time_s"] for row in rows])
        axes[0, column].plot([row["fork_x_m"] for row in rows], [row["fork_y_m"] for row in rows], lw=1)
        axes[0, column].set(xlabel="Recorded marker x (m)", ylabel="Recorded marker y (m)",
                            title=wheel.replace("_", " ").title())
        axes[0, column].set_aspect("equal", adjustable="datalim")
        for role in ROLES:
            axes[1, column].plot(t, [row[f"{role.lower()}_unwrapped_change_rad"] for row in rows], label=role, lw=1)
        axes[1, column].set(ylabel="Recorded change from first sample (rad)", xlabel="Bag elapsed time (s)")
        axes[1, column].legend()
        axes[2, column].plot(t, [row["fork_rotation_step_deg"] for row in rows], label="Fork rotation step", lw=.8)
        bad = np.array([not row["orientation_pair_continuous"] for row in rows])
        axes[2, column].fill_between(t, 0, 180, where=bad, alpha=.18, color="red", label="Continuity/age mask")
        axes[2, column].set(ylabel="Adjacent 3D rotation (deg)", xlabel="Bag elapsed time (s)", ylim=(-2, 183))
        axes[2, column].legend()
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.suptitle("Vicon fork / encoder comparison preparation\nAligned recorded histories; no physical prediction or angular error", fontsize=15)
    fig.savefig(output / "comparison_preparation.png", dpi=150)
    plt.close(fig)


def verify_artifact_manifest(source, summary, manifest, filenames):
    """Prevent archived files from different runs being silently combined."""
    source_hash = manifest.get("source_mcap", {}).get("sha256")
    if not source_hash or source_hash != summary.get("sha256"):
        raise ValueError("Artifact manifest and analysis summary identify different source MCAP hashes")
    entries = manifest.get("files", [])
    expected = {entry["path"]: entry.get("sha256") for entry in entries}
    if len(expected) != len(entries):
        raise ValueError("Artifact manifest has duplicate file entries")
    for name in filenames:
        digest = hashlib.sha256((source / name).read_bytes()).hexdigest()
        if expected.get(name) != digest:
            raise ValueError(f"Artifact manifest hash mismatch or missing entry: {name}")


def prepare(analysis_dir, output_dir, config=None, make_plot=True):
    source, output = Path(analysis_dir).expanduser().resolve(), Path(output_dir).expanduser().resolve()
    if output == source or source in output.parents:
        raise ValueError("Choose an output directory outside the original analysis exports")
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError(f"Output directory is not empty: {output}")
    settings = yaml.safe_load(Path(config).expanduser().read_text()) if config else {}
    if not isinstance(settings, dict) or not isinstance(settings.get("quality", {}), dict):
        raise ValueError("Preparation configuration must contain mapping fields")
    quality = {**DEFAULT_QUALITY, **settings.get("quality", {})}
    if set(quality) != set(DEFAULT_QUALITY):
        raise ValueError("Unknown quality configuration field")
    if any(not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v) or v <= 0 for v in quality.values()):
        raise ValueError("All quality thresholds must be positive finite numbers")
    summary = json.loads((source / "summary.json").read_text())
    frame_names = [summary.get("frames", {}).get(topic, []) for topic in POSE_TOPICS.values()]
    if any(len(names) != 1 for names in frame_names) or len({names[0] for names in frame_names}) != 1:
        raise ValueError("Base and both fork pose exports must use one common recorded frame")
    epoch = summary.get("start_epoch_ns")
    manifest_path = source / "artifact_manifest.json"
    filenames = ["summary.json", "encoder_signals.csv", *POSE_FILES.values()]
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text())
        verify_artifact_manifest(source, summary, manifest, filenames)
        if epoch is not None and manifest.get("start_epoch_ns") != epoch:
            raise ValueError("Artifact manifest and analysis summary have different epoch origins")
        if epoch is None:
            epoch = manifest.get("start_epoch_ns")
    if epoch is not None and (type(epoch) is not int or epoch < 1):
        raise ValueError("start_epoch_ns must be a positive exact integer")
    poses = {label: read_poses(source / name, quality) for label, name in POSE_FILES.items()}
    streams = read_encoders(source / "encoder_signals.csv")
    start = max(rows[0]["time_ns"] for rows in streams.values())
    end = min(rows[-1]["time_ns"] for rows in streams.values())
    if end <= start:
        raise ValueError("All six encoder streams have no shared interval")
    aligned = {wheel: align_wheel(wheel, poses[wheel], poses["base"], streams, quality, start, end, epoch)
               for wheel in WHEELS}
    if any(not rows for rows in aligned.values()):
        raise ValueError("Both fork pose streams must overlap the shared encoder interval")
    if manifest_path.is_file():
        filenames.append(manifest_path.name)
    report = {"version": 1, "source_analysis": str(source), "source_bag_sha256": summary.get("sha256"),
              "artifact_manifest_verified": manifest_path.is_file(),
              "preparation_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "config_path": str(Path(config).expanduser().resolve()) if config else None,
              "config_sha256": hashlib.sha256(Path(config).expanduser().read_bytes()).hexdigest() if config else None,
              "source_files_sha256": {name: hashlib.sha256((source/name).read_bytes()).hexdigest() for name in filenames},
              "start_epoch_ns": epoch, "coordinate_frame": frame_names[0][0],
              "common_encoder_interval_elapsed_ns": [start, end], "quality_thresholds": quality,
              "time_basis": "Recorded bag time; encoder acquisition clocks estimated. No time shift or fitted lag applied.",
              "source_clock_mapping": summary.get("clock_mapping", {}),
              "encoder_quality_policy": "Aligned published joint-state exports retain original angles. Sequence/firmware diagnostics are not inferred from these CSVs; consult the source export quality below.",
              "source_encoder_export_quality": summary.get("encoder_export_quality", {}),
              "alignment": "Each native fork timestamp uses the latest preceding base and same-wheel encoder samples. No interpolation or future samples.",
              "orientation_policy": "Raw quaternions retained. q/-q invariant jump detection; jump neighborhoods and gaps masked. Smooth segments may still have an ambiguous orientation.",
              "physical_prediction_ready": False, "required_before_physical_comparison": REQUIREMENTS,
              "future_calibration": settings.get("future_calibration", {"status": "unconfigured"}), "wheels": {}}
    output.mkdir(parents=True, exist_ok=True)
    for wheel, rows in aligned.items():
        path = output / f"{wheel}_aligned.csv"
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        report["wheels"][wheel] = {"rows": len(rows), "file": path.name,
                                   "fork_jump_count": sum(row["fork_orientation_jump"] for row in rows),
                                   "orientation_pair_continuous_rows": sum(row["orientation_pair_continuous"] for row in rows),
                                   "all_encoders_fresh_rows": sum(row["all_encoders_fresh"] for row in rows),
                                   "position_pair_usable_rows": sum(row["position_pair_usable"] for row in rows)}
    if make_plot:
        plot_preparation(output, aligned)
    (output / "comparison_setup.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("analysis_dir", type=Path, help="Directory containing the combined-bag analysis exports")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, help="Preparation quality thresholds and future calibration template")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    try:
        report = prepare(args.analysis_dir, args.output_dir, args.config, not args.no_plot)
    except (ValueError, OSError, KeyError, TypeError) as error:
        parser.exit(2, f"prepare_caster_vicon_comparison: {error}\n")
    print(json.dumps(report["wheels"], indent=2))
    print(f"Prepared comparison inputs: {args.output_dir.resolve()}")
    print("Physical prediction remains unconfigured; inspect comparison_setup.json requirements.")


if __name__ == "__main__":
    main()
