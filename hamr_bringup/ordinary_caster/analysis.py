"""Independent ordinary swivel/wheel quality and trajectory analysis."""

import argparse
import csv
import json
from pathlib import Path

from . import protocol
from .history import add_history_arguments, history_from_args


def summarize(history):
    streams = []
    for stream in history.streams:
        points = stream.observations
        flag_counts = {name: sum(bool(point.flags & (1 << bit)) for point in points)
                       for bit, name in enumerate(protocol.FLAG_NAMES)}
        adjacent = [(a, b) for a, b in zip(points, points[1:]) if a.segment == b.segment and a.valid and b.valid]
        continuous = [(a, b) for a, b in adjacent if a.continuous and b.continuous]
        calibrated = [(a, b) for a, b in continuous if a.calibrated and b.calibrated]
        report = {
            "wheel_id": stream.wheel_id, "channel": stream.channel,
            "device_id": stream.identity[1], "node_id": stream.identity[2], "boot_id": stream.identity[3],
            "sample_count": len(points), "valid_samples": sum(point.valid for point in points),
            "calibrated_samples": sum(point.valid and point.calibrated for point in points),
            "continuous_samples": sum(point.valid and point.continuous for point in points),
            "segments": len({point.segment for point in points if point.valid}),
            "first_t_us": points[0].t_us, "last_t_us": points[-1].t_us,
            "duration_s": (points[-1].t_us - points[0].t_us) / 1e6,
            "sequence_missing": sum(max(0, b.seq - a.seq - 1) for a, b in zip(points, points[1:])),
            "time_gaps": sum(b.time_ns - a.time_ns > stream.gap_ns for a, b in zip(points, points[1:])),
            "flag_counts": flag_counts, "clock_problem": stream.clock_problem,
            "clock_mapping": stream.clock_mapping, "metadata": stream.metadata[stream.channel],
            "calibrated_travel_rad": sum(abs(b.position - a.position) for a, b in calibrated),
            "travel_note": "Sum only over adjacent valid continuous calibrated intervals; gaps and unknown turns are excluded.",
        }
        if stream.channel == "wheel":
            raw = [int(point.values["raw_count"]) for point in points if point.values["raw_count"] is not None]
            report.update(first_raw_count=str(raw[0]) if raw else None,
                          origin_raw_count=str(stream.origin_raw_count) if stream.origin_raw_count is not None else None,
                          last_raw_count=str(raw[-1]) if raw else None,
                          observed_count_span=str(max(raw) - min(raw)) if raw else None,
                          relative_phase=True, absolute_phase_available=False,
                          invalid_transition_detection=False, sensor_presence_detection=False)
            rates = [(int(b.values["raw_count"]) - int(a.values["raw_count"])) * 1e6 / (b.t_us - a.t_us)
                     for a, b in adjacent]
            report["maximum_observed_count_rate_per_s"] = max(map(abs, rates), default=None)
        else:
            report["wrapped_orientation_samples"] = sum(point.valid for point in points)
            report["turn_history_complete"] = all(point.valid and point.continuous for point in points) and len({p.segment for p in points}) == 1
        streams.append(report)
    return {"version": 1, "caster_type": "ordinary", "measurement_scope": "caster_joint_history",
            "vehicle_trajectory_measured": False, "source": str(history.source), "run_id": history.run_id,
            "time_basis": history.time_basis, "accuracy_validated": False,
            "combined_timing_usable": history.time_basis == "estimated_acquisition_epoch",
            "warnings": history.warnings, "streams": streams}


def write_history_csv(history, destination):
    names = ("wheel_id", "device_id", "node_id", "boot_id", "channel", "seq", "t_us", "history_time_ns",
             "mapped_epoch_ns", "mapping_usable", "timing_uncertainty_ns",
             "time_basis", "segment", "valid", "calibrated", "continuous", "relative_position_rad",
             *protocol.VALUE_FIELDS)
    with Path(destination).open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, names)
        writer.writeheader()
        for stream in history.streams:
            for point in stream.observations:
                writer.writerow({"wheel_id": stream.wheel_id, "device_id": stream.identity[1],
                                 "node_id": stream.identity[2], "boot_id": stream.identity[3], "channel": stream.channel,
                                 "seq": point.seq, "t_us": point.t_us, "history_time_ns": point.time_ns,
                                 "mapped_epoch_ns": point.mapped_epoch_ns,
                                 "mapping_usable": int(stream.clock_problem is None),
                                 "timing_uncertainty_ns": (stream.clock_mapping or {}).get("uncertainty_ns"),
                                 "time_basis": history.time_basis, "segment": point.segment,
                                 "valid": int(point.valid), "calibrated": int(point.calibrated),
                                 "continuous": int(point.continuous),
                                 "relative_position_rad": point.position if point.valid else None,
                                 **{name: point.values.get(name) for name in protocol.VALUE_FIELDS}})


def plot_history(history, destination):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    figure, axes = plt.subplots(len(history.streams), 3, figsize=(17, 3 * len(history.streams)), squeeze=False)
    for stream, row in zip(history.streams, axes):
        axis, speed_axis, valid_axis = row
        origin = history.start_ns if history.time_basis == "estimated_acquisition_epoch" else stream.times[0]
        grouped = {}
        for point in stream.observations:
            grouped.setdefault(point.segment, []).append(point)
        for group in grouped.values():
            points = [point for point in group if point.valid]
            if not points:
                continue
            x = [(point.time_ns - origin) / 1e9 for point in points]
            if stream.channel == "wheel" and not all(point.calibrated for point in points):
                # Do not silently label an assumed ring CPR as measured radians.
                baseline = next(int(p.values["raw_count"]) for p in stream.observations if p.valid)
                y = [int(point.values["raw_count"]) - baseline for point in points]
                axis.set_ylabel("Relative counts (CPR unverified)")
            else:
                y = [point.position for point in points]
                axis.set_ylabel("Rotation / orientation [rad]")
            wrapped_only = stream.channel == "swivel" and not all(point.continuous for point in points)
            axis.plot(x, y, marker=".", markersize=3 if wrapped_only else 2,
                      linestyle="None" if wrapped_only else "-")
            if wrapped_only:
                axis.set_ylabel("Wrapped orientation [rad]; turns unknown")
            if stream.channel == "wheel" and not all(point.calibrated for point in points):
                speeds = [(int(b.values["raw_count"]) - int(a.values["raw_count"])) * 1e6 / (b.t_us - a.t_us)
                          for a, b in zip(points, points[1:])]
                speed_axis.plot(x[1:], speeds, marker=".", markersize=2)
                speed_axis.set_ylabel("Adjacent raw count rate [count/s]")
            else:
                speeds = [p.values["velocity_rad_s"] if p.flags & protocol.VELOCITY_VALID else None for p in points]
                filtered = [p.values["filtered_velocity_rad_s"] if p.flags & protocol.VELOCITY_VALID else None for p in points]
                speed_axis.plot(x, speeds, color="#6597b5", linewidth=.7, label="measured rate")
                speed_axis.plot(x, filtered, color="#225578", linewidth=1, label="filtered rate")
                speed_axis.set_ylabel("Angular velocity [rad/s]")
        invalid = [(p.time_ns - origin) / 1e9 for p in stream.observations if not p.valid]
        axis.plot(invalid, [0] * len(invalid), "rx", transform=axis.get_xaxis_transform(), markersize=3)
        calibrated = all(p.calibrated for p in stream.observations if p.valid)
        axis.set_title(f"{stream.wheel_id} {stream.channel} — boot {stream.identity[3]}" +
                       ("" if calibrated else " — calibration unverified"))
        speed_axis.set_title("Velocity")
        x = [(p.time_ns - origin) / 1e9 for p in stream.observations]
        valid_axis.step(x, [int(p.valid) for p in stream.observations], where="post", label="valid")
        valid_axis.step(x, [int(p.continuous and p.valid) * .95 for p in stream.observations], where="post", label="continuous", linestyle="--")
        valid_axis.step(x, [int(p.calibrated and p.valid) * .9 for p in stream.observations], where="post", label="calibrated", linestyle=":")
        valid_axis.set_ylim(-.1, 1.15)
        valid_axis.set_yticks([0, 1])
        valid_axis.set_title("Independent channel eligibility")
        valid_axis.legend(loc="lower right", fontsize=7)
        for panel in row:
            panel.set_xlabel("Seconds from shared mapped start" if history.time_basis == "estimated_acquisition_epoch"
                             else "Seconds from this channel's first acquisition (device-local)", fontsize=8)
            panel.grid(alpha=.25)
    figure.suptitle("Ordinary caster joint measurements — gaps are not joined; vehicle trajectory is analyzed separately")
    figure.tight_layout()
    figure.savefig(destination, dpi=150)
    plt.close(figure)


def write_analysis(history, output, plots=True):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report = summarize(history)
    (output / "quality.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    write_history_csv(history, output / "history.csv")
    if plots:
        plot_history(history, output / "trajectory.png")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_history_arguments(parser)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)
    try:
        history = history_from_args(args)
        output = args.output or history.source.parent / "ordinary_analysis"
        report = write_analysis(history, output, not args.no_plots)
    except (ValueError, OSError, ImportError) as error:
        parser.error(str(error))
    print(json.dumps({"output": str(output), "time_basis": report["time_basis"],
                      "streams": [{key: stream[key] for key in ("wheel_id", "channel", "sample_count", "valid_samples")}
                                  for stream in report["streams"]]}, indent=2))
    return 0
