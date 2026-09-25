#!/usr/bin/env python3
"""Measure sampled tracking accuracy and corner overshoot in a Gazebo run.

Time-aligned error uses each recorded reference. Geometric error is the distance
to the nearest finite route segment, so a car lagging along the correct line can
have zero geometric error and nonzero tracking error. Both are required.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


DEFAULT_LIMITS = {
    "max_tracking_error_m": .010,
    "max_cross_track_m": .005,
    "max_corner_overshoot_m": .002,
    "max_final_error_m": .001,
}


def point_segment_distances(positions, points):
    """Return N distances to a finite polyline, including endpoint caps."""
    positions, points = np.asarray(positions, float), np.asarray(points, float)
    edges = np.diff(points, axis=0)
    lengths_squared = np.sum(edges * edges, axis=1)
    if np.any(lengths_squared <= 0):
        raise ValueError("Consecutive waypoints must differ")
    relative = positions[:, None, :] - points[None, :-1, :]
    fractions = np.clip(np.sum(relative * edges, axis=2) / lengths_squared, 0., 1.)
    closest = points[None, :-1, :] + fractions[:, :, None] * edges[None, :, :]
    return np.min(np.linalg.norm(positions[:, None, :] - closest, axis=2), axis=1)


def distribution(values):
    values = np.asarray(values, float)
    if not len(values):
        return {name: None for name in ("rms_m", "max_m", "p50_m", "p95_m", "p99_m")}
    return {"rms_m": float(np.sqrt(np.mean(values**2))), "max_m": float(np.max(values)),
            **{f"p{percentile}_m": float(np.percentile(values, percentile))
               for percentile in (50, 95, 99)}}


def sample_times(report):
    """Prefer pose-stamped trajectory time; legacy startup clamps it to zero."""
    startup = report.get("startup_hold_s", 0.)
    return np.array([sample["elapsed_s"]-startup if sample.get("phase") == "startup_hold"
                     else sample.get("trajectory_time_s", sample["elapsed_s"]-startup)
                     for sample in report["samples"]], float)


def schedules(report, points):
    """New runs report actual arrivals/departures, including endpoint holds."""
    supplied = report.get("reference_schedule_s")
    if supplied is None:
        speed = float(report.get("speed_m_s", report["configuration"]["speed_m_s"]))
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("A legacy report requires a finite positive speed")
        arrivals = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1) / speed)]
        source = "legacy distance / speed"
    else:
        arrivals = np.asarray(supplied, float)
        source = "recorded actual endpoint arrival times"
    departures = np.asarray(report.get("reference_departure_schedule_s", arrivals), float)
    if len(arrivals) != len(points) or len(departures) != len(points):
        raise ValueError("Arrival and departure schedules must have one entry per waypoint")
    if not np.all(np.isfinite(arrivals)) or not np.all(np.isfinite(departures)):
        raise ValueError("Arrival and departure schedules must be finite and complete")
    if np.any(np.diff(arrivals) <= 0) or np.any(departures < arrivals - 1e-9):
        raise ValueError("Schedules must increase and departure cannot precede arrival")
    if np.any(departures[:-1] >= arrivals[1:]):
        raise ValueError("Each departure must precede the following arrival")
    return arrivals, departures, source


def analyze(report, limits=None, corner_window_s=2.):
    """Return metrics and explicit acceptance; invalid/incomplete runs cannot pass.

    Incoming-plane overshoot is geometrically well-defined for turns of at least
    90 degrees. For shallower turns the intended outgoing path crosses that plane,
    so the metric is deliberately inapplicable; finite-route error still applies.
    Corner overshoot covers the entire outgoing leg, including its preceding
    endpoint dwell: slow reorientation can peak several seconds after departure.
    The short local window is reported separately and also retains approach-side
    overshoot. Outgoing-line cross-track is measured only after departure, since
    the incoming segment is legitimately far from that line before the corner.
    """
    limits = {**DEFAULT_LIMITS, **(limits or {})}
    if any(not math.isfinite(value) or value <= 0 for value in limits.values()):
        raise ValueError("Acceptance limits must be finite and positive")
    if not math.isfinite(corner_window_s) or corner_window_s <= 0:
        raise ValueError("Corner window must be finite and positive")
    points = np.asarray(report["configuration"]["points_m"], float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2 or not np.all(np.isfinite(points)):
        raise ValueError("Need at least two finite XY waypoints")
    arrivals, departures, schedule_source = schedules(report, points)
    samples = report.get("samples", [])
    if not samples:
        raise ValueError("No simulation feedback samples")
    times = sample_times(report)
    positions = np.array([sample["position_m"][:2] for sample in samples], float)
    references = np.array([sample["reference_m"][:2] for sample in samples], float)
    if not all(np.all(np.isfinite(values)) for values in (times, positions, references)):
        raise ValueError("Non-finite feedback, reference, or sample time")
    if np.any(np.diff(times) <= 0):
        raise ValueError("Sample elapsed times must be strictly increasing")
    duration = float(report.get("summary", {}).get("reference_duration_s", arrivals[-1]))
    if not math.isfinite(duration) or duration < arrivals[-1] - 1e-9:
        raise ValueError("Reference duration must include the final arrival")
    motion = (times >= 0.) & (times <= duration + 1e-9)
    if not np.any(motion):
        raise ValueError("No samples cover the motion interval")
    errors = np.linalg.norm(positions - references, axis=1)
    geometric = point_segment_distances(positions, points)
    waypoints = []
    corners = []
    for index, (point, arrival, departure) in enumerate(zip(points, arrivals, departures)):
        bracketed = bool(times[0] <= arrival <= times[-1])
        at_arrival = np.array([np.interp(arrival, times, positions[:, dim]) for dim in (0, 1)])
        at_departure = np.array([np.interp(departure, times, positions[:, dim]) for dim in (0, 1)])
        dwell = (times >= arrival) & (times <= departure)
        waypoint = {
            "index": index, "point_m": point.tolist(), "scheduled_arrival_s": float(arrival),
            "departure_s": float(departure), "hold_duration_s": float(departure-arrival),
            "arrival_is_bracketed_by_samples": bracketed,
            "interpolated_arrival_position_error_m": float(np.linalg.norm(at_arrival-point)),
            "interpolated_departure_position_error_m": float(np.linalg.norm(at_departure-point)),
            "hold_position_error": distribution(np.linalg.norm(positions[dwell]-point, axis=1)),
        }
        waypoints.append(waypoint)
        if index in (0, len(points)-1):
            continue
        incoming = point-points[index-1]
        outgoing = points[index+1]-point
        incoming /= np.linalg.norm(incoming)
        outgoing /= np.linalg.norm(outgoing)
        cosine = float(np.clip(incoming @ outgoing, -1., 1.))
        angle = math.acos(cosine)
        if angle < 1e-7:
            waypoint["turn_angle_deg"] = 0.
            waypoint["is_corner"] = False
            continue
        waypoint["turn_angle_deg"] = math.degrees(angle)
        waypoint["is_corner"] = True
        begin = max(arrival-corner_window_s, (departures[index-1]+arrival)/2.)
        end = min(departure+corner_window_s, (departure+arrivals[index+1])/2.)
        window = (times >= begin) & (times <= end)
        after = window & (times >= departure)
        rel = positions[window]-point
        local_overrun = max(0., float(np.max(rel @ incoming))) if len(rel) and cosine <= 1e-7 else None
        outgoing_leg = (times >= arrival) & (times <= arrivals[index+1])
        leg_relative = positions[outgoing_leg]-point
        leg_overrun = max(0., float(np.max(leg_relative @ incoming))) if len(leg_relative) and cosine <= 1e-7 else None
        measured_overruns = [value for value in (local_overrun, leg_overrun) if value is not None]
        overrun = max(measured_overruns) if measured_overruns else None
        outgoing_normal = np.array([-outgoing[1], outgoing[0]])
        after_cross = np.abs((positions[after]-point) @ outgoing_normal)
        entire_leg_after = outgoing_leg & (times >= departure)
        entire_leg_cross = np.abs((positions[entire_leg_after]-point) @ outgoing_normal)
        corner = {
            "waypoint_index": index, "point_m": point.tolist(), "turn_angle_deg": math.degrees(angle),
            "arrival_s": float(arrival), "departure_s": float(departure),
            "window_s": [float(begin), float(end)], "sample_count": int(np.sum(window)),
            "outgoing_leg_window_s": [float(arrival), float(arrivals[index+1])],
            "outgoing_leg_sample_count": int(np.sum(outgoing_leg)),
            "time_aligned_error": distribution(errors[window]),
            "nearest_route_error": distribution(geometric[window]),
            "outgoing_line_error_after_departure": distribution(after_cross),
            "entire_outgoing_leg_line_error_after_departure": distribution(entire_leg_cross),
            "incoming_direction_overshoot_m": overrun,
            "local_window_incoming_direction_overshoot_m": local_overrun,
            "entire_outgoing_leg_incoming_direction_overshoot_m": leg_overrun,
            "overshoot_applicable": cosine <= 1e-7,
        }
        corners.append(corner)
    overshoots = [corner["incoming_direction_overshoot_m"] for corner in corners
                  if corner["incoming_direction_overshoot_m"] is not None]
    observed = {
        "max_tracking_error_m": float(np.max(errors[motion])),
        "max_cross_track_m": float(np.max(geometric[motion])),
        "max_corner_overshoot_m": max(overshoots, default=0.),
        "max_final_error_m": float(np.linalg.norm(positions[-1]-points[-1])),
    }
    failures = []
    completed = bool(report.get("completed", report.get("summary", {}).get("passed", False)))
    if not completed:
        failures.append("Runner did not report successful completion")
    if times[-1] < duration:
        failures.append("Feedback does not cover the full motion interval")
    if any(not waypoint["arrival_is_bracketed_by_samples"] for waypoint in waypoints):
        failures.append("Feedback does not bracket every waypoint arrival")
    if any(not corner["sample_count"] for corner in corners):
        failures.append("At least one corner has no feedback samples")
    if any(not corner["outgoing_leg_sample_count"] for corner in corners):
        failures.append("At least one complete outgoing corner leg has no feedback samples")
    for name, value in observed.items():
        if value > limits[name]:
            failures.append(f"{name}: {value:.6g} m exceeds {limits[name]:.6g} m")
    return {
        "schema_version": 2, "sample_count": len(samples), "motion_sample_count": int(np.sum(motion)),
        "motion_duration_s": duration, "schedule_source": schedule_source,
        "max_sample_gap_s": float(np.max(np.diff(times))) if len(times) > 1 else None,
        "motion_time_aligned_error": distribution(errors[motion]),
        "motion_nearest_route_error": distribution(geometric[motion]),
        "final_position_error_m": observed["max_final_error_m"],
        "corner_max_incoming_direction_overshoot_m": observed["max_corner_overshoot_m"],
        "waypoints": waypoints, "corners": corners,
        "acceptance": {"passed": not failures, "runner_completed": completed,
                       "limits_m": limits, "observed_m": observed, "failures": failures},
        "interpretation": [
            "All maxima apply to recorded samples; they are not continuous-time bounds or proof of zero error.",
            "Nearest-route error measures geometric path shape and can hide along-path lag or a wrong segment; time-aligned error is checked separately.",
            "Arrival and departure position errors interpolate adjacent measured poses; they are not independent measurements at the exact event time.",
            "Incoming-direction overshoot measures crossing the waypoint plane normal to the incoming segment, only for turns of at least 90 degrees.",
            "Corner acceptance covers the entire outgoing leg from scheduled arrival through the next arrival, including dwell, plus the local approach window; the short local window is separately reported.",
            "A collinear intermediate waypoint is not a corner.",
        ],
    }


def analyze_report(report, thresholds=None):
    """Runner entry point: pure analysis, top-level pass/fail, no plotting/I/O."""
    try:
        result = analyze(report, thresholds)
    except (ValueError, KeyError, TypeError) as error:
        return {"passed": False, "failures": [str(error)],
                "acceptance": {"passed": False, "failures": [str(error)]}}
    result["passed"] = result["acceptance"]["passed"]
    result["failures"] = result["acceptance"]["failures"]
    return result


def cli_limits(report, overrides=None):
    """Reuse recorded acceptance criteria unless the CLI explicitly overrides one."""
    recorded = report.get("configuration", {}).get("acceptance", {})
    limits = {**DEFAULT_LIMITS, **recorded}
    limits.update({name: value for name, value in (overrides or {}).items() if value is not None})
    return limits


def plot(report, metrics, output, baseline=None, baseline_metrics=None):
    """Save a standalone vector PDF and PNG with equal-scale corner insets."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(13.5, 10.))
    grid = fig.add_gridspec(3, 4, height_ratios=(1.5, 1., 1.))
    route_ax = fig.add_subplot(grid[:2, :2])
    error_ax = fig.add_subplot(grid[0, 2:])
    geometry_ax = fig.add_subplot(grid[1, 2:])
    points = np.array(report["configuration"]["points_m"], float)
    route_ax.plot(*points.T, "--", color="#727a86", linewidth=1.8, label="Waypoint path")
    datasets = [(report, metrics, "#087f8c", "Planned run")]
    if baseline is not None:
        datasets.insert(0, (baseline, baseline_metrics, "#cf6743", "Original run"))
    for data, result, color, label in datasets:
        samples = data["samples"]
        positions = np.array([sample["position_m"][:2] for sample in samples])
        references = np.array([sample["reference_m"][:2] for sample in samples])
        times = sample_times(data)
        motion = (times >= 0.) & (times <= result["motion_duration_s"])
        route_ax.plot(*positions.T, color=color, linewidth=1.1, label=label)
        error_ax.plot(times[motion], np.linalg.norm(positions-references, axis=1)[motion]*1000,
                      color=color, linewidth=1.1, label=label)
        geometry_ax.plot(times[motion], point_segment_distances(positions, points)[motion]*1000,
                         color=color, linewidth=1.1)
    route_ax.scatter(*points.T, color="#17324d", s=16, zorder=5)
    route_ax.set(aspect="equal", xlabel="World X (m)", ylabel="World Y (m)", title="Measured vehicle trajectory")
    route_ax.legend(loc="lower left", fontsize=8)
    error_ax.set(xlabel="Time since motion start (s)", ylabel="Tracking error (mm)", title="Time-aligned reference error")
    error_ax.axhline(metrics["acceptance"]["limits_m"]["max_tracking_error_m"]*1000,
                     color="#999999", linestyle=":", label="Acceptance limit")
    error_ax.legend(fontsize=8)
    geometry_ax.set(xlabel="Time since motion start (s)", ylabel="Path distance (mm)", title="Distance to nearest finite path segment")
    geometry_ax.axhline(metrics["acceptance"]["limits_m"]["max_cross_track_m"]*1000,
                        color="#999999", linestyle=":")
    maximum_local = .005
    corner_axes = []
    for column, corner in enumerate(metrics["corners"][:4]):
        ax = fig.add_subplot(grid[2, column])
        index = corner["waypoint_index"]
        center = points[index]
        ax.plot(*(points[index-1:index+2]-center).T*1000, "--", color="#727a86", linewidth=1.4)
        for data, result, color, label in datasets:
            match = next((item for item in result["corners"] if item["waypoint_index"] == index), None)
            if match is None:
                continue
            samples = data["samples"]
            times = sample_times(data)
            positions = np.array([sample["position_m"][:2] for sample in samples])
            # Display only poses within 80 mm of the corner; align by geometry,
            # never by artificially rescaling the different run durations.
            window = (times >= match["window_s"][0]) & (times <= match["outgoing_leg_window_s"][1])
            local = positions-center
            window &= np.max(np.abs(local), axis=1) <= .08
            if np.any(window):
                ax.plot(*local[window].T*1000, color=color, linewidth=1.3)
                maximum_local = max(maximum_local, float(np.max(np.abs(local[window]))))
        ax.scatter([0.], [0.], color="#17324d", s=18, zorder=5)
        ax.set(aspect="equal", xlabel="Local X (mm)", ylabel="Local Y (mm)", title=f"Corner {index}")
        corner_axes.append(ax)
    extent = min(80., max(10., maximum_local*1000*1.05))
    for ax in corner_axes:
        ax.set_xlim(-extent, extent)
        ax.set_ylim(-extent, extent)
    for ax in fig.axes:
        ax.grid(alpha=.2)
    result = metrics["motion_time_aligned_error"]
    fig.suptitle(f"Full HAMR simulation — measured tracking accuracy\n"
                 f"RMS {result['rms_m']*1000:.2f} mm; peak {result['max_m']*1000:.2f} mm; "
                 f"corner overshoot {metrics['corner_max_incoming_direction_overshoot_m']*1000:.2f} mm; "
                 f"duration {metrics['motion_duration_s']:.1f} s", fontsize=13)
    fig.text(.5, .015, "Corner insets show only poses within 80 mm of the waypoint; acceptance includes the entire outgoing leg.",
             ha="center", fontsize=9, color="#444444")
    fig.tight_layout(rect=(0., .04, 1., .94))
    fig.savefig(output.with_suffix(".png"), dpi=180)
    fig.savefig(output.with_suffix(".pdf"))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--output", type=Path, help="Output basename, defaults to REPORT_DIRECTORY/corner_tracking")
    for option in ("max-tracking-error", "max-cross-track", "max-corner-overshoot", "max-final-error"):
        parser.add_argument(f"--{option}", type=float, default=None, metavar="METERS",
                            help="Override recorded configuration.acceptance; legacy reports use default limits")
    parser.add_argument("--corner-window", type=float, default=2., metavar="SECONDS")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    output = args.output or args.report.parent/"corner_tracking"
    output.parent.mkdir(parents=True, exist_ok=True)
    report = json.loads(args.report.read_text())
    limits = cli_limits(report, {"max_tracking_error_m": args.max_tracking_error, "max_cross_track_m": args.max_cross_track,
                                "max_corner_overshoot_m": args.max_corner_overshoot, "max_final_error_m": args.max_final_error})
    try:
        result = analyze(report, limits, args.corner_window)
        baseline = json.loads(args.baseline.read_text()) if args.baseline else None
        baseline_metrics = analyze(baseline, limits, args.corner_window) if baseline else None
    except (ValueError, KeyError, TypeError) as error:
        result = {"acceptance": {"passed": False, "failures": [str(error)]}}
        output.with_suffix(".json").write_text(json.dumps(result, indent=2)+"\n")
        parser.exit(2, f"Cannot assess run: {error}\n")
    result["source_report"] = str(args.report.resolve())
    if baseline_metrics is not None:
        result["baseline"] = {"source_report": str(args.baseline.resolve()),
                              "motion_time_aligned_error": baseline_metrics["motion_time_aligned_error"],
                              "motion_nearest_route_error": baseline_metrics["motion_nearest_route_error"],
                              "corner_max_incoming_direction_overshoot_m": baseline_metrics["corner_max_incoming_direction_overshoot_m"],
                              "motion_duration_s": baseline_metrics["motion_duration_s"]}
    output.with_suffix(".json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    if not args.no_plot:
        plot(report, result, output, baseline, baseline_metrics)
    print(json.dumps({"metrics": str(output.with_suffix(".json")), "acceptance": result["acceptance"]}, indent=2))
    return 0 if result["acceptance"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
