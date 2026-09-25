#!/usr/bin/env python3
"""Validate continuous Gazebo tracking against its deliberately rounded reference.

Measurements remain raw. Position tracking uses the reference recorded at each
odometry source timestamp. Geometric error is measured against the planned curve
near the matching route time, not against the original sharp waypoint polyline.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


DEFAULT_LIMITS = {
    "max_tracking_error_m": .005,
    "max_cross_track_m": .002,
    "max_final_error_m": .0005,
    "min_corner_speed_m_s": .03,
    "max_corner_slowdown_fraction": .30,
}
MAX_SAMPLE_GAP_S = .10
LOCAL_REFERENCE_WINDOW_S = .5


def stats(values, unit="m"):
    values = np.asarray(values, float)
    if not len(values):
        return {f"{name}_{unit}": None for name in ("min", "rms", "max", "p50", "p95", "p99")}
    return {f"min_{unit}": float(np.min(values)), f"max_{unit}": float(np.max(values)),
            f"rms_{unit}": float(np.sqrt(np.mean(values**2))),
            **{f"p{percentile}_{unit}": float(np.percentile(values, percentile)) for percentile in (50, 95, 99)}}


def resolved_limits(report, overrides=None):
    recorded = report.get("configuration", {}).get("acceptance", {})
    result = {name: recorded.get(name, value) for name, value in DEFAULT_LIMITS.items()}
    result.update({name: value for name, value in (overrides or {}).items()
                   if name in DEFAULT_LIMITS and value is not None})
    if any(not math.isfinite(value) or value <= 0 for value in result.values()):
        raise ValueError("Acceptance limits must be finite and positive")
    if result["max_corner_slowdown_fraction"] >= 1.:
        raise ValueError("max_corner_slowdown_fraction must be less than one")
    return result


def arrays(report):
    samples = report.get("samples", [])
    planned = report.get("continuous_plan", {}).get("samples", [])
    if len(samples) < 2 or len(planned) < 2:
        raise ValueError("Need measured feedback and a dense planned curve")
    values = {
        "time": np.array([sample["trajectory_time_s"] for sample in samples], float),
        "position": np.array([sample["position_m"][:2] for sample in samples], float),
        "reference": np.array([sample["reference_m"][:2] for sample in samples], float),
        "velocity": np.array([sample["estimated_world_velocity_m_s"][:2] for sample in samples], float),
        "plan_time": np.array([sample["time_s"] for sample in planned], float),
        "plan_position": np.array([sample["position_m"][:2] for sample in planned], float),
        "plan_velocity": np.array([sample["velocity_m_s"][:2] for sample in planned], float),
        "plan_acceleration": np.array([sample["acceleration_m_s2"][:2] for sample in planned], float),
    }
    if not all(np.all(np.isfinite(value)) for value in values.values()):
        raise ValueError("Feedback, references, velocities, and plan must be finite")
    for name in ("position", "reference", "velocity", "plan_position", "plan_velocity", "plan_acceleration"):
        if values[name].ndim != 2 or values[name].shape[1] != 2:
            raise ValueError(f"{name} must contain XY pairs")
    for name in ("time", "plan_time"):
        if np.any(np.diff(values[name]) <= 0):
            raise ValueError(f"{name} must be strictly increasing")
    return values


def local_curve_distance(positions, times, plan_positions, plan_times,
                         window_s=LOCAL_REFERENCE_WINDOW_S):
    """Project onto nearby finite reference chords without crossing the route."""
    distances = []
    for position, time_s in zip(positions, times):
        first = max(0, int(np.searchsorted(plan_times, time_s-window_s))-1)
        last = min(len(plan_times)-1, int(np.searchsorted(plan_times, time_s+window_s)))
        if first >= last:
            first = max(0, min(first, len(plan_times)-2))
            last = first+1
        starts = plan_positions[first:last]
        edges = plan_positions[first+1:last+1]-starts
        squared = np.sum(edges**2, axis=1)
        numerator = np.sum((position-starts)*edges, axis=1)
        fraction = np.divide(numerator, squared, out=np.zeros_like(numerator), where=squared > 0.)
        closest = starts+np.clip(fraction, 0., 1.)[:, None]*edges
        distances.append(float(np.min(np.linalg.norm(position-closest, axis=1))))
    return np.asarray(distances)


def approximation_bound(plan, data):
    """Chord error <= max Cartesian acceleration * dt**2 / 8.

    This bound requires the planner's declared continuous acceleration bound.
    Sparse sampled maxima cannot establish that bound. If absent, require a
    declared continuous speed bound and use the more conservative v*dt/2.
    """
    dt = float(np.max(np.diff(data["plan_time"])))
    acceleration = plan.get("maximum_cartesian_acceleration_m_s2", plan.get("max_acceleration_m_s2"))
    if acceleration is not None:
        acceleration = float(acceleration)
        if not math.isfinite(acceleration) or acceleration <= 0:
            raise ValueError("Planner acceleration bound must be finite and positive")
        sampled_peak = float(np.max(np.linalg.norm(data["plan_acceleration"], axis=1)))
        if sampled_peak > acceleration*(1.+1e-6)+1e-10:
            raise ValueError("Sampled planned acceleration exceeds the planner's declared bound")
        return acceleration*dt*dt/8., "declared Cartesian acceleration bound * max plan dt squared / 8"
    speed = plan.get("maximum_cartesian_speed_m_s", plan.get("max_speed_m_s"))
    if speed is None or not math.isfinite(speed) or speed <= 0:
        raise ValueError("Plan needs a declared continuous acceleration or speed bound for curve discretization")
    return float(speed)*dt/2., "declared speed bound * max plan dt / 2"


def rounding_metrics(report, data):
    """Keep deliberate path-design changes separate from controller error."""
    points = np.asarray(report["configuration"]["points_m"], float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2 or not np.all(np.isfinite(points)):
        raise ValueError("Original waypoint path must contain finite XY pairs")
    edges = np.diff(points, axis=0)
    squared = np.sum(edges**2, axis=1)
    relative = data["plan_position"][:, None, :]-points[None, :-1, :]
    numerator = np.sum(relative*edges, axis=2)
    fraction = np.divide(numerator, squared, out=np.zeros_like(numerator), where=squared > 0.)
    closest = points[None, :-1, :]+np.clip(fraction, 0., 1.)[:, :, None]*edges
    deviations = np.min(np.linalg.norm(data["plan_position"][:, None, :]-closest, axis=2), axis=1)
    corners = []
    for corner in report["continuous_plan"].get("corners", []):
        row = {key: corner[key] for key in ("waypoint_index", "planned_waypoint_miss_m", "max_intended_deviation_m") if key in corner}
        index = corner["waypoint_index"]
        mask = (data["plan_time"] >= corner["start_time_s"]) & (data["plan_time"] <= corner["end_time_s"])
        if np.any(mask):
            row["sampled_min_distance_to_original_waypoint_m"] = float(np.min(np.linalg.norm(data["plan_position"][mask]-points[index], axis=1)))
            row["sampled_max_deviation_from_original_polyline_m"] = float(np.max(deviations[mask]))
        corners.append(row)
    return {"description": "Intentional reference design changes, not measured tracking errors",
            "sampled_max_deviation_from_original_polyline_m": float(np.max(deviations)), "corners": corners}


def _analyze(report, thresholds):
    limits = resolved_limits(report, thresholds)
    data = arrays(report)
    plan = report["continuous_plan"]
    time_s, plan_time = data["time"], data["plan_time"]
    duration = float(plan.get("duration_s", plan.get("trajectory_duration_s", plan_time[-1])))
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("Plan duration must be finite and positive")
    if abs(plan_time[0]) > 1e-9 or abs(plan_time[-1]-duration) > 1e-7:
        raise ValueError("Dense planned curve must cover time zero through its full duration")
    bound, bound_method = approximation_bound(plan, data)
    motion = (time_s >= 0.) & (time_s <= duration)
    if not np.any(motion):
        raise ValueError("No source-stamped measurements cover motion")
    error = np.linalg.norm(data["position"]-data["reference"], axis=1)
    geometric = local_curve_distance(data["position"][motion], time_s[motion], data["plan_position"], plan_time)
    actual_speed = np.linalg.norm(data["velocity"], axis=1)
    planned_speed_table = np.linalg.norm(data["plan_velocity"], axis=1)
    planned_speed = np.interp(time_s, plan_time, planned_speed_table)
    interpolated_reference = np.column_stack([np.interp(time_s, plan_time, data["plan_position"][:, dim]) for dim in (0, 1)])
    reference_consistency = np.linalg.norm(data["reference"]-interpolated_reference, axis=1)
    consistency_tolerance = bound+1e-7
    failures = []
    completed = bool(report.get("completed", False))
    if not completed:
        failures.append("Runner did not complete the entire trajectory and final hold")
    if time_s[0] > 0. or time_s[-1] < duration:
        failures.append("Feedback does not bracket the complete trajectory")
    max_gap = float(np.max(np.diff(time_s)))
    if max_gap > MAX_SAMPLE_GAP_S+1e-9:
        failures.append(f"Source-stamped feedback gap {max_gap:.6g} s exceeds {MAX_SAMPLE_GAP_S} s")
    if bound > min(.0001, limits["max_cross_track_m"]*.1):
        failures.append(f"Planned curve discretization bound {bound:.6g} m is too coarse")
    if np.max(reference_consistency) > consistency_tolerance:
        failures.append(f"Recorded reference is inconsistent with source time: {np.max(reference_consistency):.6g} m exceeds curve interpolation tolerance {consistency_tolerance:.6g} m")
    command_stats = report.get("command_statistics", {})
    if not all(name in command_stats for name in ("updates", "speed_limited_updates", "acceleration_limited_updates")):
        failures.append("Missing full-command speed and acceleration limiter statistics")
    else:
        if command_stats["updates"] <= 0:
            failures.append("No command updates were reported")
        for name in ("speed_limited_updates", "acceleration_limited_updates"):
            if command_stats[name] != 0:
                failures.append(f"{name} must be zero; observed {command_stats[name]}")
    corners = []
    for corner in plan.get("corners", []):
        begin, end = float(corner["start_time_s"]), float(corner["end_time_s"])
        if not math.isfinite(begin) or not math.isfinite(end) or not 0. <= begin < end <= duration:
            raise ValueError("Corner intervals must be finite, positive, and within the plan")
        mask = (time_s >= begin) & (time_s <= end)
        source_plan = (plan_time >= begin) & (plan_time <= end)
        index = corner["waypoint_index"]
        if not np.any(mask) or time_s[0] > begin or time_s[-1] < end:
            failures.append(f"Corner {index} is not covered by feedback")
            corners.append({"waypoint_index": index, "interval_s": [begin, end], "sample_count": 0})
            continue
        planned_here = planned_speed[mask]
        actual_here = actual_speed[mask]
        ratio = np.divide(actual_here, planned_here, out=np.zeros_like(actual_here), where=planned_here > 1e-12)
        slowdown = float(np.max(np.maximum(0., 1.-ratio)))
        minimum = float(np.min(actual_here))
        if minimum < limits["min_corner_speed_m_s"]:
            failures.append(f"Corner {index} measured speed {minimum:.6g} m/s is below {limits['min_corner_speed_m_s']:.6g} m/s")
        if slowdown > limits["max_corner_slowdown_fraction"]:
            failures.append(f"Corner {index} slowdown fraction {slowdown:.6g} exceeds {limits['max_corner_slowdown_fraction']:.6g}")
        if np.any(source_plan) and np.min(planned_speed_table[source_plan]) < limits["min_corner_speed_m_s"]:
            failures.append(f"Corner {index} planned speed violates the minimum corner speed")
        corners.append({"waypoint_index": index, "interval_s": [begin, end], "sample_count": int(np.sum(mask)),
                        "actual_speed": stats(actual_here, "m_s"), "planned_speed": stats(planned_here, "m_s"),
                        "max_slowdown_fraction": slowdown, "tracking_error": stats(error[mask])})
    interior = plan.get("interior_motion_interval_s")
    if interior is None:
        # Exclude only the first/last ramps, keeping any intervening slowdowns.
        moving = np.flatnonzero(planned_speed_table >= max(.05, 2*limits["min_corner_speed_m_s"]))
        if len(moving) < 2:
            raise ValueError("Plan needs an explicit interior_motion_interval_s or a sustained moving interval")
        interior = [float(plan_time[moving[0]]), float(plan_time[moving[-1]])]
        interior_source = "inferred contiguous interval between first/last planned speed above startup threshold"
    else:
        interior_source = "planner-declared interval excluding initial/final ramps"
    first, last = map(float, interior)
    if not 0. <= first < last <= duration:
        raise ValueError("Interior motion interval must be within the route")
    for corner in corners:
        if first > corner["interval_s"][0]+1e-9 or last < corner["interval_s"][1]-1e-9:
            raise ValueError("Interior motion interval must include every complete corner")
    interior_mask = (time_s >= first) & (time_s <= last)
    interior_speeds = actual_speed[interior_mask]
    if not len(interior_speeds):
        failures.append("No feedback covers the interior motion interval")
    elif np.min(interior_speeds) < limits["min_corner_speed_m_s"]:
        failures.append(f"Interior motion contains a stop/slowdown below {limits['min_corner_speed_m_s']:.6g} m/s")
    observed = {"max_tracking_error_m": float(np.max(error[motion])),
                "max_cross_track_m": float(np.max(geometric))+bound,
                "max_final_error_m": float(np.linalg.norm(data["position"][-1]-data["plan_position"][-1]))}
    for name, value in observed.items():
        if value > limits[name]:
            failures.append(f"{name}: {value:.6g} m exceeds {limits[name]:.6g} m")
    planned_jerk = np.gradient(data["plan_acceleration"], plan_time, axis=0)
    return {
        "schema_version": 1, "profile": "continuous", "passed": not failures, "failures": failures,
        "sample_count": len(time_s), "motion_sample_count": int(np.sum(motion)),
        "motion_duration_s": duration, "max_sample_gap_s": max_gap,
        "motion_time_aligned_error": stats(error[motion]),
        "motion_local_curve_error": stats(geometric),
        "curve_approximation_bound_m": bound, "curve_approximation_bound_method": bound_method,
        "source_time_reference_consistency": stats(reference_consistency),
        "source_time_reference_consistency_tolerance_m": consistency_tolerance,
        "intended_rounding": rounding_metrics(report, data),
        "conservative_max_curve_error_m": observed["max_cross_track_m"],
        "local_curve_search_window_s": LOCAL_REFERENCE_WINDOW_S,
        "final_position_error_m": observed["max_final_error_m"], "corners": corners,
        "interior_motion_interval_s": [first, last], "interior_interval_source": interior_source,
        "interior_actual_speed": stats(interior_speeds, "m_s"),
        "reference_acceleration": stats(np.linalg.norm(data["plan_acceleration"], axis=1), "m_s2"),
        "reference_numerical_jerk": stats(np.linalg.norm(planned_jerk, axis=1), "m_s3"),
        "command_statistics": command_stats,
        "acceptance": {"passed": not failures, "limits": limits, "observed_m": observed,
                       "runner_completed": completed, "failures": failures},
        "interpretation": [
            "The rounded planned curve intentionally differs from the original sharp waypoint polyline; this design difference is not a tracking error.",
            "Time-aligned errors are recomputed from raw measured positions and source-time references; measured positions are never snapped to the reference.",
            "Geometric errors project onto finite planned-curve chords within a local time window, preventing distant route crossings from hiding local error.",
            "Cross-track acceptance adds the planner-bound chord approximation error to the largest measured chord distance.",
            "Speed is measured from the recorded odometry-derived velocity; no-interior-stop validation excludes only the initial and final ramps.",
            "All observed maxima and minima describe recorded samples; they do not prove exact or continuous-time perfect tracking.",
        ],
    }


def analyze_report(report, thresholds=None):
    """Pure fail-closed analysis; malformed or incomplete reports return failure."""
    try:
        return _analyze(report, thresholds)
    except (ValueError, TypeError, KeyError, IndexError, OverflowError) as error:
        return {"schema_version": 1, "profile": "continuous", "passed": False,
                "failures": [str(error)], "acceptance": {"passed": False, "failures": [str(error)]}}


def plot(report, result, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = arrays(report)
    time_s, plan_time = data["time"], data["plan_time"]
    motion = (time_s >= 0.) & (time_s <= result["motion_duration_s"])
    geometric = local_curve_distance(data["position"][motion], time_s[motion], data["plan_position"], plan_time)
    error = np.linalg.norm(data["position"]-data["reference"], axis=1)
    speed = np.linalg.norm(data["velocity"], axis=1)
    plan_speed = np.linalg.norm(data["plan_velocity"], axis=1)
    fig, axes = plt.subplots(3, 2, figsize=(14., 13.))
    original = np.array(report["configuration"]["points_m"], float)
    axes[0, 0].plot(*original.T, "--", color="#9ba0a6", linewidth=1.2, label="Original sharp waypoint path")
    axes[0, 0].plot(*data["plan_position"].T, color="#17324d", linewidth=2., label="Intended continuous reference")
    axes[0, 0].plot(*data["position"].T, color="#087f8c", linewidth=1., label="Raw Gazebo position")
    axes[0, 0].set(aspect="equal", xlabel="World X (m)", ylabel="World Y (m)", title="Planned corner rounding and measured tracking")
    axes[0, 0].legend(fontsize=8)
    axes[0, 1].plot(time_s[motion], error[motion]*1000, color="#087f8c", label="Time-aligned tracking")
    axes[0, 1].plot(time_s[motion], geometric*1000, color="#d16b43", linewidth=1., label="Local curve distance")
    axes[0, 1].axhline(result["acceptance"]["limits"]["max_tracking_error_m"]*1000, color="#087f8c", linestyle=":", alpha=.6)
    axes[0, 1].axhline(result["acceptance"]["limits"]["max_cross_track_m"]*1000, color="#d16b43", linestyle=":", alpha=.6)
    axes[0, 1].set(xlabel="Motion time (s)", ylabel="Error (mm)", title="Error relative to the intended curve")
    axes[0, 1].legend(fontsize=8)
    axes[1, 0].plot(plan_time, plan_speed, color="#17324d", label="Planned")
    axes[1, 0].plot(time_s[motion], speed[motion], color="#087f8c", linewidth=1., label="Measured")
    first, last = result["interior_motion_interval_s"]
    axes[1, 0].plot([first, last], [result["acceptance"]["limits"]["min_corner_speed_m_s"]]*2,
                    color="#d16b43", linestyle=":", label="Minimum interior speed")
    for corner in result["corners"]:
        axes[1, 0].axvspan(*corner["interval_s"], color="#999999", alpha=.10)
    axes[1, 0].set(xlabel="Motion time (s)", ylabel="Speed (m/s)", title="Continuous motion; shaded intervals are corners")
    axes[1, 0].legend(fontsize=8)
    for dim, label in enumerate(("X", "Y")):
        axes[1, 1].plot(plan_time, data["plan_acceleration"][:, dim], linewidth=1., label=label)
    axes[1, 1].set(xlabel="Motion time (s)", ylabel="Reference acceleration (m/s²)", title="Planned acceleration")
    axes[1, 1].legend(fontsize=8)
    for corner in result["corners"]:
        if not corner["sample_count"]:
            continue
        begin, end = corner["interval_s"]
        mask = (time_s >= begin) & (time_s <= end)
        line, = axes[2, 0].plot((time_s[mask]-begin)/(end-begin), speed[mask], linewidth=1.2,
                               label=f"Corner {corner['waypoint_index']} measured")
        planned_mask = (plan_time >= begin) & (plan_time <= end)
        axes[2, 0].plot((plan_time[planned_mask]-begin)/(end-begin), plan_speed[planned_mask],
                        "--", linewidth=.9, color=line.get_color())
    axes[2, 0].axhline(result["acceptance"]["limits"]["min_corner_speed_m_s"], color="#555555", linestyle=":")
    axes[2, 0].set(xlabel="Fraction through corner interval", ylabel="Speed (m/s)", title="Every corner: solid measured, dashed planned")
    axes[2, 0].legend(fontsize=7)
    jerk = np.gradient(data["plan_acceleration"], plan_time, axis=0)
    for dim, label in enumerate(("X", "Y")):
        axes[2, 1].plot(plan_time, jerk[:, dim], linewidth=1., label=label)
    axes[2, 1].set(xlabel="Motion time (s)", ylabel="Reference jerk (m/s³)", title="Numerical derivative of planned acceleration")
    axes[2, 1].legend(fontsize=8)
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.suptitle(f"Continuous full-vehicle trajectory — {'PASS' if result['passed'] else 'FAIL'}\n"
                 f"Tracking peak {result['motion_time_aligned_error']['max_m']*1000:.2f} mm; "
                 f"curve error bound {result['conservative_max_curve_error_m']*1000:.2f} mm; "
                 f"duration {result['motion_duration_s']:.1f} s", fontsize=14)
    fig.tight_layout(rect=(0., .025, 1., .95))
    fig.text(.5, .012, f"Curve discretization bound: {result['curve_approximation_bound_m']*1e6:.2f} µm. "
             "Recorded samples quantify accuracy; they do not establish zero continuous-time error.", ha="center", fontsize=9)
    fig.savefig(output.with_suffix(".png"), dpi=180)
    fig.savefig(output.with_suffix(".pdf"))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--no-plot", action="store_true")
    for name in DEFAULT_LIMITS:
        parser.add_argument("--"+name.replace("_", "-"), type=float, default=None)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    overrides = {name: getattr(args, name) for name in DEFAULT_LIMITS}
    result = analyze_report(report, overrides)
    result["source_report"] = str(args.report.resolve())
    output = args.output or args.report.parent/"continuous_tracking"
    if output.with_suffix(".json").resolve() == args.report.resolve():
        parser.error("Analysis output must differ from the raw input report")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.with_suffix(".json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    if not args.no_plot and "motion_time_aligned_error" in result:
        plot(report, result, output)
    print(json.dumps({"metrics": str(output.with_suffix(".json")), "passed": result["passed"],
                      "failures": result["failures"]}, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
