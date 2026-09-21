#!/usr/bin/env python3
"""Compare tight/wide continuous cornering with the exact-waypoint stop profile.

Distances to original waypoints are evaluated during their particular corner
visits: the route revisits (0, 2) later, so a global closest-point search would
otherwise incorrectly hide the original corner's intentional rounding.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def finite_polyline_distance(positions, vertices):
    positions, vertices = np.asarray(positions, float), np.asarray(vertices, float)
    edges = np.diff(vertices, axis=0)
    squared = np.sum(edges**2, axis=1)
    relative = positions[:, None, :]-vertices[None, :-1, :]
    numerator = np.sum(relative*edges, axis=2)
    fraction = np.divide(numerator, squared, out=np.zeros_like(numerator), where=squared > 0.)
    nearest = vertices[None, :-1, :]+np.clip(fraction, 0., 1.)[:, :, None]*edges
    return np.min(np.linalg.norm(positions[:, None, :]-nearest, axis=2), axis=1)


def measured_arrays(report):
    samples = report["samples"]
    values = {
        "time": np.asarray([sample["trajectory_time_s"] for sample in samples]),
        "position": np.asarray([sample["position_m"][:2] for sample in samples]),
        "reference": np.asarray([sample["reference_m"][:2] for sample in samples]),
        "speed": np.linalg.norm([sample["estimated_world_velocity_m_s"] for sample in samples], axis=1),
        "planned_speed": np.linalg.norm([sample["reference_velocity_m_s"] for sample in samples], axis=1),
    }
    if not all(np.all(np.isfinite(value)) for value in values.values()):
        raise ValueError("All comparison measurements must be finite")
    return values


def analyze_one(path, label):
    source = path.read_bytes()
    report = json.loads(source)
    data = measured_arrays(report)
    points = np.asarray(report["configuration"]["points_m"], float)
    duration = float(report["summary"]["reference_duration_s"])
    motion = (data["time"] >= 0.) & (data["time"] <= duration)
    error = np.linalg.norm(data["position"]-data["reference"], axis=1)
    original_deviation = finite_polyline_distance(data["position"], points)
    plan = report.get("continuous_plan")
    if plan:
        # Recompute acceptance with the current independent analysis tool, rather
        # than trusting embedded historical pass/fail results.
        from analyze_continuous_tracking import analyze_report
        accepted = analyze_report(report)
        if "conservative_max_curve_error_m" not in accepted:
            raise ValueError(f"Cannot analyze {path}: {accepted['failures']}")
        intended_error = accepted["conservative_max_curve_error_m"]
        corners = [{"waypoint_index": item["waypoint_index"],
                    "start_time_s": item["start_time_s"], "end_time_s": item["end_time_s"],
                    "pass_time_s": item["closest_pass_time_s"],
                    "planned_waypoint_miss_m": item["planned_waypoint_miss_m"],
                    "planned_polyline_deviation_m": item["max_intended_deviation_m"]}
                   for item in plan["corners"]]
        planned_max_deviation = max((corner["planned_polyline_deviation_m"] for corner in corners), default=0.)
    else:
        from analyze_corner_tracking import analyze_report
        accepted = analyze_report(report, report["configuration"].get("acceptance"))
        intended_error = float(np.max(original_deviation[motion]))
        arrivals = report["reference_schedule_s"]
        departures = report["reference_departure_schedule_s"]
        corners = []
        for index in range(1, len(points)-1):
            incoming, outgoing = points[index]-points[index-1], points[index+1]-points[index]
            cosine = incoming @ outgoing / (np.linalg.norm(incoming)*np.linalg.norm(outgoing))
            if cosine > 1.-1e-8:
                continue
            corners.append({"waypoint_index": index, "start_time_s": arrivals[index]-2.,
                            "end_time_s": departures[index]+2., "pass_time_s": arrivals[index],
                            "planned_waypoint_miss_m": 0., "planned_polyline_deviation_m": 0.})
        planned_max_deviation = 0.
    for corner in corners:
        index = corner["waypoint_index"]
        window = (data["time"] >= corner["start_time_s"]) & (data["time"] <= corner["end_time_s"])
        if not np.any(window):
            raise ValueError(f"No measurements for corner {index} in {path}")
        corner.update({
            "original_waypoint_m": points[index].tolist(),
            "sample_count": int(np.sum(window)),
            "sampled_min_distance_to_original_waypoint_m": float(np.min(np.linalg.norm(data["position"][window]-points[index], axis=1))),
            "max_actual_deviation_from_original_polyline_m": float(np.max(original_deviation[window])),
            "max_time_aligned_tracking_error_m": float(np.max(error[window])),
            "min_actual_speed_m_s": float(np.min(data["speed"][window])),
            "min_planned_speed_m_s": float(np.min(data["planned_speed"][window])),
        })
    result = {
        "label": label, "source_report": str(path.resolve()), "source_sha256": hashlib.sha256(source).hexdigest(),
        "trajectory_profile": report["trajectory_profile"],
        "position_gain_s_inv": report["configuration"]["position_gain_s_inv"],
        "configuration_sha256": report.get("config_sha256"),
        "implementation_sha256": report.get("implementation_sha256", {}),
        "acceptance_recomputed_passed": accepted["passed"], "acceptance_failures": accepted["failures"],
        "motion_duration_s": duration,
        "tracking_rmse_m": float(np.sqrt(np.mean(error[motion]**2))),
        "max_time_aligned_tracking_error_m": float(np.max(error[motion])),
        "max_error_from_own_intended_path_m": intended_error,
        "planned_max_deviation_from_original_polyline_m": planned_max_deviation,
        "max_actual_deviation_from_original_polyline_m": float(np.max(original_deviation[motion])),
        "min_actual_corner_speed_m_s": min((corner["min_actual_speed_m_s"] for corner in corners), default=None),
        "final_position_error_m": float(np.linalg.norm(data["position"][-1]-points[-1])),
        "command_statistics": report["command_statistics"], "corners": corners,
    }
    return result, report, data


def plot_comparison(datasets, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(14., 11.))
    colors = ("#087f8c", "#cc6b40", "#536a91")
    points = np.asarray(datasets[0][1]["configuration"]["points_m"], float)
    first_index = datasets[0][0]["corners"][0]["waypoint_index"]
    center = points[first_index]
    incoming = center-points[first_index-1]
    outgoing = points[first_index+1]-center
    incoming /= np.linalg.norm(incoming)
    outgoing /= np.linalg.norm(outgoing)
    for ax in axes[0]:
        ax.plot([0., 0., 650.], [-650., 0., 0.], "--", color="#9ba0a6", linewidth=1.4, label="Original sharp path")
        ax.scatter([0.], [0.], color="#17324d", s=26, zorder=5)
        ax.set(aspect="equal", xlabel="Distance along outgoing direction (mm)",
               ylabel="Distance along incoming direction (mm)")
    for (result, report, data), color in zip(datasets, colors):
        corner = next(item for item in result["corners"] if item["waypoint_index"] == first_index)
        window = (data["time"] >= corner["pass_time_s"]-7.) & (data["time"] <= corner["pass_time_s"]+7.)
        local = data["position"][window]-center
        reference = data["reference"][window]-center
        for ax in axes[0]:
            ax.plot(reference @ outgoing*1000, reference @ incoming*1000, "--", color=color, linewidth=2., alpha=.65)
            ax.plot(local @ outgoing*1000, local @ incoming*1000, color=color, linewidth=1.1, label=result["label"])
        axes[1, 0].plot(data["time"][window]-corner["pass_time_s"], data["speed"][window], color=color,
                        linewidth=1.2, label=result["label"])
        axes[1, 0].plot(data["time"][window]-corner["pass_time_s"], data["planned_speed"][window], "--",
                        color=color, linewidth=.8, alpha=.65)
    axes[0, 0].set(xlim=(-15., 650.), ylim=(-650., 15.), title="First corner: full geometric comparison")
    axes[0, 1].set(xlim=(-5., 125.), ylim=(-125., 5.), title="Tight-corner detail: solid measured, dashed planned")
    axes[0, 0].legend(fontsize=8, loc="lower right")
    axes[1, 0].set(xlabel="Time relative to this profile's planned corner pass (s)", ylabel="Speed (m/s)",
                   title="Measured speed; each profile retains its own timing", xlim=(-7., 7.))
    axes[1, 0].legend(fontsize=8)
    axes[1, 1].axis("off")
    rows = [[result["label"], f"{result['motion_duration_s']:.2f}",
             f"{result['max_actual_deviation_from_original_polyline_m']*1000:.2f}",
             f"{result['max_time_aligned_tracking_error_m']*1000:.2f}",
             f"{result['min_actual_corner_speed_m_s']*100:.2f}"] for result, _, _ in datasets]
    table = axes[1, 1].table(cellText=rows, colLabels=["Profile", "Duration\n(s)", "Actual original-\npath distance\n(mm)",
                                                    "Tracking\npeak\n(mm)", "Corner speed\nminimum\n(cm/s)"],
                            cellLoc="center", loc="center", colWidths=(.30, .14, .20, .16, .20))
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1., 2.6)
    axes[1, 1].set_title("Rounding distance and tracking error are different quantities", fontsize=10)
    for ax in (axes[0, 0], axes[0, 1], axes[1, 0]):
        ax.grid(alpha=.2)
    fig.suptitle("Full HAMR simulation: tighter continuous corners versus wider blends and stops", fontsize=14)
    fig.text(.5, .035, "The 20 mm blend budget is a planned distance from the original polyline; "
             "the planned nearest pass to a 90° vertex is 28.28 mm.\n"
             "Actual tracking errors are measured relative to each profile's own intended reference. "
             "Distances describe the tracked base point, not robot-footprint clearance.", ha="center", fontsize=9)
    fig.tight_layout(rect=(0., .09, 1., .95))
    fig.savefig(output.with_suffix(".png"), dpi=180)
    fig.savefig(output.with_suffix(".pdf"))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tight", type=Path, help="New 20 mm continuous-blend trajectory.json")
    parser.add_argument("--wide", type=Path, required=True,
                        help="100 mm continuous-blend reference run; provide your retained report explicitly")
    parser.add_argument("--stop", type=Path, required=True,
                        help="Exact-waypoint stop-profile reference run")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    output = args.output or args.tight.parent/"continuous_profile_comparison"
    paths = [args.tight, args.wide, args.stop]
    if any(output.with_suffix(".json").resolve() == path.resolve() for path in paths):
        parser.error("Comparison output must differ from every raw report")
    try:
        datasets = [analyze_one(path, label) for path, label in zip(paths, ("Continuous 20 mm", "Continuous 100 mm", "Exact-waypoint stops"))]
        original_points = np.asarray(datasets[0][1]["configuration"]["points_m"])
        if any(not np.array_equal(original_points, dataset[1]["configuration"]["points_m"]) for dataset in datasets[1:]):
            raise ValueError("Comparison reports must use the same original waypoint coordinates")
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    result = {"schema_version": 1, "profiles": [dataset[0] for dataset in datasets],
              "notes": ["All metrics are independently recomputed from raw reports; embedded historical acceptance results are not used.",
                        "Original-polyline distance includes deliberate rounding. It is not tracking error against the planned curve.",
                        "For a symmetric 90-degree blend, nearest planned vertex distance is sqrt(2) times the maximum deviation from the original polyline.",
                        "Actual vertex distances are minima over measured samples during the corresponding corner interval, avoiding later revisits to the same waypoint.",
                        "Stop-profile corner speed windows include the arrival hold; continuous-profile windows cover each full blend.",
                        "These are sampled tracked-base-point distances, not continuous-time guarantees or robot-footprint clearances."]}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.with_suffix(".json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    if not args.no_plot:
        plot_comparison(datasets, output)
    print(json.dumps(result, indent=2))
    return 0 if all(profile["acceptance_recomputed_passed"] for profile in result["profiles"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
