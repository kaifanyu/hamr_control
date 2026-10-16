#!/usr/bin/env python3
"""Compare paired ball/swivel runs without equating their passive joint angles."""
import argparse
import json
from pathlib import Path

import numpy as np

from analyze_ball_stress import common_metrics


CONTROLLER_KEYS = ("speed_m_s", "wheel_radius_m", "half_track_m", "base_ahead_of_axle_m",
    "position_gain_s_inv", "orientation_gain_s_inv", "max_translation_m_s", "max_wheel_rate_rad_s",
    "max_wheel_acceleration_rad_s2", "max_gimbal_rate_rad_s", "fixed_turret_yaw_rad")


def equal_points(left, right):
    if left is None or right is None:
        return False
    a, b = np.asarray(left), np.asarray(right)
    return a.shape == b.shape and bool(np.allclose(a, b, atol=1e-9, rtol=0.))


def compare_trajectories(swivel, ball, primary_region=None, context=None):
    """Return quantitative within-pair differences, including comparability flags."""
    sm, sr, _, _ = common_metrics(swivel)
    bm, br, _, _ = common_metrics(ball)
    executed = (sm["executed_route_points_m"], bm["executed_route_points_m"])
    requested = (sm["requested_route_points_m"], bm["requested_route_points_m"])
    startup = any((report.get("startup_alignment") or {}).get("enabled", False) for report in (sm, bm))
    if primary_region is None:
        primary_region = "later_legs" if startup else "motion"
    if primary_region not in sm["regions"]:
        raise ValueError(f"Unknown comparison region {primary_region}")
    mismatches = {key: {"swivel": sm["configuration"].get(key), "ball": bm["configuration"].get(key)}
                  for key in CONTROLLER_KEYS if sm["configuration"].get(key) != bm["configuration"].get(key)}
    yaw_delta = (br["body_yaw_unwrapped_deg"][0]-sr["body_yaw_unwrapped_deg"][0]+180.) % 360.-180.
    source_gap = None
    if executed[0] and executed[1]:
        source_gap = float(np.linalg.norm(np.asarray(executed[1][0])-np.asarray(executed[0][0])))
    comparable = {"requested_waypoints_match": equal_points(*requested),
        "executed_waypoints_match": equal_points(*executed),
        "later_waypoints_match": equal_points(executed[0][1:] if executed[0] else None, executed[1][1:] if executed[1] else None),
        "executed_source_separation_m": source_gap,
        "controller_parameter_mismatches": mismatches,
        "initial_measured_body_yaw_difference_deg": float(yaw_delta),
        "initial_measured_xy_separation_m": float(np.hypot(br["body_x_m"][0]-sr["body_x_m"][0], br["body_y_m"][0]-sr["body_y_m"][0])),
        "startup_preparation_present": startup,
        "same_contact_geometry_or_mass": False,
        "note": "Both models retain their own geometry/mass/contact and bearing assumptions. Assisted starts can end at different first-source poses; later legs retain the same destinations but may inherit different approach states."}
    regions = {}
    for region in ("motion", "first_leg", "later_legs", "startup", "hold"):
        s, b = sm["regions"][region], bm["regions"][region]
        st, bt = s["tracking_error_m"], b["tracking_error_m"]
        delta = bt["rms"]-st["rms"] if st["rms"] is not None and bt["rms"] is not None else None
        regions[region] = {"swivel": s, "ball": b,
            "tracking_rms_ball_minus_swivel_m": delta,
            "tracking_rms_change_percent": 100*delta/st["rms"] if delta is not None and st["rms"] > 1e-12 else None,
            "tracking_peak_ball_minus_swivel_m": bt["max_abs"]-st["max_abs"] if st["max_abs"] is not None and bt["max_abs"] is not None else None,
            "drive_limited_fraction_ball_minus_swivel": b["either_limited_fraction"]-s["either_limited_fraction"] if s["either_limited_fraction"] is not None and b["either_limited_fraction"] is not None else None}
    selected = regions[primary_region]
    delta = selected["tracking_rms_ball_minus_swivel_m"]
    if delta is None:
        assessment = "Insufficient common motion samples for the selected region."
    else:
        direction = "lower" if delta < 0 else "higher" if delta > 0 else "equal"
        assessment = f"Ball tracking RMS is {direction} in this recorded pair ({primary_region}); this does not establish statistical superiority or identify caster contact friction as the cause."
        peak_delta = selected["tracking_peak_ball_minus_swivel_m"]
        if peak_delta is not None and delta*peak_delta < 0:
            assessment += " RMS and peak error move in opposite directions, so this pair has a tracking tradeoff."
    if mismatches:
        assessment += " Controller/reference settings differ; treat the pair as confounded."
    if startup:
        assessment += " Startup policies differ because the ball mechanism has no swivel-facing gate; prioritize later legs and report preparation separately."
    context = context or {}
    if context.get("non_equivalent_physical_parameters") or "swivel-only" in context.get("ball_role", ""):
        assessment += " This ball trial is a nominal repeat baseline for a swivel-only physical variant, not an equivalent parameter intervention."
    status = {model: {key: metrics[key] for key in ("completed", "trajectory_passed")}
              for model, metrics in (("swivel", sm), ("ball", bm))}
    if not all(value["completed"] and value["trajectory_passed"] for value in status.values()):
        assessment += " At least one run did not complete/pass its configured checks; lower sampled error alone cannot establish a successful comparable execution."
    return {"schema_version": 1, "context": context, "primary_region": primary_region,
            "run_status": status,
            "comparability": comparable, "regions": regions, "assessment": assessment,
            "swivel_feedback_timing": sm["feedback_timing"], "ball_feedback_timing": bm["feedback_timing"],
            "joint_alignment_comparison": None,
            "contact_slip_comparison": None,
            "measurement_note": "Duration-weighted body tracking, drive-command limiting and pose evolution are common metrics. Ball carrier angle and swivel heading are different physical quantities. Kinematic residuals are finite-difference proxies, not motor effort or calibrated contact slip."}, (sr, br)


def comparison_row(report):
    row = {"primary_region": report["primary_region"],
           "requested_waypoints_match": report["comparability"]["requested_waypoints_match"],
           "executed_waypoints_match": report["comparability"]["executed_waypoints_match"],
           "later_waypoints_match": report["comparability"]["later_waypoints_match"],
           "executed_source_separation_m": report["comparability"]["executed_source_separation_m"]}
    for model, status in report["run_status"].items():
        row[model+"_completed"] = status["completed"]
        row[model+"_trajectory_passed"] = status["trajectory_passed"]
    for region in ("motion", "first_leg", "later_legs"):
        value = report["regions"][region]
        for model in ("swivel", "ball"):
            m = value[model]
            row[f"{region}_{model}_rms_m"] = m["tracking_error_m"]["rms"]
            row[f"{region}_{model}_peak_m"] = m["tracking_error_m"]["max_abs"]
            row[f"{region}_{model}_limited_fraction"] = m["either_limited_fraction"]
            row[f"{region}_{model}_net_body_yaw_deg"] = m["body_heading"]["net_change_deg"]
            row[f"{region}_{model}_body_height_range_m"] = m["body_height_range_m"]
        row[f"{region}_rms_change_percent"] = value["tracking_rms_change_percent"]
    row["assessment"] = report["assessment"]
    return row


def plot_comparison(rows, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for data, label in zip(rows, ("Swivel", "Split ball")):
        motion = data["phase"] == "motion"
        time = data["elapsed_s"]-data["elapsed_s"][np.flatnonzero(motion)[0]] if motion.any() else data["elapsed_s"]
        axes[0, 0].plot(data["body_x_m"][motion], data["body_y_m"][motion], label=label)
        axes[0, 1].plot(time[motion], data["tracking_error_m"][motion]*1000, label=label)
        axes[1, 0].plot(time[motion], data["body_yaw_unwrapped_deg"][motion], label=label)
        axes[1, 1].plot(time[motion], data["body_height_m"][motion]*1000, label=label)
    axes[0, 0].set(title="Measured route (preparation excluded)", xlabel="World X (m)", ylabel="World Y (m)")
    axes[0, 0].set_aspect("equal", adjustable="datalim")
    axes[0, 1].set(title="Tracking", ylabel="Error (mm)")
    axes[1, 0].set(ylabel="Unwrapped chassis yaw (deg)", xlabel="Time since route start (s)")
    yaw_limits = axes[1, 0].get_ylim()
    if yaw_limits[1]-yaw_limits[0] < 1.:
        midpoint = sum(yaw_limits)/2
        axes[1, 0].set_ylim(midpoint-.5, midpoint+.5)
    axes[1, 0].ticklabel_format(useOffset=False)
    axes[1, 1].set(ylabel="Root height (mm)", xlabel="Time since route start (s)")
    for ax in axes.flat:
        ax.grid(alpha=.2)
        ax.legend()
    fig.savefig(destination, dpi=150)
    plt.close(fig)


def compare_runs(swivel_dir, ball_dir, output_dir=None, primary_region=None, context=None, make_plot=True):
    swivel_dir, ball_dir = Path(swivel_dir).expanduser().resolve(), Path(ball_dir).expanduser().resolve()
    output_dir = Path(output_dir).expanduser().resolve() if output_dir else ball_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    report, rows = compare_trajectories(json.loads((swivel_dir/"trajectory.json").read_text()),
        json.loads((ball_dir/"trajectory.json").read_text()), primary_region, context)
    report.update({"swivel_directory": str(swivel_dir), "ball_directory": str(ball_dir)})
    if make_plot:
        plot_comparison(rows, output_dir/"comparison.png")
    (output_dir/"comparison.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("swivel_directory", type=Path)
    parser.add_argument("ball_directory", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--primary-region", choices=("motion", "first_leg", "later_legs"))
    parser.add_argument("--non-equivalent-physical-parameters", action="store_true")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    report = compare_runs(args.swivel_directory, args.ball_directory, args.output, args.primary_region,
        {"non_equivalent_physical_parameters": args.non_equivalent_physical_parameters}, not args.no_plot)
    print(json.dumps(comparison_row(report), indent=2))


if __name__ == "__main__":
    main()
