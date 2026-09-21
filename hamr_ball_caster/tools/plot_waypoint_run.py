#!/usr/bin/env python3
"""Export tracking plots and a compact CSV from recorded Gazebo route feedback."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    samples = report["samples"]
    points = report["configuration"]["points_m"]
    profile = report.get("trajectory_profile", "legacy")
    continuous_plan = report.get("continuous_plan", {})
    if profile == "continuous" and not continuous_plan.get("samples"):
        parser.error("Continuous report is missing continuous_plan.samples; cannot display its planned curve")
    output = args.report.parent
    fig, axes = plt.subplots(1, 2, figsize=(12, 6), gridspec_kw={"width_ratios": [1, 1.35]})
    axes[0].plot([p[0] for p in points], [p[1] for p in points], "--", color="#808994", label="Original waypoint route", linewidth=2)
    if profile == "continuous":
        curve = continuous_plan["samples"]
        axes[0].plot([s["position_m"][0] for s in curve], [s["position_m"][1] for s in curve],
                     color="#454e9e", linestyle=":", linewidth=2.4, label="Planned continuous curve")
    axes[0].plot([s["position_m"][0] for s in samples], [s["position_m"][1] for s in samples], color="#087f8c", label="Gazebo vehicle position", linewidth=1.4)
    unique = list(dict.fromkeys(tuple(p) for p in points))
    for i, (x,y) in enumerate(unique, 1):
        axes[0].scatter([x], [y], color="#17324d", s=30)
        axes[0].annotate(str(i), (x,y), xytext=(9, 6), textcoords="offset points", fontsize=11)
    axes[0].set_aspect("equal")
    axes[0].set_xlabel("World X (m)")
    axes[0].set_ylabel("World Y (m)")
    axes[0].set_title("Waypoint order: 1 → 2 → 3 → 4 → 5 → 2 → 1")
    axes[0].legend(loc="lower left", fontsize=8)
    axes[1].plot([s["elapsed_s"] for s in samples], [s["position_error_m"]*1000 for s in samples], color="#db6535")
    axes[1].set_xlabel("Elapsed simulation time (s)")
    axes[1].set_ylabel("Position tracking error (mm)")
    startup = report["startup_hold_s"]
    end = startup + report["summary"]["reference_duration_s"]
    axes[1].axvspan(0, startup, color="gray", alpha=.12, label="Startup/final hold")
    axes[1].axvspan(end, samples[-1]["elapsed_s"], color="gray", alpha=.12)
    schedule = report.get("reference_schedule_s", [])
    planned_schedule = report.get("planned_reference_schedule_s", [])
    if profile == "continuous":
        for i, corner in enumerate(continuous_plan.get("corners", [])):
            axes[1].axvspan(startup + corner["start_time_s"], startup + corner["end_time_s"],
                           color="#454e9e", alpha=.07, label="Planned corner interval" if i == 0 else None)
    else:
        for i, arrival in enumerate(schedule[1:-1]):
            axes[1].axvline(startup + arrival, color="#17324d", alpha=.3,
                           linewidth=.8, linestyle="--", label="Waypoint arrival" if i == 0 else None)
        if planned_schedule and schedule and any(abs(a-b) > .05 for a,b in zip(schedule, planned_schedule)):
            for i, arrival in enumerate(planned_schedule[1:-1]):
                axes[1].axvline(startup + arrival, color="#808994", alpha=.25,
                               linewidth=.8, linestyle=":", label="Nominal schedule" if i == 0 else None)
    axes[1].legend(loc="upper right", fontsize=8)
    for ax in axes:
        ax.grid(alpha=.2)
    summary = report["summary"]
    speed_description = "segment speed" if profile == "legacy" else "speed limit"
    fig.suptitle(f"Full vehicle with CAD ball casters — {profile} reference, "
                 f"{report['speed_m_s']:.2f} m/s {speed_description}\n"
                 f"Motion RMS error {summary['motion_rmse_m']*1000:.1f} mm; "
                 f"peak {summary['motion_max_error_m']*1000:.1f} mm; "
                 f"motion duration {summary['reference_duration_s']:.1f} s", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, .91])
    fig.savefig(output/"trajectory_tracking.png", dpi=170)
    fig.savefig(output/"trajectory_tracking.pdf")
    with (output/"trajectory.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["elapsed_s", "simulation_time_s", "phase", "motion_state",
                         "trajectory_time_s", "reference_time_s", "segment",
                         "reference_x_m", "reference_y_m", "actual_x_m", "actual_y_m",
                         "position_error_m", "base_yaw_rad", "turret_world_yaw_rad"])
        for s in samples:
            writer.writerow([s["elapsed_s"], s["simulation_time_s"], s["phase"], s.get("motion_state", ""),
                             s.get("trajectory_time_s", ""), s.get("reference_time_s", s.get("trajectory_time_s", "")),
                             s.get("segment", ""), *s["reference_m"], *s["position_m"][:2],
                             s["position_error_m"], s["base_rpy_rad"][2], s["turret_world_yaw_rad"]])
    print(output/"trajectory_tracking.png")


if __name__ == "__main__":
    main()
