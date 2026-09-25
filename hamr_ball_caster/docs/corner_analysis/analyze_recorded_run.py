#!/usr/bin/env python3
"""Read-only analysis of the already-recorded Gazebo waypoint run.

Keeps time-aligned tracking error separate from geometric corner overrun.
No ROS imports or commands. Output lives beside this script.
"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    data = json.loads(args.report.read_text())
    rows = [s for s in data["samples"] if s["phase"] == "motion"]
    t = np.array([s["trajectory_time_s"] for s in rows])
    actual = np.array([s["position_m"][:2] for s in rows])
    reference = np.array([s["reference_m"] for s in rows])
    error = np.linalg.norm(actual-reference, axis=1)
    cfg = data["configuration"]
    points = np.array(cfg["points_m"])
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    starts = np.r_[0., np.cumsum(lengths/data["speed_m_s"])]
    directions = np.diff(points, axis=0)/lengths[:, None]
    events = []
    straight = t > 2.
    for i in range(1, len(points)-1):
        at = starts[i]
        incoming, outgoing = directions[i-1], directions[i]
        angle = np.degrees(np.arccos(np.clip(incoming@outgoing, -1., 1.)))
        window = (t >= at) & (t <= at+2.)
        ids = np.flatnonzero(window)
        peak = ids[np.argmax(error[ids])]
        relative = actual[window]-points[i]
        # Signed travel beyond waypoint along the incoming direction. This is
        # geometric overrun only at a turn, not at a collinear waypoint.
        overrun = float(np.max(relative@incoming)) if angle > 1. else None
        outgoing_normal = np.array([-outgoing[1], outgoing[0]])
        cross_track = float(np.max(np.abs(relative@outgoing_normal)))
        lag = (reference[window]-actual[window])@outgoing
        events.append({"waypoint_index": i, "reference_time_s": float(at),
                       "turn_angle_deg": float(angle),
                       "peak_tracking_error_m": float(error[peak]),
                       "peak_time_after_event_s": float(t[peak]-at),
                       "maximum_incoming_direction_overrun_m": overrun,
                       "maximum_outgoing_line_cross_track_m": cross_track,
                       "maximum_outgoing_direction_lag_m": float(np.max(lag))})
        if angle > 1.:
            straight &= ~((t >= at-.15) & (t <= at+2.5))
    metrics = {"input": str(args.report.resolve()),
               "sha256": hashlib.sha256(args.report.read_bytes()).hexdigest(),
               "motion_sample_count": len(rows),
               "motion_rms_error_m": float(np.sqrt(np.mean(error**2))),
               "motion_max_error_m": float(np.max(error)),
               "settled_straight_rms_error_m": float(np.sqrt(np.mean(error[straight]**2))),
               "settled_straight_definition": "Motion after 2 s; excludes [-0.15,+2.5] s around each true corner.",
               "events": events,
               "limitations": "These samples are downsampled to approximately 16 Hz; peaks may be underestimated. Command reconstruction uses the recorded pose and reference at each sample; it is not a new controller trace. Joint velocities were not recorded in this original file."}
    (HERE/"recorded_run_metrics.json").write_text(json.dumps(metrics, indent=2)+"\n")

    # Reconstruct the demanded wheel speeds before the existing slew limiter.
    segments = np.minimum(np.searchsorted(starts, t, side="right")-1, len(directions)-1)
    v = directions[segments]*data["speed_m_s"] + cfg["position_gain_s_inv"]*(reference-actual)
    norm = np.linalg.norm(v, axis=1)
    v *= np.minimum(1., cfg["max_translation_m_s"]/np.maximum(norm, 1e-12))[:, None]
    yaw = np.array([s["base_rpy_rad"][2] for s in rows])
    forward = np.cos(yaw)*v[:, 0] + np.sin(yaw)*v[:, 1]
    omega = (-np.sin(yaw)*v[:, 0] + np.cos(yaw)*v[:, 1])/cfg["base_ahead_of_axle_m"]
    demanded = np.stack((forward-cfg["half_track_m"]*omega,
                         forward+cfg["half_track_m"]*omega), axis=1)/cfg["wheel_radius_m"]
    demanded *= np.minimum(1., cfg["max_wheel_rate_rad_s"]/np.maximum(np.max(np.abs(demanded), axis=1), 1e-12))[:, None]
    commanded = np.array([s["commands_rad_s"][:2] for s in rows])
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    ax = axes[0, 0]
    ax.plot(t, error*1000, color="#bb4d28")
    for event in events:
        turning = event["turn_angle_deg"] > 1.
        ax.axvline(event["reference_time_s"], color="#cf582f" if turning else "#238b65", alpha=.45, ls="--")
    ax.annotate("Collinear waypoint: no velocity jump", (44., 5.), (27., 39.),
                fontsize=9, arrowprops={"arrowstyle": "->", "color": "#238b65"})
    ax.set(xlabel="Trajectory time (s)", ylabel="Tracking error (mm)", title="Existing MP4 run: spikes align with direction changes")
    at = starts[1]
    window = (t >= at-.5) & (t <= at+1.5)
    ax = axes[0, 1]
    ax.plot(reference[window, 0], reference[window, 1], "--", color="gray", label="Reference")
    ax.plot(actual[window, 0], actual[window, 1], color="#147d92", label="Gazebo position")
    ax.scatter(*points[1], s=25, color="black")
    ax.set(xlabel="World X (m)", ylabel="World Y (m)", title="First corner: geometric overrun and recovery")
    ax.axis("equal")
    ax.legend()
    for index, ax in enumerate(axes[1]):
        ax.step(t[window]-at, demanded[window, index], where="post", ls="--", color="#bb4d28", label="Demand before ramp (reconstructed)")
        ax.plot(t[window]-at, commanded[window, index], color="#147d92", label="Published command after ramp")
        ax.axvline(0., color="gray", ls=":")
        ax.set(xlabel="Time after first corner (s)", ylabel="Wheel rate (rad/s)", title=f"{'Left' if index == 0 else 'Right'} wheel: 15 rad/s² command ramp")
        ax.legend(fontsize=8)
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(HERE/"recorded_run_corner_diagnosis.png", dpi=170)
    fig.savefig(HERE/"recorded_run_corner_diagnosis.pdf")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
