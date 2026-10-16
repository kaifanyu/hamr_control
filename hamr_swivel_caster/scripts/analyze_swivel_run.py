#!/usr/bin/env python3
"""Plot measured vehicle and passive-caster motion from a saved physical run."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path, help="trajectory.json from the run")
    parser.add_argument("--output", type=Path, help="PNG output; defaults beside report")
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    samples = report["samples"]
    if len(samples) < 2:
        parser.error("Need at least two measured samples")
    times = np.array([s["elapsed_s"] for s in samples])
    positions = np.array([s["position_m"][:2] for s in samples])
    reference = np.array([s["reference_m"][:2] for s in samples])
    points = np.array(report["configuration"]["points_m"])
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False})
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    ax = axes[0, 0]
    ax.plot(reference[:, 0], reference[:, 1], "--", color="0.5", label="Reference")
    ax.plot(positions[:, 0], positions[:, 1], color="#176d9c", label="Measured")
    ax.scatter(points[:, 0], points[:, 1], s=25, color="#b95d24")
    ax.set(xlabel="World x (m)", ylabel="World y (m)", title="Vehicle trajectory")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend()
    ax = axes[0, 1]
    ax.plot(times, 1000*np.linalg.norm(positions-reference, axis=1), color="#176d9c")
    ax.set(xlabel="Simulation elapsed time (s)", ylabel="Position error (mm)",
           title="Measured tracking error")
    for side, color in (("left", "#176d9c"), ("right", "#b95d24")):
        for column, joint in enumerate(("swivel", "wheel")):
            name = f"{side}_caster_{joint}_joint"
            values = np.array([s["joint_positions_rad"][name] for s in samples])
            # Gazebo publishes continuous joint coordinates; preserve full turns.
            axes[1, column].plot(times, np.rad2deg(values) if joint == "swivel"
                                else values/(2*np.pi), color=color, label=side.title())
    axes[1, 0].set(title="Passive swivel angle relative to chassis",
                   xlabel="Simulation elapsed time (s)", ylabel="Angle (degrees)")
    axes[1, 1].set(title="Passive wheel rotation", xlabel="Simulation elapsed time (s)",
                   ylabel="Wheel revolutions")
    for ax in axes.flat:
        ax.grid(alpha=.2)
    axes[1, 0].legend()
    axes[1, 1].legend()
    fig.suptitle("POWERTEC-style front casters — Gazebo contact simulation", fontsize=15)
    destination = args.output or args.report.with_name("trajectory_and_casters.png")
    destination.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destination, dpi=160)
    print(destination)


if __name__ == "__main__":
    main()
