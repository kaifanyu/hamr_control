#!/usr/bin/env python3
"""Plot saved Gazebo measurements; no simulator or CAD parser is required."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path,
                        default=Path(__file__).resolve().parents[1] / "docs/validation")
    args = parser.parse_args()
    cases = [("rig_dt_001.json", "Saved CAD pose, 1 ms"),
             ("rig_dt_0005.json", "Saved CAD pose, 0.5 ms"),
             ("rig_seam.json", "Seam initially down, 1 ms"),
             ("rig_pole.json", "Pole initially down, 1 ms")]
    reports = {name: json.loads((args.directory / name).read_text()) for name, _ in cases}
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True,
                             gridspec_kw={"height_ratios": [1, 1, 1.35]})
    baseline = reports[cases[0][0]]["samples"]
    for joint, label in [("rig_x_joint", "Fixture X"), ("rig_y_joint", "Fixture Y")]:
        axes[0].plot([s["time"] for s in baseline],
                     [s["positions"][joint] * 1000 for s in baseline], label=label)
    axes[0].set_ylabel("Fixture position (mm)")
    for joint, label in [("caster_carrier_joint", "Carrier"),
                         ("caster_negative_hemisphere_joint", "Negative hemisphere"),
                         ("caster_positive_hemisphere_joint", "Positive hemisphere")]:
        axes[1].plot([s["time"] for s in baseline],
                     [s["positions"][joint] for s in baseline], label=label)
    axes[1].set_ylabel("Passive joint angle (rad)")
    for filename, label in cases:
        # Exclude only the first second of initial drop; retained in raw JSON.
        samples = [s for s in reports[filename]["samples"] if s["time"] >= 1]
        axes[2].plot([s["time"] for s in samples],
                     [(0.11 + s["positions"]["rig_z_joint"]) * 1000 for s in samples],
                     label=label, linewidth=1.4)
    axes[2].set_ylabel("Ball center above floor (mm)")
    axes[2].set_xlabel("Recorded simulation time (s)")
    axes[2].set_ylim(98.7, 102.1)
    for ax in axes:
        ax.grid(alpha=.25)
        ax.legend(loc="best", fontsize=9)
        ax.set_xlim(0, 28)
    fig.suptitle("CAD ball caster: measured Gazebo motion\n"
                 "2 kg fixture load; assumed caster properties; ground friction 0.8", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, .95])
    fig.savefig(args.directory / "rig_measurements.png", dpi=180)
    fig.savefig(args.directory / "rig_measurements.pdf")
    a, b = (reports[name]["metrics"] for name, _ in cases[:2])
    comparison = {"scope": "Numerical comparison of this motion sequence, not hardware calibration",
                  "final_support_height_difference_m": abs(
                      a["phases"]["settle_final"]["terminal_z_median_m"] -
                      b["phases"]["settle_final"]["terminal_z_median_m"])}
    for phase, joint in [("x_forward", "caster_carrier_joint"),
                         ("y_forward", "caster_negative_hemisphere_joint")]:
        ratios = [abs(m["phases"][phase]["passive_joints"][joint]["delta_position_rad"] /
                      m["phases"][phase]["observed_translation_m"]) for m in (a, b)]
        comparison[phase] = {"joint": joint, "rotation_per_distance_rad_m": ratios,
                             "relative_difference_percent": 100 * abs(ratios[1] / ratios[0] - 1)}
    (args.directory / "timestep_comparison.json").write_text(json.dumps(comparison, indent=2) + "\n")
    print(json.dumps(comparison, indent=2))


if __name__ == "__main__":
    main()
