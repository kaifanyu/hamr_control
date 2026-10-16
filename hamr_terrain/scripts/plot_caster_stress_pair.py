#!/usr/bin/env python3
"""Plot measured ball/swivel runs: BALL_RUN SWIVEL_RUN OUTPUT_PREFIX.

Writes SVG, 300 dpi PNG, and JSON provenance beside the chosen prefix. Reads
existing trajectory/configuration/model snapshots; never starts a simulator.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

_spec = importlib.util.spec_from_file_location(
    "caster_stress_metrics", Path(__file__).with_name("analyze_caster_stress.py"))
analysis = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(analysis)

COLORS = {"ball": "#0072B2", "swivel": "#D55E00"}
SIDES = {"left": "-", "right": "--"}


def planned_events(route):
    """Events describe the reference's travel direction, never prescribed yaw."""
    schedule = route.get("stress_reference", {}).get("stop_schedule", [])
    events = []
    for leg, following in zip(schedule, schedule[1:]):
        start = analysis.number(leg.get("motion_end_time_s"))
        end = analysis.number(following.get("start_time_s"))
        before = analysis.number(leg.get("travel_yaw_rad"))
        after = analysis.number(following.get("travel_yaw_rad"))
        if not np.isfinite([start, end, before, after]).all() or end < start:
            continue
        turn = math.degrees(math.atan2(math.sin(after-before), math.cos(after-before)))
        kind = "reversal" if abs(abs(turn)-180.) < 1e-5 else "corner" if abs(turn) > 1e-5 else "stop"
        events.append({"kind": kind, "stop_start_s": start, "restart_s": end,
                       "reference_direction_change_deg": turn})
    round_trip = route.get("out_and_back")
    if round_trip:
        total = round_trip["return_end_s"]
        mirrored = [{"kind":event["kind"],"stop_start_s":total-event["restart_s"],
                     "restart_s":total-event["stop_start_s"],
                     "reference_direction_change_deg":-event["reference_direction_change_deg"]}
                    for event in reversed(events)]
        events += [{"kind":"reversal", "stop_start_s":round_trip["outbound_end_s"],
                    "restart_s":round_trip["return_start_s"],"reference_direction_change_deg":180.}]
        events += mirrored
    return events


def unwrap_contiguous(values, times, valid, max_gap):
    result = np.full(len(times), np.nan)
    start = 0
    while start < len(times):
        if not valid[start] or not np.isfinite(values[start]):
            start += 1
            continue
        end = start+1
        while (end < len(times) and valid[end] and np.isfinite(values[end])
               and 0 < times[end]-times[end-1] <= max_gap):
            end += 1
        result[start:end] = np.unwrap(values[start:end])
        start = end
    return result


def gap_arrays(times, values, max_gap):
    """Insert a break; never draw across missing intervals or restarted clocks."""
    x, y = [], []
    for index, (time, value) in enumerate(zip(times, values)):
        if index and (not 0 < time-times[index-1] <= max_gap):
            x.append(np.nan); y.append(np.nan)
        x.append(time); y.append(value)
    return np.asarray(x), np.asarray(y)


def source_joint_channels(samples, times, field, max_gap):
    # Reuse source-time interpolation without modifying or unwrapping coordinates.
    mapped = [{"joint_state_stamp_s": s.get("joint_state_stamp_s"),
               "joint_positions_rad": s.get(field, {})} for s in samples]
    try:
        return analysis.interpolate_joints(mapped, times, max_gap), "joint-source timestamps interpolated to odometry"
    except ValueError:
        names = set().union(*(set(s.get(field, {})) for s in samples)) if samples else set()
        return {name: np.array([analysis.number(s.get(field, {}).get(name)) for s in samples]) for name in names}, "odometry-sampled joint values; source timestamps unavailable"


def planar_center_traces(robot_path, samples, times, position, rpy, motion, max_gap, min_speed):
    """The analyzer's full-FK planar wheel-center proxy, retained as time traces."""
    if not robot_path.is_file():
        return {}, "Actual robot.urdf snapshot unavailable"
    try:
        robot = ET.parse(robot_path).getroot()
        joints = {j.find("child").get("link"): j for j in robot.findall("joint")}
        roots = {l.get("name") for l in robot.findall("link")}-set(joints)
        if len(roots) != 1:
            raise ValueError("Expected one URDF root")
        root = roots.pop()
        coordinates = analysis.interpolate_joints(samples, times, max_gap)
        output = {}
        for target in ("left_wheel_link", "right_wheel_link", "left_caster_wheel_link", "right_caster_wheel_link"):
            if target not in joints:
                continue
            chain, link = [], target
            while link != root:
                if link not in joints or len(chain) > len(joints):
                    raise ValueError("Invalid URDF chain")
                joint = joints[link]
                axis = joint.find("axis")
                chain.append((joint.get("name"), joint.get("type"), analysis.origin(joint.find("origin")),
                              np.fromstring(axis.get("xyz", "1 0 0"), sep=" ") if axis is not None else np.array([1., 0., 0.])))
                link = joint.find("parent").get("link")
            chain.reverse()
            needed = [name for name, kind, _, _ in chain if kind != "fixed"]
            if any(name not in coordinates for name in needed):
                continue
            center = np.full((len(times), 3), np.nan)
            axle = center.copy()
            for i in range(len(times)):
                if not np.isfinite(np.r_[position[i], rpy[i], [coordinates[name][i] for name in needed]]).all():
                    continue
                transform = np.eye(4)
                transform[:3, 3] = position[i]
                roll, pitch, yaw = rpy[i]
                transform[:3, :3] = (analysis.rotation([0, 0, 1], yaw)
                                     @ analysis.rotation([0, 1, 0], pitch)
                                     @ analysis.rotation([1, 0, 0], roll))
                for name, kind, fixed, axis in chain:
                    transform = transform@fixed
                    if kind in ("continuous", "revolute"):
                        transform[:3, :3] = transform[:3, :3]@analysis.rotation(axis, coordinates[name][i])
                    elif kind == "prismatic":
                        transform[:3, 3] += transform[:3, :3]@axis*coordinates[name][i]
                center[i], axle[i] = transform[:3, 3], transform[:3, :3]@chain[-1][3]
            velocity = analysis.derivative(center, times, max_gap)
            axis_length = np.linalg.norm(axle[:, :2], axis=1)
            speed = np.linalg.norm(velocity[:, :2], axis=1)
            transverse = np.sum(velocity[:, :2]*axle[:, :2], axis=1)/np.maximum(axis_length, 1e-12)
            valid = motion & (speed >= min_speed) & (axis_length > .2)
            output[target] = np.where(valid, transverse, np.nan)
        return output, "Planar wheel-center velocity projected onto axle XY; not terrain-contact slip"
    except (ValueError, KeyError, ET.ParseError, OSError) as error:
        return {}, str(error)


def passive_activity(rates):
    """RMS rate within named joint groups; not energy, heading or wheel speed."""
    groups = {"Carrier (2 joints)": "carrier_joint",
              "Hemispheres (4 joints)": "hemisphere_joint",
              "Polar rollers (4 joints)": "polar_roller_joint"}
    result = {}
    for label, suffix in groups.items():
        names = sorted(name for name in rates if "caster" in name and name.endswith(suffix))
        expected = 2 if suffix == "carrier_joint" else 4
        if len(names) == expected:
            # Missing channels remain unavailable; do not change the denominator.
            result[label] = np.sqrt(np.mean(np.asarray([rates[n] for n in names])**2, axis=0))
    return result


def load_run(directory, expected_caster, max_gap, min_speed):
    result, (samples, times, config) = analysis.analyze_run(directory, max_gap, min_speed)
    if result["caster"] != expected_caster:
        raise ValueError(f"Expected {expected_caster} run, found {result['caster']} in {directory}")
    n = len(times)
    x = np.array([analysis.number(s.get("trajectory_time_s")) for s in samples])
    motion = np.array([s.get("phase") == "motion" for s in samples]) & np.isfinite(x)
    position = np.array([analysis.vector(s, ("position_m",), 3) for s in samples]).reshape(n, 3)
    rpy = np.array([analysis.vector(s, ("base_rpy_rad", "rpy_rad"), 3) for s in samples]).reshape(n, 3)
    reference = np.array([analysis.vector(s, ("reference_m", "reference_position_m", "reference_xy_m"), 2) for s in samples]).reshape(n, 2)
    proxy, proxy_note = planar_center_traces(Path(directory)/"robot.urdf", samples, times, position, rpy, motion, max_gap, min_speed)
    joints, joint_basis = source_joint_channels(samples, times, "joint_positions_rad", max_gap)
    rates, _ = source_joint_channels(samples, times, "joint_velocities_rad_s", max_gap)
    return {"result": result, "config": config, "samples": samples, "times": times, "x": x, "motion": motion,
            "error_mm": 1000*np.linalg.norm(position[:, :2]-reference, axis=1), "rpy": rpy,
            "yaw_deg": np.degrees(unwrap_contiguous(rpy[:, 2], times, motion, max_gap)),
            "joints": joints, "joint_rates": rates, "joint_time_basis": joint_basis,
            "planar_proxy": proxy, "planar_proxy_note": proxy_note}


def plot_pair(ball_directory, swivel_directory, output_prefix, max_gap=.25, min_speed=.03, title=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runs = {"ball": load_run(ball_directory, "ball", max_gap, min_speed),
            "swivel": load_run(swivel_directory, "swivel", max_gap, min_speed)}
    mismatch = [key for key in runs["ball"]["result"]["condition"]
                if runs["ball"]["result"]["condition"][key] != runs["swivel"]["result"]["condition"][key]]
    missing_conditions = sorted({key for run in runs.values() for key, value in run["result"]["condition"].items() if value is None})
    routes = [r["config"].get("route", {}) for r in runs.values()]
    events = planned_events(routes[0]) if routes[0] == routes[1] else []
    output_prefix = Path(output_prefix).expanduser().resolve()
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    terrain = runs["ball"]["result"]["terrain"]
    shape = runs["ball"]["result"]["shape"]
    observed_ends = [float(np.max(r["x"][r["motion"]])) for r in runs.values() if r["motion"].any()]
    requested_ends = [analysis.number(route.get("reference_duration_s")) for route in routes]
    xmax = max([1., *observed_ends, *[t for t in requested_ends if np.isfinite(t)]])
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.titlesize": 10,
                         "axes.labelsize": 9, "legend.fontsize": 8, "svg.fonttype": "none",
                         "axes.spines.top": False, "axes.spines.right": False,
                         "savefig.facecolor": "white", "lines.linewidth": 1.25})
    fig, grid = plt.subplots(4, 2, figsize=(12.2, 11.8), sharex=True)
    axes = grid.ravel()
    titles = ["(a) Shared XY tracking", "(b) Measured chassis heading",
              "(c) Shared body attitude", "(d) Drive software command limits",
              "(e) Ordinary caster swivel coordinates", "(f) Ordinary front-wheel planar diagnostic",
              "(g) Split-ball passive joint activity", "(h) Shared rear-wheel planar diagnostic"]
    for axis, heading in zip(axes, titles):
        axis.set_title(heading, loc="left", fontweight="semibold", pad=8)
        axis.set_xlim(0, xmax)
        axis.grid(color="0.87", linewidth=.6, zorder=0)
        for event in events:
            axis.axvspan(event["stop_start_s"], event["restart_s"], color="0.88", alpha=.65, zorder=0)
            axis.axvline(event["restart_s"], color="0.48", linewidth=.75, linestyle=":", zorder=1)
    def trace(axis, run, values, **style):
        values = np.where(run["motion"], values, np.nan)
        if np.isfinite(values).any():
            x, y = gap_arrays(run["x"], values, max_gap)
            axis.plot(x, y, **style)
            return True
        return False
    for caster, run in runs.items():
        label, color = ("Split ball" if caster == "ball" else "Swivel"), COLORS[caster]
        trace(axes[0], run, run["error_mm"], color=color, label=label)
        trace(axes[1], run, run["yaw_deg"], color=color, label=label)
        trace(axes[2], run, np.degrees(run["rpy"][:, 0]), color=color, label=label+" roll")
        trace(axes[2], run, np.degrees(run["rpy"][:, 1]), color=color, linestyle="--", label=label+" pitch")
        rear = run["planar_proxy"]
        if all(name in rear for name in ("left_wheel_link", "right_wheel_link")):
            rms = np.sqrt((rear["left_wheel_link"]**2+rear["right_wheel_link"]**2)/2)
            trace(axes[7], run, 1000*rms, color=color, label=label)
        if run["result"]["status"] != "complete" and run["motion"].any():
            end = float(max(run["x"][run["motion"]]))
            axes[0].axvline(end, color=color, linestyle="--", linewidth=.8)
            axes[0].annotate(label+" data ends", (end, .98 if caster == "ball" else .86),
                             xycoords=("data", "axes fraction"), xytext=(-3, 0), textcoords="offset points",
                             ha="right", va="top", color=color, fontsize=7)
    axes[0].set_ylabel("Euclidean XY error (mm)")
    axes[0].set_ylim(bottom=0)
    for event in events:
        axes[0].text(event["restart_s"]+.01*xmax, .96, "planned "+event["kind"],
                     transform=axes[0].get_xaxis_transform(), rotation=90, fontsize=7,
                     ha="left", va="top", color="0.38")
    axes[1].set_ylabel("World yaw (deg, locally unwrapped)")
    axes[2].set_ylabel("Roll / pitch (deg)")
    lane_labels = []
    for model_index, (caster, run) in enumerate(runs.items()):
        for flag_index, flag in enumerate(("translation", "wheel_rate", "wheel_acceleration")):
            lane = 5-(model_index*3+flag_index)
            lane_labels.insert(0, ("Ball " if caster == "ball" else "Swivel ")+flag.replace("wheel_", "").replace("_", " "))
            raw = [s.get("command_limits", {}).get(flag) for s in run["samples"]]
            values = np.array([float(v) if isinstance(v, bool) else np.nan for v in raw])
            if not trace(axes[3], run, lane+.65*values, color=COLORS[caster], drawstyle="steps-post"):
                axes[3].text(.99, (lane+.35)/6, "unavailable", transform=axes[3].transAxes,
                             ha="right", va="center", color="0.5", fontsize=7)
    axes[3].set_ylim(-.1, 6.)
    axes[3].set_yticks(np.arange(6)+.3, lane_labels, fontsize=7)
    axes[3].text(.99, 1.01, "raised = limited", transform=axes[3].transAxes, ha="right", fontsize=7)
    yaw_boundary_labels = []
    for caster, run in runs.items():
        fraction = run["result"]["metrics"]["command_saturation"].get("yaw_gimbal_at_rate_boundary_fraction")
        value = f"{100*fraction:.1f}%" if fraction is not None else "unavailable"
        yaw_boundary_labels.append(("Ball " if caster == "ball" else "Swivel ")+value)
    axes[3].text(.99, -.09, "Separate yaw-gimbal boundary occupancy: "+"; ".join(yaw_boundary_labels),
                 transform=axes[3].transAxes, ha="right", va="top", fontsize=6.8)
    swivel = runs["swivel"]
    for side, style in SIDES.items():
        coordinate = swivel["joints"].get(side+"_caster_swivel_joint")
        if coordinate is not None:
            trace(axes[4], swivel, np.degrees(coordinate), color=COLORS["swivel"], linestyle=style, label=side.title())
        proxy = swivel["planar_proxy"].get(side+"_caster_wheel_link")
        if proxy is not None:
            trace(axes[5], swivel, 1000*proxy, color=COLORS["swivel"], linestyle=style, label=side.title())
    axes[4].set_ylabel("Accumulated swivel angle (deg)")
    axes[5].set_ylabel("Signed transverse XY velocity (mm/s)")
    threshold_labels = []
    for side in ("left", "right"):
        entry = swivel["result"]["wheel_center_proxy"].get("wheels", {}).get(side+"_caster_wheel_link", {})
        fraction = entry.get("planar_axis_over_15_deg_fraction")
        longest = entry.get("planar_axis_over_15_deg_longest_episode_s")
        if fraction is not None and longest is not None:
            threshold_labels.append(f"{side.title()}: {100*fraction:.1f}% above 15°; longest {longest:.2f} s")
    if threshold_labels:
        axes[5].text(.98, .98, "Planar axis diagnostic\n"+"\n".join(threshold_labels),
                     transform=axes[5].transAxes, ha="right", va="top", fontsize=6.8,
                     bbox={"facecolor":"white", "edgecolor":"none", "alpha":.94})
    for (label, values), color in zip(passive_activity(runs["ball"]["joint_rates"]).items(),
                                     ("#0072B2", "#009E73", "#CC79A7")):
        trace(axes[6], runs["ball"], values, color=color, label=label)
    axes[6].set_yscale("symlog", linthresh=.1)
    axes[6].set_ylim(bottom=0)
    axes[6].set_ylabel("Group RMS joint rate (rad/s)\nsymlog; linear below 0.1")
    axes[7].set_ylabel("Transverse XY velocity (mm/s)\nRMS across left/right rear wheels")
    axes[7].set_ylim(bottom=0)
    for index, axis in enumerate(axes):
        if index == 3:
            continue
        handles, labels = axis.get_legend_handles_labels()
        if handles:
            # Keep panel (f)'s legend opposite the upper-right threshold summary.
            axis.legend(loc="lower left" if index == 5 else "best", frameon=True, framealpha=.88, edgecolor="0.9", ncol=2 if index in (2, 4, 5) else 1)
        else:
            axis.text(.5, .5, "Required measured channels unavailable", transform=axis.transAxes,
                      ha="center", va="center", color="0.5", fontsize=8)
    for axis in axes[-2:]:
        axis.set_xlabel("Time since trajectory start (simulation s)")
    statuses = "   |   ".join(f"{'Split ball' if c == 'ball' else 'Swivel'}: {r['result']['status'].replace('_', ' ')}" for c, r in runs.items())
    audit_note = ("Condition mismatch: "+", ".join(mismatch) if mismatch else
                  "Incomplete condition metadata; see JSON audit" if missing_conditions else
                  "Matched saved route and controller conditions")
    fig.suptitle(title or f"{terrain.replace('_', ' ')} — {shape.replace('_', ' ')}", fontsize=13, y=.987, fontweight="semibold")
    fig.text(.5, .961, statuses+"\n"+audit_note, ha="center", va="top", fontsize=8)
    event_note = ("Gray bands: planned dwell; dotted lines: planned travel-direction change (not prescribed chassis yaw)."
                  if events else "No discrete reference stop schedule shared by both runs; no inferred event markers.")
    joint_note = "" if all(r["joint_time_basis"].startswith("joint-source") for r in runs.values()) else " Joint timestamp gaps/absence are documented in JSON."
    footer = (event_note+"\nMotion phase only; gaps are not bridged. Front and rear planar diagnostics omit contact-point velocity and terrain normal; they are not contact slip.\n"
              "Yaw-gimbal boundary occupancy at ±2.5 rad/s cannot establish torque saturation. Panel (d) flags cover drive commands only.\n"
              "Split-ball activity is RMS within each joint group, not heading or dissipated power. Existing assemblies differ in mass, size, inertia, damping and contact geometry."+joint_note)
    fig.text(.065, .017, footer, ha="left", va="bottom", fontsize=7.1, linespacing=1.45)
    fig.subplots_adjust(left=.085, right=.985, bottom=.10, top=.90, hspace=.40, wspace=.30)
    svg, png = Path(str(output_prefix)+".svg"), Path(str(output_prefix)+".png")
    fig.savefig(svg)
    fig.savefig(png, dpi=300)
    plt.close(fig)
    report = {"figure_source_sha256": analysis.sha256(__file__),
              "analyzer_source_sha256": analysis.sha256(analysis.__file__),
              "settings": {"max_gap_s": max_gap, "minimum_wheel_center_speed_m_s": min_speed},
              "condition_mismatches": mismatch, "missing_condition_fields": missing_conditions, "planned_events": events,
              "figure_files": {"svg": str(svg), "png": str(png)},
              "runs": {c: {"directory": r["result"]["run_directory"], "status": r["result"]["status"],
                            "physical_failure": r["result"]["physical_failure"],
                            "infrastructure_failure": r["result"]["infrastructure_failure"],
                            "input_hashes": r["result"]["input_hashes"], "sample_quality": r["result"]["sample_quality"],
                            "joint_time_basis": r["joint_time_basis"], "planar_proxy_note": r["planar_proxy_note"],
                            "command_saturation": r["result"]["metrics"]["command_saturation"],
                            "planar_proxy_summary": r["result"]["wheel_center_proxy"],
                            "planar_proxy_links": sorted(r["planar_proxy"])} for c, r in runs.items()},
              "scope": footer}
    Path(str(output_prefix)+".json").write_text(json.dumps(analysis.finite_json(report), indent=2, allow_nan=False)+"\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ball_run", type=Path)
    parser.add_argument("swivel_run", type=Path)
    parser.add_argument("output_prefix", type=Path)
    parser.add_argument("--max-gap", type=float, default=.25)
    parser.add_argument("--min-moving-speed", type=float, default=.03)
    parser.add_argument("--title")
    args = parser.parse_args(argv)
    if args.max_gap <= 0 or args.min_moving_speed <= 0:
        parser.error("Thresholds must be positive")
    result = plot_pair(args.ball_run.expanduser(), args.swivel_run.expanduser(), args.output_prefix,
                       args.max_gap, args.min_moving_speed, args.title)
    print(json.dumps(result["figure_files"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
