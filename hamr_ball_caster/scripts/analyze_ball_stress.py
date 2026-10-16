#!/usr/bin/env python3
"""Common trajectory metrics and passive split-ball joint activity.

The CAD ball has independent shells, a carrier, seam and polar rollers. It has
no swivel-wheel heading, and poses alone do not identify its contact manifold.
No caster-axis alignment or nominal-sphere contact-slip score is assigned.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

# Pose/time weighting deliberately matches the swivel analyzer.
# Paired regression tests check common metrics without a package dependency.
def rotation(axis, angle):
    """Rodrigues rotation, accepting either one angle or a sample vector."""
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    angle = np.asarray(angle)
    return (np.eye(3) + np.sin(angle)[..., None, None]*skew
            + (1.-np.cos(angle))[..., None, None]*(skew @ skew))

def origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        roll, pitch, yaw = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        result[:3, :3] = rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll)
    return result

def sample_weights(times, max_gap_s):
    """Voronoi time cells, excluding unsupported intervals across long gaps."""
    dt = np.diff(times)
    valid_dt = np.where(dt <= max_gap_s, dt, 0.)
    weights = np.zeros(len(times))
    weights[:-1] += valid_dt/2
    weights[1:] += valid_dt/2
    return weights

def weighted_stats(values, eligible, weights):
    valid = eligible & np.isfinite(values) & (weights > 0)
    if not valid.any():
        return {"sample_count": 0, "observed_duration_s": 0., "mean_abs": None,
                "rms": None, "p95_abs": None, "max_abs": None}
    v, w = values[valid], weights[valid]
    absolute = np.abs(v)
    order = np.argsort(absolute)
    cumulative = np.cumsum(w[order])
    p95 = absolute[order][min(np.searchsorted(cumulative, .95*cumulative[-1]), len(order)-1)]
    return {"sample_count": int(valid.sum()), "observed_duration_s": float(w.sum()),
            "mean_abs": float(np.sum(w*absolute)/w.sum()),
            "rms": float(np.sqrt(np.sum(w*v*v)/w.sum())),
            "p95_abs": float(p95), "max_abs": float(absolute.max())}

def deduplicate_samples(samples):
    """Retain the latest joint feedback for repeated odometry source stamps."""
    selected = []
    for sample in samples:
        stamp = float(sample.get("odometry_stamp_s", sample["elapsed_s"]))
        if not math.isfinite(stamp):
            raise ValueError("Non-finite odometry timestamp")
        if selected and stamp < selected[-1][0]:
            raise ValueError("Odometry timestamps move backward")
        if selected and stamp == selected[-1][0]:
            selected[-1] = (stamp, sample)
        else:
            selected.append((stamp, sample))
    if len(selected) < 3:
        raise ValueError("At least three distinct odometry timestamps are needed")
    return np.array([item[0] for item in selected]), [item[1] for item in selected]

def forward_kinematics(robot, samples, targets):
    """Full transforms through every URDF joint on the requested link paths."""
    links = {link.get("name") for link in robot.findall("link")}
    joints = {joint.find("child").get("link"): joint for joint in robot.findall("joint")}
    roots = links-set(joints)
    if len(roots) != 1:
        raise ValueError("The URDF must have one root link")
    root = roots.pop()
    positions = np.array([s["position_m"] for s in samples], dtype=float)
    rpy = np.array([s["base_rpy_rad"] for s in samples], dtype=float)
    if positions.shape != (len(samples), 3) or rpy.shape != positions.shape:
        raise ValueError("Measured root position and RPY must be XYZ vectors")
    if not np.isfinite(positions).all() or not np.isfinite(rpy).all():
        raise ValueError("Non-finite root pose")
    world = np.broadcast_to(np.eye(4), (len(samples), 4, 4)).copy()
    world[:, :3, 3] = positions
    world[:, :3, :3] = (rotation([0, 0, 1], rpy[:, 2])
                       @ rotation([0, 1, 0], rpy[:, 1])
                       @ rotation([1, 0, 0], rpy[:, 0]))
    frames = {root: world}
    visiting = set()

    def frame(link):
        if link in frames:
            return frames[link]
        if link not in joints or link in visiting:
            raise ValueError(f"Disconnected or cyclic URDF at {link}")
        visiting.add(link)
        joint = joints[link]
        parent = frame(joint.find("parent").get("link"))
        transform = np.broadcast_to(origin(joint.find("origin")), world.shape).copy()
        if joint.get("type") != "fixed":
            name = joint.get("name")
            try:
                q = np.array([s["joint_positions_rad"][name] for s in samples], dtype=float)
            except KeyError as exc:
                raise ValueError(f"Missing measured joint needed by URDF FK: {name}") from exc
            if not np.isfinite(q).all():
                raise ValueError(f"Non-finite joint position: {name}")
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            motion = np.broadcast_to(np.eye(4), world.shape).copy()
            if joint.get("type") in ("continuous", "revolute"):
                motion[:, :3, :3] = rotation(axis, q)
            elif joint.get("type") == "prismatic":
                motion[:, :3, 3] = q[:, None]*axis
            else:
                raise ValueError(f"Unsupported URDF joint type {joint.get('type')}")
            transform = transform @ motion
        frames[link] = parent @ transform
        visiting.remove(link)
        return frames[link]

    for target in targets:
        frame(target)
    return root, frames

def time_aligned_samples(samples, robot, max_gap_s):
    """Interpolate joint feedback to odometry time, without extrapolating gaps."""
    stamps = np.array([s.get("joint_state_stamp_s", np.nan) for s in samples])
    if not np.isfinite(stamps).all():
        return None
    order = np.argsort(stamps, kind="stable")
    order = order[np.r_[np.diff(stamps[order]) != 0., True]]
    stamps = stamps[order]
    if len(stamps) < 3:
        return None
    target = np.array([s.get("odometry_stamp_s", s["elapsed_s"]) for s in samples])
    supported = (target >= stamps[0]) & (target <= stamps[-1])
    for index in np.flatnonzero(np.diff(stamps) > max_gap_s):
        supported &= ~((target > stamps[index]) & (target < stamps[index+1]))
    indices = np.flatnonzero(supported)
    if len(indices) < 3:
        return None
    target = target[indices]
    interpolated = {}
    for field in ("joint_positions_rad", "joint_velocities_rad_s"):
        keys = set.intersection(*(set(s.get(field, {})) for s in samples))
        interpolated[field] = {}
        for key in keys:
            values = np.array([samples[index][field][key] for index in order])
            # Gazebo/DART records accumulated joint coordinates, not angles
            # wrapped modulo 2*pi. A fast polar roller can move >pi per sample.
            interpolated[field][key] = np.interp(target, stamps, values)
    result = []
    for offset, index in enumerate(indices):
        sample = dict(samples[index])
        sample["joint_state_stamp_s"] = float(target[offset])
        for field, columns in interpolated.items():
            sample[field] = {key: float(values[offset]) for key, values in columns.items()}
        result.append(sample)
    return result


DRIVE_JOINTS = ("left_rocker_left_wheel_joint", "right_rocker_right_wheel_joint")


def regions_for(samples, times):
    phases = np.array([s.get("phase", "unknown") for s in samples])
    segment = np.array([s.get("segment", 0) for s in samples])
    motion = phases == "motion"
    startup = np.array(["startup" in p or "align" in p for p in phases])
    initial = np.zeros(len(times), dtype=bool)
    if motion.any():
        initial = motion & (times < times[np.flatnonzero(motion)[0]]+5.)
    return {"all": np.ones(len(times), bool), "startup": startup,
            "motion": motion, "initial_motion": initial,
            "later_motion": motion & ~initial,
            "first_leg": motion & (segment == 0),
            "later_legs": motion & (segment >= 1),
            "hold": ~(startup | motion)}


def joint_rate(samples, name, times):
    if all(name in s.get("joint_velocities_rad_s", {}) for s in samples):
        values = np.array([s["joint_velocities_rad_s"][name] for s in samples])
        return values, "JointState velocity"
    if all(name in s.get("joint_positions_rad", {}) for s in samples):
        values = np.array([s["joint_positions_rad"][name] for s in samples])
        return np.gradient(values, times, edge_order=2), "position finite difference"
    return None, "missing"


def common_metrics(trajectory, max_gap_s=.25):
    """Same duration-weighted pose/command metrics for either caster model."""
    if not np.isfinite(max_gap_s) or max_gap_s <= 0:
        raise ValueError("max_gap_s must be finite and positive")
    times, samples = deduplicate_samples(trajectory["samples"])
    weights = sample_weights(times, max_gap_s)
    regions = regions_for(samples, times)
    position = np.array([s["position_m"] for s in samples], dtype=float)
    reference = np.array([s["reference_m"][:2] for s in samples], dtype=float)
    rpy = np.array([s["base_rpy_rad"] for s in samples], dtype=float)
    if not all(np.isfinite(x).all() for x in (position, reference, rpy)):
        raise ValueError("Non-finite measured/reference pose")
    yaw = np.unwrap(rpy[:, 2])
    error = np.linalg.norm(position[:, :2]-reference, axis=1)
    reference_velocity = np.array([s.get("reference_velocity_m_s", [np.nan, np.nan])[:2] for s in samples])
    reference_speed = np.linalg.norm(reference_velocity, axis=1)
    tangent_valid = np.isfinite(reference_velocity).all(axis=1) & (reference_speed > 1e-9)
    tangent = np.divide(reference_velocity, reference_speed[:, None],
                        out=np.zeros_like(reference_velocity), where=tangent_valid[:, None])
    offset_xy = position[:, :2]-reference
    along_error = np.sum(offset_xy*tangent, axis=1)
    cross_error = tangent[:, 0]*offset_xy[:, 1]-tangent[:, 1]*offset_xy[:, 0]
    velocity = np.gradient(position[:, :2], times, axis=0, edge_order=2)
    omega = np.gradient(yaw, times, edge_order=2)
    derivative_valid = np.ones(len(times), bool)
    gaps = np.flatnonzero(np.diff(times) > max_gap_s)
    derivative_valid[gaps] = derivative_valid[gaps+1] = False
    if any(gaps < 2):
        derivative_valid[0] = False
    if any(gaps >= len(times)-3):
        derivative_valid[-1] = False
    scales = np.array([s.get("wheel_speed_scale", np.nan) for s in samples])
    raw = np.array([s.get("raw_wheels_rad_s", [np.nan, np.nan])[:2] for s in samples])
    commands = np.array([s.get("commands_rad_s", [np.nan, np.nan])[:2] for s in samples])
    command_valid = np.isfinite(scales) & np.isfinite(raw).all(axis=1) & np.isfinite(commands).all(axis=1)
    speed_limited = scales < 1.-1e-6
    acceleration_limited = np.max(np.abs(raw*scales[:, None]-commands), axis=1) > 1e-6
    joint_stamps = np.array([s.get("joint_state_stamp_s", np.nan) for s in samples])
    ages = np.array([s.get("odometry_age_s", np.nan) for s in samples])
    result = {"completed": bool(trajectory.get("completed")),
              "trajectory_passed": bool(trajectory.get("summary", {}).get("passed")),
              "raw_sample_count": len(trajectory["samples"]), "sample_count": len(samples),
              "duplicate_odometry_samples_removed": len(trajectory["samples"])-len(samples),
              "max_sample_gap_s": float(np.max(np.diff(times))),
              "derivative_gap_limit_s": max_gap_s,
              "duration_s": float(times[-1]-times[0]),
              "configuration": trajectory.get("configuration", {}),
              "startup_alignment": trajectory.get("startup_alignment"),
              "requested_route_points_m": trajectory.get("requested_route_points_m", trajectory.get("configuration", {}).get("points_m")),
              "executed_route_points_m": trajectory.get("executed_route_points_m", trajectory.get("configuration", {}).get("points_m")),
              "feedback_timing": {"odometry_age_s": weighted_stats(ages, regions["all"], weights),
                  "joint_minus_odometry_stamp_s": weighted_stats(joint_stamps-times, regions["all"], weights),
                  "joint_source_stamp_sample_count": int(np.isfinite(joint_stamps).sum())},
              "regions": {}}
    rows = {"odometry_stamp_s": times, "elapsed_s": np.array([s["elapsed_s"] for s in samples]),
            "phase": np.array([s.get("phase", "unknown") for s in samples]),
            "segment": np.array([s.get("segment", 0) for s in samples]), "weight_s": weights,
            "body_x_m": position[:, 0], "body_y_m": position[:, 1],
            "reference_x_m": reference[:, 0], "reference_y_m": reference[:, 1],
            "tracking_error_m": error, "body_yaw_unwrapped_deg": np.degrees(yaw),
            "along_track_error_m": np.where(tangent_valid, along_error, np.nan),
            "cross_track_error_m": np.where(tangent_valid, cross_error, np.nan),
            "body_roll_deg": np.degrees(rpy[:, 0]), "body_pitch_deg": np.degrees(rpy[:, 1]),
            "body_height_m": position[:, 2], "body_speed_m_s": np.linalg.norm(velocity, axis=1)}
    rates = [joint_rate(samples, name, times) for name in DRIVE_JOINTS]
    cfg = result["configuration"]
    residuals = {}
    if all(rate is not None for rate, _ in rates) and all(key in cfg for key in ("wheel_radius_m", "half_track_m", "base_ahead_of_axle_m")):
        left, right = rates[0][0], rates[1][0]
        radius, half_track, offset = (cfg[key] for key in ("wheel_radius_m", "half_track_m", "base_ahead_of_axle_m"))
        residuals = {
            "axle_forward_velocity_residual_m_s": np.cos(yaw)*velocity[:, 0]+np.sin(yaw)*velocity[:, 1]-radius*(left+right)/2,
            "axle_lateral_velocity_residual_m_s": -np.sin(yaw)*velocity[:, 0]+np.cos(yaw)*velocity[:, 1]-offset*omega,
            "yaw_rate_residual_rad_s": omega-radius*(right-left)/(2*half_track),
        }
        rows.update({key: np.where(derivative_valid, val, np.nan) for key, val in residuals.items()})
    result["drive_residual"] = {"available": bool(residuals), "wheel_rate_sources": [source for _, source in rates],
        "note": "Measured pose versus ideal planar rear-drive kinematics; includes feedback timing, derivatives, suspension and contact effects. This is not motor effort or isolated caster contact slip."}
    for region, mask in regions.items():
        command_mask = mask & command_valid
        duration = float(weights[command_mask].sum())
        index = np.flatnonzero(mask)
        heading = {"initial_deg": None, "net_change_deg": None, "excursion_deg": None, "total_abs_rotation_deg": None}
        height_range = None
        if len(index):
            connected = mask[:-1] & mask[1:] & (np.diff(times) <= max_gap_s)
            heading = {"initial_deg": float(np.degrees(yaw[index[0]])),
                       "net_change_deg": float(np.degrees(yaw[index[-1]]-yaw[index[0]])),
                       "excursion_deg": float(np.degrees(np.ptp(yaw[index]))),
                       "total_abs_rotation_deg": float(np.degrees(np.abs(np.diff(yaw))[connected].sum()))}
            height_range = float(np.ptp(position[index, 2]))
        result["regions"][region] = {
            "tracking_error_m": weighted_stats(error, mask, weights),
            "along_track_error_m": weighted_stats(along_error, mask & tangent_valid, weights),
            "cross_track_error_m": weighted_stats(cross_error, mask & tangent_valid, weights),
            "body_roll_deg": weighted_stats(np.degrees(rpy[:, 0]), mask, weights),
            "body_pitch_deg": weighted_stats(np.degrees(rpy[:, 1]), mask, weights),
            "body_height_range_m": height_range, "body_heading": heading,
            "command_observed_duration_s": duration,
            "speed_limited_fraction": float(weights[command_mask & speed_limited].sum()/duration) if duration else None,
            "acceleration_limited_fraction": float(weights[command_mask & acceleration_limited].sum()/duration) if duration else None,
            "either_limited_fraction": float(weights[command_mask & (speed_limited | acceleration_limited)].sum()/duration) if duration else None,
            "drive_residual": {key: weighted_stats(val, mask & derivative_valid, weights) for key, val in residuals.items()},
        }
        valid = mask & tangent_valid & (weights > 0)
        for field, values in (("along_track_error_m", along_error), ("cross_track_error_m", cross_error)):
            result["regions"][region][field]["mean_signed"] = float(np.average(values[valid], weights=weights[valid])) if valid.any() else None
    return result, rows, samples, regions


def analyze_trajectory(trajectory, robot, max_gap_s=.25):
    common, rows, samples, regions = common_metrics(trajectory, max_gap_s)
    times = rows["odometry_stamp_s"]
    weights = rows["weight_s"]
    passive = [joint for joint in robot.findall("joint") if joint.get("type") == "continuous" and
               joint.get("name", "").startswith(("left_caster_", "right_caster_"))]
    if len(passive) != 10 or not any("hemisphere" in joint.get("name") for joint in passive):
        raise ValueError("Expected the CAD split-ball architecture with ten passive caster joints")
    activity = {}
    for joint in passive:
        name = joint.get("name")
        if not all(name in sample.get("joint_positions_rad", {}) for sample in samples):
            raise ValueError(f"Missing passive ball joint {name}")
        angles = np.array([sample["joint_positions_rad"][name] for sample in samples])
        rate, source = joint_rate(samples, name, times)
        if not np.isfinite(angles).all() or not np.isfinite(rate).all():
            raise ValueError(f"Non-finite passive joint {name}")
        dynamics = joint.find("dynamics")
        activity[name] = {"rate_source": source, "axis_in_joint_frame": [float(v) for v in joint.find("axis").get("xyz").split()],
            "damping_Nm_s_per_rad": float(dynamics.get("damping", 0)) if dynamics is not None else 0.,
            "coulomb_friction_Nm": float(dynamics.get("friction", 0)) if dynamics is not None else 0., "regions": {}}
        for region, mask in regions.items():
            connected = mask[:-1] & mask[1:] & (np.diff(times) <= max_gap_s)
            activity[name]["regions"][region] = {"rate_rad_s": weighted_stats(rate, mask, weights),
                "total_absolute_rotation_rad": float(np.abs(np.diff(angles))[connected].sum())}
        rows[name+"_position_rad"] = angles
        rows[name+"_rate_rad_s"] = rate
    targets = [side+"_caster_fork_link" for side in ("left", "right")]
    root, frames = forward_kinematics(robot, samples, targets)
    for side, target in zip(("left", "right"), targets):
        for axis, column in zip("xyz", range(3)):
            rows[side+"_ball_center_"+axis+"_m"] = frames[target][:, column, 3]
    masses = {link.get("name"): float(link.find("inertial/mass").get("value"))
              for link in robot.findall("link") if link.find("inertial/mass") is not None}
    result = {"schema_version": 1, "model": "passive_split_ball_caster", "common": common,
        "joint_activity": activity, "root_pose_frame_assumption": root,
        "caster_heading_alignment": None, "contact_slip": None,
        "mass_kg": {"whole_model": sum(masses.values()),
            **{side+"_caster": sum(value for name, value in masses.items() if name.startswith(side+"_caster_")) for side in ("left", "right")}},
        "interpretation": [
            "The horizontal carrier and independently rotating split shells/polar rollers have no single swivel-wheel forward heading. Carrier angle is not scored against the trajectory tangent.",
            "The faceted CAD shells,20mm seam and protruding polar rollers invalidate a single nominal-sphere contact point. Contact-slip values require contact manifold/force instrumentation, which this run does not provide.",
            "Joint rotation and rate document passive mechanism activity; they do not certify no-slip contact, load share, efficiency or bearing life.",
            "Gazebo/DART JointState coordinates are accumulated, not modulo-2pi. Raw joint positions are retained so polar-roller increments exceeding pi between samples are not aliased by angle unwrapping.",
            "Tracking and command-limit metrics use the same midpoint time weighting, phases and gap handling as the swivel analysis. Startup first-source changes must be considered before comparing whole-route results.",
            "Motor torque/current is not measured; the inherited rear drive is an ideal velocity servo with software rate and acceleration limits."]}
    aligned = time_aligned_samples(samples, robot, max_gap_s)
    result["source_time_alignment_diagnostic"] = {"available": aligned is not None,
        "method": "Linear joint interpolation to odometry source time; no extrapolation; raw common metrics retained"}
    if aligned is not None:
        aligned_common, _, _, _ = common_metrics({**trajectory, "samples": aligned}, max_gap_s)
        result["source_time_alignment_diagnostic"]["drive_residual_regions"] = {
            name: region["drive_residual"] for name, region in aligned_common["regions"].items()}
    return result, rows


def write_csv(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(rows)
        for values in zip(*rows.values()):
            writer.writerow(["" if isinstance(v, (float, np.floating)) and not np.isfinite(v) else v for v in values])


def plot_analysis(rows, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), constrained_layout=True)
    t = rows["elapsed_s"]
    moving = rows["phase"] == "motion"
    axes[0, 0].plot(rows["reference_x_m"][moving], rows["reference_y_m"][moving], "k--", label="Executed reference")
    axes[0, 0].plot(rows["body_x_m"], rows["body_y_m"], label="Measured")
    axes[0, 0].set(xlabel="World X (m)", ylabel="World Y (m)", title="Passive split-ball trajectory")
    axes[0, 0].set_aspect("equal", adjustable="datalim")
    axes[0, 0].legend()
    axes[0, 1].plot(t[moving], rows["tracking_error_m"][moving]*1000)
    axes[0, 1].set(ylabel="Motion error (mm)")
    axes[1, 0].plot(t, rows["body_yaw_unwrapped_deg"])
    axes[1, 0].set(ylabel="Unwrapped chassis yaw (deg)")
    yaw_limits = axes[1, 0].get_ylim()
    if yaw_limits[1]-yaw_limits[0] < 1.:
        midpoint = sum(yaw_limits)/2
        axes[1, 0].set_ylim(midpoint-.5, midpoint+.5)
    axes[1, 0].ticklabel_format(useOffset=False)
    for kind in ("roll", "pitch"):
        axes[1, 1].plot(t, rows["body_"+kind+"_deg"], label=kind)
    axes[1, 1].set(ylabel="Body tilt (deg)")
    axes[1, 1].legend()
    for side, ax in zip(("left", "right"), axes[2]):
        prefix = side+"_caster_"
        for key, values in rows.items():
            if key.startswith(prefix) and key.endswith("_rate_rad_s"):
                ax.plot(t, values, label=key[len(prefix):-len("_joint_rate_rad_s")])
        ax.set(ylabel=side+" passive rates (rad/s)", xlabel="Elapsed simulation time (s)")
        ax.legend(fontsize=7)
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.savefig(destination, dpi=150)
    plt.close(fig)


def analyze_run(run_dir, max_gap_s=.25, make_plot=True):
    run_dir = Path(run_dir).expanduser().resolve()
    trajectory_path, robot_path = run_dir/"trajectory.json", run_dir/"robot.urdf"
    result, rows = analyze_trajectory(json.loads(trajectory_path.read_text()), ET.parse(robot_path).getroot(), max_gap_s)
    result.update({"run_directory": str(run_dir), "trajectory_sha256": hashlib.sha256(trajectory_path.read_bytes()).hexdigest(),
                   "robot_sha256": hashlib.sha256(robot_path.read_bytes()).hexdigest()})
    write_csv(run_dir/"ball_stress.csv", rows)
    if make_plot:
        plot_analysis(rows, run_dir/"ball_stress.png")
    (run_dir/"ball_stress.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    result = analyze_run(args.run_directory, make_plot=not args.no_plot)
    print(json.dumps(result["common"]["regions"]["motion"], indent=2))


if __name__ == "__main__":
    main()
