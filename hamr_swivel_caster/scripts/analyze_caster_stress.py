#!/usr/bin/env python3
"""Analyze measured caster alignment and planar slip proxies using URDF FK.

Usage: analyze_caster_stress.py RUN_DIRECTORY [--min-speed .03] [--no-plot]
The directory must contain trajectory.json and the actual expanded robot.urdf.
Angles are descriptive: a leading caster can roll backward without lateral slip,
and a trailing caster on a curve need not point along the chassis path tangent.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


SIDES = ("left", "right")
ANGLE_THRESHOLDS_DEG = (15., 30., 60., 150.)


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


def physical_parameters(robot, side):
    """Read applied inertias and friction from the expanded artifact, not defaults."""
    result = {"components": {}, "joint_dynamics": {}}
    for part in ("mount", "fork", "wheel"):
        inertial = robot.find(f"./link[@name='{side}_caster_{part}_link']/inertial")
        if inertial is not None:
            mass = inertial.find("mass")
            tensor = inertial.find("inertia")
            result["components"][part] = {
                "mass_kg": float(mass.get("value")) if mass is not None else None,
                "inertia_kg_m2": {key: float(value) for key, value in tensor.attrib.items()} if tensor is not None else None,
            }
    for kind in ("swivel", "wheel"):
        dynamics = robot.find(f"./joint[@name='{side}_caster_{kind}_joint']/dynamics")
        result["joint_dynamics"][kind] = {
            "damping_Nm_s_per_rad": float(dynamics.get("damping", 0.)) if dynamics is not None else 0.,
            "coulomb_friction_Nm": float(dynamics.get("friction", 0.)) if dynamics is not None else 0.,
        }
    friction = robot.find(f"./gazebo[@reference='{side}_caster_wheel_link']/mu1")
    result["tire_mu"] = float(friction.get("value", friction.text)) if friction is not None else None
    return result


def unit_xy(values):
    values = np.asarray(values)[..., :2]
    lengths = np.linalg.norm(values, axis=-1)
    return values / np.maximum(lengths[..., None], 1e-15), lengths


def angle_degrees(direction, velocity):
    """Signed directed and unsigned undirected angles; zero speed is NaN."""
    a, alen = unit_xy(direction)
    b, blen = unit_xy(velocity)
    directed = np.degrees(np.arctan2(a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0],
                                    np.sum(a*b, axis=1)))
    directed[(alen < 1e-12) | (blen < 1e-12)] = np.nan
    undirected = np.minimum(np.abs(directed), 180.-np.abs(directed))
    return directed, undirected


def sample_weights(times, max_gap_s):
    """Voronoi time cells, excluding unsupported intervals across long gaps."""
    dt = np.diff(times)
    valid_dt = np.where(dt <= max_gap_s, dt, 0.)
    weights = np.zeros(len(times))
    weights[:-1] += valid_dt/2
    weights[1:] += valid_dt/2
    return weights


def secant_velocity(values, times, window_s=.1):
    """A wider derivative for sensitivity analysis, not a contact-slip truth."""
    before = np.maximum(times[0], times-window_s/2)
    after = np.minimum(times[-1], times+window_s/2)
    return np.column_stack([(np.interp(after, times, values[:, column])
                             - np.interp(before, times, values[:, column]))/(after-before)
                            for column in range(values.shape[1])])


def longest_episode(mask, times, weights, max_gap_s):
    longest = current = 0.
    for index, active in enumerate(mask):
        if not active:
            current = 0.
            continue
        if index and times[index]-times[index-1] > max_gap_s:
            current = 0.
        current += weights[index]
        longest = max(longest, current)
    return float(longest)


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


def alignment_stats(angles, eligible, times, weights, max_gap_s):
    result = weighted_stats(angles, eligible, weights)
    valid = eligible & np.isfinite(angles) & (weights > 0)
    denominator = weights[valid].sum()
    result["units"] = "degrees"
    result["exceeds"] = {}
    for threshold in ANGLE_THRESHOLDS_DEG:
        mask = valid & (np.abs(angles) > threshold)
        duration = float(weights[mask].sum())
        result["exceeds"][str(int(threshold))] = {
            "duration_s": duration,
            "fraction_of_moving_time": duration/denominator if denominator else None,
            "longest_episode_s": longest_episode(mask, times, weights, max_gap_s),
        }
    return result


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
    joint_types = {joint.get("name"): joint.get("type") for joint in robot.findall("joint")}
    interpolated = {}
    for field in ("joint_positions_rad", "joint_velocities_rad_s"):
        keys = set.intersection(*(set(s.get(field, {})) for s in samples))
        interpolated[field] = {}
        for key in keys:
            values = np.array([samples[index][field][key] for index in order])
            if field == "joint_positions_rad" and joint_types.get(key) == "continuous":
                values = np.unwrap(values)
            interpolated[field][key] = np.interp(target, stamps, values)
    result = []
    for offset, index in enumerate(indices):
        sample = dict(samples[index])
        sample["joint_state_stamp_s"] = float(target[offset])
        for field, columns in interpolated.items():
            sample[field] = {key: float(values[offset]) for key, values in columns.items()}
        result.append(sample)
    return result


def analyze_trajectory(trajectory, robot, min_speed=.03, max_gap_s=.25,
                       initial_motion_window_s=5., _time_alignment=True):
    """Return a JSON-ready summary and arrays for CSV/plot output."""
    if not math.isfinite(min_speed) or min_speed <= 0 or not math.isfinite(max_gap_s) or max_gap_s <= 0:
        raise ValueError("Speed and gap thresholds must be finite and positive")
    if not math.isfinite(initial_motion_window_s) or initial_motion_window_s <= 0:
        raise ValueError("Initial motion window must be finite and positive")
    raw_samples = trajectory.get("samples", [])
    times, samples = deduplicate_samples(raw_samples)
    weights = sample_weights(times, max_gap_s)
    targets = [f"{side}_caster_{part}_link" for side in SIDES for part in ("fork", "wheel")]
    root, frames = forward_kinematics(robot, samples, targets)
    position = np.array([s["position_m"] for s in samples])
    reference = np.array([s["reference_m"][:2] for s in samples])
    tangent = np.array([s.get("reference_velocity_m_s", [0., 0.])[:2] for s in samples], dtype=float)
    if not np.isfinite(reference).all() or not np.isfinite(tangent).all():
        raise ValueError("Non-finite reference position or velocity")
    yaw = np.unwrap(np.array([s["base_rpy_rad"][2] for s in samples]))
    yaw_rate = np.gradient(yaw, times, edge_order=2)
    body_velocity = np.gradient(position[:, :2], times, axis=0, edge_order=2)
    phase = np.array([str(s.get("phase", "unknown")) for s in samples])
    segment = np.array([s.get("segment", 0) for s in samples])
    startup = np.array(["startup" in p or "align" in p for p in phase])
    moving_phase = phase == "motion"
    initial_motion = np.zeros(len(samples), dtype=bool)
    if moving_phase.any():
        motion_started = times[np.flatnonzero(moving_phase)[0]]
        initial_motion = moving_phase & (times < motion_started+initial_motion_window_s)
    regions = {"all": np.ones(len(samples), dtype=bool), "startup": startup,
               "motion": moving_phase, "initial_motion": initial_motion,
               "later_motion": moving_phase & ~initial_motion,
               "first_leg": moving_phase & (segment == 0),
               "later_legs": moving_phase & (segment != 0),
               "hold": ~(startup | moving_phase)}
    derivative_valid = np.ones(len(samples), dtype=bool)
    gaps = np.flatnonzero(np.diff(times) > max_gap_s)
    derivative_valid[gaps] = False
    derivative_valid[gaps+1] = False
    if any(index < 2 for index in gaps):
        derivative_valid[0] = False
    if any(index >= len(times)-3 for index in gaps):
        derivative_valid[-1] = False
    reference_speed = np.linalg.norm(tangent, axis=1)
    tracking_error = np.linalg.norm(position[:, :2]-reference, axis=1)
    odometry_age = np.array([s.get("odometry_age_s", np.nan) for s in samples], dtype=float)
    joint_stamps = np.array([s.get("joint_state_stamp_s", np.nan) for s in samples], dtype=float)
    joint_age = np.array([s.get("joint_state_age_s", np.nan) for s in samples], dtype=float)
    joint_skew = joint_stamps-times
    speed_scales = np.array([s.get("wheel_speed_scale", np.nan) for s in samples])
    raw_wheels = np.array([s.get("raw_wheels_rad_s", [np.nan, np.nan])[:2] for s in samples])
    wheel_commands = np.array([s.get("commands_rad_s", [np.nan, np.nan])[:2] for s in samples])
    command_valid = np.isfinite(speed_scales) & np.isfinite(raw_wheels).all(axis=1) & np.isfinite(wheel_commands).all(axis=1)
    speed_limited = speed_scales < 1.-1e-6
    acceleration_limited = np.max(np.abs(raw_wheels*speed_scales[:, None]-wheel_commands), axis=1) > 1e-6
    command_statistics = trajectory.get("command_statistics", {})
    updates = command_statistics.get("updates", 0)
    actuator_limits = {"reported_all_phase_update_counts": command_statistics,
                       "all_phase_speed_limited_update_fraction": command_statistics.get("speed_limited_updates", 0)/updates if updates else None,
                       "all_phase_acceleration_limited_update_fraction": command_statistics.get("acceleration_limited_updates", 0)/updates if updates else None,
                       "sampled_regions": {}}
    for region, mask in regions.items():
        valid = mask & command_valid
        duration = weights[valid].sum()
        actuator_limits["sampled_regions"][region] = {
            "observed_duration_s": float(duration),
            "speed_limited_fraction": float(weights[valid & speed_limited].sum()/duration) if duration else None,
            "acceleration_limited_fraction": float(weights[valid & acceleration_limited].sum()/duration) if duration else None,
            "either_limited_fraction": float(weights[valid & (speed_limited | acceleration_limited)].sum()/duration) if duration else None,
        }
    rows = {
        "odometry_stamp_s": times, "elapsed_s": np.array([s["elapsed_s"] for s in samples]),
        "phase": phase, "segment": segment, "weight_s": weights,
        "body_x_m": position[:, 0], "body_y_m": position[:, 1],
        "reference_x_m": reference[:, 0], "reference_y_m": reference[:, 1],
        "body_yaw_unwrapped_deg": np.degrees(yaw), "body_yaw_rate_rad_s": yaw_rate,
        "body_speed_m_s": np.linalg.norm(body_velocity, axis=1),
        "reference_speed_m_s": reference_speed, "tracking_error_m": tracking_error,
        "odometry_age_s": odometry_age, "joint_state_stamp_s": joint_stamps,
        "joint_state_age_s": joint_age, "joint_minus_odometry_stamp_s": joint_skew,
    }
    summary = {
        "schema_version": 1, "raw_sample_count": len(raw_samples), "sample_count": len(samples),
        "duplicate_odometry_samples_removed": len(raw_samples)-len(samples),
        "duration_s": float(times[-1]-times[0]), "supported_duration_s": float(weights.sum()),
        "max_sample_gap_s": float(np.max(np.diff(times))),
        "derivative_gap_limit_s": max_gap_s, "min_moving_speed_m_s": min_speed,
        "initial_motion_window_s": initial_motion_window_s,
        "root_pose_frame_assumption": root, "completed": bool(trajectory.get("completed", False)),
        "trajectory_passed": bool(trajectory.get("summary", {}).get("passed", False)),
        "startup_alignment": trajectory.get("startup_alignment"),
        "requested_route_points_m": trajectory.get("requested_route_points_m", trajectory.get("configuration", {}).get("points_m")),
        "executed_route_points_m": trajectory.get("executed_route_points_m", trajectory.get("configuration", {}).get("points_m")),
        "actuator_limits": actuator_limits,
        "feedback_timing": {
            "joint_source_stamp_sample_count": int(np.isfinite(joint_stamps).sum()),
            "odometry_age_s": weighted_stats(odometry_age, regions["all"], weights),
            "joint_state_age_s": weighted_stats(joint_age, regions["all"], weights),
            "joint_minus_odometry_stamp_s": weighted_stats(joint_skew, regions["all"], weights),
            "joint_interpolation_applied": False,
            "note": "Raw latest joint feedback is retained. Missing joint timestamps make source skew unquantifiable; 100ms and 200ms derivative sensitivities are reported separately.",
        },
        "tracking": {region: weighted_stats(tracking_error, mask, weights) for region, mask in regions.items()},
        "body_heading": {"initial_deg": float(np.degrees(yaw[0])), "final_unwrapped_deg": float(np.degrees(yaw[-1])),
                         "excursion_deg": float(np.degrees(np.ptp(yaw))),
                         "total_rotation_deg": float(np.degrees(np.sum(np.abs(np.diff(yaw))[np.diff(times) <= max_gap_s]))),
                         "yaw_rate_rad_s": weighted_stats(yaw_rate, derivative_valid, weights)},
        "casters": {},
        "interpretation": [
            "Directed angle: 0 degrees means fork +X follows velocity; 180 means leading/backward rolling. Rolling-axis angle identifies forward and reverse as equivalent.",
            "Directed angles greater than 150 degrees identify near-opposite travel (within 30 degrees of backward). Rolling-axis angles cannot exceed 90 degrees, so their 150-degree exceedances are always zero.",
            "The planned tangent is the commanded base path, not the correct caster tangent during chassis turns. Local pivot and wheel velocities include measured chassis rotation and URDF joint motion.",
            "A turning caster can have nonzero pivot alignment due to its trail. Wheel-center lateral velocity and rolling-axis alignment describe scrub more directly; angle thresholds alone do not establish a failure.",
            "Moving-only denominators use the relevant local point speed, or planned speed for path angles. Stationary direction is undefined and excluded, including low-speed caster reversal instants.",
            "Startup contains reported startup/alignment phases. Initial motion covers the first configured seconds of trajectory motion; later motion excludes that recovery window.",
            "First-leg metrics use reported segment zero, keeping a shortened first leg after startup alignment from mixing its first corner into startup recovery comparisons.",
            "Startup displacement from the original reference can be intentional heading preparation around the offset rear axle. Startup error is not normal route-tracking error; compare the requested and executed first route points.",
            "Slip values are finite-difference planar proxies, not measured contact forces. They assume a flat floor and approximately horizontal wheel axles; contact manifold, tire compliance and bearing torque are not measured.",
            "FK uses the latest recorded joint positions with each odometry source stamp. Unsynchronized joint feedback and numerical derivatives can create transient apparent slip.",
            "When every sample has a joint source timestamp, an additional diagnostic linearly interpolates joint positions and velocities to odometry time. It excludes extrapolation and large source gaps; interpolation is still an estimate, not independently measured contact slip.",
            "The 100ms and 200ms secant comparisons test derivative sensitivity, not uncertainty bounds. They smooth real brief events; averaging curved wheel motion against an instantaneous fork axis can also increase apparent lateral velocity. Raw angles and dynamics remain available.",
            "Durations use midpoint sample cells; gaps larger than the derivative limit contribute no duration and break persistence. Samples bordering such gaps are excluded from derivative-based metrics.",
        ],
    }
    summary["body_heading"]["regions"] = {}
    for region, mask in regions.items():
        indices = np.flatnonzero(mask)
        if len(indices):
            connected = mask[:-1] & mask[1:] & (np.diff(times) <= max_gap_s)
            values = yaw[indices]
            heading = {"initial_deg": float(np.degrees(values[0])),
                       "final_unwrapped_deg": float(np.degrees(values[-1])),
                       "net_change_deg": float(np.degrees(values[-1]-values[0])),
                       "excursion_deg": float(np.degrees(np.ptp(values))),
                       "total_abs_rotation_deg": float(np.degrees(np.abs(np.diff(yaw))[connected].sum()))}
        else:
            heading = {key: None for key in ("initial_deg", "final_unwrapped_deg", "net_change_deg", "excursion_deg", "total_abs_rotation_deg")}
        summary["body_heading"]["regions"][region] = heading
    for side in SIDES:
        fork = frames[f"{side}_caster_fork_link"]
        wheel = frames[f"{side}_caster_wheel_link"]
        pivot = fork[:, :3, 3]
        center = wheel[:, :3, 3]
        direction, projection = unit_xy(fork[:, :3, 0])
        leftward = np.column_stack((-direction[:, 1], direction[:, 0]))
        pivot_velocity = np.gradient(pivot[:, :2], times, axis=0, edge_order=2)
        wheel_velocity = np.gradient(center[:, :2], times, axis=0, edge_order=2)
        pivot_speed = np.linalg.norm(pivot_velocity, axis=1)
        wheel_speed = np.linalg.norm(wheel_velocity, axis=1)
        fork_yaw = np.unwrap(np.arctan2(direction[:, 1], direction[:, 0]))
        fork_rate = np.gradient(fork_yaw, times, edge_order=2)
        trail = np.sum((pivot[:, :2]-center[:, :2])*direction, axis=1)
        wheel_name = f"{side}_caster_wheel_joint"
        joint = robot.find(f"./joint[@name='{wheel_name}']")
        axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
        axle = wheel[:, :3, :3] @ (axis/np.linalg.norm(axis))
        roll_direction, _ = unit_xy(np.cross(axle, [0., 0., 1.]))
        collision = robot.find(f"./link[@name='{side}_caster_wheel_link']/collision/geometry/cylinder")
        if collision is None:
            raise ValueError("Wheel slip proxy requires the cylinder collision radius")
        radius = float(collision.get("radius"))
        if radius <= 0:
            raise ValueError("Wheel collision radius must be positive")
        q = np.array([s["joint_positions_rad"][wheel_name] for s in samples])
        measured_rate_available = all(wheel_name in s.get("joint_velocities_rad_s", {}) for s in samples)
        wheel_rate = (np.array([s["joint_velocities_rad_s"][wheel_name] for s in samples])
                      if measured_rate_available else np.gradient(q, times, edge_order=2))
        if not np.isfinite(wheel_rate).all():
            raise ValueError(f"Non-finite wheel rate for {side}")
        lateral = np.sum(wheel_velocity*leftward, axis=1)
        wider_wheel_velocity = secant_velocity(center[:, :2], times, .1)
        wider_lateral = np.sum(wider_wheel_velocity*leftward, axis=1)
        _, wider_axis_angle = angle_degrees(direction, wider_wheel_velocity)
        wider_speed = np.linalg.norm(wider_wheel_velocity, axis=1)
        widest_wheel_velocity = secant_velocity(center[:, :2], times, .2)
        widest_lateral = np.sum(widest_wheel_velocity*leftward, axis=1)
        _, widest_axis_angle = angle_degrees(direction, widest_wheel_velocity)
        widest_speed = np.linalg.norm(widest_wheel_velocity, axis=1)
        roll_slip = np.sum(wheel_velocity*roll_direction, axis=1)-radius*wheel_rate
        valid_trail = np.abs(trail) > .001
        ideal_rate = np.divide(np.sum(pivot_velocity*leftward, axis=1), trail,
                               out=np.full(len(samples), np.nan), where=valid_trail)
        # Geometric radial contact reference, not the collision engine manifold.
        down = np.array([0., 0., 1.])-axle*axle[:, 2:3]
        down /= np.maximum(np.linalg.norm(down, axis=1)[:, None], 1e-15)
        contact = center-radius*down
        local_valid = derivative_valid & (projection > .2)
        analyses = {}
        angles = {}
        for target, velocity, speed in (("commanded_path", tangent, reference_speed),
                                        ("local_pivot", pivot_velocity, pivot_speed),
                                        ("local_wheel", wheel_velocity, wheel_speed)):
            directed, undirected = angle_degrees(direction, velocity)
            eligible = (speed >= min_speed) & (projection > .2)
            if target != "commanded_path":
                eligible &= derivative_valid
            angles[target] = directed, undirected, eligible
            rows[f"{side}_{target}_directed_deg"] = np.where(eligible, directed, np.nan)
            rows[f"{side}_{target}_rolling_axis_deg"] = np.where(eligible, undirected, np.nan)
        for region, mask in regions.items():
            region_result = {}
            for target, (directed, undirected, eligible) in angles.items():
                region_result[target] = {
                    "directed": alignment_stats(directed, mask & eligible, times, weights, max_gap_s),
                    "rolling_axis": alignment_stats(undirected, mask & eligible, times, weights, max_gap_s),
                }
            active = mask & local_valid & ((pivot_speed >= min_speed) | (wheel_speed >= min_speed))
            region_result["lateral_slip_proxy_m_s"] = weighted_stats(lateral, active, weights)
            region_result["rolling_slip_proxy_m_s"] = weighted_stats(roll_slip, active, weights)
            region_result["swivel_rate_residual_rad_s"] = weighted_stats(ideal_rate-fork_rate, active & valid_trail, weights)
            region_result["absolute_lateral_distance_proxy_m"] = float(np.sum(weights[active]*np.abs(lateral[active])))
            region_result["derivative_sensitivity_100ms"] = {
                "lateral_slip_proxy_m_s": weighted_stats(wider_lateral, active, weights),
                "local_wheel_rolling_axis": alignment_stats(wider_axis_angle,
                    mask & local_valid & (wider_speed >= min_speed), times, weights, max_gap_s),
            }
            region_result["derivative_sensitivity_200ms"] = {
                "lateral_slip_proxy_m_s": weighted_stats(widest_lateral, active, weights),
                "local_wheel_rolling_axis": alignment_stats(widest_axis_angle,
                    mask & local_valid & (widest_speed >= min_speed), times, weights, max_gap_s),
            }
            analyses[region] = region_result
        summary["casters"][side] = {
            "wheel_radius_m": radius, "trail_m_min_max": [float(trail.min()), float(trail.max())],
            "applied_physical_parameters": physical_parameters(robot, side),
            "maximum_axle_tilt_deg": float(np.degrees(np.arcsin(np.clip(np.abs(axle[:, 2]), 0, 1))).max()),
            "wheel_rate_source": "JointState velocity" if measured_rate_available else "finite difference of JointState position",
            "regions": analyses,
        }
        for name, vector in (("pivot", pivot), ("wheel_center", center), ("contact_reference", contact)):
            for column, axis_name in enumerate("xyz"):
                rows[f"{side}_{name}_{axis_name}_m"] = vector[:, column]
        rows.update({f"{side}_fork_world_yaw_deg": np.degrees(fork_yaw),
                     f"{side}_pivot_speed_m_s": pivot_speed, f"{side}_wheel_center_speed_m_s": wheel_speed,
                     f"{side}_lateral_slip_proxy_m_s": np.where(local_valid, lateral, np.nan),
                     f"{side}_lateral_slip_proxy_100ms_m_s": np.where(local_valid, wider_lateral, np.nan),
                     f"{side}_lateral_slip_proxy_200ms_m_s": np.where(local_valid, widest_lateral, np.nan),
                     f"{side}_rolling_slip_proxy_m_s": np.where(local_valid, roll_slip, np.nan),
                     f"{side}_wheel_rate_rad_s": wheel_rate,
                     f"{side}_wheel_position_rad": q,
                     f"{side}_swivel_position_rad": np.array([s["joint_positions_rad"][f"{side}_caster_swivel_joint"] for s in samples]),
                     f"{side}_fork_world_yaw_rate_rad_s": fork_rate,
                     f"{side}_swivel_rate_residual_rad_s": np.where(local_valid, ideal_rate-fork_rate, np.nan)})
    if _time_alignment:
        aligned_samples = time_aligned_samples(samples, robot, max_gap_s)
        diagnostic = {"available": aligned_samples is not None,
                      "method": "Linear joint interpolation to odometry source stamps; no extrapolation; raw analysis retained",
                      "note": "Source timing correction and finite-difference smoothing answer different questions; neither establishes calibrated contact slip."}
        if aligned_samples is not None:
            aligned_trajectory = {**trajectory, "samples": aligned_samples}
            aligned_summary, aligned_rows = analyze_trajectory(aligned_trajectory, robot, min_speed,
                max_gap_s, initial_motion_window_s, _time_alignment=False)
            diagnostic["sample_count"] = len(aligned_samples)
            diagnostic["casters"] = {side: {"regions": aligned_summary["casters"][side]["regions"]} for side in SIDES}
            index = {stamp: i for i, stamp in enumerate(times)}
            destination_indices = np.array([index[stamp] for stamp in aligned_rows["odometry_stamp_s"]])
            for side in SIDES:
                for key in ("local_pivot_directed_deg", "local_wheel_rolling_axis_deg", "lateral_slip_proxy_m_s", "rolling_slip_proxy_m_s"):
                    values = np.full(len(times), np.nan)
                    values[destination_indices] = aligned_rows[f"{side}_{key}"]
                    rows[f"{side}_time_aligned_{key}"] = values
        summary["source_time_alignment_diagnostic"] = diagnostic
    return summary, rows


def plot_analysis(rows, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(14, 11), constrained_layout=True)
    t = rows["elapsed_s"]
    motion = rows["phase"] == "motion"
    startup = np.array(["startup" in phase or "align" in phase for phase in rows["phase"]])
    axes[0, 0].plot(rows["reference_x_m"][motion], rows["reference_y_m"][motion], "--", color=".5", label="Route reference")
    axes[0, 0].plot(rows["body_x_m"][~startup], rows["body_y_m"][~startup], color="#176d9c", label="Measured route")
    if startup.any():
        axes[0, 0].plot(rows["body_x_m"][startup], rows["body_y_m"][startup], color=".6", label="Preparation")
    axes[0, 0].set(xlabel="World x (m)", ylabel="World y (m)", title="Trajectory")
    axes[0, 0].set_aspect("equal", adjustable="datalim")
    axes[0, 0].legend()
    axes[0, 1].plot(t, rows["body_yaw_unwrapped_deg"], color="#176d9c")
    axes[0, 1].set(title="Measured chassis heading", ylabel="Unwrapped yaw (degrees)")
    for side, color in zip(SIDES, ("#176d9c", "#b95d24")):
        axes[1, 0].plot(t, np.abs(rows[f"{side}_local_pivot_directed_deg"]), color=color, label=side)
        axes[1, 1].plot(t, rows[f"{side}_local_wheel_rolling_axis_deg"], color=color, label=side)
        axes[2, 0].plot(t, rows[f"{side}_lateral_slip_proxy_m_s"], color=color, label=side)
        if f"{side}_time_aligned_local_wheel_rolling_axis_deg" in rows:
            axes[1, 1].plot(t, rows[f"{side}_time_aligned_local_wheel_rolling_axis_deg"], "--", color=color, alpha=.65, label=side+" time-aligned")
            axes[2, 0].plot(t, rows[f"{side}_time_aligned_lateral_slip_proxy_m_s"], "--", color=color, alpha=.65, label=side+" time-aligned")
    axes[1, 0].set(title="Directed fork angle to local pivot velocity", ylabel="Absolute angle (degrees)", ylim=(-5, 185))
    axes[1, 1].set(title="Rolling-axis angle to local wheel velocity", ylabel="Angle (degrees)", ylim=(-3, 93))
    axes[2, 0].set(title="Wheel lateral slip proxy", ylabel="Lateral speed (m/s)")
    axes[2, 1].plot(t, 1000*rows["tracking_error_m"], color="#176d9c")
    axes[2, 1].set(title="Trajectory tracking", ylabel="Position error (mm)")
    for ax in axes.flat:
        ax.grid(alpha=.2)
        if ax is not axes[0, 0]:
            ax.set_xlabel("Elapsed simulation time (s)")
            if startup.any():
                ax.axvspan(t[startup][0], t[startup][-1], color=".5", alpha=.08)
    for ax in (axes[1, 0], axes[1, 1], axes[2, 0]):
        ax.legend()
    fig.suptitle("Measured passive caster behavior — stationary directions excluded")
    fig.savefig(destination, dpi=145)
    plt.close(fig)


def analyze_run(run_dir, min_speed=.03, max_gap_s=.25, make_plot=True,
                initial_motion_window_s=5.):
    """Write caster_stress.json/.csv/.png and return the complete summary."""
    run_dir = Path(run_dir).expanduser().resolve()
    trajectory_path, robot_path = run_dir/"trajectory.json", run_dir/"robot.urdf"
    trajectory = json.loads(trajectory_path.read_text())
    robot_bytes = robot_path.read_bytes()
    summary, rows = analyze_trajectory(trajectory, ET.fromstring(robot_bytes), min_speed, max_gap_s,
                                       initial_motion_window_s)
    summary["run_directory"] = str(run_dir)
    summary["robot_sha256"] = hashlib.sha256(robot_bytes).hexdigest()
    summary["trajectory_sha256"] = hashlib.sha256(trajectory_path.read_bytes()).hexdigest()
    summary["configuration"] = trajectory.get("configuration", {})
    world_path = run_dir/"world.sdf"
    if world_path.exists():
        world = ET.parse(world_path)
        friction = world.find(".//model[@name='ground_plane']/link/collision/surface/friction/ode/mu")
        if friction is not None:
            summary["ground_mu"] = float(friction.text)
            for caster in summary["casters"].values():
                parameters = caster["applied_physical_parameters"]
                if parameters["tire_mu"] is not None:
                    parameters["effective_dart_mu"] = min(parameters["tire_mu"], summary["ground_mu"])
    for artifact in ("physical_properties.yaml", "commands.json"):
        path = run_dir/artifact
        if path.exists():
            summary[artifact+"_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    with (run_dir/"caster_stress.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(rows)
        for values in zip(*rows.values()):
            writer.writerow(["" if isinstance(v, (float, np.floating)) and not math.isfinite(v) else v for v in values])
    if make_plot:
        plot_analysis(rows, run_dir/"caster_stress.png")
    (run_dir/"caster_stress.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
    return summary


def suite_row(summary):
    """Compact comparable metrics for a suite CSV, without pass/fail inference."""
    result = {"run_directory": summary.get("run_directory"), "completed": summary["completed"],
              "trajectory_passed": summary["trajectory_passed"], "duration_s": summary["duration_s"],
              "tracking_rms_m": summary["tracking"]["motion"]["rms"],
              "tracking_max_m": summary["tracking"]["motion"]["max_abs"],
              "body_heading_excursion_deg": summary["body_heading"]["excursion_deg"],
              "body_heading_motion_net_deg": summary["body_heading"]["regions"]["motion"]["net_change_deg"],
              "body_heading_motion_total_abs_deg": summary["body_heading"]["regions"]["motion"]["total_abs_rotation_deg"],
              "max_sample_gap_s": summary["max_sample_gap_s"],
              "max_abs_odometry_age_s": summary["feedback_timing"]["odometry_age_s"]["max_abs"],
              "max_abs_joint_odometry_skew_s": summary["feedback_timing"]["joint_minus_odometry_stamp_s"]["max_abs"]}
    result.update({"motion_"+key: value for key, value in summary["actuator_limits"]["sampled_regions"]["motion"].items()})
    startup = summary.get("startup_alignment") or {}
    displacement = startup.get("preparation_displacement_m")
    errors = startup.get("final_caster_heading_errors_rad", {})
    yaw_change = startup.get("actual_chassis_yaw_change_rad")
    result.update({
        "startup_alignment_enabled": bool(startup.get("enabled", False)),
        "startup_alignment_passed": startup.get("passed"),
        "startup_alignment_duration_s": startup.get("duration_s"),
        "startup_preparation_displacement_m": float(np.linalg.norm(displacement)) if displacement is not None else None,
        "startup_actual_yaw_change_deg": math.degrees(yaw_change) if yaw_change is not None else None,
        "startup_final_caster_max_heading_error_deg": max(math.degrees(abs(v)) for v in errors.values()) if errors else None,
    })
    for side in SIDES:
        metrics = summary["casters"][side]["regions"]["motion"]
        for target in ("commanded_path", "local_pivot", "local_wheel"):
            for kind in ("directed", "rolling_axis"):
                prefix = f"{side}_{target}_{kind}"
                angle = metrics[target][kind]
                result[prefix+"_p95_deg"] = angle["p95_abs"]
                for threshold in ("15", "30", "60", "150"):
                    event = angle["exceeds"].get(threshold, {})
                    result[prefix+"_fraction_gt"+threshold] = event.get("fraction_of_moving_time")
                    result[prefix+"_longest_gt"+threshold+"_s"] = event.get("longest_episode_s")
        result[f"{side}_lateral_slip_rms_m_s"] = metrics["lateral_slip_proxy_m_s"]["rms"]
        result[f"{side}_lateral_slip_max_m_s"] = metrics["lateral_slip_proxy_m_s"]["max_abs"]
        result[f"{side}_lateral_distance_proxy_m"] = metrics["absolute_lateral_distance_proxy_m"]
        result[f"{side}_lateral_slip_100ms_rms_m_s"] = metrics["derivative_sensitivity_100ms"]["lateral_slip_proxy_m_s"]["rms"]
        result[f"{side}_lateral_slip_100ms_max_m_s"] = metrics["derivative_sensitivity_100ms"]["lateral_slip_proxy_m_s"]["max_abs"]
        result[f"{side}_lateral_slip_200ms_rms_m_s"] = metrics["derivative_sensitivity_200ms"]["lateral_slip_proxy_m_s"]["rms"]
        result[f"{side}_lateral_slip_200ms_max_m_s"] = metrics["derivative_sensitivity_200ms"]["lateral_slip_proxy_m_s"]["max_abs"]
        for region in ("startup", "initial_motion", "later_motion", "first_leg", "later_legs"):
            if region not in summary["casters"][side]["regions"]:
                result[f"{side}_{region}_lateral_slip_rms_m_s"] = None
                result[f"{side}_{region}_wheel_axis_longest_gt15_s"] = None
                continue
            region_metrics = summary["casters"][side]["regions"][region]
            result[f"{side}_{region}_lateral_slip_rms_m_s"] = region_metrics["lateral_slip_proxy_m_s"]["rms"]
            angle = region_metrics["local_wheel"]["rolling_axis"]
            result[f"{side}_{region}_wheel_axis_longest_gt15_s"] = angle["exceeds"]["15"]["longest_episode_s"]
        diagnostic = summary.get("source_time_alignment_diagnostic", {})
        for suffix in ("wheel_axis_fraction_gt15", "wheel_axis_longest_gt15_s",
                       "lateral_slip_rms_m_s", "lateral_slip_max_m_s"):
            result[f"{side}_time_aligned_{suffix}"] = None
        if diagnostic.get("available"):
            metrics = diagnostic["casters"][side]["regions"]["motion"]
            angle = metrics["local_wheel"]["rolling_axis"]["exceeds"]["15"]
            result[f"{side}_time_aligned_wheel_axis_fraction_gt15"] = angle["fraction_of_moving_time"]
            result[f"{side}_time_aligned_wheel_axis_longest_gt15_s"] = angle["longest_episode_s"]
            result[f"{side}_time_aligned_lateral_slip_rms_m_s"] = metrics["lateral_slip_proxy_m_s"]["rms"]
            result[f"{side}_time_aligned_lateral_slip_max_m_s"] = metrics["lateral_slip_proxy_m_s"]["max_abs"]
        result[f"{side}_time_aligned_first_leg_wheel_axis_longest_gt15_s"] = None
        result[f"{side}_time_aligned_first_leg_directed_pivot_longest_gt15_s"] = None
        if diagnostic.get("available") and "first_leg" in diagnostic["casters"][side]["regions"]:
            metrics = diagnostic["casters"][side]["regions"]["first_leg"]
            result[f"{side}_time_aligned_first_leg_wheel_axis_longest_gt15_s"] = metrics["local_wheel"]["rolling_axis"]["exceeds"]["15"]["longest_episode_s"]
            result[f"{side}_time_aligned_first_leg_directed_pivot_longest_gt15_s"] = metrics["local_pivot"]["directed"]["exceeds"]["15"]["longest_episode_s"]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--min-speed", type=float, default=.03)
    parser.add_argument("--max-gap", type=float, default=.25)
    parser.add_argument("--initial-motion-window", type=float, default=5.)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    result = analyze_run(args.run_directory, args.min_speed, args.max_gap, not args.no_plot,
                         args.initial_motion_window)
    print(json.dumps(suite_row(result), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
