#!/usr/bin/env python3
"""Analyze measured caster experiments, including partial and failed runs.

Usage: analyze_caster_stress.py CAMPAIGN_OR_REPLAY_ROOT [--output-dir DIRECTORY]
Only reads run artifacts; writes a separate analysis directory. Requires NumPy;
plots additionally use Matplotlib. No ROS, simulator, or model inference runs.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path, errors):
    if not path.is_file():
        return {}
    try:
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError) as error:
        errors.append(f"Cannot read {path.name}: {error}")
        return {}


def number(value):
    try:
        return float(value) if math.isfinite(float(value)) else np.nan
    except (TypeError, ValueError):
        return np.nan


def vector(sample, names, length):
    for name in names:
        value = sample.get(name)
        if isinstance(value, (list, tuple)) and len(value) >= length:
            return [number(v) for v in value[:length]]
    return [np.nan]*length


def timestamp(sample):
    for key in ("odometry_stamp_s", "simulation_time_s", "stamp_s", "elapsed_s"):
        value = number(sample.get(key))
        if np.isfinite(value):
            return value
    return np.nan


def normalize_samples(samples):
    """Keep latest duplicate stamps; stop at clock reversal instead of mixing epochs."""
    chosen, invalid, duplicates, reversed_clock = [], 0, 0, False
    for sample in samples:
        if not isinstance(sample, dict):
            invalid += 1
            continue
        t = timestamp(sample)
        if not np.isfinite(t):
            invalid += 1
            continue
        if chosen and t < chosen[-1][0]:
            reversed_clock = True
            break
        if chosen and t == chosen[-1][0]:
            chosen[-1] = (t, sample)
            duplicates += 1
        else:
            chosen.append((t, sample))
    return (np.array([t for t, _ in chosen]), [s for _, s in chosen],
            {"invalid_timestamps": invalid, "duplicate_timestamps": duplicates,
             "clock_reversal_truncated": reversed_clock})


def weights(times, eligible, max_gap):
    """Trapezoidal time weights, never crossing excluded samples or long gaps."""
    result = np.zeros(len(times))
    if len(times) > 1:
        dt = np.diff(times)
        duration = np.where((dt > 0) & (dt <= max_gap) & eligible[:-1] & eligible[1:], dt, 0.)
        result[:-1] += duration/2
        result[1:] += duration/2
    return result


def statistics(values, times, eligible, max_gap):
    values = np.asarray(values, dtype=float)
    valid = eligible & np.isfinite(values)
    w = weights(times, valid, max_gap)
    duration = float(w.sum())
    result = {"sample_count": int(valid.sum()), "observed_duration_s": duration,
              "mean": None, "mean_abs": None, "rms": None, "std": None,
              "p95_abs": None, "max_abs": None, "min": None, "max": None,
              "peak_to_peak": None}
    if not duration:
        return result
    use = valid & (w > 0)
    v, w = values[use], w[use]
    mean = float(np.dot(w, v)/duration)
    order = np.argsort(abs(v))
    index = min(int(np.searchsorted(np.cumsum(w[order]), .95*duration)), len(v)-1)
    result.update(mean=mean, mean_abs=float(np.dot(w, abs(v))/duration),
                  rms=float(np.sqrt(np.dot(w, v*v)/duration)),
                  std=float(np.sqrt(np.dot(w, (v-mean)**2)/duration)),
                  p95_abs=float(abs(v)[order][index]), max_abs=float(max(abs(v))),
                  min=float(min(v)), max=float(max(v)), peak_to_peak=float(np.ptp(v)))
    return result


def derivative(values, times, max_gap):
    """Differentiate only contiguous finite stretches; no gap interpolation."""
    values = np.asarray(values, dtype=float)
    result = np.full_like(values, np.nan, dtype=float)
    finite = np.isfinite(values) if values.ndim == 1 else np.isfinite(values).all(axis=1)
    start = 0
    while start < len(times):
        if not finite[start]:
            start += 1
            continue
        end = start+1
        while end < len(times) and finite[end] and 0 < times[end]-times[end-1] <= max_gap:
            end += 1
        if end-start >= 2:
            result[start:end] = np.gradient(values[start:end], times[start:end], axis=0)
        start = end
    return result


def longest_interval(mask, times, eligible, max_gap):
    current = longest = 0.
    for i in range(1, len(times)):
        dt = times[i]-times[i-1]
        if mask[i] and mask[i-1] and eligible[i] and eligible[i-1] and 0 < dt <= max_gap:
            current += dt
        else:
            current = 0.
        longest = max(longest, current)
    return float(longest)


def yaw_motion(yaw, times, eligible, max_gap):
    """Integrate observed heading changes without joining gaps or wrap boundaries."""
    valid = eligible & np.isfinite(yaw)
    interval = valid[:-1] & valid[1:] & (np.diff(times) > 0) & (np.diff(times) <= max_gap)
    delta = np.arctan2(np.sin(np.diff(yaw)), np.cos(np.diff(yaw)))
    spans, start = [], 0
    while start < len(times):
        if not valid[start]:
            start += 1
            continue
        end = start+1
        while end < len(times) and interval[end-1]:
            end += 1
        if end-start >= 2:
            spans.append(float(np.ptp(np.unwrap(yaw[start:end]))))
        start = end
    return {"observed_net_rotation_deg": float(np.degrees(delta[interval].sum())) if interval.any() else None,
            "observed_absolute_travel_deg": float(np.degrees(abs(delta[interval]).sum())) if interval.any() else None,
            "maximum_contiguous_span_deg": float(np.degrees(max(spans))) if spans else None,
            "contiguous_segments": len(spans),
            "scope": "Measured body heading, independent of path travel direction; assumes less than 180 degrees change between adjacent samples. Gaps are not integrated; absolute travel includes heading jitter."}


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis/np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3)+np.sin(angle)*skew+(1-np.cos(angle))*(skew@skew)


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        transform[:3, :3] = rotation([0, 0, 1], y)@rotation([0, 1, 0], p)@rotation([1, 0, 0], r)
    return transform


def audit_model(path):
    """Audit the actual expanded URDF; URDF mu is an intention, not runtime proof."""
    robot = ET.parse(path).getroot()
    masses = {l.get("name"): float(l.find("inertial/mass").get("value"))
              for l in robot.findall("link") if l.find("inertial/mass") is not None}
    caster_joints, actuators, contacts = {}, [], {}
    for joint in robot.findall("joint"):
        name = joint.get("name", "")
        if "caster" in name and joint.get("type") != "fixed":
            dynamics = joint.find("dynamics")
            caster_joints[name] = {"type": joint.get("type"),
                                  "axis": joint.find("axis").get("xyz") if joint.find("axis") is not None else "1 0 0",
                                  "dynamics": dict(dynamics.attrib) if dynamics is not None else {}}
    for plugin in robot.findall(".//plugin"):
        if plugin.get("name", "").endswith("JointController"):
            actuators.append({"joint": plugin.findtext("joint_name"),
                              "force_mode": plugin.findtext("use_force_commands", "false") == "true"})
    for gazebo in robot.findall("gazebo"):
        mu = gazebo.find("mu1")
        if mu is not None:
            contacts[gazebo.get("reference")] = number(mu.get("value", mu.text))
    radius = {}
    for link in robot.findall("link"):
        cylinder = link.find("collision/geometry/cylinder")
        if cylinder is not None and "wheel_link" in link.get("name", ""):
            radius[link.get("name")] = float(cylinder.get("radius"))
    return {"file": str(path), "sha256": sha256(path), "total_mass_kg": sum(masses.values()),
            "caster_link_masses_kg": {k:v for k,v in masses.items() if "caster" in k},
            "caster_inertials": {l.get("name"): ET.tostring(l.find("inertial"), encoding="unicode")
                                  for l in robot.findall("link") if "caster" in l.get("name", "") and l.find("inertial") is not None},
            "passive_caster_joints": caster_joints, "joint_controllers": actuators,
            "caster_actuator_present": any("caster" in str(a["joint"]) for a in actuators),
            "cylinder_radii_m": radius, "urdf_intended_mu1": contacts,
            "mu_caveat": "Mesh attachment may ignore serialized material overrides; verify actual backend contact response."}


def interpolate_joints(samples, times, max_gap):
    stamps = np.array([number(s.get("joint_state_stamp_s")) for s in samples])
    valid = np.isfinite(stamps)
    if valid.sum() < 3:
        raise ValueError("Joint source timestamps missing; synchronized FK proxy withheld")
    selected = {}
    for stamp, sample in zip(stamps[valid], np.asarray(samples, dtype=object)[valid]):
        selected[stamp] = sample.get("joint_positions_rad", {})
    jt = np.array(sorted(selected))
    if len(jt) < 3:
        raise ValueError("Insufficient distinct joint source timestamps")
    keys = set.union(*(set(selected[t]) for t in jt))
    result = {}
    for key in keys:
        values = np.array([number(selected[t].get(key)) for t in jt])
        good = np.isfinite(values)
        tx, vx = jt[good], values[good]
        if len(tx) < 2:
            continue
        q = np.interp(times, tx, vx, left=np.nan, right=np.nan)
        for i in np.flatnonzero(np.diff(tx) > max_gap):
            q[(times > tx[i]) & (times < tx[i+1])] = np.nan
        # DART coordinates accumulate revolutions; do not unwrap fast rollers.
        result[key] = q
    return result


def wheel_center_proxy(robot_path, samples, times, pos, rpy, eligible, max_gap, min_speed):
    """World-XY transverse center velocity from full URDF FK, not reference heading."""
    robot = ET.parse(robot_path).getroot()
    joints = {j.find("child").get("link"): j for j in robot.findall("joint")}
    roots = {l.get("name") for l in robot.findall("link")}-set(joints)
    if len(roots) != 1:
        raise ValueError("URDF must have one root")
    root = roots.pop()
    q = interpolate_joints(samples, times, max_gap)
    out = {}
    for target in ("left_wheel_link", "right_wheel_link", "left_caster_wheel_link", "right_caster_wheel_link"):
        if target not in joints:
            continue
        path, link = [], target
        while link != root:
            if link not in joints or len(path) > len(joints):
                raise ValueError("Invalid URDF joint chain")
            joint = joints[link]
            path.append(joint)
            link = joint.find("parent").get("link")
        path.reverse()
        needed = [j.get("name") for j in path if j.get("type") != "fixed"]
        if any(name not in q for name in needed):
            out[target] = {"available": False, "reason": "Missing synchronized FK joint positions"}
            continue
        centers = np.full((len(times), 3), np.nan)
        axes = centers.copy()
        for i in range(len(times)):
            if not np.isfinite(np.r_[pos[i], rpy[i], [q[name][i] for name in needed]]).all():
                continue
            transform = np.eye(4)
            transform[:3, 3] = pos[i]
            r, p, y = rpy[i]
            transform[:3, :3] = rotation([0, 0, 1], y)@rotation([0, 1, 0], p)@rotation([1, 0, 0], r)
            for joint in path:
                transform = transform@origin(joint.find("origin"))
                axis = np.fromstring(joint.find("axis").get("xyz", "1 0 0"), sep=" ") if joint.find("axis") is not None else np.array([1., 0., 0.])
                if joint.get("type") in ("continuous", "revolute"):
                    transform[:3, :3] = transform[:3, :3]@rotation(axis, q[joint.get("name")][i])
                elif joint.get("type") == "prismatic":
                    transform[:3, 3] += transform[:3, :3]@axis*q[joint.get("name")][i]
            centers[i] = transform[:3, 3]
            # Wheel-joint axis is unchanged by its own rolling rotation.
            last_axis = np.fromstring(path[-1].find("axis").get("xyz"), sep=" ")
            axes[i] = transform[:3, :3]@last_axis
        velocity = derivative(centers, times, max_gap)
        lengths = np.linalg.norm(axes[:, :2], axis=1)
        lateral = np.sum(velocity[:, :2]*axes[:, :2], axis=1)/np.maximum(lengths, 1e-12)
        speed = np.linalg.norm(velocity[:, :2], axis=1)
        moving = eligible & (speed >= min_speed) & (lengths > .2)
        angle = np.degrees(np.arcsin(np.clip(abs(lateral)/np.maximum(speed, 1e-12), 0, 1)))
        transverse_stats = statistics(lateral, times, moving, max_gap)
        valid_angle = moving & np.isfinite(angle)
        misaligned = angle > 15.
        threshold_stats = statistics(misaligned.astype(float), times, valid_angle, max_gap)
        out[target] = {"available": transverse_stats["observed_duration_s"] > 0,
                       "transverse_velocity_xy_m_s": transverse_stats,
                       "undirected_axis_misalignment_deg": statistics(angle, times, moving, max_gap),
                       "planar_axis_threshold_deg": 15.,
                       "planar_axis_over_15_deg_fraction": threshold_stats["mean"],
                       "planar_axis_over_15_deg_longest_episode_s": longest_interval(misaligned, times, valid_angle, max_gap) if threshold_stats["observed_duration_s"] else None,
                       "threshold_scope": "Time-weighted fraction of observed moving intervals and longest contiguous sampled episode above a descriptive planar 15-degree threshold; not contact-slip duration."}
    return {"available": any(v.get("available", False) for v in out.values()), "wheels": out,
            "scope": "Planar wheel-center kinematic proxy; no contact-point velocity or local terrain normal. Not contact slip on uneven ground. No analogous split-ball heading is assigned."}


def compute_metrics(samples, times, config, max_gap, horizon=None, min_speed=.03, start_time=0.):
    n = len(times)
    pos = np.array([vector(s, ("position_m",), 3) for s in samples]).reshape(n, 3)
    rpy = np.array([vector(s, ("base_rpy_rad", "rpy_rad"), 3) for s in samples]).reshape(n, 3)
    ref = np.array([vector(s, ("reference_m", "reference_position_m", "reference_xy_m"), 2) for s in samples]).reshape(n, 2)
    phases = np.array([s.get("phase", "unknown") for s in samples])
    trajectory_time = np.array([number(s.get("trajectory_time_s")) for s in samples])
    motion = phases == "motion"
    if not motion.any() and np.all(phases == "unknown"):
        motion[:] = True
    if horizon is not None:
        motion &= np.isfinite(trajectory_time) & (trajectory_time >= start_time) & (trajectory_time <= horizon)
    pose_valid = np.isfinite(pos).all(axis=1) & np.isfinite(rpy).all(axis=1)
    eligible = motion & np.isfinite(pos[:, :2]).all(axis=1)
    error = np.linalg.norm(pos[:, :2]-ref, axis=1)
    velocity = np.column_stack([derivative(pos[:, :2], times, max_gap), derivative(pos[:, 2], times, max_gap)])
    ref_velocity = derivative(ref, times, max_gap)
    # Prefer source-time reference velocity when logged, retaining derivative
    # fallback for old data. Intentional dwells are excluded from stall tests.
    for i, sample in enumerate(samples):
        explicit = vector(sample, ("reference_velocity_m_s", "reference_velocity_world_m_s"), 2)
        if np.isfinite(explicit).all():
            ref_velocity[i] = explicit
    speed = np.linalg.norm(velocity[:, :2], axis=1)
    ref_speed = np.linalg.norm(ref_velocity, axis=1)
    demand = eligible & (ref_speed >= min_speed) & np.isfinite(speed)
    along_reference = np.sum(velocity[:, :2]*ref_velocity, axis=1)/np.maximum(ref_speed, 1e-12)
    stall = demand & (along_reference < .02)
    low_speed = demand & (speed < .02)
    demand_w = weights(times, demand, max_gap)
    stall_duration = float(np.sum(demand_w*stall))
    ids = np.flatnonzero(eligible)
    progress = {"world_xy_displacement_m": None, "observed_xy_distance_m": None,
                "reference_xy_distance_over_observed_intervals_m": None,
                "last_motion_trajectory_time_s": None,
                "first_motion_trajectory_time_s": None,
                "low_progress_threshold_m_s": .02, "reference_motion_threshold_m_s": min_speed,
                "demanded_motion_observed_s": float(demand_w.sum()),
                "low_progress_duration_s": stall_duration,
                "low_progress_fraction": stall_duration/float(demand_w.sum()) if demand_w.sum() else None,
                "longest_low_progress_episode_s": longest_interval(stall, times, demand, max_gap),
                "low_speed_duration_s": float(np.sum(demand_w*low_speed)),
                "longest_low_speed_episode_s": longest_interval(low_speed, times, demand, max_gap),
                "along_reference_velocity_m_s": statistics(along_reference, times, demand, max_gap),
                "scope": "Low projection onto instantaneous reference travel direction; separate low-speed metric is a stall indicator, not proof of motor stall or path completion."}
    if len(ids):
        progress["world_xy_displacement_m"] = (pos[ids[-1], :2]-pos[ids[0], :2]).tolist()
        progress["last_motion_trajectory_time_s"] = float(trajectory_time[ids[-1]]) if np.isfinite(trajectory_time[ids[-1]]) else None
        progress["first_motion_trajectory_time_s"] = float(trajectory_time[ids[0]]) if np.isfinite(trajectory_time[ids[0]]) else None
        intervals = eligible[:-1] & eligible[1:] & (np.diff(times) <= max_gap)
        progress["observed_xy_distance_m"] = float(np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1)[intervals].sum())
        good_ref = intervals & np.isfinite(ref[:-1]).all(axis=1) & np.isfinite(ref[1:]).all(axis=1)
        progress["reference_xy_distance_over_observed_intervals_m"] = float(np.linalg.norm(np.diff(ref, axis=0), axis=1)[good_ref].sum()) if good_ref.any() else None
    joints = {}
    names = sorted(set.union(*(set(s.get("joint_positions_rad", {})) for s in samples))) if samples else []
    for name in names:
        if "caster" not in name:
            continue
        q = np.array([number(s.get("joint_positions_rad", {}).get(name)) for s in samples])
        rates = np.array([number(s.get("joint_velocities_rad_s", {}).get(name)) for s in samples])
        jt = np.array([number(s.get("joint_state_stamp_s")) for s in samples])
        use_source = np.isfinite(jt).all() and np.all(np.diff(jt) >= 0)
        joint_times = jt if use_source else times
        interval = eligible[:-1] & eligible[1:] & np.isfinite(q[:-1]) & np.isfinite(q[1:]) & (np.diff(joint_times) > 0) & (np.diff(joint_times) <= max_gap)
        valid_ids = np.flatnonzero(eligible & np.isfinite(q))
        joints[name] = {"angle_start_rad": float(q[valid_ids[0]]) if len(valid_ids) else None,
                        "angle_end_rad": float(q[valid_ids[-1]]) if len(valid_ids) else None,
                        "angle_travel_rad": float(abs(np.diff(q))[interval].sum()),
                        "rate_rad_s": statistics(rates, joint_times, eligible, max_gap),
                        "time_basis": "joint_source_time" if use_source else "odom_samples; joint source stamps unavailable",
                        "meaning": "swivel coordinate" if "swivel" in name else "internal rolling coordinate; not heading"}
    command = np.array([vector(s, ("commands_rad_s", "command_rad_s"), 2) for s in samples]).reshape(n, 2)
    wheel_limit = number(config.get("controller", {}).get("max_wheel_rate_rad_s"))
    saturation = {"explicit_flags": {}, "command_at_wheel_rate_boundary_fraction": None,
                  "drive_any_limited_scope": "any_limited_fraction covers drive software translation, rear-wheel rate and rear-wheel acceleration limits only; it does not cover all actuators.",
                  "yaw_gimbal_rate_boundary_rad_s": 2.5,
                  "yaw_gimbal_boundary_scope": "Boundary occupancy of the logged fifth command at the replay controller's fixed +/-2.5 rad/s yaw-gimbal clamp; cannot establish torque saturation."}
    if np.isfinite(wheel_limit):
        cw = weights(times, eligible & np.isfinite(command).all(axis=1), max_gap)
        saturation["command_at_wheel_rate_boundary_fraction"] = float(np.dot(cw, np.max(abs(command), axis=1) >= wheel_limit*(1-1e-6))/cw.sum()) if cw.sum() else None
    yaw_gimbal_command = np.array([vector(s, ("commands_rad_s", "command_rad_s"), 5)[4] for s in samples])
    yaw_gimbal_at_boundary = np.isclose(abs(yaw_gimbal_command), 2.5, rtol=1e-6, atol=0.)
    yaw_boundary_stats = statistics(yaw_gimbal_at_boundary.astype(float), times,
                                    motion & np.isfinite(yaw_gimbal_command), max_gap)
    saturation["yaw_gimbal_at_rate_boundary_fraction"] = yaw_boundary_stats["mean"]
    saturation["yaw_gimbal_boundary_observed_duration_s"] = yaw_boundary_stats["observed_duration_s"]
    flags = sorted({k for s in samples for k in s if "limit" in k or "saturat" in k})
    for key in flags:
        values = np.array([float(s[key]) if isinstance(s.get(key), bool) else np.nan for s in samples])
        if np.isfinite(values).any():
            saturation["explicit_flags"][key] = statistics(values, times, eligible, max_gap)
    nested_keys = sorted({k for s in samples for k in s.get("command_limits", {})})
    for key in nested_keys:
        values = np.array([float(s["command_limits"][key]) if isinstance(s.get("command_limits", {}).get(key), bool) else np.nan for s in samples])
        saturation["explicit_flags"][key+"_limited"] = statistics(values, times, motion, max_gap)
    any_limits = []
    for sample in samples:
        limits = sample.get("command_limits", {})
        values = [limits.get(k) for k in ("translation", "wheel_rate", "wheel_acceleration")]
        any_limits.append(1. if any(v is True for v in values) else 0. if all(v is False for v in values) else np.nan)
    saturation["explicit_flags"]["any_limited"] = statistics(any_limits, times, motion, max_gap)
    result = {"sample_count": n, "valid_motion_samples": int(eligible.sum()),
              "motion_observed_duration_s": float(weights(times, eligible, max_gap).sum()),
              "tracking_xy_error_m": statistics(error, times, eligible, max_gap),
              "body_roll_deg": statistics(np.degrees(rpy[:, 0]), times, eligible, max_gap),
              "body_pitch_deg": statistics(np.degrees(rpy[:, 1]), times, eligible, max_gap),
              "body_yaw_motion": yaw_motion(rpy[:, 2], times, eligible, max_gap),
              "world_root_z_m": statistics(pos[:, 2], times, eligible, max_gap),
              "world_vertical_velocity_m_s": statistics(velocity[:, 2], times, eligible, max_gap),
              "heave_caveat": "World-root Z variation includes terrain elevation/climb. It is not terrain-relative clearance, vibration, or suspension travel.",
              "progress": progress, "caster_joints": joints, "command_saturation": saturation,
              "invalid_pose_samples": int((~pose_valid).sum()),
              "phase_sample_counts": {str(k): int((phases == k).sum()) for k in sorted(set(phases))}}
    return result, (pos, rpy, eligible, trajectory_time)


def analyze_run(directory, max_gap=.25, min_speed=.03):
    directory = Path(directory).resolve()
    errors = []
    trajectory = read_json(directory/"trajectory.json", errors)
    summary = read_json(directory/"run_summary.json", errors) or read_json(directory/"report.json", errors)
    config = read_json(directory/"configuration.json", errors) or trajectory.get("configuration", {})
    samples = trajectory.get("samples", [])
    if not samples:
        samples = read_json(directory/"measurements.json", errors).get("robot", [])
    times, samples, quality = normalize_samples(samples)
    metrics, arrays = compute_metrics(samples, times, config, max_gap, min_speed=min_speed)
    pos, rpy, eligible, trajectory_time = arrays
    model = None
    model_path = directory/"robot.urdf"
    proxy = {"available": False, "reason": "Actual robot.urdf snapshot unavailable"}
    if model_path.is_file():
        try:
            model = audit_model(model_path)
            proxy = wheel_center_proxy(model_path, samples, times, pos, rpy, eligible, max_gap, min_speed)
        except (ValueError, KeyError, ET.ParseError) as error:
            proxy = {"available": False, "reason": str(error)}
    caster = config.get("caster", summary.get("caster", directory.name if directory.name in ("ball", "swivel") else "unknown"))
    route = config.get("route", summary.get("route", {}))
    shape = config.get("shape", summary.get("shape", route.get("shape", directory.parent.name)))
    terrain = config.get("terrain", summary.get("terrain", directory.parent.parent.name))
    failure = summary.get("physical_failure")
    infrastructure = summary.get("infrastructure_failure") or summary.get("error")
    complete = summary.get("trajectory_clock_complete", summary.get("completed")) is True
    status = "physical_failure" if failure else "infrastructure_failure" if infrastructure else "complete" if complete else "partial_or_unknown"
    physics_step = None
    if (directory/"world.sdf").is_file():
        try:
            physics_step = number(ET.parse(directory/"world.sdf").findtext("world/physics/max_step_size"))
        except ET.ParseError as error:
            errors.append(str(error))
    condition = {"route": route, "controller": config.get("controller"), "spawn": config.get("spawn"),
                 "grid_sha256": config.get("grid_sha256"), "source_world_sha256": config.get("source_world_sha256"),
                 "physics_step_s": physics_step, "runner_sha256": config.get("runner_sha256"),
                 "paths_source_sha256": config.get("paths_source_sha256")}
    result = {"run_directory": str(directory), "terrain": terrain, "shape": shape, "caster": caster,
              "trial": config.get("trial", config.get("repeat", config.get("trial_id", 0))),
              "status": status, "physical_failure": failure, "infrastructure_failure": infrastructure,
              "tracking_passed_reported": summary.get("tracking_passed"),
              "recording_complete_reported": summary.get("recording_complete"),
              "input_errors": errors, "sample_quality": quality, "condition": condition,
              "input_hashes": {name: sha256(directory/name) for name in ("trajectory.json", "configuration.json", "run_summary.json", "robot.urdf") if (directory/name).is_file()},
              "model_audit": model, "metrics": metrics, "wheel_center_proxy": proxy}
    return result, (samples, times, config)


def flat_metrics(metrics):
    return {"xy_rmse_m": metrics["tracking_xy_error_m"]["rms"],
            "xy_p95_m": metrics["tracking_xy_error_m"]["p95_abs"],
            "xy_peak_m": metrics["tracking_xy_error_m"]["max_abs"],
            "roll_rms_deg": metrics["body_roll_deg"]["rms"],
            "roll_peak_deg": metrics["body_roll_deg"]["max_abs"],
            "pitch_rms_deg": metrics["body_pitch_deg"]["rms"],
            "pitch_peak_deg": metrics["body_pitch_deg"]["max_abs"],
            "body_yaw_net_rotation_deg": metrics["body_yaw_motion"]["observed_net_rotation_deg"],
            "body_yaw_absolute_travel_deg": metrics["body_yaw_motion"]["observed_absolute_travel_deg"],
            "body_yaw_maximum_contiguous_span_deg": metrics["body_yaw_motion"]["maximum_contiguous_span_deg"],
            "root_z_std_m": metrics["world_root_z_m"]["std"],
            "root_z_peak_to_peak_m": metrics["world_root_z_m"]["peak_to_peak"],
            "xy_distance_m": metrics["progress"]["observed_xy_distance_m"],
            "low_progress_fraction": metrics["progress"]["low_progress_fraction"],
            "longest_low_progress_s": metrics["progress"]["longest_low_progress_episode_s"],
            "longest_low_speed_s": metrics["progress"]["longest_low_speed_episode_s"],
            "observed_motion_s": metrics["motion_observed_duration_s"],
            "command_at_wheel_rate_boundary_fraction": metrics["command_saturation"]["command_at_wheel_rate_boundary_fraction"],
            "yaw_gimbal_at_rate_boundary_fraction": metrics["command_saturation"].get("yaw_gimbal_at_rate_boundary_fraction"),
            **{key+"_fraction": metrics["command_saturation"]["explicit_flags"].get(key, {}).get("mean")
               for key in ("translation_limited", "wheel_rate_limited", "wheel_acceleration_limited", "any_limited")}}


def flat_proxy(proxy):
    result = {}
    for wheel in ("left_wheel_link", "right_wheel_link", "left_caster_wheel_link", "right_caster_wheel_link"):
        entry = proxy.get("wheels", {}).get(wheel, {})
        prefix = wheel.removesuffix("_link")
        result[prefix+"_planar_transverse_rms_m_s"] = entry.get("transverse_velocity_xy_m_s", {}).get("rms")
        result[prefix+"_planar_axis_p95_deg"] = entry.get("undirected_axis_misalignment_deg", {}).get("p95_abs")
        result[prefix+"_planar_axis_over_15_deg_fraction"] = entry.get("planar_axis_over_15_deg_fraction")
        result[prefix+"_planar_axis_over_15_deg_longest_episode_s"] = entry.get("planar_axis_over_15_deg_longest_episode_s")
    return result


def case_aggregate(pairs):
    """Equal weight per terrain × shape, never per frame or unequal run duration."""
    result = {"pair_count": len(pairs), "matched_pair_count": sum(p.get("paired", False) for p in pairs),
              "fully_verified_pair_count": sum(p.get("fully_verified_pair", False) for p in pairs),
              "status_counts": {}, "common_horizon_delta_by_metric": {},
              "weighting": "Only pairs with complete matching condition metadata enter aggregate metrics. First average available paired trials within terrain×shape; then give each terrain×shape one equal weight. All statuses remain counted. Descriptive deterministic summaries, no p-values or population confidence interval."}
    for caster in ("ball", "swivel"):
        states = [p.get(caster+"_status", "missing_or_duplicate") for p in pairs]
        result["status_counts"][caster] = {s: states.count(s) for s in sorted(set(states))}
    keys = set().union(*(p.get("common_horizon_ball_minus_swivel", {}).keys() for p in pairs))
    for metric in sorted(keys):
        cases = {}
        for pair in pairs:
            value = pair.get("common_horizon_ball_minus_swivel", {}).get(metric)
            if value is not None and pair.get("fully_verified_pair"):
                cases.setdefault((pair["terrain"], pair["shape"]), []).append(value)
        means = [float(np.mean(v)) for v in cases.values()]
        result["common_horizon_delta_by_metric"][metric] = {
            "terrain_shape_cases": len(means), "paired_trials": sum(map(len, cases.values())),
            "mean": float(np.mean(means)) if means else None,
            "median": float(np.median(means)) if means else None,
            "min": min(means) if means else None, "max": max(means) if means else None}
    return result


def finite_json(value):
    if isinstance(value, dict):
        return {k:finite_json(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    return value


def paired_results(runs, contexts, max_gap, min_speed):
    groups = {}
    for i, run in enumerate(runs):
        key = (run["terrain"], run["shape"], str(run["trial"]))
        groups.setdefault(key, {}).setdefault(run["caster"], []).append(i)
    pairs = []
    for key, group in sorted(groups.items()):
        pair = {"terrain": key[0], "shape": key[1], "trial": key[2],
                "available_models": sorted(group), "paired": False}
        if any(len(group.get(c, [])) != 1 for c in ("ball", "swivel")):
            pair["reason"] = "Missing or duplicate runs; assign distinct trial IDs, do not silently average"
            pairs.append(pair)
            continue
        a, b = group["ball"][0], group["swivel"][0]
        ball, swivel = runs[a], runs[b]
        mismatch = [k for k in ball["condition"] if ball["condition"][k] != swivel["condition"][k]]
        missing = [k for k, v in ball["condition"].items() if v is None]
        pair.update(ball_run=ball["run_directory"], swivel_run=swivel["run_directory"],
                    ball_status=ball["status"], swivel_status=swivel["status"],
                    condition_mismatches=mismatch, missing_condition_fields=missing,
                    paired=not mismatch, fully_verified_pair=not mismatch and not missing)
        end_times = [r["metrics"]["progress"]["last_motion_trajectory_time_s"] for r in (ball, swivel)]
        start_times = [r["metrics"]["progress"]["first_motion_trajectory_time_s"] for r in (ball, swivel)]
        horizon = min(end_times) if all(v is not None for v in end_times) else None
        start = max(start_times) if all(v is not None for v in start_times) else None
        pair["common_observed_horizon_s"] = horizon
        pair["common_observed_start_s"] = start
        pair["full_run_ball_minus_swivel"] = {}
        fa = {**flat_metrics(ball["metrics"]), **flat_proxy(ball["wheel_center_proxy"])}
        fb = {**flat_metrics(swivel["metrics"]), **flat_proxy(swivel["wheel_center_proxy"])}
        for metric in fa:
            pair["full_run_ball_minus_swivel"][metric] = fa[metric]-fb[metric] if fa[metric] is not None and fb[metric] is not None else None
        if horizon is not None and start is not None and horizon > start and not mismatch:
            common = {}
            for idx in (a, b):
                run = runs[idx]
                samples, times, config = contexts[idx]
                metric, (pos, rpy, eligible, _) = compute_metrics(samples, times, config, max_gap, horizon, min_speed, start_time=start)
                proxy = {}
                if run["wheel_center_proxy"].get("available"):
                    proxy = wheel_center_proxy(Path(run["run_directory"])/"robot.urdf", samples, times, pos, rpy, eligible, max_gap, min_speed)
                common[run["caster"]] = {**flat_metrics(metric), **flat_proxy(proxy)}
            pair["common_horizon_metrics"] = common
            aa, bb = common["ball"], common["swivel"]
            pair["common_horizon_ball_minus_swivel"] = {k: aa[k]-bb[k] if aa[k] is not None and bb[k] is not None else None for k in aa}
        pair["scope"] = "Descriptive matched-condition contrast; failure status retained. Common horizon reduces unequal exposure, not survivorship bias. No inferential p-value or hardware-design claim."
        pairs.append(pair)
    return pairs


def csv_write(path, rows):
    fields = sorted(set().union(*(r.keys() for r in rows))) if rows else ["status"]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fields)
        writer.writeheader()
        writer.writerows(rows)


def plots(output, runs, contexts):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    labels = [f"{r['terrain']} / {r['shape']} / {r['caster']}" for r in runs]
    for axis, key, title in zip(axes.flat, ("xy_rmse_m", "roll_peak_deg", "pitch_peak_deg", "root_z_peak_to_peak_m"),
                               ("XY tracking RMS (m)", "Peak |body roll| (degrees)", "Peak |body pitch| (degrees)", "World-root Z range (m; includes climb)")):
        values = [flat_metrics(r["metrics"])[key] for r in runs]
        axis.bar(np.arange(len(runs)), [v if v is not None else np.nan for v in values],
                 color=["#2563eb" if r["caster"] == "ball" else "#d97706" for r in runs])
        axis.set(title=title, xticks=np.arange(len(runs)))
        axis.set_xticklabels(labels, rotation=75, ha="right", fontsize=6)
        axis.grid(axis="y", alpha=.2)
    fig.suptitle("Measured motion windows; partial/failing runs remain included. Compare paired common horizons separately.")
    fig.tight_layout()
    fig.savefig(output/"metrics.png", dpi=160)
    plt.close(fig)
    count = max(1, len(runs))
    fig, axes = plt.subplots(math.ceil(count/4), min(4, count), figsize=(14, 3.3*math.ceil(count/4)), squeeze=False)
    for axis, run, context in zip(axes.flat, runs, contexts):
        samples, _, _ = context
        p = np.array([vector(s, ("position_m",), 2) for s in samples]).reshape(-1, 2)
        r = np.array([vector(s, ("reference_m", "reference_position_m"), 2) for s in samples]).reshape(-1, 2)
        axis.plot(r[:, 0], r[:, 1], "--", color="0.5", label="reference")
        axis.plot(p[:, 0], p[:, 1], color="#2563eb" if run["caster"] == "ball" else "#d97706", label="measured")
        axis.set(title=f"{run['terrain']} / {run['shape']}\n{run['caster']}: {run['status']}", xlabel="world X (m)", ylabel="world Y (m)")
        axis.set_aspect("equal", adjustable="datalim")
        axis.grid(alpha=.2)
    for axis in list(axes.flat)[len(runs):]:
        axis.axis("off")
    fig.tight_layout()
    fig.savefig(output/"trajectories.png", dpi=150)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--max-gap", type=float, default=.25)
    parser.add_argument("--min-moving-speed", type=float, default=.03)
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--all-directories", action="store_true", help="Include attempt/pilot folders rather than preferring canonical runs")
    args = parser.parse_args(argv)
    if args.max_gap <= 0 or args.min_moving_speed <= 0:
        parser.error("Thresholds must be positive")
    root = args.root.expanduser().resolve()
    output = (args.output_dir or root/"caster_analysis").expanduser().resolve()
    scan_root = root
    if not args.all_directories:
        for canonical in (root/"replays/runs", root/"runs"):
            if canonical.is_dir():
                scan_root = canonical
                break
    candidates = {p.parent for pattern in ("trajectory.json", "run_summary.json", "measurements.json") for p in scan_root.rglob(pattern)}
    candidates = {p for p in candidates if not p.is_relative_to(output)}
    output.mkdir(parents=True, exist_ok=True)
    runs, contexts = [], []
    for directory in sorted(candidates):
        try:
            run, context = analyze_run(directory, args.max_gap, args.min_moving_speed)
            runs.append(run)
            contexts.append(context)
        except Exception as error:
            runs.append({"run_directory": str(directory), "analysis_error": f"{type(error).__name__}: {error}"})
            contexts.append(None)
    valid = [(r, c) for r, c in zip(runs, contexts) if c is not None]
    pairs = paired_results([r for r, _ in valid], [c for _, c in valid], args.max_gap, args.min_moving_speed)
    report = {"schema_version": 1, "source_root": str(root), "scanned_root": str(scan_root), "analysis_source_sha256": sha256(__file__),
              "settings": {"max_gap_s": args.max_gap, "min_moving_speed_m_s": args.min_moving_speed},
              "runs": runs, "pairs": pairs, "case_aggregate": case_aggregate(pairs),
              "interpretation": "Existing caster assemblies differ in radius, mass, inertia, damping, friction and contact geometry. Metrics describe these simulations, not an isolated topology effect or calibrated hardware outcome."}
    (output/"analysis.json").write_text(json.dumps(finite_json(report), indent=2, allow_nan=False)+"\n")
    (output/"runs.json").write_text(json.dumps(finite_json(runs), indent=2, allow_nan=False)+"\n")
    (output/"paired.json").write_text(json.dumps(finite_json(pairs), indent=2, allow_nan=False)+"\n")
    rows = [{**{k:r[k] for k in ("run_directory", "terrain", "shape", "caster", "trial", "status")},
             **flat_metrics(r["metrics"]), **flat_proxy(r["wheel_center_proxy"]), "physical_failure": r["physical_failure"],
             "infrastructure_failure": r["infrastructure_failure"],
             "command_at_wheel_rate_boundary_fraction": r["metrics"]["command_saturation"]["command_at_wheel_rate_boundary_fraction"],
             **{key+"_fraction":value["mean"] for key,value in r["metrics"]["command_saturation"]["explicit_flags"].items()}} for r, _ in valid]
    rows.extend({"run_directory": r["run_directory"], "status": "analysis_error", "analysis_error": r["analysis_error"]} for r in runs if "analysis_error" in r)
    csv_write(output/"runs.csv", rows)
    csv_write(output/"paired.csv", [{**{k:v for k,v in p.items() if not isinstance(v, (dict, list))},
                                    **{k:"; ".join(map(str, p.get(k, []))) for k in ("condition_mismatches", "missing_condition_fields", "available_models")},
                                    **{f"delta_{k}":v for k,v in p.get("common_horizon_ball_minus_swivel", {}).items()}} for p in pairs])
    if not args.no_plots and valid:
        plots(output, [r for r, _ in valid], [c for _, c in valid])
    print(json.dumps({"output": str(output), "runs": len(runs), "pairs": len(pairs),
                      "analysis_errors": len(runs)-len(valid)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
