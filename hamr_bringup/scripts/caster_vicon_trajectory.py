"""Offline fork-marker motion estimates on the recorded encoder timeline.

Local linear fits use native timestamps within continuous segments. These are
marker trajectories, not ball-center trajectories or predicted encoder angles.
"""
from bisect import bisect_left, bisect_right
import math

import numpy as np
from scipy.spatial.transform import Rotation


DEFAULT_ESTIMATION = dict(window_s=.100, min_span_s=.020,
                          min_samples=3, max_position_step_m=.080)
ROLES = ("H1", "H2", "CENTER")


def estimation_settings(settings):
    if not isinstance(settings, dict) or set(settings) - set(DEFAULT_ESTIMATION):
        raise ValueError("Unknown or invalid trajectory_estimation configuration")
    result = {**DEFAULT_ESTIMATION, **settings}
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) or v <= 0 for v in result.values()):
        raise ValueError("Trajectory estimation settings must be positive finite numbers")
    if type(result["min_samples"]) is not int or result["min_samples"] < 3:
        raise ValueError("min_samples must be an integer of at least three")
    if result["min_span_s"] > result["window_s"]:
        raise ValueError("min_span_s cannot exceed window_s")
    return result


def linear_fit(stamps, values, at_ns):
    """Intercept and slope, subtracting integer nanoseconds before conversion."""
    t = np.asarray([stamp - at_ns for stamp in stamps], dtype=float) / 1e9
    design = np.column_stack((np.ones(len(t)), t))
    return np.linalg.lstsq(design, np.asarray(values, dtype=float), rcond=None)[0]


def enough_samples(stamps, settings):
    return (len(stamps) >= settings["min_samples"]
            and stamps[-1] - stamps[0] >= round(settings["min_span_s"] * 1e9))


def estimate_wheel(rows, streams, quality, settings):
    """Augment aligned rows in place, preserving all raw fields and quality flags."""
    settings = estimation_settings(settings)
    stamps = [row["bag_elapsed_ns"] for row in rows]
    positions = np.array([[row[f"fork_{axis}_m"] for axis in "xyz"] for row in rows])
    finite = np.all(np.isfinite(positions), axis=1)
    position_jumps = [stamps[i] for i in range(1, len(rows))
                      if finite[i-1] and finite[i]
                      and stamps[i] - stamps[i-1] <= round(quality["max_pose_gap_s"] * 1e9)
                      and np.linalg.norm(positions[i] - positions[i-1]) > settings["max_position_step_m"]]
    guard_ns = round(quality["jump_guard_s"] * 1e9)
    groups = []
    previous_valid = False
    for index, row in enumerate(rows):
        stamp = stamps[index]
        pivot = bisect_right(position_jumps, stamp)
        near_jump = any(abs(stamp - jump) <= guard_ns
                        for jump in position_jumps[max(0, pivot-1):pivot+1])
        usable = bool(finite[index] and row["fork_orientation_continuity_ok"] and not near_jump)
        if usable:
            if (not previous_valid or row["fork_segment"] != rows[index-1]["fork_segment"]
                    or stamp - stamps[index-1] > round(quality["max_pose_gap_s"] * 1e9)):
                groups.append([])
            groups[-1].append(index)
        row.update(fork_position_jump=stamp in position_jumps,
                   fork_position_jump_neighborhood=near_jump,
                   trajectory_segment=len(groups)-1 if usable else None,
                   fork_motion_usable=usable, fork_estimate_valid=False,
                   motion_window_start_ns=None, motion_window_end_ns=None,
                   motion_window_samples=0, fork_speed_m_s=None,
                   fork_angular_speed_rad_s=None, fork_segment_distance_m=None,
                   all_encoder_rate_estimates_valid=False)
        for axis in "xyz":
            row[f"fork_estimated_{axis}_m"] = None
            row[f"fork_v{axis}_m_s"] = None
            row[f"fork_w{axis}_rad_s"] = None
        for role in ROLES:
            row[f"{role.lower()}_rate_estimate_valid"] = False
            row[f"{role.lower()}_estimated_rate_rad_s"] = None
        previous_valid = usable

    enc_times = {role: [sample["time_ns"] for sample in streams[role]] for role in ROLES}
    half_window = round(settings["window_s"] * 1e9 / 2)
    max_encoder_gap = round(quality["max_encoder_age_s"] * 1e9)
    total_distance, valid_edges = 0., 0
    for group in groups:
        group_stamps = [stamps[index] for index in group]
        previous_position = None
        distance = 0.
        for index in group:
            row, stamp = rows[index], stamps[index]
            lo = bisect_left(group_stamps, stamp - half_window)
            hi = bisect_right(group_stamps, stamp + half_window)
            window = group[lo:hi]
            times = group_stamps[lo:hi]
            if not enough_samples(times, settings):
                previous_position = None
                continue
            position, velocity = linear_fit(times, positions[window], stamp)
            quaternions = [[rows[i][f"fork_q{axis}"] for axis in "xyzw"] for i in window]
            center_q = [row[f"fork_q{axis}"] for axis in "xyzw"]
            # R(t) R(center)^-1 gives local rotation vectors in the world frame.
            rotations = Rotation.from_quat(quaternions) * Rotation.from_quat(center_q).inv()
            _, omega = linear_fit(times, rotations.as_rotvec(), stamp)
            row.update(fork_estimate_valid=True, motion_window_start_ns=times[0],
                       motion_window_end_ns=times[-1], motion_window_samples=len(times),
                       fork_speed_m_s=float(np.linalg.norm(velocity)),
                       fork_angular_speed_rad_s=float(np.linalg.norm(omega)))
            for axis, p, v, w in zip("xyz", position, velocity, omega):
                row[f"fork_estimated_{axis}_m"] = float(p)
                row[f"fork_v{axis}_m_s"] = float(v)
                row[f"fork_w{axis}_rad_s"] = float(w)
            if previous_position is not None:
                step = float(np.linalg.norm(position - previous_position))
                distance += step
                total_distance += step
                valid_edges += 1
            row["fork_segment_distance_m"] = distance
            previous_position = position
            for role in ROLES:
                times_native = enc_times[role]
                left = bisect_left(times_native, times[0])
                right = bisect_right(times_native, times[-1])
                samples = streams[role][left:right]
                native = times_native[left:right]
                values = [sample["change_rad"] for sample in samples]
                # Never differentiate the repeated latest-sample alignment, or
                # fit through a missing/invalid native encoder interval.
                valid = (row[f"{role.lower()}_fresh"] and enough_samples(native, settings)
                         and np.all(np.isfinite(values))
                         and all(b-a <= max_encoder_gap for a, b in zip(native, native[1:])))
                if valid:
                    _, rate = linear_fit(native, values, stamp)
                    row[f"{role.lower()}_rate_estimate_valid"] = True
                    row[f"{role.lower()}_estimated_rate_rad_s"] = float(rate)
            row["all_encoder_rate_estimates_valid"] = all(
                row[f"{role.lower()}_rate_estimate_valid"] for role in ROLES)
    speeds = [row["fork_speed_m_s"] for row in rows if row["fork_estimate_valid"]]
    return dict(segments=len(groups), position_jump_count=len(position_jumps),
                estimated_rows=len(speeds),
                rows_with_all_encoder_rates=sum(row["all_encoder_rate_estimates_valid"] for row in rows),
                estimated_distance_over_valid_segments_m=total_distance if valid_edges else None,
                peak_estimated_marker_speed_m_s=max(speeds, default=None))


def estimate_trajectories(aligned, streams, quality, settings=None):
    settings = estimation_settings(settings or {})
    report = {
        "settings": settings,
        "quantity": "Fork Vicon marker position and world-frame velocity/angular velocity; uncalibrated.",
        "method": "Centered local linear least-squares fits on native timestamps, clipped to continuous segments; offline, uses future samples. Encoder rates use native unwrapped histories over the same window.",
        "mask_policy": "No fit across pose gaps, orientation jumps/guard neighborhoods, nonfinite poses or position jumps/guards. Encoder fits reject nonfinite changes, stale alignment and gaps exceeding max_encoder_age_s.",
        "limitations": "No marker-to-fork or ball-center transform, latency fit, rolling model, encoder error, or accuracy claim. Continuous Vicon orientations can still be ambiguous. Distance omits masked intervals; smoothing reduces noise but does not establish accuracy.",
        "wheels": {},
    }
    for wheel, rows in aligned.items():
        report["wheels"][wheel] = estimate_wheel(
            rows, {role: streams[wheel, role] for role in ROLES}, quality, settings)
    return report


def plot_trajectories(output, aligned):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    def curve(rows, x_key, y_key):
        x, y, previous = [], [], None
        for row in rows:
            if row["trajectory_segment"] != previous:
                x.append(np.nan)
                y.append(np.nan)
            x.append(row[x_key] if row[x_key] is not None else np.nan)
            y.append(row[y_key] if row[y_key] is not None else np.nan)
            previous = row["trajectory_segment"]
        return x, y

    fig, axes = plt.subplots(4, 2, figsize=(14, 13), layout="constrained")
    for column, (wheel, rows) in enumerate(aligned.items()):
        axes[0, column].plot(*curve(rows, "fork_estimated_x_m", "fork_estimated_y_m"), lw=1)
        axes[0, column].set(title=wheel.replace("_", " ").title(),
                            xlabel="Marker x (m)", ylabel="Marker y (m)")
        axes[0, column].set_aspect("equal", adjustable="datalim")
        for axis, field, label in ((axes[1, column], "fork_speed_m_s", "Marker speed (m/s)"),
                                   (axes[2, column], "fork_angular_speed_rad_s", "Marker angular speed (rad/s)")):
            axis.plot(*curve(rows, "bag_time_s", field), lw=1)
            axis.set(ylabel=label, xlabel="Bag elapsed time (s)")
        for role in ROLES:
            axes[3, column].plot(*curve(rows, "bag_time_s", f"{role.lower()}_estimated_rate_rad_s"),
                                 lw=1, label=role)
        axes[3, column].set(xlabel="Bag elapsed time (s)", ylabel="Encoder rate (rad/s)")
        axes[3, column].legend()
    for axis in axes.flat:
        axis.grid(alpha=.2)
    fig.suptitle("Fork trajectory estimates alongside caster encoder history\nUncalibrated marker motion; masked gaps omitted; no encoder prediction/error")
    fig.savefig(output / "fork_trajectory_comparison.png", dpi=150)
    plt.close(fig)
