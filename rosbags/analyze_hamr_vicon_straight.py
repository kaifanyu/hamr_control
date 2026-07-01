#!/usr/bin/env python3
"""Analyze a HAMR localization-test rosbag.

The hardware tests record Vicon ground truth plus onboard odometry. This script
keeps the straight-line Vicon summary, and also time-aligns onboard odometry to
Vicon so localization drift can be plotted and measured for teleop or autonomy.
"""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


DEFAULT_BASE_TOPIC = "/HAMR_base/odom"
DEFAULT_ONBOARD_TOPIC = "/local_HAMR/odom"
DEFAULT_WHEEL_ODOM_TOPIC = "/wheel_odom"
DEFAULT_IMU_TOPIC = "/imu/data"
DEFAULT_TURRET_TOPIC = "/HAMR_turret/odom"
DEFAULT_REFERENCE_TOPIC = "/reference_trajectory"
DEFAULT_LEFT_WHEEL_TOPIC = "/left_wheel/cmd_vel"
DEFAULT_RIGHT_WHEEL_TOPIC = "/right_wheel/cmd_vel"
DEFAULT_CALIB_STATUS_TOPIC = "/imu/calib_status"
DEFAULT_WHEEL_RADIUS_M = 0.122


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def detect_storage_id(bag_dir):
    metadata = bag_dir / "metadata.yaml"
    if metadata.exists():
        for line in metadata.read_text(encoding="utf-8").splitlines():
            stripped = line.strip()
            if stripped.startswith("storage_identifier:"):
                return stripped.split(":", 1)[1].strip()
            if stripped.startswith("storage_id:"):
                return stripped.split(":", 1)[1].strip()

    if any(bag_dir.glob("*.mcap")):
        return "mcap"
    if any(bag_dir.glob("*.db3")):
        return "sqlite3"
    return "mcap"


def open_reader(bag_dir, storage_id):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        ConverterOptions("", ""),
    )
    return reader


def pose_from_message(msg):
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        return msg.pose.pose
    if hasattr(msg, "pose"):
        return msg.pose
    raise TypeError(f"Unsupported pose message: {type(msg).__name__}")


def read_bag(
    bag_dir,
    storage_id,
    base_topic,
    onboard_topic,
    wheel_odom_topic,
    imu_topic,
    turret_topic,
    reference_topic,
    left_wheel_topic,
    right_wheel_topic,
    calib_status_topic,
):
    reader = open_reader(bag_dir, storage_id)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    base_samples = []
    onboard_samples = []
    wheel_odom_samples = []
    imu_samples = []
    turret_samples = []
    reference_samples = []
    left_wheel_samples = []
    right_wheel_samples = []
    calib_samples = []

    wanted = {
        base_topic,
        onboard_topic,
        wheel_odom_topic,
        imu_topic,
        turret_topic,
        reference_topic,
        left_wheel_topic,
        right_wheel_topic,
        calib_status_topic,
    }
    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        if topic not in wanted:
            continue

        msg_class = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_class)
        t = bag_time_ns * 1e-9

        if topic in (base_topic, onboard_topic, wheel_odom_topic, turret_topic):
            pose = pose_from_message(msg)
            sample = {
                "t": t,
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
                "yaw": float(yaw_from_quaternion(pose.orientation)),
            }
            if hasattr(msg, "twist") and hasattr(msg.twist, "twist"):
                tw = msg.twist.twist
                sample["vx"] = float(tw.linear.x)
                sample["vy"] = float(tw.linear.y)
                sample["wz"] = float(tw.angular.z)
            if topic == base_topic:
                base_samples.append(sample)
            elif topic == onboard_topic:
                onboard_samples.append(sample)
            elif topic == wheel_odom_topic:
                wheel_odom_samples.append(sample)
            else:
                turret_samples.append(sample)
        elif topic == imu_topic:
            imu_samples.append(
                {
                    "t": t,
                    "yaw": float(yaw_from_quaternion(msg.orientation)),
                    "ax": float(msg.linear_acceleration.x),
                    "ay": float(msg.linear_acceleration.y),
                    "az": float(msg.linear_acceleration.z),
                    "gz": float(msg.angular_velocity.z),
                }
            )
        elif topic == reference_topic:
            reference_samples.append(
                {
                    "t": t,
                    "x": float(msg.x),
                    "y": float(msg.y),
                    "yaw": float(msg.yaw),
                    "x_dot": float(msg.x_dot),
                    "y_dot": float(msg.y_dot),
                    "yaw_dot": float(msg.yaw_dot),
                }
            )
        elif topic == left_wheel_topic:
            left_wheel_samples.append({"t": t, "value": float(msg.data)})
        elif topic == right_wheel_topic:
            right_wheel_samples.append({"t": t, "value": float(msg.data)})
        elif topic == calib_status_topic:
            data = bytes(msg.data)
            calib_samples.append({
                "t": t,
                "sys":  data[0] if len(data) > 0 else 0,
                "gyro": data[1] if len(data) > 1 else 0,
                "accel":data[2] if len(data) > 2 else 0,
                "mag":  data[3] if len(data) > 3 else 0,
            })

    return (
        topic_types,
        base_samples,
        onboard_samples,
        wheel_odom_samples,
        imu_samples,
        turret_samples,
        reference_samples,
        left_wheel_samples,
        right_wheel_samples,
        calib_samples,
    )

def as_arrays(samples):
    return {
        "t": np.asarray([s["t"] for s in samples], dtype=float),
        "x": np.asarray([s["x"] for s in samples], dtype=float),
        "y": np.asarray([s["y"] for s in samples], dtype=float),
        "z": np.asarray([s["z"] for s in samples], dtype=float),
        "yaw": np.asarray([s["yaw"] for s in samples], dtype=float),
    }


def start_relative_path(samples, align_yaw=True):
    arr = as_arrays(samples)
    x0 = arr["x"][0]
    y0 = arr["y"][0]
    yaw0 = arr["yaw"][0] if align_yaw else 0.0

    dx = arr["x"] - x0
    dy = arr["y"] - y0
    c = math.cos(-yaw0)
    s = math.sin(-yaw0)

    rel_x = c * dx - s * dy
    rel_y = s * dx + c * dy
    rel_yaw = np.unwrap(arr["yaw"]) - arr["yaw"][0]
    rel_yaw = np.asarray([wrap_angle(v) for v in rel_yaw], dtype=float)

    return arr, rel_x, rel_y, rel_yaw


def path_length(x_vals, y_vals):
    if len(x_vals) < 2:
        return 0.0
    dx = np.diff(x_vals)
    dy = np.diff(y_vals)
    return float(np.sum(np.hypot(dx, dy)))


def summarize_base(samples, target_distance, align_yaw):
    arr, rel_x, rel_y, rel_yaw = start_relative_path(samples, align_yaw=align_yaw)

    duration = float(arr["t"][-1] - arr["t"][0])
    traveled = path_length(arr["x"], arr["y"])
    final_forward = float(rel_x[-1])
    final_lateral = float(rel_y[-1])
    final_distance = float(math.hypot(rel_x[-1], rel_y[-1]))

    return {
        "sample_count": int(len(samples)),
        "duration_s": duration,
        "start": {
            "x_m": float(arr["x"][0]),
            "y_m": float(arr["y"][0]),
            "z_m": float(arr["z"][0]),
            "yaw_rad": float(arr["yaw"][0]),
        },
        "end": {
            "x_m": float(arr["x"][-1]),
            "y_m": float(arr["y"][-1]),
            "z_m": float(arr["z"][-1]),
            "yaw_rad": float(arr["yaw"][-1]),
        },
        "target_distance_m": float(target_distance),
        "path_length_m": traveled,
        "mean_speed_mps": float(traveled / duration) if duration > 0.0 else 0.0,
        "final_forward_m": final_forward,
        "final_lateral_m": final_lateral,
        "final_displacement_m": final_distance,
        "forward_error_m": float(final_forward - target_distance),
        "distance_error_m": float(final_distance - target_distance),
        "lateral_rmse_m": float(np.sqrt(np.mean(rel_y**2))),
        "lateral_abs_max_m": float(np.max(np.abs(rel_y))),
        "yaw_drift_final_rad": float(rel_yaw[-1]),
        "yaw_drift_rmse_rad": float(np.sqrt(np.mean(rel_yaw**2))),
        "yaw_drift_abs_max_rad": float(np.max(np.abs(rel_yaw))),
    }


def nearest_reference_errors(base_samples, reference_samples, max_dt):
    if not reference_samples:
        return None

    ref_t = [s["t"] for s in reference_samples]
    matched = []

    for base in base_samples:
        idx = bisect.bisect_left(ref_t, base["t"])
        candidates = []
        if idx < len(reference_samples):
            candidates.append(reference_samples[idx])
        if idx > 0:
            candidates.append(reference_samples[idx - 1])
        if not candidates:
            continue

        ref = min(candidates, key=lambda s: abs(s["t"] - base["t"]))
        dt = abs(ref["t"] - base["t"])
        if dt > max_dt:
            continue

        matched.append(
            {
                "dt": dt,
                "err_x": ref["x"] - base["x"],
                "err_y": ref["y"] - base["y"],
                "err_yaw": wrap_angle(ref["yaw"] - base["yaw"]),
            }
        )

    if not matched:
        return None

    err_x = np.asarray([m["err_x"] for m in matched], dtype=float)
    err_y = np.asarray([m["err_y"] for m in matched], dtype=float)
    err_yaw = np.asarray([m["err_yaw"] for m in matched], dtype=float)
    err_xy = np.hypot(err_x, err_y)

    return {
        "matched_sample_count": int(len(matched)),
        "max_match_dt_s": float(max(m["dt"] for m in matched)),
        "xy_rmse_m": float(np.sqrt(np.mean(err_xy**2))),
        "x_rmse_m": float(np.sqrt(np.mean(err_x**2))),
        "y_rmse_m": float(np.sqrt(np.mean(err_y**2))),
        "yaw_rmse_rad": float(np.sqrt(np.mean(err_yaw**2))),
        "final_err_x_m": float(err_x[-1]),
        "final_err_y_m": float(err_y[-1]),
        "final_err_xy_m": float(err_xy[-1]),
        "final_err_yaw_rad": float(err_yaw[-1]),
    }



def add_relative_fields(samples, align_yaw=True):
    if not samples:
        return []

    _, rel_x, rel_y, rel_yaw = start_relative_path(samples, align_yaw=align_yaw)
    relative = []
    for sample, x_rel, y_rel, yaw_rel in zip(samples, rel_x, rel_y, rel_yaw):
        item = dict(sample)
        item["rel_x"] = float(x_rel)
        item["rel_y"] = float(y_rel)
        item["rel_yaw"] = float(yaw_rel)
        relative.append(item)
    return relative


def relative_fields_from_origin(samples, origin_sample, align_yaw=True):
    if not samples:
        return []

    x0 = origin_sample["x"]
    y0 = origin_sample["y"]
    yaw0 = origin_sample["yaw"] if align_yaw else 0.0
    c = math.cos(-yaw0)
    s = math.sin(-yaw0)

    relative = []
    for sample in samples:
        dx = sample["x"] - x0
        dy = sample["y"] - y0
        item = dict(sample)
        item["rel_x"] = float(c * dx - s * dy)
        item["rel_y"] = float(s * dx + c * dy)
        item["rel_yaw"] = float(wrap_angle(sample["yaw"] - yaw0))
        relative.append(item)
    return relative


def integrate_imu_odom_samples(imu_samples, bias_window_s=1.0, max_dt=0.1):
    """Dead-reckon x/y/yaw from IMU orientation + linear acceleration only.

    This intentionally does not use wheels or Vicon. It is useful for seeing
    how quickly pure inertial position drifts relative to wheel odom and EKF.
    """
    if not imu_samples:
        return []

    t = np.asarray([s["t"] for s in imu_samples], dtype=float)
    yaw = np.unwrap(np.asarray([s["yaw"] for s in imu_samples], dtype=float))
    ax = np.asarray([s["ax"] for s in imu_samples], dtype=float)
    ay = np.asarray([s["ay"] for s in imu_samples], dtype=float)

    bias_mask = t <= (t[0] + max(0.0, bias_window_s))
    if not np.any(bias_mask):
        bias_mask = np.asarray([True] + [False] * (len(t) - 1))
    ax_bias = float(np.mean(ax[bias_mask]))
    ay_bias = float(np.mean(ay[bias_mask]))

    def world_accel(index):
        ax_body = ax[index] - ax_bias
        ay_body = ay[index] - ay_bias
        c = math.cos(yaw[index])
        s = math.sin(yaw[index])
        return (
            c * ax_body - s * ay_body,
            s * ax_body + c * ay_body,
        )

    samples = []
    x = y_pos = vx = vy = 0.0
    prev_ax_w, prev_ay_w = world_accel(0)
    samples.append({"t": float(t[0]), "x": x, "y": y_pos, "z": 0.0, "yaw": wrap_angle(yaw[0])})

    for i in range(1, len(imu_samples)):
        dt = float(t[i] - t[i - 1])
        if dt <= 0.0:
            continue

        curr_ax_w, curr_ay_w = world_accel(i)
        if dt <= max_dt:
            ax_w = 0.5 * (prev_ax_w + curr_ax_w)
            ay_w = 0.5 * (prev_ay_w + curr_ay_w)
            x += vx * dt + 0.5 * ax_w * dt * dt
            y_pos += vy * dt + 0.5 * ay_w * dt * dt
            vx += ax_w * dt
            vy += ay_w * dt

        prev_ax_w, prev_ay_w = curr_ax_w, curr_ay_w
        samples.append(
            {
                "t": float(t[i]),
                "x": float(x),
                "y": float(y_pos),
                "z": 0.0,
                "yaw": float(wrap_angle(yaw[i])),
            }
        )

    return samples


def integrate_gyro_z(imu_samples, max_dt=0.1):
    """Numerically integrate angular_velocity.z to produce a cumulative yaw time series.

    Returns samples with {"t", "yaw"} where yaw is the unwrapped accumulated angle
    in radians, zeroed at the first sample.  Unlike the BNO055 fused orientation this
    contains no magnetometer correction — useful for isolating gyro-only drift.
    """
    if not imu_samples:
        return []
    result = [{"t": imu_samples[0]["t"], "yaw": 0.0}]
    yaw = 0.0
    for i in range(1, len(imu_samples)):
        dt = imu_samples[i]["t"] - imu_samples[i - 1]["t"]
        if 0.0 < dt <= max_dt:
            yaw += imu_samples[i]["gz"] * dt
        result.append({"t": imu_samples[i]["t"], "yaw": yaw})
    return result


def write_calib_status_plot(plot_path, calib_samples):
    """Plot BNO055 sys/gyro/accel/mag calibration levels (0–3) over the run."""
    import matplotlib.pyplot as plt

    if not calib_samples:
        raise RuntimeError("No /imu/calib_status samples found.")

    t0 = calib_samples[0]["t"]
    t     = np.asarray([s["t"]    - t0 for s in calib_samples], dtype=float)
    sys_  = np.asarray([s["sys"]       for s in calib_samples], dtype=float)
    gyro  = np.asarray([s["gyro"]      for s in calib_samples], dtype=float)
    accel = np.asarray([s["accel"]     for s in calib_samples], dtype=float)
    mag   = np.asarray([s["mag"]       for s in calib_samples], dtype=float)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.step(t, sys_,  where="post", label="sys",   linewidth=1.8)
    ax.step(t, gyro,  where="post", label="gyro",  linewidth=1.8)
    ax.step(t, accel, where="post", label="accel", linewidth=1.8)
    ax.step(t, mag,   where="post", label="mag",   linewidth=1.8)
    ax.axhline(3, color="gray", linestyle="--", linewidth=0.8, alpha=0.5, label="fully calibrated")
    ax.set_ylim(-0.2, 3.4)
    ax.set_yticks([0, 1, 2, 3])
    ax.set_xlabel("time (s)")
    ax.set_ylabel("calibration level (0–3)")
    ax.set_title("BNO055 calibration status (check_sys) over run")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)


def write_heading_comparison_plot(
    plot_path,
    base_samples,
    imu_samples,
    wheel_odom_samples,
    gyro_integrated_samples,
):
    """Compare all heading sources on a single start-zeroed plot (degrees).

    Plots Vicon yaw, BNO055 fused yaw, gyro-z integrated yaw, and
    wheel-integrated yaw — all unwrapped and zeroed at their first sample so
    divergence is immediately visible.
    """
    import matplotlib.pyplot as plt

    if not imu_samples:
        raise RuntimeError("No IMU samples for heading comparison plot.")

    def unwrap_deg(samples, key="yaw"):
        t0 = samples[0]["t"]
        t   = np.asarray([s["t"]   - t0 for s in samples], dtype=float)
        raw = np.asarray([s[key]        for s in samples], dtype=float)
        rel = np.degrees(np.unwrap(raw) - np.unwrap(raw)[0])
        return t, rel

    fig, ax = plt.subplots(figsize=(10, 5))

    if base_samples:
        t, y = unwrap_deg(base_samples)
        ax.plot(t, y, label="Vicon yaw (ground truth)", linewidth=2.2)

    t, y = unwrap_deg(imu_samples)
    ax.plot(t, y, label="IMU fused yaw (BNO055 NDOF)", linewidth=1.8)

    if gyro_integrated_samples:
        t, y = unwrap_deg(gyro_integrated_samples)
        ax.plot(t, y, label="Gyro-z integrated yaw", linewidth=1.6, linestyle="--")

    if wheel_odom_samples:
        t, y = unwrap_deg(wheel_odom_samples)
        ax.plot(t, y, label="Wheel-integrated yaw", linewidth=1.4, linestyle=":")

    ax.set_xlabel("time from start (s)")
    ax.set_ylabel("heading change (deg, start-zeroed)")
    ax.set_title("Heading source comparison: Vicon / IMU fused / gyro-z integral / wheel")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)


def matched_relative_samples(reference_samples, estimate_samples, max_dt, align_yaw=True):
    if not reference_samples or not estimate_samples:
        return []

    reference_rel = add_relative_fields(reference_samples, align_yaw=align_yaw)
    estimate_rel = add_relative_fields(estimate_samples, align_yaw=align_yaw)
    estimate_t = [s["t"] for s in estimate_rel]
    matched = []

    for ref in reference_rel:
        idx = bisect.bisect_left(estimate_t, ref["t"])
        candidates = []
        if idx < len(estimate_rel):
            candidates.append(estimate_rel[idx])
        if idx > 0:
            candidates.append(estimate_rel[idx - 1])
        if not candidates:
            continue

        est = min(candidates, key=lambda s: abs(s["t"] - ref["t"]))
        dt = est["t"] - ref["t"]
        if abs(dt) > max_dt:
            continue

        err_x = est["rel_x"] - ref["rel_x"]
        err_y = est["rel_y"] - ref["rel_y"]
        err_yaw = wrap_angle(est["rel_yaw"] - ref["rel_yaw"])
        matched.append(
            {
                "t": ref["t"],
                "dt": dt,
                "reference": ref,
                "estimate": est,
                "err_x": err_x,
                "err_y": err_y,
                "err_xy": math.hypot(err_x, err_y),
                "err_yaw": err_yaw,
            }
        )

    return matched


def summarize_trajectory_errors(matches):
    if not matches:
        return None

    err_x = np.asarray([m["err_x"] for m in matches], dtype=float)
    err_y = np.asarray([m["err_y"] for m in matches], dtype=float)
    err_xy = np.asarray([m["err_xy"] for m in matches], dtype=float)
    err_yaw = np.asarray([m["err_yaw"] for m in matches], dtype=float)

    return {
        "matched_sample_count": int(len(matches)),
        "duration_s": float(matches[-1]["t"] - matches[0]["t"]),
        "max_match_dt_s": float(max(abs(m["dt"]) for m in matches)),
        "xy_rmse_m": float(np.sqrt(np.mean(err_xy**2))),
        "x_rmse_m": float(np.sqrt(np.mean(err_x**2))),
        "y_rmse_m": float(np.sqrt(np.mean(err_y**2))),
        "xy_abs_max_m": float(np.max(np.abs(err_xy))),
        "x_abs_max_m": float(np.max(np.abs(err_x))),
        "y_abs_max_m": float(np.max(np.abs(err_y))),
        "yaw_rmse_rad": float(np.sqrt(np.mean(err_yaw**2))),
        "yaw_abs_max_rad": float(np.max(np.abs(err_yaw))),
        "final_err_x_m": float(err_x[-1]),
        "final_err_y_m": float(err_y[-1]),
        "final_err_xy_m": float(err_xy[-1]),
        "final_err_yaw_rad": float(err_yaw[-1]),
        "error_sign_convention": "estimate_minus_vicon_in_start_aligned_frame",
    }


def write_localization_csv(csv_path, matches):
    if not matches:
        raise RuntimeError("No matched onboard/Vicon samples found for localization CSV.")

    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "t_s",
                "match_dt_s",
                "vicon_x_m",
                "vicon_y_m",
                "vicon_yaw_rad",
                "onboard_x_m",
                "onboard_y_m",
                "onboard_yaw_rad",
                "err_x_m",
                "err_y_m",
                "err_xy_m",
                "err_yaw_rad",
            ]
        )
        for match in matches:
            ref = match["reference"]
            est = match["estimate"]
            writer.writerow(
                [
                    match["t"],
                    match["dt"],
                    ref["rel_x"],
                    ref["rel_y"],
                    ref["rel_yaw"],
                    est["rel_x"],
                    est["rel_y"],
                    est["rel_yaw"],
                    match["err_x"],
                    match["err_y"],
                    match["err_xy"],
                    match["err_yaw"],
                ]
            )

def write_csv(csv_path, samples, rel_x, rel_y, rel_yaw):
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "t_s",
                "x_m",
                "y_m",
                "z_m",
                "yaw_rad",
                "start_frame_x_m",
                "start_frame_y_m",
                "start_frame_yaw_rad",
            ]
        )
        for sample, x_rel, y_rel, yaw_rel in zip(samples, rel_x, rel_y, rel_yaw):
            writer.writerow(
                [
                    sample["t"],
                    sample["x"],
                    sample["y"],
                    sample["z"],
                    sample["yaw"],
                    x_rel,
                    y_rel,
                    yaw_rel,
                ]
            )


def write_pose_trace_csv(csv_path, samples, align_yaw=True):
    if not samples:
        raise RuntimeError(f"No samples available for {csv_path}.")
    _, rel_x, rel_y, rel_yaw = start_relative_path(samples, align_yaw=align_yaw)
    write_csv(csv_path, samples, rel_x, rel_y, rel_yaw)


def reference_arrays(reference_samples):
    return {
        "t": np.asarray([s["t"] for s in reference_samples], dtype=float),
        "x": np.asarray([s["x"] for s in reference_samples], dtype=float),
        "y": np.asarray([s["y"] for s in reference_samples], dtype=float),
        "yaw": np.asarray([s["yaw"] for s in reference_samples], dtype=float),
    }


def scalar_arrays(samples):
    return {
        "t": np.asarray([s["t"] for s in samples], dtype=float),
        "value": np.asarray([s["value"] for s in samples], dtype=float),
    }


def sample_indices(count, max_count):
    if count <= 0:
        return np.asarray([], dtype=int)
    if count <= max_count:
        return np.arange(count, dtype=int)
    return np.unique(np.linspace(0, count - 1, max_count, dtype=int))


def nearest_yaw_samples(base_samples, turret_samples, max_dt):
    if not turret_samples:
        return []

    turret_t = [s["t"] for s in turret_samples]
    matched = []
    for base in base_samples:
        idx = bisect.bisect_left(turret_t, base["t"])
        candidates = []
        if idx < len(turret_samples):
            candidates.append(turret_samples[idx])
        if idx > 0:
            candidates.append(turret_samples[idx - 1])
        if not candidates:
            continue

        turret = min(candidates, key=lambda s: abs(s["t"] - base["t"]))
        if abs(turret["t"] - base["t"]) <= max_dt:
            matched.append(
                {
                    "x": base["x"],
                    "y": base["y"],
                    "yaw": turret["yaw"],
                }
            )

    return matched


def write_wheel_plot(
    plot_path,
    left_samples,
    right_samples,
    base_samples,
    left_topic,
    right_topic,
    wheel_radius_m,
    smooth_window,
):
    import matplotlib.pyplot as plt

    if not left_samples and not right_samples:
        raise RuntimeError("No wheel command samples found to plot.")

    t0_candidates = []
    if base_samples:
        t0_candidates.append(base_samples[0]["t"])
    if left_samples:
        t0_candidates.append(left_samples[0]["t"])
    if right_samples:
        t0_candidates.append(right_samples[0]["t"])
    t0 = min(t0_candidates)

    tv, actual_velocity, _ = vicon_forward_speed(base_samples)
    smooth_velocity = None
    if tv.size and smooth_window and smooth_window > 1:
        smooth_velocity = moving_average(actual_velocity, smooth_window)

    fig, ax = plt.subplots(figsize=(10, 5.4))
    if tv.size:
        ax.plot(
            tv - t0,
            actual_velocity,
            color="tab:green",
            alpha=0.28 if smooth_velocity is not None else 0.9,
            linewidth=1.0,
            label="Vicon actual forward velocity",
        )
        if smooth_velocity is not None:
            ax.plot(
                tv - t0,
                smooth_velocity,
                color="tab:green",
                linewidth=2.4,
                label=f"Vicon actual forward velocity smoothed (w={smooth_window})",
            )

    wheel_configs = (
        (left_samples, left_topic, "tab:blue"),
        (right_samples, right_topic, "tab:orange"),
    )
    for samples, topic, color in wheel_configs:
        if not samples:
            continue
        values = scalar_arrays(samples)
        ax.step(
            values["t"] - t0,
            values["value"] * wheel_radius_m,
            where="post",
            color=color,
            linewidth=1.5,
            label=f"{topic} command * r ({wheel_radius_m:g} m)",
        )

    ax.axhline(0.0, color="0.35", linewidth=0.8, alpha=0.5)
    ax.set_xlabel("time from first sample (s)")
    ax.set_ylabel("velocity (m/s)")
    ax.set_title("HAMR actual Vicon velocity vs commanded wheel velocity")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)


def write_turret_yaw_plot(
    plot_path,
    base_samples,
    turret_samples,
    reference_samples,
    max_dt,
    arrow_count,
    arrow_length,
):
    import matplotlib.pyplot as plt

    if not turret_samples:
        raise RuntimeError("No turret samples found to plot yaw orientation.")

    base = as_arrays(base_samples)
    yaw_samples = nearest_yaw_samples(base_samples, turret_samples, max_dt)
    if not yaw_samples:
        raise RuntimeError(
            f"No turret yaw samples matched base timestamps within {max_dt:.3f} s."
        )

    arrow_idx = sample_indices(len(yaw_samples), arrow_count)
    arrow_x = np.asarray([yaw_samples[i]["x"] for i in arrow_idx], dtype=float)
    arrow_y = np.asarray([yaw_samples[i]["y"] for i in arrow_idx], dtype=float)
    arrow_yaw = np.asarray([yaw_samples[i]["yaw"] for i in arrow_idx], dtype=float)
    arrow_u = np.cos(arrow_yaw) * arrow_length
    arrow_v = np.sin(arrow_yaw) * arrow_length

    plt.figure(figsize=(8, 5))
    plt.plot(base["x"], base["y"], label="Actual base trajectory", linewidth=2.0)
    if reference_samples:
        ref = reference_arrays(reference_samples)
        plt.plot(ref["x"], ref["y"], "--", label="Reference trajectory", linewidth=1.5)
    plt.quiver(
        arrow_x,
        arrow_y,
        arrow_u,
        arrow_v,
        angles="xy",
        scale_units="xy",
        scale=1,
        width=0.004,
        color="tab:red",
        label="Turret yaw",
    )
    plt.scatter(
        [base["x"][0], base["x"][-1]],
        [base["y"][0], base["y"][-1]],
        c=["green", "red"],
    )
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("HAMR actual/reference trajectory with turret yaw")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_path, dpi=160)
    plt.close()


def write_plot(plot_path, samples, rel_x, rel_y, target_distance, reference_samples=None):
    import matplotlib.pyplot as plt

    plt.figure(figsize=(8, 5))
    if reference_samples:
        base = as_arrays(samples)
        ref = reference_arrays(reference_samples)
        plot_x = base["x"]
        plot_y = base["y"]
        plt.plot(plot_x, plot_y, label="Vicon base path", linewidth=2.0)
        plt.plot(ref["x"], ref["y"], "--", label="Reference trajectory", linewidth=1.5)
    else:
        plot_x = rel_x
        plot_y = rel_y
        plt.plot(plot_x, plot_y, label="Vicon base path", linewidth=2.0)
        plt.plot([0.0, target_distance], [0.0, 0.0], "k--", label="2 m target")
    plt.scatter([plot_x[0], plot_x[-1]], [plot_y[0], plot_y[-1]], c=["green", "red"])
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("HAMR path and reference trajectory")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_path, dpi=160)
    plt.close()



def write_localization_plot(
    plot_path,
    base_samples,
    onboard_samples,
    wheel_odom_samples,
    imu_odom_samples,
    reference_samples,
    align_yaw,
    base_topic,
    onboard_topic,
    wheel_odom_topic,
    imu_topic,
):
    import matplotlib.pyplot as plt

    if not onboard_samples:
        raise RuntimeError(f"No onboard odometry samples found on {onboard_topic}.")

    base_rel = add_relative_fields(base_samples, align_yaw=align_yaw)
    onboard_rel = add_relative_fields(onboard_samples, align_yaw=align_yaw)
    wheel_rel = add_relative_fields(wheel_odom_samples, align_yaw=align_yaw)
    imu_rel = None
    reference_rel = relative_fields_from_origin(
        reference_samples, base_samples[0], align_yaw=align_yaw
    )

    plt.figure(figsize=(8.5, 5.5))
    if reference_rel:
        plt.plot(
            [s["rel_x"] for s in reference_rel],
            [s["rel_y"] for s in reference_rel],
            "k--",
            label="Reference",
            linewidth=1.6,
            alpha=0.9,
        )
    plt.plot(
        [s["rel_x"] for s in base_rel],
        [s["rel_y"] for s in base_rel],
        label=f"Vicon {base_topic}",
        linewidth=2.2,
    )
    plt.plot(
        [s["rel_x"] for s in onboard_rel],
        [s["rel_y"] for s in onboard_rel],
        label=f"Onboard {onboard_topic}",
        linewidth=2.0,
    )
    if wheel_rel:
        plt.plot(
            [s["rel_x"] for s in wheel_rel],
            [s["rel_y"] for s in wheel_rel],
            label=f"Wheel {wheel_odom_topic}",
            linewidth=1.4,
            alpha=0.8,
        )
    if imu_rel:
        plt.plot(
            [s["rel_x"] for s in imu_rel],
            [s["rel_y"] for s in imu_rel],
            label=f"IMU dead reckoning {imu_topic}",
            linewidth=1.2,
            alpha=0.75,
        )

    plt.scatter([base_rel[0]["rel_x"]], [base_rel[0]["rel_y"]], c="green", s=36, label="start")
    plt.scatter([base_rel[-1]["rel_x"]], [base_rel[-1]["rel_y"]], c="red", s=36, label="vicon end")
    plt.scatter(
        [onboard_rel[-1]["rel_x"]],
        [onboard_rel[-1]["rel_y"]],
        c="orange",
        s=36,
        label="onboard end",
    )
    core_paths = [base_rel, onboard_rel, wheel_rel, reference_rel]
    core_x = [s["rel_x"] for path in core_paths for s in path]
    core_y = [s["rel_y"] for path in core_paths for s in path]
    if core_x and core_y:
        x_span = max(core_x) - min(core_x)
        y_span = max(core_y) - min(core_y)
        pad = max(0.25, 0.08 * max(x_span, y_span, 1e-9))
        plt.xlim(min(core_x) - pad, max(core_x) + pad)
        plt.ylim(min(core_y) - pad, max(core_y) + pad)

    plt.xlabel("start-aligned x (m)")
    plt.ylabel("start-aligned y (m)")
    plt.title("HAMR Vicon, reference, EKF, wheel, and IMU odometry")
    plt.gca().set_aspect("equal", adjustable="box")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_path, dpi=160)
    plt.close()


def write_imu_odom_plot(
    plot_path,
    base_samples,
    imu_odom_samples,
    align_yaw,
    base_topic,
    imu_topic,
):
    import matplotlib.pyplot as plt

    if not imu_odom_samples:
        raise RuntimeError(f"No IMU samples found on {imu_topic}.")

    base_rel = add_relative_fields(base_samples, align_yaw=align_yaw)
    imu_rel = add_relative_fields(imu_odom_samples, align_yaw=align_yaw)

    plt.figure(figsize=(8.5, 5.5))
    plt.plot(
        [s["rel_x"] for s in base_rel],
        [s["rel_y"] for s in base_rel],
        label=f"Vicon {base_topic}",
        linewidth=2.2,
    )
    plt.plot(
        [s["rel_x"] for s in imu_rel],
        [s["rel_y"] for s in imu_rel],
        label=f"IMU dead reckoning {imu_topic}",
        linewidth=1.4,
        alpha=0.8,
    )
    plt.scatter([base_rel[0]["rel_x"]], [base_rel[0]["rel_y"]], c="green", s=36, label="start")
    plt.scatter([base_rel[-1]["rel_x"]], [base_rel[-1]["rel_y"]], c="red", s=36, label="vicon end")
    plt.scatter([imu_rel[-1]["rel_x"]], [imu_rel[-1]["rel_y"]], c="purple", s=36, label="imu end")
    plt.xlabel("start-aligned x (m)")
    plt.ylabel("start-aligned y (m)")
    plt.title("HAMR IMU-only dead-reckoned odometry")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_path, dpi=160)
    plt.close()


def moving_average(values, window):
    if window <= 1 or len(values) < window:
        return values
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(values, kernel, mode="same")


def vicon_forward_speed(samples):
    """Ground-truth body-frame forward speed from differentiated Vicon pose.

    Returns (t, forward_speed, speed_magnitude). The world-frame velocity is
    obtained by finite-differencing position, then projected onto the Vicon
    heading so it is directly comparable to the odom twist.linear.x signals
    (signed: negative when reversing). Speed magnitude is kept for reference.
    """
    if len(samples) < 2:
        return np.asarray([]), np.asarray([]), np.asarray([])
    t = np.asarray([s["t"] for s in samples], dtype=float)
    x = np.asarray([s["x"] for s in samples], dtype=float)
    y = np.asarray([s["y"] for s in samples], dtype=float)
    yaw = np.asarray([s["yaw"] for s in samples], dtype=float)

    # Drop near-duplicate timestamps: Vicon arrives in bursts where consecutive
    # bag times can be ~0 apart, which would make d/dt blow up to absurd speeds.
    min_dt = 5e-3
    keep = [0]
    for i in range(1, len(t)):
        if t[i] - t[keep[-1]] >= min_dt:
            keep.append(i)
    keep = np.asarray(keep, dtype=int)
    t, x, y, yaw = t[keep], x[keep], y[keep], yaw[keep]
    if t.size < 2:
        return np.asarray([]), np.asarray([]), np.asarray([])

    vx = np.gradient(x, t)
    vy = np.gradient(y, t)
    forward = vx * np.cos(yaw) + vy * np.sin(yaw)
    return t, forward, np.hypot(vx, vy)


def write_speed_plot(
    plot_path,
    base_samples,
    onboard_samples,
    wheel_odom_samples,
    base_topic,
    onboard_topic,
    wheel_odom_topic,
    smooth_window,
):
    """Plot forward speed: Vicon ground truth vs onboard estimates.

    Vicon speed is differentiated from pose (raw + smoothed). EKF and wheel
    odometry speeds are taken straight from their twist.linear.x fields.
    """
    import matplotlib.pyplot as plt

    if not base_samples:
        raise RuntimeError("No Vicon base samples found for speed plot.")

    t0 = base_samples[0]["t"]
    fig, ax = plt.subplots(figsize=(10, 5))

    # Build a y-limit from the trustworthy onboard twist signals so a Vicon
    # marker dropout/teleport (a brief, huge d/dt spike) does not dominate.
    speed_ref = []
    for s in (onboard_samples, wheel_odom_samples):
        if s and "vx" in s[0]:
            speed_ref.extend(abs(item["vx"]) for item in s)

    tv, fwd_v, _ = vicon_forward_speed(base_samples)
    if tv.size:
        ax.plot(
            tv - t0,
            fwd_v,
            color="tab:blue",
            alpha=0.35,
            linewidth=1.0,
            label="Vicon forward speed (d/dt pose, raw)",
        )
        if smooth_window and smooth_window > 1:
            ax.plot(
                tv - t0,
                moving_average(fwd_v, smooth_window),
                color="tab:blue",
                linewidth=2.4,
                label=f"Vicon forward speed (smoothed, w={smooth_window})",
            )

    if onboard_samples and "vx" in onboard_samples[0]:
        t = np.asarray([s["t"] for s in onboard_samples], dtype=float)
        vx = np.asarray([s["vx"] for s in onboard_samples], dtype=float)
        ax.plot(
            t - t0,
            vx,
            color="tab:green",
            linewidth=1.6,
            label=f"EKF speed {onboard_topic} (twist.x)",
        )

    if wheel_odom_samples and "vx" in wheel_odom_samples[0]:
        t = np.asarray([s["t"] for s in wheel_odom_samples], dtype=float)
        vx = np.asarray([s["vx"] for s in wheel_odom_samples], dtype=float)
        ax.plot(
            t - t0,
            vx,
            color="tab:orange",
            alpha=0.8,
            linewidth=1.2,
            label=f"Wheel odom speed {wheel_odom_topic} (twist.x)",
        )

    if speed_ref:
        limit = 1.3 * max(max(speed_ref), 1e-3)
        ax.set_ylim(-limit, limit)

    ax.set_xlabel("time from start (s)")
    ax.set_ylabel("forward speed (m/s)")
    ax.set_title("HAMR forward speed: Vicon ground truth vs onboard estimates")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)


def write_localization_error_plot(plot_path, matches, estimate_label):
    import matplotlib.pyplot as plt

    if not matches:
        raise RuntimeError("No matched onboard/Vicon samples found for error plot.")

    t0 = matches[0]["t"]
    t = np.asarray([m["t"] - t0 for m in matches], dtype=float)
    err_x = np.asarray([m["err_x"] for m in matches], dtype=float)
    err_y = np.asarray([m["err_y"] for m in matches], dtype=float)
    err_xy = np.asarray([m["err_xy"] for m in matches], dtype=float)
    err_yaw_deg = np.asarray([math.degrees(m["err_yaw"]) for m in matches], dtype=float)

    fig, (ax_xy, ax_yaw) = plt.subplots(2, 1, figsize=(9, 6.2), sharex=True)
    ax_xy.plot(t, err_x, label="x error", linewidth=1.4)
    ax_xy.plot(t, err_y, label="y error", linewidth=1.4)
    ax_xy.plot(t, err_xy, label="xy error", linewidth=1.8)
    ax_xy.set_ylabel("error (m)")
    ax_xy.set_title(f"{estimate_label} minus Vicon")
    ax_xy.grid(True)
    ax_xy.legend()

    ax_yaw.plot(t, err_yaw_deg, label="yaw error", linewidth=1.5)
    ax_yaw.set_xlabel("time from first match (s)")
    ax_yaw.set_ylabel("yaw error (deg)")
    ax_yaw.grid(True)
    ax_yaw.legend()

    fig.tight_layout()
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)

def format_rad_deg(value):
    return f"{value:.4f} rad ({math.degrees(value):.2f} deg)"


def print_summary(bag_dir, metrics):
    base = metrics["base"]
    print(f"\nHAMR Vicon straight-run summary: {bag_dir}")
    print(f"  samples:        {base['sample_count']}")
    print(f"  duration:       {base['duration_s']:.3f} s")
    print(f"  path length:    {base['path_length_m']:.3f} m")
    print(f"  final forward:  {base['final_forward_m']:.3f} m")
    print(f"  final lateral:  {base['final_lateral_m']:.3f} m")
    print(f"  forward error:  {base['forward_error_m']:+.3f} m vs target {base['target_distance_m']:.3f} m")
    print(f"  lateral RMSE:   {base['lateral_rmse_m']:.3f} m")
    print(f"  lateral max:    {base['lateral_abs_max_m']:.3f} m")
    print(f"  yaw final drift:{format_rad_deg(base['yaw_drift_final_rad'])}")
    print(f"  yaw RMSE drift: {format_rad_deg(base['yaw_drift_rmse_rad'])}")

    onboard = metrics.get("onboard_tracking")
    if onboard:
        print("\nOnboard localization vs Vicon:")
        print(f"  topic:          {metrics['onboard_topic']}")
        print(f"  matched samples:{onboard['matched_sample_count']}")
        print(f"  xy RMSE:        {onboard['xy_rmse_m']:.3f} m")
        print(f"  xy max:         {onboard['xy_abs_max_m']:.3f} m")
        print(f"  final xy error: {onboard['final_err_xy_m']:.3f} m")
        print(f"  yaw RMSE:       {format_rad_deg(onboard['yaw_rmse_rad'])}")
        print(f"  final yaw error:{format_rad_deg(onboard['final_err_yaw_rad'])}")
    elif metrics.get("onboard_sample_count", 0) == 0:
        print(f"\nOnboard localization vs Vicon: no samples on {metrics['onboard_topic']}")

    wheel = metrics.get("wheel_odom_tracking")
    if wheel:
        print("\nRaw wheel odometry vs Vicon:")
        print(f"  topic:          {metrics['wheel_odom_topic']}")
        print(f"  matched samples:{wheel['matched_sample_count']}")
        print(f"  xy RMSE:        {wheel['xy_rmse_m']:.3f} m")
        print(f"  final xy error: {wheel['final_err_xy_m']:.3f} m")
        print(f"  yaw RMSE:       {format_rad_deg(wheel['yaw_rmse_rad'])}")

    imu_odom = metrics.get("imu_odom_tracking")
    if imu_odom:
        print("\nIMU-only dead reckoning vs Vicon:")
        print(f"  topic:          {metrics['imu_topic']}")
        print(f"  matched samples:{imu_odom['matched_sample_count']}")
        print(f"  xy RMSE:        {imu_odom['xy_rmse_m']:.3f} m")
        print(f"  final xy error: {imu_odom['final_err_xy_m']:.3f} m")
        print(f"  yaw RMSE:       {format_rad_deg(imu_odom['yaw_rmse_rad'])}")
    elif metrics.get("imu_sample_count", 0) == 0:
        print(f"\nIMU-only dead reckoning vs Vicon: no samples on {metrics['imu_topic']}")

    ref = metrics.get("reference_tracking")
    if ref:
        print("\nReference tracking from bag timestamps:")
        print(f"  matched samples:{ref['matched_sample_count']}")
        print(f"  xy RMSE:        {ref['xy_rmse_m']:.3f} m")
        print(f"  final xy error: {ref['final_err_xy_m']:.3f} m")
        print(f"  yaw RMSE:       {format_rad_deg(ref['yaw_rmse_rad'])}")

def parse_args():
    parser = argparse.ArgumentParser(
        description="Analyze HAMR Vicon and onboard localization trajectories from a rosbag."
    )
    parser.add_argument("bag_dir", type=Path, help="Path to rosbag directory")
    parser.add_argument("--base-topic", default=DEFAULT_BASE_TOPIC)
    parser.add_argument("--onboard-topic", default=DEFAULT_ONBOARD_TOPIC)
    parser.add_argument(
        "--source-bag-dir",
        type=Path,
        help=(
            "Optional source rosbag from which to load wheel odometry, IMU, "
            "turret, reference, wheel-command, and calibration topics. This "
            "lets a compact offline-EKF result bag be analyzed against all "
            "topics from the replayed hardware bag."
        ),
    )
    parser.add_argument("--wheel-odom-topic", default=DEFAULT_WHEEL_ODOM_TOPIC)
    parser.add_argument(
        "--wheel-bag-dir",
        type=Path,
        help=(
            "Optional rosbag from which to load the wheel-odometry topic. "
            "Useful when an offline EKF result bag contains Vicon and the EKF "
            "output but the replay source bag contains /wheel_odom."
        ),
    )
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument("--turret-topic", default=DEFAULT_TURRET_TOPIC)
    parser.add_argument("--reference-topic", default=DEFAULT_REFERENCE_TOPIC)
    parser.add_argument("--left-wheel-topic", default=DEFAULT_LEFT_WHEEL_TOPIC)
    parser.add_argument("--right-wheel-topic", default=DEFAULT_RIGHT_WHEEL_TOPIC)
    parser.add_argument("--calib-status-topic", default=DEFAULT_CALIB_STATUS_TOPIC)
    parser.add_argument("--storage-id", default="auto", help="auto, mcap, or sqlite3")
    parser.add_argument("--target-distance", type=float, default=2.0)
    parser.add_argument(
        "--no-align-yaw",
        action="store_true",
        help="Do not rotate paths into each trajectory's initial yaw frame.",
    )
    parser.add_argument(
        "--reference-max-dt",
        type=float,
        default=0.05,
        help="Max timestamp difference for base/reference matching.",
    )
    parser.add_argument(
        "--localization-max-dt",
        type=float,
        default=0.05,
        help="Max timestamp difference for Vicon/onboard matching.",
    )
    parser.add_argument(
        "--imu-bias-window-s",
        type=float,
        default=1.0,
        help="Initial seconds used to estimate IMU x/y acceleration bias.",
    )
    parser.add_argument(
        "--imu-integration-max-dt",
        type=float,
        default=0.1,
        help="Skip IMU integration steps larger than this duration.",
    )
    parser.add_argument("--json", type=Path, help="Optional metrics JSON output")
    parser.add_argument("--csv", type=Path, help="Optional Vicon base trace CSV output")
    parser.add_argument("--localization-csv", type=Path, help="Optional matched Vicon/onboard CSV output")
    parser.add_argument("--imu-odom-csv", type=Path, help="Optional derived IMU-only odometry CSV output")
    parser.add_argument("--plot", type=Path, help="Optional path/reference plot PNG output")
    parser.add_argument("--localization-plot", type=Path, help="Optional Vicon/onboard path comparison PNG output")
    parser.add_argument("--imu-odom-plot", type=Path, help="Optional full-scale IMU-only odometry plot PNG output")
    parser.add_argument("--localization-error-plot", type=Path, help="Optional Vicon/onboard error plot PNG output")
    parser.add_argument("--wheel-plot", type=Path, help="Optional Vicon actual velocity vs wheel-command velocity plot PNG output")
    parser.add_argument(
        "--wheel-radius-m",
        type=float,
        default=DEFAULT_WHEEL_RADIUS_M,
        help="Wheel radius used to convert wheel cmd_vel rad/s to linear m/s.",
    )
    parser.add_argument(
        "--speed-plot",
        type=Path,
        help="Optional forward-speed plot PNG: Vicon ground truth vs onboard estimates",
    )
    parser.add_argument(
        "--speed-smooth-window",
        type=int,
        default=5,
        help="Moving-average window (samples) for the differentiated Vicon speed.",
    )
    parser.add_argument(
        "--turret-yaw-plot",
        type=Path,
        help="Optional actual/reference trajectory plot with turret yaw arrows",
    )
    parser.add_argument(
        "--turret-max-dt",
        type=float,
        default=0.05,
        help="Max timestamp difference for base/turret yaw matching.",
    )
    parser.add_argument("--calib-plot", type=Path, help="Optional BNO055 calibration status (check_sys) plot PNG")
    parser.add_argument("--heading-comparison-plot", type=Path, help="Optional heading source comparison PNG")
    parser.add_argument(
        "--orientation-arrow-count",
        type=int,
        default=40,
        help="Maximum number of turret yaw arrows to draw.",
    )
    parser.add_argument(
        "--orientation-arrow-length",
        type=float,
        default=0.18,
        help="Turret yaw arrow length in plot units.",
    )
    return parser.parse_args()

def main():
    args = parse_args()
    bag_dir = args.bag_dir.expanduser().resolve()
    if not bag_dir.exists():
        raise FileNotFoundError(f"Bag directory does not exist: {bag_dir}")

    storage_id = detect_storage_id(bag_dir) if args.storage_id == "auto" else args.storage_id
    (
        topic_types,
        base_samples,
        onboard_samples,
        wheel_odom_samples,
        imu_samples,
        turret_samples,
        reference_samples,
        left_wheel_samples,
        right_wheel_samples,
        calib_samples,
    ) = read_bag(
        bag_dir,
        storage_id,
        args.base_topic,
        args.onboard_topic,
        args.wheel_odom_topic,
        args.imu_topic,
        args.turret_topic,
        args.reference_topic,
        args.left_wheel_topic,
        args.right_wheel_topic,
        args.calib_status_topic,
    )

    source_bag_dir = bag_dir
    if args.source_bag_dir:
        source_bag_dir = args.source_bag_dir.expanduser().resolve()
        if not source_bag_dir.exists():
            raise FileNotFoundError(
                f"Source bag directory does not exist: {source_bag_dir}"
            )
        source_storage_id = detect_storage_id(source_bag_dir)
        source = read_bag(
            source_bag_dir,
            source_storage_id,
            "/__unused/base",
            "/__unused/onboard",
            args.wheel_odom_topic,
            args.imu_topic,
            args.turret_topic,
            args.reference_topic,
            args.left_wheel_topic,
            args.right_wheel_topic,
            args.calib_status_topic,
        )
        wheel_odom_samples = source[3]
        imu_samples = source[4]
        turret_samples = source[5]
        reference_samples = source[6]
        left_wheel_samples = source[7]
        right_wheel_samples = source[8]
        calib_samples = source[9]

    wheel_bag_dir = source_bag_dir
    if args.wheel_bag_dir:
        wheel_bag_dir = args.wheel_bag_dir.expanduser().resolve()
        if not wheel_bag_dir.exists():
            raise FileNotFoundError(
                f"Wheel-odometry bag directory does not exist: {wheel_bag_dir}"
            )
        wheel_storage_id = detect_storage_id(wheel_bag_dir)
        supplemental = read_bag(
            wheel_bag_dir,
            wheel_storage_id,
            "/__unused/base",
            "/__unused/onboard",
            args.wheel_odom_topic,
            "/__unused/imu",
            "/__unused/turret",
            "/__unused/reference",
            "/__unused/left_wheel",
            "/__unused/right_wheel",
            "/__unused/calib",
        )
        wheel_odom_samples = supplemental[3]
        if not wheel_odom_samples:
            available = "\n".join(
                f"  {name} [{type_name}]"
                for name, type_name in sorted(supplemental[0].items())
            )
            raise RuntimeError(
                f"No samples found on {args.wheel_odom_topic} in "
                f"{wheel_bag_dir}. Available topics:\n{available}"
            )

    gyro_integrated_samples = integrate_gyro_z(imu_samples)

    if not base_samples:
        available = "\n".join(f"  {name} [{type_name}]" for name, type_name in sorted(topic_types.items()))
        raise RuntimeError(
            f"No samples found on {args.base_topic}. Available topics:\n{available}"
        )

    imu_odom_samples = integrate_imu_odom_samples(
        imu_samples,
        bias_window_s=args.imu_bias_window_s,
        max_dt=args.imu_integration_max_dt,
    )

    compare_requested = bool(
        args.localization_plot or args.localization_error_plot or args.localization_csv
    )
    if compare_requested and not onboard_samples:
        available = "\n".join(f"  {name} [{type_name}]" for name, type_name in sorted(topic_types.items()))
        raise RuntimeError(
            f"No onboard odometry samples found on {args.onboard_topic}. Available topics:\n{available}"
        )

    align_yaw = not args.no_align_yaw
    onboard_matches = matched_relative_samples(
        base_samples,
        onboard_samples,
        max_dt=args.localization_max_dt,
        align_yaw=align_yaw,
    )
    wheel_odom_matches = matched_relative_samples(
        base_samples,
        wheel_odom_samples,
        max_dt=args.localization_max_dt,
        align_yaw=align_yaw,
    )
    imu_odom_matches = matched_relative_samples(
        base_samples,
        imu_odom_samples,
        max_dt=args.localization_max_dt,
        align_yaw=align_yaw,
    )

    metrics = {
        "bag_dir": str(bag_dir),
        "storage_id": storage_id,
        "source_bag_dir": str(source_bag_dir),
        "base_topic": args.base_topic,
        "onboard_topic": args.onboard_topic,
        "wheel_odom_topic": args.wheel_odom_topic,
        "wheel_odom_bag_dir": str(wheel_bag_dir),
        "imu_topic": args.imu_topic,
        "turret_topic": args.turret_topic,
        "reference_topic": args.reference_topic,
        "left_wheel_topic": args.left_wheel_topic,
        "right_wheel_topic": args.right_wheel_topic,
        "wheel_radius_m": args.wheel_radius_m,
        "base": summarize_base(base_samples, args.target_distance, align_yaw),
        "onboard_sample_count": len(onboard_samples),
        "wheel_odom_sample_count": len(wheel_odom_samples),
        "imu_sample_count": len(imu_samples),
        "imu_odom_sample_count": len(imu_odom_samples),
        "gyro_integrated_sample_count": len(gyro_integrated_samples),
        "calib_sample_count": len(calib_samples),
        "imu_bias_window_s": args.imu_bias_window_s,
        "imu_integration_max_dt_s": args.imu_integration_max_dt,
        "turret_sample_count": len(turret_samples),
        "reference_sample_count": len(reference_samples),
        "left_wheel_sample_count": len(left_wheel_samples),
        "right_wheel_sample_count": len(right_wheel_samples),
    }

    onboard_metrics = summarize_trajectory_errors(onboard_matches)
    if onboard_metrics:
        metrics["onboard_tracking"] = onboard_metrics
    wheel_odom_metrics = summarize_trajectory_errors(wheel_odom_matches)
    if wheel_odom_metrics:
        metrics["wheel_odom_tracking"] = wheel_odom_metrics
    imu_odom_metrics = summarize_trajectory_errors(imu_odom_matches)
    if imu_odom_metrics:
        metrics["imu_odom_tracking"] = imu_odom_metrics

    ref_metrics = nearest_reference_errors(
        base_samples, reference_samples, max_dt=args.reference_max_dt
    )
    if ref_metrics:
        metrics["reference_tracking"] = ref_metrics

    arr, rel_x, rel_y, rel_yaw = start_relative_path(base_samples, align_yaw=align_yaw)
    del arr

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        write_csv(args.csv, base_samples, rel_x, rel_y, rel_yaw)
    if args.localization_csv:
        args.localization_csv.parent.mkdir(parents=True, exist_ok=True)
        write_localization_csv(args.localization_csv, onboard_matches)
    if args.imu_odom_csv and imu_odom_samples:
        args.imu_odom_csv.parent.mkdir(parents=True, exist_ok=True)
        write_pose_trace_csv(args.imu_odom_csv, imu_odom_samples, align_yaw=align_yaw)
    elif args.imu_odom_csv:
        print(f"Skipping IMU odom CSV: no samples found on {args.imu_topic}.")
    if args.plot:
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        write_plot(args.plot, base_samples, rel_x, rel_y, args.target_distance, reference_samples)
    if args.localization_plot:
        args.localization_plot.parent.mkdir(parents=True, exist_ok=True)
        write_localization_plot(
            args.localization_plot,
            base_samples,
            onboard_samples,
            wheel_odom_samples,
            imu_odom_samples,
            reference_samples,
            align_yaw,
            args.base_topic,
            args.onboard_topic,
            args.wheel_odom_topic,
            args.imu_topic,
        )
    if args.imu_odom_plot and imu_odom_samples:
        args.imu_odom_plot.parent.mkdir(parents=True, exist_ok=True)
        write_imu_odom_plot(
            args.imu_odom_plot,
            base_samples,
            imu_odom_samples,
            align_yaw,
            args.base_topic,
            args.imu_topic,
        )
    elif args.imu_odom_plot:
        print(f"Skipping IMU odom plot: no samples found on {args.imu_topic}.")
    if args.localization_error_plot:
        args.localization_error_plot.parent.mkdir(parents=True, exist_ok=True)
        write_localization_error_plot(
            args.localization_error_plot,
            onboard_matches,
            args.onboard_topic,
        )
    if args.wheel_plot and (left_wheel_samples or right_wheel_samples):
        args.wheel_plot.parent.mkdir(parents=True, exist_ok=True)
        write_wheel_plot(
            args.wheel_plot,
            left_wheel_samples,
            right_wheel_samples,
            base_samples,
            args.left_wheel_topic,
            args.right_wheel_topic,
            args.wheel_radius_m,
            args.speed_smooth_window,
        )
    elif args.wheel_plot:
        print(
            f"Skipping wheel command plot: no samples found on "
            f"{args.left_wheel_topic} or {args.right_wheel_topic}."
        )
    if args.speed_plot:
        args.speed_plot.parent.mkdir(parents=True, exist_ok=True)
        write_speed_plot(
            args.speed_plot,
            base_samples,
            onboard_samples,
            wheel_odom_samples,
            args.base_topic,
            args.onboard_topic,
            args.wheel_odom_topic,
            args.speed_smooth_window,
        )
    if args.turret_yaw_plot:
        args.turret_yaw_plot.parent.mkdir(parents=True, exist_ok=True)
        write_turret_yaw_plot(
            args.turret_yaw_plot,
            base_samples,
            turret_samples,
            reference_samples,
            args.turret_max_dt,
            args.orientation_arrow_count,
            args.orientation_arrow_length,
        )
    if args.calib_plot:
        args.calib_plot.parent.mkdir(parents=True, exist_ok=True)
        if calib_samples:
            write_calib_status_plot(args.calib_plot, calib_samples)
        else:
            print(f"Skipping calib plot: no samples found on {args.calib_status_topic}.")
    if args.heading_comparison_plot:
        args.heading_comparison_plot.parent.mkdir(parents=True, exist_ok=True)
        write_heading_comparison_plot(
            args.heading_comparison_plot,
            base_samples,
            imu_samples,
            wheel_odom_samples,
            gyro_integrated_samples,
        )
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(metrics, indent=2), encoding="utf-8")

    print_summary(bag_dir, metrics)


if __name__ == "__main__":
    main()
