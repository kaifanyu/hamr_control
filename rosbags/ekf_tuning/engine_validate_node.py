#!/usr/bin/env python3
"""Compare fast core replay against the actual robot_localization ROS node.

Run under /usr/bin/python3 after sourcing /opt/ros/jazzy/setup.bash. The local
ROS packages are obtained with engine_prepare_ros.sh. A private ROS domain
isolates this validation from hardware. No Vicon input is published.
"""

import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile
import time

import numpy as np
import yaml

from engine import Engine, DEFAULT_Q, DEFAULT_P, DEFAULT_MASK


def load_events(data_path):
    data = np.load(data_path)
    wheel, imu = data["wheel"], data["imu"]
    wheel_events = np.column_stack((wheel[:, 0], np.zeros(len(wheel)),
        wheel[:, 6:8], wheel[:, 5], wheel[:, 8], np.zeros((len(wheel), 2))))
    imu_events = np.column_stack((imu[:, 0], np.ones(len(imu)),
        np.zeros((len(imu), 2)), imu[:, 2:6]))
    events = np.vstack((wheel_events, imu_events))
    return events[np.argsort(events[:, 0], kind="stable")]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data", type=Path, help="extracted source bag NPZ")
    parser.add_argument("--events", type=Path, help="optional corrected N x 8 NPY")
    parser.add_argument("--parameters", type=Path, help="JSON: q_diag,p_diag,r_diag,mask,relative_imu")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rate", type=float, default=4.0)
    parser.add_argument("--domain", type=int, default=87)
    parser.add_argument("--debug-counts", action="store_true",
                        help="parse actual EKF debug log to verify every input is fused in order")
    parser.add_argument("--enqueue-delay", type=float, default=.002,
                        help="wall seconds for input callbacks before each simulated-clock tick")
    args = parser.parse_args()
    settings = json.loads(args.parameters.read_text()) if args.parameters else {}
    q = np.array(settings.get("q_diag", DEFAULT_Q), dtype=float)
    p = np.array(settings.get("p_diag", DEFAULT_P), dtype=float)
    mask = np.array(settings.get("mask", DEFAULT_MASK), dtype=np.int32)
    r = np.array(settings.get("r_diag", [.01, .01, .01, .01, .0009, .0002, .01, .01]))
    relative_imu = bool(settings.get("relative_imu", True))
    events = np.load(args.events) if args.events else load_events(args.data)
    reference = Engine().run(events, q_diag=q, p_diag=p, r_diag=r, mask=mask,
                              relative_imu=relative_imu)
    root = Path(__file__).parent
    prefix = root / "engine_ros/root/opt/ros/jazzy"
    binary = prefix / "lib/robot_localization/ekf_node"
    if not binary.exists():
        raise SystemExit("Run bash engine_prepare_ros.sh first")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["AMENT_PREFIX_PATH"] = str(prefix) + ":" + os.environ.get("AMENT_PREFIX_PATH", "/opt/ros/jazzy")
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = ":".join((str(prefix / "lib"),
        str(root / "engine_ros/root/usr/lib/x86_64-linux-gnu"),
        "/opt/ros/jazzy/lib", env.get("LD_LIBRARY_PATH", "")))
    odom_config = [False] * 15
    imu_config = [False] * 15
    for i, state in enumerate((6, 7, 5, 11)):
        odom_config[state] = bool(mask[i])
    for i, state in enumerate((5, 11, 12, 13), start=4):
        imu_config[state] = bool(mask[i])
    config = {"/**": {"ros__parameters": {
        "use_sim_time": True, "frequency": 50., "sensor_timeout": .02,
        "two_d_mode": True, "publish_tf": False,
        "odom_frame": "odom", "base_link_frame": "base_link", "world_frame": "odom",
        "odom0": "/engine/wheel", "imu0": "/engine/imu",
        "odom0_config": odom_config, "imu0_config": imu_config,
        "odom0_queue_size": 200, "imu0_queue_size": 200,
        "imu0_relative": relative_imu, "imu0_differential": False,
        "imu0_remove_gravitational_acceleration": False,
        "process_noise_covariance": np.diag(q).ravel().tolist(),
        "initial_estimate_covariance": np.diag(p).ravel().tolist(),
    }}}
    debug_path = args.output.with_suffix(".debug.log").resolve()
    if args.debug_counts:
        config["/**"]["ros__parameters"].update(debug=True, debug_out_file=str(debug_path))
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from builtin_interfaces.msg import Time

    def stamp(seconds):
        ns = round((seconds + 10.) * 1e9)
        return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    rclpy.init()
    node = rclpy.create_node("engine_validation_driver")
    clock_pub = node.create_publisher(Clock, "/clock", 100)
    wheel_pub = node.create_publisher(Odometry, "/engine/wheel", 200)
    imu_pub = node.create_publisher(Imu, "/engine/imu", 200)
    tf_pub = node.create_publisher(TFMessage, "/tf_static",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    output = []

    def receive(message):
        pose, twist = message.pose.pose, message.twist.twist
        quat = pose.orientation
        yaw = math.atan2(2*(quat.w*quat.z + quat.x*quat.y),
                         1 - 2*(quat.y*quat.y + quat.z*quat.z))
        output.append([message.header.stamp.sec + message.header.stamp.nanosec*1e-9 - 10.,
            pose.position.x, pose.position.y, yaw, twist.linear.x, twist.linear.y, twist.angular.z])

    node.create_subscription(Odometry, "/odometry/filtered", receive, 200)
    tf = TransformStamped()
    tf.header.frame_id = "base_link"
    tf.child_frame_id = "imu_link"
    tf.transform.rotation.w = 1.
    tf_pub.publish(TFMessage(transforms=[tf]))
    messages = []
    for row in events:
        if row[1] == 0:
            msg = Odometry()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            msg.pose.pose.orientation.z = math.sin(row[4]/2)
            msg.pose.pose.orientation.w = math.cos(row[4]/2)
            msg.twist.twist.linear.x = row[2]
            msg.twist.twist.linear.y = row[3]
            msg.twist.twist.angular.z = row[5]
            msg.pose.covariance = np.diag([.01,.01,1e-9,1e-9,1e-9,r[2]]).ravel().tolist()
            msg.twist.covariance = np.diag([r[0],r[1],1e-9,1e-9,1e-9,r[3]]).ravel().tolist()
            pub = wheel_pub
        else:
            msg = Imu()
            msg.header.frame_id = "imu_link"
            msg.orientation.z = math.sin(row[4]/2)
            msg.orientation.w = math.cos(row[4]/2)
            msg.angular_velocity.z = row[5]
            msg.linear_acceleration.x, msg.linear_acceleration.y = row[6], row[7]
            msg.orientation_covariance = np.diag([.0003,.0003,r[4]]).ravel().tolist()
            msg.angular_velocity_covariance = np.diag([.0002,.0002,r[5]]).ravel().tolist()
            msg.linear_acceleration_covariance = np.diag([r[6],r[7],.01]).ravel().tolist()
            pub = imu_pub
        msg.header.stamp = stamp(row[0])
        messages.append((pub, msg))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".yaml").write_text(yaml.safe_dump(config))
    log_path = args.output.with_suffix(".log")
    with tempfile.TemporaryDirectory(prefix="hamr_engine_validation_") as work, log_path.open("w") as log:
        config_path = Path(work) / "ekf.yaml"
        config_path.write_text(yaml.safe_dump(config))
        process = subprocess.Popen([str(binary), "--ros-args", "--params-file", str(config_path)],
                                   env=env, stdout=log, stderr=log)
        try:
            deadline = time.monotonic() + 10
            while wheel_pub.get_subscription_count() < 1 or imu_pub.get_subscription_count() < 1:
                # robot_localization blocks initialization until ROS time starts.
                clock_pub.publish(Clock(clock=stamp(0)))
                if process.poll() is not None:
                    raise RuntimeError(f"EKF node exited; see {log_path}")
                if time.monotonic() > deadline:
                    raise RuntimeError("EKF input discovery timed out")
                rclpy.spin_once(node, timeout_sec=.02)
            # Allow output discovery and static TF subscription to settle.
            settle = time.monotonic() + .5
            while time.monotonic() < settle:
                clock_pub.publish(Clock(clock=stamp(0)))
                rclpy.spin_once(node, timeout_sec=.01)
            next_message = 0
            wall_start = time.monotonic()
            # Simulated 50 Hz timer ticks; publication is paced to avoid DDS losses.
            for current in np.arange(0., events[-1, 0] + .04, .02):
                while next_message < len(events) and events[next_message, 0] <= current:
                    pub, message = messages[next_message]
                    pub.publish(message)
                    next_message += 1
                # Give the node a chance to enqueue messages before advancing its timer.
                enqueue_deadline = time.monotonic() + args.enqueue_delay
                while time.monotonic() < enqueue_deadline:
                    rclpy.spin_once(node, timeout_sec=max(0., min(.001, enqueue_deadline-time.monotonic())))
                clock_pub.publish(Clock(clock=stamp(float(current))))
                target = wall_start + (current + .02) / args.rate
                while time.monotonic() < target:
                    rclpy.spin_once(node, timeout_sec=max(0., min(.001, target-time.monotonic())))
            settle = time.monotonic() + .2
            while time.monotonic() < settle:
                rclpy.spin_once(node, timeout_sec=.01)
        finally:
            process.terminate()
            process.wait(timeout=5)
            node.destroy_node()
            rclpy.try_shutdown()
    actual = np.asarray(output)
    if not len(actual):
        raise RuntimeError(f"No filter output received; see {log_path}")
    indices = np.searchsorted(reference[:,0], actual[:,0] + 1e-8, side="right") - 1
    valid = (indices >= 0) & (actual[:,0] <= reference[-1,0]+1e-8)
    actual = actual[valid]
    expected = reference[indices[valid]]
    differences = actual[:,1:] - expected[:,1:]
    differences[:,2] = np.arctan2(np.sin(differences[:,2]), np.cos(differences[:,2]))
    report = {
        "data": str(args.data), "events": len(events), "node_outputs": len(actual),
        "parameter_values": {"q_diag":q.tolist(), "p_diag":p.tolist(), "r_diag":r.tolist(), "mask":mask.tolist()},
        "max_absolute_difference_x_y_yaw_vx_vy_wz": np.max(np.abs(differences),axis=0).tolist(),
        "rms_position_difference_m": float(np.sqrt(np.mean(np.sum(differences[:,:2]**2,axis=1)))),
        "rms_yaw_difference_rad": float(np.sqrt(np.mean(differences[:,2]**2))),
        "published_sensor_counts": {"wheel": int(np.sum(events[:,1] == 0)), "imu": int(np.sum(events[:,1] == 1))},
        "node_output_rate_hz": float((len(actual)-1) / (actual[-1,0]-actual[0,0])),
        "replay_rate": args.rate,
        "ros_domain_id": args.domain,
        "effective_config": str(args.output.with_suffix(".yaml")),
        "method": "Actual ROS node, planar preprocessed measurement messages, static identity IMU TF, 50 Hz simulated clock. Compare output measurement stamp to fast engine state.",
    }
    if args.debug_counts:
        counts = Counter()
        negative_deltas = 0
        out_of_order_measurements = 0
        previous_measurement_time = None
        negative_clock_prediction_deltas = 0
        maximum_queue_size = 0
        last_delta = None
        minimum_delta = None
        with debug_path.open() as debug:
            for line in debug:
                match = re.match(r"^(\d+) measurements in queue\.", line)
                if match:
                    maximum_queue_size = max(maximum_queue_size, int(match.group(1)))
                match = re.search(r"^------ FilterBase::processMeasurement \(([^)]+)\) ------", line)
                if match:
                    counts[match.group(1)] += 1
                match = re.search(r"Measurement time is (\d+), last measurement time is (\d+), delta is (-?\d+)", line)
                if match:
                    measurement_time = int(match.group(1))
                    if previous_measurement_time is not None:
                        out_of_order_measurements += measurement_time < previous_measurement_time
                    previous_measurement_time = measurement_time
                    last_delta = int(match.group(3))
                    minimum_delta = last_delta if minimum_delta is None else min(minimum_delta, last_delta)
                    negative_deltas += last_delta < 0
                    negative_clock_prediction_deltas += last_delta < 0 and int(match.group(2)) % 20_000_000 == 0
        expected_counts = {}
        wheel_count, imu_count = int(np.sum(events[:,1] == 0)), int(np.sum(events[:,1] == 1))
        if mask[0] or mask[1] or mask[3]: expected_counts["odom0_twist"] = wheel_count
        if mask[2]: expected_counts["odom0_pose"] = wheel_count
        if mask[4]: expected_counts["imu0_pose"] = imu_count
        if mask[5]: expected_counts["imu0_twist"] = imu_count
        if mask[6] or mask[7]: expected_counts["imu0_acceleration"] = imu_count
        report["input_acceptance"] = {
            "expected_filter_measurements": expected_counts,
            "actual_filter_measurements": dict(counts),
            "all_measurements_processed": dict(counts) == expected_counts,
            "negative_time_deltas": negative_deltas,
            "out_of_order_sensor_measurements": out_of_order_measurements,
            "negative_deltas_after_exact_clock_tick_prediction": negative_clock_prediction_deltas,
            "maximum_combined_measurement_queue_size": maximum_queue_size,
            "minimum_time_delta_ns": minimum_delta,
            "negative_delta_note": "A measurement can be older than an earlier timeout-only filter prediction while sensor measurement timestamps remain ordered; processMeasurement still corrects with it.",
            "debug_log": str(debug_path),
        }
    from evaluate import Recording
    recording = Recording(args.data)
    report["core_vicon_score"] = recording.score(reference)
    report["node_vicon_score"] = recording.score(actual)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    np.savez_compressed(args.output.with_suffix(".npz"), actual=actual, expected=expected)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
