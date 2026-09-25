#!/usr/bin/env python3
"""Exercise an already-running caster rig and save observed physics evidence.

This is a finite-motion smoke check, not a friction, compliance, force, or
hardware-accuracy calibration. Run once at each requested physics step size.
"""

import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import time


RIG_JOINTS = {"rig_x_joint", "rig_y_joint", "rig_yaw_joint", "rig_z_joint"}
PASSIVE_JOINTS = {
    "caster_carrier_joint", "caster_positive_hemisphere_joint",
    "caster_negative_hemisphere_joint", "caster_positive_polar_roller_joint",
    "caster_negative_polar_roller_joint",
}


def json_safe(value):
    """Retain non-finite failure evidence in valid JSON instead of losing it."""
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def summarize(samples, phases, max_z_span, max_passive_speed):
    """Evaluate recorded data without ROS, so saved reports can be inspected."""
    failures = []
    names = set().union(*(set(sample["positions"]) for sample in samples)) if samples else set()
    missing = sorted(RIG_JOINTS - names)
    if missing:
        failures.append(f"Missing rig joints: {missing}. Launch model:=rig.")
    passive = sorted(PASSIVE_JOINTS & names)
    missing_passive = sorted(PASSIVE_JOINTS - names)
    if missing_passive:
        failures.append(f"Missing passive caster joints: {missing_passive}.")
    finite = all(math.isfinite(value) for sample in samples
                 for field in ("positions", "velocities") for value in sample[field].values())
    if not finite:
        failures.append("Non-finite joint state received.")
    metrics = {"sample_count": len(samples), "all_finite": finite, "passive_joints": passive,
               "ignored_fixed_or_other_joints": sorted(names - RIG_JOINTS - PASSIVE_JOINTS),
               "phases": {}, "passive_position_span_rad": {}}
    for name in passive:
        values = [sample["positions"][name] for sample in samples if name in sample["positions"]]
        metrics["passive_position_span_rad"][name] = max(values) - min(values) if values else 0.0
    speeds = [abs(sample["velocities"].get(name, 0.0)) for sample in samples for name in passive]
    metrics["max_passive_speed_rad_s"] = max(speeds, default=0.0)
    if metrics["max_passive_speed_rad_s"] > max_passive_speed:
        failures.append(f"Passive speed exceeded {max_passive_speed} rad/s.")

    for phase in phases:
        rows = [sample for sample in samples if sample["phase"] == phase["name"]]
        if len(rows) < 5:
            failures.append(f"Too few samples in phase {phase['name']}.")
            continue
        spans = {}
        for name in passive:
            values = [sample["positions"][name] for sample in rows if name in sample["positions"]]
            spans[name] = max(values) - min(values) if values else 0.0
        metric = {"samples": len(rows), "observed_duration_s": rows[-1]["time"] - rows[0]["time"],
                  "passive_span_rad": spans, "passive_joints": {}}
        terminal_rows = [sample for sample in rows if sample["time"] >= rows[-1]["time"] - 1.0]
        for name in passive:
            positions = [sample["positions"][name] for sample in rows if name in sample["positions"]]
            terminal_positions = [sample["positions"][name] for sample in terminal_rows if name in sample["positions"]]
            velocities = [sample["velocities"][name] for sample in rows if name in sample["velocities"]]
            metric["passive_joints"][name] = {
                "first_position_rad": positions[0] if positions else None,
                "last_position_rad": positions[-1] if positions else None,
                "delta_position_rad": positions[-1] - positions[0] if positions else None,
                "terminal_mean_position_rad": statistics.mean(terminal_positions) if terminal_positions else None,
                "terminal_median_position_rad": statistics.median(terminal_positions) if terminal_positions else None,
                "mean_velocity_rad_s": statistics.mean(velocities) if velocities else None,
                "max_abs_velocity_rad_s": max(map(abs, velocities)) if velocities else None,
            }
        terminal_z = [sample["positions"]["rig_z_joint"] for sample in terminal_rows
                      if "rig_z_joint" in sample["positions"]]
        metric["terminal_z_mean_m"] = statistics.mean(terminal_z) if terminal_z else None
        metric["terminal_z_median_m"] = statistics.median(terminal_z) if terminal_z else None
        metric["terminal_z_span_m"] = max(terminal_z) - min(terminal_z) if terminal_z else None
        if phase["name"].startswith(("x_", "y_")):
            axis = phase["name"][0]
            joint = f"rig_{axis}_joint"
            if joint in rows[0]["positions"] and joint in rows[-1]["positions"]:
                observed = rows[-1]["positions"][joint] - rows[0]["positions"][joint]
                expected = phase["command"]["xyz".index(axis)] * metric["observed_duration_s"]
                metric.update({"observed_translation_m": observed, "expected_translation_m": expected})
                if abs(observed - expected) > 0.025:
                    failures.append(f"{phase['name']}: rig translation did not follow the test command.")
            if max(spans.values(), default=0.0) < 0.02:
                failures.append(f"{phase['name']}: no measurable passive caster rotation.")
        if phase["name"].startswith("settle"):
            tail = terminal_z
            if tail and metric["terminal_z_span_m"] > max_z_span:
                failures.append(f"{phase['name']}: vertical fixture failed to settle within {max_z_span} m.")
            # The supplied rig starts the ball center at 0.11 m; its CAD outer
            # envelope is about 0.10 m. A carriage resting at the -0.08 m joint
            # stop means the ground/caster collision is missing, even if stable.
            if tail and not all(-0.025 < value < 0.015 for value in tail):
                failures.append(f"{phase['name']}: ball center is outside the expected ground-support height.")
        metrics["phases"][phase["name"]] = metric
    z_values = [sample["positions"]["rig_z_joint"] for sample in samples
                if "rig_z_joint" in sample["positions"]]
    metrics["z_range_m"] = [min(z_values), max(z_values)] if z_values else None
    if z_values and max(abs(value) for value in z_values) > 1.0:
        failures.append("Rig vertical displacement exceeded 1 m.")
    return {
        "passed": not failures, "failures": failures, "metrics": metrics,
        "validation_notes": [
            "Checks the five named passive joints; fixed mounting/world joints do not count as caster degrees of freedom.",
            "Phase terminal statistics use the final one simulation second of recorded samples.",
            "Contact-point slip, individual contact forces, compliance, friction calibration and hardware agreement are not tested.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("ball_caster_physics_check.json"))
    parser.add_argument("--wall-timeout", type=float, default=180.0)
    parser.add_argument("--speed", type=float, default=0.03, help="Horizontal fixture speed, m/s (0, 0.1]")
    parser.add_argument("--max-z-span", type=float, default=0.01, help="Allowed terminal settle excursion, m")
    parser.add_argument("--max-passive-speed", type=float, default=100.0, help="Divergence threshold, rad/s")
    parser.add_argument("--label", default="", help="Record settings, e.g. dt=0.0005 mu=0.8")
    args = parser.parse_args()
    if not 0 < args.speed <= 0.1 or not math.isfinite(args.speed):
        parser.error("--speed must be in (0, 0.1] m/s")
    if any(not math.isfinite(value) or value <= 0 for value in
           (args.wall_timeout, args.max_z_span, args.max_passive_speed)):
        parser.error("Timeout and check thresholds must be finite and positive")

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64
    except ImportError as exc:
        parser.exit(2, f"ROS 2 Python packages unavailable: {exc}. Source ROS and the workspace first.\n")

    phases = [
        {"name": "settle_initial", "duration": 3.0, "command": (0.0, 0.0, 0.0)},
        {"name": "x_forward", "duration": 4.0, "command": (args.speed, 0.0, 0.0)},
        {"name": "x_reverse", "duration": 4.0, "command": (-args.speed, 0.0, 0.0)},
        {"name": "y_forward", "duration": 4.0, "command": (0.0, args.speed, 0.0)},
        {"name": "y_reverse", "duration": 4.0, "command": (0.0, -args.speed, 0.0)},
        {"name": "yaw_forward", "duration": 3.0, "command": (0.0, 0.0, 0.15)},
        {"name": "yaw_reverse", "duration": 3.0, "command": (0.0, 0.0, -0.15)},
        {"name": "settle_final", "duration": 3.0, "command": (0.0, 0.0, 0.0)},
    ]

    class Recorder(Node):
        def __init__(self):
            super().__init__("ball_caster_physics_check")
            self.sim_time = None
            self.joints = None
            self.joints_wall_time = None
            self.publishers_by_axis = [self.create_publisher(Float64, f"/ball_caster/rig/{axis}/cmd_vel", 10)
                                       for axis in ("x", "y", "yaw")]
            self.create_subscription(Clock, "/clock", self.clock_callback, qos_profile_sensor_data)
            self.create_subscription(JointState, "/joint_states", self.joint_callback, qos_profile_sensor_data)

        def clock_callback(self, msg):
            self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        def joint_callback(self, msg):
            self.joints = msg
            self.joints_wall_time = time.monotonic()

        def command(self, values):
            for publisher, value in zip(self.publishers_by_axis, values):
                publisher.publish(Float64(data=float(value)))

    rclpy.init(args=[])
    node = Recorder()
    samples = []
    error = None
    started_wall = time.monotonic()
    phase_index = 0
    started_sim = None
    phase_started = None
    last_sample = -math.inf
    last_clock = -math.inf
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.monotonic() - started_wall > args.wall_timeout:
                raise RuntimeError("Wall timeout: Gazebo may be paused, slow, or missing /clock or /joint_states.")
            if node.sim_time is None or node.joints is None:
                continue
            if time.monotonic() - node.joints_wall_time > 5.0:
                raise RuntimeError("Joint state stream stopped for more than five wall seconds.")
            now = node.sim_time
            if now < last_clock - 1e-9:
                raise RuntimeError("Simulation clock reset during the check; restart the check on a fresh rig.")
            last_clock = now
            if started_sim is None:
                if not RIG_JOINTS.issubset(node.joints.name):
                    raise RuntimeError("Expected rig joint names not found. Launch model:=rig before this check.")
                started_sim = phase_started = now
                node.get_logger().info("Starting 28 simulation seconds of slow fixture motion.")
            phase = phases[phase_index]
            if now - phase_started >= phase["duration"]:
                phase_index += 1
                if phase_index == len(phases):
                    break
                phase_started = now
                phase = phases[phase_index]
                node.get_logger().info(f"Phase: {phase['name']}")
            if now - last_sample < 0.05:
                continue
            last_sample = now
            node.command(phase["command"])
            positions = dict(zip(node.joints.name, node.joints.position))
            velocities = dict(zip(node.joints.name, node.joints.velocity))
            samples.append({"time": now - started_sim, "phase": phase["name"],
                            "positions": positions, "velocities": velocities})
    except (RuntimeError, KeyboardInterrupt) as exc:
        error = str(exc) or "Interrupted"
    finally:
        if rclpy.ok():
            for _ in range(6):
                node.command((0.0, 0.0, 0.0))
                rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    report = summarize(samples, phases, args.max_z_span, args.max_passive_speed)
    if error:
        report["passed"] = False
        report["failures"].insert(0, error)
    report.update({"label": args.label, "test": "passive_caster_rig_smoke",
                   "scope": "Finite motion, fixture travel, settling and passive rotation; not calibrated force or contact-slip accuracy.",
                   "wall_duration_s": time.monotonic() - started_wall,
                   "phases": phases, "samples": samples})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(json_safe(report), indent=2, allow_nan=False) + "\n")
    print(f"{'PASS' if report['passed'] else 'FAIL'}: {args.output.resolve()}")
    for failure in report["failures"]:
        print(f"  {failure}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
