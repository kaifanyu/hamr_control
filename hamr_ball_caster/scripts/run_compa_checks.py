#!/usr/bin/env python3
"""Check an already-running model:=compa with controller:=false.

Use the same ROS_DOMAIN_ID as its launch. Publishes 0.3 rad/s wheel commands
for three simulation seconds forward and reverse, then stops both wheels.
Run after sourcing ROS and the ball-caster workspace. No Gazebo GUI required.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


EXPECTED = {f"{side}_caster_{joint}_joint" for side in ("left", "right") for joint in (
    "carrier", "positive_hemisphere", "negative_hemisphere", "positive_polar_roller", "negative_polar_roller")}
PHASES = [("settle_initial", 4., 0.), ("forward", 3., .3), ("stop_mid", 2., 0.),
          ("reverse", 3., -.3), ("settle_final", 3., 0.)]


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * K + (1. - math.cos(angle)) * K @ K


def origin(element):
    T = np.eye(4)
    if element is not None:
        T[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        T[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    return T


def fk(robot, q):
    joints = list(robot.findall("joint"))
    result = {"base_footprint": np.eye(4)}
    while joints:
        progressed = False
        for joint in joints[:]:
            parent = joint.find("parent").get("link")
            if parent not in result:
                continue
            motion = np.eye(4)
            if joint.get("type") in ("continuous", "revolute"):
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                motion[:3, :3] = rotation(axis, q.get(joint.get("name"), 0.))
            result[joint.find("child").get("link")] = result[parent] @ origin(joint.find("origin")) @ motion
            joints.remove(joint)
            progressed = True
        if not progressed:
            raise RuntimeError("Could not reconstruct COMPA joint tree")
    return result


def angles(q):
    x, y, z, w = q
    return [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("compa_motion.json"))
    parser.add_argument("--wall-timeout", type=float, default=300.)
    args = parser.parse_args()
    description_path = Path(get_package_share_directory("hamr_ball_caster")) / "urdf/compa_ball_caster.urdf.xacro"
    description = subprocess.check_output(["xacro", str(description_path)])
    robot = ET.fromstring(description)
    rclpy.init(args=[])
    node = Node("compa_caster_physics_check")
    state = {"clock": None, "odom": None, "joints": None}
    def receive_clock(msg):
        state["clock"] = msg.clock.sec + msg.clock.nanosec*1e-9
    def receive_odom(msg):
        state["odom"] = msg
    def receive_joints(msg):
        state["joints"] = msg
    node.create_subscription(Clock, "/clock", receive_clock, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/compa/odom", receive_odom, qos_profile_sensor_data)
    node.create_subscription(JointState, "/joint_states", receive_joints, qos_profile_sensor_data)
    publishers = [node.create_publisher(Float64, f"/{side}_wheel/cmd_vel", 10) for side in ("left", "right")]
    def command(value):
        for publisher in publishers:
            publisher.publish(Float64(data=float(value)))
    wall_start = time.monotonic()
    simulation_start = phase_start = None
    phase_index = 0
    last_sample = -math.inf
    samples = []
    failure = None
    closure_baselines = {}
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            if time.monotonic()-wall_start > args.wall_timeout:
                raise RuntimeError("Timed out waiting for simulation data or motion completion")
            if any(value is None for value in state.values()):
                continue
            now = state["clock"]
            if simulation_start is None:
                missing = EXPECTED - set(state["joints"].name)
                if missing:
                    raise RuntimeError(f"Missing passive joints: {sorted(missing)}")
                simulation_start = phase_start = now
                print("Started COMPA settle/forward/reverse check", flush=True)
            phase = PHASES[phase_index]
            if now-phase_start >= phase[1]:
                phase_index += 1
                if phase_index == len(PHASES):
                    break
                phase_start = now
                phase = PHASES[phase_index]
                print(f"Phase {phase[0]}", flush=True)
            if now-last_sample < .05:
                continue
            last_sample = now
            command(phase[2])
            joint_state = state["joints"]
            q = dict(zip(joint_state.name, joint_state.position))
            velocity = dict(zip(joint_state.name, joint_state.velocity))
            transforms = fk(robot, q)
            closure = {}
            for side in ("left", "right"):
                relative = np.linalg.inv(transforms[f"{side}_rocker_link"]) @ transforms[f"tmp_{side}_rod_sphere2_t3"]
                if side not in closure_baselines:
                    closure_baselines[side] = relative
                delta = np.linalg.inv(closure_baselines[side]) @ relative
                closure[side] = {"translation_drift_m": float(np.linalg.norm(delta[:3, 3])),
                                 "rotation_drift_rad": math.acos(max(-1., min(1., (np.trace(delta[:3, :3])-1.)/2.)))}
            odom = state["odom"].pose.pose
            xyz = [odom.position.x, odom.position.y, odom.position.z]
            rpy = angles([odom.orientation.x, odom.orientation.y, odom.orientation.z, odom.orientation.w])
            samples.append({"time": now-simulation_start, "phase": phase[0], "odom_xyz": xyz,
                            "odom_rpy": rpy, "positions": q, "velocities": velocity, "closure": closure})
    except (RuntimeError, KeyboardInterrupt) as exc:
        failure = str(exc) or "Interrupted"
    finally:
        for _ in range(8):
            command(0.)
            rclpy.spin_once(node, timeout_sec=.02)
        node.destroy_node()
        rclpy.shutdown()

    failures = [failure] if failure else []
    metrics = {"passive_joints": sorted(EXPECTED), "sample_count": len(samples), "phases": {}}
    values = [number for sample in samples for field in ("odom_xyz", "odom_rpy") for number in sample[field]]
    values += [number for sample in samples for field in ("positions", "velocities") for number in sample[field].values()]
    if not all(math.isfinite(value) for value in values):
        failures.append("Non-finite pose or joint data")
    if samples:
        metrics["max_abs_base_roll_pitch_rad"] = max(abs(value) for sample in samples for value in sample["odom_rpy"][:2])
        metrics["base_link_height_range_m"] = [min(s["odom_xyz"][2]+.22 for s in samples), max(s["odom_xyz"][2]+.22 for s in samples)]
        if metrics["max_abs_base_roll_pitch_rad"] > .35:
            failures.append("Excessive body roll/pitch")
        if metrics["base_link_height_range_m"][0] < .10 or metrics["base_link_height_range_m"][1] > .5:
            failures.append("Body height outside expected support range")
        metrics["closed_loop_relative_drift"] = {}
        for side in ("left", "right"):
            translation = max(s["closure"][side]["translation_drift_m"] for s in samples)
            angular = max(s["closure"][side]["rotation_drift_rad"] for s in samples)
            metrics["closed_loop_relative_drift"][side] = {"translation_m": translation, "rotation_rad": angular}
            if translation > .003 or angular > .03:
                failures.append(f"{side} suspension closed-loop frame drift exceeded smoke threshold")
    for name, duration, speed in PHASES:
        rows = [sample for sample in samples if sample["phase"] == name]
        if len(rows) < 10:
            failures.append(f"Insufficient data for {name}")
            continue
        displacement = [rows[-1]["odom_xyz"][i]-rows[0]["odom_xyz"][i] for i in range(3)]
        passive_spans = {joint: max(s["positions"][joint] for s in rows)-min(s["positions"][joint] for s in rows) for joint in EXPECTED}
        phase_metric = {"displacement_m": displacement, "passive_spans_rad": passive_spans,
                        "terminal_z_median_m": statistics.median(s["odom_xyz"][2] for s in rows if s["time"] >= rows[-1]["time"]-1.)}
        if speed:
            travel = math.hypot(*displacement[:2])
            if not .04 < travel < .15:
                failures.append(f"{name} travel outside expected slow-wheel range: {travel}")
            for side in ("left", "right"):
                if max(span for joint, span in passive_spans.items() if joint.startswith(side)) < .1:
                    failures.append(f"{name}: {side} caster did not roll measurably")
        metrics["phases"][name] = phase_metric
    report = {"passed": not failures, "failures": failures, "metrics": metrics,
              "model_description_sha256": hashlib.sha256(description).hexdigest(),
              "phases": PHASES, "samples": samples,
              "scope": "Finite states, body support, ten passive caster DOFs, short low-speed motion, relative closed-loop frame stability. Does not calibrate contact slip, forces, masses or hardware behavior.",
              "wall_duration_s": time.monotonic()-wall_start}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({key: report[key] for key in ("passed", "failures", "metrics")}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
