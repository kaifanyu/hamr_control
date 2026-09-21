#!/usr/bin/env python3
"""Run the original simple waypoint schedule on an already-running COMPA sim.

Publishes only to the Gazebo bridge topics. Requires model:=compa and
controller:=false, a dedicated ROS_DOMAIN_ID, and live simulation feedback.
This is an offset-differential-drive simulation adapter, not a hardware node.
The chassis can rotate while the upper yaw platform maintains world yaw zero.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import statistics
import time

import yaml


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return (
        math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
        math.asin(max(-1., min(1., 2*(w*y-z*x)))),
        math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)),
    )


class PolylineReference:
    """EXPERIMENT ONLY: quintic time law per straight segment; zero endpoint speed/acceleration. Duration=1.875*length/maxspeed."""
    def __init__(self, points, speed):
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("speed must be finite and positive")
        self.points = [tuple(map(float, point)) for point in points]
        if len(self.points) < 2 or any(len(point) != 2 for point in self.points):
            raise ValueError("Need at least two XY waypoints")
        if not all(math.isfinite(v) for point in self.points for v in point):
            raise ValueError("Waypoints must be finite")
        self.starts = [0.]
        self.velocities = []
        self.length = 0.
        for first, second in zip(self.points[:-1], self.points[1:]):
            dx, dy = second[0]-first[0], second[1]-first[1]
            length = math.hypot(dx, dy)
            duration = max(1e-9, 1.875*length/speed)
            self.length += length
            self.starts.append(self.starts[-1]+duration)
            self.velocities.append((dx/duration, dy/duration))
        self.duration = self.starts[-1]

    def sample(self, time_s):
        if time_s >= self.duration:
            return (*self.points[-1], 0., 0., len(self.velocities)-1)
        index = max(0, bisect.bisect_right(self.starts, max(0., time_s))-1)
        elapsed = max(0., time_s-self.starts[index])
        x, y = self.points[index]
        xn, yn = self.points[index+1]
        duration = self.starts[index+1]-self.starts[index]
        u = min(1., elapsed/duration)
        fraction = 10*u**3-15*u**4+6*u**5
        rate = (30*u**2-60*u**3+30*u**4)/duration
        return (x+(xn-x)*fraction, y+(yn-y)*fraction,
                (xn-x)*rate, (yn-y)*rate, index)


def inverse_drive(vx_world, vy_world, base_yaw, radius, half_track, offset):
    """Inverse kinematics at a point offset ahead of a differential axle."""
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    forward = c*vx_world+s*vy_world
    lateral = -s*vx_world+c*vy_world
    yaw_rate = lateral/offset
    return ((forward-half_track*yaw_rate)/radius,
            (forward+half_track*yaw_rate)/radius, yaw_rate)


def summarize(samples, route, completed, target_yaw=0.):
    failures = []
    if not completed:
        failures.append("Trajectory and final hold did not complete")
    if not samples:
        return {"passed": False, "failures": failures+["No simulation feedback samples"]}
    errors = [s["position_error_m"] for s in samples if s["phase"] == "motion"]
    waypoint_results = []
    for i, (point, arrival) in enumerate(zip(route.points, route.starts)):
        window = [s for s in samples if arrival-1. <= s["trajectory_time_s"] <= arrival+2.]
        distances = [math.hypot(s["position_m"][0]-point[0], s["position_m"][1]-point[1]) for s in window]
        distance = min(distances) if distances else None
        waypoint_results.append({"index": i, "point_m": list(point), "scheduled_arrival_s": arrival,
                                 "minimum_distance_in_arrival_window_m": distance})
        if distance is None or distance > .20:
            failures.append(f"Waypoint {i} was not reached within 0.20 m in its arrival window")
    final_error = samples[-1]["position_error_m"]
    if final_error > .08:
        failures.append(f"Final position error exceeded 0.08 m: {final_error:.4f}")
    max_tilt = max(abs(v) for s in samples for v in s["base_rpy_rad"][:2])
    if max_tilt > .4:
        failures.append(f"Base roll/pitch exceeded 0.4 rad: {max_tilt:.4f}")
    if any(not math.isfinite(v) for s in samples for key in ("position_m", "base_rpy_rad", "commands_rad_s") for v in s[key]):
        failures.append("Non-finite simulation state or command")
    passive_names = [f"{side}_caster_{joint}_joint" for side in ("left", "right") for joint in (
        "carrier", "positive_hemisphere", "negative_hemisphere", "positive_polar_roller", "negative_polar_roller")]
    missing = set(passive_names)-set(samples[-1]["joint_positions_rad"])
    if missing:
        failures.append(f"Missing passive caster joints: {sorted(missing)}")
    passive_spans = {name: max(s["joint_positions_rad"].get(name, 0.) for s in samples)-min(s["joint_positions_rad"].get(name, 0.) for s in samples) for name in passive_names}
    for side in ("left", "right"):
        if max(value for name, value in passive_spans.items() if name.startswith(side)) < 1.:
            failures.append(f"{side} caster did not rotate through the route")
    final_yaw_error = abs(wrap(samples[-1]["turret_world_yaw_rad"]-target_yaw))
    if final_yaw_error > .10:
        failures.append(f"Final turret yaw error exceeded 0.10 rad: {final_yaw_error:.4f}")
    return {"passed": not failures, "failures": failures,
            "sample_count": len(samples), "route_length_m": route.length,
            "reference_duration_s": route.duration,
            "motion_rmse_m": math.sqrt(statistics.mean(e*e for e in errors)) if errors else None,
            "motion_max_error_m": max(errors) if errors else None,
            "final_position_error_m": final_error,
            "final_turret_yaw_error_rad": final_yaw_error,
            "max_base_roll_pitch_rad": max_tilt,
            "waypoints": waypoint_results, "passive_joint_spans_rad": passive_spans}


def main():
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path as PathMessage
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64, String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--output", type=Path, default=Path("simple_waypoint_run.json"))
    parser.add_argument("--speed", type=float, help="Override default 0.25 m/s")
    parser.add_argument("--startup-hold", type=float)
    parser.add_argument("--final-hold", type=float)
    parser.add_argument("--wall-timeout", type=float, default=900.)
    args = parser.parse_args()
    config_path = args.config or Path(get_package_share_directory("hamr_ball_caster"))/"config/simple_waypoint_sim.yaml"
    config_bytes = config_path.read_bytes()
    cfg = yaml.safe_load(config_bytes)
    speed = args.speed if args.speed is not None else cfg["speed_m_s"]
    startup = args.startup_hold if args.startup_hold is not None else cfg["startup_hold_s"]
    final_hold = args.final_hold if args.final_hold is not None else cfg["final_hold_s"]
    if any(not math.isfinite(value) or value < 0 for value in (startup, final_hold)):
        parser.error("Holds must be finite and nonnegative")
    if not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0:
        parser.error("--wall-timeout must be finite and positive")
    route = PolylineReference(cfg["points_m"], speed)
    rclpy.init(args=[])
    node = Node("hamr_simple_waypoint_sim")
    state = {"clock": None, "odom": None, "joints": None, "odom_wall": None}
    def clock_callback(msg):
        state["clock"] = msg.clock.sec+msg.clock.nanosec*1e-9
    def odom_callback(msg):
        state["odom"] = msg
        state["odom_wall"] = time.monotonic()
    def joint_callback(msg):
        state["joints"] = msg
    node.create_subscription(Clock, "/clock", clock_callback, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/compa/odom", odom_callback, qos_profile_sensor_data)
    node.create_subscription(JointState, "/joint_states", joint_callback, qos_profile_sensor_data)
    pubs = [node.create_publisher(Float64, topic, 10) for topic in (
        "/left_wheel/cmd_vel", "/right_wheel/cmd_vel", "/roll/cmd_vel", "/pitch/cmd_vel", "/yaw/cmd_vel")]
    reference_pub = node.create_publisher(PoseStamped, "/ball_caster/reference_pose", 10)
    status_pub = node.create_publisher(String, "/ball_caster/waypoint_status", 10)
    path_pub = node.create_publisher(PathMessage, "/ball_caster/waypoints_path", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def command(values):
        for pub, value in zip(pubs, values):
            pub.publish(Float64(data=float(value)))
    samples = []
    started = previous = None
    last_sample = -math.inf
    previous_wheels = (0., 0.)
    wall_started = time.monotonic()
    completed = False
    failure = None
    phase_previous = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            if time.monotonic()-wall_started > args.wall_timeout:
                raise RuntimeError("Wall-clock timeout while waiting for feedback or run completion")
            if any(state[key] is None for key in ("clock", "odom", "joints")):
                continue
            if time.monotonic()-state["odom_wall"] > 3.:
                raise RuntimeError("Simulation odometry stopped arriving")
            now = state["clock"]
            if started is None:
                if not all(pub.get_subscription_count() > 0 for pub in pubs):
                    continue
                expected = {f"{side}_caster_{joint}_joint" for side in ("left", "right") for joint in (
                    "carrier", "positive_hemisphere", "negative_hemisphere", "positive_polar_roller", "negative_polar_roller")}
                expected.update(("left_rocker_left_wheel_joint", "right_rocker_right_wheel_joint",
                                 "base_roll_joint", "roll_pitch_joint", "pitch_yaw_plate_joint"))
                missing = expected-set(state["joints"].name)
                if missing:
                    raise RuntimeError(f"Expected the full COMPA CAD-caster simulation; missing joints: {sorted(missing)}")
                started = previous = now
                path = PathMessage()
                path.header.frame_id = "odom"
                for x, y in route.points:
                    pose = PoseStamped()
                    pose.header.frame_id = "odom"
                    pose.pose.position.x, pose.pose.position.y = x, y
                    pose.pose.orientation.z = math.sin(cfg["fixed_turret_yaw_rad"]/2)
                    pose.pose.orientation.w = math.cos(cfg["fixed_turret_yaw_rad"]/2)
                    path.poses.append(pose)
                path_pub.publish(path)
                print(f"Starting original simple waypoint route: {route.length:.1f} m / {route.duration:.1f} s, speed {speed:.2f} m/s", flush=True)
            if now < previous:
                raise RuntimeError("Simulation clock moved backwards")
            dt = now-previous
            if dt < .02:
                continue
            previous = now
            elapsed = now-started
            trajectory_time = max(0., elapsed-startup)
            if elapsed < startup:
                phase = "startup_hold"
                rx, ry = route.points[0]
                vx = vy = 0.
                index = 0
            else:
                phase = "motion" if trajectory_time < route.duration else "final_hold"
                rx, ry, vx, vy, index = route.sample(trajectory_time)
            if phase != phase_previous:
                print(f"Phase {phase} at simulation elapsed {elapsed:.2f} s", flush=True)
                phase_previous = phase
            odom = state["odom"].pose.pose
            x, y, z = odom.position.x, odom.position.y, odom.position.z
            roll, pitch, yaw = quaternion_rpy(odom.orientation)
            joints = dict(zip(state["joints"].name, state["joints"].position))
            if not all(math.isfinite(v) for v in (x, y, z, roll, pitch, yaw, *joints.values())):
                raise RuntimeError("Non-finite state from Gazebo")
            if abs(roll) > .65 or abs(pitch) > .65 or not -.12 < z < .5:
                raise RuntimeError("Vehicle lost expected floor support")
            ex, ey = rx-x, ry-y
            vx += cfg["position_gain_s_inv"]*ex
            vy += cfg["position_gain_s_inv"]*ey
            norm = math.hypot(vx, vy)
            if norm > cfg["max_translation_m_s"]:
                vx *= cfg["max_translation_m_s"]/norm
                vy *= cfg["max_translation_m_s"]/norm
            left, right, omega = inverse_drive(vx, vy, yaw, cfg["wheel_radius_m"], cfg["half_track_m"], cfg["base_ahead_of_axle_m"])
            raw_wheels = (left, right)
            factor = min(1., cfg["max_wheel_rate_rad_s"]/max(abs(left), abs(right), 1e-12))
            left, right = left*factor, right*factor
            speed_limited_wheels = (left, right)
            acceleration = cfg["max_wheel_acceleration_rad_s2"]*min(dt, .1)
            left = max(previous_wheels[0]-acceleration, min(previous_wheels[0]+acceleration, left))
            right = max(previous_wheels[1]-acceleration, min(previous_wheels[1]+acceleration, right))
            previous_wheels = (left, right)
            omega = cfg["wheel_radius_m"]*(right-left)/(2*cfg["half_track_m"])
            turret_yaw = wrap(yaw+joints.get("pitch_yaw_plate_joint", 0.))
            gain = cfg["orientation_gain_s_inv"]
            gimbal = [-gain*(roll+joints.get("base_roll_joint", 0.)),
                      -gain*(pitch+joints.get("roll_pitch_joint", 0.)),
                      -omega+gain*wrap(cfg["fixed_turret_yaw_rad"]-turret_yaw)]
            gimbal = [max(-cfg["max_gimbal_rate_rad_s"], min(cfg["max_gimbal_rate_rad_s"], value)) for value in gimbal]
            commands = [left, right, *gimbal]
            command(commands)
            reference = PoseStamped()
            reference.header.frame_id = "odom"
            reference.header.stamp.sec = int(now)
            reference.header.stamp.nanosec = int((now-int(now))*1e9)
            reference.pose.position.x, reference.pose.position.y = rx, ry
            reference.pose.orientation.z = math.sin(cfg["fixed_turret_yaw_rad"]/2)
            reference.pose.orientation.w = math.cos(cfg["fixed_turret_yaw_rad"]/2)
            reference_pub.publish(reference)
            if True:  # Experiment: retain every controller update
                last_sample = now
                sample = {"simulation_time_s": now, "elapsed_s": elapsed,
                          "trajectory_time_s": trajectory_time, "phase": phase, "segment": index,
                          "reference_m": [rx, ry], "position_m": [x, y, z],
                          "position_error_m": math.hypot(ex, ey), "base_rpy_rad": [roll, pitch, yaw],
                          "turret_world_yaw_rad": turret_yaw, "commands_rad_s": commands,
                          "joint_positions_rad": joints,
                          "joint_velocities_rad_s": dict(zip(state["joints"].name, state["joints"].velocity)),
                          "odometry_stamp_s": state["odom"].header.stamp.sec + 1e-9*state["odom"].header.stamp.nanosec,
                          "joint_stamp_s": state["joints"].header.stamp.sec + 1e-9*state["joints"].header.stamp.nanosec,
                          "controller_dt_s": dt, "requested_world_velocity_m_s": [vx, vy],
                          "raw_wheels_rad_s": raw_wheels, "speed_limited_wheels_rad_s": speed_limited_wheels,
                          "odometry_twist": {"linear": [state["odom"].twist.twist.linear.x, state["odom"].twist.twist.linear.y, state["odom"].twist.twist.linear.z],
                                             "angular": [state["odom"].twist.twist.angular.x, state["odom"].twist.twist.angular.y, state["odom"].twist.twist.angular.z]}}
                samples.append(sample)
                status_pub.publish(String(data=json.dumps({key:sample[key] for key in (
                    "elapsed_s", "trajectory_time_s", "phase", "segment", "reference_m", "position_m", "position_error_m")})))
            if trajectory_time >= route.duration+final_hold:
                completed = True
                break
    except (RuntimeError, KeyboardInterrupt) as exc:
        failure = str(exc) or "Interrupted"
    finally:
        for _ in range(10):
            command([0.]*5)
            rclpy.spin_once(node, timeout_sec=.02)
        node.destroy_node()
        rclpy.shutdown()
    summary = summarize(samples, route, completed, cfg["fixed_turret_yaw_rad"])
    if failure:
        summary["passed"] = False
        summary["failures"].append(failure)
    report = {"summary": summary, "configuration": cfg, "speed_m_s": speed,
              "startup_hold_s": startup, "final_hold_s": final_hold,
              "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
              "wall_duration_s": time.monotonic()-wall_started,
              "reference_time_law": "quintic_10u3_minus15u4_plus6u5; stop at each segment end; max speed is configured speed",
              "scope": 'Experiment: full COMPA/HAMR retrofit with two CAD casters, unchanged 15 rad/s^2 wheel slew limit and physics. The seven original waypoint coordinates are retained; timing is changed to quintic 10u^3-15u^4+6u^5 on each segment, with zero velocity and acceleration at every waypoint, maximum reference speed .25 m/s, 97.5 s motion. Ideal velocity actuators and nominal physical parameters remain simulation assumptions.',
              "samples": samples}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(summary, indent=2), flush=True)
    print(f"Saved simulation measurements to {args.output}", flush=True)
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
