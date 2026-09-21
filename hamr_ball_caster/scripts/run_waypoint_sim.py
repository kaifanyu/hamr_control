#!/usr/bin/env python3
"""Run a continuous, smooth-stop or legacy route on an existing COMPA sim.

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
    """Same linear interpolation and segment timing as WaypointTraj.update."""
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
            duration = max(1e-9, length/speed)
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
        vx, vy = self.velocities[index]
        return (x+vx*elapsed, y+vy*elapsed, vx, vy, index)


class SettledWaypointSchedule:
    """Run smooth segments and release each next segment only after settling.

    Time is seconds since motion started. Historical departures allow the
    reference to be evaluated at an odometry source timestamp, including holds.
    No measurement is moved or snapped to a waypoint.
    """
    def __init__(self, route, position_tolerance, speed_tolerance, dwell, timeout):
        values = (position_tolerance, speed_tolerance, dwell, timeout)
        if not all(math.isfinite(v) and v > 0 for v in values) or timeout <= dwell:
            raise ValueError("Positive settling limits and timeout > dwell required")
        self.route = route
        self.position_tolerance, self.speed_tolerance = values[:2]
        self.dwell, self.timeout = dwell, timeout
        self.index = 0
        self.departures = [0.]
        self.arrivals = [0.] + [None]*(len(route.points)-1)
        self.good_since = None
        self.completed_at = None
        self.state = "moving"
        self.events = []

    def update(self, time_s, position, speed):
        if self.completed_at is not None:
            return
        duration = self.route.starts[self.index+1]-self.route.starts[self.index]
        arrival = self.departures[self.index]+duration
        if time_s < arrival:
            return
        self.arrivals[self.index+1] = arrival
        self.state = "waypoint_settle"
        point = self.route.points[self.index+1]
        error = math.hypot(position[0]-point[0], position[1]-point[1])
        good = error <= self.position_tolerance and speed <= self.speed_tolerance
        if good:
            if self.good_since is None:
                self.good_since = time_s
        else:
            self.good_since = None
        if self.good_since is not None and time_s-self.good_since >= self.dwell:
            self.events.append({"waypoint_index": self.index+1,
                                "arrival_s": arrival, "release_s": time_s,
                                "position_error_m": error, "speed_m_s": speed})
            self.good_since = None
            self.index += 1
            if self.index == len(self.route.points)-1:
                self.completed_at = time_s
                self.state = "complete"
            else:
                self.departures.append(time_s)
                self.state = "moving"
        elif time_s-arrival > self.timeout:
            raise RuntimeError(f"Waypoint {self.index+1} did not settle: {error:.6f} m, {speed:.6f} m/s")

    def sample(self, time_s):
        if time_s <= 0:
            return (*self.route.points[0], 0., 0., 0, 0.)
        index = min(bisect.bisect_right(self.departures, time_s)-1,
                    len(self.route.points)-2)
        duration = self.route.starts[index+1]-self.route.starts[index]
        local = max(0., min(duration, time_s-self.departures[index]))
        reference_time = self.route.starts[index]+local
        if local >= duration:
            return (*self.route.points[index+1], 0., 0., index, reference_time)
        return (*self.route.sample(reference_time), reference_time)


def inverse_drive(vx_world, vy_world, base_yaw, radius, half_track, offset):
    """Inverse kinematics at a point offset ahead of a differential axle."""
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    forward = c*vx_world+s*vy_world
    lateral = -s*vx_world+c*vy_world
    yaw_rate = lateral/offset
    return ((forward-half_track*yaw_rate)/radius,
            (forward+half_track*yaw_rate)/radius, yaw_rate)


def predict_control_pose(x, y, yaw, velocity, odom_age, enabled, continuous=False):
    """Extrapolate feedback to the control clock within the accepted age window.

    Continuous tracking accepts up to 100 ms of source age. Capping prediction
    earlier creates an artificial position error as the reference keeps moving.
    The caller rejects feedback outside that window before using this estimate.
    Existing stop/legacy profiles retain their original 40 ms prediction cap.
    """
    horizon = .10 if continuous else .04
    age = max(0., min(horizon, odom_age)) if enabled else 0.
    return x+velocity[0]*age, y+velocity[1]*age, yaw+velocity[2]*age, age


def summarize(samples, route, completed, target_yaw=0., arrivals=None, duration=None,
              continuous=False):
    failures = []
    if not completed:
        failures.append("Trajectory and final hold did not complete")
    if not samples:
        return {"passed": False, "failures": failures+["No simulation feedback samples"]}
    errors = [s["position_error_m"] for s in samples if s["phase"] == "motion"]
    waypoint_results = []
    for i, (point, arrival) in enumerate(zip(route.points, arrivals or route.starts)):
        if arrival is None:
            failures.append(f"Waypoint {i} was never scheduled")
            waypoint_results.append({"index": i, "point_m": list(point), "scheduled_arrival_s": None,
                                     "minimum_distance_in_arrival_window_m": None})
            continue
        window = [s for s in samples if arrival-1. <= s["trajectory_time_s"] <= arrival+2.]
        distances = [math.hypot(s["position_m"][0]-point[0], s["position_m"][1]-point[1]) for s in window]
        distance = min(distances) if distances else None
        waypoint_results.append({"index": i, "point_m": list(point), "scheduled_arrival_s": arrival,
                                 "minimum_distance_in_arrival_window_m": distance})
        if continuous:
            waypoint_results[-1]["planned_pass_position_m"] = list(route.sample(arrival)[:2])
            waypoint_results[-1]["event"] = "closest planned pass; original corner need not be visited"
        if not continuous and (distance is None or distance > .20):
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
            "reference_duration_s": duration if duration is not None else route.duration,
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
    parser.add_argument("--profile", choices=("continuous", "smooth", "legacy"), default="smooth")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--output", type=Path, default=Path("simple_waypoint_run.json"))
    parser.add_argument("--speed", type=float, help="Override default 0.25 m/s")
    parser.add_argument("--startup-hold", type=float)
    parser.add_argument("--final-hold", type=float)
    parser.add_argument("--wall-timeout", type=float, default=900.)
    parser.add_argument("--plan-only", type=Path,
                        help="Export the continuous reference to JSON and exit without running the vehicle")
    args = parser.parse_args()
    config_name = {"continuous": "continuous_waypoint_sim.yaml",
                   "smooth": "smooth_waypoint_sim.yaml", "legacy": "simple_waypoint_sim.yaml"}[args.profile]
    config_path = args.config or Path(get_package_share_directory("hamr_ball_caster"))/"config"/config_name
    config_bytes = config_path.read_bytes()
    cfg = yaml.safe_load(config_bytes)
    implementation_sha256 = {"run_waypoint_sim.py": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    speed = args.speed if args.speed is not None else cfg["speed_m_s"]
    startup = args.startup_hold if args.startup_hold is not None else cfg["startup_hold_s"]
    final_hold = args.final_hold if args.final_hold is not None else cfg["final_hold_s"]
    if any(not math.isfinite(value) or value < 0 for value in (startup, final_hold)):
        parser.error("Holds must be finite and nonnegative")
    if not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0:
        parser.error("--wall-timeout must be finite and positive")
    if args.plan_only and args.profile != "continuous":
        parser.error("--plan-only requires --profile continuous")
    aligned_feedback = args.profile != "legacy"
    continuous_plan = None
    if args.profile == "continuous":
        from continuous_waypoint import ContinuousWaypointReference
        from reference_trajectory import continuous_waypoint
        implementation_sha256["continuous_waypoint.py"] = hashlib.sha256(
            Path(continuous_waypoint.__file__).read_bytes()).hexdigest()
        route = ContinuousWaypointReference(
            cfg["points_m"], speed, radius=cfg["wheel_radius_m"],
            half_track=cfg["half_track_m"], offset=cfg["base_ahead_of_axle_m"],
            max_translational_acceleration=cfg["planning_max_translation_acceleration_m_s2"],
            planning_wheel_rate=cfg["max_wheel_rate_rad_s"]*cfg["planning_wheel_rate_fraction"],
            planning_wheel_acceleration=cfg["max_wheel_acceleration_rad_s2"]*cfg["planning_wheel_acceleration_fraction"],
            corner_deviation=cfg["corner_deviation_m"], max_jerk=cfg["planning_max_jerk_m_s3"])
        continuous_plan = route.metadata()
        schedule = None
    elif args.profile == "smooth":
        from feasible_waypoint import FeasibleWaypointReference
        import feasible_waypoint
        implementation_sha256["feasible_waypoint.py"] = hashlib.sha256(
            Path(feasible_waypoint.__file__).read_bytes()).hexdigest()
        route = FeasibleWaypointReference(
            cfg["points_m"], speed, radius=cfg["wheel_radius_m"],
            half_track=cfg["half_track_m"], offset=cfg["base_ahead_of_axle_m"],
            max_translational_acceleration=cfg["planning_max_translation_acceleration_m_s2"],
            planning_wheel_rate=cfg["max_wheel_rate_rad_s"]*cfg["planning_wheel_rate_fraction"],
            planning_wheel_acceleration=cfg["max_wheel_acceleration_rad_s2"]*cfg["planning_wheel_acceleration_fraction"])
        schedule = SettledWaypointSchedule(route, cfg["settle_position_tolerance_m"],
            cfg["settle_speed_tolerance_m_s"], cfg["settle_dwell_s"], cfg["settle_timeout_s"])
    else:
        route = PolylineReference(cfg["points_m"], speed)
        schedule = None
    if args.plan_only:
        args.plan_only.parent.mkdir(parents=True, exist_ok=True)
        args.plan_only.write_text(json.dumps({"trajectory_profile": args.profile,
            "configuration": cfg, "speed_m_s": speed, "continuous_plan": continuous_plan,
            "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
            "implementation_sha256": implementation_sha256}, indent=2)+"\n")
        print(f"Saved continuous reference to {args.plan_only}", flush=True)
        return 0
    control_period = cfg.get("control_period_s", .02)
    if not math.isfinite(control_period) or not .001 <= control_period <= .1:
        parser.error("control_period_s must be between 0.001 and 0.1 seconds")
    rclpy.init(args=[])
    node = Node("hamr_simple_waypoint_sim")
    state = {"clock": None, "odom": None, "joints": None, "odom_wall": None,
             "velocity": (0., 0., 0.)}
    def clock_callback(msg):
        state["clock"] = msg.clock.sec+msg.clock.nanosec*1e-9
    def odom_callback(msg):
        previous_odom = state["odom"]
        if previous_odom is not None:
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            previous_stamp = previous_odom.header.stamp.sec+previous_odom.header.stamp.nanosec*1e-9
            elapsed = stamp-previous_stamp
            if elapsed <= 0:
                return
            if elapsed < .2:
                a, b = previous_odom.pose.pose, msg.pose.pose
                previous_yaw = quaternion_rpy(a.orientation)[2]
                current_yaw = quaternion_rpy(b.orientation)[2]
                state["velocity"] = ((b.position.x-a.position.x)/elapsed,
                                     (b.position.y-a.position.y)/elapsed,
                                     wrap(current_yaw-previous_yaw)/elapsed)
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
    last_odom_sample = -math.inf
    previous_wheels = (0., 0.)
    command_stats = {"updates": 0, "speed_limited_updates": 0,
                     "acceleration_limited_updates": 0, "max_wheel_rate_rad_s": 0.,
                     "max_wheel_acceleration_rad_s2": 0.}
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
                display_points = ([point["position_m"] for point in continuous_plan["samples"]]
                                  if continuous_plan else route.points)
                for x, y in display_points:
                    pose = PoseStamped()
                    pose.header.frame_id = "odom"
                    pose.pose.position.x, pose.pose.position.y = x, y
                    pose.pose.orientation.z = math.sin(cfg["fixed_turret_yaw_rad"]/2)
                    pose.pose.orientation.w = math.cos(cfg["fixed_turret_yaw_rad"]/2)
                    path.poses.append(pose)
                path_pub.publish(path)
                print(f"Starting {args.profile} waypoint route: {route.length:.1f} m / {route.duration:.1f} planned seconds, peak speed {speed:.2f} m/s", flush=True)
            if now < previous:
                raise RuntimeError("Simulation clock moved backwards")
            dt = now-previous
            if dt < control_period:
                continue
            previous = now
            elapsed = now-started
            trajectory_time = max(0., elapsed-startup)
            odom = state["odom"].pose.pose
            x, y, z = odom.position.x, odom.position.y, odom.position.z
            roll, pitch, yaw = quaternion_rpy(odom.orientation)
            joints = dict(zip(state["joints"].name, state["joints"].position))
            odom_stamp = state["odom"].header.stamp.sec+state["odom"].header.stamp.nanosec*1e-9
            odom_age = now-odom_stamp
            velocity = state["velocity"]
            if not all(math.isfinite(v) for v in (x, y, z, roll, pitch, yaw, *velocity, *joints.values())):
                raise RuntimeError("Non-finite state from Gazebo")
            if abs(roll) > .65 or abs(pitch) > .65 or not -.12 < z < .5:
                raise RuntimeError("Vehicle lost expected floor support")
            if aligned_feedback and not -.005 <= odom_age <= .10:
                raise RuntimeError(f"Simulation odometry source age outside bounds: {odom_age:.3f} s")
            control_x, control_y, control_yaw, prediction_age = predict_control_pose(
                x, y, yaw, velocity, odom_age, cfg.get("predict_odometry", False),
                continuous=args.profile == "continuous")
            motion_state = "startup_hold"
            reference_time = trajectory_time
            if elapsed < startup:
                phase = "startup_hold"
                rx, ry = route.points[0]
                vx = vy = 0.
                index = 0
            elif schedule:
                previous_index = schedule.index
                schedule.update(trajectory_time, (x, y), math.hypot(*velocity[:2]))
                if schedule.index != previous_index:
                    print(f"Waypoint {schedule.index} settled at {trajectory_time:.3f} s", flush=True)
                phase = "final_hold" if schedule.completed_at is not None else "motion"
                motion_state = schedule.state
                rx, ry, vx, vy, index, reference_time = schedule.sample(trajectory_time)
            else:
                phase = "motion" if trajectory_time < route.duration else "final_hold"
                rx, ry, vx, vy, index = route.sample(trajectory_time)
                motion_state = phase
            if phase != phase_previous:
                print(f"Phase {phase} at simulation elapsed {elapsed:.2f} s", flush=True)
                phase_previous = phase
            reference_velocity = [vx, vy]
            # Midpoint reference compensates the zero-order-held command.
            # It remains clamped at the endpoint while awaiting actual settling.
            if aligned_feedback and phase == "motion":
                lookahead = cfg.get("reference_lookahead_s", 0.)
                ahead = (schedule.sample(trajectory_time+lookahead) if schedule
                         else route.sample(trajectory_time+lookahead))
                vx, vy = ahead[2:4]
                control_yaw += velocity[2]*lookahead
            ex, ey = rx-control_x, ry-control_y
            vx += cfg["position_gain_s_inv"]*ex
            vy += cfg["position_gain_s_inv"]*ey
            norm = math.hypot(vx, vy)
            if norm > cfg["max_translation_m_s"]:
                vx *= cfg["max_translation_m_s"]/norm
                vy *= cfg["max_translation_m_s"]/norm
            left, right, omega = inverse_drive(vx, vy, control_yaw, cfg["wheel_radius_m"], cfg["half_track_m"], cfg["base_ahead_of_axle_m"])
            raw_wheels = [left, right]
            factor = min(1., cfg["max_wheel_rate_rad_s"]/max(abs(left), abs(right), 1e-12))
            left, right = left*factor, right*factor
            speed_limited_wheels = (left, right)
            acceleration = cfg["max_wheel_acceleration_rad_s2"]*min(dt, .1)
            left = max(previous_wheels[0]-acceleration, min(previous_wheels[0]+acceleration, left))
            right = max(previous_wheels[1]-acceleration, min(previous_wheels[1]+acceleration, right))
            command_stats["updates"] += 1
            command_stats["speed_limited_updates"] += factor < 1.-1e-12
            command_stats["acceleration_limited_updates"] += any(abs(a-b) > 1e-10 for a, b in zip((left, right), speed_limited_wheels))
            command_stats["max_wheel_rate_rad_s"] = max(command_stats["max_wheel_rate_rad_s"], abs(left), abs(right))
            command_stats["max_wheel_acceleration_rad_s2"] = max(command_stats["max_wheel_acceleration_rad_s2"],
                abs(left-previous_wheels[0])/dt, abs(right-previous_wheels[1])/dt)
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
            if (aligned_feedback and odom_stamp > last_odom_sample) or (not aligned_feedback and now-last_sample >= .05):
                last_sample = now
                last_odom_sample = odom_stamp
                sample_reference = [rx, ry]
                sample_velocity = reference_velocity
                sample_time = trajectory_time
                if aligned_feedback:
                    sample_time = odom_stamp-started-startup
                    at_pose = schedule.sample(sample_time) if schedule else route.sample(sample_time)
                    sx, sy, svx, svy = at_pose[:4]
                    sample_reference = [sx, sy]
                    sample_velocity = [svx, svy]
                sample = {"simulation_time_s": now,
                          "elapsed_s": odom_stamp-started if aligned_feedback else elapsed,
                          "controller_elapsed_s": elapsed,
                          "trajectory_time_s": sample_time, "reference_time_s": reference_time,
                          "phase": phase, "motion_state": motion_state, "segment": index,
                          "reference_m": sample_reference, "reference_velocity_m_s": sample_velocity,
                          "position_m": [x, y, z], "odometry_stamp_s": odom_stamp,
                          "odometry_age_s": odom_age, "prediction_age_s": prediction_age,
                          "position_error_m": math.hypot(sample_reference[0]-x, sample_reference[1]-y),
                          "controller_reference_m": [rx, ry], "control_position_error_m": math.hypot(ex, ey),
                          "estimated_world_velocity_m_s": list(velocity[:2]),
                          "raw_wheels_rad_s": raw_wheels, "wheel_speed_scale": factor,
                          "controller_dt_s": dt, "base_rpy_rad": [roll, pitch, yaw],
                          "turret_world_yaw_rad": turret_yaw, "commands_rad_s": commands,
                          "joint_positions_rad": joints,
                          "joint_velocities_rad_s": dict(zip(state["joints"].name, state["joints"].velocity))}
                samples.append(sample)
                status_pub.publish(String(data=json.dumps({key:sample[key] for key in (
                    "elapsed_s", "trajectory_time_s", "phase", "segment", "reference_m", "position_m", "position_error_m")})))
            finish_time = schedule.completed_at if schedule else route.duration
            feedback_covers_finish = not aligned_feedback or (finish_time is not None and odom_stamp-started-startup >= finish_time)
            if finish_time is not None and trajectory_time >= finish_time+final_hold and feedback_covers_finish:
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
    arrivals = schedule.arrivals if schedule else route.starts
    motion_duration = schedule.completed_at if schedule else route.duration
    summary = summarize(samples, route, completed, cfg["fixed_turret_yaw_rad"], arrivals, motion_duration,
                        continuous=args.profile == "continuous")
    if failure:
        summary["passed"] = False
        summary["failures"].append(failure)
    report = {"summary": summary, "configuration": cfg, "speed_m_s": speed,
              "completed": completed, "trajectory_profile": args.profile,
              "reference_schedule_s": arrivals,
              "reference_departure_schedule_s": (schedule.departures+[schedule.completed_at]
                  if schedule else route.starts),
              "planned_reference_schedule_s": route.starts,
              "planned_reference_duration_s": route.duration,
              "settling_events": schedule.events if schedule else [],
              "command_statistics": command_stats,
              "planner_segments": route.segments if aligned_feedback else [],
              "measurement_alignment": "Reference evaluated at raw odometry source stamp" if aligned_feedback else "Legacy reference at controller clock, raw latest odometry",
              "startup_hold_s": startup, "final_hold_s": final_hold,
              "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
              "implementation_sha256": implementation_sha256,
              "wall_duration_s": time.monotonic()-wall_started,
              "scope": "Full COMPA/HAMR retrofit with two CAD casters; actual Gazebo physics feedback and offset-drive inverse kinematics with fixed upper yaw. Continuous profile rounds corners with a bounded-deviation curve and one uninterrupted speed plan. Smooth profile preserves exact straight segments with planned stops and measured settling; legacy retains original abrupt timing. Ideal velocity actuators and nominal physical parameters remain simulation assumptions.",
              "samples": samples}
    if continuous_plan is not None:
        report["continuous_plan"] = continuous_plan
    if aligned_feedback:
        # Same analyzer is usable offline and installed beside this runner.
        import importlib.util
        analysis_name = "analyze_continuous_tracking.py" if continuous_plan else "analyze_corner_tracking.py"
        analysis_path = Path(__file__).resolve().parent/analysis_name
        if not analysis_path.exists():
            analysis_path = Path(__file__).resolve().parents[1]/"tools"/analysis_name
        spec = importlib.util.spec_from_file_location("corner_acceptance", analysis_path)
        analyzer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(analyzer)
        implementation_sha256[analysis_name] = hashlib.sha256(analysis_path.read_bytes()).hexdigest()
        acceptance = analyzer.analyze_report(report, cfg["acceptance"])
        report["continuous_acceptance" if continuous_plan else "corner_acceptance"] = acceptance
        if not acceptance["passed"]:
            summary["passed"] = False
            summary["failures"].extend(acceptance["failures"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(summary, indent=2), flush=True)
    print(f"Saved simulation measurements to {args.output}", flush=True)
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
