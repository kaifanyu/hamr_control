"""Live waypoint replacement using the repository's physical COMPA controller.

Passive caster joints are observed, never commanded. The original smooth-stop
planner is reused and only the five existing actuator bridge topics are sent.
"""
from __future__ import annotations

import importlib.util
from dataclasses import asdict, dataclass
import math
from pathlib import Path
import threading
import time


@dataclass(frozen=True)
class MotionLimits:
    """Explicit simulator command and nominal planning limits, in SI units.

    These are software constraints, not a measurement of motor capability.
    Planning bounds normally leave room for feedback correction.
    """
    max_speed: float = .4
    max_wheel_rate: float = 6.
    max_wheel_acceleration: float = 15.
    planning_wheel_rate: float = 3.9
    planning_wheel_acceleration: float = 7.5
    max_translational_acceleration: float = .10

    def __post_init__(self):
        for name, value in asdict(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.max_speed < .02:
            raise ValueError("max_speed must be at least 0.02 m/s")
        if self.planning_wheel_rate > self.max_wheel_rate:
            raise ValueError("Planning wheel rate must not exceed the command wheel rate")
        if self.planning_wheel_acceleration > self.max_wheel_acceleration:
            raise ValueError("Planning wheel acceleration must not exceed the command wheel acceleration")

    def validate_speed(self, speed):
        if not math.isfinite(speed) or not .02 <= speed <= self.max_speed:
            raise ValueError(f"Trajectory speed must be between 0.02 and {self.max_speed:g} m/s")

    def to_dict(self):
        return asdict(self)


def add_limit_arguments(parser):
    group = parser.add_argument_group("motion limits (software bounds, not hardware ratings)")
    group.add_argument("--max-speed", type=float, default=.4, help="Translation command ceiling, m/s (default: .4)")
    group.add_argument("--wheel-rate", type=float, default=6., help="Wheel command ceiling, rad/s (default: 6)")
    group.add_argument("--wheel-acceleration", type=float, default=15., help="Wheel command acceleration, rad/s²")
    group.add_argument("--planning-wheel-rate", type=float, default=3.9, help="Nominal planning wheel ceiling, rad/s")
    group.add_argument("--planning-wheel-acceleration", type=float, default=7.5, help="Nominal planning wheel acceleration, rad/s²")
    group.add_argument("--acceleration", type=float, default=.1, help="Nominal translation acceleration, m/s²")
    return parser


def limits_from_args(args):
    return MotionLimits(max_speed=getattr(args, "max_speed", .4),
        max_wheel_rate=getattr(args, "wheel_rate", 6.),
        max_wheel_acceleration=getattr(args, "wheel_acceleration", 15.),
        planning_wheel_rate=getattr(args, "planning_wheel_rate", 3.9),
        planning_wheel_acceleration=getattr(args, "planning_wheel_acceleration", 7.5),
        max_translational_acceleration=getattr(args, "acceleration", .1))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_rpy(q):
    return (math.atan2(2 * (q.w*q.x + q.y*q.z), 1 - 2 * (q.x*q.x + q.y*q.y)),
            math.asin(max(-1., min(1., 2 * (q.w*q.y - q.z*q.x)))),
            math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z)))


def inverse_drive(vx, vy, yaw, radius=.1075, half_track=.33072, offset=.27114):
    forward = math.cos(yaw)*vx + math.sin(yaw)*vy
    angular = (-math.sin(yaw)*vx + math.cos(yaw)*vy) / offset
    return ((forward-half_track*angular)/radius,
            (forward+half_track*angular)/radius, angular)


def validate_points(points):
    result = [tuple(float(v) for v in point) for point in points]
    if not result or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in result):
        raise ValueError("Enter at least one finite XY destination")
    if any(math.dist(a, b) < 1e-5 for a, b in zip(result, result[1:])):
        raise ValueError("Consecutive waypoints must differ")
    return result


class MotionPlan:
    """Measured settling between exact, smooth straight segments."""

    def __init__(self, points, speed, start, now, repo_root=None, limits=None):
        self.limits = limits or MotionLimits()
        self.limits.validate_speed(speed)
        requested = validate_points(points)
        start = tuple(float(v) for v in start)
        if len(start) != 2 or not all(math.isfinite(v) for v in start):
            raise ValueError("Current position is invalid")
        # Continue from measured position; never teleport to a route's start.
        if math.dist(start, requested[0]) < .02:
            requested[0] = start
        else:
            requested.insert(0, start)
        if len(requested) < 2:
            raise ValueError("The destination is already at the current position")
        root = Path(repo_root or Path(__file__).resolve().parents[2])
        source = root / "hamr_ball_caster/scripts/feasible_waypoint.py"
        spec = importlib.util.spec_from_file_location("_hamr_ui_feasible", source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.route = module.FeasibleWaypointReference(requested, speed, radius=.1075,
            half_track=.33072, offset=.27114,
            max_translational_acceleration=self.limits.max_translational_acceleration,
            planning_wheel_rate=self.limits.planning_wheel_rate,
            planning_wheel_acceleration=self.limits.planning_wheel_acceleration)
        self.points, self.speed, self.index = requested, speed, 0
        self.epoch, self.settled_since = now, None
        self.state, self.error = "moving", ""

    def summary(self):
        return {"requested_peak_speed_m_s": self.speed,
                "planned_peak_speed_m_s": max(item["peak_speed_m_s"] for item in self.route.segments),
                "nominal_duration_s": self.route.duration,
                "segments": [dict(item) for item in self.route.segments],
                "motion_limits": self.limits.to_dict()}

    def sample(self, now, xy, measured_speed):
        duration = self.route.starts[self.index+1] - self.route.starts[self.index]
        elapsed = max(0., now-self.epoch)
        if elapsed >= duration:
            self.state = "settling"
            distance = math.dist(xy, self.points[self.index+1])
            if distance < .02 and measured_speed < .025:
                if self.settled_since is None:
                    self.settled_since = now
                if now-self.settled_since >= .2:
                    if self.index == len(self.points)-2:
                        self.state = "complete"
                    else:
                        self.index += 1
                        self.epoch, self.settled_since = now, None
                        elapsed, duration = 0., self.route.starts[self.index+1]-self.route.starts[self.index]
                        self.state = "moving"
            else:
                self.settled_since = None
            if elapsed > duration + 20. and self.state != "complete":
                self.state = "error"
                self.error = f"Waypoint {self.index+1} did not settle; inspect terrain or choose a new route"
        sample = self.route.sample(self.route.starts[self.index] + min(elapsed, duration))
        return sample


class LiveController:
    """Attach to an existing rclpy node; public methods are thread safe."""

    def __init__(self, node, repo_root=None, limits=None):
        from nav_msgs.msg import Odometry
        from rclpy.clock import Clock as RosClock, ClockType
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64

        self.node, self.root, self._message = node, repo_root, Float64
        self.limits = limits or MotionLimits()
        self._lock = threading.RLock()
        self._clock = self._previous_clock = None
        self._odom = self._joints = None
        self._odom_wall = 0.
        self._velocity = (0., 0., 0.)
        self._wheels = (0., 0.)
        self._plan = None
        self._manual = None
        self._manual_wall = 0.
        self._state, self._error = "idle", ""
        self._reference = None
        self._publishers = [node.create_publisher(Float64, topic, 10) for topic in (
            "/left_wheel/cmd_vel", "/right_wheel/cmd_vel", "/roll/cmd_vel", "/pitch/cmd_vel", "/yaw/cmd_vel")]
        self._subscriptions = [
            node.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data),
            node.create_subscription(Odometry, "/compa/odom", self._on_odom, qos_profile_sensor_data),
            node.create_subscription(JointState, "/joint_states", self._on_joints, qos_profile_sensor_data)]
        # Wall timer handles stalled feedback; actual integration uses /clock.
        self._timer = node.create_timer(.01, self._tick, clock=RosClock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def _stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec*1e-9

    def _on_clock(self, message):
        with self._lock:
            now = message.clock.sec + message.clock.nanosec*1e-9
            if self._clock is not None and now < self._clock:
                self.reset()
            self._clock = now

    def _on_odom(self, message):
        with self._lock:
            if self._odom is not None:
                dt = self._stamp(message)-self._stamp(self._odom)
                if 0 < dt < .5:
                    before, after = self._odom.pose.pose, message.pose.pose
                    self._velocity = ((after.position.x-before.position.x)/dt,
                                      (after.position.y-before.position.y)/dt,
                                      wrap(quaternion_rpy(after.orientation)[2]-quaternion_rpy(before.orientation)[2])/dt)
            self._odom, self._odom_wall = message, time.monotonic()

    def _on_joints(self, message):
        with self._lock:
            self._joints = message

    def _command(self, values):
        for publisher, value in zip(self._publishers, values):
            publisher.publish(self._message(data=float(value)))

    def set_route(self, points_xy, speed=.20):
        with self._lock:
            if self._odom is None or self._joints is None or self._clock is None:
                raise RuntimeError("Wait for live Gazebo odometry and caster feedback before running a route")
            if time.monotonic()-self._odom_wall > 3.:
                raise RuntimeError("Odometry is stale; resume the simulator and wait for live feedback")
            position = self._odom.pose.pose.position
            plan = MotionPlan(points_xy, float(speed), (position.x, position.y), self._clock, self.root, self.limits)
            self._plan, self._state, self._error = plan, "moving", ""
            self._manual = None
            self._previous_clock = self._clock
            return list(plan.points)

    def set_speed(self, speed):
        """Replan unvisited destinations from actual pose at the new peak speed."""
        with self._lock:
            self.limits.validate_speed(speed)
            if self._plan is None or self._plan.state == "complete":
                return
            return self.set_route(self._plan.points[self._plan.index+1:], speed)

    def manual(self, forward_m_s, yaw_rad_s, lateral_m_s=0.):
        """Body forward and chassis yaw commands; resend at >= 10 Hz while held.

        No command survives 0.30 wall seconds without a refresh. Releasing keys
        or losing focus should also call halt() immediately.
        """
        values = tuple(float(v) for v in (forward_m_s, yaw_rad_s, lateral_m_s))
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Manual commands must be finite")
        if abs(values[0]) > self.limits.max_speed or abs(values[1]) > 1.5 or abs(values[2]) > self.limits.max_speed:
            raise ValueError(f"Manual commands exceed {self.limits.max_speed:g} m/s or 1.5 rad/s")
        with self._lock:
            if self._odom is None or self._joints is None or self._clock is None:
                raise RuntimeError("Wait for live simulator feedback before driving")
            self._manual, self._manual_wall = values, time.monotonic()
            self._plan, self._state, self._error = None, "manual", ""

    def configure_limits(self, limits):
        if not isinstance(limits, MotionLimits):
            raise TypeError("limits must be MotionLimits")
        with self._lock:
            self.halt()
            self.limits = limits

    def halt(self):
        with self._lock:
            self._plan = None
            self._manual = None
            self._state, self._error, self._wheels = "stopped", "", (0., 0.)
            self._command([0.]*5)

    def reset(self):
        with self._lock:
            self.halt()
            self._odom = self._joints = self._clock = self._previous_clock = None
            self._reference, self._state = None, "idle"
            self._velocity = (0., 0., 0.)

    def snapshot(self):
        with self._lock:
            pose = self._odom.pose.pose if self._odom is not None else None
            rpy = quaternion_rpy(pose.orientation) if pose else None
            return {"state": self._state, "error": self._error,
                    "feedback_ready": self._odom is not None and self._joints is not None and self._clock is not None,
                    "feedback_age_s": max(0., time.monotonic()-self._odom_wall) if pose else None,
                    "odom_time": self._stamp(self._odom) if pose else None,
                    "points": list(self._plan.points) if self._plan else [],
                    "speed": self._plan.speed if self._plan else 0.,
                    "plan_summary": self._plan.summary() if self._plan else None,
                    "segment": self._plan.index if self._plan else 0,
                    "position": (pose.position.x, pose.position.y) if pose else None,
                    "z": pose.position.z if pose else None,
                    "roll": rpy[0] if rpy else None,
                    "pitch": rpy[1] if rpy else None,
                    "yaw": rpy[2] if rpy else None,
                    "reference": self._reference,
                    "velocity": self._velocity,
                    "sim_time": self._clock}

    def _tick(self):
        with self._lock:
            if self._manual is not None and time.monotonic()-self._manual_wall > .30:
                self.halt()
                return
            if self._plan is None and self._manual is None:
                return
            if self._clock is None or self._odom is None or self._joints is None:
                return
            if self._previous_clock is None:
                self._previous_clock = self._clock
            dt = self._clock-self._previous_clock
            if dt < .009:
                return
            self._previous_clock = self._clock
            age = self._clock-self._stamp(self._odom)
            if time.monotonic()-self._odom_wall > 3. or not -.05 <= age <= .25:
                self._command([0.]*5)
                self._wheels = (0., 0.)
                self._state, self._error = "error", "Odometry stopped or is too old; stopped the route"
                self._plan = None
                self._manual = None
                return
            pose = self._odom.pose.pose
            x, y, z = pose.position.x, pose.position.y, pose.position.z
            roll, pitch, yaw = quaternion_rpy(pose.orientation)
            joints = dict(zip(self._joints.name, self._joints.position))
            if not all(math.isfinite(v) for v in (x, y, z, roll, pitch, yaw, *joints.values())) or max(abs(roll), abs(pitch)) > .8:
                self.halt()
                self._state, self._error = "error", "Vehicle pose is invalid or the vehicle tipped; stopped the route"
                return
            if self._manual is not None:
                forward, angular, lateral = self._manual
                side = lateral + .27114*angular
                vx = math.cos(yaw)*forward-math.sin(yaw)*side
                vy = math.sin(yaw)*forward+math.cos(yaw)*side
                self._state, self._reference = "manual", None
            else:
                rx, ry, vx, vy, _ = self._plan.sample(self._clock, (x, y), math.hypot(*self._velocity[:2]))
                self._state, self._error = self._plan.state, self._plan.error
                self._reference = (rx, ry)
                if self._state == "error":
                    self._command([0.]*5)
                    self._plan = None
                    return
                prediction = max(0., min(.04, age))
                vx += 8.*(rx-x-self._velocity[0]*prediction)
                vy += 8.*(ry-y-self._velocity[1]*prediction)
            speed = math.hypot(vx, vy)
            if speed > self.limits.max_speed:
                vx, vy = vx*self.limits.max_speed/speed, vy*self.limits.max_speed/speed
            left, right, _ = inverse_drive(vx, vy, yaw)
            scale = min(1., self.limits.max_wheel_rate/max(abs(left), abs(right), 1e-9))
            acceleration = self.limits.max_wheel_acceleration*min(dt, .1)
            wheels = tuple(max(old-acceleration, min(old+acceleration, value*scale))
                           for old, value in zip(self._wheels, (left, right)))
            self._wheels = wheels
            omega = .1075*(wheels[1]-wheels[0])/(2*.33072)
            turret = wrap(yaw+joints.get("pitch_yaw_plate_joint", 0.))
            gimbal = [-4.*(roll+joints.get("base_roll_joint", 0.)),
                      -4.*(pitch+joints.get("roll_pitch_joint", 0.)), -omega-4.*turret]
            self._command([*wheels, *(max(-2.5, min(2.5, v)) for v in gimbal)])

    def close(self):
        self.halt()
        self.node.destroy_timer(self._timer)
        for subscription in self._subscriptions:
            self.node.destroy_subscription(subscription)
        for publisher in self._publishers:
            self.node.destroy_publisher(publisher)
