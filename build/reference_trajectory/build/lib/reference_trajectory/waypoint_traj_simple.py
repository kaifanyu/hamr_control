#!/usr/bin/env python3
"""Publish a timed, optionally one-shot waypoint reference trajectory."""

import math
from typing import NamedTuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from hamr_interfaces.msg import StateError
from hamr_interfaces.msg import ReferenceTraj
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


MAX_HOLD_SECONDS = 3600.0
MAX_SUBSCRIBER_STABLE_SECONDS = 60.0
MAX_REQUIRED_REFERENCE_SUBSCRIBERS = 64
MIN_REFERENCE_TIMER_HZ = 1.0
MAX_REFERENCE_TIMER_HZ = 1000.0

PHASE_STARTUP_HOLD = "startup_hold"
PHASE_MOTION = "motion"
PHASE_FINAL_HOLD = "final_hold"
PHASE_DONE = "done"


def validated_bounded_float(name, value, minimum, maximum):
    """Return a finite float in the inclusive requested range."""
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number, not a boolean")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    if result < minimum or result > maximum:
        raise ValueError(
            f"{name} must be in [{minimum}, {maximum}], got {result}"
        )
    return result


def validated_positive_float(name, value):
    """Return a finite, strictly positive float."""
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number, not a boolean")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")
    return result


def validated_required_subscribers(value):
    """Return a bounded nonnegative integer subscriber requirement."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("required_reference_subscribers must be an integer")
    if value < 0 or value > MAX_REQUIRED_REFERENCE_SUBSCRIBERS:
        raise ValueError(
            "required_reference_subscribers must be in "
            f"[0, {MAX_REQUIRED_REFERENCE_SUBSCRIBERS}]"
        )
    return value


def validated_loop(value):
    """Require the loop parameter to be an actual ROS boolean."""
    if not isinstance(value, bool):
        raise ValueError("loop must be a boolean")
    return value


class PlaybackStep(NamedTuple):
    """One deterministic trajectory-playback decision."""

    phase: str
    trajectory_time_s: float
    publish: bool
    cycle_index: int


class SubscriberReadinessGate:
    """Latch ready after enough subscribers remain present for a duration."""

    def __init__(self, required_subscribers, stable_s):
        self.required_subscribers = validated_required_subscribers(
            required_subscribers
        )
        self.stable_s = validated_bounded_float(
            "subscriber_stable_s",
            stable_s,
            0.0,
            MAX_SUBSCRIBER_STABLE_SECONDS,
        )
        self.stable_since_s = None
        self.last_observation_s = None
        self.ready = False

    def observe(self, now_s, subscriber_count):
        """Update the readiness state from a local monotonic observation."""
        now_s = validated_bounded_float(
            "subscriber observation time", now_s, 0.0, float(2**63)
        )
        subscriber_count = validated_required_subscribers(subscriber_count)

        if self.ready:
            return True

        if (
            self.last_observation_s is not None
            and now_s < self.last_observation_s
        ):
            self.stable_since_s = None
        self.last_observation_s = now_s

        if subscriber_count < self.required_subscribers:
            self.stable_since_s = None
            return False
        if self.required_subscribers == 0 or self.stable_s == 0.0:
            self.ready = True
            return True
        if self.stable_since_s is None:
            self.stable_since_s = now_s
            return False
        if now_s - self.stable_since_s >= self.stable_s:
            self.ready = True
        return self.ready


class TrajectoryPlayback:
    """State machine for startup hold, motion, final hold, and looping."""

    def __init__(
        self,
        trajectory_duration_s,
        startup_hold_s=0.0,
        final_hold_s=0.0,
        loop=True,
    ):
        self.trajectory_duration_s = validated_positive_float(
            "trajectory_duration_s", trajectory_duration_s
        )
        self.startup_hold_s = validated_bounded_float(
            "startup_hold_s", startup_hold_s, 0.0, MAX_HOLD_SECONDS
        )
        self.final_hold_s = validated_bounded_float(
            "final_hold_s", final_hold_s, 0.0, MAX_HOLD_SECONDS
        )
        self.loop = validated_loop(loop)
        self.started_at_s = None
        self.final_hold_started_at_s = None
        self.last_step_s = None
        self.completed = False
        self.cycle_index = 0

    def _cycle_start_step(self):
        if self.startup_hold_s > 0.0:
            return PlaybackStep(
                PHASE_STARTUP_HOLD, 0.0, True, self.cycle_index
            )
        return PlaybackStep(PHASE_MOTION, 0.0, True, self.cycle_index)

    def step(self, now_s):
        """Return what should be published at ``now_s``."""
        now_s = validated_bounded_float(
            "playback time", now_s, 0.0, float(2**63)
        )
        if self.last_step_s is not None and now_s < self.last_step_s:
            raise ValueError("playback time must not move backwards")
        self.last_step_s = now_s

        if self.completed:
            return PlaybackStep(
                PHASE_DONE,
                self.trajectory_duration_s,
                False,
                self.cycle_index,
            )

        # The trajectory epoch is deliberately set by the first publishing
        # callback, never by node construction or subscriber discovery.
        if self.started_at_s is None:
            self.started_at_s = now_s
            return self._cycle_start_step()

        elapsed_s = now_s - self.started_at_s
        if elapsed_s < self.startup_hold_s:
            return PlaybackStep(
                PHASE_STARTUP_HOLD, 0.0, True, self.cycle_index
            )

        trajectory_time_s = elapsed_s - self.startup_hold_s
        if trajectory_time_s < self.trajectory_duration_s:
            return PlaybackStep(
                PHASE_MOTION,
                trajectory_time_s,
                True,
                self.cycle_index,
            )

        # Entering this state always emits the endpoint once. The final-hold
        # clock begins with that publication, so a delayed timer callback can
        # never skip the requested zero-velocity dwell.
        if self.final_hold_started_at_s is None:
            self.final_hold_started_at_s = now_s
            return PlaybackStep(
                PHASE_FINAL_HOLD,
                self.trajectory_duration_s,
                True,
                self.cycle_index,
            )

        if now_s - self.final_hold_started_at_s < self.final_hold_s:
            return PlaybackStep(
                PHASE_FINAL_HOLD,
                self.trajectory_duration_s,
                True,
                self.cycle_index,
            )

        if not self.loop:
            self.completed = True
            return PlaybackStep(
                PHASE_DONE,
                self.trajectory_duration_s,
                False,
                self.cycle_index,
            )

        # Restart on a later callback only, after an endpoint publication and
        # the complete final hold. Never replace the endpoint in the callback
        # that first detects trajectory completion.
        self.cycle_index += 1
        self.started_at_s = now_s
        self.final_hold_started_at_s = None
        return self._cycle_start_step()


def scheduled_reference(trajectory, playback_step):
    """Return a trajectory sample with zero velocity in either hold."""
    if playback_step.phase == PHASE_STARTUP_HOLD:
        x, y, yaw = trajectory.points[0]
        return float(x), float(y), float(yaw), 0.0, 0.0, 0.0
    if playback_step.phase == PHASE_FINAL_HOLD:
        x, y, yaw = trajectory.points[-1]
        return float(x), float(y), float(yaw), 0.0, 0.0, 0.0
    if playback_step.phase == PHASE_MOTION:
        return trajectory.update(playback_step.trajectory_time_s)
    raise ValueError("a completed playback step has no reference to publish")

### PROBLEM:
## This current configuration does not work well most-probably bc the points are way too close
    # to the robot. the robot had way better control when the points were a meter or 2 away.
## TODO:
    # Implement server call when new waypoint -> reset I terms
    # SPLINES or more sophisticated trajectory generation
    # More spread out waypoints in turns (based on curvature) > Display waypoints and traj on rviz using marker or smth

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__("waypoint_traj_simple_node")
        v_lin = validated_positive_float(
            "v_lin", self.declare_parameter("v_lin", 0.25).value
        )
        w_yaw = validated_positive_float(
            "w_yaw", self.declare_parameter("w_yaw", 0.5).value
        )
        self.reference_timer_hz = validated_bounded_float(
            "reference_timer_hz",
            self.declare_parameter("reference_timer_hz", 100.0).value,
            MIN_REFERENCE_TIMER_HZ,
            MAX_REFERENCE_TIMER_HZ,
        )
        self.startup_hold_s = validated_bounded_float(
            "startup_hold_s",
            self.declare_parameter("startup_hold_s", 0.0).value,
            0.0,
            MAX_HOLD_SECONDS,
        )
        self.final_hold_s = validated_bounded_float(
            "final_hold_s",
            self.declare_parameter("final_hold_s", 0.0).value,
            0.0,
            MAX_HOLD_SECONDS,
        )
        self.loop = validated_loop(
            self.declare_parameter("loop", True).value
        )
        self.required_reference_subscribers = validated_required_subscribers(
            self.declare_parameter(
                "required_reference_subscribers", 0
            ).value
        )
        self.subscriber_stable_s = validated_bounded_float(
            "subscriber_stable_s",
            self.declare_parameter("subscriber_stable_s", 0.5).value,
            0.0,
            MAX_SUBSCRIBER_STABLE_SECONDS,
        )

        self.state_error_sub_ = self.create_subscription(
            StateError, "/state_error", self.callback_state_error, 1)
        self.reference_trajectory_pub_ = self.create_publisher(
            ReferenceTraj, "/reference_trajectory", 1
        )

        qos_waypoints = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.waypoints_path_pub_ = self.create_publisher(
            Path, "/waypoints_path", qos_profile=qos_waypoints
        )
        self.marker_pub_ = self.create_publisher(MarkerArray, "/traj_viz", 10)

        self.begun = False
        self.last_reference_time = None
        self.last_playback_phase = None
        self.last_playback_cycle = 0
        self.last_subscriber_wait_log_ns = None
        self.completion_logged = False

        self.err_xy = math.inf
        self.err_yaw = math.inf

        max_point = 5.0
        origin = 0.0

        def generate_ccw_circle_points(radius=5.0, steps_between=10):
            cx = 0.0
            cy = 0.0 + radius

            # Angles for waypoints (rad)
            # waypoints = [-np.pi/2, -np.pi, -3*np.pi/2, -2*np.pi, -5*np.pi/2] # CW
            waypoints = [-5*np.pi/2, -2*np.pi, -3*np.pi/2, -np.pi, -np.pi/2] # CCW
            pts = []

            # First point explicitly at (0,0,0)
            pts.append([cx, cy - radius, 0.0])

            # Generate ccw points
            for i in range(len(waypoints) - 1):
                th_start = waypoints[i]
                th_end   = waypoints[i + 1]

                # steps_between points between waypoints
                thetas = np.linspace(th_start, th_end, steps_between + 1, endpoint=False)[1:] if i == 0 else \
                        np.linspace(th_start, th_end, steps_between + 1, endpoint=False)

                for th in thetas:
                    x = cx + radius * np.cos(th)
                    y = cy + radius * np.sin(th)
                    pts.append([float(x), float(y), 0.0])

            # Close the loop back to start
            pts.append([0.0, 0.0, 0.0])

            return np.array(pts)

        # Straight hardware test path: move along +Y in the odom/mocap frame,
        # then return to the starting point.
        # waypoints = np.array([ # x, y, yaw
        #     [0.0, 0.0, 0.0],
        #     [0.0, 3.0, 0.0],
        #     [0.0, 0.0, 0.0],
        # ])

        # waypoints = generate_ccw_circle_points()

        waypoints = np.array([ # x, y, yaw
            [0.0, 0.0, 0.0],
            [0.0, 2.0, 0.0],
            [-2.0, 2.0, 0.0],
            [-2.0, 4.5, 0.0],
            [0.0, 4.5, 0.0],
            [0.0, 2.0, 0.0],
            [0.0, 0.0, 0.0],
        ])

        #     # [0.0, 0.0, 0.0], # SQUARE
        #     # [5.75, 0.0, 0.0],
        #     # [5.75, 5.75, 0.0],
        #     # [0.0, 5.75, 0.0],
        #     # [0.0, 0.0, 0.0],


        #     [-1.0, 3.0, 0.0], # HW SQUARE
        #     [-1.0, 5.0, 0.0],
        #     [1.0, 5.0, 0.0],
        #     [1.0, 3.0, 0.0],
        #     [-1.0, 3.0, 0.0],

        #     # [origin,    origin,    0.0], # SQUARE
        #     # [max_point, origin,    0.0],
        #     # [max_point, max_point, 0.0],
        #     # [origin,    max_point, 0.0],
        #     # [origin,    origin,    0.0],
            
        #     # # Back and Forth
        #     # [0.0, 0.0, 0.0],
        #     # [3.0, 0.0, 0.0],
        #     # [1.0, 1.0, 0.0],
        #     # [1.0, 0.0, 0.0],
        #     # [0.0, 0.0, 0.0],

        #     # [0.0, 0.0, 0.0], # TRIANGLE
        #     # [9.0, 4.5, 0.0],
        #     # [0.0, 9.0, 0.0],
        #     # [0.0, 0.0, 0.0],
        # ])
        
        self.trajectory = WaypointTraj(waypoints, v_lin=v_lin, w_yaw=w_yaw)
        self.subscriber_gate = SubscriberReadinessGate(
            self.required_reference_subscribers,
            self.subscriber_stable_s,
        )
        self.playback = TrajectoryPlayback(
            self.trajectory.total_time,
            startup_hold_s=self.startup_hold_s,
            final_hold_s=self.final_hold_s,
            loop=self.loop,
        )

        # This timer may poll subscriber readiness, but the trajectory clock is
        # not anchored and no reference is published until the gate is ready.
        # Construct it last so every object used by its callback is initialized.
        self.reference_timer_ = self.create_timer(
            1.0 / self.reference_timer_hz, self.reference_update
        )
        self.get_logger().info(
            "Waypoint schedule: startup hold %.3fs, final hold %.3fs, "
            "loop=%s, required reference subscribers=%d stable for %.3fs"
            % (
                self.startup_hold_s,
                self.final_hold_s,
                self.loop,
                self.required_reference_subscribers,
                self.subscriber_stable_s,
            )
        )
    
    def callback_state_error(self, msg: StateError):
        self.err_xy = math.hypot(msg.err_x, msg.err_y)
        self.err_yaw = msg.err_yaw
    
    def reference_update(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9

        if not self.begun:
            subscriber_count = (
                self.reference_trajectory_pub_.get_subscription_count()
            )
            if not self.subscriber_gate.observe(now_s, subscriber_count):
                if (
                    self.last_subscriber_wait_log_ns is None
                    or now.nanoseconds - self.last_subscriber_wait_log_ns
                    >= 2_000_000_000
                ):
                    self.get_logger().info(
                        "Waiting for reference subscribers: %d/%d; "
                        "required count must remain stable for %.3fs"
                        % (
                            subscriber_count,
                            self.required_reference_subscribers,
                            self.subscriber_stable_s,
                        )
                    )
                    self.last_subscriber_wait_log_ns = now.nanoseconds
                return

            self.begun = True
            self.get_logger().info(
                "Reference subscribers ready (%d); beginning trajectory "
                "startup hold."
                % subscriber_count
            )
            self.last_reference_time = now

            # Publish waypoints as Path for visualization
            path_msg = Path()
            path_msg.header.frame_id = "odom"
            path_msg.header.stamp = now.to_msg()

            for pt in self.trajectory.points:
                x, y, yaw = float(pt[0]), float(pt[1]), float(pt[2])

                ps = PoseStamped()
                ps.header.frame_id = "odom"
                ps.header.stamp = path_msg.header.stamp  # keep a consistent stamp
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.position.z = 0.0

                ps.pose.orientation.x = 0.0
                ps.pose.orientation.y = 0.0
                ps.pose.orientation.z = math.sin(yaw * 0.5)
                ps.pose.orientation.w = math.cos(yaw * 0.5)

                path_msg.poses.append(ps)
            self.waypoints_path_pub_.publish(path_msg)
            self._publish_waypoint_markers()

        try:
            playback_step = self.playback.step(now_s)
        except ValueError as exc:
            self.get_logger().error(
                f"Invalid trajectory clock; stopping publication: {exc}"
            )
            self.reference_timer_.cancel()
            return

        if not playback_step.publish:
            if not self.completion_logged:
                self.get_logger().info(
                    "One-shot trajectory and final hold complete; "
                    "reference publication stopped."
                )
                self.completion_logged = True
            self.reference_timer_.cancel()
            return

        if playback_step.cycle_index != self.last_playback_cycle:
            self.get_logger().info(
                "Final hold complete; beginning trajectory cycle %d."
                % (playback_step.cycle_index + 1)
            )
            self.last_playback_cycle = playback_step.cycle_index

        if playback_step.phase != self.last_playback_phase:
            if playback_step.phase == PHASE_STARTUP_HOLD:
                self.get_logger().info(
                    "Holding the initial zero-velocity reference for %.3fs."
                    % self.startup_hold_s
                )
            elif playback_step.phase == PHASE_MOTION:
                self.get_logger().info("Beginning waypoint motion.")
            elif playback_step.phase == PHASE_FINAL_HOLD:
                self.get_logger().info(
                    "Trajectory endpoint reached; holding the final "
                    "zero-velocity reference for %.3fs."
                    % self.final_hold_s
                )
            self.last_playback_phase = playback_step.phase

        x, y, yaw, x_dot, y_dot, yaw_dot = scheduled_reference(
            self.trajectory, playback_step
        )

        pose = ReferenceTraj()
        pose.x = float(x)
        pose.y = float(y)
        pose.yaw = float(yaw)
        pose.x_dot = float(x_dot)
        pose.y_dot = float(y_dot)
        pose.yaw_dot = float(yaw_dot)
        self.reference_trajectory_pub_.publish(pose)
        self.get_logger().debug(
            "pose: x=%.2f, y=%.2f, yaw=%.2f" % (x, y, yaw)
        )
        self._publish_reference_marker(x, y, yaw)

    def _publish_reference_marker(self, x, y, yaw):
        """Arrow showing the live reference pose moving along the trajectory."""
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "reference_pose"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.0
        m.pose.orientation.z = math.sin(yaw * 0.5)
        m.pose.orientation.w = math.cos(yaw * 0.5)
        m.scale.x = 0.35  # arrow length
        m.scale.y = 0.07  # arrow width
        m.scale.z = 0.07
        m.color.r = 1.0
        m.color.g = 0.4
        m.color.b = 0.0
        m.color.a = 1.0
        arr = MarkerArray()
        arr.markers.append(m)
        self.marker_pub_.publish(arr)

    def _publish_waypoint_markers(self):
        """Spheres at each discrete waypoint, published once at startup."""
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, pt in enumerate(self.trajectory.points):
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = stamp
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(pt[0])
            m.pose.position.y = float(pt[1])
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.r = 0.0
            m.color.g = 0.8
            m.color.b = 1.0
            m.color.a = 1.0
            arr.markers.append(m)
        self.marker_pub_.publish(arr)

    # Used if we want to change parameter during runtime
    def parameters_callback(self, params: list[Parameter]): 
        for p in params:
            if p.name == "v_lin":
                self.trajectory.v_lin = p.value
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name == "w_yaw":
                self.trajectory.w_yaw = p.value
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name == "reference_timer_hz":
                self.reference_timer_hz = p.value
                self.reference_timer_.cancel()
                self.reference_timer_ = self.create_timer(
                    1 / self.reference_timer_hz, self.reference_update)
                self.get_logger().info(f"{p.name} changed to {p.value}")
            
class WaypointTraj(object):
    def __init__(self, points, v_lin=0.6, w_yaw=0.3):
        """
        Inputs: points, (N, 3) array of N waypoint coordinates in 2D with yaw
        """
        points = np.array(points, dtype=float)

        # Keep points properly shaped
        if points.ndim == 1:
            if points.size % 3 != 0:
                raise ValueError("points.size % 3 != 0")
            points = points.reshape(-1, 3)
        elif points.ndim == 3 and points.shape[1] != 3:
            if points.shape[0] == 3:
                points = points.T
            else:
                raise ValueError("points must be (N, 3) or (3, N)")

        self.points = points
        self.v_lin = float(v_lin)
        self.w_yaw = float(w_yaw)
        self.N = len(points)

        def wrap_angle(a):
            return np.arctan2(np.sin(a), np.cos(a))

        d = np.diff(self.points, axis=0) # (N-1, 3)
        d_xy = d[:, :2] # (N-1,2)
        d_yaw = wrap_angle(d[:, 2]) # (N-1,)

        # Durations with separate linear/yaw limits
        eps = 1e-9
        d_xy_norm = np.linalg.norm(d_xy, axis=1) # (N-1,)
        T_lin = d_xy_norm / max(self.v_lin, eps)
        T_yaw = np.abs(d_yaw) / max(self.w_yaw, eps)
        T = np.maximum(T_lin, T_yaw)
        T[T < eps] = eps # avoid zero-length segments

        # Precompute per-segment constant velocities
        self.v_xy = (d_xy / T[:, None]) # (N-1, 2)
        self.w = (d_yaw / T) # (N-1,)

        # Timing
        self.t_start = np.hstack(([0.0], np.cumsum(T))) # (N,)
        self.total_time = float(self.t_start[-1])

        self.last_seg = 0
        

    def update(self, t: float):
        """
        Given the present time, return the desired flat output
        Inputs
            t, time, s
        Outputs
            q, position
            yaw, turret
        """
        def wrap_angle(a):
            return np.arctan2(np.sin(a), np.cos(a))
    
        if t >= self.total_time:
            x_last, y_last, yaw_last = self.points[-1]
            return float(x_last), float(y_last), float(yaw_last), 0.0, 0.0, 0.0

        seg = int(np.searchsorted(self.t_start, t, side='right') - 1)
        if seg > self.last_seg:
            self.last_seg = seg
        dt = t - self.t_start[seg]

        # Clamp dt inside segment just in case of numerical edge
        seg_end = self.t_start[seg + 1]
        if dt < 0.0: dt = 0.0
        if dt > (seg_end - self.t_start[seg]): dt = seg_end - self.t_start[seg]

        # Integrate with constant per-segment velocities
        x0, y0, yaw0 = self.points[seg]
        vx, vy = self.v_xy[seg]
        wyaw = self.w[seg]

        x = x0 + vx * dt
        y = y0 + vy * dt
        yaw = wrap_angle(yaw0 + wyaw * dt)

        #      x         y         yaw         x_dot      y_dot      yaw_dot
        return float(x), float(y), float(yaw), float(vx), float(vy), float(wyaw)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
