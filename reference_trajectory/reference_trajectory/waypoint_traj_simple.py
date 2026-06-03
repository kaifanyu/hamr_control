#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from hamr_interfaces.msg import StateError
from hamr_interfaces.msg import ReferenceTraj
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import math
import numpy as np

ALIGN_BASE_REFERENCE_ROLL = 1.0

### PROBLEM:
## This current configuration does not work well most-probably bc the points are way too close
    # to the robot. the robot had way better control when the points were a meter or 2 away.
## TODO:
    # Implement server call when new waypoint -> reset I terms
    # SPLINES or more sophisticated trajectory generation
    # More spread out waypoints in turns (based on curvature) > Display waypoints and traj on rviz using marker or smth

def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__("waypoint_traj_simple_node")
        self.v_lin = self.declare_parameter("v_lin", 0.05).value
        self.w_yaw = self.declare_parameter("w_yaw", 0.5).value
        self.odom_topic = self.declare_parameter("odom_topic", "/HAMR_base/odom").value
        self.turret_odom_topic = self.declare_parameter("turret_odom_topic", "HAMR_turret/odom").value
        self.world_frame = self.declare_parameter("world_frame", "odom").value
        self.rotate_waypoints_with_initial_yaw = self.declare_parameter(
            "rotate_waypoints_with_initial_yaw", True
        ).value
        # Use turret's actual initial world-frame heading as the desired turret yaw
        # for all waypoints. When False falls back to base initial yaw (old behaviour).
        self.use_turret_initial_yaw = self.declare_parameter("use_turret_initial_yaw", True).value
        self.align_before_path = self.declare_parameter("align_before_path", False).value
        self.alignment_target_yaw_source = self.declare_parameter(
            "alignment_target_yaw_source", "turret"
        ).value
        self.alignment_target_yaw = self.declare_parameter("alignment_target_yaw", 0.0).value
        self.alignment_yaw_tolerance = self.declare_parameter(
            "alignment_yaw_tolerance", 0.08
        ).value
        self.alignment_settle_time = self.declare_parameter(
            "alignment_settle_time", 0.5
        ).value

        self.reference_timer_hz = self.declare_parameter("reference_timer_hz", 100).value

        self.state_error_sub_ = self.create_subscription(
            StateError, "/state_error", self.callback_state_error, 1)
        self.odom_sub_ = self.create_subscription(
            Odometry, self.odom_topic, self.callback_odom, 1)
        self.turret_odom_sub_ = self.create_subscription(
            Odometry, self.turret_odom_topic, self.callback_turret_odom, 1)
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

        self.paused = self.declare_parameter("paused", True).value

        self.begun = False
        self.last_reference_time = None
        self.current_odom = None
        self.current_turret_odom = None
        self.trajectory = None
        self.waiting_for_odom_logged = False
        self.waiting_for_turret_logged = False
        self.turret_wait_deadline = None  # set when base odom arrives, give turret 3s

        self.reference_timer_ = self.create_timer(
            1 / self.reference_timer_hz, self.reference_update)
        
        self.err_xy = math.inf
        self.err_yaw = math.inf
        self.alignment_complete = not self.align_before_path
        self.alignment_target_yaw_snapshot = None
        self.alignment_settled_since = None
        self.alignment_waiting_logged = False
        self.waiting_for_unpause_logged = False

        # --- STRAIGHT-LINE TEST (active) ---
        # [dx, dy, d_yaw] offsets from initial pose in world frame
        # (rotated to initial heading when rotate_waypoints_with_initial_yaw=True).
        # d_yaw=0 means "hold turret at initial heading" throughout.
        # Desired heading is taken from the turret's actual initial world-frame yaw
        # (see build_waypoints_from_current_odom) so the turret never needs to
        # correct from startup — only maintain its orientation via the Jacobian.
        self.local_waypoints = np.array([
            [0.0, 0.0, 0.0],   # Start at current pose
            [2.0, 0.0, 0.0],   # 2 m straight in local +x — forward-drive debug
        ])

        # --- MAZE / FULL TRAJECTORY (uncomment to restore) ---
        # self.local_waypoints = np.array([
        #     [0.0, 0.0, 0.0],
        #     [0.0, 3.0, 0.0],
        #     [1.5, 3.0, 0.0],
        #     [1.5, 5.0, 0.0],
        #     [-1.5, 5.0, 0.0],
        #     [-1.5, 3.0, 0.0],
        #     [0.0, 3.0, 0.0],
        #     [0.0, 0.0, 0.0],
        # ])
        self.add_post_set_parameters_callback(self.parameters_callback)
    
    def callback_odom(self, msg: Odometry):
        self.current_odom = msg

    def callback_turret_odom(self, msg: Odometry):
        self.current_turret_odom = msg

    def callback_state_error(self, msg: StateError):
        self.err_xy = math.hypot(msg.err_x, msg.err_y)
        self.err_yaw = msg.err_yaw

    def current_base_yaw(self):
        return float(yaw_from_quaternion(self.current_odom.pose.pose.orientation))

    def current_turret_yaw(self):
        return float(yaw_from_quaternion(self.current_turret_odom.pose.pose.orientation))

    def snapshot_alignment_target_yaw(self):
        source = str(self.alignment_target_yaw_source).lower()
        if source == "base":
            yaw = self.current_base_yaw()
        elif source == "turret":
            yaw = self.current_turret_yaw()
        elif source == "param":
            yaw = float(self.alignment_target_yaw)
        else:
            self.get_logger().warn(
                "Unknown alignment_target_yaw_source='%s'; using turret yaw." %
                self.alignment_target_yaw_source)
            yaw = self.current_turret_yaw()

        yaw = wrap_angle(yaw)
        self.get_logger().info(
            "Alignment target yaw from %s: %.3f rad" %
            (source, yaw))
        return yaw

    def publish_alignment_reference(self):
        if self.alignment_target_yaw_snapshot is None:
            self.alignment_target_yaw_snapshot = self.snapshot_alignment_target_yaw()

        pose = self.current_odom.pose.pose
        ref = ReferenceTraj()
        ref.x = float(pose.position.x)
        ref.y = float(pose.position.y)
        ref.roll = ALIGN_BASE_REFERENCE_ROLL
        ref.pitch = 0.0
        ref.yaw = float(self.alignment_target_yaw_snapshot)
        ref.x_dot = 0.0
        ref.y_dot = 0.0
        ref.roll_dot = 0.0
        ref.pitch_dot = 0.0
        ref.yaw_dot = 0.0
        self.reference_trajectory_pub_.publish(ref)

    def update_alignment(self):
        if self.current_odom is None or self.current_turret_odom is None:
            if not self.alignment_waiting_logged:
                self.get_logger().info(
                    "Waiting for %s and %s before yaw alignment ..." %
                    (self.odom_topic, self.turret_odom_topic))
                self.alignment_waiting_logged = True
            return

        self.publish_alignment_reference()

        target = self.alignment_target_yaw_snapshot
        base_err = wrap_angle(target - self.current_base_yaw())
        turret_err = wrap_angle(target - self.current_turret_yaw())
        aligned = (
            abs(base_err) <= self.alignment_yaw_tolerance and
            abs(turret_err) <= self.alignment_yaw_tolerance
        )

        now = self.get_clock().now()
        if aligned:
            if self.alignment_settled_since is None:
                self.alignment_settled_since = now
                return
            settled_for = (now - self.alignment_settled_since).nanoseconds * 1e-9
            if settled_for >= self.alignment_settle_time:
                self.alignment_complete = True
                self.paused = True
                self.begun = False
                self.trajectory = None
                self.get_logger().info(
                    "Yaw alignment complete; paused before path. "
                    "Set paused:=false to snapshot the aligned pose and start the path.")
                return
        else:
            self.alignment_settled_since = None

        self.get_logger().info(
            "Aligning yaws: base_err=%.3f rad, turret_err=%.3f rad" %
            (base_err, turret_err),
            throttle_duration_sec=1.0)
    
    def build_waypoints_from_current_odom(self):
        pose = self.current_odom.pose.pose
        x0 = float(pose.position.x)
        y0 = float(pose.position.y)
        yaw_base0 = float(yaw_from_quaternion(pose.orientation))

        # Use the turret's actual world-frame heading as desired yaw so that
        # err_yaw=0 at startup and the turret never needs to actively rotate —
        # it only counter-rotates through the Jacobian to hold its heading.
        # Fall back to base yaw if turret odom hasn't arrived yet.
        if self.use_turret_initial_yaw and self.current_turret_odom is not None:
            yaw_heading0 = float(yaw_from_quaternion(
                self.current_turret_odom.pose.pose.orientation))
            self.get_logger().info(
                "Using turret initial yaw=%.3f rad (base yaw=%.3f rad)" %
                (yaw_heading0, yaw_base0))
        else:
            yaw_heading0 = yaw_base0
            if self.use_turret_initial_yaw:
                self.get_logger().warn(
                    "Turret odom not yet received; falling back to base yaw=%.3f rad" %
                    yaw_base0)

        c = math.cos(yaw_base0)
        s = math.sin(yaw_base0)
        waypoints = []
        for dx, dy, dyaw in self.local_waypoints:
            if self.rotate_waypoints_with_initial_yaw:
                x = x0 + c * dx - s * dy
                y = y0 + s * dx + c * dy
            else:
                x = x0 + dx
                y = y0 + dy
            # dyaw is applied as an offset from the turret's initial heading
            waypoints.append([x, y, wrap_angle(yaw_heading0 + dyaw)])

        return np.array(waypoints)

    def initialize_trajectory(self):
        waypoints = self.build_waypoints_from_current_odom()
        self.trajectory = WaypointTraj(waypoints, v_lin=self.v_lin, w_yaw=self.w_yaw)
        self.last_reference_time = self.get_clock().now()
        self.begun = True

        start = waypoints[0]
        end = waypoints[-1]
        self.get_logger().info(
            "Beginning trajectory from current %s pose: "
            "start=(%.3f, %.3f, %.3f), end=(%.3f, %.3f, %.3f)"
            % (
                self.odom_topic,
                start[0], start[1], start[2],
                end[0], end[1], end[2],
            )
        )

        self.publish_waypoints_path()

    def publish_waypoints_path(self):
        path_msg = Path()
        path_msg.header.frame_id = self.world_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for pt in self.trajectory.points:
            x, y, yaw = float(pt[0]), float(pt[1]), float(pt[2])

            ps = PoseStamped()
            ps.header.frame_id = self.world_frame
            ps.header.stamp = path_msg.header.stamp
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0

            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = math.sin(yaw * 0.5)
            ps.pose.orientation.w = math.cos(yaw * 0.5)

            path_msg.poses.append(ps)

        self.waypoints_path_pub_.publish(path_msg)

    def reference_update(self):
        if not self.alignment_complete:
            self.update_alignment()
            return

        if self.align_before_path and self.paused and not self.begun:
            if not self.waiting_for_unpause_logged:
                self.get_logger().info(
                    "Yaw alignment is done; waiting for paused:=false before building the path.")
                self.waiting_for_unpause_logged = True
            return

        # Build trajectory as soon as odom (and optionally turret odom) arrives
        if not self.begun:
            if self.current_odom is None:
                if not self.waiting_for_odom_logged:
                    self.get_logger().info("Waiting for %s ..." % self.odom_topic)
                    self.waiting_for_odom_logged = True
                return
            if self.use_turret_initial_yaw and self.current_turret_odom is None:
                # Start a 3-second deadline the first time base odom is seen
                if self.turret_wait_deadline is None:
                    self.turret_wait_deadline = self.get_clock().now()
                elapsed = (self.get_clock().now() - self.turret_wait_deadline).nanoseconds * 1e-9
                if elapsed < 3.0:
                    if not self.waiting_for_turret_logged:
                        self.get_logger().info(
                            "Waiting up to 3s for %s ..." % self.turret_odom_topic)
                        self.waiting_for_turret_logged = True
                    return
                self.get_logger().warn(
                    "Turret odom never arrived — falling back to base yaw for heading.")
                self.use_turret_initial_yaw = False  # fall through to base-yaw init
            self.initialize_trajectory()

        if self.paused:
            return

        now = self.get_clock().now()
        t = (now - self.last_reference_time).nanoseconds * 1e-9
        x, y, yaw, x_dot, y_dot, yaw_dot = self.trajectory.update(t)

        pose = ReferenceTraj()
        pose.x, pose.y, pose.yaw, pose.x_dot, pose.y_dot, pose.yaw_dot = float(x), float(y), float(yaw), float(x_dot), float(y_dot), float(yaw_dot)
        self.reference_trajectory_pub_.publish(pose)
        self.get_logger().info("pose: x=%.2f, y=%.2f, yaw=%.2f" % (x, y, yaw))
        if t >= self.trajectory.total_time:
            self.get_logger().info("Trajectory complete — pausing.")
            self.paused = True
            self.begun = False
            self.trajectory = None

    # Used if we want to change parameter during runtime
    def parameters_callback(self, params: list[Parameter]):
        for p in params:
            if p.name == "paused":
                was_paused = self.paused
                self.paused = p.value
                if was_paused and not self.paused:
                    # Re-snapshot current position so trajectory starts from here, not launch-time position
                    self.begun = False
                    self.trajectory = None
                    self.waiting_for_unpause_logged = False
                    self.get_logger().info("Unpaused — re-initializing trajectory from current pose.")
            elif p.name == "v_lin":
                self.v_lin = p.value
                if self.trajectory is not None:
                    self.trajectory.v_lin = p.value
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name == "w_yaw":
                self.w_yaw = p.value
                if self.trajectory is not None:
                    self.trajectory.w_yaw = p.value
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name == "rotate_waypoints_with_initial_yaw":
                self.rotate_waypoints_with_initial_yaw = p.value
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name == "use_turret_initial_yaw":
                self.use_turret_initial_yaw = p.value
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name == "align_before_path":
                self.align_before_path = p.value
                self.alignment_complete = not self.align_before_path
                self.alignment_target_yaw_snapshot = None
                self.alignment_settled_since = None
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name == "alignment_target_yaw_source":
                self.alignment_target_yaw_source = p.value
                self.alignment_target_yaw_snapshot = None
                self.alignment_settled_since = None
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name == "alignment_target_yaw":
                self.alignment_target_yaw = p.value
                if str(self.alignment_target_yaw_source).lower() == "param":
                    self.alignment_target_yaw_snapshot = None
                    self.alignment_settled_since = None
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name == "alignment_yaw_tolerance":
                self.alignment_yaw_tolerance = p.value
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name == "alignment_settle_time":
                self.alignment_settle_time = p.value
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