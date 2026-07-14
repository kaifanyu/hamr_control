#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from hamr_interfaces.msg import StateError
from hamr_interfaces.msg import ReferenceTraj
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import math
import numpy as np

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
        v_lin = self.declare_parameter("v_lin", 0.2).value
        w_yaw = self.declare_parameter("w_yaw", 0.5).value
        dwell_time = self.declare_parameter("dwell_time", 0.5).value

        self.reference_timer_hz = self.declare_parameter("reference_timer_hz", 100).value

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
        
        self.reference_timer_ = self.create_timer(
            1 / self.reference_timer_hz, self.reference_update)
        
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
            [0.0, 3.0, 0.0],
            [1.5, 3.0, 0.0],
            [1.5, 5.0, 0.0],
            [-1.5, 5.0, 0.0],
            [-1.5, 3.0, 0.0],
            [0.0, 3.0, 0.0],
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
        
        self.trajectory = WaypointTraj(waypoints, v_lin=v_lin, w_yaw=w_yaw,
                                       dwell_time=dwell_time)
    
    def callback_state_error(self, msg: StateError):
        self.err_xy = math.hypot(msg.err_x, msg.err_y)
        self.err_yaw = msg.err_yaw
    
    def reference_update(self):
        if not self.begun:
            self.begun = True
            self.get_logger().info("Beginning trajectory tracking.")
            self.last_reference_time = self.get_clock().now()

            # Publish waypoints as Path for visualization
            path_msg = Path()
            path_msg.header.frame_id = "odom"
            path_msg.header.stamp = self.get_clock().now().to_msg()

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

        now = self.get_clock().now()
        t = (now - self.last_reference_time).nanoseconds * 1e-9
        x, y, yaw, x_dot, y_dot, yaw_dot = self.trajectory.update(t)

        pose = ReferenceTraj()
        pose.x, pose.y, pose.yaw, pose.x_dot, pose.y_dot, pose.yaw_dot = float(x), float(y), float(yaw), float(x_dot), float(y_dot), float(yaw_dot)
        self.reference_trajectory_pub_.publish(pose)
        self.get_logger().info("pose: x=%.2f, y=%.2f, yaw=%.2f" % (x, y, yaw))
        self._publish_reference_marker(x, y, yaw)
        if t >= self.trajectory.total_time:
            self.get_logger().info("Resetting traj")
            self.last_reference_time = self.get_clock().now()

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
            
def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class WaypointTraj(object):
    def __init__(self, points, v_lin=0.6, w_yaw=0.3, dwell_time=0.5):
        """
        Inputs: points, (N, 3) array of N waypoint coordinates in 2D with yaw

        HOLONOMIC (base + turret): the controller independently tracks the base
        position (x, y) and the payload yaw. The robot comes to a brief STOP at
        each waypoint (dwell_time), then drives to the next with a smooth cosine
        velocity profile (velocity zero at both ends) so the command is always
        physically trackable. The base heading swings freely underneath; that
        does not affect the x-y path.
        """
        points = np.array(points, dtype=float)

        # Keep points properly shaped
        if points.ndim == 1:
            if points.size % 3 != 0:
                raise ValueError("points.size % 3 != 0")
            points = points.reshape(-1, 3)

        self.points = points
        self.v_lin = float(v_lin)
        self.w_yaw = float(w_yaw)
        self.dwell_time = float(dwell_time)
        self.N = len(points)

        eps = 1e-9

        # Build phases: dwell (stop) at each waypoint, then a smooth move.
        self.phases = []
        t = 0.0
        for i in range(self.N):
            # Dwell (brief stop) at waypoint i
            self.phases.append({
                "type": "dwell", "t0": t, "t1": t + self.dwell_time,
                "p": self.points[i],
            })
            t += self.dwell_time

            # Smooth move from waypoint i to i+1 (cosine ease-in-out)
            if i < self.N - 1:
                p0 = self.points[i]
                p1 = self.points[i + 1]
                d_xy = p1[:2] - p0[:2]
                d_yaw = _wrap(p1[2] - p0[2])

                T_lin = math.pi * np.linalg.norm(d_xy) / (2.0 * max(self.v_lin, eps))
                T_yaw = math.pi * abs(d_yaw) / (2.0 * max(self.w_yaw, eps))
                T = max(T_lin, T_yaw, eps)

                self.phases.append({
                    "type": "move", "t0": t, "t1": t + T,
                    "p0": p0, "d_xy": d_xy, "d_yaw": d_yaw,
                    "yaw0": p0[2], "T": T,
                })
                t += T

        self.total_time = float(t)

    def update(self, t: float):
        """
        Given the present time, return the desired flat output:
            x, y, yaw, x_dot, y_dot, yaw_dot
        """
        if t >= self.total_time:
            x, y, yaw = self.points[-1]
            return float(x), float(y), float(yaw), 0.0, 0.0, 0.0

        for ph in self.phases:
            if ph["t0"] <= t < ph["t1"]:
                if ph["type"] == "dwell":
                    x, y, yaw = ph["p"]
                    return float(x), float(y), float(yaw), 0.0, 0.0, 0.0

                dt = t - ph["t0"]
                T = ph["T"]
                tau = dt / T
                u = 0.5 * (1.0 - math.cos(math.pi * tau))       # 0 -> 1
                u_dot = 0.5 * math.pi / T * math.sin(math.pi * tau)  # 0 at ends

                x0, y0 = ph["p0"][0], ph["p0"][1]
                dx, dy = ph["d_xy"]
                d_yaw = ph["d_yaw"]
                yaw0 = ph["yaw0"]
                x = x0 + dx * u
                y = y0 + dy * u
                yaw = _wrap(yaw0 + d_yaw * u)
                return float(x), float(y), float(yaw), float(dx * u_dot), float(dy * u_dot), float(d_yaw * u_dot)

        # Fallback (numerical edge): hold last point
        x, y, yaw = self.points[-1]
        return float(x), float(y), float(yaw), 0.0, 0.0, 0.0

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()