#!/usr/bin/env python3
"""Smooth trajectory generator.

Drop-in replacement for the legacy waypoint_traj nodes: consumes /astar/path
(or a hardcoded `waypoints` parameter for bench tests), simplifies and
spline-smooths the path, time-parameterizes it with a curvature-aware
trapezoidal speed profile, and publishes:

  /reference_trajectory  ReferenceTraj @ publish_rate_hz  (same contract the
                         PID controller already consumes — continuous
                         velocity feedforward instead of piecewise-constant)
  /planned_trajectory    PlannedTrajectory, latched, once per (re)plan
                         (full horizon for the MPC controller)
  /smoothed_path         nav_msgs/Path, latched (RViz/Foxglove debug)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

from hamr_interfaces.msg import PlannedTrajectory, ReferenceTraj

from .common.kinematics import quat_to_yaw
from .trajectory.path_processing import GridInfo, simplify_path
from .trajectory.time_param import SampledTrajectory, build_trajectory

LATCHED = QoSProfile(depth=1,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE)


class TrajGenNode(Node):

    def __init__(self):
        super().__init__("exp_traj_gen")

        self.v_max = self.declare_parameter("v_max", 0.3).value
        self.a_max = self.declare_parameter("a_max", 0.15).value
        self.a_lat_max = self.declare_parameter("a_lat_max", 0.1).value
        self.los_inflation_m = self.declare_parameter("los_inflation_m", 0.20).value
        self.min_spacing = self.declare_parameter("min_waypoint_spacing_m", 0.10).value
        self.yaw_mode = self.declare_parameter("yaw_mode", "hold_goal").value
        self.yaw_fixed = self.declare_parameter("yaw_fixed", 0.0).value
        self.sample_dt = self.declare_parameter("sample_dt", 0.02).value
        self.loop = self.declare_parameter("loop", False).value
        self.trajectory_name = self.declare_parameter("trajectory_name", "").value
        publish_rate_hz = self.declare_parameter("publish_rate_hz", 100.0).value
        # Bench-test waypoints: flat [x1,y1,yaw1, x2,y2,yaw2, ...]. When
        # non-empty, /astar/path is ignored.
        self.waypoints_param = list(
            self.declare_parameter("waypoints", [0.0]).value)

        self.world_frame = self.declare_parameter("world_frame", "odom").value

        self.reference_pub_ = self.create_publisher(
            ReferenceTraj, "/reference_trajectory", 1)
        self.plan_pub_ = self.create_publisher(
            PlannedTrajectory, "/planned_trajectory", LATCHED)
        self.smoothed_path_pub_ = self.create_publisher(
            Path, "/smoothed_path", LATCHED)

        self.grid: GridInfo = None
        self.trajectory: SampledTrajectory = None
        self.t0 = None

        use_waypoints = len(self.waypoints_param) >= 6
        if use_waypoints:
            pts = np.array(self.waypoints_param, dtype=float).reshape(-1, 3)
            self.get_logger().info(
                f"Using {len(pts)} hardcoded waypoints (ignoring /astar/path)")
            self._build(pts[:, :2], goal_yaw=pts[-1, 2])
        else:
            self.create_subscription(OccupancyGrid, "/map",
                                     self._on_map, LATCHED)
            self.create_subscription(Path, "/astar/path",
                                     self._on_path, LATCHED)

        self.create_timer(1.0 / publish_rate_hz, self._tick)
        self.get_logger().info(
            f"exp_traj_gen started: v_max={self.v_max} a_max={self.a_max} "
            f"a_lat_max={self.a_lat_max} yaw_mode={self.yaw_mode}")

    # -- inputs ---------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid):
        self.grid = GridInfo(
            msg.data, msg.info.width, msg.info.height, msg.info.resolution,
            msg.info.origin.position.x, msg.info.origin.position.y)
        self.grid.inflate(self.los_inflation_m)

    def _on_path(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warn("Received <2 path poses; ignoring.")
            return
        pts = np.array([[p.pose.position.x, p.pose.position.y]
                        for p in msg.poses])
        goal_yaw = quat_to_yaw(msg.poses[-1].pose.orientation)
        self._build(pts, goal_yaw)

    # -- planning ---------------------------------------------------------
    def _build(self, points_xy, goal_yaw):
        simplified = simplify_path(points_xy, self.grid,
                                   min_spacing=self.min_spacing)
        if simplified is None:
            self.get_logger().warn("Degenerate path after simplification; ignoring.")
            return

        yaw = self.yaw_fixed if self.yaw_mode == "fixed" else goal_yaw
        self.trajectory = build_trajectory(
            simplified, self.v_max, self.a_max, self.a_lat_max,
            yaw=yaw, dt=self.sample_dt)
        self.t0 = self.get_clock().now()
        self.get_logger().info(
            f"Planned trajectory: {len(points_xy)} -> {len(simplified)} waypoints, "
            f"length {self.trajectory.s_total:.2f} m, "
            f"duration {self.trajectory.total_time:.1f} s")
        self._publish_plan()
        self._publish_smoothed_path()

    # -- outputs ----------------------------------------------------------
    def _tick(self):
        if self.trajectory is None:
            return
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if self.loop and t > self.trajectory.total_time + 2.0:
            self.t0 = self.get_clock().now()
            self._publish_plan()
            return
        x, y, yaw, xd, yd, yawd = self.trajectory.sample(t)
        msg = ReferenceTraj()
        msg.x, msg.y, msg.yaw = float(x), float(y), float(yaw)
        msg.x_dot, msg.y_dot, msg.yaw_dot = float(xd), float(yd), float(yawd)
        self.reference_pub_.publish(msg)

    def _publish_plan(self):
        traj = self.trajectory
        msg = PlannedTrajectory()
        msg.header.stamp = self.t0.to_msg()
        msg.header.frame_id = self.world_frame
        msg.name = self.trajectory_name
        msg.dt = traj.dt
        for i in range(len(traj.t)):
            s = ReferenceTraj()
            s.x, s.y, s.yaw = float(traj.x[i]), float(traj.y[i]), float(traj.yaw)
            s.x_dot, s.y_dot = float(traj.x_dot[i]), float(traj.y_dot[i])
            msg.samples.append(s)
        self.plan_pub_.publish(msg)

    def _publish_smoothed_path(self):
        traj = self.trajectory
        path = Path()
        path.header.frame_id = self.world_frame
        path.header.stamp = self.get_clock().now().to_msg()
        stride = max(1, int(0.1 / traj.dt))  # ~every 10 cm of time
        for i in range(0, len(traj.t), stride):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(traj.x[i])
            ps.pose.position.y = float(traj.y[i])
            heading = math.atan2(traj.y_dot[i], traj.x_dot[i]) \
                if abs(traj.x_dot[i]) + abs(traj.y_dot[i]) > 1e-6 else 0.0
            ps.pose.orientation.z = math.sin(heading * 0.5)
            ps.pose.orientation.w = math.cos(heading * 0.5)
            path.poses.append(ps)
        self.smoothed_path_pub_.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = TrajGenNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
