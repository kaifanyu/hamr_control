import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from hamr_interfaces.msg import ReferenceTraj


def yaw_from_quat(q):
    """Convert ROS quaternion to yaw."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class WaypointTrajSequence(Node):
    """
    Publishes a sequence of ReferenceTraj waypoints.

    The waypoints are LOCAL OFFSETS from the robot's starting odom pose.

    Example:
        [0.0, 3.0, 0.0]

    means:
        target_x = start_x + 0.0
        target_y = start_y + 3.0
        target_yaw = start_yaw + 0.0

    The controller receives only the current active waypoint.
    Once the robot is close enough to that waypoint, this node advances
    to the next one.
    """

    def __init__(self):
        super().__init__("waypoint_traj_sequence")

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("odom_topic", "/HAMR_base/odom")
        self.declare_parameter("reference_topic", "/reference_trajectory")

        # Distance threshold for switching to the next waypoint.
        self.declare_parameter("position_tolerance", 0.08)  # meters

        # Optional yaw threshold.
        # If your controller is currently ignoring yaw/turret, this does not matter much.
        self.declare_parameter("yaw_tolerance", 0.20)  # radians

        # If true, require both position and yaw to be close before advancing.
        # For now, I recommend false while testing base waypoint tracking.
        self.declare_parameter("require_yaw_goal", False)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        odom_topic = self.get_parameter("odom_topic").value
        ref_topic = self.get_parameter("reference_topic").value

        self.position_tolerance = float(
            self.get_parameter("position_tolerance").value
        )
        self.yaw_tolerance = float(
            self.get_parameter("yaw_tolerance").value
        )
        self.require_yaw_goal = bool(
            self.get_parameter("require_yaw_goal").value
        )

        self.local_waypoints = np.array(
            [
                # x offset, y offset, yaw offset
                [0.0, 0.0, 0.0],
                [0.0, 3.0, 0.0],
            ],
            dtype=float,
        )

        # Start pose is locked from first odom message.
        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        # Current odom pose.
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # Global waypoint list gets generated after first odom.
        self.global_waypoints = None
        self.current_waypoint_idx = 0
        self.finished = False

        self.ref_pub_ = self.create_publisher(ReferenceTraj, ref_topic, 1)

        self.odom_sub_ = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_cb,
            1,
        )

        self.create_timer(1.0 / rate_hz, self.publish_reference)

        self.get_logger().info(
            f"Waiting for first odom on '{odom_topic}'. "
            f"Publishing references to '{ref_topic}'."
        )

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw

        # Lock start pose once.
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.start_yaw = yaw

            self.build_global_waypoints()

            self.get_logger().info(
                f"Locked start pose: "
                f"x={self.start_x:.3f}, "
                f"y={self.start_y:.3f}, "
                f"yaw={self.start_yaw:.3f}"
            )

            first = self.global_waypoints[0]
            self.get_logger().info(
                f"Starting waypoint sequence with {len(self.global_waypoints)} points. "
                f"First target: x={first[0]:.3f}, y={first[1]:.3f}, yaw={first[2]:.3f}"
            )

    def build_global_waypoints(self):
        """
        Convert local offset waypoints into odom-frame global waypoints.

        Important:
        This version treats local x/y offsets as odom-frame offsets.

        So:
            [0.0, 3.0, 0.0]

        means:
            start_x + 0.0
            start_y + 3.0

        It does NOT rotate the offsets by the robot's starting yaw.
        """

        self.global_waypoints = []

        for dx, dy, dyaw in self.local_waypoints:
            gx = self.start_x + dx
            gy = self.start_y + dy
            gyaw = wrap_angle(self.start_yaw + dyaw)

            self.global_waypoints.append([gx, gy, gyaw])

        self.global_waypoints = np.array(self.global_waypoints, dtype=float)

    def reached_current_waypoint(self) -> bool:
        if self.current_x is None or self.global_waypoints is None:
            return False

        target = self.global_waypoints[self.current_waypoint_idx]
        target_x = target[0]
        target_y = target[1]
        target_yaw = target[2]

        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist = math.hypot(dx, dy)

        yaw_err = abs(wrap_angle(target_yaw - self.current_yaw))

        position_ok = dist <= self.position_tolerance
        yaw_ok = yaw_err <= self.yaw_tolerance

        if self.require_yaw_goal:
            return position_ok and yaw_ok

        return position_ok

    def advance_waypoint_if_needed(self):
        if self.finished:
            return

        if self.global_waypoints is None:
            return

        if not self.reached_current_waypoint():
            return

        old_idx = self.current_waypoint_idx

        if self.current_waypoint_idx >= len(self.global_waypoints) - 1:
            self.finished = True
            self.get_logger().info("Finished all waypoints.")
            return

        self.current_waypoint_idx += 1
        target = self.global_waypoints[self.current_waypoint_idx]

        self.get_logger().info(
            f"Reached waypoint {old_idx}. "
            f"Advancing to waypoint {self.current_waypoint_idx}: "
            f"x={target[0]:.3f}, y={target[1]:.3f}, yaw={target[2]:.3f}"
        )

    def publish_reference(self):
        if self.global_waypoints is None:
            return

        # Check whether we should move to the next target.
        self.advance_waypoint_if_needed()

        # Keep publishing the final waypoint after finishing.
        if self.finished:
            target = self.global_waypoints[-1]
        else:
            target = self.global_waypoints[self.current_waypoint_idx]

        msg = ReferenceTraj()
        msg.x = float(target[0])
        msg.y = float(target[1])
        msg.yaw = float(target[2])

        # x_dot, y_dot, yaw_dot, roll left at 0.0 by default.
        # roll=0 keeps the controller in normal position-tracking mode.
        self.ref_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointTrajSequence()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()