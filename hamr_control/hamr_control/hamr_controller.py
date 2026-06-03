import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from hamr_interfaces.msg import ReferenceTraj


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(x, hi))


class SimpleHamrController(Node):
    """Drive HAMR base to a (target_x, target_y) waypoint.

    PI on heading (eliminates wheel-asymmetry drift), P on distance.
    """

    def __init__(self):
        super().__init__("simple_hamr_controller")

        # Geometry
        self.declare_parameter("wheel_radius", 0.0762)
        self.declare_parameter("half_track", 0.149556)
        # Gains
        self.declare_parameter("k_v", 1.0)        # distance P
        self.declare_parameter("k_w", 2.0)        # heading P
        self.declare_parameter("k_iw", 0.5)       # heading I  (NEW)
        # Limits
        self.declare_parameter("v_max", 0.3)
        self.declare_parameter("w_max", 1.5)
        self.declare_parameter("iw_limit", 0.5)   # anti-windup (NEW)
        # Tolerances
        self.declare_parameter("pos_tol", 0.05)
        self.declare_parameter("heading_gate_deg", 45.0)
        # Topics & rate
        self.declare_parameter("odom_topic", "/HAMR_base/odom")
        self.declare_parameter("ref_topic", "/reference_trajectory")
        self.declare_parameter("rate_hz", 50.0)

        self.r_w = self.get_parameter("wheel_radius").value
        self.a = self.get_parameter("half_track").value
        self.k_v = self.get_parameter("k_v").value
        self.k_w = self.get_parameter("k_w").value
        self.k_iw = self.get_parameter("k_iw").value
        self.v_max = self.get_parameter("v_max").value
        self.w_max = self.get_parameter("w_max").value
        self.iw_limit = self.get_parameter("iw_limit").value
        self.pos_tol = self.get_parameter("pos_tol").value
        self.heading_gate = math.radians(self.get_parameter("heading_gate_deg").value)
        odom_topic = self.get_parameter("odom_topic").value
        ref_topic = self.get_parameter("ref_topic").value
        rate = self.get_parameter("rate_hz").value
        self.dt = 1.0 / rate

        self.pose = None
        self.target_x = None
        self.target_y = None
        self.iw = 0.0  # heading integral accumulator (NEW)
        self.tick_count = 0

        self.left_pub = self.create_publisher(Float64, "/left_wheel/cmd_vel", 1)
        self.right_pub = self.create_publisher(Float64, "/right_wheel/cmd_vel", 1)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)
        self.create_subscription(ReferenceTraj, ref_topic, self.ref_cb, 1)
        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f"simple_hamr_controller up. K_v={self.k_v}, K_w={self.k_w}, "
            f"K_iw={self.k_iw}, v_max={self.v_max}, w_max={self.w_max}, rate={rate} Hz"
        )

    def odom_cb(self, msg: Odometry):
        self.pose = msg.pose.pose

    def ref_cb(self, msg: ReferenceTraj):
        self.target_x = msg.x
        self.target_y = msg.y

    def publish_wheels(self, v: float, w: float):
        right = -(v + self.a * w) / self.r_w
        left = (v - self.a * w) / self.r_w
        rmsg = Float64(); rmsg.data = float(right); self.right_pub.publish(rmsg)
        lmsg = Float64(); lmsg.data = float(left); self.left_pub.publish(lmsg)

    def tick(self):
        if self.pose is None or self.target_x is None:
            return

        x = self.pose.position.x
        y = self.pose.position.y
        yaw = quat_to_yaw(self.pose.orientation)

        err_x = self.target_x - x
        err_y = self.target_y - y
        dist = math.hypot(err_x, err_y)

        if dist < self.pos_tol:
            self.publish_wheels(0.0, 0.0)
            self.iw = 0.0  # reset I when goal reached
            return

        heading_des = math.atan2(err_y, err_x)
        err_heading = wrap(heading_des - yaw)

        # Forward velocity gated by heading
        if abs(err_heading) > self.heading_gate:
            # Pure-rotation phase: freeze the integral (don't wind up while v=0)
            v = 0.0
        else:
            # Drive phase: accumulate heading integral to eat steady-state drift
            self.iw += err_heading * self.dt
            self.iw = clamp(self.iw, -self.iw_limit, self.iw_limit)
            v = clamp(self.k_v * dist, -self.v_max, self.v_max)
            v *= math.cos(err_heading)

        # PI on yaw rate
        w = clamp(
            self.k_w * err_heading + self.k_iw * self.iw,
            -self.w_max,
            self.w_max,
        )

        self.publish_wheels(v, w)

        self.tick_count += 1
        if self.tick_count <= 10 or self.tick_count % 50 == 0:
            right = (v + self.a * w) / self.r_w
            left = (v - self.a * w) / self.r_w
            self.get_logger().info(
                f"[t={self.tick_count}] x={x:.3f} y={y:.3f} yaw={yaw:+.3f}  "
                f"dist={dist:.3f} err_h={err_heading:+.3f}  iw={self.iw:+.3f}  "
                f"v={v:+.3f} w={w:+.3f}  L={left:+.3f} R={right:+.3f}"
            )


def main():
    rclpy.init()
    node = SimpleHamrController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()