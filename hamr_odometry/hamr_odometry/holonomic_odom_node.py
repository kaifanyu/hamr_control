#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


VICON_ODOM_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def positive_finite_parameter(name, value):
    """Validate a calibration multiplier or measurement variance at startup."""
    if isinstance(value, bool):
        raise ValueError(f'{name} must be finite and strictly positive')
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{name} must be finite and strictly positive') from exc
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f'{name} must be finite and strictly positive')
    return result


def wheel_body_twist(
    omega_left, omega_right, r_wheel, a_wheel, b_wheel, yaw_offset,
    yaw_sign=1.0, linear_velocity_scale=1.0, yaw_rate_scale=1.0,
):
    """Return the tracked point's (vx, vy, wz), expressed in base_link.

    The linear multiplier calibrates axle forward speed. The yaw multiplier
    calibrates base rotation and therefore also the offset-point velocity.
    Heading and integration interval do not belong in this body-frame mapping.
    For yaw_offset=pi/2 the result is (-b_wheel*wz, forward_speed, wz).
    """
    forward_speed = linear_velocity_scale * r_wheel * 0.5 * (
        omega_right + omega_left)
    yaw_rate = yaw_rate_scale * yaw_sign * r_wheel / (2.0 * a_wheel) * (
        omega_right - omega_left)
    c, s = math.cos(yaw_offset), math.sin(yaw_offset)
    return (
        forward_speed * c - b_wheel * yaw_rate * s,
        forward_speed * s + b_wheel * yaw_rate * c,
        yaw_rate,
    )


class HolonomicOdomNode(Node):
    def __init__(self):
        super().__init__('holonomic_odom_node')

        # Robot geometry; hardware launches supply their calibrated overrides.
        self.declare_parameter('r_wheel', 0.1250)
        self.declare_parameter('a_wheel', 0.345)
        self.declare_parameter('b_wheel', 0.301)
        self.declare_parameter('base_yaw_offset', math.pi / 2.0)
        self.declare_parameter('ticks_per_rev', 2263.7)
        self.declare_parameter('left_tick_scale', 1.0)
        self.declare_parameter('right_tick_scale', 1.0)
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('linear_velocity_scale', 1.0)
        self.declare_parameter('yaw_rate_scale', 1.0)
        self.declare_parameter('twist_variance_vx', 0.01)
        self.declare_parameter('twist_variance_vy', 0.01)
        self.declare_parameter('twist_variance_wz', 0.01)
        self.declare_parameter('ticks_per_turret_rev', 2704)  # 13 PPR × 2 quadrature × 104 gear ratio
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('base_odom_topic', 'HAMR_base/odom')
        self.declare_parameter('ekf_yaw_timeout', 0.25)
        # Set to False once the EKF node is running (it will publish TF instead)
        self.declare_parameter('publish_tf', True)

        self.r = self.get_parameter('r_wheel').value
        self.a = self.get_parameter('a_wheel').value
        self.b = self.get_parameter('b_wheel').value
        self.yaw_offset = self.get_parameter('base_yaw_offset').value
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.left_tick_scale = float(self.get_parameter('left_tick_scale').value)
        self.right_tick_scale = float(self.get_parameter('right_tick_scale').value)
        self.yaw_sign = float(self.get_parameter('yaw_sign').value)
        for name in (
            'linear_velocity_scale', 'yaw_rate_scale',
            'twist_variance_vx', 'twist_variance_vy', 'twist_variance_wz',
        ):
            setattr(self, name, positive_finite_parameter(
                name, self.get_parameter(name).value))
        self.ticks_per_turret_rev = float(self.get_parameter('ticks_per_turret_rev').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.base_odom_topic = self.get_parameter('base_odom_topic').value
        self.ekf_yaw_timeout = float(self.get_parameter('ekf_yaw_timeout').value)

        # Latest EKF base yaw from HAMR_base/odom
        self.latest_ekf_yaw = None
        self.latest_ekf_yaw_time = None

        # Integrated pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Latest cumulative encoder ticks from relay_node
        self.latest_L = None
        self.latest_R = None
        self.latest_T = 0   # default 0 so turret odom publishes immediately at startup
        self.prev_L = None
        self.prev_R = None
        self.last_time = self.get_clock().now()

        self.create_subscription(Int32, '/left_wheel/encoder_ticks', self._cb_L, 10)
        self.create_subscription(Int32, '/right_wheel/encoder_ticks', self._cb_R, 10)
        self.create_subscription(Int32, '/turret/encoder_ticks', self._cb_T, 10)

        # subscribe to odometry topic
        self.base_odom_sub = self.create_subscription(
            Odometry,
            self.base_odom_topic,
            self._cb_base_odom,
            VICON_ODOM_QOS,
        )

        self.pub_odom = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.pub_turret_odom = self.create_publisher(Odometry, 'HAMR_turret/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_timer(0.02, self._update)  # 50 Hz integration

        self.get_logger().info(
            f'HolonomicOdomNode ready  r={self.r} a={self.a} b={self.b} '
            f'ticks_per_rev={self.ticks_per_rev:.1f} '
            f'tick_scale=({self.left_tick_scale:.3f},{self.right_tick_scale:.3f}) '
            f'velocity_scale=({self.linear_velocity_scale:.6f},'
            f'{self.yaw_rate_scale:.6f}) '
            f'yaw_sign={self.yaw_sign:+.1f} publish_tf={self.publish_tf}'
        )

    def _cb_L(self, msg: Int32):
        self.latest_L = msg.data

    def _cb_R(self, msg: Int32):
        self.latest_R = msg.data

    def _cb_T(self, msg: Int32):
        self.latest_T = msg.data

    def _cb_base_odom(self, msg: Odometry):
        # read the EKF-filtered base yaw from HAMR_base/odom
        self.latest_ekf_yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.latest_ekf_yaw_time = self.get_clock().now()

    def _update(self):
        if self.latest_L is None or self.latest_R is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Seed previous ticks on the first valid call
        if self.prev_L is None:
            self.prev_L = self.latest_L
            self.prev_R = self.latest_R
            return

        # Guard against bad dt (clock jump or startup artifact)
        if dt <= 0.0 or dt > 0.5:
            self.prev_L = self.latest_L
            self.prev_R = self.latest_R
            return

        delta_L = (self.latest_L - self.prev_L) * self.left_tick_scale
        delta_R = (self.latest_R - self.prev_R) * self.right_tick_scale
        self.prev_L = self.latest_L
        self.prev_R = self.latest_R

        # Wheel angular velocities in rad/s
        omega_L = (delta_L / self.ticks_per_rev) * 2.0 * math.pi / dt
        omega_R = (delta_R / self.ticks_per_rev) * 2.0 * math.pi / dt

        # Holonomic forward kinematics. yaw_sign maps encoder-positive wheel
        # rotation into REP-103 positive yaw (CCW in the odom frame).
        vx_body, vy_body, yaw_dot = wheel_body_twist(
            omega_L, omega_R, self.r, self.a, self.b, self.yaw_offset,
            self.yaw_sign, self.linear_velocity_scale, self.yaw_rate_scale,
        )
        ct, st = math.cos(self.theta), math.sin(self.theta)
        x_dot = ct * vx_body - st * vy_body
        y_dot = st * vx_body + ct * vy_body

        # Euler integration of pose
        self.theta = wrap_angle(self.theta + yaw_dot * dt)
        self.x += x_dot * dt
        self.y += y_dot * dt

        # Quaternion from yaw (rotation only around Z)
        half = self.theta * 0.5
        qz = math.sin(half)
        qw = math.cos(half)

        # This callback runs at 50 Hz; keep per-cycle telemetry available for
        # targeted debugging without flooding normal hardware-run logs.
        self.get_logger().debug(
            f"dL={delta_L:.1f}, dR={delta_R:.1f}, "
            f"omega_L={omega_L:.3f}, omega_R={omega_R:.3f}, "
            f"yaw_dot={yaw_dot:.3f}, theta={self.theta:.3f}"
        )

        stamp = now.to_msg()

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Pose covariance (6x6 row-major: x,y,z,rx,ry,rz)
        # Tuned later; start with small diagonal values
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[14] = 1e-9   # z (not used)
        odom.pose.covariance[21] = 1e-9   # roll (not used)
        odom.pose.covariance[28] = 1e-9   # pitch (not used)
        odom.pose.covariance[35] = 0.01   # yaw

        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = yaw_dot

        # Twist covariance
        odom.twist.covariance[0]  = self.twist_variance_vx
        odom.twist.covariance[7]  = self.twist_variance_vy
        odom.twist.covariance[14] = 1e-9
        odom.twist.covariance[21] = 1e-9
        odom.twist.covariance[28] = 1e-9
        odom.twist.covariance[35] = self.twist_variance_wz

        self.pub_odom.publish(odom)

        # Publish HAMR_turret/odom — controller gates on this in hardware mode.
        # turret_yaw_world = base heading + turret angle relative to base.
        # latest_T defaults to 0 so this publishes from the first integration step
        # even if /turret/encoder_ticks hasn't arrived yet.
        turret_rel = (self.latest_T / self.ticks_per_turret_rev) * 2.0 * math.pi

        # Prefer EKF-filtered base yaw. Fall back to raw wheel-integrated yaw
        # if EKF has not published yet or is stale.
        base_yaw_for_turret = self.theta

        if self.latest_ekf_yaw is not None and self.latest_ekf_yaw_time is not None:
            ekf_age = (now - self.latest_ekf_yaw_time).nanoseconds * 1e-9
            if ekf_age <= self.ekf_yaw_timeout:
                base_yaw_for_turret = self.latest_ekf_yaw

        turret_yaw = wrap_angle(base_yaw_for_turret + turret_rel)

        t_half = turret_yaw * 0.5
        turret_odom = Odometry()
        turret_odom.header.stamp = stamp
        turret_odom.header.frame_id = self.odom_frame
        turret_odom.child_frame_id = 'turret_link'
        turret_odom.pose.pose.orientation.z = math.sin(t_half)
        turret_odom.pose.pose.orientation.w = math.cos(t_half)
        self.pub_turret_odom.publish(turret_odom)

        # Broadcast TF: odom → base_link (disabled once EKF is running)
        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = 0.0
            tf.transform.rotation.y = 0.0
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = HolonomicOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
