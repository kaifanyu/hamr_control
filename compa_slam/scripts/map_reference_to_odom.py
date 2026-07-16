#!/usr/bin/env python3
"""Transform global/route ReferenceTraj messages into smooth local odom references.

The controller consumes ``/local_HAMR/odom``, whose pose is continuous but drifts.
RTAB-Map owns ``map -> odom``. This node applies the inverse map correction to each
reference, allowing the controller to remain entirely in ``odom`` while following a
route that remains fixed in ``map``.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from hamr_interfaces.msg import ReferenceTraj
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

from reference_transform_math import (
    Pose2D,
    anchor_route_at_pose,
    compose,
    inverse,
    rotate_vector,
)


def yaw_from_quaternion(quaternion) -> float:
    """Extract planar yaw from a geometry_msgs quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class MapReferenceToOdom(Node):
    """Convert map-fixed or start-relative references into the EKF odom frame."""

    def __init__(self):
        super().__init__("map_reference_to_odom")

        self.declare_parameter("input_topic", "/reference_trajectory_map")
        self.declare_parameter("output_topic", "/reference_trajectory")
        self.declare_parameter("local_odom_topic", "/local_HAMR/odom")
        self.declare_parameter("turret_odom_topic", "/HAMR_turret/odom")
        self.declare_parameter("localization_pose_topic", "/localization_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("localization_timeout_s", 2.0)
        self.declare_parameter("transform_timeout_s", 0.1)
        self.declare_parameter("require_localization", True)
        self.declare_parameter("hold_on_stale_localization", False)
        self.declare_parameter("anchor_to_start", True)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        local_odom_topic = self.get_parameter("local_odom_topic").value
        turret_odom_topic = self.get_parameter("turret_odom_topic").value
        localization_topic = self.get_parameter("localization_pose_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.localization_timeout = float(
            self.get_parameter("localization_timeout_s").value
        )
        self.transform_timeout = float(
            self.get_parameter("transform_timeout_s").value
        )
        self.require_localization = bool(
            self.get_parameter("require_localization").value
        )
        self.hold_on_stale_localization = bool(
            self.get_parameter("hold_on_stale_localization").value
        )
        self.anchor_to_start = bool(self.get_parameter("anchor_to_start").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)

        self.reference = None
        self.local_odom = None
        self.turret_odom = None
        self.last_localization_rx = None
        self.map_from_route = None
        self.was_active = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.reference_sub = self.create_subscription(
            ReferenceTraj, input_topic, self._reference_callback, 10
        )
        self.local_odom_sub = self.create_subscription(
            Odometry, local_odom_topic, self._local_odom_callback, 20
        )
        self.turret_odom_sub = self.create_subscription(
            Odometry, turret_odom_topic, self._turret_odom_callback, 20
        )
        self.localization_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            localization_topic,
            self._localization_callback,
            10,
        )
        self.publisher = self.create_publisher(ReferenceTraj, output_topic, 10)
        self.create_timer(1.0 / max(1.0, publish_rate), self._publish_reference)

        mode = "start-relative route" if self.anchor_to_start else "absolute map"
        self.get_logger().info(
            f"Reference correction ready: {input_topic} ({mode}) -> "
            f"{output_topic} ({self.odom_frame})"
        )

    def _reference_callback(self, message: ReferenceTraj):
        self.reference = message

    def _local_odom_callback(self, message: Odometry):
        self.local_odom = message

    def _turret_odom_callback(self, message: Odometry):
        self.turret_odom = message

    def _localization_callback(self, _message: PoseWithCovarianceStamped):
        self.last_localization_rx = self.get_clock().now()

    def _localization_is_usable(self) -> bool:
        if not self.require_localization:
            return True
        if self.last_localization_rx is None:
            return False
        if not self.hold_on_stale_localization:
            # After the first successful relocalization, a temporarily missing
            # visual update should not stop smooth local control. Keep using the
            # last map->odom correction until RTAB-Map updates it again.
            return True
        age = (self.get_clock().now() - self.last_localization_rx).nanoseconds * 1e-9
        return age <= self.localization_timeout

    @staticmethod
    def _pose_from_odometry(message: Odometry) -> Pose2D:
        pose = message.pose.pose
        return Pose2D(
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )

    @staticmethod
    def _pose_from_reference(message: ReferenceTraj) -> Pose2D:
        return Pose2D(message.x, message.y, message.yaw)

    @staticmethod
    def _pose_from_transform(message) -> Pose2D:
        transform = message.transform
        return Pose2D(
            transform.translation.x,
            transform.translation.y,
            yaw_from_quaternion(transform.rotation),
        )

    def _lookup_odom_from_map(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.map_frame,
                Time(),
                timeout=Duration(seconds=self.transform_timeout),
            )
            return self._pose_from_transform(transform)
        except Exception as error:  # tf2 has multiple lookup exception classes
            if self.was_active:
                self.get_logger().warn(f"Lost {self.odom_frame} <- {self.map_frame}: {error}")
            return None

    def _publish_hold(self):
        """Publish a zero-velocity reference at the current local pose."""
        if self.local_odom is None:
            return
        base = self._pose_from_odometry(self.local_odom)
        hold_yaw = base.yaw
        if self.turret_odom is not None:
            hold_yaw = yaw_from_quaternion(
                self.turret_odom.pose.pose.orientation
            )

        hold = ReferenceTraj()
        hold.x = base.x
        hold.y = base.y
        hold.yaw = hold_yaw
        self.publisher.publish(hold)

    def _publish_reference(self):
        if self.reference is None or self.local_odom is None:
            return

        if not self._localization_is_usable():
            if self.was_active:
                self.get_logger().warn(
                    "RTAB-Map localization is stale; holding the current local pose."
                )
            self.was_active = False
            self._publish_hold()
            return

        odom_from_map = self._lookup_odom_from_map()
        if odom_from_map is None:
            self.was_active = False
            self._publish_hold()
            return

        route_reference = self._pose_from_reference(self.reference)

        if self.anchor_to_start and self.map_from_route is None:
            odom_from_base = self._pose_from_odometry(self.local_odom)
            map_from_base = compose(inverse(odom_from_map), odom_from_base)
            self.map_from_route = anchor_route_at_pose(
                map_from_base, route_reference
            )
            self.get_logger().info(
                "Anchored the simple trajectory at the current localized map pose."
            )

        if self.map_from_route is not None:
            map_from_reference = compose(self.map_from_route, route_reference)
            map_vx, map_vy = rotate_vector(
                self.map_from_route.yaw,
                self.reference.x_dot,
                self.reference.y_dot,
            )
        else:
            map_from_reference = route_reference
            map_vx, map_vy = self.reference.x_dot, self.reference.y_dot

        odom_from_reference = compose(odom_from_map, map_from_reference)
        odom_vx, odom_vy = rotate_vector(
            odom_from_map.yaw, map_vx, map_vy
        )

        corrected = ReferenceTraj()
        corrected.x = odom_from_reference.x
        corrected.y = odom_from_reference.y
        corrected.yaw = odom_from_reference.yaw
        corrected.x_dot = odom_vx
        corrected.y_dot = odom_vy
        corrected.yaw_dot = self.reference.yaw_dot
        corrected.roll = self.reference.roll
        corrected.pitch = self.reference.pitch
        corrected.roll_dot = self.reference.roll_dot
        corrected.pitch_dot = self.reference.pitch_dot
        self.publisher.publish(corrected)
        self.was_active = True


def main(args=None):
    rclpy.init(args=args)
    node = MapReferenceToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
