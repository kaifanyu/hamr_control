"""Robot state subscription helper.

Mirrors the topic conventions of hamr_controller.py:
  simulation: base pose on /hamr/odom, turret yaw relative to base from /tf
              (turret_link <- base_link)
  hardware:   base pose on HAMR_base/odom (Vicon or EKF), turret world
              orientation on HAMR_turret/odom
"""
import math
from dataclasses import dataclass

from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from .kinematics import wrap_angle, quat_to_yaw


@dataclass
class RobotState:
    x: float
    y: float
    yaw_base: float          # measured base yaw in world frame (no offset)
    yaw_turret_world: float  # turret yaw in world frame


class StateListener:
    """Owns the odometry subscriptions for a node and exposes the latest
    fused state. Not a node itself; pass the owning node in."""

    def __init__(self, node, simulating: bool,
                 base_odom_topic: str = "", turret_odom_topic: str = ""):
        self._node = node
        self._simulating = simulating
        self._pose = None
        self._yaw_turret_rel = None    # sim: turret yaw relative to base
        self._yaw_turret_world = None  # hw: turret yaw in world
        self._last_update = None

        if simulating:
            base_topic = base_odom_topic or "/hamr/odom"
            node.create_subscription(Odometry, base_topic, self._on_base_odom, 1)
            node.create_subscription(TFMessage, "/tf", self._on_tf, 1)
        else:
            base_topic = base_odom_topic or "HAMR_base/odom"
            turret_topic = turret_odom_topic or "HAMR_turret/odom"
            node.create_subscription(Odometry, base_topic, self._on_base_odom, 1)
            node.create_subscription(Odometry, turret_topic, self._on_turret_odom, 1)

    def _on_base_odom(self, msg: Odometry):
        self._pose = msg.pose.pose
        self._last_update = self._node.get_clock().now()

    def _on_turret_odom(self, msg: Odometry):
        self._yaw_turret_world = quat_to_yaw(msg.pose.pose.orientation)

    def _on_tf(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id == "turret_link" and t.header.frame_id == "base_link":
                self._yaw_turret_rel = quat_to_yaw(t.transform.rotation)
                break

    def ready(self) -> bool:
        if self._pose is None:
            return False
        if self._simulating:
            return self._yaw_turret_rel is not None
        return self._yaw_turret_world is not None

    def age_s(self) -> float:
        """Seconds since the last base odometry update (inf if never)."""
        if self._last_update is None:
            return math.inf
        return (self._node.get_clock().now() - self._last_update).nanoseconds * 1e-9

    def snapshot(self) -> RobotState:
        yaw_base = quat_to_yaw(self._pose.orientation)
        if self._simulating:
            yaw_turret_world = wrap_angle(yaw_base + self._yaw_turret_rel)
        else:
            yaw_turret_world = wrap_angle(self._yaw_turret_world)
        return RobotState(
            x=self._pose.position.x,
            y=self._pose.position.y,
            yaw_base=yaw_base,
            yaw_turret_world=yaw_turret_world,
        )
