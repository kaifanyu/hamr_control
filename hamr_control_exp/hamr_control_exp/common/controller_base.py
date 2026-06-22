"""Base class for experimental tracking controllers.

Provides the shared plumbing so controller nodes only implement compute():
parameters (geometry, limits, topics), the StateListener, the three Float64
command publishers, the 100 Hz control timer, a stale-input watchdog, and the
cap -> Jacobian -> publish path. Output topic contract is identical to the
PID node: /left_wheel/cmd_vel, /right_wheel/cmd_vel, /turret/cmd_vel.
"""
import math

import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64

from hamr_interfaces.msg import ReferenceTraj

from .kinematics import wrap_angle, cap_world_velocity, world_vel_to_joint_omegas
from .robot_state import StateListener


class ExpControllerBase(Node):

    def __init__(self, node_name: str, auto_timer: bool = True):
        super().__init__(node_name)

        # Geometry defaults match hamr_bringup/config/hamr_hw_control_params.yaml
        self.declare_parameter("r_wheel", 0.1250)
        self.declare_parameter("a_wheel", 0.345)
        self.declare_parameter("b_wheel", 0.301)
        self.declare_parameter("base_yaw_offset", 1.57079632679)
        self.declare_parameter("simulating", False)
        self.declare_parameter("base_odom_topic", "")
        self.declare_parameter("turret_odom_topic", "")
        self.declare_parameter("control_rate_hz", 100.0)
        self.declare_parameter("xy_dot_limit", 0.41)
        self.declare_parameter("yaw_dot_limit", 2.0)
        self.declare_parameter("watchdog_timeout_s", 0.5)

        self.r_wheel = self.get_parameter("r_wheel").value
        self.a_wheel = self.get_parameter("a_wheel").value
        self.b_wheel = self.get_parameter("b_wheel").value
        self.base_yaw_offset = self.get_parameter("base_yaw_offset").value
        self.simulating = self.get_parameter("simulating").value
        self.control_rate_hz = self.get_parameter("control_rate_hz").value
        self.xy_dot_limit = self.get_parameter("xy_dot_limit").value
        self.yaw_dot_limit = self.get_parameter("yaw_dot_limit").value
        self.watchdog_timeout_s = self.get_parameter("watchdog_timeout_s").value

        self.left_wheel_vel_ = self.create_publisher(Float64, "/left_wheel/cmd_vel", 1)
        self.right_wheel_vel_ = self.create_publisher(Float64, "/right_wheel/cmd_vel", 1)
        self.turret_vel_ = self.create_publisher(Float64, "/turret/cmd_vel", 1)

        self.state = StateListener(
            self, self.simulating,
            self.get_parameter("base_odom_topic").value,
            self.get_parameter("turret_odom_topic").value)

        self.reference_: ReferenceTraj = None
        self._last_reference_time = None
        self.create_subscription(ReferenceTraj, "/reference_trajectory",
                                 self._on_reference, 1)

        self._active = False  # becomes True once we have published a command
        self.last_control_time = self.get_clock().now()
        self.dt = 1.0 / self.control_rate_hz
        if auto_timer:
            self.create_timer(1.0 / self.control_rate_hz, self.control_tick)

    # -- subclass hook -------------------------------------------------
    def compute(self, state, ref: ReferenceTraj, dt: float) -> np.ndarray:
        """Return desired world velocity [x_dot, y_dot, yaw_turret_dot]."""
        raise NotImplementedError

    # -- shared machinery ----------------------------------------------
    def _on_reference(self, msg: ReferenceTraj):
        self.reference_ = msg
        self._last_reference_time = self.get_clock().now()

    def reference_age_s(self) -> float:
        if self._last_reference_time is None:
            return math.inf
        return (self.get_clock().now() - self._last_reference_time).nanoseconds * 1e-9

    def inputs_fresh(self) -> bool:
        return (self.state.ready()
                and self.state.age_s() < self.watchdog_timeout_s
                and self.reference_age_s() < self.watchdog_timeout_s)

    def control_tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now
        if not math.isfinite(dt) or dt <= 0.0:
            return
        self.dt = max(1e-4, min(dt, 0.1))

        if not self.inputs_fresh():
            # Same convention as the PID node before the first reference:
            # publish nothing. Once active, zero the wheels on stale inputs.
            if self._active:
                self.publish_zero()
            return

        state = self.state.snapshot()
        v = self.compute(state, self.reference_, self.dt)
        self.publish_world_velocity(v, state)

    def publish_world_velocity(self, v, state):
        v, _ = cap_world_velocity(v, self.xy_dot_limit, self.yaw_dot_limit)
        yaw_kin = wrap_angle(state.yaw_base + self.base_yaw_offset)
        omegas = world_vel_to_joint_omegas(
            v, yaw_kin, self.r_wheel, self.a_wheel, self.b_wheel)
        self._publish_omegas(omegas)
        self._active = True

    def publish_zero(self):
        self._publish_omegas((0.0, 0.0, 0.0))

    def _publish_omegas(self, omegas):
        for pub, w in zip(
                (self.right_wheel_vel_, self.left_wheel_vel_, self.turret_vel_),
                omegas):
            msg = Float64()
            msg.data = float(w)
            pub.publish(msg)
