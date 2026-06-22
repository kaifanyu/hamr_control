#!/usr/bin/env python3
"""Online tracking metrics for live Foxglove plotting.

Publishes Float64MultiArray on /tracking_metrics at 50 Hz:
  [0] err_x            world-frame position error x
  [1] err_y            world-frame position error y
  [2] err_norm         ||position error||
  [3] cross_track      error component perpendicular to reference velocity
  [4] along_track      error component along reference velocity
  [5] yaw_dev          turret world-yaw deviation from reference [rad]
  [6] rms_ct_running   running RMS of cross-track (last ~10 s)
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from hamr_interfaces.msg import ReferenceTraj

from .common.kinematics import wrap_angle
from .common.robot_state import StateListener


class MetricsNode(Node):

    def __init__(self):
        super().__init__("exp_metrics")
        simulating = self.declare_parameter("simulating", False).value
        base_topic = self.declare_parameter("base_odom_topic", "").value
        turret_topic = self.declare_parameter("turret_odom_topic", "").value
        rate_hz = self.declare_parameter("rate_hz", 50.0).value

        self.state = StateListener(self, simulating, base_topic, turret_topic)
        self.reference_: ReferenceTraj = None
        self.create_subscription(ReferenceTraj, "/reference_trajectory",
                                 self._on_reference, 1)
        self.pub_ = self.create_publisher(Float64MultiArray,
                                          "/tracking_metrics", 10)
        self._ct_window = deque(maxlen=int(rate_hz * 10))
        self.create_timer(1.0 / rate_hz, self._tick)

    def _on_reference(self, msg: ReferenceTraj):
        self.reference_ = msg

    def _tick(self):
        if self.reference_ is None or not self.state.ready():
            return
        ref = self.reference_
        s = self.state.snapshot()

        err_x = ref.x - s.x
        err_y = ref.y - s.y
        err_norm = math.hypot(err_x, err_y)
        speed = math.hypot(ref.x_dot, ref.y_dot)
        if speed > 0.02:
            tx, ty = ref.x_dot / speed, ref.y_dot / speed
            along = err_x * tx + err_y * ty
            cross = -err_x * ty + err_y * tx
        else:
            along, cross = 0.0, err_norm
        yaw_dev = wrap_angle(ref.yaw - s.yaw_turret_world)

        self._ct_window.append(cross * cross)
        rms_ct = math.sqrt(sum(self._ct_window) / len(self._ct_window))

        msg = Float64MultiArray()
        msg.data = [err_x, err_y, err_norm, cross, along, yaw_dev, rms_ct]
        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
