#!/usr/bin/env python3
"""LQI (LQR + integral action) tracking controller.

Because the shared Jacobian solve feedback-linearizes the base, the plant in
world coordinates is three decoupled integrators driven by commanded
velocity. We regulate the error state z = [e_pos(3); e_int(3)] with

    e_{k+1} = e_k - dt * u_k        (u = velocity correction)
    i_{k+1} = i_k + dt * e_k

and command  v = v_reference + u,  u = -K z,  where K solves the discrete
LQR problem for weights Q = diag(q_pos, q_pos, q_yaw, q_int...), R = diag(r_u...).
Gains are computed once at startup (scipy DARE); the 100 Hz loop is a single
matrix-vector multiply plus the shared Jacobian solve.

Same I/O contract as the PID node: /reference_trajectory + odometry in,
/left_wheel/cmd_vel /right_wheel/cmd_vel /turret/cmd_vel out.
"""
import numpy as np
import rclpy
from scipy.linalg import solve_discrete_are

from .common.controller_base import ExpControllerBase
from .common.kinematics import wrap_angle


class LqrControllerNode(ExpControllerBase):

    def __init__(self):
        super().__init__("exp_lqr_controller")

        q_pos = self.declare_parameter("q_pos", 4.0).value
        q_yaw = self.declare_parameter("q_yaw", 6.0).value
        q_int_pos = self.declare_parameter("q_int_pos", 0.05).value
        q_int_yaw = self.declare_parameter("q_int_yaw", 0.05).value
        r_u_xy = self.declare_parameter("r_u_xy", 1.0).value
        r_u_yaw = self.declare_parameter("r_u_yaw", 1.0).value
        self.int_limit_xy = self.declare_parameter("int_limit_xy", 0.5).value
        self.int_limit_yaw = self.declare_parameter("int_limit_yaw", 1.0).value

        dt = 1.0 / self.control_rate_hz
        I3, Z3 = np.eye(3), np.zeros((3, 3))
        A = np.block([[I3, Z3], [dt * I3, I3]])
        B = np.vstack([-dt * I3, Z3])
        Q = np.diag([q_pos, q_pos, q_yaw, q_int_pos, q_int_pos, q_int_yaw])
        R = np.diag([r_u_xy, r_u_xy, r_u_yaw])

        P = solve_discrete_are(A, B, Q, R)
        self.K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        self.e_int = np.zeros(3)

        self.get_logger().info(
            "LQR controller started. Gain K (3x6):\n" + np.array2string(
                self.K, precision=3, suppress_small=True))

    def compute(self, state, ref, dt):
        e = np.array([
            ref.x - state.x,
            ref.y - state.y,
            wrap_angle(ref.yaw - state.yaw_turret_world),
        ])
        z = np.concatenate([e, self.e_int])
        u = -self.K @ z
        v = np.array([ref.x_dot, ref.y_dot, ref.yaw_dot]) + u

        # Anti-windup: only integrate while the command is unsaturated, and
        # clamp the accumulated state.
        xy_sat = np.hypot(v[0], v[1]) > self.xy_dot_limit
        yaw_sat = abs(v[2]) > self.yaw_dot_limit
        if not xy_sat:
            self.e_int[0] = np.clip(self.e_int[0] + e[0] * dt,
                                    -self.int_limit_xy, self.int_limit_xy)
            self.e_int[1] = np.clip(self.e_int[1] + e[1] * dt,
                                    -self.int_limit_xy, self.int_limit_xy)
        if not yaw_sat:
            self.e_int[2] = np.clip(self.e_int[2] + e[2] * dt,
                                    -self.int_limit_yaw, self.int_limit_yaw)
        return v


def main(args=None):
    rclpy.init(args=args)
    node = LqrControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
