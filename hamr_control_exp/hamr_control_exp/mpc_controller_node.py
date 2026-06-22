#!/usr/bin/env python3
"""Linear time-varying MPC tracking controller (OSQP).

Plant (after the shared Jacobian feedback linearization): p_{k+1} = p_k +
dt * v_k with p = [x, y, yaw_turret], v = world velocity. Decision variables
are the N velocity steps (condensed form — positions eliminated), so the
Hessian is constant and only the linear term and constraint matrix change
per solve.

  min  sum_k ||p_k - p_ref,k||^2_Q + ||v_k - v_ref,k||^2_R
              + ||v_k - v_{k-1}||^2_S
  s.t. J^-1(yaw_k) v_k  in  [-omega_max, +omega_max]   (wheel/turret rates)

yaw_k along the horizon is predicted from the reference velocity heading
(measured kinematic yaw at k=0); this only linearizes the *constraints* —
the cost is exact — and the published command is always mapped through the
Jacobian at the *measured* yaw, with the same norm caps as the PID node as
a backstop.

Timing: solves at solve_rate_hz (default 25), publishes at control_rate_hz
(default 100) by zero-order-holding the latest optimal sequence. Solver
failure or a stale plan degrades to zero commands after
stale_solution_timeout_s.

Requires the `osqp` python package:
  sudo apt install python3-osqp   (or: pip3 install --break-system-packages osqp)
"""
import math

import numpy as np
import rclpy
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from scipy import sparse

from hamr_interfaces.msg import PlannedTrajectory

from .common.controller_base import ExpControllerBase
from .common.kinematics import joint_rate_matrix, wrap_angle

LATCHED = QoSProfile(depth=1,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE)


class MpcControllerNode(ExpControllerBase):

    def __init__(self):
        super().__init__("exp_mpc_controller", auto_timer=False)
        import osqp  # deferred so the rest of the stack works without it
        self._osqp = osqp

        self.N = self.declare_parameter("horizon_steps", 25).value
        self.dt_mpc = self.declare_parameter("horizon_dt", 0.08).value
        solve_rate_hz = self.declare_parameter("solve_rate_hz", 25.0).value
        q_pos = self.declare_parameter("q_pos", 10.0).value
        q_yaw = self.declare_parameter("q_yaw", 10.0).value
        r_v = self.declare_parameter("r_v", 0.5).value
        s_dv = self.declare_parameter("s_dv", 2.0).value
        omega_wheel_max = self.declare_parameter("omega_wheel_max", 0.0).value
        omega_turret_max = self.declare_parameter("omega_turret_max", 2.0).value
        self.stale_timeout = self.declare_parameter(
            "stale_solution_timeout_s", 0.2).value

        if omega_wheel_max <= 0.0:
            omega_wheel_max = self.xy_dot_limit / self.r_wheel

        N, n = self.N, 3
        # G maps stacked velocities to stacked positions (relative to p0):
        # p_k = p0 + dt * sum_{j<k} v_j  for k = 1..N
        L = np.tril(np.ones((N, N)))
        self.G = self.dt_mpc * np.kron(L, np.eye(n))
        # D: block first-difference (v_0 gets offset b = last applied v)
        D = np.eye(N * n) - np.kron(np.eye(N, k=-1), np.eye(n))
        self.D = D

        Qbar = np.kron(np.eye(N), np.diag([q_pos, q_pos, q_yaw]))
        self.Rvec = np.tile([r_v, r_v, r_v], N)
        Sbar = np.kron(np.eye(N), np.diag([s_dv, s_dv, s_dv]))

        self.GtQ = self.G.T @ Qbar
        self.DtS = D.T @ Sbar
        P_qp = 2.0 * (self.GtQ @ self.G + np.diag(self.Rvec) + self.DtS @ D)
        self.P_csc = sparse.triu(sparse.csc_matrix(P_qp), format="csc")

        # Constraint matrix: blockdiag of dense 3x3 blocks M(yaw_k) = J^-1.
        # Built manually (explicit zeros kept) so the sparsity pattern is
        # constant and OSQP's update(Ax=...) is valid every solve.
        self._A_indices = np.concatenate(
            [np.tile(np.arange(3 * k, 3 * k + 3), 3) for k in range(N)])
        self._A_indptr = np.arange(0, 3 * (3 * N) + 1, 3)
        bound = np.tile([omega_wheel_max, omega_wheel_max, omega_turret_max], N)
        self._l, self._u = -bound, bound

        self.plan = None           # dict(t0, dt, pos (M,3), vel (M,3))
        self._sol = None           # (N,3) optimal velocity sequence
        self._sol_time = None
        self._v_last = np.zeros(3)
        self._prob = None
        self._can_update = True  # set False if this osqp build lacks update()

        self.create_subscription(PlannedTrajectory, "/planned_trajectory",
                                 self._on_plan, LATCHED)
        self.create_timer(1.0 / solve_rate_hz, self._solve_tick)
        self.create_timer(1.0 / self.control_rate_hz, self._publish_tick)
        self.get_logger().info(
            f"MPC controller started: N={N}, dt={self.dt_mpc}s "
            f"(horizon {N * self.dt_mpc:.1f}s), solve @ {solve_rate_hz} Hz, "
            f"wheel limit {omega_wheel_max:.2f} rad/s")

    # -- inputs -----------------------------------------------------------
    def _on_plan(self, msg: PlannedTrajectory):
        m = len(msg.samples)
        if m < 2 or msg.dt <= 0.0:
            self.get_logger().warn("Ignoring empty/invalid /planned_trajectory")
            return
        pos = np.array([[s.x, s.y, s.yaw] for s in msg.samples])
        vel = np.array([[s.x_dot, s.y_dot, s.yaw_dot] for s in msg.samples])
        self.plan = {
            "t0": rclpy.time.Time.from_msg(msg.header.stamp),
            "dt": msg.dt, "pos": pos, "vel": vel,
        }
        self.get_logger().info(f"Received plan: {m} samples, "
                               f"{m * msg.dt:.1f}s")

    def _sample_plan(self, t):
        """Reference pose/velocity at plan-relative time t (held past end)."""
        idx = int(round(t / self.plan["dt"]))
        idx = max(0, min(idx, len(self.plan["pos"]) - 1))
        if t > (len(self.plan["pos"]) - 1) * self.plan["dt"]:
            return self.plan["pos"][-1], np.zeros(3)
        return self.plan["pos"][idx], self.plan["vel"][idx]

    # -- solve loop ---------------------------------------------------------
    def _solve_tick(self):
        if (self.plan is None or not self.state.ready()
                or self.state.age_s() > self.watchdog_timeout_s):
            return
        state = self.state.snapshot()
        yaw_kin0 = wrap_angle(state.yaw_base + self.base_yaw_offset)
        p0 = np.array([state.x, state.y, state.yaw_turret_world])

        now = self.get_clock().now()
        t_rel = (now - self.plan["t0"]).nanoseconds * 1e-9

        N, n = self.N, 3
        pref = np.empty(N * n)
        vref = np.empty(N * n)
        yaw_pred = np.empty(N)
        prev_yaw = yaw_kin0
        for k in range(N):
            p_r, v_r = self._sample_plan(t_rel + (k + 1) * self.dt_mpc)
            # Unwrap yaw reference around the measured turret yaw
            p_r = p_r.copy()
            p_r[2] = p0[2] + wrap_angle(p_r[2] - p0[2])
            pref[3 * k:3 * k + 3] = p_r
            vref[3 * k:3 * k + 3] = v_r
            if k == 0:
                yaw_pred[k] = yaw_kin0
            elif math.hypot(v_r[0], v_r[1]) > 0.02:
                yaw_pred[k] = math.atan2(v_r[1], v_r[0])
            else:
                yaw_pred[k] = prev_yaw
            prev_yaw = yaw_pred[k]

        # Linear term (Hessian is constant)
        b = np.zeros(N * n)
        b[:3] = self._v_last
        p0_rep = np.tile(p0, N)
        q = 2.0 * (self.GtQ @ (p0_rep - pref)
                   - self.Rvec * vref
                   - self.DtS @ b)

        # Constraint blocks at the predicted yaws (column-major per block)
        A_data = np.concatenate([
            joint_rate_matrix(yaw_pred[k], self.r_wheel,
                              self.a_wheel, self.b_wheel).flatten(order="F")
            for k in range(N)])
        A_csc = sparse.csc_matrix((A_data, self._A_indices, self._A_indptr),
                                  shape=(N * n, N * n))

        t_start = self.get_clock().now()
        if self._prob is not None and self._can_update:
            try:
                self._prob.update(q=q, Ax=A_data)
            except (AttributeError, TypeError, ValueError):
                # This osqp build lacks in-place update(); re-setup instead
                # (cheap at this problem size, costs the warm start).
                self._can_update = False
                self._prob = None
        if self._prob is None or not self._can_update:
            self._prob = self._osqp.OSQP()
            self._prob.setup(P=self.P_csc, q=q, A=A_csc,
                             l=self._l, u=self._u,
                             verbose=False, warm_starting=True)

        res = self._prob.solve()
        solve_ms = (self.get_clock().now() - t_start).nanoseconds * 1e-6
        status = str(res.info.status).lower()
        if "solved" not in status:
            self.get_logger().warn(
                f"OSQP status '{res.info.status}' ({solve_ms:.1f} ms); "
                "keeping previous solution")
            return
        self._sol = np.asarray(res.x).reshape(N, n)
        self._sol_time = now
        self.get_logger().debug(f"MPC solve {solve_ms:.2f} ms")

    # -- publish loop ---------------------------------------------------------
    def _publish_tick(self):
        if self._sol is None:
            return
        age = (self.get_clock().now() - self._sol_time).nanoseconds * 1e-9
        fresh = (age < self.stale_timeout and self.state.ready()
                 and self.state.age_s() < self.watchdog_timeout_s)
        if not fresh:
            if self._active:
                self.publish_zero()
            return
        idx = max(0, min(int(age / self.dt_mpc), self.N - 1))
        v = self._sol[idx]
        self._v_last = v
        self.publish_world_velocity(v, self.state.snapshot())


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MpcControllerNode()
    except ImportError:
        print("ERROR: the 'osqp' python package is required for the MPC "
              "controller.\nInstall with: sudo apt install python3-osqp\n"
              "or: pip3 install --break-system-packages osqp")
        rclpy.shutdown()
        return
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
