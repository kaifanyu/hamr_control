import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

from hamr_interfaces.msg import LiveGains, ReferenceTraj, StateError


### - - UTILITIES - - ###
def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(x, hi))


def quat_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class PIAccumulator:
    def __init__(self, limit: float):
        self.sum = 0.0
        self.limit = abs(limit)

    def update(self, error: float, dt: float) -> float:
        self.sum += error * dt
        self.sum = clamp(self.sum, -self.limit, self.limit)
        return self.sum

    def reset(self):
        self.sum = 0.0


class HamrControlNode(Node):
    def __init__(self):
        super().__init__("hamr_controller_node")

        ### - - HAMR Config params (m) - - ###
        default_hamr_config = {
            "r_wheel": 0.0762,
            "a_wheel": 0.149556,
            "b_wheel": -0.19682,  # kept for compatibility, not used by decoupled diff-drive control
            "simulating": True,
            "mode": "auto",       # "auto" or "manual"
        }
        for name, value in default_hamr_config.items():
            self.declare_parameter(name, value)

        self.hamr_config = {
            "r_wheel": self.get_parameter("r_wheel").value,
            "a_wheel": self.get_parameter("a_wheel").value,
            "b_wheel": self.get_parameter("b_wheel").value,
            "simulating": self.get_parameter("simulating").value,
            "mode": self.get_parameter("mode").value,
        }

        ### - - Controller gains - - ###
        # In the decoupled controller:
        #   x gains   -> distance / forward-speed control
        #   y gains   -> base-heading control
        #   yaw gains -> turret-world-yaw control
        PID_default_gains = {
            "P_x": 0.1, "I_x": 0.005, "D_x": 0.001,
            "P_y": 0.1, "I_y": 0.005, "D_y": 0.001,
            "P_yaw": 0.5, "I_yaw": 0.001, "D_yaw": 0.001,
        }
        for name, value in PID_default_gains.items():
            self.declare_parameter(name, value)

        self.gains = {
            "x": {
                "P": self.get_parameter("P_x").value,
                "I": self.get_parameter("I_x").value,
                "D": self.get_parameter("D_x").value,
            },
            "y": {
                "P": self.get_parameter("P_y").value,
                "I": self.get_parameter("I_y").value,
                "D": self.get_parameter("D_y").value,
            },
            "yaw": {
                "P": self.get_parameter("P_yaw").value,
                "I": self.get_parameter("I_yaw").value,
                "D": self.get_parameter("D_yaw").value,
            },
        }

        self.declare_parameter("control_rate_hz", 100.0)
        self.declare_parameter("d_alpha", 0.4)

        # If heading error is larger than this, rotate first instead of driving forward.
        self.declare_parameter("drive_heading_gate_rad", math.radians(60.0))

        self.add_post_set_parameters_callback(self.parameters_callback)

        ### - - Publishers and Subscribers - - ###
        self.left_wheel_vel_ = self.create_publisher(Float64, "/left_wheel/cmd_vel", 1)
        self.right_wheel_vel_ = self.create_publisher(Float64, "/right_wheel/cmd_vel", 1)
        self.turret_vel_ = self.create_publisher(Float64, "/turret/cmd_vel", 1)

        if self.hamr_config["simulating"]:
            self.get_logger().info("WORKING IN SIMULATION MODE")
            self.odom_sub_ = self.create_subscription(Odometry, "/hamr/odom", self.callback_odom, 1)
            self.tf_sub_ = self.create_subscription(TFMessage, "/tf", self.callback_tf, 1)
        else:
            self.get_logger().info("WORKING IN HARDWARE MODE")
            self.odom_sub_ = self.create_subscription(Odometry, "HAMR_base/odom", self.callback_odom, 1)
            self.turret_sub_ = self.create_subscription(Odometry, "HAMR_turret/odom", self.callback_turret_odom, 1)

        self.reference_sub_ = self.create_subscription(
            ReferenceTraj,
            "/reference_trajectory",
            self.callback_reference,
            1,
        )

        # Debug publishers
        self.gains_pub_ = self.create_publisher(LiveGains, "/live_gains", 10)
        self.state_error_pub_ = self.create_publisher(StateError, "/state_error", 10)

        ### - - Timing - - ###
        self.control_rate_hz = self.get_parameter("control_rate_hz").value
        self.last_control_time = self.get_clock().now()
        self.dt = 0.0

        if self.hamr_config["mode"] == "auto":
            self.control_timer_ = self.create_timer(1.0 / self.control_rate_hz, self.control_tick)
            self.get_logger().info(f"Auto mode: controlling at {self.control_rate_hz} Hz")
        elif self.hamr_config["mode"] == "manual":
            self.manual_sub_ = self.create_subscription(Twist, "/cmd_vel", self.manual_mode_callback, 1)
            self.get_logger().info("Manual mode: listening to /cmd_vel")

        ### - - State Variables - - ###
        self.pose_base_: PoseWithCovariance = None
        self.reference_: ReferenceTraj = None
        self.turret_to_base_orientation_: Quaternion = None   # simulation: turret wrt base
        self.turret_to_world_orientation_: Quaternion = None  # hardware: turret wrt world

        ### - - Previous errors for derivative terms - - ###
        self.err_dist_prev = 0.0
        self.err_heading_prev = 0.0
        self.err_yaw_prev = 0.0

        ### - - Filtered derivatives - - ###
        self.d_err_dist_filt = 0.0
        self.d_err_heading_filt = 0.0
        self.d_err_yaw_filt = 0.0
        self.d_alpha = self.get_parameter("d_alpha").value

        ### - - Integral accumulators - - ###
        self.I_dist = PIAccumulator(limit=0.5)
        self.I_heading = PIAccumulator(limit=0.5)
        self.I_yaw = PIAccumulator(limit=1.0)

        ### - - Thresholds - - ###
        self.threshold_x_y = 0.03      # position tolerance in meters
        self.threshold_yaw = 0.1       # turret yaw tolerance in radians

        ### - - Velocity limits - - ###
        self.xy_dot_limit = 0.41       # max base forward speed in m/s
        self.yaw_dot_limit = 2.0       # max base/turret angular speed in rad/s
        self.drive_heading_gate_rad = self.get_parameter("drive_heading_gate_rad").value

        self.get_logger().info(
            "HAMR decoupled controller started. "
            f"distance gains: P={self.gains['x']['P']}, I={self.gains['x']['I']}, D={self.gains['x']['D']}; "
            f"heading gains: P={self.gains['y']['P']}, I={self.gains['y']['I']}, D={self.gains['y']['D']}; "
            f"turret yaw gains: P={self.gains['yaw']['P']}, I={self.gains['yaw']['I']}, D={self.gains['yaw']['D']}"
        )

    def get_current_yaws(self):
        """Return base yaw in world, turret yaw in world, and turret yaw relative to base when available."""
        yaw_base_w = quat_to_yaw(self.pose_base_.pose.orientation)

        if self.hamr_config["simulating"]:
            yaw_turret_b = quat_to_yaw(self.turret_to_base_orientation_)
            yaw_turret_w = wrap_angle(yaw_base_w + yaw_turret_b)
            return yaw_base_w, yaw_turret_w, yaw_turret_b

        yaw_turret_w = wrap_angle(quat_to_yaw(self.turret_to_world_orientation_))
        yaw_turret_b = wrap_angle(yaw_turret_w - yaw_base_w)
        return yaw_base_w, yaw_turret_w, yaw_turret_b

    def filtered_derivative(self, error: float, prev_error: float, prev_filtered: float):
        d_raw = (error - prev_error) / self.dt
        d_filt = self.d_alpha * d_raw + (1.0 - self.d_alpha) * prev_filtered
        return d_filt

    def pid_step(self):
        """
        Autonomous mode, decoupled control.

        Base control:
            x/y reference -> distance error + heading error
            distance error -> forward speed v
            heading error  -> base yaw rate omega_base
            v + omega_base -> right/left wheel speeds

        Turret control:
            reference yaw - turret_world_yaw -> desired turret_world_yaw_rate
            turret_motor_rate = desired_turret_world_yaw_rate - base_yaw_rate
        """
        base_x = self.pose_base_.pose.position.x
        base_y = self.pose_base_.pose.position.y
        ref_x = self.reference_.x
        ref_y = self.reference_.y

        err_x = ref_x - base_x
        err_y = ref_y - base_y
        err_dist = math.hypot(err_x, err_y)

        yaw_base_w, yaw_turret_w, _ = self.get_current_yaws()

        # Desired heading is the direction from current base position to target x/y.
        # If already at the target, keep current base heading to avoid atan2 noise.
        if err_dist > self.threshold_x_y:
            heading_des = math.atan2(err_y, err_x)
        else:
            heading_des = yaw_base_w

        err_heading = wrap_angle(heading_des - yaw_base_w)
        err_turret_yaw = wrap_angle(self.reference_.yaw - yaw_turret_w)

        # Debug terms published through the existing LiveGains message.
        # p_x/i_x/d_x now mean distance terms.
        # p_y/i_y/d_y now mean base-heading terms.
        # p_yaw/i_yaw/d_yaw mean turret-world-yaw terms.
        P_dist = I_dist_term = D_dist = 0.0
        P_heading = I_heading_term = D_heading = 0.0
        P_yaw = I_yaw_term = D_yaw = 0.0

        ### - - Base forward velocity from distance error - - ###
        if err_dist < self.threshold_x_y:
            # Stop base translation at the target.
            desired_base_v = 0.0
            self.err_dist_prev = err_dist
            self.d_err_dist_filt = 0.0
            self.I_dist.reset()
        else:
            P_dist = self.gains["x"]["P"] * err_dist
            I_dist_term = self.gains["x"]["I"] * self.I_dist.update(err_dist, self.dt)
            self.d_err_dist_filt = self.filtered_derivative(
                err_dist,
                self.err_dist_prev,
                self.d_err_dist_filt,
            )
            D_dist = self.gains["x"]["D"] * self.d_err_dist_filt

            # Optional feed-forward: project the reference world velocity onto the robot's forward axis.
            # This helps if /reference_trajectory gives x_dot/y_dot for a moving trajectory.
            ref_x_dot = getattr(self.reference_, "x_dot", 0.0)
            ref_y_dot = getattr(self.reference_, "y_dot", 0.0)
            v_ff = math.cos(yaw_base_w) * ref_x_dot + math.sin(yaw_base_w) * ref_y_dot

            desired_base_v = v_ff + P_dist + I_dist_term + D_dist

            # Do not drive forward while the base is badly misaligned.
            # This prevents sideways-error from becoming spin/arc behavior.
            if abs(err_heading) > self.drive_heading_gate_rad:
                desired_base_v = 0.0
            else:
                # Smoothly reduce forward speed when not facing the target exactly.
                desired_base_v *= max(0.0, math.cos(err_heading))

            desired_base_v = clamp(desired_base_v, -self.xy_dot_limit, self.xy_dot_limit)
            self.err_dist_prev = err_dist

        ### - - Base yaw velocity from heading error - - ###
        if err_dist < self.threshold_x_y:
            # At the target, stop turning the base. Turret still handles final yaw.
            desired_base_yaw_dot = 0.0
            self.err_heading_prev = err_heading
            self.d_err_heading_filt = 0.0
            self.I_heading.reset()
        else:
            P_heading = self.gains["y"]["P"] * err_heading
            I_heading_term = self.gains["y"]["I"] * self.I_heading.update(err_heading, self.dt)
            self.d_err_heading_filt = self.filtered_derivative(
                err_heading,
                self.err_heading_prev,
                self.d_err_heading_filt,
            )
            D_heading = self.gains["y"]["D"] * self.d_err_heading_filt

            desired_base_yaw_dot = P_heading + I_heading_term + D_heading
            desired_base_yaw_dot = clamp(desired_base_yaw_dot, -self.yaw_dot_limit, self.yaw_dot_limit)
            self.err_heading_prev = err_heading

        ### - - Turret world yaw velocity from turret yaw error - - ###
        if abs(err_turret_yaw) < self.threshold_yaw:
            desired_turret_world_yaw_dot = getattr(self.reference_, "yaw_dot", 0.0)
            self.err_yaw_prev = err_turret_yaw
            self.d_err_yaw_filt = 0.0
            self.I_yaw.reset()
        else:
            P_yaw = self.gains["yaw"]["P"] * err_turret_yaw
            I_yaw_term = self.gains["yaw"]["I"] * self.I_yaw.update(err_turret_yaw, self.dt)
            self.d_err_yaw_filt = self.filtered_derivative(
                err_turret_yaw,
                self.err_yaw_prev,
                self.d_err_yaw_filt,
            )
            D_yaw = self.gains["yaw"]["D"] * self.d_err_yaw_filt

            desired_turret_world_yaw_dot = getattr(self.reference_, "yaw_dot", 0.0) + P_yaw + I_yaw_term + D_yaw
            desired_turret_world_yaw_dot = clamp(
                desired_turret_world_yaw_dot,
                -self.yaw_dot_limit,
                self.yaw_dot_limit,
            )
            self.err_yaw_prev = err_turret_yaw

        self.publish_live_gains(
            P_dist, D_dist, I_dist_term,
            P_heading, D_heading, I_heading_term,
            P_yaw, D_yaw, I_yaw_term,
        )

        se = StateError()
        se.err_x = err_x
        se.err_y = err_y
        se.err_yaw = err_turret_yaw
        self.state_error_pub_.publish(se)

        self.publish_decoupled_joint_cmd(
            desired_base_v,
            desired_base_yaw_dot,
            desired_turret_world_yaw_dot,
        )

    def manual_mode_callback(self, msg: Twist):
        """
        Manual mode.

        msg.linear.x  -> base forward velocity in robot/body frame
        msg.angular.z -> base yaw velocity
        msg.linear.y  -> optional turret world yaw velocity command

        If you do not want manual turret motion, keep msg.linear.y = 0.
        """
        if self.pose_base_ is None:
            return

        desired_base_v = msg.linear.x
        desired_base_yaw_dot = msg.angular.z
        desired_turret_world_yaw_dot = msg.linear.y

        self.publish_decoupled_joint_cmd(
            desired_base_v,
            desired_base_yaw_dot,
            desired_turret_world_yaw_dot,
        )

    def base_to_wheel_velocities(self, desired_base_v: float, desired_base_yaw_dot: float):
        """
        Standard differential-drive inverse kinematics.

        desired_base_v       : forward velocity of the base, m/s
        desired_base_yaw_dot : yaw rate of the base, rad/s

        a_wheel is treated as half the wheel separation / yaw lever arm,
        matching your original right=(v+a*w)/r, left=(v-a*w)/r convention.
        """
        r_w = self.hamr_config["r_wheel"]
        a = self.hamr_config["a_wheel"]

        right_cmd = (desired_base_v + a * desired_base_yaw_dot) / r_w
        left_cmd = (desired_base_v - a * desired_base_yaw_dot) / r_w
        return right_cmd, left_cmd

    def publish_decoupled_joint_cmd(
        self,
        desired_base_v: float,
        desired_base_yaw_dot: float,
        desired_turret_world_yaw_dot: float,
    ):
        """
        Publish right wheel, left wheel, and turret commands.

        The turret command is relative to the base, while the yaw controller computes
        a desired turret yaw rate in the world frame. Since:

            turret_world_yaw_dot = base_yaw_dot + turret_relative_yaw_dot

        the motor command must be:

            turret_relative_yaw_dot = turret_world_yaw_dot - base_yaw_dot
        """
        right_cmd, left_cmd = self.base_to_wheel_velocities(
            desired_base_v,
            desired_base_yaw_dot,
        )

        turret_cmd = desired_turret_world_yaw_dot - desired_base_yaw_dot
        turret_cmd = clamp(turret_cmd, -self.yaw_dot_limit, self.yaw_dot_limit)

        if not self.hamr_config["simulating"]:
            # Hardware turret positive command is opposite Vicon/world yaw-positive.
            turret_cmd = -turret_cmd

        right_wheel_omega = Float64()
        left_wheel_omega = Float64()
        turret_omega = Float64()

        right_wheel_omega.data = float(right_cmd)
        left_wheel_omega.data = float(left_cmd)
        turret_omega.data = float(turret_cmd)

        self.right_wheel_vel_.publish(right_wheel_omega)
        self.left_wheel_vel_.publish(left_wheel_omega)
        self.turret_vel_.publish(turret_omega)

    def publish_live_gains(self, P_x, D_x, I_x, P_y, D_y, I_y, P_yaw, D_yaw, I_yaw):
        gains = LiveGains()
        gains.p_x, gains.d_x, gains.i_x = P_x, D_x, I_x
        gains.p_y, gains.d_y, gains.i_y = P_y, D_y, I_y
        gains.p_yaw, gains.d_yaw, gains.i_yaw = P_yaw, D_yaw, I_yaw
        self.gains_pub_.publish(gains)

    def callback_odom(self, msg: Odometry):
        self.pose_base_ = msg.pose

    def callback_turret_odom(self, msg: Odometry):
        self.turret_to_world_orientation_ = msg.pose.pose.orientation

    def callback_tf(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id == "turret_link" and t.header.frame_id == "base_link":
                self.turret_to_base_orientation_ = t.transform.rotation
                break

    def control_tick(self):
        now = self.get_clock().now()
        dur = now - self.last_control_time
        self.last_control_time = now

        dt = dur.nanoseconds * 1e-9
        if not math.isfinite(dt) or dt <= 0.0:
            return

        self.dt = clamp(dt, 1e-4, 0.1)

        turret_check = self.turret_to_base_orientation_ or self.turret_to_world_orientation_
        if self.pose_base_ is not None and self.reference_ is not None and turret_check is not None:
            self.pid_step()

    def callback_reference(self, msg: ReferenceTraj):
        self.reference_ = msg

    def parameters_callback(self, params: list[Parameter]):
        pid_name_map = {
            "P_x": ("x", "P"),
            "I_x": ("x", "I"),
            "D_x": ("x", "D"),
            "P_y": ("y", "P"),
            "I_y": ("y", "I"),
            "D_y": ("y", "D"),
            "P_yaw": ("yaw", "P"),
            "I_yaw": ("yaw", "I"),
            "D_yaw": ("yaw", "D"),
        }
        config_name_map = {
            "r_wheel",
            "a_wheel",
            "b_wheel",
            "control_rate_hz",
            "d_alpha",
            "drive_heading_gate_rad",
        }

        for p in params:
            if p.name in pid_name_map:
                group, term = pid_name_map[p.name]
                self.gains[group][term] = p.value
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name in config_name_map:
                if p.name in self.hamr_config:
                    self.hamr_config[p.name] = p.value
                elif p.name == "control_rate_hz":
                    self.control_rate_hz = p.value
                    self.get_logger().warn("control_rate_hz changed, but existing timer period is not recreated automatically")
                elif p.name == "d_alpha":
                    self.d_alpha = p.value
                elif p.name == "drive_heading_gate_rad":
                    self.drive_heading_gate_rad = p.value
                self.get_logger().info(f"{p.name} changed to {p.value}")


def main(args=None):
    rclpy.init(args=args)
    node = HamrControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()