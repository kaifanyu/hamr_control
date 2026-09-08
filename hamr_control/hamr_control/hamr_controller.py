import math
from dataclasses import dataclass

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Float64 # to send velocity commands
from nav_msgs.msg import Odometry # used to get the base current state (position in xyz)
from geometry_msgs.msg import PoseWithCovariance # used for reference and current pose - not using covariance rn
from geometry_msgs.msg import Quaternion # for the turret relative 
from geometry_msgs.msg import Twist # for manual mode
from tf2_msgs.msg import TFMessage # to access TFs (for turret relative angle) - could also be used for position esimation with "encoders"

from hamr_interfaces.msg import LiveGains, ReferenceTraj


HARDWARE_ODOM_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# Controller-side inverse-response compensation is intentionally constrained.
# It may add at most 50 mm/s to the requested XY feedforward and can never
# reverse or attenuate that feedforward. The lower hardware value is selected
# in the bringup YAML; these are hard runtime parameter envelopes.
XY_FEEDFORWARD_GAIN_MIN = 1.0
XY_FEEDFORWARD_GAIN_MAX = 1.5
XY_FEEDFORWARD_MAX_EXTRA_LIMIT_M_S = 0.05


### - - UTILITIES - - ###
def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def quat_to_yaw(q):
    return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

def scale_wheel_pair(omega_r, omega_l, limit_rad_s):
    """Limit both wheels with one scale so their ratio is preserved."""
    limit = float(limit_rad_s)
    peak = max(abs(float(omega_r)), abs(float(omega_l)))
    if limit <= 0.0 or peak <= limit:
        return float(omega_r), float(omega_l), 1.0
    scale = limit / peak
    return float(omega_r) * scale, float(omega_l) * scale, scale


def timestamp_is_fresh(now_ns, last_update_ns, timeout_s):
    """Return whether a received input is still safe to act on."""
    if last_update_ns is None:
        return False
    try:
        timeout = float(timeout_s)
        now_ns = int(now_ns)
        last_update_ns = int(last_update_ns)
    except (TypeError, ValueError, OverflowError):
        return False
    if not math.isfinite(timeout):
        return False
    if timeout <= 0.0:
        return True
    age_ns = now_ns - last_update_ns
    if age_ns < 0:
        return False
    return age_ns <= int(timeout * 1_000_000_000)


def reference_is_fresh(now_ns, last_reference_ns, timeout_s):
    """Return whether a received reference is still safe to act on."""
    return timestamp_is_fresh(now_ns, last_reference_ns, timeout_s)


@dataclass(frozen=True)
class SourceTimestampResult:
    accepted: bool
    reason: str = ""
    stamp_ns: int | None = None


VICON_SOURCE_STAMP_POLICY = "receipt_monotonic"


def validated_vicon_source_stamp_policy(value):
    """Return the one hardware Vicon source-stamp policy we support."""
    policy = str(value).strip().lower()
    if policy != VICON_SOURCE_STAMP_POLICY:
        raise ValueError(
            "vicon_source_stamp_policy must be "
            f"{VICON_SOURCE_STAMP_POLICY!r}"
        )
    return policy


def source_timestamp_is_valid_and_monotonic(
    stamp_sec,
    stamp_nanosec,
    previous_stamp_ns=None,
):
    """Validate source fields/order without comparing independent clocks.

    The Vicon publisher and controller run on clocks that are not synchronized,
    so their absolute difference is not a freshness signal.  Local callback
    receipt time owns freshness; the source stamp remains mandatory and
    strictly increasing so pose-delta timing cannot use a duplicate or reversed
    interval.
    """
    try:
        stamp_sec = int(stamp_sec)
        stamp_nanosec = int(stamp_nanosec)
    except (TypeError, ValueError, OverflowError):
        return SourceTimestampResult(False, "invalid header timestamp")

    if stamp_sec < 0 or not 0 <= stamp_nanosec < 1_000_000_000:
        return SourceTimestampResult(False, "invalid header timestamp")

    stamp_ns = stamp_sec * 1_000_000_000 + stamp_nanosec
    if stamp_ns == 0:
        return SourceTimestampResult(False, "zero header timestamp")

    if previous_stamp_ns is not None:
        try:
            previous = int(previous_stamp_ns)
        except (TypeError, ValueError, OverflowError):
            return SourceTimestampResult(
                False,
                "invalid previous source timestamp",
                stamp_ns,
            )
        if stamp_ns <= previous:
            return SourceTimestampResult(
                False,
                "non-monotonic source timestamp",
                stamp_ns,
            )
    return SourceTimestampResult(True, stamp_ns=stamp_ns)


def _finite_vector(values, expected_size):
    """Convert a vector-like value to finite floats, or return ``None``."""
    try:
        vector = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError):
        return None
    if len(vector) != expected_size or not all(math.isfinite(v) for v in vector):
        return None
    return vector


def time_scaled_filter_alpha(nominal_alpha, elapsed_s, nominal_period_s):
    """Scale a per-tick low-pass coefficient for the actual elapsed time."""
    try:
        alpha = max(0.0, min(1.0, float(nominal_alpha)))
        elapsed = float(elapsed_s)
        nominal_period = float(nominal_period_s)
    except (TypeError, ValueError, OverflowError):
        return 0.0
    if not math.isfinite(elapsed) or not math.isfinite(nominal_period):
        return 0.0
    if elapsed <= 0.0 or nominal_period <= 0.0 or alpha <= 0.0:
        return 0.0
    if alpha >= 1.0:
        return 1.0
    return 1.0 - (1.0 - alpha) ** (elapsed / nominal_period)


def tracking_velocity_error(reference_velocity, measured_velocity):
    """Return world-frame trajectory velocity error, or ``None`` if unknown."""
    reference = _finite_vector(reference_velocity, 2)
    measured = _finite_vector(measured_velocity, 2)
    if reference is None or measured is None:
        return None
    return reference[0] - measured[0], reference[1] - measured[1]


def validated_xy_feedforward_config(gain, max_extra_m_s):
    """Return a finite, bounded XY feedforward-compensation configuration."""
    values = _finite_vector((gain, max_extra_m_s), 2)
    if values is None:
        raise ValueError("XY feedforward parameters must be finite")
    gain, max_extra_m_s = values
    if not XY_FEEDFORWARD_GAIN_MIN <= gain <= XY_FEEDFORWARD_GAIN_MAX:
        raise ValueError(
            "xy_feedforward_gain must be in "
            f"[{XY_FEEDFORWARD_GAIN_MIN}, {XY_FEEDFORWARD_GAIN_MAX}]"
        )
    if not 0.0 <= max_extra_m_s <= XY_FEEDFORWARD_MAX_EXTRA_LIMIT_M_S:
        raise ValueError(
            "xy_feedforward_max_extra_m_s must be in [0.0, "
            f"{XY_FEEDFORWARD_MAX_EXTRA_LIMIT_M_S}]"
        )
    return gain, max_extra_m_s


def compensated_xy_feedforward(reference_velocity, gain, max_extra_m_s):
    """Apply bounded inverse-response gain to world-frame XY feedforward.

    The reference remains the control objective: P/I position error and D
    velocity error are still computed from the unmodified trajectory. Only
    the nominal plant input is increased. Clamping the *extra* as one vector
    preserves its direction, and a zero reference always produces zero
    feedforward, so there is no persistent bias or integrator windup.
    """
    reference = _finite_vector(reference_velocity, 2)
    if reference is None:
        raise ValueError("reference XY velocity must be finite")
    gain, max_extra_m_s = validated_xy_feedforward_config(
        gain, max_extra_m_s
    )

    extra_x = (gain - 1.0) * reference[0]
    extra_y = (gain - 1.0) * reference[1]
    extra_norm = math.hypot(extra_x, extra_y)
    if extra_norm > max_extra_m_s and extra_norm > 0.0:
        scale = max_extra_m_s / extra_norm
        extra_x *= scale
        extra_y *= scale
    return reference[0] + extra_x, reference[1] + extra_y


class WorldVelocityEstimator:
    """Estimate world XY velocity from consecutive accepted stamped poses."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_stamp_ns = None
        self.last_position = None
        self.velocity = None

    def _prime(self, stamp_ns, position):
        self.last_stamp_ns = int(stamp_ns)
        self.last_position = position
        self.velocity = None

    def observe(self, stamp_ns, position):
        """Observe one accepted pose and return its latest finite velocity."""
        try:
            stamp_ns = int(stamp_ns)
        except (TypeError, ValueError, OverflowError):
            self.reset()
            return None
        position = _finite_vector(position, 2)
        if stamp_ns <= 0 or position is None:
            self.reset()
            return None
        if self.last_stamp_ns is None or self.last_position is None:
            self._prime(stamp_ns, position)
            return None

        elapsed_ns = stamp_ns - self.last_stamp_ns
        if elapsed_ns <= 1_000:
            # Repeated/backward source stamps cannot establish a safe
            # derivative baseline. Re-prime at the current accepted sample.
            self._prime(stamp_ns, position)
            return None

        elapsed_s = elapsed_ns * 1e-9
        velocity = (
            (position[0] - self.last_position[0]) / elapsed_s,
            (position[1] - self.last_position[1]) / elapsed_s,
        )
        if not all(math.isfinite(value) for value in velocity):
            self._prime(stamp_ns, position)
            return None

        self.last_stamp_ns = stamp_ns
        self.last_position = position
        self.velocity = velocity
        return velocity


def normalized_quaternion(quaternion):
    """Return an ``(x, y, z, w)`` unit quaternion, or ``None`` if invalid."""
    values = _finite_vector(quaternion, 4)
    if values is None:
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        return None
    return tuple(value / norm for value in values)


def quaternion_angular_distance(first, second):
    """Shortest 3-D rotation between quaternions, treating q and -q equally."""
    q_first = normalized_quaternion(first)
    q_second = normalized_quaternion(second)
    if q_first is None or q_second is None:
        return math.inf
    # q and -q encode the same rotation, so use the absolute dot product.
    dot = abs(sum(a * b for a, b in zip(q_first, q_second)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


@dataclass(frozen=True)
class AbsolutePosePlausibilityResult:
    accepted: bool
    reason: str = ""
    z_m: float = 0.0
    tilt_rad: float = 0.0


def quaternion_tilt_rad(quaternion):
    """Return yaw-invariant body-Z versus world-Z tilt, or ``None``."""
    normalized = normalized_quaternion(quaternion)
    if normalized is None:
        return None
    x, y, _, _ = normalized
    # R_zz is the dot product between body +Z and world +Z. Unlike separate
    # Euler roll/pitch limits, this physical gravity tilt is independent of yaw.
    body_z_dot_world_z = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, body_z_dot_world_z)))


def _validated_absolute_pose_limits(
    min_z_m,
    max_z_m,
    max_tilt_rad,
):
    values = _finite_vector(
        (min_z_m, max_z_m, max_tilt_rad),
        3,
    )
    if values is None:
        raise ValueError("absolute Vicon pose limits must be finite")
    min_z_m, max_z_m, max_tilt_rad = values
    if min_z_m > max_z_m:
        raise ValueError("vicon_min_z_m must be no greater than vicon_max_z_m")
    if not 0.0 <= max_tilt_rad <= math.pi:
        raise ValueError("vicon_max_tilt_rad must be in [0, pi]")
    return values


def absolute_vicon_pose_is_plausible(
    position,
    quaternion,
    min_z_m,
    max_z_m,
    max_tilt_rad,
):
    """Check a ground-test pose against an absolute height/tilt envelope.

    The incremental guard cannot identify a stable but incorrect Vicon rigid-
    body solution at startup because there is no trusted previous sample.  This
    absolute check is intentionally independent of yaw and XY location: those
    can legitimately span the Vicon workspace, while base height and tilt are
    tightly constrained for the current flat-ground tests.
    """
    finite_position = _finite_vector(position, 3)
    tilt_rad = quaternion_tilt_rad(quaternion)
    try:
        limits = _validated_absolute_pose_limits(
            min_z_m,
            max_z_m,
            max_tilt_rad,
        )
    except ValueError as error:
        return AbsolutePosePlausibilityResult(False, str(error))

    if finite_position is None or tilt_rad is None:
        return AbsolutePosePlausibilityResult(
            False, "non-finite position or invalid quaternion"
        )

    min_z_m, max_z_m, max_tilt_rad = limits
    z_m = finite_position[2]
    reasons = []
    if not min_z_m <= z_m <= max_z_m:
        reasons.append(
            f"absolute z {z_m:.3f} m outside "
            f"[{min_z_m:.3f}, {max_z_m:.3f}] m"
        )
    # Quaternion normalization/trigonometry can put an exact boundary a few
    # machine epsilons above its configured value; keep inclusive semantics.
    if tilt_rad > max_tilt_rad + 1e-12:
        reasons.append(
            f"absolute tilt {math.degrees(tilt_rad):.1f} deg exceeds "
            f"{math.degrees(max_tilt_rad):.1f} deg"
        )
    return AbsolutePosePlausibilityResult(
        accepted=not reasons,
        reason=" and ".join(reasons),
        z_m=z_m,
        tilt_rad=tilt_rad,
    )


@dataclass(frozen=True)
class ViconPoseGuardResult:
    accepted: bool
    latched: bool
    newly_latched: bool = False
    recovered: bool = False
    reason: str = ""
    position_jump_m: float = 0.0
    orientation_jump_rad: float = 0.0
    recovery_count: int = 0


class ViconPosePlausibilityGuard:
    """Latch on an impossible absolute pose/step and require stable recovery.

    While latched, samples are compared with the last accepted pose rather than
    with one another. This prevents a consistently wrong rigid-body solution
    from becoming the new baseline. Absolute ground height and gravity tilt are
    checked even when no baseline exists. A configurable run of samples close
    to the last accepted pose and inside the absolute envelope is required
    before that baseline can advance again.
    """

    def __init__(
        self,
        max_position_jump_m,
        max_orientation_jump_rad,
        recovery_samples,
        min_z_m=0.25,
        max_z_m=0.40,
        max_tilt_rad=0.35,
    ):
        self.max_position_jump_m = 0.0
        self.max_orientation_jump_rad = 0.0
        self.recovery_samples = 1
        self.min_z_m = 0.25
        self.max_z_m = 0.40
        self.max_tilt_rad = 0.35
        self.configure(
            max_position_jump_m,
            max_orientation_jump_rad,
            recovery_samples,
            min_z_m,
            max_z_m,
            max_tilt_rad,
        )
        self.reset()

    def configure(
        self,
        max_position_jump_m=None,
        max_orientation_jump_rad=None,
        recovery_samples=None,
        min_z_m=None,
        max_z_m=None,
        max_tilt_rad=None,
    ):
        if max_position_jump_m is not None:
            self.max_position_jump_m = max(0.0, float(max_position_jump_m))
        if max_orientation_jump_rad is not None:
            self.max_orientation_jump_rad = max(
                0.0, float(max_orientation_jump_rad)
            )
        if recovery_samples is not None:
            self.recovery_samples = max(1, int(recovery_samples))
        absolute_limits = _validated_absolute_pose_limits(
            self.min_z_m if min_z_m is None else min_z_m,
            self.max_z_m if max_z_m is None else max_z_m,
            self.max_tilt_rad if max_tilt_rad is None else max_tilt_rad,
        )
        self.min_z_m, self.max_z_m, self.max_tilt_rad = absolute_limits
        if hasattr(self, "recovery_count"):
            self._clear_recovery_sequence()

    def reset(self, require_validation=False):
        self.last_position = None
        self.last_quaternion = None
        self.latched = bool(require_validation)
        self.recovery_count = 0
        self.recovery_position = None
        self.recovery_quaternion = None

    def latch(self):
        """Enter the same stable-sample recovery used for pose failures."""
        newly_latched = not self.latched
        self.latched = True
        self._clear_recovery_sequence()
        return newly_latched

    def _accept(self, position, quaternion):
        self.last_position = position
        self.last_quaternion = quaternion
        self.recovery_count = 0
        self.recovery_position = None
        self.recovery_quaternion = None

    def _clear_recovery_sequence(self):
        self.recovery_count = 0
        self.recovery_position = None
        self.recovery_quaternion = None

    def _observe_recovery_candidate(self, position, quaternion):
        """Count a sample only when the recovery sequence itself is stable."""
        if self.recovery_position is None:
            recovery_position_jump_m = 0.0
            recovery_orientation_jump_rad = 0.0
        else:
            recovery_position_jump_m = math.sqrt(
                sum(
                    (candidate - previous) ** 2
                    for candidate, previous in zip(
                        position, self.recovery_position
                    )
                )
            )
            recovery_orientation_jump_rad = quaternion_angular_distance(
                quaternion, self.recovery_quaternion
            )

        position_sane = (
            self.max_position_jump_m <= 0.0
            or recovery_position_jump_m <= self.max_position_jump_m
        )
        orientation_sane = (
            self.max_orientation_jump_rad <= 0.0
            or recovery_orientation_jump_rad
            <= self.max_orientation_jump_rad
        )
        if position_sane and orientation_sane:
            self.recovery_count += 1
        else:
            # The current sample is finite and may start a new stable run, but
            # it cannot extend the run that just made an impossible step.
            self.recovery_count = 1

        self.recovery_position = position
        self.recovery_quaternion = quaternion
        return (
            position_sane and orientation_sane,
            recovery_position_jump_m,
            recovery_orientation_jump_rad,
        )

    def observe(self, position, quaternion):
        position = _finite_vector(position, 3)
        quaternion = normalized_quaternion(quaternion)

        if position is None or quaternion is None:
            newly_latched = not self.latched
            self.latched = True
            self._clear_recovery_sequence()
            return ViconPoseGuardResult(
                accepted=False,
                latched=True,
                newly_latched=newly_latched,
                reason="non-finite position or invalid quaternion",
            )

        absolute_pose = absolute_vicon_pose_is_plausible(
            position,
            quaternion,
            self.min_z_m,
            self.max_z_m,
            self.max_tilt_rad,
        )
        if not absolute_pose.accepted:
            newly_latched = not self.latched
            self.latched = True
            self._clear_recovery_sequence()
            return ViconPoseGuardResult(
                accepted=False,
                latched=True,
                newly_latched=newly_latched,
                reason=absolute_pose.reason,
            )

        if self.last_position is None:
            if not self.latched:
                # At ordinary startup there is no previous command or bad pose
                # to recover from, so establish the initial valid baseline.
                self._accept(position, quaternion)
                return ViconPoseGuardResult(accepted=True, latched=False)

            stable, position_jump_m, orientation_jump_rad = (
                self._observe_recovery_candidate(position, quaternion)
            )
            if stable and self.recovery_count >= self.recovery_samples:
                self.latched = False
                self._accept(position, quaternion)
                return ViconPoseGuardResult(
                    accepted=True,
                    latched=False,
                    recovered=True,
                    position_jump_m=position_jump_m,
                    orientation_jump_rad=orientation_jump_rad,
                )
            return ViconPoseGuardResult(
                accepted=False,
                latched=True,
                reason=("" if stable else "unstable recovery step"),
                position_jump_m=position_jump_m,
                orientation_jump_rad=orientation_jump_rad,
                recovery_count=self.recovery_count,
            )

        position_jump_m = math.sqrt(
            sum(
                (candidate - accepted) ** 2
                for candidate, accepted in zip(position, self.last_position)
            )
        )
        orientation_jump_rad = quaternion_angular_distance(
            quaternion, self.last_quaternion
        )
        position_sane = (
            self.max_position_jump_m <= 0.0
            or position_jump_m <= self.max_position_jump_m
        )
        orientation_sane = (
            self.max_orientation_jump_rad <= 0.0
            or orientation_jump_rad <= self.max_orientation_jump_rad
        )

        if not (position_sane and orientation_sane):
            newly_latched = not self.latched
            self.latched = True
            self._clear_recovery_sequence()
            reasons = []
            if not position_sane:
                reasons.append("3-D position jump")
            if not orientation_sane:
                reasons.append("quaternion rotation jump")
            return ViconPoseGuardResult(
                accepted=False,
                latched=True,
                newly_latched=newly_latched,
                reason=" and ".join(reasons),
                position_jump_m=position_jump_m,
                orientation_jump_rad=orientation_jump_rad,
            )

        if self.latched:
            stable, recovery_position_jump_m, recovery_orientation_jump_rad = (
                self._observe_recovery_candidate(position, quaternion)
            )
            if not stable or self.recovery_count < self.recovery_samples:
                return ViconPoseGuardResult(
                    accepted=False,
                    latched=True,
                    reason=("" if stable else "unstable recovery step"),
                    position_jump_m=recovery_position_jump_m,
                    orientation_jump_rad=recovery_orientation_jump_rad,
                    recovery_count=self.recovery_count,
                )

            self.latched = False
            self._accept(position, quaternion)
            return ViconPoseGuardResult(
                accepted=True,
                latched=False,
                recovered=True,
                position_jump_m=position_jump_m,
                orientation_jump_rad=orientation_jump_rad,
            )

        self._accept(position, quaternion)
        return ViconPoseGuardResult(
            accepted=True,
            latched=False,
            position_jump_m=position_jump_m,
            orientation_jump_rad=orientation_jump_rad,
        )


def turret_state_is_ready(
    turret_enabled,
    simulating,
    turret_to_base_orientation,
    turret_to_world_orientation,
):
    """Disabled turrets need no orientation input; enabled turrets do."""
    if not turret_enabled:
        return True
    if simulating:
        return turret_to_base_orientation is not None
    return turret_to_world_orientation is not None


def apply_turret_command_policy(omegas, turret_enabled):
    """Zero an unavailable turret after the full holonomic Jacobian solve.

    The Jacobian's x/y rows have a zero turret column, so this policy does not
    change either wheel command or switch the base to differential-drive mode.
    """
    safe_omegas = np.asarray(omegas, dtype=float).copy()
    if safe_omegas.shape != (3,):
        raise ValueError("expected three joint velocity commands")
    if not turret_enabled:
        safe_omegas[2] = 0.0
    return safe_omegas


class PIAccumulator:
    def __init__(self, limit: float):
        self.sum = 0.0
        self.limit = abs(limit)

    def update(self, error: float, dt: float) -> float:
        self.sum += error * dt
        self.sum = max(-self.limit, min(self.sum, self.limit))
        return self.sum

    def reset(self):
        self.sum = 0.0        

class HamrControlNode(Node):
    def __init__(self):
        super().__init__("hamr_controller_node")

        ### - - HAMR Config params (m) - - ###
        default_hamr_config = {"r_wheel": 0.0762,
                               "a_wheel": 0.149556,
                               "b_wheel": 0.19682,
                               "base_yaw_offset": 0.0,
                               "simulating": True,
                               "mode": "auto"} # "auto" or "manual"
        for a, b in default_hamr_config.items():
            self.declare_parameter(a, b)
        self.hamr_config = {
            "r_wheel": self.get_parameter("r_wheel").value,
            "a_wheel": self.get_parameter("a_wheel").value,
            "b_wheel": self.get_parameter("b_wheel").value,
            "base_yaw_offset": self.get_parameter("base_yaw_offset").value,
            "simulating": self.get_parameter("simulating").value,
            "mode": self.get_parameter("mode").value,
        }
        
        ### - - PID Parameters for x, y and yaw - - ###
        PID_default_gains = {
            "P_x": 0.1, "I_x": 0.005, "D_x": 0.001,
            "P_y": 0.1, "I_y": 0.005, "D_y": 0.001,
            "P_yaw": 0.5, "I_yaw": 0.001, "D_yaw": 0.001,
        }
        for a, b in PID_default_gains.items():
            self.declare_parameter(a, b)
        self.gains = {
            "x": {
                "P" : self.get_parameter("P_x").value,
                "I" : self.get_parameter("I_x").value,
                "D" : self.get_parameter("D_x").value,
            },
            "y": {
                "P" : self.get_parameter("P_y").value,
                "I" : self.get_parameter("I_y").value,
                "D" : self.get_parameter("D_y").value,
            },
            "yaw": {
                "P" : self.get_parameter("P_yaw").value,
                "I" : self.get_parameter("I_yaw").value,
                "D" : self.get_parameter("D_yaw").value,
            }
        }

        self.declare_parameter("control_rate_hz", 100.0)
        self.declare_parameter("d_alpha", 0.4)
        # Compensate a repeatable loaded drivetrain under-response without
        # advancing or otherwise editing the reference trajectory. The code
        # default is neutral; hardware bringup can opt into a conservative
        # measured value. A separate extra-speed bound limits the effect even
        # if the gain is raised at runtime.
        self.declare_parameter("xy_feedforward_gain", 1.0)
        self.declare_parameter("xy_feedforward_max_extra_m_s", 0.03)
        (
            self.xy_feedforward_gain,
            self.xy_feedforward_max_extra_m_s,
        ) = validated_xy_feedforward_config(
            self.get_parameter("xy_feedforward_gain").value,
            self.get_parameter("xy_feedforward_max_extra_m_s").value,
        )
        # Odometry twist is the lowest-noise velocity source when its frame is
        # known.  The portable default derives world velocity from consecutive
        # accepted poses and their source timestamps instead.
        self.declare_parameter("xy_velocity_source", "pose_delta")
        self.xy_velocity_source = str(
            self.get_parameter("xy_velocity_source").value
        ).strip().lower()
        if self.xy_velocity_source not in (
            "odom_twist_world",
            "pose_delta",
        ):
            raise ValueError(
                "xy_velocity_source must be 'odom_twist_world' or "
                "'pose_delta'"
            )
        self.declare_parameter("turret_enabled", True)
        self.turret_enabled = bool(self.get_parameter("turret_enabled").value)
        self.declare_parameter("reference_timeout_s", 0.5)
        self.reference_timeout_s = max(
            0.0, float(self.get_parameter("reference_timeout_s").value)
        )
        self.declare_parameter("odom_timeout_s", 0.12)
        self.odom_timeout_s = max(
            0.0, float(self.get_parameter("odom_timeout_s").value)
        )
        self.declare_parameter(
            "vicon_pose_guard_enabled", not self.hamr_config["simulating"]
        )
        self.declare_parameter("vicon_max_position_jump_m", 0.08)
        self.declare_parameter("vicon_max_orientation_jump_rad", 0.35)
        self.declare_parameter("vicon_recovery_samples", 10)
        self.declare_parameter("vicon_min_z_m", 0.25)
        self.declare_parameter("vicon_max_z_m", 0.40)
        self.declare_parameter("vicon_max_tilt_rad", 0.35)
        self.declare_parameter(
            "vicon_source_stamp_policy", VICON_SOURCE_STAMP_POLICY
        )
        self.vicon_pose_guard_enabled = bool(
            self.get_parameter("vicon_pose_guard_enabled").value
        )
        self.vicon_source_stamp_policy = validated_vicon_source_stamp_policy(
            self.get_parameter("vicon_source_stamp_policy").value
        )
        self.vicon_pose_guard = ViconPosePlausibilityGuard(
            self.get_parameter("vicon_max_position_jump_m").value,
            self.get_parameter("vicon_max_orientation_jump_rad").value,
            self.get_parameter("vicon_recovery_samples").value,
            self.get_parameter("vicon_min_z_m").value,
            self.get_parameter("vicon_max_z_m").value,
            self.get_parameter("vicon_max_tilt_rad").value,
        )
        self.last_reference_time_ns = None
        self.last_odom_time_ns = None
        self.last_odom_source_stamp_ns = None
        self.reference_timed_out = False
        self.odom_timed_out = False
        # A rejected hardware Vicon sample invalidates the trajectory that was
        # active at the time of the fault.  Pose recovery alone must not reuse
        # that cached reference: the publisher must first be silent for one
        # complete reference watchdog interval and then send a new reference.
        self.vicon_motion_inhibited = False
        self.vicon_reference_rearm_ready = False
        self.vicon_inhibit_last_reference_time_ns = None
        self.waiting_for_turret_state = False
        self.pid_needs_prime = True
        self.shutdown_started = False
        # Keep published commands inside the downstream 28 RPM wheel ceiling.
        # Scaling the pair together preserves the requested turn curvature.
        self.declare_parameter("wheel_speed_limit_rad_s", 28.0 * 2.0 * math.pi / 60.0)
        self.wheel_speed_limit_rad_s = float(
            self.get_parameter("wheel_speed_limit_rad_s").value
        )
        self.last_wheel_limit_log_ns = 0
        self.last_invalid_command_log_ns = None

        self.add_on_set_parameters_callback(
            self.validate_safety_parameters_callback
        )
        self.add_post_set_parameters_callback(self.parameters_callback)

        ### - - Set Publishers and Subscribers - - ##
        self.left_wheel_vel_ = self.create_publisher(Float64, "/left_wheel/cmd_vel", 1)
        self.right_wheel_vel_ = self.create_publisher(Float64, "/right_wheel/cmd_vel", 1)
        self.turret_vel_ = self.create_publisher(Float64, "/turret/cmd_vel", 1)
        
        if self.hamr_config["simulating"]:
            self.get_logger().info("WORKING IN SIMULATION MODE")
            self.odom_sub_ = self.create_subscription(Odometry, "/hamr/odom", self.callback_odom, 1)
            self.tf_sub_ = self.create_subscription(TFMessage, "/tf", self.callback_tf, 1)
        else:
            self.get_logger().info("WORKING IN HARDWARE MODE")
            self.odom_sub_ = self.create_subscription(
                Odometry,
                "HAMR_base/odom",
                self.callback_odom,
                HARDWARE_ODOM_QOS,
            )
            self.turret_sub_ = self.create_subscription(
                Odometry,
                "HAMR_turret/odom",
                self.callback_turret_odom,
                HARDWARE_ODOM_QOS,
            )

        self.reference_sub_ = self.create_subscription(ReferenceTraj, "/reference_trajectory", 
                                    self.callback_reference, 1)
        
        # For debugging
        self.gains_pub_ = self.create_publisher(LiveGains, "/live_gains", 10)
        
        # Control Rate
        self.control_rate_hz = self.get_parameter("control_rate_hz").value
        self.last_control_time = self.get_clock().now()
        
        if self.hamr_config["mode"] == "auto":
            self.control_timer_ = self.create_timer(1.0 / self.control_rate_hz, self.control_tick)
            self.get_logger().info("Auto mode: controlling at " + str(self.control_rate_hz) + " Hz")
        elif self.hamr_config["mode"] == "manual":
            self.manual_sub_ = self.create_subscription(Twist, "/cmd_vel", 
                                        self.manual_mode_callback, 1)
            self.manual_safety_timer_ = self.create_timer(
                1.0 / self.control_rate_hz, self.manual_safety_tick
            )
            self.get_logger().info("Manual mode: listening to /cmd_vel")
            
        self.dt = 0.0

        ### - - Variables - - ###

        ## - - State Variables - - ##        
        self.pose_base_: PoseWithCovariance = None # interested in x, y, yaw
        self.reference_: ReferenceTraj = None # interested in x, y, yaw
        self.turret_to_base_orientation_: Quaternion = None  # SIMULATION: interested in yaw of turret relative to base
        self.turret_to_world_orientation_: Quaternion = None # HARDWARE: interested in yaw of turret
        self.world_velocity_estimator = WorldVelocityEstimator()
        self.measured_world_velocity = None
        self.last_xy_feedforward_extra = (0.0, 0.0)

        self.err_x_prev = 0.0
        self.err_y_prev = 0.0
        self.err_yaw_prev = 0.0

        ## - - Filtered derivatives - - ##
        self.d_err_x_filt = 0.0
        self.d_err_y_filt = 0.0
        self.d_err_yaw_filt = 0.0
        # Nominal per-control-tick coefficient. pid_step scales it for the
        # actual timer interval so smoothing is independent of timer jitter.
        self.d_alpha = self.get_parameter("d_alpha").value

        ## - - Integral Accumulators - - ##
        self.I_x = PIAccumulator(limit=.5)
        self.I_y = PIAccumulator(limit=.5)
        self.I_yaw = PIAccumulator(limit=1.0)

        ## - - Thresholds - - ##
        self.threshold_x_y = 0.02 # 2cm
        self.threshold_yaw = 0.1 # 5.7 deg

        ## - - Velocity Limits (Magnitude) - - ##
        self.xy_dot_limit = 0.8
        self.yaw_dot_limit = 2.0

        self.use_diff_drive = False  # True: ignore turret & holonomic offset

        self.get_logger().info("HAMR Controller has been started with P_x: " + str(self.gains["x"]["P"]) + 
                               ", I_x: " + str(self.gains["x"]["I"]) + ", D_x: " + str(self.gains["x"]["D"])
                                + "; P_y: " + str(self.gains["y"]["P"]) + 
                               ", I_y: " + str(self.gains["y"]["I"]) + ", D_y: " + str(self.gains["y"]["D"])
                                + "; P_yaw: " + str(self.gains["yaw"]["P"]) + ", I_yaw: " + 
                                str(self.gains["yaw"]["I"]) + ", D_yaw: " + str(self.gains["yaw"]["D"]))
        self.get_logger().info(
            "XY feedforward compensation: gain %.3f, max extra %.3f m/s"
            % (
                self.xy_feedforward_gain,
                self.xy_feedforward_max_extra_m_s,
            )
        )

    def pid_step(self):
        ''' Autonomous Mode - compute velocities based on PID Controller Logic:
            - Compute errors based on pose
            - Compute desired velocities based on (a) feed-forward (b) PID corrections from pose errors
            - Feed desired velocities to jacobian (to get joint commands)
        '''
        def compute_errors():
            ''' Find the distance error to target '''
            err_x = self.reference_.x - self.pose_base_.pose.position.x
            err_y = self.reference_.y - self.pose_base_.pose.position.y

            yaw_base_w = quat_to_yaw(self.pose_base_.pose.orientation) # raw Vicon base orientation wrt to world frame
            yaw_base_kinematic_w = wrap_angle(
                yaw_base_w + self.hamr_config["base_yaw_offset"])

            if not self.turret_enabled:
                err_yaw = 0.0
            else:
                yaw_des = self.reference_.yaw
                if self.hamr_config["simulating"]:
                    yaw_turret_b = quat_to_yaw(self.turret_to_base_orientation_)
                    yaw_turret_w = wrap_angle(yaw_base_w + yaw_turret_b)
                else:
                    yaw_turret_w = wrap_angle(
                        quat_to_yaw(self.turret_to_world_orientation_)
                    )
                err_yaw = wrap_angle(yaw_des - yaw_turret_w)

            return err_x, err_y, err_yaw, yaw_base_kinematic_w # yaw passed to jacobian later
        
        err_x, err_y, err_yaw, yaw_base_w = compute_errors()
        just_primed = self.pid_needs_prime
        if self.pid_needs_prime:
            self.err_x_prev = err_x
            self.err_y_prev = err_y
            self.err_yaw_prev = err_yaw
            self.d_err_x_filt = 0.0
            self.d_err_y_filt = 0.0
            self.d_err_yaw_filt = 0.0
            self.pid_needs_prime = False

        # Use the analytic trajectory error rate in world/Vicon coordinates.
        # Differentiating step-held position errors at the 100 Hz control tick
        # created zero/double impulses because reference and Vicon callbacks
        # arrive at different rates. The Jacobian rotates this world command
        # into joint space later.
        velocity_error = tracking_velocity_error(
            (self.reference_.x_dot, self.reference_.y_dot),
            self.measured_world_velocity,
        )
        derivative_ready = velocity_error is not None and not just_primed
        if derivative_ready:
            d_filter_alpha = time_scaled_filter_alpha(
                self.d_alpha,
                self.dt,
                1.0 / float(self.control_rate_hz),
            )
            self.d_err_x_filt += d_filter_alpha * (
                velocity_error[0] - self.d_err_x_filt
            )
            self.d_err_y_filt += d_filter_alpha * (
                velocity_error[1] - self.d_err_y_filt
            )
        else:
            self.d_err_x_filt = 0.0
            self.d_err_y_filt = 0.0
        
        # For debugging and publishing gains
        P_x = D_x = I_x_term = 0.0
        P_y = D_y = I_y_term = 0.0
        P_yaw = D_yaw = I_yaw_term = 0.0

        reference_xy_velocity = (
            self.reference_.x_dot,
            self.reference_.y_dot,
        )
        feedforward_x_dot, feedforward_y_dot = compensated_xy_feedforward(
            reference_xy_velocity,
            self.xy_feedforward_gain,
            self.xy_feedforward_max_extra_m_s,
        )
        self.last_xy_feedforward_extra = (
            feedforward_x_dot - reference_xy_velocity[0],
            feedforward_y_dot - reference_xy_velocity[1],
        )

        ## X loop
        P_x = self.gains["x"]["P"] * err_x
        if abs(err_x) < self.threshold_x_y:
            # The positional threshold is only an integrator-settling policy.
            # P and D stay continuous across it to avoid command chatter.
            self.I_x.reset()
        else:
            I_x_term = self.gains["x"]["I"] * self.I_x.update(err_x, self.dt)
        if derivative_ready:
            D_x = self.gains["x"]["D"] * self.d_err_x_filt
        desired_x_dot = feedforward_x_dot + P_x + I_x_term + D_x
        self.err_x_prev = err_x
        
        ## Y loop
        P_y = self.gains["y"]["P"] * err_y
        if abs(err_y) < self.threshold_x_y:
            self.I_y.reset()
        else:
            I_y_term = self.gains["y"]["I"] * self.I_y.update(err_y, self.dt)
        if derivative_ready:
            D_y = self.gains["y"]["D"] * self.d_err_y_filt
        desired_y_dot = feedforward_y_dot + P_y + I_y_term + D_y
        self.err_y_prev = err_y

        ## Control the XY dot NORM
        desired_xy_dot_norm = math.hypot(desired_x_dot, desired_y_dot)
        if desired_xy_dot_norm > self.xy_dot_limit:
            self.get_logger().warn("CAPPING x,y velocity from " + str(desired_xy_dot_norm) + " to " + str(self.xy_dot_limit))
            desired_x_dot = (desired_x_dot / desired_xy_dot_norm) * self.xy_dot_limit
            desired_y_dot = (desired_y_dot / desired_xy_dot_norm) * self.xy_dot_limit
        
        ## Yaw loop
        if not self.turret_enabled:
            desired_yaw_dot = 0.0
            self.err_yaw_prev = 0.0
            self.d_err_yaw_filt = 0.0
            self.I_yaw.reset()
        elif abs(err_yaw) < self.threshold_yaw:
            ## Check if at target
            desired_yaw_dot = self.reference_.yaw_dot
            self.err_yaw_prev = err_yaw
            self.d_err_yaw_filt = 0.0
            self.I_yaw.reset()
            # self.get_logger().warn("RESET I_yaw At target: " + str(self.reference_.yaw))
        else:
            P_yaw = self.gains["yaw"]["P"] * err_yaw
            I_yaw_term = self.gains["yaw"]["I"] * self.I_yaw.update(err_yaw, self.dt)

            d_raw_yaw = (err_yaw - self.err_yaw_prev) / self.dt
            self.d_err_yaw_filt = (self.d_alpha * d_raw_yaw +
                                (1.0 - self.d_alpha) * self.d_err_yaw_filt)
            D_yaw = self.gains["yaw"]["D"] * self.d_err_yaw_filt

            desired_yaw_dot = max(-self.yaw_dot_limit, min(self.reference_.yaw_dot + P_yaw + I_yaw_term + D_yaw, self.yaw_dot_limit))

            self.err_yaw_prev = err_yaw
        
        self.publish_live_gains(P_x, D_x, I_x_term, P_y, D_y, I_y_term, P_yaw, D_yaw, I_yaw_term)
        self.publish_joint_cmd(np.array([desired_x_dot, desired_y_dot, 
                                        desired_yaw_dot]), yaw_base_w) # desired vel

    def ensure_base_odom_ready(self, now_ns):
        """Enforce the Vicon latch and odometry watchdog in either mode."""
        if getattr(self, "vicon_motion_inhibited", False):
            self._update_vicon_reference_rearm(now_ns)
            self.publish_zero_cmd()
            return False
        if self.vicon_pose_guard_enabled and self.vicon_pose_guard.latched:
            self.publish_zero_cmd()
            return False
        try:
            odom_timeout_s = float(self.odom_timeout_s)
        except (TypeError, ValueError, OverflowError):
            odom_timeout_s = math.nan
        # A disabled odometry timeout is a valid legacy/simulation choice only
        # when the hardware Vicon guard is disabled. Hardware Vicon freshness
        # is deliberately based on local callback receipt time because its
        # remote source clock is not synchronized to this host.
        guarded_timeout_valid = (
            math.isfinite(odom_timeout_s) and odom_timeout_s > 0.0
        )
        odom_is_fresh = timestamp_is_fresh(
            now_ns,
            self.last_odom_time_ns,
            odom_timeout_s,
        )
        if self.vicon_pose_guard_enabled and not guarded_timeout_valid:
            odom_is_fresh = False
        if not odom_is_fresh:
            active_auto_reference = (
                getattr(self, "hamr_config", {}).get("mode") == "auto"
                and getattr(self, "reference_", None) is not None
                and reference_is_fresh(
                    now_ns,
                    getattr(self, "last_reference_time_ns", None),
                    self.reference_timeout_s,
                )
            )
            if self.vicon_pose_guard_enabled and active_auto_reference:
                # Total source silence has no rejected callback on which to
                # fail-stop. Treat the watchdog expiry as a Vicon fault while
                # an auto trajectory is active, including stable-sample pose
                # recovery and the reference quiet/new-message interlock.
                self.vicon_pose_guard.latch()
                self._latch_vicon_motion_inhibit(now_ns)
                if not self.odom_timed_out:
                    self.get_logger().error(
                        "Base odometry timed out during an active reference; "
                        "reference invalidated and Vicon motion inhibit latched"
                    )
                self.odom_timed_out = True
                return False
            if not self.odom_timed_out:
                self.get_logger().error(
                    "Base odometry is missing or stale; commanding all joints to zero"
                )
                self.reset_pid_state()
                self.odom_timed_out = True
            self.publish_zero_cmd()
            return False

        self.odom_timed_out = False
        return True

    def manual_safety_tick(self):
        """Stop a held manual command if Vicon becomes unsafe or silent."""
        self.ensure_base_odom_ready(self.get_clock().now().nanoseconds)

    def manual_mode_callback(self, msg: Twist):
        ''' Manual Mode - directly compute joint commands from terminal inputs '''
        now_ns = self.get_clock().now().nanoseconds
        if not self.ensure_base_odom_ready(now_ns):
            return

        yaw_base_w = wrap_angle(
            quat_to_yaw(self.pose_base_.pose.orientation) +
            self.hamr_config["base_yaw_offset"])
        self.publish_joint_cmd(np.array([msg.linear.x, msg.linear.y, msg.angular.z]), yaw_base_w)

    def publish_live_gains(self, P_x, D_x, I_x, 
                           P_y, D_y, I_y, 
                           P_yaw, D_yaw, I_yaw):
        gains = LiveGains()
        gains.p_x, gains.d_x, gains.i_x = P_x, D_x, I_x
        gains.p_y, gains.d_y, gains.i_y = P_y, D_y, I_y
        gains.p_yaw, gains.d_yaw, gains.i_yaw = P_yaw, D_yaw, I_yaw
        self.gains_pub_.publish(gains)

    def _latch_vicon_motion_inhibit(self, now_ns):
        """Fail-stop motion and discard the reference active at a Vicon fault.

        The quiet-period anchor is reset for every rejected Vicon observation,
        including rejected recovery samples.  References received while the
        inhibit is active are discarded separately by ``callback_reference``
        and move the same anchor forward, so a continuously publishing source
        can never rearm the controller. Auto mode can complete the handshake
        with a later trajectory reference; manual mode deliberately remains
        fail-closed after a rejected Vicon sample and requires a node restart.
        """
        newly_inhibited = not getattr(self, "vicon_motion_inhibited", False)
        self.vicon_motion_inhibited = True
        self.vicon_reference_rearm_ready = False
        self.vicon_inhibit_last_reference_time_ns = int(now_ns)
        self.reference_ = None
        self.last_reference_time_ns = None
        self.reference_timed_out = False
        # Keep this order: publish the fail-stop command before doing any state
        # bookkeeping that could grow in the future.
        self.publish_zero_cmd()
        self.reset_pid_state()
        return newly_inhibited

    def _vicon_tracking_ready_for_reference_rearm(self, now_ns):
        """Return whether a fresh, guard-accepted Vicon pose is available."""
        if not getattr(self, "vicon_pose_guard_enabled", False):
            return False
        if self.vicon_pose_guard.latched:
            return False
        return timestamp_is_fresh(
            now_ns,
            getattr(self, "last_odom_time_ns", None),
            self.odom_timeout_s,
        )

    def _update_vicon_reference_rearm(self, now_ns):
        """Arm acceptance of one new reference after safe fault recovery."""
        if not getattr(self, "vicon_motion_inhibited", False):
            return False

        # Readiness is conditional on tracking remaining valid.  In particular,
        # do not retain a ready state across a later odometry timeout.
        if not self._vicon_tracking_ready_for_reference_rearm(now_ns):
            self.vicon_reference_rearm_ready = False
            return False

        if getattr(self, "vicon_reference_rearm_ready", False):
            return True

        try:
            quiet_timeout_s = float(self.reference_timeout_s)
        except (TypeError, ValueError, OverflowError):
            return False
        # A disabled reference watchdog has no positive interval with which to
        # distinguish a stopped publisher from a continuously publishing one.
        # Fail closed instead of silently allowing an automatic restart.
        if not math.isfinite(quiet_timeout_s) or quiet_timeout_s <= 0.0:
            return False

        last_seen_ns = getattr(
            self, "vicon_inhibit_last_reference_time_ns", None
        )
        if last_seen_ns is None:
            return False
        quiet_age_ns = int(now_ns) - int(last_seen_ns)
        if quiet_age_ns < int(quiet_timeout_s * 1_000_000_000):
            return False

        self.vicon_reference_rearm_ready = True
        self.get_logger().warn(
            "Vicon tracking recovered and the reference stream was quiet; "
            "motion remains inhibited until a new reference arrives"
        )
        return True

    def callback_odom(self, msg: Odometry):
        ''' Subscription callback to the pose of hamr '''
        now_ns = self.get_clock().now().nanoseconds
        if self.vicon_pose_guard_enabled:
            source_time = source_timestamp_is_valid_and_monotonic(
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                getattr(self, "last_odom_source_stamp_ns", None),
            )
            if not source_time.accepted:
                # Re-baseline after a duplicate/reversed stamp so a subsequent
                # strictly increasing stream can complete guarded recovery.
                # PID pose-delta state is reset by the inhibit below, so a
                # lower epoch can never produce a negative timing interval.
                if source_time.stamp_ns is not None:
                    self.last_odom_source_stamp_ns = source_time.stamp_ns
                newly_latched = self.vicon_pose_guard.latch()
                # Do not pass an invalidly timed pose to the geometric guard or
                # make it look fresh by recording callback receipt time.
                self._latch_vicon_motion_inhibit(now_ns)
                if newly_latched:
                    self.get_logger().error(
                        "Rejected Vicon odometry (%s); "
                        "commanding zero until %d stable fresh samples, a "
                        "quiet reference interval, and a new reference arrive"
                        % (
                            source_time.reason,
                            self.vicon_pose_guard.recovery_samples,
                        )
                    )
                return
            self.last_odom_source_stamp_ns = source_time.stamp_ns

            pose = msg.pose.pose
            result = self.vicon_pose_guard.observe(
                (pose.position.x, pose.position.y, pose.position.z),
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
            )
            if not result.accepted:
                # Publish the stop directly from the callback; waiting for the
                # next auto timer/manual command would leave a short unsafe gap.
                self._latch_vicon_motion_inhibit(now_ns)
                if result.newly_latched:
                    self.get_logger().error(
                        "Rejected Vicon pose (%s; relative position step "
                        "%.3f m, rotation step %.1f deg); "
                        "commanding zero until %d stable samples, a quiet "
                        "reference interval, and a new reference arrive"
                        % (
                            result.reason,
                            result.position_jump_m,
                            math.degrees(result.orientation_jump_rad),
                            self.vicon_pose_guard.recovery_samples,
                        )
                    )
                return
            if result.recovered:
                self.reset_pid_state()
                self.get_logger().warn(
                    "Vicon pose recovered after stable-sample validation; "
                    "PID state re-primed, but motion remains reference-inhibited"
                )

        pose = msg.pose.pose
        source_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        if source_stamp_ns <= 0:
            # Simulation odometry sometimes omits a source stamp. Receipt time
            # remains a usable fallback when the Vicon source guard is off.
            source_stamp_ns = now_ns
        # Lazy construction also keeps callback-only safety tests and partial
        # simulation fixtures compatible with the node method.
        if not hasattr(self, "world_velocity_estimator"):
            self.world_velocity_estimator = WorldVelocityEstimator()
        pose_delta_velocity = self.world_velocity_estimator.observe(
            source_stamp_ns,
            (pose.position.x, pose.position.y),
        )
        if getattr(self, "xy_velocity_source", "pose_delta") == "odom_twist_world":
            self.measured_world_velocity = _finite_vector(
                (msg.twist.twist.linear.x, msg.twist.twist.linear.y),
                2,
            )
        else:
            self.measured_world_velocity = pose_delta_velocity
        self.pose_base_ = msg.pose
        self.last_odom_time_ns = now_ns

    def callback_turret_odom(self, msg: Odometry):
        ''' HARDWARE ONLY: Subscription callback to the turret of hamr '''
        self.turret_to_world_orientation_ = msg.pose.pose.orientation

    def callback_tf(self, msg: TFMessage):
        ''' SIMULATION ONLY: Look through all TFs and find turret_link to get it's Quaternion '''
        for t in msg.transforms:
            if t.child_frame_id == "turret_link" and t.header.frame_id  == "base_link":
                self.turret_to_base_orientation_ = t.transform.rotation # Quaternion
                break

    def control_tick(self):
        ''' Send command every (1 / control_rate_hz)[s] '''
        now = self.get_clock().now()
        dur = (now - self.last_control_time) # rclpy.duration.Duration
        self.last_control_time = now

        # A fail-stop remains active even if the ROS clock jumps or produces an
        # unusable PID interval; refreshing zero does not depend on controller
        # timing being valid.
        if getattr(self, "vicon_motion_inhibited", False):
            self._update_vicon_reference_rearm(now.nanoseconds)
            self.publish_zero_cmd()
            return

        dt = dur.nanoseconds * 1e-9
        if not math.isfinite(dt) or dt <= 0.0:
            return
        
        self.dt = max(1e-4, min(dt, 0.1))
        if not reference_is_fresh(
            now.nanoseconds,
            self.last_reference_time_ns,
            self.reference_timeout_s,
        ):
            if self.reference_ is not None and not self.reference_timed_out:
                self.get_logger().error(
                    "Reference trajectory timed out; commanding all joints to zero"
                )
                self.reset_pid_state()
                self.reference_timed_out = True
            self.publish_zero_cmd()
            return

        self.reference_timed_out = False
        if not self.ensure_base_odom_ready(now.nanoseconds):
            return

        turret_ready = turret_state_is_ready(
            self.turret_enabled,
            self.hamr_config["simulating"],
            self.turret_to_base_orientation_,
            self.turret_to_world_orientation_,
        )
        if not turret_ready:
            if not self.waiting_for_turret_state:
                self.get_logger().error(
                    "Turret feedback is missing; commanding all joints to zero"
                )
                self.reset_pid_state()
                self.waiting_for_turret_state = True
            self.publish_zero_cmd()
            return

        self.waiting_for_turret_state = False
        self.pid_step()

    def callback_reference(self, msg: ReferenceTraj):
        now_ns = self.get_clock().now().nanoseconds
        if getattr(self, "vicon_motion_inhibited", False):
            # Check the interval before accounting for this callback: if the
            # preceding interval was fully quiet and Vicon is recovered, this
            # message is the required *subsequent* new reference. Otherwise it
            # is discarded and extends the quiet-period anchor.
            if not self._update_vicon_reference_rearm(now_ns):
                self.vicon_inhibit_last_reference_time_ns = int(now_ns)
                self.reference_ = None
                self.last_reference_time_ns = None
                self.publish_zero_cmd()
                return

            self.vicon_motion_inhibited = False
            self.vicon_reference_rearm_ready = False
            self.vicon_inhibit_last_reference_time_ns = None
            self.reference_timed_out = False
            self.reset_pid_state()
            self.get_logger().warn(
                "Vicon motion inhibit cleared by a new reference after "
                "recovered tracking and a quiet reference interval"
            )

        self.reference_ = msg
        self.last_reference_time_ns = now_ns
        # self.I_x.reset()
        # self.I_y.reset()
        # self.I_yaw.reset()
        # self.get_logger().info("Going to target: " + str((msg.x, msg.y, msg.yaw)))

    def compute_velocities(self, desired_velocity, yaw):
        ''' Derived Jacobian based on dynamics - returns angular velocities for:
                1. right_wheel
                2. left_wheel
                3. turret 
        '''
        r_w, b, a = self.hamr_config["r_wheel"], \
            self.hamr_config["b_wheel"], self.hamr_config["a_wheel"]
        c, s = np.cos(yaw), np.sin(yaw)

        if self.use_diff_drive:
            xdot, ydot, yawdot = desired_velocity
            v_fwd = c * xdot + s * ydot # body-frame forward speed
            
            # standard diff-drive
            omega_r = (v_fwd + a * yawdot) / r_w
            omega_l = (v_fwd - a * yawdot) / r_w
            omega_t = 0.0
            return np.array([omega_r, omega_l, omega_t])

        J = np.array([
            [r_w/2 * (c - s*b/a), r_w/2 * (c + s*b/a), 0],
            [r_w/2 * (s + c*b/a), r_w/2 * (s - c*b/a), 0],
            [r_w/(2*a), -r_w/(2*a), 1]
        ])

        return np.linalg.solve(J, desired_velocity) # will return angular vels for joints

    def reset_pid_state(self):
        """Clear accumulated/derivative state before a safe restart."""
        self.I_x.reset()
        self.I_y.reset()
        self.I_yaw.reset()
        self.err_x_prev = 0.0
        self.err_y_prev = 0.0
        self.err_yaw_prev = 0.0
        self.d_err_x_filt = 0.0
        self.d_err_y_filt = 0.0
        self.d_err_yaw_filt = 0.0
        if hasattr(self, "world_velocity_estimator"):
            self.world_velocity_estimator.reset()
        self.measured_world_velocity = None
        self.last_xy_feedforward_extra = (0.0, 0.0)
        self.pid_needs_prime = True

    def publish_turret_zero(self):
        """Immediately stop the turret without interrupting wheel commands."""
        msg = Float64()
        msg.data = 0.0
        self.turret_vel_.publish(msg)

    def publish_zero_cmd(self):
        """Actively refresh a zero command during any unsafe state."""
        for publisher in (
            self.right_wheel_vel_,
            self.left_wheel_vel_,
            self.turret_vel_,
        ):
            msg = Float64()
            msg.data = 0.0
            publisher.publish(msg)

    def _reject_invalid_joint_command(self, reason):
        """Zero every actuator for a malformed or non-finite command."""
        now_ns = self.get_clock().now().nanoseconds
        last_log_ns = getattr(self, "last_invalid_command_log_ns", None)
        if last_log_ns is None or now_ns - last_log_ns >= 2_000_000_000:
            self.get_logger().error(
                "Rejected non-finite or malformed joint command; "
                f"publishing zero ({reason})"
            )
            self.last_invalid_command_log_ns = now_ns
        self.reset_pid_state()
        self.publish_zero_cmd()
        return False

    def publish_joint_cmd(self, desired_velocity, yaw):
        finite_velocity = _finite_vector(desired_velocity, 3)
        try:
            finite_yaw = float(yaw)
        except (TypeError, ValueError, OverflowError):
            finite_yaw = math.nan
        if finite_velocity is None or not math.isfinite(finite_yaw):
            return self._reject_invalid_joint_command(
                "invalid desired body velocity or yaw"
            )

        right_wheel_omega, left_wheel_omega, turret_omega = Float64(), Float64(), Float64()
        try:
            omegas = apply_turret_command_policy(
                self.compute_velocities(finite_velocity, finite_yaw),
                self.turret_enabled,
            )
        except (
            FloatingPointError,
            TypeError,
            ValueError,
            ZeroDivisionError,
            np.linalg.LinAlgError,
        ) as error:
            return self._reject_invalid_joint_command(
                f"joint transform failed: {error}"
            )
        if _finite_vector(omegas, 3) is None:
            return self._reject_invalid_joint_command(
                "joint transform produced a non-finite value"
            )

        omegas[0], omegas[1], wheel_scale = scale_wheel_pair(
            omegas[0], omegas[1], self.wheel_speed_limit_rad_s
        )
        if _finite_vector(omegas, 3) is None:
            return self._reject_invalid_joint_command(
                "wheel limiting produced a non-finite value"
            )
        if wheel_scale < 1.0:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_wheel_limit_log_ns >= 2_000_000_000:
                self.get_logger().warn(
                    "Pair-scaling wheel commands by %.3f to preserve curvature at %.3f rad/s"
                    % (wheel_scale, self.wheel_speed_limit_rad_s)
                )
                self.last_wheel_limit_log_ns = now_ns
        # self.get_logger().info(f"Computed omegas: {omegas}")
        right_wheel_omega.data, left_wheel_omega.data, turret_omega.data = omegas
        
        self.right_wheel_vel_.publish(right_wheel_omega)
        self.left_wheel_vel_.publish(left_wheel_omega)
        self.turret_vel_.publish(turret_omega)
        return True

    def shutdown_controller(self):
        """Stop command generation and publish explicit zeros before teardown."""
        if self.shutdown_started:
            return
        self.shutdown_started = True
        for timer_name in ("control_timer_", "manual_safety_timer_"):
            if hasattr(self, timer_name):
                getattr(self, timer_name).cancel()
        self.reset_pid_state()
        for _ in range(3):
            self.publish_zero_cmd()

    def validate_safety_parameters_callback(self, params: list[Parameter]):
        """Reject invalid bounded-controller updates before they take effect."""
        for parameter in params:
            if parameter.name == "vicon_source_stamp_policy":
                try:
                    validated_vicon_source_stamp_policy(parameter.value)
                except ValueError as error:
                    return SetParametersResult(
                        successful=False,
                        reason=str(error),
                    )

        absolute_names = {
            "vicon_min_z_m": "min_z_m",
            "vicon_max_z_m": "max_z_m",
            "vicon_max_tilt_rad": "max_tilt_rad",
        }
        proposed = {
            attribute: getattr(self.vicon_pose_guard, attribute)
            for attribute in absolute_names.values()
        }
        changed = False
        for parameter in params:
            attribute = absolute_names.get(parameter.name)
            if attribute is not None:
                proposed[attribute] = parameter.value
                changed = True
        if changed:
            try:
                _validated_absolute_pose_limits(
                    proposed["min_z_m"],
                    proposed["max_z_m"],
                    proposed["max_tilt_rad"],
                )
            except ValueError as error:
                return SetParametersResult(
                    successful=False,
                    reason=str(error),
                )

        feedforward_updates = {
            parameter.name: parameter.value
            for parameter in params
            if parameter.name in (
                "xy_feedforward_gain",
                "xy_feedforward_max_extra_m_s",
            )
        }
        if feedforward_updates:
            try:
                validated_xy_feedforward_config(
                    feedforward_updates.get(
                        "xy_feedforward_gain",
                        getattr(self, "xy_feedforward_gain", 1.0),
                    ),
                    feedforward_updates.get(
                        "xy_feedforward_max_extra_m_s",
                        getattr(
                            self,
                            "xy_feedforward_max_extra_m_s",
                            0.03,
                        ),
                    ),
                )
            except ValueError as error:
                return SetParametersResult(
                    successful=False,
                    reason=str(error),
                )
        return SetParametersResult(successful=True)

    # Used if we want to change parameter during runtime
    def parameters_callback(self, params: list[Parameter]): 
        pid_name_map = {
            "P_x": ("x", "P"),
            "I_x": ("x", "I"),
            "D_x": ("x", "D"),
            "P_y": ("y", "P"),
            "I_y": ("y", "I"),
            "D_y": ("y", "D"),
            "P_yaw":("yaw", "P"),
            "I_yaw":("yaw", "I"),
            "D_yaw":("yaw", "D"),
        }
        config_name_map = ("r_wheel", "a_wheel", "b_wheel", "base_yaw_offset")
        absolute_names = {
            "vicon_min_z_m": "min_z_m",
            "vicon_max_z_m": "max_z_m",
            "vicon_max_tilt_rad": "max_tilt_rad",
        }
        absolute_updates = {
            absolute_names[p.name]: p.value
            for p in params
            if p.name in absolute_names
        }
        absolute_revalidation_done = False
        if absolute_updates:
            proposed = {
                "min_z_m": self.vicon_pose_guard.min_z_m,
                "max_z_m": self.vicon_pose_guard.max_z_m,
                "max_tilt_rad": self.vicon_pose_guard.max_tilt_rad,
            }
            proposed.update(absolute_updates)
            try:
                validated = _validated_absolute_pose_limits(
                    proposed["min_z_m"],
                    proposed["max_z_m"],
                    proposed["max_tilt_rad"],
                )
            except ValueError as error:
                # The on-set callback normally rejects this before the post-set
                # callback runs. Fail closed if this method is invoked directly
                # or a future parameter path bypasses that validation.
                self.vicon_pose_guard.latch()
                self.pose_base_ = None
                self.last_odom_time_ns = None
                self._latch_vicon_motion_inhibit(
                    self.get_clock().now().nanoseconds
                )
                self.get_logger().error(
                    f"Invalid absolute Vicon pose limits: {error}; "
                    "controller latched at zero"
                )
                return

            changed = validated != (
                self.vicon_pose_guard.min_z_m,
                self.vicon_pose_guard.max_z_m,
                self.vicon_pose_guard.max_tilt_rad,
            )
            self.vicon_pose_guard.configure(
                min_z_m=validated[0],
                max_z_m=validated[1],
                max_tilt_rad=validated[2],
            )
            target_guard_enabled = next(
                (
                    bool(parameter.value)
                    for parameter in params
                    if parameter.name == "vicon_pose_guard_enabled"
                ),
                self.vicon_pose_guard_enabled,
            )
            if changed and target_guard_enabled:
                # Neither tightening nor widening may reuse a pose accepted
                # under a different safety contract. Stop immediately and
                # require the full stable validation run under the new limits.
                self.vicon_pose_guard.reset(require_validation=True)
                self.pose_base_ = None
                self.last_odom_time_ns = None
                self._latch_vicon_motion_inhibit(
                    self.get_clock().now().nanoseconds
                )
                absolute_revalidation_done = True

        feedforward_names = {
            "xy_feedforward_gain",
            "xy_feedforward_max_extra_m_s",
        }
        feedforward_updates = {
            parameter.name: parameter.value
            for parameter in params
            if parameter.name in feedforward_names
        }
        if feedforward_updates:
            try:
                feedforward_config = validated_xy_feedforward_config(
                    feedforward_updates.get(
                        "xy_feedforward_gain", self.xy_feedforward_gain
                    ),
                    feedforward_updates.get(
                        "xy_feedforward_max_extra_m_s",
                        self.xy_feedforward_max_extra_m_s,
                    ),
                )
            except ValueError as error:
                # The on-set callback normally rejects this first. Keep the
                # prior known-safe values if this post-set method is invoked
                # directly by a test or a future parameter path.
                self.get_logger().error(
                    f"Ignoring invalid XY feedforward configuration: {error}"
                )
            else:
                changed = feedforward_config != (
                    self.xy_feedforward_gain,
                    self.xy_feedforward_max_extra_m_s,
                )
                (
                    self.xy_feedforward_gain,
                    self.xy_feedforward_max_extra_m_s,
                ) = feedforward_config
                if changed:
                    # Discard any I/D history learned under the old plant-input
                    # model. The compensation itself is stateless.
                    self.reset_pid_state()
                self.get_logger().info(
                    "XY feedforward compensation changed to gain %.3f, "
                    "max extra %.3f m/s"
                    % feedforward_config
                )

        for p in params:
            if p.name in feedforward_names:
                continue
            if p.name in pid_name_map:
                group, term = pid_name_map[p.name]
                self.gains[group][term] = p.value
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name in config_name_map:
                self.hamr_config[p.name] = p.value
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name == "d_alpha":
                self.d_alpha = max(0.0, min(float(p.value), 1.0))
                self.get_logger().info(f"{p.name} changed to {self.d_alpha}")
            elif p.name == "xy_velocity_source":
                velocity_source = str(p.value).strip().lower()
                if velocity_source not in (
                    "odom_twist_world",
                    "pose_delta",
                ):
                    self.get_logger().error(
                        "Ignoring invalid xy_velocity_source %r" % p.value
                    )
                    continue
                if velocity_source != self.xy_velocity_source:
                    self.xy_velocity_source = velocity_source
                    self.reset_pid_state()
                self.get_logger().info(
                    f"{p.name} changed to {self.xy_velocity_source}"
                )
            elif p.name == "reference_timeout_s":
                self.reference_timeout_s = max(0.0, float(p.value))
                self.get_logger().info(
                    f"{p.name} changed to {self.reference_timeout_s}"
                )
            elif p.name == "odom_timeout_s":
                self.odom_timeout_s = max(0.0, float(p.value))
                self.get_logger().info(
                    f"{p.name} changed to {self.odom_timeout_s}"
                )
            elif p.name == "vicon_pose_guard_enabled":
                enabled = bool(p.value)
                if enabled != self.vicon_pose_guard_enabled:
                    self.vicon_pose_guard_enabled = enabled
                    # A newly enabled guard must validate fresh observations;
                    # old, previously unchecked odometry cannot remain usable.
                    if not (enabled and absolute_revalidation_done):
                        self.vicon_pose_guard.reset(require_validation=enabled)
                        self.pose_base_ = None
                        self.last_odom_time_ns = None
                        self.last_odom_source_stamp_ns = None
                        if enabled:
                            self._latch_vicon_motion_inhibit(
                                self.get_clock().now().nanoseconds
                            )
                        else:
                            self.reset_pid_state()
                            self.publish_zero_cmd()
                self.get_logger().info(
                    f"{p.name} changed to {self.vicon_pose_guard_enabled}"
                )
            elif p.name == "vicon_max_position_jump_m":
                self.vicon_pose_guard.configure(
                    max_position_jump_m=p.value
                )
                self.get_logger().info(
                    f"{p.name} changed to "
                    f"{self.vicon_pose_guard.max_position_jump_m}"
                )
            elif p.name == "vicon_max_orientation_jump_rad":
                self.vicon_pose_guard.configure(
                    max_orientation_jump_rad=p.value
                )
                self.get_logger().info(
                    f"{p.name} changed to "
                    f"{self.vicon_pose_guard.max_orientation_jump_rad}"
                )
            elif p.name == "vicon_recovery_samples":
                self.vicon_pose_guard.configure(recovery_samples=p.value)
                self.get_logger().info(
                    f"{p.name} changed to "
                    f"{self.vicon_pose_guard.recovery_samples}"
                )
            elif p.name in absolute_names:
                attribute = absolute_names[p.name]
                self.get_logger().info(
                    f"{p.name} changed to "
                    f"{getattr(self.vicon_pose_guard, attribute)}"
                )
            elif p.name == "vicon_source_stamp_policy":
                self.vicon_source_stamp_policy = (
                    validated_vicon_source_stamp_policy(p.value)
                )
                self.get_logger().info(
                    f"{p.name} changed to {self.vicon_source_stamp_policy}"
                )
            elif p.name == "turret_enabled":
                turret_enabled = bool(p.value)
                if turret_enabled != self.turret_enabled:
                    self.turret_enabled = turret_enabled
                    self.reset_pid_state()
                    self.waiting_for_turret_state = False
                    if turret_enabled:
                        # Do not reuse feedback collected while the actuator was
                        # disabled; wait for the next observation.
                        self.turret_to_base_orientation_ = None
                        self.turret_to_world_orientation_ = None
                    else:
                        self.publish_turret_zero()
                self.get_logger().info(
                    f"{p.name} changed to {self.turret_enabled}"
                )
            elif p.name == "wheel_speed_limit_rad_s":
                self.wheel_speed_limit_rad_s = max(0.0, float(p.value))
                self.get_logger().info(
                    f"{p.name} changed to {self.wheel_speed_limit_rad_s}"
                )

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HamrControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.shutdown_controller()
            except Exception as exc:
                node.get_logger().error(
                    f"Failed to publish shutdown zero commands: {exc}"
                )
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    
if __name__ == "__main__":
    main()
