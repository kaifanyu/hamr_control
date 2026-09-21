"""Pure, explicitly armed execution and feedback guards for a continuous plan.

All scheduling and freshness use local monotonic seconds. Vicon source stamps
must increase, but are never compared to an unsynchronized host clock.
Stopping means reference silence: the existing controller's reference watchdog
then zeros actuators. A zero-position reference would command the origin.
"""

import math
from dataclasses import dataclass

from .continuous_waypoint import ContinuousWaypointReference

DEFAULTS = {
    'points_m_flat': [
        0.0,
        0.0,
        0.0,
        2.0,
        -2.0,
        2.0,
        -2.0,
        4.5,
        0.0,
        4.5,
        0.0,
        2.0,
        0.0,
        0.0,
    ],
    'speed_m_s': 0.15,
    'corner_deviation_m': 0.02,
    'wheel_radius_m': 0.122,
    'half_track_m': 0.350,
    'base_ahead_of_axle_m': 0.301,
    'max_wheel_rate_rad_s': 2.9321531433504737,
    'planning_wheel_rate_fraction': 0.65,
    'planning_wheel_acceleration_rad_s2': 2.0,
    'planning_max_translation_acceleration_m_s2': 0.10,
    'planning_max_jerk_m_s3': 0.20,
    'fixed_yaw_rad': 0.0,
    'origin_mode': 'start_translated_vicon',
    'odom_topic': '/HAMR_base/odom',
    'reference_timer_hz': 100.0,
    'startup_hold_s': 1.0,
    'final_hold_s': 2.0,
    'odom_timeout_s': 0.08,
    'max_publish_gap_s': 0.08,
    'odom_stable_s': 0.25,
    'odom_recovery_samples': 10,
    'start_position_tolerance_m': 0.03,
    'nominal_start_base_yaw_rad': 0.0,
    'start_yaw_tolerance_rad': 0.15,
    'start_speed_max_m_s': 0.03,
    'start_yaw_rate_max_rad_s': 0.10,
    'odom_max_position_jump_m': 0.08,
    'odom_max_yaw_jump_rad': 0.35,
    'odom_min_z_m': 0.25,
    'odom_max_z_m': 0.40,
    'odom_max_tilt_rad': 0.35,
}


def angle_difference(first, second):
    """Return the signed shortest angular difference."""
    return math.atan2(math.sin(first - second), math.cos(first - second))


def validate_config(overrides):
    """Validate hardware playback parameters, retaining conservative guards."""
    unknown = set(overrides) - set(DEFAULTS)
    if unknown:
        raise ValueError(f'Unknown continuous planner parameters: {unknown}')
    cfg = dict(DEFAULTS, **overrides)
    for name, default in DEFAULTS.items():
        if name == 'odom_recovery_samples':
            value = cfg[name]
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError('odom_recovery_samples must be an integer')
            if not 10 <= value <= 1000:
                raise ValueError('odom_recovery_samples must be in [10, 1000]')
            continue
        if isinstance(default, (float, int)):
            value = cfg[name]
            if isinstance(value, bool) or not isinstance(value, (float, int)):
                raise ValueError(f'{name} must be numeric')
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
            cfg[name] = float(value)
            if name not in ('fixed_yaw_rad', 'nominal_start_base_yaw_rad'):
                if value <= 0:
                    raise ValueError(f'{name} must be positive')
    if cfg['odom_min_z_m'] >= cfg['odom_max_z_m']:
        raise ValueError('odom_min_z_m must be below odom_max_z_m')
    if not 0 < cfg['planning_wheel_rate_fraction'] < 1:
        raise ValueError(
            'planning_wheel_rate_fraction must leave feedback room'
        )
    if cfg['max_wheel_rate_rad_s'] > DEFAULTS['max_wheel_rate_rad_s'] + 1e-10:
        raise ValueError(
            'max_wheel_rate_rad_s exceeds the 28 RPM hardware cap'
        )
    if cfg['origin_mode'] not in ('start_translated_vicon', 'vicon_absolute'):
        raise ValueError(
            'origin_mode must be start_translated_vicon or vicon_absolute'
        )
    if not isinstance(cfg['odom_topic'], str) or not cfg['odom_topic'].strip():
        raise ValueError('odom_topic must be nonempty')
    if not 20 <= cfg['reference_timer_hz'] <= 200:
        raise ValueError('reference_timer_hz must be in [20, 200]')
    if not 2 / cfg['reference_timer_hz'] <= cfg['max_publish_gap_s'] <= 0.10:
        raise ValueError(
            'max_publish_gap_s must cover 2 periods and be <= 0.10'
        )
    if cfg['odom_timeout_s'] > 0.10:
        raise ValueError(
            'odom_timeout_s must be <= 0.10 for the 0.12 s watchdog'
        )
    if cfg['startup_hold_s'] > 30 or cfg['final_hold_s'] > 30:
        raise ValueError('startup/final holds must be <= 30 s')
    if not 0.1 <= cfg['odom_stable_s'] <= 5:
        raise ValueError('odom_stable_s must be in [0.1, 5]')
    points = cfg['points_m_flat']
    if (
        not isinstance(points, (list, tuple))
        or len(points) < 4
        or len(points) % 2
    ):
        raise ValueError('points_m_flat needs at least two XY pairs')
    if any(
        isinstance(v, bool)
        or not isinstance(v, (float, int))
        or not math.isfinite(v)
        for v in points
    ):
        raise ValueError('points_m_flat must contain finite numbers')
    cfg['points_m_flat'] = [float(v) for v in points]
    return cfg


def build_plan(cfg):
    """Build the same pure planner used by the validated Gazebo runner."""
    points = cfg['points_m_flat']
    return ContinuousWaypointReference(
        list(zip(points[::2], points[1::2])),
        cfg['speed_m_s'],
        radius=cfg['wheel_radius_m'],
        half_track=cfg['half_track_m'],
        offset=cfg['base_ahead_of_axle_m'],
        max_translational_acceleration=cfg[
            'planning_max_translation_acceleration_m_s2'
        ],
        planning_wheel_rate=cfg['max_wheel_rate_rad_s']
        * cfg['planning_wheel_rate_fraction'],
        planning_wheel_acceleration=cfg['planning_wheel_acceleration_rad_s2'],
        corner_deviation=cfg['corner_deviation_m'],
        max_jerk=cfg['planning_max_jerk_m_s3'],
    )


@dataclass(frozen=True)
class Feedback:
    """One world pose with source stamp and monotonic receipt time."""

    x: float
    y: float
    yaw: float
    source_stamp_s: float
    received_s: float
    frame_id: str
    z: float = 0.30
    quaternion: tuple = (0.0, 0.0, 0.0, 1.0)


class ContinuousExecution:
    """Execute once; require explicit restart after any fault."""

    def __init__(self, cfg, plan):
        """Create an idle session with no origin or reference epoch."""
        self.cfg, self.plan = cfg, plan
        self.state, self.reason = 'idle', 'Waiting for explicit start'
        self.feedback = None
        self.recovery_samples = 0
        self.stable_since = None
        self.start_ready_since = None
        self.speed = self.yaw_rate = math.inf
        self.started_s = self.last_step_s = self.final_since_s = None
        self.translation = (0.0, 0.0)
        self.run_id = 0

    @property
    def active(self):
        """Indicate whether a service has armed this execution."""
        return self.state in ('startup_hold', 'motion', 'final_hold')

    def stop(self, reason='Stopped by operator', *, fault=False):
        """Latch silence until another successful explicit start."""
        self.state = 'aborted' if fault else 'stopped'
        self.reason = reason
        self.started_s = self.last_step_s = None

    def invalidate_feedback(self, reason):
        """Reset readiness and stop an active run on invalid feedback."""
        self.stable_since = self.start_ready_since = None
        self.recovery_samples = 0
        self.speed = self.yaw_rate = math.inf
        if self.active:
            self.stop(reason, fault=True)
        else:
            self.reason = reason

    def observe(self, sample):
        """Accept ordered poses; reject jumps and track readiness."""
        if (
            not all(
                math.isfinite(v)
                for v in (
                    sample.x,
                    sample.y,
                    sample.yaw,
                    sample.source_stamp_s,
                    sample.received_s,
                    sample.z,
                    *sample.quaternion,
                )
            )
            or sample.source_stamp_s <= 0
            or not sample.frame_id
        ):
            self.invalidate_feedback('Nonfinite or frameless odometry')
            return False
        previous = self.feedback
        accepted = True
        if previous is not None:
            dt = sample.source_stamp_s - previous.source_stamp_s
            receive_dt = sample.received_s - previous.received_s
            if dt <= 0 or receive_dt < 0:
                self.invalidate_feedback(
                    'Odometry timestamps did not increase'
                )
                return False
            position_delta = math.sqrt(
                (sample.x - previous.x) ** 2
                + (sample.y - previous.y) ** 2
                + (sample.z - previous.z) ** 2
            )
            yaw_delta = abs(angle_difference(sample.yaw, previous.yaw))
            dot = abs(
                sum(
                    a * b
                    for a, b in zip(sample.quaternion, previous.quaternion)
                )
            )
            orientation_delta = 2.0 * math.acos(min(1.0, dot))
            reason = None
            if sample.frame_id != previous.frame_id:
                reason = 'Odometry frame changed'
            elif receive_dt > self.cfg['odom_timeout_s']:
                reason = 'Odometry receipt gap exceeded timeout'
            elif dt > self.cfg['odom_timeout_s']:
                reason = 'Odometry source timestamp gap exceeded timeout'
            elif position_delta > self.cfg['odom_max_position_jump_m']:
                reason = 'Odometry position jumped'
            elif (
                max(yaw_delta, orientation_delta)
                > self.cfg['odom_max_yaw_jump_rad']
            ):
                reason = 'Odometry orientation jumped'
            if reason:
                self.invalidate_feedback(reason)
                accepted = False
            self.speed = (
                math.hypot(sample.x - previous.x, sample.y - previous.y) / dt
            )
            self.yaw_rate = yaw_delta / dt
        self.feedback = sample
        if accepted:
            self.recovery_samples += 1
        if self.stable_since is None:
            self.stable_since = sample.received_s
        stationary = (
            self.speed <= self.cfg['start_speed_max_m_s']
            and self.yaw_rate <= self.cfg['start_yaw_rate_max_rad_s']
        )
        if stationary:
            if self.start_ready_since is None:
                self.start_ready_since = sample.received_s
        else:
            self.start_ready_since = None
        return accepted

    def start(self, now_s, *, subscribers, publishers):
        """Arm a fresh run only after stationary, fresh, aligned feedback."""
        if self.active:
            return False, 'Run already active; stop before restarting'
        if publishers != 1:
            return (
                False,
                'Require this node to be the only reference publisher',
            )
        if subscribers != 1:
            return (
                False,
                'Require one hamr_controller_node reference subscriber',
            )
        sample = self.feedback
        if (
            sample is None
            or not 0 <= now_s - sample.received_s <= self.cfg['odom_timeout_s']
        ):
            return False, 'Fresh odometry is required'
        if self.recovery_samples < self.cfg['odom_recovery_samples']:
            return False, 'Require enough valid odometry recovery samples'
        if (
            self.stable_since is None
            or self.start_ready_since is None
            or now_s - self.stable_since < self.cfg['odom_stable_s']
            or now_s - self.start_ready_since < self.cfg['odom_stable_s']
        ):
            return False, 'Require stable stationary odometry before start'
        if (
            abs(
                angle_difference(
                    sample.yaw, self.cfg['nominal_start_base_yaw_rad']
                )
            )
            > self.cfg['start_yaw_tolerance_rad']
        ):
            return False, 'Base yaw is outside the start tolerance'
        first = self.plan.points[0]
        if self.cfg['origin_mode'] == 'vicon_absolute':
            if (
                math.hypot(sample.x - first[0], sample.y - first[1])
                > self.cfg['start_position_tolerance_m']
            ):
                return (
                    False,
                    'Base position is outside the absolute start tolerance',
                )
            self.translation = (0.0, 0.0)
        else:
            self.translation = (sample.x - first[0], sample.y - first[1])
        self.started_s = self.last_step_s = now_s
        self.final_since_s = None
        self.state, self.reason = 'startup_hold', 'Explicit start accepted'
        self.run_id += 1
        return True, f'Run {self.run_id} armed'

    def step(self, now_s, *, subscribers=1, publishers=1):
        """Return XY/yaw and world velocity, or None to publish nothing."""
        if not self.active:
            return None
        if (
            not math.isfinite(now_s)
            or not 0
            <= now_s - self.last_step_s
            <= self.cfg['max_publish_gap_s']
        ):
            self.stop('Reference callback gap or backwards clock', fault=True)
            return None
        sample = self.feedback
        if (
            sample is None
            or not 0 <= now_s - sample.received_s <= self.cfg['odom_timeout_s']
        ):
            self.stop('Odometry became stale', fault=True)
            return None
        if subscribers != 1 or publishers != 1:
            self.stop(
                'Reference graph ownership or subscriber lost', fault=True
            )
            return None
        self.last_step_s = now_s
        elapsed = now_s - self.started_s - self.cfg['startup_hold_s']
        if elapsed < 0:
            self.state = 'startup_hold'
            x, y = self.plan.points[0]
            vx = vy = 0.0
        elif elapsed < self.plan.duration:
            self.state = 'motion'
            x, y, vx, vy, _index = self.plan.sample(elapsed)
        else:
            if self.final_since_s is None:
                self.final_since_s = now_s
            if now_s - self.final_since_s >= self.cfg['final_hold_s']:
                self.state, self.reason = (
                    'complete',
                    'One-shot completed; reference silence',
                )
                return None
            self.state = 'final_hold'
            x, y = self.plan.points[-1]
            vx = vy = 0.0
        return (
            x + self.translation[0],
            y + self.translation[1],
            self.cfg['fixed_yaw_rad'],
            vx,
            vy,
            0.0,
        )
