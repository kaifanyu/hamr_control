#!/usr/bin/env python3
"""Publish one start-translated trajectory for repeatable HAMR studies."""

from __future__ import annotations

import json
import math
from pathlib import Path as FilePath
import re

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped

from hamr_interfaces.msg import ReferenceTraj

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String

from .trajectory_profiles import build_trajectory
from .trajectory_profiles import load_profile
from .trajectory_profiles import ORIGIN_MODE_START_TRANSLATED_VICON
from .trajectory_profiles import ORIGIN_MODE_VICON_ABSOLUTE
from .trajectory_profiles import profile_path
from .trajectory_profiles import TranslatedTrajectory
from .trajectory_profiles import validated_profile_name
from .trajectory_profiles import validated_speed_m_s
from .waypoint_traj_simple import MAX_HOLD_SECONDS
from .waypoint_traj_simple import (
    MAX_REFERENCE_TIMER_HZ,
)
from .waypoint_traj_simple import (
    MAX_SUBSCRIBER_STABLE_SECONDS,
)
from .waypoint_traj_simple import MIN_REFERENCE_TIMER_HZ
from .waypoint_traj_simple import PHASE_FINAL_HOLD
from .waypoint_traj_simple import PHASE_MOTION
from .waypoint_traj_simple import PHASE_STARTUP_HOLD
from .waypoint_traj_simple import scheduled_reference
from .waypoint_traj_simple import SubscriberReadinessGate
from .waypoint_traj_simple import TrajectoryPlayback
from .waypoint_traj_simple import validated_bounded_float
from .waypoint_traj_simple import validated_loop
from .waypoint_traj_simple import (
    validated_required_subscribers,
)


METADATA_SCHEMA_VERSION = 1
METADATA_TOPIC = '/hamr_test/trajectory_metadata'
REFERENCE_TOPIC = '/reference_trajectory'
PATH_TOPIC = '/waypoints_path'
ORIGIN_MODE = ORIGIN_MODE_START_TRANSLATED_VICON
MAX_STUDY_TEXT_LENGTH = 1000
MAX_REPETITION = 1000000
SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')
NOMINAL_START_BASE_YAW_RAD = 0.0
START_BASE_YAW_TOLERANCE_RAD = 0.15
STUDY_STACK_CONTRACT_VERSION = 1
CONTROLLER_ODOM_TIMEOUT_S = 0.0
VICON_POSE_GUARD_ENABLED = False
REFERENCE_TIMEOUT_S = 0.12
XY_VELOCITY_SOURCE = 'odom_twist_world'


def validated_study_text(name, value, *, allow_empty, maximum_length):
    """Validate a bounded operator-provided metadata string."""
    if not isinstance(value, str):
        raise ValueError(f'{name} must be a string')
    if not allow_empty and not value.strip():
        raise ValueError(f'{name} must be a nonempty string')
    if len(value) > maximum_length:
        raise ValueError(
            f'{name} must be at most {maximum_length} characters'
        )
    return value


def validated_repetition(value):
    """Return a positive bounded integer repetition index."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError('repetition must be an integer')
    if value < 1 or value > MAX_REPETITION:
        raise ValueError(f'repetition must be in [1, {MAX_REPETITION}]')
    return value


def validated_sha256(name, value):
    """Return one normalized full SHA-256 digest."""
    if not isinstance(value, str):
        raise ValueError(f'{name} must be a string')
    digest = value.strip().lower()
    if SHA256_PATTERN.fullmatch(digest) is None:
        raise ValueError(
            f'{name} must be exactly 64 hexadecimal characters'
        )
    return digest


def validated_positive_float(name, value):
    """Return a finite strictly positive numeric contract value."""
    if isinstance(value, bool):
        raise ValueError(f'{name} must be numeric, not boolean')
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f'{name} must be numeric') from error
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f'{name} must be finite and greater than zero')
    return result


def validated_boolean(name, value):
    """Return a real boolean without accepting truthy numeric values."""
    if not isinstance(value, bool):
        raise ValueError(f'{name} must be a boolean')
    return value


def validated_contract_value(name, value, expected):
    """Require one launch claim to match study-stack contract version 1."""
    if isinstance(expected, bool):
        actual = validated_boolean(name, value)
    elif isinstance(expected, int):
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f'{name} must be an integer')
        actual = value
    elif isinstance(expected, float):
        if isinstance(value, bool):
            raise ValueError(f'{name} must be numeric, not boolean')
        try:
            actual = float(value)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError(f'{name} must be numeric') from error
        if not math.isfinite(actual):
            raise ValueError(f'{name} must be finite')
    else:
        if not isinstance(value, str):
            raise ValueError(f'{name} must be a string')
        actual = value
    if actual != expected:
        raise ValueError(
            f'{name} must equal study contract value {expected!r}'
        )
    return actual


def finite_odom_xy(msg):
    """Return finite odometry XY and frame, otherwise ``None``."""
    x = float(msg.pose.pose.position.x)
    y = float(msg.pose.pose.position.y)
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    frame_id = str(msg.header.frame_id).strip() or 'odom'
    return x, y, frame_id


def finite_quaternion_yaw(orientation):
    """Return normalized quaternion yaw, or ``None`` for invalid input."""
    values = tuple(
        float(value)
        for value in (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
    )
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        return None
    x, y, z, w = (value / norm for value in values)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return yaw if math.isfinite(yaw) else None


def finite_odom_sample(msg):
    """Return finite XY, yaw, and frame provenance for one odom sample."""
    xy = finite_odom_xy(msg)
    if xy is None:
        return None
    yaw = finite_quaternion_yaw(msg.pose.pose.orientation)
    if yaw is None:
        return None
    return xy[0], xy[1], yaw, xy[2]


def place_trajectory(profile, canonical_trajectory, captured_x_m, captured_y_m):
    """Place one canonical trajectory according to its declared origin mode."""
    if profile.origin_mode == ORIGIN_MODE_START_TRANSLATED_VICON:
        origin_x = float(captured_x_m)
        origin_y = float(captured_y_m)
    elif profile.origin_mode == ORIGIN_MODE_VICON_ABSOLUTE:
        origin_x, origin_y, _yaw = canonical_trajectory.points[0]
    else:  # Loaded profiles are validated, but fail closed for test doubles.
        raise ValueError(
            f'unsupported trajectory origin mode {profile.origin_mode!r}'
        )
    return (
        TranslatedTrajectory(
            canonical_trajectory, origin_x, origin_y
        ),
        float(origin_x),
        float(origin_y),
    )


def metadata_document(
    *,
    profile,
    trajectory,
    speed_m_s,
    origin_x_m,
    origin_y_m,
    captured_start_x_m,
    captured_start_y_m,
    odom_frame_id,
    captured_start_yaw_rad,
    reference_timer_hz,
    startup_hold_s,
    final_hold_s,
    loop,
    run_id,
    planned_id,
    controller_config_sha256,
    wheel_speed_limit_rad_s,
    turret_control_enabled,
    nominal_start_base_yaw_rad,
    start_base_yaw_tolerance_rad,
    study_stack_contract_version,
    controller_odom_timeout_s,
    vicon_pose_guard_enabled,
    reference_timeout_s,
    xy_velocity_source,
    caster_type,
    terrain,
    initial_caster_orientation_deg,
    repetition,
    video_id,
    notes,
):
    """Build the stable JSON-serializable study metadata document."""
    minimum_x, maximum_x, minimum_y, maximum_y = (
        profile.bounds_offsets_m()
    )
    canonical_start_x, canonical_start_y, _ = trajectory.points[0]
    translation_x = float(origin_x_m) - canonical_start_x
    translation_y = float(origin_y_m) - canonical_start_y
    duration = float(trajectory.total_time)
    return {
        'schema_version': METADATA_SCHEMA_VERSION,
        'profile': profile.name,
        'kind': profile.kind,
        'description': profile.description,
        'profile_file': profile.source_path.name,
        'profile_sha256': profile.source_sha256,
        'profile_yaml': profile.source_yaml,
        'closed': profile.closed,
        'speed_m_s': float(speed_m_s),
        'fixed_yaw_rad': profile.fixed_yaw_rad,
        'direction': profile.direction,
        'laps': profile.laps,
        'waypoint_dwells_s': list(trajectory.waypoint_dwells_s),
        'expected_motion_phase_count': int(
            trajectory.motion_phase_count
        ),
        'expected_dwell_duration_s': float(trajectory.total_dwell_s),
        'origin_mode': profile.origin_mode,
        'origin_x_m': float(origin_x_m),
        'origin_y_m': float(origin_y_m),
        'captured_start_x_m': float(captured_start_x_m),
        'captured_start_y_m': float(captured_start_y_m),
        'odom_frame_id': odom_frame_id,
        'captured_start_yaw_rad': float(captured_start_yaw_rad),
        'expected_motion_duration_s': duration,
        'expected_total_duration_s': (
            float(startup_hold_s) + duration + float(final_hold_s)
        ),
        'startup_hold_s': float(startup_hold_s),
        'final_hold_s': float(final_hold_s),
        'reference_timer_hz': float(reference_timer_hz),
        'loop': bool(loop),
        'run_id': run_id,
        'planned_id': planned_id,
        'controller_config_sha256': controller_config_sha256,
        'wheel_speed_limit_rad_s': float(wheel_speed_limit_rad_s),
        'turret_control_enabled': bool(turret_control_enabled),
        'nominal_start_base_yaw_rad': float(
            nominal_start_base_yaw_rad
        ),
        'start_base_yaw_tolerance_rad': float(
            start_base_yaw_tolerance_rad
        ),
        'study_stack_contract_version': int(
            study_stack_contract_version
        ),
        'controller_odom_timeout_s': float(controller_odom_timeout_s),
        'vicon_pose_guard_enabled': bool(vicon_pose_guard_enabled),
        'reference_timeout_s': float(reference_timeout_s),
        'xy_velocity_source': xy_velocity_source,
        'bounds_offsets_m': {
            'min_x': minimum_x,
            'max_x': maximum_x,
            'min_y': minimum_y,
            'max_y': maximum_y,
        },
        'bounds_world_m': {
            'min_x': translation_x + minimum_x,
            'max_x': translation_x + maximum_x,
            'min_y': translation_y + minimum_y,
            'max_y': translation_y + maximum_y,
        },
        'caster_type': caster_type,
        'terrain': terrain,
        'initial_caster_orientation_deg': float(
            initial_caster_orientation_deg
        ),
        'repetition': int(repetition),
        'video_id': video_id,
        'notes': notes,
    }


def metadata_json(document):
    """Serialize metadata deterministically and reject non-finite values."""
    return json.dumps(
        document,
        allow_nan=False,
        ensure_ascii=False,
        separators=(',', ':'),
        sort_keys=True,
    )


class StudyTrajectoryNode(Node):
    """Publish a selected profile using its declared Vicon-origin policy."""

    def __init__(self):
        """Declare, validate, and connect one study trajectory run."""
        super().__init__('study_trajectory_node')

        profile_name = validated_profile_name(
            self.declare_parameter('profile', 'triangle').value
        )
        share_directory = get_package_share_directory('reference_trajectory')
        profile_directory = (
            FilePath(share_directory) / 'config' / 'trajectories'
        )
        # ``profile_path`` performs traversal-safe name validation.
        selected_path = profile_path(profile_directory, profile_name)
        self.profile = load_profile(selected_path)
        preflight_profile_sha256 = validated_sha256(
            'preflight_profile_sha256',
            self.declare_parameter(
                'preflight_profile_sha256', self.profile.source_sha256
            ).value,
        )
        if preflight_profile_sha256 != self.profile.source_sha256:
            raise ValueError(
                'preflight_profile_sha256 does not match the profile loaded '
                'by study_trajectory'
            )
        self.speed_m_s = validated_speed_m_s(
            self.declare_parameter(
                'speed_m_s', self.profile.default_speed_m_s
            ).value
        )
        self.reference_timer_hz = validated_bounded_float(
            'reference_timer_hz',
            self.declare_parameter('reference_timer_hz', 50.0).value,
            MIN_REFERENCE_TIMER_HZ,
            MAX_REFERENCE_TIMER_HZ,
        )
        self.startup_hold_s = validated_bounded_float(
            'startup_hold_s',
            self.declare_parameter('startup_hold_s', 2.0).value,
            0.0,
            MAX_HOLD_SECONDS,
        )
        self.final_hold_s = validated_bounded_float(
            'final_hold_s',
            self.declare_parameter('final_hold_s', 3.0).value,
            0.0,
            MAX_HOLD_SECONDS,
        )
        self.loop = validated_loop(
            self.declare_parameter('loop', False).value
        )
        self.required_reference_subscribers = validated_required_subscribers(
            self.declare_parameter(
                'required_reference_subscribers', 2
            ).value
        )
        self.subscriber_stable_s = validated_bounded_float(
            'subscriber_stable_s',
            self.declare_parameter('subscriber_stable_s', 1.0).value,
            0.0,
            MAX_SUBSCRIBER_STABLE_SECONDS,
        )
        self.odom_topic = validated_study_text(
            'odom_topic',
            self.declare_parameter('odom_topic', '/HAMR_base/odom').value,
            allow_empty=False,
            maximum_length=256,
        )
        self.run_id = validated_study_text(
            'run_id',
            self.declare_parameter('run_id', 'manual_study_run').value,
            allow_empty=False,
            maximum_length=256,
        )
        self.planned_id = validated_study_text(
            'planned_id',
            self.declare_parameter('planned_id', '').value,
            allow_empty=True,
            maximum_length=128,
        )
        self.controller_config_sha256 = validated_sha256(
            'controller_config_sha256',
            self.declare_parameter('controller_config_sha256').value,
        )
        self.wheel_speed_limit_rad_s = validated_positive_float(
            'wheel_speed_limit_rad_s',
            self.declare_parameter('wheel_speed_limit_rad_s').value,
        )
        self.turret_control_enabled = validated_boolean(
            'turret_control_enabled',
            self.declare_parameter('turret_control_enabled').value,
        )
        self.nominal_start_base_yaw_rad = validated_contract_value(
            'nominal_start_base_yaw_rad',
            self.declare_parameter(
                'nominal_start_base_yaw_rad',
                NOMINAL_START_BASE_YAW_RAD,
            ).value,
            NOMINAL_START_BASE_YAW_RAD,
        )
        self.start_base_yaw_tolerance_rad = validated_contract_value(
            'start_base_yaw_tolerance_rad',
            self.declare_parameter(
                'start_base_yaw_tolerance_rad',
                START_BASE_YAW_TOLERANCE_RAD,
            ).value,
            START_BASE_YAW_TOLERANCE_RAD,
        )
        self.study_stack_contract_version = validated_contract_value(
            'study_stack_contract_version',
            self.declare_parameter(
                'study_stack_contract_version',
                STUDY_STACK_CONTRACT_VERSION,
            ).value,
            STUDY_STACK_CONTRACT_VERSION,
        )
        self.controller_odom_timeout_s = validated_contract_value(
            'controller_odom_timeout_s',
            self.declare_parameter(
                'controller_odom_timeout_s',
                CONTROLLER_ODOM_TIMEOUT_S,
            ).value,
            CONTROLLER_ODOM_TIMEOUT_S,
        )
        self.vicon_pose_guard_enabled = validated_contract_value(
            'vicon_pose_guard_enabled',
            self.declare_parameter(
                'vicon_pose_guard_enabled',
                VICON_POSE_GUARD_ENABLED,
            ).value,
            VICON_POSE_GUARD_ENABLED,
        )
        self.reference_timeout_s = validated_contract_value(
            'reference_timeout_s',
            self.declare_parameter(
                'reference_timeout_s', REFERENCE_TIMEOUT_S
            ).value,
            REFERENCE_TIMEOUT_S,
        )
        self.xy_velocity_source = validated_contract_value(
            'xy_velocity_source',
            self.declare_parameter(
                'xy_velocity_source', XY_VELOCITY_SOURCE
            ).value,
            XY_VELOCITY_SOURCE,
        )
        self.caster_type = validated_study_text(
            'caster_type',
            self.declare_parameter('caster_type', 'unspecified').value,
            allow_empty=False,
            maximum_length=128,
        )
        self.terrain = validated_study_text(
            'terrain',
            self.declare_parameter('terrain', 'unspecified').value,
            allow_empty=False,
            maximum_length=128,
        )
        self.initial_caster_orientation_deg = validated_bounded_float(
            'initial_caster_orientation_deg',
            self.declare_parameter(
                'initial_caster_orientation_deg', 0.0
            ).value,
            -360.0,
            360.0,
        )
        self.repetition = validated_repetition(
            self.declare_parameter('repetition', 1).value
        )
        self.video_id = validated_study_text(
            'video_id',
            self.declare_parameter('video_id', '').value,
            allow_empty=True,
            maximum_length=256,
        )
        self.notes = validated_study_text(
            'notes',
            self.declare_parameter('notes', '').value,
            allow_empty=True,
            maximum_length=MAX_STUDY_TEXT_LENGTH,
        )

        self.canonical_trajectory = build_trajectory(
            self.profile, self.speed_m_s
        )
        self.trajectory = None
        self.latest_finite_odom = None
        self.captured_origin = None
        self.reference_origin = None
        self.begun = False
        self.last_wait_log_ns = None
        self.last_playback_phase = None
        self.last_playback_cycle = 0
        self.completion_logged = False

        self.subscriber_gate = SubscriberReadinessGate(
            self.required_reference_subscribers,
            self.subscriber_stable_s,
        )
        self.playback = TrajectoryPlayback(
            self.canonical_trajectory.total_time,
            startup_hold_s=self.startup_hold_s,
            final_hold_s=self.final_hold_s,
            loop=self.loop,
        )

        transient_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.reference_pub = self.create_publisher(
            ReferenceTraj, REFERENCE_TOPIC, 1
        )
        self.path_pub = self.create_publisher(
            Path, PATH_TOPIC, transient_qos
        )
        self.metadata_pub = self.create_publisher(
            String, METADATA_TOPIC, transient_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 1
        )
        self.reference_timer = self.create_timer(
            1.0 / self.reference_timer_hz, self.reference_update
        )

        self.get_logger().info(
            'Study profile %s (%s), %.3f m/s, %.3f s motion; waiting '
            'for %d reference subscribers stable for %.3f s and one '
            'finite Vicon XY/yaw sample on %s.'
            % (
                self.profile.name,
                self.profile.kind,
                self.speed_m_s,
                self.canonical_trajectory.total_time,
                self.required_reference_subscribers,
                self.subscriber_stable_s,
                self.odom_topic,
            )
        )

    def odom_callback(self, msg):
        """Retain only the latest finite odometry XY until start capture."""
        if self.captured_origin is not None:
            return
        sample = finite_odom_sample(msg)
        if sample is not None:
            self.latest_finite_odom = sample

    def _log_wait(self, now, message):
        if (
            self.last_wait_log_ns is None
            or now.nanoseconds - self.last_wait_log_ns >= 2_000_000_000
        ):
            self.get_logger().info(message)
            self.last_wait_log_ns = now.nanoseconds

    def _capture_start(self, now):
        subscriber_count = self.reference_pub.get_subscription_count()
        if not self.subscriber_gate.observe(
            now.nanoseconds * 1e-9, subscriber_count
        ):
            self._log_wait(
                now,
                'Waiting for reference subscribers: %d/%d; count must '
                'remain stable for %.3f s.'
                % (
                    subscriber_count,
                    self.required_reference_subscribers,
                    self.subscriber_stable_s,
                ),
            )
            return False
        if self.latest_finite_odom is None:
            self._log_wait(
                now,
                'Reference subscribers are ready; waiting for one finite '
                f'Vicon XY/yaw sample on {self.odom_topic}.',
            )
            return False

        captured_x, captured_y, start_yaw, frame_id = (
            self.latest_finite_odom
        )
        self.captured_origin = (
            captured_x, captured_y, start_yaw, frame_id
        )
        (
            self.trajectory,
            reference_origin_x,
            reference_origin_y,
        ) = place_trajectory(
            self.profile,
            self.canonical_trajectory,
            captured_x,
            captured_y,
        )
        self.reference_origin = (reference_origin_x, reference_origin_y)
        self._publish_metadata()
        self._publish_path(now)
        self.begun = True
        if self.profile.origin_mode == ORIGIN_MODE_VICON_ABSOLUTE:
            self.get_logger().info(
                'Confirmed live Vicon pose after subscriber readiness: '
                'x=%.3f, y=%.3f in %s; using absolute reference origin '
                '(%.3f, %.3f) and beginning %.3f s startup hold.'
                % (
                    captured_x,
                    captured_y,
                    frame_id,
                    reference_origin_x,
                    reference_origin_y,
                    self.startup_hold_s,
                )
            )
        else:
            self.get_logger().info(
                'Captured start XY only after subscriber readiness: '
                'x=%.3f, y=%.3f in %s; beginning %.3f s startup hold.'
                % (
                    captured_x,
                    captured_y,
                    frame_id,
                    self.startup_hold_s,
                )
            )
        return True

    def _publish_metadata(self):
        captured_x, captured_y, start_yaw, frame_id = self.captured_origin
        origin_x, origin_y = self.reference_origin
        document = metadata_document(
            profile=self.profile,
            trajectory=self.canonical_trajectory,
            speed_m_s=self.speed_m_s,
            origin_x_m=origin_x,
            origin_y_m=origin_y,
            captured_start_x_m=captured_x,
            captured_start_y_m=captured_y,
            odom_frame_id=frame_id,
            captured_start_yaw_rad=start_yaw,
            reference_timer_hz=self.reference_timer_hz,
            startup_hold_s=self.startup_hold_s,
            final_hold_s=self.final_hold_s,
            loop=self.loop,
            run_id=self.run_id,
            planned_id=self.planned_id,
            controller_config_sha256=self.controller_config_sha256,
            wheel_speed_limit_rad_s=self.wheel_speed_limit_rad_s,
            turret_control_enabled=self.turret_control_enabled,
            nominal_start_base_yaw_rad=self.nominal_start_base_yaw_rad,
            start_base_yaw_tolerance_rad=(
                self.start_base_yaw_tolerance_rad
            ),
            study_stack_contract_version=(
                self.study_stack_contract_version
            ),
            controller_odom_timeout_s=self.controller_odom_timeout_s,
            vicon_pose_guard_enabled=self.vicon_pose_guard_enabled,
            reference_timeout_s=self.reference_timeout_s,
            xy_velocity_source=self.xy_velocity_source,
            caster_type=self.caster_type,
            terrain=self.terrain,
            initial_caster_orientation_deg=(
                self.initial_caster_orientation_deg
            ),
            repetition=self.repetition,
            video_id=self.video_id,
            notes=self.notes,
        )
        message = String()
        message.data = metadata_json(document)
        self.metadata_pub.publish(message)

    def _publish_path(self, now):
        _, _, _, frame_id = self.captured_origin
        message = Path()
        message.header.frame_id = frame_id
        message.header.stamp = now.to_msg()
        for x, y, yaw in self.trajectory.points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
            pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
            message.poses.append(pose)
        self.path_pub.publish(message)

    def reference_update(self):
        """Advance the deterministic one-shot/loop playback schedule."""
        now = self.get_clock().now()
        if not self.begun and not self._capture_start(now):
            return

        try:
            playback_step = self.playback.step(now.nanoseconds * 1e-9)
        except ValueError as exc:
            self.get_logger().error(
                f'Invalid trajectory clock; stopping publication: {exc}'
            )
            self.reference_timer.cancel()
            return

        if not playback_step.publish:
            if not self.completion_logged:
                self.get_logger().info(
                    'One-shot study trajectory and final hold complete; '
                    'reference publication stopped.'
                )
                self.completion_logged = True
            self.reference_timer.cancel()
            return

        if playback_step.cycle_index != self.last_playback_cycle:
            self.get_logger().info(
                'Final hold complete; beginning trajectory cycle %d.'
                % (playback_step.cycle_index + 1)
            )
            self.last_playback_cycle = playback_step.cycle_index
        if playback_step.phase != self.last_playback_phase:
            if playback_step.phase == PHASE_STARTUP_HOLD:
                self.get_logger().info(
                    'Holding initial zero-velocity reference for %.3f s.'
                    % self.startup_hold_s
                )
            elif playback_step.phase == PHASE_MOTION:
                self.get_logger().info(
                    f'Beginning {self.profile.name} motion.'
                )
            elif playback_step.phase == PHASE_FINAL_HOLD:
                self.get_logger().info(
                    'Trajectory endpoint reached; holding final '
                    'zero-velocity reference for %.3f s.'
                    % self.final_hold_s
                )
            self.last_playback_phase = playback_step.phase

        x, y, yaw, x_dot, y_dot, yaw_dot = scheduled_reference(
            self.trajectory, playback_step
        )
        reference = ReferenceTraj()
        reference.x = float(x)
        reference.y = float(y)
        reference.yaw = float(yaw)
        reference.x_dot = float(x_dot)
        reference.y_dot = float(y_dot)
        reference.yaw_dot = float(yaw_dot)
        self.reference_pub.publish(reference)


def main(args=None):
    """Run the study trajectory publisher until the operator closes it."""
    rclpy.init(args=args)
    node = StudyTrajectoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
