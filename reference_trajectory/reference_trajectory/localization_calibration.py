"""Smooth, staged encoder/IMU calibration using the existing guarded executor.

The reference is translated to the starting Vicon XY, without rotating axes.
Its yaw field is turret yaw; base rotation is excited by the XY loops. Vicon
remains an independent evaluation reference for subsequent offline EKF tuning.
"""

import argparse
import bisect
import hashlib
import json
import math
import re
import sys
import time
from pathlib import Path

import numpy as np

import yaml

from .continuous_execution import build_plan as build_segment
from .continuous_execution import validate_config as validate_execution
from .continuous_waypoint_node import (
    create_node,
    plan_document as base_document,
)

EXTRA_DEFAULTS = {
    'straight_distance_m': 1.5,
    'loop_width_m': 1.5,
    'loop_height_m': 2.0,
    'slow_speed_m_s': 0.10,
    'segment_hold_s': 3.0,
    'speed_order': ['fast', 'slow'],
}


def default_config_path():
    """Find the installed/source calibration profile."""
    source = (
        Path(__file__).resolve().parents[1]
        / 'config/localization_calibration.yaml'
    )
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory

    return (
        Path(get_package_share_directory('reference_trajectory'))
        / 'config/localization_calibration.yaml'
    )


def validate_config(overrides):
    """Validate the calibration geometry and retain hardware watchdogs."""
    extras = {
        key: overrides.get(key, value) for key, value in EXTRA_DEFAULTS.items()
    }
    order = extras['speed_order']
    if (
        not isinstance(order, (list, tuple))
        or not 1 <= len(order) <= 2
        or any(value not in ('fast', 'slow') for value in order)
        or len(set(order)) != len(order)
    ):
        raise ValueError('speed_order must select fast and/or slow once each')
    extras['speed_order'] = list(order)
    for key, value in extras.items():
        if key == 'speed_order':
            continue
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f'{key} must be finite and positive')
        extras[key] = float(value)
    cfg = validate_execution(
        {
            key: value
            for key, value in overrides.items()
            if key not in EXTRA_DEFAULTS
        }
    )
    if not 0.05 <= extras['slow_speed_m_s'] < cfg['speed_m_s'] <= 0.15:
        raise ValueError('Require 0.05 <= slow_speed_m_s < speed_m_s <= 0.15')
    if any(
        not 0.5 <= extras[key] <= 3.0
        for key in ('straight_distance_m', 'loop_width_m', 'loop_height_m')
    ):
        raise ValueError('Calibration lengths must be in [0.5, 3.0] m')
    if not 1 <= extras['segment_hold_s'] <= 10:
        raise ValueError('segment_hold_s must be in [1, 10]')
    if cfg['startup_hold_s'] < 10 or cfg['final_hold_s'] < 10:
        raise ValueError(
            'Calibration needs at least 10 s initial and final holds'
        )
    if (
        cfg['origin_mode'] != 'start_translated_vicon'
        or cfg['odom_topic'] != '/HAMR_base/odom'
    ):
        raise ValueError(
            'Calibration requires start-translated raw Vicon feedback'
        )
    return dict(cfg, **extras)


def load_config(path):
    """Load the named ROS parameter mapping without starting ROS."""
    document = yaml.safe_load(Path(path).read_text())
    try:
        return validate_config(
            document['localization_calibration']['ros__parameters']
        )
    except (KeyError, TypeError) as exc:
        raise ValueError(
            'Expected localization_calibration.ros__parameters YAML'
        ) from exc


class CalibrationPlan:
    """Compose rest-to-rest bounded plans and explicit intermediate holds."""

    def __init__(self, cfg):
        """Build selected speed passes with rest between their legs."""
        self.cfg = cfg
        self.stages = []
        self.duration = 0.0
        self.length = 0.0
        self._metadata = None
        distance, width, height = (
            cfg[key]
            for key in ('straight_distance_m', 'loop_width_m', 'loop_height_m')
        )
        routes = [
            ('forward', [(0.0, 0.0), (0.0, distance)]),
            ('reverse', [(0.0, distance), (0.0, 0.0)]),
            (
                'ccw',
                [
                    (0.0, 0.0),
                    (0.0, height),
                    (-width, height),
                    (-width, 0.0),
                    (0.0, 0.0),
                ],
            ),
            (
                'cw',
                [
                    (0.0, 0.0),
                    (-width, 0.0),
                    (-width, height),
                    (0.0, height),
                    (0.0, 0.0),
                ],
            ),
        ]
        self.points = [(0.0, 0.0)]
        speeds = {'slow': cfg['slow_speed_m_s'], 'fast': cfg['speed_m_s']}
        for speed_name in cfg['speed_order']:
            speed = speeds[speed_name]
            for route_name, points in routes:
                if self.stages:
                    self._append(
                        f'hold_before_{speed_name}_{route_name}',
                        None,
                        cfg['segment_hold_s'],
                        self.points[-1],
                    )
                segment_cfg = dict(
                    cfg,
                    points_m_flat=[v for pt in points for v in pt],
                    speed_m_s=speed,
                )
                segment = build_segment(segment_cfg)
                self._append(
                    f'{speed_name}_{route_name}',
                    segment,
                    segment.duration,
                    None,
                )
                self.points.extend(points[1:])
                self.length += segment.length
        self.starts = [stage['start_time_s'] for stage in self.stages]

    def _append(self, name, plan, duration, position):
        self.stages.append(
            {
                'name': name,
                'start_time_s': self.duration,
                'duration_s': duration,
                'end_time_s': self.duration + duration,
                'plan': plan,
                'hold_position': position,
            }
        )
        self.duration += duration

    def stage_at(self, time_s):
        """Identify a stage for bag status and offline interval selection."""
        return self.stages[
            max(
                0,
                min(
                    len(self.stages) - 1,
                    bisect.bisect_right(self.starts, time_s) - 1,
                ),
            )
        ]['name']

    def sample_with_acceleration(self, time_s):
        """Sample the same analytic segments used for planning bounds."""
        if not math.isfinite(time_s):
            raise ValueError('Sample time must be finite')
        if time_s <= 0:
            return (*self.points[0], 0.0, 0.0, 0.0, 0.0, 0)
        if time_s >= self.duration:
            return (*self.points[-1], 0.0, 0.0, 0.0, 0.0, len(self.stages) - 1)
        index = bisect.bisect_right(self.starts, time_s) - 1
        stage = self.stages[index]
        if stage['plan'] is None:
            return (*stage['hold_position'], 0.0, 0.0, 0.0, 0.0, index)
        return (
            *stage['plan'].sample_with_acceleration(
                time_s - stage['start_time_s']
            )[:6],
            index,
        )

    def sample(self, time_s):
        """Match the existing guarded executor's XY reference interface."""
        x, y, vx, vy, _ax, _ay, index = self.sample_with_acceleration(time_s)
        return x, y, vx, vy, index

    def metadata(self):
        """Provide the stage schedule, footprint and nominal motion bounds."""
        if self._metadata is not None:
            return self._metadata
        samples, schedule, bounds = [], [], []
        for stage in self.stages:
            entry = {
                key: value
                for key, value in stage.items()
                if key not in ('plan', 'hold_position')
            }
            entry['start_from_run_s'] = (
                self.cfg['startup_hold_s'] + stage['start_time_s']
            )
            entry['end_from_run_s'] = (
                self.cfg['startup_hold_s'] + stage['end_time_s']
            )
            entry['kind'] = 'hold' if stage['plan'] is None else 'motion'
            if stage['plan'] is not None:
                info = stage['plan'].metadata()
                bounds.append(info['certification'])
                entry['requested_speed_m_s'] = stage['plan'].limits[
                    'requested_peak_speed_m_s'
                ]
                entry['length_m'] = stage['plan'].length
            schedule.append(entry)
        # Preview samples are at 10 Hz; 100 Hz playback remains analytic.
        times = np.unique(
            np.r_[
                np.arange(0.0, self.duration, 0.1), self.starts, self.duration
            ]
        )
        for t in times:
            state = self.sample_with_acceleration(float(t))
            samples.append(
                {
                    'time_s': float(t),
                    'position_m': list(state[:2]),
                    'velocity_m_s': list(state[2:4]),
                    'acceleration_m_s2': list(state[4:6]),
                }
            )
        certification = {
            key: max(bound[key] for bound in bounds)
            for key in (
                'max_speed_m_s',
                'max_acceleration_m_s2',
                'max_jerk_m_s3',
                'max_wheel_rate_rad_s',
                'max_wheel_acceleration_rad_s2',
            )
        }
        certification['scope'] = (
            'Nominal reference bounds; actual tracking, slip and actuator '
            'response require measured validation'
        )
        self._metadata = {
            'profile': 'localization_calibration',
            'speed_order': list(self.cfg['speed_order']),
            'duration_s': self.duration,
            'total_duration_with_holds_s': self.cfg['startup_hold_s']
            + self.duration
            + self.cfg['final_hold_s'],
            'length_m': self.length,
            'stages': schedule,
            'points_m': self.points,
            'footprint_offsets_m': {
                'x': [-self.cfg['loop_width_m'], 0.0],
                'y': [
                    0.0,
                    max(
                        self.cfg['loop_height_m'],
                        self.cfg['straight_distance_m'],
                    ),
                ],
            },
            'max_sample_spacing_m': certification['max_speed_m_s'] * 0.1,
            'certification': certification,
            'samples': samples,
        }
        return self._metadata


def plan_document(cfg, plan):
    """Record exact reference provenance and how to use the calibration bag."""
    document = base_document(cfg, plan)
    document['calibration_source_sha256'] = hashlib.sha256(
        Path(__file__).read_bytes()
    ).hexdigest()
    document['execution']['start_command'] = (
        'Invoking localization_calibration arms one run after valid '
        'stationary Vicon and controller discovery'
    )
    document['calibration'] = {
        'reference': '/HAMR_base/odom is control/evaluation ground truth, not an EKF measurement',
        'fit_order': [
            'wheel speed/turn scale and lever arm',
            'stationary gyro bias and freshness',
            'EKF measurement covariances',
            'process noise only if justified',
        ],
        'validation': (
            'Use entire held-out repeats/routes; never tune and '
            'report solely on the same run'
        ),
        'stationary_intervals': (
            'Holds are commanded zero-velocity; verify '
            'Vicon stationarity before estimating bias'
        ),
        'reverse': (
            'Reverse refers to the world XY reference; verify actual '
            'chassis/wheel reversal from measured data'
        ),
    }
    return document


def validate_runtime(cfg, snapshot):
    """Check the actual controller geometry and independent EKF inputs."""
    controller = snapshot['hamr_controller_node']
    if (
        controller.get('mode') != 'auto'
        or controller.get('simulating') is not False
    ):
        raise ValueError('Calibration requires the hardware auto controller')
    for planned, actual in [
        ('wheel_radius_m', 'r_wheel'),
        ('half_track_m', 'a_wheel'),
        ('base_ahead_of_axle_m', 'b_wheel'),
    ]:
        if not math.isclose(
            cfg[planned],
            controller.get(actual, math.nan),
            rel_tol=1e-9,
            abs_tol=1e-9,
        ):
            raise ValueError(
                f'Planner {planned} does not match controller {actual}'
            )
    if controller.get('xy_velocity_source') != 'odom_twist_world':
        raise ValueError(
            'Use the Vicon hardware controller with world-frame velocity feedback'
        )
    if not controller.get('vicon_pose_guard_enabled'):
        raise ValueError('Use the normal hardware Vicon validity guard')
    for key in ('reference_timeout_s', 'odom_timeout_s'):
        if not 0 < controller.get(key, 0) <= 0.12:
            raise ValueError(f'Use the normal hardware {key} <= 0.12 s')
    cap = controller.get('wheel_speed_limit_rad_s', 0)
    if not math.isfinite(cap) or cap < cfg['max_wheel_rate_rad_s'] - 1e-9:
        raise ValueError(
            'Planner wheel limit exceeds the actual controller limit'
        )
    ekf = snapshot['ekf_filter_node']
    sources = {
        key: value
        for key, value in ekf.items()
        if re.fullmatch(r'(odom|pose|twist|imu)\d+', key) and value
    }
    if sources != {'odom0': '/wheel_odom', 'imu0': '/imu/data'}:
        raise ValueError(
            'Calibration expects odom0=/wheel_odom and imu0=/imu/data only; '
            f'received {sources!r}. Keep Vicon independent.'
        )


def capture_runtime_parameters(node, timeout_s=15.0):
    """Snapshot effective parameters before any reference is published."""
    from rcl_interfaces.msg import ParameterType
    import rclpy
    from rclpy.parameter import parameter_value_to_python
    from rclpy.parameter_client import AsyncParameterClient

    names = (
        'hamr_controller_node',
        'ekf_filter_node',
        'holonomic_odom_node',
        'hamr_uros_bridge',
    )
    clients = {name: AsyncParameterClient(node, name) for name in names}
    pending, parameter_names, unset_parameters, snapshot = {}, {}, {}, {}
    deadline = time.monotonic() + timeout_s
    while len(snapshot) < len(clients) and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        if node.execution.state != 'idle':
            raise RuntimeError(
                'Calibration stopped before parameter capture completed'
            )
        for name, client in clients.items():
            if name in snapshot:
                continue
            if name not in pending and client.services_are_ready():
                pending[name] = ('names', client.list_parameters(depth=0))
            phase, future = pending.get(name, (None, None))
            if future is None or not future.done():
                continue
            if phase == 'names':
                parameter_names[name] = future.result().result.names
                pending[name] = (
                    'types',
                    client.get_parameter_types(parameter_names[name]),
                )
            elif phase == 'types':
                types = future.result().types
                names_for_node = parameter_names[name]
                if len(types) != len(names_for_node):
                    raise RuntimeError(
                        f'Incomplete parameter type response from {name}'
                    )
                # robot_localization declares unused sensor slots without
                # initializing them. rclcpp returns an empty GetParameters
                # response for the entire request if any entry is unset.
                # Query types first and record unset entries as JSON null.
                unset_parameters[name] = {
                    key: None
                    for key, kind in zip(names_for_node, types)
                    if kind == ParameterType.PARAMETER_NOT_SET
                }
                parameter_names[name] = [
                    key
                    for key in names_for_node
                    if key not in unset_parameters[name]
                ]
                pending[name] = (
                    'values',
                    client.get_parameters(parameter_names[name]),
                )
            else:
                values = future.result().values
                if len(values) != len(parameter_names[name]):
                    raise RuntimeError(
                        f'Incomplete parameter value response from {name}: '
                        f'expected {len(parameter_names[name])}, '
                        f'received {len(values)}; retry parameter capture'
                    )
                snapshot[name] = dict(unset_parameters[name])
                snapshot[name].update(
                    zip(
                        parameter_names[name],
                        [parameter_value_to_python(value) for value in values],
                    )
                )
        if time.monotonic() > deadline:
            raise RuntimeError(
                'Could not capture running parameters from: '
                + ', '.join(set(names) - set(snapshot))
            )
    validate_runtime(node.cfg, snapshot)
    return snapshot


def main(args=None):
    """Preview without ROS, or arm one calibration after hardware discovery."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(default_config_path()))
    parser.add_argument(
        '--speed-set',
        choices=('fast', 'slow', 'both'),
        help='Collect one speed, or both with fast first; overrides config order',
    )
    parser.add_argument('--plan-only', metavar='JSON')
    parsed, ros_args = parser.parse_known_args(
        sys.argv[1:] if args is None else args
    )
    cfg = load_config(parsed.config)
    if parsed.speed_set:
        order = (
            ['fast', 'slow']
            if parsed.speed_set == 'both'
            else [parsed.speed_set]
        )
        cfg = validate_config(dict(cfg, speed_order=order))
    if parsed.plan_only:
        if ros_args:
            parser.error('Use --config for plan-only parameters')
        plan = CalibrationPlan(cfg)
        output = Path(parsed.plan_only).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(plan_document(cfg, plan), indent=2, allow_nan=False)
            + '\n'
        )
        print(
            f'Wrote {output}: '
            f'{plan.metadata()["total_duration_with_holds_s"]:.1f} s '
            f'including holds; passes={" -> ".join(cfg["speed_order"])}; '
            'no ROS publication'
        )
        return
    import rclpy
    from rclpy.parameter import Parameter
    from std_srvs.srv import Trigger

    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_node(
            [Parameter(k, value=v) for k, v in cfg.items()],
            node_name='localization_calibration',
            config_defaults=cfg,
            config_validator=validate_config,
            plan_builder=CalibrationPlan,
            document_builder=plan_document,
        )
        total = node.plan.metadata()['total_duration_with_holds_s']
        node.get_logger().info(
            f'Calibration requested: {" -> ".join(node.cfg["speed_order"])}; '
            f'{total:.1f} s total. Waiting for '
            'stationary Vicon and controller; then a '
            f'{node.cfg["startup_hold_s"]:.0f} s initial hold.'
        )
        node.document['runtime_parameters'] = capture_runtime_parameters(node)
        deadline = time.monotonic() + 30.0
        last_reason = None
        # Invocation is the explicit start request. Never retry after a stop,
        # fault, or completed run, even if Vicon subsequently recovers.
        while rclpy.ok() and node.execution.state == 'idle':
            rclpy.spin_once(node, timeout_sec=0.01)
            vicon_control = any(
                info.node_name == 'hamr_controller_node'
                for info in node.get_subscriptions_info_by_topic(
                    '/HAMR_base/odom'
                )
            )
            recording = any(
                info.node_name == 'rosbag2_recorder'
                for info in node.get_subscriptions_info_by_topic(
                    '/local_HAMR/odom'
                )
            )
            if not vicon_control or not recording:
                response = Trigger.Response(
                    success=False,
                    message='Waiting for Vicon controller subscription and bag recorder',
                )
            else:
                response = node.on_start(Trigger.Request(), Trigger.Response())
            if response.success:
                break
            if response.message != last_reason:
                node.get_logger().info(response.message)
                last_reason = response.message
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f'Calibration did not become ready: {response.message}'
                )
        while rclpy.ok() and node.execution.active:
            rclpy.spin_once(node, timeout_sec=0.02)
        if node.execution.state == 'aborted':
            raise RuntimeError(f'Calibration aborted: {node.execution.reason}')
        if node.execution.state == 'complete':
            node.get_logger().info(
                'Calibration complete, including final hold. Stop the bag recorder now.'
            )
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.execution.stop('Calibration publisher shutting down')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
