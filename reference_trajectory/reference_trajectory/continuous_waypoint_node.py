"""ROS adapter for continuous planning and explicitly armed playback."""

import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path as FilePath

from .continuous_execution import ContinuousExecution, Feedback
from .continuous_execution import build_plan, validate_config


def default_config_path():
    """Find the shipped default YAML for installed and source-only previews."""
    source = (
        FilePath(__file__).resolve().parents[1]
        / 'config'
        / 'continuous_waypoint_hw.yaml'
    )
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory

    return (
        FilePath(get_package_share_directory('reference_trajectory'))
        / 'config'
        / 'continuous_waypoint_hw.yaml'
    )


def load_config(path):
    """Read the named node's ROS parameter YAML without initializing ROS."""
    import yaml

    document = yaml.safe_load(FilePath(path).read_text(encoding='utf-8'))
    try:
        parameters = document['continuous_waypoint']['ros__parameters']
    except (TypeError, KeyError) as exc:
        raise ValueError(
            'Expected continuous_waypoint.ros__parameters YAML'
        ) from exc
    return validate_config(parameters)


def plan_document(cfg, plan):
    """Describe nominal limits and the as-yet-uncaptured world placement."""
    planner_path = FilePath(__file__).with_name('continuous_waypoint.py')
    return {
        'schema_version': 1,
        'config': cfg,
        'planner_sha256': hashlib.sha256(
            planner_path.read_bytes()
        ).hexdigest(),
        'continuous_plan': plan.metadata(),
        'placement': {
            'origin_mode': cfg['origin_mode'],
            'rotation_rad': 0.0,
            'translation_m': (
                None
                if cfg['origin_mode'] == 'start_translated_vicon'
                else [0.0, 0.0]
            ),
            'axes': 'Vicon world axes; start translation, no rotation',
        },
        'execution': {
            'requires_explicit_start': True,
            'loop': False,
            'freshness_clock': 'local monotonic receipt time',
            'source_stamps': 'strictly increasing, not compared to host clock',
            'stop': 'reference silence; controller watchdog zeros outputs',
            'wheel_acceleration': 'planning assumption; requires calibration',
            'yaw': 'turret world yaw; base_yaw_offset belongs to controller',
        },
    }


def finite_feedback(msg, received_s, cfg):
    """Extract a normalized, plausible Vicon pose or raise ValueError."""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    values = [float(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite Vicon pose')
    norm = math.sqrt(sum(v * v for v in values[3:]))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError('Invalid Vicon quaternion')
    qx, qy, qz, qw = (v / norm for v in values[3:])
    tilt = math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy))))
    if not cfg['odom_min_z_m'] <= p.z <= cfg['odom_max_z_m']:
        raise ValueError('Vicon Z outside configured controller range')
    if tilt > cfg['odom_max_tilt_rad']:
        raise ValueError('Vicon tilt outside configured controller range')
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)
    )
    if (
        msg.header.stamp.sec < 0
        or not 0 <= msg.header.stamp.nanosec < 1000000000
    ):
        raise ValueError('Malformed Vicon source timestamp')
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if stamp <= 0:
        raise ValueError('Vicon source timestamp must be positive')
    return Feedback(
        float(p.x),
        float(p.y),
        yaw,
        stamp,
        received_s,
        str(msg.header.frame_id).strip(),
        float(p.z),
        (qx, qy, qz, qw),
    )


def create_node(parameter_overrides=None):
    """Construct the ROS node lazily so dry planning needs no ROS runtime."""
    from geometry_msgs.msg import PoseStamped
    from hamr_interfaces.msg import ReferenceTraj
    from nav_msgs.msg import Odometry, Path
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class ContinuousWaypointNode(Node):
        """Publish only while an explicitly started session is healthy."""

        def __init__(self):
            super().__init__(
                'continuous_waypoint', parameter_overrides=parameter_overrides
            )
            if self.get_parameter('use_sim_time').value:
                raise ValueError(
                    'Hardware continuous_waypoint requires use_sim_time=false'
                )
            cfg = {
                key: self.declare_parameter(
                    key, value, ParameterDescriptor(read_only=True)
                ).value
                for key, value in load_config(default_config_path()).items()
            }
            self.cfg = validate_config(cfg)
            self.plan = build_plan(self.cfg)
            self.execution = ContinuousExecution(self.cfg, self.plan)
            self.document = plan_document(self.cfg, self.plan)
            self.reference_topic = '/reference_trajectory'
            # Volatile depth 1 avoids replaying stale references to a new
            # controller.
            self.reference_pub = self.create_publisher(
                ReferenceTraj, self.reference_topic, 1
            )
            latched = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.status_pub = self.create_publisher(
                String, '~/status', latched
            )
            self.metadata_pub = self.create_publisher(
                String, '~/metadata', latched
            )
            self.path_pub = self.create_publisher(Path, '~/path', latched)
            odom_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.odom_sub = self.create_subscription(
                Odometry, self.cfg['odom_topic'], self.on_odom, odom_qos
            )
            self.start_service = self.create_service(
                Trigger, '~/start', self.on_start
            )
            self.stop_service = self.create_service(
                Trigger, '~/stop', self.on_stop
            )
            self.last_status = None
            self.timer = self.create_timer(
                1.0 / self.cfg['reference_timer_hz'],
                self.on_timer,
                clock=Clock(clock_type=ClockType.STEADY_TIME),
            )
            self.publish_status()
            self.metadata_pub.publish(
                String(data=json.dumps(self.document, allow_nan=False))
            )
            self.get_logger().info(
                f'Planned {self.plan.duration:.3f} s. '
                'Idle until /continuous_waypoint/start. '
                'Stop silences references; controller watchdog stops the car.'
            )

        def controller_subscribers(self):
            return sum(
                endpoint.node_name == 'hamr_controller_node'
                and endpoint.topic_type == 'hamr_interfaces/msg/ReferenceTraj'
                for endpoint in self.get_subscriptions_info_by_topic(
                    self.reference_topic
                )
            )

        def publish_status(self):
            state = (
                self.execution.state,
                self.execution.reason,
                self.execution.run_id,
            )
            if state == self.last_status:
                return
            self.last_status = state
            self.status_pub.publish(
                String(
                    data=json.dumps(
                        {
                            'state': state[0],
                            'reason': state[1],
                            'run_id': state[2],
                            'active': self.execution.active,
                        }
                    )
                )
            )
            self.get_logger().info(f'{state[0]}: {state[1]}')

        def on_odom(self, msg):
            try:
                sample = finite_feedback(msg, time.monotonic(), self.cfg)
                self.execution.observe(sample)
            except (ValueError, TypeError, OverflowError) as exc:
                self.execution.invalidate_feedback(str(exc))
            self.publish_status()

        def on_start(self, _request, response):
            response.success, response.message = self.execution.start(
                time.monotonic(),
                subscribers=self.controller_subscribers(),
                publishers=self.count_publishers(self.reference_topic),
            )
            if response.success:
                tx, ty = self.execution.translation
                feedback = self.execution.feedback
                placement = dict(
                    self.document['placement'], translation_m=[tx, ty]
                )
                self.metadata_pub.publish(
                    String(
                        data=json.dumps(
                            dict(
                                self.document,
                                placement=placement,
                                run_id=self.execution.run_id,
                                captured_start_pose={
                                    'x_m': feedback.x,
                                    'y_m': feedback.y,
                                    'raw_base_yaw_rad': feedback.yaw,
                                    'odom_frame_id': feedback.frame_id,
                                    'source_stamp_s': feedback.source_stamp_s,
                                },
                            ),
                            allow_nan=False,
                        )
                    )
                )
                path = Path()
                path.header.frame_id = self.execution.feedback.frame_id
                path.header.stamp = self.get_clock().now().to_msg()
                # Display every ~5 cm; the execution uses the analytic plan.
                stride = max(
                    1, int(0.05 / self.plan.metadata()['max_sample_spacing_m'])
                )
                samples = self.plan.metadata()['samples'][::stride]
                if samples[-1] is not self.plan.metadata()['samples'][-1]:
                    samples = samples + [self.plan.metadata()['samples'][-1]]
                for sample in samples:
                    pose = PoseStamped()
                    pose.header = path.header
                    pose.pose.position.x = sample['position_m'][0] + tx
                    pose.pose.position.y = sample['position_m'][1] + ty
                    pose.pose.orientation.w = 1.0
                    path.poses.append(pose)
                self.path_pub.publish(path)
            self.publish_status()
            return response

        def on_stop(self, _request, response):
            self.execution.stop()
            response.success, response.message = (
                True,
                'Reference publication stopped; watchdog will zero outputs',
            )
            self.publish_status()
            return response

        def on_timer(self):
            try:
                sample = self.execution.step(
                    time.monotonic(),
                    subscribers=self.controller_subscribers(),
                    publishers=self.count_publishers(self.reference_topic),
                )
                if sample is not None:
                    msg = ReferenceTraj()
                    (
                        msg.x,
                        msg.y,
                        msg.yaw,
                        msg.x_dot,
                        msg.y_dot,
                        msg.yaw_dot,
                    ) = sample
                    self.reference_pub.publish(msg)
            except Exception as exc:
                self.execution.stop(
                    f'Reference callback failed: {exc}', fault=True
                )
            self.publish_status()

    return ContinuousWaypointNode()


def main(args=None):
    """Export a pure plan or run an idle service-controlled ROS publisher."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--plan-only',
        metavar='JSON',
        help='Write canonical preview and exit without ROS',
    )
    parser.add_argument(
        '--config', metavar='YAML', help='Hardware ROS parameter YAML'
    )
    parsed, ros_args = parser.parse_known_args(
        sys.argv[1:] if args is None else args
    )
    if parsed.plan_only:
        if ros_args:
            parser.error(
                'Plan-only requires --config instead of ROS parameters'
            )
        cfg = load_config(parsed.config or default_config_path())
        plan = build_plan(cfg)
        output = FilePath(parsed.plan_only).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(plan_document(cfg, plan), indent=2, allow_nan=False)
            + '\n'
        )
        print(f'Wrote {output}: {plan.duration:.3f} s; no ROS publication')
        return
    import rclpy
    from rclpy.parameter import Parameter

    rclpy.init(args=ros_args)
    node = None
    try:
        overrides = None
        if parsed.config:
            overrides = [
                Parameter(key, value=value)
                for key, value in load_config(parsed.config).items()
            ]
        node = create_node(overrides)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.execution.stop('Node shutting down')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
