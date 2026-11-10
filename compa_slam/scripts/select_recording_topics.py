#!/usr/bin/env python3
"""Select a finite set of active recording topics after ROS discovery settles.

Print only topic names to stdout. Recording must still warm up after its own
subscriptions are created; this graph check does not certify message delivery.
"""

import argparse
import math
import os
import re
import sys
import time


RGBD_REQUIRED_TOPICS = frozenset({
    '/d455/color/image_raw',
    '/d455/depth/image_rect_raw',
    '/d455/color/camera_info',
    '/d455/imu',
    '/d455/imu_raw',
    '/local_HAMR/odom',
    '/wheel_odom',
    '/imu/data',
    '/tf',
    '/tf_static',
    '/left_wheel/encoder_ticks',
    '/right_wheel/encoder_ticks',
    '/wheel_control/status',
    '/wheel_encoder/diagnostics',
})


def environment_flag(name, environment):
    """Read an explicit boolean, rejecting typos that could disable a check."""
    value = environment.get(name, 'false').strip().lower()
    if value in ('true', '1', 'yes', 'on'):
        return True
    if value in ('false', '0', 'no', 'off'):
        return False
    raise ValueError(f'{name} must be true or false, got {value!r}')


def required_topics(environment):
    """Keep generic telemetry recording compatible; strict capture opts in."""
    topics = set()
    if environment_flag('COMPA_SLAM_REQUIRE_RGBD', environment):
        topics.update(RGBD_REQUIRED_TOPICS)
    if environment_flag('COMPA_SLAM_REQUIRE_VICON', environment):
        topics.add('/HAMR_base/odom')
    if environment_flag('COMPA_SLAM_REQUIRE_SCAN', environment):
        topics.update({
            '/HAMR_base/odom', '/reference_trajectory',
            '/continuous_waypoint/metadata', '/continuous_waypoint/status',
            '/continuous_waypoint/path',
        })
    return frozenset(topics)


def select_topics(publisher_counts, pattern):
    """Return matching active topics, excluding subscriber-only graph entries."""
    return tuple(sorted(
        topic for topic, count in publisher_counts.items()
        if count > 0 and pattern.search(topic)
    ))


class StableTopicSnapshot:
    """Require warmup, an unchanged nonempty selection, and all required topics."""

    def __init__(self, required, started_at, warmup_s=5.0, stable_s=2.0):
        self.required = frozenset(required)
        self.started_at = started_at
        self.warmup_s = warmup_s
        self.stable_s = stable_s
        self.selected = ()
        self.changed_at = started_at

    @property
    def missing(self):
        return tuple(sorted(self.required.difference(self.selected)))

    def observe(self, selected, now):
        selected = tuple(sorted(set(selected)))
        if selected != self.selected:
            self.selected = selected
            self.changed_at = now
        if (self.selected and not self.missing
                and now - self.started_at >= self.warmup_s
                and now - self.changed_at >= self.stable_s):
            return self.selected
        return None


def positive_seconds(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('regex', help='Topic selection regular expression')
    parser.add_argument('--timeout', type=positive_seconds, default=45.0)
    parser.add_argument('--warmup', type=positive_seconds, default=5.0)
    parser.add_argument('--stable', type=positive_seconds, default=2.0)
    args = parser.parse_args(argv)
    try:
        pattern = re.compile(args.regex)
        required = required_topics(os.environ)
    except (re.error, ValueError) as error:
        parser.error(str(error))
    excluded = sorted(topic for topic in required if not pattern.search(topic))
    if excluded:
        parser.error('Required topics excluded by regex: ' + ', '.join(excluded))
    if args.timeout < max(args.warmup, args.stable):
        parser.error('timeout must cover both warmup and stability intervals')

    # Import lazily so selection and readiness tests do not need a ROS graph.
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=[])
    node = None
    started_at = time.monotonic()
    snapshot = StableTopicSnapshot(required, started_at, args.warmup, args.stable)
    next_progress = started_at
    try:
        node = Node(
            f'compa_slam_recording_topic_selector_{os.getpid()}',
            enable_rosout=False,
            start_parameter_services=False,
            use_global_arguments=False,
        )
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            counts = {
                topic: node.count_publishers(topic)
                for topic, _types in node.get_topic_names_and_types()
                if pattern.search(topic)
            }
            now = time.monotonic()
            result = snapshot.observe(select_topics(counts, pattern), now)
            if result is not None:
                print(
                    f'Selected {len(result)} active recording topics after '
                    f'{now - started_at:.1f}s discovery.', file=sys.stderr,
                )
                print('\n'.join(result), flush=True)
                return 0
            if now - started_at >= args.timeout:
                reason = (
                    'missing required publishers: ' + ', '.join(snapshot.missing)
                    if snapshot.missing else
                    'no matching active publishers' if not snapshot.selected else
                    'topic set did not stabilize'
                )
                print(f'Recording topic selection timed out: {reason}.', file=sys.stderr)
                return 1
            if now >= next_progress:
                waiting = ', '.join(snapshot.missing) or 'topic discovery to settle'
                print(f'Waiting for recording topics: {waiting}.', file=sys.stderr)
                next_progress = now + 5.0
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
