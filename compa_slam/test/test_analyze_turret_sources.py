"""Keep separated turret capture analysis compatible with historical bags."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))

from analyze_rosbag import find_local_turret_odom, write_reports


def test_dedicated_turret_topic_takes_precedence_over_legacy_estimate():
    dedicated = {'yaw': [0.2]}
    legacy = {'yaw': [-0.5]}
    data = {'odom': {
        '/HAMR_turret/odom [odom->turret_link]': legacy,
        '/wheel_turret/odom [odom->turret_link]': dedicated,
    }}
    key, series = find_local_turret_odom(data)
    assert key == '/wheel_turret/odom [odom->turret_link]'
    assert series is dedicated


def test_legacy_turret_bag_remains_readable():
    legacy = {'yaw': [-0.5]}
    data = {'odom': {'/HAMR_turret/odom [odom->turret_link]': legacy}}
    assert find_local_turret_odom(data)[1] is legacy


@pytest.mark.parametrize('topics,mixed', [
    ([], False),
    (['/HAMR_turret/odom [mocap->HAMR_turret]'], False),
    (['/HAMR_turret/odom [odom->turret_link]'], False),
    (['/HAMR_turret/odom [mocap->HAMR_turret]',
      '/wheel_turret/odom [odom->turret_link]'], False),
    (['/HAMR_turret/odom [mocap->HAMR_turret]',
      '/HAMR_turret/odom [odom->turret_link]'], True),
])
def test_report_warns_about_mixed_sources_only_when_observed(tmp_path, topics, mixed):
    data = {'odom': {topic: {} for topic in topics}, 'multiarrays': {},
            'delays': {}, 'tf': {}}
    metadata = {'duration': {'nanoseconds': 1_000_000_000},
                'message_count': len(topics), 'topics_with_message_count': []}
    write_reports(data, metadata, None, str(tmp_path))
    report = (tmp_path / 'analysis_summary.txt').read_text()
    assert ('contains both mocap and odom frames' in report) is mixed
    assert 'contains two publishers' not in report
