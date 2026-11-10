"""Exercise capture requirements and discovery stability without a live robot."""

from pathlib import Path
import re
import sys

import pytest


sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))

from select_recording_topics import (  # noqa: E402
    environment_flag,
    main,
    required_topics,
    RGBD_REQUIRED_TOPICS,
    select_topics,
    StableTopicSnapshot,
)


def test_selection_excludes_subscriber_only_and_unrelated_topics():
    counts = {'/wheel_odom': 1, '/d455/color/image_raw': 1,
              '/reference_trajectory': 0, '/unrelated': 2}
    pattern = re.compile(r'^/wheel_odom$|^/d455/|^/reference_trajectory$')
    assert select_topics(counts, pattern) == ('/d455/color/image_raw', '/wheel_odom')


def test_generic_recording_has_no_mandatory_hardware_topics():
    assert required_topics({}) == frozenset()
    state = StableTopicSnapshot([], started_at=0)
    assert state.observe(['/wheel_odom'], 0) is None
    assert state.observe(['/wheel_odom'], 5) == ('/wheel_odom',)


def test_strict_rgbd_requirements_include_imu_and_onboard_odometry():
    assert required_topics({'COMPA_SLAM_REQUIRE_RGBD': 'true'}) == RGBD_REQUIRED_TOPICS
    assert {'/d455/imu', '/d455/imu_raw', '/local_HAMR/odom', '/tf_static'}.issubset(
        RGBD_REQUIRED_TOPICS)
    assert '/HAMR_base/odom' not in RGBD_REQUIRED_TOPICS
    combined = required_topics({'COMPA_SLAM_REQUIRE_RGBD': 'true',
                                'COMPA_SLAM_REQUIRE_VICON': 'true'})
    assert combined == RGBD_REQUIRED_TOPICS | {'/HAMR_base/odom'}
    assert required_topics({'COMPA_SLAM_REQUIRE_VICON': 'true'}) == {'/HAMR_base/odom'}


def test_invalid_boolean_fails_instead_of_disabling_required_inputs():
    with pytest.raises(ValueError, match='COMPA_SLAM_REQUIRE_RGBD'):
        environment_flag('COMPA_SLAM_REQUIRE_RGBD', {'COMPA_SLAM_REQUIRE_RGBD': 'ture'})


def test_scan_recording_waits_for_planner_provenance_publishers():
    required = required_topics({'COMPA_SLAM_REQUIRE_SCAN': 'true'})
    assert required == {
        '/HAMR_base/odom', '/reference_trajectory', '/continuous_waypoint/metadata',
        '/continuous_waypoint/status', '/continuous_waypoint/path',
    }
    state = StableTopicSnapshot(required, started_at=0)
    assert state.observe(required - {'/continuous_waypoint/metadata'}, 10) is None
    assert state.observe(required, 11) is None
    assert set(state.observe(required, 13)) == required


def test_late_optional_publisher_restarts_stability_window():
    state = StableTopicSnapshot(['/wheel_odom'], started_at=100)
    assert state.observe(['/wheel_odom'], 100) is None
    assert state.observe(['/wheel_odom', '/HAMR_base/odom'], 104) is None
    assert state.observe(['/wheel_odom', '/HAMR_base/odom'], 105) is None
    assert state.observe(['/wheel_odom', '/HAMR_base/odom'], 106) == (
        '/HAMR_base/odom', '/wheel_odom')


def test_required_publisher_disappearing_prevents_readiness():
    state = StableTopicSnapshot(['/wheel_odom', '/d455/imu'], started_at=0)
    assert state.observe(['/wheel_odom', '/d455/imu'], 0) is None
    assert state.observe(['/wheel_odom'], 4) is None
    assert state.observe(['/wheel_odom'], 20) is None
    assert state.missing == ('/d455/imu',)
    assert state.observe(['/wheel_odom', '/d455/imu'], 21) is None
    assert state.observe(['/wheel_odom', '/d455/imu'], 23) == (
        '/d455/imu', '/wheel_odom')


def test_empty_stable_graph_is_never_ready():
    state = StableTopicSnapshot([], started_at=0)
    assert state.observe([], 60) is None


def test_regex_excluding_required_topics_fails_before_ros_init(monkeypatch, capsys):
    monkeypatch.setenv('COMPA_SLAM_REQUIRE_RGBD', 'false')
    monkeypatch.setenv('COMPA_SLAM_REQUIRE_VICON', 'true')
    with pytest.raises(SystemExit) as error:
        main([r'^/wheel_odom$'])
    assert error.value.code == 2
    captured = capsys.readouterr()
    assert not captured.out
    assert 'Required topics excluded by regex: /HAMR_base/odom' in captured.err
