#!/usr/bin/env python3
"""Check prepared bags against immutable originals and selected engine inputs."""
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent))
from evaluate import Recording
from extract import extract
from offline_replay_config import load_profile
from prepare_calibrated_replay import MANIFEST_NAME, prepare_profile, load_odometry_config
from set_replay_covariances import MATRIX_FIELDS, TARGET_TYPES, open_source


def metadata(path):
    return yaml.safe_load((path / 'metadata.yaml').read_text())['rosbag2_bagfile_information']


def validate_one(name, parameters):
    source = HERE.parent / name
    prepared = HERE / 'prepared' / name
    source_reader, _, source_counts = open_source(source)
    copy_reader, _, copy_counts = open_source(prepared)
    assert source_counts == copy_counts
    original_meta, copied_meta = metadata(source), metadata(prepared)
    assert original_meta['starting_time'] == copied_meta['starting_time']
    source_topics = {t['topic_metadata']['name']: t['topic_metadata']
                     for t in original_meta['topics_with_message_count']}
    copied_topics = {t['topic_metadata']['name']: t['topic_metadata']
                     for t in copied_meta['topics_with_message_count']}
    assert source_topics == copied_topics
    definition_fields = lambda reader: {
        d.topic_type: (d.encoding, d.encoded_message_definition, d.type_hash)
        for d in reader.get_all_message_definitions()}
    assert definition_fields(source_reader) == definition_fields(copy_reader)
    assert sum(f['message_count'] for f in copied_meta['files']) == sum(copy_counts.values())
    manifest = json.loads((prepared / MANIFEST_NAME).read_text())
    baseline = json.loads((HERE / 'data' / f'{name}.json').read_text())
    hashes = {}
    for filename, expected in baseline['sha256'].items():
        with (source / filename).open('rb') as stream:
            actual = hashlib.file_digest(stream, 'sha256').hexdigest()
        assert actual == expected, f'Original file changed: {filename}'
        hashes[filename] = actual
    frozen_matrices, frozen_ekf = load_profile(prepared / 'offline_replay.yaml')
    current_calibration = load_odometry_config(ROOT / 'hamr_bringup/config/wheel_odometry_calibration.yaml')
    current_matrices, current_ekf = prepare_profile(
        ROOT / 'hamr_bringup/config/offline_calibrated_replay.yaml', current_calibration)
    assert (frozen_matrices, frozen_ekf) == (current_matrices, current_ekf)
    assert manifest['calibration'] == current_calibration
    classes = {topic: get_message(kind) for topic, kind in TARGET_TYPES.items()}
    counts, unchanged_counts = Counter(), Counter()
    while source_reader.has_next():
        assert copy_reader.has_next()
        topic, original_bytes, timestamp = source_reader.read_next()
        copy_topic, new_bytes, new_timestamp = copy_reader.read_next()
        assert topic == copy_topic and timestamp == new_timestamp
        counts[topic] += 1
        if topic not in classes:
            assert original_bytes == new_bytes, f'Changed non-target message: {topic}'
            unchanged_counts[topic] += 1
            continue
        original = deserialize_message(original_bytes, classes[topic])
        changed = deserialize_message(new_bytes, classes[topic])
        assert original.header == changed.header
        restored = deepcopy(changed)
        if topic == '/wheel_odom':
            assert original.pose == changed.pose
            restored.twist.twist.linear.x = original.twist.twist.linear.x
            restored.twist.twist.linear.y = original.twist.twist.linear.y
            restored.twist.twist.angular.z = original.twist.twist.angular.z
        for key, field_path in MATRIX_FIELDS[topic].items():
            old_owner, new_owner, restore_owner = original, changed, restored
            for field in field_path[:-1]:
                old_owner = getattr(old_owner, field)
                new_owner = getattr(new_owner, field)
                restore_owner = getattr(restore_owner, field)
            field = field_path[-1]
            if key in frozen_matrices:
                assert list(getattr(new_owner, field)) == frozen_matrices[key]
                setattr(restore_owner, field, getattr(old_owner, field))
        assert restored == original, f'Unexpected target field changed: {topic}'
    assert not copy_reader.has_next()
    assert {name: counts[name] for name in source_counts} == source_counts
    assert not set(counts) - set(source_counts)
    assert counts['/HAMR_base/odom'] == unchanged_counts['/HAMR_base/odom']
    extract(prepared, HERE / 'prepared' / 'data')
    original_recording = Recording(HERE / 'data' / f'{name}.npz')
    copied_recording = Recording(HERE / 'prepared' / 'data' / f'{name}.npz')
    expected_events = original_recording.events(parameters)
    actual_events = copied_recording.events()
    assert expected_events.shape == actual_events.shape
    difference = np.abs(expected_events - actual_events)
    assert np.max(difference) < 1e-10
    assert np.array_equal(original_recording.data['imu'], copied_recording.data['imu'])
    assert np.array_equal(original_recording.data['vicon'], copied_recording.data['vicon'])
    assert np.array_equal(original_recording.data['recorded'], copied_recording.data['recorded'])
    wheel_unchanged_columns = [0, 1, 2, 3, 4, 5, 9, 10, 11, 12]
    assert np.array_equal(original_recording.data['wheel'][:, wheel_unchanged_columns],
                          copied_recording.data['wheel'][:, wheel_unchanged_columns])
    return {
        'source_bag': str(source), 'prepared_bag': str(prepared),
        'all_checks_passed': True,
        'total_messages': sum(counts.values()), 'counts': dict(sorted(source_counts.items())),
        'non_target_byte_identical_messages': sum(unchanged_counts.values()),
        'vicon_byte_identical_messages': unchanged_counts['/HAMR_base/odom'],
        'all_headers_recording_timestamps_and_topic_metadata_preserved': True,
        'all_embedded_message_definitions_preserved': True,
        'per_file_and_total_counts_consistent': True,
        'wheel_pose_and_pose_covariance_preserved': True,
        'imu_sensor_payload_unchanged': True,
        'target_covariances_match_frozen_profile': True,
        'frozen_profiles_match_current_live_configuration': True,
        'original_file_sha256_unchanged': hashes,
        'engine_input_event_count': len(expected_events),
        'max_absolute_event_difference': float(np.max(difference)),
        'max_absolute_event_difference_by_column': difference.max(axis=0).tolist(),
        'event_difference_tolerance': 1e-10,
        'first_wheel_interval_policy': manifest['first_wheel_policy'],
        'max_wheel_header_interval_s': manifest['max_wheel_header_interval_s'],
    }


def inspect_launch():
    path = ROOT / 'hamr_bringup/launch/hamr_HW.launch.xml'
    launch = ET.parse(path).getroot()
    arguments = {a.attrib['name']: a.attrib.get('default') for a in launch.findall('arg')}
    lets = {a.attrib['name']: a.attrib['value'] for a in launch.findall('let')}
    assert arguments['use_mag'] == 'false' and arguments['use_orientation'] == 'false'
    assert lets['ekf_config_path'].endswith('/config/ekf_calibrated.yaml')
    assert arguments['ekf_config'] == '$(var ekf_config_path)'
    assert arguments['wheel_odom_config'].endswith('/config/wheel_odometry_calibration.yaml')
    odometry = next(n for n in launch.findall('node') if n.attrib.get('exec') == 'holonomic_odom_node')
    assert odometry.find('param').attrib == {'from': '$(var wheel_odom_config)'}
    assert next(p for p in odometry.findall('param') if p.attrib.get('name') == 'publish_tf').attrib['value'] == 'false'
    ekf = next(n for n in launch.findall('group/node') if n.attrib.get('unless') == '$(var use_mag)')
    assert ekf.find('param').attrib['from'] == '$(var ekf_config)'
    cmake = (ROOT / 'hamr_bringup/CMakeLists.txt').read_text()
    assert 'DIRECTORY launch config worlds terrain_assets' in cmake
    package_files = [
        'hamr_bringup/config/ekf_calibrated.yaml',
        'hamr_bringup/config/wheel_odometry_calibration.yaml',
        'hamr_bringup/config/offline_calibrated_replay.yaml',
        'hamr_bringup/launch/hamr_offline_ekf.launch.xml',
    ]
    for relative in package_files:
        assert (ROOT / relative).is_file()
    return {
        'hardware_default_ekf': 'hamr_bringup/config/ekf_calibrated.yaml',
        'hardware_default_wheel_odometry': 'hamr_bringup/config/wheel_odometry_calibration.yaml',
        'hardware_defaults_use_mag_and_orientation': False,
        'wheel_publish_tf': False,
        'launch_parameter_files_and_install_directories_verified': True,
        'source_package_files_verified': package_files,
        'alternate_paths': {
            'compa_slam/localization_runtime.launch.py': 'orientation mode remains default; set use_orientation:=false use_mag:=false to select calibrated EKF',
            'hamr_control_exp/launch/hamr_exp_HW.launch.xml': 'separate legacy configuration/geometry remains unchanged',
        },
        'validation_scope': 'Static source launch/config routing; no robot hardware was started.',
    }


def main():
    selected = json.loads((HERE / 'report' / 'selected.json').read_text())['parameters']
    names = [p.stem for p in sorted((HERE / 'data').glob('hamr_hw_*.npz'))]
    summary = {
        'selected_parameters': selected,
        'bags': {name: validate_one(name, selected) for name in names},
        'launch_review': inspect_launch(),
        'auxiliary_message_definitions': (
            'All original embedded definitions, including ReferenceTraj/LiveGains, '
            'were restored from the source into SQLite after the ROS writer closed. '
            'Original QoS metadata, including unknown-history/depth=0, was also '
            'restored exactly; ROS Python otherwise normalizes that depth to 10.'),
    }
    output = HERE / 'report' / 'replay_preparation_validation.json'
    output.write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(output, flush=True)


if __name__ == '__main__':
    main()
