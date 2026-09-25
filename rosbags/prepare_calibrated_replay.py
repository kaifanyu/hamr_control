#!/usr/bin/env python3
"""Prepare September 2026 wheel/IMU bags for the calibrated offline EKF.

This explicitly converts the ORIGINAL, uncalibrated wheel publisher's twist.
It first reverses that publisher's one-step body-frame rotation, then applies
the live velocity scales and tracked-point offset. Do not use it on recordings
from the corrected/calibrated live publisher. Wheel pose is left unchanged and
must not be fused. All message headers, recording timestamps, topic metadata,
and non-target message bytes are preserved. The first recorded wheel sample
has no preceding interval, so its rotation correction uses dt=0.
"""

import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import sqlite3
import struct
import tempfile

import yaml

from offline_replay_config import COVARIANCE_SIZES, load_profile
from set_replay_covariances import TARGET_TYPES, apply_profile, open_source


MANIFEST_NAME = 'calibrated_replay_manifest.json'
TRANSFORM_ID = 'hamr_september_2026_wheel_twist_v1'
CALIBRATION_KEYS = (
    'linear_velocity_scale', 'yaw_rate_scale', 'b_wheel', 'base_yaw_offset',
    'twist_variance_vx', 'twist_variance_vy', 'twist_variance_wz',
)
POSITIVE_KEYS = set(CALIBRATION_KEYS) - {'b_wheel', 'base_yaw_offset'}


def load_odometry_config(path):
    """Load explicit live calibration parameters; do not infer missing values."""
    path = Path(path).expanduser().resolve()
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8-sig'))
        parameters = document['/**']['ros__parameters']
    except (OSError, yaml.YAMLError, TypeError, KeyError) as exc:
        raise ValueError('Odometry YAML must contain /**: ros__parameters:') from exc
    if not isinstance(parameters, dict):
        raise ValueError('Odometry ros__parameters must be a mapping')
    calibration = {}
    for name in CALIBRATION_KEYS:
        value = parameters.get(name)
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f'Odometry {name} must be an explicit finite number')
        if name in POSITIVE_KEYS and value <= 0.0:
            raise ValueError(f'Odometry {name} must be strictly positive')
        calibration[name] = float(value)
    return calibration


def prepare_profile(path, calibration):
    """Validate velocity-only fusion and agreement with live wheel variances."""
    matrices, ekf_document = load_profile(path)
    parameters = ekf_document['/**']['ros__parameters']
    if any(parameters['odom0_config'][:6]):
        raise ValueError('Calibrated replay requires all wheel pose selections disabled')
    wheel_twist = matrices.get('wheel_twist')
    if wheel_twist is None:
        raise ValueError('Calibrated replay requires an explicit wheel_twist covariance')
    for index, name in ((0, 'twist_variance_vx'), (7, 'twist_variance_vy'),
                        (35, 'twist_variance_wz')):
        if not math.isclose(wheel_twist[index], calibration[name], rel_tol=1e-12,
                            abs_tol=0.0):
            raise ValueError(f'Profile wheel_twist diagonal does not match live {name}')
    # Pose belongs to the old integration. Keep its values and covariance intact.
    matrices.pop('wheel_pose', None)
    return matrices, ekf_document


def header_stamp_ns(message):
    stamp = message.header.stamp
    sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        raise ValueError('Wheel header timestamp is invalid')
    result = sec * 1_000_000_000 + nanosec
    if result <= 0:
        raise ValueError('Wheel header timestamp must be positive')
    return result


def calibrate_wheel_message(message, previous_stamp_ns, calibration):
    """Change only wheel vx/vy/wz; return the current integer header stamp.

    The old callback constructed world velocity before updating heading, then
    inverted that velocity with the updated heading. Rotating its published
    body velocity by +old_wz*dt recovers the original body velocity. Projecting
    onto the forward axis discards the old lever arm before applying the new
    physical offset. The *current* old yaw rate is required when rates change.
    """
    stamp_ns = header_stamp_ns(message)
    if previous_stamp_ns is not None and stamp_ns <= previous_stamp_ns:
        raise ValueError('Wheel header timestamps must be strictly increasing')
    dt = 0.0 if previous_stamp_ns is None else (stamp_ns - previous_stamp_ns) * 1e-9
    twist = message.twist.twist
    old_vx, old_vy, old_wz = twist.linear.x, twist.linear.y, twist.angular.z
    if not all(math.isfinite(value) for value in (old_vx, old_vy, old_wz)):
        raise ValueError('Wheel vx/vy/wz must be finite')
    delta = old_wz * dt
    c, s = math.cos(delta), math.sin(delta)
    corrected_vx = c * old_vx - s * old_vy
    corrected_vy = s * old_vx + c * old_vy
    phi = calibration['base_yaw_offset']
    cp, sp = math.cos(phi), math.sin(phi)
    axle_speed = cp * corrected_vx + sp * corrected_vy
    v = calibration['linear_velocity_scale'] * axle_speed
    wz = calibration['yaw_rate_scale'] * old_wz
    b = calibration['b_wheel']
    vx, vy = cp * v - sp * b * wz, sp * v + cp * b * wz
    if not all(math.isfinite(value) for value in (vx, vy, wz)):
        raise ValueError('Calibration produced non-finite wheel velocity')
    twist.linear.x, twist.linear.y, twist.angular.z = vx, vy, wz
    return stamp_ns


def metadata_counts(path):
    document = yaml.safe_load((path / 'metadata.yaml').read_text(encoding='utf-8'))
    return {
        item['topic_metadata']['name']: item['message_count']
        for item in document['rosbag2_bagfile_information']['topics_with_message_count']
    }


def preserve_storage_metadata(source_metadata, staged_bag, definitions):
    """Restore source QoS/schema text after rosbag2's typed metadata conversion.

    Jazzy's Python reader normalizes unknown-history/depth=0 QoS to depth=10,
    and its writer cannot accept an embedded definition via create_topic().
    Restore these storage fields in the private SQLite staging directory before
    it is published. No message payload or timestamp is changed here.
    """
    metadata_path = staged_bag / 'metadata.yaml'
    document = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    metadata = document['rosbag2_bagfile_information']
    originals = {t['topic_metadata']['name']: t['topic_metadata']
                 for t in source_metadata['topics_with_message_count']}
    for filename in metadata['relative_file_paths']:
        with sqlite3.connect(staged_bag / filename) as connection:
            for name, topic in originals.items():
                qos = topic.get('offered_qos_profiles', [])
                encoded_qos = qos if isinstance(qos, str) else yaml.safe_dump(qos)
                connection.execute(
                    'UPDATE topics SET offered_qos_profiles=? WHERE name=?',
                    (encoded_qos, name))
            for definition in definitions:
                connection.execute('DELETE FROM message_definitions WHERE topic_type=?',
                                   (definition.topic_type,))
                connection.execute(
                    'INSERT INTO message_definitions (topic_type,encoding,'
                    'encoded_message_definition,type_description_hash) VALUES (?,?,?,?)',
                    (definition.topic_type, definition.encoding,
                     definition.encoded_message_definition, definition.type_hash))
            count, start, end = connection.execute(
                'SELECT COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages').fetchone()
        # Jazzy's SequentialWriter can count messages twice in this per-file
        # field even though global/topic counts are correct. Use stored rows.
        for file_entry in metadata.get('files', []):
            if file_entry['path'] == filename:
                file_entry['message_count'] = count
                if count:
                    file_entry['starting_time'] = {'nanoseconds_since_epoch': start}
                    file_entry['duration'] = {'nanoseconds': end - start}
    for item in metadata['topics_with_message_count']:
        item['topic_metadata'] = deepcopy(originals[item['topic_metadata']['name']])
    metadata['custom_data'] = deepcopy(source_metadata.get('custom_data', {}))
    metadata_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding='utf-8')


def copy_calibrated_bag(source, output, profile, odometry_config):
    """Publish an atomically staged copy after checking every topic count."""
    source = Path(source).expanduser().resolve()
    output = Path(output).expanduser().resolve()
    profile = Path(profile).expanduser().resolve()
    odometry_config = Path(odometry_config).expanduser().resolve()
    if output.exists():
        raise ValueError(f'Output already exists: {output}')
    if source == output or source in output.parents:
        raise ValueError('Output must be outside the original bag directory')
    if (source / MANIFEST_NAME).exists():
        raise ValueError('Source is already calibrated; refusing double correction')
    calibration = load_odometry_config(odometry_config)
    matrices, ekf_document = prepare_profile(profile, calibration)
    parameters = ekf_document['/**']['ros__parameters']
    profile_bytes = profile.read_bytes()
    calibration_bytes = odometry_config.read_bytes()
    reader, topics, expected_counts = open_source(source)
    source_metadata = yaml.safe_load((source / 'metadata.yaml').read_text(
        encoding='utf-8'))['rosbag2_bagfile_information']
    source_definitions = reader.get_all_message_definitions()

    # ROS is optional for pure calibration/profile tests.
    from rclpy.serialization import deserialize_message, serialize_message
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message

    types = {name: get_message(kind) for name, kind in TARGET_TYPES.items()}
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f'.{output.name}.partial-', dir=output.parent))
    staged_bag = staging / 'bag'
    writer = None
    counts = Counter()
    previous_wheel_stamp_ns = None
    first_wheel_stamp_ns = None
    max_wheel_dt_s = 0.0
    source_digest = hashlib.sha256()
    try:
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(staged_bag), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''),
        )
        for topic in topics:
            writer.create_topic(topic)
        while reader.has_next():
            name, data, timestamp = reader.read_next()
            # Length-prefix fields so this digest unambiguously captures all
            # original serialized messages, topic names and recording times.
            topic_bytes = name.encode('utf-8')
            source_digest.update(struct.pack('<I', len(topic_bytes)))
            source_digest.update(topic_bytes)
            source_digest.update(struct.pack('<qQ', timestamp, len(data)))
            source_digest.update(data)
            if name in types:
                message = deserialize_message(data, types[name])
                if name == '/wheel_odom':
                    prior_stamp = previous_wheel_stamp_ns
                    previous_wheel_stamp_ns = calibrate_wheel_message(
                        message, prior_stamp, calibration)
                    if prior_stamp is None:
                        first_wheel_stamp_ns = previous_wheel_stamp_ns
                    else:
                        max_wheel_dt_s = max(
                            max_wheel_dt_s, (previous_wheel_stamp_ns - prior_stamp) * 1e-9)
                apply_profile(name, message, matrices, parameters)
                data = serialize_message(message)
            writer.write(name, data, timestamp)
            counts[name] += 1
        if any(counts[name] != count for name, count in expected_counts.items()) or (
            set(counts) - set(expected_counts)
        ):
            raise ValueError('Read message counts do not match source metadata')
        writer = None  # Flush the sqlite3 bag and metadata before publishing it.
        preserve_storage_metadata(source_metadata, staged_bag, source_definitions)
        if metadata_counts(staged_bag) != expected_counts:
            raise ValueError('Written message counts do not match source metadata')
        (staged_bag / 'ekf.yaml').write_text(
            yaml.safe_dump(ekf_document, sort_keys=False), encoding='utf-8')
        saved_matrices = {'wheel_pose': None}
        for key, values in matrices.items():
            size = COVARIANCE_SIZES[key]
            saved_matrices[key] = [values[i:i + size] for i in range(0, len(values), size)]
        (staged_bag / 'offline_replay.yaml').write_text(yaml.safe_dump({
            'base_ekf_config': 'ekf.yaml', 'ekf': {}, 'covariances': saved_matrices,
        }, sort_keys=False), encoding='utf-8')
        (staged_bag / 'requested_offline_replay.yaml').write_bytes(profile_bytes)
        (staged_bag / 'odometry_calibration.yaml').write_bytes(calibration_bytes)
        manifest = {
            'schema_version': 1,
            'transform': TRANSFORM_ID,
            'created_utc': datetime.now(timezone.utc).isoformat(),
            'source_bag': str(source),
            'source_metadata_sha256': hashlib.sha256(
                (source / 'metadata.yaml').read_bytes()).hexdigest(),
            'source_records_sha256': source_digest.hexdigest(),
            'source_profile': str(profile),
            'source_profile_sha256': hashlib.sha256(profile_bytes).hexdigest(),
            'source_odometry_config': str(odometry_config),
            'source_odometry_config_sha256': hashlib.sha256(calibration_bytes).hexdigest(),
            'calibration': calibration,
            'message_counts': dict(sorted(expected_counts.items())),
            'total_messages': sum(counts.values()),
            'wheel_pose_preserved': True,
            'wheel_pose_covariance_preserved': True,
            'header_and_recording_timestamps_preserved': True,
            'non_target_serialized_bytes_preserved': True,
            'source_topic_qos_and_embedded_definitions_preserved': True,
            'first_wheel_stamp_ns': first_wheel_stamp_ns,
            'first_wheel_dt_s': 0.0,
            'first_wheel_policy': (
                'Unknown preceding interval: no rotation correction for the first '
                'recorded wheel sample; calibration scales/offset still apply.'),
            'max_wheel_header_interval_s': max_wheel_dt_s,
            'interval_assumption': (
                'Consecutive recorded wheel header stamps span one original '
                'publisher update; lost wheel messages can violate this assumption.'),
            'input_contract': (
                'Original September 2026 uncalibrated wheel publisher with the '
                'post-heading-update twist rotation error; not corrected live data.'),
        }
        (staged_bag / MANIFEST_NAME).write_text(
            json.dumps(manifest, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        if output.exists():
            raise ValueError(f'Output appeared during copying: {output}')
        staged_bag.rename(output)
        staging.rmdir()
    except BaseException:
        writer = None
        shutil.rmtree(staging, ignore_errors=True)
        raise
    print(f'Created {output} ({sum(counts.values())} messages)')
    print(f'Frozen EKF: {output / "ekf.yaml"}')
    print('First wheel message uses dt=0; wheel pose is preserved and must not be fused.')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--profile', required=True, type=Path,
                        help='Tuned covariance and EKF-selection profile')
    parser.add_argument('--odometry-config', required=True, type=Path,
                        help='Live wildcard ROS odometry calibration YAML')
    args = parser.parse_args()
    try:
        copy_calibrated_bag(args.input, args.output, args.profile, args.odometry_config)
    except (OSError, ValueError, TypeError, KeyError, ImportError, RuntimeError) as exc:
        parser.exit(1, f'Error: {exc}\n')


if __name__ == '__main__':
    main()
