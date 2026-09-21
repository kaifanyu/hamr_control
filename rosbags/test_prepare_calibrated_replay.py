"""Calibration math and atomic bag-copy contracts, including a ROS smoke test."""

from copy import deepcopy
import json
import math
from pathlib import Path
import pickle
import sys
from types import SimpleNamespace as NS

import pytest
import yaml

from offline_replay_config import load_profile
import prepare_calibrated_replay as replay


def diagonal(size, value):
    return [value if i % (size + 1) == 0 else 0.0 for i in range(size * size)]


CALIBRATION = {
    'linear_velocity_scale': 0.95, 'yaw_rate_scale': 0.82,
    'b_wheel': 0.26, 'base_yaw_offset': math.pi / 2.0,
    'twist_variance_vx': 0.001, 'twist_variance_vy': 0.002,
    'twist_variance_wz': 0.003,
}
TOPICS = [NS(name=name, type=kind, serialization_format='cdr', offered_qos_profiles='keep')
          for name, kind in {**replay.TARGET_TYPES, '/vicon': 'nav_msgs/msg/Odometry'}.items()]


def old_wheel(stamp_ns, speed, wz, interval, phi=math.pi / 2.0):
    old_b = 0.45
    vx = math.cos(phi) * speed - math.sin(phi) * old_b * wz
    vy = math.sin(phi) * speed + math.cos(phi) * old_b * wz
    c, s = math.cos(wz * interval), math.sin(wz * interval)
    return NS(
        header=NS(stamp=NS(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9),
                  frame_id='odom'), child_frame_id='base_link',
        pose=NS(pose=NS(x=1.2, y=-0.3, yaw=0.7), covariance=diagonal(6, 0.7)),
        twist=NS(covariance=diagonal(6, 0.8), twist=NS(
            linear=NS(x=c * vx + s * vy, y=-s * vx + c * vy, z=0.2),
            angular=NS(x=-0.2, y=0.1, z=wz))),
    )


def imu_message():
    return NS(header=NS(stamp=NS(sec=10, nanosec=5), frame_id='imu_link'),
              orientation=[0, 0, 0, 1], angular_velocity=[0.1, 0.2, 0.3],
              linear_acceleration=[1, 2, 3], orientation_covariance=diagonal(3, 0.9),
              angular_velocity_covariance=diagonal(3, 0.8),
              linear_acceleration_covariance=diagonal(3, 0.7))


@pytest.fixture
def configs(tmp_path):
    base = tmp_path / 'base.yaml'
    parameters = {
        'odom0': '/wheel_odom', 'imu0': '/imu/data',
        'odom0_config': [False] * 6 + [True, True, False, False, False, True] + [False] * 3,
        'imu0_config': [False] * 11 + [True] + [False] * 3,
    }
    base.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
    wheel_twist = diagonal(6, 0.00001)
    for index, name in [(0, 'twist_variance_vx'), (7, 'twist_variance_vy'),
                        (35, 'twist_variance_wz')]:
        wheel_twist[index] = CALIBRATION[name]
    profile = tmp_path / 'profile.yaml'
    profile.write_text(yaml.safe_dump({
        'base_ekf_config': str(base),
        'covariances': {
            'wheel_twist': [wheel_twist[i:i + 6] for i in range(0, 36, 6)],
            'wheel_pose': [[2.0 if i == j else 0 for j in range(6)] for i in range(6)],
            'imu_angular_velocity': [[0.004 if i == j else 0 for j in range(3)]
                                     for i in range(3)],
        },
    }))
    calibration = tmp_path / 'calibration.yaml'
    calibration.write_text(yaml.safe_dump({'/**': {'ros__parameters': CALIBRATION}}))
    return profile, calibration


@pytest.mark.parametrize('phi', [math.pi / 2.0, 0.0, -0.4])
def test_recovers_changing_wheel_rates_and_intervals(phi):
    calibration = dict(CALIBRATION, base_yaw_offset=phi)
    previous = 10_000_000_000
    # Rates change sign, and the interval is deliberately nonuniform.
    for speed, wz, interval in [(0.2, 0.8, 0.02), (-0.1, -0.3, 0.07),
                                (0.35, 1.1, 0.013)]:
        stamp = previous + round(interval * 1e9)
        message = old_wheel(stamp, speed, wz, interval, phi)
        before = deepcopy(message)
        assert replay.calibrate_wheel_message(message, previous, calibration) == stamp
        new_wz = CALIBRATION['yaw_rate_scale'] * wz
        v = CALIBRATION['linear_velocity_scale'] * speed
        b = CALIBRATION['b_wheel']
        assert message.twist.twist.linear.x == pytest.approx(
            math.cos(phi) * v - math.sin(phi) * b * new_wz)
        assert message.twist.twist.linear.y == pytest.approx(
            math.sin(phi) * v + math.cos(phi) * b * new_wz)
        assert message.twist.twist.angular.z == pytest.approx(new_wz)
        assert message.pose == before.pose
        assert message.header == before.header
        assert message.twist.twist.linear.z == before.twist.twist.linear.z
        assert message.twist.twist.angular.x == before.twist.twist.angular.x
        assert message.twist.twist.angular.y == before.twist.twist.angular.y
        previous = stamp


def test_first_sample_uses_zero_dt_without_guessing_unknown_interval():
    message = old_wheel(10_000_000_000, 0.2, 0.8, 0.04)
    original_vy = message.twist.twist.linear.y
    replay.calibrate_wheel_message(message, None, CALIBRATION)
    assert message.twist.twist.linear.y == pytest.approx(original_vy * 0.95)
    assert message.twist.twist.linear.x == pytest.approx(-0.26 * 0.8 * 0.82)


@pytest.mark.parametrize('stamp', [0, 9_000_000_000, 10_000_000_000])
def test_rejects_invalid_or_nonincreasing_stamp(stamp):
    with pytest.raises(ValueError, match='timestamp'):
        replay.calibrate_wheel_message(old_wheel(stamp, 0.2, 0.8, 0.02),
                                       10_000_000_000, CALIBRATION)


@pytest.mark.parametrize('name,value', [
    ('linear_velocity_scale', -0.1), ('yaw_rate_scale', float('inf')),
    ('b_wheel', float('nan')), ('base_yaw_offset', True),
    ('twist_variance_vx', 0.0), ('twist_variance_vy', None),
    ('twist_variance_wz', '0.003'),
])
def test_rejects_invalid_live_calibration(configs, name, value):
    path = configs[1]
    parameters = dict(CALIBRATION, **{name: value})
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
    with pytest.raises(ValueError, match=name):
        replay.load_odometry_config(path)


def test_rejects_pose_fusion_and_covariance_mismatch(configs):
    profile, _ = configs
    document = yaml.safe_load(profile.read_text())
    base = Path(document['base_ekf_config'])
    base_document = yaml.safe_load(base.read_text())
    base_document['/**']['ros__parameters']['odom0_config'][5] = True
    base.write_text(yaml.safe_dump(base_document))
    with pytest.raises(ValueError, match='pose selections disabled'):
        replay.prepare_profile(profile, CALIBRATION)
    base_document['/**']['ros__parameters']['odom0_config'][5] = False
    base.write_text(yaml.safe_dump(base_document))
    with pytest.raises(ValueError, match='does not match live'):
        replay.prepare_profile(profile, dict(CALIBRATION, twist_variance_vx=0.02))


def save_fake_bag(path, rows, topics=TOPICS):
    path.mkdir(exist_ok=True)
    (path / 'data.db3').write_bytes(pickle.dumps(rows))
    metadata = {'storage_identifier': 'sqlite3', 'relative_file_paths': ['data.db3'],
                'topics_with_message_count': [
                    {'topic_metadata': {'name': topic.name},
                     'message_count': sum(row[0] == topic.name for row in rows)}
                    for topic in topics]}
    (path / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': metadata}))


class FakeReader:
    def open(self, storage, converter):
        self.rows = iter(pickle.loads((Path(storage.uri) / 'data.db3').read_bytes()))
        self.next_row = next(self.rows, None)

    def get_all_topics_and_types(self):
        return TOPICS

    def get_all_message_definitions(self):
        return []

    def has_next(self):
        return self.next_row is not None

    def read_next(self):
        row, self.next_row = self.next_row, next(self.rows, None)
        return row


class FakeWriter:
    def open(self, storage, converter):
        self.path, self.rows, self.topics = Path(storage.uri), [], []
        save_fake_bag(self.path, self.rows, self.topics)

    def create_topic(self, topic):
        self.topics.append(topic)
        save_fake_bag(self.path, self.rows, self.topics)

    def write(self, name, data, timestamp):
        self.rows.append((name, data, timestamp))
        save_fake_bag(self.path, self.rows, self.topics)


@pytest.fixture
def fake_ros(monkeypatch):
    modules = {
        'rosbag2_py': NS(SequentialReader=FakeReader, SequentialWriter=FakeWriter,
                        StorageOptions=NS, ConverterOptions=lambda *args: None),
        'rclpy.serialization': NS(deserialize_message=lambda data, kind: pickle.loads(data),
                                  serialize_message=pickle.dumps),
        'rosidl_runtime_py.utilities': NS(get_message=lambda kind: NS),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    # This fake backend stores pickle, not SQLite; real-storage preservation is
    # covered by the ROS integration test below.
    monkeypatch.setattr(replay, 'preserve_storage_metadata', lambda *args: None)


def synthetic_rows():
    return [
        ('/wheel_odom', pickle.dumps(old_wheel(10_000_000_000, 0.2, 0.8, 0.0)), 10_000_020_000),
        ('/imu/data', pickle.dumps(imu_message()), 10_000_030_000),
        ('/vicon', b'preserve this serialized payload exactly', 10_000_040_000),
        ('/wheel_odom', pickle.dumps(old_wheel(10_020_000_000, -0.1, -0.3, 0.02)),
         10_020_030_000),
    ]


def test_copy_preserves_data_times_counts_and_source(tmp_path, configs, fake_ros):
    source, output = tmp_path / 'source', tmp_path / 'calibrated'
    rows = synthetic_rows()
    save_fake_bag(source, rows)
    before = {p.name: p.read_bytes() for p in source.iterdir()}
    manifest = replay.copy_calibrated_bag(source, output, *configs)
    copied = pickle.loads((output / 'data.db3').read_bytes())
    assert [(r[0], r[2]) for r in copied] == [(r[0], r[2]) for r in rows]
    assert copied[2] == rows[2]
    for index in (0, 3):
        old, new = pickle.loads(rows[index][1]), pickle.loads(copied[index][1])
        assert new.header == old.header
        assert new.pose == old.pose
        assert new.twist.covariance[0] == 0.001
        assert new.twist.covariance[7] == 0.002
        assert new.twist.covariance[35] == 0.003
    new_imu = pickle.loads(copied[1][1])
    assert new_imu.angular_velocity_covariance == diagonal(3, 0.004)
    new_imu.angular_velocity_covariance = imu_message().angular_velocity_covariance
    assert new_imu == imu_message()
    assert before == {p.name: p.read_bytes() for p in source.iterdir()}
    assert manifest['total_messages'] == len(rows)
    assert manifest['first_wheel_dt_s'] == 0.0
    assert json.loads((output / replay.MANIFEST_NAME).read_text()) == manifest
    assert (output / 'odometry_calibration.yaml').read_bytes() == configs[1].read_bytes()
    assert (output / 'requested_offline_replay.yaml').read_bytes() == configs[0].read_bytes()
    assert load_profile(output / 'offline_replay.yaml') == replay.prepare_profile(configs[0], CALIBRATION)
    assert not list(tmp_path.glob('.calibrated.partial-*'))
    with pytest.raises(ValueError, match='already calibrated'):
        replay.copy_calibrated_bag(output, tmp_path / 'double', *configs)


@pytest.mark.parametrize('failure', ['timestamps', 'count'])
def test_mid_copy_error_never_publishes_partial_output(tmp_path, configs, fake_ros, failure):
    source, output = tmp_path / 'source', tmp_path / 'failed'
    rows = synthetic_rows()
    if failure == 'timestamps':
        rows[-1] = (rows[-1][0], rows[0][1], rows[-1][2])
    save_fake_bag(source, rows)
    if failure == 'count':
        path = source / 'metadata.yaml'
        document = yaml.safe_load(path.read_text())
        document['rosbag2_bagfile_information']['topics_with_message_count'][-1]['message_count'] += 1
        path.write_text(yaml.safe_dump(document))
    before = {p.name: p.read_bytes() for p in source.iterdir()}
    with pytest.raises(ValueError):
        replay.copy_calibrated_bag(source, output, *configs)
    assert not output.exists()
    assert not list(tmp_path.glob('.failed.partial-*'))
    assert before == {p.name: p.read_bytes() for p in source.iterdir()}


def test_output_cannot_overwrite_source_or_live_inside_it(tmp_path, configs, fake_ros):
    source = tmp_path / 'source'
    save_fake_bag(source, synthetic_rows())
    for output in (source, source / 'trial'):
        with pytest.raises(ValueError):
            replay.copy_calibrated_bag(source, output, *configs)


def test_real_ros_sqlite_bag_copy(tmp_path, configs):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    serialization = pytest.importorskip('rclpy.serialization')
    nav_msgs = pytest.importorskip('nav_msgs.msg')
    sensor_msgs = pytest.importorskip('sensor_msgs.msg')
    source, output = tmp_path / 'real_source', tmp_path / 'real_copy'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(source), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    for topic in TOPICS:
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=topic.name, type=topic.type, serialization_format='cdr'))
    wheel = nav_msgs.Odometry()
    wheel.header.stamp.sec = 10
    wheel.header.frame_id, wheel.child_frame_id = 'odom', 'base_link'
    wheel.pose.pose.position.x = 1.2
    wheel.twist.twist.linear.y = 0.2
    wheel.twist.twist.angular.z = 0.3
    imu = sensor_msgs.Imu()
    imu.header.stamp.sec = 10
    original_vicon = serialization.serialize_message(nav_msgs.Odometry())
    rows = [('/wheel_odom', serialization.serialize_message(wheel), 10_000_020_000),
            ('/imu/data', serialization.serialize_message(imu), 10_000_030_000),
            ('/vicon', original_vicon, 10_000_040_000)]
    for row in rows:
        writer.write(*row)
    del writer
    # Deliberately include the original bags' unknown-history/depth=0 profile.
    # The typed ROS API otherwise silently normalizes this to depth=10.
    metadata_path = source / 'metadata.yaml'
    source_document = yaml.safe_load(metadata_path.read_text())
    source_metadata = source_document['rosbag2_bagfile_information']
    for item in source_metadata['topics_with_message_count']:
        item['topic_metadata']['offered_qos_profiles'] = [{
            'history': 'unknown', 'depth': 0,
            'reliability': 'reliable', 'durability': 'volatile',
            'liveliness': 'automatic', 'avoid_ros_namespace_conventions': False,
            'deadline': {'sec': 9223372036, 'nsec': 854775807},
            'lifespan': {'sec': 9223372036, 'nsec': 854775807},
            'liveliness_lease_duration': {'sec': 9223372036, 'nsec': 854775807},
        }]
    metadata_path.write_text(yaml.safe_dump(source_document))
    before = {p.name: p.read_bytes() for p in source.iterdir()}
    replay.copy_calibrated_bag(source, output, *configs)
    reader, _, counts = replay.open_source(output)
    copied = []
    while reader.has_next():
        copied.append(reader.read_next())
    assert counts == {'/wheel_odom': 1, '/imu/data': 1, '/vicon': 1}
    assert [(r[0], r[2]) for r in copied] == [(r[0], r[2]) for r in rows]
    assert copied[2][1] == original_vicon
    changed = serialization.deserialize_message(copied[0][1], nav_msgs.Odometry)
    assert changed.pose == wheel.pose
    assert changed.header == wheel.header
    assert changed.twist.twist.linear.y == pytest.approx(0.2 * 0.95)
    assert changed.twist.twist.angular.z == pytest.approx(0.3 * 0.82)
    copy_metadata = yaml.safe_load((output / 'metadata.yaml').read_text())[
        'rosbag2_bagfile_information']
    assert {t['topic_metadata']['name']: t['topic_metadata'] for t in copy_metadata['topics_with_message_count']} == {
        t['topic_metadata']['name']: t['topic_metadata'] for t in source_metadata['topics_with_message_count']}
    assert copy_metadata['files'][0]['message_count'] == len(rows)
    assert before == {p.name: p.read_bytes() for p in source.iterdir()}
