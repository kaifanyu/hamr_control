"""Offline profile/copy regression checks; no ROS installation is required.

Run: python -m unittest discover -s rosbags -p test_offline_replay_config.py -v
The fake storage tests exercise our copy contract, not the ROS storage plugin.
"""

from contextlib import redirect_stdout
from copy import deepcopy
import io
from pathlib import Path
import pickle
import re
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import yaml

import offline_replay_config as profile
import set_replay_covariances as replay


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "hamr_bringup/config/offline_replay.yaml"


def diagonal(size, value):
    return [value if i % (size + 1) == 0 else 0.0 for i in range(size * size)]


def sensor_messages():
    wheel = NS(header=NS(stamp=123, frame_id="odom"), child_frame_id="base_link",
               pose=NS(covariance=diagonal(6, 0.7), pose=NS(x=1.2, yaw=0.3)),
               twist=NS(covariance=diagonal(6, 0.8), twist=NS(vx=0.4, vy=-0.2)))
    imu = NS(header=NS(stamp=124, frame_id="imu_link"), orientation=[0, 0, 0, 1],
             angular_velocity=[0.1, 0.2, 0.3], linear_acceleration=[1, 2, 3],
             orientation_covariance=diagonal(3, 0.9),
             angular_velocity_covariance=diagonal(3, 0.8),
             linear_acceleration_covariance=diagonal(3, 0.7))
    return wheel, imu


TOPICS = [NS(name=name, type=kind, serialization_format="cdr", offered_qos_profiles="keep")
          for name, kind in {**replay.TARGET_TYPES, "/vicon": "nav_msgs/msg/Odometry"}.items()]


def save_fake_bag(path, rows):
    path.mkdir(exist_ok=True)
    (path / "data.db3").write_bytes(pickle.dumps(rows))
    metadata = {"storage_identifier": "sqlite3", "relative_file_paths": ["data.db3"],
                "topics_with_message_count": [
                    {"topic_metadata": {"name": topic.name},
                     "message_count": sum(row[0] == topic.name for row in rows)}
                    for topic in TOPICS]}
    (path / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": metadata}))


class FakeReader:
    def open(self, storage, converter):
        self.rows = iter(pickle.loads((Path(storage.uri) / "data.db3").read_bytes()))
        self.next_row = next(self.rows, None)

    def get_all_topics_and_types(self):
        return TOPICS

    def has_next(self):
        return self.next_row is not None

    def read_next(self):
        row, self.next_row = self.next_row, next(self.rows, None)
        return row


class FakeWriter:
    def open(self, storage, converter):
        self.path, self.rows, self.topics = Path(storage.uri), [], []
        save_fake_bag(self.path, self.rows)

    def create_topic(self, topic):
        self.topics.append(topic)

    def write(self, name, data, timestamp):
        self.rows.append((name, data, timestamp))
        save_fake_bag(self.path, self.rows)


FAKE_ROS = {
    "rosbag2_py": NS(SequentialReader=FakeReader, SequentialWriter=FakeWriter,
                     StorageOptions=NS, ConverterOptions=lambda *args: None),
    "rclpy.serialization": NS(deserialize_message=lambda data, kind: pickle.loads(data),
                               serialize_message=pickle.dumps),
    "rosidl_runtime_py.utilities": NS(get_message=lambda kind: NS),
}


class OfflineReplayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.document = yaml.safe_load(CONFIG.read_text())
        self.document["base_ekf_config"] = str(CONFIG.with_name("ekf.yaml"))
        self.matrices, self.ekf = profile.load_profile(CONFIG)

    def write_profile(self, document=None):
        path = self.folder / "trial.yaml"
        path.write_text(yaml.safe_dump(self.document if document is None else document))
        return path

    def test_defaults_match_publishers_and_preserve_base(self):
        before = CONFIG.read_bytes(), CONFIG.with_name("ekf.yaml").read_bytes()
        base = yaml.safe_load(before[1])["/**"]["ros__parameters"]
        parameters = self.ekf["/**"]["ros__parameters"]
        for key in ("odom0_config", "imu0_config", "process_noise_covariance"):
            self.assertEqual(base[key], parameters[key])
        self.assertTrue(parameters["use_sim_time"])
        wheel_code = (ROOT / "hamr_odometry/hamr_odometry/holonomic_odom_node.py").read_text()
        for field in ("pose", "twist"):
            assignments = re.findall(r"odom\." + field + r"\.covariance\[(\d+)\]\s*=\s*([\deE.+-]+)", wheel_code)
            for index, value in assignments:
                self.assertEqual(self.matrices["wheel_" + field][int(index)], float(value))
        imu_code = (ROOT / "hamr_uros_bridge/src/relay_node.cpp").read_text()
        for field in ("orientation", "angular_velocity", "linear_acceleration"):
            body = re.search(r"msg\." + field + r"_covariance\s*=\s*\{([^}]+)\}", imu_code)[1]
            values = [float(value.strip()) for value in body.split(",") if value.strip()]
            self.assertEqual(self.matrices["imu_" + field], values)
        parameters["odom0_config"][0] = True
        self.assertFalse(profile.load_profile(CONFIG)[1]["/**"]["ros__parameters"]["odom0_config"][0])
        self.assertEqual(before, (CONFIG.read_bytes(), CONFIG.with_name("ekf.yaml").read_bytes()))

    def test_invalid_matrices_and_selections(self):
        bad_matrices = [[[1, 0], [0, 1]], [[1, 0, 0], [0, float("inf"), 0], [0, 0, 1]],
                        [[1, 0.2, 0], [0, 1, 0], [0, 0, 1]],
                        [[1, 2, 0], [2, 1, 0], [0, 0, 1]],
                        [[True, 0, 0], [0, 1, 0], [0, 0, 1]]]
        for matrix in bad_matrices:
            with self.subTest(matrix=matrix), self.assertRaises(ValueError):
                changed = deepcopy(self.document)
                changed["covariances"]["imu_angular_velocity"] = matrix
                profile.load_profile(self.write_profile(changed))
        for selection in ([False] * 14, [0] * 15, ["false"] * 15):
            with self.subTest(selection=selection), self.assertRaises(ValueError):
                changed = deepcopy(self.document)
                changed["ekf"]["imu0_config"] = selection
                profile.load_profile(self.write_profile(changed))

    def test_full_matrices_cross_terms_and_null_preserve_measurements(self):
        self.document["covariances"]["wheel_twist"][0][1] = 0.003
        self.document["covariances"]["wheel_twist"][1][0] = 0.003
        self.document["covariances"]["wheel_pose"] = None
        matrices, ekf = profile.load_profile(self.write_profile())
        self.assertNotIn("wheel_pose", matrices)
        wheel, imu = sensor_messages()
        original_wheel, original_imu = deepcopy(wheel), deepcopy(imu)
        for topic, message in (("/wheel_odom", wheel), ("/imu/data", imu)):
            replay.apply_profile(topic, message, matrices, ekf["/**"]["ros__parameters"])
        self.assertEqual(wheel.pose, original_wheel.pose)
        self.assertEqual(wheel.twist.covariance[1], 0.003)
        self.assertEqual(wheel.twist.covariance[6], 0.003)
        for key, field in replay.MATRIX_FIELDS["/imu/data"].items():
            self.assertEqual(getattr(imu, field[0]), matrices[key])
            setattr(imu, field[0], getattr(original_imu, field[0]))
        self.assertEqual(imu, original_imu)
        wheel.twist.covariance = original_wheel.twist.covariance
        self.assertEqual(wheel, original_wheel)

    def test_unavailable_imu_cannot_be_enabled_or_overwritten(self):
        _, imu = sensor_messages()
        imu.orientation_covariance[0] = -1
        parameters = deepcopy(self.ekf["/**"]["ros__parameters"])
        with self.assertRaises(ValueError):
            replay.apply_profile("/imu/data", imu, self.matrices, parameters)
        matrices = {key: value for key, value in self.matrices.items() if key != "imu_orientation"}
        replay.apply_profile("/imu/data", imu, matrices, parameters)
        self.assertEqual(imu.orientation_covariance[0], -1)
        parameters["imu0_config"][5] = True
        with self.assertRaises(ValueError):
            replay.apply_profile("/imu/data", imu, matrices, parameters)

    @patch.dict(sys.modules, FAKE_ROS)
    def test_copy_freezes_config_preserves_source_timestamps_and_unrelated_bytes(self):
        wheel, imu = sensor_messages()
        rows = [("/wheel_odom", pickle.dumps(wheel), 100), ("/imu/data", pickle.dumps(imu), 101),
                ("/vicon", b"unchanged serialized vicon payload", 102)]
        source, output = self.folder / "original", self.folder / "configured"
        save_fake_bag(source, rows)
        before = {path.name: path.read_bytes() for path in source.iterdir()}
        with redirect_stdout(io.StringIO()):
            replay.copy_bag(source, output, config=self.write_profile())
        copied = pickle.loads((output / "data.db3").read_bytes())
        self.assertEqual([(r[0], r[2]) for r in copied], [(r[0], r[2]) for r in rows])
        self.assertEqual(copied[-1], rows[-1])
        self.assertEqual(before, {path.name: path.read_bytes() for path in source.iterdir()})
        self.assertEqual(profile.load_profile(output / "offline_replay.yaml"), (self.matrices, self.ekf))
        self.assertEqual(yaml.safe_load((output / "ekf.yaml").read_text()), self.ekf)
        self.assertEqual(pickle.loads(copied[0][1]).twist.covariance, self.matrices["wheel_twist"])
        legacy = self.folder / "legacy"
        with redirect_stdout(io.StringIO()):
            replay.copy_bag(source, legacy, 0.001, 0.002, 0.003)
        legacy_rows = pickle.loads((legacy / "data.db3").read_bytes())
        self.assertEqual(pickle.loads(legacy_rows[0][1]).twist.covariance[0:8:7], [0.001, 0.002])
        self.assertEqual(pickle.loads(legacy_rows[1][1]).angular_velocity_covariance[8], 0.003)
        shown = io.StringIO()
        with redirect_stdout(shown):
            replay.show_bag(legacy)
        self.assertIn("odom.twist.covariance[0] = 0.001", shown.getvalue())
        self.assertIn("imu.angular_velocity_covariance[8] = 0.003", shown.getvalue())
        self.assertIn("Covariance is constant", shown.getvalue())


if __name__ == "__main__":
    unittest.main()
