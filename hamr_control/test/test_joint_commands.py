"""Check joint message values without requiring a ROS installation."""

import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


class Float64:
    """Minimal ROS message stand-in; message objects cannot be negated."""

    def __init__(self):
        self.data = 0.0


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        if not isinstance(message, Float64):
            raise TypeError('Expected a Float64 message')
        self.messages.append(message)


def load_publish_joint_cmd():
    source_path = (
        Path(__file__).resolve().parents[1]
        / 'hamr_control' / 'hamr_controller.py'
    )
    tree = ast.parse(source_path.read_text(encoding='utf-8'))
    controller = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'HamrControlNode'
    )
    method = next(
        node for node in controller.body
        if isinstance(node, ast.FunctionDef) and node.name == 'publish_joint_cmd'
    )
    namespace = {'Float64': Float64}
    isolated = ast.Module(body=[method], type_ignores=[])
    exec(compile(isolated, str(source_path), 'exec'), namespace)
    return namespace['publish_joint_cmd']


class JointCommandTests(unittest.TestCase):
    def test_joint_command_messages_and_signs(self):
        publish_joint_cmd = load_publish_joint_cmd()
        for simulating in (True, False):
            for velocities in ((1.25, -2.5, 0.75), (-1.25, 2.5, -0.75)):
                with self.subTest(simulating=simulating, velocities=velocities):
                    calls = []

                    def compute_velocities(desired_velocity, yaw):
                        calls.append((desired_velocity, yaw))
                        return velocities

                    node = SimpleNamespace(
                        hamr_config={'simulating': simulating},
                        compute_velocities=compute_velocities,
                        right_wheel_vel_=Publisher(),
                        left_wheel_vel_=Publisher(),
                        turret_vel_=Publisher(),
                    )
                    desired = (0.1, 0.2, 0.3)
                    publish_joint_cmd(node, desired, 0.4)
                    self.assertEqual(calls, [(desired, 0.4)])
                    right, left, turret = velocities
                    expected = (-right, left, turret if simulating else -turret)
                    publishers = (
                        node.right_wheel_vel_, node.left_wheel_vel_, node.turret_vel_
                    )
                    for publisher, value in zip(publishers, expected):
                        self.assertEqual(len(publisher.messages), 1)
                        self.assertEqual(publisher.messages[0].data, value)


if __name__ == '__main__':
    unittest.main()
