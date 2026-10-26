"""Check calibration auto-start and parameter capture without hardware."""

import os
import subprocess
import sys
from pathlib import Path


def test_isolated_calibration_entry_point():
    """Keep synthetic feedback and commands on a separate localhost domain."""
    env = dict(os.environ, ROS_DOMAIN_ID='194', ROS_LOCALHOST_ONLY='1')
    result = subprocess.run(
        [sys.executable, str(Path(__file__)), '--smoke'],
        env=env,
        capture_output=True,
        text=True,
        timeout=40,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def run_smoke():
    """Exercise the CLI with a real EKF and fake controller/recorder."""
    import json
    import time

    from ament_index_python.packages import get_package_prefix
    from hamr_interfaces.msg import ReferenceTraj
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    import yaml

    rclpy.init()
    repo = Path(__file__).resolve().parents[2]
    cfg = yaml.safe_load(
        (repo / 'hamr_bringup/config/hamr_hw_control_params.yaml').read_text()
    )['/hamr_controller_node']['ros__parameters']
    # Mirror controller_odom_timeout_s:=0.20 on the calibration hardware launch.
    cfg['odom_timeout_s'] = 0.20
    snapshots = {
        'hamr_controller_node': cfg,
        'holonomic_odom_node': {'yaw_scale': 0.813},
        'hamr_uros_bridge': {'gyro_variance': 0.04},
        'rosbag2_recorder': {},
    }
    nodes = {
        name: rclpy.create_node(
            name,
            parameter_overrides=[
                Parameter(k, value=v) for k, v in params.items()
            ],
            automatically_declare_parameters_from_overrides=True,
        )
        for name, params in snapshots.items()
    }
    controller = nodes['hamr_controller_node']
    recorder = nodes['rosbag2_recorder']
    controller.create_subscription(
        Odometry, '/HAMR_base/odom', lambda _: None, 1
    )
    references = []
    controller.create_subscription(
        ReferenceTraj,
        '/reference_trajectory',
        lambda msg: references.append((time.monotonic(), msg)),
        10,
    )
    recorder.create_subscription(
        Odometry, '/local_HAMR/odom', lambda _: None, 1
    )
    poses = recorder.create_publisher(
        Odometry,
        '/HAMR_base/odom',
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    metadata, statuses = [], []
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    recorder.create_subscription(
        String,
        '/localization_calibration/metadata',
        lambda msg: metadata.append(json.loads(msg.data)),
        qos,
    )
    recorder.create_subscription(
        String,
        '/localization_calibration/status',
        lambda msg: statuses.append(json.loads(msg.data)),
        qos,
    )
    stop = recorder.create_client(Trigger, '/localization_calibration/stop')
    executor = SingleThreadedExecutor()
    for node in nodes.values():
        executor.add_node(node)
    ekf = subprocess.Popen(
        [
            str(
                Path(get_package_prefix('robot_localization'))
                / 'lib/robot_localization/ekf_node'
            ),
            '--ros-args',
            '-r',
            '__node:=ekf_filter_node',
            '--params-file',
            str(repo / 'hamr_bringup/config/ekf_calibrated.yaml'),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    child = subprocess.Popen(
        [
            sys.executable,
            '-m',
            'reference_trajectory.localization_calibration',
            '--speed-set',
            'fast',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    started = time.monotonic()

    def pump():
        msg = Odometry()
        stamp = int(time.monotonic() * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
        msg.header.frame_id = 'world'
        msg.pose.pose.position.x, msg.pose.pose.position.y = 4.0, -3.0
        msg.pose.pose.position.z = 0.30
        msg.pose.pose.orientation.w = 1.0
        poses.publish(msg)
        until = time.monotonic() + 0.005
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=0.001)

    try:
        while time.monotonic() - started < 25:
            pump()
            assert child.poll() is None, child.communicate()[0]
            if any(s['state'] == 'motion' for s in statuses):
                break
        else:
            raise AssertionError('Calibration never reached motion')
        placed = [m for m in metadata if 'runtime_parameters' in m]
        assert placed, 'Effective parameters were not published at start'
        assert (
            placed[-1]['runtime_parameters']['holonomic_odom_node'][
                'yaw_scale'
            ]
            == 0.813
        )
        captured_ekf = placed[-1]['runtime_parameters']['ekf_filter_node']
        assert captured_ekf['odom0'] == '/wheel_odom'
        assert captured_ekf['imu0'] == '/imu/data'
        # These typed but uninitialized slots broke the previous bulk read.
        assert captured_ekf['imu1'] is None
        assert captured_ekf['odom1'] is None
        assert captured_ekf['pose0'] is None
        assert len(captured_ekf['process_noise_covariance']) == 225
        assert placed[-1]['placement']['translation_m'] == [4.0, -3.0]
        assert placed[-1]['config']['speed_order'] == ['fast']
        assert placed[-1]['config']['odom_timeout_s'] == 0.20
        captured_controller = placed[-1]['runtime_parameters']['hamr_controller_node']
        assert captured_controller['odom_timeout_s'] == 0.20
        assert captured_controller['reference_timeout_s'] == 0.12
        assert placed[-1]['continuous_plan']['speed_order'] == ['fast']
        assert {s['state'] for s in statuses} >= {'startup_hold', 'motion'}
        assert statuses[-1]['stage'] == 'fast_forward'
        assert references[-1][0] - references[0][0] >= 9.8
        for _, msg in references[:-5]:
            assert msg.x == 4.0 and msg.y == -3.0
            assert msg.x_dot == msg.y_dot == 0.0
        future = stop.call_async(Trigger.Request())
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            pump()
        assert future.done() and future.result().success
        output, _ = child.communicate(timeout=3)
        assert child.returncode == 0, output
        print(
            'Passed: parameter capture, recorder/controller discovery, '
            'translated start, 10 s hold, stage metadata, explicit stop.'
        )
    finally:
        if child.poll() is None:
            child.terminate()
            output, _ = child.communicate(timeout=3)
            print(output)
        ekf.terminate()
        try:
            ekf_output, _ = ekf.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            ekf.kill()
            ekf_output, _ = ekf.communicate()
        print(ekf_output)
        executor.shutdown()
        for node in nodes.values():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run_smoke()
