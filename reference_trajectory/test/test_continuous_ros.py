"""Exercise ROS services and topics with fake feedback and no actuators."""

import os
import subprocess
import sys
from pathlib import Path


def test_isolated_ros_start_stop_and_stale_feedback():
    """Run a separate localhost DDS domain; never launch hardware drivers."""
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID='193', ROS_LOCALHOST_ONLY='1')
    result = subprocess.run(
        [sys.executable, str(Path(__file__)), '--smoke'],
        env=env,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def run_smoke():
    """Inspect real ReferenceTraj output with synthetic Vicon feedback."""
    import json
    import time

    from hamr_interfaces.msg import ReferenceTraj
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger

    from reference_trajectory.continuous_waypoint_node import create_node

    rclpy.init()
    planner = create_node()
    observer = rclpy.create_node('continuous_contract_test')
    fake_controller = rclpy.create_node('hamr_controller_node')
    executor = SingleThreadedExecutor()
    executor.add_node(planner)
    executor.add_node(observer)
    executor.add_node(fake_controller)
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    poses = observer.create_publisher(
        Odometry,
        '/HAMR_base/odom',
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    references = []
    observer.create_subscription(
        ReferenceTraj, '/reference_trajectory', references.append, 1
    )
    start = observer.create_client(Trigger, '/continuous_waypoint/start')
    stop = observer.create_client(Trigger, '/continuous_waypoint/stop')
    start_epoch = time.monotonic()

    def pump(duration, publish=True):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if publish:
                msg = Odometry()
                elapsed_ns = int((time.monotonic() - start_epoch) * 1e9)
                msg.header.stamp.sec = 2000 + elapsed_ns // 1000000000
                msg.header.stamp.nanosec = elapsed_ns % 1000000000
                msg.header.frame_id = 'world'
                msg.pose.pose.position.x = 4.0
                msg.pose.pose.position.y = -3.0
                msg.pose.pose.position.z = 0.30
                msg.pose.pose.orientation.w = 1.0
                poses.publish(msg)
            until = time.monotonic() + 0.005
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.001)

    def call(client):
        assert client.wait_for_service(timeout_sec=2.0)
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < end:
            pump(0.01)
        assert future.done()
        return future.result()

    try:
        pump(0.7)
        assert references == [], 'Published a reference before explicit start'
        assert not call(
            start
        ).success, 'Recorder alone was mistaken for the controller'
        fake_controller.create_subscription(
            ReferenceTraj, '/reference_trajectory', lambda _msg: None, 1
        )
        pump(0.15)
        assert call(start).success
        pump(0.12)
        assert references, 'No reference after accepted start'
        assert planner.execution.active
        assert all(
            msg.x == 4.0
            and msg.y == -3.0
            and msg.x_dot == 0.0
            and msg.y_dot == 0.0
            for msg in references
        ), 'Captured-origin startup reference is incoherent'
        assert call(stop).success
        pump(0.04)
        references.clear()
        pump(0.15)
        assert references == [], 'Stop emitted reference instead of silence'
        assert call(start).success
        pump(0.08)
        pump(0.2, publish=False)
        assert planner.execution.state == 'aborted'
        references.clear()
        pump(0.35)
        assert (
            references == []
        ), 'Fresh odometry silently resumed aborted route'
        assert call(start).success
        pump(0.03)
        assert planner.execution.active
        assert call(stop).success
        competitor = observer.create_publisher(
            ReferenceTraj, '/reference_trajectory', 1
        )
        pump(0.15)
        assert not call(start).success, 'Competing reference publisher allowed'
        observer.destroy_publisher(competitor)
        print(
            json.dumps(
                {
                    'passed': True,
                    'assertions': [
                        'idle silence',
                        'explicit start',
                        'captured world origin',
                        'stop silence',
                        'stale feedback abort',
                        'no automatic resume',
                        'explicit restart',
                        'competing publisher rejected',
                    ],
                }
            )
        )
    finally:
        executor.shutdown()
        planner.destroy_node()
        observer.destroy_node()
        fake_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run_smoke()
