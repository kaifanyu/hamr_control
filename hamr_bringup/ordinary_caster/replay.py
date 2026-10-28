"""Kinematic replay of measured ordinary-caster joints, with explicit gaps."""

import argparse
import json
import math
import sys
import time

from .history import add_history_arguments, history_from_args


def frame_times(history, fps=25.0):
    if not math.isfinite(fps) or not 0 < fps <= 1000:
        raise ValueError("Frame rate must be in (0, 1000]")
    duration = history.end_ns - history.start_ns
    count = int(math.ceil(duration * fps / 1e9))
    for index in range(count):
        yield min(history.end_ns, history.start_ns + round(index * 1e9 / fps))
    yield history.end_ns


def validate_replay(history, allow_uncalibrated=False):
    if history.time_basis not in ("estimated_acquisition_epoch", "device_local"):
        raise ValueError("Replay needs clock maps or explicit --device-local mode")
    for wheel in history.wheels:
        streams = [stream for stream in history.streams if stream.wheel_id == wheel]
        if not any(point.valid and (point.calibrated or allow_uncalibrated)
                   for stream in streams for point in stream.observations):
            raise ValueError(f"{wheel} has no eligible joint observations. Verify swivel phase / wheel CPR calibration, "
                             "or use --allow-uncalibrated for an explicitly illustrative preview.")


def replay_frame(history, source_ns, allow_uncalibrated=False):
    pose = history.pose_at(source_ns, allow_uncalibrated)
    pose.update(replay_time_ns=1_000_000_000 + source_ns - history.start_ns,
                run_id=history.run_id, kinematic_replay=True,
                source_time_ns=source_ns, sensor_timestamps_preserved=True,
                tf_note="Unavailable joints are omitted. A TF viewer may retain an earlier pose; channel status is authoritative.")
    return pose


def _boolean(value):
    if isinstance(value, bool):
        return value
    if value.lower() in ("true", "1", "yes"):
        return True
    if value.lower() in ("false", "0", "no"):
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def main(argv=None):
    raw_args = sys.argv[1:] if argv is None else argv
    try:
        from rclpy.utilities import remove_ros_args
        arguments = remove_ros_args([sys.argv[0], *raw_args])[1:]
    except ImportError:
        arguments = raw_args
    parser = argparse.ArgumentParser(description=__doc__)
    add_history_arguments(parser)
    parser.add_argument("--allow-uncalibrated", action="store_true")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--start-delay", type=float, default=1.0,
                        help="Steady-clock seconds to repeat the first frame while ROS subscribers connect")
    parser.add_argument("--publish-clock", type=_boolean, nargs="?", const=True, default=True)
    parser.add_argument("--joint-topic", default="/ordinary_caster/replay/joint_states")
    parser.add_argument("--exit-on-end", action="store_true")
    args = parser.parse_args(arguments)
    try:
        if not math.isfinite(args.rate) or args.rate <= 0:
            raise ValueError("Playback rate must be positive and finite")
        if not math.isfinite(args.fps) or not 0 < args.fps <= 1000:
            raise ValueError("Frame rate must be in (0, 1000]")
        if not math.isfinite(args.start_delay) or args.start_delay < 0:
            raise ValueError("Start delay must be nonnegative and finite")
        history = history_from_args(args, require_clock=True)
        validate_replay(history, args.allow_uncalibrated)
        import rclpy
        from rclpy.clock import Clock as ROSClock, ClockType
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from visualization_msgs.msg import Marker
    except (ValueError, OSError, ImportError) as error:
        parser.error(str(error))

    class ReplayNode(Node):
        def __init__(self):
            super().__init__("replay_ordinary_caster_joints")
            self.joints = self.create_publisher(JointState, args.joint_topic, 100)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status = self.create_publisher(String, "/ordinary_caster/replay/status", latched)
            self.marker = self.create_publisher(Marker, "/ordinary_caster/replay/status_marker", latched)
            self.clock_publisher = self.create_publisher(Clock, "/clock", 10) if args.publish_clock else None
            self.started = time.monotonic_ns()
            self.finished = None
            self.done = False
            self.timer = self.create_timer(1 / args.fps, self.publish,
                                          clock=ROSClock(clock_type=ClockType.STEADY_TIME))
            self.get_logger().info("Ordinary caster kinematic history replay. Wheel phase is relative. "
                                   "Missing joints are omitted; inspect /ordinary_caster/replay/status for gaps.")

        def publish(self):
            now = time.monotonic_ns()
            elapsed = max(0, now - self.started - int(args.start_delay * 1e9))
            source = min(history.end_ns, history.start_ns + int(elapsed * args.rate))
            frame = replay_frame(history, source, args.allow_uncalibrated)
            frame["end_of_history"] = source == history.end_ns
            if frame["end_of_history"] and self.finished is None:
                self.finished = now
            stamp_ns = frame["replay_time_ns"]
            # Explicit synthetic clock avoids pretending ROS receipt time was
            # acquisition time. Joint and clock timestamps share this origin.
            message = JointState()
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            message.name, message.position = frame["joint_names"], frame["positions"]
            if self.clock_publisher:
                clock = Clock()
                clock.clock = message.header.stamp
                self.clock_publisher.publish(clock)
            if message.name:
                self.joints.publish(message)
            status = String()
            status.data = json.dumps(frame, separators=(",", ":"), allow_nan=False)
            self.status.publish(status)
            marker = Marker()
            marker.header.frame_id = "ordinary_caster_history"
            marker.header.stamp = message.header.stamp
            marker.ns, marker.id = "ordinary_history_status", 0
            marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.pose.position.z = .15
            marker.scale.z, marker.scale.x = .025, .015
            marker.color.a = 1.0
            marker.color.r, marker.color.g, marker.color.b = ((.3, 1., .4) if frame["complete_pose_available"] else (1., .3, .1))
            marker.text = "Ordinary caster measured history\n" + "\n".join(
                f"{item['wheel_id']} {item['channel']}: {item['state']}" +
                (" (uncalibrated preview)" if item.get("uncalibrated_override") else "")
                for item in frame["channels"])
            self.marker.publish(marker)
            if args.exit_on_end and self.finished is not None and now - self.finished > 1_000_000_000:
                self.done = True
                self.timer.cancel()

    rclpy.init(args=raw_args)
    node = ReplayNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
