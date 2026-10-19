"""ROS telemetry and simulation-time recordings for the HAMR desktop console.

The encoder envelope matches the hardware logger's JSON wire schema. Its
metadata explicitly identifies simulation and the source joints: a swivel
fork is not a hemisphere encoder, despite sharing the H1/H2 transport slots.
ROS is imported only when connecting so the conversion helpers are testable
without a running simulator.
"""

from collections import Counter, deque
import csv
import json
import math
import os
from pathlib import Path
import secrets
import shutil
import signal
import subprocess
import sys
import threading
import time


ROLES = {1: "H1", 2: "H2", 3: "CENTER"}
ENCODER_NAMES = {1: "caster_h1_rotation", 2: "caster_h2_rotation", 3: "caster_center_alpha"}
JOINT_MAPPING = {
    "swivel": {1: "left_caster_swivel_joint", 2: "right_caster_swivel_joint",
               3: "pitch_yaw_plate_joint"},
    "ball": {1: "right_caster_positive_hemisphere_joint",
             2: "right_caster_negative_hemisphere_joint", 3: "right_caster_carrier_joint"},
}
DIAGNOSTICS = {name: 0 for name in ("acquisition_overruns", "queue_drops",
                                 "invalid_spi_responses", "index_queue_drops", "radio_failures")}


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def finite(value):
    return isinstance(value, (float, int)) and math.isfinite(value)


def joint_readings(names, positions, velocities=()):
    """Ignore absent or nonfinite measurements instead of inventing zeroes."""
    result = {}
    for index, name in enumerate(names):
        if index >= len(positions) or not finite(positions[index]):
            continue
        velocity = velocities[index] if index < len(velocities) else None
        result[name] = {"position": float(positions[index]),
                        "velocity": float(velocity) if finite(velocity) else None}
    return result


class EncoderAdapter:
    """Create hardware-schema packets with explicit simulated joint semantics."""

    def __init__(self, caster="ball"):
        if caster not in JOINT_MAPPING:
            raise ValueError("caster must be swivel or ball")
        self.caster = caster
        self.mapping = JOINT_MAPPING[caster]
        self.reset()

    def reset(self):
        self.boot_id = secrets.token_hex(8)
        self.sequence = {node: 0 for node in ROLES}
        self.last_stamp = None

    def metadata(self):
        return {
            "version": 1, "source": "gazebo_simulation", "caster": self.caster,
            "timestamp_basis": "simulation_clock", "synthetic": True,
            "hardware_calibration": False,
            "replay_note": ("Use this run's Gazebo state log to replay the full vehicle. "
                            "H1/H2/CENTER are transport slots; read source_joint before "
                            "using a hardware CAD replay mapping."),
            "nodes": {str(node): {
                "v": 1, "type": "metadata", "node_id": node, "boot_id": self.boot_id,
                "role": role, "firmware_version": "hamr_sim_ui-synthetic-v1",
                "idf_version": "not-applicable-simulation", "source_joint": self.mapping[node],
                "angle_units": "radians", "angle_sign": 1,
                "source": "gazebo_simulation", "timestamp_basis": "simulation_clock",
            } for node, role in ROLES.items()},
        }

    def envelopes(self, readings, simulation_ns):
        if type(simulation_ns) is not int or simulation_ns < 0:
            raise ValueError("simulation_ns must be a nonnegative integer")
        # A restart begins a new encoder segment; consumers never combine
        # unrelated unwrapped joint histories under one boot ID.
        if self.last_stamp is not None and simulation_ns < self.last_stamp:
            self.reset()
        self.last_stamp = simulation_ns
        result = []
        for node, name in self.mapping.items():
            reading = readings.get(name)
            if reading is None or not finite(reading.get("position")):
                continue
            angle = float(reading["position"])
            velocity = reading.get("velocity")
            velocity = float(velocity) if finite(velocity) else None
            flags = (1 << 0) | (1 << 1) | (1 << 4)
            if velocity is not None:
                flags |= 1 << 6
            sample = {
                "seq": self.sequence[node], "t_us": simulation_ns // 1000,
                "raw_count": None, "raw_spi_word": None, "position14": None,
                "angle_rad": angle, "absolute_angle_rad": angle % math.tau,
                "calibrated_alpha_rad": angle if node == 3 else None,
                "velocity_rad_s": velocity, "filtered_velocity_rad_s": None,
                "index_event_count": 0, "index_timestamp_us": 0, "index_raw_count": None,
                "flags": flags,
            }
            self.sequence[node] += 1
            packet = {"v": 1, "type": "samples", "node_id": node,
                      "boot_id": self.boot_id, "samples": [sample],
                      "diagnostics": dict(DIAGNOSTICS)}
            result.append({
                "receive_epoch_ns": time.time_ns(), "receive_monotonic_ns": time.monotonic_ns(),
                "receive_ros_ns": simulation_ns, "source_ip": "gazebo-simulation",
                "source_port": 0, "source": "gazebo_simulation", "synthetic": True,
                "source_joint": name, "timestamp_basis": "simulation_clock", "packet": packet,
            })
        return result


def image_frame(message):
    """Normalize ROS RGB/BGR/RGBA/mono images, honoring row padding."""
    import numpy as np

    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1,
                "8UC3": 3, "8UC1": 1}.get(message.encoding)
    if channels is None:
        raise ValueError(f"Unsupported camera encoding: {message.encoding}")
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width < 1 or height < 1 or step < width * channels or len(message.data) < step * height:
        raise ValueError("Camera image has invalid dimensions or truncated pixel data")
    pixels = np.frombuffer(message.data, dtype=np.uint8, count=step * height)
    pixels = pixels.reshape(height, step)[:, :width * channels].reshape(height, width, channels)
    if channels == 1:
        pixels = np.repeat(pixels, 3, axis=2)
    else:
        pixels = pixels[:, :, :3]
        if message.encoding in ("bgr8", "bgra8"):
            pixels = pixels[:, :, ::-1]
    return {"width": width, "height": height, "step": width * 3, "encoding": "rgb8",
            "data": pixels.tobytes(), "stamp_ns": stamp_ns(message.header.stamp)}


class BagRecording:
    """One recording with raw ROS data, long-form CSV and packet JSONL."""

    def __init__(self, output_dir, metadata):
        import rosbag2_py

        self.output = Path(output_dir).expanduser().resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        if any((self.output / name).exists() for name in
               ("bag", "velocity.csv", "joints.csv", "encoder_packets.jsonl", "recording.json")):
            raise FileExistsError(f"Recording already exists in {self.output}; choose a fresh directory")
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(self.output / "bag"), storage_id="sqlite3"),
                         rosbag2_py.ConverterOptions("cdr", "cdr"))
        self._topic_type = rosbag2_py.TopicMetadata
        self.topics = {}
        self.counts = Counter()
        self.errors = []
        self.first_ns = None
        self.last_ns = None
        self.closed = False
        self.velocity_file = (self.output / "velocity.csv").open("x", newline="")
        self.joint_file = (self.output / "joints.csv").open("x", newline="")
        self.packet_file = (self.output / "encoder_packets.jsonl").open("x")
        self.velocity = csv.writer(self.velocity_file)
        self.joints = csv.writer(self.joint_file)
        self.velocity.writerow(("simulation_ns", "x_m", "y_m", "yaw_rad", "vx_m_s", "vy_m_s",
                                "speed_m_s", "yaw_rate_rad_s"))
        self.joints.writerow(("simulation_ns", "joint", "position_rad", "velocity_rad_s"))
        self.metadata = dict(metadata)
        self.metadata.update({"output_dir": str(self.output), "bag": str(self.output / "bag"),
                              "recording_complete": False, "images": "MP4 sidecars when enabled"})
        self._manifest()

    def write(self, topic, type_name, message, timestamp):
        from rclpy.serialization import serialize_message

        if self.closed:
            raise RuntimeError("Cannot append to closed recording")
        if topic not in self.topics:
            self.writer.create_topic(self._topic_type(id=len(self.topics), name=topic,
                                                      type=type_name, serialization_format="cdr"))
            self.topics[topic] = type_name
        self.writer.write(topic, serialize_message(message), int(timestamp))
        self.counts[topic] += 1
        self.first_ns = timestamp if self.first_ns is None else min(self.first_ns, timestamp)
        self.last_ns = timestamp if self.last_ns is None else max(self.last_ns, timestamp)

    def _manifest(self):
        temporary = self.output / "recording.json.tmp"
        temporary.write_text(json.dumps(self.metadata, indent=2, allow_nan=False) + "\n")
        temporary.replace(self.output / "recording.json")

    def close(self, video_results=None):
        if self.closed:
            return dict(self.metadata)
        self.writer.close()
        self.writer = None
        for stream in (self.velocity_file, self.joint_file, self.packet_file):
            stream.close()
        self.closed = True
        for result in video_results or []:
            if not result.get("success", False):
                self.errors.append(f"{result['camera']} video failed verification; inspect its recorder log")
        self.metadata.update({"recording_complete": not self.errors, "errors": self.errors,
                              "message_counts": dict(self.counts),
                              "first_simulation_ns": self.first_ns, "last_simulation_ns": self.last_ns,
                              "video": video_results or [],
                              "video_results": {result["camera"]: result for result in video_results or []}})
        self._manifest()
        return dict(self.metadata)


class RosTelemetry:
    """Own a ROS executor thread; UI reads snapshots without touching ROS objects."""

    def __init__(self, caster="ball", on_update=None, on_image=None, cameras=True,
                 preview_cameras=None, preview_fps=15.):
        if not finite(preview_fps) or preview_fps <= 0:
            raise ValueError("preview_fps must be positive and finite")
        self._preview_cameras = None if preview_cameras is None else frozenset(preview_cameras)
        if self._preview_cameras is not None and not self._preview_cameras <= {"closeup", "overview"}:
            raise ValueError("preview_cameras must contain closeup or overview")
        self._preview_interval = 1. / preview_fps
        self._last_preview_wall = {}
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Float64, String
        from tf2_msgs.msg import TFMessage

        self._lock = threading.RLock()
        self._on_update, self._on_image = on_update, on_image
        self._context = Context()
        rclpy.init(context=self._context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node("hamr_sim_ui_telemetry", context=self._context,
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self.node)
        self._string, self._joint_type = String, JointState
        self._image_type, self._sensor_qos = Image, qos_profile_sensor_data
        self._camera_subscriptions = []
        self.cameras_enabled = bool(cameras)
        self.encoder = EncoderAdapter(caster)
        self._recording = None
        self._video_processes = []
        self._closed = False
        self._last_update = 0.0
        self._last_history_ns = None
        self._cached = {}
        self._trail = deque(maxlen=10000)
        self._history = deque(maxlen=3000)
        self._clock_samples = deque(maxlen=21)
        self._camera_stats = {}
        self._state = {"sim_time": 0.0, "x": None, "y": None, "yaw": None,
                       "linear_speed": None, "vx": None, "vy": None, "yaw_rate": None,
                       "casters": {}, "joints": {}, "recording": False, "last_error": None,
                       "odom_received": False, "joints_received": False}
        reliable = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._packet_pub = self.node.create_publisher(String, "/caster_encoder/packets", reliable)
        self._metadata_pub = self.node.create_publisher(String, "/caster_encoder/metadata", latched)
        self._encoder_pubs = {node: self.node.create_publisher(
            JointState, f"/caster_encoder/{role}/joint_states", reliable) for node, role in ROLES.items()}
        self.node.create_subscription(Odometry, "/compa/odom", self._odom, qos_profile_sensor_data)
        self.node.create_subscription(JointState, "/joint_states", self._joints, qos_profile_sensor_data)
        self.node.create_subscription(Clock, "/clock", self._clock, qos_profile_sensor_data)
        self.node.create_subscription(TFMessage, "/tf", lambda msg: self._raw(
            "/tf", "tf2_msgs/msg/TFMessage", msg), qos_profile_sensor_data)
        self.node.create_subscription(TFMessage, "/tf_static", lambda msg: self._raw(
            "/tf_static", "tf2_msgs/msg/TFMessage", msg), latched)
        self._configure_camera_subscriptions(caster, cameras)
        for topic in ("/left_wheel/cmd_vel", "/right_wheel/cmd_vel", "/roll/cmd_vel",
                      "/pitch/cmd_vel", "/yaw/cmd_vel"):
            self.node.create_subscription(Float64, topic, lambda msg, topic=topic: self._raw(
                topic, "std_msgs/msg/Float64", msg), qos_profile_sensor_data)
        self._publish_metadata()
        self._thread = threading.Thread(target=self._spin, name="hamr-telemetry", daemon=True)
        self._thread.start()

    def _configure_camera_subscriptions(self, caster, cameras):
        for subscription in self._camera_subscriptions:
            self.node.destroy_subscription(subscription)
        self._camera_subscriptions.clear()
        self._last_preview_wall.clear()
        self.cameras_enabled = bool(cameras)
        self.camera_topics = ({"closeup": f"/hamr/{caster}_caster/closeup/image",
                               "overview": "/hamr/recording_camera/image"}
                              if self.cameras_enabled else {})
        for camera, topic in self.camera_topics.items():
            # MP4 writers subscribe themselves; do not deserialize an entire
            # RGB stream here when this client has no preview for that camera.
            if self._preview_cameras is not None and camera not in self._preview_cameras:
                continue
            subscription = self.node.create_subscription(
                self._image_type, topic, lambda msg, camera=camera: self._image(camera, msg),
                self._sensor_qos)
            self._camera_subscriptions.append(subscription)

    def configure(self, caster, cameras=True):
        """Reconfigure a stopped scene without replacing the controller's node.

        Run subscription changes on the executor thread so a camera callback
        cannot still be using an entity while it is destroyed. The caller must
        stop the previous simulation before invoking this method.
        """
        replacement = EncoderAdapter(caster)
        with self._lock:
            if self._recording:
                raise RuntimeError("Stop recording before switching scenes")
            if self._closed:
                raise RuntimeError("ROS telemetry is closed")
        completed = threading.Event()
        errors = []

        def apply():
            try:
                with self._lock:
                    if self._recording:
                        raise RuntimeError("Stop recording before switching scenes")
                    self._configure_camera_subscriptions(caster, cameras)
                    self.encoder = replacement
                    self.reset()
            except Exception as error:
                errors.append(error)
            finally:
                completed.set()

        if threading.get_ident() == self._thread.ident:
            apply()
        else:
            future = self._executor.create_task(apply)
            if not completed.wait(10.):
                future.cancel()
                raise RuntimeError("ROS telemetry did not respond while switching scenes")
        if errors:
            raise errors[0]

    def _spin(self):
        try:
            self._executor.spin()
        except Exception as error:
            if not self._closed:
                with self._lock:
                    self._state["last_error"] = f"ROS telemetry stopped: {error}"

    def _json(self, value):
        message = self._string()
        message.data = json.dumps(value, separators=(",", ":"), allow_nan=False)
        return message

    def _publish_metadata(self):
        message = self._json(self.encoder.metadata())
        self._metadata_pub.publish(message)
        self._raw("/caster_encoder/metadata", "std_msgs/msg/String", message)

    def _now(self):
        return self.node.get_clock().now().nanoseconds

    def _raw(self, topic, type_name, message, timestamp=None):
        with self._lock:
            self._cached[topic] = (type_name, message)
            if self._recording:
                try:
                    self._recording.write(topic, type_name, message,
                                          self._now() if timestamp is None else timestamp)
                except Exception as error:
                    self._state["last_error"] = f"Recording write failed: {error}"
                    if not self._recording.errors or self._recording.errors[-1] != str(error):
                        self._recording.errors.append(str(error))

    def _clock(self, message):
        value = stamp_ns(message.clock)
        with self._lock:
            if value / 1e9 < self._state["sim_time"]:
                self._trail.clear()
                self._history.clear()
                self._last_history_ns = None
                self._clock_samples.clear()
            self._state["sim_time"] = value / 1e9
            wall = time.monotonic()
            if not self._clock_samples or wall - self._clock_samples[-1][0] >= 0.1:
                self._clock_samples.append((wall, value / 1e9))
        self._raw("/clock", "rosgraph_msgs/msg/Clock", message, value)

    def _odom(self, message):
        position, rotation = message.pose.pose.position, message.pose.pose.orientation
        twist = message.twist.twist
        yaw = math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                         1 - 2 * (rotation.y ** 2 + rotation.z ** 2))
        values = [position.x, position.y, yaw, twist.linear.x, twist.linear.y, twist.angular.z]
        if not all(finite(value) for value in values):
            return
        timestamp = stamp_ns(message.header.stamp)
        speed = math.hypot(twist.linear.x, twist.linear.y)
        with self._lock:
            self._state.update(x=position.x, y=position.y, yaw=yaw, vx=twist.linear.x,
                               vy=twist.linear.y, linear_speed=speed, yaw_rate=twist.angular.z,
                               odom_received=True)
            if not self._trail or math.dist(self._trail[-1], (position.x, position.y)) >= 0.01:
                self._trail.append((position.x, position.y))
            self._raw("/compa/odom", "nav_msgs/msg/Odometry", message, timestamp)
            if self._recording:
                self._recording.velocity.writerow((timestamp, position.x, position.y, yaw,
                                                    twist.linear.x, twist.linear.y, speed, twist.angular.z))
        self._notify()

    def _joints(self, message):
        timestamp = stamp_ns(message.header.stamp)
        readings = joint_readings(message.name, message.position, message.velocity)
        with self._lock:
            self._state["joints"].update(readings)
            self._state["casters"].update((name, reading) for name, reading in readings.items()
                                           if "caster" in name)
            self._state["joints_received"] = True
            if self._last_history_ns is None or timestamp - self._last_history_ns >= 50_000_000:
                self._history.append({"sim_time": timestamp / 1e9,
                                       **{name: reading["position"] for name, reading in
                                          self._state["casters"].items()}})
                self._last_history_ns = timestamp
            self._raw("/joint_states", "sensor_msgs/msg/JointState", message, timestamp)
            if self._recording:
                for name, reading in readings.items():
                    self._recording.joints.writerow((timestamp, name, reading["position"],
                                                     reading["velocity"]))
            boot_id = self.encoder.boot_id
            envelopes = self.encoder.envelopes(readings, timestamp)
            if boot_id != self.encoder.boot_id:
                self._publish_metadata()
            # Sequence numbers still advance for every acquired sample. Avoid
            # constructing unused derived ROS messages during ordinary driving;
            # recordings always retain the complete stream regardless of DDS
            # subscribers. Check discovery each callback so a new subscriber
            # receives the next sample without a preview-rate delay.
            publish_packets = self._recording is not None or self._packet_pub.get_subscription_count() > 0
            for envelope in envelopes:
                if publish_packets:
                    packet_message = self._json(envelope)
                    self._packet_pub.publish(packet_message)
                    self._raw("/caster_encoder/packets", "std_msgs/msg/String", packet_message, timestamp)
                    if self._recording:
                        self._recording.packet_file.write(packet_message.data + "\n")
                node = envelope["packet"]["node_id"]
                publisher = self._encoder_pubs[node]
                if self._recording is None and publisher.get_subscription_count() == 0:
                    continue
                sample = envelope["packet"]["samples"][0]
                joint = self._joint_type()
                joint.header.stamp = message.header.stamp
                joint.name = [ENCODER_NAMES[node]]
                joint.position = [sample["angle_rad"]]
                joint.velocity = [] if sample["velocity_rad_s"] is None else [sample["velocity_rad_s"]]
                publisher.publish(joint)
                self._raw(f"/caster_encoder/{ROLES[node]}/joint_states", "sensor_msgs/msg/JointState",
                          joint, timestamp)
        self._notify()

    def _image(self, camera, message):
        with self._lock:
            if not self.cameras_enabled:
                return
            now = time.monotonic()
            previous = self._camera_stats.get(camera, {})
            self._camera_stats[camera] = {"frames": previous.get("frames", 0) + 1,
                                          "stamp_ns": stamp_ns(message.header.stamp),
                                          "wall": now}
            # The recorder subscribes to the raw image topic independently.
            # Only the on-screen preview is filtered and paced in wall time.
            if (not self._on_image or
                    (self._preview_cameras is not None and camera not in self._preview_cameras) or
                    now - self._last_preview_wall.get(camera, -math.inf) < self._preview_interval):
                return
            self._last_preview_wall[camera] = now
        if self._on_image:
            try:
                self._on_image(camera, image_frame(message))
            except Exception as error:
                with self._lock:
                    self._state["last_error"] = f"Camera {camera}: {error}"

    def _notify(self):
        now = time.monotonic()
        if self._on_update and now - self._last_update >= 0.05:
            self._last_update = now
            self._on_update(self.snapshot())

    def snapshot(self, *, include_history=True, include_trail=True):
        """Copy current state, optionally omitting unused accumulated displays.

        Omitting a history affects this snapshot only. Raw recordings and the
        internally retained samples are unaffected.
        """
        with self._lock:
            value = dict(self._state)
            for key in ("joints", "casters"):
                value[key] = {name: dict(reading) for name, reading in self._state[key].items()}
            if include_trail:
                value["trail"] = list(self._trail)
            if include_history:
                value["history"] = [dict(point) for point in self._history]
            now = time.monotonic()
            value["cameras"] = {camera: {"frames": stats["frames"], "stamp_ns": stats["stamp_ns"],
                                         "age_s": now - stats["wall"]}
                                for camera, stats in self._camera_stats.items()}
            value["cameras_enabled"] = self.cameras_enabled
            value["caster"] = self.encoder.caster
            value["real_time_factor"] = None
            if self._clock_samples:
                oldest_wall, oldest_sim = self._clock_samples[0]
                if now - self._clock_samples[-1][0] > 0.5:
                    value["real_time_factor"] = 0.0
                elif now - oldest_wall > 0.2:
                    value["real_time_factor"] = max(0.0, (self._state["sim_time"] - oldest_sim) /
                                                    (now - oldest_wall))
            for camera, process, _ in self._video_processes:
                if process.poll() is not None:
                    value["last_error"] = f"{camera} recorder stopped early ({process.returncode}); stop and inspect recording logs"
            return value

    def clear_trail(self):
        with self._lock:
            self._trail.clear()

    def reset(self, *, preserve_clock=False):
        """Clear measurements without replacing the DDS graph.

        A placement changes pose within the current world, so preserve its
        displayed simulation time even if physics is paused. Scene changes
        keep the default behavior of resetting time to zero.
        """
        with self._lock:
            if self._recording:
                raise RuntimeError("Stop recording before switching scenes")
            simulation_time = self._state["sim_time"] if preserve_clock else 0.0
            self.encoder.reset()
            self._cached.clear()
            self._trail.clear()
            self._history.clear()
            self._clock_samples.clear()
            self._camera_stats.clear()
            self._last_preview_wall.clear()
            self._last_history_ns = None
            self._last_update = 0.0
            self._state.update(sim_time=simulation_time, x=None, y=None, yaw=None, linear_speed=None,
                               vx=None, vy=None, yaw_rate=None, casters={}, joints={},
                               last_error=None, odom_received=False, joints_received=False)
            self._publish_metadata()

    def start_recording(self, output_dir, video=True, metadata=None, session_metadata=None):
        if video and not self.cameras_enabled:
            raise RuntimeError("MP4 recording needs camera sensors; enable cameras and reload the world, "
                               "or record ROS bag/CSV/encoder packets without MP4")
        if video and not shutil.which("ffmpeg"):
            raise RuntimeError("MP4 recording needs ffmpeg; source the HAMR environment or install ffmpeg")
        if video:
            from ament_index_python.packages import get_package_prefix
            recorder = Path(get_package_prefix("hamr_ball_caster")) / "lib/hamr_ball_caster/record_simulation.py"
            if not recorder.is_file():
                raise RuntimeError(f"Camera recorder is not installed: {recorder}")
        with self._lock:
            if self._recording is not None:
                raise RuntimeError("A recording is already running")
            provenance = self.encoder.metadata()
            provenance["session"] = dict(session_metadata or metadata or {})
            provenance["start_simulation_ns"] = self._now()
            self._recording = BagRecording(output_dir, provenance)
            self._state["recording"] = True
            for topic in ("/tf_static", "/caster_encoder/metadata"):
                if topic in self._cached:
                    kind, message = self._cached[topic]
                    self._recording.write(topic, kind, message, self._now())
            output = self._recording.output
        try:
            if video:
                for camera, topic in self.camera_topics.items():
                    log = (output / f"{camera}.recorder.log").open("w")
                    process = subprocess.Popen(
                        [sys.executable, str(recorder), "--no-bridge", "--topic", topic,
                         "--output", str(output / f"{camera}.mp4"), "--fps", "25",
                         "--ready-file", str(output / f"{camera}.ready.json")],
                        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    self._video_processes.append((camera, process, log))
                deadline = time.monotonic() + 20.0
                while not all((output / f"{camera}.ready.json").is_file()
                              for camera in self.camera_topics):
                    for camera, process, _ in self._video_processes:
                        if process.poll() is not None:
                            raise RuntimeError(f"{camera} recorder exited before its first frame; see {output}")
                    if time.monotonic() >= deadline:
                        raise RuntimeError("No encoded frames from both cameras within 20 seconds; "
                                           "resume physics and check camera streams before recording")
                    time.sleep(0.05)
        except Exception as error:
            with self._lock:
                if self._recording:
                    self._recording.errors.append(str(error))
            self.stop_recording()
            raise
        return output

    def stop_recording(self):
        with self._lock:
            recording, self._recording = self._recording, None
            self._state["recording"] = False
        if recording is None:
            return {}
        processes, self._video_processes = self._video_processes, []
        for _, process, _ in processes:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        results = []
        for camera, process, log in processes:
            try:
                process.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            log.close()
            video = recording.output / f"{camera}.mp4"
            report_file = video.with_suffix(".mp4.json")
            try:
                report = json.loads(report_file.read_text())
            except (ValueError, OSError):
                report = {}
            success = process.returncode == 0 and video.is_file() and report.get("success") is True
            results.append({"camera": camera, "returncode": process.returncode,
                            "path": str(video), "exists": video.is_file(),
                            "success": success, "frames_encoded": report.get("frames_encoded", 0),
                            "first_sim_time_s": report.get("first_sim_time_s"),
                            "last_sim_time_s": report.get("last_sim_time_s"),
                            "timing_report": str(report_file)})
        summary = recording.close(results)
        if not summary["recording_complete"]:
            with self._lock:
                self._state["last_error"] = "; ".join(summary["errors"])
        return summary

    def close(self):
        if self._closed:
            return
        self.stop_recording()
        self._closed = True
        self._executor.shutdown(timeout_sec=3.0)
        self._thread.join(timeout=3.0)
        self.node.destroy_node()
        self._context.shutdown()
