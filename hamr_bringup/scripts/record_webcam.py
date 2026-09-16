#!/usr/bin/env python3
"""Record a ROS camera stream, preserving source timestamps beside an MP4.

ROS/OpenCV imports are deliberately local so the recording lifecycle can be
tested on machines without ROS. Run this executable through recording.launch.py.
"""

from collections import deque
from dataclasses import dataclass
import csv
import json
import math
import os
from pathlib import Path
import signal
import sys
import time


MOTION_TOPICS = ('/left_wheel/cmd_vel', '/right_wheel/cmd_vel', '/turret/cmd_vel')
CSV_FIELDS = (
    'frame_index', 'video_time_sec', 'image_stamp_ns', 'receive_ros_ns',
    'receive_unix_ns', 'receive_monotonic_ns',
)


@dataclass
class FrameSample:
    """One BGR image and its independent acquisition/receipt time records."""

    image: object
    image_stamp_ns: int
    receive_ros_ns: int
    receive_unix_ns: int
    receive_monotonic_ns: int
    source_frame_id: str = ''


class RecordingSession:
    """Single-threaded recording state machine, independent of ROS and OpenCV."""

    def __init__(self, output_dir, writer_factory, *, video_fps=30.0,
                 codec='mp4v', start_on_motion=True, motion_threshold=0.01,
                 preroll_seconds=0.5, image_topic='/hamr_camera/image_raw',
                 on_frame=None, verify_video=None):
        self.output_dir = Path(output_dir)
        if not self.output_dir.is_absolute():
            raise ValueError('output_dir must be an absolute per-run directory')
        if not math.isfinite(video_fps) or video_fps <= 0:
            raise ValueError('video_fps must be finite and positive')
        if not math.isfinite(motion_threshold) or motion_threshold < 0:
            raise ValueError('motion_threshold must be finite and nonnegative')
        if not math.isfinite(preroll_seconds) or not 0 <= preroll_seconds <= 5:
            raise ValueError('preroll_seconds must be between 0 and 5')
        if len(codec) != 4:
            raise ValueError('codec must be a four-character OpenCV FourCC')
        self.video_path = self.output_dir / 'video.mp4'
        self.csv_path = self.output_dir / 'frame_timestamps.csv'
        self.metadata_path = self.output_dir / 'video_metadata.json'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        for path in (self.video_path, self.csv_path, self.metadata_path):
            if path.exists():
                raise FileExistsError(f'Refusing to overwrite recording: {path}')

        self.fps = video_fps
        self.codec = codec
        self.motion_threshold = motion_threshold
        self.preroll_ns = int(preroll_seconds * 1_000_000_000)
        # Time bound plus a hard memory bound: at most 60 frames are retained.
        self.preroll = deque(maxlen=min(60, max(1, math.ceil(video_fps * preroll_seconds) + 1)))
        self.writer_factory = writer_factory
        self.verify_video = verify_video
        self.on_frame = on_frame
        self.writer = None
        self.frame_size = None
        self.frame_count = 0
        self.received_count = 0
        self.recording_enabled = not start_on_motion
        self.closed = False
        self.created_monotonic_ns = time.monotonic_ns()
        self.last_receive_monotonic_ns = None
        self.last_flush_monotonic_ns = self.created_monotonic_ns
        self.metadata = {
            'format_version': 1,
            'state': 'waiting',
            'image_topic': image_topic,
            'motion_topics': list(MOTION_TOPICS),
            'start_on_motion': start_on_motion,
            'motion_threshold_rad_s': motion_threshold,
            'preroll_seconds': preroll_seconds,
            'preroll_max_frames': self.preroll.maxlen,
            'video_fps': video_fps,
            'codec': codec,
            'created_unix_ns': time.time_ns(),
            'motion_trigger': None,
            'first_image_stamp_ns': None,
            'last_image_stamp_ns': None,
            'nonincreasing_image_timestamps': 0,
            'source_frame_ids': [],
            'timestamp_notes': (
                'image_stamp_ns is the original Image.header.stamp from the USB camera '
                'driver; match it to localization message header stamps in the rosbag. '
                'The v4l2_camera driver stamps after capture using its ROS clock. '
                'This is software timing, not hardware-triggered exposure synchronization. '
                'receive_ros_ns is this node ROS clock; receive_unix_ns is host Unix time; '
                'receive_monotonic_ns is host monotonic time. video_time_sec is frame_index '
                '/ video_fps, for seeking only: dropped/delayed frames mean MP4 playback '
                'time is not the acquisition timeline. /hamr_camera/frame_stamp preserves '
                'each source stamp with frame_id=video_frame_<nine-digit frame_index>. '
                'The clip can include up to preroll_seconds before the first motion '
                'command. Motion is commanded wheel/turret speed, not measured motion.'
            ),
        }
        # Exclusive creation claims this run before any stream can overwrite it.
        with self.metadata_path.open('x', encoding='utf-8') as metadata_file:
            json.dump(self.metadata, metadata_file, indent=2)
        self.csv_file = self.csv_path.open('x', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_FIELDS)
        self.csv_file.flush()

    @property
    def state(self):
        if self.closed:
            return self.metadata['state']
        if not self.received_count:
            return 'waiting'
        return 'recording' if self.recording_enabled else 'armed'

    def observe_motion(self, value, topic, ros_ns, unix_ns, monotonic_ns):
        """Latch the first finite nonzero command; zero commands never stop a clip."""
        if self.closed or self.recording_enabled:
            return False
        if not math.isfinite(value) or abs(value) <= self.motion_threshold:
            return False
        self.recording_enabled = True
        self.metadata['motion_trigger'] = {
            'topic': topic, 'command_rad_s': value,
            'receive_ros_ns': ros_ns, 'receive_unix_ns': unix_ns,
            'receive_monotonic_ns': monotonic_ns,
        }
        self._trim_preroll(monotonic_ns)
        while self.preroll:
            self._encode(self.preroll.popleft())
        self.flush()
        return True

    def receive(self, sample):
        if self.closed:
            raise RuntimeError('Cannot add frames to a closed recording')
        self.received_count += 1
        self.last_receive_monotonic_ns = sample.receive_monotonic_ns
        if self.recording_enabled:
            self._encode(sample)
        elif self.preroll_ns:
            self.preroll.append(sample)
            self._trim_preroll(sample.receive_monotonic_ns)

    def _trim_preroll(self, now_ns):
        while self.preroll and now_ns - self.preroll[0].receive_monotonic_ns > self.preroll_ns:
            self.preroll.popleft()

    def _encode(self, sample):
        shape = sample.image.shape
        if len(shape) != 3 or shape[2] != 3:
            raise ValueError('Video encoder requires a three-channel BGR image')
        size = (int(shape[1]), int(shape[0]))
        if min(size) <= 0 or any(dimension % 2 for dimension in size):
            raise ValueError(f'Video frame dimensions must be positive and even, got {size}')
        if self.frame_size is not None and self.frame_size != size:
            raise ValueError(f'Camera resolution changed from {self.frame_size} to {size}')
        if self.writer is None:
            self.writer = self.writer_factory(str(self.video_path), self.codec, self.fps, size)
            if not self.writer.isOpened():
                raise RuntimeError(f'Cannot open {self.codec} encoder for {self.video_path}')
            self.frame_size = size
        if not self.writer.isOpened():
            raise RuntimeError('Video encoder closed unexpectedly')
        # OpenCV returns None on success, not a boolean. Exceptions and closed
        # writers are fatal; container frame count is verified after finalization.
        result = self.writer.write(sample.image)
        if result is False:
            raise RuntimeError('Video encoder rejected a frame')
        index = self.frame_count
        self.csv_writer.writerow((
            index, f'{index / self.fps:.9f}', sample.image_stamp_ns,
            sample.receive_ros_ns, sample.receive_unix_ns, sample.receive_monotonic_ns,
        ))
        previous = self.metadata['last_image_stamp_ns']
        if previous is not None and sample.image_stamp_ns <= previous:
            self.metadata['nonincreasing_image_timestamps'] += 1
        if self.frame_count == 0:
            self.metadata['first_image_stamp_ns'] = sample.image_stamp_ns
        self.metadata['last_image_stamp_ns'] = sample.image_stamp_ns
        if sample.source_frame_id not in self.metadata['source_frame_ids']:
            self.metadata['source_frame_ids'].append(sample.source_frame_id)
        self.frame_count += 1
        if self.on_frame:
            self.on_frame(index, sample)

    def check_health(self, now_ns, startup_timeout_sec=15.0, frame_timeout_sec=5.0):
        """Use monotonic receipt times, including when ROS simulation time stalls."""
        if self.last_receive_monotonic_ns is None:
            if now_ns - self.created_monotonic_ns > startup_timeout_sec * 1_000_000_000:
                raise RuntimeError('No camera images received before startup timeout')
        elif now_ns - self.last_receive_monotonic_ns > frame_timeout_sec * 1_000_000_000:
            raise RuntimeError('Camera image stream stopped; check the USB connection')
        if now_ns - self.last_flush_monotonic_ns >= 1_000_000_000:
            self.flush()
            self.last_flush_monotonic_ns = now_ns

    def flush(self):
        self.csv_file.flush()
        self.metadata.update({
            'state': self.state,
            'frames_written': self.frame_count,
            'images_received': self.received_count,
            'frame_size': list(self.frame_size) if self.frame_size else None,
            'video_file': self.video_path.name if self.frame_count else None,
            'timestamps_file': self.csv_path.name,
        })
        temporary_path = self.metadata_path.with_suffix('.json.tmp')
        with temporary_path.open('w', encoding='utf-8') as metadata_file:
            json.dump(self.metadata, metadata_file, indent=2)
            metadata_file.write('\n')
        temporary_path.replace(self.metadata_path)

    def close(self, error=None, stop_reason='shutdown'):
        """Always release the encoder and CSV, even when final validation fails."""
        if self.closed:
            return
        close_error = None
        try:
            if self.writer is not None:
                self.writer.release()
            if self.frame_count and self.verify_video:
                self.verify_video(str(self.video_path), self.frame_count)
        except Exception as exc:
            close_error = exc
        self.closed = True
        self.preroll.clear()
        self.metadata.update({
            'state': 'failed' if error or close_error else (
                'completed' if self.frame_count else 'no_frames'),
            'error': str(error or close_error) if error or close_error else None,
            'close_error': str(close_error) if close_error else None,
            'stop_reason': stop_reason,
            'closed_unix_ns': time.time_ns(),
        })
        try:
            self.flush()
            os.fsync(self.csv_file.fileno())
        finally:
            self.csv_file.close()
        if close_error:
            raise close_error


def create_ros_node():
    import cv2
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float64, Header, String

    class WebcamRecorder(Node):
        def __init__(self):
            super().__init__('webcam_recorder')
            defaults = {
                'output_dir': '', 'image_topic': '/hamr_camera/image_raw',
                'start_on_motion': True, 'motion_threshold': 0.01,
                'video_fps': 30.0, 'codec': 'mp4v', 'preroll_seconds': 0.5,
                'startup_timeout_sec': 15.0, 'frame_timeout_sec': 5.0,
            }
            values = {name: self.declare_parameter(name, default).value
                      for name, default in defaults.items()}
            for name in ('startup_timeout_sec', 'frame_timeout_sec'):
                if not math.isfinite(values[name]) or values[name] <= 0:
                    raise ValueError(f'{name} must be finite and positive')
            self.startup_timeout = values.pop('startup_timeout_sec')
            self.frame_timeout = values.pop('frame_timeout_sec')
            self.bridge = CvBridge()
            self.frame_publisher = self.create_publisher(Header, '/hamr_camera/frame_stamp', 100)
            self.status_publisher = self.create_publisher(
                String, '/hamr_camera/recording_status',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.previous_status = None
            self.session = RecordingSession(
                writer_factory=lambda path, codec, fps, size: cv2.VideoWriter(
                    path, cv2.VideoWriter_fourcc(*codec), fps, size),
                on_frame=self.publish_frame_stamp,
                verify_video=self.verify_video,
                **values,
            )
            try:
                self.create_subscription(
                    Image, values['image_topic'], self.receive_image, qos_profile_sensor_data)
                for topic in MOTION_TOPICS:
                    self.create_subscription(
                        Float64, topic,
                        lambda message, source=topic: self.receive_motion(message, source),
                        qos_profile_sensor_data)
                self.publish_status()
                self.get_logger().info(
                    f'Waiting for {values["image_topic"]}; recording directory: '
                    f'{values["output_dir"]}')
            except Exception as exc:
                self.session.close(error=exc, stop_reason='initialization_failure')
                raise

        def receive_image(self, message):
            # Capture receipt times before conversion or compression work.
            ros_ns = self.get_clock().now().nanoseconds
            unix_ns = time.time_ns()
            monotonic_ns = time.monotonic_ns()
            sample = FrameSample(
                image=self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8'),
                image_stamp_ns=message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec,
                receive_ros_ns=ros_ns, receive_unix_ns=unix_ns,
                receive_monotonic_ns=monotonic_ns,
                source_frame_id=message.header.frame_id,
            )
            self.session.receive(sample)
            self.publish_status()

        def receive_motion(self, message, topic):
            started = self.session.observe_motion(
                message.data, topic, self.get_clock().now().nanoseconds,
                time.time_ns(), time.monotonic_ns())
            if started:
                self.get_logger().info(f'Motion command on {topic}: recording until shutdown')
            self.publish_status()

        def publish_frame_stamp(self, index, sample):
            header = Header()
            header.stamp.sec, header.stamp.nanosec = divmod(sample.image_stamp_ns, 1_000_000_000)
            header.frame_id = f'video_frame_{index:09d}'
            self.frame_publisher.publish(header)

        def publish_status(self):
            status = self.session.state
            if status != self.previous_status:
                self.status_publisher.publish(String(data=status))
                self.get_logger().info(f'Camera recorder: {status}')
                self.previous_status = status

        @staticmethod
        def verify_video(path, expected_frames):
            capture = cv2.VideoCapture(path)
            try:
                if not capture.isOpened():
                    raise RuntimeError(f'Finalized video cannot be opened: {path}')
                actual_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
                readable, _ = capture.read()
                if actual_frames != expected_frames or not readable:
                    raise RuntimeError(
                        f'Video validation failed: {actual_frames} frames in MP4 versus '
                        f'{expected_frames} timestamp rows; first frame readable={readable}')
            finally:
                capture.release()

    return WebcamRecorder()


def main(args=None):
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    stop_signal = None

    def request_stop(signum, _frame):
        nonlocal stop_signal
        stop_signal = signum

    previous_handlers = {}
    for signal_name in ('SIGINT', 'SIGTERM', 'SIGHUP'):
        if hasattr(signal, signal_name):
            number = getattr(signal, signal_name)
            previous_handlers[number] = signal.signal(number, request_stop)
    node = None
    error = None
    try:
        # Keep the ROS context alive while finally flushes/releases the MP4.
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = create_ros_node()
        while rclpy.ok() and stop_signal is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if stop_signal is None:
                node.session.check_health(
                    time.monotonic_ns(), node.startup_timeout, node.frame_timeout)
    except Exception as exc:
        error = exc
        print(f'Webcam recording failed: {exc}', file=sys.stderr, flush=True)
    finally:
        if node is not None:
            try:
                reason = signal.Signals(stop_signal).name if stop_signal else 'shutdown'
                node.session.close(error=error, stop_reason=reason)
            except Exception as exc:
                error = error or exc
                print(f'Webcam finalization failed: {exc}', file=sys.stderr, flush=True)
            finally:
                # Do not attempt ROS publication after external context shutdown.
                if rclpy.ok():
                    node.publish_status()
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown(uninstall_handlers=False)
        for number, handler in previous_handlers.items():
            signal.signal(number, handler)
    return 1 if error else 0


if __name__ == '__main__':
    sys.exit(main())
