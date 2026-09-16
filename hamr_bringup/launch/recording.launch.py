"""Record a USB camera and localization into a shared, unique run directory."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError(f'{name} must be true or false')
    return value == 'true'


def _launch_recorders(context):
    record_video = _boolean(context, 'record_video')
    record_bag = _boolean(context, 'record_bag')
    if not (record_video or record_bag):
        return []

    def value(name):
        return LaunchConfiguration(name).perform(context)

    run_name = value('run_name') or datetime.now(timezone.utc).strftime(
        'hamr_%Y%m%d_%H%M%S_%fZ')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', run_name):
        raise ValueError('run_name must be a single filename starting with a letter or number')
    root = Path(os.path.expandvars(value('recording_root'))).expanduser().resolve()
    run_dir = root / run_name
    use_sim_time = _boolean(context, 'use_sim_time')
    start_on_motion = _boolean(context, 'start_on_motion')
    record_images = _boolean(context, 'record_images')
    width, height = int(value('camera_width')), int(value('camera_height'))
    fps = float(value('video_fps'))
    threshold = float(value('motion_threshold'))
    if width <= 0 or height <= 0 or not (0 < fps <= 240) or not (0 <= threshold < float('inf')):
        raise ValueError('Camera dimensions/fps must be positive; threshold must be finite >= 0')

    config_dir = Path(get_package_share_directory('hamr_bringup')) / 'config'
    topic_regex = (config_dir / 'recording_topics.regex').read_text().strip()
    if record_images:
        topic_regex += r'|^/hamr_camera/image_raw$'
    # Create only the parent: rosbag2 must create its own fresh `bag` subdirectory.
    run_dir.mkdir(parents=True, exist_ok=False)
    manifest = {
        'run_name': run_name,
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'record_video': record_video, 'record_bag': record_bag,
        'record_images': record_images, 'use_sim_time': use_sim_time,
        'camera_device': value('camera_device'),
        'requested_image_size': [width, height],
        'video_fps': fps, 'start_on_motion': start_on_motion,
        'motion_threshold_rad_sec': threshold,
        'bag_path': 'bag' if record_bag else None,
        'video_path': 'video.mp4' if record_video else None,
        'frame_timestamps_path': 'frame_timestamps.csv' if record_video else None,
        'bag_topic_regex': topic_regex,
        'synchronization': (
            'Hardware: compare CSV image_stamp_ns to localization header stamps on '
            'synchronized system clocks. For Gazebo compare receive_ros_ns to simulation '
            'stamps; USB camera image stamps remain system time. '
            'Camera driver timestamps are not a hardware trigger shared with Vicon.'
        ),
    }
    (run_dir / 'run.json').write_text(json.dumps(manifest, indent=2) + '\n')
    actions = [LogInfo(msg=f'HAMR recording directory: {run_dir}')]

    def stop_on_exit(component):
        return [EmitEvent(event=Shutdown(reason=f'{component} exited; closing this run'))]

    # Start bag discovery before camera/recorder. It stays active while video is armed.
    if record_bag:
        cmd = [
            'ros2', 'bag', 'record', '--storage', value('bag_storage'),
            '--output', str(run_dir / 'bag'), '--regex', topic_regex,
            '--qos-profile-overrides-path', str(config_dir / 'recording_qos.yaml'),
            '--disable-keyboard-controls',
        ]
        if use_sim_time:
            cmd.append('--use-sim-time')
        actions.append(ExecuteProcess(
            cmd=cmd, output='screen', sigterm_timeout='20', sigkill_timeout='10',
            on_exit=stop_on_exit('Rosbag recorder')))

    if record_video:
        actions.append(Node(
            package='hamr_bringup', executable='record_webcam.py',
            name='hamr_video_recorder', output='screen',
            parameters=[{
                'output_dir': str(run_dir), 'image_topic': '/hamr_camera/image_raw',
                'start_on_motion': start_on_motion, 'motion_threshold': threshold,
                'video_fps': fps, 'codec': 'mp4v', 'use_sim_time': use_sim_time,
            }], sigterm_timeout='20', sigkill_timeout='10',
            on_exit=stop_on_exit('Video recorder')))
        actions.append(Node(
            package='v4l2_camera', executable='v4l2_camera_node',
            name='webcam', namespace='hamr_camera', output='screen',
            parameters=[{
                'video_device': value('camera_device'), 'image_size': [width, height],
                'pixel_format': 'YUYV', 'output_encoding': 'bgr8',
                'camera_frame_id': 'hamr_camera_optical_frame', 'use_sim_time': False,
            }], on_exit=stop_on_exit('USB camera')))
    return actions


def generate_launch_description():
    defaults = {
        'record_video': ('true', 'Enable USB webcam video recording.'),
        'record_bag': ('true', 'Record localization, commands, TF and frame timestamps.'),
        'recording_root': (os.environ.get('HAMR_RECORDING_ROOT', str(Path.cwd() / 'recordings')),
                           'Parent directory for unique run folders.'),
        'run_name': ('', 'Optional unique folder name; default is a UTC timestamp.'),
        'camera_device': ('/dev/video0', 'V4L2 capture device or /dev/v4l/by-id/... path.'),
        'camera_width': ('640', 'Requested YUYV capture width.'),
        'camera_height': ('480', 'Requested YUYV capture height.'),
        'video_fps': ('30.0', 'MP4 playback frame rate; CSV retains actual frame times.'),
        'start_on_motion': ('true', 'Start clip on first nonzero wheel/turret command.'),
        'motion_threshold': ('0.01', 'Absolute command threshold in rad/s.'),
        'record_images': ('false', 'Also record raw images in bag (large files).'),
        'bag_storage': ('mcap', 'Rosbag storage plugin: mcap or sqlite3.'),
        'use_sim_time': ('false', 'Use simulation clock for bag and receive_ros_ns only.'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, (default, description) in defaults.items()],
        OpaqueFunction(function=_launch_recorders),
    ])
