# Recording hardware experiments

Run these commands on the **Ubuntu 24.04 / ROS 2 Jazzy computer connected to the
Logitech USB webcam**. The Windows checkout is source code; the camera driver
uses Linux V4L2. If control runs on a different machine, see the separate-camera
host instructions below.

## Install and build

Copy/pull this updated checkout into your ROS workspace's `src` directory. The
examples assume the existing workspace is `~/hamr_ws`; substitute your own path.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install ros-jazzy-v4l2-camera ros-jazzy-cv-bridge \
  ros-jazzy-rosbag2-storage-mcap python3-opencv python3-numpy v4l-utils
cd ~/hamr_ws
colcon build --symlink-install --packages-up-to hamr_bringup reference_trajectory
source install/setup.bash
```

This assumes the robot's existing ROS/Gazebo/Foxglove/serial dependencies are
already installed. For a fresh workspace, install its declared dependencies
first with `rosdep install --from-paths src --ignore-src -r -y`.

Identify the webcam (some Logitech models expose multiple device nodes; choose
the node offering video capture with YUYV):

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

Defaults are `/dev/video0`, YUYV, 640x480. A stable
`/dev/v4l/by-id/usb-Logitech_...-video-index0` path is preferable when available.
Use a mode actually listed by your device. Close other applications using it.
If access is denied, add your user to `video` (`sudo usermod -aG video "$USER"`)
and log out/in. `video_fps` controls MP4 playback; it does not configure the
driver's capture rate. Check the actual rate with `ros2 topic hz
/hamr_camera/image_raw`, and adjust the playback rate if desired. Timestamp
alignment always uses the CSV, even when capture intervals vary.

## Run the physical robot

Start your existing Vicon publishing setup first. This repository consumes but
does not launch Vicon. Confirm that both localization topics are arriving:

```bash
ros2 topic hz /HAMR_base/odom
# Ctrl+C ends this diagnostic only.
ros2 topic hz /HAMR_turret/odom
```

Terminal A:

```bash
source /opt/ros/jazzy/setup.bash
source ~/hamr_ws/install/setup.bash
ros2 launch hamr_bringup hamr_HW.launch.xml \
  camera_device:=/dev/video0 \
  recording_root:="$HOME/hamr_ws/recordings"
```

This starts the hardware controller, serial relay, Foxglove, webcam and bag.
The relay already runs in this launch; do not start a second copy. Both
`record_video` and `record_bag` default to `true`. The launch prints the unique
run directory. Rosbag records immediately, including localization before motion.
The webcam opens immediately to warm up, while the video recorder waits for
motion and retains up to 0.5 seconds of recent frames in memory.

Wait for the video recorder's **armed** log and the bag's subscription messages.
You can also inspect the recorder status in another terminal:

```bash
ros2 topic echo /hamr_camera/recording_status --once --qos-durability transient_local
```

Terminal B (start the current reference trajectory):

```bash
source /opt/ros/jazzy/setup.bash
source ~/hamr_ws/install/setup.bash
ros2 run reference_trajectory waypoint_traj_simple
```

The current trajectory is relative to the initial Vicon pose, reaches 5 m in
positive odometry Y and +/-1.5 m in X, and defaults to 0.05 m/s. Review the
waypoints in `reference_trajectory/reference_trajectory/waypoint_traj_simple.py`
for your test area before starting it.

The first finite wheel/turret command whose magnitude exceeds 0.01 rad/s starts
the clip, including the short preceding buffer. This is a **command-based start**,
not a measurement that the chassis physically moved. A stationary robot with
nonzero commands also triggers recording. Zero commands and pauses after that
do not split or stop the clip. If no qualifying command occurs, there is no MP4.
To record from camera startup, pass `start_on_motion:=false`.

To finish, **press Ctrl+C once in Terminal A and wait for shutdown to finish**;
the writer releases the MP4 and flushes the CSV/metadata, and rosbag finalizes
its index. Stop Terminal B too. An orderly IDE stop sending SIGINT/SIGTERM is
also handled. Force-kill, power loss, or a terminal/SSH session disappearing
cannot guarantee a finalized MP4 or rosbag; use Ctrl+C before closing it.
Stopping only the trajectory publisher is not a robot stop: the current
controller retains the last reference. The existing relay/controller also do
not guarantee a motor-zero command on process exit, so use your normal robot
stop procedure before ending the launch.

Camera/recorder/bag failure, or hardware controller/relay exit, ends the launch
so it does not silently continue an unrecorded experiment. Recording is not a
motor safety interlock: do not start the trajectory until the camera is armed.

## Saved files and synchronization

```text
recordings/hamr_YYYYMMDD_HHMMSS_microsecondsZ/
  run.json                 launch configuration and file locations
  video.mp4                one continuous clip after motion trigger
  frame_timestamps.csv      one row per saved video frame
  video_metadata.json      trigger, frame counts and final recording status
  bag/
    metadata.yaml
    bag_0.mcap
```

Directories are unique and are never reused/overwritten. Set `run_name:=trial_01`
for your own name, choosing a new name each time. Without `recording_root`, the
default is `recordings/` under the directory where you invoked the launch;
`HAMR_RECORDING_ROOT` can set that default.

The bag records these topics when publishers exist, including ones appearing
after launch:

| Data | Topics |
| --- | --- |
| Vicon localization | `/HAMR_base/odom`, `/HAMR_base/pose`, `/HAMR_turret/odom`, `/HAMR_turret/pose`, both individual marker arrays |
| Robot localization | `/odom`, `/robot_pose`, `/odometry/filtered`, `/amcl_pose`, `/hamr/odom` |
| Reference and controller | `/reference_trajectory`, `/waypoints_path`, `/state_error`, `/live_gains`, `/cmd_vel` |
| Motors | `/{left_wheel,right_wheel,turret}/cmd_vel` and `/encoder_ticks` |
| Frames and clock | `/tf`, `/tf_static`, `/clock` |
| Video synchronization | `/hamr_camera/frame_stamp`, `/hamr_camera/recording_status`, `/hamr_camera/camera_info` |

The bag does not produce localization; a topic without a publisher has no samples.
It excludes the high-bandwidth image stream by default. Pass `record_images:=true`
to also retain `/hamr_camera/image_raw` in the bag (keep `record_video:=true`
to launch the webcam, or supply that image topic externally). Raw 640x480 BGR at 30 fps is
about 28 MB/s before storage overhead; ensure adequate disk throughput/capacity.
The MP4 remains a separate artifact either way.

For each CSV row:

| Column | Meaning |
| --- | --- |
| `frame_index` | Zero-based index of that frame in `video.mp4` |
| `video_time_sec` | MP4 playback time (`frame_index / video_fps`) |
| `image_stamp_ns` | Original camera `Image.header.stamp`, in integer nanoseconds |
| `receive_ros_ns` | Recorder's ROS clock when the image callback ran |
| `receive_unix_ns` | Recorder host's Unix clock at receipt |
| `receive_monotonic_ns` | Recorder host's monotonic clock, useful for detecting wall-clock adjustments |

For hardware, find/interpolate localization samples around `image_stamp_ns`,
using `odom.header.stamp.sec * 1_000_000_000 + odom.header.stamp.nanosec`.
Use the appropriate base or turret pose and its coordinate frame. Position can
be linearly interpolated; interpolate orientations with quaternion SLERP.
Keep nanoseconds as integers to avoid losing precision. Headerless reference
and error messages must use their bag receive times.

`/hamr_camera/frame_stamp` repeats each saved frame's camera timestamp and
encodes its video index in `frame_id`; the CSV is the authoritative complete
mapping even if ROS discovery or shutdown misses a marker. Do not simply
add MP4 playback time to bag start time: camera rate variation, preroll and
dropped frames can otherwise cause drift.

Camera and localization timestamp producers need a common clock. Prefer the
same host where possible; otherwise synchronize the Ubuntu/Vicon computers
using your lab's NTP/chrony or PTP setup and verify clock offsets before a run.
The Jazzy driver stamps each image immediately after capture returns, using
the camera host's clock; this is host reception time, with USB/exposure
latency, rather than an exposure timestamp. It does **not** provide a shared
hardware exposure trigger between a consumer Logitech camera and Vicon. For tighter
alignment, record a visible synchronization event (such as a tracked object's
sharp movement) and estimate any residual offset. Relay `/odom` and
`/robot_pose` currently use ROS host reception time rather than the firmware's
acquisition time. Vicon header timing depends on your external driver.

Inspect the result after stopping:

```bash
ros2 bag info "$HOME/hamr_ws/recordings/<run-folder>/bag"
```

Open the MP4 and confirm the CSV row count matches `video_metadata.json`.
Before a real experiment, test the camera without running any robot controller:

```bash
ros2 launch hamr_bringup recording.launch.py start_on_motion:=false \
  recording_root:="$HOME/hamr_ws/recordings"
# Record for a few seconds, then Ctrl+C and inspect the saved files.
```

## Other launch options

Disable the webcam while retaining localization recording:

```bash
ros2 launch hamr_bringup hamr_HW.launch.xml record_video:=false
```

Other options include `record_bag:=false`, `bag_storage:=sqlite3`,
`camera_width:=640 camera_height:=480`, `video_fps:=30.0`, and
`motion_threshold:=0.01`. Use `--show-args` to list launch arguments.

The open **`bringup.launch.py` is the Gazebo launch**. To run it with a real USB
webcam and a bag of simulated localization:

```bash
ros2 launch hamr_bringup bringup.launch.py record_video:=true record_bag:=true \
  recording_root:="$HOME/hamr_ws/recordings"
# In another sourced terminal:
ros2 run reference_trajectory waypoint_traj_simple --ros-args \
  -p odom_topic:=/hamr/odom -p use_sim_time:=true
```

Recording defaults to off for this simulation entrypoint. The camera still has
real system timestamps; simulated odometry has `/clock` timestamps. Use CSV
`receive_ros_ns` to align webcam receipt with simulated localization, rather
than comparing epoch `image_stamp_ns` directly to simulation time. Simulation
pauses/restarts can repeat/reset that clock; split experiments at a reset.

If the webcam is attached to a **separate Ubuntu computer**, run hardware launch
with `record_video:=false record_bag:=false` on the robot and run
`recording.launch.py` on the camera computer (both packages must be built and
sourced there). Match ROS domain/network settings so Vicon and command topics
are discoverable; synchronize clocks across hosts. Recording shutdown is then
controlled by the separate recording launch and is **not** automatically tied
to the remote hardware launch. A webcam connected only to native Windows is
not accessible as `/dev/video0` on the remote robot.

The former `ros2 run hamr_bringup record_hamr_vicon_bag` bag-only helper remains
available with its existing `HAMR_BAG_ROOT`, `HAMR_BAG_NAME` and
`HAMR_BAG_STORAGE_ID` environment variables. Hardware launch now uses the shared
recording launch instead, and includes the expanded localization/topic filter.

Implementation references: [ROS Jazzy V4L2 camera driver](https://docs.ros.org/en/jazzy/p/v4l2_camera/),
[driver timestamp assignment](https://gitlab.com/boldhearts/ros2_v4l2_camera/-/blob/jazzy/src/v4l2_camera.cpp),
[Jazzy rosbag recording options](https://github.com/ros2/rosbag2/blob/jazzy/ros2bag/ros2bag/verb/record.py),
and [OpenCV VideoWriter](https://docs.opencv.org/4.x/dd/d9e/classcv_1_1VideoWriter.html).
