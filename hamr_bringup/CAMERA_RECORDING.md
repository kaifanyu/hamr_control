# Dual-camera recording with the hardware launch

`hamr_HW.launch.xml` records the Logitech C920 and Brio 101 by default. It starts
one dual-camera recorder when the hardware stack starts and records until that
launch is stopped. The separate `waypoint_traj_simple` command does not start or
stop the cameras. The same recording can therefore contain the initial stationary
period and one or more trajectory runs.

## Build and run

```bash
cd ~/hamster_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hamr_bringup
source install/setup.bash
ros2 launch hamr_bringup hamr_HW.launch.xml
```

Wait for **`Recording NOW`**. This appears only after both cameras have opened,
finished warmup, and passed the manual-control and capture-format checks. Then,
in a second terminal with the ROS workspace sourced, run your usual trajectory:

```bash
source /opt/ros/jazzy/setup.bash
source ~/hamster_ws/install/setup.bash
ros2 run reference_trajectory waypoint_traj_simple
```

The trajectory's existing defaults and controller behavior are unchanged. The
plain trajectory command starts immediately and loops; camera readiness is an
operator check, not a motion interlock. Stop the trajectory as usual, then press
Ctrl+C in the hardware-launch terminal. Wait for **`Saved complete dual-camera
session`** before opening or moving the recordings. Graceful SIGINT and SIGTERM
finalize the AVI indexes, timestamp files and session manifest. Power loss or
SIGKILL cannot guarantee finalization.

To launch the stack without recording cameras:

```bash
ros2 launch hamr_bringup hamr_HW.launch.xml record_camera:=false
```

## Saved files

The recorder prints its exact new directory under:

```text
~/hamster_ws/recordings/dual_cam_YYYYMMDD_HHMMSS_microseconds_unique/
    c920.avi
    brio101.avi
    c920_timestamps.csv
    brio101_timestamps.csv
    session.json
```

The AVI files retain the cameras' original MJPEG images, without resizing,
rotation or JPEG re-encoding. Each timestamp CSV has a global frame index and
host monotonic receive times. `session.json` records the actual controls,
configuration snapshot, calibration hashes, capture statistics and time origin.
These are `motion` sessions accepted by the dual-camera analysis pipeline once
its stereo and axis calibration are available.

Long sessions rotate each camera's video into approximately 1 GiB parts, named
`c920_0001.avi`, `c920_0002.avi`, and similarly for Brio. The cameras stay open and
frame indices/timestamps continue across the parts. The manifest lists each
part's first frame and frame count; the dual-camera reader follows those parts
in order. Keep **all parts, both CSVs, and `session.json` together**. Individual
AVI playback uses nominal FPS; use the CSVs for timing.

Robot MCAP bags remain in `HAMR_BAG_ROOT`, normally
`~/hamster_ws/src/hamr_control/rosbags`. Camera recordings are separate files and
do not add image topics to the bag. Their timestamped directory names are not a
promise of an identical bag name. The recorded host clock information supports
approximate association with local bag receipt times; it does not measure camera
exposure latency or synchronize the two webcams. Measure relative camera timing
before using moving stereo observations.

## One source of camera settings

The installed ROS wrapper imports capture code from:

```text
~/Caster_Vision/ball_caster_dual_cam
```

It reads **that repository's `config/rig.yaml` on every launch**. Change camera
device paths, exposure, gain, fixed focus, zoom, resolution, FPS and white balance
there, using the same file for calibration and trajectory recordings. Intrinsics
remain in `calibration/c920.yaml` and `calibration/brio101.yaml`, referenced by the
rig. Paths inside the rig are relative to the rig YAML's directory.

Both cameras use the rig's manual exposure and white balance settings. C920
autofocus is disabled and its configured focus/zoom are applied after streaming
starts. Brio 101 has fixed focus and receives no focus-control writes. The
recorder rejects unsupported settings or readback mismatches. Unknown historical
intrinsics capture profiles remain unverified; successful recording does not
certify geometric calibration.

The old hardware-launch arguments `camera_device`, `camera_width`,
`camera_height`, `camera_fps`, `camera_focus`, `camera_exposure`, `camera_gain`,
and `camera_zoom` are replaced by the shared rig configuration. The legacy
`record_camera_clip` executable remains available separately for C920-only MP4
capture; do not run it while the dual recorder owns that camera.

Supported launch overrides:

```bash
ros2 launch hamr_bringup hamr_HW.launch.xml \
  camera_repository:="$HOME/Caster_Vision/ball_caster_dual_cam" \
  camera_config:="$HOME/Caster_Vision/ball_caster_dual_cam/config/rig.yaml" \
  recording_root:="$HOME/hamster_ws/recordings" \
  camera_segment_mib:=1024
```

Keep the dual-camera checkout available: this integration deliberately reuses
its capture implementation instead of maintaining a second copy. Runtime capture
dependencies are Python OpenCV, NumPy, PyYAML, and `v4l2-ctl` (`v4l-utils`).

## Camera-only checks

A camera-only continuous recording, without launching robot control:

```bash
ros2 launch hamr_bringup camera_recording.launch.xml
```

A bounded three-second recording:

```bash
ros2 run hamr_bringup record_dual_cameras --duration 3 \
  --output-dir "$HOME/hamster_ws/camera_diagnostics"
```

A camera failure stops both camera recordings, saves whatever can be finalized,
and prints an error. It does **not** shut down the controller or other hardware
nodes. Check the camera process output and `session.json`; a stopped camera
process means subsequent trajectory motion is not being filmed. Close any other
program using the cameras before launch.

## Integration verification, 2026-09-16

The package was rebuilt with `colcon build --symlink-install --packages-select
hamr_bringup`. Both launch files parse through the actual Jazzy launch frontend,
including configuration/output paths containing spaces.

- Dual-camera pipeline suite: **86 passed**.
- New/updated ROS recorder and launch tests: **14 passed**.
- Broader ROS checks: **43 passed, 1 failed**. The failure is the existing Vicon
  test referencing the absent `hamr_HW_waypoint_vicon_unprotected.launch.xml`;
  this integration did not restore or change that unrelated file.
- A camera-only ROS launch recorded approximately three seconds, then received
  SIGINT. Both cameras finalized with `status: complete` and no capture errors.
- C920 recorded 89 frames at approximately 28.76 observed FPS; Brio recorded 92
  at approximately 29.93 FPS. Every saved frame decoded successfully.
- The check used 16 MiB parts to exercise rotation on both cameras: C920 had
  52 + 37 frames, Brio 63 + 29. The normal launch uses 1024 MiB parts.
- Initial, warmup and final manual-control readbacks all matched the requested
  profiles. The segmented session reader accepted the resulting recording.

Evidence: [live launch and validation files](/home/hamr/hamster_ws/camera_diagnostics/dual_cam_ros_check_20260916_230912).
The controller and trajectory were not launched during verification. This short
check establishes the recording lifecycle, not long-duration USB reliability,
geometric calibration or exposure synchronization. The current unverified
zero-offset pairing accepted 42 of the 89/92 source frames; both full streams
were saved for later timing correction.
