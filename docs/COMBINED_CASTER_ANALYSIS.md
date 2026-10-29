# Focused caster and robot analysis

Run from the `hamr_control` checkout:

```bash
bash hamr_bringup/scripts/visualize_caster_bag /path/to/encoder_YOUR_RUN
```

The input can be the recording directory, its `bags/combined` directory, an
individual `.mcap` or `.db3`, or a normal rosbag directory. For example:

```bash
bash hamr_bringup/scripts/visualize_caster_bag \
  /home/para/encoder_20261029T065650Z_7f3140ba/bags/combined/combined_0.mcap
```

The default output is in the checkout, named after the input recording:

```text
hamr_control/analysis/encoder_20261029T065650Z_7f3140ba/
  index.html
  simulation.mp4
  caster_encoder_angles.png
  robot_trajectory.png
  robot_casters_trajectory.png
  robot_ekf_aligned.png
  report.json
```

`files` means these files directly inside the named recording directory; there
is no additional `files/` subdirectory. Open `index.html` in a browser. It has
exactly five sections in the order above, with short descriptions. The command
no longer generates the full telemetry gallery, CSV exports, fork velocity
estimates, individual video frames or combined PDF. `report.json` records the
source hashes, options, availability and alignment measurements. Outputs are
ignored by Git.

The name is the enclosing `encoder_*` recording folder for the standard
`bags/combined` layout, even when passing a storage shard. For other bags it is
the input bag directory's name. Split bags read all shards listed by their
metadata. Bare MCAP files are also supported. Inputs are not modified, and
nonempty output directories are refused to avoid mixing runs. To make a second
report, use `--output /path/to/new/report`.

## The five sections

1. **Simulation video:** the existing native Simulation Studio HAMR model, with
   both casters animated from recorded joint measurements. The chassis stays
   fixed and the trajectory inset follows the recorded robot. This is kinematic
   replay, not a contact-physics simulation. The default is a 1600-pixel-wide,
   30 FPS MP4 at recorded speed.
2. **Caster encoder angles:** H1, H2 and CENTER for the left and right caster,
   plotted in radians against bag time. Source signs and phases are retained;
   CENTER wraps at 2π. Gaps and wraps break lines.
3. **Robot trajectory only:** the recorded Vicon base XY path, without turret,
   height, waypoint, reference or wheel-odometry overlays.
4. **Robot and casters:** the same base path plus the recorded left and right
   fork-marker positions, all in the same recorded frame.
5. **Robot and aligned EKF:** the base path and `/local_HAMR/odom` (falling back
   to `/odometry/filtered`) after one constant planar transform.

A missing stream is explained in its section; the other sections are still
produced. Joint-state-only bags support angle plots, but CAD replay requires
raw packets to preserve device identity and validity. A requested video failure
returns nonzero and retains `simulation.log` beside the partial report.

## EKF alignment

The default `--ekf-alignment initial-pose` pairs the first valid overlapping
robot/EKF poses and aligns both position and heading. Subsequent discrepancies
remain visible. Samples are paired at 50 Hz using bag receipt timestamps;
interpolation excludes gaps over 0.25 seconds. It does not estimate sensor
latency or calibrate the Vicon marker-to-body transform.

For a whole-run visual fit:

```bash
bash hamr_bringup/scripts/visualize_caster_bag /path/to/encoder_YOUR_RUN \
  --ekf-alignment rigid-fit --output /path/to/another/report
```

`rigid-fit` finds a proper 2D rotation and translation over the common valid
time interval. It fits no scale and applies no clock shift. Because the same
run is used for fitting and comparison, its residual is descriptive, not an
independent localization accuracy measurement. The HTML reports the method,
rotation, comparison interval, RMS difference and final difference. Full
transform values are in `report.json`.

## Rendering options and dependencies

`--render none` makes only the four plots. `--preview-time 5` replaces the video
with one preview at five seconds after encoder start. `--render auto` chooses
video when FFmpeg is available, otherwise a preview. Video is the default.

`--rate 1 --fps 30` preserves recorded playback speed. Saved CENTER references
are wheel/device specific; use `--center-reference none` for different devices,
or `--left-mapping-config` and `--right-mapping-config` for explicit mappings.
`--wheel front_left`, `front_right` or `legacy` selects standalone caster replay.
The angle plots still show whichever left/right channels were recorded.

Both front CENTER references use the confirmed upright-fork pose: H1 toward
the mounting block (+Z), with increasing raw counts moving H1 toward front (+X).
The left reference is **7792 counts**, from `center_left_turn_04`; the right is
**16183 counts**. After reviewing the rendered playback, the user corrected both
CAD directions to `center_sign: +1`. This overrides the original sign inference
while preserving the measured reference counts and H1-up poses. Single-wheel
and paired replay select the same references and corrected directions.
Each recording starts at its measured absolute phase, without resetting the
first sample to zero or applying firmware alpha offsets again. The left
full-turn return difference is 0.268 degrees; the right is -0.153 degrees.
These checks establish return repeatability, conditional on the supplied speed
bound and physical mark. H1/H2 shell phases, angular accuracy and robot/world
orientation remain separate. Reuse a reference on older recordings only if
the encoder mounting, coupling and electronic raw zero were unchanged.

`--recover-timing` explicitly permits approximate recovery from saved clock
probes. Acquisition timestamps remain estimated, not independently validated.

The Bash entry point sources an installed ROS Jazzy/Humble environment. For
Ubuntu 24.04/Jazzy, dependencies are:

```bash
sudo apt install python3-numpy python3-scipy python3-matplotlib python3-yaml \
  python3-pil python3-vtk9 fonts-dejavu-core ffmpeg xvfb \
  ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-rclpy ros-jazzy-std-msgs ros-jazzy-xacro
```

An available X display or `xvfb-run` is used automatically. Plot-only operation
needs neither VTK nor FFmpeg. After rebuilding and sourcing a colcon workspace,
`ros2 run hamr_bringup visualize_caster_bag /path/to/encoder_YOUR_RUN` provides
the same command. Run it from the checkout, or pass `--output` if the installed
package cannot locate the source checkout.

The earlier full reports in `docs/analysis/` are historical artifacts; their
original files are retained. The focused command reads recordings rather than
archived CSV analysis directories.
