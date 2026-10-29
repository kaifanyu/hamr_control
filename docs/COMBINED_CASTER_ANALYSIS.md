# One command for caster replay and combined rosbag analysis

Run this from the repository root:

```bash
bash hamr_bringup/scripts/visualize_caster_bag \
  /path/to/combined_0.mcap \
  --output "$HOME/Videos/hamr_sim/my_run"
```

Open `my_run/index.html`. That page contains the paired caster replay, every
plot, one `all_plots.pdf`, CSV downloads, and the status of each processing step.
No ROS playback, motor controller, live encoders, or Gazebo process is started.
The CAD animation replays measured joints; it is not a contact-physics simulation.

The input can be a downloaded `encoder_<run>` folder, its `bags/combined`
directory, an individual `.mcap` or `.db3`, or `packets.jsonl`. Both casters are
selected by default. A file belonging to a split bag selects every shard listed
in `metadata.yaml`; a standalone MCAP needs neither YAML nor separate sidecars.
Use a new or empty output folder. Originals are never modified. Omitting
`--output` creates a timestamped folder under `~/Videos/hamr_sim`.

After building and sourcing the workspace, the same command is installed as:

```bash
ros2 run hamr_bringup visualize_caster_bag /path/to/combined_0.mcap \
  --output "$HOME/Videos/hamr_sim/my_run"
```

Useful options:

```bash
# All graphs/CSV/PDF, without a CAD rendering dependency.
bash hamr_bringup/scripts/visualize_caster_bag /path/to/combined_0.mcap \
  --output "$HOME/hamr_analysis/my_run" --render none

# One CAD frame plus all graphs; seconds are relative to encoder start.
bash hamr_bringup/scripts/visualize_caster_bag /path/to/encoder_run \
  --output "$HOME/hamr_analysis/my_preview" --preview-time 5
```

The default `--render auto` creates an MP4 when FFmpeg is available, otherwise a
PNG preview. `--render video` explicitly requires an MP4; a rendering failure
returns nonzero while preserving the completed analysis. `--render preview`
selects a midpoint preview. Rendering failures or absent data are displayed in
`index.html` and `report.json`; they are never reported as a successful replay.
`--no-plots` writes CSV/JSON only. `--wheel front_left`, `front_right`, or `legacy`
selects one wheel for packet diagnostics and standalone CAD rendering.

## What the report includes

- Base, turret and left/right fork XY paths and height; recorded reference paths.
- All six caster angle and reported-rate histories, plus raw packet CSVs,
  validity flags, sequence gaps, duplicate/reorder counts, firmware counters,
  index consistency and retained device metadata.
- Vicon roll/pitch/yaw, shortest quaternion rotation steps, gaps, timestamp
  differences, and a coverage inventory of every topic in the bag.
- Every supported numeric telemetry channel: drive commands and encoder ticks,
  wheel target/measured RPM, PWM/PI terms and saturation, controller gains and
  errors, odometry pose/twist, reference position/velocity, IMU acceleration and
  gyro, magnetic field, and IMU calibration status.
- Uncalibrated fork-marker trajectories, linear/angular velocity estimates,
  and each wheel's H1/H2/CENTER rate estimates over the same fitting window.
- Paired native-size Simulation Studio CAD, separate caster closeups, six angle
  plots and a progressive same-run base-trajectory inset.

`analysis/topics.csv` inventories every original topic, including zero-count
and unplotted types. Images, point clouds, TF and other unsupported structured
payloads remain in the original MCAP; the tool does not pretend to graph them.
`analysis/topics/*.csv` contains decoded numeric channels with exact integer
bag/header timestamps. `analysis/encoder_packets/<wheel>/` holds raw sample
fields and quality reports. `comparison/*_aligned.csv` retains raw values,
estimates, fitting windows, sample ages, segment IDs and validity flags.

Partial recordings still produce all available plots. A missing base/fork
stream disables only the fork comparison. Joint-state-only bags produce angle
plots, but CAD replay requires raw packets to preserve device identity, encoder
validity, and saved raw CENTER references. Missing measurements are not fabricated.

## Geometry, timing and calibration

Paired replay uses `compa_ball_caster_recording.urdf.xacro`, with unchanged native
mounts and dimensions. Each wheel has its own saved CENTER reference and mapping.
The paired defaults use `center_sign: +1` for both wheels. Rotation is reversed
about the existing anchors: left raw count 1210 at model angle 3.620181390266952 rad,
right raw count 16183 at 5.190977717061848 rad. Each run's initial orientation
comes from its first valid raw `position14` relative to that wheel's anchor;
there is no per-run zero reset or second application of firmware alpha offsets.
Use `--center-reference none` for other devices without those references, or
`--left-mapping-config` / `--right-mapping-config` for independently verified
mappings. An identity mismatch stops rendering instead of applying another
wheel's calibration. Raw graphs never apply the CAD sign/phase offsets.

Onboard packets retain their embedded estimated acquisition clock maps. Live
UDP packets retain bag receipt timing; samples in a batch share its receipt
time. Only local recordings with original probes may use `--recover-timing` for
explicitly approximate visualization. Another run's Vicon history is never
shifted onto this run. Sources and processing settings are recorded in JSON.

Fork estimates use centered 100 ms local linear fits, at least three samples
spanning 20 ms, and the thresholds in `caster_vicon_comparison.yaml`. Fits stop
at invalid samples, tracking jumps/guards and timestamp gaps. Encoder fits also
stop at continuity-segment changes. Position jumps over 80 mm are rejected with
a guard interval. These are offline estimates and can use later samples inside
a continuous segment. Smooth Vicon orientation may still be ambiguous.

Vicon observes marker bodies on the forks. CENTER observes internal carrier
rotation; H1/H2 observe shell spin. The plots compare histories, not equal
physical angles. No marker-to-ball-center transform, latency fit, rolling
model, or physical encoder error is inferred. Those require independent physical
calibration. The original independent-reference CSV/sign/zero/clock options are
available on this same command; see `--help` and [encoder calibration](CASTER_ENCODERS.md#7-add-an-independent-angle-reference-later).

## Dependencies

The entry point sources an installed ROS Jazzy/Humble environment automatically.
For Ubuntu 24.04 / Jazzy:

```bash
sudo apt install python3-numpy python3-scipy python3-matplotlib python3-yaml \
  python3-pil python3-vtk9 fonts-dejavu-core ffmpeg xvfb \
  ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-rclpy ros-jazzy-std-msgs ros-jazzy-xacro
```

An X11 VTK build needs a display; an available local X display or `xvfb-run` is
used automatically. Analysis with `--render none` does not require VTK or FFmpeg.

## Historical evidence

The [October 29 source archive](analysis/encoder_20261029T065650Z_7f3140ba/README.md),
[original preparation](analysis/caster_vicon_comparison_20261029T065650Z/README.md),
and [fork trajectory snapshot](analysis/caster_fork_history_20261029/comparison_setup.json)
remain unchanged. Their old command lines are historical provenance. To recreate
the fork comparison from the archived CSVs without ROS, use the same entry point:

```bash
bash hamr_bringup/scripts/visualize_caster_bag \
  docs/analysis/encoder_20261029T065650Z_7f3140ba \
  --output "$HOME/hamr_analysis/archive_comparison" --render none
```

The former separate render/analyze/preparation command-line scripts have been
removed. Their reusable parsing, calibration, quality and rendering code is
maintained as internal modules behind `visualize_caster_bag`.

[Validation results](analysis/unified_caster_visualization_20261029.json) cover
the package build, 217 focused regressions, a complete synthetic combined-MCAP
MP4/report, and real partial encoder/Vicon recordings. The generated full
fixture verifies software behavior; it is not physical calibration evidence.
