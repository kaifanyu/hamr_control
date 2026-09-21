# Record a complete vehicle run to MP4

Commands in this guide assume a checkout of this repository. From any directory
inside that checkout, set these portable paths before running the examples:

```bash
export HAMR_REPO="$(git rev-parse --show-toplevel)"
export HAMR_RUNS="${HAMR_RUNS:-$HOME/Videos/hamr_sim}"
```

Recorded videos and full run reports are external artifacts, not files supplied
by a Git clone. Paths under `${HAMR_RUNS}` identify retained runs or new outputs
created by the recording commands.

The recording command starts Gazebo, spawns the complete vehicle with both CAD
ball casters, waits for the camera to record, follows the original simple waypoint
positions with a smooth stop at each waypoint, finalizes an MP4 and stops its own
processes. The original abrupt schedule remains available as `--profile legacy`.

## Run it on this WSL system

```bash
cd "${HAMR_REPO}"
source hamr_ball_caster/scripts/env.sh
ros2 run hamr_ball_caster record_waypoint_run.py
```

Each run creates a new timestamped directory under `~/Videos/hamr_sim/`.
The default replay file is **`hamr_smooth_waypoint.mp4`**: H.264, 1280 × 720,
25 fps, yuv420p, with the MP4 index prepared for seeking. It contains the Gazebo
camera view, not a desktop capture. Open it in a standard video player.

The final smooth recording is available at
`smooth_waypoint_20260920_final/hamr_smooth_waypoint.mp4` (`${HAMR_RUNS}/smooth_waypoint_20260920_final/hamr_smooth_waypoint.mp4`).
It is **105.72 seconds** long, including **98.731 seconds of motion and endpoint
settling**. Measured RMS tracking error was **0.265 mm**, peak tracking error
**1.064 mm**, and full-leg geometric deviation and corner overshoot **0.834 mm**.
It passed the strict trajectory checks, video timing coverage and full-video
decoding. See [SMOOTH_WAYPOINT.md](SMOOTH_WAYPOINT.md) for the baseline comparison
and the timing tradeoff.

This recording contains 2,643 encoded frames: 1,723 received camera images and
920 repeated frames preserving simulation timing under software-rendering
delays. No frames were dropped from the encoding queue; the largest image gap
was 0.44 simulation seconds. Capture and finalization took about 369 wall
seconds. The repeated frames affect visual smoothness, not the measured path or
the duration of the replay.

The earlier **legacy-profile** recording is saved at
`${HAMR_RUNS}/simple_waypoint_20260920_185228/hamr_simple_waypoint.mp4`.
It is 63.56 seconds long and passed full-video decoding and trajectory checks.
Recorded motion RMS position error was 12.7 mm, with a 64.0 mm peak during
direction changes; every waypoint was approached within 7.1 mm of its scheduled
arrival window. Full-run evidence is in
[full_waypoint_recording.json](validation/full_waypoint_recording.json).

The recorded stream contains 1,589 frames: 1,241 received camera images and 348
repeated frames to preserve simulation timing under software-rendering delays.
No frames were lost from the recorder's encoding queue. The largest camera-image
gap was 0.44 simulation seconds. The replay therefore preserves route duration
but can show brief repeated frames; these are renderer limits, not changes to
the physical trajectory. Capture and finalization took about 225 wall seconds.

The default smooth profile uses a configured speed limit of 0.25 m/s. It starts
and ends each segment at rest, so its duration is longer than the legacy
52-second motion. Its report records both the planned schedule and actual
schedule, including any endpoint settling. See [SMOOTH_WAYPOINT.md](SMOOTH_WAYPOINT.md)
for the planner and controller behavior. Startup and final holds, plus camera
startup, add a short lead-in and tail. Wall time is longer because the simulator
targets half real time to leave capacity for this machine's software renderer.
Video timestamps come from simulation time, so the replay preserves the actual
simulated motion timing, including stops.

To see the live Gazebo window too:

```bash
ros2 run hamr_ball_caster record_waypoint_run.py --gui
```

To replay the original abrupt reference as a comparison:

```bash
ros2 run hamr_ball_caster record_waypoint_run.py --profile legacy
```

That selects `config/simple_waypoint_sim.yaml`, saves `hamr_simple_waypoint.mp4`,
and retains the original 0.25 m/s, 52-second reference motion. The default smooth
profile selects `config/smooth_waypoint_sim.yaml`. Use `--config /path/to/config.yaml`
to select another configuration and `--speed 0.20` to override only the speed
limit. Without `--speed`, the selected configuration determines the speed.
Before launching Gazebo, the command saves the selected configuration's exact
bytes as `configuration.yaml` in the output directory and runs from that copy.
`commands.json` and `run_summary.json` record its SHA-256, source path and any
speed override. The report records the effective reference speed.

To choose the output location, use a new or empty directory:

```bash
ros2 run hamr_ball_caster record_waypoint_run.py \
  --output-dir "$HOME/Videos/hamr_sim/my_waypoint_run"
```

The terminal prints the final video path and PASS/FAIL. Failure keeps the video
and logs for diagnosis. Ctrl+C cancels the run, stops its simulation, and attempts
to finalize the partial video. A partial run is not reported as passing.

## Route and control

The reference matches the active points in
`reference_trajectory/reference_trajectory/waypoint_traj_simple.py`:

```text
(0,0) → (0,2) → (−2,2) → (−2,4.5) → (0,4.5) → (0,2) → (0,0)
```

Coordinates are meters in the simulation world. The total reference length is
13 m. Floor labels identify the five unique positions; the visit order is
**1 → 2 → 3 → 4 → 5 → 2 → 1**. The line, labels and grid have no collisions.

The original executable's default is 0.25 m/s; its separate study-catalog YAML
defaults to 0.20 m/s. `--profile legacy` uses the original piecewise linear timing;
at `--speed 0.20` it takes 65 seconds of motion. Tests compare that legacy
reference against the original `WaypointTraj` class, including segment boundaries.
The smooth profile preserves the same line segments and waypoint order while
changing their timing to remove the instantaneous velocity changes. Both
profiles keep the upper platform's world yaw at zero.

The new `run_waypoint_sim.py` is a simulation controller for this vehicle's
offset differential drive. It uses Gazebo odometry and joint states as feedback,
commands the two drive wheels and three gimbal joints, and lets the chassis
rotate while the upper yaw platform counter-rotates. Both five-joint ball
casters remain entirely passive. The vehicle is moved by Gazebo physics;
positions are not teleported or animated onto the reference.

The controller uses the generated URDF's wheel radius and axle offsets. It checks
for the expected complete vehicle before commanding motion, records tracking
errors, and verifies completion, waypoint proximity, floor support and caster
articulation. This uses ideal simulated velocity actuators and the nominal
physical assumptions documented in [CAD_FINDINGS.md](CAD_FINDINGS.md).
The chassis is the separate COMPA/HAMR retrofit from the previous setup.

## Saved evidence

| Output | Contents |
| --- | --- |
| `hamr_smooth_waypoint.mp4` or `hamr_simple_waypoint.mp4` | Complete camera recording for the selected profile |
| `run_summary.json` | Combined trajectory and video verification |
| `trajectory.json` | Selected profile/configuration, planned and actual timing, actual/reference positions, commands, joint angles and errors |
| `video_metadata.json` | Camera timestamps, frame counts, duplicates, queue drops and encoding checks |
| `video_probe.json` | Codec, resolution, duration and frame rate reported by ffprobe |
| `commands.json` | Exact launched commands and isolated ROS/Gazebo environment |
| `configuration.yaml` | Exact controller/planner configuration snapshot used by the runner |
| `gazebo.log`, `recording.log`, `trajectory.log` | Startup, execution and diagnostic logs |

Generate an actual-versus-reference plot and CSV from a saved run:

```bash
/usr/bin/python3 "${HAMR_REPO}/hamr_ball_caster/tools/plot_waypoint_run.py" \
  "$HOME/Videos/hamr_sim/my_waypoint_run/trajectory.json"
```

The combined command uses ROS domain 131 with localhost discovery and a unique
Gazebo partition by default. It launches its own simulator; a separately opened
caster simulator is not needed. Choose another domain with `--domain-id` if 131
is already in use. The launch and recorder clean up their owned processes.

## Record an already-running simulation

For manual experiments, launch the camera world first:

```bash
source "${HAMR_REPO}/hamr_ball_caster/scripts/env.sh"
ros2 launch hamr_ball_caster ball_caster.launch.py model:=compa gui:=true \
  "world:=${HAMR_REPO}/hamr_ball_caster/worlds/hamr_waypoint_recording.sdf" \
  "gui_config:=${HAMR_REPO}/hamr_ball_caster/config/waypoint_gui.config" \
  real_time_factor:=0.5
```

In a second terminal, source the same environment and start recording:

```bash
source "${HAMR_REPO}/hamr_ball_caster/scripts/env.sh"
ros2 run hamr_ball_caster record_simulation.py --output /tmp/my_simulation.mp4
```

Wait for `RECORDING`, then run `run_waypoint_sim.py --profile smooth` or publish your own drive
commands from a third terminal with the same environment. Ctrl+C in the recorder
terminal finalizes its MP4 while leaving the simulation running. Alternatively,
`--duration 10` records ten simulation seconds and exits automatically. Existing
video files are never overwritten.

## Installation and recording implementation

The current machine is prepared. For a normal apt-based installation, install
`ffmpeg` in addition to the dependencies in the main README, then rebuild:

```bash
sudo apt install ffmpeg
cd "${HAMR_REPO}"
bash hamr_ball_caster/scripts/build.sh
```

The rootless Gazebo cache on this WSL system also contains the official Ubuntu
FFmpeg binaries. `scripts/env.sh` exposes them alongside Gazebo. The optional
bootstrap in [rootless_runtime.md](rootless_runtime.md) includes that dependency.

The recording world has a fixed 25 Hz camera and Gazebo Sensors system. The
recorder starts an image-only ROS bridge, queues RGB frames, and encodes them
with FFmpeg. Camera simulation timestamps determine the output frame index;
missing intervals repeat the previous frame and are counted in metadata, so
rendering delays do not silently speed up the replay. Both the frame queue and
large timestamp discontinuities are bounded.

The combined recording command selects `visual_detail:=overview`: dense bearing,
screw and nut visuals are hidden to make this WSL software renderer practical.
The external shells, polar rollers, forks and structural parts retain their
original meshes. A regression test verifies that every collision, mass, inertia,
joint and other nonvisual model element is identical to the full-detail model.
Recording-world shadows are disabled as a further rendering optimization. The
normal caster launch still defaults to full visual detail.

Gazebo's native CameraVideoRecorder was tested but is unsuitable for this
software-rendered setup: its PostRender callback forces rendering at the physics
update rate, even when the camera requests 25 Hz. The ordinary image-stream
approach avoids that cost. See upstream [Sensors.cc](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/sensors/Sensors.cc)
and [CameraVideoRecorder.cc](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/camera_video_recorder/CameraVideoRecorder.cc).
