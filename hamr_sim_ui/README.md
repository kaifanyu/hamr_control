# HAMR Simulation Studio

A local desktop frontend for the existing ROS 2 Jazzy / Gazebo Harmonic
simulation. Choose the **COMPA simulation chassis with front ball-caster
assemblies or ordinary swivel casters** already used by this repository. Ball
casters are the desktop default. This is the repository's simulation
retrofit, with its existing estimated mechanics; it is not a new calibrated
physical HAMR model.

## Start

From the repository root:

```bash
bash hamr_sim_ui/scripts/start.sh
```

The flat test floor opens automatically with **camera sensors off**. Enable
**Cameras** and press **Load world**, or pass `--cameras on`, for the close-up
preview and optional MP4s. The native Gazebo window embeds in the
main panel on X11 / WSLg, with its parent and size verified and kept in sync
when resizing. A native Gazebo window can appear briefly during startup before
attachment. Other window systems use a separate native Gazebo window.
No browser server or cloud service is needed. All simulation processes use a
separate local ROS domain and Gazebo partition.

Options:

```bash
bash hamr_sim_ui/scripts/start.sh --terrain road_bumps
bash hamr_sim_ui/scripts/start.sh --caster swivel --cameras off --speed 0.20
bash hamr_sim_ui/scripts/start.sh --cameras on
bash hamr_sim_ui/scripts/start.sh --no-autostart
bash hamr_sim_ui/scripts/start.sh --check
bash hamr_sim_ui/scripts/start.sh --output "$HOME/Videos/hamr_experiments"
```

Use the system Python, not Conda. The launcher sources the existing simulation
workspace and the project's rootless Gazebo runtime when needed. On a new
machine, install ROS 2 Jazzy on Ubuntu 24.04 first. With its apt repository
configured, install the simulation and desktop dependencies, then build:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-rviz2 \
  ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-default-plugins \
  ros-jazzy-ament-cmake-pytest python3-colcon-common-extensions \
  python3-pyqt5 python3-pytest python3-numpy python3-scipy python3-yaml \
  python3-pil python3-matplotlib python3-vtk9 ffmpeg mesa-utils x11-utils
bash hamr_terrain/scripts/build.sh
bash hamr_sim_ui/scripts/start.sh --check
bash hamr_sim_ui/scripts/start.sh
```

The launcher reports missing ROS/Python/workspace dependencies with instructions.
It does not install packages or change system graphics settings. For a custom
workspace set `HAMR_SWIVEL_CASTER_WS` before building and launching; the default
is `~/hamr_swivel_caster_ws`. Use a local checkout containing the terrain and
caster mesh assets. The current machine already has these dependencies.

## Rendering speed and GPU selection

Both launchers probe the actual OpenGL renderer before opening Qt or Gazebo
and print the selected device. On WSLg they try Mesa D3D12, selecting NVIDIA
when available. On native Linux they probe NVIDIA PRIME or Mesa `DRI_PRIME`
offload and the normal driver. Failed candidates do not alter the launch
environment. Explicit graphics overrides, including software rendering, take
precedence. If no working hardware renderer is found, startup reports the
software or unverified fallback. GPU drivers must already be installed.

The optional `mesa-utils` package supplies `glxinfo` / `eglinfo`. Without a
probe the launcher reports that it cannot verify the renderer. Without
`DISPLAY` it probes surfaceless EGL; GPU availability there can differ from
desktop GLX. It does not force WSLg D3D12 into an unsupported displayless setup.
Existing application processes must be restarted to pick up renderer changes.

Check the renderer with the same environment as the application:

```bash
source hamr_terrain/scripts/env.sh
source hamr_sim_ui/scripts/graphics_env.sh
glxinfo -B
```

The expected result here is `D3D12 (NVIDIA GeForce RTX 4060)` and
`Accelerated: yes`. `direct rendering: Yes` alone does **not** establish GPU
acceleration. Explicit software fallback is available for troubleshooting:

```bash
GALLIUM_DRIVER=llvmpipe bash hamr_sim_ui/scripts/start.sh
```

GPU rendering accelerates the view and camera images. DART contact/dynamics
still runs on the CPU. This model has detailed caster collisions, a 1 ms physics
step, optional two 1280×720 camera streams at 25 FPS, and continuous scene-state
logging; these also affect speed. Cameras are **off by default** in both the
desktop and terminal runner: the camera sensors, camera bridge and unused
sensor renderer are absent. If enabled, uncheck **Cameras** and press
**Load world** to remove them again. The main
interactive 3D view remains available. Unchecking MP4 alone does not disable
camera rendering. Use CLI `--headless --cameras off` when no view is needed.
Increasing the physics time
step or simplifying collisions needs separate tracking/contact validation.

The UI also avoids copying unused telemetry history, converts only the visible
close-up preview at up to 15 wall FPS, and skips the unused overview image
subscription. MP4 recorders keep their independent full-rate streams. Marker
updates use a persistent native Gazebo connection when the bindings are
available (otherwise the CLI), skip unchanged overlays, and sample the terrain
only after reducing display trail points. In a 10,000-point hill-map check,
trail preparation fell from about 124 ms to 15 ms for a changed path and
0.13 ms for an unchanged path. These are UI preparation timings, not overall
simulation speed measurements. Raw recording rates, the physics step and
collision geometry remain unchanged; detailed contacts and ROS processing
can still keep simulation below 1× real time.
Closing or reloading a world also waits for the entire owned process group,
including Gazebo children whose launcher has already exited, so old servers
do not keep running in the background.

When no recording or external subscriber needs the derived encoder outputs,
telemetry skips their JSON and ROS message construction. Encoder sequence
numbers still advance with every sample; subscribing resumes on the next
sample, and recording always retains the complete streams. An isolated
3,209-message callback check fell from 0.853 to 0.295 CPU seconds with no
recording or subscribers. This measures callback overhead, not simulator
real-time factor.

References: [WSLg GPU selection](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg),
[Mesa D3D12](https://docs.mesa3d.org/drivers/d3d12.html),
[Mesa software renderer](https://docs.mesa3d.org/drivers/llvmpipe.html),
[Mesa GPU selection](https://docs.mesa3d.org/envvars.html),
[NVIDIA PRIME offload](https://download.nvidia.com/XFree86/Linux-x86_64/460.27.04/README/primerenderoffload.html).

## Drive and plan

- **3D view:** use Gazebo's mouse controls to orbit, pan and zoom. The native
  renderer displays the real terrain and robot state. Olive lines and spheres
  mark the route; sand shows the measured driven path.
- **Map and wheel type:** under **World**, choose a terrain and either
  **Ball casters** or **Swivel casters**, then press
  **Load world**. This stops the current
  motion, finishes an active recording, and restarts the simulation. There are
  23 terrain maps plus a flat floor. Terrain physics and friction are preserved.
- **Camera on/off:** the **Cameras** checkbox applies with the
  same Load button. Off hides the side camera panel and removes both camera
  sensors and streams. Bags, encoder packets and full-scene replay still work.
- **Place the car:** press **Place car** beside the waypoint map, then click a
  location. Drag from that location to choose the heading as well. Esc or
  right-click cancels. Placement stops motion, saves any active recording,
  waits for the car to settle, then moves the whole vehicle above the terrain.
  It preserves the caster joint angles and the previous pause state. A paused
  world runs briefly to apply stop commands and verify settling before being
  paused again. Gravity settles the terrain clearance when running; placing
  on a slope does not guarantee a stable resting pose. The draft waypoints
  remain available; press **Drive route** or enable WASD to move again.
  If the car cannot settle, placement is cancelled rather than carrying its
  momentum to the new position. The old drawn trail/history is cleared, and
  `events.jsonl` records the placement target and result.
- **Waypoints:** left-click the map to add a point, drag to move one, right-click
  to remove one. The map uses Gazebo world X/Y in meters and the terrain height
  grid. Editing changes the draft; **Drive route** replaces the active
  route from the car's measured current position. Waypoints do not perform
  obstacle avoidance or guarantee that rough terrain is traversable.
- **Presets:** square, straight, out-and-back, and a polygonal circle are placed
  relative to the car's current heading. Press **Set** to place one. The **Route** menu saves/loads JSON waypoints.
- **Trajectory → Speed:** change the requested speed live, from 0.02 m/s up to the
  configured maximum (initially 0.40 m/s). **Limits…** changes
  that maximum and the wheel/acceleration bounds; applying limits stops motion.
  The controller rebuilds the remaining route from the measured pose. Actual
  speed is limited by acceleration, wheel rates and terrain. Routes smoothly
  stop and settle at each waypoint; the circle preset is a sequence of segments.
- **WASD:** click **Enable WASD** in the **Manual** tab and hold W/S for forward/reverse,
  A/D for left/right turns, or Q/E for body-sideways motion through the existing
  offset-turret controller. Adjust **Speed** and **Turn** live.
  Keyboard driving replaces the route. Release keys to stop. Use the control
  window for keyboard focus; clicking the native Gazebo viewport may transfer
  keyboard focus to Gazebo. Enable manual driving again to return focus.
- **Stop:** Space or **Stop** clears the route/manual command. Losing
  window focus or failing to refresh a manual command for 0.30 wall seconds
  also stops manual commands, including when the physics clock is paused.
- **Pause:** freezes simulation time; Resume continues the route.

The close-up camera follows the right front rocker and shows the selected
caster from its outer side. Both assemblies' raw joints are logged. With ball
casters, H1/H2/CENTER readouts show the right caster. With swivel casters,
readouts show left swivel, right swivel and turret angles. Displayed angles
wrap to ±180°; stored joint positions retain their continuous values.

### Why the initial 0.4 m/s maximum?

It is a conservative frontend software setting, not Gazebo's maximum or a
measured motor rating. Increasing it is allowed. There are separate command
wheel rates and planner limits: with the model's 0.1075 m wheel radius,
6 rad/s corresponds to about 0.645 m/s in ideal straight motion. Turns require
additional wheel rate. The default planner's conservative, heading-independent
3.9 rad/s bound limits a sufficiently long segment to about 0.266 m/s; short
segments can be slower because of acceleration and stops at every waypoint.

The UI and CLI report **requested peak** and **planned peak** separately.
Raise the speed ceiling and relevant wheel/acceleration limits together to
experiment above the defaults. These settings do not increase motor effort
limits, grip or contact stability. Higher-speed tracking on a particular map
needs to be checked against the recorded actual velocity.

The 0.60 m/s example below was tested on the flat floor with ball casters: the
logged peak was 0.6015 m/s and the 1.5 m straight route settled at 1.500015 m.
This verifies that setup, not higher-speed turning or rough-terrain tracking.

## Run a complete experiment from the terminal

The runner opens a normal Gazebo window, drives the route, then stops and saves
everything automatically. Camera sensors default to **off** for this mode.
Each command creates a unique `session_<timestamp>/` under `--output` (or
`~/Videos/hamr_sim_ui`). It always saves native Gazebo replay, ROS bag, velocity
and joint CSV, and simulated encoder packets locally. `run.json` records the
settings, actual points, planned peak, completion status and any errors.
The command exits nonzero on failure; Ctrl-C finalizes partial recordings.

```bash
cd ~/hamr_control

# Ordinary swivel wheels, flat floor, 1.5 m straight trajectory.
bash hamr_sim_ui/scripts/run.sh \
  --caster swivel --terrain flat --trajectory straight --speed 0.20 \
  --cameras off --output "$HOME/hamr_runs/swivel_flat"

# Ball wheels and another map. List all map IDs before choosing one.
bash hamr_sim_ui/scripts/run.sh --list-terrains
bash hamr_sim_ui/scripts/run.sh \
  --caster ball --terrain road_bumps --trajectory straight --speed 0.20 \
  --cameras off --output "$HOME/hamr_runs/ball_bumps"

# Ball wheels with both camera MP4s as well as all the normal logs.
bash hamr_sim_ui/scripts/run.sh \
  --caster ball --terrain flat --trajectory square --speed 0.20 \
  --cameras on --video --output "$HOME/hamr_runs/ball_video"

# An explicit faster simulation experiment; not a hardware speed rating.
bash hamr_sim_ui/scripts/run.sh \
  --caster ball --terrain flat --trajectory straight --speed 0.60 \
  --max-speed 0.80 --wheel-rate 12 --planning-wheel-rate 10 \
  --wheel-acceleration 60 --planning-wheel-acceleration 40 --acceleration 0.50 \
  --cameras off --output "$HOME/hamr_runs/ball_fast"

# Replay the exact session path printed by a completed run.
bash hamr_sim_ui/scripts/run.sh \
  --replay "$HOME/hamr_runs/swivel_flat/session_<timestamp>"
```

`--trajectory` accepts `straight`, `square`, `out-and-back` or `circle`.
`--scale 0.5` halves preset distances. Presets start from the measured car pose.
Use `--route-json "$HOME/hamr_route.json"` instead of `--trajectory` for a route
saved by the UI, a `[[x,y], ...]` array, or `{"points": [[x,y], ...]}`. Coordinates
are absolute world meters; an exported route's terrain must match `--terrain`.
The command's `--speed` sets the requested speed even when the file stores one.
Bounds are checked, but routes do not avoid obstacles or guarantee traversal.

Add `--headless` to omit the Gazebo window. `--timeout 1800` sets the total
wall-time limit; long routes in slow simulations may need a larger value.
`--video` requires `--cameras on`. All motion-limit flags also work with
`scripts/start.sh` to initialize the desktop with the same settings.

## Record and replay

Press **Record** for bags, CSV and encoder packets. With **Cameras** enabled,
check **MP4** for both camera videos. The UI waits for the video encoders to receive their first frames
before showing recording ready. Start the route after that status. **Save recording**
finishes the MP4 containers and flushes the bag. Changing worlds or closing the
application also finalizes an active recording. Placing the car finalizes the
current recording before teleporting; start a new recording for the next run.
The full-session native Gazebo replay continues across placements.

Files default to `~/Videos/hamr_sim_ui/session_<timestamp>/`:

| File or directory | Contents |
| --- | --- |
| `session.json`, `world.sdf`, `gui.config` | Terrain, spawn, launch and environment provenance |
| `gazebo_log/` | Native state history covering the **entire session** |
| `events.jsonl` (when used) | Placement targets, results, measured stop velocity and pause state |
| `recording_<timestamp>/bag/` | ROS 2 SQLite3 bag with raw joint states, odometry, clock, TF and encoder topics |
| `velocity.csv` | Simulation timestamps, pose, velocity components, speed and yaw rate |
| `joints.csv` | Both selected casters and the vehicle's measured joint positions/velocities |
| `encoder_packets.jsonl` | Hardware-compatible packet envelopes, explicitly marked as simulated |
| `overview.mp4`, `closeup.mp4` | Camera videos encoded using simulation timestamps |
| `*.mp4.json` | Source frame timestamps, image gaps, encoder drops and verification |
| `recording.json` | Recording interval, joint mapping, message counts, video results and session link |

Recording files in the last six rows are inside `recording_<timestamp>/` for
the desktop, or `recording/` for the terminal runner. `run.json` is present for
terminal runs. Camera-off recordings omit the MP4 files and their manifests.
MP4s use simulation time, so a slow simulation does not produce slow-motion
video. The UI shows the measured real-time factor: 1.0× means simulation keeps
up with wall time. The automatic overview MP4 uses a fixed terrain-wide camera;
it does not capture mouse movements of your interactive viewport. Gazebo's
native viewport video button can capture that view separately.

**Replay…** accepts a session folder, recording folder, or
`gazebo_log` folder. It finalizes the live session and opens Gazebo's recorded
state in the same native 3D viewer. It replays the complete session, including
time before a recording toggle. Use Gazebo's playback controls to seek.
After seeking while paused, press Play or Step to refresh the rendered scene.
Motion controls and live telemetry are disabled during scene replay; view the
saved close-up MP4 and CSV/bag for the recording interval. The shorter ROS bag
is deliberately not played alongside the longer scene as if they were aligned.
Keep the session folder and original model/terrain assets available for replay.

### Encoder compatibility

The simulation publishes and records `/caster_encoder/packets`, metadata,
H1/H2/CENTER joint-state topics, and raw `/joint_states`. The default mapping is:

| Role | Measured simulation joint |
| --- | --- |
| H1 | `right_caster_positive_hemisphere_joint` |
| H2 | `right_caster_negative_hemisphere_joint` |
| CENTER | `right_caster_carrier_joint` |

These packets use the existing receiver/replay schema. Metadata identifies
Gazebo, source joints and simulation timestamps. Raw SPI words/counts and
hardware calibration are not fabricated. The protocol roundtrip tests feed
these packets into the repository's existing `ReplayModel` and verify the
angles. The existing hardware encoder CAD replay is a separate visualization;
its physical sign/zero calibration still applies when comparing hardware.
Do not feed uncalibrated hardware encoder bags into vehicle physics and expect
an exact trajectory: those encoders alone do not contain vehicle pose or all
joint motion. Native Gazebo scene logs provide full simulated-scene replay.

For **ordinary swivel** mode, the same transport roles contain
`left_caster_swivel_joint`, `right_caster_swivel_joint`, and `pitch_yaw_plate_joint`
respectively. Recording metadata identifies this different mapping. These are
swivel/turret angles, not measurements of ball-caster H1/H2/CENTER hardware;
use the native scene replay for the selected simulated wheel mechanism.

## Verification

```bash
source hamr_terrain/scripts/env.sh
PYTHONPATH="$PWD/hamr_sim_ui:$PYTHONPATH" \
  QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m pytest -q \
  hamr_sim_ui/test
```

Tests cover map coordinates, world-physics preservation, route validity,
waypoint completion, the manual deadman, process ownership, recording protocol
compatibility and a real DDS/rosbag roundtrip. Live local smoke checks exercised
native window attachment, camera frames and H.264 MP4s, manual driving and stop,
waypoint following, scene logging and playback. Terrain traversability and
hardware tracking accuracy are not established by these checks.
