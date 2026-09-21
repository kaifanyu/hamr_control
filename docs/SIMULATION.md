# CAD ball caster and continuous trajectory simulation

Use ROS 2 Jazzy on Ubuntu 24.04, including Ubuntu under WSL2 with WSLg for the
Gazebo window. The new entry point runs the complete COMPA simulation chassis
with two CAD ball casters and the shared continuous planner. Simulation uses its
own chassis dimensions and gains; use the [hardware guide](REAL_CAR_CONTINUOUS.md)
for the physical HAMR.

## 1. Install and build

On a fresh Ubuntu/Jazzy installation, install the simulation dependencies:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-rviz2 \
  ros-jazzy-ament-cmake-pytest python3-colcon-common-extensions \
  python3-pytest python3-numpy python3-yaml python3-matplotlib ffmpeg
```

Clone this repository if it is not already present:

```bash
git clone https://github.com/kaifanyu/hamr_control.git ~/hamr_control
cd ~/hamr_control
bash hamr_ball_caster/scripts/build.sh
source hamr_ball_caster/scripts/env.sh
```

The build script builds `hamr_interfaces`, `reference_trajectory` and
`hamr_ball_caster` into `~/hamr_ball_caster_ws`, then runs the caster regression
tests. Set `HAMR_BALL_CASTER_WS` before both scripts to use another workspace.
On the existing WSL machine, `env.sh` also recognizes the previously installed
Gazebo cache if a system installation is absent. A fresh machine can use the
apt packages above; it does not need another user's home directory or raw CAD.
The optional cache bootstrap is described in
[rootless_runtime.md](../hamr_ball_caster/docs/rootless_runtime.md).

Use Ubuntu's Python rather than Conda for ROS. The helper scripts arrange that
interpreter on PATH. Source `env.sh` in every simulation terminal. Simulation
uses local discovery and domain 88; keep it separate from a real robot's ROS
domain. Do not source this simulation environment in a hardware terminal.

## 2. Run the complete vehicle

```bash
cd ~/hamr_control
source hamr_ball_caster/scripts/env.sh
ros2 launch hamr_ball_caster continuous_sim.launch.py
```

Gazebo opens, the vehicle waits for simulation clock/odometry/joints, and the
whole route runs once. The launch shuts down its own simulator after saving
the results. The default points are the `simple_waypoint` route:

```text
(0,0) → (0,2) → (-2,2) → (-2,4.5) → (0,4.5) → (0,2) → (0,0)
```

The reference rounds each 90° corner within 2 cm of the original straight
segments and passes about 2.83 cm from the corner vertex. It decelerates before
each bend and keeps rolling through it. Only the beginning and end have zero
planned speed. This is a deliberately bounded curve, not an exact sharp 90°
corner at nonzero speed. Nominal simulation motion lasts about 73.2 seconds.

Run without a GUI, or choose an explicit **new/empty** output directory:

```bash
ros2 launch hamr_ball_caster continuous_sim.launch.py \
  gui:=false output_dir:=$HOME/hamr_results/tight_run_01
```

The terminal prints the result directory. Check `run_summary.json` for `passed:
true`; `trajectory.json` contains measured tracking, accepted source timestamps,
the planned curve and individual acceptance checks. `configuration.yaml` and
`commands.json` preserve what was executed. A failed run exits with an error.

To inspect the full vehicle without starting a route:

```bash
ros2 launch hamr_ball_caster ball_caster.launch.py model:=compa gui:=true
```

For the separate caster test fixture, use `model:=rig` instead. In WSL, a GUI
requires WSLg/display support; `gui:=false` supports headless execution.

## 3. Save an MP4 replay

```bash
ros2 launch hamr_ball_caster continuous_sim.launch.py \
  record:=true gui:=false \
  output_dir:=$HOME/Videos/hamr_sim/tight_recording_01
```

Open `hamr_continuous_waypoint.mp4` in that directory. Recording runs simulation
at 0.15× wall time to give the renderer enough time; allow roughly 9–12 minutes.
The MP4 plays at simulation speed, with a 25 fps overhead camera and a blue
planned-path overlay. It includes startup and final settling. The recording
pipeline checks trajectory acceptance and gaps in actual camera timestamps;
`run_summary.json` reports the combined result. Generated videos and raw logs
are kept outside the source tree.

For lower-level recorder options:

```bash
ros2 run hamr_ball_caster record_waypoint_run.py --help
```

## 4. Change or preview the path

Simulation defaults to
[`continuous_tight_waypoint_sim.yaml`](../hamr_ball_caster/config/continuous_tight_waypoint_sim.yaml).
Copy it before tuning. `points_m`, `speed_m_s`, `corner_deviation_m`, and the
planning acceleration/jerk limits determine the reference. Keep actuator limits
consistent with the simulation controller. Preview without Gazebo:

```bash
ros2 run hamr_ball_caster run_waypoint_sim.py --profile continuous \
  --config ~/hamr_control/hamr_ball_caster/config/continuous_tight_waypoint_sim.yaml \
  --plan-only /tmp/hamr_sim_plan.json

ros2 launch hamr_ball_caster continuous_sim.launch.py \
  config:=$HOME/my_continuous_sim.yaml
```

The 10 cm blend and stop-at-waypoint profiles remain available for comparisons;
see [planner details](../hamr_ball_caster/docs/CONTINUOUS_WAYPOINT.md) and
[the 2 cm results](../hamr_ball_caster/docs/TIGHT_CONTINUOUS.md).

## 5. New URDF and model scope

| Model | Editable source | Expanded URDF |
|---|---|---|
| Standalone CAD caster | [ball_caster.urdf.xacro](../hamr_ball_caster/urdf/ball_caster.urdf.xacro) | [ball_caster.urdf](../hamr_ball_caster/urdf/ball_caster.urdf) |
| Complete simulation vehicle with two casters | [compa_ball_caster.urdf.xacro](../hamr_ball_caster/urdf/compa_ball_caster.urdf.xacro) | [compa_ball_caster.urdf](../hamr_ball_caster/urdf/compa_ball_caster.urdf) |

The carrier, both hemispheres and both polar rollers retain their extracted CAD
poses and independent passive joints. Runtime meshes and CAD provenance ship in
the package; native SolidWorks/STEP files are not required for a clone to run.
See [CAD findings](../hamr_ball_caster/docs/CAD_FINDINGS.md) for transforms,
hemisphere orientation and reconstruction evidence.

This is the COMPA chassis retrofit used in the validated full-vehicle Gazebo
runs. It is not a new calibrated whole-HAMR hardware digital twin. Mass/friction
and Gazebo velocity actuators remain model assumptions; simulation accuracy
does not establish motor, contact or tracking accuracy on the physical car.

Regenerate the expanded files after editing Xacro:

```bash
cd ~/hamr_control
source hamr_ball_caster/scripts/env.sh
xacro hamr_ball_caster/urdf/ball_caster.urdf.xacro \
  > hamr_ball_caster/urdf/ball_caster.urdf
xacro hamr_ball_caster/urdf/compa_ball_caster.urdf.xacro \
  > hamr_ball_caster/urdf/compa_ball_caster.urdf
```
