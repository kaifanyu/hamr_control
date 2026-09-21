# Tighter continuous corners: 2 cm preset

Commands in this guide assume a checkout of this repository. From any directory
inside that checkout, set these portable paths before running the examples:

```bash
export HAMR_REPO="$(git rev-parse --show-toplevel)"
export HAMR_RUNS="${HAMR_RUNS:-$HOME/Videos/hamr_sim}"
```

Recorded videos and full run reports are external artifacts, not files supplied
by a Git clone. Paths under `${HAMR_RUNS}` identify retained runs or new outputs
created by the recording commands.

The `continuous_tight_waypoint_sim.yaml` preset follows the original waypoint
order with a maximum planned distance of 20 mm from the original straight
segments. It slows down before each corner, rolls through a small curved blend,
and accelerates afterward. The collinear return waypoint is passed at cruising
speed. Only the beginning and end of the route require a stop.

This uses the existing continuous planner and controller through `--profile
continuous --config ...`. The separate configuration changes only the preset
name and corner-deviation budget. Acceleration, jerk, wheel limits, feedback
gain, odometry prediction and acceptance thresholds stay the same as the
validated 10 cm configuration. The 10 cm and stopping presets remain available.

## Verified replay and measurements

Watch the complete 80.08-second MP4 (`${HAMR_RUNS}/continuous_tight_waypoint_20260921_validated/hamr_continuous_waypoint.mp4`)
or inspect the corner-detail and speed comparison (`${HAMR_RUNS}/continuous_tight_waypoint_20260921_validated/profile_comparison.png`).
The new run finishes all four turns without an observed interior stop. It
slows noticeably through the tight bends, then returns to cruising speed.

| Measurement | Recorded 2 cm run |
| --- | ---: |
| Motion duration | 73.194 s |
| Tracking RMS / peak against planned curve | 0.397 / 1.658 mm |
| Conservative maximum distance from planned curve | 1.486 mm |
| Sampled maximum distance from original straight polyline | 19.900 mm |
| Sampled nearest distance to original corner vertices | 28.011–28.660 mm |
| Minimum interior speed | 0.06264 m/s |
| Wheel-speed / acceleration-limit activations | 0 / 0 |

The corner-vertex distances are measured during each corresponding turn.
Searching the entire route for the nearest visit would conceal rounding at
`(0,2)`, since the vehicle passes that coordinate again on its return straight.
These are distances of the tracked base point, not robot-footprint clearances.

The same original waypoint order is retained. The previous 10 cm blend took
65.155 s and passed approximately 141 mm from the corner vertices. The new
2 cm blend takes 73.194 s and passes approximately 28 mm away. The earlier
exact-waypoint stopping profile took 98.731 s. The tighter and wider continuous
presets use the same controller and limits; the stopping result is retained as
an earlier-profile comparison.

Three fresh full-route simulations passed without relaxing the limits:

| Run | Physics step | Peak tracking error | Conservative curve error | Minimum interior speed |
| --- | ---: | ---: | ---: | ---: |
| nominal | 1.0 ms | 1.656 mm | 1.485 mm | 0.06210 m/s |
| half_step | 0.5 ms | 1.730 mm | 1.494 mm | 0.06342 m/s |
| recording | 1.0 ms | 1.658 mm | 1.486 mm | 0.06264 m/s |

The largest commanded wheel acceleration in the camera run was
14.779 rad/s² against the
15 rad/s² cap. This leaves limited margin at that peak, even though no command
was limited. The two headless peaks were approximately 10.2 and 10.1 rad/s²;
feedback delivery and simulated contact transients influence this demand.

Every control update is included in the zero-limiter check. Acceptance also
requires source-aligned tracking error ≤5 mm, curve error ≤2 mm, final error
≤0.5 mm, interior speed ≥0.03 m/s, and corner slowdown ≤30%. Raw Gazebo poses
and source timestamps determine the measurements; controller predictions do
not replace measured positions in these checks. Both headless runs and the
recorder cleaned up their owned simulator processes normally.

The MP4 is H.264, 1280×720 at 25 fps and includes startup and final holds.
It passed complete decoding, source-clock coverage and the moving-image-gap
check; its maximum motion image gap was 0.160 s
against the 0.16 s threshold. The exported plan equals the executed plan, and
configuration, source, world and reference-mesh hashes were verified. The blue
reference overlay is visual only and preserves the original physics world.

The existing **195 Python tests** passed, with **205 checks** including the
ten CTest wrappers. The new preset requires no production controller or planner
changes; its numerical reference and physical execution were checked explicitly.
The new comparison tool was exercised on the retained simulation reports.

- Raw recording report (`${HAMR_RUNS}/continuous_tight_waypoint_20260921_validated/trajectory.json`)
- Independent continuous analysis (`${HAMR_RUNS}/continuous_tight_waypoint_20260921_validated/continuous_tracking.json`)
- Independent profile comparison (`${HAMR_RUNS}/continuous_tight_waypoint_20260921_validated/profile_comparison.json`)
- Video and provenance verification (`${HAMR_RUNS}/continuous_tight_waypoint_20260921_validated/verification.json`)
- [All new run metrics and report hashes](validation/tight_continuous_runs.json)
- [Implementation and configuration hashes](validation/tight_continuous_implementation.json)
- [Build and test log](validation/tight_continuous_build.log)

These are sampled simulation results. The small remaining tracking error and
speed ripple are separate from the planned geometric rounding.

## Planned motion

| Quantity | 2 cm preset |
| --- | ---: |
| Maximum planned distance from original straight segments | 20.000 mm |
| Closest planned pass to each original corner vertex | 28.284 mm |
| Minimum local turning radius of the reference | approximately 50.3 mm |
| Planned speed through each complete corner blend | 0.07091 m/s |
| Maximum straight speed | 0.250 m/s |
| Motion duration | 73.193743 s |
| Rounded path length | 12.876881 m |
| Cartesian acceleration limit | 0.100 m/s² |
| Cartesian jerk limit | 0.200 m/s³ |

The four sharp vertices remain slightly rounded: the preset does not promise
an exact 90-degree change of direction at nonzero speed. The 20 mm rounding
budget is also distinct from tracking error. Actual deviation from the original
polyline includes both intended rounding and measured tracking error.

The planner exports a complete time schedule before movement begins. Its
velocity and acceleration are continuous, with bounded jerk. The controller
uses feedforward velocity plus live position feedback. Both CAD casters remain
passive and move through the same Gazebo contact dynamics.

## Run and record in this WSL system

The package is built in `${HOME}/hamr_ball_caster_ws`. In a WSL Bash terminal:

```bash
cd "${HAMR_REPO}"
source hamr_ball_caster/scripts/env.sh
ros2 run hamr_ball_caster record_waypoint_run.py --profile continuous \
  --config "${HAMR_REPO}/hamr_ball_caster/config/continuous_tight_waypoint_sim.yaml"
```

This starts its own isolated full-vehicle simulation, records the route, checks
the motion and video, and stops its processes. Add `--gui` to see Gazebo live.
Results go into a new timestamped directory under `~/Videos/hamr_sim/`, including
`hamr_continuous_waypoint.mp4`, raw measurements and the exact configuration
snapshot. The profile remains named `continuous`; the retained configuration
identifies the tighter preset.

Recording defaults to real-time factor 0.15 for this WSL software renderer.
Expect approximately nine minutes of wall time; the MP4 plays at normal
simulation speed. Blue floor markings show the intended tight curve and white
markings show the original straight route.

For a run without recording:

```bash
ros2 run hamr_ball_caster evaluate_waypoint_profile.py --profile continuous \
  --config "${HAMR_REPO}/hamr_ball_caster/config/continuous_tight_waypoint_sim.yaml" \
  --output-dir "$HOME/Videos/hamr_sim/tight_corner_check"
```

Use a new or empty output directory. Add `--max-step-size 0.0005` to repeat at
half the default physics timestep, also choosing a different output directory.

For manual operation, start Gazebo in one terminal:

```bash
source "${HAMR_REPO}/hamr_ball_caster/scripts/env.sh"
ros2 launch hamr_ball_caster ball_caster.launch.py \
  model:=compa controller:=false gui:=true
```

Then start the trajectory in a second terminal:

```bash
source "${HAMR_REPO}/hamr_ball_caster/scripts/env.sh"
ros2 run hamr_ball_caster run_waypoint_sim.py --profile continuous \
  --config "${HAMR_REPO}/hamr_ball_caster/config/continuous_tight_waypoint_sim.yaml" \
  --output /tmp/hamr_tight_continuous.json
```

Restart Gazebo before another manual run to restore the initial vehicle state.
The combined recording command additionally prepares the blue reference overlay
and camera world automatically.

After editing or moving the package, rebuild before use:

```bash
cd "${HAMR_REPO}"
bash hamr_ball_caster/scripts/build.sh
source hamr_ball_caster/scripts/env.sh
```

See the [continuous trajectory guide](CONTINUOUS_WAYPOINT.md) for controller,
planner and recorder details and the [10 cm validation report](CONTINUOUS_VALIDATION.md)
for the preceding configuration. Simulation accuracy depends on the nominal
physical model and is not a measured hardware guarantee.

## Regenerate the profile comparison

```bash
/usr/bin/python3 "${HAMR_REPO}/hamr_ball_caster/tools/compare_continuous_runs.py" \
  /path/to/new/trajectory.json \
  --wide /path/to/wide-continuous/trajectory.json \
  --stop /path/to/smooth-stops/trajectory.json \
  --output /path/to/new/profile_comparison
```

The comparison requires explicit paths to the retained 10 cm and stopping reports
with `--wide` and `--stop`. It recomputes the measurements and
produces JSON, PNG and PDF outputs, with each original corner visit separated
from later crossings of the same waypoint. `--no-plot` exports metrics only.
