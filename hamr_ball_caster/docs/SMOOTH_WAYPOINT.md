# Follow the original route with smooth corner stops

Commands in this guide assume a checkout of this repository. From any directory
inside that checkout, set these portable paths before running the examples:

```bash
export HAMR_REPO="$(git rev-parse --show-toplevel)"
export HAMR_RUNS="${HAMR_RUNS:-$HOME/Videos/hamr_sim}"
```

Recorded videos and full run reports are external artifacts, not files supplied
by a Git clone. Paths under `${HAMR_RUNS}` identify retained runs or new outputs
created by the recording commands.

The smooth profile preserves the original 13 m waypoint route and its sharp
corners. It changes the time schedule: the vehicle slows down before each
waypoint, reaches it at rest, and starts the next segment smoothly. This removes
the original reference's instantaneous change of velocity at a corner.

```text
(0,0) → (0,2) → (−2,2) → (−2,4.5) → (0,4.5) → (0,2) → (0,0)
```

It also stops at the second visit to `(0,2)`, even though the two adjacent
segments are collinear. The geometric route is unchanged. A sharp corner at
nonzero speed cannot have finite acceleration, so retaining both the original
52-second constant-speed schedule and exact corner geometry is not the aim of
this profile.

## Verified complete recording

The final full-vehicle run on this WSL system is saved at:

Play the smooth waypoint MP4 (`${HAMR_RUNS}/smooth_waypoint_20260920_final/hamr_smooth_waypoint.mp4`)

The run completed all six legs with both CAD casters passive. Independent
analysis of all recorded poses passed the configured 5 mm tracking, 2 mm
cross-track, 1 mm corner-overshoot and 0.5 mm final-position limits.

| Measurement | Original recording | Final smooth recording |
| --- | ---: | ---: |
| Maximum geometric path deviation | 50.499 mm | **0.834 mm** |
| Maximum incoming-direction overshoot, entire outgoing legs | 50.499 mm | **0.834 mm** |
| Reported time-aligned tracking RMS | 12.690 mm | **0.265 mm** |
| Reported time-aligned tracking peak | 64.013 mm | **1.064 mm** |
| Motion duration, including endpoint settling | 52.000 s | **98.731 s** |

Geometric deviation and full-leg overshoot decreased by **98.3%**. The smooth
run's peak wheel command was 2.760 rad/s and peak commanded wheel acceleration
was 13.642 rad/s²; neither speed nor acceleration limiting activated. The
recorded comparison uses the different time-alignment conventions explained
below, so geometric error is the direct measure of the path improvement.

This is accurate sampled tracking with small residual error, not a proof of a
perfect continuous-time trajectory. The tradeoff is slower completion: the
same 13 m route takes 98.7 rather than 52 seconds because it decelerates and
stops at the waypoints. The nominal plan is 97.5 seconds and measured settling
adds 1.231 seconds. No caster geometry, contact parameters or physics timestep
were changed for this recording.

The comparison plot (`${HAMR_RUNS}/smooth_waypoint_20260920_final/corner_tracking.png`),
independent metrics (`${HAMR_RUNS}/smooth_waypoint_20260920_final/corner_tracking.json`)
and run summary (`${HAMR_RUNS}/smooth_waypoint_20260920_final/run_summary.json`)
are saved beside the video. The video lasts 105.72 seconds including startup
and final holds and is H.264, 1280 × 720, 25 fps. It passed complete decoding
and timestamp-coverage checks.

## Launch and watch in Gazebo

Build the package after pulling or editing its files:

```bash
cd "${HAMR_REPO}"
bash hamr_ball_caster/scripts/build.sh
source hamr_ball_caster/scripts/env.sh
```

Start the full vehicle with both passive CAD ball casters:

```bash
ros2 launch hamr_ball_caster ball_caster.launch.py \
  model:=compa controller:=false gui:=true
```

In a second WSL terminal, run the smooth profile once:

```bash
source "${HAMR_REPO}/hamr_ball_caster/scripts/env.sh"
ros2 run hamr_ball_caster run_waypoint_sim.py --profile smooth \
  --output /tmp/hamr_smooth_waypoint.json
```

The runner waits for the full vehicle, its odometry, its joint states and all
powered command bridges before moving. It uses simulation time, stops its
powered joints when finished, writes measurements, and reports PASS or FAIL.
The caster joints remain passive. The controller moves the vehicle through
Gazebo's wheel and contact dynamics; it does not place the vehicle on the path
by changing its pose.

Source the same environment in both terminals and keep the ROS domain and
Gazebo partition equal. The script's defaults are isolated from the hardware
configuration. Run only one controller in that simulation. Restart Gazebo before
comparing another run so that the vehicle starts from the same state.

For an automatic headless check without recording, use a new or empty output
directory:

```bash
ros2 run hamr_ball_caster evaluate_waypoint_profile.py --profile smooth \
  --output-dir "$HOME/Videos/hamr_sim/smooth_headless_check"
```

This command starts its own fresh simulator, runs the profile and its strict
checks, saves measurements and logs, and stops the simulator.

## Save a complete MP4

The combined command starts a fresh simulator, records the complete run,
validates the resulting file, and stops its own processes:

```bash
source "${HAMR_REPO}/hamr_ball_caster/scripts/env.sh"
ros2 run hamr_ball_caster record_waypoint_run.py --profile smooth
```

Add `--gui` to see the live window during recording. The terminal prints the
timestamped output directory under `~/Videos/hamr_sim/`. Replay
`hamr_smooth_waypoint.mp4` from that directory. The video preserves simulation
time, including slowing, waypoint stops and final settling; it is not sped up
to the legacy route's duration.

For a controlled comparison of the original timing:

```bash
ros2 run hamr_ball_caster record_waypoint_run.py --profile legacy
```

That saves `hamr_simple_waypoint.mp4` and selects the retained baseline
configuration. It runs the old abrupt reference with its original controller
settings. The two recording profiles use the same camera, vehicle geometry,
caster contact model and physics launch settings.

See [RECORDING.md](RECORDING.md) for video metadata, timing checks, manual camera
recording and troubleshooting.

## Configuration and timing

| Profile | Default configuration | Reference behavior |
| --- | --- | --- |
| `smooth` | `config/smooth_waypoint_sim.yaml` | Smooth start/stop at every waypoint, with endpoint settling before departure |
| `legacy` | `config/simple_waypoint_sim.yaml` | Constant speed on each straight, instantaneous changes of direction |

Both the runner and recorder accept `--config /absolute/path/to/config.yaml`.
The selected file is authoritative unless a command-line override is provided.
For example, `--speed 0.20` sets a 0.20 m/s reference speed limit. In the smooth
profile this is a maximum, not a constant speed; the planner computes longer
segment durations when its other bounds require them.

Each straight segment uses a quintic progress law,
`s(u) = 10u³ − 15u⁴ + 6u⁵`, where `u` increases from zero to one over that
segment's planned duration. Position, velocity and acceleration are continuous
across the route (`C²` continuity), with zero velocity and acceleration at each
waypoint. Jerk can change at a boundary; this is not a jerk-continuous planner.

The planner chooses the duration to satisfy all of these nominal kinematic
bounds at every possible chassis heading:

| Planning bound | Default |
| --- | --- |
| Peak translation speed | 0.25 m/s |
| Peak translation acceleration | 0.10 m/s² |
| Wheel speed | 65% of the 6 rad/s command limit: 3.9 rad/s |
| Wheel acceleration | 50% of the 15 rad/s² command limit: 7.5 rad/s² |

Wheel acceleration bounds include the changing chassis heading while following
a straight world path. Reserving speed and acceleration below the actuator
command limits leaves room for feedback correction. These are conservative
kinematic bounds for the nominal model, not measured motor torque or friction
limits. The controller's speed and acceleration limits remain active.

The smooth profile does not advance directly into the next segment as soon as
the planned segment time expires. It holds the endpoint until measured position
error and measured speed stay within their configured tolerances for the
configured dwell. It fails if settling exceeds the timeout. Holding the
reference does not move the measured position or erase the remaining error.
The default release condition is position error no greater than **0.5 mm** and
speed no greater than **3 mm/s**, maintained for **0.20 seconds**. The maximum
wait is eight seconds per waypoint.

The smooth controller updates at 100 Hz and receives raw Gazebo odometry at
50 Hz. It predicts the current control position over the bounded odometry age
and uses a 5 ms reference lookahead to account for a portion of the command
hold. Performance measurements use the raw odometry position and evaluate the
reference at that measurement's own source timestamp. Prediction is used for
control, not substituted for measured accuracy.
The selected simulation configuration uses position proportional gain 16 s⁻¹
alongside planned velocity feedforward. The gain was selected with the feasible
reference and command limits retained; it is not a proposed hardware gain.

The report separates these clocks:

- `elapsed_s` and `trajectory_time_s`: time at the raw odometry source stamp,
  measured from the beginning of the run and the beginning of motion,
  respectively. `reference_m` is evaluated at that same source stamp.
- `controller_elapsed_s`: elapsed time at the controller update, including the
  delay since that odometry sample.
- `reference_time_s`: progress through the nominal planned trajectory at the
  controller update; pauses during an endpoint hold. It is diagnostic planner
  time, not the timestamp used to evaluate the recorded `reference_m`.
- `planned_reference_schedule_s`: nominal waypoint times before endpoint holds.
- `reference_schedule_s`: actual scheduled endpoint arrivals after prior holds.

`summary.reference_duration_s` includes the endpoint settling used in that run.
The planned duration is also recorded. Startup and final holds are separate.

## Read the measurements

The recorded `trajectory.json` contains actual and reference positions, errors,
wheel commands, joint positions, profile, configuration and timing. Generate a
plot and CSV from it:

```bash
/usr/bin/python3 "${HAMR_REPO}/hamr_ball_caster/tools/plot_waypoint_run.py" \
  /path/to/recording/trajectory.json
```

The plot labels the profile and speed limit, shows actual Gazebo positions, and
marks waypoint times. Its error curve remains on actual elapsed simulation time,
so settling is visible. The CSV includes both actual and reference clocks.
Each combined recording also retains the exact `configuration.yaml` used for
the run, its hash and any command-line speed override.

The retained legacy profile compares its latest raw odometry against the
controller clock, as the original runner did. Its time-aligned tracking error
therefore includes that older sampling convention. Geometric cross-track error
and corner overshoot are the more direct comparison of actual path accuracy;
they do not depend on aligning a moving reference clock to an odometry sample.

Run the independent geometric and timing analysis with the same strict
acceptance limits used for the smooth profile:

```bash
/usr/bin/python3 "${HAMR_REPO}/hamr_ball_caster/tools/analyze_corner_tracking.py" \
  /path/to/recording/trajectory.json \
  --max-tracking-error 0.005 --max-cross-track 0.002 \
  --max-corner-overshoot 0.001 --max-final-error 0.0005
```

This produces `corner_tracking.json`, `.png` and `.pdf` beside the report. Add
`--baseline /path/to/legacy/trajectory.json` to overlay the original run. The
analysis includes every sample of each leg, so a small turn-window error alone
cannot hide a deviation later along a straight.
When explicit limit flags are omitted, the analyzer uses the acceptance limits
saved in the report's configuration, if present.

Small residual contact, measurement and discrete-time errors can remain. A
validated simulation run establishes the measured accuracy of this nominal
model. It does not establish identical hardware performance; the hardware wheel
speed loop and physical calibration identified in the
[corner investigation](corner_analysis/README.md) require their own validation.
