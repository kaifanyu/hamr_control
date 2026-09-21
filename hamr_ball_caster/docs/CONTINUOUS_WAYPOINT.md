# Continuous motion through rounded corners

Commands in this guide assume a checkout of this repository. From any directory
inside that checkout, set these portable paths before running the examples:

```bash
export HAMR_REPO="$(git rev-parse --show-toplevel)"
export HAMR_RUNS="${HAMR_RUNS:-$HOME/Videos/hamr_sim}"
```

Recorded videos and full run reports are external artifacts, not files supplied
by a Git clone. Paths under `${HAMR_RUNS}` identify retained runs or new outputs
created by the recording commands.

The `continuous` profile follows the original waypoint order with a rounded
reference around each sharp corner. The vehicle starts from rest, keeps moving
through the interior route, and comes to rest at the final point. It does not
use the smooth profile's stop-and-settle behavior at intermediate waypoints.

The existing default remains `smooth`. Select `--profile continuous` explicitly
to use the new rounded reference. The original waypoint order is unchanged:

```text
(0,0) → (0,2) → (−2,2) → (−2,4.5) → (0,4.5) → (0,2) → (0,0)
```

Continuous motion with finite acceleration requires rounding these sharp
corners. The rounded reference therefore passes near the four corner vertices;
it does not claim to pass through those exact vertices at nonzero speed.
The collinear return visit to `(0,2)` does not require a corner or a stop.

For a tighter 2 cm rounding budget, use the separate configuration in the
[tight continuous trajectory guide](TIGHT_CONTINUOUS.md). It uses this same
pipeline with a lower planned corner speed.

## Verified full-vehicle replay

Play the final continuous MP4 (`${HAMR_RUNS}/continuous_waypoint_20260920_validated/hamr_continuous_waypoint.mp4`)

The complete recorded run passed the trajectory and independent continuous-motion
checks. It follows a **12.384 m rounded reference in 65.155 seconds**, with
positive measured speed through every interior corner. The startup and final
ramps bring the vehicle from and back to rest.

| Measurement | Recorded result |
| --- | ---: |
| Tracking RMS relative to the rounded reference | **0.419 mm** |
| Maximum time-aligned tracking error | **1.656 mm** |
| Conservative maximum local curve error, including chord approximation | **1.492 mm** |
| Planned corner speed | **0.15856 m/s** |
| Minimum measured interior/corner speed | **0.13923 m/s** |
| Largest sampled slowdown relative to planned corner speed | **12.2%** |
| Wheel speed/acceleration limit activations | **0 / 0** |

The largest measured wheel commands were 3.198 rad/s and 10.257 rad/s²,
within the retained 6 rad/s and 15 rad/s² command limits. The independent
5 mm tracking, 2 mm curve-error, 0.5 mm final-position and corner-speed checks
all passed. These are recorded-sample results with small residual tracking
error; they do not establish zero error at every instant.

The planned rounding intentionally deviates up to **100 mm** from the original
sharp polyline and passes **141.4 mm** from each original corner vertex. Those
distances are separate from the approximately 1–2 mm tracking error to the
rounded curve. The blue floor ribbon displays the new reference; the white
markings retain the old polyline.

The continuous-motion plot (`${HAMR_RUNS}/continuous_waypoint_20260920_validated/continuous_tracking.png`),
independent metrics (`${HAMR_RUNS}/continuous_waypoint_20260920_validated/continuous_tracking.json`),
raw run report (`${HAMR_RUNS}/continuous_waypoint_20260920_validated/trajectory.json`)
and verification record (`${HAMR_RUNS}/continuous_waypoint_20260920_validated/verification.json`)
are saved beside the MP4. See [CONTINUOUS_VALIDATION.md](CONTINUOUS_VALIDATION.md)
for the additional headless and smaller-timestep checks.
The stop-versus-continuous speed comparison (`${HAMR_RUNS}/continuous_waypoint_20260920_validated/stop_vs_continuous.png`)
shows the difference from the earlier stop-at-waypoint solution. The final
package passed 195 Python tests, with 205 checks including CTest wrappers.

The MP4 is **71.72 seconds**, H.264, 1280 × 720 at 25 fps, including startup
and final holds. It passed full-video decoding and timestamp-coverage checks.
Software rendering produced 1,708 camera images; the recorder repeated 85
frames to preserve the simulated timing, giving 1,793 encoded frames with no
encoding-queue drops. The largest encoded source-image gap during motion was
0.16 simulation seconds, passing the automatic replay-quality gate. Independent
decoded-frame analysis found no interior freeze longer than 0.16 seconds and
only one interval equal to that threshold, on the final straight. All four
corner intervals had no such freezes. Capture took about 487 wall seconds at
real-time factor 0.15; the MP4 retains the original simulation timing.

The exported plan exactly matches the executed plan, and configuration, plan,
world and ribbon-mesh hashes were verified. Removing the added visual model
from the generated recording world reproduces the source world's structure;
the overlay has no collision geometry. Actual decoded frames were inspected
at the start, all four corner passes and the final hold.

The initial capture (`${HAMR_RUNS}/continuous_waypoint_20260920_final/hamr_continuous_waypoint.mp4`)
is retained for comparison. It contains longer repeated-frame intervals from
software rendering; use the validated replay linked above to view continuous
motion.

## Run it on this WSL system

Rebuild after updating the package, then source its environment:

```bash
cd "${HAMR_REPO}"
bash hamr_ball_caster/scripts/build.sh
source hamr_ball_caster/scripts/env.sh
```

To run a fresh headless simulation and save its measurements:

```bash
ros2 run hamr_ball_caster evaluate_waypoint_profile.py --profile continuous \
  --output-dir "$HOME/Videos/hamr_sim/continuous_headless_check"
```

Use a new or empty output directory. The command starts the full vehicle with
both CAD casters, waits for its feedback and command bridges, executes the
reference, runs acceptance checks, saves the report, and stops its own processes.

To record the complete run as an MP4 instead:

```bash
ros2 run hamr_ball_caster record_waypoint_run.py --profile continuous
```

The recorder creates a timestamped directory named `continuous_waypoint_...`
under `~/Videos/hamr_sim/` and writes `hamr_continuous_waypoint.mp4`. Add `--gui`
to open the live Gazebo window while recording. The recording uses simulation
timestamps, so rendering delays do not shorten the simulated motion.
Continuous recording defaults to `--real-time-factor 0.15`, giving this WSL
software renderer more wall time per camera frame. This slows capture, while
the MP4 still plays at the original simulation speed. The smooth and legacy
recording profiles retain their 0.5 defaults. An explicit `--real-time-factor`
overrides the selected profile's default.
For this profile, the combined command also fails its video-quality check if
successive source images reaching the encoder are more than 0.16 simulation
seconds apart during motion. Gaps confined to startup or final holds are
excluded. Repeated output frames preserve timing and do not count as new source
images for this check. The result is recorded as `video_quality` in
`run_summary.json`.

For a manual live run, start Gazebo in one terminal:

```bash
source "${HAMR_REPO}/hamr_ball_caster/scripts/env.sh"
ros2 launch hamr_ball_caster ball_caster.launch.py \
  model:=compa controller:=false gui:=true
```

Then run the controller from a second terminal with the same environment:

```bash
source "${HAMR_REPO}/hamr_ball_caster/scripts/env.sh"
ros2 run hamr_ball_caster run_waypoint_sim.py --profile continuous \
  --output /tmp/hamr_continuous_waypoint.json
```

Restart Gazebo before another manual run so that each experiment begins with
the same vehicle state. The combined evaluation and recording commands start
their own isolated simulations and require no separately opened simulator.

## Configuration and tradeoff

The profile selects `config/continuous_waypoint_sim.yaml`. `--config` overrides
that file in the runner, evaluator or recorder. The combined commands copy its
exact bytes to `configuration.yaml` in the result directory and run from that
snapshot. The record command and runner also accept an optional `--speed` override;
omitting it preserves the configured speed.

The nominal configuration requests a peak speed of 0.25 m/s, translation
acceleration no greater than 0.10 m/s², Cartesian jerk no greater than 0.20 m/s³,
and a maximum intentional corner deviation of 0.10 m from the original
polyline. Wheel planning uses 65% of the 6 rad/s command speed limit and 50%
of the 15 rad/s² command acceleration limit, leaving feedback headroom. These
limits describe the nominal simulation and have not been calibrated as hardware
limits.

Each rounded section uses a symmetric quintic Bézier blend with zero curvature
at its joins. The reference maintains a positive constant speed through that
blend; quintic velocity ramps on the neighboring straights connect the speeds.
Cartesian velocity and acceleration are continuous. Jerk stays bounded but can
have finite jumps at joins; this is not a jerk-continuous reference.

Smaller rounding tolerance forces tighter curvature and can require slower
motion. Larger tolerance permits wider turns and changes the geometric path
more. The planner computes its time schedule from the configured limits;
0.25 m/s is a requested maximum, not a promise of constant speed through a turn.

The controller retains wheel speed and acceleration limits. Both ball casters
remain passive, and the vehicle moves through Gazebo wheel/contact dynamics.
The continuous profile does not change the URDF, contact parameters or physics
timestep.

The selected simulation controller uses planned velocity feedforward and
position proportional gain **12 s⁻¹**, updating at 100 Hz with 50 Hz raw Gazebo
odometry. Its control pose is extrapolated over the complete accepted source
age, up to 100 ms, and feedback older than that limit is rejected. This keeps
the current reference and predicted control pose on the same clock instead of
manufacturing position error when odometry arrives late. Reported tracking
accuracy still uses raw measured positions at their source timestamps. These
simulation settings are not a proposed hardware gain or calibration.

## Distinguish corner rounding from tracking error

Three different distances matter:

| Quantity | Meaning |
| --- | --- |
| Planned deviation from the original polyline | Intentional geometric rounding specified by `corner_deviation_m` |
| Nearest pass to an original corner vertex | How close the rounded reference gets to that particular waypoint; this can exceed its distance to the nearest line segment |
| Actual tracking error | Difference between measured vehicle motion and the rounded reference it was commanded to follow |

For example, passing 0.10 m from each side of a right-angle corner can place
the reference about 0.14 m from the corner vertex. That is planned rounding,
not 0.14 m of controller error. Conversely, reporting a small error to the
rounded reference does not imply that the vehicle followed the original sharp
polyline exactly.

The recording retains the original floor markings and adds a blue ribbon
showing the planned continuous curve. Before launch, the recorder exports the
plan from the retained configuration and generates an overlay world from those
exact plan samples. The ribbon is visual-only, with no collision geometry;
all original physics settings and floor markers are preserved. `reference_plan.json`,
`recording_world.sdf`, the overlay mesh and their provenance are saved alongside
the video. The tracking plot also draws the original polyline, the planned
continuous curve, and the measured vehicle path separately:

```bash
/usr/bin/python3 "${HAMR_REPO}/hamr_ball_caster/tools/plot_waypoint_run.py" \
  /path/to/continuous_run/trajectory.json
```

It produces `trajectory_tracking.png`, `.pdf` and a measurement CSV. For this
profile it marks the planned corner intervals, not stop or exact-arrival events.

To regenerate the independent continuous-motion analysis and plots:

```bash
/usr/bin/python3 "${HAMR_REPO}/hamr_ball_caster/tools/analyze_continuous_tracking.py" \
  /path/to/continuous_run/trajectory.json
```

This writes `continuous_tracking.json`, `.png` and `.pdf` beside the report,
using its saved acceptance limits.

The full report includes `continuous_plan` with sampled reference positions,
velocities and accelerations plus corner timing, and `continuous_acceptance`
with the independent analysis. `summary.passed` includes that analysis alongside
the full-vehicle checks. Corner-speed checks assess whether the vehicle kept
moving through the corners and how much it slowed relative to its plan; startup
and final stopping are treated separately.

Raw Gazebo odometry and its own source timestamps are used to measure tracking.
Controller prediction is not substituted for measured accuracy. Small residual
errors can remain, and a successful nominal simulation does not establish
identical performance on hardware.
