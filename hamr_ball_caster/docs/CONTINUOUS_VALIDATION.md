# Continuous trajectory implementation and validation

Commands in this guide assume a checkout of this repository. From any directory
inside that checkout, set these portable paths before running the examples:

```bash
export HAMR_REPO="$(git rev-parse --show-toplevel)"
export HAMR_RUNS="${HAMR_RUNS:-$HOME/Videos/hamr_sim}"
```

Recorded videos and full run reports are external artifacts, not files supplied
by a Git clone. Paths under `${HAMR_RUNS}` identify retained runs or new outputs
created by the recording commands.

Watch the complete 71.72-second MP4 (`${HAMR_RUNS}/continuous_waypoint_20260920_validated/hamr_continuous_waypoint.mp4`)
or inspect the measured path and speed comparison (`${HAMR_RUNS}/continuous_waypoint_20260920_validated/stop_vs_continuous.png`).
The final recorded run passes both the physical-motion and recording-quality
checks: **1.656 mm peak tracking error**,
**1.492 mm conservative maximum curve deviation**,
and **0.139 m/s minimum measured interior speed**.
No intermediate stop or wheel-command limiting was observed.

The `continuous` profile plans the complete route before motion, then follows
that reference using live Gazebo feedback. It starts and finishes at rest and
keeps moving through four rounded corners and the collinear return waypoint.
It retains the feedforward velocity and position-feedback controller with the
HAMR offset-drive Jacobian. The [run guide](CONTINUOUS_WAYPOINT.md) contains
launch, recording and configuration steps; `smooth` and `legacy` remain available.

## Planned motion and geometric tradeoff

| Quantity | Continuous reference |
| --- | ---: |
| Motion duration | 65.155461 s |
| Rounded path length | 12.384405 m |
| Peak straight speed | 0.250 m/s |
| Speed through each complete corner blend | 0.158561 m/s |
| Maximum intended distance from the original polyline | 0.100 m |
| Nearest planned distance to each original corner vertex | 0.141421 m |
| Cartesian acceleration bound | 0.100 m/s² |
| Cartesian jerk bound | 0.179600 m/s³ |
| Nominal wheel-speed bound over all chassis headings | 3.668057 rad/s |
| Nominal wheel-acceleration bound over all chassis headings | 4.849292 rad/s² |

The four corners use symmetric polynomial blends with zero curvature at their
straight-section joins. Arc length is evaluated by Gaussian quadrature and
inverted with Newton refinement, so position and analytic velocity agree.
Quintic velocity ramps on the straight portions reach the appropriate speed
before each blend. A forward/backward reachability pass reduces corner speeds
if short straight sections cannot connect their speed limits.

Velocity and acceleration are continuous. Jerk is bounded but can have finite
jumps at the curve/straight joins; this implementation does not claim continuous
jerk. The bounds combine analytic ramp extrema with interval bounds on curvature
and its derivative with respect to arc length. The curved-path wheel bound is
`K*(abs(v_dot) + v²*(abs(curvature) + 1/b))`, where
`K = sqrt(1 + (a/b)²)/r`. It includes the chassis-heading evolution for HAMR's
tracked point ahead of the drive axle.

The old stop profile traverses the exact 13 m polyline in 98.731 seconds including
settling. The continuous plan takes about 34% less motion time and has a shorter
rounded path. The two profiles therefore represent a different geometric
tradeoff. The 100 mm intentional rounding and 141 mm corner-vertex miss are not
controller tracking errors; tracking is measured against the intended curve.

## Final full-vehicle simulation results

These two fresh runs use the final controller, gain 12 s⁻¹, unchanged command
limits, and full accepted-age odometry prediction. Their configuration and
implementation hashes match the delivered source.

| Run | Physics step | RMS tracking error | Peak tracking error | Conservative curve error | Minimum interior speed | Result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Final finer-timestep validation | 0.5 ms | 0.416 mm | 1.734 mm | 1.492 mm | 0.1387 m/s | Pass |
| Final camera recording | 1 ms | 0.419 mm | 1.656 mm | 1.492 mm | 0.1392 m/s | Pass |

Both execute the complete 65.155-second reference and final hold on the full
vehicle with both passive CAD ball casters. Neither wheel limiter activated.
The final camera run's maximum commanded wheel acceleration is
10.257 rad/s², below the unchanged 15 rad/s² limit.
Its worst measured corner slowdown is 12.2%, below the 30% acceptance
limit. These runs check consistency at two timesteps; different feedback delivery
and rendering conditions mean they are not a formal convergence study.

The MP4 includes startup and final holds and preserves simulation timing.
It comes from a Gazebo camera; measured positions were not moved onto the
reference to produce the video. Full reports, settings, hashes and trial history:

- [Run comparison and report hashes](validation/continuous_waypoint_runs.json)
- Headless results and retained trials (`${HAMR_RUNS}/continuous_validation_20260920`)
- Final recording and verification artifacts (`${HAMR_RUNS}/continuous_waypoint_20260920_validated`)

## Feedback and rendering issues found during validation

The initial gain-16 simulations passed physical-motion checks, but their
maximum wheel acceleration left little feedback margin. A slower camera run
then exposed three acceleration-limiter activations and correctly failed.
The limiter threshold was not relaxed to accept that run.

One recorded event reconstructed a controller state that was 54 ms old while
pose prediction stopped at 40 ms. The reference had already advanced to the
current control time. This mismatch created about 2.2 mm of artificial forward
error, followed by a large corrective wheel command when newer feedback arrived.
A 60 ms gap between control-recorded fresh poses supports this stale-state
mechanism; it does not by itself distinguish transport delay from callback
scheduling or prove a publisher dropout.

Continuous mode now predicts across the complete accepted source age, up to
100 ms. The existing source-age rejection guard remains unchanged, and the
old stop/legacy profiles retain their original prediction behavior. A synthetic
constant-velocity regression reproduces the 54 ms case and verifies that the
new prediction reaches the current pose without inventing a tracking error.
Constant-velocity prediction still has acceleration and velocity-noise error;
raw, unmodified pose measurements determine acceptance.

A separate event came mainly from feedback amplifying a measured velocity
transient: approximately 92% of its wheel-command change was the correction
term. Reducing the simulation-only position gain from 16 to 12 s⁻¹ gave more
acceleration margin. Both gain-12 trial timesteps passed before the timing fix;
the final runs above validate the combined correction. These changes do not
alter the Jacobian or hardware configuration.

Earlier candidates are retained as evidence, not presented as final successes:
the first camera run (`continuous_waypoint_20260920_final`) passed physics but
contained a 0.56-second moving-image freeze. The second
(`continuous_waypoint_20260920_replay`) passed the new image-gap check but failed
the zero-limiter check. An earlier gain-16 finer-timestep launcher's parent was
terminated with status 143; its independently running simulator/controller
completed without reset, and owned-group cleanup was recovered explicitly.
That interruption is recorded in its summary. Both final runs completed with
their normal harness cleanup.

## Acceptance method

The runner records unmodified Gazebo odometry. The analyzer independently
recomputes tracking error using references evaluated at each pose's source
timestamp, and cross-checks those references against the exported plan.
Controller pose prediction is not used as a substitute for measured accuracy.

Geometric error is distance to nearby finite chords of the dense planned curve
within a half-second local reference window. This avoids using another visit
to the same route crossing to conceal local tracking error. The analyzer adds
the certified chord approximation bound, approximately 1.25 micrometres, to the
largest observed chord distance for geometric acceptance.

Every full run must satisfy all of these checks:

- Maximum source-time tracking error at most 5 mm.
- Conservative maximum distance from the planned curve at most 2 mm.
- Final position error at most 0.5 mm.
- Measured speed at least 0.03 m/s throughout all complete corner intervals and
  the entire interior route, excluding only the initial and final speed ramps.
- Corner speed no more than 30% below the planned speed.
- No wheel-speed or wheel-acceleration command limiting, counted over every
  control update rather than only the downsampled report.
- Complete trajectory coverage, increasing source timestamps, and no feedback
  gap larger than 0.1 s, plus the existing full-vehicle support, passive-caster,
  finite-state and final platform-yaw checks.

These are sampled measurements, not mathematical bounds on all intermediate
instants. The measured speed can have small residual ripple despite the smooth
reference. The model still uses nominal physical parameters and ideal velocity
actuators; successful Gazebo tracking does not establish equivalent hardware
accuracy. No hardware commands or hardware configuration changes are involved.

## Recording quality

Continuous recording defaults to a real-time factor of **0.15**, allowing more
wall time for WSL software rendering while the MP4 retains normal simulation
speed. Other profiles retain their 0.5 default; `--real-time-factor` overrides it.
Missing images are represented by repeated frames so elapsed simulation time
is preserved. This does not establish that every encoded frame is fresh.

The recorder retains timestamps for fresh images actually written to FFmpeg.
The combined recorder rejects source-image gaps greater than **0.16 s** that
overlap motion, including missed messages and encoder-queue drops. Long gaps
wholly within startup or final holds are excluded. The final run's maximum
motion image gap is **0.160 s** and its
`video_quality` check passes. Full-file decoding, source-clock coverage, plan
identity, configuration hashes and actual decoded frames were checked.

## Tests and reproduction

All **195 pytest tests** pass in **ten CTest groups**; colcon reports 205 checks
including the wrappers. The planner contributes 66 tests, the analyzer 15, and
recording quality 11. Eight additional prediction regressions cover the timing
fix and preserve existing profile behavior. Coverage includes independent wheel
heading integration, derivatives, join continuity, acceleration/jerk bounds,
short segments, supported turn angles, clock mismatch, missing route sections,
and an interior stop that must fail. The recording-world regression checks that
the blue reference preserves all source-world elements and has no collisions.

- [Build and test output](validation/continuous_waypoint_build.log)
- [Implementation/configuration hashes](validation/continuous_waypoint_implementation.json)
- [Continuous planner](../scripts/continuous_waypoint.py)
- [Independent acceptance analyzer](../tools/analyze_continuous_tracking.py)
- [Simulation configuration](../config/continuous_waypoint_sim.yaml)

```bash
source "${HAMR_REPO}/hamr_ball_caster/scripts/env.sh"
ros2 run hamr_ball_caster record_waypoint_run.py --profile continuous --gui
```

The recorder starts an isolated simulation and writes the MP4, exact settings,
exported plan, raw trajectory, acceptance results, camera timing metadata and
reference-overlay world. White floor markings show the original straight route;
blue shows the rounded reference.
