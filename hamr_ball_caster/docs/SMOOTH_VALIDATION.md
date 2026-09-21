# Smooth waypoint implementation and validation

Commands in this guide assume a checkout of this repository. From any directory
inside that checkout, set these portable paths before running the examples:

```bash
export HAMR_REPO="$(git rev-parse --show-toplevel)"
export HAMR_RUNS="${HAMR_RUNS:-$HOME/Videos/hamr_sim}"
```

Recorded videos and full run reports are external artifacts, not files supplied
by a Git clone. Paths under `${HAMR_RUNS}` identify retained runs or new outputs
created by the recording commands.

The full vehicle with the two passive CAD ball casters now follows the original
waypoint geometry with planned stops. The final recorded run has **0.834 mm
maximum geometric deviation and full-leg corner overshoot**, **1.064 mm maximum
time-aligned tracking error**, and **0.265 mm RMS tracking error**. This is a
measured tolerance in the nominal Gazebo model, not a claim of mathematical zero
error or equivalent hardware accuracy.

Watch the complete MP4 (`${HAMR_RUNS}/smooth_waypoint_20260920_final/hamr_smooth_waypoint.mp4`)
or open the original-versus-corrected comparison (`${HAMR_RUNS}/smooth_waypoint_20260920_final/corner_tracking.png`).

## Implemented behavior

- The new pure-Python planner uses quintic position interpolation on each exact
  straight segment. Velocity and acceleration reach zero at every waypoint.
- Segment times satisfy conservative nominal wheel-speed and wheel-acceleration
  bounds over all chassis headings, with reserve for feedback correction.
- Each waypoint holds until measured position error is below 0.5 mm and measured
  speed below 3 mm/s continuously for 0.2 s. Failure to settle produces a failed
  run rather than letting the reference continue indefinitely.
- The simulation controller uses a nominal 100 Hz loop, feedforward velocity,
  source-time pose prediction, midpoint feedforward, and a validated position
  feedback gain of 16 s⁻¹. The Jacobian formula is unchanged.
- Wheel-speed and acceleration limits remain 6 rad/s and 15 rad/s². No selected
  validation run activated either limiter. The recorded run's maximum commanded
  wheel acceleration was 13.642 rad/s².
- Raw pose samples remain unchanged. Tracking metrics evaluate the reference at
  each pose's source timestamp. No pose teleportation, waypoint snapping, or
  artificial correction of measured positions is used.
- `smooth` is the default runner/recorder profile. `legacy` preserves the earlier
  abrupt schedule and its configuration for comparison.

The route remains 13 m with the same seven waypoint entries, including the
collinear intermediate return waypoint. Peak reference speed remains 0.25 m/s.
Planned motion lasts 97.5 s; measured settling increases the final recorded
motion to **98.731 s**, compared with the original **52 s** schedule. The complete
video lasts **105.72 s**, including startup and final holds.

## Controlled validation

| Run | Gain | Physics timestep | Maximum timed error | Maximum path deviation | Full-leg corner overshoot | Strict result |
|---|---:|---:|---:|---:|---:|---|
| Original recorded abrupt reference | 1.5 | 1 ms | 64.013 mm* | 50.499 mm | 50.499 mm | Fail |
| Smooth candidate | 4 | 1 ms | 3.367 mm | 3.019 mm | 2.784 mm | Fail |
| Smooth candidate | 8 | 1 ms | 2.025 mm | 1.591 mm | 1.553 mm | Fail |
| Smooth candidate | 8 | 0.5 ms | 2.030 mm | 1.593 mm | 1.583 mm | Fail |
| Selected, headless | 16 | 1 ms | 1.053 mm | 0.808 mm | 0.808 mm | Pass |
| Selected, finer timestep | 16 | 0.5 ms | 1.087 mm | 0.835 mm | 0.835 mm | Pass |
| Selected, fresh MP4 run | 16 | 1 ms | 1.064 mm | 0.834 mm | 0.834 mm | Pass |

The strict limits are 5 mm maximum timed tracking error, 2 mm geometric route
distance, 1 mm corner overshoot, and 0.5 mm final position error. All three
selected runs completed the full route and final hold. No wheel command clipping
occurred. Reducing the physics timestep changes the selected run's geometric
peak by only 0.027 mm; this is a useful consistency check rather than a complete
numerical convergence study.

Corner acceptance deliberately includes the **entire outgoing leg**, through
the next waypoint arrival, plus the approach window. A short two-second window
missed deviations occurring approximately three to four seconds after departure
with the slower plan. The analyzer was strengthened and earlier trials were
reanalyzed before selecting the final setting. Historical raw trial reports are
retained unchanged; the current version-2 reanalysis is authoritative.

\*The original recording compared the current-clock reference with the latest
available pose; the smooth reports align both to the pose source timestamp.
Geometric path distance and corner overshoot do not depend on this change and
provide the direct comparison: the maximum geometric deviation fell by about
**98.3%**, from 50.499 to 0.834 mm in the final recording.

The 50 Hz odometry samples establish sampled maxima, not a bound on every
continuous-time instant. These results use nominal model masses, contacts, and
ideal velocity actuators; hardware wheel-speed control and calibration remain
separate work. This task did not command or reconfigure the hardware.

## Evidence and reproduction

- [Run comparison and SHA-256 provenance](validation/smooth_waypoint_runs.json)
- [Implementation hashes and 94 passing pytest tests](validation/smooth_waypoint_implementation.json)
- [Build and all seven CTest-group results](validation/smooth_waypoint_build.log)
- Archived candidate and headless measurements (`${HAMR_RUNS}/smooth_validation_20260920`)
- Final recording, configuration snapshot, measurements, and plots (`${HAMR_RUNS}/smooth_waypoint_20260920_final`)
- [Planner module](../scripts/feasible_waypoint.py), [simulation runner](../scripts/run_waypoint_sim.py),
  [selected configuration](../config/smooth_waypoint_sim.yaml), and
  [accuracy analyzer](../tools/analyze_corner_tracking.py)

To repeat the complete recorded run:

```bash
source "${HAMR_REPO}/hamr_ball_caster/scripts/env.sh"
ros2 run hamr_ball_caster record_waypoint_run.py --profile smooth --gui
```

For a fresh headless check, choose a new or empty result directory:

```bash
ros2 run hamr_ball_caster evaluate_waypoint_profile.py --profile smooth \
  --output-dir ~/Videos/hamr_sim/smooth_headless_check
```

Add `--max-step-size 0.0005` to the headless command for the finer-timestep check.
The [step-by-step guide](SMOOTH_WAYPOINT.md) includes live Gazebo launch,
configuration overrides, analysis commands, and the retained legacy profile.
