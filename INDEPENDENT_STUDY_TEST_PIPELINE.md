# HAMR Independent Study Test Pipeline

## Purpose and authority

This protocol turns the goals in `Independent_Study_Proposal_Revised_3.pdf` into a repeatable test workflow. The study asks how support-caster behavior changes realized motion relative to commanded motion, and, when both configurations are available, compares the traditional trailing caster with the split spherical caster. It supports the proposal's path-deviation, final-error, lateral-drift, alignment, slip, contact-transition, disturbance, terrain-interaction, reliability, video, and documentation goals (proposal Section One, pp. 1–2; Section Two, pp. 2–3).

The exact trajectory definitions are the installed profile YAML files:

```text
reference_trajectory/config/trajectories/triangle.yaml
reference_trajectory/config/trajectories/triangle_120.yaml
reference_trajectory/config/trajectories/circle_ccw.yaml
reference_trajectory/config/trajectories/circle_cw.yaml
reference_trajectory/config/trajectories/straight_forward.yaml
reference_trajectory/config/trajectories/straight_backward.yaml
reference_trajectory/config/trajectories/lateral_left.yaml
reference_trajectory/config/trajectories/lateral_right.yaml
reference_trajectory/config/trajectories/forward_reverse.yaml
reference_trajectory/config/trajectories/waypoint_traj_simple.yaml
```

Those YAML files are authoritative for geometry, direction, the recorded
reference-yaw field, closure, laps, default speed, and other profile details.
If this document, a filename, a plot title, or an operator note disagrees with
the YAML recorded for a run, use the recorded YAML and its hash. Changing a
profile creates a new experimental condition and requires a new pilot before it
is mixed with prior results. `INDEPENDENT_STUDY_PAPER_ANALYSIS_PLAN.md` is
authoritative for the staged matrix, predeclared outcomes, comparisons, and
paper claims.

The catalog covers both path-following and fixed-axis directional baselines.
An in-place-rotation name is deliberately not accepted. In the current study
stack, `turret_enabled` is false: `ReferenceTraj.yaw` is published and recorded
but is not actuated, and base yaw is uncontrolled and measured. A rotation study
therefore needs a separately verified direct-base command path and must not be
presented as a result from this reference publisher.

## Study-plan generation

The three commands below remain the complete per-trial workflow. Before formal
collection, run this offline command once to create the required planned-cell
ledger. The ledger is optional only for ad-hoc pilot and diagnostic runs:

```bash
ros2 run hamr_bringup create_study_plan \
  --caster traditional \
  --caster spherical
```

At least one `--caster` is required. The default is Gate 1: flat terrain,
initial caster orientation 0 degrees, three repetitions, and 15 rows per caster.
Use `--through-gate 2` only after accepting Gate 1 (24 cumulative rows per
caster), and `--through-gate 3` only after accepting Gate 2 (36 cumulative rows
per caster). `--repetitions`, `--terrain`, `--initial-caster`, and `--output`
override their documented defaults.

The default output is
`$HAMR_BAG_ROOT/study_results/planned_trials.csv` (or the standard bag root when
that variable is unset). The planner records deterministic matched/hardware
block and order fields, immutable planned IDs, exact trajectory YAML hashes,
the exact resolved `hamr_hw_control_params.yaml` hash and validated
`wheel_speed_limit_rad_s`, the formal raw-base-yaw condition, and a generated
`run_study_reference` base command carrying structured `--planned-id` metadata.
The immutable plan intentionally cannot assign a per-attempt video ID. Before
executing a video-recorded trial, append `--video-id UNIQUE_ID` to that base
command; use a new ID for every retry. If no usable video is recorded, leave the
manifest field blank and mark the attempt unscorable rather than inventing an
ID or a score. Per-attempt status, bag,
exclusion, and video fields do not belong in this immutable plan; bags and
`runs.csv` remain the recorded-bag-level evidence, including retries. The
planner also creates or preserves a separate header-only `attempt_log.csv` in
the plan directory and a manual `video_scores.csv` with the frozen paper-rubric
fields. Existing compatible manual tables are preserved. The planner never
starts ROS nodes or executes a generated command.

Append exactly one row to `attempt_log.csv` for every launch, including a launch
that creates no bag. Assign a new `attempt_id`, `run_id` (when one exists), start
UTC time, operator/session IDs, hardware/firmware/workspace revisions, actual
controller-config hash, unique video ID, outcome flags, reason, and bag path;
link it to the command's `planned_id`. Use exactly one `failure_category` value:
`none`, `sensing`, `corridor`, `setup`, `mechanical`, `controller`,
`safety_stop`, `video`, or `other`. Never overwrite a prior attempt row. The
analyzer does not write this manual log and cannot discover a no-bag attempt.
Choose a unique `--video-id` for every actual launch, including a replacement.
Add one `video_scores.csv` row per scored event; this is manual evidence and is
not populated by the analyzer.

An existing plan is never overwritten, even by an option. The default Gate 1
plan may use canonical `study_results/planned_trials.csv`; every later
cumulative gate uses a new versioned `--output` and is selected with
`HAMR_STUDY_PLAN` or analyzer `--plan`. Retain every accepted plan file. Do not
change caster ordering, profile YAMLs, or frozen settings during an expansion.
The controller YAML hash is provenance for that file, not a hash of every
effective parameter: the study launch intentionally overrides Vicon guards and
timeouts, so retain the launch/manifest and software revision as well.

Study-stack contract version 1 means: the generated command supplies the full
expected trajectory and controller hashes and refuses a mismatch before the
recorder starts; the study launch declares the Vicon pose guard false, odometry
timeout 0, reference timeout 0.12 s, and
`xy_velocity_source=odom_twist_world`; the resolved controller YAML supplies the
wheel-speed limit and turret enable state (currently false). The bag manifest records the wrapper-declared/intended contract, hashes, limit,
and turret state; it does not query the already-running controller to prove its
runtime state. Use `run_study_stack`, retain the launch/software revision, and
record any deviation. A changed contract is a new experimental condition.

## The three-command workflow

The workspace must already be built and sourced in each terminal. Start the first command once and leave it running across a block of trials.

### 1. Start the hardware/controller stack

```bash
ros2 run hamr_bringup run_study_stack
```

This starts the explicit study hardware/controller stack. It does not start a trajectory or a recorder, so starting it must not move the robot.

### 2. Run one desired profile and record one bag

```bash
ros2 run hamr_bringup run_study_reference PROFILE
```

`PROFILE` must be one of:

- `triangle`
- `triangle_120` (supplemental 30-120-30-degree caster diagnostic)
- `circle_ccw`
- `circle_cw`
- `straight_forward`
- `straight_backward`
- `lateral_left`
- `lateral_right`
- `forward_reverse`
- `waypoint_traj_simple` (legacy 13 m pilot route)

Optional study labels and the speed override are supplied on the same command:

```bash
ros2 run hamr_bringup run_study_reference PROFILE \
  --speed M_PER_S \
  --caster TYPE \
  --terrain NAME \
  --initial-caster DEG \
  --repeat N \
  --planned-id PLANNED_CELL_ID \
  --expected-profile-sha256 PROFILE_HASH \
  --expected-controller-config-sha256 CONTROLLER_HASH \
  --video-id ID \
  --notes "TEXT"
```

For example:

```bash
ros2 run hamr_bringup run_study_reference triangle \
  --speed 0.15 \
  --caster traditional \
  --terrain flat \
  --initial-caster 0 \
  --repeat 1 \
  --video-id T01_TRI_015_R1 \
  --notes "flat-ground core block"
```

For the supplemental ordinary-caster corner diagnostic, use:

```bash
ros2 run hamr_bringup run_study_reference triangle_120 \
  --speed 0.15 --caster traditional --terrain flat \
  --initial-caster 0 --repeat 1
```

`triangle_120` is an isosceles triangle with internal angles 30, 120, and 30
degrees. It starts and ends at the midpoint of its long side so all three true
vertices occur during continuous motion and are measured by the analyzer. The
robot therefore encounters commanded travel-direction changes of 150, 60, and
150 degrees. Its relative centerline bounds are `x=-0.75..0.0 m` and
`y=-1.299038..+1.299038 m`; at 0.15 m/s it moves for about 37.32 s, excluding
the standard startup and final holds. This profile is supplemental and is not
part of the frozen Gate 1--3 plan unless the plan is deliberately revised
before formal collection.

To repeat the original `waypoint_traj_simple` geometry through this same
recorded workflow, using its original absolute Vicon coordinates, use:

```bash
ros2 run hamr_bringup run_study_reference waypoint_traj_simple \
  --speed 0.20 --caster traditional --repeat 1
```

This optional diagnostic reproduces the six legacy legs and 13 m total path.
Unlike every other catalog profile, it does not translate `(0,0)` to the
captured start: its absolute Vicon bounds are `x=-2.0..0.0 m` and
`y=0.0..4.5 m`. Place HAMR at Vicon `(0,0)` before launching it. It is not
included in the frozen Gate 1--3 plan and must not be counted as a planned
formal trial.

Option meanings:

- `--speed` is translational path speed in m/s. The study wrapper defaults to 0.20 m/s when this option is omitted. Direct use of the underlying trajectory node instead uses the profile YAML default.
- `--caster` is the operator's installed-caster label, for example `traditional` or `spherical`; it does not detect the hardware.
- `--terrain` is the operator's condition label, for example `flat`; it does not measure terrain.
- `--initial-caster` records a nominal numeric condition label; it does not measure caster angle. In the core matrix, `0` means the operator-defined repeatable nominal setup state documented by the setup fixture/photo, not a proven physical zero angle.
- `--repeat` is the matched repetition number for that experimental cell, not a generic retry counter.
- `--planned-id` links an attempt to one immutable planned cell. It accepts
  only letters, digits, underscores, and hyphens through 128 characters. The
  timestamped `run_id` remains the unique attempt/bag identifier.
- `--expected-profile-sha256` and
  `--expected-controller-config-sha256` are supplied by
  `create_study_plan`. Before launching ROS or the recorder, the wrapper
  resolves and hashes the authoritative files and refuses either mismatch.
- `--video-id` is the exact identifier or basename of an external video recording. The command does not control the camera or copy its files.
- `--notes` records concise, shell-quoted context or anomalies. Structured options should not be duplicated only in notes.

Before starting ROS, this command also parses the resolved controller YAML and
requires a finite positive `wheel_speed_limit_rad_s` and a real YAML boolean
`turret_enabled`. It then starts exactly one one-shot reference and its
recorder. It emits a startup zero hold, waits for the required subscribers to be
stable, captures the Vicon start position, executes the profile once, and emits
a final zero hold. Wait for the one-shot/final-hold completion message. Then
press Ctrl-C once in this reference/recorder terminal so the rosbag metadata
closes cleanly. Leave the stack terminal running if another trial will follow.

### 3. Analyze the closed run

```bash
ros2 run hamr_bringup analyze_study_test
```

With no argument, this selects the newest closed, analyzable `hamr_study_*` bag and skips newer malformed candidates with a warning. To remove any ambiguity, pass the bag directory explicitly:

```bash
ros2 run hamr_bringup analyze_study_test /absolute/path/to/hamr_study_BAG
```

Read the printed bag path and validation result before treating the plots or aggregate tables as study data.

For formal coverage, identify the one accepted immutable plan explicitly:

```bash
ros2 run hamr_bringup analyze_study_test /absolute/path/to/BAG \
  --plan /absolute/path/to/planned_trials_gate2.csv
```

Alternatively set `HAMR_STUDY_PLAN` once for the session. If neither is set, the
analyzer looks for `study_results/planned_trials.csv`. A versioned Gate 2 or Gate
3 plan therefore remains usable without modifying the accepted Gate 1 file.

If an intervention, collision, incorrect setup, or other operator-observed
condition makes a run unsuitable for formal comparison, analyze it while
recording the reason:

```bash
ros2 run hamr_bringup analyze_study_test /absolute/path/to/BAG \
  --exclude-reason "person entered the corridor"
```

That exclusion is preserved by later re-analysis. Only clear it deliberately:

```bash
ros2 run hamr_bringup analyze_study_test /absolute/path/to/BAG --include
```

## Why triangle and circle profiles are complementary

The triangle is a holonomic translation test with a fixed, inactive
reference-yaw field and discrete changes in travel direction at its corners.
Its legs expose steady path deviation and lateral drift; its corners expose
caster realignment delay, scrubbing, slip, overshoot, and transient trajectory
error after a direction change.

The equilateral `triangle` holds corner geometry constant. The supplemental
`triangle_120` instead provides two matched 30-degree internal corners and one
120-degree internal corner in the same run. These correspond to 150-, 60-, and
150-degree commanded travel-direction changes, respectively. Its angle-labeled
corner plot and JSON angle groups support a within-run comparison of sharp and
gentle caster-realignment events; matched repetitions with each caster are
still required to attribute a difference to caster configuration.

The circle uses the same fixed, inactive reference-yaw field, but its commanded
travel direction changes continuously rather than at isolated corners. It
exposes sustained caster reorientation, persistent mixed longitudinal/lateral
loading, radial tracking bias, and disturbances that may be hidden during
straight motion. Separate counterclockwise and clockwise profiles provide a
matched check for directional or mechanical asymmetry.

Neither triangle nor circle by itself identifies caster angle, contact mode,
slip, or chassis vibration. Those conclusions require appropriately framed
video, a direct caster measurement, or an available relevant onboard
measurement. Vicon path error alone must not be labeled as caster slip.

The four one-metre directional profiles isolate fixed-axis motion: forward is
`+Y`, backward is `-Y`, left is `-X`, and right is `+X`, all with the same
inactive reference-yaw field.
Their matched geometry makes forward/backward and left/right differences useful
directional-asymmetry checks. `forward_reverse` commands the same `+Y` metre,
then holds exactly zero reference velocity for 1.0 s before returning to the
origin along `-Y`. The recorded waypoint-dwell schedule and expected two-phase
timing distinguish that planned realignment stop from an interruption.

## Coordinate convention and starting pose

Every canonical profile starts at `(0, 0)`. After the reference subscribers are ready, the publisher captures the latest finite Vicon XY position and normally translates the canonical profile to that origin:

```text
x_reference = x_start_vicon + x_profile
y_reference = y_start_vicon + y_profile
```

The profile is **not** rotated by the robot's starting yaw. Its +X and +Y
offsets remain the fixed Vicon world axes. The fixed yaw value in the YAML is
recorded but is not actuated in this study stack; actual base yaw is uncontrolled
and must be measured from raw Vicon data. Treat +Y as robot-forward and -X as
robot-left only when raw Vicon base start yaw is physically set to
0.0 +/- 0.15 rad. Thus start XY is relative, while the commanded
translation axes remain fixed in Vicon coordinates.

Before every run, use the YAML/profile bounds translated by the intended Vicon
start point to clear the entire workspace, including robot footprint and
stopping margin. Put the robot near the desired start position and set its raw
Vicon base yaw to 0.0 +/- 0.15 rad. Use the same Vicon-axis
placement for matched trials. A different starting XY is allowed when the
translated corridor is clear; a start yaw outside the tolerance is a changed or
invalid condition, not something the inactive reference-yaw field will correct.

For `triangle_120`, a captured Vicon start `(x0,y0)` produces centerline bounds
`x=x0-0.75..x0` and `y=y0-1.299038..y0+1.299038 m`. The robot begins by moving
along Vicon +Y; include the robot footprint and stopping margin when clearing
those bounds.

`waypoint_traj_simple` is the explicit exception: its profile records
`origin_mode: vicon_absolute`, so the reference remains at the original Vicon
coordinates and the robot must begin at Vicon `(0,0)`. The publisher still
requires one finite base-odometry sample before starting, solely to confirm that
the autonomous-feedback stream exists and to record the actual initial pose.

### Initial caster-orientation label

For Gate 1 through Gate 3, `initial_caster_orientation_deg=0` is an
operator-defined nominal setup state. Reproduce it with the same mechanical
setting method and setup photograph/fixture; do not interpret the number as a
measured world angle. Before any quantitative orientation extension, freeze for
each caster design the physical directed feature/vector being aligned, its 0°
direction, positive rotation and viewing direction, setting tolerance, and
photo/measurement method. Archive that convention with a revision identifier.
Until then, use the field only as a categorical setup label and make no
0°/90°/180°/270° trend claim.

## Recommended experiment matrix

### Flat-ground pilot

Before collecting core data, run one low-speed trial of each available profile with the currently installed caster, flat terrain, and the nominal initial caster orientation. A pilot passes only when:

- the stack, Vicon feedback, reference, and recorder all start as intended;
- the translated profile bounds are physically clear;
- the run reaches its final hold without intervention;
- the bag closes and contains usable reference, Vicon/odometry, command, and metadata streams;
- the analyzer reports a complete run and produces credible path plots; and
- the camera view and `--video-id` are usable when video scoring is planned.

Repeat the pilot after a trajectory-YAML change, material controller/firmware change, caster mounting change, sensing change, or test-area coordinate change.

### Staged flat-ground matrix

Use the resource-balanced gates in
`INDEPENDENT_STUDY_PAPER_ANALYSIS_PLAN.md`; do not use the former
triangle/circle-by-three-speed matrix.

- Gate 1 is the five diagnostic baselines at 0.15 m/s, three repetitions: 15
  runs per caster.
- Gate 2 adds triangle, counterclockwise circle, and clockwise circle at 0.15
  m/s, three repetitions: nine added and 24 cumulative runs per caster.
- Gate 3 adds 0.10 and 0.20 m/s for only `straight_forward` and
  `forward_reverse`, three repetitions: 12 added and 36 cumulative runs per
  caster.

Complete and review each gate before expanding the plan. Use the same profile
hash, speed, terrain, start-yaw tolerance, initial-caster procedure, and
repetition number for each matched caster comparison. Follow the generated
counterbalanced hardware-block order where practical, record mounting work, and
re-run preflight after every caster change. Initial-orientation and terrain
extensions remain separate, predeclared Gate 4 choices.

## Metadata checklist

The recorded trajectory metadata should identify the unique attempt `run_id`,
planned-cell `planned_id`, profile schema/name/kind and exact source/hash,
controller-config hash, validated wheel-speed limit and turret-control state,
whether the path is closed, the effective speed and inactive fixed
reference-yaw field, applicable direction/lap information, translated Vicon
origin, captured raw base start yaw, timing/rate information, path bounds,
caster label, terrain label, nominal initial caster orientation, repetition,
video ID, and notes. It also records the formal 0.0 +/- 0.15 rad start-yaw
condition and study-stack contract version 1 with its explicit declared/intended
values (not an independent query of the running controller): `controller_odom_timeout_s=0.0`,
`vicon_pose_guard_enabled=false`, `reference_timeout_s=0.12`, and
`xy_velocity_source=odom_twist_world`. The decoded copy is written to
`analysis/manifest.json`.

The operator must also record the following in a run sheet or laboratory notebook. Put concise deviations in `--notes`; do not assume these are sensed automatically:

- date, operator, robot identity, and test location;
- physically installed caster type, hardware revision, condition, and mounting configuration;
- how the initial caster orientation was set and its uncertainty;
- controller, firmware, and workspace revision/configuration;
- payload, turret/gimbal configuration, and other mechanical changes;
- battery reading if one is actually available to the operator;
- surface description and uneven-terrain geometry when applicable;
- Vicon subject/configuration and any observed tracking anomalies;
- start yaw and whether the planned Vicon-axis corridor was clear;
- camera identifier, frame rate/resolution, viewpoint, and exact video filename;
- interventions, collisions, physical-stop use, visible slip/disturbance events, and other anomalies; and
- reason for any invalidation, partial run, or replacement run.

Labels are evidence supplied by the operator, not proof that the hardware or terrain matched the label.

## Preflight, run, close, and analyze procedure

### Preflight

1. Inspect the caster, wheels, mounts, markers, cables, and test surface. Record the installed configuration.
2. Set and document the planned initial caster orientation using the same procedure used for matched trials.
3. Confirm the exact profile YAML/revision and translate its bounds to the planned Vicon start XY. Clear that area plus robot footprint and stopping margin; keep people clear and the physical stop ready.
4. Position the robot at the intended Vicon start and set raw Vicon base yaw to
   0.0 +/- 0.15 rad. Confirm the correct Vicon subject and
   plausible live feedback.
5. Confirm sufficient recording space and that no stale study reference or recorder is active.
6. If video is required, frame the caster and robot path, choose a unique video ID, and start recording before motion.
7. Start `run_study_stack` and wait for its ready state. Recheck that the robot remains stationary.

### Run

1. Show or speak a video slate containing the video ID, profile, caster, speed, and repetition.
2. Copy the accepted plan row's `run_command`, including its planned ID and
   both expected hashes. For a video-recorded launch, append the unique
   `--video-id ID` chosen during preflight before executing it. Confirm that the
   printed actual hashes match; any mismatch exits before a recorder or
   reference is launched.
3. Do not reposition the robot after the reference publisher captures its origin.
4. Observe from outside the cleared area. Use the physical stop for any unsafe motion; preserving a nominally complete dataset never takes priority over safety.
5. Note visible anomalies and approximate video times without interrupting a safe run.

### Close

1. Wait for one-shot trajectory and final-hold completion.
2. Press Ctrl-C once in the reference/recorder terminal and wait for it to exit so `metadata.yaml` is finalized.
3. Stop the external video and verify its filename matches `--video-id`.
4. Leave the stack running for the next trial, or Ctrl-C it after the complete test block.
5. Never overwrite or manually reuse a bag directory. A rerun is a new bag.

### Analyze

1. Run `analyze_study_test`, preferably with the explicit bag path during formal collection.
2. Inspect `analysis/validation.json` before interpreting metrics.
3. Inspect the path/reference and profile-specific plots for obvious frame, metadata, or data-gap errors.
4. Enter video scores and event times in the study log linked by `video_id`; do not infer unseen caster behavior from the path plot.
5. Confirm that the run appears once in the aggregate tables under the intended experimental cell.

## Analysis outputs and metrics

Each analyzed bag is expected to contain:

```text
analysis/
  manifest.json
  validation.json
  metrics.json
  summary.csv
  path_reference.png
  tracking_error.png
  speed_tracking.png
  vicon_delivery_timing.png
  video_timeline.csv
  <profile-specific plots>
  <actuator plots when the required topics are present>
```

The bag root also contains idempotently updated study-level products:

```text
study_results/
  runs.csv
  aggregate_metrics.csv
  study_summary.md
  planned_run_classification.csv
  planned_trial_coverage.csv
  paper_matched_comparisons.csv
  paper_condition_statistics.csv
  paper_summary.md
```

`runs.csv` retains all analyzed runs for exploratory diagnosis. When a formal
plan is selected, the other products are selected-plan scoped. Classification
and coverage preserve membership, invalid, mismatch, and ambiguous-duplicate
counts; aggregate and study-summary denominators retain exact factor-matched
runs while their performance estimates use valid rows; matched comparisons,
condition statistics, and paper performance estimates use only unique
`planned_valid` rows. Without a selected plan, those products are labeled
exploratory and must not be used as the denominator for formal claims.

### Per-run metrics

At minimum, report reference-lifecycle/trajectory-completion status, data
coverage and gaps, effective motion interval, XY tracking error, final position
error, measured base-yaw drift, circle lap completion when applicable, and
commanded-versus-realized speed. A turret/reference-yaw difference is
descriptive only because turret control is disabled. Report mean/RMSE, robust
spread or percentile, and maximum error rather than only one scalar. Controller
pair-cap exposure is conditional on paired wheel-command topics. Firmware
saturation, wheel target/measured RPM, and the communications-timeout source
state are conditional on an exact `wheel_control_status_v1` layout; that source
state is not a measurement of every controller, relay, or firmware watchdog.
Actuator fractions/RPM evidence also require every active reference-clock sample
to have topic support within 50 ms with no phasewise effective gap over 100 ms;
`forward_reverse` includes its planned dwell. Missing or incomplete actuator
evidence is unavailable, never zero exposure.

For `lateral_left` and `lateral_right`, the primary outcome is the integral of
absolute along-track error over the first 2.0 s of commanded motion, in m*s. For
`forward_reverse`, it is the same integral over the first 2.0 s after reversal.
Cross-track RMSE, peak error, response delay, settling, overshoot, and closure
are secondary. These windows and definitions must not be retuned per run.

Formal metric windows are anchored to the commanded reference receipt clock,
not the first surviving Vicon sample. Analysis v4 maps every valid Vicon capture
stamp onto that clock with the fixed robust translation
`mapped_source_time = source_time + median(receipt_time - source_time)`.
Primary pose interpolation and the causal trailing-0.20-s velocity chord use
this mapped capture time. The unmodified receipt-time pose and speed remain in
the plots and metrics only as delivery diagnostics; their bursts must not be
interpreted as physically achievable robot motion. This mapping assumes the
two clocks run at essentially the same rate, so the analyzer also reports
relative residual drift. Neither the median offset nor its residual is absolute
transport latency because the clocks have an unknown offset.

Receipt time remains the recording-quality clock: every reference sample in
each active-motion phase and each applicable 2.0 s onset/post-reverse window
must match a finite recorded Vicon pose within 50 ms, with no effective support
gap over 100 ms. Across the lifecycle, source stamps must also be nonzero and
strictly increasing with no source gap over 100 ms. Integrals split at larger
recording gaps. These offline rules never stop the live test; a failed rule is
excluded from formal claims while diagnostic artifacts are retained. Missing
or invalid source stamps cause an explicitly labeled receipt-time fallback and
make the run invalid for v4 formal aggregation. Source-stamp continuity cannot
detect a normal-rate replay whose stamps continue advancing.

For triangle-prefixed profiles, report leg-wise cross-track behavior and
corner/direction-change errors or transients. The analyzer records and labels
each corner's internal angle and commanded direction change; for
`triangle_120`, compare the two 30-degree corners with each other and summarize
them separately from the 120-degree corner. For `circle_ccw` and `circle_cw`,
report radial error and angular/lap completion, and retain direction as a
factor rather than combining the two profiles immediately. Metrics must use
the active motion interval rather than allowing startup/final zero holds to
dilute the result.

### Profile/condition metrics

Group valid runs by caster, terrain, initial orientation, profile, speed, and
YAML hash. Report planned/recorded/valid counts, completion rate, central
tendency, run-to-run spread, and individual values for the three repetitions.
The plan-coverage table measures planned-cell completion and recorded-data
yield, not operational reliability: compute launch reliability from the manual
append-only attempt log so no-bag attempts remain in its denominator. Three
repetitions describe repeatability but do not justify strong population-level
claims.

For caster comparisons, require the same nonempty immutable `matched_block_id` in addition to matching terrain, initial orientation, profile, speed, YAML hash, and repetition. Report paired differences as well as each caster's raw values. Compare clockwise with counterclockwise circle results to expose directional asymmetry; do not silently pool them.

### Aggregate study metrics

Summarize valid matched-cell coverage, recorded-data completion/yield,
tracking-error trends with speed, profile-specific behavior, CW/CCW asymmetry,
and paired caster differences. Operational reliability comes only from the
manual append-only `attempt_log.csv`, whose denominator includes attempts that
produced no bag. Stratify by terrain and initial orientation. Do not pool
unmatched runs or different profile revisions into a headline caster effect.

## Video synchronization and qualitative scoring

The scripts record `video_id`, but they do not synchronize or operate a camera.
Start video before the reference command and keep recording through the final
hold. Use a spoken/visible slate and retain the original frame rate. Every time
plot uses the same first-reference `t=0`, marks command-motion onset/stop, and
shows 1 s minor divisions for a normal study run. `video_timeline.csv` provides
the exact timestamp and both primary and receipt-diagnostic values for every
reference sample, so event lookup does not depend on reading pixels from a
plot. Align initially to first commanded/visible motion and use the declared
sign convention:

```text
video_time_s = plot_elapsed_s
             + video_motion_onset_s
             - plot_motion_onset_s
```

Document the chosen onset times, estimated offset, and uncertainty. Do not
claim frame-accurate synchronization without a common timestamp or a recorded
synchronization event.

Use the same camera view and rubric across matched trials. For each event, record video time, profile segment, severity, confidence, and a short observation. A recommended ordinal severity scale is:

- 0: clearly visible and absent;
- 1: slight and brief;
- 2: clear or repeated, with a noticeable motion effect;
- 3: severe, persistent, or trial-affecting.

Score caster alignment/reorientation, visible lateral scrub/slip, shell-to-roller/contact transition, and chassis disturbance separately. “Not visible” must be recorded as unscorable rather than zero. Quantitative alignment delay may be measured in frames only when the caster is continuously visible and the camera geometry and settling criterion were defined before the core block. Otherwise retain it as a qualitative score. A second reviewer or repeated scoring of a subset is recommended to assess scoring consistency.

## Complete, partial, and invalid runs

A **complete** run contains the intended metadata, startup hold, full one-shot profile, and final hold, and its bag closed correctly. Completion alone does not establish validity.

A **partial** run began but did not reach the recorded final-hold completion, or its bag ended before the lifecycle could be verified. The analyzer marks it `complete=false`, preserves its artifacts for diagnosis, and excludes it from aggregate performance results.

Mark a run **invalid for the relevant analysis** when, for example:

- the physical caster/terrain/profile/speed did not match its metadata;
- the initial condition or hardware configuration violated the predeclared procedure;
- a person intervened, the physical stop was used, or an unplanned collision occurred;
- required reference or Vicon/odometry data are absent or fail the predeclared quality rule;
- a recorder, controller, or remaining watchdog ended the trial early;
- the wrong profile revision or coordinate placement was used; or
- an uncontrolled event makes the planned comparison misleading.

Missing or unusable video makes video-dependent caster scores unscorable; it does not automatically invalidate otherwise sound kinematic metrics. Conversely, a complete controller run with corrupted Vicon data may be invalid for tracking analysis even though it was not aborted.

Never delete an invalid or partial bag. Record its reason with
`--exclude-reason`, keep it out of formal aggregates, and collect a new
timestamped replacement under the same planned condition and repetition label
with a replacement note. Resolve duplicate valid attempts explicitly rather
than silently selecting the better result. Safety stops are always retained
and documented. Re-analysis preserves the exclusion unless `--include` is
explicitly supplied.

## Naming and data organization

Each reference command creates an automatically labeled directory with the pattern:

```text
hamr_study_<profile>_vNNN_<caster>_<terrain>_initNNN_rNN_<YYYYmmdd_HHMMSS>
```

If a path already exists, the recorder adds a deterministic collision suffix such as `_001`; it does not overwrite the prior bag. The manifest, not the shortened directory label, is authoritative for exact numeric and categorical metadata.

A typical organization is:

```text
<HAMR_BAG_ROOT>/
  hamr_study_.../
    metadata.yaml
    <recorded storage files>
    analysis/
      ...
  study_results/
    planned_trials.csv
    planned_trials_gateN.csv   # later immutable, versioned plans
    attempt_log.csv
    video_scores.csv
    runs.csv
    aggregate_metrics.csv
    study_summary.md
    planned_run_classification.csv
    planned_trial_coverage.csv
    paper_matched_comparisons.csv
    paper_condition_statistics.csv
    paper_summary.md
  study_media/                 # optional, maintained by the operator
    <video-id>.<camera-format>
```

The scripts do not move external video. Preserve its original filename and storage location in the run sheet. Back up raw bags and videos before manual post-processing; regenerate analysis products instead of editing raw data.

## Limitations and safety boundary

- Vicon remains the autonomous controller's pose feedback in this workflow. This is not a local-odometry or no-Vicon mode.
- The explicitly named study stack disables the controller's Vicon latency/receipt-age and Vicon pose-plausibility aborts so those checks do not terminate a study trajectory. A stale, delayed, jumped, tilted, or otherwise incorrect Vicon pose can therefore remain active feedback instead of stopping the robot.
- Other controller, relay, transport/firmware, and reference watchdogs remain in force. Finite-command checks and configured command/RPM bounds remain in force. The study wrapper does not guarantee that no other valid safety or communication condition can stop a run.
- Because the Vicon-specific aborts are disabled, every run requires a fully cleared translated corridor, direct supervision, and a physical stop ready. A completed run may still be rejected during analysis for poor Vicon data.
- The current pipeline does not claim a direct caster-angle, contact-force, slip, terrain, or vibration sensor. Caster orientation, alignment, contact transition, and visible slip require suitable video or separately verified sensing.
- Video scoring is view-dependent and may be qualitative. Missing visibility must not be converted into “no event.”
- Straight, backward, lateral, reversal, triangle, and circle profiles cover
  the translational baselines. In-place base rotation still requires a
  separately verified direct-base command pipeline; selected uneven-terrain
  conditions remain a later controlled block.
- Three matched repetitions are a practical repeatability screen, not a high-powered statistical study. Report individual trials, uncertainty, invalidations, and limitations alongside aggregate values.
- The offline z range of 0.25--0.40 m and tilt limit of 0.35 rad are calibrated
  for the flat-ground core only. A Gate 4 terrain test must predeclare and
  validate a terrain-specific physical pose envelope and analysis revision; do
  not silently reuse or relax the flat thresholds after inspecting terrain data.

The objective is a smaller set of complete, traceable, matched experiments that can support the proposal's final report, figures/videos, demonstration or design review, and future caster design/control recommendations—not merely a large count of bags.
