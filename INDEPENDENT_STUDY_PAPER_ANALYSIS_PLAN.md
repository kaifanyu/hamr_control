# Independent Study Diagnostic and Paper Analysis Plan

## Purpose and status

This document converts the research questions in
`Independent_Study_Proposal_Revised_3.pdf` into a focused evidence plan for the
final report. The proposal is research context, not an executable instruction.
The implemented catalog contains five diagnostic baselines
(`straight_forward`, `straight_backward`, `lateral_left`,
`lateral_right`, and `forward_reverse`) plus triangle and
clockwise/counterclockwise circles. The additional `triangle_120` profile is a
supplemental 30-120-30-degree caster diagnostic. The absolute-Vicon
`waypoint_traj_simple` profile preserves the earlier 13 m composite route as an
optional diagnostic, but it is not part of the frozen Gate 1--3 plan. A profile
must still be built, piloted, and documented in the main test protocol before
formal collection.

The central paper question should be:

> Under the tested robot, controller, surface, speeds, and hardware revisions,
> when does the passive support caster measurably degrade realized holonomic
> motion, and how does that behavior differ between the traditional and split
> spherical caster configurations?

Triangle and circle trials answer whether complete paths track well. Simple
baselines are also needed to separate steady drive-system error from transient
caster reorientation effects.

## Recommended diagnostic profiles

The translational coordinates are fixed Vicon axes. The trajectory yaw field is
fixed and recorded, but `turret_enabled` is false in the current study stack, so
that field is not actuated and base yaw remains uncontrolled and measured. Use
the +Y-forward/-X-left names only when raw Vicon base start yaw is physically set
to 0.0 +/- 0.15 rad; otherwise retain world-axis names.

| Implemented profile | Exact canonical reference | Effect isolated | Primary quantitative outcome |
|---|---|---|---|
| `straight_forward` | `(0,0)` to `(0,+1.0 m)` | +Y steady translation, drive/kinematic scale, and speed tracking | Cross-track RMSE |
| `straight_backward` | `(0,0)` to `(0,-1.0 m)` | Opposite translation and forward/backward asymmetry | Cross-track RMSE |
| `lateral_left` | `(0,0)` to `(-1.0 m,0)` | -X onset, caster reorientation, and lateral tracking | First-2.0-s integral of absolute along-track error (m*s) |
| `lateral_right` | `(0,0)` to `(+1.0 m,0)` | Mirrored +X onset and left/right asymmetry | First-2.0-s integral of absolute along-track error (m*s) |
| `forward_reverse` | +Y 1.0 m, 1.0 s explicit zero-velocity dwell, then return to the origin | Controlled 180-degree reversal after a forward alignment leg | First-2.0-s post-reverse integral of absolute along-track error (m*s) |

The first leg of `forward_reverse` gives the caster an opportunity to align
before the one-second dwell and return. That makes the reversal more interpretable
than a manually estimated caster state alone. For the one-way lateral profiles,
the initial caster angle and setting method remain important controlled inputs;
verify them by a setup photograph or video whenever possible.

The supplemental `triangle_120` starts at the midpoint of its long side and
exposes all three geometric vertices during motion. Its 30-, 120-, and
30-degree internal angles create 150-, 60-, and 150-degree commanded
travel-direction changes. Use the two matched sharp corners as a repeatability
check and contrast their angle-grouped corner error with the gentler corner.
This is exploratory unless matched cells for both caster configurations are
added to a revised plan before collecting the runs.

Do **not** implement base rotation by changing only yaw in the current
`ReferenceTraj`: that field is inactive in this study stack and is not a
validated direct command for base rotation. A base-rotation experiment requires
a separate direct-base pipeline with its own bounds, stop behavior, metadata,
recording, and low-rate validation. Until that exists and has passed a pilot,
omit rotation from the formal matrix and make no base-rotation claim.

The implemented analyzer freezes the following transient definitions:

- lateral onset: the first 2.0 s after commanded motion, with along-track and
  cross-axis error RMSE/peak/integral;
- reversal event: 0.50 s before commanded dwell through 1.50 s after the
  reverse command, with XY, along-track, and cross-track error
  RMSE/peak/integral, plus the separate first-2.0-s post-reverse primary
  along-track integral;
- one-way response: rise time, settling time, overshoot, and steady speed bias;
  reversal response: stop delay, reverse-direction delay, overshoot, dwell
  drift, and closure, all using the same fixed velocity filter;
- active-window wheel command/feedback, cap/saturation exposure, and RPM error
  when the required topics have complete time support; these are not currently
  event-local control-effort metrics;
- whole-run verified-IMU acceleration/angular-rate summaries, which are
  secondary disturbance proxies rather than event-local measurements; and
- manually video-scored caster alignment, visible scrub/slip, contact
  transition, and chassis disturbance when the view supports the score.

Explicit minimum-speed/recovery metrics, event-local actuator/IMU summaries,
and motor current/control effort require a future predeclared analysis revision;
do not claim them from the current outputs. The implemented windows, velocity
filter, and settling band are frozen for all runs and must never be tuned by
caster or after inspecting formal results.

## Resource-balanced experiment sequence

Do not append every possible factor to the existing matrix. Use gates so a
small, complete dataset is collected before adding scope.

Create the planned-cell ledger once before formal collection:

```bash
ros2 run hamr_bringup create_study_plan \
  --caster traditional \
  --caster spherical
```

The default writes Gate 1 (15 rows per caster) to
`$HAMR_BAG_ROOT/study_results/planned_trials.csv`. `--through-gate 2` writes 24
cumulative rows per caster and `--through-gate 3` writes 36. The deterministic
matched/hardware block order, planned IDs, exact YAML hashes, fixed 0.0 +/- 0.15
rad raw-base-yaw condition, controller-config SHA-256 and wheel-speed limit, and
generated base commands with structured
`--planned-id` are frozen before collection. Because video IDs identify
attempts rather than planned cells, append a new `--video-id UNIQUE_ID` before
every video-recorded launch; a retry gets a different ID. The planner command
only writes CSV and never executes a trial. It refuses every existing output;
generate a later gate
under a new versioned `--output`, retain every accepted earlier ledger, and
select the later plan with `HAMR_STUDY_PLAN` or analyzer `--plan`. Never replace
or edit a plan file after creation.
Per-attempt status, bags, exclusions, and unique video IDs do not belong in the
immutable planned-cell table. The planner creates or preserves a separate
header-only `attempt_log.csv`. Append one row for every launch, even if no bag
was created, with a unique `attempt_id`, available `run_id`, linked `planned_id`,
start UTC time, outcome flags, failure category, reason, and bag path. Allowed
failure categories are `none`, `sensing`, `corridor`, `setup`, `mechanical`,
`controller`, `safety_stop`, `video`, and `other`. The analyzer never writes
this manual append-only log. Each row also records operator/session IDs,
hardware/firmware/workspace revisions, actual controller-config hash, and the
unique `--video-id` for that attempt. The controller YAML hash does not cover
launch-time guard/timeout overrides, so archive the launch/manifest and software
revision too. The planner also creates or preserves `video_scores.csv`; it is a
manual evidence table and the analyzer does not populate it.

Analyzer status `planned_valid` proves exact automated plan factors and bag-data
quality only. The analyzer does not join the attempt log or automatically
condition comparisons on session, mounting, hardware/firmware/workspace
revision, or matched start placement. Audit those manual fields and recorded
origin XY before making a causal caster claim; unmatched or changed blocks must
remain descriptive.

Under study-stack contract version 1, generated commands preflight the full
expected trajectory/controller hashes before recording. The recorded contract
declares that the study launch sets the Vicon pose guard false, odometry
timeout to 0, reference timeout to 0.12 s, and world-frame odometry-twist
velocity, while the resolved controller YAML supplies the wheel-speed limit and
turret enable state (currently false). These fields are not an independent query of the running
controller; using `run_study_stack` and auditing the retained launch/software
revision are procedural controls. Treat a changed contract or hash as a
different experimental condition.

Analyze against a versioned accepted plan with `analyze_study_test --plan PATH`
or set `HAMR_STUDY_PLAN`; otherwise the analyzer uses canonical
`study_results/planned_trials.csv`.

### Gate 0: system qualification

Before comparing casters, verify the data path and estimate the measurement
floor:

1. Three `straight_forward` runs at 0.15 m/s; use their startup/final holds to
   quantify stationary Vicon/control jitter.
2. One low-speed pilot of `straight_backward`, both lateral profiles, and
   `forward_reverse`.
3. Inspect commanded/measured wheel RPM, limit state, battery and IMU topics (if
   available), base/turret pose semantics, and video visibility.

Confirm stable metadata, closed bags, Vicon coverage, sensible actuator traces,
and repeatable start conditions. A common speed deficit during the one-way
profiles is a system issue to diagnose before attributing composite-path error to
the caster.

### Gate 1: small diagnostic flat-ground comparison

For each test-ready caster, run the following on flat ground at 0.15 m/s,
nominal initial orientation, with three independent repetitions:

- `straight_forward` and `straight_backward`;
- `lateral_left` and `lateral_right`;
- `forward_reverse`.

This is 15 formal runs per caster and 30 for a two-caster comparison. It tests
steady opposite-axis translation, lateral onset, directional asymmetry, and a
controlled reversal at one medium speed. Complete and review this block before
committing time to composite paths or speed sweeps.

If only one caster is test-ready, the same matrix supports a characterization
paper, but not a claim that one caster design outperforms the other.

### Gate 2: selected path-following confirmation

Only after Gate 1 is reliable, run the implemented `triangle`, `circle_ccw`, and
`circle_cw` profiles at 0.15 m/s, three repetitions per caster. This adds nine
runs per caster (18 total, 48 cumulative for two casters) and connects the
isolated diagnostic effects to complete-path performance. Keep both circle directions; choosing only the favorable direction
would hide directional asymmetry.

### Gate 3: targeted speed sensitivity

If Gate 2 remains repeatable, add 0.10 and 0.20 m/s only to
`straight_forward` and `forward_reverse`, with three repetitions per caster.
This adds 12 runs per caster (24 total, 72 cumulative for two casters). It tests
whether speed error is a general drive-scale problem and whether the reversal
penalty grows with speed. If continuous-curvature speed sensitivity is instead
central to the paper, predeclare both circle directions
as the speed-sweep family; do not select a direction after seeing its result.

### Gate 4: one focused extension

Choose at most one extension based on the stated research emphasis and hardware
readiness:

- **Initial orientation:** only after freezing a caster-design-specific physical
  directed feature/vector, 0° direction, positive/viewing direction, setting
  tolerance, and photo/measurement procedure, use one diagnostic profile, a
  mechanically set and video-verified orientation set (for example 0, 90, 180,
  and 270 degrees), one speed, and three repetitions.
- **Uneven terrain:** one fully dimensioned, fixed obstacle/surface condition,
  its matched flat control, one diagnostic profile, one speed, and three
  repetitions. The current analyzer's z range of 0.25--0.40 m and tilt limit of
  0.35 rad are flat-core-only; predeclare and validate a terrain-specific pose
  envelope and analysis revision rather than silently reusing or relaxing them
  after inspecting terrain data.
- **Prototype mechanism:** close video of a spherical-caster shell/roller
  transition during one predeclared profile, linked to the same event window in
  the robot data.
- **Base rotation:** the separately validated direct-base clockwise and
  counterclockwise profiles at one angular rate, three repetitions per caster.

Treat a Gate 4 condition selected after seeing earlier results as exploratory.
It can motivate design recommendations, but it is not an independent
confirmation of the pattern that caused it to be selected.

## Experimental controls and run order

The experimental unit is one independently reset robot run, not each Vicon
sample. Hold constant or record:

- robot, caster hardware revision, mount and fastener state, tire/wheel state,
  payload, turret/gimbal configuration, and marker layout;
- firmware, controller gains, RPM limit, wheel-radius/kinematic parameters,
  workspace revision, trajectory YAML text/hash, and analysis revision;
- commanded and measured wheel RPM, RPM-limit/saturation state, motor-control
  output or current when available, and measured battery voltage under load;
- base, turret, and gimbal orientation as distinct signals; the inactive
  reference-yaw field and turret orientation are not substitutes for measured
  base yaw;
- IMU identity, mounting axis, rate, calibration, and accelerometer/gyro streams
  when available and verified; otherwise describe chassis disturbance from
  video qualitatively rather than claiming measured vibration;
- surface and obstacle geometry, start XY and raw base-yaw tolerance, lead-in geometry,
  nominal initial-caster setup label and setting method; for an orientation
  experiment also record the frozen feature/vector, zero/sign/view convention,
  tolerance, verification method, and convention revision;
- battery state when measurable, warm-up procedure, operator, session/day, and
  Vicon/camera configuration; and
- physical interventions, safety stops, collisions, tracking gaps, and mounting
  work between runs.

Run one unscored warm-up after startup or a caster swap. Follow the planner's
frozen deterministic rotate/reverse maneuver order within each hardware block;
it is counterbalanced, not randomized. Follow its counterbalanced caster order
across blocks so battery, surface, and time do not always favor the same design,
and acknowledge residual order effects as a limitation. If caster swaps
cannot be counterbalanced, report caster configuration as confounded with
session/order. Repetition labels should identify matched planned trials, not
retries.

Keep every attempted bag. Classify outcomes before examining performance:

- **valid kinematic run:** complete lifecycle, correct setup, and passing
  reference/Vicon quality;
- **valid reliability event:** any planned attempt, including safety stops or
  hardware/controller failures, with a categorized reason;
- **unscorable video:** kinematics may remain valid, but caster/contact claims
  are unavailable; or
- **externally invalid:** wrong setup, corridor intervention, or unusable
  sensing, retained with the exclusion reason.

Maintain an immutable pre-run **planned-cell ledger** with `planned_id`, caster,
profile, speed/rate, terrain, initial condition, repetition, matched and
hardware block, planned order, profile hash, and raw-base-yaw condition. Each
launch is a separate `attempt_id` linked to exactly one
`planned_id`. Record whether an attempt produced a closed bag, completed motion,
passed kinematic validation, and had scorable video. A replacement gets a new
`attempt_id`; it never erases the failed attempt or changes the planned count.

Never remove an outlier merely because its motion was poor. Replace an invalid
planned cell with a new timestamped run while retaining both attempts.

At each milestone report this flow explicitly:

```text
planned cells -> attempted launches -> closed bags -> completed trajectories
              -> valid kinematic runs -> video-scorable runs
```

Use planned cells as the denominator for matrix completion, attempted launches
from `attempt_log.csv` as the denominator for operational reliability, and valid
runs only for kinematic performance. Analyzer plan coverage measures cell
completion and recorded-data yield, not operational reliability, because it
cannot observe launches that made no bag. Report failure/exclusion categories
beside each count.

## Manual video scoring record

Use one `video_scores.csv` row per visible event, linked by `attempt_id`,
`run_id`, `planned_id`, and `video_id`. Store at least:

- scorer ID, rubric revision, camera/view, native frame rate, and whether the
  caster/contact region stayed visible;
- synchronization event, robot-to-video time offset, and synchronization
  uncertainty;
- profile segment/event, video start and end time (or frames), and the nominal
  caster angle before the event;
- first visible caster motion, alignment-complete time, derived alignment delay,
  and the predeclared definition of "aligned";
- scrub/slip score, contact-transition score, and chassis-disturbance score,
  each on the same 0--3 ordinal rubric;
- score confidence, occlusion flag, safety/intervention flag, and a short factual
  observation separate from interpretation.

Use the analyzer's exact `video_timeline.csv` rows and the same sign convention
for every run:

```text
video_time_s = plot_elapsed_s
             + video_motion_onset_s
             - plot_motion_onset_s
```

This is an onset-based alignment estimate unless the camera shares a clock or a
synchronization event was recorded; preserve its uncertainty and do not call it
frame-accurate otherwise.

Use a frozen rubric: 0 = clearly visible and absent, 1 = slight/brief, 2 =
clear or repeated with a noticeable motion effect, and 3 = severe, persistent,
or trial-affecting. Use `NA/unscorable` for an occluded or ambiguous event;
never convert missing visibility to zero. Freeze the rubric before formal
scoring. Have a second scorer independently score a predeclared subset and report exact/weighted
agreement rather than resolving every disagreement silently.

## Predeclared hypotheses and outcome mapping

| Hypothesis | Comparison | Evidence that supports it | Important alternative explanation |
|---|---|---|---|
| H1: steady speed mismatch is primarily system-wide | One-way baselines across axes, speeds, and casters | Similar multiplicative speed bias in matched directions/configurations | Wheel-radius/kinematic scale, motor loop, battery, saturation, or surface loss |
| H2: caster realignment creates a transient penalty | Lateral onset and `forward_reverse` versus a time-matched `straight_forward` segment | Larger peak/integrated error, speed loss, and settling time at onset/reversal | Reference acceleration, dwell response, or controller bandwidth |
| H3: caster designs differ most during reorientation | Matched spherical-minus-traditional differences for lateral/reversal and composite paths | Consistent raw-unit effect with video-aligned caster behavior | Mounting/session changes or actuator asymmetry |
| H4: the system has directional asymmetry | `straight_forward` versus `straight_backward`, left versus right lateral, and CW versus CCW circles | Repeatable signed difference across repetitions | Vicon-axis placement, wheel/motor asymmetry, start caster angle, or uneven surface |
| H5: reorientation cost grows with speed | Gate 3 speed-by-caster trend | Reversal penalty increases from 0.10 to 0.20 m/s beyond the straight baseline | RPM cap, controller saturation, or poorer Vicon differentiation |
| H6: uneven terrain changes the caster tradeoff | Matched flat-versus-terrain difference for one frozen profile | Repeatable degradation or improvement linked to documented contact events | Obstacle placement variability and approach error |

## Research-question-to-evidence map

| Proposal research question | Profile/data | Metric | Paper figure | Claim supported |
|---|---|---|---|---|
| What behavior causes the largest commanded-versus-realized deviation? | Five diagnostic baselines plus actuator data and synchronized video | Common active-interval time-weighted mean Euclidean XY tracking error (m), plus speed bias, reversal penalty, saturation exposure fraction, and video event score | Ranked common-metric plot and event-aligned command/wheel/base trace | Identify the tested maneuver with the largest repeatable common-metric deviation; identify caster association only when actuation and video evidence agree |
| How does behavior change for sideways, backward, reversal, rotation, or uneven terrain? | Left/right lateral, backward, `forward_reverse`, triangle/circles; direct-base rotation and terrain only if their separate gated tests are completed | Directional contrast, reversal transient, corner/radial error, optional terrain degradation | Mirrored-direction paired plots and normalized path overlays | Compare only the completed modes; rotation or terrain cannot be inferred from translational flat-ground paths |
| When does the spherical caster help or hurt relative to the traditional caster? | Counterbalanced, matched caster trials for every selected cell | Spherical-minus-traditional primary-outcome difference, variability, completion, IMU/video secondary outcomes | Paired dot plot with every repetition plus mechanism video frames | Make a configuration- and condition-specific tradeoff statement, not universal superiority |
| Where does the physical robot depart from ideal holonomic behavior? | Reference, Vicon base motion, wheel command/feedback, limits, manually recorded battery when available, and verified IMU | XY/along/cross-track error, speed error, corner/radial error, actuator mismatch, disturbance proxy | Reference/actual overlays and command-to-wheel-to-base diagnostic panels | Quantify the departure and distinguish system-wide actuation evidence from caster-associated transients without asserting a unique cause |

Cross-profile ranking uses only the time-weighted mean Euclidean XY tracking
error over each profile's active interval, in metres. It is a common descriptive
metric, not a substitute for the mechanism-specific outcomes below. Never rank
or subtract the profile-specific primary outcomes across maneuvers because they
have different definitions and units.

Use profile-specific primary outcomes for within-profile caster comparisons to
avoid choosing the best-looking metric after collection:

- `straight_forward` and `straight_backward`: cross-track RMSE;
- `lateral_left` and `lateral_right`: integral of absolute along-track error
  over the first 2.0 s of commanded motion (m*s); cross-track RMSE, peak, and
  response delay are secondary;
- `forward_reverse`: integral of absolute along-track error over the first 2.0 s
  after reversal (m*s); peak, delay, overshoot, and closure are secondary;
- triangle: mean time-aligned corner error;
- circle: radial RMSE.

Speed bias, settling time, maxima, actuator measures, and video scores are
secondary outcomes unless the final protocol explicitly promotes one before
formal data collection.

## Analysis suitable for a small independent study

### Per-run processing

1. Validate metadata, lifecycle, reference completeness, and Vicon coverage
   before computing a performance result. For formal metrics, require every
   reference sample in each active-motion phase and applicable frozen 2.0 s
   event window to match a finite Vicon pose within 50 ms, with no effective
   Vicon support gap over 100 ms. Across the reference lifecycle, also require
   nonzero, strictly increasing base-odometry source stamps with no source gap
   over 100 ms. These rules are offline and never abort a run.
2. Use the frozen analysis-v4 dual-clock policy: map Vicon capture stamps with
   `mapped_source_time = source_time + median(receipt_time - source_time)`, use
   mapped capture time for primary pose and causal trailing-0.20-s velocity,
   and retain receipt time for recording coverage and delivery diagnostics.
   Do not manually shift a trace per run. This fixed-offset mapping assumes
   negligible relative clock-rate error, so retain its residual/drift
   diagnostics. Source/receipt clocks are unsynchronized; the offset is not
   absolute latency and advancing source stamps cannot rule out normal-rate
   replay.
3. Use active-motion samples for tracking metrics and the frozen event window
   for transient metrics. Keep startup/final holds for noise and endpoint checks.
4. Apply the same interpolation, angle unwrapping, derivative filter, and
   settling rule to every configuration.
5. Preserve raw values, validation status, exclusion reason, and video score
   alongside every derived value.

### Summaries and comparisons

With three repetitions per cell, show all three points. Report mean, sample
standard deviation, median, and range in physical units; sample SD is
unavailable rather than zero when fewer than two valid runs exist. Do not hide
variability behind a bar chart. A paired estimate is allowed only when both
rows share the immutable plan's nonempty `matched_block_id`; repetition labels
alone do not prove pairing. For each genuinely matched cell, calculate:

```text
caster difference       = spherical - traditional
reorientation penalty   = event-window outcome - matched straight-window outcome
directional asymmetry   = left/CW outcome - right/CCW outcome
terrain degradation     = terrain outcome - matched flat outcome
```

Report percent change only when the denominator is well above the hold-derived
noise floor; otherwise report the raw difference. Plot paired points connected
by condition and show the individual repetitions behind any mean trend. In the
implemented staged plan, the Gate 3 0.10 and 0.20 m/s rows are block-matched and
may be paired; the Gate 1 0.15 m/s rows are from a different collection block
and must be compared with Gate 3 speeds only through unpaired descriptive
condition summaries.

Three runs are a repeatability screen, not enough for a persuasive per-cell
normality test or a stand-alone claim of statistical superiority. Do not treat
hundreds of time samples from one run as independent observations. If enough
counterbalanced blocks are completed, a block-respecting permutation test or a
mixed model with caster, maneuver, speed, and predeclared interactions may be a
secondary analysis. Its unit of resampling must be the independent run/block,
and model complexity must be reduced if the data cannot support it. Emphasize
raw-unit effect sizes, consistency, and uncertainty rather than a binary
`p < 0.05` conclusion.

For completion/reliability, report attempts, completions, and categorized
failures as `n/N` for each caster and condition. Do not mix Vicon/corridor
invalidations with mechanical/controller failures in a single failure rate.

### Attribution checks

Use the command-to-motion evidence chain before blaming the caster:

1. If measured wheel speed does not reach wheel command, or a limit is active,
   the immediate evidence points to actuation/controller saturation.
2. If wheel speed tracks command but chassis speed has the same scale bias in
   every caster and direction, investigate wheel-radius/kinematic calibration
   and surface slip. The test cannot uniquely identify wheel radius by itself.
3. If error is concentrated after direction changes, changes with caster
   configuration, and coincides with video-observed realignment or scrub, it is
   reasonable to call it **caster-associated reorientation behavior**.
4. If CW/CCW or left/right asymmetry persists through caster swaps, investigate
   chassis, wheel, surface, and coordinate asymmetry. If it follows a caster
   assembly through counterbalanced swaps, caster attribution is stronger.
5. If Vicon gaps/jumps explain the trace, classify it as a sensing-quality issue,
   not caster behavior.

## Paper-ready results and argument

Organize the results around the proposal's guiding questions rather than around
chronological test days:

1. **Measurement and repeatability:** hold-derived noise floor, planned versus
   recorded/valid counts, one-way baseline repeatability, and the frozen
   hardware/software configuration.
2. **Where ideal holonomic behavior breaks down:** compare forward/backward and
   left/right lateral baselines with reversal, triangle, and circle outcomes.
3. **Caster comparison:** paired raw differences by maneuver, with individual
   runs and video-confirmed mechanisms.
4. **Operating-condition effects:** predeclared speed trend and, if completed,
   one orientation or terrain extension.
5. **Design tradeoff:** identify where a caster reduces tracking/reorientation
   error and where it increases disturbance, contact transitions, variability,
   or failures.

Recommended final figures and tables are:

- a setup/coordinate-frame figure and a hardware/software configuration table;
- normalized reference/actual path overlays with all repetitions visible;
- paired dot plots of each profile's primary outcome by caster;
- event-aligned lateral-onset/reversal error and speed traces with selected
  synchronized video frames;
- forward/backward, left/right, and CW/CCW asymmetry plots;
- speed-bias and reorientation-penalty trends versus commanded speed;
- a video-score heatmap, marking unscorable observations explicitly; and
- an attempted/valid/completed run table with categorized exclusion reasons.

The strongest defensible conclusion has this form:

> For this HAMR/CHUTNI configuration, controller, surface, and tested operating
> range, caster A showed a specified raw change in a predeclared tracking or
> transient metric relative to caster B, consistently/inconsistently across the
> listed matched trials; synchronized observations were consistent with a stated
> contact or alignment mechanism.

The results cannot, without additional sensing or experiments, establish direct
contact force, true wheel/caster slip, vibration magnitude, a unique wheel-radius
error, universal off-road superiority, or performance outside the tested
hardware/surface/speed range. Vicon path error alone is not proof of caster slip,
and three repetitions per cell do not establish broad population-level
superiority. Negative or mixed results remain useful: they define conditions in
which the caster effect is smaller than repeatability, reveal confounds, and
identify the sensing or design changes needed next.

## Paper completion criterion

The study is ready to write when the selected core matrix has no unexplained
missing cells; raw bags, manifests, exclusions, analysis outputs, and videos are
traceable; primary outcomes and processing rules are frozen; all attempts and
invalidations are accounted for; and every caster-mechanism statement is either
supported by suitable video/sensing or explicitly labeled as an inference.
