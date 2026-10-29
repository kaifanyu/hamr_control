# Combined caster replay, analysis, and Vicon comparison preparation

The `combined-sim` workflow keeps the recorded encoder history, the Vicon
measurements, and the visualization mapping identifiable. It provides a paired
caster replay, a reproducible seven-page bag report, and aligned inputs for a
later physical comparison.

## Saved recording and report

The [October 29 recording archive](analysis/encoder_20261029T065650Z_7f3140ba/README.md)
contains the original report, seven plots, numerical summary, decoded pose and
encoder histories, and updated simulation previews. Its artifact manifest
records hashes and the exact combined-bag start epoch. The original MCAP is
identified by SHA-256:

```text
d0f738c772b523fcdba41eff4d30cf8f041fbb2f53b6e2f12032089ee6b4a48e
```

The recording lasts 122.176 seconds. All six encoder streams overlap at bag
times 20.591–86.028 seconds. The descriptive smoothed base path is approximately
12.16 metres long, with a 0.31 m/s peak speed and about 0.11 metres between its
start and finish during encoder coverage.

The encoder export reports continuous approximately 200 Hz streams with no
missing sequences or invalid samples in this run. This establishes recorded
continuity, not physical angle calibration. During their common interval the
left and right Vicon forks have 8 and 106 adjacent orientation changes exceeding
10 degrees, reaching nearly 180 degrees. These use shortest quaternion rotation
angles and persist relative to the base; quaternion sign equivalence and Euler
wrapping do not explain them. Tracking ambiguity is a plausible explanation,
but individual marker measurements are absent, so its cause is unresolved.

The left drive's saturation flag is active for approximately 41% of active
samples. Later prediction should use measured motion. Encoder clock-map
uncertainty fields are approximately 50–99 ms; these are estimates rather than
validated error bounds. Vicon header-to-bag delay is not a direct measurement
of end-to-end capture latency.

## Reproduce the analysis

Use a ROS 2 Humble/Jazzy environment with the MCAP storage reader, NumPy, SciPy,
Matplotlib, and PyYAML. The canonical analysis script accepts the recording
directory or combined bag and writes into an explicitly selected output folder:

```bash
source /opt/ros/humble/setup.bash
python3 hamr_bringup/scripts/analyze_combined_caster_bag.py \
  /path/to/encoder_20261029T065650Z_7f3140ba \
  --output-dir "$HOME/hamr_analysis/20261029T065650Z/analysis"
```

When supplying a bag separately, use `--sidecar-dir /path/to/encoder_recording`
to supply its `export-summary.json` and `clock-map.json`. The source is read
without modification. Reports retain source identity, timing basis, decoded
topic counts, frame identities, and the processing assumptions. The archived
report remains the original snapshot; the portable script provides the current
reproduction entry point.

## Replay both recorded casters

```bash
bash hamr_bringup/scripts/render_caster_recording.sh \
  /path/to/encoder_20261029T065650Z_7f3140ba \
  "$HOME/Videos/hamr_sim/combined-sim" --wheel both --width 1920
```

The base remains stationary while both casters use one replay clock. Six plots
show the source encoder angles. The progressive trajectory inset reads the
same run's `/HAMR_base/odom`. The full vehicle is the native-size model selected
by `hamr_sim_ui`: `compa_ball_caster_recording.urdf.xacro`. Front-left CENTER uses
sign −1 about its saved reference count 1210; front-right keeps its own reference.
These CAD mappings are separate from the unmodified sensor traces.

The later `20261029T070111Z_7b76081e` recording has two missing clock maps and
requires explicit `--recover-timing` for approximate replay. Its source folder
has no matching robot bag, so its trajectory inset is empty. The earlier run's
Vicon data must not be shifted onto that different run. See the
[local recording guide](LOCAL_CASTER_RECORDING.md#render-a-recording-on-ubuntu)
for timing recovery and per-wheel mapping controls.

## Prepare the later fork-versus-encoder comparison

This step runs directly from the archived CSVs and summary; it does not require
ROS or the original MCAP:

```bash
python3 hamr_bringup/scripts/prepare_caster_vicon_comparison.py \
  docs/analysis/encoder_20261029T065650Z_7f3140ba \
  --config hamr_bringup/config/caster_vicon_comparison.yaml \
  --output-dir "$HOME/hamr_analysis/20261029T065650Z/comparison"
```

The [checked preparation example](analysis/caster_vicon_comparison_20261029T065650Z/README.md)
contains its diagnostic plot and metadata. It reproduces 8 left / 106 right
orientation jumps across 7,233 left / 7,215 right aligned fork observations.
When an archive manifest is present, the command verifies the source identity,
exact epoch origin, and input hashes before producing outputs.

It aligns each fork observation with available base-pose and same-wheel encoder
history on the shared bag clock, records the sample ages, and flags gaps and
orientation jumps. Original poses remain available. Near-180-degree changes
are not silently flipped back, and interpolation does not bridge rejected
orientation transitions. Separate position and orientation quality fields make
the masks inspectable. Smooth orientation segments alone do not establish that
Vicon chose the correct tracked-body orientation.

Vicon measures the tracked fork rigid body. CENTER measures rotation of the
internal carrier relative to that fork, and H1/H2 measure shell spin. Directly
subtracting fork yaw from CENTER would compare different physical quantities.
The preparation exports therefore support inspection and future model input;
they do not report calibrated encoder error or independently predicted ball
rotation.

Before a physical prediction or accuracy comparison, record and validate:

1. The marker-frame-to-fork transform and marker-origin-to-ball-center lever arm
   for each wheel, plus the tracked base frame's relation to the robot model.
2. Physical encoder signs, zeros, shell assignments, and PPR/scale calibration.
3. Clock offset/latency assumptions and uncertainty, retaining the original
   timestamps alongside any explicitly calibrated alignment.
4. A treatment of ambiguous Vicon orientations and a rolling/contact model with
   stated slip and unmeasured-roller assumptions.

The comparison configuration leaves physical prediction unconfigured until
these prerequisites are supplied. The configuration and preparation summary
retain that distinction for later work.
