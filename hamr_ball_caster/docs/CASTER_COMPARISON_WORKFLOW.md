# Repeat the swivel study with the passive ball caster

The comparison records the existing passive CAD split-ball mechanism on every
saved swivel-study route. It preserves the requested waypoints, initial chassis
yaw, reference speed, rear-drive controller settings and ground coefficient.
Both front ball assemblies remain passive. Each has a carrier, two hemispheres
and two polar rollers; carrier phase is not a swivel-facing angle.

## Reproduce

Build and source the workspace containing both packages:

```bash
bash hamr_swivel_caster/scripts/build.sh
source hamr_swivel_caster/scripts/env.sh
```

Choose a new output directory. Preparation copies the actual recorded source
configuration rather than reconstructing it from assumptions:

```bash
ros2 run hamr_ball_caster run_ball_comparison.py /tmp/caster_comparison \
  --swivel-suite /absolute/path/to/swivel-suite \
  --mode prepare
ros2 run hamr_ball_caster run_ball_comparison.py /tmp/caster_comparison \
  --swivel-suite /absolute/path/to/swivel-suite \
  --mode record --domain-id 93 --resume
```

Use `--select straight_y,reversal_180` for a subset. Separate queues can use
separate ROS domain IDs with disjoint selected case lists. An exclusive claim
prevents simultaneously writing the same case. `--resume` retains saved run
summaries; inspect unsuccessful run and video checks before retrying. Preserve
any partial attempt separately before restarting that case. `--mode screen`
uses physics without cameras and saves under a separate `screen` directory.

For a single new route:

```bash
ros2 run hamr_ball_caster run_ball_caster.py --profile legacy --record \
  --config /absolute/route.yaml --initial-yaw-deg 90 --ground-mu 0.8 \
  --domain-id 93 --output-dir /tmp/my_ball_run
```

The runner saves the expanded URDF actually loaded, complete visual URDF,
world, configuration, command manifest, controller/physics logs, trajectory
JSON, passive-joint CSV and recording metadata. Default `--visual-detail
overview` removes hidden, dense bearing/fastener visuals while preserving every
collision, joint, inertia, shell, roller and fork. `--visual-detail full`
retains all source visuals. The snapshot audit checks that nonvisual XML is
identical. Both cameras record 1280 × 720 at 25 frames per simulation second.

## Analyze and publish the local page

Analyze every completed pair from the saved matrix, validate and annotate the
recordings, then build the report and offline page:

```bash
ros2 run hamr_ball_caster analyze_ball_comparison.py /tmp/caster_comparison
ros2 run hamr_ball_caster finalize_ball_comparison.py /tmp/caster_comparison
ros2 run hamr_ball_caster summarize_caster_comparison.py /tmp/caster_comparison
ros2 run hamr_ball_caster build_caster_comparison.py /tmp/caster_comparison
```

The common comparison separates motion, first leg, later legs, startup and
hold. Assisted starts use the same chassis recovery and forward lead-in, but
the ball mechanism skips the nonexistent swivel-angle release gate. Their
prepared first-segment source can differ; later legs are the primary tracking
comparison, with preparation time and displacement reported separately.

Swivel-only damping, inertia and tire-grip variations have no one-to-one ball
mapping. Their ball runs are explicitly nominal repeats. The architectures
also differ in mass, estimated bearing losses and contact geometry; this study
is not an equal-mass isolation of caster topology. See
[BALL_COMPARISON_ASSUMPTIONS.md](BALL_COMPARISON_ASSUMPTIONS.md).

Video validation checks timestamps, frame counts, source gaps and full decode.
The page uses timestamp offsets for approximate browser playback alignment.
Plots use full logged samples for metrics and decimate only display traces.
Local swivel-axis alignment is a rolling-direction diagnostic; it is neither
a direct contact-force measurement nor interchangeable with path-tangent error.
The ball mechanism is not assigned a fictitious zero alignment or slip error.
