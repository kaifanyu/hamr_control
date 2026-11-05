# HAMR caster orientation reconstruction — technical handoff

Date: 2026-10-30. Primary example: `encoder_20261030T005046Z_7a2266b7`.

This work reconstructs caster joint rotations from recorded Vicon paths, base yaw and one initial encoder state per valid segment, then compares the predictions with the remaining encoder history. It provides a reproducible ideal kinematic baseline and visual replay. It does **not** yet establish a calibrated physical orientation predictor or identify slip as the cause of disagreement.

Paths below refer to the downloaded data root, not this Git directory. See [README.md](README.md) for verified downloads and portable reproduction commands.

## Start here

- **All-channel reconstruction** (`$DATA_ROOT/outputs/all_caster_reconstruction/index.html`): CENTER, H1 and H2 for both front casters, with full cumulative histories and a coasting sensitivity.
- **Trajectory videos and contact-only comparisons** (`$DATA_ROOT/outputs/caster_trajectory_replay/index.html`): recorded Vicon paths with encoder-derived rotations versus predicted rotations; contact-restricted phase, residual, rate and per-contact increment plots.
- **Reconstruction results** (`$DATA_ROOT/outputs/all_caster_reconstruction/results.json`), **input manifest** (`$DATA_ROOT/outputs/all_caster_reconstruction/input_manifest.json`) and **contact metrics** (`$DATA_ROOT/outputs/caster_trajectory_replay/contact/contact_metrics.json`): authoritative machine-readable definitions, exclusions and scores.
- **Original ideal-sphere baseline** (`$DATA_ROOT/outputs/ideal_sphere_results.json`) and **constrained CENTER results** (`$DATA_ROOT/outputs/constrained_caster_center_results.json`): earlier models retained to explain what changed.

The original all-channel reconstruction was preserved when the separate replay/contact analysis was added. Predictions are not refitted or reset at contact transitions.

## Scope and data

The available collection contains **12 encoder sessions**. **11 sessions yield 13 valid synchronized segments, totaling 479.15 seconds** and six predicted/recorded channels per segment. A case with excluded data gaps is initialized independently after each gap; no phase continuity or trajectory is invented across missing data. The selected example supplies one 45.04-second segment.

Original source locations were `/home/para/Encoder Rosbag` and the matching batch under `/home/para/hamr_control/analysis/batch_20261029_233502`. The manifest preserves original filenames, hashes, device identities, timing metadata, validity decisions and recovery notes. Use the manifest to distinguish duplicate copies from independent recordings.

The twelfth session, `encoder_20261030T021139Z_a7eb78f9`, remains **encoder-only**. No matching local robot/Vicon bag or pose cache was found. Its recorded remote bag location is `/home/hamr/hamster_ws/src/hamr_control/rosbags/hamr_hw_20261029_221107`; the remote robot was not contacted. The session is marked `needs_recovery`, and front-left H1 plus front-right CENTER have no accepted mapped timestamps. Native board-clock traces are retained as evidence, not presented as synchronized Vicon comparisons. Recover the bag and clock mapping before reconstructing this case.

## Geometry, frames and initial state

| Quantity | Current convention |
|---|---|
| Nominal sphere radius | `R = 0.100 m`, fixed; no radius fit in these reconstructions |
| Equatorial gap | `20 mm` total axial width; half-gap `10 mm` |
| Polar opening | `50 mm` opening diameter; **not** the small roller's outside diameter |
| Ground | Horizontal, world `+Z` normal; sphere center height assumed `R` |
| Translation input | Each caster's Vicon marker XY, used as an uncalibrated ball-center proxy |
| Upright fork heading | `psi = base Vicon yaw + pi/2` |
| CENTER | `delta`, saved H1-up reference, current raw encoder sign `+1` |
| H1/H2 | Node 1 / node 2; independent axial joint phases `beta1`, `beta2` |
| Initial axial phases | Changes from the segment's first reading; material longitude starts at a common arbitrary zero |

CENTER saved reference counts are left `7792/16384` and right `16183/16384`. The source mapping is `hamr_bringup/config/caster_encoder_replay_pair.yaml`; its hash is in the manifest. Do not silently change these conventions during comparison.

Positions are resampled at 100 Hz and smoothed separately within valid segments using an 11-sample, order-2 Savitzky–Golay filter. This is an offline reconstruction using recorded trajectories and future samples in the smoothing window. Encoder timestamps use the existing clock mapping; timing accuracy has not been independently established. Right caster Vicon quaternion flips are one reason the model uses base yaw and caster positions rather than treating caster-marker quaternion as reliable shell orientation.

## Model evolution

1. **Free rigid sphere:** integrate `omega = (n cross v)/R`, with normal-axis spin set to zero. A sensitivity adds base yaw rate as normal spin. This gives a full virtual orientation, but its split axis can leave the actual carrier plane. Adding base yaw rate alone does not enforce the carrier constraint.
2. **Constrained CENTER:** preserve the carrier's joint geometry and integrate `delta_dot = -v_x/R`. This exposes where exact smooth-sphere axial rolling becomes singular and remains a useful CENTER prediction through those states.
3. **Current separate-shell baseline:** integrate CENTER throughout; allow the nominal contacting shell's axial joint to roll only in its material domain, and hold the other axial joint. In gap/polar sectors both axial joints are held. This declared continuation gives complete finite H1/H2 histories without pretending to solve the missing roller/rim mechanics.

For local fork velocity `(v_x, v_y)`, define `F = Rz(psi) Ry(-delta)`. The H1 pole axis is `a = F ez`; H2's positive spin axis is `-a`.

```text
delta_dot = -v_x / R
H1 active: beta1_dot =  v_y / (R sin(delta))
H2 active: beta2_dot = -v_y / (R sin(delta))
Inactive or non-shell axial joint: beta_dot = 0

Q_H1 = Rz(psi) Ry(-delta) Rz( beta1)
Q_H2 = Rz(psi) Ry(-delta) Rz(-beta2)
```

Both `Q` matrices map their common-reference material coordinates into world coordinates. H2's opposite axial sign is intentional. Holding an axial joint does not freeze its full world orientation: the shell still follows CENTER and fork yaw.

Let `u = cos(delta)`. Nominal shell eligibility is

```text
0.1 <= abs(u) <= sqrt(1 - 0.25^2).
H1 is the nominal lower shell if u < 0; H2 if u > 0.
```

The hemisphere's axial rolling lever arm is `R |sin(delta)|`, rather than a single fitted radius. It is at least 25 mm within the eligible material domain. CENTER retains the ideal 100 mm sphere assumption in every mode. All predictive mode decisions use **predicted** CENTER; subsequent encoder values never choose prediction modes.

Each 10 ms displacement step uses midpoint fork yaw. The integrator splits that step at every material-domain crossing, treats CENTER as linear within the step and integrates `csc(delta)` analytically using `log|tan(delta/2)|`; very small angular steps use its stable midpoint limit. This is exact for the declared piecewise input model apart from that small-step approximation.

Without the geometry gate or another explicit continuation, the exact two-joint smooth-sphere model need not exist for an arbitrary prescribed path: near a pole with nonzero `v_y`, axial rate diverges as `1/sin(delta)`. Setting the one pole sample's rate to zero does not repair its neighborhood. The current non-shell hold is an assumption, not an exact no-slip solution through such states.

A dashed sensitivity retains each shell's last instantaneous active joint rate while inactive, initially zero until its first active interval. It has no decay, may spin through stops, and is neither a calibrated free-body model nor a confidence interval. The non-shell `integral(abs(v_y) dt)` is unexplained transverse travel at the virtual sphere bottom, not measured physical slip.

## Results to preserve and interpret correctly

For the selected 45.04-second run, the earlier free-sphere split-axis RMSE was approximately **95.48° left / 102.19° right**. Fresh initialization within a straight portion gave **6.09° / 2.99°**. Those short-window scores do not describe full-run accuracy, and that model's axis could leave the carrier plane.

The current geometry-gated model has these **full-segment unwrapped phase RMSEs**:

| Caster | CENTER | H1 | H2 |
|---|---:|---:|---:|
| Left | 224.92° | 380.44° | 1348.68° |
| Right | 22.13° | 309.47° | 439.99° |

Do not compare these directly with the earlier bounded split-axis metric: unwrapped phase error retains complete missing turns. Quaternion geodesic and wrapped phase errors can conceal them.

Contact-only primary scores use **encoder-inferred nominal contact** under the stated geometry, applying the same mask to the stored prediction and encoder phase. This is evaluation selection, not encoder feedback and not a contact-sensor measurement. CENTER uses the H1/H2 union; each hemisphere uses its own sector. Whole-step rate statistics reject any 10 ms interval that crosses a sector boundary.

For the selected run:

| Caster/channel | Contact-only cumulative phase RMSE (turns) | Predicted same-shell agreement during observed contact |
|---|---:|---:|
| Left H1 | 1.004 | 26.9% |
| Left H2 | 3.494 | 46.1% |
| Right H1 | 0.833 | 75.1% |
| Right H2 | 1.261 | 66.1% |

The separate intersection score includes only matching predicted/observed contact. Across all sessions it retains approximately 65% of observed hemisphere-contact samples, so using it alone would hide substantial mode disagreement. Always report its coverage alongside error.

Per-contact increment plots subtract each curve's own value at the first valid contact sample. They isolate change within that episode and start at zero **by construction**; the stored cumulative prediction is unchanged. Their smaller errors are not evidence of a corrected full-run predictor.

Videos use **identical recorded Vicon paths in both panes**. The comparison is encoder-derived versus predicted rotation. Matching translation is imposed. Red dots show the virtual sphere bottom; neither the dots nor the nominal shell badges establish actual support contact. Shell quaternions are reconstructed from the same yaw/encoder conventions and are not independent Vicon ground truth for a hemisphere.

## Reproduce and extend

Use the portable runners documented in [README.md](README.md), which accept
`--data-root` and write new results under `DATA_ROOT/reproduced` by default.
The original scripts are retained byte-for-byte in the data snapshot for
provenance; their workstation paths should not be used as portable entry points.

```bash
GUIDE=docs/analysis/caster_reconstruction_20261030
python "$GUIDE/scripts/run.py" reconstruct --data-root "$DATA_ROOT"
python "$GUIDE/scripts/run.py" contact --data-root "$DATA_ROOT"
python "$GUIDE/scripts/run.py" videos --data-root "$DATA_ROOT" --all
```

The numerical stages need Python, NumPy, SciPy, Matplotlib, Pillow and PyYAML.
Video encoding additionally needs FFmpeg. Raw CSV preparation uses the included
native Vicon caches; fresh MCAP extraction still requires ROS 2 Jazzy. The
original 12 recording folders and original analysis batch are separate release
assets so they remain available for independent re-extraction and inspection.

Important schemas: `time_s` uses the case's Vicon-origin clock; `case_elapsed_s` preserves excluded gaps; `segment_elapsed_s` restarts at each valid segment. Phase changes restart only at segment initialization. NPZ histories contain recorded/predicted/coasting phase arrays, rates, mode fractions, positions, yaw and shell quaternions. Consult the input manifest and results JSON before concatenating segments.

Independent checks confirmed event integration against numerical quadrature, coasting subdivision consistency, H1/H2 signs, orientation formulas, all 78 segment/wheel/channel contact comparisons, strict rate episode indexing, no plotted contact-gap bridges in these data, and identical video paths. Those checks validate implementation consistency, not physical model accuracy.

## Highest-priority follow-up

1. **Audit selected-run left startup before changing model parameters.** Left CENTER error is near zero through 3 seconds, then approximately −239.9° by 5 seconds; right is +13.2° at 5 seconds. Inspect native counts, packet continuity, clock alignment, actual initial motion and reference handling. This early discrepancy can shift all subsequent predicted contact modes.
2. **Verify encoder conventions physically.** Confirm device-to-channel mapping, H1/H2 direction, counts per revolution and CENTER reference/sign with controlled rotations. Current source flags do not establish verified PPR, phase reference or timing. Keep an independent validation run after any calibration.
3. **Calibrate frames and timing.** Measure marker-to-ball-center and base-marker-to-fork transforms, validate the 90° heading offset, and estimate/check cross-system latency and drift. Caster-marker XY is currently only a center proxy.
4. **Separate mode disagreement from within-mode rolling error.** Examine cumulative CENTER error, observed/predicted contact overlap, and local rate/increment errors together. Do not silently choose measured modes inside a predictor or reinitialize at each contact to improve its scores.
5. **Add actual support mechanics only after those checks.** Rim/polar roller geometry, support height, contact transitions, inactive shell dynamics, friction and slip are not solved by the present model. A faithful split-body predictor needs explicit contact/free-spin equations and their validation; residuals here do not uniquely identify slip.
6. **Recover the missing twelfth run.** Retrieve its recorded robot bag and repair/validate the rejected clock mapping, preserving native records and existing exclusions.

Keep the current baseline and held-out comparisons intact while investigating. Record any new calibration, model assumption or reinitialization separately so improved scores remain interpretable.
