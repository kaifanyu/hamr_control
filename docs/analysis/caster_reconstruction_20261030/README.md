# Caster reconstruction and Vicon replay — 2026-10-30

This handoff contains the code, recorded evidence, plots and videos needed to
continue the ball-caster investigation. It reconstructs CENTER and H1/H2 from
Vicon paths, base yaw and initial encoder state, then compares them with the
remaining encoder history.

**Start with the saved results.** The release includes all original recordings
and every analysis output; you do not need ROS or a Gazebo installation to view
the reports or rerun the prepared-data calculations.

- [Download the complete evidence and media](https://github.com/kaifanyu/hamr_control/releases/tag/caster-analysis-2026-10-30)
- [Technical findings, equations and investigation priorities](TECHNICAL_HANDOFF.md)
- [Artifact inventory and SHA-256 hashes](artifacts.json)
- [Portable scripts](scripts/)

## What is included

| Asset | Contents |
|---|---|
| `caster-analysis-20261030.zip` | All saved reports, 11 trajectory videos, 44 contact-only plots, earlier sphere comparisons, arrays, native Vicon pose caches, original scripts, fixed configuration snapshots and supplied project context |
| `recording-<run>.zip` (12 files) | Every file from each original recording folder: MCAP, CSV, binary history, logs, clock maps and sidecars; no original recording file omitted |
| `existing-analysis-batch-20261029.zip` | The complete original analysis batch initially supplied, including its reports, plots and simulations |

Large sensor recordings and videos are release assets, consistent with the
repository's existing archive policy. The Git branch contains the portable
source, technical handoff, compact previews and verifiable manifests.
Original evidence is retained byte-for-byte. Workstation paths inside archived
metadata are provenance, not requirements for the portable runners.

## Download and open

Run from the repository root using Python 3.12 (the tested version):

```bash
GUIDE=docs/analysis/caster_reconstruction_20261030
DATA_ROOT=/path/to/caster-analysis-data
python3 "$GUIDE/scripts/download_data.py" --destination "$DATA_ROOT"
```

The default downloads the analysis snapshot. Add `--all` to include every raw
recording and the original analysis batch. The complete evidence expands to
roughly 3.6 GB, in addition to the downloaded ZIP cache. Downloads are checked
against their published SHA-256 hashes; extraction refuses to replace a
different existing file.

Open these local pages:

- `$DATA_ROOT/outputs/caster_trajectory_replay/index.html`: side-by-side
  flat-ground videos and contact-only comparisons.
- `$DATA_ROOT/outputs/all_caster_reconstruction/index.html`: the preserved
  all-channel comparison page, including cumulative rotations and coasting
  sensitivity.

Both video panes follow the **same recorded Vicon trajectory**. The left uses
encoder-derived rotations; the right uses the ideal prediction. Matching
translation is imposed, not an independent prediction success.

## Reproduce without ROS

Use a virtual environment and install the numerical dependencies:

```bash
python3 -m venv .venv-caster-analysis
source .venv-caster-analysis/bin/activate
python -m pip install -r "$GUIDE/scripts/requirements.txt"
python "$GUIDE/scripts/run.py" reconstruct --data-root "$DATA_ROOT"
python "$GUIDE/scripts/run.py" contact --data-root "$DATA_ROOT"
```

New results go under `$DATA_ROOT/reproduced/`. The runner refuses output paths
inside the released `outputs`, `work`, `recordings` or `config` evidence trees.
Use `--output-dir /another/fresh/path` to choose another result root.

To render the featured recording, install FFmpeg on PATH and run:

```bash
python "$GUIDE/scripts/run.py" videos --data-root "$DATA_ROOT" \
  --case encoder_20261030T005046Z_7a2266b7
```

Use `--all` for all available videos, `--poster-only` for a quick visual check,
and `--skip-existing` to resume rendering. The saved videos are H.264/yuv420p,
1600 × 900, 15 fps. The numerical histories and contact plots use 100 Hz.

To prepare the inputs again from raw CSV histories and native Vicon pose caches:

```bash
python "$GUIDE/scripts/download_data.py" --destination "$DATA_ROOT" --recordings
python "$GUIDE/scripts/run.py" prepare --data-root "$DATA_ROOT"
python "$GUIDE/scripts/run.py" reconstruct --data-root "$DATA_ROOT"
python "$GUIDE/scripts/run.py" contact --data-root "$DATA_ROOT"
```

Reconstruction prefers newly prepared inputs when present. `--case <exact-name>`
limits preparation, reconstruction and contact analysis to one case. Use a
separate output directory when switching between a case-only experiment and a
full collection. `report` rebuilds pages from existing results.

Fresh extraction from MCAP needs ROS 2 Jazzy and the original extraction
scripts retained in the snapshot. The portable preparation stage uses the
included native pose caches and does not claim to replace ROS bag extraction.
The [reference environment](scripts/REFERENCE_ENVIRONMENT.txt) records the
versions used for the original analysis.

## Scope and interpretation

- 12 original sessions are preserved. Eleven support 13 valid shared segments,
  totaling **479.15 seconds**, with CENTER/H1/H2 for both front casters.
- The final `021139Z_a7eb78f9` case has encoder histories but no locally
  available Vicon bag and two rejected/missing timestamp mappings. Its
  recorded-only view is included; recovery details are in the technical guide.
- Gaps are excluded and the predictor is initialized independently at each
  valid segment start. Subsequent encoder values never correct the prediction.
- The current model is a **geometry-gated ideal shell-rolling baseline**:
  CENTER integrates motion with `R = 100 mm`; a hemisphere spins in its
  predicted spherical contact sector and its axial angle is held otherwise.
  A separate constant-rate coasting sensitivity is included. Actual roller/rim
  contact and free-shell dynamics are not solved.
- Contact-only plots select intervals from the **recorded CENTER angle** and
  nominal geometry. They are not contact-sensor measurements. Both curves use
  the same evaluation mask. Per-contact increments remove each episode's
  starting value only for that diagnostic; stored predictions remain unchanged.
- H1/H2 signs/scales, marker-to-center offsets and cross-system timing remain
  uncalibrated. Large discrepancies must not be labeled as measured slip alone.

The immediate investigation priority is the selected run's left CENTER startup
discrepancy, followed by encoder conventions, frame/timing calibration and then
the contact/free-spin model. Preserve the current baseline while changing one
assumption at a time.

## Validation and provenance

Archive inventories include per-file hashes, source sizes, ZIP CRC checks and
post-archive source verification. The original comparison page and all 72 files
in its report folder were preserved while adding the new replay/contact page.
Numerical checks cover rolling signs, threshold integration, quaternion
assembly, contact masks and episode boundaries. All 11 videos passed media
checks. These tests establish implementation consistency, not physical accuracy.

The [portable reproduction validation](scripts/PORTABILITY_VALIDATION.json)
records exact equality for the selected run's prepared arrays, reconstructed
histories and contact metrics, plus a pixel-identical poster. The release
downloader's extraction guards can be checked with:

```bash
python -m unittest discover -s "$GUIDE/scripts" -p 'test_*.py'
```

The repository base for this handoff is
`4f132eabb8d6cff4cf65867926e9829e3d4a13a3`. No controller, hardware configuration,
original rosbag or existing analysis output is modified by this PR.
