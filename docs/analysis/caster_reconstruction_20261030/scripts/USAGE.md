# Portable reconstruction scripts

Use Python 3.10 or newer. Install `requirements.txt`; install the `ffmpeg` executable separately if rendering MP4 videos. Video posters do not require ffmpeg. The renderer uses the DejaVu fonts bundled with Matplotlib, so no operating-system-specific font directory is required.

Extract the analysis data release into one directory, here called `DATA_ROOT`:

```text
DATA_ROOT/
  outputs/                         original reports and numerical results
  work/calibration_audit/           original Vicon pose caches
  work/all_caster_reconstruction/   prepared inputs and coverage manifest
  config/caster_encoder_replay_pair.yaml
  recordings/<case>/samples.csv     optional raw recording release archives
```

The `prepare` stage needs raw recording archives. The other stages can reproduce from the prepared inputs and pose caches supplied in the analysis release. Preparation uses the preserved pose caches; it does not require ROS or re-decode the MCAP files.

Run from this script directory, substituting the absolute extracted-data path:

```bash
python -m pip install -r requirements.txt
python run.py reconstruct --data-root /path/to/DATA_ROOT
python run.py contact --data-root /path/to/DATA_ROOT
python run.py videos --data-root /path/to/DATA_ROOT --all --skip-existing
python run.py report --data-root /path/to/DATA_ROOT
```

Outputs go to `DATA_ROOT/reproduced/`, leaving the released `outputs/`, `work/`, `recordings/`, and `config/` evidence unchanged. Use `--output-dir /another/fresh/folder` to choose a separate destination. The same flags work on Windows with a quoted Windows path. `CASTER_DATA_ROOT` may be set instead of repeating `--data-root`.

For raw CSV preparation before reconstruction:

```bash
python run.py prepare --data-root /path/to/DATA_ROOT
python run.py reconstruct --data-root /path/to/DATA_ROOT
```

Reconstruction prefers newly generated `reproduced/prepared/inputs/` when it exists; otherwise it uses the release's `work/all_caster_reconstruction/inputs/`.

For a quick selected-case run:

```bash
python run.py reconstruct --data-root /path/to/DATA_ROOT --output-dir /path/to/selected-reproduction --case encoder_20261030T005046Z_7a2266b7
python run.py contact --data-root /path/to/DATA_ROOT --output-dir /path/to/selected-reproduction --case encoder_20261030T005046Z_7a2266b7
python run.py videos --data-root /path/to/DATA_ROOT --output-dir /path/to/selected-reproduction --case encoder_20261030T005046Z_7a2266b7 --poster-only
```

Use a fresh output directory when changing from an all-case run to a selected-case run. This avoids mixing histories from different scopes. `reconstruct` builds the joint-angle report automatically; `contact` adds the contact-filtered plots and separate trajectory page. `videos` updates that page when contact results are already available. The default video case is `005046`; use `--all` for all available histories.

Open these local pages in a browser:

```text
reproduced/all_caster_reconstruction/index.html
reproduced/caster_trajectory_replay/index.html
```

The numerical algorithms are copied from the executed analysis. Changes in this portable edition are limited to path arguments, case selection, report scope labels, source bundling, and portable font lookup. The model retains all documented assumptions, including nominal geometry-based shell sectors and held inactive axial phases. Missing Vicon data is not reconstructed or fabricated. Contact filtering does not feed encoder observations back into the predictor.

See `REFERENCE_ENVIRONMENT.txt` for the original library versions and `SMOKE_TEST.json` for the selected-case portability check. The requirements allow newer compatible versions; small floating-point or font-rendering differences may occur across environments.
