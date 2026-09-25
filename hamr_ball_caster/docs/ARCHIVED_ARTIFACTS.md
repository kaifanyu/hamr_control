# External CAD and experiment archive

The repository includes the runtime URDF/Xacro, complete STL meshes, source
tools, configurations, compact validation results, and plots. Native CAD,
large raw experiment traces, derived hardware arrays, and one exported map
were moved to an external archive while preserving every file byte-for-byte.
The archive preserves the original repository-relative directory layout.

The portable [archive manifest](validation/cleanup_archive_manifest.json)
records all 119 files, their byte counts, and SHA-256 hashes. The external
archive also contains `manifest.json` with the original source and destination
paths and the verified transfer status. The archived payload totals
288,457,574 bytes. Nothing in that payload was discarded.

Set paths for your checkout and the archive supplied with your development data:

```bash
export HAMR_REPO="$(git rev-parse --show-toplevel)"
export HAMR_ARCHIVE="/path/to/your/hamr-development-archive"
```

| Archived content | Path beneath `${HAMR_ARCHIVE}` | Purpose |
|---|---|---|
| 110 native CAD files | `CAD files/HAMR3-2/` | Original SolidWorks parts/assemblies and STEP files; required only to repeat native CAD extraction |
| Five full Gazebo ablation traces | `hamr_ball_caster/docs/corner_analysis/experiments/` | `baseline.json`, `no_slew.json`, `no_slew_half_step.json`, `half_speed.json`, `quintic_stop.json` |
| Three derived hardware arrays | `hamr_ball_caster/docs/corner_analysis/` | `hamr_hw_20260916_184054.npz`, `hamr_hw_20260916_190842.npz`, `hamr_hw_20260916_193228.npz` |
| Exported point cloud | `compa_slam/maps/compa_real_cloud.ply` | A generated map of one physical environment; not required by the caster simulator |

To repeat CAD extraction, install the optional dependencies listed in
`tools/cad-requirements.txt` in a separate environment, then provide the archive
directory explicitly:

```bash
python "$HAMR_REPO/hamr_ball_caster/tools/extract_cad.py" \
  --cad-dir "$HAMR_ARCHIVE/CAD files/HAMR3-2"
python "$HAMR_REPO/hamr_ball_caster/tools/extract_assembly.py" \
  --cad-dir "$HAMR_ARCHIVE/CAD files/HAMR3-2"
```

The existing generated meshes and provenance are sufficient to build, test,
and run the simulator; CAD extraction is not part of a normal build. Several
complete COMPA meshes in `meshes/compa` restore truncated legacy assets, so they
must remain in the repository. `prepare_compa_variant.py` verifies those local
copies against `assets/provenance/compa_retrofit.json` during regeneration.

To rerun analysis of the original archived corner experiments without rerunning
Gazebo, copy the five traces into their ignored working locations:

```bash
for case in baseline no_slew no_slew_half_step half_speed quintic_stop; do
  cp -- "$HAMR_ARCHIVE/hamr_ball_caster/docs/corner_analysis/experiments/$case.json" \
    "$HAMR_REPO/hamr_ball_caster/docs/corner_analysis/experiments/$case.json"
done
```

The hardware response plots can also be regenerated from the archived reduced
arrays without reading the original bags. Restore those arrays before running
`explain_hardware_response.py`:

```bash
cp -- "$HAMR_ARCHIVE"/hamr_ball_caster/docs/corner_analysis/hamr_hw_*.npz \
  "$HAMR_REPO/hamr_ball_caster/docs/corner_analysis/"
python "$HAMR_REPO/hamr_ball_caster/docs/corner_analysis/explain_hardware_response.py"
```

The [corner investigation](corner_analysis/README.md) retains the analysis
scripts, exact configurations, compact metrics, and plots. Its original bags
remain separate local sensor recordings under `rosbags/`; this cleanup did not
move or remove the existing EKF work. The `rosbags/COLCON_IGNORE` marker keeps
that analysis subtree out of ROS package discovery.

MP4 recordings and their full trajectory reports are also external outputs,
normally written beneath `${HOME}/Videos/hamr_sim`. Their run summaries and
implementation hashes remain in the committed validation records. A Git clone
does not include those recordings; the recording commands recreate them.
