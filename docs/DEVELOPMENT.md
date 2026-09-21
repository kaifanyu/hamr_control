# Repository layout and validation

## Entry points

| Location | Purpose |
|---|---|
| `reference_trajectory/reference_trajectory/continuous_waypoint.py` | Shared pure geometric planner and time parameterization |
| `reference_trajectory/reference_trajectory/continuous_execution.py` | Hardware start/stop state machine, timing and feedback checks |
| `reference_trajectory/reference_trajectory/continuous_waypoint_node.py` | ROS `ReferenceTraj` adapter and Trigger services |
| `reference_trajectory/config/continuous_waypoint_hw.yaml` | Real HAMR geometry and conservative reference limits |
| `hamr_bringup/launch/hamr_continuous_HW.launch.py` | Existing hardware stack plus the explicitly started planner |
| `hamr_ball_caster/launch/continuous_sim.launch.py` | Complete simulation run, optionally with MP4 capture |
| `hamr_ball_caster/urdf`, `meshes`, `assets/provenance` | CAD model, runtime meshes and reconstruction evidence |
| `hamr_ball_caster/config/continuous_tight_waypoint_sim.yaml` | Validated 2 cm simulation profile |
| `rosbags/ekf_tuning` | Existing calibration analysis, retained source and compact reports |

The simulation's small `scripts/continuous_waypoint.py` module only re-exports
the shared class. There is one planner implementation. Simulation and hardware
use different controller and actuator parameters. The planner generates a
reference; it does not replace the hardware feedback controller.

## Build and check

Follow [SIMULATION.md](SIMULATION.md) and
[REAL_CAR_CONTINUOUS.md](REAL_CAR_CONTINUOUS.md) for dependencies and builds.
After the simulation build, run the shared planner and hardware launch contract
tests from the repository root:

```bash
source hamr_ball_caster/scripts/env.sh
/usr/bin/python3 -m pytest -q \
  reference_trajectory/test/test_continuous_execution.py \
  reference_trajectory/test/test_continuous_ros.py \
  reference_trajectory/test/test_waypoint_traj_simple.py \
  reference_trajectory/test/test_study_trajectory.py \
  hamr_bringup/test/test_continuous_hardware_launch.py
```

The ROS contract test uses a separate local DDS domain, synthetic Vicon data and
a fake controller subscriber. It opens no serial devices and sends no motor
commands. It checks idle silence, explicit arming, captured start position,
stopping, stale-feedback abort, explicit restart and competing publishers.
These tests do not measure physical tracking accuracy.

The caster build script runs mesh/pose/inertia tests, whole-vehicle topology
checks, planner bounds, raw-feedback trajectory analysis and recording-quality
regressions. For a fresh full physics run:

```bash
ros2 launch hamr_ball_caster continuous_sim.launch.py gui:=false \
  output_dir:=$HOME/hamr_results/release_check_01
```

Inspect the generated `run_summary.json` and its individual checks. Historical
continuous-run evidence is described in
[TIGHT_CONTINUOUS.md](../hamr_ball_caster/docs/TIGHT_CONTINUOUS.md). Preserve
configuration snapshots and source hashes with new run evidence; a reference
curve's deviation from the original polyline is distinct from tracking error
relative to that curve.

## Repository integration check

The candidate Git tree was exported to a different directory and built with a
normal copied install, using no previous workspace overlay. All eight selected
packages built, and 326 regression cases passed: 195 caster, 113 shared/reference
and 18 hardware-launch cases. Both installed launches list their arguments;
hardware and simulation dry previews produce 98.318 s and 73.194 s plans.
Installed full-vehicle and fixture Xacros expand, and every referenced mesh
resolves inside the installed package. No native CAD archive is needed.

A fresh complete run through `continuous_sim.launch.py` passed after the planner
was moved into the shared package. The 73.194 s reference produced 0.394 mm RMS
and 1.646 mm peak sampled tracking error, with no intermediate stop and no wheel
speed/acceleration limiter activation. The largest observed commanded wheel
acceleration was 14.913 rad/s² against a 15 rad/s² simulation limit, so this is
validation of this configuration, not a claim of large actuator margin.
[The compact report](../hamr_ball_caster/docs/validation/repository_launch_validation.json)
contains all acceptance checks, configuration and implementation hashes, and
hashes identifying the external raw run artifacts. Both owned process groups
exited after the run. Real-car performance remains unmeasured by these checks.

## Source and development artifacts

Keep generated workspaces, bags, native CAD archives, raw experiment dumps and
videos outside the source tree. `.gitignore` excludes local generated output;
`rosbags/COLCON_IGNORE` also prevents nested analysis workspaces from being
mistaken for duplicate ROS packages. Runtime meshes, expanded URDFs, Xacro,
compact validation data, CAD provenance and third-party license files remain
versioned.

The cleanup preserved original CAD and large experiments in an external archive
with SHA-256 manifests. See
[ARCHIVED_ARTIFACTS.md](../hamr_ball_caster/docs/ARCHIVED_ARTIFACTS.md) for contents
and recovery conventions. Raw CAD is only needed to repeat CAD extraction;
building, running and geometry regression tests use the included meshes and
provenance. Existing EKF recordings and workspaces were preserved locally.

The large exported point cloud was removed from the current source tree; this
does not rewrite Git history or remove its old blobs from existing commits.
No history rewrite or remote push is part of repository preparation.

Before pushing your local branch:

```bash
git status --short
git diff --check
git log -1 --oneline
git push -u origin feature/continuous-planner-caster
```

Review the target remote with `git remote -v` if publishing to a different fork.
