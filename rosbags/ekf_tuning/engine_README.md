# Deterministic robot_localization evaluation engine

This engine calls the **unmodified 15-state robot_localization EKF** from the
pinned Jazzy source in `engine_vendor`. It avoids ROS transport during parameter
search, so a roughly 5,600-message recording evaluates in about 0.07 seconds on
one core of the development machine. Each call owns its filter; Python threads
may evaluate independent parameter sets in parallel.

Build and verify:

```bash
cd rosbags/ekf_tuning
bash engine_build.sh
/usr/bin/python3 -m unittest engine_test.py
```

Requires Eigen headers, a C++17 compiler, Python/NumPy, and the ROS Jazzy core
libraries in `/opt/ros/jazzy` (`ROS_PREFIX` can override that path). The generated
`engine_core.so` is a local build artifact.

```python
from engine import Engine
trajectory = Engine().run(events, q_diag=q, r_diag=r, mask=mask)
```

- Input rows: `[time_seconds, type, vx, vy, yaw, wz, ax, ay]`; type 0 is wheel,
  type 1 is IMU. Inputs must be finite, sorted by common-clock timestamp, with
  axes already transformed to `base_link`. Units are SI.
- Output rows: `[time_seconds, x, y, yaw, vx, vy, wz]` after each input message.
  `full_state=True` returns all 15 state elements instead.
- `mask` and measurement variance `r_diag` order: wheel vx, vy, yaw, wz;
  IMU yaw, wz, ax, ay. A true mask enables that individual measurement.
- `q_diag` and `p_diag` follow the standard 15-element RL state ordering.
- Defaults for Q and P match the repository's original `ekf.yaml`. **Specify R
  explicitly** from the recording or replay profile: convenience defaults are
  not an estimate of sensor uncertainty.

Equivalence is limited to the selected planar measurement setup: identity
sensor mounting transforms, diagonal R/Q/P, no differential pose mode, no
control input, no rejection gate, and ascending measurement timestamps. It
constructs separate IMU pose, angular velocity, and acceleration measurements,
and applies the same seven 2D pseudo-measurements at variance 1e-6 as the ROS
wrapper. It does not simulate DDS losses, callback scheduling, publication
timers, TF lookup failures, or timeout-only prediction between messages.

For gyro-only fusion, IMU orientation is unused. For orientation fusion with
nonzero roll/pitch, compute `yaw(q_initial.inverse() * q_current)` externally
and pass `relative_imu=False` to reproduce the ROS wrapper's full quaternion
relative transform. Scalar subtraction through `relative_imu=True` is exact
for planar quaternions. Gravity removal and IMU lever-arm compensation must
likewise be handled before calling this engine when applicable.

Vicon is never an engine input. Reference alignment, timestamp interpolation,
dropout masks, calibration, and objective scoring are external to the filter.

## Actual ROS node validation

The packaged Jazzy 3.8.3 numerical library and the vendored numerical core gave
bit-identical states for a complete recording with gyro-only, yaw+gyro, and
yaw+gyro+accelerometer+wheel-yaw-rate masks. `bash engine_build.sh --packaged`
builds a second adapter against that locally extracted numerical library;
load it with `Engine("/absolute/path/engine_installed_core.so")` to repeat the
comparison. To validate the whole node:

```bash
bash engine_prepare_ros.sh
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 engine_validate_node.py data/hamr_hw_20260916_184054.npz \
  --output engine_results/node_baseline.json --rate 4
```

Packages are downloaded and extracted locally; system installation is not
changed. The harness runs on ROS domain 87 by default, publishes only wheel,
IMU, clock, and identity static TF, and records output from the actual packaged
`ekf_node`. It reconstructs planar message quaternions from the provided yaw,
so its comparison concerns the preprocessed planar data and numerical filter.
The harness supports `--events corrected_events.npy` and `--parameters trial.json`
(keys `q_diag`, `p_diag`, `r_diag`, `mask`, and `relative_imu`) to validate a
selected parameter trial. Generated JSON reports quantify the differences;
NPZ files hold paired actual-node and direct-core trajectories.

Final real-time node validation of the selected rounded calibration produced
Vicon position RMSEs of 0.04886, 0.10194, and 0.06698 m, versus 0.34088, 0.31633,
and 0.39599 m for the original gyro-only configuration. Node-versus-core
trajectory RMS differences were 0.000142, 0.000486, and 0.000412 m.

`--debug-counts` enables full upstream debug logging and checks actual
`processMeasurement` calls: all 21,140 inputs were processed, sensor timestamps
remained in order, and output rates were 49.97–50.02 Hz. Thirty-five
measurements were older than a previous timer-only prediction; the node still
fused each through correction. Full counts, effective YAML snapshots, and
metrics are in `engine_results/node_final*`. The harness uses input queue depths
of 200 to isolate numerical/timing validation from transport buffer capacity.
These node comparisons are not bit-exact; the numerical-library comparison,
which removes transport and timers, was exact.
