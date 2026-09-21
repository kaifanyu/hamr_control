# Repository and localization audit (2026-09-20)

This audit records the original checkout's architecture, deployment paths, scoring
limitations, and existing-test results. It does not change controller or launch
behavior. Numerical bag experiments and their selected configuration are reported
separately. Paths below are relative to the repository root.

## Source inventory

| Area | Role and material reviewed | Localization implications |
|---|---|---|
| `hamr_uros_bridge` | C++ serial protocol, relay, IMU/tick/status publishers, command watchdog, legacy `odometry_eval` | Supplies the actual measurements; there is no EKF implementation in the bridge. IMU covariances are embedded in `src/relay_node.cpp`. |
| `hamr_odometry` | Python encoder integration, wheel/turret publishers, geometry and QoS | Supplies `/wheel_odom`; measurement covariances are embedded in `holonomic_odom_node.py`. |
| `hamr_bringup` | Hardware/simulation/offline launches, EKF YAML variants, recorder, calibration/response tools, supervised study wrappers | Selects which EKF really runs. Several alternative workflows have different geometry or filter defaults. |
| `rosbags` | Covariance-copy/profile helpers, Vicon/localization analyzer, study analyzer, firmware/actuator diagnostics and tests | Existing offline workflow prepares a new bag and runs the actual `robot_localization` ROS node; it is not a standalone batch optimizer. |
| `hamr_control` | Production `hamr_controller.py`, older hardware controller, graphing and tests | Normal hardware closes the control loop on Vicon, not on `/local_HAMR/odom`. The installed `hamr_controller` entry point uses `hamr_controller.py`. |
| `reference_trajectory` | Waypoint, bounded one-shot, study profiles, translation/circle/triangle definitions and tests | These are commanded trajectories, not ground truth. Turret yaw is inactive in the current hardware study stack. |
| `hamr_control_exp` | Shared Jacobian, LQR/MPC controllers, smoothed trajectories, metrics and launches | Separate experimental stack; defaults should not silently replace the primary hardware stack. |
| `compa_slam` | RTAB-Map/D455 launch composition, frame adapter and math, runtime/mapping/handoff docs | Uses local EKF for continuous `odom -> base_link` and RTAB-Map for `map -> odom`. Global visual corrections are not injected into the local EKF. |
| `hamr_control_cpp` | Legacy control, path tracers, map/grid sources, A*/PRM/off-road planners | Consumers and simulation/planning tools, not the selected hardware EKF. |
| `compa_control_py` | COMPA five-axis inverse-Jacobian controller and visualization | COMPA simulation/control path, separate from HAMR encoder/IMU fusion. |
| `hamr_description`, `compa_description` | URDF/Xacro, Gazebo, meshes, RViz | Frame and simulation context. Live hardware currently publishes the IMU mounting transform directly from launch. |
| `hamr_interfaces`, `hamr_teleop` | Message interfaces and keyboard/velocity control | No localization math; generated message support is needed for full controller tests. |
| Root docs and `documents` | README, offline quickstart, two independent-study documents, four readable design/planner PDFs | The study documentation distinguishes reference tracking from localization and documents Vicon quality failures. |
| `img`, `map`, worlds and terrain assets | Demonstration media, raster maps, simulation geometry | Not measured localization trajectories; no EKF parameters. |

The tracked inventory contained 335 files across these areas at audit time.
The supplied `COMPA_paper.pdf` and `documents/[2017 Costa] Designing for Uniform
Mobility Using Holonomicity.pdf` could not be parsed: both lack a readable EOF
marker and raise `PdfStreamError: Stream has ended unexpectedly`. The four other
PDFs were extracted and read; they cover mobility ellipsoids, terrain A*, and map
representations, without EKF tuning information. Meshes, videos and raster assets
were inventoried rather than represented as text that had been read.

## Original active data flow and defaults

```text
MCU serial -> relay_node -> wheel encoder ticks -> holonomic_odom_node
                         -> /imu/data                 -> /wheel_odom
                                 \______________________/
                                          |
                                  robot_localization EKF
                                          |
                                  /local_HAMR/odom
                                  odom -> base_link TF

Vicon /HAMR_base/odom -> normal hardware controller
                      -> comparison/reference data in recorded bags
                      -> turret-yaw helper only inside holonomic_odom_node
```

The wheel odometry callback does not use Vicon to calculate wheel pose or twist.
Its Vicon subscription only supplies base yaw for the separate turret odometry
message; comments calling that input “EKF” are misleading. Vicon must remain
outside candidate filter updates during localization evaluation.

| Filter file | Original wheel selection | Original IMU selection | Selected by |
|---|---|---|---|
| `ekf.yaml` | `vx, vy` | gyro `vyaw` | Offline default; primary hardware only with `use_mag:=false use_orientation:=false`; experimental default |
| `ekf_mag.yaml` | `vy` only | relative quaternion `yaw` plus gyro `vyaw` | **Primary hardware launch default**, because `use_mag` defaults true |
| `ekf_orientation.yaml` | `vx, vy` | relative quaternion `yaw` only | `use_orientation:=true`; default in `compa_slam/localization_runtime.launch.py` |
| `ekf_trial_vy_gyro.yaml` | `vy` only | gyro `vyaw` | Explicit trial file |

All are 50 Hz, two-dimensional, local `odom` filters. Base `ekf.yaml` Q diagonal
is `[.05,.05,.06,.03,.03,.06,.025,.025,.04,.01,.01,.02,.01,.01,.015]` in the
standard 15-state order. Initial pose variances are `1e-9`; velocity/acceleration
variances are `1`. Original wheel `vx,vy,vyaw` measurement variances are `.01`;
IMU angular-velocity variances are `.0002`; quaternion yaw variance is `.0009`.
The alternate magnetometer profile uses Q yaw `.005` and Q yaw-rate `.01`.

An improvement to `ekf.yaml` alone therefore does **not** update the normal
hardware launch default or the SLAM runtime default. Any release must state
which profiles and launch routes receive the new settings. Replaying an old bag
also requires transferring sensor covariance changes into messages, not merely
editing the EKF YAML.

Primary hardware encoder geometry is `r=.1250`, `a=.350`, `b=.45`, yaw offset
`pi/2`, ticks/revolution `2263.7`, left/right tick scales `1.001460/1.000044`.
The controller YAML uses `r=.122`, `a=.350`, `b=.301`. Experimental hardware
odometry uses `r=.1250`, `a=.345`, `b=.301`. Thus comments stating that every
geometry matches the controller are stale. Calibration must identify the
physical tracked point, effective wheel radius/track width, and yaw convention.

## Concrete modeling and evaluation risks

1. **Published wheel twist has a one-step rotation error.** `_update` computes
   world `x_dot,y_dot` using the old heading, increments `theta`, then rotates
   those velocities into body coordinates with the new heading. The result is
   rotated by `-yaw_dot*dt`. With the configured `pi/2` offset, the intended body
   twist before that extra rotation is `vx=-b*yaw_dot, vy=v`. This is small per
   sample but systematic in turns and matters when fitting a lever arm.
2. **Covariance changes cannot remove model bias.** Wheel scale, track width,
   tracked-point offset, mounting yaw and gyro bias produce systematic errors.
   Standard velocity/gyro-only fusion has no external absolute position or yaw
   observation and cannot guarantee perfect long-term agreement with Vicon.
3. **Dropping wheel `vx` changes the motion model.** Here `vx` contains the
   offset-point velocity in turns; it is not simply lateral slip. A `vy`-only
   profile omits that measured component. Conversely, an incorrect `b` can make
   the omitted component wrong. Test calibrated geometry rather than assuming
   either selection is inherently correct.
4. **BNO055 relative heading is not immune to magnetic disturbance.** Subtracting
   the initial quaternion removes a fixed offset; time-varying heading error
   remains. Relative quaternion and raw gyro also arise from the same sensor
   and may contain correlated information. Evaluate orientation-only,
   gyro-only, and combined models separately.
5. **Vicon header timestamps are on a remote unsynchronized clock.** The live
   controller explicitly uses receipt time for freshness and source stamps for
   monotonic ordering/deltas. Do not join Vicon source timestamps directly with
   local IMU/wheel stamps. `analyze_hamr_study.vicon_source_clock_mapping` has a
   tested median(receipt-source) mapping and reports jitter and drift. The
   unknown constant combines clock offset and typical transport latency.
6. **The simple localization analyzer uses bag receipt times.** It does not read
   source timestamps, reject source freezes or Vicon geometric glitches, or
   interpolate trajectories for its headline score. It anchors each trajectory
   at its own first sample and then uses nearest-time matches. Unequal start
   times, moving startup, replay recorder delay, delivery bursts and differing
   sample coverage can change scores without improving localization. Use a
   common overlap origin, consistent valid time support and a fixed alignment
   policy for comparisons; report scored duration/sample counts.
7. **Raw Vicon needs quality checks.** Existing controller/study code documents
   previously observed wrong-pose clusters, jumps and delivery stalls. Absolute
   error relative to corrupt Vicon is not sensor calibration evidence. Apply
   ground-truth quality rules consistently, and report exclusions.
8. **Long stops can dominate sample-weighted RMSE.** Report full-run error and
   movement/turn error, final error, heading error and duration/coverage. Weight
   runs explicitly; a longer bag should not silently determine all settings.
9. **Use independent runs for validation.** Fit shared parameters to multiple
   runs and preferably leave each bag out once. Label a final all-data refit as
   in-sample; do not call the same three fitted bags unseen validation. Do not
   use per-bag fitted yaw/scale/time corrections as deployment parameters.
10. **Replay is not live callback scheduling.** Original wheel velocity comes
    from timer intervals and unstamped latest tick messages; IMU is stamped on
    host receipt. A batch filter must disclose its timestamp/order/output policy
    and be checked against `robot_localization` before claiming ROS equivalence.
11. **Keep frame/TF contracts intact.** Wheel twist and EKF twist are body-frame;
    Vicon driver twist was validated as world-frame despite its child frame.
    The Vicon-free runtime intentionally uses pose-delta world velocity. EKF
    owns `odom -> base_link`; camera localization owns `map -> odom`.
12. **Some ancillary tools are legacy.** `odometry_eval.cpp` expects `/robot_pose`
    as PoseWithCovarianceStamped and source-stamp synchronization; it is not the
    current `/local_HAMR/odom` analyzer. Its Procrustes rotation forms `U*V.T`
    for covariance `X*Y.T`, the inverse of the usual X-to-Y solution; it should
    not supply the tuning score without correction and validation.

## Existing verification

Tests were run before candidate filter integration, without launching hardware
or opening a serial port. `/usr/bin/python3` is required here: shell-default
Conda Python lacks the ROS/test dependencies, and subprocess script shebangs
also need `/usr/bin` ahead of Conda in PATH.

| Test group | Result |
|---|---|
| Offline covariance/profile, firmware analyzer, study analyzer, waypoint analyzer, study plan, Vicon delivery configuration, SLAM frame math | 114 passed; 1 failed because `hamr_HW_waypoint_vicon_unprotected.launch.xml` is absent |
| Experimental kinematics and trajectories; odometry QoS with ROS sourced and package roots in PYTHONPATH | 48 passed |
| Camera controls/launch/recording, actuator plots, waypoint wrappers, split commands, study commands | 104 passed; 1 failed because `hamr_study_reference.launch.xml` is absent |

Total across these nonoverlapping groups: **266 passed, 2 existing missing-file
failures**. Root-level recursive collection also encounters unbuilt
`hamr_interfaces`, optional ament lint imports, and vendored upstream launch
tests; it is not a substitute for targeted package tests. Controller safety and
reference tests require the repository's generated ROS message package, and
were not represented as passing in this audit.

`hamr_bringup/CMakeLists.txt` installs configs/launches and selected analyzers;
the covariance preparation helper and any new standalone optimizer are not
automatically installed. `/rosbags/*` is ignored by Git except named existing
scripts, so new reproducible tools/reports need explicit ignore exceptions if
they should be retained in a commit. Raw bags and generated trials should remain
outside source tracking.
