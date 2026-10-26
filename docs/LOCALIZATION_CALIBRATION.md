# Encoder/IMU calibration run

`localization_calibration` records useful excitation for wheel calibration,
gyro-bias/freshness checks and offline EKF covariance tuning. It uses the existing
Vicon hardware controller while the wheel/IMU EKF runs independently. Vicon is
ground truth for fitting/evaluation and is not fused into the evaluated EKF.

Collect the missing fast-speed data first, using a separate short run for each
repeat. Select the pass with `--speed-set`:

| Selection | Requested straight speed | Total including initial/final holds |
| --- | --- | --- |
| `--speed-set fast` | 0.15 m/s | **2 minutes 40 seconds** |
| `--speed-set slow` | 0.10 m/s | **3 minutes 22 seconds** |
| `--speed-set both` | 0.15, then 0.10 m/s | **5 minutes 45 seconds** |

The shipped default is now **fast first, then slow**. The explicit single-speed
commands are recommended for repeats because each gets its own stationary
initialization, final hold, and bag. Every invocation performs:

1. 10 seconds holding the captured starting XY position.
2. At the selected speed: 1.5 m forward, return to the start, a rounded
   counterclockwise loop, then a rounded clockwise loop.
3. If `both` is selected, the same four movements at the second speed.
4. 10 seconds holding the final position, then reference silence and exit.

There is a 3-second zero-velocity hold between movements. Velocity and
acceleration are continuous at segment joins, with nominal reference bounds of
0.10 m/s² and 0.20 m/s³. Reference wheel speed stays below the controller's
28 RPM limit, reserving authority for tracking corrections. Actual chassis
motion, slip and wheel acceleration must be evaluated from the recording.

The reference footprint is **1.5 m in X by 2 m in Y**, at offsets
`X=[-1.5,0], Y=[0,2]` from the captured starting position. Allow additional room
for the whole robot and tracking error. Axes remain the Vicon world axes.
Start with raw Vicon base yaw near zero (within 0.15 rad / 8.6 degrees).
Reference yaw controls the turret; the XY loops produce base rotation.
"Reverse" describes the XY reference; measured wheel/chassis motion determines
whether a particular interval is usable as a pure backward calibration leg.

## Start the recording and localization

Source the workspace in each terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/hamster_ws/install/setup.bash
```

Keep the Vicon driver running. In terminal 1, start a fresh hardware stack and
a named bag:

```bash
HAMR_BAG_NAME=hamr_hw_calib_fast_01 \
ros2 launch hamr_bringup hamr_HW.launch.xml \
  record_bag:=true record_camera:=false use_mag:=false use_orientation:=false \
  controller_odom_timeout_s:=0.20
```

This retains the current wheel calibration and EKF configuration. The recorder
prints the new bag directory. This trajectory translates its reference to the
current Vicon XY, so it does not command a jump to the arena origin. This
translation does not reset the EKF. Keep the same calibration for all repeats.
Camera recording is disabled for this collection to reduce CPU/disk load; the
Vicon, encoder and IMU topics are still recorded. This may reduce scheduling
pressure but does not guarantee that Vicon receipt gaps will disappear.
The `controller_odom_timeout_s:=0.20` override matches the calibration
publisher's 200 ms Vicon limit. Without it, the normal 120 ms controller
timeout would stop first, so the calibration preflight rejects that mismatch.

To align the EKF's initial XY/yaw with Vicon, keep the robot stationary and run
the existing initialization helper in terminal 2 before the trajectory:

```bash
python3 ~/hamster_ws/src/hamr_control/rosbags/hamr_hw_20261024_195358/analysis/capture_start_pose.py
ros2 service call /set_pose robot_localization/srv/SetPose --stdin < /tmp/hamr_ekf_start.json
```

Require successful capture and a service response before proceeding. This is
one initial pose assignment; Vicon is not subsequently fused into the EKF.

In terminal 2, invoke the new reference publisher:

```bash
ros2 run reference_trajectory localization_calibration --speed-set fast
```

That command requests **one run**. It captures the running controller, wheel
odometry, relay and EKF parameters, waits for the controller/recorder and stable
stationary Vicon, then automatically holds for **10 seconds before motion**.
There is no need to time that wait yourself. It logs each motion/hold stage.
Do not run another waypoint/reference publisher concurrently.

Wait for `Calibration complete, including final hold`, then stop terminal 1 to
finish the bag. If feedback becomes stale or invalid, the reference publisher
aborts and stays stopped; it does not skip ahead or restart automatically.
The controller's existing watchdog stops the wheels on reference loss.
The trajectory and the above controller launch use a **200 ms** Vicon feedback
timeout. Reference callback gaps still have an independent **80 ms** limit,
and the controller's reference
watchdog is **120 ms**. If a run aborts, save that bag, then make another
explicitly started attempt; do not treat the partial bag as a completed repeat.

To stop early, Ctrl-C terminal 2, or use:

```bash
ros2 service call /localization_calibration/stop std_srvs/srv/Trigger '{}'
```

## Collect independent repeats

1. Finish the fast run and its final hold. Stop terminal 1 with Ctrl-C and wait
   for the recorder to finish closing the bag.
2. With the hardware stack stopped, return the robot to the same starting
   position and orientation. Raw Vicon base yaw must again be near zero. The
   route returns to the starting XY, but does not restore the base heading.
3. Relaunch terminal 1 with `HAMR_BAG_NAME=hamr_hw_calib_fast_02`, then run the
   same `--speed-set fast` command in terminal 2. Repeat for `fast_03`.
4. Collect slow repeats the same way: name the bags
   `hamr_hw_calib_slow_01`, `hamr_hw_calib_slow_02`, `hamr_hw_calib_slow_03` and
   use `--speed-set slow`. Each takes about 3 minutes 22 seconds.

Each fresh hardware launch resets wheel odometry and the EKF. If using the
Vicon-to-EKF initialization helper, apply it before each trajectory, following
the same procedure each time. Keep surface, payload, battery conditions, and
hardware/calibration settings as consistent as practical. The recorder prints
the actual output path and appends a suffix if a name already exists.

The trajectory never automatically loops or retries an aborted run. Changing
the speed selection does not change the current wheel calibration, EKF noise
matrices, IMU settings, or watchdog thresholds.

## Timing aborts

`Odometry became stale` means the trajectory node has gone more than 200 ms
without a processed Vicon pose on `/HAMR_base/odom`. This is the trajectory's
feedback, not `/local_HAMR/odom` from the EKF. A receipt/source timestamp gap
can also abort when the next pose arrives. `Reference callback gap exceeded
timeout` means trajectory execution missed its own 80 ms deadline. These
conditions require investigating delivery or scheduling; changing EKF Q/R
does not correct them.

The calibration profile's `odom_timeout_s` was raised to 0.20 seconds
on October 25. This applies to feedback age, receipt gaps and source-stamp
gaps, and is recorded in each run's metadata. Runtime validation requires
the controller's odometry timeout to be at least as long. The ordinary
`waypoint_traj_simple` publisher has no additional Vicon freshness check;
with `hamr_HW.launch.xml`, its controller defaults to 120 ms unless overridden.
The calibration launch above explicitly selects 200 ms. This tolerates the
observed 97 ms and 117 ms interruptions, but does not prevent delays or allow
gaps longer than 200 ms. Earlier bags retain their recorded timeout settings.

Timeout reasons include the measured duration and configured limit. Status
messages also record `last_graph_query_ms` and `last_reference_callback_ms`
(callback work through reference publication, excluding status publication).
They help distinguish time spent querying the ROS graph from a delay between
callbacks. Freshness is evaluated after graph queries, so a blocking query
cannot authorize a reference using the time from before the query. Clock
reversal is reported separately from an ordinary delayed callback; the older
combined wording did not establish that the clock actually reversed.

For the October 24 fast attempt `hamr_hw_calib_fast_01`, the recorder saw a
117 ms Vicon gap at the abort, after the forward and return legs. The next
bag, `hamr_hw_calib_fast_01_001`, contains a **slow** attempt despite its name;
it stopped during the initial hold, with a 339 ms gap between recorded
references. Other feedback streams continued around that second abort.
See each bag's `analysis/timing_audit.json`. Recorder timestamps establish
the visible interruptions, but do not identify the responsible driver,
transport, middleware call, or scheduling delay. Neither is a complete repeat.

## Preview and provenance

Preview without ROS publication or hardware access:

```bash
ros2 run reference_trajectory localization_calibration \
  --speed-set fast --plan-only /tmp/hamr_localization_calibration_fast.json
```

The profile is `reference_trajectory/config/localization_calibration.yaml`.
Use `--config /absolute/path/to/profile.yaml` for a custom profile, both in
preview and live execution. The node verifies that planned geometry matches
the running controller, and that the EKF inputs are wheel and IMU only.
The `speed_order` profile parameter selects `[fast]`, `[slow]`, or both; an
explicit `--speed-set` overrides it. To reproduce the original slow-first
order, use `speed_order: [slow, fast]` in a custom profile. The chosen order and
all stages are saved in the bag metadata.

The standard hardware recorder includes:

- `/localization_calibration/metadata`: exact plan/stage schedule, starting
  placement, source hashes and effective runtime parameter snapshots.
- `/localization_calibration/status`: execution state and stage transitions.
- `/localization_calibration/path`: placed preview path.
- Existing Vicon, wheel odometry, IMU, calibration status, encoder ticks,
  actuator commands and onboard EKF output.

## Retuning from the resulting bag

Use the saved history in this order:

1. Check completion, Vicon validity, dropouts, wheel clipping and actual IMU
   payload refresh. Identify truly stationary samples from Vicon; a requested
   hold alone does not prove zero motion.
2. Fit wheel linear/turn scales and the effective tracked-point lever arm.
   These October bags already contain calibrated body twists; do not apply the
   old September coordinate correction again.
3. Estimate gyro bias from confirmed stationary periods and compare beginning
   versus end. Check timestamp/measurement lag separately from noise.
4. Replay the EKF with candidate measurement covariances R and sensor choices;
   tune process noise Q only when residual evidence supports it. Keep initial
   covariance P0 distinct from motion calibration and sensor bias.
5. Score aligned XY/yaw errors, transients, endpoint error and stationary drift
   against Vicon. Evaluate full held-out repeats/routes as well as fitted runs.

One run supports diagnosis and a candidate fit. Record **three complete
repeats at each speed** with the same surface/load/settings, and use a separate
ordinary waypoint route as additional validation. The future bag is needed before reporting a new tuned
matrix; the trajectory command does not automatically modify the live EKF.

Wheel+IMU calibration can substantially improve local odometry. It does not
provide an absolute XY reference or guarantee bounded long-term global drift.

## October 25 six-run analysis

All six complete runs were analyzed; the usable first fast recording ends in
`fast_01_003`. Fast/slow runs 1 and 2 were used for fitting, and both run 3
recordings were held out. Lowering the effective gyro Z measurement variance
from 0.04 to 0.001 reduced held-out XY RMSE from 55.6 to 28.6 cm (fast) and
49.3 to 21.9 cm (slow). Actual ROS EKF replay confirmed the result.

On October 26, this weighting was promoted to the **default hardware profile**
at the user's request for a new ordinary waypoint validation run.
`hamr_bringup/config/hamr_uros_bridge.yaml` now sets
`imu_gyro_z_variance: 0.001`; no `bridge_config` override is needed. Restart
the hardware launch after sourcing the rebuilt workspace to load the change.
Apply the same stationary Vicon `/set_pose` initialization before the trajectory
and record the bag for comparison. Control still uses Vicon; the onboard EKF
is evaluated separately against it.

The tune regresses two older waypoint recordings, so the new run is needed
to assess its behavior on an ordinary waypoint path. The maintained wheel
calibration, Q, P0 and wheel covariances remain unchanged.

The default offline covariance profile,
`hamr_bringup/config/offline_calibrated_replay.yaml`, now matches this weighting.
The dated bridge and offline `calibration_20261025` profiles retain matching
named copies. These October inputs already contain wheel calibration; apply
covariance overrides only. The recorded baseline used gyro variance 0.04;
preserve that value when replaying the baseline for before/after comparisons.
See [the full report](../rosbags/ekf_tuning/calibration_20261025/REPORT.md) for
all six scores, plots, IMU freshness/lag findings, and the older-route checks.
