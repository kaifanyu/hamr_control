# Offline EKF replay

The calibrated configuration tested on all three September recordings is now the
default in `hamr_HW.launch.xml`. Its position RMSE is **4.9 / 10.2 / 6.7 cm**,
versus **41.2 / 48.7 / 61.2 cm** for the recorded EKF. See the
[full report and plots](rosbags/ekf_tuning/report/REPORT.md).

## Replay the tuned configuration on the three original bags

These recordings need a wheel-twist correction and calibration as well as new
covariances. The covariance-only helper below cannot apply that correction.
Prepared copies already exist under `rosbags/ekf_tuning/prepared/` in this workspace.
To create another copy, choose an output path that does not yet exist:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/para/hamr_control
/usr/bin/python3 rosbags/prepare_calibrated_replay.py \
  rosbags/hamr_hw_20260916_193228 \
  rosbags/calibrated_193228_new \
  --profile hamr_bringup/config/offline_calibrated_replay.yaml \
  --odometry-config hamr_bringup/config/wheel_odometry_calibration.yaml
```

The output contains `ekf.yaml`, reusable covariance and calibration snapshots,
and `calibrated_replay_manifest.json`. Original bags and timestamps are preserved.
Wheel pose is retained only for diagnostics and is not fused. Use that copied
bag as `$BAG` and its `ekf.yaml` with steps 3–6 below. Restart the filter for each
bag. The IMU/Vicon topics are unchanged; Vicon remains evaluation-only.

For **future recordings made with the new live calibration**, do not apply the
legacy correction again. Use `set_replay_covariances.py --config
hamr_bringup/config/offline_calibrated_replay.yaml` to make covariance-only trials.

To rerun numerical evaluation in seconds without ROS playback terminals:

```bash
bash rosbags/ekf_tuning/engine_build.sh
/usr/bin/python3 rosbags/ekf_tuning/extract.py \
  rosbags/hamr_hw_20260916_184054 \
  rosbags/hamr_hw_20260916_190842 \
  rosbags/hamr_hw_20260916_193228
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 rosbags/ekf_tuning/generate_report.py
```

This uses the actual upstream numerical EKF core. The winning result has also
been validated with the ROS node on all three recordings. See
[engine instructions](rosbags/ekf_tuning/engine_README.md) for node verification.

## Manual covariance-only experiments (original gyro baseline)

Edit **`hamr_bringup/config/offline_replay.yaml`** for each trial.
It retains the original baseline: wheel vx/vy + IMU gyro z. The separately named
`offline_calibrated_replay.yaml` selects the tuned wheel vx/vy/yaw-rate + gyro setup.

- `ekf.odom0_config`: which wheel measurements to use (`true` = use).
- `ekf.imu0_config`: which IMU measurements to use.
- `covariances`: the complete wheel and IMU measurement covariance matrices.
- Smaller diagonal = more trust; larger diagonal = less trust. Values are variances,
  not standard deviations. Keep positive diagonals and zero cross terms initially.
- `null` instead of a matrix preserves that matrix from the original bag.

The 15 selection entries have this order (also labeled beside each row in the file):

```text
x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az
```

For example, wheel yaw rate is the third entry in the fourth `odom0_config` row;
IMU orientation yaw is the third entry in the second `imu0_config` row.
With `two_d_mode: true`, z/roll/pitch and their derivatives are constrained to zero.
Enabling orientation requires usable, correctly aligned orientation in the bag.
Start with the existing velocity inputs; wheel pose and wheel velocity come from
shared encoder data, so enabling both can count the same information twice.
[ROS configuration reference](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst)

**1. Setup in all three Ubuntu ROS 2 Jazzy terminals.**

Use the updated repository. Adjust workspace and source bag paths to your machine:

```bash
source /opt/ros/jazzy/setup.bash
source ~/hamster_ws/install/setup.bash
cd ~/hamster_ws/src/hamr_control
export ROS_DOMAIN_ID=77
export SOURCE_BAG="/absolute/path/to/original_bag"
export TRIAL="$PWD/rosbags/trial_01"
export BAG="$TRIAL/input"
export RUN="$TRIAL/result"
```

Use the same unused domain in all terminals. Pick a new `TRIAL` name for every run.
The original bag needs `/wheel_odom`, `/imu/data`, and `/HAMR_base/odom` with nonzero
message counts. Check once with `ros2 bag info "$SOURCE_BAG"`.

**2. Edit the profile, then prepare the trial once in any terminal.**

```bash
python3 rosbags/set_replay_covariances.py "$SOURCE_BAG" "$BAG" \
  --config hamr_bringup/config/offline_replay.yaml
```

This creates a separate bag with the selected matrices, preserves measurements and
recording/header timestamps, and saves the matching **`$BAG/ekf.yaml`**. It also
saves a reusable `offline_replay.yaml` snapshot inside the copied bag. Original bag
and live source settings stay unchanged. No rebuild is needed.

The source profile is for the next trial: editing it does not update an existing
copy or a running EKF. Prepare a new trial after changing either selections or matrices.

**3. Terminal A: start the EKF using this trial's saved settings.**

```bash
ros2 launch hamr_bringup hamr_offline_ekf.launch.xml \
  ekf_config:="$BAG/ekf.yaml"
```

**4. Terminal B: record the new EKF and replayed Vicon.**

```bash
ros2 bag record --use-sim-time -s sqlite3 -o "$RUN" \
  /offline/local_HAMR/odom_ekf /offline/HAMR_base/odom
```

**5. Terminal C: replay.**

```bash
ros2 bag play "$BAG" --clock 100 --rate 0.5 --delay 3 \
  --topics /wheel_odom /imu/data /HAMR_base/odom \
  --remap /wheel_odom:=/offline/wheel_odom \
          /imu/data:=/offline/imu/data \
          /HAMR_base/odom:=/offline/HAMR_base/odom
```

Let playback finish. **Ctrl+C in B**, wait for saving to finish, then **Ctrl+C in A**.
Restart the EKF for every trial. The launch supplies the project's identity
`base_link -> imu_link` transform. Vicon is the comparison reference, not an EKF input.

**6. Terminal C: compare.**

```bash
python3 rosbags/analyze_hamr_vicon_straight.py "$RUN" \
  --base-topic /offline/HAMR_base/odom \
  --onboard-topic /offline/local_HAMR/odom_ekf \
  --source-bag-dir "$BAG" \
  --json "$RUN/metrics.json" \
  --localization-plot "$RUN/path.png" \
  --localization-error-plot "$RUN/error.png"
```

Read **Onboard localization vs Vicon**: compare XY RMSE, yaw RMSE, and final XY
error. Lower is better. Compare complete runs with similar matched sample counts.
Paths start at their own initial position/yaw, so this measures relative drift.
Use recordings beginning stationary and keep `--use-sim-time` on the recorder:
this analyzer matches bag timestamps.

**Simple tuning order:**

1. Run the unchanged profile as your baseline.
2. Change only the first two `wheel_twist` diagonal entries: try `0.001`, then `0.1`
   (default `0.01`). Keep the better wheel setting.
3. Change only the bottom-right `imu_angular_velocity` entry: try `0.00002`, then
   `0.002` (default `0.0002`).
4. Validate the winner on another bag, including straight motion and turns.
5. Test different measurement selections separately from covariance changes.

Leave process noise Q and initial covariance alone initially. They are inherited
from `base_ekf_config: ekf.yaml`, and can later be overridden under `ekf` using ROS
parameter names. Those 15x15 matrices use a flat row-major list of 225 values.
Measurement covariance R is the `covariances` section you normally tune here.
Covariance cannot correct wheel scale, frame/sign, timestamp, or gyro-bias errors.

**See actual bag values in Python assignment format:**

```bash
python3 rosbags/set_replay_covariances.py "$SOURCE_BAG" --show
python3 rosbags/set_replay_covariances.py "$BAG" --show
```

This shows the first input sensor covariance and checks whether it changes during
the recording. The EKF calculates its own output covariance separately.

**Apply changes to future live recordings:** the tested hardware launch loads
wheel calibration and twist variances from
`hamr_bringup/config/wheel_odometry_calibration.yaml`, gyro-z variance from
`hamr_bringup/config/hamr_uros_bridge.yaml`, and EKF selections/Q/P0 from
`hamr_bringup/config/ekf_calibrated.yaml`. Rebuild `hamr_odometry`,
`hamr_uros_bridge`, and `hamr_bringup`, then restart the live stack. The offline
profiles configure copied bags only.

Validation now includes all three source recordings, parameter searches,
leave-one-bag-out refits, actual ROS EKF-node replay, and original/copy integrity
checks. Complete evidence is in `rosbags/ekf_tuning/report/`.
