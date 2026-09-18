# Offline EKF replay: edit one file

Edit **`hamr_bringup/config/offline_replay.yaml`** for each trial.
It starts with the current source defaults: wheel vx/vy + IMU gyro z.

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

**Apply a winning setup to future live recordings:** wheel measurement covariance
is in `hamr_odometry/hamr_odometry/holonomic_odom_node.py`; IMU covariance is in
`hamr_uros_bridge/src/relay_node.cpp`; EKF selections are in
`hamr_bringup/config/ekf.yaml`. Transfer the chosen settings, rebuild affected
packages, and restart the live stack. The offline profile is used only by the helper.

Validation: profile and synthetic-message checks run locally. A real ROS replay
still needs your ROS computer; this checkout has no bags and local WSL lacks rosbag2.
