# Real-environment mapping capture

This is the hardware runbook for recording a D455 RGB-D trajectory and building
an RTAB-Map database with the current encoder-v4 stack.

## What changed with encoder v4

No RTAB-Map image, depth, IMU, or TF topic changed. The new firmware/relay still
publishes cumulative physical-left and physical-right ticks on
`/left_wheel/encoder_ticks` and `/right_wheel/encoder_ticks`; the calibrated
wheel node still publishes `/wheel_odom`, and the EKF still publishes
`/local_HAMR/odom`. RTAB-Map therefore needs no topic remap or algorithm change.

The encoder change does matter in three places:

1. Firmware protocol v4, the current `hamr_uros_bridge`, and the current
   `hamr_odometry` parameters must be deployed together. The wheel calibration
   now uses `yaw_sign=+1`, `left_tick_scale=1.001460`, and
   `right_tick_scale=1.000044` in `hamr_HW.launch.xml`.
2. New `/wheel_control/status` and `/wheel_encoder/diagnostics` topics are useful
   evidence that the new counts and wheel-speed loop behaved correctly. The
   COMPA SLAM recorder now includes them.
3. A database built from the old asymmetric scales/sign contains old odometry
   priors. Preserve it for comparison, but build a new database from a new bag
   before relying on localization.

The Vicon-specific controller defaults are also unsuitable when the controller
is fed `/local_HAMR/odom`: local odometry has planar `z=0`, and its twist follows
the normal child/base-frame convention. `localization_runtime.launch.py` now
disables the Vicon geometric guard and selects pose-delta world velocity when it
switches the controller to local EKF odometry.

## 1. Prepare and build

Use the matching encoder-v4 firmware. The firmware prints a build identifier on
its USB log port; the Pi relay and firmware packet `VER` must both be `4`.

Measure the fixed transform from `base_link` to the RealSense `camera_link` in
metres and radians in the robot's **actual deployed `base_link` axes**. Do not
use the launch defaults unless they are the measured values. If the camera is
on a gimbal, mechanically lock it for the whole mapping and localization run
and measure the locked pose relative to `base_link`.

The present hardware odometry uses `base_yaw_offset=pi/2`: with initial yaw
zero, equal positive wheel ticks produce `linear.y > 0`, not `linear.x > 0`.
That convention is shared by the existing holonomic controller and should not
be changed as part of an encoder/SLAM update. It does mean that a camera mounted
in the physical wheel-forward direction may require a translation along
`base_link` +Y and a yaw offset. Measure the full transform; do not label a tape
measure direction as +X merely because it is physically forward.

There are also undocumented geometry differences to resolve before precision
autonomous operation. The controller uses `r_wheel=0.122 m` and
`b_wheel=0.301 m`; wheel odometry uses `r_wheel=0.125 m` and
`b_wheel=0.45 m` (`a_wheel=0.35 m` agrees). Firmware also defines 0.125 m,
although its current radians-per-second command mode does not use that radius.
Manual mapping (`run_controller:=false`) is unaffected by the controller values, but saved-map
localization plus autonomous control is not. Measure the loaded rolling radius
and define which physical control point `base_link`/`b_wheel` represents. Keep
the values different only if that frame distinction is intentional and
documented; do not guess which values to overwrite.

From a clean terminal:

```bash
cd ~/hamster_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to compa_slam hamr_teleop --symlink-install
source install/setup.bash
```

Make sure `/dev/ttyUSB0` is the robot data UART and the D455 is on USB 3. Check
free disk space before starting. Uncompressed 640x480x30 RGB plus depth can
approach 3 GiB/min before overhead; lower numbers usually mean frames were
dropped, not that the capture became more efficient.

## 2. Start the robot, camera, and recorder

Choose a new bag name and substitute the six measured mount values:

```bash
ros2 launch compa_slam record_trajectory.launch.py \
  bag_name:=map_encoder_v4_01 \
  run_controller:=false run_foxglove:=false pointcloud:=false \
  use_orientation:=true use_mag:=false \
  mount_x:=MEASURED_X mount_y:=MEASURED_Y mount_z:=MEASURED_Z \
  mount_roll:=MEASURED_ROLL mount_pitch:=MEASURED_PITCH mount_yaw:=MEASURED_YAW
```

`run_controller:=false` is intentional for manual mapping. Otherwise the
100 Hz automatic controller and manual teleop both publish wheel commands.
`run_foxglove:=false` and `pointcloud:=false` leave CPU and I/O bandwidth for
the RGB/depth bag; RTAB-Map reconstructs its cloud during replay.

If the robot stack is already running, avoid duplicate relay/EKF/TF publishers:

```bash
ros2 launch compa_slam record_trajectory.launch.py \
  robot:=false bag_name:=map_encoder_v4_01 \
  run_foxglove:=false pointcloud:=false \
  mount_x:=MEASURED_X mount_y:=MEASURED_Y mount_z:=MEASURED_Z \
  mount_roll:=MEASURED_ROLL mount_pitch:=MEASURED_PITCH mount_yaw:=MEASURED_YAW
```

## 3. Do a stationary preflight

Before driving, leave the robot stationary for about 5 seconds and check:

```bash
ros2 topic hz /d455/color/image_raw
ros2 topic hz /d455/depth/image_rect_raw
ros2 topic hz /local_HAMR/odom
ros2 topic hz /left_wheel/encoder_ticks
ros2 topic echo /wheel_encoder/diagnostics --once
ros2 topic echo /wheel_control/status --once
ros2 topic echo /wheel_odom --once
ros2 topic echo /local_HAMR/odom --field pose.covariance --once
ros2 topic echo /local_HAMR/odom --field twist.covariance --once
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

Aim for 30 Hz RGB and depth and about 50 Hz local odometry. Do not start a map
run below 15 Hz RGB/depth; fix USB/CPU/storage load first. The TF command must
show one continuous, measured chain from the robot base to the image frame.
The first pose- and twist-covariance entries must be finite and well below
9999. The capture launch deliberately selects `ekf_orientation.yaml`, which
observes both body-velocity components. Do not override `use_orientation:=true`
with the older `ekf_mag.yaml` preset: that preset leaves one velocity state
unobserved and its covariance became pathological in recorded tests.

With the wheels safely lifted, make one very slow wheel-forward test. Both
physical wheel tick topics must increase. Under the current `pi/2` kinematic
offset, `/wheel_odom.twist.twist.linear.y` should be positive and X should be
near zero for equal ticks. During a physical counter-clockwise turn, yaw must
increase. Then move about one metre on the floor and confirm the local-odometry
distance and direction agree with the robot and the measured camera transform.
Stop and fix polarity, axes, or calibration before mapping if any check fails.

## 4. Drive a mapping trajectory

The existing keyboard teleop can be bounded to a capture-friendly speed. Run it
in a second sourced terminal:

```bash
ros2 run hamr_teleop teleop_node --ros-args \
  -p max_rpm_cmd:=10.0 \
  -p mix_scale:=0.8 \
  -p publish_rate_hz:=50.0 \
  -p stale_timeout_s:=0.75
```

Use `W/S` for forward/reverse, `A/D` for turns, and Space for an immediate stop.
At these settings, a straight command is approximately 0.84 rad/s at each wheel,
or 0.105 m/s with the configured 0.125 m wheel radius. A pure turn is about
0.30 rad/s. The short stale timeout requires continued key input and prevents a
lost terminal from leaving a long-lived motion command.

A good room trajectory is:

1. Start facing a feature-rich view and remain still for 5 seconds.
2. Drive the outside perimeter once, keeping visible structure about 0.7-3 m
   away and making slow, rounded corners.
3. Cross the interior in two or more overlapping lanes rather than recording
   only the walls. Space adjacent lanes so the camera still sees substantial
   overlap, typically 1-2 m in an indoor room.
4. Revisit major intersections and the starting view from the same direction.
5. Repeat the main loop in the same direction so revisited images have strong
   visual overlap. An opposite-direction partial loop is useful afterward, but
   it is not a substitute for the same-view revisit.
6. Return to the starting pose and heading and remain still for another 5 seconds.

Prefer 0.10-0.15 m/s translation and 0.20-0.35 rad/s rotation. Up to 0.20 m/s is
reasonable only while RGB and depth remain at least 15 Hz and images are sharp.
Avoid long in-place spins, abrupt reversals, featureless-wall closeups, moving
people filling the image, and pointing the depth camera into direct sunlight.

For a constant straight command without keyboard teleop, publish each wheel at
at least 10 Hz (the relay command watchdog is 0.25 seconds):

```bash
ros2 topic pub --rate 20 /left_wheel/cmd_vel std_msgs/msg/Float64 "{data: 1.0}"
ros2 topic pub --rate 20 /right_wheel/cmd_vel std_msgs/msg/Float64 "{data: 1.0}"
```

Those are two concurrently running terminal commands. Stop both and publish
zero before changing manoeuvres. A one-hertz default `ros2 topic pub` is not a
valid drive source because the relay will repeatedly time out between messages.

## 5. Stop and validate the bag

Press Space, stop the teleop, then press Ctrl-C once in the launch terminal. Wait
until rosbag reports that it has closed the MCAP file before powering down.

```bash
ros2 bag info ~/hamster_ws/src/hamr_control/rosbags/map_encoder_v4_01
ros2 run compa_slam analyze_rosbag.py \
  ~/hamster_ws/src/hamr_control/rosbags/map_encoder_v4_01
```

Confirm nonzero counts for RGB, depth, camera info, `/local_HAMR/odom`, both
wheel tick topics, `/tf`, `/tf_static`, `/wheel_control/status`, and
`/wheel_encoder/diagnostics`. Divide each message count by bag duration to check
the achieved rate. In `analysis_summary.txt`, review the encoder-v4 sections:
the full-quadrature/legacy ratios should be near 2, invalid-transition counters
should not grow rapidly, target-tracking error should be bounded, and sustained
wheel saturation should be investigated. Duplicate-sample callbacks are filtered
before they can become legacy ticks, but large or strongly asymmetric growth is
still evidence to inspect encoder wiring, pull-ups, and interrupt behavior.
Review the generated camera samples and trajectory plots before building a map.

## 6. Build a new RTAB-Map database

Use a new database filename so the old map remains recoverable. Mapping mode
passes `--delete_db_on_start` and will erase an existing target database. On the
Pi, leave the GUI off and replay at half speed if necessary:

```bash
ros2 launch compa_slam rtabmap_real.launch.py \
  bag:=$HOME/hamster_ws/src/hamr_control/rosbags/map_encoder_v4_01 \
  database_path:=$HOME/hamster_ws/src/hamr_control/compa_slam/maps/compa_real_encoder_v4.db \
  rate:=0.5 use_rtabmap_viz:=false use_odom_topic:=true
```

The compatibility default `use_odom_topic:=false` makes RTAB-Map read the
recorded `odom -> base_link` TF with provisional fixed variances. It salvages
older bags whose EKF preset left one body-velocity state unobserved: both pose
and twist covariance eventually crossed RTAB-Map's reset threshold even though
the TF remained continuous. Current capture selects `ekf_orientation.yaml`,
which observes both velocity components, so `use_odom_topic:=true` is preferable
after the analyzer reports zero non-finite samples and zero reset-guard
crossings for the *whole* new bag.
If a new capture fails that check, fix the EKF and recapture; the TF fallback is
for recovering historical data, not for declaring a bad new covariance model
healthy.

On a desktop, `use_rtabmap_viz:=true` makes loop closures easy to inspect. Let
the bag finish, then Ctrl-C the launch cleanly. A usable run should add map nodes,
accept visual loop closures when revisiting areas, and leave a nonempty database.
Do not work around rejected loop closures by loosening graph-error thresholds
until the measured camera transform, camera rate, encoder polarity, and local
odometry direction have all been verified.

Inspect the result and export a cloud if needed:

```bash
rtabmap-info \
  ~/hamster_ws/src/hamr_control/compa_slam/maps/compa_real_encoder_v4.db | tail -n 35
rtabmap-databaseViewer \
  ~/hamster_ws/src/hamr_control/compa_slam/maps/compa_real_encoder_v4.db
rtabmap-export --cloud \
  --output ~/hamster_ws/src/hamr_control/compa_slam/maps/compa_real_encoder_v4_cloud \
  ~/hamster_ws/src/hamr_control/compa_slam/maps/compa_real_encoder_v4.db
```

Look for nonzero `GlobalClosure` and/or `LocalSpaceClosure` counts and inspect
the optimized graph and cloud for doubling, bending, or discontinuities.
