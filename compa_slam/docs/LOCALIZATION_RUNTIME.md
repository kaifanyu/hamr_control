# Vicon-free local control with RTAB-Map correction

This runtime deliberately keeps local control and global localization separate:

```text
wheel ticks + BNO055 -> robot_localization -> odom -> base_link
saved .db + D455    -> RTAB-Map          -> map  -> odom
map/route reference -> reference adapter -> odom ReferenceTraj -> controller
```

The EKF is the only publisher of `odom -> base_link`. RTAB-Map is the only
publisher of `map -> odom`. RTAB-Map is **not** fused back into the local EKF,
so a visual relocalization cannot make the controller's measured pose jump.

## Before running

1. Put the saved database somewhere persistent. The default is
   `~/.ros/compa_real.db`; an explicit absolute path is also accepted.
2. Measure the D455 mount and pass the six `mount_*` values. The defaults are
   placeholders.
3. The current real-camera launch publishes a static `base_link -> camera_link`.
   This is valid only if the D455 is fixed relative to the base. If it is mounted
   on a moving gimbal, lock the gimbal for mapping/localization or replace the
   static transform with a measured dynamic joint TF chain.
4. Do not run Vicon, a static `map -> odom`, or RTAB-Map visual odometry. They
   are not needed and would violate TF ownership.

Install the runtime dependencies on ROS 2 Jazzy if they are not already present:

```bash
sudo apt install \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-realsense2-camera \
  ros-jazzy-imu-filter-madgwick \
  ros-jazzy-robot-localization
```

Build and source:

```bash
cd ~/hamster_ws
colcon build --packages-up-to compa_slam --symlink-install
source install/setup.bash
```

## First run: localization and local odometry only

Keep the wheels off the ground or disable motor power for the first TF/topic
check. Start the whole Vicon-free stack without a trajectory:

```bash
ros2 launch compa_slam localization_runtime.launch.py \
  database_path:=$HOME/hamster_ws/src/hamr_control/compa_slam/maps/compa_real.db \
  mount_x:=MEASURED_X mount_y:=MEASURED_Y mount_z:=MEASURED_Z \
  mount_roll:=MEASURED_ROLL mount_pitch:=MEASURED_PITCH mount_yaw:=MEASURED_YAW
```

The launch does the following:

- starts the relay, encoder integration, BNO055 EKF, and hardware controller;
- remaps the controller's former `/HAMR_base/odom` Vicon input to
  `/local_HAMR/odom`;
- uses the same EKF topic for the turret world-yaw helper;
- starts the D455 and RTAB-Map in localization mode against the existing DB;
- starts the map/route-reference to local-odom adapter;
- does **not** start a waypoint source by default.

On a desktop with a display, add `use_rtabmap_viz:=true`. Leave it false on a
headless Raspberry Pi.

## Checkpoint tests

### 1. Confirm the controller no longer uses Vicon

```bash
ros2 node info /hamr_controller_node
ros2 node info /holonomic_odom_node
```

Both nodes' subscriptions should include `/local_HAMR/odom`, not
`/HAMR_base/odom`. It is fine for `/HAMR_base/odom` to have zero publishers.

### 2. Check the local EKF output

```bash
ros2 topic hz /wheel_odom
ros2 topic hz /imu/data
ros2 topic hz /local_HAMR/odom
ros2 topic echo /local_HAMR/odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

Expected `/local_HAMR/odom` message contract:

```text
header.frame_id: odom
child_frame_id: base_link
pose: smooth wheel/IMU-integrated local pose
twist: base-frame velocity estimate
```

It normally starts near `(x=0, y=0, yaw=0)` every time the EKF starts. It is
continuous and useful for control, but it accumulates wheel/IMU drift.

### 3. Check visual localization correction

```bash
ros2 topic hz /d455/color/image_raw
ros2 topic hz /d455/depth/image_rect_raw
ros2 topic list | grep localization_pose
ros2 topic echo /localization_pose --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

`/localization_pose` and `map -> odom` should appear after the current image is
matched to the database. `map -> odom` may move when RTAB-Map corrects drift;
`odom -> base_link` and `/local_HAMR/odom` must remain smooth.

If localization does not start, check the database path, RGB/depth rates and
QoS, the camera TF, and whether the current view overlaps the saved map. Move
the robot manually only after the stationary checks pass.

## Run `waypoint_traj_simple` with visual correction

Start the runtime without waypoints, wait until `/localization_pose` is updating,
then start the simple trajectory in another sourced terminal:

```bash
ros2 run reference_trajectory waypoint_traj_simple --ros-args \
  --log-level warn \
  -p v_lin:=0.10 \
  -r /reference_trajectory:=/reference_trajectory_map \
  -r /waypoints_path:=/route_waypoints_path \
  -r /traj_viz:=/route_traj_viz
```

The adapter treats the simple trajectory as a route and anchors its first
reference at the robot's current localized map pose and starting yaw. It then transforms every
moving reference through the latest `map -> odom` correction and publishes the
result on `/reference_trajectory`, in the same `odom` frame used by the
controller.

The hardcoded simple route is:

```text
(0,0) -> (0,3) -> (-2,3) -> (-2,4.5) -> (2,4.5)
      -> (2,3) -> (0,3) -> (0,0), yaw=0
```

It is 17 m long. At its original `v_lin=0.25 m/s`, one loop takes about 68 s;
at the safer first-test value above (`0.10 m/s`), it takes about 170 s. This
trajectory is time-based, so it keeps advancing even if the robot falls behind.

For one-command startup after the staged tests pass, add:

```bash
run_waypoint_simple:=true waypoint_v_lin:=0.10
```

to the localization launch. Manual startup remains preferable because it lets
you confirm visual localization before the moving reference begins.

`/route_waypoints_path` and `/route_traj_viz` are the simple source node's raw
route-coordinate visualization. They are separated from the controller topics
because the source messages have a hardcoded `odom` frame label; use the
corrected `/reference_trajectory` values and TF for controller-frame debugging.

### What correction looks like

Suppose the EKF starts at local `(0,0)` while RTAB-Map recognizes that the robot
is at `(8,-3)` in the saved map. RTAB-Map expresses that difference in
`map -> odom`; `/local_HAMR/odom` still begins at `(0,0)`.

As wheel drift accumulates:

- `/local_HAMR/odom` continues smoothly along its locally integrated path;
- `map -> odom` changes to compensate for the detected drift;
- `/reference_trajectory_map` continues describing the fixed route;
- the corrected `/reference_trajectory` shifts in `odom`, causing the local
  controller to steer back toward the globally fixed route.

The robot therefore does not teleport in its own odometry. The target moves
slightly in local coordinates when a global correction arrives.

The adapter will not release a corrected route until the first
`/localization_pose` arrives. After that, the default behavior is to continue
smooth local control with the last `map -> odom` correction if visual updates
temporarily stop; the EKF remains available even when the camera cannot match.

For a strict stop-on-visual-loss policy, launch with:

```bash
hold_on_stale_localization:=true localization_timeout_s:=2.0
```

That mode publishes a zero-velocity hold reference at the current local pose
when localization is stale. Because `waypoint_traj_simple` is time-based, stop
and restart it after a long localization loss so it does not resume at a much
later point in the route.

## Local-only waypoint comparison

To intentionally test only the EKF and controller, publish the simple trajectory
directly without remapping it:

```bash
ros2 run reference_trajectory waypoint_traj_simple --ros-args \
  --log-level warn -p v_lin:=0.10
```

In that mode both pose and reference are in `odom`, so tracking is smooth, but
RTAB-Map has no effect on the driven route. The complete path will be displaced
or rotated in the saved map as local drift accumulates. Never run the local-only
and map-corrected waypoint publishers simultaneously.

## Offline/unit checks

The frame math is ROS-independent and covered by tests:

```bash
colcon test --packages-select compa_slam
colcon test-result --verbose
```

The tests verify transform inversion, route anchoring, velocity rotation, and
that changing `map -> odom` moves the local goal without changing the local EKF
pose.
