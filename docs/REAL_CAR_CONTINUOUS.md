# Continuous 2 cm corner planning on the real HAMR

The separate entry point is:

```bash
ros2 launch hamr_bringup hamr_continuous_HW.launch.py
```

It starts the existing hardware controller, serial bridge, calibrated wheel
odometry, EKF and bag recorder, plus the new continuous reference publisher.
Launching it does **not** start the trajectory. The operator starts one run with
`/continuous_waypoint/start` and can end it with `/continuous_waypoint/stop`.

The planner is the same geometric and time-parameterization implementation used
by the ball-caster simulation. Real hardware uses its own geometry, limits and
existing controller gains. WSL validation does not establish real-car tracking
accuracy or the loaded motors' acceleration capability.

## 1. Build the hardware packages

On the computer connected to the car, use ROS 2 Jazzy and a separate colcon
workspace. The repository can remain at `~/hamr_control`:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-rosbag2-storage-mcap
# On a new system only, initialize rosdep once: sudo rosdep init
rosdep update
rosdep install --from-paths ~/hamr_control/hamr_bringup \
  ~/hamr_control/hamr_control ~/hamr_control/hamr_description \
  ~/hamr_control/hamr_interfaces ~/hamr_control/hamr_odometry \
  ~/hamr_control/hamr_uros_bridge ~/hamr_control/reference_trajectory \
  --ignore-src --rosdistro jazzy -y
```

Then build:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/hamr_hw_ws
cd ~/hamr_hw_ws
colcon build --symlink-install --base-paths ~/hamr_control \
  --packages-up-to hamr_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source ~/hamr_hw_ws/install/setup.bash
```

The hardware runtime needs `robot_localization`, `rosbag2_transport`, and MCAP
storage. Camera recording and Foxglove are optional and default to off in this
new launch. The existing hardware launch retains its original defaults.

Source `/opt/ros/jazzy/setup.bash` and `~/hamr_hw_ws/install/setup.bash` in each
operator terminal. Use the same `ROS_DOMAIN_ID` as the Vicon publisher and robot;
do not source the ball-caster simulation environment for a hardware run.
If this shell previously ran the simulator, clear its localhost-only discovery
setting and select the actual hardware domain before launching ROS processes:

```bash
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID=0  # Replace 0 if your Vicon/robot deployment uses another domain.
```

Retain your deployment's existing DDS peer configuration if it uses explicit
peers instead of subnet discovery.

## 2. Check the configuration and preview the plan

The profile is
[`reference_trajectory/config/continuous_waypoint_hw.yaml`](../reference_trajectory/config/continuous_waypoint_hw.yaml).
Its default relative waypoints are:

```text
(0,0) → (0,2) → (-2,2) → (-2,4.5) → (0,4.5) → (0,2) → (0,0)
```

At explicit start, `start_translated_vicon` translates this path to the accepted
measured starting XY position. The axes remain the Vicon world axes; the path
does not rotate with the chassis. The route spans 2 m in X and 4.5 m in Y, before
allowing room for the vehicle itself. The initial raw Vicon base yaw must match
`nominal_start_base_yaw_rad` within `start_yaw_tolerance_rad`.

| Setting | Hardware default | Meaning |
|---|---:|---|
| Straight speed | 0.15 m/s | Requested cruise speed; planning can reduce it |
| Corner deviation | 0.02 m | Maximum planned departure from the original straight segments |
| Cartesian acceleration | 0.10 m/s² | Reference-planning bound |
| Cartesian jerk | 0.20 m/s³ | Reference-planning bound |
| Wheel radius / half track / axle offset | 0.122 / 0.350 / 0.301 m | Must match the calibrated controller geometry |
| Wheel speed cap | 2.932153 rad/s (28 RPM) | Existing hardware controller/firmware ceiling |
| Planning wheel speed fraction | 0.65 | Reserves wheel-speed authority for feedback |
| Planning wheel acceleration | 2.0 rad/s² | Provisional reference limit, not a measured motor guarantee |

A 2 cm blend passes approximately 2.83 cm from each original 90° vertex. The
reference has continuous velocity and acceleration, bounded jerk, and positive
speed through its interior corners; it starts and finishes at rest. The real
vehicle can still develop tracking error or stop on a feedback fault.
The default hardware reference takes approximately 98.3 seconds of motion, plus
a 1-second startup hold and a 2-second final hold. This differs from the faster
simulation profile because the hardware geometry and reserved motor authority
are different.

Export the plan without launching nodes, opening serial ports, or sending
commands:

```bash
ros2 run reference_trajectory continuous_waypoint \
  --plan-only /tmp/hamr_continuous_hw_plan.json
```

For a changed profile, copy the YAML to an operator-owned path and use that same
file for both preview and launch:

```bash
ros2 run reference_trajectory continuous_waypoint \
  --config ~/hamr_continuous_hw.yaml \
  --plan-only /tmp/hamr_continuous_hw_plan.json

ros2 launch hamr_bringup hamr_continuous_HW.launch.py \
  planner_config:=$HOME/hamr_continuous_hw.yaml
```

Keep these existing hardware settings calibrated:

- [`hamr_hw_control_params.yaml`](../hamr_bringup/config/hamr_hw_control_params.yaml):
  real geometry, wheel speed cap, feedback gains, `base_yaw_offset`, turret
  availability and Vicon validity bounds. The current `base_yaw_offset=π/2`
  converts raw Vicon base yaw for the wheel Jacobian. Do not apply that rotation
  a second time to the path. `fixed_yaw_rad` is the turret reference; the turret
  is currently disabled in the hardware controller.
- [`hamr_uros_bridge.yaml`](../hamr_bringup/config/hamr_uros_bridge.yaml): serial
  device and baud; currently `/dev/ttyUSB0` at 460800 baud.
- [`wheel_odometry_calibration.yaml`](../hamr_bringup/config/wheel_odometry_calibration.yaml)
  and [`ekf_calibrated.yaml`](../hamr_bringup/config/ekf_calibrated.yaml): existing
  encoder/IMU calibration. This launch does not change the localization pipeline.

The wrapper checks planner geometry against the installed controller YAML and
rejects a mismatched profile before starting any hardware processes. The planning
limits bound the reference; feedback corrections can demand additional wheel
acceleration. Retain the current conservative profile until loaded motor behavior
and recorded tracking have been evaluated on the car.

## 3. Start the stack and verify readiness

Start the existing Vicon driver separately. This launch expects fresh
`nav_msgs/msg/Odometry` on `/HAMR_base/odom` and uses this same source for both
the planner and the hardware controller. The current controller expects XY
twist in Vicon/world coordinates, as provided by the existing driver.

Terminal 1:

```bash
export HAMR_BAG_ROOT="$HOME/hamr_recordings/bags"
export HAMR_BAG_NAME="continuous_tight_$(date +%Y%m%d_%H%M%S)"
ros2 launch hamr_bringup hamr_continuous_HW.launch.py
```

Terminal 2:

```bash
ros2 topic hz /HAMR_base/odom
ros2 topic echo /continuous_waypoint/status --once \
  --qos-reliability reliable --qos-durability transient_local
```

Leave the car stationary while the planner obtains stable odometry. Stop any
older `waypoint_traj_simple`, study trajectory, teleoperation or simulation
process sharing these command topics. The planner refuses to start with a
competing reference publisher. The hardware controller must be present before
the start service can succeed.

Readiness requires both 0.25 seconds of stable stationary feedback and at least
10 accepted recovery samples, matching the existing controller's recovery latch.
At lower Vicon delivery rates, collecting those samples can take longer than
0.25 seconds. This prevents restarting the reference before the controller is
ready to accept it.

Useful optional launch arguments:

```bash
# Include both caster cameras using the existing external rig configuration:
ros2 launch hamr_bringup hamr_continuous_HW.launch.py \
  record_camera:=true \
  camera_repository:=$HOME/Caster_Vision/ball_caster_dual_cam \
  camera_config:=$HOME/Caster_Vision/ball_caster_dual_cam/config/rig.yaml

# Include Foxglove, if installed:
ros2 launch hamr_bringup hamr_continuous_HW.launch.py run_foxglove:=true
```

Use one stack launch at a time. `record_bag`, `camera_segment_mib`,
`recording_root`, `ekf_config`, `wheel_odom_config`, `use_mag` and
`use_orientation` are also forwarded to the existing stack. `recording_root`
controls camera files; `HAMR_BAG_ROOT` controls bags. `odom_topic` can rename the
raw Vicon source for both consumers; it does not transform frames or make an EKF
topic equivalent to the validated Vicon contract.

## 4. Start, stop and replay a real run

Start one complete path explicitly:

```bash
ros2 service call /continuous_waypoint/start std_srvs/srv/Trigger '{}'
```

Read the service response. A refused start leaves the reference silent; correct
the reported readiness condition before trying again. To end the run:

```bash
ros2 service call /continuous_waypoint/stop std_srvs/srv/Trigger '{}'
```

Stopping or aborting ends reference publication. The existing controller's
120 ms reference watchdog then commands zero; the serial bridge has its own
250 ms stale-command timeout. This is a software stop, so retain the vehicle's
physical stop control during operation. The trajectory never automatically
resumes after a fault; inspect the status and explicitly start again after
recovery. Ctrl-C closes the launch and its recorder.

The bag includes references, measured odometry, wheel commands/encoders, and the
latched `/continuous_waypoint/metadata`, `/continuous_waypoint/status` and
`/continuous_waypoint/path`. Inspect it with:

```bash
ros2 bag info "$HAMR_BAG_ROOT/$HAMR_BAG_NAME"
```

If an output name already exists, the recorder appends an unused numeric suffix;
use the actual path printed by the recorder. Replay measured data for inspection
in an isolated ROS domain, with the real stack stopped:

```bash
ROS_DOMAIN_ID=179 ros2 bag play /path/to/the/recorded/bag \
  --topics /HAMR_base/odom /reference_trajectory \
    /continuous_waypoint/path /continuous_waypoint/metadata \
    /continuous_waypoint/status
```

This command intentionally selects observation/reference topics and omits motor
command topics. Set the same isolated domain for the visualization client.

## Timing, frames and URDF scope

Hardware always uses `use_sim_time=false`. Vicon source stamps are required to
be valid and increasing, but freshness is measured from local receipt because
the Vicon host clock is not synchronized. The planner's odometry and publication
gap limits are stricter than the controller's existing 120 ms watchdogs. A
stale/invalid pose or interrupted reference stream aborts the active trajectory.

The new CAD ball-caster URDF is documented in
[`hamr_ball_caster/README.md`](../hamr_ball_caster/README.md). It models the COMPA
simulation chassis retrofit, its rocker/gimbal links and two split ball casters.
The real controller currently uses separately calibrated kinematic dimensions;
the older `hamr_description` model has a different topology and dimensions. This
hardware launch does not insert a simulation robot-state publisher or synthetic
caster joint states into the real TF tree. Actual passive caster angles require
measurements if they are to be displayed as measured states.
