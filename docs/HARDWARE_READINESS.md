# Local hardware connection check

After starting `hamr_HW.launch.xml`, run this in a second sourced terminal:

```bash
source ~/hamster_ws/install/setup.bash
ros2 run hamr_bringup check_hamr_ready
```

The command observes ROS data for ten seconds, checks all six ball-caster
recorder boards, and prints `PASS`, `WARN`, or `FAIL` for each connection. It
requires no Foxglove, browser, or additional caster UDP receiver. It subscribes
to ROS topics, discovers recorders, and reads their status; it does not publish
motion commands, start/stop recorders, or erase/download their data.

## Use with onboard encoder recording

Keep `run_caster_encoders:=false` in this launch:

```bash
ros2 launch hamr_bringup hamr_HW.launch.xml \
  use_orientation:=false use_mag:=false \
  controller_odom_topic:=/HAMR_base/odom \
  controller_xy_velocity_source:=odom_twist_world \
  controller_vicon_pose_guard_enabled:=true \
  run_caster_encoders:=false \
  caster_encoder_config:=/home/hamr/caster_encoders.yaml \
  record_bag:=true record_camera:=false \
  run_foxglove:=false
```

Once the robot bag is recording:

```bash
ros2 run hamr_bringup check_hamr_ready
ros2 run hamr_bringup start_encoder_recording
ros2 run hamr_bringup check_hamr_ready
```

`start_encoder_recording` controls the ESP boards' onboard flash recording
directly. `caster_encoders.launch.xml` is only needed for live UDP-to-ROS
streaming. Live UDP is suppressed while boards record onboard, so requiring
live caster ROS topics during this workflow would give false missing-data
errors. The checker instead queries each board's recorder API.

Before start, idle boards can pass the availability check. Their API does not
expose live encoder measurements, and the output explicitly says sample
acquisition is not yet verified. After start, all six boards must report the
same run ID and increasing sample counts across successive observations.
Mixed idle/recording boards, frozen counts, identity changes, and storage or
firmware errors fail the check. Advancing counts demonstrate logging progress;
per-sample validity flags are inspected after download.

## What is checked

| Connection | Evidence |
| --- | --- |
| Vicon base | Fresh `/HAMR_base/odom` and `/HAMR_base/pose` |
| Vicon turret and caster forks | Fresh `/HAMR_turret/pose`, `/Left_Caster/pose`, `/Right_Caster/pose` |
| Local EKF | Fresh `/local_HAMR/odom`, plus its wheel-odometry and gyro inputs |
| Wheel odometry and IMU | Fresh `/wheel_odom` and `/imu/data`; finite measurements |
| Drive and turret encoders | Fresh left/right/turret `encoder_ticks` messages; stationary zero counts are valid |
| Drive diagnostics | Fresh `/wheel_control/status` and `/wheel_encoder/diagnostics` with finite, nonempty expected payloads |
| Six caster ESPs | Discovery, HTTP response, wheel/role/MAC identity, recorder state and storage capacity |
| Robot recorder | The latest robot bag resolves to exactly one active recorder process owned by the user |

ROS checks require an actual valid message stream, not just topic discovery.
Missing publishers, duplicate publishers, invalid poses, repeated/non-advancing
source stamps, and stale data fail. Freshness uses local monotonic receipt time,
so a Vicon computer with a different wall clock is supported. The displayed Hz
is the rate observed by this checker, not a calibrated sensor acquisition rate.
Unstamped tick messages establish receipt only; unchanged tick values while
stationary are expected.

The default age limits are one second for measurement streams and three
seconds for drive diagnostic arrays. These are connection checks, not the
controller's 120 ms stopping deadline or permission to move. Base height outside
the supplied launch's 0.25–0.40 m pose guard is shown as a warning. Magnetometer
and IMU orientation are not required for the supplied gyro-only EKF settings.

The checker uses the same `HAMR_BAG_ROOT` and recorder configuration defaults as
`start_encoder_recording`. If `~/caster_encoders.yaml` exists, it also enforces
that file's wheel/role MAC and IP pins without starting the UDP receivers.
Unpinned MAC identities produce a warning. To select explicit inventories:

```bash
ros2 run hamr_bringup check_hamr_ready \
  --caster-config /home/hamr/caster_encoders.yaml \
  --config /path/to/caster_recording.yaml
```

## Monitoring and saved results

```bash
# Continuous terminal display; Ctrl-C exits.
ros2 run hamr_bringup check_hamr_ready --watch

# Longer observation, optional storage-duration check, and JSON result.
ros2 run hamr_bringup check_hamr_ready \
  --timeout 15 --duration 60 --report /tmp/hamr_readiness.json
```

Use `--bag /absolute/active/bag` when selecting a bag explicitly. If broadcast
discovery is unavailable, repeat `--node front_left:H1@IP:8080` for all six
wheel/role endpoints, or set `--discovery-address` to your network's broadcast
address. Discovery uses UDP 5010 and does not bind the live telemetry ports.

Exit status is `0` when every connection passes (possibly with warnings), `1`
when any connection fails, `2` for setup/configuration errors, and `130` for
Ctrl-C. The JSON result includes the individual failures and a separate
`all_six_recorder_counts_advancing` field. In watch mode, `--report` is refreshed
alongside the terminal.

## Install or run from the checkout

After syncing the change to the robot:

```bash
cd ~/hamster_ws
colcon build --packages-select hamr_bringup --symlink-install
source install/setup.bash
ros2 run hamr_bringup check_hamr_ready
```

For immediate use before rebuilding, source the existing workspace and run:

```bash
python3 ~/hamster_ws/src/hamr_control/hamr_bringup/scripts/check_hamr_ready
```

If all ROS rows are missing, check the sourced workspace, `ROS_DOMAIN_ID`, and
network discovery. Vicon still needs its separate driver/host. A fresh EKF
output cannot compensate for missing inputs, which are checked separately.
