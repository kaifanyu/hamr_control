# HAMR live diagnostics in Foxglove

Run the Foxglove desktop/web client on your laptop and the bridge on the Pi.
Use a Foxglove WebSocket connection to `ws://10.42.0.14:8765` (the Pi's address
when this setup was installed; substitute its current LAN IP if it changes).
Import `hamr_bringup/config/foxglove_hamr.json` using Foxglove's layout import.
An installed copy is under `$(ros2 pkg prefix hamr_bringup)/share/hamr_bringup/config/`.
Current Foxglove plans require a developer seat for direct live connections.

## Choose one bridge owner

Source the workspace in each Pi terminal:

```bash
source /home/hamr/hamster_ws/install/setup.bash
```

For **caster-only acquisition with visualization**, use:

```bash
ros2 launch hamr_bringup caster_encoders.launch.xml \
  record_bag:=false run_foxglove:=true \
  config_file:=/home/hamr/caster_encoders.yaml
```

This runs the two UDP receivers, the WebSocket bridge, and the passive monitor.
The operator-owned YAML contains the front-left board inventory; the repository
template does not. Front-right inventory remains unverified until its physical
MAC assignments are configured. The monitor reports that as a warning.

For **the full hardware session in one command**, enable the optional caster
receivers on `hamr_HW.launch.xml`:

```bash
ros2 launch hamr_bringup hamr_HW.launch.xml \
  use_orientation:=false use_mag:=false \
  controller_odom_topic:=/HAMR_base/odom \
  controller_xy_velocity_source:=odom_twist_world \
  controller_vicon_pose_guard_enabled:=true \
  run_caster_encoders:=true \
  caster_encoder_config:=/home/hamr/caster_encoders.yaml \
  record_bag:=true record_camera:=false \
  run_foxglove:=true
```

`run_caster_encoders` defaults to `false`; `caster_encoder_config` defaults to
`$HOME/caster_encoders.yaml`. The included caster launch starts both UDP
receivers in dual mode with its own recorder and Foxglove disabled. The parent
hardware launch owns one recorder (including caster topics) and one bridge/monitor.
Its recording and Foxglove flags remain independent of the caster flag.

Stop any standalone caster receivers before enabling this flag: each UDP port
must have one receiver. To keep separate terminals instead, leave
`run_caster_encoders:=false` on hardware and run `caster_encoders.launch.xml`
with `record_bag:=false run_foxglove:=false` and your `config_file`.

Hardware launch also starts the motor/serial stack and localization. Vicon is
provided by its separate existing driver/host. A trajectory/reference publisher
is also separate. The caster-only launch starts neither of those stacks.

For **visualization independent of hardware launch lifetime**:

```bash
ros2 launch hamr_bringup foxglove.launch.xml
```

Then use `run_foxglove:=false` on both acquisition/hardware launches. If an older
bridge is already running, start only the monitor with
`ros2 launch hamr_bringup foxglove.launch.xml run_bridge:=false`.
Run one bridge per port and one HAMR monitor per ROS domain. The bridge sees
all discoverable topics in its ROS domain, including topics started later.
It does not start missing publishers or cross ROS domains by itself.

## Dashboard contents

- **Health:** `/hamr/diagnostics` overview, front-left/front-right filters,
  and the existing `/diagnostics` feed (including EKF diagnostics when enabled).
  Click an entry for receipt age, measured Hz, counters and caster flags.
- **Caster values:** H1, H2 and CENTER positions and velocities for each wheel.
  Units are radians and radians/second. These are different physical axes;
  the plots do not imply that their angles should match.
- **Odometry + IMU:** local EKF position, Vicon position, acceleration and gyro.
  Local odometry and Vicon are shown separately because their coordinate frames
  and origins differ. Alignment is required before interpreting their difference.
- **Drive encoders:** left/right/turret ticks, wheel odometry velocity and raw
  controller/encoder diagnostic arrays.
- **Pi + raw diagnostics:** original caster diagnostic JSON, IMU calibration,
  and the bridge's CPU/memory system information.

Every ordinary ROS topic is available in the Topics sidebar, including custom
messages, TF, commands, metadata, and camera topics if their drivers publish them.
Add a Plot, Raw Messages, Image or 3D panel as needed. The supplied layout does
not request camera images, point clouds or high-rate raw caster packet JSON.
The bridge is configured for observation (connection graph and assets), without
client publication or service calls.

## What health means

The monitor publishes `diagnostic_msgs/msg/DiagnosticArray` at 2 Hz and uses
monotonic **local receipt time**, so an unsynchronized Vicon source clock does
not make the stream falsely stale. Each expected stream exists in the overview
even if it has never published. Zero readings remain valid readings.
The native C++ monitor counts serialized receipts at sensor rate and decodes
only the latest measurement at each dashboard update. Short faults between
updates may therefore only appear in the original sensor stream/recording.

- **OK:** messages arriving with finite displayed measurements, plus current
  firmware validity/verification checks for casters.
- **WARN:** caster diagnostics unavailable, or receiving but inventory, PPR,
  rate bounds or angle continuity are unverified. Details explain the reason.
- **ERROR:** non-finite/empty measurements or a caster firmware sample fault.
- **STALE:** never received, no recent messages, or no advancing caster samples.

Click a caster entry to inspect `sample_valid`, parity/SPI faults, continuity,
PPR verification, source IP, device ID, rejected-packet count and firmware
counters. Counters are cumulative; historical counts alone do not make every
future valid sample an error. Metadata/raw packets remain available separately.

The 1-second default stream timeout is a dashboard setting, **not** a motor
controller safety deadline. IMU calibration/control diagnostic arrays allow
3 seconds; caster diagnostic reports allow 2.5 seconds. The panel itself marks
entries stale after 3 seconds without monitor messages. Missing hardware is
expected to show STALE during caster-only use. A fresh EKF output does not
prove every EKF input is fresh; inspect its inputs too. Receipt Hz does not
prove the underlying sensor updates at that frequency (the IMU can repeat
measurements). This monitor is for live operation, not bag-time health analysis.

Edit `hamr_bringup/config/foxglove_monitor.yaml` to add expected typed streams
or change dashboard timeouts. Restart the monitor after changes. All other
topics remain available through the bridge without adding them to this list.

## Pi load

The laptop does the plotting. The Pi still handles ROS subscriptions, message
forwarding and network traffic. Discovery of all topics does not mean forwarding
all messages: clients request the topics needed by their panels. The monitor
itself continuously subscribes to the listed small telemetry streams and the
two 1 Hz caster reports, even when no Foxglove client is connected.

The bridge uses two executor threads, bounded queues/buffers and best-effort
subscriptions for the main high-rate telemetry. These settings limit buffering;
they are not a CPU cap, stream rate limiter or guarantee of no latency. Multiple
clients, raw images, point clouds and raw packet JSON can add substantial load.
Start with the Health tab and add plots gradually. Monitor CPU/memory and receipt
rates during an actual hardware run. Wi-Fi streaming shares bandwidth with the
ESP telemetry, so use a wired Pi-to-laptop/network link when practical.

A synthetic loopback test on this Pi 5, using six 200 Hz caster streams and
the other telemetry streams (about 2,000 messages/s), measured:

| Process | CPU, percent of one core | Resident memory |
|---|---:|---:|
| Foxglove bridge while streaming | 29.0% | 49.8 MiB |
| C++ health monitor | 17.8% | 31.3 MiB |

Combined CPU was about 12% of the four-core capacity; WebSocket payload was
about 461 KiB/s. With no telemetry/client, combined CPU was about 1.4% of one
core. These figures cover these two processes, excluding the simulated source,
client, and other applications. This is not an actual control/camera/Wi-Fi
latency benchmark. See [validation results](assets/foxglove_validation.json).

## Validation and references

Health logic and launch tests:

```bash
colcon test --packages-select hamr_bringup --ctest-args -R '^(topic_health|foxglove_monitor)$' --output-on-failure
```

No hardware controller or motion is needed to test the standalone bridge.
Live sensor accuracy and timing must be checked with the relevant drivers running.

- [Foxglove ROS 2 connection guide](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2)
- [Diagnostics panel](https://docs.foxglove.dev/docs/visualization/panels/diagnostics)
- [Bridge configuration](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md)
- [Live connection options](https://docs.foxglove.dev/docs/visualization/connecting/live)
