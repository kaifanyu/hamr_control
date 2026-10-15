# Caster encoder Wi-Fi recording and replay

The `caster-encoder-controller` firmware measures the ball caster's two shell
rotations (H1/H2) and common-rod rotation (CENTER). This integration receives its
existing version-1 UDP protocol; no firmware or micro-ROS conversion is required.
It adds passive telemetry to HAMR. The existing drive-wheel tick inputs, motor
control and EKF continue to use their existing sensors.

```text
H1 ESP ----\
H2 ESP ----- 2.4 GHz Wi-Fi / UDP 5005 --> recording computer
CENTER ESP-/                             caster_encoder_bridge
                                                |
                                /caster_encoder/* ROS topics
                                                |
                      existing HAMR recorder --> MCAP rosbag
                                                |
                  Foxglove / ROS playback / analyze_caster_encoders.py
```

## 1. Network and firmware

Run the ROS receiver on the computer whose reachable LAN IPv4 address is compiled
into each ESP's `RECEIVER_IPV4`. The ESPs initiate UDP delivery; the ROS receiver
does not need their individual IP addresses. `0.0.0.0` means listen on all local
interfaces, **not** an address to put in the ESP configuration.

In `caster-encoder-controller/config/local.h`, confirm:

- `WIFI_SSID` / `WIFI_PASSWORD`: the accessible 2.4 GHz network.
- `RECEIVER_IPV4`: the recording computer's LAN address, preferably reserved by DHCP.
- `UDP_PORT`: `5005`, matching the bridge parameter.
- Each physical board has the correct role: H1=1, H2=2, CENTER=3.

The private `local.h` is not in the supplied checkout. Inspect the configuration
used to flash your boards, or their startup logs, to establish the actual target.
If it already points to the ROS computer, keep the running firmware. If the
receiver address changes, rebuild and flash each role using that repository's
README. Follow its power/wiring instructions when connecting USB.

Allow inbound UDP 5005 from the ESP network in the receiver firewall and disable
AP/client isolation for this network. The default firmware samples at 200 Hz per
node and sends batches of two at approximately 100 datagrams/s per node.
Metadata repeats approximately every two seconds. The stream is best effort;
there is no on-device storage or recovery of samples lost during a Wi-Fi outage.

Only one receiver may bind the UDP port. Stop `host/logger.py` before starting
the ROS bridge. A second bridge fails explicitly if the address is in use.
On WSL, ESPs must be able to reach the Linux receiver through the Windows network
and firewall; a working localhost test does not establish LAN delivery. Native
Ubuntu on the robot/recording host is the simplest existing deployment path.

## 2. Build on the ROS recording computer

These commands assume your existing ROS 2 Jazzy HAMR workspace is
`~/hamster_ws`, with this updated repository under `src/hamr_control`:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/hamster_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select hamr_ball_caster hamr_bringup
source install/setup.bash
```

The package installs the bridge, its vendored decoder and the analyzer together.
The encoder firmware checkout is not required on the recording host. MCAP,
`rosbag2_py`, NumPy and Matplotlib are already declared package dependencies.
For a fresh workspace, first build the hardware dependencies using
[REAL_CAR_CONTINUOUS.md](REAL_CAR_CONTINUOUS.md#1-build-the-hardware-packages).

## 3. Collect live encoder data

For an encoder-only bench session, without starting motor control:

```bash
source /opt/ros/jazzy/setup.bash
source ~/hamster_ws/install/setup.bash
export HAMR_BAG_ROOT="$HOME/hamr_recordings/bags"
export HAMR_BAG_NAME="caster_$(date +%Y%m%d_%H%M%S)"
ros2 launch hamr_bringup caster_encoders.launch.xml
```

This starts the receiver and the existing test-bag recorder. Its topic filter also
captures the usual HAMR/Vicon topics if those are already available. The console
prints the actual bag directory; an existing name gets a numeric suffix.

For the regular hardware workflow, both `hamr_HW.launch.xml` and
`hamr_continuous_HW.launch.py` now enable the receiver by default. Use your usual
hardware command; its existing recorder owns the bag. The continuous workflow
still requires its explicit trajectory-start service described in
[REAL_CAR_CONTINUOUS.md](REAL_CAR_CONTINUOUS.md).

Available hardware launch overrides:

```bash
ros2 launch hamr_bringup hamr_continuous_HW.launch.py \
  caster_encoders:=true \
  caster_encoder_bind_address:=0.0.0.0 \
  caster_encoder_udp_port:=5005 \
  caster_encoder_stale_timeout_s:=1.0
```

Use `caster_encoders:=false` when the receiver already runs in a separate process.
For an existing hardware session that predates this change, start just the bridge:

```bash
ros2 launch hamr_bringup caster_encoders.launch.xml record_bag:=false
```

Restart/rebuild the bag recorder to apply its updated topic filter. Running only
`record_hamr_test_bag` or `record_hamr_vicon_bag` records ROS topics; those scripts
do not themselves open the ESP socket.

In another terminal, after sourcing the same ROS workspace:

```bash
ros2 topic echo /caster_encoder/diagnostics --once
ros2 topic echo /caster_encoder/metadata --once --qos-durability transient_local
ros2 topic hz /caster_encoder/H1/joint_states
ros2 topic echo /caster_encoder/CENTER/joint_states --once
```

Confirm all three roles are fresh and metadata has arrived before beginning a
trial. Approximately 200 Hz valid JointStates per node is expected at the default
sample rate; invalid measurements are deliberately absent from the display topic.
An unchanged angle alone does not prove an incremental encoder is connected.
Stop with Ctrl+C and let rosbag finalize, then check:

```bash
ros2 bag info "$HAMR_BAG_ROOT/$HAMR_BAG_NAME"
```

Use the printed suffixed path if the requested bag name already existed.

### Marked-rotation comparison: H1 versus H2

Use this check to distinguish reported scale from the number of physical turns.
Keep the existing bridge running and source the same ROS environment in another
terminal. The script subscribes to `/caster_encoder/packets`; it does not bind the
UDP port, command motors, change encoder settings, or need a firmware reflash.
It uses raw integer counts and ESP acquisition timestamps, not JointState receipt
timestamps, which can be identical for two samples in a packet.

From the updated `hamr_control` repository directory:

```bash
python3 hamr_bringup/scripts/record_caster_rotation.py record H1
python3 hamr_bringup/scripts/record_caster_rotation.py record H2
```

Run these sequentially. For each run:

1. Mark the encoder shaft's position relative to its housing and hold it still.
2. Wait for valid samples, then press Enter to set the start count.
3. Rotate the shaft exactly **one full revolution in one direction**. Measure
   shaft motion relative to the encoder housing, rather than whole-assembly motion.
4. Stop at the original mark, hold still for at least one second so the final
   counts arrive, then press Enter to finish. Ctrl+C saves an incomplete run and
   deliberately produces no calibration.

To improve the manual turn measurement, add `--turns 3` and make exactly three
complete revolutions. The count comparison is normalized by the entered turns.
Hand-rotation speeds do not have to match: compare counts per physical revolution,
not the two runs' peak velocities.

Each run creates a unique `recordings/caster_rotations/H1-...` or `H2-...` folder:

- `packets.jsonl`: complete received envelopes for the selected node, including metadata.
- `samples.csv`: raw counts, angles, velocities, acquisition and receipt times,
  flags, index snapshots, and diagnostic counters; waiting samples are retained too.
- `markers.json`: exact start and end sample identities and entered physical turns.
- `summary.json` and `summary.txt`: count/angle change, observed counts per turn,
  scale ratio, velocity statistics, missing sequences, duplicates, direction
  reversals, and index intervals. Summary statistics use the inclusive marked interval.

Compare the two **actual directories printed by the script**:

```bash
python3 hamr_bringup/scripts/record_caster_rotation.py compare \
  recordings/caster_rotations/H1-ACTUAL_RUN \
  recordings/caster_rotations/H2-ACTUAL_RUN
```

For configured 2048 PPR, one complete turn should produce approximately 8192
counts and an angle change of magnitude 6.283185 radians. `count_scale_ratio`
should be near 1; H1/H2 counts-per-revolution ratio should also be near 1 if both
have the same physical encoder resolution and complete pulse capture. The
reference PPR comes from received firmware metadata, falling back to an explicitly
reported 2048 assumption. `--expected-ppr` overrides only the report's reference.

`observed_ppr_equivalent` is counts per physical turn divided by four. It is
**not proof of the encoder's programmed PPR**: missed pulses, mechanical slip,
or an inaccurate physical turn can produce the same number. The script never
changes PPR automatically. A different boot ID, conflicting samples, lost count
continuity, or incomplete markers invalidate a calibration. Missing sequences
are reported separately; continuous hardware counting may still preserve net
travel across packet loss. Index intervals require monotonically directed motion
and sufficient captured index snapshots to be interpreted as turns.

No pip packages are needed beyond the existing ROS environment for recording;
offline comparison uses only Python's standard library and the adjacent
`caster_encoder_protocol.py`. Keep both scripts together if copying them to
another computer. After rebuilding `hamr_bringup`, the same commands are available
as `ros2 run hamr_bringup record_caster_rotation.py record H1` and `... compare ...`.

## 4. What is in the bag

| Topic | Type | Meaning |
|---|---|---|
| `/caster_encoder/packets` | `std_msgs/msg/String` | One validated JSON envelope per datagram, including metadata and every raw sample, duplicate, flag and counter |
| `/caster_encoder/metadata` | `std_msgs/msg/String` | Latest per-node configuration cache; transient-local for late subscribers |
| `/caster_encoder/diagnostics` | `std_msgs/msg/String` | Receiver health, stale/never-seen nodes and separate node/boot sequence tracking |
| `/caster_encoder/H1/joint_states` | `sensor_msgs/msg/JointState` | Valid continuous H1 `angle_rad` and valid velocity |
| `/caster_encoder/H2/joint_states` | `sensor_msgs/msg/JointState` | Valid continuous H2 `angle_rad` and valid velocity |
| `/caster_encoder/CENTER/joint_states` | `sensor_msgs/msg/JointState` | Valid wrapped `calibrated_alpha_rad`; velocity only when independently valid |

Joint names are `caster_h1_rotation`, `caster_h2_rotation`, and
`caster_center_alpha`. Angles are radians; angular velocities are radians/second.
Raw counts stay decimal strings to preserve all int64 digits. Missing values
remain null in JSON and empty in CSV. Invalid samples, duplicates and reordered
samples remain in `/packets`, even when excluded from live angle display.

Every packet envelope keeps `receive_epoch_ns`, `receive_monotonic_ns`, source IP
and port, and the original nested `packet`. Each original sample retains `seq`,
`t_us`, and all firmware fields. `boot_id` separates restarts; never integrate
different boots into one continuous angle.

The JointState header is **host receipt time**, not synchronized acquisition
time. Samples within one datagram may share it. ESP `t_us` is microseconds since
that individual node boot. Firmware clock-sync flags remain false; matching
arrival times does not prove simultaneous acquisition. Preserve original times
for subsequent independently measured clock calibration.

The recorder explicitly requests reliable ROS delivery and transient-local
metadata. This covers the ROS-to-bag hop; it cannot repair UDP loss. ROS bag topic
discovery and QoS override behavior are documented in the
[ROS 2 Jazzy rosbag2 guide](https://github.com/ros2/rosbag2/blob/jazzy/README.md).

## 5. Replay the measured motion in the 3D caster model

No external reference CSV is needed for this replay. After recording a bag:

```bash
source /opt/ros/jazzy/setup.bash
source ~/hamster_ws/install/setup.bash
ros2 launch hamr_bringup caster_encoder_replay.launch.py \
  bag:="$HOME/hamr_recordings/bags/YOUR_RUN" rate:=0.5
```

This opens the existing CAD caster in RViz and drives its joints from the bag at
half speed. Use `rate:=1.0` for normal speed. It is a **kinematic simulation
replay**: recorded angles prescribe the model's motion. It does not calculate
contact forces, reconstruct robot ground travel, or provide independent ground
truth. The existing Gazebo rig's caster joints are passive and have no measured
joint-position replay interface; this separate viewer preserves the measurements
without adding artificial actuator dynamics.

The launch plays only the raw encoder packet topic, remapped into
`/caster_replay/packets`, and supplies the bag clock. It starts a replay adapter,
`robot_state_publisher`, and RViz; no live ESP receiver or motor controller is
started. Replay joint states, transforms and visual markers use separate topics
and a `caster_replay/` frame prefix. Close another simulator publishing `/clock`
in the same ROS domain before starting this replay. For headless playback use
`rviz:=false`.

| Encoder | Default model joint | Display interpretation |
|---|---|---|
| H1 | `caster_positive_hemisphere_joint` | Relative shell rotation; illustrative side/phase assignment |
| H2 | `caster_negative_hemisphere_joint` | Relative shell rotation; illustrative side/phase assignment |
| CENTER | `caster_carrier_joint` | Wrapped common-rod/carrier angle |
| Polar rollers | Both polar-roller joints | Nominal zero; their rotations are unmeasured |

Colored phase markers make shell spin visible. The viewer reports missing,
invalid and stale measurements rather than filling gaps with invented motion.
It separates reboots and reports the limitations of the displayed configuration.
The red/green dots belong to the CAD positive/negative hemispheres, respectively;
the blue arrow belongs to the carrier. Swapping H1/H2 does not swap those colors.

Physical H1/H2 side assignment, directions and CAD zero are not established by
these repositories. To adjust the visualization, copy the installed mapping:

```bash
cp "$(ros2 pkg prefix hamr_bringup)/share/hamr_bringup/config/caster_encoder_replay.yaml" \
  "$HOME/caster_encoder_replay.yaml"
# Edit the copy, then:
ros2 launch hamr_bringup caster_encoder_replay.launch.py \
  bag:="/path/to/YOUR_RUN" mapping_config:="$HOME/caster_encoder_replay.yaml"
```

The ROS YAML node is `replay_caster_encoder_joints`. Swap `h1_joint` and `h2_joint`
if your physical labels correspond to the opposite CAD hemispheres. For each of
`h1`, `h2`, and `center`, set `*_sign` to +1.0 or -1.0 and `*_zero_rad` to an
independently established zero. The displayed coordinate is
`q_model = sign * (q_encoder - zero_rad)`. These corrections apply on top of the
firmware's calibration and do not change the stored data. Leave
`mapping_verified: false` until you have checked the actual side/sign/phase
mapping. A visually aligned model still does not measure encoder accuracy.

## 6. Export plots and replay raw ROS topics

Analyze a finalized bag without running playback:

```bash
BAG="$HOME/hamr_recordings/bags/YOUR_RUN"
ros2 run hamr_bringup analyze_caster_encoders.py "$BAG" \
  --output "$HOME/hamr_recordings/reports/YOUR_RUN"
```

The report contains `samples.csv`, `metadata.jsonl`, `summary.json`, angle/velocity
plots and quality plots per node/boot. It reports missing sequences between the
lowest and highest observed sequence numbers, separately lists earlier unobserved
sequences, and reports duplicates, ordering, rates, validity and cumulative
firmware counter maxima. Earlier unobserved samples may include pre-recording
activity or acquisition/network/recording loss; their cause is not established. No
reference means **accuracy unavailable**, not zero error. Existing bags recorded
before this change do not contain the new ESP measurements and cannot recover them.
Choose a new or empty output directory for each report; the analyzer refuses to
mix results from different runs. `packets.jsonl` also exports the raw envelopes.

You can also process the encoder repository's original logger output:

```bash
python3 rosbags/analyze_caster_encoders.py \
  --packets-jsonl /path/to/recordings/run-.../packets.jsonl \
  --output /path/to/report
```

Open the MCAP directly in Foxglove and add Plot panels for
`/caster_encoder/H1/joint_states.position[0]`, the corresponding H2/CENTER
positions, and `velocity[0]` where present. Plot the three channels separately:
they describe different rotations. The standalone PNGs work without Foxglove.

For ROS replay, start only the recorded sensor topics, then attach your plotting
tool. The original HAMR bag also contains motor commands, so use this explicit
sensor-topic list:

```bash
ros2 bag play "$BAG" --clock --topics \
  /caster_encoder/packets /caster_encoder/metadata /caster_encoder/diagnostics \
  /caster_encoder/H1/joint_states /caster_encoder/H2/joint_states \
  /caster_encoder/CENTER/joint_states
```

The UDP bridge is a live receiver and rejects simulation time; it is not needed
during playback. Avoid mixing its live streams with the replayed topic names.

## 7. Add an independent angle reference later

A good-looking trace, a high sample rate, and low packet loss establish data
quality, not physical angle accuracy. For quantitative error, supply measured
angles for the **same rotation**, with known direction, phase and time alignment.
Vicon body position alone does not independently observe H1/H2 shell phase or
CENTER angle. Likewise, requested motor velocity is not a measured angle reference.

Prepare a CSV from an independent calibrated angular fixture or valid camera
measurements. Required columns are:

```csv
node_id,boot_id,time_epoch_ns,angle_rad
1,0123456789abcdef,1800000000000000000,0.10
1,0123456789abcdef,1800000000010000000,0.12
3,fedcba9876543210,1800000000000000000,3.13
```

Those rows are format examples, not calibration. Copy the actual boot IDs from
the run. Timestamps must be integer nanoseconds in the recording host's epoch
clock domain. H1/H2 reference angles must be continuous/unwrapped and in the
same relative coordinate as their encoders. CENTER uses wrapped alpha. A video
frame index divided by FPS is not an epoch timestamp: align the camera recording
using measured capture timestamps/common events first, and retain only valid
vision measurements whose angular uncertainty has been independently checked.

A header-only template is installed at
`share/hamr_bringup/config/caster_encoder_reference_template.csv`. It also includes
an optional `valid` column: use 1 for a usable reference row and 0 for an excluded
row. No reference data or accuracy scores are fabricated when the template is empty.

```bash
ros2 run hamr_bringup analyze_caster_encoders.py "$BAG" \
  --reference-csv /path/to/independent_angles.csv \
  --max-match-dt-s 0.02 \
  --output "$HOME/hamr_recordings/reports/YOUR_RUN_reference"
```

The report adds matched error plots and RMSE, MAE, bias, p95 and maximum absolute error.
CENTER residuals wrap across 0/2pi; H1/H2 retain full-turn errors. Nearest matches
outside the time tolerance are excluded. It never silently fits a clock offset,
phase zero or sign to make the error smaller. When independently measured,
explicit reference corrections can be supplied using
`--reference-time-offset-s`, `--reference-sign NODE=1` (or `NODE=-1`) and
`--reference-zero-rad NODE=VALUE`. Check `--help` for their exact convention.

By default comparison uses host receive times and must be interpreted with
unknown Wi-Fi and batching delay. A 20 ms matching window is an acceptance
window, not a measured timing accuracy. An independently calibrated per-node/boot
affine clock mapping can be supplied with `--clock-map measured-clock-map.json`,
using the format in `caster-encoder-controller/docs/protocol.md`. Recalibrate
after a reboot; do not mark a mapping calibrated based only on packet arrival.

Before claiming physical accuracy, verify H1/H2 PPR/counts per revolution, signs,
physical zero and time alignment. The tentative configuration is 2048 PPR at x4
decoding, or 8192 counts/revolution. H1/H2 physical phase validity is false in the
current firmware. CENTER's raw single-turn resolution is 14 bits, but resolution
is not measured accuracy; its unwrapped angle and velocity can legitimately be
unavailable until the rate bound is verified. Keep these flags with the results.

## 8. Troubleshooting

| Observation | Check |
|---|---|
| All nodes never seen | Compiled `RECEIVER_IPV4`, UDP port, receiving interface, AP isolation and firewall |
| Address already in use | Stop the standalone logger or duplicate ROS bridge |
| One missing role | Board power, flashed role, Wi-Fi connection and metadata |
| Packets present but no H1/H2 angle | `ready`, `sample_valid`, `unwrap_continuous`, `continuity_lost`, raw count validity |
| CENTER angle but no velocity | Valid wrapped position does not imply trusted unwrapping/rate |
| New boot ID | Treat as a new segment; refresh zero and clock calibration |
| Bag has no caster packets | Updated recorder running, bridge topics present, same ROS domain, allow discovery before trial |
| No reference error reported | Supply valid matching node/boot reference rows within the time tolerance |

## 9. Implementation validation (2026-10-14)

The bridge's 39 focused tests cover wire validation, int64/nanosecond precision,
boot separation, display validity, missing/duplicate/reordered samples, staleness,
socket ownership and simulation-clock rejection. The encoder repository's
14 Python tests and portable C++ encoder-math tests also pass under WSL Ubuntu.

A real loopback UDP -> ROS bridge -> production recorder -> bag-playback check
passed with both SQLite3 and MCAP. Each bag preserved 147 raw packet messages
exactly through playback, 46 valid JointStates per node, and all three nodes'
transient-local metadata despite recording starting after metadata publication.
The fixtures exercised malformed packets, loss, late recovery, duplicates,
staleness and raw counts above 2^53. These are synthetic software tests, not
encoder accuracy measurements.

A four-second nominal-rate check recorded 1,200 datagrams and all 2,400 samples
(800 per node, 600 samples/s aggregate) to MCAP without missing sequences.
The analyzer's 29 tests cover known reference errors, full-turn versus circular
residuals, reference parsing, calibrated clocks and plots. Installed analysis of
synthetic reference data recovered the injected 0.05 rad RMSE for all three
nodes. These tests establish the local software pipeline, not Wi-Fi reliability.

The installed headless CAD replay launch also passed against captured MCAP and SQLite3 bags:
all five model joints and prefixed transforms were published, multiple distinct
shell orientations were observed, and the final measured pose remained at EOF.
No messages were published to global `/tf` or `/joint_states`, and no motor
controller was launched. The two polar-roller positions were explicitly nominal.
The replay mapper and launch have 26 focused tests covering side/sign/zero
mapping, invalid/held measurements, reboots, clock resets and topic isolation.

An actual RViz render was also checked using a virtual display. The CAD model,
phase markers, carrier arrow and readable measurement/status text rendered
correctly. This preview used synthetic encoder data.

This workstation has ROS 2 Humble, so execution validation used Humble with an
isolated rosbag package overlay; the repository targets Jazzy. The real ESPs did
not deliver UDP traffic to this Windows host during a five-second passive check.
Actual flashed receiver addresses, live Wi-Fi delivery and physical encoder
accuracy remain to be verified on the recording computer with the running boards.
