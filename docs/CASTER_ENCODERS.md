# Caster encoder Wi-Fi recording and replay

The firmware measures two shell rotations (H1/H2) and the common rod (CENTER)
on each of the front-left and front-right wheels. Protocol v2 includes the
physical wheel and permanent device MAC in every packet. Two Pi receivers keep
boots, metadata, loss counters and live measurements independent. Existing
motor control, drive-wheel odometry and EKF inputs are unchanged.

For Wi-Fi start/stop commands, onboard flash history and automatic verified
download after a trajectory, see [commanded local recording](LOCAL_CASTER_RECORDING.md).
The live UDP workflow below remains available outside those recordings.

For the complete firmware configuration, six flashing commands and Pi inventory,
follow [Front-left and front-right caster setup](DUAL_CASTER_SETUP.md).

```text
Front-left H1/H2/CENTER  -- UDP 5005 --> /caster_encoder/front_left/*
Front-right H1/H2/CENTER -- UDP 5006 --> /caster_encoder/front_right/*
                                       | one MCAP bag, separate topic branches
```

## 1. Network and identity

Edit `config/front_left.local.h` and `config/front_right.local.h` in the firmware
repository. `RECEIVER_IPV4` is the Pi address; `UDP_PORT` defaults to 5005/5006.
Each ESP may use DHCP, a router MAC reservation, or its role-specific fixed IP
field. Every board has a unique source address. Configure the six verified MACs
in the Pi's `caster_encoders.yaml`, in H1/H2/CENTER order for each wheel. Optional
source-IP checks require stable source addresses. Allow both UDP ports through
the Pi firewall and disable Wi-Fi client isolation.

A wheel-scoped bridge rejects mismatched wheel/device/source identity before
publishing or changing live state. Empty inventory entries allow commissioning
with process-lifetime device pinning; diagnostics report `inventory_verified:
false`. Fill every expected MAC before experiments. UDP is best effort and this
live stream cannot recover packets lost during a radio outage. Commanded local
recording retains a separate flash history. Independent ESP clocks remain
unsynchronized; local recording saves pre/post exchanges for an estimated
offline mapping. Only one live receiver may bind each UDP telemetry port.

## 2. Build on the ROS recording computer

In an existing ROS 2 Jazzy workspace containing this repository:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/hamster_ws
colcon build --symlink-install --packages-select hamr_ball_caster hamr_bringup
source install/setup.bash
```

For a new workspace, first follow [the hardware build guide](REAL_CAR_CONTINUOUS.md#1-build-the-hardware-packages).
The package installs its decoder, selection helpers and analyzer; no sibling
firmware checkout is needed on the Pi.

## 3. Collect live encoder data

Copy `hamr_bringup/config/caster_encoders.yaml` to `$HOME/caster_encoders.yaml`,
fill the physical board inventory and confirm each port matches the firmware.
For passive acquisition without motor control:

```bash
export HAMR_BAG_ROOT="$HOME/hamr_recordings/bags"
export HAMR_BAG_NAME="caster_$(date +%Y%m%d_%H%M%S)"
ros2 launch hamr_bringup caster_encoders.launch.xml \
  config_file:="$HOME/caster_encoders.yaml"
```

This starts two bridges and one recorder. Hardware bringup starts its own robot
bag. To include both receivers in that same launch, add
`run_caster_encoders:=true caster_encoder_config:="$HOME/caster_encoders.yaml"`
to `hamr_HW.launch.xml`; the caster flag defaults to false. The hardware launch
owns the recorder and Foxglove bridge, so the included receivers start neither.
Stop standalone receivers before enabling the flag. Alternatively, with a
separate hardware launch already recording, add
`record_bag:=false run_foxglove:=false` to the encoder launch above so its
topics are captured in the existing robot bag. For onboard flash recording,
follow [the local recorder guide](LOCAL_CASTER_RECORDING.md).

```bash
ros2 topic echo /caster_encoder/front_left/diagnostics --once
ros2 topic echo /caster_encoder/front_right/diagnostics --once
ros2 topic echo /caster_encoder/front_left/metadata --once --qos-durability transient_local
ros2 topic hz /caster_encoder/front_left/H1/joint_states
```

Check all six roles are fresh, metadata arrived, and both inventories are
verified. Stop with Ctrl+C and inspect `ros2 bag info` on the actual printed bag
path. Both wheel branches, six topics each, must contain messages. Existing bag
names receive a numeric suffix.

The examples below select `front_left`; use `front_right` for the other wheel.
Analysis/replay of a two-wheel bag requires a selector. Original v1 bags remain
readable with `--wheel legacy` (replay `wheel:=legacy`) or single-topic automatic
selection. To receive old firmware, explicitly set `mode:=legacy` in the separate
encoder launch. That mode represents
one unassigned wheel only. Old bind/port/timeout launch arguments apply only in
legacy mode; dual mode uses the per-node YAML.

### Marked-rotation comparison: H1 versus H2

Use this check to distinguish reported scale from the number of physical turns.
Keep the existing bridge running and source the same ROS environment in another
terminal. The script subscribes to `/caster_encoder/front_left/packets`; it does not bind the
UDP port, command motors, change encoder settings, or need a firmware reflash.
It uses raw integer counts and ESP acquisition timestamps, not JointState receipt
timestamps, which can be identical for two samples in a packet.

From the updated `hamr_control` repository directory:

```bash
python3 hamr_bringup/scripts/record_caster_rotation.py record H1 --wheel front_left
python3 hamr_bringup/scripts/record_caster_rotation.py record H2 --wheel front_left
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
`caster_encoder_protocol.py` and `caster_encoder_selection.py`. Keep the script
and both helper modules together if copying them to
another computer. After rebuilding `hamr_bringup`, the same commands are available
as `ros2 run hamr_bringup record_caster_rotation.py record H1 --wheel front_left` and `... compare ...`.

## 4. What is in the bag

| Topic | Type | Meaning |
|---|---|---|
| `/caster_encoder/front_left/packets` | `std_msgs/msg/String` | One validated JSON envelope per datagram, including metadata and every raw sample, duplicate, flag and counter |
| `/caster_encoder/front_left/metadata` | `std_msgs/msg/String` | Latest per-node configuration cache; transient-local for late subscribers |
| `/caster_encoder/front_left/diagnostics` | `std_msgs/msg/String` | Receiver health, stale/never-seen nodes and separate node/boot sequence tracking |
| `/caster_encoder/front_left/H1/joint_states` | `sensor_msgs/msg/JointState` | Valid continuous H1 `angle_rad` and valid velocity |
| `/caster_encoder/front_left/H2/joint_states` | `sensor_msgs/msg/JointState` | Valid continuous H2 `angle_rad` and valid velocity |
| `/caster_encoder/front_left/CENTER/joint_states` | `sensor_msgs/msg/JointState` | Valid wrapped `calibrated_alpha_rad`; velocity only when independently valid |

Joint names are `front_left_caster_h1_rotation`, `front_left_caster_h2_rotation`,
and `front_left_caster_center_alpha`; the right branch uses `front_right_`. Angles are radians; angular velocities are radians/second.
Raw counts stay decimal strings to preserve all int64 digits. Missing values
remain null in JSON and empty in CSV. Invalid samples, duplicates and reordered
samples remain in `/packets`, even when excluded from live angle display.

Every v2 packet keeps `wheel_id` and permanent `device_id`, and every envelope keeps `receive_epoch_ns`, `receive_monotonic_ns`, source IP
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
  bag:="$HOME/hamr_recordings/bags/YOUR_RUN" wheel:=front_left rate:=0.5
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
  bag:="/path/to/YOUR_RUN" wheel:=front_left mapping_config:="$HOME/caster_encoder_replay.yaml"
```

The ROS YAML node is `replay_caster_encoder_joints`. Swap `h1_joint` and `h2_joint`
if your physical labels correspond to the opposite CAD hemispheres. For each of
`h1`, `h2`, and `center`, set `*_sign` to +1.0 or -1.0 and `*_zero_rad` to an
independently established zero. The displayed coordinate is
`q_model = sign * (q_encoder - zero_rad)`. These corrections apply on top of the
firmware's calibration and do not change the stored data. Leave
`mapping_verified: false` until you have checked the actual side/sign/phase
mapping. A visually aligned model still does not measure encoder accuracy.

### Export the focused recording report

Use the same command for a downloaded recording, bag directory or MCAP file:

```bash
ros2 run hamr_bringup visualize_caster_bag /path/to/combined_0.mcap \
  --output "$HOME/Videos/hamr_sim/my_run"
```

The source-checkout equivalent is
`bash hamr_bringup/scripts/visualize_caster_bag ...`. Open the output `index.html`
for the simulation video, six encoder angles, robot trajectory, robot with both
casters, and robot with aligned EKF. Omitting `--output` writes to
`hamr_control/analysis/<recording-name>/`. `--render none` produces the four plots
without CAD; `--preview-time 5` replaces the video with one CAD frame. Video is
the default; `--render auto` permits a preview when FFmpeg is absent.

Both wheels are selected by default. Use `--wheel front_left`, `front_right`,
or `legacy` for a standalone caster. `--left-mapping-config` and
`--right-mapping-config` select explicit side/sign/zero mappings. The saved
raw CENTER references remain device-specific; use `--center-reference none`
for other devices. Rendering retains validity, duplicate, stale-data and reboot
handling, and holds the last valid measured pose. Unknown roller motion is not
inferred. `--rate 1 --fps 30` retains recorded speed regardless of rendering time.

The [unified workflow guide](COMBINED_CASTER_ANALYSIS.md) describes dependencies,
all output files, clock assumptions and the uncalibrated Vicon comparison.

## 6. Export plots and replay raw ROS topics

Analyze a finalized bag without running playback:

```bash
BAG="$HOME/hamr_recordings/bags/YOUR_RUN"
ros2 run hamr_bringup visualize_caster_bag "$BAG" --wheel front_left \
  --output "$HOME/hamr_recordings/reports/YOUR_RUN"
```

Open `index.html` for the five-section report. The focused command exports only
the requested media and `report.json`; it does not generate packet-quality CSVs
or independent-reference error plots. Missing robot or caster streams are
identified in the corresponding section. Choose a new or empty output directory
for each report; existing reports are not overwritten.

You can also process the encoder repository's original logger output:

```bash
bash hamr_bringup/scripts/visualize_caster_bag \
  /path/to/recordings/run-.../packets.jsonl \
  --output /path/to/report
```

Open the MCAP directly in Foxglove and add Plot panels for
`/caster_encoder/front_left/H1/joint_states.position[0]`, the corresponding H2/CENTER
positions, and `velocity[0]` where present. Plot the three channels separately:
they describe different rotations. The standalone PNGs work without Foxglove.

For ROS replay, start only the recorded sensor topics, then attach your plotting
tool. The original HAMR bag also contains motor commands, so use this explicit
sensor-topic list:

```bash
ros2 bag play "$BAG" --clock --topics \
  /caster_encoder/front_left/packets /caster_encoder/front_left/metadata /caster_encoder/front_left/diagnostics \
  /caster_encoder/front_left/H1/joint_states /caster_encoder/front_left/H2/joint_states \
  /caster_encoder/front_left/CENTER/joint_states
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
measurements. For v2 measurements, include the exact wheel and device identity as well as the role and boot:

```csv
wheel_id,device_id,node_id,boot_id,time_epoch_ns,angle_rad
front_left,aa:bb:cc:dd:ee:01,1,0123456789abcdef,1800000000000000000,0.10
front_left,aa:bb:cc:dd:ee:01,1,0123456789abcdef,1800000000010000000,0.12
front_left,aa:bb:cc:dd:ee:03,3,fedcba9876543210,1800000000000000000,3.13
```

Those rows are format examples, not calibration. Legacy references may omit wheel/device fields; they cannot match v2 measurements. Copy the actual boot IDs from
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

Independent-reference matching is not part of the focused visualization command.
The reusable `caster_packet_analysis.py` module retains `read_reference`,
`ClockMapper` and `analyze` for dedicated calibration workflows. Its CENTER
residuals wrap across 0/2π; H1/H2 retain full-turn errors. Match tolerance,
measured clock corrections and reference signs/zeros must be supplied explicitly;
the focused report's EKF alignment is not an encoder calibration.

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
| Rejected packets | Diagnostics `last_rejection`; physical label, wheel profile, expected MAC/IP and port |
| Inventory unverified | Fill all six physical MACs in the Pi YAML and restart the receivers |
| One missing role | Board power, flashed role, Wi-Fi connection and metadata |
| Packets present but no H1/H2 angle | `ready`, `sample_valid`, `unwrap_continuous`, `continuity_lost`, raw count validity |
| CENTER angle but no velocity | Valid wrapped position does not imply trusted unwrapping/rate |
| New boot ID | Treat as a new segment; refresh zero and clock calibration |
| Bag has no caster packets | Updated recorder running, bridge topics present, same ROS domain, allow discovery before trial |
| No reference error reported | Supply valid matching node/boot reference rows within the time tolerance |

## 9. Two-wheel implementation validation (2026-10-24)

The focused bridge, recording, analyzer, calibration, replay, video and hardware
launch suites pass **189 tests**. These include v1 compatibility; six v2 devices
sharing role/boot values without collisions; independent reboots and sequence
tracking; wrong wheel/MAC/IP rejection before state changes; exact reference and
clock-map identity; and preservation of wheel/device IDs in exports. A warning
about this workstation's optional Matplotlib Axes3D installation does not affect
the tested 2D plots.

`test_dual_caster_pipeline.py` drives six synthetic UDP sources through two real
ROS bridge processes, the production `record_hamr_test_bag` wrapper, MCAP, and
the analyzer. All 12 caster topics are stored separately. Wrong-wheel, device
and source-IP packets stay out of accepted recordings, and a left-H1 restart
does not change right-H1 identity. Each exported CSV contains only the selected
wheel and its three device IDs. The test waits for both diagnostic streams before
stopping the recorder; DDS discovery is completed before sending test samples.

An installed `caster_encoders.launch.xml` also loaded both named YAML profiles,
bound their separate loopback ports, and shut down cleanly. Missing/misspelled
configuration node keys fail before binding; runtime edits of routing parameters
are rejected until restart. Executable permissions were corrected for the
bridge, calibration, analyzer and replay entrypoints, including symlink installs.

Execution validation used the workstation's ROS 2 Humble with a temporary ROS
bag/launch overlay; the deployment guide targets Jazzy. Two additional tests in
the unchanged `hamr_sim_ui` simulator encounter its existing Jazzy-only
`TopicMetadata(id=...)` API when run under Humble. Those simulator failures are
outside the two-wheel acquisition pipeline; its 189 focused tests all pass.

Physical board assignment, actual Pi Wi-Fi delivery and encoder calibration
still require the bench procedure in [the setup guide](DUAL_CASTER_SETUP.md).
No connected ESP boards were flashed by this implementation work. Firmware build
and serializer results are recorded separately in the encoder repository's
`docs/validation.md`.

## 10. Earlier single-wheel implementation validation (2026-10-14)

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
