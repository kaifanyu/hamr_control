# Ordinary caster: acquisition, recording and orientation replay

Use **four ESP32-C6 boards total**, two per caster: front-left `SWIVEL`,
front-left `WHEEL`, front-right `SWIVEL`, and front-right `WHEEL`. The `SWIVEL`
board reads AMT222A-V absolute SPI; the `WHEEL` board reads both Hall A/B
quadrature channels. Left and right use identical pin assignments for the same
role. Each board has its own MAC, boot history, sequence and clock. The
controller firmware and wiring guide are in the sibling
`caster-encoder-controller/ordinary_caster/README.md`.

This pipeline measures passive caster motion. The existing robot trajectory
controller still commands the vehicle. A ball caster's CENTER angle measures
its carrier rotation; it is not the ordinary caster's vertical swivel angle.

## Family separation and compatibility

| Layer | Ball casters | Ordinary casters |
|---|---|---|
| Firmware driver/configuration | `ball_caster/` (legacy root configs supported) | `ordinary_caster/` |
| Boards | H1, H2, CENTER per wheel | SWIVEL (5), WHEEL (6) per wheel |
| Live ports, left/right | 5005 / 5006 | 5007 / 5008 |
| Wire / flash format | v2 / CASTLOG1 | v4 / CASTLOG2 (legacy v3 readable) |
| ROS configuration | Existing `config/caster_*.yaml` | `config/ordinary_caster/` |
| ROS processing | Existing ball scripts and tools | `hamr_bringup/ordinary_caster/` |
| Launch family folders | `launch/ball_caster/` | `launch/ordinary_caster/` |
| Default recording root | `$HAMR_BAG_ROOT` | `$HAMR_BAG_ROOT/ordinary_caster` |

Existing ball commands retain their behavior. The shared `caster_recording`
command defaults to ball. Add **`--caster-type ordinary` before the subcommand**
to select the four ordinary boards. Discovery shares UDP 5010; HTTP commands and
downloads use TCP 8080. Discovery, latest-run lookup and saved manifests enforce
family and board identity. A run never silently mixes ordinary and ball logs.

The default ordinary layout is `split`, requiring both roles on each selected
caster before recording starts. For old combined hardware only, pass
`--ordinary-layout combined` before `start` to require one `ORDINARY`/4 board per
caster. Existing v3 recordings remain readable and recoverable. Do not mix the
combined and split layouts on the same caster.

## Prepare the boards and Pi

1. Build and flash all four ordinary profiles following the controller guide.
   Set the Pi's address in `RECEIVER_IPV4`. Use a full firmware/partition flash.
2. Measure the swivel's body-relative zero and sign on each caster. Set
   `SWIVEL_PHASE_REFERENCE_VALID` only after checking them. Qualify the maximum
   swivel rate before setting `SWIVEL_MAX_RPM_VERIFIED`; this controls whether
   multi-turn swivel integration and velocities can be trusted.
3. Turn each wheel several complete revolutions in both directions. Measure
   its actual x4 CPR and sign. The default 128 CPR is provisional; set
   `WHEEL_CPR_VERIFIED` only after checking it. Wheel phase is relative to boot
   and the configured software count offset, since there is no index sensor.
4. Enter each printed MAC into private copies of
   `hamr_bringup/config/ordinary_caster/encoders.yaml` and `recording.yaml`.
   The bridge arrays are ordered `[SWIVEL, WHEEL]`; the recorder uses explicit
   role/node entries for all four boards. Blank inventories allow commissioning
   but are not physical identification.

Both left boards send to Pi UDP 5007, both right boards to 5008. All four use
the same `RECEIVER_IPV4` (the Pi's actual Wi-Fi address); each ESP has a different
IP address. DHCP is supported. Wi-Fi recording/download is the same workflow
as ball casters, but sharing Wi-Fi does not synchronize acquisition clocks.

On the Pi, build and source the workspace containing this checkout:

```bash
cd ~/hamster_ws
colcon build --packages-select hamr_swivel_caster hamr_bringup --symlink-install
source install/setup.bash
ros2 run hamr_bringup caster_recording --caster-type ordinary discover
```

For live bench observation without starting a robot bag:

```bash
ros2 launch hamr_bringup ordinary_caster_encoders.launch.xml record_bag:=false
ros2 topic echo /ordinary_caster/front_left/diagnostics
ros2 topic echo /ordinary_caster/front_left/swivel/joint_states
ros2 topic echo /ordinary_caster/front_left/wheel/joint_states
```

The right caster uses `/ordinary_caster/front_right/...`. Each wheel also has
`packets` (full original samples and receipt envelope) and latched `metadata`.
Live JointState headers are **Pi receipt time**, not synchronized acquisition
time. Use the stored logs for quantitative trajectory history. Invalid or stale
channels appear in diagnostics; a valid wheel sample remains usable during an
SPI failure. Delayed samples and packets from an old boot do not refresh poses.

## Run the same vehicle experiment

Keep the existing trajectory, speed, floor, payload, reference tracking and
robot bag workflow consistent between caster trials. Label the physical caster
type in the study metadata. The encoder pipeline does not change motor gains
or start vehicle motion.

```bash
export HAMR_BAG_ROOT="$HOME/hamster_ws/src/hamr_control/rosbags"

# First launch the existing hardware stack and robot bag. Disable its ball
# bridges with caster_encoders:=false. Wait until its bag is recording.

# Start all four local encoders BEFORE triggering the vehicle trajectory.
ros2 run hamr_bringup caster_recording --caster-type ordinary \
  start ordinary_trial_001 --bag latest --duration 60

# Trigger and finish the normal vehicle trajectory, then stop/download.
ros2 run hamr_bringup caster_recording --caster-type ordinary stop
ros2 run hamr_bringup caster_recording --caster-type ordinary timing
```

`--duration` checks storage capacity; it is not an automatic stop timer. Allow
for waiting, trajectory execution and stop latency. Ordinary records are 116
bytes at the configured sample rate; trust the capacity printed for each board.
During commanded recording, firmware suppresses live sample UDP and records
locally. A Wi-Fi interruption does not discard the recording, but stop requires
connectivity. The normal robot bag recorders include ordinary topics whenever
the live bridge is running, and continue collecting robot/Vicon data throughout.

`stop` checks and durably saves each binary before acknowledging it. `fetch`
retries a stopped run's download after interruption. Explicit run IDs, the
latest-run commands, and guarded `--discard-existing` follow the shared
[local recorder guide](LOCAL_CASTER_RECORDING.md). Destructive discard is never
required to start a run when previous data were downloaded successfully.

Use `--wheel front_left` before the subcommand for a two-board, one-caster run.
Networks blocking broadcast can use four explicit addresses before the subcommand:

```bash
ros2 run hamr_bringup caster_recording --caster-type ordinary \
  --node front_left:SWIVEL@192.168.1.101 \
  --node front_left:WHEEL@192.168.1.102 \
  --node front_right:SWIVEL@192.168.1.103 \
  --node front_right:WHEEL@192.168.1.104 discover
```

Supply the actual ESP addresses. Change `discover` to the desired subcommand.
Use `--config PATH` for a private inventory or `--root PATH` for an explicit
sidecar root. `--bag latest` resolves the normal robot bag root by default.

## Read and analyze the history

The default sidecar is
`$HAMR_BAG_ROOT/ordinary_caster/encoder_ordinary_trial_001/`. Keep its
`manifest.json`, original binary files, `metadata.jsonl`, `packets.jsonl`,
`samples.csv` and export report together. Every split-board sample produces one
row for its sensor: swivel uses its SPI acquisition timestamp and wheel uses its
PCNT timestamp. Four board/boot clock estimates align the four channel histories.
Inactive channels are never invented. Legacy combined samples export two rows.
Counts remain exact signed 64-bit values; zeros remain valid numeric values.

```bash
RUN="$HAMR_BAG_ROOT/ordinary_caster/encoder_ordinary_trial_001"
ros2 run hamr_bringup analyze_ordinary_caster "$RUN" --output "$RUN/analysis"
```

The ordinary analysis reports joint trajectories and data quality. Run the
existing `analyze_study_test` on the associated robot bag for the same vehicle
tracking-error comparisons used for ball trials. Join joint observations to
Vicon/reference time through the saved clock estimates; do not compare raw
ESP boot timestamps across boards or use download times as sample times.

Clock estimates come from exchanges before and after recording. They retain
their uncertainty and valid interval, and do not constitute hardware clock
synchronization. Missing maps still permit device-local analysis. Combined
replay requires bounded clock uncertainty and coverage for every sample.

## Replay the measured orientation

```bash
ros2 launch hamr_bringup ordinary_caster_replay.launch.py input:="$RUN"
ros2 run hamr_bringup render_ordinary_caster_recording "$RUN" \
  --output "$RUN/render" --video --fps 25
```

Replay publishes `/clock`, `/ordinary_caster/replay/joint_states`, and explicit
validity status. Swivel drives each model's vertical joint; wheel angle drives
its axle joint. For numerically accurate relative wheel phase, replay subtracts
the first valid integer count in each recording/boot before converting to
radians; the original boot count and configured offset remain in the saved data.
The replay clock starts at one second plus elapsed acquisition time, with a
one-second startup delay for subscribers; status retains the source timestamps.
It uses measured acquisition intervals, retains gaps and boot
boundaries, and does not invent rotations across ambiguous intervals. RViz can
retain the last transform when a channel is invalid; the status marker is the
authority for whether the displayed pose is current and valid.

For an unsynchronized single-sensor bench log, explicitly choose its caster,
role and device-local time. Selecting only a caster is insufficient because
its two boards have independent clocks:

```bash
ros2 launch hamr_bringup ordinary_caster_replay.launch.py input:="$RUN" \
  wheel:=front_left role:=SWIVEL device_local:=true
```

Exploratory playback of provisional calibration requires
`allow_uncalibrated:=true` (CLI `--allow-uncalibrated`) and remains labeled as
such. It does not verify CPR or establish a physical swivel zero.

This is kinematic replay of measured joint history. The existing swivel URDF
and the displayed spacing are illustrative, not a calibrated digital twin or
a new friction/dynamics simulation. Wheel slip and vehicle displacement require
the associated external pose measurements. Verify physical wiring, calibration,
timing uncertainty and maximum recording duration on the bench before study runs.

## Software validation

All four split ordinary profiles were compiled and linked with
ESP-IDF 5.4.2. The legacy combined and six ball profiles were also compiled
during the original ordinary integration. Controller native acquisition/math,
serializer and flash lifecycle tests cover the split profiles and ball behavior.
ROS tests cover actual firmware-generated binary fixtures, CRC and
exact-count decoding, four-board HTTP recording/download/recovery, independent clock mapping,
invalid-channel handling and conservative replay. The installed ordinary CMake
test suites and real UDP bridge/replay/robot-state-publisher checks pass under
ROS 2 Humble in WSL; the repository's deployment target remains ROS 2 Jazzy.
No physical board was flashed or experimentally calibrated during this change.
