# Commanded local caster recording

This guide retains the ball-caster defaults. For the four-board ordinary setup,
use `--caster-type ordinary` and follow
[ordinary recording, analysis and replay](ORDINARY_CASTER_RECORDING.md).

Each ESP32 stores samples in its own flash only between the Pi's `start` and
`stop` commands. Wi-Fi stays connected for control; live sample UDP is suppressed
while preparing/recording. A temporary Wi-Fi outage does not discard the local
recording. A stop command needs connectivity: until it reaches the board, that
board continues recording, or stops when its storage is full.

`stop` automatically downloads and verifies all selected boards, saves the
original binary logs and analysis exports beside the robot's bags, then
acknowledges the verified downloads. The acknowledgement permits a future
`start` to reuse storage; it does not immediately erase the retained recording.
The low-level `caster_recording` commands do not stop the robot bag. The unified
start/stop workflow below also closes the associated bag and creates a merged
copy, while leaving hardware and trajectory nodes running.

## Unified manual start and stop (ball casters)

Launch your normal `hamr_HW.launch.xml` with `record_bag:=true` first. Wait until
its bag recorder reports that recording has started. Then, in another terminal:

```bash
source ~/hamster_ws/install/setup.bash
ros2 run hamr_bringup start_encoder_recording

# Run your experiment. When finished, in any sourced terminal:
ros2 run hamr_bringup stop_encoder_recording
```

The start command discovers **all available H1, H2, and CENTER boards on both
front_left and front_right**, validates their identities, prints missing roles,
**erases their previous onboard recordings, including undownloaded logs**, and
starts the new recordings with the full recording partition available. It
returns after startup; no background
client or open terminal is needed. A unique session ID is saved automatically.
Boards that appear later are not added to the running session. Zero boards,
duplicate identities, an inactive robot bag, or firmware without guarded erase
support cause an error before any boards are erased. The selected boards pass
identity, storage, and firmware checks before preparation. Any old onboard
recording still running is stopped before its flash is erased. Firmware,
configuration, and files already saved on the Pi are preserved.

An active unified session must be stopped before starting another one. If its
stop closed the robot bag but an ESP download failed, a new start can discard
that remaining onboard data after you launch a new robot bag. Old Pi session
folders remain available. This automatic erase policy applies to
`start_encoder_recording`; the low-level `caster_recording start` still requires
an explicit `--discard-existing` to discard unread data.

The start output reports each board's actual flash capacity in seconds. Manual
stop does **not** remove that limit (approximately 2.5 minutes on the current
4 MiB layout at 200 Hz); stop before the shortest reported limit. A full flash
stops acquisition early and is reported as incomplete even if download succeeds.

The stop command:

1. Stops the selected ESP recordings and takes final clock probes.
2. Downloads their binary logs, verifies lengths/CRCs/identities, and acknowledges
   only verified, durable files.
3. Sends SIGINT to **the exact robot bag recorder saved at start**, allowing it
   to close cleanly. This ends that bag's recording; hardware, motion control,
   and trajectory nodes keep running. It never signals the whole launch.
4. Creates a new combined MCAP rosbag containing every original robot message
   plus the encoder samples. The original robot bag and ESP logs are retained.

The default root is `~/hamster_ws/src/hamr_control/rosbags`, or `HAMR_BAG_ROOT`.
Both commands accept `--root /path`. Files are organized as:

```text
rosbags/
  hamr_hw_<timestamp>/              # original Pi bag
  encoder_<session-id>/
    session.json                   # exact bag/process, missing boards, outcome
    manifest.json                  # device identities, clock probes, CRCs
    front_left_H1_<MAC>.bin         # one original binary per selected board
    ...
    samples.csv, packets.jsonl, metadata.jsonl
    clock-map.json, export-summary.json
    bags/
      encoders/                    # aligned onboard messages only
      combined/                    # use this rosbag for the full experiment
      merge-result.json            # verified message counts
```

Existing odometry, IMU, Vicon, transforms, actuator and reference-trajectory
messages retain their original serialized data and bag timestamps. Only topics
actually published and selected by the hardware recorder can be included; this
does not synthesize missing odometry or trajectory history. The original bag's
full time span is retained, including data before encoder start and during the
stop/download step.

The hardware recorder (`record_hamr_test_bag`), standalone Vicon recorder
(`record_hamr_vicon_bag`), and COMPA recorder (`record_compa_slam_bag`) include the
fork-mounted Vicon subjects using their case-sensitive subject names:

| Subject | Recorded topics |
| --- | --- |
| Left caster | `/Left_Caster/odom`, `/Left_Caster/pose`, `/Left_Caster/individual_marker_array` |
| Right caster | `/Right_Caster/odom`, `/Right_Caster/pose`, `/Right_Caster/individual_marker_array` |

The caster odometry and pose messages use the `mocap` frame; odometry child frames
are `Left_Caster/base_link` and `Right_Caster/base_link`. These are measurements of
the tracked subjects above the forks, without a transform to the wheel centers.
Both subjects' odometry and pose streams were verified at approximately 120 Hz
on 2026-10-29. The marker-array topics were advertised but delivered no messages
during that check; they will be captured if the Vicon bridge publishes them.
The unified stop/merge workflow preserves these topics with the rest of the
original robot bag. Start a new recording to use an updated topic filter.

New onboard topics use a separate namespace so they cannot be confused with
live UDP data already in the robot bag:

| Topic | Contents |
| --- | --- |
| `/caster_recording/front_left/packets` | Original samples, flags, identities, acquisition times and estimated clock maps (JSON String) |
| `/caster_recording/front_left/metadata` | Device firmware/configuration and timing provenance (JSON String) |
| `/caster_recording/front_left/H1/joint_states` | Valid H1 readings with estimated Pi acquisition timestamps |
| `/caster_recording/front_left/H2/joint_states` | Valid H2 readings |
| `/caster_recording/front_left/CENTER/joint_states` | Valid calibrated CENTER angles |
| `/caster_recording/session` | Session manifest and clock uncertainty (JSON String) |

The same per-wheel topics exist for `front_right` when those boards are present.
Invalid samples remain in `packets` and the original binaries; they are not
turned into valid joint measurements. ESP microsecond timestamps are mapped to
Pi epoch time using clock probes before and after recording. These are
**estimates**, with uncertainty retained, not a claim of hardware synchronization.
Original firmware clock flags remain unchanged. Missing/invalid clock maps
prevent an aligned merge; downloads are retained without substituting download
time for acquisition time.

Optional explicit selection and recovery:

```bash
# Name a run and select an exact active robot bag if multiple are running:
ros2 run hamr_bringup start_encoder_recording trial_001 --bag /absolute/robot/bag
ros2 run hamr_bringup stop_encoder_recording trial_001

# If broadcast discovery is blocked, repeat --node for each available board:
ros2 run hamr_bringup start_encoder_recording \
  --node front_left:H1@192.168.1.111 \
  --node front_right:CENTER@192.168.1.123
```

Both commands accept `--config` for the **recording** inventory (not the live
`caster_encoders.yaml`), `--discovery-address`, and `--discovery-timeout`.
Rerun the same stop command after a transient network/download failure; it
reuses verified files and never restarts recording. A completed stop is
idempotent and requires no boards online. A new session requires a new active
robot bag because stop finalized the preceding one. Foxglove and the live
caster bridges are not required for this workflow.

## Install once

Build and flash the matching wheel/role firmware from the sibling
`caster-encoder-controller` repository onto each board. This feature requires
the new **partition table as well as the application**; use the normal full
profile `flash` command, not an application-only update. Preserve any recordings
before firmware updates. Follow that repository's existing USB/power procedure.

On the Pi, rebuild and source the ROS workspace containing this checkout:

```bash
cd ~/hamster_ws
colcon build --packages-select hamr_bringup --symlink-install
source install/setup.bash
ros2 run hamr_bringup caster_recording discover
```

Discovery uses UDP **5010**, independently of the live bridges on UDP 5005/5006.
Commands and downloads use TCP **8080** on each ESP32. All six boards and the Pi
must be reachable on the same network. Discovery never starts recording.

The default configuration is
`hamr_bringup/config/caster_recording.yaml`. Fill its device MAC inventory after
checking the physical wheel and role labels. Empty identities permit initial
commissioning; duplicate or conflicting wheel/role assignments are rejected.
Use `--config /path/to/caster_recording.yaml` to select a private configuration.
Broadcast-blocking networks can use repeated explicit endpoints, for example
`--node front_left:H1@192.168.1.111`, including every role in the selected wheel(s).

## Record a trajectory

Use the same bag root as the existing robot recorder. Its default is
`~/hamster_ws/src/hamr_control/rosbags`; both tools honor `HAMR_BAG_ROOT`.

```bash
export HAMR_BAG_ROOT="$HOME/hamster_ws/src/hamr_control/rosbags"

# Launch your usual hardware/trajectory stack and its robot bag first.
# Onboard flash recording is started separately from hardware bringup.
# Its optional run_caster_encoders flag only receives live UDP telemetry.
# Wait for the robot bag to report that recording has started.

# Example experiment identifier; choose a new identifier for every run.
ros2 run hamr_bringup caster_recording start trial_001 --duration 60

# Only after start succeeds on all selected boards, trigger your trajectory.
# When it finishes, stop the local encoders and automatically download:
ros2 run hamr_bringup caster_recording stop trial_001
```

`--duration 60` checks whether the **planned total recording interval** fits; it
does not schedule an automatic stop. Include time waiting to trigger the
trajectory, motion, and delay before issuing `stop`. The actual limit is printed
per board before recording. The full 100-byte sample format at 200 Hz fits roughly
2.5 minutes on the current 4 MiB flash layout. Capacity exhaustion stops the board
with an explicit reason and is reported as an incomplete experiment.

To associate the sidecar with a particular robot bag, pass its exact path:

```bash
ros2 run hamr_bringup caster_recording start trial_002 \
  --bag "$HAMR_BAG_ROOT/hamr_hw_YYYYMMDD_HHMMSS" --duration 60
```

The `--bag` option records the association in the manifest; it neither creates
nor changes that bag. For a bench run on one wheel, place the global option
before the command: `caster_recording --wheel front_left start bench_001`.
The command defaults to both wheels and requires all three roles for each
selected wheel. Global options such as `--root`, `--config`, and `--wheel` go
before `start`, `stop`, or `fetch`.

## Discard existing onboard data and start fresh

Normal `start` already erases reusable storage before recording. To also discard
an unread recording or recover a corrupt header such as `invalid_header`, use
`--discard-existing` on each new start:

```bash
ros2 run hamr_bringup caster_recording --wheel front_left \
  start trial_002 --discard-existing --duration 60

# Run the experiment after all three recorders confirm successful start.
ros2 run hamr_bringup caster_recording stop trial_002
```

**This deletes existing onboard logs on every selected board, including recordings
that have not been downloaded.** Existing bags, downloaded files, and manifests
on the host are retained. Choose a fresh run ID; a failed `trial_001` attempt still
reserves its local sidecar directory. The option never overwrites that directory.
For the default continuous hardware trajectory (about 101 seconds including
holds), use a planned interval such as `--duration 120` and allow for trigger and
stop delays; the duration option remains a capacity check, not a timer.

This option requires the updated recorder firmware (0.3.1 or later with
`supports_discard_existing: true` in `status`) on **every selected board** and the
updated `hamr_bringup` client. Updating the ROS workspace alone cannot add it to
already flashed boards. Build and flash the matching wheel/role profiles from
`caster-encoder-controller`, retaining the actual private Wi-Fi/calibration
settings. For front-left, connect each matching labeled board separately:

```bash
cd ~/caster-encoder-controller
# Use the actual serial port; run only the matching connected board's command.
./scripts/idf.sh front_left H1 -p /dev/ttyACM0 flash monitor
./scripts/idf.sh front_left H2 -p /dev/ttyACM0 flash monitor
./scripts/idf.sh front_left CENTER -p /dev/ttyACM0 flash monitor
```

The firmware helper needs ESP-IDF v5.4.2 or its documented Docker environment and
your existing `config/front_left.local.h`. Disconnect battery/UBEC before USB
programming; exit each monitor with Ctrl+] before changing boards. Then rebuild
and source `hamr_bringup` using the installation commands above.

Before modifying any board, the client checks the entire selected inventory,
firmware support, and capacity. If a previous run is active, it stops and flushes
that run without downloading it, then requests erasure and preparation for the
new run. The erase request is bound to the checked device, boot, and previous
run; a changed board or competing run is rejected. Repeating preparation for the
same successfully prepared run never erases it again. The manifest records the
discard choice and previous status. If preparation fails partway through the
fleet, data already discarded on successful boards cannot be restored.

There is no automatic wipe or recording on boot/reconnect. A normal start without
this option continues to protect unread or corrupt logs. Missing flash partitions
and hardware/NVS failures still require repair; this option does not bypass them.

## Saved files and later alignment

The Pi creates `$HAMR_BAG_ROOT/encoder_trial_001/`, separate from rosbag2's own
directory:

- `manifest.json`: run identity, selected physical devices, bag association,
  command outcomes, clock exchanges, download verification, and recovery state.
- One `.bin` file per board: original flash header and complete verified sample
  records. Every record has a CRC; the complete record stream is also checked
  against the board's stopped status. Exact 64-bit timestamps and counts survive.
- `samples.csv`, `packets.jsonl`, and `metadata.jsonl`: identified offline data
  for analysis. Download times are not substituted for acquisition times.
- `clock-map.json`: per-device/per-boot affine estimates derived from repeated
  two-way clock exchanges before and after recording.
- `export-summary.json`: sample coverage, timing results, and incomplete devices.

The unified visualization command accepts the exported packet file:

```bash
bash hamr_bringup/scripts/visualize_caster_bag \
  "$HAMR_BAG_ROOT/encoder_trial_001/packets.jsonl" \
  --render none --output "$HOME/hamr_analysis/trial_001"
```

Mapped time is an **estimate**, with RTT/residual information retained. The
export does not claim validated synchronization accuracy or change the firmware
clock-valid flags. Clock steps, mismatched boots, insufficient anchors, and
implausible drift leave affected data unmapped, while preserving its raw local
timestamps. Camera/Vicon clock alignment remains a separate calibration.
The original robot bag and these sidecars can be merged later using mapped
acquisition times. Do not feed delayed data through the live UDP bridge: that
bridge uses arrival time for its ROS messages.

## Render a recording on Ubuntu

Use the single offline visualization command for the downloaded folder, a
combined bag directory, or its MCAP file:

```bash
bash hamr_bringup/scripts/visualize_caster_bag \
  /path/to/encoder_YOUR_RUN \
  --output "$HOME/Videos/hamr_sim/YOUR_RUN"
```

Open the output `index.html` for both casters, six encoder histories, robot/fork
trajectories, telemetry and timing plots, the combined PDF and raw CSV downloads.
The command sources ROS automatically and uses native Simulation Studio geometry.
An MP4 is generated when FFmpeg is available, otherwise a PNG preview is used.
Add `--render none` for all analysis/plots without CAD, or `--preview-time 5` for
one CAD frame plus every plot. The input and prior reports are never overwritten.

The [unified workflow guide](COMBINED_CASTER_ANALYSIS.md) contains dependencies,
input formats, output layout and calibration limits. Both wheels share one
recorded clock; each keeps its own saved CENTER reference. Use
`--left-mapping-config` / `--right-mapping-config` to override a wheel's mapping,
or `--center-reference none` for a recording from different devices. Raw plotted
angles remain independent of the CAD mapping. Missing clocks require the explicit
`--recover-timing` option and original clock probes for approximate replay.

This is kinematic measured-motion replay. It does not infer contact dynamics,
missing roller motion, or physical encoder accuracy. Vicon data must come from
the same run; unrelated recordings are never shifted onto its timeline.

## Saved CENTER absolute-angle reference

The front-left CENTER reference is stored in
`hamr_bringup/config/front_left_center_calibration.json`, for device
`10:bd:a3:b2:14:a4`. It includes the original packet observations, physical pose
descriptions, sample flags, timestamps and calibration limitations:

| Physical pose | Raw count | Raw angle | Angle from saved zero |
| --- | ---: | ---: | ---: |
| H1 front, H2 back; both at floor, fork upright | 1210 | 26.5869140625° | 0° |
| H2 front, H1 back | 9417 | 206.91650390625° | 180.32958984375° |

The increasing-count separation is 8207 counts, 15 counts (0.32958984375°)
beyond an ideal half-turn. This is a consistency residual, not a measured
accuracy bound. Two samples at each pose agree, but independent placement
repeatability and direction have not been measured. The native scale remains
16384 counts/revolution; the second pose is a check, not a scale correction.
Increasing counts define positive provisionally. A known signed quarter-turn
is still needed to establish the physical direction.

For any future valid raw reading from that same encoder:

```python
orientation_deg = ((position14 - 1210) % 16384) * 360.0 / 16384
```

This gives the single-turn CENTER shaft phase relative to the saved pose.
Robot vertical-axis yaw/world heading and H1/H2 shell spin phase are separate.
Keep the fork's orientation fixed when establishing carrier reference poses.
The saved reference survives a reboot as long as mounting, coupling and the
encoder's electronic raw zero do not change; it cannot recover completed turns.

### Read the saved-reference angle in ROS

After building `hamr_bringup` and sourcing the workspace, enable the passive
receiver if it is not already running:

```bash
source ~/hamster_ws/install/setup.bash
ros2 launch hamr_bringup caster_encoders.launch.xml record_bag:=false
```

In another sourced terminal:

```bash
ros2 run hamr_bringup caster_center_angle --once
# Or monitor at up to two printed observations per second:
ros2 run hamr_bringup caster_center_angle
```

The JSON field `orientation_deg` is relative to the saved zero. The command
reads raw absolute counts, checks wheel/device identity and sample validity,
and reports direction/accuracy validation separately. It does not change
firmware or ROS parameters. Firmware `calibrated_alpha_rad` changes do not
affect this raw-count calculation. Single-turn data remain usable when
`angle_rad` is null and unwrapped continuity is unavailable.

Live UDP is suppressed during onboard recording. Use this command before
starting the recording, or after stopping/finalizing it. It times out instead
of indefinitely displaying an old observation. To calculate offline from a
count in `samples.csv`, no ROS graph or hardware is required:

```bash
ros2 run hamr_bringup caster_center_angle --position14 9417
# orientation_deg: 180.32958984375
```

The source helper can also run without ROS using Python and
`--calibration /path/to/front_left_center_calibration.json --position14 COUNT`.
Save/copy the calibration with the recordings when moving them to a workstation.

### Preserve absolute starting orientation in the video

Paired replay automatically selects each saved raw CENTER reference. To select
an explicit wheel mapping:

```bash
bash hamr_bringup/scripts/visualize_caster_bag \
  /path/to/encoder_YOUR_RUN --wheel front_left \
  --left-mapping-config hamr_bringup/config/caster_encoder_replay_front_left_center.yaml
```

A different recorded raw CENTER count retains its different initial orientation.
The device-specific reference maps raw `position14` independently of firmware
alpha offsets, avoiding a second correction. No per-run ground-contact reset is
applied. H1/H2 shell phase and physical mapping accuracy remain unverified; an
apparently aligned CAD model does not establish encoder accuracy.

## Interrupted commands and recordings

```bash
# Retry stop if connectivity failed or the previous process was interrupted.
ros2 run hamr_bringup caster_recording stop trial_001

# Retry transfer/export of an already stopped recording.
ros2 run hamr_bringup caster_recording fetch trial_001

ros2 run hamr_bringup caster_recording status
```

Keep the original experiment identifier and manifest. A failed transfer never
acknowledges unverified data. Completed downloads are reused after verification;
incomplete downloads restart from byte zero. Start failures attempt to stop all
selected boards, retain recovery information, and return an error rather than
claiming the experiment is ready. Run IDs cannot overwrite existing sidecars.

After a board resets, the firmware recovers the contiguous CRC-valid flash
prefix and reports an interrupted recording. Samples still buffered in RAM at
power loss cannot be recovered. The retained recording keeps its original boot
identity, and new-boot clock exchanges cannot synchronize that old clock.
Firmware never automatically resumes recording on boot or reconnect.

## Acquisition qualification

Flash is erased during preparation, before recording begins. During recording,
RAM staging feeds a background flash writer. Flash writes can still delay
acquisition: test actual boards at the required speeds and inspect sequence
gaps, acquisition overruns, maximum sample interval, and CENTER continuity.
Checksums certify transfer integrity, not uninterrupted acquisition or encoder
calibration. CENTER starts a new unwrap/velocity segment at each recording;
do not integrate its unwrapped angle across separate runs. H1/H2 hardware counts
are not reset by recording commands.

After the first local recording segment, resumed live CENTER UDP marks the
unwrapped angle unavailable until reboot. Wrapped alpha/absolute angle and
valid velocities remain available; the downloaded local logs retain their
per-run unwrapped values. This prevents an old live stream from silently joining
two different unwrap origins under the same boot ID.

The caster commands control only the sensor recorders. Vicon or other remote
inputs used by the robot controller still require their existing connection.
