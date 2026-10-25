# Commanded local caster recording

Each ESP32 stores samples in its own flash only between the Pi's `start` and
`stop` commands. Wi-Fi stays connected for control; live sample UDP is suppressed
while preparing/recording. A temporary Wi-Fi outage does not discard the local
recording. A stop command needs connectivity: until it reaches the board, that
board continues recording, or stops when its storage is full.

`stop` automatically downloads and verifies all selected boards, saves the
original binary logs and analysis exports beside the robot's bags, then
acknowledges the verified downloads. The acknowledgement permits a future
`start` to reuse storage; it does not immediately erase the retained recording.
Robot motion and the original rosbag are never modified by these commands.

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
# Add caster_encoders:=false to disable the live caster receivers.
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

The existing analyzer accepts the exported packet file:

```bash
python3 rosbags/analyze_caster_encoders.py \
  --packets-jsonl "$HAMR_BAG_ROOT/encoder_trial_001/packets.jsonl" \
  --wheel front_left --output "$HAMR_BAG_ROOT/encoder_trial_001/analysis_front_left"
```

Mapped time is an **estimate**, with RTT/residual information retained. The
export does not claim validated synchronization accuracy or change the firmware
clock-valid flags. Clock steps, mismatched boots, insufficient anchors, and
implausible drift leave affected data unmapped, while preserving its raw local
timestamps. Camera/Vicon clock alignment remains a separate calibration.
The original robot bag and these sidecars can be merged later using mapped
acquisition times. Do not feed delayed data through the live UDP bridge: that
bridge uses arrival time for its ROS messages.

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
