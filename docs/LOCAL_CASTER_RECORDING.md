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

## Render a recording on Ubuntu

Copy the downloaded `encoder_<run>` folder and this updated repository to your
Ubuntu workstation. The script uses the Python helpers and CAD meshes in the
repository, so keep the checkout together. It does not need the Pi, connected
encoders, a running ROS graph, or Gazebo.

Use Ubuntu 24.04 with ROS 2 Jazzy installed. In addition to ROS, install the
renderer dependencies once if they are not already present:

```bash
sudo apt install ffmpeg python3-numpy python3-matplotlib python3-pil \
  python3-vtk9 python3-yaml fonts-dejavu-core \
  ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-rclpy ros-jazzy-std-msgs
```

From your workstation checkout, pass the **encoder recording folder**, followed
optionally by a new output folder:

```bash
cd /path/to/hamr_control
bash hamr_bringup/scripts/render_caster_recording.sh \
  /path/to/encoder_trial_002 \
  "$HOME/Videos/hamr_sim/trial_002"
```

This automatically sources ROS, selects the wheel when the input contains only
one, creates a replay bag, and renders the video with H1/H2/CENTER plots at 1×
speed and 30 fps. For this front-left example, the video is:

```text
~/Videos/hamr_sim/trial_002/encoder_trial_002_front_left_1x_plots.mp4
```

The output also includes full-sample CSV/PNG/PDF plots, video metadata, example
frames, and `replay_bag/encoder_trial_002/`. Originals are never modified.
If the output argument is omitted, a new timestamped folder is created under
`~/Videos/hamr_sim/`. An explicit output folder must be empty or absent.

For a recording containing both wheels, select one wheel per invocation:

```bash
bash hamr_bringup/scripts/render_caster_recording.sh \
  /path/to/encoder_trial_003 \
  "$HOME/Videos/hamr_sim/trial_003_front_right" \
  --wheel front_right --rate 1 --fps 30
```

Use `--no-plots` for the caster animation alone, `--width 1280` for a smaller
video, or `--mapping-config /path/to/caster_encoder_replay.yaml` for an existing
verified CAD mapping. Run `--help` for all options. The Bash script uses Ubuntu's
`/usr/bin/python3` so an active Conda environment does not replace ROS Python.
On a headless workstation whose VTK build requires X, install `xvfb` and prefix
the command with `xvfb-run -a`.

Conversion uses the clock estimates embedded in `packets.jsonl`, preserving
integer timestamp precision and assigning each individual sample its own replay
time. It requires metadata and usable clock maps for all three encoder roles;
it cannot align recordings whose clock maps are missing. The derived bag's
`local-recording-replay.json` and the video's JSON metadata identify the original
packet file by SHA-256 and explicitly label the estimated timing. Synthetic
receive fields required by the older renderer exist only in the derivative;
they are not actual UDP receipt timestamps. Clock-valid flags are unchanged.

This is measured caster motion visualization, including manual movements.
Whole-robot trajectory reconstruction and physics simulation are separate.
The original robot/Vicon bag is not needed for this caster video.

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

Use the fixed CENTER replay mapping explicitly:

```bash
cd ~/hamster_ws/src/hamr_control
bash hamr_bringup/scripts/render_caster_recording.sh \
  /path/to/encoder_YOUR_RUN --wheel front_left \
  --start-pose recorded \
  --mapping-config hamr_bringup/config/caster_encoder_replay_front_left_center.yaml
```

This keeps a different CENTER starting reading different in the video. The
experimental `ground-contact` mode deliberately resets the first complete pose
of every run and would override the fixed reference. Normal recording commands
do not need to change, and the firmware does not need reflashing.

The current CAD's H1-positive, model-+X-forward reference has a carrier joint
angle of 3.620181390266952 rad. Consequently the YAML uses
`center_zero_rad: -3.1561522019315458`; this is a CAD mapping offset, **not** the
physical raw zero of 0.46402918833540646 rad. H1/H2 shell phases and the CAD
direction/side assumptions remain unverified, so `mapping_verified` stays false.

The existing video mapper consumes firmware `calibrated_alpha_rad`. The supplied
YAML therefore requires firmware CENTER sign +1 and zero 0, as in these two
observations where raw and calibrated angles are equal. If firmware sign/zero
changes, adapt that YAML before reuse; the raw-count calibration JSON and
`caster_center_angle` command continue to use the original absolute reference.

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
