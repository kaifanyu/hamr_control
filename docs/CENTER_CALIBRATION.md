# Calibrate left and right ball-caster CENTER

Use `calibrate_caster_center` for the **ball caster**, not the ordinary caster.
It captures each wheel's absolute zero, verifies a user-positioned quarter-turn,
checks the opposite pose and return repeatability, and estimates two effective
rolling radii from a straight-line Vicon bag. It reads sensors and writes new
JSON files; it does not command motion, reflash boards, or apply controller
parameters. No firmware change in `caster-encoder-controller` is needed.

The fixed main axle/fork stays in its mounted orientation; the carrier measured
by CENTER must remain free to rotate. CENTER is carrier phase, not swivel yaw or
H1/H2 shell spin. Keep the native 16384-count full-turn scale.

## Quick bench check: one CENTER revolution on either wheel

To record your current H1-down pose and one full carrier turn without assigning
a physical zero, use the short `center_turn_check` utility. Start the live
receiver as below, then run:

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/hamster_ws/src/hamr_control/hamr_bringup/scripts/center_turn_check \
  record ~/center_left_turn_01.jsonl
```

It reads and prints the starting raw count. Hold still for 3 seconds; at **GO**,
turn the whole carrier slowly around the fixed horizontal axle, in one direction,
through one complete revolution back to the physical starting mark. Hold there
until the 30-second recording ends automatically. Do not turn backward through
another revolution: completing 360° already returns you to the starting pose.
Keep the hand rotation below one revolution/second for the bound in this example.
`--seconds 45` allows more time. Ctrl+C preserves partial history.

Evaluate later (no Vicon or ROS subscription needed for this step):

```bash
python3 ~/hamster_ws/src/hamr_control/hamr_bringup/scripts/center_turn_check \
  check ~/center_left_turn_01.jsonl \
  --max-rpm 60 --report ~/center_left_turn_01_check.json
```

These direct source commands do not require a rebuild. After building
`hamr_bringup`, `ros2 run hamr_bringup center_turn_check` is also available.

For front-right, select the wheel when recording and use separate filenames:

```bash
python3 ~/hamster_ws/src/hamr_control/hamr_bringup/scripts/center_turn_check \
  record ~/center_right_turn_01.jsonl --wheel front_right
python3 ~/hamster_ws/src/hamr_control/hamr_bringup/scripts/center_turn_check \
  check ~/center_right_turn_01.jsonl \
  --max-rpm 60 --report ~/center_right_turn_01_check.json
```

The checker reads the wheel from the recording header and verifies the packets
match it. Omitting `--wheel` on `record` retains the front-left default.

The JSON report includes averaged starting/ending counts, accumulated signed
degrees/revolutions, return error and maximum reverse excursion. A pass requires
one revolution and return within 2°, at most 2° backtracking, and stationary first/
last two seconds. An ambiguous gap, reboot or changed device prevents evaluation.
`--max-rpm 60` is a supplied upper bound for the entire motion, not a fitted speed;
do not reduce it to make gaps pass. Raw packets, including invalid reads, remain
in the JSONL. Existing files are never overwritten. No saved zero is changed,
and passing is not an independent angular-accuracy measurement.

## 1. Build and start the receivers

On the robot's Linux workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/hamster_ws
colcon build --packages-select hamr_bringup --symlink-install
source install/setup.bash
```

Use the existing working per-wheel receiver configuration, including verified
board identities. If the receivers are not already running, start them in a
separate terminal (source the same ROS/workspace setup there):

```bash
ros2 launch hamr_bringup caster_encoders.launch.xml record_bag:=false
```

Do not start a second receiver on the same UDP ports. Use live telemetry for
this workflow: stop onboard flash recording first, since it suppresses live
samples. The tool needs `/caster_encoder/front_left/packets` and
`/caster_encoder/front_right/packets`; do not feed it replay during pose capture.
The default installed YAML uses ports 5005/5006. Pass your normal `config_file`
to the launch if your working inventory is in a different file.

## 2. Capture the four poses on each wheel

Support the robot with the motors stopped so the carrier can turn freely.
Keep the fork upright and the main axle fixed. Position the **carrier/split
axis**, not an independently spinning shell marking. Front/back refer to the
robot's physical forward direction, the same on left and right.

Use a fresh output directory for this calibration session:

```bash
ros2 run hamr_bringup calibrate_caster_center zero \
  --wheel both --output ~/center_calibration_01
```

The script prompts for front-left first, then front-right. For each wheel:

| Prompt | Physical pose | Purpose |
| --- | --- | --- |
| `zero` | H1 front, H2 back; split axis horizontal, fork upright | Raw absolute zero |
| `quarter` | Rotate forward by +90°: H1 down, H2 up | Determine encoder sign |
| `opposite` | Continue to +180°: H1 back, H2 front | Half-turn consistency check |
| `return_zero` | Return to H1 front, H2 back | Check positioning repeatability |

Press Enter **after** placing and holding the requested pose. Keep holding
through the one-second receiver warmup and three-second capture. The script
prints the mean raw count and measured stationary spread after each capture.
Use a simple alignment fixture or physical axis reference to reproduce poses.
A flip directly from front to back is the opposite-pose check; it cannot replace
the quarter-turn sign check. Clockwise/counterclockwise depends on viewing side,
so use H1-front → H1-down on both wheels.

By default a pose must have at least 20 unique samples, cover at least 75% of the
capture duration, have no acquisition gap over 0.25 s, and stay within 0.5° of
its circular mean. Quarter/opposite errors must be at most 5°; return-to-zero
error must be at most 2°. These are rejection thresholds, not accuracy claims.
The final zero averages the two zero-pose means circularly and rounds to the
nearest native count. The other poses check sign/consistency without rescaling
the encoder. `accuracy_validated` stays false.

Outputs:

```text
center_calibration_01/
  front_left_center_observations.json
  front_left_center_calibration.json
  front_right_center_observations.json
  front_right_center_calibration.json
```

The calibration includes all accepted raw samples and wheel/device identity.
Files are never overwritten. A failed geometric check preserves the observations
but does not write that wheel's calibration. Use a new session directory for a
repeat; `--wheel front_left` or `--wheel front_right` can capture one side alone.
Completed left-side output remains intact if the right-side capture is interrupted.

For a live check afterward:

```bash
ros2 run hamr_bringup caster_center_angle \
  --calibration ~/center_calibration_01/front_left_center_calibration.json --once
ros2 run hamr_bringup caster_center_angle \
  --calibration ~/center_calibration_01/front_right_center_calibration.json --once
```

At H1-front/H2-back these should read approximately 0°, and at H1-down about 90°.
Calibration uses raw `position14`, so firmware sign/zero does not get applied
twice. The old `caster_center_angle` default still points to the old left-only
reference; use the explicit new path.

## 3. Record one straight forward run with Vicon

Return the robot to normal loaded floor contact. Start the normal Vicon pipeline
and the existing live encoder receivers. Do not move the encoder mounting or
electronic zero after calibration. Exact initial carrier phase does **not** need
to be reset for the radius run.

In another sourced terminal:

```bash
ros2 run hamr_bringup record_hamr_vicon_bag center_radius_forward_01
```

1. Hold the robot stationary for **5 seconds** after recording starts.
2. Move slowly **2–3 m straight forward**, perpendicular to the carrier axle,
   with the fork/body heading fixed. Use your established motion method; this
   tool never drives the robot.
3. Stop and hold stationary for **5 seconds**.
4. Stop the bag recorder with Ctrl+C.

Use a separate bag for each pass. Avoid combining forward and reverse in one
fit. The recorder includes both wheels' raw packets and `/HAMR_base/odom`.
Default bag location is `~/hamster_ws/src/hamr_control/rosbags/`; the recorder's
printed output path is authoritative if `HAMR_BAG_ROOT` is set.

## 4. Estimate both radii

```bash
ros2 run hamr_bringup calibrate_caster_center radius \
  --bag ~/hamster_ws/src/hamr_control/rosbags/center_radius_forward_01 \
  --calibration-dir ~/center_calibration_01 \
  --vicon-forward-offset-deg 90 \
  --max-rpm 120 \
  --output ~/center_calibration_01/radius_forward_01.json
```

Two explicit inputs matter:

- `--vicon-forward-offset-deg 90` adds 90° to raw Vicon yaw, matching the current
  hardware `base_yaw_offset=π/2` in `hamr_hw_control_params.yaml`. Confirm this is
  the forward rolling direction of the fixed caster axle on your assembly. Use
  another value if your marker frame or mounting differs; a straight sideways
  run is not a forward radius calibration. Default Vicon input is
  `/HAMR_base/odom`; `--vicon-topic` can select a corresponding pose topic.
- `--max-rpm 120` asserts that CENTER cannot exceed **two revolutions/second
  during this run**. This is an example conservative bound for a slow run, not a
  measured hardware limit or an instruction to move at that speed. Supply a
  credible bound for your setup. Do not lower it just to make a missing-data
  error pass. The code needs `bound × acquisition_gap < half a revolution`,
  allowing two counts of quantization, to count wraps unambiguously. It also
  rejects observed increments above the bound. Firmware flags are unchanged.

The estimator uses the first/last two seconds of common data as stationary
windows. It averages Vicon position and unwrapped CENTER angle in those windows:

```text
signed_distance = projection of (finish_position - start_position) onto forward
signed_rotation = calibrated CENTER_finish - calibrated CENTER_start
effective_radius = signed_distance / signed_rotation
```

The full valid CENTER history is used to count turns. Endpoint averaging avoids
requiring motion-time synchronization of ESP acquisition clocks with Vicon.
It assumes endpoint holds outlast packet transport delays. It does not report
instantaneous synchronized speed or a timing accuracy claim. Downloaded onboard
flash recordings require a separately aligned workflow and are rejected here.

The report gives each radius in meters, signed angle change, turn count, raw
zero/sign/device used, largest acquisition gap, traveled distance, lateral and
attitude checks. No controller or firmware radius is changed.

Default acceptance limits: at least 1 m and one full CENTER revolution, stationary
endpoint Vicon deviation at most 5 mm and CENTER range at most 0.5°, lateral path
deviation at most 2 cm, attitude change at most 2°, height variation at most
2 cm, and no Vicon gap above 0.25 s. It rejects reversal, conflicting duplicates,
device changes, reboots, invalid direction, and ambiguous encoder wrap gaps.
Small permitted body rotation still contributes uncertainty because no measured
wheel-center lever arms are supplied. A locked/fixed body heading is preferable.

For a longer bag containing extra activity, `--start-s` and `--end-s` select a
single straight run including stationary endpoints. Seconds are relative to
the **first Vicon message's bag timestamp**. Prefer dedicated short bags.

## 5. Validate before using the radius

Repeat another forward pass and a separate reverse pass with the same load and
floor. For reverse, use the same physical forward-axis offset and add
`--direction reverse` to the radius command. Keep distinct output filenames.
Compare the left/right estimates **across runs**, rather than assuming the two
wheels must match each other. A repeat should predict Vicon displacement from
its independently measured CENTER rotation using the first run's radius.

The nominal CAD ball radius is 100 mm; the fitted value is an effective rolling
radius. Constant slip is inseparable from radius in this experiment, and seam/
polar-roller contact can vary with phase. Inconsistent results require inspecting
the recording and mechanical contact, not changing the 16384-count encoder scale.
The new reference describes carrier orientation only; full hemisphere orientation
also requires H1/H2 spin references. Replay's CAD joint convention is separate:
do not paste this forward-positive sign into a CAD mapping without converting its
axis convention.

## Saved right reference and reconstruction

`front_right_center_calibration.json` now records the measured reference from
`center_right_turn_01.jsonl`, for CENTER device `10:bd:a3:af:f3:30`:

- Raw reference: **16183 / 16384 counts**, or **6.206102772588332 rad**.
- Physical pose: H1 points toward the fork mounting block. Both point down in
  the bench photograph, so H1 points **up (+Z)** with the fork upright in CAD.
- Increasing raw counts initially move H1 toward the front (+X), as confirmed
  by the operator. The standalone CAD carrier axis is -Y, so `center_sign: -1.0`.
- Return test: +359.8470136° raw travel and -0.1529864° endpoint residual. These
  check return repeatability, not absolute angular accuracy. The native 16384
  scale is retained; the initial stationary pose defines zero, not the final pose.

`caster_encoder_replay_front_right_center.yaml` implements the anchored mapping:

```text
q_CAD = reference_model_rad + center_sign * wrap(raw_angle - reference_raw_angle)
reference_model_rad = 5.190977717061848
```

The renderer and RViz launcher automatically select this reference for
`front_right`. In particular, `visualize_caster_bag ... --wheel front_right`
now starts from the actual recorded raw count, instead of assuming the first
frame is H1-front. The unified renderer does not reset the initial pose separately for each run. The same absolute orientation is recovered across boots
if the encoder mounting and electronic zero have not changed. This does not
recover turns during missing data. Raw counts are used independently of firmware
`calibrated_alpha_rad`, avoiding a second sign/zero correction. A wrong wheel or
CENTER device identity stops reconstruction.

**To reverse reconstructed CENTER motion later**, change only `center_sign`
between `-1.0` and `1.0` in the right reference YAML. The anchored formula keeps
H1-up at raw 16183 in either direction. Do not change the reference count, model
reference angle, or count scale to reverse direction. You can also copy the YAML
and pass it using `--center-reference /absolute/path/to/copy.yaml` (video tools)
or `center_reference:=/absolute/path/to/copy.yaml` (RViz launch).

`--center-reference none` disables automatic reference selection for a comparison
with the old assumed-start preset. An explicitly enabled reference in
`--mapping-config` remains explicit. Existing left-side behavior is retained.
The full reconstruction still requires H1/H2/CENTER measurements; a CENTER-only
bench recording does not measure the shell spin phases or polar-roller motion.
No wheel radius has been fitted from this bench rotation.
