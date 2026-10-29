# Front-left and front-right caster setup

Each wheel has three ESP32-C6 boards: H1, H2 and CENTER. Firmware protocol v2
identifies every packet by `wheel_id`, permanent `device_id` (Wi-Fi STA MAC),
`node_id` (1/2/3) and `boot_id`. Both wheels keep the same role IDs. The Pi runs
two receivers with separate state and publishes separate ROS topic branches:

| Wheel | Firmware configuration | Default Pi UDP port | ROS node | Topic prefix |
|---|---|---|---|---|
| Front left | `encoder-controller/config/front_left.local.h` | 5005 | `/caster_encoder_front_left` | `/caster_encoder/front_left` |
| Front right | `encoder-controller/config/front_right.local.h` | 5006 | `/caster_encoder_front_right` | `/caster_encoder/front_right` |

## 1. Configure the firmware

In `/home/para/encoder-controller`, create the private headers if they do not
already exist. These files are ignored by Git. Never replace a configured local
header with an example when updating the repository.

```bash
cd /home/para/encoder-controller
test -f config/front_left.local.h || cp config/front_left.example.h config/front_left.local.h
test -f config/front_right.local.h || cp config/front_right.example.h config/front_right.local.h
```

Edit **both** local headers:

- `WIFI_SSID` and `WIFI_PASSWORD`: the 2.4 GHz network used by the boards.
- `RECEIVER_IPV4`: the Pi's reachable LAN IPv4 address. Reserve this address in
  your router so it survives Pi restarts. It is not an ESP32 address.
- `UDP_PORT`: 5005 for left, 5006 for right; match the Pi YAML below.
- `H1_IPV4`, `H2_IPV4`, `CENTER_IPV4`: leave `""` for DHCP. For stable individual
  addresses, router reservations using each board's MAC are recommended. You
  can instead put a distinct fixed IPv4 address in each of these three fields
  and set `NETWORK_GATEWAY`/`NETWORK_NETMASK`. Reserve/exclude those addresses
  from the router's DHCP pool to avoid collisions. All six boards need distinct
  source addresses; the two wheels normally use the same Pi destination IP.
- Each wheel has its own H1/H2 PPR, sign and count zero, CENTER sign/zero, and
  rate limits. Keep verification flags false until physically verified.

The helper selects wheel identity and role from its arguments. Do not define
`CASTER_WHEEL_ID` yourself. The old shared `config/local.h` is not used by these
wheel profiles. Wi-Fi, addressing and calibration changes require rebuilding
and reflashing the affected boards.

## 2. Build and flash the six boards

Label the physical boards **FL-H1, FL-H2, FL-CENTER, FR-H1, FR-H2, FR-CENTER**.
Use the pinned ESP-IDF v5.4.2 environment or Docker, as described in the firmware
README. To build all six binaries without connecting hardware:

```bash
cd /home/para/encoder-controller
for wheel in front_left front_right; do
  for role in H1 H2 CENTER; do
    ./scripts/idf.sh "$wheel" "$role" build
  done
done
```

Build outputs are isolated under `build/<wheel>/<role>/`. There is no need to
edit a header between left and right flashes.

**Disconnect the battery/UBEC before USB programming.** Connect one labeled
board at a time, identify its USB serial path, and run only its matching command:

```bash
# Connect FL-H1, then:
./scripts/idf.sh front_left H1 -p /dev/ttyACM0 flash monitor
# Connect FL-H2, then:
./scripts/idf.sh front_left H2 -p /dev/ttyACM0 flash monitor
# Connect FL-CENTER, then:
./scripts/idf.sh front_left CENTER -p /dev/ttyACM0 flash monitor

# Connect FR-H1, then:
./scripts/idf.sh front_right H1 -p /dev/ttyACM0 flash monitor
# Connect FR-H2, then:
./scripts/idf.sh front_right H2 -p /dev/ttyACM0 flash monitor
# Connect FR-CENTER, then:
./scripts/idf.sh front_right CENTER -p /dev/ttyACM0 flash monitor
```

Substitute the actual serial path each time. Exit each monitor with **Ctrl+]**
before changing boards. `flash` builds when necessary. Check the startup line's
wheel, role, MAC `device_id`, and receiver destination against the physical label.
Record the six MACs for the Pi inventory. Disconnect USB before restoring normal
battery power; keep the mechanism still during startup.

## 3. Configure and build the Pi receiver

The Pi configuration template is
[`hamr_bringup/config/caster_encoders.yaml`](../hamr_bringup/config/caster_encoders.yaml).
Use an operator-owned copy so repository updates cannot overwrite your inventory:

```bash
cp /home/para/hamr_control/hamr_bringup/config/caster_encoders.yaml \
  "$HOME/caster_encoders.yaml"
```

Keep the top-level ROS node keys `/caster_encoder_front_left` and
`/caster_encoder_front_right` and the matching `configured_wheel_id` markers.
Dual launch refuses a missing or incorrectly selected node configuration.
Under each node:

- `udp_port` must match that wheel's firmware port. `bind_address: "0.0.0.0"`
  listens on the Pi's interfaces; do not put an ESP address there.
- `expected_device_ids` is exactly three lowercase MAC strings in **H1, H2,
  CENTER order**, copied from the physical boards' startup logs. Each board must
  occupy only its verified wheel/role slot.
- `expected_source_ips` is an optional list in the same order. Leave the strings
  empty for unrestricted DHCP source addresses; fill them only when you have
  stable DHCP reservations or fixed addresses.

For example, replace these illustrative MACs with the actual front-left IDs:

```yaml
/caster_encoder_front_left:
  ros__parameters:
    wheel_id: front_left
    configured_wheel_id: front_left
    bind_address: "0.0.0.0"
    udp_port: 5005
    stale_timeout_s: 1.0
    expected_node_ids: [1, 2, 3]
    expected_device_ids: ["aa:bb:cc:dd:ee:01", "aa:bb:cc:dd:ee:02", "aa:bb:cc:dd:ee:03"]
    expected_source_ips: ["", "", ""]
    use_sim_time: false
```

The supplied template leaves MACs empty to permit commissioning. In that mode
each receiver pins the first device for each role until it restarts and reports
`inventory_verified: false`. **Fill all six expected MACs before experiments.**
Configured identity mismatches, wrong-wheel packets, legacy packets, duplicate
devices, and configured source-IP mismatches are rejected before publication
or changes to current boot/metadata. Diagnostics report rejection counts and
the most recent reason. Wheel IDs/MACs protect configuration correctness; they
are not cryptographic network authentication.

On the Pi, source its ROS installation and build the updated package in your
existing workspace. For the documented Jazzy workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/hamster_ws
colcon build --symlink-install --packages-select hamr_bringup
source install/setup.bash
```

This assumes the workspace already contains `hamr_control` and its dependencies.
For a new workspace, follow [the hardware build guide](REAL_CAR_CONTINUOUS.md#1-build-the-hardware-packages).
Allow the two inbound UDP ports and disable Wi-Fi client isolation.

## 4. Record and verify both wheels

Start passive acquisition and a single MCAP recorder:

```bash
export HAMR_BAG_ROOT="$HOME/hamr_recordings/bags"
export HAMR_BAG_NAME="two_casters_$(date +%Y%m%d_%H%M%S)"
ros2 launch hamr_bringup caster_encoders.launch.xml \
  config_file:="$HOME/caster_encoders.yaml"
```

This starts no motor controller. For a single hardware launch, add
`run_caster_encoders:=true caster_encoder_config:="$HOME/caster_encoders.yaml"`
to `hamr_HW.launch.xml`. This opt-in flag starts both receivers and uses the
hardware launch's existing recorder and Foxglove bridge. Stop the standalone
encoder launch before switching to that arrangement.

For separate terminals, keep the hardware caster flag false and add
`record_bag:=false run_foxglove:=false` to the encoder launch above when the
robot bag/bridge is already running. Only one receiver may bind each port;
do not run the firmware's standalone logger on those ports at the same time.

Each wheel publishes `packets`, `metadata`, `diagnostics`, `H1/joint_states`,
`H2/joint_states`, and `CENTER/joint_states` under its prefix. Joint names are
also qualified, e.g. `front_left_caster_h1_rotation`. Both the standard test and
Vicon recorders include all 12 topics with appropriate QoS. One bag preserves
wheel separation through topic names and the identities inside every raw packet.

Position is in radians and velocity in radians per second. For H1/H2, position
is relative to boot with the configured count zero; at 2048 PPR, 8192 x4 counts
is one revolution. CENTER joint position uses the wrapped, sign/zero-adjusted
`calibrated_alpha_rad`. With `CENTER_MAX_RPM_VERIFIED false`, CENTER's ROS
velocity array is empty and raw unwrapped angle/velocity are null; wrapped
position remains available after a valid SPI read. This is expected during
commissioning. Establish a physical rate bound before enabling live CENTER
velocity. Firmware `0.2.1` also reports the first continuity-loss reason over
USB. Follow the encoder repository's `docs/bench-checklist.md` for known-speed,
direction, and wrap tests on each wheel.

```bash
ros2 topic echo /caster_encoder/front_left/diagnostics --once
ros2 topic echo /caster_encoder/front_right/diagnostics --once
ros2 topic echo /caster_encoder/front_left/metadata --once --qos-durability transient_local
ros2 topic hz /caster_encoder/front_left/H1/joint_states
ros2 topic hz /caster_encoder/front_right/H1/joint_states
```

Before a trial, verify all three roles per wheel are fresh, metadata is present,
`inventory_verified` is true, and no rejection counter is increasing. Rotate each
labeled encoder separately and check the corresponding topic. Reboot one board:
only its boot/continuity should change, and the opposite wheel must continue.
Stop with Ctrl+C, let the recorder finalize, then inspect the printed bag path:

```bash
ros2 bag info "$HAMR_BAG_ROOT/$HAMR_BAG_NAME"
```

Use the actual suffixed path printed by the recorder if the name already existed.
Confirm all 12 topics have messages. UDP remains best effort; wheel separation
does not recover packets lost over Wi-Fi. Acquisition clocks are independent
and must not be treated as synchronized merely because the samples share a bag.

## 5. Visualize and analyze the recording

```bash
ros2 run hamr_bringup visualize_caster_bag /path/to/combined_0.mcap \
  --output "$HOME/Videos/hamr_sim/my_run"
```

This command produces the simulation video, caster encoder angles, robot path,
robot with both casters, and robot with aligned EKF. Open the output `index.html`.
Without `--output`, it writes to `hamr_control/analysis/<recording-name>/`. Use `--render none` for graphs only or `--preview-time 5` for a
single CAD frame. A recording folder or rosbag directory also works. See the
[unified workflow guide](COMBINED_CASTER_ANALYSIS.md) for dependencies and outputs.

For individual calibration or live ROS visualization, the acquisition/replay
interfaces remain:

```bash
ros2 run hamr_bringup record_caster_rotation.py record H1 \
  --wheel front_left --output /path/to/calibration/front_left
ros2 launch hamr_bringup caster_encoder_replay.launch.py \
  bag:=/path/to/bag wheel:=front_left
```

The unified command accepts `--wheel front_left` / `front_right` for a single
caster, or `--wheel legacy` for older unassigned v1 recordings. Packet processing
retains wheel, device and boot identities, and inconsistent identities are
rejected. Independent reference CSVs and clock maps must match those identities.
Mixed-wheel local JSONL exports are supported without merging the wheel axes.
