# compa_slam

Self-contained **visual SLAM + off-road perception** pipeline for the COMPA robot.
It integrates with the existing HAMR bringup, encoder/IMU odometry, controller,
and serial bridge through their current topic and launch contracts.

> **Continuing this work?** Read [`docs/HANDOFF.md`](docs/HANDOFF.md) — full knowledge
> transfer: current setup, what's built, and every remaining milestone (Phase 0→3).

> **Run without Vicon:** [`docs/LOCALIZATION_RUNTIME.md`](docs/LOCALIZATION_RUNTIME.md)
> covers the completed wheel/IMU local-control runtime, RTAB-Map saved-DB
> corrections, staged tests, and `waypoint_traj_simple` behavior.

> **Record a real environment:** [`docs/MAPPING_CAPTURE.md`](docs/MAPPING_CAPTURE.md)
> is the encoder-v4 preflight, safe driving, bag validation, and map-build runbook.

## Pipeline

```
RealSense D455 (real)  ─┐
gz depth camera (sim)  ─┴─► RTAB-Map ──► map→odom correction ────┐
                             │  loop closure + localization       ├─► elevation_mapping (CPU)
                             │  saves/loads maps/compa.db         │     └─► /elevation_map
                             │                                    ▼
wheel encoders + BNO055 ──► local EKF ──► odom→base_link    planners / map route
                                  │                              │
                                  │      map-reference adapter ◄─┘
                                  │                │ odom reference
                                  ▼                ▼
                            local hardware controller ──► wheel/turret commands
```

## Hardware / constraints

- Camera: **RealSense D455** (RGBD + IMU, wide FOV) → IMU-fused visual-inertial odometry.
- Robot compute: **CPU-only, no NVIDIA GPU.** Therefore we use the C++ **`elevation_mapping`**
  package, NOT `elevation_mapping_cupy` (CuPy is GPU-only and will not run here). Revisit
  CuPy only if an NVIDIA GPU / Jetson is added later.
- Onboard odometry already exists: encoders + IMU are fused by `robot_localization` and
  published as `/local_HAMR/odom` plus `odom -> base_link`; RTAB-Map uses the
  continuous TF by default for compatibility with legacy bags whose partly
  unobservable EKF covariance triggered false map resets. Topic odometry remains
  available after covariance validation.

## Directory layout

| Dir | Holds | Status |
|-----|-------|--------|
| `launch/` | launch files | `slam_sim.launch.py`, `rtabmap_sim.launch.py`, `replay_map_sim.launch.py` ✔ |
| `config/` | YAML params (bridge, rtabmap, realsense, elevation map) | `gazebo_bridge_slam.yaml`, `rtabmap.yaml` ✔ |
| `scripts/`| sim-setup tools | `map_to_sim.py` (recorded map → Gazebo heightmap world) ✔ |
| `urdf/`   | D455 sensor (`compa_d455.urdf.xacro`) + spawnable robot (`compa_slam.urdf.xacro`) | ✔ |
| `worlds/` | textured Gazebo world for visual odometry (`feature_world.sdf`) | ✔ |
| `rviz/`   | RViz configs for SLAM viz (using `rtabmap_viz` for now; custom config TBD) | — |
| `maps/`   | saved `*.db` + exported clouds (git-ignored) **and** generated `*_heightmap.png`/`*.yaml`/`*.sdf` | working dir |
| `bags/`   | raw RealSense recordings (**git-ignored**, created on first record) | working dir |

## Phased plan (status)

- [~] **Phase 0 — Sim SLAM.** The TF ownership, Gazebo IMU bridge, and RTAB-Map path were
      validated headlessly (including a synthetic RGB-D map build). Full OGRE2 RGB-D rendering
      of the textured world still needs a host with a working OpenGL display.
- [~] **Phase 1 — Real SLAM.** Camera bring-up + trajectory recording are **done and verified on
      the Pi** (`launch/realsense.launch.py`, `config/realsense_d455.yaml`,
      `launch/record_trajectory.launch.py`, `scripts/record_compa_slam_bag`). The real bag
      mapping launch exists (`launch/rtabmap_real.launch.py`) and builds a map. The Vicon-free
      localization/local-control runtime is code-complete (`localization_runtime.launch.py`) but
      still needs hardware validation. A new encoder-v4 map still requires a measured real
      `base_link -> camera_link` mount and a sustained camera rate.
- [~] **Phase 1.5 — Sim replay of a recorded map.** `scripts/map_to_sim.py` turns a recorded
      RTAB-Map cloud/`.db` (or a DEM) into a Gazebo heightmap world + `/elevation_map` +
      `/costmap`; `replay_map_sim.launch.py` drives that terrain in sim. Launch/TF paths are
      validated; rendered terrain/physics inspection remains display-host work. See **Sim replay**.
- [ ] **Phase 2 — Elevation mapping (CPU).** Feed RTAB-Map pose + point cloud into
      `elevation_mapping`; publish `grid_map` as `/elevation_map` (the topic the planner reads).
- [~] **Phase 3 — Planning + control.** Vicon-free local EKF control plus map-corrected simple
      trajectories are code-complete. Live elevation/cost maps and planner-path integration remain.

## Build

From the workspace root:

```bash
colcon build --packages-select compa_slam --symlink-install
source install/setup.bash
```

> This package is launch/config/assets only (ament_cmake, no compiled nodes). The SLAM stack
> itself comes from apt: `ros-jazzy-rtabmap-ros`, `ros-jazzy-realsense2-camera`,
> `ros-jazzy-imu-filter-madgwick`. `elevation_mapping` is installed in Phase 2.

## Status

Phase 0 SLAM stack is headless-validated; rendered Gazebo inspection remains. Two entry points:

**A. Topic checkpoint** — sim + camera only, confirm the D455 reaches ROS:
```bash
colcon build --packages-select compa_slam --symlink-install
source install/setup.bash
ros2 launch compa_slam slam_sim.launch.py
# another terminal:
ros2 topic hz /d455/color/image_raw          # ~30 Hz
ros2 topic hz /d455/depth/image_rect_raw      # ~30 Hz
ros2 topic echo /d455/color/camera_info --once
ros2 topic hz /d455/imu                        # ~200 Hz
```

**B. Build a map** — sim + camera + RTAB-Map (mapping mode):
```bash
ros2 launch compa_slam rtabmap_sim.launch.py
# another terminal — drive around to build the map:
ros2 topic pub /left_wheel/cmd_vel  std_msgs/msg/Float64 "{data: 3.0}"
ros2 topic pub /right_wheel/cmd_vel std_msgs/msg/Float64 "{data: 3.0}"
```
Map saves to `~/.ros/compa_sim.db` by default on shutdown. Watch `rtabmap_viz` for tracked features +
loop closures. A dedicated sim localization launch is still pending; the real-hardware saved-DB
runtime is `localization_runtime.launch.py`.

**C. Record a real trajectory (hardware D455)** — camera + onboard local odom + optional Vicon:
```bash
sudo apt install ros-jazzy-imu-filter-madgwick   # for /d455/imu with orientation
colcon build --packages-select compa_slam --symlink-install && source install/setup.bash
ros2 launch compa_slam record_trajectory.launch.py \
  bag_name:=map_encoder_v4_01 run_controller:=false run_foxglove:=false \
  use_orientation:=true use_mag:=false \
  mount_x:=MEASURED_X mount_y:=MEASURED_Y mount_z:=MEASURED_Z \
  mount_roll:=MEASURED_ROLL mount_pitch:=MEASURED_PITCH mount_yaw:=MEASURED_YAW
```
Bags land in `hamr_control/rosbags/`. Already running your own robot bringup? add `robot:=false`.
Use a continuous command source at about 0.10-0.15 m/s; a one-hertz default
`ros2 topic pub` will trip the 0.25-second relay watchdog. Follow the full
[mapping capture runbook](docs/MAPPING_CAPTURE.md) before driving. In
particular, measure the camera transform in the deployed `base_link` axes: the
current holonomic stack's `pi/2` kinematic offset means wheel-forward is +Y.

**D. Build a map from a recorded bag** — replay + RTAB-Map (EKF odom + visual loop closures):
```bash
ros2 launch compa_slam rtabmap_real.launch.py \
    bag:=$HOME/hamster_ws/src/hamr_control/rosbags/map_encoder_v4_01 \
    database_path:=$HOME/hamster_ws/src/hamr_control/compa_slam/maps/compa_real_encoder_v4.db \
    use_odom_topic:=true
# Ctrl-C after the bag finishes -> the .db is saved.
```
Use topic mode for a new capture only after `analyze_rosbag.py` reports zero
non-finite covariance samples and zero RTAB-Map reset-guard crossings; leave
the TF compatibility default for older bags recorded with the unobservable EKF preset.
> The first June capture rejected its loop closures, but the current July
> `compa_real.db` is a valid 227 MB database with 443 nodes, 47 global closures,
> and 18 local-space closures. It predates encoder v4, so preserve it as evidence
> and build a new database from a v4 bag for current localization. The measured
> `base_link -> camera_link` transform remains mandatory. See `docs/HANDOFF.md` M1.4.

## Sim replay (record -> convert -> run in sim)

Turn a **recorded real-world map** into a Gazebo simulation of the same terrain. The bridge
is a **heightmap PNG**: it drives both the Gazebo `<heightmap>` terrain *and* the planner's
`/elevation_map` + `/costmap` (via `hamr_control_cpp/cost_map_publisher`), so physics and
planner see identical, aligned terrain.

**Which map does the sim use — RTAB-Map or the elevation/"cupy" map?** Neither is fed to
Gazebo directly. RTAB-Map gives you localization + a dense **point cloud**; the elevation map
is just that cloud **rasterised** into a height grid (we use the **CPU** path — `cupy` is GPU
and unusable here). `map_to_sim.py` does that rasterisation offline, so you feed it the
RTAB-Map cloud/`.db` (recommended) **or** an existing DEM — both become the same heightmap.
Full reasoning: [`docs/HANDOFF.md` §10](docs/HANDOFF.md).

```bash
# from a recorded RTAB-Map database (maps/compa_real.db):
ros2 run compa_slam map_to_sim.py --db maps/compa_real.db --name compa_real
#   or from an exported cloud:  --cloud maps/compa_real.ply
#   tune: --res 0.05  --zclip 1 99  --size 40  --agg max|mean  --bits 8|16
# -> writes maps/compa_real_heightmap.png + compa_real.yaml + compa_real.sdf

colcon build --packages-select compa_slam --symlink-install && source install/setup.bash

ros2 launch compa_slam replay_map_sim.launch.py map:=compa_real          # drive the terrain
ros2 launch compa_slam replay_map_sim.launch.py map:=compa_real run_planner:=true  # + or_planner
```

> Caveat: `cost_map_publisher` centres `/elevation_map` at the origin but hard-codes the
> `/costmap` origin at (-20,-20), so for **planner** runs generate with `--size 40` until a
> parameterised publisher replaces it. Just *driving* the terrain works at any extent.
