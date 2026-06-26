# compa_slam

Self-contained **visual SLAM + off-road perception** pipeline for the COMPA robot.
Adds only new files; the rest of the `hamr_holonomic_robot` workspace is untouched.

> **Continuing this work?** Read [`docs/HANDOFF.md`](docs/HANDOFF.md) — full knowledge
> transfer: current setup, what's built, and every remaining milestone (Phase 0→3).

## Pipeline

```
RealSense D455 (real)  ─┐
gz depth camera (sim)  ─┴─► RTAB-Map ──► robot pose (map→base) ─┐
                             │  rgbd_odometry / wheel odom       │
                             │  loop closure + graph             ├─► elevation_mapping (CPU)
                             │  saves maps/compa.db              │     └─► /elevation_map (grid_map)
                             └─► localization mode (map→odom)    │
                                                                 ▼
                            EXISTING planners (or_planner / astar_search / prm_builder)
                            consume /elevation_map + /costmap ──► /reference_trajectory
                                                                 ▼
                            EXISTING compa_controller (PID + Jacobian)
                            pose source switched: Vicon/`/compa/odom` ──► SLAM pose
```

## Hardware / constraints

- Camera: **RealSense D455** (RGBD + IMU, wide FOV) → IMU-fused visual-inertial odometry.
- Robot compute: **CPU-only, no NVIDIA GPU.** Therefore we use the C++ **`elevation_mapping`**
  package, NOT `elevation_mapping_cupy` (CuPy is GPU-only and will not run here). Revisit
  CuPy only if an NVIDIA GPU / Jetson is added later.
- Onboard odometry already exists: the MCU EKF (encoders + IMU) is published as `/odom` and
  `/robot_pose` by `hamr_uros_bridge/relay_node` — usable as RTAB-Map's external odom input.

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

- [~] **Phase 0 — Sim SLAM.** Depth/RGBD camera + textured world + bridge + RTAB-Map mapping
      launch are all in place. Remaining: run on the Linux/ROS box, confirm topics + a map
      builds, tune sync/QoS if needed, then add a localization launch.
- [~] **Phase 1 — Real SLAM.** Camera bring-up + trajectory recording are **done and verified on
      the Pi** (`launch/realsense.launch.py`, `config/realsense_d455.yaml`,
      `launch/record_trajectory.launch.py`, `scripts/record_compa_slam_bag`). The real bag
      mapping launch exists (`launch/rtabmap_real.launch.py`) and builds a map, but loop closures
      are rejected until the real `base_link -> camera_link` mount is measured and the recorded
      camera frame rate is raised.
- [x] **Phase 1.5 — Sim replay of a recorded map.** `scripts/map_to_sim.py` turns a recorded
      RTAB-Map cloud/`.db` (or a DEM) into a Gazebo heightmap world + `/elevation_map` +
      `/costmap`; `replay_map_sim.launch.py` drives that terrain in sim. (Code-complete,
      untested on Linux — same status as Phase 0.) See **Sim replay** below.
- [ ] **Phase 2 — Elevation mapping (CPU).** Feed RTAB-Map pose + point cloud into
      `elevation_mapping`; publish `grid_map` as `/elevation_map` (the topic the planner reads).
- [ ] **Phase 3 — Planning + control.** Point `or_planner`/`astar` at the live `/elevation_map`;
      switch `compa_controller`'s pose input to the SLAM pose; run autonomous trajectories.

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

Phase 0 SLAM stack complete (untested on hardware). Two entry points:

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
Map saves to `maps/compa_sim.db` on shutdown. Watch `rtabmap_viz` for tracked features +
loop closures. Next: a localization launch (run against the saved `.db`).

**C. Record a real trajectory (hardware D455)** — camera + onboard local odom + Vicon odom:
```bash
sudo apt install ros-jazzy-imu-filter-madgwick   # for /d455/imu with orientation
colcon build --packages-select compa_slam --symlink-install && source install/setup.bash
ros2 launch compa_slam record_trajectory.launch.py bag_name:=loop_lab_01
# drive the robot (raw wheel cmds), then Ctrl-C to finalize the bag:
ros2 topic pub /left_wheel/cmd_vel  std_msgs/msg/Float64 "{data: 3.0}"
ros2 topic pub /right_wheel/cmd_vel std_msgs/msg/Float64 "{data: 3.0}"
```
Bags land in `hamr_control/rosbags/`. Already running your own robot bringup? add `robot:=false`.
Camera only (verify topics first): `ros2 launch compa_slam realsense.launch.py use_madgwick:=false`.

**D. Build a map from a recorded bag** — replay + RTAB-Map (external EKF odom + visual loop closures):
```bash
ros2 launch compa_slam rtabmap_real.launch.py \
    bag:=$HOME/hamster_ws/src/hamr_control/rosbags/loop_lab_01 \
    database_path:=$HOME/hamster_ws/src/hamr_control/compa_slam/maps/compa_real.db
# Ctrl-C after the bag finishes -> the .db is saved.
```
> Tested 2026-06-24: builds (~134 nodes) but loop closures get rejected until the real
> `base_link -> camera_link` mount is measured (set `mount_*` in `realsense.launch.py`) and the
> recorded camera frame rate is raised. See `docs/HANDOFF.md` M1.4.

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
