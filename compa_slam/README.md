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
| `launch/` | launch files | `slam_sim.launch.py`, `rtabmap_sim.launch.py` ✔ |
| `config/` | YAML params (bridge, rtabmap, realsense, elevation map) | `gazebo_bridge_slam.yaml`, `rtabmap.yaml` ✔ |
| `urdf/`   | D455 sensor (`compa_d455.urdf.xacro`) + spawnable robot (`compa_slam.urdf.xacro`) | ✔ |
| `worlds/` | textured Gazebo world for visual odometry (`feature_world.sdf`) | ✔ |
| `rviz/`   | RViz configs for SLAM viz (using `rtabmap_viz` for now; custom config TBD) | — |
| `maps/`   | saved `compa.db` databases (**git-ignored**) | working dir |
| `bags/`   | raw RealSense recordings (**git-ignored**, created on first record) | working dir |

## Phased plan (status)

- [~] **Phase 0 — Sim SLAM.** Depth/RGBD camera + textured world + bridge + RTAB-Map mapping
      launch are all in place. Remaining: run on the Linux/ROS box, confirm topics + a map
      builds, tune sync/QoS if needed, then add a localization launch.
- [ ] **Phase 1 — Real SLAM.** Bring up the D455 + IMU madgwick filter; record a raw sensor
      bag while driving the space; build & save `maps/compa.db`; verify localization mode.
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
