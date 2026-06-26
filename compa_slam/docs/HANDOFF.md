# COMPA Visual-SLAM Pipeline — Handoff / Build Brief

> Purpose: a self-contained brief to continue and finish this work without the original
> chat history. Read top-to-bottom once, then use the **Milestones** section as the task
> list. Written for ROS 2 Jazzy / Ubuntu 24.04.

---

## 1. Mission

Add a **visual SLAM + off-road perception + planning** pipeline to the existing COMPA
robot, as a **separate, additive package** (`compa_slam`) that does not modify any existing
code. End state:

```
RealSense D455 RGBD ─► RTAB-Map (visual SLAM) ─► robot pose (map→base) + point cloud
                                                      │
                                                      ▼
                                       elevation_mapping (CPU) ─► /elevation_map (grid_map)
                                                      │
                                                      ▼
                       EXISTING planners (or_planner / astar_search / prm_builder)
                                                      │  ─► planned path
                                                      ▼
                       path-follower ─► /reference_trajectory ─► compa_controller (PID+Jac)
                                                      │  (pose source = SLAM, not Vicon)
                                                      ▼
                          sim: ros_gz_bridge   |   hardware: relay_node → serial → MCU
```

Validate everything **in sim first** (ground truth available), then deploy to the real robot.

---

## 2. Hard constraints (do not violate)

- **Compute is CPU-only — no NVIDIA GPU.** Therefore **`elevation_mapping_cupy` is NOT usable**
  (CuPy is GPU-only). Use the C++ CPU `elevation_mapping`, or a custom grid_map point-cloud
  node (see Milestone M2). Revisit CuPy only if a Jetson/NVIDIA GPU is added.
- **Camera = Intel RealSense D455** (RGBD + IMU, wide FOV).
- **Additive only.** New files live under `compa_slam/`. Do **not** edit existing packages
  (`compa_description`, `hamr_*`, etc.). Match repo conventions (ament_cmake, `package.xml`
  format 3, style of existing launch/xacro).
- The robot/sim runs on **Linux**. (Dev editing also happens on a Windows machine; nothing
  here runs on Windows.)

---

## 3. The existing system (what was here before this work)

ROS 2 Jazzy workspace `hamr_holonomic_robot`. Relevant pieces:

| Package | Role | Key topics |
|---|---|---|
| `compa_control_py` | **compa_controller.py** — 5-axis PID + inverse-Jacobian controller (x,y,roll,pitch,yaw). Reads pose + gimbal TF, outputs joint velocities. | sub `/compa/odom` (Odometry), `/tf`, `/reference_trajectory`; pub `/{left,right}_wheel/cmd_vel`, `/{roll,pitch,yaw}/cmd_vel` |
| `reference_trajectory` | **waypoint_traj_simple.py** — emits a moving setpoint from hardcoded waypoints. | pub `/reference_trajectory` (hamr_interfaces/ReferenceTraj) |
| `hamr_control_cpp` | Off-road planners: `or_planner` (A* on costmap+elevation w/ traversability: `max_tilt_deg`, `max_step_m`, slope), `astar_search`, `prm_builder`, `compa_path_tracer` (odom→Path viz), and **image→map** sources `ImageToGridmap`, `map_publisher`, `cost_map_publisher`. | `or_planner` sub `/elevation_map` (grid_map_msgs/GridMap), `/costmap` (OccupancyGrid), `/goal_pose`; pub a path |
| `compa_description` | Robot URDF. `compa_back.urdf.xacro` (body + gimbal + gz plugins) includes `camera.urdf.xacro` (**plain RGB** camera, `/camera/image_raw`). | — |
| `hamr_bringup` | Launch + gz worlds + `gazebo_bridge_compa.yaml` (gz↔ROS) + bag recording. | — |
| `hamr_uros_bridge` | **relay_node.cpp** — serial bridge to the MCU. Sends wheel/turret cmd_vel; receives encoder ticks + BNO055 IMU. | pub `/imu/data`, `/{left,right,turret}/encoder_ticks`; sub `/{left,right}_wheel/cmd_vel`, `/turret/cmd_vel` |
| `hamr_odometry` | **holonomic_odom_node.py** — integrates encoder ticks into wheel odometry. | pub `/wheel_odom`, `/HAMR_turret/odom` |
| `hamr_interfaces` | msgs: `ReferenceTraj` (x,y,yaw,roll,pitch + dots), `LiveGains`, `StateError` | — |

**Localization today (pre-SLAM):**
- **Sim:** Gazebo ground-truth odometry `/compa/odom`.
- **Hardware:** external **Vicon** mocap (`/HAMR_base/odom`) + onboard fused odometry. The
  onboard chain is: `relay_node` (serial → `/imu/data` + encoder ticks) → `holonomic_odom_node`
  (ticks → `/wheel_odom`) → **robot_localization EKF** (`/wheel_odom` + `/imu/data` →
  **`/local_HAMR/odom`** and the `odom→base_link` TF). So the **onboard local odom topic is
  `/local_HAMR/odom`** (`/wheel_odom` is the raw pre-EKF version). There is **no** `/odom` or
  `/robot_pose` from relay_node — earlier drafts of this doc said so; that was wrong. No
  camera-based localization yet. Vicon only works inside the mocap arena — the core reason
  SLAM is being added.

**Key alignment win:** `or_planner` was already written to consume a `grid_map` `/elevation_map`
and `/costmap`. Today those come from **heightmap PNG images** (`ImageToGridmap`/`map_publisher`/
`cost_map_publisher`). The SLAM pipeline simply **replaces the image source with live
perception** (`elevation_mapping`). The planner itself does not change.

---

## 4. What has been built so far — the `compa_slam` package

ament_cmake package at `hamr_holonomic_robot/compa_slam/`. Status: **Phase 0 sim SLAM is
code-complete but UNTESTED on hardware/Linux** (author had no ROS environment).

| File | What it is | Notes / decisions |
|---|---|---|
| `package.xml`, `CMakeLists.txt` | ament_cmake manifest + install of `launch config urdf worlds rviz` | rtabmap/realsense/imu deps are `exec_depend` so build doesn't require them installed |
| `README.md` | living doc + phase status + run commands | keep updated as milestones complete |
| `.gitignore` | ignores `maps/*.db`, `bags/` | maps & bags are large working dirs |
| `urdf/compa_d455.urdf.xacro` | **simulated D455** (gz `rgbd_camera` + `imu` sensors). Links `d455_link`, `d455_optical_link` (REP-103). Mounted on `yaw_plate_link` at `0.2 0 0.2`, pitched **~20° down**. | distinct `d455_*` names avoid clashing with stock `camera_link`. Needs `Sensors`+`Imu` world plugins. gz pubs: `/d455/image`, `/d455/depth_image`, `/d455/points`, `/d455/camera_info`, `/d455/imu` |
| `urdf/compa_slam.urdf.xacro` | spawnable robot = `compa_back.urdf.xacro` (existing body) **+** the D455 | `name="compa"` (keeps `/model/compa/...` topics). Pulls in the stock RGB camera too (harmless, unused) |
| `worlds/feature_world.sdf` | textured 16×16 m room: 4 colored walls + 8 distinct-colored pillars + sun | world name `'empty'` (bridge hardcodes `/world/empty/...`). **Adds the `Imu` system plugin** (stock worlds lack it). Feature-rich on purpose — visual odometry needs texture |
| `config/gazebo_bridge_slam.yaml` | ros_gz bridge: robot control/state **+** D455, renamed to RealSense-style names | `/d455/color/image_raw`, `/d455/depth/image_rect_raw`, `/d455/color/camera_info`, `/d455/depth/color/points`, `/d455/imu` |
| `config/rtabmap.yaml` | shared RTAB-Map params for all 3 nodes (`/**` wildcard) | `frame_id: base_link`, `approx_sync`, `qos: 1`, `Reg/Force3DoF: true`, IMU gravity. **Reused verbatim on real D455** (only `use_sim_time` flips) |
| `launch/slam_sim.launch.py` | sim bring-up: gz + world + spawn + robot_state_publisher + bridge + optional static `map→odom` + optional RViz | sets `GZ_SIM_RESOURCE_PATH` so gz resolves `package://` meshes; `use_sim_time: true`. Args: `world, use_rviz, publish_map_odom_tf, x,y,z,yaw` |
| `launch/rtabmap_sim.launch.py` | **mapping-mode SLAM**: includes `slam_sim` (rviz off, static map→odom off) + `rgbd_odometry` + `rtabmap` + `rtabmap_viz` | remaps canonical `rgb/image,depth/image,rgb/camera_info,imu` → `/d455/...`. Saves `maps/compa_sim.db`. Args: `database_path, use_rtabmap_viz` |

### Canonical SLAM topic contract (sim AND real feed these)
| Canonical ROS topic | Meaning |
|---|---|
| `/d455/color/image_raw` | RGB |
| `/d455/depth/image_rect_raw` | depth registered to color ("aligned depth") |
| `/d455/color/camera_info` | intrinsics |
| `/d455/depth/color/points` | point cloud (for elevation mapping) |
| `/d455/imu` | IMU **with orientation** (sim: gz provides it; real: madgwick must produce it) |

### TF tree (must stay intact)
```
map ─(rtabmap)→ odom ─(rgbd_odometry OR wheel odom)→ base_link ─(URDF static)→ … → yaw_plate_link → d455_link → d455_optical_link
```

---

## 5. RTAB-Map knowledge primer (so you don't need external docs first)

- RTAB-Map is **3 nodes**: `rgbd_odometry` (fast, drifting pose → `odom→base_link`),
  `rtabmap` (graph + loop closure → corrects drift via `map→odom`, builds the `.db`),
  `rtabmap_viz` (visualization).
- **Inputs** (must be time-synced; hence `approx_sync:=true`): RGB, **aligned** depth,
  camera_info, (optional) IMU. In gz the depth is already registered to color.
- **Mapping mode** (`--delete_db_on_start`): drive around, revisit places to trigger loop
  closures, the `.db` saves on shutdown. **The `.db` IS the map.**
- **Localization mode** (`localization:=true` / `Mem/IncrementalMemory false`): loads the
  `.db`, stops growing it, publishes live `map→odom` + `/rtabmap/localization_pose`.
- **Odometry choice:** visual (`rgbd_odometry`) is simplest but fragile on low texture / fast
  motion. On a wheeled robot, feeding **wheel/EKF odom** (already on `/odom` from relay_node)
  as external odometry and letting `rtabmap` add visual loop closures is more robust off-road.
- **Always also record a raw rosbag** of the camera while mapping — it's the portable artifact
  you replay to rebuild/re-tune a map without re-driving. (The `.db` is space-specific.)
- **A real-world `.db` is NOT portable into sim.** Sim and real share the pipeline/config, not
  the map.

---

## 6. Milestones (the task list)

Each milestone has **Do / Done-when / Watch-outs**. Work top-down.

### Phase 0 — Sim SLAM (finish it)

**M0.1 — Topics alive.**
- Do: `colcon build --packages-select compa_slam --symlink-install`; `apt install ros-jazzy-rtabmap-ros`;
  `ros2 launch compa_slam slam_sim.launch.py`.
- Done-when: `ros2 topic hz` shows `/d455/color/image_raw` (~30), `/d455/depth/image_rect_raw`
  (~30), `/d455/imu` (~200); `camera_info` echoes valid intrinsics.
- Watch-outs: gz may name the rgbd subtopics differently — verify with
  `ros2 topic list | grep d455` and fix `gz_topic_name` in `gazebo_bridge_slam.yaml`. Confirm
  points type is `gz.msgs.PointCloudPacked`.

**M0.2 — Map builds.**
- Do: `ros2 launch compa_slam rtabmap_sim.launch.py`; drive with
  `ros2 topic pub /left_wheel/cmd_vel std_msgs/msg/Float64 "{data: 3.0}"` (and right). Loop the room.
- Done-when: `rtabmap_viz` shows tracked features + at least one loop closure; `map→odom` is
  published (`ros2 run tf2_tools view_frames`); `maps/compa_sim.db` is written on Ctrl-C.
- Watch-outs: "Did not receive data"/sync → flip `qos: 1→2` or raise `approx_sync_max_interval`.
  Odometry stuck on "waiting for IMU" → `wait_imu_to_init: false`. Odom lost → drive slower /
  confirm features in view.

**M0.3 — Localization launch.** *(new file: `launch/rtabmap_localization.launch.py`)*
- Do: copy `rtabmap_sim.launch.py`; set rtabmap to localization (`Mem/IncrementalMemory: false`,
  drop `--delete_db_on_start`, keep `database_path` pointing at the saved `.db`); optionally
  `RGBD/OptimizeMaxError`, `Mem/InitWMWithAllNodes: true`.
- Done-when: starting on the saved map, `/rtabmap/localization_pose` locks on and `map→odom` is
  stable while driving (no large jumps).

**M0.4 (optional) — Teleop helper.** *(new node or launch)*
- The robot takes **raw wheel** `cmd_vel` (Float64), not Twist. Add a tiny `twist→wheels` mixer
  (`/cmd_vel` Twist → `/{left,right}_wheel/cmd_vel`) so `teleop_twist_keyboard` works. Makes
  mapping much easier than `topic pub`.

**M0.5 (optional) — `rviz/slam.rviz`.** Preconfigured RViz (TF, RobotModel, Image, PointCloud2,
`/rtabmap/grid_map`, `/rtabmap/cloud_map`). Hand-writing `.rviz` is error-prone; alternatively
just rely on `rtabmap_viz`.

### Phase 1 — Real D455 SLAM

**M1.1 — RealSense driver launch. ✔ DONE** *(`launch/realsense.launch.py`, `config/realsense_d455.yaml`)*
- Built + **verified on the Pi with the camera attached.** Publishes the canonical `/d455/...`
  topics via absolute remaps from the driver's native `/d455/d455_camera/...` names, so
  `rtabmap.yaml` is reused unchanged. Verified rates: color ~30 Hz, aligned depth ~30 Hz,
  imu_raw ~200 Hz, valid 640×480 intrinsics.
- **Pi-specific gotcha (important):** the RealSense *defaults* (1280×720 color + depth + both
  infrared streams) saturate the Pi's USB → `"Frames didn't arrive within 5 seconds"` and no
  data. Fix baked into `config/realsense_d455.yaml`: drop IR streams (`enable_infra1/2:=false`),
  use `640x480x30` for color+depth, and `initial_reset:=true`. Point cloud is OFF by default
  (CPU-bound on the Pi; RTAB-Map rebuilds it on replay) — enable with `pointcloud:=true` when
  you reach Phase 2 elevation mapping.
- TF: the driver roots its frames at `camera_link` (NOT `d455_camera_link`); the color image
  frame is `camera_color_optical_frame`. The hardware bringup runs no robot_state_publisher,
  so `realsense.launch.py` also publishes a static `base_link → camera_link` (mount offsets are
  launch args — **MEASURE and set them**; defaults mirror the sim mount).

**M1.2 — IMU orientation. ✔ DONE (needs the package installed)** On the real D455 the raw IMU has
no orientation. `realsense.launch.py` runs `imu_filter_madgwick` (`use_mag:=false`, `publish_tf:=false`)
subscribing the raw IMU `/d455/imu_raw` and publishing **`/d455/imu`** (with orientation). In sim gz
already gives orientation, so this node is real-only. **Install:** `apt install ros-jazzy-imu-filter-madgwick`
(not yet installed; until then launch with `use_madgwick:=false` and the bag still captures `/d455/imu_raw`).

**M1.3 — D455 on the real robot's TF.** The hardware URDF must contain `base_link → d455_optical`
(measure the physical mount). Either include a hardware variant of `compa_d455.urdf.xacro` in the
real robot description, or publish a measured `static_transform_publisher`. Without this, RTAB-Map
gets no camera extrinsics and fails. Calibrate the mount offset.

**M1.4 — Odometry on hardware. ✔ launch written** *(`launch/rtabmap_real.launch.py`)* Feeds
**`/local_HAMR/odom`** (the robot_localization EKF output) to rtabmap as external odometry
(`odom_topic:=/local_HAMR/odom`, default `visual_odometry:=false`) and lets rtabmap add visual loop
closures — more robust off-road than pure visual, and the only clean option when replaying a bag (the
bag already carries odom + the `odom→base_link` TF, which would collide with rgbd_odometry). NOTE:
there is no `/odom` from relay_node; use `/local_HAMR/odom`, or `/wheel_odom` for raw pre-EKF.

**⚠ First real map (2026-06-24, `loop_lab_01`): builds but 0 loop closures (18 rejected).** Map saved
fine (59 MB, ~134 nodes) but every visual loop closure was rejected by `RGBD/OptimizeMaxError` ("wrong
loop closure ... graph error ratio" / "transform too large"). Root causes, in priority order:
  1. **The `base_link→camera_link` mount transform is still the placeholder** from `realsense.launch.py`
     — wrong extrinsics make the camera geometry disagree with wheel/IMU odom, so loop closures look
     wrong and get rejected. **Measure the real mount first.**
  2. **Recorded camera frame rate was low** (color ~5.5 Hz, depth ~11.7 Hz — the Pi dropped frames
     while recording the full stack to SD). Fewer/sparser frames = weaker feature matching. Record
     lighter (e.g. color+depth at 15 fps, drop foxglove, or record the camera to its own bag).
  3. Only after 1+2: if odom drift is the remaining issue, relax `RGBD/OptimizeMaxError` (3.0→~5.0).
     Do NOT relax it first — that just accepts corrupt loop closures.

**M1.5 — Record + build the real map.** *(record tooling ✔ DONE)*
- Record a trajectory bag while driving the space — **one command brings up camera + onboard
  odom + recorder:**
  `ros2 launch compa_slam record_trajectory.launch.py bag_name:=loop_lab_01`
  (or just the recorder: `ros2 run compa_slam record_compa_slam_bag <name>`). It captures camera
  (`/d455/color/image_raw`, `/d455/depth/image_rect_raw`, `/d455/color/camera_info`, `/d455/imu`,
  `/d455/imu_raw`, and `/d455/depth/color/points` if `pointcloud:=true`), onboard local odom
  (`/local_HAMR/odom`, `/wheel_odom`, `/imu/data`, encoder ticks), Vicon (`/HAMR_base/odom`,
  `/HAMR_turret/odom` + poses), commands, and `/tf`+`/tf_static`. QoS is handled by
  `config/record_slam_qos.yaml` (camera/cmd_vel → best_effort, `/tf_static` → transient_local so
  the latched camera extrinsics are captured). Bags land in `hamr_control/rosbags/`.
- Build & save `maps/compa_real.db` (live, or by replaying the bag to re-tune). **Needs**
  `apt install ros-jazzy-rtabmap-ros` (not yet installed).
- Done-when: a globally-consistent real map with loop closures exists.

**M1.6 — Real localization.** Run the localization launch against `compa_real.db`; confirm stable
pose while driving the real space.

### Phase 2 — Elevation mapping (CPU)

**M2.0 — Pick the elevation engine (decision point).**
- Preferred: ANYbotics **`elevation_mapping`** (C++, CPU) ROS 2 port (needs `grid_map`, `kindr`,
  `message_logger`; likely a **source build** on Jazzy).
- Fallback if that's painful on Jazzy: a **custom node** turning `/d455/depth/color/points` +
  pose into a `grid_map` height layer using **`grid_map_pcl`**. This integrates directly with the
  existing grid_map planners and avoids heavy deps. *(Recommended if the port fights you.)*
- **Do NOT use `elevation_mapping_cupy`** (no GPU).

**M2.1 — Elevation launch + config.** *(new: `launch/elevation_mapping.launch.py`,
`config/elevation_map.yaml`)*
- Inputs: point cloud `/d455/depth/color/points` + robot pose TF (`map→base` from RTAB-Map).
- Output: `grid_map` → **publish/remap to `/elevation_map`** (exactly what `or_planner` expects).
- Tune: map size/resolution, RealSense depth noise model, voxel downsample, fused-vs-raw.
- Done-when: `/elevation_map` shows a sensible height surface that tracks the robot.

**M2.2 — Costmap/traversability.** `or_planner` also subscribes `/costmap` (OccupancyGrid). Derive
a traversability/costmap layer from elevation (slope/step filters via `grid_map_filters`, or adapt
the existing `cost_map_publisher`). Done-when both `/elevation_map` and `/costmap` are live.

### Phase 3 — Planning + control integration

**M3.1 — Planner on live maps.** Point `or_planner`/`astar`/`prm` at the live `/elevation_map` +
`/costmap`. Send a goal (`/goal_pose` from RViz). Done-when a planned path appears and respects
slope/step limits. Mind frames (planner works in `map`).

**M3.2 — Path follower.** *(new node)* The planners emit a `nav_msgs/Path`; `compa_controller`
consumes a single moving setpoint `/reference_trajectory` (hamr_interfaces/ReferenceTraj). Write a
follower that walks the planned Path and emits ReferenceTraj setpoints (model it on
`reference_trajectory/waypoint_traj_simple.py`, but driven by the planned Path instead of hardcoded
waypoints).

**M3.3 — SLAM pose → controller.** *(new adapter node)* `compa_controller` reads pose from
`/compa/odom` (sim truth) / Vicon. Replace with the SLAM pose: look up TF `map→base_link` and
publish `nav_msgs/Odometry` (frame `map`) on the topic the controller reads, or remap. The
controller computes x/y/yaw error in the world frame, which is now `map`.

**M3.4 — Closed loop in sim.** goal → plan (elevation) → path follower → reference traj →
compa_controller → wheels. Done-when the robot autonomously drives a planned off-road path using
**SLAM** localization in sim.

**M3.5 — Closed loop on hardware.** Hardware bringup that runs: RealSense + madgwick + rtabmap
(localization) + elevation_mapping + planner + path follower + pose adapter + compa_controller +
**relay_node** (serial to MCU). No gz bridge on hardware. Done-when the real robot navigates a
planned trajectory localized by the camera (no Vicon).

---

## 7. Known risks / first-run tuning (expected, not bugs)

1. **gz rgbd subtopic names** — verify and fix bridge `gz_topic_name`s (M0.1).
2. **RGB/depth/IMU sync** — `qos` 1↔2, `approx_sync_max_interval`, `wait_imu_to_init`.
3. **Two cameras in sim robot** (RGB + D455) — cosmetic; to remove cleanly you'd have to
   parameterize the camera include in `compa_description` (out of "additive-only" scope unless you
   decide to relax it).
4. **`elevation_mapping` on ROS 2 Jazzy** — may need source build; have the custom grid_map_pcl
   fallback ready.
5. **Frames & `use_sim_time`** — keep `map→odom→base_link` clean; `use_sim_time true` in sim,
   `false` on hardware (it's centralized in `config/rtabmap.yaml` + the launch params).
6. **D455 min range ~0.4–0.6 m** and depth noise grow with distance — affects mapping near
   obstacles and elevation quality.

---

## 8. Working conventions

- New files under `compa_slam/` only; don't edit existing packages.
- Update `compa_slam/README.md` phase status + this file's milestone checkboxes as you go.
- Match existing style (ament_cmake, package.xml format 3, launch/xacro idioms).
- Reuse the canonical `/d455/...` topic contract so sim and hardware share `rtabmap.yaml`.

## 9. Command cheat-sheet

```bash
# build
colcon build --packages-select compa_slam --symlink-install && source install/setup.bash

# sim: camera only (verify topics)
ros2 launch compa_slam slam_sim.launch.py

# sim: SLAM mapping (drive around, builds maps/compa_sim.db)
ros2 launch compa_slam rtabmap_sim.launch.py
ros2 topic pub /left_wheel/cmd_vel  std_msgs/msg/Float64 "{data: 3.0}"
ros2 topic pub /right_wheel/cmd_vel std_msgs/msg/Float64 "{data: 3.0}"

# inspect
ros2 topic list | grep d455
ros2 run tf2_tools view_frames
```

---

## 10. Sim replay of a recorded map  (record → convert → run in sim)

This is the "I recorded a map, now drive it in sim" pipeline. New, additive, lives in
`compa_slam/scripts/` + `compa_slam/launch/`. Nothing in the SLAM mapping flow changes.

### 10.1 The key idea: a heightmap is the shared currency
The existing sim already proves the pattern (see `hamr_bringup/worlds/compa_OR_test_map.sdf`
+ `hamr_control_cpp/cost_map_publisher.cpp`): **one grayscale heightmap PNG drives BOTH**
- the **Gazebo terrain** (`<heightmap>` geometry → the robot physically drives on it), and
- the **planner inputs** (`cost_map_publisher` reads the same PNG → `/elevation_map`
  grid_map + `/costmap`, which `or_planner` consumes).

So "run a recorded map in sim" = **turn whatever SLAM produced into a heightmap PNG**
(+ a sidecar of extent/height/resolution), then generate a world SDF and point the planner
publisher at the same PNG. Sim physics and planner map are then identical and aligned.

### 10.2 RTAB-Map vs. the elevation / "cupy" terrain map — which feeds the sim?
Short answer: **neither is fed to Gazebo directly; both are upstream of the heightmap.**

- **RTAB-Map** is the *localizer + dense 3D reconstruction*. Its useful output here is the
  dense **point cloud** (`/rtabmap/cloud_map`, or exported from the `.db` with
  `rtabmap-export --cloud`). RTAB-Map alone does **not** give a 2.5D terrain surface — its
  `/rtabmap/grid_map` is a flat 2D occupancy projection (loses height = useless for off-road
  slope/step planning).
- **The elevation / terrain map** (CPU `elevation_mapping` or `grid_map_pcl`; **"cupy" is the
  GPU variant we do NOT use** — robot is CPU-only) is just a **rasterisation of that same
  cloud** into a height-per-cell grid. That grid *is* a heightmap in ROS form.

Therefore:

| Source | What it is | Role in the sim pipeline |
|---|---|---|
| RTAB-Map `.db` / cloud | pose graph + dense 3D points | **the input** you record on the real robot |
| Elevation/terrain map (grid_map) | 2.5D height grid (rasterised cloud) | the natural heightmap; what `or_planner` eats |
| Gazebo `<heightmap>` | terrain geometry (a DEM image) | what the simulator actually ingests |

**`map_to_sim.py` does the cloud→DEM rasterisation offline**, so you don't need
`elevation_mapping` running just to build a sim world. "Either way works" only in the sense
that both eventually become the *same* heightmap PNG:
- give it the **RTAB-Map cloud/`.db`** → it rasterises (recommended; this is the recorded map);
- give it an **already-made elevation DEM** (`--dem`) → it just reformats it for Gazebo.

You **cannot** hand Gazebo an RTAB-Map `.db` or a live grid_map — hence the conversion step.
(Also: a real-world `.db` is not portable into sim; the heightmap derived from it is.)

### 10.3 The new files
| File | What it does |
|---|---|
| `scripts/map_to_sim.py` | recorded map → `maps/<name>_heightmap.png` + `<name>.yaml` + `<name>.sdf`. Reads `.ply/.pcd/.xyz` clouds (or runs `rtabmap-export` on a `.db`, or reuses a `--dem`). NumPy+Pillow only — no ROS/open3d. Rasterises (max-Z top surface, percentile outlier clip, hole-fill), snaps to a Gazebo-legal `(2^n)+1` square, writes a world referencing the PNG with a package-relative `file://compa_slam/maps/...` URI. |
| `launch/replay_map_sim.launch.py` | reads `maps/<map>.yaml`, launches Gazebo on `<map>.sdf` + robot + bridge + static `map→odom` + `cost_map_publisher` (extent/height taken from the YAML so `/elevation_map` + `/costmap` match the world exactly) + optional `or_planner` + RViz. |

### 10.4 Run it
```bash
# 1) on the real robot: build a map (Phase 1) AND record a sensor bag (M1.5).
#    The .db ends up in maps/, e.g. maps/compa_real.db

# 2) export a dense cloud from the .db (skip if map_to_sim.py runs it for you):
rtabmap-export --cloud --output maps/compa_real maps/compa_real.db   # -> maps/compa_real.ply

# 3) convert: cloud -> heightmap PNG + .yaml + .sdf
ros2 run compa_slam map_to_sim.py --cloud maps/compa_real.ply --name compa_real
#    (or straight from the db:)  ros2 run compa_slam map_to_sim.py --db maps/compa_real.db --name compa_real
#    tune:  --res 0.05   --zclip 1 99   --size 40   --agg max|mean   --bits 8|16

# 4) (re)build so <share>/compa_slam/maps sees the new files (instant w/ symlink-install)
colcon build --packages-select compa_slam --symlink-install && source install/setup.bash

# 5) drive the recorded terrain in sim
ros2 launch compa_slam replay_map_sim.launch.py map:=compa_real
ros2 topic pub /left_wheel/cmd_vel  std_msgs/msg/Float64 "{data: 3.0}"
ros2 topic pub /right_wheel/cmd_vel std_msgs/msg/Float64 "{data: 3.0}"
#    plan on it:  ros2 launch ... replay_map_sim.launch.py map:=compa_real run_planner:=true  (+ /goal_pose in RViz)
```

### 10.5 Watch-outs / known caveats
1. **`cost_map_publisher` hard-codes the `/costmap` origin at (-20, -20)** while it centres
   `/elevation_map` at (0, 0). They only align when the map extent is **40 m** square. For
   `or_planner` (needs both layers aligned), generate with `--size 40` until a parameterised
   publisher replaces it (Phase 3 candidate; can't edit `hamr_control_cpp` under additive-only).
   For just *driving* the terrain (no planner), any extent is fine — Gazebo uses the centred
   heightmap directly.
2. **8-bit vs 16-bit PNG.** Default 8-bit matches `cost_map_publisher` (it `imread`s grayscale).
   `--bits 16` gives Gazebo a smoother surface but the planner side still quantises to 8-bit.
3. **Height datum offset.** Gazebo places the terrain's lowest point at world `z=z_min`
   (true heights); `cost_map_publisher`'s `/elevation_map` starts elevation at 0. Slopes/steps
   (what `or_planner` checks) are unaffected; absolute z differs by `z_min`.
4. **Outliers inflate the height scale.** SLAM ceiling/flyaway points blow up `z_max`. Lower
   `--zclip` HI (e.g. `--zclip 1 99`) if `z-scale` printed by the script looks too tall.
5. **Cloud frame.** `map_to_sim.py` assumes the cloud's +Z is up and XY is the ground plane
   (true for RTAB-Map with `Reg/Force3DoF` + IMU gravity, as configured in `rtabmap.yaml`).

### 10.6 Where this sits in the milestones
New **Phase 1.5 — Sim replay** between Phase 1 (real SLAM) and Phase 2 (live elevation):
once you can build a real `.db` (M1.5), you can immediately regenerate that space in sim and
develop/validate planning + control against ground truth — without waiting on live
`elevation_mapping`. Phase 2 then swaps the offline heightmap for the *live* `/elevation_map`,
and the same `or_planner` / controller wiring carries over unchanged.
