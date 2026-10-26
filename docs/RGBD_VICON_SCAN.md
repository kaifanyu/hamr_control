# RGB-D scan of the 4 × 10 m Vicon area

This route uses the operator-provided Vicon limits `x=[-2,2]`, `y=[0,10] m`
and a D455 rigidly attached to the base, looking in the driving direction.
It is a coverage-oriented capture route for RTAB-Map, not a certification that
every surface is visible. No obstacle layout was supplied.

The executable profile is
[`rgbd_vicon_scan.yaml`](../reference_trajectory/config/rgbd_vicon_scan.yaml).
It uses the existing continuous-waypoint planner and hardware controller.
Coordinates are **absolute Vicon coordinates**, not offsets from placement.
Start at **(1.20, 1.80) m**, with raw Vicon base yaw **0 rad**, corresponding
to physical forward along **+Y** under the current controller convention.
The existing start gate requires position within 0.03 m and yaw within 0.15 rad.

![Two-phase waypoint layout](assets/rgbd_vicon_scan.png)

## Route and purpose

Execute A then B as one continuous run. `S=(1.2,1.8)`.

**A — perimeter with a crosswise excursion:**

```text
S → (1.2,4.4) → (-1.2,4.4) → (-1.2,5.6) → (1.2,5.6)
  → (1.2,9.2) → (-1.2,9.2) → (-1.2,0.8) → (1.2,0.8) → S
```

**B — overlapping interior lanes and a return through familiar views:**

```text
S → (1.2,9.2) → (0.4,9.2) → (0.4,0.8) → (-0.4,0.8)
  → (-0.4,9.2) → (-1.2,9.2) → (-1.2,0.8) → (1.2,0.8) → S
```

The four longitudinal lanes are 0.8 m apart. The crosswise legs at Y=4.4 and
5.6 add views toward both sides. Repeated outer legs approach previously seen
structure from the same direction, providing opportunities for visual loop
closure. B ends travelling +Y toward the original starting view. Opposite-facing
lane images are not assumed to match each other.

RTAB-Map recognizes revisited appearances and uses accepted loop closures to
correct its graph; returning to similar camera views is therefore useful.
See the [RTAB-Map project overview](https://introlab.github.io/rtabmap/).

The existing XY controller lets chassis heading emerge from motion. Its
`fixed_yaw_rad` is a turret reference, not a camera/base-heading command.
Connected bends encourage forward turning; an exact reverse could instead
drive backwards while the camera keeps facing the same way. This route avoids
such immediate reversals. The plot arrows indicate travel direction, not
guaranteed camera orientation. Confirm actual views during the capture.

## Speed, clearance and coverage limits

| Setting | Value |
| --- | --- |
| Nominal translation speed | 0.10 m/s |
| Corner deviation bound | 0.10 m; corners are blended, not stopped at |
| Reference XY envelope | X=[-1.2,1.2], Y=[0.8,9.2] m |
| Boundary inset of reference point | 0.8 m |
| Blended path length | 62.686 m (unrounded waypoints: 64.8 m) |
| Planned motion duration | 10 min 28.7 s |
| Initial/final stationary holds | 10 s each |
| Total after accepted start | **10 min 48.7 s** |

The 0.8 m inset applies to the tracked reference point, **not the outside of
the vehicle**. Confirm that the whole rotating chassis/caster envelope plus
tracking error fits, and clear every leg of obstacles. This waypoint runner
does not use depth for obstacle avoidance and does not enforce a workspace
geofence. Smaller objects inside the rectangle require route changes.

At 0.10 m/s, the current 2 Hz RTAB-Map detection setting sees about 5 cm of
straight travel per update. Capture all configured 30 Hz RGB/depth images;
processing rate can be adjusted on replay. The ideal controller model gives
`|base yaw rate| <= speed / 0.301`, about 0.332 rad/s (19 degrees/s) at cruise;
this is not a measured limit on feedback corrections or actual rotation.

RealSense specifies an ideal D455-family operating range of 0.6–6 m. Prefer
useful textured structure roughly 1–3 m away where practical; the nominal
10 m arena length does not mean the camera can see its whole length at once.
Actual usable view depends on resolution, calibration, depth validity and
surface properties. See the [manufacturer specification](https://www.realsenseai.com/product-family/450-series/).

This is a planar navigation-map capture. A fixed-height, fixed-pitch camera
cannot guarantee coverage of ceilings, high shelves, undersides or occluded
surfaces. Inspect the reconstructed cloud and add views for holes. Camera
height/pitch should include stable scene features as well as the required floor;
do not assume the launch's simulated 20-degree pitch is the real mount.

## Preview without moving hardware

Build the updated reference package once, then source the workspace in each
terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/hamr/hamster_ws/install/setup.bash
cd /home/hamr/hamster_ws
colcon build --packages-select reference_trajectory
source /home/hamr/hamster_ws/install/setup.bash
```

The node update keeps dense samples in `--plan-only` output but omits them from
live metadata, and prepares visualization poses before arming. For this route,
serializing dense metadata alone took about 256 ms on the preparation host;
constructing the visualization poses took another 83 ms. Doing that after arming
would exceed the unchanged 80 ms watchdog. Compact live metadata and cached
poses remove that route-size-dependent startup work without relaxing the guards.
A synthetic start of this full route on an isolated localhost ROS domain took
11.6 ms after the change and produced a valid first reference decision. That
checks startup overhead on this host; it is not a timing guarantee under live
camera, DDS and storage load.

Preview the exact source profile:

```bash
ros2 run reference_trajectory continuous_waypoint \
  --config /home/hamr/hamster_ws/src/hamr_control/reference_trajectory/config/rgbd_vicon_scan.yaml \
  --plan-only /tmp/hamr_rgbd_vicon_scan_plan.json
```

The offline check sampled 62,926 reference states, verified the XY envelope and
stationary endpoints, and retained the original geometry and watchdog settings.
Analytic nominal bounds were 0.10 m/s² translation acceleration,
0.1643 m/s³ jerk, 1.2571 rad/s wheel speed and 1.6748 rad/s² wheel acceleration.
These validate the reference, not physical tracking or sensor coverage. The
compact [validation report](assets/rgbd_vicon_scan_validation.json) and
[waypoint CSV](assets/rgbd_vicon_scan_waypoints.csv) accompany the profile.

## Record and execute

First measure the six `base_link → camera_link` mount values. The camera launch
defaults are simulation placeholders. Physical wheel-forward is currently
base_link +Y; a forward camera does not imply a zero camera-mount yaw. Retain the
driver's optical-frame transform and measure the camera-link mounting transform.

Budget about **35–40 GiB free** for the bag and startup time at the current raw
640×480 RGB/depth configuration, plus separate space for the RTAB-Map database.
The preparation check found the D455 on USB 3, but no live `/d455/*` topics.
Connection alone does not verify streaming or recording.

Keep the Vicon driver running. With the robot placed at S and no competing
controller or reference publisher, terminal 1 starts the stack and idle planner:

```bash
ros2 launch hamr_bringup hamr_continuous_HW.launch.py \
  planner_config:=/home/hamr/hamster_ws/src/hamr_control/reference_trajectory/config/rgbd_vicon_scan.yaml \
  record_bag:=false record_camera:=false run_foxglove:=false \
  use_orientation:=true use_mag:=false
```

`use_orientation:=true` follows the existing mapping runbook's EKF choice.
Vicon still controls the route; recorded wheel/IMU odometry is RTAB-Map's default
motion input. The robot geometry values remain unchanged; the mapping runbook
documents controller/odometry geometry differences that need measurement before
claiming precision. This route does not resolve that calibration question.

Terminal 2 starts the camera and the camera-inclusive bag. Replace all six
`MEASURED_*` tokens with measured numeric values first:

```bash
ros2 launch compa_slam record_trajectory.launch.py \
  robot:=false bag_name:=vicon_rgbd_scan_01 pointcloud:=false \
  mount_x:=MEASURED_X mount_y:=MEASURED_Y mount_z:=MEASURED_Z \
  mount_roll:=MEASURED_ROLL mount_pitch:=MEASURED_PITCH mount_yaw:=MEASURED_YAW
```

If the camera already has the correct launch and transform, add `camera:=false`.
The ordinary hardware recorder alone does not contain RGB-D. This SLAM recorder
captures RGB, aligned depth, RGB camera info, camera IMU, local odometry, Vicon,
commands, and `/tf` plus transient-local `/tf_static`. Point-cloud generation
stays off; RTAB-Map reconstructs it from the bag.

Before starting, verify RGB and aligned depth near 30 Hz, camera info, a correct
TF chain to the optical frame, and continuous finite local odometry. Use the
checks in [MAPPING_CAPTURE.md](../compa_slam/docs/MAPPING_CAPTURE.md). Do not
accept an empty bag or missing image stream just because the recorder is running.

Terminal 3 explicitly starts one run:

```bash
ros2 service call /continuous_waypoint/start std_srvs/srv/Trigger '{}'
```

An accepted start includes the 10-second initial hold. The middle revisit of S
does not stop. Wait for completion including the final hold, then Ctrl-C terminal
2 and wait for MCAP finalization; shut down terminal 1 as well. To end the route
early, use:

```bash
ros2 service call /continuous_waypoint/stop std_srvs/srv/Trigger '{}'
```

The existing planner aborts on an 80 ms feedback gap or reference callback gap;
the controller has its own 120 ms watchdogs. Camera/disk load can expose these
timing limits. An abort is a partial capture, not a completed scan. Inspect and
correct the cause; do not silently increase the limits or automatically retry.

## Validate and build the map

Use the actual bag directory printed by the recorder (a suffix is added if the
requested name already exists):

```bash
ros2 bag info /home/hamr/hamster_ws/src/hamr_control/rosbags/vicon_rgbd_scan_01
ros2 run compa_slam analyze_rosbag.py \
  /home/hamr/hamster_ws/src/hamr_control/rosbags/vicon_rgbd_scan_01
```

Require nonzero RGB/depth/camera-info/TF/odometry data, useful image rates and
valid covariance throughout. After stopping live hardware processes, replay
on a separate ROS domain so recorded command topics cannot reach the robot.
Choose a domain unused by your deployment; 88 below is an example.
The mapping launch deletes an existing database at the requested output path,
so select a new filename for each build:

```bash
ROS_DOMAIN_ID=88 ros2 launch compa_slam rtabmap_real.launch.py \
  bag:=/home/hamr/hamster_ws/src/hamr_control/rosbags/vicon_rgbd_scan_01 \
  database_path:=/home/hamr/.ros/vicon_rgbd_scan_01.db \
  rate:=0.5 use_rtabmap_viz:=false use_odom_topic:=true
```

Use `use_odom_topic:=true` only after whole-bag covariance validation succeeds,
as described in the mapping runbook. Inspect accepted loop closures, the optimized
trajectory and the cloud for missing surfaces, doubled walls and discontinuities.
That inspection determines whether supplemental coverage is required.
