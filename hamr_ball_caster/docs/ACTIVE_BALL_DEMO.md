# Four active ball casters: holonomic concept demo

This separate concept puts four powered spherical wheels under a compact chassis.
The chassis translates forward, backward, sideways and diagonally while its
heading stays at **0° (+X)**. All four spheres visibly roll with the commanded
translation. A persistent arrow on the chassis makes its fixed heading clear.

The scene uses a new, simplified concept chassis, ideal 0.10 m radius spheres,
and stylized actuator housings. It does not reuse the passive split-caster CAD
as an assumed functioning active transmission. This is a prescribed kinematic
visualization: motor design, traction, contact forces and closed-loop performance
are not simulated. The existing COMPA/passive-caster simulation remains available.

## Replay

The generated replay on the development workstation is:

```text
~/Videos/hamr_sim/active_ball_demo/four_active_ball_casters.mp4
```

It is **1600 × 900, 30 fps, H.264/yuv420p**, about **74.9 seconds** long. The
view combines a fixed perspective of the box, a close-up of the robot, a top
view, the active destination, visited-target count, heading and speed. It is
rendered from the shared trajectory with VTK at exact frame times; it is not
a screen capture of Gazebo. Both displays use the same trajectory and sphere
rolling orientations.

To reproduce from the repository root on this workstation:

```bash
source hamr_ball_caster/scripts/env.sh
/usr/bin/python3 hamr_ball_caster/scripts/render_active_ball_demo.py \
  --output-dir "$HOME/Videos/hamr_sim/active_ball_demo_02"
```

Use a new output directory; the renderer refuses to overwrite an existing MP4.
Rendering runs as quickly as the machine allows; playback retains simulation
timing. The current WSLg display supports offscreen VTK rendering. For a machine
without a display, use an X server such as `xvfb-run` or an EGL-capable VTK build.

On Ubuntu, the additional rendering dependencies are `python3-vtk9`,
`python3-pil`, `python3-numpy` and `ffmpeg`. A standard installation can use:

```bash
sudo apt install python3-vtk9 python3-pil python3-numpy ffmpeg
```

Preview one frame without encoding a video:

```bash
/usr/bin/python3 hamr_ball_caster/scripts/render_active_ball_demo.py \
  --output-dir /tmp/active_ball_preview --preview-time 5
```

## Native Gazebo preview

Open a separate Gazebo Harmonic world and run the route:

```bash
source hamr_ball_caster/scripts/env.sh
/usr/bin/python3 hamr_ball_caster/scripts/active_ball_gazebo.py
```

The runner creates the floor, chassis, four colored ball models and local visual
mesh, launches its own isolated Gazebo partition, sends ideal poses, and closes
its own Gazebo process when the route completes. Its output directory contains
the generated SDF, mesh, Gazebo log and report. It uses preview wall time to
schedule poses; the Gazebo physics pause button does not pause the scripted path.

Run a server-only check, or generate the scene without starting Gazebo:

```bash
/usr/bin/python3 hamr_ball_caster/scripts/active_ball_gazebo.py --headless
/usr/bin/python3 hamr_ball_caster/scripts/active_ball_gazebo.py \
  --write-world /tmp/active_ball_scene/active_ball_demo.sdf
```

The SDF by itself displays the concept at rest. Motion requires the runner.
`--speed 2` plays the native preview twice as fast; `--duration 8` limits it to
the first eight trajectory seconds for a quick check. These flags do not change
the MP4's normal timing. No ROS hardware topics or controllers are involved.

## Hard-coded route and motion

The box is **2.4 × 2.4 m**, centered at `(0, 0)`. Robot-center targets are the
geometric corners and edge midpoints; the floor box is a marking, not a physical
wall or a constraint on the whole robot footprint. Every visit starts and ends
at center. Target order:

| Visit | Target | x, y (m) |
|---|---|---|
| 1 | North-east corner | 1.2, 1.2 |
| 2 | North edge midpoint | 0, 1.2 |
| 3 | North-west corner | -1.2, 1.2 |
| 4 | West edge midpoint | -1.2, 0 |
| 5 | South-west corner | -1.2, -1.2 |
| 6 | South edge midpoint | 0, -1.2 |
| 7 | South-east corner | 1.2, -1.2 |
| 8 | East edge midpoint | 1.2, 0 |

Each straight leg uses `s(u) = 10u³ - 15u⁴ + 6u⁵`, with zero velocity and
acceleration at each end and a 0.70 m/s peak speed. Holds last 0.50 s at a target,
0.35 s at center, and 3 s at the beginning and end. The exact route duration is
74.8798 s. This speed is a visualization setting, not a hardware rating.

For ball position `(x_i, y_i)` in chassis coordinates, the ideal center velocity
is `(vx - yaw_rate*y_i, vy + yaw_rate*x_i)`. The sphere's absolute angular
velocity is `(-v_i_y/r, v_i_x/r, 0)`, enforcing zero velocity at its bottom
point. Sphere orientation integrates the exact traveled distance on each leg.
The supplied inverse/forward kinematics support a general planar body twist;
this demo keeps yaw rate zero throughout.

Edit [active_ball_trajectory.py](../scripts/active_ball_trajectory.py) for the
route, speed, holds and ideal geometry. The renderer is
[render_active_ball_demo.py](../scripts/render_active_ball_demo.py); the native
preview is [active_ball_gazebo.py](../scripts/active_ball_gazebo.py). These
entry points are also installed by the normal package build.

The video output directory includes `trajectory.csv`, a `run_summary.json`
with the schedule and assumptions, representative PNG frames, and the encoder
log. The trajectory tests verify all visits, fixed heading, smooth stops,
rolling/contact kinematics, orientation continuity, and final return to center:

```bash
/usr/bin/python3 -m pytest -q hamr_ball_caster/test/test_active_ball_trajectory.py
```
