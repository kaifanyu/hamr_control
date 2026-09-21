# CAD ball caster and continuous planning

ROS 2 Jazzy / Gazebo Harmonic package for a CAD-derived passive split ball
caster, a caster test rig, and a complete COMPA simulation chassis with two
replacement casters.

- [Build, launch simulation, record MP4, and edit the model](../docs/SIMULATION.md)
- [Launch the shared planner on the real HAMR](../docs/REAL_CAR_CONTINUOUS.md)
- [CAD geometry and hemisphere orientation](docs/CAD_FINDINGS.md)
- [Planner design](docs/CONTINUOUS_WAYPOINT.md) and [2 cm validation](docs/TIGHT_CONTINUOUS.md)
- [Physical model validation](docs/VALIDATION.md)

From the repository root:

```bash
bash hamr_ball_caster/scripts/build.sh
source hamr_ball_caster/scripts/env.sh
ros2 launch hamr_ball_caster continuous_sim.launch.py
# Add record:=true to save an MP4; gui:=false runs headless.
```

The default route keeps moving through 2 cm corner blends; it starts and ends
at rest. The planner implementation lives in
`reference_trajectory/reference_trajectory/continuous_waypoint.py` and is shared
with the real-car reference publisher. Hardware and simulation have separate
geometry, speed limits, controllers and launch files.

The new expanded files are [ball_caster.urdf](urdf/ball_caster.urdf) and
[compa_ball_caster.urdf](urdf/compa_ball_caster.urdf). Their Xacro sources, runtime
meshes and CAD provenance are included. Native CAD, large experimental traces,
recordings and build output are external development artifacts. They are not
required to build or launch the model.

The full simulation model is a COMPA chassis retrofit, not a calibrated model of
every current real HAMR component. Keep the hardware's existing calibrated
controller/TF configuration when running the real-car launch.
