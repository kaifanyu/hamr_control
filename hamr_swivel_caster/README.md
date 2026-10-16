# Passive swivel-caster simulator

This ROS 2 Jazzy / Gazebo Harmonic package fits two passive swivel casters to
the COMPA simulation chassis. The forks swivel and wheels roll under contact
dynamics; the rear drive wheels, rocker suspension, and stabilized turret use
the shared controller interface.

From the repository root:

```bash
bash hamr_swivel_caster/scripts/build.sh
source hamr_swivel_caster/scripts/env.sh
ros2 launch hamr_swivel_caster swivel_caster.launch.py gui:=true
```

To execute the configured waypoint route:

```bash
ros2 run hamr_swivel_caster run_swivel_caster.py --gui
```

The default route is a 0.8 m square followed by a 0.6 m out-and-back leg.
Use `--config /absolute/path/to/config.yaml` for another route, or
`--profile legacy` for the original simple-waypoint route. Optional recordings
use `--record --output-dir /absolute/path/to/new-directory`.

The workspace defaults to `~/hamr_swivel_caster_ws`; set `HAMR_SWIVEL_CASTER_WS`
before building and sourcing to choose another location. Dependencies are
described in the [simulation guide](../docs/SIMULATION.md).

Shared terrain assets live in [`../maps`](../maps/README.md). Use the
[terrain launcher](../hamr_terrain/README.md) to load them with this vehicle.

The model uses 160 mm wheel diameter, 50 mm width, 192 mm mounting height,
and estimated 45 mm caster trail. Mass, inertia, tire friction, and bearing
drag are simulation assumptions, not calibrated hardware measurements.
See [physical properties](config/physical_properties.yaml) and
[physics assumptions](docs/PHYSICS_ASSUMPTIONS.md). Each caster has independent
passive swivel and rolling joints. The expanded URDFs and editable Xacros
are included under `urdf/`.
