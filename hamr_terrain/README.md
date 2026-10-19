# Shared terrain simulator

The 23 terrain maps are stored in [`../maps`](../maps/README.md), including
world SDFs, collision and visual meshes, height grids, metadata, and source data.
The package installs those files into its ROS share directory. Source-directory
symlinks preserve the existing generator and launch interfaces.

Build from the repository root on Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic:

```bash
bash hamr_terrain/scripts/build.sh
source hamr_terrain/scripts/env.sh
ros2 launch hamr_terrain terrain_sim.launch.py terrain:=road_bumps caster:=ball
```

Use `caster:=swivel` for the ordinary passive caster model, or `caster:=none`
to inspect the map. Add `gui:=false` for a headless server. The launcher loads
the world and vehicle; it does not start a trajectory controller.

The default spawn `(0,-2.8,0)` facing +Y is for the basic terrain approach.
For other maps, use the spawn coordinates in `maps/config/<terrain>.json`
with `spawn_x`, `spawn_y`, `spawn_z`, and `initial_yaw` launch arguments.
For a configured trajectory, the existing `run_terrain_replay.py` runner reads
that metadata and chooses the supported spawn height.

Each map's metadata records its bounds, geometry hashes, and generation
assumptions. Visual and collision assets describe static rigid surfaces;
see [physical assumptions](docs/PHYSICS_ASSUMPTIONS.md).

Regenerate the five basic maps from their source definitions with:

```bash
/usr/bin/python3 hamr_terrain/scripts/generate_terrains.py
```

Other generators cover fully uneven, caster stress, rocky, and Verti-Arena
maps. `--output /tmp/terrain-preview` creates a separate asset set. Recordings,
run directories, reports, and comparison plots are not included with the maps.
