# Tested simulation maps

This folder contains the 23 terrain maps and their runtime assets.
The largest individual file is approximately 22.2 MiB; every file is below
100 MiB. No run recordings or simulation results are included.

| Family | Map IDs |
|---|---|
| Basic terrain | `road_bumps`, `gentle_hill`, `cross_slope`, `rutted_track`, `rock_garden` |
| Fully uneven | `full_uneven_low`, `full_uneven_medium`, `full_uneven_high`, `full_uneven_flat_reference` |
| Caster stress | `caster_stress_saddle_l1`, `caster_stress_saddle_l2`, `caster_stress_staggered_l1`, `caster_stress_staggered_l2`, `caster_stress_washboard_l1`, `caster_stress_washboard_l2` |
| Rocky | `rocky_stone_steps`, `rocky_staggered_rocks`, `rocky_cross_slope`, `rocky_diagonal_ledges` |
| Verti-Arena | `verti_arena_low`, `verti_arena_medium`, `verti_arena_full`, `verti_arena_edge25` |

`worlds/` contains Gazebo SDF scenes, `meshes/` contains collision STL, visual
Collada, textures, and NPZ height grids, and `config/` contains per-map metadata
and family manifests. `assets/` retains the Verti-Arena source data with
provenance. Verti-Arena is a separate dataset from Verti-Bench.

`navigation/` contains the 2D occupancy grid used by the legacy maze planner.
It is separate from the 3D Gazebo terrains listed above.

The `hamr_terrain` package installs the 3D terrain directories into its ROS share folder.
Relative source symlinks keep existing tooling compatible without duplicating
the map files. Build and launch instructions are in the
[terrain simulator guide](../hamr_terrain/README.md).
