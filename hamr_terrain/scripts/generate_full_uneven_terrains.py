#!/usr/bin/env python3
"""Three bounded, continuously uneven surfaces, with no flat approaches/aprons."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

import generate_terrains as geometry
from generate_terrains import triangle_height

SEED = 20261006
LEVELS = (("low", .015, 3.), ("medium", .030, 6.), ("high", .045, 9.))
IDS = tuple(f"full_uneven_{level}" for level, _, _ in LEVELS)
FLAT_ID = "full_uneven_flat_reference"
WAVELENGTHS = np.array([2.7, 2.1, 1.6, 1.25, 1., .8, .66, .55, .46, .39]) / 1.4
WEIGHTS = np.array([1., .85, .7, .58, .48, .37, .26, .18, .12, .08])


def height_fields(seed=SEED):
    x, y = np.linspace(-3., 3., 241), np.linspace(-4., 4., 321)
    xx, yy = np.meshgrid(x, y)
    rng = np.random.default_rng(seed)
    raw = np.zeros_like(xx)
    components = []
    for wavelength, weight in zip(WAVELENGTHS, WEIGHTS):
        angle, phase = rng.uniform(0., 2*np.pi, size=2)
        raw += weight * np.sin(2*np.pi*(xx*np.cos(angle)+yy*np.sin(angle))/wavelength+phase)
        components.append(dict(wavelength_m=float(wavelength), weight=float(weight),
                               direction_rad=float(angle), phase_rad=float(phase)))
    normalized = (raw-raw.min()) / np.ptp(raw)
    return x, y, normalized, components


def surface_statistics(x, y, z):
    # Exact gradients of the exported two triangles in every cell.
    dx, dy = np.diff(z, axis=1)/np.diff(x), np.diff(z, axis=0)/np.diff(y)[:, None]
    gradients = np.r_[np.hypot(dx[:-1], dy[:, 1:]).ravel(),
                      np.hypot(dx[1:], dy[:, :-1]).ravel()]
    # Every possible 0.4 x 0.4 m grid-aligned window, including the boundaries.
    from numpy.lib.stride_tricks import sliding_window_view
    width = 17
    row_windows = sliding_window_view(z, width, axis=1)
    local_max = sliding_window_view(row_windows.max(axis=2), width, axis=0).max(axis=2)
    local_min = sliding_window_view(row_windows.min(axis=2), width, axis=0).min(axis=2)
    return {"min_height_m": float(z.min()), "max_height_m": float(z.max()),
            "height_range_m": float(np.ptp(z)), "height_std_m": float(z.std()),
            "max_surface_slope_deg": float(np.degrees(np.arctan(gradients.max()))),
            "horizontal_surface_triangles": int(np.count_nonzero(gradients < 1e-8)),
            "surface_triangles": int(len(gradients)),
            "minimum_0_4m_patch_relief_m": float((local_max-local_min).min()),
            "maximum_adjacent_node_height_difference_m": float(max(np.abs(np.diff(z,axis=0)).max(), np.abs(np.diff(z,axis=1)).max())),
            "slope_method": "Exact planar gradient on every exported top triangle; closing walls excluded.",
            "coverage_method": "Height range in every grid-aligned 0.4 x 0.4 m window, with 25 mm stride; entire 6 x 8 m surface including edges."}


def generate(output, seed=SEED):
    output = Path(output).resolve()
    for folder in ("meshes", "worlds", "config"):
        (output/folder).mkdir(parents=True, exist_ok=True)
    x, y, normalized, components = height_fields(seed)
    metadata = []
    for level, limit, slope_limit in (*LEVELS, ("flat_reference", 0., 0.)):
        name = f"full_uneven_{level}"
        z = normalized * limit
        stats = surface_statistics(x, y, z)
        assert stats["max_surface_slope_deg"] <= slope_limit
        if limit:
            assert stats["horizontal_surface_triangles"] == 0
            assert stats["minimum_0_4m_patch_relief_m"] > .001
        _, _, _, vertices, faces, top = geometry.mesh_from_height_grid(x, y, z)
        geometry.stl(output/"meshes"/f"{name}.stl", vertices, faces)
        geometry.dae(output/"meshes"/f"{name}.dae", name, vertices, faces, top)
        np.savez_compressed(output/"meshes"/f"{name}_height.npz", x=x, y=y, z=z)
        geometry.make_world(output, name, lambda px, py: triangle_height(x, y, z, px, py))
        world_file = output/"worlds"/f"{name}.sdf"
        tree = ET.parse(world_file)
        world = tree.getroot().find("world")
        for model in list(world.findall("model")):
            if model.get("name", "").startswith(("apron_", "marker_")):
                world.remove(model)
        world.find("model[@name='detail_camera']/pose").text = " ".join(map(str, geometry.look_at((1.3,-3.4,.55),(-.4,.3,.025))))
        world.find("light/direction").text = "-.8 .35 -.5"
        ET.indent(tree, space="  ")
        tree.write(world_file, encoding="utf-8", xml_declaration=True)
        data = {"id": name, "title": f"Entire-map uneven terrain: {level.replace('_',' ')}",
                "description": f"Continuous irregular relief over all 6 x 8 m; {limit*1000:g} mm peak-to-valley bound. No approach, border blend, flat apron, clipping or flat infill.",
                "world": f"../worlds/{name}.sdf", "height_grid": f"../meshes/{name}_height.npz",
                "collision_mesh": f"../meshes/{name}.stl", "visual_mesh": f"../meshes/{name}.dae",
                "bounds_m": {"x": [-3,3], "y": [-4,4]}, "grid_spacing_m": .025,
                "vertices": len(vertices), "triangles": len(faces), **stats,
                "level": level, "peak_to_valley_limit_m": limit, "slope_limit_deg": slope_limit,
                "terrain_friction": {"mu": 1., "mu2": 1., "restitution": 0.},
                "generation": {"kind": "Deterministic procedural multiscale directional waves",
                               "seed": seed, "components": components,
                               "normalization": "One shared spatial field mapped linearly to [0, level height]; no clipping, flattening or border taper.",
                               "matched_levels": "Same pattern; only vertical amplitude changes."},
                "height_basis": {"neutral_main_chassis_clearance_m": .1906,
                                 "clearance_derivation": "URDF main chassis center .220 m minus .025 m half-thickness minus .0044 m neutral wheel support plane.",
                                 "swivel_wheel_radius_m": .080, "ball_nominal_radius_m": .100,
                                 "rear_wheel_radius_m": .1075,
                                 "high_level_fraction_of_chassis_clearance": .045/.1906,
                                 "scope": "Conservative geometry bound, not a guarantee against dynamic tipping; check both vehicles at the intended speed."},
                "spawn": {"x":0., "y":-3.2, "z":float(z.max()+.025), "yaw":float(np.pi/2)},
                "flat_friction_probes": False,
                "probes": [{"name": f"surface_{i}", "xy_m": xy, "kind": "feature"}
                           for i,xy in enumerate([[-2.3,-3.3],[1.8,-2.2],[0.,0.],[2.2,2.5],[-2.1,3.2]])],
                "limitations": ["Rigid ground; no soil deformation or loose particles.",
                                "No flat driving patches on the three uneven levels; isolated extrema and small planar mesh facets are unavoidable.",
                                "Geometry and contact parameters are estimates. A passed test covers its route, speed and initial state only."],
                "sha256": {ext: hashlib.sha256((output/"meshes"/f"{name}.{ext}").read_bytes()).hexdigest() for ext in ("stl","dae")}}
        if not limit:
            data["description"] = "Auxiliary flat control for the matched weaving trajectory; not one of the three uneven levels."
        (output/"config"/f"{name}.json").write_text(json.dumps(data, indent=2)+"\n")
        if limit:
            metadata.append(data)
    (output/"config/full_uneven_terrains.json").write_text(json.dumps(metadata, indent=2)+"\n")
    return metadata


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=geometry.package_directory())
    parser.add_argument("--seed", type=int, default=SEED)
    args=parser.parse_args()
    for entry in generate(args.output, args.seed):
        print(json.dumps({key:entry[key] for key in ("id","height_range_m","max_surface_slope_deg","minimum_0_4m_patch_relief_m","horizontal_surface_triangles")}))
