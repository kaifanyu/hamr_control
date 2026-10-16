#!/usr/bin/env python3
"""Import genuine released ADTG heightfields as documented HAMR Gazebo surfaces.

Inference is separate from ROS: sample_adtg.py uses the isolated ONNX environment.
This importer never synthesizes a procedural substitute for missing model output.
"""

import argparse
import hashlib
import json
import os
import math
from pathlib import Path

import numpy as np

import generate_terrains as geometry
from sample_adtg import SOURCE_COMMIT, SOURCE_URL, PACKAGE_VERSION, MODEL_SHA256


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def blend_weights(x, y):
    """Flat fixture approach and zero-height border, identical for every sample."""
    xx, yy = np.meshgrid(x, y)
    return (
        geometry.ease((3.0 - np.abs(xx)) / 0.7)
        * geometry.ease((yy + 2.1) / 1.0)
        * geometry.ease((3.8 - yy) / 0.7)
    )


def triangle_height(x, y, z, px, py):
    """Interpolate the same a-b-c / a-c-d diagonal as the collision export."""
    i = int(np.clip(np.searchsorted(x, px, side="right") - 1, 0, len(x) - 2))
    j = int(np.clip(np.searchsorted(y, py, side="right") - 1, 0, len(y) - 2))
    u, v = (px - x[i]) / (x[i + 1] - x[i]), (py - y[j]) / (y[j + 1] - y[j])
    a, b, c, d = z[j, i], z[j, i + 1], z[j + 1, i + 1], z[j + 1, i]
    return float(
        a + (b - a) * u + (c - b) * v if u >= v else a + (c - d) * u + (d - a) * v
    )


def adapt(raw, vertical_scale=0.1):
    raw = np.asarray(raw)
    if raw.shape != (321, 241) or not np.isfinite(raw).all():
        raise ValueError(
            "Expected finite (321,241) official heightfield; no implicit resizing is applied"
        )
    if raw.min() < 0 or not math.isfinite(vertical_scale) or vertical_scale <= 0:
        raise ValueError(
            "Use official zero_floor output and a positive finite common vertical scale"
        )
    x, y = np.linspace(-3, 3, 241), np.linspace(-4, 4, 321)
    z = raw.astype(np.float64) * vertical_scale * blend_weights(x, y)
    return x, y, z


def features(x, y, z):
    xx, yy = np.meshgrid(x, y)
    dy, dx = np.gradient(z, y, x)
    grade = np.degrees(np.arctan(np.hypot(dx, dy)))
    interior = (abs(xx) <= 1.2) & (yy >= -0.8) & (yy <= 2.4)
    corridor = (abs(xx) <= 0.6) & (yy >= -0.8) & (yy <= 2.4)
    picks = [
        ("high_point", np.where(interior, z, -np.inf).argmax()),
        ("low_point", np.where(interior, z, np.inf).argmin()),
        ("steep_point", np.where(interior, grade, -np.inf).argmax()),
    ]
    probes = [{"name": "flat_approach", "xy_m": [0, -2.6], "kind": "flat"}]
    for name, index in picks:
        row, col = np.unravel_index(index, z.shape)
        probes.append(
            {"name": name, "xy_m": [float(x[col]), float(y[row])], "kind": "feature"}
        )
    return probes, {
        "min_height_m": float(z.min()),
        "max_height_m": float(z.max()),
        "height_mean_m": float(z.mean()),
        "height_std_m": float(z.std()),
        "max_surface_slope_deg": float(grade.max()),
        "corridor_height_max_m": float(z[corridor].max()),
        "corridor_slope_p95_deg": float(np.percentile(grade[corridor], 95)),
        "corridor_slope_max_deg": float(grade[corridor].max()),
        "corridor_definition": "|x|<=0.6m, -0.8<=y<=2.4m; excludes the entry/exit blends",
        "slope_method": "Grid-node central finite differences over the top surface; excludes closing side walls.",
    }


def generate(raw_directory, output, vertical_scale=0.1):
    raw_directory, output = Path(raw_directory).resolve(), Path(output).resolve()
    manifest_path = raw_directory / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if not manifest.get("complete"):
        raise ValueError("Raw inference manifest is incomplete")
    if (
        manifest.get("source_commit") != SOURCE_COMMIT
        or manifest.get("source_url") != SOURCE_URL
        or manifest.get("source_dirty_before_generation") is not False
        or manifest.get("official_checkpoint", {}).get("sha256") != MODEL_SHA256
        or manifest.get("environment", {})
        .get("distributions", {})
        .get("adaptive-terrain-diffusion")
        != PACKAGE_VERSION
    ):
        raise ValueError(
            "Raw manifest does not describe the pinned official model and source"
        )
    request = manifest.get("generation", {})
    samples = manifest.get("samples", [])
    difficulties = [s.get("difficulty") for s in samples]
    if (
        not samples
        or any(type(d) is not int or not 1 <= d <= 100 for d in difficulties)
        or len(set(difficulties)) != len(difficulties)
        or sorted(difficulties) != sorted(request.get("difficulty_values", []))
        or request.get("shape_y_x") != [321, 241]
        or request.get("zero_floor") is not True
        or any(s.get("seed") != request.get("seed") for s in samples)
    ):
        raise ValueError(
            "Raw manifest sample difficulty, shape or seed is inconsistent"
        )
    prepared = []
    for sample in samples:
        source = (raw_directory / sample["file"]).resolve()
        if (
            not source.is_relative_to(raw_directory)
            or sha256(source) != sample["sha256"]
        ):
            raise ValueError(f"Untrusted path or changed raw sample: {source}")
        raw = np.load(source, allow_pickle=False)
        x, y, z = adapt(raw, vertical_scale)
        prepared.append((sample, source, raw, x, y, z))
    for directory in ("meshes", "worlds", "config"):
        (output / directory).mkdir(parents=True, exist_ok=True)
    result = []
    for sample, source, raw, x, y, z in prepared:
        difficulty = int(sample["difficulty"])
        name = f"adtg_d{difficulty:02d}"
        _, _, _, vertices, faces, top = geometry.mesh_from_height_grid(x, y, z)
        geometry.stl(output / "meshes" / f"{name}.stl", vertices, faces)
        geometry.dae(output / "meshes" / f"{name}.dae", name, vertices, faces, top)
        np.savez_compressed(output / "meshes" / f"{name}_height.npz", x=x, y=y, z=z)
        geometry.make_world(
            output, name, lambda px, py: triangle_height(x, y, z, px, py)
        )
        probes, stats = features(x, y, z)
        xx, yy = np.meshgrid(x, y)
        footprint = (abs(xx) <= 0.6) & (yy >= -1.5) & (yy <= -0.4)
        spawn_z = float(z[footprint].max() + 0.07)
        data = {
            "id": name,
            "title": f"ADTG learned terrain · difficulty {difficulty}",
            "description": f'Official standalone diffusion terrain, difficulty {difficulty}/100, seed {sample["seed"]}; common HAMR height multiplier {vertical_scale:g}, 25 mm XY grid, with flat approach and border blends.',
            "world": f"../worlds/{name}.sdf",
            "bounds_m": {"x": [-3, 3], "y": [-4, 4]},
            "height_grid": f"../meshes/{name}_height.npz",
            "collision_mesh": f"../meshes/{name}.stl",
            "visual_mesh": f"../meshes/{name}.dae",
            "grid_spacing_m": 0.025,
            "vertices": len(vertices),
            "triangles": len(faces),
            "surface_triangles": top,
            **stats,
            "terrain_friction": {
                "mu": 1.0,
                "mu2": 1.0,
                "restitution": 0.0,
                "basis": "Matches DART mesh attachment defaults; runtime sliding probes measure approximate effective response.",
            },
            "spawn": {"x": 0, "y": -2.8, "z": 0, "yaw": math.pi / 2},
            "probes": probes,
            "smoke": {
                "spawn_x": 0,
                "spawn_y": -1.0,
                "spawn_z": spawn_z,
                "initial_yaw": math.pi / 2,
                "drive_speed_m_s": 0.12,
                "duration_s": 10.0,
                "scope": "Common short center corridor; does not traverse the entire map or its steep edge extrema.",
            },
            "generation": {
                "source_url": manifest["source_url"],
                "source_commit": manifest["source_commit"],
                "package": "adaptive-terrain-diffusion",
                "version": PACKAGE_VERSION,
                "model": "Authors released endpoint-distilled, difficulty-conditioned one-step ONNX model",
                "checkpoint_sha256": manifest["official_checkpoint"]["sha256"],
                "difficulty": difficulty,
                "seed": sample["seed"],
                "shape_y_x": list(raw.shape),
                "raw_file": os.path.relpath(source, (output / "config").resolve()),
                "raw_sha256": sample["sha256"],
                "raw_manifest": os.path.relpath(manifest_path, (output / "config").resolve()),
                "raw_manifest_sha256": sha256(manifest_path),
                "inference": manifest["generation"],
                "scope": "Learned elevation generation only; no reproduction of policy-adaptive curriculum, RL training, canopy or sim-to-real performance.",
            },
            "adaptation": {
                "vertical_multiplier": vertical_scale,
                "horizontal_spacing_m": 0.025,
                "official_adapter_default_spacing_m": 0.1,
                "relative_xy_scale": 0.25,
                "isotropic_scale": False,
                "array_axes": "row=y, column=x; first sample at(-3,-4)",
                "height_formula": "z = float64(raw_height_m) * vertical_multiplier * border_weight(x) * entry_weight(y) * exit_weight(y)",
                "flat_approach_y_max_m": -2.1,
                "full_entry_y_m": -1.1,
                "full_exit_y_m": 3.1,
                "zero_exit_y_m": 3.8,
                "full_lateral_abs_x_max_m": 2.3,
                "zero_lateral_abs_x_m": 3.0,
                "weight_function": "0.5-0.5*cos(pi*clip(t,0,1))",
                "smoothing": "none",
                "clipping": "none",
                "per_map_height_normalization": "none",
                "note": "Boundary blending changes edge peaks; raw and adapted statistics are retained separately.",
            },
            "raw_statistics": sample,
            "limitations": [
                "Rigid ground; no loose gravel, deformable mud, sinkage or vegetation.",
                "Difficulty label is not calibrated to HAMR traversability; extrema need not be monotonic.",
                "Uncalibrated ideal rear-wheel velocity servos; short drive does not establish motor/climb capability.",
                "Adapted physical scales and boundary blends differ from the original paper environment.",
            ],
            "sha256": {
                ext: sha256(output / "meshes" / f"{name}.{ext}")
                for ext in ("stl", "dae")
            },
        }
        (output / "config" / f"{name}.json").write_text(
            json.dumps(data, indent=2) + "\n"
        )
        result.append(data)
        print(
            json.dumps(
                {
                    key: data[key]
                    for key in ("id", "max_height_m", "corridor_slope_p95_deg")
                }
            ),
            flush=True,
        )
    (output / "config/adtg_terrains.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )
    return result


if __name__ == "__main__":
    package = geometry.package_directory()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-dir", type=Path, default=package / "assets/adtg/raw")
    parser.add_argument("--output", type=Path, default=package)
    parser.add_argument("--vertical-scale", type=float, default=0.1)
    args = parser.parse_args()
    generate(args.raw_dir, args.output, args.vertical_scale)
