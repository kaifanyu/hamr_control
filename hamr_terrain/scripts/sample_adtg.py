#!/usr/bin/env python3
"""Sample the pinned official ADTG model without changing the ROS environment.

Run setup_adtg.sh once, then pass a new/empty output directory. Raw heights use
the official API defaults; HAMR scaling and landing pads belong to the importer.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

SOURCE_URL = "https://github.com/youwyu/Adaptive-Diffusion-Terrain"
SOURCE_COMMIT = "262488cd981f616898541198537a4b302fa2922a"
PACKAGE_VERSION = "0.4.2"
MODEL_NAME = "terrain_consistency_unet_difficulty100.onnx"
MODEL_SHA256 = "95be54b4a39d58320e187a7ec533351c957f16a00be58773c86074828ee67887"


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    temporary = Path(path).with_suffix(".json.tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def validate_request(difficulties, shape, seed):
    if not difficulties or any(not 1 <= d <= 100 for d in difficulties):
        raise ValueError("Difficulties must be integers from 1 to 100")
    if len(set(difficulties)) != len(difficulties):
        raise ValueError("Duplicate difficulties would overwrite sample files")
    if len(shape) != 2 or min(shape) < 2:
        raise ValueError("Shape must contain two dimensions of at least 2")
    if seed < 0:
        raise ValueError("Seed must be nonnegative")


def require_empty_output(output):
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError(f"Refusing to overwrite nonempty output: {output}")


def inspect_provenance(source, installed_package, version):
    """Require the reviewed official source, code and checkpoint before inference."""
    if version != PACKAGE_VERSION:
        raise ValueError(f"Expected adaptive-terrain-diffusion {PACKAGE_VERSION}, found {version}")
    commit = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if commit != SOURCE_COMMIT:
        raise ValueError(f"Source checkout must be pinned at {SOURCE_COMMIT}")
    status = subprocess.check_output([
        "git", "-C", str(source), "status", "--porcelain", "--untracked-files=normal",
        "--", "contexts/standalone"], text=True)
    if status:
        raise ValueError("Standalone source has local modifications")
    package = source/"contexts/standalone"
    source_package = package/"terrain_diffusion"
    hashes = {}
    tracked = subprocess.check_output([
        "git", "-C", str(source), "ls-files", "-z", "contexts/standalone"], text=True)
    for relative in tracked.split("\0"):
        if relative:
            path = source/relative
            hashes[str(path.relative_to(package))] = sha256(path)
    # Compare every installed Python module and the weights with the reviewed
    # checkout. A package version string alone does not establish provenance.
    for path in sorted(source_package.rglob("*")):
        if path.is_file() and path.suffix in (".py", ".onnx"):
            installed = installed_package/path.relative_to(source_package)
            if not installed.is_file() or sha256(installed) != sha256(path):
                raise ValueError(f"Installed official package differs from source: {installed}")
    model = installed_package/"weights"/MODEL_NAME
    if sha256(model) != MODEL_SHA256:
        raise ValueError("Bundled ONNX checkpoint does not match the reviewed official model")
    return {
        "source_url": SOURCE_URL, "source_commit": commit,
        "source_checkout": str(source), "source_dirty_before_generation": False,
        "standalone_source_tree_sha256": hashes,
        "official_checkpoint": {"source_file": str(source_package/"weights"/MODEL_NAME),
                                "sha256": MODEL_SHA256, "bytes": model.stat().st_size},
    }


def terrain_statistics(values, shape):
    import numpy as np

    if values.shape != tuple(shape) or values.dtype != np.float32:
        raise ValueError("Official generator returned an unexpected shape or dtype")
    if not np.isfinite(values).all():
        raise ValueError("Official generator returned nonfinite heights")
    result = {
        "shape_y_x": list(values.shape), "dtype": str(values.dtype), "finite": True,
        "height_min_m": float(values.min()), "height_max_m": float(values.max()),
        "height_range_m": float(np.ptp(values)), "height_mean_m": float(values.mean()),
        "height_std_m": float(values.std()),
        "height_percentiles_m": dict(zip(["p1", "p5", "p50", "p95", "p99"],
                                        map(float, np.percentile(values, [1, 5, 50, 95, 99])))),
    }
    for spacing in (.1, .025):
        gy, gx = np.gradient(values.astype(np.float64), spacing, spacing)
        gradient = np.hypot(gx, gy)
        result[f"unscaled_height_slopes_at_xy_spacing_{spacing}_m"] = {
            "maximum_deg": float(np.degrees(np.arctan(gradient.max()))),
            "p95_deg": float(np.degrees(np.arctan(np.percentile(gradient, 95)))),
            "rms_gradient": float(np.sqrt(np.mean(gradient**2))),
        }
    return result


def sample(args):
    import numpy as np
    import terrain_diffusion
    from terrain_diffusion import TerrainGenerator

    output = args.output_dir.expanduser().resolve()
    require_empty_output(output)
    source = args.cache_dir.expanduser().resolve()/"source"
    provenance = inspect_provenance(source, Path(terrain_diffusion.__file__).parent,
                                   importlib.metadata.version("adaptive-terrain-diffusion"))
    output.mkdir(parents=True, exist_ok=True)
    # Atomically reserve the directory against two samplers starting together.
    with (output/".sampling.lock").open("x") as lock:
        lock.write(f"pid={os.getpid()}\n")
    shutil.copyfile(__file__, output/"sample_adtg.py")
    height, width = args.shape
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(), "complete": False,
        **provenance,
        "environment": {"python": sys.version, "executable": sys.executable,
                        "platform": platform.platform(),
                        "distributions": {d.metadata["Name"]: d.version for d in importlib.metadata.distributions()},
                        "cpu_runtime_note": "Official source installed with --no-deps; CPU onnxruntime supplies its supported CPU API instead of the Linux metadata default onnxruntime-gpu. Source and weights unchanged."},
        "generation": {"seed": args.seed, "seed_reset_for_each_difficulty": True,
                       "num_terrains_per_call": 1, "shape_y_x": args.shape,
                       "difficulty_values": args.difficulties, "size_mode": "auto",
                       "resolved_size_mode": "tile" if max(args.shape) > 128 else "resize",
                       "tile_overlap": 32, "batch_size": 16, "zero_floor": True,
                       "sampling_steps": 1, "guidance_scale": 1.0,
                       "postprocessing": "Only official API decoding, overlapping tile blending and zero_floor; no custom filtering, clipping, scaling or blending."},
        "units": {"heights": "meters as interpreted by official SimulatorTerrain",
                  "generator_xy_spacing": None, "official_simulator_default_spacing_m": .1,
                  "official_simulator_default_vertical_multiplier": 1.,
                  "physical_extent_xy_m_at_default_spacing": [(width-1)*.1, (height-1)*.1],
                  "hamr_candidate_spacing_m_not_applied": .025,
                  "candidate_extent_xy_m": [(width-1)*.025, (height-1)*.025]},
        "scope": "Official current endpoint-distilled one-step consistency-diffusion generator; no claim this reproduces the complete original CoRL training/curriculum experiment.",
        "samples": [],
    }
    write_json(output/"manifest.json", manifest)
    try:
        start = time.monotonic()
        generator = TerrainGenerator(device="cpu")
        manifest["runtime"] = {
            "device": generator.device, "providers": list(generator.providers),
            "model_metadata": generator.metadata, "installed_model_path": generator.model_path,
            "session_setup_wall_s": time.monotonic()-start,
            "model_height_range_m": generator.height_range,
            "model_asinh_scale_m": generator.asinh_scale,
        }
        for difficulty in args.difficulties:
            start = time.monotonic()
            values = generator.generate(1, shape=tuple(args.shape), difficulty=difficulty, seed=args.seed)[0]
            record = {"difficulty": difficulty, "seed": args.seed,
                      **terrain_statistics(values, args.shape)}
            path = output/f"difficulty_{difficulty:03d}_seed{args.seed}.npy"
            with path.open("xb") as handle:
                np.save(handle, values, allow_pickle=False)
            record.update(file=path.name, sha256=sha256(path), wall_s=time.monotonic()-start)
            manifest["samples"].append(record)
            write_json(path.with_suffix(".json"), record)
            write_json(output/"manifest.json", manifest)
            print(json.dumps(record), flush=True)
        difficulty = 50 if 50 in args.difficulties else args.difficulties[0]
        start = time.monotonic()
        repeated = generator.generate(1, shape=tuple(args.shape), difficulty=difficulty, seed=args.seed)[0]
        prior = np.load(output/f"difficulty_{difficulty:03d}_seed{args.seed}.npy", allow_pickle=False)
        manifest["reproducibility_check"] = {
            "difficulty": difficulty, "seed": args.seed,
            "array_equal": bool(np.array_equal(prior, repeated)),
            "maximum_absolute_difference_m": float(np.max(np.abs(prior-repeated))),
            "wall_s": time.monotonic()-start,
        }
        if not manifest["reproducibility_check"]["array_equal"]:
            raise ValueError("Repeatability check failed on this runtime")
        requirements = subprocess.check_output([sys.executable, "-I", "-m", "pip", "freeze"], text=True)
        (output/"requirements.lock.txt").write_text(requirements)
        manifest["complete"] = True
        manifest["completed_utc"] = datetime.now(timezone.utc).isoformat()
    except BaseException as error:
        manifest["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        write_json(output/"manifest.json", manifest)
        (output/".sampling.lock").unlink(missing_ok=True)
    print(f"Completed {len(manifest['samples'])} official raw samples in {output}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--difficulties", type=int, nargs="+", default=[5, 25, 50, 75, 95])
    parser.add_argument("--shape", type=int, nargs=2, metavar=("ROWS", "COLUMNS"), default=[321, 241])
    parser.add_argument("--cache-dir", type=Path,
                        default=Path(os.environ.get("HAMR_ADTG_CACHE", Path.home()/".cache/hamr-adtg")))
    args = parser.parse_args(argv)
    try:
        validate_request(args.difficulties, args.shape, args.seed)
        require_empty_output(args.output_dir.expanduser().resolve())
    except ValueError as error:
        parser.error(str(error))
    venv = args.cache_dir.expanduser().resolve()/"venv"
    if Path(sys.prefix).resolve() != venv or not sys.flags.isolated:
        python = venv/"bin/python"
        if not python.is_file():
            parser.error("Isolated generator is not installed; run setup_adtg.sh first")
        arguments = sys.argv[1:] if argv is None else list(argv)
        os.execv(str(python), [str(python), "-I", str(Path(__file__).resolve()), *arguments])
    sample(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
