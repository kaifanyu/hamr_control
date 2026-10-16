"""Check that learned terrain provenance and physical scaling survive export."""

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
spec = importlib.util.spec_from_file_location(
    "adtg_import", ROOT / "scripts/generate_adtg_terrains.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize("difficulty", (5, 25, 50, 75, 95))
def test_export_preserves_common_scale_and_authenticated_raw_input(difficulty):
    metadata = json.loads(
        (ROOT / "config" / f"adtg_d{difficulty:02d}.json").read_text()
    )
    source = ROOT / "assets/adtg/raw" / f"difficulty_{difficulty:03d}_seed7.npy"
    assert (
        hashlib.sha256(source.read_bytes()).hexdigest()
        == metadata["generation"]["raw_sha256"]
    )
    raw = np.load(source)
    grid = np.load(ROOT / "meshes" / f"adtg_d{difficulty:02d}_height.npz")
    x, y, z = grid["x"], grid["y"], grid["z"]
    np.testing.assert_array_equal(
        z, raw.astype(float) * 0.1 * module.blend_weights(x, y)
    )
    interior = (abs(x)[None, :] <= 2.3) & (y[:, None] >= -1.1) & (y[:, None] <= 3.1)
    np.testing.assert_allclose(
        z[interior], raw[interior].astype(float) * 0.1, rtol=0, atol=0
    )
    assert (z[y <= -2.1] == 0).all()
    assert metadata["adaptation"]["smoothing"] == "none"
    assert metadata["adaptation"]["clipping"] == "none"
    assert metadata["raw_statistics"]["height_max_m"] == float(raw.max())
    assert (
        metadata["generation"]["checkpoint_sha256"]
        == "95be54b4a39d58320e187a7ec533351c957f16a00be58773c86074828ee67887"
    )
    assert metadata["smoke"]["spawn_y"] == -1.0
    assert metadata["smoke"]["spawn_z"] > 0


def test_bad_raw_shapes_values_and_scales_are_rejected():
    for array in [
        np.zeros((128, 128)),
        np.full((321, 241), np.nan),
        np.full((321, 241), -1.0),
    ]:
        with pytest.raises(ValueError):
            module.adapt(array)
    for scale in [0, -1, np.inf, np.nan]:
        with pytest.raises(ValueError):
            module.adapt(np.zeros((321, 241)), scale)


def test_triangle_interpolation_does_not_use_bilinear_saddle():
    assert (
        module.triangle_height(
            np.array([0, 1]), np.array([0, 1]), np.array([[1.0, 0], [0, 1]]), 0.5, 0.5
        )
        == 1
    )


def test_changed_raw_bytes_cannot_silently_generate_world(tmp_path):
    np.save(tmp_path / "sample.npy", np.ones((321, 241)))
    manifest = json.loads((ROOT / "assets/adtg/raw/manifest.json").read_text())
    manifest["samples"] = [
        {"difficulty": 5, "seed": 7, "file": "sample.npy", "sha256": "wrong"}
    ]
    manifest["generation"]["difficulty_values"] = [5]
    (tmp_path / "manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="changed raw sample"):
        module.generate(tmp_path, tmp_path / "output")


@pytest.mark.parametrize(
    "field", ("checkpoint", "source", "version", "seed", "difficulty")
)
def test_inconsistent_or_different_generator_is_not_labeled_official(tmp_path, field):
    manifest = json.loads((ROOT / "assets/adtg/raw/manifest.json").read_text())
    if field == "checkpoint":
        manifest["official_checkpoint"]["sha256"] = "different"
    elif field == "source":
        manifest["source_commit"] = "different"
    elif field == "version":
        manifest["environment"]["distributions"][
            "adaptive-terrain-diffusion"
        ] = "different"
    elif field == "seed":
        manifest["samples"][0]["seed"] = 123
    else:
        manifest["samples"][0]["difficulty"] = 123
    (tmp_path / "manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        module.generate(tmp_path, tmp_path / "output")
    assert not (tmp_path / "output").exists()


def test_mesh_rejects_invalid_grid_and_bottom():
    for x, y, z, bottom in [
        ([0, 0], [0, 1], np.zeros((2, 2)), -1),
        ([0, 1], [0, 1], np.zeros((3, 2)), -1),
        ([0, 1], [0, 1], np.zeros((2, 2)), 0),
    ]:
        with pytest.raises(ValueError):
            module.geometry.mesh_from_height_grid(x, y, z, bottom)


def test_custom_world_cannot_load_an_installed_mesh_with_the_same_name(tmp_path):
    (tmp_path / "worlds").mkdir()
    module.geometry.make_world(tmp_path, "adtg_d50", lambda x, y: 0.0)
    uris = [
        n.text
        for n in ET.parse(tmp_path / "worlds/adtg_d50.sdf").findall(".//mesh/uri")
    ]
    assert uris == [
        str(tmp_path / "meshes/adtg_d50.stl"),
        str(tmp_path / "meshes/adtg_d50.dae"),
    ]
