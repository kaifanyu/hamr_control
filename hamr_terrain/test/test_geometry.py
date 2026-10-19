"""Verify exported physical surfaces, rather than screenshots or XML alone."""

import importlib.util
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "terrain_generator", ROOT / "scripts/generate_terrains.py"
)
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


ALL_IDS = generator.IDS


@pytest.mark.parametrize("name", ALL_IDS)
def test_closed_oriented_collision_matches_visual(name):
    x, y, z, vertices, faces, top_count = generator.mesh_arrays(name)
    triangles = vertices[faces]
    normals = np.cross(
        triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    )
    assert np.all(np.linalg.norm(normals, axis=1) > 1e-9)
    assert np.all(normals[:top_count, 2] > 0)
    assert np.einsum("ij,ij->", triangles[:, 0], normals) / 6 > 0
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    _, inverse, count = np.unique(
        np.sort(edges, axis=1), axis=0, return_inverse=True, return_counts=True
    )
    assert np.all(count == 2), "Collision mesh must be watertight"
    assert np.all(
        np.bincount(inverse, weights=np.where(edges[:, 0] < edges[:, 1], 1, -1)) == 0
    )
    data = (ROOT / "meshes" / f"{name}.stl").read_bytes()
    assert struct.unpack("<I", data[80:84])[0] == len(faces)
    records = np.frombuffer(
        data[84:], dtype=[("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")]
    )
    np.testing.assert_allclose(records["v"], triangles, rtol=0, atol=2e-7)
    np.testing.assert_allclose(
        records["n"], normals / np.linalg.norm(normals, axis=1)[:, None], atol=1e-7
    )
    mesh = ET.parse(ROOT / "meshes" / f"{name}.dae").find(".//c:mesh", NS)
    positions = np.fromstring(
        mesh.find("c:source[@id='positions']/c:float_array", NS).text, sep=" "
    ).reshape(-1, 3)
    np.testing.assert_allclose(positions, vertices, rtol=0, atol=6e-8)
    exported_normals = np.fromstring(
        mesh.find("c:source[@id='normals']/c:float_array", NS).text, sep=" "
    ).reshape(-1, 3)
    rendered_faces = []
    for block in mesh.findall("c:triangles", NS):
        indices = np.fromstring(block.find("c:p", NS).text, sep=" ", dtype=int).reshape(
            -1, 3, 2
        )
        rendered_faces.extend(indices[:, :, 0])
        expected = normals[indices[:, 0, 1]]
        np.testing.assert_allclose(
            exported_normals[indices[:, 0, 1]],
            expected / np.linalg.norm(expected, axis=1)[:, None],
            atol=1e-7,
        )
    sort_rows = lambda a: a[np.lexsort(a.T[::-1])]
    np.testing.assert_array_equal(sort_rows(np.array(rendered_faces)), sort_rows(faces))
    assert np.all(z[0, :] == 0) and np.all(z[-1, :] == 0)
    assert np.all(z[:, 0] == 0) and np.all(z[:, -1] == 0)
    assert np.max(np.abs(z[y < -2.1])) < 1e-12, "Spawn approach must be flat"


@pytest.mark.parametrize("name", ALL_IDS)
def test_world_has_real_relief_and_no_flat_infill(name):
    root = ET.parse(ROOT / "worlds" / f"{name}.sdf").getroot()
    world = root.find("world")
    assert world.find("physics/max_step_size").text == "0.001"
    assert world.find("physics/dart/collision_detector").text == "ode"
    assert world.find("physics/dart/solver/solver_type").text == "dantzig"
    assert not world.findall(".//plane")
    terrain = world.find("model[@name='terrain']")
    assert terrain.find("static").text == "true"
    collision = terrain.find("link/collision")
    assert collision.find("geometry/mesh/uri").text.endswith(f"{name}.stl")
    assert collision.find("surface/friction/ode/mu").text == "1.0"
    assert collision.find("surface/bounce/restitution_coefficient").text == "0"
    for model in world.findall("model"):
        if not model.get("name").startswith("apron_"):
            continue
        pos = np.fromstring(model.find("pose").text, sep=" ")
        size = np.fromstring(
            model.find("link/collision/geometry/box/size").text, sep=" "
        )
        left, right = pos[:2] - size[:2] / 2, pos[:2] + size[:2] / 2
        assert right[0] <= -3 or left[0] >= 3 or right[1] <= -4 or left[1] >= 4
    metadata = json.loads((ROOT / "config" / f"{name}.json").read_text())
    grid = np.load(ROOT / "meshes" / f"{name}_height.npz")
    assert grid["z"].max() > 0.005
    if name == "rutted_track":
        assert grid["z"].min() < -0.02
    assert metadata["triangles"] == 156960


def test_feature_dimensions_at_known_points():
    for y, h in [(-1.15, 0.015), (0.25, 0.025), (1.75, 0.04)]:
        assert float(
            generator.height("road_bumps", np.array(0), np.array(y))
        ) == pytest.approx(h)
    assert float(
        generator.height("gentle_hill", np.array(0), np.array(0.7))
    ) == pytest.approx(0.14)
    x = np.array([-0.7, 0.7])
    y = np.array([0.7, 0.7])
    heights = generator.height("cross_slope", x, y)
    assert np.rad2deg(np.arctan((heights[1] - heights[0]) / 1.4)) == pytest.approx(3.0)


def test_triangle_interpolation_matches_collision_diagonal():
    axis = np.array([0., 1.])
    saddle = np.array([[1., 0.], [0., 1.]])
    assert generator.triangle_height(axis, axis, saddle, .5, .5) == 1.
    assert generator.triangle_height(axis, axis, saddle, .75, .25) == .5
    assert generator.triangle_height(axis, axis, saddle, .25, .75) == .5


def test_mesh_rejects_invalid_grid_and_bottom():
    for x, y, z, bottom in [
        ([0, 0], [0, 1], np.zeros((2, 2)), -1),
        ([0, 1], [0, 1], np.zeros((3, 2)), -1),
        ([0, 1], [0, 1], np.zeros((2, 2)), 0),
    ]:
        with pytest.raises(ValueError):
            generator.mesh_from_height_grid(x, y, z, bottom)


def test_custom_world_uses_its_own_mesh_paths(tmp_path):
    (tmp_path / "worlds").mkdir()
    generator.make_world(tmp_path, "gentle_hill", lambda x, y: 0.0)
    uris = [
        node.text
        for node in ET.parse(tmp_path / "worlds/gentle_hill.sdf").findall(".//mesh/uri")
    ]
    assert uris == [
        str(tmp_path / "meshes/gentle_hill.stl"),
        str(tmp_path / "meshes/gentle_hill.dae"),
    ]
