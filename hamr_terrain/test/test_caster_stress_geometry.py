"""Check the frozen stress exports and their physical comparison invariants."""

import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
SPEC = importlib.util.spec_from_file_location(
    "stress_geometry", ROOT / "scripts/generate_caster_stress_terrains.py"
)
GEN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GEN)
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
CASES = json.loads((ROOT / "config/caster_stress_terrains.json").read_text())
IDS = [case["id"] for case in CASES]


@pytest.fixture(scope="module")
def surfaces():
    result = {}
    for case in CASES:
        data = np.load(ROOT / "meshes" / f"{case['id']}_height.npz")
        result[case["id"]] = GEN.geometry.mesh_from_height_grid(
            data["x"], data["y"], data["z"]
        )
    return result


@pytest.mark.parametrize("name", IDS)
def test_frozen_exports_are_identical_closed_surfaces(name, surfaces):
    x, y, z, vertices, faces, top = surfaces[name]
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0],
                       triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    assert np.all(lengths > 1e-9)
    assert np.all(normals[:top, 2] > 0)
    assert np.einsum("ij,ij->", triangles[:, 0], normals) > 0
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    _, inverse, counts = np.unique(np.sort(edges, axis=1), axis=0,
                                    return_inverse=True, return_counts=True)
    assert np.all(counts == 2)
    assert np.all(np.bincount(inverse, weights=np.where(
        edges[:, 0] < edges[:, 1], 1, -1)) == 0)

    binary = (ROOT / "meshes" / f"{name}.stl").read_bytes()
    assert struct.unpack("<I", binary[80:84])[0] == len(faces)
    records = np.frombuffer(binary[84:], dtype=[
        ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    np.testing.assert_allclose(records["vertices"], triangles, rtol=0, atol=2e-7)
    np.testing.assert_allclose(records["normal"], normals / lengths[:, None], atol=1e-7)

    mesh = ET.parse(ROOT / "meshes" / f"{name}.dae").find(".//c:mesh", NS)
    positions = np.fromstring(mesh.find(
        "c:source[@id='positions']/c:float_array", NS).text, sep=" ").reshape(-1, 3)
    exported_normals = np.fromstring(mesh.find(
        "c:source[@id='normals']/c:float_array", NS).text, sep=" ").reshape(-1, 3)
    np.testing.assert_allclose(positions, vertices, rtol=0, atol=6e-8)
    rendered_faces = []
    for block in mesh.findall("c:triangles", NS):
        indices = np.fromstring(block.find("c:p", NS).text,
                                sep=" ", dtype=int).reshape(-1, 3, 2)
        rendered_faces.extend(indices[:, :, 0])
        face_ids = indices[:, 0, 1]
        np.testing.assert_allclose(exported_normals[face_ids],
            normals[face_ids] / lengths[face_ids, None], atol=1e-7)
    def sorted_rows(rows):
        return rows[np.lexsort(rows.T[::-1])]
    np.testing.assert_array_equal(sorted_rows(np.array(rendered_faces)), sorted_rows(faces))
    assert np.all(z[y <= -2.35] == 0)
    assert np.all(z[[0, -1]] == 0) and np.all(z[:, [0, -1]] == 0)


@pytest.mark.parametrize("case", CASES, ids=IDS)
def test_metadata_matches_exported_geometry_and_hashes(case, surfaces):
    name = case["id"]
    x, y, z, vertices, faces, top = surfaces[name]
    gx, gy, expected = GEN.grid(case["family"], case["level"])
    np.testing.assert_array_equal(x, gx)
    np.testing.assert_array_equal(y, gy)
    np.testing.assert_array_equal(z, expected)
    assert len(vertices) == case["vertices"] == 78482
    assert len(faces) == case["triangles"] == 156960
    assert top == case["surface_triangles"] == 153600
    assert case["min_height_m"] == float(z.min())
    assert case["max_height_m"] == float(z.max())
    dzdy, dzdx = np.gradient(z, y, x)
    grades = np.degrees(np.arctan(np.hypot(dzdx, dzdy)))
    assert case["max_surface_slope_deg"] == pytest.approx(grades.max())
    triangles = vertices[faces[:top]]
    normals = np.cross(triangles[:, 1] - triangles[:, 0],
                       triangles[:, 2] - triangles[:, 0])
    face_grades = np.degrees(np.arctan2(np.linalg.norm(normals[:, :2], axis=1), normals[:, 2]))
    assert case["max_triangle_face_slope_deg"] == pytest.approx(face_grades.max())
    for key, relative in [("stl", case["collision_mesh"]),
                          ("dae", case["visual_mesh"]), ("world", case["world"])]:
        assert hashlib.sha256((ROOT / "config" / relative).read_bytes()).hexdigest() == case["sha256"][key]
    assert json.loads((ROOT / "config" / f"{name}.json").read_text()) == case


@pytest.mark.parametrize("name", IDS)
def test_world_contact_configuration_and_no_infill(name):
    world = ET.parse(ROOT / "worlds" / f"{name}.sdf").getroot().find("world")
    assert world.findtext("physics/max_step_size") == "0.001"
    assert world.findtext("physics/dart/collision_detector") == "ode"
    assert world.findtext("physics/dart/solver/solver_type") == "dantzig"
    assert not world.findall(".//plane")
    terrain = world.find("model[@name='terrain']")
    assert terrain.findtext("static") == "true"
    collisions = terrain.findall("link/collision")
    assert len(collisions) == 1
    collision = collisions[0]
    assert collision.findtext("geometry/mesh/uri").endswith(name + ".stl")
    assert collision.findtext("surface/friction/ode/mu") == "1.0"
    assert collision.findtext("surface/friction/ode/mu2") == "1.0"
    assert collision.findtext("surface/bounce/restitution_coefficient") == "0"
    assert terrain.findtext("link/visual/geometry/mesh/uri").endswith(name + ".dae")
    for model in world.findall("model"):
        assert not model.get("name").startswith("marker_")
        if model.get("name").startswith("apron_"):
            center = np.fromstring(model.findtext("pose"), sep=" ")[:2]
            size = np.fromstring(model.findtext("link/collision/geometry/box/size"), sep=" ")[:2]
            low, high = center - size / 2, center + size / 2
            assert high[0] <= -3 or low[0] >= 3 or high[1] <= -4 or low[1] >= 4


@pytest.mark.parametrize("family", GEN.FAMILIES)
def test_relief_levels_change_amplitude_only(family, surfaces):
    x, y, low, *_ = surfaces[f"caster_stress_{family}_l1"]
    x2, y2, high, *_ = surfaces[f"caster_stress_{family}_l2"]
    np.testing.assert_array_equal(x, x2)
    np.testing.assert_array_equal(y, y2)
    np.testing.assert_allclose(high, low * GEN.HEIGHTS[family][1] / GEN.HEIGHTS[family][0],
                               rtol=1e-14, atol=1e-17)


def test_nominal_caster_tracks_alternate_pad_loading():
    for row, y in enumerate([-1.05, -.35, .35, 1.05]):
        heights = GEN.height("staggered", 2, np.array([-.22, .22]), y)
        np.testing.assert_allclose(heights, [.055, 0] if row % 2 == 0 else [0, .055], atol=1e-14)


def test_all_sliding_lanes_have_identical_flat_topology(surfaces):
    reference = None
    for x, y, z, vertices, faces, top in surfaces.values():
        triangles = vertices[faces[:top]]
        lane = triangles[np.all(triangles[:, :, 1] < -2.35, axis=1)]
        assert np.all(lane[:, :, 2] == 0)
        if reference is not None:
            np.testing.assert_array_equal(lane, reference)
        reference = lane


def test_interpolator_follows_mesh_diagonal_in_both_cell_halves(surfaces):
    x, y, z, *_ = surfaces["caster_stress_saddle_l2"]
    i, j = 127, 137
    a, b, c, d = z[j, i], z[j, i+1], z[j+1, i+1], z[j+1, i]
    for u, v, expected in [(.8, .2, .2*a+.6*b+.2*c),
                            (.2, .8, .2*a+.2*c+.6*d)]:
        actual = GEN.triangle_height(x, y, z, x[i]+u*(x[i+1]-x[i]), y[j]+v*(y[j+1]-y[j]))
        assert actual == pytest.approx(expected, abs=1e-15)
