"""Independent CAD-to-URDF checks, using reconstructed kinematics and mesh data.

Run with: python -m pytest hamr_ball_caster/test/test_geometry.py
Requires numpy, pytest and xacro. No CAD parser, Gazebo or ROS runtime is needed.
"""

from functools import lru_cache
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro


PACKAGE = Path(__file__).resolve().parents[1]
ASSEMBLY = json.loads((PACKAGE / "assets/provenance/assembly.json").read_text())
EXTRACTION = json.loads((PACKAGE / "assets/provenance/extraction.json").read_text())
# Independent statement of the documented convention: native -Y->ROS +X,
# native -X->ROS +Y and native -Z->ROS +Z. Do not import the generator's Q.
NATIVE_TO_ROS = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
GROUP_LINK = {
    "fork": "caster_fork_link",
    "carrier": "caster_carrier_link",
    "positive_shell": "caster_positive_hemisphere_link",
    "negative_shell": "caster_negative_hemisphere_link",
    "positive_polar_roller": "caster_positive_polar_roller_link",
    "negative_polar_roller": "caster_negative_polar_roller_link",
}


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def pose(element):
    transform = np.eye(4)
    if element is None:
        return transform
    transform[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
    roll, pitch, yaw = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
    transform[:3, :3] = rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll)
    return transform


@lru_cache(maxsize=8)
def expanded(phase=0.0, mass_scale=1.0):
    document = xacro.process_file(
        str(PACKAGE / "urdf/caster_test_rig.urdf.xacro"),
        mappings={"carrier_phase": str(phase), "caster_mass_scale": str(mass_scale), "load_mass": "2.0"},
    )
    return ET.fromstring(document.toxml())


def forward_kinematics(robot, angles=None):
    angles = angles or {}
    links = {link.attrib["name"] for link in robot.findall("link")}
    joints = list(robot.findall("joint"))
    children = {joint.find("child").attrib["link"] for joint in joints}
    roots = links - children
    assert len(roots) == 1, "URDF must form one rooted tree"
    result = {roots.pop(): np.eye(4)}
    while joints:
        progress = False
        for joint in joints[:]:
            parent = joint.find("parent").attrib["link"]
            child = joint.find("child").attrib["link"]
            if parent not in result:
                continue
            assert child not in result, "A link cannot have two parents"
            motion = np.eye(4)
            angle = angles.get(joint.attrib["name"], 0.0)
            axis_element = joint.find("axis")
            axis = np.fromstring(axis_element.get("xyz"), sep=" ") if axis_element is not None else [1, 0, 0]
            if joint.attrib["type"] in ("continuous", "revolute"):
                motion[:3, :3] = rotation(axis, angle)
            elif joint.attrib["type"] == "prismatic":
                motion[:3, 3] = np.asarray(axis) * angle
            result[child] = result[parent] @ pose(joint.find("origin")) @ motion
            joints.remove(joint)
            progress = True
        assert progress, "Cycle or missing parent in URDF"
    return result


def occurrence_pose(robot, fk, occurrence, kind="visual"):
    link = robot.find(f"./link[@name='{GROUP_LINK[occurrence['group']]}']")
    element = link.find(f"./{kind}[@name='caster_{occurrence['key']}_{kind}']")
    assert element is not None, occurrence["path"]
    return np.linalg.inv(fk["caster_fork_link"]) @ fk[link.attrib["name"]] @ pose(element.find("origin"))


def mesh_vertices(part_key):
    record = EXTRACTION["parts"][part_key]
    content = (PACKAGE / record["mesh_file"]).read_bytes()
    triangles = struct.unpack_from("<I", content, 80)[0]
    assert len(content) == 84 + 50 * triangles, "Expected complete binary STL"
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    return np.frombuffer(content, dtype=dtype, offset=84, count=triangles)["vertices"].reshape(-1, 3).astype(float)


def inertia_matrix(link):
    values = link.find("inertial/inertia").attrib
    return np.array([[float(values["ixx"]), float(values["ixy"]), float(values["ixz"])],
                     [float(values["ixy"]), float(values["iyy"]), float(values["iyz"])],
                     [float(values["ixz"]), float(values["iyz"]), float(values["izz"])]])


def test_every_native_occurrence_pose_is_preserved():
    robot = expanded()
    fk = forward_kinematics(robot)
    Q = np.eye(4)
    Q[:3, :3] = NATIVE_TO_ROS
    occurrences = ASSEMBLY["occurrences"]
    assert len(occurrences) > 20, "Test the complete assembly, including hardware"
    assert len(robot.findall(".//visual")) == len(occurrences) + 1  # fixture load
    for occurrence in occurrences:
        native = np.asarray(occurrence["transform"])
        np.testing.assert_allclose(native[:3, :3].T @ native[:3, :3], np.eye(3), atol=1e-10)
        assert np.linalg.det(native[:3, :3]) == pytest.approx(1., abs=1e-10)
        np.testing.assert_allclose(occurrence_pose(robot, fk, occurrence), Q @ native, atol=2e-12,
                                   err_msg=occurrence["path"])


def test_five_independent_passive_joint_axes_and_centers():
    robot = expanded()
    fk = forward_kinematics(robot)
    passive = [j for j in robot.findall("joint") if j.attrib["name"].startswith("caster_") and j.attrib["type"] != "fixed"]
    assert len(passive) == 5
    assert all(j.attrib["type"] == "continuous" and j.find("mimic") is None for j in passive)
    for group, native in ASSEMBLY["joint_axes"].items():
        child = GROUP_LINK[group]
        joint = next(j for j in passive if j.find("child").get("link") == child)
        assert joint.find("parent").get("link") == GROUP_LINK[native["parent"]]
        origin_in_fork = np.linalg.inv(fk["caster_fork_link"]) @ fk[child]
        np.testing.assert_allclose(origin_in_fork[:3, 3], NATIVE_TO_ROS @ native["origin"], atol=2e-12)
        joint_axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
        np.testing.assert_allclose(origin_in_fork[:3, :3] @ joint_axis, NATIVE_TO_ROS @ native["axis"], atol=2e-12)
    commanded = {entry.text for plugin in robot.findall("./gazebo/plugin")
                 if plugin.get("name", "").endswith("::JointController") for entry in plugin.findall("joint_name")}
    assert commanded == {"rig_x_joint", "rig_y_joint", "rig_yaw_joint"}


@pytest.mark.parametrize("phase", [0.0, 0.4, math.pi / 2, math.pi])
def test_carrier_phase_rotates_all_moving_components_rigidly(phase):
    base = expanded()
    model = expanded(phase)
    base_fk = forward_kinematics(base)
    phase_fk = forward_kinematics(model)
    rotation_about_carrier = np.eye(4)
    rotation_about_carrier[:3, :3] = rotation([0, -1, 0], phase)
    for occurrence in ASSEMBLY["occurrences"]:
        expected = occurrence_pose(base, base_fk, occurrence)
        if occurrence["group"] != "fork":
            expected = rotation_about_carrier @ expected
        np.testing.assert_allclose(occurrence_pose(model, phase_fk, occurrence), expected, atol=3e-12)


def test_one_hemisphere_can_rotate_without_moving_the_other_or_rollers():
    robot = expanded()
    zero = forward_kinematics(robot)
    moved = forward_kinematics(robot, {"caster_positive_hemisphere_joint": 0.37})
    for occurrence in ASSEMBLY["occurrences"]:
        before = occurrence_pose(robot, zero, occurrence)
        after = occurrence_pose(robot, moved, occurrence)
        if occurrence["group"] == "positive_shell":
            assert not np.allclose(after[:3, :3], before[:3, :3])
        else:
            np.testing.assert_allclose(after, before, atol=2e-12)


def test_link_inertias_are_physical_and_mass_scaling_preserves_distribution():
    robot = expanded()
    scaled = expanded(mass_scale=1.7)
    for link in robot.findall("link"):
        if link.get("name") == "world":
            continue
        mass = float(link.find("inertial/mass").get("value"))
        I = inertia_matrix(link)
        assert math.isfinite(mass) and mass > 0
        assert np.isfinite(I).all()
        eigenvalues = np.linalg.eigvalsh(I)
        assert eigenvalues[0] > 0
        assert eigenvalues[-1] <= sum(eigenvalues[:2]) + 1e-12
        if link.get("name").startswith("caster_"):
            other = scaled.find(f"./link[@name='{link.get('name')}']")
            assert float(other.find("inertial/mass").get("value")) == pytest.approx(1.7 * mass)
            np.testing.assert_allclose(inertia_matrix(other), 1.7 * I, atol=1e-12)
            np.testing.assert_allclose(pose(other.find("inertial/origin")), pose(link.find("inertial/origin")), atol=1e-12)


def test_shell_sphere_centers_radius_seam_and_polar_opening():
    robot = expanded()
    fk = forward_kinematics(robot)
    split_axis = NATIVE_TO_ROS @ np.asarray(ASSEMBLY["joint_axes"]["positive_shell"]["axis"])
    for occurrence in ASSEMBLY["occurrences"]:
        if occurrence["part_key"] not in ("semi_spherical_wheel", "semi_spherical_wheel_2"):
            continue
        transform = occurrence_pose(robot, fk, occurrence)
        # Analytic spheres in the native part records establish a common center.
        spheres = [surface["definition"] for surface in EXTRACTION["parts"][occurrence["part_key"]]["analytic_surfaces_mm"]
                   if surface["kind"] == "sphere"]
        outer_spheres = [sphere for sphere in spheres if abs(sphere["radius"] - 100.) < 1e-7]
        assert outer_spheres
        for sphere in outer_spheres:
            np.testing.assert_allclose([sphere["center"][coordinate] for coordinate in ("x", "y", "z")],
                                       np.zeros(3), atol=1e-7)
        np.testing.assert_allclose(transform[:3, 3], np.zeros(3), atol=2e-12)
        vertices = mesh_vertices(occurrence["part_key"]) @ transform[:3, :3].T + transform[:3, 3]
        radii = np.linalg.norm(vertices, axis=1)
        assert radii.max() == pytest.approx(0.100, abs=2e-8)
        projection = vertices @ split_axis
        if occurrence["group"] == "positive_shell":
            assert projection.min() >= 0.010 - 2e-8
        else:
            assert projection.max() <= -0.010 + 2e-8
    cap = mesh_vertices("semi_spherical_wheel_2")
    assert cap[:, 1].max() == pytest.approx(0.09682458365518543, abs=1e-8)
    near_pole = cap[cap[:, 1] > 0.075]
    assert len(near_pole) > 50
    # The 25 mm radius polar cutout must remain open for the independent roller.
    assert np.linalg.norm(near_pole[:, [0, 2]], axis=1).min() == pytest.approx(0.025, abs=2e-8)


def test_collision_meshes_keep_cad_shell_openings_and_roller_geometry():
    robot = expanded()
    fk = forward_kinematics(robot)
    for occurrence in ASSEMBLY["occurrences"]:
        if occurrence["part_key"] not in ("semi_spherical_wheel", "semi_spherical_wheel_2", "roller"):
            continue
        record = EXTRACTION["parts"][occurrence["part_key"]]
        content = (PACKAGE / record["mesh_file"]).read_bytes()
        assert hashlib.sha256(content).hexdigest() == record["mesh_sha256"]
        link = robot.find(f"./link[@name='{GROUP_LINK[occurrence['group']]}']")
        collision = link.find(f"./collision[@name='caster_{occurrence['key']}_collision']")
        assert collision is not None
        mesh = collision.find("geometry/mesh")
        assert mesh is not None, "A sphere primitive would erase the actual seam and polar holes"
        assert mesh.get("filename") == "package://hamr_ball_caster/" + record["mesh_file"]
        np.testing.assert_allclose(occurrence_pose(robot, fk, occurrence, "collision"),
                                   occurrence_pose(robot, fk, occurrence), atol=2e-12)


def test_physics_summary_requires_named_dofs_and_reports_terminal_statistics():
    spec = importlib.util.spec_from_file_location("physics_check", PACKAGE / "scripts/run_physics_checks.py")
    check = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(check)
    names = check.PASSIVE_JOINTS | check.RIG_JOINTS | {"caster_mount_joint", "rig_world_joint"}
    samples = []
    for index in range(10):
        positions = {name: 0.0 for name in names}
        positions["rig_z_joint"] = -0.010 + index * 0.00002
        samples.append({"time": index * 0.2, "phase": "settle_final", "positions": positions,
                        "velocities": {name: 0.0 for name in names}})
    phases = [{"name": "settle_final", "duration": 2.0, "command": [0., 0., 0.]}]
    report = check.summarize(samples, phases, 0.01, 100.)
    assert report["passed"]
    assert len(report["metrics"]["passive_joints"]) == 5
    phase = report["metrics"]["phases"]["settle_final"]
    assert phase["terminal_z_median_m"] == pytest.approx(-0.00987)
    assert phase["terminal_z_mean_m"] == pytest.approx(-0.00987)
    for sample in samples:
        del sample["positions"]["caster_negative_polar_roller_joint"]
        del sample["velocities"]["caster_negative_polar_roller_joint"]
    # Two fixed joints must not hide the missing fifth passive degree of freedom.
    report = check.summarize(samples, phases, 0.01, 100.)
    assert not report["passed"]
    assert any("caster_negative_polar_roller_joint" in failure for failure in report["failures"])
