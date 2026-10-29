"""Keep Simulation Studio geometry and encoder coordinates identical in replay."""
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from caster_pair_scene import MODEL_FILE, build_robot, model_name, paired_joint_angles
from render_caster_encoder_video import link_transforms, package_share


@pytest.fixture
def source_robot():
    xacro = pytest.importorskip("xacro")
    return ET.fromstring(xacro.process_file(str(
        package_share("hamr_ball_caster") / "urdf" / MODEL_FILE),
        mappings={"enable_closeup_camera": "false"}).toxml())


def test_replay_preserves_simulation_studio_visuals_mounts_and_joint_origins(source_robot):
    before = ET.tostring(source_robot)
    robot, provenance = build_robot(source_robot)
    assert ET.tostring(source_robot) == before
    assert provenance["chassis_model"].endswith("compa_ball_caster_recording.urdf.xacro")
    assert provenance["caster_uniform_scale"] == 1.
    assert provenance["joint_angle_scale"] == 1.
    assert provenance["incomplete_mesh_fallbacks"] == []
    assert len(robot.findall("link")) == len(source_robot.findall("link"))
    for source in source_robot.findall("link"):
        output = robot.find(f"./link[@name='{source.get('name')}']")
        assert [ET.tostring(v) for v in output.findall("visual")] == [
            ET.tostring(v) for v in source.findall("visual")]
    assert [ET.tostring(j) for j in robot.findall("joint")] == [
        ET.tostring(j) for j in source_robot.findall("joint")]
    transforms = link_transforms(robot, {})
    np.testing.assert_allclose(transforms["left_caster_fork_link"][:3, 3], [.315, .257, .10348])
    np.testing.assert_allclose(transforms["right_caster_fork_link"][:3, 3], [.315, -.257, .10348])
    assert not robot.findall("gazebo")


def test_changing_one_wheel_cannot_rotate_other_wheel_or_body(source_robot):
    robot, _ = build_robot(source_robot)
    frames = {wheel: {"joints": {"caster_carrier_joint": .2,
                                  "caster_positive_hemisphere_joint": .3,
                                  "caster_negative_hemisphere_joint": -.4}}
              for wheel in ("front_left", "front_right")}
    before = link_transforms(robot, paired_joint_angles(frames))
    frames["front_left"]["joints"]["caster_positive_hemisphere_joint"] += 1.1
    moved = link_transforms(robot, paired_joint_angles(frames))
    changed = {link for link in before if not np.allclose(before[link], moved[link])}
    assert changed == {"left_caster_positive_hemisphere_link"}
    frames["front_left"]["joints"]["caster_carrier_joint"] += .7
    moved = link_transforms(robot, paired_joint_angles(frames))
    for link in before:
        if not link.startswith("left_caster_"):
            np.testing.assert_allclose(moved[link], before[link])


def test_saved_standalone_encoder_references_keep_same_phase_in_full_model(source_robot):
    robot, _ = build_robot(source_robot)
    standalone = ET.parse(package_share("hamr_ball_caster") / "urdf/ball_caster.urdf").getroot()
    for angle in [0., 3.620181390266952, 5.190977717061848, -1.234]:
        joints = {"caster_carrier_joint": angle,
                  "caster_positive_hemisphere_joint": .71,
                  "caster_negative_hemisphere_joint": -.83}
        frames = {wheel: {"joints": joints} for wheel in ("front_left", "front_right")}
        full = link_transforms(robot, paired_joint_angles(frames))
        local = link_transforms(standalone, joints)
        for wheel in frames:
            inverse_mount = np.linalg.inv(full[model_name(wheel, "caster_fork_link")])
            for name, pose in local.items():
                if name != "caster_reference":
                    np.testing.assert_allclose(
                        inverse_mount @ full[model_name(wheel, name)], pose, atol=1e-12)


def test_full_vehicle_meshes_are_complete_original_assets(source_robot):
    robot, _ = build_robot(source_robot)
    meshes = {mesh.get("filename") for mesh in robot.findall("./link/visual/geometry/mesh")}
    assert any("meshes/compa/base_link.STL" in mesh for mesh in meshes)
    assert any("meshes/compa/pitch_link.STL" in mesh for mesh in meshes)
    for uri in meshes:
        package, relative = uri.removeprefix("package://").split("/", 1)
        path = package_share(package) / relative
        with path.open("rb") as stream:
            header = stream.read(84)
        assert len(header) == 84
        assert path.stat().st_size >= 84 + struct.unpack_from("<I", header, 80)[0] * 50
