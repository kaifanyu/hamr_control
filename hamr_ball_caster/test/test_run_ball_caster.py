"""Ball runs retain their real passive joints, world contacts and startup policy."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from run_ball_caster import BALL_JOINT_NAMES, export_caster_measurements, normalize_configuration
from prepare_caster_world import prepare_world


def test_configuration_uses_actual_ball_joints_and_explicit_heading_only_gate():
    original = {"startup_alignment": {"enabled": True}}
    config = normalize_configuration(original)
    assert config["passive_joint_names"] == BALL_JOINT_NAMES
    assert len(config["passive_joint_names"]) == 10
    assert config["startup_alignment"]["caster_model"] == "ball"
    assert original == {"startup_alignment": {"enabled": True}}
    with pytest.raises(ValueError, match="ten passive"):
        normalize_configuration({"passive_joint_names": ["left_caster_swivel_joint"]})
    with pytest.raises(ValueError, match="caster_model"):
        normalize_configuration({"startup_alignment": {"enabled": True, "caster_model": "swivel"}})


def test_ball_feedback_export_all_axes_without_requiring_each_axis_to_roll(tmp_path):
    samples = [{"simulation_time_s": i, "trajectory_time_s": i, "phase": "motion",
                "joint_positions_rad": {name: i*2. if "positive_hemisphere" in name else 0.
                                        for name in BALL_JOINT_NAMES},
                "joint_velocities_rad_s": {name: 2. if "positive_hemisphere" in name else 0.
                                          for name in BALL_JOINT_NAMES}}
               for i in range(3)]
    report = export_caster_measurements({"samples": samples}, tmp_path)
    assert report["passed"]
    assert len(report["joints"]) == 10
    assert len((tmp_path / "caster_joints.csv").read_text().splitlines()) == 4
    for sample in samples:
        sample["joint_positions_rad"]["right_caster_positive_hemisphere_joint"] = 0.
    assert not export_caster_measurements({"samples": samples}, tmp_path)["passed"]
    samples[-1]["joint_positions_rad"]["right_caster_carrier_joint"] = float("nan")
    with pytest.raises(ValueError, match="Non-finite"):
        export_caster_measurements({"samples": samples}, tmp_path)


def test_ball_world_uses_same_camera_and_contacts_as_swivel(tmp_path):
    source = PACKAGE / "worlds/ball_caster_comparison.sdf"
    world = tmp_path / "world.sdf"
    prepare_world(source, world, [[0., 0.], [0., 2.], [-1., 2.]], .001, .4)
    tree = ET.parse(world).getroot().find("world")
    assert tree.get("name") == "ball_caster_comparison"
    assert float(tree.findtext("physics/real_time_factor")) == .4
    camera = tree.find("./model[@name='overview_camera']/link/sensor")
    assert camera.findtext("topic") == "/hamr/recording_camera/image"
    assert camera.findtext("update_rate") == "25"
    assert tree.find("./model[@name='ground_plane']/link/collision") is not None
    assert not tree.findall("./model[@name='trajectory_markings']//collision")


def test_overview_filter_retains_every_nonvisual_property_and_snapshots_actual_robot():
    from run_ball_caster import load_overview_filter, prepare_robot_visuals
    original = '''<robot name="example"><link name="ball">
      <inertial><mass value="1.23"/><inertia ixx=".1" iyy=".2" izz=".3"/></inertial>
      <visual><geometry><mesh filename="package://hamr_ball_caster/meshes/skate_bearing.stl"/></geometry></visual>
      <visual><geometry><sphere radius=".05"/></geometry></visual>
      <collision><geometry><mesh filename="package://hamr_ball_caster/meshes/skate_bearing.stl"/></geometry></collision>
      </link><joint name="passive" type="continuous"><dynamics damping=".0002" friction="0"/></joint>
      <gazebo reference="ball"><mu1>1.0</mu1><mu2>1.0</mu2></gazebo></robot>'''
    filtered, report = prepare_robot_visuals(original, "overview", load_overview_filter(PACKAGE))
    robot = ET.fromstring(filtered)
    assert len(robot.findall("./link/visual")) == 1
    assert robot.find("./link/collision/geometry/mesh").get("filename").endswith("skate_bearing.stl")
    assert robot.find("./link/inertial/mass").get("value") == "1.23"
    assert robot.find("./joint/dynamics").get("damping") == ".0002"
    assert robot.findtext("./gazebo/mu1") == "1.0"
    assert report["removed_visual_count"] == 1
    assert report["nonvisual_structure_identical"]
    assert report["full_nonvisual_structural_sha256"] == report["rendered_nonvisual_structural_sha256"]
    unchanged, full_report = prepare_robot_visuals(original, "full", lambda text: "must not run")
    assert unchanged == original and full_report["removed_visual_count"] == 0
    with pytest.raises(ValueError, match="nonvisual"):
        prepare_robot_visuals(original, "overview", lambda text: text.replace('value="1.23"', 'value="99"'))
