"""Check real launch parsing without opening a camera or starting the robot."""

from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


PACKAGE = Path(__file__).resolve().parents[1]


def test_camera_launch_passes_paths_with_spaces_to_one_dual_recorder(tmp_path):
    launch_file = Path(__file__).resolve().parents[1] / "launch/camera_recording.launch.xml"
    description = get_launch_description_from_any_launch_file(str(launch_file))
    nodes = [action for action in description.entities if isinstance(action, Node)]
    assert len(nodes) == 1
    context = LaunchContext()
    repository = str(tmp_path / "caster vision")
    config = str(tmp_path / "camera profiles" / "rig.yaml")
    output = str(tmp_path / "camera clips")
    context.launch_configurations.update({
        "camera_repository": repository,
        "camera_config": config,
        "recording_root": output,
        "camera_segment_mib": "512",
    })
    arguments = [perform_substitutions(context, arg) for arg in nodes[0].cmd[1:]]
    assert arguments == [
        "--repository", repository, "--config", config,
        "--output-dir", output, "--segment-mib", "512", "--ros-args",
    ]
    xml_node = ET.parse(launch_file).getroot().find("node")
    assert xml_node.get("exec") == "record_dual_cameras"
    assert float(perform_substitutions(context, nodes[0].sigterm_timeout)) >= 15


def test_hardware_launch_records_both_by_default_with_opt_out_and_external_motion():
    launch_file = PACKAGE / "launch/hamr_HW.launch.xml"
    description = get_launch_description_from_any_launch_file(str(launch_file))
    includes = [action for action in description.entities
                if isinstance(action, IncludeLaunchDescription)]
    assert len(includes) == 1
    context = LaunchContext()
    context.launch_configurations["record_camera"] = "true"
    assert includes[0].condition.evaluate(context)
    context.launch_configurations["record_camera"] = "false"
    assert not includes[0].condition.evaluate(context)

    root = ET.parse(launch_file).getroot()
    defaults = {arg.get("name"): arg.get("default") for arg in root.findall("arg")}
    assert defaults["record_camera"] == "true"
    assert defaults["record_bag"] == "true"
    include = root.find("include")
    assert include.get("file").endswith("/launch/camera_recording.launch.xml")
    assert {arg.get("name"): arg.get("value") for arg in include.findall("arg")} == {
        key: f"$(var {key})" for key in
        ("camera_repository", "camera_config", "recording_root", "camera_segment_mib")
    }
    assert not [node for node in root.iter("node")
                if node.get("pkg") == "reference_trajectory"]
    assert not [node for node in root.iter("node")
                if node.get("exec") in ("record_camera_clip", "record_dual_cameras")]
