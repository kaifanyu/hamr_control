"""Exercise the actual recorder command and passive encoder launch wiring."""

import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_caster_entrypoints_are_executable_for_symlink_install():
    # ament's symlink install preserves source permissions; a readable script
    # may pass Python unit tests but still fail ros2 run/launch discovery.
    for path in (
        PACKAGE / "scripts/caster_encoder_bridge",
        PACKAGE / "scripts/record_caster_rotation.py",
        PACKAGE / "scripts/replay_caster_encoder_joints",
        PACKAGE / "scripts/render_caster_encoder_video.py",
        PACKAGE.parent / "rosbags/analyze_caster_encoders.py",
    ):
        assert path.stat().st_mode & 0o111, f"Missing executable permission: {path}"


TOPICS = [
    f"/caster_encoder/{wheel}{topic}"
    for wheel in ("", "front_left/", "front_right/")
    for topic in ("packets", "metadata", "diagnostics", "H1/joint_states", "H2/joint_states", "CENTER/joint_states")
]


@pytest.mark.parametrize("script", ["record_hamr_test_bag", "record_hamr_vicon_bag"])
def test_recorders_select_encoder_payloads_and_qos(tmp_path, script):
    if os.name == "nt" or shutil.which("bash") is None:
        pytest.skip("Recorder execution requires a POSIX bash environment")
    # Invoke the real wrapper with a stub ROS CLI: validate the final regex and
    # QoS arguments, including paths containing spaces, without touching a robot.
    cli = tmp_path / "ros2"
    cli.write_text("#!/usr/bin/env python3\nimport json, sys\nprint(json.dumps(sys.argv[1:]))\n")
    cli.chmod(0o755)
    env = dict(os.environ, PATH=f"{tmp_path}{os.pathsep}{os.environ['PATH']}")
    env.pop("HAMR_BAG_QOS_OVERRIDES", None)
    result = subprocess.run(
        ["bash", str(PACKAGE / "scripts" / script), "fixture", str(tmp_path / "bag with spaces")],
        env=env, text=True, capture_output=True, check=True,
    )
    args = json.loads(result.stdout.splitlines()[-1])
    assert args[:2] == ["bag", "record"]
    regex = re.compile(args[args.index("--regex") + 1])
    for topic in TOPICS + ["/left_wheel/encoder_ticks", "/HAMR_base/odom"]:
        assert regex.fullmatch(topic), topic
    assert not regex.fullmatch("/caster_encoder/debug_image")
    assert not regex.fullmatch("/unrelated/packets")
    qos_path = Path(args[args.index("--qos-profile-overrides-path") + 1])
    qos = yaml.safe_load(qos_path.read_text())
    for topic in TOPICS:
        assert qos[topic]["reliability"] == "reliable"
    for wheel in ("", "front_left/", "front_right/"):
        assert qos[f"/caster_encoder/{wheel}metadata"]["durability"] == "transient_local"
    assert not regex.fullmatch("/caster_encoder/rear_left/packets")


def test_encoder_only_launch_is_passive_and_hw_does_not_duplicate_bag():
    passive = ET.parse(PACKAGE / "launch/caster_encoders.launch.xml").getroot()
    assert {(node.get("pkg"), node.get("exec")) for node in passive.findall("node")} == {
        ("hamr_bringup", "caster_encoder_bridge"),
        ("hamr_bringup", "record_hamr_test_bag"),
    }
    assert not passive.findall("include")
    hardware = ET.parse(PACKAGE / "launch/hamr_HW.launch.xml").getroot()
    include = next(item for item in hardware.iter("include")
                   if item.get("file").endswith("/caster_encoders.launch.xml"))
    assert include.get("if") == "$(var caster_encoders)"
    forwarded = {arg.get("name"): arg.get("value") for arg in include.findall("arg")}
    assert forwarded["record_bag"] == "false"
    assert forwarded["config_file"] == "$(var caster_encoder_config)"
    assert forwarded["mode"] == "$(var caster_encoder_mode)"
    config = yaml.safe_load((PACKAGE / "config/caster_encoders.yaml").read_text())
    for wheel, port in (("front_left", 5005), ("front_right", 5006)):
        settings = config[f"/caster_encoder_{wheel}"]["ros__parameters"]
        assert settings["wheel_id"] == wheel
        assert settings["configured_wheel_id"] == wheel
        assert settings["udp_port"] == port
        assert settings["expected_device_ids"] == ["", "", ""]
        assert settings["expected_source_ips"] == ["", "", ""]
    assert sum(node.get("exec") == "record_hamr_test_bag"
               for node in hardware.findall("node")) == 1


@pytest.mark.parametrize("record_bag", ["true", "false"])
@pytest.mark.parametrize("caster_encoders", ["true", "false"])
@pytest.mark.parametrize("mode", ["dual", "legacy"])
def test_hardware_recording_survives_encoder_include(
        monkeypatch, record_bag, caster_encoders, mode):
    """Evaluate real launch scopes and conditions without executing any nodes."""
    pytest.importorskip("launch_xml")
    from launch import LaunchContext
    from launch.launch_description_sources import AnyLaunchDescriptionSource
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare

    monkeypatch.setattr(FindPackageShare, "find", lambda self, package: str(PACKAGE.parent / package))
    context = LaunchContext()
    context.launch_configurations.update({
        "record_bag": record_bag, "caster_encoders": caster_encoders,
        "caster_encoder_mode": mode,
        "caster_encoder_config": str(PACKAGE / "config/caster_encoders.yaml"),
        "record_camera": "false", "run_foxglove": "false",
    })
    description = AnyLaunchDescriptionSource(
        str(PACKAGE / "launch/hamr_HW.launch.xml")
    ).get_launch_description(context)
    enabled_nodes = []

    def visit(entity):
        if isinstance(entity, Node):
            if entity.condition is None or entity.condition.evaluate(context):
                enabled_nodes.append(perform_substitutions(
                    context, normalize_to_list_of_substitutions(entity.node_executable)))
            return  # Never open serial ports, bind UDP sockets, or start a recorder.
        for child in entity.visit(context) or []:
            visit(child)

    visit(description)
    assert enabled_nodes.count("record_hamr_test_bag") == (1 if record_bag == "true" else 0)
    expected_bridges = (2 if mode == "dual" else 1) if caster_encoders == "true" else 0
    assert enabled_nodes.count("caster_encoder_bridge") == expected_bridges
    assert context.launch_configurations["record_bag"] == record_bag


@pytest.mark.parametrize("mode", ["dual", "legacy"])
def test_launch_frontend_resolves_encoder_parameters(mode):
    pytest.importorskip("launch_xml")
    from launch import LaunchContext
    from launch.launch_description_sources import AnyLaunchDescriptionSource
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    context = LaunchContext()
    context.launch_configurations.update({
        "mode": mode, "config_file": str(PACKAGE / "config/caster_encoders.yaml"),
        "bind_address": "0.0.0.0", "udp_port": "15005",
        "stale_timeout_s": "2.5", "record_bag": "false",
    })
    description = AnyLaunchDescriptionSource(
        str(PACKAGE / "launch/caster_encoders.launch.xml")
    ).get_launch_description(context)
    bridges = [action for action in description.entities
               if isinstance(action, Node) and perform_substitutions(
                   context, normalize_to_list_of_substitutions(action.node_executable)
               ) == "caster_encoder_bridge" and action.condition.evaluate(context)]
    assert len(bridges) == (2 if mode == "dual" else 1)
    for bridge in bridges:
        parameters = evaluate_parameters(context, bridge._Node__parameters)
        merged = {key: value for group in parameters if isinstance(group, dict)
                  for key, value in group.items()}
        assert merged["use_sim_time"] is False
        if mode == "dual":
            assert merged["wheel_id"] in ("front_left", "front_right")
            assert merged["legacy_mode"] is False
            assert merged["require_config"] is True
            assert Path(parameters[0]) == PACKAGE / "config/caster_encoders.yaml"
        else:
            assert merged == {"wheel_id": "", "legacy_mode": True,
                              "bind_address": "0.0.0.0", "udp_port": 15005,
                              "stale_timeout_s": 2.5, "use_sim_time": False}
