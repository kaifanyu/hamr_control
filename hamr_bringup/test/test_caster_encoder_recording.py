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
TOPICS = [
    "/caster_encoder/packets", "/caster_encoder/metadata",
    "/caster_encoder/diagnostics",
    *[f"/caster_encoder/{role}/joint_states" for role in ("H1", "H2", "CENTER")],
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
    assert qos["/caster_encoder/metadata"]["durability"] == "transient_local"


def test_encoder_only_launch_is_passive_and_hw_does_not_duplicate_bag():
    passive = ET.parse(PACKAGE / "launch/caster_encoders.launch.xml").getroot()
    assert {(node.get("pkg"), node.get("exec")) for node in passive.findall("node")} == {
        ("hamr_bringup", "caster_encoder_bridge"),
        ("hamr_bringup", "record_hamr_test_bag"),
    }
    assert not passive.findall("include")
    hardware = ET.parse(PACKAGE / "launch/hamr_HW.launch.xml").getroot()
    include = next(item for item in hardware.findall("include")
                   if item.get("file").endswith("/caster_encoders.launch.xml"))
    assert include.get("if") == "$(var caster_encoders)"
    forwarded = {arg.get("name"): arg.get("value") for arg in include.findall("arg")}
    assert forwarded["record_bag"] == "false"
    assert sum(node.get("exec") == "record_hamr_test_bag"
               for node in hardware.findall("node")) == 1


def test_launch_frontend_resolves_encoder_parameters():
    pytest.importorskip("launch_xml")
    from launch import LaunchContext
    from launch.launch_description_sources import AnyLaunchDescriptionSource
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    context = LaunchContext()
    context.launch_configurations.update({
        "bind_address": "0.0.0.0", "udp_port": "15005",
        "stale_timeout_s": "2.5", "record_bag": "false",
    })
    description = AnyLaunchDescriptionSource(
        str(PACKAGE / "launch/caster_encoders.launch.xml")
    ).get_launch_description(context)
    bridge = next(action for action in description.entities
                  if isinstance(action, Node) and perform_substitutions(
                      context, normalize_to_list_of_substitutions(action.node_executable)
                  ) == "caster_encoder_bridge")
    parameters = evaluate_parameters(context, bridge._Node__parameters)
    merged = {key: value for group in parameters for key, value in group.items()}
    assert merged == {"bind_address": "0.0.0.0", "udp_port": 15005,
                      "stale_timeout_s": 2.5, "use_sim_time": False}
