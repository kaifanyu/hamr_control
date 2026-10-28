"""Deployment contracts for family isolation and actual ROS topic capture."""
import importlib.util
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ordinary_caster.model import dual_caster_description


def test_model_joint_names_axes_and_distinct_frames():
    model = ET.fromstring(dual_caster_description(ROOT.parent / "hamr_swivel_caster/urdf/swivel_caster.urdf"))
    joints = {j.get("name"): j for j in model.findall("joint")}
    for wheel in ("front_left", "front_right"):
        assert joints[wheel + "_caster_swivel_joint"].find("axis").get("xyz") == "0 0 1"
        assert joints[wheel + "_caster_wheel_joint"].find("axis").get("xyz") == "0 1 0"
    links = [j.get("name") for j in model.findall("link")]
    assert len(links) == len(set(links)) == 9
    assert all(element.get("link") in links for element in model.iter() if element.tag in ("parent", "child"))


def test_bag_regex_and_qos_include_both_families():
    qos = yaml.safe_load((ROOT / "config/record_qos.yaml").read_text())
    for script in ("record_hamr_test_bag", "record_hamr_vicon_bag"):
        text = (ROOT / "scripts" / script).read_text()
        patterns = re.findall(r'TOPIC_REGEX="\$\{TOPIC_REGEX\}\|(.*)"', text)
        pattern = re.compile("|".join(patterns))
        for wheel in ("front_left", "front_right"):
            for topic in ("packets", "metadata", "diagnostics", "swivel/joint_states", "wheel/joint_states"):
                name = f"/ordinary_caster/{wheel}/{topic}"
                assert pattern.fullmatch(name)
                assert name in qos
            assert qos[f"/ordinary_caster/{wheel}/metadata"]["durability"] == "transient_local"
            assert pattern.fullmatch(f"/caster_encoder/{wheel}/CENTER/joint_states")
        assert not pattern.fullmatch("/ordinary_caster/front_left/cmd_vel")


def test_live_configuration_ports_and_launch_markers():
    config = yaml.safe_load((ROOT / "config/ordinary_caster/encoders.yaml").read_text())
    launch = ET.parse(ROOT / "launch/ordinary_caster/encoders.launch.xml")
    nodes = [n for n in launch.getroot().findall("node") if n.get("exec") == "ordinary_caster_bridge"]
    assert len(nodes) == 2
    for node, wheel, port in zip(nodes, ("front_left", "front_right"), (5007, 5008)):
        params = config["/" + node.get("name")]["ros__parameters"]
        assert params["wheel_id"] == params["configured_wheel_id"] == wheel
        assert params["udp_port"] == port
        assert node.find("param[@name='require_config']").get("value") == "true"
