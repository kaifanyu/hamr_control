"""Shared terrain launch must preserve world physics and spawn either model."""
import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "terrain_launch", ROOT / "hamr_terrain/launch/terrain_sim.launch.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize("caster", ["ball", "swivel", "none"])
def test_shared_launch_preserves_world_and_uses_its_name(tmp_path, monkeypatch, caster):
    share = tmp_path / "terrain package"
    (share / "worlds").mkdir(parents=True)
    world = share / "worlds/road_bumps.sdf"
    world.write_text('''<sdf version="1.9"><world name="custom_terrain_name">
      <physics name="test" type="ignored"><max_step_size>0.0007</max_step_size></physics>
      <model name="road"><link name="surface"><collision name="contact"><surface>
      <friction><ode><mu>0.31</mu><mu2>0.73</mu2></ode></friction>
      </surface></collision></link></model></world></sdf>''')
    original = world.read_bytes()
    snapshot = tmp_path / "robot snapshot.urdf"
    snapshot.write_text('<robot name="compa"><link name="base_footprint"/></robot>')
    monkeypatch.setattr(MODULE, "get_package_share_directory", lambda name:
                        str(share if name == "hamr_terrain" else ROOT / name))
    monkeypatch.setattr(MODULE, "Node", lambda **kwargs: {"node": kwargs})
    monkeypatch.setattr(MODULE, "IncludeLaunchDescription", lambda source, **kwargs:
                        {"include": source, "arguments": dict(kwargs["launch_arguments"])})
    context = LaunchContext()
    context.launch_configurations.update({
        "terrain": "road_bumps", "caster": caster, "gui": "false", "visual_detail": "full",
        "closeup_camera": "false", "headless_rendering": "true", "spawn_x": "0", "spawn_y": "-2.8", "spawn_z": "0",
        "initial_yaw": str(math.pi/2), "world": "", "gui_config": "",
        "robot_description_file": str(snapshot),
    })
    actions = MODULE._start(context)
    nodes = [action["node"] for action in actions if isinstance(action, dict) and "node" in action]
    assert world.read_bytes() == original
    included = next(action for action in actions if isinstance(action, dict) and "include" in action)
    assert "-s " in included["arguments"]["gz_args"]
    assert "--headless-rendering " in included["arguments"]["gz_args"]
    if caster == "none":
        assert len(nodes) == 1
        assert nodes[0]["arguments"] == ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
        return
    spawn = next(node for node in nodes if node["executable"] == "create")["arguments"]
    for option, expected in (("-world", "custom_terrain_name"), ("-x", "0.0"),
                             ("-y", "-2.8"), ("-z", "0.0"), ("-Y", str(math.pi/2))):
        assert spawn[spawn.index(option)+1] == expected
    bridge = next(node for node in nodes if node["executable"] == "parameter_bridge")
    assert ("hamr_ball_caster" if caster == "ball" else "hamr_swivel_caster") in bridge["parameters"][0]["config_file"]


@pytest.mark.parametrize("caster", ["ball", "swivel"])
def test_default_models_can_disable_unused_closeup_sensor(caster):
    description, _ = MODULE.robot_description(caster, "", "overview", closeup_camera=False)
    model = ET.fromstring(description)
    assert model.get("name") == "compa"
    assert not model.findall(".//sensor[@type='camera']")
    passive = [joint for joint in model.findall("joint") if joint.get("type") == "continuous"
               and joint.get("name").startswith(("left_caster_", "right_caster_"))]
    assert len(passive) == (10 if caster == "ball" else 4)


def test_terrain_path_must_not_escape_world_directory(tmp_path):
    with pytest.raises(ValueError, match="lowercase world name"):
        MODULE.resolve_world(tmp_path, "../outside")
