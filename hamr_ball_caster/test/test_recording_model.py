"""Recording simplification must preserve the complete physical model."""
import ast
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro

PACKAGE = Path(__file__).resolve().parents[1]


def test_overview_preserves_every_nonvisual_model_element():
    source = ast.parse((PACKAGE / "launch/ball_caster.launch.py").read_text())
    function = next(node for node in source.body if isinstance(node, ast.FunctionDef)
                    and node.name == "overview_description")
    namespace = {"ET": ET, "Path": Path}
    exec(compile(ast.Module(body=[function], type_ignores=[]), "overview", "exec"), namespace)
    full = ET.fromstring(xacro.process_file(str(PACKAGE / "urdf/compa_ball_caster.urdf.xacro")).toxml())
    overview = ET.fromstring(namespace["overview_description"](ET.tostring(full, encoding="unicode")))
    count_full = len(full.findall(".//visual"))
    count_overview = len(overview.findall(".//visual"))
    assert count_overview < count_full - 30
    assert len(overview.findall(".//visual/geometry/mesh")) > 20
    # Visible rolling surfaces must keep their original geometry and poses.
    for link in full.findall("link"):
        if any(word in link.get("name") for word in ("hemisphere_link", "polar_roller_link")):
            remaining = overview.find(f"./link[@name='{link.get('name')}']")
            for visual in link.findall("visual"):
                mesh = visual.find("geometry/mesh")
                if mesh is not None and any(word in mesh.get("filename") for word in ("semi_spherical", "/roller.stl")):
                    counterpart = remaining.find(f"./visual[@name='{visual.get('name')}']")
                    assert ET.tostring(visual) == ET.tostring(counterpart)
    for robot in (full, overview):
        for link in robot.findall("link"):
            for visual in list(link.findall("visual")):
                link.remove(visual)
        # Whitespace is serialization formatting, not model content.
        for element in robot.iter():
            if element.text is not None and not element.text.strip():
                element.text = None
            if element.tail is not None and not element.tail.strip():
                element.tail = None
    assert ET.tostring(full) == ET.tostring(overview)


def test_continuous_reference_overlay_preserves_world_physics(tmp_path):
    spec = importlib.util.spec_from_file_location("continuous_world", PACKAGE/"tools/prepare_continuous_world.py")
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    source = PACKAGE/"worlds/hamr_waypoint_recording.sdf"
    output = tmp_path/"continuous.sdf"
    provenance = helper.prepare_continuous_world(source, output, {"samples": [
        {"position_m": [0., 0.]}, {"position_m": [0., 0.]},
        {"position_m": [0., 1.]}, {"position_m": [-.2, 1.2]},
    ]})
    original, changed = ET.parse(source).getroot(), ET.parse(output).getroot()
    world = changed.find("world")
    marking = world.find("./model[@name='continuous_reference_marking']")
    assert marking is not None
    assert not marking.findall(".//collision")
    assert marking.find("static").text == "true"
    assert provenance["triangles"] == 4
    assert Path(provenance["reference_mesh"]).is_file()
    world.remove(marking)
    for root in (original, changed):
        for element in root.iter():
            if element.text is not None and not element.text.strip():
                element.text = None
            if element.tail is not None and not element.tail.strip():
                element.tail = None
    assert ET.tostring(original) == ET.tostring(changed)
