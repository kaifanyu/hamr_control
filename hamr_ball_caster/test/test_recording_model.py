"""Recording simplification must preserve the complete physical model."""
import ast
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import xacro

PACKAGE = Path(__file__).resolve().parents[1]


def _substantive_xml(robot):
    return [(node.tag, node.attrib, (node.text or '').strip()) for node in robot.iter()]


def test_ball_comparison_wrapper_preserves_original_model_when_camera_disabled():
    original = ET.fromstring(xacro.process_file(str(PACKAGE / 'urdf/compa_ball_caster.urdf.xacro')).toxml())
    recording = ET.fromstring(xacro.process_file(
        str(PACKAGE / 'urdf/compa_ball_caster_recording.urdf.xacro'),
        mappings={'enable_closeup_camera': 'false'},
    ).toxml())
    assert _substantive_xml(recording) == _substantive_xml(original)


def test_ball_comparison_camera_adds_no_collision_or_caster_actuation():
    original = ET.fromstring(xacro.process_file(str(PACKAGE / 'urdf/compa_ball_caster.urdf.xacro')).toxml())
    recording = ET.fromstring(xacro.process_file(str(PACKAGE / 'urdf/compa_ball_caster_recording.urdf.xacro')).toxml())
    for name in ('ball_closeup_camera_link', 'ball_closeup_camera_joint'):
        node = next(node for node in recording if node.get('name') == name)
        if node.tag == 'link':
            assert node.find('collision') is None
            assert float(node.find('inertial/mass').get('value')) == .001
        else:
            assert node.get('type') == 'fixed'
            assert node.find('parent').get('link') == 'right_rocker_link'
        recording.remove(node)
    camera = recording.find("./gazebo[@reference='ball_closeup_camera_link']")
    assert camera.findtext('sensor/topic') == '/hamr/ball_caster/closeup/image'
    assert camera.findtext('sensor/update_rate') == '25'
    assert camera.findtext('sensor/camera/image/width') == '1280'
    assert camera.findtext('sensor/camera/image/height') == '720'
    recording.remove(camera)
    assert _substantive_xml(recording) == _substantive_xml(original)


def test_ball_comparison_camera_contains_the_rolling_mesh_envelope():
    from test_geometry import forward_kinematics
    recording = ET.fromstring(xacro.process_file(str(PACKAGE / 'urdf/compa_ball_caster_recording.urdf.xacro')).toxml())
    fk = forward_kinematics(recording)
    camera_inv = np.linalg.inv(fk['ball_closeup_camera_link'])
    center_frame = fk['right_caster_carrier_link']
    points = []
    for latitude in np.linspace(-np.pi/2, np.pi/2, 19):
        for longitude in np.linspace(-np.pi, np.pi, 36):
            direction = np.array([np.cos(latitude)*np.cos(longitude),
                                  np.cos(latitude)*np.sin(longitude), np.sin(latitude)])
            # CAD shells have a 100 mm radius; the polar rollers extend the
            # recovered rolling-component envelope to about 101.799 mm.
            points.append(camera_inv @ center_frame @ np.r_[.1018*direction, 1.])
    projected = np.array(points)
    horizontal = np.tan(.5)
    vertical = horizontal*720/1280
    assert np.all(projected[:, 0] > .02)
    assert np.all(np.abs(projected[:, 1]/projected[:, 0]) < horizontal)
    assert np.all(np.abs(projected[:, 2]/projected[:, 0]) < vertical)


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
