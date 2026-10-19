"""Create disposable viewing additions around an unchanged terrain world."""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET


def camera_pose(bounds=None):
    """An oblique view aimed at the centre of the selected map."""
    xmin, xmax, ymin, ymax = bounds or (-4., 4., -4., 4.)
    values = (xmin, xmax, ymin, ymax)
    if not all(math.isfinite(float(v)) for v in values) or xmin >= xmax or ymin >= ymax:
        raise ValueError("Map bounds must be finite (xmin, xmax, ymin, ymax)")
    span = max(xmax - xmin, ymax - ymin, 4.)
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    dx, dy, dz = .72 * span, -.85 * span, 1.1 * span
    return (cx + dx, cy + dy, dz, 0., math.atan2(dz, math.hypot(dx, dy)),
            math.atan2(-dy, -dx))


def _child(parent, tag, value=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if value is not None:
        element.text = str(value)
    return element


def _plugin(parent, filename, name):
    for item in parent.findall("plugin"):
        if item.get("name") == name or item.get("filename") == filename:
            return item
    return _child(parent, "plugin", filename=filename, name=name)


def prepare_world(source, destination, bounds=None, *, log_path=None, cameras=True):
    """Configure camera sensors and optional native state log for a run.

    Physics, friction, collisions, robot mechanics and real-time factor remain
    exactly as specified by the source. Snapshots use absolute relative-file
    resources so relocation into a run folder does not break terrain meshes.
    """
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if source == destination:
        raise ValueError("Write a run snapshot; do not overwrite the source world")
    tree = ET.parse(source)
    worlds = tree.getroot().findall("world")
    if len(worlds) != 1 or not worlds[0].get("name"):
        raise ValueError("Expected an SDF file containing one named world")
    world = worlds[0]
    for uri in world.iter("uri"):
        value = (uri.text or "").strip()
        if value and "://" not in value and not Path(value).is_absolute():
            uri.text = str((source.parent / value).resolve())
    if cameras:
        sensors = _plugin(world, "gz-sim-sensors-system", "gz::sim::systems::Sensors")
        if sensors.find("render_engine") is None:
            _child(sensors, "render_engine", "ogre2")
    else:
        # Removing sensors, rather than hiding their previews or unsubscribing,
        # avoids Ogre's server-side render loop, including in source worlds
        # which already contain an overview camera.
        camera_types = {"camera", "depth", "depth_camera", "rgbd_camera", "thermal_camera",
                        "segmentation_camera", "boundingbox_camera", "wideanglecamera"}
        for parent in world.iter():
            for sensor in list(parent.findall("sensor")):
                if sensor.get("type") in camera_types:
                    parent.remove(sensor)
        # Keep this system if another rendering sensor or unresolved include
        # might need it. IMU/contact sensors use their own systems.
        rendering_types = camera_types | {"gpu_lidar", "gpu_ray"}
        if (not any(sensor.get("type") in rendering_types for sensor in world.iter("sensor"))
                and world.find(".//include") is None):
            for plugin in list(world.findall("plugin")):
                if (plugin.get("name") == "gz::sim::systems::Sensors"
                        or plugin.get("filename") == "gz-sim-sensors-system"):
                    world.remove(plugin)
    _plugin(world, "gz-sim-user-commands-system", "gz::sim::systems::UserCommands")
    _plugin(world, "gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster")
    # Reuse an existing image source if this world already supplies it.
    if cameras and not any((topic.text or "").strip() == "/hamr/recording_camera/image"
                           for topic in world.findall(".//sensor/topic")):
        model = _child(world, "model", name="hamr_ui_overview_camera")
        _child(model, "static", "true")
        _child(model, "pose", " ".join(f"{v:.9g}" for v in camera_pose(bounds)))
        link = _child(model, "link", name="camera_link")
        sensor = _child(link, "sensor", name="overview", type="camera")
        _child(sensor, "always_on", "true")
        _child(sensor, "update_rate", "25")
        _child(sensor, "topic", "/hamr/recording_camera/image")
        camera = _child(sensor, "camera")
        _child(camera, "horizontal_fov", "1.05")
        image = _child(camera, "image")
        _child(image, "width", "1280")
        _child(image, "height", "720")
        _child(image, "format", "R8G8B8")
        clip = _child(camera, "clip")
        _child(clip, "near", ".03")
        _child(clip, "far", "1000")
    if log_path is not None:
        log_path = Path(log_path).resolve()
        if (log_path / "state.tlog").exists():
            raise ValueError(f"Refusing to overwrite existing replay: {log_path}")
        log = _plugin(world, "gz-sim-log-system", "gz::sim::systems::LogRecord")
        for tag, value in (("record_path", log_path), ("record_period", ".02"),
                           ("record_resources", "false"), ("compress", "false")):
            element = log.find(tag)
            if element is None:
                element = _child(log, tag)
            element.text = str(value)
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return world.get("name")


def write_gui_config(destination, bounds=None, *, playback=False):
    """A native Gazebo 3D viewport, mouse controls, markers and MP4 button."""
    root = ET.Element("config")
    # Playback has no SDF argument; otherwise Gazebo waits in Quick Start
    # before loading this GUI or starting the recorded world.
    _child(root, "dialog", name="quick_start", show_again="false")
    window = _child(root, "window")
    _child(window, "width", "1120")
    _child(window, "height", "780")
    _child(window, "dialog_on_exit", "false")
    _child(window, "style", material_theme="Dark", material_primary="Grey",
           material_accent="LightGreen")
    scene = _plugin(root, "MinimalScene", "3D View")
    settings = _child(scene, "gz-gui")
    _child(settings, "title", "3D View")
    _child(settings, "property", "false", type="bool", key="showTitleBar")
    _child(settings, "property", "docked", type="string", key="state")
    _child(scene, "engine", "ogre2")
    _child(scene, "scene", "scene")
    _child(scene, "ambient_light", ".6 .6 .6")
    _child(scene, "background_color", ".082 .094 .075")
    _child(scene, "camera_pose", " ".join(f"{v:.9g}" for v in camera_pose(bounds)))
    clip = _child(scene, "camera_clip")
    _child(clip, "near", ".01")
    _child(clip, "far", "1000")
    for filename in ("GzSceneManager", "InteractiveViewControl", "CameraTracking",
                     "MarkerManager", "EntityContextMenuPlugin", "SelectEntities", "Screenshot"):
        plugin = _plugin(root, filename, filename)
        settings = _child(plugin, "gz-gui")
        for key, kind, value in (("state", "string", "floating"),
                                 ("showTitleBar", "bool", "false"),
                                 ("width", "double", "5"), ("height", "double", "5")):
            _child(settings, "property", value, type=kind, key=key)
    # This records the interactive camera itself, complementing sensor MP4s.
    plugin = _plugin(root, "VideoRecorder", "Record viewport")
    recording = _child(plugin, "record_video")
    _child(recording, "use_sim_time", "true")
    settings = _child(plugin, "gz-gui")
    for key, kind, value in (("state", "string", "floating"),
                             ("showTitleBar", "bool", "false"),
                             ("width", "double", "110"), ("height", "double", "50"),
                             ("x", "double", "12"), ("y", "double", "12")):
        _child(settings, "property", value, type=kind, key=key)
    controls = _plugin(root, "WorldControl", "World control")
    settings = _child(controls, "gz-gui")
    for key, kind, value in (("state", "string", "floating"),
                             ("showTitleBar", "bool", "false"),
                             ("resizable", "bool", "false"),
                             ("height", "double", "72"), ("z", "double", "1")):
        _child(settings, "property", value, type=kind, key=key)
    anchors = _child(settings, "anchors", target="3D View")
    _child(anchors, "line", own="left", target="left")
    _child(anchors, "line", own="bottom", target="bottom")
    for tag, value in (("play_pause", "true"), ("step", "true"),
                       ("start_paused", "false"), ("use_event", "true")):
        _child(controls, tag, value)
    if playback:
        scrubber = _plugin(root, "PlaybackScrubber", "Playback timeline")
        settings = _child(scrubber, "gz-gui")
        for key, kind, value in (("state", "string", "floating"),
                                 ("showTitleBar", "bool", "false"),
                                 ("height", "double", "90"),
                                 ("width", "double", "350"), ("z", "double", "1")):
            _child(settings, "property", value, type=kind, key=key)
        anchors = _child(settings, "anchors", target="3D View")
        _child(anchors, "line", own="horizontalCenter", target="horizontalCenter")
        _child(anchors, "line", own="bottom", target="bottom")
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Gazebo's config is an XML fragment with multiple top-level elements.
    ET.indent(root, space="  ")
    destination.write_text('<?xml version="1.0"?>\n' + '\n'.join(
        ET.tostring(child, encoding="unicode") for child in root), encoding="utf-8")
    return destination
