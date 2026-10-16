"""Open a shared terrain world with either passive-caster vehicle or no robot.

The selected world is passed directly to Gazebo. In particular, this launcher
does not replace terrain-specific friction, collision, or physics settings.
"""
import importlib.util
import math
import os
from pathlib import Path
import re
import shlex
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def resolve_world(share, terrain, override=""):
    """Resolve a generated terrain or an explicit snapshot without rewriting it."""
    if override:
        world = Path(override).expanduser().resolve()
    else:
        if not re.fullmatch(r"[a-z][a-z0-9_]*", terrain):
            raise ValueError("terrain must be a lowercase world name, such as road_bumps")
        world = share / "worlds" / f"{terrain}.sdf"
    if not world.is_file():
        available = ", ".join(path.stem for path in sorted((share / "worlds").glob("*.sdf")))
        raise ValueError(f"Missing terrain world {world}; available terrains: {available}")
    worlds = ET.parse(world).getroot().findall("world")
    if len(worlds) != 1 or not worlds[0].get("name"):
        raise ValueError("world must be an SDF file containing one named world")
    return world, worlds[0].get("name")


def robot_description(caster, snapshot, visual_detail, closeup_camera=False):
    """Use unchanged caster mechanics; optionally omit dense ball internals."""
    package = "hamr_ball_caster" if caster == "ball" else "hamr_swivel_caster"
    share = Path(get_package_share_directory(package))
    if snapshot:
        description = Path(snapshot).expanduser().read_text()
    else:
        filename = ("compa_ball_caster_recording.urdf.xacro" if caster == "ball"
                    else "compa_swivel_caster.urdf.xacro")
        description = subprocess.check_output([
            "xacro", str(share / "urdf" / filename),
            f"enable_closeup_camera:={str(closeup_camera).lower()}"], text=True)
    if ET.fromstring(description).tag != "robot":
        raise ValueError("robot_description_file must contain an expanded URDF robot")
    if caster == "ball" and visual_detail == "overview":
        # Reuse the already tested visual-only filter. It retains every
        # collision, inertia, joint, sensor and actuator element.
        source = share / "launch/ball_caster.launch.py"
        spec = importlib.util.spec_from_file_location("_terrain_ball_launch", source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.overview_description(description)
    bridge = share / ("config/bridge_compa.yaml" if caster == "ball" else "config/bridge.yaml")
    return description, bridge


def _start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory("hamr_terrain"))
    caster, gui, detail = value("caster"), value("gui").lower(), value("visual_detail")
    if caster not in ("ball", "swivel", "none"):
        raise ValueError("caster must be ball, swivel, or none")
    if gui not in ("true", "false"):
        raise ValueError("gui must be true or false")
    if detail not in ("full", "overview"):
        raise ValueError("visual_detail must be full or overview")
    closeup_camera = value("closeup_camera").lower()
    if closeup_camera not in ("true", "false"):
        raise ValueError("closeup_camera must be true or false")
    headless_rendering = value("headless_rendering").lower()
    if headless_rendering not in ("true", "false"):
        raise ValueError("headless_rendering must be true or false")
    spawn = {name: float(value(name)) for name in ("spawn_x", "spawn_y", "spawn_z", "initial_yaw")}
    if not all(math.isfinite(number) for number in spawn.values()):
        raise ValueError("Spawn coordinates and initial_yaw must be finite")
    world, world_name = resolve_world(share, value("terrain"), value("world"))
    paths = [str(world.parent), str(share), str(share.parent)]
    for package in ("hamr_ball_caster", "hamr_swivel_caster"):
        paths.append(str(Path(get_package_share_directory(package)).parent))
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        paths.append(os.environ["GZ_SIM_RESOURCE_PATH"])
    gui_config = value("gui_config")
    if gui_config:
        gui_config = Path(gui_config).expanduser().resolve()
        if not gui_config.is_file():
            raise ValueError(f"Missing GUI configuration: {gui_config}")
    else:
        default_gui = share / "config/terrain_gui.config"
        gui_config = default_gui if default_gui.is_file() else None
    gui_args = (f"--gui-config {shlex.quote(str(gui_config))} " if gui_config else "") if gui == "true" else ("-s --headless-rendering " if headless_rendering == "true" else "-s ")
    gz_share = Path(get_package_share_directory("ros_gz_sim"))
    actions = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", ":".join(dict.fromkeys(paths))),
        LogInfo(msg=f"Shared terrain {world_name}: caster={caster}; original world physics/friction retained; {world}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz_share / "launch/gz_sim.launch.py")),
            launch_arguments={"gz_args": f"-r -v 3 {gui_args}{shlex.quote(str(world))}",
                              "on_exit_shutdown": "true"}.items()),
    ]
    if caster == "none":
        actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]))
        return actions
    description, bridge = robot_description(caster, value("robot_description_file"), detail,
                                            closeup_camera == "true")
    actions += [
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": ParameterValue(description, value_type=str), "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-world", world_name, "-topic", "/robot_description", "-name", "compa",
                        "-allow_renaming", "false", "-x", str(spawn["spawn_x"]),
                        "-y", str(spawn["spawn_y"]), "-z", str(spawn["spawn_z"]),
                        "-Y", str(spawn["initial_yaw"])]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": str(bridge), "use_sim_time": True}]),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("terrain", default_value="road_bumps", description="Installed terrain world filename without .sdf"),
        DeclareLaunchArgument("caster", default_value="ball", description="ball, swivel, or none"),
        DeclareLaunchArgument("gui", default_value="true", description="Show Gazebo GUI"),
        DeclareLaunchArgument("headless_rendering", default_value="true", description="Use EGL without a display; false permits display-backed GPU rendering with gui:=false"),
        DeclareLaunchArgument("spawn_x", default_value="0", description="Initial root world X in meters"),
        DeclareLaunchArgument("spawn_y", default_value="-2.8", description="Initial root world Y on the flat approach"),
        DeclareLaunchArgument("spawn_z", default_value="0", description="Initial root world Z in meters"),
        DeclareLaunchArgument("initial_yaw", default_value=str(math.pi/2), description="Initial chassis world yaw in radians; default faces +Y"),
        DeclareLaunchArgument("visual_detail", default_value="overview", description="Ball overview omits internal bearing/fastener visuals only; full retains them"),
        DeclareLaunchArgument("closeup_camera", default_value="false", description="Add the body-mounted caster camera during Xacro expansion; an explicit URDF snapshot is used as supplied"),
        DeclareLaunchArgument("world", default_value="", description="Optional source SDF snapshot; its physics and friction are preserved"),
        DeclareLaunchArgument("robot_description_file", default_value="", description="Optional expanded URDF snapshot"),
        DeclareLaunchArgument("gui_config", default_value="", description="Optional GUI configuration; defaults to the terrain package configuration when available"),
        OpaqueFunction(function=_start),
    ])
