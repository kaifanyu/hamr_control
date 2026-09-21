"""Launch the CAD caster rig or the separate COMPA caster variant."""

import math
import os
from pathlib import Path
import shlex
import shutil
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def overview_description(xml_text):
    """Hide dense bearing/fastener visuals; preserve every physics element."""
    hidden = {"skate_bearing.stl", "4668k11_permanently_lubricated_stainless_steel_ball_bearing.stl",
              "5905k22_needle_roller_bearing.stl", "5_16_18screw.stl", "5_16nuts.stl"}
    robot = ET.fromstring(xml_text)
    for link in robot.findall("link"):
        for visual in list(link.findall("visual")):
            mesh = visual.find("geometry/mesh")
            if mesh is not None and Path(mesh.get("filename", "")).name in hidden:
                link.remove(visual)
    return ET.tostring(robot, encoding="unicode")


def _bool(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ("true", "false"):
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def _start(context):
    share = Path(get_package_share_directory("hamr_ball_caster"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    model = LaunchConfiguration("model").perform(context)
    if model not in ("rig", "compa"):
        raise ValueError("model must be rig or compa")
    controller = _bool(context, "controller")
    if controller and model != "compa":
        raise ValueError("controller:=true is available only for model:=compa")
    timestep = float(LaunchConfiguration("max_step_size").perform(context))
    real_time_factor = float(LaunchConfiguration("real_time_factor").perform(context))
    friction = float(LaunchConfiguration("ground_friction").perform(context))
    if not math.isfinite(timestep) or not 0 < timestep <= 0.01:
        raise ValueError("max_step_size must be in (0, 0.01] seconds")
    if not math.isfinite(real_time_factor) or not 0 < real_time_factor <= 10:
        raise ValueError("real_time_factor must be in (0, 10]")
    if not math.isfinite(friction) or not 0 <= friction <= 1:
        raise ValueError("ground_friction must be in [0, 1] for this DART mesh setup")
    carrier_phase = float(LaunchConfiguration("carrier_phase").perform(context))
    caster_mass_scale = float(LaunchConfiguration("caster_mass_scale").perform(context))
    load_mass = float(LaunchConfiguration("load_mass").perform(context))
    if not math.isfinite(carrier_phase):
        raise ValueError("carrier_phase must be finite, in radians")
    if not math.isfinite(caster_mass_scale) or caster_mass_scale <= 0:
        raise ValueError("caster_mass_scale must be finite and positive")
    if not math.isfinite(load_mass) or load_mass <= 0:
        raise ValueError("load_mass must be finite and positive, in kilograms")

    source_world = LaunchConfiguration("world").perform(context)
    source_world = Path(source_world) if source_world else share / "worlds/ball_caster_test.sdf"
    tree = ET.parse(source_world)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("world must name an SDF file with a world element")
    physics = world.find("physics")
    if physics is None:
        raise ValueError("world must include physics/max_step_size")
    step = physics.find("max_step_size")
    if step is None:
        step = ET.SubElement(physics, "max_step_size")
    step.text = str(timestep)
    rate = physics.find("real_time_factor")
    if rate is None:
        rate = ET.SubElement(physics, "real_time_factor")
    rate.text = str(real_time_factor)
    ground = world.find("./model[@name='ground_plane']/link/collision/surface/friction/ode")
    if ground is None:
        raise ValueError("world must contain ground_plane with an ODE friction surface")
    for name in ("mu", "mu2"):
        element = ground.find(name)
        if element is None:
            element = ET.SubElement(ground, name)
        element.text = str(friction)

    temp_dir = tempfile.mkdtemp(prefix="hamr_ball_caster_")
    world_path = Path(temp_dir) / "world.sdf"
    tree.write(world_path, encoding="utf-8", xml_declaration=True)

    def cleanup(_context):
        shutil.rmtree(temp_dir, ignore_errors=True)
        return []

    description_file = share / "urdf" / (
        "caster_test_rig.urdf.xacro" if model == "rig" else "compa_ball_caster.urdf.xacro"
    )
    description_command = Command([FindExecutable(name="xacro"), " ", str(description_file),
                 " carrier_phase:=", str(carrier_phase),
                 " caster_mass_scale:=", str(caster_mass_scale),
                 " load_mass:=", str(load_mass)])
    visual_detail = LaunchConfiguration("visual_detail").perform(context)
    if visual_detail not in ("full", "overview"):
        raise ValueError("visual_detail must be full or overview")
    description = ParameterValue(
        overview_description(description_command.perform(context)) if visual_detail == "overview"
        else description_command, value_type=str)
    gui = _bool(context, "gui")
    gui_config = LaunchConfiguration("gui_config").perform(context)
    gui_config = Path(gui_config) if gui_config else share / 'config' / f'{model}_gui.config'
    gui_args = (f"--gui-config {shlex.quote(str(gui_config))} "
                if gui else "-s ")
    gz_args = f"-r -v 3 {gui_args}{shlex.quote(str(world_path))}"
    resource_paths = [str(share.parent), str(source_world.resolve().parent)]
    for dependency in ("compa_description", "hamr_description"):
        try:
            resource_paths.append(str(Path(get_package_share_directory(dependency)).parent))
        except LookupError:
            pass
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        resource_paths.append(os.environ["GZ_SIM_RESOURCE_PATH"])

    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", ":".join(resource_paths)),
        LogInfo(msg=f"Caster {model}: DART step={timestep}s, ground friction={friction}; {world_path}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ros_gz_share / "launch/gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="ros_gz_sim", executable="create", output="screen",
            arguments=["-world", world.attrib["name"], "-topic", "/robot_description",
                       "-name", "caster_test_rig" if model == "rig" else "compa",
                       "-allow_renaming", "false", "-z", "0"],
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", output="screen",
            parameters=[{"config_file": str(share / f"config/bridge_{model}.yaml"),
                         "use_sim_time": True}],
        ),
    ]
    if _bool(context, "rviz"):
        rviz_config = share / "config/ball_caster.rviz"
        actions.append(Node(
            package="rviz2", executable="rviz2", output="screen",
            arguments=["-d", str(rviz_config)] if rviz_config.exists() else [],
            parameters=[{"use_sim_time": True}],
        ))
    if controller:
        bringup = Path(get_package_share_directory("hamr_bringup"))
        actions.append(Node(
            package="compa_control_py", executable="compa_controller", output="screen",
            parameters=[str(bringup / "config/compa_pid_params.yaml"), {"use_sim_time": True}],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="rig", description="rig or compa"),
        DeclareLaunchArgument("gui", default_value="true", description="Show Gazebo GUI"),
        DeclareLaunchArgument("gui_config", default_value="", description="Optional Gazebo GUI configuration"),
        DeclareLaunchArgument("visual_detail", default_value="full", description="overview hides internal fastener/bearing visuals; physics is identical"),
        DeclareLaunchArgument("rviz", default_value="false", description="Also open RViz"),
        DeclareLaunchArgument("controller", default_value="false", description="Start optional COMPA trajectory controller"),
        DeclareLaunchArgument("world", default_value="", description="Optional source SDF world"),
        DeclareLaunchArgument("max_step_size", default_value="0.001", description="Physics step in seconds; use 0.0005 for convergence comparison"),
        DeclareLaunchArgument("real_time_factor", default_value="1.0", description="Target simulation/wall-time ratio; reduce for smooth camera recording"),
        DeclareLaunchArgument("ground_friction", default_value="0.8", description="Ground Coulomb coefficient, 0 to 1"),
        DeclareLaunchArgument("carrier_phase", default_value="0.0", description="Initial carrier rotation from the saved CAD pose, radians"),
        DeclareLaunchArgument("caster_mass_scale", default_value="1.0", description="Positive multiplier for caster masses and inertias"),
        DeclareLaunchArgument("load_mass", default_value="2.0", description="Vertical fixture load mass in kilograms; rig model only"),
        OpaqueFunction(function=_start),
    ])
