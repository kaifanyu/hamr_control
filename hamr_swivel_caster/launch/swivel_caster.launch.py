"""Launch the complete COMPA vehicle with two passive, ordinary swivel casters."""
import os
import math
from pathlib import Path
import shlex
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start(context):
    share = Path(get_package_share_directory("hamr_swivel_caster"))
    gui = LaunchConfiguration("gui").perform(context).lower()
    if gui not in ("true", "false"):
        raise ValueError("gui must be true or false")
    world = Path(LaunchConfiguration("world").perform(context)).expanduser().resolve()
    initial_yaw = float(LaunchConfiguration("initial_yaw").perform(context))
    if not math.isfinite(initial_yaw):
        raise ValueError("initial_yaw must be finite radians")
    description_file = LaunchConfiguration("robot_description_file").perform(context)
    if description_file:
        description = Path(description_file).expanduser().read_text()
    else:
        description = subprocess.check_output([
            "xacro", str(share / "urdf/compa_swivel_caster.urdf.xacro")], text=True)
    resource_paths = [str(share.parent), str(world.parent)]
    for package in ("hamr_ball_caster",):
        resource_paths.append(str(Path(get_package_share_directory(package)).parent))
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        resource_paths.append(os.environ["GZ_SIM_RESOURCE_PATH"])
    gz_share = Path(get_package_share_directory("ros_gz_sim"))
    gui_args = (f"--gui-config {shlex.quote(str(share/'config/gui.config'))} "
                if gui == "true" else "-s ")
    gz_args = f"-r -v 3 {gui_args}{shlex.quote(str(world))}"
    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", ":".join(resource_paths)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz_share / "launch/gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items()),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description, "use_sim_time": True}], output="screen"),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-world", "swivel_caster_test", "-topic", "/robot_description",
                        "-name", "compa", "-allow_renaming", "false", "-z", "0",
                        "-Y", str(initial_yaw)]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": str(share / "config/bridge.yaml"), "use_sim_time": True}]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("hamr_swivel_caster"))
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("initial_yaw", default_value="0", description="Initial chassis world yaw in radians"),
        DeclareLaunchArgument("world", default_value=str(share / "worlds/swivel_caster_test.sdf")),
        DeclareLaunchArgument("robot_description_file", default_value="",
                              description="Optional expanded URDF snapshot for this run"),
        OpaqueFunction(function=_start),
    ])
