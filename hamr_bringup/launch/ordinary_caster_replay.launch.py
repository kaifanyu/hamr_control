"""Kinematic replay of recorded wheel/swivel history; no motor commands."""
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    package = Path(get_package_prefix("hamr_bringup"))
    sys.path.insert(0, str(package / "lib" / "hamr_bringup"))
    from ordinary_caster.model import dual_caster_description
    get = lambda name: LaunchConfiguration(name).perform(context)
    args = [get("input"), "--rate", get("rate"),
            "--max-clock-uncertainty-ms", get("max_clock_uncertainty_ms")]
    if get("wheel"):
        args += ["--wheel", get("wheel")]
    if get("role"):
        args += ["--role", get("role")]
    for name in ("device_local", "allow_uncalibrated"):
        if get(name).lower() == "true":
            args += ["--" + name.replace("_", "-")]
    source = Path(get_package_share_directory("hamr_swivel_caster")) / "urdf" / "swivel_caster.urdf"
    nodes = [Node(package="hamr_bringup", executable="replay_ordinary_caster_joints",
                  arguments=args, output="screen"),
             Node(package="robot_state_publisher", executable="robot_state_publisher",
                  parameters=[{"robot_description": dual_caster_description(source),
                               "use_sim_time": True}],
                  remappings=[("joint_states", "/ordinary_caster/replay/joint_states")])]
    if get("rviz").lower() == "true":
        config = Path(get_package_share_directory("hamr_bringup")) / "config" / "ordinary_caster" / "replay.rviz"
        nodes.append(Node(package="rviz2", executable="rviz2", arguments=["-d", str(config)],
                          parameters=[{"use_sim_time": True}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input", description="Downloaded run directory or packets.jsonl"),
        DeclareLaunchArgument("wheel", default_value=""),
        DeclareLaunchArgument("role", default_value="", description="SWIVEL or WHEEL for one-board device-local replay"),
        DeclareLaunchArgument("device_local", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("allow_uncalibrated", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("max_clock_uncertainty_ms", default_value="10.0"),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=start)])
