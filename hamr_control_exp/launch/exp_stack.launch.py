"""Experimental control stack selector.

Args:
  controller := pid | lqr | mpc | none   (default pid — the existing,
                unmodified hamr_control PID node)
  traj       := smooth | linear | none   (default smooth; linear = legacy
                waypoint_traj_simple for A/B baselines)
  metrics    := true | false
  simulating := true | false
  world_frame, params_file, pid_params_file

Exactly one controller and one trajectory node are started, so there is a
single publisher on the wheel command topics. controller:=pid runs the
byte-identical legacy controller — only its input trajectory changes.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1")


def launch_setup(context):
    controller = LaunchConfiguration("controller").perform(context)
    traj = LaunchConfiguration("traj").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    pid_params_file = LaunchConfiguration("pid_params_file").perform(context)
    world_frame = LaunchConfiguration("world_frame").perform(context)
    simulating = _bool(context, "simulating")
    sim_override = {"simulating": simulating}

    nodes = []

    if traj == "smooth":
        nodes.append(Node(
            package="hamr_control_exp", executable="traj_gen",
            name="exp_traj_gen", output="screen",
            parameters=[params_file, {"world_frame": world_frame}]))
    elif traj == "linear":
        nodes.append(Node(
            package="reference_trajectory", executable="waypoint_traj_simple",
            output="screen",
            parameters=[{"world_frame": world_frame}]))
    elif traj != "none":
        raise ValueError(f"unknown traj '{traj}' (smooth|linear|none)")

    if controller == "pid":
        # The untouched legacy controller with its own params file.
        nodes.append(Node(
            package="hamr_control", executable="hamr_controller",
            output="screen", parameters=[pid_params_file]))
    elif controller == "lqr":
        nodes.append(Node(
            package="hamr_control_exp", executable="lqr_controller",
            name="exp_lqr_controller", output="screen",
            parameters=[params_file, sim_override]))
    elif controller == "mpc":
        nodes.append(Node(
            package="hamr_control_exp", executable="mpc_controller",
            name="exp_mpc_controller", output="screen",
            parameters=[params_file, sim_override]))
    elif controller != "none":
        raise ValueError(f"unknown controller '{controller}' (pid|lqr|mpc|none)")

    if _bool(context, "metrics"):
        nodes.append(Node(
            package="hamr_control_exp", executable="traj_metrics",
            name="exp_metrics", output="screen",
            parameters=[params_file, sim_override]))

    return nodes


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("hamr_control_exp"),
        "config", "exp_params.yaml")
    default_pid_params = os.path.join(
        get_package_share_directory("hamr_bringup"),
        "config", "hamr_hw_control_params.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="pid"),
        DeclareLaunchArgument("traj", default_value="smooth"),
        DeclareLaunchArgument("metrics", default_value="true"),
        DeclareLaunchArgument("simulating", default_value="false"),
        DeclareLaunchArgument("world_frame", default_value="odom"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("pid_params_file",
                              default_value=default_pid_params),
        OpaqueFunction(function=launch_setup),
    ])
