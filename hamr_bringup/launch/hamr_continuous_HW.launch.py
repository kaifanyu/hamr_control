"""Start the existing hardware stack and an explicitly armed continuous planner.

This launch does not send a trajectory until /continuous_waypoint/start is called.
The hardware controller, serial bridge, localization and calibration stay in the
existing hamr_HW.launch.xml; this file adds only the reference publisher.
"""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def _parameters(path, node_name):
    """Read a single named ROS parameter mapping before starting hardware."""
    with Path(path).open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"Invalid ROS parameter file: {path}")
    entry = document.get(node_name, document.get("/" + node_name))
    if not isinstance(entry, dict) or not isinstance(entry.get("ros__parameters"), dict):
        raise ValueError(f"{path} must contain {node_name}.ros__parameters")
    return entry["ros__parameters"]


def validate_hardware_configuration(planner, controller):
    """Prevent a simulation profile or inconsistent kinematics reaching hardware."""
    if planner.get("use_sim_time", False) or controller.get("use_sim_time", False):
        raise ValueError("Real-car continuous planning requires use_sim_time=false")
    # Do this before returning the included hardware launch: malformed paths or
    # misspelled parameters must fail before the serial bridge is started.
    from reference_trajectory.continuous_execution import build_plan, validate_config
    planner = validate_config({key: value for key, value in planner.items()
                               if key != "use_sim_time"})
    build_plan(planner)
    if controller.get("simulating") is not False or controller.get("mode") != "auto":
        raise ValueError("Hardware controller must have simulating=false and mode=auto")
    if planner["odom_recovery_samples"] < controller["vicon_recovery_samples"]:
        raise ValueError("Planner recovery samples must cover the controller's Vicon recovery latch")
    for planner_name, controller_name in (
        ("wheel_radius_m", "r_wheel"),
        ("half_track_m", "a_wheel"),
        ("base_ahead_of_axle_m", "b_wheel"),
    ):
        try:
            planned = float(planner[planner_name])
            calibrated = float(controller[controller_name])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"Missing or invalid hardware geometry: {planner_name}") from error
        if not (math.isfinite(planned) and planned > 0.0
                and math.isfinite(calibrated) and calibrated > 0.0
                and math.isclose(planned, calibrated, rel_tol=1e-9, abs_tol=1e-9)):
            raise ValueError(
                f"Planner {planner_name}={planned} does not match calibrated "
                f"controller {controller_name}={calibrated}"
            )
    try:
        planned_cap = float(planner["max_wheel_rate_rad_s"])
        hardware_cap = float(controller["wheel_speed_limit_rad_s"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("Missing or invalid hardware wheel speed limits") from error
    if not (math.isfinite(planned_cap) and math.isfinite(hardware_cap)
            and 0.0 < planned_cap <= hardware_cap + 1e-9):
        raise ValueError("Planner wheel speed limit exceeds the hardware controller limit")
    # The included hardware launch is deliberately kept at its existing 120 ms
    # reference and accepted-odometry watchdogs. The publisher must abort first.
    for name in ("odom_timeout_s", "max_publish_gap_s"):
        try:
            timeout = float(planner[name])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"Missing or invalid planner watchdog: {name}") from error
        if not (math.isfinite(timeout) and 0.0 < timeout < 0.12):
            raise ValueError(f"{name} must be positive and below the hardware 0.12 s watchdog")


def _configure(context):
    bringup = Path(get_package_share_directory("hamr_bringup"))
    planner_config = LaunchConfiguration("planner_config").perform(context)
    planner = _parameters(planner_config, "continuous_waypoint")
    controller = _parameters(bringup / "config/hamr_hw_control_params.yaml", "hamr_controller_node")
    validate_hardware_configuration(planner, controller)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    if not odom_topic.startswith("/") or any(char.isspace() for char in odom_topic):
        raise ValueError("odom_topic must be an absolute ROS topic in the Vicon world frame")
    forwarded = {
        name: LaunchConfiguration(name)
        for name in (
            "record_bag", "record_camera", "camera_repository", "camera_config",
            "recording_root", "camera_segment_mib", "run_foxglove", "use_mag",
            "use_orientation", "ekf_config", "wheel_odom_config",
        )
    }
    forwarded.update({
        "run_controller": "true",
        "controller_odom_topic": odom_topic,
        "controller_xy_velocity_source": "odom_twist_world",
        "controller_reference_timeout_s": "0.12",
        "controller_odom_timeout_s": "0.12",
        "controller_vicon_pose_guard_enabled": "true",
        "controller_vicon_min_z_m": str(planner.get("odom_min_z_m", 0.25)),
        "controller_vicon_max_z_m": str(planner.get("odom_max_z_m", 0.40)),
        "controller_vicon_source_stamp_policy": "receipt_monotonic",
    })
    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(str(bringup / "launch/hamr_HW.launch.xml")),
            launch_arguments=forwarded.items(),
        ),
        Node(
            package="reference_trajectory",
            executable="continuous_waypoint",
            name="continuous_waypoint",
            output="screen",
            parameters=[planner_config, {"odom_topic": odom_topic, "use_sim_time": False}],
        ),
    ]


def generate_launch_description():
    """Declare operator settings, then validate before launching any process."""
    bringup = FindPackageShare("hamr_bringup")
    planner = FindPackageShare("reference_trajectory")
    home = EnvironmentVariable("HOME")
    arguments = [
        DeclareLaunchArgument(
            "planner_config",
            default_value=PathJoinSubstitution([planner, "config", "continuous_waypoint_hw.yaml"]),
            description="Calibrated real-car continuous trajectory ROS parameter YAML",
        ),
        DeclareLaunchArgument(
            "odom_topic", default_value="/HAMR_base/odom",
            description="Raw Vicon world odometry used by both planner and hardware controller",
        ),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument(
            "record_camera", default_value="false",
            description="Opt in to the external dual-camera repository and calibrated rig",
        ),
        DeclareLaunchArgument(
            "camera_repository",
            default_value=PathJoinSubstitution([home, "Caster_Vision", "ball_caster_dual_cam"]),
        ),
        DeclareLaunchArgument(
            "camera_config",
            default_value=PathJoinSubstitution([LaunchConfiguration("camera_repository"), "config", "rig.yaml"]),
        ),
        DeclareLaunchArgument(
            "recording_root", default_value=PathJoinSubstitution([home, "hamr_recordings", "cameras"]),
        ),
        DeclareLaunchArgument("camera_segment_mib", default_value="1024"),
        DeclareLaunchArgument("run_foxglove", default_value="false"),
        DeclareLaunchArgument("use_mag", default_value="false"),
        DeclareLaunchArgument("use_orientation", default_value="false"),
        DeclareLaunchArgument(
            "ekf_config", default_value=PathJoinSubstitution([bringup, "config", "ekf_calibrated.yaml"]),
        ),
        DeclareLaunchArgument(
            "wheel_odom_config",
            default_value=PathJoinSubstitution([bringup, "config", "wheel_odometry_calibration.yaml"]),
        ),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_configure)])
