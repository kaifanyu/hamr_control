"""Record the Vicon RGB-D scan with the operator-specified D455 mount.

Vicon controls the test and is recorded as ground truth only. Wheel/IMU onboard
localization owns odom -> base_link; RGB-D is recorded for RTAB-Map processing.
Motion requires an explicit /continuous_waypoint/start service call.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Combine the existing guarded scan runner and camera-inclusive recorder."""
    slam = get_package_share_directory("compa_slam")
    bringup = get_package_share_directory("hamr_bringup")
    reference = get_package_share_directory("reference_trajectory")

    # Operator-specified base_link -> camera_link, in metres and radians.
    # Physical wheel-forward is base_link +Y. A level, upright camera therefore
    # needs +pi/2 yaw to point camera_link +X in the physical forward direction.
    mount_defaults = {
        "mount_x": "0.0",
        "mount_y": "0.20",
        "mount_z": "0.30",
        "mount_roll": "0.0",
        "mount_pitch": "0.0",
        "mount_yaw": str(math.pi / 2.0),
    }
    arguments = [
        DeclareLaunchArgument(
            "bag_name", default_value="vicon_rgbd_scan_01",
            description="Recording name; existing names receive an unused suffix.",
        ),
        DeclareLaunchArgument(
            "planner_config",
            default_value=os.path.join(reference, "config", "rgbd_vicon_scan.yaml"),
            description="Scan offsets translated to the captured Vicon start; inspect placed bounds.",
        ),
        DeclareLaunchArgument(
            "camera", default_value="true", choices=["true", "false"],
            description="Start the D455; false only if its correct TF and streams already run.",
        ),
    ]
    arguments.extend(
        DeclareLaunchArgument(name, default_value=value)
        for name, value in mount_defaults.items()
    )

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, "launch", "hamr_continuous_HW.launch.py")
        ),
        launch_arguments={
            "planner_config": LaunchConfiguration("planner_config"),
            "record_bag": "false",
            "record_camera": "false",
            "run_foxglove": "false",
            "run_localization": "true",
            # Use the current calibrated wheel/gyro EKF, without Vicon fusion.
            "use_orientation": "false",
            "use_mag": "false",
            # Even the wheel node's auxiliary turret-yaw input stays onboard.
            "wheel_base_odom_topic": "/local_HAMR/odom",
            "controller_odom_timeout_s": "0.20",
        }.items(),
    )
    capture_arguments = {
        "robot": "false",  # The hardware include above owns controller/relay/EKF.
        "camera": LaunchConfiguration("camera"),
        "record": "true",
        "bag_name": LaunchConfiguration("bag_name"),
        "pointcloud": "false",
        **{name: LaunchConfiguration(name) for name in mount_defaults},
    }
    capture = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam, "launch", "record_trajectory.launch.py")
        ),
        launch_arguments=capture_arguments.items(),
    )
    return LaunchDescription(arguments + [
        GroupAction(actions=[hardware], scoped=True),
        GroupAction(actions=[capture], scoped=True),
    ])
