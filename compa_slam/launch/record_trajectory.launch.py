#!/usr/bin/env python3
"""
record_trajectory.launch.py
===========================
Phase 1 (hardware): ONE command to drive + record a trajectory with everything.

Starts (each toggleable):
  robot   -> includes hamr_bringup/hamr_HW.launch.xml with its controller,
             recorder, and Foxglove bridge disabled by default.
             That is the calibrated onboard stack: relay_node (serial->IMU+ticks),
             holonomic_odom_node (/wheel_odom) and the EKF (/local_HAMR/odom + the
             odom->base_link TF). This launch owns the camera-inclusive recording,
             and disabling the controller avoids a second publisher overwriting
             manual wheel commands with zero while mapping.
  camera  -> includes realsense.launch.py: D455 on canonical /d455/... + madgwick + the
             base_link->camera_link static TF.
  record  -> runs scripts/record_compa_slam_bag: camera + onboard local odom + Vicon
             odom + tf into one mcap bag.

Vicon (/HAMR_base/odom) comes from the external mocap system / its bridge — start that
the way you normally do; this launch records it but does not provide it.

Run:
    ros2 launch compa_slam record_trajectory.launch.py
    # then drive with a continuously publishing teleop or command source.
    # Ctrl-C finalizes the bag.

Already running your own robot bringup in another terminal? Add robot:=false:
    ros2 launch compa_slam record_trajectory.launch.py robot:=false

Name the bag:
    ros2 launch compa_slam record_trajectory.launch.py bag_name:=loop_lab_01
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_slam = get_package_share_directory("compa_slam")
    pkg_bringup = get_package_share_directory("hamr_bringup")

    realsense_launch = os.path.join(pkg_slam, "launch", "realsense.launch.py")
    hw_launch = os.path.join(pkg_bringup, "launch", "hamr_HW.launch.xml")

    robot = LaunchConfiguration("robot")
    camera = LaunchConfiguration("camera")
    record = LaunchConfiguration("record")
    bag_name = LaunchConfiguration("bag_name")
    pointcloud = LaunchConfiguration("pointcloud")
    run_controller = LaunchConfiguration("run_controller")
    run_foxglove = LaunchConfiguration("run_foxglove")
    use_orientation = LaunchConfiguration("use_orientation")
    use_mag = LaunchConfiguration("use_mag")
    mount_x = LaunchConfiguration("mount_x")
    mount_y = LaunchConfiguration("mount_y")
    mount_z = LaunchConfiguration("mount_z")
    mount_roll = LaunchConfiguration("mount_roll")
    mount_pitch = LaunchConfiguration("mount_pitch")
    mount_yaw = LaunchConfiguration("mount_yaw")

    declare = [
        DeclareLaunchArgument("robot", default_value="true",
                              description="Include the onboard robot stack "
                                          "(hamr_HW.launch.xml, record_bag:=false)."),
        DeclareLaunchArgument("camera", default_value="true",
                              description="Bring up the D455 (realsense.launch.py)."),
        DeclareLaunchArgument("record", default_value="true",
                              description="Record the trajectory bag."),
        DeclareLaunchArgument("bag_name", default_value="",
                              description="Bag folder name (default: compa_slam_<timestamp>)."),
        DeclareLaunchArgument("pointcloud", default_value="false",
                              description="Enable + record the D455 point cloud (CPU-heavy)."),
        DeclareLaunchArgument(
            "run_controller",
            default_value="false",
            description="Run the closed-loop controller. Leave false for manual mapping; "
                        "otherwise it publishes on the same wheel command topics.",
        ),
        DeclareLaunchArgument(
            "run_foxglove",
            default_value="false",
            description="Run Foxglove on the robot (off by default to preserve camera/recording CPU).",
        ),
        DeclareLaunchArgument(
            "use_orientation",
            default_value="true",
            description="Use the bounded-velocity EKF with relative BNO055 quaternion yaw. "
                        "Keep true for mapping unless another EKF is fully validated.",
        ),
        DeclareLaunchArgument(
            "use_mag",
            default_value="false",
            description="Use the alternate magnetometer EKF when use_orientation is false.",
        ),
        # These are only starting values. Pass the measured fixed transform on
        # every real capture; map geometry and loop-closure validation depend on it.
        DeclareLaunchArgument("mount_x", default_value="0.2"),
        DeclareLaunchArgument("mount_y", default_value="0.0"),
        DeclareLaunchArgument("mount_z", default_value="0.2"),
        DeclareLaunchArgument("mount_roll", default_value="0.0"),
        DeclareLaunchArgument("mount_pitch", default_value="0.349"),
        DeclareLaunchArgument("mount_yaw", default_value="0.0"),
    ]

    robot_stack = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(hw_launch),
        launch_arguments={
            "record_bag": "false",
            "run_controller": run_controller,
            "run_foxglove": run_foxglove,
            "use_orientation": use_orientation,
            "use_mag": use_mag,
        }.items(),
        condition=IfCondition(robot),
    )

    camera_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            "pointcloud": pointcloud,
            "use_sim_time": "false",
            "mount_x": mount_x,
            "mount_y": mount_y,
            "mount_z": mount_z,
            "mount_roll": mount_roll,
            "mount_pitch": mount_pitch,
            "mount_yaw": mount_yaw,
        }.items(),
        condition=IfCondition(camera),
    )

    # Empty bag_name -> the script falls back to compa_slam_<timestamp>.
    recorder = ExecuteProcess(
        cmd=["ros2", "run", "compa_slam", "record_compa_slam_bag", bag_name],
        output="screen",
        condition=IfCondition(record),
    )

    return LaunchDescription(declare + [robot_stack, camera_stack, recorder])
