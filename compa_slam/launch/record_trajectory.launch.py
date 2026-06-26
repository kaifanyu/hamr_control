#!/usr/bin/env python3
"""
record_trajectory.launch.py
===========================
Phase 1 (hardware): ONE command to drive + record a trajectory with everything.

Starts (each toggleable):
  robot   -> includes hamr_bringup/hamr_HW.launch.xml with record_bag:=false.
             That is the calibrated onboard stack: relay_node (serial->IMU+ticks),
             holonomic_odom_node (/wheel_odom) and the EKF (/local_HAMR/odom + the
             odom->base_link TF). We pass record_bag:=false so ITS recorder stays off
             and this launch owns the (camera-inclusive) recording.
  camera  -> includes realsense.launch.py: D455 on canonical /d455/... + madgwick + the
             base_link->camera_link static TF.
  record  -> runs scripts/record_compa_slam_bag: camera + onboard local odom + Vicon
             odom + tf into one mcap bag.

Vicon (/HAMR_base/odom) comes from the external mocap system / its bridge — start that
the way you normally do; this launch records it but does not provide it.

Run:
    ros2 launch compa_slam record_trajectory.launch.py
    # then drive the robot (raw wheel cmds), e.g.:
    ros2 topic pub /left_wheel/cmd_vel  std_msgs/msg/Float64 "{data: 3.0}"
    ros2 topic pub /right_wheel/cmd_vel std_msgs/msg/Float64 "{data: 3.0}"
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
from launch_ros.actions import Node  # noqa: F401  (kept for easy local additions)


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
    ]

    robot_stack = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(hw_launch),
        launch_arguments={"record_bag": "false"}.items(),
        condition=IfCondition(robot),
    )

    camera_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={"pointcloud": pointcloud, "use_sim_time": "false"}.items(),
        condition=IfCondition(camera),
    )

    # Empty bag_name -> the script falls back to compa_slam_<timestamp>.
    recorder = ExecuteProcess(
        cmd=["ros2", "run", "compa_slam", "record_compa_slam_bag", bag_name],
        output="screen",
        condition=IfCondition(record),
    )

    return LaunchDescription(declare + [robot_stack, camera_stack, recorder])
