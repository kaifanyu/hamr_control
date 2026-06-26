#!/usr/bin/env python3
"""
realsense.launch.py
===================
Phase 1 (hardware): bring up the real Intel RealSense D455 and present it on the SAME
canonical /d455/... topics the sim publishes, so config/rtabmap.yaml is reused unchanged.

Brings up:
  - realsense2_camera_node   -> RGB + aligned depth + camera_info + raw IMU, remapped to
                                /d455/color/image_raw, /d455/depth/image_rect_raw,
                                /d455/color/camera_info, /d455/imu_raw
                                (+ /d455/depth/color/points when pointcloud:=true)
  - imu_filter_madgwick      -> fuses the raw IMU into an oriented IMU on /d455/imu
                                (the real D455 IMU has no orientation; sim already does)
  - static base_link -> camera_link TF   -> the hardware bringup runs no robot_state_
                                publisher, so RTAB-Map would have no camera extrinsics
                                without this. MEASURE the real mount and set the offsets.

Verified topic rates on the Pi: color ~30 Hz, depth ~30 Hz, imu_raw ~200 Hz.

Requires (apt):  ros-jazzy-realsense2-camera  ros-jazzy-imu-filter-madgwick

Quick check after launching:
    ros2 topic hz /d455/color/image_raw          # ~30
    ros2 topic hz /d455/depth/image_rect_raw     # ~30
    ros2 topic hz /d455/imu                       # ~200 (madgwick output, has orientation)
    ros2 topic echo /d455/color/camera_info --once
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("compa_slam")
    rs_params = os.path.join(pkg, "config", "realsense_d455.yaml")

    pointcloud = LaunchConfiguration("pointcloud")
    use_madgwick = LaunchConfiguration("use_madgwick")
    publish_tf = LaunchConfiguration("publish_tf")
    use_sim_time = LaunchConfiguration("use_sim_time")
    # base_link -> camera_link mount offset (REP-103: x fwd, y left, z up; rpy in rad).
    mount_x = LaunchConfiguration("mount_x")
    mount_y = LaunchConfiguration("mount_y")
    mount_z = LaunchConfiguration("mount_z")
    mount_roll = LaunchConfiguration("mount_roll")
    mount_pitch = LaunchConfiguration("mount_pitch")
    mount_yaw = LaunchConfiguration("mount_yaw")

    declare = [
        DeclareLaunchArgument("pointcloud", default_value="false",
                              description="Publish /d455/depth/color/points (CPU-heavy "
                                          "on the Pi; enable for Phase 2 elevation)."),
        DeclareLaunchArgument("use_madgwick", default_value="true",
                              description="Run imu_filter_madgwick to add orientation -> /d455/imu."),
        DeclareLaunchArgument("publish_tf", default_value="true",
                              description="Publish the static base_link -> camera_link TF."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        # --- MEASURE THESE on the real robot. Defaults mirror the sim mount as a
        #     starting point: 0.2 m forward, 0.2 m up, pitched ~20 deg (0.349 rad) down. ---
        DeclareLaunchArgument("mount_x", default_value="0.2"),
        DeclareLaunchArgument("mount_y", default_value="0.0"),
        DeclareLaunchArgument("mount_z", default_value="0.2"),
        DeclareLaunchArgument("mount_roll", default_value="0.0"),
        DeclareLaunchArgument("mount_pitch", default_value="0.349"),
        DeclareLaunchArgument("mount_yaw", default_value="0.0"),
    ]

    # RealSense driver. Native topics land at /d455/d455_camera/...; absolute remaps
    # rename them to the canonical contract. (Verified working on the Pi.)
    realsense = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="d455",
        name="d455_camera",
        output="screen",
        parameters=[
            rs_params,
            {"pointcloud.enable": ParameterValue(pointcloud, value_type=bool)},
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[
            ("/d455/d455_camera/color/image_raw", "/d455/color/image_raw"),
            ("/d455/d455_camera/color/camera_info", "/d455/color/camera_info"),
            ("/d455/d455_camera/aligned_depth_to_color/image_raw", "/d455/depth/image_rect_raw"),
            ("/d455/d455_camera/depth/color/points", "/d455/depth/color/points"),
            ("/d455/d455_camera/imu", "/d455/imu_raw"),
        ],
    )

    # The raw D455 IMU has no orientation; madgwick produces it on /d455/imu.
    madgwick = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="d455_imu_filter",
        output="screen",
        condition=IfCondition(use_madgwick),
        parameters=[{
            "use_mag": False,          # D455 has no magnetometer
            "world_frame": "enu",
            "publish_tf": False,       # do NOT let it broadcast a TF (avoid frame fights)
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
        remappings=[
            ("imu/data_raw", "/d455/imu_raw"),
            ("imu/data", "/d455/imu"),
        ],
    )

    # base_link -> camera_link. The hardware bringup runs no robot_state_publisher, so
    # without this RTAB-Map has no path from base_link to the camera optical frames
    # (the driver itself publishes camera_link -> camera_*_optical_frame).
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_base_to_d455",
        output="screen",
        condition=IfCondition(publish_tf),
        arguments=[
            "--x", mount_x, "--y", mount_y, "--z", mount_z,
            "--roll", mount_roll, "--pitch", mount_pitch, "--yaw", mount_yaw,
            "--frame-id", "base_link", "--child-frame-id", "camera_link",
        ],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    return LaunchDescription(declare + [realsense, madgwick, static_tf])
