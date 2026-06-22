#!/usr/bin/env python3
"""
slam_sim.launch.py
==================
Phase 0 sim bring-up for the COMPA visual-SLAM pipeline.

Brings up:
  - Gazebo (gz sim) with the textured feature_world.sdf
  - the COMPA robot + D455 (urdf/compa_slam.urdf.xacro), spawned via robot_description
  - robot_state_publisher (publishes the static TF chain incl. base_link -> d455_optical)
  - the ros_gz bridge (control/state + D455 RGBD+IMU, renamed to /d455/...)
  - a static map -> odom TF (placeholder until RTAB-Map provides it)
  - RViz (optional)

RTAB-Map itself is added in the next step (rtabmap_sim.launch.py). Run this first and
confirm the /d455/* topics are alive before wiring SLAM:
    ros2 topic hz /d455/color/image_raw
    ros2 topic hz /d455/depth/image_rect_raw
    ros2 topic echo /d455/color/camera_info --once

Drive the robot with raw wheel commands, e.g.:
    ros2 topic pub /left_wheel/cmd_vel  std_msgs/msg/Float64 "{data: 3.0}"
    ros2 topic pub /right_wheel/cmd_vel std_msgs/msg/Float64 "{data: 3.0}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_compa_slam = get_package_share_directory("compa_slam")
    pkg_compa_desc = get_package_share_directory("compa_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    default_world = os.path.join(pkg_compa_slam, "worlds", "feature_world.sdf")
    xacro_file = os.path.join(pkg_compa_slam, "urdf", "compa_slam.urdf.xacro")
    bridge_config = os.path.join(pkg_compa_slam, "config", "gazebo_bridge_slam.yaml")

    # --- Launch args ---
    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("use_rviz")
    publish_map_odom_tf = LaunchConfiguration("publish_map_odom_tf")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    declare_args = [
        DeclareLaunchArgument("world", default_value=default_world,
                              description="Path to the gz world .sdf"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # Set false when RTAB-Map is running — it owns the map->odom transform.
        DeclareLaunchArgument("publish_map_odom_tf", default_value="true"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        # Spawn a bit above ground and let it settle (matches compa.launch.xml style).
        DeclareLaunchArgument("z", default_value="1.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
    ]

    # Let gz resolve package:// mesh URIs (the COMPA meshes live in compa_description).
    # GZ_SIM_RESOURCE_PATH must contain the .../share dir that holds compa_description/.
    set_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.dirname(pkg_compa_desc)
    )

    # Expand xacro -> URDF string for robot_description.
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]), value_type=str
    )

    # --- Gazebo ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world, " -r"]}.items(),
    )

    # --- Robot state publisher (URDF + TF) ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # --- Spawn the robot from the robot_description topic ---
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "compa",
            "-x", x, "-y", y, "-z", z, "-Y", yaw,
        ],
    )

    # --- ros <-> gz bridge (control/state + D455) ---
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
    )

    # --- Placeholder map -> odom until RTAB-Map publishes it ---
    # Disabled (publish_map_odom_tf:=false) when running rtabmap_sim, since RTAB-Map
    # publishes this transform itself; two publishers would conflict.
    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(publish_map_odom_tf),
    )

    # --- RViz (optional). A tailored slam.rviz config is added with the RTAB-Map step. ---
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        declare_args
        + [
            set_resource_path,
            gz_sim,
            robot_state_publisher,
            spawn,
            bridge,
            static_map_to_odom,
            rviz,
        ]
    )
