#!/usr/bin/env python3
"""
rtabmap_sim.launch.py
=====================
Phase 0 visual SLAM in sim (MAPPING mode).

Includes slam_sim.launch.py (Gazebo + COMPA + D455 + bridge), but with its RViz and
its static map->odom DISABLED, then adds the RTAB-Map stack:

  rgbd_odometry  -> visual odometry, publishes odom -> base_link
  rtabmap        -> SLAM graph + loop closure, publishes map -> odom, builds the .db
  rtabmap_viz    -> live SLAM visualization (features, loop closures, map)

All three read config/rtabmap.yaml and the /d455/... topics from the bridge.

Run it, then drive the robot around the room to build a map:
    ros2 launch compa_slam rtabmap_sim.launch.py
    # in another terminal:
    ros2 topic pub /left_wheel/cmd_vel  std_msgs/msg/Float64 "{data: 3.0}"
    ros2 topic pub /right_wheel/cmd_vel std_msgs/msg/Float64 "{data: 3.0}"

The map is saved to maps/compa_sim.db on shutdown. This launch uses MAPPING mode
(--delete_db_on_start = start a fresh map each run). Localization mode against a saved
.db is a separate launch (added later).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("compa_slam")
    params = os.path.join(pkg, "config", "rtabmap.yaml")
    slam_sim = os.path.join(pkg, "launch", "slam_sim.launch.py")
    default_db = os.path.join(pkg, "maps", "compa_sim.db")

    database_path = LaunchConfiguration("database_path")
    use_rtabmap_viz = LaunchConfiguration("use_rtabmap_viz")

    declare = [
        DeclareLaunchArgument("database_path", default_value=default_db,
                              description="Where to write the RTAB-Map database."),
        DeclareLaunchArgument("use_rtabmap_viz", default_value="true"),
    ]

    # The RTAB-Map nodes all subscribe to these canonical input names; we remap them
    # to the /d455/... topics the bridge publishes. (Same names work on real hardware.)
    remappings = [
        ("rgb/image", "/d455/color/image_raw"),
        ("depth/image", "/d455/depth/image_rect_raw"),
        ("rgb/camera_info", "/d455/color/camera_info"),
        ("imu", "/d455/imu"),
    ]

    # Sim + robot + camera, minus its own RViz and static map->odom (RTAB-Map owns it).
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_sim),
        launch_arguments={
            "use_rviz": "false",
            "publish_map_odom_tf": "false",
        }.items(),
    )

    rgbd_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        output="screen",
        parameters=[params],
        remappings=remappings,
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        output="screen",
        parameters=[params, {"database_path": database_path}],
        remappings=remappings,
        arguments=["--delete_db_on_start"],  # MAPPING mode: fresh map each run
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        output="screen",
        parameters=[params],
        remappings=remappings,
        condition=IfCondition(use_rtabmap_viz),
    )

    return LaunchDescription(declare + [sim, rgbd_odometry, rtabmap, rtabmap_viz])
