#!/usr/bin/env python3
"""
rtabmap_real.launch.py
======================
Phase 1: build (or localize against) a map from the REAL D455 — either by replaying a
recorded trajectory bag, or live on hardware.

Default mode = MAPPING from a replayed bag, using the onboard EKF odometry
(/local_HAMR/odom, ~47 Hz, continuous) as RTAB-Map's odometry and letting RTAB-Map add
visual loop closures from the camera. This is far more robust than visual odometry when
the recorded camera frame rate is low or uneven (which it is — recording drops frames
under load on the Pi).

It reuses config/rtabmap.yaml unchanged (same canonical /d455/... topic remaps as sim);
only use_sim_time matters: keep it TRUE for bag replay (the bag carries the clock via
`ros2 bag play --clock`), FALSE for live hardware.

Build a map from your bag (one command — it plays the bag for you):
    ros2 launch compa_slam rtabmap_real.launch.py \
        bag:=$HOME/hamster_ws/src/hamr_control/rosbags/loop_lab_01
    # -> writes maps/compa_real.db on shutdown (auto-stops when the bag ends if you Ctrl-C)

Or drive the replay yourself in a second terminal:
    ros2 launch compa_slam rtabmap_real.launch.py            # waits for data
    ros2 bag play ~/hamster_ws/src/hamr_control/rosbags/loop_lab_01 --clock

Then later, LOCALIZE against the saved map (live on hardware, use_sim_time:=false):
    ros2 launch compa_slam rtabmap_real.launch.py localization:=true use_sim_time:=false visual_odometry:=true

Needs:  ros-jazzy-rtabmap-ros
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("compa_slam")
    params = os.path.join(pkg, "config", "rtabmap.yaml")
    default_db = os.path.join(pkg, "maps", "compa_real.db")
    # rtabmap won't create a missing parent dir for the database -> ensure it exists.
    os.makedirs(os.path.dirname(default_db), exist_ok=True)

    database_path = LaunchConfiguration("database_path")
    localization = LaunchConfiguration("localization")
    visual_odometry = LaunchConfiguration("visual_odometry")
    odom_topic = LaunchConfiguration("odom_topic")
    use_rtabmap_viz = LaunchConfiguration("use_rtabmap_viz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    bag = LaunchConfiguration("bag")
    rate = LaunchConfiguration("rate")

    declare = [
        DeclareLaunchArgument("database_path", default_value=default_db,
                              description="Where to read/write the RTAB-Map database."),
        DeclareLaunchArgument("localization", default_value="false",
                              description="true = localize against an existing .db (no growth)."),
        DeclareLaunchArgument("visual_odometry", default_value="false",
                              description="false = use external odom_topic (robust, bag replay); "
                                          "true = run rgbd_odometry (live hardware only)."),
        DeclareLaunchArgument("odom_topic", default_value="/local_HAMR/odom",
                              description="External odometry topic (the onboard EKF output)."),
        DeclareLaunchArgument("use_rtabmap_viz", default_value="false",
                              description="GUI viz — leave off on a headless Pi."),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="TRUE for bag replay (--clock), FALSE for live."),
        DeclareLaunchArgument("bag", default_value="",
                              description="If set, auto-play this bag with --clock."),
        DeclareLaunchArgument("rate", default_value="1.0",
                              description="Bag playback rate (lower if the Pi can't keep up)."),
    ]

    sim_time = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    # Canonical SLAM inputs -> the /d455/... topics (same as sim).
    remappings = [
        ("rgb/image", "/d455/color/image_raw"),
        ("depth/image", "/d455/depth/image_rect_raw"),
        ("rgb/camera_info", "/d455/color/camera_info"),
        ("imu", "/d455/imu"),
    ]
    # In external-odom mode RTAB-Map reads odometry from odom_topic; in visual mode it
    # reads the /odom that rgbd_odometry publishes.
    odom_remap = ("odom", PythonExpression(
        ["'/odom' if '", visual_odometry, "' == 'true' else '", odom_topic, "'"]))

    # Visual odometry — only for LIVE hardware (the bag already carries odom + odom->base_link TF).
    rgbd_odometry = Node(
        package="rtabmap_odom", executable="rgbd_odometry", output="screen",
        parameters=[params, sim_time],
        remappings=remappings,
        condition=IfCondition(visual_odometry),
    )

    # MAPPING: fresh DB each run.
    rtabmap_mapping = Node(
        package="rtabmap_slam", executable="rtabmap", output="screen",
        parameters=[params, sim_time, {"database_path": database_path}],
        remappings=remappings + [odom_remap],
        arguments=["--delete_db_on_start"],
        condition=UnlessCondition(localization),
    )

    # LOCALIZATION: load the DB, stop growing it.
    rtabmap_localization = Node(
        package="rtabmap_slam", executable="rtabmap", output="screen",
        parameters=[params, sim_time, {
            "database_path": database_path,
            "Mem/IncrementalMemory": "false",
            "Mem/InitWMWithAllNodes": "true",
        }],
        remappings=remappings + [odom_remap],
        condition=IfCondition(localization),
    )

    rtabmap_viz = Node(
        package="rtabmap_viz", executable="rtabmap_viz", output="screen",
        parameters=[params, sim_time],
        remappings=remappings + [odom_remap],
        condition=IfCondition(use_rtabmap_viz),
    )

    # Optional convenience: play the bag (only when bag:=<path> is given).
    play_bag = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag, "--clock", "--rate", rate],
        output="screen",
        condition=IfCondition(PythonExpression(["'", bag, "' != ''"])),
    )

    return LaunchDescription(
        declare + [rgbd_odometry, rtabmap_mapping, rtabmap_localization, rtabmap_viz, play_bag]
    )
