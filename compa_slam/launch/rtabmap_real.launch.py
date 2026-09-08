#!/usr/bin/env python3
"""
rtabmap_real.launch.py
======================
Phase 1: build (or localize against) a map from the REAL D455 — either by replaying a
recorded trajectory bag, or live on hardware.

Default mode = MAPPING from a replayed bag, using the onboard EKF's continuous
odom -> base_link TF as RTAB-Map's odometry and letting RTAB-Map add visual loop
closures from the camera. TF mode avoids covariance-triggered resets seen in
legacy bags recorded with an older, partly unobservable EKF configuration. For
a newly validated bag, use_odom_topic:=true remains available. External EKF
odometry is more robust than visual odometry when camera rate is low or uneven.

It reuses config/rtabmap.yaml unchanged (same canonical /d455/... topic remaps as sim);
only use_sim_time matters: keep it TRUE for bag replay (the bag carries the clock via
`ros2 bag play --clock`), FALSE for live hardware.

Build a map from your bag (one command — it plays the bag for you):
    ros2 launch compa_slam rtabmap_real.launch.py \
        bag:=$HOME/hamster_ws/src/hamr_control/rosbags/loop_lab_01
    # -> writes ~/.ros/compa_real.db by default (pass database_path for another location)

Or drive the replay yourself in a second terminal:
    ros2 launch compa_slam rtabmap_real.launch.py            # waits for data
    ros2 bag play ~/hamster_ws/src/hamr_control/rosbags/loop_lab_01 --clock

Then later, LOCALIZE against the saved map with the onboard EKF (live hardware):
    ros2 launch compa_slam rtabmap_real.launch.py localization:=true use_sim_time:=false visual_odometry:=false

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
    # Runtime databases do not belong in the installed package share (and *.db files
    # are intentionally excluded from installation). Keep the conventional default
    # in ~/.ros and allow callers to pass an explicit source-workspace path.
    default_db = os.path.expanduser("~/.ros/compa_real.db")
    # RTAB-Map won't create a missing parent dir for the database -> ensure it exists.
    os.makedirs(os.path.dirname(default_db), exist_ok=True)

    database_path = LaunchConfiguration("database_path")
    localization = LaunchConfiguration("localization")
    visual_odometry = LaunchConfiguration("visual_odometry")
    odom_topic = LaunchConfiguration("odom_topic")
    use_odom_topic = LaunchConfiguration("use_odom_topic")
    use_rtabmap_viz = LaunchConfiguration("use_rtabmap_viz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    bag = LaunchConfiguration("bag")
    rate = LaunchConfiguration("rate")
    playback_delay = LaunchConfiguration("playback_delay")

    declare = [
        DeclareLaunchArgument("database_path", default_value=default_db,
                              description="Where to read/write the RTAB-Map database."),
        DeclareLaunchArgument("localization", default_value="false",
                              description="true = localize against an existing .db (no growth)."),
        DeclareLaunchArgument("visual_odometry", default_value="false",
                              description="false = use external EKF odometry (TF by default); "
                                          "true = run rgbd_odometry."),
        DeclareLaunchArgument("odom_topic", default_value="/local_HAMR/odom",
                              description="External odometry topic used only when use_odom_topic=true."),
        DeclareLaunchArgument(
            "use_odom_topic",
            default_value="false",
            description="Use nav_msgs/Odometry instead of odom->base_link TF. TF is the "
                        "compatibility default because legacy bags contain pathological "
                        "EKF covariance; validate covariance before enabling topic mode.",
        ),
        DeclareLaunchArgument("use_rtabmap_viz", default_value="false",
                              description="GUI viz — leave off on a headless Pi."),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="TRUE for bag replay (--clock), FALSE for live."),
        DeclareLaunchArgument("bag", default_value="",
                              description="If set, auto-play this bag with --clock."),
        DeclareLaunchArgument("rate", default_value="1.0",
                              description="Bag playback rate (lower if the Pi can't keep up)."),
        DeclareLaunchArgument("playback_delay", default_value="3.0",
                              description="Seconds to let RTAB-Map initialize before bag playback."),
    ]

    sim_time = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    # Canonical SLAM inputs -> the /d455/... topics (same as sim).
    remappings = [
        ("rgb/image", "/d455/color/image_raw"),
        ("depth/image", "/d455/depth/image_rect_raw"),
        ("rgb/camera_info", "/d455/color/camera_info"),
        ("imu", "/d455/imu"),
    ]
    # This remap is active only when use_odom_topic=true. In visual mode that
    # topic is /odom from rgbd_odometry; otherwise it is the selected EKF topic.
    odom_remap = ("odom", PythonExpression(
        ["'/odom' if '", visual_odometry, "' == 'true' else '", odom_topic, "'"]))
    # Some legacy bags used an EKF preset that did not observe one body-velocity
    # state, so both pose and twist covariance eventually crossed CoreWrapper's
    # reset threshold. TF mode preserves their continuous transforms with
    # provisional explicit variances. Topic mode remains available for bags whose
    # covariance has been checked over the complete capture.
    external_odom = {
        "odom_frame_id": PythonExpression(
            ["'' if '", use_odom_topic, "' == 'true' else 'odom'"]
        ),
        "odom_tf_linear_variance": 0.001,
        "odom_tf_angular_variance": 0.01,
    }

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
        parameters=[params, sim_time, external_odom, {
            "database_path": database_path,
        }],
        remappings=remappings + [odom_remap],
        arguments=["--delete_db_on_start"],
        condition=UnlessCondition(localization),
    )

    # LOCALIZATION: load the DB, stop growing it.
    rtabmap_localization = Node(
        package="rtabmap_slam", executable="rtabmap", output="screen",
        parameters=[params, sim_time, external_odom, {
            "database_path": database_path,
            "Mem/IncrementalMemory": "false",
            "Mem/InitWMWithAllNodes": "true",
        }],
        remappings=remappings + [odom_remap],
        condition=IfCondition(localization),
    )

    rtabmap_viz = Node(
        package="rtabmap_viz", executable="rtabmap_viz", output="screen",
        parameters=[params, sim_time, external_odom],
        remappings=remappings + [odom_remap],
        condition=IfCondition(use_rtabmap_viz),
    )

    # Optional convenience: play the bag (only when bag:=<path> is given).
    play_bag = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag, "--clock", "--rate", rate,
             "--delay", playback_delay],
        output="screen",
        condition=IfCondition(PythonExpression(["'", bag, "' != ''"])),
    )

    return LaunchDescription(
        declare + [rgbd_odometry, rtabmap_mapping, rtabmap_localization, rtabmap_viz, play_bag]
    )
