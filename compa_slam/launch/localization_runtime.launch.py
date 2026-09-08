#!/usr/bin/env python3
"""Vicon-free hardware runtime: local EKF control + RTAB-Map correction.

TF ownership is deliberately split:

    RTAB-Map:          map -> odom
    robot_localization: odom -> base_link

The HAMR hardware controller is scoped under a remap from the former Vicon topic
``/HAMR_base/odom`` to the continuous EKF topic ``/local_HAMR/odom``. The same
remap makes the wheel-odometry helper use EKF yaw for turret world orientation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def _validate_database(context):
    path = os.path.expanduser(LaunchConfiguration("database_path").perform(context))
    if not os.path.isfile(path):
        raise RuntimeError(
            f"RTAB-Map database does not exist: {path}. "
            "Pass database_path:=/absolute/path/to/compa_real.db"
        )
    return []


def generate_launch_description():
    pkg_slam = get_package_share_directory("compa_slam")
    pkg_bringup = get_package_share_directory("hamr_bringup")

    hw_launch = os.path.join(pkg_bringup, "launch", "hamr_HW.launch.xml")
    camera_launch = os.path.join(pkg_slam, "launch", "realsense.launch.py")
    rtabmap_launch = os.path.join(pkg_slam, "launch", "rtabmap_real.launch.py")

    database_path = LaunchConfiguration("database_path")
    use_rtabmap_viz = LaunchConfiguration("use_rtabmap_viz")
    use_odom_topic = LaunchConfiguration("use_odom_topic")
    pointcloud = LaunchConfiguration("pointcloud")
    use_orientation = LaunchConfiguration("use_orientation")
    use_mag = LaunchConfiguration("use_mag")
    run_waypoint_simple = LaunchConfiguration("run_waypoint_simple")
    waypoint_v_lin = LaunchConfiguration("waypoint_v_lin")
    waypoint_w_yaw = LaunchConfiguration("waypoint_w_yaw")
    anchor_to_start = LaunchConfiguration("anchor_to_start")
    hold_on_stale = LaunchConfiguration("hold_on_stale_localization")
    localization_timeout = LaunchConfiguration("localization_timeout_s")

    mount_x = LaunchConfiguration("mount_x")
    mount_y = LaunchConfiguration("mount_y")
    mount_z = LaunchConfiguration("mount_z")
    mount_roll = LaunchConfiguration("mount_roll")
    mount_pitch = LaunchConfiguration("mount_pitch")
    mount_yaw = LaunchConfiguration("mount_yaw")

    declare = [
        DeclareLaunchArgument(
            "database_path",
            default_value="~/.ros/compa_real.db",
            description="Existing RTAB-Map database used for localization.",
        ),
        DeclareLaunchArgument(
            "use_rtabmap_viz",
            default_value="false",
            description="Start the RTAB-Map GUI (leave false on the robot computer).",
        ),
        DeclareLaunchArgument(
            "use_odom_topic",
            default_value="true",
            description="Feed validated /local_HAMR/odom covariance to RTAB-Map. Set false "
                        "only for legacy/diagnostic TF compatibility mode.",
        ),
        DeclareLaunchArgument(
            "pointcloud",
            default_value="false",
            description="Enable the D455 point cloud (not needed for localization).",
        ),
        DeclareLaunchArgument(
            "use_orientation",
            default_value="true",
            description="Use relative BNO055 quaternion yaw in the local EKF.",
        ),
        DeclareLaunchArgument(
            "use_mag",
            default_value="false",
            description="Use the alternate magnetometer EKF when orientation is false.",
        ),
        DeclareLaunchArgument(
            "run_waypoint_simple",
            default_value="false",
            description="Start waypoint_traj_simple through map->odom correction. "
            "For the safest test, leave false and start it after localization is verified.",
        ),
        DeclareLaunchArgument(
            "waypoint_v_lin",
            default_value="0.10",
            description="Simple-trajectory linear speed in m/s.",
        ),
        DeclareLaunchArgument(
            "waypoint_w_yaw",
            default_value="0.5",
            description="Simple-trajectory yaw speed in rad/s.",
        ),
        DeclareLaunchArgument(
            "anchor_to_start",
            default_value="true",
            description="Treat simple-trajectory coordinates as offsets anchored at "
            "the robot's first localized pose instead of absolute map coordinates.",
        ),
        DeclareLaunchArgument(
            "localization_timeout_s",
            default_value="2.0",
            description="Stale timeout used only with hold_on_stale_localization.",
        ),
        DeclareLaunchArgument(
            "hold_on_stale_localization",
            default_value="false",
            description="After first localization, hold position when visual updates "
            "are stale instead of continuing with the last map correction.",
        ),
        DeclareLaunchArgument("mount_x", default_value="0.2"),
        DeclareLaunchArgument("mount_y", default_value="0.0"),
        DeclareLaunchArgument("mount_z", default_value="0.2"),
        DeclareLaunchArgument("mount_roll", default_value="0.0"),
        DeclareLaunchArgument("mount_pitch", default_value="0.349"),
        DeclareLaunchArgument("mount_yaw", default_value="0.0"),
    ]

    # Scoped remapping replaces the wheel-odometry helper's former Vicon yaw input
    # with the onboard EKF output.  The controller also receives explicit launch
    # overrides below because the hardware bringup's defaults are intentionally
    # Vicon-specific: Vicon twist is world-frame and its pose guard requires the
    # mocap marker's nonzero height.  /local_HAMR/odom instead has planar z=0 and
    # standard base-frame twist, so the controller must derive world velocity from
    # consecutive EKF poses and must not apply the Vicon geometric guard.
    robot_stack = GroupAction(
        actions=[
            SetRemap(src="/HAMR_base/odom", dst="/local_HAMR/odom"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(hw_launch),
                launch_arguments={
                    "record_bag": "false",
                    "run_foxglove": "false",
                    "use_orientation": use_orientation,
                    "use_mag": use_mag,
                    "controller_odom_topic": "/local_HAMR/odom",
                    "controller_xy_velocity_source": "pose_delta",
                    "controller_vicon_pose_guard_enabled": "false",
                }.items(),
            ),
        ]
    )

    camera_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
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
    )

    localization_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch),
        launch_arguments={
            "database_path": database_path,
            "localization": "true",
            "visual_odometry": "false",
            "odom_topic": "/local_HAMR/odom",
            "use_odom_topic": use_odom_topic,
            "use_rtabmap_viz": use_rtabmap_viz,
            "use_sim_time": "false",
        }.items(),
    )

    reference_adapter = Node(
        package="compa_slam",
        executable="map_reference_to_odom.py",
        name="map_reference_to_odom",
        output="screen",
        parameters=[{
            "input_topic": "/reference_trajectory_map",
            "output_topic": "/reference_trajectory",
            "local_odom_topic": "/local_HAMR/odom",
            "turret_odom_topic": "/HAMR_turret/odom",
            "localization_pose_topic": "/localization_pose",
            "map_frame": "map",
            "odom_frame": "odom",
            "require_localization": True,
            "hold_on_stale_localization": ParameterValue(
                hold_on_stale, value_type=bool
            ),
            "anchor_to_start": ParameterValue(anchor_to_start, value_type=bool),
            "localization_timeout_s": ParameterValue(
                localization_timeout, value_type=float
            ),
        }],
    )

    waypoint_simple = Node(
        package="reference_trajectory",
        executable="waypoint_traj_simple",
        name="waypoint_traj_simple_map_source",
        output="screen",
        condition=IfCondition(run_waypoint_simple),
        # waypoint_traj_simple logs every 100 Hz update at INFO; suppress that on
        # the robot so logging cannot starve the camera/localization pipeline.
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[{
            "v_lin": ParameterValue(waypoint_v_lin, value_type=float),
            "w_yaw": ParameterValue(waypoint_w_yaw, value_type=float),
        }],
        remappings=[
            ("/reference_trajectory", "/reference_trajectory_map"),
            ("/waypoints_path", "/route_waypoints_path"),
            ("/traj_viz", "/route_traj_viz"),
        ],
    )

    return LaunchDescription(
        declare
        + [
            OpaqueFunction(function=_validate_database),
            robot_stack,
            camera_stack,
            localization_stack,
            reference_adapter,
            waypoint_simple,
        ]
    )
