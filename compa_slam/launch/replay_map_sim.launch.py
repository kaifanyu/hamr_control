#!/usr/bin/env python3
"""
replay_map_sim.launch.py
========================
Run a RECORDED real-world map as a Gazebo simulation.

This is the "drive the recorded terrain in sim" entry point. It pairs with
scripts/map_to_sim.py, which converts a recorded map (RTAB-Map cloud / .db, or a DEM)
into three artifacts in maps/<name>.*:
    <name>_heightmap.png   the 2.5D terrain (DEM)
    <name>.yaml            extent / height-scale / resolution metadata
    <name>.sdf             a Gazebo world that loads the heightmap

What this launch brings up:
  - Gazebo with maps/<name>.sdf (the reconstructed terrain)
  - the COMPA robot + D455 (urdf/compa_slam.urdf.xacro), spawned above the terrain
  - robot_state_publisher + the ros_gz bridge (control/state + D455)
  - a static map->odom (sim uses Gazebo ground-truth /compa/odom; map==odom==world)
  - cost_map_publisher (from hamr_control_cpp) pointed at the SAME heightmap, so it
    publishes /elevation_map (grid_map) + /costmap that or_planner consumes -- the
    planner therefore sees EXACTLY the terrain the physics engine simulates.
  - optionally or_planner (run_planner:=true): send a /goal_pose in RViz to plan on it.

The terrain extent/height come from the sidecar YAML, so Gazebo and the planner stay
in sync automatically. See docs/HANDOFF.md "Sim replay" for the full pipeline + the
RTAB-Map vs. elevation/cupy explanation.

Run:
    ros2 launch compa_slam replay_map_sim.launch.py map:=compa_real
    ros2 launch compa_slam replay_map_sim.launch.py map:=compa_real run_planner:=true

If you generated the map AFTER your last build, either rebuild (instant with
--symlink-install) so <share>/compa_slam/maps sees it, or pass the source dir:
    ros2 launch compa_slam replay_map_sim.launch.py map:=compa_real \\
        maps_dir:=$HOME/ws/compa_slam/maps
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context, *args, **kwargs):
    pkg_compa_slam = get_package_share_directory("compa_slam")
    pkg_compa_desc = get_package_share_directory("compa_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    map_name = LaunchConfiguration("map").perform(context)
    maps_dir = LaunchConfiguration("maps_dir").perform(context)
    if not maps_dir:
        maps_dir = os.path.join(pkg_compa_slam, "maps")

    yaml_path = os.path.join(maps_dir, f"{map_name}.yaml")
    world_path = os.path.join(maps_dir, f"{map_name}.sdf")
    if not os.path.exists(yaml_path):
        raise RuntimeError(
            f"Map metadata not found: {yaml_path}\n"
            f"Generate it first:  ros2 run compa_slam map_to_sim.py "
            f"--cloud <cloud>.ply --name {map_name}\n"
            f"or point maps_dir:= at the directory that holds {map_name}.yaml.")
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    heightmap = meta["heightmap"]
    width_m = float(meta["map_width_m"])
    length_m = float(meta["map_length_m"])
    height_m = float(meta["map_height_m"])
    z_max = float(meta.get("z_max", height_m))

    # Spawn above the highest terrain point unless the user overrode z.
    z_arg = LaunchConfiguration("z").perform(context)
    spawn_z = str(z_max + 0.5) if z_arg == "auto" else z_arg

    xacro_file = os.path.join(pkg_compa_slam, "urdf", "compa_slam.urdf.xacro")
    bridge_config = os.path.join(pkg_compa_slam, "config", "gazebo_bridge_slam.yaml")

    # Resolve package:// meshes AND file://compa_slam/maps/... AND file://hamr_bringup/...
    # by putting the <share> root (the dir that holds all those package folders) on the
    # gz resource path. Also add maps_dir so a source-tree heightmap URI still resolves.
    set_share_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.dirname(pkg_compa_desc))
    set_maps_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.dirname(maps_dir))

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": [world_path, " -r"]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "compa",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", spawn_z,
            "-Y", LaunchConfiguration("yaw"),
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
    )

    # Sim ground truth: identity map->odom (Gazebo /compa/odom is already world-frame).
    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}],
    )

    # Publish /elevation_map (grid_map) + /costmap from the SAME heightmap the world uses.
    # Extent/height come from the sidecar YAML so they match Gazebo exactly.
    cost_map_publisher = Node(
        package="hamr_control_cpp",
        executable="cost_map_publisher",
        name="terrain_map_publisher",
        output="screen",
        parameters=[{
            "image_path": heightmap,
            "map_width_m": width_m,
            "map_length_m": length_m,
            "map_height_m": height_m,
            "frame_id": "map",
            "publish_rate": 1.0,
            "use_sim_time": True,
        }],
    )

    # Optional: the off-road A* planner on the live terrain. Send /goal_pose from RViz.
    or_planner = Node(
        package="hamr_control_cpp",
        executable="or_planner",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("run_planner")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[{"use_sim_time": True}],
    )

    return [
        set_share_path,
        set_maps_path,
        gz_sim,
        robot_state_publisher,
        spawn,
        bridge,
        static_map_to_odom,
        cost_map_publisher,
        or_planner,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="compa_real",
                              description="basename of maps/<map>.{yaml,sdf,_heightmap.png}"),
        DeclareLaunchArgument("maps_dir", default_value="",
                              description="dir holding the map artifacts "
                                          "(default: <share>/compa_slam/maps)"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("run_planner", default_value="false",
                              description="also start or_planner on the live terrain"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="auto",
                              description="'auto' = spawn above the terrain's high point"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        OpaqueFunction(function=_setup),
    ])
