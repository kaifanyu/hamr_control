"""Replay measured caster rotations on the existing CAD model in isolated RViz."""

import importlib.util
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _helper():
    source = Path(__file__).resolve().parents[1] / "scripts"
    installed = Path(get_package_prefix("hamr_bringup")) / "lib/hamr_bringup"
    scripts = source if (source / "caster_replay_model.py").exists() else installed
    if str(scripts) not in sys.path:
        sys.path.insert(0, str(scripts))
    spec = importlib.util.spec_from_file_location("caster_replay_launch_helpers", scripts / "caster_replay_model.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _check_encoder_topic(bag, wheel_id=None, packet_topic=None):
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""),
                    rosbag2_py.ConverterOptions("", ""))
        topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    except (RuntimeError, ValueError) as error:
        raise ValueError(f"Cannot open encoder replay bag {bag}: {error}") from error
    finally:
        del reader
    from caster_encoder_selection import resolve_selection
    wheel_id, packet_topic = resolve_selection(wheel_id, packet_topic, topics)
    if topics.get(packet_topic) != "std_msgs/msg/String":
        raise ValueError(
            f"Bag has no {packet_topic} topic of type std_msgs/msg/String. "
            "Record a new run with caster_encoder_bridge; older bags cannot reconstruct missing ESP measurements."
        )
    return wheel_id, packet_topic


def _start(context):
    helper = _helper()
    bag, rate = helper.validate_playback(
        LaunchConfiguration("bag").perform(context), LaunchConfiguration("rate").perform(context))
    wheel_id, packet_topic = _check_encoder_topic(
        bag, LaunchConfiguration("wheel").perform(context),
        LaunchConfiguration("packet_topic").perform(context))
    enable_rviz = LaunchConfiguration("rviz").perform(context).lower()
    if enable_rviz not in ("true", "false"):
        raise ValueError("rviz must be true or false")
    bringup_share = Path(get_package_share_directory("hamr_bringup"))
    model_share = Path(get_package_share_directory("hamr_ball_caster"))
    mapping_config = Path(LaunchConfiguration("mapping_config").perform(context)).expanduser().resolve()
    if not mapping_config.is_file():
        raise ValueError(f"Mapping configuration does not exist: {mapping_config}")
    model = model_share / "urdf/ball_caster.urdf"
    if not model.is_file():
        raise ValueError(f"Install hamr_ball_caster to provide the existing CAD model: {model}")
    description = model.read_text(encoding="utf-8")
    tf_remaps = [("/tf", "/caster_replay/tf"), ("/tf_static", "/caster_replay/tf_static")]
    actions = [
        LogInfo(msg=(
            f"Kinematic CAD replay of {wheel_id or packet_topic} from {bag} at {rate:g}x. Only recorded encoder packets are played. "
            "Signs/zeros are illustrative until calibrated; no accuracy reference or physics solver is implied."
        )),
        Node(
            package="hamr_bringup", executable="replay_caster_encoder_joints",
            name="replay_caster_encoder_joints", output="screen",
            parameters=[str(mapping_config), {"use_sim_time": True, "wheel_id": wheel_id or ""}],
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace="caster_replay", name="state_publisher", output="screen",
            parameters=[{"robot_description": description, "use_sim_time": True,
                         "frame_prefix": "caster_replay/", "publish_frequency": 30.0}],
            remappings=[("joint_states", "/caster_replay/joint_states"), *tf_remaps],
        ),
    ]
    if enable_rviz == "true":
        actions.append(Node(
            package="rviz2", executable="rviz2", namespace="caster_replay",
            name="rviz", output="screen",
            arguments=["-d", str(bringup_share / "config/caster_encoder_replay.rviz")],
            parameters=[{"use_sim_time": True}], remappings=tf_remaps,
        ))
    # Allow subscriptions/model rendering to initialize before the first sample.
    # No exit handler: keep the final model and playback controls visible at EOF.
    actions.append(ExecuteProcess(
        cmd=["ros2", "bag", "play", bag, "--rate", str(rate), "--clock", "--delay", "2.0",
             "--topics", packet_topic, "--remap",
             f"{packet_topic}:=/caster_replay/packets"],
        output="screen",
    ))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("hamr_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("bag", description="Finalized bag containing caster encoder packets"),
        DeclareLaunchArgument("wheel", default_value="", description="front_left, front_right or legacy; required for bags with multiple wheels"),
        DeclareLaunchArgument("packet_topic", default_value="", description="Optional absolute packet topic; inferred from wheel or unique bag topic"),
        DeclareLaunchArgument("rate", default_value="1.0", description="Finite positive playback speed"),
        DeclareLaunchArgument("rviz", default_value="true", description="Open the CAD replay viewer"),
        DeclareLaunchArgument("mapping_config", default_value=str(share / "config/caster_encoder_replay.yaml"),
                              description="Physical side/sign/zero to CAD mapping YAML"),
        OpaqueFunction(function=_start),
    ])
