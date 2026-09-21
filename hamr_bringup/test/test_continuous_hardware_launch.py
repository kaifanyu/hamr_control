"""Check launch contracts without starting ROS nodes or opening serial ports."""

import copy
import importlib.util
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parent
sys.path.insert(0, str(REPOSITORY / "reference_trajectory"))
SPEC = importlib.util.spec_from_file_location(
    "continuous_hardware_launch", PACKAGE / "launch/hamr_continuous_HW.launch.py"
)
LAUNCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LAUNCH)


@pytest.fixture
def configurations():
    planner = LAUNCH._parameters(
        REPOSITORY / "reference_trajectory/config/continuous_waypoint_hw.yaml",
        "continuous_waypoint",
    )
    controller = LAUNCH._parameters(
        PACKAGE / "config/hamr_hw_control_params.yaml", "hamr_controller_node"
    )
    return planner, controller


def test_real_profile_matches_calibrated_hardware(configurations):
    LAUNCH.validate_hardware_configuration(*configurations)


@pytest.mark.parametrize("key,value", [
    ("wheel_radius_m", 0.1075),
    ("half_track_m", 0.33072),
    ("base_ahead_of_axle_m", 0.27114),
    ("max_wheel_rate_rad_s", 6.0),
    ("max_wheel_rate_rad_s", float("nan")),
    ("use_sim_time", True),
    ("odom_timeout_s", 0.12),
    ("max_publish_gap_s", 0.0),
    ("points_m_flat", [0.0, 0.0, 0.0, 0.0]),
    ("points_m_flat", [0.0, 0.0, 0.0]),
    ("speed_m_s", -0.15),
    ("misspelled_speed", 0.15),
    ("odom_recovery_samples", 9),
])
def test_inconsistent_profile_is_rejected_before_hardware(configurations, key, value):
    planner, controller = copy.deepcopy(configurations)
    planner[key] = value
    with pytest.raises(ValueError):
        LAUNCH.validate_hardware_configuration(planner, controller)


def test_planner_recovery_covers_a_stricter_controller(configurations):
    planner, controller = copy.deepcopy(configurations)
    controller["vicon_recovery_samples"] = 12
    with pytest.raises(ValueError, match="controller's Vicon recovery latch"):
        LAUNCH.validate_hardware_configuration(planner, controller)


def test_wrapper_adds_one_reference_node_and_reuses_one_controller(monkeypatch):
    monkeypatch.setattr(
        LAUNCH, "get_package_share_directory", lambda package: str(REPOSITORY / package)
    )
    context = LaunchContext()
    context.launch_configurations.update({
        "planner_config": str(REPOSITORY / "reference_trajectory/config/continuous_waypoint_hw.yaml"),
        "odom_topic": "/renamed_vicon/odom",
        "record_bag": "true", "record_camera": "false", "run_foxglove": "false",
        "camera_repository": "/path with spaces/camera",
        "camera_config": "/path with spaces/camera/rig.yaml",
        "recording_root": "/path with spaces/recordings",
        "camera_segment_mib": "1024", "use_mag": "false", "use_orientation": "false",
        "ekf_config": str(PACKAGE / "config/ekf_calibrated.yaml"),
        "wheel_odom_config": str(PACKAGE / "config/wheel_odometry_calibration.yaml"),
    })
    actions = LAUNCH._configure(context)
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    nodes = [action for action in actions if isinstance(action, Node)]
    assert len(includes) == len(nodes) == 1
    assert nodes[0].node_package == "reference_trajectory"
    assert nodes[0].node_executable == "continuous_waypoint"
    parameters = evaluate_parameters(context, nodes[0]._Node__parameters)
    assert parameters[1] == {"odom_topic": "/renamed_vicon/odom", "use_sim_time": False}
    forwarded = dict(includes[0].launch_arguments)
    assert forwarded["run_controller"] == "true"
    assert forwarded["controller_odom_topic"] == parameters[1]["odom_topic"]
    assert forwarded["controller_reference_timeout_s"] == "0.12"
    assert forwarded["controller_odom_timeout_s"] == "0.12"
    assert forwarded["controller_vicon_pose_guard_enabled"] == "true"
    assert forwarded["controller_vicon_source_stamp_policy"] == "receipt_monotonic"
    assert perform_substitutions(context, [forwarded["camera_config"]]) == context.launch_configurations["camera_config"]
    existing = ET.parse(PACKAGE / "launch/hamr_HW.launch.xml").getroot()
    assert sum(node.get("pkg") == "hamr_control" for node in existing.findall("node")) == 1
    assert not any(node.get("pkg") == "reference_trajectory" for node in existing.findall("node"))


def test_launch_has_no_motion_or_clock_override_argument():
    description = LAUNCH.generate_launch_description()
    declared = {
        action.name: action for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert "autostart" not in declared
    assert "use_sim_time" not in declared
    context = LaunchContext()
    for name in ("record_bag", "record_camera", "run_foxglove", "odom_topic"):
        declared[name].execute(context)
    assert context.launch_configurations == {
        "record_bag": "true", "record_camera": "false", "run_foxglove": "false",
        "odom_topic": "/HAMR_base/odom",
    }


def test_bag_keeps_latched_plan_and_status_without_unrelated_topics():
    script = (PACKAGE / "scripts/record_hamr_test_bag").read_text()
    addition = re.search(r'TOPIC_REGEX="\$\{TOPIC_REGEX\}\|([^"\n]*continuous_waypoint[^"\n]*)"', script)
    assert addition is not None
    regex = re.compile(addition.group(1))
    qos = yaml.safe_load((PACKAGE / "config/record_qos.yaml").read_text())
    for suffix in ("metadata", "status", "path"):
        topic = f"/continuous_waypoint/{suffix}"
        assert regex.fullmatch(topic)
        assert qos[topic]["durability"] == "transient_local"
        assert qos[topic]["reliability"] == "reliable"
    assert regex.fullmatch("/continuous_waypoint/debug_image") is None
