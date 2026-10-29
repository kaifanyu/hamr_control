"""Verify onboard mapping odometry stays independent of Vicon test control."""

import importlib.util
from pathlib import Path
import re

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parent


def test_scan_uses_onboard_localization_and_keeps_vicon_for_test_control(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'rgbd_scan_launch', PACKAGE / 'launch/rgbd_vicon_scan.launch.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(REPOSITORY / name))
    description = module.generate_launch_description()
    context = LaunchContext()
    includes = []
    nodes = []
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, GroupAction):
            includes.extend(a for a in action.get_sub_entities()
                            if isinstance(a, IncludeLaunchDescription))
        elif isinstance(action, Node):
            nodes.append(action)
    arguments = [dict((key, perform_substitutions(
        context, normalize_to_list_of_substitutions(value)))
        for key, value in include.launch_arguments) for include in includes]
    hardware, capture = arguments
    assert hardware['run_localization'] == 'true'
    assert hardware['use_orientation'] == hardware['use_mag'] == 'false'
    assert hardware['wheel_base_odom_topic'] == '/local_HAMR/odom'
    assert 'caster_encoders' not in hardware
    assert hardware['record_bag'] == 'false'
    assert hardware['controller_odom_timeout_s'] == '0.20'
    cfg = yaml.safe_load(Path(hardware['planner_config']).read_text())
    cfg = cfg['continuous_waypoint']['ros__parameters']
    assert cfg['odom_timeout_s'] == 0.20
    assert cfg['max_publish_gap_s'] == 0.08
    assert cfg['origin_mode'] == 'start_translated_vicon'
    assert 'start_position_tolerance_m' not in cfg
    assert capture['robot'] == 'false' and capture['record'] == 'true'
    assert nodes == []  # No additional Vicon or visual TF publisher.
    assert cfg['odom_topic'] == '/HAMR_base/odom'
    ekf = yaml.safe_load((REPOSITORY / 'hamr_bringup/config/ekf_calibrated.yaml').read_text())
    params = ekf['/**']['ros__parameters']
    assert params['odom0'] == '/wheel_odom'
    assert params['imu0'] == '/imu/data'
    assert params['publish_tf'] is True
    assert params['world_frame'] == 'odom'
    assert not any('HAMR_base' in str(value) for value in params.values())


def test_recorder_captures_latched_scan_metadata_and_state():
    script = (PACKAGE / 'scripts/record_compa_slam_bag').read_text()
    regex = re.search(r'TOPIC_REGEX="\$\{TOPIC_REGEX\}\|([^"\n]+)"', script).group(1)
    qos = yaml.safe_load((PACKAGE / 'config/record_slam_qos.yaml').read_text())
    for suffix in ('metadata', 'status', 'path'):
        topic = f'/continuous_waypoint/{suffix}'
        assert re.fullmatch(regex, topic)
        assert qos[topic]['durability'] == 'transient_local'
