"""Static regressions for hardware Vicon delivery defenses."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONTROLLER_CONFIG = PACKAGE_DIR / 'config' / 'hamr_hw_control_params.yaml'
BRIDGE_CONFIG = PACKAGE_DIR / 'config' / 'hamr_uros_bridge.yaml'
RECORD_QOS_CONFIG = PACKAGE_DIR / 'config' / 'record_qos.yaml'
HARDWARE_LAUNCH = PACKAGE_DIR / 'launch' / 'hamr_HW.launch.xml'
WAYPOINT_VICON_UNPROTECTED_LAUNCH = (
    PACKAGE_DIR
    / 'launch'
    / 'hamr_HW_waypoint_vicon_unprotected.launch.xml'
)
HARDWARE_WHEEL_SPEED_LIMIT_RAD_S = 2.93215314335


def test_hardware_xy_derivative_gains_are_conservative():
    config = yaml.safe_load(CONTROLLER_CONFIG.read_text(encoding='utf-8'))
    params = config['/hamr_controller_node']['ros__parameters']

    assert params['D_x'] == 0.4
    assert params['D_y'] == 0.4


def test_hardware_controller_uses_receipt_monotonic_vicon_clock_policy():
    config = yaml.safe_load(CONTROLLER_CONFIG.read_text(encoding='utf-8'))
    params = config['/hamr_controller_node']['ros__parameters']
    assert params['reference_timeout_s'] == pytest.approx(0.12)
    assert params['odom_timeout_s'] == pytest.approx(0.12)
    assert params['vicon_source_stamp_policy'] == 'receipt_monotonic'
    assert 'vicon_max_source_age_s' not in params
    assert 'vicon_max_future_skew_s' not in params

    launch = ET.parse(HARDWARE_LAUNCH).getroot()
    reference_timeout_args = [
        element
        for element in launch.findall('arg')
        if element.get('name') == 'controller_reference_timeout_s'
    ]
    assert len(reference_timeout_args) == 1
    assert float(reference_timeout_args[0].get('default')) == pytest.approx(
        0.12
    )
    timeout_args = [
        element
        for element in launch.findall('arg')
        if element.get('name') == 'controller_odom_timeout_s'
    ]
    assert len(timeout_args) == 1
    assert float(timeout_args[0].get('default')) == pytest.approx(0.12)
    source_policy_args = [
        element
        for element in launch.findall('arg')
        if element.get('name') == 'controller_vicon_source_stamp_policy'
    ]
    assert len(source_policy_args) == 1
    assert source_policy_args[0].get('default') == 'receipt_monotonic'
    assert not any(
        element.get('name') in (
            'controller_vicon_max_source_age_s',
            'controller_vicon_max_future_skew_s',
        )
        for element in launch.findall('arg')
    )

    controller_nodes = [
        element
        for element in launch.findall('node')
        if element.get('pkg') == 'hamr_control'
        and element.get('exec') == 'hamr_controller'
    ]
    assert len(controller_nodes) == 1
    reference_timeout_overrides = [
        element
        for element in controller_nodes[0].findall('param')
        if element.get('name') == 'reference_timeout_s'
    ]
    assert len(reference_timeout_overrides) == 1
    assert reference_timeout_overrides[0].get('value') == (
        '$(var controller_reference_timeout_s)'
    )
    timeout_overrides = [
        element
        for element in controller_nodes[0].findall('param')
        if element.get('name') == 'odom_timeout_s'
    ]
    assert len(timeout_overrides) == 1
    assert timeout_overrides[0].get('value') == '$(var controller_odom_timeout_s)'
    source_policy_overrides = [
        element
        for element in controller_nodes[0].findall('param')
        if element.get('name') == 'vicon_source_stamp_policy'
    ]
    assert len(source_policy_overrides) == 1
    assert source_policy_overrides[0].get('value') == (
        '$(var controller_vicon_source_stamp_policy)'
    )
    assert not any(
        element.get('name') in (
            'vicon_max_source_age_s',
            'vicon_max_future_skew_s',
        )
        for element in controller_nodes[0].findall('param')
    )


def test_normal_hardware_feedback_defaults_remain_vicon_specific():
    launch = ET.parse(HARDWARE_LAUNCH).getroot()
    defaults = {
        element.get('name'): element.get('default')
        for element in launch.findall('arg')
    }
    assert defaults['controller_odom_topic'] == '/HAMR_base/odom'
    assert defaults['controller_xy_velocity_source'] == 'odom_twist_world'
    assert defaults['controller_vicon_pose_guard_enabled'] == 'true'
    assert float(defaults['controller_vicon_min_z_m']) == pytest.approx(0.25)
    assert float(defaults['controller_vicon_max_z_m']) == pytest.approx(0.40)

    controller = next(
        element
        for element in launch.findall('node')
        if element.get('pkg') == 'hamr_control'
        and element.get('exec') == 'hamr_controller'
    )
    params = {
        element.get('name'): element.get('value')
        for element in controller.findall('param')
        if element.get('name')
    }
    assert params['xy_velocity_source'] == (
        '$(var controller_xy_velocity_source)'
    )
    assert params['vicon_pose_guard_enabled'] == (
        '$(var controller_vicon_pose_guard_enabled)'
    )
    assert params['vicon_min_z_m'] == '$(var controller_vicon_min_z_m)'
    assert params['vicon_max_z_m'] == '$(var controller_vicon_max_z_m)'

    odom_remaps = [
        element
        for element in controller.findall('remap')
        if element.get('from') == 'HAMR_base/odom'
    ]
    assert len(odom_remaps) == 1
    assert odom_remaps[0].get('to') == '$(var controller_odom_topic)'


def test_waypoint_vicon_unprotected_wrapper_is_narrow_and_one_shot():
    launch = ET.parse(WAYPOINT_VICON_UNPROTECTED_LAUNCH).getroot()
    defaults = {
        element.get('name'): element.get('default')
        for element in launch.findall('arg')
    }
    assert defaults['enable_waypoint'] == 'false'
    assert float(defaults['v_lin']) == pytest.approx(0.20)
    assert float(defaults['reference_timer_hz']) == pytest.approx(50.0)
    assert float(defaults['startup_hold_s']) == pytest.approx(2.0)
    assert float(defaults['final_hold_s']) == pytest.approx(3.0)
    assert defaults['loop'] == 'false'
    assert int(defaults['required_reference_subscribers']) == 2
    assert float(defaults['subscriber_stable_s']) == pytest.approx(1.0)

    includes = launch.findall('include')
    assert len(includes) == 1
    include_args = {
        element.get('name'): element.get('value')
        for element in includes[0].findall('arg')
    }
    assert float(
        include_args['controller_reference_timeout_s']
    ) == pytest.approx(0.12)
    assert include_args['controller_odom_topic'] == '/HAMR_base/odom'
    assert float(include_args['controller_odom_timeout_s']) == pytest.approx(
        0.0
    )
    assert include_args['controller_xy_velocity_source'] == (
        'odom_twist_world'
    )
    assert include_args['controller_vicon_pose_guard_enabled'] == 'false'
    assert 'controller_vicon_min_z_m' not in include_args
    assert 'controller_vicon_max_z_m' not in include_args
    assert 'controller_vicon_source_stamp_policy' not in include_args

    # The wrapper does not weaken downstream actuator shutdown behavior.
    bridge_config = yaml.safe_load(
        BRIDGE_CONFIG.read_text(encoding='utf-8')
    )['hamr_uros_bridge']['ros__parameters']
    assert bridge_config['command_timeout_s'] == pytest.approx(0.25)
    assert bridge_config['shutdown_zero_packets'] == 10

    controller_config = yaml.safe_load(
        CONTROLLER_CONFIG.read_text(encoding='utf-8')
    )['/hamr_controller_node']['ros__parameters']
    assert controller_config['reference_timeout_s'] == pytest.approx(0.12)
    assert controller_config['wheel_speed_limit_rad_s'] == pytest.approx(
        HARDWARE_WHEEL_SPEED_LIMIT_RAD_S
    )
    assert controller_config['vicon_max_position_jump_m'] == pytest.approx(
        0.08
    )
    assert controller_config[
        'vicon_max_orientation_jump_rad'
    ] == pytest.approx(0.35)
    assert controller_config['vicon_max_tilt_rad'] == pytest.approx(0.35)
    assert controller_config['vicon_recovery_samples'] == 10

    waypoint_nodes = [
        element
        for element in launch.findall('node')
        if element.get('pkg') == 'reference_trajectory'
        and element.get('exec') == 'waypoint_traj_simple'
    ]
    assert len(waypoint_nodes) == 1
    waypoint = waypoint_nodes[0]
    assert waypoint.get('if') == '$(var enable_waypoint)'
    waypoint_params = {
        element.get('name'): element.get('value')
        for element in waypoint.findall('param')
    }
    assert waypoint_params == {
        'v_lin': '$(var v_lin)',
        'w_yaw': '$(var w_yaw)',
        'reference_timer_hz': '$(var reference_timer_hz)',
        'startup_hold_s': '$(var startup_hold_s)',
        'final_hold_s': '$(var final_hold_s)',
        'loop': '$(var loop)',
        'required_reference_subscribers': (
            '$(var required_reference_subscribers)'
        ),
        'subscriber_stable_s': '$(var subscriber_stable_s)',
    }


def test_ros_feedforward_compensation_is_bounded_but_neutral_by_default():
    config = yaml.safe_load(CONTROLLER_CONFIG.read_text(encoding='utf-8'))
    params = config['/hamr_controller_node']['ros__parameters']

    # The firmware wheel PI repair is the primary correction. Keeping this
    # gain at unity prevents accidentally stacking two inverse-response fixes;
    # the dormant cap supports a deliberate fallback trial if one is needed.
    assert params['xy_feedforward_gain'] == pytest.approx(1.0)
    assert params['xy_feedforward_max_extra_m_s'] == pytest.approx(0.025)


def test_hardware_uses_one_global_firmware_matched_wheel_speed_limit():
    config = yaml.safe_load(CONTROLLER_CONFIG.read_text(encoding='utf-8'))
    params = config['/hamr_controller_node']['ros__parameters']
    configured = params['wheel_speed_limit_rad_s']

    assert configured == pytest.approx(HARDWARE_WHEEL_SPEED_LIMIT_RAD_S)
    assert configured * 60.0 / (2.0 * math.pi) == pytest.approx(28.0)

    # Keep one authoritative hardware value in YAML; a launch argument or
    # per-node override could silently diverge from supervised preflight.
    launch = ET.parse(HARDWARE_LAUNCH).getroot()
    assert not any(
        element.get('name') == 'controller_wheel_speed_limit_rad_s'
        for element in launch.findall('arg')
    )
    controller_nodes = [
        element
        for element in launch.findall('node')
        if element.get('pkg') == 'hamr_control'
        and element.get('exec') == 'hamr_controller'
    ]
    assert len(controller_nodes) == 1
    assert not any(
        element.get('name') == 'wheel_speed_limit_rad_s'
        for element in controller_nodes[0].findall('param')
    )


def test_recorder_vicon_topics_are_best_effort_latest_only():
    qos = yaml.safe_load(RECORD_QOS_CONFIG.read_text(encoding='utf-8'))
    for topic in (
        '/HAMR_base/odom',
        '/HAMR_base/pose',
        '/HAMR_turret/odom',
        '/HAMR_turret/pose',
    ):
        assert qos[topic] == {
            'reliability': 'best_effort',
            'durability': 'volatile',
            'history': 'keep_last',
            'depth': 1,
        }


def test_foxglove_can_be_disabled_but_remains_on_by_default():
    launch = ET.parse(HARDWARE_LAUNCH).getroot()
    run_foxglove_args = [
        element
        for element in launch.findall('arg')
        if element.get('name') == 'run_foxglove'
    ]
    assert len(run_foxglove_args) == 1
    assert run_foxglove_args[0].get('default') == 'true'

    foxglove_nodes = [
        element
        for element in launch.findall('node')
        if element.get('pkg') == 'foxglove_bridge'
    ]
    assert len(foxglove_nodes) == 1
    assert foxglove_nodes[0].get('if') == '$(var run_foxglove)'
