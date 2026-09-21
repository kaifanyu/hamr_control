"""Keep default hardware, offline profile and measured calibration consistent."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'hamr_bringup/config'


def read(name):
    return yaml.safe_load((CONFIG / name).read_text())


def test_default_hardware_selects_calibrated_profile():
    tree = ET.parse(ROOT / 'hamr_bringup/launch/hamr_HW.launch.xml')
    args = {x.attrib['name']: x.attrib.get('default') for x in tree.findall('arg')}
    assert args['use_mag'] == 'false'
    assert args['use_orientation'] == 'false'
    assert args['wheel_odom_config'].endswith('/config/wheel_odometry_calibration.yaml')
    definitions = {x.attrib['name']: x.attrib.get('value') for x in tree.findall('let')}
    assert definitions['ekf_config_path'].endswith('/config/ekf_calibrated.yaml')


def test_live_and_replay_use_the_same_selected_inputs_and_variances():
    profile = read('offline_calibrated_replay.yaml')
    ekf = read(profile['base_ekf_config'])['/**']['ros__parameters']
    wheel = read('wheel_odometry_calibration.yaml')['/**']['ros__parameters']
    imu = read('hamr_uros_bridge.yaml')['hamr_uros_bridge']['ros__parameters']
    assert [i for i, value in enumerate(ekf['odom0_config']) if value] == [6, 7, 11]
    assert [i for i, value in enumerate(ekf['imu0_config']) if value] == [11]
    assert profile['ekf']['odom0_config'] == ekf['odom0_config']
    assert profile['ekf']['imu0_config'] == ekf['imu0_config']
    for key, index in [('vx', 0), ('vy', 1), ('wz', 5)]:
        assert profile['covariances']['wheel_twist'][index][index] == pytest.approx(
            wheel['twist_variance_' + key])
    assert profile['covariances']['imu_angular_velocity'][2][2] == pytest.approx(
        imu['imu_gyro_z_variance'])
    assert ekf['odom0'] == '/wheel_odom'
    assert ekf['imu0'] == '/imu/data'
    assert not any(k.startswith(('odom1', 'pose0', 'imu1')) for k in ekf)


def test_covariance_yaml_remains_numeric_when_replay_profile_is_saved(tmp_path):
    import sys
    sys.path.insert(0, str(ROOT / 'rosbags'))
    from offline_replay_config import load_profile
    _, document = load_profile(CONFIG / 'offline_calibrated_replay.yaml')
    saved = tmp_path / 'ekf.yaml'
    saved.write_text(yaml.safe_dump(document))
    parameters = yaml.safe_load(saved.read_text())['/**']['ros__parameters']
    for key in ('process_noise_covariance', 'initial_estimate_covariance'):
        assert len(parameters[key]) == 225
        assert all(type(value) in (int, float) for value in parameters[key])
    original = read('ekf.yaml')['/**']['ros__parameters']
    assert parameters['initial_estimate_covariance'] == original['initial_estimate_covariance']
