"""Offline regressions for the split three-terminal waypoint workflow."""

import os
from pathlib import Path
import stat
import subprocess

import pytest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
STACK_SCRIPT = PACKAGE_DIR / 'scripts' / 'run_waypoint_stack'
REFERENCE_SCRIPT = PACKAGE_DIR / 'scripts' / 'run_waypoint_reference'


def make_fake_commands(tmp_path, timestamp='20260820_200000'):
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    fake_ros2 = fake_bin / 'ros2'
    fake_ros2.write_text(
        '#!/usr/bin/env bash\n'
        'printf "%s\\n" "$@" > "${CAPTURE_ARGS}"\n'
        'printf "%s\\n" "${HAMR_BAG_NAME:-}" > "${CAPTURE_BAG_NAME}"\n',
        encoding='utf-8',
    )
    fake_ros2.chmod(fake_ros2.stat().st_mode | stat.S_IXUSR)
    fake_date = fake_bin / 'date'
    fake_date.write_text(
        f'#!/usr/bin/env bash\nprintf "%s\\n" "{timestamp}"\n',
        encoding='utf-8',
    )
    fake_date.chmod(fake_date.stat().st_mode | stat.S_IXUSR)
    return fake_bin


def run_script(tmp_path, script, *args):
    fake_bin = make_fake_commands(tmp_path)
    capture_args = tmp_path / 'args.txt'
    capture_bag_name = tmp_path / 'bag_name.txt'
    bag_root = tmp_path / 'bags'
    bag_root.mkdir()
    environment = {
        **os.environ,
        'PATH': f'{fake_bin}:{os.environ["PATH"]}',
        'HAMR_BAG_ROOT': str(bag_root),
        'HAMR_BAG_NAME': 'stale_name',
        'CAPTURE_ARGS': str(capture_args),
        'CAPTURE_BAG_NAME': str(capture_bag_name),
    }
    completed = subprocess.run(
        [str(script), *args],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )
    command = (
        capture_args.read_text(encoding='utf-8').splitlines()
        if capture_args.exists()
        else []
    )
    bag_name = (
        capture_bag_name.read_text(encoding='utf-8').strip()
        if capture_bag_name.exists()
        else ''
    )
    return completed, command, bag_name


def test_stack_starts_recorder_and_controller_without_trajectory(tmp_path):
    completed, command, bag_name = run_script(tmp_path, STACK_SCRIPT)

    assert completed.returncode == 0, completed.stderr
    assert command == [
        'launch',
        'hamr_bringup',
        'hamr_HW_waypoint_vicon_unprotected.launch.xml',
        'run_foxglove:=false',
        'record_bag:=true',
        'enable_waypoint:=false',
    ]
    assert bag_name == (
        'hamr_waypoint_simple_vicon_noguard_v020_20260820_200000'
    )


@pytest.mark.parametrize(
    'override',
    (
        'enable_waypoint:=true',
        'record_bag:=false',
        'run_controller:=false',
        'v_lin:=0.15',
        'w_yaw:=0.25',
        'reference_timer_hz:=10.0',
        'required_reference_subscribers:=1',
    ),
)
def test_stack_fixed_contract_rejects_every_override(tmp_path, override):
    completed, command, bag_name = run_script(
        tmp_path,
        STACK_SCRIPT,
        override,
    )

    assert completed.returncode == 2
    assert 'takes no arguments' in completed.stderr
    assert command == []
    assert bag_name == ''


def test_reference_command_is_only_the_fixed_one_shot_publisher(tmp_path):
    completed, command, bag_name = run_script(
        tmp_path,
        REFERENCE_SCRIPT,
    )

    assert completed.returncode == 0, completed.stderr
    assert command == [
        'run',
        'reference_trajectory',
        'waypoint_traj_simple',
        '--ros-args',
        '-p',
        'v_lin:=0.20',
        '-p',
        'w_yaw:=0.50',
        '-p',
        'reference_timer_hz:=50.0',
        '-p',
        'startup_hold_s:=2.0',
        '-p',
        'final_hold_s:=3.0',
        '-p',
        'loop:=false',
        '-p',
        'required_reference_subscribers:=2',
        '-p',
        'subscriber_stable_s:=1.0',
    ]
    # The reference process neither creates nor relabels the recorder.
    assert bag_name == 'stale_name'


def test_reference_profile_cannot_be_silently_overridden(tmp_path):
    completed, command, bag_name = run_script(
        tmp_path,
        REFERENCE_SCRIPT,
        'v_lin:=0.15',
    )

    assert completed.returncode == 2
    assert 'takes no arguments' in completed.stderr
    assert command == []
    assert bag_name == ''
