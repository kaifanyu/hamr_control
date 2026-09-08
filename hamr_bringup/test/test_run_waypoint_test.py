"""Offline tests for the concise Vicon waypoint operator command."""

import os
from pathlib import Path
import re
import stat
import subprocess


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_DIR / 'scripts' / 'run_waypoint_test'


def make_fake_commands(tmp_path, timestamp='20260820_191530'):
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()

    fake_ros2 = fake_bin / 'ros2'
    fake_ros2.write_text(
        '#!/usr/bin/env bash\n'
        'printf "%s\\n" "$@" > "${CAPTURE_ARGS}"\n'
        'printf "%s\\n" "${HAMR_BAG_NAME}" > "${CAPTURE_BAG_NAME}"\n',
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


def run_wrapper(tmp_path, *args, timestamp='20260820_191530'):
    fake_bin = make_fake_commands(tmp_path, timestamp)
    bag_root = tmp_path / 'bags'
    bag_root.mkdir()
    capture_args = tmp_path / 'args.txt'
    capture_bag_name = tmp_path / 'bag_name.txt'
    environment = {
        **os.environ,
        'PATH': f'{fake_bin}:{os.environ["PATH"]}',
        'HAMR_BAG_ROOT': str(bag_root),
        'HAMR_BAG_NAME': 'stale_name_that_must_not_be_reused',
        'CAPTURE_ARGS': str(capture_args),
        'CAPTURE_BAG_NAME': str(capture_bag_name),
    }
    completed = subprocess.run(
        [str(SCRIPT), *args],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )
    captured_args = (
        capture_args.read_text(encoding='utf-8').splitlines()
        if capture_args.exists()
        else []
    )
    captured_bag_name = (
        capture_bag_name.read_text(encoding='utf-8').strip()
        if capture_bag_name.exists()
        else ''
    )
    return completed, captured_args, captured_bag_name, bag_root


def test_default_command_enables_motion_and_auto_labels_bag(tmp_path):
    completed, args, bag_name, _bag_root = run_wrapper(tmp_path)

    assert completed.returncode == 0, completed.stderr
    assert args == [
        'launch',
        'hamr_bringup',
        'hamr_HW_waypoint_vicon_unprotected.launch.xml',
        'run_foxglove:=false',
        'record_bag:=true',
        'enable_waypoint:=true',
    ]
    assert bag_name == (
        'hamr_waypoint_simple_vicon_noguard_v020_20260820_191530'
    )
    assert 'analyze_waypoint_test' in completed.stdout


def test_optional_launch_arguments_are_forwarded_without_shell_evaluation(
    tmp_path,
):
    sentinel = tmp_path / 'must_not_exist'
    literal = f'w_yaw:=0.45;touch {sentinel}'
    completed, args, bag_name, _bag_root = run_wrapper(
        tmp_path,
        'v_lin:=0.15',
        'run_foxglove:=false',
        literal,
    )

    assert completed.returncode == 0, completed.stderr
    assert args[-3:] == [
        'v_lin:=0.15',
        'run_foxglove:=false',
        literal,
    ]
    assert bag_name.startswith(
        'hamr_waypoint_simple_vicon_noguard_v015_'
    )
    assert not sentinel.exists()


def test_explicit_enable_argument_is_not_duplicated(tmp_path):
    completed, args, _bag_name, _bag_root = run_wrapper(
        tmp_path,
        'enable_waypoint:=false',
    )

    assert completed.returncode == 0, completed.stderr
    assert args.count('enable_waypoint:=false') == 1
    assert 'enable_waypoint:=true' not in args


def test_recording_and_foxglove_defaults_remain_caller_overridable(tmp_path):
    completed, args, _bag_name, _bag_root = run_wrapper(
        tmp_path,
        'record_bag:=false',
        'run_foxglove:=true',
        'required_reference_subscribers:=1',
    )

    assert completed.returncode == 0, completed.stderr
    assert args.count('record_bag:=false') == 1
    assert args.count('run_foxglove:=true') == 1
    assert 'record_bag:=true' not in args
    assert 'run_foxglove:=false' not in args


def test_same_second_existing_bag_gets_deterministic_suffix(tmp_path):
    completed, _args, first_name, bag_root = run_wrapper(tmp_path)
    assert completed.returncode == 0, completed.stderr
    (bag_root / first_name).mkdir()

    # Reuse the same fake-command directory through a fresh pytest temp child.
    second_root = tmp_path / 'second'
    second_root.mkdir()
    fake_bin = make_fake_commands(second_root)
    capture_args = second_root / 'args.txt'
    capture_bag_name = second_root / 'bag_name.txt'
    environment = {
        **os.environ,
        'PATH': f'{fake_bin}:{os.environ["PATH"]}',
        'HAMR_BAG_ROOT': str(bag_root),
        'CAPTURE_ARGS': str(capture_args),
        'CAPTURE_BAG_NAME': str(capture_bag_name),
    }
    second = subprocess.run(
        [str(SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert second.returncode == 0, second.stderr
    assert capture_bag_name.read_text(encoding='utf-8').strip() == (
        f'{first_name}_001'
    )


def test_malformed_speed_fails_before_launch(tmp_path):
    completed, args, bag_name, _bag_root = run_wrapper(
        tmp_path,
        'v_lin:=fast',
    )

    assert completed.returncode == 2
    assert 'v_lin must be a positive decimal' in completed.stderr
    assert args == []
    assert bag_name == ''


def test_speed_label_never_rounds_a_more_precise_command(tmp_path):
    completed, args, bag_name, _bag_root = run_wrapper(
        tmp_path,
        'v_lin:=0.154',
    )

    assert completed.returncode == 2
    assert 'at most two fractional digits' in completed.stderr
    assert args == []
    assert bag_name == ''


def test_zero_speed_fails_before_launch(tmp_path):
    completed, args, bag_name, _bag_root = run_wrapper(
        tmp_path,
        'v_lin:=0.00',
    )

    assert completed.returncode == 2
    assert 'v_lin must be positive' in completed.stderr
    assert args == []
    assert bag_name == ''


def test_script_never_uses_eval_for_launch_arguments():
    source = SCRIPT.read_text(encoding='utf-8')

    assert re.search(r'LAUNCH_ARGS=\("\$@"\)', source)
    assert 'eval ' not in source
