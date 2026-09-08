"""Offline tests for the independent-study run-sheet generator."""

import csv
import os
from pathlib import Path
import shlex
import shutil
import stat
import subprocess


PACKAGE_DIR = Path(__file__).resolve().parents[1]
PLANNER = PACKAGE_DIR / 'scripts' / 'create_study_plan'
ATTEMPT_LEVEL_FIELDS = {
    'attempt_id',
    'status',
    'bag_path',
    'bag_closed',
    'trajectory_completed',
    'kinematic_validation',
    'exclusion_reason',
    'video_scorable',
    'video_alignment_delay_s',
    'video_scrub_slip_score_0_3',
    'video_contact_transition_score_0_3',
    'video_chassis_disturbance_score_0_3',
    'video_notes',
}
ATTEMPT_LOG_HEADER = [
    'attempt_id',
    'run_id',
    'planned_id',
    'start_utc',
    'operator_id',
    'session_id',
    'hardware_revision',
    'firmware_revision',
    'workspace_revision',
    'controller_config_sha256',
    'video_id',
    'closed_bag',
    'completed_motion',
    'kinematic_valid',
    'video_scorable',
    'failure_category',
    'reason',
    'bag_path',
]
VIDEO_SCORE_HEADER = [
    'score_id',
    'attempt_id',
    'run_id',
    'planned_id',
    'video_id',
    'scorer_id',
    'rubric_revision',
    'camera_view',
    'frame_rate_hz',
    'visible',
    'sync_offset_s',
    'sync_uncertainty_s',
    'event',
    'start_frame_or_s',
    'end_frame_or_s',
    'nominal_caster_angle_deg',
    'first_caster_motion_s',
    'alignment_complete_s',
    'alignment_definition',
    'alignment_delay_s',
    'scrub_slip_score_0_3',
    'contact_transition_score_0_3',
    'chassis_disturbance_score_0_3',
    'confidence',
    'occluded',
    'intervention',
    'factual_observation',
]


def invoke(tmp_path, *arguments, bag_root=None, path_prefix=None):
    """Run the planner without importing ROS or workspace modules."""
    environment = dict(os.environ)
    environment['HAMR_BAG_ROOT'] = str(bag_root or tmp_path / 'bags')
    if path_prefix is not None:
        environment['PATH'] = f"{path_prefix}:{environment['PATH']}"
    return subprocess.run(
        [str(PLANNER), *arguments],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def read_rows(path):
    """Read the complete generated table."""
    with path.open(encoding='utf-8', newline='') as stream:
        return list(csv.DictReader(stream))


def test_default_is_exact_gate_one_matrix_and_canonical_schema(tmp_path):
    output = tmp_path / 'plan.csv'

    completed = invoke(
        tmp_path,
        '--caster',
        'Traditional',
        '--output',
        str(output),
    )
    rows = read_rows(output)

    assert completed.returncode == 0, completed.stderr
    assert len(rows) == 15
    assert {row['gate_added'] for row in rows} == {'1'}
    assert {row['caster_type'] for row in rows} == {'traditional'}
    assert {row['speed_m_s'] for row in rows} == {'0.15'}
    assert {row['terrain'] for row in rows} == {'flat'}
    assert {row['initial_caster_orientation_deg'] for row in rows} == {'0'}
    assert {row['repetition'] for row in rows} == {'1', '2', '3'}
    assert {row['profile'] for row in rows} == {
        'straight_forward',
        'straight_backward',
        'lateral_left',
        'lateral_right',
        'forward_reverse',
    }
    assert ATTEMPT_LEVEL_FIELDS.isdisjoint(rows[0])
    assert {row['nominal_start_base_yaw_rad'] for row in rows} == {'0.0'}
    assert {row['start_base_yaw_tolerance_rad'] for row in rows} == {'0.15'}
    assert all(len(row['profile_sha256']) == 64 for row in rows)
    assert all(len(row['controller_config_sha256']) == 64 for row in rows)
    assert {
        row['wheel_speed_limit_rad_s'] for row in rows
    } == {'2.93215314335'}
    assert {row['turret_control_enabled'] for row in rows} == {'false'}
    assert {row['study_stack_contract_version'] for row in rows} == {'1'}


def test_gate_counts_ids_commands_and_order_are_deterministic(tmp_path):
    first = tmp_path / 'first.csv'
    second = tmp_path / 'second.csv'
    gate_one = tmp_path / 'gate_one.csv'
    common = (
        '--caster',
        'traditional',
        '--caster',
        'spherical',
        '--through-gate',
        '3',
    )

    first_result = invoke(tmp_path, *common, '--output', str(first))
    second_result = invoke(tmp_path, *common, '--output', str(second))
    gate_one_result = invoke(
        tmp_path,
        '--caster',
        'traditional',
        '--caster',
        'spherical',
        '--output',
        str(gate_one),
    )
    rows = read_rows(first)

    assert first_result.returncode == 0, first_result.stderr
    assert second_result.returncode == 0, second_result.stderr
    assert gate_one_result.returncode == 0, gate_one_result.stderr
    assert first.read_bytes() == second.read_bytes()
    assert len(rows) == 72
    assert sum(row['gate_added'] == '1' for row in rows) == 30
    assert sum(row['gate_added'] == '2' for row in rows) == 18
    assert sum(row['gate_added'] == '3' for row in rows) == 24
    assert len({row['planned_id'] for row in rows}) == len(rows)
    assert all(len(row['planned_id']) <= 128 for row in rows)
    assert {
        row['planned_id'] for row in read_rows(gate_one)
    } == {
        row['planned_id'] for row in rows if row['gate_added'] == '1'
    }

    expected_cells = {
        ('straight_forward', '0.10'),
        ('straight_forward', '0.20'),
        ('forward_reverse', '0.10'),
        ('forward_reverse', '0.20'),
    }
    assert {
        (row['profile'], row['speed_m_s'])
        for row in rows
        if row['gate_added'] == '3'
    } == expected_cells

    for row in rows:
        command = shlex.split(row['run_command'])
        assert command == [
            'ros2',
            'run',
            'hamr_bringup',
            'run_study_reference',
            row['profile'],
            '--speed',
            row['speed_m_s'],
            '--caster',
            row['caster_type'],
            '--terrain',
            row['terrain'],
            '--initial-caster',
            row['initial_caster_orientation_deg'],
            '--repeat',
            row['repetition'],
            '--planned-id',
            row['planned_id'],
            '--expected-profile-sha256',
            row['profile_sha256'],
            '--expected-controller-config-sha256',
            row['controller_config_sha256'],
        ]


def test_caster_block_order_is_counterbalanced(tmp_path):
    output = tmp_path / 'plan.csv'
    completed = invoke(
        tmp_path,
        '--caster',
        'traditional',
        '--caster',
        'spherical',
        '--output',
        str(output),
    )
    rows = read_rows(output)

    assert completed.returncode == 0, completed.stderr
    block_order = []
    for row in rows:
        pair = (row['hardware_block_id'], row['caster_type'])
        if pair not in block_order:
            block_order.append(pair)
    assert [caster for _block, caster in block_order] == [
        'traditional',
        'spherical',
        'spherical',
        'traditional',
        'traditional',
        'spherical',
    ]


def test_default_output_is_under_bag_root_and_planner_never_runs_ros(tmp_path):
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    marker = tmp_path / 'ros_was_run'
    fake_ros = fake_bin / 'ros2'
    fake_ros.write_text(
        f'#!/usr/bin/env bash\ntouch "{marker}"\n', encoding='utf-8'
    )
    fake_ros.chmod(fake_ros.stat().st_mode | stat.S_IXUSR)
    bag_root = tmp_path / 'custom_bags'

    completed = invoke(
        tmp_path,
        '--caster',
        'traditional',
        bag_root=bag_root,
        path_prefix=fake_bin,
    )

    output = bag_root / 'study_results' / 'planned_trials.csv'
    attempt_log = bag_root / 'study_results' / 'attempt_log.csv'
    video_scores = bag_root / 'study_results' / 'video_scores.csv'
    assert completed.returncode == 0, completed.stderr
    assert output.is_file()
    with attempt_log.open(encoding='utf-8', newline='') as stream:
        assert list(csv.reader(stream)) == [ATTEMPT_LOG_HEADER]
    with video_scores.open(encoding='utf-8', newline='') as stream:
        assert list(csv.reader(stream)) == [VIDEO_SCORE_HEADER]
    assert not marker.exists()


def test_existing_plan_is_immutable_even_when_content_would_change(tmp_path):
    output = tmp_path / 'plan.csv'
    first = invoke(
        tmp_path,
        '--caster',
        'traditional',
        '--output',
        str(output),
    )
    original = output.read_bytes()

    refused = invoke(
        tmp_path,
        '--caster',
        'traditional',
        '--terrain',
        'tile',
        '--output',
        str(output),
    )
    assert first.returncode == 0
    assert refused.returncode == 2
    assert 'ledgers are immutable' in refused.stderr
    assert output.read_bytes() == original


def test_invalid_or_duplicate_caster_is_rejected_without_output(tmp_path):
    output = tmp_path / 'plan.csv'
    missing = invoke(tmp_path, '--output', str(output))
    duplicate = invoke(
        tmp_path,
        '--caster',
        'Traditional',
        '--caster',
        'traditional',
        '--output',
        str(output),
    )

    assert missing.returncode == 2
    assert duplicate.returncode == 2
    assert 'must be unique' in duplicate.stderr
    assert not output.exists()


def test_compatible_attempt_log_is_preserved_across_versioned_plans(tmp_path):
    attempt_log = tmp_path / 'attempts.csv'
    first_plan = tmp_path / 'gate1.csv'
    later_plan = tmp_path / 'gate2.csv'
    first = invoke(
        tmp_path,
        '--caster',
        'traditional',
        '--output',
        str(first_plan),
        '--attempt-log',
        str(attempt_log),
    )
    assert first.returncode == 0, first.stderr
    first_planned_id = read_rows(first_plan)[0]['planned_id']
    with attempt_log.open('a', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=ATTEMPT_LOG_HEADER)
        writer.writerow(
            {
                'attempt_id': 'A0001',
                'planned_id': first_planned_id,
                'failure_category': 'sensing',
                'reason': 'no bag was created',
            }
        )
    before = attempt_log.read_bytes()

    later = invoke(
        tmp_path,
        '--caster',
        'traditional',
        '--through-gate',
        '2',
        '--output',
        str(later_plan),
        '--attempt-log',
        str(attempt_log),
    )

    assert later.returncode == 0, later.stderr
    assert attempt_log.read_bytes() == before


def test_profile_revision_changes_only_affected_planned_ids(tmp_path):
    source_profiles = (
        PACKAGE_DIR.parent / 'reference_trajectory' / 'config' / 'trajectories'
    )
    modified_profiles = tmp_path / 'modified_profiles'
    shutil.copytree(source_profiles, modified_profiles)
    changed = modified_profiles / 'straight_forward.yaml'
    changed.write_text(
        changed.read_text(encoding='utf-8') + '\n# test revision\n',
        encoding='utf-8',
    )
    baseline = tmp_path / 'baseline.csv'
    revised = tmp_path / 'revised.csv'

    first = invoke(
        tmp_path,
        '--caster',
        'traditional',
        '--output',
        str(baseline),
        '--profile-dir',
        str(source_profiles),
    )
    second = invoke(
        tmp_path,
        '--caster',
        'traditional',
        '--output',
        str(revised),
        '--profile-dir',
        str(modified_profiles),
    )
    baseline_rows = read_rows(baseline)
    revised_rows = read_rows(revised)

    assert first.returncode == 0, first.stderr
    assert second.returncode == 0, second.stderr
    for before, after in zip(baseline_rows, revised_rows, strict=True):
        if before['profile'] == 'straight_forward':
            assert before['planned_id'] != after['planned_id']
            assert before['profile_sha256'] != after['profile_sha256']
        else:
            assert before['planned_id'] == after['planned_id']
            assert before['profile_sha256'] == after['profile_sha256']


def test_terrain_and_initial_orientation_are_part_of_planned_id(tmp_path):
    baseline = tmp_path / 'flat_init0.csv'
    variant = tmp_path / 'tile_init90.csv'
    first = invoke(
        tmp_path,
        '--caster',
        'traditional',
        '--output',
        str(baseline),
    )
    second = invoke(
        tmp_path,
        '--caster',
        'traditional',
        '--terrain',
        'tile',
        '--initial-caster',
        '90',
        '--output',
        str(variant),
    )

    assert first.returncode == 0, first.stderr
    assert second.returncode == 0, second.stderr
    assert {
        row['planned_id'] for row in read_rows(baseline)
    }.isdisjoint(
        row['planned_id'] for row in read_rows(variant)
    )


def test_controller_revision_changes_ids_and_invalid_limit_is_rejected(tmp_path):
    source_config = PACKAGE_DIR / 'config' / 'hamr_hw_control_params.yaml'
    original_text = source_config.read_text(encoding='utf-8')
    revised_config = tmp_path / 'revised_controller.yaml'
    revised_config.write_text(
        original_text.replace(
            'wheel_speed_limit_rad_s: 2.93215314335',
            'wheel_speed_limit_rad_s: 2.5',
        ),
        encoding='utf-8',
    )
    invalid_config = tmp_path / 'invalid_controller.yaml'
    invalid_config.write_text(
        original_text.replace(
            'wheel_speed_limit_rad_s: 2.93215314335',
            'wheel_speed_limit_rad_s: -1.0',
        ),
        encoding='utf-8',
    )
    baseline = tmp_path / 'baseline_config.csv'
    revised = tmp_path / 'revised_config.csv'
    invalid = tmp_path / 'invalid_config.csv'

    first = invoke(
        tmp_path,
        '--caster',
        'traditional',
        '--output',
        str(baseline),
        '--controller-config',
        str(source_config),
    )
    second = invoke(
        tmp_path,
        '--caster',
        'traditional',
        '--output',
        str(revised),
        '--controller-config',
        str(revised_config),
    )
    refused = invoke(
        tmp_path,
        '--caster',
        'traditional',
        '--output',
        str(invalid),
        '--controller-config',
        str(invalid_config),
    )
    baseline_rows = read_rows(baseline)
    revised_rows = read_rows(revised)

    assert first.returncode == 0, first.stderr
    assert second.returncode == 0, second.stderr
    assert refused.returncode == 2
    assert 'must be a positive number' in refused.stderr
    assert not invalid.exists()
    assert {row['wheel_speed_limit_rad_s'] for row in revised_rows} == {'2.5'}
    assert all(
        before['controller_config_sha256']
        != after['controller_config_sha256']
        and before['planned_id'] != after['planned_id']
        for before, after in zip(baseline_rows, revised_rows, strict=True)
    )
