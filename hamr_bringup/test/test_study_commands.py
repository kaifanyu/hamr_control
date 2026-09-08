"""Offline regressions for the three-command independent-study workflow."""

import csv
import hashlib
import os
from pathlib import Path
import re
import shlex
import shutil
import stat
import subprocess

import pytest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
STACK = PACKAGE_DIR / "scripts" / "run_study_stack"
REFERENCE = PACKAGE_DIR / "scripts" / "run_study_reference"
LAUNCH = PACKAGE_DIR / "launch" / "hamr_study_reference.launch.xml"
RECORDER = PACKAGE_DIR / "scripts" / "record_hamr_test_bag"
QOS = PACKAGE_DIR / "config" / "record_qos.yaml"
PLANNER = PACKAGE_DIR / "scripts" / "create_study_plan"
PROFILE_DIRECTORY = (
    PACKAGE_DIR.parent / "reference_trajectory" / "config" / "trajectories"
)
CONTROLLER_CONFIG = PACKAGE_DIR / "config" / "hamr_hw_control_params.yaml"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fake_commands(tmp_path: Path, timestamp: str = "20260824_120000") -> Path:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    ros2 = fake_bin / "ros2"
    ros2.write_text(
        '#!/usr/bin/env bash\nprintf "%s\\n" "$@" > "${CAPTURE_ARGS}"\n'
        'printf "%s\\n" "${HAMR_BAG_NAME:-}" > "${CAPTURE_BAG}"\n',
        encoding="utf-8",
    )
    ros2.chmod(ros2.stat().st_mode | stat.S_IXUSR)
    date = fake_bin / "date"
    date.write_text(
        f'#!/usr/bin/env bash\nprintf "%s\\n" "{timestamp}"\n',
        encoding="utf-8",
    )
    date.chmod(date.stat().st_mode | stat.S_IXUSR)
    return fake_bin


def run_script(tmp_path: Path, script: Path, *arguments: str):
    fake_bin = fake_commands(tmp_path)
    capture_args = tmp_path / "args.txt"
    capture_bag = tmp_path / "bag.txt"
    bag_root = tmp_path / "bags"
    bag_root.mkdir()
    environment = {
        **os.environ,
        "PATH": f'{fake_bin}:{os.environ["PATH"]}',
        "CAPTURE_ARGS": str(capture_args),
        "CAPTURE_BAG": str(capture_bag),
        "HAMR_BAG_ROOT": str(bag_root),
        "HAMR_BAG_NAME": "stale_must_not_survive",
    }
    completed = subprocess.run(
        [str(script), *arguments],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )
    command = capture_args.read_text().splitlines() if capture_args.exists() else []
    bag_name = capture_bag.read_text().strip() if capture_bag.exists() else ""
    return completed, command, bag_name, bag_root


def test_study_stack_is_motionless_and_has_no_recorder(tmp_path):
    completed, command, bag_name, _root = run_script(tmp_path, STACK)

    assert completed.returncode == 0, completed.stderr
    assert command == [
        "launch",
        "hamr_bringup",
        "hamr_HW_waypoint_vicon_unprotected.launch.xml",
        "enable_waypoint:=false",
        "record_bag:=false",
        "run_foxglove:=false",
    ]
    assert bag_name == "stale_must_not_survive"


def test_study_stack_rejects_arguments_before_launch(tmp_path):
    completed, command, _bag, _root = run_script(tmp_path, STACK, "anything")

    assert completed.returncode == 2
    assert command == []


def test_reference_auto_labels_and_passes_one_manifest_source(tmp_path):
    profile_hash = sha256(PROFILE_DIRECTORY / "circle_ccw.yaml")
    controller_hash = sha256(CONTROLLER_CONFIG)
    planned_id = "G2_spherical_flat_circle_ccw_formal_cell_r03"
    completed, command, bag_name, _root = run_script(
        tmp_path,
        REFERENCE,
        "circle_ccw",
        "--speed",
        "0.15",
        "--caster",
        "Spherical",
        "--terrain",
        "flat",
        "--initial-caster",
        "90",
        "--repeat",
        "3",
        "--planned-id",
        planned_id,
        "--expected-profile-sha256",
        profile_hash,
        "--expected-controller-config-sha256",
        controller_hash,
        "--video-id",
        "cam03",
        "--notes",
        "clean trial",
    )

    expected = (
        "hamr_study_circle_ccw_v015_spherical_flat_"
        "init090_r03_20260824_120000"
    )
    assert completed.returncode == 0, completed.stderr
    assert bag_name == expected
    assert command[:4] == [
        "launch",
        "hamr_bringup",
        "hamr_study_reference.launch.xml",
        f"bag_name:={expected}",
    ]
    assert f"run_id:={expected}" in command
    assert f"planned_id:={planned_id}" in command
    assert "profile:=circle_ccw" in command
    assert f"profile_sha256:={profile_hash}" in command
    assert f"controller_config_sha256:={controller_hash}" in command
    assert "wheel_speed_limit_rad_s:=2.93215314335" in command
    assert "turret_control_enabled:=false" in command
    assert "nominal_start_base_yaw_rad:=0.0" in command
    assert "start_base_yaw_tolerance_rad:=0.15" in command
    assert "study_stack_contract_version:=1" in command
    assert "controller_odom_timeout_s:=0.0" in command
    assert "vicon_pose_guard_enabled:=false" in command
    assert "reference_timeout_s:=0.12" in command
    assert "xy_velocity_source:=odom_twist_world" in command
    assert "speed_m_s:=0.15" in command
    assert "caster_type:=spherical" in command
    assert "initial_caster_orientation_deg:=90" in command
    assert "repetition:=3" in command
    assert "notes:=clean trial" in command


@pytest.mark.parametrize(
    "profile",
    [
        "straight_forward",
        "straight_backward",
        "lateral_left",
        "lateral_right",
        "forward_reverse",
    ],
)
def test_reference_accepts_baseline_profiles_before_recording(
    tmp_path, profile
):
    completed, command, bag_name, _root = run_script(
        tmp_path, REFERENCE, profile
    )

    assert completed.returncode == 0, completed.stderr
    assert f"profile:={profile}" in command
    assert bag_name.startswith(f"hamr_study_{profile}_v020_")


def test_reference_accepts_obtuse_triangle_profile_before_recording(tmp_path):
    completed, command, bag_name, _root = run_script(
        tmp_path,
        REFERENCE,
        "triangle_120",
        "--speed",
        "0.15",
        "--caster",
        "traditional",
    )

    assert completed.returncode == 0, completed.stderr
    assert "profile:=triangle_120" in command
    assert "speed_m_s:=0.15" in command
    assert bag_name.startswith("hamr_study_triangle_120_v015_traditional_")


def test_reference_accepts_start_translated_legacy_waypoint_profile(tmp_path):
    completed, command, bag_name, _root = run_script(
        tmp_path,
        REFERENCE,
        "waypoint_traj_simple",
        "--speed",
        "0.20",
        "--caster",
        "traditional",
    )

    assert completed.returncode == 0, completed.stderr
    assert "profile:=waypoint_traj_simple" in command
    assert "speed_m_s:=0.20" in command
    assert bag_name.startswith(
        "hamr_study_waypoint_traj_simple_v020_traditional_"
    )


def test_unplanned_pilot_omits_empty_optional_launch_arguments(tmp_path):
    completed, command, bag_name, _root = run_script(
        tmp_path,
        REFERENCE,
        "circle_cw",
        "--speed",
        "0.15",
        "--caster",
        "traditional",
        "--repeat",
        "1",
    )

    assert completed.returncode == 0, completed.stderr
    assert bag_name.startswith("hamr_study_circle_cw_v015_traditional_")
    assert "profile:=circle_cw" in command
    assert "planned_id:=" not in command
    assert "video_id:=" not in command
    assert "notes:=" not in command


def test_planned_id_accepts_safe_values_through_128_characters(tmp_path):
    planned_id = "P" + "a" * 127
    completed, command, _bag, _root = run_script(
        tmp_path, REFERENCE, "straight_forward",
        "--planned-id", planned_id,
    )

    assert completed.returncode == 0, completed.stderr
    assert f"planned_id:={planned_id}" in command


@pytest.mark.parametrize(
    "planned_id",
    ["a" * 129, "contains spaces", "../unsafe", "-leading"],
)
def test_reference_rejects_unsafe_planned_id_before_recording(
    tmp_path, planned_id
):
    completed, command, bag_name, _root = run_script(
        tmp_path, REFERENCE, "straight_forward",
        "--planned-id", planned_id,
    )

    assert completed.returncode == 2
    assert command == []
    assert bag_name == ""


@pytest.mark.parametrize(
    ("option", "expected_error"),
    (
        ("--expected-profile-sha256", "Profile SHA-256 mismatch"),
        (
            "--expected-controller-config-sha256",
            "Controller-config SHA-256 mismatch",
        ),
    ),
)
def test_expected_hash_mismatch_is_rejected_before_launch(
    tmp_path, option, expected_error
):
    completed, command, bag_name, _root = run_script(
        tmp_path, REFERENCE, "straight_forward", option, "0" * 64
    )

    assert completed.returncode == 2
    assert expected_error in completed.stderr
    assert command == []
    assert bag_name == ""


@pytest.mark.parametrize(
    "option",
    [
        "--expected-profile-sha256",
        "--expected-controller-config-sha256",
    ],
)
def test_malformed_expected_hash_is_rejected_before_launch(tmp_path, option):
    completed, command, bag_name, _root = run_script(
        tmp_path, REFERENCE, "straight_forward", option, "not-a-digest"
    )

    assert completed.returncode == 2
    assert "64 hexadecimal" in completed.stderr
    assert command == []
    assert bag_name == ""


def copied_runner_with_config(tmp_path, controller_text):
    """Copy the wrapper into a minimal source layout for invalid-config tests."""
    layout = tmp_path / "layout"
    scripts = layout / "hamr_bringup" / "scripts"
    config = layout / "hamr_bringup" / "config"
    profiles = layout / "reference_trajectory" / "config" / "trajectories"
    scripts.mkdir(parents=True)
    config.mkdir(parents=True)
    shutil.copytree(PROFILE_DIRECTORY, profiles)
    copied_runner = scripts / "run_study_reference"
    shutil.copy2(REFERENCE, copied_runner)
    (config / "hamr_hw_control_params.yaml").write_text(
        controller_text, encoding="utf-8"
    )
    return copied_runner


@pytest.mark.parametrize(
    ("old", "new", "expected_error"),
    (
        (
            "wheel_speed_limit_rad_s: 2.93215314335",
            "wheel_speed_limit_rad_s: 0.0",
            "must be a positive number",
        ),
        (
            "turret_enabled: false",
            "turret_enabled: disabled",
            "must be a YAML boolean",
        ),
    ),
)
def test_invalid_controller_provenance_is_rejected_before_launch(
    tmp_path, old, new, expected_error
):
    controller_text = CONTROLLER_CONFIG.read_text(encoding="utf-8")
    copied_runner = copied_runner_with_config(
        tmp_path, controller_text.replace(old, new)
    )
    execution = tmp_path / "execution"
    execution.mkdir()

    completed, command, bag_name, _root = run_script(
        execution, copied_runner, "straight_forward"
    )

    assert completed.returncode == 2
    assert expected_error in completed.stderr
    assert command == []
    assert bag_name == ""


def test_every_unique_gate_three_planner_command_passes_preflight(tmp_path):
    plan = tmp_path / "plan.csv"
    completed = subprocess.run(
        [
            str(PLANNER),
            "--caster",
            "traditional",
            "--through-gate",
            "3",
            "--repetitions",
            "1",
            "--output",
            str(plan),
        ],
        check=False,
        capture_output=True,
        text=True,
        env={**os.environ, "HAMR_BAG_ROOT": str(tmp_path / "plan_bags")},
    )
    assert completed.returncode == 0, completed.stderr
    with plan.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 12

    for index, row in enumerate(rows):
        generated = shlex.split(row["run_command"])
        assert generated[:4] == [
            "ros2", "run", "hamr_bringup", "run_study_reference"
        ]
        execution = tmp_path / f"planned_{index:02d}"
        execution.mkdir()
        result, command, _bag, _root = run_script(
            execution, REFERENCE, *generated[4:]
        )
        assert result.returncode == 0, (
            f"{row['planned_id']}: {result.stderr}"
        )
        assert f"planned_id:={row['planned_id']}" in command
        assert f"profile_sha256:={row['profile_sha256']}" in command
        assert (
            f"controller_config_sha256:="
            f"{row['controller_config_sha256']}"
        ) in command


@pytest.mark.parametrize("profile", ["Triangle", "circle", "unknown", "../triangle"])
def test_reference_rejects_unknown_profile_before_recording(tmp_path, profile):
    completed, command, bag_name, _root = run_script(tmp_path, REFERENCE, profile)

    assert completed.returncode == 2
    assert command == []
    assert bag_name == ""


@pytest.mark.parametrize("speed", ["0", "0.001", "0.21", "1.01", "fast", "0.155"])
def test_reference_rejects_unsupported_or_ambiguous_speed(tmp_path, speed):
    completed, command, _bag, _root = run_script(
        tmp_path, REFERENCE, "triangle", "--speed", speed
    )

    assert completed.returncode == 2
    assert command == []


def test_reference_collision_gets_deterministic_suffix(tmp_path):
    first, _command, first_name, bag_root = run_script(
        tmp_path, REFERENCE, "triangle"
    )
    assert first.returncode == 0
    (bag_root / first_name).mkdir()

    second_root = tmp_path / "second"
    second_root.mkdir()
    fake_bin = fake_commands(second_root)
    capture_args = second_root / "args.txt"
    capture_bag = second_root / "bag.txt"
    environment = {
        **os.environ,
        "PATH": f'{fake_bin}:{os.environ["PATH"]}',
        "CAPTURE_ARGS": str(capture_args),
        "CAPTURE_BAG": str(capture_bag),
        "HAMR_BAG_ROOT": str(bag_root),
    }
    completed = subprocess.run(
        [str(REFERENCE), "triangle"],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert completed.returncode == 0
    assert capture_bag.read_text().strip() == f"{first_name}_001"


def test_launch_has_fixed_lifecycle_and_exactly_one_recorder_and_reference():
    source = LAUNCH.read_text(encoding="utf-8")

    assert source.count('exec="record_hamr_test_bag"') == 1
    assert source.count('exec="study_trajectory"') == 1
    assert '<param name="run_id"' in source
    assert '<param name="planned_id"' in source
    assert '<param name="preflight_profile_sha256"' in source
    assert '<param name="controller_config_sha256"' in source
    assert '<param name="wheel_speed_limit_rad_s"' in source
    assert '<param name="turret_control_enabled"' in source
    assert '<param name="nominal_start_base_yaw_rad"' in source
    assert '<param name="start_base_yaw_tolerance_rad"' in source
    assert '<param name="study_stack_contract_version"' in source
    assert '<param name="controller_odom_timeout_s"' in source
    assert '<param name="vicon_pose_guard_enabled"' in source
    assert '<param name="reference_timeout_s"' in source
    assert '<param name="xy_velocity_source"' in source
    assert '<param name="reference_timer_hz" value="50.0"' in source
    assert '<param name="startup_hold_s" value="2.0"' in source
    assert '<param name="final_hold_s" value="3.0"' in source
    assert '<param name="loop" value="false"' in source
    assert '<param name="required_reference_subscribers" value="2"' in source


def test_metadata_topic_is_recorded_with_transient_local_qos():
    recorder = RECORDER.read_text(encoding="utf-8")
    qos = QOS.read_text(encoding="utf-8")

    assert "^/hamr_test/trajectory_metadata$" in recorder
    assert "/hamr_test/trajectory_metadata:" in qos
    metadata_qos = qos.split("/hamr_test/trajectory_metadata:", 1)[1]
    assert "reliability: reliable" in metadata_qos
    assert "durability: transient_local" in metadata_qos
    assert not re.search(r"\beval\b", REFERENCE.read_text(encoding="utf-8"))
