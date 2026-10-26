"""Keep the saved CENTER reference usable without assuming yaw or turn history."""

import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"
sys.path.insert(0, str(SCRIPTS))

from caster_center_calibration import load_calibration, main, orientation, packet_reading
from caster_replay_model import ReplayModel
from render_caster_encoder_video import link_transforms


CALIBRATION_PATH = PACKAGE / "config/front_left_center_calibration.json"
MAPPING_PATH = PACKAGE / "config/caster_encoder_replay_front_left_center.yaml"


@pytest.fixture
def calibration():
    return load_calibration(CALIBRATION_PATH)


def test_saved_observations_preserve_designated_zero_and_half_turn_residual(calibration):
    before = copy.deepcopy(calibration)
    observed = [packet_reading(observation, calibration) for observation in calibration["observations"]]
    assert [reading["position14"] for reading in observed] == [1210, 9417]
    assert [reading["orientation_deg"] for reading in observed] == [0.0, 180.32958984375]
    assert [reading["raw_absolute_deg"] for reading in observed] == [26.5869140625, 206.91650390625]
    assert [reading["seq"] for reading in observed] == [476040, 489912]
    assert observed[1]["orientation_deg"] - 180.0 == 0.32958984375
    for reading in observed:
        assert reading["physical_direction_verified"] is False
        assert reading["accuracy_validated"] is False
        assert reading["positive_direction"] == "increasing raw counts"
        assert "not robot yaw" in reading["angle_definition"]
    assert calibration == before


@pytest.mark.parametrize("count, expected", [
    (1210, 0.0),
    (1209, 359.97802734375),
    (0, 333.4130859375),
    (16383, 333.39111328125),
])
def test_native_wrap_has_one_count_steps_without_half_turn_scale_fit(calibration, count, expected):
    assert orientation(count, calibration)["orientation_deg"] == expected


def test_explicit_negative_convention_reverses_angle_without_changing_zero(calibration):
    calibration["direction_sign"] = -1
    assert orientation(1210, calibration)["orientation_deg"] == 0.0
    result = orientation(9417, calibration)
    assert result["orientation_deg"] == 179.67041015625
    assert result["positive_direction"] == "decreasing raw counts"
    assert result["physical_direction_verified"] is False


def test_valid_absolute_position_does_not_need_multiturn_continuity(calibration):
    observation = calibration["observations"][0]
    sample = observation["packet"]["samples"][-1]
    assert sample["angle_rad"] is None
    assert sample["flags"] & (1 << 8)  # continuity_lost in the supplied packet
    assert not sample["flags"] & (1 << 4)
    assert packet_reading(observation, calibration)["orientation_deg"] == 0.0


@pytest.mark.parametrize("fault", ["parity", "spi", "not_ready", "invalid", "missing_count", "missing_raw"])
def test_latest_invalid_sample_never_falls_back_to_older_good_reading(calibration, fault):
    observation = copy.deepcopy(calibration["observations"][0])
    sample = observation["packet"]["samples"][-1]
    if fault == "parity":
        sample["flags"] |= 1 << 7
    elif fault == "spi":
        sample["flags"] |= 1 << 12
    elif fault == "not_ready":
        sample["flags"] &= ~1
    elif fault == "invalid":
        sample["flags"] &= ~2
    elif fault == "missing_count":
        sample["position14"] = None
    else:
        sample["absolute_angle_rad"] = None
    assert observation["packet"]["samples"][0]["flags"] == 1347
    assert packet_reading(observation, calibration) is None


@pytest.mark.parametrize("field, value", [
    ("device_id", "10:bd:a3:b2:14:a5"),
    ("wheel_id", "front_right"),
])
def test_calibration_cannot_be_applied_to_a_different_center(calibration, field, value):
    observation = copy.deepcopy(calibration["observations"][0])
    observation["packet"][field] = value
    with pytest.raises(ValueError, match="identity does not match"):
        packet_reading(observation, calibration)


def test_firmware_alpha_change_cannot_double_apply_the_raw_reference(calibration):
    observation = copy.deepcopy(calibration["observations"][1])
    for sample in observation["packet"]["samples"]:
        sample["calibrated_alpha_rad"] = 0.25
    assert packet_reading(observation, calibration)["orientation_deg"] == 180.32958984375


def test_reboot_does_not_reset_absolute_reference_to_new_start(calibration):
    observation = copy.deepcopy(calibration["observations"][1])
    observation["packet"]["boot_id"] = "0123456789abcdef"
    for index, sample in enumerate(observation["packet"]["samples"]):
        sample.update(seq=index, t_us=5000 * (index + 1))
    result = packet_reading(observation, calibration)
    assert result["orientation_deg"] == 180.32958984375
    assert result["boot_id"] == "0123456789abcdef"
    assert result["seq"] == 1


def test_inconsistent_live_raw_count_and_angle_is_rejected(calibration):
    observation = copy.deepcopy(calibration["observations"][0])
    observation["packet"]["samples"][-1]["absolute_angle_rad"] += 0.01
    with pytest.raises(ValueError, match="Raw CENTER angle disagrees"):
        packet_reading(observation, calibration)


@pytest.mark.parametrize("field, value", [
    ("version", 2),
    ("version", True),
    ("node_id", 1),
    ("node_id", 3.0),
    ("wheel_id", "other"),
    ("device_id", "not-a-mac"),
    ("device_id", None),
    ("counts_per_revolution", 8207),
    ("counts_per_revolution", 16384.0),
    ("zero_position14", -1),
    ("zero_position14", 16384),
    ("zero_position14", True),
    ("zero_absolute_angle_rad", 0.0),
    ("zero_absolute_angle_rad", float("nan")),
    ("direction_sign", 0),
    ("direction_sign", 1.0),
    ("physical_direction_verified", 0),
    ("accuracy_validated", None),
])
def test_saved_calibration_rejects_bad_schema_or_inconsistent_zero(calibration, tmp_path, field, value):
    calibration[field] = value
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(calibration))
    with pytest.raises(ValueError):
        load_calibration(path)


@pytest.mark.parametrize("count", [-1, 16384])
def test_manual_cli_rejects_out_of_range_counts_without_live_subscription(count, capsys):
    assert main(["--calibration", str(CALIBRATION_PATH), "--position14", str(count)]) == 2
    captured = capsys.readouterr()
    assert captured.out == ""
    assert "position14" in captured.err


def test_executable_offline_cli_prints_json_and_finds_saved_default():
    process = subprocess.run(
        [sys.executable, str(SCRIPTS / "caster_center_angle"), "--position14", "9417"],
        capture_output=True, text=True, check=False,
    )
    assert process.returncode == 0, process.stderr
    assert not process.stderr
    result = json.loads(process.stdout)
    assert result["input"] == "manual_position14"
    assert result["device_id"] == "10:bd:a3:b2:14:a4"
    assert result["orientation_deg"] == 180.32958984375
    assert result["accuracy_validated"] is False


def test_saved_replay_mapping_preserves_both_starting_poses_in_cad(calibration):
    mapping = yaml.safe_load(MAPPING_PATH.read_text())["replay_caster_encoder_joints"]["ros__parameters"]
    stale_timeout_s = mapping.pop("stale_timeout_s")
    model = ReplayModel(mapping, stale_timeout_s, calibration["wheel_id"])
    robot = ET.parse(PACKAGE.parent / "hamr_ball_caster/urdf/ball_caster.urdf").getroot()
    carrier_positions = []
    h1_axes = []
    for observation in calibration["observations"]:
        stamp = observation["receive_ros_ns"]
        for node_id, device in ((1, "10:bd:a3:af:f9:10"), (2, "10:bd:a3:b2:33:b0")):
            shell = copy.deepcopy(observation)
            shell["packet"].update(node_id=node_id, device_id=device)
            for sample in shell["packet"]["samples"]:
                sample.update(raw_count="0", raw_spi_word=None, position14=None,
                              angle_rad=0.0, absolute_angle_rad=None, calibrated_alpha_rad=None,
                              flags=1 | 2 | 16)
            model.receive(shell, stamp)
        model.receive(observation, stamp)
        names, positions, status = model.snapshot(stamp)
        assert status["complete_pose_available"]
        assert status["mapping_calibration_current"] is False
        assert status["accuracy"] == "unavailable_without_independent_reference"
        joint_positions = dict(zip(names, positions))
        carrier_positions.append(joint_positions["caster_carrier_joint"])
        transforms = link_transforms(robot, joint_positions)
        joint = robot.find(f"./joint[@name='{mapping['h1_joint']}']")
        axis = transforms[joint.find("child").get("link")][:3, :3] @ np.fromstring(joint.find("axis").get("xyz"), sep=" ")
        h1_axes.append(axis)
    np.testing.assert_allclose(h1_axes[0], [1, 0, 0], atol=1e-12)
    assert h1_axes[1][0] < -0.9999
    assert abs(h1_axes[1][2]) < 0.006
    assert math.degrees(carrier_positions[1] - carrier_positions[0]) == pytest.approx(180.32958984375)
    assert mapping["mapping_verified"] is False
    assert calibration["physical_direction_verified"] is False
