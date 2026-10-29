"""Both replay routes retain physical references and the corrected display sign."""
import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from caster_center_calibration import load_calibration
from caster_replay_model import ReplayModel, select_center_reference
from render_caster_encoder_video import link_transforms
from render_caster_pair_recording import paired_mappings


@pytest.fixture(params=["front_left", "front_right"])
def reference(request):
    wheel = request.param
    calibration = load_calibration(PACKAGE / f"config/{wheel}_center_calibration.json")
    mapping, _ = select_center_reference(wheel)
    return wheel, calibration, mapping


def event(calibration, count, seq=0, boot=None):
    envelope = copy.deepcopy(calibration["observations"][0])
    packet = envelope["packet"]
    packet["samples"] = packet["samples"][:1]
    packet["samples"][0].update(
        seq=seq, t_us=(seq + 1) * 5000, position14=count,
        absolute_angle_rad=count * math.tau / 16384,
        calibrated_alpha_rad=.123, angle_rad=None, flags=1 | 2 | (1 << 8))
    if boot is not None:
        packet["boot_id"] = boot
    return envelope


def h1_axis(position):
    robot = ET.parse(PACKAGE.parent / "hamr_ball_caster/urdf/ball_caster.urdf").getroot()
    transforms = link_transforms(robot, {"caster_carrier_joint": position})
    joint = robot.find("./joint[@name='caster_positive_hemisphere_joint']")
    return transforms[joint.find("child").get("link")][:3, :3] @ np.fromstring(
        joint.find("axis").get("xyz"), sep=" ")


def test_paired_and_standalone_references_match_confirmed_calibration(reference):
    wheel, calibration, single = reference
    paired, _ = paired_mappings({}, SimpleNamespace(
        center_reference="auto", mapping_config=None, left_mapping_config=None,
        right_mapping_config=None, start_pose="recorded"))
    assert calibration["physical_direction_verified"]
    assert not calibration["accuracy_validated"]
    assert calibration["counts_per_revolution"] == 16384
    assert calibration["checks"]["result"] == "PASS"
    assert single["center_reference_count"] == calibration["zero_position14"]
    assert single["center_reference_model_rad"] == calibration["cad_reference"]["center_reference_model_rad"]
    # The user reversed the rendered direction after reviewing both casters.
    # Preserve the original calibration evidence independently of this override.
    assert calibration["cad_reference"]["center_sign"] == -1
    assert single["center_sign"] == 1
    assert ReplayModel(single).mapping == ReplayModel(paired[wheel]).mapping


@pytest.mark.parametrize("offset,axis", [
    (0, [0, 0, 1]), (4096, [-1, 0, 0]), (8192, [0, 0, -1]), (12288, [1, 0, 0]),
])
def test_any_initial_phase_and_reboot_retain_absolute_orientation(reference, offset, axis):
    wheel, calibration, mapping = reference
    count = (calibration["zero_position14"] + offset) % 16384
    model = ReplayModel(mapping, wheel_id=wheel)
    value = event(calibration, count)
    original = copy.deepcopy(value)
    model.receive(value, 1_000_000_000)
    np.testing.assert_allclose(h1_axis(model.values[3]["position"]), axis, atol=1e-12)
    assert value == original  # Firmware phase and stored data were not rewritten.
    model.receive(event(calibration, count, boot="0123456789abcdef"), 1_010_000_000)
    np.testing.assert_allclose(h1_axis(model.values[3]["position"]), axis, atol=1e-12)


def test_rotation_crosses_raw_and_relative_wraps_without_visual_discontinuity(reference):
    wheel, calibration, mapping = reference
    model = ReplayModel(mapping, wheel_id=wheel)
    zero = calibration["zero_position14"]
    # Includes both the raw 16383->0 transition and the reference +/-pi branch.
    for seq, count in enumerate([16383, 0, (zero + 8191) % 16384, (zero + 8192) % 16384,
                                 (zero + 8193) % 16384]):
        model.receive(event(calibration, count, seq), 1_000_000_000 + seq * 5_000_000)
        phase = (count - zero) * math.tau / 16384
        np.testing.assert_allclose(h1_axis(model.values[3]["position"]),
                                   [-math.sin(phase), 0, math.cos(phase)], atol=1e-12)
