"""Anchor right carrier phase to the measured raw count without resetting runs."""
import copy
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from caster_center_calibration import load_calibration, orientation
from caster_replay_model import DEFAULT_MAPPING, ReplayModel, select_center_reference, status_text
from caster_encoder_selection import WheelIdentityError
from render_caster_encoder_video import ground_start_mapping, link_transforms, sample_frames

CAL = load_calibration(PACKAGE / "config/front_right_center_calibration.json")
MAPPING = yaml.safe_load((PACKAGE / "config/caster_encoder_replay_front_right_center.yaml").read_text())["replay_caster_encoder_joints"]["ros__parameters"]
MAPPING = {k: v for k, v in MAPPING.items() if k in DEFAULT_MAPPING}
ROBOT = ET.parse(PACKAGE.parent / "hamr_ball_caster/urdf/ball_caster.urdf").getroot()


def envelope(count, seq=1, boot=None):
    result = copy.deepcopy(CAL["observations"][0])
    sample = result["packet"]["samples"][0]
    sample.update(seq=seq, t_us=seq * 5000, position14=count,
                  absolute_angle_rad=count * math.tau / 16384,
                  calibrated_alpha_rad=0.123, angle_rad=None,
                  flags=1 | 2 | (1 << 8))  # Absolute phase survives lost unwrapping.
    result["packet"]["samples"] = [sample]
    if boot:
        result["packet"]["boot_id"] = boot
    return result


def shell(node, stamp=1_000_000_000):
    result = envelope(CAL["zero_position14"])
    result["packet"].update(node_id=node, device_id=f"02:00:00:00:00:{node:02d}")
    sample = result["packet"]["samples"][0]
    sample.update(raw_count="0", position14=None, absolute_angle_rad=None,
                  calibrated_alpha_rad=None, angle_rad=0., flags=1 | 2 | 16)
    return result


def h1_axis(model, stamp):
    names, angles, status = model.snapshot(stamp)
    joint = ROBOT.find(f"./joint[@name='{model.mapping['h1_joint']}']")
    transforms = link_transforms(ROBOT, dict(zip(names, angles)))
    return transforms[joint.find("child").get("link")][:3, :3] @ np.fromstring(joint.find("axis").get("xyz"), sep=" "), status


def seed_shells(model):
    for node in (1, 2):
        model.receive(shell(node), 1_000_000_000)


def test_saved_reference_uses_initial_pose_and_retains_native_scale():
    assert CAL["zero_position14"] == MAPPING["center_reference_count"] == 16183
    assert CAL["counts_per_revolution"] == 16384
    assert CAL["physical_direction_verified"] is True
    assert CAL["accuracy_validated"] is False
    assert CAL["checks"]["signed_rotation_deg"] == pytest.approx(359.84701360003305)
    assert CAL["cad_reference"]["center_sign"] == MAPPING["center_sign"] == -1
    assert CAL["cad_reference"]["center_reference_model_rad"] == MAPPING["center_reference_model_rad"]
    assert "H1 up" in orientation(16183, CAL)["angle_definition"]


@pytest.mark.parametrize("counts,expected", [(0, [0, 0, 1]), (4096, [1, 0, 0]),
                                             (8192, [0, 0, -1]), (12288, [-1, 0, 0])])
def test_any_recording_starts_at_measured_absolute_phase(counts, expected):
    model = ReplayModel(MAPPING, wheel_id="front_right")
    seed_shells(model)
    sample = envelope((16183 + counts) % 16384)
    unchanged = copy.deepcopy(sample)
    model.receive(sample, 1_000_000_000)
    axis, status = h1_axis(model, 1_000_000_000)
    np.testing.assert_allclose(axis, expected, atol=1e-12)
    assert sample == unchanged
    assert status["center_angle_basis"] == "raw position14"
    assert status["center_absolute_reference"]
    assert not status["mapping_calibration_current"]
    assert "fixed raw absolute reference" in status_text(status)


def test_reversing_sign_preserves_reference_pose():
    model = ReplayModel({**MAPPING, "center_sign": 1.})
    seed_shells(model)
    model.receive(envelope(16183), 1_000_000_000)
    np.testing.assert_allclose(h1_axis(model, 1_000_000_000)[0], [0, 0, 1], atol=1e-12)
    model.receive(envelope((16183 + 4096) % 16384, seq=2), 1_010_000_000)
    np.testing.assert_allclose(h1_axis(model, 1_010_000_000)[0], [-1, 0, 0], atol=1e-12)


def test_raw_wrap_reboot_firmware_zero_and_invalid_reads():
    model = ReplayModel(MAPPING)
    seed_shells(model)
    model.receive(envelope(16383), 1_000_000_000)
    before = model.values[3]["position"]
    sample = envelope(0, seq=2)
    sample["packet"]["samples"][0]["calibrated_alpha_rad"] = None
    model.receive(sample, 1_010_000_000)
    assert model.values[3]["position"] - before == pytest.approx(-math.tau / 16384)
    prior = model.values[3]["position"]
    sample = envelope(6000, seq=3)
    sample["packet"]["samples"][0]["flags"] |= 1 << 7
    model.receive(sample, 1_020_000_000)
    assert model.values[3]["position"] == prior
    assert model.snapshot(1_020_000_000)[2]["nodes"][2]["display_state"] == "invalid_held"
    model.receive(envelope(16183, boot="0123456789abcdef"), 1_030_000_000)
    np.testing.assert_allclose(h1_axis(model, 1_030_000_000)[0], [0, 0, 1], atol=1e-12)


def test_reference_rejects_wrong_wheel_device_and_raw_disagreement():
    with pytest.raises(ValueError, match="different wheel"):
        ReplayModel(MAPPING, wheel_id="front_left")
    model = ReplayModel(MAPPING)
    sample = envelope(16183)
    sample["packet"]["device_id"] = "02:00:00:00:00:03"
    with pytest.raises(WheelIdentityError, match="saved wheel/device"):
        model.receive(sample, 1_000_000_000)
    assert model.snapshot(1_000_000_000)[2]["identity_error"]
    model = ReplayModel(MAPPING)
    sample = envelope(16183)
    sample["packet"]["samples"][0]["absolute_angle_rad"] = .5
    with pytest.raises(ValueError, match="disagrees"):
        model.receive(sample, 1_000_000_000)


def test_reference_never_resets_to_first_pose_or_mixes_with_old_offset():
    with pytest.raises(ValueError, match="Ground-contact reset"):
        ground_start_mapping([], ROBOT, MAPPING, wheel_id="front_right")
    with pytest.raises(ValueError, match="center_zero_rad"):
        ReplayModel({**MAPPING, "center_zero_rad": .1})
    with pytest.raises(ValueError, match="different H1 CAD side"):
        ReplayModel({**MAPPING, "h1_joint": MAPPING["h2_joint"], "h2_joint": MAPPING["h1_joint"]})


def test_auto_selection_preserves_left_behavior_and_explicit_direction():
    left, path = select_center_reference("front_left", {"center_sign": -1.})
    assert left == {"center_sign": -1.} and path is None
    right, path = select_center_reference("front_right", {"center_sign": 1.})
    assert right["center_reference_enabled"] and right["center_sign"] == -1
    assert path.endswith("caster_encoder_replay_front_right_center.yaml")
    explicit, _ = select_center_reference("front_right", {**right, "center_sign": 1.})
    assert explicit["center_sign"] == 1.
    disabled, _ = select_center_reference("front_right", {}, "none")
    assert not disabled


def test_sample_frames_uses_reference_for_full_pose():
    events = [(1_000_000_000, shell(node)) for node in (1, 2)]
    events.append((1_000_000_000, envelope(16183)))
    events.append((1_010_000_000, envelope((16183 + 4096) % 16384, seq=2)))
    frames = sample_frames(events, [1_000_000_000, 1_010_000_000], MAPPING, wheel_id="front_right")
    assert frames[0]["joints"]["caster_carrier_joint"] == pytest.approx(MAPPING["center_reference_model_rad"])
    assert frames[1]["joints"]["caster_carrier_joint"] == pytest.approx(MAPPING["center_reference_model_rad"] - math.pi/2)


@pytest.mark.parametrize("wheel_selector", [None, "front_right"])
def test_renderer_auto_reference_overrides_assumed_start(tmp_path, monkeypatch, wheel_selector):
    from PIL import Image
    import render_caster_encoder_video as renderer
    bag = tmp_path / "fixture.mcap"
    bag.touch()
    events = [(1_000_000_000, shell(node)) for node in (1, 2)]
    # Begin at a different raw phase to catch accidental first-frame re-zero.
    events.append((1_000_000_000, envelope((16183 + 8192) % 16384)))
    monkeypatch.setattr(renderer, "read_bag", lambda *args: (events, 1_000_000_000, 1_100_000_000))
    drawn = []
    class Scene:
        def __init__(self, *args):
            self.ground_z = -.1
            self.window = SimpleNamespace(Finalize=lambda: None)
            assert args[-1]["caster_carrier_joint"] == pytest.approx(MAPPING["center_reference_model_rad"] + math.pi)
        def render(self, frame):
            drawn.append(frame)
            return Image.new("RGB", (16, 16))
    monkeypatch.setattr(renderer, "Scene", Scene)
    monkeypatch.setattr(renderer, "Overlay", lambda *args: SimpleNamespace(draw=lambda image, frame: image))
    renderer.render(SimpleNamespace(bag=bag, rate=1., width=800, fps=30, wheel=wheel_selector,
                                    topic=None, mapping_config=PACKAGE / "config/caster_encoder_replay_simulation.yaml",
                                    center_reference="auto", start_pose="ground-contact", plots=False,
                                    preview_time=0., full_detail=False, output_dir=tmp_path / "output"))
    assert drawn[0]["status"]["start_pose"]["mode"] == "absolute-center-reference"
    assert drawn[0]["status"]["center_absolute_reference"]
    assert drawn[0]["joints"]["caster_carrier_joint"] == pytest.approx(MAPPING["center_reference_model_rad"] + math.pi)
