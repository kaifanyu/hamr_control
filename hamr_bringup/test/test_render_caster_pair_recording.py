"""Paired playback must share time while retaining independent references."""
from argparse import Namespace
import math
from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from caster_pair_recording import load_pair_recording
from caster_replay_model import ReplayModel
from render_caster_pair_recording import paired_frames, paired_mappings
from test_render_caster_recording import local_packets, save_packets
from test_caster_encoder_replay import wheel_envelope


def arguments(**kwargs):
    values = dict(center_reference='auto', mapping_config=None,
                  left_mapping_config=None, right_mapping_config=None,
                  start_pose='ground-contact')
    values.update(kwargs)
    return Namespace(**values)


def test_default_pair_references_preserve_each_wheels_absolute_phase():
    mappings, refs = paired_mappings({}, arguments())
    left, right = mappings['front_left'], mappings['front_right']
    assert left['center_reference_count'] == 7792
    assert right['center_reference_count'] == 16183
    assert left['center_sign'] == 1
    assert right['center_sign'] == 1
    assert left['center_reference_model_rad'] == 5.190977717061848
    assert right['center_reference_model_rad'] == 5.190977717061848
    assert left['center_reference_device_id'] != right['center_reference_device_id']
    assert all(ref['mode'] == 'absolute-center-reference' for ref in refs.values())
    for wheel, mapping in mappings.items():
        model = ReplayModel(mapping, wheel_id=wheel)
        assert model.mapping['center_zero_rad'] == 0
        assert model.mapping['h1_zero_rad'] == model.mapping['h2_zero_rad'] == 0


def test_pair_samples_one_clock_and_waits_for_later_wheel(tmp_path):
    left, right = local_packets(), local_packets('front_right')
    for envelope in right:
        envelope['clock_mapping']['offset_ns'] += 2_000_000
        for sample in envelope['packet'].get('samples', []):
            if envelope['packet']['node_id'] != 3:
                sample['angle_rad'] += 1
    save_packets(tmp_path, left + right)
    events, provenance = load_pair_recording(tmp_path)
    times = [provenance['start_ns'] + 1_000_000, provenance['end_ns']]
    refs = {wheel: {'stale_timeout_s': 1.} for wheel in events}
    frames = paired_frames(events, times, dict.fromkeys(events, {}), refs)
    assert frames[0]['front_left']['status']['complete_pose_available']
    assert not frames[0]['front_right']['status']['complete_pose_available']
    for frame, stamp in zip(frames, times):
        assert {f['stamp_ns'] for f in frame.values()} == {stamp}
    joint = 'caster_positive_hemisphere_joint'
    assert frames[-1]['front_right']['joints'][joint] - frames[-1]['front_left']['joints'][joint] == pytest.approx(1)


def test_explicit_mapping_only_changes_selected_wheel(tmp_path):
    path = tmp_path / 'left.yaml'
    path.write_text('replay_caster_encoder_joints:\n  ros__parameters:\n'
                    '    h1_sign: -1.0\n    center_sign: -1.0\n')
    mappings, refs = paired_mappings({}, arguments(left_mapping_config=path, start_pose='recorded'))
    assert mappings['front_left'] == {'h1_sign': -1., 'center_sign': -1.}
    assert mappings['front_right']['center_reference_count'] == 16183
    assert refs['front_left']['mode'] == 'recorded'


def test_pair_mapping_rejects_wrong_wheel_reference(tmp_path):
    path = tmp_path / 'left.yaml'
    path.write_text((SCRIPTS.parent / 'config/caster_encoder_replay_front_right_center.yaml').read_text())
    with pytest.raises(ValueError, match='different wheel'):
        paired_mappings({}, arguments(left_mapping_config=path))


@pytest.mark.parametrize('wheel', ['front_left', 'front_right'])
def test_center_direction_reverses_about_saved_reference(wheel):
    mapping = paired_mappings({}, arguments())[0][wheel]
    model = ReplayModel(mapping, wheel_id=wheel)
    reference = mapping['center_reference_model_rad']
    positions = []
    for seq, offset in enumerate((0, 1024)):
        count = (mapping['center_reference_count'] + offset) % 16384
        event = wheel_envelope(3, wheel=wheel, device=mapping['center_reference_device_id'], seq=seq)
        sample = event['packet']['samples'][0]
        sample.update(position14=count, absolute_angle_rad=count * math.tau / 16384,
                      calibrated_alpha_rad=.123)
        model.receive(event, 1_000_000_000 + seq * 5_000_000)
        positions.append(model.values[3]['position'])
    assert positions[0] == pytest.approx(reference)
    assert positions[1] - positions[0] == pytest.approx(math.tau / 16)
    # Starting a new run at this phase must retain the absolute orientation.
    restarted = ReplayModel(mapping, wheel_id=wheel)
    restarted.receive(event, 2_000_000_000)
    assert restarted.values[3]['position'] == pytest.approx(positions[1])
