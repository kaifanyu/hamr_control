"""Portable analysis must preserve timing and distinguish true Vicon flips."""
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
import yaml

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
import analyze_combined_caster_bag as analysis

EPOCH = 1_793_256_992_820_270_894


def recording(tmp_path):
    root = tmp_path / 'recording with spaces'
    bag = root / 'bags/combined'
    bag.mkdir(parents=True)
    (bag / 'part_0.mcap').write_bytes(b'fake storage for path-resolution tests')
    (bag / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
        'relative_file_paths': ['part_0.mcap'],
        'starting_time': {'nanoseconds_since_epoch': EPOCH},
        'duration': {'nanoseconds': 2_000_000_001},
        'message_count': 0, 'topics_with_message_count': [],
    }}))
    (root / 'clock-map.json').write_text('{"mappings": []}')
    (root / 'export-summary.json').write_text('{"nodes": []}')
    return root, bag


def test_quaternion_sign_equivalence_is_not_a_physical_flip():
    quaternions = np.array([[0, 0, 0, 1], [0, 0, 0, -2], [2, 0, 0, 0], [-1, 0, 0, 0]])
    np.testing.assert_allclose(analysis.quaternion_steps_deg(quaternions), [0, 180, 0], atol=1e-10)
    with pytest.raises(ValueError, match='undefined'):
        analysis.quaternion_steps_deg([[0, 0, 0, 0]])


def test_common_window_keeps_staggered_encoder_start_and_end_times():
    arrays = {topic: np.array([[index / 10, 0, 0, 0], [10-index / 10, 0, 0, 0]])
              for index, topic in enumerate(analysis.ENC)}
    assert analysis.common_encoder_window(arrays) == (.5, 9.5)
    arrays[analysis.ENC[0]][:, 0] += 20
    with pytest.raises(ValueError, match='no common'):
        analysis.common_encoder_window(arrays)
    del arrays[analysis.ENC[-1]]
    with pytest.raises(ValueError, match='all six'):
        analysis.common_encoder_window(arrays)


def test_recording_bag_and_file_inputs_preserve_integer_epoch(tmp_path):
    root, bag = recording(tmp_path)
    before = {p: p.read_bytes() for p in root.rglob('*') if p.is_file()}
    for source in (root, bag, bag / 'part_0.mcap'):
        output = tmp_path / 'new report'
        analysis.configure(source, output)
        assert analysis.T0 == EPOCH
        assert type(analysis.T0) is int
        assert analysis.ROOT == root
        assert analysis.BAG_FILES == [bag / 'part_0.mcap']
        assert not output.exists()  # Configuration does not create or mutate files.
    assert all(p.read_bytes() == content for p, content in before.items())


def test_output_refuses_existing_reports_and_recording_root(tmp_path):
    root, _ = recording(tmp_path)
    with pytest.raises(ValueError, match='separate report'):
        analysis.configure(root, root)
    output = tmp_path / 'report'
    output.mkdir()
    marker = output / 'keep.txt'
    marker.write_text('original')
    with pytest.raises(ValueError, match='not empty'):
        analysis.configure(root, output)
    assert marker.read_text() == 'original'


def test_split_bag_requires_complete_metadata_counted_shards(tmp_path):
    root, bag = recording(tmp_path)
    path = bag / 'metadata.yaml'
    metadata = yaml.safe_load(path.read_text())
    metadata['rosbag2_bagfile_information']['relative_file_paths'].append('part_1.mcap')
    path.write_text(yaml.safe_dump(metadata))
    with pytest.raises(ValueError, match='complete local'):
        analysis.configure(root, tmp_path / 'output')
    (bag / 'part_1.mcap').write_bytes(b'second shard')
    with pytest.raises(ValueError, match='split bag'):
        analysis.configure(bag / 'part_0.mcap', tmp_path / 'output')
    analysis.configure(bag, tmp_path / 'output')
    assert len(analysis.BAG_FILES) == 2


def test_cli_requires_explicit_output_and_help_never_reads_a_recording(tmp_path):
    script = SCRIPTS / 'analyze_combined_caster_bag.py'
    result = subprocess.run([sys.executable, str(script), '--help'], cwd=tmp_path,
                            capture_output=True, text=True)
    assert result.returncode == 0
    assert '--output-dir' in result.stdout and '--sidecar-dir' in result.stdout
    result = subprocess.run([sys.executable, str(script), str(tmp_path)], cwd=tmp_path,
                            capture_output=True, text=True)
    assert result.returncode == 2 and '--output-dir' in result.stderr
    assert list(tmp_path.iterdir()) == []
