"""Unified source resolution and numerical continuity contracts."""
from pathlib import Path
import sys
import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from caster_bag_reader import resolve_source
from caster_bag_report import quaternion_steps_deg, segmented_change

EPOCH = 1_793_256_992_820_270_894


def recording(tmp_path):
    root = tmp_path / "recording with spaces"
    bag = root / "bags/combined"
    bag.mkdir(parents=True)
    (bag / "part_0.mcap").write_bytes(b"fake storage for path-resolution tests")
    (bag / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {
        "relative_file_paths": ["part_0.mcap"],
        "starting_time": {"nanoseconds_since_epoch": EPOCH},
        "duration": {"nanoseconds": 2_000_000_001},
        "message_count": 0, "topics_with_message_count": [],
    }}))
    return root, bag


def test_quaternion_sign_equivalence_is_not_a_physical_flip():
    quaternions = np.array([[0, 0, 0, 1], [0, 0, 0, -2], [2, 0, 0, 0], [-1, 0, 0, 0]])
    np.testing.assert_allclose(quaternion_steps_deg(quaternions), [0, 180, 0], atol=1e-10)
    with pytest.raises(ValueError, match="undefined"):
        quaternion_steps_deg([[0, 0, 0, 0]])


def test_recording_bag_and_file_inputs_need_no_sidecars_and_preserve_epoch(tmp_path):
    root, bag = recording(tmp_path)
    before = {p: p.read_bytes() for p in root.rglob("*") if p.is_file()}
    for source in (root, bag, bag / "part_0.mcap"):
        resolved = resolve_source(source)
        assert resolved.metadata["starting_time"]["nanoseconds_since_epoch"] == EPOCH
        assert resolved.root == root
        assert resolved.files == [bag / "part_0.mcap"]
    assert all(path.read_bytes() == contents for path, contents in before.items())


def test_split_file_resolves_every_metadata_listed_shard(tmp_path):
    root, bag = recording(tmp_path)
    path = bag / "metadata.yaml"
    metadata = yaml.safe_load(path.read_text())
    metadata["rosbag2_bagfile_information"]["relative_file_paths"].append("part_1.mcap")
    path.write_text(yaml.safe_dump(metadata))
    with pytest.raises(ValueError, match="complete local"):
        resolve_source(root)
    (bag / "part_1.mcap").write_bytes(b"second shard")
    assert len(resolve_source(bag / "part_0.mcap").files) == 2
    metadata["rosbag2_bagfile_information"]["relative_file_paths"].append("../outside.mcap")
    path.write_text(yaml.safe_dump(metadata))
    with pytest.raises(ValueError, match="complete local"):
        resolve_source(bag)


def test_standalone_mcap_does_not_require_metadata_yaml(tmp_path):
    source = tmp_path / "standalone.mcap"
    source.write_bytes(b"source")
    assert resolve_source(source).files == [source]
    assert not resolve_source(source).metadata


def test_center_unwrap_stops_at_invalid_samples_sequence_segments_and_time_gaps():
    rows = [dict(stamp_ns=EPOCH+i*10_000_000, angle_rad=value, segment=0)
            for i, value in enumerate((6.2, .1, None, .2, .3, .4))]
    rows[4]["segment"] = 1
    rows[5]["stamp_ns"] += 1_000_000_000
    values = segmented_change(rows, "CENTER")
    assert values[0] == (0., 0)
    assert values[1][0] == pytest.approx(.1-6.2+2*np.pi)
    assert values[2] == (None, None)
    assert values[3:] == [(0., 1), (0., 2), (0., 3)]
