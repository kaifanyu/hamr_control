"""Preparation must preserve identity/time and expose Vicon orientation faults."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from caster_vicon_comparison import (
    DEFAULT_QUALITY, POSE_FILES, POSE_TOPICS, ROLES, WHEELS,
    align_wheel, elapsed_ns, prepare, quaternion_step_deg, read_poses,
)

EPOCH = 1_793_256_992_820_270_894


def csv_file(path, fields, rows):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(rows)


def poses(path, times=(0, .01, .02, .03, .04), quaternions=None, x=0):
    qs = quaternions or [(0, 0, 0, 1)] * len(times)
    csv_file(path, ["bag_time_s", "header_time_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"],
             [[t, t, x, 1, 2, *q] for t, q in zip(times, qs)])


def analysis_source(path):
    path.mkdir()
    for label, filename in POSE_FILES.items():
        poses(path / filename, x={"base": 0, "front_left": 1, "front_right": -1}[label])
    rows = []
    for index, wheel in enumerate(WHEELS):
        for role_index, role in enumerate(ROLES):
            for t in (0, .01, .02, .03, .04):
                angle = index*10 + role_index + t
                rows.append([f"/caster_recording/{wheel}/{role}/joint_states", t, angle, t, "nan"])
    csv_file(path / "encoder_signals.csv",
             ["topic", "bag_time_s", "recorded_angle_rad", "unwrapped_change_from_first_sample_rad", "reported_velocity_rad_s"], rows)
    (path / "summary.json").write_text(json.dumps({"frames": {topic: ["mocap"] for topic in POSE_TOPICS.values()}, "sha256": "a"*64}))
    manifest = {"start_epoch_ns": EPOCH, "source_mcap": {"sha256": "a"*64},
                "files": [{"path": p.name, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                          for p in path.iterdir()]}
    (path / "artifact_manifest.json").write_text(json.dumps(manifest))


def test_quaternion_sign_is_not_a_rotation_and_half_turn_is():
    q = [0, 0, math.sin(.3), math.cos(.3)]
    assert quaternion_step_deg(q, [-v for v in q]) == pytest.approx(0, abs=2e-6)
    assert quaternion_step_deg([0, 0, 0, 1], [1, 0, 0, 0]) == pytest.approx(180)


def test_jump_neighborhood_and_gaps_are_marked_without_pose_correction(tmp_path):
    path = tmp_path / "pose.csv"
    raw_q = [(0, 0, 0, 1), (0, 0, 0, -1), (1, 0, 0, 0), (1, 0, 0, 0), (1, 0, 0, 0)]
    poses(path, times=(0, .01, .02, .03, .25), quaternions=raw_q)
    rows = read_poses(path, {**DEFAULT_QUALITY, "jump_guard_s": .011})
    assert [row["orientation_jump"] for row in rows] == [False, False, True, False, False]
    assert [row["q"] for row in rows] == [list(q) for q in raw_q]
    assert all(rows[i]["jump_neighborhood"] for i in (1, 2, 3))
    assert not rows[-1]["gap_ok"]
    assert rows[-1]["segment"] == 2


def test_alignment_never_uses_future_or_other_wheel_samples(tmp_path):
    path = tmp_path / "poses.csv"
    poses(path, times=(0, .015, .03))
    base = read_poses(path, DEFAULT_QUALITY)
    forks = [dict(base[1], time_ns=20_000_000)]
    streams = {}
    for wheel_index, wheel in enumerate(WHEELS):
        for role in ROLES:
            streams[wheel, role] = [dict(time_ns=10_000_000, angle_rad=wheel_index+1., change_rad=0., velocity_rad_s=0.),
                                   dict(time_ns=21_000_000, angle_rad=999., change_rad=999., velocity_rad_s=0.)]
    left = align_wheel("front_left", forks, base, streams, DEFAULT_QUALITY, 0, 30_000_000, EPOCH)[0]
    right = align_wheel("front_right", forks, base, streams, DEFAULT_QUALITY, 0, 30_000_000, EPOCH)[0]
    assert left["base_elapsed_ns"] == 15_000_000
    assert left["h1_recorded_angle_rad"] == 1.
    assert right["h1_recorded_angle_rad"] == 2.
    assert left["h1_age_s"] == pytest.approx(.01)
    assert left["epoch_ns"] == EPOCH + 20_000_000
    assert left["physical_prediction_ready"] is False
    np.testing.assert_allclose([left[f"fork_relative_q{x}"] for x in "xyzw"], [0, 0, 0, 1])
    stale = align_wheel("front_left", forks, base, streams,
                        {**DEFAULT_QUALITY, "max_encoder_age_s": .001}, 0, 30_000_000)[0]
    assert not stale["all_encoders_fresh"]


def test_export_uses_exact_epoch_manifest_and_keeps_sources(tmp_path):
    source, target = tmp_path / "source", tmp_path / "output"
    analysis_source(source)
    before = {p.name: p.read_bytes() for p in source.iterdir()}
    report = prepare(source, target, make_plot=False)
    assert report["start_epoch_ns"] == EPOCH
    assert report["artifact_manifest_verified"] is True
    assert report["physical_prediction_ready"] is False
    assert report["version"] == 2
    assert len(report["trajectory_estimator_sha256"]) == 64
    assert report["trajectory_estimation"]["wheels"]["front_left"]["estimated_rows"] == 4
    assert report["wheels"]["front_left"]["rows"] == 5
    with (target / "front_right_aligned.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert int(rows[2]["epoch_ns"]) == EPOCH + 20_000_000
    assert float(rows[2]["center_recorded_angle_rad"]) == pytest.approx(12.02)
    assert float(rows[2]["fork_speed_m_s"]) == pytest.approx(0., abs=1e-12)
    assert float(rows[2]["center_estimated_rate_rad_s"]) == pytest.approx(1.)
    assert before == {p.name: p.read_bytes() for p in source.iterdir()}
    with pytest.raises(ValueError, match="not empty"):
        prepare(source, target, make_plot=False)


def test_missing_epoch_stays_unknown_and_mixed_frames_fail(tmp_path):
    source = tmp_path / "source"
    analysis_source(source)
    (source / "artifact_manifest.json").unlink()
    report = prepare(source, tmp_path / "unknown_epoch", make_plot=False)
    assert report["start_epoch_ns"] is None
    assert elapsed_ns("20.590937477") == 20_590_937_477
    summary = json.loads((source / "summary.json").read_text())
    summary["frames"]["/Right_Caster/pose"] = ["mocap", "odom"]
    (source / "summary.json").write_text(json.dumps(summary))
    with pytest.raises(ValueError, match="common recorded frame"):
        prepare(source, tmp_path / "invalid_frames", make_plot=False)


def test_mixed_archive_files_are_rejected_before_output(tmp_path):
    source = tmp_path / "source"
    analysis_source(source)
    with (source / "encoder_signals.csv").open("a") as stream:
        stream.write("\n")
    output = tmp_path / "output"
    with pytest.raises(ValueError, match="hash mismatch"):
        prepare(source, output, make_plot=False)
    assert not output.exists()


def test_base_marker_transform_uses_inverse_orientation(tmp_path):
    path = tmp_path / "poses.csv"
    q = (0, 0, math.sin(math.pi/4), math.cos(math.pi/4))
    poses(path, quaternions=[q]*5)
    base = read_poses(path, DEFAULT_QUALITY)
    fork = dict(base[2], p=[0, 2, 2])
    streams = {(wheel, role): [dict(time_ns=20_000_000, angle_rad=0., change_rad=0., velocity_rad_s=0.)]
               for wheel in WHEELS for role in ROLES}
    row = align_wheel("front_left", [fork], base, streams, DEFAULT_QUALITY, 0, 40_000_000)[0]
    np.testing.assert_allclose([row[f"fork_marker_in_base_marker_{axis}_m"] for axis in "xyz"], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose([row[f"fork_relative_q{axis}"] for axis in "xyzw"], [0, 0, 0, 1], atol=1e-12)
