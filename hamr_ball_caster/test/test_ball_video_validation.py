"""Media audit distinguishes camera quality, file integrity and robot behavior."""
import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from finalize_ball_comparison import VIDEO_NAMES, aggregate_audit, sha256, validate_stream


def fixture_run(suite):
    run = suite / "ball/example"
    run.mkdir(parents=True)
    videos = {}
    for name in VIDEO_NAMES:
        path = run / (name + ".mp4")
        path.write_bytes(b"example media file")
        videos[name] = {"full_decode_passed": True, "sha256": sha256(path),
                        "streams": [{"width": 1280, "height": 720,
                                     "r_frame_rate": "25/1", "nb_frames": "25"}],
                        "format": {"duration": "1.0"}}
        if name != "overview_annotated":
            videos[name]["source_timing"] = {"passed": True, "max_source_image_gap_s": .04,
                "frames_queue_dropped": 0, "frames_duplicated": 0,
                "motion_quality": {"passed": True, "max_motion_image_gap_s": .04}}
    (run / "video_validation.json").write_text(json.dumps(videos))
    (run / "run_summary.json").write_text(json.dumps({"passed": False}))
    (run / "trajectory.json").write_text(json.dumps({"completed": True}))
    return run, videos


def test_audit_keeps_media_quality_separate_from_behavior_and_detects_changed_video(tmp_path):
    run, _ = fixture_run(tmp_path)
    audit = aggregate_audit(tmp_path, [{"name": "example"}])
    assert audit["passed"] and audit["fully_decoded_video_count"] == 3
    assert audit["behavior_checks_passed_count"] == 0
    assert audit["trajectory_completed_count"] == 1
    (run / "caster_closeup.mp4").write_bytes(b"changed after decode")
    changed = aggregate_audit(tmp_path, [{"name": "example"}])
    assert not changed["passed"]
    assert "changed after" in changed["failures"][0]


def test_decodable_video_with_a_camera_timestamp_gap_fails_media_audit(tmp_path):
    run, videos = fixture_run(tmp_path)
    videos["overview"]["source_timing"]["passed"] = False
    videos["overview"]["source_timing"]["motion_quality"] = {"passed": False, "max_motion_image_gap_s": .4}
    (run / "video_validation.json").write_text(json.dumps(videos))
    audit = aggregate_audit(tmp_path, [{"name": "example"}])
    assert not audit["passed"]
    assert audit["fully_decoded_video_count"] == 3
    assert audit["worst_motion_source_image_gap_s"] == .4


def test_incomplete_matrix_never_claims_all_videos_passed(tmp_path):
    fixture_run(tmp_path)
    audit = aggregate_audit(tmp_path, [{"name": "example"}, {"name": "not_yet_recorded"}])
    assert not audit["passed"] and not audit["complete"]
    assert audit["missing_cases"] == ["not_yet_recorded"]


def test_report_refresh_detects_late_pair_analysis_and_video_validation(tmp_path):
    from finalize_ball_comparison import report_input_signature
    initial = report_input_signature(tmp_path)
    pair = tmp_path / "pairs/example/comparison.json"
    pair.parent.mkdir(parents=True)
    pair.write_text('{}')
    analyzed = report_input_signature(tmp_path)
    assert analyzed != initial
    fixture_run(tmp_path)
    finalized = report_input_signature(tmp_path)
    assert finalized != analyzed
    (tmp_path / "FINDINGS.md").write_text('Rendered report is not an input')
    assert report_input_signature(tmp_path) == finalized
    (tmp_path / "KEY_FINDINGS.md").write_text('Curated conclusions are a report input')
    assert report_input_signature(tmp_path) != finalized
