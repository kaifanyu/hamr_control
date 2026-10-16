"""Finite, resumable matrix analysis preserves incomplete capture evidence."""
import json
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
import analyze_ball_comparison as BATCH
from test_ball_comparison import trajectory


def case_fixture(tmp_path):
    ball, swivel = tmp_path/"ball/example", tmp_path/"ordinary/example"
    ball.mkdir(parents=True)
    swivel.mkdir(parents=True)
    case = {"name": "example", "category": "trajectory", "ball_directory": str(ball),
            "swivel_directory": str(swivel), "tracking_comparison_region": "motion"}
    (tmp_path/"comparison_matrix.json").write_text(json.dumps([case]))
    return case, ball, swivel


def test_finite_pass_marks_unfinished_capture_pending(tmp_path):
    _, ball, _ = case_fixture(tmp_path)
    (ball/"partial.log").write_text("retain me")
    result = BATCH.analyze_suite(tmp_path, make_plots=False)
    assert result["pending_count"] == 1
    assert not result["complete"]
    assert (ball/"partial.log").read_text() == "retain me"


def test_capture_without_required_data_is_an_explicit_analysis_error(tmp_path):
    _, ball, _ = case_fixture(tmp_path)
    (ball/"run_summary.json").write_text('{"passed":false}')
    result = BATCH.analyze_suite(tmp_path, make_plots=False)
    assert result["analysis_error_count"] == 1
    assert (ball/"analysis_error.json").exists()
    assert (ball/"run_summary.json").read_text() == '{"passed":false}'


def test_completed_pair_is_cached_and_failed_trajectory_status_is_retained(tmp_path):
    _, ball, swivel = case_fixture(tmp_path)
    data = trajectory()
    data["summary"]["passed"] = False
    (ball/"run_summary.json").write_text('{"passed":false}')
    (ball/"trajectory.json").write_text(json.dumps(data))
    (swivel/"trajectory.json").write_text(json.dumps(trajectory()))
    model = Path(__file__).resolve().parents[1]/"urdf/compa_ball_caster.urdf"
    (ball/"robot.urdf").write_bytes(model.read_bytes())
    first = BATCH.analyze_suite(tmp_path, make_plots=False)
    second = BATCH.analyze_suite(tmp_path, make_plots=False)
    assert first["complete"] and second["complete"]
    assert first["cases"][0]["recomputed"]
    assert not second["cases"][0]["recomputed"]
    rows = json.loads((tmp_path/"paired_index.json").read_text())
    assert not rows[0]["ball_trajectory_passed"]
