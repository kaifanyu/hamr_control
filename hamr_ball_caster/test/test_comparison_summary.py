"""Partial scientific summaries must not imply unrecorded outcomes."""
import json
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
import summarize_caster_comparison as SUMMARY


def test_pending_case_is_explicit_and_regeneration_is_deterministic(tmp_path):
    case = {"name": "horizontal_shuttle_slow", "category": "speed_control",
            "tracking_comparison_region": "motion", "speed_m_s": .08,
            "ball_role": "nominal architecture comparison"}
    (tmp_path/"comparison_matrix.json").write_text(json.dumps([case]))
    first = SUMMARY.export_summary(tmp_path)
    text = (tmp_path/"FINDINGS.md").read_text()
    assert not first["complete"]
    assert first["analyzed_count"] == 0
    assert "Partial study" in text and "0/1 pairs" in text
    assert "**Pending:**" in text
    assert "All planned pairs analyzed" not in text
    SUMMARY.export_summary(tmp_path)
    assert (tmp_path/"FINDINGS.md").read_text() == text


def test_repeat_statistics_do_not_invent_uncertainty_from_one_run():
    one = SUMMARY.distribution([.01])
    assert one["sample_std"] is None
    several = SUMMARY.distribution([.01, .02, .03])
    assert several["mean"] == pytest.approx(.02)
    assert several["sample_std"] == pytest.approx(.01)
    assert SUMMARY.distribution([])["mean"] is None


def test_ball_repeat_of_swivel_only_parameter_is_labeled_non_equivalent():
    explanation = SUMMARY.route_explanation({"name": "swivel_drag_10x", "category": "sensitivity",
                                            "swivel_xacro_overrides": ["swivel_damping=0.15"]})
    assert "nominal parameters" in explanation
    assert "not evidence that the same parameter change" in explanation
