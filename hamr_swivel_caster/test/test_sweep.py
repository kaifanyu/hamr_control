"""Check the stress experiment covers distinct paths and matched comparisons."""
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from run_caster_sweep import experiments


def test_experiments_cover_distinct_paths_and_valid_waypoint_geometry():
    cases = experiments()
    paths = {tuple(tuple(point) for point in c["points_m"]) for c in cases if c["category"] == "trajectory"}
    assert len(paths) >= 10
    assert len({c["name"] for c in cases}) == len(cases)
    for case in cases:
        assert case["points_m"][0] == [0, 0]
        assert all(math.dist(a, b) > 1e-9 for a, b in zip(case["points_m"], case["points_m"][1:]))
        assert all(math.isfinite(v) for p in case["points_m"] for v in p)


def test_parameter_comparisons_use_the_same_nominal_geometry_and_speed():
    cases = experiments()
    for case in cases:
        if case["category"] == "sensitivity":
            baseline_name = "reversal_180" if case["name"].startswith("reversal_") else "tiny_square_four_laps"
            base = next(c for c in cases if c["name"] == baseline_name)
            for key in ("points_m", "initial_yaw_deg", "speed_m_s"):
                assert case[key] == base[key]
            assert case["xacro_args"]
    heading = [c for c in cases if c["category"] == "initial_heading"]
    assert {c["initial_yaw_deg"] for c in heading} >= {0, 90, 180, 270}
    assert all(c["points_m"] == heading[0]["points_m"] for c in heading)
