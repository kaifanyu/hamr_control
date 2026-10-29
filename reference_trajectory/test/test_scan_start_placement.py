"""Start the scan from the reported offset without a reference jump."""

from pathlib import Path

import pytest

from reference_trajectory.continuous_execution import ContinuousExecution, Feedback, build_plan
from reference_trajectory.continuous_waypoint_node import load_config


def test_actual_scan_accepts_reported_position_and_translates_the_whole_route():
    cfg = load_config(Path(__file__).parents[1] / 'config/rgbd_vicon_scan.yaml')
    plan = build_plan(cfg)
    session = ContinuousExecution(cfg, plan)
    x0, y0 = .025, -.074
    for i in range(40):
        session.observe(Feedback(x0, y0, 0., 9000. + i * .01, i * .01, 'world'))
    assert session.start(.39, subscribers=1, publishers=1)[0]
    assert session.translation == pytest.approx((x0, y0))
    # Startup hold is exactly at the measured position, with no motion to origin.
    assert session.step(.39) == pytest.approx((x0, y0, 0., 0., 0., 0.))
    assert plan.points[-1][0] + session.translation[0] == pytest.approx(x0)
    assert plan.points[-1][1] + session.translation[1] == pytest.approx(y0)
    assert max(p[1] for p in plan.points) + y0 == pytest.approx(4.926)
    assert min(p[0] for p in plan.points) + x0 == pytest.approx(-1.175)
    assert max(p[0] for p in plan.points) + x0 == pytest.approx(1.225)
    assert cfg['odom_timeout_s'] == .20
    assert cfg['max_publish_gap_s'] == .08
    assert cfg['start_yaw_tolerance_rad'] == .15
