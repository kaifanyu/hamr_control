"""Check event semantics, source timing and missing-data behavior of pair figures."""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location("stress_pair_plot", Path(__file__).resolve().parents[1]/"scripts/plot_caster_stress_pair.py")
plot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plot)


def test_reversal_event_is_planned_direction_change_not_yaw_measurement():
    route = {"stress_reference":{"stop_schedule":[
        {"motion_end_time_s":3., "travel_yaw_rad":np.pi/2},
        {"start_time_s":3.4, "motion_end_time_s":6.4, "travel_yaw_rad":-np.pi/2},
        {"start_time_s":6.8, "travel_yaw_rad":0.}]}}
    events = plot.planned_events(route)
    assert events[0]["kind"] == "reversal"
    assert events[0]["stop_start_s"] == 3 and events[0]["restart_s"] == 3.4
    assert events[1]["kind"] == "corner"
    assert plot.planned_events({}) == []


def test_round_trip_marks_turnaround_and_reversed_corner_schedule():
    route={'stress_reference':{'stop_schedule':[
        {'motion_end_time_s':3.,'travel_yaw_rad':0.},
        {'start_time_s':3.4,'travel_yaw_rad':np.pi/2}]},
        'out_and_back':{'outbound_end_s':7.,'return_start_s':7.75,'return_end_s':14.75}}
    events=plot.planned_events(route)
    assert [e['kind'] for e in events]==['corner','reversal','corner']
    assert [e['reference_direction_change_deg'] for e in events]==pytest.approx([90,180,-90])
    assert events[2]['stop_start_s']==pytest.approx(14.75-3.4)
    assert events[2]['restart_s']==pytest.approx(14.75-3.)


def test_plot_trace_breaks_gaps_and_local_yaw_does_not_bridge_them():
    t = np.array([0., .1, .2, 3., 3.1])
    values = np.deg2rad([170., 179., -172., -10., 0.])
    unwrapped = plot.unwrap_contiguous(values, t, np.ones(5,dtype=bool), .25)
    assert np.degrees(unwrapped).tolist() == pytest.approx([170., 179., 188., -10., 0.])
    x, y = plot.gap_arrays(t, values, .25)
    assert len(x) == 6 and np.isnan(x[3]) and np.isnan(y[3])


def test_fk_time_trace_matches_analyzer_summary_and_withholds_unsynchronized_data(tmp_path):
    path = tmp_path/"robot.urdf"
    path.write_text('''<robot name="test"><link name="base"/><link name="left_caster_wheel_link"/>
      <joint name="wheel" type="continuous"><parent link="base"/><child link="left_caster_wheel_link"/>
      <origin xyz="1 0 0"/><axis xyz="0 1 0"/></joint></robot>''')
    t = np.linspace(0, 1, 101)
    samples = [{"joint_state_stamp_s":float(v+.003), "joint_positions_rad":{"wheel":0.}} for v in t]
    pos = np.zeros((101,3)); rpy = np.column_stack([t*0, t*0, t]); motion = np.ones(101,dtype=bool)
    trace, _ = plot.planar_center_traces(path, samples, t, pos, rpy, motion, .25, .03)
    values = trace["left_caster_wheel_link"]
    assert np.isnan(values[0])
    summary = plot.analysis.wheel_center_proxy(path, samples, t, pos, rpy, motion, .25, .03)
    actual = plot.analysis.statistics(values, t, motion, .25)["rms"]
    assert actual == pytest.approx(summary["wheels"]["left_caster_wheel_link"]["transverse_velocity_xy_m_s"]["rms"])
    assert actual == pytest.approx(1., abs=.001)
    missing, reason = plot.planar_center_traces(path, [{}]*101, t, pos, rpy, motion, .25, .03)
    assert not missing and "timestamps missing" in reason


def test_ball_activity_requires_complete_group_and_is_not_heading():
    rates = {"left_caster_carrier_joint":np.array([3., np.nan]),
             "right_caster_carrier_joint":np.array([4., 1.]),
             "left_caster_positive_hemisphere_joint":np.array([10., 10.])}
    groups = plot.passive_activity(rates)
    assert list(groups) == ["Carrier (2 joints)"]
    assert groups["Carrier (2 joints)"][0] == pytest.approx(np.sqrt(12.5))
    assert np.isnan(groups["Carrier (2 joints)"][1])


def test_partial_pair_produces_svg_png_and_failure_provenance(tmp_path):
    for caster in ("ball", "swivel"):
        directory = tmp_path/caster; directory.mkdir()
        config = {"caster":caster, "terrain":"test_flat", "route":{"shape":"line", "reference_duration_s":1.}}
        samples = [{"simulation_time_s":float(t+2), "trajectory_time_s":float(t), "phase":"motion",
                    "position_m":[float(t),0.,0.], "reference_m":[float(t),0.], "base_rpy_rad":[0.,0.,0.]}
                   for t in np.linspace(0, .5 if caster == "ball" else 1., 11)]
        (directory/"configuration.json").write_text(json.dumps(config))
        (directory/"trajectory.json").write_text(json.dumps({"samples":samples}))
        (directory/"run_summary.json").write_text(json.dumps({"physical_failure":"tipped_over" if caster == "ball" else None,
                                                             "trajectory_clock_complete":caster == "swivel"}))
    result = plot.plot_pair(tmp_path/"ball", tmp_path/"swivel", tmp_path/"figure")
    assert (tmp_path/"figure.svg").is_file() and (tmp_path/"figure.png").stat().st_size > 10000
    assert result["runs"]["ball"]["status"] == "physical_failure"
    assert result["runs"]["ball"]["planar_proxy_links"] == []
    assert "not contact slip" in result["scope"]
