"""Physics/measurement checks for partial paired caster analysis."""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location("terrain_stress_analysis", Path(__file__).resolve().parents[1]/"scripts/analyze_caster_stress.py")
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def samples(count=21, stationary=False):
    result = []
    for t in np.linspace(0, 2, count):
        result.append({"simulation_time_s": float(t), "trajectory_time_s": float(t), "phase": "motion",
                       "position_m": [0 if stationary else float(t), 0., .1], "reference_m": [float(t), 0.],
                       "reference_velocity_m_s": [1., 0.], "base_rpy_rad": [0., 0., 0.]})
    return result


def test_stationary_robot_has_error_and_low_speed_despite_reference_clock_progress():
    s = samples(stationary=True); t = np.array([v["simulation_time_s"] for v in s])
    m, _ = analysis.compute_metrics(s, t, {}, .25)
    assert m["progress"]["observed_xy_distance_m"] == 0
    assert m["progress"]["low_progress_fraction"] == 1
    assert m["progress"]["longest_low_speed_episode_s"] == pytest.approx(2)
    assert m["tracking_xy_error_m"]["rms"] > 1


def test_dwell_does_not_count_as_stall():
    s = samples(stationary=True)
    for v in s: v["reference_velocity_m_s"] = [0., 0.]
    m, _ = analysis.compute_metrics(s, np.linspace(0, 2, 21), {}, .25)
    assert m["progress"]["demanded_motion_observed_s"] == 0
    assert m["progress"]["low_progress_fraction"] is None


def test_time_weights_do_not_cross_missing_data_or_phase_gaps():
    t = np.array([0., .1, .2, 5., 5.1])
    w = analysis.weights(t, np.ones(5, dtype=bool), .25)
    assert w.sum() == pytest.approx(.3)
    w = analysis.weights(np.array([0., .1, .2]), np.array([True, False, True]), .25)
    assert w.sum() == 0


def test_backward_clock_is_reported_and_truncated():
    t, s, q = analysis.normalize_samples([{"stamp_s": 1.}, {"stamp_s": 2.}, {"stamp_s": 2.}, {"stamp_s": .1}])
    assert t.tolist() == [1., 2.]
    assert q["duplicate_timestamps"] == 1 and q["clock_reversal_truncated"]


def test_missing_run_channels_remain_null(tmp_path):
    (tmp_path/"run_summary.json").write_text(json.dumps({"caster": "ball", "physical_failure": "tipped_over"}))
    result, _ = analysis.analyze_run(tmp_path)
    assert result["status"] == "physical_failure"
    assert result["metrics"]["tracking_xy_error_m"]["rms"] is None
    assert not result["wheel_center_proxy"]["available"]


def test_fk_proxy_uses_local_velocity_during_yaw_not_reference_direction(tmp_path):
    p = tmp_path/"robot.urdf"
    p.write_text('''<robot name="test"><link name="base"/><link name="left_wheel_link"/>
      <joint name="wheel" type="continuous"><parent link="base"/><child link="left_wheel_link"/>
      <origin xyz="1 0 0"/><axis xyz="0 1 0"/></joint></robot>''')
    t = np.linspace(0, 1, 101)
    s = [{"joint_state_stamp_s":float(v), "joint_positions_rad":{"wheel":0.}} for v in t]
    pos = np.zeros((101, 3)); rpy = np.column_stack([t*0, t*0, t])
    result = analysis.wheel_center_proxy(p, s, t, pos, rpy, np.ones(101,dtype=bool), .25, .03)
    metric = result["wheels"]["left_wheel_link"]["transverse_velocity_xy_m_s"]
    assert metric["rms"] == pytest.approx(1., abs=.001)
    assert result["wheels"]["left_wheel_link"]["planar_axis_over_15_deg_fraction"] == 1
    assert result["wheels"]["left_wheel_link"]["planar_axis_over_15_deg_longest_episode_s"] == pytest.approx(1)
    gapped = np.ones(101, dtype=bool); gapped[40:61] = False
    interrupted = analysis.wheel_center_proxy(p, s, t, pos, rpy, gapped, .25, .03)
    assert interrupted["wheels"]["left_wheel_link"]["planar_axis_over_15_deg_longest_episode_s"] == pytest.approx(.39)
    with pytest.raises(ValueError, match="timestamps missing"):
        analysis.wheel_center_proxy(p, [{}]*101, t, pos, rpy, np.ones(101,dtype=bool), .25, .03)


def test_planar_alignment_fraction_weights_time_not_sample_count(tmp_path):
    p = tmp_path/"robot.urdf"
    p.write_text('''<robot name="test"><link name="base"/><link name="left_caster_wheel_link"/>
      <joint name="wheel" type="continuous"><parent link="base"/><child link="left_caster_wheel_link"/>
      <axis xyz="0 1 0"/></joint></robot>''')
    t = np.linspace(0, 1, 101)
    pos = np.column_stack([np.minimum(t,.5), np.maximum(t-.5,0), 0*t])
    s = [{"joint_state_stamp_s":float(v), "joint_positions_rad":{"wheel":0.}} for v in t]
    result = analysis.wheel_center_proxy(p, s, t, pos, np.zeros((101,3)), np.ones(101,dtype=bool), .25, .03)
    entry = result["wheels"]["left_caster_wheel_link"]
    assert entry["planar_axis_over_15_deg_fraction"] == pytest.approx(.505)
    assert entry["planar_axis_over_15_deg_longest_episode_s"] == pytest.approx(.5)


def test_case_aggregation_weights_cases_not_trials_or_frames():
    pairs = [{"terrain":"a", "shape":"line", "paired":True, "fully_verified_pair":True, "ball_status":"complete", "swivel_status":"complete",
              "common_horizon_ball_minus_swivel":{"xy_rmse_m":v}} for v in [1., 3.]]
    pairs.append({"terrain":"b", "shape":"line", "paired":True, "fully_verified_pair":True, "ball_status":"physical_failure", "swivel_status":"complete",
                  "common_horizon_ball_minus_swivel":{"xy_rmse_m":10.}})
    result = analysis.case_aggregate(pairs)
    assert result["common_horizon_delta_by_metric"]["xy_rmse_m"]["mean"] == 6
    assert result["status_counts"]["ball"]["physical_failure"] == 1
    pairs.append({"terrain":"unverified", "shape":"line", "paired":True, "fully_verified_pair":False,
                  "common_horizon_ball_minus_swivel":{"xy_rmse_m":100.}})
    assert analysis.case_aggregate(pairs)["common_horizon_delta_by_metric"]["xy_rmse_m"]["mean"] == 6


def test_nested_limits_are_fractional_and_missing_is_not_false():
    s = samples()
    for index, sample in enumerate(s):
        sample["command_limits"] = {"translation":False, "wheel_rate":False,
                                    "wheel_acceleration":index >= 10}
    metrics, _ = analysis.compute_metrics(s, np.linspace(0, 2, 21), {}, .25)
    flat = analysis.flat_metrics(metrics)
    assert flat["translation_limited_fraction"] == 0
    assert flat["wheel_acceleration_limited_fraction"] == pytest.approx(.525)
    assert flat["any_limited_fraction"] == pytest.approx(.525)
    missing, _ = analysis.compute_metrics(samples(), np.linspace(0, 2, 21), {}, .25)
    assert analysis.flat_metrics(missing)["any_limited_fraction"] is None


def test_yaw_gimbal_boundary_is_time_weighted_separate_from_drive_flags():
    t = np.array([0., .1, .2, .4, .6, .7])
    s = samples(count=6)
    for index, sample in enumerate(s):
        sample["phase"] = "settling" if index == 0 else "final_hold" if index == 5 else "motion"
        sample["commands_rad_s"] = [0., 0., 0., 0., [2.5, 2.5, -2.5, 0., 0., -2.5][index]]
        sample["command_limits"] = {"translation":False, "wheel_rate":False, "wheel_acceleration":False}
    metrics, _ = analysis.compute_metrics(s, t, {}, .25)
    flat = analysis.flat_metrics(metrics)
    assert flat["any_limited_fraction"] == 0
    assert flat["yaw_gimbal_at_rate_boundary_fraction"] == pytest.approx(.4)
    assert metrics["command_saturation"]["yaw_gimbal_boundary_observed_duration_s"] == pytest.approx(.5)
    assert "cannot establish torque saturation" in metrics["command_saturation"]["yaw_gimbal_boundary_scope"]
    assert "does not cover all actuators" in metrics["command_saturation"]["drive_any_limited_scope"]


def test_missing_yaw_gimbal_command_is_unavailable_and_outside_boundary_is_not_at_it():
    s = samples()
    for sample in s:
        sample["commands_rad_s"] = [0., 0.]
    missing, _ = analysis.compute_metrics(s, np.linspace(0, 2, 21), {}, .25)
    assert analysis.flat_metrics(missing)["yaw_gimbal_at_rate_boundary_fraction"] is None
    for sample in s:
        sample["commands_rad_s"] = [0., 0., 0., 0., 3.]
    outside, _ = analysis.compute_metrics(s, np.linspace(0, 2, 21), {}, .25)
    assert analysis.flat_metrics(outside)["yaw_gimbal_at_rate_boundary_fraction"] == 0


def test_yaw_wraps_describe_full_turn_without_inventing_gap_rotation():
    t = np.linspace(0, 2, 21)
    yaw = np.arctan2(np.sin(t*np.pi), np.cos(t*np.pi))
    result = analysis.yaw_motion(yaw, t, np.ones(21, dtype=bool), .25)
    assert result["observed_net_rotation_deg"] == pytest.approx(360)
    assert result["maximum_contiguous_span_deg"] == pytest.approx(360)
    t[11:] += 10
    gapped = analysis.yaw_motion(yaw, t, np.ones(21, dtype=bool), .25)
    assert gapped["observed_net_rotation_deg"] == pytest.approx(342)
    assert gapped["maximum_contiguous_span_deg"] == pytest.approx(180)
    assert gapped["contiguous_segments"] == 2


def test_missing_attitude_does_not_erase_valid_xy_error():
    s = samples(stationary=True)
    for sample in s: del sample["base_rpy_rad"]
    metric, _ = analysis.compute_metrics(s, np.linspace(0, 2, 21), {}, .25)
    assert metric["tracking_xy_error_m"]["rms"] > 1
    assert metric["body_roll_deg"]["rms"] is None


def test_failed_pair_keeps_status_and_compares_only_common_horizon(tmp_path):
    runs, contexts = [], []
    for caster in ("ball", "swivel"):
        directory = tmp_path/caster
        directory.mkdir()
        s = samples(stationary=caster == "ball")
        if caster == "ball": s = s[:11]
        for sample in s:
            sample["command_limits"] = {"translation":False, "wheel_rate":False, "wheel_acceleration":caster == "ball"}
        config = {"caster":caster, "terrain":"flat", "route":{"shape":"line"}, "controller":{}, "spawn":{},
                  "grid_sha256":"grid", "source_world_sha256":"world", "runner_sha256":"runner", "paths_source_sha256":"paths"}
        (directory/"configuration.json").write_text(json.dumps(config))
        (directory/"trajectory.json").write_text(json.dumps({"samples":s}))
        (directory/"run_summary.json").write_text(json.dumps({"physical_failure":"tip" if caster == "ball" else None,
                                                             "trajectory_clock_complete":caster == "swivel"}))
        (directory/"world.sdf").write_text('<sdf><world><physics><max_step_size>.001</max_step_size></physics></world></sdf>')
        run, context = analysis.analyze_run(directory)
        runs.append(run); contexts.append(context)
    pair = analysis.paired_results(runs, contexts, .25, .03)[0]
    assert pair["paired"] and pair["fully_verified_pair"]
    assert pair["ball_status"] == "physical_failure"
    assert pair["common_observed_horizon_s"] == 1
    assert pair["common_horizon_metrics"]["swivel"]["observed_motion_s"] == pytest.approx(1)
    assert pair["common_horizon_ball_minus_swivel"]["wheel_acceleration_limited_fraction"] == 1
    assert pair["common_horizon_ball_minus_swivel"]["xy_rmse_m"] > .5
