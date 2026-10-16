"""Check route/world agreement and measured passive-joint evidence exports."""
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from prepare_swivel_world import prepare_world
from run_swivel_caster import export_caster_measurements, heading_measurements


def test_generated_environment_preserves_contacts_and_marks_exact_route(tmp_path):
    config = yaml.safe_load((PACKAGE / "config/waypoint_sim.yaml").read_text())
    output = tmp_path / "world.sdf"
    preparation = prepare_world(PACKAGE / "worlds/swivel_caster_test.sdf", output,
                                config["points_m"], .0005, .15)
    world = ET.parse(output).getroot().find("world")
    assert world.get("name") == "swivel_caster_test"
    assert float(world.findtext("physics/max_step_size")) == .0005
    assert float(world.findtext("physics/real_time_factor")) == .15
    assert world.find("./model[@name='ground_plane']/link/collision") is not None
    markings = world.find("./model[@name='trajectory_markings']")
    assert not markings.findall(".//collision")
    assert len([v for v in markings.findall(".//visual") if v.get("name").startswith("route_")]) == 6
    assert preparation["route_length_m"] == pytest.approx(4.4)
    assert world.findtext("./model[@name='overview_camera']/link/sensor/topic") == "/hamr/recording_camera/image"


def test_feedback_export_requires_both_steering_and_rolling(tmp_path):
    names = [f"{side}_caster_{kind}_joint" for side in ("left", "right") for kind in ("swivel", "wheel")]
    samples = [{"simulation_time_s": float(i), "trajectory_time_s": float(i), "phase": "motion",
                "joint_positions_rad": {name: i*(.5 if "swivel" in name else 2.) for name in names},
                "joint_velocities_rad_s": {name: .5 for name in names}} for i in range(3)]
    summary = export_caster_measurements({"samples": samples}, tmp_path)
    assert summary["passed"]
    assert summary["joints"]["right_caster_wheel_joint"]["position_span_rad"] == 4.
    assert len((tmp_path / "caster_joints.csv").read_text().splitlines()) == 4
    for sample in samples:
        sample["joint_positions_rad"]["right_caster_swivel_joint"] = 0.
    assert not export_caster_measurements({"samples": samples}, tmp_path)["passed"]
    assert export_caster_measurements({"samples": samples}, tmp_path, require_swivel=False)["passed"]


def test_motion_coverage_accounts_for_waypoint_settling():
    # The source stamp minus trajectory time is the absolute motion start.
    # The measured reference duration includes intermediate settling holds.
    location = PACKAGE.parent / "hamr_ball_caster/scripts/record_waypoint_run.py"
    spec = importlib.util.spec_from_file_location("record_shared", location)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    trajectory = {"samples": [{"phase": "motion", "odometry_stamp_s": 10.2, "trajectory_time_s": .2}],
                  "summary": {"reference_duration_s": 2.}}
    recording = {"encoded_source_timestamps_s": [9.9+i*.1 for i in range(24)]}
    assert module.continuous_video_quality(trajectory, recording)["passed"]
    recording["encoded_source_timestamps_s"] = [9.9, 10., 11.9, 12.]
    assert not module.continuous_video_quality(trajectory, recording)["passed"]


def test_simple_profile_matches_original_geometry_speed_and_timing():
    import math
    config = yaml.safe_load((PACKAGE / "config/simple_waypoint_sim.yaml").read_text())
    original = yaml.safe_load((PACKAGE.parent / "hamr_ball_caster/config/simple_waypoint_sim.yaml").read_text())
    assert config["points_m"] == original["points_m"]
    assert config["speed_m_s"] == original["speed_m_s"] == .25
    length = sum(math.dist(a,b) for a,b in zip(config["points_m"], config["points_m"][1:]))
    assert length == 13.
    assert length/config["speed_m_s"] == 52.


@pytest.mark.parametrize("initial_degrees", [0., 180.])
def test_heading_evidence_requires_requested_spawn_and_alignment_while_moving(initial_degrees):
    import math
    samples = []
    for i, degrees in enumerate([initial_degrees, (initial_degrees+90)/2, 90.]):
        samples.append({"base_rpy_rad": [0.,0.,math.radians(degrees)],
                        "phase": "startup_hold" if i == 0 else "motion", "segment": 0,
                        "trajectory_time_s": float(i), "position_m": [0.,float(i),0.],
                        "estimated_world_velocity_m_s": [0.,.25]})
    report = {"configuration": {"points_m": [[0,0],[0,2]], "expect_first_leg_alignment": True},
              "samples": samples}
    result = heading_measurements(report, initial_degrees)
    assert result["passed"] and result["first_leg_translation_m"] > 0
    assert result["time_to_within_5deg_of_first_leg_s"] == 2.
    assert not heading_measurements(report, initial_degrees+180)["passed"]
    samples[-1]["base_rpy_rad"][2] = math.pi
    assert not heading_measurements(report, initial_degrees)["passed"]


def test_legacy_camera_coverage_uses_controller_reference_clock():
    location = PACKAGE.parent / "hamr_ball_caster/scripts/record_waypoint_run.py"
    spec = importlib.util.spec_from_file_location("record_clock", location)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    trajectory = {"trajectory_profile": "legacy",
                  "samples": [{"phase": "motion", "simulation_time_s": 10.2,
                               "odometry_stamp_s": 9.7, "trajectory_time_s": .2}],
                  "summary": {"reference_duration_s": 2.}}
    recording = {"encoded_source_timestamps_s": [10.+i*.1 for i in range(21)]}
    assert module.continuous_video_quality(trajectory, recording)["motion_interval_sim_s"] == [10.,12.]
    assert module.continuous_video_quality(trajectory, recording)["passed"]
