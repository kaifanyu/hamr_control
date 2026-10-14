"""Verify measured-motion CAD replay without ROS, hardware, or a reference CSV."""

import ast
import importlib.util
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location("caster_replay_model_under_test", SCRIPTS / "caster_replay_model.py")
replay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(replay)
BOOT = "0123456789abcdef"
SECOND_BOOT = "fedcba9876543210"


def envelope(node, seq=0, angle=0.0, boot=BOOT, valid=True):
    flags = 1 | 2 | 16 | 64 if valid else 1
    sample = {
        "seq": seq, "t_us": 250000 + 5000 * seq,
        "raw_count": "9007199254740993" if node != 3 else None,
        "raw_spi_word": None if node != 3 else 0x61AB,
        "position14": None if node != 3 else 8619,
        "angle_rad": angle if node != 3 and valid else None,
        "absolute_angle_rad": angle if node == 3 and valid else None,
        "calibrated_alpha_rad": angle if node == 3 and valid else None,
        "velocity_rad_s": 0.2 if valid else None,
        "filtered_velocity_rad_s": 0.2 if valid else None,
        "index_event_count": 0, "index_timestamp_us": 0,
        "index_raw_count": None, "flags": flags,
    }
    return {
        "receive_epoch_ns": 1800000000000000001,
        "receive_monotonic_ns": 1800000000000000002,
        "source_ip": "192.168.1.10", "source_port": 5005,
        "packet": {
            "v": 1, "type": "samples", "node_id": node, "boot_id": boot,
            "samples": [sample],
            "diagnostics": {"acquisition_overruns": 0, "queue_drops": 0,
                            "invalid_spi_responses": 0, "index_queue_drops": 0, "radio_failures": 0},
        },
    }


def seed(model, replay_ns=1000000000):
    for node, angle in ((1, 0.4), (2, -0.2), (3, 2.5)):
        model.receive(envelope(node, angle=angle), replay_ns)


def test_initial_missing_rotation_does_not_fabricate_complete_pose():
    model = replay.ReplayModel()
    model.receive(envelope(1, angle=0.3), 1000000000)
    names, positions, status = model.snapshot(1000000000)
    assert names == positions == []
    assert not status["complete_pose_available"]
    assert status["nodes"][0]["model_position_rad"] == 0.3
    assert status["nodes"][1]["model_position_rad"] is None
    assert status["nodes"][1]["display_state"] == "unknown"
    assert "unmeasured" in replay.status_text(status)


def test_known_angles_map_to_exact_cad_joints_without_integration():
    model = replay.ReplayModel()
    seed(model)
    names, positions, status = model.snapshot(1000000000)
    assert names == [
        "caster_positive_hemisphere_joint", "caster_negative_hemisphere_joint",
        "caster_carrier_joint", "caster_positive_polar_roller_joint", "caster_negative_polar_roller_joint",
    ]
    assert positions == [0.4, -0.2, 2.5, 0.0, 0.0]
    assert all(node["display_state"] == "fresh" for node in status["nodes"])
    assert status["unmeasured_joints_held_at_nominal_zero"] == list(replay.ROLLER_JOINTS)
    assert status["accuracy"] == "unavailable_without_independent_reference"
    assert not status["mapping_calibration_current"]
    text = replay.status_text(status)
    assert "PPR unverified: H1, H2" in text
    assert "ESP clocks unsynchronized" in text


def test_side_swap_sign_and_zero_have_explicit_convention():
    model = replay.ReplayModel({
        "h1_joint": replay.SHELL_JOINTS[1], "h2_joint": replay.SHELL_JOINTS[0],
        "h1_sign": -1.0, "h1_zero_rad": 0.1,
        "h2_zero_rad": -0.4, "center_sign": -1.0, "center_zero_rad": 1.5,
        "mapping_verified": True,
    })
    seed(model)
    names, positions, status = model.snapshot(1000000000)
    assert names[:2] == list(reversed(replay.SHELL_JOINTS))
    assert positions == pytest.approx([-0.3, 0.2, -1.0, 0.0, 0.0])
    assert status["mapping_calibration_current"]


def test_center_can_replay_wrapped_alpha_with_lost_unwrap():
    model = replay.ReplayModel()
    seed(model)
    value = envelope(3, seq=1, angle=0.01)
    value["packet"]["samples"][0]["flags"] = 1 | 2 | 256
    model.receive(value, 1100000000)
    assert model.snapshot(1100000000)[1][2] == 0.01


def test_invalid_sample_holds_previous_value_and_is_visible_in_status():
    model = replay.ReplayModel()
    seed(model)
    model.receive(envelope(1, seq=1, angle=99, valid=False), 1100000000)
    _, positions, status = model.snapshot(1100000000)
    assert positions[0] == 0.4
    assert status["nodes"][0]["display_state"] == "invalid_held"
    assert "invalid_held" in replay.status_text(status)
    assert status["nodes"][0]["flags"] == 1


def test_gaps_hold_without_interpolation_and_duplicate_traffic_remains_stale():
    model = replay.ReplayModel()
    seed(model)
    model.receive(envelope(1, angle=99), 3000000000)  # duplicate, not new motion
    _, positions, status = model.snapshot(3000000000)
    assert positions[:3] == [0.4, -0.2, 2.5]
    assert status["nodes"][0]["display_state"] == "stale_held"
    model.receive(envelope(1, seq=3, angle=1.7), 3100000000)
    _, positions, status = model.snapshot(3100000000)
    assert positions[0] == 1.7
    assert status["streams"][0]["missing_samples_in_recording"] == 2
    model.receive(envelope(1, seq=2, angle=1.2), 3200000000)
    assert model.snapshot(3200000000)[1][0] == 1.7  # no backwards visual update


def test_reboot_is_a_new_segment_and_invalidates_current_zero_claim():
    model = replay.ReplayModel({"mapping_verified": True})
    seed(model)
    model.receive(envelope(1, boot=SECOND_BOOT, valid=False), 1100000000)
    _, positions, status = model.snapshot(1100000000)
    assert positions[0] == 0.4
    assert status["nodes"][0]["display_state"] == "held_after_reboot"
    assert not status["mapping_calibration_current"]
    model.receive(envelope(1, seq=1, angle=0.05, boot=SECOND_BOOT), 1200000000)
    assert model.snapshot(1200000000)[1][0] == 0.05  # no stitched turns/phase
    assert "reboot" in replay.status_text(model.snapshot(1200000000)[2])
    model.receive(envelope(1, seq=50, angle=77.0), 1300000000)
    assert model.snapshot(1300000000)[1][0] == 0.05  # old boot cannot take over


def test_backward_playback_clock_resets_values_and_sequence_history():
    model = replay.ReplayModel()
    seed(model, replay_ns=5000000000)
    names, positions, status = model.snapshot(1000000000)
    assert names == positions == []
    assert status["clock_resets"] == 1
    assert all(node["display_state"] == "unknown" for node in status["nodes"])
    seed(model, replay_ns=1100000000)
    assert model.snapshot(1100000000)[1] == [0.4, -0.2, 2.5, 0.0, 0.0]


def test_initial_clock_adjustment_without_data_is_not_reported_as_lost_segment():
    model = replay.ReplayModel()
    model.snapshot(5000000000)
    assert model.snapshot(1000000000)[2]["clock_resets"] == 0


def test_clock_pause_preserves_pose_and_simulation_age():
    model = replay.ReplayModel()
    seed(model)
    first = model.snapshot(1000000000)
    for _ in range(10):
        assert model.snapshot(1000000000) == first


@pytest.mark.parametrize("bad", ["not JSON", '{"packet":{},"packet":{}}', "[]", "{}"])
def test_malformed_envelope_is_counted_and_does_not_replace_pose(bad):
    model = replay.ReplayModel()
    seed(model)
    with pytest.raises(ValueError, match="Invalid replay envelope"):
        model.receive(bad, 1100000000)
    assert model.snapshot(1100000000)[1][0] == 0.4
    assert model.snapshot(1100000000)[2]["invalid_envelopes"] == 1


@pytest.mark.parametrize("mapping", [
    {"h1_sign": 0}, {"center_sign": float("nan")}, {"h1_zero_rad": float("inf")},
    {"h1_joint": replay.SHELL_JOINTS[1]}, {"h2_joint": "left_wheel_joint"},
    {"mapping_verified": "true"}, {"unexpected": 1},
])
def test_bad_coordinate_mappings_are_rejected(mapping):
    with pytest.raises(ValueError):
        replay.ReplayModel(mapping)


def test_launch_input_validation(tmp_path):
    bag = tmp_path / "recording"
    bag.mkdir()
    with pytest.raises(ValueError, match="metadata.yaml"):
        replay.validate_playback(str(bag), "1")
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    assert replay.validate_playback(str(bag), "0.5") == (str(bag.resolve()), 0.5)
    for rate in ("nan", "inf", "0", "-0.5", "not-a-number"):
        with pytest.raises(ValueError, match="rate"):
            replay.validate_playback(str(bag), rate)
    with pytest.raises(ValueError, match="does not exist"):
        replay.validate_playback(str(tmp_path / "missing"), 1)


def test_joint_mapping_matches_actual_cad_tree_and_opposed_shell_axes():
    model = ET.parse(PACKAGE.parent / "hamr_ball_caster/urdf/ball_caster.urdf").getroot()
    joints = {joint.get("name"): joint for joint in model.findall("joint")}
    for name in (*replay.SHELL_JOINTS, replay.CARRIER_JOINT, *replay.ROLLER_JOINTS):
        assert joints[name].get("type") == "continuous"
    positive, negative = [joints[name] for name in replay.SHELL_JOINTS]
    assert positive.find("parent").get("link") == negative.find("parent").get("link") == "caster_carrier_link"
    a = [float(v) for v in positive.find("axis").get("xyz").split()]
    b = [float(v) for v in negative.find("axis").get("xyz").split()]
    assert a == pytest.approx([-v for v in b])
    assert math.sqrt(sum(v * v for v in a)) == pytest.approx(1.0)
    assert joints[replay.CARRIER_JOINT].find("axis").get("xyz") == "0 -1 0"


def test_launch_whitelists_raw_packets_and_remaps_away_from_live_encoder_topics():
    source = (PACKAGE / "launch/caster_encoder_replay.launch.py").read_text()
    tree = ast.parse(source)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name) and node.func.id == "ExecuteProcess"]
    assert len(calls) == 1
    cmd = next(keyword.value for keyword in calls[0].keywords if keyword.arg == "cmd")
    tokens = [value.value for value in cmd.elts if isinstance(value, ast.Constant)]
    start, end = tokens.index("--topics"), tokens.index("--remap")
    assert tokens[start + 1:end] == ["/caster_encoder/packets"]
    assert tokens[end + 1:] == ["/caster_encoder/packets:=/caster_replay/packets"]
    assert "--clock" in tokens and "--delay" in tokens
    assert '"/tf", "/caster_replay/tf"' in source
    assert '"/tf_static", "/caster_replay/tf_static"' in source
    node_calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                  and isinstance(node.func, ast.Name) and node.func.id == "Node"]
    executables = [keyword.value.value for node in node_calls for keyword in node.keywords
                   if keyword.arg == "executable" and isinstance(keyword.value, ast.Constant)]
    assert "caster_encoder_bridge" not in executables
    assert set(executables) == {"replay_caster_encoder_joints", "robot_state_publisher", "rviz2"}


def test_mapper_uses_steady_render_timer_so_final_packet_is_not_lost_at_eof():
    source = (SCRIPTS / "replay_caster_encoder_joints").read_text()
    assert "ClockType.STEADY_TIME" in source
    assert "clock=self.render_clock" in source


def test_rviz_markers_use_ros2_topic_property_for_visible_status_and_phase():
    import yaml

    config = yaml.safe_load((PACKAGE / "config/caster_encoder_replay.rviz").read_text())
    displays = {item["Class"]: item for item in config["Visualization Manager"]["Displays"]}
    # "Marker Topic" is ignored by ROS 2 RViz, silently leaving these displays
    # unsubscribed even though the model and Global Status both look healthy.
    for kind, topic in (("Marker", "/caster_replay/status_marker"),
                        ("MarkerArray", "/caster_replay/phase_markers")):
        display = displays[f"rviz_default_plugins/{kind}"]
        assert display["Enabled"] is True
        assert display["Topic"]["Value"] == topic
        assert display["Topic"]["Durability Policy"] == "Transient Local"
