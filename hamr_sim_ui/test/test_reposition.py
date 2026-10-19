"""Placement geometry, pause preservation and failure boundaries."""
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hamr_sim_ui.session import SimulationSession, Terrain

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def live(tmp_path, monkeypatch):
    session = SimulationSession(ROOT, tmp_path)
    session.terrain = session.terrains["flat"]
    session.world_name = "test"
    session.run_dir = tmp_path
    session.process = type("Process", (), {"poll": lambda self: None})()
    monkeypatch.setattr(session, "_read_paused", lambda: session.paused)
    return session


def feedback():
    return {"feedback_ready": True, "position": (0., 0.), "yaw": .8,
            "velocity": (0., 0., 0.), "sim_time": 12., "odom_time": 12.,
            "feedback_age_s": .01, "z": 0., "roll": 0., "pitch": 0.}


def test_new_sessions_disable_camera_rendering_by_default(tmp_path, monkeypatch):
    session = SimulationSession(ROOT, tmp_path)
    assert not session.cameras
    commands = []
    monkeypatch.setattr(session, "_spawn", lambda command, output: commands.append(command))
    session.start("flat", gui=False)
    assert not session.cameras
    assert len(commands) == 1
    assert "closeup_camera:=false" in commands[0]


@pytest.mark.parametrize("x,y,yaw", [(math.nan, 0, 0), (0, math.inf, 0),
                                    (0, 0, math.nan), (3.01, 0, 0), (0, -3.01, 0)])
def test_invalid_placement_has_no_simulation_effect(live, monkeypatch, x, y, yaw):
    monkeypatch.setattr(live, "_read_paused", lambda: pytest.fail("must validate first"))
    with pytest.raises(ValueError):
        live.reposition(x, y, yaw, feedback=feedback)


def test_height_clears_peak_under_wheels_not_only_center(live, tmp_path):
    coordinates = np.linspace(-3, 3, 121)
    heights = np.zeros((121, 121))
    heights[60, 69] = .3  # Narrow peak 0.45 m away from the selected root.
    path = tmp_path / "height.npz"
    np.savez(path, x=coordinates, y=coordinates, z=heights)
    live.terrain = Terrain("synthetic", "Synthetic", tmp_path / "world.sdf",
                           (-3., 3., -3., 3.), (0., 0., 0., 0.), path)
    assert live.placement_pose(0, 0, 0)["z"] == pytest.approx(.34)
    assert live.placement_pose(-2, -2, 0)["z"] == pytest.approx(.04)


@pytest.mark.parametrize("paused", [False, True])
def test_reposition_restores_pause_confirms_target_and_records_event(live, monkeypatch, paused):
    live.paused = paused
    transitions, commands = [], []
    def pause(value):
        transitions.append(value)
        live.paused = value
    monkeypatch.setattr(live, "set_paused", pause)
    monkeypatch.setattr(live, "_wait_stopped", lambda *args: feedback())
    monkeypatch.setattr(live, "_model_pose", lambda: (1., -.5, .04, 0., 0., .8))
    def run(command, **kwargs):
        commands.append(command)
        return subprocess.CompletedProcess(command, 0, "data: true", "")
    monkeypatch.setattr(subprocess, "run", run)
    result = live.reposition(1., -.5, feedback=feedback)
    assert result == {"x": 1., "y": -.5, "z": .04, "yaw": .8}
    assert live.paused == paused
    assert transitions == ([False, True] if paused else [True, False])
    assert commands[0][4:8] == ["--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean"]
    assert commands[0][3] == "/world/test/set_pose/blocking"
    assert 'name: "compa"' in commands[0][-1]
    event = json.loads((live.run_dir / "events.jsonl").read_text())
    assert event["status"] == "placed"
    assert event["was_paused"] == paused


def test_unsettled_car_is_never_moved_and_pause_restored(live, monkeypatch):
    live.paused = True
    monkeypatch.setattr(live, "set_paused", lambda value: setattr(live, "paused", value))
    def unsettled(*args):
        raise RuntimeError("Car did not stop")
    monkeypatch.setattr(live, "_wait_stopped", unsettled)
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: pytest.fail("must not teleport"))
    with pytest.raises(RuntimeError, match="did not stop"):
        live.reposition(1, 1, 0, feedback=feedback)
    assert live.paused
    assert json.loads((live.run_dir / "events.jsonl").read_text())["status"] == "failed"


def test_declined_service_is_not_reported_as_success(live, monkeypatch):
    monkeypatch.setattr(live, "set_paused", lambda value: setattr(live, "paused", value))
    monkeypatch.setattr(live, "_wait_stopped", lambda *args: feedback())
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 0, "data: false", ""))
    with pytest.raises(RuntimeError, match="acknowledge"):
        live.reposition(1, 1, 0, feedback=feedback)
    assert not live.paused


def test_feedback_must_advance_in_simulation_time(live, monkeypatch):
    # A paused/stale zero-velocity sample must never pass settling by wall time.
    with pytest.raises(RuntimeError, match="did not stop"):
        live._wait_stopped(feedback, .06)


@pytest.mark.parametrize("axis", ["z", "roll", "pitch"])
def test_falling_or_tilting_car_does_not_pass_planar_settle_check(live, monkeypatch, axis):
    elapsed = [0.]
    def advance(duration):
        elapsed[0] += duration
    monkeypatch.setattr("hamr_sim_ui.session.time.sleep", advance)
    monkeypatch.setattr("hamr_sim_ui.session.time.monotonic", lambda: elapsed[0])
    def moving():
        state = feedback()
        state["odom_time"] += elapsed[0]
        state[axis] = elapsed[0]
        return state
    with pytest.raises(RuntimeError, match="did not stop"):
        live._wait_stopped(moving, .5)


def test_fresh_stationary_samples_pass_settle_check(live, monkeypatch):
    elapsed = [0.]
    monkeypatch.setattr("hamr_sim_ui.session.time.sleep", lambda duration: elapsed.__setitem__(0, elapsed[0]+duration))
    monkeypatch.setattr("hamr_sim_ui.session.time.monotonic", lambda: elapsed[0])
    def stationary():
        state = feedback()
        state["odom_time"] += elapsed[0]
        return state
    assert live._wait_stopped(stationary, .5)["odom_time"] >= 12.2


def test_placement_disallowed_during_replay(live):
    live.replay = True
    with pytest.raises(RuntimeError, match="live simulation"):
        live.reposition(1, 1, 0, feedback=feedback)


def test_actual_pause_state_includes_gazebo_viewport_controls(live, monkeypatch):
    monkeypatch.undo()
    live.paused = False
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 0,
        '{"simTime":{"sec":"1"},"paused":true}', ""))
    assert live._read_paused()


def test_model_pose_parsing(live, monkeypatch):
    output = "Model: [12]\n  - Name: compa\n  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n    [1.000000 -0.500000 0.040000]\n    [0.000000 -0.000000 0.800000]\n"
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 0, output, ""))
    assert live._model_pose() == (1., -.5, .04, 0., 0., .8)
