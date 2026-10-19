"""Motion configuration and repeatable-run recording/failure contracts."""
import json
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hamr_sim_ui.controller import LiveController, MotionLimits, MotionPlan, limits_from_args
from hamr_sim_ui.presets import preset_points
from hamr_sim_ui import runner

ROOT = Path(__file__).resolve().parents[2]


def test_raising_requested_cap_does_not_silently_raise_physics_or_planning_limits():
    plan = MotionPlan([(1.5, 0.)], .6, (0., 0.), 0., ROOT, MotionLimits(max_speed=.8))
    assert plan.summary()["planned_peak_speed_m_s"] < .27
    assert plan.limits.max_wheel_rate == 6.
    assert plan.limits.planning_wheel_rate == 3.9


def test_explicit_faster_limits_change_nominal_plan_and_report_actual_peak():
    args = runner.parser().parse_args(["--speed", ".6", "--max-speed", ".8", "--wheel-rate", "12",
        "--planning-wheel-rate", "10", "--wheel-acceleration", "60", "--planning-wheel-acceleration", "40",
        "--acceleration", ".5"])
    plan = MotionPlan([(1.5, 0.)], args.speed, (0., 0.), 0., ROOT, limits_from_args(args))
    assert plan.summary()["planned_peak_speed_m_s"] == pytest.approx(.6)
    segment = plan.route.segments[0]
    assert segment["wheel_speed_bound_rad_s"] <= 10.
    assert segment["wheel_acceleration_bound_rad_s2"] <= 40.


@pytest.mark.parametrize("values", [{"max_speed": math.nan}, {"max_speed": .01},
    {"max_wheel_rate": 3.}, {"planning_wheel_acceleration": 16.},
    {"max_translational_acceleration": 0.}])
def test_reject_incoherent_or_nonfinite_limits(values):
    with pytest.raises(ValueError):
        MotionLimits(**values)


def test_changing_limits_stops_existing_manual_motion():
    controller = LiveController.__new__(LiveController)
    controller._lock = threading.RLock()
    controller._manual = (.2, 0., 0.)
    controller._plan = object()
    commands = []
    controller._command = commands.append
    controller.configure_limits(MotionLimits(max_speed=.8))
    assert controller._manual is None and controller._plan is None
    assert controller.limits.max_speed == .8
    assert commands == [[0.]*5]


@pytest.mark.parametrize("wheel_limit,expected", [(12., .8/.1075), (4., 4.)])
def test_manual_tick_uses_configured_translation_and_wheel_command_caps(wheel_limit, expected):
    controller = LiveController.__new__(LiveController)
    controller._lock = threading.RLock()
    controller.limits = MotionLimits(max_speed=.8, max_wheel_rate=wheel_limit,
                                    max_wheel_acceleration=100.)
    controller._manual = (.8, 0., 0.)
    controller._manual_wall = controller._odom_wall = time.monotonic()
    controller._plan = None
    controller._clock, controller._previous_clock = 1., .9
    pose = SimpleNamespace(position=SimpleNamespace(x=0., y=0., z=0.),
                           orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    controller._odom = SimpleNamespace(pose=SimpleNamespace(pose=pose),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
    controller._joints = SimpleNamespace(name=[], position=[])
    controller._wheels = (0., 0.)
    commands = []
    controller._command = commands.append
    controller._tick()
    assert commands[0][:2] == pytest.approx([expected, expected])


def test_preset_is_relative_to_current_position_and_heading():
    points = preset_points("straight", (1., -2.), math.pi/2, scale=.5)
    assert points[0] == (1., -2.)
    assert points[1] == pytest.approx((1., -1.25))


def test_route_file_has_explicit_world_coordinates(tmp_path):
    path = tmp_path / "route.json"
    path.write_text(json.dumps({"points": [[1, 2], [2, 3]]}))
    assert runner.load_route(path) == [(1., 2.), (2., 3.)]
    path.write_text('{"points": [[1, 2], [1, 2]]}')
    with pytest.raises(ValueError, match="differ"):
        runner.load_route(path)


def test_desktop_route_export_checks_map_and_cli_speed_overrides_saved_speed(tmp_path):
    path = tmp_path / "route.json"
    path.write_text(json.dumps({"terrain": "flat", "speed_m_s": .3, "points_m": [[0, 0], [1, 0]]}))
    assert runner.load_route(path, "flat") == [(0., 0.), (1., 0.)]
    with pytest.raises(ValueError, match="saved for terrain"):
        runner.load_route(path, "road_bumps")
    args = runner.parser().parse_args(["--route-json", str(path), "--terrain", "flat", "--speed", ".15"])
    runner.validate_arguments(args)
    assert args.speed == .15


def test_video_requires_actual_camera_sensors():
    with pytest.raises(ValueError, match="--cameras on"):
        runner.validate_arguments(runner.parser().parse_args(["--video"]))


@pytest.mark.parametrize("failure", [None, "controller", "write", "missing_packets"])
def test_runner_records_before_driving_and_finalizes_on_success_or_error(tmp_path, monkeypatch, failure):
    calls = []
    run_dir = tmp_path / "session"
    (run_dir / "gazebo_log").mkdir(parents=True)
    (run_dir / "gazebo_log/state.tlog").write_bytes(b"scene")
    class Telemetry:
        node = object()
        def __init__(self, **kwargs):
            assert kwargs == {"caster": "ball", "cameras": False, "preview_cameras": ()}
            self.recording = False
        def snapshot(self, **kwargs):
            assert kwargs == {"include_history": False, "include_trail": False}
            return {"odom_received": True, "joints_received": True, "sim_time": 10.,
                    "last_error": "write failed" if failure == "write" and self.recording else None}
        def start_recording(self, path, **kwargs):
            self.recording = True
            calls.append("record")
        def stop_recording(self):
            calls.append("finalize")
            counts = dict.fromkeys(("/clock", "/compa/odom", "/joint_states", "/caster_encoder/packets"), 10)
            if failure == "missing_packets":
                counts.pop("/caster_encoder/packets")
            return {"recording_complete": True, "message_counts": counts}
        def close(self):
            calls.append("telemetry_close")
    class Controller:
        def __init__(self, *args, **kwargs):
            pass
        def snapshot(self):
            return {"position": (0., 0.), "yaw": 0., "sim_time": 1., "feedback_ready": True,
                    "plan_summary": {"planned_peak_speed_m_s": .2, "nominal_duration_s": 2.},
                    "state": "error" if failure == "controller" else "complete", "error": "tipped",
                    "segment": 0}
        def set_route(self, points, speed):
            calls.append("drive")
            return points
        def halt(self):
            calls.append("halt")
        def close(self):
            calls.append("controller_close")
    session = SimpleNamespace(start=lambda *args, **kwargs: run_dir, poll=lambda: None,
        terrain=SimpleNamespace(bounds=(-3., 3., -3., 3.)), gazebo_log=run_dir / "gazebo_log",
        stop=lambda: calls.append("session_stop"))
    monkeypatch.setattr(runner, "RosTelemetry", Telemetry)
    monkeypatch.setattr(runner, "LiveController", Controller)
    args = runner.parser().parse_args([])
    result = runner.run_simulation(args, session, MotionLimits())
    assert result == (0 if failure is None else 1)
    assert calls.index("record") < calls.index("drive")
    assert calls[-3:] == ["finalize", "telemetry_close", "session_stop"]
    info = json.loads((run_dir / "run.json").read_text())
    assert info["status"] == ("complete" if failure is None else "failed")
    assert info["scene_recording_complete"]
