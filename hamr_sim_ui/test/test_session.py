"""Planning, process boundaries and command safety without a Gazebo server."""
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hamr_sim_ui.controller import LiveController, MotionPlan, inverse_drive
from hamr_sim_ui.session import SimulationSession, discover_terrains, stop_process_group, _process_group_alive

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("caster", ["ball", "swivel"])
@pytest.mark.parametrize("cameras", [True, False])
def test_camera_and_caster_configuration_reaches_world_and_processes(tmp_path, monkeypatch, caster, cameras):
    session = SimulationSession(ROOT, tmp_path)
    commands = []
    monkeypatch.setattr(session, "_spawn", lambda command, output: commands.append(command))
    run = session.start("flat", gui=False, caster=caster, cameras=cameras)
    assert f"caster:={caster}" in commands[0]
    assert f"closeup_camera:={str(cameras).lower()}" in commands[0]
    assert len(commands) == (2 if cameras else 1)
    world = ET.parse(run / "world.sdf").getroot().find("world")
    assert bool(world.findall(".//sensor")) == cameras
    assert (world.find("plugin[@name='gz::sim::systems::Sensors']") is not None) == cameras
    if cameras:
        assert f"/hamr/{caster}_caster/closeup/image@sensor_msgs/msg/Image[gz.msgs.Image" in commands[1]
    metadata = json.loads((run / "session.json").read_text())
    assert metadata["caster"] == session.caster == caster
    assert metadata["cameras"] == session.cameras == cameras


def test_catalogue_has_real_maps_and_spawn_information():
    maps = {item.id: item for item in discover_terrains(ROOT)}
    assert {"flat", "road_bumps", "gentle_hill"} <= maps.keys()
    assert maps["road_bumps"].spawn == (0., -2.8, 0., math.pi/2)
    assert maps["road_bumps"].world.is_file()
    assert maps["road_bumps"].height_grid.is_file()


def test_environment_is_shared_with_children_but_isolated_from_parent(tmp_path):
    inherited = {"ROS_DOMAIN_ID": "5", "GZ_PARTITION": "other", "PATH": "/usr/bin"}
    first = SimulationSession(ROOT, tmp_path, inherited, domain_id=177)
    second = SimulationSession(ROOT, tmp_path, inherited, domain_id=178)
    assert inherited["ROS_DOMAIN_ID"] == "5"
    assert first.env["ROS_DOMAIN_ID"] == "177"
    assert first.env["GZ_PARTITION"] != second.env["GZ_PARTITION"]


def test_stop_terminates_only_owned_group():
    owned = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"],
                             start_new_session=True, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"],
                                 start_new_session=True, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
    try:
        stop_process_group(owned, grace=.2)
        assert owned.poll() is not None
        assert unrelated.poll() is None
    finally:
        stop_process_group(owned, grace=.2)
        stop_process_group(unrelated, grace=.2)


@pytest.mark.parametrize("launcher_exits_first", [False, True])
def test_stop_cleans_descendant_after_launcher_exit(tmp_path, launcher_exits_first):
    ready = tmp_path / "child.pid"
    # The server survives SIGINT to exercise group escalation after the ros2
    # launcher has exited, including when it exits before shutdown begins.
    child_code = ("import os,signal,time; from pathlib import Path; "
                  "signal.signal(signal.SIGINT, signal.SIG_IGN); "
                  f"Path({str(ready)!r}).write_text(str(os.getpid())); time.sleep(30)")
    launcher_code = ("import subprocess,sys,time; from pathlib import Path\n"
                     f"subprocess.Popen([sys.executable, '-c', {child_code!r}])\n"
                     f"while not Path({str(ready)!r}).exists(): time.sleep(.01)\n" +
                     ("" if launcher_exits_first else "time.sleep(30)\n"))
    owned = subprocess.Popen([sys.executable, "-c", launcher_code], start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"],
                                 start_new_session=True, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + 5.
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        assert ready.exists()
        if launcher_exits_first:
            owned.wait(timeout=5.)
        assert _process_group_alive(owned.pid)
        start = time.monotonic()
        stop_process_group(owned, grace=.1)
        assert not _process_group_alive(owned.pid)
        assert unrelated.poll() is None
        # Zombie-only groups do not consume the TERM/KILL grace periods.
        assert time.monotonic() - start < 2.
    finally:
        stop_process_group(owned, grace=.1)
        stop_process_group(unrelated, grace=.1)


def test_route_starts_at_measured_pose_and_waits_for_waypoint_settling():
    plan = MotionPlan([(1., 0.), (1., 1.)], .2, (.2, 0.), 10., ROOT)
    assert plan.points == [(.2, 0.), (1., 0.), (1., 1.)]
    arrival = 10. + plan.route.starts[1]
    plan.sample(arrival + .01, (.8, 0.), 0.)
    assert plan.index == 0 and plan.state == "settling"
    plan.sample(arrival + .1, (1., 0.), .1)
    assert plan.index == 0  # Position alone is insufficient while still moving.
    plan.sample(arrival + .2, (1., 0.), 0.)
    plan.sample(arrival + .41, (1., 0.), 0.)
    assert plan.index == 1 and plan.state == "moving"


def test_unreachable_waypoint_stops_with_specific_error():
    plan = MotionPlan([(1., 0.)], .2, (0., 0.), 0., ROOT)
    plan.sample(plan.route.duration+21., (0., 0.), 0.)
    assert plan.state == "error"
    assert "Waypoint 1" in plan.error


@pytest.mark.parametrize("points,speed", [([(math.nan, 0.)], .1), ([(1., 0.), (1., 0.)], .1),
                                        ([(1., 0.)], 0.), ([(1., 0.)], 1.)])
def test_reject_bad_commands(points, speed):
    with pytest.raises(ValueError):
        MotionPlan(points, speed, (0., 0.), 0., ROOT)


def test_offset_kinematics_gives_equal_forward_wheels_and_opposite_turn_wheels():
    left, right, angular = inverse_drive(.1, 0., 0.)
    assert left == pytest.approx(right)
    assert angular == 0.
    left, right, angular = inverse_drive(0., .27114*.5, 0.)
    assert left == pytest.approx(-right)
    assert angular == pytest.approx(.5)


def test_manual_deadman_operates_without_advancing_sim_clock():
    # Exercise the wall-clock guard with a paused simulation and no ROS runtime.
    controller = LiveController.__new__(LiveController)
    controller._lock = threading.RLock()
    controller._manual = (.1, 0., 0.)
    controller._manual_wall = time.monotonic()-.31
    controller._plan = None
    commands = []
    controller._command = commands.append
    controller._tick()
    assert controller._manual is None
    assert controller._state == "stopped"
    assert commands == [[0.]*5]


def test_native_replay_has_no_rosbag_or_drive_process(tmp_path, monkeypatch):
    run = tmp_path / "session"
    log = run / "gazebo_log"
    log.mkdir(parents=True)
    (log / "state.tlog").write_bytes(b"test")
    (run / "session.json").write_text(json.dumps({"terrain": "flat", "world_name": "test"}))
    recording = run / "recording_1"
    recording.mkdir()
    (recording / "recording.json").write_text(json.dumps({"session": {"gazebo_log": str(log)}}))
    session = SimulationSession(ROOT, tmp_path, domain_id=179)
    commands = []
    monkeypatch.setattr(session, "_spawn", lambda command, output: commands.append(command))
    session.start_replay(recording, gui=False)
    assert session.replay is True
    assert session.world_name == "default"
    assert commands == [["gz", "sim", "-r", "-v", "3", "--playback", str(log),
                         "-s", "--headless-rendering"]]


def test_pause_requires_positive_service_ack(tmp_path, monkeypatch):
    session = SimulationSession(ROOT, tmp_path)
    class Process:
        def poll(self):
            return None
    session.process, session.world_name = Process(), "map"
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 0, "data: false", ""))
    with pytest.raises(RuntimeError, match="acknowledge"):
        session.set_paused(True)
    assert not session.paused
