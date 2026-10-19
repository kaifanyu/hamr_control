"""Placement must distinguish fresh, settled feedback from stale odometry."""
import math
import threading
from types import SimpleNamespace

import pytest

from hamr_sim_ui.controller import LiveController


def test_snapshot_exposes_feedback_age_and_full_body_attitude(monkeypatch):
    controller = LiveController.__new__(LiveController)
    controller._lock = threading.RLock()
    controller._state, controller._error = "stopped", ""
    controller._joints, controller._clock = object(), 5.
    controller._plan = controller._reference = None
    controller._velocity = (0., 0., 0.)
    controller._odom_wall = 11.25
    pose = SimpleNamespace(position=SimpleNamespace(x=1., y=2., z=3.),
                           orientation=SimpleNamespace(x=math.sin(.15), y=0., z=0., w=math.cos(.15)))
    controller._odom = SimpleNamespace(pose=SimpleNamespace(pose=pose),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=4, nanosec=900_000_000)))
    monkeypatch.setattr("hamr_sim_ui.controller.time.monotonic", lambda: 12.)
    state = controller.snapshot()
    assert state["feedback_ready"] and state["feedback_age_s"] == pytest.approx(.75)
    assert state["odom_time"] == pytest.approx(4.9)
    assert state["position"] == (1., 2.) and state["z"] == 3.
    assert (state["roll"], state["pitch"], state["yaw"]) == pytest.approx((.3, 0., 0.))

    controller._command = lambda command: None
    controller.reset()
    reset = controller.snapshot()
    assert not reset["feedback_ready"]
    assert all(reset[key] is None for key in ("feedback_age_s", "odom_time", "position", "z", "roll", "pitch", "yaw"))
