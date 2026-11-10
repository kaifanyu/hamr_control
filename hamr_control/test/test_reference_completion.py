"""Completion is a stop, never an exemption from the next watchdog."""
from types import SimpleNamespace

import pytest
from rclpy.time import Time
from std_srvs.srv import Trigger

from hamr_control.hamr_controller import HamrControlNode


@pytest.fixture
def state():
    node = object.__new__(HamrControlNode)
    clock = SimpleNamespace(ns=1_000_000_000)
    events, errors, infos = [], [], []
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=clock.ns))
    node.get_logger = lambda: SimpleNamespace(info=infos.append, error=errors.append)
    node.publish_zero_cmd = lambda: events.append("zero")
    node.reset_pid_state = lambda: events.append("reset")
    node.pid_step = lambda: events.append("pid")
    node.ensure_base_odom_ready = lambda _: True
    node.hamr_config = {"mode": "auto", "simulating": False}
    node.turret_enabled = False
    node.turret_to_base_orientation_ = None
    node.turret_to_world_orientation_ = None
    node.waiting_for_turret_state = False
    node.reference_timeout_s = 0.12
    node.reference_ = object()
    node.last_reference_time_ns = clock.ns
    node.reference_timed_out = False
    node.last_control_time = Time(nanoseconds=clock.ns - 10_000_000)
    return node, clock, events, errors, infos


def finish(node):
    return node.finish_reference(Trigger.Request(), Trigger.Response())


def test_successful_completion_stays_zero_without_timeout_errors(state):
    node, clock, events, errors, infos = state
    assert finish(node).success
    assert events == ["zero", "reset"]
    assert node.reference_ is None and node.last_reference_time_ns is None
    for _ in range(100):
        clock.ns += 10_000_000
        node.control_tick()
    assert events[2:] == ["zero"] * 100
    assert not errors and len(infos) == 1
    assert node.reference_timeout_s == 0.12


def test_queued_final_reference_cannot_restart_completed_trial(state):
    node, clock, events, errors, _ = state
    finish(node)
    for _ in range(50):
        clock.ns += 20_000_000
        node.callback_reference(object())
        node.control_tick()
    assert node.reference_ is None and node.reference_finished
    assert "pid" not in events and not errors


def test_next_trial_can_start_and_its_timeout_still_reports_error(state):
    node, clock, events, errors, _ = state
    finish(node)
    clock.ns += 120_000_000
    reference = object()
    node.callback_reference(reference)
    node.control_tick()
    assert node.reference_ is reference and not node.reference_finished
    assert events[-1] == "pid"
    clock.ns += 120_000_001
    node.control_tick()
    assert events[-2:] == ["reset", "zero"]
    assert len(errors) == 1 and "timed out" in errors[0]
    clock.ns += 10_000_000
    node.control_tick()
    assert len(errors) == 1  # Once per actual outage.


def test_completion_does_not_clear_vicon_inhibit(state):
    node, _, _, _, _ = state
    node.vicon_motion_inhibited = True
    node.vicon_reference_rearm_ready = False
    assert finish(node).success
    assert node.vicon_motion_inhibited
    assert not node.vicon_reference_rearm_ready


@pytest.mark.parametrize("timeout", [0.0, float("nan")])
def test_no_positive_quiet_interval_cannot_rearm_completion(state, timeout):
    node, clock, _, _, _ = state
    finish(node)
    node.reference_timeout_s = timeout
    clock.ns += 10_000_000_000
    node.callback_reference(object())
    assert node.reference_ is None and node.reference_finished


def test_manual_mode_rejects_auto_completion(state):
    node, _, events, _, _ = state
    node.hamr_config["mode"] = "manual"
    assert not finish(node).success
    assert not events
