"""Avoid repeated work without dropping changed overlays or failed retries."""
import threading
from types import SimpleNamespace

import pytest

from hamr_sim_ui import markers


def wait_for(publisher, predicate):
    with publisher._condition:
        assert publisher._condition.wait_for(predicate, timeout=3.)


@pytest.fixture
def delivery(monkeypatch):
    calls = []

    def send(request):
        calls.append(request)
        return ""

    monkeypatch.setattr(markers, "_native_sender", lambda env: send)
    publisher = markers.MarkerPublisher({"GZ_PARTITION": "test"})
    try:
        yield publisher, calls
    finally:
        publisher.close()


def test_unchanged_trace_does_not_repeat_height_sampling_or_delivery(delivery):
    publisher, calls = delivery
    sampled = []

    def height(x, y):
        sampled.append((x, y))
        return .1

    points = [(index * .01, 0.) for index in range(10_000)]
    publisher.set_trace(points, height)
    wait_for(publisher, lambda: "trace" in publisher._sent)
    assert len(sampled) <= 1201
    assert 'x: 99.99' in calls[0]  # Preserve the latest location.
    first_count = len(sampled)
    publisher.set_trace(points, height)
    with publisher._condition:
        assert not publisher._pending and not publisher._active
    assert len(calls) == 1 and len(sampled) == first_count
    publisher.reset()
    publisher.set_trace(points, height)
    wait_for(publisher, lambda: "trace" in publisher._sent)
    assert len(calls) == 2 and len(sampled) == first_count * 2


def test_latest_overlay_survives_inflight_update(monkeypatch):
    entered, release = threading.Event(), threading.Event()
    calls = []

    def send(request):
        calls.append(request)
        if len(calls) == 1:
            entered.set()
            assert release.wait(3.)
        return ""

    monkeypatch.setattr(markers, "_native_sender", lambda env: send)
    publisher = markers.MarkerPublisher({})
    try:
        first, latest = [(0., 0.), (1., 0.)], [(0., 0.), (3., 0.)]
        publisher.set_trace(first)
        assert entered.wait(3.)
        publisher.set_trace([(0., 0.), (2., 0.)])
        publisher.set_trace(latest)
        release.set()
        wait_for(publisher, lambda: publisher._sent.get("trace") == markers.trace_request(latest))
        assert calls == [markers.trace_request(first), markers.trace_request(latest)]
    finally:
        release.set()
        publisher.close()


def test_failure_retries_identical_update_and_cli_fallback(monkeypatch):
    def unavailable(env):
        raise ImportError("Optional Gazebo Python bindings are absent")

    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0, stdout="data: true" if len(calls) > 1 else "",
                               stderr="")

    monkeypatch.setattr(markers, "_native_sender", unavailable)
    monkeypatch.setattr(markers.subprocess, "run", run)
    publisher = markers.MarkerPublisher({"GZ_PARTITION": "test"})
    try:
        points = [(0., 0.), (1., 0.)]
        publisher.set_trace(points)
        wait_for(publisher, lambda: bool(publisher.last_error))
        publisher.set_trace(points)
        wait_for(publisher, lambda: "trace" in publisher._sent)
        assert publisher.backend == "cli" and len(calls) == 2
        assert publisher.last_error == ""
    finally:
        publisher.close()


def test_reset_ignores_previous_scene_acknowledgement(monkeypatch):
    entered, release = threading.Event(), threading.Event()

    def send(request):
        entered.set()
        assert release.wait(3.)
        return ""

    monkeypatch.setattr(markers, "_native_sender", lambda env: send)
    publisher = markers.MarkerPublisher({})
    try:
        publisher.set_trace([(0., 0.), (1., 0.)])
        assert entered.wait(3.)
        publisher.reset()
        release.set()
        # close joins the in-flight send; an old scene must never repopulate
        # the successful cache and suppress the new scene's identical route.
        publisher.close()
        assert publisher._sent == {}
    finally:
        release.set()
        publisher.close()
