"""Collision-free live route and driven-path overlays in Gazebo's own scene."""
from __future__ import annotations

import math
import os
from pathlib import Path
import subprocess
import sys
import threading


def _points(points, height):
    result = []
    for point in points:
        x, y = float(point[0]), float(point[1])
        z = float(point[2]) if len(point) > 2 else float(height(x, y) if callable(height) else height)
        if not all(math.isfinite(v) for v in (x, y, z)):
            raise ValueError("Marker coordinates must be finite")
        result.append((x, y, z))
    return result


def route_request(points, height=.04):
    """A Marker_V protobuf in text format; inputs never enter a shell."""
    points = _points(points, height)
    chunks = ['marker { ns: "hamr_ui_route" action: DELETE_ALL }']
    if not points:
        return " ".join(chunks)
    line = points
    if callable(height) and len(points) > 1:
        # A line directly joining two terrain heights can pass through a hill.
        # Sample the map surface so the requested path remains visible in 3D.
        lengths = [math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:])]
        spacing = max(.08, sum(lengths) / 900.)
        line = []
        for a, b, length in zip(points, points[1:], lengths):
            count = max(1, math.ceil(length / spacing))
            for index in range(count):
                weight = index / count
                x, y = a[0]+weight*(b[0]-a[0]), a[1]+weight*(b[1]-a[1])
                z = float(height(x, y))
                if not math.isfinite(z):
                    raise ValueError("Terrain marker heights must be finite")
                line.append((x, y, z))
        line.append(points[-1])
    locations = " ".join(f"point {{ x: {x:.7g} y: {y:.7g} z: {z:.7g} }}" for x, y, z in line)
    color = "material { ambient { r: .694 g: .741 b: .510 a: 1 } diffuse { r: .694 g: .741 b: .510 a: 1 } }"
    if len(points) > 1:
        chunks.append('marker { ns: "hamr_ui_route" id: 1 type: LINE_STRIP '
                      'action: ADD_MODIFY pose { orientation { w: 1 } } '
                      'scale { x: 1 y: 1 z: 1 } ' + color + " " + locations + " }")
    for index, (x, y, z) in enumerate(points):
        chunks.append(f'marker {{ ns: "hamr_ui_route" id: {index + 2} type: SPHERE '
                      f'action: ADD_MODIFY pose {{ position {{ x: {x:.7g} y: {y:.7g} z: {z:.7g} }} '
                      'orientation { w: 1 } } scale { x: .09 y: .09 z: .09 } ' + color + " }")
    return " ".join(chunks)


def trace_request(points, height=.04):
    points = _points(points, height)
    if len(points) < 2:
        return 'marker { ns: "hamr_ui_trace" action: DELETE_ALL }'
    locations = " ".join(f"point {{ x: {x:.7g} y: {y:.7g} z: {z:.7g} }}" for x, y, z in points)
    return ('marker { ns: "hamr_ui_trace" id: 1 type: LINE_STRIP action: ADD_MODIFY '
            'pose { orientation { w: 1 } } scale { x: 1 y: 1 z: 1 } '
            'material { ambient { r: .808 g: .725 b: .541 a: 1 } diffuse { r: .808 g: .725 b: .541 a: 1 } } '
            + locations + " }")


def _native_sender(env):
    """Reuse one Gazebo connection when Harmonic's optional bindings exist."""
    for prefix in (Path("/"), Path.home() / ".cache/hamr-gazebo-root"):
        for relative in ("usr/lib/python3/dist-packages",
                         "opt/ros/jazzy/opt/gz_transport_vendor/lib/python",
                         "opt/ros/jazzy/opt/gz_msgs_vendor/lib/python"):
            path = prefix / relative
            if path.is_dir() and str(path) not in sys.path:
                sys.path.append(str(path))
    from google.protobuf.text_format import Parse
    from gz.transport13 import Node, NodeOptions
    from gz.msgs10.marker_v_pb2 import Marker_V
    from gz.msgs10.boolean_pb2 import Boolean

    options = NodeOptions()
    if env.get("GZ_PARTITION"):
        options.partition = env["GZ_PARTITION"]
    node = Node(options)

    def send(request):
        message = Parse(request, Marker_V())
        success, response = node.request("/marker_array", message, Marker_V, Boolean, 1000)
        return "" if success and response.data else "Gazebo markers are not connected"
    return send


class MarkerPublisher:
    """Coalesce changed overlays off the GUI thread using one connection."""
    def __init__(self, env=None):
        self.env = dict(os.environ if env is None else env)
        self.last_error = ""
        self._condition = threading.Condition()
        self._pending = {}
        self._active = {}
        self._sent = {}
        self._generation = 0
        self._trace_cache = None
        self._closed = False
        self.backend = None
        self._worker = threading.Thread(target=self._run, name="gazebo-markers", daemon=True)
        self._worker.start()

    def _queue(self, name, request):
        with self._condition:
            if self._closed or self._pending.get(name) == request:
                return
            if name not in self._active and self._sent.get(name) == request:
                self._pending.pop(name, None)
                return
            self._pending[name] = request
            self._condition.notify()

    def set_route(self, points, height=.04):
        self._queue("route", route_request(points, height))

    def set_trace(self, points, height=.04):
        # Keep the argv comfortably below the Linux per-argument limit.
        # Decimate before terrain sampling; long runs retain their full trail
        # in telemetry and recordings, while only the overlay is simplified.
        points = list(points)
        if len(points) > 1200:
            stride = math.ceil(len(points) / 1200)
            points = points[::stride] + [points[-1]]
        key = (tuple(tuple(point) for point in points), height)
        with self._condition:
            if self._trace_cache is None or self._trace_cache[0] != key:
                self._trace_cache = (key, trace_request(points, height))
            request = self._trace_cache[1]
        self._queue("trace", request)

    def reset(self):
        """Invalidate overlays after a new scene, replay, or relocated vehicle."""
        with self._condition:
            self._generation += 1
            self._pending.clear()
            self._active.clear()
            self._sent.clear()
            self._trace_cache = None

    def _cli_send(self, request):
        result = subprocess.run([
            "gz", "service", "-s", "/marker_array", "--reqtype", "gz.msgs.Marker_V",
            "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req", request],
            env=self.env, text=True, capture_output=True, timeout=3.)
        return "" if result.returncode == 0 and "true" in result.stdout else (
            result.stderr.strip() or result.stdout.strip() or "Gazebo markers are not connected")

    def _run(self):
        send = None
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or bool(self._pending))
                if self._closed:
                    return
                pending, self._pending = self._pending, {}
                generation = self._generation
                self._active.update(pending)
            request = " ".join(pending.values())
            try:
                if send is None:
                    try:
                        send = _native_sender(self.env)
                        self.backend = "native"
                    except (ImportError, OSError, RuntimeError, AttributeError, TypeError):
                        send, self.backend = self._cli_send, "cli"
                self.last_error = send(request)
            except Exception as error:
                self.last_error = str(error)
            with self._condition:
                if generation == self._generation:
                    for name in pending:
                        self._active.pop(name, None)
                    if not self.last_error:
                        self._sent.update(pending)
                        # A duplicate may have arrived while this was in flight.
                        self._pending = {name: value for name, value in self._pending.items()
                                         if self._sent.get(name) != value}
                self._condition.notify_all()

    def close(self):
        with self._condition:
            self._closed = True
            self._condition.notify()
        if threading.current_thread() is not self._worker:
            self._worker.join(timeout=3.5)
