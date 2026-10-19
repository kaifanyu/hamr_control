"""Embed this session's native Gazebo window on X11 / WSLg."""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
import re
import shutil
import subprocess
import time

from PyQt5.QtCore import QEvent, QPoint, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QGuiApplication, QWindow
from PyQt5.QtWidgets import QLabel, QVBoxLayout, QWidget


def descendant_pids(root_pid):
    """Include only our process tree, never another user's Gazebo session."""
    parents = {}
    for item in Path("/proc").iterdir():
        if not item.name.isdigit():
            continue
        try:
            # comm may itself contain spaces or parentheses.
            fields = (item / "stat").read_text().rsplit(")", 1)[1].split()
            parents[int(item.name)] = int(fields[1])
        except (OSError, IndexError, ValueError):
            pass
    found = {int(root_pid)}
    while True:
        more = {pid for pid, parent in parents.items() if parent in found}
        if more.issubset(found):
            return found
        found.update(more)


def find_window(root_pid):
    """Find the main Gazebo view owned by root_pid's tree.

    Qt also creates one-pixel helper windows named "Gazebo GUI". Never attach
    those, a transient menu, or a Gazebo instance from another process tree.
    """
    pids = descendant_pids(root_pid)
    result = subprocess.run(["xwininfo", "-root", "-tree"], capture_output=True,
                            text=True, timeout=1.5)
    candidates = []
    for line in result.stdout.splitlines():
        if not re.search(r"gazebo|gz sim", line, re.IGNORECASE):
            continue
        match = re.match(r"\s*(0x[0-9a-fA-F]+)\s", line)
        if not match:
            continue
        size = re.search(r"\s(\d+)x(\d+)[+-]\d+[+-]\d+", line)
        if not size or min(map(int, size.groups())) < 200:
            continue
        candidates.append((int(size.group(1)) * int(size.group(2)), match.group(1)))
    for _, window_id in sorted(candidates, reverse=True):
        info = subprocess.run(["xprop", "-id", window_id, "_NET_WM_PID"],
                              capture_output=True, text=True, timeout=1.)
        pid = re.search(r"=\s*(\d+)", info.stdout)
        if pid and int(pid.group(1)) in pids:
            return int(window_id, 16)
    return None


@dataclass(frozen=True)
class WindowGeometry:
    parent: int
    x: int
    y: int
    width: int
    height: int


def window_geometry(window_id):
    """Read server-side geometry; a foreign QWindow can have stale Qt state."""
    result = subprocess.run(["xwininfo", "-id", hex(window_id), "-tree", "-stats"],
                            capture_output=True, text=True, timeout=1.)
    if result.returncode:
        return None
    patterns = (r"Parent window id:\s*(0x[0-9a-fA-F]+)",
                r"Relative upper-left X:\s*(-?\d+)", r"Relative upper-left Y:\s*(-?\d+)",
                r"Width:\s*(\d+)", r"Height:\s*(\d+)")
    matches = [re.search(pattern, result.stdout) for pattern in patterns]
    if not all(matches):
        return None
    values = [int(match.group(1), 16 if index == 0 else 10)
              for index, match in enumerate(matches)]
    return WindowGeometry(*values)


class GazeboView(QWidget):
    """The actual Gazebo viewport, with a graceful external-window fallback.

    Call attach_process(launch_process.pid) after launch, and detach() before
    stopping that process. No screenshot streaming or substitute renderer.
    """
    status_changed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._message = QLabel("Start a simulation to open the live Gazebo 3D view.")
        self._message.setAlignment(Qt.AlignCenter)
        self._message.setWordWrap(True)
        self._layout.addWidget(self._message)
        self._window = None
        self._container = None
        self._pid = None
        self._deadline = 0.
        self._verification_deadline = 0.
        self._confirmed = False
        self._timer = QTimer(self)
        self._timer.setInterval(250)
        self._timer.timeout.connect(self._try_attach)
        self._geometry_timer = QTimer(self)
        self._geometry_timer.setInterval(1000)
        self._geometry_timer.timeout.connect(self._check_embedding)

    @property
    def embedded(self):
        return self._confirmed

    def attach_process(self, pid):
        self.detach()
        if (QGuiApplication.platformName() != "xcb" or not os.environ.get("DISPLAY")
                or not shutil.which("xwininfo") or not shutil.which("xprop")):
            self._set_message("Gazebo opened in a separate window.\n"
                              "Embedding requires X11 (xcb), xwininfo and xprop.")
            return
        self._pid = int(pid)
        self._deadline = time.monotonic() + 90.
        self._set_message("Connecting Gazebo…")
        self._timer.start()
        self._try_attach()

    def _set_message(self, text):
        self._message.setText(text)
        self.status_changed.emit(text.splitlines()[0])

    def _try_attach(self):
        if self._pid is None:
            return
        if time.monotonic() > self._deadline:
            self._timer.stop()
            self._set_message("Gazebo is available in its own window.\n"
                              "If it did not open, check the session log below.")
            return
        try:
            window_id = find_window(self._pid)
        except (OSError, subprocess.SubprocessError):
            return
        if window_id is None:
            return
        self._timer.stop()
        self._window = QWindow.fromWinId(window_id)
        if self._window is None:
            self._set_message("Use the separate Gazebo window for the 3D view.")
            return
        self._container = QWidget.createWindowContainer(self._window, self)
        self._container.setFocusPolicy(Qt.StrongFocus)
        self._container.setMinimumSize(400, 300)
        self._container.installEventFilter(self)
        self._message.hide()
        self._layout.addWidget(self._container)
        self._verification_deadline = time.monotonic() + 5.
        self._geometry_timer.start()
        QTimer.singleShot(0, self._check_embedding)

    def _check_embedding(self):
        """Correct Gazebo's startup resize and verify the real X11 parent.

        Gazebo applies its configured 1120x780 size after Qt embeds the view.
        Qt's foreign-window wrapper does not observe that change, so it can
        believe the panel already fits while Gazebo covers adjacent controls.
        A small intermediate resize invalidates that cache and sends the real
        panel size again. The monitor also catches a delayed Gazebo resize.
        """
        if self._window is None or self._container is None:
            return
        try:
            actual = window_geometry(int(self._window.winId()))
        except (OSError, subprocess.SubprocessError):
            return
        parent = self._window.parent()
        if actual is None or parent is None:
            self._confirmed = False
            return
        expected_parent = int(parent.winId())
        if actual.parent != expected_parent:
            self._confirmed = False
            # Let pending native Qt events settle before falling back.
            if time.monotonic() > self._verification_deadline:
                self.detach(show_external=True)
                self._set_message("Gazebo opened in a separate window.\n"
                                  "This display could not embed the native view.")
            return
        ratio = self._window.devicePixelRatio()
        width, height = self._container.width(), self._container.height()
        expected_size = (round(width * ratio), round(height * ratio))
        parent_widget = QWidget.find(expected_parent)
        position = (self._container.mapTo(parent_widget, QPoint()) if parent_widget is not None
                    else self._window.geometry().topLeft())
        expected_position = (round(position.x() * ratio), round(position.y() * ratio))
        if (actual.x, actual.y) != expected_position:
            self._window.setPosition(position.x() + 1, position.y())
            self._window.setPosition(position)
            self._confirmed = False
            return
        if (actual.width, actual.height) != expected_size:
            self._window.resize(max(1, width - 1), max(1, height - 1))
            self._window.resize(width, height)
            self._confirmed = False
            return
        if not self._confirmed:
            self._confirmed = True
            self.status_changed.emit("Live Gazebo view embedded")

    def eventFilter(self, watched, event):
        if watched is self._container and event.type() in (QEvent.Resize, QEvent.Show):
            QTimer.singleShot(0, self._check_embedding)
        return super().eventFilter(watched, event)

    def detach(self, *, show_external=False):
        """Release the foreign window before its process exits."""
        self._timer.stop()
        self._geometry_timer.stop()
        self._pid = None
        self._confirmed = False
        if self._window is not None:
            # Detach only precedes stopping/replacing this session. Showing it
            # here creates an unnecessary popup on every reload and shutdown.
            self._window.hide()
            self._window.setParent(None)
            if show_external:
                self._window.show()
        if self._container is not None:
            self._layout.removeWidget(self._container)
            self._container.deleteLater()
        self._container = self._window = None
        self._message.show()
        self._message.setText("Start a simulation to open the live Gazebo 3D view.")

    def closeEvent(self, event):
        self.detach()
        super().closeEvent(event)
