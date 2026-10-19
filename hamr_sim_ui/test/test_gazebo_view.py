"""Verify owned-window selection and the foreign-window geometry workaround."""
import os
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5.QtWidgets import QApplication, QWidget

from hamr_sim_ui.gazebo_view import GazeboView, WindowGeometry, find_window, window_geometry

_APP = QApplication.instance() or QApplication([])


def test_selects_owned_main_window_not_helper_or_another_session():
    tree = '''
     0x10 "Gazebo Sim": ("gz-sim-gui" "Gazebo GUI")  1800x1000+0+0  +0+0
     0x11 "Gazebo GUI": ()  1x1+0+0  +0+0
     0x12 "Gazebo Sim": ("gz-sim-gui" "Gazebo GUI")  1120x780+0+0  +0+0
'''
    def run(command, **_kwargs):
        if command[0] == "xwininfo":
            return SimpleNamespace(stdout=tree, returncode=0)
        return SimpleNamespace(stdout="_NET_WM_PID(CARDINAL) = " +
                               {"0x10": "99", "0x12": "42"}[command[2]], returncode=0)
    with patch("hamr_sim_ui.gazebo_view.descendant_pids", return_value={40, 41, 42}), \
         patch("hamr_sim_ui.gazebo_view.subprocess.run", side_effect=run):
        assert find_window(40) == 0x12


def test_reads_native_parent_and_size_instead_of_cached_qt_state():
    info = '''
Parent window id: 0x600006 "HAMR"
Relative upper-left X: 68
Relative upper-left Y: 9
Width: 1120
Height: 780
'''
    with patch("hamr_sim_ui.gazebo_view.subprocess.run",
               return_value=SimpleNamespace(stdout=info, returncode=0)):
        assert window_geometry(0x12) == WindowGeometry(0x600006, 68, 9, 1120, 780)
    with patch("hamr_sim_ui.gazebo_view.subprocess.run",
               return_value=SimpleNamespace(stdout="", returncode=1)):
        assert window_geometry(0x12) is None


def test_delayed_gazebo_size_is_corrected_before_reporting_embedded():
    view = GazeboView()
    foreign = MagicMock()
    foreign.winId.return_value = 0x12
    foreign.parent.return_value.winId.return_value = 0x60
    foreign.devicePixelRatio.return_value = 1.
    foreign.geometry.return_value.topLeft.return_value.x.return_value = 0
    foreign.geometry.return_value.topLeft.return_value.y.return_value = 0
    view._window = foreign
    container = QWidget(view)
    container.resize(640, 480)
    view._container = container
    with patch("hamr_sim_ui.gazebo_view.window_geometry",
               return_value=WindowGeometry(0x60, 0, 0, 1120, 780)):
        view._check_embedding()
    assert not view.embedded
    assert [call.args for call in foreign.resize.call_args_list] == [(639, 479), (640, 480)]
    with patch("hamr_sim_ui.gazebo_view.window_geometry",
               return_value=WindowGeometry(0x60, 0, 0, 640, 480)):
        view._check_embedding()
    assert view.embedded
    view._verification_deadline = float('inf')
    with patch("hamr_sim_ui.gazebo_view.window_geometry",
               return_value=WindowGeometry(0x438, 0, 0, 640, 480)):
        view._check_embedding()
    assert not view.embedded
    view.detach()
    foreign.hide.assert_called_once()
    foreign.show.assert_not_called()
    view.close()


def test_reparent_failure_restores_visible_external_fallback():
    view = GazeboView()
    foreign = MagicMock()
    foreign.winId.return_value = 0x12
    foreign.parent.return_value.winId.return_value = 0x60
    view._window = foreign
    view._container = QWidget(view)
    view._layout.addWidget(view._container)
    view._message.hide()
    with patch("hamr_sim_ui.gazebo_view.window_geometry",
               return_value=WindowGeometry(0x438, 0, 0, 640, 480)):
        view._check_embedding()
    assert view._container is None
    assert view._window is None
    assert not view._message.isHidden()
    assert "separate window" in view._message.text()
    foreign.show.assert_called_once()
    view.close()
