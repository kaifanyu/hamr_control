"""Coordinate editing and keyboard release/focus guarantees without a simulator."""
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5 import QtCore, QtGui, QtTest, QtWidgets

from hamr_sim_ui.widgets import CameraView, TerrainMap
from hamr_sim_ui.app import Console


_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
_ROOT = Path(__file__).resolve().parents[2]


def mouse(widget, kind, point, button=QtCore.Qt.LeftButton):
    event = QtGui.QMouseEvent(kind, QtCore.QPointF(point), button,
                             button if kind != QtCore.QEvent.MouseButtonRelease else QtCore.Qt.NoButton,
                             QtCore.Qt.NoModifier)
    _APP.sendEvent(widget, event)


class MapEditingTest(unittest.TestCase):
    def setUp(self):
        self.map = TerrainMap()
        self.map.resize(530, 410)
        self.map.show()
        _APP.processEvents()

    def tearDown(self):
        self.map.close()

    def test_world_pixel_roundtrip_and_outside_click(self):
        self.map.bounds = (-4., 8., -2., 3.)
        for point in ((-4., -2.), (0., 0.), (8., 3.), (2.34, -.56)):
            actual = self.map.pixel_to_world(self.map.world_to_pixel(point))
            self.assertAlmostEqual(point[0], actual[0])
            self.assertAlmostEqual(point[1], actual[1])
        mouse(self.map, QtCore.QEvent.MouseButtonPress, QtCore.QPointF(1, 1))
        self.assertEqual(self.map.points, [])

    def test_add_drag_remove_live_waypoint(self):
        changes = QtTest.QSignalSpy(self.map.changed)
        origin = self.map.world_to_pixel((0., 0.))
        destination = self.map.world_to_pixel((1.2, -.8))
        mouse(self.map, QtCore.QEvent.MouseButtonPress, origin)
        mouse(self.map, QtCore.QEvent.MouseMove, destination)
        mouse(self.map, QtCore.QEvent.MouseButtonRelease, destination)
        self.assertAlmostEqual(self.map.points[0][0], 1.2)
        self.assertAlmostEqual(self.map.points[0][1], -.8)
        self.assertGreaterEqual(len(changes), 2)
        mouse(self.map, QtCore.QEvent.MouseButtonPress, destination, QtCore.Qt.RightButton)
        self.assertEqual(self.map.points, [])

    def test_replacing_or_disabling_map_cancels_drag(self):
        origin = self.map.world_to_pixel((0., 0.))
        mouse(self.map, QtCore.QEvent.MouseButtonPress, origin)
        self.map.set_points([])
        mouse(self.map, QtCore.QEvent.MouseMove, origin)
        self.assertEqual(self.map.points, [])
        mouse(self.map, QtCore.QEvent.MouseButtonPress, origin)
        self.map.editable = False
        mouse(self.map, QtCore.QEvent.MouseMove, self.map.world_to_pixel((1., 1.)))
        self.assertAlmostEqual(self.map.points[0][0], 0.)
        self.assertAlmostEqual(self.map.points[0][1], 0.)
        self.assertIsNone(self.map.drag_index)

    def test_terrain_change_clears_route_and_notifies_count(self):
        self.map.set_points([(1, 2)])
        changes = QtTest.QSignalSpy(self.map.changed)
        self.map.set_terrain(SimpleNamespace(bounds=(-2, 2, -2, 2), height_grid=None))
        self.assertEqual(self.map.points, [])
        self.assertEqual(list(changes[-1]), [[]])

    def test_placement_drag_sets_pose_without_editing_waypoints(self):
        self.map.set_points([[.5, .5]])
        self.map.set_placement_mode(True, yaw=.4)
        placements = QtTest.QSignalSpy(self.map.placement_requested)
        origin = self.map.world_to_pixel((-1., -1.))
        heading = self.map.world_to_pixel((-1., 1.))
        mouse(self.map, QtCore.QEvent.MouseButtonPress, origin)
        mouse(self.map, QtCore.QEvent.MouseMove, heading)
        mouse(self.map, QtCore.QEvent.MouseButtonRelease, heading)
        self.assertEqual(self.map.points, [[.5, .5]])
        self.assertEqual(len(placements), 1)
        self.assertAlmostEqual(placements[0][0], -1.)
        self.assertAlmostEqual(placements[0][1], -1.)
        self.assertAlmostEqual(placements[0][2], 1.5707963267948966)

    def test_placement_click_preserves_heading_and_can_be_cancelled(self):
        self.map.set_points([[0., 0.]])
        self.map.set_placement_mode(True, yaw=.4)
        placements = QtTest.QSignalSpy(self.map.placement_requested)
        cancelled = QtTest.QSignalSpy(self.map.placement_cancelled)
        origin = self.map.world_to_pixel((0., 0.))
        mouse(self.map, QtCore.QEvent.MouseButtonPress, origin)
        mouse(self.map, QtCore.QEvent.MouseButtonRelease, origin)
        self.assertAlmostEqual(placements[0][2], .4)
        mouse(self.map, QtCore.QEvent.MouseButtonPress, origin, QtCore.Qt.RightButton)
        self.assertFalse(self.map.placement_mode)
        self.assertEqual(len(cancelled), 1)
        self.assertEqual(self.map.points, [[0., 0.]])

    def test_placement_release_outside_map_does_not_move_the_car(self):
        self.map.set_placement_mode(True)
        placements = QtTest.QSignalSpy(self.map.placement_requested)
        mouse(self.map, QtCore.QEvent.MouseButtonPress, self.map.world_to_pixel((0., 0.)))
        mouse(self.map, QtCore.QEvent.MouseButtonRelease, QtCore.QPointF(-1, -1))
        self.assertEqual(len(placements), 0)

    def test_camera_clear_cannot_resurrect_previous_frame(self):
        camera = CameraView()
        camera.set_frame(dict(encoding="rgb8", width=2, height=1, step=6,
                              data=b"\xff\0\0\0\xff\0"))
        self.assertIsNotNone(camera.source)
        camera.clear()
        camera.resize(400, 250)
        camera._resize()
        self.assertIsNone(camera.source)
        self.assertTrue(camera.pixmap() is None or camera.pixmap().isNull())


class ConsoleControlsTest(unittest.TestCase):
    def setUp(self):
        self.environment = patch.dict(os.environ)
        self.environment.start()
        self.fake_telemetry = MagicMock()
        self.fake_controller = MagicMock()
        self.fake_controller.snapshot.return_value = {'plan_summary': None}
        self.patches = [
            patch("hamr_sim_ui.telemetry.RosTelemetry", return_value=self.fake_telemetry),
            patch("hamr_sim_ui.controller.LiveController", return_value=self.fake_controller),
            patch("hamr_sim_ui.markers.MarkerPublisher"),
        ]
        for item in self.patches:
            item.start()
        self.window = Console(SimpleNamespace(root=_ROOT, output=None, terrain="flat",
                                              no_autostart=True, headless=True))
        self.window.timer.stop()
        self.window.show()
        _APP.processEvents()
        self.window.manual_btn.setChecked(True)
        self.fake_controller.halt.reset_mock()

    def tearDown(self):
        _APP.removeEventFilter(self.window)
        self.window._shutdown_done = True
        self.window.close()
        for item in reversed(self.patches):
            item.stop()
        self.environment.stop()

    def test_last_key_release_stops_and_repeat_release_does_not(self):
        QtTest.QTest.keyPress(self.window, QtCore.Qt.Key_W)
        QtTest.QTest.keyPress(self.window, QtCore.Qt.Key_A)
        autorepeat = QtGui.QKeyEvent(QtCore.QEvent.KeyRelease, QtCore.Qt.Key_W,
                                     QtCore.Qt.NoModifier, "w", True)
        _APP.sendEvent(self.window, autorepeat)
        self.assertIn(QtCore.Qt.Key_W, self.window.keys)
        QtTest.QTest.keyRelease(self.window, QtCore.Qt.Key_W)
        self.fake_controller.halt.assert_not_called()
        QtTest.QTest.keyRelease(self.window, QtCore.Qt.Key_A)
        self.assertEqual(self.window.keys, set())
        self.fake_controller.halt.assert_called_once()

    def test_leaving_application_stops_held_commands(self):
        QtTest.QTest.keyPress(self.window, QtCore.Qt.Key_W)
        _APP.sendEvent(self.window, QtCore.QEvent(QtCore.QEvent.ApplicationDeactivate))
        self.assertEqual(self.window.keys, set())
        self.fake_controller.halt.assert_called_once()

    def test_focus_transfer_to_foreign_view_stops_held_commands(self):
        QtTest.QTest.keyPress(self.window, QtCore.Qt.Key_W)
        # A native embedded child may take X11 focus without deactivating
        # the top-level app; its key releases belong to the Gazebo process.
        _APP.sendEvent(self.window, QtGui.QFocusEvent(QtCore.QEvent.FocusOut))
        self.assertEqual(self.window.keys, set())
        self.fake_controller.halt.assert_called_once()

    def test_space_clears_manual_mode_and_stops_motion(self):
        QtTest.QTest.keyPress(self.window, QtCore.Qt.Key_W)
        QtTest.QTest.keyPress(self.window, QtCore.Qt.Key_Space)
        self.assertFalse(self.window.manual_btn.isChecked())
        self.assertEqual(self.window.keys, set())
        self.fake_controller.halt.assert_called_once()

    def test_leaving_manual_tab_disarms_keyboard_and_stops_motion(self):
        self.window.control_tabs.setCurrentIndex(1)
        QtTest.QTest.keyPress(self.window, QtCore.Qt.Key_W)
        self.fake_controller.halt.reset_mock()
        self.window.control_tabs.setCurrentIndex(0)
        self.assertFalse(self.window.manual_btn.isChecked())
        self.assertEqual(self.window.keys, set())
        self.fake_controller.halt.assert_called_once()

    def test_camera_off_hides_preview_disables_video_and_ignores_queued_frame(self):
        self.assertFalse(self.window.loaded_cameras)
        self.assertFalse(self.window.cameras_box.isChecked())
        self.window.loaded_cameras = False
        self.window._refresh_model_ui()
        self.assertTrue(self.window.camera_panel.isHidden())
        self.assertFalse(self.window.video_box.isChecked())
        self.assertFalse(self.window.video_box.isEnabled())
        self.window._image('closeup', dict(encoding='rgb8', width=2, height=1,
                                         step=6, data=b'\xff\0\0\0\xff\0'))
        self.assertIsNone(self.window.camera.source)

    def test_placement_stops_manual_and_escape_cancels(self):
        self.window.session.process = SimpleNamespace(pid=99999, poll=lambda: None)
        self.fake_telemetry.snapshot.return_value = {'odom_received': True, 'yaw': .3}
        self.window.place_btn.setChecked(True)
        self.window.toggle_placement()
        self.assertFalse(self.window.manual_btn.isChecked())
        self.assertTrue(self.window.map.placement_mode)
        self.fake_controller.halt.assert_called_once()
        QtTest.QTest.keyClick(self.window, QtCore.Qt.Key_Escape)
        self.assertFalse(self.window.map.placement_mode)
        self.assertFalse(self.window.place_btn.isChecked())

    def test_placement_saves_recording_before_move_and_starts_new_history(self):
        self.window.session.process = SimpleNamespace(pid=99999, poll=lambda: None)
        self.window.map.set_points([[.5, .5]])
        self.window.map.trail = [(0., 0.), (.1, .1)]
        self.fake_telemetry.snapshot.return_value = {'recording': True}
        calls = []
        self.fake_telemetry.stop_recording.side_effect = lambda: (calls.append('save') or {'recording_complete': True})
        self.window.work = lambda operation, callback=None: callback(operation())
        def reposition(x, y, yaw, **kwargs):
            calls.append('move')
            self.assertIs(kwargs['feedback'], self.fake_controller.snapshot)
            return dict(x=x, y=y, z=.04, yaw=yaw)
        with patch.object(self.window.session, 'reposition', side_effect=reposition):
            self.window.place_car(1., -1., .3)
        self.assertEqual(calls, ['save', 'move'])
        self.fake_controller.reset.assert_called_once()
        self.fake_telemetry.reset.assert_called_once_with(preserve_clock=True)
        self.assertEqual(self.window.map.trail, [])
        self.assertEqual(self.window.map.position, (1., -1., .3))
        self.assertEqual(self.window.map.points, [[.5, .5]])

    def test_recording_save_failure_does_not_teleport(self):
        self.window.session.process = SimpleNamespace(pid=99999, poll=lambda: None)
        self.fake_telemetry.snapshot.return_value = {'recording': True}
        self.fake_telemetry.stop_recording.return_value = {'recording_complete': False}
        self.window.work = lambda operation, callback=None: callback(operation())
        with patch.object(self.window.session, 'reposition') as reposition:
            with self.assertRaisesRegex(RuntimeError, 'finalized'):
                self.window.place_car(1., 1., 0.)
            reposition.assert_not_called()

    def test_load_applies_selected_wheels_and_cameras_and_refreshes_history(self):
        self.fake_telemetry.snapshot.return_value = {'recording': False}
        self.window.caster_box.setCurrentIndex(self.window.caster_box.findData('swivel'))
        self.window.cameras_box.setChecked(False)
        self.window.session.terrain = self.window.session.terrains['flat']
        self.window.session.process = SimpleNamespace(pid=99999, poll=lambda: None)
        self.window.work = lambda operation, callback=None: callback(operation())
        with patch.object(self.window.session, 'stop'), \
             patch.object(self.window.session, 'start', return_value=Path('/tmp/example')) as start, \
             patch.object(self.window.gazebo, 'detach'), \
             patch.object(self.window.gazebo, 'attach_process'):
            self.window.load_terrain()
        start.assert_called_once_with('flat', gui=False, caster='swivel', cameras=False)
        self.fake_telemetry.configure.assert_called_once_with('swivel', cameras=False)
        self.assertEqual(self.window.encoder_title.text(), 'LEFT / RIGHT / TURRET')
        self.assertEqual(self.window.history.joint_names[0], 'left_caster_swivel_joint')
        self.assertTrue(self.window.camera_panel.isHidden())

    def test_speed_dialog_allows_raising_conservative_ceiling(self):
        def accept(dialog):
            form = next(layout for layout in dialog.findChildren(QtWidgets.QFormLayout))
            values = {'Maximum requested speed': .8, 'Wheel command limit': 12.,
                      'Planner wheel limit': 10., 'Wheel acceleration limit': 60.,
                      'Planner wheel acceleration': 40., 'Trajectory acceleration': .5}
            for row in range(form.rowCount()):
                name = form.itemAt(row, QtWidgets.QFormLayout.LabelRole).widget().text()
                form.itemAt(row, QtWidgets.QFormLayout.FieldRole).widget().setValue(values[name])
            return QtWidgets.QDialog.Accepted
        with patch.object(QtWidgets.QDialog, 'exec_', accept):
            self.window.edit_limits()
        limits = self.fake_controller.configure_limits.call_args.args[0]
        self.assertEqual(limits.max_speed, .8)
        self.assertEqual(limits.planning_wheel_rate, 10.)
        self.assertEqual(self.window.route_speed.maximum(), .8)
        self.assertEqual(self.window.manual_speed.maximum(), .8)
        self.fake_controller.halt.assert_called_once()


if __name__ == "__main__":
    unittest.main()
