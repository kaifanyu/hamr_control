"""One-window local experiment console; Gazebo remains the real renderer."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

from PyQt5 import QtCore, QtGui, QtWidgets

from .session import SimulationSession
from .widgets import CameraView, HistoryPlot, TerrainMap

from .theme import BG, SURFACE, RAISED, BORDER, TEXT, MUTED, ACCENT, SAND, DANGER

STYLE = """
QWidget { color: @TEXT; font-family: 'DejaVu Sans'; font-size: 12px; }
QMainWindow, QWidget#workspace, QWidget#sidebar, QScrollArea { background: @BG; }
QDialog, QMenu { background: @SURFACE; }
QFrame#card { background: @SURFACE; border: 1px solid @BORDER; border-radius: 8px; }
QLabel { background: transparent; border: none; }
QLabel#brand { color: @ACCENT; font-size: 14px; font-weight: 700; }
QLabel#title { font-size: 23px; font-weight: 600; }
QLabel#subtitle, QLabel#muted { color: @MUTED; font-size: 11px; }
QLabel#section { color: @MUTED; font-weight: 600; font-size: 10px; padding: 2px 0; }
QLabel#status { color: @ACCENT; font-size: 11px; }
QLabel#metric { font-size: 22px; font-weight: 500; color: @TEXT; }
QPushButton { border: 1px solid @BORDER; background: @RAISED; border-radius: 5px; padding: 7px 10px; min-height: 18px; }
QPushButton:hover { background: #353f2c; border-color: #6f7c57; }
QPushButton:pressed, QPushButton:checked { background: #424e34; border-color: @ACCENT; }
QPushButton:disabled { color: #747b68; background: @SURFACE; border-color: #30372b; }
QPushButton#primary { color: @BG; background: @ACCENT; border-color: @ACCENT; font-weight: 600; }
QPushButton#primary:hover { background: #c4cf9c; }
QPushButton#primary:disabled { color: #8d957b; background: #343e2b; border-color: #343e2b; }
QPushButton#stop { color: #e7b6a5; background: #3b2821; border-color: #785344; }
QPushButton#stop:hover { background: #563a30; }
QPushButton#quiet { color: @MUTED; background: transparent; border-color: transparent; padding: 5px 7px; }
QPushButton#quiet:hover { color: @TEXT; background: @RAISED; }
QComboBox, QDoubleSpinBox { background: @RAISED; border: 1px solid @BORDER; border-radius: 5px; padding: 6px 8px; min-height: 18px; selection-background-color: #53613f; }
QComboBox:hover, QDoubleSpinBox:hover { border-color: #6f7c57; }
QComboBox:focus, QDoubleSpinBox:focus { border-color: @ACCENT; }
QComboBox::drop-down { border: none; width: 22px; }
QComboBox::down-arrow, QDoubleSpinBox::down-arrow { image: url('@ASSETS/chevron_down.svg'); width: 12px; height: 12px; }
QDoubleSpinBox::up-arrow { image: url('@ASSETS/chevron_up.svg'); width: 12px; height: 12px; }
QComboBox QAbstractItemView { background: @RAISED; color: @TEXT; selection-background-color: #53613f; border: 1px solid @BORDER; padding: 4px; }
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { width: 17px; border-left: 1px solid @BORDER; background: @RAISED; }
QTabWidget::pane { border: none; background: @SURFACE; }
QTabBar::tab { background: @RAISED; color: @MUTED; border: 1px solid @BORDER; padding: 8px 14px; min-width: 78px; margin-right: 4px; border-radius: 5px; }
QTabBar::tab:selected { background: #3b472d; border-color: #74845a; color: @TEXT; }
QSplitter::handle { background: @BG; }
QCheckBox { spacing: 7px; padding: 4px 0; }
QCheckBox::indicator { width: 14px; height: 14px; border: 1px solid #6f7c57; border-radius: 3px; background: @RAISED; }
QCheckBox::indicator:checked { background: @ACCENT; border-color: @ACCENT; image: url('@ASSETS/check.svg'); }
QCheckBox:disabled { color: #747b68; }
QCheckBox::indicator:disabled { background: @SURFACE; border-color: @BORDER; }
QMenu { border: 1px solid @BORDER; padding: 5px; }
QMenu::item { padding: 7px 18px; border-radius: 3px; }
QMenu::item:selected { background: #3b472d; }
QMenu::separator { height: 1px; background: @BORDER; margin: 4px 8px; }
QScrollBar:vertical { background: @BG; width: 7px; margin: 0; }
QScrollBar::handle:vertical { background: @BORDER; border-radius: 3px; min-height: 25px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }
QStatusBar { background: @BG; color: @MUTED; font-size: 11px; }
QToolTip { background: @RAISED; color: @TEXT; border: 1px solid @BORDER; padding: 5px; }
"""
for _name in ('BG', 'SURFACE', 'RAISED', 'BORDER', 'TEXT', 'MUTED', 'ACCENT', 'SAND', 'DANGER'):
    STYLE = STYLE.replace('@' + _name, globals()[_name])
STYLE = STYLE.replace('@ASSETS', (Path(__file__).resolve().parent / 'assets').as_posix())



class Events(QtCore.QObject):
    result = QtCore.pyqtSignal(object, object)
    image = QtCore.pyqtSignal(str, object)


def label(text, name=None):
    w = QtWidgets.QLabel(text)
    if name:
        w.setObjectName(name)
    w.setWordWrap(name not in ('title', 'subtitle'))
    return w


def button(text, callback, name=None):
    w = QtWidgets.QPushButton(text)
    w.setFocusPolicy(QtCore.Qt.NoFocus)
    if name:
        w.setObjectName(name)
    w.clicked.connect(callback)
    return w


class Console(QtWidgets.QMainWindow):
    def __init__(self, args):
        super().__init__()
        from .telemetry import RosTelemetry
        from .controller import LiveController, limits_from_args
        from .gazebo_view import GazeboView
        from .markers import MarkerPublisher
        self.args = args
        self.session = SimulationSession(args.root, args.output)
        for key in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_LOCALHOST_ONLY', 'GZ_PARTITION', 'GZ_SIM_RESOURCE_PATH'):
            os.environ[key] = self.session.env[key]
        self.events = Events()
        self.events.result.connect(self._finished)
        self.events.image.connect(self._image)
        # Keep only the newest camera frame. Queuing every frame while a window
        # is being resized would otherwise retain an unbounded amount of memory.
        self._image_lock = threading.Lock()
        self._images = {}
        self.loaded_caster = getattr(args, 'caster', 'ball')
        self.loaded_cameras = getattr(args, 'cameras', 'off') == 'on'
        self.limits = limits_from_args(args)
        self.telemetry = RosTelemetry(caster=self.loaded_caster, cameras=self.loaded_cameras,
                                      on_image=self._receive_image, preview_cameras=('closeup',))
        self.controller = LiveController(self.telemetry.node, args.root, limits=self.limits)
        self.markers = MarkerPublisher(self.session.env)
        self.busy, self.closing = False, False
        self._shutdown_done = False
        self.keys = set()
        self.loaded_terrain = None
        self.recording_dir = None
        self.last_recording = None
        self.last_error = ''
        self._last_marker = 0
        self._last_manual = 0
        self._height_grid = None
        self.setWindowTitle('HAMR · Simulation Studio')
        self.resize(1650, 1000)
        self.setMinimumSize(1100, 750)
        self.setStyleSheet(STYLE)
        root = QtWidgets.QWidget()
        root.setObjectName('workspace')
        outer = QtWidgets.QVBoxLayout(root)
        outer.setContentsMargins(18, 14, 18, 10)
        heading = QtWidgets.QHBoxLayout()
        heading.setSpacing(16)
        brand = label('HAMR', 'brand')
        heading.addWidget(brand)
        title = QtWidgets.QVBoxLayout()
        title.setSpacing(2)
        title.addWidget(label('Simulation Studio', 'title'))
        self.subtitle = label('Ball caster · Live simulation', 'subtitle')
        title.addWidget(self.subtitle)
        heading.addLayout(title, 1)
        self.state_label = label('●  READY', 'status')
        self.state_label.setWordWrap(False)
        heading.addWidget(self.state_label)
        self.pause_btn = button('Pause', self.pause)
        self.pause_btn.setToolTip('Pause or resume the physics clock.')
        heading.addWidget(self.pause_btn)
        self.stop_btn = button('Stop  /  Space', self.stop_motion, 'stop')
        heading.addWidget(self.stop_btn)
        outer.addLayout(heading)
        outer.addSpacing(6)
        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        split.setHandleWidth(12)
        outer.addWidget(split, 1)
        sidebar = QtWidgets.QWidget()
        sidebar.setObjectName('sidebar')
        controls = QtWidgets.QVBoxLayout(sidebar)
        controls.setContentsMargins(0, 0, 0, 0)
        controls.setSpacing(12)

        def card(title):
            panel = QtWidgets.QFrame()
            panel.setObjectName('card')
            layout = QtWidgets.QVBoxLayout(panel)
            layout.setContentsMargins(14, 12, 14, 14)
            layout.setSpacing(10)
            if title:
                layout.addWidget(label(title, 'section'))
            controls.addWidget(panel)
            return layout

        world = card('WORLD')
        self.terrain_box = QtWidgets.QComboBox()
        self.terrain_box.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.terrain_box.setMinimumContentsLength(12)
        self.terrain_box.setAccessibleName('Terrain map')
        for terrain in self.session.terrains.values():
            self.terrain_box.addItem(terrain.title, terrain.id)
        default = self.terrain_box.findData(args.terrain)
        if default >= 0:
            self.terrain_box.setCurrentIndex(default)
        world.addWidget(self.terrain_box)
        self.caster_box = QtWidgets.QComboBox()
        self.caster_box.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.caster_box.setMinimumContentsLength(12)
        self.caster_box.setAccessibleName('Caster wheel type')
        self.caster_box.addItem('Ball casters', 'ball')
        self.caster_box.addItem('Swivel casters', 'swivel')
        self.caster_box.setCurrentIndex(self.caster_box.findData(self.loaded_caster))
        self.caster_box.setToolTip('Wheel type applies when you load the world.')
        world.addWidget(self.caster_box)
        world_actions = QtWidgets.QHBoxLayout()
        self.cameras_box = QtWidgets.QCheckBox('Cameras')
        self.cameras_box.setChecked(self.loaded_cameras)
        self.cameras_box.setToolTip('Enable both camera sensors. Apply with Load world; off removes camera rendering.')
        world_actions.addWidget(self.cameras_box)
        world_actions.addStretch()
        self.load_btn = button('Load world', self.load_terrain)
        self.load_btn.setToolTip('Apply map, wheels and cameras. Stops motion and saves any active recording.')
        world_actions.addWidget(self.load_btn)
        world.addLayout(world_actions)
        self.terrain_box.currentIndexChanged.connect(self._terrain_selected)

        motion = card(None)
        motion_heading = QtWidgets.QHBoxLayout()
        motion_heading.addWidget(label('CONTROL', 'section'), 1)
        self.limits_btn = button('Limits…', self.edit_limits, 'quiet')
        self.limits_btn.setToolTip('Speed ceiling and actuator limits')
        motion_heading.addWidget(self.limits_btn)
        motion.addLayout(motion_heading)
        self.control_tabs = QtWidgets.QTabWidget()
        self.control_tabs.setDocumentMode(True)
        self.control_tabs.tabBar().setDrawBase(False)
        motion.addWidget(self.control_tabs)
        route_page = QtWidgets.QWidget()
        route_layout = QtWidgets.QVBoxLayout(route_page)
        route_layout.setContentsMargins(0, 14, 0, 0)
        route_layout.setSpacing(12)
        preset_row = QtWidgets.QHBoxLayout()
        self.preset = QtWidgets.QComboBox()
        self.preset.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.preset.setMinimumContentsLength(12)
        self.preset.addItems(['Square · 0.8 m', 'Straight · 1.5 m', 'Out & back · 1 m', 'Circle · r 0.6 m'])
        self.preset.setAccessibleName('Trajectory preset')
        preset_row.addWidget(self.preset, 1)
        preset_btn = button('Set', self.use_preset)
        preset_btn.setToolTip('Place this preset from the current car position.')
        preset_row.addWidget(preset_btn)
        route_layout.addLayout(preset_row)
        speed_row = QtWidgets.QHBoxLayout()
        speed_row.addWidget(label('Speed'), 1)
        self.route_speed = QtWidgets.QDoubleSpinBox()
        self.route_speed.setRange(.02, self.limits.max_speed)
        self.route_speed.setSingleStep(.02)
        self.route_speed.setValue(getattr(args, 'speed', .15))
        self.route_speed.setSuffix(' m/s')
        self.route_speed.setToolTip('Requested peak speed. The planner also respects wheel and acceleration limits.')
        self.route_speed.valueChanged.connect(self._route_speed_changed)
        speed_row.addWidget(self.route_speed)
        route_layout.addLayout(speed_row)
        route_tools = QtWidgets.QHBoxLayout()
        route_tools.addWidget(button('Undo', self.undo_point))
        route_tools.addWidget(button('Clear', lambda: self.map.set_points([])))
        route_files = QtWidgets.QPushButton('Route')
        route_files.setFocusPolicy(QtCore.Qt.NoFocus)
        route_menu = QtWidgets.QMenu(route_files)
        route_menu.addAction('Save route…', self.save_route)
        route_menu.addAction('Load route…', self.open_route)
        route_files.setMenu(route_menu)
        route_tools.addWidget(route_files)
        route_layout.addLayout(route_tools)
        self.apply_btn = button('Drive route', self.apply_route, 'primary')
        route_layout.addWidget(self.apply_btn)
        self.plan_note = label('', 'muted')
        self.plan_note.hide()
        route_layout.addWidget(self.plan_note)
        route_layout.addStretch()
        self.control_tabs.addTab(route_page, 'Trajectory')

        manual_page = QtWidgets.QWidget()
        manual_layout = QtWidgets.QVBoxLayout(manual_page)
        manual_layout.setContentsMargins(0, 14, 0, 0)
        manual_layout.setSpacing(12)
        manual_form = QtWidgets.QFormLayout()
        manual_form.setVerticalSpacing(12)
        self.manual_speed = QtWidgets.QDoubleSpinBox()
        self.manual_speed.setRange(.02, self.limits.max_speed)
        self.manual_speed.setValue(getattr(args, 'speed', .15))
        self.manual_speed.setSingleStep(.02)
        self.manual_speed.setSuffix(' m/s')
        manual_form.addRow('Speed', self.manual_speed)
        self.turn_speed = QtWidgets.QDoubleSpinBox()
        self.turn_speed.setRange(.1, 1.0)
        self.turn_speed.setValue(.5)
        self.turn_speed.setSingleStep(.1)
        self.turn_speed.setSuffix(' rad/s')
        manual_form.addRow('Turn', self.turn_speed)
        manual_layout.addLayout(manual_form)
        self.manual_btn = button('Enable WASD', self.toggle_manual, 'primary')
        self.manual_btn.setCheckable(True)
        self.manual_btn.setToolTip('Hold keys to drive. Release to stop. Selecting the 3D view stops held commands.')
        manual_layout.addWidget(self.manual_btn)
        manual_layout.addWidget(label('W/S  Move    A/D  Turn    Q/E  Strafe', 'muted'))
        manual_layout.addStretch()
        self.control_tabs.addTab(manual_page, 'Manual')
        self.control_tabs.currentChanged.connect(self._control_tab_changed)

        capture = card('RECORDING')
        record_row = QtWidgets.QHBoxLayout()
        self.record_btn = button('●  Record', self.toggle_recording, 'record')
        self.record_btn.setToolTip('Save ROS bag, velocity, joints and encoder packets locally.')
        record_row.addWidget(self.record_btn, 1)
        self.video_box = QtWidgets.QCheckBox('MP4')
        self.video_box.setChecked(self.loaded_cameras)
        self.video_box.setToolTip('Include overview and caster videos. Requires camera sensors.')
        record_row.addWidget(self.video_box)
        capture.addLayout(record_row)
        recording_tools = QtWidgets.QHBoxLayout()
        recording_tools.addWidget(button('Replay…', self.replay))
        recording_tools.addWidget(button('Files', self.open_output))
        capture.addLayout(recording_tools)
        controls.addStretch()
        sidebar_scroll = QtWidgets.QScrollArea()
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        sidebar_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        sidebar_scroll.setMinimumWidth(292)
        sidebar_scroll.setMaximumWidth(330)
        sidebar_scroll.setWidget(sidebar)
        split.addWidget(sidebar_scroll)
        center = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.gazebo = GazeboView()
        self.gazebo.status_changed.connect(lambda text: self.statusBar().showMessage(text, 8000))
        center.addWidget(self.gazebo)
        measurements = QtWidgets.QWidget()
        measurements.setMinimumHeight(200)
        metrics_layout = QtWidgets.QVBoxLayout(measurements)
        metrics_layout.setContentsMargins(0, 8, 0, 0)
        metrics = QtWidgets.QHBoxLayout()
        self.metrics = {}
        for key, name in [('speed', 'SPEED / m/s'), ('yaw_rate', 'YAW / rad/s'), ('sim_time', 'TIME / s'), ('encoder', 'H1 / H2 / CENTER')]:
            item = QtWidgets.QVBoxLayout()
            metric_title = label(name, 'section')
            if key == 'encoder':
                self.encoder_title = metric_title
            item.addWidget(metric_title)
            self.metrics[key] = label('—', 'metric')
            self.metrics[key].setWordWrap(False)
            if key == 'encoder':
                self.metrics[key].setStyleSheet(f'font-size: 13px; color: {TEXT};')
            item.addWidget(self.metrics[key])
            metrics.addLayout(item)
        metrics_layout.addLayout(metrics)
        self.history = HistoryPlot()
        metrics_layout.addWidget(self.history)
        center.addWidget(measurements)
        center.setSizes([650, 230])
        split.addWidget(center)
        right = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        map_panel = QtWidgets.QWidget()
        map_layout = QtWidgets.QVBoxLayout(map_panel)
        map_layout.setContentsMargins(10, 0, 0, 0)
        map_heading = QtWidgets.QHBoxLayout()
        map_heading.addWidget(label('WAYPOINT MAP', 'section'), 1)
        self.place_btn = button('Place car', self.toggle_placement)
        self.place_btn.setCheckable(True)
        self.place_btn.setToolTip('Click the map to move the car. Drag to set its heading. Saves any active recording first.')
        map_heading.addWidget(self.place_btn)
        map_layout.addLayout(map_heading)
        map_layout.addWidget(label('Olive / route    Sand / driven', 'muted'))
        self.map = TerrainMap()
        self.map.setToolTip('Click to add a waypoint. Drag to move; right-click to remove.')
        self.map.changed.connect(self._points_changed)
        self.map.placement_requested.connect(self.place_car)
        self.map.placement_cancelled.connect(self.cancel_placement)
        map_layout.addWidget(self.map, 1)
        self.point_count = label('0 waypoints', 'muted')
        map_layout.addWidget(self.point_count)
        right.addWidget(map_panel)
        self.camera_panel = QtWidgets.QWidget()
        camera_layout = QtWidgets.QVBoxLayout(self.camera_panel)
        camera_layout.setContentsMargins(10, 0, 0, 0)
        camera_layout.addWidget(label('FRONT CASTER / SIDE CAMERA', 'section'))
        self.camera = CameraView()
        camera_layout.addWidget(self.camera, 1)
        right.addWidget(self.camera_panel)
        right.setSizes([500, 330])
        split.addWidget(right)
        split.setSizes([310, 880, 440])
        self.setCentralWidget(root)
        self.statusBar().showMessage('Ready')
        QtWidgets.QApplication.instance().installEventFilter(self)
        self._terrain_selected()
        self._refresh_model_ui()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        if not args.no_autostart:
            QtCore.QTimer.singleShot(300, self.load_terrain)

    def _refresh_model_ui(self):
        ball = self.loaded_caster == 'ball'
        self.subtitle.setText(('Ball casters' if ball else 'Swivel casters') +
                              (' / Scene replay' if self.session.replay else ' / Live simulation'))
        self.encoder_title.setText('H1 / H2 / CENTER' if ball else 'LEFT / RIGHT / TURRET')
        self.history.configure(self.loaded_caster)
        self.camera.clear()
        self.camera_panel.setVisible(self.loaded_cameras)
        if not self.loaded_cameras:
            self.video_box.setChecked(False)
            self.camera.setText('Camera sensors disabled\nBags, encoder packets and scene replay remain available.')
        self.video_box.setEnabled(self.loaded_cameras)

    def _show_plan(self):
        plan = self.controller.snapshot().get('plan_summary')
        if plan:
            self.plan_note.setText(f"Planned peak  {plan['planned_peak_speed_m_s']:.2f} m/s")
            self.plan_note.setToolTip(f"Requested {plan['requested_peak_speed_m_s']:.2f} m/s; wheel and acceleration limits apply.")
            self.plan_note.show()
        else:
            self.plan_note.hide()

    def edit_limits(self):
        from .controller import MotionLimits
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle('Simulation speed and actuator limits')
        layout = QtWidgets.QVBoxLayout(dialog)
        layout.addWidget(label('Simulation limits · Applying stops motion.', 'muted'))
        form = QtWidgets.QFormLayout()
        fields = {}
        specs = [('max_speed', 'Maximum requested speed', ' m/s', 10.),
                 ('max_wheel_rate', 'Wheel command limit', ' rad/s', 300.),
                 ('planning_wheel_rate', 'Planner wheel limit', ' rad/s', 300.),
                 ('max_wheel_acceleration', 'Wheel acceleration limit', ' rad/s²', 500.),
                 ('planning_wheel_acceleration', 'Planner wheel acceleration', ' rad/s²', 500.),
                 ('max_translational_acceleration', 'Trajectory acceleration', ' m/s²', 20.)]
        for key, title, units, maximum in specs:
            field = QtWidgets.QDoubleSpinBox()
            field.setDecimals(3)
            field.setRange(.02 if key == 'max_speed' else .001, maximum)
            field.setValue(getattr(self.limits, key))
            field.setSuffix(units)
            fields[key] = field
            form.addRow(title, field)
        layout.addLayout(form)
        actions = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        actions.accepted.connect(dialog.accept)
        actions.rejected.connect(dialog.reject)
        layout.addWidget(actions)
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        try:
            limits = MotionLimits(**{key: field.value() for key, field in fields.items()})
            self.stop_motion()
            self.controller.configure_limits(limits)
            self.limits = limits
            self.route_speed.setMaximum(limits.max_speed)
            self.manual_speed.setMaximum(limits.max_speed)
            self.plan_note.hide()
            self.statusBar().showMessage('Motion limits updated')
        except ValueError as error:
            self.error(error)

    def _receive_image(self, camera, frame):
        with self._image_lock:
            self._images[camera] = frame

    def _image(self, camera, frame):
        if camera == 'closeup' and self.loaded_cameras and not self.session.replay:
            self.camera.set_frame(frame)

    def _terrain_selected(self):
        terrain = self.session.terrains.get(self.terrain_box.currentData())
        if terrain:
            self.terrain_box.setToolTip(terrain.description or 'Flat floor for route and keyboard testing.')
            if self.loaded_terrain is None:
                self.map.set_terrain(terrain)

    def work(self, operation, callback=None):
        if self.busy:
            return
        self.busy = True
        def run():
            try:
                result = operation()
                self.events.result.emit(callback, (True, result))
            except Exception as error:
                self.events.result.emit(callback, (False, str(error)))
        threading.Thread(target=run, daemon=True).start()

    def _finished(self, callback, result):
        self.busy = False
        ok, value = result
        if ok:
            if callback:
                callback(value)
        else:
            self.error(value)
        if self.closing:
            self._begin_shutdown()

    def error(self, text):
        self.last_error = str(text)
        self.statusBar().showMessage(self.last_error)
        QtWidgets.QMessageBox.warning(self, 'Simulation Studio', self.last_error)

    def _finalize_recording(self):
        if self.telemetry.snapshot(include_history=False)['recording']:
            return self.telemetry.stop_recording()

    def load_terrain(self):
        if self.busy:
            return
        terrain_id = self.terrain_box.currentData()
        caster, cameras = self.caster_box.currentData(), self.cameras_box.isChecked()
        self.stop_motion()
        self.gazebo.detach()
        self.state_label.setText('●  LOADING WORLD…')
        def start():
            self._finalize_recording()
            self.session.stop()
            self.controller.reset()
            self.telemetry.configure(caster, cameras=cameras)
            with self._image_lock:
                self._images.clear()
            return self.session.start(terrain_id, gui=not self.args.headless, caster=caster, cameras=cameras)
        def ready(path):
            self.loaded_terrain = self.session.terrain
            self.loaded_caster, self.loaded_cameras = caster, cameras
            self._refresh_model_ui()
            self.markers.reset()
            self.map.set_terrain(self.loaded_terrain)
            self._points_changed([])
            self.history.samples.clear()
            self.history.last_time = None
            self._load_heights()
            self.gazebo.attach_process(self.session.process.pid)
            self.statusBar().showMessage(f'World loaded · {path}')
            self.recording_dir = None
            self.camera.source = None
            self.camera.clear()
            self.camera.setText('Waiting for the caster camera…' if self.loaded_cameras else 'Camera sensors disabled\nBags, encoder packets and scene replay remain available.')
        self.work(start, ready)

    def _load_heights(self):
        import numpy as np
        self._height_grid = None
        if self.loaded_terrain and self.loaded_terrain.height_grid:
            with np.load(self.loaded_terrain.height_grid) as grid:
                self._height_grid = {k: grid[k].copy() for k in ('x', 'y', 'z')}

    def terrain_height(self, x, y):
        import numpy as np
        if self._height_grid is None:
            return .04
        grid = self._height_grid
        i = int(np.clip(np.searchsorted(grid['x'], x), 0, len(grid['x'])-1))
        j = int(np.clip(np.searchsorted(grid['y'], y), 0, len(grid['y'])-1))
        return float(grid['z'][j, i])+.05

    def _points_changed(self, points):
        self.point_count.setText(f'{len(points)} waypoint'+('s' if len(points) != 1 else '')+' · draft')
        if self.session.running and not self.session.replay:
            self.markers.set_route(points, self.terrain_height)

    def use_preset(self):
        state = self.telemetry.snapshot(include_history=False)
        if state['x'] is None:
            self.error('Load a world and wait for the car before placing a relative preset.')
            return
        from .presets import PRESET_NAMES, preset_points
        points = preset_points(PRESET_NAMES[self.preset.currentIndex()], (state['x'], state['y']), state['yaw'] or 0.)
        if not self.points_inside(points):
            self.error('This preset extends beyond the terrain. Move the car or draw a smaller route.')
            return
        self.map.set_points(points)

    def points_inside(self, points):
        x0,x1,y0,y1 = self.map.bounds
        return all(len(p)==2 and all(math.isfinite(v) for v in p) and x0<=p[0]<=x1 and y0<=p[1]<=y1 for p in points)

    def undo_point(self):
        self.map.set_points(self.map.points[:-1])

    def cancel_placement(self):
        self.place_btn.setChecked(False)
        self.place_btn.setText('Place car')
        self.map.set_placement_mode(False)

    def toggle_placement(self):
        enabled = self.place_btn.isChecked()
        self.stop_motion()
        if not enabled:
            return
        state = self.telemetry.snapshot(include_history=False, include_trail=False)
        if self.busy or not self.session.running or self.session.replay or not state.get('odom_received'):
            self.statusBar().showMessage('Wait for the live car before placing it.')
            return
        self.place_btn.setChecked(True)
        self.place_btn.setText('Cancel place')
        self.map.set_placement_mode(True, yaw=state['yaw'] or 0.)
        self.statusBar().showMessage('Click to place · Drag for heading · Esc to cancel')

    def place_car(self, x, y, yaw):
        if self.busy or not self.session.running or self.session.replay:
            self.cancel_placement()
            return
        self.stop_motion()
        self.map.editable = False
        self.state_label.setText('●  PLACING CAR')
        def move():
            report = self._finalize_recording()
            if report:
                self.last_recording = self.recording_dir
                if not report.get('recording_complete', False):
                    raise RuntimeError('Recording could not be finalized; inspect recording.json before placing the car.')
            try:
                pose = self.session.reposition(x, y, yaw, feedback=self.controller.snapshot)
            finally:
                # A failed verification can follow an accepted native move;
                # never differentiate velocity across that uncertain jump.
                self.controller.reset()
                self.telemetry.reset(preserve_clock=True)
            return pose
        def ready(pose):
            self.markers.reset()
            self.markers.set_trace([])
            self.markers.set_route(self.map.points, self.terrain_height)
            self.map.trail = []
            self.map.position = (pose['x'], pose['y'], pose['yaw'])
            self.map.update()
            self.history.configure(self.loaded_caster)
            self._last_marker = 0.
            with self._image_lock:
                self._images.clear()
            self.recording_dir = None
            self.statusBar().showMessage(f"Car placed at {pose['x']:.2f}, {pose['y']:.2f} m · {math.degrees(pose['yaw']):.0f}°")
        self.work(move, ready)

    def apply_route(self):
        if not self.session.running or self.session.replay or self.busy:
            return
        try:
            if not self.points_inside(self.map.points):
                raise ValueError('Waypoints must stay inside the current terrain bounds.')
            self.stop_motion()
            points = self.controller.set_route(self.map.points, self.route_speed.value())
            self.markers.set_route(points, self.terrain_height)
            self._show_plan()
            self.statusBar().showMessage('Route applied from the measured car position. Edit the draft map at any time.')
        except Exception as error:
            self.error(error)

    def _route_speed_changed(self, speed):
        if hasattr(self, 'controller'):
            try:
                self.controller.set_speed(speed)
                self._show_plan()
            except Exception as error:
                self.error(error)

    def _control_tab_changed(self, index):
        if index != 1 and self.manual_btn.isChecked():
            self.stop_motion()

    def toggle_manual(self):
        self.cancel_placement()
        self.controller.halt()
        self.keys.clear()
        self.manual_btn.setText('WASD active' if self.manual_btn.isChecked() else 'Enable WASD')
        self.setFocus()

    def stop_motion(self):
        self.cancel_placement()
        self.keys.clear()
        self.manual_btn.setChecked(False)
        self.manual_btn.setText('Enable WASD')
        self.controller.halt()
        self.plan_note.hide()
        self.statusBar().showMessage('Motion stopped')

    def eventFilter(self, obj, event):
        kind = event.type()
        if kind in (QtCore.QEvent.ApplicationDeactivate, QtCore.QEvent.WindowDeactivate, QtCore.QEvent.FocusOut):
            if self.keys:
                self.keys.clear()
                self.controller.halt()
        if kind in (QtCore.QEvent.KeyPress, QtCore.QEvent.KeyRelease):
            key = event.key()
            if key == QtCore.Qt.Key_Escape and self.map.placement_mode and kind == QtCore.QEvent.KeyPress:
                self.cancel_placement()
                self.statusBar().showMessage('Placement cancelled')
                return True
            if key == QtCore.Qt.Key_Space and kind == QtCore.QEvent.KeyPress:
                self.stop_motion()
                return True
            allowed = {QtCore.Qt.Key_W, QtCore.Qt.Key_A, QtCore.Qt.Key_S, QtCore.Qt.Key_D, QtCore.Qt.Key_Q, QtCore.Qt.Key_E}
            if self.manual_btn.isChecked() and key in allowed:
                if event.isAutoRepeat():
                    return True
                if kind == QtCore.QEvent.KeyPress:
                    self.keys.add(key)
                else:
                    self.keys.discard(key)
                    if not self.keys:
                        self.controller.halt()
                return True
        return super().eventFilter(obj, event)

    def pause(self):
        self.keys.clear()
        if self.manual_btn.isChecked():
            self.controller.halt()
        paused = not self.session.paused
        self.work(lambda: self.session.set_paused(paused))

    def toggle_recording(self):
        if self.busy or not self.session.running or self.session.replay:
            return
        if self.telemetry.snapshot(include_history=False)['recording']:
            self.work(self.telemetry.stop_recording, self._recorded)
        else:
            output = self.session.run_dir / datetime.now().strftime('recording_%H%M%S_%f')
            metadata = {'gazebo_log': str(self.session.gazebo_log), 'terrain': self.loaded_terrain.id,
                        'session_dir': str(self.session.run_dir), 'route_points': self.map.points,
                        'route_speed_m_s': self.route_speed.value(),
                        'caster': self.loaded_caster, 'cameras': self.loaded_cameras,
                        'motion_limits': self.limits.to_dict(),
                        'scene_log_scope': 'Full session; bag and MP4 cover this recording interval'}
            self.recording_dir = output
            video = self.loaded_cameras and self.video_box.isChecked()
            self.work(lambda: self.telemetry.start_recording(output, video=video, metadata=metadata),
                      lambda path: self.statusBar().showMessage(f'Recording · {path}'))

    def _recorded(self, summary):
        self.last_recording = self.recording_dir
        self.statusBar().showMessage(f'Saved recording · {self.recording_dir}')
        errors = [v for v in (summary or {}).get('video', []) if v.get('returncode') != 0 or not v.get('exists')]
        errors.extend((summary or {}).get('errors', []))
        if errors:
            self.error('Data saved, but video verification failed. Inspect recording.json and video logs.')

    def replay(self):
        if self.busy:
            return
        path = QtWidgets.QFileDialog.getExistingDirectory(self, 'Choose session or recording folder', str(self.session.output_root))
        if not path:
            return
        self.stop_motion()
        self.gazebo.detach()
        def start():
            self._finalize_recording()
            self.controller.reset()
            self.telemetry.reset()
            return self.session.start_replay(path, gui=not self.args.headless)
        def ready(_):
            self.markers.reset()
            self.loaded_terrain = self.session.terrain
            self.loaded_caster, self.loaded_cameras = self.session.caster, self.session.cameras
            self.caster_box.setCurrentIndex(self.caster_box.findData(self.loaded_caster))
            self.cameras_box.setChecked(self.loaded_cameras)
            self._refresh_model_ui()
            if self.loaded_terrain:
                self.map.set_terrain(self.loaded_terrain)
                self._load_heights()
                index = self.terrain_box.findData(self.loaded_terrain.id)
                if index >= 0:
                    self.terrain_box.setCurrentIndex(index)
            self.map.set_points([], emit=False)
            self.map.trail, self.map.position = [], None
            self.history.samples.clear()
            self.history.last_time = None
            self.history.update()
            self.point_count.setText('Scene replay · map editing disabled')
            with self._image_lock:
                self._images.clear()
            self.gazebo.attach_process(self.session.process.pid)
            self.camera.source = None
            self.camera.clear()
            self.camera.setText('Scene replay\nIf recorded, open the saved caster MP4 for the close-up.'
                                if self.loaded_cameras else 'Scene replay\nCamera sensors were disabled for this session.')
            self.statusBar().showMessage('Replaying the full recorded Gazebo session. Live controls and telemetry are disabled.')
        self.work(start, ready)

    def save_route(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Save route', str(Path.home()/'hamr_route.json'), 'Route (*.json)')
        if path:
            try:
                Path(path).write_text(json.dumps({'terrain': self.loaded_terrain.id if self.loaded_terrain else self.terrain_box.currentData(), 'speed_m_s': self.route_speed.value(), 'points_m': self.map.points}, indent=2)+'\n')
            except Exception as error:
                self.error(error)

    def open_route(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Open route', str(Path.home()), 'Route (*.json)')
        if path:
            try:
                value = json.loads(Path(path).read_text())
                points = [[float(v) for v in p] for p in value['points_m']]
                if not points or not self.points_inside(points):
                    raise ValueError('Route has invalid coordinates or extends beyond the current map.')
                speed = float(value.get('speed_m_s', .15))
                self.limits.validate_speed(speed)
                active_terrain = self.loaded_terrain.id if self.loaded_terrain else self.terrain_box.currentData()
                if value.get('terrain') and value['terrain'] != active_terrain:
                    raise ValueError('Load the route’s terrain first: '+str(value['terrain']))
                self.map.set_points(points)
                self.route_speed.setValue(speed)
            except Exception as error:
                self.error(error)

    def open_output(self):
        self.session.output_root.mkdir(parents=True, exist_ok=True)
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(self.session.output_root)))

    def tick(self):
        state = self.telemetry.snapshot(include_history=False)
        running, replay = self.session.running, self.session.replay
        ready = running and not replay and not self.busy
        for w in (self.apply_btn, self.manual_btn):
            w.setEnabled(ready and not self.map.placement_mode)
        self.place_btn.setEnabled(ready and state.get('odom_received', False))
        self.record_btn.setEnabled(ready and (not self.session.paused or state['recording']))
        self.pause_btn.setEnabled(running and not self.busy)
        self.load_btn.setEnabled(not self.busy)
        self.limits_btn.setEnabled(not self.busy and not replay)
        self.video_box.setEnabled(self.loaded_cameras and not self.busy and not state['recording'] and not replay)
        self.map.editable = not replay and not self.busy
        self.pause_btn.setText('Resume' if self.session.paused else 'Pause')
        self.record_btn.setText('■  Save recording' if state['recording'] else '●  Record')
        self.record_btn.setStyleSheet(f'background: #50372b; border-color: {DANGER}; color: {TEXT};' if state['recording'] else '')
        if replay:
            self.state_label.setText('●  REPLAY')
            for metric in self.metrics.values():
                metric.setText('Replay')
        elif running:
            motion = self.controller.snapshot()
            suffix = 'PAUSED' if self.session.paused else motion['state'].upper()
            rtf = state.get('real_time_factor')
            pace = f' · {rtf:.2f}×' if rtf is not None else ''
            self.state_label.setText('●  '+suffix+pace+('  /  RECORDING' if state['recording'] else ''))
            self.metrics['sim_time'].setText(f"{state['sim_time']:.1f}")
            self.metrics['speed'].setText('—' if state['linear_speed'] is None else f"{state['linear_speed']:.3f}")
            self.metrics['yaw_rate'].setText('—' if state['yaw_rate'] is None else f"{state['yaw_rate']:+.2f}")
            values = []
            from .telemetry import JOINT_MAPPING
            for joint in JOINT_MAPPING[self.loaded_caster].values():
                value = state.get('joints', state['casters']).get(joint, {}).get('position')
                values.append('—' if value is None else f'{math.degrees(math.atan2(math.sin(value),math.cos(value))):+.0f}°')
            self.metrics['encoder'].setText(' / '.join(values))
            self.history.append(state)
            self.map.trail = state['trail']
            if state['x'] is not None:
                self.map.position = (state['x'],state['y'],state['yaw'])
            self.map.update()
            if time.monotonic()-self._last_marker > 1 and len(state['trail']) > 1:
                self._last_marker = time.monotonic()
                self.markers.set_trace(state['trail'], self.terrain_height)
            if motion.get('error'):
                self.statusBar().showMessage(motion['error'])
            if ready and self.manual_btn.isChecked() and self.keys and not self.session.paused:
                k = self.keys
                f = float(QtCore.Qt.Key_W in k)-float(QtCore.Qt.Key_S in k)
                turn = float(QtCore.Qt.Key_A in k)-float(QtCore.Qt.Key_D in k)
                side = float(QtCore.Qt.Key_Q in k)-float(QtCore.Qt.Key_E in k)
                try:
                    self.controller.manual(f*self.manual_speed.value(),turn*self.turn_speed.value(),side*self.manual_speed.value())
                except Exception as error:
                    self.stop_motion()
                    self.statusBar().showMessage(str(error))
            with self._image_lock:
                frames, self._images = self._images, {}
            for camera, frame in frames.items():
                self._image(camera, frame)
        elif not self.busy:
            self.state_label.setText('●  STOPPED')
        failure = None if self.busy else self.session.poll()
        if failure and failure != self.last_error:
            self.last_error = failure
            self.stop_motion()
            self.statusBar().showMessage(failure)
        if state.get('last_error'):
            self.statusBar().showMessage(state['last_error'])

    def closeEvent(self, event):
        if self._shutdown_done:
            self.timer.stop()
            self.markers.close()
            event.accept()
            return
        event.ignore()
        self.stop_motion()
        self.gazebo.detach()
        self.closing = True
        self.state_label.setText('●  SAVING & CLOSING…')
        self._begin_shutdown()

    def _begin_shutdown(self):
        if self.busy:
            return
        if self._shutdown_done:
            self.close()
            return
        def shutdown():
            try:
                self.telemetry.close()
            finally:
                self.session.stop()
                self._shutdown_done = True
        def done(_):
            self._shutdown_done = True
        self.work(shutdown, done)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--output', type=Path)
    parser.add_argument('--terrain', default='flat')
    parser.add_argument('--caster', choices=('ball', 'swivel'), default='ball')
    parser.add_argument('--cameras', choices=('on', 'off'), default='off', help='Camera sensors (default: off)')
    parser.add_argument('--speed', type=float, default=.15, help='Initial requested route/manual speed in m/s')
    from .controller import add_limit_arguments, limits_from_args
    add_limit_arguments(parser)
    parser.add_argument('--no-autostart', action='store_true')
    parser.add_argument('--headless', action='store_true', help='Run Gazebo without a native GUI (for automated integration tests)')
    parser.add_argument('--check', action='store_true', help='Check dependencies and map catalogue without opening windows')
    args = parser.parse_args()
    try:
        limits_from_args(args).validate_speed(args.speed)
    except ValueError as error:
        parser.error(str(error))
    if args.check:
        import importlib
        import shutil
        for module in ('rclpy','rosbag2_py','PyQt5.QtWidgets','numpy','reference_trajectory'):
            importlib.import_module(module)
            print(f'OK {module}')
        for executable in ('ros2','gz','ffmpeg'):
            if not shutil.which(executable):
                raise RuntimeError(f'Missing {executable}; launch through scripts/start.sh')
            print(f'OK {executable}')
        from .session import discover_terrains
        terrains = discover_terrains(args.root)
        if not terrains:
            raise RuntimeError('No terrain maps found')
        print(f'OK {len(terrains)} terrain maps')
        return 0
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName('HAMR Simulation Studio')
    window = Console(args)
    window.show()
    signal.signal(signal.SIGINT, lambda *_: window.close())
    return app.exec_()
