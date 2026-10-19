"""Qt views; map coordinates are the world X/Y used by Gazebo."""
import math
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from .theme import ACCENT, BACKGROUND, BORDER, MUTED, RAISED, SAND, SERIES, SURFACE, TEXT


class TerrainMap(QtWidgets.QWidget):
    changed = QtCore.pyqtSignal(list)
    placement_requested = QtCore.pyqtSignal(float, float, float)
    placement_cancelled = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(280, 260)
        self.setMouseTracking(True)
        self.bounds = (-3., 3., -4., 4.)
        self.points, self.trail, self.position = [], [], None
        self.image = None
        self.editable = True
        self.drag_index = None
        self.cursor_world = None
        self.placement_mode = False
        self.placement_anchor = None
        self.placement_yaw = 0.

    def set_terrain(self, terrain):
        self.set_placement_mode(False)
        self.bounds = terrain.bounds
        self.points, self.trail, self.position = [], [], None
        self.drag_index = self.cursor_world = None
        self.image = None
        if terrain.height_grid and terrain.height_grid.is_file():
            with np.load(terrain.height_grid) as grid:
                z = np.asarray(grid['z'], dtype=float)
            z = np.nan_to_num(z)
            h = (z - z.min()) / max(float(np.ptp(z)), 0.02)
            gy, gx = np.gradient(z)
            light = np.clip(0.8 - 6 * gx + 4 * gy, .3, 1.15)
            low, high = np.array([43, 49, 36]), np.array([108, 117, 87])
            rgb = np.clip((low + h[..., None] * (high-low)) * light[..., None], 0, 255).astype(np.uint8)
            rgb = np.ascontiguousarray(rgb[::-1])
            self.image = QtGui.QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
        self.update()
        self.changed.emit(self.points)

    def plot_rect(self):
        x0, x1, y0, y1 = self.bounds
        scale = min(max(1, self.width()-56)/(x1-x0), max(1, self.height()-50)/(y1-y0))
        width, height = (x1-x0)*scale, (y1-y0)*scale
        return QtCore.QRectF((self.width()-width)/2, (self.height()-height)/2, width, height)

    def world_to_pixel(self, xy):
        r = self.plot_rect()
        x0, x1, y0, y1 = self.bounds
        return QtCore.QPointF(r.left()+(xy[0]-x0)/(x1-x0)*r.width(), r.bottom()-(xy[1]-y0)/(y1-y0)*r.height())

    def pixel_to_world(self, p):
        r = self.plot_rect()
        if not r.contains(p):
            return None
        x0, x1, y0, y1 = self.bounds
        return [x0+(p.x()-r.left())/r.width()*(x1-x0), y0+(r.bottom()-p.y())/r.height()*(y1-y0)]

    def set_points(self, points, emit=True):
        self.points = [list(map(float, p[:2])) for p in points]
        self.drag_index = None
        self.update()
        if emit:
            self.changed.emit(self.points)

    def set_placement_mode(self, enabled, yaw=None):
        self.placement_mode = bool(enabled)
        self.placement_anchor = None
        self.drag_index = None
        if yaw is not None:
            self.placement_yaw = float(yaw)
        self.setCursor(QtCore.Qt.CrossCursor if enabled else QtCore.Qt.ArrowCursor)
        self.update()

    def mousePressEvent(self, event):
        if not self.editable:
            return
        if self.placement_mode and event.button() == QtCore.Qt.RightButton:
            self.set_placement_mode(False)
            self.placement_cancelled.emit()
            return
        p = self.pixel_to_world(event.localPos())
        if p is None:
            return
        if self.placement_mode:
            if event.button() == QtCore.Qt.LeftButton:
                self.placement_anchor = p
                self.cursor_world = p
                self.update()
            return
        near = next((i for i, xy in enumerate(self.points) if (self.world_to_pixel(xy)-event.localPos()).manhattanLength() < 16), None)
        if event.button() == QtCore.Qt.RightButton:
            if near is not None:
                del self.points[near]
                self.changed.emit(self.points)
        elif event.button() == QtCore.Qt.LeftButton:
            if near is None:
                self.points.append(p)
                near = len(self.points)-1
            self.drag_index = near
            self.changed.emit(self.points)
        self.update()

    def mouseMoveEvent(self, event):
        self.cursor_world = self.pixel_to_world(event.localPos())
        if self.placement_mode:
            if not self.editable:
                self.placement_anchor = None
            elif self.placement_anchor is not None and self.cursor_world is not None:
                if math.dist(self.placement_anchor, self.cursor_world) > .03:
                    self.placement_yaw = math.atan2(self.cursor_world[1]-self.placement_anchor[1],
                                                   self.cursor_world[0]-self.placement_anchor[0])
            self.update()
            return
        if not self.editable or (self.drag_index is not None and self.drag_index >= len(self.points)):
            self.drag_index = None
        if self.drag_index is not None and self.cursor_world is not None:
            self.points[self.drag_index] = self.cursor_world
            self.changed.emit(self.points)
        self.update()

    def mouseReleaseEvent(self, event):
        if self.placement_mode and event.button() == QtCore.Qt.LeftButton:
            anchor = self.placement_anchor
            self.placement_anchor = None
            if self.editable and anchor is not None and self.pixel_to_world(event.localPos()) is not None:
                self.placement_requested.emit(*anchor, self.placement_yaw)
            self.update()
            return
        self.drag_index = None

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor(SURFACE))
        r = self.plot_rect()
        if self.image:
            painter.drawImage(r, self.image)
        else:
            painter.fillRect(r, QtGui.QColor(RAISED))
        x0, x1, y0, y1 = self.bounds
        painter.setPen(QtGui.QPen(QtGui.QColor(BORDER), .6))
        step = max(1, math.ceil(max(x1-x0, y1-y0)/12))
        for x in range(math.ceil(x0), math.floor(x1)+1, step):
            a, b = self.world_to_pixel((x, y0)), self.world_to_pixel((x, y1))
            painter.drawLine(a, b)
            painter.setPen(QtGui.QColor(MUTED))
            painter.drawText(QtCore.QRectF(a.x()-18, r.bottom()+3, 36, 18), QtCore.Qt.AlignCenter, f'{x}')
            painter.setPen(QtGui.QPen(QtGui.QColor(BORDER), .6))
        for y in range(math.ceil(y0), math.floor(y1)+1, step):
            a, b = self.world_to_pixel((x0, y)), self.world_to_pixel((x1, y))
            painter.drawLine(a, b)
            painter.setPen(QtGui.QColor(MUTED))
            painter.drawText(QtCore.QRectF(r.left()-31, a.y()-9, 27, 18), QtCore.Qt.AlignRight, f'{y}')
            painter.setPen(QtGui.QPen(QtGui.QColor(BORDER), .6))
        painter.save()
        painter.setClipRect(r)
        for points, color, width in ((self.trail, SAND, 2), (self.points, ACCENT, 2)):
            if len(points) > 1:
                painter.setPen(QtGui.QPen(QtGui.QColor(color), width))
                painter.drawPolyline(QtGui.QPolygonF([self.world_to_pixel(p) for p in points]))
        for i, xy in enumerate(self.points):
            p = self.world_to_pixel(xy)
            painter.setBrush(QtGui.QColor(ACCENT))
            painter.setPen(QtGui.QPen(QtGui.QColor(BACKGROUND), 2))
            painter.drawEllipse(p, 9, 9)
            painter.setPen(QtGui.QColor(BACKGROUND))
            painter.drawText(QtCore.QRectF(p.x()-9, p.y()-9, 18, 18), QtCore.Qt.AlignCenter, str(i+1))
        if self.position:
            x, y, yaw = self.position
            painter.save()
            p = self.world_to_pixel((x, y))
            painter.translate(p)
            painter.rotate(-math.degrees(yaw))
            painter.setBrush(QtGui.QColor(TEXT))
            painter.setPen(QtGui.QPen(QtGui.QColor(BACKGROUND), 2))
            painter.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(12, 0), QtCore.QPointF(-9, -7), QtCore.QPointF(-6, 0), QtCore.QPointF(-9, 7)]))
            painter.restore()
        if self.placement_mode:
            destination = self.placement_anchor or self.cursor_world
            if destination is not None:
                painter.save()
                p = self.world_to_pixel(destination)
                painter.translate(p)
                painter.setBrush(QtCore.Qt.NoBrush)
                painter.setPen(QtGui.QPen(QtGui.QColor(ACCENT), 1.5, QtCore.Qt.DashLine))
                painter.drawEllipse(QtCore.QPointF(), 17, 17)
                painter.rotate(-math.degrees(self.placement_yaw))
                painter.setBrush(QtGui.QColor(ACCENT))
                painter.setPen(QtGui.QPen(QtGui.QColor(BACKGROUND), 1.5))
                painter.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(13, 0), QtCore.QPointF(-8, -6),
                                                     QtCore.QPointF(-5, 0), QtCore.QPointF(-8, 6)]))
                painter.restore()
        painter.restore()
        painter.setPen(QtGui.QColor(MUTED))
        label = 'World X / Y · meters'
        if self.cursor_world:
            label = f'X {self.cursor_world[0]:+.2f}  Y {self.cursor_world[1]:+.2f} m'
        painter.drawText(QtCore.QRectF(8, 4, self.width()-16, 20), QtCore.Qt.AlignRight, label)


class HistoryPlot(QtWidgets.QWidget):
    """A short rolling history in simulation time, with honest independent axes."""
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(130)
        self.samples = []
        self.last_time = None
        self.configure('ball')

    def configure(self, caster):
        from .telemetry import JOINT_MAPPING
        self.joint_names = tuple(JOINT_MAPPING[caster].values())
        self.legend = 'H1 / H2 / CENTER' if caster == 'ball' else 'Left / Right / Turret'
        self.samples.clear()
        self.last_time = None
        self.update()

    def append(self, state):
        t = state.get('sim_time')
        if t is None or t == self.last_time:
            return
        if self.last_time is not None and t < self.last_time:
            self.samples.clear()
        self.last_time = t
        c = state.get('joints') or state.get('casters', {})
        left, right, center = [c.get(name, {}).get('position') for name in self.joint_names]
        self.samples.append((t, state.get('linear_speed'), left, right, center))
        self.samples = [s for s in self.samples[-1200:] if s[0] >= t-30]
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor(SURFACE))
        p.setPen(QtGui.QColor(MUTED))
        p.drawText(12, 21, '30 s history · Speed / ' + self.legend)
        r = QtCore.QRectF(45, 35, max(1, self.width()-96), self.height()-58)
        p.setPen(QtGui.QPen(QtGui.QColor(BORDER), 1))
        for ratio in (0, .5, 1):
            y = r.top()+ratio*r.height()
            p.drawLine(QtCore.QPointF(r.left(), y), QtCore.QPointF(r.right(), y))
        if not self.samples:
            p.setPen(QtGui.QColor(MUTED))
            p.drawText(r, QtCore.Qt.AlignCenter, 'Awaiting telemetry')
            return
        speed_scale = max(.25, max(s[1] or 0 for s in self.samples)*1.15)
        t_end = self.samples[-1][0]
        for col, color in enumerate(SERIES, start=1):
            path = QtGui.QPainterPath()
            prior = None
            for s in self.samples:
                if s[col] is None:
                    prior = None
                    continue
                value = s[col]/speed_scale if col == 1 else (math.atan2(math.sin(s[col]), math.cos(s[col]))+math.pi)/(2*math.pi)
                point = QtCore.QPointF(r.left()+(s[0]-t_end+30)/30*r.width(), r.bottom()-value*r.height())
                if prior is None or (col > 1 and abs(value-prior) > .5):
                    path.moveTo(point)
                else:
                    path.lineTo(point)
                prior = value
            p.setPen(QtGui.QPen(QtGui.QColor(color), 1.7))
            p.drawPath(path)
        p.setPen(QtGui.QColor(MUTED))
        p.drawText(4, int(r.top()+8), f'{speed_scale:.2f}')
        p.drawText(13, int(r.bottom()), '0.0')
        p.drawText(int(r.right()+6), int(r.top()+8), '+180°')
        p.drawText(int(r.right()+6), int(r.bottom()), '−180°')
        p.drawText(int(r.left()), self.height()-5, f'{max(0,t_end-30):.1f} s')
        p.drawText(int(r.right()-45), self.height()-5, f'{t_end:.1f} s')


class CameraView(QtWidgets.QLabel):
    def __init__(self):
        super().__init__('Waiting for the caster camera…')
        self.setMinimumSize(280, 180)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet(f'background: {BACKGROUND}; color: {MUTED}; border-radius: 6px;')
        self.source = None

    def clear(self):
        """Forget the previous session's image, including its resize cache."""
        self.source = None
        super().clear()

    def set_frame(self, frame):
        formats = {'rgb8': QtGui.QImage.Format_RGB888, 'bgr8': QtGui.QImage.Format_RGB888,
                   'rgba8': QtGui.QImage.Format_RGBA8888, 'bgra8': QtGui.QImage.Format_ARGB32,
                   'mono8': QtGui.QImage.Format_Grayscale8}
        encoding = frame['encoding'].lower()
        if encoding not in formats:
            self.setText(f'Unsupported image encoding: {encoding}')
            return
        image = QtGui.QImage(frame['data'], frame['width'], frame['height'], frame['step'], formats[encoding]).copy()
        if encoding == 'bgr8':
            image = image.rgbSwapped()
        self.source = QtGui.QPixmap.fromImage(image)
        self._resize()

    def _resize(self):
        if self.source:
            self.setPixmap(self.source.scaled(self.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    def resizeEvent(self, event):
        self._resize()
