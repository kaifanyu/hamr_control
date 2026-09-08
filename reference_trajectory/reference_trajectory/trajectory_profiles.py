"""Load and sample validated independent-study trajectory profiles."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import re

import yaml


SCHEMA_VERSION = 1
SUPPORTED_KINDS = frozenset(('polyline', 'circle'))
SUPPORTED_DIRECTIONS = frozenset(('ccw', 'cw'))
ORIGIN_MODE_START_TRANSLATED_VICON = 'start_translated_vicon'
ORIGIN_MODE_VICON_ABSOLUTE = 'vicon_absolute'
SUPPORTED_ORIGIN_MODES = frozenset(
    (ORIGIN_MODE_START_TRANSLATED_VICON, ORIGIN_MODE_VICON_ABSOLUTE)
)
PROFILE_NAME_PATTERN = re.compile(r'^[a-z][a-z0-9_]{0,63}$')
MAX_PROFILE_COORDINATE_M = 100.0
MAX_PROFILE_RADIUS_M = 20.0
MAX_PROFILE_SPEED_M_S = 1.0
MAX_PROFILE_WAYPOINT_DWELL_S = 60.0
MAX_PROFILE_LAPS = 100
MIN_VISUALIZATION_SAMPLES = 17
MAX_VISUALIZATION_SAMPLES = 4097


class ProfileValidationError(ValueError):
    """Report an invalid or internally inconsistent trajectory profile."""


def _finite_number(name, value, *, minimum=None, maximum=None):
    if isinstance(value, bool):
        raise ProfileValidationError(f'{name} must be numeric, not boolean')
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ProfileValidationError(f'{name} must be numeric') from exc
    if not math.isfinite(result):
        raise ProfileValidationError(f'{name} must be finite')
    if minimum is not None and result < minimum:
        raise ProfileValidationError(f'{name} must be >= {minimum}')
    if maximum is not None and result > maximum:
        raise ProfileValidationError(f'{name} must be <= {maximum}')
    return result


def validated_speed_m_s(value):
    """Return a finite supported study speed."""
    speed = _finite_number(
        'speed_m_s', value, minimum=0.0, maximum=MAX_PROFILE_SPEED_M_S
    )
    if speed == 0.0:
        raise ProfileValidationError('speed_m_s must be greater than zero')
    return speed


def _integer(name, value, *, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProfileValidationError(f'{name} must be an integer')
    if value < minimum or value > maximum:
        raise ProfileValidationError(
            f'{name} must be in [{minimum}, {maximum}]'
        )
    return value


def _nonempty_string(name, value, *, maximum_length=500):
    if not isinstance(value, str) or not value.strip():
        raise ProfileValidationError(f'{name} must be a nonempty string')
    if len(value) > maximum_length:
        raise ProfileValidationError(
            f'{name} must be at most {maximum_length} characters'
        )
    return value


def _xy_pair(name, value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ProfileValidationError(f'{name} must contain exactly [x, y]')
    return (
        _finite_number(
            f'{name}[0]',
            value[0],
            minimum=-MAX_PROFILE_COORDINATE_M,
            maximum=MAX_PROFILE_COORDINATE_M,
        ),
        _finite_number(
            f'{name}[1]',
            value[1],
            minimum=-MAX_PROFILE_COORDINATE_M,
            maximum=MAX_PROFILE_COORDINATE_M,
        ),
    )


def validated_profile_name(value):
    """Return a profile name safe for package lookup and bag labels."""
    name = _nonempty_string('profile name', value, maximum_length=64)
    if PROFILE_NAME_PATTERN.fullmatch(name) is None:
        raise ProfileValidationError(
            'profile name must match [a-z][a-z0-9_]*'
        )
    return name


@dataclass(frozen=True)
class TrajectoryProfile:
    """A validated, immutable trajectory-profile description."""

    source_path: Path
    source_sha256: str
    source_yaml: str
    name: str
    kind: str
    description: str
    closed: bool
    default_speed_m_s: float
    fixed_yaw_rad: float
    origin_mode: str
    points_m: tuple[tuple[float, float], ...] = ()
    waypoint_dwells_s: tuple[float, ...] = ()
    center_m: tuple[float, float] | None = None
    radius_m: float | None = None
    start_angle_rad: float | None = None
    direction: str | None = None
    laps: int | None = None
    visualization_samples: int | None = None

    def bounds_offsets_m(self):
        """Return exact canonical ``min_x, max_x, min_y, max_y`` bounds."""
        if self.kind == 'circle':
            cx, cy = self.center_m
            radius = self.radius_m
            return cx - radius, cx + radius, cy - radius, cy + radius
        xs = [point[0] for point in self.points_m]
        ys = [point[1] for point in self.points_m]
        return min(xs), max(xs), min(ys), max(ys)


COMMON_KEYS = frozenset(
    (
        'schema_version',
        'name',
        'kind',
        'description',
        'closed',
        'default_speed_m_s',
        'fixed_yaw_rad',
    )
)
ORIGIN_MODE_KEY = frozenset(('origin_mode',))
POLYLINE_KEYS = COMMON_KEYS | frozenset(('points_m',))
POLYLINE_DWELL_KEYS = POLYLINE_KEYS | frozenset(('waypoint_dwells_s',))
CIRCLE_KEYS = COMMON_KEYS | frozenset(
    (
        'center_m',
        'radius_m',
        'start_angle_rad',
        'direction',
        'laps',
        'visualization_samples',
    )
)


def _require_exact_keys(document, expected):
    missing = sorted(expected - set(document))
    unexpected = sorted(set(document) - expected)
    if missing:
        raise ProfileValidationError(
            'profile is missing keys: ' + ', '.join(missing)
        )
    if unexpected:
        raise ProfileValidationError(
            'profile has unexpected keys: ' + ', '.join(unexpected)
        )


def _validated_common(document, source_path):
    schema = _integer(
        'schema_version',
        document.get('schema_version'),
        minimum=SCHEMA_VERSION,
        maximum=SCHEMA_VERSION,
    )
    if schema != SCHEMA_VERSION:
        raise ProfileValidationError(
            f'unsupported schema_version {schema}; expected {SCHEMA_VERSION}'
        )
    name = validated_profile_name(document.get('name'))
    if source_path.stem != name:
        raise ProfileValidationError(
            f'profile name {name!r} must match filename {source_path.stem!r}'
        )
    kind = _nonempty_string('kind', document.get('kind'), maximum_length=32)
    if kind not in SUPPORTED_KINDS:
        choices = ', '.join(sorted(SUPPORTED_KINDS))
        raise ProfileValidationError(f'kind must be one of: {choices}')
    description = _nonempty_string(
        'description', document.get('description'), maximum_length=500
    )
    closed = document.get('closed')
    if not isinstance(closed, bool):
        raise ProfileValidationError('closed must be a boolean')
    default_speed = validated_speed_m_s(document.get('default_speed_m_s'))
    fixed_yaw = _finite_number(
        'fixed_yaw_rad',
        document.get('fixed_yaw_rad'),
        minimum=-math.pi,
        maximum=math.pi,
    )
    origin_mode = _nonempty_string(
        'origin_mode',
        document.get(
            'origin_mode', ORIGIN_MODE_START_TRANSLATED_VICON
        ),
        maximum_length=32,
    )
    if origin_mode not in SUPPORTED_ORIGIN_MODES:
        choices = ', '.join(sorted(SUPPORTED_ORIGIN_MODES))
        raise ProfileValidationError(
            f'origin_mode must be one of: {choices}'
        )
    return (
        name,
        kind,
        description,
        closed,
        default_speed,
        fixed_yaw,
        origin_mode,
    )


def load_profile(path):
    """Load and strictly validate one YAML trajectory profile."""
    source_path = Path(path).expanduser().resolve()
    try:
        source_bytes = source_path.read_bytes()
    except OSError as exc:
        raise ProfileValidationError(
            f'cannot read trajectory profile {source_path}: {exc}'
        ) from exc
    try:
        source_yaml = source_bytes.decode('utf-8')
    except UnicodeDecodeError as exc:
        raise ProfileValidationError(
            f'trajectory profile {source_path} must be UTF-8'
        ) from exc
    try:
        document = yaml.safe_load(source_bytes)
    except yaml.YAMLError as exc:
        raise ProfileValidationError(
            f'invalid YAML in trajectory profile {source_path}: {exc}'
        ) from exc
    if not isinstance(document, dict):
        raise ProfileValidationError(
            'trajectory profile must be a YAML mapping'
        )

    common = _validated_common(document, source_path)
    (
        name,
        kind,
        description,
        closed,
        default_speed,
        fixed_yaw,
        origin_mode,
    ) = common
    digest = hashlib.sha256(source_bytes).hexdigest()

    if kind == 'polyline':
        polyline_keys = (
            POLYLINE_DWELL_KEYS
            if 'waypoint_dwells_s' in document
            else POLYLINE_KEYS
        )
        if 'origin_mode' in document:
            polyline_keys = polyline_keys | ORIGIN_MODE_KEY
        _require_exact_keys(document, polyline_keys)
        raw_points = document.get('points_m')
        if not isinstance(raw_points, list) or len(raw_points) < 2:
            raise ProfileValidationError(
                'points_m must contain at least two [x, y] points'
            )
        points = tuple(
            _xy_pair(f'points_m[{index}]', point)
            for index, point in enumerate(raw_points)
        )
        if math.hypot(points[0][0], points[0][1]) > 1e-9:
            raise ProfileValidationError(
                'canonical polyline must begin exactly at (0, 0)'
            )
        for index, (first, second) in enumerate(zip(points, points[1:])):
            if math.hypot(second[0] - first[0], second[1] - first[1]) <= 1e-9:
                raise ProfileValidationError(
                    f'points_m[{index}] and points_m[{index + 1}] '
                    'must be distinct'
                )
        endpoint_error = math.hypot(
            points[-1][0] - points[0][0],
            points[-1][1] - points[0][1],
        )
        if closed and endpoint_error > 1e-9:
            raise ProfileValidationError(
                'closed polyline must repeat its first point exactly at the '
                'end'
            )
        raw_dwells = document.get(
            'waypoint_dwells_s', [0.0] * len(points)
        )
        if not isinstance(raw_dwells, list) or len(raw_dwells) != len(points):
            raise ProfileValidationError(
                'waypoint_dwells_s must contain one value per point'
            )
        waypoint_dwells = tuple(
            _finite_number(
                f'waypoint_dwells_s[{index}]',
                value,
                minimum=0.0,
                maximum=MAX_PROFILE_WAYPOINT_DWELL_S,
            )
            for index, value in enumerate(raw_dwells)
        )
        if waypoint_dwells[0] != 0.0 or waypoint_dwells[-1] != 0.0:
            raise ProfileValidationError(
                'first and final waypoint dwell must be exactly zero; '
                'startup/final holds own endpoint timing'
            )
        return TrajectoryProfile(
            source_path=source_path,
            source_sha256=digest,
            source_yaml=source_yaml,
            name=name,
            kind=kind,
            description=description,
            closed=closed,
            default_speed_m_s=default_speed,
            fixed_yaw_rad=fixed_yaw,
            origin_mode=origin_mode,
            points_m=points,
            waypoint_dwells_s=waypoint_dwells,
        )

    circle_keys = CIRCLE_KEYS
    if 'origin_mode' in document:
        circle_keys = circle_keys | ORIGIN_MODE_KEY
    _require_exact_keys(document, circle_keys)
    if not closed:
        raise ProfileValidationError('circle profiles must be closed')
    center = _xy_pair('center_m', document.get('center_m'))
    radius = _finite_number(
        'radius_m',
        document.get('radius_m'),
        minimum=0.0,
        maximum=MAX_PROFILE_RADIUS_M,
    )
    if radius == 0.0:
        raise ProfileValidationError('radius_m must be greater than zero')
    start_angle = _finite_number(
        'start_angle_rad', document.get('start_angle_rad')
    )
    direction = _nonempty_string(
        'direction', document.get('direction'), maximum_length=3
    )
    if direction not in SUPPORTED_DIRECTIONS:
        choices = ', '.join(sorted(SUPPORTED_DIRECTIONS))
        raise ProfileValidationError(f'direction must be one of: {choices}')
    laps = _integer(
        'laps', document.get('laps'), minimum=1, maximum=MAX_PROFILE_LAPS
    )
    samples = _integer(
        'visualization_samples',
        document.get('visualization_samples'),
        minimum=MIN_VISUALIZATION_SAMPLES,
        maximum=MAX_VISUALIZATION_SAMPLES,
    )
    start_x = center[0] + radius * math.cos(start_angle)
    start_y = center[1] + radius * math.sin(start_angle)
    if math.hypot(start_x, start_y) > 1e-9:
        raise ProfileValidationError(
            'canonical circle must begin exactly at (0, 0)'
        )
    return TrajectoryProfile(
        source_path=source_path,
        source_sha256=digest,
        source_yaml=source_yaml,
        name=name,
        kind=kind,
        description=description,
        closed=closed,
        default_speed_m_s=default_speed,
        fixed_yaw_rad=fixed_yaw,
        origin_mode=origin_mode,
        center_m=center,
        radius_m=radius,
        start_angle_rad=start_angle,
        direction=direction,
        laps=laps,
        visualization_samples=samples,
    )


def profile_path(profile_directory, profile_name):
    """Return the safe YAML path for a validated catalog profile name."""
    name = validated_profile_name(profile_name)
    return Path(profile_directory).expanduser().resolve() / f'{name}.yaml'


class PolylineTraj:
    """Constant-speed piecewise-linear trajectory with fixed reference yaw."""

    def __init__(
        self,
        points_m,
        speed_m_s,
        fixed_yaw_rad=0.0,
        waypoint_dwells_s=None,
    ):
        """Validate points and precompute constant segment velocities."""
        self.speed_m_s = validated_speed_m_s(speed_m_s)
        self.fixed_yaw_rad = _finite_number(
            'fixed_yaw_rad', fixed_yaw_rad, minimum=-math.pi, maximum=math.pi
        )
        if len(points_m) < 2:
            raise ProfileValidationError('polyline needs at least two points')
        xy_points = tuple(
            _xy_pair(f'points_m[{index}]', point)
            for index, point in enumerate(points_m)
        )
        self.points = tuple(
            (point[0], point[1], self.fixed_yaw_rad) for point in xy_points
        )
        if waypoint_dwells_s is None:
            dwells = (0.0,) * len(xy_points)
        else:
            if len(waypoint_dwells_s) != len(xy_points):
                raise ProfileValidationError(
                    'waypoint_dwells_s must contain one value per point'
                )
            dwells = tuple(
                _finite_number(
                    f'waypoint_dwells_s[{index}]',
                    value,
                    minimum=0.0,
                    maximum=MAX_PROFILE_WAYPOINT_DWELL_S,
                )
                for index, value in enumerate(waypoint_dwells_s)
            )
        if dwells[0] != 0.0 or dwells[-1] != 0.0:
            raise ProfileValidationError(
                'first and final waypoint dwell must be exactly zero'
            )
        self.waypoint_dwells_s = dwells
        self.motion_phase_count = len(xy_points) - 1
        self.total_dwell_s = sum(dwells)
        self._phases = []
        self._phase_end_times = []
        elapsed = 0.0
        for index, (first, second) in enumerate(
            zip(xy_points, xy_points[1:])
        ):
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            length = math.hypot(dx, dy)
            if length <= 1e-9:
                raise ProfileValidationError(
                    f'polyline segment {index} has zero length'
                )
            duration = length / self.speed_m_s
            vx = dx / duration
            vy = dy / duration
            start_time = elapsed
            elapsed += duration
            self._phases.append(
                ('motion', first[0], first[1], start_time, duration, vx, vy)
            )
            self._phase_end_times.append(elapsed)
            dwell = dwells[index + 1]
            if dwell > 0.0:
                self._phases.append(
                    ('dwell', second[0], second[1], elapsed, dwell, 0.0, 0.0)
                )
                elapsed += dwell
                self._phase_end_times.append(elapsed)
        self.total_time = elapsed

    def update(self, time_s):
        """Return ``x, y, yaw, x_dot, y_dot, yaw_dot`` at trajectory time."""
        t = _finite_number('trajectory time', time_s, minimum=0.0)
        if t >= self.total_time:
            x, y, yaw = self.points[-1]
            return x, y, yaw, 0.0, 0.0, 0.0
        phase_index = bisect_right(self._phase_end_times, t)
        kind, x0, y0, start_time, duration, vx, vy = self._phases[
            phase_index
        ]
        if kind == 'dwell':
            return x0, y0, self.fixed_yaw_rad, 0.0, 0.0, 0.0
        local_time = min(duration, max(0.0, t - start_time))
        return (
            x0 + vx * local_time,
            y0 + vy * local_time,
            self.fixed_yaw_rad,
            vx,
            vy,
            0.0,
        )


class CircleTraj:
    """Analytic constant-speed circular trajectory with exact closure."""

    def __init__(
        self,
        *,
        center_m,
        radius_m,
        start_angle_rad,
        direction,
        laps,
        speed_m_s,
        fixed_yaw_rad=0.0,
        visualization_samples=129,
    ):
        """Validate circle geometry and precompute its visualization path."""
        self.center_m = _xy_pair('center_m', center_m)
        self.radius_m = _finite_number(
            'radius_m',
            radius_m,
            minimum=0.0,
            maximum=MAX_PROFILE_RADIUS_M,
        )
        if self.radius_m == 0.0:
            raise ProfileValidationError('radius_m must be greater than zero')
        self.start_angle_rad = _finite_number(
            'start_angle_rad', start_angle_rad
        )
        if direction not in SUPPORTED_DIRECTIONS:
            choices = ', '.join(sorted(SUPPORTED_DIRECTIONS))
            raise ProfileValidationError(
                f'direction must be one of: {choices}'
            )
        self.direction = direction
        self.laps = _integer(
            'laps', laps, minimum=1, maximum=MAX_PROFILE_LAPS
        )
        self.speed_m_s = validated_speed_m_s(speed_m_s)
        self.fixed_yaw_rad = _finite_number(
            'fixed_yaw_rad', fixed_yaw_rad, minimum=-math.pi, maximum=math.pi
        )
        sample_count = _integer(
            'visualization_samples',
            visualization_samples,
            minimum=MIN_VISUALIZATION_SAMPLES,
            maximum=MAX_VISUALIZATION_SAMPLES,
        )
        self.direction_sign = 1.0 if direction == 'ccw' else -1.0
        self.waypoint_dwells_s = ()
        self.motion_phase_count = 1
        self.total_dwell_s = 0.0
        self.angular_speed_rad_s = (
            self.direction_sign * self.speed_m_s / self.radius_m
        )
        self.total_time = (
            2.0 * math.pi * self.radius_m * self.laps / self.speed_m_s
        )
        self._start_point = self._position(self.start_angle_rad)
        if math.hypot(*self._start_point) <= 1e-9:
            self._start_point = (0.0, 0.0)
        sampled_points = []
        for index in range(sample_count):
            fraction = index / (sample_count - 1)
            angle = self.start_angle_rad + (
                self.direction_sign * 2.0 * math.pi * self.laps * fraction
            )
            x, y = self._position(angle)
            sampled_points.append((x, y, self.fixed_yaw_rad))
        # Exact identity matters for final-hold and closed-path tests; avoid a
        # trigonometric round-off difference after one or more full laps.
        sampled_points[0] = (
            self._start_point[0],
            self._start_point[1],
            self.fixed_yaw_rad,
        )
        sampled_points[-1] = sampled_points[0]
        self.points = tuple(sampled_points)

    def _position(self, angle):
        cx, cy = self.center_m
        return (
            cx + self.radius_m * math.cos(angle),
            cy + self.radius_m * math.sin(angle),
        )

    def update(self, time_s):
        """Return the exact analytic circular reference at trajectory time."""
        t = _finite_number('trajectory time', time_s, minimum=0.0)
        if t >= self.total_time:
            return (
                self._start_point[0],
                self._start_point[1],
                self.fixed_yaw_rad,
                0.0,
                0.0,
                0.0,
            )
        angle = self.start_angle_rad + self.angular_speed_rad_s * t
        x, y = self._position(angle)
        x_dot = -self.radius_m * math.sin(angle) * self.angular_speed_rad_s
        y_dot = self.radius_m * math.cos(angle) * self.angular_speed_rad_s
        if t == 0.0:
            x, y = self._start_point
        x_dot = 0.0 if abs(x_dot) <= 1e-15 else x_dot
        y_dot = 0.0 if abs(y_dot) <= 1e-15 else y_dot
        return x, y, self.fixed_yaw_rad, x_dot, y_dot, 0.0


class TranslatedTrajectory:
    """Translate a canonical trajectory without rotating its fixed axes."""

    def __init__(self, trajectory, origin_x_m, origin_y_m):
        """Anchor the canonical first point at the requested world XY."""
        if not getattr(trajectory, 'points', None):
            raise ProfileValidationError('trajectory has no points to anchor')
        origin_x = _finite_number('origin_x_m', origin_x_m)
        origin_y = _finite_number('origin_y_m', origin_y_m)
        first_x, first_y, _ = trajectory.points[0]
        self.dx = origin_x - first_x
        self.dy = origin_y - first_y
        self.total_time = trajectory.total_time
        self._trajectory = trajectory
        self.points = tuple(
            (x + self.dx, y + self.dy, yaw)
            for x, y, yaw in trajectory.points
        )

    def update(self, time_s):
        """Return the translated position with unchanged velocities and yaw."""
        x, y, yaw, x_dot, y_dot, yaw_dot = self._trajectory.update(time_s)
        return x + self.dx, y + self.dy, yaw, x_dot, y_dot, yaw_dot


def build_trajectory(profile, speed_m_s=None):
    """Build the trajectory implementation selected by a loaded profile."""
    speed = validated_speed_m_s(
        profile.default_speed_m_s if speed_m_s is None else speed_m_s
    )
    if profile.kind == 'polyline':
        return PolylineTraj(
            profile.points_m,
            speed,
            fixed_yaw_rad=profile.fixed_yaw_rad,
            waypoint_dwells_s=profile.waypoint_dwells_s,
        )
    if profile.kind == 'circle':
        return CircleTraj(
            center_m=profile.center_m,
            radius_m=profile.radius_m,
            start_angle_rad=profile.start_angle_rad,
            direction=profile.direction,
            laps=profile.laps,
            speed_m_s=speed,
            fixed_yaw_rad=profile.fixed_yaw_rad,
            visualization_samples=profile.visualization_samples,
        )
    raise ProfileValidationError(f'unsupported profile kind {profile.kind!r}')
