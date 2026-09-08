"""Offline tests for independent-study profiles and reference metadata."""

import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from reference_trajectory.study_trajectory import finite_odom_sample
from reference_trajectory.study_trajectory import finite_odom_xy
from reference_trajectory.study_trajectory import finite_quaternion_yaw
from reference_trajectory.study_trajectory import metadata_document
from reference_trajectory.study_trajectory import metadata_json
from reference_trajectory.study_trajectory import METADATA_TOPIC
from reference_trajectory.study_trajectory import NOMINAL_START_BASE_YAW_RAD
from reference_trajectory.study_trajectory import ORIGIN_MODE
from reference_trajectory.study_trajectory import place_trajectory
from reference_trajectory.study_trajectory import REFERENCE_TIMEOUT_S
from reference_trajectory.study_trajectory import (
    START_BASE_YAW_TOLERANCE_RAD,
)
from reference_trajectory.study_trajectory import (
    STUDY_STACK_CONTRACT_VERSION,
)
from reference_trajectory.study_trajectory import validated_boolean
from reference_trajectory.study_trajectory import validated_contract_value
from reference_trajectory.study_trajectory import validated_positive_float
from reference_trajectory.study_trajectory import validated_repetition
from reference_trajectory.study_trajectory import validated_sha256
from reference_trajectory.study_trajectory import validated_study_text
from reference_trajectory.study_trajectory import XY_VELOCITY_SOURCE
from reference_trajectory.trajectory_profiles import build_trajectory
from reference_trajectory.trajectory_profiles import CircleTraj
from reference_trajectory.trajectory_profiles import load_profile
from reference_trajectory.trajectory_profiles import (
    ORIGIN_MODE_START_TRANSLATED_VICON,
)
from reference_trajectory.trajectory_profiles import (
    ORIGIN_MODE_VICON_ABSOLUTE,
)
from reference_trajectory.trajectory_profiles import profile_path
from reference_trajectory.trajectory_profiles import ProfileValidationError
from reference_trajectory.trajectory_profiles import TranslatedTrajectory
from reference_trajectory.waypoint_traj_simple import PHASE_FINAL_HOLD
from reference_trajectory.waypoint_traj_simple import PHASE_MOTION
from reference_trajectory.waypoint_traj_simple import PHASE_STARTUP_HOLD
from reference_trajectory.waypoint_traj_simple import scheduled_reference
from reference_trajectory.waypoint_traj_simple import TrajectoryPlayback

import yaml


PROFILE_DIRECTORY = Path(__file__).resolve().parents[1] / 'config/trajectories'
PROFILE_NAMES = (
    'triangle',
    'triangle_120',
    'circle_ccw',
    'circle_cw',
    'straight_forward',
    'straight_backward',
    'lateral_left',
    'lateral_right',
    'forward_reverse',
    'waypoint_traj_simple',
)


def catalog_profile(name):
    """Load one source-tree catalog profile."""
    return load_profile(PROFILE_DIRECTORY / f'{name}.yaml')


@pytest.mark.parametrize(
    ('name', 'kind', 'closed', 'default_speed_m_s', 'origin_mode'),
    (
        (
            'triangle', 'polyline', True, 0.15,
            ORIGIN_MODE_START_TRANSLATED_VICON,
        ),
        (
            'triangle_120', 'polyline', True, 0.15,
            ORIGIN_MODE_START_TRANSLATED_VICON,
        ),
        (
            'circle_ccw', 'circle', True, 0.15,
            ORIGIN_MODE_START_TRANSLATED_VICON,
        ),
        (
            'circle_cw', 'circle', True, 0.15,
            ORIGIN_MODE_START_TRANSLATED_VICON,
        ),
        (
            'straight_forward', 'polyline', False, 0.15,
            ORIGIN_MODE_START_TRANSLATED_VICON,
        ),
        (
            'straight_backward', 'polyline', False, 0.15,
            ORIGIN_MODE_START_TRANSLATED_VICON,
        ),
        (
            'lateral_left', 'polyline', False, 0.15,
            ORIGIN_MODE_START_TRANSLATED_VICON,
        ),
        (
            'lateral_right', 'polyline', False, 0.15,
            ORIGIN_MODE_START_TRANSLATED_VICON,
        ),
        (
            'forward_reverse', 'polyline', True, 0.15,
            ORIGIN_MODE_START_TRANSLATED_VICON,
        ),
        (
            'waypoint_traj_simple', 'polyline', True, 0.20,
            ORIGIN_MODE_VICON_ABSOLUTE,
        ),
    ),
)
def test_catalog_profiles_are_auditable_and_start_at_origin(
    name, kind, closed, default_speed_m_s, origin_mode
):
    """Require every installed catalog profile to honor common contracts."""
    profile = catalog_profile(name)
    trajectory = build_trajectory(profile)

    assert profile.name == name
    assert profile.kind == kind
    assert profile.closed is closed
    assert profile.default_speed_m_s == pytest.approx(default_speed_m_s)
    assert profile.fixed_yaw_rad == pytest.approx(0.0)
    assert profile.origin_mode == origin_mode
    assert len(profile.source_sha256) == 64
    assert profile.source_yaml == profile.source_path.read_text(
        encoding='utf-8'
    )
    assert trajectory.points[0][:2] == pytest.approx((0.0, 0.0), abs=1e-12)
    if closed:
        assert trajectory.points[-1] == trajectory.points[0]
    else:
        assert trajectory.points[-1] != trajectory.points[0]


def test_profile_names_cover_the_installed_yaml_catalog():
    """Keep tests synchronized with every installed study profile."""
    installed_names = tuple(
        sorted(path.stem for path in PROFILE_DIRECTORY.glob('*.yaml'))
    )

    assert installed_names == tuple(sorted(PROFILE_NAMES))


def test_waypoint_traj_simple_profile_preserves_legacy_route_exactly():
    """Expose the old long route in unchanged absolute Vicon coordinates."""
    profile = catalog_profile('waypoint_traj_simple')
    trajectory = build_trajectory(profile)

    assert profile.points_m == (
        (0.0, 0.0),
        (0.0, 2.0),
        (-2.0, 2.0),
        (-2.0, 4.5),
        (0.0, 4.5),
        (0.0, 2.0),
        (0.0, 0.0),
    )
    assert profile.bounds_offsets_m() == pytest.approx(
        (-2.0, 0.0, 0.0, 4.5)
    )
    assert trajectory.motion_phase_count == 6
    assert trajectory.total_dwell_s == 0.0
    assert trajectory.total_time == pytest.approx(65.0)
    placed, origin_x, origin_y = place_trajectory(
        profile, trajectory, 3.25, -1.75
    )
    assert (origin_x, origin_y) == pytest.approx((0.0, 0.0))
    assert placed.points == trajectory.points
    assert placed.points[3] == pytest.approx((-2.0, 4.5, 0.0))


def test_normal_profile_still_translates_to_captured_vicon_start():
    """Keep the absolute policy isolated to the explicit legacy profile."""
    profile = catalog_profile('triangle')
    trajectory = build_trajectory(profile)
    placed, origin_x, origin_y = place_trajectory(
        profile, trajectory, 3.25, -1.75
    )

    assert (origin_x, origin_y) == pytest.approx((3.25, -1.75))
    assert placed.points[0] == pytest.approx((3.25, -1.75, 0.0))
    assert placed.points[-1] == placed.points[0]


@pytest.mark.parametrize(
    ('name', 'endpoint', 'velocity', 'bounds'),
    (
        (
            'straight_forward',
            (0.0, 1.0),
            (0.0, 0.15),
            (0.0, 0.0, 0.0, 1.0),
        ),
        (
            'straight_backward',
            (0.0, -1.0),
            (0.0, -0.15),
            (0.0, 0.0, -1.0, 0.0),
        ),
        (
            'lateral_left',
            (-1.0, 0.0),
            (-0.15, 0.0),
            (-1.0, 0.0, 0.0, 0.0),
        ),
        (
            'lateral_right',
            (1.0, 0.0),
            (0.15, 0.0),
            (0.0, 1.0, 0.0, 0.0),
        ),
    ),
)
def test_one_metre_directional_baselines_are_exact(
    name, endpoint, velocity, bounds
):
    """Keep the four directional baselines paired and unambiguous."""
    profile = catalog_profile(name)
    trajectory = build_trajectory(profile)

    assert trajectory.total_time == pytest.approx(1.0 / 0.15)
    assert trajectory.total_dwell_s == 0.0
    assert trajectory.motion_phase_count == 1
    assert trajectory.update(0.0) == pytest.approx(
        (0.0, 0.0, 0.0, velocity[0], velocity[1], 0.0)
    )
    midpoint = trajectory.update(0.5 * trajectory.total_time)
    assert midpoint[:2] == pytest.approx(
        (0.5 * endpoint[0], 0.5 * endpoint[1])
    )
    assert midpoint[3:5] == pytest.approx(velocity)
    assert trajectory.update(trajectory.total_time) == pytest.approx(
        (endpoint[0], endpoint[1], 0.0, 0.0, 0.0, 0.0)
    )
    assert profile.bounds_offsets_m() == pytest.approx(bounds)


def test_forward_reverse_stops_one_second_before_exact_return():
    """Avoid an aggressive +speed-to--speed command at the reversal point."""
    profile = catalog_profile('forward_reverse')
    trajectory = build_trajectory(profile)
    leg_time = 1.0 / 0.15

    assert profile.waypoint_dwells_s == (0.0, 1.0, 0.0)
    assert trajectory.motion_phase_count == 2
    assert trajectory.total_dwell_s == pytest.approx(1.0)
    assert trajectory.total_time == pytest.approx(2.0 * leg_time + 1.0)
    assert trajectory.update(0.0) == pytest.approx(
        (0.0, 0.0, 0.0, 0.0, 0.15, 0.0)
    )
    assert trajectory.update(leg_time) == pytest.approx(
        (0.0, 1.0, 0.0, 0.0, 0.0, 0.0)
    )
    assert trajectory.update(leg_time + 0.999) == pytest.approx(
        (0.0, 1.0, 0.0, 0.0, 0.0, 0.0)
    )
    assert trajectory.update(leg_time + 1.0) == pytest.approx(
        (0.0, 1.0, 0.0, 0.0, -0.15, 0.0)
    )
    assert trajectory.update(trajectory.total_time) == pytest.approx(
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    )
    assert profile.bounds_offsets_m() == pytest.approx(
        (0.0, 0.0, 0.0, 1.0)
    )


def test_triangle_is_equilateral_with_exact_constant_speed_segments():
    """Keep the triangle geometry and speed schedule exact at its corners."""
    profile = catalog_profile('triangle')
    trajectory = build_trajectory(profile)
    side_lengths = [
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(profile.points_m, profile.points_m[1:])
    ]

    assert side_lengths == pytest.approx((2.0, 2.0, 2.0))
    assert trajectory.total_time == pytest.approx(40.0)
    assert trajectory.update(0.0) == pytest.approx(
        (0.0, 0.0, 0.0, 0.0, 0.15, 0.0)
    )

    first_corner_s = 2.0 / 0.15
    at_corner = trajectory.update(first_corner_s)
    assert at_corner[:3] == pytest.approx((0.0, 2.0, 0.0))
    assert math.hypot(at_corner[3], at_corner[4]) == pytest.approx(0.15)
    assert at_corner[3] < 0.0
    assert at_corner[4] < 0.0
    assert trajectory.update(trajectory.total_time) == pytest.approx(
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    )


def test_triangle_120_exposes_all_three_vertices_with_exact_geometry():
    """Preserve the 30-120-30 diagnostic and its three direction changes."""
    profile = catalog_profile('triangle_120')
    trajectory = build_trajectory(profile)
    points = profile.points_m
    segment_vectors = [
        (second[0] - first[0], second[1] - first[1])
        for first, second in zip(points, points[1:])
    ]
    segment_lengths = [math.hypot(*vector) for vector in segment_vectors]
    headings = [math.atan2(vector[1], vector[0]) for vector in segment_vectors]
    direction_changes = [
        math.degrees(
            math.acos(
                max(
                    -1.0,
                    min(
                        1.0,
                        sum(a * b for a, b in zip(first, second))
                        / (math.hypot(*first) * math.hypot(*second)),
                    ),
                )
            )
        )
        for first, second in zip(segment_vectors, segment_vectors[1:])
    ]

    expected_points = (
        (0.0, 0.0),
        (0.0, 3.0 * math.sqrt(3.0) / 4.0),
        (-0.75, 0.0),
        (0.0, -3.0 * math.sqrt(3.0) / 4.0),
        (0.0, 0.0),
    )
    for point, expected in zip(points, expected_points):
        assert point == pytest.approx(expected)
    assert segment_lengths == pytest.approx(
        (3.0 * math.sqrt(3.0) / 4.0, 1.5, 1.5,
         3.0 * math.sqrt(3.0) / 4.0)
    )
    assert direction_changes == pytest.approx((150.0, 60.0, 150.0))
    assert [180.0 - change for change in direction_changes] == pytest.approx(
        (30.0, 120.0, 30.0)
    )
    assert headings == pytest.approx(
        (math.pi / 2.0, -2.0 * math.pi / 3.0,
         -math.pi / 3.0, math.pi / 2.0)
    )
    assert profile.bounds_offsets_m() == pytest.approx(
        (-0.75, 0.0, -3.0 * math.sqrt(3.0) / 4.0,
         3.0 * math.sqrt(3.0) / 4.0)
    )
    assert trajectory.motion_phase_count == 4
    assert trajectory.total_dwell_s == 0.0
    assert trajectory.total_time == pytest.approx(
        (3.0 + 1.5 * math.sqrt(3.0)) / 0.15
    )
    assert trajectory.update(0.0) == pytest.approx(
        (0.0, 0.0, 0.0, 0.0, 0.15, 0.0)
    )


def test_cw_and_ccw_circles_have_same_start_velocity_and_mirrored_paths():
    """Make direction the only difference in the two circle profiles."""
    counterclockwise = build_trajectory(catalog_profile('circle_ccw'))
    clockwise = build_trajectory(catalog_profile('circle_cw'))

    assert isinstance(counterclockwise, CircleTraj)
    assert isinstance(clockwise, CircleTraj)
    assert counterclockwise.total_time == pytest.approx(
        4.0 * math.pi / 0.15
    )
    assert clockwise.total_time == pytest.approx(counterclockwise.total_time)
    expected_start = (0.0, 0.0, 0.0, 0.0, 0.15, 0.0)
    assert counterclockwise.update(0.0) == expected_start
    assert clockwise.update(0.0) == expected_start
    assert counterclockwise.points[0] == (0.0, 0.0, 0.0)
    assert clockwise.points[0] == (0.0, 0.0, 0.0)

    for fraction in (0.125, 0.25, 0.5, 0.75, 0.999):
        ccw = counterclockwise.update(
            fraction * counterclockwise.total_time
        )
        cw = clockwise.update(fraction * clockwise.total_time)
        assert cw[0] == pytest.approx(-ccw[0], abs=1e-12)
        assert cw[1] == pytest.approx(ccw[1], abs=1e-12)
        assert cw[3] == pytest.approx(-ccw[3], abs=1e-12)
        assert cw[4] == pytest.approx(ccw[4], abs=1e-12)
        assert math.hypot(ccw[3], ccw[4]) == pytest.approx(0.15)
        assert math.hypot(cw[3], cw[4]) == pytest.approx(0.15)

    assert counterclockwise.points[-1] == counterclockwise.points[0]
    assert clockwise.points[-1] == clockwise.points[0]
    assert counterclockwise.update(
        counterclockwise.total_time
    ) == pytest.approx((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), abs=1e-12)
    assert clockwise.update(clockwise.total_time) == pytest.approx(
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), abs=1e-12
    )


def test_translation_anchors_once_without_rotating_velocity_or_fixed_yaw():
    """Translate only XY while leaving axes, velocity, and yaw unchanged."""
    canonical = build_trajectory(catalog_profile('circle_ccw'))
    translated = TranslatedTrajectory(canonical, 1.25, -0.50)

    assert translated.points[0] == pytest.approx((1.25, -0.50, 0.0))
    canonical_quarter = canonical.update(canonical.total_time / 4.0)
    world_quarter = translated.update(translated.total_time / 4.0)
    assert world_quarter[:2] == pytest.approx(
        (canonical_quarter[0] + 1.25, canonical_quarter[1] - 0.50)
    )
    assert world_quarter[2:] == pytest.approx(canonical_quarter[2:])
    assert translated.update(translated.total_time) == pytest.approx(
        (1.25, -0.50, 0.0, 0.0, 0.0, 0.0)
    )


def write_profile(tmp_path, name, document):
    """Write one intentionally varied profile for validation tests."""
    path = tmp_path / f'{name}.yaml'
    path.write_text(yaml.safe_dump(document), encoding='utf-8')
    return path


def triangle_document():
    """Return a minimal valid closed polyline profile mapping."""
    return {
        'schema_version': 1,
        'name': 'test_profile',
        'kind': 'polyline',
        'description': 'test triangle',
        'closed': True,
        'default_speed_m_s': 0.15,
        'fixed_yaw_rad': 0.0,
        'points_m': [[0.0, 0.0], [0.0, 1.0], [0.0, 0.0]],
    }


def circle_document():
    """Return a minimal valid closed circle profile mapping."""
    return {
        'schema_version': 1,
        'name': 'test_circle',
        'kind': 'circle',
        'description': 'test circle',
        'closed': True,
        'default_speed_m_s': 0.15,
        'fixed_yaw_rad': 0.0,
        'center_m': [-1.0, 0.0],
        'radius_m': 1.0,
        'start_angle_rad': 0.0,
        'direction': 'ccw',
        'laps': 1,
        'visualization_samples': 33,
    }


def dwell_polyline_document():
    """Return a valid polyline with an explicit internal zero-speed dwell."""
    document = triangle_document()
    document['waypoint_dwells_s'] = [0.0, 1.0, 0.0]
    return document


@pytest.mark.parametrize(
    ('field', 'bad_value'),
    (
        ('default_speed_m_s', 0.0),
        ('default_speed_m_s', math.nan),
        ('fixed_yaw_rad', math.inf),
        ('closed', 'true'),
    ),
)
def test_profile_loader_rejects_invalid_common_values(
    tmp_path, field, bad_value
):
    """Reject invalid common schema values before constructing motion."""
    document = triangle_document()
    document[field] = bad_value

    with pytest.raises(ProfileValidationError):
        load_profile(write_profile(tmp_path, 'test_profile', document))


def test_profile_loader_accepts_only_declared_origin_modes(tmp_path):
    """Prevent an origin-policy typo from silently changing coordinates."""
    document = triangle_document()
    document['origin_mode'] = 'absolute_maybe'

    with pytest.raises(ProfileValidationError, match='origin_mode'):
        load_profile(write_profile(tmp_path, 'test_profile', document))


def test_profile_loader_rejects_nonorigin_polyline_and_circle(tmp_path):
    """Enforce the authoritative canonical-origin contract for all kinds."""
    polyline = triangle_document()
    polyline['points_m'][0] = [0.01, 0.0]
    circle = circle_document()
    circle['center_m'] = [-0.9, 0.0]

    with pytest.raises(ProfileValidationError, match='begin exactly'):
        load_profile(write_profile(tmp_path, 'test_profile', polyline))
    with pytest.raises(ProfileValidationError, match='begin exactly'):
        load_profile(write_profile(tmp_path, 'test_circle', circle))


@pytest.mark.parametrize(
    ('field', 'bad_value'),
    (
        ('direction', 'left'),
        ('radius_m', 0.0),
        ('laps', 0),
        ('visualization_samples', 4),
        ('closed', False),
    ),
)
def test_circle_profile_rejects_invalid_geometry(tmp_path, field, bad_value):
    """Reject unsafe or nonsensical analytic-circle geometry."""
    document = circle_document()
    document[field] = bad_value

    with pytest.raises(ProfileValidationError):
        load_profile(write_profile(tmp_path, 'test_circle', document))


def test_profile_loader_rejects_unknown_key_and_unsafe_lookup(tmp_path):
    """Reject schema typos and names that could traverse the catalog path."""
    document = triangle_document()
    document['typo_speed'] = 0.2

    with pytest.raises(ProfileValidationError, match='unexpected keys'):
        load_profile(write_profile(tmp_path, 'test_profile', document))
    with pytest.raises(ProfileValidationError):
        profile_path(tmp_path, '../triangle')


@pytest.mark.parametrize(
    'dwells',
    (
        [0.0, 1.0],
        [0.1, 1.0, 0.0],
        [0.0, 1.0, 0.1],
        [0.0, -1.0, 0.0],
        [0.0, math.nan, 0.0],
        [0.0, True, 0.0],
    ),
)
def test_profile_loader_rejects_invalid_waypoint_dwells(tmp_path, dwells):
    """Keep planned zero-speed holds finite, bounded, and internal."""
    document = dwell_polyline_document()
    document['waypoint_dwells_s'] = dwells

    with pytest.raises(ProfileValidationError):
        load_profile(write_profile(tmp_path, 'test_profile', document))


def test_metadata_is_deterministic_complete_and_reports_mirrored_bounds():
    """Record enough deterministic provenance to reconstruct an old run."""
    profile = catalog_profile('circle_cw')
    trajectory = build_trajectory(profile)
    document = metadata_document(
        profile=profile,
        trajectory=trajectory,
        speed_m_s=0.15,
        origin_x_m=4.0,
        origin_y_m=-2.0,
        captured_start_x_m=4.0,
        captured_start_y_m=-2.0,
        odom_frame_id='vicon/world',
        captured_start_yaw_rad=0.25,
        reference_timer_hz=50.0,
        startup_hold_s=2.0,
        final_hold_s=3.0,
        loop=False,
        run_id='hamr_traj_circle_cw_v015_20260824_120000',
        planned_id='G2_circle_cw_r03',
        controller_config_sha256='c' * 64,
        wheel_speed_limit_rad_s=2.93215314335,
        turret_control_enabled=False,
        nominal_start_base_yaw_rad=NOMINAL_START_BASE_YAW_RAD,
        start_base_yaw_tolerance_rad=START_BASE_YAW_TOLERANCE_RAD,
        study_stack_contract_version=STUDY_STACK_CONTRACT_VERSION,
        controller_odom_timeout_s=0.0,
        vicon_pose_guard_enabled=False,
        reference_timeout_s=REFERENCE_TIMEOUT_S,
        xy_velocity_source=XY_VELOCITY_SOURCE,
        caster_type='trailing',
        terrain='flat',
        initial_caster_orientation_deg=90.0,
        repetition=3,
        video_id='cam-a-003',
        notes='baseline',
    )
    encoded = metadata_json(document)

    assert METADATA_TOPIC == '/hamr_test/trajectory_metadata'
    assert document['origin_mode'] == ORIGIN_MODE
    assert document['captured_start_x_m'] == pytest.approx(4.0)
    assert document['captured_start_y_m'] == pytest.approx(-2.0)
    assert document['profile_yaml'] == profile.source_path.read_text(
        encoding='utf-8'
    )
    assert document['odom_frame_id'] == 'vicon/world'
    assert document['captured_start_yaw_rad'] == pytest.approx(0.25)
    assert document['run_id'] == (
        'hamr_traj_circle_cw_v015_20260824_120000'
    )
    assert document['planned_id'] == 'G2_circle_cw_r03'
    assert document['controller_config_sha256'] == 'c' * 64
    assert document['wheel_speed_limit_rad_s'] == pytest.approx(
        2.93215314335
    )
    assert document['turret_control_enabled'] is False
    assert document['nominal_start_base_yaw_rad'] == 0.0
    assert document['start_base_yaw_tolerance_rad'] == pytest.approx(0.15)
    assert document['study_stack_contract_version'] == 1
    assert document['controller_odom_timeout_s'] == 0.0
    assert document['vicon_pose_guard_enabled'] is False
    assert document['reference_timeout_s'] == pytest.approx(0.12)
    assert document['xy_velocity_source'] == 'odom_twist_world'
    assert document['bounds_offsets_m'] == pytest.approx(
        {'min_x': 0.0, 'max_x': 4.0, 'min_y': -2.0, 'max_y': 2.0}
    )
    assert document['bounds_world_m'] == pytest.approx(
        {'min_x': 4.0, 'max_x': 8.0, 'min_y': -4.0, 'max_y': 0.0}
    )
    assert document['expected_motion_duration_s'] == pytest.approx(
        4.0 * math.pi / 0.15
    )
    assert document['expected_total_duration_s'] == pytest.approx(
        5.0 + 4.0 * math.pi / 0.15
    )
    assert document['waypoint_dwells_s'] == []
    assert document['expected_motion_phase_count'] == 1
    assert document['expected_dwell_duration_s'] == 0.0
    assert document['repetition'] == 3
    assert json.loads(encoded) == document
    assert metadata_json(document) == encoded
    assert '"schema_version":1' in encoded


def test_forward_reverse_metadata_preserves_planned_phase_schedule():
    """Make the deliberate midpoint stop distinguishable from interruption."""
    profile = catalog_profile('forward_reverse')
    trajectory = build_trajectory(profile)
    document = metadata_document(
        profile=profile,
        trajectory=trajectory,
        speed_m_s=0.15,
        origin_x_m=1.0,
        origin_y_m=2.0,
        captured_start_x_m=1.0,
        captured_start_y_m=2.0,
        odom_frame_id='vicon/world',
        captured_start_yaw_rad=0.0,
        reference_timer_hz=50.0,
        startup_hold_s=2.0,
        final_hold_s=3.0,
        loop=False,
        run_id='forward_reverse_metadata_test',
        planned_id='G1_forward_reverse_r01',
        controller_config_sha256='d' * 64,
        wheel_speed_limit_rad_s=2.93215314335,
        turret_control_enabled=False,
        nominal_start_base_yaw_rad=NOMINAL_START_BASE_YAW_RAD,
        start_base_yaw_tolerance_rad=START_BASE_YAW_TOLERANCE_RAD,
        study_stack_contract_version=STUDY_STACK_CONTRACT_VERSION,
        controller_odom_timeout_s=0.0,
        vicon_pose_guard_enabled=False,
        reference_timeout_s=REFERENCE_TIMEOUT_S,
        xy_velocity_source=XY_VELOCITY_SOURCE,
        caster_type='traditional',
        terrain='flat',
        initial_caster_orientation_deg=0.0,
        repetition=1,
        video_id='',
        notes='',
    )

    assert document['waypoint_dwells_s'] == [0.0, 1.0, 0.0]
    assert document['expected_motion_phase_count'] == 2
    assert document['expected_dwell_duration_s'] == pytest.approx(1.0)
    assert document['expected_motion_duration_s'] == pytest.approx(
        2.0 / 0.15 + 1.0
    )
    assert document['expected_total_duration_s'] == pytest.approx(
        2.0 / 0.15 + 6.0
    )
    assert document['bounds_offsets_m'] == pytest.approx(
        {'min_x': 0.0, 'max_x': 0.0, 'min_y': 0.0, 'max_y': 1.0}
    )


def test_finite_odom_xy_accepts_only_finite_position_and_preserves_frame():
    """Capture finite pose provenance without using yaw to rotate the path."""
    def message(x, y, frame='vicon/world', quaternion=(0.0, 0.0, 0.0, 1.0)):
        position = SimpleNamespace(x=x, y=y)
        orientation = SimpleNamespace(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
        )
        pose = SimpleNamespace(
            pose=SimpleNamespace(
                position=position,
                orientation=orientation,
            )
        )
        return SimpleNamespace(
            pose=pose,
            header=SimpleNamespace(frame_id=frame),
        )

    assert finite_odom_xy(message(1.0, -2.0)) == (
        1.0,
        -2.0,
        'vicon/world',
    )
    assert finite_odom_xy(message(1.0, 2.0, '')) == (1.0, 2.0, 'odom')
    assert finite_odom_xy(message(math.nan, 0.0)) is None
    assert finite_odom_xy(message(0.0, math.inf)) is None
    yaw_quaternion = (0.0, 0.0, math.sin(0.3), math.cos(0.3))
    sample = finite_odom_sample(message(1.0, 2.0, quaternion=yaw_quaternion))
    assert sample[:3] == pytest.approx((1.0, 2.0, 0.6), abs=1e-12)
    assert sample[3] == 'vicon/world'
    assert finite_quaternion_yaw(
        SimpleNamespace(x=0.0, y=0.0, z=0.0, w=2.0)
    ) == pytest.approx(0.0)
    assert finite_odom_sample(
        message(1.0, 2.0, quaternion=(0.0, 0.0, 0.0, 0.0))
    ) is None


def test_study_metadata_field_validation_is_bounded_and_typed():
    """Keep operator labels bounded and repetition unambiguous."""
    assert validated_repetition(1) == 1
    assert validated_study_text(
        'run_id', 'manual_study_run', allow_empty=False, maximum_length=32
    ) == 'manual_study_run'
    assert validated_sha256('digest', 'A' * 64) == 'a' * 64
    assert validated_positive_float('limit', 2.5) == 2.5
    assert validated_boolean('turret', False) is False
    assert validated_contract_value('timeout', 0.12, 0.12) == 0.12
    assert validated_contract_value('source', 'odom_twist_world',
                                    'odom_twist_world') == (
        'odom_twist_world'
    )
    with pytest.raises(ValueError):
        validated_repetition(0)
    with pytest.raises(ValueError):
        validated_repetition(1.0)
    with pytest.raises(ValueError):
        validated_study_text(
            'run_id', '', allow_empty=False, maximum_length=32
        )
    with pytest.raises(ValueError):
        validated_study_text(
            'notes', 'x' * 5, allow_empty=True, maximum_length=4
        )
    for invalid_digest in ('', 'g' * 64, 'a' * 63):
        with pytest.raises(ValueError):
            validated_sha256('digest', invalid_digest)
    for invalid_limit in (True, 0.0, -1.0, math.nan):
        with pytest.raises(ValueError):
            validated_positive_float('limit', invalid_limit)
    with pytest.raises(ValueError):
        validated_boolean('turret', 0)
    with pytest.raises(ValueError):
        validated_contract_value('timeout', 0.0, 0.12)


def test_circle_uses_existing_exact_hold_and_one_shot_schedule():
    """Apply the proven endpoint-hold schedule to analytic circles."""
    trajectory = TranslatedTrajectory(
        build_trajectory(catalog_profile('circle_ccw')), 2.0, 3.0
    )
    playback = TrajectoryPlayback(
        trajectory.total_time,
        startup_hold_s=2.0,
        final_hold_s=3.0,
        loop=False,
    )

    startup = playback.step(10.0)
    assert startup.phase == PHASE_STARTUP_HOLD
    assert scheduled_reference(trajectory, startup) == pytest.approx(
        (2.0, 3.0, 0.0, 0.0, 0.0, 0.0)
    )
    motion = playback.step(12.0)
    assert motion.phase == PHASE_MOTION
    assert scheduled_reference(trajectory, motion)[3:] == pytest.approx(
        (0.0, 0.15, 0.0), abs=1e-12
    )
    endpoint_time = 12.0 + trajectory.total_time
    endpoint = playback.step(endpoint_time)
    assert endpoint.phase == PHASE_FINAL_HOLD
    assert scheduled_reference(trajectory, endpoint) == pytest.approx(
        (2.0, 3.0, 0.0, 0.0, 0.0, 0.0), abs=1e-12
    )
    assert playback.step(endpoint_time + 2.999).phase == PHASE_FINAL_HOLD
    assert not playback.step(endpoint_time + 3.0).publish
