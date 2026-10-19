"""Coverage, reference timing, and unchanged physical terrain for replays."""
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from terrain_replay_paths import (Reference, SHAPES, STRESS_SHAPES, SUPPORTED_SHAPES,
                                 prepare_replay_world, reference_for_terrain,
                                 augment_robot_cameras, replay_camera_configuration)


@pytest.mark.parametrize('shape', SHAPES)
def test_map_scale_reference_bounds_endpoints_and_timing(shape):
    reference = Reference(shape)
    assert reference.length > 6
    points = reference.points
    assert np.max(abs(points[:, 0])) <= 2.20001
    assert np.max(abs(points[:, 1])) <= 3.20001
    assert np.ptp(points[:, 1]) >= 4.399
    if shape != 'straight':
        assert np.ptp(points[:, 0]) >= 4.399
        np.testing.assert_allclose(points[0], points[-1], atol=1e-12)
    np.testing.assert_allclose(reference.sample(-2)[0], points[0])
    np.testing.assert_allclose(reference.sample(reference.duration+2)[0], points[-1], atol=1e-12)
    assert np.linalg.norm(reference.sample(0)[1]) == 0
    assert np.linalg.norm(reference.sample(reference.duration)[1]) == 0
    np.testing.assert_allclose(np.linalg.norm(reference.sample(reference.duration/2)[1]), .25)
    times = np.linspace(0, reference.duration, 10001)
    integrated_speed = np.trapz([np.linalg.norm(reference.sample(t)[1]) for t in times], times)
    assert abs(integrated_speed-reference.length) < 1e-4


@pytest.mark.parametrize('shape,n', [('triangle',4),('square',5)])
def test_polygon_keeps_exact_requested_corners(shape,n):
    reference = Reference(shape)
    assert len(reference.points)==n
    for distance, point in zip(reference.starts[1:-1], reference.points[1:-1]):
        arrival = distance/reference.speed+.5*reference.ramp
        np.testing.assert_allclose(reference.sample(arrival)[0], point, atol=1e-12)


def test_replay_world_changes_no_physics_or_collision(tmp_path):
    source = PACKAGE/'worlds/gentle_hill.sdf'
    output = tmp_path/'world.sdf'
    before = ET.parse(source).getroot().find('world')
    with np.load(PACKAGE/'meshes/gentle_hill_height.npz') as grid:
        prepare_replay_world(source,output,grid,Reference('figure_eight'))
    after = ET.parse(output).getroot().find('world')
    assert ET.tostring(before.find('physics'))==ET.tostring(after.find('physics'))
    assert before.find('gravity').text==after.find('gravity').text
    assert len(before.findall('.//collision[geometry]'))==len(after.findall('.//collision[geometry]'))
    assert not after.findall("model[@name='planned_route_visual']//collision")
    for a,b in zip(before.findall('.//collision[geometry]'),after.findall('.//collision[geometry]')):
        for uri in a.findall('.//uri'):
            uri.text=str((PACKAGE/uri.text.removeprefix('model://hamr_terrain/')).resolve())
        assert ET.tostring(a)==ET.tostring(b)
    assert len(after.findall('.//sensor[@type="camera"]'))==1
    assert after.find('.//sensor[@type="camera"]/update_rate').text=='15'


@pytest.mark.parametrize('speed', [0,-1,float('nan'),float('inf')])
def test_invalid_speed_rejected(speed):
    with pytest.raises(ValueError): Reference('straight',speed)


def test_stress_shapes_are_opt_in_and_do_not_redefine_original_campaign():
    assert SHAPES == ('straight', 'circle', 'triangle', 'square', 'figure_eight')
    assert set(STRESS_SHAPES).isdisjoint(SHAPES)
    assert set(STRESS_SHAPES).issubset(SUPPORTED_SHAPES)
    # These are externally saved reference contracts, not stress timing.
    assert Reference('straight').duration == pytest.approx(26.6)
    assert Reference('square').duration == pytest.approx(71.4)
    assert 'stress_reference' not in Reference('square').as_dict()


@pytest.mark.parametrize('shape', STRESS_SHAPES)
def test_stress_reference_fits_patch_and_has_bounded_continuous_velocity(shape):
    reference = Reference(shape, .25)
    assert 15 < reference.duration < 35
    assert np.max(abs(reference.points[:, 0]))+.65 < 3
    assert np.max(abs(reference.points[:, 1]))+.65 < 4
    np.testing.assert_allclose(reference.sample(-1)[0], reference.points[0], atol=1e-12)
    np.testing.assert_allclose(reference.sample(reference.duration+1)[0], reference.points[-1], atol=1e-12)
    assert np.linalg.norm(reference.sample(-1)[1]) == 0
    assert np.linalg.norm(reference.sample(reference.duration+1)[1]) == 0
    times = np.linspace(0, reference.duration, 10001)
    velocities = np.array([reference.sample(t)[1] for t in times])
    speeds = np.linalg.norm(velocities, axis=1)
    assert speeds.max() <= .25+1e-12
    assert abs(np.trapz(speeds, times)-reference.length) < 1e-5
    accelerations = np.linalg.norm(np.diff(velocities, axis=0)/np.diff(times)[:, None], axis=1)
    # A direction jump at cruising speed would exceed this by orders of magnitude.
    assert accelerations.max() < .40
    metadata = json.loads(json.dumps(reference.as_dict()))['stress_reference']
    assert metadata['minimum_nominal_footprint_edge_clearance_m'] > 1
    assert metadata['caster_joints_actuated'] is False
    assert metadata['chassis_heading_prescribed'] is False


@pytest.mark.parametrize('shape', ['stress_reversal', 'stress_lateral', 'stress_zigzag'])
def test_stress_polygon_brakes_and_dwells_at_each_requested_corner(shape):
    reference = Reference(shape)
    assert len(reference.stop_schedule) == 3
    for leg in reference.stop_schedule[:-1]:
        i = leg['segment']
        arrival, departure = leg['motion_end_time_s'], leg['dwell_end_time_s']
        assert departure-arrival >= .25-1e-12
        for t in (arrival, (arrival+departure)/2, departure):
            position, velocity, _ = reference.sample(t)
            np.testing.assert_allclose(position, reference.points[i+1], atol=1e-12)
            np.testing.assert_allclose(velocity, [0, 0], atol=1e-12)
        for t in (arrival-1e-6, departure+1e-6):
            assert np.linalg.norm(reference.sample(t)[1]) < 3e-7


def test_reversal_has_two_exact_travel_reversals_without_prescribing_body_yaw():
    reference = Reference('stress_reversal')
    velocities = [reference.sample((leg['start_time_s']+leg['motion_end_time_s'])/2)[1]
                  for leg in reference.stop_schedule]
    np.testing.assert_allclose(velocities, [[0, .25], [0, -.25], [0, .25]])
    assert reference.length == pytest.approx(6.6)
    assert reference.duration == pytest.approx(30.1)
    assert 'Chassis heading is a controller outcome' in reference.as_dict()['stress_reference']['description']


def test_compact_eight_velocity_matches_position_derivative_at_crossings_and_table_knots():
    reference = Reference('stress_tight_figure_eight')
    assert reference.length == pytest.approx(7.035786590919, abs=1e-10)
    # Each lobe passes the central crossing, with distinct diagonal tangents.
    crossing_times = reference.starts[[360, 1080]]/reference.speed+.5*reference.ramp
    crossing_velocities = []
    for t in crossing_times:
        xy, velocity, _ = reference.sample(t)
        np.testing.assert_allclose(xy, [0, 0], atol=1e-12)
        crossing_velocities.append(velocity)
    assert crossing_velocities[0][1] > 0 > crossing_velocities[1][1]
    times = np.r_[np.linspace(.01, reference.duration-.01, 301), crossing_times]
    epsilon = 1e-5
    for t in times:
        derivative = (reference.sample(t+epsilon)[0]-reference.sample(t-epsilon)[0])/(2*epsilon)
        np.testing.assert_allclose(derivative, reference.sample(t)[1], atol=1e-7)


@pytest.mark.parametrize('shape', STRESS_SHAPES)
def test_nominal_inverse_drive_motion_stays_within_limits_and_eight_rotates_chassis(shape):
    # Exercise the same rear-wheel kinematics used by both replay vehicles.
    # This is an ideal planar prediction, not evidence of Gazebo tracking.
    module_path = PACKAGE.parent/'hamr_ball_caster/scripts/run_waypoint_sim.py'
    spec = importlib.util.spec_from_file_location('stress_kinematics', module_path)
    shared = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(shared)
    reference = Reference(shape)
    dt, yaw = .002, reference.yaw
    headings, wheel_speeds = [yaw], []
    for t in np.arange(0, reference.duration, dt):
        velocity = reference.sample(t+dt/2)[1]
        left, right, yaw_rate = shared.inverse_drive(*velocity, yaw, .1075, .33072, .27114)
        wheel_speeds.append([left, right])
        yaw += dt*yaw_rate
        headings.append(yaw)
    wheel_speeds = np.array(wheel_speeds)
    assert np.max(abs(wheel_speeds)) < 6
    assert np.max(abs(np.diff(wheel_speeds, axis=0)/dt)) < 15
    if shape == 'stress_tight_figure_eight':
        assert np.ptp(headings) > math.radians(270)
        assert np.sum(abs(np.diff(headings))) > math.radians(500)
    if shape == 'stress_reversal':
        assert np.ptp(headings) < 1e-10  # Backing does not require a chassis turn.


@pytest.mark.parametrize('shape', STRESS_SHAPES)
def test_out_and_back_exactly_retraces_position_with_opposite_velocity(shape):
    outward=Reference(shape)
    combined=Reference(shape+'_out_back')
    assert combined.duration == pytest.approx(2*outward.duration+.75)
    assert combined.length == pytest.approx(2*outward.length)
    for t in np.linspace(0,outward.duration,1001):
        xy,velocity,_=outward.sample(t)
        returned,backward,_=combined.sample(combined.return_start+outward.duration-t)
        np.testing.assert_allclose(returned,xy,atol=1e-10)
        np.testing.assert_allclose(backward,-velocity,atol=1e-10)
    np.testing.assert_allclose(combined.sample(combined.duration)[0],outward.points[0],atol=1e-12)
    for t in np.linspace(outward.duration,combined.return_start,20):
        xy,velocity,_=combined.sample(t)
        np.testing.assert_allclose(xy,outward.points[-1],atol=1e-12)
        np.testing.assert_allclose(velocity,0,atol=1e-12)
    assert combined.leg_at(outward.duration/2)=='outbound'
    assert combined.leg_at(outward.duration+.1)=='turnaround'
    assert combined.leg_at(combined.return_start+.1)=='return'
    assert combined.as_dict()['out_and_back']['chassis_heading_prescribed'] is False


def map_metadata():
    return {'bounds_m': {'x': [-30., 30.], 'y': [-30., 30.]},
            'replay_route': {'points_m': [[-20., -15.], [0., -15.], [15., 10.]],
                             'description': 'A metric route through the full source map'}}


def test_metric_map_route_stops_at_corners_and_preserves_waypoints():
    metadata = map_metadata()
    reference = reference_for_terrain('map_route', .25, metadata)
    np.testing.assert_allclose(reference.points, metadata['replay_route']['points_m'])
    assert reference.length == pytest.approx(20+math.hypot(15, 25))
    assert reference.yaw == 0
    corner = reference.stop_schedule[0]['motion_end_time_s']
    np.testing.assert_allclose(reference.sample(corner)[0], [0, -15])
    np.testing.assert_allclose(reference.sample(corner)[1], [0, 0])
    np.testing.assert_allclose(reference.sample(reference.duration)[0], [15, 10])
    assert reference.as_dict()['map_route']['clearance_m'] == .65
    assert reference.as_dict()['map_route']['stop_at_waypoints'] is True
    # Opt-in metadata cannot silently rescale a historical reference.
    assert reference_for_terrain('straight', .25, metadata).as_dict() == Reference('straight').as_dict()


def test_map_route_supports_continuous_polyline_and_heading_override():
    metadata = map_metadata()
    metadata['replay_route'].update(stop_at_waypoints=False, initial_yaw_rad=1.2)
    reference = reference_for_terrain('map_route', .3, metadata)
    assert reference.stop_schedule == []
    assert reference.yaw == 1.2
    assert reference.duration == pytest.approx(reference.length/.3+1)


def test_metric_map_route_return_is_exact_time_reversal():
    metadata = map_metadata()
    outward = reference_for_terrain('map_route', .25, metadata)
    combined = reference_for_terrain('map_route_out_back', .25, metadata)
    for t in np.linspace(0, outward.duration, 101):
        position, velocity, _ = outward.sample(t)
        returned, reversed_velocity, _ = combined.sample(combined.return_start+outward.duration-t)
        np.testing.assert_allclose(returned, position, atol=1e-10)
        np.testing.assert_allclose(reversed_velocity, -velocity, atol=1e-10)
    assert combined.as_dict()['map_route']['bounds_m'] == metadata['bounds_m']


@pytest.mark.parametrize('change', [
    {'points_m': [[0, 0]]}, {'points_m': [[0, 0], [float('nan'), 1]]},
    {'points_m': [[0, 0], [0, 0]]}, {'points_m': [[0, 0], [29.5, 0]]},
    {'points_m': [[0, 0, 0], [1, 1, 1]]}, {'clearance_m': -1},
    {'clearance_m': float('inf')}, {'initial_yaw_rad': float('nan')},
    {'stop_at_waypoints': 'false'},
])
def test_map_route_rejects_invalid_geometry_and_metadata(change):
    metadata = map_metadata()
    metadata['replay_route'].update(change)
    with pytest.raises(ValueError):
        reference_for_terrain('map_route', .25, metadata)


def test_map_route_requires_map_metadata_and_accepts_exact_clearance():
    with pytest.raises(ValueError, match='metadata'):
        reference_for_terrain('map_route', .25, {})
    metadata = map_metadata()
    metadata['replay_route']['points_m'] = [[-29.35, -29.35], [29.35, 29.35]]
    assert reference_for_terrain('map_route', .25, metadata).length > 80


def test_replay_camera_override_preserves_source_world_and_physics(tmp_path):
    from generate_terrains import look_at
    source = PACKAGE/'worlds/road_bumps.sdf'
    output = tmp_path/'world.sdf'
    original = source.read_bytes()
    camera = {'eye': [15., -18., 14.], 'target': [1., 2., .5], 'far': 500.}
    with np.load(PACKAGE/'meshes/road_bumps_height.npz') as grid:
        prepare_replay_world(source, output, grid, Reference('straight'),
                             metadata={'replay_camera': camera})
    world = ET.parse(output).getroot().find('world')
    original_world = ET.fromstring(original).find('world')
    np.testing.assert_allclose(list(map(float, world.find("model[@name='overview_camera']/pose").text.split())),
                               look_at(camera['eye'], camera['target']))
    assert world.find("model[@name='overview_camera']/link/sensor/camera/clip/far").text == '500.0'
    assert ET.tostring(world.find('physics')) == ET.tostring(original_world.find('physics'))
    assert source.read_bytes() == original


def test_dense_spawn_matches_legacy_and_sparse_spawn_encloses_cells():
    from run_terrain_replay import spawn_height, support_floor
    x = np.linspace(-3, 3, 241)
    y = np.linspace(-4, 4, 321)
    xx, yy = np.meshgrid(x, y)
    z = .1*xx+.2*yy
    start = np.array([.3, -1.2])
    legacy_mask = (abs(xx-start[0]) <= .65) & (abs(yy-start[1]) <= .65)
    assert spawn_height({'x': x, 'y': y, 'z': z}, start) == float(z[legacy_mask].max()+.025)
    sparse = {'x': np.array([0., 2., 4.]), 'y': np.array([0., 2., 4.]),
              'z': np.array([[0., 1., 2.], [3., 4., 5.], [6., 7., 8.]])}
    assert spawn_height(sparse, [1., 1.]) == pytest.approx(4.025)
    assert support_floor({}) == -.25
    assert support_floor({'support_floor_z_m': -15.}) == -15.
    with pytest.raises(ValueError, match='finite'):
        support_floor({'support_floor_z_m': float('nan')})


@pytest.mark.parametrize('axis', [0, 1])
def test_fractional_sparse_spawn_clears_rising_cell_inside_footprint(axis):
    from generate_terrains import triangle_height
    from run_terrain_replay import spawn_height
    coordinates = np.arange(5.)
    heights = np.broadcast_to([0., 0., 0., 2., 2.], (5, 5))
    if axis == 1:
        heights = heights.T
    start = np.array([1.5, 1.5])
    # Both axes have two vertices inside the footprint, but these omit the
    # rising cell intersected by its outer 0.15 m on the positive side.
    supported_point = start.copy()
    supported_point[axis] += .6
    surface = triangle_height(coordinates, coordinates, heights, *supported_point)
    assert surface == pytest.approx(.2)
    spawn = spawn_height({'x': coordinates, 'y': coordinates, 'z': heights}, start)
    assert spawn == 2.025
    assert spawn > surface



def noncamera_robot_structure(description):
    robot = ET.fromstring(description)
    for parent in robot.iter():
        for child in list(parent):
            if child.tag == 'sensor' and child.get('type') == 'camera':
                parent.remove(child)
    for gazebo in list(robot.findall('gazebo')):
        if not len(gazebo) and not (gazebo.text or '').strip():
            robot.remove(gazebo)
    for element in robot.iter():
        element.text = (element.text or '').strip() or None
        element.tail = None
    return ET.canonicalize(ET.tostring(robot, encoding='unicode'))


@pytest.mark.parametrize('caster', ['ball', 'swivel'])
def test_vehicle_follow_adds_only_sensor_to_real_robot_and_aligns_closeup_rate(caster):
    from generate_terrains import look_at
    package = PACKAGE.parent/f'hamr_{caster}_caster'
    source = package/'urdf'/('compa_ball_caster_recording.urdf.xacro' if caster == 'ball'
                            else 'compa_swivel_caster.urdf.xacro')
    description = subprocess.check_output(['xacro', str(source), 'enable_closeup_camera:=true'], text=True)
    metadata = {'replay_camera': {'mode': 'vehicle_follow', 'eye': [100, 100, 80],
                                  'target': [40, 30, 0]}}
    updated = augment_robot_cameras(description, metadata=metadata, fps=15)
    assert noncamera_robot_structure(updated) == noncamera_robot_structure(description)
    robot = ET.fromstring(updated)
    cameras = robot.findall('.//sensor[@type="camera"]')
    assert len(cameras) == 2
    overview = [s for s in cameras if s.findtext('topic') == '/hamr/terrain/overview/image']
    assert len(overview) == 1
    assert robot.find("gazebo[@reference='base_link']/sensor[@name='terrain_vehicle_follow']") is not None
    assert overview[0].findtext('camera/image/width') == '1280'
    assert overview[0].findtext('camera/image/height') == '960'
    assert all(s.findtext('update_rate') == '15' for s in cameras)
    np.testing.assert_allclose(list(map(float, overview[0].findtext('pose').split())),
                               look_at([-3, -4, 3.5], [0, 0, .25]))
    # Reapplying camera adaptation cannot create a duplicate image publisher.
    again = ET.fromstring(augment_robot_cameras(updated, metadata=metadata, fps=15))
    assert sum(s.findtext('topic') == '/hamr/terrain/overview/image'
               for s in again.findall('.//sensor')) == 1


def test_vehicle_follow_world_snapshot_removes_static_camera_without_physics_change(tmp_path):
    source = PACKAGE/'worlds/road_bumps.sdf'
    original = source.read_bytes()
    output = tmp_path/'world.sdf'
    metadata = {'replay_camera': {'mode': 'vehicle_follow'}}
    with np.load(PACKAGE/'meshes/road_bumps_height.npz') as grid:
        prepare_replay_world(source, output, grid, Reference('straight'), metadata=metadata)
    before = ET.fromstring(original).find('world')
    after = ET.parse(output).getroot().find('world')
    assert after.find("model[@name='overview_camera']") is None
    assert not after.findall('.//sensor[@type="camera"]')
    assert ET.tostring(before.find('physics')) == ET.tostring(after.find('physics'))
    for a, b in zip(before.findall('.//collision[geometry]'), after.findall('.//collision[geometry]')):
        for uri in a.findall('.//uri'):
            uri.text = str((PACKAGE/uri.text.removeprefix('model://hamr_terrain/')).resolve())
        assert ET.tostring(a) == ET.tostring(b)
    assert source.read_bytes() == original


def test_fixed_camera_robot_adaptation_does_not_add_follow_sensor():
    original = '<robot name="test"><link name="base_link"/></robot>'
    updated = augment_robot_cameras(original, fps=15)
    assert noncamera_robot_structure(updated) == noncamera_robot_structure(original)
    assert not ET.fromstring(updated).findall('.//sensor')
    assert replay_camera_configuration({}) == {'mode': 'fixed'}


@pytest.mark.parametrize('camera', [
    {'mode': 'unknown'}, {'mode': 'vehicle_follow', 'local_eye': [0, 0]},
    {'mode': 'vehicle_follow', 'local_eye': [0, 0, .25]},
    {'mode': 'vehicle_follow', 'far': .01},
])
def test_follow_camera_rejects_invalid_configuration(camera):
    with pytest.raises(ValueError):
        augment_robot_cameras('<robot><link name="base_link"/></robot>',
                             metadata={'replay_camera': camera})
