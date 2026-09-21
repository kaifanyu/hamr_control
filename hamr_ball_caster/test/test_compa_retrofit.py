"""Independent integration checks for the COMPA ball-caster retrofit.

These check mechanical fit, retained source integrity, passive articulation and
coarse chassis clearance. They do not claim a calibrated whole-robot model.
"""
from functools import lru_cache
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

from test_geometry import forward_kinematics, pose

PACKAGE = Path(__file__).resolve().parents[1]
PROVENANCE = json.loads((PACKAGE / 'assets/provenance/compa_retrofit.json').read_text())
ASSEMBLY = json.loads((PACKAGE / 'assets/provenance/assembly.json').read_text())
NATIVE_TO_ROS = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])


@lru_cache(maxsize=1)
def compa():
    return ET.fromstring(xacro.process_file(
        str(PACKAGE / 'urdf/compa_ball_caster.urdf.xacro'),
        mappings={'carrier_phase': '0', 'caster_mass_scale': '1'},
    ).toxml())


def test_original_description_sources_and_copied_assets_match_provenance():
    original = PACKAGE.parent / 'compa_description/urdf'
    for name, digest in PROVENANCE['source_files'].items():
        assert hashlib.sha256((original / name).read_bytes()).hexdigest() == digest, name
    for name, record in PROVENANCE['mesh_files'].items():
        path = PACKAGE / 'meshes/compa' / name
        assert hashlib.sha256(path.read_bytes()).hexdigest() == record['sha256'], name


def test_old_caster_links_and_embedded_visuals_are_removed():
    robot = compa()
    names = {element.get('name') for element in robot.findall('link') + robot.findall('joint')}
    assert not any(name.endswith('caster_wheel_link') or name.endswith('caster_wheel_joint') for name in names)
    for side in ('left', 'right'):
        rocker = robot.find(f"./link[@name='{side}_rocker_link']")
        # Old rocker STL contains a complete old caster and cannot stay visible.
        assert rocker.find('.//mesh') is None
        assert len(rocker.findall('visual/geometry/box')) == 1
        assert len(rocker.findall('collision/geometry/box')) == 1
        caster_visuals = [visual for link in robot.findall('link')
                          if link.get('name').startswith(side + '_caster_')
                          for visual in link.findall('visual')]
        assert len(caster_visuals) == len(ASSEMBLY['occurrences'])
    assert not any('rocker_link.STL' in element.get('filename') for element in robot.findall('.//mesh'))


def test_both_casters_have_five_independent_unactuated_joints():
    robot = compa()
    passive = [j for j in robot.findall('joint') if '_caster_' in j.get('name') and j.get('type') != 'fixed']
    assert len(passive) == 10
    for side in ('left', 'right'):
        joints = [j for j in passive if j.get('name').startswith(side + '_caster_')]
        assert len(joints) == 5
        assert all(j.get('type') == 'continuous' and j.find('mimic') is None for j in joints)
        carrier = side + '_caster_carrier_link'
        assert sum(j.find('parent').get('link') == carrier for j in joints) == 4
    commanded = {element.text for plugin in robot.findall('./gazebo/plugin')
                 if 'Controller' in plugin.get('name', '') for element in plugin.findall('joint_name')}
    assert not commanded & {j.get('name') for j in passive}


def test_square_rails_fit_and_fully_traverse_measured_fork_sockets():
    robot = compa()
    fk = forward_kinematics(robot)
    socket = np.array(ASSEMBLY['geometry']['fork_socket_center_native_m'])
    socket_width = ASSEMBLY['geometry']['fork_socket_inner_width_m']
    socket_height = ASSEMBLY['geometry']['fork_socket_inner_height_m']
    # Native fork extends ±50.8mm along its rail-channel direction.
    socket_half_length = .0508
    for side in ('left', 'right'):
        rocker_name = side + '_rocker_link'
        fork_name = side + '_caster_fork_link'
        rail = robot.find(f"./link[@name='{rocker_name}']/collision")
        dimensions = np.fromstring(rail.find('geometry/box').get('size'), sep=' ')
        rail_pose = pose(rail.find('origin'))
        fork_in_rocker = np.linalg.inv(fk[rocker_name]) @ fk[fork_name]
        socket_in_rocker = fork_in_rocker @ np.r_[NATIVE_TO_ROS @ socket, 1.]
        # The measured channel and the rail have coincident transverse centerlines.
        np.testing.assert_allclose(socket_in_rocker[1:3], rail_pose[1:3, 3], atol=1e-12)
        rail_direction = rail_pose[:3, 0]
        socket_direction = fork_in_rocker[:3, :3] @ NATIVE_TO_ROS @ np.array([0., 1., 0.])
        assert abs(rail_direction @ socket_direction) == pytest.approx(1., abs=1e-12)
        assert 0 < socket_width - dimensions[1] < .001
        assert 0 < socket_height - dimensions[2] < .001
        low, high = rail_pose[0, 3] + np.array([-1., 1.]) * dimensions[0] / 2
        assert low <= socket_in_rocker[0] - socket_half_length
        assert high >= socket_in_rocker[0] + socket_half_length - 1e-12
        wheel = robot.find(f"./joint[@name='{side}_rocker_{side}_wheel_joint']")
        wheel_z = pose(wheel.find('origin'))[2, 3]
        wheel_radius = float(robot.find(f"./link[@name='{side}_wheel_link']/collision/geometry/cylinder").get('radius'))
        ball_bottom = fork_in_rocker[2, 3] - ASSEMBLY['geometry']['nominal_sphere_radius_m']
        assert abs((wheel_z - wheel_radius) - ball_bottom) < .001


@lru_cache(maxsize=None)
def mesh_triangles(filename):
    prefix = 'package://hamr_ball_caster/'
    assert filename.startswith(prefix)
    data = (PACKAGE / filename[len(prefix):]).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + 50 * count
    dtype = np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')])
    return np.frombuffer(data, dtype=dtype, offset=84, count=count)['v'].astype(float)


def triangles_overlapping_box(triangles, half):
    """Triangle/AABB separating-axis test; excludes gaps, allows tolerance."""
    tolerance = 1e-9
    active = np.all(triangles.max(axis=1) >= -half + tolerance, axis=1)
    active &= np.all(triangles.min(axis=1) <= half - tolerance, axis=1)
    tri = triangles[active]
    if not len(tri):
        return 0
    edges = np.roll(tri, -1, axis=1) - tri
    axes = [np.cross(edges[:, 0], edges[:, 1])]
    for edge in range(3):
        for basis in np.eye(3):
            axes.append(np.cross(edges[:, edge], basis))
    separated = np.zeros(len(tri), dtype=bool)
    for axis in axes:
        nonzero = np.linalg.norm(axis, axis=1) > 1e-14
        projected = np.einsum('nvi,ni->nv', tri, axis)
        radius = np.abs(axis) @ half
        separated |= nonzero & ((projected.min(1) > radius + tolerance) |
                                (projected.max(1) < -radius - tolerance))
    return int(np.count_nonzero(~separated))


@pytest.mark.parametrize('carrier_angle', [0., 1.1])
def test_caster_meshes_clear_existing_chassis_primitives_at_neutral_rockers(carrier_angle):
    """Conservative primitive boxes; excludes intentional rail/socket insertion.

This checks actual collision triangles at two carrier phases. It is a geometric
regression check, not an all-configuration swept-volume or contact stability test.
"""
    robot = compa()
    fk = forward_kinematics(robot, {side + '_caster_carrier_joint': carrier_angle for side in ('left', 'right')})
    chassis = []
    for link in robot.findall('link'):
        name = link.get('name')
        if '_caster_' in name or name.endswith('_rocker_link'):
            continue
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            if geometry.find('box') is not None:
                half = np.fromstring(geometry.find('box').get('size'), sep=' ') / 2
            elif geometry.find('cylinder') is not None:
                cylinder = geometry.find('cylinder')
                radius = float(cylinder.get('radius'))
                half = np.array([radius, radius, float(cylinder.get('length')) / 2])
            elif geometry.find('sphere') is not None:
                half = np.repeat(float(geometry.find('sphere').get('radius')), 3)
            else:
                raise AssertionError('Unexpected chassis collision shape')
            chassis.append((name, np.linalg.inv(fk[name] @ pose(collision.find('origin'))), half))
    for link in robot.findall('link'):
        name = link.get('name')
        if '_caster_' not in name:
            continue
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            assert mesh is not None
            triangles = mesh_triangles(mesh.get('filename'))
            T = fk[name] @ pose(collision.find('origin'))
            for body, world_to_box, half in chassis:
                relative = world_to_box @ T
                box_triangles = triangles @ relative[:3, :3].T + relative[:3, 3]
                overlaps = triangles_overlapping_box(box_triangles, half)
                assert overlaps == 0, f'{name}/{collision.get("name")} overlaps {body} ({overlaps} triangles)'
