"""Physical invariants for the POWERTEC envelope and unchanged COMPA chassis.

These verify support, passive articulation, inertia, and camera framing. They do
not validate uncertain hardware parameters or replace a dynamic trajectory run.
"""
from functools import lru_cache
import math
import shutil
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((PACKAGE / 'config/physical_properties.yaml').read_text())


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1. - math.cos(angle)) * (skew @ skew)


def pose(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        roll, pitch, yaw = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
        transform[:3, :3] = rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll)
    return transform


def fk(robot, angles=None):
    angles = angles or {}
    joints = list(robot.findall('joint'))
    links = {link.get('name') for link in robot.findall('link')}
    children = [joint.find('child').get('link') for joint in joints]
    assert len(children) == len(set(children))
    roots = links - set(children)
    assert len(roots) == 1
    result = {roots.pop(): np.eye(4)}
    while joints:
        progressed = False
        for joint in joints[:]:
            parent = joint.find('parent').get('link')
            if parent not in result:
                continue
            child = joint.find('child').get('link')
            motion = np.eye(4)
            if joint.get('type') in ('continuous', 'revolute'):
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                motion[:3, :3] = rotation(axis, angles.get(joint.get('name'), 0.))
            result[child] = result[parent] @ pose(joint.find('origin')) @ motion
            joints.remove(joint)
            progressed = True
        assert progressed, 'URDF has a disconnected link or a cycle'
    assert set(result) == links
    return result


@lru_cache(maxsize=None)
def model(phase=0., mass_scale=1., camera=True, **overrides):
    return ET.fromstring(xacro.process_file(
        str(PACKAGE / 'urdf/compa_swivel_caster.urdf.xacro'),
        mappings={'initial_swivel_rad': str(phase), 'caster_mass_scale': str(mass_scale),
                  'enable_closeup_camera': str(camera).lower(),
                  **{name: str(value) for name, value in overrides.items()}},
    ).toxml())


def test_two_free_casters_with_positive_trail_and_no_caster_controller():
    robot = model()
    passive = [joint for joint in robot.findall('joint')
               if '_caster_' in joint.get('name') and joint.get('type') != 'fixed']
    assert len(passive) == 4
    commanded = {node.text for plugin in robot.findall('./gazebo/plugin')
                 if 'Controller' in plugin.get('name', '') for node in plugin.findall('joint_name')}
    assert not commanded.intersection(j.get('name') for j in passive)
    for side in ('right', 'left'):
        swivel = robot.find(f"./joint[@name='{side}_caster_swivel_joint']")
        wheel = robot.find(f"./joint[@name='{side}_caster_wheel_joint']")
        assert swivel.get('type') == wheel.get('type') == 'continuous'
        assert swivel.find('mimic') is None and wheel.find('mimic') is None
        assert swivel.find('axis').get('xyz') == '0 0 1'
        assert wheel.find('axis').get('xyz') == '0 1 0'
        assert pose(wheel.find('origin'))[0, 3] == pytest.approx(-.045)
        assert wheel.find('parent').get('link') == side + '_caster_fork_link'
        assert float(swivel.find('dynamics').get('damping')) > 0


@pytest.mark.parametrize('phase', [0., .6, math.pi, -math.pi/2])
def test_ground_support_and_manufacturer_envelope_at_all_swivel_phases(phase):
    robot = model(phase=phase)
    frames = fk(robot)
    for side in ('right', 'left'):
        radius = float(robot.find(f"./link[@name='{side}_caster_wheel_link']/collision/geometry/cylinder").get('radius'))
        width = float(robot.find(f"./link[@name='{side}_caster_wheel_link']/collision/geometry/cylinder").get('length'))
        assert radius == pytest.approx(.080)
        assert width == pytest.approx(.050)
        caster_bottom = frames[side + '_caster_wheel_link'][2, 3] - radius
        rear_bottom = frames[side + '_wheel_link'][2, 3] - .1075
        assert caster_bottom == pytest.approx(.0044, abs=1e-12)
        assert caster_bottom == pytest.approx(rear_bottom, abs=1e-12)
        assert frames[side + '_caster_mount_link'][2, 3] - caster_bottom == pytest.approx(.192)
        # The full swept tire is below the base collision's lower face.
        assert frames[side + '_caster_wheel_link'][2, 3] + radius < .195
        relative = np.linalg.inv(frames[side + '_rocker_link']) @ frames[side + '_caster_mount_link']
        assert relative[0, 3] == pytest.approx(.315)
        assert abs(relative[1, 3]) == pytest.approx(.072)
        assert relative[2, 3] == pytest.approx(-.0597)


def test_swivel_moves_trailing_contact_and_wheel_rotation_keeps_center_fixed():
    robot = model()
    straight = fk(robot)
    turned = fk(robot, {'right_caster_swivel_joint': math.pi/2, 'right_caster_wheel_joint': 1.2})
    wheel = 'right_caster_wheel_link'
    swivel = straight['right_caster_fork_link'][:3, 3]
    np.testing.assert_allclose(straight[wheel][:3, 3] - swivel, [-.045, 0, -.0932375], atol=1e-12)
    np.testing.assert_allclose(turned[wheel][:3, 3] - swivel, [0, -.045, -.0932375], atol=1e-12)
    rolled = fk(robot, {'right_caster_wheel_joint': 2.3})
    np.testing.assert_allclose(rolled[wheel][:3, 3], straight[wheel][:3, 3])
    assert not np.allclose(rolled[wheel][:3, :3], straight[wheel][:3, :3])


@pytest.mark.parametrize('scale', [.5, 1., 2.])
def test_mass_budget_and_inertia_are_physically_admissible(scale):
    robot = model(mass_scale=scale)
    for side in ('right', 'left'):
        caster_links = [robot.find(f"./link[@name='{side}_caster_{part}_link']")
                        for part in ('mount', 'fork', 'wheel')]
        assert sum(float(link.find('inertial/mass').get('value')) for link in caster_links) == pytest.approx(2.64*scale)
        for link in caster_links:
            i = link.find('inertial/inertia').attrib
            tensor = np.array([[float(i['ixx']), float(i['ixy']), float(i['ixz'])],
                               [float(i['ixy']), float(i['iyy']), float(i['iyz'])],
                               [float(i['ixz']), float(i['iyz']), float(i['izz'])]])
            values = np.linalg.eigvalsh(tensor)
            assert np.all(values > 0)
            assert values[-1] <= values[0] + values[1] + 1e-12


def test_chassis_rear_drives_and_odom_match_existing_ball_caster_model():
    original = ET.fromstring(xacro.process_file(str(PACKAGE.parent / 'hamr_ball_caster/urdf/compa_ball_caster.urdf.xacro')).toxml())
    robot = model()
    for tag in ('link', 'joint'):
        for node in original.findall(tag):
            if '_caster_' in node.get('name'):
                continue
            current = robot.find(f"./{tag}[@name='{node.get('name')}']")
            assert current is not None
            # Ignore whitespace-only XML indentation, compare substantive data.
            old = [(e.tag, e.attrib, (e.text or '').strip()) for e in node.iter()]
            new = [(e.tag, e.attrib, (e.text or '').strip()) for e in current.iter()]
            assert new == old
    assert robot.find("./gazebo/plugin[@name='gz::sim::systems::OdometryPublisher']/odom_topic").text == '/model/compa/odometry'
    assert robot.find("./gazebo/plugin[@name='gz::sim::systems::JointStatePublisher']/topic").text == '/swivel_caster/joint_states'


def test_attached_camera_sees_right_caster_through_full_swivel():
    robot = model()
    sensor = robot.find("./gazebo[@reference='caster_closeup_camera_link']/sensor")
    assert sensor.find('topic').text == '/hamr/swivel_caster/closeup/image'
    assert robot.find("./joint[@name='caster_closeup_camera_joint']/parent").get('link') == 'right_rocker_link'
    hfov = float(sensor.find('camera/horizontal_fov').text)
    aspect = float(sensor.find('camera/image/width').text) / float(sensor.find('camera/image/height').text)
    tan_h = math.tan(hfov/2)
    tan_v = tan_h/aspect
    for phase in np.linspace(-math.pi, math.pi, 25):
        frames = fk(robot, {'right_caster_swivel_joint': phase})
        camera_inv = np.linalg.inv(frames['caster_closeup_camera_link'])
        points = []
        for a in np.linspace(0, 2*math.pi, 40):
            for y in (-.025, .025):
                points.append(frames['right_caster_wheel_link'] @ np.array([.080*math.cos(a), y, .080*math.sin(a), 1]))
        for x in (-.0575, .0575):
            for y in (-.05, .05):
                points.append(frames['right_caster_mount_link'] @ np.array([x, y, 0, 1]))
        projected = (camera_inv @ np.array(points).T).T
        assert np.all(projected[:, 0] > .02)
        assert np.all(np.abs(projected[:, 1]/projected[:, 0]) < tan_h)
        assert np.all(np.abs(projected[:, 2]/projected[:, 0]) < tan_v)
    assert model(camera=False).find("./link[@name='caster_closeup_camera_link']") is None



def test_component_mass_and_inertia_scales_change_separately():
    nominal = model()
    varied = model(caster_wheel_mass_scale=1.5, caster_fork_mass_scale=.8,
                   caster_wheel_inertia_scale=2., caster_fork_inertia_scale=3.)
    for side in ('right', 'left'):
        for part, mass_ratio, inertia_ratio in [('wheel', 1.5, 3.), ('fork', .8, 2.4), ('mount', 1., 1.)]:
            path = f"./link[@name='{side}_caster_{part}_link']/inertial"
            old, new = nominal.find(path), varied.find(path)
            assert float(new.find('mass').get('value')) / float(old.find('mass').get('value')) == pytest.approx(mass_ratio)
            for field in ('ixx', 'iyy', 'izz'):
                assert float(new.find('inertia').get(field)) / float(old.find('inertia').get(field)) == pytest.approx(inertia_ratio)
    for name, frame in fk(nominal).items():
        np.testing.assert_allclose(fk(varied)[name], frame)


def test_custom_physical_file_and_explicit_override_precedence(tmp_path):
    config = yaml.safe_load((PACKAGE / 'config/physical_properties.yaml').read_text())
    config['joints']['swivel_damping_Nm_s_per_rad'] = .05
    config['joints']['swivel_friction_Nm'] = .02
    config['joints']['wheel_damping_Nm_s_per_rad'] = .012
    config['mass']['wheel_kg'] = 1.8
    config['geometry']['swivel_trail_m'] = .055
    config['contact']['mu'] = .3
    config_path = tmp_path / 'custom_parameters.yaml'
    config_path.write_text(yaml.safe_dump(config))
    custom = model(caster_physical_config=config_path)
    overridden = model(caster_physical_config=config_path, swivel_damping=.15, tire_mu=.1)
    for robot, damping, mu in [(custom, .05, .3), (overridden, .15, .1)]:
        for side in ('right', 'left'):
            dynamics = robot.find(f"./joint[@name='{side}_caster_swivel_joint']/dynamics")
            assert float(dynamics.get('damping')) == pytest.approx(damping)
            assert float(dynamics.get('friction')) == pytest.approx(.02)
            assert float(robot.find(f"./gazebo[@reference='{side}_caster_wheel_link']/mu1").text) == pytest.approx(mu)
            assert float(robot.find(f"./link[@name='{side}_caster_wheel_link']/inertial/mass").get('value')) == pytest.approx(1.8)
            assert pose(robot.find(f"./joint[@name='{side}_caster_wheel_joint']/origin"))[0, 3] == pytest.approx(-.055)


@pytest.mark.parametrize('overrides', [
    {'caster_wheel_mass_scale': 0}, {'caster_fork_inertia_scale': -1},
    {'swivel_damping': -1}, {'swivel_friction': 'nan'}, {'tire_mu': 'inf'},
    {'caster_trail': 0},
])
def test_invalid_physical_overrides_fail_before_simulation(overrides):
    with pytest.raises(xacro.XacroException, match='must be finite'):
        model(**overrides)


@pytest.mark.parametrize('overrides,mu,swivel_damping,swivel_friction', [
    ({}, .9, .015, 0.),
    ({'tire_mu': .3, 'swivel_damping': .15, 'swivel_friction': .1, 'wheel_damping': .018}, .3, .15, .1),
])
def test_urdf_to_sdf_retains_real_contact_and_joint_parameters(tmp_path, overrides, mu, swivel_damping, swivel_friction):
    # Crucial integration check: a named URDF tire collision previously caused
    # the link's Gazebo friction extension to disappear during SDF conversion.
    if shutil.which('gz') is None:
        pytest.skip('Gazebo command line is unavailable; source scripts/env.sh to check conversion')
    urdf = tmp_path / 'robot.urdf'
    ET.ElementTree(model(**overrides)).write(urdf)
    completed = subprocess.run(['gz', 'sdf', '-p', str(urdf)], text=True, capture_output=True, check=True)
    sdf = ET.fromstring(completed.stdout).find('model')
    for side in ('right', 'left'):
        tire = sdf.find(f"./link[@name='{side}_caster_wheel_link']/collision")
        assert tire.find('surface/friction/ode') is not None, completed.stderr
        for name in ('mu', 'mu2'):
            assert float(tire.findtext('surface/friction/ode/' + name)) == pytest.approx(mu)
        swivel = sdf.find(f"./joint[@name='{side}_caster_swivel_joint']/axis/dynamics")
        assert float(swivel.findtext('damping')) == pytest.approx(swivel_damping)
        assert float(swivel.findtext('friction')) == pytest.approx(swivel_friction)
        wheel = sdf.find(f"./joint[@name='{side}_caster_wheel_joint']/axis/dynamics")
        assert float(wheel.findtext('damping')) == pytest.approx(overrides.get('wheel_damping', .006))
