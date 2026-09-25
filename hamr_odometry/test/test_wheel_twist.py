"""Physical wheel-kinematic and published-message regression checks."""

import math
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.time import Time

from hamr_odometry.holonomic_odom_node import (
    HolonomicOdomNode,
    positive_finite_parameter,
    wheel_body_twist,
)


def test_equal_wheels_follow_offset_axis_without_rotation():
    for offset in (0.0, math.pi / 2.0, -0.4):
        vx, vy, wz = wheel_body_twist(2.0, 2.0, 0.125, 0.35, 0.45, offset)
        assert vx == pytest.approx(0.25 * math.cos(offset), abs=1e-12)
        assert vy == pytest.approx(0.25 * math.sin(offset), abs=1e-12)
        assert wz == 0.0


def test_counterrotating_wheels_give_offset_point_tangential_speed():
    vx, vy, wz = wheel_body_twist(
        -1.0, 1.0, 0.125, 0.35, 0.45, math.pi / 2.0)
    assert wz == pytest.approx(0.125 / 0.35)
    assert vx == pytest.approx(-0.45 * wz)
    assert vy == pytest.approx(0.0, abs=1e-12)


def test_calibration_scales_axle_speed_and_rotation_independently():
    vx, vy, wz = wheel_body_twist(
        1.0, 3.0, 0.125, 0.35, 0.45, math.pi / 2.0,
        yaw_sign=-1.0, linear_velocity_scale=0.95, yaw_rate_scale=0.82,
    )
    assert vy == pytest.approx(0.25 * 0.95)
    assert wz == pytest.approx(-0.125 / 0.35 * 0.82)
    assert vx == pytest.approx(-0.45 * wz)


def fake_odometry_update(dt, heading):
    """Exercise the production callback at a constant analytic wheel speed."""
    now = Time(nanoseconds=2_000_000_000)
    odometry_messages = []
    node = SimpleNamespace(
        r=0.125, a=0.35, b=0.45, yaw_offset=math.pi / 2.0,
        yaw_sign=1.0, linear_velocity_scale=0.95, yaw_rate_scale=0.82,
        # One tick corresponds to one radian for this synthetic encoder.
        ticks_per_rev=2.0 * math.pi,
        left_tick_scale=1.0, right_tick_scale=1.0,
        latest_L=dt, latest_R=3.0 * dt, latest_T=0,
        prev_L=0.0, prev_R=0.0,
        last_time=Time(nanoseconds=2_000_000_000 - round(dt * 1e9)),
        x=0.0, y=0.0, theta=heading,
        odom_frame='odom', base_frame='base_link',
        twist_variance_vx=0.001, twist_variance_vy=0.002,
        twist_variance_wz=0.003,
        ticks_per_turret_rev=2704.0, latest_ekf_yaw=None,
        latest_ekf_yaw_time=None, tf_broadcaster=None,
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        get_logger=lambda: SimpleNamespace(debug=lambda _: None),
        pub_odom=SimpleNamespace(publish=odometry_messages.append),
        pub_turret_odom=SimpleNamespace(publish=lambda _: None),
    )
    HolonomicOdomNode._update(node)
    assert len(odometry_messages) == 1
    return node, odometry_messages[0]


@pytest.mark.parametrize('dt', [0.01, 0.02, 0.20])
@pytest.mark.parametrize('heading', [0.0, 1.2, -2.8])
def test_published_turning_twist_is_independent_of_heading_and_interval(dt, heading):
    # The old callback rotated the body twist by -wz*dt because it used the
    # updated pose heading to invert a velocity computed with the old heading.
    node, message = fake_odometry_update(dt, heading)
    expected_wz = 0.125 / 0.35 * 0.82
    expected_vx, expected_vy = -0.45 * expected_wz, 0.25 * 0.95
    twist = message.twist.twist
    assert twist.linear.x == pytest.approx(expected_vx)
    assert twist.linear.y == pytest.approx(expected_vy)
    assert twist.angular.z == pytest.approx(expected_wz)
    assert message.header.frame_id == 'odom'
    assert message.child_frame_id == 'base_link'
    # Pose remains the existing forward-Euler integration of the same twist.
    assert node.x == pytest.approx(dt * (
        math.cos(heading) * expected_vx - math.sin(heading) * expected_vy))
    assert node.y == pytest.approx(dt * (
        math.sin(heading) * expected_vx + math.cos(heading) * expected_vy))
    assert node.theta == pytest.approx(heading + expected_wz * dt)
    assert message.twist.covariance[0] == 0.001
    assert message.twist.covariance[7] == 0.002
    assert message.twist.covariance[35] == 0.003


@pytest.mark.parametrize('bad', [0.0, -1.0, math.nan, math.inf, -math.inf,
                                     None, True, 'invalid'])
def test_invalid_calibration_or_variance_is_rejected(bad):
    with pytest.raises(ValueError, match='finite and strictly positive'):
        positive_finite_parameter('linear_velocity_scale', bad)


def test_ros_parameter_overrides_are_loaded_and_validated():
    overrides = {
        'linear_velocity_scale': 0.95,
        'yaw_rate_scale': 0.82,
        'twist_variance_vx': 0.001,
        'twist_variance_vy': 0.002,
        'twist_variance_wz': 0.003,
    }
    args = ['--ros-args']
    for name, value in overrides.items():
        args.extend(['-p', f'{name}:={value}'])
    rclpy.init(args=args)
    node = None
    try:
        node = HolonomicOdomNode()
        for name, value in overrides.items():
            assert getattr(node, name) == value
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('name', [
    'linear_velocity_scale', 'yaw_rate_scale',
    'twist_variance_vx', 'twist_variance_vy', 'twist_variance_wz',
])
def test_ros_rejects_invalid_startup_calibration(name):
    rclpy.init(args=['--ros-args', '-p', f'{name}:=-0.1'])
    try:
        with pytest.raises(ValueError, match=name):
            HolonomicOdomNode()
    finally:
        rclpy.shutdown()
