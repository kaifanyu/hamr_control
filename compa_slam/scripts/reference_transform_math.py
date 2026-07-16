#!/usr/bin/env python3
"""Small, ROS-independent helpers for planar reference-frame transforms."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    """A planar rigid transform: translation followed by a yaw rotation."""

    x: float
    y: float
    yaw: float


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compose(parent_from_middle: Pose2D, middle_from_child: Pose2D) -> Pose2D:
    """Return ``parent_from_child`` from two chained planar transforms."""
    cosine = math.cos(parent_from_middle.yaw)
    sine = math.sin(parent_from_middle.yaw)
    return Pose2D(
        parent_from_middle.x
        + cosine * middle_from_child.x
        - sine * middle_from_child.y,
        parent_from_middle.y
        + sine * middle_from_child.x
        + cosine * middle_from_child.y,
        wrap_angle(parent_from_middle.yaw + middle_from_child.yaw),
    )


def inverse(parent_from_child: Pose2D) -> Pose2D:
    """Return the inverse planar transform, ``child_from_parent``."""
    cosine = math.cos(parent_from_child.yaw)
    sine = math.sin(parent_from_child.yaw)
    return Pose2D(
        -cosine * parent_from_child.x - sine * parent_from_child.y,
        sine * parent_from_child.x - cosine * parent_from_child.y,
        wrap_angle(-parent_from_child.yaw),
    )


def rotate_vector(yaw: float, x: float, y: float) -> tuple[float, float]:
    """Rotate a planar vector without applying translation."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def anchor_route_at_pose(map_from_base: Pose2D, route_from_reference: Pose2D) -> Pose2D:
    """Place a route so its current reference coincides with the current map pose."""
    return compose(map_from_base, inverse(route_from_reference))
