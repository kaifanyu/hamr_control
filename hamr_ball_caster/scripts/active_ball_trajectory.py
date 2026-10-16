"""Deterministic ideal holonomic motion for the four-active-ball demo.

This is a geometric, no-slip visualization model, not a motor/contact dynamics
simulation.  Four spherical drives may command planar rolling independently.
The hardcoded route visits each corner and edge midpoint of a 2.4 m square,
returning to its center between visits while the chassis keeps yaw = 0.

Coordinates are metres, +X is chassis forward, +Y is left/north, and +Z is up.
Ball angular velocities are rad/s. Rotation matrices map ball-local vectors
into world coordinates; they integrate the exact rolling angle of each leg.
"""

from bisect import bisect_right
import math

import numpy as np


BALL_RADIUS = 0.10
BALL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")
BALL_OFFSETS = np.array(((0.30, 0.27), (0.30, -0.27),
                         (-0.30, 0.27), (-0.30, -0.27)), dtype=float)
BOX_HALF_EXTENT = 1.20
PEAK_SPEED = 0.70
TARGET_DWELL = 0.50
CENTER_DWELL = 0.35
START_HOLD = 3.0
FINISH_HOLD = 3.0
TARGETS = [
    {"name": "North-east corner", "label": "NE", "xy": (1.2, 1.2)},
    {"name": "North edge midpoint", "label": "N", "xy": (0.0, 1.2)},
    {"name": "North-west corner", "label": "NW", "xy": (-1.2, 1.2)},
    {"name": "West edge midpoint", "label": "W", "xy": (-1.2, 0.0)},
    {"name": "South-west corner", "label": "SW", "xy": (-1.2, -1.2)},
    {"name": "South edge midpoint", "label": "S", "xy": (0.0, -1.2)},
    {"name": "South-east corner", "label": "SE", "xy": (1.2, -1.2)},
    {"name": "East edge midpoint", "label": "E", "xy": (1.2, 0.0)},
]


def _geometry(radius, offsets):
    radius = float(radius)
    offsets = np.asarray(offsets, dtype=float)
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("Ball radius must be finite and positive")
    if offsets.ndim != 2 or offsets.shape[1] != 2 or not len(offsets):
        raise ValueError("Ball offsets must have shape (N, 2)")
    if not np.isfinite(offsets).all():
        raise ValueError("Ball offsets must be finite")
    return radius, offsets


def inverse_kinematics(vx, vy, yaw_rate=0.0, *, radius=BALL_RADIUS,
                       offsets=BALL_OFFSETS):
    """Return ideal absolute ball angular velocities in chassis coordinates.

    Input is a chassis-frame body twist (vx, vy, yaw_rate). Each ball center
    moves at (vx - yaw_rate*y, vy + yaw_rate*x). Choose zero vertical spin,
    since point-contact no-slip does not constrain that angular component.
    At nonzero chassis yaw, rotate these vectors into world coordinates.
    """
    radius, offsets = _geometry(radius, offsets)
    twist = np.asarray((vx, vy, yaw_rate), dtype=float)
    if not np.isfinite(twist).all():
        raise ValueError("Body twist must be finite")
    velocities = np.column_stack((vx - yaw_rate * offsets[:, 1],
                                  vy + yaw_rate * offsets[:, 0]))
    return np.column_stack((-velocities[:, 1] / radius,
                             velocities[:, 0] / radius,
                             np.zeros(len(offsets))))


def forward_kinematics(ball_omega_body, *, radius=BALL_RADIUS,
                       offsets=BALL_OFFSETS):
    """Least-squares body twist from absolute ball rates in chassis axes.

    Vertical spin is ignored because it does not move the point contact.
    Offsets must admit a unique three-component body twist.
    """
    radius, offsets = _geometry(radius, offsets)
    rates = np.asarray(ball_omega_body, dtype=float)
    if rates.shape != (len(offsets), 3) or not np.isfinite(rates).all():
        raise ValueError("Ball angular velocities must be finite with shape (N, 3)")
    design = np.zeros((2 * len(offsets), 3))
    design[0::2, 0] = 1.0
    design[1::2, 1] = 1.0
    design[0::2, 2] = -offsets[:, 1]
    design[1::2, 2] = offsets[:, 0]
    velocities = np.column_stack((radius * rates[:, 1], -radius * rates[:, 0]))
    twist, _, rank, _ = np.linalg.lstsq(design, velocities.ravel(), rcond=None)
    if rank < 3:
        raise ValueError("Ball offsets do not determine a unique body twist")
    return twist


def rotation_from_vector(vector):
    """Rodrigues rotation for an axis-angle vector expressed in radians."""
    vector = np.asarray(vector, dtype=float)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-15:
        return np.eye(3)
    x, y, z = vector / angle
    cross = np.array(((0., -z, y), (z, 0., -x), (-y, x, 0.)))
    return np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * (cross @ cross)


class DemoTrajectory:
    """Hardcoded center/target/center route with exact random-access sampling.

    A quintic ease polynomial gives zero velocity and acceleration at every
    stop. Its peak derivative is 15/8, so each leg lasts distance*15/(8*speed).
    Sampling never advances state: frame rate and sampling order cannot affect
    pose, rolling orientation, visit counts, or the final result.
    """

    def __init__(self):
        self.segments = []
        self._rotation_starts = []
        rotations = np.repeat(np.eye(3)[None, :, :], 4, axis=0)

        def append(phase, duration, start_xy, end_xy, target_index, completed):
            nonlocal rotations
            start_time = self.segments[-1]["end_time"] if self.segments else 0.0
            self.segments.append({
                "phase": phase,
                "start_time": start_time,
                "end_time": start_time + duration,
                "duration": duration,
                "start_xy": tuple(start_xy),
                "end_xy": tuple(end_xy),
                "target_index": target_index,
                "target_name": TARGETS[target_index]["name"] if target_index >= 0 else "Center",
                "completed_targets": completed,
            })
            self._rotation_starts.append(rotations.copy())
            dx, dy = np.asarray(end_xy) - np.asarray(start_xy)
            increment = rotation_from_vector((-dy / BALL_RADIUS, dx / BALL_RADIUS, 0.))
            rotations = increment[None, :, :] @ rotations

        center = (0., 0.)
        append("start_hold", START_HOLD, center, center, -1, 0)
        for index, target in enumerate(TARGETS):
            point = target["xy"]
            travel_time = float(np.linalg.norm(point)) * 1.875 / PEAK_SPEED
            append("outbound", travel_time, center, point, index, index)
            append("target_hold", TARGET_DWELL, point, point, index, index + 1)
            append("return", travel_time, point, center, index, index + 1)
            append("center_hold", CENTER_DWELL, center, center, index, index + 1)
        append("finish_hold", FINISH_HOLD, center, center, -1, len(TARGETS))
        self._starts = [segment["start_time"] for segment in self.segments]
        self.duration = self.segments[-1]["end_time"]

    def sample(self, time_s):
        """Return pose, velocity, ball rolling state, and display metadata.

        ``time`` is clamped to [0, duration]; ``progress`` is local phase
        progress, while ``route_progress`` spans the full replay. Target index
        is 0..7 for a center/target/center visit and -1 for the opening/closing
        hold. ``completed_targets`` counts arrivals (0..8), not departures.
        """
        time_s = float(time_s)
        if not math.isfinite(time_s):
            raise ValueError("Sample time must be finite")
        time_s = min(max(time_s, 0.0), self.duration)
        index = min(bisect_right(self._starts, time_s) - 1, len(self.segments) - 1)
        segment = self.segments[index]
        progress = min(max((time_s - segment["start_time"]) / segment["duration"], 0.), 1.)
        # Quintic smoothstep and its exact time derivative.
        ease = progress**3 * (10. + progress * (-15. + 6. * progress))
        ease_rate = 30. * progress**2 * (1. - progress)**2 / segment["duration"]
        start = np.asarray(segment["start_xy"])
        delta = np.asarray(segment["end_xy"]) - start
        displacement = ease * delta
        position = start + displacement
        velocity = ease_rate * delta
        dx, dy = displacement
        increment = rotation_from_vector((-dy / BALL_RADIUS, dx / BALL_RADIUS, 0.))
        rotations = increment[None, :, :] @ self._rotation_starts[index]
        return {
            "time": time_s,
            "x": float(position[0]), "y": float(position[1]), "yaw": 0.,
            "vx": float(velocity[0]), "vy": float(velocity[1]), "yaw_rate": 0.,
            "phase": segment["phase"],
            "target_index": segment["target_index"],
            "target_name": segment["target_name"],
            "progress": progress,
            "route_progress": time_s / self.duration,
            "completed_targets": segment["completed_targets"],
            "ball_omega_world": inverse_kinematics(*velocity),
            "ball_rotations": rotations,
        }

    def summary(self):
        """JSON-serializable description suitable for a replay manifest."""
        return {
            "model": "Ideal kinematic four-active-ball visualization",
            "duration_s": self.duration,
            "box_side_m": 2.0 * BOX_HALF_EXTENT,
            "peak_speed_m_s": PEAK_SPEED,
            "ball_radius_m": BALL_RADIUS,
            "ball_offsets_m": BALL_OFFSETS.tolist(),
            "fixed_yaw_rad": 0.,
            "targets": TARGETS,
            "segments": self.segments,
            "limitations": [
                "Prescribed no-slip poses; no contact, traction, or motor dynamics.",
                "Ideal spherical drives; internal actuator design is not modeled.",
            ],
        }
