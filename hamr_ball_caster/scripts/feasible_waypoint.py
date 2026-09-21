#!/usr/bin/env python3
"""Rest-to-rest waypoint planning for the offset differential-drive simulator.

Each original straight segment uses a quintic time law. Position, velocity,
and acceleration remain continuous at every waypoint, where the vehicle stops.
The original path is preserved exactly; its original constant-speed schedule
is intentionally replaced. This module has no ROS or numerical dependencies.

The wheel bounds are conservative nominal kinematic bounds, not a claim about
motor torque, contact friction, or tracking feedback. Keep a margin between
these planning bounds and the controller's actual command limits.
"""

import bisect
import math


class FeasibleWaypointReference:
    """Plan exact straight segments with continuous velocity and acceleration.

    ``speed`` is a peak translation speed, not an average route speed. Wheel
    planning limits apply to the nominal reference for every chassis heading.
    Consecutive duplicate waypoints are rejected explicitly; silently removing
    them would change waypoint indices and make arrival reports ambiguous.
    """

    def __init__(
        self, points, speed, *, radius, half_track, offset,
        max_translational_acceleration, planning_wheel_rate,
        planning_wheel_acceleration,
    ):
        parameters = {
            "speed": speed,
            "radius": radius,
            "half_track": half_track,
            "offset": offset,
            "max_translational_acceleration": max_translational_acceleration,
            "planning_wheel_rate": planning_wheel_rate,
            "planning_wheel_acceleration": planning_wheel_acceleration,
        }
        for name, value in parameters.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self.points = [tuple(map(float, point)) for point in points]
        if len(self.points) < 2 or any(len(point) != 2 for point in self.points):
            raise ValueError("Need at least two XY waypoints")
        if not all(math.isfinite(value) for point in self.points for value in point):
            raise ValueError("Waypoints must be finite")

        # A wheel is (v/r) * [cos(delta) +/- (a/b) sin(delta)], hence |w| <= K*v.
        # On a straight reference, |theta_dot| <= v/b. Differentiation then gives
        # |w_dot| <= K*(|v_dot| + v**2/b), uniformly in chassis heading.
        gain = math.hypot(1., half_track / offset) / radius
        self.limits = {
            "peak_translation_speed_m_s": float(speed),
            "translation_acceleration_m_s2": float(max_translational_acceleration),
            "planning_wheel_rate_rad_s": float(planning_wheel_rate),
            "planning_wheel_acceleration_rad_s2": float(planning_wheel_acceleration),
            "wheel_speed_gain_rad_m": gain,
            "radius_m": float(radius),
            "half_track_m": float(half_track),
            "offset_m": float(offset),
        }
        self.starts = [0.]
        self.segments = []
        self.length = 0.
        for index, (first, second) in enumerate(zip(self.points[:-1], self.points[1:])):
            dx, dy = second[0] - first[0], second[1] - first[1]
            length = math.hypot(dx, dy)
            if length == 0:
                raise ValueError(f"Consecutive duplicate waypoints at indices {index} and {index+1}")
            if not math.isfinite(length):
                raise ValueError("Waypoint separation must be finite")

            # For s(u)=10u^3-15u^4+6u^5: max(s')=15/8 and max(|s''|)=10/sqrt(3).
            velocity_numerator = (15. / 8.) * length
            acceleration_numerator = (10. / math.sqrt(3.)) * length
            candidates = {
                "translation_speed": velocity_numerator / speed,
                "translation_acceleration": math.sqrt(
                    acceleration_numerator / max_translational_acceleration),
                "wheel_speed": gain * velocity_numerator / planning_wheel_rate,
                "wheel_acceleration": math.sqrt(gain * (
                    acceleration_numerator + velocity_numerator**2 / offset
                ) / planning_wheel_acceleration),
            }
            duration = max(candidates.values())
            if not math.isfinite(duration) or duration <= 0:
                raise ValueError("Waypoint timing is outside the supported finite range")
            peak_speed = velocity_numerator / duration
            peak_acceleration = acceleration_numerator / duration**2
            self.segments.append({
                "index": index,
                "length_m": length,
                "duration_s": duration,
                "direction": [dx / length, dy / length],
                "peak_speed_m_s": peak_speed,
                "peak_acceleration_m_s2": peak_acceleration,
                "wheel_speed_bound_rad_s": gain * peak_speed,
                "wheel_acceleration_bound_rad_s2": gain * (
                    peak_acceleration + peak_speed**2 / offset),
                "duration_constraints_s": candidates,
            })
            self.length += length
            self.starts.append(self.starts[-1] + duration)
            if not math.isfinite(self.length) or not math.isfinite(self.starts[-1]):
                raise ValueError("Total route length and duration must be finite")
        self.duration = self.starts[-1]

    def sample(self, time_s):
        """Return ``(x, y, vx, vy, segment_index)`` in metres and seconds."""
        x, y, vx, vy, _ax, _ay, index = self.sample_with_acceleration(time_s)
        return x, y, vx, vy, index

    def sample_with_acceleration(self, time_s):
        """Return ``(x, y, vx, vy, ax, ay, segment_index)``; hold outside route."""
        if not math.isfinite(time_s):
            raise ValueError("Sample time must be finite")
        if time_s <= 0:
            return (*self.points[0], 0., 0., 0., 0., 0)
        if time_s >= self.duration:
            return (*self.points[-1], 0., 0., 0., 0., len(self.segments) - 1)
        index = bisect.bisect_right(self.starts, time_s) - 1
        segment = self.segments[index]
        duration = segment["duration_s"]
        u = (time_s - self.starts[index]) / duration
        # Factorized derivatives give exact zeros at the segment endpoints.
        progress = u**3 * (10. + u * (-15. + 6. * u))
        progress_rate = 30. * u**2 * (1. - u)**2 / duration
        progress_acceleration = 60. * u * (1. - u) * (1. - 2. * u) / duration**2
        x0, y0 = self.points[index]
        x1, y1 = self.points[index + 1]
        dx, dy = x1 - x0, y1 - y0
        return (
            x0 + dx * progress, y0 + dy * progress,
            dx * progress_rate, dy * progress_rate,
            dx * progress_acceleration, dy * progress_acceleration,
            index,
        )
