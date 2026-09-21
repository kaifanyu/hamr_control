#!/usr/bin/env python3
"""Continuous, bounded-deviation waypoint planning for the offset drive.

Corners use symmetric quintic Bezier control polygons (their symmetry reduces
the position polynomial to degree four). Curvature is zero at both ends. Each
blend runs at constant positive speed; straight sections accelerate/decelerate
using quintic *velocity* ramps. Thus Cartesian velocity and acceleration are
continuous, and Cartesian jerk is finite and bounded, including at joins.

Bounds describe nominal kinematics, with planning headroom left for feedback.
They do not model motor torque, contact friction, or hardware calibration.
"""

import bisect
import math

import numpy as np


_GL_X, _GL_W = np.polynomial.legendre.leggauss(8)
_ARC_BINS = 256
_BOUND_BINS = 1024


def _positive(name, value):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")


def _blend_derivatives(curve, u):
    """Position and three parameter derivatives, with u in [0, 1]."""
    u = np.asarray(u)
    d, direction, change = curve["setback"], curve["incoming"], curve["change"]
    integral = u**3 - .5 * u**4
    beta = 3. * u**2 - 2. * u**3
    position = curve["entry"] + 2. * d * (
        u[..., None] * direction + integral[..., None] * change)
    first = 2. * d * (direction + beta[..., None] * change)
    second = 12. * d * (u * (1.-u))[..., None] * change
    third = 12. * d * (1.-2.*u)[..., None] * change
    return position, first, second, third


def _blend_speed(curve, u):
    beta = 3.*u*u - 2.*u*u*u
    g = 1. - 2.*(1.-curve["cos_turn"])*beta*(1.-beta)
    return 2.*curve["setback"]*np.sqrt(np.maximum(g, 0.))


def _integrate_arc(curve, lower, upper):
    middle, half = (lower+upper)/2., (upper-lower)/2.
    return half * np.sum(_GL_W * _blend_speed(curve, middle[..., None]+half[..., None]*_GL_X), axis=-1)


def _prepare_blend(point, incoming, outgoing, setback, waypoint_index):
    cosine = float(np.dot(incoming, outgoing))
    sine = float(incoming[0]*outgoing[1]-incoming[1]*outgoing[0])
    curve = {
        "kind": "blend", "waypoint_index": waypoint_index,
        "point": point, "incoming": incoming, "outgoing": outgoing,
        "change": outgoing-incoming, "setback": setback,
        "entry": point-setback*incoming, "exit": point+setback*outgoing,
        "cos_turn": cosine, "sin_turn": sine,
    }
    grid = np.linspace(0., 1., _ARC_BINS+1)
    lengths = _integrate_arc(curve, grid[:-1], grid[1:])
    curve["arc_grid"] = np.concatenate(([0.], np.cumsum(lengths)))
    curve["length"] = float(curve["arc_grid"][-1])

    # All factors are bounded on each closed interval, not merely sampled.
    # beta=3u²-2u³ is monotone. g=cos²(phi/2)+4sin²(phi/2)(beta-.5)².
    # These interval maxima enclose the analytic curvature derivative exactly
    # up to floating point rounding (with a small outward safety inflation).
    lo = np.linspace(0., 1., _BOUND_BINS+1)[:-1]
    hi = lo + 1./_BOUND_BINS
    b_lo, b_hi = 3.*lo**2-2.*lo**3, 3.*hi**2-2.*hi**3
    b_near = np.clip(.5, b_lo, b_hi)
    g_min = 1.-2.*(1.-cosine)*b_near*(1.-b_near)
    mid_near = np.clip(.5, lo, hi)
    u_product_max = mid_near*(1.-mid_near)
    one_minus_2u_max = np.maximum(abs(1.-2.*lo), abs(1.-2.*hi))
    one_minus_2b_max = np.maximum(abs(1.-2.*b_lo), abs(1.-2.*b_hi))
    g_prime_max = 12.*(1.-cosine)*u_product_max*one_minus_2b_max
    curvature_interval = 3.*abs(sine)*u_product_max/(setback*g_min**1.5)
    curvature_s_interval = 3.*abs(sine)/(2.*setback**2) * (
        one_minus_2u_max/g_min**2 + 1.5*u_product_max*g_prime_max/g_min**3)
    curve["curvature_bound"] = float(np.max(curvature_interval))*(1.+1e-12)
    curve["curvature_s_bound"] = float(np.max(curvature_s_interval))*(1.+1e-12)
    curve["jerk_geometry_bound"] = float(np.max(np.hypot(
        curvature_s_interval, curvature_interval**2)))*(1.+1e-12)
    curve["deviation_bound"] = (3./16.)*setback*abs(sine)
    curve["waypoint_miss"] = (3./8.)*setback*math.sqrt(max(0., (1.-cosine)/2.))
    return curve


def _blend_parameter(curve, distance):
    if distance <= 0:
        return 0.
    if distance >= curve["length"]:
        return 1.
    grid = curve["arc_grid"]
    index = int(np.searchsorted(grid, distance, side="right")-1)
    lo, hi = index/_ARC_BINS, (index+1)/_ARC_BINS
    u = lo + (hi-lo)*(distance-grid[index])/(grid[index+1]-grid[index])
    for _ in range(3):
        current = grid[index] + float(_integrate_arc(curve, np.asarray(lo), np.asarray(u)))
        u = max(lo, min(hi, u-(current-distance)/float(_blend_speed(curve, u))))
    return u


def _ramp_duration(first, last, acceleration, jerk):
    change = abs(last-first)
    if change <= 1e-14:
        return 0.
    return max((15./8.)*change/acceleration, math.sqrt((10./math.sqrt(3.))*change/jerk))


def _ramp_state(phase, elapsed):
    duration, first, last = phase["duration"], phase["v0"], phase["v1"]
    u = max(0., min(1., elapsed/duration))
    change = last-first
    if phase["kind"] == "cruise":
        return first*elapsed, first, 0., 0.
    integral = u**4*(2.5 + u*(-3.+u))
    speed_law = u**3*(10.+u*(-15.+6.*u))
    return (
        duration*(first*u+change*integral),
        first+change*speed_law,
        change*30.*u*u*(1.-u)**2/duration,
        change*60.*u*(1.-u)*(1.-2.*u)/duration**2,
    )


class ContinuousWaypointReference:
    """Round corners within a deviation budget and retain positive interior speed.

    ``points`` remain the original waypoints for display and provenance. True
    corner coordinates are deliberately bypassed; ``starts`` records closest
    planned pass times. Collinear waypoints are traversed exactly without stops.
    """

    def __init__(self, points, speed, *, radius, half_track, offset,
                 max_translational_acceleration, planning_wheel_rate,
                 planning_wheel_acceleration, corner_deviation, max_jerk):
        values = locals().copy()
        for name in ("speed", "radius", "half_track", "offset",
                     "max_translational_acceleration", "planning_wheel_rate",
                     "planning_wheel_acceleration", "corner_deviation", "max_jerk"):
            _positive(name, values[name])
        self.points = [tuple(map(float, point)) for point in points]
        if len(self.points) < 2 or any(len(point) != 2 for point in self.points):
            raise ValueError("Need at least two XY waypoints")
        array = np.asarray(self.points, dtype=float)
        if not np.isfinite(array).all():
            raise ValueError("Waypoints must be finite")
        edges = np.diff(array, axis=0)
        lengths = np.linalg.norm(edges, axis=1)
        if not np.isfinite(lengths).all() or np.any(lengths <= 1e-8):
            raise ValueError("Consecutive waypoints must be distinct, with finite separation greater than 1e-8 m")
        directions = edges/lengths[:, None]
        gain = math.hypot(1., half_track/offset)/radius
        # Retain at least half of the wheel-acceleration budget for tangential
        # acceleration even when the chassis initially points across a straight.
        speed_cap = min(speed, planning_wheel_rate/gain,
                        math.sqrt(planning_wheel_acceleration*offset/(2.*gain)))
        straight_acceleration = min(max_translational_acceleration,
                                    planning_wheel_acceleration/gain-speed_cap**2/offset)
        self.limits = {
            "requested_peak_speed_m_s": float(speed),
            "peak_speed_m_s": speed_cap,
            "max_acceleration_m_s2": float(max_translational_acceleration),
            "max_jerk_m_s3": float(max_jerk),
            "planning_wheel_rate_rad_s": float(planning_wheel_rate),
            "planning_wheel_acceleration_rad_s2": float(planning_wheel_acceleration),
            "max_intended_deviation_m": float(corner_deviation),
            "wheel_speed_gain_rad_m": gain,
            "radius_m": float(radius), "half_track_m": float(half_track), "offset_m": float(offset),
        }
        self._curves = {}
        for index in range(1, len(array)-1):
            incoming, outgoing = directions[index-1:index+1]
            cosine = float(np.clip(np.dot(incoming, outgoing), -1., 1.))
            sine = float(incoming[0]*outgoing[1]-incoming[1]*outgoing[0])
            if cosine < -1.+1e-8:
                raise ValueError(f"U-turn at waypoint {index} is unsupported; specify a finite-radius turning path")
            if abs(sine) < 1e-8 and cosine > 0:
                continue
            # Each adjacent blend consumes <=45% of its shared original edge,
            # so blends never overlap and remain inside adjacent segment extents.
            setback = min(corner_deviation/((3./16.)*abs(sine)),
                          .45*lengths[index-1], .45*lengths[index])
            curve = _prepare_blend(array[index], incoming, outgoing, setback, index)
            curvature = curve["curvature_bound"]
            curve["speed_cap"] = min(
                speed_cap,
                math.sqrt(max_translational_acceleration/curvature),
                (max_jerk/curve["jerk_geometry_bound"])**(1./3.),
                math.sqrt(planning_wheel_acceleration/(gain*(curvature+1./offset))),
            )
            self._curves[index] = curve

        anchors = [0, *self._curves, len(array)-1]
        self._lines = []
        for first, last in zip(anchors[:-1], anchors[1:]):
            entry = self._curves[first]["exit"] if first in self._curves else array[first]
            end = self._curves[last]["entry"] if last in self._curves else array[last]
            distance = float(np.linalg.norm(end-entry))
            self._lines.append({"kind": "line", "entry": entry, "exit": end,
                                "direction": (end-entry)/distance, "length": distance,
                                "first_waypoint": first, "last_waypoint": last})
        speeds = [0., *[self._curves[i]["speed_cap"] for i in anchors[1:-1]], 0.]

        def reachable(first, desired, distance):
            if desired <= first:
                return desired
            def required(last):
                return .5*(first+last)*_ramp_duration(first, last, straight_acceleration, max_jerk)
            if required(desired) <= distance:
                return desired
            lo, hi = first, desired
            for _ in range(60):
                middle = (lo+hi)/2.
                if required(middle) <= distance:
                    lo = middle
                else:
                    hi = middle
            return lo

        # Forward/backward reachability reduces corner speed when short straights
        # cannot connect neighboring corner caps under acceleration/jerk bounds.
        for _ in range(4):
            for index, line in enumerate(self._lines):
                speeds[index+1] = reachable(speeds[index], speeds[index+1], line["length"])
            for index in range(len(self._lines)-1, -1, -1):
                speeds[index] = reachable(speeds[index+1], speeds[index], self._lines[index]["length"])
        if any(value <= 0 for value in speeds[1:-1]):
            raise ValueError("Geometry and bounds do not admit positive interior corner speeds")

        self._phases = []
        self._geometry = []
        self.segments = []
        now = 0.
        self.length = 0.

        def append_phase(geometry, kind, first, last, duration, distance, start_distance):
            nonlocal now
            if duration <= 1e-12:
                return
            phase = {"geometry": geometry, "kind": kind, "v0": first, "v1": last,
                     "start": now, "duration": duration, "distance": distance,
                     "start_distance": start_distance}
            geometry.setdefault("phases", []).append(phase)
            self._phases.append(phase)
            now += duration

        for index, line in enumerate(self._lines):
            v0, v1 = speeds[index:index+2]
            def ramp_distance(peak):
                first_t = _ramp_duration(v0, peak, straight_acceleration, max_jerk)
                last_t = _ramp_duration(peak, v1, straight_acceleration, max_jerk)
                return .5*(v0+peak)*first_t+.5*(peak+v1)*last_t
            lo, hi = max(v0, v1), speed_cap
            if ramp_distance(lo) > line["length"]+1e-10:
                raise ValueError("Insufficient straight distance to connect corner speeds")
            if ramp_distance(hi) > line["length"]:
                for _ in range(60):
                    middle = (lo+hi)/2.
                    if ramp_distance(middle) <= line["length"]:
                        lo = middle
                    else:
                        hi = middle
                peak = lo
            else:
                peak = hi
            first_t = _ramp_duration(v0, peak, straight_acceleration, max_jerk)
            last_t = _ramp_duration(peak, v1, straight_acceleration, max_jerk)
            first_d, last_d = .5*(v0+peak)*first_t, .5*(peak+v1)*last_t
            cruise_d = max(0., line["length"]-first_d-last_d)
            line["start_time"] = now
            append_phase(line, "ramp", v0, peak, first_t, first_d, 0.)
            append_phase(line, "cruise", peak, peak, cruise_d/peak, cruise_d, first_d)
            append_phase(line, "ramp", peak, v1, last_t, last_d, first_d+cruise_d)
            line["end_time"] = now
            self._geometry.append(line)
            self.length += line["length"]
            self.segments.append({"kind": "line", "length_m": line["length"],
                                  "start_time_s": line["start_time"], "end_time_s": now,
                                  "duration_s": now-line["start_time"], "peak_speed_m_s": peak})
            if index < len(self._lines)-1:
                curve = self._curves[anchors[index+1]]
                corner_speed = speeds[index+1]
                curve["start_time"] = now
                curve["speed"] = corner_speed
                append_phase(curve, "cruise", corner_speed, corner_speed,
                             curve["length"]/corner_speed, curve["length"], 0.)
                curve["end_time"] = now
                self._geometry.append(curve)
                self.length += curve["length"]
                self.segments.append({"kind": "blend", "waypoint_index": curve["waypoint_index"],
                                      "length_m": curve["length"], "start_time_s": curve["start_time"],
                                      "end_time_s": now, "duration_s": now-curve["start_time"],
                                      "peak_speed_m_s": corner_speed,
                                      "curvature_bound_m_inv": curve["curvature_bound"]})
        self.duration = now
        if not math.isfinite(now) or now <= 0:
            raise ValueError("Planned duration must be finite and positive")
        self._phase_starts = [phase["start"] for phase in self._phases]
        self.starts = [0.]*len(array)
        self.starts[-1] = self.duration
        for index, curve in self._curves.items():
            self.starts[index] = .5*(curve["start_time"]+curve["end_time"])
        for line in self._lines:
            for index in range(line["first_waypoint"]+1, line["last_waypoint"]):
                distance = float(np.dot(array[index]-line["entry"], line["direction"]))
                self.starts[index] = self._line_time_at_distance(line, distance)
        self._metadata = None

    @staticmethod
    def _line_time_at_distance(line, distance):
        for phase in line["phases"]:
            if distance <= phase["start_distance"]+phase["distance"]+1e-12:
                target = max(0., distance-phase["start_distance"])
                lo, hi = 0., phase["duration"]
                for _ in range(60):
                    middle = .5*(lo+hi)
                    if _ramp_state(phase, middle)[0] < target:
                        lo = middle
                    else:
                        hi = middle
                return phase["start"]+.5*(lo+hi)
        return line["end_time"]

    def sample_with_jerk(self, time_s):
        """Return x,y,vx,vy,ax,ay,jx,jy,index; endpoint holds have zero derivatives."""
        if not math.isfinite(time_s):
            raise ValueError("Sample time must be finite")
        if time_s <= 0:
            return (*self.points[0], 0., 0., 0., 0., 0., 0., 0)
        if time_s >= self.duration:
            return (*self.points[-1], 0., 0., 0., 0., 0., 0., len(self.points)-2)
        phase = self._phases[bisect.bisect_right(self._phase_starts, time_s)-1]
        distance, speed, acceleration, jerk = _ramp_state(phase, time_s-phase["start"])
        geometry = phase["geometry"]
        distance += phase["start_distance"]
        if geometry["kind"] == "line":
            tangent = geometry["direction"]
            position = geometry["entry"]+distance*tangent
            curvature = curvature_s = 0.
        else:
            u = _blend_parameter(geometry, distance)
            position, first, second, third = _blend_derivatives(geometry, u)
            norm = float(np.linalg.norm(first))
            tangent = first/norm
            cross = first[0]*second[1]-first[1]*second[0]
            curvature = cross/norm**3
            curvature_s = ((first[0]*third[1]-first[1]*third[0])/norm**4
                           -3.*cross*np.dot(first, second)/norm**6)
        normal = np.array([-tangent[1], tangent[0]])
        velocity = speed*tangent
        acceleration_xy = acceleration*tangent+curvature*speed**2*normal
        jerk_xy = (jerk-curvature**2*speed**3)*tangent+(
            3.*curvature*speed*acceleration+curvature_s*speed**3)*normal
        index = max(0, min(len(self.points)-2, bisect.bisect_right(self.starts, time_s)-1))
        return (*map(float, position), *map(float, velocity), *map(float, acceleration_xy),
                *map(float, jerk_xy), index)

    def sample_with_acceleration(self, time_s):
        x, y, vx, vy, ax, ay, _jx, _jy, index = self.sample_with_jerk(time_s)
        return x, y, vx, vy, ax, ay, index

    def sample(self, time_s):
        x, y, vx, vy, _ax, _ay, index = self.sample_with_acceleration(time_s)
        return x, y, vx, vy, index

    def metadata(self):
        """JSON-friendly dense reference plus analytic/interval bound certificate."""
        if self._metadata is not None:
            return self._metadata
        gain, offset = self.limits["wheel_speed_gain_rad_m"], self.limits["offset_m"]
        acceleration_bounds, jerk_bounds, wheel_acceleration_bounds, speed_bounds = [], [], [], []
        for phase in self._phases:
            geometry = phase["geometry"]
            peak_speed = max(phase["v0"], phase["v1"])
            speed_bounds.append(peak_speed)
            if geometry["kind"] == "blend":
                acceleration_bounds.append(geometry["curvature_bound"]*peak_speed**2)
                jerk_bounds.append(geometry["jerk_geometry_bound"]*peak_speed**3)
                wheel_acceleration_bounds.append(gain*peak_speed**2*(geometry["curvature_bound"]+1./offset))
            else:
                change = abs(phase["v1"]-phase["v0"])
                acceleration = (15./8.)*change/phase["duration"]
                acceleration_bounds.append(acceleration)
                jerk_bounds.append((10./math.sqrt(3.))*change/phase["duration"]**2)
                wheel_acceleration_bounds.append(gain*(acceleration+peak_speed**2/offset))
        times = np.unique(np.concatenate((np.linspace(0., self.duration, math.ceil(self.duration/.01)+1),
                                         self.starts, self._phase_starts, [self.duration])))
        samples = []
        for time_s in times:
            values = self.sample_with_acceleration(float(time_s))
            samples.append({"time_s": float(time_s), "position_m": list(values[:2]),
                            "velocity_m_s": list(values[2:4]), "acceleration_m_s2": list(values[4:6])})
        corners = [{
            "waypoint_index": index,
            "start_time_s": curve["start_time"], "end_time_s": curve["end_time"],
            "closest_pass_time_s": self.starts[index],
            "planned_waypoint_miss_m": curve["waypoint_miss"],
            "max_intended_deviation_m": curve["deviation_bound"],
            "setback_m": curve["setback"], "speed_m_s": curve["speed"],
            "curvature_bound_m_inv": curve["curvature_bound"],
            "curvature_derivative_bound_m_inv2": curve["curvature_s_bound"],
            "entry_m": list(map(float, curve["entry"])), "exit_m": list(map(float, curve["exit"])),
        } for index, curve in self._curves.items()]
        startup_end = self._phases[0]["start"]+self._phases[0]["duration"]
        final_deceleration_start = self._phases[-1]["start"]
        max_dt = float(np.max(np.diff(times)))
        bound = {
            "method": "Analytic quintic velocity extrema, analytic deviation envelope, interval bounds on curvature and arc-length curvature derivative",
            "curvature_interval_count": _BOUND_BINS,
            "max_speed_m_s": max(speed_bounds),
            "max_acceleration_m_s2": max(acceleration_bounds),
            "max_jerk_m_s3": max(jerk_bounds),
            "max_wheel_rate_rad_s": gain*max(speed_bounds),
            "max_wheel_acceleration_rad_s2": max(wheel_acceleration_bounds),
            "max_intended_deviation_m": max((c["max_intended_deviation_m"] for c in corners), default=0.),
            "wheel_acceleration_formula": "K*(abs(v_dot)+v^2*(abs(curvature)+1/offset))",
            "scope": "Nominal reference for any initial chassis heading; feedback, traction and motor dynamics require simulation/hardware validation",
        }
        self._metadata = {
            "schema_version": 1, "profile": "continuous", "duration_s": self.duration,
            "trajectory_duration_s": self.duration, "length_m": self.length,
            "points_m": [list(p) for p in self.points], "waypoint_pass_times_s": self.starts,
            "interior_motion_interval_s": [startup_end, final_deceleration_start],
            "max_acceleration_m_s2": self.limits["max_acceleration_m_s2"],
            "max_sample_interval_s": max_dt, "max_sample_spacing_m": max(speed_bounds)*max_dt,
            "linear_chord_error_bound_m": self.limits["max_acceleration_m_s2"]*max_dt**2/8.,
            "corners": corners, "limits": self.limits, "certification": bound, "samples": samples,
        }
        return self._metadata
