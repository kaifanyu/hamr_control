"""Measured, rear-wheel-only startup recovery for passive swivel or ball casters.

The axle can rotate in place; the tracked base point ahead of it cannot. Its
physical excursion is retained. The subsequent route resumes from that measured
point and still visits the original first destination and all later waypoints.
"""
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class StartupAlignment:
    def __init__(self, config, points, radius, half_track, offset):
        self.config = dict(config)
        self.caster_model = config.get("caster_model", "swivel")
        if self.caster_model not in ("swivel", "ball"):
            raise ValueError("startup_alignment.caster_model must be swivel or ball")
        self.required_joint_names = (("left_caster_swivel_joint", "right_caster_swivel_joint")
                                     if self.caster_model == "swivel" else ())
        self.points = [tuple(map(float, point)) for point in points]
        self.radius, self.half_track, self.offset = radius, half_track, offset
        dx = self.points[1][0]-self.points[0][0]
        dy = self.points[1][1]-self.points[0][1]
        self.first_leg_length = math.hypot(dx, dy)
        if self.first_leg_length <= 0:
            raise ValueError("Startup alignment requires a nonzero first leg")
        self.target_yaw = math.atan2(dy, dx)
        self.direction = (dx/self.first_leg_length, dy/self.first_leg_length)
        defaults = {"timeout_s": 50., "max_yaw_rate_rad_s": .60,
                    "heading_gain_s_inv": 2., "heading_tolerance_rad": math.radians(3.),
                    "caster_tolerance_rad": math.radians(15.), "alignment_dwell_s": .30,
                    "lead_in_speed_m_s": .08, "min_lead_in_distance_m": .16,
                    "max_lead_in_distance_m": .65, "min_first_leg_remaining_m": .12,
                    "settle_speed_tolerance_m_s": .01}
        for name, default in defaults.items():
            value = float(config.get(name, default))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"startup_alignment.{name} must be finite and positive")
            setattr(self, name, value)
        if self.min_lead_in_distance_m >= self.max_lead_in_distance_m:
            raise ValueError("Startup minimum lead-in must be below maximum lead-in")
        if self.first_leg_length <= self.min_first_leg_remaining_m + self.min_lead_in_distance_m:
            raise ValueError("First leg is too short for startup alignment and its remaining-distance guard")
        self.zero_offset = float(config.get("caster_zero_offset_rad", 0.))
        if not math.isfinite(self.zero_offset):
            raise ValueError("caster_zero_offset_rad must be finite")
        self.phase = "startup_heading_recovery"
        self.started = None
        self.previous_yaw = None
        self.yaw_unwrapped = None
        self.initial_yaw = None
        self.target_unwrapped = None
        self.initial_position = None
        self.axle_center = None
        self.lead_start = None
        self.lead_started = None
        self.good_since = None
        self.done = False
        self.failure = None
        self.events = []
        self.last = None

    def _transition(self, phase, now):
        self.events.append({"phase": self.phase, "end_time_s": now-self.started})
        self.phase = phase
        self.good_since = None

    def _dwell(self, good, now):
        if not good:
            self.good_since = None
        elif self.good_since is None:
            self.good_since = now
        return self.good_since is not None and now-self.good_since >= self.alignment_dwell_s

    def update(self, now, position, yaw, joints, speed):
        """Return desired rear-wheel rates; caster angles are observations only."""
        if self.started is None:
            self.started = now
            self.initial_yaw = self.yaw_unwrapped = yaw
            self.previous_yaw = yaw
            self.initial_position = tuple(position)
            delta = wrap(self.target_yaw-yaw)
            # Exact 180 degree ambiguity always chooses positive/CCW rotation.
            if abs(abs(delta)-math.pi) < .01:
                # Retain the requested angle, choosing its positive equivalent
                # inside a small band so settled-pose noise cannot flip the tie.
                delta = delta if delta >= 0 else delta+2*math.pi
            self.target_unwrapped = yaw+delta
            self.axle_center = (position[0]-self.offset*math.cos(yaw),
                                position[1]-self.offset*math.sin(yaw))
        else:
            self.yaw_unwrapped += wrap(yaw-self.previous_yaw)
            self.previous_yaw = yaw
        caster_errors = ({side: wrap(joints[f"{side}_caster_swivel_joint"] + self.zero_offset)
                          for side in ("left", "right")}
                         if self.caster_model == "swivel" else {})
        yaw_error = self.target_unwrapped-self.yaw_unwrapped
        heading_good = abs(yaw_error) <= self.heading_tolerance_rad
        casters_good = all(abs(value) <= self.caster_tolerance_rad for value in caster_errors.values())
        progress = sum((position[i]-self.points[0][i])*self.direction[i] for i in (0, 1))
        lead_distance = (sum((position[i]-self.lead_start[i])*self.direction[i] for i in (0, 1))
                         if self.lead_start is not None else 0.)
        self.last = {"time_s": now-self.started, "position_m": list(position),
                     "yaw_rad": yaw, "unwrapped_yaw_rad": self.yaw_unwrapped,
                     "heading_error_rad": yaw_error, "caster_heading_errors_rad": caster_errors,
                     "first_leg_progress_m": progress, "lead_in_distance_m": lead_distance,
                     "phase": self.phase}
        if not self.done:
            if now-self.started > self.timeout_s:
                self.failure = "Startup alignment timed out"
            elif progress >= self.first_leg_length-self.min_first_leg_remaining_m:
                self.failure = "Startup alignment would consume the first destination; use a longer first leg"
            elif lead_distance > self.max_lead_in_distance_m:
                self.failure = "Passive casters did not align within maximum forward lead-in distance"
            if self.failure:
                raise RuntimeError(self.failure)
        v = 0.
        omega = max(-self.max_yaw_rate_rad_s, min(self.max_yaw_rate_rad_s,
                    self.heading_gain_s_inv*yaw_error))
        reference = [self.axle_center[0]+self.offset*math.cos(self.target_yaw),
                     self.axle_center[1]+self.offset*math.sin(self.target_yaw)]
        if self.phase == "startup_heading_recovery":
            if self._dwell(heading_good, now):
                self._transition("startup_caster_lead_in", now)
                self.lead_start = tuple(position)
                self.lead_started = now
        if self.phase == "startup_caster_lead_in":
            v = self.lead_in_speed_m_s
            # A tiny bounded steering pulse breaks exact leading-caster symmetry
            # (q=pi), which forward translation alone can preserve indefinitely.
            if max((abs(error) for error in caster_errors.values()), default=0.) > math.radians(150.):
                omega += .08*math.sin(min(math.pi, (now-self.lead_started)*math.pi))
            reference = [self.lead_start[i]+self.min_lead_in_distance_m*self.direction[i]
                         for i in (0, 1)]
            if self._dwell(heading_good and casters_good and lead_distance >= self.min_lead_in_distance_m, now):
                self._transition("startup_alignment_settle", now)
                v = 0.
                self.stop_position = tuple(position)
        if self.phase == "startup_alignment_settle":
            reference = list(self.stop_position)
            if self._dwell(heading_good and casters_good and speed <= self.settle_speed_tolerance_m_s, now):
                self._transition("startup_alignment_complete", now)
                self.done = True
        if self.done:
            v = omega = 0.
        omega = max(-self.max_yaw_rate_rad_s, min(self.max_yaw_rate_rad_s, omega))
        self.last["phase"] = self.phase
        self.last["commanded_forward_speed_m_s"] = v
        self.last["commanded_yaw_rate_rad_s"] = omega
        return ((v-self.half_track*omega)/self.radius,
                (v+self.half_track*omega)/self.radius, reference, dict(self.last))

    def report(self):
        final = self.last or {}
        yaw_change = (self.yaw_unwrapped-self.initial_yaw if self.initial_yaw is not None else None)
        initial_degrees = math.degrees(self.initial_yaw) % 360. if self.initial_yaw is not None else None
        return {"enabled": True, "passed": self.done and self.failure is None,
                "failure": self.failure, "phase": self.phase, "target_heading_rad": self.target_yaw,
                "initial_heading_rad": self.initial_yaw, "final_heading_rad": final.get("yaw_rad"),
                "unwrapped_final_heading_rad": self.yaw_unwrapped,
                "initial_heading_deg": initial_degrees,
                "final_heading_unwrapped_deg": (initial_degrees+math.degrees(yaw_change)
                                                if yaw_change is not None else None),
                "actual_chassis_yaw_change_rad": yaw_change,
                "requested_yaw_change_rad": (self.target_unwrapped-self.initial_yaw
                                            if self.initial_yaw is not None else None),
                "duration_s": final.get("time_s"), "initial_position_m": self.initial_position,
                "executed_start_position_m": final.get("position_m"),
                "preparation_displacement_m": ([final["position_m"][i]-self.initial_position[i] for i in (0, 1)]
                                                if self.initial_position is not None else None),
                "final_caster_heading_errors_rad": final.get("caster_heading_errors_rad"),
                "lead_in_distance_m": final.get("lead_in_distance_m"), "events": self.events,
                "parameters": {**self.config, "caster_zero_offset_rad": self.zero_offset},
                "caster_model": self.caster_model,
                "caster_orientation_gate_applied": self.caster_model == "swivel",
                "caster_orientation_gate_note": (
                    "Measured swivel angles must align before the route starts"
                    if self.caster_model == "swivel" else
                    "Ball casters have no preferred forward swivel orientation; heading recovery, minimum forward lead-in, dwell and settling are retained, with no caster-angle gate"),
                "actuation": "Rear drive wheels only; every front caster joint remains passive",
                "path_policy": "The base moves around the rear axle during rotation. Startup samples preserve that excursion; normal motion replans only the source of the first segment from the measured aligned position. The first requested destination and every later waypoint remain unchanged."}
