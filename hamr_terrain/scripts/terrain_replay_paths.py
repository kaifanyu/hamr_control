#!/usr/bin/env python3
"""Map-scale references and visual-only recording worlds for terrain replays."""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from generate_terrains import look_at, triangle_height
from terrain_simulation_check import prepare_world

SHAPES = ("straight", "circle", "triangle", "square", "figure_eight")
STRESS_SHAPES = ("stress_reversal", "stress_tight_figure_eight",
                 "stress_lateral", "stress_zigzag")
ONE_WAY_SHAPES = SHAPES + ("weave",) + STRESS_SHAPES + ("map_route",)
SUPPORTED_SHAPES = ONE_WAY_SHAPES + tuple(s+"_out_back" for s in ONE_WAY_SHAPES)

# Opt-in references; the original campaign's SHAPES and timing stay unchanged.
STRESS_DESCRIPTIONS = {
    "stress_reversal": "Forward, reverse, then forward along one line; two 180-degree travel-direction changes challenge passive caster realignment. Chassis heading is a controller outcome.",
    "stress_tight_figure_eight": "Two compact smooth lobes repeatedly change curvature sign while maintaining forward reference motion.",
    "stress_lateral": "North-east-north translation with two 90-degree travel-direction changes after bounded braking.",
    "stress_zigzag": "Three alternating diagonal legs create two opposing travel-direction changes after bounded braking.",
}


def _tight_eight(parameter):
    """Analytic centerline and tangent, independent of the sampled arc table."""
    return (np.stack((.65*np.sin(2*parameter), np.sin(parameter)), axis=-1),
            np.stack((1.3*np.cos(2*parameter), np.cos(parameter)), axis=-1))


def route_points(shape):
    if shape == "stress_reversal":
        return np.array([[0., -1.1], [0., 1.1], [0., -1.1], [0., 1.1]])
    if shape == "stress_lateral":
        return np.array([[0., -1.5], [0., 0.], [1.2, 0.], [1.2, 1.5]])
    if shape == "stress_zigzag":
        return np.array([[-.6, -1.5], [.6, -.5], [-.6, .5], [.6, 1.5]])
    if shape == "stress_tight_figure_eight":
        return _tight_eight(np.linspace(-math.pi/2, 3*math.pi/2, 1441))[0]
    if shape == "weave":
        t = np.linspace(0., 1., 501)
        return np.column_stack((2.1*np.sin(2*np.pi*t), -3.2+6.4*t))
    if shape == "straight":
        return np.array([[0., -3.2], [0., 3.2]])
    if shape == "triangle":
        return np.array([[-2.2, -3.0], [0., 3.1], [2.2, -3.0], [-2.2, -3.0]])
    if shape == "square":
        return np.array([[-2.2, -2.2], [2.2, -2.2], [2.2, 2.2], [-2.2, 2.2], [-2.2, -2.2]])
    if shape == "circle":
        t = np.linspace(-math.pi/2, 3*math.pi/2, 361)
        return np.column_stack((2.2*np.cos(t), 2.2*np.sin(t)))
    if shape == "figure_eight":
        t = np.linspace(-math.pi/2, 3*math.pi/2, 721)
        return np.column_stack((2.2*np.sin(2*t), 3.1*np.sin(t)))
    raise ValueError(f"Unknown shape: {shape}")


def reference_for_terrain(shape, speed, metadata):
    """Use explicit per-map routes only when selected; retain all old paths."""
    if shape.removesuffix("_out_back") == "map_route":
        return Reference(shape, speed, map_route=metadata.get("replay_route"),
                         bounds=metadata.get("bounds_m"))
    return Reference(shape, speed)


def validated_map_route(route, bounds):
    """Check every polyline segment has the requested rectangular clearance."""
    if not isinstance(route, dict) or not isinstance(bounds, dict):
        raise ValueError("map_route requires replay_route and bounds_m metadata")
    points = np.asarray(route.get("points_m", []), dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError("replay_route.points_m must contain at least two finite XY points")
    if np.any(np.linalg.norm(np.diff(points, axis=0), axis=1) <= 1e-9):
        raise ValueError("replay_route cannot contain adjacent duplicate points")
    clearance = float(route.get("clearance_m", .65))
    if not math.isfinite(clearance) or clearance < 0:
        raise ValueError("replay_route.clearance_m must be finite and nonnegative")
    for index, axis in enumerate(("x", "y")):
        limits = np.asarray(bounds.get(axis, []), dtype=float)
        if limits.shape != (2,) or not np.isfinite(limits).all() or limits[1] <= limits[0]:
            raise ValueError("bounds_m must provide increasing finite x and y bounds")
        if np.any(points[:, index] < limits[0]+clearance-1e-9) or np.any(points[:, index] > limits[1]-clearance+1e-9):
            raise ValueError(f"replay_route leaves the {axis} bounds with {clearance:g} m clearance")
    stops = route.get("stop_at_waypoints", True)
    if not isinstance(stops, bool):
        raise ValueError("replay_route.stop_at_waypoints must be a boolean")
    if "initial_yaw_rad" in route and not math.isfinite(float(route["initial_yaw_rad"])):
        raise ValueError("replay_route.initial_yaw_rad must be finite")
    return points, clearance, stops


class Reference:
    """Constant arc-length speed, with a 1 s acceleration/deceleration ramp.

    Original polygon corners remain corners. Opt-in stress polygons instead
    stop at each exact corner with bounded acceleration and a recorded dwell;
    their velocity never jumps instantaneously to the new travel direction.
    """
    def __init__(self, shape, speed=.25, *, map_route=None, bounds=None):
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("Speed must be finite and positive")
        self.shape, self.speed = shape, speed
        self.base_reference = None
        if shape.endswith("_out_back"):
            self.base_reference = base = Reference(shape.removesuffix("_out_back"), speed,
                                                  map_route=map_route, bounds=bounds)
            self.turnaround_dwell = .75
            self.return_start = base.duration+self.turnaround_dwell
            self.duration = 2*base.duration+self.turnaround_dwell
            self.length = 2*base.length
            self.points = np.concatenate((base.points, base.points[-2::-1]))
            self.ramp, self.yaw = base.ramp, base.yaw
            self.stop_schedule = []
            return
        if shape == "map_route":
            self.points, self.clearance, self.map_stops = validated_map_route(map_route, bounds)
            self.map_route, self.bounds = dict(map_route), dict(bounds)
        else:
            self.points = route_points(shape)
        self.delta = np.diff(self.points, axis=0)
        self.lengths = np.linalg.norm(self.delta, axis=1)
        if shape == "stress_tight_figure_eight":
            self.curve_parameters = np.linspace(-math.pi/2, 3*math.pi/2, len(self.points))
            self.curve_speeds = np.linalg.norm(_tight_eight(self.curve_parameters)[1], axis=1)
            middle = (self.curve_parameters[:-1]+self.curve_parameters[1:])/2
            # Simpson integration gives true curve length instead of polygonal
            # chord length, retaining a compact, deterministic lookup table.
            self.lengths = np.diff(self.curve_parameters)/6*(
                self.curve_speeds[:-1]+self.curve_speeds[1:]
                +4*np.linalg.norm(_tight_eight(middle)[1], axis=1))
        self.starts = np.r_[0., np.cumsum(self.lengths)]
        self.length = float(self.starts[-1])
        self.ramp = min(1., self.length/speed)
        self.duration = self.length/speed + self.ramp
        self.yaw = math.atan2(*self.delta[0][::-1])
        if shape == "map_route":
            self.yaw = float(map_route.get("initial_yaw_rad", self.yaw))
        self.stop_schedule = []
        if ((shape in STRESS_SHAPES and shape != "stress_tight_figure_eight")
                or (shape == "map_route" and self.map_stops)):
            dwell = .35 if shape == "stress_reversal" else .25
            elapsed = 0.
            for index, length in enumerate(self.lengths):
                ramp = min(1., length/speed)
                motion_duration = length/speed+ramp
                pause = dwell if index < len(self.lengths)-1 else 0.
                self.stop_schedule.append({
                    "segment": index, "start_time_s": elapsed,
                    "motion_end_time_s": elapsed+motion_duration,
                    "dwell_end_time_s": elapsed+motion_duration+pause,
                    "ramp_s": ramp, "dwell_s": pause,
                    "length_m": float(length),
                    "travel_yaw_rad": math.atan2(*self.delta[index][::-1]),
                })
                elapsed += motion_duration+pause
            self.duration = elapsed
        if shape == "stress_tight_figure_eight":
            tangent = _tight_eight(self.curve_parameters[0])[1]
            self.yaw = math.atan2(*tangent[::-1])

    def _sample_stopping_path(self, elapsed):
        t = max(0., min(float(elapsed), self.duration))
        index = max(i for i, leg in enumerate(self.stop_schedule)
                    if leg["start_time_s"] <= t)
        leg = self.stop_schedule[index]
        local = t-leg["start_time_s"]
        duration = leg["motion_end_time_s"]-leg["start_time_s"]
        if local >= duration:
            return self.points[index+1].copy(), np.zeros(2), index
        ramp = leg["ramp_s"]
        acceleration = self.speed/ramp
        if local < ramp:
            distance, velocity = .5*acceleration*local**2, acceleration*local
        elif local > duration-ramp:
            remaining = duration-local
            distance = self.lengths[index]-.5*acceleration*remaining**2
            velocity = acceleration*remaining
        else:
            distance, velocity = self.speed*(local-.5*ramp), self.speed
        direction = self.delta[index]/self.lengths[index]
        return self.points[index]+direction*distance, direction*velocity, index

    def sample(self, elapsed):
        if self.base_reference is not None:
            base = self.base_reference
            if elapsed < base.duration:
                return base.sample(elapsed)
            if elapsed <= self.return_start:
                return base.points[-1].copy(), np.zeros(2), len(base.points)-1
            backwards_time = base.duration-min(elapsed-self.return_start, base.duration)
            xy, velocity, index = base.sample(backwards_time)
            return xy, -velocity, 2*len(base.points)-2-index
        if self.stop_schedule:
            return self._sample_stopping_path(elapsed)
        t = max(0., min(float(elapsed), self.duration))
        a = self.speed/self.ramp
        if t < self.ramp:
            distance, velocity = .5*a*t*t, a*t
        elif t > self.duration-self.ramp:
            remaining = self.duration-t
            distance, velocity = self.length-.5*a*remaining**2, a*remaining
        else:
            distance, velocity = self.speed*(t-.5*self.ramp), self.speed
        index = min(len(self.lengths)-1, int(np.searchsorted(self.starts, distance, side="right")-1))
        if self.shape == "stress_tight_figure_eight":
            # Hermite inversion of the arc table preserves tangent continuity
            # and avoids the speed ripple of linear parameter interpolation.
            length = self.lengths[index]
            u = (distance-self.starts[index])/length
            first, last = self.curve_parameters[index:index+2]
            parameter = ((2*u**3-3*u**2+1)*first+(-2*u**3+3*u**2)*last
                         +(u**3-2*u**2+u)*length/self.curve_speeds[index]
                         +(u**3-u**2)*length/self.curve_speeds[index+1])
            xy, tangent = _tight_eight(parameter)
            return xy, tangent/np.linalg.norm(tangent)*velocity, index
        direction = self.delta[index]/self.lengths[index]
        xy = self.points[index] + direction*(distance-self.starts[index])
        return xy, direction*velocity, index

    def as_dict(self):
        if self.base_reference is not None:
            base = self.base_reference
            result = base.as_dict()
            result.update(shape=self.shape, points_m=self.points.tolist(),
                          route_length_m=self.length, reference_duration_s=self.duration,
                          scope="Original reference followed by its exact time-reversed reference, with a stopped turnaround; chassis yaw remains a controller outcome.")
            result["out_and_back"] = {"base_shape": base.shape,
                "outbound_end_s": base.duration, "turnaround_dwell_s": self.turnaround_dwell,
                "return_start_s": self.return_start, "return_end_s": self.duration,
                "return_reference": "p_return(t)=p_outbound(T-t); v_return(t)=-v_outbound(T-t)",
                "chassis_heading_prescribed": False,
                "reverse_body_motion_required": False}
            if "stress_reference" in result:
                result["stress_reference"]["description"] += " Then retrace the entire reference backwards to the original start."
                result["stress_reference"]["stop_schedule_scope"] = "Outbound schedule; return is exact time reversal."
            return result
        result = {"shape": self.shape, "points_m": self.points.tolist(),
                "route_length_m": self.length, "reference_duration_s": self.duration,
                "speed_m_s": self.speed, "ramp_s": self.ramp,
                "initial_yaw_rad": self.yaw,
                "scope": "Map-scale path with vehicle clearance; a circle/square is width-limited, not a full surface-coverage sweep."}
        if self.shape == "map_route":
            result["scope"] = "Explicit metric per-map reference; the full terrain is retained. Chassis heading and traversability are physical outcomes."
            result["map_route"] = {
                "description": self.map_route.get("description", "Per-map route"),
                "bounds_m": self.bounds, "clearance_m": self.clearance,
                "stop_at_waypoints": self.map_stops, "stop_schedule": self.stop_schedule,
            }
        if self.shape in STRESS_SHAPES:
            minimum_edge_distance = min(3.-np.max(abs(self.points[:, 0])),
                                        4.-np.max(abs(self.points[:, 1])))
            result["scope"] = "Opt-in caster stress reference on an obstacle-free 6 by 8 m patch; terrain, controller limits and actual tracking can reduce execution feasibility."
            result["stress_reference"] = {
                "description": STRESS_DESCRIPTIONS[self.shape],
                "time_parameterization": "Rest-to-rest trapezoids at exact corners" if self.stop_schedule else "Smooth analytic curve with interpolated arc-length timing",
                "stop_schedule": self.stop_schedule,
                "assumed_vehicle_clearance_radius_m": .65,
                "minimum_nominal_footprint_edge_clearance_m": float(minimum_edge_distance-.65),
                "caster_joints_actuated": False,
                "chassis_heading_prescribed": False,
            }
        return result

    def leg_at(self, elapsed):
        if elapsed < 0:
            return "settling"
        if elapsed >= self.duration:
            return "finished"
        if self.base_reference is None:
            return "one_way"
        if elapsed < self.base_reference.duration:
            return "outbound"
        return "turnaround" if elapsed < self.return_start else "return"


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def replay_camera_configuration(metadata):
    """Describe the camera frame explicitly; fixed-world defaults stay intact."""
    supplied = (metadata or {}).get("replay_camera")
    if supplied is not None and not isinstance(supplied, dict):
        raise ValueError("replay_camera must be a mapping")
    camera = dict(supplied or {})
    mode = camera.setdefault("mode", "fixed")
    if mode not in ("fixed", "vehicle_follow"):
        raise ValueError("replay_camera.mode must be fixed or vehicle_follow")
    if mode == "vehicle_follow":
        camera.setdefault("local_eye", [-3., -4., 3.5])
        camera.setdefault("local_target", [0., 0., .25])
        camera.setdefault("far", 100.)
        camera["parent_link"] = "base_link"
        camera["scope"] = "Vehicle-mounted overview camera; view rotates and tilts with the chassis. No vehicle pose is prescribed."
    return camera


def augment_robot_cameras(description, *, metadata=None, fps=15):
    """Add a massless attached sensor and align optional closeup capture rates."""
    if not isinstance(fps, int) or not 1 <= fps <= 60:
        raise ValueError("Camera fps must be an integer in 1..60")
    robot = ET.fromstring(description)
    if robot.tag != "robot":
        raise ValueError("Camera adaptation requires an expanded URDF robot")
    camera = replay_camera_configuration(metadata)
    overview_topic = "/hamr/terrain/overview/image"
    closeup_topics = {f"/hamr/{caster}_caster/closeup/image" for caster in ("ball", "swivel")}
    for parent in list(robot.iter()):
        for sensor in list(parent.findall("sensor[@type='camera']")):
            topic = sensor.findtext("topic")
            if camera["mode"] == "vehicle_follow" and topic == overview_topic:
                parent.remove(sensor)
            elif topic in closeup_topics:
                rate = sensor.find("update_rate")
                if rate is None:
                    rate = ET.SubElement(sensor, "update_rate")
                rate.text = str(fps)
    if camera["mode"] == "vehicle_follow":
        if robot.find("link[@name='base_link']") is None:
            raise ValueError("Vehicle-follow camera requires the robot base_link")
        eye = np.asarray(camera["local_eye"], dtype=float)
        target = np.asarray(camera["local_target"], dtype=float)
        if (eye.shape != (3,) or target.shape != (3,) or not np.isfinite(eye).all()
                or not np.isfinite(target).all() or np.linalg.norm(target-eye) < 1e-9):
            raise ValueError("Follow-camera local eye and target must be distinct finite XYZ points")
        far = float(camera["far"])
        if not math.isfinite(far) or far <= .03:
            raise ValueError("Follow-camera far clip must be finite and greater than .03 m")
        gazebo = robot.find("gazebo[@reference='base_link']")
        if gazebo is None:
            gazebo = ET.SubElement(robot, "gazebo", reference="base_link")
        sensor = ET.SubElement(gazebo, "sensor", name="terrain_vehicle_follow", type="camera")
        for tag, value in (("always_on", "true"), ("update_rate", str(fps)),
                           ("visualize", "false"), ("topic", overview_topic),
                           ("pose", " ".join(map(str, look_at(eye, target))))):
            ET.SubElement(sensor, tag).text = value
        image_camera = ET.SubElement(sensor, "camera")
        ET.SubElement(image_camera, "horizontal_fov").text = "1.0"
        image = ET.SubElement(image_camera, "image")
        for tag, value in (("width", "1280"), ("height", "960"), ("format", "R8G8B8")):
            ET.SubElement(image, tag).text = value
        clip = ET.SubElement(image_camera, "clip")
        ET.SubElement(clip, "near").text = ".03"
        ET.SubElement(clip, "far").text = str(far)
    return ET.tostring(robot, encoding="unicode")


def prepare_replay_world(source, output, grid, reference, fps=15, contact_logging=False, *, metadata=None):
    """Retain physics; select one recording camera and add collision-free route."""
    world_name = prepare_world(source, output, None)
    tree = ET.parse(output)
    world = tree.getroot().find("world")
    if not contact_logging:
        # Contact telemetry on this Gazebo backend emits at every 1 ms step.
        # Removing the observer leaves all physical collision elements intact.
        terrain_link = world.find("model[@name='terrain']/link")
        for contact in terrain_link.findall("sensor[@type='contact']"):
            terrain_link.remove(contact)
    detail = world.find("model[@name='detail_camera']")
    if detail is not None:
        world.remove(detail)
    camera = replay_camera_configuration(metadata)
    overview_model = world.find("model[@name='overview_camera']")
    if camera["mode"] == "vehicle_follow":
        if overview_model is not None:
            world.remove(overview_model)
        if any(sensor.findtext("topic") == "/hamr/terrain/overview/image"
               for sensor in world.findall(".//sensor[@type='camera']")):
            raise ValueError("World has another camera publishing the vehicle-follow overview topic")
    else:
        sensor = overview_model.find("link/sensor")
        sensor.find("update_rate").text = str(fps)
        sensor.find("camera/image/width").text = "1280"
        sensor.find("camera/image/height").text = "960"
        if "eye" in camera or "target" in camera:
            eye = np.asarray(camera.get("eye", []), dtype=float)
            target = np.asarray(camera.get("target", []), dtype=float)
            if (eye.shape != (3,) or target.shape != (3,) or not np.isfinite(eye).all()
                    or not np.isfinite(target).all() or np.linalg.norm(target-eye) < 1e-9):
                raise ValueError("replay_camera eye and target must be distinct finite XYZ points")
            model = overview_model
            pose = model.find("pose")
            if pose is None:
                pose = ET.SubElement(model, "pose")
            pose.text = " ".join(map(str, look_at(eye, target)))
        if "far" in camera:
            far = float(camera["far"])
            near = float(sensor.find("camera/clip/near").text)
            if not math.isfinite(far) or far <= near:
                raise ValueError("replay_camera.far must be finite and greater than the near clip")
            sensor.find("camera/clip/far").text = str(far)
    # Thin bright route segments follow the exact terrain triangle surface.
    # Visual-only objects cannot support a wheel or change friction.
    route = ET.SubElement(world, "model", name="planned_route_visual")
    ET.SubElement(route, "static").text = "true"
    link = ET.SubElement(route, "link", name="route")
    visual_reference = reference.base_reference or reference
    count = max(2, int(math.ceil(visual_reference.length/.12)))
    locations = [visual_reference.sample(t)[0] for t in np.linspace(0., visual_reference.duration, count+1)]
    for i, (a, b) in enumerate(zip(locations, locations[1:])):
        za = triangle_height(grid['x'], grid['y'], grid['z'], *a)+.012
        zb = triangle_height(grid['x'], grid['y'], grid['z'], *b)+.012
        delta = np.r_[b-a, zb-za]
        length = float(np.linalg.norm(delta))
        if length < 1e-8:
            continue
        center = np.r_[(a+b)/2, (za+zb)/2]
        yaw = math.atan2(delta[1], delta[0])
        pitch = -math.atan2(delta[2], math.hypot(*delta[:2]))
        visual = ET.SubElement(link, "visual", name=f"segment_{i}")
        ET.SubElement(visual, "pose").text = ' '.join(map(str, [*center, 0., pitch, yaw]))
        ET.SubElement(ET.SubElement(ET.SubElement(visual, "geometry"), "box"), "size").text = f"{length} .025 .004"
        material = ET.SubElement(visual, "material")
        for key in ("ambient", "diffuse", "emissive"):
            ET.SubElement(material, key).text = ".05 .8 .35 1"
    tree.write(output, encoding="unicode", xml_declaration=True)
    return world_name
