#!/usr/bin/env python3
"""Prepare a Gazebo floor and overview camera from the exact run configuration."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare_world(source, destination, points, step=.001, real_time_factor=1.):
    points = [tuple(float(v) for v in point) for point in points]
    if len(points) < 2 or any(len(point) != 2 for point in points):
        raise ValueError("At least two XY waypoints are required")
    if any(not math.isfinite(v) for point in points for v in point):
        raise ValueError("Waypoints must be finite")
    if not math.isfinite(step) or not 0 < step <= .01:
        raise ValueError("Physics step must be in (0, .01]")
    if not math.isfinite(real_time_factor) or not 0 < real_time_factor <= 1:
        raise ValueError("Real-time factor must be in (0, 1]")
    tree = ET.parse(source)
    world = tree.getroot().find("world")
    world.find("physics/max_step_size").text = str(step)
    world.find("physics/real_time_factor").text = str(real_time_factor)
    for old in list(world.findall("./model[@name='trajectory_markings']")):
        world.remove(old)
    model = ET.SubElement(world, "model", name="trajectory_markings")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="markings")

    def box(name, x, y, z, length, width, yaw, color):
        visual = ET.SubElement(link, "visual", name=name)
        ET.SubElement(visual, "pose").text = f"{x} {y} {z} 0 0 {yaw}"
        ET.SubElement(visual, "cast_shadows").text = "false"
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(ET.SubElement(geometry, "box"), "size").text = f"{length} {width} .001"
        material = ET.SubElement(visual, "material")
        for field in ("ambient", "diffuse"):
            ET.SubElement(material, field).text = color

    xmin, xmax = min(p[0] for p in points), max(p[0] for p in points)
    ymin, ymax = min(p[1] for p in points), max(p[1] for p in points)
    cx, cy = (xmin+xmax)/2, (ymin+ymax)/2
    for x in range(math.floor(xmin)-2, math.ceil(xmax)+3):
        box(f"grid_x_{x}", x, cy, .001, ymax-ymin+4., .008,
            math.pi/2, "0.52 0.56 0.60 1")
    for y in range(math.floor(ymin)-2, math.ceil(ymax)+3):
        box(f"grid_y_{y}", cx, y, .001, xmax-xmin+4., .008,
            0., "0.52 0.56 0.60 1")
    for index, (a, b) in enumerate(zip(points, points[1:])):
        if math.dist(a, b) <= 1e-9:
            raise ValueError("Consecutive waypoints must differ")
        box(f"route_{index}", (a[0]+b[0])/2, (a[1]+b[1])/2, .004,
            math.dist(a, b), .025, math.atan2(b[1]-a[1], b[0]-a[0]),
            "0.95 0.97 1 1")
    for index, (x, y) in enumerate(dict.fromkeys(points)):
        box(f"waypoint_{index}", x, y, .006, .065, .065, math.pi/4,
            "0.05 0.65 0.30 1" if index == 0 else "0.05 0.35 0.90 1")
    # Include the entire route and the full vehicle, with a useful oblique view.
    scale = max(1., (xmax-xmin+2.)/3., (ymax-ymin+2.)/3.)
    dx, dy, dz = 2.6*scale, -2.8*scale, 2.9*scale
    pose = (cx+dx, cy+dy, .30+dz, 0., math.atan2(dz, math.hypot(dx,dy)),
            math.atan2(-dy, -dx))
    camera = world.find("./model[@name='overview_camera']")
    camera.find("pose").text = " ".join(str(value) for value in pose)
    ET.indent(tree, space="  ")
    Path(destination).parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return {"points_m": points, "route_length_m": sum(math.dist(a,b) for a,b in zip(points,points[1:])),
            "markings_have_collisions": False, "overview_camera_pose": pose,
            "max_step_size_s": step, "target_real_time_factor": real_time_factor}
