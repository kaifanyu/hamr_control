#!/usr/bin/env python3
"""Add collision-free route markings and a recording camera to the caster world."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

PACKAGE = Path(__file__).resolve().parents[1]


def main():
    source = PACKAGE.parent / "reference_trajectory/config/trajectories/waypoint_traj_simple.yaml"
    if not source.exists():
        source = PACKAGE / "config/waypoint_route.yaml"
    route = yaml.safe_load(source.read_text())
    points = route["points_m"]
    (PACKAGE / "config/waypoint_route.yaml").write_text(source.read_text())
    tree = ET.parse(PACKAGE / "worlds/ball_caster_test.sdf")
    world = tree.getroot().find("world")
    world.set("name", "hamr_waypoints")
    world.find("physics/real_time_factor").text = "0.5"
    world.find("scene/shadows").text = "false"
    world.find("light/cast_shadows").text = "false"
    ground = world.find("./model[@name='ground_plane']/link/visual/material")
    for name in ("ambient", "diffuse"):
        ground.find(name).text = "0.68 0.71 0.74 1"
    model = ET.SubElement(world, "model", name="waypoint_floor_markings")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="markings")

    def visual(name, x, y, z, yaw, color):
        element = ET.SubElement(link, "visual", name=name)
        ET.SubElement(element, "pose").text = f"{x} {y} {z} 0 0 {yaw}"
        geometry = ET.SubElement(element, "geometry")
        material = ET.SubElement(element, "material")
        for key in ("ambient", "diffuse"):
            ET.SubElement(material, key).text = color
        return geometry

    def bar(name, x, y, length, width, yaw=0, color="0.95 0.97 1 1", z=.003):
        geometry = visual(name, x, y, z, yaw, color)
        ET.SubElement(ET.SubElement(geometry, "box"), "size").text = f"{length} {width} .001"

    # A one-meter reference grid is purely visual and does not change contacts.
    for x in range(-4, 3):
        bar(f"grid_x_{x+4}", x, 2.25, 7.5, .008, math.pi/2, "0.52 0.56 0.60 1", .001)
    for y in range(-1, 7):
        bar(f"grid_y_{y+1}", -1, y, 7, .008, 0, "0.52 0.56 0.60 1", .001)
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        bar(f"route_segment_{i+1}", (a[0]+b[0])/2, (a[1]+b[1])/2,
            math.hypot(dx, dy), .032, math.atan2(dy, dx))

    # Seven-segment labels use geometry, avoiding texture/font dependencies.
    segment_layout = {"a": (0, .15, .13, .025, 0), "b": (.075, .075, .13, .025, math.pi/2),
                      "c": (.075, -.075, .13, .025, math.pi/2), "d": (0, -.15, .13, .025, 0),
                      "e": (-.075, -.075, .13, .025, math.pi/2), "f": (-.075, .075, .13, .025, math.pi/2),
                      "g": (0, 0, .13, .025, 0)}
    digit_segments = {1: "bc", 2: "abged", 3: "abgcd", 4: "fgbc", 5: "afgcd"}
    unique = list(dict.fromkeys(tuple(p) for p in points))
    for i, (x, y) in enumerate(unique, 1):
        color = "0.12 0.65 0.35 1" if i == 1 else "0.06 0.48 0.76 1"
        geometry = visual(f"waypoint_{i}", x, y, .005, 0, color)
        cylinder = ET.SubElement(geometry, "cylinder")
        ET.SubElement(cylinder, "radius").text = ".10"
        ET.SubElement(cylinder, "length").text = ".002"
        for key in digit_segments[i]:
            sx, sy, length, width, angle = segment_layout[key]
            bar(f"label_{i}_{key}", x + .28 + sx, y + sy, length, width, angle,
                "0.13 0.17 0.22 1", .006)
    camera = ET.parse(PACKAGE / "worlds/waypoint_camera.sdf").getroot().find("model")
    world.append(camera)
    plugin = ET.SubElement(world, "plugin", filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors")
    ET.SubElement(plugin, "render_engine").text = "ogre2"
    ET.indent(tree, space="  ")
    output = PACKAGE / "worlds/hamr_waypoint_recording.sdf"
    tree.write(output, encoding="utf-8", xml_declaration=True)
    provenance = {"route_source": "reference_trajectory/config/trajectories/waypoint_traj_simple.yaml",
                  "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                  "points_m": points, "floor_label_sequence": [1, 2, 3, 4, 5, 2, 1],
                  "route_length_m": sum(math.dist(a,b) for a,b in zip(points,points[1:])),
                  "markings_have_collisions": False,
                  "speed_note": "Executable waypoint_traj_simple defaults to0.25m/s; catalog YAML defaults to0.20m/s. Recording runner defaults to executable speed."}
    (PACKAGE / "assets/provenance/waypoint_route.json").write_text(json.dumps(provenance, indent=2)+"\n")
    print(output)


if __name__ == "__main__":
    main()
