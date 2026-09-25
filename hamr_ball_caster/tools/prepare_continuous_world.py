#!/usr/bin/env python3
"""Overlay a continuous reference with a visual-only ribbon in a camera world."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare_continuous_world(source_world, output_world, plan):
    """Preserve the source world physics and add a collision-free blue STL strip.

    A single mesh keeps software rendering inexpensive. Its vertices are built
    from the same dense planned positions used for the recorded route, in metres.
    Markings are reference geometry only and never alter measured vehicle poses.
    """
    source_world, output_world = Path(source_world), Path(output_world)
    tree = ET.parse(source_world)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("Source SDF must contain a world")
    points = []
    for sample in plan["samples"]:
        p = tuple(float(v) for v in sample["position_m"][:2])
        if len(p) != 2 or not all(math.isfinite(v) for v in p):
            raise ValueError("Non-finite or invalid planned ribbon point")
        if not points or math.dist(p, points[-1]) > 1e-9:
            points.append(p)
    if len(points) < 2:
        raise ValueError("Need at least two distinct planned positions")
    output_world.parent.mkdir(parents=True, exist_ok=True)
    mesh_path = output_world.with_name(output_world.stem+"_reference.stl")
    half_width = .012
    triangles = []
    for a, b in zip(points[:-1], points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        norm = math.hypot(dx, dy)
        nx, ny = -dy/norm*half_width, dx/norm*half_width
        corners = ((a[0]+nx,a[1]+ny), (a[0]-nx,a[1]-ny),
                   (b[0]-nx,b[1]-ny), (b[0]+nx,b[1]+ny))
        for indices in ((0,1,2), (0,2,3)):
            triangles.append("  facet normal 0 0 1\n    outer loop\n"+
                "".join(f"      vertex {corners[i][0]:.10g} {corners[i][1]:.10g} 0\n"
                        for i in indices)+"    endloop\n  endfacet\n")
    mesh_path.write_text("solid continuous_reference\n"+"".join(triangles)+"endsolid continuous_reference\n")
    model = ET.SubElement(world, "model", name="continuous_reference_marking")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = "0 0 .009 0 0 0"
    link = ET.SubElement(model, "link", name="reference_ribbon")
    visual = ET.SubElement(link, "visual", name="planned_curve")
    ET.SubElement(visual, "cast_shadows").text = "false"
    geometry = ET.SubElement(visual, "geometry")
    mesh = ET.SubElement(geometry, "mesh")
    ET.SubElement(mesh, "uri").text = mesh_path.resolve().as_uri()
    material = ET.SubElement(visual, "material")
    for name in ("ambient", "diffuse", "emissive"):
        ET.SubElement(material, name).text = "0.04 0.25 0.95 1"
    ET.indent(tree, space="  ")
    tree.write(output_world, encoding="utf-8", xml_declaration=True)
    return {"source_world": str(source_world.resolve()),
            "world": str(output_world.resolve()), "reference_mesh": str(mesh_path.resolve()),
            "reference_mesh_sha256": hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
            "world_sha256": hashlib.sha256(output_world.read_bytes()).hexdigest(),
            "ribbon_width_m": 2*half_width, "triangles": len(triangles),
            "markings_have_collisions": False,
            "legend": "Blue: planned continuous curve; white: original straight route"}


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_world", type=Path)
    parser.add_argument("plan", type=Path)
    parser.add_argument("output_world", type=Path)
    args = parser.parse_args()
    report = json.loads(args.plan.read_text())
    print(json.dumps(prepare_continuous_world(args.source_world, args.output_world,
                     report.get("continuous_plan", report)), indent=2))


if __name__ == "__main__":
    main()
