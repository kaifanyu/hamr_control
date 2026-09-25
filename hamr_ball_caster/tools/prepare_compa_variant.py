#!/usr/bin/env python3
"""Create a separate compatible COMPA comparison chassis; never edit legacy URDF.

This is a documented retrofit, not a whole-HAMR3 CAD conversion. Legacy chassis,
gimbal, rocker linkage and inertias are inherited. The rocker beam geometry is
replaced by 25.4 mm square rails that fit the recovered 25.64 mm fork socket.
The two old cylindrical casters are removed, and five-DOF CAD casters added.
"""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
NS = "http://www.ros.org/wiki/xacro"
ET.register_namespace("xacro", NS)


def valid_stl(path):
    if not path.is_file():
        return False
    data = path.read_bytes()
    return len(data) >= 84 and len(data) == 84 + 50 * struct.unpack_from("<I", data, 80)[0]


def select_mesh_source(original, packaged, recorded_sha256=None):
    """Use the verified complete package copy when a legacy STL is truncated.

    The source checkout contains several truncated legacy meshes. Their complete
    package copies are versioned runtime assets, with recorded provenance hashes;
    regeneration must not depend on an author's private home-directory backup.
    """
    if valid_stl(original):
        return original
    if not valid_stl(packaged) or not recorded_sha256:
        raise RuntimeError(f"Incomplete legacy mesh {original}; a verified packaged mesh is required")
    content = packaged.read_bytes()
    if hashlib.sha256(content).hexdigest() != recorded_sha256:
        raise RuntimeError(f"Packaged mesh does not match recorded provenance: {packaged}")
    if original.exists() and not content.startswith(original.read_bytes()):
        raise RuntimeError(f"Packaged mesh does not extend the truncated legacy mesh: {original}")
    return packaged


def prepare():
    source = PACKAGE.parent / "compa_description/urdf"
    target = PACKAGE / "urdf"
    target.mkdir(exist_ok=True)
    root = ET.parse(source / "compa_back.urdf.xacro").getroot()
    root.set("name", "compa")
    provenance = {"basis": "COMPA retrofit, not native full HAMR3 assembly", "source_files": {}, "mesh_files": {},
                  "rail_size_m": [0.6658, 0.0254, 0.0254], "rail_x_extent_m": [-0.3, 0.3658],
                  "socket_center_above_ball_m": 0.13162}
    for filename in ("compa_back.urdf.xacro", "common_properties.urdf.xacro", "compa_gazebo.urdf.xacro"):
        provenance["source_files"][filename] = hashlib.sha256((source / filename).read_bytes()).hexdigest()
    for node in list(root):
        if node.tag == f"{{{NS}}}include":
            name = node.attrib["filename"]
            if name == "camera.urdf.xacro":
                root.remove(node)
            elif name == "common_properties.urdf.xacro":
                node.set("filename", "compa_common.xacro")
            elif name == "compa_gazebo.urdf.xacro":
                node.set("filename", "compa_gazebo.xacro")
    shutil.copyfile(source / "common_properties.urdf.xacro", target / "compa_common.xacro")
    root.insert(0, ET.Comment(" GENERATED separate COMPA retrofit. See docs/CAD_FINDINGS.md and tools/prepare_compa_variant.py. "))
    root.insert(1, ET.Element(f"{{{NS}}}arg", {"name": "caster_mass_scale", "default": "1.0"}))
    root.insert(2, ET.Element(f"{{{NS}}}arg", {"name": "carrier_phase", "default": "0.0"}))
    root.insert(3, ET.Element(f"{{{NS}}}include", {"filename": "ball_caster.xacro"}))
    for side, sign in (("right", -1), ("left", 1)):
        for node in list(root):
            if node.get("name") in (f"{side}_caster_wheel_link", f"{side}_rocker_{side}_caster_wheel_joint"):
                root.remove(node)
        rocker = root.find(f"./link[@name='{side}_rocker_link']")
        # Extend only the forward end: the socket spans x=.2642 to .3658.
        offset = f"0.0329 {sign*0.072} -0.021"
        for node in list(rocker):
            if node.tag in ("visual", "collision"):
                rocker.remove(node)
            elif node.tag == f"{{{NS}}}box_inertia":
                node.set("x", "0.6658"); node.set("y", "0.0254"); node.set("z", "0.0254")
                node.set("o_xyz", offset)
        for role in ("visual", "collision"):
            node = ET.SubElement(rocker, role)
            ET.SubElement(node, "origin", {"xyz": offset, "rpy": "0 0 0"})
            geom = ET.SubElement(node, "geometry")
            ET.SubElement(geom, "box", {"size": "0.6658 0.0254 0.0254"})
            if role == "visual":
                mat = ET.SubElement(node, "material", {"name": side + "_square_rail"})
                ET.SubElement(mat, "color", {"rgba": "0.68 0.70 0.72 1"})
        ET.SubElement(root, f"{{{NS}}}ball_caster", {
            "prefix": side + "_caster_", "parent": side + "_rocker_link",
            "center_xyz": f"0.315 {sign*0.072} -0.15262", "center_rpy": "0 0 0",
            "mass_scale": "$(arg caster_mass_scale)", "carrier_phase": "$(arg carrier_phase)",
        })
    mesh_dest = PACKAGE / "meshes/compa"
    mesh_dest.mkdir(parents=True, exist_ok=True)
    provenance_path = PACKAGE / "assets/provenance/compa_retrofit.json"
    previous_meshes = (json.loads(provenance_path.read_text()).get("mesh_files", {})
                       if provenance_path.exists() else {})
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename", "")
        if not uri.startswith("package://compa_description/meshes/"):
            continue
        name = uri.rsplit("/", 1)[-1]
        original = PACKAGE.parent / "compa_description/meshes" / name
        packaged = mesh_dest / name
        source_mesh = select_mesh_source(original, packaged, previous_meshes.get(name, {}).get("sha256"))
        if source_mesh.resolve() != packaged.resolve():
            shutil.copyfile(source_mesh, packaged)
        mesh.set("filename", "package://hamr_ball_caster/meshes/compa/" + name)
        provenance["mesh_files"][name] = {"sha256": hashlib.sha256(source_mesh.read_bytes()).hexdigest(),
                                          "restored_truncated_legacy_copy": source_mesh != original}
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(target / "compa_ball_caster.urdf.xacro", encoding="utf-8", xml_declaration=True)
    plugins = ET.parse(source / "compa_gazebo.urdf.xacro").getroot()
    for gz in list(plugins):
        for plugin in list(gz):
            if plugin.tag != "plugin":
                continue
            name = plugin.get("name", "")
            if name.endswith("JointStatePublisher"):
                ET.SubElement(plugin, "topic").text = "/ball_caster/joint_states"
            if name.endswith("WheelSlip"):
                # The inherited fixed wheel-normal-force assumption doesn't
                # describe the changed caster loads. Use rigid wheel contact.
                gz.remove(plugin)
    ET.indent(plugins, space="  ")
    ET.ElementTree(plugins).write(target / "compa_gazebo.xacro", encoding="utf-8", xml_declaration=True)
    (PACKAGE / "assets/provenance/compa_retrofit.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print("Prepared separate COMPA retrofit and complete visual assets")


if __name__ == "__main__":
    prepare()
