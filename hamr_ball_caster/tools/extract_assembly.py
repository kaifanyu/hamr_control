#!/usr/bin/env python3
"""Decode local SolidWorks assembly XML with stdlib and CRC verification.

No CAD upload and no native CAD application needed. Geometric occurrence poses
are native facts; rigid body grouping and bearings are explicit simulation
interpretations, not a solved SolidWorks mate graph.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path, PureWindowsPath
import re
import struct
import xml.etree.ElementTree as ET
import zlib

PACKAGE = Path(__file__).resolve().parents[1]
NS = {"sw": "http://www.solidworks.com/sw2003/schema"}
MARKER = b"\x14\x00\x06\x00\x08\x00"
IDENTITY = [[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]


def slug(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def streams(path):
    """Only accept framed raw-DEFLATE streams with matching length and CRC."""
    data = path.read_bytes()
    if len(data) < 8 or data[4:8] != b"\x00\x00\x00\x04":
        raise ValueError(f"Unsupported SolidWorks envelope: {path}")
    result = {}
    for marker in re.finditer(re.escape(MARKER), data):
        offset = marker.start()
        if offset + 26 > len(data):
            continue
        _, crc, compressed, unpacked, preamble = struct.unpack_from("<5I", data, offset + 6)
        start = offset + 26 + preamble
        if preamble > 4096 or start + compressed > len(data) or unpacked > 512 * 1024 * 1024:
            continue
        encoded = data[offset + 26:start]
        try:
            name = bytes((b >> 4) | ((b & 15) << 4) for b in encoded).decode("utf-8").strip("\0")
            payload = zlib.decompress(data[start:start + compressed], -15)
        except (UnicodeError, zlib.error):
            continue
        if len(payload) == unpacked and zlib.crc32(payload) == crc:
            result[name] = {"data": payload, "offset": offset, "crc32": f"{crc:08x}", "size": unpacked}
    if not result:
        raise ValueError(f"No valid compressed sections: {path}")
    return result


def matrix(text):
    v = [float(x) for x in text.split()]
    if len(v) != 16:
        raise ValueError("Native transform must contain 16 values")
    # XML stores a row-vector transform; transpose for p_parent = T @ p_local.
    return [[v[c * 4 + r] for c in range(4)] for r in range(4)]


def multiply(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def assembly(path):
    sections = streams(path)
    section = sections["swXmlContents/COMPINSTANCETREE"]
    root = ET.fromstring(section["data"])
    models = {m.attrib["id"]: m for m in root.findall("./sw:swModelList/sw:swModel", NS)}
    files = {f.attrib["id"]: f.attrib for f in root.findall("./sw:swHeader/sw:swFile", NS)}
    config = root.find("./sw:swConfigurationList/sw:swConfiguration", NS)
    root_model = models[config.attrib["swModelRef"]]
    rows = []

    def walk(model, parent, prefix):
        for ref in model.findall("sw:swReference", NS):
            a = ref.attrib
            child = models[a["swModelRef"]]
            source = files[child.attrib["swFileRef"]]
            local = matrix(a["swTransform"])
            world = multiply(parent, local)
            occurrence = f"{a['swName']}-{a['swReferenceNumber']}"
            filename = PureWindowsPath(source["swPath"]).name
            bounds = [float(v) for v in child.attrib.get("swBoundingBox", "").split()]
            rows.append({
                "key": f"{slug(a['swName'])}_{a['swReferenceNumber']}",
                "path": prefix + "/" + occurrence,
                "name": a["swName"],
                "part_key": slug(Path(filename).stem),
                "instance_number": int(a["swReferenceNumber"]),
                "source_file": filename,
                "document_type": source["swDocType"],
                "configuration": a["swConfigurationName"],
                "transform": world,
                "local_transform": local,
                "cached_bounds_m": [bounds[:3], bounds[3:]] if bounds else None,
                "visibility": "hidden" if a["swHidden"] == "YES" else "visible",
                "suppressed": a["swSuppressed"] == "YES",
            })
            walk(child, world, prefix + "/" + occurrence)

    walk(root_model, IDENTITY, path.stem)
    provenance = {"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                  "tree_stream": "swXmlContents/COMPINSTANCETREE", "tree_crc32": section["crc32"],
                  "tree_offset": section["offset"], "tree_size_bytes": section["size"],
                  "valid_stream_count": len(sections)}
    return rows, provenance, sections


def mate_references(data):
    rows = []
    for match in re.finditer(rb"(?:[ -~]\x00){5,}", data):
        value = match.group().decode("utf-16le")
        if re.fullmatch(r"(?:Coincident|Concentric|Distance|Parallel|Angle|Lock|Perpendicular|Tangent|Gear|Width)\d+", value):
            rows.append({"name": value, "offset": match.start(), "references": []})
        elif rows and "@Ball Caster Assembly" in value:
            rows[-1]["references"].append(value)
    return rows


def classify(row, split_axis):
    part = row["part_key"]
    projection = dot([row["transform"][i][3] for i in range(3)], split_axis)
    side = "positive" if projection >= 0 else "negative"
    if part == "caster_fork":
        return "fork", "Native fixed assembly reference (identity pose); root support."
    if part == "ball_caster_shaft_2":
        return "fork", "Main axle on assembly X; fork/shaft Coincident137 and Coincident138. Assumed fixed to fork; carrier needle bearings roll around it."
    if part in ("semi_spherical_wheel", "semi_spherical_wheel_2"):
        yaxis = [row["transform"][i][1] for i in range(3)]
        side = "positive" if dot(yaxis, split_axis) > 0 else "negative"
        return side + "_shell", "Common sphere center and matching local-Y axis. Band+cap are paired by Coincident1/2/22 (positive), Coincident4/6/21 (negative), and fasteners."
    if part == "roller":
        return side + "_polar_roller", "Polar roller spins on its own small transverse shaft; center sign along split axis determines side."
    if part in ("5_16_18screw", "5_16nuts"):
        return side + "_shell", "Fastener center lies on this side of the split and native mates connect corresponding shell holes; rigidly lumped."
    if part in ("skate_bearing", "flange_bearing"):
        return side + "_shell", "Single CAD bearing solid lumped with adjacent shell using axial position. Inner/outer race motion unresolved; bearing rolling dynamics omitted."
    if part in ("bearing_shaft2", "collet_block", "roller_mount", "roller_shaft"):
        return "carrier", "Cross-shaft carrier hardware; mount/shaft native mate references (including Coincident171/172) and orthogonal axes support grouping."
    if part in ("5905k22_needle_roller_bearing", "4668k11_permanently_lubricated_stainless_steel_ball_bearing"):
        return "carrier", "Bearing mass/visual lumped with carrier-supported housing. Individual races and balls are not articulated; bearing drag is a simulation parameter."
    if part == "spacing":
        return "carrier", "Spacer halves mate to main shaft and collet (Coincident154–169); carrier lumping is a model reduction, not a decoded mate-lock flag."
    raise ValueError(f"No documented group for {row['source_file']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cad-dir", type=Path, default=PACKAGE.parent / "CAD files" / "HAMR3-2")
    parser.add_argument("--output", type=Path, default=PACKAGE / "assets/provenance/assembly.json")
    args = parser.parse_args()
    rows, source, sections = assembly(args.cad_dir / "Ball Caster Assembly.SLDASM")
    shell = next(r for r in rows if r["key"] == "semi_spherical_wheel_1")
    split_axis = [shell["transform"][i][1] for i in range(3)]
    for row in rows:
        row["group"], row["group_evidence"] = classify(row, split_axis)
    joints = {"carrier": {"parent": "fork", "child": "carrier", "origin": [0., 0., 0.], "axis": [1., 0., 0.], "type": "continuous"}}
    for side, sign in (("positive", 1.), ("negative", -1.)):
        group = side + "_shell"
        joints[group] = {"parent": "carrier", "child": group, "origin": [0., 0., 0.], "axis": [sign * v for v in split_axis], "type": "continuous"}
        roller = next(r for r in rows if r["group"] == side + "_polar_roller")
        group = side + "_polar_roller"
        joints[group] = {"parent": "carrier", "child": group,
                         "origin": [roller["transform"][i][3] for i in range(3)],
                         "axis": [roller["transform"][i][1] for i in range(3)], "type": "continuous"}
    whole = {}
    for name in ("HAMR3-3", "HAMR3-2"):
        poses, robot_source, _ = assembly(args.cad_dir / (name + ".SLDASM"))
        selected = [r for r in poses if r["name"] in ("Ball Caster Assembly", "Caster Fork", "DriveWheels (1)", "Rail long", "Semi-Spherical Wheel")]
        rails = [r for r in selected if r["name"] == "Rail long"]
        frame = [sum(r["transform"][i][3] for r in rails) / len(rails) for i in range(3)]
        whole[name] = {"source": robot_source, "frame_center_from_long_rail_origins_m": frame,
                       "global_up": [0., 1., 0.], "occurrences": selected,
                       "note": "Cached whole-robot snapshot; geometry and some visibility differ from current parts and standalone assembly. Drive wheel nominal radius 0.130m differs from COMPA collision radius 0.1075m."}
    report = {
        "schema_version": 1, "length_unit": "meter", "source": source,
        "transform_convention": "4x4 column-vector transform: point_assembly = transform @ point_part. Native XML's row-vector swTransform is transposed. All standalone original rotations and translations retained.",
        "method": "Raw-DEFLATE version-4 SolidWorks streams with payload length and CRC-32 validation; native component-tree XML.",
        "format_reference": "https://github.com/cadmpeg/cadmpeg/blob/main/docs/formats/sldprt.md",
        "format_reference_note": "Unofficial format documentation; actual stream CRC and matrix consistency checked locally.",
        "occurrences": rows, "joint_axes": joints, "whole_robot": whole,
        "mate_reference_evidence": mate_references(sections["Contents/Config-0-MatesList"]["data"]),
        "geometry": {"nominal_sphere_radius_m": .1, "equatorial_gap_m": .02,
                     "band_local_y_m": [.01, .04], "cap_local_y_m": [.04, .09682458365518543],
                     "polar_roller_center_radius_m": .09038, "split_axis_native": split_axis,
                     "fork_socket_center_native_m": [0., 0., -.13162],
                     "fork_socket_direction_native": [0., 1., 0.],
                     "fork_socket_inner_width_m": .02564, "fork_socket_inner_height_m": .02564,
                     "fork_socket_planes_m": {"x": [-.01282, .01282], "z": [-.14444, -.11880]},
                     "fork_socket_evidence": "Recovered Caster Fork analytic plane records surf3550,3553,3554,3558 in extraction.json; checked against current saved tessellation. Center is a useful attachment reference, not a native named coordinate system.",
                     "fork_highest_surface_native_z_m": -.15642},
        "caveats": [
            "Kinematic grouping is a documented engineering interpretation of axes, bearings, component poses and mate reference names. Native binary mate lock flags and a complete constraint solve were not decoded.",
            "Shell-part model history says Material <not specified>. Generic Steel/defaultplastic appearance strings do not establish physical density; masses and contact parameters require physical calibration.",
            "Bearing CAD solids are lumped into shell or carrier; rolling elements and separate races are not resolved.",
            "Current part geometry can differ from cached assembly bounds. Shaft2 cached length 0.210m versus recovered current mesh length 0.230m, and fork bounds differ. See extraction.json for per-part comparisons.",
            "Standalone and whole-robot snapshots have different saved carrier roll angles. The hemisphere tilt is an articulation state, not fixed camber.",
            "Wheel1 preview shows tread detail that may differ from saved display tessellation. Recovery is based on saved geometry caches; native SolidWorks rebuild/export remains the strongest fidelity check.",
            "Existing COMPA simulation has different chassis, wheel sizes and mount positions. A new COMPA variant is a retrofit, not an exact full HAMR3-3 export.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(f"Wrote {args.output}: {len(rows)} occurrences, {len(set(r['part_key'] for r in rows))} part types, {len(joints)} revolute joints")


if __name__ == "__main__":
    main()
