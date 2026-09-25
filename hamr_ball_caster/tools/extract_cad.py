#!/usr/bin/env python3
"""Recover saved SolidWorks display meshes locally; never uploads CAD.

The saved triangle strips are a tessellated representation of the supplied CAD,
not a new solid-kernel export. Source hashes, parser losses, assembly-cache
differences and mesh quality are retained in assets/provenance/extraction.json.
Mass/inertia are reported for unit density, not as measured material properties.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import importlib.metadata
import json
from pathlib import Path, PureWindowsPath
import re
import xml.etree.ElementTree as ET

import numpy as np
import sldkit
import trimesh


PACKAGE = Path(__file__).resolve().parents[1]


def slug(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cad-dir", type=Path,
                        default=PACKAGE.parent / "CAD files" / "HAMR3-2")
    parser.add_argument("--output-dir", type=Path, default=PACKAGE)
    args = parser.parse_args()
    cad_dir = args.cad_dir.resolve()
    out = args.output_dir.resolve()
    meshes = out / "meshes" / "cad"
    provenance = out / "assets" / "provenance"
    meshes.mkdir(parents=True, exist_ok=True)
    provenance.mkdir(parents=True, exist_ok=True)
    assembly_path = cad_dir / "Ball Caster Assembly.SLDASM"
    inventory = sldkit.inspect_file(assembly_path).inventory
    entry = next(e for e in inventory.entries
                 if e.path == "swXmlContents/COMPINSTANCETREE")
    assembly_xml = sldkit.extract_file(assembly_path, entry.id).data
    if assembly_xml is None:
        raise RuntimeError("No decoded assembly component tree")
    tree = ET.fromstring(assembly_xml)
    ns = {"sw": "http://www.solidworks.com/sw2003/schema"}
    files = {f.attrib["id"]: f.attrib for f in tree.findall(".//sw:swFile", ns)}
    report = {
        "schema_version": 1,
        "method": "Saved SolidWorks DisplayLists triangles, sldkit 0.2.0",
        "parser_source": "https://github.com/monozukuri-ai/sldkit",
        "parser_license": "sldkit 0.2.0: PolyForm Noncommercial 1.0.0; see upstream notices",
        "dependencies": {p: importlib.metadata.version(p)
                         for p in ("sldkit", "trimesh", "numpy", "scipy")},
        "mesh_length_unit": "meter",
        "assembly_file": assembly_path.name,
        "assembly_sha256": hashlib.sha256(assembly_path.read_bytes()).hexdigest(),
        "mesh_processing": [
            "Convert parser millimeters to meters",
            "Merge vertices to eight decimal places in meters",
            "Remove zero-area/degenerate triangles and unused vertices",
            "Repair triangle winding per connected component; no new faces",
        ],
        "physical_property_caveat": (
            "Unit-density mass properties integrate the recovered closed mesh. "
            "They do not establish material density, manufactured mass, bearing "
            "drag, compliance, friction, or accuracy beyond the saved tessellation."
        ),
        "parts": {},
    }
    for model in tree.findall(".//sw:swModel", ns):
        attr = model.attrib
        source_file = files[attr["swFileRef"]]
        if source_file["swDocType"] != "PART":
            continue
        filename = PureWindowsPath(source_file["swPath"]).name
        source_path = cad_dir / filename
        key = slug(source_path.stem)
        if key in report["parts"]:
            continue
        result = sldkit.decode_geometry_file(source_path)
        document = sldkit.parse_file(source_path).document
        source_configurations = [
            {"id": c.index.value, "name": c.name.value if c.name else None}
            for c in document.configurations
        ]
        configuration_matches = (
            len(source_configurations) == 1
            and source_configurations[0]["name"] == attr.get("swConfigurationName")
            and str(source_configurations[0]["id"]) == attr.get("swConfigurationId")
        )
        if not configuration_matches:
            raise RuntimeError(
                f"Cannot verify uniquely matching assembly configuration for {filename}: "
                f"expected {attr.get('swConfigurationName')}, found {source_configurations}"
            )
        geometry = result.geometry
        if geometry is None or not geometry.model.tessellations:
            raise RuntimeError(f"No saved tessellation decoded for {filename}")
        if geometry.length_unit != "millimeter":
            raise RuntimeError(f"Unexpected parser units for {filename}")
        patches = [trimesh.Trimesh(vertices=np.asarray(t.vertices) * 0.001,
                                  faces=np.asarray(t.triangles), process=False)
                   for t in geometry.model.tessellations if t.triangles]
        mesh = trimesh.util.concatenate(patches)
        raw_face_count = len(mesh.faces)
        mesh.merge_vertices(digits_vertex=8)
        mesh.update_faces(mesh.nondegenerate_faces())
        mesh.remove_unreferenced_vertices()
        mesh.fix_normals(multibody=True)
        mesh_path = meshes / f"{key}.stl"
        mesh.export(mesh_path)
        cache_bounds = np.asarray([float(v) for v in attr["swBoundingBox"].split()]).reshape(2, 3)
        quality = {
            "watertight": bool(mesh.is_watertight),
            "winding_consistent": bool(mesh.is_winding_consistent),
            "positive_volume": bool(mesh.volume > 0),
        }
        part = {
            "source_file": filename,
            "source_sha256": geometry.source.sha256,
            "mesh_file": str(mesh_path.relative_to(out)),
            "mesh_sha256": hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
            "assembly_configuration": attr.get("swConfigurationName"),
            "source_configurations": source_configurations,
            "assembly_configuration_verified": configuration_matches,
            "parser_status": str(result.status),
            "saved_tessellation_patch_count": len(patches),
            "raw_triangle_count": raw_face_count,
            "triangle_count": len(mesh.faces),
            "vertex_count": len(mesh.vertices),
            "mesh_quality": quality,
            "bounds_m": mesh.bounds.tolist(),
            "assembly_cached_bounds_m": cache_bounds.tolist(),
            "max_assembly_cache_bound_difference_m": float(np.max(np.abs(mesh.bounds - cache_bounds))),
            "surface_area_m2": float(mesh.area),
            "unit_density_properties": None,
            "parser_losses": [dataclasses.asdict(loss) for loss in geometry.fidelity.losses],
            "analytic_surfaces_mm": [dataclasses.asdict(c)
                                     for c in geometry.model.carriers if c.domain == "surface"],
        }
        if all(quality.values()):
            part["unit_density_properties"] = {
                "volume_m3": float(mesh.volume),
                "center_of_mass_m": mesh.center_mass.tolist(),
                "inertia_at_com_m5": mesh.moment_inertia.tolist(),
                "mass_formula": "mass_kg = volume_m3 * density_kg_per_m3",
                "inertia_formula": "inertia_kg_m2 = inertia_at_com_m5 * density_kg_per_m3",
            }
        report["parts"][key] = part
        print(f"{filename}: {len(mesh.faces)} triangles; "
              f"watertight={mesh.is_watertight}; "
              f"cache bound difference={part['max_assembly_cache_bound_difference_m']:.6g} m",
              flush=True)
        (provenance / "extraction.json").write_text(
            json.dumps(report, indent=2, default=str) + "\n")
    print(f"Wrote {len(report['parts'])} meshes and {provenance / 'extraction.json'}")


if __name__ == "__main__":
    main()
