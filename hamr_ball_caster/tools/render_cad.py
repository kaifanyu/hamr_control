#!/usr/bin/env python3
"""Render the supplied saved assembly poses from the extracted CAD meshes.

Optional documentation tool: requires cad-requirements.txt, matplotlib, and
fast-simplification. Mesh simplification affects these images only.
"""

from pathlib import Path
import argparse
import re
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import sldkit
import trimesh


ROOT = Path(__file__).resolve().parents[1]
Q = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
COLORS = {"half1": "#2684b8", "half2": "#df8b35", "fork": "#8a9cab",
          "roller": "#4caf86", "carrier": "#5c617b", "hardware": "#b7b9c0"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cad-dir", type=Path, default=ROOT.parent / "CAD files" / "HAMR3-2")
    parser.add_argument("--output", type=Path, default=ROOT / "docs" / "cad_assembly_views.png")
    args = parser.parse_args()
    source = args.cad_dir / "Ball Caster Assembly.SLDASM"
    entry = next(e for e in sldkit.inspect_file(source).inventory.entries
                 if e.path == "swXmlContents/COMPINSTANCETREE")
    tree = ET.fromstring(sldkit.extract_file(source, entry.id).data)
    ns = {"sw": "http://www.solidworks.com/sw2003/schema"}
    models = {m.attrib["id"]: m.attrib for m in tree.findall(".//sw:swModel", ns)}
    parts = []
    cache = {}
    for reference in tree.findall(".//sw:swReference", ns):
        a = reference.attrib
        if a.get("swSuppressed") == "YES" or "swTransform" not in a:
            continue
        model = models[a["swModelRef"]]
        name = model["swName"]
        key = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
        path = ROOT / "meshes" / "cad" / f"{key}.stl"
        if not path.exists():
            continue
        if key not in cache:
            mesh = trimesh.load_mesh(path)
            if len(mesh.faces) > 12000:
                mesh = mesh.simplify_quadric_decimation(face_count=1200)
            cache[key] = mesh
        mesh = cache[key]
        transform = np.asarray([float(v) for v in a["swTransform"].split()]).reshape(4, 4).T
        rot = Q @ transform[:3, :3]
        triangles = mesh.triangles @ rot.T + Q @ transform[:3, 3]
        normals = mesh.face_normals @ rot.T
        if name.startswith("Semi-Spherical Wheel"):
            group = "half1" if a["swReferenceNumber"] == "1" else "half2"
        elif name == "Caster Fork":
            group = "fork"
        elif name == "Roller":
            group = "roller"
        elif name in ("Roller Mount", "Collet Block", "Bearing Shaft2", "Roller Shaft"):
            group = "carrier"
        else:
            group = "hardware"
        parts.append((triangles, normals, group, name))
    direction = Q @ np.asarray([0., .887645731278795, -.460526932700503])
    fig = plt.figure(figsize=(17, 11), facecolor="#fbfcff")
    views = [
        ("Front: X–Z", 0, -90, "assembled"),
        ("Side: Y–Z", 0, 0, "assembled"),
        ("Top: X–Y", 90, -90, "assembled"),
        ("Saved assembly pose", 23, -52, "assembled"),
        ("Exploded shells: same orientation", 23, -52, "exploded"),
        ("Interior: both shells hidden", 23, -52, "interior"),
    ]
    light = np.array([-.3, -.5, 1.]); light /= np.linalg.norm(light)
    for index, (title, elevation, azimuth, mode) in enumerate(views, 1):
        ax = fig.add_subplot(2, 3, index, projection="3d", facecolor="#fbfcff")
        ax.set_proj_type("ortho")
        all_triangles = []
        all_colors = []
        for triangles, normals, group, name in parts:
            if mode == "interior" and group.startswith("half"):
                continue
            shown = triangles.copy()
            if mode == "exploded" and group.startswith("half"):
                shown += direction * (.095 if group == "half1" else -.095)
            rgb = np.asarray(matplotlib.colors.to_rgb(COLORS[group]))
            shade = .55 + .45 * np.abs(normals @ light)
            faces = np.minimum(1, shade[:, None] * rgb)
            all_triangles.append(shown)
            all_colors.append(faces)
        # A single collection sorts individual triangles across different parts.
        # Separate collections can incorrectly paint a band over its own cap.
        ax.add_collection3d(Poly3DCollection(np.concatenate(all_triangles),
                                           facecolors=np.concatenate(all_colors), linewidths=0,
                                           edgecolors="none", rasterized=True))
        # Both rotation axes intersect at the CAD sphere center.
        if mode in ("interior", "exploded"):
            a = direction * .17
            ax.plot([-a[0], a[0]], [-a[1], a[1]], [-a[2], a[2]],
                    color="#9657ba", linestyle="--", linewidth=1.3)
            ax.plot([0, 0], [-.15, .15], [0, 0], color="#dd5656", linestyle="--", linewidth=1.3)
        span = .245 if mode == "exploded" else .145
        ax.set_xlim(-span, span); ax.set_ylim(-span, span)
        ax.set_zlim(-.115 if mode != "exploded" else -.205,
                    .175 if mode != "exploded" else .205)
        ax.set_box_aspect((2*span, 2*span, .29 if mode != "exploded" else .41))
        ax.view_init(elevation, azimuth)
        ax.set_title(title, fontsize=12, fontweight="bold", pad=3)
        ax.set_xlabel("X [m]", labelpad=0); ax.set_ylabel("Y [m]", labelpad=0)
        ax.set_zlabel("Z [m]", labelpad=0)
        if index == 1:
            ax.set_yticks([]); ax.set_ylabel("")
        elif index == 2:
            ax.set_xticks([]); ax.set_xlabel("")
        elif index == 3:
            ax.set_zticks([]); ax.set_zlabel("")
        ax.tick_params(labelsize=7, pad=0)
        ax.grid(True, alpha=.2)
    handles = [Patch(color=COLORS[k], label=l) for k,l in [
        ("half1", "Hemisphere 1: band + cap"), ("half2", "Hemisphere 2: band + cap"),
        ("roller", "Pole rollers"), ("fork", "Fixed fork"), ("carrier", "Inner carrier")]]
    fig.legend(handles=handles, loc="lower center", ncol=5, bbox_to_anchor=(.5,.055), frameon=False)
    fig.suptitle("Ball caster — supplied CAD geometry and saved assembly transforms", fontsize=17, fontweight="bold", y=.975)
    fig.text(.5,.945,"Z-up frame; sphere center at (0,0,0). Blue/orange shells retain the native 27.42° carrier pose.",
             ha="center", fontsize=10, color="#48515d")
    fig.text(.5,.026,"Exploded view shifts shell groups ±95 mm only; interior view hides shell bands/caps. "
             "Small hardware meshes simplified for this image only.\n"
             "Saved tessellation is used; material, friction and load calibration are separate. "
             "Dashed purple: split axis; dashed red: fork axle.", ha="center", fontsize=9, color="#48515d")
    fig.subplots_adjust(top=.91, bottom=.115, hspace=.1, wspace=.02)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=170)
    print(args.output)


if __name__ == "__main__":
    main()
