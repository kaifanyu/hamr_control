#!/usr/bin/env python3
"""Deterministic, closed terrain meshes shared by Gazebo visuals and collisions."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np

IDS = ("road_bumps", "gentle_hill", "cross_slope", "rutted_track", "rock_garden")
TITLES = (
    "Progressive road bumps",
    "Gentle raised hill",
    "Banked cross slope",
    "Ruts and washboard",
    "Fixed rock garden",
)
SPACING = 0.025


def package_directory():
    """Find data in a source checkout, symlink install, or regular ROS install."""
    candidate = Path(__file__).resolve().parents[1]
    if (candidate / "package.xml").is_file():
        return candidate
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("hamr_terrain"))


def ease(t):
    return 0.5 - 0.5 * np.cos(np.pi * np.clip(t, 0, 1))


def window(t, start=-1.8, rise=1.0, end=3.2, fall=1.0):
    return ease((t - start) / rise) * ease((end - t) / fall)


def bump(y, center, width, height):
    t = (y - center) / (width / 2)
    return np.where(np.abs(t) < 1, 0.5 * height * (1 + np.cos(np.pi * t)), 0.0)


def height(name, x, y):
    # All patches meet the flat apron continuously, with a flat south approach.
    lateral = ease((3.0 - np.abs(x)) / 0.9)
    if name == "road_bumps":
        road = ease((1.6 - np.abs(x)) / 0.35)
        return road * sum(
            bump(y, c, w, h)
            for c, w, h in [
                (-1.15, 0.40, 0.015),
                (0.25, 0.50, 0.025),
                (1.75, 0.60, 0.040),
            ]
        )
    if name == "gentle_hill":
        return 0.14 * window(y, -1.8, 2.2, 3.2, 2.2) * lateral
    if name == "cross_slope":
        return (
            (0.07 + np.tan(np.deg2rad(3.0)) * np.clip(x, -1.2, 1.2))
            * window(y, -1.8, 1.5, 3.2, 1.5)
            * lateral
        )
    if name == "rutted_track":
        rut = -0.022 * (
            np.exp(-(((x - 0.22) / 0.105) ** 2)) + np.exp(-(((x + 0.22) / 0.105) ** 2))
        )
        waves = 0.006 * np.sin(2 * np.pi * (y + 0.8) / 0.4)
        return (
            (rut + waves * np.exp(-((x / 1.25) ** 8)))
            * window(y, -1.8, 1.0, 3.2, 1.0)
            * lateral
        )
    if name == "rock_garden":
        # Embedded, immovable rounded rocks: no deformable/loose-gravel claim.
        rocks = [
            (-0.18, -1.25, 0.20, 0.24, 0.025),
            (0.18, -0.65, 0.18, 0.24, 0.035),
            (-0.28, 0.0, 0.20, 0.25, 0.045),
            (0.15, 0.62, 0.22, 0.22, 0.040),
            (-0.10, 1.25, 0.19, 0.28, 0.055),
            (0.32, 1.95, 0.25, 0.22, 0.045),
            (-0.38, 2.55, 0.25, 0.25, 0.035),
            (0.72, 0.30, 0.23, 0.34, 0.04),
            (-0.8, 1.6, 0.3, 0.25, 0.045),
            (0.9, 2.2, 0.25, 0.3, 0.03),
        ]
        z = sum(
            h * np.exp(-2 * ((x - cx) / rx) ** 2 - 2 * ((y - cy) / ry) ** 2)
            for cx, cy, rx, ry, h in rocks
        )
        z += (
            0.003
            * (np.sin(9 * x + 4 * y) * np.cos(7 * y - 2 * x) + 1.0)
            * window(y)
            * np.exp(-((x / 1.5) ** 8))
        )
        return z * window(y, -2.0, 0.35, 3.35, 0.35) * lateral
    raise ValueError(name)


def mesh_arrays(name, spacing=SPACING):
    x = np.linspace(-3, 3, round(6 / spacing) + 1)
    y = np.linspace(-4, 4, round(8 / spacing) + 1)
    xx, yy = np.meshgrid(x, y)
    zz = height(name, xx, yy)
    return mesh_from_height_grid(x, y, zz)


def mesh_from_height_grid(x, y, zz, bottom_z=-0.25):
    """Close a measured/generated height grid without changing its top triangles."""
    x, y, zz = (np.asarray(a, dtype=float) for a in (x, y, zz))
    if x.ndim != 1 or y.ndim != 1 or min(len(x), len(y)) < 2:
        raise ValueError(
            "Height-grid axes must be one dimensional with at least two samples"
        )
    if zz.shape != (len(y), len(x)) or not all(
        np.isfinite(a).all() for a in (x, y, zz)
    ):
        raise ValueError("Finite heights must have shape (len(y), len(x))")
    if not (np.all(np.diff(x) > 0) and np.all(np.diff(y) > 0)):
        raise ValueError("Height-grid axes must increase strictly")
    if not np.isfinite(bottom_z) or bottom_z >= float(zz.min()):
        raise ValueError("Mesh bottom must lie below every terrain height")
    xx, yy = np.meshgrid(x, y)
    vertices = np.column_stack((xx.ravel(), yy.ravel(), zz.ravel()))
    nx = len(x)
    a = (np.arange(len(y) - 1)[:, None] * nx + np.arange(nx - 1)[None, :]).ravel()
    faces = np.vstack(
        (
            np.column_stack((a, a + 1, a + nx + 1)),
            np.column_stack((a, a + nx + 1, a + nx)),
        )
    )
    # Perimeter traverses counterclockwise; top faces point upward.
    rim = (
        list(range(nx))
        + [j * nx + nx - 1 for j in range(1, len(y))]
        + list(range((len(y) - 1) * nx + nx - 2, (len(y) - 1) * nx - 1, -1))
        + [j * nx for j in range(len(y) - 2, 0, -1)]
    )
    base = len(vertices)
    bottom = vertices[rim].copy()
    bottom[:, 2] = bottom_z
    vertices = np.vstack(
        (vertices, bottom, [[(x[0] + x[-1]) / 2, (y[0] + y[-1]) / 2, bottom_z]])
    )
    center = len(vertices) - 1
    extra = []
    for i, r in enumerate(rim):
        j = (i + 1) % len(rim)
        rn = rim[j]
        extra += [
            (r, base + i, base + j),
            (r, base + j, rn),
            (center, base + j, base + i),
        ]
    return (
        x,
        y,
        zz,
        vertices,
        np.vstack((faces, np.array(extra, dtype=int))),
        len(faces),
    )


def stl(path, vertices, faces):
    triangles = vertices[faces]
    normal = np.cross(
        triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    )
    normal /= np.linalg.norm(normal, axis=1)[:, None]
    records = np.zeros(
        len(faces), dtype=[("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")]
    )
    records["n"] = normal
    records["v"] = triangles
    path.write_bytes(
        b"HAMR physical terrain; metres; Z up".ljust(80, b"\0")
        + struct.pack("<I", len(faces))
        + records.tobytes()
    )


def material_groups(name, vertices, faces, top_count):
    centers = vertices[faces].mean(axis=1)
    colors = [
        (0.30, 0.36, 0.29, 1),
        (0.23, 0.27, 0.23, 1),
        (0.19, 0.22, 0.26, 1),
        (0.31, 0.34, 0.38, 1),
        (0.86, 0.67, 0.23, 1),
        (0.53, 0.39, 0.26, 1),
        (0.35, 0.25, 0.16, 1),
        (0.53, 0.52, 0.45, 1),
        (0.64, 0.63, 0.54, 1),
        (0.78, 0.78, 0.70, 1),
    ]
    group = np.zeros(len(faces), dtype=int)
    x, y, z = centers.T
    if name == "road_bumps":
        group[:top_count] = 3
        group[(abs(x) < 1.25) & (np.arange(len(faces)) < top_count)] = 2
        raised = (z > 0.002) & (np.arange(len(faces)) < top_count)
        group[raised] = np.where((np.floor((x[raised] + 2) * 6) % 2) == 0, 4, 2)
    elif name in ("gentle_hill", "cross_slope"):
        group[:top_count] = np.where(abs(x[:top_count]) < 1.3, 3, 0)
        contours = (np.floor(z / 0.025) % 2) == 1
        group[contours & (abs(x) < 1.3)] = 2
    elif name == "rutted_track":
        group[:top_count] = 5
        group[(z < -0.008) & (np.arange(len(faces)) < top_count)] = 6
        group[(abs(x) > 1.3) & (np.arange(len(faces)) < top_count)] = 0
    elif name.startswith("rocky_"):
        # A rocky appearance is visual only; collision/material remain explicit.
        normals = np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],
                           vertices[faces[:,2]]-vertices[faces[:,0]])
        steep = abs(normals[:,2]) < .6*np.linalg.norm(normals,axis=1)
        group[:] = np.where(steep, 7, 5)
        group[(np.arange(len(faces))%7)<3] = np.where(steep[(np.arange(len(faces))%7)<3],8,7)
    elif name.startswith("full_uneven_"):
        # The complete patch uses one altitude palette; no painted road strip.
        group[:top_count] = np.asarray([6, 5, 7, 8, 9])[
            np.digitize(z[:top_count], [.009, .018, .027, .036])
        ]
    elif name.startswith("adtg_"):
        # Shared altitude colors across difficulty levels; material remains unchanged.
        group[:top_count] = np.asarray([5, 6, 7, 8, 9])[
            np.digitize(z[:top_count], [0.03, 0.06, 0.10, 0.18])
        ]
    else:
        group[:top_count] = 5
        group[z > 0.010] = 7
        group[z > 0.023] = 8
        group[z > 0.038] = 9
        group[(abs(x) > 1.3) & (np.arange(len(faces)) < top_count)] = 0
    group[top_count:] = 1
    return colors, group


def dae(path, name, vertices, faces, top_count):
    colors, groups = material_groups(name, vertices, faces, top_count)
    doc = ET.Element(
        "COLLADA", xmlns="http://www.collada.org/2005/11/COLLADASchema", version="1.4.1"
    )
    asset = ET.SubElement(doc, "asset")
    ET.SubElement(asset, "unit", name="meter", meter="1")
    ET.SubElement(asset, "up_axis").text = "Z_UP"
    effects = ET.SubElement(doc, "library_effects")
    materials = ET.SubElement(doc, "library_materials")
    for i, color in enumerate(colors):
        effect = ET.SubElement(effects, "effect", id=f"e{i}")
        profile = ET.SubElement(effect, "profile_COMMON")
        tech = ET.SubElement(profile, "technique", sid="common")
        lam = ET.SubElement(tech, "lambert")
        diff = ET.SubElement(lam, "diffuse")
        ET.SubElement(diff, "color").text = " ".join(map(str, color))
        ET.SubElement(ET.SubElement(lam, "ambient"), "color").text = " ".join(
            map(str, color)
        )
        ET.SubElement(ET.SubElement(lam, "emission"), "color").text = "0 0 0 1"
        material = ET.SubElement(materials, "material", id=f"m{i}", name=f"m{i}")
        ET.SubElement(material, "instance_effect", url=f"#e{i}")
    geometry = ET.SubElement(
        ET.SubElement(doc, "library_geometries"), "geometry", id="terrain"
    )
    mesh = ET.SubElement(geometry, "mesh")
    source = ET.SubElement(mesh, "source", id="positions")
    ET.SubElement(
        source, "float_array", id="positions_array", count=str(vertices.size)
    ).text = " ".join(f"{v:.7f}" for v in vertices.ravel())
    common = ET.SubElement(source, "technique_common")
    acc = ET.SubElement(
        common,
        "accessor",
        source="#positions_array",
        count=str(len(vertices)),
        stride="3",
    )
    for axis in "XYZ":
        ET.SubElement(acc, "param", name=axis, type="float")
    # Explicit face normals are required by Gazebo's Collada/Ogre2 importer.
    tri = vertices[faces]
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    ns = ET.SubElement(mesh, "source", id="normals")
    ET.SubElement(
        ns, "float_array", id="normals_array", count=str(normals.size)
    ).text = " ".join(f"{v:.7f}" for v in normals.ravel())
    na = ET.SubElement(
        ET.SubElement(ns, "technique_common"),
        "accessor",
        source="#normals_array",
        count=str(len(normals)),
        stride="3",
    )
    for axis in "XYZ":
        ET.SubElement(na, "param", name=axis, type="float")
    vs = ET.SubElement(mesh, "vertices", id="vertices")
    ET.SubElement(vs, "input", semantic="POSITION", source="#positions")
    for i in range(len(colors)):
        fs = faces[groups == i]
        if not len(fs):
            continue
        triangles = ET.SubElement(
            mesh, "triangles", count=str(len(fs)), material=f"m{i}"
        )
        ET.SubElement(
            triangles, "input", semantic="VERTEX", source="#vertices", offset="0"
        )
        ET.SubElement(
            triangles, "input", semantic="NORMAL", source="#normals", offset="1"
        )
        indices = np.stack(
            (fs, np.repeat(np.flatnonzero(groups == i)[:, None], 3, axis=1)), axis=2
        )
        ET.SubElement(triangles, "p").text = " ".join(map(str, indices.ravel()))
    scene = ET.SubElement(
        ET.SubElement(doc, "library_visual_scenes"), "visual_scene", id="Scene"
    )
    node = ET.SubElement(scene, "node", id="terrain_node")
    ig = ET.SubElement(node, "instance_geometry", url="#terrain")
    bind = ET.SubElement(ET.SubElement(ig, "bind_material"), "technique_common")
    for i in range(len(colors)):
        ET.SubElement(bind, "instance_material", symbol=f"m{i}", target=f"#m{i}")
    ET.SubElement(ET.SubElement(doc, "scene"), "instance_visual_scene", url="#Scene")
    ET.ElementTree(doc).write(path, encoding="utf-8", xml_declaration=True)


def element(parent, tag, value=None, **attrs):
    e = ET.SubElement(parent, tag, attrs)
    if value is not None:
        e.text = str(value)
    return e


def surface(collision, mu=1.0):
    s = element(collision, "surface")
    o = element(element(s, "friction"), "ode")
    element(o, "mu", mu)
    element(o, "mu2", mu)
    element(element(s, "bounce"), "restitution_coefficient", 0)


def camera(world, name, pose, topic, fov=1.0):
    model = element(world, "model", name=name)
    element(model, "static", "true")
    element(model, "pose", " ".join(map(str, pose)))
    link = element(model, "link", name="camera_link")
    sensor = element(link, "sensor", name="camera", type="camera")
    element(sensor, "always_on", "true")
    element(sensor, "update_rate", 5)
    element(sensor, "topic", topic)
    cam = element(sensor, "camera")
    element(cam, "horizontal_fov", fov)
    im = element(cam, "image")
    element(im, "width", 1600)
    element(im, "height", 1000)
    element(im, "format", "R8G8B8")
    clip = element(cam, "clip")
    element(clip, "near", 0.03)
    element(clip, "far", 100)


def look_at(origin, target):
    dx, dy, dz = np.subtract(target, origin)
    return list(origin) + [0, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx)]


def make_world(root, name, height_sampler=None):
    root = Path(root).resolve()
    # A custom preview must load its own meshes, even if the same model name is
    # already installed. Absolute local URIs are also used by the run snapshots.
    installed_output = root == package_directory().resolve()
    sdf = ET.Element("sdf", version="1.9")
    w = element(sdf, "world", name="hamr_terrain_" + name)
    p = element(w, "physics", name="terrain_dart", type="ignored")
    element(p, "max_step_size", 0.001)
    element(p, "real_time_factor", 1)
    dart = element(p, "dart")
    element(dart, "collision_detector", "ode")
    element(element(dart, "solver"), "solver_type", "dantzig")
    element(w, "gravity", "0 0 -9.80665")
    for filename, cls in [
        ("physics", "Physics"),
        ("user-commands", "UserCommands"),
        ("scene-broadcaster", "SceneBroadcaster"),
        ("contact", "Contact"),
    ]:
        pl = element(
            w,
            "plugin",
            filename=f"gz-sim-{filename}-system",
            name=f"gz::sim::systems::{cls}",
        )
        if cls == "Physics":
            element(element(pl, "engine"), "filename", "gz-physics-dartsim-plugin")
    sc = element(w, "scene")
    element(sc, "ambient", ".55 .55 .55 1")
    element(sc, "background", ".67 .75 .81 1")
    element(sc, "shadows", "true")
    terrain = element(w, "model", name="terrain")
    element(terrain, "static", "true")
    link = element(terrain, "link", name="surface")
    for kind, ext in [("collision", "stl"), ("visual", "dae")]:
        part = element(link, kind, name="terrain_" + kind)
        g = element(part, "geometry")
        m = element(g, "mesh")
        mesh_uri = (
            f"model://hamr_terrain/meshes/{name}.{ext}"
            if installed_output
            else str(root / "meshes" / f"{name}.{ext}")
        )
        element(m, "uri", mesh_uri)
        element(m, "scale", "1 1 1")
        if kind == "collision":
            surface(part)
            element(part, "max_contacts", 20)
    sensor = element(link, "sensor", name="terrain_contacts", type="contact")
    element(sensor, "always_on", "true")
    element(sensor, "update_rate", 100)
    contact = element(sensor, "contact")
    element(contact, "collision", "terrain_collision")
    element(contact, "topic", "/hamr/terrain/contacts")
    # Four non-overlapping outer aprons; no plane under the ruts or mesh interior.
    for i, (xyz, size) in enumerate(
        [
            ((-6, 0, -0.15), (6, 8, 0.3)),
            ((6, 0, -0.15), (6, 8, 0.3)),
            ((0, -6, -0.15), (18, 4, 0.3)),
            ((0, 6, -0.15), (18, 4, 0.3)),
        ]
    ):
        model = element(w, "model", name=f"apron_{i}")
        element(model, "static", "true")
        element(model, "pose", " ".join(map(str, (*xyz, 0, 0, 0))))
        l = element(model, "link", name="ground")
        for kind in ("collision", "visual"):
            part = element(l, kind, name=kind)
            element(
                element(element(part, "geometry"), "box"),
                "size",
                " ".join(map(str, size)),
            )
            if kind == "collision":
                surface(part)
            else:
                mat = element(part, "material")
                element(mat, "ambient", ".29 .35 .27 1")
                element(mat, "diffuse", ".36 .42 .30 1")
    light = element(w, "light", name="sun", type="directional")
    element(light, "pose", "0 0 10 0 0 0")
    element(light, "diffuse", ".9 .86 .78 1")
    element(light, "specular", ".15 .15 .15 1")
    element(light, "direction", "-.6 .45 -1")
    element(light, "cast_shadows", "true")
    # Lane markers are visual only, raised2mm above the exact sampled profile.
    for side in (-1, 1):
        for j, y in enumerate(np.arange(-3.5, 3.51, 0.5)):
            x = side * 1.55
            sampled = (
                height_sampler(x, y)
                if height_sampler is not None
                else height(name, np.array(x), np.array(y))
            )
            z = float(sampled) + 0.002
            model = element(w, "model", name=f"marker_{side}_{j}")
            element(model, "static", "true")
            element(model, "pose", f"{x} {y} {z} 0 0 0")
            l = element(model, "link", name="marker")
            v = element(l, "visual", name="visual")
            element(element(element(v, "geometry"), "box"), "size", ".055 .15 .003")
            mat = element(v, "material")
            element(mat, "ambient", ".85 .85 .72 1")
            element(mat, "diffuse", ".85 .85 .72 1")
    camera(
        w,
        "overview_camera",
        look_at((7.2, -7.2, 6.6), (0, 0.2, 0)),
        "/hamr/terrain/overview/image",
        0.95,
    )
    camera(
        w,
        "detail_camera",
        look_at((2.3, -3.9, 1.6), (0, 0.15, 0.04)),
        "/hamr/terrain/detail/image",
        1.05,
    )
    pl = element(
        w, "plugin", filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors"
    )
    element(pl, "render_engine", "ogre2")
    ET.indent(sdf, space="  ")
    ET.ElementTree(sdf).write(
        root / "worlds" / f"{name}.sdf", encoding="utf-8", xml_declaration=True
    )


DESCRIPTIONS = {
    "road_bumps": "Three smooth raised-cosine road bumps: 15, 25, 40 mm high; 0.40, 0.50, 0.60 m long.",
    "gentle_hill": "A smooth 140 mm rise with broad ascent, short crest and descent; centerline peak grade about 5.7 degrees.",
    "cross_slope": "A 3 degree bank across the driving lane, with smooth entry and exit ramps.",
    "rutted_track": "Two 22 mm wheel-track depressions and 6 mm amplitude washboard, with 0.40 m wavelength.",
    "rock_garden": "Fixed rounded embedded rocks 25–55 mm high plus small surface undulation; rigid packed-ground approximation.",
}
SMOKE_STARTS = {
    "road_bumps": (-1.75, 0.07),
    "gentle_hill": (-1.4, 0.09),
    "cross_slope": (-1.1, 0.12),
    "rutted_track": (-1.1, 0.07),
    "rock_garden": (-0.3, 0.10),
}
PROBES = {
    "road_bumps": [(0, -1.15), (0, 0.25), (0, 1.75)],
    "gentle_hill": [(0, -0.7), (0, 0.7), (0.5, 0.7)],
    "cross_slope": [(-0.7, 0.5), (0.7, 0.5), (0, -0.7)],
    "rutted_track": [(0.22, 0.9), (-0.22, 0.9), (0, 0.9)],
    "rock_garden": [(-0.18, -1.25), (-0.10, 1.25), (0.32, 1.95)],
}


def generate(root):
    root = Path(root)
    for sub in ("meshes", "worlds", "config"):
        (root / sub).mkdir(parents=True, exist_ok=True)
    manifest = []
    for name, title in zip(IDS, TITLES):
        x, y, z, v, f, n = mesh_arrays(name)
        stl(root / "meshes" / f"{name}.stl", v, f)
        dae(root / "meshes" / f"{name}.dae", name, v, f, n)
        np.savez_compressed(root / "meshes" / f"{name}_height.npz", x=x, y=y, z=z)
        make_world(root, name)
        dy, dx = np.gradient(z, y, x)
        slope = np.rad2deg(np.arctan(np.hypot(dx, dy)))
        data = {
            "id": name,
            "title": title,
            "description": DESCRIPTIONS[name],
            "world": f"../worlds/{name}.sdf",
            "bounds_m": {"x": [-3, 3], "y": [-4, 4]},
            "height_grid": f"../meshes/{name}_height.npz",
            "collision_mesh": f"../meshes/{name}.stl",
            "visual_mesh": f"../meshes/{name}.dae",
            "grid_spacing_m": SPACING,
            "vertices": len(v),
            "triangles": len(f),
            "surface_triangles": n,
            "min_height_m": float(z.min()),
            "max_height_m": float(z.max()),
            "max_surface_slope_deg": float(slope.max()),
            "slope_method": "Grid-node central finite differences over the top surface; excludes closing side walls.",
            "terrain_friction": {
                "mu": 1.0,
                "mu2": 1.0,
                "restitution": 0.0,
                "basis": "DART mesh attachment defaults; explicit matching SDF values; runtime probe validation required.",
            },
            "spawn": {"x": 0, "y": -2.8, "z": 0, "yaw": math.pi / 2},
            "probes": [{"name": "flat_approach", "xy_m": [0, -2.6], "kind": "flat"}]
            + [
                {"name": f"feature_{i}", "xy_m": list(xy), "kind": "feature"}
                for i, xy in enumerate(PROBES[name])
            ],
            "smoke": {
                "spawn_x": 0,
                "spawn_y": SMOKE_STARTS[name][0],
                "spawn_z": SMOKE_STARTS[name][1],
                "initial_yaw": math.pi / 2,
                "drive_speed_m_s": 0.12,
                "duration_s": 10.0,
            },
            "limitations": [
                "Rigid terrain, no loose particles, soil deformation or sinkage.",
                "Estimated contact and robot parameters; no calibrated motor torque limits.",
            ],
            "sha256": {
                ext: hashlib.sha256(
                    (root / "meshes" / f"{name}.{ext}").read_bytes()
                ).hexdigest()
                for ext in ("stl", "dae")
            },
        }
        (root / "config" / f"{name}.json").write_text(json.dumps(data, indent=2) + "\n")
        manifest.append(data)
    (root / "config/terrains.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=package_directory())
    args = parser.parse_args()
    print(
        json.dumps(
            [
                {
                    k: d[k]
                    for k in (
                        "id",
                        "triangles",
                        "min_height_m",
                        "max_height_m",
                        "max_surface_slope_deg",
                    )
                }
                for d in generate(args.output)
            ],
            indent=2,
        )
    )
