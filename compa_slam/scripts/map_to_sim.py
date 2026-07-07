#!/usr/bin/env python3
"""
map_to_sim.py
=============
Convert a RECORDED real-world map into a Gazebo simulation of the same terrain.

This is the "record -> convert -> run in sim" bridge for the COMPA pipeline. It takes
the 3D geometry you captured on the real robot and rasterises it into a 2.5D heightmap,
which is the ONE artifact the simulator and the off-road planner both consume:

      recorded 3D map                 this script               sim + planner
   ----------------------         -----------------         ---------------------
   RTAB-Map dense cloud   ──►   heightmap PNG (DEM)   ──►   Gazebo <heightmap> world
   (.ply/.pcd, exported)        + sidecar .yaml            (robot drives the terrain)
   OR an elevation DEM          + world .sdf               + cost_map_publisher reads
   (grayscale/16-bit PNG)                                    the SAME PNG -> /elevation_map
                                                             + /costmap for or_planner

Why a heightmap is the target (see docs/HANDOFF.md "Sim replay" section):
  * Gazebo cannot ingest an RTAB-Map .db or a ROS grid_map directly. Its terrain is a
    <heightmap> geometry = a 2.5D digital elevation model (DEM) = a grayscale image.
  * The existing planner (or_planner) already eats /elevation_map (grid_map) + /costmap,
    and hamr_control_cpp/cost_map_publisher already builds BOTH of those from a PNG.
  * So a DEM PNG makes the *physics terrain* and the *planner map* identical and aligned.

RTAB-Map vs. the elevation/"cupy" terrain map -- which feeds the sim?
  * RTAB-Map's job is localisation + a dense 3D point cloud. That cloud is the INPUT here.
  * The elevation map (CPU `elevation_mapping` / grid_map_pcl; "cupy" is the GPU variant we
    do NOT use) is just a live, online rasterisation of that same cloud into a height grid.
  * This script does the rasterisation offline. So: the sim terrain ultimately comes from
    RTAB-Map's reconstruction, expressed as an elevation DEM. Either entry point works:
      - feed it the RTAB-Map cloud  (`--cloud map.ply`  or  `--db map.db`)
      - feed it an elevation DEM    (`--dem elevation.png`) if you already have one.

------------------------------------------------------------------------------------------
USAGE
------------------------------------------------------------------------------------------
  # 0) (if starting from an RTAB-Map .db) export a dense cloud first:
  rtabmap-export --cloud --output maps/compa_real maps/compa_real.db
  #   -> writes maps/compa_real.ply

  # 1a) from a point cloud (.ply / .pcd / .xyz):
  python3 map_to_sim.py --cloud maps/compa_real.ply --name compa_real

  # 1b) straight from the .db (runs rtabmap-export for you if it's on PATH):
  python3 map_to_sim.py --db maps/compa_real.db --name compa_real

  # 1c) from an already-made elevation DEM image:
  python3 map_to_sim.py --dem maps/compa_real_dem.png --size 8 --height 0.7 --name compa_real

Outputs (into --outdir, default = the package's maps/ dir):
  <name>_heightmap.png   the Gazebo/planner DEM   ((2^n)+1 square, grayscale)
  <name>.yaml            metadata (extent, height scale, resolution, origin)
  <name>.sdf             a ready-to-launch Gazebo world referencing the PNG

Then:
  ros2 launch compa_slam replay_map_sim.launch.py map:=compa_real

Dependencies: numpy + pillow only (no open3d/ROS needed). `pip install numpy pillow`.
The point-cloud readers cover ASCII/binary-little-endian PLY, ASCII/binary PCD, and XYZ.
"""

import argparse
import os
import shutil
import subprocess
import sys

import numpy as np

try:
    from PIL import Image
except ImportError:
    sys.exit("ERROR: pillow is required.  pip install pillow")


# Gazebo heightmaps want a SQUARE image whose side is (2^n)+1 (terrain paging needs this).
_VALID_SIDES = [2 ** n + 1 for n in range(5, 13)]  # 33, 65, 129, ... 4097


# --------------------------------------------------------------------------------------
# Point-cloud readers (dependency-light; cover the formats rtabmap-export / ROS emit)
# --------------------------------------------------------------------------------------
def _read_ply(path):
    """ASCII or binary_little_endian PLY. Returns Nx3 float array of x,y,z."""
    with open(path, "rb") as f:
        if f.readline().strip() != b"ply":
            raise ValueError(f"{path}: not a PLY file")
        fmt = None
        n_vert = 0
        props = []          # list of (name, numpy_dtype) for the vertex element
        in_vertex = False
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if line.startswith("format"):
                fmt = line.split()[1]
            elif line.startswith("element"):
                parts = line.split()
                in_vertex = parts[1] == "vertex"
                if in_vertex:
                    n_vert = int(parts[2])
            elif line.startswith("property") and in_vertex:
                parts = line.split()
                if parts[1] == "list":
                    props.append((parts[-1], "list"))
                else:
                    props.append((parts[2], _ply_dtype(parts[1])))
            elif line == "end_header":
                break

        names = [n for n, _ in props]
        if not {"x", "y", "z"} <= set(names):
            raise ValueError(f"{path}: vertex element lacks x/y/z")

        if fmt == "ascii":
            data = np.loadtxt(f, max_rows=n_vert)
            idx = [names.index(c) for c in ("x", "y", "z")]
            return np.atleast_2d(data)[:, idx].astype(np.float64)

        # binary_little_endian / big_endian. 'list' props (faces) don't appear in the
        # vertex element in practice, so assume the vertex record is all scalars.
        endian = "<" if fmt == "binary_little_endian" else ">"
        vdtype = np.dtype([(n, dt) for n, dt in props if dt != "list"]).newbyteorder(endian)
        buf = f.read(vdtype.itemsize * n_vert)
        arr = np.frombuffer(buf, dtype=vdtype, count=n_vert)
        return np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float64)


def _ply_dtype(t):
    return {
        "char": "i1", "uchar": "u1", "uint8": "u1", "int8": "i1",
        "short": "i2", "ushort": "u2", "int16": "i2", "uint16": "u2",
        "int": "i4", "uint": "u4", "int32": "i4", "uint32": "u4",
        "float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
    }[t]


def _read_pcd(path):
    """ASCII or binary PCD. Returns Nx3 float array of x,y,z."""
    with open(path, "rb") as f:
        fields, sizes, types, counts = [], [], [], []
        npts, data_kind = 0, "ascii"
        while True:
            raw = f.readline()
            if not raw:
                raise ValueError(f"{path}: truncated PCD header")
            line = raw.decode("ascii", "replace").strip()
            if line.startswith("#") or not line:
                continue
            key, *vals = line.split()
            key = key.upper()
            if key == "FIELDS":
                fields = vals
            elif key == "SIZE":
                sizes = [int(v) for v in vals]
            elif key == "TYPE":
                types = vals
            elif key == "COUNT":
                counts = [int(v) for v in vals]
            elif key == "POINTS":
                npts = int(vals[0])
            elif key == "DATA":
                data_kind = vals[0].lower()
                break
        if not counts:
            counts = [1] * len(fields)
        idx = {fld: i for i, fld in enumerate(fields)}
        if not {"x", "y", "z"} <= set(idx):
            raise ValueError(f"{path}: PCD lacks x/y/z fields")

        if data_kind == "ascii":
            data = np.loadtxt(f, max_rows=npts)
            data = np.atleast_2d(data)
            return data[:, [idx["x"], idx["y"], idx["z"]]].astype(np.float64)

        # binary (not binary_compressed)
        np_types = {"F": "f", "I": "i", "U": "u"}
        dtype = np.dtype([
            (fld, np_types[types[i]] + str(sizes[i]))
            for i, fld in enumerate(fields) for _ in range(1)
        ])
        buf = f.read(dtype.itemsize * npts)
        arr = np.frombuffer(buf, dtype=dtype, count=npts)
        return np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float64)


def _read_xyz(path):
    data = np.loadtxt(path)
    return np.atleast_2d(data)[:, :3].astype(np.float64)


def read_cloud(path):
    ext = os.path.splitext(path)[1].lower()
    reader = {".ply": _read_ply, ".pcd": _read_pcd,
              ".xyz": _read_xyz, ".txt": _read_xyz}.get(ext)
    if reader is None:
        raise ValueError(f"Unsupported cloud format: {ext} (use .ply/.pcd/.xyz)")
    pts = reader(path)
    pts = pts[np.isfinite(pts).all(axis=1)]
    if pts.shape[0] == 0:
        raise ValueError(f"{path}: no finite points")
    return pts


def export_db_to_ply(db_path, out_base):
    """Run rtabmap-export to turn a .db into a dense .ply. Returns the .ply path."""
    if shutil.which("rtabmap-export") is None:
        sys.exit(
            "ERROR: --db given but `rtabmap-export` is not on PATH.\n"
            "Install rtabmap (ros-jazzy-rtabmap-ros) or export manually:\n"
            f"  rtabmap-export --cloud --output {out_base} {db_path}\n"
            "then re-run with  --cloud <that>.ply")
    print(f"[map_to_sim] rtabmap-export --cloud --output {out_base} {db_path}")
    subprocess.run(["rtabmap-export", "--cloud", "--output", out_base, db_path],
                   check=True)
    ply = out_base + ".ply"
    if not os.path.exists(ply):
        sys.exit(f"ERROR: expected {ply} from rtabmap-export but it was not created.")
    return ply


# --------------------------------------------------------------------------------------
# Rasterisation: point cloud -> DEM grid
# --------------------------------------------------------------------------------------
def cloud_to_dem(pts, side, size=None, z_clip=(0.5, 99.9), agg="max"):
    """
    Rasterise an Nx3 cloud into a (side x side) DEM (float meters, NaN where empty).
    Returns (dem, meta) where meta has extent/resolution/origin/height info.

    The grid is square, centred on the cloud's XY centroid. `size` (meters) sets the
    side length; if None it is the larger of the cloud's X/Y span (+5% margin).
    z_clip percentiles drop SLAM outliers (stray ceiling/floor points).
    agg = "max" keeps the top surface (drive ON the terrain); "mean" smooths.
    """
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

    # Drop vertical outliers (ceilings, flyaway points) before measuring extent.
    zlo, zhi = np.percentile(z, z_clip)
    keep = (z >= zlo) & (z <= zhi)
    x, y, z = x[keep], y[keep], z[keep]

    cx, cy = 0.5 * (x.min() + x.max()), 0.5 * (y.min() + y.max())
    if size is None:
        span = max(x.max() - x.min(), y.max() - y.min())
        size = float(span * 1.05) if span > 0 else 1.0
    half = size / 2.0
    x0, y0 = cx - half, cy - half          # bottom-left corner in world XY
    res = size / (side - 1)

    # Bin each point to a cell index.
    cols = np.clip(((x - x0) / res).astype(int), 0, side - 1)
    rows = np.clip(((y - y0) / res).astype(int), 0, side - 1)
    flat = rows * side + cols

    dem = np.full(side * side, np.nan, dtype=np.float64)
    order = np.argsort(z)                    # so "max" keeps the highest z per cell
    if agg == "max":
        dem[flat[order]] = z[order]          # last write per cell wins = highest
    else:  # mean
        sums = np.bincount(flat, weights=z, minlength=side * side)
        cnts = np.bincount(flat, minlength=side * side)
        nz = cnts > 0
        dem[nz] = sums[nz] / cnts[nz]
    dem = dem.reshape(side, side)

    dem = _fill_holes(dem)
    meta = dict(size=float(size), resolution=float(res),
                origin_x=float(x0), origin_y=float(y0),
                center_x=float(cx), center_y=float(cy))
    return dem, meta


def _fill_holes(dem):
    """Fill NaN cells by iterative nearest-neighbour dilation (no SciPy needed)."""
    out = dem.copy()
    nan = np.isnan(out)
    if not nan.any():
        return out
    # Seed completely-empty maps with 0 so the loop terminates.
    if nan.all():
        return np.zeros_like(out)
    for _ in range(max(out.shape)):
        if not nan.any():
            break
        # Average of up-to-4 neighbours that are already filled.
        acc = np.zeros_like(out)
        cnt = np.zeros_like(out)
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            shifted = np.roll(np.where(nan, 0.0, out), (dr, dc), axis=(0, 1))
            valid = np.roll(~nan, (dr, dc), axis=(0, 1))
            acc += np.where(valid, shifted, 0.0)
            cnt += valid
        fillable = nan & (cnt > 0)
        out[fillable] = acc[fillable] / cnt[fillable]
        nan = np.isnan(out)
    out[np.isnan(out)] = np.nanmin(dem)
    return out


def dem_from_image(path, height):
    """Load an existing grayscale DEM image -> float DEM in [0, height] meters."""
    img = np.asarray(Image.open(path).convert("I"))   # 32-bit int view
    maxv = 65535.0 if img.max() > 255 else 255.0
    return (img.astype(np.float64) / maxv) * height


# --------------------------------------------------------------------------------------
# DEM -> Gazebo heightmap PNG (square, (2^n)+1, normalised)
# --------------------------------------------------------------------------------------
def dem_to_png(dem, png_path, bits=8):
    """
    Normalise a float DEM to a grayscale PNG. Returns (z_min, z_max).
    Gazebo maps pixel 0 -> world height 0 and pixel max -> world height = <size>.z,
    so the PNG carries the SHAPE and the .sdf carries the vertical scale (z_max - z_min).
    cost_map_publisher uses the identical convention, so sim terrain == planner map.
    """
    z_min, z_max = float(np.min(dem)), float(np.max(dem))
    span = (z_max - z_min) or 1.0
    norm = (dem - z_min) / span                       # 0..1, low terrain -> black
    # PNG row 0 is the TOP; world +Y is UP. Flip so north stays north in Gazebo.
    norm = np.flipud(norm)
    if bits == 16:
        arr = np.round(norm * 65535).astype(np.uint16)
        Image.fromarray(arr, mode="I;16").save(png_path)
    else:
        arr = np.round(norm * 255).astype(np.uint8)
        Image.fromarray(arr, mode="L").save(png_path)
    return z_min, z_max


def nearest_side(n_hint):
    """Pick the (2^n)+1 side closest to (and >=, when possible) the hint."""
    for s in _VALID_SIDES:
        if s >= n_hint:
            return s
    return _VALID_SIDES[-1]


# --------------------------------------------------------------------------------------
# World SDF generation
# --------------------------------------------------------------------------------------
_SDF_TEMPLATE = """<?xml version="1.0" ?>
<!--
  {name}.sdf  -- AUTO-GENERATED by compa_slam/scripts/map_to_sim.py
  Gazebo world reconstructed from a recorded map.
  Heightmap: {png}
  Extent: {size_x:.3f} x {size_y:.3f} m,  vertical scale (z): {size_z:.3f} m
  Resolution: {res:.4f} m/pixel ({side}x{side} samples)
  Regenerate, don't hand-edit. World name kept 'empty' so the existing ros_gz bridge
  ( /world/empty/... ) and gazebo_bridge_slam.yaml work unchanged.
-->
<sdf version="1.10">
  <world name="empty">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Contact" filename="gz-sim-contact-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
    <!-- Needed if you also run the D455 (IMU sensor) in this world for live SLAM. -->
    <plugin name="gz::sim::systems::Imu" filename="gz-sim-imu-system"/>

    <gravity>0 0 -9.8</gravity>
    <magnetic_field>5.5645e-06 2.28758e-05 -4.23884e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.7 0.8 0.9 1</background>
      <shadows>true</shadows>
    </scene>

    <light name="sun" type="directional">
      <pose>0 0 20 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.5 0.1 -0.9</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>

    <!-- Reconstructed terrain. Centred at origin; lowest point sits at world z={z_min:.3f}. -->
    <model name="recorded_terrain">
      <static>true</static>
      <pose>0 0 {z_min:.4f} 0 0 0</pose>
      <link name="terrain_link">
        <visual name="terrain_visual">
          <geometry>
            <heightmap>
              <use_terrain_paging>true</use_terrain_paging>
              <uri>{png_uri}</uri>
              <size>{size_x:.4f} {size_y:.4f} {size_z:.4f}</size>
              <pos>0 0 0</pos>
              <texture>
                <diffuse>file://hamr_bringup/terrain_assets/textures/checker.png</diffuse>
                <normal>file://hamr_bringup/terrain_assets/textures/checker.png</normal>
                <size>2.0</size>
              </texture>
            </heightmap>
          </geometry>
        </visual>
        <collision name="terrain_collision">
          <geometry>
            <heightmap>
              <uri>{png_uri}</uri>
              <size>{size_x:.4f} {size_y:.4f} {size_z:.4f}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
"""


def heightmap_uri(png_path):
    """
    Prefer a package-relative gz URI ( file://compa_slam/maps/<png> ) so Gazebo resolves
    it via GZ_SIM_RESOURCE_PATH -- works both from the source tree and after install,
    exactly like the stock worlds' file://hamr_bringup/... references. Falls back to an
    absolute file:// path if the PNG isn't under a compa_slam/maps/ directory.
    """
    png_abs = os.path.abspath(png_path).replace("\\", "/")
    parts = png_abs.split("/")
    if len(parts) >= 3 and parts[-3] == "compa_slam" and parts[-2] == "maps":
        return "file://compa_slam/maps/" + parts[-1]
    return "file://" + png_abs


def write_world(sdf_path, name, png_path, side, size_x, size_y, size_z, res, z_min):
    with open(sdf_path, "w") as f:
        f.write(_SDF_TEMPLATE.format(
            name=name, png=os.path.basename(png_path),
            png_uri=heightmap_uri(png_path),
            side=side, size_x=size_x, size_y=size_y, size_z=size_z,
            res=res, z_min=z_min))


def write_yaml(yaml_path, name, png_path, side, size_x, size_y, size_z,
               res, z_min, z_max, origin_x, origin_y):
    png_abs = os.path.abspath(png_path).replace("\\", "/")
    with open(yaml_path, "w") as f:
        f.write(
            f"# {name} -- terrain metadata for compa_slam replay_map_sim.launch.py\n"
            f"# Auto-generated by map_to_sim.py. The launch reads this to configure\n"
            f"# Gazebo + cost_map_publisher so /elevation_map and /costmap match the world.\n"
            f"name: {name}\n"
            f"heightmap: {png_abs}\n"
            f"map_width_m: {size_x:.6f}      # world & grid_map X extent (size.x)\n"
            f"map_length_m: {size_y:.6f}     # world & grid_map Y extent (size.y)\n"
            f"map_height_m: {size_z:.6f}     # vertical scale: white pixel -> this many m\n"
            f"resolution: {res:.6f}          # m / pixel\n"
            f"side: {side}                   # PNG is side x side, (2^n)+1\n"
            f"z_min: {z_min:.6f}             # world z of the lowest terrain point\n"
            f"z_max: {z_max:.6f}\n"
            f"origin_x: {origin_x:.6f}       # world XY of the DEM bottom-left corner\n"
            f"origin_y: {origin_y:.6f}\n")


# --------------------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------------------
def _default_outdir():
    here = os.path.dirname(os.path.abspath(__file__))
    maps = os.path.normpath(os.path.join(here, "..", "maps"))
    return maps if os.path.isdir(maps) else os.getcwd()


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Convert a recorded map into a Gazebo heightmap world + planner DEM.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--cloud", help="point cloud (.ply/.pcd/.xyz) from rtabmap-export")
    src.add_argument("--db", help="RTAB-Map .db (runs rtabmap-export to get a cloud)")
    src.add_argument("--dem", help="existing grayscale DEM image to reuse directly")
    p.add_argument("--name", required=True, help="output basename, e.g. compa_real")
    p.add_argument("--outdir", default=None, help="output dir (default: package maps/)")
    p.add_argument("--size", type=float, default=None,
                   help="square world side in meters (default: cloud XY span +5%%)")
    p.add_argument("--height", type=float, default=None,
                   help="vertical scale in meters (default: cloud z span; required for --dem)")
    p.add_argument("--side", type=int, default=None,
                   help="heightmap pixels per side; snapped to (2^n)+1 (default: from resolution)")
    p.add_argument("--res", type=float, default=0.05,
                   help="target resolution m/pixel (used when --side omitted)")
    p.add_argument("--agg", choices=("max", "mean"), default="max",
                   help="per-cell height aggregation (max=top surface)")
    p.add_argument("--zclip", type=float, nargs=2, default=(0.5, 99.9),
                   metavar=("LO", "HI"),
                   help="drop points below/above these z percentiles (SLAM outliers). "
                        "Lower HI (e.g. 99) if a ceiling/flyaway points inflate the height.")
    p.add_argument("--bits", type=int, choices=(8, 16), default=8,
                   help="PNG bit depth (8 matches cost_map_publisher exactly)")
    args = p.parse_args(argv)

    outdir = args.outdir or _default_outdir()
    os.makedirs(outdir, exist_ok=True)
    png_path = os.path.join(outdir, f"{args.name}_heightmap.png")
    yaml_path = os.path.join(outdir, f"{args.name}.yaml")
    sdf_path = os.path.join(outdir, f"{args.name}.sdf")

    if args.dem:
        if args.height is None or args.size is None:
            sys.exit("ERROR: --dem requires both --size and --height (the DEM has no scale).")
        dem = dem_from_image(args.dem, args.height)
        side = nearest_side(args.side or max(dem.shape))
        if dem.shape != (side, side):
            # PIL float mode ('F') is 32-bit; cast so resize never rejects float64.
            resized = Image.fromarray(dem.astype(np.float32), mode="F").resize(
                (side, side), Image.BILINEAR)
            dem = np.asarray(resized, dtype=np.float64)
        size_x = size_y = args.size
        res = args.size / (side - 1)
        origin_x = origin_y = -args.size / 2.0
    else:
        cloud_path = args.cloud
        if args.db:
            cloud_path = export_db_to_ply(args.db, os.path.join(outdir, args.name))
        print(f"[map_to_sim] reading cloud: {cloud_path}")
        pts = read_cloud(cloud_path)
        print(f"[map_to_sim] {pts.shape[0]:,} points; "
              f"x[{pts[:,0].min():.2f},{pts[:,0].max():.2f}] "
              f"y[{pts[:,1].min():.2f},{pts[:,1].max():.2f}] "
              f"z[{pts[:,2].min():.2f},{pts[:,2].max():.2f}]")
        # Choose side from requested resolution, then snap to a valid Gazebo size.
        span = args.size or max(pts[:, 0].max() - pts[:, 0].min(),
                                pts[:, 1].max() - pts[:, 1].min()) * 1.05
        side = nearest_side(args.side or int(round(span / args.res)) + 1)
        dem, meta = cloud_to_dem(pts, side, size=args.size, agg=args.agg,
                                 z_clip=tuple(args.zclip))
        size_x = size_y = meta["size"]
        res = meta["resolution"]
        origin_x, origin_y = meta["origin_x"], meta["origin_y"]
        if args.height is not None:                 # let user clamp the vertical scale
            dem = np.clip(dem, dem.min(), dem.min() + args.height)

    z_min, z_max = dem_to_png(dem, png_path, bits=args.bits)
    size_z = (z_max - z_min) or 1.0

    write_yaml(yaml_path, args.name, png_path, side, size_x, size_y, size_z,
               res, z_min, z_max, origin_x, origin_y)
    write_world(sdf_path, args.name, png_path, side, size_x, size_y, size_z, res, z_min)

    print("\n[map_to_sim] DONE")
    print(f"  heightmap : {png_path}   ({side}x{side}, {args.bits}-bit)")
    print(f"  metadata  : {yaml_path}")
    print(f"  world     : {sdf_path}")
    print(f"  extent    : {size_x:.2f} x {size_y:.2f} m,  z-scale {size_z:.2f} m,"
          f"  res {res:.3f} m/px")
    print(f"\nRun it:\n  ros2 launch compa_slam replay_map_sim.launch.py map:={args.name}")


if __name__ == "__main__":
    main()
