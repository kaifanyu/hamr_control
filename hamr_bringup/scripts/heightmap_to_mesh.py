#!/usr/bin/env python3
"""Convert a heightmap image into a textured OBJ mesh for Gazebo.

Why this exists - gz-sim's ogre2 backend only supports heightmap geometry
for regular RGB cameras. Depth/RGBD cameras (and GPU lidar) crash or see
nothing when a <heightmap> visual is in the scene (gz-rendering #712/#968).
The workaround is to render terrain from a mesh visual while keeping the
cheap <heightmap> collision for physics.

Usage:
    python3 heightmap_to_mesh.py <heightmap.png> <out.obj> \
        --size-x 60 --size-y 60 --size-z 1.5 \
        --grid 257 --texture ../textures/checker.png --tex-repeat 2.0

The output OBJ is centered at (0,0), spans [-size/2, +size/2], with z from
0 to size_z, matching gz <heightmap><size>X Y Z</size> with <pos>0 0 0</pos>.
Image row 0 maps to +y (Gazebo heightmap convention). UVs repeat the texture
every --tex-repeat meters, like <texture><size> in the heightmap SDF.
"""
import argparse
import os

import cv2
import numpy as np


def heightmap_to_mesh(png_path, obj_path, size_x, size_y, size_z,
                      grid, texture, tex_repeat):
    img = cv2.imread(png_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(png_path)
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    max_val = np.iinfo(img.dtype).max if img.dtype.kind == 'u' else 1.0
    h = img.astype(np.float64) / float(max_val)

    h = cv2.resize(h, (grid, grid), interpolation=cv2.INTER_AREA)

    n = grid
    xs = np.linspace(-size_x / 2.0, size_x / 2.0, n)
    ys = np.linspace(size_y / 2.0, -size_y / 2.0, n)  # row 0 -> +y
    xx, yy = np.meshgrid(xs, ys)
    zz = h * size_z

    # per-vertex normals from height gradients
    dz_dy, dz_dx = np.gradient(zz, ys, xs)
    nx, ny, nz = -dz_dx, -dz_dy, np.ones_like(zz)
    norm = np.sqrt(nx * nx + ny * ny + nz * nz)
    nx, ny, nz = nx / norm, ny / norm, nz / norm

    # texture repeats every tex_repeat meters
    uu = (xx + size_x / 2.0) / tex_repeat
    vv = (yy + size_y / 2.0) / tex_repeat

    mtl_path = os.path.splitext(obj_path)[0] + '.mtl'
    mtl_name = os.path.basename(os.path.splitext(obj_path)[0]) + '_mat'
    with open(mtl_path, 'w') as f:
        f.write(f"newmtl {mtl_name}\n")
        f.write("Ka 1.0 1.0 1.0\nKd 1.0 1.0 1.0\nKs 0.0 0.0 0.0\n")
        if texture:
            f.write(f"map_Kd {texture}\n")

    lines = [f"mtllib {os.path.basename(mtl_path)}\n"]
    v = np.stack([xx, yy, zz], axis=-1).reshape(-1, 3)
    vn = np.stack([nx, ny, nz], axis=-1).reshape(-1, 3)
    vt = np.stack([uu, vv], axis=-1).reshape(-1, 2)
    lines += [f"v {a:.4f} {b:.4f} {c:.4f}\n" for a, b, c in v]
    lines += [f"vt {a:.4f} {b:.4f}\n" for a, b in vt]
    lines += [f"vn {a:.4f} {b:.4f} {c:.4f}\n" for a, b, c in vn]
    lines.append(f"usemtl {mtl_name}\n")

    # two triangles per grid cell, CCW seen from +z
    idx = np.arange(n * n).reshape(n, n)
    a = idx[:-1, :-1].ravel() + 1  # OBJ indices are 1-based
    b = idx[:-1, 1:].ravel() + 1
    c = idx[1:, :-1].ravel() + 1
    d = idx[1:, 1:].ravel() + 1
    # row 0 is +y, so winding a->c->b / b->c->d faces up
    for t1, t2, t3 in zip(a, c, b):
        lines.append(f"f {t1}/{t1}/{t1} {t2}/{t2}/{t2} {t3}/{t3}/{t3}\n")
    for t1, t2, t3 in zip(b, c, d):
        lines.append(f"f {t1}/{t1}/{t1} {t2}/{t2}/{t2} {t3}/{t3}/{t3}\n")

    with open(obj_path, 'w') as f:
        f.writelines(lines)
    print(f"wrote {obj_path} ({n}x{n} grid, {2 * (n - 1) ** 2} tris) "
          f"and {mtl_path}")


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('png')
    p.add_argument('obj')
    p.add_argument('--size-x', type=float, default=60.0)
    p.add_argument('--size-y', type=float, default=60.0)
    p.add_argument('--size-z', type=float, default=1.5)
    p.add_argument('--grid', type=int, default=257)
    p.add_argument('--texture', default='../textures/checker.png',
                   help='path written into the .mtl, relative to the .obj')
    p.add_argument('--tex-repeat', type=float, default=2.0)
    args = p.parse_args()
    heightmap_to_mesh(args.png, args.obj, args.size_x, args.size_y,
                      args.size_z, args.grid, args.texture, args.tex_repeat)
