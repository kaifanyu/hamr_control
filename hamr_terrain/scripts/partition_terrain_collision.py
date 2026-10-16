#!/usr/bin/env python3
"""Partition a terrain collision STL spatially, preserving every triangle byte.

Tiles contain original surface triangles and existing exterior walls only.
The original bottom is separate. No internal seam walls, interpolation,
resampling, new triangles, or physical parameter changes are introduced.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np

STL_RECORD = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)),
                       ('attribute', '<u2')])


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_collision_tiles(source_mesh, output_dir, *, top_count, tile_size=8., bounds=None):
    """Write byte-exact collision parts and return their manifest.

    ``top_count`` is the original leading surface-triangle count. Optional
    bounds are ``(xmin, xmax, ymin, ymax)``; otherwise use source mesh bounds.
    Existing triangle edges must align with requested tile boundaries.
    """
    source_mesh = Path(source_mesh).resolve()
    output_dir = Path(output_dir).resolve()
    if not math.isfinite(tile_size) or tile_size <= 0:
        raise ValueError('tile_size must be finite and positive')
    data = source_mesh.read_bytes()
    if len(data) < 84:
        raise ValueError('Expected a binary STL with a complete header')
    count = struct.unpack('<I', data[80:84])[0]
    if len(data) != 84+50*count or not count:
        raise ValueError('Binary STL length does not match its triangle count')
    if not isinstance(top_count, (int, np.integer)) or not 0 < top_count <= count:
        raise ValueError('top_count must identify the original leading surface triangles')
    records = np.frombuffer(data, dtype=STL_RECORD, offset=84, count=count)
    triangles = records['vertices']
    if not np.isfinite(triangles).all() or not np.isfinite(records['normal']).all():
        raise ValueError('Terrain STL contains nonfinite coordinates or normals')
    minimum, maximum = triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))
    if bounds is None:
        bounds = (float(minimum[0]), float(maximum[0]), float(minimum[1]), float(maximum[1]))
    bounds = np.asarray(bounds, dtype=float)
    if (bounds.shape != (4,) or not np.isfinite(bounds).all()
            or bounds[1] <= bounds[0] or bounds[3] <= bounds[2]):
        raise ValueError('bounds must be finite increasing xmin,xmax,ymin,ymax')
    if not np.allclose(bounds, [minimum[0], maximum[0], minimum[1], maximum[1]], rtol=0, atol=1e-6):
        raise ValueError('Partition bounds must retain the entire source terrain')
    nx, ny = (int(math.ceil((bounds[1]-bounds[0])/tile_size)),
              int(math.ceil((bounds[3]-bounds[2])/tile_size)))
    centers = triangles.astype(float).mean(axis=1)
    bottom = ((np.arange(count) >= top_count)
              & np.all(triangles[:, :, 2] == minimum[2], axis=1)
              & (records['normal'][:, 2] < -.99))
    column = np.clip(np.floor((centers[:, 0]-bounds[0])/tile_size).astype(int), 0, nx-1)
    row = np.clip(np.floor((centers[:, 1]-bounds[2])/tile_size).astype(int), 0, ny-1)
    groups = []
    for j in range(ny):
        for i in range(nx):
            indices = np.flatnonzero((column == i) & (row == j) & ~bottom)
            if not len(indices):
                continue
            low = np.array([bounds[0]+i*tile_size, bounds[2]+j*tile_size])
            high = np.minimum(low+tile_size, bounds[[1, 3]])
            xy = triangles[indices, :, :2]
            if np.any(xy < low-1e-5) or np.any(xy > high+1e-5):
                raise ValueError('A source triangle crosses a tile boundary; choose a tile size aligned with native triangle edges')
            groups.append((f'terrain_tile_{j:02}_{i:02}', indices))
    if np.any(bottom):
        groups.append(('terrain_bottom', np.flatnonzero(bottom)))
    used = np.concatenate([indices for _, indices in groups])
    if not np.array_equal(np.sort(used), np.arange(count)):
        raise ValueError('Partition must assign every original triangle exactly once')
    # This checks the source union, not the open individual collision pieces.
    edges = Counter()
    for triangle in triangles:
        for first, last in zip(triangle, (triangle[1], triangle[2], triangle[0])):
            edges[tuple(sorted((first.tobytes(), last.tobytes())))] += 1
    closed = all(value == 2 for value in edges.values())
    output_dir.mkdir(parents=True, exist_ok=True)
    entries = []
    for name, indices in groups:
        path = output_dir/f'{name}.stl'
        selected = records[indices]
        path.write_bytes(data[:80]+struct.pack('<I', len(indices))+selected.tobytes())
        entries.append({'name': name, 'mesh': str(path), 'triangles': len(indices),
                        'top_triangles': int(np.count_nonzero(indices < top_count)),
                        'original_triangle_indices': indices.tolist(), 'sha256': digest(path),
                        'aabb_min': selected['vertices'].min(axis=(0, 1)).astype(float).tolist(),
                        'aabb_max': selected['vertices'].max(axis=(0, 1)).astype(float).tolist()})
    manifest = {'tile_size_m': float(tile_size), 'collision_shapes': len(entries),
                'source_mesh': str(source_mesh), 'source_mesh_sha256': hashlib.sha256(data).hexdigest(),
                'triangles': count, 'surface_triangles': int(top_count),
                'bounds_m': {'x': bounds[:2].tolist(), 'y': bounds[2:].tolist()},
                'source_triangle_multiset_preserved_exactly': True, 'new_internal_walls': False,
                'closed_individual_tiles': False, 'globally_closed_original_union': closed,
                'contact_note': 'Collision grouping changes at tile seams. Original triangle geometry and material values remain unchanged; runtime seam checks are required.',
                'entries': entries}
    (output_dir/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-mesh', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--top-count', type=int, required=True)
    parser.add_argument('--tile-size', type=float, default=8.)
    args = parser.parse_args()
    manifest = write_collision_tiles(args.source_mesh, args.output,
                                     top_count=args.top_count, tile_size=args.tile_size)
    print(json.dumps({key: value for key, value in manifest.items() if key != 'entries'}, indent=2))


if __name__ == '__main__':
    main()
