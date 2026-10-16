"""Validate full-surface relief and preserve the original replay campaign."""
import json
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from generate_full_uneven_terrains import IDS,LEVELS,height_fields,surface_statistics
from terrain_replay_paths import Reference
from terrain_simulation_check import probe_specs
from run_terrain_replay_suite import matrix
from build_terrain_replays import build_gallery


@pytest.mark.parametrize('name,level',zip(IDS,LEVELS))
def test_entire_exported_surface_is_uneven_and_bounded(name,level):
    metadata=json.loads((ROOT/'config'/f'{name}.json').read_text())
    grid=np.load(ROOT/'meshes'/f'{name}_height.npz')
    x,y,z=(grid[key] for key in ('x','y','z'))
    stats=surface_statistics(x,y,z)
    assert z.shape==(321,241)
    assert 0<=z.min()<=z.max()<=level[1]+1e-12
    assert np.ptp(z)==pytest.approx(level[1])
    assert stats['max_surface_slope_deg']<=level[2]
    assert stats['horizontal_surface_triangles']==0
    assert stats['minimum_0_4m_patch_relief_m']>.001
    for edge in (z[0],z[-1],z[:,0],z[:,-1]):assert np.ptp(edge)>.003
    assert level[1]<.25*metadata['height_basis']['neutral_main_chassis_clearance_m']
    assert level[1]<.6*metadata['height_basis']['swivel_wheel_radius_m']
    raw=(ROOT/'meshes'/f'{name}.stl').read_bytes()
    count=struct.unpack_from('<I',raw,80)[0]
    rec=np.frombuffer(raw,offset=84,count=count,dtype=[('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')])
    assert count==156960
    assert np.all(rec['n'][:stats['surface_triangles'],2]>0)
    # Both vertices of every top triangle are sampled from this exact height grid.
    vertices=rec['v'][:stats['surface_triangles']].reshape(-1,3)
    ix=np.rint((vertices[:,0]+3)/.025).astype(int)
    iy=np.rint((vertices[:,1]+4)/.025).astype(int)
    np.testing.assert_allclose(vertices[:,2],z[iy,ix],atol=4e-9,rtol=0)
    tree=ET.parse(ROOT/'worlds'/f'{name}.sdf')
    world=tree.getroot().find('world')
    assert not tree.findall('.//plane')
    assert not any(m.get('name').startswith(('apron_','marker_')) for m in world.findall('model'))
    collisions=world.findall('.//collision[geometry]')
    assert len(collisions)==1
    assert collisions[0].find('geometry/mesh/uri').text.endswith(f'{name}.stl')
    assert world.find('physics/max_step_size').text=='0.001'
    assert world.find('gravity').text=='0 0 -9.80665'
    assert collisions[0].find('surface/friction/ode/mu').text=='1.0'
    assert len(probe_specs(metadata,grid))==5
    assert all(p['kind']=='guided_drop' for p in probe_specs(metadata,grid))


def test_levels_are_matched_and_deterministic():
    x,y,z,_=height_fields()
    np.testing.assert_array_equal(z,height_fields()[2])
    for name,(_,height,_) in zip(IDS,LEVELS):
        grid=np.load(ROOT/'meshes'/f'{name}_height.npz')
        np.testing.assert_allclose(grid['z'],z*height,atol=1e-15)


def test_weave_has_new_shape_and_room_for_vehicle():
    ref=Reference('weave')
    assert ref.points.shape==(501,2)
    assert ref.points[:,0].min()==pytest.approx(-2.1)
    assert ref.points[:,0].max()==pytest.approx(2.1)
    assert np.all(np.diff(ref.points[:,1])>0)
    np.testing.assert_allclose(ref.points[[0,-1]],[[0,-3.2],[0,3.2]],atol=1e-14)
    assert 6.4<ref.length<12


def test_selected_matrix_and_gallery(tmp_path):
    manifest=ROOT/'config/full_uneven_terrains.json'
    cases=matrix(tmp_path,manifest,('weave',))
    assert len(cases)==6
    assert {c['terrain'] for c in cases}==set(IDS)
    assert all(c['shape']=='weave' for c in cases)
    report=build_gallery(tmp_path)
    assert report['expected']==6 and not report['complete']
    page=(tmp_path/'index.html').read_text()
    assert '3 terrains × 1 paths' in page and page.count('<h3>Weave</h3>')==3
    assert '<h3>Circle</h3>' not in page
