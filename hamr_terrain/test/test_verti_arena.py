"""Physical scale, source provenance, unknown-area mask, and mesh contracts."""
import json
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from generate_verti_arena import IDS,LEVELS,digest,load_scan
from generate_full_uneven_terrains import surface_statistics
from run_terrain_replay import outside_terrain


def test_metric_source_and_checksums():
    z,rgb,valid,gap,meta,provenance=load_scan(ROOT/'assets/verti_arena')
    assert z.shape==(787,1427) and rgb.shape==(*z.shape,3)
    assert float(meta['resolution'])==pytest.approx(.008)
    assert .5<valid.mean()<.6
    assert np.isfinite(z).all() and gap.max()>0
    assert np.all(gap[valid]==0)
    assert 0<=rgb.min()<=rgb.max()<=1
    assert len(provenance['commit'])==40


@pytest.mark.parametrize('name,level',zip(IDS,LEVELS))
def test_exported_geometry_and_provenance(name,level):
    meta=json.loads((ROOT/'config'/f'{name}.json').read_text())
    grid=np.load(ROOT/'meshes'/f'{name}_height.npz')
    x,y,z=(grid[k] for k in ('x','y','z'))
    assert z.shape==(321,321)
    np.testing.assert_allclose([x[0],x[-1],y[0],y[-1]],[-4,4,-4,4])
    assert np.ptp(z)==pytest.approx(level[1]) and z.min()==0
    stats=surface_statistics(x,y,z)
    assert stats['horizontal_surface_triangles']==0
    assert stats['minimum_0_4m_patch_relief_m']>0
    unknown=grid['inferred_weight']
    assert unknown.shape==z.shape and 0<unknown.mean()<.2
    assert np.all((unknown>=0)&(unknown<=1+1e-12))
    assert meta['generation']['composed_inferred_weight_fraction']==pytest.approx(unknown.mean())
    raw=(ROOT/'meshes'/f'{name}.stl').read_bytes()
    count=struct.unpack_from('<I',raw,80)[0]
    rec=np.frombuffer(raw,offset=84,count=count,dtype=[('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')])
    points=rec['v'][:stats['surface_triangles']].reshape(-1,3)
    ix=np.rint((points[:,0]+4)/.025).astype(int);iy=np.rint((points[:,1]+4)/.025).astype(int)
    np.testing.assert_allclose(points[:,2],z[iy,ix],atol=4e-8,rtol=0)
    assert np.all(rec['n'][:stats['surface_triangles'],2]>0)
    dae=ET.parse(ROOT/'meshes'/f'{name}.dae');ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    visual=np.fromstring(dae.find('.//c:float_array[@id="positions_array"]',ns).text,sep=' ').reshape(-1,3)
    np.testing.assert_allclose(visual[:z.size,2],z.ravel(),atol=5.1e-8,rtol=0)
    assert dae.find('.//c:image/c:init_from',ns).text=='verti_arena_rgb.png'
    assert digest(ROOT/'meshes/verti_arena_rgb.png')==meta['texture_sha256']
    world=ET.parse(ROOT/'worlds'/f'{name}.sdf').getroot().find('world')
    assert not world.findall('.//plane')
    assert len(world.findall('.//collision[geometry]'))==1
    assert not any(m.get('name','').startswith('apron') for m in world.findall('model'))
    assert world.find('physics/max_step_size').text=='0.001'
    assert world.find('gravity').text=='0 0 -9.80665'


def test_levels_share_xy_pattern():
    grids=[np.load(ROOT/'meshes'/f'{name}_height.npz')['z']/level[1] for name,level in zip(IDS,LEVELS)]
    for z in grids[1:]:np.testing.assert_allclose(z,grids[0],atol=2e-16)


def test_replay_respects_actual_map_boundary():
    meta={'bounds_m':{'x':[-4,4],'y':[-4,4]}}
    assert not outside_terrain([3.5,0,.1],meta)
    assert outside_terrain([4.01,0,.1],meta)
    assert outside_terrain([0,-4.01,.1],meta)
    assert outside_terrain([3.5,0,.1],{})
