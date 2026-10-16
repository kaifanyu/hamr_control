"""Physical geometry contracts for exact ledges and support probes."""
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from generate_rocky_terrains import stone, sphere_support, surface_height, validate_component


def box():
    from scipy.spatial import ConvexHull
    v=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (0.,.1)])
    hull=ConvexHull(v);f=hull.simplices.copy();tri=v[f]
    wrong=np.sum(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])*hull.equations[:,:3],axis=1)<0
    f[wrong]=f[wrong][:,[0,2,1]]
    return v,f


def test_sphere_support_on_step_top_edge_and_corner():
    v,f=box();validate_component(v,f)
    assert sphere_support(v,f,[0,0],.04)==pytest.approx(.14)
    assert sphere_support(v,f,[.52,0],.04)==pytest.approx(.1+np.sqrt(.04**2-.02**2))
    assert sphere_support(v,f,[.52,.52],.04)==pytest.approx(.1+np.sqrt(.04**2-2*.02**2))
    assert surface_height(v,f,0,0)==pytest.approx(.1)
    assert np.isneginf(surface_height(v,f,.6,0))


def test_support_on_tilted_surface_is_not_height_plus_radius():
    v,f=box();v[:,2]+=.25*v[:,0]
    assert sphere_support(v,f,[0,0],.04)==pytest.approx(.1+.04*np.sqrt(1+.25**2))


def test_stone_is_closed_and_keeps_vertical_faces():
    v,f,info=stone(0,0,.4,.3,.3,.03,lambda x,y:.01,np.random.default_rng(1))
    validate_component(v,f)
    normals=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
    assert (abs(normals[:,2])<1e-12).any()
    x,y=.0,.0
    assert surface_height(v,f,x,y)==pytest.approx(info['top_center_z_m'])
    gradient=np.array(info['top_gradient'])
    assert sphere_support(v,f,[x,y],.04)==pytest.approx(info['top_center_z_m']+.04*np.sqrt(1+np.dot(gradient,gradient)))
