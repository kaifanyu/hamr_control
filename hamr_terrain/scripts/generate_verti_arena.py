#!/usr/bin/env python3
"""Build full-area off-road worlds from a pinned public Verti-Arena scan.

These are remixed measured patches, not a reconstruction of the paper's layout.
The missing-data mask and every resampling/height transform are retained.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, gaussian_filter, map_coordinates, median_filter

import generate_terrains as geometry
from generate_adtg_terrains import triangle_height
from generate_full_uneven_terrains import surface_statistics

REVISION = '2a82faceacf8eaf1a8564b05058062e7584b0f45'
LEVELS = (('low', .15), ('medium', .35), ('full', .70))
IDS = tuple('verti_arena_'+level for level, _ in LEVELS)
# 4.4 m patches overlap by 0.8 m. Original horizontal scale is retained.
PANELS = ((900, 430, 0, -1.8, -1.8), (1030, 420, 1, 1.8, -1.8),
          (800, 430, 2, -1.8, 1.8), (650, 430, 3, 1.8, 1.8))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_scan(source):
    source = Path(source)
    provenance = json.loads((source/'provenance.json').read_text())
    if provenance['commit'] != REVISION:
        raise ValueError('Unexpected scan revision')
    for entry in provenance['files']:
        if digest(source/entry['file']) != entry['sha256']:
            raise ValueError('Source checksum mismatch: '+entry['file'])
    meta = dict(line.split(maxsplit=1) for line in (source/'fused_elevation_map_meta.txt').read_text().splitlines())
    z = np.load(source/'fused_elevation_map.npy', allow_pickle=False)
    rgb = np.load(source/'fused_rgb_map.npy', allow_pickle=False)
    if z.shape != (int(meta['height']), int(meta['width'])) or rgb.shape != (*z.shape, 3):
        raise ValueError('Source dimensions do not match metadata')
    valid = np.isfinite(z)
    color_valid = np.isfinite(rgb).all(axis=2)
    if valid.mean() < .4 or color_valid.mean() < .4:
        raise ValueError('Insufficient valid scan data')
    gap, nearest = distance_transform_edt(~valid, return_indices=True)
    filled = z[tuple(nearest)]
    _, color_nearest = distance_transform_edt(~color_valid, return_indices=True)
    colors = rgb[tuple(color_nearest)]
    meta['rgb_valid_fraction'] = str(float(color_valid.mean()))
    # Suppress isolated reconstruction speckle at 40 mm, then a 12 mm sigma.
    filtered = gaussian_filter(median_filter(filled, size=5), sigma=1.5)
    return filtered, colors, valid, gap*float(meta['resolution']), meta, provenance


def compose(source):
    scan, colors, valid, gap, meta, provenance = load_scan(source)
    resolution = float(meta['resolution'])
    x = y = np.linspace(-4., 4., 321)
    xx, yy = np.meshgrid(x, y)
    zsum = np.zeros_like(xx); wsum = np.zeros_like(xx)
    csum = np.zeros((*xx.shape, 3)); inferred = np.zeros_like(xx)
    gap_sum = np.zeros_like(xx); panels = []
    for cx, cy, rotation, tx, ty in PANELS:
        u, v = xx-tx, yy-ty
        # Cosine overlap; only panel joins are blended, no flat edge taper.
        weight = np.ones_like(xx)
        for coordinate in (u, v):
            edge = np.clip((2.2-np.abs(coordinate))/.8, 0., 1.)
            weight *= .5-.5*np.cos(np.pi*edge)
        # Exterior boundary belongs to its adjacent panel; retain its full relief.
        weight[(np.abs(u)>2.2+1e-9)|(np.abs(v)>2.2+1e-9)] = 0.
        angle = rotation*np.pi/2
        sx = cx+(np.cos(angle)*u-np.sin(angle)*v)/resolution
        sy = cy+(np.sin(angle)*u+np.cos(angle)*v)/resolution
        coords = np.array([sy, sx])
        sampled = map_coordinates(scan, coords, order=1, mode='nearest')
        zsum += weight*sampled; wsum += weight
        for channel in range(3):
            csum[:,:,channel] += weight*map_coordinates(colors[:,:,channel], coords, order=1, mode='nearest')
        inferred += weight*map_coordinates((~valid).astype(float), coords, order=1, mode='nearest')
        gap_sum += weight*map_coordinates(gap, coords, order=1, mode='nearest')
        mask = weight>1e-9
        panels.append({'source_center_pixel_xy':[cx,cy], 'rotation_ccw_deg':rotation*90,
                       'target_center_xy_m':[tx,ty], 'side_m':4.4,
                       'maximum_gap_filled_m':float(map_coordinates(gap,coords,order=1,mode='nearest')[mask].max())})
    # At the outside endpoints both weight and sum are zero. Sample epsilon
    # inside the boundary through nearest nonzero grid values, then add relief.
    missing = wsum < 1e-12
    _, nearest = distance_transform_edt(missing, return_indices=True)
    wsum = wsum[tuple(nearest)]
    zz = zsum[tuple(nearest)]/wsum
    rgb = csum[tuple(nearest)]/wsum[:,:,None]
    inferred = inferred[tuple(nearest)]/wsum
    gap_sum = gap_sum[tuple(nearest)]/wsum
    # Explicit synthetic microrelief: prevents a flat scanned floor patch from
    # becoming a flat driving lane. It is not represented as measured terrain.
    micro = (.0018*np.sin(14.3*xx+8.1*yy+.3)
             + .0013*np.sin(7.7*xx-17.1*yy+1.1)
             + .0009*np.cos(25.4*xx+6.2*yy))
    zz += micro
    raw_range = float(np.ptp(zz))
    normalized = (zz-zz.min())/raw_range
    info = {'source_commit':REVISION, 'source_grid_shape':list(scan.shape),
            'source_resolution_m':resolution, 'source_origin_xy_m':[float(meta['origin_x']),float(meta['origin_y'])],
            'source_valid_fraction':float(valid.mean()),
            'source_rgb_valid_fraction':float(meta['rgb_valid_fraction']),
            'source_array_extent_m':[(scan.shape[1]-1)*resolution,(scan.shape[0]-1)*resolution],
            'panels':panels, 'blend_width_m':.8,
            'composed_inferred_weight_fraction':float(inferred.mean()),
            'maximum_blended_gap_fill_distance_m':float(gap_sum.max()),
            'pre_scaling_height_range_m':raw_range,
            'filter':'5x5 median (40 mm), Gaussian sigma 1.5 pixels (12 mm); 25 mm bilinear output',
            'microrelief':'Three deterministic sinusoids; summed amplitude bound 4 mm before level scaling',
            'height_transform':'Subtract composed minimum and scale linearly to each requested relief; no height clipping',
            'layout':'Four rotated measured 4.4 m patches, 0.8 m cosine overlap, new 8x8 m layout. Not the paper layout.',
            'coordinate_convention':'Array row increases with source odom Y; columns with X, following upstream StaticMap.world_to_grid.',
            'provenance':provenance}
    return x, y, normalized, rgb.clip(0,1), inferred, gap_sum, info


def textured_dae(path, vertices, faces, texture_name, bounds=((-4.,4.),(-4.,4.))):
    """UV-textured visual with exactly the same triangles as collision STL."""
    e = ET.SubElement
    doc=ET.Element('COLLADA',xmlns='http://www.collada.org/2005/11/COLLADASchema',version='1.4.1')
    asset=e(doc,'asset');e(asset,'unit',name='meter',meter='1');e(asset,'up_axis').text='Z_UP'
    im=e(e(doc,'library_images'),'image',id='terrain_image');e(im,'init_from').text=texture_name
    effect=e(e(doc,'library_effects'),'effect',id='terrain_effect');profile=e(effect,'profile_COMMON')
    surface=e(e(profile,'newparam',sid='surface'),'surface',type='2D');e(surface,'init_from').text='terrain_image'
    sampler=e(e(profile,'newparam',sid='sampler'),'sampler2D');e(sampler,'source').text='surface'
    lam=e(e(profile,'technique',sid='common'),'lambert')
    e(e(lam,'diffuse'),'texture',texture='sampler',texcoord='UVSET0')
    e(e(lam,'ambient'),'color').text='.45 .45 .45 1'
    material=e(e(doc,'library_materials'),'material',id='terrain_material');e(material,'instance_effect',url='#terrain_effect')
    mesh=e(e(e(doc,'library_geometries'),'geometry',id='terrain_mesh'),'mesh')
    tri=vertices[faces];normals=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);normals/=np.linalg.norm(normals,axis=1)[:,None]
    limits=np.asarray(bounds,dtype=float)
    if limits.shape!=(2,2) or np.any(limits[:,1]<=limits[:,0]):
        raise ValueError('Texture bounds must be increasing X and Y intervals')
    uv=(vertices[:,:2]-limits[:,0])/(limits[:,1]-limits[:,0])
    for name,values,axes in [('positions',vertices,'XYZ'),('normals',normals,'XYZ'),('uv',uv,'ST')]:
        source=e(mesh,'source',id=name)
        e(source,'float_array',id=name+'_array',count=str(values.size)).text=' '.join(f'{v:.7f}' for v in values.ravel())
        acc=e(e(source,'technique_common'),'accessor',source='#'+name+'_array',count=str(len(values)),stride=str(len(axes)))
        for axis in axes:e(acc,'param',name=axis,type='float')
    e(e(mesh,'vertices',id='vertices'),'input',semantic='POSITION',source='#positions')
    triangles=e(mesh,'triangles',count=str(len(faces)),material='terrain_material')
    for semantic,source,offset in [('VERTEX','#vertices','0'),('NORMAL','#normals','1'),('TEXCOORD','#uv','2')]:
        inp=e(triangles,'input',semantic=semantic,source=source,offset=offset)
        if semantic=='TEXCOORD':inp.set('set','0')
    indices=np.stack((faces,np.repeat(np.arange(len(faces))[:,None],3,axis=1),faces),axis=2)
    e(triangles,'p').text=' '.join(map(str,indices.ravel()))
    scene=e(e(doc,'library_visual_scenes'),'visual_scene',id='Scene')
    inst=e(e(scene,'node',id='terrain'),'instance_geometry',url='#terrain_mesh')
    bound=e(e(e(inst,'bind_material'),'technique_common'),'instance_material',symbol='terrain_material',target='#terrain_material')
    e(bound,'bind_vertex_input',semantic='UVSET0',input_semantic='TEXCOORD',input_set='0')
    e(e(doc,'scene'),'instance_visual_scene',url='#Scene')
    ET.ElementTree(doc).write(path,encoding='utf-8',xml_declaration=True)


def generate(output, source=None):
    output=Path(output).resolve()
    source=Path(source) if source else geometry.package_directory()/'assets/verti_arena'
    for folder in ('meshes','worlds','config'):(output/folder).mkdir(exist_ok=True,parents=True)
    x,y,normalized,rgb,inferred,gaps,info=compose(source)
    texture=output/'meshes/verti_arena_rgb.png'
    Image.fromarray(np.rint(np.flipud(rgb)*255).astype('uint8')).save(texture)
    entries=[]
    for level,relief in LEVELS:
        name='verti_arena_'+level;z=normalized*relief
        stats=surface_statistics(x,y,z)
        if stats['horizontal_surface_triangles'] or stats['minimum_0_4m_patch_relief_m']<=0:
            raise ValueError('Terrain contains flat driving areas')
        _,_,_,vertices,faces,top=geometry.mesh_from_height_grid(x,y,z)
        geometry.stl(output/'meshes'/f'{name}.stl',vertices,faces)
        textured_dae(output/'meshes'/f'{name}.dae',vertices,faces,texture.name)
        np.savez_compressed(output/'meshes'/f'{name}_height.npz',x=x,y=y,z=z,inferred_weight=inferred,gap_fill_distance_m=gaps)
        geometry.make_world(output,name,lambda px,py:triangle_height(x,y,z,px,py))
        path=output/'worlds'/f'{name}.sdf';tree=ET.parse(path);world=tree.getroot().find('world')
        for model in list(world.findall('model')):
            if model.get('name','').startswith(('apron_','marker_')):world.remove(model)
        for camera,eye,target in [('overview_camera',(9.,-10.,8.),(0.,0.,.2)),('detail_camera',(3.8,-4.6,1.8),(0.,.2,.25))]:
            world.find(f"model[@name='{camera}']/pose").text=' '.join(map(str,geometry.look_at(eye,target)))
        ET.indent(tree,space='  ');tree.write(path,encoding='utf-8',xml_declaration=True)
        spawn_mask=(np.abs(x[None,:])<.7)&(np.abs(y[:,None]+3.2)<.7)
        entry={'id':name,'title':f'Verti-Arena scan-derived {level}: {relief*100:.0f} cm relief',
               'description':'Full 8x8 m off-road terrain assembled from metric measured patches; new layout, rigid-ground approximation.',
               'world':f'../worlds/{name}.sdf','height_grid':f'../meshes/{name}_height.npz',
               'collision_mesh':f'../meshes/{name}.stl','visual_mesh':f'../meshes/{name}.dae',
               'bounds_m':{'x':[-4,4],'y':[-4,4]},'grid_spacing_m':.025,'level':level,
               'vertices':len(vertices),'triangles':len(faces),'peak_to_valley_limit_m':relief,**stats,
               'generation':info,'terrain_friction':{'mu':1.,'mu2':1.,'restitution':0.,'status':'Estimated uniform rigid contact, not calibrated from RGB or paper'},
               'spawn':{'x':0.,'y':-3.2,'z':float(z[spawn_mask].max()+.025),'yaw':float(np.pi/2)},
               'flat_friction_probes':False,
               'probes':[{'name':f'feature_{i}','xy_m':xy,'kind':'feature'} for i,xy in enumerate([[-2.6,-2.6],[2.3,-2.3],[0.,0.],[-2.3,2.3],[2.5,2.5]])],
               'limitations':['Not a digital twin or original arena layout. Source scan is partial and newer than the paper.',
                              'Missing scan cells are nearest-filled; the weight mask records inferred regions.',
                              'RGB is appearance only; no measured semantic friction classification.',
                              'Rigid collision: no sand sinkage, soil deformation, movable gravel, or vegetation entanglement.',
                              'A heightfield cannot reproduce overhangs or the underside of a bridge.',
                              'The 0.70 m full level intentionally exceeds previous HAMR clearance caps. Traversability is not guaranteed.'],
               'sha256':{ext:digest(output/'meshes'/f'{name}.{ext}') for ext in ('stl','dae')},
               'texture_sha256':digest(texture)}
        (output/'config'/f'{name}.json').write_text(json.dumps(entry,indent=2)+'\n');entries.append(entry)
    (output/'config/verti_arena_terrains.json').write_text(json.dumps(entries,indent=2)+'\n')
    return entries


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=geometry.package_directory())
    parser.add_argument('--source',type=Path)
    args=parser.parse_args()
    for entry in generate(args.output,args.source):
        print(json.dumps({k:entry[k] for k in ('id','height_range_m','max_surface_slope_deg','minimum_0_4m_patch_relief_m')}),flush=True)
