#!/usr/bin/env python3
"""Four scan-derived rigid rocky terrains with explicit closed stone geometry.

The background is a filtered, amplitude-reduced remix of the pinned Verti-Arena
scan. Stones are synthetic and deterministic. The projected height grid is for
route visualization / spawning; the closed triangle mesh is physical truth.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.spatial import ConvexHull

import generate_terrains as geo
from generate_verti_arena import compose

FAMILIES = ("stone_steps", "diagonal_ledges", "staggered_rocks", "cross_slope")
TITLES = {"stone_steps": "Irregular stone steps", "diagonal_ledges": "Diagonal rock ledges",
          "staggered_rocks": "Staggered rock garden", "cross_slope": "Rocky cross-slope"}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stone(cx, cy, sx, sy, yaw, height, ground, rng):
    # Eight-sided footprints, vertical walls, 3 mm beveled top edges; all units m.
    ring = np.array([[-.5,-.32],[-.32,-.5],[.32,-.5],[.5,-.32],
                     [.5,.32],[.32,.5],[-.32,.5],[-.5,.32]])
    ring *= rng.uniform(.93, 1.05, (8, 1))*[sx, sy]
    rot = np.array([[np.cos(yaw), -np.sin(yaw)],[np.sin(yaw), np.cos(yaw)]])
    xy = ring @ rot.T + [cx,cy]
    zbase = ground(cx, cy)
    top = zbase+height
    tilt = rng.uniform(-.009, .009, 2)
    def plane(points): return top+(points-[cx,cy]) @ tilt
    bevel = min(.003, height*.18)
    inner = (xy-[cx,cy])*.94+[cx,cy]
    vertices = np.vstack((np.c_[xy, np.full(8, zbase-.035)],
                          np.c_[xy, plane(xy)-bevel], np.c_[inner,plane(inner)]))
    hull = ConvexHull(vertices)
    faces = hull.simplices.copy()
    tri = vertices[faces]
    normals = np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    wrong = np.sum(normals*hull.equations[:,:3],axis=1)<0
    faces[wrong] = faces[wrong][:,[0,2,1]]
    return vertices, faces, {"center_xy_m":[cx,cy], "size_xy_m":[sx,sy],
        "yaw_rad":yaw, "nominal_height_above_center_ground_m":height,
        "top_center_z_m":top, "top_gradient":tilt.tolist(), "bevel_m":bevel}


def specs(family):
    if family == "stone_steps":
        # Each flight rises 15 -> 30 -> 40 mm, then descends. Small seams split slabs.
        for row, y in enumerate(np.arange(-2.1, 3.2, .53)):
            h = [.015,.030,.040,.030,.015][row % 5]
            for x in np.arange(-2.5, 2.7, .62):
                yield float(x),float(y),.61,.52,0.,h
    elif family == "diagonal_ledges":
        angle = np.deg2rad(30)
        for row, offset in enumerate(np.arange(-2.7,3.4,.72)):
            for u in np.arange(-2.8,2.9,.65):
                x,y = np.cos(angle)*u-np.sin(angle)*offset,np.sin(angle)*u+np.cos(angle)*offset
                if abs(x)<2.65 and -3.25<y<3.25:
                    yield x,y,.64,.24,angle,[.020,.030,.040][row%3]
    else:
        step = .52 if family == "staggered_rocks" else .70
        for row,y in enumerate(np.arange(-3.1,3.2,step)):
            for col,x in enumerate(np.arange(-2.6,2.7,step)):
                x += .22*(row%2)
                if abs(x)>2.7:continue
                yield float(x),float(y),.30,.25,((row*3+col*7)%11)*.22, [.015,.025,.035][(row+col)%3]


def surface_height(vertices, faces, x, y, initial=None):
    """Vertical upper envelope of mesh triangles; also handles explicit ledges."""
    xx,yy = np.broadcast_arrays(np.asarray(x,dtype=float),np.asarray(y,dtype=float))
    shape = xx.shape
    xx,yy = xx.ravel(),yy.ravel()
    z = np.full(len(xx),-np.inf) if initial is None else np.broadcast_to(initial,shape).ravel().copy()
    for a,b,c in vertices[faces]:
        normal = np.cross(b-a,c-a)
        if normal[2] <= 1e-12: continue
        mask = ((xx>=min(a[0],b[0],c[0])-1e-10)&(xx<=max(a[0],b[0],c[0])+1e-10)
                &(yy>=min(a[1],b[1],c[1])-1e-10)&(yy<=max(a[1],b[1],c[1])+1e-10))
        ids = np.flatnonzero(mask)
        if not len(ids):continue
        dx,dy = xx[ids]-a[0],yy[ids]-a[1]
        u = (dx*(c[1]-a[1])-dy*(c[0]-a[0]))/normal[2]
        v = ((b[0]-a[0])*dy-(b[1]-a[1])*dx)/normal[2]
        ids = ids[(u>=-1e-9)&(v>=-1e-9)&(u+v<=1+1e-9)]
        zz = a[2]-(normal[0]*(xx[ids]-a[0])+normal[1]*(yy[ids]-a[1]))/normal[2]
        z[ids] = np.maximum(z[ids],zz)
    return z.reshape(shape)


def sphere_support(vertices, faces, xy, radius):
    """Highest vertical sphere-center contact with triangle faces/edges/vertices.

    Analytic face tangencies and edge optima; no rasterized approximation at a ledge.
    The sphere is translated down from above at the specified horizontal center.
    """
    center = np.asarray(xy,dtype=float)
    tri = vertices[faces]
    candidates = ((tri[:,:,:2].min(axis=1)<=center+radius).all(axis=1)
                  &(tri[:,:,:2].max(axis=1)>=center-radius).all(axis=1))
    best = -np.inf
    for a,b,c in tri[candidates]:
        n = np.cross(b-a,c-a); n /= np.linalg.norm(n)
        if n[2]>1e-10:
            height = (np.dot(n,a)-np.dot(n[:2],center)+radius)/n[2]
            point = np.r_[center,height]-radius*n
            edges = (b-a,c-a)
            mat = np.array([[np.dot(e,f) for f in edges] for e in edges])
            u,v = np.linalg.solve(mat,[np.dot(point-a,e) for e in edges])
            if u>=-1e-9 and v>=-1e-9 and u+v<=1+1e-9:best=max(best,height)
        for p,q in ((a,b),(b,c),(c,a)):
            d = q-p; rxy = p[:2]-center; dd = np.dot(d[:2],d[:2])
            if dd<1e-18:
                remainder=radius**2-np.dot(rxy,rxy)
                if remainder>=0:best=max(best,max(p[2],q[2])+np.sqrt(remainder))
                continue
            u0=-np.dot(d[:2],rxy)/dd
            rho2=max(0.,np.dot(rxy,rxy)-np.dot(d[:2],rxy)**2/dd)
            remaining=radius**2-rho2
            if remaining<0:continue
            span=np.sqrt(remaining/dd)
            lo,hi=max(0.,u0-span),min(1.,u0+span)
            if lo>hi:continue
            optimum=u0+d[2]*np.sqrt(remaining)/np.sqrt(dd*(dd+d[2]**2))
            u=np.clip(optimum,lo,hi)
            value=p[2]+d[2]*u+np.sqrt(max(0.,remaining-dd*(u-u0)**2))
            best=max(best,value)
    if not np.isfinite(best):raise ValueError("No supporting mesh under probe")
    return float(best)


def validate_component(vertices, faces):
    tri=vertices[faces]
    area=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)
    edges=np.sort(np.concatenate((faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]])),axis=1)
    _,counts=np.unique(edges,axis=0,return_counts=True)
    volume=np.sum(tri[:,0]*np.cross(tri[:,1],tri[:,2]))/6
    if not (np.isfinite(vertices).all() and np.min(area)>1e-12 and (counts==2).all() and volume>0):
        raise ValueError("Mesh component must be a finite, closed, outward-oriented solid")


def generate(output):
    output=Path(output).resolve()
    for name in ("meshes","worlds","config","figures"):(output/name).mkdir(parents=True,exist_ok=True)
    sx,sy,normalized,_,_,_,source=compose(geo.package_directory()/"assets/verti_arena")
    # Fixed scan backdrop, reduced to 14 mm broad relief; no flat lane or apron.
    scan=gaussian_filter(normalized,8.)
    x=np.arange(-3.,3.001,.05); y=np.arange(-4.,4.001,.05)
    ix=np.rint((x-sx[0])/(sx[1]-sx[0])).astype(int)
    iy=np.rint((y-sy[0])/(sy[1]-sy[0])).astype(int)
    zz=scan[np.ix_(iy,ix)]; zz=(zz-zz.min())/np.ptp(zz)*.014
    xx,yy=np.meshgrid(x,y)
    zz+=.0015*np.sin(9.3*xx+7.1*yy)+.001*np.cos(5.8*xx-11.2*yy)
    zz-=zz.min()
    manifest=[]
    from generate_adtg_terrains import triangle_height
    for family in FAMILIES:
        name="rocky_"+family
        ground=zz.copy()
        if family=="cross_slope":
            # 5 degree crossfall over the route; eased near map sides, no cliff walls inside.
            ground+=np.tan(np.deg2rad(5.))*np.clip(xx,-2.,2.)*(.75+.25*np.sin(1.7*yy))
            ground-=ground.min()
        _,_,_,gv,gf,top=geo.mesh_from_height_grid(x,y,ground)
        validate_component(gv,gf)
        sampler=lambda px,py:triangle_height(x,y,ground,px,py)
        vertices,faces=[gv],[gf]; size=len(gv); objects=[]
        rng=np.random.default_rng(20261007)
        upper=ground.copy(); stone_vertices=[]; stone_faces=[]; stone_size=0
        for number,params in enumerate(specs(family)):
            v,f,info=stone(*params,sampler,rng)
            validate_component(v,f)
            vertices.append(v);faces.append(f+size);size+=len(v)
            stone_vertices.append(v);stone_faces.append(f+stone_size);stone_size+=len(v)
            objects.append({"id":number,**info})
        v,f=np.concatenate(vertices),np.concatenate(faces)
        upper=surface_height(np.concatenate(stone_vertices),np.concatenate(stone_faces),xx,yy,ground)
        geo.stl(output/"meshes"/f"{name}.stl",v,f)
        geo.dae(output/"meshes"/f"{name}.dae",name,v,f,len(f))
        np.savez_compressed(output/"meshes"/f"{name}_height.npz",x=x,y=y,z=upper,ground_z=ground,
                            collision_vertices=v,collision_faces=f)
        geo.make_world(output,name,lambda px,py:triangle_height(x,y,upper,px,py))
        world_path=output/"worlds"/f"{name}.sdf"
        tree=ET.parse(world_path);world=tree.getroot().find("world")
        for model in list(world.findall("model")):
            if model.get("name","").startswith(("apron_","marker_")):world.remove(model)
        for camera,eye,target in (("overview_camera",(5.4,-6.4,5.2),(0.,0.,.12)),
                                  ("detail_camera",(2.5,-3.1,1.35),(0.,0.,.05))):
            world.find(f"model[@name='{camera}']/pose").text=" ".join(map(str,geo.look_at(eye,target)))
        ET.indent(tree,space="  ");tree.write(world_path,encoding="utf-8",xml_declaration=True)
        # Probe exact faces, ledge transitions and rough background; not height-grid proxies.
        near=sorted(objects,key=lambda o:np.linalg.norm(o["center_xy_m"]))[:2]
        probes=[{"name":"rough_background","xy_m":[-2.85,-3.7],"kind":"feature"}]
        for i,obj in enumerate(near):
            center=np.array(obj["center_xy_m"]); yaw=obj["yaw_rad"]
            edge=center+np.array([-np.sin(yaw),np.cos(yaw)])*obj["size_xy_m"][1]*.48
            probes.extend([{"name":f"stone_top_{i}","xy_m":center.tolist(),"kind":"feature"},
                           {"name":f"stone_edge_{i}","xy_m":edge.tolist(),"kind":"feature"}])
        entry={"id":name,"title":TITLES[family],"description":"Filtered scan-derived ground plus explicit fixed stone solids.",
               "family":family,"world":f"../worlds/{name}.sdf","height_grid":f"../meshes/{name}_height.npz",
               "collision_mesh":f"../meshes/{name}.stl","visual_mesh":f"../meshes/{name}.dae",
               "bounds_m":{"x":[-3,3],"y":[-4,4]},"grid_spacing_m":.05,
               "height_range_m":float(np.ptp(upper)),"min_height_m":float(upper.min()),"max_height_m":float(upper.max()),
               "maximum_stone_height_m":max(o["nominal_height_above_center_ground_m"] for o in objects),
               "stones":objects,"vertices":len(v),"triangles":len(f),"flat_friction_probes":False,"probes":probes,
               "spawn":{"x":0.,"y":-1.5,"z":float(upper.max()+.04),"yaw":float(np.pi/2)},
               "terrain_friction":{"mu":1.,"mu2":1.,"restitution":0.,"status":"Estimated rigid contact; inherited backend settings"},
               "generation":{"seed":20261007,"source":source,"ground_relief_before_microrelief_m":.014,
                 "ground_filter_sigma_m":.2,"background_grid_m":.05,"stones":"Explicit closed convex polyhedra with vertical sides, beveled tops and small top tilts",
                 "grid_role":"Projected upper envelope for spawn and route visuals only; collision_vertices/faces are authoritative for physical support",
                 "closed_components":len(objects)+1,"all_components_validated":True},
               "limitations":["New synthetic stone layouts; not a reconstruction of GMU arena.","Fixed rigid rocks; no granular motion, soil sinkage or tire compliance.",
                  "Local step heights are engineering test choices, not established traversal limits.","Current vehicle configurations differ in radius, mass and bearing parameters."],
               "sha256":{ext:sha(output/"meshes"/f"{name}.{ext}") for ext in ("stl","dae")}}
        entry["sha256"]["world"]=sha(world_path)
        (output/"config"/f"{name}.json").write_text(json.dumps(entry,indent=2)+"\n")
        manifest.append(entry)
        print(json.dumps({"terrain":name,"stones":len(objects),"triangles":len(f),"height_range_m":entry["height_range_m"]}),flush=True)
    (output/"config/rocky_terrains.json").write_text(json.dumps(manifest,indent=2)+"\n")
    return manifest


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=geo.package_directory())
    generate(parser.parse_args().output)
