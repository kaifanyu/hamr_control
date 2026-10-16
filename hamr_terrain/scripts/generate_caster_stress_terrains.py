#!/usr/bin/env python3
"""Generate three controlled caster challenges at two fixed relief levels.

Only caster_stress_* assets are written. Geometry is predeclared independently
of either robot's results. These rigid-contact conditions do not model tire
elastic shimmy, material damage, deformable soil, or calibrated drive torque.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

import generate_terrains as geometry


FAMILIES = ("washboard", "staggered", "saddle")
LEVELS = (1, 2)
HEIGHTS = {"washboard": (.020, .036), "staggered": (.030, .055),
           "saddle": (.035, .060)}
TITLES = {"washboard": "Oblique washboard", "staggered": "Staggered rounded pads",
          "saddle": "Alternating cross slopes"}
SOURCES = [
    {"title": "Kauzlarich, Bruning and Thacker: Wheelchair caster shimmy and turning resistance (1984)",
     "url": "https://pubmed.ncbi.nlm.nih.gov/6530672/",
     "evidence": "Experiments and equations distinguish caster shimmy and sliding-friction turning resistance; tire geometry changes both.",
     "limit": "No critical shimmy speed or measured tire-loss coefficient is transferred to HAMR."},
    {"title": "Kauzlarich, Bruning and Thacker: Wheelchair caster shimmy II: damping (2000)",
     "url": "https://pubmed.ncbi.nlm.nih.gov/10917262/",
     "evidence": "The correction to the earlier shimmy analysis considers tire friction, spindle-bearing friction and hydraulic damping.",
     "limit": "Rigid Coulomb contact and estimated joint damping do not reproduce calibrated tire-elastic shimmy."},
    {"title": "Sprigle, Huang and Misch: Measurement of rolling resistance and scrub torque of manual wheelchair drive wheels and casters (2022)",
     "url": "https://pubmed.ncbi.nlm.nih.gov/31891276/",
     "evidence": "Separate component tests measure coast-down rolling loss and loaded-wheel scrub torque; losses depend on load and surface and can trade off.",
     "limit": "Local alignment, tracking and drive effort must be measured separately; one cannot infer real energy loss from yaw alignment alone."},
    {"title": "Development of wheelchair caster testing equipment and preliminary testing of caster models (2017)",
     "url": "https://pmc.ncbi.nlm.nih.gov/articles/PMC5645576/",
     "full_text_xml": "https://www.ebi.ac.uk/europepmc/webservices/rest/PMC5645576/fullTextXML",
     "evidence": "The apparatus tests square and 30-degree oblique slat impacts and reverses travel to exercise stem bearings. The preliminary study found no significant durability difference between impact angles.",
     "limit": "Oblique excitation motivates a test condition, not a prediction of greater damage or a fatigue/standards-validation claim."},
    {"title": "Udengaard and Iagnemma: Analysis, Design, and Control of an Omnidirectional Mobile Robot in Rough Terrain (2009)",
     "url": "https://doi.org/10.1115/1.4000214",
     "evidence": "The rough-terrain controller considers wheel-terrain contact angles in an active split-offset caster system.",
     "limit": "The mechanism is active and differs from both passive HAMR models; this motivates terrain-normal-aware interpretation, not transferring performance claims."},
]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def envelope(x, y):
    # All planned paths and their approximately 0.6m wheel/body footprint are
    # inside the unattenuated field. Flat fixture lanes lie south of y=-2.35.
    return (geometry.ease((3.-np.abs(x))/.60)
            * geometry.ease((y+2.35)/.35)
            * geometry.ease((3.80-y)/.55))


def pad_centers():
    for row, cy in enumerate(np.arange(-1.75, 2.46, .70)):
        # At nominal caster tracks x=+/-0.22, even rows peak near+0.20
        # and odd rows near-0.15; the opposite track lies between pads.
        for cx in np.arange(-2.60, 2.31, .70) + (.35 if row % 2 else 0.):
            if abs(cx) <= 2.7:
                yield float(cx), float(cy)


def height(family, level, x, y):
    if family not in FAMILIES or level not in LEVELS:
        raise ValueError("Unknown stress family or level")
    peak = HEIGHTS[family][level-1]
    if family == "washboard":
        theta = math.radians(30.)
        normal_distance = np.sin(theta)*x + np.cos(theta)*y
        z = .5*peak*(1.+np.cos(2.*np.pi*normal_distance/.34))
    elif family == "staggered":
        z = np.zeros(np.broadcast(x, y).shape, dtype=float)
        for cx, cy in pad_centers():
            radius = np.hypot((x-cx)/.24, (y-cy)/.28)
            # Flat inner30% radius, then a C1 half-cosine ramp to zero.
            profile = geometry.ease((1.-radius)/.70)
            z = np.maximum(z, peak*profile)
    else:
        # Phase shift breaks mirror symmetry around the centerline while
        # retaining a separable, bounded surface and the same level geometry.
        sx = np.sin(2.*np.pi*x/1.20 + np.pi/5.)
        sy = np.sin(2.*np.pi*y/.90 + np.pi/7.)
        z = .5*peak*(1.+sx*sy)
    return z*envelope(x, y)


def grid(family, level):
    x, y = np.linspace(-3., 3., 241), np.linspace(-4., 4., 321)
    xx, yy = np.meshgrid(x, y)
    return x, y, height(family, level, xx, yy)


def triangle_height(x, y, z, px, py):
    i = int(np.clip(np.searchsorted(x, px, side="right")-1, 0, len(x)-2))
    j = int(np.clip(np.searchsorted(y, py, side="right")-1, 0, len(y)-2))
    u, v = (px-x[i])/(x[i+1]-x[i]), (py-y[j])/(y[j+1]-y[j])
    a, b, c, d = z[j,i], z[j,i+1], z[j+1,i+1], z[j+1,i]
    return float(a+(b-a)*u+(c-b)*v if u>=v else a+(c-d)*u+(d-a)*v)


def family_parameters(family, level):
    peak = HEIGHTS[family][level-1]
    common = {"nominal_peak_height_m": peak, "level_changes": "Height multiplier only; XY geometry, phases and contact settings stay fixed."}
    if family == "washboard":
        return {**common, "normal_wavelength_m": .34, "wavefront_obliquity_deg": 30.,
            "formula": "H/2*(1+cos(2*pi*(sin(30deg)*x+cos(30deg)*y)/0.34))",
            "analytic_unblended_max_grade_deg": math.degrees(math.atan(math.pi*peak/.34)),
            "excitation_frequency_at_world_y_speed_0p25_m_s_hz": .25*math.cos(math.pi/6)/.34,
            "mechanism": "Repeated oblique support-normal changes and wheel/body excitation; direction changes require passive caster realignment while traversing ridges."}
    if family == "staggered":
        return {**common, "pad_semiaxes_m": [.24, .28], "flat_top_radial_fraction": .30,
            "x_pitch_m": .70, "y_pitch_m": .70, "alternate_row_x_offset_m": .35,
            "nominal_track_check_x_m": [-.22,.22],
            "pad_centers_xy_m": list(pad_centers()),
            "formula": "max_i H*ease((1-sqrt(((x-cx_i)/.24)^2+((y-cy_i)/.28)^2))/.70)",
            "analytic_unblended_max_grade_upper_bound_deg": math.degrees(math.atan(math.pi*peak/(2*.70*.24))),
            "mechanism": "Localized30/55mm rounded height transitions, with alternating left/right contacts and spatially changing transverse normals. No sharp vertical curb or movable obstacle."}
    return {**common, "wavelength_x_m": 1.20, "wavelength_y_m": .90,
        "phase_x_rad": math.pi/5, "phase_y_rad": math.pi/7,
        "formula": "H/2*(1+sin(2*pi*x/1.20+pi/5)*sin(2*pi*y/.90+pi/7))",
        "analytic_unblended_max_grade_upper_bound_deg": math.degrees(math.atan(math.pi*peak/.90)),
        "mechanism": "Alternating cross-slope/camber and roll demand; phase offsets make the centerline exposure asymmetric without changing material properties."}


def validate_geometry(x, y, z, vertices, faces, top):
    triangles = vertices[faces]
    normals = np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    edges = np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]])
    _, inverse, counts = np.unique(np.sort(edges,axis=1),axis=0,return_inverse=True,return_counts=True)
    balance = np.bincount(inverse,weights=np.where(edges[:,0]<edges[:,1],1.,-1.))
    checks = {
        "finite": bool(np.isfinite(vertices).all()),
        "bounded_relief_at_most_60mm": bool(z.min()>=0 and z.max()<=.06000000001),
        "nondegenerate_faces": bool(np.all(np.linalg.norm(normals,axis=1)>1e-9)),
        "upward_top_normals": bool(np.all(normals[:top,2]>0)),
        "closed_oriented_manifold": bool(np.all(counts==2) and np.all(balance==0)),
        "positive_enclosed_volume": bool(np.einsum("ij,ij->",triangles[:,0],normals)>0),
        "zero_perimeter_height": bool(np.all(z[[0,-1],:]==0) and np.all(z[:,[0,-1]]==0)),
        "flat_probe_approach": bool(np.all(z[y<=-2.35]==0)),
    }
    if not all(checks.values()):
        raise ValueError(f"Invalid generated stress geometry: {checks}")
    top_normals = normals[:top]
    face_grade = np.degrees(np.arctan2(np.linalg.norm(top_normals[:,:2],axis=1),top_normals[:,2]))
    return checks, face_grade


def render_figure(output, name, title, x, y, z, grade, face_max):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    directory = output/"figures"
    directory.mkdir(exist_ok=True)
    fig, axes = plt.subplots(1,2,figsize=(10,6.3),layout="constrained")
    for ax, values, cmap, maximum, label in (
        (axes[0],z*1000.,"terrain",60.,"Physical height (mm)"),
        (axes[1],grade,"magma",30.,"Grid-node surface grade (degrees)")):
        image = ax.imshow(values,origin="lower",extent=[x[0],x[-1],y[0],y[-1]],
                          vmin=0.,vmax=maximum,cmap=cmap,interpolation="nearest")
        fig.colorbar(image,ax=ax,shrink=.80,label=label)
        ax.add_patch(Rectangle((-1.8,-2.),3.6,4.,fill=False,edgecolor="#00cbe6",linewidth=1.5,linestyle="--"))
        ax.set(xlabel="World X (m)",ylabel="World Y (m)",aspect="equal")
        ax.plot([-2.6,-1.6],[-3.5,-3.5],color="black",linewidth=3)
        ax.text(-2.1,-3.35,"1 m",ha="center",fontsize=8,color="black",
                bbox={"facecolor":"white","edgecolor":"none","alpha":.8,"pad":1})
    axes[0].set_title(f"Measured relief: {np.ptp(z)*1000:.1f} mm")
    axes[1].set_title(f"Max triangle-face grade: {face_max:.2f}°")
    fig.suptitle(title+"\nDashed region: prescribed route and approximate wheel footprint",fontsize=12)
    for extension in ("png","svg"):
        fig.savefig(directory/f"{name}_height_slope.{extension}",dpi=180)
    plt.close(fig)
    return {"png":f"../figures/{name}_height_slope.png","svg":f"../figures/{name}_height_slope.svg",
            "source":"Derived from exact generated height grid; not a Gazebo camera image.",
            "common_height_limits_mm":[0,60],"common_grade_limits_deg":[0,30]}


def generate(output, families=FAMILIES, levels=LEVELS):
    output = Path(output).expanduser().resolve()
    for subdirectory in ("config", "worlds", "meshes"):
        (output/subdirectory).mkdir(parents=True,exist_ok=True)
    metadata = []
    for family in families:
        for level in levels:
            name = f"caster_stress_{family}_l{level}"
            x, y, z = grid(family,level)
            _,_,_,vertices,faces,top = geometry.mesh_from_height_grid(x,y,z)
            checks, face_grade = validate_geometry(x,y,z,vertices,faces,top)
            collision, visual = output/"meshes"/f"{name}.stl", output/"meshes"/f"{name}.dae"
            geometry.stl(collision,vertices,faces)
            # Reuse the existing full-surface altitude palette, without a
            # visually privileged central strip. This is only the Collada ID.
            geometry.dae(visual,"full_uneven_"+name,vertices,faces,top)
            np.savez_compressed(output/"meshes"/f"{name}_height.npz",x=x,y=y,z=z)
            geometry.make_world(output,name,lambda px,py:triangle_height(x,y,z,px,py))
            world_path = output/"worlds"/f"{name}.sdf"
            tree = ET.parse(world_path)
            world = tree.getroot().find("world")
            for model in list(world.findall("model")):
                if model.get("name","").startswith("marker_"):
                    world.remove(model)
            ET.indent(tree,space="  ")
            tree.write(world_path,encoding="utf-8",xml_declaration=True)
            xx, yy = np.meshgrid(x,y)
            gy,gx = np.gradient(z,y,x)
            grade = np.degrees(np.arctan(np.hypot(gx,gy)))
            figure = render_figure(output,name,f"{TITLES[family]} · level {level}",x,y,z,grade,float(face_grade.max()))
            region = (abs(xx)<=1.8)&(abs(yy)<=2.)
            picks = [("high_point",np.where(region,z,-np.inf).argmax()),
                     ("steep_point",np.where(region,grade,-np.inf).argmax())]
            probes = [{"name":"flat_approach","xy_m":[0.,-2.6],"kind":"flat"},
                      {"name":"crossfall","xy_m":[.22,.20],"kind":"feature"}]
            for title,index in picks:
                j,i = np.unravel_index(index,z.shape)
                probes.append({"name":title,"xy_m":[float(x[i]),float(y[j])],"kind":"feature"})
            footprint = (abs(xx)<=.65)&(yy>=-1.9)&(yy<=-.65)
            params = family_parameters(family,level)
            data = {
                "id":name,"title":f"{TITLES[family]} · level {level}",
                "description":f"Controlled {family} challenge with {HEIGHTS[family][level-1]*1000:g}mm nominal local relief; identical geometry and material for both caster models.",
                "family":family,"level":level,"world":f"../worlds/{name}.sdf",
                "bounds_m":{"x":[-3.,3.],"y":[-4.,4.]},
                "height_grid":f"../meshes/{name}_height.npz",
                "collision_mesh":f"../meshes/{name}.stl","visual_mesh":f"../meshes/{name}.dae",
                "grid_spacing_m":.025,"vertices":len(vertices),"triangles":len(faces),"surface_triangles":top,
                "min_height_m":float(z.min()),"max_height_m":float(z.max()),"height_range_m":float(np.ptp(z)),
                "max_surface_slope_deg":float(grade.max()),"max_triangle_face_slope_deg":float(face_grade.max()),
                "route_region":{"definition":"|x|<=1.8m and |y|<=2m, covering prescribed chassis routes and approximate wheel footprint.",
                    "height_min_m":float(z[region].min()),"height_max_m":float(z[region].max()),
                    "grade_p95_deg":float(np.percentile(grade[region],95)),"grade_max_deg":float(grade[region].max()),
                    "maximum_adjacent_grid_height_change_m":float(max(np.abs(np.diff(z,axis=0)).max(),np.abs(np.diff(z,axis=1)).max()))},
                "slope_method":"Grid-node finite differences, with separate exact triangle-face maximum; all exclude closing side walls. Whole-surface extrema include boundary blends.",
                "parameters":params,
                "figure":figure,
                "boundary_blend":{"formula":"ease((3-|x|)/.60)*ease((y+2.35)/.35)*ease((3.80-y)/.55)",
                    "ease":"0.5-0.5*cos(pi*clip(t,0,1))","full_region":{"x":[-2.4,2.4],"y":[-2.,3.25]},
                    "note":"Only outside the stress-route region is a flat probe/entry margin or boundary taper; no special flat road through the center."},
                "terrain_friction":{"mu":1.,"mu2":1.,"restitution":0.,"basis":"Matches DART mesh attachment defaults; identical to prior validated shared terrain worlds. Runtime validation remains required."},
                "spawn":{"x":0.,"y":-2.8,"z":0.,"yaw":math.pi/2},"probes":probes,
                "smoke":{"spawn_x":0.,"spawn_y":-1.3,"spawn_z":float(z[footprint].max()+.07),
                    "initial_yaw":math.pi/2,"drive_speed_m_s":.25,"duration_s":10.,
                    "scope":"Short common centerline segment for pilot contact checks, not a full trajectory comparison or a guaranteed pass."},
                "comparison_design":{"geometry_fixed_before_robot_results":True,"same_geometry_both_models":True,
                    "intended_speed_m_s":.25,"levels_are_deterministic_conditions_not_statistical_replicates":True,
                    "intended_paths":"Reversals y±1.1; tight figure-eight x±.65,y±1; lateral turns x0..1.2,y−1.5..1.5; zigzag x±.6,y−1.5..1.5.",
                    "interpretation":"Compare paired runs within terrain/level/route/seed. Do not select only conditions favoring either caster or call commanded-path yaw difference tire slip on a sloped turn."},
                "research_sources":SOURCES,
                "limitations":["Rigid analytic geometry; no soil deformation, loose gravel, fatigue or tire elasticity.",
                    "Estimated caster inertia, bearing loss and different wheel geometries remain model confounds.",
                    "Height levels are bounded engineering choices, not literature-derived pass/fail thresholds.",
                    "No guarantee either robot traverses until a physical pilot is run; no motor/climb capability claim.",
                    "Mesh colors encode altitude only and do not change material friction."],
                "geometry_validation":{"passed":True,"checks":checks,"runtime_status":"not_run_by_generator"},
                "generator":{"script":"generate_caster_stress_terrains.py","script_sha256":sha256(__file__),
                    "shared_exporter":"generate_terrains.py","shared_exporter_sha256":sha256(geometry.__file__),
                    "randomness":"none; analytic grid and fixed phases/pad centers"},
                "sha256":{"stl":sha256(collision),"dae":sha256(visual),"world":sha256(world_path)},
            }
            (output/"config"/f"{name}.json").write_text(json.dumps(data,indent=2,allow_nan=False)+"\n")
            metadata.append(data)
            print(json.dumps({k:data[k] for k in ("id","min_height_m","max_height_m","max_surface_slope_deg","max_triangle_face_slope_deg")}),flush=True)
    (output/"config/caster_stress_terrains.json").write_text(json.dumps(metadata,indent=2,allow_nan=False)+"\n")
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=None)
    parser.add_argument("--families",choices=FAMILIES,nargs="+",default=list(FAMILIES))
    parser.add_argument("--levels",type=int,choices=LEVELS,nargs="+",default=list(LEVELS))
    args = parser.parse_args()
    if len(set(args.families))!=len(args.families) or len(set(args.levels))!=len(args.levels):
        parser.error("Family and level selections must be unique")
    generate(args.output or geometry.package_directory(),args.families,args.levels)


if __name__ == "__main__":
    main()
