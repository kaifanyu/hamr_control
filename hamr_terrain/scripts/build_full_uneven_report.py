#!/usr/bin/env python3
"""Present real terrain previews and matched measured motion against flat ground."""
import argparse
import csv
import html
import json
from pathlib import Path

import numpy as np

from generate_full_uneven_terrains import IDS, FLAT_ID
from generate_terrains import package_directory
from terrain_replay_paths import sha256


def measured(directory):
    validation=directory/'replay_validation.json'
    if not validation.exists():return None
    summary=json.loads((directory/'run_summary.json').read_text())
    source=json.loads((directory/'trajectory.json').read_text())
    samples=[s for s in source['samples'] if s['phase']=='motion' and .5<=s['trajectory_time_s']<=summary['route']['reference_duration_s']-.5]
    t=np.array([s['trajectory_time_s'] for s in samples])
    xyz=np.array([s['position_m'] for s in samples])
    rpy=np.array([s['base_rpy_rad'] for s in samples])
    error=np.array([s['position_error_m'] for s in samples])
    return dict(summary=summary,t=t,xyz=xyz,rpy=rpy,error=error)


def build(root):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    root=Path(root).resolve();package=package_directory()
    (root/'plots').mkdir(exist_ok=True)
    maps=[json.loads((package/'config'/f'{name}.json').read_text()) for name in IDS]
    runs={}
    for name in (FLAT_ID,*IDS):
        for caster in ('swivel','ball'):
            runs[name,caster]=measured(root/'replays/runs'/name/'weave'/caster)
    rows=[]
    for caster in ('swivel','ball'):
        fig,axes=plt.subplots(3,1,figsize=(10,8),layout='constrained',sharex=True)
        baseline=runs[FLAT_ID,caster]
        for name in (FLAT_ID,*IDS):
            run=runs[name,caster]
            if run is None:continue
            label=name.removeprefix('full_uneven_').replace('_',' ')
            axes[0].plot(run['t'],1000*(run['xyz'][:,2]-run['xyz'][0,2]),label=label)
            axes[1].plot(run['t'],np.degrees(run['rpy'][:,1]),label=label)
            axes[2].plot(run['t'],1000*run['error'],label=label)
            row={'terrain':name,'caster':caster,'tracking_passed':run['summary']['tracking_passed'],
                 'physical_failure':run['summary']['physical_failure'],
                 'height_range_mm':float(1000*np.ptp(run['xyz'][:,2])),
                 'roll_rms_deg':float(np.degrees(np.sqrt(np.mean(run['rpy'][:,0]**2)))),
                 'pitch_rms_deg':float(np.degrees(np.sqrt(np.mean(run['rpy'][:,1]**2)))),
                 'maximum_absolute_tilt_deg':float(np.degrees(np.abs(run['rpy'][:,:2]).max())),
                 'xy_tracking_rms_mm':run['summary']['measurements']['tracking_rmse_m']*1000,
                 'xy_difference_from_flat_rms_mm':None}
            if baseline is not None:
                mask=(run['t']>=baseline['t'][0])&(run['t']<=baseline['t'][-1])
                ref=np.column_stack([np.interp(run['t'][mask],baseline['t'],baseline['xyz'][:,i]) for i in range(2)])
                row['xy_difference_from_flat_rms_mm']=float(1000*np.sqrt(np.mean(np.sum((run['xyz'][mask,:2]-ref)**2,axis=1))))
            rows.append(row)
        axes[0].set_ylabel('Height change (mm)');axes[1].set_ylabel('Chassis pitch (°)')
        axes[2].set_ylabel('XY tracking error (mm)');axes[2].set_xlabel('Trajectory time (s)')
        axes[0].legend(ncol=4,fontsize=9)
        for axis in axes:axis.grid(alpha=.25)
        fig.suptitle(f'{caster.title()} caster: measured motion on the same weaving reference')
        fig.savefig(root/'plots'/f'{caster}_motion.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(11,5),layout='constrained')
    for axis,terrain in zip(axes,maps):
        grid=np.load(package/'meshes'/f"{terrain['id']}_height.npz")
        im=axis.pcolormesh(grid['x'],grid['y'],grid['z']*1000,cmap='terrain',vmin=0,vmax=45,shading='auto')
        axis.set_aspect('equal');axis.set(xlabel='X (m)',ylabel='Y (m)',title=f"{terrain['level'].title()} · {terrain['height_range_m']*1000:.0f} mm relief")
    fig.colorbar(im,ax=axes,label='Surface height (mm)');fig.savefig(root/'plots/height_maps.png',dpi=160);plt.close(fig)
    sections=[];contact_reports=[]
    for terrain in maps:
        name=terrain['id'];directory=root/'contacts'/name
        path=directory/'report.json'
        report=json.loads(path.read_text()) if path.exists() else {}
        valid=bool(report.get('passed'))
        images=[]
        for key in ('overview','detail'):
            entry=report.get('images',{}).get(key)
            if not entry:valid=False;continue
            file=directory/entry['file']
            if not file.is_file() or sha256(file)!=entry['sha256']:valid=False;continue
            images.append(f'<figure><a href="contacts/{name}/{entry["file"]}"><img src="contacts/{name}/{entry["file"]}"></a><figcaption>Actual Gazebo {key} camera; physical scale</figcaption></figure>')
        contact_reports.append({'terrain':name,'passed':valid,'probes':len(report.get('probes',{}))})
        videos=[]
        for caster in ('swivel','ball'):
            rel=f'replays/runs/{name}/weave/{caster}'
            if runs[name,caster] is not None:
                videos.append(f'<article><h3>{caster.title()} caster</h3><video controls preload="none" poster="{rel}/preview.jpg" src="{rel}/replay.mp4"></video><p><a href="{rel}/replay.mp4">MP4</a> · <a href="{rel}/trajectory.csv">Trajectory CSV</a> · <a href="{rel}/trajectory.json">All joint feedback</a></p></article>')
        sections.append(f'<section><h2>{html.escape(terrain["title"])}</h2><p>Relief ≤{terrain["height_range_m"]*1000:.0f} mm; maximum actual triangle slope {terrain["max_surface_slope_deg"]:.2f}°; smallest 40 cm patch height range {terrain["minimum_0_4m_patch_relief_m"]*1000:.2f} mm. Contact checks: {"passed" if valid else "pending"}.</p><div class="pair">{"".join(images)}</div><div class="pair">{"".join(videos)}</div><p><a href="contacts/{name}/report.json">Contact evidence</a></p></section>')
    complete=len(rows)==8 and all(r['passed'] for r in contact_reports)
    result={'complete':complete,'recordings':len(rows),'expected_recordings':8,'tracking_passed':sum(r['tracking_passed'] for r in rows),
            'contact_checks':contact_reports,'maps':[{k:m[k] for k in ('id','height_range_m','max_surface_slope_deg','minimum_0_4m_patch_relief_m','horizontal_surface_triangles')} for m in maps],
            'motion':rows,'metrics_scope':'Motion samples from 0.5 s after the start to 0.5 s before the end. XY differences use time-interpolated measured flat-reference motion, not artificially altered robot poses.'}
    (root/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    if rows:
        with (root/'motion_comparison.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    table=''.join(f'<tr><td>{r["terrain"].removeprefix("full_uneven_")}</td><td>{r["caster"]}</td><td>{r["height_range_mm"]:.2f}</td><td>{r["maximum_absolute_tilt_deg"]:.2f}</td><td>{r["xy_tracking_rms_mm"]:.2f}</td><td>{r["xy_difference_from_flat_rms_mm"]:.2f}</td><td>{"Pass" if r["tracking_passed"] else "Fail"}</td></tr>' for r in rows if r['xy_difference_from_flat_rms_mm'] is not None)
    page=f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>HAMR — three fully uneven terrains</title>
<style>body{{font:16px system-ui;max-width:1400px;margin:auto;padding:30px;background:#edf2f6;color:#172637}}p{{line-height:1.5}}.pair{{display:grid;grid-template-columns:1fr 1fr;gap:18px}}img,video{{width:100%}}figure,article{{margin:0;background:white;padding:10px}}section{{margin-top:35px}}td,th{{padding:9px;text-align:left;border-bottom:1px solid #ccd5df}}table{{border-collapse:collapse}}a{{color:#165da0}}@media(max-width:800px){{.pair{{grid-template-columns:1fr}}}}</style>
<h1>Three levels of uneven terrain over the entire map</h1><p>{len(rows)}/8 validated recordings, including two flat-reference controls. The three driving maps have no flat approach, flat border, road lane or flat support plane. They share one irregular spatial pattern, scaled to 15, 30 and 45 mm total relief.</p>
<p>The conservative height bounds are below the model's approximately 190 mm neutral main-chassis clearance and 80 mm swivel-wheel radius. Slope is also bounded. These bounds reduce obstacle risk; tested runs provide the evidence about tipping at 0.25 m/s.</p>
<p>Both vehicles follow a new 4.2 × 6.4 m weaving reference with unchanged controls. Ground relief changes the measured height, chassis tilt and wheel/suspension response. Closed-loop XY tracking may remain close to the reference; no deviation has been injected into the controller or robot pose.</p>
<p><a href="replays/index.html">All replay videos, including flat controls</a> · <a href="motion_comparison.csv">Measured comparison CSV</a> · <a href="results.json">Results</a> · <a href="README.md">Launch and reproduce</a></p>
<img src="plots/height_maps.png" alt="Height maps with one common millimeter color scale">
{''.join(sections)}
<h2>Measured trajectory changes</h2><p>Height range is vertical motion of the root frame; tilt is chassis roll/pitch, not the stabilized upper plate. XY difference compares the actual trajectory with the same car's measured flat-ground run at matching trajectory times.</p>
<table><thead><tr><th>Surface</th><th>Caster</th><th>Height range (mm)</th><th>Max tilt (°)</th><th>XY RMS (mm)</th><th>XY difference from flat (mm RMS)</th><th>Tracking</th></tr></thead><tbody>{table}</tbody></table>
<img src="plots/swivel_motion.png"><img src="plots/ball_motion.png"><p>Rigid-ground simulation with estimated robot/contact parameters and ideal wheel velocity servos. The drop probes verify support on the non-flat mesh; the flat-floor friction estimator is deliberately not applied on slopes.</p></html>'''
    (root/'index.html').write_text(page)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path);parser.add_argument('--require-complete',action='store_true')
    args=parser.parse_args();result=build(args.root)
    print(json.dumps({k:v for k,v in result.items() if k not in ('motion','maps')},indent=2))
    raise SystemExit(1 if args.require_complete and not result['complete'] else 0)
