#!/usr/bin/env python3
"""Build an offline report from real Gazebo captures and measured trajectories."""
import argparse
import csv
import html
import json
from pathlib import Path
import shutil

import numpy as np

from generate_terrains import package_directory
from generate_verti_arena import IDS, digest


def build(root):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    root=Path(root).resolve();package=package_directory()
    plots=root/'plots';plots.mkdir(exist_ok=True)
    maps=[json.loads((package/'config'/f'{name}.json').read_text()) for name in IDS]
    fig,axes=plt.subplots(1,3,figsize=(13,4.8),layout='constrained')
    for ax,meta in zip(axes,maps):
        grid=np.load(package/'meshes'/f"{meta['id']}_height.npz")
        im=ax.pcolormesh(grid['x'],grid['y'],grid['z'],vmin=0,vmax=.7,cmap='terrain',shading='auto')
        ax.set(title=f"{meta['level'].title()} — {meta['height_range_m']:.2f} m relief",xlabel='X (m)',ylabel='Y (m)',aspect='equal')
    fig.colorbar(im,ax=axes,label='Surface height (m)');fig.savefig(plots/'heights.png',dpi=160);plt.close(fig)
    full=np.load(package/'meshes/verti_arena_full_height.npz')
    fig,ax=plt.subplots(figsize=(6,5),layout='constrained')
    im=ax.pcolormesh(full['x'],full['y'],full['inferred_weight'],vmin=0,vmax=1,cmap='magma',shading='auto')
    ax.set(title='Inferred elevation contribution',xlabel='X (m)',ylabel='Y (m)',aspect='equal');fig.colorbar(im,ax=ax,label='Blend weight from missing source cells')
    fig.savefig(plots/'inferred.png',dpi=150);plt.close(fig)
    rows=[];sections=[];contacts=[]
    fig,axes=plt.subplots(2,3,figsize=(15,9),layout='constrained')
    for index,meta in enumerate(maps):
        name=meta['id'];contact_dir=root/'contacts'/name
        report=json.loads((contact_dir/'report.json').read_text())
        valid=report['passed']
        for item in report['images'].values():
            valid=valid and digest(contact_dir/item['file'])==item['sha256']
        contacts.append({'terrain':name,'passed':valid,'probe_count':len(report['probes'])})
        videos=[];xy=axes[0,index];tilt=axes[1,index]
        grid=np.load(package/'meshes'/f'{name}_height.npz')
        xy.pcolormesh(grid['x'],grid['y'],grid['z'],cmap='Greys',vmin=0,vmax=.7,shading='auto',alpha=.6)
        for caster,color in [('swivel','#1266be'),('ball','#dc5921')]:
            rel=f'replays/runs/{name}/weave/{caster}';directory=root/rel
            if not (directory/'replay_validation.json').exists():continue
            summary=json.loads((directory/'run_summary.json').read_text())
            trajectory=json.loads((directory/'trajectory.json').read_text())
            samples=[s for s in trajectory['samples'] if s['phase']!='settling']
            if not samples:continue
            p=np.array([s['position_m'] for s in samples]);rpy=np.array([s['base_rpy_rad'] for s in samples]);t=np.array([s['trajectory_time_s'] for s in samples])
            status='Pass' if summary['tracking_passed'] else summary['physical_failure'] or 'Tracking outside tolerance'
            row={'terrain':name,'caster':caster,'tracking_passed':summary['tracking_passed'],
                 'physical_failure':summary['physical_failure'],'outcome':status,
                 'tracking_rms_m':summary['measurements']['tracking_rmse_m'],
                 'final_position_error_m':summary['measurements']['final_position_error_m'],
                 'maximum_abs_roll_or_pitch_deg':float(np.degrees(np.abs(rpy[:,:2]).max())),
                 'body_height_range_m':float(np.ptp(p[:,2])),
                 'last_recorded_trajectory_time_s':float(t[-1]),
                 'reference_duration_s':summary['route']['reference_duration_s']}
            motion=[s for s in samples if s['phase']=='motion']
            tail=[s for s in motion if s['trajectory_time_s']>=motion[-1]['trajectory_time_s']-5.] if motion else []
            tail_positions=np.array([s['position_m'][:2] for s in tail])
            tail_distance=float(np.linalg.norm(np.diff(tail_positions,axis=0),axis=1).sum()) if len(tail)>1 else None
            drive_joints=('left_rocker_left_wheel_joint','right_rocker_right_wheel_joint')
            drive_rates=[abs(s['joint_velocities_rad_s'][j]) for s in tail for j in drive_joints if j in s['joint_velocities_rad_s']]
            row['last_5s_motion_distance_m']=tail_distance
            row['last_5s_mean_abs_drive_speed_rad_s']=float(np.mean(drive_rates)) if drive_rates else None
            row['sustained_immobility_with_spinning_drives']=bool(len(tail)>50 and
                tail[-1]['trajectory_time_s']-tail[0]['trajectory_time_s']>3. and
                tail_distance<.01 and drive_rates and np.mean(drive_rates)>1.)
            rows.append(row)
            ref=np.array(summary['route']['points_m'])
            if caster=='swivel':xy.plot(ref[:,0],ref[:,1],'g--',lw=1.4,label='Reference')
            xy.plot(p[:,0],p[:,1],color=color,lw=1.5,label=caster)
            tilt.plot(t,np.degrees(np.abs(rpy[:,:2]).max(axis=1)),color=color,label=caster)
            videos.append(f'<article><h3>{caster.title()} — {html.escape(status.replace("_"," "))}</h3><video controls preload="none" poster="{rel}/preview.jpg" src="{rel}/replay.mp4"></video><p><a href="{rel}/replay.mp4">MP4</a> · <a href="{rel}/trajectory.csv">Trajectory CSV</a> · <a href="{rel}/trajectory.json">Joint feedback</a> · <a href="{rel}/run_summary.json">Run result</a></p></article>')
        xy.set(xlim=(-4,4),ylim=(-4,4),aspect='equal',title=meta['level'].title(),xlabel='X (m)',ylabel='Y (m)')
        tilt.set(xlabel='Trajectory time (s)',ylabel='Max |roll| or |pitch| (°)')
        for ax in (xy,tilt):
            ax.grid(alpha=.25)
            if ax.lines:ax.legend(fontsize=8)
        sections.append(f'<section><h2>{html.escape(meta["title"])}</h2><p>Contact probes: {"passed" if valid else "failed"}. Maximum local triangle slope {meta["max_surface_slope_deg"]:.1f}° includes abrupt obstacle faces.</p><div class="pair"><img src="contacts/{name}/overview.png" alt="Actual Gazebo overview"><img src="contacts/{name}/detail.png" alt="Actual Gazebo detail"></div><div class="pair">{"".join(videos)}</div><p><a href="contacts/{name}/report.json">Contact measurements</a></p></section>')
    fig.savefig(plots/'vehicle_comparison.png',dpi=160);plt.close(fig)
    result={'complete':len(rows)==6 and all(c['passed'] for c in contacts),'recordings':len(rows),
            'tracking_passed':sum(r['tracking_passed'] for r in rows),'contacts':contacts,'runs':rows,
            'scope':'Scan-derived new layouts; same rigid collision and caster models. Failed trajectories are retained. No soft-soil validation.'}
    (root/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    if rows:
        with (root/'comparison.csv').open('w',newline='') as out:
            w=csv.DictWriter(out,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    table=''.join(f'<tr><td>{r["terrain"].removeprefix("verti_arena_")}</td><td>{r["caster"]}</td><td>{html.escape(r["outcome"].replace("_"," "))}</td><td>{r["tracking_rms_m"]:.3f}</td><td>{r["maximum_abs_roll_or_pitch_deg"]:.1f}</td><td>{r["body_height_range_m"]*1000:.0f}</td></tr>' for r in rows)
    stuck=[r for r in rows if r['sustained_immobility_with_spinning_drives']]
    diagnosis=''.join(f'<p><strong>{html.escape(r["terrain"])} / {r["caster"]}:</strong> in the final five seconds of commanded motion, measured XY travel was {r["last_5s_motion_distance_m"]*1000:.2f} mm while the rear drive joints averaged {r["last_5s_mean_abs_drive_speed_rad_s"]:.2f} rad/s in absolute speed. This supports immobilization with wheel spin, rather than successful path following. Contact location and the video should be checked before attributing it specifically to caster swivel alignment.</p>' for r in stuck)
    page=f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>HAMR — Verti-Arena scan-derived terrain</title>
<style>body{{font:16px system-ui;max-width:1400px;margin:auto;padding:28px;background:#edf2f6;color:#172637}}p{{line-height:1.5}}.pair{{display:grid;grid-template-columns:1fr 1fr;gap:18px}}img,video{{width:100%}}article{{background:white;padding:12px}}section{{margin-top:35px}}td,th{{padding:9px;text-align:left;border-bottom:1px solid #ccd5df}}table{{border-collapse:collapse}}a{{color:#165da0}}@media(max-width:800px){{.pair{{grid-template-columns:1fr}}}}</style>
<h1>Verti-Arena scan-derived off-road maps</h1><p>Three complete 8 × 8 m maps with 15, 35 and 70 cm relief. {len(rows)}/6 validated recordings; {result['tracking_passed']} trajectories passed. All contact probes: {"passed" if all(c['passed'] for c in contacts) else "not passed"}.</p>
<p>Measured terrain and RGB data from <a href="https://github.com/fracance/verti-arena-extractor">the public Verti-Arena Extractor</a> are rotated and blended into new layouts. These are <strong>not exact replicas of GMU’s arena</strong>. The 70 cm level matches the elevation range reported in the <a href="https://arxiv.org/html/2508.08226v1">paper</a>. Lower levels are our adaptations.</p>
<p>Original horizontal patch scale; 25 mm collision grid; no flat apron or support plane. Missing source measurements contribute 3.66% of blended elevation weight, recorded explicitly. A small synthetic ripple prevents flat driving patches. RGB is appearance only: all ground uses the same estimated rigid friction. No sand sinkage, movable gravel or soil deformation is simulated.</p>
<p><a href="README.md">Setup and reproduction</a> · <a href="replays/index.html">All MP4s and logs</a> · <a href="comparison.csv">Comparison CSV</a> · <a href="results.json">Results JSON</a> · <a href="plots/inferred.png">Inferred-data map</a></p>
<img src="plots/heights.png" alt="Three physical height maps on the same scale">
{''.join(sections)}
<h2>Measured traversal</h2><p>Same 0.25 m/s weaving reference and unchanged controller on all levels. This is a geometric stress test; the controller does not plan around terrain obstacles. A completed video can record a failed trajectory.</p>
<table><thead><tr><th>Level</th><th>Caster</th><th>Outcome</th><th>XY RMS (m)</th><th>Max chassis tilt (°)</th><th>Body height range (mm)</th></tr></thead><tbody>{table}</tbody></table>
{diagnosis}
<img src="plots/vehicle_comparison.png" alt="Measured trajectories and chassis tilt"><p>Height and tilt summaries include post-start motion and final/failure holds. Controls use ideal velocity servos and estimated model parameters; results are not hardware validation.</p></html>'''
    (root/'index.html').write_text(page)
    shutil.copy2(package/'docs/VERTI_ARENA.md',root/'README.md')
    if (root/'replays').exists():shutil.copy2(package/'docs/VERTI_ARENA.md',root/'replays/README.md')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path)
    parser.add_argument('--require-complete',action='store_true');args=parser.parse_args()
    result=build(args.root);print(json.dumps({k:v for k,v in result.items() if k!='runs'},indent=2))
    raise SystemExit(1 if args.require_complete and not result['complete'] else 0)
