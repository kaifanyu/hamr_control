#!/usr/bin/env python3
"""Report outward/return tracking and measured chassis-backward travel.

Reads a canonical matrix and saved source timestamps, retaining failed runs.
Produces an offline gallery, CSV/JSON and figures without changing raw runs.
"""
import argparse
from collections import Counter
import csv
import html
import json
import os
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import analyze_caster_stress as analysis

COLORS={'swivel':'#c66a25','ball':'#246a9a'}
ROUTES={'stress_reversal_out_back':'Reversal + return',
        'stress_tight_figure_eight_out_back':'Tight eight + return',
        'stress_lateral_out_back':'Lateral turns + return',
        'stress_zigzag_out_back':'Zigzag + return'}
TERRAINS={'rocky_stone_steps':'Stone steps','rocky_diagonal_ledges':'Diagonal ledges',
          'rocky_staggered_rocks':'Staggered rocks','rocky_cross_slope':'Rocky cross-slope'}


def read(path):return json.loads(Path(path).read_text())


def distance_to_path(positions,points):
    """Unsigned XY distance to the polyline, independent of timing/traversal order."""
    positions=np.asarray(positions,dtype=float);points=np.asarray(points,dtype=float)
    start=points[:-1];delta=np.diff(points,axis=0)
    length_squared=np.sum(delta*delta,axis=1)
    keep=length_squared>1e-18
    start,delta,length_squared=start[keep],delta[keep],length_squared[keep]
    if not len(start):return np.linalg.norm(positions-points[0],axis=1)
    result=np.empty(len(positions))
    for first in range(0,len(positions),128):
        offset=positions[first:first+128,None,:]-start[None,:,:]
        fraction=np.clip(np.sum(offset*delta,axis=2)/length_squared,0.,1.)
        nearest_offset=offset-fraction[:,:,None]*delta
        result[first:first+128]=np.sqrt(np.min(np.sum(nearest_offset**2,axis=2),axis=1))
    return result


def series(samples,points=None):
    times,samples,quality=analysis.normalize_samples(samples)
    pos=np.array([s['position_m'][:2] for s in samples])
    ref=np.array([s['reference_m'] for s in samples])
    yaw=np.array([s['base_rpy_rad'][2] for s in samples])
    velocity=analysis.derivative(pos,times,.25)
    body=np.sum(velocity*np.c_[np.cos(yaw),np.sin(yaw)],axis=1)
    demand=np.linalg.norm([s['reference_velocity_m_s'] for s in samples],axis=1)>=.03
    moving=np.linalg.norm(velocity,axis=1)>=.03
    return {'samples':samples,'time':times,'t':np.array([s['trajectory_time_s'] for s in samples]),
            'xy':pos,'yaw':yaw,'body_forward':body,'demand':demand,'moving':moving,
            'error':np.linalg.norm(pos-ref,axis=1),'quality':quality,
            'path_distance':distance_to_path(pos,points) if points is not None else np.full(len(times),np.nan),
            'motion':np.array([s['phase']=='motion' for s in samples])}


def leg_metrics(data,leg,start=None,end=None):
    mask=data['motion'] & np.array([s.get('route_leg')==leg for s in data['samples']])
    if start is not None:mask &= data['t']>=start
    if end is not None:mask &= data['t']<=end
    stats=analysis.statistics(data['error'],data['time'],mask,.25)
    geometric=analysis.statistics(data['path_distance'],data['time'],mask,.25)
    eligible=mask & data['demand'] & data['moving'] & np.isfinite(data['body_forward'])
    weights=analysis.weights(data['time'],eligible,.25)
    duration=float(weights.sum())
    backwards=float(np.sum(weights*(data['body_forward']<-.03)))
    inds=np.flatnonzero(mask)
    return {'leg':leg,'xy_rms_mm':stats['rms']*1000 if stats['rms'] is not None else None,
            'xy_peak_mm':stats['max_abs']*1000 if stats['max_abs'] is not None else None,
            'nearest_path_rms_mm':geometric['rms']*1000 if geometric['rms'] is not None else None,
            'nearest_path_peak_mm':geometric['max_abs']*1000 if geometric['max_abs'] is not None else None,
            'observed_motion_s':float(analysis.weights(data['time'],mask,.25).sum()),
            'demanded_and_moving_s':duration,'body_backward_s':backwards,
            'body_backward_percent':100*backwards/duration if duration else None,
            'first_observed_trajectory_s':float(data['t'][inds[0]]) if len(inds) else None,
            'last_observed_trajectory_s':float(data['t'][inds[-1]]) if len(inds) else None}


def load_runs(root,matrix):
    runs={};hashes={str(root/'experiment_matrix.json'):analysis.sha256(root/'experiment_matrix.json')}
    for case in matrix:
        directory=root/case['directory']
        if not (directory/'replay_validation.json').is_file():continue
        summary=read(directory/'run_summary.json')
        if not read(directory/'replay_validation.json')['passed']:continue
        data=series(read(directory/'trajectory.json')['samples'],case['route']['points_m'])
        key=(case['terrain'],case['shape'],case['caster'])
        runs[key]={'directory':directory,'case':case,'summary':summary,'data':data,
                   'legs':{leg:leg_metrics(data,leg) for leg in ('outbound','return')}}
        for name in ('trajectory.json','run_summary.json','configuration.json','robot.urdf','world.sdf','contact_samples.json'):
            p=directory/name
            if p.is_file():hashes[str(p)]=analysis.sha256(p)
    return runs,hashes


def paired_legs(runs):
    rows=[]
    for terrain in TERRAINS:
        for route in ROUTES:
            models={m:runs.get((terrain,route,m)) for m in COLORS}
            if not all(models.values()):continue
            configs=[read(run['directory']/'configuration.json') for run in models.values()]
            fields=('route','controller','spawn','grid_sha256','source_world_sha256','runner_sha256','paths_source_sha256')
            mismatches=[k for k in fields if configs[0].get(k)!=configs[1].get(k)]
            if mismatches:raise ValueError(f'Mismatched pair {terrain}/{route}: {mismatches}')
            for leg in ('outbound','return'):
                limits=[run['legs'][leg] for run in models.values()]
                if any(p['first_observed_trajectory_s'] is None for p in limits):
                    rows.append({'terrain':terrain,'route':route,'leg':leg,'available':False})
                    continue
                start=max(p['first_observed_trajectory_s'] for p in limits)
                end=min(p['last_observed_trajectory_s'] for p in limits)
                metrics={m:leg_metrics(run['data'],leg,start,end) for m,run in models.items()}
                rows.append({'terrain':terrain,'route':route,'leg':leg,'available':end>start,
                             'common_start_s':start,'common_end_s':end,'models':metrics,
                             'both_full_runs_passed':all(r['summary']['tracking_passed'] for r in models.values())})
    return rows


def contact_window(directory,center,half_width=.5):
    path=directory/'contact_samples.json'
    if not path.is_file():return {'available':False}
    saved=read(path);origin=saved['motion_start_s']
    rows=[s for s in saved['samples'] if abs(s['simulation_time_s']-origin-center)<=half_width]
    presence=Counter();transverse_normals=Counter();groups=Counter();depths=[]
    for row in rows:
        links=set();side_links=set()
        for contact in row['contacts']:
            names=[contact['collision1'],contact['collision2']]
            robot=[name for name in names if name.startswith('compa::')]
            if not robot:continue
            link=robot[0].split('::')[1];links.add(link)
            depths.extend(contact['depths_m'])
            if any(abs(n[2])<.5 for n in contact['normals']):side_links.add(link)
        presence.update(links);transverse_normals.update(side_links)
        for side in ('left','right'):
            if any(link.startswith(side+'_caster_') for link in links):groups[side+'_caster_assembly']+=1
            if side+'_wheel_link' in links:groups[side+'_rear_wheel']+=1
    times=[s['simulation_time_s']-origin for s in rows]
    return {'available':True,'requested_center_trajectory_s':center,'requested_half_width_s':half_width,
            'snapshots':len(rows),'first_trajectory_s':min(times) if times else None,
            'last_trajectory_s':max(times) if times else None,
            'maximum_source_gap_s':max(np.diff(times),default=None),
            'snapshots_with_link_contact':dict(presence),
            'snapshots_with_assembly_contact':dict(groups),
            'snapshots_with_reported_abs_normal_z_below_half':dict(transverse_normals),
            'maximum_reported_depth_mm':1000*max(depths) if depths else None}


def peak_contact_windows(runs):
    windows=[]
    for terrain in TERRAINS:
        for route in ROUTES:
            swivel=runs.get((terrain,route,'swivel'))
            if not swivel or not swivel['data']['motion'].any():continue
            d=swivel['data'];indices=np.flatnonzero(d['motion'])
            peak=indices[np.argmax(d['error'][indices])];center=float(d['t'][peak])
            models={}
            for model in COLORS:
                run=runs.get((terrain,route,model))
                if not run:continue
                data=run['data'];closest=int(np.argmin(abs(data['t']-center)))
                nearby=data['samples'][closest]
                pose={'sample_trajectory_s':float(data['t'][closest]),
                      'distance_from_requested_time_s':float(abs(data['t'][closest]-center)),
                      'xy_error_mm':float(data['error'][closest]*1000),
                      'roll_pitch_deg':np.degrees(nearby['base_rpy_rad'][:2]).tolist(),
                      'route_leg':nearby.get('route_leg')}
                models[model]={'nearest_pose_sample':pose,
                               'contact_window':contact_window(run['directory'],center)}
            windows.append({'terrain':terrain,'route':route,'center_trajectory_s':center,
                            'selection':'Largest sampled swivel XY error during motion; descriptive selection after the run.',
                            'models':models})
    return {'scope':'One-second windows centered on each swivel run maximum; both models use the same trajectory-clock window. '
                    'Counts refer to received contact snapshots, not contact duration or calibrated load. '
                    'A missing link contact does not establish airborne motion. Raw force frames/signs are not interpreted.',
            'windows':windows}


def plot_grid(runs,output,channel):
    fig,axes=plt.subplots(4,4,figsize=(15,12))
    fig.subplots_adjust(left=.075,right=.98,top=.89,bottom=.10,wspace=.32,hspace=.48)
    for row,terrain in enumerate(TERRAINS):
        for col,route in enumerate(ROUTES):
            ax=axes[row,col]
            ax.set_title(ROUTES[route] if row==0 else '',fontsize=10)
            for model in COLORS:
                run=runs.get((terrain,route,model))
                if not run:continue
                d=run['data'];trip=run['case']['route']['out_and_back']
                values=d['error']*1000 if channel=='tracking' else d['body_forward']
                valid=d['motion'].copy()
                # Do not connect missing source-time intervals.
                valid[1:] &= np.diff(d['time'])<=.25
                ax.plot(d['t'],np.where(valid,values,np.nan),color=COLORS[model],lw=.8,label=model.title())
                ax.set_xlim(0,run['case']['route']['reference_duration_s'])
                if run['summary']['physical_failure'] and valid.any():
                    last=np.flatnonzero(valid)[-1]
                    ax.scatter(d['t'][last],values[last],marker='x',color=COLORS[model],s=35)
            any_run=next((runs[terrain,route,m] for m in COLORS if (terrain,route,m) in runs),None)
            if any_run:
                trip=any_run['case']['route']['out_and_back']
                ax.axvspan(trip['outbound_end_s'],trip['return_start_s'],color='0.75',alpha=.45)
                ax.axvline(trip['return_start_s'],color='#7541a0',ls='--',lw=.8)
            ax.grid(alpha=.18)
            ax.set_xlabel('Trajectory time (s)',fontsize=8)
            if channel=='tracking':ax.set_ylim(bottom=0)
            else:ax.axhline(0,color='.3',lw=.6)
            if col==0:ax.set_ylabel(TERRAINS[terrain]+'\n'+('XY error (mm)' if channel=='tracking' else 'Body forward speed (m/s)'),fontsize=9)
    handles=[plt.Line2D([],[],color=COLORS[m],label='Ordinary swivel' if m=='swivel' else 'Split ball') for m in COLORS]
    fig.legend(handles=handles,loc='upper left',bbox_to_anchor=(.07,.947),ncol=2,frameon=False)
    title='Measured tracking through the outward and return legs' if channel=='tracking' else 'Does the chassis actually travel backward?'
    fig.suptitle(title,x=.06,y=.98,ha='left',fontsize=18,weight='bold')
    footer='Gray band: stopped turnaround; dashed line: return begins. Panels have independent vertical scales.\n'
    footer+=('Timed XY error against the reference; sampled motion only. Failure ends remain visible.' if channel=='tracking' else
             'Negative speed is chassis-backward travel. Position derivatives use source timestamps; missing intervals are not bridged.')
    fig.text(.06,.028,footer,fontsize=9,color='#435361')
    for ext in ('png','svg','pdf'):fig.savefig(output/f'{channel}_all_cases.{ext}',dpi=180)
    plt.close(fig)


def plot_leg_contrasts(pairs,output):
    fig,axes=plt.subplots(1,2,figsize=(13,6))
    fig.subplots_adjust(left=.16,right=.97,top=.78,bottom=.22,wspace=.3)
    matrices=[]
    for leg in ('outbound','return'):
        a=np.full((4,4),np.nan)
        for p in pairs:
            if p['leg']!=leg or not p['available']:continue
            a[list(TERRAINS).index(p['terrain']),list(ROUTES).index(p['route'])]=p['models']['ball']['xy_rms_mm']-p['models']['swivel']['xy_rms_mm']
        matrices.append(a)
    limit=max([float(np.nanmax(abs(a))) for a in matrices if np.isfinite(a).any()]+[.01])
    for ax,leg,a in zip(axes,('Outward','Return'),matrices):
        im=ax.imshow(a,cmap='RdBu_r',vmin=-limit,vmax=limit,aspect='auto')
        ax.set_xticks(range(4),['Reversal','Tight eight','Lateral','Zigzag'])
        ax.set_yticks(range(4),list(TERRAINS.values()) if ax is axes[0] else [])
        ax.set_title(leg,loc='left',weight='bold')
        for (i,j),v in np.ndenumerate(a):
            failed=any(p['terrain']==list(TERRAINS)[i] and p['route']==list(ROUTES)[j] and
                       p['leg']==('outbound' if leg=='Outward' else 'return') and p.get('available') and
                       not p['both_full_runs_passed'] for p in pairs)
            label=(f'{v:+.2f}'+('*' if failed else '')) if np.isfinite(v) else 'pending'
            ax.text(j,i,label,ha='center',va='center',color='white' if abs(v)>.65*limit else '#213547')
    fig.suptitle('Timed XY RMS: split ball minus ordinary swivel',x=.06,ha='left',fontsize=17,weight='bold')
    fig.text(.06,.86,'Negative / blue: lower ball RMS. Positive / red: lower swivel RMS. Units: mm.',fontsize=10)
    fig.colorbar(im,ax=axes,orientation='horizontal',fraction=.035,pad=.17,label='Common-horizon RMS difference (mm), shared color scale')
    fig.text(.06,.03,'Each leg uses its pair’s common observed window. * At least one full run failed tracking acceptance.\nDo not rank truncated failures by RMS. One execution per configuration; descriptive simulation results.',fontsize=9)
    for ext in ('png','svg','pdf'):fig.savefig(output/f'outbound_return_rms.{ext}',dpi=180)
    plt.close(fig)


def plot_paths(runs,output):
    fig,axes=plt.subplots(4,4,figsize=(13,14))
    fig.subplots_adjust(left=.09,right=.98,top=.88,bottom=.08,wspace=.35,hspace=.38)
    for row,terrain in enumerate(TERRAINS):
        for col,route in enumerate(ROUTES):
            ax=axes[row,col]
            available=[runs[terrain,route,m] for m in COLORS if (terrain,route,m) in runs]
            bounds=[]
            if available:
                reference=np.asarray(available[0]['case']['route']['points_m'])
                bounds.append(reference)
                ax.plot(*reference.T,color='#9aa6b1',lw=3,alpha=.65,zorder=1)
                ax.scatter(*reference[0],marker='o',facecolors='white',edgecolors='#263b4c',s=35,zorder=6)
            for model in COLORS:
                run=runs.get((terrain,route,model))
                if not run:continue
                d=run['data']
                bounds.append(d['xy'][d['motion']])
                for leg,style in [('outbound','-'),('return','--')]:
                    valid=d['motion'] & np.array([s.get('route_leg')==leg for s in d['samples']])
                    valid[1:] &= np.diff(d['time'])<=.25
                    ax.plot(*np.where(valid[:,None],d['xy'],np.nan).T,color=COLORS[model],
                            ls=style,lw=1.1,zorder=3 if model=='swivel' else 2)
                if run['summary']['physical_failure'] and d['motion'].any():
                    ax.scatter(*d['xy'][np.flatnonzero(d['motion'])[-1]],marker='x',
                               color=COLORS[model],s=45,zorder=7)
            if bounds:
                points=np.concatenate(bounds)
                low,high=np.nanmin(points,axis=0),np.nanmax(points,axis=0)
                center=(low+high)/2;half=.6*max(float(np.max(high-low)),1.)
                ax.set_xlim(center[0]-half,center[0]+half)
                ax.set_ylim(center[1]-half,center[1]+half)
            else:
                ax.text(.5,.5,'Recording pending',ha='center',va='center',transform=ax.transAxes)
                ax.set_xticks([]);ax.set_yticks([])
            ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.18)
            if row==0:ax.set_title(ROUTES[route],fontsize=10)
            ax.set_xlabel('World X (m)',fontsize=8)
            ax.set_ylabel((TERRAINS[terrain]+'\n' if col==0 else '')+'World Y (m)',fontsize=9)
    handles=[plt.Line2D([],[],color=COLORS[m],label='Ordinary swivel' if m=='swivel' else 'Split ball') for m in COLORS]
    handles += [plt.Line2D([],[],color='#9aa6b1',lw=3,label='Reference'),
                plt.Line2D([],[],color='#263b4c',ls='-',label='Outward'),
                plt.Line2D([],[],color='#263b4c',ls='--',label='Return')]
    fig.legend(handles=handles,loc='upper left',bbox_to_anchor=(.07,.945),ncol=5,frameon=False)
    fig.suptitle('Measured outward and return paths',x=.06,y=.98,ha='left',fontsize=18,weight='bold')
    fig.text(.06,.026,'Open circle: start and final return target. Equal X/Y scale within each panel.\n'
             'Overlapping traces can hide small differences; use the timed error plots and per-leg metrics to quantify them.',fontsize=9,color='#435361')
    for ext in ('png','svg','pdf'):fig.savefig(output/f'xy_paths_all_cases.{ext}',dpi=180)
    plt.close(fig)


def build(campaign):
    root=campaign/'replays';output=campaign/'report';output.mkdir(exist_ok=True)
    matrix=read(root/'experiment_matrix.json')
    runs,hashes=load_runs(root,matrix)
    attempts=[]
    for path in sorted((root/'attempts').glob('*/run_summary.json')):
        saved=read(path)
        attempts.append({'summary_file':str(path),'terrain':saved['terrain'],'route':saved['shape'],
                         'model':saved['caster'],'trajectory_started':saved.get('motion_start_s') is not None,
                         'physical_failure':saved.get('physical_failure'),
                         'infrastructure_failure':saved.get('infrastructure_failure')})
        hashes[str(path)]=analysis.sha256(path)
    pairs=paired_legs(runs)
    contacts=peak_contact_windows(runs)
    (output/'contact_peak_windows.json').write_text(json.dumps(analysis.finite_json(contacts),indent=2,allow_nan=False)+'\n')
    plt.rcParams.update({'font.family':'DejaVu Sans','svg.fonttype':'none','pdf.fonttype':42,
                         'axes.spines.top':False,'axes.spines.right':False})
    plot_grid(runs,output,'tracking');plot_grid(runs,output,'body_direction');plot_leg_contrasts(pairs,output)
    plot_paths(runs,output)
    rows=[]
    for key,run in runs.items():
        for leg,metrics in run['legs'].items():
            rows.append({'terrain':key[0],'route':key[1],'model':key[2],
                         'full_run_tracking_passed':run['summary']['tracking_passed'],
                         'physical_failure':run['summary']['physical_failure'],**metrics})
    if rows:
        with (output/'leg_metrics.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    data={'expected_runs':len(matrix),'validated_runs':len(runs),'tracking_passes':sum(r['summary']['tracking_passed'] for r in runs.values()),
          'analysis_source_sha256':analysis.sha256(__file__),
          'tracking_passes_by_model':{m:sum(r['summary']['tracking_passed'] for k,r in runs.items() if k[2]==m) for m in COLORS},
          'archived_attempts':attempts,
          'individual_leg_metrics':rows,'paired_common_leg_metrics':pairs,'input_sha256':hashes,
          'backward_definition':'Projection of source-pose XY velocity onto measured chassis +X less than -0.03 m/s. Denominator: observed moving time with reference speed and measured XY speed both >=0.03 m/s.',
          'nearest_path_definition':'Shortest XY distance to any finite reference-polyline segment, ignoring time and traversal order; not verification of the correct branch at a crossing.',
          'scope':'Exact reversed XY references; no prescribed chassis heading. Returned path does not imply chassis-backward driving for its whole duration.'}
    (output/'return_analysis.json').write_text(json.dumps(analysis.finite_json(data),indent=2,allow_nan=False)+'\n')
    def relative(p):return html.escape(os.path.relpath(p,output),quote=True)
    def number(v):return '—' if v is None else f'{v:.2f}'
    blocks=[]
    for terrain,title in TERRAINS.items():
        blocks.append(f'<section id="{terrain}"><h2>{title}</h2><img class="map" src="{relative(campaign/"contacts"/terrain/"detail.png")}" alt="Actual Gazebo {title}">')
        blocks.append(f'<p><a href="{relative(campaign/"contacts"/terrain/"overview.png")}">Whole-map view</a> · <a href="{relative(campaign/"contacts"/terrain/"report.json")}">Contact support validation</a></p>')
        for route,label in ROUTES.items():
            blocks.append(f'<h3>{label}</h3><table><tr><th>Vehicle</th><th>Outward timed RMS / peak (mm)</th><th>Return timed RMS / peak (mm)</th><th>Nearest-path peak, outward / return (mm)</th><th>Return moving time backward (%)</th><th>Full-run outcome</th></tr>')
            for model in COLORS:
                run=runs.get((terrain,route,model))
                if not run:
                    blocks.append(f'<tr><td>{model}</td><td colspan="5">Recording pending</td></tr>');continue
                a,b=run['legs']['outbound'],run['legs']['return']
                state='Passed' if run['summary']['tracking_passed'] else run['summary']['physical_failure'] or 'Outside tracking tolerance'
                blocks.append(f'<tr><td>{model}</td><td>{number(a["xy_rms_mm"])} / {number(a["xy_peak_mm"])}</td><td>{number(b["xy_rms_mm"])} / {number(b["xy_peak_mm"])}</td><td>{number(a["nearest_path_peak_mm"])} / {number(b["nearest_path_peak_mm"])}</td><td>{number(b["body_backward_percent"])}</td><td>{html.escape(state)}</td></tr>')
            blocks.append('</table>')
            event=next((e for e in contacts['windows'] if e['terrain']==terrain and e['route']==route),None)
            if event:
                blocks.append(f'<details><summary>Contact observations around the swivel error peak (t = {event["center_trajectory_s"]:.2f} s)</summary><p>One-second windows at the same trajectory time, selected after observing the swivel peak. Counts are received snapshots containing any contact on that caster assembly.</p><table><tr><th>Vehicle</th><th>Nearby XY error (mm)</th><th>Left caster contact snapshots</th><th>Right caster contact snapshots</th><th>Total received snapshots</th></tr>')
                for model,evidence in event['models'].items():
                    window=evidence['contact_window'];counts=window.get('snapshots_with_assembly_contact',{})
                    pose=evidence['nearest_pose_sample']
                    error=number(pose['xy_error_mm']) if pose['distance_from_requested_time_s']<=.1 else 'No nearby pose'
                    blocks.append(f'<tr><td>{model}</td><td>{error}</td><td>{counts.get("left_caster_assembly",0)}</td><td>{counts.get("right_caster_assembly",0)}</td><td>{window.get("snapshots",0)}</td></tr>')
                blocks.append('</table><p>Contact counts are not contact duration or calibrated wheel load. A missing contact alone does not prove airborne motion. Raw collision names, normals, source gaps and timing offsets are in the linked contact-window JSON.</p></details>')
            clip=campaign/'paired_videos'/f'{terrain}__{route}__camera.mp4'
            closeup=campaign/'paired_videos'/f'{terrain}__{route}__closeup.mp4'
            if clip.is_file():blocks.append(f'<video controls preload="none" src="{relative(clip)}"></video><p><a href="{relative(clip)}">Download synchronized overview MP4</a> · <a href="{relative(closeup)}">Synchronized caster close-up MP4</a></p>')
            blocks.append('<div class="individual">')
            for model in COLORS:
                run=runs.get((terrain,route,model))
                if not run:continue
                directory=run['directory']
                links=' · '.join(f'<a href="{relative(directory/name)}">{label}</a>' for name,label in [('replay.mp4','Annotated replay'),('closeup.mp4','Caster close-up'),('trajectory.csv','Trajectory CSV'),('contact_samples.json','Contact snapshots')])
                blocks.append(f'<p><b>{model.title()}</b>: {links}</p>')
            blocks.append('</div>')
        blocks.append('</section>')
    body=f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rocky terrain outward and return comparison</title><style>body{{font:16px/1.55 system-ui,sans-serif;max-width:1250px;margin:auto;padding:28px;color:#203443;background:#f3f6f8}}section{{background:white;padding:24px;border-radius:12px;margin:24px 0}}img,video{{max-width:100%;width:100%;height:auto}}img.map{{max-width:780px}}table{{border-collapse:collapse;width:100%;margin:12px 0}}td,th{{text-align:left;padding:9px;border-bottom:1px solid #d8e0e6}}th{{background:#eaf0f5}}a{{color:#146590}}.individual{{font-size:14px}}</style>
<h1>Four rocky maps · four routes · outward and return</h1>
<p><b>{len(runs)}/{len(matrix)} validated recordings; {data['tracking_passes']} tracking passes.</b> Both caster configurations use the same terrain geometry, reference, controller and nominal 0.25 m/s speed. Each original route is followed by a 0.75 s stop and its exact time reversal.</p>
<nav>{' · '.join(f'<a href="#{key}">{title}</a>' for key,title in TERRAINS.items())}</nav>
<p>A tracking pass requires completion without a physical abort, peak timed XY error ≤0.5 m and final error ≤0.2 m. This broad acceptance threshold does not imply precise tracking; compare the measured errors below.</p>
<p><b>Timing versus path shape:</b> Timed XY error measures distance from the position commanded at that instant. Nearest-path distance measures the shortest XY distance to any reference segment and ignores timing. It distinguishes a scheduling delay from departure from the geometric route, but cannot verify traversal order or the correct branch at a figure-eight intersection.</p>
<p>The original reversal route already changes travel direction by 180° twice. Its out-and-back version has five travel reversals. A travel reversal does not prescribe a 180° chassis turn. On a closed figure-eight, the robot completes the loop and then traverses it in the opposite direction.</p>
<p><b>Retracing versus backing:</b> The existing offset-drive controller can rotate the chassis during the return. The table reports measured chassis-backward motion, defined as velocity projected onto chassis +X below −0.03 m/s, as a fraction of observed moving time with reference and actual speed at least 0.03 m/s. It does not claim enforced reverse-gear driving.</p>
<p><a href="leg_metrics.csv">Per-leg CSV</a> · <a href="return_analysis.json">Paired common-window metrics and provenance</a> · <a href="contact_peak_windows.json">Contact windows around swivel error peaks</a> · <a href="../replays/comparison.html">Detailed caster analysis</a> · <a href="../README.md">Setup and reproduction</a></p>
<p>Tables use each run's observed leg; the difference chart uses matched common windows. Failed or incomplete runs must not be ranked by shortened RMS alone. One nominal execution per model/cell; estimated mechanical parameters, unequal wheel radii and velocity-controlled actuators limit hardware conclusions.</p>
<p>All four maps passed five guided sphere-support checks each. These checks exercise solid ground, stone tops and edges. They do not calibrate tire friction or motor torque. The new layouts combine a filtered, amplitude-reduced Verti-Arena scan with synthetic fixed stones; loose gravel and deformable soil are not modeled.</p>
'''
    if attempts:
        body+=f'<details><summary>Earlier attempts retained: {len(attempts)}</summary><p>These attempts are archived separately from the 32-run comparison. The summary links preserve the failure reason and whether trajectory motion started.</p><ul>'
        for attempt in attempts:
            label=' / '.join(attempt[k] for k in ('terrain','route','model'))
            reason=attempt['infrastructure_failure'] or attempt['physical_failure'] or 'Incomplete recording'
            body+=f'<li><a href="{relative(Path(attempt["summary_file"]))}">{html.escape(label)}</a>: {html.escape(reason)}. Trajectory started: {attempt["trajectory_started"]}.</li>'
        body+='</ul></details>'
    for name,title in [('outbound_return_rms','Outward / return comparison'),('xy_paths_all_cases','Measured paths across all four maps'),('tracking_all_cases','Tracking error through each run'),('body_direction_all_cases','Actual direction relative to the chassis')]:
        body+=f'<section><h2>{title}</h2><p><a href="{name}.pdf">PDF</a> · <a href="{name}.svg">SVG</a></p><img src="{name}.png" alt="{title}"></section>'
    if (output/'stone_steps_tight8_diagnostics.png').is_file():
        body+='<section><h2>Stone-step tight-eight diagnostics</h2><p>Selected case with a large outward tracking excursion. Joint-axis diagnostics are not measured tire slip.</p><a href="stone_steps_tight8_diagnostics.svg"><img src="stone_steps_tight8_diagnostics.png" alt="Detailed tracking, attitude, command limits and passive caster motion"></a></section>'
    (output/'index.html').write_text(body+'\n'.join(blocks)+'</html>\n')
    print(json.dumps({k:data[k] for k in ('expected_runs','validated_runs','tracking_passes')},indent=2))
    return data


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign',type=Path)
    build(parser.parse_args().campaign.expanduser().resolve())
