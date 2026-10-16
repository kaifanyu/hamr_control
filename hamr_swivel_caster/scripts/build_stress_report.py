#!/usr/bin/env python3
"""Build a portable, offline HTML gallery and static exports from a caster suite.

Usage: build_stress_report.py SUITE_DIRECTORY
Reads existing run artifacts without rerunning analysis or changing run folders.
Missing/planned runs and partially written JSON are tolerated. Rebuild after the
sweep completes to include new runs. All links remain relative to the suite.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from urllib.parse import quote

import numpy as np

from analyze_caster_stress import suite_row

CATEGORIES = {
    'trajectory': 'Path shapes', 'initial_heading': 'Initial headings',
    'sensitivity': 'Physical sensitivity', 'speed_control': 'Speed controls',
    'repeat_control': 'Repeat controls',
    'other': 'Other cases',
}
COLORS = {'trajectory': '#2563a6', 'initial_heading': '#8a4fb5',
          'sensitivity': '#ba632b', 'speed_control': '#168076', 'other': '#6c7481'}

SUITE_EVIDENCE = ('FINDINGS.md', 'PHYSICS_ASSUMPTIONS.md',
                  'startup_270_comparison.mp4', 'horizontal_shuttle_comparison.mp4',
                  'tiny_square_slow_comparison.mp4')


def clean(value):
    """Strict JSON, including when an older analyzer emitted NaN."""
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def read_json(path, warnings):
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError) as error:
        warnings.append(f'{path.name}: {error}')
        return {}


def matrix_cases(data):
    if isinstance(data, list):
        return [x for x in data if isinstance(x, dict) and x.get('name')]
    if isinstance(data, dict):
        for key in ('cases', 'experiments', 'matrix'):
            if isinstance(data.get(key), list):
                return matrix_cases(data[key])
    return []


def relative_link(path, suite):
    return quote(path.relative_to(suite).as_posix(), safe='/')


def finite(value):
    return isinstance(value, (float, int)) and math.isfinite(value)


def thin_samples(samples, maximum=1800):
    """Retain phase boundaries and endpoints while bounding embedded HTML size."""
    valid = [s for s in samples if isinstance(s, dict) and
             finite(s.get('elapsed_s')) and len(s.get('position_m', [])) >= 2 and
             all(finite(v) for v in s['position_m'][:2])]
    if not valid:
        return []
    selected = set(np.linspace(0, len(valid)-1, min(maximum, len(valid)), dtype=int))
    for i in range(1, len(valid)):
        if valid[i].get('phase') != valid[i-1].get('phase'):
            selected.update((i-1, i))
    yaws = np.unwrap([s.get('base_rpy_rad', [0., 0., 0.])[2] for s in valid])
    rows = []
    for i in sorted(selected):
        s = valid[i]
        ref = s.get('reference_m', [None, None])
        rows.append([s['elapsed_s'], *s['position_m'][:2], *ref[:2],
                     float(np.degrees(yaws[i])), s.get('phase', 'unknown')])
    return rows


def failure_reasons(run, trajectory):
    reasons = []
    if run.get('failure'):
        reasons.append(str(run['failure']))
    for reason in trajectory.get('summary', {}).get('failures', []):
        reasons.append(str(reason))
    startup = trajectory.get('startup_alignment', {})
    if startup.get('failure'):
        reasons.append('Startup: ' + str(startup['failure']))
    for name, camera in run.get('videos', {}).items():
        if camera.get('passed') is False:
            detail = camera.get('failure') or camera.get('motion_quality', {}).get('failure') or 'recording check failed'
            reasons.append(f'{name}: {detail}')
    for name in ('heading_measurements', 'caster_measurements', 'corner_acceptance'):
        check = run.get(name)
        if isinstance(check, dict) and check.get('passed') is False:
            reasons.append(f'{name}: recorded acceptance check failed')
    return list(dict.fromkeys(reasons))


def read_case(suite, directory, specification):
    warnings = []
    trajectory = read_json(directory/'trajectory.json', warnings)
    run = read_json(directory/'run_summary.json', warnings)
    stress = read_json(directory/'caster_stress.json', warnings)
    heading = read_json(directory/'heading_summary.json', warnings) or run.get('heading_measurements', {})
    startup = trajectory.get('startup_alignment', {})
    config = trajectory.get('configuration', {})
    name = directory.name
    category = specification.get('category', 'other')
    row = {'name': name, 'category': category, 'run_directory': directory.relative_to(suite).as_posix(),
           'run_passed': run.get('passed'), 'completed': trajectory.get('completed'),
           'initial_yaw_deg': specification.get('initial_yaw_deg', run.get('initial_chassis_yaw_deg')),
           'speed_m_s': specification.get('speed_m_s', trajectory.get('speed_m_s')),
           'route_length_m': specification.get('route_length_m', trajectory.get('summary', {}).get('route_length_m')),
           'tracking_rms_m': trajectory.get('summary', {}).get('motion_rmse_m'),
           'tracking_max_m': trajectory.get('summary', {}).get('motion_max_error_m'),
           'final_position_error_m': trajectory.get('summary', {}).get('final_position_error_m'),
           'startup_enabled': startup.get('enabled', False), 'startup_passed': startup.get('passed'),
           'startup_duration_s': startup.get('duration_s'),
           'startup_actual_yaw_change_deg': math.degrees(startup['actual_chassis_yaw_change_rad']) if finite(startup.get('actual_chassis_yaw_change_rad')) else None,
           'startup_displacement_m': math.hypot(*startup['preparation_displacement_m'][:2]) if startup.get('preparation_displacement_m') else None,
           'first_leg_final_heading_error_deg': heading.get('first_leg_final_heading_error_deg'),
           'first_leg_forward_motion_fraction': heading.get('forward_motion_fraction_first_leg')}
    if stress:
        try:
            row.update(suite_row(stress))
            row['run_directory'] = directory.relative_to(suite).as_posix()
        except (KeyError, TypeError, ValueError) as error:
            warnings.append(f'Incomplete caster analysis: {error}')
    status = 'passed' if run.get('passed') is True else 'failed' if run.get('passed') is False else 'partial'
    reasons = failure_reasons(run, trajectory)
    if status == 'failed' and not reasons:
        reasons = ['Run acceptance failed; inspect the linked run summary for details.']
    row['status'] = status
    row['failure_reasons'] = '; '.join(reasons)
    files = {}
    for filename in ('overview_annotated.mp4', 'overview.mp4', 'caster_closeup.mp4',
                     'caster_stress.png', 'caster_stress.json', 'caster_stress.csv',
                     'trajectory_and_casters.png', 'trajectory.json', 'run_summary.json',
                     'heading_summary.json', 'configuration.yaml', 'physical_properties.yaml',
                     'robot.urdf', 'world.sdf', 'commands.json', 'gazebo.log', 'trajectory.log'):
        path = directory/filename
        if path.is_file() and path.stat().st_size:
            files[filename] = relative_link(path, suite)
    for side in ('left', 'right'):
        error = startup.get('final_caster_heading_errors_rad', {}).get(side)
        row[f'startup_{side}_caster_error_deg'] = math.degrees(error) if finite(error) else None
    samples = thin_samples(trajectory.get('samples', []))
    return clean({'name': name, 'category': category, 'status': status,
                  'metrics': row, 'failures': reasons, 'warnings': warnings, 'files': files,
                  'route': trajectory.get('requested_route_points_m', specification.get('points_m', config.get('points_m', []))),
                  'executed_route': trajectory.get('executed_route_points_m', config.get('points_m', [])),
                  'samples': samples, 'startup': startup, 'heading': heading,
                  'analysis_present': bool(stress), 'has_trajectory': bool(trajectory.get('samples')),
                  'interpretation': stress.get('interpretation', [])})


def collect_suite(suite):
    suite = Path(suite).resolve()
    warnings = []
    matrix = matrix_cases(read_json(suite/'experiment_matrix.json', warnings))
    specifications = {str(case['name']): case for case in matrix}
    record = suite/'record'
    directories = {p.name: p for p in record.iterdir() if p.is_dir()} if record.is_dir() else {}
    # Matrix order first; posthoc folders without a matrix entry follow by name.
    ordered = [name for name in specifications if name in directories]
    ordered += sorted(set(directories)-set(ordered))
    cases = [read_case(suite, directories[name], specifications.get(name, {})) for name in ordered]
    categories = sorted(set(CATEGORIES) | {case['category'] for case in cases})
    counts = {}
    for category in categories:
        group = [case for case in cases if case['category'] == category]
        planned = [case for case in matrix if case.get('category', 'other') == category]
        counts[category] = {'planned': len(planned), 'present': len(group),
                            'with_trajectory': sum(case['has_trajectory'] for case in group),
                            'completed': sum(case['metrics'].get('completed') is True for case in group),
                            'passed': sum(case['status'] == 'passed' for case in group)}
    shapes = {json.dumps(case['route']) for case in cases
              if case['category'] == 'trajectory' and case['has_trajectory'] and case['route']}
    return clean({'schema_version': 1, 'generated_utc': datetime.now(timezone.utc).isoformat(),
                  'suite_name': suite.name, 'counts_by_category': counts,
                  'path_shapes_with_trajectory': len(shapes),
                  'planned_case_count': len(matrix), 'present_case_count': len(cases),
                  'missing_cases': [name for name in specifications if name not in directories],
                  'suite_files': {name: relative_link(suite/name, suite) for name in SUITE_EVIDENCE
                                  if (suite/name).is_file()},
                  'warnings': warnings, 'cases': cases,
                  'interpretation': 'Caster alignment angles and planar slip proxies describe this estimated rigid-body model. They do not establish hardware defects. Run acceptance is separate from caster alignment.'})


def static_exports(suite, report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False,
                         'savefig.facecolor': 'white'})
    cases = report['cases']
    measured = [c for c in cases if c['has_trajectory']]
    columns = 4
    nrows = max(1, math.ceil(len(measured)/columns))
    fig, axes = plt.subplots(nrows, columns, figsize=(15, 3.1*nrows), squeeze=False)
    for ax in axes.flat:
        ax.set_visible(False)
    for ax, case in zip(axes.flat, measured):
        ax.set_visible(True)
        rows = case['samples']
        route = np.asarray(case['route'])
        if route.ndim == 2 and route.shape[1] >= 2:
            ax.plot(route[:, 0], route[:, 1], '--', c='#b2bac2', lw=1.1, label='Requested route')
        if rows:
            xy = np.array([[s[1], s[2]] for s in rows])
            ref = np.array([[s[3], s[4]] for s in rows if finite(s[3]) and finite(s[4])])
            if len(ref):
                ax.plot(ref[:, 0], ref[:, 1], '--', c='#545e69', lw=1.0, label='Executed reference')
            ax.plot(xy[:, 0], xy[:, 1], c='#2768ac', lw=1.25, label='Measured')
            startup = ['startup' in s[6] or 'align' in s[6] for s in rows]
            if any(startup):
                p = xy[startup]
                ax.plot(p[:, 0], p[:, 1], c='#df9326', lw=1.9, label='Startup')
            ax.scatter(*xy[0], s=15, c='#168076', zorder=4)
        ax.set_title(case['name'].replace('_', ' '), fontsize=9)
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=.18)
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    if measured:
        axes.flat[0].legend(fontsize=7, loc='best')
    else:
        axes.flat[0].set_visible(True)
        axes.flat[0].text(.5, .5, 'No completed trajectory data yet', ha='center', transform=axes.flat[0].transAxes)
        axes.flat[0].set_axis_off()
    fig.suptitle('Measured paths, requested routes, and startup preparation', fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, .97))
    fig.savefig(suite/'trajectory_montage.png', dpi=150)
    plt.close(fig)

    measured = [c for c in cases if finite(c['metrics'].get('tracking_rms_m'))]
    fig, axes = plt.subplots(1, 3, figsize=(16, max(4, .33*len(measured)+1.6)), sharey=True)
    y = np.arange(len(measured))
    colors = [COLORS.get(c['category'], COLORS['other']) for c in measured]
    for ax, metric, scale, title in zip(axes,
        ['tracking_rms_m', 'local_wheel_rolling_axis_p95_deg', 'lateral_slip_rms_m_s'],
        [100., 1., 100.], ['Moving path RMS error (cm)', 'Wheel-axis alignment p95 (deg)', 'Lateral slip proxy RMS (cm/s)']):
        values = []
        for case in measured:
            row = case['metrics']
            if metric == 'tracking_rms_m':
                value = row.get(metric)
            else:
                both = [row.get(side+'_'+metric) for side in ('left', 'right')]
                present = [v for v in both if finite(v)]
                value = max(present) if present else None
            values.append(value*scale if finite(value) else np.nan)
        ax.barh(y, values, color=colors)
        ax.set_title(title)
        ax.grid(axis='x', alpha=.2)
        ax.set_axisbelow(True)
        for pos, val in enumerate(values):
            if not finite(val):
                ax.text(0, pos, 'analysis pending', va='center', fontsize=7, color='#777')
    axes[0].set_yticks(y, [c['name'] for c in measured])
    axes[0].invert_yaxis()
    if not measured:
        axes[0].text(.1, .5, 'No measured metrics yet', transform=axes[0].transAxes)
    fig.suptitle('Moving-phase comparison · alignment and slip use the larger of the two casters', fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .97))
    fig.savefig(suite/'suite_comparison.png', dpi=150)
    plt.close(fig)


HTML = r'''<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>HAMR caster experiments</title>
<style>
:root{--ink:#14283c;--muted:#5c6e7f;--blue:#2563a6;--line:#d9e1e8;--bg:#f3f6f8;--gold:#d68b22}*{box-sizing:border-box}body{margin:0;font:15px/1.5 system-ui,sans-serif;color:var(--ink);background:var(--bg)}header{background:#11283e;color:white;padding:25px 34px}h1{font-size:27px;margin:0 0 6px}h2{font-size:20px;margin:0 0 12px}h3{font-size:16px;margin:0 0 9px}p{margin:6px 0 13px}header p{color:#c4d4e3;margin:0}.pill{display:inline-block;border:1px solid #496277;border-radius:20px;padding:3px 12px;margin:12px 6px 0 0;font-size:13px}a{color:#1e65a3;text-decoration:none}a:hover{text-decoration:underline}.layout{display:grid;grid-template-columns:265px minmax(0,1fr);gap:22px;max-width:1680px;margin:auto;padding:24px}aside{align-self:start;position:sticky;top:15px;max-height:95vh;overflow:auto}.panel{background:white;border:1px solid var(--line);border-radius:12px;padding:20px;margin-bottom:18px;box-shadow:0 3px 12px #17324d06}input,select{padding:9px;width:100%;border:1px solid #c5d0d9;border-radius:7px;font:inherit;margin-bottom:9px}.case{width:100%;text-align:left;cursor:pointer;background:white;color:var(--ink);border:1px solid var(--line);padding:10px 11px;margin:0 0 6px;border-radius:8px;font-size:13px}.case.active{border-color:var(--blue);background:#eaf3fc}.case small{display:block;color:var(--muted);font-size:11px;margin-top:3px}.status{display:inline-block;font-size:12px;font-weight:600;border-radius:5px;padding:3px 8px;background:#eef2f5}.status.passed{color:#176649;background:#e2f3e9}.status.failed{color:#a44825;background:#fff0e5}.status.partial{color:#7d6b28;background:#fff7dc}.muted{color:var(--muted);font-size:13px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:18px}.stats{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}.stat{padding:14px;background:#f2f6fa;border-radius:8px}.stat b{display:block;font-size:23px;font-weight:650}.stat span{color:var(--muted);font-size:12px}.plot{width:100%;height:auto;display:block;background:#fbfcfd;border-radius:8px}video{width:100%;background:#0b1721;border-radius:8px;aspect-ratio:16/9}.legend{font-size:12px;color:var(--muted)}.legend span{margin-right:12px}dl{display:grid;grid-template-columns:1fr 1fr;gap:5px 15px;margin:0;font-size:13px}dt{color:var(--muted)}dd{margin:0;text-align:right;font-variant-numeric:tabular-nums}.notice{border-left:3px solid var(--gold);background:#fff9ee;padding:10px 14px;font-size:13px;margin:12px 0}.files{display:flex;gap:8px 18px;flex-wrap:wrap;font-size:13px}details{margin-top:12px}summary{cursor:pointer;color:var(--blue)}pre{white-space:pre-wrap;word-break:break-word;background:#f3f6f8;padding:12px;border-radius:7px;font-size:12px}.figurelink img{width:100%;border:1px solid var(--line);border-radius:8px}button.control{padding:7px 12px;border:1px solid #c5d0d9;background:white;border-radius:6px;cursor:pointer}.empty{padding:32px;color:var(--muted)}@media(max-width:1000px){.layout{grid-template-columns:205px 1fr;padding:14px;gap:14px}.grid{grid-template-columns:1fr}.stats{grid-template-columns:1fr 1fr}}@media(max-width:650px){.layout{display:block}aside{position:static;max-height:270px;margin-bottom:20px}header{padding:20px}.panel{padding:14px}}
</style>
<header><h1>HAMR · passive caster experiments</h1><p id="suite"></p><div id="counts"></div></header>
<div class="layout"><aside><input id="search" placeholder="Find a case…" aria-label="Find a case"><select id="category" aria-label="Category"><option value="all">All categories</option></select><div id="cases"></div><p class="muted" id="pending"></p></aside>
<main><div class="panel"><h2 id="name">Select an experiment</h2><span id="status"></span><p id="subtitle" class="muted"></p><div id="stats" class="stats"></div><div id="failures"></div></div>
<div class="grid"><section class="panel"><h3>Measured path and requested route</h3><svg id="path" class="plot" viewBox="0 0 640 430" role="img" aria-label="Trajectory plot"></svg><div class="legend"><span style="color:#b2bac2">┄ requested route</span><span style="color:#545e69">┄ executed reference</span><span style="color:#2563a6">━ measured motion</span><span style="color:#d68b22">━ startup preparation</span><span style="color:#168076">● start</span></div></section>
<section class="panel"><h3>Heading and startup evidence</h3><svg id="heading" class="plot" viewBox="0 0 640 260" role="img" aria-label="Measured chassis heading over time"></svg><dl id="startup"></dl><p id="startupPolicy" class="muted"></p><details><summary>Startup and heading measurements</summary><pre id="startupRaw"></pre></details></section></div>
<section class="panel"><h3>Simulation recordings</h3><p class="muted">Overview uses the annotated recording when available. Each camera preserves its recorded timing; start-both control is approximate.</p><button class="control" id="playBoth">Play both from start</button><div class="grid" style="margin-top:12px"><div><h3>Vehicle overview</h3><video id="overview" controls preload="metadata"></video><p id="overviewEmpty" class="muted"></p></div><div><h3>Right caster close-up</h3><video id="closeup" controls preload="metadata"></video><p id="closeupEmpty" class="muted"></p></div></div></section>
<section class="panel"><h3>Moving-phase caster metrics</h3><div class="grid"><dl id="left"></dl><dl id="right"></dl></div><p class="muted">Rolling-axis angles treat forward and backward rolling as equivalent. Directed angles distinguish a trailing caster from a leading caster. Slip values are finite-difference planar proxies, not measured contact forces.</p><div id="analysisLink"></div><details><summary>All recorded metrics and interpretation</summary><pre id="metricsRaw"></pre></details></section>
<section class="panel"><h3>Case files</h3><div id="files" class="files"></div><p id="warnings" class="muted"></p></section>
<section class="panel"><h3>Suite exports</h3><p id="suiteEvidence" class="files"></p><p class="files"><a href="suite_summary.csv">Summary CSV</a><a href="suite_summary.json">Summary JSON</a><a href="experiment_matrix.json">Experiment matrix</a><a href="suite_comparison.png">Comparison PNG</a><a href="trajectory_montage.png">Trajectory montage PNG</a></p><details><summary>View comparison chart</summary><a class="figurelink" href="suite_comparison.png"><img loading="lazy" src="suite_comparison.png" alt="Comparison of trajectory error, caster alignment, and slip proxies"></a></details><details><summary>View trajectory montage</summary><a class="figurelink" href="trajectory_montage.png"><img loading="lazy" src="trajectory_montage.png" alt="Trajectory montage"></a></details><p class="muted">Estimated rigid-body physics with passive casters and ideal drive velocity actuators. These experiments describe simulation behavior; they do not establish a hardware defect. Run acceptance and caster alignment are separate measures.</p></section>
</main></div>
<script id="report-data" type="application/json">__REPORT_DATA__</script>
<script>
'use strict';
const report=JSON.parse(document.getElementById('report-data').textContent), cases=report.cases;
const labels={trajectory:'Path shapes',initial_heading:'Initial headings',sensitivity:'Physical sensitivity',speed_control:'Speed controls',other:'Other cases'};
const el=id=>document.getElementById(id), fmt=(x,n=2)=>typeof x==='number'&&Number.isFinite(x)?x.toFixed(n):'—';
let current=null;
function textNode(tag,text,cls){const e=document.createElement(tag);e.textContent=text;if(cls)e.className=cls;return e;}
function link(name,url){const a=textNode('a',name);a.href=url;return a;}
function dl(target,pairs){target.replaceChildren();pairs.forEach(([a,b])=>{target.append(textNode('dt',a),textNode('dd',b));});}
function stat(label,value){const d=document.createElement('div');d.className='stat';d.append(textNode('b',value),textNode('span',label));return d;}
function svgNode(tag,attrs={}){const n=document.createElementNS('http://www.w3.org/2000/svg',tag);Object.entries(attrs).forEach(([k,v])=>n.setAttribute(k,v));return n;}
function plot(target, series, width, height, xlabel,ylabel,equal){
 target.replaceChildren();const points=series.flatMap(s=>s.points).filter(p=>p.every(Number.isFinite));if(!points.length){const t=svgNode('text',{x:30,y:50,fill:'#5c6e7f'});t.textContent='Measured data not available yet';target.append(t);return;}
 let xmin=Math.min(...points.map(p=>p[0])),xmax=Math.max(...points.map(p=>p[0])),ymin=Math.min(...points.map(p=>p[1])),ymax=Math.max(...points.map(p=>p[1]));
 let dx=Math.max(xmax-xmin,.1),dy=Math.max(ymax-ymin,.1),cx=(xmin+xmax)/2,cy=(ymin+ymax)/2;dx*=1.18;dy*=1.18;
 const l=58,r=20,t=20,b=43,W=width-l-r,H=height-t-b;if(equal){const scale=Math.max(dx/W,dy/H);dx=W*scale;dy=H*scale;}xmin=cx-dx/2;ymin=cy-dy/2;
 const X=x=>l+(x-xmin)/dx*W,Y=y=>height-b-(y-ymin)/dy*H;
 for(let i=0;i<=4;i++){const x=xmin+dx*i/4,y=ymin+dy*i/4;target.append(svgNode('line',{x1:X(x),y1:t,x2:X(x),y2:height-b,stroke:'#e4eaf0'}),svgNode('line',{x1:l,y1:Y(y),x2:width-r,y2:Y(y),stroke:'#e4eaf0'}));const tx=svgNode('text',{x:X(x),y:height-b+19,'text-anchor':'middle',fill:'#5c6e7f','font-size':11});tx.textContent=fmt(x,dx>100?0:2);const ty=svgNode('text',{x:l-7,y:Y(y)+4,'text-anchor':'end',fill:'#5c6e7f','font-size':11});ty.textContent=fmt(y,dy>100?0:2);target.append(tx,ty);}
 series.forEach(s=>{const chunks=[];let chunk=[];s.points.forEach(p=>{if(p.every(Number.isFinite)){chunk.push(p);}else if(chunk.length){chunks.push(chunk);chunk=[];}});if(chunk.length)chunks.push(chunk);chunks.forEach(ps=>target.append(svgNode('polyline',{points:ps.map(p=>X(p[0])+','+Y(p[1])).join(' '),fill:'none',stroke:s.color,'stroke-width':s.width||2,'stroke-dasharray':s.dash||''})));});
 const first=series.find(s=>s.start&&s.points.length);if(first){const p=first.points[0];target.append(svgNode('circle',{cx:X(p[0]),cy:Y(p[1]),r:4,fill:'#168076'}));}
 const a=svgNode('text',{x:l+W/2,y:height-5,'text-anchor':'middle',fill:'#5c6e7f','font-size':12});a.textContent=xlabel;const c=svgNode('text',{x:8,y:12,fill:'#5c6e7f','font-size':11});c.textContent=ylabel;target.append(a,c);
}
function startupPhase(p){return p.includes('startup')||p.includes('align');}
function setVideo(id,source,empty){const v=el(id);v.pause();if(source){v.src=source;v.style.display='block';el(empty).textContent='';}else{v.removeAttribute('src');v.style.display='none';el(empty).textContent='Recording not available in this partial run.';}v.load();}
function select(name){const c=cases.find(c=>c.name===name);if(!c)return;current=c;location.hash=encodeURIComponent(name);renderList();const m=c.metrics,s=c.startup||{};el('name').textContent=c.name.replaceAll('_',' ');el('status').replaceChildren(textNode('span',c.status==='passed'?'Run acceptance passed':c.status==='failed'?'Run acceptance failed':'Partial run','status '+c.status));el('subtitle').textContent=(labels[c.category]||c.category)+' · requested start '+fmt(m.initial_yaw_deg,0)+'° · speed '+fmt(m.speed_m_s,2)+' m/s · route '+fmt(m.route_length_m,2)+' m';
 el('stats').replaceChildren(stat('Moving path RMS',fmt(m.tracking_rms_m*100,2)+' cm'),stat('Maximum path error',fmt(m.tracking_max_m*100,2)+' cm'),stat('Final position error',fmt(m.final_position_error_m*100,2)+' cm'),stat('Observed duration',fmt(m.duration_s,1)+' s'));
 // Missing numerical fields must remain unknown, not become zero through null coercion.
 const missing=[m.tracking_rms_m,m.tracking_max_m,m.final_position_error_m,m.duration_s];[...el('stats').children].forEach((d,i)=>{if(typeof missing[i]!=='number')d.querySelector('b').textContent='—';});
 el('failures').replaceChildren();c.failures.forEach(f=>el('failures').append(textNode('p',f,'notice')));if(c.status==='partial')el('failures').append(textNode('p','This run is incomplete or its final summary is not available. Rebuild the report after the sweep finishes.','notice'));
 const xy=c.samples.map(s=>[s[1],s[2]]),motion=c.samples.map(s=>startupPhase(s[6])?[NaN,NaN]:[s[1],s[2]]),startup=c.samples.map(s=>startupPhase(s[6])?[s[1],s[2]]:[NaN,NaN]);
 plot(el('path'),[{points:c.route,color:'#b2bac2',dash:'3 4',width:1.7},{points:c.samples.map(s=>[s[3],s[4]]),color:'#545e69',dash:'7 4',width:1.4},{points:xy,color:'#b5cce2',width:1,start:true},{points:motion,color:'#2563a6',width:2.2},{points:startup,color:'#d68b22',width:2.8}],640,430,'x (m)','y (m)',true);
 plot(el('heading'),[{points:c.samples.map(s=>[s[0],s[5]]),color:'#2563a6'},{points:c.samples.map(s=>startupPhase(s[6])?[s[0],s[5]]:[NaN,NaN]),color:'#d68b22',width:2.6}],640,260,'elapsed simulation time (s)','unwrapped yaw (°)',false);
 dl(el('startup'),[['Preparation enabled',s.enabled?'Yes':'No'],['Preparation accepted',s.passed===true?'Yes':s.passed===false?'No':'—'],['Preparation duration',fmt(s.duration_s)+' s'],['Measured chassis rotation',fmt(m.startup_actual_yaw_change_deg)+'°'],['Preparation displacement',fmt(m.startup_displacement_m,3)+' m'],['Left final caster error',fmt(m.startup_left_caster_error_deg)+'°'],['Right final caster error',fmt(m.startup_right_caster_error_deg)+'°'],['First-leg final heading error',fmt(m.first_leg_final_heading_error_deg)+'°']]);
 el('startupPolicy').textContent=s.path_policy||'Measured startup samples remain visible in the orange path and heading traces.';el('startupRaw').textContent=JSON.stringify({startup:s,heading:c.heading},null,2);
 setVideo('overview',c.files['overview_annotated.mp4']||c.files['overview.mp4'],'overviewEmpty');setVideo('closeup',c.files['caster_closeup.mp4'],'closeupEmpty');
 ['left','right'].forEach(side=>dl(el(side),[[side.toUpperCase()+' CASTER',''],['Wheel rolling-axis p95',fmt(m[side+'_local_wheel_rolling_axis_p95_deg'])+'°'],['Wheel directed-angle p95',fmt(m[side+'_local_wheel_directed_p95_deg'])+'°'],['Moving time axis >30°',typeof m[side+'_local_wheel_rolling_axis_fraction_gt30']==='number'?fmt(m[side+'_local_wheel_rolling_axis_fraction_gt30']*100,1)+'%':'—'],['Longest axis >30° episode',fmt(m[side+'_local_wheel_rolling_axis_longest_gt30_s'])+' s'],['Lateral slip proxy RMS',fmt(m[side+'_lateral_slip_rms_m_s'],4)+' m/s'],['Lateral distance proxy',fmt(m[side+'_lateral_distance_proxy_m'],3)+' m']]));
 el('analysisLink').replaceChildren();if(c.files['caster_stress.png'])el('analysisLink').append(link('Open caster time-series plot',c.files['caster_stress.png']));else el('analysisLink').append(textNode('p','Caster analysis has not been generated yet.','muted'));el('metricsRaw').textContent=JSON.stringify({metrics:m,interpretation:c.interpretation},null,2);el('files').replaceChildren();Object.entries(c.files).forEach(([name,url])=>el('files').append(link(name,url)));el('warnings').textContent=c.warnings.join(' · ');
}
function renderList(){const q=el('search').value.toLowerCase(),cat=el('category').value;el('cases').replaceChildren();cases.filter(c=>(cat==='all'||c.category===cat)&&c.name.toLowerCase().includes(q)).forEach(c=>{const b=textNode('button',c.name.replaceAll('_',' '),'case'+(current?.name===c.name?' active':''));b.append(textNode('small',(labels[c.category]||c.category)+' · '+c.status));b.onclick=()=>select(c.name);el('cases').append(b);});}
const evidenceLabels={'FINDINGS.md':'Findings and conclusions','PHYSICS_ASSUMPTIONS.md':'Physics assumptions','startup_270_comparison.mp4':'270° startup comparison','horizontal_shuttle_comparison.mp4':'Horizontal shuttle comparison','tiny_square_slow_comparison.mp4':'Slow tiny-square caster comparison'};Object.entries(report.suite_files||{}).forEach(([name,url])=>el('suiteEvidence').append(link(evidenceLabels[name]||name,url)));
el('suite').textContent=report.suite_name+' · generated '+new Date(report.generated_utc).toLocaleString();Object.entries(report.counts_by_category).filter(([k,v])=>v.planned||v.present).forEach(([k,v])=>{const p=textNode('span',(labels[k]||k)+': '+v.with_trajectory+' observed / '+Math.max(v.planned,v.present)+' planned','pill');el('counts').append(p);const o=textNode('option',labels[k]||k);o.value=k;el('category').append(o);});el('pending').textContent=report.missing_cases.length+' planned runs not present. '+report.path_shapes_with_trajectory+' distinct path-shape routes have measurements.';el('search').oninput=renderList;el('category').onchange=renderList;el('playBoth').onclick=()=>{['overview','closeup'].forEach(id=>{const v=el(id);if(v.getAttribute('src')){v.currentTime=0;v.play().catch(()=>{});}});};window.onhashchange=()=>{const name=decodeURIComponent(location.hash.slice(1));if(current?.name!==name)select(name);};renderList();if(cases.length){const initial=decodeURIComponent(location.hash.slice(1));select(cases.some(c=>c.name===initial)?initial:cases[0].name);}else el('name').textContent='No run folders are available yet';
</script></html>'''


def build_report(suite):
    suite = Path(suite).resolve()
    if not suite.is_dir():
        raise ValueError(f'Suite directory does not exist: {suite}')
    report = collect_suite(suite)
    # Keep machine-readable summary compact; the HTML alone embeds plot samples.
    summary = {**report, 'cases': [{k: v for k, v in case.items() if k != 'samples'} for case in report['cases']]}
    (suite/'suite_summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    rows = [case['metrics'] for case in report['cases']]
    fields = list(dict.fromkeys(key for row in rows for key in row)) or ['name', 'status']
    with (suite/'suite_summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)
    static_exports(suite, report)
    embedded = json.dumps(report, separators=(',', ':'), allow_nan=False).replace('<', '\\u003c').replace('>', '\\u003e').replace('&', '\\u0026')
    (suite/'index.html').write_text(HTML.replace('__REPORT_DATA__', embedded))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite_directory', type=Path)
    args = parser.parse_args()
    result = build_report(args.suite_directory)
    print(json.dumps({'index': str(args.suite_directory.resolve()/'index.html'),
                      'present_cases': result['present_case_count'],
                      'planned_cases': result['planned_case_count'],
                      'path_shapes_with_trajectory': result['path_shapes_with_trajectory']}, indent=2))


if __name__ == '__main__':
    main()
