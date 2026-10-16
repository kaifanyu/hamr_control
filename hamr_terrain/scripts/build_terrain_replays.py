#!/usr/bin/env python3
"""Assemble measured trace panels, MP4 replays, and an offline campaign gallery."""
import argparse
import csv
import fcntl
import html
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from terrain_replay_paths import SHAPES, sha256
from run_terrain_replay import write_json


def trace_bounds(summary, terrain_bounds):
    """Show local metric routes legibly while retaining historical map views."""
    if summary['shape'].removesuffix('_out_back') == 'map_route':
        points = np.asarray(summary['route']['points_m'], dtype=float)
        return {axis: [float(points[:, i].min()-1.), float(points[:, i].max()+1.)]
                for i, axis in enumerate(('x', 'y'))}
    return {'x': terrain_bounds.get('x', (-3, 3)),
            'y': terrain_bounds.get('y', (-4, 4))}


def finalize_case(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image,ImageDraw,ImageFont
    directory=Path(directory)
    summary=json.loads((directory/'run_summary.json').read_text())
    if not summary['recording_complete']:
        raise ValueError(f'Camera timing/coverage failed: {directory}')
    trajectory=json.loads((directory/'trajectory.json').read_text())
    samples=trajectory['samples']
    p=np.array([s['position_m'][:2] for s in samples if s['phase']!='settling'])
    points=np.array(summary['route']['points_m'])
    fig,axis=plt.subplots(figsize=(4,5.8),dpi=100)
    axis.set_facecolor('#eaf0e7')
    axis.plot(points[:,0],points[:,1],color='#159447',linestyle='--',lw=2,label='Planned')
    round_trip=summary['route'].get('out_and_back')
    if round_trip:
        for leg,color in (('outbound','#b76822'),('return','#7541a0')):
            trace=np.array([s['position_m'][:2] if s.get('route_leg')==leg and s['phase']=='motion' else [np.nan,np.nan] for s in samples])
            axis.plot(trace[:,0],trace[:,1],color=color,lw=1.6,label=leg.title())
    elif len(p):axis.plot(p[:,0],p[:,1],color='#1459bd',lw=1.6,label='Measured')
    axis.scatter(points[0,0],points[0,1],s=35,color='#101827',zorder=5)
    terrain_file=directory/'terrain.json'
    bounds=json.loads(terrain_file.read_text()).get('bounds_m',{}) if terrain_file.exists() else {}
    bounds=trace_bounds(summary,bounds)
    trace_title=('Full-run XY trace (route view)' if summary['shape'].removesuffix('_out_back')=='map_route'
                 else 'Full-run XY trace')
    axis.set(xlim=bounds['x'],ylim=bounds['y'],xlabel='World X (m)',ylabel='World Y (m)',title=trace_title)
    axis.set_aspect('equal');axis.grid(alpha=.25);axis.legend(loc='upper left',fontsize=8)
    fig.tight_layout();fig.savefig(directory/'trajectory.png');plt.close(fig)
    panel=Image.new('RGB',(400,960),'#f5f7fa');draw=ImageDraw.Draw(panel)
    regular='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font=ImageFont.truetype(regular,22);small=ImageFont.truetype(regular,17)
    draw.text((20,20),summary['terrain'],font=font,fill='#122137')
    shape_label=summary['shape'].removeprefix('stress_').removesuffix('_out_back').replace('_',' ').title()
    draw.text((20,56),shape_label+(' + return' if round_trip else ''),font=font,fill='#122137')
    draw.text((20,93),f"{summary['caster'].title()} caster | {summary['route']['speed_m_s']:g} m/s",font=small,fill='#34445a')
    plot=Image.open(directory/'trajectory.png').convert('RGB')
    panel.paste(plot,(0,130))
    metrics=summary['measurements']
    outcome='Tracking passed' if summary['tracking_passed'] else 'Tracking outside tolerance'
    if summary['physical_failure']:outcome=summary['physical_failure'].replace('_',' ').title()
    lines=[outcome,
           f"Path length: {summary['route']['route_length_m']:.2f} m",
           f"Tracking RMS: {metrics['tracking_rmse_m']:.3f} m" if metrics['tracking_rmse_m'] is not None else 'Tracking RMS: unavailable',
           f"Maximum error: {metrics['tracking_max_error_m']:.3f} m" if metrics['tracking_max_error_m'] is not None else 'Maximum error: unavailable',
           'Actual Gazebo camera and dynamics',
           'Full-run trace shown throughout',
           'Playback uses simulation time']
    for i,line in enumerate(lines):draw.text((18,735+28*i),line,font=small,fill='#21334c')
    panel.save(directory/'replay_panel.png')
    movie=directory/'replay.mp4'
    temporary=directory/'replay.encoding.mp4'
    camera=json.loads((directory/'camera.json').read_text())
    filters='[0:v][1:v]hstack=inputs=2:shortest=1[v]'
    if round_trip:
        origin=summary['motion_start_s']-camera['first_sim_time_s']
        intervals=[('OUTBOUND',origin,origin+round_trip['outbound_end_s']),
                   ('STOP - REVERSING PATH',origin+round_trip['outbound_end_s'],origin+round_trip['return_start_s']),
                   ('RETURN - RETRACING PATH',origin+round_trip['return_start_s'],origin+round_trip['return_end_s'])]
        overlays=','.join(f"drawtext=fontfile={regular}:text='{label}':x=20:y=20:fontsize=26:fontcolor=white:box=1:boxcolor=black@0.65:boxborderw=10:enable='gte(t,{a:.6f})*lt(t,{b:.6f})'" for label,a,b in intervals)
        filters=f'[0:v]{overlays}[camera];[camera][1:v]hstack=inputs=2:shortest=1[v]'
    cmd=['ffmpeg','-nostdin','-y','-hide_banner','-loglevel','warning',
         '-i',str(directory/'camera.mp4'),'-loop','1','-framerate','15','-i',str(directory/'replay_panel.png'),
         '-filter_complex',filters,
         '-map','[v]','-an','-c:v','libx264','-threads','2','-preset','veryfast','-crf','20',
         '-pix_fmt','yuv420p','-movflags','+faststart','-shortest',str(temporary)]
    with (directory/'replay_encoding.log').open('w') as log:
        subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=600)
    temporary.replace(movie)
    probe=subprocess.run(['ffprobe','-v','error','-show_entries','stream=codec_name,width,height,nb_frames,r_frame_rate:format=duration,size',
                          '-of','json',str(movie)],text=True,capture_output=True,check=True,timeout=30)
    info=json.loads(probe.stdout)
    camera=json.loads((directory/'camera.json').read_text())
    duration=float(info['format']['duration'])
    expected=camera['expected_video_duration_s']
    if abs(duration-expected)>.15:raise ValueError('Annotated replay duration differs from source camera')
    # Decode every delivered frame once, detecting truncated or corrupt MP4s.
    decoded=subprocess.run(['ffmpeg','-nostdin','-v','error','-threads','2','-i',str(movie),'-f','null','-'],
                           text=True,capture_output=True,timeout=600)
    if decoded.returncode or decoded.stderr.strip():raise ValueError(f'Replay decode failed: {decoded.stderr}')
    subprocess.run(['ffmpeg','-nostdin','-y','-v','error','-ss',str(duration*.5),
                    '-i',str(movie),'-frames:v','1','-q:v','2',str(directory/'preview.jpg')],
                   check=True,capture_output=True,timeout=60)
    evidence={'passed':True,'file':'replay.mp4','sha256':sha256(movie),'ffprobe':info,
              'source_camera_sha256':sha256(directory/'camera.mp4'),
              'full_decode_passed':True,'duration_matches_source':True,
              'panel':'Static full-run measured trace; camera is the actual evolving simulation.'}
    write_json(directory/'replay_validation.json',evidence)
    fields=['simulation_time_s','trajectory_time_s','phase','x_m','y_m','z_m','roll_rad','pitch_rad','yaw_rad','reference_x_m','reference_y_m','position_error_m','route_leg','body_forward_speed_m_s','reference_body_forward_speed_m_s']
    with (directory/'trajectory.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(fields)
        for s in samples:writer.writerow([s['simulation_time_s'],s['trajectory_time_s'],s['phase'],*s['position_m'],*s['base_rpy_rad'],*s['reference_m'],s['position_error_m'],s.get('route_leg'),s.get('body_forward_speed_m_s'),s.get('reference_body_forward_speed_m_s')])
    return evidence


def build_gallery(root):
    root=Path(root).resolve()
    with (root/'.gallery.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        return _build_gallery(root)


def _build_gallery(root):
    root=Path(root).resolve()
    matrix=json.loads((root/'experiment_matrix.json').read_text())
    completed=[];body=[];rows=[]
    for terrain in dict.fromkeys(case['terrain'] for case in matrix):
        body.append(f'<section id="{html.escape(terrain)}"><h2>{html.escape(terrain)}</h2>')
        for shape in dict.fromkeys(c['shape'] for c in matrix if c['terrain']==terrain):
            body.append(f'<h3>{shape.replace("_"," ").title()}</h3><div class="pair">')
            for caster in ('swivel','ball'):
                directory=root/'runs'/terrain/shape/caster
                rel=directory.relative_to(root).as_posix()
                summary_path=directory/'run_summary.json'
                validation=directory/'replay_validation.json'
                if validation.exists():
                    summary=json.loads(summary_path.read_text())
                    evidence=json.loads(validation.read_text())
                    good=evidence.get('passed') and (directory/'replay.mp4').is_file()
                    if good:completed.append(summary)
                    status='Tracking passed' if summary['tracking_passed'] else (summary['physical_failure'] or 'Tracking outside tolerance')
                    rms=summary['measurements']['tracking_rmse_m']
                    end_error=summary['measurements']['final_position_error_m']
                    rms_label=f'{rms:.3f}' if rms is not None else 'unavailable'
                    end_label=f'{end_error:.3f}' if end_error is not None else 'unavailable'
                    poster='preview.jpg' if (directory/'preview.jpg').exists() else 'trajectory.png'
                    body.append(f'<article><h4>{caster.title()} caster — {html.escape(status)}</h4>'
                        f'<video controls preload="none" poster="{rel}/{poster}"><source src="{rel}/replay.mp4" type="video/mp4"></video>'
                        f'<p><a href="{rel}/replay.mp4" download="{terrain}_{shape}_{caster}.mp4">MP4</a> · <a href="{rel}/camera.mp4" download="{terrain}_{shape}_{caster}_camera.mp4">Camera original</a> · '
                        f'<a href="{rel}/trajectory.csv">CSV</a> · <a href="{rel}/trajectory.json">Full feedback</a> · '
                        f'<a href="{rel}/run_summary.json">Results</a> · <a href="{rel}/configuration.json">Configuration</a></p>'
                        f'<p>RMS tracking error {rms_label} m; final error {end_label} m.</p>'
                        +(f'<p><a href="{rel}/closeup.mp4">Right caster close-up MP4</a></p>'
                          if summary.get('closeup_video',{} ) and summary['closeup_video'].get('valid')
                          else '')+'</article>')
                    rows.append({'terrain':terrain,'shape':shape,'caster':caster,'video':f'{rel}/replay.mp4',
                                 'tracking_passed':summary['tracking_passed'],
                                 'route_length_m':summary['route']['route_length_m'],
                                 'video_duration_s':float(evidence['ffprobe']['format']['duration']),
                                 **summary['measurements']})
                else:
                    progress=json.loads((directory/'progress.json').read_text()) if (directory/'progress.json').exists() else {}
                    phase=progress.get('phase','pending')
                    label={'settling':'Robot settling','motion':'Recording trajectory',
                           'final_hold':'Recording final pose','complete':'Encoding and validating replay'}.get(phase,'Pending')
                    if phase=='motion' and progress.get('reference_duration_s',0)>0:
                        percent=max(0,min(100,round(100*progress.get('trajectory_time_s',0)/progress['reference_duration_s'])))
                        label+=f' — {percent}%'
                    if summary_path.exists() and not json.loads(summary_path.read_text()).get('recording_complete'):
                        label='Capture check failed; retry or inspection pending'
                    body.append(f'<article><h4>{caster.title()} caster</h4><p>{html.escape(label)}</p></article>')
            body.append('</div>')
        body.append('</section>')
    total=len(matrix);passed=sum(x['tracking_passed'] for x in completed)
    terrain_count=len({c['terrain'] for c in matrix})
    shape_count=len({c['shape'] for c in matrix})
    speeds=sorted({c['route']['speed_m_s'] for c in matrix})
    speed_note=('Reference speed '+', '.join(f'{speed:g}' for speed in speeds)+' m/s.')
    route_note=('Per-map metric routes are executed on the complete terrain geometry. '
                'The trace panels show the route area with a 1 m margin; these local trials do not establish full-map traversability. '
                'Exact route points, lengths, and timing are saved in the matrix and each run.'
                if any(c['shape'].removesuffix('_out_back')=='map_route' for c in matrix) else
                'The stress routes target reversals, short-radius turns, lateral transitions, and front-wheel loading. '
                'Their exact points, lengths, and timing are saved in the matrix and each run.'
                if any(c['shape'] not in SHAPES and c['shape']!='weave' for c in matrix) else
                'The weaving path spans 4.2 m east–west and 6.4 m north–south, crossing the uneven patch repeatedly.'
                if any(c['shape']=='weave' for c in matrix) else
                'Straight and figure-eight paths span about 6.4/6.2 m north–south; the triangle spans 4.4 × 6.1 m. The 4.4 m circle diameter and square side are limited by map width and vehicle clearance. These are map-scale trajectories, not exhaustive coverage of every surface point.')
    if any(c['terrain']=='full_uneven_flat_reference' for c in matrix):
        route_note+=' The flat reference is a separate control world.'
    navigation=' · '.join(f'<a href="#{html.escape(t)}">{html.escape(t)}</a>' for t in dict.fromkeys(c['terrain'] for c in matrix))
    failures=[run for run in completed if not run['tracking_passed']]
    failure_links=''
    if failures:
        items=[]
        for run in failures:
            rel='runs/{terrain}/{shape}/{caster}/replay.mp4'.format(**run)
            label='{terrain} / {shape} / {caster}'.format(**run).replace('_',' ')
            reason=(run['physical_failure'] or 'tracking outside tolerance').replace('_',' ')
            items.append(f'<li><a href="{rel}">{html.escape(label)}</a> — {html.escape(reason)}</li>')
        failure_links='<h2>Runs outside tracking limits</h2><ul>'+''.join(items)+'</ul>'
    bundle_link='<p><a href="hamr_100_terrain_replays.zip" download>Download all 100 annotated MP4s (ZIP)</a></p>' if (root/'hamr_100_terrain_replays.zip').exists() else ''
    text=f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>HAMR terrain trajectory replays</title><style>body{{font:16px system-ui;margin:0 auto;max-width:1500px;padding:30px;background:#f0f4f8;color:#14243a}}h1{{font-size:32px}}h2{{border-top:2px solid #cbd6e2;padding-top:24px}}.pair{{display:grid;grid-template-columns:1fr 1fr;gap:20px}}article{{background:white;padding:16px;border-radius:8px}}video{{width:100%;background:#101722;aspect-ratio:1.75}}a{{color:#1158a4}}p{{line-height:1.5}}@media(max-width:800px){{.pair{{grid-template-columns:1fr}}}}</style>
<h1>HAMR: {terrain_count} terrains × {shape_count} paths × two caster models</h1>
<p><strong>{len(completed)}/{total} validated replays available.</strong> {passed} currently meet the tracking criterion; {len(completed)-passed} record a tracking or physical failure.</p>
<p>Reload this page for newly finished videos.</p><nav>{navigation}</nav>
<p>{speed_note} Original 1 ms terrain physics and caster mechanics. The green line in Gazebo is a visual-only reference; the side panel shows the <strong>full-run</strong> planned and measured XY paths throughout playback. Videos use simulation-time playback, even when computation is slower.</p>
<p>{route_note}</p>
<p>Tracking passes require completion without a physical abort, maximum timed position error ≤0.5 m, and final error ≤0.2 m. A completed recording can preserve a failed trajectory. Polygon corners are sharp reference corners; curves use dense arc-length samples. Both vehicles receive the same paths and controller limits.</p>
<p><a href="experiment_matrix.json">All {total} requests</a> · <a href="suite_summary.json">Current results</a> · <a href="results.csv">Results CSV</a> · <a href="playlist.m3u">Video playlist</a> · <a href="README.md">Reproduction and limitations</a></p>
{bundle_link}{failure_links}
{''.join(body)}</html>'''
    (root/'index.html').write_text(text)
    report={'expected':total,'validated_replays':len(completed),'complete':len(completed)==total,
            'tracking_passed':passed,'tracking_failed':len(completed)-passed,'runs':rows}
    write_json(root/'suite_summary.json',report)
    fields=['terrain','shape','caster','tracking_passed','route_length_m','video_duration_s',
            'tracking_rmse_m','tracking_max_error_m','final_position_error_m','video']
    with (root/'results.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore')
        writer.writeheader();writer.writerows(rows)
    playlist=['#EXTM3U']
    for row in rows:
        playlist.extend([f"#EXTINF:{row['video_duration_s']:.3f},{row['terrain']} / {row['shape']} / {row['caster']}",row['video']])
    (root/'playlist.m3u').write_text('\n'.join(playlist)+'\n')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path)
    p.add_argument('--case',type=Path)
    p.add_argument('--require-complete',action='store_true')
    a=p.parse_args()
    if a.case:finalize_case(a.case)
    result=build_gallery(a.root)
    print(json.dumps({k:v for k,v in result.items() if k!='runs'},indent=2))
    return 1 if a.require_complete and not result['complete'] else 0


if __name__=='__main__':raise SystemExit(main())
