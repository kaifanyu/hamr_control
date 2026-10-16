#!/usr/bin/env python3
"""Render two saved Gazebo videos side by side on a common trajectory clock.

Source recordings are read-only. Camera metadata must describe the recorder's
constant-frame-rate simulation timeline. Source hashes, trim offsets, encoder
command and full-decode verification are written beside the output MP4.
"""
import argparse
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda:handle.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def probe(path, executable):
    result=subprocess.run([executable,'-v','error','-select_streams','v:0',
        '-show_entries','stream=width,height,codec_name,r_frame_rate,nb_frames:format=duration',
        '-of','json',str(path)],capture_output=True,text=True,check=True,timeout=30)
    return json.loads(result.stdout)


def input_description(directory, model, source, ffprobe):
    directory=Path(directory).expanduser().resolve()
    video=directory/(source+'.mp4')
    metadata_path=directory/(source+'.json')
    summary_path=directory/'run_summary.json'
    summary,metadata=read(summary_path),read(metadata_path)
    if not video.is_file():
        raise ValueError(f'Missing source recording: {video}')
    if summary.get('caster',model)!=model:
        raise ValueError(f'{directory} is not the requested {model} model')
    first,start=(metadata.get('first_sim_time_s'),summary.get('motion_start_s'))
    if not all(finite(x) for x in (first,start)):
        raise ValueError(f'Missing finite camera simulation origin or motion-start timestamp: {directory}')
    info=probe(video,ffprobe)
    fps=float(Fraction(info['streams'][0]['r_frame_rate']))
    duration=float(info['format']['duration'])
    expected=metadata.get('expected_video_duration_s')
    if not finite(expected) or abs(duration-expected)>max(.15,1/fps):
        raise ValueError(f'Video duration disagrees with its timing metadata: {video}')
    declared_fps=metadata.get('fps')
    if not finite(declared_fps) or not math.isclose(fps,declared_fps,abs_tol=1e-6):
        raise ValueError(f'Video frame rate disagrees with its timing metadata: {video}')
    actual_hash=sha256(video)
    saved_video=summary.get('closeup_video' if source=='closeup' else 'video',{})
    if saved_video.get('sha256') and saved_video['sha256']!=actual_hash:
        raise ValueError(f'Video hash disagrees with its run summary: {video}')
    trajectory_path=directory/'trajectory.json'
    horizon=None
    horizon_method=None
    if trajectory_path.is_file():
        samples=read(trajectory_path).get('samples',[])
        times=[s.get('trajectory_time_s') for s in samples if s.get('phase')=='motion'
               and finite(s.get('trajectory_time_s')) and s['trajectory_time_s']>=0]
        if times:
            horizon=max(times);horizon_method='Last recorded motion-phase trajectory timestamp'
    requested_horizon=summary.get('route',{}).get('reference_duration_s')
    if horizon is None and summary.get('trajectory_clock_complete') and finite(requested_horizon):
        horizon=requested_horizon;horizon_method='Completed reference duration from run summary'
    if horizon is None:
        raise ValueError(f'Cannot establish an observed or completed trajectory horizon: {directory}')
    origin=first-start
    return {'model':model,'file':str(video),'metadata_file':str(metadata_path),
            'run_summary_file':str(summary_path),'sha256':actual_hash,
            'metadata_sha256':sha256(metadata_path),'run_summary_sha256':sha256(summary_path),
            'trajectory_sha256':sha256(trajectory_path) if trajectory_path.is_file() else None,
            'camera_first_sim_time_s':first,'motion_start_s':start,
            'playback_zero_trajectory_time_s':origin,'video_duration_s':duration,
            'fps':fps,'observed_motion_horizon_s':horizon,'horizon_method':horizon_method,
            'trajectory_coverage_start_s':origin,'trajectory_coverage_end_s':min(origin+duration,horizon),
            'tracking_passed_reported':summary.get('tracking_passed'),
            'physical_failure':summary.get('physical_failure'),
            'source_recording_validated':saved_video.get('valid',saved_video.get('coverage_passed')),
            'ffprobe':info}


def aligned_interval(inputs, start=0., end=None, fps=None):
    if not finite(start) or (end is not None and (not finite(end) or end<=start)):
        raise ValueError('Requested time interval must be finite and nonempty')
    rate=min(item['fps'] for item in inputs) if fps is None else fps
    if not finite(rate) or rate<=0 or rate>120:
        raise ValueError('Output frame rate must lie within (0,120]')
    begin=max(start,*(item['trajectory_coverage_start_s'] for item in inputs))
    finish=min(item['trajectory_coverage_end_s'] for item in inputs)
    if end is not None:
        finish=min(finish,end)
    frames=math.floor((finish-begin)*rate+1e-9)
    if frames<1:
        raise ValueError('The source videos have no shared requested trajectory interval')
    return {'start_s':begin,'end_s':begin+frames/rate,'duration_s':frames/rate,
            'frames':frames,'fps':rate,'requested_start_s':start,'requested_end_s':end,
            'available_common_end_s':finish}


def render_pair(swivel_run, ball_run, output, source='closeup', start=0., end=None,
                fps=None, panel_width=960, ffmpeg=None, ffprobe=None):
    ffmpeg=ffmpeg or shutil.which('ffmpeg')
    ffprobe=ffprobe or shutil.which('ffprobe')
    if not ffmpeg or not ffprobe:
        raise RuntimeError('ffmpeg/ffprobe unavailable; source hamr_terrain/scripts/env.sh first')
    if source not in ('closeup','camera'):
        raise ValueError('Source must be closeup or camera')
    if panel_width<160 or panel_width%2:
        raise ValueError('Panel width must be an even integer of at least 160 pixels')
    output=Path(output).expanduser().resolve()
    if output.suffix.lower()!='.mp4':
        raise ValueError('Output must have an .mp4 extension')
    sidecar=output.with_suffix('.json')
    if output.exists() or sidecar.exists():
        raise FileExistsError(f'Refusing to overwrite existing output or provenance: {output}')
    inputs=[input_description(path,model,source,ffprobe)
            for path,model in ((swivel_run,'swivel'),(ball_run,'ball'))]
    if output in (Path(item['file']) for item in inputs):
        raise ValueError('Output cannot replace a source recording')
    interval=aligned_interval(inputs,start,end,fps)
    height=round(panel_width*9/16/2)*2
    header=max(48,round(panel_width*.075/2)*2)
    font_size=max(12,round(panel_width/36))
    small_size=max(9,round(panel_width/60))
    filters=[]
    for index,(item,label) in enumerate(zip(inputs,('Swivel caster','Split-ball caster'))):
        offset=interval['start_s']-item['playback_zero_trajectory_time_s']
        item['input_playback_trim_start_s']=offset
        item['input_playback_trim_end_s']=offset+interval['duration_s']
        # Preserve PTS relative to the requested common clock, rather than
        # rebasing each stream to its independently rounded first kept frame.
        filters.append(f'[{index}:v]trim=start={offset:.12f}:end={offset+interval["duration_s"]:.12f},'
            f'setpts=PTS-({offset:.12f})/TB,fps=fps={interval["fps"]:g}:start_time=0:round=near,'
            f'scale={panel_width}:{height}:force_original_aspect_ratio=decrease,'
            f'pad={panel_width}:{height}:(ow-iw)/2:(oh-ih)/2:color=black,setsar=1,'
            f'pad={panel_width}:{height+header}:0:{header}:color=0x152a38,'
            f"drawtext=text='{label}':fontcolor=white:fontsize={font_size}:x=12:y=8,"
            f"drawtext=text='Actual simulation - passive caster':fontcolor=0xd8e4eb:fontsize={small_size}:x=12:y={header-small_size-8}[v{index}]")
    routes=[read(Path(p)/'run_summary.json').get('route',{}) for p in (swivel_run,ball_run)]
    round_trip=routes[0].get('out_and_back') if routes[0]==routes[1] else None
    if round_trip:
        filters.append('[v0][v1]hstack=inputs=2:shortest=1[paired]')
        origin=interval['start_s']
        phases=[('OUTBOUND',0.,round_trip['outbound_end_s']),
                ('STOP - REVERSING PATH',round_trip['outbound_end_s'],round_trip['return_start_s']),
                ('RETURN - RETRACING PATH',round_trip['return_start_s'],round_trip['return_end_s'])]
        overlays=','.join(f"drawtext=text='{label}':fontcolor=white:fontsize={font_size}:x=(w-text_w)/2:y=h-38:box=1:boxcolor=black@0.7:boxborderw=7:enable='gte(t,{a-origin:.9f})*lt(t,{b-origin:.9f})'" for label,a,b in phases)
        filters.append('[paired]'+overlays+'[out]')
    else:
        filters.append('[v0][v1]hstack=inputs=2:shortest=1[out]')
    output.parent.mkdir(parents=True,exist_ok=True)
    temporary=output.with_name(output.stem+'.encoding.mp4')
    if temporary.exists():
        raise FileExistsError(f'Interrupted output exists; inspect it before retrying: {temporary}')
    command=[ffmpeg,'-nostdin','-hide_banner','-loglevel','warning','-n',
             '-threads','2','-i',inputs[0]['file'],'-threads','2','-i',inputs[1]['file'],
             '-filter_complex_threads','1','-filter_complex',';'.join(filters),'-map','[out]',
             '-frames:v',str(interval['frames']),'-an','-c:v','libx264','-threads','2',
             '-preset','veryfast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(temporary)]
    log=output.with_suffix('.encoding.log')
    with log.open('w') as handle:
        subprocess.run(command,stdout=handle,stderr=subprocess.STDOUT,check=True,timeout=600)
    info=probe(temporary,ffprobe)
    observed_duration=float(info['format']['duration'])
    observed_frames=int(info['streams'][0]['nb_frames'])
    if observed_frames!=interval['frames'] or abs(observed_duration-interval['duration_s'])>1/interval['fps']+.001:
        raise RuntimeError('Encoded duration/frame count differs from the planned common interval')
    decode_command=[ffmpeg,'-nostdin','-v','error','-threads','2','-i',str(temporary),'-f','null','-']
    decode=subprocess.run(decode_command,capture_output=True,text=True,timeout=600)
    if decode.returncode or decode.stderr.strip():
        raise RuntimeError(f'Output failed full decoding: {decode.stderr}')
    if any(sha256(item['file'])!=item['sha256'] for item in inputs):
        raise RuntimeError('A source recording changed during rendering')
    temporary.replace(output)
    report={'schema_version':1,'output':str(output),'output_sha256':sha256(output),
            'renderer_sha256':sha256(__file__),'source_type':source,'inputs':inputs,
            'common_trajectory_interval_s':interval,'output_ffprobe':info,
            'command':command,'full_decode_command':decode_command,
            'validation':{'full_decode_passed':True,'frame_count_matches':True,
                          'duration_matches':True,'source_hashes_unchanged':True},
            'timing_method':'Playback zero = camera first_sim_time_s - run motion_start_s. Trim and preserve PTS on the shared trajectory clock, then sample a common CFR output grid.',
            'timing_limit':'Source and output frames remain discrete; synchronization has frame-level quantization. CFR duplicate frames can preserve earlier camera images. No motion is synthesized.',
            'maximum_nominal_frame_quantization_s':max(1/item['fps'] for item in inputs)+1/interval['fps'],
            'scope':'Labeled actual saved Gazebo camera frames, scaled and padded without cropping. No trajectory, pose, physics or original video changes.'}
    sidecar.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--swivel-run',required=True,type=Path)
    parser.add_argument('--ball-run',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--source',choices=('closeup','camera'),default='closeup')
    parser.add_argument('--start-s',type=float,default=0.)
    parser.add_argument('--end-s',type=float)
    parser.add_argument('--fps',type=float)
    parser.add_argument('--panel-width',type=int,default=960)
    args=parser.parse_args(argv)
    result=render_pair(args.swivel_run,args.ball_run,args.output,args.source,args.start_s,
                       args.end_s,args.fps,args.panel_width)
    print(json.dumps({'output':result['output'],'interval':result['common_trajectory_interval_s'],
                      'validation':result['validation']},indent=2))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
