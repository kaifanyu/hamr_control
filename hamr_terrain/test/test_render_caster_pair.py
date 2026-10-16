"""Exercise timestamp alignment with real short encoded videos, without Gazebo."""
import json
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from render_caster_pair import aligned_interval, input_description, render_pair, sha256


def test_common_horizon_is_intersection_and_whole_output_frames():
    inputs=[{'fps':15.,'trajectory_coverage_start_s':-3.,'trajectory_coverage_end_s':8.},
            {'fps':25.,'trajectory_coverage_start_s':.1,'trajectory_coverage_end_s':5.04}]
    interval=aligned_interval(inputs)
    assert interval['start_s']==.1
    assert interval['end_s']<=5.04
    assert interval['frames']==74
    assert interval['duration_s']==pytest.approx(74/15)
    with pytest.raises(ValueError,match='no shared'):aligned_interval(inputs,start=7)
    with pytest.raises(ValueError):aligned_interval(inputs,start=float('nan'))


@pytest.fixture
def saved_videos(tmp_path):
    ffmpeg,ffprobe=shutil.which('ffmpeg'),shutil.which('ffprobe')
    if not ffmpeg or not ffprobe:
        pytest.skip('Source hamr_terrain/scripts/env.sh to enable FFmpeg integration checks')
    for model,motion_start,red_duration in (('swivel',1.0,1.2),('ball',1.4,1.6)):
        directory=tmp_path/model;directory.mkdir()
        movie=directory/'closeup.mp4'
        subprocess.run([ffmpeg,'-nostdin','-v','error','-f','lavfi','-i',
            f'color=c=red:s=160x90:r=10:d={red_duration}',
            '-f','lavfi','-i',f'color=c=blue:s=160x90:r=10:d={2.4-red_duration}',
            '-filter_complex_threads','1','-filter_complex','[0:v][1:v]concat=n=2:v=1:a=0[out]',
            '-map','[out]','-c:v','libx264','-threads','2','-pix_fmt','yuv420p',str(movie)],
            check=True,capture_output=True,timeout=30)
        summary={'caster':model,'motion_start_s':motion_start,'trajectory_clock_complete':True,
                 'route':{'reference_duration_s':1.},'tracking_passed':True,
                 'closeup_video':{'valid':True,'sha256':sha256(movie)}}
        (directory/'run_summary.json').write_text(json.dumps(summary))
        metadata={'first_sim_time_s':.2,'expected_video_duration_s':2.4,'fps':10}
        (directory/'closeup.json').write_text(json.dumps(metadata))
    return tmp_path,ffmpeg,ffprobe


def test_real_video_alignment_preserves_sources_and_matches_visible_event(saved_videos):
    root,ffmpeg,ffprobe=saved_videos
    originals={m:sha256(root/m/'closeup.mp4') for m in ('swivel','ball')}
    output=root/'pair.mp4'
    report=render_pair(root/'swivel',root/'ball',output,panel_width=160)
    assert report['validation']['full_decode_passed']
    assert report['common_trajectory_interval_s']['frames']==10
    assert [x['input_playback_trim_start_s'] for x in report['inputs']]==pytest.approx([.8,1.2])
    info=report['output_ffprobe']['streams'][0];width,height=info['width'],info['height']
    selected=subprocess.run([ffmpeg,'-nostdin','-v','error','-threads','2','-i',str(output),
        '-vf',r'select=eq(n\,1)+eq(n\,7)','-vsync','0','-frames:v','2','-pix_fmt','rgb24',
        '-f','rawvideo','pipe:1'],capture_output=True,check=True,timeout=30)
    frames=np.frombuffer(selected.stdout,dtype=np.uint8).reshape(2,height,width,3)
    # Both source events occur at trajectory t=.4 s despite their .4 s offset.
    for x in (80,240):
        early,late=frames[0,93,x],frames[1,93,x]
        assert early[0]>180 and early[2]<60
        assert late[2]>180 and late[0]<60
    assert originals=={m:sha256(root/m/'closeup.mp4') for m in originals}
    assert json.loads(output.with_suffix('.json').read_text())['output_sha256']==sha256(output)
    with pytest.raises(FileExistsError):render_pair(root/'swivel',root/'ball',output,panel_width=160)


def test_failed_partial_run_uses_observed_motion_horizon(saved_videos):
    root,_,ffprobe=saved_videos
    directory=root/'ball'
    summary=json.loads((directory/'run_summary.json').read_text())
    summary.update(trajectory_clock_complete=False,tracking_passed=False,physical_failure='tip_over')
    (directory/'run_summary.json').write_text(json.dumps(summary))
    (directory/'trajectory.json').write_text(json.dumps({'samples':[
        {'phase':'motion','trajectory_time_s':0.},
        {'phase':'motion','trajectory_time_s':.6},
        {'phase':'failure_hold','trajectory_time_s':1.}]}))
    result=input_description(directory,'ball','closeup',ffprobe)
    assert result['observed_motion_horizon_s']==.6
    assert result['trajectory_coverage_end_s']==.6
    assert result['physical_failure']=='tip_over'
