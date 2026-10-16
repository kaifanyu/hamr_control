"""Offline paired presentation preserves failures, missing data and provenance."""
import json
from html.parser import HTMLParser
from pathlib import Path
import sys
from urllib.parse import unquote, urlsplit

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from build_caster_stress_comparison import build_comparison, collect, digest, patch_bounds, video_sources


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def campaign(root, pending=False, failed=False):
    terrain = 'caster_stress_staggered_l2'
    shape = 'stress_reversal'
    route = {'shape':shape, 'points_m':[[0,-1],[0,1],[0,-1]], 'speed_m_s':.25}
    matrix, runs = [], []
    for model in ('swivel','ball'):
        directory = root/'runs'/terrain/shape/model
        matrix.append({'terrain':terrain,'shape':shape,'caster':model,
                       'route':route,'directory':directory.relative_to(root).as_posix()})
        summary = {'tracking_passed':not (failed and model=='swivel'),
                   'physical_failure':'tip_over' if failed and model=='swivel' else None,
                   'trajectory_clock_complete':True, 'recording_complete':True,
                   'motion_start_s':3.2 if model=='swivel' else 3.7,
                   'closeup_video':{'valid':True}}
        write(directory/'run_summary.json',summary)
        write(directory/'trajectory.json',{'samples':[
            {'phase':'motion','position_m':[.01 if model=='ball' else 0,y,0],
             'odometry_stamp_s':3.2+i*.1,'trajectory_time_s':i*.1}
            for i,y in enumerate((-1,0,1,0,-1))]})
        write(directory/'replay_validation.json',{'passed':True})
        for movie,meta in (('camera.mp4','camera.json'),('replay.mp4','camera.json'),('closeup.mp4','closeup.json')):
            (directory/movie).write_bytes(b'fixture placeholder; validator is external')
            write(directory/meta,{'first_sim_time_s':.1,'expected_video_duration_s':10.,
                                  'fps':15,'encoded_source_timestamps_s':[.1,.2],
                                  'frames_encoded':150,'frames_duplicated':148})
        runs.append({'run_directory':str(directory),'terrain':terrain,'shape':shape,'caster':model,
                     'input_hashes':{'trajectory.json':digest(directory/'trajectory.json')}})
    pair = {'terrain':terrain,'shape':shape,'trial':'0','paired':True,'fully_verified_pair':True,
            'swivel_run':runs[0]['run_directory'],'ball_run':runs[1]['run_directory'],
            'common_observed_horizon_s':8.,'common_horizon_ball_minus_swivel':{'xy_rmse_m':-.001},
            'common_horizon_metrics':{'swivel':{'xy_rmse_m':.003},'ball':{'xy_rmse_m':.002}}}
    if pending:
        matrix.extend([{**item,'shape':'stress_lateral','directory':item['directory'].replace(shape,'stress_lateral')} for item in matrix])
    write(root/'experiment_matrix.json',matrix)
    write(root/'caster_analysis/analysis.json',{'runs':runs,'pairs':[pair]})
    return matrix, runs, pair


class Links(HTMLParser):
    def __init__(self):
        super().__init__();self.urls=[]
    def handle_starttag(self, tag, attrs):
        self.urls.extend(value for key,value in attrs if key in ('src','href') and value)


def test_full_page_retains_failures_pending_cases_and_portable_figures(tmp_path, monkeypatch):
    import generate_terrains
    from PIL import Image

    # Saved study figures are optional local outputs, not repository fixtures.
    source = tmp_path / 'source'
    figures = source / 'figures'
    figures.mkdir(parents=True)
    stem = figures / 'caster_stress_staggered_l2_height_slope'
    Image.new('RGB', (2, 2), 'white').save(stem.with_suffix('.png'))
    stem.with_suffix('.svg').write_text('<svg xmlns="http://www.w3.org/2000/svg"/>')
    monkeypatch.setattr(generate_terrains, 'package_directory', lambda: source)
    campaign(tmp_path,pending=True,failed=True)
    report=build_comparison(tmp_path)
    assert (report['expected_runs'],report['expected_pairs'])==(4,2)
    assert report['failure_runs']==1
    assert report['pending_runs']==2
    assert report['verified_pairs']==1
    assert not report['complete']
    page=(tmp_path/'comparison.html').read_text()
    assert 'tip_over' in page and 'physical failure' in page
    assert page.count('<section class="case"')==2
    assert 'simulation-time origin' in page
    links=Links();links.feed(page)
    for url in links.urls:
        assert not urlsplit(url).scheme
        if not url.startswith('#'):
            assert (tmp_path/unquote(url.split('#')[0])).is_file(),url
    for stem in ('paired_heatmap','paired_xy_overview','pair_001','pair_002'):
        assert (tmp_path/'comparison_assets'/f'{stem}.png').stat().st_size>1000
        assert '<svg' in (tmp_path/'comparison_assets'/f'{stem}.svg').read_text()
    figures=report['terrain_figures']
    assert len(figures)==1
    assert all(v['sha256']==v['source_sha256'] for v in figures[0]['files'].values())


def test_optional_terrain_figures_can_be_absent(tmp_path, monkeypatch):
    import generate_terrains
    from build_caster_stress_comparison import copy_terrain_figures

    monkeypatch.setattr(generate_terrains, 'package_directory', lambda: tmp_path)
    assert copy_terrain_figures(tmp_path, [{'terrain': 'caster_stress_staggered_l2'}]) == []


def test_video_alignment_uses_cfr_origin_not_fresh_frame_array_indices(tmp_path):
    matrix,_,_=campaign(tmp_path)
    directory=tmp_path/matrix[0]['directory']
    summary=json.loads((directory/'run_summary.json').read_text())
    videos=video_sources(tmp_path,directory,summary,[])
    assert len(videos)==3
    assert all(v['synchronizable'] for v in videos)
    assert all(v['trajectory_origin_s']==pytest.approx(-3.1) for v in videos)
    assert all(v['duration_s']==10 for v in videos)
    # Only two fresh source stamps were logged, yet the CFR video has 150 frames.
    assert videos[0]['duration_s']!=2/15
    (directory/'camera.json').unlink()
    assert not video_sources(tmp_path,directory,summary,[])[0]['synchronizable']


def test_stale_analysis_withholds_deltas_without_hiding_current_trace(tmp_path):
    matrix,_,_=campaign(tmp_path)
    path=tmp_path/matrix[0]['directory']/'trajectory.json'
    value=json.loads(path.read_text());value['samples'][0]['position_m'][0]=.5;write(path,value)
    report=build_comparison(tmp_path,figures=False)
    assert report['verified_pairs']==0
    data=json.loads((tmp_path/'comparison_data.json').read_text())['pairs'][0]
    assert data['common_horizon_ball_minus_swivel_xy_rmse_m'] is None
    assert 'Analysis is stale' in (tmp_path/'comparison.html').read_text()


def test_condition_mismatch_is_visible_and_not_reported_as_matched(tmp_path):
    _,runs,pair=campaign(tmp_path)
    pair.update(paired=False,fully_verified_pair=False,condition_mismatches=['spawn'])
    write(tmp_path/'caster_analysis/analysis.json',{'runs':runs,'pairs':[pair]})
    build_comparison(tmp_path,figures=False)
    row=json.loads((tmp_path/'comparison_data.json').read_text())['pairs'][0]
    assert row['paired'] is False
    assert row['common_horizon_ball_minus_swivel_xy_rmse_m'] is None
    assert 'spawn' in (tmp_path/'comparison.html').read_text()


def test_relocated_analysis_and_complete_failure_preserve_correct_counts(tmp_path):
    _,runs,pair=campaign(tmp_path,failed=True)
    for run in runs:
        run['run_directory']=run['run_directory'].replace(str(tmp_path),'/old/machine/campaign')
    for model in ('ball','swivel'):
        pair[model+'_run']=pair[model+'_run'].replace(str(tmp_path),'/old/machine/campaign')
    write(tmp_path/'caster_analysis/analysis.json',{'runs':runs,'pairs':[pair]})
    report=build_comparison(tmp_path,figures=False)
    assert report['complete'] is True  # Complete evidence is distinct from success.
    assert report['failure_runs']==1
    assert report['runs_with_validated_overviews']==2


def test_missing_analysis_is_pending_not_zero_difference(tmp_path):
    campaign(tmp_path)
    (tmp_path/'caster_analysis/analysis.json').unlink()
    report=build_comparison(tmp_path,figures=False)
    row=json.loads((tmp_path/'comparison_data.json').read_text())['pairs'][0]
    assert report['analysis_available'] is False
    assert row['common_horizon_ball_minus_swivel_xy_rmse_m'] is None
    assert not report['complete']


def test_patch_edges_come_from_saved_terrain_dimensions(tmp_path):
    matrix,_,_=campaign(tmp_path)
    for case in matrix:
        write(tmp_path/case['directory']/'terrain.json',{'bounds_m':{'x':[-4,4],'y':[-4,4]}})
    groups,_,_,_=collect(tmp_path,tmp_path/'caster_analysis/analysis.json')
    assert patch_bounds(groups[0])=={'x':[-4,4],'y':[-4,4]}
    assert patch_bounds({'models':{'ball':{}}})=={'x':[-3.,3.],'y':[-4.,4.]}


@pytest.mark.parametrize('stem,label,view',[
    ('high_pad_reversal_closeups_side_by_side','High staggered pads','close-up'),
    ('washboard_l1_reversal_overviews_side_by_side','Washboard level 1','overview'),
])
def test_optional_paired_clip_is_linked_hashed_and_never_overwritten(tmp_path,stem,label,view):
    campaign(tmp_path)
    clip=tmp_path/'comparison_assets'/f'{stem}.mp4'
    clip.parent.mkdir()
    clip.write_bytes(b'paired clip fixture; preserve original bytes')
    write(clip.with_suffix('.json'),{'validation':{'full_decode_passed':True}})
    poster=clip.with_name(stem+'_frame11p2.png')
    poster.write_bytes(b'poster fixture; preserve original bytes')
    original=digest(clip)
    report=build_comparison(tmp_path,figures=False)
    assert digest(clip)==original
    assert len(report['paired_video_clips'])==1
    assert report['paired_video_clips'][0]['sha256']==original
    assert report['paired_video_clips'][0]['provenance_sha256']==digest(clip.with_suffix('.json'))
    assert report['paired_video_clips'][0]['poster_sha256']==digest(poster)
    assert report['output_sha256'][clip.relative_to(tmp_path).as_posix()]==original
    page=(tmp_path/'comparison.html').read_text()
    assert label in page
    assert clip.name in page
    assert f'Download synchronized {view} MP4' in page
    assert f'poster="comparison_assets/{poster.name}"' in page


@pytest.mark.parametrize('mutation',['duplicate','escape'])
def test_invalid_matrix_cannot_silently_merge_or_escape_campaign(tmp_path,mutation):
    matrix,_,_=campaign(tmp_path)
    if mutation=='duplicate':matrix.append(matrix[0])
    else:matrix[0]['directory']='../elsewhere'
    write(tmp_path/'experiment_matrix.json',matrix)
    with pytest.raises(ValueError):collect(tmp_path,tmp_path/'caster_analysis/analysis.json')
