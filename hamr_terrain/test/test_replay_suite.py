"""Ensure the saved campaign contains every requested, matched experiment."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_terrain_replay_suite import matrix
from build_terrain_replays import build_gallery


def test_matrix_contains_all_50_unique_matched_runs(tmp_path):
    cases=matrix(tmp_path)
    assert len(cases)==50
    assert len({c['directory'] for c in cases})==50
    assert {c['terrain'] for c in cases} == {
        'road_bumps', 'gentle_hill', 'cross_slope', 'rutted_track', 'rock_garden'}
    assert len({c['shape'] for c in cases})==5
    for a,b in zip(cases[::2],cases[1::2]):
        assert (a['caster'],b['caster'])==('swivel','ball')
        assert a['route']==b['route']
        assert a['terrain']==b['terrain']
    assert matrix(tmp_path)==cases


def test_resume_refuses_changed_saved_request(tmp_path):
    cases=matrix(tmp_path)
    cases[0]['route']['speed_m_s']=.99
    (tmp_path/'experiment_matrix.json').write_text(json.dumps(cases))
    with pytest.raises(ValueError,match='differs'):matrix(tmp_path)


def test_speed_control_is_matched_and_cannot_resume_into_different_speed(tmp_path):
    cases=matrix(tmp_path,shapes=('straight',),speed=.12)
    assert len(cases)==10
    assert all(c['route']['speed_m_s']==.12 for c in cases)
    assert all(a['route']==b['route'] for a,b in zip(cases[::2],cases[1::2]))
    with pytest.raises(ValueError,match='differs'):
        matrix(tmp_path,shapes=('straight',),speed=.25)


def test_map_route_matrix_uses_each_map_metadata_and_refuses_changed_route(tmp_path, monkeypatch):
    import run_terrain_replay_suite as suite
    package = tmp_path/'package'
    config = package/'config'
    config.mkdir(parents=True)
    manifest = config/'maps.json'
    manifest.write_text(json.dumps([{'id': 'large_a'}, {'id': 'large_b'}]))
    metadata = {'bounds_m': {'x': [-40, 40], 'y': [-40, 40]},
                'replay_route': {'points_m': [[-10, -15], [10, 15]]}}
    (config/'large_a.json').write_text(json.dumps(metadata))
    metadata['replay_route']['points_m'] = [[-20, 0], [20, 0]]
    (config/'large_b.json').write_text(json.dumps(metadata))
    monkeypatch.setattr(suite, 'package_directory', lambda: package)
    output = tmp_path/'runs'
    output.mkdir()
    cases = matrix(output, manifest, ('map_route', 'map_route_out_back'))
    assert len(cases) == 8
    for a, b in zip(cases[::2], cases[1::2]):
        assert a['route'] == b['route']
    assert cases[0]['route']['points_m'] == [[-10, -15], [10, 15]]
    assert cases[4]['route']['points_m'] == [[-20, 0], [20, 0]]
    metadata['replay_route']['points_m'] = [[-20, 0], [21, 0]]
    (config/'large_b.json').write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match='differs'):
        matrix(output, manifest, ('map_route', 'map_route_out_back'))


def test_pending_gallery_does_not_claim_videos_exist(tmp_path):
    matrix(tmp_path)
    report=build_gallery(tmp_path)
    assert report['expected']==50
    assert report['validated_replays']==0
    assert not report['complete']
    assert '<video ' not in (tmp_path/'index.html').read_text()


def test_sigterm_stops_active_worker_without_dispatching_next_case(tmp_path):
    """Exercise the real dispatcher with lightweight owned child processes."""
    scripts=Path(__file__).resolve().parents[1]/'scripts'
    fake_runner=tmp_path/'run_terrain_replay.py'
    fake_runner.write_text('''import json,os,signal,sys,time
from pathlib import Path
p=Path(sys.argv[sys.argv.index('--output-dir')+1]);p.mkdir(parents=True)
(p/'started.tmp').write_text(json.dumps({'pid':os.getpid()}));(p/'started.tmp').replace(p/'started.json')
def stop(*_):
 (p/'run_summary.json').write_text(json.dumps({'recording_complete':False,'infrastructure_failure':'test cancellation'}))
 raise SystemExit(1)
signal.signal(signal.SIGINT,stop)
signal.signal(signal.SIGTERM,stop)
while True:time.sleep(.05)
''')
    wrapper=tmp_path/'dispatcher.py'
    wrapper.write_text(f'''import sys
from pathlib import Path
sys.path.insert(0,{str(scripts)!r})
import run_terrain_replay_suite as suite
suite.__file__={str(tmp_path/'run_terrain_replay_suite.py')!r}
suite.matrix=lambda root:[{{'terrain':'test','shape':shape,'caster':'ball','directory':'runs/'+shape}} for shape in ('first','second')]
suite.build_gallery=lambda root:{{'complete':False,'expected':2,'validated_replays':0}}
raise SystemExit(suite.main())
''')
    output=tmp_path/'output'
    process=subprocess.Popen([sys.executable,str(wrapper),str(output),'--workers','1',
                              '--wall-timeout','43200'],
                             stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True,text=True)
    child_pid=None
    try:
        marker=output/'runs/first/started.json'
        deadline=time.monotonic()+15
        while not marker.exists():
            if process.poll() is not None:
                raise AssertionError(process.communicate())
            assert time.monotonic()<deadline
            time.sleep(.05)
        child_pid=json.loads(marker.read_text())['pid']
        command=json.loads((output/'execution/worker_0.json').read_text())['command']
        assert command[command.index('--wall-timeout')+1] == '43200.0'
        started=time.monotonic()
        process.send_signal(signal.SIGTERM)
        stdout,stderr=process.communicate(timeout=8)
        assert time.monotonic()-started<8
        assert process.returncode==1,(stdout,stderr)
        assert not (output/'runs/second').exists()
        assert not list((output/'claims').glob('*.json'))
        with pytest.raises(ProcessLookupError):os.kill(child_pid,0)
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGKILL);process.wait()
        if child_pid:
            try:os.kill(child_pid,signal.SIGKILL)
            except ProcessLookupError:pass
