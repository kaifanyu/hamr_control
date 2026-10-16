#!/usr/bin/env python3
"""Resume-safe, bounded parallel execution of all ten terrain replay matrices."""
import argparse
from concurrent.futures import ThreadPoolExecutor,wait,FIRST_COMPLETED
from datetime import datetime,timezone
import json
import math
import os
import queue
from pathlib import Path
import signal
import subprocess
import sys
import threading

from generate_terrains import package_directory
from terrain_replay_paths import reference_for_terrain,SHAPES,SUPPORTED_SHAPES
from run_terrain_replay import write_json
from build_terrain_replays import build_gallery,finalize_case


def matrix(root, terrain_manifest=None, shapes=SHAPES, speed=.25):
    package=package_directory()
    terrains=[]
    manifests=[Path(terrain_manifest)] if terrain_manifest is not None else [package/'config'/name for name in ('terrains.json','adtg_terrains.json')]
    for path in manifests:
        terrains.extend(t['id'] for t in json.loads(path.read_text()))
    if not terrains or len(set(terrains))!=len(terrains):
        raise ValueError('Select nonempty, unique terrain identifiers')
    if not shapes or len(set(shapes))!=len(shapes) or any(s not in SUPPORTED_SHAPES for s in shapes):
        raise ValueError('Select nonempty, unique supported shapes')
    metadata = ({terrain:json.loads((package/'config'/f'{terrain}.json').read_text())
                 for terrain in terrains}
                if any(s.removesuffix('_out_back') == 'map_route' for s in shapes) else {})
    cases=[{'terrain':terrain,'shape':shape,'caster':caster,
            'route':reference_for_terrain(shape,speed,metadata.get(terrain,{})).as_dict(),
            'directory':f'runs/{terrain}/{shape}/{caster}'}
           for terrain in terrains for shape in shapes for caster in ('swivel','ball')]
    destination=root/'experiment_matrix.json'
    if destination.exists() and json.loads(destination.read_text())!=cases:
        raise ValueError('Existing matrix differs; use another output directory')
    write_json(destination,cases)
    return cases


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    parser.add_argument('--workers',type=int,default=3)
    parser.add_argument('--worker-offset',type=int,default=0,help='Disjoint worker/domain IDs for another resume-safe process sharing this matrix')
    parser.add_argument('--domain-base',type=int,default=125)
    parser.add_argument('--render-backend',choices=('egl','display','d3d12'),default='egl')
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--terrain-manifest',type=Path,help='Optional terrain registry instead of the original ten maps')
    parser.add_argument('--shapes',nargs='+',choices=SUPPORTED_SHAPES,help='Selected paths; default remains the original five')
    parser.add_argument('--speed',type=float,default=.25,help='Reference speed in m/s; default 0.25')
    parser.add_argument('--wall-timeout',type=float,default=7200,
                        help='Maximum wall seconds for each simulator/recorder run')
    parser.add_argument('--closeup-camera',action='store_true',help='Record the right front caster in every case')
    parser.add_argument('--contact-logging',action='store_true',help='Enable contact observations in every case')
    args=parser.parse_args()
    if not 1<=args.workers<=6:parser.error('Use one to six workers')
    if not math.isfinite(args.wall_timeout) or args.wall_timeout<=0:parser.error('wall-timeout must be finite and positive')
    if args.worker_offset<0 or not 0<=args.domain_base<=232-args.workers-args.worker_offset+1:parser.error('ROS domain range must remain within 0..232')
    root=args.output.expanduser().resolve();root.mkdir(parents=True,exist_ok=True)
    cases=matrix(root,args.terrain_manifest,args.shapes or SHAPES,args.speed) if args.terrain_manifest is not None or args.shapes is not None or args.speed!=.25 else matrix(root)
    (root/'claims').mkdir(exist_ok=True);(root/'attempts').mkdir(exist_ok=True)
    (root/'execution').mkdir(exist_ok=True)
    build_gallery(root)
    if args.prepare_only:return 0
    stopping=threading.Event();active={};lock=threading.Lock()
    def stop(*_):
        stopping.set()
        with lock:
            for process in active.values():
                if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    pending=queue.Queue()
    for case in cases:pending.put(case)
    def run_worker(worker):
        results=[]
        while not stopping.is_set():
            try:case=pending.get_nowait()
            except queue.Empty:break
            directory=root/case['directory']
            identity='_'.join(case[k] for k in ('terrain','shape','caster'))
            claim=root/'claims'/f'{identity}.json'
            if (directory/'replay_validation.json').exists():
                results.append({'case':identity,'retained':True});continue
            try:
                with claim.open('x') as f:json.dump({'pid':os.getpid(),'worker':worker},f)
            except FileExistsError:
                results.append({'case':identity,'claimed_by_another_worker':True})
                continue
            try:
                summary=directory/'run_summary.json'
                data=json.loads(summary.read_text()) if summary.exists() else {}
                if not data.get('recording_complete') or data.get('infrastructure_failure'):
                    for attempt in range(2):
                        if directory.exists():
                            stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%f')
                            directory.rename(root/'attempts'/f'{identity}_{stamp}')
                        directory.parent.mkdir(parents=True,exist_ok=True)
                        command=[sys.executable,str(Path(__file__).with_name('run_terrain_replay.py')),
                                 '--terrain',case['terrain'],'--shape',case['shape'],'--caster',case['caster'],
                                 '--output-dir',str(directory),'--domain-id',str(args.domain_base+worker),
                                 '--render-backend',args.render_backend,
                                 '--wall-timeout',str(args.wall_timeout),
                                 '--speed',str(case.get('route',{}).get('speed_m_s',args.speed))]
                        if args.closeup_camera:command.append('--closeup-camera')
                        if args.contact_logging:command.append('--contact-logging')
                        log=root/'execution'/f'{identity}_attempt{attempt+1}.log'
                        write_json(root/'execution'/f'worker_{worker}.json',{'case':case,'command':command,'phase':'running'})
                        with log.open('w') as stream:
                            process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
                            with lock:active[worker]=process
                            if stopping.is_set() and process.poll() is None:
                                os.killpg(process.pid,signal.SIGINT)
                            code=process.wait()
                            with lock:active.pop(worker,None)
                        data=json.loads(summary.read_text()) if summary.exists() else {}
                        if code==0 and data.get('recording_complete') and not data.get('infrastructure_failure'):break
                        if stopping.is_set():break
                    if not data.get('recording_complete') or data.get('infrastructure_failure'):
                        results.append({'case':identity,'infrastructure_failure':data.get('infrastructure_failure','recording validation failed')})
                        print(json.dumps(results[-1]),flush=True);continue
                # Matplotlib's global state is not thread safe. Finalization runs
                # as its own process, independent of the simulation controller.
                command=[sys.executable,str(Path(__file__).with_name('build_terrain_replays.py')),str(root),'--case',str(directory)]
                with (root/'execution'/f'{identity}_finalize.log').open('w') as stream:
                    result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT)
                results.append({'case':identity,'finalize_exit':result.returncode,'tracking_passed':data.get('tracking_passed')})
                print(json.dumps(results[-1]),flush=True)
            finally:
                claim.unlink(missing_ok=True)
        write_json(root/'execution'/f'worker_{worker}_results.json',results)
        return results
    all_results=[]
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures=[pool.submit(run_worker,i+args.worker_offset) for i in range(args.workers)]
        outstanding=set(futures)
        while outstanding:
            finished,outstanding=wait(outstanding,timeout=.5,return_when=FIRST_COMPLETED)
            for future in finished:all_results.extend(future.result())
    report=build_gallery(root)
    results_name='batch_results.json' if args.worker_offset==0 else f'batch_results_offset{args.worker_offset}.json'
    write_json(root/'execution'/results_name,all_results)
    print(json.dumps({k:v for k,v in report.items() if k!='runs'},indent=2),flush=True)
    return 0 if report['complete'] else 1


if __name__=='__main__':raise SystemExit(main())
