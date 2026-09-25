#!/usr/bin/env python3
"""Isolated experiment harness; does not change production configurations."""
import argparse, json, os, signal, subprocess, sys, time
from pathlib import Path

def stop_group(process):
    for sig, timeout in ((signal.SIGINT, 8),(signal.SIGTERM,3),(signal.SIGKILL,1)):
        try: os.killpg(process.pid,sig)
        except ProcessLookupError: break
        try: process.wait(timeout=timeout)
        except subprocess.TimeoutExpired: pass
        # Gazebo subprocesses can survive ROS launch exit, so signal the group
        # again even when the immediate process has exited.
        time.sleep(.5)
    try: process.wait(timeout=2)
    except subprocess.TimeoutExpired: pass

p=argparse.ArgumentParser();p.add_argument('cases',nargs='*',default=['baseline','no_slew','half_speed'])
a=p.parse_args(); root=Path(__file__).resolve().parent
results={}
for index,case in enumerate(a.cases):
    env=dict(os.environ);env['ROS_DOMAIN_ID']=str({'baseline':70,'no_slew':71,'half_speed':72,'quintic_stop':73,'no_slew_half_step':74}[case]);env['GZ_PARTITION']=f'hamr_corner_{os.getpid()}_{case}'
    env['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    launch=['ros2','launch','hamr_ball_caster','ball_caster.launch.py','model:=compa','gui:=false','controller:=false']
    if case.endswith('half_step'):launch+=['max_step_size:=0.0005']
    print(f'Starting {case} at {time.ctime()}',flush=True)
    with (root/f'{case}_launch.log').open('w') as log, (root/f'{case}_runner.log').open('w') as report:
        process=subprocess.Popen(launch,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        start=time.monotonic()
        try:
            checker=subprocess.run([sys.executable,str(root/('instrumented_quintic_runner.py' if case=='quintic_stop' else 'instrumented_waypoint_runner.py')),'--config',str(root/f'{case}.yaml'),'--output',str(root/f'{case}.json'),'--wall-timeout','1200'],env=env,stdout=report,stderr=subprocess.STDOUT,timeout=1230)
            results[case]={'exit_code':checker.returncode,'wall_duration_s':time.monotonic()-start,'domain':env['ROS_DOMAIN_ID'],'partition':env['GZ_PARTITION']}
        except subprocess.TimeoutExpired:
            results[case]={'exit_code':124,'wall_duration_s':time.monotonic()-start}
        finally:stop_group(process)
    (root/('ablation_results_'+'_'.join(a.cases)+'.json')).write_text(json.dumps(results,indent=2)+'\n')
    print(f'Finished {case}: {results[case]}',flush=True)
    if (root/f'{case}.json').exists():print(json.dumps(json.loads((root/f'{case}.json').read_text())['summary']),flush=True)
