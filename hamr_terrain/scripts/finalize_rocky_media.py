#!/usr/bin/env python3
"""Resume paired rocky overview/close-up exports as validated recordings arrive."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import time

from render_caster_pair import render_pair,sha256


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('campaign',type=Path)
    p.add_argument('--wait',action='store_true')
    p.add_argument('--wall-timeout',type=float,default=14400.)
    args=p.parse_args();root=args.campaign.expanduser().resolve()
    matrix=json.loads((root/'replays/experiment_matrix.json').read_text())
    groups={}
    for case in matrix:groups.setdefault((case['terrain'],case['shape']),{})[case['caster']]=root/'replays'/case['directory']
    destination=root/'paired_videos';destination.mkdir(exist_ok=True)
    started=time.monotonic()
    while True:
        done=0;pending=0
        for (terrain,shape),models in groups.items():
            ready=all((models[m]/'replay_validation.json').is_file() for m in ('swivel','ball'))
            for source in ('camera','closeup'):
                output=destination/f'{terrain}__{shape}__{source}.mp4'
                sidecar=output.with_suffix('.json')
                if output.is_file() and sidecar.is_file():
                    report=json.loads(sidecar.read_text())
                    if report['output_sha256']!=sha256(output):raise ValueError(f'Existing video changed: {output}')
                    done+=1;continue
                if not ready:pending+=1;continue
                temporary=output.with_name(output.stem+'.encoding.mp4')
                if temporary.exists():
                    failed=destination/'interrupted';failed.mkdir(exist_ok=True)
                    temporary.rename(failed/(temporary.stem+'_'+datetime.now(timezone.utc).strftime('%H%M%S')+'.mp4'))
                print(json.dumps({'rendering':output.name}),flush=True)
                render_pair(models['swivel'],models['ball'],output,source=source,panel_width=960)
                done+=1
        status={'completed_paired_clips':done,'expected_paired_clips':2*len(groups),'pending':pending}
        (destination/'progress.json').write_text(json.dumps(status,indent=2)+'\n')
        if not pending or not args.wait:
            print(json.dumps(status),flush=True);return 0 if not pending else 2
        if time.monotonic()-started>args.wall_timeout:raise TimeoutError('Waiting for recordings exceeded wall timeout')
        time.sleep(10)


if __name__=='__main__':raise SystemExit(main())
