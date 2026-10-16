#!/usr/bin/env python3
"""Audit every requested MP4, physical world snapshot, and measured run."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from run_terrain_replay import write_json
from terrain_replay_paths import sha256


def audit(root):
    root=Path(root).resolve()
    matrix=json.loads((root/'experiment_matrix.json').read_text())
    errors=[];pending=[];records=[]
    for case in matrix:
        directory=root/case['directory']
        validation=directory/'replay_validation.json'
        if not validation.exists():
            pending.append(case['directory']);continue
        try:
            evidence=json.loads(validation.read_text())
            summary=json.loads((directory/'run_summary.json').read_text())
            configuration=json.loads((directory/'configuration.json').read_text())
            camera=json.loads((directory/'camera.json').read_text())
            trajectory=json.loads((directory/'trajectory.json').read_text())
            checks={
                'replay_hash':sha256(directory/'replay.mp4')==evidence['sha256'],
                'camera_hash':sha256(directory/'camera.mp4')==evidence['source_camera_sha256']==summary['video']['sha256'],
                'full_decode_passed':evidence['full_decode_passed'],
                'video_coverage':summary['recording_complete'] and summary['video']['coverage_passed'],
                'no_infrastructure_failure':summary['infrastructure_failure'] is None,
                'requested_identity':all(summary[k]==case[k] for k in ('terrain','caster','shape')),
                'requested_route':summary['route']==case['route']==configuration['route'],
                'trajectory_configuration':trajectory['configuration']==configuration,
                'source_world_hash':sha256(configuration['source_world'])==configuration['source_world_sha256'],
                'snapshot_world_hash':sha256(directory/'world.sdf')==configuration['snapshot_sha256'],
                'timestamps_monotonic':all(b>a for a,b in zip(camera['encoded_source_timestamps_s'],camera['encoded_source_timestamps_s'][1:])),
                'no_camera_queue_drop':camera['frames_queue_dropped']==0,
                'samples_present':len(trajectory['samples'])>5,
            }
            if 'robot_snapshot_sha256' in configuration:
                checks['robot_snapshot_hash']=(sha256(directory/'robot.urdf')==
                                               configuration['robot_snapshot_sha256'])
                checks['robot_snapshot_launched']=(f'robot_description_file:={directory/"robot.urdf"}'
                    in configuration['commands']['simulation'])
            if summary.get('closeup_video'):
                closeup=json.loads((directory/'closeup.json').read_text())
                checks['closeup_hash']=(sha256(directory/'closeup.mp4')==
                                         summary['closeup_video']['sha256'])
                checks['closeup_recording_valid']=(summary['closeup_video']['valid']
                    and summary['closeup_video']['coverage_passed']
                    and closeup['success'] and closeup['frames_queue_dropped']==0)
            source=ET.parse(configuration['source_world']).getroot().find('world')
            snapshot=ET.parse(directory/'world.sdf').getroot().find('world')
            checks['one_ms_step']=float(snapshot.find('physics/max_step_size').text)==.001
            checks['original_gravity']=source.find('gravity').text==snapshot.find('gravity').text
            a=source.findall('.//collision[geometry]');b=snapshot.findall('.//collision[geometry]')
            checks['collision_count_unchanged']=len(a)==len(b)
            collision_equal=True
            for original,saved in zip(a,b):
                for uri in original.findall('.//uri'):
                    if uri.text.startswith('model://hamr_terrain/'):
                        uri.text=str((Path(configuration['source_world']).parent.parent/uri.text.removeprefix('model://hamr_terrain/')).resolve())
                if ET.tostring(original)!=ET.tostring(saved):collision_equal=False
            checks['collision_geometry_and_materials_unchanged']=collision_equal
            checks['route_visual_has_no_collisions']=not snapshot.findall("model[@name='planned_route_visual']//collision")
            for key,passed in checks.items():
                if not passed:errors.append(f"{case['directory']}: {key}")
            records.append({'case':case['directory'],'evidence_passed':all(checks.values()),
                            'tracking_passed':summary['tracking_passed'],'physical_failure':summary['physical_failure'],
                            'checks':checks,'video_bytes':(directory/'replay.mp4').stat().st_size,
                            'camera_span_s':camera['simulation_span_s']})
        except Exception as error:
            errors.append(f"{case['directory']}: {type(error).__name__}: {error}")
    result={'complete':len(records)==len(matrix) and not pending,'evidence_passed':not errors,
            'expected_runs':len(matrix),'audited_runs':len(records),'pending':pending,'errors':errors,
            'tracking_passed':sum(r['tracking_passed'] for r in records),
            'tracking_failed':sum(not r['tracking_passed'] for r in records),
            'total_replay_bytes':sum(r['video_bytes'] for r in records),
            'total_camera_duration_s':sum(r['camera_span_s'] for r in records),'runs':records}
    write_json(root/'validation_audit.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    parser.add_argument('--require-complete',action='store_true')
    args=parser.parse_args();result=audit(args.root)
    print(json.dumps({k:v for k,v in result.items() if k not in ('runs','pending')},indent=2))
    raise SystemExit(0 if result['evidence_passed'] and (result['complete'] or not args.require_complete) else 1)
