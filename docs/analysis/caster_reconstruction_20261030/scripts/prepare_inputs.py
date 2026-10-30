"""Prepare all shared valid caster intervals without changing source recordings.

Run under Ubuntu-24.04 Python. Existing pose caches retain Vicon bag receipt
timestamps. No prediction, clock fit, yaw fit, radius fit or encoder correction
is performed here. Segment-local phase origins are explicit.
"""
from pathlib import Path
import argparse
from _paths import DATA_ROOT, OUTPUT_ROOT, reconstruction_source, prepared_inputs, relocated
import csv, hashlib, json
import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp
from scipy.signal import savgol_filter

ROOT=DATA_ROOT
# Same fixed mapping and helper definitions used by sphere_simulation/simulate.py.
# Keep this preparation script self-contained for the saved source bundle.
MAPPING_PATH=ROOT/'config/caster_encoder_replay_pair.yaml'
MAPPING=yaml.safe_load(MAPPING_PATH.read_text())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def wrap(x):return np.arctan2(np.sin(x),np.cos(x))
SOURCE=ROOT/'recordings'
OUT=OUTPUT_ROOT/'prepared'
INPUTS=OUT/'inputs'; INPUTS.mkdir(exist_ok=True,parents=True)
NATIVE=OUT/'native_encoder'; NATIVE.mkdir(exist_ok=True,parents=True)
DT=.01; MIN_DURATION=1.; GAP=.1
WHEELS=['front_left','front_right']
POSES=['_HAMR_base_pose','_Left_Caster_pose','_Right_Caster_pose']

def read_channels(path):
    channels={(w,n):[] for w in WHEELS for n in [1,2,3]}
    counts={k:{'rows':0,'invalid_samples':0,'unmapped_samples':0} for k in channels}
    with path.open() as f:
        for row in csv.DictReader(f):
            key=(row['wheel_id'],int(row['node_id']))
            if key not in channels: continue
            counts[key]['rows']+=1
            if row['sample_valid']!='1':
                counts[key]['invalid_samples']+=1;continue
            a=float(row['angle_rad']) if key[1]!=3 else int(row['position14'])*2*np.pi/16384
            if not np.isfinite(a):continue
            epoch=int(row['mapped_epoch_ns']) if row['mapped_epoch_ns'] else None
            if epoch is None:counts[key]['unmapped_samples']+=1
            channels[key].append({'epoch':epoch,'angle':a,'boot':row['boot_id'],'seq':int(row['seq']),
                't_us':int(row['t_us']),'device':row['device_id'],
                'ppr_verified':row['ppr_verified']=='1','phase_reference_valid':row['phase_reference_valid']=='1',
                'unwrap_continuous':row['unwrap_continuous']=='1','clock_sync_valid':row['clock_sync_valid']=='1'})
    meta={}
    for key,rows in channels.items():
        bootgroups={}
        for r in rows:bootgroups.setdefault(r['boot'],[]).append(r)
        meta['%s/%d'%key]={**counts[key], 'valid_samples':len(rows),
            'devices':sorted(set(r['device'] for r in rows)),
            'ppr_verified_all':all(r['ppr_verified'] for r in rows),
            'phase_reference_valid_all':all(r['phase_reference_valid'] for r in rows),
            'clock_sync_valid_all':all(r['clock_sync_valid'] for r in rows),
            'boots':[{ 'boot_id':b,'samples':len(v), 'board_start_us':v[0]['t_us'],'board_end_us':v[-1]['t_us'],
                'board_duration_s':(v[-1]['t_us']-v[0]['t_us'])*1e-6,
                'max_board_gap_s':max(np.diff([r['t_us'] for r in v]),default=0)*1e-6,
                'mapped_samples':sum(r['epoch'] is not None for r in v)} for b,v in bootgroups.items()]}
    return channels,meta

def save_native(run,channels):
    """All valid raw histories, including unmapped boards; never merge clocks."""
    arrays={}
    for (w,n),rows in channels.items():
        for bi,b in enumerate(dict.fromkeys(r['boot'] for r in rows)):
            v=[r for r in rows if r['boot']==b]; p=f'{w}__node{n}__boot{bi}__'
            arrays[p+'board_elapsed_s']=np.array([(r['t_us']-v[0]['t_us'])*1e-6 for r in v])
            arrays[p+'board_t_us']=np.array([r['t_us'] for r in v],dtype=np.int64)
            raw=np.unwrap([r['angle'] for r in v]);arrays[p+'angle_change_rad']=raw-raw[0]
            arrays[p+'mapped_epoch_ns']=np.array([r['epoch'] if r['epoch'] is not None else -1 for r in v],dtype=np.int64)
            arrays[p+'seq']=np.array([r['seq'] for r in v],dtype=np.int64)
            arrays[p+'boot_id']=np.array(b)
    dest=NATIVE/(run+'__native_encoder.npz');np.savez_compressed(dest,**arrays)
    return str(dest)

def exclusions_and_segments(lo,hi,poses,enc):
    bad=[]
    def add(a,b,reason,source):
        a=max(lo,float(a));b=min(hi,float(b))
        if b>a:bad.append({'start_s':a,'end_s':b,'reason':reason,'source':source})
    for name,a in poses.items():
        for i in np.flatnonzero(np.diff(a[:,0])>GAP):add(a[i,0],a[i+1,0],'pose gap >0.1 s',name)
        for i in np.flatnonzero(np.linalg.norm(np.diff(a[:,1:4],axis=0),axis=1)>.08):
            add(a[i,0]-.1,a[i+1,0]+.1,'pose position jump >80 mm; 0.1 s guard',name)
    b=poses['_HAMR_base_pose'];q=b[:,4:]/np.linalg.norm(b[:,4:],axis=1)[:,None]
    steps=2*np.degrees(np.arccos(np.clip(abs(np.sum(q[:-1]*q[1:],axis=1)),0,1)))
    for i in np.flatnonzero(steps>10):add(b[i,0]-.1,b[i+1,0]+.1,'base rotation jump >10 degrees; 0.1 s guard','_HAMR_base_pose')
    for key,v in enc.items():
        name='%s/%d'%key
        for i in range(len(v)-1):
            gap=v[i+1]['t']-v[i]['t']
            if gap<=0:raise ValueError('Nonincreasing mapped time: '+name)
            if gap>GAP:add(v[i]['t'],v[i+1]['t'],'encoder gap >0.1 s',name)
            if v[i]['boot']!=v[i+1]['boot']:add(v[i]['t']-DT,v[i+1]['t']+DT,'encoder boot transition; never bridge',name)
            if not v[i+1]['unwrap_continuous']:add(v[i]['t']-DT,v[i+1]['t']+DT,'encoder continuity flag lost',name)
    merged=[]
    for a,b in sorted((r['start_s'],r['end_s']) for r in bad):
        if merged and a<=merged[-1][1]:merged[-1][1]=max(merged[-1][1],b)
        else:merged.append([a,b])
    candidates=[];cursor=lo
    for a,b in merged:
        if a>cursor:candidates.append([cursor,a])
        cursor=max(cursor,b)
    if cursor<hi:candidates.append([cursor,hi])
    return bad,merged,[q for q in candidates if q[1]-q[0]>=MIN_DURATION],[q for q in candidates if q[1]-q[0]<MIN_DURATION]

def channel_interpolate(v,t,center_wheel=None):
    times=np.array([r['t'] for r in v]);ia=max(0,np.searchsorted(times,t[0],side='right')-1);ib=min(len(v),np.searchsorted(times,t[-1],side='left')+1)
    sub=v[ia:ib]
    assert len(set(r['boot'] for r in sub))==1, 'Segment spans boot boundary'
    assert np.max(np.diff([r['t'] for r in sub]))<=GAP+1e-9, 'Segment spans encoder gap'
    raw=np.unwrap([r['angle'] for r in sub])
    if center_wheel:
        ref=MAPPING[center_wheel]['center_reference_count']*2*np.pi/16384
        raw=MAPPING[center_wheel]['center_sign']*(raw-raw[0]+wrap(raw[0]-ref))
    out=np.interp(t,[r['t'] for r in sub],raw)
    if not center_wheel:out-=out[0]
    return out,sub[0]['boot']

def prepare(run_dir):
    run=run_dir.name;channels,meta=read_channels(run_dir/'samples.csv')
    native=save_native(run,channels)
    cache=ROOT/'work/calibration_audit'/(run+'.npz')
    report={'run':run,'source_csv':str(run_dir/'samples.csv'),'source_csv_sha256':sha(run_dir/'samples.csv'),
        'channels':meta,'native_encoder_npz':native,'segments':[],'source_sidecars':{p.name:sha(p) for p in run_dir.glob('*.json')},
        'local_bag_files':[str(p) for p in run_dir.rglob('*') if p.suffix in ['.mcap','.db3']],
        'source_pose_cache':str(cache) if cache.exists() else None}
    if not cache.exists():
        report['status']='encoder_only_no_local_pose_cache'
        report['unavailable_reasons']=['No local Vicon/robot bag inside this recording folder or matching pose cache.']
        clock=json.loads((run_dir/'clock-map.json').read_text());session=json.loads((run_dir/'session.json').read_text())
        report['saved_clock_errors']=clock.get('clock_errors',[])
        report['session_state']=session.get('state');report['remote_robot_bag_path']=session.get('robot_recorder',{}).get('bag_path')
        report['unmapped_channels']=[name for name,m in meta.items() if m['unmapped_samples']]
        return report
    report['source_pose_cache_sha256']=sha(cache)
    with np.load(cache) as data:
        poses={k:data[k].copy() for k in POSES}
        first=next(r for r in channels['front_left',3] if r['epoch'] is not None)
        origin=first['epoch']-round(data['front_left'][0,0]*1e9)
    enc={k:[dict(r,t=(r['epoch']-origin)*1e-9) for r in v if r['epoch'] is not None] for k,v in channels.items()}
    if any(not v for v in enc.values()):
        report['status']='insufficient_mapped_encoder_channels';return report
    for w in WHEELS:assert meta[w+'/3']['devices']==[MAPPING[w]['center_reference_device_id']]
    for name,a in poses.items():assert np.all(np.diff(a[:,0])>0),name+' nonincreasing pose time'
    lo=max(*[a[0,0] for a in poses.values()],*[v[0]['t'] for v in enc.values()])
    hi=min(*[a[-1,0] for a in poses.values()],*[v[-1]['t'] for v in enc.values()])
    report.update(epoch_origin_ns=origin,common_available_interval_s=[float(lo),float(hi)],common_available_duration_s=float(hi-lo),case_elapsed_origin_s=float(lo))
    report['source_intervals_s']={**{k:[float(a[0,0]),float(a[-1,0])] for k,a in poses.items()},**{'%s/%d'%k:[v[0]['t'],v[-1]['t']] for k,v in enc.items()}}
    bad,merged,segments,short=exclusions_and_segments(lo,hi,poses,enc)
    report.update(exclusion_events=bad,excluded_merged_intervals_s=merged,short_valid_intervals_not_sampled_s=short)
    base=poses['_HAMR_base_pose'];slerp=Slerp(base[:,0],Rotation.from_quat(base[:,4:]))
    for sid,(a,b) in enumerate(segments):
        # Keep endpoints in observed continuous domains; no sample crosses a gap.
        t=a+np.arange(int(np.floor((b-a)/DT))+1)*DT
        psi=np.unwrap(slerp(t).as_euler('xyz')[:,2])+np.pi/2
        arr={'time_s':t,'case_elapsed_s':t-lo,'elapsed_s':t-lo,'segment_elapsed_s':t-t[0],
            'case_epoch_origin_ns':np.array(origin,dtype=np.int64),'segment_id':np.array(sid,dtype=np.int32)}
        channel_boots={}
        for w,key in [('front_left','_Left_Caster_pose'),('front_right','_Right_Caster_pose')]:
            pp=poses[key];xy=np.column_stack([np.interp(t,pp[:,0],pp[:,j]) for j in [1,2]])
            xy=savgol_filter(xy,11,2,axis=0)
            delta,boot=channel_interpolate(enc[w,3],t,w);channel_boots[w+'/3']=boot
            b1,boot=channel_interpolate(enc[w,1],t);channel_boots[w+'/1']=boot
            b2,boot=channel_interpolate(enc[w,2],t);channel_boots[w+'/2']=boot
            for k,v in {'p_xy':xy,'psi':psi,'delta_enc_rad':delta,'beta_h1_change_rad':b1,'beta_h2_change_rad':b2}.items():arr[w+'__'+k]=v
        dest=INPUTS/(run+f'__segment{sid:03d}.npz')
        np.savez_compressed(dest,**arr)
        report['segments'].append({'segment_id':sid,'path':str(dest),'sha256':sha(dest),'time_interval_s':[float(t[0]),float(t[-1])],
            'case_elapsed_interval_s':[float(t[0]-lo),float(t[-1]-lo)],'duration_s':float(t[-1]-t[0]),'samples':len(t),
            'encoder_boot_ids':channel_boots,'initialization':'Fresh initial measured CENTER and zero H1/H2 phase at this segment start; no comparison winding carried across excluded intervals.'})
    report['status']='prepared' if report['segments'] else 'no_shared_continuous_interval_at_least_1_s'
    report['prepared_duration_s']=sum(s['duration_s'] for s in report['segments'])
    report['coverage_fraction']=report['prepared_duration_s']/(hi-lo)
    return report

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',help='Exact recording folder name; omitted means all.')
    args=parser.parse_args()
    manifest={'schema_version':1,'generator_script':str(Path(__file__)),'generator_sha256':sha(Path(__file__)),
        'mapping_source':str(MAPPING_PATH),'mapping_sha256':sha(MAPPING_PATH),'dt_s':DT,'min_segment_duration_s':MIN_DURATION,
        'definition':'100 Hz resampling, 11-sample order-2 Savitzky-Golay smoothing of each fork-marker XY independently within each valid segment. Upright yaw = base Vicon yaw + pi/2. Ground +Z. No fitted parameters.',
        'time_schema':{'time_s':'Seconds relative to earliest Vicon receipt time in the case; identical origin to prior pose audit.',
            'case_elapsed_s':'time_s minus first common available timestamp across 3 poses and 6 encoders; retains excluded gaps.',
            'elapsed_s':'Alias of case_elapsed_s; never reset across files from the same case.',
            'segment_elapsed_s':'Seconds from this valid segment start.'},
        'phase_schema':{'delta_enc_rad':'Unwrapped CENTER within this segment, initialized to saved-reference principal angle. Unknown integer-turn offsets across gaps are not connected.',
            'beta_h1_change_rad':'Recorded H1 angle change from this segment start; raw positive sign and source count scale unchanged.',
            'beta_h2_change_rad':'Recorded H2 angle change from this segment start; raw positive sign and source count scale unchanged.'},
        'limitations':['Fork marker XY is an uncalibrated ball-center proxy.',
            'Base yaw + pi/2 and saved CENTER references/signs are prior assumptions, not fitted here.',
            'Encoder mapped timestamps are existing two-way midpoint estimates; accuracy has not been independently validated.',
            'H1/H2 count scale, sign and absolute phase remain unverified.',
            'No interpolation over gaps >0.1 s, encoder boot changes, guarded pose jumps, or rejected base quaternion jumps.',
            'Native encoder-only files use separate board clocks; their curves must not be presented as synchronized with Vicon.'],
        'cases':[],'failures':[]}
    for d in sorted(SOURCE.iterdir()):
        if not d.is_dir() or not (d/'samples.csv').exists():continue
        if args.case and d.name != args.case:continue
        try:
            r=prepare(d);manifest['cases'].append(r)
            print(json.dumps({'run':r['run'],'status':r['status'],'segments':len(r['segments']),
                'common_s':r.get('common_available_duration_s'),'prepared_s':r.get('prepared_duration_s'),
                'case_elapsed_intervals':[s['case_elapsed_interval_s'] for s in r['segments']]}),flush=True)
        except Exception as e:
            manifest['failures'].append({'run':d.name,'error':repr(e)});print('ERROR '+d.name+' '+repr(e),flush=True)
    if args.case and not manifest['cases'] and not manifest['failures']:
        raise ValueError('No recording found: '+args.case)
    (OUT/'input_manifest.json').write_text(json.dumps(manifest,indent=2))
    print('MANIFEST '+str(OUT/'input_manifest.json'),flush=True)
    assert not manifest['failures'],manifest['failures']

if __name__=='__main__':main()
