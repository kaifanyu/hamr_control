#!/usr/bin/env python3
"""Independent three-bag sensor audit. Run with sourced ROS and /usr/bin/python3.

Derivative fits are diagnostics of timing and model error, not white-noise R.
All clocks use bag receipt time; Vicon source clock is unsynchronized.
"""
from collections import Counter, defaultdict
from pathlib import Path
import json
import math
import numpy as np
from scipy.signal import savgol_filter
from scipy.ndimage import minimum_filter1d
from scipy.integrate import cumulative_trapezoid
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
TOPICS = {'/HAMR_base/pose':'pose', '/HAMR_base/odom':'vicon',
          '/wheel_odom':'wheel', '/imu/data':'imu', '/local_HAMR/odom':'ekf',
          '/imu/calib_status':'calib', '/left_wheel/encoder_ticks':'left',
          '/right_wheel/encoder_ticks':'right', '/tf_static':'tf'}

def quat(q):
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
    pitch = math.asin(np.clip(2*(q.w*q.y-q.z*q.x), -1, 1))
    return yaw, roll, pitch, q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w

def stats(a):
    a = np.asarray(a)
    if not len(a): return {'n':0}
    return dict(n=len(a),mean=float(a.mean()),std=float(a.std()),
                rmse=float(np.sqrt(np.mean(a*a))),median=float(np.median(a)),
                p01=float(np.quantile(a,.01)),p99=float(np.quantile(a,.99)),
                min=float(a.min()),max=float(a.max()))

def extract(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=str(bag), storage_id='mcap'), ConverterOptions('', ''))
    ty = {x.name:get_message(x.type) for x in r.get_all_topics_and_types() if x.name in TOPICS}
    data = defaultdict(list); frames = defaultdict(Counter); covariance = {}; tfs=[]
    while r.has_next():
        topic,raw,t = r.read_next()
        if topic not in ty: continue
        m = deserialize_message(raw,ty[topic]); name=TOPICS[topic]
        if name=='tf':
            for tf in m.transforms:
                tfs.append(dict(parent=tf.header.frame_id,child=tf.child_frame_id,
                  xyz=[tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z],
                  quat=[tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w]))
            continue
        a = dict(t=t/1e9)
        if hasattr(m,'header'):
            a['header']=m.header.stamp.sec+m.header.stamp.nanosec/1e9
            frames[name][m.header.frame_id+' -> '+getattr(m,'child_frame_id','')]+=1
        if name=='calib': a.update(dict(zip(['sys','gyro','accel','mag'],m.data)))
        elif name in ('left','right'): a['ticks']=m.data
        elif name=='imu':
            ya,ro,pi,norm = quat(m.orientation)
            a.update(yaw=ya,roll=ro,pitch=pi,qnorm=norm,
                     wx=m.angular_velocity.x,wy=m.angular_velocity.y,wz=m.angular_velocity.z,
                     ax=m.linear_acceleration.x,ay=m.linear_acceleration.y,az=m.linear_acceleration.z)
            covariance[name]={k:list(getattr(m,k)) for k in ('orientation_covariance','angular_velocity_covariance','linear_acceleration_covariance')}
        else:
            p = m.pose.pose if hasattr(m.pose,'pose') else m.pose
            ya,ro,pi,norm = quat(p.orientation)
            a.update(x=p.position.x,y=p.position.y,z=p.position.z,yaw=ya,roll=ro,pitch=pi,qnorm=norm)
            if hasattr(m,'twist'):
                a.update(vx=m.twist.twist.linear.x,vy=m.twist.twist.linear.y,wz=m.twist.twist.angular.z)
                covariance[name]=dict(pose=list(m.pose.covariance),twist=list(m.twist.covariance))
        data[name].append(a)
    origin = min(rows[0]['t'] for rows in data.values())
    D={k:{f:np.array([a[f] for a in rows]) for f in rows[0]} for k,rows in data.items()}
    for a in D.values():
        a['t']-=origin
        if 'header' in a:a['header']-=origin
    np.savez_compressed(OUT/(bag.name+'.npz'),**{k+'_'+f:a for k,rows in D.items() for f,a in rows.items()},origin=origin)
    return D,frames,covariance,tfs

def audit(bag):
    D,frames,cov,tfs=extract(bag)
    R=dict(bag=bag.name,timing={},frames={k:dict(v) for k,v in frames.items()},covariance=cov,tf=tfs)
    for k,a in D.items():
        dt=np.diff(a['t'])
        R['timing'][k]=dict(n=len(a['t']),duration=float(a['t'][-1]-a['t'][0]),dt=stats(dt),gaps_over_100ms=int(sum(dt>.1)))
        if 'header' in a:
            dh=np.diff(a['header']); age=a['t']-a['header']
            R['timing'][k].update(age=stats(age),reversed_stamps=int(sum(dh<0)),duplicate_stamps=int(sum(dh==0)))
    R['calibration']={f:dict(zip(map(str,np.unique(a)),map(int,np.unique(a,return_counts=True)[1]))) for f,a in D['calib'].items() if f!='t'}
    v=D['vicon']; dt=np.diff(v['t']); dist=np.hypot(np.diff(v['x']),np.diff(v['y']))
    bad=(v['qnorm']<.9)|(v['qnorm']>1.1)|(v['z']<.25)|(v['z']>.40)|(abs(v['roll'])>.35)|(abs(v['pitch'])>.35)
    jump=np.r_[False,dist>.08]; moving=np.r_[False,dist/np.maximum(dt,1e-9)>2.0]
    R['vicon_validity']=dict(invalid_pose=int(sum(bad)),jump_over_8cm=int(sum(jump)),receipt_speed_over_2mps=int(sum(moving)),
       invalid_times=v['t'][bad].tolist(),jump_times=v['t'][jump].tolist(),z=stats(v['z']),roll=stats(v['roll']),pitch=stats(v['pitch']))
    t=np.arange(max(D[k]['t'][0] for k in ('vicon','wheel','imu'))+.3,
                min(D[k]['t'][-1] for k in ('vicon','wheel','imu'))-.3,.01)
    V={}; valid=np.ones(len(t),bool)
    for k in ('vicon','wheel','imu','ekf'):
        a=D[k]; idx=np.clip(np.searchsorted(a['t'],t),1,len(a['t'])-1)
        good=(t-a['t'][idx-1]<.08)&(a['t'][idx]-t<.08)
        age=a['t']-a['header']; med=np.median(age)
        good&=(abs(age[idx]-med)<.06)&(abs(age[idx-1]-med)<.06)
        if k=='vicon':good&=~bad[idx]&~bad[idx-1]
        valid&=good
        V[k]={f:np.interp(t,a['t'],np.unwrap(x) if f=='yaw' else x) for f,x in a.items() if f not in ('t','header')}
    valid=minimum_filter1d(valid.astype(int),31,mode='constant').astype(bool)
    p=V['vicon']; w=V['wheel']; imu=V['imu']; yaw=savgol_filter(p['yaw'],31,3)
    dx=savgol_filter(p['x'],31,3,deriv=1,delta=.01);dy=savgol_filter(p['y'],31,3,deriv=1,delta=.01)
    truth=dict(vx=np.cos(yaw)*dx+np.sin(yaw)*dy,vy=-np.sin(yaw)*dx+np.cos(yaw)*dy,
               wz=savgol_filter(p['yaw'],31,3,deriv=1,delta=.01))
    stationary=minimum_filter1d(((np.hypot(dx,dy)<.015)&(abs(truth['wz'])<.025)&valid).astype(int),41).astype(bool)
    R['stationary_raw_imu_approx']={f:stats(imu[f][stationary]) for f in ('wx','wy','wz','ax','ay','az')}
    R['valid_duration_s']=float(sum(valid)*.01)
    R['stationary_duration_s']=float(sum(stationary)*.01)
    R['velocity_fit']={}
    for label,sensor,target in [('wheel_vx',w['vx'],truth['vx']),('wheel_vy',w['vy'],truth['vy']),('wheel_wz',w['wz'],truth['wz']),
                                ('imu_wz',imu['wz'],truth['wz']),('imu_yaw_rate',savgol_filter(imu['yaw'],31,3,deriv=1,delta=.01),truth['wz'])]:
        fitmask=valid&(np.hypot(dx,dy)>.04)
        best=None
        for delay in np.arange(-.7,.701,.01):
            s=np.interp(t+delay,t,sensor)
            M=np.c_[s[fitmask],np.ones(sum(fitmask))]
            coeff=np.linalg.lstsq(M,target[fitmask],rcond=None)[0]
            err=M@coeff-target[fitmask];score=float(np.sqrt(np.mean(err*err)))
            if best is None or score<best['rmse']:
                best=dict(delay=float(delay),scale=float(coeff[0]),offset=float(coeff[1]),rmse=score)
        zero_coeff=np.linalg.lstsq(np.c_[sensor[fitmask],np.ones(sum(fitmask))],target[fitmask],rcond=None)[0]
        best['zero_lag_coeff']=zero_coeff.tolist();best['raw_zero_lag_rmse']=float(np.sqrt(np.mean((sensor[fitmask]-target[fitmask])**2)))
        R['velocity_fit'][label]=best
    fitmask=valid&(np.hypot(dx,dy)>.04)
    # Fit reference body velocity against encoder mean velocity and yaw rate.
    # Wheel vx has a one-step rotation artifact; undo it to recover mean speed.
    orig=D['wheel'];hdt=np.r_[np.median(np.diff(orig['header'])),np.diff(orig['header'])]
    ang=orig['wz']*hdt
    mean_speed=np.sin(ang)*orig['vx']+np.cos(ang)*orig['vy']
    mean=np.interp(t,orig['t'],mean_speed)
    M=np.c_[mean,w['wz'],np.ones(len(t))]
    R['geometry_fit']={f:np.linalg.lstsq(M[fitmask],truth[f][fitmask],rcond=None)[0].tolist() for f in ('vx','vy','wz')}
    # yaw-rate-dependent lever arm should be evaluated using independently measured gyro.
    Mg=np.c_[mean,imu['wz'],np.ones(len(t))]
    R['geometry_fit_gyro']={f:np.linalg.lstsq(Mg[fitmask],truth[f][fitmask],rcond=None)[0].tolist() for f in ('vx','vy','wz')}
    R['yaw_relation']={}
    ytruth=p['yaw']-p['yaw'][0]
    for name,angle in [('imu_orientation',imu['yaw']-imu['yaw'][0]),('integrated_gyro',cumulative_trapezoid(imu['wz'],t,initial=0)),('wheel',w['yaw']-w['yaw'][0])]:
        M=np.c_[angle[valid],np.ones(sum(valid))];coeff=np.linalg.lstsq(M,ytruth[valid],rcond=None)[0]
        R['yaw_relation'][name]=dict(scale=coeff[0],offset=coeff[1],rmse=float(np.sqrt(np.mean((M@coeff-ytruth[valid])**2))),
               uncorrected_rmse=float(np.sqrt(np.mean((angle[valid]-ytruth[valid])**2))))
    # Save diagnostic-only regular grid and masks for consistent cross-bag fitting.
    np.savez_compressed(OUT/(bag.name+'_grid.npz'),t=t,valid=valid,stationary=stationary,mean=mean,
       **{'truth_'+k:a for k,a in truth.items()},**{k+'_'+f:a for k,rows in V.items() for f,a in rows.items()})
    (OUT/(bag.name+'.json')).write_text(json.dumps(R,indent=2))
    print(json.dumps({k:R[k] for k in ['bag','vicon_validity','valid_duration_s','stationary_duration_s','stationary_raw_imu_approx','velocity_fit','geometry_fit','geometry_fit_gyro','yaw_relation']},indent=2))
    return R

if __name__=='__main__':
    results=[audit(b) for b in sorted(ROOT.glob('hamr_hw_*')) if b.is_dir()]
    (OUT/'all_bags.json').write_text(json.dumps(results,indent=2))
