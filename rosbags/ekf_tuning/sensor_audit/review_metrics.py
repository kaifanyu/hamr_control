#!/usr/bin/env python3
"""Independent secondary scoring of the frozen EKF candidate."""
import json
from pathlib import Path
import sys
import numpy as np
from scipy.ndimage import minimum_filter1d
from scipy.signal import savgol_filter

OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(OUT.parent))
from engine import Engine
from evaluate import load_recordings, interp_pose, relative_pose, wrap
from tune import settings

candidate=dict(mode='wheel_gyro',linear_velocity_scale=.95,yaw_rate_scale=.813,
               lever_arm=.2825,r_vx=.000165,r_vy=.000165,r_wheel_wz=.004,r_gyro=.04)
profiles={'gyro_baseline':dict(mode='gyro'),
          'live_config_baseline':dict(mode='live',q_yaw=.005,q_wz=.01),
          'candidate':candidate}

def summary(err,mask):
    e=err[mask]
    if not len(e):return {'n':0}
    return dict(n=len(e),xy_rmse_m=float(np.sqrt(np.mean(np.sum(e[:,:2]**2,axis=1)))),
      yaw_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(e[:,2]**2)))),
      final_xy_m=float(np.linalg.norm(e[-1,:2])))

engine=Engine();results={}
for bag in load_recordings():
    results[bag.name]={};t=bag.times;v=bag.data['vicon'];age=v[:,1]-v[:,0]
    ix=np.clip(np.searchsorted(v[:,1],t),1,len(v)-1)
    strict=bag.valid&(abs(age[ix]-np.median(age))<.05)&(abs(age[ix-1]-np.median(age))<.05)
    strict=minimum_filter1d(strict.astype(int),11,mode='constant').astype(bool)
    p=bag.reference
    dx=savgol_filter(p[:,0],15,3,deriv=1,delta=.02);dy=savgol_filter(p[:,1],15,3,deriv=1,delta=.02)
    wz=savgol_filter(p[:,2],15,3,deriv=1,delta=.02)
    stationary=minimum_filter1d(((np.hypot(dx,dy)<.015)&(abs(wz)<.025)&strict).astype(int),31).astype(bool)
    moving=(np.hypot(dx,dy)>.04)&strict
    for name,profile in profiles.items():
        q,r,mask=settings(profile);events=bag.events(profile)
        tr=engine.run(events,q_diag=q,r_diag=r,mask=mask)
        pose=relative_pose(interp_pose(t,tr[:,:4]));err=pose-bag.reference;err[:,2]=wrap(err[:,2])
        vel=np.column_stack([np.interp(t,tr[:,0],tr[:,k]) for k in [4,5,6]])
        chunks=np.split(np.flatnonzero(stationary),np.flatnonzero(np.diff(np.flatnonzero(stationary))>1)+1)
        stops=[]
        for chunk in chunks:
            if len(chunk)<26:continue
            a,b=chunk[0],chunk[-1]
            stops.append(dict(start=float(t[a]),duration=float(t[b]-t[a]),
                 estimator_position_drift_m=float(np.linalg.norm(pose[b,:2]-pose[a,:2])),
                 reference_position_change_m=float(np.linalg.norm(p[b,:2]-p[a,:2])),
                 estimator_yaw_drift_deg=float(np.rad2deg(wrap(pose[b,2]-pose[a,2])))))
        out={'primary':summary(err,bag.valid),'strict_mask':summary(err,strict),
             'moving':summary(err,moving),'stationary':summary(err,stationary),
             'stationary_velocity_rms_m_s':float(np.sqrt(np.mean(np.sum(vel[stationary,:2]**2,axis=1)))),
             'stationary_wz_rms_rad_s':float(np.sqrt(np.mean(vel[stationary,2]**2))),
             'stationary_intervals':stops,'alignment_sensitivity':{}}
        for shift in [0.,.25,.5,1.]:
            tt=t[t>=t[0]+shift];po=relative_pose(interp_pose(tt,tr[:,:4]));ref=relative_pose(interp_pose(tt,v[:,[1,2,3,5]]));e=po-ref;e[:,2]=wrap(e[:,2]);ms=bag.valid[t>=t[0]+shift]
            out['alignment_sensitivity'][str(shift)]=summary(e,ms)
        results[bag.name][name]=out
    # Score pre-recorded EKF using the same interval and alignment.
    results[bag.name]['recorded_ekf']=bag.score(bag.recorded_pose)
(OUT/'frozen_candidate_review.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
