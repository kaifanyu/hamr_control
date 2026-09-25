#!/usr/bin/env python3
"""Quantify payload refresh independently from publication frequency."""
from pathlib import Path
import json
import numpy as np
from scipy.optimize import minimize_scalar
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
fields=['yaw','roll','pitch','wx','wy','wz','ax','ay','az']
results={};fig,axes=plt.subplots(3,1,figsize=(13,10))
for ax,path in zip(axes,sorted(OUT.glob('hamr*_grid.npz'))):
    name=path.name.replace('_grid.npz','');d=np.load(OUT/(name+'.npz'));g=np.load(path)
    t=d['imu_t'];values=np.column_stack([d['imu_'+k] for k in fields])
    changed=np.r_[True,np.any(np.diff(values,axis=0)!=0,axis=1)];ct=t[changed]
    intervals=np.diff(ct);moving=abs(d['imu_wz'][1:])>.03
    # Compare the first occurrence of each changed gyro sample against Vicon.
    # No zero-order-held duplicates are used in this acquisition-delay fit.
    gt=g['t'];mask=g['valid'];truth=g['truth_wz'];vi=np.interp(ct,gt,mask.astype(float))>.99
    vi&=(ct>gt[0]+.6)&(ct<gt[-1]-.6)
    def score(delay):
        target=np.interp(ct[vi]-delay,gt,truth)
        sens=d['imu_wz'][changed][vi]
        scale=np.dot(sens,target)/np.dot(sens,sens)
        return np.mean((scale*sens-target)**2)
    opt=minimize_scalar(score,bounds=(-.3,.6),method='bounded')
    delays=np.arange(-.3,.6001,.002);scores=np.array([score(x) for x in delays]);best=delays[np.argmin(scores)]
    target=np.interp(ct[vi]-best,gt,truth);sens=d['imu_wz'][changed][vi]
    scale=np.dot(sens,target)/np.dot(sens,sens)
    results[name]=dict(message_count=len(t),unique_payload_count=int(sum(changed)),
      publication_hz=float((len(t)-1)/(t[-1]-t[0])),payload_refresh_hz=float((sum(changed)-1)/(ct[-1]-ct[0])),
      identical_consecutive_payload_fraction=float(np.mean(~changed[1:])),gyro_repeated_while_turning=float(np.mean(np.diff(d['imu_wz'])[moving]==0)),
      refresh_interval_quantiles_s=dict(zip(['min','p10','median','p90','max'],map(float,np.quantile(intervals,[0,.1,.5,.9,1])))),
      fresh_gyro_delay_s=float(best),fresh_gyro_scale=float(scale),fresh_gyro_fit_rmse=float(np.sqrt(np.min(scores))))
    # Focus on a consistently informative first turn.
    inds=np.flatnonzero((abs(truth)>.20)&mask);center=gt[inds[0]] if len(inds) else gt[len(gt)//2]
    ax.plot(gt,np.where(mask,truth,np.nan),label='Vicon yaw derivative (0.31 s smoothing)',color='black',lw=2)
    ax.step(t,d['imu_wz'],where='post',label='65 Hz IMU published gyro',color='C1')
    ax.scatter(ct,d['imu_wz'][changed],label='Complete IMU payload changed',color='C3',s=24,zorder=4)
    ax.plot(gt,g['wheel_wz']*.83,label='Wheel yaw rate × 0.83',alpha=.7,color='C0')
    ax.set_xlim(center-1,center+5);ax.set_ylim(-.12,1.05);ax.set_ylabel('Yaw rate (rad/s)');ax.set_title(name+' — new IMU payload every ≈0.337 s');ax.grid(alpha=.3)
axes[0].legend(fontsize=8);axes[-1].set_xlabel('Seconds since bag start');fig.tight_layout();fig.savefig(OUT/'imu_payload_refresh.png',dpi=170)
(OUT/'imu_freshness.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
