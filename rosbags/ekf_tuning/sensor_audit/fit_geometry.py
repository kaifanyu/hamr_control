#!/usr/bin/env python3
"""Fit shared physical wheel parameters with held-out bag evaluations.

This is explicit midpoint dead reckoning, not an EKF. It diagnoses systematic
errors independently of Q/R. No per-run spatial alignment beyond the initial
reference pose is allowed. Vicon is only the calibration/scoring target.
"""
from pathlib import Path
import json
import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent

def load():
    bags=[]
    for path in sorted(OUT.glob('hamr*_grid.npz')):
        name=path.name.replace('_grid.npz','');d=np.load(OUT/(name+'.npz'));g=np.load(path)
        t=g['t'];mask=g['valid'];v=d['vicon_t']
        yaw=np.interp(t,v,np.unwrap(d['vicon_yaw']));x=np.interp(t,v,d['vicon_x']);y=np.interp(t,v,d['vicon_y'])
        ct,st=np.cos(yaw[0]),np.sin(yaw[0]);x-=x[0];y-=y[0]
        truth=np.c_[ct*x+st*y,-st*x+ct*y,yaw-yaw[0]]
        w=d['wheel_wz'];wdt=np.r_[np.median(np.diff(d['wheel_header'])),np.diff(d['wheel_header'])]
        mean=d['wheel_vx']*np.sin(w*wdt)+d['wheel_vy']*np.cos(w*wdt)
        bags.append(dict(name=name,t=t,mask=mask,truth=truth,wt=d['wheel_t'],mean=mean,w=w,it=d['imu_t'],gyro=d['imu_wz']))
    return bags

def predict(b,p,mode='wheel',time=False):
    # s = mean distance scale; diff = fractional right/left mismatch;
    # sy = wheel yaw scale; arm = base origin lever arm; lag = wheel delay.
    s,diff,sy,arm=p[:4];lag=p[4] if len(p)>4 else 0.
    t=b['t'];mean=np.interp(t-lag,b['wt'],b['mean']);w=np.interp(t-lag,b['wt'],b['w'])
    velocity=s*(mean+diff*.350*w)
    wheel_rate=sy*(w+diff/.350*mean)
    gyro=np.interp(t+(.36 if time else 0),b['it'],b['gyro'])
    rate=wheel_rate if mode=='wheel' else gyro
    if mode=='blend':rate=.8*wheel_rate+.2*gyro
    yaw=cumulative_trapezoid(rate,t,initial=0)
    vx=-arm*wheel_rate
    dx=np.cos(yaw)*vx-np.sin(yaw)*velocity;dy=np.sin(yaw)*vx+np.cos(yaw)*velocity
    return np.c_[cumulative_trapezoid(dx,t,initial=0),cumulative_trapezoid(dy,t,initial=0),yaw]

def metrics(b,p,mode='wheel',time=False):
    pred=predict(b,p,mode,time);err=(pred-b['truth'])[b['mask']];xy=np.linalg.norm(err[:,:2],axis=1)
    return dict(xy_rmse=float(np.sqrt(np.mean(xy*xy))),xy_final=float(xy[-1]),
                yaw_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(err[:,2]**2)))),yaw_final_deg=float(np.rad2deg(err[-1,2])))

def fit(bags,mode='wheel',time=False,delay=False):
    start=np.array([1.,0.,.83,.30]+([.15] if delay else []));lo=[.75,-.08,.65,.1]+([0.] if delay else []);hi=[1.25,.08,1.1,.6]+([.4] if delay else [])
    def residual(p):
        return np.concatenate([((predict(b,p,mode,time)-b['truth'])[b['mask']][::5]*[1,1,1])/np.sqrt(sum(b['mask'])/5) for b in bags]).ravel()
    result=least_squares(residual,start,bounds=(lo,hi),max_nfev=250,x_scale='jac',ftol=1e-9,xtol=1e-9,gtol=1e-9)
    return result.x

def main():
    bags=load();results={'method':'Midpoint continuous dead reckoning on receipt timeline. Fits minimize equally weighted per-bag XY and yaw residuals. Units: yaw rad receives 1 m/rad weight. Only initial SE2 alignment. p=[velocity_scale,right_left_asymmetry,yaw_scale,lever_arm_m,optional_wheel_delay_s]. Positive wheel delay evaluates wheel(t-delay), causal past wheel. Gyro advance is diagnostic and noncausal.', 'experiments':{}}
    for mode,time,delay in [('wheel',False,False),('wheel',False,True),('gyro',False,False),('gyro',True,False),('blend',False,False),('blend',False,True)]:
        name=mode+('_gyro_advance' if time else '')+('_wheel_delay' if delay else '')
        p=fit(bags,mode,time,delay);r=dict(parameters=p.tolist(),scores={b['name']:metrics(b,p,mode,time) for b in bags},heldout=[])
        for i,b in enumerate(bags):
            pp=fit([bb for j,bb in enumerate(bags) if j!=i],mode,time,delay)
            r['heldout'].append(dict(bag=b['name'],parameters=pp.tolist(),score=metrics(b,pp,mode,time)))
        results['experiments'][name]=r
        print(name,json.dumps(r),flush=True)
    results['baseline_correct_but_uncalibrated']={b['name']:metrics(b,[1,0,1,.45]) for b in bags}
    (OUT/'geometry_fits.json').write_text(json.dumps(results,indent=2))
    fig,axs=plt.subplots(1,3,figsize=(15,5))
    for ax,b in zip(axs,bags):
        ax.plot(b['truth'][:,0],b['truth'][:,1],color='black',lw=2,label='Vicon')
        for name in ['wheel','wheel_wheel_delay','gyro','blend_wheel_delay']:
            p=results['experiments'][name]['parameters'];mode=name.split('_')[0]
            pred=predict(b,p,mode);ax.plot(pred[:,0],pred[:,1],label=name)
        ax.set_title(b['name'].split('_')[-1]);ax.axis('equal');ax.grid(alpha=.3);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
    axs[0].legend(fontsize=7);fig.suptitle('Shared geometry fitted to all bags; direct integration diagnostic');fig.tight_layout();fig.savefig(OUT/'geometry_paths.png',dpi=160)

if __name__=='__main__':main()
