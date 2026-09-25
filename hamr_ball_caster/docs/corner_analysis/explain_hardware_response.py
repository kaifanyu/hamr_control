#!/usr/bin/env python3
"""Offline command/firmware reconstruction and first-turn telemetry figure."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
def held(v,t):
    return v[np.clip(np.searchsorted(v[:,0],t,side='right')-1,0,len(v)-1)]

results=[]
for path in sorted(OUT.glob('hamr_hw_*.npz')):
    d=np.load(path); r=d['reference_trajectory']; b=d['HAMR_base_odom']; g=d['live_gains']
    l=d['left_wheel_cmd_vel']; rr=d['right_wheel_cmd_vel']; st=d['wheel_control_status']
    t=l[:,0]; t0=r[0,0]; gh=held(g,t);rh=held(r,t);bh=held(b,t)
    actual=np.column_stack([np.interp(t,rr[:,0],rr[:,1]),l[:,1]])
    active=(t>t0+.1)&(t<r[-1,0]-.1)&(np.linalg.norm(actual,axis=1)>.01)&(t-gh[:,0]<.025)
    vx=rh[:,3]+gh[:,1]+gh[:,2]+gh[:,3]; vy=rh[:,4]+gh[:,4]+gh[:,5]+gh[:,6]
    yaw=bh[:,3]+np.pi/2; c=np.cos(yaw);s=np.sin(yaw)
    vx_b=c*vx+s*vy; vy_b=-s*vx+c*vy
    raw=np.column_stack([(vx_b+.35/.301*vy_b)/.122,(vx_b-.35/.301*vy_b)/.122])
    lim=28*2*np.pi/60
    scale=np.minimum(1,lim/np.maximum(np.max(np.abs(raw),axis=1),1e-6))
    scaled=raw*scale[:,None]; clipped=np.clip(raw,-lim,lim); error=abs(actual-scaled)[active]
    inferred={'bag':path.stem,'wheel_command_reconstruction':{
        'model':'r=.122m,a=.350m,b=.301m,yaw=Vicon+pi/2; reference + recorded P/D/I terms; common wheel scaling to28RPM',
        'sample_count':int(np.count_nonzero(active)),
        'absolute_component_error_median_p95_p99_rad_s':np.quantile(error,[.5,.95,.99]).tolist(),
        'pair_scaling_rmse_rad_s':float(np.sqrt(np.mean((actual[active]-scaled[active])**2))),
        'independent_clipping_rmse_rad_s':float(np.sqrt(np.mean((actual[active]-clipped[active])**2))),
        'nominal_feedforward_only_cap_fraction_at_recorded_yaw':None},'firmware_fit':{}}
    vxb=c*rh[:,3]+s*rh[:,4];vyb=-s*rh[:,3]+c*rh[:,4]
    nominal=np.column_stack([(vxb+.35/.301*vyb)/.122,(vxb-.35/.301*vyb)/.122])
    inferred['wheel_command_reconstruction']['nominal_feedforward_only_cap_fraction_at_recorded_yaw']=float(np.mean(np.max(abs(nominal[active]),axis=1)>lim))
    for side,col in [('left',1),('right',9)]:
        m=(st[:,0]>t0+.1)&(st[:,0]<r[-1,0]-.1)&(st[:,17]==1)&(abs(st[:,col+4])<4094)
        A=st[m][:,[col+5,col+6]]; pid=st[m,col+3];coef=np.linalg.lstsq(A,pid,rcond=None)[0]
        fit={'Kp_pwm_per_rpm':float(coef[0]),'Ki_pwm_per_rpm_second':float(coef[1]),
             'fit_rmse_pwm':float(np.sqrt(np.mean((A@coef-pid)**2)))}
        for direction,sign in [('positive',1),('negative',-1)]:
            mask=(st[:,col]*sign>1)&(abs(st[:,col+2])<3499)&(st[:,17]==1)
            ratio=st[mask,col+2]/st[mask,col]
            fit[direction+'_feedforward_pwm_per_rpm_median']=float(np.median(ratio))
        inferred['firmware_fit'][side]=fit
    # Check recorded D recurrence against the current analytic velocity-error form.
    rh=held(r,g[:,0]-.002);bh=held(b,g[:,0]-.002)
    ve=rh[:,3:5]-bh[:,4:6];df=g[:,[2,5]];dt=np.diff(g[:,0]);alpha=1-.8**(dt/.01)
    residual=df[1:]-(1-alpha[:,None])*df[:-1]-alpha[:,None]*.4*ve[1:]
    m=(g[1:,0]>t0+.1)&(g[1:,0]<r[-1,0]-.1)&(dt<.04)
    inferred['D_recurrence_consistency_rmse_m_s']=float(np.sqrt(np.mean(residual[m]**2)))
    results.append(inferred)
    if path.stem!='hamr_hw_20260916_193228':continue
    i=(np.flatnonzero(np.linalg.norm(np.diff(r[:,3:5],axis=0),axis=1)>.01)+1)[0];tc=r[i,0]
    fig,axs=plt.subplots(3,2,figsize=(14,11))
    pose=(b[:,0]>tc-.8)&(b[:,0]<tc+1.5);ref=(r[:,0]>tc-.8)&(r[:,0]<tc+1.5)
    axs[0,0].plot(r[ref,1],r[ref,2],'--',label='Reference');axs[0,0].plot(b[pose,1],b[pose,2],label='Vicon');axs[0,0].scatter([0],[2],marker='x',c='k',label='Corner');axs[0,0].set_aspect('equal',adjustable='datalim');axs[0,0].set_xlabel('World X (m)');axs[0,0].set_ylabel('World Y (m)');axs[0,0].set_title('Geometric overshoot: maximum Y = 2.0678 m')
    for name,col,color in [('old-direction Y',5,'tab:orange'),('new-direction −X',4,'tab:blue')]:
        sign=1 if col==5 else -1
        axs[0,1].plot(b[:,0]-tc,sign*b[:,col],c=color,label=name+' measured')
        axs[0,1].plot(r[:,0]-tc,sign*r[:,col-1],'--',c=color,label=name+' reference')
    axs[0,1].set_xlim(-.5,1.5);axs[0,1].set_ylabel('Velocity (m/s)');axs[0,1].set_title('Old-direction speed takes ~0.31 s to fall below 10%')
    for side,ci,mi,color in [('left',1,2,'tab:blue'),('right',9,10,'tab:orange')]:
        axs[1,0].plot(st[:,0]-tc,st[:,ci],'--',c=color,label=side+' target')
        axs[1,0].plot(st[:,0]-tc,st[:,mi],c=color,label=side+' measured',lw=1)
    axs[1,0].set_ylabel('RPM');axs[1,0].set_title('Target reaches 28 RPM; reported speed stays below it')
    for label,col in [('Feedforward',11),('PI correction',12),('Final output',13)]:
        axs[1,1].plot(st[:,0]-tc,st[:,col],label=label,lw=1)
    axs[1,1].axhline(4095,c='k',ls=':',label='PWM ceiling');axs[1,1].set_ylabel('PWM');axs[1,1].set_title('Right-wheel feedforward clips at 3500 PWM')
    rxy=np.column_stack([np.interp(b[:,0],r[:,0],r[:,j]) for j in [1,2]])
    err=b[:,1:3]-rxy
    axs[2,0].plot(b[:,0]-tc,np.linalg.norm(err,axis=1),label='Position vs scheduled reference')
    axs[2,0].plot(b[:,0]-tc,abs(b[:,2]-2),label='Distance from outgoing Y = 2 line')
    axs[2,0].set_ylabel('Error (m)');axs[2,0].set_ylim(-.005,.30);axs[2,0].set_title('~0.256 m scheduled tracking error is mostly lag, not corner bulge')
    axs[2,1].plot(st[:,0]-tc,st[:,15],label='Right-wheel stored integral (RPM·s)')
    axs[2,1].axhline(0,c='k',ls=':');axs[2,1].set_ylabel('RPM·s');axs[2,1].set_title('Integral starts negative, then unwinds and builds correction')
    for ax in axs.flat:
        ax.grid(alpha=.25);ax.legend(fontsize=8)
    for ax in [axs[0,1],*axs[1],*axs[2]]:
        ax.axvline(0,c='k',alpha=.3);ax.set_xlabel('Seconds from first reference corner')
        if ax!=axs[0,1]:ax.set_xlim(-.5,6)
    fig.suptitle('Recorded hardware: 2026-09-16 19:32:28, 0.25 m/s — aligned by local bag receipt time')
    fig.tight_layout();fig.savefig(OUT/'hardware_first_corner.png',dpi=160);plt.close(fig)
(OUT/'hardware_response_reconstruction.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
