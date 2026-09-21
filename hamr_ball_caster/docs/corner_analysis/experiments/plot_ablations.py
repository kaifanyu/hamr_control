#!/usr/bin/env python3
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent
names=['baseline','no_slew','half_speed','quintic_stop']
labels={'baseline':'Original: 0.25 m/s, 15 rad/s²','no_slew':'No wheel slew limit: 0.25 m/s','half_speed':'Half speed: 0.125 m/s, 15 rad/s²','quintic_stop':'Quintic stops: peak 0.25 m/s, 15 rad/s²'}
colors=dict(zip(names,['#c93838','#e99513','#287fc2','#23854a']))
reports={n:json.loads((root/f'{n}.json').read_text())for n in names}
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.22})
fig,axes=plt.subplots(2,2,figsize=(13,9),layout='constrained');a,b,c,d=axes.ravel()
for n,r in reports.items():
 ss=r['samples'];tc=r['summary']['waypoints'][1]['scheduled_arrival_s'];ts=np.array([s['trajectory_time_s']-tc for s in ss]);pos=np.array([s['position_m'][:2]for s in ss]);errors=np.array([s['position_error_m']for s in ss]);m=(ts>=-1)&(ts<=1.6)
 a.plot(pos[m,0]*1000,(pos[m,1]-2)*1000,color=colors[n],lw=2,label=labels[n])
 b.plot(ts[m],errors[m]*1000,color=colors[n],lw=2,label=labels[n])
a.plot([0,0,-500],[-300,0,0],color='black',ls='--',lw=1.5,label='Reference path');a.scatter([0],[0],marker='x',color='black',s=60,zorder=5)
a.set(xlim=(-420,45),ylim=(-220,85),xlabel='World x relative to first corner (mm)',ylabel='World y relative to first corner (mm)',title='A. Same corner geometry; timing differs by case');a.set_aspect('equal',adjustable='box')
b.axvline(0,color='black',ls='--',lw=1);b.set(xlim=(-.2,1.5),ylim=(0,72),xlabel='Simulation time after first corner (s)',ylabel='Timed position error (mm)',title='B. Corner error collapses with a feasible reference');b.legend(loc='upper right',fontsize=8)
for n,style in [('baseline','-'),('no_slew','-')]:
 ss=reports[n]['samples'];tc=reports[n]['summary']['waypoints'][1]['scheduled_arrival_s'];ts=np.array([s['trajectory_time_s']-tc for s in ss]);m=(ts>=-.12)&(ts<=.65)
 measured=np.array([s['joint_velocities_rad_s']['left_rocker_left_wheel_joint']for s in ss]);c.plot(ts[m],measured[m],color=colors[n],lw=2,label=f'{n.replace("_"," ")}: measured left wheel')
 if n=='baseline':
  raw=np.array([s['raw_wheels_rad_s'][0]for s in ss]);cmd=np.array([s['commands_rad_s'][0]for s in ss]);c.plot(ts[m],raw[m],color='black',ls='--',lw=1.2,label='baseline: raw inverse-kinematic request');c.plot(ts[m],cmd[m],color=colors[n],ls=':',lw=1.8,label='baseline: published after slew limit')
c.axvline(0,color='black',ls='--',lw=1);c.set(xlim=(-.12,.65),xlabel='Simulation time after first corner (s)',ylabel='Left wheel angular velocity (rad/s)',title='C. A limiter delays wheel reversal');c.legend(fontsize=8,loc='upper right')
res=json.loads((root/'wheel_body_residual.json').read_text())['no_slew']['first_corner_50_to_250ms'];tc=reports['no_slew']['summary']['waypoints'][1]['scheduled_arrival_s'];ts=np.array([r['trajectory_time_s']-tc for r in res]);
d.plot(ts,[r['measured_body_forward_m_s']for r in res],color='#303b50',lw=2,label='Measured body forward velocity (pose differences)');d.plot(ts,[np.mean(r['wheel_surface_velocity_m_s'])for r in res],color=colors['no_slew'],lw=2,label='Forward velocity inferred from wheel speeds');d.axhline(0,color='black',lw=1,ls=':');d.set(xlabel='Simulation time after first corner (s)',ylabel='Body forward velocity (m/s)',title='D. No slew limit: wheel reversal does not reverse body instantly');d.legend(fontsize=8,loc='upper right')
fig.suptitle('Full Gazebo vehicle: controlled corner experiments\nAll cases retain the same CAD casters, geometry, contact model and controller gains',fontsize=14)
fig.savefig(root.parent/'gazebo_ablation_comparison.png',dpi=180)
fig.savefig(root.parent/'gazebo_ablation_comparison.pdf')
print(root.parent/'gazebo_ablation_comparison.png')
