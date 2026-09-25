#!/usr/bin/env python3
"""Timestamp-aligned rolling-constraint diagnostics, not calibrated tire slip."""
import json,math
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parent
out={}
for case in ('baseline','no_slew','half_speed','quintic_stop','no_slew_half_step'):
 path=root/f'{case}.json'
 if not path.exists():continue
 report=json.loads(path.read_text());cfg=report['configuration'];ss=report['samples'];ss=[s for s in ss if s['phase']=='motion'];
 jt=np.array([s['joint_stamp_s'] for s in ss]);jv=np.array([[s['joint_velocities_rad_s'][k] for k in ('left_rocker_left_wheel_joint','right_rocker_right_wheel_joint')]for s in ss]);
 # Unique odometry timestamps, yielding position finite differences in the same
 # simulator time basis as the interpolated joint observations.
 states={s['odometry_stamp_s']:s for s in ss}; states=sorted(states.items());rows=[]
 for (ta,a),(tb,b) in zip(states[:-1],states[1:]):
  dt=tb-ta
  if not 0<dt<.06:continue
  t=(ta+tb)/2; da=math.atan2(math.sin(b['base_rpy_rad'][2]-a['base_rpy_rad'][2]),math.cos(b['base_rpy_rad'][2]-a['base_rpy_rad'][2]));yaw=a['base_rpy_rad'][2]+da/2
  vx,vy=(np.array(b['position_m'][:2])-a['position_m'][:2])/dt
  forward=math.cos(yaw)*vx+math.sin(yaw)*vy;lateral=-math.sin(yaw)*vx+math.cos(yaw)*vy;omega=da/dt
  query=np.linspace(ta,tb,21);w=np.array([np.trapezoid(np.interp(query,jt,jv[:,i]),query)/dt for i in range(2)]) if hasattr(np,'trapezoid') else np.array([np.trapz(np.interp(query,jt,jv[:,i]),query)/dt for i in range(2)])
  rolling=cfg['wheel_radius_m']*w;point_velocity=np.array([forward-cfg['half_track_m']*omega,forward+cfg['half_track_m']*omega]);residual=rolling-point_velocity
  rows.append({'trajectory_time_s':(a['trajectory_time_s']+b['trajectory_time_s'])/2,'odometry_interval_s':[ta,tb],'measured_body_forward_m_s':forward,'measured_base_lateral_m_s':lateral,'measured_yaw_rad_s':omega,'wheel_inferred_yaw_rad_s':(rolling[1]-rolling[0])/(2*cfg['half_track_m']),'measured_axle_lateral_m_s':lateral-cfg['base_ahead_of_axle_m']*omega,'wheel_surface_velocity_m_s':rolling.tolist(),'rigid_body_wheel_point_velocity_m_s':point_velocity.tolist(),'wheel_surface_minus_body_point_velocity_m_s':residual.tolist()})
 corner1=report['summary']['waypoints'][1]['scheduled_arrival_s'];window=[r for r in rows if corner1+.05<=r['trajectory_time_s']<=corner1+.25];steady=[r for r in rows if corner1-1.<=r['trajectory_time_s']<=corner1-.1]
 out[case]={'first_corner_50_to_250ms':window,'peak_abs_rolling_residual_m_s_in_window':max(abs(v)for r in window for v in r['wheel_surface_minus_body_point_velocity_m_s']),'steady_before_first_corner_max_abs_residual_m_s':max(abs(v)for r in steady for v in r['wheel_surface_minus_body_point_velocity_m_s']),'limitations':'Pose differences use simulator source timestamps; wheel feedback is integrated after interpolation onto same interval. Native joint publication is sampled at controller~21ms, so interpolation near an instantaneous switch is uncertain within one controller period. Rigid nominal chassis geometry neglects small rocker/pitch motion and true contact patch. Values diagnose nonideal rolling/body response; not a direct measured contact-force or tire-slip sensor.'}
(root/'wheel_body_residual.json').write_text(json.dumps(out,indent=2)+'\n')
for case,r in out.items():print(case,r['peak_abs_rolling_residual_m_s_in_window'],r['steady_before_first_corner_max_abs_residual_m_s'])
