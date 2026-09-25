#!/usr/bin/env python3
"""Calculate reproducible geometric and feedback metrics for Gazebo ablations."""
from pathlib import Path
import json, math
import numpy as np

ROOT=Path(__file__).resolve().parent
DRIVE=('left_rocker_left_wheel_joint','right_rocker_right_wheel_joint')

def analyze(path):
    report=json.loads(path.read_text());cfg=report['configuration'];s=report['samples'];N=len(s)
    T=np.array([x['trajectory_time_s'] for x in s]); P=np.array([x['position_m'][:2] for x in s]); E=np.array([x['position_error_m'] for x in s]);R=np.array([x['reference_m'] for x in s]);
    motion=np.array([x['phase']=='motion' for x in s]); cmd=np.array([x['commands_rad_s'][:2] for x in s]);raw=np.array([x['raw_wheels_rad_s'] for x in s]); actual=np.array([[x['joint_velocities_rad_s'].get(n,float('nan')) for n in DRIVE] for x in s]);
    pts=np.array(cfg['points_m']);delta=np.diff(pts,axis=0);L=np.linalg.norm(delta,axis=1);U=delta/L[:,None];starts=np.array([w['scheduled_arrival_s'] for w in report['summary']['waypoints']])
    distances=[]
    for a,b in zip(pts[:-1],pts[1:]):
        v=b-a; f=np.clip(((P-a)@v)/(v@v),0,1); distances.append(np.linalg.norm(P-(a+f[:,None]*v),axis=1))
    cross=np.min(distances,axis=0)
    corners=[]
    for i in range(1,len(pts)-1):
        tc=starts[i];ids=np.flatnonzero(motion&(T>=tc)&(T<=tc+2.0));pe=P[ids]-R[ids];corner_distance=P[ids]-pts[i]
        corner={'index':i,'scheduled_s':float(tc),'direction_change_deg':float(np.degrees(np.arccos(np.clip(U[i-1]@U[i],-1,1)))),
                'peak_time_tracking_error_m':float(max(E[ids])),
                'peak_nearest_polyline_distance_m':float(max(cross[ids])),
                'peak_incoming_direction_overshoot_m':float(max(corner_distance@U[i-1])),
                'peak_outgoing_direction_lag_m':float(max(-(pe@U[i]))),
                'peak_rate_limiter_discrepancy_rad_s':float(np.max(np.abs(raw[ids]-cmd[ids]))),
                'peak_actual_minus_previous_command_rad_s':float(np.max(np.abs(actual[ids]-cmd[np.maximum(ids-1,0)])))}
        if len(ids):
            peak=ids[np.argmax(E[ids])];corner['peak_error_time_after_corner_s']=float(T[peak]-tc)
        if corner['direction_change_deg'] < 1e-6:
            corner['peak_incoming_direction_overshoot_m'] = None
        corners.append(corner)
    dt=np.array([x['controller_dt_s'] for x in s]); odom_age=np.array([x['simulation_time_s']-x['odometry_stamp_s'] for x in s]);joint_age=np.array([x['simulation_time_s']-x['joint_stamp_s'] for x in s]);
    # Compare a 100ms finite difference of measured base position to mean
    # rolling-kinematic velocity reconstructed from measured wheel velocities.
    yaw=np.array([x['base_rpy_rad'][2] for x in s]);radius=cfg['wheel_radius_m'];halftrack=cfg['half_track_m'];offset=cfg['base_ahead_of_axle_m'];
    fwd=radius*np.mean(actual,axis=1);omega=radius*(actual[:,1]-actual[:,0])/(2*halftrack)
    pred=np.stack((np.cos(yaw)*fwd-np.sin(yaw)*offset*omega,np.sin(yaw)*fwd+np.cos(yaw)*offset*omega),axis=1)
    stamp=np.array([x['odometry_stamp_s'] for x in s]);residual=[]
    for j in np.flatnonzero(motion):
        k=max(0,np.searchsorted(stamp,stamp[j]-.1,side='right')-1)
        if stamp[j]-stamp[k]>.08:
            measured=(P[j]-P[k])/(stamp[j]-stamp[k]);predicted=np.mean(pred[k:j+1],axis=0);residual.append(float(np.linalg.norm(measured-predicted)))
    summary={'case':path.stem,'speed_m_s':report['speed_m_s'],'max_wheel_acceleration_rad_s2':cfg['max_wheel_acceleration_rad_s2'],
             'passed':report['summary']['passed'],'wall_duration_s':report['wall_duration_s'],'sample_count':N,
             'reference_duration_s':report['summary']['reference_duration_s'],
             'maximum_wheel_command_acceleration_rad_s2':float(np.max(np.abs(np.diff(cmd,axis=0)/np.diff(np.array([x['simulation_time_s']for x in s]))[:,None]))),
             'maximum_raw_minus_published_wheel_rate_rad_s':float(np.max(np.abs(raw-cmd))),
             'maximum_published_wheel_rate_rad_s':float(np.max(np.abs(cmd))),
             'motion_rmse_m':float(np.sqrt(np.mean(E[motion]**2))),'motion_max_error_m':float(max(E[motion])),
             'max_route_cross_track_m':float(max(cross[motion])),
             'max_requested_wheel_rate_rad_s':float(np.max(np.abs(raw[motion]))),
             'wheel_feedback_minus_previous_command_rmse_rad_s':float(np.sqrt(np.mean((actual[1:]-cmd[:-1])**2))),
             'wheel_feedback_minus_current_command_rmse_rad_s':float(np.sqrt(np.mean((actual-cmd)**2))),
             'controller_dt_s':{'median':float(np.median(dt)),'min':float(min(dt)),'max':float(max(dt)),'p99':float(np.percentile(dt,99))},
             'odometry_age_s':{'median':float(np.median(odom_age)),'max':float(max(odom_age))},
             'joint_state_age_s':{'median':float(np.median(joint_age)),'max':float(max(joint_age))},
             'measured_velocity_vs_feedback_rolling_model_residual_m_s':{'median':float(np.median(residual)),'p95':float(np.percentile(residual,95)),'max':float(max(residual))},
             'corners':corners,
             'limitations':'Global route cross-track uses nearest geometric segment; timed tracking includes along-path lag. Wheel feedback is asynchronous with odometry/commands; rolling-model residual is diagnostic, not a calibrated slip measurement.'}
    return summary

def main():
    results={}
    for name in ('baseline','no_slew','half_speed','quintic_stop','no_slew_half_step'):
        path=ROOT/f'{name}.json'
        if path.exists():results[name]=analyze(path)
    (ROOT/'metrics.json').write_text(json.dumps(results,indent=2)+'\n')
    for name,r in results.items():
        print(name, 'RMS mm',r['motion_rmse_m']*1000,'max mm',r['motion_max_error_m']*1000,'cross mm',r['max_route_cross_track_m']*1000)
        print('corners',[(c['index'],round(c['peak_time_tracking_error_m']*1000,2),(None if c['peak_incoming_direction_overshoot_m'] is None else round(c['peak_incoming_direction_overshoot_m']*1000,2)),round(c['peak_outgoing_direction_lag_m']*1000,2))for c in r['corners']])
if __name__=='__main__':main()
