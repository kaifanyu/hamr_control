#!/usr/bin/env python3
"""Read recorded MCAP files; do not create ROS nodes or publish commands."""
from pathlib import Path
import json
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
TOPICS = ['/reference_trajectory', '/HAMR_base/odom', '/left_wheel/cmd_vel',
          '/right_wheel/cmd_vel', '/wheel_control/status', '/live_gains',
          '/turret/cmd_vel']

def load(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in TOPICS}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    data = {t: [] for t in types}
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        msg = deserialize_message(raw, types[topic])
        if topic == '/reference_trajectory':
            row = [msg.x, msg.y, msg.x_dot, msg.y_dot, msg.yaw]
        elif topic == '/HAMR_base/odom':
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            row = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                   msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                   msg.twist.twist.angular.z, msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                   msg.pose.pose.position.z]
        elif topic == '/wheel_control/status':
            if len(msg.data) != 17:
                raise ValueError('Unexpected wheel status schema')
            row = list(msg.data)
        elif topic == '/live_gains':
            row = [getattr(msg, k) for k in ['p_x','d_x','i_x','p_y','d_y','i_y','p_yaw','d_yaw','i_yaw']]
        else:
            row = [msg.data]
        data[topic].append([stamp*1e-9, *row])
    return {k: np.array(v) for k,v in data.items()}

def interp(rows, time, columns):
    return np.column_stack([np.interp(time, rows[:,0], rows[:,c]) for c in columns])

def summarize(bag):
    d = load(bag)
    ref = d['/reference_trajectory']; base=d['/HAMR_base/odom']; status=d['/wheel_control/status']
    gains=d['/live_gains']; lc=d['/left_wheel/cmd_vel']; rc=d['/right_wheel/cmd_vel']
    start=ref[0,0]
    corners=np.flatnonzero(np.linalg.norm(np.diff(ref[:,3:5],axis=0),axis=1)>0.01)+1
    corners=[i for i in corners if np.linalg.norm(ref[i,3:5])>0.02 and np.linalg.norm(ref[i-1,3:5])>0.02]
    events=[]
    for i in corners:
        t=ref[i,0]; before=ref[i-1,3:5]; after=ref[i,3:5]
        # The new segment has advanced by at most one publisher period.
        corner=ref[i,1:3].copy()
        before_hat=before/np.linalg.norm(before); after_hat=after/np.linalg.norm(after)
        b=base[(base[:,0]>=t)&(base[:,0]<t+2.0)]
        st=status[(status[:,0]>=t-0.25)&(status[:,0]<t+1.0)]
        pre=status[(status[:,0]>=t-0.5)&(status[:,0]<t-0.1)]
        err=b[:,1:3]-interp(ref,b[:,0],[1,2]); old_projection=(b[:,1:3]-corner)@before_hat
        active=np.linalg.norm(interp(ref,st[:,0],[3,4]),axis=1)>0.02
        st=st[active]
        gain_window=gains[(gains[:,0]>=t)&(gains[:,0]<t+1)]
        extended=base[(base[:,0]>=t)&(base[:,0]<min(t+6.0,ref[-1,0]))]
        extended_err=extended[:,1:3]-interp(ref,extended[:,0],[1,2])
        ext_status=status[(status[:,0]>=t)&(status[:,0]<t+4)]
        old_velocity=b[:,4:6]@before_hat
        new_velocity=b[:,4:6]@after_hat
        below=np.flatnonzero(old_velocity<=.1*np.linalg.norm(before))
        above=np.flatnonzero(new_velocity>=.9*np.linalg.norm(after))
        cap=np.any(np.abs(ext_status[:,[1,9]])>=27.99,axis=1)
        cap_wheel=np.abs(ext_status[:,[1,9]])>=27.99
        corner_at_switch=interp(base,[t],[1,2])[0]
        event={'time_from_reference_start_s':float(t-start), 'old_velocity_m_s':before.tolist(),
               'new_velocity_m_s':after.tolist(), 'corner_xy_m':corner.tolist(),
               'position_error_at_switch_m':(interp(base,[t],[1,2])[0]-corner).tolist(),
               'maximum_past_corner_in_old_direction_m':float(max(0,np.max(old_projection))),
               'additional_motion_in_old_direction_after_switch_m':float(max(0,np.max((b[:,1:3]-corner_at_switch)@before_hat))),
               'maximum_xy_tracking_error_next_2s_m':float(np.max(np.linalg.norm(err,axis=1))),
               'maximum_xy_tracking_error_next_6s_m':float(np.max(np.linalg.norm(extended_err,axis=1))),
               'peak_error_in_6s_after_s':float(extended[np.argmax(np.linalg.norm(extended_err,axis=1)),0]-t),
               'peak_error_after_s':float(b[np.argmax(np.linalg.norm(err,axis=1)),0]-t),
               'first_old_velocity_below_10pct_after_s':float(b[below[0],0]-t) if len(below) else None,
               'first_new_velocity_above_90pct_after_s':float(b[above[0],0]-t) if len(above) else None,
               'firmware_target_cap_fraction_next_4s':float(np.mean(cap)),
               'capped_wheel_measured_abs_median_rpm_next_4s':float(np.median(np.abs(ext_status[:,[2,10]][cap_wheel]))) if np.any(cap_wheel) else None,
               'maximum_abs_xy_D_term_next_1s_m_s':float(np.max(np.abs(gain_window[:,[2,5]]))) if len(gain_window) else None,
               'firmware_any_saturation_fraction_minus_025_plus_1s':float(np.mean((st[:,8]>0.5)|(st[:,16]>0.5))),
               'firmware_target_peak_rpm':float(np.max(np.abs(st[:,[1,9]]))),
               'firmware_measured_minus_target_rmse_rpm':float(np.sqrt(np.mean((st[:,[2,10]]-st[:,[1,9]])**2))),
               'firmware_target_measured_pre_mean_rpm':np.mean(pre[:,[1,2,9,10]],axis=0).tolist(),
               'firmware_pid_abs_peak_pwm':float(np.max(np.abs(st[:,[4,12]]))),
               'firmware_ff_abs_peak_pwm':float(np.max(np.abs(st[:,[3,11]]))),
               'firmware_output_abs_peak_pwm':float(np.max(np.abs(st[:,[5,13]])))}
        events.append(event)
    active=np.linalg.norm(interp(ref,status[:,0],[3,4]),axis=1)>.02
    active&=(status[:,0]>=ref[0,0])&(status[:,0]<=ref[-1,0])
    st=status[active]
    rates={k:{'n':len(v),'median_interval_s':float(np.median(np.diff(v[:,0]))),
               'p99_interval_s':float(np.quantile(np.diff(v[:,0]),.99)),
               'max_interval_s':float(np.max(np.diff(v[:,0])))} for k,v in d.items() if len(v)>1}
    result={'bag':bag.name,'reference_duration_s':float(ref[-1,0]-start),
            'reference_speed_m_s':sorted(set(np.round(np.linalg.norm(ref[:,3:5],axis=1),6))),
            'rates_from_bag_receipt':rates,
            'active_firmware_status':{'count':len(st),'target_peak_rpm':float(np.max(np.abs(st[:,[1,9]]))),
              'target_cap_fraction':float(np.mean(np.any(np.abs(st[:,[1,9]])>=27.99,axis=1))),
              'saturated_fraction':float(np.mean((st[:,8]>.5)|(st[:,16]>.5))),
              'mean_target_minus_measured_rpm':np.mean(st[:,[1,9]]-st[:,[2,10]],axis=0).tolist(),
              'pid_absolute_max_pwm':float(np.max(np.abs(st[:,[4,12]]))),
              'ff_absolute_max_pwm':float(np.max(np.abs(st[:,[3,11]]))),
              'output_absolute_max_pwm':float(np.max(np.abs(st[:,[5,13]]))),
              'source_counts':dict(zip(*[[str(x) for x in np.unique(st[:,17])], [int(x) for x in np.unique(st[:,17],return_counts=True)[1]]]))},
            'live_gain_term_absolute_max':np.max(np.abs(gains[:,1:]),axis=0).tolist(),
            'turret_command_abs_max_rad_s':float(np.max(np.abs(d['/turret/cmd_vel'][:,1]))),
            'corner_events':events}
    fig,axes=plt.subplots(3,2,figsize=(14,10))
    axes[0,0].plot(ref[:,1],ref[:,2],'--',label='reference');axes[0,0].plot(base[:,1],base[:,2],label='Vicon');axes[0,0].axis('equal');axes[0,0].legend();axes[0,0].set_title('World path (m)')
    for side,cmd,ci,mi,si in [('left',lc,1,2,8),('right',rc,9,10,16)]:
        axes[0,1].plot(status[:,0]-start,status[:,ci],label=f'{side} target',lw=.8)
        axes[0,1].plot(status[:,0]-start,status[:,mi],label=f'{side} measured',lw=.7,alpha=.65)
    axes[0,1].legend();axes[0,1].set_title('Firmware wheel speed (RPM)')
    for ax,i in zip(axes[1],[0,1]):
        ax.plot(ref[:,0]-start,ref[:,3+i],label='reference');ax.plot(base[:,0]-start,base[:,4+i],label='Vicon world twist',lw=.7);ax.set_title(f'{"xy"[i]} velocity (m/s)');ax.legend()
    for axis,column in [('x',2),('y',5)]:axes[2,0].plot(gains[:,0]-start,gains[:,column],label=f'D_{axis}',lw=.7)
    axes[2,0].legend();axes[2,0].set_title('Recorded XY derivative correction (m/s)')
    axes[2,1].plot(status[:,0]-start,status[:,8],label='left saturated');axes[2,1].plot(status[:,0]-start,status[:,16],label='right saturated');axes[2,1].legend();axes[2,1].set_title('Firmware saturation flags')
    for ax in axes.flat:
        ax.grid(alpha=.25)
    for ax in [axes[0,1],*axes[1],*axes[2]]:
        ax.set_xlabel('seconds from first reference');ax.set_xlim(-1,ref[-1,0]-start+1)
        for e in events: ax.axvline(e['time_from_reference_start_s'],color='k',alpha=.12)
    fig.suptitle(bag.name+' — recorded hardware; historical controller/firmware not proven identical to current files')
    fig.tight_layout();fig.savefig(OUT/(bag.name+'.png'),dpi=150);plt.close(fig)
    np.savez_compressed(OUT/(bag.name+'.npz'),**{k.strip('/').replace('/','_'):v for k,v in d.items()})
    return result

if __name__=='__main__':
    results=[summarize(p) for p in sorted((ROOT/'rosbags').glob('hamr_hw_*')) if (p/'metadata.yaml').exists()]
    (OUT/'hardware_corner_metrics.json').write_text(json.dumps(results,indent=2)+'\n')
    for r in results:
        print(r['bag'],r['active_firmware_status'])
        for c in r['corner_events']:print(c)
