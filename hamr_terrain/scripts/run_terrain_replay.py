#!/usr/bin/env python3
"""Run one measured physical terrain trajectory and record an actual Gazebo MP4."""
import argparse
from datetime import datetime, timezone
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np

from generate_terrains import package_directory
from terrain_replay_paths import (reference_for_terrain, SUPPORTED_SHAPES, prepare_replay_world,
                                 augment_robot_cameras, replay_camera_configuration, sha256)
from terrain_simulation_check import stop_group


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def stamp(value):
    return value.sec+value.nanosec*1e-9


def outside_terrain(xyz, metadata):
    bounds = metadata.get('bounds_m', {'x':[-3.,3.], 'y':[-4.,4.]})
    return any(not bounds[axis][0] <= xyz[i] <= bounds[axis][1]
               for i, axis in enumerate(('x','y')))


def spawn_height(grid, start, radius=.65):
    """Keep dense placement; enclose cells on sparsely sampled footprints.

    An axis is sparse when a cell is at least one footprint radius wide
    (at most two such cells across the diameter). Include intersecting-cell
    vertices even when a fractional start has two samples inside its footprint.
    Dense grids retain the original sample-within-footprint placement.
    """
    x, y, z = (np.asarray(grid[key]) for key in ('x', 'y', 'z'))
    indices = [np.flatnonzero(abs(axis-coordinate) <= radius)
               for axis, coordinate in zip((x, y), start)]
    sparse = any(np.any(np.diff(axis) >= radius) for axis in (x, y))
    if sparse or any(len(index) < 2 for index in indices):
        # A native large map may have metre-scale cells. Include every vertex
        # of cells intersecting the footprint so the vehicle spawns above them.
        indices = [np.arange(max(0, np.searchsorted(axis, coordinate-radius)-1),
                             min(len(axis), np.searchsorted(axis, coordinate+radius, side='right')+1))
                   for axis, coordinate in zip((x, y), start)]
    if any(len(index) == 0 for index in indices):
        raise ValueError('Route start has no terrain support samples')
    height = float(np.max(z[np.ix_(indices[1], indices[0])])+.025)
    if not math.isfinite(height):
        raise ValueError('Route start has nonfinite terrain heights')
    return height


def support_floor(metadata):
    height = float(metadata.get('support_floor_z_m', -.25))
    if not math.isfinite(height):
        raise ValueError('support_floor_z_m must be finite')
    return height


def startup_timeout(metadata):
    seconds = float(metadata.get('startup_timeout_s', 150.))
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('startup_timeout_s must be finite and positive')
    return seconds


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--terrain', required=True)
    parser.add_argument('--caster', choices=('ball', 'swivel'), required=True)
    parser.add_argument('--shape', choices=SUPPORTED_SHAPES, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, default=121)
    parser.add_argument('--speed', type=float, default=.25)
    parser.add_argument('--wall-timeout', type=float, default=7200)
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--render-backend', choices=('egl','display','d3d12'), default='egl')
    parser.add_argument('--contact-logging', action='store_true', help='Collect every contact event; substantially slower than trajectory/joint logging')
    parser.add_argument('--closeup-camera', action='store_true', help='Record a separate body-mounted right caster view')
    args = parser.parse_args(argv)
    if not 0 <= args.domain_id <= 232 or not 1 <= args.fps <= 60:
        parser.error('Invalid domain or frame rate')
    if not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0:
        parser.error('wall-timeout must be finite and positive')
    package = package_directory()
    metadata_path = package/'config'/f'{args.terrain}.json'
    if not metadata_path.is_file():
        parser.error(f'Unknown terrain: {args.terrain}')
    metadata = json.loads(metadata_path.read_text())
    reference = reference_for_terrain(args.shape, args.speed, metadata)
    support_floor_z = support_floor(metadata)
    startup_timeout_s = startup_timeout(metadata)
    source_world = (metadata_path.parent/metadata['world']).resolve()
    grid_path = (metadata_path.parent/metadata['height_grid']).resolve()
    grid = np.load(grid_path)
    output = args.output_dir.expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        parser.error('Output must be new or empty')
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),
                      GZ_PARTITION=f'hamr_replay_{args.domain_id}_{os.getpid()}',
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    if args.render_backend == 'd3d12':
        os.environ.update(GALLIUM_DRIVER='d3d12',MESA_D3D12_DEFAULT_ADAPTER_NAME='NVIDIA')
    from ament_index_python.packages import get_package_prefix
    from ament_index_python.packages import get_package_share_directory
    ball_bin = Path(get_package_prefix('hamr_ball_caster'))/'lib/hamr_ball_caster'
    spec = importlib.util.spec_from_file_location('_waypoint', ball_bin/'run_waypoint_sim.py')
    shared = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(shared)
    source = output/'world.sdf'
    world_name = prepare_replay_world(source_world, source, grid, reference, args.fps,
                                     args.contact_logging, metadata=metadata)
    caster_package = 'hamr_ball_caster' if args.caster == 'ball' else 'hamr_swivel_caster'
    caster_share = Path(get_package_share_directory(caster_package))
    xacro_name = ('compa_ball_caster_recording.urdf.xacro' if args.caster == 'ball'
                  else 'compa_swivel_caster.urdf.xacro')
    robot_snapshot = output/'robot.urdf'
    description = subprocess.check_output([
        'xacro', str(caster_share/'urdf'/xacro_name),
        f'enable_closeup_camera:={str(args.closeup_camera).lower()}'], text=True)
    robot_snapshot.write_text(augment_robot_cameras(description, metadata=metadata, fps=args.fps))
    start = reference.points[0]
    spawn_z = spawn_height(grid, start)
    command = ['ros2','launch','hamr_terrain','terrain_sim.launch.py',f'world:={source}',
               f'caster:={args.caster}','gui:=false','visual_detail:=overview',
               f'robot_description_file:={robot_snapshot}',
               f'spawn_x:={start[0]}',f'spawn_y:={start[1]}',f'spawn_z:={spawn_z}',
               f'initial_yaw:={reference.yaw}',
               f'headless_rendering:={str(args.render_backend=="egl").lower()}']
    recorder_command = [sys.executable,str(ball_bin/'record_simulation.py'),
                        '--output',str(output/'camera.mp4'),'--topic','/hamr/terrain/overview/image',
                        '--fps',str(args.fps),'--ready-file',str(output/'camera_ready.json'),
                        '--metadata',str(output/'camera.json'),'--reliable-images',
                        '--wall-timeout',str(args.wall_timeout)]
    closeup_command = ([sys.executable,str(ball_bin/'record_simulation.py'),
                        '--output',str(output/'closeup.mp4'),
                        '--topic',f'/hamr/{"ball_caster" if args.caster=="ball" else "swivel_caster"}/closeup/image',
                        '--fps',str(args.fps),'--ready-file',str(output/'closeup_ready.json'),
                        '--metadata',str(output/'closeup.json'),'--reliable-images',
                        '--wall-timeout',str(args.wall_timeout)] if args.closeup_camera else None)
    provenance = {'created_utc':datetime.now(timezone.utc).isoformat(),
                  'terrain':args.terrain,'caster':args.caster,'route':reference.as_dict(),
                  'world_name':world_name,'source_world':str(source_world),
                  'source_world_sha256':sha256(source_world),'snapshot_sha256':sha256(source),
                  'metadata_sha256':sha256(metadata_path),'grid_sha256':sha256(grid_path),
                  'source_mesh_sha256':metadata['sha256'],
                  'robot_snapshot':str(robot_snapshot),'robot_snapshot_sha256':sha256(robot_snapshot),
                  'robot_xacro_source':str(caster_share/'urdf'/xacro_name),
                  'robot_xacro_source_sha256':sha256(caster_share/'urdf'/xacro_name),
                  'runner_sha256':sha256(__file__),
                  'paths_source_sha256':sha256(Path(__file__).with_name('terrain_replay_paths.py')),
                  'commands':{'simulation':command,'recording':recorder_command,
                              'closeup_recording':closeup_command},
                  'environment':{k:os.environ[k] for k in ('ROS_DOMAIN_ID','GZ_PARTITION','GALLIUM_DRIVER','MESA_D3D12_DEFAULT_ADAPTER_NAME') if k in os.environ},
                  'render_backend':args.render_backend,
                  'startup_timeout_s':startup_timeout_s,
                  'contact_telemetry_enabled':args.contact_logging,
                  'closeup_camera_enabled':args.closeup_camera,
                  'replay_camera':dict(replay_camera_configuration(metadata), fps=args.fps,
                                       width=1280,height=960,topic='/hamr/terrain/overview/image'),
                  'feedback_processing':'Persistent single-thread executor; sensor-data QoS for odometry and joints',
                  'spawn':{'x':float(start[0]),'y':float(start[1]),'z':spawn_z,'yaw':reference.yaw},
                  'support_floor_z_m':support_floor_z,
                  'physics':'Original terrain/caster collision, inertia, friction and 1 ms timestep. Only camera settings and collision-free path visuals differ.',
                  'controller':{'position_gain_s_inv':1.5,'nominal_speed_m_s':args.speed,
                                'max_translation_m_s':.5,'max_wheel_rate_rad_s':6.,
                                'max_wheel_acceleration_rad_s2':15.,'period_s':.02,
                                'startup_s':3.,'final_hold_s':6.,
                                'wheel_radius_m':.1075,'half_track_m':.33072,'base_ahead_of_axle_m':.27114}}
    write_json(output/'configuration.json',provenance)
    write_json(output/'terrain.json',metadata)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
    from ros_gz_interfaces.msg import Contacts
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('terrain_replay_controller')
    executor=SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    state = {'clock':None,'odom':None,'joints':None,'velocity':np.zeros(3),'odom_wall':None}
    contacts = {'enabled':args.contact_logging,'count':0,'maximum_depth_m':0.,'collision_names':set()}
    contact_samples=[]
    def odom_callback(msg):
        previous = state['odom']
        if previous is not None:
            dt = stamp(msg.header.stamp)-stamp(previous.header.stamp)
            if dt > 1e-8:
                a,b = previous.pose.pose,msg.pose.pose
                yaw_a,yaw_b = shared.quaternion_rpy(a.orientation)[2],shared.quaternion_rpy(b.orientation)[2]
                state['velocity'] = np.array([(b.position.x-a.position.x)/dt,
                    (b.position.y-a.position.y)/dt,shared.wrap(yaw_b-yaw_a)/dt])
        state['odom'],state['odom_wall'] = msg,time.monotonic()
    def contact_callback(msg):
        contacts['count'] += len(msg.contacts)
        for contact in msg.contacts:
            contacts['collision_names'].update((contact.collision1.name,contact.collision2.name))
            contacts['maximum_depth_m'] = max(contacts['maximum_depth_m'],max(contact.depths,default=0.))
        contact_time=stamp(msg.header.stamp)
        if contact_samples and contact_time-contact_samples[-1]['simulation_time_s']<.02:
            return
        vector=lambda v:[float(v.x),float(v.y),float(v.z)]
        records=[]
        for contact in msg.contacts:
            records.append({'collision1':contact.collision1.name,'collision2':contact.collision2.name,
                'positions_m':[vector(v) for v in contact.positions],
                'normals':[vector(v) for v in contact.normals], 'depths_m':list(contact.depths),
                'wrenches':[{'body1':w.body_1_name.data,'body2':w.body_2_name.data,
                    'body1_force_N':vector(w.body_1_wrench.force),'body2_force_N':vector(w.body_2_wrench.force),
                    'body1_torque_Nm':vector(w.body_1_wrench.torque),'body2_torque_Nm':vector(w.body_2_wrench.torque),
                    'frame_id':w.header.frame_id} for w in contact.wrenches]})
        contact_samples.append({'simulation_time_s':contact_time,'contacts':records})
    node.create_subscription(Clock,'/clock',lambda m:state.update(clock=stamp(m.clock)),10)
    node.create_subscription(Odometry,'/compa/odom',odom_callback,qos_profile_sensor_data)
    node.create_subscription(JointState,'/joint_states',lambda m:state.update(joints=m),qos_profile_sensor_data)
    if args.contact_logging:
        node.create_subscription(Contacts,'/hamr/terrain/contacts',contact_callback,qos_profile_sensor_data)
    pubs = [node.create_publisher(Float64,topic,10) for topic in (
        '/left_wheel/cmd_vel','/right_wheel/cmd_vel','/roll/cmd_vel','/pitch/cmd_vel','/yaw/cmd_vel')]
    def send(values):
        for pub,value in zip(pubs,values): pub.publish(Float64(data=float(value)))
    def interrupted(*_): raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,interrupted)
    signal.signal(signal.SIGTERM,interrupted)
    processes, handles, samples = {},[],[]
    physical_failure = infrastructure_failure = None
    started_wall = last_progress = time.monotonic()
    epoch = previous = last_sample = None
    prior_wheels = np.zeros(2)
    motion_start = motion_end = None
    recording_finished = False
    try:
        bridge = ['ros2','run','ros_gz_bridge','parameter_bridge',
                  '/hamr/terrain/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts']
        jobs=[('simulation',command),('recording',recorder_command)]
        if closeup_command:jobs.append(('closeup',closeup_command))
        if args.contact_logging:jobs.append(('contacts',bridge))
        for name,cmd in jobs:
            handle = (output/f'{name}.log').open('w'); handles.append(handle)
            processes[name] = subprocess.Popen(cmd,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        while True:
            executor.spin_once(timeout_sec=.002)
            wall = time.monotonic()
            if wall-started_wall > args.wall_timeout: raise TimeoutError('Wall timeout')
            if any(p.poll() is not None for p in processes.values()):
                raise RuntimeError('Simulation, recorder, or contact bridge exited early')
            if (any(state[k] is None for k in ('clock','odom','joints'))
                or not (output/'camera_ready.json').exists()
                or (closeup_command and not (output/'closeup_ready.json').exists())):
                if wall-started_wall > startup_timeout_s: raise TimeoutError('No camera/odometry/joint readiness')
                continue
            now = state['clock']
            if wall-state['odom_wall'] > 10: raise RuntimeError('Odometry stopped arriving')
            if epoch is None:
                epoch = previous = now
                motion_start = now+3.
            dt = now-previous
            if dt < .02: continue
            previous = now
            elapsed = now-motion_start
            pose = state['odom'].pose.pose
            xyz = np.array([pose.position.x,pose.position.y,pose.position.z])
            rpy = np.array(shared.quaternion_rpy(pose.orientation))
            if not np.isfinite(np.r_[xyz,rpy,state['velocity']]).all():
                physical_failure='nonfinite_physical_state'; break
            source_time = stamp(state['odom'].header.stamp)
            age = now-source_time
            if age > .5: raise RuntimeError(f'Stale odometry: {age:.3f} simulation seconds')
            if elapsed >= 0 and physical_failure is None:
                if max(abs(rpy[:2])) > 1.05:
                    physical_failure='tipped_over'; motion_end=now
                elif outside_terrain(xyz, metadata):
                    physical_failure='left_terrain_patch'; motion_end=now
                elif xyz[2] < support_floor_z:
                    physical_failure='lost_ground_support'; motion_end=now
            target,feedforward,index = reference.sample(max(0.,elapsed))
            predicted = xyz[:2]+state['velocity'][:2]*max(0.,min(age,.1))
            velocity = feedforward+1.5*(target-predicted)
            norm = np.linalg.norm(velocity)
            translation_limited = bool(norm > .5)
            if translation_limited: velocity *= .5/norm
            left,right,_ = shared.inverse_drive(*velocity,rpy[2],.1075,.33072,.27114)
            raw_wheels = np.array([left,right])
            wheel_rate_limited = bool(np.max(abs(raw_wheels)) > 6.)
            requested_wheels = raw_wheels*min(1.,6./max(np.max(abs(raw_wheels)),1e-12))
            delta_wheels = requested_wheels-prior_wheels
            acceleration_step = 15.*min(dt,.1)
            wheel_acceleration_limited = bool(np.any(abs(delta_wheels) > acceleration_step))
            wheels = prior_wheels+np.clip(delta_wheels,-acceleration_step,acceleration_step)
            joints = dict(zip(state['joints'].name,state['joints'].position))
            omega = .1075*(wheels[1]-wheels[0])/(2*.33072)
            gimbal = np.clip([-4.*(rpy[0]+joints.get('base_roll_joint',0.)),
                             -4.*(rpy[1]+joints.get('roll_pitch_joint',0.)),
                             -omega+4.*shared.wrap(-rpy[2]-joints.get('pitch_yaw_plate_joint',0.))],-2.5,2.5)
            values = [*wheels,*gimbal]
            if elapsed < 0 or physical_failure is not None: values=[0.]*5
            prior_wheels = np.array(values[:2]); send(values)
            if last_sample is None or source_time-last_sample >= .049:
                last_sample = source_time
                ref_at_pose, ref_velocity, _ = reference.sample(max(0.,source_time-motion_start))
                samples.append({'simulation_time_s':source_time,'trajectory_time_s':source_time-motion_start,
                    'phase':'settling' if elapsed<0 else 'failure_hold' if physical_failure else 'motion' if elapsed<reference.duration else 'final_hold',
                    'position_m':xyz.tolist(),'base_rpy_rad':rpy.tolist(),'reference_m':ref_at_pose.tolist(),
                    'position_error_m':float(np.linalg.norm(xyz[:2]-ref_at_pose)),
                    'estimated_velocity_m_s':state['velocity'].tolist(),'commands_rad_s':list(map(float,values)),
                    'reference_velocity_m_s':ref_velocity.tolist(),
                    'route_leg':reference.leg_at(source_time-motion_start),
                    'body_forward_speed_m_s':float(np.dot(state['velocity'][:2], [math.cos(rpy[2]),math.sin(rpy[2])])),
                    'reference_body_forward_speed_m_s':float(np.dot(ref_velocity, [math.cos(rpy[2]),math.sin(rpy[2])])),
                    'raw_wheel_rates_rad_s':raw_wheels.tolist(),
                    'rate_limited_wheel_rates_rad_s':requested_wheels.tolist(),
                    'command_limits':{'translation':translation_limited,
                                      'wheel_rate':wheel_rate_limited,
                                      'wheel_acceleration':wheel_acceleration_limited},
                    'joint_state_stamp_s':stamp(state['joints'].header.stamp),
                    'joint_positions_rad':joints,'joint_velocities_rad_s':dict(zip(state['joints'].name,state['joints'].velocity)),
                    'odometry_age_s':age,'reference_segment':index})
            if wall-last_progress >= 5:
                write_json(output/'progress.json',{'terrain':args.terrain,'caster':args.caster,'shape':args.shape,
                    'phase':samples[-1]['phase'] if samples else 'startup','simulation_time_s':now,
                    'trajectory_time_s':elapsed,'reference_duration_s':reference.duration,
                    'wall_elapsed_s':wall-started_wall,'samples':len(samples),'physical_failure':physical_failure})
                last_progress=wall
            if (physical_failure and now-motion_end>=2.) or elapsed>=reference.duration+6.:
                motion_end=now; recording_finished=True; break
        # Rendering and image transport can trail /clock. Keep the camera live
        # for a further simulated second so the final commanded hold is covered
        # by fresh source frames before asking the encoder to stop.
        if recording_finished:
            send([0.]*5)
            tail_end=state['clock']+1.
            while state['clock']<tail_end:
                if time.monotonic()-started_wall>args.wall_timeout:
                    raise TimeoutError('Camera-tail wall timeout')
                if processes['simulation'].poll() is not None:
                    raise RuntimeError('Simulation exited during camera tail')
                executor.spin_once(timeout_sec=.01)
    except (Exception,KeyboardInterrupt) as error:
        infrastructure_failure=f'{type(error).__name__}: {error}'
    finally:
        for _ in range(5):
            send([0.]*5); executor.spin_once(timeout_sec=.01)
        cleanup={}
        # Allow MP4 encoder to drain before stopping its publisher.
        for name in ('recording','closeup'):
            recorder=processes.get(name)
            if recorder is not None and recorder.poll() is None:
                os.killpg(recorder.pid,signal.SIGINT)
                try: recorder.wait(timeout=60)
                except subprocess.TimeoutExpired: pass
        for name in ('recording','closeup','contacts','simulation'):
            cleanup[name]=stop_group(processes.get(name))
        executor.shutdown()
        node.destroy_node(); rclpy.shutdown()
        for handle in handles: handle.close()
    contacts['collision_names']=sorted(contacts['collision_names'])
    if args.contact_logging:
        write_json(output/'contact_samples.json',{'motion_start_s':motion_start,
            'scope':'At most 50 Hz snapshots of received contact events; short impacts and messages may be missed. Wrenches retain backend-reported frames and sign conventions.',
            'samples':contact_samples})
        contacts['sample_file']='contact_samples.json'
        contacts['sample_count']=len(contact_samples)
    motion = [s for s in samples if s['phase']=='motion']
    errors = [s['position_error_m'] for s in motion]
    final_error = float(np.linalg.norm(np.array(samples[-1]['position_m'][:2])-reference.points[-1])) if samples else None
    camera = json.loads((output/'camera.json').read_text()) if (output/'camera.json').exists() else {}
    timestamps = camera.get('encoded_source_timestamps_s',[])
    coverage = bool(timestamps and motion_start is not None and motion_end is not None
                    and timestamps[0]<=motion_start+.15 and timestamps[-1]>=motion_end-.2)
    video_ok = bool(camera.get('success') and camera.get('frames_queue_dropped')==0
                    and camera.get('max_image_gap_s',math.inf)<=.21 and coverage)
    closeup = json.loads((output/'closeup.json').read_text()) if (output/'closeup.json').exists() else {}
    closeup_timestamps = closeup.get('encoded_source_timestamps_s',[])
    closeup_coverage = bool(closeup_timestamps and motion_start is not None and motion_end is not None
                            and closeup_timestamps[0]<=motion_start+.15
                            and closeup_timestamps[-1]>=motion_end-.2)
    closeup_ok = bool(closeup.get('success') and closeup.get('frames_queue_dropped')==0
                      and closeup.get('max_image_gap_s',math.inf)<=.21 and closeup_coverage)
    if args.closeup_camera: video_ok = video_ok and closeup_ok
    completed = bool(recording_finished and physical_failure is None)
    tracking_ok = bool(completed and final_error is not None and final_error<=.2 and errors and max(errors)<=.5)
    report = {'terrain':args.terrain,'caster':args.caster,'shape':args.shape,
              'recording_complete':video_ok,'trajectory_clock_complete':completed,'tracking_passed':tracking_ok,
              'physical_failure':physical_failure,'infrastructure_failure':infrastructure_failure,
              'motion_start_s':motion_start,'motion_end_s':motion_end,
              'wall_duration_s':time.monotonic()-started_wall,'route':reference.as_dict(),
              'measurements':{'samples':len(samples),'tracking_rmse_m':float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                  'tracking_max_error_m':max(errors) if errors else None,'final_position_error_m':final_error,
                  'max_abs_roll_pitch_deg':np.degrees(np.max(np.abs(np.array([s['base_rpy_rad'][:2] for s in samples])),axis=0)).tolist() if samples else None},
              'contacts':contacts,'video':{'path':'camera.mp4','sha256':sha256(output/'camera.mp4') if (output/'camera.mp4').exists() else None,
                  'coverage_passed':coverage,'metadata':'camera.json'},
              'closeup_video':({'path':'closeup.mp4','sha256':sha256(output/'closeup.mp4') if (output/'closeup.mp4').exists() else None,
                                'coverage_passed':closeup_coverage,'metadata':'closeup.json','valid':closeup_ok}
                               if args.closeup_camera else None),
              'cleanup':cleanup,
              'scope':'Measured Gazebo dynamics. A completed reference clock alone does not mean successful path tracking. Estimated mechanics/ideal velocity servos remain uncalibrated.'}
    write_json(output/'trajectory.json',{'configuration':provenance,'samples':samples})
    write_json(output/'run_summary.json',report)
    write_json(output/'progress.json',{'phase':'complete','recording_complete':video_ok,'tracking_passed':tracking_ok})
    print(json.dumps({k:report[k] for k in ('terrain','caster','shape','recording_complete','tracking_passed','physical_failure','infrastructure_failure','wall_duration_s')},indent=2),flush=True)
    return 0 if video_ok and infrastructure_failure is None else 1


if __name__=='__main__':
    raise SystemExit(main())
