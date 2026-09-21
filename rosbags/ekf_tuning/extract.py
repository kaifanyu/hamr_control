#!/usr/bin/env python3
"""Extract immutable numeric inputs for repeatable EKF experiments (ROS Jazzy)."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def yaw(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def extract(bag, output):
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    t0_ns = metadata['starting_time']['nanoseconds_since_epoch']
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id=metadata['storage_identifier']),
                ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {'/wheel_odom': 'wheel', '/imu/data': 'imu', '/HAMR_base/odom': 'vicon',
              '/local_HAMR/odom': 'recorded', '/imu/calib_status': 'calib'}
    classes = {k: get_message(types[k]) for k in wanted if k in types}
    rows = {v: [] for v in wanted.values()}
    covariances, frames = {}, {}
    while reader.has_next():
        topic, data, time_ns = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(data, classes[topic])
        key = wanted[topic]
        receipt = (time_ns - t0_ns) * 1e-9
        if key == 'calib':
            rows[key].append([receipt, *bytes(msg.data)])
            continue
        stamp = (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec - t0_ns) * 1e-9
        frames[key] = [msg.header.frame_id, getattr(msg, 'child_frame_id', '')]
        if key == 'imu':
            a, w, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
            rows[key].append([stamp, receipt, yaw(q), w.z, a.x, a.y, a.z,
                              q.x, q.y, q.z, q.w, w.x, w.y])
            covariances[key] = {'orientation': list(msg.orientation_covariance),
                               'angular_velocity': list(msg.angular_velocity_covariance),
                               'linear_acceleration': list(msg.linear_acceleration_covariance)}
        else:
            p, w, q = msg.pose.pose.position, msg.twist.twist, msg.pose.pose.orientation
            rows[key].append([stamp, receipt, p.x, p.y, p.z, yaw(q),
                              w.linear.x, w.linear.y, w.angular.z, q.x, q.y, q.z, q.w])
            covariances[key] = {'pose': list(msg.pose.covariance), 'twist': list(msg.twist.covariance)}
    arrays = {k: np.asarray(v, dtype=np.float64) for k, v in rows.items()}
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output / (bag.name + '.npz'), **arrays)
    summary = {'bag': str(bag.resolve()), 'epoch_ns': t0_ns,
               'columns': {'odom': 'stamp,receipt,x,y,z,yaw,vx,vy,wz,qx,qy,qz,qw',
                           'imu': 'stamp,receipt,yaw,wz,ax,ay,az,qx,qy,qz,qw,wx,wy',
                           'calib': 'receipt,sys,gyro,accel,mag'},
               'counts': {k: len(v) for k, v in arrays.items()},
               'frames': frames, 'last_covariances': covariances,
               'sha256': {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                          for f in sorted(bag.glob('*.mcap'))}}
    (output / (bag.name + '.json')).write_text(json.dumps(summary, indent=2) + '\n')
    print(bag.name, summary['counts'], flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'data')
    args = parser.parse_args()
    for bag in args.bags:
        extract(bag, args.output)
