#!/usr/bin/env python3
"""Compare VSLAM-estimated trajectory against simulator ground truth.

Reads a rosbag containing /hamr/odom (ground truth from Gazebo) and
/rtabmap/odom (visual odometry estimate), time-aligns them, and produces:
  - a plot with both paths overlaid + position error over time
  - drift statistics (mean / RMSE / max error, error per meter traveled)

Usage:
    compare_vslam_traj.py <bag_dir> -o comparison.png
"""
import argparse
import math

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

GT_TOPIC = '/hamr/odom'
VO_TOPIC = '/rtabmap/odom'


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[GT_TOPIC, VO_TOPIC]))

    gt, vo = [], []  # (t, x, y)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, Odometry)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        # rgbd_odometry publishes null (all-zero, huge covariance) poses
        # while tracking is lost - skip those
        if topic == VO_TOPIC and msg.pose.covariance[0] > 9000.0:
            continue
        (gt if topic == GT_TOPIC else vo).append((t, p.x, p.y))
    return np.array(gt), np.array(vo)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag')
    p.add_argument('-o', '--output', default='vslam_comparison.png')
    args = p.parse_args()

    gt, vo = read_bag(args.bag)
    print(f"ground truth points: {len(gt)}, visual odometry points: {len(vo)}")
    if len(gt) < 10 or len(vo) < 10:
        raise SystemExit("not enough data in bag")

    # interpolate ground truth at VO timestamps
    gx = np.interp(vo[:, 0], gt[:, 0], gt[:, 1])
    gy = np.interp(vo[:, 0], gt[:, 0], gt[:, 2])
    err = np.hypot(vo[:, 1] - gx, vo[:, 2] - gy)
    t0 = vo[0, 0]

    dist = np.sum(np.hypot(np.diff(gt[:, 1]), np.diff(gt[:, 2])))
    print(f"traveled (ground truth): {dist:.1f} m")
    print(f"error: mean={err.mean():.3f} m  rmse={np.sqrt((err**2).mean()):.3f} m  "
          f"max={err.max():.3f} m  final={err[-1]:.3f} m")
    if dist > 0:
        print(f"drift: {100.0 * err[-1] / dist:.1f}% of distance traveled")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 7))
    ax1.plot(gt[:, 1], gt[:, 2], 'b-', label='ground truth (sim)')
    ax1.plot(vo[:, 1], vo[:, 2], 'r-', label='visual odometry', alpha=0.8)
    ax1.plot(gt[0, 1], gt[0, 2], 'go', markersize=10, label='start')
    ax1.set_aspect('equal'); ax1.grid(True); ax1.legend()
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)')
    ax1.set_title('Trajectory: VSLAM estimate vs ground truth')

    ax2.plot(vo[:, 0] - t0, err, 'r-')
    ax2.grid(True)
    ax2.set_xlabel('time (s)'); ax2.set_ylabel('position error (m)')
    ax2.set_title(f'VSLAM position error  (rmse {np.sqrt((err**2).mean()):.2f} m, '
                  f'final {err[-1]:.2f} m over {dist:.0f} m)')

    fig.tight_layout()
    fig.savefig(args.output, dpi=110)
    print(f"saved {args.output}")


if __name__ == '__main__':
    main()
