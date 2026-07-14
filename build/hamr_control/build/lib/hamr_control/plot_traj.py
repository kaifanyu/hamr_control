#!/usr/bin/env python3
"""Read a rosbag and save a PNG comparing actual vs reference trajectory."""

import argparse
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Waypoints from waypoint_traj_simple.py — the ground-truth reference path
WAYPOINTS = [
    [0.0, 0.0],
    [0.0, 3.0],
    [1.5, 3.0],
    [1.5, 5.0],
    [-1.5, 5.0],
    [-1.5, 3.0],
    [0.0, 3.0],
    [0.0, 0.0],
]


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    filter_ = rosbag2_py.StorageFilter(topics=['/hamr/odom'])
    reader.set_filter(filter_)

    actual_x, actual_y = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/hamr/odom':
            msg = deserialize_message(data, Odometry)
            actual_x.append(msg.pose.pose.position.x)
            actual_y.append(msg.pose.pose.position.y)

    return actual_x, actual_y


def main():
    parser = argparse.ArgumentParser(description='Plot actual vs reference trajectory from a rosbag.')
    parser.add_argument('bag', nargs='?', default='/home/ranaudo/hamr_run',
                        help='Path to the rosbag directory (default: ~/hamr_run)')
    parser.add_argument('-o', '--output', default='trajectory.png',
                        help='Output image filename (default: trajectory.png)')
    args = parser.parse_args()

    print(f"Reading bag: {args.bag}")
    actual_x, actual_y = read_bag(args.bag)
    print(f"  Actual odom points: {len(actual_x)}")

    ref_x = [p[0] for p in WAYPOINTS]
    ref_y = [p[1] for p in WAYPOINTS]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(ref_x, ref_y, '--', color='royalblue', linewidth=2, label='Reference trajectory')
    ax.plot(actual_x, actual_y, color='crimson', linewidth=1.5, label='Actual trajectory')
    ax.plot(actual_x[0], actual_y[0], 'go', markersize=10, label='Start')
    ax.plot(actual_x[-1], actual_y[-1], 'rs', markersize=10, label='End')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Actual vs Reference Trajectory')
    ax.axis('equal')
    ax.legend()
    ax.grid(True)

    fig.savefig(args.output, dpi=150, bbox_inches='tight')
    print(f"Saved: {args.output}")


if __name__ == '__main__':
    main()
