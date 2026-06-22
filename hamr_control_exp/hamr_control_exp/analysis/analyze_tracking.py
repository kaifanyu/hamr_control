#!/usr/bin/env python3
"""Offline tracking analysis of recorded rosbags.

Compares one or more runs (e.g. PID vs LQR vs MPC on the same trajectory)
using the topics already captured by record_hamr_test_bag /
record_hamr_vicon_bag. ReferenceTraj has no header, so alignment uses bag
receive timestamps — fine at 100 Hz for these metrics.

Usage:
  python3 analyze_tracking.py BAG_DIR [BAG_DIR ...] [--plot OUT_DIR]
      [--gt-topic /HAMR_base/odom] [--ref-topic /reference_trajectory]

Metrics per bag: RMS / max cross-track error, RMS / max turret yaw
deviation, control effort (sum |omega| dt), wheel saturation %.
"""
import argparse
import math
import sys

import numpy as np


def read_bag(bag_dir, topics):
    """Returns {topic: (t_array_seconds, [deserialized msgs])}."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id=""),
                ConverterOptions(input_serialization_format="cdr",
                                 output_serialization_format="cdr"))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {t: ([], []) for t in topics if t in type_map}
    missing = [t for t in topics if t not in type_map]
    if missing:
        print(f"  note: topics not in bag: {missing}")
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic in out:
            out[topic][0].append(t_ns * 1e-9)
            out[topic][1].append(
                deserialize_message(raw, get_message(type_map[topic])))
    return {t: (np.array(ts), msgs) for t, (ts, msgs) in out.items()}


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def interp_xy(t_query, t, x, y):
    return np.interp(t_query, t, x), np.interp(t_query, t, y)


def analyze(bag_dir, gt_topic, ref_topic, turret_topic, cmd_limit):
    cmd_topics = ["/left_wheel/cmd_vel", "/right_wheel/cmd_vel",
                  "/turret/cmd_vel"]
    data = read_bag(bag_dir, [gt_topic, ref_topic, turret_topic] + cmd_topics)
    if ref_topic not in data or gt_topic not in data:
        print(f"  SKIP: missing {ref_topic} or {gt_topic}")
        return None

    t_ref, refs = data[ref_topic]
    rx = np.array([m.x for m in refs])
    ry = np.array([m.y for m in refs])
    rvx = np.array([m.x_dot for m in refs])
    rvy = np.array([m.y_dot for m in refs])
    ryaw = np.array([m.yaw for m in refs])

    t_gt, gts = data[gt_topic]
    gx = np.array([m.pose.pose.position.x for m in gts])
    gy = np.array([m.pose.pose.position.y for m in gts])

    # Evaluate at reference times, restricted to the overlapping window
    mask = (t_ref >= t_gt[0]) & (t_ref <= t_gt[-1])
    if mask.sum() < 10:
        print("  SKIP: <10 overlapping samples")
        return None
    t_eval = t_ref[mask]
    ax, ay = interp_xy(t_eval, t_gt, gx, gy)
    ex, ey = rx[mask] - ax, ry[mask] - ay

    speed = np.hypot(rvx[mask], rvy[mask])
    moving = speed > 0.02
    tx = np.where(moving, rvx[mask] / np.maximum(speed, 1e-9), 0.0)
    ty = np.where(moving, rvy[mask] / np.maximum(speed, 1e-9), 0.0)
    cross = np.where(moving, -ex * ty + ey * tx, np.hypot(ex, ey))

    result = {
        "bag": bag_dir,
        "duration_s": float(t_eval[-1] - t_eval[0]),
        "rms_cross_track_m": float(np.sqrt(np.mean(cross ** 2))),
        "max_cross_track_m": float(np.max(np.abs(cross))),
        "rms_err_norm_m": float(np.sqrt(np.mean(ex ** 2 + ey ** 2))),
    }

    if turret_topic in data and len(data[turret_topic][0]) > 1:
        t_tu, tus = data[turret_topic]
        tyaw = np.unwrap([quat_to_yaw(m.pose.pose.orientation) for m in tus])
        m2 = (t_eval >= t_tu[0]) & (t_eval <= t_tu[-1])
        if m2.sum() > 10:
            yaw_act = np.interp(t_eval[m2], t_tu, tyaw)
            dev = np.array([wrap_angle(a) for a in (ryaw[mask][m2] - yaw_act)])
            result["rms_yaw_dev_deg"] = float(np.degrees(np.sqrt(np.mean(dev ** 2))))
            result["max_yaw_dev_deg"] = float(np.degrees(np.max(np.abs(dev))))

    effort = 0.0
    sat_count, n_cmd = 0, 0
    for ct in cmd_topics[:2]:  # wheels only for saturation
        if ct not in data or len(data[ct][0]) < 2:
            continue
        t_c, cs = data[ct]
        w = np.array([m.data for m in cs])
        effort += float(np.sum(np.abs(w[:-1]) * np.diff(t_c)))
        sat_count += int(np.sum(np.abs(w) >= cmd_limit * 0.98))
        n_cmd += len(w)
    if n_cmd:
        result["wheel_effort_rad"] = effort
        result["wheel_saturation_pct"] = 100.0 * sat_count / n_cmd

    return {**result, "_traces": (t_eval, cross)}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bags", nargs="+", help="rosbag2 directories")
    ap.add_argument("--gt-topic", default="/HAMR_base/odom",
                    help="ground-truth odometry (Vicon on HW maze runs; "
                         "use /local_HAMR/odom or /hamr/odom otherwise)")
    ap.add_argument("--ref-topic", default="/reference_trajectory")
    ap.add_argument("--turret-topic", default="/HAMR_turret/odom")
    ap.add_argument("--cmd-limit", type=float, default=3.28,
                    help="wheel rad/s considered saturated (xy_limit/r_wheel)")
    ap.add_argument("--plot", metavar="OUT_DIR", default=None,
                    help="write cross-track PNG plots to this directory")
    args = ap.parse_args()

    results = []
    for bag in args.bags:
        print(f"\n=== {bag}")
        r = analyze(bag, args.gt_topic, args.ref_topic,
                    args.turret_topic, args.cmd_limit)
        if r:
            results.append(r)
            for k, v in r.items():
                if not k.startswith("_"):
                    print(f"  {k:24s} {v:.4f}" if isinstance(v, float)
                          else f"  {k:24s} {v}")

    if len(results) > 1:
        print("\n=== comparison (RMS cross-track, m)")
        for r in sorted(results, key=lambda r: r["rms_cross_track_m"]):
            print(f"  {r['rms_cross_track_m']:.4f}  {r['bag']}")

    if args.plot and results:
        import os
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        os.makedirs(args.plot, exist_ok=True)
        fig, ax = plt.subplots(figsize=(10, 4))
        for r in results:
            t, ct = r["_traces"]
            ax.plot(t - t[0], ct, label=os.path.basename(r["bag"]))
        ax.set_xlabel("t [s]")
        ax.set_ylabel("cross-track [m]")
        ax.legend()
        ax.grid(True)
        out = os.path.join(args.plot, "cross_track.png")
        fig.savefig(out, dpi=120, bbox_inches="tight")
        print(f"\nplot saved: {out}")

    return 0 if results else 1


if __name__ == "__main__":
    sys.exit(main())
