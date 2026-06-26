#!/usr/bin/env python3
"""Plot localization, sensors, actuators, and camera samples from a ROS 2 bag.

Usage:
  python3 analyze_rosbag.py BAG_DIR [--output OUTPUT_DIR]

Cross-topic comparisons use bag receive time. Vicon XY is rigidly aligned to
the local odometry frame over the first ten seconds of detected motion; scale
is deliberately not fitted, so wheel calibration error remains visible.
"""

import argparse
import csv
import math
import os
import sys
from collections import Counter, defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


def rpy(q):
    sr, cr = 2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)
    sp = 2*(q.w*q.y-q.z*q.x)
    sy, cy = 2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)
    return (math.atan2(sr, cr), math.copysign(math.pi/2, sp) if abs(sp) >= 1 else math.asin(sp),
            math.atan2(sy, cy))


def as_arrays(values):
    return {name: np.asarray(items) for name, items in values.items()}


def metadata_for(bag):
    with open(os.path.join(bag, "metadata.yaml"), encoding="utf-8") as stream:
        return yaml.safe_load(stream)["rosbag2_bagfile_information"]


def read_bag(bag, metadata):
    """Extract numeric data; retain only early/middle/late camera frames."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    start_ns = metadata["starting_time"]["nanoseconds_since_epoch"]
    duration = metadata["duration"]["nanoseconds"] * 1e-9
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=metadata["storage_identifier"]),
                ConverterOptions(input_serialization_format="cdr",
                                 output_serialization_format="cdr"))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    times, odom, imus, scalars = defaultdict(list), defaultdict(lambda: defaultdict(list)), \
        defaultdict(lambda: defaultdict(list)), defaultdict(lambda: defaultdict(list))
    delays, tf_pairs = defaultdict(list), defaultdict(Counter)
    images, image_dist = defaultdict(lambda: [None]*3), defaultdict(lambda: [float("inf")]*3)
    targets = np.array([.05, .5, .95]) * duration
    supported = {"nav_msgs/msg/Odometry", "sensor_msgs/msg/Imu", "sensor_msgs/msg/Image",
                 "std_msgs/msg/Float64", "std_msgs/msg/Int32", "tf2_msgs/msg/TFMessage"}

    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        time = (stamp_ns-start_ns)*1e-9
        times[topic].append(time)
        msg_type = types[topic]
        if msg_type not in supported:
            continue
        msg = deserialize_message(raw, get_message(msg_type))
        if msg_type == "nav_msgs/msg/Odometry":
            key = f"{topic} [{msg.header.frame_id}->{msg.child_frame_id}]"
            roll, pitch, yaw = rpy(msg.pose.pose.orientation)
            row = {"t": time, "x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y,
                   "z": msg.pose.pose.position.z, "roll": roll, "pitch": pitch, "yaw": yaw,
                   "vx": msg.twist.twist.linear.x, "vy": msg.twist.twist.linear.y,
                   "vz": msg.twist.twist.linear.z, "wx": msg.twist.twist.angular.x,
                   "wy": msg.twist.twist.angular.y, "wz": msg.twist.twist.angular.z}
            for name, value in row.items():
                odom[key][name].append(value)
            header = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            delays[topic].append(stamp_ns*1e-9-header)
        elif msg_type == "sensor_msgs/msg/Imu":
            roll, pitch, yaw = rpy(msg.orientation)
            row = {"t": time, "ax": msg.linear_acceleration.x, "ay": msg.linear_acceleration.y,
                   "az": msg.linear_acceleration.z, "gx": msg.angular_velocity.x,
                   "gy": msg.angular_velocity.y, "gz": msg.angular_velocity.z,
                   "roll": roll, "pitch": pitch, "yaw": yaw}
            for name, value in row.items():
                imus[topic][name].append(value)
            header = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            delays[topic].append(stamp_ns*1e-9-header)
        elif msg_type in {"std_msgs/msg/Float64", "std_msgs/msg/Int32"}:
            scalars[topic]["t"].append(time)
            scalars[topic]["value"].append(msg.data)
        elif msg_type == "tf2_msgs/msg/TFMessage":
            for transform in msg.transforms:
                tf_pairs[topic][(transform.header.frame_id, transform.child_frame_id)] += 1
        else:
            for index, distance in enumerate(abs(targets-time)):
                if distance < image_dist[topic][index]:
                    image_dist[topic][index], images[topic][index] = distance, msg
    return {"types": types, "times": {k: np.asarray(v) for k, v in times.items()},
            "odom": {k: as_arrays(v) for k, v in odom.items()},
            "imus": {k: as_arrays(v) for k, v in imus.items()},
            "scalars": {k: as_arrays(v) for k, v in scalars.items()},
            "delays": {k: np.asarray(v) for k, v in delays.items()},
            "tf": tf_pairs, "images": images}


def find_odom(data, topic, frame=None):
    for key, value in data["odom"].items():
        if key.startswith(topic+" [") and (frame is None or f"[{frame}->" in key):
            return key, value
    return None, None


def sample(series, field, query, unwrap=False):
    values = np.unwrap(series[field]) if unwrap else series[field]
    return np.interp(query, series["t"], values)


def rigid_fit(source, target):
    sc, tc = source.mean(0), target.mean(0)
    u, _, vt = np.linalg.svd((source-sc).T @ (target-tc))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    return rotation, tc-sc@rotation.T


def path_length(xy):
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())


def compare_localization(data):
    _, gt = find_odom(data, "/HAMR_base/odom", "mocap")
    _, local = find_odom(data, "/local_HAMR/odom")
    _, wheel = find_odom(data, "/wheel_odom")
    if gt is None or local is None:
        return None
    start = max(gt["t"][0], local["t"][0], wheel["t"][0] if wheel is not None else 0)
    end = min(gt["t"][-1], local["t"][-1], wheel["t"][-1] if wheel is not None else 1e99)
    time = np.arange(start, end, .02)
    gt_xy = np.c_[sample(gt, "x", time), sample(gt, "y", time)]
    local_xy = np.c_[sample(local, "x", time), sample(local, "y", time)]
    wheel_xy = None if wheel is None else np.c_[sample(wheel, "x", time), sample(wheel, "y", time)]
    baseline = np.median(gt_xy[time <= start+2], axis=0)
    moved = np.flatnonzero(np.linalg.norm(gt_xy-baseline, axis=1) > .05)
    motion_index = int(moved[0]) if len(moved) else 0
    motion_time = time[motion_index]
    fit = (time >= motion_time) & (time <= motion_time+10)
    if fit.sum() < 3 or np.ptp(gt_xy[fit], axis=0).max() < .02:
        fit[:] = True
    rotation, translation = rigid_fit(gt_xy[fit], local_xy[fit])
    gt_aligned = gt_xy@rotation.T+translation
    moving = time >= motion_time
    gt_yaw = sample(gt, "yaw", time, True)
    local_yaw = sample(local, "yaw", time, True)
    wheel_yaw = None if wheel is None else sample(wheel, "yaw", time, True)
    gt_delta, local_delta = gt_yaw-gt_yaw[motion_index], local_yaw-local_yaw[motion_index]
    wheel_delta = None if wheel_yaw is None else wheel_yaw-wheel_yaw[motion_index]

    def metrics(xy, yaw_delta):
        error = np.linalg.norm(xy-gt_aligned, axis=1)
        yaw_error = np.angle(np.exp(1j*(yaw_delta-gt_delta)))
        active, active_yaw = error[moving], yaw_error[moving]
        return {"error": error, "yaw_error": yaw_error,
                "rmse": float(np.sqrt(np.mean(active**2))), "p95": float(np.percentile(active, 95)),
                "max": float(active.max()), "end": float(active[-1]),
                "yaw_rmse": float(np.degrees(np.sqrt(np.mean(active_yaw**2)))),
                "path": path_length(xy[moving])}
    result = {"t": time, "gt": gt_aligned, "local": local_xy, "wheel": wheel_xy,
              "gt_yaw": gt_delta, "local_yaw": local_delta, "wheel_yaw": wheel_delta,
              "motion": motion_time, "angle": math.degrees(math.atan2(rotation[1, 0], rotation[0, 0])),
              "gt_path": path_length(gt_aligned[moving]), "local_m": metrics(local_xy, local_delta)}
    if wheel_xy is not None:
        result["wheel_m"] = metrics(wheel_xy, wheel_delta)
        result["local_wheel_rms"] = float(np.sqrt(np.mean(np.sum((local_xy[moving]-wheel_xy[moving])**2, axis=1))))
    return result


def save_localization(result, out):
    if result is None:
        return
    t = result["t"]
    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    axes[0, 0].plot(*result["gt"].T, label="Vicon (aligned)", lw=2)
    axes[0, 0].plot(*result["local"].T, label="local_HAMR")
    if result["wheel"] is not None:
        axes[0, 0].plot(*result["wheel"].T, "--", label="wheel_odom")
    axes[0, 0].axis("equal"); axes[0, 0].set(xlabel="x [m]", ylabel="y [m]", title="XY trajectory")
    for column, field in [(1, 0), (2, 1)]:
        axes[0, column].plot(t, result["gt"][:, field], label="Vicon (aligned)")
        axes[0, column].plot(t, result["local"][:, field], label="local_HAMR")
        if result["wheel"] is not None:
            axes[0, column].plot(t, result["wheel"][:, field], "--", label="wheel_odom")
        axes[0, column].set(xlabel="bag time [s]", ylabel="xy"[field]+" [m]")
    axes[1, 0].plot(t, result["local_m"]["error"], label="local_HAMR")
    if "wheel_m" in result:
        axes[1, 0].plot(t, result["wheel_m"]["error"], "--", label="wheel_odom")
    axes[1, 0].axvline(result["motion"], c="k", ls=":", label="motion starts")
    axes[1, 0].set(xlabel="bag time [s]", ylabel="position error [m]", title="Error from Vicon")
    axes[1, 1].plot(t, np.degrees(result["gt_yaw"]), label="Vicon")
    axes[1, 1].plot(t, np.degrees(result["local_yaw"]), label="local_HAMR")
    if result["wheel_yaw"] is not None:
        axes[1, 1].plot(t, np.degrees(result["wheel_yaw"]), "--", label="wheel_odom")
    axes[1, 1].set(xlabel="bag time [s]", ylabel="yaw change [deg]", title="Unwrapped heading change")
    axes[1, 2].plot(t, np.degrees(result["local_m"]["yaw_error"]), label="local_HAMR")
    if "wheel_m" in result:
        axes[1, 2].plot(t, np.degrees(result["wheel_m"]["yaw_error"]), "--", label="wheel_odom")
    axes[1, 2].set(xlabel="bag time [s]", ylabel="wrapped error [deg]", title="Heading error")
    for axis in axes.flat:
        axis.grid(alpha=.3); axis.legend(fontsize=8)
    fig.suptitle(f"Localization vs Vicon — fitted frame rotation {result['angle']:.1f}°")
    fig.tight_layout(); fig.savefig(os.path.join(out, "01_localization_vs_vicon.png"), dpi=160); plt.close(fig)


def save_odometry(data, out):
    sources = []
    for topic, label, frame in [("/HAMR_base/odom", "Vicon", "mocap"),
                                ("/local_HAMR/odom", "local_HAMR", None),
                                ("/wheel_odom", "wheel_odom", None)]:
        _, values = find_odom(data, topic, frame)
        if values is not None: sources.append((label, values))
    fig, axes = plt.subplots(3, 2, figsize=(15, 10), sharex=True)
    for axis, (field, unit) in zip(axes.flat, [("x","x [m]"),("y","y [m]"),("yaw","yaw [deg]"),
                                                ("vx","vx [m/s]"),("vy","vy [m/s]"),("wz","wz [rad/s]")]):
        for label, values in sources:
            y = np.degrees(np.unwrap(values[field])) if field == "yaw" else values[field]
            axis.plot(values["t"], y, label=label, lw=1)
        axis.set_ylabel(unit); axis.grid(alpha=.3)
    axes[0,0].legend(fontsize=8); axes[-1,0].set_xlabel("bag time [s]"); axes[-1,1].set_xlabel("bag time [s]")
    fig.suptitle("Base odometry in native frames"); fig.tight_layout()
    fig.savefig(os.path.join(out, "02_odometry_states.png"), dpi=160); plt.close(fig)


def save_controls(data, out):
    cmds = sorted(k for k in data["scalars"] if k.endswith("/cmd_vel"))
    encs = sorted(k for k in data["scalars"] if k.endswith("/encoder_ticks"))
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    for topic in cmds:
        v=data["scalars"][topic]; axes[0].plot(v["t"],v["value"],label=topic)
    for topic in encs:
        v=data["scalars"][topic]; axes[1].plot(v["t"],v["value"]-v["value"][0],label=topic)
    axes[0].set(ylabel="command [rad/s]",title="Actuator commands")
    axes[1].set(xlabel="bag time [s]",ylabel="ticks relative to start",title="Encoders")
    for axis in axes: axis.grid(alpha=.3); axis.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(out,"03_controls_and_encoders.png"),dpi=160); plt.close(fig)


def save_imus(data, out):
    fig, axes = plt.subplots(3, 2, figsize=(16, 11), sharex=False)
    for topic, values in sorted(data["imus"].items()):
        for field in ("ax","ay","az"): axes[0,0].plot(values["t"],values[field],lw=.6,label=f"{topic} {field}")
        for field in ("gx","gy","gz"): axes[0,1].plot(values["t"],values[field],lw=.6,label=f"{topic} {field}")
        for axis,field in [(axes[1,0],"roll"),(axes[1,1],"pitch"),(axes[2,0],"yaw")]:
            axis.plot(values["t"],np.degrees(np.unwrap(values[field])),label=topic)
        gaps=np.diff(values["t"]); axes[2,1].hist(gaps[gaps<np.percentile(gaps,99.5)]*1000,bins=70,histtype="step",label=topic)
    for axis,title in zip(axes.flat,["Acceleration [m/s²]","Angular velocity [rad/s]","Roll [deg]","Pitch [deg]","Yaw [deg]","Message gaps [ms]"]):
        axis.set_title(title); axis.grid(alpha=.3); axis.legend(fontsize=6,ncol=2)
    fig.suptitle("All IMU streams"); fig.tight_layout(); fig.savefig(os.path.join(out,"04_imu_streams.png"),dpi=160); plt.close(fig)


def decode_image(msg):
    raw=bytes(msg.data); enc=msg.encoding.lower()
    if enc in {"rgb8","bgr8"}:
        image=np.frombuffer(raw,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        return image[...,::-1] if enc=="bgr8" else image
    if enc in {"16uc1","mono16"}:
        dtype=">u2" if msg.is_bigendian else "<u2"
        return np.frombuffer(raw,dtype).reshape(msg.height,msg.step//2)[:,:msg.width].astype(float)*.001
    if enc in {"mono8","8uc1"}: return np.frombuffer(raw,np.uint8).reshape(msg.height,msg.step)[:,:msg.width]
    return None


def save_cameras(data, out):
    topics=sorted(k for k,v in data["images"].items() if any(x is not None for x in v))
    fig,axes=plt.subplots(len(topics),3,figsize=(14,4.2*len(topics)),squeeze=False)
    for row,topic in enumerate(topics):
        decoded=[decode_image(msg) if msg else None for msg in data["images"][topic]]
        depth=np.concatenate([x[x>0] for x in decoded if x is not None and x.ndim==2]) if any(x is not None and x.ndim==2 for x in decoded) else np.array([])
        vmax=np.percentile(depth,98) if len(depth) else None
        for col,(label,image) in enumerate(zip(("early","middle","late"),decoded)):
            if image is not None and image.ndim==2:
                artist=axes[row,col].imshow(np.ma.masked_less_equal(image,0),cmap="turbo",vmin=0,vmax=vmax)
                fig.colorbar(artist,ax=axes[row,col],label="depth [m]",fraction=.046)
            elif image is not None: axes[row,col].imshow(image)
            axes[row,col].set_title(f"{topic} — {label}"); axes[row,col].axis("off")
    fig.suptitle("Recorded camera samples"); fig.tight_layout(); fig.savefig(os.path.join(out,"05_camera_samples.png"),dpi=160); plt.close(fig)


def save_overview(data, metadata, out):
    duration=metadata["duration"]["nanoseconds"]*1e-9
    rows=sorted([(x["topic_metadata"]["name"],x["message_count"]/duration) for x in metadata["topics_with_message_count"]],key=lambda x:x[1])
    fig,axes=plt.subplots(1,2,figsize=(17,max(7,len(rows)*.34)))
    topics=[x[0] for x in rows]; axes[0].barh(topics,[max(x[1],1e-3) for x in rows]); axes[0].set_xscale("log")
    axes[0].set(xlabel="average messages/s (log)",title="Topic rates"); axes[0].grid(axis="x",alpha=.3)
    for y,topic in enumerate(topics):
        t=data["times"].get(topic,np.array([])); stride=max(1,len(t)//700)
        if len(t): axes[1].scatter(t[::stride],np.full(len(t[::stride]),y),s=2)
    axes[1].set_yticks(range(len(topics)),topics); axes[1].set(xlabel="bag time [s]",title="Recording coverage")
    axes[1].grid(axis="x",alpha=.3); fig.suptitle(f"{metadata['message_count']:,} messages over {duration:.2f} s")
    fig.tight_layout(); fig.savefig(os.path.join(out,"06_topic_overview.png"),dpi=160); plt.close(fig)


def save_turret(data, out):
    _,base=find_odom(data,"/HAMR_base/odom","mocap"); _,tv=find_odom(data,"/HAMR_turret/odom","mocap")
    _,tl=find_odom(data,"/HAMR_turret/odom","odom"); enc=data["scalars"].get("/turret/encoder_ticks")
    fig,axes=plt.subplots(2,1,figsize=(14,8),sharex=True)
    if tv is not None: axes[0].plot(tv["t"],np.degrees(np.unwrap(tv["yaw"])),label="Vicon turret")
    if tl is not None: axes[0].plot(tl["t"],np.degrees(np.unwrap(tl["yaw"])),label="local turret odom")
    if base is not None and tv is not None:
        rel=np.unwrap(np.angle(np.exp(1j*(np.unwrap(tv["yaw"])-sample(base,"yaw",tv["t"],True)))))
        axes[1].plot(tv["t"],np.degrees(rel-rel[0]),label="Vicon turret relative to base")
    if enc is not None: axes[1].plot(enc["t"],(enc["value"]-enc["value"][0])*360/2704,label="encoder (2704 ticks/rev)")
    axes[0].set(ylabel="yaw [deg]",title="Turret yaw sources"); axes[1].set(xlabel="bag time [s]",ylabel="relative angle [deg]")
    for axis in axes: axis.grid(alpha=.3); axis.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(out,"07_turret.png"),dpi=160); plt.close(fig)


def write_reports(data, metadata, result, out):
    duration=metadata["duration"]["nanoseconds"]*1e-9
    with open(os.path.join(out,"topic_inventory.csv"),"w",newline="",encoding="utf-8") as f:
        w=csv.writer(f); w.writerow(["topic","type","messages","average_hz"])
        for item in sorted(metadata["topics_with_message_count"],key=lambda x:x["topic_metadata"]["name"]):
            m=item["topic_metadata"]; w.writerow([m["name"],m["type"],item["message_count"],item["message_count"]/duration])
    lines=["HAMR rosbag analysis","="*60,f"Duration: {duration:.3f} s",f"Messages: {metadata['message_count']:,}",""]
    if result:
        m=result["local_m"]; ratio=m["path"]/result["gt_path"]
        lines += ["Localization vs Vicon",f"  Motion starts: {result['motion']:.3f} s",
                  f"  Frame rotation fit: {result['angle']:.2f} deg",f"  Vicon/local path: {result['gt_path']:.3f} / {m['path']:.3f} m",
                  f"  local position RMSE/p95/max/end: {m['rmse']:.3f} / {m['p95']:.3f} / {m['max']:.3f} / {m['end']:.3f} m",
                  f"  local heading RMSE: {m['yaw_rmse']:.2f} deg"]
        if "wheel_m" in result: lines += [f"  wheel position RMSE: {result['wheel_m']['rmse']:.3f} m",f"  local-wheel RMS separation: {result['local_wheel_rms']:.4f} m"]
        lines += ["","Automatic observations:",f"  - Local odometry reports {ratio*100:.1f}% of Vicon path length."]
        if result["gt_yaw"][-1]*result["local_yaw"][-1] < 0: lines.append("  - Vicon and local yaw accumulate with opposite signs; check encoder polarity/yaw_sign.")
        if result.get("local_wheel_rms",1)<.02: lines.append("  - local_HAMR and wheel_odom are nearly identical; EKF does not materially correct position drift.")
    lines += ["","Median publisher-to-bag delay:"]+[f"  {k}: {np.median(v)*1000:.2f} ms" for k,v in sorted(data["delays"].items())]
    lines += ["","TF frame pairs:"]
    for topic,pairs in data["tf"].items():
        lines += [f"  {topic}: {a} -> {b} ({count})" for (a,b),count in pairs.items()]
    lines += ["","Note: /HAMR_turret/odom contains two publishers; mocap and odom frames are separated in the plots."]
    report="\n".join(lines)+"\n"; open(os.path.join(out,"analysis_summary.txt"),"w",encoding="utf-8").write(report); print(report)


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument("bag"); parser.add_argument("--output")
    args=parser.parse_args(); bag=os.path.abspath(os.path.expanduser(args.bag)); out=os.path.abspath(os.path.expanduser(args.output or os.path.join(bag,"plots")))
    if not os.path.isfile(os.path.join(bag,"metadata.yaml")): parser.error(f"not a ROS 2 bag: {bag}")
    os.makedirs(out,exist_ok=True); metadata=metadata_for(bag)
    print(f"Reading {metadata['message_count']:,} messages ..."); data=read_bag(bag,metadata); result=compare_localization(data)
    save_localization(result,out); save_odometry(data,out); save_controls(data,out); save_imus(data,out)
    save_cameras(data,out); save_overview(data,metadata,out); save_turret(data,out); write_reports(data,metadata,result,out)
    print(f"Plots written to: {out}"); return 0


if __name__ == "__main__": sys.exit(main())
