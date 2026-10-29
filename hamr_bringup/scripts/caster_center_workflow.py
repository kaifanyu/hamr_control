"""Operator-guided CENTER calibration; never commands motion or changes firmware.

Positive carrier phase means forward rolling: from H1 front/H2 back, H1 moves
downward at +90 degrees. This is independent of the CAD joint's -Y convention.
Radius uses stationary endpoints, not a fitted receive-time velocity alignment.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

import caster_encoder_protocol as protocol
from caster_center_calibration import load_calibration

N = 16384
TAU = 2 * math.pi
POSES = (
    ("zero", "H1 FRONT, H2 BACK; fork upright, split axis horizontal"),
    ("quarter", "+90 degrees from zero: H1 DOWN, H2 UP (forward rolling direction)"),
    ("opposite", "+180 degrees from zero: H1 BACK, H2 FRONT"),
    ("return_zero", "Return to H1 FRONT, H2 BACK; same upright reference"),
)


def wrap(value):
    return (np.asarray(value) + math.pi) % TAU - math.pi


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError("must be finite")
    return value


def write_new(path, value):
    """Never overwrite an existing reference or measurement report."""
    path = Path(path)
    payload = json.dumps(value, indent=2, allow_nan=False) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        stream.write(payload)


def center_rows(envelope, wheel, receive_ns):
    packet = protocol.decode_packet(json.dumps(envelope["packet"], allow_nan=False).encode())
    if packet["v"] != 2 or packet.get("wheel_id") != wheel:
        raise ValueError(f"Expected identified {wheel} packets")
    if packet["node_id"] != 3 or packet["type"] != "samples":
        return []
    rows = []
    for sample in packet["samples"]:
        flags = sample["flags"]
        if flags & 3 != 3 or flags & ((1 << 7) | (1 << 12)):
            continue
        count, angle = sample["position14"], sample["absolute_angle_rad"]
        if count is None or angle is None:
            continue
        if not math.isclose(angle, count * TAU / N, abs_tol=1e-12, rel_tol=0):
            raise ValueError("Raw CENTER angle disagrees with position14")
        rows.append(dict(wheel_id=wheel, device_id=packet["device_id"],
                         boot_id=packet["boot_id"], seq=sample["seq"],
                         t_us=sample["t_us"], position14=count,
                         receive_ns=int(receive_ns)))
    return rows


def unique_rows(rows, wheel, device=None):
    if not rows:
        raise ValueError(f"No valid CENTER samples for {wheel}")
    identities = {(r["wheel_id"], r["device_id"], r["boot_id"]) for r in rows}
    if len(identities) != 1:
        raise ValueError(f"{wheel}: device changed or reboot occurred in the selected capture")
    identity = next(iter(identities))
    if identity[0] != wheel or (device is not None and identity[1] != device):
        raise ValueError(f"{wheel}: device identity does not match calibration")
    seen = {}
    for row in rows:
        old = seen.get(row["seq"])
        if old is not None:
            if (old["t_us"], old["position14"]) != (row["t_us"], row["position14"]):
                raise ValueError(f"{wheel}: conflicting duplicate sequence")
            continue
        seen[row["seq"]] = row
    result = sorted(seen.values(), key=lambda r: r["seq"])
    if any(b["t_us"] <= a["t_us"] for a, b in zip(result, result[1:])):
        raise ValueError(f"{wheel}: acquisition clock is not increasing")
    return result


def pose_summary(rows, wheel, min_duration_s=1.5, max_spread_deg=.5):
    rows = unique_rows(rows, wheel)
    if len(rows) < 20 or (rows[-1]["t_us"] - rows[0]["t_us"]) * 1e-6 < min_duration_s:
        raise ValueError(f"{wheel}: insufficient fresh samples for stationary pose")
    if max(np.diff([r["t_us"] for r in rows])) > 250000:
        raise ValueError(f"{wheel}: pose capture has a gap above 0.25 s")
    angles = np.array([r["position14"] for r in rows]) * TAU / N
    mean = float(np.angle(np.mean(np.exp(1j * angles))) % TAU)
    spread = float(np.max(np.abs(wrap(angles - mean))) * 180 / math.pi)
    if spread > max_spread_deg:
        raise ValueError(f"{wheel}: CENTER moved during pose capture ({spread:.3f} deg)")
    return dict(mean_raw_rad=mean, max_deviation_deg=spread, sample_count=len(rows),
                device_id=rows[0]["device_id"], boot_id=rows[0]["boot_id"],
                samples=rows)


def solve_reference(wheel, observations, pose_tolerance_deg=5., return_tolerance_deg=2.):
    if set(observations) != {p[0] for p in POSES}:
        raise ValueError("Require zero, quarter, opposite, and return_zero captures")
    devices = {obs["device_id"] for obs in observations.values()}
    if len(devices) != 1:
        raise ValueError("CENTER device changed between poses")
    z, q, h, r = [observations[name]["mean_raw_rad"] for name, _ in POSES]
    quarter = float(wrap(q - z))
    sign = 1 if quarter > 0 else -1
    quarter_error = abs(math.degrees(quarter) - sign * 90)
    half_error = abs(float(np.degrees(wrap(h - z - math.pi))))
    return_error = abs(float(np.degrees(wrap(r - z))))
    if quarter_error > pose_tolerance_deg:
        raise ValueError(f"Quarter-turn check failed ({quarter_error:.2f} deg error); a half-turn cannot determine sign")
    if half_error > pose_tolerance_deg:
        raise ValueError(f"Opposite-pose check failed ({half_error:.2f} deg error)")
    if return_error > return_tolerance_deg:
        raise ValueError(f"Return-to-zero check failed ({return_error:.2f} deg error)")
    mean_zero = float(np.angle(np.exp(1j * z) + np.exp(1j * r)) % TAU)
    count = int(round(mean_zero * N / TAU)) % N
    return dict(version=1, wheel_id=wheel, device_id=next(iter(devices)), node_id=3,
                role="CENTER", counts_per_revolution=N, zero_position14=count,
                zero_absolute_angle_rad=count * TAU / N, direction_sign=sign,
                physical_direction_verified=True, accuracy_validated=False,
                reference_pose=POSES[0][1],
                direction_convention="Positive = forward rolling; H1 moves from front to down at +90 degrees",
                direction_verification="User-positioned known +90 degree pose; not an independent angular accuracy measurement",
                quantity="Carrier shaft phase relative to fixed upright fork; not robot yaw or H1/H2 shell spin",
                angle_source="Raw position14, before firmware sign/zero; native scale retained",
                created_utc=datetime.now(timezone.utc).isoformat(),
                checks=dict(quarter_error_deg=quarter_error, half_turn_error_deg=half_error,
                            return_zero_error_deg=return_error), observations=observations)


def capture_live(wheel, duration, timeout):
    """New volatile subscription for each pose; discard initial queued traffic."""
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
    except ImportError as error:
        raise RuntimeError("Live capture needs ROS 2; source /opt/ros/jazzy/setup.bash and the workspace") from error
    rclpy.init(args=[])
    node = rclpy.create_node("calibrate_caster_center")
    rows, errors = [], []
    start = time.monotonic()
    warmup = 1.

    def callback(msg):
        if time.monotonic() - start < warmup:
            return
        try:
            envelope = protocol.parse_json(msg.data)
            received = envelope.get("receive_epoch_ns", 0)
            if abs(time.time_ns() - received) > 1_000_000_000:
                raise ValueError("Stale/clock-mismatched live packet; use the live bridge, not bag playback")
            rows.extend(center_rows(envelope, wheel, received))
        except (ValueError, KeyError, TypeError) as error:
            errors.append(error)

    node.create_subscription(String, f"/caster_encoder/{wheel}/packets", callback,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    try:
        while rclpy.ok() and time.monotonic() - start < warmup + duration:
            rclpy.spin_once(node, timeout_sec=.05)
            if errors:
                raise errors[0]
            if time.monotonic() - start > timeout and not rows:
                raise ValueError(f"No fresh {wheel} CENTER data; enable live telemetry and stop onboard recording")
        if not rclpy.ok():
            raise RuntimeError("ROS shutdown during capture")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return pose_summary(rows, wheel, min_duration_s=duration * .75)


def capture_reference(args):
    for wheel in protocol.WHEELS if args.wheel == "both" else (args.wheel,):
        target = args.output / f"{wheel}_center_calibration.json"
        observations_path = args.output / f"{wheel}_center_observations.json"
        if target.exists() or observations_path.exists():
            raise ValueError(f"{wheel} output already exists; use a new output directory for a repeat")
        observations = {}
        print(f"\n{wheel}: support the robot so the carrier can be positioned freely.\n"
              "Keep the fork/main axle fixed. Rotate the carrier, not just a hemisphere shell.\n"
              "Positive phase is forward rolling, the same physical convention on both sides.", flush=True)
        for name, instruction in POSES:
            input(f"\n{name}: {instruction}. Hold still, then press Enter: ")
            observations[name] = capture_live(wheel, args.duration, args.timeout)
            obs = observations[name]
            print(f"  raw mean {obs['mean_raw_rad'] * N / TAU:.2f} counts; "
                  f"max deviation {obs['max_deviation_deg']:.3f} deg", flush=True)
        write_new(observations_path, dict(wheel_id=wheel, poses=observations))
        calibration = solve_reference(wheel, observations)
        write_new(target, calibration)
        print(f"Saved {target}: zero={calibration['zero_position14']}, "
              f"sign={calibration['direction_sign']:+d}; angular accuracy remains unvalidated.")


def rpy_from_quaternion(q):
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(values).all() or abs(norm - 1.) > .01:
        raise ValueError("Invalid Vicon orientation quaternion")
    x, y, z, w = values / norm
    return [math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)),
            math.asin(float(np.clip(2 * (w*y - z*x), -1, 1))),
            math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))]


def read_radius_bag(path, vicon_topic):
    try:
        import rosbag2_py as bag
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError("Bag input needs ROS 2 rosbag2_py; source ROS and your workspace") from error
    if not path.exists():
        raise ValueError(f"Bag does not exist: {path}")
    storage = "sqlite3"
    if path.suffix == ".mcap" or (path.is_dir() and any(path.glob("*.mcap"))):
        storage = "mcap"
    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(path), storage_id=storage), bag.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics = {f"/caster_encoder/{w}/packets": w for w in protocol.WHEELS}
    if vicon_topic not in types or any(t not in types for t in topics):
        raise ValueError("Bag must contain Vicon and both wheels' /packets topics (live telemetry)")
    if types[vicon_topic] not in ("nav_msgs/msg/Odometry", "geometry_msgs/msg/PoseStamped", "geometry_msgs/msg/PoseWithCovarianceStamped"):
        raise ValueError("Vicon topic must be Odometry, PoseStamped or PoseWithCovarianceStamped")
    if any(types[t] != "std_msgs/msg/String" for t in topics):
        raise ValueError("CENTER packets must use std_msgs/msg/String")
    wanted = [vicon_topic, *topics]
    reader.set_filter(bag.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    centers = {w: [] for w in protocol.WHEELS}
    poses, last_header, last_pose, frame = [], None, None, None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        if topic in topics:
            envelope = protocol.parse_json(msg.data)
            if envelope.get("origin") == "encoder_local_recording":
                raise ValueError("Use a live telemetry bag; downloaded flash data needs separate clock alignment")
            centers[topics[topic]].extend(center_rows(envelope, topics[topic], stamp))
        else:
            header = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if header <= 0 or (last_header is not None and header < last_header):
                raise ValueError("Vicon source timestamp is zero or moves backwards")
            if frame is not None and frame != msg.header.frame_id:
                raise ValueError("Vicon frame changed")
            frame = msg.header.frame_id
            pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
            xyz = [pose.position.x, pose.position.y, pose.position.z]
            rpy = rpy_from_quaternion(pose.orientation)
            if not np.isfinite(xyz).all():
                raise ValueError("Non-finite Vicon position")
            if header == last_header:
                if (xyz, rpy) != last_pose:
                    raise ValueError("Conflicting Vicon poses at the same source timestamp")
                continue
            poses.append(dict(receive_ns=int(stamp), xyz=xyz, rpy=rpy))
            last_header, last_pose = header, (xyz, rpy)
    return poses, centers


def unwrap_center(rows, wheel, device, max_rpm):
    rows = unique_rows(rows, wheel, device)
    if len(rows) < 20:
        raise ValueError(f"{wheel}: too few CENTER samples")
    counts = np.array([r["position14"] for r in rows], dtype=float)
    dt = np.diff([r["t_us"] for r in rows]) * 1e-6
    possible = max_rpm / 60 * N * dt + 2  # allow two counts of quantization
    if np.any(possible >= N / 2):
        raise ValueError(f"{wheel}: ambiguous CENTER gap at the supplied maximum RPM; lost turns cannot be recovered")
    delta = (np.diff(counts) + N / 2) % N - N / 2
    if np.any(np.abs(delta) > possible):
        raise ValueError(f"{wheel}: observed motion exceeds supplied maximum RPM")
    radians = np.r_[0., np.cumsum(delta)] * TAU / N
    return rows, radians


def endpoint_masks(times, start, end, dwell):
    return (times >= start) & (times <= start + dwell), (times >= end - dwell) & (times <= end)


def check_window(times, mask, dwell, label, max_gap=.25):
    selected = np.sort(times[mask])
    if len(selected) < 10 or selected[-1] - selected[0] < .8 * dwell:
        raise ValueError(f"{label}: insufficient stationary endpoint coverage")
    if np.max(np.diff(np.sort(selected))) > max_gap:
        raise ValueError(f"{label}: endpoint data gap exceeds {max_gap:g} s")


def estimate_radii(poses, centers, calibrations, *, max_rpm, forward_offset_deg,
                   direction="forward", dwell=2., start_s=None, end_s=None):
    if not math.isfinite(max_rpm) or max_rpm <= 0 or not math.isfinite(forward_offset_deg):
        raise ValueError("Require a finite positive maximum RPM and finite forward offset")
    if not math.isfinite(dwell) or dwell <= 0:
        raise ValueError("Dwell must be positive")
    if len(poses) < 20 or any(not centers[w] for w in protocol.WHEELS):
        raise ValueError("Missing Vicon or CENTER data")
    origin = min(p["receive_ns"] for p in poses)
    # Relative seconds keep precision without converting epoch nanoseconds first.
    pt = np.array([(p["receive_ns"] - origin) * 1e-9 for p in poses])
    if np.any(np.diff(pt) <= 0):
        raise ValueError("Vicon bag timestamps must increase")
    ct = {w: np.array([(r["receive_ns"] - origin) * 1e-9 for r in centers[w]]) for w in protocol.WHEELS}
    start = max(pt[0], *(float(min(t)) for t in ct.values())) if start_s is None else start_s
    end = min(pt[-1], *(float(max(t)) for t in ct.values())) if end_s is None else end_s
    if not math.isfinite(start) or not math.isfinite(end) or end - start < 2 * dwell + 1:
        raise ValueError("Selected interval needs two stationary dwells plus travel")
    mask = (pt >= start) & (pt <= end)
    selected = [p for p, keep in zip(poses, mask) if keep]
    pt = pt[mask]
    if len(pt) < 20 or np.max(np.diff(pt)) > .25:
        raise ValueError("Vicon data missing or gap exceeds 0.25 s")
    xyz = np.array([p["xyz"] for p in selected])
    rpy = np.array([p["rpy"] for p in selected])
    if not np.isfinite(xyz).all() or not np.isfinite(rpy).all():
        raise ValueError("Non-finite Vicon pose")
    first, last = endpoint_masks(pt, start, end, dwell)
    for name, window in (("start", first), ("finish", last)):
        check_window(pt, window, dwell, f"Vicon {name}")
        if np.max(np.linalg.norm(xyz[window] - np.mean(xyz[window], axis=0), axis=1)) > .005:
            raise ValueError(f"Vicon {name} is moving; hold still for at least {dwell:g} s")
    attitude_change = np.max(np.abs(wrap(rpy - rpy[first][0])), axis=0)
    if np.max(attitude_change) > math.radians(2.):
        raise ValueError("Robot orientation changed over 2 degrees; use a fixed-fork straight run")
    start_xyz, end_xyz = np.mean(xyz[first], axis=0), np.mean(xyz[last], axis=0)
    yaw = float(np.angle(np.mean(np.exp(1j * rpy[first, 2])))) + math.radians(forward_offset_deg)
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    lateral = np.array([-forward[1], forward[0]])
    displacement = (xyz[:, :2] - start_xyz[:2]) @ forward
    distance = float((end_xyz[:2] - start_xyz[:2]) @ forward)
    expected_sign = 1 if direction == "forward" else -1
    if distance * expected_sign < 1.:
        raise ValueError("Need at least 1 m in the selected direction; check Vicon forward offset")
    lateral_error = float(np.max(np.abs((xyz[:, :2] - start_xyz[:2]) @ lateral)))
    if lateral_error > .02:
        raise ValueError("Lateral motion exceeds 2 cm; check axle alignment and Vicon forward offset")
    progress = displacement * expected_sign
    if np.max(np.maximum.accumulate(progress) - progress) > .02:
        raise ValueError("Run reverses direction; analyze separate forward and reverse bags")
    if np.ptp(xyz[:, 2]) > .02:
        raise ValueError("Vicon height changes over 2 cm; use a level-floor run")
    results = {}
    for wheel in protocol.WHEELS:
        cal = calibrations[wheel]
        if cal["wheel_id"] != wheel or cal.get("physical_direction_verified") is not True:
            raise ValueError(f"{wheel}: first complete zero/direction calibration")
        if cal.get("direction_convention") != "Positive = forward rolling; H1 moves from front to down at +90 degrees":
            raise ValueError(f"{wheel}: radius requires this tool's forward-positive reference convention")
        subset = [r for r, t in zip(centers[wheel], ct[wheel]) if start <= t <= end]
        rows, angle = unwrap_center(subset, wheel, cal["device_id"], max_rpm)
        times = np.array([(r["receive_ns"] - origin) * 1e-9 for r in rows])
        a, b = endpoint_masks(times, start, end, dwell)
        for name, window in (("start", a), ("finish", b)):
            check_window(times, window, dwell, f"{wheel} {name}")
            if np.ptp(angle[window]) > math.radians(.5):
                raise ValueError(f"{wheel} {name}: CENTER is not stationary")
        delta = float(np.mean(angle[b]) - np.mean(angle[a])) * cal["direction_sign"]
        if abs(delta) < TAU:
            raise ValueError(f"{wheel}: need at least one full CENTER turn")
        if delta * expected_sign <= 0:
            raise ValueError(f"{wheel}: rolling direction disagrees with quarter-turn calibration")
        radius = distance / delta
        if not .02 <= radius <= .3:
            raise ValueError(f"{wheel}: implausible caster radius {radius:.4f} m; inspect data and geometry")
        results[wheel] = dict(device_id=cal["device_id"], boot_id=rows[0]["boot_id"],
                              radius_m=radius, calibrated_angle_change_rad=delta,
                              revolutions=abs(delta) / TAU, sample_count=len(rows),
                              zero_position14=cal["zero_position14"], direction_sign=cal["direction_sign"],
                              max_valid_acquisition_gap_s=float(np.max(np.diff([r["t_us"] for r in rows])) * 1e-6))
    return dict(version=1, method="Signed stopped-endpoint Vicon displacement / unwrapped CENTER angle",
                effective_radius_only=True, accuracy_validated=False,
                timing_basis="Bag receipt times select stationary windows only; no motion-time synchronization claimed",
                assumptions=["Fixed transverse carrier axle and upright fork", "Negligible slip and approximately spherical rolling contact",
                             "Vicon forward offset matches the physical carrier rolling direction",
                             "Stationary endpoints outlast telemetry delay; supplied maximum RPM bounds all motion"],
                maximum_rpm_assumption=max_rpm, vicon_forward_offset_deg=forward_offset_deg,
                direction=direction, start_s=start, end_s=end, dwell_s=dwell,
                signed_distance_m=distance, lateral_max_m=lateral_error,
                attitude_max_change_deg=np.degrees(attitude_change).tolist(), wheels=results)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest="command", required=True)
    zero = subs.add_parser("zero", help="Guided live captures: zero, +90, opposite, return zero")
    zero.add_argument("--wheel", choices=["both", *protocol.WHEELS], default="both")
    zero.add_argument("--output", type=Path, required=True, help="Directory for new per-wheel references")
    zero.add_argument("--duration", type=positive, default=3., help="Stationary capture seconds per pose")
    zero.add_argument("--timeout", type=positive, default=3.)
    radius = subs.add_parser("radius", help="Estimate both radii from one stopped-start/stopped-end straight bag")
    radius.add_argument("--bag", type=Path, required=True)
    radius.add_argument("--calibration-dir", type=Path, required=True)
    radius.add_argument("--output", type=Path, required=True, help="New JSON report; references are preserved")
    radius.add_argument("--vicon-topic", default="/HAMR_base/odom")
    radius.add_argument("--vicon-forward-offset-deg", type=finite, required=True,
                        help="Angle ADDED to raw Vicon yaw for carrier forward; current HAMR hardware config uses 90")
    radius.add_argument("--max-rpm", type=positive, required=True,
                        help="Conservative physical upper bound for CENTER throughout this run; not a fitted rate")
    radius.add_argument("--direction", choices=("forward", "reverse"), default="forward")
    radius.add_argument("--dwell", type=positive, default=2., help="Stationary seconds at each endpoint")
    radius.add_argument("--start-s", type=finite, help="Interval start relative to first Vicon bag message")
    radius.add_argument("--end-s", type=finite, help="Interval end relative to first Vicon bag message")
    args = parser.parse_args(argv)
    try:
        if args.command == "zero":
            capture_reference(args)
        else:
            if args.output.exists():
                raise ValueError("Output already exists; choose a new report filename")
            calibrations = {w: load_calibration(args.calibration_dir / f"{w}_center_calibration.json") for w in protocol.WHEELS}
            poses, centers = read_radius_bag(args.bag, args.vicon_topic)
            result = estimate_radii(poses, centers, calibrations, max_rpm=args.max_rpm,
                                    forward_offset_deg=args.vicon_forward_offset_deg,
                                    direction=args.direction, dwell=args.dwell,
                                    start_s=args.start_s, end_s=args.end_s)
            result.update(bag=str(args.bag.resolve()), calibration_dir=str(args.calibration_dir.resolve()),
                          vicon_topic=args.vicon_topic, created_utc=datetime.now(timezone.utc).isoformat())
            write_new(args.output, result)
            for wheel, value in result["wheels"].items():
                print(f"{wheel}: effective radius {value['radius_m'] * 1000:.3f} mm "
                      f"over {value['revolutions']:.3f} revolutions")
            print(f"Saved {args.output}. Repeat with a separate run to validate; no firmware/config was changed.")
        return 0
    except (ValueError, OSError, RuntimeError, KeyError, TypeError, EOFError) as error:
        print(f"calibrate_caster_center: {error}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Calibration interrupted; no motion was commanded.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
