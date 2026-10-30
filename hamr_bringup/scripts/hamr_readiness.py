"""Terminal readiness for the onboard ball-caster recording workflow.

ROS subscriptions, recorder discovery and HTTP GET /api/status only. Local
monotonic time owns freshness; remote header stamps only establish ordering.
"""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
import http.client
import json
import math
import os
from pathlib import Path
import sys
import time

import caster_recording_client as client


@dataclass(frozen=True)
class Spec:
    name: str
    topic: str
    kind: str
    timeout: float = 1.0


SPECS = (
    Spec("Vicon base odom", "/HAMR_base/odom", "nav_msgs/msg/Odometry"),
    *(Spec(f"Vicon {label}", f"/{subject}/pose", "geometry_msgs/msg/PoseStamped")
      for label, subject in (("base pose", "HAMR_base"), ("turret", "HAMR_turret"),
                             ("left caster", "Left_Caster"), ("right caster", "Right_Caster"))),
    Spec("Local EKF odom", "/local_HAMR/odom", "nav_msgs/msg/Odometry"),
    Spec("Wheel odom", "/wheel_odom", "nav_msgs/msg/Odometry"),
    Spec("IMU gyro/accel", "/imu/data", "sensor_msgs/msg/Imu"),
    *(Spec(f"{label} encoder ticks", f"/{subject}/encoder_ticks", "std_msgs/msg/Int32")
      for label, subject in (("Left drive", "left_wheel"), ("Right drive", "right_wheel"),
                             ("Turret", "turret"))),
    Spec("Drive control status", "/wheel_control/status", "std_msgs/msg/Float64MultiArray", 3.0),
    Spec("Drive encoder diagnostics", "/wheel_encoder/diagnostics", "std_msgs/msg/Float64MultiArray", 3.0),
)
EXPECTED = tuple((wheel, node) for wheel in client.WHEELS for node in client.ROLES)


def row(name, status, detail, **values):
    return dict(name=name, status=status, detail=detail, **values)


def finite(values):
    return all(math.isfinite(value) for value in values)


def xyz(value):
    return value.x, value.y, value.z


def measurement_error(spec, message):
    if spec.kind in ("nav_msgs/msg/Odometry", "geometry_msgs/msg/PoseStamped"):
        pose = message.pose.pose if spec.kind == "nav_msgs/msg/Odometry" else message.pose
        q = (*xyz(pose.orientation), pose.orientation.w)
        if not finite((*xyz(pose.position), *q)):
            return "Non-finite pose"
        if abs(sum(v*v for v in q) - 1.0) > .05:
            return "Invalid orientation quaternion"
        if not message.header.frame_id:
            return "Empty pose frame_id"
        if spec.kind == "nav_msgs/msg/Odometry":
            if not finite((*xyz(message.twist.twist.linear), *xyz(message.twist.twist.angular),
                           *message.pose.covariance, *message.twist.covariance)):
                return "Non-finite odometry twist/covariance"
    elif spec.kind == "sensor_msgs/msg/Imu":
        # use_orientation=false/use_mag=false: verify the EKF's gyro input.
        if message.angular_velocity_covariance[0] == -1:
            return "IMU reports angular velocity unavailable"
        if not finite((*xyz(message.angular_velocity), *xyz(message.linear_acceleration),
                       *message.angular_velocity_covariance, *message.linear_acceleration_covariance)):
            return "Non-finite IMU measurement/covariance"
    elif spec.kind == "std_msgs/msg/Float64MultiArray":
        minimum, label = ((17, "wheel_control_status_v1") if spec.topic == "/wheel_control/status"
                          else (22, "wheel_encoder_diag_v1"))
        if len(message.data) < minimum or not finite(message.data):
            return f"Require at least {minimum} finite diagnostic values"
        if message.layout.dim and message.layout.dim[0].label != label:
            return f"Unexpected diagnostic layout (expected {label})"
    return None


@dataclass
class Stream:
    spec: Spec
    receipts: deque = field(default_factory=lambda: deque(maxlen=20000))
    last_good: float = None
    stamp: int = None
    error: str = None
    good_count: int = 0
    invalid_count: int = 0
    warning: str = None

    def receive(self, message, now):
        self.receipts.append(now)
        self.error = measurement_error(self.spec, message)
        if self.error is None and hasattr(message, "header"):
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if stamp <= 0:
                self.error = "Zero/invalid source timestamp"
            elif self.stamp is not None and stamp <= self.stamp:
                self.error = "Source timestamp is not advancing"
            else:
                self.stamp = stamp
        if self.error:
            self.invalid_count += 1
            return
        self.good_count += 1
        self.last_good = now
        self.warning = None
        if self.spec.topic == "/HAMR_base/odom":
            z = message.pose.pose.position.z
            if not .25 <= z <= .40:
                self.warning = f"Base z={z:.3f} m is outside the launch's controller guard [0.25, 0.40]"

    def report(self, now, publishers):
        times = [t for t in self.receipts if now - t <= 5.0]
        hz = (len(times)-1) / (times[-1]-times[0]) if len(times) > 1 and times[-1] > times[0] else 0.0
        age = None if self.last_good is None else max(0.0, now-self.last_good)
        status, detail = "PASS", "Receiving valid data"
        if publishers != 1:
            status, detail = "FAIL", ("No publisher discovered" if publishers == 0 else
                                       f"{publishers} publishers; check duplicate drivers or bag playback")
        elif self.error:
            status, detail = "FAIL", self.error
        elif age is None:
            status, detail = "FAIL", "Publisher exists but no valid data received (check QoS/domain)"
        elif age > self.spec.timeout:
            status, detail = "FAIL", f"No fresh valid data for {age:.2f}s"
        elif self.good_count < 2:
            status, detail = "FAIL", "Waiting for a second valid measurement"
        elif self.warning:
            status, detail = "WARN", self.warning
        if status == "PASS" and self.invalid_count:
            status, detail = "WARN", f"Receiving now; {self.invalid_count} invalid/non-advancing messages observed"
        return row(self.spec.name, status, detail, topic=self.spec.topic, hz=hz, age_s=age,
                   publishers=publishers, valid_messages=self.good_count,
                   invalid_messages=self.invalid_count)


def load_inventory(path):
    """Read the same wheel/role MAC pins as the optional UDP receivers."""
    if path is None:
        return []
    import yaml
    config = yaml.safe_load(Path(path).expanduser().read_text())
    result = []
    for wheel in client.WHEELS:
        params = config[f"/caster_encoder_{wheel}"]["ros__parameters"]
        if params.get("wheel_id") != wheel or params.get("configured_wheel_id") != wheel:
            raise ValueError(f"Caster config wheel marker does not match {wheel}")
        macs = params.get("expected_device_ids", ["", "", ""])
        ips = params.get("expected_source_ips", ["", "", ""])
        if len(macs) != 3 or len(ips) != 3:
            raise ValueError("Caster MAC/IP inventories must have three entries: H1, H2, CENTER")
        for node, (mac, ip) in enumerate(zip(macs, ips), 1):
            if not isinstance(mac, str) or (mac and not client.MAC.fullmatch(mac)):
                raise ValueError(f"Invalid configured MAC for {wheel}/{client.ROLES[node]}")
            if not isinstance(ip, str):
                raise ValueError("Configured source IP must be a string")
            result.append(dict(wheel_id=wheel, node_id=node, device_id=mac, host=ip))
    return result


def probe_recorders(args, config, inventory):
    """Discovery and status GETs, using the existing recorder protocol."""
    transport = client.Transport(timeout=args.http_timeout, retries=1)
    raw = (client.explicit_nodes(args.node, transport) if args.node else client.discover(
        args.discovery_address or config.get("discovery_address", "255.255.255.255"),
        config.get("discovery_port", 5010), args.discovery_timeout))
    devices = client.validate_inventory(raw, inventory=config.get("devices", []), require_all=False)
    devices = client.validate_inventory(devices, inventory=inventory, require_all=False)

    def fetch(device):
        try:
            status = transport.request(device, "/api/status")
            client.check_identity(status, device, same_boot=True)
            return device, status, None, time.monotonic()
        except (OSError, ValueError, http.client.HTTPException, client.RecordingError) as error:
            return device, None, str(error), time.monotonic()
    with ThreadPoolExecutor(max_workers=6) as pool:
        return list(pool.map(fetch, devices))


class Recorders:
    def __init__(self, inventory, duration=None):
        self.inventory = {(d["wheel_id"], d["node_id"]): d for d in inventory}
        self.duration = duration
        self.current = {}
        self.previous = {}
        self.error = None

    def update(self, replies):
        self.previous = self.current
        self.current = {(d["wheel_id"], d["node_id"]): (d, s, error, now)
                        for d, s, error, now in replies}
        self.error = None

    def report(self, now, timeout=8.0):
        result = []
        for key in EXPECTED:
            name = f"ESP {key[0]}/{client.ROLES[key[1]]}"
            item = self.current.get(key)
            if self.error or item is None:
                result.append(row(name, "FAIL", self.error or "No recorder discovered; check power/Wi-Fi or --node"))
                continue
            device, status, error, received = item
            details = dict(device_id=device["device_id"], host=device["host"], age_s=max(0, now-received))
            if error or now-received > timeout:
                result.append(row(name, "FAIL", error or "Recorder status is stale", **details))
                continue
            state = status.get("state")
            count, hz, capacity = (status.get(k) for k in ("sample_count", "sample_hz", "capacity_samples"))
            level, text, advancing = "PASS", f"{state}; recorder API reachable", False
            if state not in ("idle", "prepared", "recording", "stopped", "error"):
                level, text = "FAIL", "Unknown recorder state"
            elif status.get("finalizing"):
                level, text = "FAIL", "Recorder is finalizing; wait for it to finish"
            elif state == "error":
                level, text = "FAIL", f"Recorder error: {status.get('stop_reason')}"
            elif status.get("supports_discard_existing") is not True:
                level, text = "FAIL", "Firmware lacks the start_encoder_recording discard capability"
            elif (type(count) is not int or count < 0 or type(hz) not in (int, float) or
                  not math.isfinite(hz) or hz <= 0 or type(capacity) is not int or capacity <= 0):
                level, text = "FAIL", "Missing/invalid sample count, rate or recording capacity"
            elif count > capacity:
                level, text = "FAIL", "Recorded sample count exceeds reported capacity"
            elif not isinstance(status.get("diagnostics", {}), dict):
                level, text = "FAIL", "Malformed recorder diagnostics"
            elif self.duration is not None and self.duration >= capacity/hz:
                level, text = "FAIL", f"Requested {self.duration:g}s exceeds capacity {capacity/hz:.1f}s"
            elif state == "prepared":
                level, text = "FAIL", "Prepared but not recording; start has not completed"
            elif state == "recording":
                prior = self.previous.get(key)
                if not isinstance(status.get("run_id"), str) or not client.RUN_ID.fullmatch(status["run_id"]):
                    level, text = "FAIL", "Active recorder has no valid run ID"
                elif prior is None or prior[1] is None:
                    level, text = "FAIL", "Recording; waiting for a second sample-count observation"
                else:
                    old = prior[1]
                    same = all(status.get(k) == old.get(k) for k in ("device_id", "boot_id", "run_id"))
                    old_count = old.get("sample_count")
                    advancing = same and type(old_count) is int and count > old_count
                    if not advancing:
                        level, text = "FAIL", "Recording sample count stopped, reset, or identity changed"
                    else:
                        text = f"Recording {status['run_id']}; samples {old_count} -> {count}"
                        before, after = old.get("diagnostics", {}), status.get("diagnostics", {})
                        before = before if isinstance(before, dict) else {}
                        increased = [k for k, v in after.items() if type(v) is int and
                                     type(before.get(k)) is int and v > before[k]]
                        if increased:
                            level, text = "WARN", text + "; counters increased: " + ", ".join(increased)
            else:
                text = f"{state}; ready for start_encoder_recording; capacity {capacity/hz:.1f}s"
                if state == "stopped" and not status.get("acknowledged", False):
                    level, text = "WARN", text + "; previous onboard log is unacknowledged"
            expected = self.inventory.get(key, {})
            if level != "FAIL" and not expected.get("device_id"):
                level, text = "WARN", text + "; physical MAC inventory is not pinned"
            result.append(row(name, level, text, **details, recorder_state=state,
                              run_id=status.get("run_id"), sample_count=count,
                              samples_advancing=advancing))
        active = [r for r in result if r.get("recorder_state") == "recording"]
        if active and (len(active) != 6 or len({r.get("run_id") for r in active}) != 1):
            result.append(row("ESP recording consistency", "FAIL", "All six boards must record the same run together"))
        return result


def bag_report(root, selection):
    from caster_recording_session import active_recorder
    try:
        bag = client.resolve_bag(root, selection)
        process = active_recorder(bag)
        return row("Robot bag recorder", "PASS", str(bag), pid=process["pid"])
    except (OSError, ValueError, client.RecordingError) as error:
        return row("Robot bag recorder", "FAIL", str(error))


def summarize(rows, elapsed):
    failed = sum(r["status"] == "FAIL" for r in rows)
    warnings = sum(r["status"] == "WARN" for r in rows)
    boards = [r for r in rows if r["name"].startswith("ESP front_")]
    sampling = len(boards) == 6 and all(r.get("samples_advancing", False) for r in boards)
    return dict(ready=failed == 0, result="NOT READY" if failed else "READY WITH WARNINGS" if warnings else "READY",
                elapsed_s=elapsed, failed=failed, warnings=warnings, checks=rows,
                all_six_recorder_counts_advancing=sampling,
                scope="ROS data reception and onboard recorder availability; no motion commands or recorder state changes.",
                sample_validation=("All six recorded sample counts advance; per-sample validity is checked after download."
                                   if sampling else "Idle recorder status does not expose live encoder measurements. Run again after starting to verify sample-count progress."))


def display(report, watch=False):
    if watch and sys.stdout.isatty():
        print("\033[2J\033[H", end="")
    print(f"HAMR connections: {report['result']} | {report['elapsed_s']:.1f}s observed")
    print(f"{'CHECK':30} {'STATE':5} {'Hz':>7} {'AGE':>7}  DETAIL")
    for item in report["checks"]:
        rate = f"{item['hz']:.1f}" if "hz" in item else "-"
        age = f"{item['age_s']:.2f}s" if item.get("age_s") is not None else "-"
        print(f"{item['name']:30} {item['status']:5} {rate:>7} {age:>7}  {item['detail']}")
        if item.get("status") == "FAIL" and "topic" in item:
            print("  " + item["topic"])
    print(report["sample_validation"])
    print("If ROS rows are missing: check the sourced workspace, ROS_DOMAIN_ID, and the separate Vicon driver.")
    sys.stdout.flush()


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def parser():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--timeout", type=positive, default=10., help="Observation time in seconds (default: 10)")
    cli.add_argument("--watch", action="store_true", help="Refresh continuously until Ctrl-C")
    cli.add_argument("--report", type=Path, help="Write the latest result as JSON")
    cli.add_argument("--config", type=Path, help="Recorder inventory YAML, same as start_encoder_recording --config")
    cli.add_argument("--caster-config", type=Path, help="Receiver MAC/IP pins; defaults to ~/caster_encoders.yaml if present")
    cli.add_argument("--root", type=Path, default=client.default_root(), help="HAMR_BAG_ROOT / unified recording root")
    cli.add_argument("--bag", type=Path, default=Path("latest"), help="Active robot bag, default: latest")
    cli.add_argument("--duration", type=positive, help="Check that planned recording seconds fit each ESP's storage")
    cli.add_argument("--node", action="append", default=[], metavar="WHEEL:ROLE@IP[:PORT]", help="Explicit ESP endpoint; repeat for all six if broadcast is unavailable")
    cli.add_argument("--discovery-address", help="Recorder discovery broadcast address")
    cli.add_argument("--discovery-timeout", type=positive, default=1.)
    cli.add_argument("--http-timeout", type=positive, default=1.5)
    return cli


def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    config = client.load_config(args.config or client.config_path())
    pins = args.caster_config or (Path.home()/"caster_encoders.yaml" if (Path.home()/"caster_encoders.yaml").is_file() else None)
    inventory = load_inventory(pins)
    # Configuration pins take precedence only when the two inventories agree;
    # probe_recorders validates both inventories independently.
    pinned = {(d["wheel_id"], d["node_id"]): d for d in config.get("devices", [])}
    for item in inventory:
        key = item["wheel_id"], item["node_id"]
        pinned[key] = {**pinned.get(key, {}), **{k: v for k, v in item.items() if v}}
    recorders = Recorders(list(pinned.values()), args.duration)
    streams = [Stream(spec) for spec in SPECS]
    rclpy.init(args=[])
    node = Node("hamr_readiness_check", start_parameter_services=False)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    subscriptions = [node.create_subscription(get_message(s.spec.kind), s.spec.topic,
                     lambda message, stream=s: stream.receive(message, time.monotonic()), qos) for s in streams]
    pool = ThreadPoolExecutor(max_workers=1)
    future = None
    started, next_probe, next_print = time.monotonic(), 0., 0.
    latest = None
    print(f"Checking for {args.timeout:g}s; ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}. "
          "Onboard ESP recording mode: caster UDP receivers and Foxglove are not required.", flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if future is not None and future.done():
                try:
                    recorders.update(future.result())
                except (OSError, ValueError, TypeError, KeyError, http.client.HTTPException, client.RecordingError) as error:
                    recorders.error = str(error)
                future, next_probe = None, now + .5
            if future is None and now >= next_probe:
                future = pool.submit(probe_recorders, args, config, inventory)
            due = now-started >= args.timeout
            if (args.watch and now >= next_print) or (due and not args.watch):
                rows = [s.report(now, node.count_publishers(s.spec.topic)) for s in streams]
                rows += recorders.report(now, timeout=max(8., 2*(args.discovery_timeout+args.http_timeout+1)))
                rows.append(bag_report(args.root, args.bag))
                latest = summarize(rows, now-started)
                display(latest, args.watch)
                if args.report:
                    args.report.parent.mkdir(parents=True, exist_ok=True)
                    client.write_json(args.report, latest)
                next_print = now + (1. if sys.stdout.isatty() else 5.)
                if not args.watch:
                    return 0 if latest["ready"] else 1
        return 130
    finally:
        pool.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        return run(args)
    except KeyboardInterrupt:
        return 130
    except ImportError as error:
        print(f"check_hamr_ready: {error}. Source ROS and ~/hamster_ws/install/setup.bash first.", file=sys.stderr)
        return 2
    except (OSError, ValueError, KeyError, TypeError, http.client.HTTPException, client.RecordingError) as error:
        print(f"check_hamr_ready: {error}", file=sys.stderr)
        return 2
