"""Read-only CENTER reference calibration on the native absolute-count scale."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

from caster_encoder_protocol import DEVICE_ID, WHEELS, decode_packet, integer, parse_json


def default_calibration():
    name = "front_left_center_calibration.json"
    source = Path(__file__).resolve().parent.parent / "config" / name
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("hamr_bringup")) / "config" / name


def load_calibration(path):
    value = parse_json(Path(path).read_text(encoding="utf-8"))
    if (not isinstance(value, dict) or type(value.get("version")) is not int or value["version"] != 1
            or type(value.get("node_id")) is not int or value["node_id"] != 3):
        raise ValueError("Expected a version 1 CENTER calibration")
    if (value.get("wheel_id") not in WHEELS or not isinstance(value.get("device_id"), str)
            or not DEVICE_ID.fullmatch(value["device_id"])):
        raise ValueError("Calibration must identify a wheel and permanent device MAC")
    if type(value.get("counts_per_revolution")) is not int or value["counts_per_revolution"] != 16384:
        raise ValueError("CENTER calibration must retain the native 16384 counts/revolution")
    zero = integer(value.get("zero_position14"), "zero_position14", 0, 16383)
    if type(value.get("direction_sign")) is not int or value["direction_sign"] not in (-1, 1):
        raise ValueError("direction_sign must be +1 or -1")
    for name in ("physical_direction_verified", "accuracy_validated"):
        if type(value.get(name)) is not bool:
            raise ValueError(f"Calibration requires explicit {name}")
    angle = value.get("zero_absolute_angle_rad")
    if type(angle) not in (int, float) or not math.isclose(angle, zero * math.tau / 16384, rel_tol=0, abs_tol=1e-12):
        raise ValueError("Saved zero angle disagrees with its native absolute count")
    return value


def orientation(position14, calibration):
    count = integer(position14, "position14", 0, 16383)
    bins = calibration["counts_per_revolution"]
    delta = (calibration["direction_sign"] * (count - calibration["zero_position14"])) % bins
    return {
        "wheel_id": calibration["wheel_id"], "device_id": calibration["device_id"],
        "position14": count, "raw_absolute_deg": count * 360 / bins,
        "reference_position14": calibration["zero_position14"],
        "orientation_deg": delta * 360 / bins,
        "physical_direction_verified": calibration["physical_direction_verified"],
        "accuracy_validated": calibration["accuracy_validated"],
        "angle_definition": "Single-turn shaft phase relative to " + calibration.get(
            "reference_pose", "the saved H1-front/H2-back pose") + "; not robot yaw",
        "positive_direction": "increasing raw counts" if calibration["direction_sign"] == 1 else "decreasing raw counts",
    }


def packet_reading(envelope, calibration):
    packet = decode_packet(json.dumps(envelope["packet"], allow_nan=False).encode())
    if packet["node_id"] != 3:
        return None
    if (packet.get("wheel_id"), packet.get("device_id")) != (calibration["wheel_id"], calibration["device_id"]):
        raise ValueError("CENTER identity does not match the saved calibration")
    if packet["type"] != "samples":
        return None
    # Only the latest sample in the batch is a current observation. A failed
    # latest sample must not silently substitute an older good one.
    sample = packet["samples"][-1]
    flags = sample["flags"]
    if flags & 3 != 3 or flags & ((1 << 7) | (1 << 12)):
        return None  # Requires ready + sample_valid, no parity/SPI error.
    count, raw = sample["position14"], sample["absolute_angle_rad"]
    if count is None or raw is None:
        return None
    if not math.isclose(raw, count * math.tau / 16384, rel_tol=0, abs_tol=1e-12):
        raise ValueError("Raw CENTER angle disagrees with position14")
    # No unwrap-continuity requirement: this is a single-turn absolute reading.
    result = orientation(count, calibration)
    result.update(input="live_raw_absolute", boot_id=packet["boot_id"], seq=sample["seq"],
                  device_t_us=sample["t_us"], sample_flags=flags,
                  receive_epoch_ns=envelope.get("receive_epoch_ns"))
    return result


def monitor(calibration, once, timeout):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init(args=[])
    node = rclpy.create_node("caster_center_angle")
    topic = f"/caster_encoder/{calibration['wheel_id']}/packets"
    last_valid, last_print = time.monotonic(), -math.inf
    done = False
    highest = {}
    current_boot, retired_boots = None, set()

    def callback(message):
        nonlocal last_valid, last_print, done, current_boot
        result = packet_reading(parse_json(message.data), calibration)
        if result is None:
            return
        boot, seq = result["boot_id"], result["seq"]
        if boot in retired_boots:
            return
        if current_boot is not None and boot != current_boot:
            retired_boots.add(current_boot)
        current_boot = boot
        if seq <= highest.get(boot, -1):
            return
        highest[boot] = seq
        last_valid = time.monotonic()
        if once or last_valid - last_print >= .5:
            print(json.dumps(result, indent=2 if once else None), flush=True)
            last_print = last_valid
            done = once

    node.create_subscription(String, topic, callback,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    try:
        while rclpy.ok() and not done:
            if time.monotonic() - last_valid > timeout:
                raise RuntimeError(f"No fresh valid CENTER reading for {timeout:g}s on {topic}; enable the live receiver and stop onboard recording first")
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, help="default: saved front-left CENTER reference")
    parser.add_argument("--position14", type=int, help="calculate offline from one raw count instead of subscribing to ROS")
    parser.add_argument("--once", action="store_true", help="print one live observation and exit")
    parser.add_argument("--timeout", type=float, default=10., help="maximum seconds without a fresh valid live reading")
    args = parser.parse_args(argv)
    try:
        calibration = load_calibration(args.calibration or default_calibration())
        if not math.isfinite(args.timeout) or args.timeout <= 0:
            raise ValueError("--timeout must be finite and positive")
        if args.position14 is not None:
            print(json.dumps(dict(orientation(args.position14, calibration), input="manual_position14"), indent=2))
        else:
            monitor(calibration, args.once, args.timeout)
        return 0
    except KeyboardInterrupt:
        return 0
    except (ValueError, OSError, RuntimeError, KeyError, TypeError) as error:
        print(f"caster_center_angle: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
