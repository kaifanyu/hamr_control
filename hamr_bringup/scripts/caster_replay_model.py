#!/usr/bin/env python3
"""Map recorded encoder samples into the existing CAD caster's joint coordinates.

This is measured-motion visualization, without a physics solver or an external
accuracy reference. Unknown polar-roller motion is never inferred from shells.
"""

import json
import math
from pathlib import Path

from caster_encoder_protocol import (
    ROLES, TelemetryState, integer, parse_json,
)


SHELL_JOINTS = (
    "caster_positive_hemisphere_joint", "caster_negative_hemisphere_joint",
)
CARRIER_JOINT = "caster_carrier_joint"
ROLLER_JOINTS = (
    "caster_positive_polar_roller_joint", "caster_negative_polar_roller_joint",
)
DEFAULT_MAPPING = {
    "h1_joint": SHELL_JOINTS[0], "h2_joint": SHELL_JOINTS[1],
    "h1_sign": 1.0, "h2_sign": 1.0, "center_sign": 1.0,
    "h1_zero_rad": 0.0, "h2_zero_rad": 0.0, "center_zero_rad": 0.0,
    "mapping_verified": False,
}


def validate_playback(bag, rate):
    """Validate launch inputs before any ROS process is started."""
    bag_path = Path(bag).expanduser().resolve()
    if not bag_path.exists():
        raise ValueError(f"Bag does not exist: {bag_path}")
    if bag_path.is_dir() and not (bag_path / "metadata.yaml").is_file():
        raise ValueError(f"Bag directory has no metadata.yaml: {bag_path}")
    if bag_path.is_file() and bag_path.suffix not in (".mcap", ".db3"):
        raise ValueError("Bag file must end in .mcap or .db3")
    try:
        speed = float(rate)
    except (TypeError, ValueError) as error:
        raise ValueError("rate must be finite and positive") from error
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("rate must be finite and positive")
    return str(bag_path), speed


class ReplayModel:
    def __init__(self, mapping=None, stale_timeout_s=1.0):
        self.mapping = dict(DEFAULT_MAPPING)
        if mapping is not None:
            unknown = set(mapping) - set(self.mapping)
            if unknown:
                raise ValueError("Unknown mapping fields: " + ", ".join(sorted(unknown)))
            self.mapping.update(mapping)
        if {self.mapping["h1_joint"], self.mapping["h2_joint"]} != set(SHELL_JOINTS):
            raise ValueError("h1_joint and h2_joint must uniquely assign the two CAD hemisphere joints")
        for role in ("h1", "h2", "center"):
            if type(self.mapping[f"{role}_sign"]) not in (int, float) or self.mapping[f"{role}_sign"] not in (-1, 1):
                raise ValueError(f"{role}_sign must be +1 or -1")
            zero = self.mapping[f"{role}_zero_rad"]
            if type(zero) not in (int, float) or not math.isfinite(zero):
                raise ValueError(f"{role}_zero_rad must be finite")
        if type(self.mapping["mapping_verified"]) is not bool:
            raise ValueError("mapping_verified must be true or false")
        self.stale_timeout_s = stale_timeout_s
        self.state = TelemetryState(stale_timeout_s=stale_timeout_s)
        self.values = {}
        self.last_replay_ns = None
        self.clock_resets = 0
        self.boot_changes = {node: 0 for node in ROLES}
        self.invalid_envelopes = 0
        self.last_error = None

    def advance_time(self, replay_ns):
        integer(replay_ns, "replay_ns", 0, (1 << 63) - 1)
        if self.last_replay_ns is not None and replay_ns < self.last_replay_ns:
            had_data = bool(self.state.valid_packets or self.values)
            self.state = TelemetryState(stale_timeout_s=self.stale_timeout_s)
            self.values.clear()
            self.boot_changes = {node: 0 for node in ROLES}
            # Player startup can initialize /clock backwards before any data.
            # Report a replay reset only when it actually discarded a segment.
            if had_data:
                self.clock_resets += 1
        self.last_replay_ns = replay_ns

    def receive(self, message, replay_ns):
        """Accept an envelope and return every accepted source-angle reading."""
        self.advance_time(replay_ns)
        try:
            envelope = parse_json(message) if isinstance(message, (str, bytes)) else message
            if not isinstance(envelope, dict) or not isinstance(envelope.get("packet"), dict):
                raise ValueError("Expected a recorded packet envelope")
            integer(envelope.get("receive_epoch_ns"), "receive_epoch_ns", 0, (1 << 63) - 1)
            integer(envelope.get("receive_monotonic_ns"), "receive_monotonic_ns", 0, (1 << 63) - 1)
            integer(envelope.get("source_port"), "source_port", 0, 65535)
            if not isinstance(envelope.get("source_ip"), str) or not envelope["source_ip"]:
                raise ValueError("Envelope requires source_ip")
            previous_boots = dict(self.state.current_boot)
            _, readings, _ = self.state.receive(
                json.dumps(envelope["packet"], allow_nan=False).encode("utf-8"),
                (envelope["source_ip"], envelope["source_port"]),
                envelope["receive_epoch_ns"], replay_ns, replay_ns,
            )
            for node, boot in self.state.current_boot.items():
                if node in previous_boots and previous_boots[node] != boot:
                    self.boot_changes[node] += 1
            for reading in readings:
                node = reading["node_id"]
                role = ROLES[node].lower()
                value = self.mapping[f"{role}_sign"] * (
                    reading["position"] - self.mapping[f"{role}_zero_rad"])
                self.values[node] = {
                    "position": value, "source_angle_rad": reading["position"],
                    "seq": reading["seq"], "boot_id": self.state.current_boot[node],
                    "last_valid_replay_ns": replay_ns,
                }
            self.last_error = None
            return readings
        except (ValueError, TypeError, OverflowError, RecursionError) as error:
            self.invalid_envelopes += 1
            self.last_error = str(error)
            raise ValueError(f"Invalid replay envelope: {error}") from error

    def snapshot(self, replay_ns):
        self.advance_time(replay_ns)
        health = self.state.diagnostics_snapshot(replay_ns)
        nodes = []
        for node in health["nodes"]:
            node = dict(node)
            identity = node["node_id"]
            value = self.values.get(identity)
            stream = self.state.streams.get((identity, node["boot_id"]))
            highest = -1 if stream is None else stream["highest_display_seq"]
            if value is None:
                condition = "unknown"
            elif value["boot_id"] != node["boot_id"]:
                condition = "held_after_reboot"
            elif node["stale"]:
                condition = "stale_held"
            elif value["seq"] < highest:
                condition = "invalid_held"
            else:
                condition = "fresh"
            node.update(
                display_state=condition,
                model_position_rad=None if value is None else value["position"],
                displayed_boot_id=None if value is None else value["boot_id"],
                last_valid_age_s=None if value is None else max(0, replay_ns - value["last_valid_replay_ns"]) / 1e9,
                boot_changes=self.boot_changes[identity],
                flags=None if stream is None else stream["last_flags"],
            )
            nodes.append(node)
        # Before all three measured rotations exist, do not invent a complete
        # initial mechanism pose. Afterwards hold last valid joints with status.
        complete = len(self.values) == 3
        names = [self.mapping["h1_joint"], self.mapping["h2_joint"], CARRIER_JOINT, *ROLLER_JOINTS]
        positions = [self.values[node]["position"] for node in (1, 2, 3)] + [0.0, 0.0] if complete else []
        status = {
            **health, "nodes": nodes, "mode": "measured_motion_kinematic_replay",
            "complete_pose_available": complete, "mapping": dict(self.mapping),
            "clock_resets": self.clock_resets, "invalid_envelopes": self.invalid_envelopes,
            "last_error": self.last_error,
            "unmeasured_joints_held_at_nominal_zero": list(ROLLER_JOINTS),
            "interpolation": "none; hold last valid sample; reset on backwards clock",
            "accuracy": "unavailable_without_independent_reference",
            "mapping_calibration_current": self.mapping["mapping_verified"] and not any(
                self.boot_changes[node] for node in (1, 2)),
        }
        return names if complete else [], positions, status


def status_text(status):
    mapping = "user-calibrated CAD mapping" if status["mapping_calibration_current"] else "ILLUSTRATIVE mapping: signs / zeros unverified"
    rows = ["Recorded caster motion | kinematic CAD replay", mapping]
    for node in status["nodes"]:
        value = node["model_position_rad"]
        angle = "unknown" if value is None else f"{value:.3f} rad"
        rows.append(f"{node['role']}: {angle}  [{node['display_state']}]")
    if any(node["boot_changes"] for node in status["nodes"]):
        rows.append("Encoder reboot: new segment; recheck physical zero")
    rows.append("Polar rollers: unmeasured; shown at nominal zero")
    unverified_ppr = [node["role"] for node in status["nodes"]
                      if node["node_id"] in (1, 2) and not (node["flags"] or 0) & (1 << 2)]
    if unverified_ppr:
        rows.append("PPR unverified: " + ", ".join(unverified_ppr))
    rows.append("Timing: host receipt; ESP clocks unsynchronized")
    rows.append("Phase dots: illustrative | Accuracy: no independent reference")
    if status["clock_resets"]:
        rows.append(f"Playback clock resets: {status['clock_resets']}")
    if status["last_error"]:
        rows.append("Malformed replay packet rejected")
    return "\n".join(rows)
