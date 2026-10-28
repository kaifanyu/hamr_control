"""Passive live state. Receipt time is never claimed as acquisition time."""

import math

from caster_encoder_protocol import SequenceTracker
from .protocol import decode_packet, identity, channels as sensor_channels, IdentityGuard, SAMPLE_VALID, VELOCITY_VALID


class LiveState:
    def __init__(self, wheel_id, stale_timeout_s=1.0, expected_device_id="", expected_source_ip="", node_id=4):
        if wheel_id not in ("front_left", "front_right"):
            raise ValueError("wheel_id must be front_left or front_right")
        if not math.isfinite(stale_timeout_s) or stale_timeout_s <= 0:
            raise ValueError("stale_timeout_s must be positive and finite")
        self.node_id = node_id
        sensor_channels(node_id)
        if expected_device_id:
            identity(dict(caster_type="ordinary", wheel_id=wheel_id, node_id=node_id,
                          device_id=expected_device_id, boot_id="0000000000000000"))
        self.wheel_id, self.stale_timeout_s = wheel_id, stale_timeout_s
        self.expected_device_id, self.expected_source_ip = expected_device_id, expected_source_ip
        self.device_id = expected_device_id or None
        self.boot_id = None
        self.retired_boots = set()
        self.streams = {}
        self.channels = {}
        self.metadata = None
        self.accepted_packets = self.rejected_packets = 0
        self.last_rejection = None

    def receive(self, data, source, epoch_ns, monotonic_ns, ros_ns=None):
        try:
            packet = decode_packet(data)
            wheel, device, node, boot = identity(packet)
            if node != self.node_id:
                raise ValueError("packet belongs to another sensor role")
            if wheel != self.wheel_id:
                raise ValueError("packet belongs to another wheel")
            if self.device_id and device != self.device_id:
                raise ValueError("device identity conflicts with this wheel's inventory")
            if self.expected_source_ip and source[0] != self.expected_source_ip:
                raise ValueError("source IP conflicts with configured inventory")
        except ValueError as error:
            self.rejected_packets += 1
            self.last_rejection = str(error)
            raise
        self.accepted_packets += 1
        self.device_id = device
        changed = False
        if boot != self.boot_id and boot not in self.retired_boots:
            if self.boot_id is not None:
                self.retired_boots.add(self.boot_id)
            self.boot_id, self.channels, self.metadata = boot, {}, None
            changed = True
        active = boot == self.boot_id
        envelope = dict(packet=packet, source_ip=source[0], source_port=source[1],
                        receive_epoch_ns=epoch_ns, receive_monotonic_ns=monotonic_ns,
                        receive_ros_ns=ros_ns)
        readings = []
        if packet["type"] == "metadata":
            if active and self.metadata != packet:
                self.metadata, changed = packet, True
            return envelope, readings, changed
        stream = self.streams.setdefault(boot, dict(tracker=SequenceTracker(), last_time=-1))
        tracker = stream["tracker"]
        for sample in packet["samples"]:
            # A bridge may join midway through a boot; pre-join history is unknown.
            if tracker.highest_seq < 0:
                tracker.highest_seq = sample["seq"] - 1
            status = tracker.observe(sample["seq"])
            if not active or status != "in_order" or sample["t_us"] <= stream["last_time"]:
                continue
            stream["last_time"] = sample["t_us"]
            for channel in sensor_channels(self.node_id):
                values = sample[channel]
                timestamp = sample["t_us"] if channel == "swivel" else values["t_us"]
                previous = self.channels.get(channel)
                if previous and timestamp <= previous["t_us"]:
                    continue
                position = values["calibrated_angle_rad"] if channel == "swivel" else values["angle_rad"]
                valid = bool(values["flags"] & SAMPLE_VALID) and position is not None
                self.channels[channel] = dict(t_us=timestamp, seq=sample["seq"],
                    receive_monotonic_ns=monotonic_ns, valid=valid, flags=values["flags"])
                if valid:
                    velocity = values["filtered_velocity_rad_s"] if values["flags"] & VELOCITY_VALID else None
                    readings.append(dict(channel=channel, name=f"{wheel}_caster_{channel}_joint",
                        position=position, velocity=velocity, acquisition_t_us=timestamp))
        return envelope, readings, changed

    def metadata_snapshot(self):
        return dict(caster_type="ordinary", wheel_id=self.wheel_id, device_id=self.device_id, node_id=self.node_id,
            boot_id=self.boot_id, inventory_verified=bool(self.expected_device_id and self.boot_id),
            header_time_basis="host_receipt", acquisition_time_basis="esp_boot_us",
            clocks_synchronized=False, metadata=self.metadata)

    def diagnostics_snapshot(self, monotonic_ns):
        channels = {}
        for channel in sensor_channels(self.node_id):
            sample = self.channels.get(channel)
            age = max(0.0, (monotonic_ns - sample["receive_monotonic_ns"]) / 1e9) if sample else None
            config = (self.metadata or {}).get(channel, {})
            channels[channel] = dict(sample or {}, age_s=age,
                stale=age is None or age > self.stale_timeout_s,
                calibration_verified=bool(config.get("phase_reference_valid") if channel == "swivel"
                                          else config.get("cpr_verified")))
        return dict(self.metadata_snapshot(), channels=channels, accepted_packets=self.accepted_packets,
            rejected_packets=self.rejected_packets, last_rejection=self.last_rejection,
            streams={boot: stream["tracker"].summary() for boot, stream in self.streams.items()})


class SplitLiveState:
    """Two independent ESP clocks and boot histories on one caster UDP port."""

    def __init__(self, wheel_id, stale_timeout_s=1.0, expected_device_ids=("", ""), expected_source_ips=("", "")):
        if (len(expected_device_ids) != 2 or len(expected_source_ips) != 2 or
                any(type(value) is not str for value in (*expected_device_ids, *expected_source_ips))):
            raise ValueError("ordinary inventories must contain [SWIVEL, WHEEL] strings")
        if expected_device_ids[0] and expected_device_ids[0] == expected_device_ids[1]:
            raise ValueError("SWIVEL and WHEEL require distinct physical MAC addresses")
        self.wheel_id = wheel_id
        self.boards = {node: LiveState(wheel_id, stale_timeout_s, mac, ip, node_id=node)
                       for node, mac, ip in zip((5, 6), expected_device_ids, expected_source_ips)}
        self.guard = IdentityGuard()
        self.rejected_packets = 0
        self.last_rejection = None

    def receive(self, data, source, epoch_ns, monotonic_ns, ros_ns=None):
        try:
            packet = decode_packet(data)
            wheel, device, node, _ = identity(packet)
            if wheel != self.wheel_id or node not in self.boards:
                raise ValueError("expected this wheel's split SWIVEL/WHEEL board")
            board = self.boards[node]
            if board.device_id and device != board.device_id:
                raise ValueError("device identity conflicts with sensor inventory")
            if board.expected_source_ip and source[0] != board.expected_source_ip:
                raise ValueError("source IP conflicts with sensor inventory")
            self.guard.accept(packet)
        except ValueError as error:
            self.rejected_packets += 1
            self.last_rejection = str(error)
            raise
        return board.receive(data, source, epoch_ns, monotonic_ns, ros_ns)

    def metadata_snapshot(self):
        boards = {str(node): board.metadata_snapshot() for node, board in self.boards.items()}
        return dict(caster_type="ordinary", ordinary_layout="split", wheel_id=self.wheel_id,
                    boards=boards, inventory_verified=all(board["inventory_verified"] for board in boards.values()),
                    header_time_basis="host_receipt", acquisition_time_basis="independent_esp_boot_us",
                    clocks_synchronized=False)

    def diagnostics_snapshot(self, monotonic_ns):
        boards = {str(node): board.diagnostics_snapshot(monotonic_ns) for node, board in self.boards.items()}
        return dict(self.metadata_snapshot(), boards=boards,
                    channels={channel: values for board in boards.values() for channel, values in board["channels"].items()},
                    accepted_packets=sum(board["accepted_packets"] for board in boards.values()),
                    rejected_packets=self.rejected_packets, last_rejection=self.last_rejection)
