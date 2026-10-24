"""Select one physical caster without conflating roles shared by two wheels."""

from caster_encoder_protocol import packet_identity

WHEELS = ("front_left", "front_right", "legacy")
LEGACY_TOPIC = "/caster_encoder/packets"


class WheelIdentityError(ValueError):
    """A selected input contains a conflicting physical wheel/device identity."""


def topic_for_wheel(wheel_id):
    if wheel_id not in WHEELS:
        raise ValueError("wheel must be front_left, front_right or legacy")
    return LEGACY_TOPIC if wheel_id == "legacy" else f"/caster_encoder/{wheel_id}/packets"


def wheel_for_topic(topic):
    return next((wheel for wheel in WHEELS if topic == topic_for_wheel(wheel)), None)


def resolve_selection(wheel_id=None, topic=None, available_topics=None):
    """Resolve explicit selectors or a unique bag topic; never guess between wheels."""
    wheel_id = wheel_id or None
    topic = topic or None
    if wheel_id is not None and wheel_id not in WHEELS:
        raise ValueError("wheel must be front_left, front_right or legacy")
    if topic:
        if not topic.startswith("/") or not topic.endswith("/packets"):
            raise ValueError("packet topic must be absolute and end with /packets")
        inferred = wheel_for_topic(topic)
        if wheel_id and inferred and wheel_id != inferred:
            raise ValueError("--wheel and packet topic identify different wheels")
        return wheel_id or inferred, topic
    if wheel_id:
        return wheel_id, topic_for_wheel(wheel_id)
    if available_topics is not None:
        candidates = [topic_for_wheel(wheel) for wheel in WHEELS
                      if topic_for_wheel(wheel) in available_topics]
        if len(candidates) != 1:
            raise ValueError("Select --wheel front_left or front_right (or --topic): "
                             f"bag contains {len(candidates)} recognized packet topics")
        return wheel_for_topic(candidates[0]), candidates[0]
    return None, LEGACY_TOPIC


def metadata_topic(packet_topic):
    return packet_topic.rsplit("/", 1)[0] + "/metadata"


class WheelIdentityGuard:
    """Lock to one wheel and one physical board per role, before any state mutation."""

    def __init__(self, wheel_id=None):
        if wheel_id is not None and wheel_id not in WHEELS:
            raise ValueError("wheel must be front_left, front_right or legacy")
        self.wheel_id = wheel_id
        self.devices = {}

    def accept(self, packet):
        wheel, device, node, boot = packet_identity(packet)
        if self.wheel_id is not None and wheel != self.wheel_id:
            raise WheelIdentityError(f"Expected wheel {self.wheel_id}, received {wheel}; "
                                     "select a single wheel's packet topic or JSONL file")
        if node in self.devices and self.devices[node] != device:
            raise WheelIdentityError(f"Wheel {wheel} node {node} changed device identity "
                                     f"from {self.devices[node]} to {device}")
        if device and any(known == device and role != node for role, known in self.devices.items()):
            raise WheelIdentityError(f"Device {device} is assigned to multiple roles on wheel {wheel}")
        self.wheel_id = wheel
        self.devices[node] = device
        return wheel, device, node, boot
