"""Shared, vehicle-relative waypoint presets for desktop and CLI runs."""
import math

from .controller import validate_points


PRESET_NAMES = ("square", "straight", "out-and-back", "circle")


def preset_points(name, position, yaw, scale=1.):
    """Rotate a preset into world coordinates at the measured vehicle pose.

    Circle is a 16-segment polygon; the existing controller stops at each
    waypoint. Scale changes distances only, not the requested speed.
    """
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("Preset scale must be finite and positive")
    if not math.isfinite(yaw):
        raise ValueError("Vehicle yaw must be finite")
    x, y = validate_points([position])[0]
    routes = {
        "square": [(0., 0.), (.8, 0.), (.8, .8), (0., .8), (0., 0.)],
        "straight": [(0., 0.), (1.5, 0.)],
        "out-and-back": [(0., 0.), (1., 0.), (0., 0.)],
        "circle": [(.6*math.sin(i*math.pi/8), .6*(1-math.cos(i*math.pi/8))) for i in range(17)],
    }
    if name not in routes:
        raise ValueError(f"Unknown trajectory {name!r}; choose {', '.join(PRESET_NAMES)}")
    return [(x+scale*(px*math.cos(yaw)-py*math.sin(yaw)),
             y+scale*(px*math.sin(yaw)+py*math.cos(yaw))) for px, py in routes[name]]


def points_inside(points, bounds):
    x0, x1, y0, y1 = bounds
    return all(x0 <= x <= x1 and y0 <= y <= y1 for x, y in validate_points(points))
