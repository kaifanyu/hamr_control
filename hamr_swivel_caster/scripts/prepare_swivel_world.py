#!/usr/bin/env python3
"""Compatibility import for the shared exact-route floor/camera preparation."""
from pathlib import Path
import sys

try:
    from prepare_caster_world import prepare_world
except ModuleNotFoundError:
    source = Path(__file__).resolve().parents[2] / "hamr_ball_caster/scripts"
    if not source.is_dir():
        from ament_index_python.packages import get_package_prefix
        source = Path(get_package_prefix("hamr_ball_caster")) / "lib/hamr_ball_caster"
    sys.path.insert(0, str(source))
    from prepare_caster_world import prepare_world
