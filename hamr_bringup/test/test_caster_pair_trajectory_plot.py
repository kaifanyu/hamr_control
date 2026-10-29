"""Trajectory insets preserve epoch alignment and never invent missing motion."""
import csv
from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from caster_pair_trajectory_plot import PairTrajectoryPlot


START = 1_800_000_000_000_000_001


def test_inset_only_reveals_past_positions_on_the_encoder_clock(tmp_path):
    from PIL import Image

    rows = [{"stamp_ns": START+100, "x": 0., "y": 0., "yaw": .1},
            {"stamp_ns": START+500_000_000, "x": 1., "y": 1., "yaw": .2}]
    plot = PairTrajectoryPlot(rows, START, START+1_000_000_000, 1920,
                              {"topic": "/HAMR_base/odom", "frame_id": "odom"})
    assert plot.current(START+99) is None
    assert plot.current(START+100)["x"] == 0
    assert plot.current(START+499_999_999)["x"] == 0
    assert plot.current(START+500_000_000)["x"] == 1
    before = Image.new("RGB", (1920, 1080), "black")
    after = before.copy()
    plot.draw(before, START+100)
    plot.draw(after, START+500_000_000)
    x, y = plot.points[1]
    pixel = (620+round(x), 210+round(y))
    assert before.getpixel(pixel) != after.getpixel(pixel)
    assert before.getpixel((619, 400)) == (0, 0, 0)
    assert before.getpixel((950, 400)) == (0, 0, 0)
    assert rows[0]["stamp_ns"] == START+100
    plot.export(tmp_path / "pair.mp4")
    with (tmp_path / "pair.trajectory.csv").open() as stream:
        exported = list(csv.DictReader(stream))
    assert int(exported[0]["stamp_ns"]) == START+100
    assert [float(row["yaw"]) for row in exported] == [.1, .2]
    assert Image.open(tmp_path / "pair.trajectory.png").size == (330, 370)


def test_nonoverlapping_recording_stays_empty_and_exports_no_invented_path(tmp_path):
    from PIL import Image

    plot = PairTrajectoryPlot([{"stamp_ns": START-1, "x": 99., "y": 99.}],
                              START, START+1_000_000_000, 1920,
                              {"reason": "The downloaded folder contains no matching robot bag."})
    assert plot.current(START+1_000_000_000) is None
    assert plot.rows == []
    assert plot.metadata["available"] is False
    assert plot.metadata["reason"] == "The downloaded folder contains no matching robot bag."
    first = Image.new("RGB", (1920, 1080), "black")
    last = first.copy()
    plot.draw(first, START)
    plot.draw(last, START+1_000_000_000)
    assert first.tobytes() == last.tobytes()
    plot.export(tmp_path / "empty.mp4")
    with (tmp_path / "empty.trajectory.csv").open() as stream:
        assert list(csv.DictReader(stream)) == []


def test_long_position_gaps_break_path_and_invalid_positions_fail():
    rows = [{"stamp_ns": START, "x": 0, "y": 0},
            {"stamp_ns": START+100_000_000, "x": 1, "y": 0},
            {"stamp_ns": START+2_000_000_000, "x": 2, "y": 0}]
    plot = PairTrajectoryPlot(rows, START, START+3_000_000_000, 1920)
    assert plot.segments == [0, 2]
    with pytest.raises(ValueError, match="finite"):
        PairTrajectoryPlot([dict(rows[0], x=float("nan"))], START, START+10, 1920)
    with pytest.raises(ValueError, match="integer nanoseconds"):
        PairTrajectoryPlot([dict(rows[0], stamp_ns=float(START))], START, START+10, 1920)
