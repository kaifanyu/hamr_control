"""Paired charts keep the six identities and one acquisition replay clock."""
import csv
from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from caster_pair_plots import PairEncoderPlots, WHEELS
from render_caster_encoder_video import sample_frames
from test_caster_encoder_replay import wheel_envelope


START = 1_800_000_000_000_000_001


def paired_events():
    return {
        wheel: [(START + offset, wheel_envelope(node, wheel=wheel, angle=node / 10 + i))
                for node in (1, 2, 3)]
        for i, (wheel, offset) in enumerate(zip(WHEELS, (10, 100_000_000)))
    }


def test_two_columns_share_start_preserve_source_angles_and_export_identity(tmp_path):
    events = paired_events()
    plots = PairEncoderPlots(events, START, START+1_000_000_000, 1920,
                             "dual recording", "Estimated clock mapping; approximate recovery.")
    assert plots.current("front_left", "H1", START + 10)["angle_rad"] == .1
    assert plots.current("front_right", "H1", START + 10) is None
    right = plots.current("front_right", "H1", START + 100_000_000)
    assert right["angle_rad"] == 1.1
    assert right["time_s"] == .1
    assert plots.current("front_left", "H1", START + 9) is None
    assert plots.metadata["cad_mapping_applied"] is False
    assert plots.metadata["time_basis"] == "Estimated clock mapping; approximate recovery."

    plots.export(tmp_path / "paired.mp4")
    with (tmp_path / "paired.encoders.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 6
    assert {(r["wheel_id"], r["role"]) for r in rows} == {
        (wheel, role) for wheel in WHEELS for role in ("H1", "H2", "CENTER")}
    assert int(rows[0]["replay_time_ns"]) == START+10
    assert {float(r["time_s"]) for r in rows if r["wheel_id"] == "front_right"} == {.1}
    assert "encoder_angle_rad" in rows[0]
    assert "model_position_rad" not in rows[0]
    assert (tmp_path / "paired.encoders.png").stat().st_size > 1000
    assert (tmp_path / "paired.encoders.pdf").stat().st_size > 1000


def test_panel_requires_one_timestamp_and_does_not_apply_cad_offsets():
    from PIL import Image

    events = paired_events()
    plots = PairEncoderPlots(events, START, START+1_000_000_000, 1920,
                             "dual recording", "Estimated clock mapping.")
    frames = {wheel: sample_frames(data, [START + 200_000_000],
                                  mapping={"h1_zero_rad": 100}, wheel_id=wheel)[0]
              for wheel, data in events.items()}
    assert frames["front_left"]["joints"]["caster_positive_hemisphere_joint"] == -99.9
    assert plots.current("front_left", "H1", START + 200_000_000)["angle_rad"] == .1
    image = Image.new("RGB", (1920, 1080), "black")
    plots.draw(image, frames)
    assert image.getpixel((959, 500)) == (0, 0, 0)
    assert image.getpixel((960, 500)) != (0, 0, 0)
    assert image.getpixel((1919, 954)) == (0, 0, 0)
    frames["front_right"]["stamp_ns"] += 1
    with pytest.raises(ValueError, match="same replay timestamp"):
        plots.draw(image, frames)


def test_paired_plots_reject_cross_wheel_identity():
    events = paired_events()
    events["front_left"].append((START+200_000_000, wheel_envelope(1, wheel="front_right")))
    with pytest.raises(ValueError, match="Expected wheel front_left"):
        PairEncoderPlots(events, START, START+1_000_000_000, 1920, "test", "Estimated clock mapping.")
