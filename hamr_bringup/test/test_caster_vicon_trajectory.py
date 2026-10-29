"""Numerical motion estimation and discontinuity regressions for both casters."""
import copy
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from caster_vicon_trajectory import estimate_trajectories, estimation_settings, linear_fit
from prepare_caster_vicon_comparison import DEFAULT_QUALITY, ROLES, WHEELS


def histories():
    aligned, streams = {}, {}
    for wheel_index, wheel in enumerate(WHEELS):
        direction = 1 if wheel_index == 0 else -1
        aligned[wheel] = []
        for i in range(101):
            t = i / 100
            # A moving marker with a constant world-z angular velocity.
            q = [0., 0., math.sin(.3*t), math.cos(.3*t)]
            if i % 2:
                q = [-value for value in q]
            row = dict(bag_elapsed_ns=i*10_000_000, bag_time_s=t,
                       fork_orientation_continuity_ok=True, fork_segment=0)
            row.update({f"fork_{axis}_m": value for axis, value in zip("xyz", [direction*.2*t, .1*t, .3])})
            row.update({f"fork_q{axis}": value for axis, value in zip("xyzw", q)})
            row.update({f"{role.lower()}_fresh": True for role in ROLES})
            aligned[wheel].append(row)
        for j, role in enumerate(ROLES):
            rate = direction * (j+1)
            streams[wheel, role] = [dict(time_ns=i*5_000_000, change_rad=rate*i/200)
                                    for i in range(201)]
    return aligned, streams


def estimate(rows, streams, **settings):
    return estimate_trajectories(rows, streams, DEFAULT_QUALITY, settings)


def test_known_translation_rotation_and_independent_encoder_rates():
    rows, streams = histories()
    before = copy.deepcopy(rows)
    report = estimate(rows, streams)
    for index, wheel in enumerate(WHEELS):
        sign = 1 if index == 0 else -1
        for row, raw in zip(rows[wheel], before[wheel]):
            assert all(row[key] == value for key, value in raw.items())
            assert row["fork_estimate_valid"]
            assert row["fork_vx_m_s"] == pytest.approx(sign*.2)
            assert row["fork_vy_m_s"] == pytest.approx(.1)
            assert row["fork_vz_m_s"] == pytest.approx(0., abs=1e-12)
            assert row["fork_wz_rad_s"] == pytest.approx(.6)
            assert row["fork_angular_speed_rad_s"] == pytest.approx(.6)
            for j, role in enumerate(ROLES):
                assert row[f"{role.lower()}_estimated_rate_rad_s"] == pytest.approx(sign*(j+1))
        assert report["wheels"][wheel]["estimated_distance_over_valid_segments_m"] == pytest.approx(math.hypot(.2, .1))


def test_masked_orientation_and_position_jumps_never_get_fitted_across():
    rows, streams = histories()
    for i, row in enumerate(rows["front_left"]):
        if 45 <= i <= 55:
            row["fork_orientation_continuity_ok"] = False
        if i >= 50:
            row["fork_x_m"] += 5
            row["fork_segment"] = 1
    report = estimate(rows, streams)
    left = rows["front_left"]
    assert report["wheels"]["front_left"]["position_jump_count"] == 1
    assert all(not row["fork_estimate_valid"] for row in left[40:61])
    assert left[39]["motion_window_end_ns"] < left[40]["bag_elapsed_ns"]
    assert left[61]["motion_window_start_ns"] > left[60]["bag_elapsed_ns"]
    assert report["wheels"]["front_left"]["estimated_distance_over_valid_segments_m"] < .2
    assert all(row["fork_speed_m_s"] == pytest.approx(math.hypot(.2, .1))
               for row in left if row["fork_estimate_valid"])
    assert report["wheels"]["front_right"]["segments"] == 1


def test_pose_gaps_nonfinite_samples_and_short_segments_have_no_velocity():
    rows, streams = histories()
    left = rows["front_left"]
    left[25]["fork_x_m"] = float("nan")
    rows["front_left"] = left[:40] + left[70:]
    rows["front_right"] = rows["front_right"][:2]
    report = estimate(rows, streams)
    assert not left[25]["fork_estimate_valid"]
    assert left[24]["motion_window_end_ns"] < left[25]["bag_elapsed_ns"]
    assert left[39]["motion_window_end_ns"] < left[70]["bag_elapsed_ns"]
    assert report["wheels"]["front_right"]["estimated_rows"] == 0
    assert report["wheels"]["front_right"]["estimated_distance_over_valid_segments_m"] is None


def test_encoder_gaps_invalid_values_and_staleness_do_not_create_rates():
    rows, streams = histories()
    streams["front_left", "H1"] = [sample for sample in streams["front_left", "H1"]
                                    if not 480_000_000 < sample["time_ns"] < 520_000_000]
    streams["front_left", "H2"][100]["change_rad"] = float("nan")
    rows["front_left"][50]["center_fresh"] = False
    estimate(rows, streams)
    row = rows["front_left"][50]
    assert row["fork_estimate_valid"]
    assert not row["all_encoder_rate_estimates_valid"]
    assert all(row[f"{role.lower()}_estimated_rate_rad_s"] is None for role in ROLES)
    assert rows["front_right"][50]["all_encoder_rate_estimates_valid"]


def test_fit_preserves_integer_epoch_precision():
    epoch = 1_793_256_992_820_270_894
    offset, slope = linear_fit([epoch, epoch+10, epoch+20], [0., 1., 2.], epoch+10)
    assert offset == pytest.approx(1.)
    assert slope == pytest.approx(1e8)


@pytest.mark.parametrize("settings", [dict(window_s=0), dict(min_samples=2),
                                      dict(min_samples=3.5), dict(min_span_s=1),
                                      dict(window_s=float("nan")), dict(unknown=1)])
def test_invalid_estimation_settings_fail(settings):
    with pytest.raises(ValueError):
        estimation_settings(settings)
