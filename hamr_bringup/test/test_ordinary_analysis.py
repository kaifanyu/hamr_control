import csv
import json
from pathlib import Path
import pytest

from test_ordinary_history import make_board, write_run, load_history
from ordinary_caster.analysis import summarize, write_analysis


def test_independent_fault_counts_and_raw_exact_csv(tmp_path):
    board = make_board(calibrated=False)
    board[1]["samples"][2]["swivel"].update(flags=1 | 128, calibrated_angle_rad=None)
    history = load_history(write_run(tmp_path / "run", board))
    report = write_analysis(history, tmp_path / "analysis", plots=False)
    swivel, wheel = report["streams"]
    assert swivel["valid_samples"] == 4
    assert wheel["valid_samples"] == 5
    assert swivel["flag_counts"]["parity_error"] == 1
    assert wheel["flag_counts"]["parity_error"] == 0
    assert wheel["calibrated_travel_rad"] == 0
    assert wheel["first_raw_count"] == str((1 << 63) - 100)
    assert wheel["observed_count_span"] == "4"
    rows = list(csv.DictReader((tmp_path / "analysis/history.csv").open()))
    assert len(rows) == 10
    assert rows[5]["raw_count"] == str((1 << 63) - 100)
    assert int(rows[0]["t_us"]) - int(rows[5]["t_us"]) == 100
    assert int(rows[0]["mapped_epoch_ns"]) - int(rows[5]["mapped_epoch_ns"]) == 100000
    assert rows[0]["mapping_usable"] == "1"
    assert json.loads((tmp_path / "analysis/quality.json").read_text())["caster_type"] == "ordinary"


def test_travel_excludes_gap_and_unverified_turns(tmp_path):
    board = make_board()
    del board[1]["samples"][2]
    history = load_history(write_run(tmp_path, board))
    report = summarize(history)["streams"][0]
    assert report["sequence_missing"] == 1
    assert report["segments"] == 2
    assert abs(report["calibrated_travel_rad"] - .2) < 1e-9
    assert not report["turn_history_complete"]


def test_poor_map_retained_for_review_without_claiming_alignment(tmp_path):
    board = make_board()
    board[2]["uncertainty_ns"] = 100_000_000
    history = load_history(write_run(tmp_path / "run", board))
    write_analysis(history, tmp_path / "analysis", plots=False)
    rows = list(csv.DictReader((tmp_path / "analysis/history.csv").open()))
    assert rows[0]["time_basis"] == "separate_device_clocks"
    assert rows[0]["mapped_epoch_ns"] == str(board[2]["offset_ns"] + 1_000_000_000)
    assert rows[0]["mapping_usable"] == "0"
    assert rows[0]["timing_uncertainty_ns"] == "100000000"


def test_unwrapped_unknown_plot_never_connects_across_wrapped_angle(tmp_path, monkeypatch):
    pytest.importorskip("matplotlib")
    from matplotlib.axes import Axes
    from ordinary_caster.analysis import plot_history
    board = make_board(calibrated=False)
    for index, sample in enumerate(board[1]["samples"]):
        sample["swivel"].update(flags=1 | 2 | 256, angle_rad=None,
                                calibrated_angle_rad=3.1 if index % 2 == 0 else -3.1)
    calls = []
    original = Axes.plot
    def capture(axis, *args, **kwargs):
        if len(args) >= 2 and isinstance(args[1], list) and args[1] == [3.1, -3.1, 3.1, -3.1, 3.1]:
            calls.append(kwargs)
        return original(axis, *args, **kwargs)
    monkeypatch.setattr(Axes, "plot", capture)
    history = load_history(write_run(tmp_path / "run", board))
    plot_history(history, tmp_path / "plot.png")
    assert calls and all(call["linestyle"] == "None" for call in calls)
