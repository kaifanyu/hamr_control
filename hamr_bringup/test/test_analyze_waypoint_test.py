"""Offline tests for waypoint bag discovery and analysis wiring."""

import importlib.util
from importlib.machinery import SourceFileLoader
import os
from pathlib import Path
import sys

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_DIR / "scripts" / "analyze_waypoint_test"
SPEC = importlib.util.spec_from_loader(
    "analyze_waypoint_test",
    loader=SourceFileLoader("analyze_waypoint_test", str(SCRIPT)),
)
analyzer = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = analyzer
SPEC.loader.exec_module(analyzer)


def make_bag(root: Path, name: str, reference_count: int = 3500) -> Path:
    bag = root / name
    bag.mkdir()
    storage = bag / "data_0.mcap"
    storage.write_bytes(b"not-empty")
    counts = {
        topic: reference_count if topic == analyzer.REFERENCE_TOPIC else 1
        for topic in analyzer.REQUIRED_TOPIC_COUNTS
    }
    metadata = {
        "rosbag2_bagfile_information": {
            "storage_identifier": "mcap",
            "relative_file_paths": [storage.name],
            "topics_with_message_count": [
                {
                    "topic_metadata": {"name": topic},
                    "message_count": count,
                }
                for topic, count in counts.items()
            ],
        }
    }
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump(metadata), encoding="utf-8"
    )
    return bag


def completion(*, span=69.98, zero=2.98, count=3500, extent=True):
    return analyzer.ReferenceCompletion(count, span, zero, 0.0, 0.0, extent)


def test_default_pattern_accepts_all_auto_labeled_waypoint_speeds():
    assert analyzer.DEFAULT_PATTERN == (
        "hamr_waypoint_simple_vicon_noguard_v*_*"
    )
    assert Path(
        "hamr_waypoint_simple_vicon_noguard_v015_20260820_190000"
    ).match(analyzer.DEFAULT_PATTERN)


def test_latest_closed_bag_is_selected_even_when_run_is_partial(tmp_path):
    older = make_bag(
        tmp_path,
        "hamr_waypoint_simple_vicon_noguard_v020_20260820_190000",
    )
    newer = make_bag(
        tmp_path,
        "hamr_waypoint_simple_vicon_noguard_v020_20260820_191000",
    )
    os.utime(older / "metadata.yaml", (10, 10))
    os.utime(newer / "metadata.yaml", (20, 20))

    selected, result, rejected = analyzer.select_latest_analyzable(
        tmp_path,
        completion_reader=lambda bag: completion(span=31.0, zero=0.0)
        if bag == newer
        else completion(),
    )

    assert selected == newer.resolve()
    assert not result.complete
    assert rejected == []


def test_metadata_preflight_accepts_partial_but_rejects_missing_data(tmp_path):
    partial = make_bag(tmp_path, "partial", reference_count=900)
    missing = make_bag(tmp_path, "missing", reference_count=0)

    valid, _ = analyzer.metadata_preflight(partial)
    missing_valid, reason = analyzer.metadata_preflight(missing)

    assert valid
    assert not missing_valid
    assert analyzer.REFERENCE_TOPIC in reason


def test_analysis_commands_write_expected_artifacts_under_bag(monkeypatch, tmp_path):
    bag = tmp_path / "bag"
    output = bag / "analysis"
    monkeypatch.setattr(
        analyzer, "find_helper", lambda name: Path("/installed") / name
    )

    commands = analyzer.analysis_commands(bag, output)

    flattened = [part for command in commands for part in command]
    assert str(output / "path_reference.png") in flattened
    assert str(output / "turn_comparison.png") in flattened
    assert str(output / "metrics.json") in flattened
    assert str(output / "wheel_velocity_compare.png") in flattened
    assert str(output / "forward_speed.png") in flattened
    assert str(output / "base_trace.csv") in flattened
    assert str(output / "actuator") in flattened
    assert Path(commands[0][1]).name == "analyze_hamr_vicon_straight.py"
    assert Path(commands[1][1]).name == "plot_hamr_actuators.py"


def test_reference_completion_requires_full_span_zero_tail_and_endpoint():
    assert completion().complete
    assert not completion(zero=2.7).complete
    assert not completion(extent=False).complete
    assert not analyzer.ReferenceCompletion(
        3500, 70.0, 3.0, 0.01, 0.0, True
    ).complete


def test_explicit_partial_bag_runs_only_offline_helpers(
    monkeypatch, tmp_path, capsys
):
    bag = make_bag(tmp_path, "explicit_partial", reference_count=900)
    output = bag / "analysis"
    calls = []
    artifacts = (
        "path_reference.png",
        "turn_comparison.png",
        "metrics.json",
        "wheel_velocity_compare.png",
        "forward_speed.png",
        "base_trace.csv",
        "actuator_overview.png",
        "actuator_stop_zoom.png",
    )
    monkeypatch.setattr(
        analyzer, "read_reference_completion", lambda _bag: completion(zero=0.0)
    )
    monkeypatch.setattr(
        analyzer,
        "analysis_commands",
        lambda _bag, _output: [["offline-tracking"], ["offline-actuators"]],
    )

    def fake_run(command, check):
        assert check
        calls.append(command)
        output.mkdir(exist_ok=True)
        for name in artifacts:
            (output / name).write_bytes(b"artifact")

    monkeypatch.setattr(analyzer.subprocess, "run", fake_run)
    monkeypatch.setattr(analyzer.sys, "argv", [str(SCRIPT), str(bag)])

    assert analyzer.main() == 0
    assert calls == [["offline-tracking"], ["offline-actuators"]]
    assert "incomplete reference lifecycle" in capsys.readouterr().err
