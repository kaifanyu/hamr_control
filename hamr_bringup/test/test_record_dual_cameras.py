"""Exercise recorder ownership and shutdown without cameras or ROS processes."""

from datetime import datetime
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path
import signal
import sys
from types import ModuleType, SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/record_dual_cameras"


@pytest.fixture
def recorder(tmp_path, monkeypatch):
    loader = SourceFileLoader("dual_camera_wrapper_under_test", str(SCRIPT))
    spec = spec_from_loader(loader.name, loader)
    module = module_from_spec(spec)
    loader.exec_module(module)
    # The wrapper adds its checkout to sys.path; keep that mutation test-local.
    monkeypatch.setattr(sys, "path", list(sys.path))
    repository = tmp_path / "caster repository"
    (repository / "dualcam").mkdir(parents=True)
    (repository / "dualcam/capture.py").touch()
    (repository / "config").mkdir()
    config_path = repository / "config/rig.yaml"
    config_path.write_text("schema_version: 1\n")

    capture = ModuleType("dualcam.capture")
    config = {"schema_version": 1}

    def load_config(path):
        path.read_text()  # Preserve the real missing-config failure boundary.
        return config, path

    capture.load_capture_config = load_config
    capture.record_session = lambda *args, **kwargs: pytest.fail("unexpected capture")
    package = ModuleType("dualcam")
    package.__path__ = []
    package.capture = capture
    monkeypatch.setitem(sys.modules, "dualcam", package)
    monkeypatch.setitem(sys.modules, "dualcam.capture", capture)

    original_handlers = {signal.SIGINT: object(), signal.SIGTERM: object()}
    handlers = original_handlers.copy()

    def install_handler(signum, handler):
        previous = handlers[signum]
        handlers[signum] = handler
        return previous

    monkeypatch.setattr(module, "signal", SimpleNamespace(
        SIGINT=signal.SIGINT, SIGTERM=signal.SIGTERM, signal=install_handler))
    return SimpleNamespace(module=module, capture=capture, config=config,
                           repository=repository, config_path=config_path,
                           output=tmp_path / "camera output", handlers=handlers,
                           original_handlers=original_handlers)


def arguments(recorder, *extra):
    return ["--repository", str(recorder.repository),
            "--output-dir", str(recorder.output), *extra]


def completed_session(status="complete"):
    return {"status": status,
            "stats": {name: {"frames": 12, "observed_fps": 30.0}
                      for name in ("c920", "brio101")},
            "cameras": {name: {"warnings": []} for name in ("c920", "brio101")}}


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_continuous_capture_finalizes_on_either_shutdown_signal(recorder, signum):
    def capture(config, path, output, duration, mode, stop, **kwargs):
        assert config is recorder.config
        assert path == recorder.config_path
        assert output.parent == recorder.output
        assert not output.exists()
        assert duration is None
        assert mode == "motion"
        assert kwargs == {"max_video_file_bytes": 1024**3}
        assert not stop.is_set()
        recorder.handlers[signum](signum, None)
        assert stop.is_set()
        return completed_session("interrupted")

    recorder.capture.record_session = capture
    assert recorder.module.main(arguments(recorder, "--ros-args", "-r", "__node:=ignored")) == 0
    assert recorder.handlers == recorder.original_handlers


def test_optional_duration_config_and_segment_size_reach_capture(recorder, tmp_path):
    config_path = tmp_path / "separate rig.yaml"
    config_path.write_text("schema_version: 1\n")

    def capture(config, path, output, duration, mode, stop, **kwargs):
        assert path == config_path
        assert duration == 2.5
        assert mode == "motion"
        assert kwargs["max_video_file_bytes"] == 64 * 1024**2
        return completed_session()

    recorder.capture.record_session = capture
    assert recorder.module.main(arguments(
        recorder, "--config", str(config_path), "--duration", "2.5", "--segment-mib", "64")) == 0
    assert recorder.handlers == recorder.original_handlers


def test_concurrent_name_components_preserve_previous_recordings(recorder, monkeypatch):
    # Freeze wall time so uniqueness depends on the explicit collision component.
    monkeypatch.setattr(recorder.module, "datetime", SimpleNamespace(
        now=lambda: datetime(2026, 9, 16, 12, 0, 0)))
    identifiers = iter(("a" * 32, "b" * 32))
    monkeypatch.setattr(recorder.module, "uuid4", lambda: SimpleNamespace(hex=next(identifiers)))
    outputs = []

    def capture(config, path, output, duration, mode, stop, **kwargs):
        output.mkdir(parents=True, exist_ok=False)
        (output / "preserved.txt").write_text(str(len(outputs)))
        outputs.append(output)
        return completed_session()

    recorder.capture.record_session = capture
    assert recorder.module.main(arguments(recorder)) == 0
    assert recorder.module.main(arguments(recorder)) == 0
    assert outputs[0] != outputs[1]
    assert [(out / "preserved.txt").read_text() for out in outputs] == ["0", "1"]


@pytest.mark.parametrize("missing", ["repository", "config"])
def test_missing_inputs_fail_before_capture_or_signal_install(recorder, missing, capsys):
    if missing == "repository":
        (recorder.repository / "dualcam/capture.py").unlink()
    else:
        recorder.config_path.unlink()
    assert recorder.module.main(arguments(recorder)) == 1
    assert recorder.handlers == recorder.original_handlers
    assert not recorder.output.exists()
    assert "failed" in capsys.readouterr().err.lower()


def test_capture_failure_preserves_partial_files_and_restores_handlers(recorder, capsys):
    def capture(config, path, output, duration, mode, stop, **kwargs):
        output.mkdir(parents=True)
        (output / "session.json").write_text('{"status": "failed"}\n')
        raise RuntimeError("camera disconnected")

    recorder.capture.record_session = capture
    assert recorder.module.main(arguments(recorder)) == 1
    assert recorder.handlers == recorder.original_handlers
    manifests = list(recorder.output.glob("*/session.json"))
    assert len(manifests) == 1
    assert '"failed"' in manifests[0].read_text()
    error = capsys.readouterr().err
    assert "camera disconnected" in error
    assert "camera recording is no longer active" in error


@pytest.mark.parametrize("args", [
    ["--duration", "0"], ["--duration", "nan"], ["--duration", "inf"],
    ["--segment-mib", "0.5"], ["--segment-mib", "4096"],
])
def test_invalid_duration_or_avi_limits_are_rejected_before_capture(recorder, args):
    with pytest.raises(SystemExit) as error:
        recorder.module.main(arguments(recorder, *args))
    assert error.value.code == 2
    assert recorder.handlers == recorder.original_handlers
