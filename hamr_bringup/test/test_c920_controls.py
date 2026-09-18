"""Exercise fixed camera configuration without a camera or OpenCV dependency."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "c920_controls.py"
SPEC = importlib.util.spec_from_file_location("c920_controls_under_test", SCRIPT)
controls = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(controls)

CV2 = SimpleNamespace(
    CAP_PROP_FOURCC=6, CAP_PROP_FRAME_WIDTH=3, CAP_PROP_FRAME_HEIGHT=4,
    CAP_PROP_FPS=5,
    VideoWriter_fourcc=lambda *chars: sum(ord(char) << (8 * i)
                                        for i, char in enumerate(chars)),
)


class CameraControls:
    """Small stateful stand-in for the V4L2 text interface."""

    def __init__(self, legacy=False):
        self.focus_auto = "focus_auto" if legacy else "focus_automatic_continuous"
        self.exposure_auto = "exposure_auto" if legacy else "auto_exposure"
        self.dynamic_fps = "exposure_auto_priority" if legacy else "exposure_dynamic_framerate"
        self.exposure = "exposure_absolute" if legacy else "exposure_time_absolute"
        self.limits = {
            "focus_absolute": (0, 250, 5), self.exposure: (3, 2047, 1),
            "gain": (0, 255, 1), "zoom_absolute": (100, 500, 1),
            "pan_absolute": (-36000, 36000, 3600),
            "tilt_absolute": (-36000, 36000, 3600),
        }
        self.values = {name: limits[0] for name, limits in self.limits.items()}
        self.values.update({self.focus_auto: 1, self.exposure_auto: 3,
                            self.dynamic_fps: 1})
        self.mutations = []

    def __call__(self, device, option):
        assert device == "/dev/fake-camera"
        if option == "--list-ctrls":
            lines = [f"{name} 0x009a0901 (bool) : value={self.values[name]}"
                     for name in (self.focus_auto, self.exposure_auto, self.dynamic_fps)]
            lines.extend(f"{name} 0x009a090a (int) : min={low} max={high} step={step}"
                         for name, (low, high, step) in self.limits.items())
            return "\n".join(lines)
        if option.startswith("--set-ctrl="):
            update = {name: int(value) for name, value in
                      (pair.split("=") for pair in option.split("=", 1)[1].split(","))}
            # Manual controls must be written only after automatic modes are off.
            if "focus_absolute" in update:
                assert self.values[self.focus_auto] == 0
                assert self.values[self.exposure_auto] == 1
            self.mutations.append(update)
            self.values.update(update)
            return ""
        assert option.startswith("--get-ctrl=")
        names = option.split("=", 1)[1].split(",")
        return "\n".join(f"{name}: {self.values[name]}" for name in names)


class Capture:
    def __init__(self, hardware, *, fps=15.0, shape=(1080, 1920, 3), drift=False,
                 fourcc=None):
        self.hardware = hardware
        self.fps = fps
        self.shape = shape
        self.drift = drift
        self.fourcc = CV2.VideoWriter_fourcc(*"MJPG") if fourcc is None else fourcc
        self.reads = 0

    def set(self, _prop, _value):
        return True

    def get(self, prop):
        return self.fps if prop == CV2.CAP_PROP_FPS else self.fourcc

    def read(self):
        self.reads += 1
        if self.reads == 1:
            # Real C920 firmware can reset controls as streaming starts.
            self.hardware.values[self.hardware.focus_auto] = 1
            self.hardware.values[self.hardware.exposure_auto] = 3
        if self.drift and self.reads == 3:
            self.hardware.values[self.hardware.focus_auto] = 1
        return True, SimpleNamespace(shape=self.shape)


def configure(capture):
    return controls.configure_capture(capture, CV2, "/dev/fake-camera",
                                      1920, 1080, 15.0, 50, 77, 180, 100)


@pytest.mark.parametrize("legacy", [False, True], ids=["modern-names", "legacy-names"])
def test_manual_controls_support_v4l2_aliases(monkeypatch, legacy):
    hardware = CameraControls(legacy)
    monkeypatch.setattr(controls, "v4l2", hardware)
    expected = controls.configure_controls("/dev/fake-camera", 50, 77, 180, 100)
    assert expected == {
        hardware.focus_auto: 0, hardware.exposure_auto: 1, hardware.dynamic_fps: 0,
        "focus_absolute": 50, hardware.exposure: 77, "gain": 180,
        "zoom_absolute": 100, "pan_absolute": 0, "tilt_absolute": 0,
    }
    assert hardware.values == expected


@pytest.mark.parametrize("focus", [-5, 255, 52], ids=["below-range", "above-range", "off-step"])
def test_invalid_focus_rejected_before_any_control_mutation(monkeypatch, focus):
    hardware = CameraControls()
    monkeypatch.setattr(controls, "v4l2", hardware)
    with pytest.raises(RuntimeError, match="Invalid focus_absolute"):
        controls.configure_controls("/dev/fake-camera", focus, 77, 180, 100)
    assert hardware.mutations == []


def test_manual_controls_survive_stream_start_reset(monkeypatch):
    hardware = CameraControls()
    monkeypatch.setattr(controls, "v4l2", hardware)
    expected = configure(Capture(hardware))
    assert hardware.values == expected
    assert hardware.values[hardware.focus_auto] == 0
    assert hardware.values[hardware.exposure_auto] == 1


def test_control_drift_after_stream_start_is_rejected(monkeypatch):
    hardware = CameraControls()
    monkeypatch.setattr(controls, "v4l2", hardware)
    with pytest.raises(RuntimeError, match="Camera controls did not stay fixed.*focus"):
        configure(Capture(hardware, drift=True))


def test_wrong_resolution_is_rejected(monkeypatch):
    hardware = CameraControls()
    monkeypatch.setattr(controls, "v4l2", hardware)
    with pytest.raises(RuntimeError, match="Camera delivered 640x480.*requested 1920x1080"):
        configure(Capture(hardware, shape=(480, 640, 3)))


@pytest.mark.parametrize("fps", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_negotiated_fps_is_rejected_before_streaming(monkeypatch, fps):
    hardware = CameraControls()
    monkeypatch.setattr(controls, "v4l2", hardware)
    capture = Capture(hardware, fps=fps)
    with pytest.raises(RuntimeError, match="Camera negotiated"):
        configure(capture)
    assert capture.reads == 0
    assert hardware.mutations == []


def test_non_mjpeg_capture_is_rejected(monkeypatch):
    hardware = CameraControls()
    monkeypatch.setattr(controls, "v4l2", hardware)
    capture = Capture(hardware, fourcc=CV2.VideoWriter_fourcc(*"YUYV"))
    with pytest.raises(RuntimeError, match="required native MJPEG"):
        configure(capture)
    assert hardware.mutations == []
