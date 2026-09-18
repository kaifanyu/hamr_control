"""Verified V4L2 settings shared by C920 photo and video capture."""

import math
import re
import subprocess


DEFAULT_DEVICE = "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_05577B1F-video-index0"
DEFAULT_WIDTH = 1920
DEFAULT_HEIGHT = 1080
DEFAULT_FPS = 15.0  # Sustained native 1080p MP4 encoding on this Raspberry Pi.
DEFAULT_FOCUS = 50  # Current lens position; calibration focus was not recorded.
DEFAULT_EXPOSURE = 77  # V4L2 units of 100 us: 7.7 ms (C920 rounds 83 to 77).
DEFAULT_GAIN = 180
DEFAULT_ZOOM = 100


def v4l2(device, *options):
    try:
        result = subprocess.run(
            ["v4l2-ctl", "--device", str(device), *options],
            capture_output=True, text=True, timeout=10, check=False,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("v4l2-ctl is missing; install v4l-utils") from exc
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f"Timed out configuring {device}") from exc
    if result.returncode:
        raise RuntimeError(f"v4l2-ctl failed: {result.stderr.strip() or result.stdout.strip()}")
    return result.stdout


def read_controls(device, names):
    output = v4l2(device, "--get-ctrl=" + ",".join(names))
    return {name: int(value) for name, value in re.findall(
        r"^\s*(\w+):\s*(-?\d+)", output, re.MULTILINE)}


def verify_controls(device, expected):
    actual = read_controls(device, expected)
    mismatches = [f"{name}: requested {value}, got {actual.get(name)!r}"
                  for name, value in expected.items() if actual.get(name) != value]
    if mismatches:
        raise RuntimeError("Camera controls did not stay fixed: " + "; ".join(mismatches))
    return actual


def configure_controls(device, focus, exposure, gain, zoom):
    """Disable auto modes before setting manual values; reject silent clamping."""
    listing = v4l2(device, "--list-ctrls")
    available = {}
    for line in listing.splitlines():
        match = re.match(r"\s*(\w+)\s+0x[0-9a-f]+\s+\([^)]*\)\s*:\s*(.*)", line)
        if match:
            available[match[1]] = {
                key: int(value) for key, value in re.findall(
                    r"(min|max|step)=(-?\d+)", match[2])}

    def control(*aliases):
        for name in aliases:
            if name in available:
                return name
        raise RuntimeError("Camera lacks required control: " + "/".join(aliases))

    automatic = {
        control("focus_automatic_continuous", "focus_auto"): 0,
        control("auto_exposure", "exposure_auto"): 1,
        control("exposure_dynamic_framerate", "exposure_auto_priority"): 0,
    }
    manual = {
        control("focus_absolute"): focus,
        control("exposure_time_absolute", "exposure_absolute"): exposure,
        control("gain"): gain,
        control("zoom_absolute"): zoom,
        control("pan_absolute"): 0,
        control("tilt_absolute"): 0,
    }
    # Validate every value before changing any device settings.
    for name, value in manual.items():
        limits = available[name]
        if (not limits["min"] <= value <= limits["max"]
                or (value - limits["min"]) % limits.get("step", 1)):
            raise RuntimeError(f"Invalid {name}={value}; device limits: {limits}")
    for settings in (automatic, manual):
        v4l2(device, "--set-ctrl=" + ",".join(f"{key}={value}" for key, value in settings.items()))
    expected = {**automatic, **manual}
    verify_controls(device, expected)
    return expected


def configure_capture(capture, cv2, device, width, height, fps, focus, exposure, gain, zoom):
    if exposure * 0.0001 > 1.0 / fps:
        raise RuntimeError("Exposure is longer than the requested frame interval")
    # Native MJPEG permits C920 1080p30 over USB; do not resize or crop frames.
    for prop, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")),
                        (cv2.CAP_PROP_FRAME_WIDTH, width),
                        (cv2.CAP_PROP_FRAME_HEIGHT, height),
                        (cv2.CAP_PROP_FPS, fps)):
        if not capture.set(prop, value):
            raise RuntimeError(f"Camera rejected capture property {prop}={value}")
    actual_fps = capture.get(cv2.CAP_PROP_FPS)
    if not math.isfinite(actual_fps) or actual_fps <= 0 or abs(actual_fps - fps) > 0.1:
        raise RuntimeError(f"Camera negotiated {actual_fps:g} FPS instead of {fps:g}")
    # C920 firmware may reset controls at stream start. Start first, then apply
    # manual settings, and discard buffered frames before accepting images.
    ok, frame = capture.read()
    if not ok or frame is None:
        raise RuntimeError(f"Could not start the camera stream on {device}")
    actual_fourcc = int(capture.get(cv2.CAP_PROP_FOURCC))
    if actual_fourcc != cv2.VideoWriter_fourcc(*"MJPG"):
        raise RuntimeError("Camera did not negotiate the required native MJPEG format")
    controls = configure_controls(device, focus, exposure, gain, zoom)
    for _ in range(15):
        ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError(f"Could not read a warm-up frame from {device}")
        if frame.shape[:2] != (height, width):
            raise RuntimeError(f"Camera delivered {frame.shape[1]}x{frame.shape[0]}, "
                               f"requested {width}x{height}; calibration would not match")
    verify_controls(device, controls)
    return controls
