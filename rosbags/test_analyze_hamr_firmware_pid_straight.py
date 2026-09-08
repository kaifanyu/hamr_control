"""Synthetic regression tests for the encoder-only firmware PI analyzer."""

from importlib.util import module_from_spec, spec_from_file_location
import math
from pathlib import Path
import sys

import numpy as np


SCRIPT = Path(__file__).with_name("analyze_hamr_firmware_pid_straight.py")
SPEC = spec_from_file_location("analyze_hamr_firmware_pid_straight", SCRIPT)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def synthetic_traces():
    level = 1.35
    rpm_scale = 60.0 / (2.0 * math.pi)
    tick_scale = 2263.7 / (2.0 * math.pi)

    command_t = np.arange(0.0, 7.001, 0.02)

    def command(timestamp):
        if 3.0 <= timestamp < 3.5:
            return level * (timestamp - 3.0) / 0.5
        if 3.5 <= timestamp < 5.0:
            return level
        return 0.0

    command_v = np.asarray([command(timestamp) for timestamp in command_t])

    status_t = np.arange(0.0, 7.001, 0.05)
    status = np.zeros((status_t.size, len(MODULE.STATUS_FIELDS)), dtype=float)
    for index, timestamp in enumerate(status_t):
        cmd = command(timestamp)
        target = cmd * rpm_scale
        if cmd > 0.0:
            measured_left = target * (1.0 + 0.012 * math.sin(18.0 * timestamp))
            measured_right = target * (1.0 + 0.010 * math.cos(17.0 * timestamp))
            source = 1.0
            integral_left = 0.20
            integral_right = 0.18
        else:
            measured_left = measured_right = 0.0
            source = 5.0
            integral_left = integral_right = 0.0
        ff = target * 175.0
        pid_left = 60.0 * (target - measured_left) + 20.0 * integral_left
        pid_right = 60.0 * (target - measured_right) + 20.0 * integral_right
        values = {
            "left_target_rpm": target,
            "left_measured_rpm": measured_left,
            "left_ff_pwm": ff,
            "left_pid_pwm": pid_left,
            "left_output_pwm": ff + pid_left,
            "left_error_rpm": target - measured_left,
            "left_integral_rpm_s": integral_left,
            "left_saturated": 0.0,
            "right_target_rpm": target,
            "right_measured_rpm": measured_right,
            "right_ff_pwm": ff,
            "right_pid_pwm": pid_right,
            "right_output_pwm": ff + pid_right,
            "right_error_rpm": target - measured_right,
            "right_integral_rpm_s": integral_right,
            "right_saturated": 0.0,
            "source": source,
        }
        status[index] = [values[name] for name in MODULE.STATUS_FIELDS]

    tick_t = command_t.copy()
    tick_values = np.zeros(tick_t.size, dtype=float)
    for index in range(1, tick_t.size):
        dt = tick_t[index] - tick_t[index - 1]
        tick_values[index] = tick_values[index - 1] + command_v[index - 1] * tick_scale * dt
    left_ticks = np.rint(tick_values)
    right_ticks = np.rint(1.005 * tick_values)

    diag_t = np.arange(0.0, 7.001, 0.05)
    diag = np.zeros((diag_t.size, MODULE.DIAGNOSTIC_WIDTH), dtype=float)
    left_diag_ticks = np.interp(diag_t, tick_t, left_ticks)
    right_diag_ticks = np.interp(diag_t, tick_t, right_ticks)
    width = len(MODULE.DIAGNOSTIC_WHEEL_FIELDS)
    for wheel_index, values in enumerate((left_diag_ticks, right_diag_ticks)):
        offset = wheel_index * width
        legacy = np.rint(values)
        quadrature = 2.0 * legacy
        diag[:, offset] = legacy
        diag[:, offset + 1] = quadrature
        for counter_index in range(2, 7):
            diag[:, offset + counter_index] = quadrature
        diag[:, offset + 8] = 0.0
        diag[:, offset + 9] = 0.0
        diag[:, offset + 10] = 0.0

    return {
        "left_cmd": {"t": command_t, "v": command_v},
        "right_cmd": {"t": command_t + 0.001, "v": command_v},
        "turret_cmd": {"t": command_t + 0.002, "v": np.zeros_like(command_v)},
        "left_ticks": {"t": tick_t, "v": left_ticks},
        "right_ticks": {"t": tick_t + 0.001, "v": right_ticks},
        "status": {
            "t": status_t,
            "v": status,
            "labels": [MODULE.STATUS_SCHEMA] * status_t.size,
        },
        "diagnostics": {
            "t": diag_t,
            "v": diag,
            "labels": [MODULE.DIAGNOSTIC_SCHEMA] * diag_t.size,
        },
    }


def add_synthetic_vicon(traces, radius=0.123, receipt_delay=0.159):
    source_t = np.arange(0.0, 7.001, 0.01)
    command = np.interp(
        source_t,
        traces["left_cmd"]["t"],
        traces["left_cmd"]["v"],
    )
    distance = np.zeros(source_t.size)
    distance[1:] = np.cumsum(
        0.5 * (command[1:] + command[:-1]) * np.diff(source_t) * radius
    )
    position = np.column_stack((distance, np.zeros_like(distance),
                                np.full_like(distance, 0.33)))
    quaternion = np.zeros((source_t.size, 4))
    quaternion[:, 3] = 1.0
    traces["vicon"] = {
        "source_t": source_t,
        "receipt_t": source_t + receipt_delay,
        "position": position,
        "quaternion": quaternion,
    }
    return traces


def test_synthetic_nominal_run_passes_and_plots(tmp_path):
    traces = synthetic_traces()
    report = MODULE.analyze_traces(traces, MODULE.Config())
    assert report["verdict"] == {
        "acquisition_pass": True,
        "performance_pass": True,
        "overall_pass": True,
    }
    assert report["expected"]["target_rpm"] == pytest.approx(12.89155039)
    output = tmp_path / "response.png"
    MODULE.plot_report(traces, report, output)
    assert output.stat().st_size > 10_000
    assert not report["metrics"]["opportunistic_vicon"]["available"]


def test_saturation_is_an_acquisition_failure():
    traces = synthetic_traces()
    moving = traces["status"]["v"][:, MODULE.STATUS_INDEX["source"]] == 1.0
    traces["status"]["v"][moving, MODULE.STATUS_INDEX["left_saturated"]] = 1.0
    report = MODULE.analyze_traces(traces, MODULE.Config())
    assert not report["checks"]["acquisition"]["no_saturation"]["pass"]
    assert not report["verdict"]["acquisition_pass"]


def test_stale_but_valid_vicon_is_usable_offline_and_does_not_change_verdict(tmp_path):
    traces = add_synthetic_vicon(synthetic_traces())
    report = MODULE.analyze_traces(traces, MODULE.Config())
    vicon = report["metrics"]["opportunistic_vicon"]
    assert report["verdict"]["overall_pass"]
    assert vicon["usable"]
    assert vicon["stale_for_live_control"]
    assert vicon["live_fresh_fraction"] == 0.0
    assert vicon["receipt_age_median_s"] == pytest.approx(0.159)
    assert vicon["ground_speed_m_s"] == pytest.approx(0.123 * 1.35, rel=2e-3)
    assert vicon["effective_radius_m"] == pytest.approx(0.123 / 1.0025, rel=4e-3)
    assert vicon["lag_usable"]
    output = tmp_path / "response_with_vicon.png"
    MODULE.plot_report(traces, report, output)
    assert output.stat().st_size > 10_000


def test_nonmonotonic_vicon_source_stamps_fail_closed_without_affecting_firmware():
    traces = add_synthetic_vicon(synthetic_traces())
    traces["vicon"]["source_t"][430] = traces["vicon"]["source_t"][429]
    report = MODULE.analyze_traces(traces, MODULE.Config())
    vicon = report["metrics"]["opportunistic_vicon"]
    assert report["verdict"]["overall_pass"]
    assert not vicon["usable"]
    assert not vicon["checks"]["source_stamp_monotonic"]["pass"]
    assert vicon["ground_speed_m_s"] is None
    assert vicon["effective_radius_m"] is None


def test_invalid_vicon_geometry_is_clearly_unusable():
    traces = add_synthetic_vicon(synthetic_traces())
    traces["vicon"]["position"][430, 2] = 0.60
    report = MODULE.analyze_traces(traces, MODULE.Config())
    vicon = report["metrics"]["opportunistic_vicon"]
    assert report["verdict"]["overall_pass"]
    assert not vicon["usable"]
    assert not vicon["checks"]["z_envelope"]["pass"]
    assert any("z_envelope" in reason for reason in vicon["reasons"])


# Imported last to keep the standalone analyzer free of a pytest dependency.
import pytest
