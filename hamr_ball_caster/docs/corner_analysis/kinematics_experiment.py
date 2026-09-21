#!/usr/bin/env python3
"""Offline analytical audit; does not import ROS, publish, or modify controllers.

Extracts the actual inverse methods by AST and compares with independently
derived forward kinematics. Then integrates exact planar differential-drive
motion during each held wheel command through one 90 degree corner. This is a
counterfactual mechanism experiment, not a fitted physical vehicle model.
"""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]


def extract_function(relative_path, name, class_name=None):
    tree = ast.parse((REPO / relative_path).read_text())
    if class_name:
        tree = next(node for node in tree.body
                    if isinstance(node, ast.ClassDef) and node.name == class_name)
    function = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = {"np": np, "math": math}
    exec(compile(ast.Module(body=[function], type_ignores=[]),
                 str(relative_path), "exec"), namespace)
    return namespace[name]


def inverse(vx, vy, yaw, r, a, b):
    forward = math.cos(yaw) * vx + math.sin(yaw) * vy
    lateral = -math.sin(yaw) * vx + math.cos(yaw) * vy
    omega = lateral / b
    return np.array([(forward + a * omega) / r,
                     (forward - a * omega) / r])  # right, left


def forward(wheels, yaw, r, a, b):
    right, left = wheels
    forward_speed = r * (right + left) / 2
    omega = r * (right - left) / (2 * a)
    return np.array([math.cos(yaw) * forward_speed - b * math.sin(yaw) * omega,
                     math.sin(yaw) * forward_speed + b * math.cos(yaw) * omega,
                     omega])


def audit():
    hamr = extract_function("hamr_control/hamr_control/hamr_controller.py",
                            "compute_velocities", "HamrControlNode")
    compa = extract_function("compa_control_py/compa_control_py/compa_controller.py",
                             "compute_velocities", "CompaControlNode")
    adapter = extract_function("hamr_ball_caster/scripts/run_waypoint_sim.py", "inverse_drive")
    rng = np.random.default_rng(20260920)
    maximum = {"hamr_forward_residual": 0., "compa_forward_residual": 0.,
               "adapter_forward_residual": 0., "hamr_adapter_wheel_difference": 0.}
    for _ in range(10000):
        r, a, b = rng.uniform(.05, .2), rng.uniform(.1, .5), rng.uniform(.1, .5)
        yaw = rng.uniform(-math.pi, math.pi)
        target = rng.uniform(-1., 1., 3)
        config = {"r_wheel": r, "a_wheel": a, "b_wheel": b}
        u = hamr(SimpleNamespace(hamr_config=config, use_diff_drive=False), target, yaw)
        actual = forward(u[:2], yaw, r, a, b)
        actual[2] += u[2]
        maximum["hamr_forward_residual"] = max(maximum["hamr_forward_residual"],
                                                float(np.max(np.abs(actual-target))))
        u5 = compa(SimpleNamespace(compa_config=config),
                   np.array([target[0], target[1], .1, -.2, target[2]]), yaw)
        actual5 = forward(u5[:2], yaw, r, a, b)
        actual5[2] += u5[4]
        maximum["compa_forward_residual"] = max(maximum["compa_forward_residual"],
                                                 float(np.max(np.abs(actual5-target))))
        left, right, omega = adapter(target[0], target[1], yaw, r, a, b)
        actual2 = forward((right, left), yaw, r, a, b)
        maximum["adapter_forward_residual"] = max(maximum["adapter_forward_residual"],
                                                   float(np.max(np.abs(actual2[:2]-target[:2]))))
        maximum["hamr_adapter_wheel_difference"] = max(maximum["hamr_adapter_wheel_difference"],
                                                         float(np.max(np.abs(u[:2]-(right, left)))))
    return maximum


def exact_step(position, yaw, wheels, dt, r, a, b):
    """Integrate held wheel rates exactly, then recover the offset point."""
    axle = position - b*np.array([math.cos(yaw), math.sin(yaw)])
    speed = r * sum(wheels) / 2
    omega = r * (wheels[0] - wheels[1]) / (2 * a)
    new_yaw = yaw + omega*dt
    if abs(omega) < 1e-12:
        axle += speed*dt*np.array([math.cos(yaw), math.sin(yaw)])
    else:
        axle += speed/omega*np.array([math.sin(new_yaw)-math.sin(yaw),
                                      math.cos(yaw)-math.cos(new_yaw)])
    return axle + b*np.array([math.cos(new_yaw), math.sin(new_yaw)]), new_yaw


def corner(label, dt=.02, acceleration=None, delay=0., speed=.25,
           r=.1075, a=.33072, b=.27114, wheel_limit=6.):
    """Begin exactly at corner, aligned with incoming +X at steady speed.

    From t=0 the reference moves along +Y. There are no caster forces, slip,
    motor lag, pose delay/noise, or wrong dimensions. P gain equals sim's 1.5.
    An optional command delay delays the new target's first application only.
    """
    position, yaw = np.zeros(2), 0.
    wheels = np.full(2, speed/r)
    rows = []
    saturated = 0
    for tick in range(round(4/dt)+1):
        time = tick*dt
        reference = np.array([0., speed*time])
        error = reference-position
        rows.append([time, *position, yaw, float(np.linalg.norm(error)), *wheels])
        if time < delay-1e-12:
            target = np.full(2, speed/r)
        else:
            desired = np.array([0., speed]) + 1.5*error
            target = inverse(*desired, yaw, r, a, b)
            scale = min(1., wheel_limit/max(float(np.max(np.abs(target))), 1e-12))
            saturated += scale < 1.-1e-12
            target *= scale
        if acceleration is not None:
            wheels += np.clip(target-wheels, -acceleration*dt, acceleration*dt)
        else:
            wheels = target
        position, yaw = exact_step(position, yaw, wheels, dt, r, a, b)
    rows = np.array(rows)
    return {"label": label, "dt_s": dt, "acceleration_limit_rad_s2": acceleration,
            "initial_command_delay_s": delay, "speed_m_s": speed,
            "max_tracking_error_m": float(np.max(rows[:, 4])),
            "max_past_corner_m": float(np.max(rows[:, 1])),
            "peak_time_s": float(rows[np.argmax(rows[:, 4]), 0]),
            "saturated_control_ticks": int(saturated)}, rows


def main():
    parameters = {}
    for name, r, a, b, limit in (
            ("simulation", .1075, .33072, .27114, 6.),
            ("hardware_config", .122, .350, .301, 2.93215314335)):
        k = a/b
        speed = .25
        wheel_before = speed/r
        wheel_after = a*speed/(b*r)
        c = r*limit/speed
        peak = math.sqrt(1+k*k)
        saturation_range = None
        if c < peak:
            phi = math.atan(k)
            half_span = math.acos(c/peak)
            saturation_range = [math.degrees(phi-half_span), math.degrees(phi+half_span)]
        parameters[name] = {
            "radius_m": r, "half_track_a_m": a, "offset_b_m": b,
            "xy_condition_number": max(k, 1/k),
            "xy_jacobian_singular_values_m_per_rad": [r/math.sqrt(2), r*b/(a*math.sqrt(2))],
            "straight_wheel_rad_s_at_025": wheel_before,
            "corner_initial_yaw_rate_rad_s": speed/b,
            "corner_initial_wheels_right_left_rad_s": [wheel_after, -wheel_after],
            "initial_max_wheel_jump_rad_s": wheel_before+wheel_after,
            "frozen_yaw_transition_time_at_15_rad_s2_s": (wheel_before+wheel_after)/15.,
            "nominal_max_wheel_rad_s_during_alignment": speed/r*peak,
            "max_point_speed_all_directions_m_s": r*limit/peak,
            "heading_error_for_max_wheel_deg": math.degrees(math.atan(k)),
            "nominal_saturation_heading_range_deg_at_025": saturation_range,
            "heading_small_error_time_constant_s": b/speed,
        }
    cases = [
        corner("exact held wheels, 0.5 ms", dt=.0005),
        corner("exact held wheels, 20 ms"),
        corner("15 rad/s2 command slew, 20 ms", acceleration=15.),
        corner("15 rad/s2 plus 40 ms initial delay", acceleration=15., delay=.04),
        corner("half speed, 15 rad/s2", acceleration=15., speed=.125),
        corner("hardware geometry + cap, ideal wheel response", dt=.001,
               r=.122, a=.350, b=.301, wheel_limit=2.93215314335),
    ]
    results = {"scope": "Offline counterfactual planar kinematics; no Gazebo or hardware actuation",
               "algebra_audit_10000_random_cases": audit(), "parameters": parameters,
               "single_corner_cases": [summary for summary, _ in cases]}
    (HERE/"kinematics_results.json").write_text(json.dumps(results, indent=2)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for summary, rows in cases[:5]:
        axes[0].plot(rows[:, 1]*1000, rows[:, 2]*1000, label=summary["label"])
        axes[1].plot(rows[:, 0], rows[:, 4]*1000, label=summary["label"])
    axes[0].plot([-20, 0, 0], [0, 0, 200], "k--", label="Sharp reference corner")
    axes[0].set(xlabel="Past-corner distance (mm)", ylabel="Along outgoing leg (mm)",
                xlim=(-8, 62), ylim=(-5, 150), title="Exact Jacobian; no caster or slip")
    axes[1].set(xlabel="Time after corner (s)", ylabel="Reference tracking error (mm)",
                xlim=(0, 2), title="Effect of sampling, wheel slew, and delay")
    for ax in axes:
        ax.grid(alpha=.25)
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(HERE/"kinematics_ablation.png", dpi=170)
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
