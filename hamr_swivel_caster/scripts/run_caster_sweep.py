#!/usr/bin/env python3
"""Reproducible trajectory, initial-heading and physical-parameter experiments.

Every case uses the real Gazebo runner. Failed behavior checks are retained as
experimental outcomes, never turned into successful acceptance checks.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import yaml


def experiments():
    cases = []

    def add(name, points, yaw=90., speed=.25, category="trajectory", **extra):
        cases.append(dict(name=name, points_m=points, initial_yaw_deg=yaw,
                          speed_m_s=speed, category=category, **extra))

    def circle(radius, laps=1, count=48):
        return [[radius*(math.cos(2*math.pi*i/count)-1), radius*math.sin(2*math.pi*i/count)]
                for i in range(laps*count+1)]

    add("straight_y", [[0, 0], [0, 3]])
    add("horizontal_x", [[0, 0], [3, 0]])
    add("reversal_180", [[0, 0], [0, 1.2], [0, -.6], [0, 1.2]])
    add("horizontal_shuttle", [[0, 0], [.3, 0], [-.3, 0], [.3, 0], [-.3, 0], [.3, 0], [0, 0]], speed=.4)
    add("square_360", [[0, 0], [0, 1.2], [-1.2, 1.2], [-1.2, 0], [0, 0], [0, 1.2]])
    add("circle_large_360", circle(.8))
    add("circle_tight_three_laps", circle(.2, 3, 32))
    figure = []
    for i in range(65):
        a = 2*math.pi*i/64
        x, y = .8*math.sin(a), .8*math.sin(a)*math.cos(a)
        figure.append([(x-y)/math.sqrt(2), (x+y)/math.sqrt(2)])
    add("figure_eight", figure)
    add("hairpin", [[0, 0], [0, 1.2], [.12, 1.2], [.12, 0], [.24, 0], [.24, 1.2]])
    add("sharp_zigzag", [[0, 0], [0, .8], [.15, .8], [-.15, 1.], [.15, 1.2], [-.15, 1.4], [.15, 1.6], [-.15, 1.8], [0, 2]], speed=.4)
    vertices = [[.65*math.cos(math.pi/2+2*math.pi*i/5), .65*math.sin(math.pi/2+2*math.pi*i/5)] for i in range(5)]
    star = [[vertices[i][0]-vertices[0][0], vertices[i][1]-vertices[0][1]] for i in [0, 2, 4, 1, 3, 0]]
    add("star_reversals", star)
    tiny = [[0, 0]] + [[0, .12], [.12, .12], [.12, 0], [0, 0]]*4
    add("tiny_square_four_laps", tiny)
    add("smooth_s_curve", [[.35*math.sin(2*math.pi*i/40), 2.4*i/40] for i in range(41)])

    original = [[0, 0], [0, 2], [-2, 2], [-2, 4.5], [0, 4.5], [0, 2], [0, 0]]
    for yaw in (0, 90, 180, 225, 270):
        add(f"heading_{yaw:03}_aligned", original, yaw=yaw, category="initial_heading", startup_alignment=True)
    add("heading_270_unassisted", original, yaw=270, category="initial_heading")

    # Match geometry/speed and vary one physical factor at a time. These are
    # sensitivity assumptions, not calibrated properties of the purchased unit.
    stress = next(c for c in cases if c["name"] == "tiny_square_four_laps")["points_m"]
    for name, args in (
        ("swivel_drag_10x", ["swivel_damping=0.15"]),
        ("swivel_coulomb_0p1", ["swivel_friction=0.1"]),
        ("inertia_3x", ["caster_fork_inertia_scale=3", "caster_wheel_inertia_scale=3"]),
        ("caster_grip_0p2", ["tire_mu=0.2"]),
    ):
        add(name, stress, category="sensitivity", xacro_args=args)
    add("tiny_square_slow", stress, speed=.08, category="speed_control")
    add("tight_circle_slow", circle(.2, 3, 32), speed=.08, category="speed_control")
    shuttle = next(c for c in cases if c["name"] == "horizontal_shuttle")["points_m"]
    add("horizontal_shuttle_half_speed", shuttle, speed=.20, category="speed_control")
    add("horizontal_shuttle_slow", shuttle, speed=.08, category="speed_control")
    reversal = next(c for c in cases if c["name"] == "reversal_180")["points_m"]
    add("reversal_repeat", reversal, category="repeat_control")
    add("reversal_drag_10x", reversal, category="sensitivity", xacro_args=["swivel_damping=0.15"])
    add("reversal_inertia_3x", reversal, category="sensitivity",
        xacro_args=["caster_fork_inertia_scale=3", "caster_wheel_inertia_scale=3"])
    return cases


def prepare(output, base_config):
    output.mkdir(parents=True, exist_ok=True)
    configs = output / "configs"
    configs.mkdir(exist_ok=True)
    base = yaml.safe_load(base_config.read_text())
    cases = experiments()
    for case in cases:
        cfg = dict(base)
        cfg.update(name=case["name"], points_m=case["points_m"], speed_m_s=case["speed_m_s"],
                   fixed_turret_yaw_rad=math.pi/2, startup_hold_s=2., final_hold_s=3.,
                   expect_first_leg_alignment=False, expect_caster_swivel_motion=False)
        if case.get("startup_alignment"):
            cfg["startup_alignment"] = {"enabled": True,
                "heading_tolerance_rad": math.radians(2), "caster_tolerance_rad": math.radians(3)}
            cfg["expect_first_leg_alignment"] = True
        config = configs / (case["name"] + ".yaml")
        content = yaml.safe_dump(cfg, sort_keys=False)
        if config.exists() and config.read_text() != content:
            raise ValueError(f"Refusing to change existing experiment configuration: {config}")
        config.write_text(content)
        case["configuration"] = str(config)
        case["route_length_m"] = sum(math.dist(a, b) for a, b in zip(case["points_m"], case["points_m"][1:]))
    (output / "experiment_matrix.json").write_text(json.dumps(cases, indent=2) + "\n")
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--base-config", type=Path)
    parser.add_argument("--mode", choices=("prepare", "screen", "record"), default="prepare")
    parser.add_argument("--select", help="Comma-separated exact case names; default all")
    parser.add_argument("--real-time-factor", type=float, default=.4, help="Capture simulation/wall ratio")
    parser.add_argument("--domain-id", type=int, default=89)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    if args.base_config is None:
        from ament_index_python.packages import get_package_share_directory
        args.base_config = Path(get_package_share_directory("hamr_swivel_caster")) / "config/simple_waypoint_sim.yaml"
    output = args.output.expanduser().resolve()
    cases = prepare(output, args.base_config)
    if args.select:
        requested = args.select.split(",")
        names = set(requested)
        unknown = names - {c["name"] for c in cases}
        if unknown:
            parser.error(f"Unknown experiments: {sorted(unknown)}")
        if len(requested) != len(names):
            parser.error("--select must not repeat experiments")
        by_name = {c["name"]: c for c in cases}
        cases = [by_name[name] for name in requested]
    if args.mode == "prepare":
        print(f"Prepared {len(cases)} experiments: {output}")
        return 0
    results = output / args.mode
    results.mkdir(exist_ok=True)
    index_path = output / f"{args.mode}_index.json"
    index = json.loads(index_path.read_text()) if index_path.exists() else {}
    for number, case in enumerate(cases, 1):
        destination = results / case["name"]
        summary_file = destination / "run_summary.json"
        if args.resume and summary_file.exists():
            print(f"Existing result retained: {case['name']}", flush=True)
            continue
        command = ["ros2", "run", "hamr_swivel_caster", "run_swivel_caster.py", "--profile", "legacy",
                   "--initial-yaw-deg", str(case["initial_yaw_deg"]), "--config", case["configuration"],
                   "--output-dir", str(destination), "--domain-id", str(args.domain_id)]
        if args.mode == "record":
            command += ["--record", "--real-time-factor", str(args.real_time_factor)]
        else:
            command += ["--physics-only"]
        for value in case.get("xacro_args", []):
            command += ["--xacro-arg", value]
        print(f"[{number}/{len(cases)}] {case['name']} ({args.mode})", flush=True)
        started = datetime.now(timezone.utc).isoformat()
        code = subprocess.call(command)
        summary = json.loads(summary_file.read_text()) if summary_file.exists() else {}
        index[case["name"]] = {"command": command, "started_utc": started, "exit_code": code,
                              "summary": str(summary_file), "behavior_checks_passed": summary.get("passed", False),
                              "failure": summary.get("failure"), "configuration": case}
        index_path.write_text(json.dumps(index, indent=2) + "\n")
        if not summary:
            raise RuntimeError(f"Experiment produced no summary: {case['name']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
