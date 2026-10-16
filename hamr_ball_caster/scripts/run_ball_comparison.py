#!/usr/bin/env python3
"""Repeat a saved swivel-caster experiment matrix with the passive CAD ball model."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import yaml


BALL_JOINTS = [f"{side}_caster_{kind}_joint" for side in ("left", "right")
               for kind in ("carrier", "positive_hemisphere", "negative_hemisphere",
                            "positive_polar_roller", "negative_polar_roller")]


def atomic_write(path, text):
    temporary = path.with_name(path.name + f".{os.getpid()}.tmp")
    temporary.write_text(text)
    temporary.replace(path)


def prepare(source, output):
    """Keep every original request; explicitly label non-equivalent prep/physics."""
    source, output = Path(source).resolve(), Path(output).resolve()
    original = json.loads((source / "experiment_matrix.json").read_text())
    (output / "configs").mkdir(parents=True, exist_ok=True)
    cases = []
    for item in original:
        name = item["name"]
        swivel_run = source / "record" / name
        if not (swivel_run / "run_summary.json").is_file():
            raise ValueError(f"Missing completed swivel baseline: {name}")
        source_config = swivel_run / "configuration.yaml"
        original_bytes = source_config.read_bytes()
        config = yaml.safe_load(original_bytes)
        config["name"] = "ball_comparison_" + name
        config["passive_joint_names"] = list(BALL_JOINTS)
        config.pop("expect_caster_swivel_motion", None)
        assisted = bool(config.get("startup_alignment", {}).get("enabled"))
        if assisted:
            config["startup_alignment"]["caster_model"] = "ball"
        destination = output / "configs" / (name + ".yaml")
        content = yaml.safe_dump(config, sort_keys=False)
        if destination.exists() and destination.read_text() != content:
            raise ValueError(f"Refusing to alter an existing comparison configuration: {destination}")
        atomic_write(destination, content)
        sensitivity = bool(item.get("xacro_args"))
        cases.append({"name": name, "category": item["category"],
                      "initial_yaw_deg": item["initial_yaw_deg"],
                      "speed_m_s": item["speed_m_s"], "points_m": item["points_m"],
                      "route_length_m": item["route_length_m"],
                      "ball_configuration": str(destination),
                      "swivel_directory": str(swivel_run),
                      "ball_directory": str(output / "ball" / name),
                      "swivel_configuration_sha256": hashlib.sha256(original_bytes).hexdigest(),
                      "swivel_xacro_overrides": item.get("xacro_args", []),
                      "ball_xacro_overrides": [], "ground_mu": .8,
                      "non_equivalent_physical_parameters": sensitivity,
                      "startup_policy": "chassis recovery and lead-in; no swivel-angle gate" if assisted else "none",
                      "tracking_comparison_region": "later_legs" if assisted else "motion",
                      "ball_role": "nominal repeated baseline for swivel-only sensitivity" if sensitivity else "nominal architecture comparison",
                      "comparison_note": ("Both start at the same original pose and visit the same destinations; "
                         "preparation can yield different first segment sources. Compare startup separately and use later legs."
                         if assisted else "Same original pose, requested route, speed, controller settings and floor."),
                      "physical_note": ("Swivel physical override has no one-to-one ball equivalent; ball parameters remain nominal."
                          if sensitivity else "Architecture-specific estimated masses, contact geometry and bearing losses remain different.")})
    atomic_write(output / "comparison_matrix.json", json.dumps(cases, indent=2) + "\n")
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--swivel-suite", type=Path, required=True)
    parser.add_argument("--mode", choices=("prepare", "screen", "record"), default="prepare")
    parser.add_argument("--select", help="Comma-separated case names, in execution order")
    parser.add_argument("--domain-id", type=int, default=93)
    parser.add_argument("--real-time-factor", type=float, default=.4)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    cases = prepare(args.swivel_suite.expanduser(), output)
    if args.select:
        requested = args.select.split(",")
        mapping = {c["name"]: c for c in cases}
        if len(requested) != len(set(requested)) or set(requested) - mapping.keys():
            parser.error("--select must contain unique names from the saved matrix")
        cases = [mapping[n] for n in requested]
    if args.mode == "prepare":
        print(f"Prepared {len(cases)} matched ball cases: {output}")
        return 0
    execution = output / ("execution" if args.mode == "record" else "screen_execution")
    execution.mkdir(exist_ok=True)
    (output / "claims").mkdir(exist_ok=True)
    for i, case in enumerate(cases, 1):
        destination = output / ("ball" if args.mode == "record" else "screen") / case["name"]
        summary_path = destination / "run_summary.json"
        if args.resume and summary_path.exists():
            print(f"Retained completed case: {case['name']}", flush=True)
            continue
        claim = output / "claims" / (args.mode + "_" + case["name"] + ".json")
        with claim.open("x") as stream:
            json.dump({"pid": os.getpid(), "case": case["name"]}, stream)
        try:
            command = ["ros2", "run", "hamr_ball_caster", "run_ball_caster.py",
                       "--profile", "legacy", "--config", case["ball_configuration"],
                       "--initial-yaw-deg", str(case["initial_yaw_deg"]),
                       "--output-dir", str(destination), "--domain-id", str(args.domain_id),
                       "--ground-mu", str(case["ground_mu"]), "--wall-timeout", "3600"]
            command += (["--record", "--real-time-factor", str(args.real_time_factor)]
                        if args.mode == "record" else ["--physics-only"])
            print(f"[{i}/{len(cases)}] {case['name']} ({args.mode})", flush=True)
            started = datetime.now(timezone.utc).isoformat()
            code = subprocess.call(command)
            summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
            atomic_write(execution / (case["name"] + ".json"), json.dumps({
                "command": command, "started_utc": started, "exit_code": code,
                "summary": str(summary_path), "behavior_checks_passed": summary.get("passed", False),
                "failure": summary.get("failure"), "case": case}, indent=2) + "\n")
            if not summary:
                raise RuntimeError(f"Case produced no final summary: {destination}")
            if args.mode == "record" and not all(v.get("passed") for v in summary.get("videos", {}).values()):
                raise RuntimeError(f"Camera validation failed; preserve evidence and inspect: {destination}")
        finally:
            claim.unlink()
    return 0


if __name__ == "__main__":
    sys.exit(main())
