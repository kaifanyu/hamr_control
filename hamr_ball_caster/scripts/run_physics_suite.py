#!/usr/bin/env python3
"""Run isolated headless caster cases and retain logs and measured joint states."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("ball_caster_validation"))
    parser.add_argument("--cases", nargs="+", default=["baseline", "half_step", "seam", "pole"],
                        choices=["baseline", "half_step", "seam", "pole"])
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    # Native a=(0,.8876457,-.4605269); canonical a=(-.8876457,0,.4605269).
    # Carrier rotates about canonical -Y. These phases place the split axis
    # horizontally (seam) and downward (positive polar roller), respectively.
    seam = math.atan2(0.4605269327005004, 0.8876457312787961)
    cases = {"baseline": (0.001, 0.0, "rig_dt_001.json"),
             "half_step": (0.0005, 0.0, "rig_dt_0005.json"),
             "seam": (0.001, seam, "rig_seam.json"),
             "pole": (0.001, seam + math.pi / 2, "rig_pole.json")}
    results = {}
    for index, name in enumerate(args.cases):
        step, phase, filename = cases[name]
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = str(100 + index)
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        env["GZ_PARTITION"] = f"hamr_caster_check_{os.getpid()}_{name}"
        command = ["ros2", "launch", "hamr_ball_caster", "ball_caster.launch.py",
                   "model:=rig", "gui:=false", f"max_step_size:={step}", f"carrier_phase:={phase}"]
        log_path = output / f"{name}.log"
        print(f"Running {name}: dt={step}, carrier phase={phase:.8f}", flush=True)
        with log_path.open("w") as log:
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                checker = Path(__file__).with_name("run_physics_checks.py")
                check = subprocess.run([sys.executable, str(checker), "--output", str(output / filename),
                                        "--wall-timeout", "240", "--label",
                                        f"DART; dt={step} mu=0.8 phase={phase} load=2kg"],
                                       env=env, timeout=270)
                results[name] = {"exit_code": check.returncode, "report": filename,
                                 "launch_log": log_path.name}
            except subprocess.TimeoutExpired:
                results[name] = {"exit_code": 124, "error": "check timed out", "launch_log": log_path.name}
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait()
        if results[name]["exit_code"] != 0:
            print(f"Case {name} failed. Inspect {log_path} and its JSON report.", file=sys.stderr)
    (output / "suite_results.json").write_text(json.dumps(results, indent=2) + "\n")
    return int(any(r["exit_code"] != 0 for r in results.values()))


if __name__ == "__main__":
    raise SystemExit(main())
