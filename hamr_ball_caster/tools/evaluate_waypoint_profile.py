#!/usr/bin/env python3
"""Evaluate one production waypoint profile in a fresh, isolated Gazebo.

Source scripts/env.sh before running. Results include the exact command arrays,
an immutable configuration snapshot, logs, trajectory measurements, and a run
summary. This harness starts and stops its own simulation process groups only.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

PROFILE_NAMES = {"smooth": "smooth", "continuous": "continuous", "legacy": "simple"}


def _group_exists(group):
    try:
        os.killpg(group, 0)
        return True
    except ProcessLookupError:
        return False


def stop_process_group(process):
    """Stop an exclusively owned group, including children whose launcher exited."""
    if process is None:
        return None
    result = {"process_group": process.pid, "signals": []}
    for sig, timeout in ((signal.SIGINT, 15.), (signal.SIGTERM, 3.), (signal.SIGKILL, 1.)):
        process.poll()  # Reap the direct child even if only its descendants remain.
        if not _group_exists(process.pid):
            break
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        result["signals"].append(sig.name)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()
            if not _group_exists(process.pid):
                break
            time.sleep(.1)
    result["leader_exit_code"] = process.poll()
    # A reparented zombie can remain in the process table until its new parent
    # reaps it; SIGKILL nevertheless prevents any owned executable from running.
    result["group_present_after_cleanup"] = _group_exists(process.pid)
    return result


def _interrupt(_signum, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="Override the installed profile configuration")
    parser.add_argument("--profile", choices=tuple(PROFILE_NAMES), default="smooth")
    parser.add_argument("--output-dir", required=True, type=Path, help="New or empty result directory")
    parser.add_argument("--domain-id", type=int, default=172, help="Dedicated local ROS domain")
    parser.add_argument("--gui", action="store_true", help="Open the live Gazebo GUI")
    parser.add_argument("--max-step-size", type=float, help="Optional Gazebo physics timestep, in seconds")
    parser.add_argument("--wall-timeout", type=float, default=1200.)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error("--domain-id must be in [0, 232]")
    if not math.isfinite(args.wall_timeout) or args.wall_timeout < 60.:
        parser.error("--wall-timeout must be finite and at least 60 seconds")
    if args.max_step_size is not None and (
        not math.isfinite(args.max_step_size) or not 0 < args.max_step_size <= .01
    ):
        parser.error("--max-step-size must be in (0, 0.01] seconds")

    # Keep --help usable without a sourced ROS environment.
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory("hamr_ball_caster"))
    source_config = (args.config.expanduser().resolve() if args.config else
                     share / "config" / f"{PROFILE_NAMES[args.profile]}_waypoint_sim.yaml")
    if not source_config.is_file():
        parser.error(f"Configuration file does not exist: {source_config}")
    output = args.output_dir.expanduser().resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        parser.error(f"Output must be a new or empty directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config_bytes = source_config.read_bytes()
    snapshot = output / "configuration.yaml"
    snapshot.write_bytes(config_bytes)
    report = output / "trajectory.json"
    env = dict(os.environ)
    env.update(
        ROS_DOMAIN_ID=str(args.domain_id),
        ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
        ROS_LOCALHOST_ONLY="1",
        GZ_PARTITION=f"hamr_waypoint_evaluation_{os.getpid()}_{time.time_ns()}",
    )
    commands = {
        "launch": ["ros2", "launch", "hamr_ball_caster", "ball_caster.launch.py",
                   "model:=compa", "controller:=false", f"gui:={str(args.gui).lower()}"],
        "trajectory": ["ros2", "run", "hamr_ball_caster", "run_waypoint_sim.py",
                       "--profile", args.profile, "--config", str(snapshot),
                       "--output", str(report), "--wall-timeout", str(args.wall_timeout)],
    }
    if args.max_step_size is not None:
        commands["launch"].append(f"max_step_size:={args.max_step_size}")
    provenance = {
        "commands": commands,
        "environment": {key: env[key] for key in (
            "ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_LOCALHOST_ONLY", "GZ_PARTITION")},
        "configuration_source": str(source_config),
        "configuration_snapshot": str(snapshot),
        "configuration_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "package_share": str(share),
        "started_utc": datetime.now(timezone.utc).isoformat(),
    }
    (output / "commands.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(f"Evaluating {args.profile} profile in ROS domain {args.domain_id}: {output}", flush=True)

    simulator = runner = None
    failure = None
    runner_code = None
    cleanup = {}
    started = time.monotonic()
    previous_term_handler = signal.signal(signal.SIGTERM, _interrupt)
    try:
        with (output / "gazebo.log").open("w") as gz_log, (output / "trajectory.log").open("w") as trajectory_log:
            try:
                simulator = subprocess.Popen(commands["launch"], env=env, stdout=gz_log,
                                             stderr=subprocess.STDOUT, start_new_session=True)
                # The production runner waits for clock, odometry, complete
                # vehicle joints and command subscribers before starting motion.
                runner = subprocess.Popen(commands["trajectory"], env=env, stdout=trajectory_log,
                                          stderr=subprocess.STDOUT, start_new_session=True)
                last_message = started
                while runner.poll() is None:
                    if simulator.poll() is not None:
                        raise RuntimeError("Gazebo exited during evaluation; see gazebo.log")
                    elapsed = time.monotonic() - started
                    if elapsed > args.wall_timeout:
                        raise RuntimeError("Evaluation exceeded its wall-time timeout")
                    if time.monotonic() - last_message >= 20.:
                        print(f"Evaluation continues; {elapsed:.0f} wall seconds elapsed.", flush=True)
                        last_message = time.monotonic()
                    time.sleep(.2)
                runner_code = runner.returncode
                if runner_code:
                    raise RuntimeError(f"Trajectory runner exited with status {runner_code}; see trajectory.log and trajectory.json")
            except (RuntimeError, OSError, KeyboardInterrupt) as exc:
                failure = str(exc) or "Interrupted"
                print(failure, file=sys.stderr, flush=True)
            finally:
                # A second interrupt must not bypass cleanup of an owned simulator.
                previous_int_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
                signal.signal(signal.SIGTERM, signal.SIG_IGN)
                try:
                    cleanup["trajectory"] = stop_process_group(runner)
                    cleanup["gazebo"] = stop_process_group(simulator)
                finally:
                    signal.signal(signal.SIGINT, previous_int_handler)
    finally:
        signal.signal(signal.SIGTERM, previous_term_handler)

    result = {
        "passed": failure is None,
        "failure": failure,
        "trajectory_exit_code": runner_code,
        "trajectory_profile": args.profile,
        "trajectory_report": str(report),
        "configuration_source": str(source_config),
        "configuration_snapshot": str(snapshot),
        "configuration_sha256": provenance["configuration_sha256"],
        "requested_max_step_size_s": args.max_step_size,
        "wall_duration_s": time.monotonic() - started,
        "cleanup": cleanup,
        "scope": "Fresh Gazebo full COMPA/HAMR retrofit, two passive CAD casters, isolated local ROS domain",
        "gui": args.gui,
    }
    try:
        trajectory = json.loads(report.read_text())
        result["trajectory_summary"] = trajectory["summary"]
        if not trajectory["summary"]["passed"]:
            result.update(passed=False, failure=failure or "Trajectory acceptance checks failed")
        if args.profile == "continuous":
            result["continuous_plan"] = trajectory["continuous_plan"]
            result["continuous_acceptance"] = trajectory["continuous_acceptance"]
    except (OSError, ValueError, KeyError) as exc:
        result.update(passed=False, failure=failure or f"Missing or invalid trajectory report: {exc}")
    (output / "run_summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"{'PASS' if result['passed'] else 'FAIL'}: {output / 'run_summary.json'}", flush=True)
    if "trajectory_summary" in result:
        summary = result["trajectory_summary"]
        print(json.dumps({key: summary.get(key) for key in (
            "motion_rmse_m", "motion_max_error_m", "final_position_error_m")}), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
