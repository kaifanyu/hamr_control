"""Run one reproducible local trajectory and finalize every recording on exit.

    python -m hamr_sim_ui.runner --caster ball --terrain flat --trajectory straight

The desktop is optional. Gazebo scene state, ROS bag, velocity/joint CSV and
hardware-schema encoder packets are always saved. Video is opt-in.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

from .controller import LiveController, add_limit_arguments, limits_from_args, validate_points
from .presets import PRESET_NAMES, points_inside, preset_points
from .session import SimulationSession, discover_terrains
from .telemetry import RosTelemetry


def parser():
    result = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    result.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    result.add_argument("--caster", choices=("ball", "swivel"), default="ball")
    result.add_argument("--terrain", default="flat", help="Map ID; use --list-terrains to list")
    route = result.add_mutually_exclusive_group()
    route.add_argument("--trajectory", choices=PRESET_NAMES, default="straight")
    route.add_argument("--route-json", type=Path,
                       help='Absolute map XY array or desktop-exported points_m JSON; --speed overrides saved speed')
    result.add_argument("--scale", type=float, default=1., help="Preset distance multiplier (default: 1)")
    result.add_argument("--speed", type=float, default=.2, help="Requested peak trajectory speed, m/s; planner may lower it")
    result.add_argument("--cameras", choices=("on", "off"), default="off", help="Camera sensors and bridge (default: off)")
    result.add_argument("--video", action="store_true", help="Record overview and caster MP4; requires --cameras on")
    display = result.add_mutually_exclusive_group()
    display.add_argument("--gui", dest="gui", action="store_true", default=True, help="Open normal Gazebo window (default)")
    display.add_argument("--headless", dest="gui", action="store_false", help="Run without Gazebo's GUI")
    result.add_argument("--output", type=Path, help="Parent output directory (default: ~/Videos/hamr_sim_ui)")
    result.add_argument("--timeout", type=float, default=1800., help="Whole-run wall-clock timeout, seconds (default: 1800)")
    result.add_argument("--startup-timeout", type=float, default=120., help="Maximum wait for live ROS feedback, wall seconds")
    result.add_argument("--replay", type=Path, help="Replay a finalized session or recording folder; Ctrl-C closes")
    result.add_argument("--list-terrains", action="store_true")
    add_limit_arguments(result)
    return result


def validate_arguments(args):
    limits = limits_from_args(args)
    limits.validate_speed(args.speed)
    for name in ("timeout", "startup_timeout", "scale"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            raise ValueError(f"--{name.replace('_', '-')} must be finite and positive")
    if args.video and args.cameras != "on":
        raise ValueError("--video requires --cameras on")
    if args.route_json:
        load_route(args.route_json, args.terrain)
    return limits


def load_route(path, terrain=None):
    value = json.loads(Path(path).expanduser().read_text())
    if isinstance(value, dict):
        if terrain is not None and value.get("terrain") is not None and value["terrain"] != terrain:
            raise ValueError(f"Route was saved for terrain {value['terrain']!r}, not {terrain!r}; choose its --terrain")
        value = value.get("points_m", value.get("points"))
    if not isinstance(value, list):
        raise ValueError("Route JSON must be an XY array or an object containing points_m or points")
    return validate_points(value)


def _save(path, value):
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def _check(session, deadline, telemetry=None):
    error = session.poll()
    if error:
        raise RuntimeError(error)
    if time.monotonic() >= deadline:
        raise TimeoutError("Run exceeded its wall-clock timeout; partial recordings were retained")
    if telemetry:
        error = telemetry.snapshot(include_history=False, include_trail=False).get("last_error")
        if error:
            raise RuntimeError(error)


def run_simulation(args, session, limits):
    """Return an exit code after owned processes and recordings are finalized."""
    telemetry = controller = None
    run_dir = None
    recording = None
    summary = {"status": "starting", "caster": args.caster, "terrain": args.terrain,
               "trajectory": str(args.route_json) if args.route_json else args.trajectory,
               "preset_scale": args.scale, "requested_speed_m_s": args.speed,
               "motion_limits": limits.to_dict(), "cameras": args.cameras == "on",
               "video": args.video, "timeout_wall_s": args.timeout}
    code = 1
    deadline = time.monotonic() + args.timeout
    try:
        telemetry = RosTelemetry(caster=args.caster, cameras=args.cameras == "on", preview_cameras=())
        controller = LiveController(telemetry.node, args.root, limits=limits)
        run_dir = session.start(args.terrain, gui=args.gui, caster=args.caster, cameras=args.cameras == "on")
        _save(run_dir / "run.json", summary)
        print(f"Session: {run_dir}", flush=True)
        ready_deadline = min(deadline, time.monotonic() + args.startup_timeout)
        while True:
            _check(session, ready_deadline, telemetry)
            state = telemetry.snapshot(include_history=False, include_trail=False)
            motion = controller.snapshot()
            if state["odom_received"] and state["joints_received"] and motion["feedback_ready"]:
                break
            time.sleep(.05)
        points = load_route(args.route_json, args.terrain) if args.route_json else preset_points(
            args.trajectory, motion["position"], motion["yaw"], args.scale)
        if not points_inside(points, session.terrain.bounds):
            raise ValueError("Trajectory extends beyond the map bounds; reduce --scale or edit the route")
        metadata = {**summary, "gazebo_log": str(session.gazebo_log),
                    "scene_log_scope": "Full session; bag/video begin before trajectory commands",
                    "requested_points": points}
        recording = run_dir / "recording"
        telemetry.start_recording(recording, video=args.video, session_metadata=metadata)
        points = controller.set_route(points, args.speed)
        plan = controller.snapshot()["plan_summary"]
        summary.update(status="running", points=points, plan=plan, recording=str(recording))
        _save(run_dir / "run.json", summary)
        print(f"Requested peak {args.speed:g} m/s; planned peak {plan['planned_peak_speed_m_s']:.3f} m/s; "
              f"nominal motion {plan['nominal_duration_s']:.1f} sim seconds + waypoint settling.", flush=True)
        next_progress = 0.
        while True:
            _check(session, deadline, telemetry)
            motion = controller.snapshot()
            if motion["state"] == "error":
                raise RuntimeError(motion["error"])
            if motion["state"] == "complete":
                controller.halt()
                # Capture the zero command and post-stop feedback before closing the bag.
                settled_at = motion["sim_time"]
                while telemetry.snapshot(include_history=False, include_trail=False)["sim_time"] < settled_at + .25:
                    _check(session, deadline, telemetry)
                    time.sleep(.05)
                break
            if time.monotonic() >= next_progress:
                print(f"Simulation {motion['sim_time']:.2f} s · {motion['state']} · "
                      f"waypoint {motion['segment'] + 1}/{len(points) - 1}", flush=True)
                next_progress = time.monotonic() + 10.
            time.sleep(.05)
        summary.update(status="complete", final_position=motion["position"],
                       final_sim_time_s=motion["sim_time"])
        code = 0
    except KeyboardInterrupt:
        summary.update(status="interrupted", error="Interrupted by user; recordings finalized")
        code = 130
    except Exception as error:
        summary.update(status="failed", error=str(error))
        print(f"Run failed: {error}", file=sys.stderr, flush=True)
    finally:
        # Every cleanup step gets a chance even if a recorder or ROS node failed.
        errors = []
        if controller is not None:
            try:
                controller.close()
            except Exception as error:
                errors.append(f"Controller shutdown: {error}")
        if telemetry is not None:
            try:
                report = telemetry.stop_recording()
                summary["recording_complete"] = report.get("recording_complete", False)
                if recording and not summary["recording_complete"]:
                    errors.append("Recording incomplete: " + "; ".join(report.get("errors", [])))
                if recording:
                    missing = [topic for topic in ("/clock", "/compa/odom", "/joint_states", "/caster_encoder/packets")
                               if not report.get("message_counts", {}).get(topic)]
                    if missing:
                        errors.append("Recording is missing required data: " + ", ".join(missing))
            except Exception as error:
                errors.append(f"Recording finalization: {error}")
            try:
                telemetry.close()
            except Exception as error:
                errors.append(f"Telemetry shutdown: {error}")
        try:
            session.stop()
        except Exception as error:
            errors.append(f"Gazebo shutdown: {error}")
        if run_dir:
            state_log = run_dir / "gazebo_log/state.tlog"
            summary["scene_recording_complete"] = state_log.is_file() and state_log.stat().st_size > 0
            if not summary["scene_recording_complete"]:
                errors.append("Gazebo scene state log is missing or empty")
            if errors:
                summary["finalization_errors"] = errors
                if code == 0:
                    code = 1
                    summary["status"] = "failed"
            _save(run_dir / "run.json", summary)
            print(f"{summary['status'].capitalize()}. Local files: {run_dir}", flush=True)
        for error in errors:
            print(error, file=sys.stderr)
    return code


def run_replay(args, session):
    try:
        path = session.start_replay(args.replay, gui=args.gui)
        print(f"Replaying: {path}\nClose Gazebo or press Ctrl-C to stop.", flush=True)
        deadline = time.monotonic() + args.timeout
        while session.running:
            if time.monotonic() >= deadline:
                raise TimeoutError("Replay exceeded --timeout wall seconds")
            time.sleep(.1)
        if session.process is not None and session.process.returncode:
            raise RuntimeError(session.poll())
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print(f"Replay failed: {error}", file=sys.stderr)
        return 1
    finally:
        session.stop()


def main(argv=None):
    arguments = parser()
    args = arguments.parse_args(argv)
    if args.list_terrains:
        for terrain in discover_terrains(args.root):
            print(f"{terrain.id:32} {terrain.title}")
        return 0
    try:
        limits = validate_arguments(args)
    except (ValueError, OSError, TypeError) as error:
        arguments.error(str(error))
    session = SimulationSession(args.root, args.output)
    if not args.replay and args.terrain not in session.terrains:
        arguments.error(f"Unknown terrain {args.terrain!r}; use --list-terrains")
    for key in ("ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_LOCALHOST_ONLY", "GZ_PARTITION", "GZ_SIM_RESOURCE_PATH"):
        os.environ[key] = session.env[key]
    previous = signal.getsignal(signal.SIGTERM)
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        return run_replay(args, session) if args.replay else run_simulation(args, session, limits)
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    raise SystemExit(main())
