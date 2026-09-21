#!/usr/bin/env python3
"""Launch the full caster-equipped vehicle, record one route to MP4, then stop."""
import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory

PROFILE_NAMES = {"smooth": "smooth", "continuous": "continuous", "legacy": "simple"}


def continuous_video_quality(trajectory, recording, max_gap_s=.16):
    """Check encoded source-image gaps overlapping actual planned motion.

    Source timestamps count images that reached the encoder, so dropped camera
    or queue frames cannot be hidden by constant-rate duplicate video frames.
    Startup/final-only gaps do not affect this motion-quality check.
    """
    result = {"passed": False, "max_allowed_motion_image_gap_s": max_gap_s,
              "failure": None, "motion_gaps_over_limit": []}
    try:
        if not math.isfinite(max_gap_s) or max_gap_s <= 0:
            raise ValueError("Maximum image gap must be finite and positive")
        sample = next(s for s in trajectory["samples"] if s["phase"] == "motion")
        start = float(sample["odometry_stamp_s"]) - float(sample["trajectory_time_s"])
        duration = float(trajectory["summary"]["reference_duration_s"])
        stamps = [float(value) for value in recording["encoded_source_timestamps_s"]]
        if not math.isfinite(start) or not math.isfinite(duration) or duration <= 0:
            raise ValueError("Motion interval must be finite with positive duration")
        if len(stamps) < 2 or not all(math.isfinite(stamp) for stamp in stamps):
            raise ValueError("At least two finite encoded source-image timestamps are required")
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            raise ValueError("Encoded source-image timestamps must increase strictly")
        end = start + duration
        result["motion_interval_sim_s"] = [start, end]
        result["encoded_source_image_count"] = len(stamps)
        if stamps[0] > start + 1e-9 or stamps[-1] < end - 1e-9:
            raise ValueError("Encoded source images do not cover the complete motion interval")
        gaps = [{"start_sim_s": a, "end_sim_s": b, "gap_s": b-a}
                for a, b in zip(stamps, stamps[1:]) if b > start and a < end]
        result["motion_image_gap_count"] = len(gaps)
        result["max_motion_image_gap_s"] = max(gap["gap_s"] for gap in gaps)
        result["motion_gaps_over_limit"] = [gap for gap in gaps if gap["gap_s"] > max_gap_s + 1e-9]
        if result["motion_gaps_over_limit"]:
            raise ValueError(f"Encoded source-image gap during motion exceeded {max_gap_s:.2f} s: "
                             f"{result['max_motion_image_gap_s']:.3f} s")
        result["passed"] = True
    except (KeyError, TypeError, ValueError, StopIteration) as exc:
        result["failure"] = str(exc) or "Motion samples are missing"
    return result


def stop_process(process, timeout=25, signal_group=True):
    if process is None:
        return
    try:
        if signal_group:
            os.killpg(process.pid, signal.SIGINT)
        elif process.poll() is None:
            process.send_signal(signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
    # A launcher can exit before its shell's Gazebo child. Its process group
    # belongs exclusively to this run; never use a global pkill / process name.
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=tuple(PROFILE_NAMES), default="smooth",
                        help="Smooth waypoint stops, continuous rounded corners, or the original abrupt reference")
    parser.add_argument("--config", type=Path,
                        help="Override the selected profile's installed controller/planner configuration")
    parser.add_argument("--output-dir", type=Path,
                        help="New or empty directory; default: ~/Videos/hamr_sim/<profile>_waypoint_<timestamp>")
    parser.add_argument("--speed", type=float,
                        help="Override configured reference speed limit in m/s")
    parser.add_argument("--real-time-factor", type=float,
                        help="Wall-time capture rate: defaults to 0.15 for continuous, 0.5 otherwise; MP4 uses simulation time")
    parser.add_argument("--gui", action="store_true", help="Also open the live Gazebo GUI")
    parser.add_argument("--domain-id", type=int, default=131,
                        help="Dedicated local ROS domain for this run")
    parser.add_argument("--wall-timeout", type=float, default=1800.)
    args = parser.parse_args()
    if args.real_time_factor is None:
        args.real_time_factor = .15 if args.profile == "continuous" else .5
    if args.speed is not None and (not math.isfinite(args.speed) or not 0 < args.speed <= .5):
        parser.error("--speed must be in (0, 0.5] m/s")
    if not math.isfinite(args.real_time_factor) or not 0 < args.real_time_factor <= 1:
        parser.error("--real-time-factor must be in (0, 1]")
    if not math.isfinite(args.wall_timeout) or args.wall_timeout < 60:
        parser.error("--wall-timeout must be finite and at least 60 seconds")
    if not 0 <= args.domain_id <= 232:
        parser.error("--domain-id must be in [0, 232]")
    default_output = Path.home()/"Videos/hamr_sim"/datetime.now().strftime(
        f"{PROFILE_NAMES[args.profile]}_waypoint_%Y%m%d_%H%M%S")
    output = (args.output_dir or default_output).expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        parser.error(f"Output directory is not empty; choose a new directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    share = Path(get_package_share_directory("hamr_ball_caster"))
    config = (args.config.expanduser().resolve() if args.config else
              share/"config"/f"{PROFILE_NAMES[args.profile]}_waypoint_sim.yaml")
    if not config.is_file():
        parser.error(f"Configuration file does not exist: {config}")
    # Run the exact bytes retained with this recording. Editing the source
    # configuration during a long capture cannot change its recorded settings.
    config_bytes = config.read_bytes()
    snapshot = output/"configuration.yaml"
    snapshot.write_bytes(config_bytes)
    config_sha256 = hashlib.sha256(config_bytes).hexdigest()
    scripts = Path(__file__).resolve().parent
    env = dict(os.environ)
    env.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
               GZ_PARTITION=f"hamr_waypoint_movie_{os.getpid()}")
    video = output/f"hamr_{PROFILE_NAMES[args.profile]}_waypoint.mp4"
    report = output/"trajectory.json"
    ready = output/"camera_ready.json"
    recording_metadata = output/"video_metadata.json"
    source_world = share/"worlds/hamr_waypoint_recording.sdf"
    world = source_world
    preparation = None
    if args.profile == "continuous":
        plan_file = output/"reference_plan.json"
        plan_command = [sys.executable, str(scripts/"run_waypoint_sim.py"),
                        "--profile", "continuous", "--config", str(snapshot),
                        "--plan-only", str(plan_file)]
        if args.speed is not None:
            plan_command.extend(("--speed", str(args.speed)))
        print("Preparing the continuous reference and its Gazebo floor overlay.", flush=True)
        try:
            planned = subprocess.run(plan_command, env=env, capture_output=True, text=True, timeout=60)
            (output/"planning.log").write_text(planned.stdout + planned.stderr)
            if planned.returncode:
                parser.error(f"Continuous planning failed; see {output/'planning.log'}")
            plan_report = json.loads(plan_file.read_text())
            if plan_report["config_sha256"] != config_sha256:
                parser.error("Planned reference configuration does not match the retained snapshot")
            # Support both installed scripts and direct execution from source.
            helper_locations = (Path(__file__).absolute().parent/"prepare_continuous_world.py",
                                scripts.parent/"tools/prepare_continuous_world.py")
            helper = next((path for path in helper_locations if path.is_file()), None)
            if helper is None:
                parser.error("prepare_continuous_world.py is missing; rebuild hamr_ball_caster")
            sys.path.insert(0, str(helper.parent))
            from prepare_continuous_world import prepare_continuous_world
            world = output/"recording_world.sdf"
            world_provenance = prepare_continuous_world(source_world, world, plan_report["continuous_plan"])
            preparation = {"command": plan_command, "plan_file": str(plan_file),
                           "plan_sha256": hashlib.sha256(plan_file.read_bytes()).hexdigest(),
                           "source_world": str(source_world),
                           "source_world_sha256": hashlib.sha256(source_world.read_bytes()).hexdigest(),
                           "generated_world": str(world),
                           "generated_world_sha256": hashlib.sha256(world.read_bytes()).hexdigest(),
                           "overlay": world_provenance}
        except (OSError, ValueError, KeyError, subprocess.TimeoutExpired) as exc:
            parser.error(f"Unable to prepare continuous recording: {exc}")
    commands = {
        "launch": ["ros2", "launch", "hamr_ball_caster", "ball_caster.launch.py",
                   "model:=compa", "controller:=false", f"gui:={str(args.gui).lower()}",
                   "visual_detail:=overview",
                   f"world:={world}",
                   f"gui_config:={share/'config/waypoint_gui.config'}",
                   f"real_time_factor:={args.real_time_factor}"],
        "record": [sys.executable, str(scripts/"record_simulation.py"), "--output", str(video),
                   "--ready-file", str(ready), "--metadata", str(recording_metadata),
                   "--wall-timeout", str(args.wall_timeout)],
        "trajectory": [sys.executable, str(scripts/"run_waypoint_sim.py"), "--output", str(report),
                       "--profile", args.profile, "--config", str(snapshot),
                       "--wall-timeout", str(args.wall_timeout)],
    }
    if args.speed is not None:
        commands["trajectory"].extend(("--speed", str(args.speed)))
    (output/"commands.json").write_text(json.dumps({"commands": commands,
        "continuous_preparation": preparation,
        "configuration": {"source": str(config), "snapshot": str(snapshot),
                          "sha256": config_sha256, "speed_override_m_s": args.speed},
        "environment": {key: env[key] for key in ("ROS_DOMAIN_ID", "GZ_PARTITION", "ROS_AUTOMATIC_DISCOVERY_RANGE")}}, indent=2)+"\n")
    print(f"Recording full vehicle run to {output}", flush=True)
    simulator = recorder = runner = None
    failure = None
    runner_code = None
    started = time.monotonic()
    with (output/"gazebo.log").open("w") as gz_log, (output/"recording.log").open("w") as video_log, (output/"trajectory.log").open("w") as trajectory_log:
        try:
            simulator = subprocess.Popen(commands["launch"], env=env, stdout=gz_log,
                                         stderr=subprocess.STDOUT, start_new_session=True)
            recorder = subprocess.Popen(commands["record"], env=env, stdout=video_log,
                                        stderr=subprocess.STDOUT, start_new_session=True)
            while not ready.exists():
                if simulator.poll() is not None:
                    raise RuntimeError("Gazebo exited before camera readiness; see gazebo.log")
                if recorder.poll() is not None:
                    raise RuntimeError("Recorder exited before first frame; see recording.log")
                if time.monotonic()-started > 180:
                    raise RuntimeError("No camera frame within 180 seconds; see recording.log and gazebo.log")
                time.sleep(.2)
            print(f"Camera is recording. Starting {args.profile} waypoint route.", flush=True)
            runner = subprocess.Popen(commands["trajectory"], env=env, stdout=trajectory_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            last_message = time.monotonic()
            while runner.poll() is None:
                if recorder.poll() is not None:
                    raise RuntimeError("Recorder stopped during the route; see recording.log")
                if simulator.poll() is not None:
                    raise RuntimeError("Gazebo stopped during the route; see gazebo.log")
                elapsed = time.monotonic()-started
                if elapsed > args.wall_timeout:
                    raise RuntimeError("Full run exceeded wall-time timeout")
                if time.monotonic()-last_message >= 20:
                    print(f"Run continues; {elapsed:.0f} wall seconds elapsed.", flush=True)
                    last_message = time.monotonic()
                time.sleep(.2)
            runner_code = runner.returncode
            if runner_code:
                raise RuntimeError("Trajectory checks failed; video retained for diagnosis. See trajectory.json")
            print("Route finished. Finalizing MP4.", flush=True)
        except (RuntimeError, KeyboardInterrupt) as exc:
            failure = str(exc) or "Interrupted"
            print(failure, file=sys.stderr, flush=True)
        finally:
            # Stop velocity publication while physics is still running; the
            # trajectory runner zeros all powered-joint commands on SIGINT.
            stop_process(runner)
            # Signal the Python recorder only: it drains frames and closes
            # ffmpeg's stdin itself. Interrupting ffmpeg simultaneously truncates
            # the queue and can prevent clean MP4 finalization.
            stop_process(recorder, timeout=90, signal_group=False)
            stop_process(simulator)
    result = {"passed": failure is None, "failure": failure, "trajectory_exit_code": runner_code,
              "video": str(video), "trajectory_report": str(report),
              "wall_duration_s": time.monotonic()-started,
              "trajectory_profile": args.profile, "configuration_file": str(snapshot),
              "source_configuration_file": str(config), "configuration_sha256": config_sha256,
              "requested_speed_override_m_s": args.speed, "target_real_time_factor": args.real_time_factor,
              "scope": "Gazebo-rendered camera, simulation-time MP4, complete COMPA/HAMR retrofit with two passive CAD casters"}
    if preparation is not None:
        result["continuous_preparation"] = preparation
    if report.exists():
        trajectory = json.loads(report.read_text())
        result["trajectory_summary"] = trajectory["summary"]
        result["reference_speed_m_s"] = trajectory["speed_m_s"]
        if not trajectory["summary"]["passed"]:
            result.update(passed=False, failure=failure or "Trajectory acceptance checks failed")
        if args.profile == "continuous":
            missing = [field for field in ("continuous_plan", "continuous_acceptance") if field not in trajectory]
            if missing:
                result.update(passed=False, failure="Continuous trajectory report is missing " + ", ".join(missing))
            else:
                result["continuous_plan"] = trajectory["continuous_plan"]
                result["continuous_acceptance"] = trajectory["continuous_acceptance"]
    else:
        result.update(passed=False, failure="Trajectory runner did not produce a report")
    if recording_metadata.exists():
        result["recording"] = json.loads(recording_metadata.read_text())
        if not result["recording"].get("success", False):
            result.update(passed=False, failure="Recorder reported errors; see video_metadata.json")
        elif report.exists() and trajectory.get("samples"):
            samples = trajectory["samples"]
            first = result["recording"]["first_sim_time_s"]
            last = result["recording"]["last_sim_time_s"]
            if first > samples[0]["simulation_time_s"] or last < samples[-1]["simulation_time_s"] - .25:
                result.update(passed=False, failure="Camera timestamps do not cover the complete trajectory run")
    else:
        result.update(passed=False, failure="Recorder did not produce timing metadata")
    if args.profile == "continuous" and report.exists() and "recording" in result:
        result["video_quality"] = continuous_video_quality(trajectory, result["recording"])
        if not result["video_quality"]["passed"]:
            result.update(passed=False, failure=result["video_quality"]["failure"])
    ffprobe = shutil.which("ffprobe")
    if not video.exists() or video.stat().st_size < 20000:
        result.update(passed=False, failure="MP4 is absent or unexpectedly small")
    elif ffprobe:
        probe = subprocess.run([ffprobe, "-v", "error", "-select_streams", "v:0", "-show_entries",
                                "stream=codec_name,width,height,pix_fmt,avg_frame_rate,nb_frames:format=duration,size",
                                "-of", "json", str(video)], capture_output=True, text=True)
        if probe.returncode:
            result.update(passed=False, failure="ffprobe could not read the completed MP4")
        else:
            result["video_probe"] = json.loads(probe.stdout)
            (output/"video_probe.json").write_text(probe.stdout)
            expected = result.get("trajectory_summary", {}).get("reference_duration_s")
            if expected is not None and float(result["video_probe"]["format"]["duration"]) < expected:
                result.update(passed=False, failure="MP4 is shorter than the reference motion")
    else:
        result.update(passed=False, failure="ffprobe is required to verify the completed MP4")
    (output/"run_summary.json").write_text(json.dumps(result, indent=2)+"\n")
    print(f"{'PASS' if result['passed'] else 'FAIL'}: {video}", flush=True)
    print(f"Measurements: {output/'run_summary.json'}", flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
