#!/usr/bin/env python3
"""Run the physical swivel-caster vehicle and optionally record two Gazebo views.

The existing HAMR waypoint planner, controller, measurement checks and camera
encoder are shared with hamr_ball_caster. This harness owns an isolated ROS
domain and Gazebo partition and stops only the processes it starts.
"""
import argparse
from contextlib import ExitStack
import csv
from datetime import datetime, timezone
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
import xml.etree.ElementTree as ET

import yaml

from prepare_swivel_world import prepare_world

CAMERAS = {
    "overview": "/hamr/recording_camera/image",
    "caster_closeup": "/hamr/swivel_caster/closeup/image",
}


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + "\n")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _interrupt(_signum, _frame):
    raise KeyboardInterrupt


def heading_measurements(trajectory, requested_yaw_deg):
    """Measure the chassis turning toward the first world-frame path segment."""
    samples = trajectory["samples"]
    points = trajectory.get("executed_route_points_m", trajectory["configuration"]["points_m"])
    target = math.atan2(points[1][1]-points[0][1], points[1][0]-points[0][0])
    wrap = lambda angle: math.atan2(math.sin(angle), math.cos(angle))
    initial = samples[0]["base_rpy_rad"][2]
    first_leg = [s for s in samples if s["phase"] == "motion" and s["segment"] == 0]
    if not first_leg:
        raise ValueError("No feedback covers the first leg")
    last = first_leg[-1]
    aligned = next((s for s in first_leg if abs(wrap(s["base_rpy_rad"][2]-target)) <= math.radians(5)), None)
    forward_speeds = [math.cos(s["base_rpy_rad"][2])*s["estimated_world_velocity_m_s"][0] +
                      math.sin(s["base_rpy_rad"][2])*s["estimated_world_velocity_m_s"][1] for s in first_leg]
    result = {
        "requested_initial_chassis_yaw_deg": requested_yaw_deg,
        "measured_initial_chassis_yaw_deg": math.degrees(initial),
        "initial_heading_error_deg": math.degrees(abs(wrap(initial-math.radians(requested_yaw_deg)))),
        "first_leg_direction_deg": math.degrees(target),
        "first_leg_final_chassis_yaw_deg": math.degrees(last["base_rpy_rad"][2]),
        "first_leg_final_heading_error_deg": math.degrees(abs(wrap(last["base_rpy_rad"][2]-target))),
        "time_to_within_5deg_of_first_leg_s": aligned["trajectory_time_s"] if aligned else None,
        "position_when_within_5deg_m": aligned["position_m"][:2] if aligned else None,
        "minimum_body_forward_speed_first_leg_m_s": min(forward_speeds),
        "maximum_body_forward_speed_first_leg_m_s": max(forward_speeds),
        "forward_motion_fraction_first_leg": sum(v >= -.005 for v in forward_speeds)/len(forward_speeds),
        "first_leg_translation_m": math.dist(first_leg[0]["position_m"][:2], last["position_m"][:2]),
        "source": "Measured chassis odometry; no prescribed chassis yaw or caster angles",
    }
    result["passed"] = result["initial_heading_error_deg"] < 2.
    if trajectory["configuration"].get("expect_first_leg_alignment", False):
        result["passed"] &= (aligned is not None and result["first_leg_final_heading_error_deg"] < 5.
                             and result["forward_motion_fraction_first_leg"] > .95)
    return result


def export_caster_measurements(trajectory, output, require_swivel=True):
    """Export actual free-joint feedback, not commanded or animated positions."""
    names = [f"{side}_caster_{kind}_joint" for side in ("left", "right")
             for kind in ("swivel", "wheel")]
    samples = trajectory.get("samples", [])
    if not samples:
        raise ValueError("Trajectory has no measured caster samples")
    fields = ["simulation_time_s", "trajectory_time_s", "phase"]
    fields += [name + "_position_rad" for name in names]
    fields += [name + "_velocity_rad_s" for name in names]
    with (output / "caster_joints.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for sample in samples:
            row = {key: sample[key] for key in fields[:3]}
            for name in names:
                row[name + "_position_rad"] = sample["joint_positions_rad"][name]
                row[name + "_velocity_rad_s"] = sample["joint_velocities_rad_s"][name]
            writer.writerow(row)
    joints = {}
    for name in names:
        positions = [float(s["joint_positions_rad"][name]) for s in samples]
        velocities = [float(s["joint_velocities_rad_s"][name]) for s in samples]
        if not all(math.isfinite(value) for value in positions + velocities):
            raise ValueError(f"Non-finite passive joint feedback: {name}")
        joints[name] = {"initial_position_rad": positions[0], "final_position_rad": positions[-1],
                       "min_position_rad": min(positions), "max_position_rad": max(positions),
                       "position_span_rad": max(positions)-min(positions),
                       "max_abs_velocity_rad_s": max(abs(v) for v in velocities)}
    result = {"source": "Measured Gazebo JointState position and velocity",
              "sample_count": len(samples), "joints": joints,
              "csv": str(output / "caster_joints.csv")}
    # The intended route exercises steering and rolling on each front caster.
    result["swivel_motion_required"] = require_swivel
    result["passed"] = all(joints[name]["position_span_rad"] >
                           (.25 if "swivel" in name else 1.) for name in names
                           if require_swivel or "wheel" in name)
    write_json(output / "caster_joint_summary.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", action="store_true", help="Save separate overview and side-closeup MP4s")
    parser.add_argument("--gui", action="store_true", help="Open Gazebo GUI (headless server is the default)")
    parser.add_argument("--output-dir", type=Path, help="New or empty directory for all results")
    parser.add_argument("--config", type=Path, help="Override the selected waypoint configuration")
    parser.add_argument("--physical-config", type=Path, help="Complete caster physical YAML override; saved with the run")
    parser.add_argument("--xacro-arg", action="append", default=[], metavar="NAME=VALUE",
                        help="Numeric caster sensitivity override (repeatable); recorded in provenance")
    parser.add_argument("--ground-mu", type=float, help="Override both floor friction coefficients")
    parser.add_argument("--physics-only", action="store_true", help="Disable the rendering system for unrecorded screening")
    parser.add_argument("--profile", choices=("smooth", "legacy"), default="smooth",
                        help="Smooth compact route, or original waypoint_traj_simple geometry and timing")
    parser.add_argument("--initial-yaw-deg", type=float, default=0.,
                        help="Initial chassis world heading in degrees; 180 reverses the normal start")
    parser.add_argument("--domain-id", type=int, default=89)
    parser.add_argument("--real-time-factor", type=float, help="Default 0.10 for recording, 1.0 otherwise")
    parser.add_argument("--max-step-size", type=float, default=.001)
    parser.add_argument("--initial-swivel", type=float, default=0., help="Both passive swivel starting angles, radians")
    parser.add_argument("--wall-timeout", type=float, default=1800.)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error("--domain-id must be in [0, 232]")
    if args.real_time_factor is None:
        args.real_time_factor = .10 if args.record else 1.
    if not math.isfinite(args.real_time_factor) or not 0 < args.real_time_factor <= 1:
        parser.error("--real-time-factor must be in (0, 1]")
    if not math.isfinite(args.max_step_size) or not 0 < args.max_step_size <= .01:
        parser.error("--max-step-size must be in (0, .01]")
    if not math.isfinite(args.initial_swivel):
        parser.error("--initial-swivel must be finite")
    if not math.isfinite(args.initial_yaw_deg):
        parser.error("--initial-yaw-deg must be finite")
    if args.physics_only and (args.record or args.gui):
        parser.error("--physics-only cannot be combined with --record or --gui")
    if args.ground_mu is not None and (not math.isfinite(args.ground_mu) or args.ground_mu < 0):
        parser.error("--ground-mu must be finite and nonnegative")
    allowed_xacro = {"caster_mass_scale", "caster_wheel_mass_scale", "caster_fork_mass_scale",
                     "caster_wheel_inertia_scale", "caster_fork_inertia_scale", "swivel_damping",
                     "swivel_friction", "wheel_damping", "wheel_friction", "tire_mu", "caster_trail"}
    xacro_overrides = {}
    for item in args.xacro_arg:
        try:
            name, raw = item.split("=", 1)
            value = float(raw)
        except ValueError:
            parser.error("--xacro-arg must be NAME=NUMBER")
        if name not in allowed_xacro or not math.isfinite(value) or value < 0:
            parser.error(f"Invalid numeric caster override: {item}")
        if ("scale" in name or name == "caster_trail") and value <= 0:
            parser.error(f"Caster mass, inertia and trail must be positive: {item}")
        xacro_overrides[name] = value
    if not math.isfinite(args.wall_timeout) or args.wall_timeout < 60:
        parser.error("--wall-timeout must be finite and at least 60 seconds")
    if args.record and (not shutil.which("ffmpeg") or not shutil.which("ffprobe")):
        parser.error("Recording requires ffmpeg and ffprobe in PATH")

    # Defer ROS imports so --help also works in a fresh terminal.
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    share = Path(get_package_share_directory("hamr_swivel_caster"))
    ball_bin = Path(get_package_prefix("hamr_ball_caster")) / "lib/hamr_ball_caster"
    sys.path.insert(0, str(ball_bin))
    from evaluate_waypoint_profile import stop_process_group
    from record_waypoint_run import continuous_video_quality

    default_config = "simple_waypoint_sim.yaml" if args.profile == "legacy" else "waypoint_sim.yaml"
    config = (args.config or share / "config" / default_config).expanduser().resolve()
    if not config.is_file():
        parser.error(f"Missing configuration: {config}")
    cfg = yaml.safe_load(config.read_bytes())
    if len(cfg.get("passive_joint_names", [])) != 4:
        parser.error("Configuration must specify the four passive swivel/wheel joint names")
    output = (args.output_dir or Path.home() / "Videos/hamr_sim" /
              datetime.now().strftime("swivel_caster_%Y%m%d_%H%M%S_%f")).expanduser().resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        parser.error(f"Output must be a new or empty directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    snapshot = output / "configuration.yaml"
    snapshot.write_bytes(config.read_bytes())
    physical_source = (args.physical_config or share / "config/physical_properties.yaml").expanduser().resolve()
    if not physical_source.is_file():
        parser.error(f"Missing physical configuration: {physical_source}")
    if physical_source.is_file():
        shutil.copyfile(physical_source, output / "physical_properties.yaml")
    world = output / "world.sdf"
    world_preparation = prepare_world(share / "worlds/swivel_caster_test.sdf", world,
                                     cfg["points_m"], args.max_step_size, args.real_time_factor)
    if args.physics_only or args.ground_mu is not None:
        tree = ET.parse(world)
        world_node = tree.getroot().find("world")
        if args.physics_only:
            for plugin in list(world_node.findall("plugin")):
                if "sensors-system" in plugin.get("filename", ""):
                    world_node.remove(plugin)
        if args.ground_mu is not None:
            for tag in ("mu", "mu2"):
                world_node.find(f"./model[@name='ground_plane']/link/collision/surface/friction/ode/{tag}").text = str(args.ground_mu)
        ET.indent(tree, space="  ")
        tree.write(world, encoding="utf-8", xml_declaration=True)
    world_preparation.update(rendering_enabled=not args.physics_only, ground_mu=args.ground_mu)
    robot = output / "robot.urdf"
    xacro_command = ["xacro", str(share / "urdf/compa_swivel_caster.urdf.xacro"),
                     f"initial_swivel_rad:={args.initial_swivel}", "enable_closeup_camera:=true"]
    if args.physical_config:
        xacro_command.append(f"caster_physical_config:={output / 'physical_properties.yaml'}")
    xacro_command.extend(f"{name}:={value}" for name, value in xacro_overrides.items())
    expanded = subprocess.run(xacro_command, capture_output=True, text=True)
    (output / "xacro.log").write_text(expanded.stderr)
    if expanded.returncode:
        parser.error(f"URDF expansion failed; see {output/'xacro.log'}")
    robot.write_text(expanded.stdout)
    report = output / "trajectory.json"
    env = dict(os.environ)
    env.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
               ROS_LOCALHOST_ONLY="1", GZ_PARTITION=f"hamr_swivel_{os.getpid()}_{time.time_ns()}")
    commands = {
        "xacro": xacro_command,
        "launch": ["ros2", "launch", "hamr_swivel_caster", "swivel_caster.launch.py",
                   f"gui:={str(args.gui).lower()}", f"world:={world}", f"robot_description_file:={robot}",
                   f"initial_yaw:={math.radians(args.initial_yaw_deg)}"],
        "trajectory": [sys.executable, str(ball_bin / "run_waypoint_sim.py"),
                       "--profile", args.profile, "--config", str(snapshot), "--output", str(report),
                       "--wall-timeout", str(args.wall_timeout)],
    }
    recordings = {}
    if args.record:
        for name, topic in CAMERAS.items():
            recordings[name] = {"video": output / f"{name}.mp4", "metadata": output / f"{name}_metadata.json",
                                "ready": output / f"{name}_ready.json"}
            item = recordings[name]
            commands["record_" + name] = [sys.executable, str(ball_bin / "record_simulation.py"),
                "--output", str(item["video"]), "--metadata", str(item["metadata"]),
                "--ready-file", str(item["ready"]), "--topic", topic, "--fps", "25",
                "--wall-timeout", str(args.wall_timeout), "--reliable-images"]
    provenance = {"commands": commands, "started_utc": datetime.now(timezone.utc).isoformat(),
                  "configuration_source": str(config), "configuration_sha256": sha256(snapshot),
                  "expanded_robot_sha256": sha256(robot), "world_sha256": sha256(world),
                  "world_preparation": world_preparation,
                  "trajectory_profile": args.profile, "initial_chassis_yaw_deg": args.initial_yaw_deg,
                  "initial_caster_swivel_rad": args.initial_swivel,
                  "physical_configuration_source": str(physical_source),
                  "physical_configuration_sha256": sha256(output / "physical_properties.yaml"),
                  "caster_xacro_overrides": xacro_overrides,
                  "environment": {key: env[key] for key in ("ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE",
                                  "ROS_LOCALHOST_ONLY", "GZ_PARTITION")}}
    write_json(output / "commands.json", provenance)
    print(f"Running swivel caster trajectory in domain {args.domain_id}: {output}", flush=True)

    simulator = runner = None
    recorders = {}
    cleanup = {}
    failure = None
    runner_code = None
    started = time.monotonic()
    previous_term = signal.signal(signal.SIGTERM, _interrupt)
    try:
        with ExitStack() as stack:
            logs = {name: stack.enter_context((output / f"{name}.log").open("w"))
                    for name in ("gazebo", "trajectory", *recordings)}
            try:
                simulator = subprocess.Popen(commands["launch"], env=env, stdout=logs["gazebo"],
                                             stderr=subprocess.STDOUT, start_new_session=True)
                for name in recordings:
                    recorders[name] = subprocess.Popen(commands["record_" + name], env=env,
                        stdout=logs[name], stderr=subprocess.STDOUT, start_new_session=True)
                while recordings and not all(item["ready"].exists() for item in recordings.values()):
                    if simulator.poll() is not None:
                        raise RuntimeError("Gazebo exited before camera readiness; see gazebo.log")
                    for name, process in recorders.items():
                        if process.poll() is not None:
                            raise RuntimeError(f"{name} recorder exited before readiness; see {name}.log")
                    if time.monotonic()-started > min(180., args.wall_timeout):
                        raise RuntimeError("Both cameras did not provide images before readiness timeout")
                    time.sleep(.2)
                if recordings:
                    print("Overview and side closeup are recording; starting trajectory.", flush=True)
                runner = subprocess.Popen(commands["trajectory"], env=env, stdout=logs["trajectory"],
                                          stderr=subprocess.STDOUT, start_new_session=True)
                last_message = time.monotonic()
                while runner.poll() is None:
                    if simulator.poll() is not None:
                        raise RuntimeError("Gazebo exited during trajectory; see gazebo.log")
                    for name, process in recorders.items():
                        if process.poll() is not None:
                            raise RuntimeError(f"{name} recorder exited during trajectory; see {name}.log")
                    elapsed = time.monotonic()-started
                    if elapsed > args.wall_timeout:
                        raise RuntimeError("Run exceeded wall-time timeout")
                    if time.monotonic()-last_message > 20.:
                        print(f"Run continues; {elapsed:.0f} wall seconds elapsed.", flush=True)
                        last_message = time.monotonic()
                    time.sleep(.2)
                runner_code = runner.returncode
                if runner_code:
                    raise RuntimeError("Trajectory acceptance checks failed; see trajectory.json")
            except (RuntimeError, OSError, KeyboardInterrupt) as exc:
                failure = str(exc) or "Interrupted"
                print(failure, file=sys.stderr, flush=True)
            finally:
                previous_int = signal.signal(signal.SIGINT, signal.SIG_IGN)
                signal.signal(signal.SIGTERM, signal.SIG_IGN)
                try:
                    cleanup["trajectory"] = stop_process_group(runner)
                    # Signal recorder parents only, preserving encoder processes
                    # until their queued images have drained and MP4 is finalized.
                    for process in recorders.values():
                        if process.poll() is None:
                            process.send_signal(signal.SIGINT)
                    for name, process in recorders.items():
                        try:
                            process.wait(timeout=90.)
                        except subprocess.TimeoutExpired:
                            failure = failure or f"{name} recorder did not finalize within 90 seconds"
                        cleanup[name] = stop_process_group(process)
                    cleanup["gazebo"] = stop_process_group(simulator)
                finally:
                    signal.signal(signal.SIGINT, previous_int)
    finally:
        signal.signal(signal.SIGTERM, previous_term)

    result = {"passed": failure is None, "failure": failure, "trajectory_exit_code": runner_code,
              "trajectory_profile": args.profile, "initial_chassis_yaw_deg": args.initial_yaw_deg,
              "trajectory_report": str(report), "configuration_sha256": provenance["configuration_sha256"],
              "world_preparation": world_preparation, "robot_description": str(robot),
              "wall_duration_s": time.monotonic()-started, "recorded": args.record,
              "cleanup": cleanup, "scope": "Actual Gazebo physics; two freely swiveling and rolling front casters; estimated mechanical parameters"}
    failures = [failure] if failure else []
    trajectory = None
    try:
        trajectory = json.loads(report.read_text())
        result["trajectory_summary"] = trajectory["summary"]
        result["corner_acceptance"] = trajectory.get("corner_acceptance")
        if not trajectory["summary"]["passed"]:
            failures.append("Trajectory acceptance failed")
        result["heading_measurements"] = heading_measurements(trajectory, args.initial_yaw_deg)
        write_json(output / "heading_summary.json", result["heading_measurements"])
        if not result["heading_measurements"]["passed"]:
            failures.append("Initial chassis heading or first-leg reorientation checks failed")
        result["caster_measurements"] = export_caster_measurements(
            trajectory, output, require_swivel=cfg.get("expect_caster_swivel_motion", True))
        if not result["caster_measurements"]["passed"]:
            failures.append("Caster movement evidence did not meet the configured rolling/swivel requirements")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        failures.append(f"Missing or invalid trajectory feedback: {exc}")
    result["videos"] = {}
    for name, item in recordings.items():
        validation = {"video": str(item["video"]), "passed": False}
        try:
            metadata = json.loads(item["metadata"].read_text())
            validation["metadata"] = str(item["metadata"])
            if not metadata.get("success"):
                raise ValueError("Camera encoder reported failure")
            if trajectory is None or not trajectory.get("samples"):
                raise ValueError("No trajectory samples for camera coverage validation")
            samples = trajectory["samples"]
            if (metadata["first_sim_time_s"] > samples[0]["simulation_time_s"] or
                    metadata["last_sim_time_s"] < samples[-1]["simulation_time_s"]-.25):
                raise ValueError("Camera timestamps do not cover the complete trajectory")
            validation["motion_quality"] = continuous_video_quality(trajectory, metadata)
            if not validation["motion_quality"]["passed"]:
                raise ValueError(validation["motion_quality"]["failure"])
            if not item["video"].is_file() or item["video"].stat().st_size < 20000:
                raise ValueError("MP4 is absent or unexpectedly small")
            probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                "stream=codec_name,width,height,pix_fmt,avg_frame_rate,nb_frames:format=duration,size",
                "-of", "json", str(item["video"])], capture_output=True, text=True, timeout=30.)
            if probe.returncode:
                raise ValueError("ffprobe could not decode the MP4")
            validation["ffprobe"] = json.loads(probe.stdout)
            if not validation["ffprobe"].get("streams"):
                raise ValueError("MP4 has no video stream")
            validation["passed"] = True
        except (OSError, ValueError, KeyError, TypeError, subprocess.TimeoutExpired) as exc:
            validation["failure"] = str(exc)
            failures.append(f"{name}: {exc}")
        result["videos"][name] = validation
    result.update(passed=not failures, failure="; ".join(failures) or None)
    write_json(output / "run_summary.json", result)
    print(f"{'PASS' if result['passed'] else 'FAIL'}: {output/'run_summary.json'}", flush=True)
    if result.get("trajectory_summary"):
        print(json.dumps({key: result["trajectory_summary"].get(key) for key in
                          ("motion_rmse_m", "motion_max_error_m", "final_position_error_m")}), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
