"""Generate a simulation video, encoder angles and three robot trajectory plots."""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def parser():
    cli = argparse.ArgumentParser(description=__doc__, prog="visualize_caster_bag",
        epilog="Example: visualize_caster_bag /data/encoder_run/bags/combined\n"
               "Default: hamr_control/analysis/encoder_run/index.html (five sections only).",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument("source", type=Path, help="Recording folder, rosbag directory, .mcap/.db3, or packets.jsonl")
    cli.add_argument("--output", "-o", type=Path, help="New/empty directory; default: hamr_control/analysis/<recording-name>")
    cli.add_argument("--render", choices=("video", "auto", "preview", "none"), default="video",
                     help="video (default), preview, none, or auto (preview if FFmpeg is absent)")
    cli.add_argument("--ekf-alignment", choices=("initial-pose", "rigid-fit"), default="initial-pose",
                     help="Align the first shared pose (default), or fit rotation/translation over the shared run")
    cli.add_argument("--preview-time", type=float, help="Preview seconds from encoder start; implies --render preview")
    cli.add_argument("--wheel", choices=("both", "front_left", "front_right", "legacy"), default="both")
    cli.add_argument("--fps", type=int, default=30)
    cli.add_argument("--rate", type=float, default=1.)
    cli.add_argument("--width", type=int, default=1600)
    cli.add_argument("--center-reference", default="auto", help="Saved CENTER reference: auto, none, or YAML path")
    cli.add_argument("--left-mapping-config", type=Path)
    cli.add_argument("--right-mapping-config", type=Path)
    cli.add_argument("--recover-timing", action="store_true", help="Allow approximate timing recovery from saved probes")
    cli.add_argument("--full-detail", action="store_true", help="Render small internal CAD bearings/fasteners")
    return cli


def repository_root():
    # Source execution and common colcon install layouts both resolve to the checkout.
    for anchor in (Path(__file__).resolve(), Path.cwd()):
        for parent in (anchor, *anchor.parents):
            for candidate in (parent, parent / "src/hamr_control", parent / "hamr_control"):
                if (candidate / "hamr_bringup/CMakeLists.txt").is_file():
                    return candidate
    raise ValueError("Cannot locate the hamr_control checkout; run from it or pass --output")


def validate_args(args):
    from caster_bag_reader import resolve_source
    if not math.isfinite(args.rate) or args.rate <= 0 or not 1 <= args.fps <= 120:
        raise ValueError("Require a positive finite --rate and --fps in 1..120")
    if args.width < 800 or args.width % 32:
        raise ValueError("--width must be >=800 and divisible by 32")
    if args.preview_time is not None and (not math.isfinite(args.preview_time) or args.preview_time < 0):
        raise ValueError("--preview-time must be finite and nonnegative")
    recording = resolve_source(args.source)
    args.source = recording.source
    args.recording_name = recording.root.name
    args.output = (args.output or repository_root() / "analysis" / args.recording_name).expanduser().resolve()
    if args.output == recording.root or recording.root in args.output.parents:
        raise ValueError("Choose an output directory outside the original recording")
    if args.output.exists() and (not args.output.is_dir() or any(args.output.iterdir())):
        raise ValueError(f"Output directory is not empty: {args.output}; use --output for another report")
    for name in ("left_mapping_config", "right_mapping_config"):
        value = getattr(args, name)
        if value:
            value = value.expanduser().resolve()
            if not value.is_file():
                raise ValueError(f"Missing {name}: {value}")
            setattr(args, name, value)
    if args.preview_time is not None:
        if args.render == "none":
            raise ValueError("--preview-time cannot be used with --render none")
        args.render = "preview"


def trajectory_for_replay(recording, start, end):
    from caster_pair_trajectory import PREFERRED_TOPICS
    for topic in PREFERRED_TOPICS:
        rows = recording.poses.get(topic, [])
        if not rows or len(recording.frames[topic]) != 1:
            continue
        selected = [row for row in rows if start <= row["stamp_ns"] <= end
                    and all(math.isfinite(row[axis]) for axis in "xyz")]
        if selected:
            return selected, dict(available=True, topic=topic, frame_id=next(iter(recording.frames[topic])),
                                   source_bag=str(recording.source), time_shift_ns=0,
                                   time_basis="Recorded bag timestamps; no clock shift", reason="")
    return [], dict(available=False, topic=None, source_bag=str(recording.source),
                    time_shift_ns=0, reason="No same-run base trajectory overlaps the encoder interval")


def render_worker(task_path):
    import resource
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    from caster_bag_reader import read_recording, replay_events
    from caster_bag_report import write_json
    from caster_local_recording import check_dependencies
    from render_caster_pair_recording import render_pair
    task = json.loads(Path(task_path).read_text())
    args = argparse.Namespace(**task)
    args.mapping_config = None
    args.start_pose = "recorded"
    args.no_plots = False
    args.trajectory_bag = args.trajectory_topic = None
    check_dependencies(True, video=args.render == "video")
    recording = read_recording(args.source)
    selected = ("front_left", "front_right") if args.wheel == "both" else (args.wheel,)
    events, provenance = replay_events(recording, args.recover_timing, selected)
    duration = (provenance["end_ns"]-provenance["start_ns"])/1e9
    if args.render == "preview" and args.preview_time is None:
        args.preview_time = duration/2
    target = Path(args.output) / "replay"
    if args.wheel == "both":
        result = render_pair(args, Path(args.source), target, (events, provenance),
                             trajectory_for_replay(recording, provenance["start_ns"], provenance["end_ns"]))
    else:
        from caster_local_recording import write_replay_bag
        from render_caster_encoder_video import render
        bag = target / "encoder_replay"
        target.mkdir(parents=True, exist_ok=True)
        write_replay_bag(bag, events[args.wheel], dict(provenance, wheel_id=args.wheel))
        args.bag, args.output_dir, args.topic, args.plots = bag, target, None, True
        args.mapping_config = (args.left_mapping_config if args.wheel == "front_left" else args.right_mapping_config)
        if args.mapping_config:
            args.mapping_config = Path(args.mapping_config)
        render(args)
        result = dict(mode="single_caster_kinematic_replay")
    write_json(Path(args.output)/"replay_result.json", result)


def run_renderer(args):
    if args.render == "none":
        return dict(status="disabled", detail="Video generation was disabled with --render none.")
    mode = args.render
    if mode == "auto":
        mode = "video" if shutil.which("ffmpeg") else "preview"
    environment = dict(os.environ)
    if not environment.get("DISPLAY") and Path("/tmp/.X11-unix/X0").exists():
        environment["DISPLAY"] = ":0"
    # The renderer's diagnostic exports are temporary. Keep only the requested media.
    with tempfile.TemporaryDirectory(prefix=".caster-render-", dir=args.output.parent) as temporary:
        scratch = Path(temporary)
        task = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
        task.update(render=mode, output=str(scratch))
        task_path = scratch / "request.json"
        task_path.write_text(json.dumps(task, indent=2) + "\n")
        command = [sys.executable, str(Path(__file__).resolve()), "--render-worker", str(task_path)]
        if not environment.get("DISPLAY") and shutil.which("xvfb-run"):
            command = ["xvfb-run", "-a", *command]
        print(f"Rendering simulation {mode} for {args.wheel}...", flush=True)
        with (scratch / "render.log").open("w") as log:
            completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=environment)
        if completed.returncode:
            log = (scratch / "render.log").read_text(errors="replace")
            (args.output / "simulation.log").write_text(log)
            return dict(status="failed", detail="Simulation could not be generated: " +
                        (log.strip().splitlines()[-1] if log.strip() else "see simulation.log"))
        result = json.loads((scratch / "replay_result.json").read_text())
        candidates = list((scratch / "replay").glob("*.mp4" if mode == "video" else "*_preview.png"))
        if len(candidates) != 1:
            return dict(status="failed", detail="Renderer did not produce exactly one requested media file.")
        filename = "simulation.mp4" if mode == "video" else "simulation_preview.png"
        shutil.copy2(candidates[0], args.output / filename)
        metadata = {key: result[key] for key in (
            "mode", "recording_duration_s", "video_duration_s", "frames", "fps", "rate", "width", "height",
            "timing", "references", "mapping", "robot_body_animated", "video_sha256") if key in result}
    return dict(status="complete", file=filename,
                detail=("Recorded caster joint motion on the simulation model; the chassis stays fixed. "
                        "The trajectory inset follows the recorded robot. This is a kinematic replay."),
                metadata=metadata)


def run(args):
    from caster_bag_reader import read_recording, digest
    from caster_bag_report import packet_reports, write_json
    from caster_focused_report import generate_plots, write_index
    recording = read_recording(args.source)
    args.output.mkdir(parents=True, exist_ok=True)
    # Packet-only inputs can still supply angles, without exporting packet diagnostics.
    if len(recording.encoder_topics()) < 6 and recording.packet_streams():
        with tempfile.TemporaryDirectory(prefix=".caster-angles-", dir=args.output.parent) as temporary:
            packet_reports(recording, Path(temporary), plots=False)
    print(f"Read {sum(len(t) for t in recording.times.values()):,} messages; generating four plots...", flush=True)
    manifest = dict(version=2, source=str(args.source), recording_name=args.recording_name,
                    source_sha256={str(path): digest(path) for path in recording.files},
                    arguments={key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
                    decode_issues=recording.issues,
                    stages=generate_plots(recording, args.output, args.ekf_alignment))
    manifest["stages"]["simulation"] = run_renderer(args)
    manifest["status"] = "partial" if recording.issues or any(
        stage["status"] in ("failed", "partial", "unavailable") for stage in manifest["stages"].values()) else "complete"
    write_json(args.output / "report.json", manifest)
    write_index(args.output, manifest)
    for name, stage in manifest["stages"].items():
        print(f"{name}: {stage['status']} — {stage['detail']}", flush=True)
    print(f"Open {args.output / 'index.html'}", flush=True)
    return 1 if manifest["stages"]["simulation"]["status"] == "failed" else 0


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) == 2 and argv[0] == "--render-worker":
        render_worker(argv[1])
        return 0
    cli = parser()
    args = cli.parse_args(argv)
    try:
        validate_args(args)
        return run(args)
    except (ValueError, OSError, RuntimeError, ImportError, KeyError, TypeError) as error:
        cli.exit(2, f"visualize_caster_bag: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
