"""One offline report: paired CAD replay, trajectories and all numeric bag channels."""
import argparse
from datetime import datetime, timezone
import html
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
from urllib.parse import quote


def parser():
    cli = argparse.ArgumentParser(description=__doc__, prog="visualize_caster_bag",
        epilog="Example: visualize_caster_bag /data/combined_0.mcap --output ~/hamr_report\n"
               "Open OUTPUT/index.html for replay, every plot, the PDF and CSV downloads.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument("source", type=Path, help="Recording folder, rosbag directory, .mcap/.db3, packets.jsonl, or archived analysis exports")
    cli.add_argument("--output", "-o", type=Path, help="New/empty report directory (default: ~/Videos/hamr_sim/<run>_<timestamp>)")
    cli.add_argument("--render", choices=("auto", "video", "preview", "none"), default="auto",
                     help="auto: video when ffmpeg is available, otherwise PNG preview; none: all analysis/plots only")
    cli.add_argument("--preview-time", type=float, help="PNG preview at seconds from encoder start; implies preview mode")
    cli.add_argument("--wheel", choices=("both", "front_left", "front_right", "legacy"), default="both")
    cli.add_argument("--fps", type=int, default=30)
    cli.add_argument("--rate", type=float, default=1.)
    cli.add_argument("--width", type=int, default=1600)
    cli.add_argument("--center-reference", default="auto", help="Saved per-wheel raw CENTER reference: auto, none, or YAML path")
    cli.add_argument("--left-mapping-config", type=Path)
    cli.add_argument("--right-mapping-config", type=Path)
    cli.add_argument("--recover-timing", action="store_true", help="Explicitly allow approximate replay recovery from saved clock probes")
    cli.add_argument("--comparison-config", type=Path, help="Optional Vicon quality/trajectory-estimation YAML")
    cli.add_argument("--full-detail", action="store_true", help="Include small internal CAD bearings/fasteners")
    cli.add_argument("--no-plots", action="store_true", help="Export CSV/JSON only; also disables CAD rendering")
    reference = cli.add_argument_group("optional independently calibrated encoder reference")
    reference.add_argument("--reference-csv", type=Path)
    reference.add_argument("--reference-time-offset-s", type=float, default=0.)
    reference.add_argument("--reference-sign", action="append", default=[], metavar="NODE=SIGN")
    reference.add_argument("--reference-zero-rad", action="append", default=[], metavar="NODE=ANGLE")
    reference.add_argument("--clock-map", type=Path)
    reference.add_argument("--max-match-dt-s", type=float, default=.02)
    reference.add_argument("--max-continuity-gap-s", type=float, default=.1)
    return cli


def validate_args(args):
    if not math.isfinite(args.rate) or args.rate <= 0 or not 1 <= args.fps <= 120:
        raise ValueError("Require a positive finite --rate and --fps in 1..120")
    if args.width < 800 or args.width % 32:
        raise ValueError("--width must be >=800 and divisible by 32")
    if args.preview_time is not None and (not math.isfinite(args.preview_time) or args.preview_time < 0):
        raise ValueError("--preview-time must be finite and nonnegative")
    source = args.source.expanduser().resolve()
    if not source.exists():
        raise ValueError(f"Input does not exist: {source}")
    args.source = source
    args.output = (args.output or Path.home() / "Videos/hamr_sim" /
                   f"{source.stem}_{datetime.now(timezone.utc):%Y%m%d_%H%M%S_%f}").expanduser().resolve()
    if args.output == source or (source.is_dir() and source in args.output.parents):
        raise ValueError("Choose an output directory outside the original recording")
    if args.output.exists() and (not args.output.is_dir() or any(args.output.iterdir())):
        raise ValueError(f"Output directory is not empty: {args.output}")
    for name in ("left_mapping_config", "right_mapping_config", "comparison_config", "reference_csv", "clock_map"):
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
    if args.no_plots:
        args.render = "none"
    if (not math.isfinite(args.max_match_dt_s) or args.max_match_dt_s < 0
            or not math.isfinite(args.max_continuity_gap_s) or args.max_continuity_gap_s <= 0):
        raise ValueError("Reference match tolerance must be nonnegative; continuity gap must be positive and finite")


def reference_options(args):
    import caster_packet_analysis as analysis
    signs = analysis.node_values(args.reference_sign, "--reference-sign", sign=True)
    zeros = analysis.node_values(args.reference_zero_rad, "--reference-zero-rad")
    analysis.seconds_to_ns(args.reference_time_offset_s)
    if not args.reference_csv and (signs or zeros or args.reference_time_offset_s):
        raise ValueError("Reference transforms require --reference-csv")
    reference = analysis.read_reference(args.reference_csv, args.reference_time_offset_s, signs, zeros) if args.reference_csv else None
    clock = analysis.protocol.parse_json(args.clock_map.read_bytes()) if args.clock_map else None
    mapper = analysis.protocol.ClockMapper(clock)
    mapper.estimated_keys = {analysis.protocol.packet_identity(entry) for entry in (clock or {}).get("mappings", [])
                             if entry.get("timing_estimated") or entry.get("accuracy_validated") is False}
    return dict(reference=reference, mapper=mapper, wheel=args.wheel,
                max_match_dt_s=args.max_match_dt_s, max_continuity_gap_s=args.max_continuity_gap_s)


def write_index(output, manifest):
    def href(path):
        return quote(path.relative_to(output).as_posix(), safe="/")
    pictures = sorted(output.rglob("*.png"), key=lambda path: ("/replay/" not in path.as_posix(), str(path)))
    videos = sorted(path for path in output.rglob("*.mp4") if ".partial." not in path.name)
    files = sorted(path for path in output.rglob("*") if path.is_file() and path.suffix in (".pdf", ".csv", ".json", ".jsonl"))
    items = "\n".join(f'<li><a href="{href(path)}">{html.escape(path.relative_to(output).as_posix())}</a></li>' for path in files)
    media = "\n".join(f'<video controls preload="metadata" src="{href(path)}"></video>' for path in videos)
    media += "\n".join(f'<figure><a href="{href(path)}"><img loading="lazy" src="{href(path)}" alt="{html.escape(path.stem)}"></a>'
                       f'<figcaption>{html.escape(path.relative_to(output).as_posix())}</figcaption></figure>' for path in pictures)
    stages = "".join(f'<li><strong>{html.escape(name)}</strong>: {html.escape(stage["status"])} — '
                     f'{html.escape(stage.get("reason", ""))}</li>' for name, stage in manifest["stages"].items())
    (output / "index.html").write_text(f'''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>HAMR caster recording report</title>
<style>body{{font:16px system-ui,sans-serif;max-width:1300px;margin:32px auto;padding:0 20px;background:#f4f7fa;color:#193047}}
h1{{font-size:28px}}img,video{{max-width:100%;height:auto}}figure{{background:white;padding:12px;margin:24px 0;border-radius:8px}}
figcaption{{font-size:13px;overflow-wrap:anywhere}}a{{color:#1764ab}}li{{margin:8px 0}}code{{overflow-wrap:anywhere}}</style>
<h1>HAMR caster recording report</h1><p>Source: <code>{html.escape(manifest['source'])}</code></p>
<p>Uncalibrated marker trajectories and measured encoder history. CAD replay is kinematic; no contact physics or encoder accuracy is inferred.</p>
<ul>{stages}</ul><details><summary>PDF, CSV and provenance downloads</summary><ul>{items}</ul></details>{media}</html>''')


def assemble_pdf(output):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    from PIL import Image
    pictures = sorted(path for path in output.rglob("*.png") if "/replay/" not in path.as_posix())
    if not pictures:
        return
    with PdfPages(output / "all_plots.pdf") as pdf:
        for path in pictures:
            with Image.open(path) as bitmap:
                fig = plt.figure(figsize=(12, max(3, min(16, 12*bitmap.height/bitmap.width))))
                ax = fig.add_axes((0, .025, 1, .95))
                ax.imshow(bitmap)
                ax.axis("off")
                fig.text(.02, .006, path.relative_to(output).as_posix(), fontsize=6)
                pdf.savefig(fig, dpi=140)
                plt.close(fig)


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
        return dict(status="disabled", reason="--render none / --no-plots")
    mode = args.render
    fallback = ""
    if mode == "auto":
        mode = "video" if shutil.which("ffmpeg") else "preview"
        if mode == "preview":
            fallback = "ffmpeg unavailable; created a PNG preview instead of MP4. "
    task = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    task["render"] = mode
    task_path = args.output / "replay_request.json"
    task_path.write_text(json.dumps(task, indent=2)+"\n")
    environment = dict(os.environ)
    if not environment.get("DISPLAY") and Path("/tmp/.X11-unix/X0").exists():
        environment["DISPLAY"] = ":0"
    command = [sys.executable, str(Path(__file__).resolve()), "--render-worker", str(task_path)]
    if not environment.get("DISPLAY") and shutil.which("xvfb-run"):
        command = ["xvfb-run", "-a", *command]
    print(f"Rendering CAD {mode} for {args.wheel} (progress: {args.output / 'replay.log'})...", flush=True)
    with (args.output / "replay.log").open("w") as log:
        completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=environment)
    if completed.returncode:
        lines = (args.output / "replay.log").read_text(errors="replace").strip().splitlines()
        return dict(status="unavailable" if args.render == "auto" else "failed",
                    reason=f"CAD renderer exited {completed.returncode}: " + (lines[-1] if lines else "see replay.log"))
    return dict(status="complete", reason=fallback + f"{mode}; {args.wheel} caster selection, recorded timing retained")


def run(args):
    from caster_bag_reader import read_recording, digest
    from caster_bag_report import export_report, write_json
    from caster_vicon_comparison import prepare
    options = reference_options(args)
    archive = args.source.is_dir() and (args.source/"encoder_signals.csv").is_file() and (args.source/"summary.json").is_file()
    recording = None if archive else read_recording(args.source, clock_mapper=options["mapper"])
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = dict(version=1, source=str(args.source), stages={}, status="running",
                    arguments={key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
                    implementation_sha256={name: digest(Path(__file__).with_name(name)) for name in (
                        "caster_visualization.py", "caster_bag_reader.py", "caster_bag_report.py",
                        "caster_packet_analysis.py", "caster_vicon_comparison.py", "caster_vicon_trajectory.py")},
                    configuration_sha256={name: digest(getattr(args, name)) for name in (
                        "comparison_config", "left_mapping_config", "right_mapping_config", "reference_csv", "clock_map")
                        if getattr(args, name)})
    if recording:
        print(f"Read {sum(len(t) for t in recording.times.values()):,} messages; generating telemetry plots and CSVs...", flush=True)
        report = export_report(recording, args.output/"analysis", plots=not args.no_plots, **options)
        manifest["stages"]["bag_analysis"] = dict(status="complete", reason=f"{len(report['topics'])} topics inventoried; {len(report['plots'])} overview/topic plots")
        comparison_source = args.output/"analysis"
        if report["decode_issues"]:
            manifest["stages"]["decode_quality"] = dict(status="partial", reason="Some messages could not be decoded; details in analysis/summary.json")
    else:
        comparison_source = args.source
        manifest["stages"]["bag_analysis"] = dict(status="existing_exports", reason="Using archived numeric exports; original bag was not supplied")
    try:
        required = {"encoder_signals.csv": "caster encoder history",
                    "HAMR_base_pose.csv": "base pose", "Left_Caster_pose.csv": "left fork pose",
                    "Right_Caster_pose.csv": "right fork pose"}
        missing = [label for name, label in required.items() if not (comparison_source/name).is_file()]
        if missing:
            raise ValueError("Not recorded for fork comparison: " + ", ".join(missing))
        comparison = prepare(comparison_source, args.output/"comparison", args.comparison_config, not args.no_plots)
        counts = comparison["trajectory_estimation"]["wheels"]
        manifest["stages"]["fork_comparison"] = dict(status="complete", reason="; ".join(
            f"{wheel}: {value['estimated_rows']} uncalibrated marker-motion estimates" for wheel, value in counts.items()))
    except (ValueError, OSError, KeyError, TypeError) as error:
        manifest["stages"]["fork_comparison"] = dict(status="unavailable", reason=str(error))
    write_json(args.output/"report.json", manifest)
    if recording:
        manifest["stages"]["cad_replay"] = run_renderer(args)
    else:
        manifest["stages"]["cad_replay"] = dict(status="unavailable", reason="Archived CSVs have no raw packet/device identity for CAD replay")
    if not args.no_plots:
        print("Building all_plots.pdf and index.html...", flush=True)
        assemble_pdf(args.output)
    manifest["status"] = "partial" if any(stage["status"] in ("failed", "partial", "unavailable") for stage in manifest["stages"].values()) else "complete"
    write_json(args.output/"report.json", manifest)
    write_index(args.output, manifest)
    for stage, value in manifest["stages"].items():
        print(f"{stage}: {value['status']} — {value.get('reason', '')}", flush=True)
    print(f"Open {args.output / 'index.html'}", flush=True)
    return 1 if manifest["stages"]["cad_replay"]["status"] == "failed" else 0


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
