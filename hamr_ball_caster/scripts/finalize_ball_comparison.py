#!/usr/bin/env python3
"""Label and fully decode completed ball comparison videos without altering analysis."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

VIDEO_NAMES = ("overview", "overview_annotated", "caster_closeup")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def default_annotator():
    source = Path(__file__).resolve().parents[2] / "hamr_swivel_caster/scripts/annotate_overview.py"
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_prefix
    return Path(get_package_prefix("hamr_swivel_caster")) / "lib/hamr_swivel_caster/annotate_overview.py"


def execute(command, run):
    completed = subprocess.run(command, capture_output=True, text=True)
    with (run / "video_validation_commands.jsonl").open("a") as log:
        log.write(json.dumps({"utc": datetime.now(timezone.utc).isoformat(), "command": command,
                              "exit_code": completed.returncode, "stderr": completed.stderr}) + "\n")
    if completed.returncode or completed.stderr.strip():
        raise RuntimeError(f"Command failed or reported an error: {command[0]}: {completed.stderr}")
    return completed.stdout


def validate_stream(metadata):
    stream = metadata["streams"][0]
    if (stream["width"], stream["height"], stream["r_frame_rate"]) != (1280, 720, "25/1"):
        raise ValueError("Expected 1280x720 at25fps")
    duration = float(metadata["format"]["duration"])
    if duration <= 0 or abs(int(stream["nb_frames"]) - 25 * duration) > 1.01:
        raise ValueError("Frame count disagrees with playback duration")
    return duration


def finalize_run(run, annotator):
    summary = json.loads((run / "run_summary.json").read_text())
    if not summary.get("recorded"):
        raise ValueError("Run has no requested recording")
    raw_hashes = {name: sha256(run / (name + ".mp4")) for name in ("overview", "caster_closeup")}
    if not (run / "overview_annotated.mp4").exists():
        execute([sys.executable, str(annotator), str(run)], run)
    videos = {}
    durations = {}
    for name in VIDEO_NAMES:
        path = run / (name + ".mp4")
        if not path.is_file() or path.stat().st_size <= 0:
            raise ValueError(f"Missing or empty recording: {path}")
        execute(["ffmpeg", "-v", "error", "-nostdin", "-threads", "2", "-i", str(path),
                 "-f", "null", "-"], run)
        metadata = json.loads(execute(["ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries", "stream=width,height,codec_name,r_frame_rate,nb_frames:format=duration,size",
            "-of", "json", str(path)], run))
        durations[name] = validate_stream(metadata)
        videos[name] = {"full_decode_passed": True, "sha256": sha256(path), **metadata}
        if name != "overview_annotated":
            recording = json.loads((run / (name + "_metadata.json")).read_text())
            stamps = recording["encoded_source_timestamps_s"]
            gaps = [b-a for a, b in zip(stamps, stamps[1:])]
            if not stamps or any(gap <= 0 for gap in gaps):
                raise ValueError(f"Invalid source timestamps: {name}")
            if abs(durations[name] - recording["expected_video_duration_s"]) > .041:
                raise ValueError(f"Playback differs from simulation timestamp duration: {name}")
            videos[name]["source_timing"] = {
                "passed": summary["videos"][name]["passed"],
                "max_source_image_gap_s": max(gaps, default=0.),
                "frames_queue_dropped": recording["frames_queue_dropped"],
                "frames_duplicated": recording["frames_duplicated"],
                "first_sim_time_s": recording["first_sim_time_s"],
                "last_sim_time_s": recording["last_sim_time_s"],
                "motion_quality": summary["videos"][name]["motion_quality"]}
            if videos[name]["sha256"] != raw_hashes[name]:
                raise ValueError(f"Source recording changed during annotation: {name}")
    if abs(durations["overview"] - durations["overview_annotated"]) > .041:
        raise ValueError("Annotation changed video playback duration")
    write_json(run / "video_validation.json", videos)
    return videos


def aggregate_audit(suite, matrix):
    cases = []
    missing = []
    failures = []
    source_timing = []
    video_count = 0
    behavior_passes = 0
    completed_count = 0
    for item in matrix:
        run = suite / "ball" / item["name"]
        validation_path = run / "video_validation.json"
        if not validation_path.is_file():
            missing.append(item["name"])
            continue
        validation = json.loads(validation_path.read_text())
        summary = json.loads((run / "run_summary.json").read_text())
        trajectory = json.loads((run / "trajectory.json").read_text())
        behavior_passes += summary["passed"] is True
        completed_count += trajectory["completed"] is True
        if set(validation) != set(VIDEO_NAMES):
            failures.append(f"{item['name']}: missing one or more required video validations")
        for name in VIDEO_NAMES:
            video = validation.get(name, {})
            if not video.get("full_decode_passed") or not (run / (name + ".mp4")).is_file():
                failures.append(f"{item['name']}/{name}: failed decode or missing file")
                continue
            validate_stream(video)
            video_count += 1
            if name != "overview_annotated":
                timing = video["source_timing"]
                source_timing.append({"case": item["name"], "camera": name, **timing})
                if not timing["passed"] or not timing["motion_quality"]["passed"]:
                    failures.append(f"{item['name']}/{name}: source camera coverage/gap check failed")
        cases.append({"name": item["name"], "validation_file": str(validation_path.relative_to(suite)),
                      "behavior_checks_passed": summary["passed"], "trajectory_completed": trajectory["completed"],
                      "videos": validation})
    complete = not missing and len(cases) == len(matrix)
    if complete:
        for case in cases:
            run = suite / "ball" / case["name"]
            for name, video in case["videos"].items():
                if sha256(run / (name + ".mp4")) != video["sha256"]:
                    failures.append(f"{case['name']}/{name}: video changed after full decode validation")
    result = {"generated_utc": datetime.now(timezone.utc).isoformat(),
              "passed": complete and not failures, "complete": complete,
              "planned_cases": len(matrix), "validated_cases": len(cases),
              "source_camera_video_count": len(source_timing),
              "annotated_overview_count": sum("overview_annotated" in case["videos"] for case in cases),
              "fully_decoded_video_count": video_count,
              "behavior_checks_passed_count": behavior_passes,
              "trajectory_completed_count": completed_count,
              "missing_cases": missing, "failures": failures,
              "worst_source_image_gap_s": max((t["max_source_image_gap_s"] for t in source_timing), default=None),
              "worst_motion_source_image_gap_s": max((t["motion_quality"]["max_motion_image_gap_s"] for t in source_timing), default=None),
              "source_queue_drops": sum(t["frames_queue_dropped"] for t in source_timing),
              "duplicated_frames": sum(t["frames_duplicated"] for t in source_timing),
              "raw_recordings_retained": len(source_timing) == 2*len(cases),
              "playback_validation": "1280x720 at25fps; frame counts and source simulation timestamp durations agree; overview annotation retains duration",
              "cases": cases}
    write_json(suite / "ball_video_audit.json", result)
    return result



def report_input_signature(suite):
    paths = [*suite.glob("pairs/*/comparison.json"),
             *suite.glob("ball/*/run_summary.json"),
             *suite.glob("ball/*/video_validation.json")]
    key_findings = suite / "KEY_FINDINGS.md"
    if key_findings.is_file():
        paths.append(key_findings)
    paths += [Path(__file__).with_name(name) for name in
              ("summarize_caster_comparison.py", "build_caster_comparison.py")]
    return tuple(sorted((str(path), path.stat().st_mtime_ns,
                         path.stat().st_size) for path in paths))


def refresh_reports(suite):
    for name in ("summarize_caster_comparison.py", "build_caster_comparison.py"):
        execute([sys.executable, str(Path(__file__).with_name(name)), str(suite)], suite)
    print("Refreshed comparison findings and page.", flush=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", type=Path)
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--poll-seconds", type=float, default=45.)
    parser.add_argument("--annotator", type=Path)
    parser.add_argument("--refresh-reports", action="store_true",
                        help="Regenerate comparison summary and page when paired analyses or videos change")
    args = parser.parse_args()
    if not 1 <= args.poll_seconds <= 60:
        parser.error("--poll-seconds must be between1 and60")
    suite = args.suite.expanduser().resolve()
    matrix = json.loads((suite / "comparison_matrix.json").read_text())
    annotator = args.annotator or default_annotator()
    last_report_signature = None
    while True:
        for item in matrix:
            run = suite / "ball" / item["name"]
            if (run / "run_summary.json").is_file() and not (run / "video_validation.json").exists():
                finalize_run(run, annotator)
                print(f"Annotated and fully decoded: {item['name']}", flush=True)
        audit = aggregate_audit(suite, matrix)
        print(f"Ball media: {audit['validated_cases']}/{audit['planned_cases']} cases; "
              f"{audit['fully_decoded_video_count']} MP4s fully decoded", flush=True)
        report_ready = True
        if args.refresh_reports:
            signature = report_input_signature(suite)
            if signature != last_report_signature:
                try:
                    refresh_reports(suite)
                    last_report_signature = signature
                except RuntimeError as exc:
                    print(f"Report refresh will retry after the next polling interval: {exc}", flush=True)
            report_ready = (last_report_signature == signature and
                            len(list(suite.glob("pairs/*/comparison.json"))) == len(matrix))
        if not args.watch or (audit["complete"] and report_ready):
            return 0 if audit["passed"] else 1
        time.sleep(args.poll_seconds)


if __name__ == "__main__":
    raise SystemExit(main())
